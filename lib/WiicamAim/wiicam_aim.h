// wiicam_aim.h -- RP2040 + wiicam (SEN0158) front end for the aim pipeline.
// Masks the driver's seen flags (unseen slots retain stale coordinates),
// normalises into the pipeline's 240x176 space, applies the stored lens
// correction, runs the quad resolver, applies latency lead, and emits the
// same Q/T telemetry the desktop tools use.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Native wiicam report space.
#define WIICAM_W 1024.0f
#define WIICAM_H 768.0f
// The pipeline's native space (what the resolver gates and tools assume).
#define WIICAM_NORM_W 240.0f
#define WIICAM_NORM_H 176.0f

void wiicam_aim_begin(void);

// One sensor poll. px/py are the driver's positionX/Y arrays (native units),
// seen is the driver's seenFlags. Returns true and writes sx,sy (normalised
// screen 0..1) when the calibrated pipeline produced a position; false means
// fall through to stock OpenFIRE handling.
bool wiicam_aim_process(const int* px, const int* py, unsigned seen,
                        uint64_t now_us, float* sx, float* sy);

// Same, plus the driver's per-blob sizes (extended data format only; pass 0
// when they are not available). Sizes are the one hardware fact that separates
// a compact LED from a window: the size GATE below drops a blob outside its
// window before the resolver ever sees it, which turns "an impostor took a
// slot and warped the quad" into "three real corners and a reconstruction".
bool wiicam_aim_process_sz(const int* px, const int* py, const int* sizes,
                           unsigned seen, uint64_t now_us,
                           float* sx, float* sy);

// Report format. Basic by default -- the sensor is read exactly as it was
// before this code existed until '~cam=fmt:' asks otherwise.
//   0 basic     x,y only
//   1 extended  + a 4-bit size per blob
//   2 full      + the blob's bounding box and an 8-bit intensity
//
// The read LENGTH must match the sensor's format register: a basic-length read
// of an extended frame still returns bytes and still reports success, it just
// decodes to nonsense. So the format switch belongs to whoever owns the camera
// poll: it reads wiicam_aim_fmt(), and re-applies whenever the epoch changes.
// Read the STATE word, not the two halves: it carries the wanted format in
// bits 0-1 and the epoch above it, in one value. The camera poll and the
// command parser run on different cores, and reading the format and the epoch
// separately lets a poll pair a new epoch with the old format -- it writes the
// old format, latches the new epoch, and then never corrects itself.
#define WIICAM_FMT_BASIC 0
#define WIICAM_FMT_EXT   1
#define WIICAM_FMT_FULL  2
int  wiicam_aim_fmt_state(void);
int  wiicam_aim_fmt(void);            // convenience: bits 0-1 of the state
int  wiicam_aim_fmt_epoch(void);      // convenience: the epoch half
void wiicam_aim_format_dirty(void);   // camera was rebuilt: re-apply the format

// ---- full mode -------------------------------------------------------------
// The vendored driver declares this format and cannot read it: its receive
// union is 13 bytes and a full report is 37, there is no rawFull[4], and the
// enumerator is commented out. Rather than patch a library the working build
// depends on, the read lives here and the hook owns nothing but the bus
// transaction: write 0x36, then read len bytes. Non-zero means the bytes
// arrived. The first three bytes of each object are laid out exactly as in
// extended, so x, y and size decode with the same arithmetic.
void wiicam_set_fullread_hook(int (*fn)(unsigned char* buf, int len));
#define WIICAM_FULL_LEN 37            // 1 header + 4 objects x 9 bytes

// One full-mode poll. Fills px/py/sizes/seen exactly as the driver's
// extendedAtomic would, and keeps each blob's box and intensity for the
// report. Returns 1 when a frame was read, 0 on a bus error or no hook.
int  wiicam_aim_full_poll(int* px, int* py, int* sizes, unsigned* seen);

// The byte written to the mode register (0x33) to select full mode.
//
// Settable because we are honestly not certain of it. The driver's own WORKING
// constants for the other two formats are the doubled nibble -- 0x11 basic,
// 0x33 extended -- so 0x55 is the value consistent with what this sensor
// already accepts every time it is powered on. Wiibrew documents the modes as
// 1 / 3 / 5, under which 0x05 works too because only the low nibble is read.
// 0x55 is right under both readings and 0x05 only under one, so it is the
// default; '~cam=fullreg:5' tries the other without a reflash. Not persisted.
//
// It rides in the state word above, for the same reason the format does: the
// camera poll reads it when it acts on an epoch, and a plain variable stored
// beside a volatile one carries no ordering guarantee between the two writes.
// Kept outside, the one knob that exists to rescue a broken full mode could
// be latched a version late -- the poll re-initialising with the OLD byte
// while cam? reported the new one, and nothing ever correcting it.
int  wiicam_aim_fullreg(void);

// Give up on a format the sensor will not accept, and go back to one that
// works. The camera poll calls this when the format register refuses the
// write: an I2C master clocks out every byte it asks for whether or not the
// sensor is producing them, so a 37-byte read against a sensor still in
// extended returns 37 bytes, matches on the retry (the trailing registers are
// static), unpacks four plausible coordinates and reports SUCCESS. There is no
// error anywhere for the user to see -- just a cursor that is wrong until the
// next reboot. Falling back is the only outcome that is both safe and visible:
// the tools watch fmt and will show it drop back on its own.
void wiicam_aim_fmt_fallback(int fmt);

// The sensor's own blob-size thresholds, registers 0x06 (MAXSIZE) and 0x1B
// (MINSIZE). They gate inside the sensor, BEFORE it allocates its four object
// slots, which makes them the only settings that can stop a stray light source
// from costing us a corner instead of merely being noticed after it has.
//
// The write costs tens of milliseconds of settling, so it does not belong in
// the camera poll: the hook is called from wiicam_aim_hw_tick() on whichever
// core runs the serial pump. Both default to "leave the register alone".
// The hook must return non-zero ONLY when the byte really reached the sensor.
// A hook that cannot write yet (no camera, bus not free) returns 0 and the
// request stays pending -- otherwise it is consumed and lost while cam? goes on
// reporting a value the sensor does not hold.
void wiicam_set_blobreg_hook(int (*fn)(int reg, int val));
void wiicam_aim_hw_tick(void);        // cheap; writes only when something changed
// Mark the thresholds for rewriting without disturbing the data format. The
// firmware calls this after anything that rewrites register 0x06 behind our
// back -- a sensitivity change from the pause menu, or a profile switch.
void wiicam_aim_hw_dirty(void);

// Camera bus ownership. Every loop that polls the sensor must skip its poll
// while wiicam_aim_cam_held() is true and call wiicam_aim_cam_ack() instead --
// including OpenFIRE's own calibration and verification loops, not just the
// main run loop. The acknowledgement is what makes a configuration write safe:
// without it the writer is guessing at a drain time it cannot know.
void wiicam_aim_cam_hold(int on);
int  wiicam_aim_cam_held(void);
void wiicam_aim_cam_ack(void);
int  wiicam_aim_cam_acked(void);

// The '~cam...' command subset for this board: cam? / camsave / camreset /
// cam=res:..,dash:..,dashhz:..,lead:..,lens:..,lk1u:..,lk2u:..,lfpx:..,
// lfeq:..,sens:..  Returns true if the line was handled.
bool wiicam_cam_command(const char* line);

// Telemetry (Q/T lines) and replies, same split as the ESP32 build: telemetry
// may be dropped when the host is not draining, replies may block briefly.
void wiicam_set_line_sink(void (*fn)(const char*));
void wiicam_set_reply_sink(void (*fn)(const char*));
void wiicam_set_diag_hook(int (*fn)(void));   // "~camdiag" runs it

// Sensitivity is OpenFIRE's own persisted setting; the patch wires these to
// FW_Common so '~cam=sens:n' goes through the same path as their UI.
void wiicam_set_sens_hooks(void (*set_fn)(int), int (*get_fn)(void),
                           void (*save_fn)(void));

#ifdef __cplusplus
}
#endif
