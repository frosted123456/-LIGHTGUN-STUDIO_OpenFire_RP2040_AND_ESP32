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

// Extended data format. OFF by default -- the sensor is read exactly as
// before until '~cam=ext:1' asks otherwise.
//
// The read LENGTH must match the sensor's format register: a basic-length read
// of an extended frame still returns bytes and still reports success, it just
// decodes to nonsense. So the format switch belongs to whoever owns the camera
// poll: it reads wiicam_aim_ext(), and re-applies whenever the epoch changes.
// Read the STATE word, not the two halves: it carries the wanted format in
// bit 0 and the epoch above it, in one value. The camera poll and the command
// parser run on different cores, and reading the flag and the epoch separately
// lets a poll pair a new epoch with the old flag -- it writes the old format,
// latches the new epoch, and then never corrects itself.
int  wiicam_aim_ext_state(void);
int  wiicam_aim_ext(void);            // convenience: bit 0 of the state
int  wiicam_aim_ext_epoch(void);      // convenience: the epoch half
void wiicam_aim_format_dirty(void);   // camera was rebuilt: re-apply the format

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
