// wiicam_aim.cpp -- see header. All processing is in 240x176 space.
#include "wiicam_aim.h"
#include "wiicam_learn.h"
#include "quad_resolver.h"
#include "aim_runtime.h"
#include "recoil_fx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// Per-axis normalisation from 1024x768 to 240x176. The small aspect
// difference is a fixed linear map the calibration absorbs.
#define SX (WIICAM_NORM_W / WIICAM_W)
#define SY (WIICAM_NORM_H / WIICAM_H)

// Same lead ceilings as the ESP32 capture layer, in the same 240-space units.
#define WIICAM_LEAD_MS_MAX 50.0f
#define WIICAM_LEAD_PX_MAX 40.0f

// Microsecond clock for the recoil engine's timeline; the shim provides it on
// both boards and the host test never calls through here.
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_RP2040)
extern "C" int64_t esp_timer_get_time(void);
static uint64_t fx_now(void) { return (uint64_t)esp_timer_get_time(); }
#else
static uint64_t fx_now(void) { return 0; }
#endif

static void (*s_line)(const char*)  = 0;
static void (*s_reply)(const char*) = 0;
static void (*s_sens_set)(int) = 0;
static int  (*s_sens_get)(void) = 0;
static void (*s_sens_save)(void) = 0;
static int  (*s_diag)(void) = 0;
static void (*s_preflash)(void) = 0;   // run before the loop's own flash write

void wiicam_set_diag_hook(int (*fn)(void)) { s_diag = fn; }
void wiicam_set_preflash_hook(void (*fn)(void)) { s_preflash = fn; }

void wiicam_set_line_sink(void (*fn)(const char*))  { s_line = fn; }
void wiicam_set_reply_sink(void (*fn)(const char*)) { s_reply = fn; }
void wiicam_set_sens_hooks(void (*set_fn)(int), int (*get_fn)(void),
                           void (*save_fn)(void))
{ s_sens_set = set_fn; s_sens_get = get_fn; s_sens_save = save_fn; }

// printf-checked: a format that gains a conversion without an argument
// is a compile error, not a garbage line on the wire.
__attribute__((format(printf, 1, 2)))
static void reply(const char* fmt, ...)
{
    // Sized for the LONGEST reply at its widest: '~camblob?' line 1 is a
    // ~109-byte prefix plus fifteen unsigned-long counters at full width, about
    // 350 bytes. A shorter buffer truncates it SILENTLY, taking the br4..br0
    // buckets every percentage in the tools is drawn from.
    char b[512];
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (s_reply) s_reply(b); else fputs(b, stdout);
}

// ---- runtime state --------------------------------------------------------
// The wiicam's resolver tuning: Batch A's vetoes on, which the OV path leaves
// off. See /work/schema/pipeline_schematic.html, Level 5 and Order of work 1-3.
static QuadConfig s_quad_cfg = []{
    QuadConfig c = quad_default_config();
    c.veto_seed    = true;
    c.partial_lock = true;
    // A 60 degree tilt foreshortens a rectangle to about 2:1; leave room.
    c.cold_aniso_max = 2.6f;
    return c;
}();
// quad_reset() must run on the camera core: the serial core raises this and
// wiicam_aim_process_sz() performs it (S2).
static volatile bool s_quad_reset_pending = false;

static uint8_t  s_res  = 2;        // 0 = raw (lens sweeps), 2 = resolver
static uint8_t  s_dash = 0;        // 0 = quiet, 2 = Q stream on
static uint32_t s_dash_min_dt_us = 0;
static uint64_t s_dash_last_us = 0;
static float    s_lead_ms = 0.0f;
// lens correction, in 240-space (fitted from Q lines, which are 240-space)
static uint8_t  s_lens = 0;
static float    s_lk1 = 0.0f, s_lk2 = 0.0f;
// The wiicam reports X mirrored relative to the pipeline's convention.
// Un-mirrored by default; 'mirx' / 'miry' keys override for other modules.
static uint8_t  s_mirx = 1, s_miry = 0;
static float    s_lfpx = 184.7f, s_lfeq = 90.0f;
static float    s_lcx = 0.0f, s_lcy = 0.0f;   // distortion-centre offset, px
static uint64_t s_prev_us = 0;

// Duplicate-report cache. The caller polls at loop rate but the wiicam only
// updates internally at its own frame rate, so most polls return the exact
// bytes of the previous one. Reprocessing those would (a) hand the resolver,
// lead and One Euro filter a dt of the LOOP interval instead of the camera
// interval, decaying the velocity estimate between real frames, and (b) flood
// the Q stream at loop rate during an uncapped lens sweep.
static int      s_cache_px[4], s_cache_py[4], s_cache_sz[4];
static unsigned s_cache_seen = 0xFFFFFFFFu;    // impossible: never matches first
static bool     s_cache_ret = false;
static float    s_cache_sx = 0.0f, s_cache_sy = 0.0f;

// ---- ambient-light rejection ----------------------------------------------
// The wiicam finds blobs in HARDWARE and reports four slots, so a bright
// window does not add a fifth point: it TAKES one, and an LED goes missing.
// Nothing downstream can recover a point the sensor never sent. What the
// sensor does give us, in extended format, is each blob's size -- and a window
// is a different size from an LED. Dropping an out-of-window blob leaves the
// resolver three real corners to reconstruct from instead of four points one
// of which is a lie.
//
// All of it is inert by default: ext off means no sizes, and no sizes means no
// gate, whatever the limits say.
//
// The wanted format and its epoch live in ONE word, written with one store:
// bits 0-1 are the format, the rest is the epoch. The camera poll runs on the
// other core from the command parser, and reading a format and an epoch as two
// separate values lets a poll pair a NEW epoch with the OLD format -- it would
// then write the old format, latch the new epoch, and never correct itself.
// One word cannot tear that way. Two bits rather than one because there are
// three formats now; widening the field and leaving a reader on the old shift
// is exactly the desync this word exists to prevent, so every reader goes
// through the accessors below.
static volatile int s_ext_state = 0;        // (epoch << 3) | (fullreg05 << 2) | fmt
static uint8_t  s_bmin = 0, s_bmax = 15;    // accepted blob size window
// Relative gate tolerance, in SIZE STEPS away from the consensus; 0 = off.
// Steps rather than a percentage because the reported size is a 4-bit number:
// a percentage of a median of 3 floored to the same single step for every
// setting from 1 to 66, so the knob was binary and most aggressive at its
// lowest non-zero value. See size_consensus_drop().
static uint8_t  s_rtol = 0;

// ---- the SHAPE gate -------------------------------------------------------
// The 4-bit size is nearly useless on this rig -- 52,624 confirmed LED blobs
// came in at size 1 (95.6%) or 2 (4.4%), with no response to a 1.8x distance
// change. Full mode's bounding box and pixel count are a real measurement, and
// these two knobs gate on them.
//
// A ONE-CLASS envelope, not a discriminator. Both bounds come from what an LED
// has actually looked like, with a step of margin, so the gate can only ever
// refuse something OUTSIDE everything we have ever measured. That is the shape
// of gate that cannot invent a false negative out of a guess: to lose a real
// LED it would have to be unlike all 52,624 of them.
//
//   pxmax  the blob's pixel count. 0 = off, and 0 is what ships: the numbers
//          this was first set from were measured on ONE bar, and a bar with
//          five LEDs per cluster produces blobs several times larger than one
//          with two. A ceiling that fits the first blinds the second, and the
//          owner of the second has no way to know that is what happened. Any
//          non-zero value has to come from THIS rig, which is what camfit is.
//   armax  longest side / shortest side, in EIGHTHS (8 = 1:1, 16 = 2:1).
//          DEPRECATED, 0 = off. Measured at sensitivity 1 it looked like the
//          one feature that survives changing the bar -- an LED is a point
//          source, so 63% of blobs came out exactly square. Sensitivity 2 is
//          the default now, and its horizontal smear puts the median LED near
//          4:1. The feature did not survive the change it was chosen for.
//          Still loaded and still applied so a stored setting keeps its
//          meaning; never suggested and never set by camfit.
//
// Both need full mode; without a box there is nothing to judge and the gate
// stands down rather than guessing.
static uint8_t  s_pxmax = 0;
static uint8_t  s_armax = 0;
// Box HEIGHT, and it is the one that works. Measured over 11,996 confirmed
// blobs in daylight at sensitivity 2: 99.73% of LEDs come in at height 7 or
// less, and the rest jump straight to 31+. Every stray in that capture ran 15
// to 56. A cut at 10 caught 84% of them and cost ZERO LEDs -- not a trade-off,
// a gap.
//
// Height rather than width or area because at sensitivity 2 the sensor smears
// horizontally: the same LEDs went from a 2x2 box to 12x3 when the gain went
// up, width x5.5 and height x1.5. Width stops measuring the source and starts
// measuring the gain, and area and aspect inherit that. Height is the axis the
// smear does not touch, so it is where "an extended source is larger than a
// point source" is still measurable -- which is the discriminator we actually
// want, on the only axis that still carries it.
static uint8_t  s_bhmax = 0;

// The rig's own measured envelope: the LARGER of what this capture has seen
// and what an earlier one recorded in flash. Both are real measurements of
// this rig; the stored one is just older.
//
// MAX of the two, not "live first". Preferring the live edge was a bug with a
// nasty shape: a thin capture -- a dim room, a few blobs at long range -- gives
// a SMALL maximum, which LOWERS the floor, which lets through a ceiling that
// blinds the gun at play distance. And it does it while a real, larger, 500-blob
// measurement sits in flash unused. Note the asymmetry that made it easy to
// miss: camfit refuses to DERIVE from fewer than 500 blobs, but the refusal
// floor believed a single one.
//
// Taking the max is the conservative direction on both counts. An older
// measurement of this rig is still a measurement; if the bar really did get
// smaller, camreset clears the stored pair and the next capture sets it fresh.
//
// Returns -1 for "this rig has never been measured", which is the only case
// where a number is taken on trust.
static int rig_led_max_h(void)
{
    wl_envelope_t e;
    wl_envelope(&e);
    int best = e.led_max_h;
    int led = -1, stray = -1, px = -1;
    if (aim_fit_load(&led, &stray, &px) && led > best) best = led;
    return best;
}
// The floor under 'pxmax', with one honest caveat. pxmax gates on the report's
// pixel-count byte; this bound is measured from the bounding-box AREA, because
// the learning sink has no histogram of the raw byte. Area is an upper bound on
// pixel count -- a blob cannot fill more pixels than its own box -- so using it
// errs by refusing a slightly wider band than strictly necessary, which is the
// safe direction.
//
// The clamp is the part that is NOT safe, and it is returned as a distinct
// answer rather than as a number. WL_AREA saturates at bin 31 while pxmax
// accepts 0..63, so on a rig whose LED blobs exceed 31 px -- a 12x3 smear does
// -- the measurement pins at 31 and every value above it looks unmeasured. A
// pxmax of 32 would then be accepted while cutting real LEDs: the same shape as
// the live-first defect above, and the reason this returns -2 for "measured,
// but the measurement ran off the end of its scale" instead of quietly handing
// back the clamp as if it were the maximum.
#define RIG_PX_UNBOUNDED (-2)
static int rig_led_max_px(void)
{
    wl_envelope_t e;
    wl_envelope(&e);
    int best = e.led_max_px;
    int led = -1, stray = -1, px = -1;
    if (aim_fit_load(&led, &stray, &px) && px > best) best = px;
    if (best >= WL_BINS - 1) return RIG_PX_UNBOUNDED;
    return best;
}
static uint32_t s_bsrej = 0;   // blobs the shape gate wanted to reject

// ---- the sensor's OWN size thresholds -------------------------------------
// Registers 0x06 (MAXSIZE) and 0x1B (MINSIZE) gate blobs inside the sensor,
// BEFORE it allocates its four slots. That makes them the only settings that
// can stop a stray light source from costing us a corner rather than merely
// being noticed after it already has.
//
// They are asymmetric in risk, and the asymmetry decides which one to reach
// for. Our LEDs are sub-pixel point sources, so their blobs are already near
// the SMALL end: a ceiling can never reject them, while a floor set to catch
// faint junk could easily reject a real LED at the far end of the play range.
// MAXSIZE is the useful knob; MINSIZE is here mostly so it stops being an
// unknown -- the driver has never written it on any gun, so it sits at a
// power-on default nobody has ever read.
//
// -1 means "leave it alone", and is the default: a build carrying this code
// configures the sensor exactly as the build before it until asked otherwise.
#define WIICAM_HW_LEAVE   (-1)   // do not touch the register
#define WIICAM_HW_RESTORE (-2)   // put it back to a safe value, then leave it
// volatile: written by the command parser on one core, read by the tick on
// the other.
static volatile int16_t s_hwmax = WIICAM_HW_LEAVE;   // register 0x06
static volatile int16_t s_hwmin = WIICAM_HW_LEAVE;   // register 0x1B
// Set whenever the values must be (re)written to the sensor. Writing them
// costs tens of milliseconds, so it happens on the serial-pump core rather
// than in the camera poll -- and it has to happen again after any CameraSet,
// because begin() rewrites 0x06 from the sensitivity preset and would silently
// undo ours.
static volatile bool s_hw_dirty = false;
static volatile bool s_hw_busy  = false;
// Returns non-zero only when the byte really reached the sensor. A hook that
// cannot write right now (no camera, bus not free) says so and the request
// stays pending, instead of being consumed and quietly lost while cam? goes on
// reporting a value the sensor does not hold.
static int (*s_blobreg)(int reg, int val) = 0;

void wiicam_set_blobreg_hook(int (*fn)(int reg, int val)) { s_blobreg = fn; }

// ---- the hwmax loop -------------------------------------------------------
// MAXSIZE (0x06) acts before the sensor hands out its four slots, so it is the
// only gate that can keep the sun from displacing an LED. The resolver says
// each frame which side of it we are on; the loop moves the register on that
// verdict and never needs its units. Shape doc Level 5, guardrails K1-K6.
#define LOOP_DWELL      50    // frames in one dwell (about 1/4 s at 200 Hz)
// Wall time, not frames: a parked gun's duplicate frames never reach here (K2).
#define LOOP_RECENT_US 500000u   // 0.5 s since the last lock still counts
#define LOOP_SETTLE      8    // frames ignored after a write, for it to land
#define LOOP_SETTLED     4    // consecutive HOLD dwells before the value is kept
#define LOOP_RAISE_N     5    // consecutive V_CUT frames that raise immediately (K3)
#define LOOP_HI_UNKNOWN 256   // no value is known to admit a stray yet
// What each sensitivity preset writes into 0x06; the loop never goes above it.
#define LOOP_PRESET_MAX 255   // sensitivity 2
#define LOOP_PRESET_LOW 144   // sensitivity 0 and 1

enum { V_NONE = 0, V_CLEAN, V_STRAY, V_CUT };
enum { LOOP_HOLD = 0, LOOP_LOWER, LOOP_RAISE, LOOP_NOSAFE, LOOP_OFF };

// Loop state is owned by the camera core. The two flags the pump core also
// reads/writes are volatile; a handler switching the loop off clears s_loop_on
// FIRST so a write in flight is at worst one stale byte cam? still shows.
static volatile bool s_loop_on = true;
static volatile bool s_loop_store_req = false;  // settled: store hwl0 from the pump core
static int  s_loop_preset_seen = LOOP_PRESET_MAX; // preset the current search is about
static int  s_loop_lo    = 0;                 // highest value known to cut an LED
static int  s_loop_hi    = LOOP_HI_UNKNOWN;   // lowest known to admit a stray
static int  s_loop_val   = LOOP_PRESET_MAX;   // what we last asked 0x06 for
static int  s_loop_state = LOOP_HOLD;
static int  s_loop_dwell = 0;                 // frames into the current dwell
static int  s_loop_settle = 0;                // frames left to ignore after a write
static int  s_loop_cut_run = 0;               // consecutive V_CUT frames
static int  s_loop_hold_run = 0;              // consecutive HOLD dwells
static int  s_loop_nclean = 0, s_loop_nstray = 0, s_loop_ncut = 0;
static volatile bool s_loop_saved = false;    // this value is in flash (set by the pump core)
static uint64_t s_loop_last_lock_us = 0;      // last lock, caller's clock (K2)
static bool     s_loop_ever_lock = false;
// Boot value came from flash and no lock has vouched for it yet: a value that
// cuts an LED never locks, so a whole dwell without a lock is the K4 evidence.
static bool     s_loop_from_flash = false;

// The value the sensitivity preset puts in 0x06, and the loop's own ceiling.
static int loop_preset(void)
{
    const int lv = s_sens_get ? s_sens_get() : 2;
    return (lv >= 2) ? LOOP_PRESET_MAX : LOOP_PRESET_LOW;
}

static void loop_new_dwell(void)
{
    s_loop_dwell = 0;
    s_loop_nclean = 0; s_loop_nstray = 0; s_loop_ncut = 0;
    s_loop_cut_run = 0;
}

// Through the existing path: the pump core does the bus write. loop_tick
// waits for it to land, then the settle counter skips the frames still taken
// under the old value.
static void loop_write(int v)
{
    if (v < 1) v = 1;                    // 0 in MAXSIZE blinds the sensor
    const int preset = loop_preset();
    if (v > preset) v = preset;          // never above what the preset writes
    s_loop_val = v;
    s_hwmax = (int16_t)v;
    s_hw_dirty = true;
    s_loop_settle = LOOP_SETTLE;
    s_loop_saved = false;                // a new value is not the saved one
}

// lo cuts an LED, hi admits a stray, nothing in between: a room problem, not
// a threshold problem (K5). Back to the preset, which protects the LEDs.
static void loop_nosafe(void)
{
    loop_write(loop_preset());
    s_loop_state = LOOP_NOSAFE;
    s_loop_hold_run = 0;
    loop_new_dwell();
}

// RAISE, immediate (K3): lo moves up to the value that cut an LED, and we jump
// to the highest value not yet known to admit a stray.
static void loop_raise(void)
{
    const int preset = loop_preset();
    // At the preset MAXSIZE cannot be what cut the LED; recording lo = preset
    // from an off-screen glance would poison the bounds.
    if (s_loop_val >= preset) { loop_new_dwell(); return; }
    s_loop_lo = s_loop_val;
    if (s_loop_hi < LOOP_HI_UNKNOWN && s_loop_hi - 1 <= s_loop_lo) {
        loop_nosafe();
        return;
    }
    int nv = (s_loop_hi < LOOP_HI_UNKNOWN) ? s_loop_hi - 1 : preset;
    if (nv < s_loop_lo + 1) nv = s_loop_lo + 1;
    loop_write(nv);
    s_loop_state = LOOP_RAISE;
    s_loop_hold_run = 0;
    loop_new_dwell();
}

// End of a dwell: LOWER if strays got in and nothing was cut, otherwise HOLD.
static void loop_dwell_end(void)
{
    if (s_loop_state == LOOP_NOSAFE) {
        // Only a clean dwell leaves NOSAFE; the bounds are cleared because the
        // room, not the threshold, is what changed.
        if (s_loop_nclean >= LOOP_DWELL / 2) {
            s_loop_lo = 0; s_loop_hi = LOOP_HI_UNKNOWN;
            s_loop_state = LOOP_HOLD;
            s_loop_hold_run = 0;
            s_loop_saved = false;
        }
        loop_new_dwell();
        return;
    }
    // K4: a saved value that never locked in a whole dwell cuts an LED. Back
    // to the preset and record it as lo -- unless it IS the preset (a lens cap
    // or a gun pointed away must not leave the loop NOSAFE all session).
    if (s_loop_from_flash && !s_loop_ever_lock) {
        s_loop_from_flash = false;
        if (s_loop_val < loop_preset()) {
            s_loop_lo = s_loop_val;
            loop_write(loop_preset());
            s_loop_state = LOOP_RAISE;
            s_loop_hold_run = 0;
        }
        loop_new_dwell();
        return;
    }
    if (s_loop_ncut == 0 && s_loop_nstray >= LOOP_DWELL / 2) {
        if (s_loop_val > s_loop_lo + 1) {
            s_loop_hi = s_loop_val;
            loop_write((s_loop_lo + s_loop_val) / 2);
            s_loop_state = LOOP_LOWER;
            s_loop_hold_run = 0;
            loop_new_dwell();
            return;
        }
        // val == lo+1: the stray gets in one step above the cut, bounds met.
        s_loop_hi = s_loop_val;
        loop_nosafe();
        return;
    }
    s_loop_state = LOOP_HOLD;
    // Only a clean dwell counts toward settling; an indecisive one (nothing in
    // frame) is not evidence, so flash only ever gets a value this gun aimed with.
    if (s_loop_nclean >= LOOP_DWELL / 2 && s_loop_hold_run < LOOP_SETTLED)
        ++s_loop_hold_run;
    // Settled: keep it, never below what an LED has needed (K4). The flash
    // write itself is done by the pump core (wiicam_aim_hw_tick).
    if (s_loop_hold_run >= LOOP_SETTLED && !s_loop_saved
        && s_loop_val > s_loop_lo)
        s_loop_store_req = true;
    loop_new_dwell();
}

// One judged frame: count it, then decide whether to move the register.
static void loop_tick(int verdict)
{
    if (!s_loop_on) return;
    // A write still waiting for the pump core has not landed; the frames after
    // it lands were still taken under the old value. Neither is evidence.
    if (s_hw_dirty) return;
    if (s_loop_settle > 0) { --s_loop_settle; return; }
    if      (verdict == V_CLEAN) ++s_loop_nclean;
    else if (verdict == V_STRAY) ++s_loop_nstray;
    else if (verdict == V_CUT)   ++s_loop_ncut;
    ++s_loop_dwell;
    if (verdict == V_CUT) ++s_loop_cut_run;
    else                  s_loop_cut_run = 0;
    // NOSAFE ignores a cut: it is sitting at the preset, where a missing
    // corner is an off-screen gun and not something MAXSIZE can fix.
    if (s_loop_state != LOOP_NOSAFE && s_loop_cut_run >= LOOP_RAISE_N) {
        loop_raise();
        return;
    }
    if (s_loop_dwell >= LOOP_DWELL) loop_dwell_end();
}

// A new preset, a new search: the register the loop was tuning has just been
// rewritten under it, so every bound it found is about a sensor that no longer
// exists. Also used by camreset and 'loop:1'.
static void loop_reset(int val)
{
    s_loop_lo = 0;
    s_loop_hi = LOOP_HI_UNKNOWN;
    s_loop_val = val;
    s_loop_state = LOOP_HOLD;
    s_loop_hold_run = 0;
    s_loop_saved = false;
    s_loop_from_flash = false;
    s_loop_settle = 0;
    loop_new_dwell();
}

static const char* loop_state_name(void)
{
    switch (s_loop_state) {
        case LOOP_LOWER:  return "LOWER";
        case LOOP_RAISE:  return "RAISE";
        case LOOP_NOSAFE: return "NOSAFE";
        case LOOP_OFF:    return "OFF";
        default:          return "HOLD";
    }
}

// ---- full mode -------------------------------------------------------------
// Kept out of the vendored driver on purpose: its receive union is 13 bytes,
// a full report is 37, and widening a library the working build depends on to
// chase a format we have never seen work is the wrong risk to take. The hook
// does the bus transaction and nothing else; the unpack lives here where the
// host tests can reach it.
static int (*s_fullread)(unsigned char* buf, int len) = 0;
// Per-blob box and intensity from the last full frame. Reported, not gated:
// a discriminator invented before anyone has seen a real number from it is
// the same guess the size window would have been.
//
// TWO arrays, and the difference is the whole bug. The poll fills s_f* by
// HARDWARE SLOT, 0..3 as the sensor numbers them. Everything the report shows
// -- position, size, kept -- is indexed by the COMPACTED position in the seen
// list, because process_sz skips empty slots as it walks. Publish the box
// straight from s_f* and the two disagree the moment any slot but the last is
// empty: blob 0 gets slot 0's box, which belongs to no blob on screen and may
// be left over from an earlier frame entirely. That is precisely the
// ambient-light case this instrumentation exists to measure, so it would have
// been wrong exactly when it mattered. process_sz compacts s_f* into s_b*
// alongside the coordinates, through the same index.
static uint8_t  s_fw[4], s_fh[4], s_fi[4];   // by hardware slot
static uint8_t  s_bw[4], s_bh[4], s_bi[4];   // by compacted report index
// The box ORIGIN as the sensor gave it, unscaled and untransformed. Width and
// height alone cannot answer the one question that decides what the numbers
// mean: are these 7-bit fields in the sensor's native 128x96 array, or in some
// other space? With the origin, the box centre can be compared against the
// reported position, which settles it arithmetically instead of by assertion.
// It is also what a preview needs to draw the box where the blob actually is.
static uint8_t  s_fxm[4], s_fym[4];
static uint8_t  s_bxm[4], s_bym[4];

void wiicam_set_fullread_hook(int (*fn)(unsigned char* buf, int len))
{
    s_fullread = fn;
}
int wiicam_aim_full_poll(int* px, int* py, int* sizes, unsigned* seen)
{
    if (!s_fullread || !px || !py || !sizes || !seen) return 0;
    unsigned char a[WIICAM_FULL_LEN], b[WIICAM_FULL_LEN];
    if (!s_fullread(a, WIICAM_FULL_LEN)) return 0;
    // The same atomic workaround the driver uses for the other two formats:
    // the sensor offers no frame sync, so a report can be read across an
    // update and come back half old. Read again and accept only a match,
    // ignoring the header byte, which is not part of the frame.
    //
    // Two retries and then GIVE UP, because that is what the other two formats
    // do here: GetPosition asks for Retry_2, whose even value makes the driver
    // return Error_DataMismatch rather than use the frame, and the caller
    // drops it. An earlier version of this comment claimed to match Retry_1s
    // and kept the newest frame -- which would have made full mode quietly
    // more permissive than the paths it stands in for, and fed the resolver a
    // torn frame in the one case the check exists to catch.
    int matched = 0;
    for (int i = 0; i < 2 && !matched; ++i) {
        if (!s_fullread(b, WIICAM_FULL_LEN)) return 0;
        if (!memcmp(a + 1, b + 1, WIICAM_FULL_LEN - 1)) matched = 1;
        else memcpy(a, b, WIICAM_FULL_LEN);
    }
    if (!matched) return 0;
    *seen = 0;
    // Cleared for every slot, not just the ones that report: an unseen slot
    // must read as "no box measured", never as last frame's box.
    for (int i = 0; i < 4; ++i) {
        s_fw[i] = 0; s_fh[i] = 0; s_fi[i] = 0; s_fxm[i] = 0; s_fym[i] = 0;
    }
    for (int i = 0; i < 4; ++i) {
        // Byte 0 of the report is a header; each object is 9 bytes, and its
        // first three are laid out exactly as in extended.
        const unsigned char* f = a + 1 + i * 9;
        const int y = (int)f[1] | (((int)(f[2] & 0xC0u)) << 2);
        if (y > 767) continue;              // the driver's "not seen" test
        py[i] = y;
        px[i] = (int)f[0] | (((int)(f[2] & 0x30u)) << 4);
        sizes[i] = (int)(f[2] & 0x0Fu);
        // Box fields are 7-bit, in the sensor's native 128x96 array -- NOT the
        // 1024x768 the positions are reported in. Stored as width and height
        // because that is what a shape test wants and the corners are not.
        const int xmn = f[3] & 0x7F, ymn = f[4] & 0x7F;
        const int xmx = f[5] & 0x7F, ymx = f[6] & 0x7F;
        s_fw[i] = (uint8_t)(xmx > xmn ? xmx - xmn : 0);
        s_fh[i] = (uint8_t)(ymx > ymn ? ymx - ymn : 0);
        s_fxm[i] = (uint8_t)xmn;
        s_fym[i] = (uint8_t)ymn;
        s_fi[i] = f[8];
        *seen |= 1u << i;
    }
    return 1;
}

// ---- camera bus ownership -------------------------------------------------
// Whoever wants the sensor bus for a configuration write TAKES it, and the
// core polling the camera ACKNOWLEDGES that it has seen the request and is
// between transactions. A fixed drain delay cannot establish that: the Wire
// timeout is 100 ms, so on a marginal solder joint -- the gun this whole
// diagnostic exists for -- the poller can still be inside one read long after
// any delay we would be willing to wait, and a second master would then start
// clocking the same bus.
//
// It lives HERE rather than in the sketch because the sketch is not the only
// place that polls: OpenFIRE's own calibration and verification loops do too,
// and every one of them has to honour the same flag.
static volatile int s_cam_hold = 0;
static volatile int s_cam_ack  = 0;

void wiicam_aim_cam_hold(int on)
{
    if (on) s_cam_ack = 0;
    s_cam_hold = on ? 1 : 0;
}
int  wiicam_aim_cam_held(void)  { return s_cam_hold; }
void wiicam_aim_cam_ack(void)   { s_cam_ack = 1; }
int  wiicam_aim_cam_acked(void) { return s_cam_ack; }
// Counters, cumulative since boot. The tools sample them and show the DELTA,
// which is what says "this many frames lost a corner in the last two seconds"
// rather than a number that only grows.
static uint32_t s_bframes = 0;   // frames processed
static uint32_t s_brej    = 0;   // blobs dropped by the absolute size window
static uint32_t s_brrej   = 0;   // blobs dropped for not matching the others
static uint32_t s_bvalve  = 0;   // blobs the floor had to give back: the window
                                 // is set so tight it would blind the gun
static uint32_t s_bdrop   = 0;   // frames the resolver could not turn into a quad
// The two halves of "was that SHAPE-gate rejection right?", judged on position
// alone, and counted on every frame whether or not a capture is running.
// bfar is a blob the shape gate dropped that sat nowhere near any corner -- a
// stray, correctly refused. bnear is one that sat exactly where the missing
// LED should have been, which means the shape gate almost certainly took a
// real corner. Shape gate only, on purpose: the tools subtract both from
// bsrej (which counts shape rejections alone) to get "rejections nobody could
// vouch for", and their advice on a bnear is "bhmax:0 turns it off" -- both
// are only true if these count the same gate bsrej does. bnear climbing is
// the earliest warning that the ceiling is too tight, and it arrives long
// before the symptom does. The size window's own too-tight signal is bvalve.
static uint32_t s_bfar    = 0;
static uint32_t s_bnear   = 0;
// Frames where a blob far larger than the rest was held back from an unlocked
// resolver -- a size outlier in the set, which is not a corner.
static uint32_t s_bseedveto = 0;
// Frames the resolver handed straight back: no model yet, or the set was
// refused -- not a corner count, so they stay out of the buckets below.
static uint32_t s_bcold   = 0;
static uint32_t s_breal[5] = {0, 0, 0, 0, 0};   // frames by corners actually SEEN
// Last frame's blobs, for '~camblob?': pipeline space, size, and whether the
// gate kept it. Evidence first -- a gate set from a guess is worth nothing.
static int      s_bn = 0;
static int16_t  s_bx[4], s_by[4];
static int8_t   s_bsz[4], s_bkeep[4];

int wiicam_aim_fmt_state(void)  { return s_ext_state; }
int wiicam_aim_fmt(void)        { return s_ext_state & 3; }
int wiicam_aim_fmt_epoch(void)  { return s_ext_state >> 3; }
// Bit 2 chooses which byte full mode writes to the format register. It lives
// in the state word for the same reason the format does: it is payload the
// camera poll reads when it acts on an epoch, and an ordinary variable beside
// a volatile one has no ordering guarantee between the two stores. Kept out,
// the one knob that exists to rescue a broken full mode could be latched a
// version late -- the poll would re-init with the OLD byte while cam? cheerily
// reported the new one, and nothing would ever correct it.
int wiicam_aim_fullreg(void)    { return (s_ext_state & 4) ? 0x05 : 0x55; }

// One store, so a reader on the other core sees the format, the register byte
// and the epoch move together or not at all. Unchanged is a no-op: every epoch
// bump costs the camera poll a re-init with the sensor's settling delay, and
// the tools re-send a value on every keypress.
static void fmt_set(int want, int freg05)
{
    if (want < 0) want = 0;
    if (want > WIICAM_FMT_FULL) want = WIICAM_FMT_FULL;
    if (want != WIICAM_FMT_FULL)
        for (int i = 0; i < 4; ++i) {
            s_fw[i] = 0; s_fh[i] = 0; s_fi[i] = 0; s_fxm[i] = 0; s_fym[i] = 0;
        }
    const int cur = s_ext_state;
    const int payload = (freg05 ? 4 : 0) | want;
    if ((cur & 7) == payload) return;
    // The register byte only means anything to a poll that is APPLYING full
    // mode. Changing it in any other format still has to be recorded -- it is
    // what full mode will use when it is next selected -- but it must not bump
    // the epoch, or flipping the compatibility switch while the gun is in
    // basic costs a camera re-init that writes basic again, and the tools'
    // two-value ladder re-sends on every keypress.
    const int bump = (want == WIICAM_FMT_FULL) || ((cur & 3) != want);
    s_ext_state = bump ? ((((cur >> 3) + 1) << 3) | payload)
                       : ((cur & ~7) | payload);
}
// Format only, keeping whichever register byte is selected.
static void ext_set(int want) { fmt_set(want, s_ext_state & 4); }

void wiicam_aim_fmt_fallback(int fmt) { ext_set(fmt); }

void wiicam_aim_format_dirty(void)
{
    // Forced, not conditional: the sensor was re-initialised behind our back,
    // so the format has to be written again even though nothing we hold
    // changed. fmt_set() would correctly decide there was nothing to do.
    const int cur = s_ext_state;
    s_ext_state = (((cur >> 3) + 1) << 3) | (cur & 7);
    // A rebuilt camera has been re-initialised from the sensitivity preset,
    // which writes register 0x06 -- so our MAXSIZE is gone and has to go back.
    s_hw_dirty = true;
}

// Called from the serial-pump core every loop: cheap when there is nothing to
// do, and the right place for the write when there is, because it can afford
// the sensor's settling delays and the camera poll cannot.
void wiicam_aim_hw_tick(void)
{
    // The loop's settled value goes to flash from here, not from the camera
    // poll: a flash write parks both cores, so the recoil outputs are dropped
    // first the way camsave does it.
    if (s_loop_store_req) {
        s_loop_store_req = false;
        if (s_preflash) s_preflash();
        s_loop_saved = aim_hwloop_store(s_loop_val, s_loop_lo, s_loop_hi);
    }
    if (!s_hw_dirty || !s_blobreg) return;
    // Two masters clocking one bus is how a bus locks up; the second caller
    // leaves and the request stays pending.
    if (s_hw_busy) return;
    s_hw_busy = true;
    // Cleared BEFORE the work, so a mark raised while this tick is running --
    // a profile switch on the other core rewriting register 0x06 behind us --
    // survives instead of being wiped by our own success.
    s_hw_dirty = false;
    int done = 1;
    if (s_hwmax == WIICAM_HW_RESTORE) {
        // Back to whatever the sensitivity preset writes into 0x06. That IS
        // the restore: the preset is the only value we know is sane for this
        // sensor, and re-selecting the current level rewrites it.
        //
        // The sentinel is consumed only if that actually happened. Clearing it
        // unconditionally left MAXSIZE at our value with no way to notice --
        // the reset reported success and changed nothing.
        if (s_sens_set && s_sens_get) {
            int lv = s_sens_get();
            if (lv < 0) lv = 0;
            if (lv > 2) lv = 2;          // SetIrSensitivity refuses anything else
            s_sens_set(lv);
            s_hwmax = WIICAM_HW_LEAVE;
        } else {
            done = 0;                    // no way to restore it yet; try later
        }
    } else if (s_hwmax != WIICAM_HW_LEAVE) {
        if (!s_blobreg(0x06, s_hwmax)) done = 0;
    }
    if (s_hwmin == WIICAM_HW_RESTORE) {
        // 0 is the direction that cannot cost an LED: accept the smallest
        // blobs. The power-on default is unknown, so there is nothing else to
        // go back to.
        if (s_blobreg(0x1B, 0)) s_hwmin = WIICAM_HW_LEAVE;
        else done = 0;
    } else if (s_hwmin != WIICAM_HW_LEAVE) {
        if (!s_blobreg(0x1B, s_hwmin)) done = 0;
    }
    // Re-marked on failure, so a write refused because the camera was down is
    // tried again rather than forgotten while cam? claims it landed.
    if (!done) s_hw_dirty = true;
    s_hw_busy = false;
}

// Called by the firmware after the preset rewrote register 0x06 (the pause
// menu, a profile switch, '~cam=sens:'). A changed preset restarts the loop's
// search from it; the same preset re-applied gets the loop's value put back.
void wiicam_aim_hw_dirty(void)
{
    s_hw_dirty = true;
    const int p = loop_preset();
    if (p == s_loop_preset_seen) return;
    s_loop_preset_seen = p;
    if (s_loop_on) { loop_reset(p); s_hwmax = WIICAM_HW_LEAVE; }
}

// ---- the relative gate ----------------------------------------------------
// The four emitters are the same hardware driven the same way, so in any ONE
// frame they should look alike. A blob that does not resemble the others is
// the suspect -- and judging by resemblance rather than by an absolute size
// makes the test immune to the thing that defeats absolute thresholds here:
// the play distance varies about 2x, which is a 4x swing in brightness and
// therefore in blob size. A fixed window has to straddle all of it. This one
// does not care.
//
// Cost asymmetry sets the aggression. Dropping a real LED is RECOVERABLE --
// the resolver rebuilds the fourth corner from the other three. Admitting an
// impostor that lands inside a slot's gate is CORRUPTING: it warps the learned
// rig model and the cursor jumps. So drop on suspicion, but never below three.
//
// Returns the number of blobs newly dropped, and clears their keep flags.
static int size_consensus_drop(const int* sz, int* keep, int n)
{
    if (s_rtol == 0 || n < 3) return 0;
    // Only blobs with a KNOWN size and still kept can take part: a consensus
    // built from blobs the absolute window already refused would be a
    // consensus about the contamination.
    int v[4], idx[4], m = 0;
    for (int i = 0; i < n; ++i)
        if (keep[i] && sz[i] >= 0) { v[m] = sz[i]; idx[m] = i; ++m; }
    if (m < 3) return 0;
    // median of at most four, by insertion sort
    int s[4];
    for (int i = 0; i < m; ++i) {
        int x = v[i], j = i - 1;
        while (j >= 0 && s[j] > x) { s[j + 1] = s[j]; --j; }
        s[j + 1] = x;
    }
    const int med = (m & 1) ? s[m / 2] : (s[m / 2 - 1] + s[m / 2]) / 2;
    const int tol = (int)s_rtol;        // steps, straight from the setting
    // Worst first, so the floor spends its allowance on the worst offenders.
    int dropped = 0;
    // Never leave fewer than three. This gate is statistical -- a consensus of
    // four samples -- so it gets the more cautious floor of the two.
    int budget = m - 3;
    while (budget > 0) {
        int worst = -1, worst_d = 0;
        for (int i = 0; i < m; ++i) {
            if (!keep[idx[i]]) continue;
            const int d = v[i] > med ? v[i] - med : med - v[i];
            if (d > tol && d > worst_d) { worst_d = d; worst = idx[i]; }
        }
        if (worst < 0) break;
        keep[worst] = 0;
        ++dropped;
        --budget;
    }
    return dropped;
}

void wiicam_aim_begin(void)
{
    quad_reset(&s_quad_cfg);       // single-threaded at boot, so direct
    int lead = 0;
    if (aim_lead_load(&lead)) s_lead_ms = (float)lead;
    int smooth = 0;
    if (aim_smooth_load(&smooth)) aim_smooth_set(smooth);
    int dead = 0;
    if (aim_dead_load(&dead)) aim_dead_set(dead);
    int beta = -1;
    if (aim_beta_load(&beta)) aim_beta_set(beta);
    // The blob gate, as one unit. ext_set() rather than a bare assignment so
    // the epoch moves with it and the camera poll applies the format on its
    // first tick -- a stored format that never reaches the sensor is a read
    // length that does not match the report, which decodes to nonsense.
    {
        int gfmt = 0, gmin = 0, gmax = 15, grtol = 0;
        if (aim_gate_load(&gfmt, &gmin, &gmax, &grtol)) {
            s_bmin = (uint8_t)gmin; s_bmax = (uint8_t)gmax;
            s_rtol = (uint8_t)grtol;
            ext_set(gfmt);
        }
        int gpx = 0, gar = 0, gbh = 0;
        if (aim_gate2_load(&gpx, &gar, &gbh)) {
            s_pxmax = (uint8_t)gpx; s_armax = (uint8_t)gar;
            s_bhmax = (uint8_t)gbh;
        }
    }
    // Recoil engine: defaults (dormant), then whatever was saved. The reply
    // sink is shared so FX: lines reach the tools the same way CAM: does.
    {
        fx_params_t fp2;
        fx_defaults(&fp2);
        fx_init(&fp2, (uint32_t)(fx_now() | 1u));
        fx_load();
        // Forwarded, not captured: the sink can be installed after begin().
        fx_set_reply([](const char* l) {
            if (s_reply) s_reply(l); else fputs(l, stdout);
        });
    }
    aim_lens_t ls;
    if (aim_lens_load(&ls)) {
        s_lens = (uint8_t)ls.model;
        s_lk1 = ls.k1; s_lk2 = ls.k2; s_lfpx = ls.fpx; s_lfeq = ls.feq;
        s_lcx = ls.cx; s_lcy = ls.cy;
    }
    // G3: the capture is armed at boot, so the gun learns as you play.
    wl_enable(1);
    // The loop. Bounds start UNKNOWN even with a saved value: the first
    // dwell's RAISE rule protects the LEDs (K4), not a bound from another room.
    s_loop_on = true;
    s_loop_last_lock_us = 0; s_loop_ever_lock = false;
    s_loop_store_req = false;
    s_loop_preset_seen = loop_preset();
    loop_reset(s_loop_preset_seen);
    {
        int lv = 0, llo = 0, lhi = LOOP_HI_UNKNOWN;
        if (aim_hwloop_load(&lv, &llo, &lhi) && lv > 0) {
            loop_write(lv);
            s_loop_state = LOOP_HOLD;
            s_loop_saved = true;      // it came from flash, so it is still there
            s_loop_from_flash = true; // ...but nothing has vouched for it yet
        }
    }
    s_prev_us = 0;
    s_cache_seen = 0xFFFFFFFFu;
    s_cache_ret = false;
}

// Identical model to the ESP32 build's lens_undistort, in 240-space.
static void lens_undistort(float* px, float* py)
{
    if (!s_lens) return;
    const float cx = WIICAM_NORM_W * 0.5f + s_lcx;
    const float cy = WIICAM_NORM_H * 0.5f + s_lcy;
    const float dx = *px - cx, dy = *py - cy;
    const float rd = sqrtf(dx*dx + dy*dy);
    if (rd < 1e-3f) return;
    float k;
    if (s_lens == 2) {
        float th = rd / s_lfeq;
        if (th > 1.45f) th = 1.45f;
        k = (s_lfpx * tanf(th)) / rd;
    } else {
        const float rdn = rd / s_lfpx;
        float ru = rdn;
        for (int i = 0; i < 3; ++i) {
            const float r2 = ru*ru;
            const float f  = ru*(1.0f + s_lk1*r2 + s_lk2*r2*r2) - rdn;
            const float df = 1.0f + 3.0f*s_lk1*r2 + 5.0f*s_lk2*r2*r2;
            if (fabsf(df) < 1e-6f) break;
            ru -= f/df;
        }
        k = ru / rdn;
    }
    *px = cx + dx*k; *py = cy + dy*k;
}

// Q,<ms>,<n>,x0,y0,..,x3,y3,<kind>,<real>,<ldx>,<ldy>: always four pairs (x10,
// 240x176 space, -1,-1 = unused); kind r raw / p partial / c corners PRE-lead;
// real = bit mask of pairs measured this frame; ld = the lead vector, x10.
static void emit_q(uint64_t now_us, const float* xs, const float* ys, int n,
                   char kind, unsigned real, float ldx, float ldy)
{
    if (s_dash != 2 || !s_line) return;
    if (s_dash_min_dt_us && (now_us - s_dash_last_us) < s_dash_min_dt_us) return;
    s_dash_last_us = now_us;
    char b[128];
    int o = snprintf(b, sizeof(b), "Q,%lu,%d",
                     (unsigned long)(now_us / 1000ull), n);
    for (int i = 0; i < 4 && o > 0 && o < (int)sizeof(b); ++i) {
        if (i < n)
            o += snprintf(b + o, sizeof(b) - o, ",%d,%d",
                          (int)lroundf(xs[i] * 10.0f), (int)lroundf(ys[i] * 10.0f));
        else
            o += snprintf(b + o, sizeof(b) - o, ",-1,-1");
    }
    if (o > 0 && o < (int)sizeof(b))
        o += snprintf(b + o, sizeof(b) - o, ",%c,%u,%d,%d\n", kind, real & 15u,
                      (int)lroundf(ldx * 10.0f), (int)lroundf(ldy * 10.0f));
    // snprintf reports the would-be length, so a truncated line is dropped whole.
    if (o > 0 && o < (int)sizeof(b)) s_line(b);
}

bool wiicam_aim_process(const int* px, const int* py, unsigned seen,
                        uint64_t now_us, float* sx, float* sy)
{
    return wiicam_aim_process_sz(px, py, 0, seen, now_us, sx, sy);
}

bool wiicam_aim_process_sz(const int* px, const int* py, const int* sizes,
                           unsigned seen, uint64_t now_us,
                           float* sx, float* sy)
{
    // The serial core's quad_reset, performed here on the camera core -- and
    // BEFORE the duplicate check, or a cached return would swallow it.
    if (s_quad_reset_pending) { s_quad_reset_pending = false; quad_reset(&s_quad_cfg); }

    // A byte-identical report is the previous camera frame seen again: return
    // the cached answer and leave every stateful stage untouched.
    bool same = (seen == s_cache_seen);
    for (int i = 0; same && i < 4; ++i)
        if ((seen & (1u << i)) && (px[i] != s_cache_px[i] || py[i] != s_cache_py[i]))
            same = false;
    // Sizes count too. Comparing coordinates alone threw away a frame whose
    // blob SIZES had changed -- and a stationary stray light source with a
    // flickering size is precisely what the gates exist to catch, seen in
    // precisely the posture the tuning readout is read in: gun resting still.
    if (same && sizes)
        for (int i = 0; same && i < 4; ++i)
            if ((seen & (1u << i)) && sizes[i] != s_cache_sz[i])
                same = false;
    if (same) {
        *sx = s_cache_sx; *sy = s_cache_sy;
        return s_cache_ret;
    }
    s_cache_seen = seen;
    for (int i = 0; i < 4; ++i) {
        s_cache_px[i] = px[i]; s_cache_py[i] = py[i];
        s_cache_sz[i] = sizes ? sizes[i] : -1;
    }
    s_cache_ret = false;

    // Seen-mask gate: the driver retains an unseen slot's previous
    // coordinates, so unmasked reads would feed the resolver stale points.
    float ax[4], ay[4];
    int   asz[4];
    int   aslot[4] = {0, 0, 0, 0};   // which hardware slot each entry came from
    int   an = 0;
    for (int i = 0; i < 4; ++i) {
        if (!(seen & (1u << i))) continue;
        float nx = s_mirx ? (WIICAM_W - 1.0f - (float)px[i]) : (float)px[i];
        float ny = s_miry ? (WIICAM_H - 1.0f - (float)py[i]) : (float)py[i];
        float x = nx * SX;
        float y = ny * SY;
        lens_undistort(&x, &y);
        ax[an] = x; ay[an] = y;
        asz[an] = sizes ? sizes[i] : -1;      // -1 = basic format, size unknown
        aslot[an] = i;                        // see the s_f*/s_b* note above
        ++an;
    }

    // Blob size gate. A size of -1 is never judged, so the gate cannot act on
    // a number the sensor did not send.
    //
    // The window is ORDERED here rather than when it is set: the tools send
    // one key per keystroke, so clamping bmin against the current bmax as it
    // arrives makes the result depend on which end was typed first -- asking
    // for 8..12 from a stored 0..3 gave 3..12.
    const int glo = (s_bmin <= s_bmax) ? (int)s_bmin : (int)s_bmax;
    const int ghi = (s_bmin <= s_bmax) ? (int)s_bmax : (int)s_bmin;
    int keep[4];
    int srej[4] = {0, 0, 0, 0};     // rejected by the SHAPE gate specifically
    int nk = 0;
    for (int i = 0; i < an; ++i) {
        keep[i] = 1;
        if (asz[i] >= 0 && (asz[i] < glo || asz[i] > ghi))
            keep[i] = 0;
        nk += keep[i];
    }
    // What the window WANTED to reject, counted before any floor gives blobs
    // back. Counting after the floor reported brej=0 on a window rejecting
    // every blob -- the readout said the gate was idle in exactly the case it
    // was misconfigured, which is the one case the number exists to reveal.
    for (int i = 0; i < an; ++i) if (!keep[i]) ++s_brej;

    // The shape gate, on the same keep[] and before the same floor. Full mode
    // only: s_fw/s_fh/s_fi are meaningless in any other format, and judging a
    // blob by a box the sensor never sent is how a gate rejects everything.
    if ((s_bhmax || s_pxmax || s_armax) && (s_ext_state & 3) == WIICAM_FMT_FULL) {
        for (int i = 0; i < an; ++i) {
            if (!keep[i]) continue;               // already gone, do not double-count
            const int w = (int)s_fw[aslot[i]];
            const int h = (int)s_fh[aslot[i]];
            // Not 'px': this function's first parameter is also px, and two
            // very different things three lines apart under one name is how
            // the next edit here goes wrong.
            const int bpx = (int)s_fi[aslot[i]];
            int drop = 0;
            if (s_bhmax && h > (int)s_bhmax) drop = 1;
            if (!drop && s_pxmax && bpx > (int)s_pxmax) drop = 1;
            // A zero side has no ratio -- it is a blob one pixel across in
            // that axis, which is the SMALLEST thing the sensor reports, not
            // an infinitely elongated one. Judging it would reject every
            // faint LED at the far end of the play range.
            if (!drop && s_armax && w > 0 && h > 0) {
                const int lo = (w < h) ? w : h;
                const int hi = (w < h) ? h : w;
                if ((8 * hi) / lo > (int)s_armax) drop = 1;
            }
            if (drop) { keep[i] = 0; srej[i] = 1; --nk; ++s_bsrej; }
        }
    }

    // The floor. Below TWO points the resolver cannot fit anything at all and
    // GetPosition falls through to the stock, uncalibrated path -- the cursor
    // jumps. A window that would do that is set wrong, and giving the least
    // offending blobs back is the correct response. Two rather than three
    // because this gate is a physical bound the user set deliberately, not a
    // statistical guess: it earns more trust than the consensus gate.
    if (an >= 2 && nk < 2) {
        // Give back the ones closest to the window, worst kept out longest.
        while (nk < 2) {
            int best = -1, best_d = 0;
            for (int i = 0; i < an; ++i) {
                if (keep[i]) continue;
                int d = 0;
                if (asz[i] >= 0) {
                    if (asz[i] < glo)      d = glo - asz[i];
                    else if (asz[i] > ghi) d = asz[i] - ghi;
                }
                if (best < 0 || d < best_d) { best_d = d; best = i; }
            }
            if (best < 0) break;
            keep[best] = 1;
            ++nk;
            ++s_bvalve;
        }
    }

    // Then the relative gate, on what the absolute window left: does each blob
    // look like the others in this same frame? It has its own floor, so it can
    // never take the count below three.
    s_brrej += (uint32_t)size_consensus_drop(asz, keep, an);

    float xs[4], ys[4];
    int   osz[4];                        // size of each OFFERED blob, -1 = unknown
    int   oidx[4];                       // and which s_b* entry it came from
    int n = 0;
    for (int i = 0; i < an; ++i) {
        s_bx[i]   = (int16_t)lroundf(ax[i]);
        s_by[i]   = (int16_t)lroundf(ay[i]);
        s_bsz[i]  = (int8_t)asz[i];
        s_bkeep[i] = (int8_t)keep[i];
        s_bw[i] = s_fw[aslot[i]];
        s_bh[i] = s_fh[aslot[i]];
        s_bi[i] = s_fi[aslot[i]];
        s_bxm[i] = s_fxm[aslot[i]];
        s_bym[i] = s_fym[aslot[i]];
        if (!keep[i]) continue;              // already tallied by its own gate
        xs[n] = ax[i]; ys[n] = ay[i]; osz[n] = asz[i]; oidx[n] = i; ++n;
    }
    // Count LAST. '~camblob?' is answered on the other core, and publishing
    // the count before the entries are written let it print a tail left over
    // from the previous frame -- in the one readout the size window is
    // supposed to be chosen from.
    s_bn = an;
    ++s_bframes;

    const float dt = (s_prev_us && now_us > s_prev_us)
                   ? (float)(now_us - s_prev_us) * 1e-6f : 0.0f;
    s_prev_us = now_us;

    if (s_res == 0) {                       // raw mode, for the lens sweep
        // Bucketed here too, by blobs kept: without this the frame counters
        // kept climbing during a lens sweep while the per-frame buckets did
        // not, so the percentages drawn from them stopped adding up and the
        // readout froze exactly when the gun is being waved around.
        s_breal[n > 4 ? 4 : n]++;
        emit_q(now_us, xs, ys, n, 'r', (1u << n) - 1u, 0.0f, 0.0f);
        // This return is also what pauses the hwmax loop: with no resolver
        // there is no verdict to read, so loop_tick does not run until 'res:2'
        // brings the resolver back. K2's window is wall time and keeps ageing,
        // which is right -- a lens sweep is not evidence about anything.
        return false;
    }

    // Level 5's fourth check, caller-side because the sizes are: a 15 among 2s
    // is not a corner, and the resolver is never told the sizes. Only with NO
    // learned model -- with one, the reseed has its own vetoes -- and only the
    // single worst offender, so three points always reach the resolver.
    // The Batch B oracle counts what the SENSOR gave: read bn, not bsv.
    if (an == 4 && n == 4 && !quad_has_model()
        && osz[0] >= 0 && osz[1] >= 0 && osz[2] >= 0 && osz[3] >= 0) {
        int v[4];
        for (int i = 0; i < 4; ++i) v[i] = osz[i];
        for (int i = 1; i < 4; ++i) {          // insertion sort, four values
            const int tv = v[i];
            int j = i - 1;
            while (j >= 0 && v[j] > tv) { v[j + 1] = v[j]; --j; }
            v[j + 1] = tv;
        }
        const int med = (v[1] + v[2]) / 2;
        int worst = 0;
        for (int i = 1; i < 4; ++i) if (osz[i] > osz[worst]) worst = i;
        if (osz[worst] > med + 6) {
            s_bkeep[oidx[worst]] = 0;      // not offered, so not 'kept' in camblob?
            for (int i = worst; i < 3; ++i) {
                xs[i] = xs[i + 1]; ys[i] = ys[i + 1];
                osz[i] = osz[i + 1]; oidx[i] = oidx[i + 1];
            }
            n = 3;
            ++s_bseedveto;
        }
    }

    QuadResult r = quad_update(xs, ys, n);
    // Blobs this frame sitting far from every resolved corner. Computed in the
    // learning block below and read by the verdict after it, so a frame that
    // never reaches that branch honestly reads zero.
    int nfar_frame = 0;
    // How many corners were really SEEN this frame is the honest measure of
    // how much the light is costing: a run that sits on 3 is a run where one
    // LED is being taken from us every frame. A passthrough frame is no model
    // yet, or the set was refused -- not a corner count, so it goes elsewhere.
    if (r.passthrough) {
        ++s_bcold;
    } else {
        int nr = r.n_real;
        if (nr < 0) nr = 0;
        if (nr > 4) nr = 4;
        ++s_breal[nr];
    }
    // ---- shape learning -----------------------------------------------
    // Fed only from frames whose label came from GEOMETRY -- where the blobs
    // sit, which no size or shape gate has a vote in. Nothing being taught is
    // downstream of the teacher, so the gate can never learn its own drift.
    //
    // Two labels, and the second one took a rewrite to get right. The obvious
    // reading -- "kept blobs in a confirmed frame are LEDs, rejected ones are
    // not" -- cannot produce a single negative sample on this sensor. It has
    // four object slots and only KEPT blobs reach the resolver, so a rejection
    // leaves three points and n_real can never be 4 in the same frame. The
    // positive class filled and the negative class was structurally empty,
    // which would have quietly reduced this whole exercise to "here is what an
    // LED looks like" with no way to ask the question that matters.
    //
    // The CLASSIFICATION runs on every frame; only the RECORDING into the
    // histograms is gated on the capture being armed. They were one block
    // once, and that made bfar and bnear count only while a capture ran --
    // which ships them at zero: the capture is off by default, nothing arms
    // it at boot, and the tools switch it off on the way out of the sweep. So
    // during ordinary play the tools' "unexplained rejections" warning, which
    // subtracts bfar and bnear from bsrej, degenerated to the raw count it was
    // written to replace, and the false-negative meter -- the one number that
    // says a gate is WRONG rather than working -- was structurally dead in
    // exactly the sessions where a wrong gate does its damage.
    {
        const int learning = wl_enabled();
        const int fmt = s_ext_state & 3;
        const int flags = (fmt == WIICAM_FMT_FULL) ? WL_HAS_BOX : 0;
        int nkept = 0, jrej = -1;
        for (int i = 0; i < an; ++i) {
            if (keep[i]) ++nkept;
            else jrej = i;
        }
        // Median intensity over the KEPT blobs, so WL_IREL is measured against
        // this frame's own brightness rather than a number that only held at
        // the distance it was learned from.
        int imed = 0;
        if (learning && flags && nkept) {
            int v[4], m = 0;
            for (int i = 0; i < an; ++i) if (keep[i]) v[m++] = (int)s_bi[i];
            for (int i = 1; i < m; ++i) {          // insertion sort, m <= 4
                const int t = v[i];
                int j = i - 1;
                while (j >= 0 && v[j] > t) { v[j + 1] = v[j]; --j; }
                v[j + 1] = t;
            }
            imed = (m & 1) ? v[m / 2] : (v[m / 2 - 1] + v[m / 2]) / 2;
        }
        if (r.locked && r.n_real == 4) {
            // Four corners really seen, ON A MODEL THE RESOLVER TRUSTS.
            //
            // 'locked' is not decoration here, and leaving it off was a real
            // bug: n_real == 4 alone is also true on the cold-start raw
            // passthrough, which publishes ANY four blobs with no rectangle
            // check and no scale check at all, and true again for the frames
            // immediately after an angular seed, which sorts four blobs by
            // angle and asks nothing else of them. Lock takes lock_frames of
            // consistent geometry to come true; that wait IS the verification
            // this class claims in the header ("a plausible rectangle at a
            // plausible scale"), and without it the window and the lamp get
            // learned as LEDs on the first frames after every quad_reset --
            // which the tools trigger routinely with res:, mirx: and miry:,
            // so the hole reopens long after boot.
            //
            // The damage is not just a loose ceiling. rig_led_max_h() reads
            // this same edge, so junk in the positive class RAISES the refusal
            // floor: a lamp learned as an LED is then what refuses the cut
            // that would have caught it, and the user is locked out of the one
            // setting they need with a message citing a measurement that never
            // happened. The negative branch below always had this condition.
            if (learning) {
                for (int i = 0; i < an; ++i)
                    wl_note(0, asz[i], (int)s_bw[i], (int)s_bh[i],
                            (int)s_bi[i], imed, flags);
                wl_note_frame();
            }
        } else if (r.locked && r.count == 4 && r.n_real == 3) {
            // Three real corners and a reconstructed fourth. The
            // reconstruction says where the missing LED must be, so any blob
            // in this frame can be judged on POSITION alone: far from every
            // corner and it was not an LED, whatever its size.
            //
            // The test is the RESOLVER's association, not the gate's verdict.
            // Requiring a gate rejection here was a design error that would
            // have shipped the feature dead: every gate defaults to 0, so
            // nothing is ever rejected, so the negative class stayed
            // structurally empty and 'camfit' could never reach a verdict on a
            // gun straight out of the box -- on which it is the only thing
            // that would have set a gate in the first place. The window case
            // is not a rejection at all: the sensor has four slots, a bright
            // window DISPLACES an LED, and the stray arrives as an ordinary
            // kept blob that the resolver then declines to associate.
            //
            // Twice the resolver's own association radius, not once. Being
            // wrong here poisons the negative distribution with real LEDs and
            // teaches the gate to reject them, so the bar for calling
            // something a stray is deliberately higher than the bar the
            // resolver uses for calling something a corner.
            const float gate2 = 2.0f * quad_default_config().gate;
            int nfar = 0, jfar = -1;
            for (int i = 0; i < an; ++i) {
                float dmin = 1e9f;
                for (int k = 0; k < 4; ++k) {
                    const float dx = ax[i] - r.p[k].x;
                    const float dy = ay[i] - r.p[k].y;
                    const float d = dx * dx + dy * dy;
                    if (d < dmin) dmin = d;
                }
                if (dmin > gate2 * gate2) { ++nfar; jfar = i; }
            }
            nfar_frame = nfar;
            // Exactly one, and no more. One corner is missing, so at most one
            // blob in this frame can honestly be the thing standing in its
            // place. Two or more far blobs means the resolver's own
            // association is in doubt, and a label drawn from a geometry we do
            // not trust is worse than no label.
            if (nfar == 1 && learning)
                wl_note(1, asz[jfar], (int)s_bw[jfar], (int)s_bh[jfar],
                        (int)s_bi[jfar], imed, flags);
            // The two counters the tools subtract from bsrej, and they count
            // SHAPE-GATE rejections only -- not every far blob, and not every
            // rejection by any gate. bfar used to count every far blob once
            // the label went positional, which put kept strays in it; bnear
            // counted a blob dropped by the size window or the odd-one-out
            // gate too. Subtracting either from bsrej, which counts shape
            // rejections alone, mixed three populations: one kept stray a
            // frame cancelled the threshold and masked a shape gate eating a
            // real LED, and a bnear from rtol sent the user to switch off a
            // height gate that was not the one doing it. With both narrowed
            // to what the shape gate threw away, bsrej - bfar - bnear is
            // exactly "shape rejections the resolver could not account for",
            // and "bhmax:0 turns it off" is true whenever bnear speaks.
            //
            // bfar: the shape gate dropped it AND it sat far from every
            // corner -- a stray, vouched for. bnear: the shape gate dropped it
            // AND it sat where the missing LED had to be -- almost certainly
            // a real corner thrown away. A gate that is too tight shows up in
            // bnear long before it shows up as a cursor that will not track.
            if (jrej >= 0 && nkept == an - 1 && srej[jrej]) {
                if (jrej == jfar && nfar == 1) ++s_bfar;
                else if (jrej != jfar)         ++s_bnear;
            }
        }
    }

    // ---- the hwmax loop's oracle --------------------------------------
    // K1: judged on 'an', what the SENSOR reported, before any software gate.
    {
        // K2: a corner missing while the gun points away is not evidence; the
        // window is wall time so it ages while a parked gun sends duplicates.
        const bool recent = s_loop_ever_lock
                         && (now_us - s_loop_last_lock_us) < LOOP_RECENT_US;
        int verdict = V_NONE;
        if (an == 4 && r.locked && r.n_real == 4)
            verdict = V_CLEAN;
        else if (an == 4 && r.locked && r.count == 4 && r.n_real == 3
                 && nfar_frame == 1)
            verdict = V_STRAY;          // a slot went to something that is not a corner
        else if (an == 4 && !r.locked)
            verdict = V_STRAY;          // four sensor blobs and still no lock: also "lower"
        else if (an <= 3 && recent)
            verdict = V_CUT;            // a corner we had a moment ago is simply gone
        // Right after a LOWER the value is untested and a stray-only room may
        // never have locked, so three blobs is a cut there too (K4).
        else if (an == 3 && s_loop_state == LOOP_LOWER)
            verdict = V_CUT;
        loop_tick(verdict);
    }
    if (r.locked) { s_loop_last_lock_us = now_us; s_loop_ever_lock = true; }

    if (r.count < 4) {
        ++s_bdrop;
        emit_q(now_us, xs, ys, n, 'p', (1u << n) - 1u, 0.0f, 0.0f);
        return false;
    }

    // Latency lead: extrapolate the published quad along its velocity,
    // clamped; never fed back into the resolver.
    float qx[4], qy[4];
    float lx = 0.0f, ly = 0.0f;
    // 'locked' here for the same reason as in the learning hook above, and to
    // stop the two front ends disagreeing: the OV2640 path has always guarded
    // this identical computation with q.locked && q.n_real >= 2. Before a lock,
    // r.vx/vy is a real per-frame delta of an UNVERIFIED four-point model, and
    // scaling it by lead_ms/dt pushes every published corner along it. The
    // n_real >= 2 half is covered already -- the resolver zeroes vx/vy below
    // two associated points -- so only the lock was missing. Cost of the guard
    // is no lead for the pre-lock frames after each quad_reset, which is the
    // conservative direction: less extrapolation, never more.
    if (s_lead_ms > 0.0f && dt > 1e-4f && r.locked) {
        const float frames = (s_lead_ms * 1e-3f) / dt;
        lx = r.vx * frames; ly = r.vy * frames;
        const float m = sqrtf(lx*lx + ly*ly);
        if (m > WIICAM_LEAD_PX_MAX) { lx *= WIICAM_LEAD_PX_MAX/m; ly *= WIICAM_LEAD_PX_MAX/m; }
    }
    // The wire carries the resolver's corners and the lead SEPARATELY; the
    // solve below still runs on corners + lead, exactly as before.
    {
        float cx[4], cy[4];
        unsigned real = 0;
        for (int i = 0; i < 4; ++i) {
            cx[i] = r.p[i].x; cy[i] = r.p[i].y;
            if (r.p[i].real) real |= 1u << i;
        }
        emit_q(now_us, cx, cy, 4, 'c', real, lx, ly);
    }
    for (int i = 0; i < 4; ++i) { qx[i] = r.p[i].x + lx; qy[i] = r.p[i].y + ly; }

    if (!aim_runtime_active()) return false;
    aim_pt_t q[4];
    for (int i = 0; i < 4; ++i) { q[i].x = qx[i]; q[i].y = qy[i]; }
    s_cache_ret = aim_runtime_solve(q, WIICAM_NORM_W, WIICAM_NORM_H,
                                    &s_cache_sx, &s_cache_sy, dt);
    *sx = s_cache_sx; *sy = s_cache_sy;
    return s_cache_ret;
}

// ---- the '~cam' command subset -------------------------------------------
// Reports through *got whether any digit was actually present. Without that,
// a value-less or garbled key ("hwmax:", "hwmax:0x40" -- a plausible thing to
// type for a register) read as ZERO, and zero written to MAXSIZE tells the
// sensor to reject every blob: the gun goes dark, from a typo.
static int parse_int(const char** p, int* got)
{
    int sgn = 1, v = 0, n = 0;
    if (**p == '-') { sgn = -1; ++(*p); }
    // Digits past the cap are consumed but not accumulated: signed overflow is
    // undefined behaviour, and a long digit string on a serial line is a typo
    // or a garbled byte, not a value worth wrapping the world around. Every
    // key clamps to its own range afterwards.
    while (**p >= '0' && **p <= '9') {
        const int d = *(*p)++ - '0';
        ++n;
        if (v < 100000000) v = v * 10 + d;
    }
    if (got) *got = (n > 0);
    return v * sgn;
}

bool wiicam_cam_command(const char* line)
{
    if (!line) return false;
    // The recoil engine's commands ride the same channel: fx=, fx?, fxsave.
    if (fx_command(line, fx_now())) return true;
    if (!strncmp(line, "camsave", 7)) {
        // Every store is checked. A reply that says "saved" while one write
        // quietly failed is worse than no reply, because the tools report it
        // to the user as confirmed. No short-circuit: all stores still run.
        bool ok = aim_lead_store((int)s_lead_ms);
        ok = aim_smooth_store(aim_smooth_get()) && ok;
        ok = aim_dead_store(aim_dead_get()) && ok;
        ok = aim_beta_store(aim_beta_get()) && ok;
        ok = (fx_store() != 0) && ok;       // recoil knobs ride the same save
        // The software gate rides it too. It has to: the first real capture
        // showed the gun holding 99.7% four-corner with rtol on, and every
        // one of those settings died on the next power cycle -- including the
        // format, without which the size arrives as -1 and the gate silently
        // judges nothing at all. The two SENSOR thresholds stay out on
        // purpose: they are the only settings that can leave a gun dark, and
        // a power cycle has to remain a way out.
        // The FORMAT is saved AS IT STANDS, full mode included. This used to
        // be clamped to extended, with a note saying to lift the clamp once
        // full mode was confirmed -- and the clamp then outlived its reason and
        // quietly gutted the feature built on top of it. The shape gate runs in
        // full mode only, so a clamped format meant every saved bhmax loaded
        // into a gun that could never execute it: the gate died on each power
        // cycle while the tools reported it as active. A gate that silently
        // does nothing is strictly worse than no gate, because the user stops
        // looking for the real problem.
        //
        // Full mode is confirmed now: 0x55 selects it on this sensor, and two
        // daylight captures came back with 36,420 confirmed LED boxes. The
        // original hazard -- booting into a format the sensor does not honour,
        // reading a length that does not match the report, and aiming wildly
        // with no console to recover from -- is covered by machinery that
        // already exists: GetPosition counts failed format writes and calls
        // wiicam_aim_fmt_fallback(WIICAM_FMT_EXT) after five, so a sensor that
        // will not do full mode drops back within a few frames on its own.
        // fullreg is deliberately NOT saved, so a boot always starts from the
        // 0x55 default rather than from a byte someone was experimenting with.
        const int save_fmt = (int)(s_ext_state & 3);
        ok = aim_gate_store(save_fmt, (int)s_bmin, (int)s_bmax, (int)s_rtol)
             && ok;
        ok = aim_gate2_store((int)s_pxmax, (int)s_armax, (int)s_bhmax) && ok;
        // The LED edge of the provenance rides with it -- GATED and MAXED, or
        // it undoes the floor through the back door. An earlier version wrote
        // the live envelope whenever it had one: arm the capture in a dim
        // corner, collect forty small blobs, press any Save button, and a
        // stored led_max_h of 7 became 3. The refusal floor reads max(live,
        // stored), so replacing 'stored' with the thin capture is the same
        // failure that rule exists to prevent, one command later. Now the
        // write needs the same 500 blobs camfit needs before it will derive
        // anything, and only ever RAISES what is already in flash. The stray
        // edge is left alone: it is camfit=apply's to record, since it is
        // what a ceiling was derived FROM, and a hand-set ceiling has no
        // stray behind it to record.
        //
        // And its verdict is kept OUT of 'ok'. fit0 sits in its own key so a
        // bad write there can never cost the gate; folding it into the reply
        // handed the tools a "SAVE FAILED" for a gate that had saved fine,
        // and pical then told the user the format would not survive a power
        // cycle when it had.
        {
            wl_envelope_t e;
            wl_envelope(&e);
            int sled = -1, sstray = -1, spx = -1;
            const bool have = aim_fit_load(&sled, &sstray, &spx);
            if (e.led_n >= 500 && e.led_max_h >= 0) {
                const int nl = (have && sled > e.led_max_h) ? sled : e.led_max_h;
                const int np = (have && spx > e.led_max_px) ? spx : e.led_max_px;
                if (!have || nl > sled || np > spx)
                    (void)aim_fit_store(nl, have ? sstray : 0, np);
            }
        }
        if (s_sens_save) s_sens_save();     // sens lives in OpenFIRE's profile
        aim_lens_t ls = { (int)s_lens, s_lk1, s_lk2, s_lfpx, s_lfeq,
                          s_lcx, s_lcy };
        bool lens_ok = true;
        if (ls.model == 0) aim_lens_clear();
        else               lens_ok = aim_lens_store(&ls);
        // beta rides in the reply so a tool can VERIFY what was written rather
        // than assume it; cam? reports the live value, this reports the stored one.
        reply(ok && lens_ok
              ? "CAM: saved lead=%dms smooth=%d dead=%d beta=%d lens=%d fmt=%d bmin=%d bmax=%d rtol=%d bhmax=%d pxmax=%d armax=%d (sens lives in the OpenFIRE profile; hwmin and a hand-set hwmax are NOT saved; the loop's own hwmax is saved by the loop when it settles; fullreg is not saved either and comes back as 0x55)\n"
              : "CAM: SAVE FAILED lead=%dms smooth=%d dead=%d beta=%d lens=%d\n",
              (int)s_lead_ms, aim_smooth_get(), aim_dead_get(), aim_beta_get(),
              (int)s_lens,
              save_fmt,
              (int)s_bmin, (int)s_bmax, (int)s_rtol,
              (int)s_bhmax, (int)s_pxmax, (int)s_armax);
        return true;
    }
    if (!strncmp(line, "camdiag", 7)) {
        // Sensor connection test: which of power, wiring and the sensor
        // itself is broken, from the gun's own pins.
        if (s_diag) s_diag();
        else reply("CAM: diag not available in this build\n");
        return true;
    }
    if (!strncmp(line, "camblob?", 8)) {
        // What the sensor actually handed us last frame, and what the gate did
        // with it. This is the measurement the gate has to be set from: if the
        // window and the LEDs land on the same size, no gate can separate them
        // and the honest answer is to say so rather than to tune in the dark.
        // Every bucket, including the one where nothing at all was seen: the
        // percentages a tool draws from these have to add up, or "80% good"
        // can quietly be 80% of the frames that were not already hopeless.
        // bms is the gun's own millisecond clock. With it, (delta bframes /
        // delta bms) is the camera's TRUE new-frame rate -- the number nobody
        // has ever measured on this sensor, and the one that decides whether a
        // bigger read per frame costs us anything. Timing it host-side instead
        // would measure the tool's poll jitter, not the camera.
        // ONE read of the count, used by both lines. The two replies are
        // emitted back to back but the camera poll runs on the other core and
        // can publish a new frame between them: read separately, the count
        // came from one frame and the list from the next, and 1.4% of the rows
        // in the first real capture disagreed with themselves. Nothing in the
        // pipeline reads these fields, but every log we analyse does.
        const int fmt = s_ext_state & 3;
        const int bn  = s_bn;
        // And copy the entries, not just the count. The count alone stopped
        // the two lines DISAGREEING, but the poll core republishes the arrays
        // at frame rate and the second reply is emitted after the first: a
        // frame landing in between left line 2 printing this frame's first two
        // blobs and the previous frame's last two, under a count that looked
        // consistent. Four blobs is 28 bytes of stack to make the whole reply
        // one frame's worth of truth.
        int16_t bx[4], by[4];
        int8_t  bsz[4], bkeep[4];
        uint8_t bw[4], bh[4], bi[4], bxm[4], bym[4];
        for (int i = 0; i < 4; ++i) {
            bx[i] = s_bx[i]; by[i] = s_by[i];
            bsz[i] = s_bsz[i]; bkeep[i] = s_bkeep[i];
            bw[i] = s_bw[i]; bh[i] = s_bh[i]; bi[i] = s_bi[i];
            bxm[i] = s_bxm[i]; bym[i] = s_bym[i];
        }
        reply("CAM: blob fmt=%u ext=%u fullreg=%u bmin=%u bmax=%u rtol=%u "
              "bhmax=%u pxmax=%u armax=%u hwmax=%d hwmin=%d "
              "bn=%d brej=%lu brrej=%lu bvalve=%lu bframes=%lu bms=%lu "
              "bdrop=%lu bsrej=%lu bfar=%lu bnear=%lu bsv=%lu bcold=%lu "
              "br4=%lu br3=%lu br2=%lu br1=%lu br0=%lu\n",
              (unsigned)fmt, (unsigned)(fmt >= WIICAM_FMT_EXT),
              (unsigned)wiicam_aim_fullreg(), (unsigned)s_bmin, (unsigned)s_bmax,
              (unsigned)s_rtol, (unsigned)s_bhmax, (unsigned)s_pxmax, (unsigned)s_armax,
              (int)(s_hwmax < 0 ? -1 : s_hwmax),
              (int)(s_hwmin < 0 ? -1 : s_hwmin), bn,
              (unsigned long)s_brej, (unsigned long)s_brrej,
              (unsigned long)s_bvalve, (unsigned long)s_bframes,
              (unsigned long)(fx_now() / 1000ull),
              (unsigned long)s_bdrop, (unsigned long)s_bsrej,
              (unsigned long)s_bfar, (unsigned long)s_bnear,
              (unsigned long)s_bseedveto, (unsigned long)s_bcold,
              (unsigned long)s_breal[4],
              (unsigned long)s_breal[3], (unsigned long)s_breal[2],
              (unsigned long)s_breal[1], (unsigned long)s_breal[0]);
        // In full mode each blob carries three more numbers -- box width, box
        // height and intensity -- so the line grows and the buffer with it.
        // Nine fields a blob in full mode, four of them added since this was
        // 128 bytes. Sized for the worst case the sensor can produce rather
        // than for the typical one: a truncated readout line is a silently
        // short row in every log analysed afterwards.
        char b[320];
        int o = snprintf(b, sizeof(b), "CAM: blobs");
        const int need = (fmt == WIICAM_FMT_FULL) ? 46 : 20;
        for (int i = 0; i < bn && i < 4 && o > 0 && o < (int)sizeof(b) - need; ++i) {
            o += snprintf(b + o, sizeof(b) - o, " %d,%d,%d,%d",
                          (int)bx[i], (int)by[i], (int)bsz[i], (int)bkeep[i]);
            if (fmt == WIICAM_FMT_FULL && o > 0 && o < (int)sizeof(b) - 24)
                o += snprintf(b + o, sizeof(b) - o, ",%d,%d,%d,%d,%d",
                              (int)bw[i], (int)bh[i], (int)bi[i],
                              (int)bxm[i], (int)bym[i]);
        }
        if (fmt == WIICAM_FMT_BASIC && o > 0 && o < (int)sizeof(b) - 34)
            o += snprintf(b + o, sizeof(b) - o, " (sizes need fmt:1)");
        reply("%s\n", b);
        return true;
    }
    if (!strncmp(line, "camfit", 6)) {
        // Turn the two measured distributions into a ceiling, or say honestly
        // that this rig has none. Everything here is derived from what THIS
        // gun has seen; no number in it came from another rig.
        wl_envelope_t e;
        wl_envelope(&e);
        // Exactly "=apply", not "=apply" plus anything. Every '~cam' command
        // here matches on a prefix, which is harmless for a query but not for
        // the one form that writes flash: 'camfit=applyfoo' would have set and
        // saved a gate the user never asked for, from a typo.
        const bool apply = !strcmp(line + 6, "=apply");
        reply("CAM: fit ledn=%lu ledmaxh=%d straym=%lu strayminh=%d\n",
              e.led_n, e.led_max_h, e.stray_n, e.stray_min_h);
        // Contamination is said out loud rather than silently discounted. The
        // samples set aside are almost certainly a window or the sun learned
        // during a cold-start lock (see wl_envelope), and a user who sees
        // "32 ignored at 31" understands their rig better than one who sees
        // a clean 7 and never learns the sun was in the LED class at all.
        if (e.led_outliers_h)
            reply("CAM: fit %lu LED samples ignored -- they reach %d tall, far "
                  "above the %d the rest stop at, and are almost certainly "
                  "stray light learned while the resolver locked on it\n",
                  e.led_outliers_h, e.led_abs_max_h, e.led_max_h);
        if (e.led_n < 500 || e.led_max_h < 0) {
            // The stored pair, if there is one. This is the whole reason it is
            // written: after a power cycle the histograms are empty, and
            // without this "why is bhmax 8?" has no answer at all -- the gun
            // carries a number and no record of where it came from.
            int sled = -1, sstray = -1, spx = -1;
            if (aim_fit_load(&sled, &sstray, &spx))
                reply("CAM: fit STORED ledmaxh=%d strayminh=%d ledmaxpx=%d -- "
                      "from an earlier capture on this gun; recapture if the "
                      "bar, the lens or the room has changed\n",
                      sled, sstray, spx);
            reply("CAM: fit NEEDS MORE LED DATA -- aim at the bar with the "
                  "capture on; %lu blobs so far, 500 wanted\n", e.led_n);
            return true;
        }
        if (e.stray_n < 20 || e.stray_min_h < 0) {
            reply("CAM: fit NO STRAY DATA -- sweep the room with the screen in "
                  "view so a lamp or window enters frame; %lu seen, 20 wanted. "
                  "Your LEDs measured %d tall.\n", e.stray_n, e.led_max_h);
            return true;
        }
        const int gap = e.stray_min_h - e.led_max_h;
        if (gap <= 0) {
            // The one answer a gate cannot give itself. Said plainly, because
            // the alternative is a number that half-works on a rig where
            // nothing can work, and months of tuning a knob that was never
            // going to help.
            reply("CAM: fit NO SAFE GATE -- your LEDs reach %d tall and the "
                  "stray light starts at %d, so a size gate cannot tell them "
                  "apart. Move the bar, block the light, or use brighter "
                  "LEDs.\n", e.led_max_h, e.stray_min_h);
            return true;
        }
        // The gate keeps h <= bhmax and drops only h > bhmax, so the ceiling
        // is the tallest height STILL ALLOWED. With a gap of 1 there is no
        // room between the two, and the only value that separates them is the
        // LED maximum itself: it keeps every LED this rig has measured and
        // rejects the stray one step above. Setting led_max_h + 1 -- which is
        // what this did -- lands exactly ON the stray height and therefore
        // rejects nothing at all, a "TIGHT" gate that is really a no-op.
        const int ceil_ = (gap > 1) ? (e.led_max_h + gap / 2) : e.led_max_h;
        reply("CAM: fit bhmax=%d (LEDs reach %d, stray starts at %d%s)\n",
              ceil_, e.led_max_h, e.stray_min_h,
              gap == 1 ? " -- TIGHT, only one step between them" : "");
        if (apply) {
            s_bhmax = (uint8_t)ceil_;
            // Apply means APPLY. The shape gate runs in full mode only, and a
            // verdict can only ever have come from full-mode data (the box
            // features need it), so full mode is what this ceiling was
            // measured in and what it needs to act. Switch to it now if the
            // gun has since dropped out of it, and persist FULL -- not
            // whatever format happens to be live. The first cut stored the
            // live format: apply from fmt:1 and the gun saved "bhmax=8,
            // fmt=1", inert on this boot and every boot after, while the
            // reply told the user to "set fmt:2" and let them believe the
            // fix would stick. Both tools had also grown a workaround telling
            // the user to press Save as well; a command called 'apply' that
            // does not survive a reboot is the bug, not something to
            // document. bmin/bmax/rtol ride along unchanged because gate0
            // holds all four in one word.
            if ((s_ext_state & 3) != WIICAM_FMT_FULL) {
                ext_set(WIICAM_FMT_FULL);
                reply("CAM: fit switched to fmt:2 -- the shape gate needs it "
                      "and the ceiling was measured in it\n");
            }
            const bool ok = aim_gate2_store((int)s_pxmax, (int)s_armax,
                                            (int)s_bhmax)
                         && aim_gate_store(WIICAM_FMT_FULL, (int)s_bmin,
                                           (int)s_bmax, (int)s_rtol)
                         && aim_fit_store(e.led_max_h, e.stray_min_h,
                                          e.led_max_px);
            reply(ok ? "CAM: fit applied and saved\n"
                     : "CAM: fit applied but SAVE FAILED -- it will be gone on "
                       "the next power cycle\n");

        } else {
            reply("CAM: fit not applied -- send camfit=apply to set and save "
                  "it\n");
        }
        return true;
    }
    if (!strncmp(line, "camlearn", 8)) {
        const char* p = line + 8;
        if (!strncmp(p, "=reset", 6)) {
            wl_reset();
            reply("CAM: learn cleared\n");
            return true;
        }
        if (!strncmp(p, "=on:", 4)) {
            // Turning it on clears first (see wl_enable): a distribution
            // accumulated across a change of sensitivity or of the LED bar is
            // two rigs averaged into one wide spread, which reads as "cannot
            // be separated" when in truth neither rig was measured.
            wl_enable(p[4] != '0');
            reply("CAM: learn %s -- feeding only resolver-confirmed frames\n",
                  wl_enabled() ? "ON" : "off");
            return true;
        }
        // The histograms, one line per class and feature. Split rather than
        // packed because a single reply carrying 384 numbers would not fit any
        // sane buffer, and because a tool that loses one line should lose one
        // feature rather than the whole capture.
        reply("CAM: learn on=%d frames=%lu led=%lu rej=%lu bins=%d\n",
              wl_enabled(), (unsigned long)wl_frames(),
              (unsigned long)wl_blobs(0), (unsigned long)wl_blobs(1), WL_BINS);
        for (int cls = 0; cls < WL_CLASSES; ++cls) {
            for (int f = 0; f < WL_NFEAT; ++f) {
                const uint16_t* h = wl_hist(cls, f);
                if (!h) continue;
                char b[224];
                int o = snprintf(b, sizeof(b), "CAM: hist c=%d f=%s", cls,
                                 wl_feat_name(f));
                for (int i = 0; i < WL_BINS && o > 0 && o < (int)sizeof(b) - 8; ++i)
                    o += snprintf(b + o, sizeof(b) - o, " %u", (unsigned)h[i]);
                reply("%s\n", b);
            }
        }
        return true;
    }
    if (!strncmp(line, "camreset", 8)) {
        aim_lens_clear();
        s_lens = 0; s_lead_ms = 0.0f;
        s_lcx = 0.0f; s_lcy = 0.0f;
        // The blob gate goes back to inert here too: it is the one setting
        // that can stop a gun aiming if it is set wrong, so the command a user
        // reaches for when nothing works must undo it.
        if (s_ext_state & 3) ext_set(0);
        s_bmin = 0; s_bmax = 15;
        s_rtol = 0;
        s_pxmax = 0; s_armax = 0; s_bhmax = 0;
        // RESTORE, not "leave alone". Marking them untouched left whatever we
        // had written in the sensor -- so the one command a user reaches for
        // when the gun has gone dark could not undo the one setting able to
        // make it go dark. MAXSIZE comes back from the sensitivity preset;
        // MINSIZE goes to 0, the direction that cannot cost an LED.
        s_hwmax = WIICAM_HW_RESTORE; s_hwmin = WIICAM_HW_RESTORE;
        s_hw_dirty = true;
        // The loop starts over from the preset, and its saved value goes too,
        // or the reset would fix the session and lose it on the next boot.
        loop_reset(loop_preset());
        s_loop_on = true;
        // Erase every saved key, each attempted whatever the others did.
        bool gone = aim_gate_clear();
        gone = aim_gate2_clear() && gone;
        gone = aim_fit_clear()   && gone;
        gone = aim_hwloop_clear() && gone;
        // The live histograms go too (or the floor outlives the reset), and
        // the capture is armed as at boot (G3), so it refills from here.
        wl_reset();
        wl_enable(1);
        reply(gone ? "CAM: lens + lead cleared, blob gates off and unsaved, "
                     "sensor thresholds restored\n"
                   : "CAM: lens + lead cleared, blob gates off, sensor "
                     "thresholds restored -- SAVED GATE NOT ERASED, it will "
                     "come back on the next boot\n");
        return true;
    }
    if (!strncmp(line, "cam?", 4)) {
        reply("CAM: board=rp2040-wiicam sens=%d mirx=%d miry=%d lead=%d "
              "smooth=%d dead=%d lens=%d lk1u=%d lk2u=%d lfpx=%d lfeq=%d "
              "lcxu=%d lcyu=%d beta=%d res=%u dash=%u "
              "ext=%u fmt=%u fullreg=%u bmin=%u bmax=%u rtol=%u "
              "bhmax=%u pxmax=%u armax=%u hwmax=%d hwmin=%d "
              "loop=%d hwv=%d hwlo=%d hwhi=%d hws=%s\n",
              s_sens_get ? s_sens_get() : -1, (int)s_mirx, (int)s_miry,
              (int)s_lead_ms, aim_smooth_get(), aim_dead_get(),
              (int)s_lens, (int)(s_lk1*1e6f), (int)(s_lk2*1e6f),
              (int)(s_lfpx*10.0f), (int)(s_lfeq*10.0f),
              (int)(s_lcx*10.0f), (int)(s_lcy*10.0f),
              aim_beta_get(),
              (unsigned)s_res, (unsigned)s_dash,
              (unsigned)((s_ext_state & 3) >= WIICAM_FMT_EXT),
              (unsigned)(s_ext_state & 3), (unsigned)wiicam_aim_fullreg(),
              (unsigned)s_bmin, (unsigned)s_bmax, (unsigned)s_rtol,
              (unsigned)s_bhmax, (unsigned)s_pxmax, (unsigned)s_armax,
              (int)(s_hwmax < 0 ? -1 : s_hwmax),
              (int)(s_hwmin < 0 ? -1 : s_hwmin),
              (int)s_loop_on, s_loop_val, s_loop_lo, s_loop_hi,
              loop_state_name());
        return true;
    }
    if (!strncmp(line, "camloop?", 8)) {
        // The whole controller in one line: where it is, what it is bracketed
        // between, and what this dwell has seen. Read it while the gun runs --
        // the counts are per dwell and reset with every register write.
        reply("CAM: loop on=%d state=%s val=%d lo=%d hi=%d dwell=%d/%d "
              "clean=%d stray=%d cut=%d settled=%d saved=%d\n",
              (int)s_loop_on, loop_state_name(), s_loop_val, s_loop_lo,
              s_loop_hi, s_loop_dwell, LOOP_DWELL,
              s_loop_nclean, s_loop_nstray, s_loop_ncut,
              (int)(s_loop_hold_run >= LOOP_SETTLED), (int)s_loop_saved);
        return true;
    }
    if (strncmp(line, "cam=", 4) != 0) return false;
    const char* p = line + 4;
    while (*p) {
        char key[8] = {0}; int ki = 0;
        while (*p && *p != ':' && *p != ',' && ki < 7) key[ki++] = *p++;
        if (*p != ':') { while (*p && *p != ',') ++p; if (*p) ++p; continue; }
        ++p;
        int have = 0;
        const int val = parse_int(&p, &have);
        if (*p == ',') ++p;
        // A key with no number after the colon is a truncated or garbled line,
        // not a request to set zero. Skipped entirely rather than acted on.
        if (!have) continue;
        // Reset when the value CHANGES, either way -- res:0 used to leave the
        // resolver standing. The tools re-send res:2 on every connect, and a
        // locked gun must not lose its lock for that.
        if      (!strcmp(key, "res"))  { const uint8_t nv = (uint8_t)(val ? 2 : 0);
                                         if (nv != s_res) s_quad_reset_pending = true;
                                         s_res = nv; }
        else if (!strcmp(key, "dash")) { s_dash = (uint8_t)(val < 0 ? 0 : (val > 2 ? 2 : val));
                                         if (s_dash == 2 && s_dash_min_dt_us == 0)
                                             s_dash_min_dt_us = 1000000 / 60; }
        else if (!strcmp(key, "dashhz")) { s_dash_min_dt_us = (val > 0) ? (1000000u / (uint32_t)val) : 0; }
        else if (!strcmp(key, "lead")) { float v = (float)(val < 0 ? 0 : val);
                                         if (v > WIICAM_LEAD_MS_MAX) v = WIICAM_LEAD_MS_MAX;
                                         s_lead_ms = v; }
        else if (!strcmp(key, "lens")) { s_lens = (uint8_t)(val < 0 ? 0 : (val > 2 ? 2 : val)); }
        else if (!strcmp(key, "lk1u")) { s_lk1 = (float)val * 1e-6f; }
        else if (!strcmp(key, "lk2u")) { s_lk2 = (float)val * 1e-6f; }
        else if (!strcmp(key, "lfpx")) { if (val > 0) s_lfpx = (float)val / 10.0f; }
        else if (!strcmp(key, "lfeq")) { if (val > 0) s_lfeq = (float)val / 10.0f; }
        else if (!strcmp(key, "lcxu")) { float v = (float)val / 10.0f;
                                         if (v > 30.0f) v = 30.0f;
                                         if (v < -30.0f) v = -30.0f;
                                         s_lcx = v; }
        else if (!strcmp(key, "lcyu")) { float v = (float)val / 10.0f;
                                         if (v > 30.0f) v = 30.0f;
                                         if (v < -30.0f) v = -30.0f;
                                         s_lcy = v; }
        else if (!strcmp(key, "smooth")) { aim_smooth_set(val); }
        else if (!strcmp(key, "dead"))   { aim_dead_set(val); }
        else if (!strcmp(key, "beta"))   { aim_beta_set(val); }
        else if (!strcmp(key, "sens")) {
            if (s_sens_set && val >= 0 && val <= 2) {
                // A capture that spans a sensitivity change is two rigs
                // averaged into one: the same LEDs go from a 2x2 box to 12x3
                // between sens 1 and 2, and a histogram holding both reads as
                // "cannot be separated" when neither gain was measured. The
                // sink's own comment says to stop the capture first; nobody
                // does, and the tools' sensitivity row sits on the very screen
                // the capture runs from. So an ACTUAL change clears it and
                // says so. Only an actual change -- the tools re-send a value
                // on every keypress, and a same-value repeat must cost nothing.
                const int was = s_sens_get ? s_sens_get() : -1;
                s_sens_set(val);
                // The preset rewrote 0x06 under the loop: same path as the
                // pause menu, which restarts the search from the new preset.
                wiicam_aim_hw_dirty();
                if (was >= 0 && was != val && wl_enabled()) {
                    wl_reset();
                    reply("CAM: learn cleared -- sensitivity changed from %d "
                          "to %d, and a capture spanning both is not a "
                          "measurement of either\n", was, val);
                }
            }
        }
        // fmt is the canonical key (0 basic, 1 extended, 2 full); ext is the
        // name the tools used when there were only two formats and still
        // works, clamped the same way. Only bump the epoch on a real change:
        // every bump costs the camera poll a re-init with its settling delay.
        else if (!strcmp(key, "fmt") || !strcmp(key, "ext")) {
            // ext is clamped to its own documented range, not fmt's. It named
            // a two-value setting for the whole time it was the only name, and
            // a hand-typed 'ext:2' silently selecting a format we have never
            // seen work is not a reading anyone intended.
            const int hi = !strcmp(key, "ext") ? WIICAM_FMT_EXT : WIICAM_FMT_FULL;
            ext_set(val < 0 ? 0 : (val > hi ? hi : val));
        }
        // The mode byte for full mode. Refused outside the two candidates so a
        // typo cannot write a random value into the format register and leave
        // the sensor in a state no read length matches.
        else if (!strcmp(key, "fullreg")) {
            if (val == 0x05 || val == 0x55) {
                // One store with the format, so the poll can never act on a
                // new epoch holding the previous byte. fmt_set() skips the
                // epoch bump when nothing actually changed, which matters
                // here: a held-down key on the tools' two-value ladder re-sends
                // the same value several times a second, and every bump would
                // be another camera re-init with its settling delay.
                fmt_set(s_ext_state & 3, val == 0x05);
            } else {
                reply("CAM: fullreg must be 5 or 85 (0x05 or 0x55) -- not set\n");
            }
        }
        // Kept exactly as asked (0..15); the gate orders them when it runs, so
        // typing the two ends in either order gives the same window.
        //
        // Except for a window whose upper end is 0, which admits no size the
        // sensor can report and so rejects every blob. The give-back floor
        // keeps the gun aiming on two reconstructed corners, but badly -- and
        // now that the gate is PERSISTED, one fat-finger on a spinner that
        // starts at 0 followed by any calibration writes that to flash and it
        // comes back on every boot. Refused by name, the same way hwmax:0 is.
        else if (!strcmp(key, "bmin") || !strcmp(key, "bmax")) {
            const int v = val < 0 ? 0 : (val > 15 ? 15 : val);
            const uint8_t nmin = !strcmp(key, "bmin") ? (uint8_t)v : s_bmin;
            const uint8_t nmax = !strcmp(key, "bmin") ? s_bmax : (uint8_t)v;
            if ((nmin > nmax ? nmin : nmax) < 1)
                reply("CAM: a size window of 0..0 rejects every blob -- not set\n");
            else { s_bmin = nmin; s_bmax = nmax; }
        }
        // Size steps, so the whole useful range is 0..15 like the size itself.
        else if (!strcmp(key, "rtol")) { s_rtol = (uint8_t)(val < 0 ? 0 : (val > 15 ? 15 : val)); }
        // The shape gate. Refused below THIS RIG's own measured envelope
        // rather than clamped to it: a ceiling under what the gun has actually
        // watched its own LEDs produce is not a gate, it is an outage waiting
        // for the right angle. No number here comes from another rig -- a bar
        // with five LEDs per cluster makes blobs several times larger than one
        // with two, and a floor borrowed from the second refuses the only
        // workable setting for the first.
        else if (!strcmp(key, "pxmax")) {
            // Rig-derived, for the same reason as bhmax below. The floor here
            // used to be a flat 12, measured off a two-LED-per-corner bar --
            // and a five-per-cluster bar makes blobs several times larger, so
            // that floor refused the only workable setting for it. A pixel
            // count is not scale-free the way an aspect ratio pretends to be.
            const int v = val < 0 ? 0 : (val > 63 ? 63 : val);
            const int fl = rig_led_max_px();
            if (v && fl == RIG_PX_UNBOUNDED)
                reply("CAM: pxmax not set -- this rig's LED blobs are larger "
                      "than the pixel measurement can express, so no safe "
                      "ceiling can be derived for it. Use bhmax, which camfit "
                      "measures properly\n");
            else if (v && fl >= 0 && v < fl)
                reply("CAM: pxmax %d is below the largest LED this rig has been "
                      "measured at (%d px) -- not set\n", v, fl);
            else {
                s_pxmax = (uint8_t)v;
                if (v && (s_ext_state & 3) != WIICAM_FMT_FULL)
                    reply("CAM: pxmax %d set but INERT -- the shape gate needs "
                          "fmt:2 and this gun is in fmt:%d\n",
                          v, (int)(s_ext_state & 3));
            }
        }
        else if (!strcmp(key, "bhmax")) {
            // The floor is THIS RIG's tallest measured LED, not a number from
            // someone else's. It used to be a hard 8, taken from a bar with two
            // LEDs per corner -- and a bar with five per cluster produces blobs
            // several times taller, so that floor would have refused the only
            // sane setting for it while cheerfully accepting one that blinds
            // it. Whatever the last capture actually saw is the only defensible
            // bound, and with no capture there is nothing to defend, so the
            // value is taken and the tools carry the warning.
            const int v = val < 0 ? 0 : (val > 63 ? 63 : val);
            const int fl = rig_led_max_h();
            if (v && fl >= 0 && v < fl)
                reply("CAM: bhmax %d is below the tallest LED this rig has been "
                      "measured at (%d) -- not set\n", v, fl);
            else {
                s_bhmax = (uint8_t)v;
                // Set but not acting is the one state a user cannot see from
                // 'cam?', so it is said at the moment it is created rather
                // than left for them to wonder about.
                if (v && (s_ext_state & 3) != WIICAM_FMT_FULL)
                    reply("CAM: bhmax %d set but INERT -- the shape gate needs "
                          "fmt:2 and this gun is in fmt:%d\n",
                          v, (int)(s_ext_state & 3));
            }
        }
        else if (!strcmp(key, "armax")) {
            // DEPRECATED. It was measured at sensitivity 1, where LED blobs
            // came out round; sensitivity 2 is the default now and its
            // horizontal smear puts the median LED near 4:1, so the shape this
            // gate was built to recognise is not the shape the sensor produces
            // any more. It still loads and still applies, because a setting
            // already in a gun's flash must not change meaning under it, but
            // nothing suggests a value for it and camfit never sets it.
            const int v = val < 0 ? 0 : (val > 63 ? 63 : val);
            if (v && v < 16)
                reply("CAM: armax %d rejects blobs rounder than 2:1, which is "
                      "most measured LEDs -- not set\n", v);
            else {
                s_armax = (uint8_t)v;
                if (v) reply("CAM: armax is deprecated -- it was measured at "
                             "sensitivity 1 and the default is now 2, where "
                             "LED blobs are wide, not round. Use camfit\n");
            }
        }
        // The sensor's own thresholds. -1 leaves the register alone; anything
        // else is written to it, on the pump core, at the next hw tick.
        else if (!strcmp(key, "hwmax")) {
            // Zero in MAXSIZE tells the sensor to reject every blob. That is
            // not a tuning value, it is a dark gun, and it is what a typo
            // lands on -- so it is refused by name rather than obeyed.
            if (val == 0) reply("CAM: hwmax:0 refused -- it blinds the sensor\n");
            else {
                const int16_t nv = (int16_t)(val < 0 ? WIICAM_HW_LEAVE
                                             : (val > 255 ? 255 : val));
                // Same rule as sens: the sensor's own size threshold changes
                // what it reports, so a capture across the change is two rigs.
                if (nv != s_hwmax && wl_enabled()) {
                    wl_reset();
                    reply("CAM: learn cleared -- hwmax changed\n");
                }
                // A hand-set value is a manual override: the loop stops
                // driving the register rather than fighting the user for it.
                s_loop_on = false;
                s_loop_state = LOOP_OFF;
                if (nv >= 0) s_loop_val = nv;
                s_hwmax = nv; s_hw_dirty = true;
            } }
        else if (!strcmp(key, "hwmin")) {
            const int16_t nv = (int16_t)(val < 0 ? WIICAM_HW_LEAVE
                                         : (val > 255 ? 255 : val));
            if (nv != s_hwmin && wl_enabled()) {
                wl_reset();
                reply("CAM: learn cleared -- hwmin changed\n");
            }
            s_hwmin = nv; s_hw_dirty = true;
        }
        // The loop's own switch. On clears the bounds and adopts whatever the
        // register holds; off hands the register back to the sensitivity preset.
        else if (!strcmp(key, "loop")) {
            if (val) {
                loop_reset(s_hwmax >= 0 ? (int)s_hwmax : loop_preset());
                s_loop_on = true;
            } else {
                s_loop_on = false;
                s_loop_state = LOOP_OFF;
                s_loop_val = loop_preset();
                s_loop_saved = false;
                s_hwmax = WIICAM_HW_RESTORE; s_hw_dirty = true;
            }
        }
        else if (!strcmp(key, "mirx")) { s_mirx = (uint8_t)(val != 0); s_quad_reset_pending = true; }
        else if (!strcmp(key, "miry")) { s_miry = (uint8_t)(val != 0); s_quad_reset_pending = true; }
    }
    s_cache_seen = 0xFFFFFFFFu;    // settings changed: reprocess the next report
    reply("CMD ok (tune) | sens=%d lead=%d smooth=%d beta=%d "
          "lens=%u res=%u dash=%u ext=%u fmt=%u fullreg=%u "
          "bmin=%u bmax=%u rtol=%u bhmax=%u pxmax=%u armax=%u hwmax=%d hwmin=%d\n",
          s_sens_get ? s_sens_get() : -1, (int)s_lead_ms, aim_smooth_get(),
          aim_beta_get(),
          (unsigned)s_lens, (unsigned)s_res, (unsigned)s_dash,
          (unsigned)((s_ext_state & 3) >= WIICAM_FMT_EXT),
          (unsigned)(s_ext_state & 3), (unsigned)wiicam_aim_fullreg(),
          (unsigned)s_bmin, (unsigned)s_bmax, (unsigned)s_rtol,
          (unsigned)s_bhmax, (unsigned)s_pxmax, (unsigned)s_armax,
          (int)(s_hwmax < 0 ? -1 : s_hwmax),
          (int)(s_hwmin < 0 ? -1 : s_hwmin));
    return true;
}
