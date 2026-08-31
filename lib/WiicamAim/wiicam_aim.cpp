// wiicam_aim.cpp -- see header. All processing is in 240x176 space.
#include "wiicam_aim.h"
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

void wiicam_set_diag_hook(int (*fn)(void)) { s_diag = fn; }

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
    char b[320];        // cam? reaches ~210 with a fitted lens and the blob
                        // gate keys; 192 was too tight, 256 left no headroom
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (s_reply) s_reply(b); else fputs(b, stdout);
}

// ---- runtime state --------------------------------------------------------
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
// bit 0 is the flag, the rest is the epoch. The camera poll runs on the other
// core from the command parser, and reading a flag and an epoch as two
// separate values lets a poll pair a NEW epoch with the OLD flag -- it would
// then write the old format, latch the new epoch, and never correct itself.
// One word cannot tear that way.
static volatile int s_ext_state = 0;        // (epoch << 1) | wanted
static uint8_t  s_bmin = 0, s_bmax = 15;    // accepted blob size window
// Relative gate tolerance, in SIZE STEPS away from the consensus; 0 = off.
// Steps rather than a percentage because the reported size is a 4-bit number:
// a percentage of a median of 3 floored to the same single step for every
// setting from 1 to 66, so the knob was binary and most aggressive at its
// lowest non-zero value. See size_consensus_drop().
static uint8_t  s_rtol = 0;

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
static uint32_t s_breal[5] = {0, 0, 0, 0, 0};   // frames by corners actually SEEN
// Last frame's blobs, for '~camblob?': pipeline space, size, and whether the
// gate kept it. Evidence first -- a gate set from a guess is worth nothing.
static int      s_bn = 0;
static int16_t  s_bx[4], s_by[4];
static int8_t   s_bsz[4], s_bkeep[4];

int wiicam_aim_ext_state(void)  { return s_ext_state; }
int wiicam_aim_ext(void)        { return s_ext_state & 1; }
int wiicam_aim_ext_epoch(void)  { return s_ext_state >> 1; }

// One store, so a reader on the other core sees the flag and the epoch move
// together or not at all.
static void ext_set(int want)
{
    s_ext_state = (((s_ext_state >> 1) + 1) << 1) | (want ? 1 : 0);
}

void wiicam_aim_format_dirty(void)
{
    ext_set(s_ext_state & 1);
    // A rebuilt camera has been re-initialised from the sensitivity preset,
    // which writes register 0x06 -- so our MAXSIZE is gone and has to go back.
    s_hw_dirty = true;
}

// Called from the serial-pump core every loop: cheap when there is nothing to
// do, and the right place for the write when there is, because it can afford
// the sensor's settling delays and the camera poll cannot.
void wiicam_aim_hw_tick(void)
{
    if (!s_hw_dirty || !s_blobreg) return;
    // Both pump sites can run this -- core 1 in Run mode, core 0 when the gun
    // is paused or docked -- and during a mode change both can be in their
    // pump at once. Two masters clocking the same bus is how a bus locks up,
    // so the second one leaves and the request stays pending.
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

// Only the sensor thresholds, without disturbing the data format. Used by the
// firmware after anything that rewrites register 0x06 behind our back -- a
// sensitivity change from the pause menu or a profile switch does exactly that.
void wiicam_aim_hw_dirty(void) { s_hw_dirty = true; }

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
    quad_reset(0);                 // defaults are tuned for exactly this space
    int lead = 0;
    if (aim_lead_load(&lead)) s_lead_ms = (float)lead;
    int smooth = 0;
    if (aim_smooth_load(&smooth)) aim_smooth_set(smooth);
    int dead = 0;
    if (aim_dead_load(&dead)) aim_dead_set(dead);
    int beta = -1;
    if (aim_beta_load(&beta)) aim_beta_set(beta);
    int tmode = 0;
    if (aim_tmode_load(&tmode)) aim_tmode_set(tmode);
    int fk = 0, fp = 0;
    if (aim_fir_load(&fk, &fp)) aim_fir_set(fk, fp);
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

static void emit_q(uint64_t now_us, const float* xs, const float* ys, int n)
{
    if (s_dash != 2 || !s_line) return;
    if (s_dash_min_dt_us && (now_us - s_dash_last_us) < s_dash_min_dt_us) return;
    s_dash_last_us = now_us;
    char b[128];
    int o = snprintf(b, sizeof(b), "Q,%lu,%d",
                     (unsigned long)(now_us / 1000ull), n);
    for (int i = 0; i < n && o > 0 && o < (int)sizeof(b) - 14; ++i)
        o += snprintf(b + o, sizeof(b) - o, ",%d,%d",
                      (int)lroundf(xs[i] * 10.0f), (int)lroundf(ys[i] * 10.0f));
    if (o > 0 && o < (int)sizeof(b) - 2) { b[o++] = '\n'; b[o] = 0; s_line(b); }
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
    int n = 0;
    for (int i = 0; i < an; ++i) {
        s_bx[i]   = (int16_t)lroundf(ax[i]);
        s_by[i]   = (int16_t)lroundf(ay[i]);
        s_bsz[i]  = (int8_t)asz[i];
        s_bkeep[i] = (int8_t)keep[i];
        if (!keep[i]) continue;              // already tallied by its own gate
        xs[n] = ax[i]; ys[n] = ay[i]; ++n;
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
        emit_q(now_us, xs, ys, n);
        return false;
    }

    QuadResult r = quad_update(xs, ys, n);
    // How many corners were really SEEN this frame is the honest measure of
    // how much the light is costing: a run that sits on 3 is a run where one
    // LED is being taken from us every frame.
    {
        int nr = r.n_real;
        if (nr < 0) nr = 0;
        if (nr > 4) nr = 4;
        ++s_breal[nr];
    }
    if (r.count < 4) { ++s_bdrop; emit_q(now_us, xs, ys, n); return false; }

    // Latency lead: extrapolate the published quad along its velocity,
    // clamped; never fed back into the resolver.
    float qx[4], qy[4];
    float lx = 0.0f, ly = 0.0f;
    // Temporal mode 1 folds the lead into the pipeline's own FIR, so publish
    // the quad unled there; both numbers it needs come from here.
    aim_lead_note(s_lead_ms);
    aim_conf_note((float)r.n_real * 0.25f);
    // Mode 1 folds the lead into the FIR inside aim_runtime_solve, which is
    // only reached with a calibration loaded. Uncalibrated, the lead must stay
    // here or it disappears with nothing replacing it.
    if ((aim_tmode_get() == 0 || !aim_runtime_active())
        && s_lead_ms > 0.0f && dt > 1e-4f) {
        const float frames = (s_lead_ms * 1e-3f) / dt;
        lx = r.vx * frames; ly = r.vy * frames;
        const float m = sqrtf(lx*lx + ly*ly);
        if (m > WIICAM_LEAD_PX_MAX) { lx *= WIICAM_LEAD_PX_MAX/m; ly *= WIICAM_LEAD_PX_MAX/m; }
    }
    for (int i = 0; i < 4; ++i) { qx[i] = r.p[i].x + lx; qy[i] = r.p[i].y + ly; }
    emit_q(now_us, qx, qy, 4);

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
        ok = aim_tmode_store(aim_tmode_get()) && ok;
        ok = aim_fir_store(aim_fir_k(), aim_fir_pct()) && ok;
        ok = (fx_store() != 0) && ok;       // recoil knobs ride the same save
        if (s_sens_save) s_sens_save();     // sens lives in OpenFIRE's profile
        aim_lens_t ls = { (int)s_lens, s_lk1, s_lk2, s_lfpx, s_lfeq,
                          s_lcx, s_lcy };
        bool lens_ok = true;
        if (ls.model == 0) aim_lens_clear();
        else               lens_ok = aim_lens_store(&ls);
        // beta rides in the reply so a tool can VERIFY what was written rather
        // than assume it; cam? reports the live value, this reports the stored one.
        reply(ok && lens_ok
              ? "CAM: saved lead=%dms smooth=%d dead=%d beta=%d lens=%d tmode=%d firk=%d firpct=%d (sens lives in the OpenFIRE profile; blob gates and sensor thresholds are NOT saved)\n"
              : "CAM: SAVE FAILED lead=%dms smooth=%d dead=%d beta=%d lens=%d tmode=%d firk=%d firpct=%d\n",
              (int)s_lead_ms, aim_smooth_get(), aim_dead_get(), aim_beta_get(),
              (int)s_lens, aim_tmode_get(), aim_fir_k(), aim_fir_pct());
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
        reply("CAM: blob ext=%u bmin=%u bmax=%u rtol=%u hwmax=%d hwmin=%d "
              "bn=%d brej=%lu brrej=%lu bvalve=%lu bframes=%lu bms=%lu "
              "bdrop=%lu br4=%lu br3=%lu br2=%lu br1=%lu br0=%lu\n",
              (unsigned)(s_ext_state & 1), (unsigned)s_bmin, (unsigned)s_bmax,
              (unsigned)s_rtol, (int)(s_hwmax < 0 ? -1 : s_hwmax),
              (int)(s_hwmin < 0 ? -1 : s_hwmin), s_bn,
              (unsigned long)s_brej, (unsigned long)s_brrej,
              (unsigned long)s_bvalve, (unsigned long)s_bframes,
              (unsigned long)(fx_now() / 1000ull),
              (unsigned long)s_bdrop, (unsigned long)s_breal[4],
              (unsigned long)s_breal[3], (unsigned long)s_breal[2],
              (unsigned long)s_breal[1], (unsigned long)s_breal[0]);
        char b[128];
        int o = snprintf(b, sizeof(b), "CAM: blobs");
        for (int i = 0; i < s_bn && o > 0 && o < (int)sizeof(b) - 20; ++i)
            o += snprintf(b + o, sizeof(b) - o, " %d,%d,%d,%d",
                          (int)s_bx[i], (int)s_by[i], (int)s_bsz[i],
                          (int)s_bkeep[i]);
        if (!(s_ext_state & 1) && o > 0 && o < (int)sizeof(b) - 34)
            o += snprintf(b + o, sizeof(b) - o, " (sizes need ext:1)");
        reply("%s\n", b);
        return true;
    }
    if (!strncmp(line, "camreset", 8)) {
        aim_lens_clear();
        s_lens = 0; s_lead_ms = 0.0f;
        s_lcx = 0.0f; s_lcy = 0.0f;
        // The blob gate goes back to inert here too: it is the one setting
        // that can stop a gun aiming if it is set wrong, so the command a user
        // reaches for when nothing works must undo it.
        if (s_ext_state & 1) ext_set(0);
        s_bmin = 0; s_bmax = 15;
        s_rtol = 0;
        // RESTORE, not "leave alone". Marking them untouched left whatever we
        // had written in the sensor -- so the one command a user reaches for
        // when the gun has gone dark could not undo the one setting able to
        // make it go dark. MAXSIZE comes back from the sensitivity preset;
        // MINSIZE goes to 0, the direction that cannot cost an LED.
        s_hwmax = WIICAM_HW_RESTORE; s_hwmin = WIICAM_HW_RESTORE;
        s_hw_dirty = true;
        reply("CAM: lens + lead cleared, blob gates off, sensor thresholds "
              "restored\n");
        return true;
    }
    if (!strncmp(line, "cam?", 4)) {
        reply("CAM: board=rp2040-wiicam sens=%d mirx=%d miry=%d lead=%d "
              "smooth=%d dead=%d lens=%d lk1u=%d lk2u=%d lfpx=%d lfeq=%d "
              "lcxu=%d lcyu=%d beta=%d tmode=%d firk=%d firpct=%d res=%u dash=%u "
              "ext=%u bmin=%u bmax=%u rtol=%u hwmax=%d hwmin=%d\n",
              s_sens_get ? s_sens_get() : -1, (int)s_mirx, (int)s_miry,
              (int)s_lead_ms, aim_smooth_get(), aim_dead_get(),
              (int)s_lens, (int)(s_lk1*1e6f), (int)(s_lk2*1e6f),
              (int)(s_lfpx*10.0f), (int)(s_lfeq*10.0f),
              (int)(s_lcx*10.0f), (int)(s_lcy*10.0f),
              aim_beta_get(), aim_tmode_get(), aim_fir_k(), aim_fir_pct(),
              (unsigned)s_res, (unsigned)s_dash,
              (unsigned)(s_ext_state & 1), (unsigned)s_bmin, (unsigned)s_bmax,
              (unsigned)s_rtol, (int)(s_hwmax < 0 ? -1 : s_hwmax),
              (int)(s_hwmin < 0 ? -1 : s_hwmin));
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
        if      (!strcmp(key, "res"))  { s_res = (uint8_t)(val ? 2 : 0); if (s_res) quad_reset(0); }
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
        else if (!strcmp(key, "tmode"))  { aim_tmode_set(val); }
        else if (!strcmp(key, "firk"))   { aim_fir_set(val, aim_fir_pct()); }
        else if (!strcmp(key, "firpct")) { aim_fir_set(aim_fir_k(), val); }
        else if (!strcmp(key, "sens")) { if (s_sens_set && val >= 0 && val <= 2) s_sens_set(val); }
        else if (!strcmp(key, "ext"))  { const int v = val ? 1 : 0;
                                         if (v != (s_ext_state & 1)) ext_set(v); }
        // Kept exactly as asked (0..15); the gate orders them when it runs, so
        // typing the two ends in either order gives the same window.
        else if (!strcmp(key, "bmin")) { s_bmin = (uint8_t)(val < 0 ? 0 : (val > 15 ? 15 : val)); }
        else if (!strcmp(key, "bmax")) { s_bmax = (uint8_t)(val < 0 ? 0 : (val > 15 ? 15 : val)); }
        // Size steps, so the whole useful range is 0..15 like the size itself.
        else if (!strcmp(key, "rtol")) { s_rtol = (uint8_t)(val < 0 ? 0 : (val > 15 ? 15 : val)); }
        // The sensor's own thresholds. -1 leaves the register alone; anything
        // else is written to it, on the pump core, at the next hw tick.
        else if (!strcmp(key, "hwmax")) {
            // Zero in MAXSIZE tells the sensor to reject every blob. That is
            // not a tuning value, it is a dark gun, and it is what a typo
            // lands on -- so it is refused by name rather than obeyed.
            if (val == 0) reply("CAM: hwmax:0 refused -- it blinds the sensor\n");
            else { s_hwmax = (int16_t)(val < 0 ? WIICAM_HW_LEAVE
                                       : (val > 255 ? 255 : val));
                   s_hw_dirty = true; } }
        else if (!strcmp(key, "hwmin")) { s_hwmin = (int16_t)(val < 0 ? WIICAM_HW_LEAVE
                                                     : (val > 255 ? 255 : val));
                                          s_hw_dirty = true; }
        else if (!strcmp(key, "mirx")) { s_mirx = (uint8_t)(val != 0); quad_reset(0); }
        else if (!strcmp(key, "miry")) { s_miry = (uint8_t)(val != 0); quad_reset(0); }
    }
    s_cache_seen = 0xFFFFFFFFu;    // settings changed: reprocess the next report
    reply("CMD ok (tune) | sens=%d lead=%d smooth=%d beta=%d tmode=%d firk=%d "
          "firpct=%d lens=%u res=%u dash=%u ext=%u bmin=%u bmax=%u "
          "rtol=%u hwmax=%d hwmin=%d\n",
          s_sens_get ? s_sens_get() : -1, (int)s_lead_ms, aim_smooth_get(),
          aim_beta_get(), aim_tmode_get(), aim_fir_k(), aim_fir_pct(),
          (unsigned)s_lens, (unsigned)s_res, (unsigned)s_dash,
          (unsigned)(s_ext_state & 1), (unsigned)s_bmin, (unsigned)s_bmax,
          (unsigned)s_rtol, (int)(s_hwmax < 0 ? -1 : s_hwmax),
          (int)(s_hwmin < 0 ? -1 : s_hwmin));
    return true;
}
