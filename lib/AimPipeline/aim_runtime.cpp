#include "aim_runtime.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

// AIM_HAVE_STORE: real NVS on ESP32, the RP2040 LittleFS shim when the build
// defines AIM_NVS_SHIM, no persistence otherwise. One switch, so a new board
// only has to provide the four headers, not edit this file.
#if defined(ESP_PLATFORM) || defined(AIM_NVS_SHIM)
  #define AIM_HAVE_STORE 1
#endif

#if defined(AIM_HAVE_STORE)
  #include "nvs.h"
  #include "nvs_flash.h"
  // Must be the real header: a hand-written extern here gets C++ linkage and
  // fails to link.
  #include "esp_timer.h"
  #include "esp_system.h"   // esp_reset_reason(), for the boot forensics
  #define AIM_NVS_NS   "aimcal"
  #define AIM_NVS_KEY  "c0"
  #define AIM_NVS_CAM  "cam0"
  #define AIM_NVS_LEAD "lead0"
  #define AIM_NVS_LENS "lens0"
  #define AIM_NVS_BOOT "boots"
#endif

// Which hardware this pipeline fronts; the desktop tools read it from ~ping.
#ifndef AIM_BOARD
#define AIM_BOARD "esp32s3-ov2640"
#endif

// ---- reply output --------------------------------------------------------
static void (*s_out)(const char*) = 0;

// Installs the reply sink; 0 means stdout.
void aim_set_out(void (*fn)(const char*)) { s_out = fn; }

// Formats a reply and sends it to the installed sink, so the answer comes back
// on the same channel the question arrived on.
static void aim_out(const char* fmt, ...)
{
    char b[224];
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (s_out) s_out(b);
    else       fputs(b, stdout);
}

static aim_calib_t s_c;                 // the one active calibration
static bool        s_enabled = true;
static bool        s_capture = false;   // emit T,<ms> trigger markers
static bool        s_trig_prev = false;

// ---- One Euro filter state ----------------------------------------------
// The derivative is in normalised screen widths per second, so beta is order 10.
#ifndef AIM_FILT_MIN_CUTOFF
#define AIM_FILT_MIN_CUTOFF 3.5f   // == SMOOTH_TAB level 3, the default level
#endif
#ifndef AIM_FILT_BETA
#define AIM_FILT_BETA 15.0f
#endif
#define AIM_FILT_DCUTOFF 1.0f
#define AIM_NOMINAL_DT   (1.0f/135.0f)

static float s_fc = AIM_FILT_MIN_CUTOFF;
static float s_beta = AIM_FILT_BETA;
static bool  s_f_have = false;
static float s_f_x[2] = {0,0};      // filtered value, per axis
static float s_f_dx[2] = {0,0};     // filtered derivative, per axis

// One Euro smoothing factor for a cutoff in Hz and a timestep in seconds.
static inline float oe_alpha(float cutoff, float dt)
{
    // tau = 1/(2*pi*fc); alpha = 1/(1 + tau/dt)
    const float tau = 1.0f / (6.28318531f * cutoff);
    return 1.0f / (1.0f + tau / dt);
}

// Sets the filter coefficients and resets its history.
void  aim_filter_set(float min_cutoff, float beta)
{
    s_fc = min_cutoff; s_beta = beta; s_f_have = false;
}
// Discards the filter history.
static void fir_reset(void);           // defined with the FIR state, below
void  aim_filter_reset(void) { s_f_have = false; fir_reset(); }
float aim_filter_min_cutoff(void) { return s_fc; }
float aim_filter_beta(void) { return s_beta; }

// ---- smoothing level ------------------------------------------------------
// One knob over the One Euro pair. min_cutoff sets the at-rest lag (tau =
// 1/(2*pi*fc)); beta is flat and high so fast motion is never filtered at any
// level. Level 10 is the pre-retune default pair, so the old feel stays
// reachable. Rest lag per level, in ms:
//   L1 20  L2 32  L3 45  L4 64  L5 76  L6 88  L7 103  L8 118  L9 138  L10 159
#define AIM_NVS_SMOOTH "smth0"
static const float SMOOTH_TAB[11][2] = {
    {0.00f,  0.0f},   // 0: filter off
    {8.00f, 15.0f},   // 1: lightest
    {5.00f, 15.0f},
    {3.50f, 15.0f},   // 3: the default
    {2.50f, 15.0f},
    {2.10f, 15.0f},
    {1.80f, 15.0f},
    {1.55f, 15.0f},
    {1.35f, 15.0f},
    {1.15f, 15.0f},
    {1.00f, 15.0f},   // 10: heaviest -- the pre-retune default pair
};
static int s_smooth = 3;

// Speed sensitivity of the smoothing, overriding the table's beta.
// -1 means "use the table", which is 15 at every level.
#define AIM_NVS_BETA "beta0"
#define AIM_BETA_MAX 60
static int s_beta_ovr = -1;

// The beta actually in force for a level.
static float beta_for(int level)
{
    return (s_beta_ovr >= 0) ? (float)s_beta_ovr : SMOOTH_TAB[level][1];
}

// Clamps, applies the mapped coefficients, and remembers the level.
void aim_smooth_set(int level)
{
    if (level < 0) level = 0;
    if (level > 10) level = 10;
    s_smooth = level;
    aim_filter_set(SMOOTH_TAB[level][0], beta_for(level));
}

// One Euro's cutoff is min_cutoff + beta*speed, so `smooth` sets how heavy the
// filter is AT REST and beta sets how quickly it lets go once the gun moves.
// Raising beta shortens the trail on a swipe without touching rest stability.
// -1 restores the table value.
void aim_beta_set(int beta)
{
    if (beta > AIM_BETA_MAX) beta = AIM_BETA_MAX;
    s_beta_ovr = (beta < 0) ? -1 : beta;
    aim_filter_set(SMOOTH_TAB[s_smooth][0], beta_for(s_smooth));
}
int aim_beta_get(void) { return s_beta_ovr; }

// Persists the beta override; -1 is stored as "follow the table".
bool aim_beta_store(int beta)
{
    if (beta > AIM_BETA_MAX) beta = AIM_BETA_MAX;
    if (beta < 0) beta = -1;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_i16(h, AIM_NVS_BETA, (int16_t)beta) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)beta; return true;
#endif
}

// Reads the persisted beta override; false if nothing is stored.
bool aim_beta_load(int* out_beta)
{
#if defined(AIM_HAVE_STORE)
    if (!out_beta) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    int16_t v = 0;
    const esp_err_t e = nvs_get_i16(h, AIM_NVS_BETA, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    if (v > AIM_BETA_MAX) v = AIM_BETA_MAX;
    *out_beta = (v < 0) ? -1 : v;
    return true;
#else
    (void)out_beta; return false;
#endif
}
int aim_smooth_get(void) { return s_smooth; }

// Persists the smoothing level, clamped to 0..10.
bool aim_smooth_store(int level)
{
    if (level < 0) level = 0;
    if (level > 10) level = 10;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_i16(h, AIM_NVS_SMOOTH, (int16_t)level) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)level; return true;
#endif
}

// Reads the persisted smoothing level; false if nothing is stored.
bool aim_smooth_load(int* out_level)
{
#if defined(AIM_HAVE_STORE)
    if (!out_level) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    int16_t v = 0;
    const esp_err_t e = nvs_get_i16(h, AIM_NVS_SMOOTH, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    *out_level = (int)v;
    return true;
#else
    (void)out_level; return false;
#endif
}

// ---- output dead-band -----------------------------------------------------
// Swallows sub-threshold shimmer around the last SENT position; motion at or
// above the threshold passes immediately, so it never adds delay. Units are
// the final output units at the move call. 0 = off, the default.
#define AIM_NVS_DEAD "dead0"
static int s_dead = 0;
static int s_dead_x = -1000000, s_dead_y = -1000000;

// Clamps and applies the threshold.
void aim_dead_set(int units)
{
    if (units < 0) units = 0;
    if (units > 2000) units = 2000;
    s_dead = units;
}
int aim_dead_get(void) { return s_dead; }

// True when this position should be sent; updates the reference on send.
bool aim_dead_pass(int x, int y)
{
    const int dx = x - s_dead_x, dy = y - s_dead_y;
    if (s_dead > 0 &&
        (long long)dx * dx + (long long)dy * dy < (long long)s_dead * s_dead)
        return false;
    s_dead_x = x; s_dead_y = y;
    return true;
}

// Persists the dead-band threshold under its own key.
bool aim_dead_store(int units)
{
    if (units < 0) units = 0;
    if (units > 2000) units = 2000;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_i16(h, AIM_NVS_DEAD, (int16_t)units) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)units; return true;
#endif
}

// Reads the persisted threshold; false if nothing is stored.
bool aim_dead_load(int* out_units)
{
#if defined(AIM_HAVE_STORE)
    if (!out_units) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    int16_t v = 0;
    const esp_err_t e = nvs_get_i16(h, AIM_NVS_DEAD, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    *out_units = (int)v;
    return true;
#else
    (void)out_units; return false;
#endif
}

// ---- wiicam blob gate ------------------------------------------------------
// One packed word rather than four keys: the four values are meaningless apart
// and a partial read is the failure that hurts (see the header). The tag makes
// a stale or foreign key read as "nothing stored" instead of as gate 0,0,0,0,
// which would silently switch the size window off on a gun that had one.
#define AIM_NVS_GATE "gate0"
#define AIM_NVS_GATE2 "gate1"
#define AIM_NVS_FIT  "fit0"
#define AIM_GATE_TAG 0x6A000000u

bool aim_gate_store(int fmt, int bmin, int bmax, int rtol)
{
    if (fmt  < 0)  fmt  = 0;
    if (fmt  > 2)  fmt  = 2;
    if (bmin < 0)  bmin = 0;
    if (bmin > 15) bmin = 15;
    if (bmax < 0)  bmax = 0;
    if (bmax > 15) bmax = 15;
    if (rtol < 0)  rtol = 0;
    if (rtol > 15) rtol = 15;
    // u32, not i32: neither the RP2040 LittleFS shim nor the host stub carries
    // the i32 pair, and a key written through an API that does not exist is a
    // link error on the one board this runs on.
    const uint32_t v = (uint32_t)AIM_GATE_TAG | ((uint32_t)fmt << 12)
                     | ((uint32_t)bmin << 8) | ((uint32_t)bmax << 4)
                     | (uint32_t)rtol;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_u32(h, AIM_NVS_GATE, v) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)v; return true;
#endif
}

bool aim_gate_load(int* out_fmt, int* out_bmin, int* out_bmax, int* out_rtol)
{
#if defined(AIM_HAVE_STORE)
    if (!out_fmt || !out_bmin || !out_bmax || !out_rtol) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint32_t v = 0;
    const esp_err_t e = nvs_get_u32(h, AIM_NVS_GATE, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    if ((v & 0xFF000000u) != (uint32_t)AIM_GATE_TAG) return false;
    const int fmt = (int)((v >> 12) & 0x3);
    *out_fmt  = (fmt > 2) ? 2 : fmt;
    *out_bmin = (int)((v >> 8) & 0xF);
    *out_bmax = (int)((v >> 4) & 0xF);
    *out_rtol = (int)(v & 0xF);
    return true;
#else
    (void)out_fmt; (void)out_bmin; (void)out_bmax; (void)out_rtol; return false;
#endif
}

// Erasing matters as much as writing. A saved gate that stops the gun aiming
// would otherwise come back on the next boot, and '~camreset' -- the command a
// user reaches for when nothing works -- would fix the session and lose the
// argument. Absent key counts as cleared.
bool aim_gate_clear(void)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const esp_err_t e = nvs_erase_key(h, AIM_NVS_GATE);
    const bool ok = (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    return true;
#endif
}

bool aim_gate2_store(int pxmax, int armax, int bhmax)
{
    if (pxmax < 0)  pxmax = 0;
    if (pxmax > 63) pxmax = 63;
    if (armax < 0)  armax = 0;
    if (armax > 63) armax = 63;
    if (bhmax < 0)  bhmax = 0;
    if (bhmax > 63) bhmax = 63;
    const uint32_t v = (uint32_t)AIM_GATE_TAG | ((uint32_t)bhmax << 12)
                     | ((uint32_t)pxmax << 6) | (uint32_t)armax;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_u32(h, AIM_NVS_GATE2, v) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)v; return true;
#endif
}

bool aim_gate2_load(int* out_pxmax, int* out_armax, int* out_bhmax)
{
#if defined(AIM_HAVE_STORE)
    if (!out_pxmax || !out_armax || !out_bhmax) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint32_t v = 0;
    const esp_err_t e = nvs_get_u32(h, AIM_NVS_GATE2, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    if ((v & 0xFF000000u) != (uint32_t)AIM_GATE_TAG) return false;
    *out_pxmax = (int)((v >> 6) & 0x3F);
    *out_armax = (int)(v & 0x3F);
    *out_bhmax = (int)((v >> 12) & 0x3F);
    return true;
#else
    (void)out_pxmax; (void)out_armax; (void)out_bhmax; return false;
#endif
}

bool aim_gate2_clear(void)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const esp_err_t e = nvs_erase_key(h, AIM_NVS_GATE2);
    const bool ok = (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    return true;
#endif
}

// ---- provenance ------------------------------------------------------------
// The two numbers the shape ceiling was derived from. Deliberately in their own
// key: a gun whose flash predates this build has no 'fit0', and the load has to
// report that plainly rather than hand back a pair of zeroes that would read as
// "LEDs measured 0 tall, stray starts at 0" -- a rig on which no gate could
// ever work. Its own key also means a bad write here can never cost the gate
// itself, which is the setting that actually matters.
bool aim_fit_store(int led_max_h, int stray_min_h, int led_max_px)
{
    if (led_max_h < 0)    led_max_h = 0;
    if (led_max_h > 63)   led_max_h = 63;
    if (stray_min_h < 0)  stray_min_h = 0;
    if (stray_min_h > 63) stray_min_h = 63;
    if (led_max_px < 0)   led_max_px = 0;
    if (led_max_px > 63)  led_max_px = 63;
    const uint32_t v = (uint32_t)AIM_GATE_TAG | ((uint32_t)led_max_px << 12)
                     | ((uint32_t)led_max_h << 6) | (uint32_t)stray_min_h;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_u32(h, AIM_NVS_FIT, v) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)v; return true;
#endif
}

bool aim_fit_load(int* out_led_max_h, int* out_stray_min_h, int* out_led_max_px)
{
#if defined(AIM_HAVE_STORE)
    if (!out_led_max_h || !out_stray_min_h || !out_led_max_px) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint32_t v = 0;
    const esp_err_t e = nvs_get_u32(h, AIM_NVS_FIT, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    if ((v & 0xFF000000u) != (uint32_t)AIM_GATE_TAG) return false;
    *out_led_max_h   = (int)((v >> 6) & 0x3F);
    *out_stray_min_h = (int)(v & 0x3F);
    *out_led_max_px  = (int)((v >> 12) & 0x3F);
    return true;
#else
    (void)out_led_max_h; (void)out_stray_min_h; (void)out_led_max_px;
    return false;
#endif
}

bool aim_fit_clear(void)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const esp_err_t e = nvs_erase_key(h, AIM_NVS_FIT);
    const bool ok = (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    return true;
#endif
}

// ---- temporal mode ---------------------------------------------------------
// Mode 0 is the shipped pair: One Euro here, latency lead on the quad in the
// capture layer. Mode 1 replaces both with one causal least-squares fit.
#define AIM_NVS_TMODE "tmod0"
#define AIM_NVS_FIR   "fir0"
#define AIM_FIR_KMIN  3
#define AIM_FIR_KMAX  15
#define AIM_FIR_MAX_STEP 0.35f   // normalised screen units, glitch backstop
static int   s_tmode   = 0;
static int   s_fir_k   = 7;      // window in samples
static int   s_fir_pct = 100;    // horizon as a percentage of the lead
// Written by the capture task, read by the solve. Scalar float writes, like
// the confidence the capture layer already keeps for the dashboard.
static volatile float s_lead_ms = 0.0f;
static volatile float s_conf    = 1.0f;
static float s_fir_x[AIM_FIR_KMAX], s_fir_y[AIM_FIR_KMAX];
static int   s_fir_n = 0;        // samples held; index 0 is the OLDEST
static float s_fir_dt = 0.0f;   // learned sample spacing, for gap detection

static void fir_reset(void) { s_fir_n = 0; s_fir_dt = 0.0f; }

// Ceiling matches aim_lead_store's, so a foreign or corrupt stored lead cannot
// hand the FIR a horizon no capture layer would ever apply itself.
void aim_lead_note(float ms)
{
    if (!(ms > 0.0f)) ms = 0.0f;
    if (ms > 50.0f)   ms = 50.0f;
    s_lead_ms = ms;
}
void aim_conf_note(float conf) { s_conf = conf; }

int  aim_tmode_get(void) { return s_tmode; }

// Mode 1 lost to mode 0 on hardware: a fixed-window fit cannot reproduce One
// Euro's speed-adaptive cutoff, which is the property that makes the shipped
// filter feel right. The code and its tests stay, because a measured negative
// result is worth keeping, but selecting it needs -D AIM_FIR_MODE. A gun with
// tmode=1 left in NVS from testing falls back to 0 on the next boot.
void aim_tmode_set(int mode)
{
#if defined(AIM_FIR_MODE)
    const int m = (mode == 1) ? 1 : 0;
#else
    const int m = 0;
    (void)mode;
#endif
    if (m != s_tmode) { s_tmode = m; fir_reset(); s_f_have = false; }
}

int aim_fir_k(void)   { return s_fir_k; }
int aim_fir_pct(void) { return s_fir_pct; }

// Clamps and applies the FIR shape. Changing the window invalidates the
// buffer: a shorter window would otherwise fit over samples left from the
// longer one.
void aim_fir_set(int k, int pct)
{
    if (k < AIM_FIR_KMIN) k = AIM_FIR_KMIN;
    if (k > AIM_FIR_KMAX) k = AIM_FIR_KMAX;
    if (pct < 0)   pct = 0;
    if (pct > 140) pct = 140;
    s_fir_k = k; s_fir_pct = pct;
    fir_reset();          // any shape change invalidates the window it fitted
}

// Weights of a causal degree-1 least-squares fit over k samples, evaluated tf
// frames past the newest. Closed form, so the horizon can be a live parameter:
//   w_i = 1/k + (i - (k-1)/2) * (tf + (k-1)/2) * 12/(k^3 - k)
// index 0 = oldest. They sum to 1 by construction, so a static aim cannot
// drift, and a constant-velocity ramp is extrapolated exactly.
static void fir_weights(int k, float tf, float* w)
{
    if (k < 2) { for (int i = 0; i < k; ++i) w[i] = 1.0f; return; }  // k^3-k = 0
    const float kf   = (float)k;
    const float half = 0.5f * (kf - 1.0f);
    const float g    = (tf + half) * 12.0f / (kf * kf * kf - kf);
    for (int i = 0; i < k; ++i) w[i] = 1.0f / kf + ((float)i - half) * g;
}

// One fit does the smoothing and the prediction. Replaces One Euro AND the
// capture-side lead when mode 1 is selected.
static void fir_filter(float* x, float* y, float dt)
{
    // !(dt > 0) also catches NaN, which fails every ordinary comparison.
    if (!(dt > 0.0f) || dt > 0.25f) dt = AIM_NOMINAL_DT;  // stall or first frame
    if (!(*x == *x) || !(*y == *y)) return;               // never buffer a NaN
    int k = s_fir_k;
    if (k < AIM_FIR_KMIN) k = AIM_FIR_KMIN;
    if (k > AIM_FIR_KMAX) k = AIM_FIR_KMAX;
    if (s_fir_n > k) s_fir_n = 0;                        // window was shortened

    // The fit assumes evenly spaced samples, so a gap invalidates it: losing
    // lock and re-acquiring elsewhere would otherwise be fitted as one huge
    // velocity and fling the cursor off-screen for k frames. The expected
    // spacing is learned rather than assumed, so this works at any frame rate.
    if (s_fir_dt <= 0.0f) s_fir_dt = dt;
    if (dt > 2.5f * s_fir_dt) s_fir_n = 0;
    s_fir_dt += 0.05f * (dt - s_fir_dt);

    if (s_fir_n < k) {
        s_fir_x[s_fir_n] = *x; s_fir_y[s_fir_n] = *y;
        ++s_fir_n;
    } else {
        for (int i = 1; i < k; ++i) {
            s_fir_x[i-1] = s_fir_x[i]; s_fir_y[i-1] = s_fir_y[i];
        }
        s_fir_x[k-1] = *x; s_fir_y[k-1] = *y;
    }
    // Until the window is full there is nothing to fit. Passing the sample
    // through beats fitting two points, which would extrapolate wildly at the
    // exact moment the pipeline comes up.
    if (s_fir_n < k) return;

    // The capture layer reports the fraction of corners actually MEASURED this
    // frame, not the resolver's miss-damped confidence: a wide lens can leave
    // one LED dim for a long run, and the damped figure charged that against
    // the lead for as long as it lasted. Three measured corners already pin
    // the velocity, so only a genuine dropout shortens the horizon.
    float conf = s_conf;
    if (!(conf > 0.0f)) conf = 0.0f;                     // also catches NaN
    conf *= (1.0f / 0.75f);                              // 3 of 4 = full lead
    if (conf > 1.0f) conf = 1.0f;
    float tf = ((float)s_lead_ms * 1e-3f / dt)
             * ((float)s_fir_pct * 0.01f) * conf;
    if (!(tf > 0.0f)) tf = 0.0f;
    // Horizon ceiling: the window's own span. Never extrapolate further than
    // the samples the fit can actually see. A tighter ceiling than this makes
    // the prediction knob depend on the smoothing knob, which is precisely the
    // coupling this mode exists to remove -- at half a span, a 30 ms lead was
    // silently clipped to 22 and could never match mode 0.
    const float tf_max = (float)(k - 1);
    if (tf > tf_max) tf = tf_max;

    float w[AIM_FIR_KMAX];
    fir_weights(k, tf, w);
    float ax = 0.0f, ay = 0.0f, lox = s_fir_x[0], hix = s_fir_x[0];
    float loy = s_fir_y[0], hiy = s_fir_y[0];
    for (int i = 0; i < k; ++i) {
        ax += w[i] * s_fir_x[i];
        ay += w[i] * s_fir_y[i];
        if (s_fir_x[i] < lox) lox = s_fir_x[i];
        if (s_fir_x[i] > hix) hix = s_fir_x[i];
        if (s_fir_y[i] < loy) loy = s_fir_y[i];
        if (s_fir_y[i] > hiy) hiy = s_fir_y[i];
    }
    // Excursion cap, the counterpart of mode 0's LEAD_PX_MAX. A line fitted to
    // a noisy window can leave the range of its own inputs; let it leave only
    // in proportion to how far ahead it is projecting. At rest the window
    // spans only noise, so the cap is tight; in motion it spans real travel,
    // so real prediction passes untouched.
    const float g = tf / (float)(k - 1);
    const float mx = (hix - lox) * g, my = (hiy - loy) * g;
    if (ax < lox - mx) ax = lox - mx; else if (ax > hix + mx) ax = hix + mx;
    if (ay < loy - my) ay = loy - my; else if (ay > hiy + my) ay = hiy + my;
    // Absolute backstop, the counterpart of mode 0's LEAD_PX_MAX. A third of
    // the screen is far past any legitimate prediction (a 4 m/s swipe leads
    // 0.10 screen widths at 30 ms) but bounds a glitch to something survivable.
    const float nx = s_fir_x[k-1], ny = s_fir_y[k-1];
    if (ax > nx + AIM_FIR_MAX_STEP) ax = nx + AIM_FIR_MAX_STEP;
    else if (ax < nx - AIM_FIR_MAX_STEP) ax = nx - AIM_FIR_MAX_STEP;
    if (ay > ny + AIM_FIR_MAX_STEP) ay = ny + AIM_FIR_MAX_STEP;
    else if (ay < ny - AIM_FIR_MAX_STEP) ay = ny - AIM_FIR_MAX_STEP;
    *x = ax; *y = ay;
}

// Persists the temporal mode, clamped to 0..1.
bool aim_tmode_store(int mode)
{
    const int m = (mode == 1) ? 1 : 0;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_i16(h, AIM_NVS_TMODE, (int16_t)m) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)m; return true;
#endif
}

// Reads the persisted temporal mode; false if nothing is stored.
bool aim_tmode_load(int* out_mode)
{
#if defined(AIM_HAVE_STORE)
    if (!out_mode) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    int16_t v = 0;
    const esp_err_t e = nvs_get_i16(h, AIM_NVS_TMODE, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    *out_mode = (v == 1) ? 1 : 0;
    return true;
#else
    (void)out_mode; return false;
#endif
}

// Persists the FIR shape. Window and horizon share one key: pct never exceeds
// 140, so the pair packs into the low and high bytes of a single int16.
bool aim_fir_store(int k, int pct)
{
    if (k < AIM_FIR_KMIN) k = AIM_FIR_KMIN;
    if (k > AIM_FIR_KMAX) k = AIM_FIR_KMAX;
    if (pct < 0)   pct = 0;
    if (pct > 140) pct = 140;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const int16_t packed = (int16_t)((k << 8) | pct);
    const bool ok = (nvs_set_i16(h, AIM_NVS_FIR, packed) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)k; (void)pct; return true;
#endif
}

// Reads the persisted FIR shape; false if nothing is stored or it is nonsense.
bool aim_fir_load(int* out_k, int* out_pct)
{
#if defined(AIM_HAVE_STORE)
    if (!out_k || !out_pct) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    int16_t v = 0;
    const esp_err_t e = nvs_get_i16(h, AIM_NVS_FIR, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    const int k = (v >> 8) & 0xFF, pct = v & 0xFF;
    if (k < AIM_FIR_KMIN || k > AIM_FIR_KMAX || pct > 140) return false;
    *out_k = k; *out_pct = pct;
    return true;
#else
    (void)out_k; (void)out_pct; return false;
#endif
}

// Applies the One Euro filter in place to one screen coordinate pair.
static void oe_filter(float* x, float* y, float dt)
{
    // !(dt > 0) also catches NaN: with `dt <= 0` a NaN sailed through and
    // poisoned the filter state permanently, since nothing reseeds it.
    if (!(dt > 0.0f) || dt > 0.25f) dt = AIM_NOMINAL_DT; // stall or first frame
    float in[2] = { *x, *y };
    if (!s_f_have) {
        s_f_x[0] = in[0]; s_f_x[1] = in[1];
        s_f_dx[0] = 0.0f; s_f_dx[1] = 0.0f;
        s_f_have = true;
        return;
    }
    for (int i = 0; i < 2; ++i) {
        const float dx = (in[i] - s_f_x[i]) / dt;
        const float ad = oe_alpha(AIM_FILT_DCUTOFF, dt);
        s_f_dx[i] += ad * (dx - s_f_dx[i]);
        const float cutoff = s_fc + s_beta * fabsf(s_f_dx[i]);
        const float a = oe_alpha(cutoff, dt);
        s_f_x[i] += a * (in[i] - s_f_x[i]);
    }
    *x = s_f_x[0]; *y = s_f_x[1];
}

// ---------------------------------------------------------------------------
// validity: the SAME gates aim_calib_fit applies, so a calibration cannot enter
// through the serial door that the fitter would have rejected.
// ---------------------------------------------------------------------------
static bool plausible(const aim_calib_t* c)
{
    if (!c || c->magic != AIM_CAL_MAGIC) return false;
    if (!(c->w > 0.02f && c->w < 20.0f)) return false;
    if (!(c->h > 0.02f && c->h < 20.0f)) return false;
    if (!(fabsf(c->bx) < 60.0f && fabsf(c->by) < 60.0f)) return false;
    if (!(fabsf(c->cx) < 20.0f && fabsf(c->cy) < 20.0f)) return false;
    // NaN would sail through every comparison above except this one.
    if (c->w != c->w || c->h != c->h || c->bx != c->bx || c->by != c->by) return false;
    if (c->cx != c->cx || c->cy != c->cy || c->lever != c->lever) return false;
    return true;
}

// ---------------------------------------------------------------------------
// persistence
// ---------------------------------------------------------------------------

// Writes the calibration blob to NVS.
static bool nvs_store(const aim_calib_t* c)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = nvs_set_blob(h, AIM_NVS_KEY, c, sizeof(*c));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
#else
    (void)c; return true;                // host build: nothing to persist to
#endif
}

// Reads the calibration blob from NVS; rejects a blob of the wrong size.
static bool nvs_load(aim_calib_t* c)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = sizeof(*c);
    esp_err_t e = nvs_get_blob(h, AIM_NVS_KEY, c, &len);
    nvs_close(h);
    // A size mismatch means the struct changed under a stored blob; reject
    // rather than reinterpret old bytes as new fields.
    if (e == ESP_OK && len != sizeof(*c)) {
        aim_out("AIM: stored calibration is from an older firmware layout"
                " (%u bytes, expected %u) -- ignored, please recalibrate\n",
                (unsigned)len, (unsigned)sizeof(*c));
        return false;
    }
    return (e == ESP_OK && len == sizeof(*c));
#else
    (void)c; return false;
#endif
}

// ---- camera settings persistence ----------------------------------------

// Range-checks camera settings; ranges match what ov2640_tune() accepts.
static bool cam_plausible(const aim_cam_t* c)
{
    if (!c) return false;
    if (c->thr   < 8  || c->thr   > 255) return false;
    if (c->aec   < 0  || c->aec   > 1200) return false;
    if (c->agc   < 0  || c->agc   > 30) return false;
    if (c->boost < 0  || c->boost > 1) return false;
    return true;
}

// Persists the latency lead, clamped to 0..50 ms.
bool aim_lead_store(int ms)
{
    if (ms < 0) ms = 0;
    if (ms > 50) ms = 50;              // the same ceiling the capture layer clamps to
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = (nvs_set_i16(h, AIM_NVS_LEAD, (int16_t)ms) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    (void)ms; return true;
#endif
}

// Reads the persisted latency lead in ms; false if nothing is stored.
bool aim_lead_load(int* out_ms)
{
#if defined(AIM_HAVE_STORE)
    if (!out_ms) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    int16_t v = 0;
    const esp_err_t e = nvs_get_i16(h, AIM_NVS_LEAD, &v);
    nvs_close(h);
    if (e != ESP_OK) return false;
    if (v < 0) v = 0;
    if (v > 50) v = 50;
    *out_ms = (int)v;
    return true;
#else
    (void)out_ms; return false;
#endif
}

// Reads the persisted camera settings; false if nothing valid is stored.
bool aim_cam_load(aim_cam_t* out)
{
#if defined(AIM_HAVE_STORE)
    if (!out) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = sizeof(*out);
    const esp_err_t e = nvs_get_blob(h, AIM_NVS_CAM, out, &len);
    nvs_close(h);
    return (e == ESP_OK && len == sizeof(*out) && cam_plausible(out));
#else
    (void)out; return false;
#endif
}

// Persists camera settings; false if they fail the range check.
bool aim_cam_store(const aim_cam_t* c)
{
    if (!cam_plausible(c)) return false;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = nvs_set_blob(h, AIM_NVS_CAM, c, sizeof(*c));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
#else
    return true;
#endif
}

// Erases the stored camera settings.
bool aim_cam_clear(void)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, AIM_NVS_CAM); nvs_commit(h); nvs_close(h);
    }
#endif
    return true;
}

// Same ranges the tune console accepts; NaN-safe (x==x).
static bool lens_plausible(const aim_lens_t* c)
{
    if (!c) return false;
    if (c->model < 0 || c->model > 2) return false;
    if (!(c->k1 == c->k1) || c->k1 < -2.0f || c->k1 > 2.0f) return false;
    if (!(c->k2 == c->k2) || c->k2 < -2.0f || c->k2 > 2.0f) return false;
    if (!(c->fpx == c->fpx) || c->fpx < 10.0f || c->fpx > 2000.0f) return false;
    if (!(c->feq == c->feq) || c->feq < 10.0f || c->feq > 2000.0f) return false;
    if (!(c->cx == c->cx) || c->cx < -60.0f || c->cx > 60.0f) return false;
    if (!(c->cy == c->cy) || c->cy < -60.0f || c->cy > 60.0f) return false;
    return true;
}

bool aim_lens_load(aim_lens_t* out)
{
#if defined(AIM_HAVE_STORE)
    if (!out) return false;
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = sizeof(*out);
    const esp_err_t e = nvs_get_blob(h, AIM_NVS_LENS, out, &len);
    nvs_close(h);
    return (e == ESP_OK && len == sizeof(*out) && lens_plausible(out));
#else
    (void)out; return false;
#endif
}

bool aim_lens_store(const aim_lens_t* c)
{
    if (!lens_plausible(c)) return false;
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = nvs_set_blob(h, AIM_NVS_LENS, c, sizeof(*c));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
#else
    return true;
#endif
}

bool aim_lens_clear(void)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, AIM_NVS_LENS); nvs_commit(h); nvs_close(h);
    }
#endif
    return true;
}

// Boot forensics.
static uint32_t s_boot_count = 0;

uint32_t aim_boot_count(void) { return s_boot_count; }

uint32_t aim_uptime_s(void)
{
#if defined(AIM_HAVE_STORE)
    return (uint32_t)(esp_timer_get_time() / 1000000LL);
#else
    return 0;
#endif
}

const char* aim_reset_reason(void)
{
#if defined(AIM_HAVE_STORE)
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
#else
    return "HOST";
#endif
}

// Counts this boot; failures ignored -- a diagnostic must never block aiming.
static void boot_count_bump(void)
{
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint32_t v = 0;
    nvs_get_u32(h, AIM_NVS_BOOT, &v);
    s_boot_count = v + 1u;
    nvs_set_u32(h, AIM_NVS_BOOT, s_boot_count);
    nvs_commit(h);
    nvs_close(h);
#endif
}

static bool s_hid = true;   // RAM only, and it boots enabled

// Reports whether the gun may drive the cursor.
bool aim_runtime_hid_enabled(void) { return s_hid; }
// Sets the pointer gate for this session only.
void aim_runtime_hid_set(bool on)   { s_hid = on; }

// Loads the stored calibration at boot and reports what was found.
void aim_runtime_begin(void)
{
    memset(&s_c, 0, sizeof(s_c));
    s_enabled = true;
    boot_count_bump();
    aim_calib_t t;
    if (nvs_load(&t) && plausible(&t)) {
        s_c = t;
        aim_out("AIM: calibration loaded  bore %+.2f,%+.2f  rect %.4f x %.4f"
               "  rms %.5f  (%u shots)\n",
               (double)s_c.bx, (double)s_c.by, (double)s_c.w, (double)s_c.h,
               (double)s_c.fit_rms, (unsigned)s_c.n_shots);
    } else {
        aim_out("AIM: no stored calibration -- stock OpenFIRE geometry in use.\n"
               "     run tools/aim_calib.py, then send the aimcal= line it prints.\n");
    }
}

// Reports whether trigger-marker capture mode is on.
bool aim_runtime_capture_on(void)      { return s_capture; }

// Milliseconds since boot, or 0 off target.
static uint32_t aim_now_ms(void)
{
#if defined(AIM_HAVE_STORE)
    return (uint32_t)(esp_timer_get_time() / 1000);
#else
    return 0;
#endif
}

// Emits a "T,<ms>" marker on the trigger press edge while capture mode is on.
void aim_runtime_trigger_tick(bool pressed)
{
    // Edge-detected here rather than trusting the button library, whose
    // pressedReleased semantics differ between poll modes.
    if (s_capture && pressed && !s_trig_prev)
        aim_out("T,%lu\n", (unsigned long)aim_now_ms());
    s_trig_prev = pressed;
}

// True when a valid calibration is loaded and enabled.
bool aim_runtime_active(void)          { return s_enabled && plausible(&s_c); }
void aim_runtime_enable(bool on)       { s_enabled = on; }
const aim_calib_t* aim_runtime_calib(void) { return &s_c; }

// Installs a calibration, optionally persisting it. False if implausible.
bool aim_runtime_set(const aim_calib_t* c, bool persist)
{
    if (!plausible(c)) return false;
    s_c = *c;
    s_enabled = true;
    aim_filter_reset();          // a new mapping invalidates the filter history
    return persist ? nvs_store(&s_c) : true;
}

// Forgets the calibration and erases it from NVS.
bool aim_runtime_clear(void)
{
    memset(&s_c, 0, sizeof(s_c));
#if defined(AIM_HAVE_STORE)
    nvs_handle_t h;
    if (nvs_open(AIM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, AIM_NVS_KEY); nvs_commit(h); nvs_close(h);
    }
#endif
    return true;
}

// Hot path: native-px quad -> filtered normalised screen coords.
bool aim_runtime_solve(const aim_pt_t q[4], float frame_w, float frame_h,
                       float* sx, float* sy, float dt_s)
{
    if (!s_enabled || s_c.magic != AIM_CAL_MAGIC) return false;
    if (!aim_solve(&s_c, q, frame_w, frame_h, sx, sy)) return false;
    if (s_tmode == 1)        fir_filter(sx, sy, dt_s);
    else if (s_fc > 0.0f)   oe_filter(sx, sy, dt_s);
    // A caller mapping this into integer screen units must never see a NaN.
    if (*sx != *sx || *sy != *sy) return false;
    return true;
}

// ---------------------------------------------------------------------------
// serial
//   aimcal=cx,cy,w,h,bx,by[,lever[,rx,ry]]  install + persist
//   aimcal?                          print the active one
//   aimcal=off / aimcal=on           toggle without forgetting it
//   aimcal=clear                     erase from NVS
// ---------------------------------------------------------------------------
static bool (*s_extra)(const char*) = 0;

// Installs the handler for claimed lines this module does not own.
void aim_serial_set_extra(bool (*fn)(const char* line)) { s_extra = fn; }

// Accumulates a '~'-prefixed line one byte at a time; true if we claimed the byte.
bool aim_serial_rx(char ch)
{
    static char buf[128];
    static int  n = 0;
    static bool in_line = false;

    if (!in_line) {
        if (ch != '~') return false;      // not ours; leave the byte alone
        in_line = true; n = 0;
        return true;                      // consume the sentinel
    }
    if (ch == '\n' || ch == '\r') {
        buf[n] = 0;
        in_line = false;
        if (n) {
            if (!aim_runtime_command(buf) && !(s_extra && s_extra(buf)))
                aim_out("AIM: unknown command \"%s\"\n", buf);
        }
        n = 0;
        return true;
    }
    if (n < (int)sizeof(buf) - 1) buf[n++] = ch;
    else { n = 0; in_line = false; }       // overlong: resync rather than wrap
    return true;
}

// Executes one command line; true if it was ours. Replies via aim_out.
bool aim_runtime_command(const char* line)
{
    if (!line) return false;
    while (*line == ' ' || *line == '\t') ++line;
    // A leading '~' is optional, so both transports accept the same dialect.
    if (*line == '~') ++line;
    if (!strncmp(line, "ping", 4)) {
        // up/boots/rst: a silent idle reboot shows here -- short uptime, boots
        // one higher, and the reset reason names the culprit.
        aim_out("AIM: pong  board=%s calib=%s filter=%.2f/%.2f capture=%s"
                "  up=%lus boots=%lu rst=%s\n",
                AIM_BOARD,
                plausible(&s_c) ? (s_enabled ? "active" : "loaded") : "none",
                (double)s_fc, (double)s_beta, s_capture ? "on" : "off",
                (unsigned long)aim_uptime_s(), (unsigned long)aim_boot_count(),
                aim_reset_reason());
        return true;
    }
    if (!strncmp(line, "aimhid", 6)) {
        const char* q = line + 6;
        if (*q == '?') {
            aim_out("AIM: pointer %s\n", s_hid ? "ON" : "FROZEN");
            return true;
        }
        if (*q != '=') return false;
        // A trailing '!' (once "do not persist") is accepted and ignored.
        aim_runtime_hid_set(q[1] != '0');
        aim_out("AIM: pointer %s (this session only)\n", s_hid ? "ON" : "FROZEN");
        return true;
    }
    if (!strncmp(line, "aimfilt", 7)) {
        const char* q = line + 7;
        if (*q == '?') {
            aim_out("AIM: filter min_cutoff=%.2f Hz beta=%.3f %s\n",
                   (double)s_fc, (double)s_beta, s_fc > 0.0f ? "" : "(OFF)");
            return true;
        }
        if (*q != '=') return false;
        ++q;
        char* end;
        const float a = strtof(q, &end);
        float b = s_beta;
        if (end != q) {
            q = end; while (*q == ',' || *q == ' ') ++q;
            const float t = strtof(q, &end);
            if (end != q) b = t;
        }
        aim_filter_set(a, b);
        if (a <= 0.0f) aim_out("AIM: filter OFF\n");
        else aim_out("AIM: filter min_cutoff=%.2f Hz beta=%.3f\n", (double)a, (double)b);
        return true;
    }
    if (!strncmp(line, "aimcap=", 7)) {
        s_capture = (line[7] != '0');
        s_trig_prev = false;
        aim_out("AIM: trigger markers %s\n", s_capture ? "ON" : "off");
        return true;
    }
    if (strncmp(line, "aimcal", 6) != 0) return false;
    const char* p = line + 6;

    if (*p == '?') {
        if (plausible(&s_c))
            aim_out("AIM: %s  cx=%.5f cy=%.5f w=%.5f h=%.5f bx=%.3f by=%.3f lever=%.5f"
                   " rx=%.5f ry=%.5f"
                   "  rms=%.5f spread=%.2f roll=%.2f shots=%u rej=%d\n",
                   s_enabled ? "ACTIVE" : "loaded-but-disabled",
                   (double)s_c.cx, (double)s_c.cy, (double)s_c.w, (double)s_c.h,
                   (double)s_c.bx, (double)s_c.by, (double)s_c.lever,
                   (double)s_c.rx, (double)s_c.ry,
                   (double)s_c.fit_rms, (double)s_c.fit_spread,
                   (double)s_c.fit_roll,
                   (unsigned)s_c.n_shots, aim_calib_n_rejected(&s_c));
        else
            aim_out("AIM: none stored (stock OpenFIRE geometry)\n");
        return true;
    }
    // "aimcal!=..." applies WITHOUT writing NVS, for live preview.
    bool persist = true;
    if (p[0] == '!' && p[1] == '=') { persist = false; ++p; }
    if (*p != '=') return false;
    ++p;

    if (!strncmp(p, "off", 3))   { s_enabled = false; aim_out("AIM: disabled\n");  return true; }
    if (!strncmp(p, "on", 2))    { s_enabled = true;  aim_out("AIM: enabled\n");   return true; }
    if (!strncmp(p, "clear", 5)) { aim_runtime_clear(); aim_out("AIM: cleared\n"); return true; }

    // 6 values = the original form, 7 adds the lever, 9 adds the roll pair.
    aim_calib_t c; memset(&c, 0, sizeof(c));
    float v[9] = {0,0,0,0,0,0,0,0,0};
    int n = 0;
    char* end;
    while (n < 9) {
        const float f = strtof(p, &end);
        if (end == p) break;
        v[n++] = f; p = end;
        while (*p == ',' || *p == ' ') ++p;
    }
    if (n != 6 && n != 7 && n != 9) {
        aim_out("AIM: need 6, 7 or 9 numbers, got %d\n", n); return true;
    }
    c.magic = AIM_CAL_MAGIC;
    c.cx = v[0]; c.cy = v[1]; c.w = v[2]; c.h = v[3];
    c.bx = v[4]; c.by = v[5]; c.lever = (n >= 7) ? v[6] : 0.0f;
    c.rx = (n >= 9) ? v[7] : 0.0f;
    c.ry = (n >= 9) ? v[8] : 0.0f;
    if (!aim_runtime_set(&c, persist)) {
        aim_out("AIM: REJECTED -- values fail the same checks the fitter applies\n");
        return true;
    }
    aim_out("AIM: installed%s. bore %+.2f,%+.2f  rect %.4f x %.4f  roll %s\n",
           persist ? " and saved" : " (preview, not saved)",
           (double)c.bx, (double)c.by, (double)c.w, (double)c.h,
           (c.rx != 0.0f || c.ry != 0.0f) ? "corrected" : "not fitted");
    return true;
}
