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
#define AIM_FILT_MIN_CUTOFF 1.0f
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
void  aim_filter_reset(void) { s_f_have = false; }
float aim_filter_min_cutoff(void) { return s_fc; }
float aim_filter_beta(void) { return s_beta; }

// ---- smoothing level ------------------------------------------------------
// One knob over the One Euro pair. Level 3 equals the build-time defaults, so
// a gun that never stored a level behaves exactly as before the knob existed.
#define AIM_NVS_SMOOTH "smth0"
static const float SMOOTH_TAB[11][2] = {
    {0.00f,  0.0f},   // 0: filter off
    {2.00f, 40.0f},   // 1: barely there
    {1.40f, 25.0f},
    {1.00f, 15.0f},   // 3: the build-time default pair
    {0.90f, 10.0f},
    {0.80f,  7.0f},
    {0.70f,  5.0f},
    {0.60f,  3.5f},
    {0.50f,  2.5f},
    {0.45f,  1.7f},
    {0.40f,  1.2f},   // 10: heaviest, noticeable glide
};
static int s_smooth = 3;

// Clamps, applies the mapped coefficients, and remembers the level.
void aim_smooth_set(int level)
{
    if (level < 0) level = 0;
    if (level > 10) level = 10;
    s_smooth = level;
    aim_filter_set(SMOOTH_TAB[level][0], SMOOTH_TAB[level][1]);
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

// Applies the One Euro filter in place to one screen coordinate pair.
static void oe_filter(float* x, float* y, float dt)
{
    if (dt <= 0.0f || dt > 0.25f) dt = AIM_NOMINAL_DT;   // stall or first frame
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
    if (s_fc > 0.0f) oe_filter(sx, sy, dt_s);
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
