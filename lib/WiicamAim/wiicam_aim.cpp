// wiicam_aim.cpp -- see header. All processing is in 240x176 space.
#include "wiicam_aim.h"
#include "quad_resolver.h"
#include "aim_runtime.h"
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

static void (*s_line)(const char*)  = 0;
static void (*s_reply)(const char*) = 0;
static void (*s_sens_set)(int) = 0;
static int  (*s_sens_get)(void) = 0;
static void (*s_sens_save)(void) = 0;

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
    char b[256];        // cam? reaches ~185 with a fitted lens; 192 was too tight
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
static int      s_cache_px[4], s_cache_py[4];
static unsigned s_cache_seen = 0xFFFFFFFFu;    // impossible: never matches first
static bool     s_cache_ret = false;
static float    s_cache_sx = 0.0f, s_cache_sy = 0.0f;

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
    // A byte-identical report is the previous camera frame seen again: return
    // the cached answer and leave every stateful stage untouched.
    bool same = (seen == s_cache_seen);
    for (int i = 0; same && i < 4; ++i)
        if ((seen & (1u << i)) && (px[i] != s_cache_px[i] || py[i] != s_cache_py[i]))
            same = false;
    if (same) {
        *sx = s_cache_sx; *sy = s_cache_sy;
        return s_cache_ret;
    }
    s_cache_seen = seen;
    for (int i = 0; i < 4; ++i) { s_cache_px[i] = px[i]; s_cache_py[i] = py[i]; }
    s_cache_ret = false;

    // Seen-mask gate: the driver retains an unseen slot's previous
    // coordinates, so unmasked reads would feed the resolver stale points.
    float xs[4], ys[4];
    int n = 0;
    for (int i = 0; i < 4; ++i) {
        if (!(seen & (1u << i))) continue;
        float nx = s_mirx ? (WIICAM_W - 1.0f - (float)px[i]) : (float)px[i];
        float ny = s_miry ? (WIICAM_H - 1.0f - (float)py[i]) : (float)py[i];
        float x = nx * SX;
        float y = ny * SY;
        lens_undistort(&x, &y);
        xs[n] = x; ys[n] = y; ++n;
    }

    const float dt = (s_prev_us && now_us > s_prev_us)
                   ? (float)(now_us - s_prev_us) * 1e-6f : 0.0f;
    s_prev_us = now_us;

    if (s_res == 0) {                       // raw mode, for the lens sweep
        emit_q(now_us, xs, ys, n);
        return false;
    }

    QuadResult r = quad_update(xs, ys, n);
    if (r.count < 4) { emit_q(now_us, xs, ys, n); return false; }

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
static int parse_int(const char** p)
{
    int sgn = 1, v = 0;
    if (**p == '-') { sgn = -1; ++(*p); }
    while (**p >= '0' && **p <= '9') v = v * 10 + (*(*p)++ - '0');
    return v * sgn;
}

bool wiicam_cam_command(const char* line)
{
    if (!line) return false;
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
        if (s_sens_save) s_sens_save();     // sens lives in OpenFIRE's profile
        aim_lens_t ls = { (int)s_lens, s_lk1, s_lk2, s_lfpx, s_lfeq,
                          s_lcx, s_lcy };
        bool lens_ok = true;
        if (ls.model == 0) aim_lens_clear();
        else               lens_ok = aim_lens_store(&ls);
        // beta rides in the reply so a tool can VERIFY what was written rather
        // than assume it; cam? reports the live value, this reports the stored one.
        reply(ok && lens_ok
              ? "CAM: saved lead=%dms smooth=%d dead=%d beta=%d lens=%d tmode=%d firk=%d firpct=%d (sens lives in the OpenFIRE profile)\n"
              : "CAM: SAVE FAILED lead=%dms smooth=%d dead=%d beta=%d lens=%d tmode=%d firk=%d firpct=%d\n",
              (int)s_lead_ms, aim_smooth_get(), aim_dead_get(), aim_beta_get(),
              (int)s_lens, aim_tmode_get(), aim_fir_k(), aim_fir_pct());
        return true;
    }
    if (!strncmp(line, "camreset", 8)) {
        aim_lens_clear();
        s_lens = 0; s_lead_ms = 0.0f;
        s_lcx = 0.0f; s_lcy = 0.0f;
        reply("CAM: lens + lead cleared\n");
        return true;
    }
    if (!strncmp(line, "cam?", 4)) {
        reply("CAM: board=rp2040-wiicam sens=%d mirx=%d miry=%d lead=%d "
              "smooth=%d dead=%d lens=%d lk1u=%d lk2u=%d lfpx=%d lfeq=%d "
              "lcxu=%d lcyu=%d beta=%d tmode=%d firk=%d firpct=%d res=%u dash=%u\n",
              s_sens_get ? s_sens_get() : -1, (int)s_mirx, (int)s_miry,
              (int)s_lead_ms, aim_smooth_get(), aim_dead_get(),
              (int)s_lens, (int)(s_lk1*1e6f), (int)(s_lk2*1e6f),
              (int)(s_lfpx*10.0f), (int)(s_lfeq*10.0f),
              (int)(s_lcx*10.0f), (int)(s_lcy*10.0f),
              aim_beta_get(), aim_tmode_get(), aim_fir_k(), aim_fir_pct(),
              (unsigned)s_res, (unsigned)s_dash);
        return true;
    }
    if (strncmp(line, "cam=", 4) != 0) return false;
    const char* p = line + 4;
    while (*p) {
        char key[8] = {0}; int ki = 0;
        while (*p && *p != ':' && *p != ',' && ki < 7) key[ki++] = *p++;
        if (*p != ':') { while (*p && *p != ',') ++p; if (*p) ++p; continue; }
        ++p;
        const int val = parse_int(&p);
        if (*p == ',') ++p;
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
        else if (!strcmp(key, "mirx")) { s_mirx = (uint8_t)(val != 0); quad_reset(0); }
        else if (!strcmp(key, "miry")) { s_miry = (uint8_t)(val != 0); quad_reset(0); }
    }
    s_cache_seen = 0xFFFFFFFFu;    // settings changed: reprocess the next report
    reply("CMD ok (tune) | sens=%d lead=%d smooth=%d beta=%d tmode=%d firk=%d "
          "firpct=%d lens=%u res=%u dash=%u\n",
          s_sens_get ? s_sens_get() : -1, (int)s_lead_ms, aim_smooth_get(),
          aim_beta_get(), aim_tmode_get(), aim_fir_k(), aim_fir_pct(),
          (unsigned)s_lens, (unsigned)s_res, (unsigned)s_dash);
    return true;
}
