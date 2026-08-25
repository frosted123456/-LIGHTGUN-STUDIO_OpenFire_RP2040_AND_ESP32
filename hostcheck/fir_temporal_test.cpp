// Host test for temporal mode 1: the single FIR that replaces the One Euro
// plus the capture-side latency lead.
//
// The properties that matter are algebraic, so they are asserted exactly:
// weights sum to 1 (a static aim cannot drift), a constant-velocity ramp is
// extrapolated with zero error (a line fitted to a line), and mode 0 output is
// unchanged to the last bit by the existence of mode 1.
#include "../lib/AimPipeline/aim_runtime.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static int fails = 0;
static void ck(bool ok, const char* m)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", m);
    if (!ok) fails++;
}

static const float DT = 1.0f / 135.0f;
static const char* CAL =
    "aimcal=0.500000,0.500000,0.400000,1.200000,0.000,0.000";

// Feeds a quad whose centre tracks (u,v) in native px, and returns the solved
// screen point. The calibration above is centred and axis-aligned, so screen
// motion is a fixed linear function of quad motion: enough to test the
// temporal stage without depending on the geometry.
static void feed(float u, float v, float* sx, float* sy)
{
    aim_pt_t q[4] = { {u-30,v-40}, {u+30,v-40}, {u-30,v+40}, {u+30,v+40} };
    aim_runtime_solve(q, 240, 176, sx, sy, DT);
}

static void reset_stream(void) { aim_filter_reset(); }

int main()
{
    printf("temporal mode 1: single-FIR smoothing + prediction\n\n");
    aim_runtime_begin();
    if (!aim_runtime_command(CAL)) { printf("setup failed\n"); return 1; }

    printf("defaults and clamps:\n");
    ck(aim_tmode_get() == 0, "mode 0 is the default -- shipped behaviour unchanged");
    ck(aim_fir_k() == 7 && aim_fir_pct() == 100,
       "FIR defaults to the FULL lead, so switching mode is a fair comparison");
    aim_fir_set(1, 999);
    ck(aim_fir_k() == 3 && aim_fir_pct() == 140, "window and horizon clamp low/high");
    aim_fir_set(99, -5);
    ck(aim_fir_k() == 15 && aim_fir_pct() == 0, "and the other way");
    aim_tmode_set(7);
    ck(aim_tmode_get() == 0, "an out-of-range mode falls back to 0, never to FIR");
    aim_fir_set(7, 50);

    // ---- mode 0 must be bit-identical to a build without mode 1 ----------
    printf("\nmode 0 is untouched:\n");
    aim_tmode_set(0);
    aim_smooth_set(3);
    aim_lead_note(30.0f);            // mode 0 ignores this: the lead is upstream
    aim_conf_note(0.25f);            // and ignores this too
    float ref[40][2];
    reset_stream();
    for (int i = 0; i < 40; ++i) feed(100.0f + 0.7f*i, 88.0f, &ref[i][0], &ref[i][1]);
    float again[40][2];
    reset_stream();
    aim_lead_note(0.0f); aim_conf_note(1.0f);
    for (int i = 0; i < 40; ++i) feed(100.0f + 0.7f*i, 88.0f, &again[i][0], &again[i][1]);
    bool same = true;
    for (int i = 0; i < 40; ++i)
        if (ref[i][0] != again[i][0] || ref[i][1] != again[i][1]) same = false;
    ck(same, "mode 0 output does not depend on the FIR's inputs at all");

    // ---- DC gain: a static aim must not drift ----------------------------
    printf("\nstatic aim (weights sum to 1):\n");
    aim_tmode_set(1);
    aim_lead_note(20.0f); aim_conf_note(1.0f); aim_fir_set(7, 100);
    reset_stream();
    float x0 = 0, y0 = 0, xn = 0, yn = 0;
    for (int i = 0; i < 200; ++i) {
        feed(120.0f, 88.0f, &xn, &yn);
        if (i == 20) { x0 = xn; y0 = yn; }
    }
    ck(fabsf(xn - x0) < 1e-6f && fabsf(yn - y0) < 1e-6f,
       "200 identical frames produce no drift");

    // ---- a line fitted to a line is exact -------------------------------
    printf("\nconstant-velocity ramp (extrapolation must be exact):\n");
    for (int k = 3; k <= 15; k += 2) {
        aim_fir_set(k, 100);
        aim_lead_note(5.0f);
        reset_stream();
        const float step = 0.5f;                   // native px per frame
        float sx = 0, sy = 0;
        for (int i = 0; i < 60; ++i) feed(120.0f + step*i, 88.0f, &sx, &sy);
        // where the UNFILTERED solve would put the aim `tf` frames ahead.
        // 5 ms keeps tf under the 0.5*(k-1) horizon ceiling even at k=3, so
        // this measures the fit's exactness and not the clamp.
        const float tf = (5.0f * 1e-3f) / DT;      // lead ms -> frames
        aim_tmode_set(0); aim_smooth_set(0);
        float px = 0, py = 0;
        feed(120.0f + step*(59.0f + tf), 88.0f, &px, &py);
        aim_tmode_set(1);
        char msg[96];
        snprintf(msg, sizeof msg,
                 "k=%d: ramp extrapolated to within %.2e of the true future point",
                 k, (double)fabsf(sx - px));
        ck(fabsf(sx - px) < 2e-5f, msg);
    }

    // The ceiling must bind: ask for far more horizon than the window supports
    // and the output must match what the ceiling allows, not what was asked.
    printf("\nhorizon ceiling:\n");
    aim_fir_set(7, 140); aim_lead_note(50.0f); aim_conf_note(1.0f);
    reset_stream();
    float capped = 0, dref = 0;
    for (int i = 0; i < 30; ++i) feed(120.0f + 0.5f*i, 88.0f, &capped, &dref);
    // 50 ms at 140% is 9.5 frames; the ceiling is the window span, k-1 = 6
    aim_tmode_set(0); aim_smooth_set(0);
    float at6 = 0;
    feed(120.0f + 0.5f*(29.0f + 6.0f), 88.0f, &at6, &dref);
    aim_tmode_set(1);
    ck(fabsf(capped - at6) < 5e-5f,
       "an impossible horizon clamps to the window span, never beyond it");
    aim_lead_note(20.0f);

    // The knobs must stay independent: the horizon a given lead produces must
    // not depend on the window, which a tighter ceiling silently broke.
    printf("\nknob independence:\n");
    aim_lead_note(30.0f); aim_fir_set(7, 100); aim_conf_note(1.0f);
    float w7 = 0, w9 = 0;
    reset_stream();
    for (int i = 0; i < 30; ++i) feed(120.0f + 0.4f*i, 88.0f, &w7, &dref);
    aim_fir_set(9, 100); reset_stream();
    for (int i = 0; i < 30; ++i) feed(120.0f + 0.4f*i, 88.0f, &w9, &dref);
    ck(fabsf(w7 - w9) < 5e-5f,
       "a 30 ms lead reaches the same point at k=7 and k=9");
    aim_fir_set(7, 100);

    // And mode 1 at 100% must lead exactly as far as mode 0 does at the same ms.
    aim_lead_note(30.0f); reset_stream();
    float m1 = 0;
    for (int i = 0; i < 30; ++i) feed(120.0f + 0.4f*i, 88.0f, &m1, &dref);
    aim_tmode_set(0); aim_smooth_set(0);
    float m0 = 0;
    feed(120.0f + 0.4f*(29.0f + 30.0f * 0.135f), 88.0f, &m0, &dref);
    aim_tmode_set(1);
    ck(fabsf(m1 - m0) < 5e-5f,
       "mode 1 at 100% compensates the same 30 ms that mode 0 does");

    // ---- warm-up: no wild output before the window fills -----------------
    printf("\nwarm-up:\n");
    aim_fir_set(9, 140);
    reset_stream();
    float first[12][2];
    for (int i = 0; i < 12; ++i) feed(120.0f, 88.0f, &first[i][0], &first[i][1]);
    bool tame = true;
    for (int i = 0; i < 12; ++i)
        if (fabsf(first[i][0] - first[11][0]) > 1e-5f) tame = false;
    ck(tame, "a still gun gives a constant output from the very first frame");

    // ---- confidence gates the horizon -----------------------------------
    printf("\ndropout handling (confidence scales the horizon):\n");
    aim_fir_set(7, 100);
    aim_lead_note(30.0f);
    const float step = 1.0f;
    float full = 0, none = 0, dummy = 0, raw = 0;
    // The unpredicted reference first. Panning the gun right moves the LED
    // quad LEFT in the image, so "further ahead" is not a fixed sign -- judge
    // the DISTANCE from the unpredicted point instead.
    aim_tmode_set(0); aim_smooth_set(0); reset_stream();
    for (int i = 0; i < 40; ++i) feed(120.0f + step*i, 88.0f, &raw, &dummy);
    aim_tmode_set(1);
    aim_conf_note(1.0f); reset_stream();
    for (int i = 0; i < 40; ++i) feed(120.0f + step*i, 88.0f, &full, &dummy);
    aim_conf_note(0.0f); reset_stream();
    for (int i = 0; i < 40; ++i) feed(120.0f + step*i, 88.0f, &none, &dummy);
    ck(fabsf(full - raw) > fabsf(none - raw) + 1e-4f,
       "confidence 1.0 predicts further from the unled point than 0.0 does");
    ck(fabsf(none - raw) < 2e-5f,
       "at zero confidence the FIR is a pure smoother, no prediction at all");

    // ---- a gap in the sample stream must not be fitted across ----------
    // Review finding: losing lock and re-acquiring elsewhere was fitted as one
    // enormous velocity and threw the cursor 600 px off-screen for k frames.
    printf("\ndropped lock (a gap invalidates the fit):\n");
    aim_fir_set(7, 100); aim_lead_note(20.0f); aim_conf_note(1.0f);
    reset_stream();
    float held = 0, dd = 0;
    for (int i = 0; i < 30; ++i) feed(100.0f, 88.0f, &held, &dd);
    // the gun reappears 60 px away after a 300 ms blackout
    float after[8][2]; float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 8; ++i) {
        aim_pt_t qq[4] = { {130,48}, {190,48}, {130,128}, {190,128} };
        aim_runtime_solve(qq, 240, 176, &after[i][0], &after[i][1],
                          i ? DT : 0.300f);
        if (after[i][0] < lo) lo = after[i][0];
        if (after[i][0] > hi) hi = after[i][0];
    }
    float settled = 0;
    feed(160.0f, 88.0f, &settled, &dd);
    ck(lo > -0.05f && hi < 1.05f,
       "re-acquisition stays on screen instead of overshooting off the edge");

    // ---- the excursion cap bounds a step ------------------------------
    printf("\nstep response (excursion cap):\n");
    aim_fir_set(7, 140); aim_lead_note(50.0f);   // the most aggressive settings
    reset_stream();
    float base = 0;
    for (int i = 0; i < 20; ++i) feed(100.0f, 88.0f, &base, &dd);
    float peak = base, target = 0;
    aim_tmode_set(0); aim_smooth_set(0); reset_stream();
    for (int i = 0; i < 20; ++i) feed(130.0f, 88.0f, &target, &dd);
    aim_tmode_set(1); aim_fir_set(7, 140); reset_stream();
    for (int i = 0; i < 20; ++i) feed(100.0f, 88.0f, &base, &dd);
    for (int i = 0; i < 12; ++i) {
        float o = 0; feed(130.0f, 88.0f, &o, &dd);
        if (fabsf(o - base) > fabsf(peak - base)) peak = o;
    }
    const float over = fabsf(peak - target) / fabsf(target - base);
    char m2[128];
    snprintf(m2, sizeof m2,
             "a 30 px step overshoots %.0f%% of the step and stays inside the "
             "0.35 backstop", (double)(over * 100.0));
    // A degree-1 fit cannot tell a step from a ramp, so at the most aggressive
    // settings it WILL overshoot a step by about its own size. What must hold
    // is the absolute bound -- the analogue of mode 0's LEAD_PX_MAX.
    ck(over < 1.30f && fabsf(peak - base) < 0.35f + fabsf(target - base) + 1e-3f,
       m2);

    // ---- a shortened window must not fit over stale samples -------------
    printf("\nwindow change:\n");
    aim_fir_set(9, 100); aim_lead_note(20.0f); reset_stream();
    for (int i = 0; i < 20; ++i) feed(100.0f + 2.0f*i, 88.0f, &dd, &dd);
    aim_fir_set(3, 100);            // shrink mid-stream
    float shrunk = 0;
    feed(140.0f, 88.0f, &shrunk, &dd);
    float fresh = 0;
    aim_fir_set(3, 100); reset_stream();
    feed(140.0f, 88.0f, &fresh, &dd);
    ck(fabsf(shrunk - fresh) < 1e-6f,
       "shrinking the window discards it rather than fitting the oldest samples");

    // NaN must never escape: the patches map the result into integer px.
    printf("\nNaN containment:\n");
    aim_conf_note(NAN); aim_lead_note(30.0f); reset_stream();
    float nx = 0, ny = 0; bool clean = true;
    for (int i = 0; i < 40; ++i) {
        feed(120.0f + step*i, 88.0f, &nx, &ny);
        if (nx != nx || ny != ny) clean = false;
    }
    ck(clean, "a NaN confidence cannot poison the output");
    aim_conf_note(1.0f);

    // ---- a wider window must not cost lead ------------------------------
    // Frank's hardware finding: a wider window is the answer to a jittery
    // prediction. That only holds if widening it does not shorten the lead.
    printf("\nwide windows keep the lead:\n");
    aim_lead_note(30.0f); aim_conf_note(1.0f);
    float ref9 = 0, dz = 0;
    aim_tmode_set(0); aim_smooth_set(0); reset_stream();
    feed(120.0f + 0.4f * (39.0f + 30.0f * 0.135f), 88.0f, &ref9, &dz);  // 40 frames fed
    aim_tmode_set(1);
    bool allmatch = true;
    for (int k = 7; k <= 15; k += 2) {
        aim_fir_set(k, 100); reset_stream();
        float o = 0;
        for (int i = 0; i < 40; ++i) feed(120.0f + 0.4f*i, 88.0f, &o, &dz);
        if (fabsf(o - ref9) > 5e-5f) allmatch = false;
    }
    ck(allmatch, "k=7..15 all deliver the same 30 ms of lead on a steady sweep");

    // Three measured corners must not shorten the horizon: a dim LED on a
    // wide lens was quietly costing a third of the lead for as long as it lasted.
    aim_fir_set(7, 100);
    float c4 = 0, c3 = 0;
    aim_conf_note(1.0f); reset_stream();
    for (int i = 0; i < 40; ++i) feed(120.0f + 0.4f*i, 88.0f, &c4, &dz);
    aim_conf_note(0.75f); reset_stream();
    for (int i = 0; i < 40; ++i) feed(120.0f + 0.4f*i, 88.0f, &c3, &dz);
    ck(fabsf(c4 - c3) < 5e-5f,
       "3 of 4 corners measured still gets the full lead");
    aim_conf_note(1.0f);

    printf("\n%s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
