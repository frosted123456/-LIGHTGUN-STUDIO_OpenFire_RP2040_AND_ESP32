// lead_test.cpp -- the latency lead must actually move the quad, by a known
// amount. Drives the REAL quad_resolver with a synthetic pan and applies the
// same arithmetic ov2640_capture uses at publish, reported in screen px.
#include "../lib/OV2640Capture/quad_resolver.h"
#include <math.h>
#include <stdio.h>

// mirrors ov2640_capture.cpp
#define FPS_NOMINAL   135.0f
#define LEAD_MS_MAX   30.0f
#define LEAD_PX_MAX   40.0f
static float lead_px(float vx, float ms)
{
    if (ms <= 0.0f) return 0.0f;
    if (ms > LEAD_MS_MAX) ms = LEAD_MS_MAX;
    float l = vx * (ms * (FPS_NOMINAL / 1000.0f));
    if (l >  LEAD_PX_MAX) l =  LEAD_PX_MAX;
    if (l < -LEAD_PX_MAX) l = -LEAD_PX_MAX;
    return l;
}

static const float GAIN = 26.0f;      // screen px per camera px, measured
static int fails = 0;
static void ck(bool ok, const char* m) { printf("  [%s] %s\n", ok?"PASS":"FAIL", m); if(!ok)fails++; }

// a rig, panned at a constant image speed
static QuadResult pan(float vx_per_frame, int frames)
{
    quad_reset(nullptr);
    QuadResult r{};
    float x = 120.0f, y = 88.0f;
    const float hw = 27.0f, hh = 50.0f;
    for (int f = 0; f < frames; ++f) {
        const float xs[4] = { x-hw, x+hw, x-hw, x+hw };
        const float ys[4] = { y-hh, y-hh, y+hh, y+hh };
        r = quad_update(xs, ys, 4);
        x += vx_per_frame;
    }
    return r;
}

int main(void)
{
    printf("latency lead, end to end through the real resolver\n\n");

    QuadResult still = pan(0.0f, 60);
    ck(still.locked, "locks on a stationary rig");
    ck(fabsf(still.vx) < 0.02f, "reports ~zero velocity when still");
    ck(fabsf(lead_px(still.vx, 10.0f)) < 0.5f, "...so a still gun gets no lead");

    printf("\n%-26s %10s %10s %10s %10s\n",
           "pan speed", "vx px/fr", "lead@5ms", "lead@10ms", "lead@20ms");
    struct { const char* n; float v; } SPEEDS[] = {
        {"slow track  (~4 deg/s)",  0.10f},
        {"normal aim  (~15 deg/s)", 0.36f},
        {"fast sweep  (~60 deg/s)", 1.45f},
        {"flick       (~200 deg/s)",4.80f},
    };
    for (unsigned i = 0; i < sizeof(SPEEDS)/sizeof(SPEEDS[0]); ++i) {
        QuadResult r = pan(SPEEDS[i].v, 60);
        printf("%-26s %10.3f %9.0fpx %9.0fpx %9.0fpx\n", SPEEDS[i].n, r.vx,
               lead_px(r.vx, 5.0f)*GAIN, lead_px(r.vx, 10.0f)*GAIN,
               lead_px(r.vx, 20.0f)*GAIN);
        // the reported velocity must track the truth, or the lead is scaled by
        // an unknown factor
        if (fabsf(r.vx - SPEEDS[i].v) > 0.15f * SPEEDS[i].v + 0.02f) {
            printf("      [FAIL] reported %.3f px/frame for a true %.3f\n", r.vx, SPEEDS[i].v);
            fails++;
        }
    }
    printf("        (screen px, at the measured %.0f screen px per camera px)\n", GAIN);

    QuadResult fast = pan(1.45f, 60);
    ck(lead_px(fast.vx, 10.0f) * GAIN > 20.0f,
       "10 ms is worth more than 20 screen px on a fast sweep -- i.e. visible");
    ck(lead_px(fast.vx, 2.0f) * GAIN < 15.0f,
       "2 ms is under 15 px -- one or two button presses is NOT expected to be felt");
    ck(fabsf(lead_px(pan(9.0f, 60).vx, 30.0f)) <= LEAD_PX_MAX + 0.01f,
       "clamped at 40 camera px so a velocity glitch cannot fling the quad");

    printf("\n%s (%d failures)\n", fails ? "FAILURES" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
