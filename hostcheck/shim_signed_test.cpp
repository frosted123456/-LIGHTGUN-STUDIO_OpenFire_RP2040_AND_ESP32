// Signed-coordinate round trip: compiles the REAL shim (DFRobotIRPositionEx.h)
// against a stub bridge and asserts an out-of-frame (negative / past-edge)
// coordinate keeps its sign and ordering instead of wrapping unsigned.
#include "ov2640_bridge.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

static ov2640_bridge_frame_t g_slot;
static uint32_t g_seq = 0;
extern "C" void ov2640_bridge_publish(const ov2640_bridge_frame_t* f) { g_slot = *f; }
extern "C" uint32_t ov2640_bridge_read(ov2640_bridge_frame_t* out) { *out = g_slot; return g_seq; }

#include "DFRobotIRPositionEx.h"

static int fails = 0;
static void ck(bool ok, const char* what, long got = 0, long want = 0)
{
    printf("  %s  %s", ok ? "PASS" : "FAIL", what);
    if (!ok) printf("   (got %ld, want %ld)", got, want);
    printf("\n");
    if (!ok) ++fails;
}

// same encoder as ov2640_capture.cpp's enc16() (kept in sync by the assert
// below on the in-frame reference value)
static int16_t enc16(float v)
{
    float u = v * 16.0f + (v >= 0.0f ? 0.5f : -0.5f);
    if (!(u > -32000.0f)) u = -32000.0f;
    if (u > 32000.0f)     u =  32000.0f;
    return (int16_t)u;
}

static void publish(float x0, float y0, float x1, float y1)
{
    ov2640_bridge_frame_t f = {};
    f.frame_w = 240; f.frame_h = 176; f.count = 2;
    f.x16[0] = enc16(x0); f.y16[0] = enc16(y0);
    f.x16[1] = enc16(x1); f.y16[1] = enc16(y1);
    f.area4[0] = f.area4[1] = 8;
    ++g_seq; f.frame_seq = g_seq;
    ov2640_bridge_publish(&f);
}

int main()
{
    DFRobotIRPositionEx cam;
    cam.begin();
    printf("v29.5 signed wire-format round trip\n");

    // in-frame reference: centre maps to the middle of the output space
    publish(120.0f, 88.0f, 0.0f, 0.0f);
    ck(cam.basicAtomic() == DFRobotIRPositionEx::Error_Success, "new frame reported");
    const int cxm = cam.x(0), cym = cam.y(0);
    ck(std::abs(cxm - 511) <= 2, "frame centre -> output centre X", cxm, 511);
    ck(std::abs(cym - 383) <= 2, "frame centre -> output centre Y", cym, 383);
    // x is mirrored, so native 0 must map to the output MAXIMUM
    ck(std::abs(cam.x(1) - 1023) <= 2, "native x=0 -> mirrored output max", cam.x(1), 1023);
    ck(std::abs(cam.y(1) - 0) <= 2, "native y=0 -> output 0", cam.y(1), 0);

    // a corner extrapolated OFF the sensor: -30 native px must land beyond the
    // mirrored maximum, not wrap to a huge positive
    publish(-30.0f, -20.0f, 270.0f, 200.0f);
    ck(cam.basicAtomic() == DFRobotIRPositionEx::Error_Success, "new frame reported");
    ck(cam.x(0) > 1023, "negative native x -> output ABOVE max (no wrap)", cam.x(0), 1024);
    ck(cam.y(0) < 0,    "negative native y -> negative output (no wrap)",  cam.y(0), -1);
    ck(cam.x(1) < 0,    "past-edge native x -> negative output (mirrored)", cam.x(1), -1);
    ck(cam.y(1) > 767,  "past-edge native y -> output above max",          cam.y(1), 768);

    // a corner crossing the frame edge must produce a CONTINUOUS output: a
    // discontinuity there deforms the quad and moves the cursor
    bool mono = true;
    int prev = 1 << 30;
    for (float nx = -40.0f; nx <= 40.0f; nx += 2.0f) {
        publish(nx, 88.0f, 0.0f, 0.0f);
        cam.basicAtomic();
        const int cur = cam.x(0);          // mirrored => decreasing in nx
        if (cur > prev) mono = false;
        prev = cur;
    }
    ck(mono, "output X is monotone across the frame edge (no jump)");

    // sanity: the encoder's own sign handling
    ck(enc16(-1.0f) == -16, "enc16 rounds negatives away from zero", enc16(-1.0f), -16);
    ck(enc16(0.0f) == 0, "enc16(0) == 0", enc16(0.0f), 0);

    printf("\n%s  (%d failure%s)\n", fails ? "SHIM TEST FAILED" : "SHIM TEST PASSED",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
