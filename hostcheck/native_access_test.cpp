// Guards the exact member-access pattern FW_Common::GetPosition() uses: the
// nativeX/Y/W/H/Count members must be reachable from OUTSIDE the class. A header
// that compiles on its own can still break every call site.
#include <stdio.h>
#include "DFRobotIRPositionEx.h"
#include "aim_core.h"

static bool call_site(DFRobotIRPositionEx* dfrIRPos, float* sx, float* sy)
{
    // verbatim shape of the block in FW_Common::GetPosition()
    if (dfrIRPos->nativeCount >= 4) {
        aim_pt_t q[4];
        for (int i = 0; i < 4; ++i) {
            q[i].x = dfrIRPos->nativeX[i];
            q[i].y = dfrIRPos->nativeY[i];
        }
        aim_calib_t c = {};
        c.magic = AIM_CAL_MAGIC;
        c.cx = 0.5046f; c.cy = 0.2988f; c.w = 0.3513f; c.h = 1.2003f;
        c.bx = 4.80f;   c.by = -2.12f;
        return aim_solve(&c, q, (float)dfrIRPos->nativeW,
                                (float)dfrIRPos->nativeH, sx, sy) != 0;
    }
    return false;
}

int main()
{
    DFRobotIRPositionEx d;
    d.nativeCount = 4;
    const float qx[4] = {147.600f, 180.750f, 147.350f, 181.800f};
    const float qy[4] = { 66.450f,  65.400f, 129.050f, 130.800f};
    for (int i = 0; i < 4; ++i) { d.nativeX[i] = qx[i]; d.nativeY[i] = qy[i]; }
    d.nativeW = 240; d.nativeH = 176;
    float sx = 0, sy = 0;
    const bool ok = call_site(&d, &sx, &sy);
    // a real captured shot aimed at (0.08, 0.08); the replay gives ~(0.073, 0.078)
    const bool sane = ok && sx > 0.00f && sx < 0.20f && sy > 0.00f && sy < 0.20f;
    printf("native member access from outside the class: %s  (%.4f, %.4f)\n",
           sane ? "OK" : "FAILED", (double)sx, (double)sy);
    return sane ? 0 : 1;
}
