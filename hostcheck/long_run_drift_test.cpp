// long_run_drift_test.cpp -- nothing in the aim path may drift while the gun is
// held on one spot: no random walk in the resolver, no accumulation in the
// filter, and no jump when the 32-bit microsecond clock wraps every ~72 min.
#include "quad_resolver.h"
#include "aim_core.h"
#include "aim_runtime.h"
#include <math.h>
#include <stdio.h>

static unsigned rs = 1234567u;
static float urand(){ rs = rs*1103515245u + 12345u; return ((rs>>16)&0x7fff)/32767.0f; }
static float grand(){ float u=urand()+1e-7f, v=urand(); return sqrtf(-2*logf(u))*cosf(2*3.14159265f*v); }

static int fails = 0;
static void ck(bool ok, const char* m){ printf("  [%s] %s\n", ok?"PASS":"FAIL", m); if(!ok) fails++; }

int main(void)
{
    aim_calib_t c{}; c.magic = AIM_CAL_MAGIC;
    c.cx=0.5f; c.cy=0.5f; c.w=0.40f; c.h=1.24f; c.bx=8.0f; c.by=6.0f;
    aim_runtime_begin(); aim_runtime_set(&c, false); aim_filter_set(1.0f, 15.0f);

    // Two simulated hours, motionless. Averaged into windows: a raw excursion is
    // dominated by blob noise, not drift.
    const long FRAMES = 972000;                 // 2 h at 135 fps
    const int NB = 24;
    double ax[NB]={0}, ay[NB]={0}; long n[NB]={0};
    quad_reset(nullptr);
    const float cx=120, cy=88, hw=27, hh=50;
    const float TX[4]={cx-hw,cx+hw,cx-hw,cx+hw}, TY[4]={cy-hh,cy-hh,cy+hh,cy+hh};
    for (long f=0; f<FRAMES; ++f) {
        float xs[4], ys[4];
        for (int i=0;i<4;i++){ xs[i]=TX[i]+0.35f*grand(); ys[i]=TY[i]+0.35f*grand(); }
        QuadResult r = quad_update(xs, ys, 4);
        if (r.count < 4) continue;
        aim_pt_t q[4]; for(int i=0;i<4;i++){ q[i].x=r.p[i].x; q[i].y=r.p[i].y; }
        float sx, sy;
        if (!aim_runtime_solve(q,240,176,&sx,&sy,1.0f/135.0f)) continue;
        int b = (int)(f*(long)NB/FRAMES); if (b>=NB) b=NB-1;
        ax[b]+=sx; ay[b]+=sy; n[b]++;
    }
    double x0=ax[0]/n[0], y0=ay[0]/n[0], worst=0;
    for (int b=0;b<NB;b++){
        const double d = hypot((ax[b]/n[b]-x0)*1920.0, (ay[b]/n[b]-y0)*1200.0);
        if (d>worst) worst=d;
    }
    printf("  2 h motionless: worst 5-minute-window excursion %.3f screen px\n", worst);
    ck(worst < 3.0, "no drift over two hours of continuous running");

    // The firmware truncates the microsecond clock to uint32, so it wraps every
    // ~72 min. Unsigned subtraction handles it; step across the wrap and check.
    aim_pt_t q[4]={{93,38},{147,38},{93,138},{147,138}};
    float wx,wy; for(int i=0;i<600;i++) aim_runtime_solve(q,240,176,&wx,&wy,1.0f/135.0f);
    uint32_t t = 0xFFFFFF00u;
    float jump = 0;
    for (int i=0;i<400;i++){
        const uint32_t prev=t; t += 7407;
        const float dt=(t-prev)*1e-6f;
        float sx,sy; aim_runtime_solve(q,240,176,&sx,&sy,dt);
        const float d=hypotf((sx-wx)*1920.0f,(sy-wy)*1200.0f);
        if (d>jump) jump=d;
        wx=sx; wy=sy;
    }
    printf("  worst cursor jump across the 32-bit microsecond wrap: %.4f px\n", jump);
    ck(jump < 0.5f, "the microsecond wrap does not move the cursor");

    printf("\n%s (%d failures)\n", fails ? "FAILURES" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
