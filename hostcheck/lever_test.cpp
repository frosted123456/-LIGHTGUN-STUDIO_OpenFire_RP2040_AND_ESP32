// lever_test.cpp -- what a camera physically OFFSET from the barrel costs.
// aim_core models the boresight as a pure ANGULAR offset, so a translational
// lever arm is absorbed into it at the calibration distances; this measures the
// residual left everywhere else, and that a static mount tilt costs nothing.
#include "../lib/AimPipeline/aim_core.h"
#include <math.h>
#include <stdio.h>

static const double F=184.7, FW=240.0, FH=176.0;
static const double SW=0.518, SH=0.518/1.6;
static const double CX=0.512161, CY=0.535141, WN=0.401183, HN=1.241383;

struct V3{double x,y,z;};
static V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static V3 add(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 crs(V3 a,V3 b){return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};}
static V3 nrm(V3 a){double n=sqrt(a.x*a.x+a.y*a.y+a.z*a.z);return {a.x/n,a.y/n,a.z/n};}
static V3 scl(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}

// world->camera rotation whose optical axis is d, with the given roll
static void frame_from(V3 d, double roll, double R[9]){
    V3 zc=nrm(d);
    V3 xc=nrm(crs({0,1,0}, zc));   // world y is DOWN; cross(y,z)=+x keeps handedness
    V3 yc=crs(zc,xc);
    const double c=cos(roll), s=sin(roll);
    V3 xr=add(scl(xc,c),scl(yc,s)), yr=add(scl(yc,c),scl(xc,-s));
    R[0]=xr.x;R[1]=xr.y;R[2]=xr.z; R[3]=yr.x;R[4]=yr.y;R[5]=yr.z; R[6]=zc.x;R[7]=zc.y;R[8]=zc.z;
}
static V3 mul(const double R[9],V3 a){return {R[0]*a.x+R[1]*a.y+R[2]*a.z, R[3]*a.x+R[4]*a.y+R[5]*a.z, R[6]*a.x+R[7]*a.y+R[8]*a.z};}
static V3 mulT(const double R[9],V3 a){return {R[0]*a.x+R[3]*a.y+R[6]*a.z, R[1]*a.x+R[4]*a.y+R[7]*a.z, R[2]*a.x+R[5]*a.y+R[8]*a.z};}

static V3 LED[4];
static V3 LEVER;                     // camera optical centre, in the gun frame
static double TILT = 0.0;            // STATIC camera-in-mount tilt (rad): rotates
                                     // the camera but NOT the lever

// The user holds the gun at B and aims the BARREL at the on-screen dot.
// Returns the quad the camera then sees.
static void observe(V3 B, double tx, double ty, double roll, aim_pt_t q[4]){
    const V3 dot = {(tx-0.5)*SW, (ty-0.5)*SH, 0.0};
    double R[9]; frame_from(sub(dot,B), roll, R);
    const V3 P = add(B, mulT(R, LEVER));          // where the camera actually is
    double Rc[9]; frame_from(sub(dot,B), roll + TILT, Rc);   // crooked in its mount
    for(int i=0;i<4;i++){
        V3 c = mul(Rc, sub(LED[i],P));
        q[i].x=(float)(F*c.x/c.z + FW*0.5);
        q[i].y=(float)(F*c.y/c.z + FH*0.5);
    }
}

int main(void)
{
    const double W=WN*SW, H=HN*SH, ox=(CX-0.5)*SW, oy=(CY-0.5)*SH;
    LED[0]={ox-W/2,oy-H/2,0}; LED[1]={ox+W/2,oy-H/2,0};
    LED[2]={ox-W/2,oy+H/2,0}; LED[3]={ox+W/2,oy+H/2,0};

    struct { const char* n; double lx, ly; } cases[] = {
        {"co-axial (ideal)",        0.00,  0.00},
        {"box 3cm left, 5cm up",   -0.03, -0.05},
        {"box 6cm left, 8cm up",   -0.06, -0.08},
    };
    const double dots[5][2]={{0.5,0.5},{0.15,0.15},{0.85,0.15},{0.15,0.85},{0.85,0.85}};
    const double cal_D[3]={1.20,1.60,2.00};
    const double ver[9][2]={{.12,.12},{.5,.12},{.88,.12},{.12,.5},{.5,.5},{.88,.5},{.12,.88},{.5,.88},{.88,.88}};

    int fails = 0;
    double coax_worst = 0.0, tilt_delta = 0.0, tilted_mean[2] = {0,0};

    for (unsigned ci=0; ci<sizeof(cases)/sizeof(cases[0]); ++ci) {
        LEVER = {cases[ci].lx, cases[ci].ly, 0.0};
        aim_shotset_t ss; aim_shots_reset(&ss);
        for (int d=0; d<3; ++d)
            for (int k=0; k<5; ++k) {
                aim_pt_t q[4]; observe({0,0,-cal_D[d]}, dots[k][0], dots[k][1], 0.0, q);
                aim_shots_add(&ss,q,(float)dots[k][0],(float)dots[k][1]);
            }
        for (int lv=0; lv<2; ++lv) {
            aim_calib_t c{};
            if (!aim_calib_fit(&ss,(float)FW,(float)FH,lv,&c)) { printf("%-24s lever=%d  FIT FAILED\n",cases[ci].n,lv); continue; }
            // verify at a distance the calibration never saw
            double sum=0, worst=0; double ex[9], ey[9];
            for (int k=0;k<9;++k){
                aim_pt_t q[4]; observe({0,0,-1.473}, ver[k][0], ver[k][1], 0.0, q);
                float sx,sy; aim_solve(&c,q,(float)FW,(float)FH,&sx,&sy);
                ex[k]=(sx-ver[k][0])*1920.0; ey[k]=(sy-ver[k][1])*1200.0;
                const double m=sqrt(ex[k]*ex[k]+ey[k]*ey[k]); sum+=m; if(m>worst)worst=m;
            }
            // regress err_x on (1, x-.5, y-.5) to expose the shape
            double n=9,sx1=0,sy1=0,sxx=0,syy=0,sxy=0,se=0,sex=0,sey=0;
            for(int k=0;k<9;++k){double X=ver[k][0]-.5,Y=ver[k][1]-.5;
                sx1+=X;sy1+=Y;sxx+=X*X;syy+=Y*Y;sxy+=X*Y;se+=ex[k];sex+=ex[k]*X;sey+=ex[k]*Y;}
            const double det=sxx*syy-sxy*sxy;
            const double bxs=(sey*sxy - sex*syy)/(sxy*sxy - sxx*syy);
            const double bys=(sex*sxy - sey*sxx)/(sxy*sxy - sxx*syy);
            (void)det;(void)n;(void)sx1;(void)sy1;
            printf("%-24s lever=%d  mean %6.1f px  worst %6.1f px   err_x = %+6.1f %+6.1f*(x-.5) %+6.1f*(y-.5)   fit_rms %.5f\n",
                   ci? (lv?"":cases[ci].n):cases[ci].n, lv, sum/9.0, worst, se/9.0, bxs, bys, c.fit_rms);
            if (ci == 0 && worst > coax_worst) coax_worst = worst;
            if (ci == 2 && lv == 0) tilted_mean[TILT != 0.0 ? 1 : 0] = sum/9.0;
        }
    }
    // ---- invariants -------------------------------------------------------
    // 1. A camera ON the sight line has nothing to correct, at any pose.
    printf("\nco-axial worst error: %.3f px\n", coax_worst);
    if (coax_worst > 0.5) { printf("  [FAIL] co-axial camera should be exact\n"); fails++; }

    // 2. A camera glued in CROOKED costs nothing: a constant tilt is absorbed by
    //    the fit's intercept, so the mount does not need shimming.
    LEVER = { -0.06, -0.08, 0.0 };
    double before = 0.0, after = 0.0;
    for (int pass = 0; pass < 2; ++pass) {
        TILT = pass ? (12.0 * M_PI / 180.0) : 0.0;
        aim_shotset_t ss; aim_shots_reset(&ss);
        for (int d=0; d<3; ++d)
            for (int k=0; k<5; ++k) {
                aim_pt_t q[4]; observe({0,0,-cal_D[d]}, dots[k][0], dots[k][1], 0.0, q);
                aim_shots_add(&ss,q,(float)dots[k][0],(float)dots[k][1]);
            }
        aim_calib_t c{};
        if (!aim_calib_fit(&ss,(float)FW,(float)FH,0,&c)) { printf("  [FAIL] tilt pass %d did not fit\n", pass); fails++; continue; }
        double sum = 0;
        for (int k=0;k<9;++k) {
            aim_pt_t q[4]; observe({0,0,-1.473}, ver[k][0], ver[k][1], 0.0, q);
            float sx,sy; aim_solve(&c,q,(float)FW,(float)FH,&sx,&sy);
            sum += sqrt(pow((sx-ver[k][0])*1920.0,2) + pow((sy-ver[k][1])*1200.0,2));
        }
        (pass ? after : before) = sum/9.0;
    }
    TILT = 0.0;
    tilt_delta = fabs(after - before);
    printf("static 12 deg mount tilt: %.2f px -> %.2f px  (delta %.3f)\n",
           before, after, tilt_delta);
    if (tilt_delta > 1.0) {
        printf("  [FAIL] a constant mount tilt should be absorbed by the fit\n"); fails++;
    }
    (void)tilted_mean;
    printf("\n%s\n", fails ? "lever_test FAILED" : "ALL PASS");
    return fails ? 1 : 0;
}
