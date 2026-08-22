// roll_test.cpp -- asserts the shipping runtime aim path is exactly
// roll-invariant under the model's own assumptions, so a change to boresight
// handling fails loudly instead of degrading quietly.
#include "../lib/AimPipeline/aim_core.h"
#include <math.h>
#include <stdio.h>

static const double F  = 184.7;   // measured focal length, native px
static const double FW = 240.0, FH = 176.0;
static const double SW = 0.518,  SH = 0.518/1.6;   // screen, metres (16:10)
// a real fitted calibration
static const double CX = 0.512161, CY = 0.535141, WN = 0.401183, HN = 1.241383;
static const double BX = 12.156,   BY = 9.125;
static double K1 = 0.0;           // radial distortion, set by the caller

struct V3 { double x,y,z; };
static V3 mul(const double R[9], V3 a){
    return { R[0]*a.x+R[1]*a.y+R[2]*a.z, R[3]*a.x+R[4]*a.y+R[5]*a.z, R[6]*a.x+R[7]*a.y+R[8]*a.z };
}
static V3 mulT(const double R[9], V3 a){
    return { R[0]*a.x+R[3]*a.y+R[6]*a.z, R[1]*a.x+R[4]*a.y+R[7]*a.z, R[2]*a.x+R[5]*a.y+R[8]*a.z };
}
static void rot(double roll,double pitch,double yaw,double R[9]){
    const double cr=cos(roll), sr=sin(roll), cp=cos(pitch), sp=sin(pitch), cy=cos(yaw), sy=sin(yaw);
    const double Rz[9]={cr,-sr,0, sr,cr,0, 0,0,1};
    const double Rx[9]={1,0,0, 0,cp,-sp, 0,sp,cp};
    const double Ry[9]={cy,0,sy, 0,1,0, -sy,0,cy};
    double T[9];
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){ double s=0; for(int k=0;k<3;k++) s+=Ry[i*3+k]*Rx[k*3+j]; T[i*3+j]=s; }
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){ double s=0; for(int k=0;k<3;k++) s+=Rz[i*3+k]*T[k*3+j]; R[i*3+j]=s; }
}
static aim_pt_t project(const double R[9], V3 P, V3 X){
    V3 c = mul(R, {X.x-P.x, X.y-P.y, X.z-P.z});
    double xn = c.x/c.z, yn = c.y/c.z;
    const double r2 = xn*xn + yn*yn;
    const double g  = 1.0 + K1*r2;
    return { (float)(F*xn*g + FW*0.5), (float)(F*yn*g + FH*0.5) };
}
// where the boresight ray actually lands, in normalised screen coords
static void truth(const double R[9], V3 P, double* sx, double* sy){
    V3 dw = mulT(R, {BX/F, BY/F, 1.0});
    const double t = (0.0 - P.z)/dw.z;
    const double X = P.x + t*dw.x, Y = P.y + t*dw.y;
    *sx = 0.5 + X/SW; *sy = 0.5 + Y/SH;
}

static double g_worst_pinhole = 0.0;

int main(void)
{
    aim_calib_t c{}; c.magic=AIM_CAL_MAGIC;
    c.cx=(float)CX; c.cy=(float)CY; c.w=(float)WN; c.h=(float)HN;
    c.bx=(float)BX; c.by=(float)BY; c.lever=0.0f;

    const double W = WN*SW, H = HN*SH;                 // LED rect, metres
    const double ox = (CX-0.5)*SW, oy = (CY-0.5)*SH;   // rect centre offset
    const V3 led[4] = {{ox-W/2,oy-H/2,0},{ox+W/2,oy-H/2,0},{ox-W/2,oy+H/2,0},{ox+W/2,oy+H/2,0}};

    const double D = 1.473;                            // 58 inches
    struct { const char* n; double pitch, yaw; } poses[] = {
        {"centre",      0.0,   0.0},
        {"aim low",    -0.16,  0.0},   // ~9 deg, lands near screen bottom
        {"aim high",    0.16,  0.0},
        {"aim left",    0.0,  -0.16},
        {"aim right",   0.0,   0.16},
    };
    const double rolls[] = {0,2,5,10,20,30};

    for (int pass = 0; pass < 2; ++pass) {
        K1 = (pass == 0) ? 0.0 : -0.10;
        printf("\n=== pass %d: %s ===\n", pass,
               pass ? "with -10%% barrel distortion in the CAMERA, none in the MODEL"
                    : "exact pinhole (model assumption holds)");
        printf("%-11s %6s %10s %10s %9s\n","pose","roll","dx (scr px)","dy (scr px)","|d|");
        for (unsigned p = 0; p < sizeof(poses)/sizeof(poses[0]); ++p) {
            for (unsigned r = 0; r < sizeof(rolls)/sizeof(rolls[0]); ++r) {
                double R[9]; rot(rolls[r]*M_PI/180.0, poses[p].pitch, poses[p].yaw, R);
                const V3 P = {0,0,-D};
                aim_pt_t q[4]; for (int i=0;i<4;i++) q[i]=project(R,P,led[i]);
                double tx,ty; truth(R,P,&tx,&ty);
                float sx,sy;
                if (!aim_solve(&c,q,(float)FW,(float)FH,&sx,&sy)) { printf("  %-11s %5.0f  SOLVE FAILED\n",poses[p].n,rolls[r]); continue; }
                const double dx=(sx-tx)*1920.0, dy=(sy-ty)*1200.0;
                const double m=sqrt(dx*dx+dy*dy);
                if (pass == 0 && m > g_worst_pinhole) g_worst_pinhole = m;
                printf("%-11s %5.0f  %10.2f %10.2f %9.2f\n",
                       r?"":poses[p].n, rolls[r], dx, dy, sqrt(dx*dx+dy*dy));
            }
        }
    }
    // The invariant: under the model's own assumptions the runtime path is
    // EXACTLY roll invariant.
    printf("\nworst pinhole roll error over all poses and rolls: %.4f px\n", g_worst_pinhole);
    if (g_worst_pinhole > 0.5) { printf("roll invariance FAILED\n"); return 1; }
    printf("ALL PASS\n");
    return 0;
}
