// Host validation of aim_core against a synthetic camera with known truth: the
// quad algebra, the single-distance degeneracy gate, calibration accuracy across
// distances, and the runtime solve path.
#include "../lib/AimPipeline/aim_core.h"
#include <stdio.h>
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>

static const float FW = 240.0f, FH = 176.0f, FPX = 184.7f;
static const double SW = 0.597, SH = 0.336;          // 27" 16:9, metres
static const double RIG_W = 0.208, RIG_H = 0.402;    // truth, never told to the solver
static const double B_TRUE_X = 5.0, B_TRUE_Y = -3.0; // truth
static const double LEVER_M  = 0.06;                 // camera 6cm below aim axis

static std::mt19937 rng(12345);
static double gauss(double s){ std::normal_distribution<double> d(0.0,s); return s>0? d(rng):0.0; }

struct V3 { double x,y,z; };
static V3 sub(V3 a,V3 b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
static double dot(V3 a,V3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static V3 cross(V3 a,V3 b){ return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
static V3 norm(V3 a){ double n=sqrt(dot(a,a)); return {a.x/n,a.y/n,a.z/n}; }
static V3 mul(double M[9], V3 v){ return {M[0]*v.x+M[1]*v.y+M[2]*v.z,
                                          M[3]*v.x+M[4]*v.y+M[5]*v.z,
                                          M[6]*v.x+M[7]*v.y+M[8]*v.z}; }

// unit direction through image px (u,v)
static V3 ray(double u,double v){ return norm({(u-FW/2)/FPX, -(v-FH/2)/FPX, -1.0}); }

// rotation taking a -> w (Rodrigues, shortest arc)
static void rot_a_to_w(V3 a, V3 w, double R[9]){
    V3 v=cross(a,w); double s=sqrt(dot(v,v)), c=dot(a,w);
    if(s<1e-12){ for(int i=0;i<9;++i) R[i]=(i%4==0)?1.0:0.0; return; }
    double vx[9]={0,-v.z,v.y, v.z,0,-v.x, -v.y,v.x,0};
    double vv[9];
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){ double t=0;
        for(int k=0;k<3;++k) t+=vx[i*3+k]*vx[k*3+j]; vv[i*3+j]=t; }
    double f=(1-c)/(s*s);
    for(int i=0;i<9;++i) R[i]=((i%4==0)?1.0:0.0)+vx[i]+vv[i]*f;
}

// project the 4 LEDs, camera centre C, rotation R (camera->world)
static void project(V3 C, double R[9], double bs, aim_pt_t out[4]){
    const double hw=RIG_W/2, hh=RIG_H/2;
    V3 L[4]={{-hw,+hh,0},{+hw,+hh,0},{-hw,-hh,0},{+hw,-hh,0}};  // TL TR BL BR
    double Rt[9]={R[0],R[3],R[6], R[1],R[4],R[7], R[2],R[5],R[8]};
    for(int i=0;i<4;++i){
        V3 pc=mul(Rt, sub(L[i],C));
        double z=-pc.z;
        out[i].x=(float)(FW/2 + FPX*pc.x/z + gauss(bs));
        out[i].y=(float)(FH/2 - FPX*pc.y/z + gauss(bs));
    }
}

// one trigger pull: user aims at (tx,ty) normalised screen, from `dist`
static void shoot(double tx,double ty,double dist,double aim_sd_deg,double blob_sd,
                  aim_pt_t out[4]){
    double Xm=(tx-0.5)*SW, Ym=(0.5-ty)*SH;
    V3 eye={gauss(0.06), gauss(0.04), dist};
    static double dr_x=0, dr_y=0;   /* slow aim drift, AR(1) */
    dr_x=0.90*dr_x+gauss(0.35); dr_y=0.90*dr_y+gauss(0.35);
    double jx=tan((dr_x*aim_sd_deg+gauss(aim_sd_deg*0.3))*M_PI/180.0)*dist,
           jy=tan((dr_y*aim_sd_deg+gauss(aim_sd_deg*0.3))*M_PI/180.0)*dist;
    V3 T={Xm+jx, Ym+jy, 0.0};
    V3 a=ray(FW/2+B_TRUE_X, FH/2+B_TRUE_Y);
    double R[9]; rot_a_to_w(a, norm(sub(T,eye)), R);
    V3 camoff=mul(R,{0,-LEVER_M,0});
    project({eye.x+camoff.x,eye.y+camoff.y,eye.z+camoff.z}, R, blob_sd, out);
}

static int fails=0;
static void ck(bool ok,const char*msg){ printf("  [%s] %s\n", ok?"PASS":"FAIL", msg); if(!ok) fails++; }

// average N trigger pulls into one shot
static void avg_shot(double tx,double ty,double d,double asd,double bsd,int n,aim_pt_t o[4]){
    double ax[4]={0,0,0,0}, ay[4]={0,0,0,0};
    for(int k=0;k<n;++k){ aim_pt_t q[4]; shoot(tx,ty,d,asd,bsd,q);
        for(int i=0;i<4;++i){ ax[i]+=q[i].x; ay[i]+=q[i].y; } }
    for(int i=0;i<4;++i){ o[i].x=(float)(ax[i]/n); o[i].y=(float)(ay[i]/n); }
}

struct Res { double rms, p95; };
static Res evaluate(const aim_calib_t* c, double dist, double blob_sd, int n){
    std::vector<double> e;
    const double T[5][2]={{0.5,0.5},{0.1,0.1},{0.9,0.1},{0.1,0.9},{0.9,0.9}};
    for(int t=0;t<5;++t) for(int k=0;k<n;++k){
        aim_pt_t q[4]; shoot(T[t][0],T[t][1],dist,0.0,blob_sd,q);
        float sx,sy; if(!aim_solve(c,q,FW,FH,&sx,&sy)) continue;
        double dx=(sx-T[t][0])*1920.0, dy=(sy-T[t][1])*1080.0;
        e.push_back(sqrt(dx*dx+dy*dy));
    }
    if(e.empty()) return {1e9,1e9};
    std::sort(e.begin(),e.end());
    double s=0; for(double v:e) s+=v;
    return { s/e.size(), e[(size_t)(0.95*(e.size()-1))] };
}

int main(){
    printf("aim_core host validation  (27in screen, rig 20.8x40.2cm, boresight (5.0,-3.0))\n\n");

    // ---- 1. Heckbert round trip -----------------------------------------
    printf("1. quad->square algebra\n");
    { aim_pt_t q[4]={{40,30},{200,25},{35,150},{205,158}};
      float u,v; bool ok=aim_quad_to_square(q,40,30,&u,&v);
      ck(ok && fabsf(u)<1e-4f && fabsf(v)<1e-4f, "TL maps to (0,0)");
      ok=aim_quad_to_square(q,205,158,&u,&v);
      ck(ok && fabsf(u-1)<1e-4f && fabsf(v-1)<1e-4f, "BR maps to (1,1)");
      ok=aim_quad_to_square(q,200,25,&u,&v);
      ck(ok && fabsf(u-1)<1e-4f && fabsf(v)<1e-4f, "TR maps to (1,0)");
      aim_pt_t deg[4]={{10,10},{10,10},{10,10},{10,10}};
      ck(!aim_quad_to_square(deg,10,10,&u,&v), "degenerate quad rejected"); }

    // ---- 2. the degeneracy gate -----------------------------------------
    printf("\n2. single-stance calibration must be REJECTED\n");
    { aim_shotset_t ss; aim_shots_reset(&ss);
      const double G[5][2]={{0.08,0.08},{0.92,0.08},{0.08,0.92},{0.92,0.92},{0.5,0.5}};
      for(int t=0;t<5;++t){ aim_pt_t q[4]; avg_shot(G[t][0],G[t][1],1.473,0.3,0.7,6,q);
                            aim_shots_add(&ss,q,G[t][0],G[t][1]); }
      aim_calib_t c;
      int ok=aim_calib_fit(&ss,FW,FH,0,&c);
      printf("      span spread = %.3f (gate is 1.15)\n", aim_shots_span_spread(&ss));
      ck(!ok, "one distance -> fit refused, not silently wrong"); }

    // ---- 3. two and three distances --------------------------------------
    printf("\n3. multi-distance calibration\n");
    const double G[5][2]={{0.08,0.08},{0.92,0.08},{0.08,0.92},{0.92,0.92},{0.5,0.5}};
    struct { const char* name; int nd; double d[3]; } procs[] = {
        {"2 distances (1.2 / 2.2 m)", 2, {1.2,2.2,0}},
        {"3 distances (1.1/1.7/2.5)", 3, {1.1,1.7,2.5}},
    };
    for(auto& pr : procs){
        aim_shotset_t ss; aim_shots_reset(&ss);
        for(int s=0;s<pr.nd;++s) for(int t=0;t<5;++t){
            aim_pt_t q[4]; avg_shot(G[t][0],G[t][1],pr.d[s],0.3,0.7,6,q);
            aim_shots_add(&ss,q,G[t][0],G[t][1]); }
        aim_calib_t c;
        int ok=aim_calib_fit(&ss,FW,FH,0,&c);
        if(!ok){ printf("  [FAIL] %s -> fit refused\n",pr.name); fails++; continue; }
        Res a=evaluate(&c,1.2,0.7,200), b=evaluate(&c,1.7,0.7,200), d=evaluate(&c,2.5,0.7,200);
        printf("  %s\n", pr.name);
        printf("      boresight (%6.2f,%6.2f)  truth (5.00,-3.00)   err %.2f px\n",
               c.bx,c.by, sqrt((c.bx-5.0)*(c.bx-5.0)+(c.by+3.0)*(c.by+3.0)));
        printf("      LED rect  %.3f x %.3f of screen  (truth %.3f x %.3f)\n",
               c.w,c.h, RIG_W/SW, RIG_H/SH);
        printf("      fit rms %.5f    aim err rms/p95:  1.2m %4.0f/%4.0f   1.7m %4.0f/%4.0f   2.5m %4.0f/%4.0f\n",
               c.fit_rms, a.rms,a.p95, b.rms,b.p95, d.rms,d.p95);
        ck(sqrt((c.bx-5.0)*(c.bx-5.0)+(c.by+3.0)*(c.by+3.0)) < 4.0,
           "boresight recovered to better than 4 px");
        ck(a.p95 < 130 && d.p95 < 260, "aim error holds across distances");
        ck(fabsf(c.w - (float)(RIG_W/SW)) < 0.10f, "LED rect width recovered");
    }

    // ---- 4. blob noise sensitivity ---------------------------------------
    printf("\n4. sensitivity to blob sigma (calibrated at 1.2/2.2 m, evaluated at 1.473 m)\n");
    for(double bs : {0.2,0.5,0.7,1.0}){
        aim_shotset_t ss; aim_shots_reset(&ss);
        for(double d : {1.2,2.2}) for(int t=0;t<5;++t){
            aim_pt_t q[4]; avg_shot(G[t][0],G[t][1],d,0.3,bs,6,q);
            aim_shots_add(&ss,q,G[t][0],G[t][1]); }
        aim_calib_t c;
        if(!aim_calib_fit(&ss,FW,FH,0,&c)){ printf("      sigma %.1f -> refused\n",bs); continue; }
        Res r=evaluate(&c,1.473,bs,400);
        printf("      blob sigma %.1f px -> aim err %5.1f rms / %5.1f p95 screen px  (bore err %.2f px)\n",
               bs, r.rms, r.p95, sqrt((c.bx-5.0)*(c.bx-5.0)+(c.by+3.0)*(c.by+3.0)));
    }

    // ---- 5. runtime path is float32-clean and off-screen-safe -------------
    printf("\n5. runtime path\n");
    { aim_shotset_t ss; aim_shots_reset(&ss);
      for(double d : {1.2,2.2}) for(int t=0;t<5;++t){
          aim_pt_t q[4]; avg_shot(G[t][0],G[t][1],d,0.3,0.5,6,q);
          aim_shots_add(&ss,q,G[t][0],G[t][1]); }
      aim_calib_t c; aim_calib_fit(&ss,FW,FH,0,&c);
      aim_pt_t q[4]; shoot(0.5,0.5,1.473,0,0,q);
      float sx,sy; ck(aim_solve(&c,q,FW,FH,&sx,&sy)!=0, "solves at screen centre");
      ck(fabsf(sx-0.5f)<0.05f && fabsf(sy-0.5f)<0.05f, "centre lands near 0.5,0.5");
      // aiming well off screen must still produce a finite answer outside 0..1
      shoot(2.5,0.5,1.473,0,0,q);
      int ok=aim_solve(&c,q,FW,FH,&sx,&sy);
      ck(ok && sx>1.0f, "off-screen aim reported as >1, not clamped");
      aim_calib_t bad={}; ck(aim_solve(&bad,q,FW,FH,&sx,&sy)==0, "invalid calib refused"); }

    printf("\n%s  (%d failures)\n", fails? "FAILURES PRESENT":"ALL PASS", fails);
    return fails?1:0;
}
