// Host test for aim_runtime: the serial command surface and the guards that
// stop a bad calibration reaching the hot path.
#include "../lib/AimPipeline/aim_runtime.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int fails=0;
static void ck(bool ok,const char*m){printf("  [%s] %s\n",ok?"PASS":"FAIL",m); if(!ok)fails++;}
int main(){
    printf("aim_runtime host validation\n\n");
    aim_runtime_begin();
    aim_filter_set(0.0f, 0.0f);   // determinism: the filter is tested separately
    ck(!aim_runtime_active(), "starts inactive with nothing stored");

    aim_pt_t q[4]={{100,60},{140,61},{101,120},{141,121}};
    float sx,sy;
    ck(!aim_runtime_solve(q,240,176,&sx,&sy,0.0f), "solve refuses with no calibration");

    printf("\nOne Euro speed sensitivity (beta):\n");
    aim_smooth_set(3);
    ck(aim_beta_get() == -1, "beta follows the smoothing table by default");
    ck(aim_filter_beta() == 15.0f, "which is 15 at level 3");
    aim_beta_set(30);
    ck(aim_beta_get() == 30 && aim_filter_beta() == 30.0f,
       "an override takes effect immediately");
    ck(aim_filter_min_cutoff() == 3.5f,
       "and leaves the at-rest cutoff alone -- beta is speed only");
    aim_smooth_set(7);
    ck(aim_filter_beta() == 30.0f && aim_filter_min_cutoff() == 1.55f,
       "the override survives a smoothing change");
    aim_beta_set(999);
    ck(aim_beta_get() == 60, "beta clamps high");
    aim_beta_set(-1);
    ck(aim_beta_get() == -1 && aim_filter_beta() == 15.0f,
       "-1 hands beta back to the table");
    aim_smooth_set(3);
    aim_filter_set(0.0f, 0.0f);   // restore the deterministic setup

    printf("\nserial surface:\n");
    ck(!aim_runtime_command("thr=20"),        "unrelated line not claimed");
    ck(aim_runtime_command("aimcal?"),        "aimcal? claimed");
    // a real fitted calibration
    ck(aim_runtime_command("aimcal=0.504600,0.298800,0.351300,1.200300,4.800,-2.120"),
                                              "real calibration accepted");
    ck(aim_runtime_active(),                  "now active");
    const aim_calib_t* c=aim_runtime_calib();
    ck(fabsf(c->bx-4.80f)<1e-3f && fabsf(c->w-0.3513f)<1e-5f, "values round-tripped");
    ck(aim_runtime_solve(q,240,176,&sx,&sy,0.0f),   "solve works");
    printf("       -> screen (%.4f, %.4f)\n",sx,sy);

    // The 9-number form carries the roll coefficients: a parser that stops at 6
    // silently discards the whole roll correction.
    printf("\npreview (non-persisting) install:\n");
    ck(aim_runtime_command("aimcal!=0.4,0.4,0.30,1.10,1.5,2.5"),
                                              "aimcal!= claimed");
    c = aim_runtime_calib();
    ck(fabsf(c->w-0.30f)<1e-5f,               "preview values are live");
    ck(aim_runtime_command("aimcal=0.504600,0.298800,0.351300,1.200300,4.800,-2.120"),
                                              "normal form still works after a preview");

    printf("\npointer gate:\n");
    ck(aim_runtime_hid_enabled(),             "boots ENABLED -- a crash must never leave a dead cursor");
    ck(aim_runtime_command("aimhid=0"),       "aimhid=0 claimed");
    ck(!aim_runtime_hid_enabled(),            "pointer frozen");
    ck(aim_runtime_command("aimhid?"),        "aimhid? claimed");
    ck(aim_runtime_command("~aimhid=1"),      "tilde form claimed too");
    ck(aim_runtime_hid_enabled(),             "pointer released");
    // a trailing '!' is a legacy form that must still be tolerated, not rejected
    ck(aim_runtime_command("aimhid=1!"),      "legacy aimhid=1! still claimed");
    ck(aim_runtime_hid_enabled(),             "...and still releases the pointer");

    // the gate must not touch the geometry, or freezing the cursor would move
    // where the gun thinks it is pointing
    aim_runtime_command("aimhid=0");
    float gx,gy; const bool solved = aim_runtime_solve(q,240,176,&gx,&gy,0.0f);
    aim_runtime_command("aimhid=1");
    float hx,hy; aim_runtime_solve(q,240,176,&hx,&hy,0.0f);
    ck(solved && gx==hx && gy==hy,            "the gate does not affect the solved position");

    printf("\nroll coefficients over the wire:\n");
    ck(aim_runtime_command(
         "aimcal=0.504600,0.298800,0.351300,1.200300,4.800,-2.120,0.000000,-0.100500,0.002000"),
                                              "9-number form accepted");
    c = aim_runtime_calib();
    ck(fabsf(c->rx+0.1005f)<1e-5f && fabsf(c->ry-0.002f)<1e-5f,
                                              "rx,ry round-tripped");
    float rsx,rsy;
    aim_runtime_solve(q,240,176,&rsx,&rsy,0.0f);
    ck(fabsf(rsx-sx)>1e-4f,                   "the roll term actually moves the output");
    // and a malformed count must be refused rather than half-applied
    ck(aim_runtime_command("aimcal=0.5,0.3,0.35,1.2,4.8,-2.1,0.0,-0.1"),
                                              "8-number form claimed");
    c = aim_runtime_calib();
    ck(fabsf(c->rx+0.1005f)<1e-5f,            "...but rejected, leaving the old one intact");
    ck(aim_runtime_command("aimcal=0.504600,0.298800,0.351300,1.200300,4.800,-2.120"),
                                              "6-number form still accepted");
    c = aim_runtime_calib();
    ck(c->rx==0.0f && c->ry==0.0f,            "6-number form clears the roll term");

    printf("\nguards (each of these must be REFUSED):\n");
    struct { const char* cmd; const char* why; } bad[] = {
      {"aimcal=0.5,0.5,0.0,1.0,0,0",              "zero width"},
      {"aimcal=0.5,0.5,50.0,1.0,0,0",             "absurd width"},
      {"aimcal=0.5,0.5,0.35,1.2,999,0",           "boresight way off frame"},
      {"aimcal=0.5,0.5,0.35,1.2,0,-999",          "boresight way off frame (y)"},
      {"aimcal=999,0.5,0.35,1.2,0,0",             "rect centre off in space"},
      {"aimcal=0.5,0.5,0.35",                     "too few numbers"},
      {"aimcal=nan,0.5,0.35,1.2,0,0",             "NaN"},
    };
    for (auto& b : bad) {
        aim_runtime_command("aimcal=0.5046,0.2988,0.3513,1.2003,4.8,-2.12");  // known good
        const aim_calib_t before=*aim_runtime_calib();
        aim_runtime_command(b.cmd);
        const aim_calib_t after=*aim_runtime_calib();
        ck(memcmp(&before,&after,sizeof(before))==0, b.why);
    }

    printf("\ntoggles:\n");
    aim_runtime_command("aimcal=0.5046,0.2988,0.3513,1.2003,4.8,-2.12");
    aim_runtime_command("aimcal=off");
    ck(!aim_runtime_active(), "aimcal=off deactivates");
    ck(!aim_runtime_solve(q,240,176,&sx,&sy,0.0f), "and the hot path really stops");
    aim_runtime_command("aimcal=on");
    ck(aim_runtime_active(), "aimcal=on restores without re-sending values");
    aim_runtime_command("aimcal=clear");
    ck(!aim_runtime_active(), "aimcal=clear forgets it");

    printf("\ndegenerate quad on the hot path:\n");
    aim_runtime_command("aimcal=0.5046,0.2988,0.3513,1.2003,4.8,-2.12");
    aim_pt_t deg[4]={{10,10},{10,10},{10,10},{10,10}};
    ck(!aim_runtime_solve(deg,240,176,&sx,&sy,0.0f), "collapsed quad refused (caller falls back)");

    printf("\noff-screen aim must be reported, not clamped:\n");
    aim_pt_t far_[4]={{200,60},{240,61},{201,120},{241,121}};
    bool ok=aim_runtime_solve(far_,240,176,&sx,&sy,0.0f);
    printf("       -> ok=%d screen (%.3f, %.3f)\n",ok,sx,sy);
    ck(ok && (sx<0.0f||sx>1.0f||sy<0.0f||sy>1.0f), "produces an out-of-range coord");

    printf("\noutput filter:\n");
    aim_runtime_command("aimcal=0.5046,0.2988,0.3513,1.2003,4.8,-2.12");
    ck(aim_runtime_command("aimfilt?"), "aimfilt? claimed");
    ck(aim_runtime_command("aimfilt=1.0,15.0"), "aimfilt= sets both params");
    ck(aim_filter_min_cutoff()==1.0f && aim_filter_beta()==15.0f, "params stored");
    // a steady input must converge to itself: a biased filter would shift the
    // whole calibration
    aim_filter_reset();
    float lx=0, ly=0;
    for(int i=0;i<600;++i) aim_runtime_solve(q,240,176,&lx,&ly,1.0f/135.0f);
    aim_filter_set(0.0f,0.0f);
    float ux=0, uy=0; aim_runtime_solve(q,240,176,&ux,&uy,0.0f);
    printf("       filtered %.6f,%.6f  unfiltered %.6f,%.6f\n",lx,ly,ux,uy);
    ck(fabsf(lx-ux)<1e-4f && fabsf(ly-uy)<1e-4f,
       "a constant input converges to the unfiltered value (no steady-state bias)");
    aim_runtime_command("aimfilt=0");
    ck(aim_filter_min_cutoff()==0.0f, "aimfilt=0 disables it");

    printf("\n%s (%d failures)\n", fails?"FAILURES":"ALL PASS", fails);
    return fails?1:0;
}
