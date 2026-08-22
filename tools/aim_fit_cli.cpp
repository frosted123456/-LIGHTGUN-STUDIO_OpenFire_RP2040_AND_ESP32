// Host CLI around aim_core's fit. Reads shots on stdin:
//    frame_w frame_h
//    tx ty  x0 y0 x1 y1 x2 y2 x3 y3      (one line per shot, native px)
#include "../lib/AimPipeline/aim_core.h"
#include <stdio.h>
#include <math.h>
int main(int argc,char**argv){
    float fw,fh; if(scanf("%f %f",&fw,&fh)!=2){ fprintf(stderr,"bad header\n"); return 2; }
    aim_shotset_t ss; aim_shots_reset(&ss);
    float tx,ty; aim_pt_t q[4];
    while(scanf("%f %f %f %f %f %f %f %f %f %f",&tx,&ty,
        &q[0].x,&q[0].y,&q[1].x,&q[1].y,&q[2].x,&q[2].y,&q[3].x,&q[3].y)==10)
        aim_shots_add(&ss,q,tx,ty);
    printf("shots=%u  span_spread=%.3f\n",ss.n,aim_shots_span_spread(&ss));
    aim_calib_t c;
    int lever = (argc>1 && argv[1][0]=='L');
    if(!aim_calib_fit(&ss,fw,fh,lever,&c)){
        printf("FIT REFUSED");
        if(aim_shots_span_spread(&ss)<1.15f)
            printf(" -- span spread %.3f < 1.15: all shots at one distance.\n"
                   "   The boresight is not identifiable from a single stance.\n",
                   aim_shots_span_spread(&ss));
        else printf(" -- see aim_calib_fit gates (too few shots, or implausible rect)\n");
        return 1;
    }
    printf("\n--- CALIBRATION ---\n");
    printf("  boresight      bx=%+.2f  by=%+.2f   native px from frame centre\n",c.bx,c.by);
    printf("  aimcal line    aimcal=%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.6f,%.6f,%.6f\n",
           c.cx,c.cy,c.w,c.h,c.bx,c.by,c.lever,c.rx,c.ry);
    printf("  LED rectangle  %.4f x %.4f of screen  (centre %.4f, %.4f)\n",c.w,c.h,c.cx,c.cy);
    printf("  fit rms        %.5f  (= %.1f px on a 1920x1080 screen)\n",
           c.fit_rms, c.fit_rms*sqrtf(1920.f*1920.f+1080.f*1080.f)/sqrtf(2.f));
    printf("  shots used     %u   rejected as outliers %d\n",c.n_shots,aim_calib_n_rejected(&c));
    printf("  span spread    %.3f\n",c.fit_spread);
    printf("  roll spread    %.4f%s\n",c.fit_roll,
           (c.rx!=0.0f||c.ry!=0.0f)?"  (roll term fitted)":"  (below gate, roll term off)");
    printf("  roll coeffs    rx=%+.6f ry=%+.6f\n",c.rx,c.ry);
    if(c.lever!=0.0f) printf("  lever          %+.4f px per px of quad span\n",c.lever);
    printf("\n--- per-shot residual (screen px on 1920x1080) ---\n");
    for(int i=0;i<ss.n;++i){
        float sx,sy;
        if(!aim_solve(&c,ss.s[i].q,fw,fh,&sx,&sy)){ printf("  shot %2d  UNSOLVED\n",i); continue; }
        printf("  shot %2d  target(%.3f,%.3f)  got(%.3f,%.3f)  err %5.1f,%5.1f px\n",
               i,ss.s[i].tx,ss.s[i].ty,sx,sy,
               (sx-ss.s[i].tx)*1920.f,(sy-ss.s[i].ty)*1080.f);
    }
    return 0;
}
