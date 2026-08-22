// Prove the calibration is invariant to the resolver's slot order -- all 24
// permutations of the same four points must give the identical screen coord.
#include "aim_runtime.h"
#include <stdio.h>
#include <math.h>
#include <algorithm>
int main(){
    aim_runtime_begin();
    // The output filter makes aim_runtime_solve STATEFUL, so 24 consecutive
    // solves would each differ. Geometry invariance is a property of the
    // unfiltered path.
    aim_filter_set(0.0f, 0.0f);
    aim_runtime_command("aimcal=0.504600,0.298800,0.351300,1.200300,4.800,-2.120");
    // a real captured shot, in the TL TR BL BR order the log happened to use
    aim_pt_t base[4]={{147.600f,66.450f},{180.750f,65.400f},
                      {147.350f,129.050f},{181.800f,130.800f}};
    int p[4]={0,1,2,3};
    float rx=0, ry=0; int n=0, bad=0;
    do {
        aim_pt_t q[4];
        for(int i=0;i<4;++i) q[i]=base[p[i]];
        float sx,sy;
        if(!aim_runtime_solve(q,240,176,&sx,&sy,0.0f)){ printf("perm %d UNSOLVED\n",n); bad++; continue; }
        if(n==0){ rx=sx; ry=sy; }
        else if(fabsf(sx-rx)>1e-6f || fabsf(sy-ry)>1e-6f){
            printf("perm %d differs: (%.6f,%.6f) vs (%.6f,%.6f)\n",n,sx,sy,rx,ry); bad++; }
        n++;
    } while(std::next_permutation(p,p+4));
    printf("%d permutations of the SAME quad -> %s  (all give %.4f, %.4f)\n",
           n, bad?"*** MISMATCH ***":"identical result", rx, ry);
    // and the top/bottom-swapped order that drew an X on screen
    aim_pt_t xshape[4]={{147.350f,129.050f},{181.800f,130.800f},
                        {147.600f,66.450f},{180.750f,65.400f}};  // bottom pair first
    float sx,sy; aim_runtime_solve(xshape,240,176,&sx,&sy,0.0f);
    printf("top/bottom-swapped input -> (%.4f, %.4f)  %s\n", sx, sy,
           (fabsf(sx-rx)<1e-6f && fabsf(sy-ry)<1e-6f) ? "OK (same)" : "*** DIFFERS ***");
    return bad?1:0;
}
