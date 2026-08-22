#!/usr/bin/env python3
"""The window->screen conversion must recover the TRUE LED rectangle even when the
GUI window covers only part of the screen; otherwise the fit comes out with a
silently wrong aspect."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import aim_calib as A, aim_fit, numpy as np
TRUE_W, TRUE_H = 0.208/0.592, 0.402/0.333     # rig / a real 16:9 screen
for frac_w, label in [(1.00,"window covers the screen"),
                      (0.845,"window is 84.5% of screen width (session 2)"),
                      (0.70,"window is 70% of screen width")]:
    sess=A.CaptureSession(stances=2)
    SW,SH,WW,WH,WX,WY = 1920,1080,int(1920*frac_w),1080,0,0
    sess.to_screen = lambda fx,fy: ((WX+fx*WW)/float(SW),(WY+fy*WH)/float(SH))
    src=A.SimSource(sess, blob_sigma=0.2, screen_w=0.592)
    # the simulated user aims at where the reticle PHYSICALLY is on the screen
    orig=src._quad
    src._quad=lambda tx,ty,d: orig(*sess.to_screen(tx,ty), d)
    now=0.0; guard=0
    while sess.state!=sess.S_DONE and guard<60000:
        guard+=1
        d=src.dists[min(sess.stance,len(src.dists)-1)]
        sess.feed(src._quad(*sess.target(), d), now); now+=1/60.0
    c,why=aim_fit.fit(sess.shots,240.,176.)
    if not c: print("  %-44s REFUSED (%s)"%(label,why)); continue
    print("  %-44s w %.4f (true %.4f, err %+.1f%%)  aspect %.3f"%(
        label,c['w'],TRUE_W,100*(c['w']/TRUE_W-1),(0.208/0.402)*c['h']/c['w']))
    if abs(c['w']/TRUE_W-1) > 0.03:
        print("  *** FAILED: rectangle wrong by more than 3%"); sys.exit(1)
print("window/screen geometry: OK")
