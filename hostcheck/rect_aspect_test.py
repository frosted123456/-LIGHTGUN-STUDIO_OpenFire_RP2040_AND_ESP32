#!/usr/bin/env python3
"""Self-check of aim_fit.rect_aspect: it must recover a known aspect exactly over
54 poses, stay unbiased with range under noise (a range-dependent bias would look
like the LED rig drifting), and re-sort shuffled corners.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import aim_fit

F = 184.7
FW, FH = aim_fit.FRAME_W_DEF, aim_fit.FRAME_H_DEF


def synth(W, H, dist, yaw, pitch, roll):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    R = Rz @ Rx @ Ry
    P = np.array([[-W/2, -H/2, 0], [W/2, -H/2, 0], [-W/2, H/2, 0], [W/2, H/2, 0]])
    C = R @ (P - np.array([0, 0, -dist])).T
    return np.stack([FW/2 + F*C[0]/C[2], FH/2 + F*C[1]/C[2]], 1)


fails = []
worst = 0.0
for W, H in ((0.208, 0.402), (0.30, 0.30), (0.40, 0.20)):
    for d in (1.2, 1.8, 2.4):
        for yaw in (-0.25, 0.0, 0.25):
            for pitch in (-0.2, 0.0, 0.2):
                for roll in (0.0, 0.25):
                    a = aim_fit.rect_aspect(synth(W, H, d, yaw, pitch, roll), F)
                    worst = max(worst, abs(a - H/W)/(H/W))
print("exact recovery, 54 poses: worst relative error %.6f%%" % (100*worst))
if worst > 1e-4:
    fails.append("does not recover a known aspect exactly (%.4f%%)" % (100*worst))

rng = np.random.default_rng(11)
print("\n%-9s %10s %12s %10s" % ("distance", "quad span", "median est", "bias"))
for d in (1.1, 1.4, 1.8, 2.3):
    v = [aim_fit.rect_aspect(synth(0.208, 0.402, d, rng.uniform(-0.25, 0.25),
                                   rng.uniform(-0.2, 0.2), rng.uniform(-0.1, 0.1))
                             + rng.normal(0, 0.6, (4, 2)), F) for _ in range(1500)]
    v = np.array(v); v = v[np.isfinite(v)]
    bias = (np.median(v) - 1.9327)/1.9327
    print("%8.1fm %10.1f %12.3f %9s" % (d, aim_fit.quad_span(synth(0.208, 0.402, d, 0, 0, 0)),
                                        np.median(v), "%+.2f%%" % (100*bias)))
    if abs(bias) > 0.01:
        fails.append("biased by %+.1f%% at %.1f m -- would look like rig drift" % (100*bias, d))

# a wrong corner correspondence must not quietly produce something believable
bad = aim_fit.rect_aspect(synth(0.208, 0.402, 1.6, 0.1, 0.1, 0.0)[[0, 1, 3, 2]], F)
print("\nsanity: shuffled corners give %.3f (canon should re-sort them: expect 1.933)" % bad)
if abs(bad - 1.9327) > 0.01:
    fails.append("canon did not re-sort shuffled corners")

print()
for f in fails:
    print("  [FAIL] %s" % f)
print("rect_aspect: %s" % ("ALL PASS" if not fails else "FAILED"))
sys.exit(1 if fails else 0)
