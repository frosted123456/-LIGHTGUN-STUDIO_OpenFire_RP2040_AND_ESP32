#!/usr/bin/env python3
"""The frame-edge guard, checked against two real bench sessions: it must reject
much of the too-close session (data/real_tooclose.csv) and almost none of the one
that calibrated fine (data/real_good.csv). Both directions matter.
"""
import sys, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import aim_calib as A

def load(name):
    return np.loadtxt(os.path.join(HERE, "data", name), delimiter=",", skiprows=1).reshape(-1, 4, 2)


def shot_margins(name):
    """what the guard actually judges: the margin of a 4-pull median quad"""
    q = load(name)
    out = []
    for i in range(0, len(q) - 4, 4):
        out.append(A.CaptureSession.edge_margin(np.median(q[i:i+4], axis=0)))
    return np.array(out)

fails = []
print("%-22s %8s %10s %10s" % ("session", "frames", "rejected", "median margin"))
res = {}
for name, tag in (("real_good.csv", "good"), ("real_tooclose.csv", "too close")):
    q = load(name)
    m = np.array([A.CaptureSession.edge_margin(f) for f in q])
    rej = float(np.mean(m < A.EDGE_MARGIN))
    res[tag] = rej
    print("%-22s %8d %9.1f%% %10.1f px" % (tag, len(q), 100*rej, np.median(m)))

# the guard judges median quads, so check it on those too
mg = shot_margins("real_good.csv"); mb = shot_margins("real_tooclose.csv")
print("\nmedian-quad margin   good: min %.1f p25 %.1f   too close: min %.1f p25 %.1f"
      % (mg.min(), np.percentile(mg, 25), mb.min(), np.percentile(mb, 25)))
print("rejected at EDGE_MARGIN=%.0f   good %.1f%%   too close %.1f%%"
      % (A.EDGE_MARGIN, 100*np.mean(mg < A.EDGE_MARGIN), 100*np.mean(mb < A.EDGE_MARGIN)))
if np.mean(mg < A.EDGE_MARGIN) > 0.05:
    fails.append("the margin rejects %.0f%% of a session that calibrated fine"
                 % (100*np.mean(mg < A.EDGE_MARGIN)))
if np.mean(mb < A.EDGE_MARGIN) < 0.10:
    fails.append("the margin only rejects %.0f%% of the too-close session"
                 % (100*np.mean(mb < A.EDGE_MARGIN)))

if res["good"] > 0.15:
    fails.append("the guard rejects %.0f%% of a session that calibrated fine "
                 "-- too aggressive" % (100*res["good"]))
if res["too close"] < 0.15:
    fails.append("the guard only rejects %.0f%% of the session that produced the "
                 "worst calibration of the project -- too permissive"
                 % (100*res["too close"]))

# and the margin itself must be computed the obvious way
probe = np.array([[10.0, 10.0], [230.0, 10.0], [10.0, 166.0], [230.0, 166.0]])
if abs(A.CaptureSession.edge_margin(probe) - 10.0) > 1e-6:
    fails.append("edge_margin is not the distance to the nearest frame edge")

print()
for f in fails:
    print("  [FAIL] %s" % f)
print("edge guard: %s" % ("ALL PASS" if not fails else "FAILED"))
sys.exit(1 if fails else 0)
