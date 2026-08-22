#!/usr/bin/env python3
"""Fine-tune math: the angular/parallax split across distances, the single-
station SAVE NOW path (saved == the live preview, no double stacking), and the
lead 'worth' readout being in screen units."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import aim_calib as A
import aim_fit

FW, FH = A.FRAME_W, A.FRAME_H
NEAR, FAR = 1.3, 2.3
CHECK = (1.2, 1.7, 2.3, 3.0)


def gun(lever, bore):
    s = A.CaptureSession(plan=A.make_plan(3, 0))
    src = A.SimSource(s, blob_sigma=0.0, tremor_deg=0.0, lever_m=lever, bore=bore)
    src.rng = np.random.default_rng(1)
    src._stance_bias *= 0; src._dot_bias *= 0
    return src


def calibrate(src):
    sh = []
    for si, d in enumerate((1.2, 1.7, 2.3)):
        for di, (tx, ty) in enumerate(A.DOTS):
            sh.append(dict(q=src._quad(tx, ty, d, 0.0, stance=si, dot=di),
                           tx=tx, ty=ty, stance=si))
    return aim_fit.fit(sh, FW, FH)[0]


def err(c, src, d):
    """what the user sees: cursor minus where the iron sights are pointing"""
    q = src._quad(0.5, 0.5, d, 0.0, stance=0, dot=0)
    r = aim_fit.solve(c, q, FW, FH)
    return np.array([r[0] - 0.5, r[1] - 0.5])


def px(e):
    return float(np.hypot(e[0]*1920.0, e[1]*1200.0))


fails = []
truth = gun(0.0, (5.0, -3.0))
c = calibrate(truth)

CASES = [("angular only",        gun(0.00, (11.0, -3.0))),
         ("parallax only",       gun(0.03, (5.0, -3.0))),
         ("both together",       gun(0.03, (11.0, -3.0))),
         ("nothing wrong",       gun(0.00, (5.0, -3.0)))]

print("%-18s %9s %9s %9s %9s   %s" % ("case", *["%.1f m" % d for d in CHECK], "worst after"))
for name, src in CASES:
    before = [px(err(c, src, d)) for d in CHECK]
    nq = src._quad(0.5, 0.5, NEAR, 0.0, stance=0, dot=0)
    fq = src._quad(0.5, 0.5, FAR, 0.0, stance=0, dot=0)
    # what the user would dial in at each distance: the negative of the error
    r = aim_fit.split_offset(c, nq, -err(c, src, NEAR), fq, -err(c, src, FAR), FW, FH)
    if r is None:
        fails.append("%s: the split was refused" % name); continue
    fixed = aim_fit.apply_offset(c, *r)
    after = [px(err(fixed, src, d)) for d in CHECK]
    print("%-18s %9.1f %9.1f %9.1f %9.1f   %9.2f" % (name, *before, max(after)))
    if max(after) > 1.0:
        fails.append("%s: %.1f px left after correction, at a distance it was "
                     "not measured at" % (name, max(after)))
    if name == "nothing wrong" and max(abs(np.array(r))) > 1e-6:
        fails.append("a correctly aimed gun got a non-zero correction")

# the degeneracy: both nudges at the SAME distance cannot separate the two terms
nq = CASES[0][1]._quad(0.5, 0.5, NEAR, 0.0, stance=0, dot=0)
e = -err(c, CASES[0][1], NEAR)
r = aim_fit.split_offset(c, nq, e, nq, e, FW, FH)
print("\nboth measurements at one distance -> %s" % ("refused" if r is None else "ACCEPTED"))
if r is not None:
    fails.append("the split accepted two measurements at the same distance; "
                 "the angular and constant terms are not separable there")


# ---------------------------------------------------------------------------
# The "worth about N screen px" readout. It has to be in SCREEN units: the
# first version differenced camera pixels and divided by the CAMERA frame
# width, which is low by the whole master gain -- a camera pixel is worth about
# 26 screen px here, not 1920/240 = 8.
def _worth_check():
    src = gun(0.0, (5.0, -3.0))
    cc = calibrate(src)
    q0 = src._quad(0.30, 0.5, 1.7, 0.0, stance=0, dot=0)
    q1 = src._quad(0.40, 0.5, 1.7, 0.0, stance=0, dot=0)
    a = aim_fit.solve(cc, q0, FW, FH); b = aim_fit.solve(cc, q1, FW, FH)
    moved = abs(b[0] - a[0])
    if abs(moved - 0.10) > 0.01:
        return ("the solve does not reproduce a known 0.10-screen-width pan "
                "(got %.3f)" % moved)
    v = moved / 0.1                       # 1.0 screen widths per second
    worth = v * (10 / 1000.0) * 1920.0    # 10 ms of lead
    if abs(worth - 19.2) > 2.0:
        return "worth readout is %.1f px where 19.2 is correct" % worth
    return None


_w = _worth_check()
if _w:
    fails.append("lead 'worth' readout: %s" % _w)
else:
    print("lead 'worth' readout: 10 ms at 1.0 screen widths/s = 19 px  OK")


# ---------------------------------------------------------------------------
# Shooting the ring must MEASURE the offset, not just record the pose. The
# first version stored the quad and left the whole correction to the arrow
# buttons, so a user who shot the ring and read the display saw "0, 0 px"
# however far off the cursor actually was.
def _measure_check():
    import aim_finetune as FT
    src = gun(0.03, (11.0, -3.0))            # a gun with a real sight offset
    cc = calibrate(gun(0.0, (5.0, -3.0)))    # calibrated on the ideal one
    t = FT.Tuner(cc)

    # aim the IRON SIGHTS at the ring: the sim's target IS where the user points
    q = src._quad(FT.TARGET[0], FT.TARGET[1], 1.4, 0.0, stance=0, dot=0)
    t.note_quad(q)
    if not t.measured[0]:
        return "a clean shot on the ring was not accepted: %s" % t.msg
    got = np.hypot(t.off[0][0]*1920.0, t.off[0][1]*1200.0)
    if got < 20.0:
        return ("shooting the ring measured only %.1f px on a gun that is "
                "visibly off -- this is the bug that shipped" % got)

    # the residual is additive, so a second shot converges rather than doubling
    before = t.off[0].copy()
    t.note_quad(q)
    second = np.hypot((t.off[0][0]-before[0])*1920.0, (t.off[0][1]-before[1])*1200.0)
    if second > 2.0:
        return ("a second shot added another %.1f px -- the residual is not "
                "being measured against the correction already applied" % second)

    # and a gun that is already right must measure ~nothing
    t2 = FT.Tuner(cc)
    t2.note_quad(gun(0.0, (5.0, -3.0))._quad(FT.TARGET[0], FT.TARGET[1], 1.4, 0.0,
                                             stance=0, dot=0))
    if np.hypot(t2.off[0][0]*1920.0, t2.off[0][1]*1200.0) > 2.0:
        return "a correctly aimed gun measured a non-zero offset"

    # a shot nowhere near the ring is refused rather than believed
    t3 = FT.Tuner(cc)
    t3.note_quad(src._quad(0.05, 0.95, 1.4, 0.0, stance=0, dot=0))
    if t3.measured[0]:
        return "a shot a whole screen away from the ring was accepted"
    print("shooting the ring measures %.0f px, converges on the second shot" % got)
    return None


_m = _measure_check()
if _m:
    fails.append("ring measurement: %s" % _m)

print()
for f in fails:
    print("  [FAIL] %s" % f)
# ---------------------------------------------------------------------------
# SAVE NOW: the single-station direct save. What is saved must be EXACTLY the
# preview on screen, it must not require a ring shot, and shooting the ring
# after manual nudges must converge instead of stacking.
# ---------------------------------------------------------------------------
import aim_finetune

t = aim_finetune.Tuner(dict(c))
if t.solve_direct() is not None:
    fails.append("empty direct save was not refused")
else:
    print("\nempty direct save -> refused (nothing to keep yet)")
for _ in range(3):
    t.nudge(+1, 0)
t.nudge(0, +2)
out = t.solve_direct()
if out is None:
    fails.append("manual-only save refused")
else:
    pv = t.preview()
    if any(abs(out[k] - pv[k]) > 1e-12 for k in ("cx", "cy", "w", "h", "bx", "by")):
        fails.append("direct save differs from the live preview")
    if abs(out["w"] - c["w"]) > 1e-12 or abs(out["bx"] - c["bx"]) > 1e-12:
        fails.append("direct save disturbed the fitted geometry")
    print("manual-only save -> the preview itself; bore and rectangle untouched")

# no double stacking: a deliberate wrong nudge, then ring shots on a known-good
# aim. The first shot must cancel the nudge; the second must change ~nothing.
src0 = CASES[3][1]                       # 'nothing wrong'
t2 = aim_finetune.Tuner(dict(c))
q = src0._quad(aim_finetune.TARGET[0], aim_finetune.TARGET[1], NEAR, 0.0, stance=0, dot=0)
t2.nudge(+5, 0)
t2.note_quad(q)
a = t2.off[0].copy()
t2.note_quad(q)
b = t2.off[0].copy()
move2 = float(np.hypot((b - a)[0] * 1920.0, (b - a)[1] * 1200.0))
resid = aim_fit.solve(t2.preview(), q, FW, FH)
left = float(np.hypot((resid[0] - aim_finetune.TARGET[0]) * 1920.0,
                      (resid[1] - aim_finetune.TARGET[1]) * 1200.0))
print("wrong nudge + ring shot -> %.2f px from the ring; second shot moved %.2f px"
      % (left, move2))
if left > 1.0:
    fails.append("ring shot after a manual nudge did not cancel it (%.2f px left)" % left)
if move2 > 0.5:
    fails.append("repeat ring shot double-stacked (%.2f px)" % move2)

print("finetune split: %s" % ("ALL PASS" if not fails else "FAILED"))
if fails:
    for f in fails: print("  -", f)
sys.exit(1 if fails else 0)

