#!/usr/bin/env python3
"""The roll term end to end, on IDENTICAL data with the term forced on and forced
off. Asserts it never makes aim worse, that the gate fires only on rolled
stances, and that the fitted coefficient recovers the simulator's physical lever.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import aim_calib as A
import aim_fit

FW, FH = A.FRAME_W, A.FRAME_H
DISTS = (1.2, 1.7, 2.3)
VDIST = 1.45
HAND_ROLL_DEG = 8.0
PULLS = 4
SCREEN_W_M = 0.597
TARGETS = [(x, y) for y in (0.12, 0.5, 0.88) for x in (0.12, 0.5, 0.88)]


def src_for(seed, sigma, bias_seed, lever):
    s = A.CaptureSession(plan=A.make_plan(3, 0))
    src = A.SimSource(s, blob_sigma=sigma, lever_m=lever)
    src.rng = np.random.default_rng(seed)
    b = np.random.default_rng(bias_seed)
    src._stance_bias = b.normal(0, np.deg2rad(src.sb), (12, 2))
    src._dot_bias = b.normal(0, np.deg2rad(src.db), (16, 2))
    return src


def capture(src, dist, tilt, stance, dot, tx, ty):
    qs = [src._quad(tx, ty, dist, np.deg2rad(tilt), stance=stance, dot=dot)
          for _ in range(PULLS)]
    return np.median(np.array(qs), axis=0)


def shots_for(src, rolls=(12, -12)):
    """3 level distance stances plus the rolled pair -- ONE data set, used for
       both arms, so the only difference between them is the model."""
    out = []
    for si, d in enumerate(DISTS):
        for di, (tx, ty) in enumerate(A.DOTS):
            out.append(dict(q=capture(src, d, 0.0, si, di, tx, ty), tx=tx, ty=ty, stance=si))
    for j, r in enumerate(rolls):
        for di, (tx, ty) in enumerate(A.DOTS):
            out.append(dict(q=capture(src, DISTS[0], r, 3+j, di, tx, ty),
                            tx=tx, ty=ty, stance=3+j))
    return out


def fit_forced(shots, roll):
    keep = aim_fit.ROLL_MIN
    aim_fit.ROLL_MIN = keep if roll else 99.0
    try:
        return aim_fit.fit(shots, FW, FH)[0]
    finally:
        aim_fit.ROLL_MIN = keep


def score(c, seed, sigma, bias_seed, lever):
    """A different day: fresh aim bias, fresh noise, a distance never calibrated
       at, and the hand rolling as hands do. Tremor off -- it is unfixable and
       only dilutes the measurement."""
    src = src_for(seed + 777, sigma, bias_seed, lever)
    src.tr = 0.0
    rng = np.random.default_rng(seed + 31337)
    e = []
    for di, (tx, ty) in enumerate(TARGETS):
        for _ in range(8):
            q = src._quad(tx, ty, VDIST, np.deg2rad(rng.uniform(-HAND_ROLL_DEG, HAND_ROLL_DEG)),
                          stance=7, dot=di)
            r = aim_fit.solve(c, q, FW, FH)
            if r:
                e.append(np.hypot((r[0]-tx)*1920.0, (r[1]-ty)*1200.0))
    return float(np.mean(e))


def main():
    fails = []
    SIGMA = 0.6
    print("IDENTICAL data, only the model differs.  blob sigma %.1f, 6 trials.\n" % SIGMA)
    print("%-11s %10s %10s %9s %12s %12s" % (
        "true lever", "roll OFF", "roll ON", "change", "fitted rx", "expected rx"))
    for lever in (0.0, 0.06, 0.12):
        off, on, rxs = [], [], []
        for t in range(6):
            src = src_for(100+t, SIGMA, 500+t, lever)
            shots = shots_for(src)
            c0, c1 = fit_forced(shots, False), fit_forced(shots, True)
            if c0 is None or c1 is None:
                fails.append("lever %.0f cm trial %d: fit refused" % (lever*100, t)); continue
            # the gate must follow the data
            if c0.get('rx') or c0.get('ry'):
                fails.append("lever %.0f cm: roll term fitted with the gate forced off"
                             % (lever*100))
            if not (c1.get('rx') or c1.get('ry')):
                fails.append("lever %.0f cm: rolled stances present but no roll term fitted"
                             % (lever*100))
            off.append(score(c0, 100+t, SIGMA, 900+t, lever))
            on.append(score(c1, 100+t, SIGMA, 900+t, lever))
            rxs.append(c1['rx'])
        if not off:
            continue
        mo, mn = np.mean(off), np.mean(on)
        exp = -lever / SCREEN_W_M
        print("%8.1f cm %10.1f %10.1f %8s%% %12.4f %12.4f" % (
            lever*100, mo, mn, "%+.0f" % (100*(mn-mo)/mo), np.mean(rxs), exp))
        # NOT "must improve" -- it does not reliably. Must not HURT: that catches
        # a sign flip, a blow-up, or a gate letting an unidentifiable term through.
        if mn > mo * 1.10:
            fails.append("lever %.0f cm: the roll term made it %.0f%% WORSE"
                         % (lever*100, 100*(mn-mo)/mo))
        # where the lever is large enough to be recoverable the coefficient must
        # be the physical one -- this is what catches a sign error
        if lever >= 0.06 and abs(np.mean(rxs) - exp) > 0.35 * abs(exp):
            fails.append("lever %.0f cm: coefficient %+.4f does not recover the "
                         "physical %+.4f" % (lever*100, np.mean(rxs), exp))

    # the gate itself, on level-only data
    src = src_for(7, SIGMA, 11, 0.06)
    level = [s for s in shots_for(src) if s['stance'] < 3]
    c = aim_fit.fit(level, FW, FH)[0]
    if c is None:
        fails.append("level-only data did not fit at all")
    elif c.get('rx') or c.get('ry'):
        fails.append("roll term fitted on level-only data -- the gate is not holding")
    else:
        print("\nlevel-only data: roll term correctly gated off")

    print()
    for f in fails:
        print("  [FAIL] %s" % f)
    print("roll end-to-end: %s" % ("ALL PASS" if not fails else "FAILED"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
