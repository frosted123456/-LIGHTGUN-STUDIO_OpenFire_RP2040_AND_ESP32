#!/usr/bin/env python3
"""Lens parameters for the OV2640 overlay's undistortion (LIGHTGUN_LENS_*).

  model 1 (poly):    r_d = r_u * (1 + k1*r_u^2 + k2*r_u^4)   on r/FPX
  model 2 (fisheye): r_px = FEQ * theta, mapped to a pinhole of focal FPX

Three ways to get them:
  spec      from the lens datasheet FOV alone
  fit       from a logged sweep (rotational self-calibration)
  selftest  synthetic sweeps with known parameters, recovery checked

The fit needs ROTATION-ONLY views: stand in ONE spot ~2 m from the rig and
slowly pan/tilt/roll for ~20 s, pushing the LEDs out to the image edges (four
points can never over-determine a homography, so free motion reveals nothing;
rotation-only frames over-determine the lens massively). Capture raw:
res=0 lens=0 dash=2 dashhz=0. Studio's Lens step runs this whole protocol for
you; this CLI exists for logs and for the selftest."""
import argparse, math, re, sys
import numpy as np

IMG_W, IMG_H = 240.0, 176.0
CX, CY = IMG_W / 2.0, IMG_H / 2.0

def fpx_for(fov_deg):
    """Output pinhole focal for a given lens FOV. Maps at most 120 deg of
    field into the frame: beyond that the mapping is pointless for a lightgun
    (the LEDs live in the central region) and at 180 deg it is impossible
    outright -- tan(90deg) is infinite, so the unclamped formula returned a
    ZERO focal, which the firmware rejected, leaving a stale fpx behind."""
    eff = min(float(fov_deg), 120.0)
    return (IMG_W / 2.0) / math.tan(math.radians(eff) / 2.0)

# --------------------------------------------------------------------------
# parsing + correspondence
# --------------------------------------------------------------------------
QLINE = re.compile(r"^Q,(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),"
                   r"(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")

def parse_log(path):
    """Q-lines with 4 valid points -> (N,4,2) px array."""
    frames = []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            m = QLINE.match(line)
            if not m:
                continue
            vals = [int(g) for g in m.groups()]
            if vals[1] != 4:
                continue
            pts = np.array(vals[2:], float).reshape(4, 2) / 10.0
            if (pts < 0).any():
                continue
            frames.append(pts)
    return np.array(frames)

def order_and_track(frames):
    """Angular order within each frame, cyclic shift aligned to the previous
    frame; returns the LONGEST continuous segment (global correspondence must
    not be broken by a dropout, so we simply keep the best run)."""
    if len(frames) == 0:
        return frames
    ordered, segs, cur = [], [], []
    prev = None
    for pts in frames:
        c = pts.mean(axis=0)
        ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
        p = pts[np.argsort(ang)]
        if prev is not None:
            shifts = [np.roll(p, s, axis=0) for s in range(4)]
            costs = [np.linalg.norm(sh - prev, axis=1).sum() for sh in shifts]
            best = int(np.argmin(costs))
            if costs[best] > 4 * 40.0:            # continuity broken
                segs.append(cur); cur = []
                prev = None
            else:
                p = shifts[best]
        cur.append(p)
        prev = p
    segs.append(cur)
    seg = np.array(max(segs, key=len))
    # Temporal smoothing (centred boxcar-5 per corner track). The bundle
    # estimator carries a small systematic bias that grows with noise VARIANCE
    # (selftest: sigma=0.3px biased FEQ by +3.8% across every seed, noiseless
    # recovery was exact to 0.01%) -- more frames do not help a bias, less
    # noise does. The protocol sweeps slowly at 135 fps, so 5 frames span
    # ~37 ms of near-linear motion and the geometric cost is negligible.
    if len(seg) >= 5:
        k = np.ones(5) / 5.0
        sm = seg.astype(float).copy()
        for c in range(4):
            for ax in range(2):
                sm[2:-2, c, ax] = np.convolve(seg[:, c, ax], k, mode="valid")
        seg = sm[2:-2]
    return seg

# --------------------------------------------------------------------------
# lens models: distorted px -> unit rays (this is what the fit compares)
# --------------------------------------------------------------------------
def rays_fisheye(pts, feq):
    d = pts - (CX, CY)
    r = np.linalg.norm(d, axis=-1)
    th = np.clip(r / feq, 0, 1.45)
    s = np.where(r > 1e-9, np.sin(th) / np.maximum(r, 1e-9), 1.0 / feq)
    out = np.empty(pts.shape[:-1] + (3,))
    out[..., 0] = d[..., 0] * s
    out[..., 1] = d[..., 1] * s
    out[..., 2] = np.cos(th)
    return out

def rays_poly(pts, k1, k2, fpx):
    """Mirror of the firmware: r_dn = r/FPX, 3 Newton steps invert
    r_u(1+k1 r_u^2+k2 r_u^4) = r_dn, ray = (dir * r_u, 1) normalized."""
    d = pts - (CX, CY)
    r = np.linalg.norm(d, axis=-1)
    rdn = r / fpx
    ru = rdn.copy()
    for _ in range(3):
        r2 = ru * ru
        f = ru * (1 + k1 * r2 + k2 * r2 * r2) - rdn
        df = 1 + 3 * k1 * r2 + 5 * k2 * r2 * r2
        df = np.where(np.abs(df) < 1e-6, 1e-6, df)
        ru = ru - f / df
    s = np.where(r > 1e-9, ru / np.maximum(r, 1e-9), 1.0 / fpx)
    out = np.empty(pts.shape[:-1] + (3,))
    out[..., 0] = d[..., 0] * s
    out[..., 1] = d[..., 1] * s
    out[..., 2] = 1.0
    return out / np.linalg.norm(out, axis=-1, keepdims=True)

# --------------------------------------------------------------------------
# rotation-only bundle: rays_f[i] ~ R_f · d_i
# --------------------------------------------------------------------------
def kabsch(A, B):
    """R minimizing ||R A - B|| (columns are vectors)."""
    H = B @ A.T
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt

def bundle_rms(rays, iters=25):
    """Alternating: d_i (shared corner directions) vs per-frame rotations.
    Returns RMS angular residual in radians."""
    N = len(rays)
    d = rays[0].T.copy()                        # 3x4, gauge: R_0 = I
    for _ in range(iters):
        Rs = np.empty((N, 3, 3))
        for f in range(N):
            Rs[f] = kabsch(d, rays[f].T)
        acc = np.zeros((3, 4))
        for f in range(N):
            acc += Rs[f].T @ rays[f].T
        d = acc / np.linalg.norm(acc, axis=0, keepdims=True)
    res = []
    for f in range(N):
        pred = (Rs[f] @ d).T
        dot = np.clip((pred * rays[f]).sum(axis=1), -1, 1)
        res.append(np.arccos(dot))
    return float(np.sqrt(np.mean(np.square(res))))

def golden(fun, lo, hi, tol, trace=None):
    g = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    c, dv = b - g * (b - a), a + g * (b - a)
    fc, fd = fun(c), fun(dv)
    while abs(b - a) > tol:
        if fc < fd:
            b, dv, fd = dv, c, fc
            c = b - g * (b - a); fc = fun(c)
        else:
            a, c, fc = c, dv, fd
            dv = a + g * (b - a); fd = fun(dv)
        if trace is not None:
            trace.append((a + b) / 2)
    return (a + b) / 2

def subsample(frames, cap=300):
    if len(frames) <= cap:
        return frames
    idx = np.linspace(0, len(frames) - 1, cap).astype(int)
    return frames[idx]

def coverage(frames):
    """max radial excursion of any point, as a fraction of the frame's
    limiting half-extent — how far the sweep actually pushed the LEDs."""
    r = np.linalg.norm(frames - (CX, CY), axis=-1)
    return float(r.max() / min(CX, CY))

def scan_then_refine(fun, lo, hi, n, tol, log=False):
    """Grid-scan the WHOLE bracket, then golden-refine around an interior
    minimum. Returns (x, fx, ident) where ident is the identifiability: how
    much worse the best bracket EDGE is than the minimum. A minimum ON the
    edge, or edges barely above the minimum, means the data cannot pin the
    parameter -- the first selftest draft showed a 'contrast around the
    optimum' check happily blessing a fit pegged at the bracket edge with 75%
    error, because a garbage optimum still has a sloped neighbourhood. The
    global scan is the honest test."""
    xs = np.geomspace(lo, hi, n) if log else np.linspace(lo, hi, n)
    vals = [fun(x) for x in xs]
    i = int(np.argmin(vals))
    if i == 0 or i == len(xs) - 1:
        return float(xs[i]), float(vals[i]), 0.0          # pegged: refuse
    ident = (min(vals[0], vals[-1]) - vals[i]) / max(vals[i], 1e-12)
    x = golden(fun, xs[i - 1], xs[i + 1], tol)
    return float(x), float(fun(x)), float(ident)

def fit_fisheye(frames, feq0=None):
    frames = subsample(frames)
    # Cost in PIXELS (angular rms x feq): the raw angular cost is not
    # scale-invariant -- the same pixel error subtends a smaller angle at a
    # larger feq, so an unnormalised cost drifts to the bracket's top edge.
    fun = lambda feq: bundle_rms(rays_fisheye(frames, feq)) * feq
    # Data-bounded bracket, independent of the typed lens label. Lower bound:
    # no observed point may reach the 1.45 rad clip -- below that, clipped
    # points collapse onto a cone the bundle fits spuriously well, creating a
    # fake minimum at the bracket edge.
    rmax = float(np.linalg.norm(frames - (CX, CY), axis=-1).max())
    lo = max(20.0, rmax / 1.40)
    hi = max(600.0, lo * 4.0)
    return scan_then_refine(fun, lo, hi, 17, 0.05, log=True)

def fit_poly(frames, fpx, fit_k2=False):
    frames = subsample(frames)
    fun1 = lambda k1: bundle_rms(rays_poly(frames, k1, 0.0, fpx))
    k1, r, ident = scan_then_refine(fun1, -0.8, 0.4, 13, 5e-4)
    if not fit_k2 or ident <= 0.0:
        return k1, 0.0, r, ident
    fun2 = lambda k2: bundle_rms(rays_poly(frames, k1, k2, fpx))
    k2, _, _ = scan_then_refine(fun2, -0.3, 0.3, 7, 1e-3)
    fun = lambda k1v: bundle_rms(rays_poly(frames, k1v, k2, fpx))
    k1 = golden(fun, k1 - 0.2, k1 + 0.2, 5e-4)
    return k1, k2, fun(k1), ident

# --------------------------------------------------------------------------
# output
# --------------------------------------------------------------------------
def emit(model, fpx, feq=None, k1=None, k2=None, rms_px=None):
    print()
    if rms_px is not None:
        print(f"fit residual: {rms_px:.2f} px RMS "
              f"({'good' if rms_px < 1.0 else 'check the sweep' if rms_px < 3.0 else 'BAD — redo the sweep (did you stand still? res=0 lens=0?)'})")
    print("\nSHIP build (platformio.ini build_flags):")
    if model == "fisheye":
        print(f"  -D LIGHTGUN_LENS_MODEL=2")
        print(f"  -D LIGHTGUN_LENS_FEQ={feq:.1f}f")
        print(f"  -D LIGHTGUN_LENS_FPX={fpx:.1f}f")
        print("\nDIAG tune line (UART0):")
        print(f"  lens=2 lfeq={round(feq*10)} lfpx={round(fpx*10)}")
    else:
        print(f"  -D LIGHTGUN_LENS_MODEL=1")
        print(f"  -D LIGHTGUN_LENS_K1={k1:.4f}f")
        print(f"  -D LIGHTGUN_LENS_K2={k2:.4f}f")
        print(f"  -D LIGHTGUN_LENS_FPX={fpx:.1f}f")
        print("\nDIAG tune line (UART0):")
        print(f"  lens=1 lk1u={round(k1*1e6)} lk2u={round(k2*1e6)} lfpx={round(fpx*10)}")
    print("\nThen verify: lens on, res=2, aim at each corner of the screen —")
    print("OpenFIRE's cursor should now land flat in the corners, and the")
    print("resolver RES/s line should show envrej staying ~0.")

def cmd_spec(a):
    fpx = a.fpx if a.fpx is not None else fpx_for(a.fov)
    print(f"output pinhole FPX = {fpx:.1f} px "
          f"({'user override' if a.fpx is not None else 'full field -> 240px frame, no publish clamping'})")
    if a.model == "fisheye":
        feq = (IMG_W / 2) / math.radians(a.fov) * 2  # (W/2) / (fov/2 rad)
        print(f"equidistant fisheye, {a.fov:.0f}deg full FOV -> FEQ = {feq:.1f} px/rad")
        emit("fisheye", fpx, feq=feq)
    else:
        k1 = a.k1 if a.k1 is not None else 0.0
        print(f"pinhole {a.fov:.0f}deg "
              f"(spec mode cannot derive k1; pass --k1 or use fit)")
        emit("poly", fpx, k1=k1, k2=0.0)

def cmd_fit(a):
    if a.fpx is None and a.fov is None:
        sys.exit("pass --fov <lens full FOV in deg> (or an explicit --fpx): the "
                 "output focal must match the lens or the publish clamp bites")
    a.fpx = a.fpx if a.fpx is not None else fpx_for(a.fov)
    print(f"output pinhole FPX = {a.fpx:.1f} px")
    raw = parse_log(a.log)
    if len(raw) < 30:
        sys.exit(f"only {len(raw)} usable Q-lines in {a.log} — need a real sweep "
                 f"(res=0 lens=0 dash=2 dashhz=0, all four LEDs in frame)")
    frames = order_and_track(raw)
    print(f"{len(raw)} 4-point frames, longest continuous segment {len(frames)}")
    if len(frames) < 30:
        sys.exit("continuity too broken to track corners — sweep more slowly")
    cov = coverage(frames)
    print(f"radial coverage: {cov*100:.0f}% of the frame half-extent"
          + ("  (low — expect the identifiability gate to complain)" if cov < 0.55 else ""))
    REFUSE = ("the data cannot pin the parameter (no interior minimum with\n"
              "real contrast) — a fit reported anyway would just be noise.\n"
              "Redo the sweep pushing the LEDs out to the image edges.")
    if a.model == "fisheye":
        feq0 = a.feq0 or (IMG_W / 2) / math.radians(a.fov or 160) * 2
        feq, rms, ident = fit_fisheye(frames, feq0)
        print(f"identifiability (bracket-edge contrast): {ident*100:.0f}%")
        if ident < 0.30:
            sys.exit(REFUSE)
        emit("fisheye", a.fpx, feq=feq, rms_px=rms)
    else:
        k1, k2, rms, ident = fit_poly(frames, a.fpx, fit_k2=a.k2)
        print(f"identifiability (bracket-edge contrast): {ident*100:.0f}%")
        if ident < 0.30:
            sys.exit(REFUSE)
        emit("poly", a.fpx, k1=k1, k2=k2, rms_px=rms * a.fpx)

# --------------------------------------------------------------------------
# programmatic entry point (gun_studio's Lens step drives this directly)
# --------------------------------------------------------------------------
def fit_from_frames(raw, fov):
    """raw: (N,4,2) px frames captured with res=0 lens=0. Returns a dict:
       {ok, why, model, k1, k2, fpx, feq, rms_px, ident, coverage} -- the same
       gates cmd_fit applies, so the GUI cannot bless a sweep the CLI would
       refuse. Both models are fitted and the lower-residual one wins."""
    out = dict(ok=False, why="", model=None, k1=0.0, k2=0.0,
               fpx=fpx_for(fov), feq=0.0, rms_px=0.0, ident=0.0, coverage=0.0)
    raw = np.asarray(raw, float)
    if len(raw) < 30:
        out["why"] = "only %d usable 4-LED frames -- sweep longer" % len(raw)
        return out
    frames = order_and_track(raw)
    if len(frames) < 30:
        out["why"] = "corner tracking kept breaking -- sweep more slowly"
        return out
    cov = coverage(frames)
    out["coverage"] = cov
    if cov < 0.55:
        out["why"] = ("sweep only reached %.0f%% of the frame -- push the LEDs "
                      "out toward the image edges and corners" % (cov * 100))
        return out
    # A tiny LED quad carries no usable distortion signal at any noise level,
    # so refuse it before fitting anything (a lucky ident on 6 px quads would
    # otherwise bless a garbage fit).
    spans = np.linalg.norm(frames.max(axis=1) - frames.min(axis=1), axis=1)
    span = float(np.median(spans))
    if span < 25.0:
        out["why"] = ("the LED quad is only %.0f px across -- too small "
                      "to measure the lens; stand closer (~0.5 m) and "
                      "re-sweep" % span)
        return out
    fpx = out["fpx"]
    feq, rms_f, id_f = fit_fisheye(frames)
    k1, k2, rms_p, id_p = fit_poly(frames, fpx)
    fisheye_ok = id_f >= 0.30
    poly_ok = id_p >= 0.30
    if not fisheye_ok and not poly_ok:
        # Third verdict: no measurable distortion means pinhole, not failure.
        # Distortion displacement grows with the cube of the field angle, so a
        # mild wide converter on a narrow-FOV sensor is sub-noise -- the right
        # correction is none. Relative test: pinhole wins when neither model
        # beats it meaningfully, with an absolute cap against garbage sweeps.
        rms_pin = bundle_rms(rays_poly(frames, 0.0, 0.0, fpx)) * fpx
        rms_best = min(rms_f, rms_p * fpx)
        if rms_pin <= max(1.0, 1.15 * rms_best) and rms_pin <= 3.0:
            out.update(ok=True, model="none", rms_px=rms_pin, ident=0.0)
            return out
        out["why"] = ("the sweep is inconsistent with every model, pinhole "
                      "included (%.2f px rms) -- LEDs unstable at the frame "
                      "edges, or the lens is not radially symmetric" % rms_pin)
        return out
    # px residuals on a common footing
    cand = []
    if fisheye_ok: cand.append(("fisheye", rms_f, id_f))
    if poly_ok:    cand.append(("poly", rms_p * fpx, id_p))
    model, rms_px, ident = min(cand, key=lambda c: c[1])
    out.update(ok=True, model=model, rms_px=rms_px, ident=ident)
    if model == "fisheye":
        out["feq"] = feq
        out["fpx"] = feq          # centre-matched output pinhole
    else:
        out["k1"], out["k2"] = k1, k2
    return out


def tune_line(r):
    """the ~cam= payload for a fit_from_frames result (no leading ~cam=)."""
    if r["model"] == "none":
        return "lens:0"
    assert r["fpx"] > 1.0, "refusing to emit a degenerate output focal"
    if r["model"] == "fisheye":
        return "lens:2,lfeq:%d,lfpx:%d" % (round(r["feq"] * 10), round(r["fpx"] * 10))
    return "lens:1,lk1u:%d,lk2u:%d,lfpx:%d" % (
        round(r["k1"] * 1e6), round(r["k2"] * 1e6), round(r["fpx"] * 10))


def spec_fisheye(fov):
    """Precalc from the datasheet FOV alone (equidistant assumption).
    fpx = feq: the correction is identity at the image centre, so gates and
    gain match the raw lens and only the edges get straightened."""
    feq = (IMG_W / 2) / math.radians(fov) * 2
    return dict(model="fisheye", fpx=feq, feq=feq, k1=0.0, k2=0.0)


# --------------------------------------------------------------------------
# selftest: synthesize rotation-only sweeps, recover known parameters
# --------------------------------------------------------------------------
def _rotm(pan, tilt, roll):
    cp, sp = math.cos(pan), math.sin(pan)
    ct, st = math.cos(tilt), math.sin(tilt)
    cr, sr = math.cos(roll), math.sin(roll)
    Rp = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rt = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
    Rr = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return Rr @ Rt @ Rp

def _synth(model, n=400, noise=0.3, seed=7, feq=85.9, k1=-0.30, fpx=84.0,
           amp_override=None):
    rng = np.random.default_rng(seed)
    hw, hh = 0.40, 0.225
    corners = np.array([[-hw, -hh, 2.0], [hw, -hh, 2.0],
                        [hw, hh, 2.0], [-hw, hh, 2.0]])
    frames = []
    # amplitudes matter: the sweep must push the LEDs toward the image edges
    # or the lens is unobservable (this is exactly what the coverage/flatness
    # gates in cmd_fit exist to catch — the first selftest draft swept only
    # the central 45% and the fisheye objective was flat to within noise).
    amp = amp_override if amp_override is not None else (1.0 if model == "fisheye" else 0.62)
    for t in np.linspace(0, 1, n):
        pan = 0.85 * amp * math.sin(2 * math.pi * 2.0 * t)
        tilt = 0.48 * amp * math.sin(2 * math.pi * 3.1 * t + 1.0)
        roll = 0.50 * math.sin(2 * math.pi * 1.3 * t + 2.0)
        R = _rotm(pan, tilt, roll)
        cam = (R @ corners.T).T
        pts = np.empty((4, 2))
        for i, v in enumerate(cam):
            if v[2] < 0.1:
                break
            if model == "fisheye":
                rn = math.hypot(v[0], v[1]) / v[2]
                th = math.atan(rn)
                rpx = feq * th
                s = rpx / max(rn, 1e-12) / v[2]
            else:
                rn2 = (v[0] ** 2 + v[1] ** 2) / v[2] ** 2
                s = fpx * (1 + k1 * rn2) / v[2]
            pts[i] = (CX + v[0] * s, CY + v[1] * s)
        else:
            if (pts[:, 0] > 1).all() and (pts[:, 0] < IMG_W - 1).all() \
               and (pts[:, 1] > 1).all() and (pts[:, 1] < IMG_H - 1).all():
                frames.append(pts + rng.normal(0, noise, (4, 2)))
    return np.array(frames)

def cmd_selftest(_a):
    ok = True
    fr = order_and_track(_synth("fisheye", feq=85.9))
    feq, rms, ident = fit_fisheye(fr, 75.0)
    err = abs(feq - 85.9) / 85.9
    print(f"fisheye: {len(fr)} frames, coverage {coverage(fr)*100:.0f}%, fitted "
          f"FEQ {feq:.2f} (true 85.90, err {err*100:.1f}%), residual "
          f"{rms:.2f} px, ident {ident*100:.0f}%")
    ok &= err < 0.02 and ident > 0.30
    fr = order_and_track(_synth("poly", k1=-0.30, fpx=84.0))
    k1, k2, rms, ident = fit_poly(fr, 84.0)
    print(f"poly:    {len(fr)} frames, coverage {coverage(fr)*100:.0f}%, fitted "
          f"k1 {k1:.3f} (true -0.300), residual {rms*84.0:.2f} px, "
          f"ident {ident*100:.0f}%")
    ok &= abs(k1 - (-0.30)) < 0.03 and ident > 0.30
    # negative control: the WRONG model must fit measurably worse. A poly can
    # approximate an equidistant map surprisingly well over this field, so the
    # honest discriminator is the residual RATIO, not an absolute px number.
    fr = order_and_track(_synth("fisheye", feq=85.9))
    _, rms_right, _ = fit_fisheye(fr, 75.0)          # already px
    _, _, rms_wrong, _ = fit_poly(fr, 84.0)
    ratio = (rms_wrong * 84.0) / max(rms_right, 1e-12)   # poly angular -> px
    print(f"control: wrong-model residual ratio {ratio:.1f}x (need > 1.8x)")
    ok &= ratio > 1.8
    # indeterminacy control: a CENTRAL-ONLY sweep must be REFUSED (this is the
    # exact failure the first draft's local-contrast gate waved through: fit
    # pegged at the bracket edge, 75% wrong, sloped neighbourhood)
    fr_c = order_and_track(_synth("fisheye", feq=85.9, amp_override=0.25))
    _, _, id_c = fit_fisheye(fr_c, 75.0)
    print(f"control: central-only sweep (coverage {coverage(fr_c)*100:.0f}%) "
          f"ident {id_c*100:.0f}% (must be < 30% -> correctly refused)")
    ok &= id_c < 0.30
    # the programmatic path the Lens step uses must apply the same gates
    fr_raw = _synth("fisheye", feq=85.9)
    r = fit_from_frames(fr_raw, 160)
    print(f"frames-api: model={r['model']} feq={r['feq']:.1f} fpx={r['fpx']:.1f} "
          f"rms={r['rms_px']:.2f}px ok={r['ok']}")
    ok &= r["ok"] and r["model"] == "fisheye" and abs(r["feq"] - 85.9) / 85.9 < 0.02
    ok &= r["fpx"] > 1.0
    # a wildly wrong typed FOV must not change the fit (absolute bracket)
    r2 = fit_from_frames(fr_raw, 60)
    print(f"frames-api: same sweep, FOV typed as 60 -> feq={r2['feq']:.1f} "
          f"(must still recover)")
    ok &= r2["ok"] and abs(r2["feq"] - 85.9) / 85.9 < 0.02
    # a MILD lens (distortion below the noise floor) must come back as a
    # pinhole verdict, not a refusal -- the 0.67x converter case
    r3 = fit_from_frames(_synth("poly", n=1500, k1=-0.008, fpx=405.1,
                                noise=2.0, amp_override=0.15), 33)
    print(f"frames-api: mild 0.67x-like lens -> model={r3['model']} "
          f"rms={r3['rms_px']:.2f}px ok={r3['ok']} (want none/ok)")
    ok &= r3["ok"] and r3["model"] == "none"
    ok &= tune_line(dict(model="none", fpx=0.0)) == "lens:0"
    # a sweep with tiny quads (LEDs too far) must be refused with the
    # stand-closer hint, never fitted -- the 180-deg-at-2m case
    _rng = np.random.default_rng(3)
    _raw = _synth("fisheye", feq=85.9)
    _cent = _raw.mean(axis=1, keepdims=True)
    r4 = fit_from_frames(_cent + (_raw - _cent) * 0.12
                         + _rng.normal(0, 0.5, _raw.shape), 160)
    print(f"frames-api: tiny-quad sweep refused ({not r4['ok']}): {r4['why'][:55]}")
    ok &= (not r4["ok"]) and "stand closer" in r4["why"]
    # the 180-deg degenerate: preset must never emit fpx=0
    sp = spec_fisheye(180)
    print(f"spec 180deg: feq={sp['feq']:.1f} fpx={sp['fpx']:.1f} (fpx=0 was the bug)")
    ok &= sp["fpx"] > 1.0 and abs(sp["fpx"] - sp["feq"]) < 1e-6
    r = fit_from_frames(_synth("fisheye", feq=85.9, amp_override=0.25), 160)
    print(f"frames-api: central-only sweep refused ({not r['ok']}): {r['why'][:60]}")
    ok &= not r["ok"]
    r = fit_from_frames(fr_raw[:10], 160)
    ok &= not r["ok"]
    print("SELFTEST", "PASSED" if ok else "FAILED")
    sys.exit(0 if ok else 1)

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 epilog="Full protocol in the module docstring.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("spec", help="parameters from the lens datasheet FOV")
    s.add_argument("--model", choices=["fisheye", "poly"], required=True)
    s.add_argument("--fov", type=float, required=True, help="full FOV, degrees")
    s.add_argument("--k1", type=float, default=None)
    s.add_argument("--fpx", type=float, default=None,
                   help="output pinhole focal, px (default: full field -> frame)")
    s.set_defaults(fn=cmd_spec)
    f = sub.add_parser("fit", help="fit from a dashboard log sweep")
    f.add_argument("log")
    f.add_argument("--model", choices=["fisheye", "poly"], required=True)
    f.add_argument("--fov", type=float, default=None,
                   help="datasheet full FOV, deg (sets FPX + FEQ seed)")
    f.add_argument("--feq0", type=float, default=None, help="FEQ search seed")
    f.add_argument("--fpx", type=float, default=None,
                   help="output pinhole focal, px (default: from --fov)")
    f.add_argument("--k2", action="store_true", help="also fit k2 (poly)")
    f.set_defaults(fn=cmd_fit)
    t = sub.add_parser("selftest", help="verify the fit machinery on synthetic data")
    t.set_defaults(fn=cmd_selftest)
    a = ap.parse_args()
    a.fn(a)

if __name__ == "__main__":
    main()
