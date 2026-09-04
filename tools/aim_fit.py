"""Python port of lib/AimPipeline/aim_core.cpp's fit, used by the calibration tools.
hostcheck/py_c_crosscheck.py asserts the two implementations agree.
"""
import numpy as np

MAGIC = 0x414D4331

def canon_index(q):
    """The permutation canon() applies: wire slots in TL,TR,BL,BR order, so a
    per-slot flag can follow its corner through the sort."""
    q = np.asarray(q, float)
    t0, t1, b0, b1 = np.argsort(q[:,1], kind='stable')
    if q[t0,0] > q[t1,0]: t0, t1 = t1, t0
    if q[b0,0] > q[b1,0]: b0, b1 = b1, b0
    return [int(t0), int(t1), int(b0), int(b1)]

def canon(q):
    """Sort into TL,TR,BL,BR by geometry; must match aim_core.cpp's aim_canon."""
    q = np.asarray(q, float)
    return q[canon_index(q)]


def square_to_quad(q):
    """Heckbert 1989 sec 2.2. q order is TL TR BL BR; his walk is TL TR BR BL."""
    (x0,y0),(x1,y1),(x2,y2),(x3,y3) = q[0],q[1],q[3],q[2]
    dx1,dx2,dx3 = x1-x2, x3-x2, x0-x1+x2-x3
    dy1,dy2,dy3 = y1-y2, y3-y2, y0-y1+y2-y3
    if abs(dx3) < 1e-7 and abs(dy3) < 1e-7:
        return np.array([[x1-x0, x2-x1, x0],[y1-y0, y2-y1, y0],[0.,0.,1.]])
    den = dx1*dy2 - dy1*dx2
    if abs(den) < 1e-9: return None
    a13 = (dx3*dy2 - dy3*dx2)/den
    a23 = (dx1*dy3 - dy1*dx3)/den
    return np.array([[x1-x0+a13*x1, x3-x0+a23*x3, x0],
                     [y1-y0+a13*y1, y3-y0+a23*y3, y0],
                     [a13,          a23,          1.0]])

def quad_to_square(q, px, py):
    m = square_to_quad(canon(q))
    if m is None: return None
    try: inv = np.linalg.inv(m)
    except np.linalg.LinAlgError: return None
    w = inv[2,0]*px + inv[2,1]*py + inv[2,2]
    if abs(w) < 1e-9: return None
    u = (inv[0,0]*px + inv[0,1]*py + inv[0,2])/w
    v = (inv[1,0]*px + inv[1,1]*py + inv[1,2])/w
    if not (-50 < u < 50 and -50 < v < 50): return None
    return u, v

def quad_span(q):
    # mean of the quad's own two diagonals: rotation invariant
    k = np.asarray(canon(q), dtype=float)
    return 0.5*(float(np.hypot(*(k[3]-k[0]))) + float(np.hypot(*(k[2]-k[1]))))

ROLL_MIN = 0.15
FRAME_W_DEF, FRAME_H_DEF = 240.0, 176.0   # native camera frame, for rect_aspect

def quad_roll_sin(q):
    """sin of the quad's roll, from the mean of the top and bottom edge directions."""
    k = np.asarray(canon(q), dtype=float)
    d = (k[1]-k[0]) + (k[3]-k[2])
    nrm = float(np.hypot(*d))
    return 0.0 if nrm < 1e-6 else float(d[1]/nrm)

def rect_aspect(q, fpx):
    """Physical height/width of the LED rectangle from one image; quad must be TL,TR,BL,BR."""
    k = np.asarray(canon(q), dtype=float)
    m = [np.array([p[0], p[1], 1.0]) for p in k]      # TL, TR, BL, BR
    m1, m2, m3, m4 = m
    d2 = np.dot(np.cross(m2, m4), m3)
    d4 = np.dot(np.cross(m3, m4), m2)
    if abs(d2) < 1e-12 or abs(d4) < 1e-12: return float('nan')
    k2 = np.dot(np.cross(m1, m4), m3)/d2
    k3 = np.dot(np.cross(m1, m4), m2)/d4
    n2 = k2*m2 - m1
    n3 = k3*m3 - m1
    K = np.array([[fpx, 0, FRAME_W_DEF*0.5], [0, fpx, FRAME_H_DEF*0.5], [0, 0, 1.0]])
    Ki = np.linalg.inv(K); M = Ki.T @ Ki
    a = float(n2 @ M @ n2); b = float(n3 @ M @ n3)
    if a <= 0 or b <= 0: return float('nan')
    return float(np.sqrt(b/a))


def roll_spread(shots):
    if len(shots) < 2: return 0.0
    r = [quad_roll_sin(sh['q']) for sh in shots]
    return max(r) - min(r)

def span_spread(shots):
    s = [quad_span(sh['q']) for sh in shots]
    return (max(s)/min(s)) if s and min(s) > 1e-6 else 1.0

def _inner(shots, fw, fh, bx, by, lever, use_roll=False):
    """Closed-form regressions for the remaining unknowns at a trial boresight."""
    m = 3 if use_roll else 2
    Mx = np.zeros((m, m+1)); My = np.zeros((m, m+1))
    uvs = []
    for sh in shots:
        byy = by + (lever*quad_span(sh['q']) if lever else 0.0)
        r = quad_to_square(sh['q'], fw*0.5+bx, fh*0.5+byy)
        if r is None: return None
        u,v = r
        sr = quad_roll_sin(sh['q']) if use_roll else 0.0
        uvs.append((u,v,sr))
        cx = np.array([u, 1.0, sr][:m]); cy = np.array([v, 1.0, sr][:m])
        Mx[:, :m] += np.outer(cx, cx); Mx[:, m] += cx*sh['tx']
        My[:, :m] += np.outer(cy, cy); My[:, m] += cy*sh['ty']
    if len(uvs) < m + 1: return None
    try:
        # singular => a regressor is collinear (no roll diversity, or collinear dots)
        if min(abs(np.linalg.det(Mx[:, :m])), abs(np.linalg.det(My[:, :m]))) < 1e-12:
            return None
        px = np.linalg.solve(Mx[:, :m], Mx[:, m])
        py = np.linalg.solve(My[:, :m], My[:, m])
    except np.linalg.LinAlgError:
        return None
    w, B = px[0], px[1]; rx = px[2] if use_roll else 0.0
    h, Bv = py[0], py[1]; ry = py[2] if use_roll else 0.0
    ss = 0.0; per = []
    for (u,v,sr),sh in zip(uvs, shots):
        ex = w*u + B  + rx*sr - sh['tx']; ey = h*v + Bv + ry*sr - sh['ty']
        ss += ex*ex + ey*ey; per.append(np.hypot(ex,ey))
    return ss, w, B, h, Bv, per, float(rx), float(ry)

def _fit_once(shots, fw, fh, solve_lever, use_roll=False):
    bx=by=lever=0.0
    r = _inner(shots, fw, fh, 0,0,0, use_roll)
    if r is None: return None
    best, w, B, h, Bv, per, rx, ry = r
    step = 16.0
    while step > 0.01:
        improved = True
        while improved:
            improved = False
            ls = step*0.01 if solve_lever else 0.0
            cand = [(step,0,0),(-step,0,0),(0,step,0),(0,-step,0),
                    (step,step,0),(-step,-step,0)] + ([(0,0,ls),(0,0,-ls)] if solve_lever else [])
            for dx,dy,dl in cand:
                nx,ny,nl = bx+dx, by+dy, lever+dl
                if abs(nx) > 60 or abs(ny) > 60: continue
                rr = _inner(shots, fw, fh, nx, ny, nl, use_roll)
                if rr is None: continue
                if rr[0] < best - 1e-14:
                    best, w, B, h, Bv, per, rx, ry = rr
                    bx,by,lever = nx,ny,nl; improved = True
        step *= 0.5
    return dict(best=best,bx=bx,by=by,lever=lever,w=w,B=B,h=h,Bv=Bv,per=per,
                rx=rx,ry=ry)

def fit(shots, fw, fh, solve_lever=False):
    """shots: [{'q': (4,2) array, 'tx':, 'ty':}].  Returns (calib_dict, reason)."""
    if len(shots) < 5: return None, "need at least 5 shots, have %d" % len(shots)
    keep = list(shots); rejected = 0
    # roll term only with roll diversity, else sin(roll) is collinear with the intercept
    rspread = roll_spread(shots)
    use_roll = rspread >= ROLL_MIN
    for pas in range(2):
        r = _fit_once(keep, fw, fh, solve_lever, use_roll)
        if r is None: return None, "degenerate geometry (collinear dots, or a bad quad)"
        if pas == 1: break
        per = np.array(r['per'])
        med = np.median(per); mad = np.median(np.abs(per-med))
        thr = med + 3.0*mad + 1e-6
        trimmed = [s for s,e in zip(keep,per) if e <= thr]
        rejected = len(keep) - len(trimmed)
        if len(trimmed) < 6 or span_spread(trimmed) < 1.15:
            rejected = 0; break
        if use_roll and roll_spread(trimmed) < ROLL_MIN:
            rejected = 0; break
        keep = trimmed
    spread = span_spread(keep)
    if spread < 1.15:
        return None, ("all shots were taken at effectively one distance "
                      "(span spread %.3f, need >=1.15). The boresight cannot be "
                      "separated from the screen mapping without a step back." % spread)
    c = dict(magic=MAGIC,
             cx=r['B']+r['w']*0.5, cy=r['Bv']+r['h']*0.5,
             w=r['w'], h=r['h'], bx=r['bx'], by=r['by'], lever=r['lever'],
             rx=r['rx'], ry=r['ry'],
             fit_rms=float(np.sqrt(r['best']/(2.0*len(keep)))),
             fit_spread=spread, fit_roll=rspread,
             n_shots=len(keep), n_rejected=rejected)
    if not (0.02 < c['w'] < 20.0 and 0.02 < c['h'] < 20.0):
        return None, "fitted LED rectangle is implausible (%.3f x %.3f of screen)" % (c['w'],c['h'])
    return c, None

def solve(c, q, fw, fh):
    """runtime path: labelled quad -> normalised screen coords"""
    if not c or c.get('magic') != MAGIC: return None
    by = c['by'] + (c['lever']*quad_span(q) if c['lever'] else 0.0)
    r = quad_to_square(q, fw*0.5+c['bx'], fh*0.5+by)
    if r is None: return None
    u,v = r
    sx = c['w']*u + (c['cx']-c['w']*0.5)
    sy = c['h']*v + (c['cy']-c['h']*0.5)
    rx, ry = c.get('rx', 0.0), c.get('ry', 0.0)
    if rx or ry:
        sr = quad_roll_sin(q)
        sx += rx*sr; sy += ry*sr
    return sx, sy


# ---------------------------------------------------------------------------
# fine-tune: split a sight offset into its angular and parallax parts
# ---------------------------------------------------------------------------
def bore_jacobian(c, q, fw, fh, eps=1.0):
    """d(screen)/d(boresight), at the pose q. 2x2, screen units per native px."""
    r0 = solve(c, q, fw, fh)
    if r0 is None: return None
    J = np.zeros((2, 2))
    for j, k in enumerate(('bx', 'by')):
        cc = dict(c); cc[k] = c[k] + eps
        r1 = solve(cc, q, fw, fh)
        if r1 is None: return None
        J[0, j] = (r1[0] - r0[0]) / eps
        J[1, j] = (r1[1] - r0[1]) / eps
    return J


def split_offset(c, near_q, near_off, far_q, far_off, fw, fh):
    """Split near/far sight offsets (normalised screen units) into (dbx, dby, dcx, dcy)."""
    Jn = bore_jacobian(c, near_q, fw, fh)
    Jf = bore_jacobian(c, far_q, fw, fh)
    if Jn is None or Jf is None: return None
    # unknowns [dbx, dby, dcx, dcy]; the constant part contributes identity
    A = np.zeros((4, 4)); b = np.zeros(4)
    A[0:2, 0:2] = Jn; A[0:2, 2:4] = np.eye(2); b[0:2] = near_off
    A[2:4, 0:2] = Jf; A[2:4, 2:4] = np.eye(2); b[2:4] = far_off
    # too-similar poses make the split unidentifiable
    if abs(np.linalg.det(A)) < 1e-12: return None
    try:
        x = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None
    return float(x[0]), float(x[1]), float(x[2]), float(x[3])


def apply_offset(c, dbx, dby, dcx, dcy):
    out = dict(c)
    out['bx'] = c['bx'] + dbx
    out['by'] = c['by'] + dby
    out['cx'] = c['cx'] + dcx
    out['cy'] = c['cy'] + dcy
    return out
