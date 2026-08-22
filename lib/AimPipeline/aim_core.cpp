#include "aim_core.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Heckbert, "Fundamentals of Texture Mapping and Image Warping", MSc thesis,
// UC Berkeley 1989, section 2.2: square->quad in closed form, inverted via the
// adjugate.  His corner order walks the quad (0,0) (1,0) (1,1) (0,1); ours is
// TL TR BL BR, so his q2 is our BR and his q3 is our BL.
// ---------------------------------------------------------------------------

// Sorts four points into TL,TR,BL,BR by geometry.
void aim_canon(const aim_pt_t in[4], aim_pt_t out[4])
{
    int idx[4] = {0,1,2,3};
    // insertion sort by y (4 elements)
    for (int i = 1; i < 4; ++i) {
        const int k = idx[i]; int j = i - 1;
        while (j >= 0 && in[idx[j]].y > in[k].y) { idx[j+1] = idx[j]; --j; }
        idx[j+1] = k;
    }
    const int t0 = idx[0], t1 = idx[1], b0 = idx[2], b1 = idx[3];
    const int TL = (in[t0].x <= in[t1].x) ? t0 : t1;
    const int TR = (in[t0].x <= in[t1].x) ? t1 : t0;
    const int BL = (in[b0].x <= in[b1].x) ? b0 : b1;
    const int BR = (in[b0].x <= in[b1].x) ? b1 : b0;
    out[0] = in[TL]; out[1] = in[TR]; out[2] = in[BL]; out[3] = in[BR];
}

// Builds the unit-square -> quad homography. Returns 0 on a collinear quad.
static int square_to_quad(const aim_pt_t p[4], float m[9])
{
    const float x0 = p[0].x, y0 = p[0].y;   // (0,0) TL
    const float x1 = p[1].x, y1 = p[1].y;   // (1,0) TR
    const float x2 = p[3].x, y2 = p[3].y;   // (1,1) BR
    const float x3 = p[2].x, y3 = p[2].y;   // (0,1) BL

    const float dx1 = x1 - x2, dx2 = x3 - x2, dx3 = x0 - x1 + x2 - x3;
    const float dy1 = y1 - y2, dy2 = y3 - y2, dy3 = y0 - y1 + y2 - y3;

    if (fabsf(dx3) < 1e-7f && fabsf(dy3) < 1e-7f) {      // affine
        m[0] = x1 - x0;  m[1] = x2 - x1;  m[2] = x0;
        m[3] = y1 - y0;  m[4] = y2 - y1;  m[5] = y0;
        m[6] = 0.0f;     m[7] = 0.0f;     m[8] = 1.0f;
        return 1;
    }
    const float den = dx1 * dy2 - dy1 * dx2;
    if (fabsf(den) < 1e-9f) return 0;                    // collinear
    const float a13 = (dx3 * dy2 - dy3 * dx2) / den;
    const float a23 = (dx1 * dy3 - dy1 * dx3) / den;
    m[0] = x1 - x0 + a13 * x1;  m[1] = x3 - x0 + a23 * x3;  m[2] = x0;
    m[3] = y1 - y0 + a13 * y1;  m[4] = y3 - y0 + a23 * y3;  m[5] = y0;
    m[6] = a13;                 m[7] = a23;                 m[8] = 1.0f;
    return 1;
}

// Inverts a 3x3 via its adjugate. Returns 0 if the matrix is singular.
static int adjugate(const float m[9], float r[9])
{
    r[0] = m[4]*m[8] - m[5]*m[7];  r[1] = m[2]*m[7] - m[1]*m[8];  r[2] = m[1]*m[5] - m[2]*m[4];
    r[3] = m[5]*m[6] - m[3]*m[8];  r[4] = m[0]*m[8] - m[2]*m[6];  r[5] = m[2]*m[3] - m[0]*m[5];
    r[6] = m[3]*m[7] - m[4]*m[6];  r[7] = m[1]*m[6] - m[0]*m[7];  r[8] = m[0]*m[4] - m[1]*m[3];
    const float det = m[0]*r[0] + m[1]*r[3] + m[2]*r[6];
    return fabsf(det) > 1e-12f;
}

// Maps a native-px point through the quad -> unit-square warp (runtime path).
int aim_quad_to_square(const aim_pt_t q_in[4], float px, float py, float* u, float* v)
{
    aim_pt_t q[4];
    aim_canon(q_in, q);                 // slot order from the resolver is arbitrary
    float s2q[9], q2s[9];
    if (!square_to_quad(q, s2q))   return 0;
    if (!adjugate(s2q, q2s))       return 0;
    const float w = q2s[6]*px + q2s[7]*py + q2s[8];
    if (fabsf(w) < 1e-9f) return 0;
    *u = (q2s[0]*px + q2s[1]*py + q2s[2]) / w;
    *v = (q2s[3]*px + q2s[4]*py + q2s[5]) / w;
    // Beyond this is the projective pole and must not be published.
    if (!(*u > -50.0f && *u < 50.0f && *v > -50.0f && *v < 50.0f)) return 0;
    return 1;
}

// Double-precision quad->square, used only by the calibration fit.
static int qts_d(const aim_pt_t q_in[4], double px, double py, double* u, double* v)
{
    aim_pt_t q[4];
    aim_canon(q_in, q);
    const double x0=q[0].x, y0=q[0].y, x1=q[1].x, y1=q[1].y;
    const double x2=q[3].x, y2=q[3].y, x3=q[2].x, y3=q[2].y;
    const double dx1=x1-x2, dx2=x3-x2, dx3=x0-x1+x2-x3;
    const double dy1=y1-y2, dy2=y3-y2, dy3=y0-y1+y2-y3;
    double m[9];
    if (fabs(dx3) < 1e-7 && fabs(dy3) < 1e-7) {
        m[0]=x1-x0; m[1]=x2-x1; m[2]=x0;
        m[3]=y1-y0; m[4]=y2-y1; m[5]=y0;
        m[6]=0; m[7]=0; m[8]=1;
    } else {
        const double den = dx1*dy2 - dy1*dx2;
        if (fabs(den) < 1e-9) return 0;
        const double a13=(dx3*dy2-dy3*dx2)/den, a23=(dx1*dy3-dy1*dx3)/den;
        m[0]=x1-x0+a13*x1; m[1]=x3-x0+a23*x3; m[2]=x0;
        m[3]=y1-y0+a13*y1; m[4]=y3-y0+a23*y3; m[5]=y0;
        m[6]=a13;          m[7]=a23;          m[8]=1;
    }
    double r[9];
    r[0]=m[4]*m[8]-m[5]*m[7]; r[1]=m[2]*m[7]-m[1]*m[8]; r[2]=m[1]*m[5]-m[2]*m[4];
    r[3]=m[5]*m[6]-m[3]*m[8]; r[4]=m[0]*m[8]-m[2]*m[6]; r[5]=m[2]*m[3]-m[0]*m[5];
    r[6]=m[3]*m[7]-m[4]*m[6]; r[7]=m[1]*m[6]-m[0]*m[7]; r[8]=m[0]*m[4]-m[1]*m[3];
    const double det = m[0]*r[0]+m[1]*r[3]+m[2]*r[6];
    if (fabs(det) < 1e-12) return 0;
    const double w = r[6]*px + r[7]*py + r[8];
    if (fabs(w) < 1e-9) return 0;
    *u = (r[0]*px + r[1]*py + r[2])/w;
    *v = (r[3]*px + r[4]*py + r[5])/w;
    return (*u > -50.0 && *u < 50.0 && *v > -50.0 && *v < 50.0);
}

// Apparent size of the quad in native px, used as a stand-in for range.
// Mean of the two diagonals, so it is rotation invariant.
static float quad_span(const aim_pt_t q[4])
{
    aim_pt_t k[4]; aim_canon(q, k);
    const float ax = k[3].x - k[0].x, ay = k[3].y - k[0].y;   // TL->BR
    const float bx = k[2].x - k[1].x, by = k[2].y - k[1].y;   // TR->BL
    return 0.5f * (sqrtf(ax*ax + ay*ay) + sqrtf(bx*bx + by*by));
}

// Returns sin of the quad's roll, or 0 for a degenerate quad.
float aim_quad_roll_sin(const aim_pt_t q[4])
{
    aim_pt_t k[4]; aim_canon(q, k);
    // top edge TL->TR plus bottom edge BL->BR; the sum cancels most keystone.
    const float dx = (k[1].x - k[0].x) + (k[3].x - k[2].x);
    const float dy = (k[1].y - k[0].y) + (k[3].y - k[2].y);
    const float n  = sqrtf(dx*dx + dy*dy);
    return (n < 1e-6f) ? 0.0f : (dy / n);
}

// Runtime path: labelled quad (native px) -> normalised screen coords.
int aim_solve(const aim_calib_t* c, const aim_pt_t q[4],
              float frame_w, float frame_h, float* sx, float* sy)
{
    if (!c || c->magic != AIM_CAL_MAGIC) return 0;
    float bx = c->bx, by = c->by;
    if (c->lever != 0.0f) by += c->lever * quad_span(q);
    float u, v;
    if (!aim_quad_to_square(q, frame_w * 0.5f + bx, frame_h * 0.5f + by, &u, &v))
        return 0;
    *sx = c->w * u + (c->cx - c->w * 0.5f);
    *sy = c->h * v + (c->cy - c->h * 0.5f);
    if (c->rx != 0.0f || c->ry != 0.0f) {
        const float sr = aim_quad_roll_sin(q);
        *sx += c->rx * sr;
        *sy += c->ry * sr;
    }
    return 1;
}

// ---------------------------------------------------------------------------
// calibration
// ---------------------------------------------------------------------------

// Empties a shot set.
void aim_shots_reset(aim_shotset_t* ss) { if (ss) ss->n = 0; }

// Appends one shot; returns 0 when the set is full.
int aim_shots_add(aim_shotset_t* ss, const aim_pt_t q[4], float tx, float ty)
{
    if (!ss || ss->n >= AIM_MAX_SHOTS) return 0;
    aim_shot_t* s = &ss->s[ss->n];
    for (int i = 0; i < 4; ++i) s->q[i] = q[i];
    s->tx = tx; s->ty = ty;
    ss->n++;
    return 1;
}

// Median of n floats, sorting the array in place.
static float med_of(float* v, int n)
{
    // insertion sort: n is the number of trigger pulls, single digits
    for (int i = 1; i < n; ++i) {
        const float k = v[i]; int j = i - 1;
        while (j >= 0 && v[j] > k) { v[j+1] = v[j]; --j; }
        v[j+1] = k;
    }
    return (n & 1) ? v[n/2] : 0.5f * (v[n/2 - 1] + v[n/2]);
}

// Reduces n trigger pulls at one dot to a single quad by component-wise median.
void aim_pulls_median(const aim_pt_t* pulls, int n, aim_pt_t out[4])
{
    if (n <= 0) return;
    if (n > 32) n = 32;
    for (int c = 0; c < 4; ++c) {
        float xs[32], ys[32];
        for (int k = 0; k < n; ++k) { xs[k] = pulls[k*4 + c].x; ys[k] = pulls[k*4 + c].y; }
        out[c].x = med_of(xs, n);
        out[c].y = med_of(ys, n);
    }
}

// Returns how many shots the fit discarded as outliers.
int aim_calib_n_rejected(const aim_calib_t* c) { return c ? (int)(c->flags & 0x3F) : 0; }

// Returns the quad-span spread (max/min) across the shot set.
float aim_shots_span_spread(const aim_shotset_t* ss)
{
    if (!ss || ss->n < 2) return 1.0f;
    float mn = 1e30f, mx = 0.0f;
    for (int i = 0; i < ss->n; ++i) {
        const float s = quad_span(ss->s[i].q);
        if (s < mn) mn = s;
        if (s > mx) mx = s;
    }
    return (mn > 1e-6f) ? (mx / mn) : 1.0f;
}

// Returns max-min of sin(roll) across the shot set.
float aim_shots_roll_spread(const aim_shotset_t* ss)
{
    if (!ss || ss->n < 2) return 0.0f;
    float lo = 1.0f, hi = -1.0f;
    for (int i = 0; i < ss->n; ++i) {
        const float r = aim_quad_roll_sin(ss->s[i].q);
        if (r < lo) lo = r;
        if (r > hi) hi = r;
    }
    return hi - lo;
}

// Solves a symmetric m x m system in place (m is 2 or 3). Returns 0 if singular.
static int solve_sym(int m, double A[3][4], double* out)
{
    for (int i = 0; i < m; ++i) {
        int piv = i;
        for (int r = i + 1; r < m; ++r) if (fabs(A[r][i]) > fabs(A[piv][i])) piv = r;
        // A tiny pivot means a regressor is collinear with the others.
        if (fabs(A[piv][i]) < 1e-12) return 0;
        for (int c = 0; c <= m; ++c) { const double t = A[i][c]; A[i][c] = A[piv][c]; A[piv][c] = t; }
        for (int r = 0; r < m; ++r) {
            if (r == i) continue;
            const double f = A[r][i] / A[i][i];
            for (int c = 0; c <= m; ++c) A[r][c] -= f * A[i][c];
        }
    }
    for (int i = 0; i < m; ++i) out[i] = A[i][m] / A[i][i];
    return 1;
}

// Inner linear solve at a trial boresight: a 2-parameter regression per axis
// (plus sin(roll) when use_roll). Returns the sum of squared residuals, or -1.
static double inner_fit(const aim_shotset_t* ss, float frame_w, float frame_h,
                        double bx, double by, double lever, int use_roll,
                        double* w, double* B, double* h, double* Bv,
                        double* rx, double* ry)
{
    const int m = use_roll ? 3 : 2;
    // accumulators for the design columns [u|v, 1, sin(roll)]
    double Mx[3][4] = {{0}}, My[3][4] = {{0}};
    double U[AIM_MAX_SHOTS], V[AIM_MAX_SHOTS], S[AIM_MAX_SHOTS];
    int n = 0;
    for (int i = 0; i < ss->n; ++i) {
        const aim_shot_t* s = &ss->s[i];
        double byy = by;
        if (lever != 0.0) byy += lever * (double)quad_span(s->q);
        double u, v;
        if (!qts_d(s->q, frame_w*0.5 + bx, frame_h*0.5 + byy, &u, &v))
            return -1.0;
        U[i] = u; V[i] = v; S[i] = use_roll ? (double)aim_quad_roll_sin(s->q) : 0.0;
        const double cx[3] = { u, 1.0, S[i] };
        const double cy[3] = { v, 1.0, S[i] };
        for (int a = 0; a < m; ++a) {
            for (int b = 0; b < m; ++b) { Mx[a][b] += cx[a]*cx[b]; My[a][b] += cy[a]*cy[b]; }
            Mx[a][m] += cx[a] * s->tx;
            My[a][m] += cy[a] * s->ty;
        }
        n++;
    }
    if (n < m + 1) return -1.0;

    double px[3] = {0,0,0}, py[3] = {0,0,0};
    if (!solve_sym(m, Mx, px)) return -1.0;
    if (!solve_sym(m, My, py)) return -1.0;
    *w = px[0]; *B = px[1]; *rx = use_roll ? px[2] : 0.0;
    *h = py[0]; *Bv = py[1]; *ry = use_roll ? py[2] : 0.0;

    double ss2 = 0.0;
    for (int i = 0; i < ss->n; ++i) {
        const double rxr = *w * U[i] + *B  + *rx * S[i] - ss->s[i].tx;
        const double ryr = *h * V[i] + *Bv + *ry * S[i] - ss->s[i].ty;
        ss2 += rxr*rxr + ryr*ryr;
    }
    return ss2;
}

// Per-shot residual magnitude under a given solution, for outlier trimming.
static double shot_resid(const aim_shot_t* s, float frame_w, float frame_h,
                         double bx, double by, double lever,
                         double w, double B, double h, double Bv,
                         double rcx, double rcy)
{
    double byy = by;
    if (lever != 0.0) byy += lever * (double)quad_span(s->q);
    double u, v;
    if (!qts_d(s->q, frame_w*0.5 + bx, frame_h*0.5 + byy, &u, &v))
        return 1e9;
    const double sr = (rcx != 0.0 || rcy != 0.0) ? (double)aim_quad_roll_sin(s->q) : 0.0;
    const double rx = w * u + B  + rcx * sr - s->tx;
    const double ry = h * v + Bv + rcy * sr - s->ty;
    return sqrt(rx*rx + ry*ry);
}

static int fit_once(const aim_shotset_t* ss, float frame_w, float frame_h,
                    int solve_lever, int use_roll,
                    double* obx, double* oby, double* olev,
                    double* ow, double* oB, double* oh, double* oBv,
                    double* orx, double* ory, double* obest);

// Fits a calibration from a shot set: fit, trim outliers, refit, validate.
int aim_calib_fit(const aim_shotset_t* ss, float frame_w, float frame_h,
                  int solve_lever, aim_calib_t* out)
{
    if (!ss || !out || ss->n < 5) return 0;
    memset(out, 0, sizeof(*out));

    // Pass 1 fits everything; pass 2 drops shots beyond median + 3*MAD and refits.
    aim_shotset_t keep = *ss;
    int rejected = 0;
    // Fit the roll term only when the shots contain roll diversity; otherwise
    // sin(roll) is collinear with the intercept.
    const float roll_spread = aim_shots_roll_spread(ss);
    const int use_roll = (roll_spread >= AIM_ROLL_MIN) ? 1 : 0;
    for (int pass = 0; pass < 2; ++pass) {
        double bx, by, lev, w, B, h, Bv, rcx, rcy, best;
        if (!fit_once(&keep, frame_w, frame_h, solve_lever, use_roll,
                      &bx, &by, &lev, &w, &B, &h, &Bv, &rcx, &rcy, &best)) return 0;
        if (pass == 1) break;

        double r[AIM_MAX_SHOTS];
        for (int i = 0; i < keep.n; ++i)
            r[i] = shot_resid(&keep.s[i], frame_w, frame_h, bx, by, lev, w, B, h, Bv, rcx, rcy);
        float tmp[AIM_MAX_SHOTS];
        for (int i = 0; i < keep.n; ++i) tmp[i] = (float)r[i];
        const float med = med_of(tmp, keep.n);
        for (int i = 0; i < keep.n; ++i) tmp[i] = (float)fabs(r[i] - med);
        const float mad = med_of(tmp, keep.n);
        const double thr = (double)med + 3.0 * (double)mad + 1e-6;

        aim_shotset_t trimmed; trimmed.n = 0;
        for (int i = 0; i < keep.n; ++i) {
            if (r[i] <= thr || trimmed.n + (keep.n - i) <= 6) {
                trimmed.s[trimmed.n++] = keep.s[i];
            } else rejected++;
        }
        // Never trim below what the fit needs, nor away the diversity it depends on.
        if (trimmed.n < 6 || aim_shots_span_spread(&trimmed) < 1.15f) { rejected = 0; break; }
        if (use_roll && aim_shots_roll_spread(&trimmed) < AIM_ROLL_MIN) { rejected = 0; break; }
        keep = trimmed;
    }
    const aim_shotset_t* use = &keep;

    double bx = 0.0, by = 0.0, lever = 0.0;
    double w=0, B=0, h=0, Bv=0, rcx=0.0, rcy=0.0, best = 0.0;
    if (!fit_once(use, frame_w, frame_h, solve_lever, use_roll,
                  &bx, &by, &lever, &w, &B, &h, &Bv, &rcx, &rcy, &best)) return 0;

    const float spread = aim_shots_span_spread(use);
    // Degeneracy gate: without distance diversity the boresight is not identifiable.
    if (spread < 1.15f) return 0;

    out->cx = (float)(B + w * 0.5);
    out->cy = (float)(Bv + h * 0.5);
    out->w  = (float)w;
    out->h  = (float)h;
    out->bx = (float)bx;
    out->by = (float)by;
    out->lever = (float)lever;
    out->rx = (float)rcx;
    out->ry = (float)rcy;
    out->fit_roll = roll_spread;
    out->fit_rms = (float)sqrt(best / (2.0 * ss->n));
    out->fit_spread = spread;
    out->n_shots = use->n;
    out->flags = (uint8_t)(rejected & 0x3F);
    // An implausible rectangle means the solve latched onto a wrong branch.
    if (!(out->w > 0.02f && out->w < 20.0f && out->h > 0.02f && out->h < 20.0f))
        return 0;
    out->magic = AIM_CAL_MAGIC;
    return 1;
}

// Outer coarse-to-fine pattern search over the boresight (and lever), bounded
// to +/-60 native px. Returns 0 if no inner fit was possible.
static int fit_once(const aim_shotset_t* ss, float frame_w, float frame_h,
                    int solve_lever, int use_roll,
                    double* obx, double* oby, double* olev,
                    double* ow, double* oB, double* oh, double* oBv,
                    double* orx, double* ory, double* obest)
{
    double bx = 0.0, by = 0.0, lever = 0.0, w = 0, B = 0, h = 0, Bv = 0;
    double rcx = 0.0, rcy = 0.0;
    double best = inner_fit(ss, frame_w, frame_h, 0, 0, 0, use_roll, &w, &B, &h, &Bv, &rcx, &rcy);
    if (best < 0.0) return 0;

    for (double step = 16.0; step > 0.01; step *= 0.5) {
        int improved = 1;
        while (improved) {
            improved = 0;
            const double lstep = solve_lever ? step * 0.01 : 0.0;
            const double cand[8][3] = {
                { step,0,0},{-step,0,0},{0, step,0},{0,-step,0},
                { step,step,0},{-step,-step,0},{0,0,lstep},{0,0,-lstep}
            };
            for (int k = 0; k < (solve_lever ? 8 : 6); ++k) {
                const double nx = bx + cand[k][0], ny = by + cand[k][1];
                const double nl = lever + cand[k][2];
                if (fabs(nx) > 60.0 || fabs(ny) > 60.0) continue;
                double tw, tB, th, tBv, trx, try_;
                const double e = inner_fit(ss, frame_w, frame_h, nx, ny, nl, use_roll,
                                           &tw, &tB, &th, &tBv, &trx, &try_);
                if (e >= 0.0 && e < best - 1e-14) {
                    best = e; bx = nx; by = ny; lever = nl;
                    w = tw; B = tB; h = th; Bv = tBv; rcx = trx; rcy = try_;
                    improved = 1;
                }
            }
        }
    }
    *obx=bx; *oby=by; *olev=lever; *ow=w; *oB=B; *oh=h; *oBv=Bv;
    *orx=rcx; *ory=rcy; *obest=best;
    return 1;
}
