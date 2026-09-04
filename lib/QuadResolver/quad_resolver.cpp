// quad_resolver.cpp — persistent 4-corner identity and rigid reconstruction.
// Three parts: associate blobs to slots, reconstruct the missing corners, and
// learn the rig shape. See the header for the published contract.

#include "quad_resolver.h"
#include <math.h>
#include <string.h>

// Microsecond clock for the cost meter; a real one on host and target.
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  #include "esp_timer.h"
  static inline int64_t quad_now_us(void) { return esp_timer_get_time(); }
#else
  #include <chrono>
  static inline int64_t quad_now_us(void) {
      using namespace std::chrono;
      return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
  }
#endif

namespace {

// Per-corner tracking state.
struct Slot {
    float x, y;      // last published position
    float vx, vy;    // per-frame velocity, EMA
    bool  live;      // has ever been seeded
    int   miss;      // consecutive frames without a real detection
};

QuadConfig  C;
Slot        S[4];
float       MX[4], MY[4];     // rig model, centroid-normalised
bool        model_valid;
int         lock_count;
bool        locked;
QuadStats   ST;

// ---- learned plausibility envelope and homography memory ------------------
// On every all-4-real frame the true deformation is known, so a slowly decaying
// maximum of what this rig has actually shown bounds later reconstructions.
// The rig is planar, so model->image is exactly a homography (8 DOF); Hr holds
// the most recent effective one, re-anchored by the 4-real solve, the 3-real
// rung and the fits.
float Hr[9]; bool Hr_valid = false;

float       env_aniso_max = 1.0f;    // largest s_max/s_min seen, decayed
bool        env_valid = false;
const float ENV_MARGIN  = 1.30f;     // allow 30% beyond anything observed
const float ENV_DECAY   = 0.99995f;  // ~halves over 3-4 min of play at 135fps
const float ANISO_FLOOR = 1.8f;      // floor for the anisotropy ceiling

const float VEL_LR = 0.35f;   // velocity EMA; loose enough to track a flick

// ---- recovery tunables ----
const float COAST_DAMP  = 0.80f;  // velocity bleed per coasted frame
const float GATE_GROW   = 0.60f;  // gate widening per missed frame
const float GATE_GROW_MAX = 3.0f; // max gate multiplier
const int   RESEED_AFTER = 12;    // frames with <2 real before re-acquiring
int consec_bad = 0;
// Self-heal: a sustained affine residual above this on 4-real frames means the
// slot assignment is wrong.
const float RESHAPE_RESID_PX = 4.0f;
const int   RESHAPE_FRAMES   = 20;      // ~150ms, long enough to ignore a glitch
int reshape_bad = 0;
// Deadlock breaker: blobs keep arriving that every slot refuses.
const int STUCK_AFTER = 20;             // ~150ms of refusing offered blobs
int stuck_cnt = 0;
// partial_lock: every second 3-real frame advances the lock, so this counts the
// phase. Reset on any 4-real or 0-real frame.
int partial_phase = 0;
// veto_seed: the four positions the residual detector just condemned. seed()
// refuses that set back while the ban lives, or the impostor re-seeds itself
// and the model it teaches is never questioned again (R2's other half).
const int BAN_FRAMES = 120;             // ~0.6s at 200fps
float banned_x[4], banned_y[4];
int   banned_ttl = 0;

// Squared distance between two points.
inline float d2(float ax, float ay, float bx, float by) {
    const float dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy;
}

// Ordering-only substitute for atan2f: monotone in atan2, mapped to [0,4).
inline float pseudo_angle(float dx, float dy)
{
    const float s = fabsf(dx) + fabsf(dy);
    if (s < 1e-12f) return 0.0f;
    const float p = dy / s;                       // [-1, 1]
    if (dx < 0.0f) return 2.0f - p;               // Q2: 1..2   Q3: 2..3
    return (dy < 0.0f) ? (4.0f + p) : p;          // Q4: 3..4   Q1: 0..1
}

// Signed area x2 of triangle (a,b,c). Sign = winding.
inline float cross3(float ax, float ay, float bx, float by, float cx, float cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// True if the four points are in convex position -- a cheap prune, since only
// a convex set can be the projection of a rectangle.
bool convex4(const float* x, const float* y)
{
    for (int d = 0; d < 4; ++d) {
        const int a = (d + 1) & 3, b = (d + 2) & 3, c = (d + 3) & 3;
        const float s1 = cross3(x[a], y[a], x[b], y[b], x[d], y[d]);
        const float s2 = cross3(x[b], y[b], x[c], y[c], x[d], y[d]);
        const float s3 = cross3(x[c], y[c], x[a], y[a], x[d], y[d]);
        if ((s1 > 0 && s2 > 0 && s3 > 0) || (s1 < 0 && s2 < 0 && s3 < 0))
            return false;                          // point d is inside the rest
    }
    return true;
}

// The two reconstruction fits. Affine (6 DOF) is exact at 3 points and models
// the weak-perspective parallelogram, so it carries tilt and shear; similarity
// (4 DOF) is exact at 2 but cannot express tilt. 4 real needs no fit at all,
// <=1 real falls back to a velocity coast.

// 2x2 linear part of a fit.
struct Lin2 { float a, b, c, d; };      // [[a b],[c d]]

// Singular values of a 2x2, closed form. s_max/s_min is the fit's anisotropy,
// sqrt(|det|) its scale.
void lin2_svals(const Lin2& L, float* smax, float* smin)
{
    const float E = (L.a + L.d) * 0.5f, F = (L.a - L.d) * 0.5f;
    const float G = (L.c + L.b) * 0.5f, H = (L.c - L.b) * 0.5f;
    const float Q = sqrtf(E * E + H * H), R = sqrtf(F * F + G * G);
    *smax = Q + R;
    *smin = fabsf(Q - R);
}

// Least-squares affine fit of the model subset onto the observed subset;
// exact when k == 3 and the three model points are not collinear.
bool fit_affine(const int* idx, int k, const float* ox, const float* oy,
                Lin2* L, float* cmx_o, float* cmy_o, float* cox_o, float* coy_o)
{
    if (k < 3) return false;
    float cmx = 0, cmy = 0, cox = 0, coy = 0;
    for (int t = 0; t < k; ++t) {
        cmx += MX[idx[t]]; cmy += MY[idx[t]];
        cox += ox[idx[t]]; coy += oy[idx[t]];
    }
    cmx /= k; cmy /= k; cox /= k; coy /= k;

    // Mc*Mc^T (symmetric) and Oc*Mc^T
    float mxx = 0, mxy = 0, myy = 0;
    float axx = 0, axy = 0, ayx = 0, ayy = 0;
    for (int t = 0; t < k; ++t) {
        const float mx = MX[idx[t]] - cmx, my = MY[idx[t]] - cmy;
        const float dx = ox[idx[t]] - cox, dy = oy[idx[t]] - coy;
        mxx += mx * mx; mxy += mx * my; myy += my * my;
        axx += dx * mx; axy += dx * my;
        ayx += dy * mx; ayy += dy * my;
    }
    const float det = mxx * myy - mxy * mxy;
    if (fabsf(det) < 1e-6f) return false;       // model points collinear
    const float i00 =  myy / det, i01 = -mxy / det, i11 =  mxx / det;
    L->a = axx * i00 + axy * i01;   L->b = axx * i01 + axy * i11;
    L->c = ayx * i00 + ayy * i01;   L->d = ayx * i01 + ayy * i11;
    *cmx_o = cmx; *cmy_o = cmy; *cox_o = cox; *coy_o = coy;
    return true;
}

// Similarity fit (rotation + uniform scale), exact at 2 points; 2D Procrustes.
bool fit_similarity(const int* idx, int k, const float* ox, const float* oy,
                    Lin2* L, float* cmx_o, float* cmy_o, float* cox_o, float* coy_o)
{
    if (k < 2) return false;
    float cmx = 0, cmy = 0, cox = 0, coy = 0;
    for (int t = 0; t < k; ++t) {
        cmx += MX[idx[t]]; cmy += MY[idx[t]];
        cox += ox[idx[t]]; coy += oy[idx[t]];
    }
    cmx /= k; cmy /= k; cox /= k; coy /= k;
    float a = 0, b = 0, den = 0;
    for (int t = 0; t < k; ++t) {
        const float mx = MX[idx[t]] - cmx, my = MY[idx[t]] - cmy;
        const float dx = ox[idx[t]] - cox, dy = oy[idx[t]] - coy;
        a += mx * dx + my * dy;
        b += mx * dy - my * dx;
        den += mx * mx + my * my;
    }
    if (den < 1e-6f) return false;
    const float A = a / den, B = b / den;
    if (sqrtf(A * A + B * B) < 1e-4f) return false;
    L->a = A; L->b = -B; L->c = B; L->d = A;
    *cmx_o = cmx; *cmy_o = cmy; *cox_o = cox; *coy_o = coy;
    return true;
}

// Maps model point j through a fit into image coordinates.
inline void lin2_apply(const Lin2& L, float cmx, float cmy, float cox, float coy,
                       int j, float* x, float* y)
{
    const float mx = MX[j] - cmx, my = MY[j] - cmy;
    *x = cox + L.a * mx + L.b * my;
    *y = coy + L.c * mx + L.d * my;
}

// RMS px distance between the fit's prediction and the observed points -- the
// direct measure of the perspective the parallelogram model does not capture.
float fit_residual(const Lin2& L, float cmx, float cmy, float cox, float coy,
                   const int* idx, int k, const float* ox, const float* oy)
{
    float acc = 0;
    for (int t = 0; t < k; ++t) {
        float px, py;
        lin2_apply(L, cmx, cmy, cox, coy, idx[t], &px, &py);
        acc += d2(px, py, ox[idx[t]], oy[idx[t]]);
    }
    return sqrtf(acc / (float)k);
}

// Solves the model->image homography from 4 correspondences (DLT plus gaussian
// elimination). Returns false if the solve is degenerate or implausible.
bool h_solve4(const float* ox, const float* oy, float* H)
{
    double A[8][9] = {{0}};
    for (int i = 0; i < 4; ++i) {
        const double X = MX[i], Y = MY[i], u = ox[i], v = oy[i];
        A[2*i][0]=X; A[2*i][1]=Y; A[2*i][2]=1;
        A[2*i][6]=-X*u; A[2*i][7]=-Y*u; A[2*i][8]=u;
        A[2*i+1][3]=X; A[2*i+1][4]=Y; A[2*i+1][5]=1;
        A[2*i+1][6]=-X*v; A[2*i+1][7]=-Y*v; A[2*i+1][8]=v;
    }
    for (int c = 0; c < 8; ++c) {
        int piv=c; for (int r=c+1;r<8;++r) if (fabs(A[r][c])>fabs(A[piv][c])) piv=r;
        if (fabs(A[piv][c]) < 1e-9) return false;
        if (piv!=c) for (int k2=0;k2<9;++k2){ double t=A[c][k2]; A[c][k2]=A[piv][k2]; A[piv][k2]=t; }
        for (int r=0;r<8;++r){ if (r==c) continue; double f=A[r][c]/A[c][c];
            for (int k2=c;k2<9;++k2) A[r][k2]-=f*A[c][k2]; }
    }
    float Ht[9];
    for (int i = 0; i < 8; ++i) Ht[i]=(float)(A[i][8]/A[i][i]);
    Ht[8]=1.0f;
    if (!((Ht[0]*Ht[4] - Ht[1]*Ht[3]) > 0.0f)) return false;
    // cheirality guard: every model corner must project with w safely positive,
    // else h_apply() divides by ~0 and the NaN slot is unmatchable forever
    for (int i = 0; i < 4; ++i) {
        if (!(Ht[6]*MX[i] + Ht[7]*MY[i] + Ht[8] > 1e-3f)) return false;
    }
    for (int i = 0; i < 9; ++i) H[i] = Ht[i];   // commit only a valid solve
    return true;
}
// Projects a model point through H.
inline void h_apply(const float* H, float X, float Y, float* u, float* v)
{
    const float w = H[6]*X + H[7]*Y + H[8];
    *u = (H[0]*X + H[1]*Y + H[2]) / w;
    *v = (H[3]*X + H[4]*Y + H[5]) / w;
}
// Solves H from 3 points with the perspective terms (g,h) frozen from Hbase:
// the projective equation is then linear in the remaining 6, so this is exact.
bool h_solve3_frozen(const float* Hbase, const int* idx, int k,
                     const float* ox, const float* oy, float* H)
{
    if (k != 3) return false;
    const float g = Hbase[6], h = Hbase[7];
    // same cheirality requirement as h_solve4, for all four model corners
    for (int i = 0; i < 4; ++i) {
        if (!(g*MX[i] + h*MY[i] + 1.0f > 1e-3f)) return false;
    }
    double M[3][3], bu[3], bv[3];
    for (int t = 0; t < 3; ++t) {
        const int i = idx[t];
        const double w = g*MX[i] + h*MY[i] + 1.0;
        M[t][0]=MX[i]; M[t][1]=MY[i]; M[t][2]=1.0;
        bu[t]=ox[i]*w; bv[t]=oy[i]*w;
    }
    double A[3][5];
    for (int r=0;r<3;++r){ A[r][0]=M[r][0]; A[r][1]=M[r][1]; A[r][2]=M[r][2]; A[r][3]=bu[r]; A[r][4]=bv[r]; }
    for (int c=0;c<3;++c){
        int piv=c; for (int r=c+1;r<3;++r) if (fabs(A[r][c])>fabs(A[piv][c])) piv=r;
        if (fabs(A[piv][c])<1e-9) return false;
        if (piv!=c) for (int k2=0;k2<5;++k2){ double t=A[c][k2]; A[c][k2]=A[piv][k2]; A[piv][k2]=t; }
        for (int r=0;r<3;++r){ if(r==c) continue; double f=A[r][c]/A[c][c];
            for (int k2=c;k2<5;++k2) A[r][k2]-=f*A[c][k2]; }
    }
    H[0]=(float)(A[0][3]/A[0][0]); H[1]=(float)(A[1][3]/A[1][1]); H[2]=(float)(A[2][3]/A[2][2]);
    H[3]=(float)(A[0][4]/A[0][0]); H[4]=(float)(A[1][4]/A[1][1]); H[5]=(float)(A[2][4]/A[2][2]);
    H[6]=g; H[7]=h; H[8]=1.0f;
    return (H[0]*H[4] - H[1]*H[3]) > 0.0f;
}
// scale of H's affine part (for the max_stretch sanity guard between frames)
inline float h_scale(const float* H){ return sqrtf(fabsf(H[0]*H[4]-H[1]*H[3])); }

// Scores one candidate 4-point set against the learned rig shape and returns the
// best slot assignment for it. Only the 4 cyclic rotations are geometrically
// possible, and the correct one is the least warped. `mo` is the model's own
// angular order, hoisted by the caller because it is constant across subsets.
bool score_quad(const float* xs, const float* ys, const int* mo,
                float* score_o, float* aniso_o, float pick[4][2])
{
    int oo[4] = {0,1,2,3};
    {   // angular order of the observation about its centroid
        float cx = 0, cy = 0;
        for (int i = 0; i < 4; ++i) { cx += xs[i]; cy += ys[i]; }
        cx *= 0.25f; cy *= 0.25f;
        float oa[4];
        for (int i = 0; i < 4; ++i) oa[i] = pseudo_angle(xs[i] - cx, ys[i] - cy);
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                if (oa[oo[j]] < oa[oo[i]]) { int t = oo[i]; oo[i] = oo[j]; oo[j] = t; }
    }

    const int all4[4] = {0,1,2,3};
    float best = 1e18f, best_aniso = 1e18f; int bestrot = -1;
    for (int r = 0; r < 4; ++r) {
            // slot mo[i] <- observed point oo[(i+r) & 3]
        float tx[4], ty[4];
        for (int i = 0; i < 4; ++i) {
            tx[mo[i]] = xs[oo[(i + r) & 3]];
            ty[mo[i]] = ys[oo[(i + r) & 3]];
        }
        Lin2 L; float a1,a2,a3,a4;
        if (!fit_affine(all4, 4, tx, ty, &L, &a1, &a2, &a3, &a4)) continue;
        if (L.a * L.d - L.b * L.c <= 0.0f) continue;          // no reflections
        float smax, smin; lin2_svals(L, &smax, &smin);
        const float aniso = (smin > 1e-6f) ? (smax / smin) : 1e6f;
        // least-warped wins, with the fit error as a tiebreak
        const float score = aniso + 0.05f * fit_residual(L, a1, a2, a3, a4, all4, 4, tx, ty);
        if (score < best) {
            best = score; best_aniso = aniso; bestrot = r;
            for (int i = 0; i < 4; ++i) { pick[i][0] = tx[i]; pick[i][1] = ty[i]; }
        }
    }
    if (bestrot < 0) return false;
    *score_o = best; *aniso_o = best_aniso;
    return true;
}

// Re-acquires identity by choosing the best four of up to QUAD_MAX_IN blobs and
// matching them to the learned rig shape, so a corner keeps its slot across a
// blackout. Each subset is pruned by convex4() before it is scored.
bool reseed_with_model(const float* xs, const float* ys, int n, bool strict = false)
{
    if (!model_valid || n < 4) return false;

    int mo[4] = {0,1,2,3};
    {   // angular order of the model about its own centroid (centroid is 0,0).
        float ma[4];
        for (int i = 0; i < 4; ++i) ma[i] = pseudo_angle(MX[i], MY[i]);
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                if (ma[mo[j]] < ma[mo[i]]) { int t = mo[i]; mo[i] = mo[j]; mo[j] = t; }
    }

    float bpick[4][2]; float bscore = 1e18f; bool have = false;

    if (n == 4 && !strict) {
        // veto_seed: a lone candidate gets the cheap vetoes too. Cold, the
        // envelope reads 1.0, so cold_aniso_max is the ceiling.
        float ceil_aniso = env_aniso_max * ENV_MARGIN;
        if (ceil_aniso < C.cold_aniso_max) ceil_aniso = C.cold_aniso_max;
        float sc, an;
        if (score_quad(xs, ys, mo, &sc, &an, bpick)
            && (!C.veto_seed
                || (convex4(xs, ys) && an <= ceil_aniso
                    && (sc - an) * 20.0f <= RESHAPE_RESID_PX * 3.0f))) {
            bscore = sc; have = true;
        }
    } else if (n == 4) {
        // strict: one candidate, but every cheap veto -- convex position, the
        // learned envelope, and an absolute residual ceiling junk cannot fake
        float ceil_aniso = env_valid ? (env_aniso_max * ENV_MARGIN) : 1e18f;
        if (env_valid && ceil_aniso < C.cold_aniso_max) ceil_aniso = C.cold_aniso_max;
        float sc, an;
        if (convex4(xs, ys) && score_quad(xs, ys, mo, &sc, &an, bpick)
            && an <= ceil_aniso
            && (sc - an) * 20.0f <= RESHAPE_RESID_PX * 3.0f) {
            bscore = sc; have = true;
        }
    } else {
        float ceil_aniso = env_valid ? (env_aniso_max * ENV_MARGIN) : 1e18f;
        if (env_valid && ceil_aniso < C.cold_aniso_max) ceil_aniso = C.cold_aniso_max;
        int c0, c1, c2, c3;
        for (c0 = 0;      c0 < n - 3; ++c0)
        for (c1 = c0 + 1; c1 < n - 2; ++c1)
        for (c2 = c1 + 1; c2 < n - 1; ++c2)
        for (c3 = c2 + 1; c3 < n;     ++c3) {
            const float qx[4] = { xs[c0], xs[c1], xs[c2], xs[c3] };
            const float qy[4] = { ys[c0], ys[c1], ys[c2], ys[c3] };
            if (!convex4(qx, qy)) continue;
            float sc, an, pk[4][2];
            if (!score_quad(qx, qy, mo, &sc, &an, pk)) continue;
            if (an > ceil_aniso) continue;      // a shape this rig has never shown
            // strict mode also demands a residual an interloper cannot fake
            if (strict && (sc - an) * 20.0f > RESHAPE_RESID_PX * 3.0f) continue;
            if (sc < bscore) {
                bscore = sc; have = true;
                for (int i = 0; i < 4; ++i) { bpick[i][0] = pk[i][0]; bpick[i][1] = pk[i][1]; }
            }
        }
    }
    if (!have) return false;

    for (int i = 0; i < 4; ++i) {
        S[i].x = bpick[i][0]; S[i].y = bpick[i][1];
        S[i].vx = 0; S[i].vy = 0; S[i].live = true; S[i].miss = 0;
    }
    lock_count = 1;
    Hr_valid = false;                                // refreshed on next 4-real
    reshape_bad = 0;                                 // fresh identity, fresh clocks:
    stuck_cnt   = 0;                                 // old streaks must not bill it
    partial_phase = 0;
    ST.reseeds++;
    return true;
}

// True if this angularly-ordered set IS the condemned one, up to a rotation.
// The radius is RESHAPE_RESID_PX, not the association gate: the impostor's
// position is known to a pixel, so a real LED a few px away gets straight back in.
bool banned_match(const float* xs, const float* ys, const int* order)
{
    if (banned_ttl <= 0) return false;
    const float g2 = RESHAPE_RESID_PX * RESHAPE_RESID_PX;
    for (int r = 0; r < 4; ++r) {
        bool all = true;
        for (int i = 0; i < 4 && all; ++i) {
            const int s = order[(i + r) & 3];
            if (d2(xs[s], ys[s], banned_x[i], banned_y[i]) > g2) all = false;
        }
        if (all) return true;
    }
    return false;
}

// Seeds identity from a full, clean quad, in canonical angular slot order so the
// order is reproducible across resets and matches the model's own ordering.
// Returns false when veto_seed refuses the set.
bool seed(const float* xs, const float* ys)
{
    // Only a convex set can be the projection of a rectangle. Anisotropy and
    // residual are measured against a model, and seed() runs with none, so
    // convexity, the ban below and the caller's size veto are the whole
    // cold-start defence.
    if (C.veto_seed && !convex4(xs, ys)) return false;

    float cx = 0, cy = 0;
    for (int i = 0; i < 4; ++i) { cx += xs[i]; cy += ys[i]; }
    cx /= 4; cy /= 4;

    int order[4] = {0, 1, 2, 3};
    float ang[4];
    // pseudo_angle, not atan2f: reseed_with_model() orders the model with the
    // same function, so the two must agree
    for (int i = 0; i < 4; ++i) ang[i] = pseudo_angle(xs[i] - cx, ys[i] - cy);
    for (int i = 0; i < 4; ++i)                       // tiny insertion sort
        for (int j = i + 1; j < 4; ++j)
            if (ang[order[j]] < ang[order[i]]) { int t = order[i]; order[i] = order[j]; order[j] = t; }

    // The set the residual detector just condemned may not walk straight back
    // in; the caller then publishes it raw and un-locked, which is the honest
    // "cannot lock on this" the hwmax loop needs.
    if (C.veto_seed && banned_match(xs, ys, order)) return false;

    for (int i = 0; i < 4; ++i) {
        const int s = order[i];
        S[i].x = xs[s]; S[i].y = ys[s];
        S[i].vx = 0;    S[i].vy = 0;
        S[i].live = true; S[i].miss = 0;
        MX[i] = xs[s] - cx; MY[i] = ys[s] - cy;
    }
    model_valid = true;
    lock_count  = 1;
    Hr_valid = false;                                // rebuilt on next 4-real
    reshape_bad = 0; stuck_cnt = 0;                  // fresh clocks
    partial_phase = 0;
    banned_ttl = 0;                                  // a different set got in
    return true;
}

} // namespace

#ifdef QUAD_DEBUG_HOOK
QuadDebugHook quad_dbg;   // TEST/DEBUG ONLY -- see header
#endif

// Returns the default tuning.
QuadConfig quad_default_config(void)
{
    QuadConfig c;
    c.gate        = 20.0f;   // px; ~650 deg/s of pan margin at 135 fps
    c.model_lr    = 1.0f / 64.0f;
    c.lock_frames = 4;
    c.max_stretch = 1.35f;   // one frame cannot legitimately rescale the rig
    // Batch A off by default: the OV path must not change behaviour.
    c.veto_seed      = false;
    c.partial_lock   = false;
    c.cold_aniso_max = ANISO_FLOOR;   // the existing floor
    return c;
}

// State as of the last quad_update(), for the camera core -- see the header.
bool quad_locked(void)    { return locked; }
bool quad_has_model(void) { return model_valid; }

// Clears all state and installs cfg (NULL = defaults).
void quad_reset(const QuadConfig* cfg)
{
    C = cfg ? *cfg : quad_default_config();
    memset(S, 0, sizeof(S));
    memset(&ST, 0, sizeof(ST));
    for (int i = 0; i < 4; ++i) { MX[i] = MY[i] = 0.0f; }
    model_valid = false;
    lock_count  = 0;
    locked      = false;
    env_aniso_max = 1.0f;
    env_valid = false;
    Hr_valid = false;
    consec_bad = 0;
    reshape_bad = 0;
    stuck_cnt = 0;
    partial_phase = 0;
    banned_ttl = 0;
}

// Returns telemetry accumulated since the last call and zeroes the counters.
QuadStats quad_take_stats(void)
{
    QuadStats s = ST;
    memset(&ST, 0, sizeof(ST));
    return s;
}

// Resolves one frame of detected blobs into four identified corners.
QuadResult quad_update(const float* xs, const float* ys, int n)
{
    QuadResult R;
    memset(&R, 0, sizeof(R));
#ifdef QUAD_DEBUG_HOOK
    memset(&quad_dbg, 0, sizeof(quad_dbg));
    for (int i = 0; i < 4; ++i) { quad_dbg.slot_of[i] = -1; quad_dbg.d_hm[i] = -1.0f; }
#endif
    ST.frames++;
    // RAII cost meter, so every return path is charged including the early ones
    struct TimeGuard {
        int64_t t0;
        TimeGuard() : t0(quad_now_us()) {}
        ~TimeGuard() {
            const uint32_t us = (uint32_t)(quad_now_us() - t0);
            if (us > ST.worst_us) ST.worst_us = us;
            ST.total_us += us;
        }
    } tguard;
    if (n > QUAD_MAX_IN) { ST.dropped_blobs += (n - QUAD_MAX_IN); n = QUAD_MAX_IN; }
    if (banned_ttl > 0) --banned_ttl;      // the seed ban is a frame TTL

    int live = 0;
    for (int i = 0; i < 4; ++i) if (S[i].live) live++;

    // ---- cold start / re-acquire ------------------------------------------
    // 4 or more blobs with a learned model go to the subset search; exactly 4
    // with no model yet takes the angular seed.
    if (live < 4) {
        bool got = false;
        if (n >= 4) got = reseed_with_model(xs, ys, n);
        // angular seed ONLY when there is no learned model: a failed re-acquire
        // must not overwrite MX/MY with whatever four blobs are in frame
        if (!got && n == 4 && !model_valid) got = seed(xs, ys);
        if (got) {
            consec_bad = 0;
        } else {
            // With a learned model a sub-4 blob set is unverifiable, and
            // publishing it raw wakes the consumer's own partial-point
            // machinery. Report nothing; the reseed lands within a few frames.
            if (model_valid) {
                R.count = 0; R.n_real = 0; R.locked = false; R.confidence = 0.0f;
                ST.dropped_blobs += n;
                return R;
            }
            // True cold boot (no model yet): raw passthrough.
            // R.p holds FOUR entries while n may be up to QUAD_MAX_IN.
            const int m = n < 4 ? n : 4;
            for (int i = 0; i < m; ++i) { R.p[i].x = xs[i]; R.p[i].y = ys[i]; R.p[i].real = true; }
            R.count = m; R.n_real = m; R.locked = false; R.confidence = 0.0f;
            R.passthrough = true;        // blobs, not corners -- see the header
            ST.dropped_blobs += (n - m);
            return R;
        }
    }

    // ---- 1. ASSOCIATE: greedy nearest-neighbour, blob -> predicted slot -----
    float px[4], py[4];
    for (int i = 0; i < 4; ++i) { px[i] = S[i].x + S[i].vx; py[i] = S[i].y + S[i].vy; }

    int  slot_of[4] = {-1, -1, -1, -1};   // blob index taken by each slot
    bool blob_used[QUAD_MAX_IN] = {false};
    // a slot that has been missing is less sure where it is, so widen its gate
    float g2[4];
    for (int i = 0; i < 4; ++i) {
        float g = C.gate * (1.0f + GATE_GROW * (float)S[i].miss);
        if (g > C.gate * GATE_GROW_MAX) g = C.gate * GATE_GROW_MAX;
        g2[i] = g * g;
    }

    // Rank by RAW distance: the per-slot gate is an admission test only and never
    // part of the ranking, or a slot that has missed would outcompete a healthy one.
    for (;;) {
        float best = 1e18f; int bs = -1, bb = -1;
        for (int s = 0; s < 4; ++s) {
            if (slot_of[s] >= 0) continue;
            for (int b = 0; b < n; ++b) {
                if (blob_used[b]) continue;
                const float d = d2(px[s], py[s], xs[b], ys[b]);
                if (d > g2[s]) continue;          // admission: this slot's gate
                if (d < best) { best = d; bs = s; bb = b; }   // ranking: raw
            }
        }
        if (bs < 0) break;
        slot_of[bs] = bb; blob_used[bb] = true;
    }
    // veto_seed: a slot re-associating after a miss has a gate up to 3x wide and
    // snapped to anything inside it. Demand that the resulting four-set is still
    // convex, or leave the slot unmatched this frame.
    if (C.veto_seed) {
        for (int s = 0; s < 4; ++s) {
            if (slot_of[s] < 0 || S[s].miss == 0) continue;
            float cx4[4], cy4[4];
            for (int i = 0; i < 4; ++i) { cx4[i] = S[i].x; cy4[i] = S[i].y; }
            cx4[s] = xs[slot_of[s]]; cy4[s] = ys[slot_of[s]];
            if (!convex4(cx4, cy4)) { blob_used[slot_of[s]] = false; slot_of[s] = -1; }
        }
    }
    for (int b = 0; b < n; ++b) if (!blob_used[b]) ST.dropped_blobs++;

    // ---- 2. update matched slots -------------------------------------------
    float ox[4], oy[4];
    int   midx[4], k = 0;
    for (int s = 0; s < 4; ++s) {
        if (slot_of[s] < 0) continue;
        const int b = slot_of[s];
        if (S[s].miss > 0) {
            // Re-association after a miss streak: the innovation is accumulated
            // reconstruction error, not motion, so snap the position and zero the
            // velocity instead of feeding a huge sample into the EMA.
            ST.reassoc++;
            S[s].vx = 0.0f; S[s].vy = 0.0f;
        } else {
            const float nvx = xs[b] - S[s].x, nvy = ys[b] - S[s].y;
            S[s].vx += VEL_LR * (nvx - S[s].vx);
            S[s].vy += VEL_LR * (nvy - S[s].vy);
        }
        S[s].x = xs[b]; S[s].y = ys[b];
        S[s].miss = 0;
        ox[s] = S[s].x; oy[s] = S[s].y;
        midx[k++] = s;
    }
    R.n_real = k;

    // ---- 3. RECONSTRUCT the rest: the homography ladder ---------------------
    // Anchored by Hr, the most recent effective homography; each rung is exact for
    // its DOF count. 3 real: freeze the perspective terms and solve the other 6.
    // 2 real: a stateless model fit, deliberately NOT a delta on Hr, which could
    // never change the quad's shape. 1 real: a translation delta on Hr. 0 real:
    // damped coast. The guards are on CHANGE (orientation in the solvers, and a
    // per-frame scale limit), not on pose, which play legitimately explores.
    float Heff[9]; bool have_H = false; int rung = 0;
    if (Hr_valid && model_valid) {
#ifndef QUAD_EXP_AFFINE3   // experiment-only escape hatch: never defined in firmware
        if (k == 3) {
            if (h_solve3_frozen(Hr, midx, 3, ox, oy, Heff)) { have_H = true; rung = 3; }
        } else
#endif
        if (k == 1) {
            float px0,py0;
            h_apply(Hr, MX[midx[0]], MY[midx[0]], &px0, &py0);
            const float tx=ox[midx[0]]-px0, ty=oy[midx[0]]-py0;
            for (int i=0;i<9;++i) Heff[i]=Hr[i];
            Heff[2]+=tx*Hr[8]; Heff[5]+=ty*Hr[8];
            Heff[0]+=tx*Hr[6]; Heff[1]+=tx*Hr[7];
            Heff[3]+=ty*Hr[6]; Heff[4]+=ty*Hr[7];
            have_H = true; rung = 1;
        }
        if (have_H) {
            const float sc = h_scale(Heff), sr = h_scale(Hr);
#ifdef QUAD_DEBUG_HOOK
            quad_dbg.scr = (sr > 1e-9f) ? sc / sr : 0.0f;
#endif
            if (!(sc > 1e-6f) || sc > sr*C.max_stretch || sc < sr/C.max_stretch) {
                have_H = false; ST.env_rejects++;   // counter now means "rung sanity reject"
#ifdef QUAD_DEBUG_HOOK
                quad_dbg.sanity_reject = 1;
#endif
            }
        }
    }

    // the fits: the only path for k==2, and the 3-real path during the cold phase
    // before any 4-real H exists
    Lin2  L; float cmx = 0, cmy = 0, cox = 0, coy = 0;
    bool  have_fit = false, fit_is_affine = false;
    if (!have_H) {
        if (model_valid && k >= 3 && fit_affine(midx, k, ox, oy, &L, &cmx, &cmy, &cox, &coy)) {
            have_fit = true; fit_is_affine = true;
        } else if (model_valid && k == 2 &&
                   fit_similarity(midx, k, ox, oy, &L, &cmx, &cmy, &cox, &coy)) {
            have_fit = true;
        }
    }

    // Places a reconstructed slot: refuses a non-finite fit and caps the velocity
    // sample at the association gate, so a correction (a rung switching, a
    // strained fit snapping) cannot become coast momentum.
    auto place = [&](int s, float fx, float fy) -> bool {
        if (!(fabsf(fx) < 1e6f && fabsf(fy) < 1e6f)) return false;  // inf/NaN
        float dvx = fx - S[s].x, dvy = fy - S[s].y;
        if (dvx >  C.gate) dvx =  C.gate; else if (dvx < -C.gate) dvx = -C.gate;
        if (dvy >  C.gate) dvy =  C.gate; else if (dvy < -C.gate) dvy = -C.gate;
        S[s].vx += VEL_LR * (dvx - S[s].vx);
        S[s].vy += VEL_LR * (dvy - S[s].vy);
        S[s].x = fx; S[s].y = fy;
        return true;
    };
#ifdef QUAD_DEBUG_HOOK
    Lin2 dbgL; float dbg1, dbg2, dbg3, dbg4; bool dbg_fit = false;
    if (have_H && k >= 2 && model_valid)
        dbg_fit = fit_similarity(midx, k, ox, oy, &dbgL, &dbg1, &dbg2, &dbg3, &dbg4);
#endif
    for (int s = 0; s < 4; ++s) {
        if (slot_of[s] >= 0) continue;
        S[s].miss++;
        bool done = false;
        if (have_H) {
            float fx, fy;
            h_apply(Heff, MX[s], MY[s], &fx, &fy);
#ifdef QUAD_DEBUG_HOOK
            if (dbg_fit) {
                float dmx, dmy;
                lin2_apply(dbgL, dbg1, dbg2, dbg3, dbg4, s, &dmx, &dmy);
                quad_dbg.d_hm[s] = sqrtf(d2(fx, fy, dmx, dmy));
            }
#endif
            if (place(s, fx, fy)) {
                if (rung == 3) ST.reconstructed++;
                else ST.recon_t++;
                done = true;
            }
        } else if (have_fit) {
            float fx, fy;
            lin2_apply(L, cmx, cmy, cox, coy, s, &fx, &fy);
            if (place(s, fx, fy)) {
                if (fit_is_affine) ST.reconstructed++; else ST.recon_sim++;
                done = true;
            }
        }
        if (!done) {
            // damped coast: an undamped slot accelerates out of the frame, is
            // clamped at publish, and can then never re-associate
            S[s].x += S[s].vx; S[s].y += S[s].vy;
            S[s].vx *= COAST_DAMP; S[s].vy *= COAST_DAMP;
            ST.coasted++;
        }
        ox[s] = S[s].x; oy[s] = S[s].y;
    }
    // any rung that produced an H becomes the new prediction base
    if (have_H) { for (int i=0;i<9;++i) Hr[i]=Heff[i]; Hr_valid = true; }
    else if (have_fit) {
        // A fit is a fresh pose measurement, so embed it as the prediction base
        // (an affine or similarity IS an H with g=h=0). Guarded by an absolute
        // scale floor, the max_stretch limit, and preserved orientation.
        const float det = L.a * L.d - L.b * L.c;
        const float s = det > 0.0f ? sqrtf(det) : 0.0f;
        if (s > 1e-3f && (!Hr_valid || (s <= h_scale(Hr) * C.max_stretch
                                     && s >= h_scale(Hr) / C.max_stretch))) {
            Hr[0]=L.a; Hr[1]=L.b; Hr[2]=cox - (L.a*cmx + L.b*cmy);
            Hr[3]=L.c; Hr[4]=L.d; Hr[5]=coy - (L.c*cmx + L.d*cmy);
            Hr[6]=0.0f; Hr[7]=0.0f; Hr[8]=1.0f;
            Hr_valid = true;
        }
    }

    // ---- deadlock breaker ---------------------------------------------------
    // k<=2 with enough refused blobs to rebuild is the deadlock signature; the
    // strict vetoes make a false positive decline, not misfire, and a declined
    // reseed backs off instead of re-running the subset search per frame.
    // partial_lock waits the clock out rather than firing at once, which pinned
    // lock_count at 1 every frame (R5).
    if (k < 4 && n > k) {
        const bool signature = (k <= 2 && n >= 4);
        bool fire = signature && stuck_cnt == 0 && !C.partial_lock;
        if (!fire) fire = (++stuck_cnt >= STUCK_AFTER);
        if (fire) {
            if (reseed_with_model(xs, ys, n, /*strict=*/true)) {
                // identity re-verified from the full blob set; fresh gates,
                // H rebuilt on the next 4-real frame
                Hr_valid = false;
                // it passed convexity, the envelope and the residual: that has
                // earned most of a lock, not one frame of it
                if (C.partial_lock) lock_count = C.lock_frames - 1;
                ST.reshapes++;               // same "locked-but-wrong repaired" meaning
                stuck_cnt = 0;
                // the give-up clock below reads this frame's k, which is stale
                // the instant identity is rebound
                consec_bad = 0;
                // every slot is model-derived after a rebind, so none of them
                // may be published as measured
                for (int s2 = 0; s2 < 4; ++s2) slot_of[s2] = -1;
                R.n_real = 0;
            } else {
                stuck_cnt = STUCK_AFTER / 2; // couldn't verify (junk?) -- retry soon
            }
        }
    } else {
        stuck_cnt = 0;
    }

    // ---- 4. LEARN: rig shape AND the deformation envelope -------------------
    // Both train only on all-4-real frames, where the observation is the ground
    // truth for this pose.
    if (k == 4 || k == 0) partial_phase = 0;   // the phase runs across 3-real frames only
    bool condemned = false;      // veto_seed: this frame's model was just refused
    if (k == 4) {
        Lin2 L4; float a1, a2, a3, a4;
        if (model_valid && fit_affine(midx, 4, ox, oy, &L4, &a1, &a2, &a3, &a4)) {
            float smax, smin;
            lin2_svals(L4, &smax, &smin);
            const float aniso = (smin > 1e-6f) ? (smax / smin) : 1.0f;
            // do not widen the envelope while the residual clock is suspicious
            if (reshape_bad == 0) {
            if (!env_valid) {
                env_aniso_max = aniso;
                env_valid = true;
            } else {
                env_aniso_max *= ENV_DECAY;
                if (aniso > env_aniso_max) env_aniso_max = aniso;
            }
            }
            // perspective the parallelogram model is not capturing, px RMS
            const float r = fit_residual(L4, a1, a2, a3, a4, midx, 4, ox, oy);
            ST.resid_x100 = (uint32_t)(r * 100.0f);
            // Self-heal: a sustained residual on 4-REAL frames proves the
            // assignment does not describe this rig, so force a re-acquire.
            if (r > RESHAPE_RESID_PX) {
                if (++reshape_bad >= RESHAPE_FRAMES) {
                    for (int i = 0; i < 4; ++i) { S[i].live = false; S[i].miss = 0; S[i].vx = S[i].vy = 0; }
                    lock_count = 0; locked = false; reshape_bad = 0;
                    Hr_valid = false;                // pose memory is part of the error
                    ST.reshapes++;
                    if (C.veto_seed) {
                        // A model learned from a wrong four cannot vet the next
                        // reseed, so condemn it too; the reseed then rests on
                        // convexity, anisotropy and the caller's size veto.
                        model_valid = false;
                        for (int i = 0; i < 4; ++i) { MX[i] = MY[i] = 0.0f; }
                        // and ban the four positions themselves, or seed() takes
                        // the impostor straight back on convexity alone.
                        for (int i = 0; i < 4; ++i) {
                            banned_x[i] = S[i].x; banned_y[i] = S[i].y;
                        }
                        banned_ttl = BAN_FRAMES;
                        condemned = true;   // and do not learn from this frame
                    }
                }
            } else if (reshape_bad) {
                reshape_bad--;
            }
            if (ST.resid_x100 > ST.resid_max_x100) ST.resid_max_x100 = ST.resid_x100;
            ST.env_aniso_x100 = (uint32_t)(env_aniso_max * 100.0f);
        }
        // Nothing below may run on a condemned frame: it would re-learn the
        // model, refresh Hr and advance the lock from the four just refused.
        if (!condemned) {
        float cx = 0, cy = 0;
        for (int i = 0; i < 4; ++i) { cx += S[i].x; cy += S[i].y; }
        cx /= 4; cy /= 4;
        // Learn at quarter rate while the residual clock is suspicious: full rate
        // races the 20-frame reshape detector and can erase its evidence, while
        // freezing entirely would break the rig-repositioning promise.
        const float lr = (reshape_bad > 0) ? C.model_lr * 0.25f : C.model_lr;
        for (int i = 0; i < 4; ++i) {
            MX[i] += lr * ((S[i].x - cx) - MX[i]);
            MY[i] += lr * ((S[i].y - cy) - MY[i]);
        }
        model_valid = true;
        ST.relearns++;
        // refresh the homography memory: exact on 4 real corners, and never from
        // a frame the residual clock distrusts. h_solve4 commits only a valid
        // solve, so Hr survives a refused one.
        if (reshape_bad == 0 && h_solve4(ox, oy, Hr)) Hr_valid = true;
        if (lock_count < C.lock_frames) lock_count++;
        }
    } else if (C.partial_lock && R.n_real == 3 && n == 3 && model_valid
               && have_H && rung == 3) {
        // R1: three real corners plus a rung-3 reconstruction is evidence too,
        // at half rate -- so one displaced LED no longer pins the lock off.
        // n == 3 as well: an unmatched blob means the fourth slot may be parked
        // on nothing while the real LED sits outside its gate, and locking a
        // phantom corner is worse than not locking at all.
        // R.n_real, not k: the stuck breaker zeroes it when it rebinds identity.
        if (++partial_phase >= 2) {
            partial_phase = 0;
            if (lock_count < C.lock_frames) lock_count++;
        }
    } else if (lock_count > 0 && k == 0) {
        lock_count--;                      // total loss slowly un-locks
    }
    locked = (lock_count >= C.lock_frames);

    // ---- 4b. GIVE UP AND RE-ACQUIRE ----------------------------------------
    // Sustained <2 real means we are extrapolating blind, so clear `live` and let
    // the next clean quad re-seed. The learned model and envelope deliberately
    // survive: they are what makes the re-seed keep its identity. The clock runs
    // on COUNT, because re-acquire is the only operation that re-verifies it.
    if (k < 2) {
        if (++consec_bad >= RESEED_AFTER) {
            for (int i = 0; i < 4; ++i) { S[i].live = false; S[i].miss = 0; S[i].vx = S[i].vy = 0; }
            lock_count = 0; locked = false; consec_bad = 0;
            Hr_valid = false;                        // stale pose must not seed rungs
            reshape_bad = 0; stuck_cnt = 0;          // dead lock, dead clocks
            ST.lock_losses++;
        }
    } else if (consec_bad > 0) {
        // decay, don't reset: an occasional 2-blob frame must not clear the whole
        // clock and keep a dead lock published forever
        consec_bad--;
    }

    // ---- 5. emit: always four, stable order --------------------------------
    int worst_miss = 0;
    for (int i = 0; i < 4; ++i) {
        R.p[i].x = S[i].x; R.p[i].y = S[i].y;
        R.p[i].real = (slot_of[i] >= 0);
        if (S[i].miss > worst_miss) worst_miss = S[i].miss;
    }
    R.count  = 4;
    R.locked = locked;
    // Rigid velocity, px per frame, for the publish-side latency lead. Mean over
    // all four slots so leading with it is a pure translation and cannot deform
    // the quad. Meaningless while blind, so reported as zero there.
    if (k >= 2) {
        float sx = 0, sy = 0;
        for (int i = 0; i < 4; ++i) { sx += S[i].vx; sy += S[i].vy; }
        R.vx = sx * 0.25f; R.vy = sy * 0.25f;
    } else {
        R.vx = 0.0f; R.vy = 0.0f;
    }
    // confidence: fraction measured, damped by the worst corner's miss streak.
    // From R.n_real, not k: the strict reseed zeroes n_real and k is stale there.
    R.confidence = ((float)R.n_real / 4.0f) / (1.0f + 0.05f * (float)worst_miss);
    // cost metering is handled by tguard's destructor on every return path
#ifdef QUAD_DEBUG_HOOK
    quad_dbg.k = k; quad_dbg.n = n;
    quad_dbg.rung = have_H ? rung : (have_fit ? (fit_is_affine ? -1 : -2) : 0);
    for (int i = 0; i < 4; ++i) { quad_dbg.slot_of[i] = slot_of[i]; quad_dbg.miss[i] = S[i].miss; }
    quad_dbg.hr_valid = Hr_valid ? 1 : 0; quad_dbg.h_valid = 0;   // legacy field
    quad_dbg.stuck = stuck_cnt; quad_dbg.cbad = consec_bad;
#endif
    return R;
}
