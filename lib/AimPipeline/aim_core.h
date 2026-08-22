// aim_core.h -- maps a labelled image quad to a normalised screen coordinate.
// Owns the whole geometry: the quad->screen mapping and the calibration fit.
// Free of ESP-IDF, Arduino and OpenFIRE headers so it is host-testable.
#ifndef AIM_CORE_H
#define AIM_CORE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AIM_CAL_MAGIC   0x414D4331u   /* 'AMC1' */
#define AIM_MAX_SHOTS   64

// A point in native camera px.
// Corner order inside this module is 0=TL 1=TR 2=BL 3=BR; every entry point
// canonicalises first, so callers may pass the four points in any order.
typedef struct { float x, y; } aim_pt_t;

// A fitted calibration: LED rectangle, boresight and correction terms.
typedef struct {
    uint32_t magic;      // AIM_CAL_MAGIC when valid
    float    cx, cy;     // LED rectangle centre, normalised screen coords
    float    w,  h;      // LED rectangle size,   normalised screen coords
    float    bx, by;     // boresight offset from the frame centre, native px
    float    lever;      // optional: by shifts by lever*quad_span (0 = unused)
    float    rx, ry;     // roll correction, normalised screen units per unit
                         // sin(roll).  0 = not fitted.
    float    fit_rms;    // rms residual of the fit, normalised screen units
    float    fit_spread; // max/min quad span across shots; <1.15 => degenerate
    float    fit_roll;   // sin(roll) spread across shots; <0.15 => rx,ry are 0
    uint16_t n_shots;
    uint8_t  n_stances;  // distinct distance clusters seen
    uint8_t  flags;
} aim_calib_t;

// Sorts four points into TL,TR,BL,BR by geometry.
void aim_canon(const aim_pt_t in[4], aim_pt_t out[4]);

// ---- runtime -------------------------------------------------------------

// Returns sin of the quad's roll, or 0 for a degenerate quad.
float aim_quad_roll_sin(const aim_pt_t q[4]);

// Heckbert quad->unit-square.  Returns 0 on a degenerate quad.
int  aim_quad_to_square(const aim_pt_t q[4], float px, float py,
                        float* u, float* v);

// Labelled quad (native px) -> normalised screen coords, which may fall outside
// 0..1 meaning "aiming off-screen".
// Returns 0 if the calibration is invalid or the quad is degenerate.
int  aim_solve(const aim_calib_t* c, const aim_pt_t q[4],
               float frame_w, float frame_h, float* sx, float* sy);

// ---- calibration ---------------------------------------------------------

// One calibration shot: observed quad plus the screen dot aimed at.
typedef struct {
    aim_pt_t q[4];       // observed quad (already averaged over trigger pulls)
    float    tx, ty;     // the dot the user was aiming at, normalised screen
} aim_shot_t;

// The set of calibration shots a fit is computed from.
typedef struct {
    aim_shot_t s[AIM_MAX_SHOTS];
    uint16_t   n;
} aim_shotset_t;

// Empties a shot set.
void aim_shots_reset(aim_shotset_t* ss);
// Appends one shot; returns 0 when the set is full.
int  aim_shots_add(aim_shotset_t* ss, const aim_pt_t q[4], float tx, float ty);

// Fits a calibration by separable least squares. frame_w/h locate the frame
// centre that bx,by are measured from.  Returns 0 and leaves *out invalid if
// the data cannot determine the boresight (e.g. all shots at one distance).
int  aim_calib_fit(const aim_shotset_t* ss, float frame_w, float frame_h,
                   int solve_lever, aim_calib_t* out);

// Returns the quad-span spread across the shot set. <1.15 means the user never
// changed distance and the fit must be rejected.
float aim_shots_span_spread(const aim_shotset_t* ss);

// Returns max-min of sin(roll) across the shot set. Below AIM_ROLL_MIN the roll
// term is not identifiable and aim_calib_fit leaves rx,ry at zero.
#define AIM_ROLL_MIN 0.15f
float aim_shots_roll_spread(const aim_shotset_t* ss);

// Reduces several trigger pulls at one dot to a single quad (component-wise
// median). n may be 1.
void aim_pulls_median(const aim_pt_t* pulls /*[n][4]*/, int n, aim_pt_t out[4]);

// Returns how many shots the fit discarded as outliers.
int  aim_calib_n_rejected(const aim_calib_t* c);

#ifdef __cplusplus
}
#endif
#endif // AIM_CORE_H
