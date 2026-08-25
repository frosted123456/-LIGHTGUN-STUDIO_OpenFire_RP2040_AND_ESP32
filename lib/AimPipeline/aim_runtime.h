// aim_runtime.h -- the live aiming path, its persistence and serial control.
// Owns the one active calibration and turns a native-px quad into a normalised
// screen coordinate. Behind USE_AIM_PIPELINE so it can be compiled out.
#ifndef AIM_RUNTIME_H
#define AIM_RUNTIME_H

#include "aim_core.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Loads the stored calibration from NVS at boot. Safe when nothing is stored.
void aim_runtime_begin(void);

bool aim_runtime_active(void);          // a valid calibration is loaded AND enabled
void aim_runtime_enable(bool on);       // runtime toggle, not persisted
const aim_calib_t* aim_runtime_calib(void);

// Installs a calibration and optionally persists it. Returns false (changing
// nothing) if it fails the same validity checks aim_calib_fit applies.
bool aim_runtime_set(const aim_calib_t* c, bool persist);
bool aim_runtime_clear(void);           // forget it, back to stock behaviour

// ---- output smoothing ----------------------------------------------------
//
// One Euro filter (Casiez, Roussel & Vogel, CHI 2012) on the output pair.
//   min_cutoff  Hz. Lower = smoother when still, but slower to settle.
//   beta        how fast the cutoff opens with speed. Its derivative is in
//               normalised screen widths per second, so beta is order 10.
// Tune live with "aimfilt=<min_cutoff>,<beta>"; "aimfilt=0" disables it.

// Sets the filter coefficients and resets its history.
void  aim_filter_set(float min_cutoff, float beta);
// Discards the filter history.
void  aim_filter_reset(void);
float aim_filter_min_cutoff(void);
float aim_filter_beta(void);

// Smoothing level: one knob over the One Euro pair. 0 = filter off,
// 1 = lightest, 10 = heaviest; 3 is the default and matches the build-time
// coefficients. min_cutoff sets the AT-REST lag; beta stays high at every
// level so fast motion is never filtered. Rest lag by level, in ms:
//   L1 20  L2 32  L3 45  L4 64  L5 76  L6 88  L7 103  L8 118  L9 138  L10 159
void aim_smooth_set(int level);
int  aim_smooth_get(void);
// Persisted under its own NVS key, like the lead.
bool aim_smooth_load(int* out_level);
bool aim_smooth_store(int level);

// Speed sensitivity of the One Euro filter. Its cutoff is
//   cutoff = min_cutoff + beta * speed
// so `smooth` sets how heavy the filter is AT REST and beta sets how fast it
// lets go once the gun moves. Raise beta if the cursor trails on a swipe but
// feels fine when settled; lower it if fast motion looks noisy. Range 0..60,
// -1 = follow the smoothing table (15 at every level, the default).
void aim_beta_set(int beta);
int  aim_beta_get(void);            // -1 when following the table
bool aim_beta_load(int* out_beta);
bool aim_beta_store(int beta);

// Temporal mode. 0 (default) is the shipped pair: One Euro on the output plus
// the latency lead applied to the quad in the capture layer. 1 replaces both
// with a single causal least-squares fit over the last k samples, evaluated
// `lead` ahead of the newest one. Mode 1 is COMPILED OUT unless AIM_FIR_MODE
// is defined -- it lost to mode 0 on hardware, because a fixed window cannot
// reproduce One Euro's speed-adaptive cutoff. It is kept, tested and off. Smoothing and prediction come from the same
// fit, so the window sets the smoothing and the horizon sets the prediction
// independently, instead of the filter's lag cancelling the lead's gain.
void aim_tmode_set(int mode);
int  aim_tmode_get(void);
bool aim_tmode_load(int* out_mode);
bool aim_tmode_store(int mode);

// FIR shape, used only in mode 1. k is the window in samples (3..15; 7 is the
// default). pct is the horizon as a percentage of the stored lead (0..140; 100
// is the default, so the same `lead` means the same compensation in both
// modes and switching mode is a fair comparison). Both are persisted in one NVS key.
//
// k is the smoothing knob and pct the prediction knob, and they are
// independent -- which is the point, since mode 0's filter lag and lead partly
// cancel. Note the fit is evaluated at the window's newest end, so its weights
// are mixed-sign even at pct=0: it is a WEAKER pure noise filter than One Euro
// at the same span, and only comes out ahead once the lead is counted, because
// it predicts without amplifying. Do not read k as a One Euro level.
void aim_fir_set(int k, int pct);
int  aim_fir_k(void);
int  aim_fir_pct(void);
bool aim_fir_load(int* out_k, int* out_pct);
bool aim_fir_store(int k, int pct);

// The capture layer owns the lead value and how much of the quad was really
// seen; mode 1 needs both, so it reports them here. conf is the fraction of
// corners MEASURED this frame (n_real/4), not the resolver's miss-damped
// confidence. It scales the horizon: a coasting resolver is the one way a
// mixed-sign filter can leave the range of its own inputs. Widening the
// window lowers the noise gain at the same horizon, so a wider window is the
// answer to a jittery prediction rather than a shorter lead.
void aim_lead_note(float ms);
void aim_conf_note(float conf);

// Output dead-band, in final output units at the move call. Swallows
// sub-threshold shimmer around the last SENT position; motion at or above the
// threshold passes with no added delay. 0 = off, the default; ~16-32 suits a
// wide lens. The patches call aim_dead_pass() just before the cursor move.
bool aim_dead_pass(int x, int y);
void aim_dead_set(int units);
int  aim_dead_get(void);
bool aim_dead_load(int* out_units);
bool aim_dead_store(int units);

// The hot path. quad in native camera px, corner order TL TR BL BR.
// Writes normalised screen coords (may fall outside 0..1 = aiming off-screen).
// Returns false if there is no calibration or the quad is degenerate, in which
// case the caller must fall through to its own solver.
// dt_s is the time since the previous solve, in seconds; pass <=0 and the
// nominal frame period is assumed. The filter runs INSIDE this call.
bool aim_runtime_solve(const aim_pt_t q[4], float frame_w, float frame_h,
                       float* sx, float* sy, float dt_s);

// Call every Run-loop iteration with the trigger state. On the press edge, and
// only while capture mode is on ("aimcap=1"), emits "T,<ms>" to the out sink.
void aim_runtime_trigger_tick(bool pressed);
bool aim_runtime_capture_on(void);

// Pointer gate. When false the Run-mode absolute mouse move is skipped, so the
// gun stops driving the cursor while everything else keeps working.
// NOT PERSISTED: lives in RAM and boots enabled.
bool aim_runtime_hid_enabled(void);
void aim_runtime_hid_set(bool on);

// ---- camera settings persistence ----------------------------------------
//
// Stored in the same NVS namespace as the calibration but under its own key,
// so the two are independent -- clearing one does not touch the other.

// Persisted camera tuning values.
typedef struct {
    int thr, aec, agc, boost;
} aim_cam_t;

// Latency lead in milliseconds, persisted under its own key so growing
// aim_cam_t never invalidates it (its stored blob is size-checked on load).
bool aim_lead_load(int* out_ms);       // false if nothing stored
bool aim_lead_store(int ms);

bool aim_cam_load(aim_cam_t* out);     // false if nothing stored
bool aim_cam_store(const aim_cam_t* c);
bool aim_cam_clear(void);

// Lens undistortion parameters, own NVS key so aim_cam_t's size check stays
// untouched. model 0=off, 1=poly barrel (k1,k2), 2=fisheye equidistant.
typedef struct {
    int   model;
    float k1, k2;
    float fpx, feq;
    // Distortion-centre offset from the frame centre, in native px.
    // Non-zero for a decentered clip-on lens; the fitter measures it.
    float cx, cy;
} aim_lens_t;

bool aim_lens_load(aim_lens_t* out);
bool aim_lens_store(const aim_lens_t* c);
bool aim_lens_clear(void);

// Boot forensics: a silent idle reboot becomes diagnosable from one ~ping.
uint32_t    aim_boot_count(void);
const char* aim_reset_reason(void);
uint32_t    aim_uptime_s(void);

// Installs an output sink so replies go back on the channel the command arrived
// on. Pass 0 for printf.
void aim_set_out(void (*fn)(const char*));

// Feed every byte arriving on OpenFIRE's Serial here BEFORE their parser sees
// it. Returns true if we claimed the byte, in which case the caller must
// consume it; false means it is theirs and we have not touched it.
// Our lines are prefixed '~', absent from their protocol:
//   ~aimcal=...   ~aimfilt=...   ~aimcap=1   ~cam=thr:60,aec:40
bool aim_serial_rx(char ch);

// Installs a handler for claimed lines this module does not own (e.g. camera
// tuning). Called with the '~' already stripped.
void aim_serial_set_extra(bool (*fn)(const char* line));

// Handles "aimcal=..." / "aimcal?" / "aimcal=off" / "aimcal=clear" / "aimcap=".
// Returns true if the line was ours. Replies are printed by the callee.
bool aim_runtime_command(const char* line);

#ifdef __cplusplus
}
#endif
#endif // AIM_RUNTIME_H
