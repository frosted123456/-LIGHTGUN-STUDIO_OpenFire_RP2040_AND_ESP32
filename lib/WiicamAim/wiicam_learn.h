// wiicam_learn.h -- what an LED actually looks like to THIS rig, measured.
//
// The size window and the odd-one-out gate both rest on a guess about blob
// shape. This module replaces the guess with a distribution, and it does so
// without ever learning from its own decisions.
//
// THE LABEL IS GEOMETRY, NOT SHAPE. A gate that learns from "blobs I kept" and
// then keeps blobs by what it learned is a feedback loop: drift once and it
// never comes back, and the failure is silent -- the gun works, and then one
// day it has learned the window. So the positive class is taken only from
// frames where the QUAD RESOLVER found four real corners: four blobs forming a
// plausible rectangle at a plausible scale are LEDs, and that judgement comes
// from where they sit, not from what they look like. Nothing downstream of the
// size gate votes on what the size gate is taught.
//
// The negative class is labelled by geometry too, and it took three attempts
// to get there. "A blob the gate rejected" is the obvious answer and it cannot
// produce a single sample on a gun as shipped: every gate defaults to off, so
// nothing is ever rejected, and the negative class stays structurally empty
// on exactly the gun that most needs a verdict. Nor is a rejection what a
// window looks like on this sensor -- with four object slots a bright window
// DISPLACES an LED, and arrives as an ordinary kept blob.
//
// So a negative comes from a frame the resolver has LOCKED with three real
// corners and a reconstructed fourth, where exactly one blob -- kept or not
// -- sits farther than twice the association radius from every corner. The
// reconstruction says where the missing LED must be, and a blob nowhere near
// ANY corner was not an LED whatever its size. That is a positional verdict
// no size or shape gate had a vote in.
//
// Having both classes is the whole point: one distribution tells you what an
// LED looks like, two tell you whether an LED and a window can be told apart
// at all. If they overlap, no gate can separate them and the honest answer is
// to say so rather than tune in the dark.
//
// This module MEASURES; it does not gate. What reads it: '~camfit' turns the
// two edges into a ceiling (wiicam_aim.cpp), the bhmax/pxmax setters use the
// LED edge as a refusal floor, and camsave / camfit=apply persist that edge in
// the 'fit0' key (aim_runtime.h) so the floor survives a power cycle. Nothing
// in the per-frame aim path reads it.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WL_CLASSES 2      // 0 = resolver-confirmed LED, 1 = resolver-placed stray
#define WL_NFEAT   6
#define WL_BINS    32

// The features, and the exact mapping each one uses to land in a bin. Every
// one is clamped rather than wrapped: an outlier belongs in the end bin, where
// it is visible, not folded back into the middle where it corrupts the shape.
//
//   WL_SZ      the 4-bit reported size          bin = size            (0..15)
//   WL_BW      box width, native px             bin = min(w, 31)
//   WL_BH      box height, native px            bin = min(h, 31)
//   WL_ASPECT  longer side / shorter side x8    bin = clamp(r8,8,39)-8
//              so bin 0 is a perfectly round blob, bin 24 is exactly 4:1,
//              and bins above that run to 4.9:1 before the clamp pins them.
//              This is the feature that should survive ANY bar geometry and
//              ANY viewing angle: an LED is a point source, so its blob is set
//              by the optics and the blooming, not by the shape of the emitter.
//   WL_AREA    w*h, native px                   bin = min(w*h, 31)
//   WL_IREL    intensity / frame median x16     bin = clamp(16*i/med, 0, 31)
//              RELATIVE, because distance changes intensity by about 1/r^2 and
//              any absolute threshold breaks the moment the player steps back.
//              Bin 16 is "exactly the median of this frame".
enum {
    WL_SZ = 0, WL_BW, WL_BH, WL_ASPECT, WL_AREA, WL_IREL
};

// Box-derived features need full mode. In extended only WL_SZ is fed, and the
// rest of that class's histograms stay empty rather than filling with zeroes
// that would read as a real measurement of a round, area-zero blob.
#define WL_HAS_BOX 1

void wl_reset(void);
void wl_enable(int on);
int  wl_enabled(void);

// One blob. cls is WL_CLASSES-bounded; sz is 0..15 or negative when unknown;
// bw/bh/intens are meaningful only when flags carries WL_HAS_BOX; imed is the
// frame's median intensity and may be 0, in which case WL_IREL is skipped
// rather than divided by zero.
void wl_note(int cls, int sz, int bw, int bh, int intens, int imed, int flags);

// Read-out. hist() returns WL_BINS counts, or 0 for an out-of-range request;
// blobs() is how many blobs went into that class, frames() how many frames
// contributed to the positive one. The serial surface lives in wiicam_aim.cpp
// with the rest of the '~cam' commands, so there is one place that formats a
// reply and one place that owns the sink.
const uint16_t* wl_hist(int cls, int feat);
uint32_t wl_blobs(int cls);
uint32_t wl_frames(void);
void     wl_note_frame(void);      // one frame contributed to the positive class

// Feature names, for the report and the tools. Index with the enum above;
// out of range returns "?".
const char* wl_feat_name(int feat);

// ---- the envelope -----------------------------------------------------------
// What the two distributions say at their edges, which is all a ceiling needs.
// Read from the histograms rather than tracked separately so there is one
// source of truth.
//
// The LED HEIGHT edge is the top of the CONTIGUOUS body of the distribution --
// the connected run of bins holding the most samples -- not the absolute
// highest occupied bin. The reason
// is in the data: a daylight capture with no gate came back with LED heights
// 2..7 and then, after twenty-three empty bins, 32 samples at 31 -- the sun,
// learned as an LED. The resolver's lock is self-consistency, not geometry: at
// a cold start it seeds on whichever four blobs it sees and learns that shape,
// so sun-plus-three-LEDs locks like anything else until the player moves and
// the parallax breaks it. Those frames are honest "locked, four real corners"
// and there is no flag to exclude them by. What they cannot do is CONNECT to
// the LED body -- a point source and a window are a dozen bins apart -- and
// that is the only signature the sink can act on.
//
// It matters because this edge is also the refusal FLOOR under bhmax. Read as
// the absolute max, that capture would have set the floor at 31 and refused
// the bhmax of 8 that was measured to catch 84% of strays at zero LED cost:
// the lamp learned as an LED is then what refuses the cut that would have
// caught it. Read as the body, the floor is 7 and 8 is accepted.
//
// Only HEIGHT gets this treatment. Area is a product (w*h) and has arithmetic
// holes -- no blob has area 13 -- so a contiguity walk on it stops at the
// first prime and reports a floor far below the real LEDs, which is the
// dangerous direction. The stray edge stays an absolute MIN: an outlier there
// can only shrink the gap, which refuses a gate rather than loosening it.
typedef struct {
    int led_max_h;      // top of the LED height body; -1 if nothing measured
    int led_max_px;     // absolute largest LED box area; -1 if none
    int stray_min_h;    // shortest confirmed stray box; -1 if none measured
    int stray_min_px;
    int led_abs_max_h;  // absolute highest LED-height bin, for the report
    unsigned long led_outliers_h;   // LED samples ABOVE the body: contamination
    unsigned long led_n, stray_n;
} wl_envelope_t;

void wl_envelope(wl_envelope_t* out);

#ifdef __cplusplus
}
#endif
