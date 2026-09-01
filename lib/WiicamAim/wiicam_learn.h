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
// The negative class is labelled by geometry too, and it took a second attempt
// to get there. "A blob the gate rejected" is the obvious answer and it cannot
// produce a single sample on this sensor: there are four object slots, only
// KEPT blobs reach the resolver, so a rejection leaves three points and the
// frame can never be confirmed. The positive class would have filled while the
// negative one stayed structurally empty -- and the module would have looked
// like it was working.
//
// So a negative comes from a frame with three real corners and a reconstructed
// fourth, where exactly one blob was rejected. The reconstruction says where
// the missing LED must be, and a rejected blob sitting nowhere near ANY corner
// was not an LED whatever its size. That is still a positional verdict the
// size gate had no vote in.
//
// Having both classes is the whole point: one distribution tells you what an
// LED looks like, two tell you whether an LED and a window can be told apart
// at all. If they overlap, no gate can separate them and the honest answer is
// to say so rather than tune in the dark.
//
// This module MEASURES. It does not gate, it does not persist, and nothing in
// the aim path reads it. That is deliberate: the features worth gating on are
// the ones the data picks, and the data does not exist yet.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WL_CLASSES 2      // 0 = resolver-confirmed LED, 1 = gate-rejected blob
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

#ifdef __cplusplus
}
#endif
