// quad_resolver.h — persistent 4-corner identity plus rigid reconstruction.
// Tracks the four emitters across frames, fits a similarity/affine transform to
// the corners actually seen, and fills in the missing ones. Once locked it
// always emits 4 points in a stable slot order, each flagged real or filled.

#pragma once
#include <stdint.h>

// Max blobs that may be offered per frame; the resolver picks its own four, so
// the caller must not pre-select them by mass.
#define QUAD_MAX_IN 8

// One corner of the quad.
struct QuadPoint {
    float x, y;
    bool  real;      // false = reconstructed from the rigid model (or coasted)
};

// The four corners resolved for one frame.
struct QuadResult {
    QuadPoint p[4];
    int   count;      // 4 once locked; fewer during cold start, or under
                      // veto_seed while no offered set is accepted
    bool  locked;     // model is trusted
    int   n_real;     // how many of the 4 were actually detected this frame
    float confidence; // 0..1 — n_real/4 damped by how long we have extrapolated
    // Rigid-body image velocity in px per frame, averaged over the four slots.
    // For latency lead at publish time: consumers may publish p[] + (vx,vy)*lead.
    // Deliberately not applied to the resolver's own state.
    float vx, vy;
    // Cold raw passthrough: no model yet, and no offered set accepted. p[] is
    // the blobs as handed in, so a consumer must not count them as corners seen.
    bool  passthrough;
    // merge_split: corners this frame that came from splitting a merged
    // blob (0, or 2 per split). They are published as real; a learner of
    // blob shapes should skip the frame, since the merged blob is not an LED.
    int   split;
};

// Tunables for association and model learning.
struct QuadConfig {
    float gate;         // association radius in px, prediction -> blob
    float model_lr;     // EMA rate for re-learning the rig shape (0..1)
    int   lock_frames;  // consecutive all-4-real frames before trusting model
    float max_stretch;  // reject a similarity fit whose scale moves more than
                        // this factor in one frame (guards a bad association)
    // Batch A, wiicam only. With the defaults the OV path is behaviourally
    // identical, except `confidence` on the strict-reseed frame, which was
    // wrong and is now 0; only a diagnostic line reads it. Level 5 (K6).
    bool  veto_seed;    // refuse an implausible four-set at seed/reseed and
                        // condemn a model the residual detector rejects
    bool  partial_lock; // let 3-real frames advance the lock, at half rate
    float cold_aniso_max;  // anisotropy ceiling before the rig has shown one
    // Refuse to seed or re-seed on four blobs whose widest is more than this
    // many times its narrowest: four LEDs of one bar are alike, a window's
    // fragments never are. Relative, inside the frame -- no rig assumed.
    // Needs quad_offer_widths() each frame; 0 = off (the OV path).
    float seed_wratio;
    // The gate a slot widens to after misses (up to 3x) is capped at this
    // fraction of the model's SHORTEST side, never below the base gate: on a
    // rig 35 px across a 60 px gate reaches the next corner and past it, and
    // a stray there is adopted as the missing LED. Relative to what this rig
    // measures; 0 = off (the OV path).
    float gate_cap_ratio;
    // A blob inside the gate of two unmatched slots, with the second-nearest
    // closer than this many times the nearest, is assigned to NEITHER: two
    // LEDs merged into one blob sit at their midpoint, and snapping either
    // corner onto it warps the model and rotates corner identity. The other
    // blobs still associate; the frame is used for what it holds. 0 = off.
    float assoc_ambig_ratio;
    // Re-acquire the model from three blobs with the fourth corner
    // reconstructed, when no four-set matches. The sensor has four object
    // slots; a stray holding one means it never reports all four LEDs, and
    // without this the lock cannot come back while the stray is in view.
    // Off = the OV path.
    bool  reseed3;
    // Two LEDs merged along a row into one blob, refused as ambiguous, are
    // split into their two corners when the blob sits at the midpoint of
    // the two predictions and its width is their separation plus one LED
    // width (from this frame's other blobs). Needs quad_offer_widths().
    // Off = refused and reconstructed, as before.
    bool  merge_split;
};

// Returns the defaults, tuned for a 240x176 sensor at ~135 fps.
QuadConfig quad_default_config(void);

void       quad_reset(const QuadConfig* cfg);   // cfg may be NULL -> defaults
QuadResult quad_update(const float* xs, const float* ys, int n);  // n <= QUAD_MAX_IN
// Box widths of the blobs the NEXT quad_update() is offered, in the same
// order; consumed by that call. Only read when seed_wratio is set.
void       quad_offer_widths(const int* w, int n);
// State as of the last quad_update(). Both are for the CAMERA CORE only: they
// read resolver state with no hold, so a serial-core caller can see it mid-update.
bool       quad_locked(void);     // same flag the last QuadResult carried
bool       quad_has_model(void);  // a rig shape has been learned
// Blobs refused as ambiguous between two slots since boot: a merged LED
// pair, mostly. Cumulative across quad_reset() and never reset by a reader
// (the stats are the OV path's; this one the wiicam log reads as it runs).
uint32_t   quad_ambig_total(void);
// Bumped every time the four slots are bound to blobs afresh (seed, re-seed
// from the model, re-seed from three). Between two equal readings the slot
// order carries the same corner identity, so a consumer may label the slots
// once and keep the labels; a change means label again. Camera core only.
uint32_t   quad_identity_epoch(void);

// Telemetry, reset by the reader.
struct QuadStats {
    uint32_t frames;        // update() calls since last read
    uint32_t reconstructed; // points filled by the AFFINE fit (3+ real)
    uint32_t recon_sim;     // points filled by the 2-real similarity fit
    uint32_t recon_t;       // points filled by the 1-real rung (translation delta)
    uint32_t env_rejects;   // fits refused as implausible for this rig
    uint32_t aniso_x100;    // deformation of the last reconstruction fit, x100
    uint32_t env_aniso_x100;// learned ceiling, x100
    uint32_t resid_x100;    // affine residual on the last 4-real frame, px x100
    uint32_t resid_max_x100;// worst residual since last read, px x100
    uint32_t coasted;       // points filled from velocity (model unusable)
    uint32_t reassoc;       // a slot re-acquired a blob after >=1 miss
    uint32_t dropped_blobs; // detected blobs that matched no slot
    uint32_t relearns;      // model EMA updates (all-4-real frames)
    uint32_t reseeds;       // identity re-acquired using the learned rig shape
    uint32_t lock_losses;   // correspondence abandoned so it could re-acquire
    uint32_t reshapes;      // locked-but-wrong assignment detected and rebuilt
    uint32_t worst_us;      // worst single quad_update(), microseconds
    uint32_t total_us;      // summed quad_update() time, microseconds
    uint32_t giveups;       // veto_seed: a model dropped -- refused every re-acquire, or a near blob it never matched
    uint32_t ambig;         // blobs refused as ambiguous between two slots (a merged LED pair)
    uint32_t splits;        // merged pairs split into two corners (merge_split)
};
QuadStats quad_take_stats(void);

#ifdef QUAD_DEBUG_HOOK
// Test/simulator only, never defined in a firmware build. Read-only window into
// each quad_update(); must not change behaviour.
struct QuadDebugHook {
    int   k, n;              // matched slots, offered blobs
    int   rung;              // 3/2/1 = H-ladder rung, -1 = fallback affine,
                             // -2 = fallback similarity, 0 = none
    int   swap_suspect;      // always 0; legacy field kept so dump formats stay put
    int   sanity_reject;     // rung max_stretch sanity fired
    int   slot_of[4];        // blob index consumed by each slot (-1 = none)
    int   miss[4];           // per-slot miss streak after this frame
    float pair_sep;          // always 0; legacy field
    float rot;               // always 0; legacy field
    float scr;               // any rung: h_scale(Heff)/h_scale(Hr)
    float d_hm[4];           // per reconstructed slot: |H-ladder place minus
                             // model-similarity place| in px; -1 = n/a
    int   hr_valid;          // homography memory state after the frame
    int   h_valid;           // always 0; legacy field kept so dumps stay put
    int   stuck, cbad;       // breaker / give-up clocks after the frame
};
extern QuadDebugHook quad_dbg;
#endif
