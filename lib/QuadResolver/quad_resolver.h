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
    int   count;      // 4 once locked; fewer only during cold start
    bool  locked;     // model is trusted
    int   n_real;     // how many of the 4 were actually detected this frame
    float confidence; // 0..1 — n_real/4 damped by how long we have extrapolated
    // Rigid-body image velocity in px per frame, averaged over the four slots.
    // For latency lead at publish time: consumers may publish p[] + (vx,vy)*lead.
    // Deliberately not applied to the resolver's own state.
    float vx, vy;
};

// Tunables for association and model learning.
struct QuadConfig {
    float gate;         // association radius in px, prediction -> blob
    float model_lr;     // EMA rate for re-learning the rig shape (0..1)
    int   lock_frames;  // consecutive all-4-real frames before trusting model
    float max_stretch;  // reject a similarity fit whose scale moves more than
                        // this factor in one frame (guards a bad association)
};

// Returns the defaults, tuned for a 240x176 sensor at ~135 fps.
QuadConfig quad_default_config(void);

void       quad_reset(const QuadConfig* cfg);   // cfg may be NULL -> defaults
QuadResult quad_update(const float* xs, const float* ys, int n);  // n <= QUAD_MAX_IN

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
    int   hr_valid, h_valid; // homography memory state after the frame
    int   stuck, cbad;       // breaker / give-up clocks after the frame
};
extern QuadDebugHook quad_dbg;
#endif
