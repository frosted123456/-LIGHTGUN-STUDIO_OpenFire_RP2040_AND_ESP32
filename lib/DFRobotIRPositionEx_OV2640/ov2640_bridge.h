// ov2640_bridge.h — the ONLY contract between the capture stack and the shim.
//
// The capture side (esp32-camera fork + blob detector, running in the driver's
// copy-task) publishes the latest frame's blobs here; the shim consumes them
// from OpenFIRE's loop task.
//
// Concurrency: single-writer / single-reader, lock-free, never blocks the
// writer; the reader retries if a write overlapped its copy.
#pragma once
#include <stdint.h>

#define OV2640_BRIDGE_MAX_PTS 4

// One published frame: up to four blobs in native camera px, fixed-point x16.
typedef struct {
    // coordinates in the CAMERA's native pixel grid, fixed-point x16
    // (240x176 sensor -> 0..3839 / 0..2815). The shim decides the output
    // scaling; the bridge stays in native units.
    //
    // SIGNED, and that is load-bearing: a reconstructed corner outside the
    // sensor frame is legitimate geometry (aim at a screen corner and the far
    // LED leaves the field of view), and clamping it deforms the quad.
    // int16_t covers +/-2048 native px.
    int16_t x16[OV2640_BRIDGE_MAX_PTS];
    int16_t y16[OV2640_BRIDGE_MAX_PTS];
    uint8_t  area4[OV2640_BRIDGE_MAX_PTS]; // blob area >> 4, clamped 0..255
                                           // (shim clamps to DFRobot's 0..15)
    uint8_t  count;        // 0..4 valid points, brightest-first (by mass)
    uint8_t  frame_w_log;  // reserved
    uint16_t frame_w;      // native frame size these coords live in
    uint16_t frame_h;
    uint32_t frame_seq;    // increments once per finished frame
} ov2640_bridge_frame_t;

#ifdef __cplusplus
extern "C" {
#endif

// ---- writer side (capture stack) ----
// Call once per finished frame from the capture/copy task. Copies `f` into the
// published slot. Cheap (~tens of ns).
void ov2640_bridge_publish(const ov2640_bridge_frame_t* f);

// ---- reader side (shim) ----
// Copies the latest published frame into `out`. Returns the frame_seq, or 0 if
// nothing was ever published. Lock-free; retries internally on torn reads.
uint32_t ov2640_bridge_read(ov2640_bridge_frame_t* out);

#ifdef __cplusplus
}
#endif
