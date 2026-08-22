// Blob detector: threshold + run-length connected components + subpixel centroids.
// Pure C++ (no Arduino deps), host-testable. The streaming API is a single
// non-reentrant instance fed incrementally as camera DMA chunks land.
#pragma once
#include <cstdint>
#include <cstddef>

// One detected blob: centroid, size and saturation metrics.
struct Blob {
    float cx, cy;        // intensity-weighted centroid, subpixel px
    uint32_t pixels;     // area, px
    uint64_t mass;       // sum of intensities
    uint8_t  peak;       // max raw pixel value in the blob (255 = clipped)
    uint32_t sat;        // pixels at >=254 (how flat-topped the blob is)
    int16_t  bx0, bx1;   // bounding box, inclusive [bx0..bx1] x [by0..by1]
    int16_t  by0, by1;   // pixels/(w*h) = fill factor; a round dot is ~0.78
};

// Max blobs reported per frame.
constexpr int BLOB_MAX_OUT = 8;

// The blobs found in one frame.
struct BlobResult {
    Blob blobs[BLOB_MAX_OUT];  // brightest blobs, sorted by mass desc
    int count;                 // how many valid (0..BLOB_MAX_OUT)
};

// ---- streaming API ----
// Starts a frame. px_budget = max bright pixels per frame, 0 = unlimited;
// exceeding it aborts the scan and finish() then reports 0 blobs.
void blobstream_begin(int w, uint8_t threshold, uint32_t min_px, uint32_t max_px,
                      uint32_t px_budget = 0);
// base: frame buffer start; bytes_total: bytes valid so far (monotonic within a frame)
void blobstream_feed(const uint8_t* base, size_t bytes_total);
// Ends the frame and returns its blobs.
BlobResult blobstream_finish();
bool blobstream_flooded();   // true if the LAST finished frame blew its px budget

// ---- one-shot wrapper: TEST ONLY ----------------
// Scans a whole frame in one call. Not called by the firmware.
BlobResult detect_blobs(const uint8_t* frame, int w, int h,
                        uint8_t threshold, uint32_t min_px, uint32_t max_px);
