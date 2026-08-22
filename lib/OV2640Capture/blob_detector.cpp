// Blob detector implementation: run-length extraction, union-find labelling
// and intensity-weighted centroids, streamed row by row.
#include "blob_detector.h"
#include <cstring>

// Place the hot scan in IRAM on ESP32 so flash-cache misses never stall it.
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  #include "esp_attr.h"
  #define BLOB_IRAM IRAM_ATTR
#else
  #define BLOB_IRAM
#endif

namespace {

constexpr int MAX_RUNS_PER_ROW = 64;   // bright runs kept per row
constexpr int MAX_LABELS       = 256;  // distinct components per frame

// A horizontal run of bright pixels and the label it belongs to.
struct Run { int16_t xs, xe; int16_t label; };

// Per-label accumulator for centroid, area and saturation.
struct Acc {
    uint64_t sx = 0, sy = 0, mass = 0;  // intensity-weighted sums
    uint32_t px = 0;
    uint32_t sat = 0;                   // pixels at >=254 (clip evidence)
    uint8_t  peak = 0;                  // max raw value seen
    int16_t  x0 = INT16_MAX, x1 = -1;   // bbox, used for the fill-factor
    int16_t  y0 = INT16_MAX, y1 = -1;   // shape gate
};

// ---- streaming state (single instance; driver copy-task context on ESP32) ----
Run s_prev[MAX_RUNS_PER_ROW], s_cur[MAX_RUNS_PER_ROW];
int16_t s_parent[MAX_LABELS];
Acc s_acc[MAX_LABELS];
int s_n_prev = 0, s_n_labels = 0;
int s_w = 0, s_rows_done = 0;
uint8_t s_thr = 128;
uint32_t s_min_px = 2, s_max_px = 4000;
uint32_t s_px_budget = 0;        // 0 = unlimited (host/one-shot path)
uint32_t s_px_seen = 0;          // bright pixels accumulated this frame
bool s_flooded = false;          // this frame blew the budget -> scanning aborted
bool s_flood_latch = false;      // flood state of the last FINISHED frame

// Returns the representative label of a set, with path compression.
int find_root(int16_t* parent, int i) {
    while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
    return i;
}

// Merges two label sets.
void unite(int16_t* parent, int a, int b) {
    a = find_root(parent, a); b = find_root(parent, b);
    if (a != b) parent[b > a ? b : a] = (b > a ? a : b);
}

// Word-at-a-time "any byte >= T" test used to skip dark pixels four at a time.
// Exact for T <= 128; for T > 128 it is a superset filter (bytes in [128, T)
// pass) and the byte-wise loop re-checks.
inline bool word_has_candidate(uint32_t word, uint8_t threshold, uint32_t swar_add) {
    if (threshold <= 128) {
        uint32_t x = word & 0x7F7F7F7Fu;
        return (((x + swar_add) | word) & 0x80808080u) != 0;
    }
    return (word & 0x80808080u) != 0;
}

// Extracts one row's bright runs, links them to the previous row and
// accumulates their pixels into the matching labels.
BLOB_IRAM void process_row(const uint8_t* row, int y)
{
    const int w = s_w;
    const uint8_t threshold = s_thr;
    int n_cur = 0;

    // --- extract runs of bright pixels in this row ---
    const uint32_t swar_add = 0x80808080u - 0x01010101u * (uint32_t)threshold;
    int x = 0;
    while (x < w) {
        // fast path: skip 4 dark pixels at a time; requires 4-byte aligned rows
        // (frame is DMA-allocated and all supported widths are multiples of 4)
        if (((x & 3) == 0) && x + 4 <= w) {
            uint32_t word;
            memcpy(&word, row + x, 4);
            if (!word_has_candidate(word, threshold, swar_add)) { x += 4; continue; }
        }
        if (row[x] >= threshold) {
            int xs = x;
            while (x < w && row[x] >= threshold) ++x;
            // flood guard: charge every bright pixel walked, including runs the
            // per-row cap drops
            if (s_px_budget) {
                s_px_seen += (uint32_t)(x - xs);
                if (s_px_seen > s_px_budget) { s_flooded = true; return; }
            }
            if (n_cur < MAX_RUNS_PER_ROW) {
                s_cur[n_cur++] = { (int16_t)xs, (int16_t)(x - 1), -1 };
            }
        } else ++x;
    }

    // --- connect to previous row (8-connectivity), assign labels ---
    for (int i = 0; i < n_cur; ++i) {
        for (int j = 0; j < s_n_prev; ++j) {
            if (s_prev[j].label < 0) continue;   // dropped run: never pass -1 to unite()
            if (s_prev[j].xe + 1 >= s_cur[i].xs &&
                s_prev[j].xs - 1 <= s_cur[i].xe) {
                if (s_cur[i].label < 0)
                    s_cur[i].label = s_prev[j].label;
                else
                    unite(s_parent, s_cur[i].label, s_prev[j].label);
            }
        }
        if (s_cur[i].label < 0) {
            if (s_n_labels < MAX_LABELS) {
                s_parent[s_n_labels] = (int16_t)s_n_labels;
                s_cur[i].label = (int16_t)s_n_labels++;
            } else {
                // label pool exhausted: treat like a flood so the caller is told
                // the frame is unusable rather than handed a merged wrong blob
                s_flooded = true;
                return;
            }
        }
        // --- accumulate this run into its label ---
        // Weight is threshold-relative, (v - threshold + 1), so the noise floor
        // cannot bias the centroid.
        Acc& a = s_acc[find_root(s_parent, s_cur[i].label)];
        for (int px = s_cur[i].xs; px <= s_cur[i].xe; ++px) {
            uint8_t raw = row[px];
            uint32_t v = (uint32_t)raw - threshold + 1;
            a.sx += (uint64_t)v * px;
            a.sy += (uint64_t)v * y;
            a.mass += v;
            a.px  += 1;
            if (raw > a.peak) a.peak = raw;
            if (raw >= 254)   a.sat++;
        }
        // bbox from run extents, per-run rather than per-pixel
        if (s_cur[i].xs < a.x0) a.x0 = s_cur[i].xs;
        if (s_cur[i].xe > a.x1) a.x1 = s_cur[i].xe;
        if (y < a.y0) a.y0 = (int16_t)y;
        if (y > a.y1) a.y1 = (int16_t)y;
    }
    memcpy(s_prev, s_cur, sizeof(Run) * n_cur);
    s_n_prev = n_cur;
}

} // namespace

// Arms the streaming detector for a new frame.
void blobstream_begin(int w, uint8_t threshold, uint32_t min_px, uint32_t max_px,
                      uint32_t px_budget)
{
    s_w = w; s_thr = threshold; s_min_px = min_px; s_max_px = max_px;
    s_px_budget = px_budget; s_px_seen = 0; s_flooded = false;
    s_rows_done = 0; s_n_prev = 0; s_n_labels = 0;
    for (int i = 0; i < MAX_LABELS; ++i) s_acc[i] = Acc{};
}

// Processes whatever rows are now complete in the frame buffer.
BLOB_IRAM void blobstream_feed(const uint8_t* base, size_t bytes_total)
{
    if (s_w <= 0) return;
    if (s_flooded) return;   // flood guard tripped: rest of frame costs nothing
    int rows_avail = (int)(bytes_total / (size_t)s_w);
    while (s_rows_done < rows_avail) {
        process_row(base + (size_t)s_rows_done * s_w, s_rows_done);
        s_rows_done++;
        if (s_flooded) return;
    }
}

// True if the last finished frame exceeded its pixel budget.
bool blobstream_flooded() { return s_flood_latch; }

// Closes the frame: merges labels and returns the brightest blobs.
BlobResult blobstream_finish()
{
    s_flood_latch = s_flooded;
    if (s_flooded) {
        // flooded frame: report empty rather than merge hundreds of noise labels
        BlobResult empty{}; empty.count = 0;
        s_w = 0;
        return empty;
    }
    // --- merge accumulators up to roots ---
    for (int i = s_n_labels - 1; i >= 0; --i) {
        int r = find_root(s_parent, i);
        if (r != i) {
            s_acc[r].sx += s_acc[i].sx; s_acc[r].sy += s_acc[i].sy;
            s_acc[r].mass += s_acc[i].mass; s_acc[r].px += s_acc[i].px;
            s_acc[r].sat += s_acc[i].sat;
            if (s_acc[i].peak > s_acc[r].peak) s_acc[r].peak = s_acc[i].peak;
            if (s_acc[i].x0 < s_acc[r].x0) s_acc[r].x0 = s_acc[i].x0;
            if (s_acc[i].x1 > s_acc[r].x1) s_acc[r].x1 = s_acc[i].x1;
            if (s_acc[i].y0 < s_acc[r].y0) s_acc[r].y0 = s_acc[i].y0;
            if (s_acc[i].y1 > s_acc[r].y1) s_acc[r].y1 = s_acc[i].y1;
            s_acc[i].px = 0;
        }
    }

    // --- pick top-BLOB_MAX_OUT by mass, size-filtered ---
    BlobResult out{}; out.count = 0;
    for (int i = 0; i < s_n_labels; ++i) {
        if (s_acc[i].px < s_min_px || s_acc[i].px > s_max_px || s_acc[i].mass == 0) continue;
        Blob b;
        b.cx = (float)((double)s_acc[i].sx / (double)s_acc[i].mass);
        b.cy = (float)((double)s_acc[i].sy / (double)s_acc[i].mass);
        b.pixels = s_acc[i].px;
        b.mass = s_acc[i].mass;
        b.peak = s_acc[i].peak;
        b.sat  = s_acc[i].sat;
        b.bx0 = s_acc[i].x0; b.bx1 = s_acc[i].x1;
        b.by0 = s_acc[i].y0; b.by1 = s_acc[i].y1;
        int pos = out.count < BLOB_MAX_OUT ? out.count : BLOB_MAX_OUT;
        for (int k = 0; k < out.count && k < BLOB_MAX_OUT; ++k) {
            if (b.mass > out.blobs[k].mass) { pos = k; break; }
        }
        if (pos < BLOB_MAX_OUT) {
            for (int k = (out.count < BLOB_MAX_OUT ? out.count : BLOB_MAX_OUT - 1); k > pos; --k)
                out.blobs[k] = out.blobs[k - 1];
            out.blobs[pos] = b;
            if (out.count < BLOB_MAX_OUT) out.count++;
        }
    }
    s_w = 0;   // stream consumed; next begin() re-arms
    return out;
}

// Test-only one-shot wrapper; referenced by test/, dropped from firmware by
// --gc-sections.
BlobResult detect_blobs(const uint8_t* frame, int w, int h,
                        uint8_t threshold, uint32_t min_px, uint32_t max_px)
{
    blobstream_begin(w, threshold, min_px, max_px);
    blobstream_feed(frame, (size_t)w * (size_t)h);
    return blobstream_finish();
}
