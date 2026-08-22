// ov2640_capture.cpp — camera init, in-driver blob detection, quad resolution
// and publication into ov2640_bridge. Ships a fixed, measured-good boot recipe
// for this OV2640 module; see the header for the public API.
#include "ov2640_capture.h"
#ifdef USE_AIM_PIPELINE
#include "aim_runtime.h"
#endif

// OV_TELEMETRY: compile in the Q/STAT telemetry stream and the tune parser.
// Both the diagnostic build and the aim pipeline (whose calibration app needs
// them on a shipping build) pull them in. They stay OFF at runtime until asked,
// so the cost when unused is a few KB of flash and nothing on the hot path.
#if LIGHTGUN_DIAG || defined(USE_AIM_PIPELINE)
  #define OV_TELEMETRY 1
#else
  #define OV_TELEMETRY 0
#endif

// Telemetry line sink; printf() (UART0) when none is installed.
static void (*s_line_sink)(const char*) = 0;
extern "C" void ov2640_set_line_sink(void (*fn)(const char*)) { s_line_sink = fn; }

// Separate sink for command REPLIES: long and rare, so they need the chunked
// policy, while per-frame telemetry must never wait.
static void (*s_reply_sink)(const char*) = 0;
extern "C" void ov2640_set_reply_sink(void (*fn)(const char*)) { s_reply_sink = fn; }

#include <stdarg.h>
#include <stdio.h>
// Emits one command reply, to the reply sink if installed, else printf.
static void ov_reply(const char* fmt, ...)
{
    char b[256];
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (s_reply_sink) s_reply_sink(b);
    else              fputs(b, stdout);
}

// Emits one telemetry line, to the line sink if installed, else printf.
static void ov_emit(const char* fmt, ...)
{
    char b[192];
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    // the sink must be non-blocking: telemetry is expendable, the aim path is not
    if (s_line_sink) s_line_sink(b);
    else             fputs(b, stdout);
}
#include "ov2640_bridge.h"
#include "blob_detector.h"
#include "quad_resolver.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"      // MEM line (PSRAM presence + DRAM headroom)

// Freenove S3-WROOM CAM pin map (matches firmware/src/board_esp32s3.h)
#define P_XCLK 15
#define P_SIOD 4
#define P_SIOC 5
#define P_D7 16
#define P_D6 17
#define P_D5 18
#define P_D4 12
#define P_D3 10
#define P_D2 8
#define P_D1 9
#define P_D0 11
#define P_VSYNC 6
#define P_HREF 7
#define P_PCLK 13

// LIGHTGUN_DIAG=1 compiles the instrumentation: the per-second telemetry block,
// the dashboard stream and the UART0 tune console. 0 (the shipping default)
// removes all of it at compile time. Deliberately NOT behind the flag:
// drain_task itself (its fb get/return IS the framebuffer drain, so capture
// stops without it), the one-line boot banner, and the ISR counters in
// cam_hal.c.
#ifndef LIGHTGUN_DIAG
#define LIGHTGUN_DIAG 0
#endif

static const int   FRAME_W = 240, FRAME_H = 176;

// ---- boot recipe: the measured-good configuration -------------------------
// Change these four, and nothing else, to retune. boost trades amplified gain
// for real exposure, which keeps the noise floor low enough for a low
// threshold; raising boost again would need thr raised with it.
static const int BOOT_THR   = 80;   // pixel threshold (runtime: "thr=N")
static const int BOOT_AEC   = 40;   // exposure in lines (runtime: "aec=N")
static const int BOOT_AGC   = 2;    // PGA index, 0..30 (runtime: "agc=N")
static const int BOOT_BOOST = 0;    // REG45[7:6] x4 stages (runtime: "boost=N")

static uint8_t  THR = (uint8_t)BOOT_THR;   // RUNTIME-TUNABLE via ov2640_tune()
static const uint32_t MIN_PX = 4;   // min blob size, px
// max blob size in px, and the per-frame bright-pixel flood budget
static const uint32_t MAX_PX = (uint32_t)(240 * 176 / 4), PX_BUDGET = 8000;

extern "C" {
    // exported by our patched cam_hal.c
    typedef void (*cam_chunk_cb_t)(const uint8_t* base, size_t len, bool frame_end);
    extern cam_chunk_cb_t cam_patch_chunk_cb;
    // VSYNC period measured in the ISR, immune to the cam_task scheduling
    // jitter that pollutes a task-context measurement
    extern volatile uint32_t cam_patch_vsync_isr_period_us;
    // Driver-side vertical-alignment instrumentation. cam_patch_restart_us is
    // the worst time between the real VSYNC and cam_start_frame(); capture
    // begins wherever the sensor currently is, so every microsecond of that
    // delay is a VERTICAL OFFSET (~25us per line at 136fps).
    extern volatile uint32_t cam_patch_stitch_rej;
    extern volatile uint32_t cam_patch_restart_us;
    extern volatile uint32_t cam_patch_restart_us_last;   // most recent sample
    extern volatile uint32_t cam_patch_nostart;
    extern volatile uint32_t cam_patch_vs_long;
    extern volatile uint32_t cam_patch_vs_short;
    extern volatile uint32_t cam_patch_chunk_rej;        // chunk count != expected
    extern volatile uint32_t cam_patch_chunks_expected;  // expected chunks per frame
    // per-second fault attribution, matching the lab's own STAT line
    extern volatile uint32_t cam_patch_ovf_vsync;
    extern volatile uint32_t cam_patch_ovf_eof;
    extern volatile uint32_t cam_patch_vs_min_us;
    extern volatile uint32_t cam_patch_vs_max_us;
    extern volatile uint32_t cam_patch_vs_count;
    extern volatile uint64_t cam_patch_vs_sum_us;
    extern volatile uint32_t cam_patch_vs_ref_us;
    // 0 = the two overlay-only frame gates are OBSERVE-ONLY (the default);
    // 1 = they reject framebuffers
    extern volatile uint32_t cam_patch_gate_en;
    extern volatile uint32_t cam_patch_gap_max_us;    // starvation meter, us
    extern volatile uint32_t cam_patch_gap_over1ms;
    extern volatile uint32_t cam_patch_gap_over3ms;
    extern volatile uint32_t cam_patch_vs_total;          // free-running
    extern volatile uint32_t cam_patch_frames_delivered;
    extern volatile uint32_t cam_patch_frames_rejected;
    extern volatile uint32_t cam_patch_rej_size;      // rejection attribution
    extern volatile uint32_t cam_patch_rej_queue;
    // DMA ring geometry, filled in at cam_config. This is the overrun budget:
    // dma_halfs * lines_per_half * t_LINE is how long cam_task may be blocked
    // before the DMA laps the ring and the frame is spliced.
    extern volatile uint32_t cam_patch_evq_depth;
    extern volatile uint32_t cam_patch_dma_halfs;
    extern volatile uint32_t cam_patch_dma_lines;
    // chunks-per-frame histogram: short = restart-late or a masked/coalesced
    // EOF interrupt, long = the frame ran past its VSYNC (a genuine stitch)
    extern volatile uint32_t cam_patch_cnt_hist[16];
    // ISR-grade VSYNC period and raw ISR event counts: splits "the interrupt is
    // late" from "the event is dequeued out of order"
    extern volatile uint32_t cam_patch_vsi_min_us;
    extern volatile uint32_t cam_patch_vsi_max_us;
    extern volatile uint32_t cam_patch_vsi_cnt;
    extern volatile uint64_t cam_patch_vsi_sum_us;
    extern volatile uint32_t cam_patch_eof_isr_total;
    extern volatile uint32_t cam_patch_vs_isr_total;
    extern volatile uint32_t cam_patch_eof_drained;   // EOFs drained
}

static volatile bool s_started = false;
static sensor_t* s_sensor = nullptr;   // kept for live tuning
static uint32_t s_frame_seq = 0;

// ---- frame gate: one rule left --------------------------------------------
// Publish only if the byte count is EXACTLY one full frame. A short frame is
// missing lines because capture restarted late, so every y carries an unknown
// offset; it is counted in ov2640_stat_rej_size and nothing is published, so
// the bridge keeps last-good.
#if OV_TELEMETRY
// frame period / total lines: CIF is 336 lines, so 7353/336 = 21.9us at 136fps
static const float T_LINE_US = 21.9f;
#endif

// the one surviving frame-gate statistic
volatile uint32_t ov2640_stat_rej_size = 0;

// In-situ debug stream (D-lines): exactly what the shim hands OpenFIRE, in its
// 0..1023 / 0..767 space, decimated to ~10Hz. Enable with "dbg=1".
#if OV_TELEMETRY
// Default depends on the build: a DIAG build has no console to switch this on,
// while the shipping build has the '~' channel and must not pay for a printf
// here (~1.8ms of busy-wait) that it never asked for.
#if LIGHTGUN_DIAG
static volatile bool s_dbg = true;
#else
static volatile bool s_dbg = false;
#endif

// ghost / jump meter accumulators (see the jump meter block in on_chunk)
static volatile uint32_t s_ghost_over4 = 0;   // frames where the detector saw >4
static volatile uint32_t s_jump_pts   = 0;   // points that moved >=8 lines
static volatile uint32_t s_jump_q16   = 0;   // ...of which, on the 16-line grid
static volatile float    s_jump_worst = 0;   // worst single move, lines
static volatile uint8_t  s_ghost_max   = 0;   // most blobs seen in one frame

// ---- dashboard link -------------------------------------------------------
// One fix line per published frame plus a STAT line per second, in the format
// tools/dashboard.py parses. The cost is TIME rather than bandwidth: printf()
// to UART0 busy-waits ~1.8ms per line, so the stream is off by default and
// rate-limitable.
//   dash=0 off, 1 = B-lines (one point, lab wire format), 2 = Q-lines (all
//   four, ~50 bytes, so mode 2 starts rate-capped; dashhz=0 lifts it)
//   dashb=0..3 which point the B-line carries; dashhz=N caps the rate
// SNAP is unsupported: the overlay never keeps a framebuffer copy.
static volatile uint8_t  s_dash = 0;
static volatile uint8_t  s_dash_blob = 0;
static volatile uint32_t s_dash_min_dt_us = 0;    // 0 = publish every frame
#endif  // OV_TELEMETRY (diagnostic state)
// Shadowed because the sensor's own registers are write-only through this API.
static int s_cfg_aec = BOOT_AEC, s_cfg_agc = BOOT_AGC, s_cfg_boost = BOOT_BOOST;

// UART0 ownership. printf() always lands on UART0 in both builds, so the
// dashboard can always read us; draining its RX depends on the build. In the
// harness, Serial IS UART0 and the sketch already forwards lines to
// ov2640_tune(); in the combined build Serial is the USB CDC that OpenFIRE
// owns, leaving UART0 free for us. -D OV_DASH_NO_UART0_RX opts out. The tune
// console is diagnostic-only: the shipping build installs no UART driver.
#if LIGHTGUN_DIAG && defined(ARDUINO_USB_CDC_ON_BOOT) && \
    (ARDUINO_USB_CDC_ON_BOOT == 1) && !defined(OV_DASH_NO_UART0_RX)
  #define OV_DASH_OWN_UART0 1
  #include "driver/uart.h"
#else
  #define OV_DASH_OWN_UART0 0
#endif

// Candidates the filter considers on the legacy path, matching the lab's own
// BLOB_MAX_OUT: feeding it more lets a dropped duplicate backfill from the dim
// tail, and the consumer then locks onto that junk.
static const int LAB_CANDIDATES = 4;         // == the lab's BLOB_MAX_OUT

// ---- temporal coincidence gate --------------------------------------------
// Require a blob in 2 consecutive frames before accepting it, which kills
// one-frame junk and delays only ACQUISITION, by one frame. The history holds
// the CANDIDATES, not the survivors: holding survivors would mean a genuinely
// new emitter could never get in.
static const float COIN_R2 = 12.0f * 12.0f;   // px^2; above centroid noise, below LED spacing
static volatile bool     s_coin = true;       // "coin=0" disables for A/B
static volatile uint32_t s_coin_killed = 0;   // blobs dropped, per-second delta

// ---- publish-boundary audit ------------------------------------------------
// What predicts the consumer's behaviour is the published COUNT and its CHURN,
// not the map: every count<4 frame makes OpenFIRE invent the missing corners,
// and every change of count fires its kinematic spring, which then bleeds the
// error off over ~70 frames. pub[i] = frames published with count i; churn =
// count differing from the previous published frame.
// ---- quad resolver ---------------------------------------------------------
// Persistent corner identity plus rigid reconstruction (quad_resolver.*). It
// answers a dropout by reconstructing the corner and keeping the count at 4,
// where the coincidence gate answered it by removing a point.
// 0 = off (legacy coincidence gate), 1 = resolver fed the top-4 by mass,
// 2 = resolver fed up to QUAD_MAX_IN blobs and choosing four by geometry.
static volatile uint8_t s_resolver = 2;

// ---- lens correction: undistort centroids before anything geometric --------
// The resolver's plausibility logic and OpenFIRE's solver both assume a pinhole
// projection of the rig, so a wide or fisheye lens is corrected here first.
//   model 0: none (default)
//   model 1: polynomial barrel r_d = r_u(1 + k1 r_u^2 + k2 r_u^4), radius
//            normalised by FPX, inverted with 3 Newton steps
//   model 2: fisheye (equidistant) r_px = FEQ * theta, output mapped back to
//            a pinhole of focal FPX
// SHIP: the LIGHTGUN_LENS_* defines. DIAG: live keys lens= lk1u= lk2u= lfpx=
// lfeq= (k in micro units, f in tenths of a px; negative values accepted).
#ifndef LIGHTGUN_LENS_MODEL
#define LIGHTGUN_LENS_MODEL 0
#endif
#ifndef LIGHTGUN_LENS_K1
#define LIGHTGUN_LENS_K1 0.0f
#endif
#ifndef LIGHTGUN_LENS_K2
#define LIGHTGUN_LENS_K2 0.0f
#endif
#ifndef LIGHTGUN_LENS_FPX
#define LIGHTGUN_LENS_FPX 184.7f     // pinhole focal, px (66deg over 240px)
#endif
#ifndef LIGHTGUN_LENS_FEQ
#define LIGHTGUN_LENS_FEQ 90.0f      // fisheye equidistant focal, px
#endif
static volatile uint8_t s_lens  = LIGHTGUN_LENS_MODEL;
static float s_lk1  = LIGHTGUN_LENS_K1;
static float s_lk2  = LIGHTGUN_LENS_K2;
static float s_lfpx = LIGHTGUN_LENS_FPX;
static float s_lfeq = LIGHTGUN_LENS_FEQ;

// ---- latency lead ----------------------------------------------------------
// Publish the quad extrapolated forward along its own measured rigid velocity
// by a fixed lead time. This does NOT touch the resolver's state (see
// QuadResult::vx): prediction fed back into a tracker biases its association.
// Only our own ~8ms share of the chain is safely compensable; leading further
// overshoots on direction reversals, hence the hard ceiling.
//   SHIP: -D LIGHTGUN_LEAD_MS=10      DIAG: live key  lead=10   (0 = off)
#ifndef LIGHTGUN_LEAD_MS
#define LIGHTGUN_LEAD_MS 0           // opt-in: 0 = no lead at all
#endif
#define LIGHTGUN_LEAD_MS_MAX  50.0f  // beyond this the overshoot is worse than the lag
#define LIGHTGUN_LEAD_PX_MAX  40.0f  // a velocity glitch must not fling the quad
#define LIGHTGUN_FPS_NOMINAL  135.0f // set by xclk=27MHz / pdiv=3, not measured
static volatile float s_lead_ms = (float)LIGHTGUN_LEAD_MS;

// ---- vfill: LED vertical span vs screen height -----------------------------
// OpenFIRE warps the LED quad onto a destination rectangle spanning the full
// screen height -- it ASSERTS that the LEDs sit on the top and bottom edges,
// and derives the horizontal pair from the measured aspect. If they span only
// a fraction of the screen height, every mapped position is scaled about the
// screen centre, and the 4-point calibration can cancel that only to first
// order. This knob pre-scales the published quad about its own centroid by
// 100/vfill, which makes the assumption true.
//   vfill = (vertical distance between the LED bars) / (screen height) x 100
// It is routinely ABOVE 100, because the standard square layout puts one bar
// above the display and one below it, bracketing the screen. Horizontal
// spacing genuinely does not matter (it is derived from the measured aspect).
// LIMIT: pre-scaling cannot change the k_x/k_y ratio, so an aspect mismatch
// between the LED rectangle and the screen stays out of reach.
//   SHIP: -D LIGHTGUN_VFILL_PCT=136   DIAG: live key  vfill=136  (100 = off)
// RECALIBRATE in OpenFIRE after changing this -- it moves the geometry the
// stored profile was measured against.
#ifndef LIGHTGUN_VFILL_PCT
#define LIGHTGUN_VFILL_PCT 100       // opt-in: 100 = no scaling at all
#endif
static volatile float s_vfill_pct = (float)LIGHTGUN_VFILL_PCT;

// Maps a distorted centroid back to pinhole coordinates, in place.
static inline void lens_undistort(float* px, float* py)
{
    if (!s_lens) return;
    const float cx = FRAME_W * 0.5f, cy = FRAME_H * 0.5f;
    const float dx = *px - cx, dy = *py - cy;
    const float rd = sqrtf(dx*dx + dy*dy);
    if (rd < 1e-3f) return;
    float k;
    if (s_lens == 2) {
        float th = rd / s_lfeq;
        if (th > 1.45f) th = 1.45f;              // ~83deg half-angle cap
        k = (s_lfpx * tanf(th)) / rd;
    } else {
        const float rdn = rd / s_lfpx;
        float ru = rdn;                           // Newton: invert the forward poly
        for (int i = 0; i < 3; ++i) {
            const float r2 = ru*ru;
            const float f  = ru*(1.0f + s_lk1*r2 + s_lk2*r2*r2) - rdn;
            const float df = 1.0f + 3.0f*s_lk1*r2 + 5.0f*s_lk2*r2*r2;
            if (fabsf(df) < 1e-6f) break;
            ru -= f/df;
        }
        k = ru / rdn;
    }
    *px = cx + dx*k; *py = cy + dy*k;
}
static volatile float s_res_conf = 0.0f;
// End-to-end latency, stamped here at publish; the shim subtracts it the moment
// OpenFIRE consumes the frame. Defined here rather than in the shim header,
// which would be a duplicate symbol; declared extern in DFRobotIRPositionEx.h.
volatile uint32_t ov2640_pub_t_us       = 0;
volatile uint32_t ov2640_shim_lat_us    = 0;   // worst since read, written by the shim
volatile uint32_t ov2640_shim_lat_last_us = 0; // most recent sample
static volatile uint32_t s_pub_hist[5] = {0,0,0,0,0};
static volatile uint32_t s_pub_churn = 0;

// The one place native pixels become wire coordinates: signed 1/16 px with no
// border clamp, because a reconstructed corner outside the sensor is real
// geometry that OpenFIRE's solve needs. Only the int16 range limits it.
static inline int16_t enc16(float v)
{
    float u = v * 16.0f + (v >= 0.0f ? 0.5f : -0.5f);
    if (!(u > -32000.0f)) u = -32000.0f;      // also catches NaN
    if (u > 32000.0f)     u =  32000.0f;
    return (int16_t)u;
}

// Keeps only the blobs that were also present in the previous frame.
static BlobResult coincidence_gate(const BlobResult& in)
{
    static float pcx[LAB_CANDIDATES], pcy[LAB_CANDIDATES];
    static int   pn = 0;
    BlobResult out{}; out.count = 0;
    for (int i = 0; i < in.count; ++i) {
        bool seen_before = false;
        for (int k = 0; k < pn; ++k) {
            const float dx = in.blobs[i].cx - pcx[k];
            const float dy = in.blobs[i].cy - pcy[k];
            if (dx * dx + dy * dy <= COIN_R2) { seen_before = true; break; }
        }
        if (seen_before) out.blobs[out.count++] = in.blobs[i];
        else             s_coin_killed++;
    }
    pn = in.count < LAB_CANDIDATES ? in.count : LAB_CANDIDATES;   // candidates
    for (int k = 0; k < pn; ++k) { pcx[k] = in.blobs[k].cx; pcy[k] = in.blobs[k].cy; }
    return out;
}

// Filters detections: keep the top `cand_cap` by mass, drop duplicates within
// 4px, and drop smear stripes (a third blob sharing a column with two others,
// which is what a DMA-spliced twin looks like). The last two are load-bearing,
// so res=2 asks for wider caps rather than bypassing the filter.
static BlobResult lab_filter_blobs(const BlobResult& in,
                                   int cand_cap = LAB_CANDIDATES, int out_cap = 4)
{
    BlobResult out{}; out.count = 0;
    if (out_cap > BLOB_MAX_OUT) out_cap = BLOB_MAX_OUT;
    if (out_cap < 1) out_cap = 1;
    const int n = in.count < cand_cap ? in.count : cand_cap;
    for (int i = 0; i < n; ++i) {
        const Blob& b = in.blobs[i];
        bool drop = false;
        int col_mates = 0;
        for (int k = 0; k < out.count; ++k) {
            float dx = b.cx - out.blobs[k].cx;
            float dy = b.cy - out.blobs[k].cy;
            if (dx * dx + dy * dy < 16.f) { drop = true; break; }   // duplicate
            if (dx < 3.5f && dx > -3.5f) col_mates++;               // same column
        }
        if (!drop && col_mates >= 2) drop = true;                   // smear stripe
        if (!drop) {
            out.blobs[out.count++] = b;
            if (out.count == out_cap) break;
        }
    }
    return out;
}

// Handle of whatever task actually runs the capture path, captured from inside
// it so the stack-headroom telemetry measures the right task's stack.
static volatile TaskHandle_t s_cam_task = nullptr;

// Chunk callback, run in the DRIVER's copy-task: feeds the blob stream and, at
// frame end, filters, resolves and publishes the frame.
static void on_chunk(const uint8_t* base, size_t len, bool frame_end)
{
    if (!s_cam_task) s_cam_task = xTaskGetCurrentTaskHandle();
    static size_t prev_len = 0;
    static bool began = false;
    if (len == 0) {                                  // explicit frame start
        blobstream_begin(FRAME_W, THR, MIN_PX, MAX_PX, PX_BUDGET);
        began = true; prev_len = 0;
        return;
    }
    if (!began || len < prev_len) {
        blobstream_begin(FRAME_W, THR, MIN_PX, MAX_PX, PX_BUDGET);
        began = true;
    }
    blobstream_feed(base, len);
    prev_len = len;
    if (frame_end) {
        // Detector -> filter -> publish, with the one gate the lab also has: a
        // short frame is missing lines because capture restarted late, so every
        // y carries an unknown offset. Keep last-good instead of publishing it.
        // (The driver deliberately delivers frames >=90% complete rather than
        // dropping them, so these are common.)
        const size_t lab_full = (size_t)FRAME_W * (size_t)FRAME_H;
        if (len != lab_full) {
            ov2640_stat_rej_size++;              // counted, never published
            began = false; prev_len = 0;
            return;                              // keep last-good, like the lab
        }
        static BlobResult lr;
        static BlobResult raw_lr;
        raw_lr = blobstream_finish();
#if LIGHTGUN_DIAG
        // ---- jump meter --------------------------------------------------
        // Measures CHANGE BETWEEN FRAMES, which is zero for any rig held still
        // whatever its geometry. A point is matched to the previous frame by x
        // (points keep their column; a splice displaces y), and a move of >=8
        // lines while the gun is still cannot be optics or noise. onGrid16
        // counts those landing within 2 lines of a multiple of 16, the DMA
        // half-buffer grid: that is the splice fingerprint.
        {
            static float pvx[8], pvy[8]; static int pvn = 0;
            for (int a = 0; a < raw_lr.count && a < 8; ++a) {
                float best = 1e9f; int bi = -1;
                for (int b = 0; b < pvn; ++b) {
                    const float d = raw_lr.blobs[a].cx - pvx[b];
                    const float ad = d < 0 ? -d : d;
                    if (ad < best) { best = ad; bi = b; }
                }
                if (bi >= 0 && best < 4.0f) {
                    float dy = raw_lr.blobs[a].cy - pvy[bi];
                    if (dy < 0) dy = -dy;
                    if (dy >= 8.0f) {
                        s_jump_pts++;
                        if (dy > s_jump_worst) s_jump_worst = dy;
                        const float q = dy / 16.0f;
                        float r = q - (float)(int)(q + 0.5f);
                        if (r < 0) r = -r;
                        if (r * 16.0f <= 2.0f) s_jump_q16++;
                    }
                }
            }
            pvn = raw_lr.count < 8 ? raw_lr.count : 8;
            for (int a = 0; a < pvn; ++a) { pvx[a] = raw_lr.blobs[a].cx; pvy[a] = raw_lr.blobs[a].cy; }
            if (raw_lr.count > 4) s_ghost_over4++;      // still valid: >4 blobs from 4 emitters
            if (raw_lr.count > s_ghost_max) s_ghost_max = (uint8_t)raw_lr.count;
        }
#endif  // LIGHTGUN_DIAG (jump meter)
        // res=2 wants the SAME filter with WIDER caps, not no filter
        lr = (s_resolver >= 2) ? lab_filter_blobs(raw_lr, BLOB_MAX_OUT, QUAD_MAX_IN)
                               : lab_filter_blobs(raw_lr);
        // undistort ONCE, here, so resolver, bridge, dashboard and OpenFIRE all
        // see pinhole coordinates coherently
        if (s_lens)
            for (int i = 0; i < lr.count; ++i)
                lens_undistort(&lr.blobs[i].cx, &lr.blobs[i].cy);
        ov2640_bridge_frame_t lf = {};
        lf.frame_w = FRAME_W; lf.frame_h = FRAME_H;
        if (s_resolver) {
            // Identity plus rigid reconstruction: emits a stable 4 once locked,
            // so OpenFIRE never sees a count change. res=1 keeps the historical
            // top-4-by-mass trim; res=2 (default) hands the resolver up to
            // QUAD_MAX_IN filtered blobs and lets it CHOOSE the four by
            // geometry, because mass ranking and corner identity disagree -- a
            // bright reflection outranks a dim LED. Note it is still the
            // FILTERED list, just a wider one: handing over raw blobs would undo
            // the duplicate and smear-stripe rejections.
            float qx[QUAD_MAX_IN], qy[QUAD_MAX_IN];
            int qn = lr.count; if (qn > QUAD_MAX_IN) qn = QUAD_MAX_IN;
            for (int i = 0; i < qn; ++i) { qx[i] = lr.blobs[i].cx; qy[i] = lr.blobs[i].cy; }
            const QuadResult q = quad_update(qx, qy, qn);
            s_res_conf = q.confidence;
            // Off-screen honesty. Out-of-frame corners are transmitted
            // faithfully (the bridge field is signed -- see ov2640_bridge.h), so
            // an anchored quad with an extrapolated corner off-sensor is
            // published as-is and OpenFIRE solves it correctly. What must NOT be
            // published is a quad nobody is holding: with no real corner the
            // extrapolation is guesswork, and once it has drifted off-sensor
            // there is no information left in it. Then count=0 -- the DFRobot
            // "nothing seen" idiom, OpenFIRE's native off-screen state -- so the
            // cursor holds instead of being flung into a corner.
            bool q_offscreen = false;
            if (q.count == 4 && q.n_real <= 1) {
                int out_any = 0, out_far = 0;
                for (int i = 0; i < 4; ++i) {
                    const float px = q.p[i].x, py = q.p[i].y;
                    const float dx = px < 0 ? -px : (px > (float)(FRAME_W - 1) ? px - (float)(FRAME_W - 1) : 0.0f);
                    const float dy = py < 0 ? -py : (py > (float)(FRAME_H - 1) ? py - (float)(FRAME_H - 1) : 0.0f);
                    const float d = dx > dy ? dx : dy;
                    if (d > 8.0f)  out_any++;
                    if (d > 24.0f) out_far++;
                }
                q_offscreen = (q.n_real == 0 && out_any > 0) || (out_far >= 2);
            }
            lf.count = q_offscreen ? 0 : (uint8_t)(q.count > 4 ? 4 : q.count);
            // Latency lead: rigid translation only, so the quad's SHAPE is
            // untouched and the solve stays exact. Capped in both time and
            // pixels, and skipped unless the lock is real and at least two
            // corners were measured this frame.
            float lead_x = 0.0f, lead_y = 0.0f;
            if (s_lead_ms > 0.0f && q.locked && q.n_real >= 2) {
                float ms = s_lead_ms;
                if (ms > LIGHTGUN_LEAD_MS_MAX) ms = LIGHTGUN_LEAD_MS_MAX;
                const float frames = ms * (LIGHTGUN_FPS_NOMINAL / 1000.0f);
                lead_x = q.vx * frames;
                lead_y = q.vy * frames;
                const float m = LIGHTGUN_LEAD_PX_MAX;
                if (lead_x >  m) lead_x =  m; else if (lead_x < -m) lead_x = -m;
                if (lead_y >  m) lead_y =  m; else if (lead_y < -m) lead_y = -m;
            }
            // vfill geometry compensation: scale the published quad about its own
            // centroid so OpenFIRE's full-height assumption becomes true. Uniform
            // in both axes, because OpenFIRE derives TLled/TRled from the
            // measured aspect. Applied BEFORE the lead: geometry, then time.
            float vs = 1.0f, cqx = 0.0f, cqy = 0.0f;
            if (lf.count == 4 && s_vfill_pct > 1.0f
                              && (s_vfill_pct < 99.5f || s_vfill_pct > 100.5f)) {
                vs = 100.0f / s_vfill_pct;
                for (int i = 0; i < 4; ++i) { cqx += q.p[i].x; cqy += q.p[i].y; }
                cqx *= 0.25f; cqy *= 0.25f;
            }
            for (int i = 0; i < lf.count; ++i) {
                // faithful, no border clamp -- see ov2640_bridge.h
                const float qx = (vs == 1.0f) ? q.p[i].x : cqx + (q.p[i].x - cqx) * vs;
                const float qy = (vs == 1.0f) ? q.p[i].y : cqy + (q.p[i].y - cqy) * vs;
                lf.x16[i] = enc16(qx + lead_x);
                lf.y16[i] = enc16(qy + lead_y);
                // area doubles as the REAL/RECONSTRUCTED marker for the
                // dashboard: 8 = measured, 4 = synthesised by the resolver.
                // Nothing downstream reads size() on the square path.
                lf.area4[i] = q.p[i].real ? 8 : 4;
            }
            lf.frame_seq = ++s_frame_seq;
            ov2640_pub_t_us = (uint32_t)esp_timer_get_time();
            {
                static uint8_t s_prev_pub_count = 255;
                s_pub_hist[lf.count > 4 ? 4 : lf.count] =
                    s_pub_hist[lf.count > 4 ? 4 : lf.count] + 1;   // not ++: -Wvolatile
                if (s_prev_pub_count != 255 && lf.count != s_prev_pub_count)
                    s_pub_churn = s_pub_churn + 1;        // not ++: -Wvolatile
                s_prev_pub_count = lf.count;
            }
            ov2640_bridge_publish(&lf);
            began = false; prev_len = 0;
            return;
        }
        // legacy path: kill one-frame junk by REMOVING it
        if (s_coin) lr = coincidence_gate(lr);
        lf.count = (uint8_t)(lr.count > 4 ? 4 : lr.count);
        for (int i = 0; i < lf.count; ++i) {
            // raw detections are in-frame by construction; same encoder, so there
            // is exactly one coordinate convention on the wire
            lf.x16[i] = enc16(lr.blobs[i].cx);
            lf.y16[i] = enc16(lr.blobs[i].cy);
            uint32_t a4 = lr.blobs[i].pixels >> 4;
            lf.area4[i] = (uint8_t)(a4 > 255 ? 255 : a4);
        }
        lf.frame_seq = ++s_frame_seq;
        {   // measure exactly what OpenFIRE will be handed.
            static uint8_t s_prev_pub_count = 255;
            s_pub_hist[lf.count > 4 ? 4 : lf.count] =
                    s_pub_hist[lf.count > 4 ? 4 : lf.count] + 1;   // not ++: -Wvolatile
            if (s_prev_pub_count != 255 && lf.count != s_prev_pub_count)
                s_pub_churn = s_pub_churn + 1;        // not ++: -Wvolatile
            s_prev_pub_count = lf.count;
        }
        ov2640_bridge_publish(&lf);
        began = false; prev_len = 0;
    }
}

#if OV_DASH_OWN_UART0
// Drains UART0's RX and hands whole "k=v&k=v\n" lines to the tuner, offering
// "aimcal=..." to the aim pipeline first. NOTE the whole function sits inside
// OV_DASH_OWN_UART0, which is gated on LIGHTGUN_DIAG, so a calibration can only
// be INSTALLED from the diag build; the shipping build still USES it, because
// aim_runtime_begin() loads it from NVS and NVS survives a reflash.
static void dash_rx_poll(void)
{
    static char ln[128];
    static int  n = 0;
    uint8_t b;
    while (uart_read_bytes(UART_NUM_0, &b, 1, 0) == 1) {
        if (b == '\n' || b == '\r') {
            if (n) {
                ln[n] = 0; n = 0;
#ifdef USE_AIM_PIPELINE
                if (aim_runtime_command(ln)) continue;
#endif
#if OV_TELEMETRY
                ov2640_tune(ln);
#endif
            }
        } else if (n < (int)sizeof(ln) - 1) {
            ln[n++] = (char)b;
        } else {
            n = 0;                                   // overlong: resync
        }
    }
}
#endif

#if OV_TELEMETRY
// Converts a wire coordinate into the dashboard's native-px x10 form. Signed:
// a Q/B line can legitimately carry a NEGATIVE coordinate (a reconstructed
// corner off-sensor), so readers must use `count`, not a sign test, to tell
// present from absent.
static inline int dash_x10(int16_t v16)
{
    const int32_t v = (int32_t)v16;
    return (int)((v * 10 + (v >= 0 ? 8 : -8)) / 16);
}

// Emits one dashboard fix line per published frame: B carries one point in the
// lab's wire format, Q carries all four.
static void dash_emit_fix(const ov2640_bridge_frame_t& f, uint32_t now_ms)
{
    if (s_dash == 1) {                               // legacy single-point form
        const uint8_t i = s_dash_blob;
        if (f.count == 0 || i >= f.count) {
            // n is still the truth: "4 points, but not the one you asked for"
            // reads differently from "nothing seen at all"
            ov_emit("B,%lu,%u,-1,-1\n", (unsigned long)now_ms, (unsigned)f.count);
            return;
        }
        ov_emit("B,%lu,%u,%d,%d\n", (unsigned long)now_ms, (unsigned)f.count,
               dash_x10(f.x16[i]), dash_x10(f.y16[i]));
        return;
    }
    int x[4], y[4];
    for (int i = 0; i < 4; ++i) {
        const bool have = (i < f.count);
        x[i] = have ? dash_x10(f.x16[i]) : -1;
        y[i] = have ? dash_x10(f.y16[i]) : -1;
    }
    ov_emit("Q,%lu,%u,%d,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)now_ms,
           (unsigned)f.count, x[0], y[0], x[1], y[1], x[2], y[2], x[3], y[3]);
}

// Emits the once-a-second STAT line, field for field as dashboard.py's panel
// regexes expect; overlay-only counters ride at the end.
static void dash_emit_stat(uint32_t now_ms, float fps, uint32_t worst_gap_ms,
                           uint8_t n)
{
    const uint32_t rus = cam_patch_restart_us; cam_patch_restart_us = 0;
    const float line_us = T_LINE_US;
    const int off_lines = (int)(rus / line_us);
    uint32_t vmin = cam_patch_vs_min_us, vmax = cam_patch_vs_max_us;
    uint32_t vcnt = cam_patch_vs_count;
    uint64_t vsum = cam_patch_vs_sum_us;
    cam_patch_vs_min_us = 0xFFFFFFFF; cam_patch_vs_max_us = 0;
    cam_patch_vs_count = 0; cam_patch_vs_sum_us = 0;
    const uint32_t vavg = vcnt ? (uint32_t)(vsum / vcnt) : 0;
    const uint32_t vspread = (vcnt && vmax >= vmin) ? (vmax - vmin) : 0;
    // NON-DESTRUCTIVE: the harness R-line and the V-line read the same counters
    // as cumulative totals, so zeroing them here would silently redefine two
    // existing diagnostics. Deltas against our own snapshot instead.
    static uint32_t p_ovfv = 0, p_ovfe = 0, p_vlong = 0, p_vshrt = 0, p_nost = 0;
    const uint32_t c_ovfv = cam_patch_ovf_vsync, c_ovfe = cam_patch_ovf_eof;
    const uint32_t c_vlong = cam_patch_vs_long, c_vshrt = cam_patch_vs_short;
    const uint32_t c_nost = cam_patch_nostart;
    const uint32_t ovfv = c_ovfv - p_ovfv, ovfe = c_ovfe - p_ovfe;
    const uint32_t vlong = c_vlong - p_vlong, vshrt = c_vshrt - p_vshrt;
    const uint32_t nost = c_nost - p_nost;
    p_ovfv = c_ovfv; p_ovfe = c_ovfe; p_vlong = c_vlong;
    p_vshrt = c_vshrt; p_nost = c_nost;
    char lost[80];
    if (ovfv | ovfe | vlong | vshrt | nost)
        snprintf(lost, sizeof(lost), "lost=ovf%u/%u,skip%u,short%u,nofb%u,ref%uus",
                 (unsigned)ovfv, (unsigned)ovfe, (unsigned)vlong,
                 (unsigned)vshrt, (unsigned)nost, (unsigned)cam_patch_vs_ref_us);
    else
        snprintf(lost, sizeof(lost), "lost=0");
    // flood=/short= are LAB concepts (aborted scan / truncated frame) that the
    // overlay does not measure. Pinned to 0 rather than repurposed, so two
    // different faults cannot read identically on one panel.
    ov_emit("STAT,%lu,%.1f,%.1f,%.0f,%d,thr=%d aec=%d agc=%d boost=%d y8=0 "
           "xclk=27 pdiv=3 flood=0%% short=0%% restart=%uus(~%dlines) "
           "vsync=%uus(min%u max%u spread%uus~%dlines) %s mode=%s\n",
           (unsigned long)now_ms, (double)fps, (double)worst_gap_ms, 0.0,
           (int)n, (int)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost,
           (unsigned)rus, off_lines,
           (unsigned)vavg, (unsigned)(vcnt ? vmin : 0), (unsigned)vmax,
           (unsigned)vspread, (int)(vspread / line_us), lost, "LAB");
}
#endif  // OV_TELEMETRY (dashboard stream)

// Drains the frame queue so the pipeline never stalls (detection already
// happened in the chunk callback; the fb itself is not needed).
static void drain_task(void*)
{
#if OV_TELEMETRY
    uint32_t last_dbg = 0;
    uint32_t dash_last_seq = 0, dash_last_us = 0;
    uint32_t dash_win_ms = 0, dash_win_seq = 0, dash_gap_ms = 0, dash_prev_ms = 0;
#endif
    for (;;) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        else vTaskDelay(1);
#if OV_TELEMETRY  // tune console + dashboard + telemetry
#if OV_DASH_OWN_UART0
        dash_rx_poll();
#endif
        // ---- dashboard stream (same task, same rule: never in on_chunk) ----
        if (s_dash) {
            ov2640_bridge_frame_t df;
            const uint32_t seq = ov2640_bridge_read(&df);
            const uint32_t us  = (uint32_t)esp_timer_get_time();
            const uint32_t ms  = us / 1000u;
            if (seq && seq != dash_last_seq) {
                if (!s_dash_min_dt_us || (uint32_t)(us - dash_last_us) >= s_dash_min_dt_us) {
                    dash_last_us = us;
                    dash_emit_fix(df, ms);
                }
                // worst frame-to-frame gap this second: the number that says
                // "we stopped seeing frames", independent of the rate cap
                if (dash_prev_ms && (ms - dash_prev_ms) > dash_gap_ms)
                    dash_gap_ms = ms - dash_prev_ms;
                dash_prev_ms = ms;
                dash_last_seq = seq;
            }
            if (!dash_win_ms) { dash_win_ms = ms; dash_win_seq = seq; }
            else if (ms - dash_win_ms >= 1000u) {
                const float fps = (float)(seq - dash_win_seq) * 1000.0f
                                  / (float)(ms - dash_win_ms);
                dash_emit_stat(ms, fps, dash_gap_ms, df.count);
                dash_win_ms = ms; dash_win_seq = seq; dash_gap_ms = 0;
            }
        }
        // Debug stream, deliberately NOT in on_chunk: a printf there blocks ~4ms
        // inside the DMA-chunk service path, which starves the driver and
        // destroys capture. This task is free to block.
        if (s_dbg) {
            const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            // 10Hz normally, 1Hz while the dashboard is streaming
            if (now - last_dbg >= (s_dash ? 1000u : 100u)) {
                last_dbg = now;
                ov2640_bridge_frame_t f;
                if (ov2640_bridge_read(&f)) {
                    // signed math, and the divisors must be SIGNED too -- mixing
                    // int32 with an unsigned denominator promotes the whole
                    // expression to unsigned and a negative coordinate becomes
                    // a huge positive
                    const int32_t xm = (int32_t)FRAME_W * 16 - 1, ym = (int32_t)FRAME_H * 16 - 1;
                    int sx[4] = {1023, 1023, 1023, 1023};
                    int sy[4] = {1023, 1023, 1023, 1023};
                    for (int i = 0; i < f.count; ++i) {  // mirror+scale = shim out
                        sx[i] = 1023 - (int)((int32_t)f.x16[i] * 1023 / xm);
                        sy[i] = (int)((int32_t)f.y16[i] * 767 / ym);
                    }
                    ov_emit("D,%lu,n=%u,%d,%d,%d,%d,%d,%d,%d,%d\n",
                           (unsigned long)f.frame_seq, (unsigned)f.count,
                           sx[0], sy[0], sx[1], sy[1], sx[2], sy[2], sx[3], sy[3]);
                    // once a second: the vertical-alignment health line
                    static uint32_t last_v = 0;
                    if (now - last_v >= 1000) {
                        last_v = now;
                        // ---- frame accounting ----
                        // Every frame the sensor starts must land in exactly one
                        // bucket. `hole` is the number that matters: it must be 0.
                        {
                            static uint32_t p_vs = 0, p_dl = 0, p_rj = 0, p_pub = 0;
                            static uint32_t p_cs = 0, p_st2 = 0, p_sz = 0, p_qu = 0;
                            const uint32_t vs = cam_patch_vs_total;
                            const uint32_t dl = cam_patch_frames_delivered;
                            const uint32_t rj = cam_patch_frames_rejected;
                            const uint32_t pub = s_frame_seq;
                            const uint32_t d_vs = vs - p_vs, d_dl = dl - p_dl;
                            const uint32_t d_rj = rj - p_rj, d_pub = pub - p_pub;
                            printf("ACCT/s: sensor=%lu delivered=%lu rejected=%lu "
                                   "published=%lu | HOLE=%ld | why: chunk=%lu stitch=%lu size=%lu queue=%lu\n",
                                   (unsigned long)d_vs, (unsigned long)d_dl,
                                   (unsigned long)d_rj, (unsigned long)d_pub,
                                   (long)d_vs - (long)d_dl - (long)d_rj,
                                   (unsigned long)(cam_patch_chunk_rej - p_cs),
                                   (unsigned long)(cam_patch_stitch_rej - p_st2),
                                   (unsigned long)(cam_patch_rej_size - p_sz),
                                   (unsigned long)(cam_patch_rej_queue - p_qu));
                            p_vs = vs; p_dl = dl; p_rj = rj; p_pub = pub;
                            p_cs = cam_patch_chunk_rej; p_st2 = cam_patch_stitch_rej;
                            p_sz = cam_patch_rej_size;
                            p_qu = cam_patch_rej_queue;
                        }
                        {   // ---- PTS: what OpenFIRE is actually handed.
                            // Target: pub4 == published, churn == 0.
                            static uint32_t p_h[5] = {0,0,0,0,0}, p_ch = 0;
                            uint32_t d[5], tot = 0;
                            for (int b = 0; b < 5; ++b) {
                                const uint32_t v = s_pub_hist[b];
                                d[b] = v - p_h[b]; p_h[b] = v; tot += d[b];
                            }
                            const uint32_t ch = s_pub_churn;
                            const uint32_t d_ch = ch - p_ch; p_ch = ch;
                            const uint32_t bad = tot - d[4];
                            printf("PTS/s: pub0=%lu pub1=%lu pub2=%lu pub3=%lu pub4=%lu "
                                   "| short=%lu/%lu (%lu%%) churn=%lu/s | every short "
                                   "frame makes OpenFIRE INVENT a corner; churn fires "
                                   "its spring (~70 frames of held error)\n",
                                   (unsigned long)d[0], (unsigned long)d[1],
                                   (unsigned long)d[2], (unsigned long)d[3],
                                   (unsigned long)d[4], (unsigned long)bad,
                                   (unsigned long)tot,
                                   (unsigned long)(tot ? (bad * 100u / tot) : 0u),
                                   (unsigned long)d_ch);
                            if (s_resolver) {
                                const QuadStats qs = quad_take_stats();
                                printf("RES/s: affine=%lu sim=%lu t1=%lu coast=%lu reassoc=%lu "
                                       "junk=%lu envrej=%lu reseed=%lu lost=%lu conf=%.2f | tilt=%.2f "
                                       "learned_max=%.2f | perspective resid=%.2fpx "
                                       "(worst %.2f) | resid climbing => affine no "
                                       "longer enough\n",
                                       (unsigned long)qs.reconstructed,
                                       (unsigned long)qs.recon_sim,
                                       (unsigned long)qs.recon_t,
                                       (unsigned long)qs.coasted,
                                       (unsigned long)qs.reassoc,
                                       (unsigned long)qs.dropped_blobs,
                                       (unsigned long)qs.env_rejects,
                                       (unsigned long)qs.reseeds,
                                       (unsigned long)qs.lock_losses,
                                       (double)s_res_conf,
                                       qs.aniso_x100 / 100.0,
                                       qs.env_aniso_x100 / 100.0,
                                       qs.resid_x100 / 100.0,
                                       qs.resid_max_x100 / 100.0);
                                // Stack headroom. The resolver runs INSIDE
                                // cam_task, which has a 4 KB stack at priority
                                // 23, so an overflow there is a panic, not a
                                // slowdown. Free bytes remaining, worst since
                                // boot; if this nears a few hundred, raise
                                // CAM_TASK_STACK before anything else.
                                const unsigned long stack_free = s_cam_task
                                    ? (unsigned long)uxTaskGetStackHighWaterMark(
                                          (TaskHandle_t)s_cam_task) * sizeof(StackType_t)
                                    : 0UL;
                                printf("COST/s: resolver worst=%luus total=%luus "
                                       "(%.2f%% of cam_task) reshape=%lu | "
                                       "LATENCY capture->OpenFIRE: last=%luus worst=%luus | "
                                       "cam_task stack free=%lub\n",
                                       (unsigned long)qs.worst_us,
                                       (unsigned long)qs.total_us,
                                       qs.total_us / 10000.0,
                                       (unsigned long)qs.reshapes,
                                       (unsigned long)ov2640_shim_lat_last_us,
                                       (unsigned long)ov2640_shim_lat_us,
                                       stack_free);
                                ov2640_shim_lat_us = 0;
                            }
                        }
                        {   // ---- ISR-grade VSYNC timing, vs the cam_task one.
                            // VSI is the VSYNC period measured IN THE ISR; the
                            // STAT vsync= field is the same period measured in
                            // cam_task. Tight VSI with a wide STAT one means the
                            // event is attributed to the wrong frame; both wide
                            // means the interrupt itself is late. eof/s must read
                            // ~11 x fps, else EOFs are coalescing.
                            static uint32_t p_eof = 0, p_vs = 0;
                            const uint32_t vmin = cam_patch_vsi_min_us;
                            const uint32_t vmax = cam_patch_vsi_max_us;
                            const uint32_t vcnt = cam_patch_vsi_cnt;
                            const uint64_t vsum = cam_patch_vsi_sum_us;
                            cam_patch_vsi_min_us = 0xFFFFFFFF; cam_patch_vsi_max_us = 0;
                            cam_patch_vsi_cnt = 0; cam_patch_vsi_sum_us = 0;
                            const uint32_t eof = cam_patch_eof_isr_total;
                            const uint32_t vst = cam_patch_vs_isr_total;
                            const uint32_t d_eof = eof - p_eof, d_vs = vst - p_vs;
                            p_eof = eof; p_vs = vst;
                            printf("ISR/s: vsync=%lu eof=%lu (expect eof=11xvsync=%lu) | "
                                   "VSI avg=%luus min=%lu max=%lu spread=%luus (~%.1f chunks) "
                                   "| tight VSI + wide STAT = ORDERING, both wide = ISR LATENCY\n",
                                   (unsigned long)d_vs, (unsigned long)d_eof,
                                   (unsigned long)(d_vs * 11u),
                                   (unsigned long)(vcnt ? (uint32_t)(vsum / vcnt) : 0),
                                   (unsigned long)(vcnt ? vmin : 0), (unsigned long)vmax,
                                   (unsigned long)((vcnt && vmax >= vmin) ? (vmax - vmin) : 0),
                                   (double)((vcnt && vmax >= vmin) ? (vmax - vmin) : 0)
                                       / (16.0 * (double)T_LINE_US));
                        }
                        {   // ---- CHUNKS: per-frame chunk-count histogram.
                            // Only the buckets that moved this second. Healthy =
                            // everything in bucket 11. Below 11 = the frame
                            // arrived short; above 11 = it ran past its VSYNC.
                            static uint32_t p_h[16] = {0};
                            char hb[160]; int hn = 0; uint32_t tot = 0, lo = 0, hi = 0;
                            for (int b = 0; b < 16; ++b) {
                                const uint32_t v = cam_patch_cnt_hist[b];
                                const uint32_t d = v - p_h[b];
                                p_h[b] = v;
                                if (!d) continue;
                                tot += d;
                                if (b < 11) lo += d; else if (b > 11) hi += d;
                                if (hn < (int)sizeof(hb) - 24)
                                    hn += snprintf(hb + hn, sizeof(hb) - hn,
                                                   " %d:%lu", b, (unsigned long)d);
                            }
                            if (!hn) { hb[0] = ' '; hb[1] = '-'; hb[2] = 0; }
                            static uint32_t p_ck = 0, p_dr = 0;
                            const uint32_t ck = s_coin_killed;
                            const uint32_t dr = cam_patch_eof_drained;
                            printf("CHUNKS/s (cnt:frames):%s | total=%lu short=%lu "
                                   "long=%lu | drained=%lu/s | coin=%s killed=%lu/s "
                                   "| healthy = all in 11, short+long = 0\n",
                                   hb, (unsigned long)tot,
                                   (unsigned long)lo, (unsigned long)hi,
                                   (unsigned long)(dr - p_dr),
                                   s_coin ? "on" : "OFF",
                                   (unsigned long)(ck - p_ck));
                            p_ck = ck; p_dr = dr;
                        }
                        {   // JUMP: geometry-independent. Hold still => 0.
                            static uint32_t p_o4 = 0, p_jp = 0, p_jq = 0;
                            const uint32_t o4 = s_ghost_over4, jp = s_jump_pts, jq = s_jump_q16;
                            printf("JUMP/s: moved=%lu onGrid16=%lu worst=%.1f lines "
                                   "over4=%lu maxBlobs=%u | holding still => moved must be 0\n",
                                   (unsigned long)(jp - p_jp), (unsigned long)(jq - p_jq),
                                   (double)s_jump_worst, (unsigned long)(o4 - p_o4),
                                   (unsigned)s_ghost_max);
                            p_o4 = o4; p_jp = jp; p_jq = jq;
                            s_jump_worst = 0; s_ghost_max = 0;
                        }
                        // Rates, not just the running totals: making the reader
                        // subtract two counters a second apart is how a clean
                        // experiment turns into a transcription error.
                        {
                            static uint32_t p_cr = 0, p_st = 0, p_rs = 0, p_vl = 0;
                            const uint32_t cr = cam_patch_chunk_rej, stc = cam_patch_stitch_rej;
                            const uint32_t rs = ov2640_stat_rej_size, vl = cam_patch_vs_long;
                            printf("LAPS/s: ring=%lu (chunkrej) stitch=%lu appRej=%lu "
                                   "sensorSkip=%lu | harness-alone reference ~1.2/s\n",
                                   (unsigned long)(cr - p_cr), (unsigned long)(stc - p_st),
                                   (unsigned long)(rs - p_rs), (unsigned long)(vl - p_vl));
                            p_cr = cr; p_st = stc; p_rs = rs; p_vl = vl;
                            static uint32_t p_g1 = 0, p_g3 = 0;
                            const uint32_t g1 = cam_patch_gap_over1ms, g3 = cam_patch_gap_over3ms;
                            printf("STARVE: worst gap %lu us this second (DMA needs "
                                   "servicing every ~350us) | >1ms x%lu  >3ms x%lu\n",
                                   (unsigned long)cam_patch_gap_max_us,
                                   (unsigned long)(g1 - p_g1), (unsigned long)(g3 - p_g3));
                            p_g1 = g1; p_g3 = g3; cam_patch_gap_max_us = 0;
                        }
                        // rejP= went with the VSYNC-period gate that fed it;
                        // dashboard.py tolerates a missing key
                        printf("V,restart_us=%lu,nostart=%lu,vs_long=%lu,"
                               "vs_short=%lu,rejS=%lu,chunks=%lu,"
                               "chunkrej=%lu,stitch=%lu\n",
                               (unsigned long)cam_patch_restart_us,
                               (unsigned long)cam_patch_nostart,
                               (unsigned long)cam_patch_vs_long,
                               (unsigned long)cam_patch_vs_short,
                               (unsigned long)ov2640_stat_rej_size,
                               (unsigned long)cam_patch_chunks_expected,
                               (unsigned long)cam_patch_chunk_rej,
                               (unsigned long)cam_patch_stitch_rej);
                        printf("Y,last_restart=%luus => y_fix=%.1f lines "
                               "(this is the vertical offset being removed)\n",
                               (unsigned long)cam_patch_restart_us_last,
                               (double)cam_patch_restart_us_last / 21.9);
                        // Running MAX: exactly ONE consumer may clear it, or each
                        // sees a fragment of the window and both under-report.
                        // STAT owns it while the dashboard stream is on.
                        if (!s_dash) cam_patch_restart_us = 0;
                    }
                }
            }
        }
#endif  // OV_TELEMETRY (per-second telemetry)
    }
}

// Starts the camera and the drain task. Returns 0 on success or if already
// started, so it is safe to call repeatedly.
int ov2640_capture_start(void)
{
#ifdef USE_AIM_PIPELINE
    aim_runtime_begin();          // load the stored calibration, or announce there is none
    aim_serial_set_extra(ov2640_cam_command);   // "~cam=thr:60,aec:40" -> tune console
#endif
    if (s_started) return 0;
    // Boot banner: prints the constants ACTUALLY COMPILED into this flash, so a
    // stale build is visible at a glance. Kept in both builds, deliberately.
    printf("OV2640Capture v29.7 %s | thr=%u aec=%d agc=%d boost=%d xclk=27 pdiv=3 | "
           "coin=%s res=%s lens=%u lead=%dms vfill=%d%%\n",
           LIGHTGUN_DIAG ? "DIAG" : "SHIP",
           (unsigned)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost,
           s_coin ? "on" : "off",
           s_resolver == 2 ? "ON(geom8)" : (s_resolver == 1 ? "ON(top4)" : "off"),
           (unsigned)s_lens, (int)s_lead_ms, (int)s_vfill_pct);
#if OV_TELEMETRY
    // dashboard handshake: the " HQVGA " token sets tools/dashboard.py's frame
    // size, and the k=v tail seeds its panel instead of leaving it a guess
    printf("CFG-ACTIVE: HQVGA 240x176 gray thr=%u aec=%d agc=%d boost=%d y8=0 "
           "xclk=27 pdiv=3 dbl=1 div=0 r32=0\n",
           (unsigned)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost);
    printf("dashboard: send \"dash=1\" for the B/STAT stream that "
           "tools/dashboard.py plots (off by default; ~2.9KB/s + ~25%% of this "
           "task busy-waiting on UART0 at full frame rate — use dashhz=60 to "
           "halve it). dashb=0..3 picks which point it plots. SNAP unsupported."
           "%s\n",
#if OV_DASH_OWN_UART0
           " UART0 RX is ours: dashboard commands work while OpenFIRE runs.");
#else
           " UART0 RX belongs to the sketch (harness forwards it).");
#endif
    {   // ===== MEM line: positive confirmation that PSRAM is alive ==========
        const size_t ps_tot = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        const size_t ps_fre = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        const size_t dr_fre = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const size_t dr_big = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        printf("MEM: psram total=%uK free=%uK | internal free=%uK largest=%uK | %s\n",
               (unsigned)(ps_tot / 1024), (unsigned)(ps_fre / 1024),
               (unsigned)(dr_fre / 1024), (unsigned)(dr_big / 1024),
               ps_tot ? "PSRAM OK" : "!! PSRAM DEAD - board_build.arduino.memory_type "
                                     "must be qio_opi on this N8R8 hardware");
    }
#endif  // OV_TELEMETRY (boot detail)
    quad_reset(nullptr);                             // arm the quad resolver
    cam_patch_chunk_cb = on_chunk;                   // hook BEFORE init
    camera_config_t c = {};
    c.pin_pwdn = -1; c.pin_reset = -1; c.pin_xclk = P_XCLK;
    c.pin_sccb_sda = P_SIOD; c.pin_sccb_scl = P_SIOC;
    c.pin_d7 = P_D7; c.pin_d6 = P_D6; c.pin_d5 = P_D5; c.pin_d4 = P_D4;
    c.pin_d3 = P_D3; c.pin_d2 = P_D2; c.pin_d1 = P_D1; c.pin_d0 = P_D0;
    c.pin_vsync = P_VSYNC; c.pin_href = P_HREF; c.pin_pclk = P_PCLK;
    c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
    c.xclk_freq_hz = 27000000;                       // 136fps; pdiv=3 slows the DVP bus
    c.pixel_format = PIXFORMAT_GRAYSCALE;
    c.frame_size = FRAMESIZE_HQVGA;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
    esp_err_t e = esp_camera_init(&c);
    if (e != ESP_OK) return (int)e;
#if LIGHTGUN_DIAG
    // Ring geometry, printed AFTER esp_camera_init(): cam_patch_dma_* are filled
    // in by cam_config(), inside it.
    {
        const uint32_t halfs = cam_patch_dma_halfs, lines = cam_patch_dma_lines;
        const uint32_t fh = lines ? (FRAME_H / lines) : 0;   // halves per frame
        printf("ALIGN: frame=%lu halves ring=%lu halves | frame%%ring=%lu %s\n",
               (unsigned long)fh, (unsigned long)halfs,
               (unsigned long)(halfs ? fh % halfs : 0),
               (!halfs || !lines) ? "UNKNOWN -- driver did not publish geometry"
                 : ((fh % halfs) == 0) ? "ALIGNED (phase repeats every frame)"
                                       : "MISALIGNED -- ring phase walks every frame");
        printf("RING: evq=%lu dma_halfs=%lu lines/half=%lu => overrun budget "
               "%.2f ms (%lu lines) | gate=%s\n",
               (unsigned long)cam_patch_evq_depth, (unsigned long)halfs,
               (unsigned long)lines, (double)(halfs * lines) * T_LINE_US / 1000.0,
               (unsigned long)(halfs * lines),
               cam_patch_gate_en ? "ON (spliced frames rejected)" : "OFF");
        printf("EXPECT: dma_halfs=4 lines/half=16 evq=3 budget~1.40ms "
               "chunks/frame=11 -- if dma_halfs differs, the ring is NOT the "
               "lab's and CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX is the difference\n");
    }
#endif  // LIGHTGUN_DIAG (ALIGN/RING geometry report)
    sensor_t* s = esp_camera_sensor_get();
    s_sensor = s;
#ifdef USE_AIM_PIPELINE
    bool s_cam_restored = false;
    // Restore tuned camera settings. AFTER the sensor exists, or the writes go
    // nowhere.
    {
        aim_cam_t cs;
        if (aim_cam_load(&cs)) {
            THR = (uint8_t)cs.thr;
            // Record only: the boot recipe below runs after this and writes
            // BOOT_AEC/BOOT_AGC unconditionally, so writing them here was undone
            // moments later. Re-applied after the recipe.
            s_cfg_aec = cs.aec;
            s_cfg_agc = cs.agc;
            s_cfg_boost = cs.boost;
            s_cam_restored = true;
            printf("CAM: restored thr=%d aec=%d agc=%d boost=%d from NVS\n",
                   cs.thr, cs.aec, cs.agc, cs.boost);
        }
        // The lead lives in its own key so that growing aim_cam_t cannot
        // invalidate a user's stored camera tuning.
        int lead_ms = 0;
        if (aim_lead_load(&lead_ms)) {
            s_lead_ms = (float)lead_ms;
            printf("CAM: restored lead=%dms from NVS\n", lead_ms);
        }
        // Lens is pure software state (no sensor registers), so the boot-recipe
        // ordering that bit the AEC/AGC restore cannot bite here.
        aim_lens_t ls;
        if (aim_lens_load(&ls)) {
            s_lens = (uint8_t)ls.model;
            s_lk1  = ls.k1;  s_lk2  = ls.k2;
            s_lfpx = ls.fpx; s_lfeq = ls.feq;
            printf("CAM: restored lens=%d k1=%.4f k2=%.4f fpx=%.1f feq=%.1f from NVS\n",
                   ls.model, (double)ls.k1, (double)ls.k2,
                   (double)ls.fpx, (double)ls.feq);
        }
    }
#endif
    // SCCB ACK check and retry. A transient NACK on the 0xFF bank-select leaves
    // every later write landing in the WRONG BANK -- a silently mis-programmed
    // sensor -- so the whole recipe is retried until it ACKs.
    int tries = 0;
    for (; s && tries < 3; ++tries) {                 // sensor recipe
        int rc = 0;
        rc |= s->set_whitebal(s, 0); rc |= s->set_special_effect(s, 0);
        rc |= s->set_lenc(s, 0);     rc |= s->set_raw_gma(s, 0);
        rc |= s->set_bpc(s, 0);      rc |= s->set_wpc(s, 0);   // low gain => off
        rc |= s->set_hmirror(s, 0);  rc |= s->set_vflip(s, 0);
        rc |= s->set_gain_ctrl(s, 0);     rc |= s->set_agc_gain(s, BOOT_AGC);  // boot recipe
        rc |= s->set_exposure_ctrl(s, 0); rc |= s->set_aec2(s, 0);
        rc |= s->set_aec_value(s, BOOT_AEC);         // see the boot recipe block
        rc |= s->set_reg(s, 0x112, 0x02, 0x00);      // COM7[1] test pattern OFF
        rc |= s->set_reg(s, 0x113, 0x20, 0x00);      // banding filter off
        rc |= s->set_reg(s, 0x113, 0x01, 0x00);      // COM8[0] AEC enable bit clear
        rc |= s->set_reg(s, 0x103, 0xC0, 0x00);      // COM1: no dummy frames
        // datasheet frame-timing kill list (Table 13):
        rc |= s->set_reg(s, 0x12D, 0xFF, 0x00); rc |= s->set_reg(s, 0x12E, 0xFF, 0x00); // ADDVSL/H: VSYNC width +0 lines
        // REG2A is an overlay-only write; its reset value is 0 and we write 0,
        // so it is a no-op in practice
        rc |= s->set_reg(s, 0x12A, 0xF0, 0x00);      // REG2A[7:4]: line-interval adj MSBs = 0
        rc |= s->set_reg(s, 0x12B, 0xFF, 0x00);      // FRARL: line-interval adj LSBs = 0
        rc |= s->set_reg(s, 0x146, 0xFF, 0x00); rc |= s->set_reg(s, 0x147, 0xFF, 0x00); // FLL/FLH: frame length +0
        rc |= s->set_reg(s, 0x111, 0xFF, 0x80);      // CLKRC: 2x, div 1
        rc |= s->set_reg(s, 0x132, 0xFF, 0x89);      // REG32 CIF (r32=0 form)
        rc |= s->set_reg(s, 0x0D3, 0xFF, 0x03);      // pdiv=3: the 136fps unlock
        // MUST be last: REG45 low bits belong to AEC (set above); only [7:6]
        rc |= s->set_reg(s, 0x145, 0xC0, BOOT_BOOST ? 0xC0 : 0x00);   // boost off
        if (rc == 0) break;
        printf("SCCB: recipe pass %d NACKed (rc=%d) - retrying\n", tries + 1, rc);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s && tries >= 3)
        printf("!! SCCB: recipe NEVER fully ACKed - the sensor is mis-programmed "
               "and every measurement after this line is suspect\n");

#ifdef USE_AIM_PIPELINE
    // Re-apply the stored exposure and gain after the boot recipe, which wrote
    // BOOT_AEC/BOOT_AGC over them. THR is a software threshold and needs no
    // re-apply; these two are sensor registers and do.
    if (s && s_cam_restored) {
        s->set_aec_value(s, s_cfg_aec);
        s->set_agc_gain(s, s_cfg_agc);
        printf("CAM: re-applied stored aec=%d agc=%d after the boot recipe\n",
               s_cfg_aec, s_cfg_agc);
    }
#endif
#if OV_DASH_OWN_UART0
    // Take UART0's RX so the dashboard can send commands while OpenFIRE owns the
    // USB CDC. TX buffer 0 on purpose: we never write through the driver, only
    // via printf()'s polling path, so the two can never interleave a line.
    if (!uart_is_driver_installed(UART_NUM_0))
        uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
#endif
    xTaskCreatePinnedToCore(drain_task, "ov_drain", 3072, nullptr, 3, nullptr, 1);
    s_started = true;
    return 0;
}

// ---- live tuning (bench tool) ---------------------------------------------
// Accepts "k=v&k=v": thr, aec, agc, boost and the rest, applied immediately.
// Bench use only -- writes race the camera task by one frame.
#if OV_TELEMETRY
// Translates "cam=thr:60,aec:40" from the app's '~' channel into the tune
// console's own "k=v&k=v" form, so there is exactly one parser for camera state.
extern "C" bool ov2640_cam_command(const char* line)
{
    if (!line) return false;
#ifdef USE_AIM_PIPELINE
    // "camsave" commits whatever is live now; "camreset" forgets it so the next
    // boot uses the compiled-in recipe again.
    if (!strncmp(line, "camsave", 7)) {
        aim_cam_t cs = { (int)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost };
        const bool ok = aim_cam_store(&cs);
        aim_lead_store((int)s_lead_ms);
        // Lens rides along; model 0 clears rather than stores, so a stale
        // stored lens cannot survive a return to the stock lens.
        aim_lens_t ls = { (int)s_lens, s_lk1, s_lk2, s_lfpx, s_lfeq };
        bool lens_ok = true;
        if (ls.model == 0) aim_lens_clear();
        else               lens_ok = aim_lens_store(&ls);
        ov_reply(ok && lens_ok
                 ? "CAM: saved thr=%d aec=%d agc=%d boost=%d lead=%dms lens=%d\n"
                 : "CAM: SAVE FAILED (values out of range?) thr=%d aec=%d agc=%d boost=%d lead=%dms lens=%d\n",
                 cs.thr, cs.aec, cs.agc, cs.boost, (int)s_lead_ms, ls.model);
        return true;
    }
    if (!strncmp(line, "camreset", 8)) {
        aim_cam_clear();
        aim_lens_clear();
        ov_reply("CAM: stored settings cleared (camera + lens); next boot uses the built-in recipe\n");
        return true;
    }
    if (!strncmp(line, "cam?", 4)) {
        // Everything the tools read back, one line: lead and lens included.
        ov_reply("CAM: thr=%d aec=%d agc=%d boost=%d lead=%d "
                 "lens=%d lk1u=%d lk2u=%d lfpx=%d lfeq=%d vfill=%d\n",
                 (int)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost, (int)s_lead_ms,
                 (int)s_lens, (int)(s_lk1*1e6f), (int)(s_lk2*1e6f),
                 (int)(s_lfpx*10.0f), (int)(s_lfeq*10.0f), (int)s_vfill_pct);
        return true;
    }
#endif
    if (strncmp(line, "cam=", 4) != 0) return false;
    char t[128]; int j = 0;
    for (const char* p = line + 4; *p && j < (int)sizeof(t) - 1; ++p)
        t[j++] = (*p == ':') ? '=' : ((*p == ',') ? '&' : *p);
    t[j] = 0;
    extern void ov2640_tune(const char* cmd);
    ov2640_tune(t);
    return true;
}

// Parses and applies a "k=v&k=v" tuning line.
extern "C" void ov2640_tune(const char* cmd)
{
    if (!cmd) return;
    const char* p = cmd;
    while (*p) {
        char key[8] = {0}; int ki = 0;
        while (*p && *p != '=' && *p != '&' && ki < 7) key[ki++] = *p++;
        if (*p != '=') { while (*p && *p != '&') ++p; if (*p) ++p; continue; }
        ++p;
        int val = 0; bool any = false; int sgn = 1;
        if (*p == '-') { sgn = -1; ++p; }               // lens k can be negative
        while (*p >= '0' && *p <= '9') { val = val * 10 + (*p++ - '0'); any = true; }
        val *= sgn;
        while (*p && *p != '&') ++p;
        if (*p) ++p;
        if (!any) continue;
        if (!strcmp(key, "thr")) {
            if (val < 8)   val = 8;
            if (val > 250) val = 250;
            THR = (uint8_t)val;
        } else if (!strcmp(key, "res")) {
            // 0 = off, 1 = resolver on top-4-by-mass, 2 = resolver picks four
            // from up to 8 by geometry (default)
            if (val < 0) val = 0;
            if (val > 2) val = 2;
            s_resolver = (uint8_t)val;
            quad_reset(nullptr);
        } else if (!strcmp(key, "lead")) {
            // latency lead in MILLISECONDS; 0 = off
            if (val < 0) val = 0;
            if ((float)val > LIGHTGUN_LEAD_MS_MAX) val = (int)LIGHTGUN_LEAD_MS_MAX;
            s_lead_ms = (float)val;
        } else if (!strcmp(key, "vfill")) {
            // LED vertical span as a PERCENT of screen height; 100 = off.
            // Range is 20..300, not 20..100: the standard OpenFIRE layout puts
            // one bar above the display and one below, so the span is routinely
            // 110-150% and the needed value is ABOVE 100.
            // RECALIBRATE in OpenFIRE after changing this.
            if (val < 20)  val = 20;
            if (val > 300) val = 300;
            s_vfill_pct = (float)val;
        } else if (!strcmp(key, "lens")) {
            if (val < 0) val = 0;
            if (val > 2) val = 2;
            s_lens = (uint8_t)val;                     // 0 off / 1 poly / 2 fisheye
        } else if (!strcmp(key, "lk1u")) {
            s_lk1 = (float)val * 1e-6f;                // micro units, signed
        } else if (!strcmp(key, "lk2u")) {
            s_lk2 = (float)val * 1e-6f;
        } else if (!strcmp(key, "lfpx")) {
            if (val > 0) s_lfpx = (float)val / 10.0f;  // tenths of a pixel
        } else if (!strcmp(key, "lfeq")) {
            if (val > 0) s_lfeq = (float)val / 10.0f;
        } else if (!strcmp(key, "coin")) {
            s_coin = (val != 0);       // temporal coincidence gate, A/B live
        } else if (!strcmp(key, "dbg")) {
            s_dbg = (val != 0);
        } else if (!strcmp(key, "dash")) {              // dashboard stream
            s_dash = (uint8_t)(val < 0 ? 0 : (val > 2 ? 2 : val));
            // Q is 2.4x the bytes of B; starting mode 2 uncapped would put the
            // drain task at ~60% busy-wait, and 60Hz is plenty to see a teleport.
            if (s_dash == 2 && s_dash_min_dt_us == 0) s_dash_min_dt_us = 1000000 / 60;
        } else if (!strcmp(key, "drvgate")) {
            cam_patch_gate_en = (val != 0) ? 1u : 0u;
        } else if (!strcmp(key, "dashb")) {
            s_dash_blob = (uint8_t)(val < 0 ? 0 : (val > 3 ? 3 : val));
        } else if (!strcmp(key, "dashhz")) {
            s_dash_min_dt_us = (val > 0) ? (uint32_t)(1000000 / val) : 0;
        } else if (!strcmp(key, "frame")) {
            // the dashboard's SNAP button: answer it rather than dying in silence
            printf("SNAPABORT: the overlay never keeps a framebuffer copy "
                   "(detection happens on the DMA chunks, the fb is returned "
                   "immediately). Pixel snapshots are a lab-firmware feature; "
                   "flash firmware/ for SNAP. B/STAT still work here.\n");
        } else if (s_sensor) {
            if      (!strcmp(key, "aec"))   { s_sensor->set_aec_value(s_sensor, val); s_cfg_aec = val; }
            else if (!strcmp(key, "agc"))   { s_sensor->set_agc_gain(s_sensor, val);  s_cfg_agc = val; }
            else if (!strcmp(key, "boost")) { s_sensor->set_reg(s_sensor, 0x145, 0xC0,
                                                                val ? 0xC0 : 0x00);
                                              s_cfg_boost = val ? 1 : 0; }
        }
    }
    // "CMD ok" prefix: dashboard.py runs parse_kv() on it, so its panel picks the
    // new values up immediately instead of waiting for the next STAT.
    ov_reply("CMD ok (tune) | thr=%u aec=%d agc=%d boost=%d | mode=LAB dash=%u "
           "dashb=%u dashhz=%lu drvgate=%u res=%u coin=%u\n",
           (unsigned)THR, s_cfg_aec, s_cfg_agc, s_cfg_boost,
           (unsigned)s_dash, (unsigned)s_dash_blob,
           (unsigned long)(s_dash_min_dt_us ? 1000000u / s_dash_min_dt_us : 0),
           (unsigned)cam_patch_gate_en,
           (unsigned)s_resolver, (unsigned)(s_coin ? 1 : 0));
}
#endif  // OV_TELEMETRY (tune console)
