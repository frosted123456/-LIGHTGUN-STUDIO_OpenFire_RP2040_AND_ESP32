// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <stdalign.h>
#include "esp_heap_caps.h"
#include "ll_cam.h"
#include "cam_hal.h"

#if (ESP_IDF_VERSION_MAJOR == 3) && (ESP_IDF_VERSION_MINOR == 3)
#include "rom/ets_sys.h"
#else
#include "esp_timer.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"  // will be removed in idf v5.0
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/ets_sys.h"
#endif
#endif // ESP_IDF_VERSION_MAJOR
#define ESP_CAMERA_ETS_PRINTF ets_printf

#if CONFIG_CAMERA_TASK_STACK_SIZE
#define CAM_TASK_STACK_RAW         CONFIG_CAMERA_TASK_STACK_SIZE
#else
#define CAM_TASK_STACK_RAW         (2*1024)
#endif
/* LIGHTGUN: floor the stack for ALL configurations. The patched paths log from
 * cam_task (short-frame / FB-SIZE / OVF), and plain-IDF logging overflows the
 * stock 2048-byte stack. This used to be applied only in the core-1 branch, so
 * dropping -DPATCH_CAM_TASK_CORE1 crashed instead of doing the A/B it invited. */
#if CAM_TASK_STACK_RAW < 4096
#define CAM_TASK_STACK             4096
#else
#define CAM_TASK_STACK             CAM_TASK_STACK_RAW
#endif

static const char *TAG = "cam_hal";

#ifdef PATCH_ACCEPT_SHORT_FRAMES
/* LIGHTGUN PATCH phase 2: optional per-chunk callback for streaming processing.
 * Called from cam_task (high prio) after each DMA chunk is filtered into the
 * frame buffer (frame_end=false), and once when a frame is accepted
 * (frame_end=true, before it is queued). Callback MUST be fast and never
 * block/print/alloc. App sets this before/after esp_camera_init at will. */
void (*cam_patch_chunk_cb)(const uint8_t* fb_base, size_t len_so_far, bool frame_end) = NULL;
/* LIGHTGUN: worst restart latency (us) since the app last read it. Capture is
 * restarted at the sensor's CURRENT position, so this latency IS the vertical
 * offset of the next frame, in lines: offset ~= latency_us / line_time_us.
 * A growing value here is exactly the "image drifts further and further" case. */
volatile uint32_t cam_patch_restart_us = 0;
/* v12.4: the SAME latency, but for the frame that is about to be captured and
 * NOT reduced to a running maximum. ll_cam_start() resumes capture wherever
 * the sensor currently is, so this microsecond count IS that frame's vertical
 * offset: offset_lines = restart_us_last / t_LINE. The app subtracts it from
 * every centroid, which turns a measured error into a correction instead of a
 * mystery. Frank's combined-build capture: 113-255us = 5-11 LINES, varying
 * frame to frame, which is exactly the size of the alternating vertical states
 * in his D-stream. */
volatile uint32_t cam_patch_restart_us_last = 0;
/* LIGHTGUN: VSYNC-to-VSYNC period statistics, in microseconds. This is the
 * SENSOR's own frame period as the ESP observes it. It is the measurement that
 * separates the two possible worlds:
 *   - period rock steady, picture still rolls  -> WE are losing alignment (software)
 *   - period itself wandering/creeping         -> the SENSOR's timing is moving
 *     (PLL/thermal at 35%% past spec), which no amount of driver work can fix.
 * Reset by the app each time it reads them. */
volatile uint32_t cam_patch_vs_min_us = 0xFFFFFFFF;
volatile uint32_t cam_patch_vs_max_us = 0;
volatile uint32_t cam_patch_vs_last_us = 0;
volatile uint32_t cam_patch_vs_count = 0;
volatile uint64_t cam_patch_vs_sum_us = 0;
/* LIGHTGUN v66 — WHY a period doubles. "max == 2 x typical" can mean three
 * different things and they need different fixes, so count each one instead of
 * inferring from a single max:
 *   ovf_vsync/ovf_eof : the event queue was FULL and the ISR threw the event
 *                       away. Purely our fault (cam_task too slow). The queue is
 *                       only dma_half_buffer_cnt-1 deep BY DESIGN — it is
 *                       backpressure matched to the DMA ring, so making it
 *                       deeper would swap a clean drop for silent corruption.
 *                       The fix for these is to make cam_task faster.
 *   vs_long           : a VSYNC-to-VSYNC interval >1.5x the rolling reference
 *                       with NO overflow recorded -> the pulse never reached us
 *                       at all, i.e. the SENSOR skipped it (overclock).
 *   vs_short          : <0.7x reference -> spurious edge (also sensor-side).
 *   nostart           : a real VSYNC arrived but no frame buffer was free, so
 *                       the frame was skipped. Costs fps, period stays correct.
 * ovf_* and vs_long are mutually exclusive explanations; whichever is nonzero
 * tells you which layer to work on. */
volatile uint32_t cam_patch_ovf_vsync = 0;
/* LIGHTGUN v9.1: frames rejected because DMA overran the buffer (a VSYNC
 * event was lost mid-frame -> frame N tail stitched to frame N+1 head at
 * EXACTLY full length -> previously delivered as "complete" with every blob
 * vertically shifted; the app's exact-size gate could not catch it). */
volatile uint32_t cam_patch_stitch_rej = 0;
static bool s_lightgun_overrun = false;
/* v12.2 CHUNK-COUNT ALIGNMENT CHECK.
 * Frank's field report: "sometimes 1 point shifts, sometimes 2, sometimes all
 * 4 — it is not tremor, it shifts suddenly on its own." That is the exact
 * signature of a frame whose ROWS ARE SPLICED PART-WAY DOWN: everything above
 * the splice keeps its position, everything below is displaced. So the number
 * of points that appear to jump depends only on where the splice lands.
 * A splice happens when a frame boundary is missed and two frames are joined.
 * The existing detector for that (FB-OVF, cam_patch_stitch_rej) only fires
 * when the joined data OVERFLOWS the buffer — if the pieces happen to fit,
 * the frame passes every size check with silently displaced rows.
 * A healthy frame is always assembled from the SAME number of DMA chunks.
 * So: learn that number at runtime and reject any frame that differs. This
 * catches spliced and truncated frames alike, whatever their byte count. */
volatile uint32_t cam_patch_chunks_expected = 0;
volatile uint32_t cam_patch_chunk_rej = 0;
/* v15 (2026-08-11): BOTH of the gates above are OVERLAY-ONLY. The lab driver
 * (firmware/lib/esp32-camera-s3) has neither, and the lab tracks perfectly —
 * so as long as the overlay runs them, "lab-identical mode" is a claim we
 * cannot make. They stay in the tree (they were added for real, reproduced
 * faults) but are now RUNTIME-SWITCHED and default OFF, so the baseline is
 * genuinely the lab's. Turn back on with "drvgate=1".
 * NOTE what these gates do and do NOT do: they reject the FRAMEBUFFER at
 * frame close, which is AFTER the chunk callbacks have already run. What they
 * DO suppress is the frame_end callback itself (see the !frames[].en guard at
 * the patch_done_buf assignment) — so a rejected frame is never PUBLISHED.
 *
 * v15.1 REVERTED TO DEFAULT ON. Defaulting these off was my mistake and Frank
 * caught it in one test ("capture is incorrect ... points duplicated on the Y
 * axis"). The reasoning that produced it — "the lab does not have these, so
 * lab parity means removing them" — is wrong, because the lab does not have
 * the FAULT either. This build measures ~30 DMA overruns per second; the lab
 * measures none. A protection that answers a fault the reference does not have
 * is not a divergence to be flattened, it is the one thing keeping a spliced
 * frame out of the aim stream. Parity applies to the ALGORITHM, never to the
 * defences. "drvgate=0" still turns them off for A/B, and the counters keep
 * incrementing either way so the fault stays visible. */
volatile uint32_t cam_patch_gate_en = 1;
/* v15.2 starvation meter — see the comment at the xQueueReceive below. */
volatile uint32_t cam_patch_gap_max_us  = 0;
volatile uint32_t cam_patch_gap_over1ms = 0;
volatile uint32_t cam_patch_gap_over3ms = 0;
static  int64_t   s_lg_gap_prev_us = 0;
/* v15.3 FRAME ACCOUNTING. The v15.2 starvation meter REFUTED its own
 * hypothesis (its >1ms count was exactly one per frame = normal inter-frame
 * blanking, and its 8.6ms outliers correlated 16/16 with sensor VSYNC skips,
 * not with anything OpenFIRE does). Neither A/B lever moved the lap rate.
 * Meanwhile the books do not balance: the sensor delivers 135 fps, ~26/s are
 * rejected, and only 60-90/s are published -- 20-45 frames per second vanish
 * with no counter naming them. These three close that gap. Every frame the
 * driver closes lands in EXACTLY ONE of delivered/rejected, and vs_total is
 * free-running so nothing else can reset it out from under the arithmetic. */
volatile uint32_t cam_patch_frames_delivered = 0;
volatile uint32_t cam_patch_frames_rejected  = 0;
/* v15.5: HALF of all rejections had no counter. ACCT measured rejected=50-64/s
 * while chunkrej+stitch only ever reached 25-32/s. These name the rest, so the
 * rejection total is fully attributed instead of half-explained. */
volatile uint32_t cam_patch_rej_size  = 0;   /* FB-SIZE, below the 90% floor */
volatile uint32_t cam_patch_rej_queue = 0;   /* frame queue full (FBQ-SND/RCV) */
volatile uint32_t cam_patch_ovf_eof   = 0;
volatile uint32_t cam_patch_vs_long   = 0;
volatile uint32_t cam_patch_vs_short  = 0;
volatile uint32_t cam_patch_vs_ref_us = 0;
volatile uint32_t cam_patch_nostart   = 0;
/* ring geometry, filled in at cam_config: event queue depth / DMA half-buffers */
volatile uint32_t cam_patch_evq_depth = 0;
volatile uint32_t cam_patch_dma_halfs = 0;
volatile uint32_t cam_patch_dma_lines = 0;
/* ===== LIGHTGUN v16 INSTRUMENT: chunks-per-frame histogram =================
 * cam_patch_chunk_rej only ever told us "cnt != expected". It never said WHICH
 * WAY, and the two directions have opposite causes:
 *   cnt < expected  -> the frame arrived SHORT: capture restarted inside the
 *                      active region, or an EOF interrupt was masked/coalesced
 *                      (masked interrupts are NOT counted by cam_patch_ovf_eof,
 *                      which only counts xQueueSendFromISR failures).
 *   cnt > expected  -> the frame ran PAST its VSYNC boundary: two frames merged,
 *                      i.e. a genuine stitch.
 * Bucket [i] = frames that closed with cnt == i; bucket 15 = 15 or more.
 * Read it once a second from the app; nothing here allocates or blocks. */
volatile uint32_t cam_patch_cnt_hist[16] = {0};
/* ===== LIGHTGUN v17: ISR-GRADE EVENT ACCOUNTING ===========================
 * v16's CHUNKS histogram proved the EOF count is CONSERVED across a fault pair
 * (one frame closes at 12 -- invisible, the FB-OVF guard eats the 12th before
 * cnt++ -- and the next at 10, and short==stitch==chunk_rej exactly). So no DMA
 * chunk is lost: the FRAME BOUNDARY is misplaced by exactly one chunk. The task-
 * context VSYNC period is bimodal 7047/7745us, i.e. +-350us = +-1 chunk, which
 * is the same statement.
 * That leaves exactly two candidates, and they need opposite fixes:
 *   (a) the VSYNC INTERRUPT itself is late (masked / behind a shared LOWMED
 *       chain / non-IRAM handler during a cache-disable window), or
 *   (b) the interrupt is punctual and the EVENT is dequeued late, i.e. the GDMA
 *       EOF ISR enqueues ahead of the LCD_CAM VSYNC ISR and cam_task -- which is
 *       strictly FIFO -- attributes the chunk to the wrong frame.
 * These counters are written INSIDE ll_cam_send_event(), which runs in ISR
 * context for both event types, so they measure (a) directly:
 *   VSI spread small + task-side spread 690us  => (b), an ordering problem
 *   VSI spread also ~690us                     => (a), an ISR-latency problem
 * eof_total also settles coalescing outright: it must read ~11 x fps. */
volatile uint32_t cam_patch_vsi_min_us = 0xFFFFFFFF;
volatile uint32_t cam_patch_vsi_max_us = 0;
volatile uint32_t cam_patch_vsi_cnt    = 0;
volatile uint64_t cam_patch_vsi_sum_us = 0;
volatile uint32_t cam_patch_eof_isr_total = 0;   /* EOF events raised by the ISR */
volatile uint32_t cam_patch_vs_isr_total  = 0;   /* VSYNC events raised by the ISR */
static  int64_t   s_vsi_prev_us = 0;
/* v19: tail EOFs recovered at the frame boundary by the race fix. If the race
 * theory is right this should land at roughly the OLD chunk_rej rate while
 * chunk_rej and stitch_rej both collapse toward zero. If drained/s stays 0 and
 * the faults persist, the theory is wrong and I need to look elsewhere. */
volatile uint32_t cam_patch_eof_drained = 0;
#endif
static cam_obj_t *cam_obj = NULL;

static const uint32_t JPEG_SOI_MARKER = 0xFFD8FF;  // written in little-endian for esp32
static const uint16_t JPEG_EOI_MARKER = 0xD9FF;  // written in little-endian for esp32

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        if (memcmp(&inbuf[i], &JPEG_SOI_MARKER, 3) == 0) {
            //ESP_LOGW(TAG, "SOI: %d", (int) i);
            return i;
        }
    }
    ESP_LOGW(TAG, "NO-SOI");
    return -1;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length)
{
    int offset = -1;
    uint8_t *dptr = (uint8_t *)inbuf + length - 2;
    while (dptr > inbuf) {
        if (memcmp(dptr, &JPEG_EOI_MARKER, 2) == 0) {
            offset = dptr - inbuf;
            //ESP_LOGW(TAG, "EOI: %d", length - (offset + 2));
            return offset;
        }
        dptr--;
    }
    return -1;
}

static bool cam_get_next_frame(int * frame_pos)
{
    if(!cam_obj->frames[*frame_pos].en){
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].en) {
                *frame_pos = x;
                return true;
            }
        }
    } else {
        return true;
    }
    return false;
}

static bool cam_start_frame(int * frame_pos)
{
    if (cam_get_next_frame(frame_pos)) {
        if(ll_cam_start(cam_obj, *frame_pos)){
            // Vsync the frame manually
            ll_cam_do_vsync(cam_obj);
            uint64_t us = (uint64_t)esp_timer_get_time();
            cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = us / 1000000UL;
            cam_obj->frames[*frame_pos].fb.timestamp.tv_usec = us % 1000000UL;
            return true;
        }
    }
    return false;
}

void IRAM_ATTR ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t * HPTaskAwoken)
{
#ifdef PATCH_ACCEPT_SHORT_FRAMES
    /* v17: ISR-grade accounting. This is the ONLY place both event types pass
     * through in interrupt context, so it measures the interrupt itself rather
     * than when cam_task got round to looking. esp_timer_get_time() is
     * IRAM-safe; the EOF path is a single increment at ~1.5 kHz. */
    if (cam_event == CAM_VSYNC_EVENT) {
        const int64_t nv = esp_timer_get_time();
        cam_patch_vs_isr_total++;
        if (s_vsi_prev_us) {
            const uint32_t d = (uint32_t)(nv - s_vsi_prev_us);
            if (d > 3000u && d < 1000000u) {      /* skip the fake restart edge */
                if (d < cam_patch_vsi_min_us) cam_patch_vsi_min_us = d;
                if (d > cam_patch_vsi_max_us) cam_patch_vsi_max_us = d;
                cam_patch_vsi_sum_us += d;
                cam_patch_vsi_cnt++;
            }
        }
        s_vsi_prev_us = nv;
    } else {
        cam_patch_eof_isr_total++;
    }
#endif
    if (xQueueSendFromISR(cam->event_queue, (void *)&cam_event, HPTaskAwoken) != pdTRUE) {
        ll_cam_stop(cam);
        cam->state = CAM_STATE_IDLE;
#ifdef PATCH_ACCEPT_SHORT_FRAMES
        /* LIGHTGUN PATCH: printing from ISR context on every overflow floods the
         * UART and makes the overflow spiral worse. Print 1 in 128 with a count. */
        if (cam_event == CAM_IN_SUC_EOF_EVENT) cam_patch_ovf_eof++;
        else                                   cam_patch_ovf_vsync++;
        static uint32_t s_ovf_cnt = 0;
        if ((s_ovf_cnt++ & 0x7F) == 0) {
            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: EV-%s-OVF (x%u)\r\n"),
                cam_event==CAM_IN_SUC_EOF_EVENT ? DRAM_STR("EOF") : DRAM_STR("VSYNC"),
                (unsigned) s_ovf_cnt);
        }
#else
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: EV-%s-OVF\r\n"), cam_event==CAM_IN_SUC_EOF_EVENT ? DRAM_STR("EOF") : DRAM_STR("VSYNC"));
#endif
    }
}

//Copy fram from DMA dma_buffer to fram dma_buffer
static void cam_task(void *arg)
{
    int cnt = 0;
    int frame_pos = 0;
    cam_obj->state = CAM_STATE_IDLE;
    cam_event_t cam_event = 0;
    int64_t last_vsync_us = 0;      /* LIGHTGUN: per-session (was function-static,
                                     * which leaked across deinit/reinit) */

    xQueueReset(cam_obj->event_queue);

    while (1) {
        xQueueReceive(cam_obj->event_queue, (void *)&cam_event, portMAX_DELAY);
        /* LIGHTGUN v15.2 — STARVATION METER.
         * Bisecting OpenFIRE by build flag is not available to us: stripping
         * USES_DISPLAY or the wireless block fails to compile in OpenFIRE's OWN
         * tree (pauseModeSelection / ESP_MAC_WIFI_STA are used outside their
         * guards), and we do not fork. So measure the thing directly instead of
         * inferring it from a feature bisect.
         * This is the gap between consecutive services of the DMA event queue.
         * The DMA delivers a half-buffer every 16 lines (~350us at 136fps) and
         * the ring holds a handful of them, so once this gap exceeds the ring
         * budget the DMA laps and the frame is spliced — which is exactly the
         * +-4 / +-8 half-buffer displacements measured in the field. A worst
         * gap of N ms says how long core 1 was taken away from us, whoever
         * took it. That is the number the A/B is really chasing. */
        {
            const int64_t nowg = esp_timer_get_time();
            if (s_lg_gap_prev_us) {
                const uint32_t g = (uint32_t)(nowg - s_lg_gap_prev_us);
                if (g > cam_patch_gap_max_us) cam_patch_gap_max_us = g;
                if (g > 1000)  cam_patch_gap_over1ms++;
                if (g > 3000)  cam_patch_gap_over3ms++;
            }
            s_lg_gap_prev_us = nowg;
        }
        DBG_PIN_SET(1);
#ifdef PATCH_ACCEPT_SHORT_FRAMES
        if (cam_event == CAM_VSYNC_EVENT) {
            static int64_t vs_prev = 0;          /* per-boot is fine for stats */
            int64_t vs_now = esp_timer_get_time();
            if (vs_prev) {
                uint32_t d = (uint32_t)(vs_now - vs_prev);
                if (d < 1000000) {               /* ignore init gaps */
                    cam_patch_vs_last_us = d;
                    if (d < cam_patch_vs_min_us) cam_patch_vs_min_us = d;
                    if (d > cam_patch_vs_max_us) cam_patch_vs_max_us = d;
                    cam_patch_vs_sum_us += d;
                    cam_patch_vs_count++;
                    /* v66: classify against a ROLLING minimum, not the session
                     * average — the average is polluted by the very anomalies we
                     * are trying to count, and the session minimum never
                     * recovers after one bad sample. Window of 512 (~4s @136fps)
                     * tracks thermal drift without tracking the glitches. */
                    static uint32_t win_n = 0, win_min = 0xFFFFFFFF;
                    if (d < win_min) win_min = d;
                    if (++win_n >= 512) {
                        cam_patch_vs_ref_us = win_min;
                        win_min = 0xFFFFFFFF; win_n = 0;
                    }
                    uint32_t r = cam_patch_vs_ref_us;
                    if (r == 0) { cam_patch_vs_ref_us = d; r = d; }
                    if (d > r + (r >> 1))        cam_patch_vs_long++;   /* >1.5x */
                    else if (d * 10u < r * 7u)   cam_patch_vs_short++;  /* <0.7x */
                }
            }
            vs_prev = vs_now;
        }
#endif
#ifdef PATCH_ACCEPT_SHORT_FRAMES
        /* LIGHTGUN PATCH: VSYNC glitch filter. A cold overclocked sensor emits
         * spurious VSYNC edges (field: EV-VSYNC-OVF storms + minutes-long fps
         * ramp-up). Max real frame rate is ~135fps = 7.4ms period; any VSYNC
         * arriving <3ms after the previous one is physically impossible and
         * gets dropped instead of closing/restarting frames. */
        if (cam_event == CAM_VSYNC_EVENT && cam_obj->state == CAM_STATE_IDLE) {
            int64_t nowv = esp_timer_get_time();
            if (last_vsync_us != 0 && (nowv - last_vsync_us) < 3000) {
                static uint32_t s_vs_glitch = 0;
                if ((s_vs_glitch++ & 0x3FF) == 0) {
                    ESP_LOGW(TAG, "VSYNC glitch filtered (x%u)", (unsigned) s_vs_glitch);
                }
                DBG_PIN_SET(0);
                continue;
            }
            last_vsync_us = nowv;
        } else if (cam_event == CAM_VSYNC_EVENT) {
            /* In READ_BUF a VSYNC is a real frame boundary — NEVER drop it.
             * Dropping one lets DMA run past the boundary; the frame then closes
             * on the following VSYNC holding the tail of frame N stitched to the
             * head of N+1, at full length, so it is delivered as "complete" and
             * every later frame inherits the shift. */
            last_vsync_us = esp_timer_get_time();
        }
#endif
        switch (cam_obj->state) {

            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) {
                    //DBG_PIN_SET(1);
                    if(cam_start_frame(&frame_pos)){
                        cam_obj->frames[frame_pos].fb.len = 0;
                        s_lightgun_overrun = false;   /* v9.1: fresh frame */
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                        /* LIGHTGUN: explicit FRAME-START signal (len == 0). The
                         * app used to infer frame boundaries from the byte count
                         * going down, which fails when a frame is REJECTED (no
                         * frame_end callback) and the next frame's first chunk
                         * happens to match the old length — the row cursor then
                         * never reset and the frame's leading rows went
                         * unscanned. An explicit signal removes the guesswork. */
                        if (cam_patch_chunk_cb && !cam_obj->jpeg_mode && !cam_obj->psram_mode) {
                            cam_patch_chunk_cb(cam_obj->frames[frame_pos].fb.buf, 0, false);
                        }
#endif
                        cam_obj->state = CAM_STATE_READ_BUF;
                    }
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                    else {
                        /* v66: a real VSYNC we could not act on because every
                         * frame buffer is still held by the app. Loses a frame
                         * but NOT a VSYNC — the period stays correct, so this
                         * shows up as low fps with a clean vsync= field. */
                        cam_patch_nostart++;
                    }
#endif
                    cnt = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                camera_fb_t * frame_buffer_event = &cam_obj->frames[frame_pos].fb;
                size_t pixels_per_dma = (cam_obj->dma_half_buffer_size * cam_obj->fb_bytes_per_pixel) / (cam_obj->dma_bytes_per_item * cam_obj->in_bytes_per_pixel);

                if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                    if(!cam_obj->psram_mode){
                        if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
                            static uint32_t s_ovf_b = 0;      /* LIGHTGUN: throttled */
                            if ((s_ovf_b++ & 0x7F) == 0)
                                ESP_LOGW(TAG, "FB-OVF (x%u)", (unsigned)s_ovf_b);
                            /* v9.1: chunks beyond a full buffer mean a VSYNC was
                             * missed — this frame is frame-N + frame-N+1 STITCHED.
                             * Its length is exactly fb_size, so it would pass the
                             * size check at close: POISON it explicitly. */
                            s_lightgun_overrun = true;
                            ll_cam_stop(cam_obj);
                            DBG_PIN_SET(0);
                            continue;
                        }
                        frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                            &frame_buffer_event->buf[frame_buffer_event->len],
                            &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                            cam_obj->dma_half_buffer_size);
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                        if (cam_patch_chunk_cb && !cam_obj->jpeg_mode) {
                            cam_patch_chunk_cb(frame_buffer_event->buf, frame_buffer_event->len, false);
                        }
#endif
                    }
                    //Check for JPEG SOI in the first buffer. stop if not found
                    if (cam_obj->jpeg_mode && cnt == 0 && cam_verify_jpeg_soi(frame_buffer_event->buf, frame_buffer_event->len) != 0) {
                        ll_cam_stop(cam_obj);
                        cam_obj->state = CAM_STATE_IDLE;
                    }
                    cnt++;

                } else if (cam_event == CAM_VSYNC_EVENT) {
                    //DBG_PIN_SET(1);
                    int64_t patch_vsync_us = esp_timer_get_time();
                    ll_cam_stop(cam_obj);
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                    /* LIGHTGUN: the frame-end notification is DEFERRED to after
                     * capture restarts (see below). Between ll_cam_stop() here
                     * and cam_start_frame() further down, the sensor keeps
                     * streaming into nothing — every microsecond spent in app
                     * code lands as lost lines at the TOP of the next frame.
                     * The app's frame_end work (connected-components merge) was
                     * being paid for exactly there. */
                    uint8_t *patch_done_buf = NULL;
                    size_t   patch_done_len = 0;
                    int      patch_done_pos = frame_pos;
                    bool     patch_started = false;
                    bool     patch_restarted = false;

                    /* ===== v19: THE EOF/VSYNC RACE FIX ======================
                     * ROOT CAUSE, pinned by five independent measurements:
                     *   eof_isr_total == 11 x vsync_isr_total EXACTLY  -> no EOF
                     *       is ever lost or coalesced;
                     *   VSI (ISR-measured VSYNC period) spread 8-9us   -> the
                     *       VSYNC interrupt is punctual;
                     *   chunk_rej == stitch_rej, always                -> one
                     *       fault costs exactly two consecutive frames;
                     *   cnt is never >= 12 at close, and the event queue never
                     *       overflows.
                     * The only mechanism consistent with all five: the 11th
                     * (last) EOF of a frame completes at the END of active
                     * video, and VSYNC asserts at the START of blanking -- the
                     * same instant. They are two SEPARATE interrupt sources, so
                     * when VSYNC wins the race its event is enqueued first:
                     *   frame N   closes with cnt=10, len=38400  -> chunk_rej
                     *   frame N+1 inherits N's 11th EOF, reaches 12 -> the
                     *             FB-OVF guard trips              -> stitch_rej
                     * Conserved, paired, invisible to every existing counter.
                     * Ring size cannot affect it (it is a COUNT fault, not a
                     * slot fault), which is why -D LIGHTGUN_DMA_RING_BYTES
                     * changed nothing. IRAM ISRs cut it 4x by reducing the
                     * jitter that decides the race, but cannot remove it.
                     *
                     * THE FIX: at the frame boundary, drain any EOF that is
                     * ALREADY PENDING in the queue and attribute it to the
                     * frame that just ended -- which is provably where it
                     * belongs. Proof: vertical blanking is ~160 lines (~3.5 ms)
                     * and the next chunk's EOF cannot occur until 350 us of
                     * active video after blanking ends. So an EOF pending at
                     * the frame-end VSYNC is unambiguously this frame's tail.
                     * A non-EOF event is pushed back with xQueueSendToFront so
                     * no frame boundary can ever be swallowed.
                     *
                     * v19.1: the drain runs AFTER cam_start_frame(), not before.
                     * Placing it before cost 320us of extra stop->restart
                     * latency (restart_us jumped 40 -> 367us, ~16 lines) because
                     * the memcpy AND the app's blobstream_feed on those rows ran
                     * inside the window the "RESTART CAPTURE FIRST" block exists
                     * to keep empty. It was harmless -- 367us is still deep
                     * inside the ~3.5ms vertical blanking, and short/rejS stayed
                     * 0 -- but it ate 10% of the blanking margin for nothing.
                     * Draining after the restart is equally correct: the old
                     * frame's buffer is already claimed (en=0) and
                     * frame_buffer_event still points at it, while the next new
                     * EOF cannot arrive for >3ms (blanking + 350us of active
                     * video). See the drain site further down. */
#endif

                    if (cnt || !cam_obj->jpeg_mode || cam_obj->psram_mode) {
                        if (cam_obj->jpeg_mode) {
                            if (!cam_obj->psram_mode) {
                                if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
                                    /* LIGHTGUN: throttled — this can fire every
                                     * frame on a format mismatch, and logging
                                     * from cam_task at frame rate is itself
                                     * enough to destroy capture. */
                                    static uint32_t s_ovf_a = 0;
                                    if ((s_ovf_a++ & 0x7F) == 0)
                                        ESP_LOGW(TAG, "FB-OVF (x%u)", (unsigned)s_ovf_a);
                                    cnt--;
                                } else {
                                    frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                                        &frame_buffer_event->buf[frame_buffer_event->len],
                                        &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                                        cam_obj->dma_half_buffer_size);
                                }
                            }
                            cnt++;
                        }

#ifdef PATCH_ACCEPT_SHORT_FRAMES
                        /* LIGHTGUN PATCH — TAIL FLUSH: **DISABLED**, it was doing harm.
                         *
                         * Original premise: "the final DMA half-buffer never fires an
                         * EOF (it isn't full), so copy the missing bytes out of DMA
                         * memory." That premise is false here — ll_cam_calc_rgb_dma()
                         * forces height %% lines_per_half_buffer == 0, so in a healthy
                         * frame every half-buffer is exactly full and every EOF fires.
                         *
                         * What actually happened: len and fb_size are therefore both
                         * exact multiples of pixels_per_dma, so `missing` is always a
                         * WHOLE number of chunks. Combined with the `missing <=
                         * pixels_per_dma` gate, the copy only ever ran when exactly one
                         * chunk was absent — and then copied a FULL half-buffer from
                         * slot (cnt %% 2). With only 2 slots that slot holds data from
                         * two chunks ago, i.e. rows from EARLIER IN THE SAME FRAME.
                         * The frame was then stamped len == fb_size, so it reported
                         * 100%% complete: duplicated rows, phantom blobs at plausible
                         * coordinates, and nothing in short_pct to reveal it.
                         *
                         * Correct behaviour for a genuinely incomplete frame is the
                         * >=90%% acceptance rule below (which reports it honestly via
                         * fb->len) or rejection. Kept as documentation of a real trap. */
#endif

                        cam_obj->frames[frame_pos].en = 0;
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                        /* ==== LIGHTGUN: RESTART CAPTURE FIRST ====
                         * ll_cam_start() begins capturing at the sensor's CURRENT
                         * position and ll_cam_do_vsync() then fakes the frame-start
                         * edge, so the captured window's origin is wherever the
                         * sensor happened to be when we restarted. Every
                         * microsecond between the real VSYNC and cam_start_frame()
                         * is therefore a VERTICAL OFFSET (~25us per line at 135fps).
                         *
                         * The whole frame-completion block below — size checks,
                         * rate-limited logging, xQueueSend, cam_give, queue pops —
                         * used to run INSIDE that window, and its cost varies with
                         * queue depth and log traffic. A varying offset frame after
                         * frame is exactly the "image starts OK and drifts worse
                         * over time" symptom: the capture window slides relative to
                         * the sensor's frame, i.e. the picture rolls.
                         *
                         * The finished frame is already claimed (en = 0 above), so
                         * the allocator cannot hand it back to us. Restart now and
                         * do the bookkeeping afterwards, off the critical path. */
                        patch_done_pos = frame_pos;
                        patch_started = cam_start_frame(&frame_pos);
                        {
                            uint32_t dt = (uint32_t)(esp_timer_get_time() - patch_vsync_us);
                            if (dt > cam_patch_restart_us) cam_patch_restart_us = dt;
                            cam_patch_restart_us_last = dt;      /* v12.4 */
                        }
                        if (patch_started) cam_obj->frames[frame_pos].fb.len = 0;
                        patch_restarted = patch_started;   /* else retry below,
                                                            * after cam_give()
                                                            * frees a buffer */

                        /* ===== v19 THE EOF/VSYNC RACE FIX (drain site) ======
                         * Capture is running again, so spending time here is
                         * free. Claim any EOF already pending in the queue for
                         * the frame that just ended -- provably where it belongs
                         * (blanking is ~3.5ms; the next chunk's EOF cannot occur
                         * until 350us of active video after blanking ends, so an
                         * EOF pending at the frame-end VSYNC is this frame's
                         * tail and nothing else's). Full rationale at the top of
                         * this VSYNC branch. */
                        if (!cam_obj->psram_mode && !cam_obj->jpeg_mode) {
                            cam_event_t pend;
                            while (frame_buffer_event->len + pixels_per_dma <= cam_obj->fb_size &&
                                   xQueueReceive(cam_obj->event_queue, (void *)&pend, 0) == pdTRUE) {
                                if (pend != CAM_IN_SUC_EOF_EVENT) {
                                    /* a real boundary -- never lose it. The queue
                                     * always has room: we just took an item. */
                                    xQueueSendToFront(cam_obj->event_queue, (void *)&pend, 0);
                                    break;
                                }
                                frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                                    &frame_buffer_event->buf[frame_buffer_event->len],
                                    &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                                    cam_obj->dma_half_buffer_size);
                                if (cam_patch_chunk_cb) {
                                    cam_patch_chunk_cb(frame_buffer_event->buf,
                                                       frame_buffer_event->len, false);
                                }
                                cnt++;
                                cam_patch_eof_drained++;
                            }
                        }
#endif

                        if (cam_obj->psram_mode) {
                            if (cam_obj->jpeg_mode) {
                                frame_buffer_event->len = cnt * cam_obj->dma_half_buffer_size;
                            } else {
                                frame_buffer_event->len = cam_obj->recv_size;
                            }
                        } else if (!cam_obj->jpeg_mode) {
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                            /* v16 INSTRUMENT: record how many DMA chunks this
                             * frame actually delivered, BEFORE any gate decides
                             * its fate. This is the datum the chunk gate has
                             * been throwing away since v12.2. */
                            cam_patch_cnt_hist[(cnt < 0) ? 0
                                              : (((uint32_t)cnt < 15u) ? (uint32_t)cnt : 15u)]++;
#endif
                            /* v12.2: chunk-count alignment gate (see above).
                             * v15: gated behind cam_patch_gate_en (default 0 =
                             * lab-identical). Counting still runs so the
                             * telemetry keeps working while the gate is off. */
                            if (!cam_patch_gate_en) {
                                if (cam_patch_chunks_expected == 0 &&
                                    frame_buffer_event->len == cam_obj->fb_size)
                                    cam_patch_chunks_expected = (uint32_t)cnt;
                                else if (cam_patch_chunks_expected &&
                                         (uint32_t)cnt != cam_patch_chunks_expected)
                                    cam_patch_chunk_rej++;   /* observed, not acted on */
                            } else if (cam_patch_chunks_expected == 0) {
                                if (frame_buffer_event->len == cam_obj->fb_size)
                                    cam_patch_chunks_expected = (uint32_t)cnt;
                            } else if ((uint32_t)cnt != cam_patch_chunks_expected) {
                                cam_obj->frames[patch_done_pos].en = 1;
                                cam_patch_chunk_rej++;
                                static uint32_t s_ck = 0;
                                if ((s_ck++ & 0x3F) == 0)
                                    ESP_LOGW(TAG, "chunk-count %d != %u, frame "
                                             "rejected (x%u)", cnt,
                                             (unsigned)cam_patch_chunks_expected,
                                             (unsigned)cam_patch_chunk_rej);
                            }
                            if (s_lightgun_overrun && !cam_patch_gate_en) {
                                cam_patch_stitch_rej++;      /* observed, not acted on */
                                s_lightgun_overrun = false;
                            }
                            if (s_lightgun_overrun) {
                                /* v9.1: stitched frame (see FB-OVF above) —
                                 * reject regardless of its (full!) length. */
                                cam_obj->frames[patch_done_pos].en = 1;
                                cam_patch_stitch_rej++;
                                static uint32_t s_st_cnt = 0;
                                if ((s_st_cnt++ & 0x3F) == 0)
                                    ESP_LOGW(TAG, "stitched frame rejected (x%u)",
                                             (unsigned)cam_patch_stitch_rej);
                            } else
                            if (frame_buffer_event->len != cam_obj->fb_size) {
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                                /* LIGHTGUN PATCH: at high fps the classic ESP32
                                 * loses a few DMA chunks per frame and stock code
                                 * discards the whole (95%-complete) frame. Accept
                                 * frames >= 90% complete; fb->len carries the real
                                 * byte count so the app can scan len/width rows.
                                 * Frames below 90% are still rejected. */
                                if ((uint64_t)frame_buffer_event->len * 10 >= (uint64_t)cam_obj->fb_size * 9) {
                                    static uint32_t s_short_cnt = 0;
                                    if ((s_short_cnt++ & 0xFF) == 0) {
                                        ESP_LOGW(TAG, "short frame accepted: %u/%u (x%u)",
                                                 frame_buffer_event->len, (unsigned) cam_obj->fb_size,
                                                 (unsigned) s_short_cnt);
                                    }
                                } else {
                                    cam_obj->frames[patch_done_pos].en = 1;
                                    cam_patch_rej_size++;                 /* v15.5 */
                                    static uint32_t s_rej_cnt = 0;
                                    if ((s_rej_cnt++ & 0x3F) == 0) {
                                        ESP_LOGE(TAG, "FB-SIZE: %u != %u (rejected x%u)",
                                                 frame_buffer_event->len, (unsigned) cam_obj->fb_size,
                                                 (unsigned) s_rej_cnt);
                                    }
                                }
#else
                                cam_obj->frames[patch_done_pos].en = 1;
                                ESP_LOGE(TAG, "FB-SIZE: %u != %u", frame_buffer_event->len, (unsigned) cam_obj->fb_size);
#endif
                            }
                            s_lightgun_overrun = false;   /* v9.1: consumed */
                        }
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                        /* !psram_mode is REQUIRED here: in psram_mode the buffer
                         * still holds raw YUV422 (2 bytes/px, len = 2x the
                         * grayscale size) — conversion happens later in
                         * cam_take(). Feeding that to a 1-byte/px scanner reads
                         * chroma (~127) as luma and produces garbage. The
                         * per-chunk call site above is already inside a
                         * !psram_mode branch; this one was not. On S3,
                         * psram_mode is enabled whenever xclk == 16MHz. */
                        if (cam_patch_chunk_cb && !cam_obj->jpeg_mode && !cam_obj->psram_mode
                            && !cam_obj->frames[patch_done_pos].en) {
                            patch_done_buf = frame_buffer_event->buf;   /* deliver later */
                            patch_done_len = frame_buffer_event->len;
                        }
#endif
                        //send frame
                        if(!cam_obj->frames[patch_done_pos].en && xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                            //pop frame buffer from the queue
                            camera_fb_t * fb2 = NULL;
                            if(xQueueReceive(cam_obj->frame_buffer_queue, &fb2, 0) == pdTRUE) {
                                //push the new frame to the end of the queue
                                if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                                    cam_obj->frames[patch_done_pos].en = 1;
                                    cam_patch_rej_queue++;                /* v15.5 */
                                    ESP_LOGE(TAG, "FBQ-SND");
                                }
                                //free the popped buffer
                                cam_give(fb2);
                            } else {
                                //queue is full and we could not pop a frame from it
                                cam_obj->frames[patch_done_pos].en = 1;
                                cam_patch_rej_queue++;                    /* v15.5 */
                                ESP_LOGE(TAG, "FBQ-RCV");
                            }
                        }
                        /* v15.3: bucket this frame, after EVERY path that can
                         * reject it (size, stitch, chunk-count, queue-full). */
                        if (cam_obj->frames[patch_done_pos].en) cam_patch_frames_rejected++;
                        else                                    cam_patch_frames_delivered++;
                    }

                    if (!patch_restarted) {          /* frame-complete path skipped */
                        patch_started = cam_start_frame(&frame_pos);
                        uint32_t dt = (uint32_t)(esp_timer_get_time() - patch_vsync_us);
                        if (dt > cam_patch_restart_us) cam_patch_restart_us = dt;
                        if (patch_started) cam_obj->frames[frame_pos].fb.len = 0;
                    }
                    if(!patch_started){
                        cam_obj->state = CAM_STATE_IDLE;
                    }
#ifdef PATCH_ACCEPT_SHORT_FRAMES
                    /* Capture is running again — safe to spend time here now.
                     * ORDER MATTERS: finish the OLD frame before announcing the
                     * NEW one, or the app resets its row cursor first and then
                     * "finishes" an empty scan. */
                    if (patch_done_buf && cam_patch_chunk_cb) {
                        cam_patch_chunk_cb(patch_done_buf, patch_done_len, true);
                    }
                    if (patch_started && cam_patch_chunk_cb
                        && !cam_obj->jpeg_mode && !cam_obj->psram_mode) {
                        cam_patch_chunk_cb(cam_obj->frames[frame_pos].fb.buf, 0, false);
                    }
#endif
                    cnt = 0;
                }
            }
            break;
        }
        DBG_PIN_SET(0);
    }
}

static lldesc_t * allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t * buffer)
{
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (int x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    return dma;
}

static esp_err_t cam_dma_config(const camera_config_t *config)
{
    bool ret = ll_cam_dma_sizes(cam_obj);
    if (0 == ret) {
        return ESP_FAIL;
    }

    cam_obj->dma_node_cnt = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size; // Number of DMA nodes
    cam_obj->frame_copy_cnt = cam_obj->recv_size / cam_obj->dma_half_buffer_size; // Number of interrupted copies, ping-pong copy

    ESP_LOGI(TAG, "buffer_size: %d, half_buffer_size: %d, node_buffer_size: %d, node_cnt: %d, total_cnt: %d",
             (int) cam_obj->dma_buffer_size, (int) cam_obj->dma_half_buffer_size, (int) cam_obj->dma_node_buffer_size,
             (int) cam_obj->dma_node_cnt, (int) cam_obj->frame_copy_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    cam_obj->frames = (cam_frame_t *)heap_caps_aligned_calloc(alignof(cam_frame_t), 1, cam_obj->frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(cam_obj->frames != NULL, "frames malloc failed", ESP_FAIL);

    uint8_t dma_align = 0;
    size_t fb_size = cam_obj->fb_size;
    if (cam_obj->psram_mode) {
        dma_align = ll_cam_get_dma_align(cam_obj);
        if (cam_obj->fb_size < cam_obj->recv_size) {
            fb_size = cam_obj->recv_size;
        }
    }

    /* Allocate memory for frame buffer */
    size_t alloc_size = fb_size * sizeof(uint8_t) + dma_align;
    uint32_t _caps = MALLOC_CAP_8BIT;
    if (CAMERA_FB_IN_DRAM == config->fb_location) {
        _caps |= MALLOC_CAP_INTERNAL;
    } else {
        _caps |= MALLOC_CAP_SPIRAM;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].dma = NULL;
        cam_obj->frames[x].fb_offset = 0;
        cam_obj->frames[x].en = 0;
        ESP_LOGI(TAG, "Allocating %d Byte frame buffer in %s", alloc_size, _caps & MALLOC_CAP_SPIRAM ? "PSRAM" : "OnBoard RAM");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        // In IDF v4.2 and earlier, memory returned by heap_caps_aligned_alloc must be freed using heap_caps_aligned_free.
        // And heap_caps_aligned_free is deprecated on v4.3.
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_aligned_alloc(16, alloc_size, _caps);
#else
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_malloc(alloc_size, _caps);
#endif
        CAM_CHECK(cam_obj->frames[x].fb.buf != NULL, "frame buffer malloc failed", ESP_FAIL);
        if (cam_obj->psram_mode) {
            //align PSRAM buffer. TODO: save the offset so proper address can be freed later
            cam_obj->frames[x].fb_offset = dma_align - ((uint32_t)cam_obj->frames[x].fb.buf & (dma_align - 1));
            cam_obj->frames[x].fb.buf += cam_obj->frames[x].fb_offset;
            ESP_LOGI(TAG, "Frame[%d]: Offset: %u, Addr: 0x%08X", x, cam_obj->frames[x].fb_offset, (unsigned) cam_obj->frames[x].fb.buf);
            cam_obj->frames[x].dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->frames[x].fb.buf);
            CAM_CHECK(cam_obj->frames[x].dma != NULL, "frame dma malloc failed", ESP_FAIL);
        }
        cam_obj->frames[x].en = 1;
    }

    if (!cam_obj->psram_mode) {
        cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
        if(NULL == cam_obj->dma_buffer) {
            ESP_LOGE(TAG,"%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
                     (int) cam_obj->dma_buffer_size, (int) heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return ESP_FAIL;
        }

        cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer);
        CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);
    }

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    cam_obj->swap_data = 0;
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = true;

    ll_cam_set_pin(cam_obj, config);
    ret = ll_cam_config(cam_obj, config);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam initialize failed", err);

#if CAMERA_DBG_PIN_ENABLE
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[DBG_PIN_NUM], PIN_FUNC_GPIO);
    gpio_set_direction(DBG_PIN_NUM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DBG_PIN_NUM, GPIO_FLOATING);
#endif

    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return ESP_FAIL;
}

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    ret = ll_cam_set_sample_mode(cam_obj, (pixformat_t)config->pixel_format, config->xclk_freq_hz, sensor_pid);

    cam_obj->jpeg_mode = config->pixel_format == PIXFORMAT_JPEG;
#if CONFIG_IDF_TARGET_ESP32
    cam_obj->psram_mode = false;
#else
    cam_obj->psram_mode = (config->xclk_freq_hz == 16000000);
#endif
    cam_obj->frame_cnt = config->fb_count;
    cam_obj->width = resolution[frame_size].width;
    cam_obj->height = resolution[frame_size].height;

    if(cam_obj->jpeg_mode){
#ifdef CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_AUTO
        cam_obj->recv_size = cam_obj->width * cam_obj->height / 5;
#else
        cam_obj->recv_size = CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE;
#endif
        cam_obj->fb_size = cam_obj->recv_size;
    } else {
        cam_obj->recv_size = cam_obj->width * cam_obj->height * cam_obj->in_bytes_per_pixel;
        cam_obj->fb_size = cam_obj->width * cam_obj->height * cam_obj->fb_bytes_per_pixel;
    }

    ret = cam_dma_config(config);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    size_t queue_size = cam_obj->dma_half_buffer_cnt - 1;
    if (queue_size == 0) {
        queue_size = 1;
    }
    cam_obj->event_queue = xQueueCreate(queue_size, sizeof(cam_event_t));
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);
#ifdef PATCH_ACCEPT_SHORT_FRAMES
    /* v66: publish the ring geometry. queue_size is NOT a free parameter — it is
     * one less than the number of DMA half-buffers, so that an overflow happens
     * exactly when cam_task has fallen far enough behind that the oldest DMA
     * buffer is about to be overwritten underneath it. Enlarging it in isolation
     * converts a visible dropped frame into invisible torn rows. If ovf_eof is
     * nonzero the answer is to make cam_task faster (or make the half-buffers
     * smaller so there are more of them), never to pad this queue. */
    cam_patch_evq_depth = (uint32_t) queue_size;
    cam_patch_dma_halfs = (uint32_t) cam_obj->dma_half_buffer_cnt;
    /* Output LINES per DMA chunk. Must divide by dma_bytes_per_item as well:
     * on the ESP32 classic the I2S path inflates every sample (2 or 4 bytes per
     * byte of pixel data), so dma_half_buffer_size is NOT in output bytes there.
     * Omitting it reported 2-4x too many lines on the classic while being right
     * on the S3, which is exactly the kind of target-dependent lie that sends
     * you hunting a phantom. */
    {
        uint32_t denom = (uint32_t)(cam_obj->width * cam_obj->in_bytes_per_pixel)
                       * (cam_obj->dma_bytes_per_item ? cam_obj->dma_bytes_per_item : 1);
        cam_patch_dma_lines = denom ? (cam_obj->dma_half_buffer_size / denom) : 0;
    }
    ESP_LOGW(TAG, "RING: evq=%u dma_halfs=%u half=%u B (%u lines) copies/frame=%u",
             (unsigned) cam_patch_evq_depth, (unsigned) cam_patch_dma_halfs,
             (unsigned) cam_obj->dma_half_buffer_size,
             (unsigned) cam_patch_dma_lines, (unsigned) cam_obj->frame_copy_cnt);
#endif

    size_t frame_buffer_queue_len = cam_obj->frame_cnt;
    if (config->grab_mode == CAMERA_GRAB_LATEST && cam_obj->frame_cnt > 1) {
        frame_buffer_queue_len = cam_obj->frame_cnt - 1;
    }
    cam_obj->frame_buffer_queue = xQueueCreate(frame_buffer_queue_len, sizeof(camera_fb_t*));
    CAM_CHECK_GOTO(cam_obj->frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    ret = ll_cam_init_isr(cam_obj);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);


#if defined(PATCH_CAM_TASK_CORE1)
    /* LIGHTGUN PATCH: move cam_task off core 0 so WiFi interrupts stop starving
     * it (EV-VSYNC-OVF / lost DMA chunks). Remove -DPATCH_CAM_TASK_CORE1 from
     * platformio.ini to compare. */
    /* also: 4096 stack — the 2048 default overflows with the plain-IDF log path */
    xTaskCreatePinnedToCore(cam_task, "cam_task", (CAM_TASK_STACK) < 4096 ? 4096 : (CAM_TASK_STACK), NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 1);
#elif CONFIG_CAMERA_CORE0
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 0);
#elif CONFIG_CAMERA_CORE1
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 1);
#else
    xTaskCreate(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle);
#endif

    ESP_LOGI(TAG, "cam config ok");
#ifdef PATCH_ACCEPT_SHORT_FRAMES
    /* Proof-of-linkage banner: if you don't see this at init, the stock
     * prebuilt driver got linked instead of this patched fork. */
    ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: LIGHTGUN PATCH ACTIVE (short-frame tolerant, core1, 4k stack)\r\n"));
#endif
    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void)
{
    if (!cam_obj) {
        return ESP_FAIL;
    }

    cam_stop();
    if (cam_obj->task_handle) {
        vTaskDelete(cam_obj->task_handle);
    }
    if (cam_obj->event_queue) {
        vQueueDelete(cam_obj->event_queue);
    }
    if (cam_obj->frame_buffer_queue) {
        vQueueDelete(cam_obj->frame_buffer_queue);
    }

    ll_cam_deinit(cam_obj);

    if (cam_obj->dma) {
        free(cam_obj->dma);
    }
    if (cam_obj->dma_buffer) {
        free(cam_obj->dma_buffer);
    }
    if (cam_obj->frames) {
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            free(cam_obj->frames[x].fb.buf - cam_obj->frames[x].fb_offset);
            if (cam_obj->frames[x].dma) {
                free(cam_obj->frames[x].dma);
            }
        }
        free(cam_obj->frames);
    }

    free(cam_obj);
    cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void)
{
    ll_cam_vsync_intr_enable(cam_obj, false);
    ll_cam_stop(cam_obj);
}

void cam_start(void)
{
    ll_cam_vsync_intr_enable(cam_obj, true);
}

camera_fb_t *cam_take(TickType_t timeout)
{
    camera_fb_t *dma_buffer = NULL;
    TickType_t start = xTaskGetTickCount();
    xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, timeout);
#if CONFIG_IDF_TARGET_ESP32S3
    // Currently (22.01.2024) there is a bug in ESP-IDF v5.2, that causes
    // GDMA to fall into a strange state if it is running while WiFi STA is connecting.
    // This code tries to reset GDMA if frame is not received, to try and help with
    // this case. It is possible to have some side effects too, though none come to mind
    if (!dma_buffer) {
        ll_cam_dma_reset(cam_obj);
        xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, timeout);
    }
#endif
    if (dma_buffer) {
        if(cam_obj->jpeg_mode){
            // find the end marker for JPEG. Data after that can be discarded
            int offset_e = cam_verify_jpeg_eoi(dma_buffer->buf, dma_buffer->len);
            if (offset_e >= 0) {
                // adjust buffer length
                dma_buffer->len = offset_e + sizeof(JPEG_EOI_MARKER);
                return dma_buffer;
            } else {
                ESP_LOGW(TAG, "NO-EOI");
                cam_give(dma_buffer);
                TickType_t ticks_spent = xTaskGetTickCount() - start;
                if (ticks_spent >= timeout) {
                    return NULL; /* We are out of time */
                }
                return cam_take(timeout - ticks_spent);//recurse!!!!
            }
        } else if(cam_obj->psram_mode && cam_obj->in_bytes_per_pixel != cam_obj->fb_bytes_per_pixel){
            //currently this is used only for YUV to GRAYSCALE
            dma_buffer->len = ll_cam_memcpy(cam_obj, dma_buffer->buf, dma_buffer->buf, dma_buffer->len);
        }
        return dma_buffer;
    } else {
        ESP_LOGW(TAG, "Failed to get the frame on time!");
// #if CONFIG_IDF_TARGET_ESP32S3
//         ll_cam_dma_print_state(cam_obj);
// #endif
    }
    return NULL;
}

void cam_give(camera_fb_t *dma_buffer)
{
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        if (&cam_obj->frames[x].fb == dma_buffer) {
            cam_obj->frames[x].en = 1;
            break;
        }
    }
}

void cam_give_all(void) {
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].en = 1;
    }
}


/* ==== LIGHTGUN: IDF5-API SHIMS ============================================
 * esp_camera.c (IDF5 generation) calls these three. Our capture path is the
 * IDF4-lineage one with framebuffers pinned in DRAM, so PSRAM-DMA mode is
 * hardwired off: set is accepted-and-ignored (logged once), get returns false.
 * cam_get_available_frames is a straight transcription of upstream. */
bool cam_get_available_frames(void)
{
    return cam_obj && 0 < uxQueueMessagesWaiting(cam_obj->frame_buffer_queue);
}

void cam_set_psram_mode(bool enable)
{
    if (enable) {
        ESP_LOGW(TAG, "psram DMA mode not supported by the lightgun fork (DRAM fb by design)");
    }
}

bool cam_get_psram_mode(void)
{
    return false;
}
