// ov2640_capture.h — capture core: camera init plus in-driver blob detection,
// publishing into ov2640_bridge. Started idempotently from the shim's begin().
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// Routes telemetry lines (B/Q/D/STAT) to a custom sink; pass 0 for printf.
void ov2640_set_line_sink(void (*fn)(const char*));

// Sink for command replies, as opposed to per-frame telemetry.
void ov2640_set_reply_sink(void (*fn)(const char*));

// Parses a "cam=thr:60,aec:40" line; returns false if not a camera command.
// Must be declared here inside extern "C", not forward-declared at call sites.
bool ov2640_cam_command(const char* line);
// Returns 0 on success (or if already started). Safe to call repeatedly.
int ov2640_capture_start(void);

// Frame-gate statistic for diagnostics; driver-side rejections are counted
// separately by the driver's own cam_patch_* counters.
extern volatile uint32_t ov2640_stat_rej_size;    // byte count != full frame

// Live tuning, e.g. "thr=90&boost=0&aec=40"; applied immediately.
// Diagnostic builds only: compiled out when LIGHTGUN_DIAG is 0.
#if !defined(LIGHTGUN_DIAG) || LIGHTGUN_DIAG
void ov2640_tune(const char* cmd);
#endif
#ifdef __cplusplus
}
#endif
