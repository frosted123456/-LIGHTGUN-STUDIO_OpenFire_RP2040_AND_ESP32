// wiicam_aim.h -- RP2040 + wiicam (SEN0158) front end for the aim pipeline.
// Masks the driver's seen flags (unseen slots retain stale coordinates),
// normalises into the pipeline's 240x176 space, applies the stored lens
// correction, runs the quad resolver, applies latency lead, and emits the
// same Q/T telemetry the desktop tools use.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Native wiicam report space.
#define WIICAM_W 1024.0f
#define WIICAM_H 768.0f
// The pipeline's native space (what the resolver gates and tools assume).
#define WIICAM_NORM_W 240.0f
#define WIICAM_NORM_H 176.0f

void wiicam_aim_begin(void);

// One sensor poll. px/py are the driver's positionX/Y arrays (native units),
// seen is the driver's seenFlags. Returns true and writes sx,sy (normalised
// screen 0..1) when the calibrated pipeline produced a position; false means
// fall through to stock OpenFIRE handling.
bool wiicam_aim_process(const int* px, const int* py, unsigned seen,
                        uint64_t now_us, float* sx, float* sy);

// The '~cam...' command subset for this board: cam? / camsave / camreset /
// cam=res:..,dash:..,dashhz:..,lead:..,lens:..,lk1u:..,lk2u:..,lfpx:..,
// lfeq:..,sens:..  Returns true if the line was handled.
bool wiicam_cam_command(const char* line);

// Telemetry (Q/T lines) and replies, same split as the ESP32 build: telemetry
// may be dropped when the host is not draining, replies may block briefly.
void wiicam_set_line_sink(void (*fn)(const char*));
void wiicam_set_reply_sink(void (*fn)(const char*));
void wiicam_set_diag_hook(int (*fn)(void));   // "~camdiag" runs it

// Sensitivity is OpenFIRE's own persisted setting; the patch wires these to
// FW_Common so '~cam=sens:n' goes through the same path as their UI.
void wiicam_set_sens_hooks(void (*set_fn)(int), int (*get_fn)(void),
                           void (*save_fn)(void));

#ifdef __cplusplus
}
#endif
