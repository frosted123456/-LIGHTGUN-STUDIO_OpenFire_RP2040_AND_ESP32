// Recoil effect engine: turns one trigger event into a solenoid + rumble
// timeline. Pure timing logic -- no pins, no PWM, no Arduino -- so the exact
// waveform is host-testable against a fake clock. The caller polls fx_step()
// every loop and drives the hardware from its outputs; nothing here blocks.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Solenoid output states. FULL is a solid drive; HOLD means the caller runs
// its PWM at fx duty (the engine times the phase, the glue owns the PWM).
enum { FX_SOL_OFF = 0, FX_SOL_FULL = 1, FX_SOL_HOLD = 2 };

#define FX_MAX_PULSES 3
// How long the solenoid must be OFF before a fresh strike can land: the
// spring has to pull the armature back first. A hardware property of the
// solenoid, not a taste knob.
#define FX_RELATCH_MS 18
#define FX_AB_TIMEOUT_US (10ull * 60 * 1000000)   // dry-fire mode auto-expiry

typedef struct {
    int enabled;     // 0 = engine dormant, stock OpenFIRE behaviour (default)
    int drive_ms;    // initial full drive, 5..100 (matches OpenFIRE's own range)
    int hold_ms;     // PWM hold after the strike, 0..500
    int duty_pct;    // hold duty for the glue's PWM, 25..70
    int pulses;      // after-pulses following release, 0..3
    int gap_ms;      // off time before each after-pulse, 15..120
    int jit_pct;     // random stretch on hold and gaps only, 0..15
    int rum_off_ms;  // rumble start relative to the strike, -20..50
    int rum_ms;      // rumble run time, 0 (off) .. 200
    int space_ms;    // minimum quiet time between sequences, 0..500
    int auto_ms;     // trigger hold time before autofire engages, 0..1000
} fx_params_t;

void fx_defaults(fx_params_t* p);
int  fx_clamp(fx_params_t* p);                 // 1 if anything was out of range
void fx_init(const fx_params_t* p, uint32_t seed);
void fx_set(const fx_params_t* p);             // live retune; clamps internally
const fx_params_t* fx_get(void);

int  fx_fire(uint64_t now_us);                 // 1 = accepted, 0 = busy/off
int  fx_fire_forced(uint64_t now_us);          // test path: fires even dormant
// A deliberate pull DURING a sequence: cut the remainder and re-strike as
// soon as the armature can physically land again. Never refuses while on.
int  fx_fire_preempt(uint64_t now_us);
void fx_cancel(void);                          // kill a playing sequence dead
void fx_step(uint64_t now_us, int* sol, int* rumble);
int  fx_busy(uint64_t now_us);                 // sequence or spacing still open

// Dry-fire (A/B) mode: the caller lets the trigger reach fx_fire without an
// IR lock while this is on. It expires by itself so it cannot be left armed.
void fx_ab_set(int on, uint64_t now_us);
int  fx_ab_active(uint64_t now_us);
int  fx_ab_left_s(uint64_t now_us);   // seconds until dry-fire expires; 0 = off

#ifdef __cplusplus
}
#endif

// ---- serial command surface -----------------------------------------------
// "fx=k:v,k:v" tunes; "fx?" reports; "fxsave" persists. Same k:v shape and
// the same '~' gatekeeper channel as the cam commands, so the tools reuse
// their existing plumbing. Replies go to the installed sink (else stdout).
void fx_set_reply(void (*sink)(const char* line));
int  fx_command(const char* line, uint64_t now_us);   // 1 = claimed the line

// Persistence, one blob under the aim NVS namespace. Load returns 0 when
// nothing (or a foreign version) is stored; the caller keeps its defaults.
int  fx_store(void);
int  fx_load(void);

// Last temperature state the trigger path saw (-1 unknown, 0 safe, 1 warning,
// 2 fatal). Reported by fx?, so a disconnected TMP36 reads as a WORD on
// screen instead of as a gun that mysteriously stopped firing.
void fx_temp_note(int state);
int  fx_temp_get(void);
