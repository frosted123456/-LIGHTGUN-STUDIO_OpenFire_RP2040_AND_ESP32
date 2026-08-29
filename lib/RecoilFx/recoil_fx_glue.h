// Recoil engine glue for the RP2040 firmware build: pins, PWM and the clock.
// Everything here is a no-op unless LIGHTGUN_RECOIL_FX is defined AND the
// engine is switched on at runtime, so a build carrying this code behaves
// exactly like the build before it until asked not to.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// temp_state: 0 = safe, 1 = warning (extra cooldown), 2 = fatal (no fire).
void fx_glue_begin(int sol_pin, int rum_pin, int rum_strength);
void fx_glue_poll(void);
int  fx_glue_fire(int temp_state);   // 1 = a sequence started (serial test path)
// The trigger path: fires on the press EDGE; while held, refires only when
// the caller's autofire toggle is on AND the hold has lasted auto_ms.
int  fx_glue_trigger(int first_press, int autofire_on, int temp_state);
int  fx_glue_on(void);               // engine switched on at runtime
int  fx_glue_dryfire(void);          // dry-fire (A/B) mode active
int  fx_glue_owns_rumble(void);      // engine on AND rum_ms > 0
void fx_glue_shutdown(void);         // cancel + force both pins low

#ifdef __cplusplus
}
#endif
