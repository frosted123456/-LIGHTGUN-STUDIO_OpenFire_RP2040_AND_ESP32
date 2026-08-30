// Recoil engine glue implementation. The engine says which PHASE the outputs
// are in; this file turns phases into pin writes -- full drive as a plain
// high, the hold as 20 kHz PWM at the tuned duty, rumble at the profile's
// strength. Writes happen on phase CHANGES only, so polling is nearly free.
#include "recoil_fx.h"
#include "recoil_fx_glue.h"

#if defined(ARDUINO_ARCH_RP2040) && defined(LIGHTGUN_RECOIL_FX)
#include <Arduino.h>

static int s_sol = -1, s_rum = -1, s_strength = 255;
static int s_lsol = -99, s_lrum = -99;
static unsigned long s_last_fire_ms = 0;

static uint64_t now_us(void) { return time_us_64(); }

void fx_glue_begin(int sol_pin, int rum_pin, int rum_strength)
{
    s_sol = sol_pin;
    s_rum = rum_pin;
    s_strength = rum_strength;
    fx_params_t d;
    fx_defaults(&d);
    fx_init(&d, (uint32_t)now_us() | 1u);
    fx_load();
    // 20 kHz keeps the hold PWM above audibility. analogWriteFreq is global
    // on this core; the rumble motor does not care about carrier frequency.
    analogWriteFreq(20000);
}

// Both answers come from the engine's own policy (fx_owns_*), so quiet mode
// and the on-switch are decided in one host-tested place. Quiet mode makes
// BOTH true even with the engine dormant: that is what silences the stock
// paths, which is the half a "turn every knob down" trick could never do.
//
// Quiet mode is asserted WITHOUT the solenoid-pin test the engine needs. On a
// rumble-only gun (no solenoid pin, OpenFIRE's rumble-as-recoil mode) that
// test made quiet mode a complete no-op while the gun still answered
// "quiet=1" -- so the tools reported silence while the motor shook the camera
// through the whole calibration.
int fx_glue_on(void)
{
    const uint64_t t = now_us();
    return fx_quiet_active(t) || (s_sol >= 0 && fx_owns_outputs(t));
}
int fx_glue_dryfire(void) { return fx_ab_active(now_us()); }
// When the engine's rumble co-fire is in use, the stock rumble paths must not
// touch the motor pin -- a release handler forcing it LOW mid-window read as
// "rumble randomly cuts out".
int fx_glue_owns_rumble(void)
{
    const uint64_t t = now_us();
    return fx_quiet_active(t) || (s_sol >= 0 && fx_owns_rumble(t));
}

static unsigned long s_press_ms = 0;

// Edge-triggered firing. The feedback hook runs EVERY loop while the trigger
// is held; firing from each call made a refused pull go off LATE (the moment
// the engine freed, mid-hold) and autofired regardless of the autofire
// toggle. The edge fires; the hold refires only with autofire on, and only
// after auto_ms of hold, so quick semi pulls can never double.
int fx_glue_trigger(int first_press, int autofire_on, int temp_state)
{
    if (!fx_glue_on()) return 0;
    fx_temp_note(temp_state);
    if (first_press) {
        // The press EDGE fires regardless of temperature -- the same policy
        // as stock OpenFIRE's first shot. A disconnected or garbage TMP36
        // reads as Fatal, and blocking edge shots on it turned an open gun on
        // a bench into one that silently stopped firing. Sustained fire is
        // where the heat is, and that is what the gates below still guard.
        s_press_ms = millis();
        // A pull edge preempts: a fast second pull cuts the playing tail and
        // re-strikes as soon as the armature can land, instead of vanishing.
        if (fx_fire_preempt(now_us())) { s_last_fire_ms = millis(); return 1; }
        return 0;
    }
    if (!autofire_on) return 0;
    if (millis() - s_press_ms < (unsigned long)fx_get()->auto_ms) return 0;
    return fx_glue_fire(temp_state);
}

int fx_glue_fire(int temp_state)
{
    if (!fx_glue_on()) return 0;
    // Fatal temperature refuses NEW fires but does not cancel a playing
    // shot: the edge fired by policy, and autofire cancelling it mid-play
    // contradicted that. The whole engine is silenced by FFBShutdown paths.
    if (temp_state >= 2) return 0;
    if (temp_state == 1 && millis() - s_last_fire_ms < 500)
        return 0;                                     // warning: forced cooldown
    if (fx_fire(now_us())) {
        s_last_fire_ms = millis();
        return 1;
    }
    return 0;
}

void fx_glue_poll(void)
{
    const uint64_t t = now_us();
    // Quiet mode: hold BOTH pins down on every poll, not just when the phase
    // changes. While it is on, this is the only code left that may touch them
    // -- every stock path that would normally release them is skipped -- and
    // on a dual-core build those stock paths run on the OTHER core. A rumble
    // write landing just after a single change-detected write would then stay
    // high for the whole calibration, with nothing left to take it down.
    // Re-asserting costs a couple of register writes on a core that is idle,
    // and only while a measurement is actually running.
    // Each pin guarded on its own: a rumble-only gun has no solenoid pin, and
    // its motor is exactly the output quiet mode is there to stop.
    if (fx_quiet_active(t)) {
        if (s_sol >= 0) { analogWrite(s_sol, 0); digitalWrite(s_sol, LOW); }
        if (s_rum >= 0) { analogWrite(s_rum, 0); digitalWrite(s_rum, LOW); }
        s_lsol = s_lrum = -99;     // force a real write when quiet ends
        return;
    }
    if (s_sol < 0) return;
    int sol, rum;
    fx_step(t, &sol, &rum);
    if (sol != s_lsol) {
        if (sol == FX_SOL_FULL)      digitalWrite(s_sol, HIGH);
        else if (sol == FX_SOL_HOLD) analogWrite(s_sol,
                                         (fx_get()->duty_pct * 255) / 100);
        else { analogWrite(s_sol, 0); digitalWrite(s_sol, LOW); }
        s_lsol = sol;
    }
    if (rum != s_lrum) {
        if (s_rum >= 0) {
            if (rum) analogWrite(s_rum, s_strength);
            else   { analogWrite(s_rum, 0); digitalWrite(s_rum, LOW); }
        }
        s_lrum = rum;
    }
}

void fx_glue_shutdown(void)
{
    fx_cancel();
    s_lsol = s_lrum = -99;
    if (s_sol >= 0) { analogWrite(s_sol, 0); digitalWrite(s_sol, LOW); }
    if (s_rum >= 0) { analogWrite(s_rum, 0); digitalWrite(s_rum, LOW); }
}

#else  // host build, other boards, or the feature compiled out: inert stubs

void fx_glue_begin(int sol_pin, int rum_pin, int rum_strength)
{ (void)sol_pin; (void)rum_pin; (void)rum_strength; }
void fx_glue_poll(void) {}
int  fx_glue_trigger(int first_press, int autofire_on, int temp_state)
{ (void)first_press; (void)autofire_on; (void)temp_state; return 0; }
int  fx_glue_fire(int temp_state) { (void)temp_state; return 0; }
int  fx_glue_on(void) { return 0; }
int  fx_glue_dryfire(void) { return 0; }
int  fx_glue_owns_rumble(void) { return 0; }
void fx_glue_shutdown(void) {}

#endif
