// Recoil engine waveform test: walks a fake clock through fire sequences and
// asserts every phase edge -- strike, hold, gaps, after-pulses, rumble window,
// refire spacing, jitter bounds, dry-fire expiry. The engine is pure timing,
// so what passes here is exactly what the pins will do.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "recoil_fx.h"

static int fails = 0;
#define CK(cond, msg) do { \
    if (cond) printf("  [PASS] %s\n", msg); \
    else { printf("  [FAIL] %s\n", msg); ++fails; } \
} while (0)

// Samples the solenoid line over a window at 100 us steps and returns how many
// microseconds it spent in each state.
static void span(uint64_t t0, uint64_t t1, uint64_t* full, uint64_t* hold,
                 uint64_t* rum)
{
    *full = *hold = *rum = 0;
    for (uint64_t t = t0; t < t1; t += 100) {
        int s, r;
        fx_step(t, &s, &r);
        if (s == FX_SOL_FULL) *full += 100;
        if (s == FX_SOL_HOLD) *hold += 100;
        if (r) *rum += 100;
    }
}

int main()
{
    fx_params_t p;
    fx_defaults(&p);
    CK(!fx_clamp(&p), "the defaults are inside every range");
    CK(p.enabled == 0, "the engine ships dormant: a fresh flash changes nothing");
    fx_init(&p, 1);
    CK(fx_fire(1000) == 0, "a dormant engine refuses to fire");
    p.enabled = 1;   // every test below runs with the engine switched on

    // ---- clamping --------------------------------------------------------
    fx_params_t bad = p;
    bad.drive_ms = 999; bad.duty_pct = 1; bad.rum_off_ms = -500; bad.pulses = 9;
    CK(fx_clamp(&bad) == 1, "out-of-range values are reported");
    CK(bad.drive_ms == 100 && bad.duty_pct == 25 && bad.rum_off_ms == -20
       && bad.pulses == FX_MAX_PULSES, "and clamped to the documented limits");
    {
        // the stock waveform must be expressible: 45 ms drive, no hold
        fx_params_t st; fx_defaults(&st);
        st.enabled = 1; st.drive_ms = 45; st.hold_ms = 0;
        CK(!fx_clamp(&st) && st.drive_ms == 45,
           "a 45 ms stock-length strike survives the clamp untouched");
    }

    // ---- the basic strike + hold timeline, no jitter ---------------------
    fx_defaults(&p);                       // drive 15, hold 80, no pulses
    p.enabled = 1;
    fx_init(&p, 1);
    uint64_t t0 = 1000000;
    CK(fx_fire(t0) == 1, "an idle engine accepts a fire");
    int s, r;
    fx_step(t0, &s, &r);
    CK(s == FX_SOL_FULL, "the strike begins immediately");
    fx_step(t0 + 14990, &s, &r);
    CK(s == FX_SOL_FULL, "full drive runs the whole drive time");
    fx_step(t0 + 15010, &s, &r);
    CK(s == FX_SOL_HOLD, "then the hold phase takes over");
    fx_step(t0 + 15000 + 79990, &s, &r);
    CK(s == FX_SOL_HOLD, "and runs the whole hold time");
    fx_step(t0 + 15000 + 80010, &s, &r);
    CK(s == FX_SOL_OFF, "release is clean at the end of the hold");
    CK(fx_busy(t0 + 95000 + 79000), "the spacing keeps the engine busy after release");
    CK(!fx_busy(t0 + 95000 + 81000), "and frees it once the quiet time passes");

    // ---- refire rules ----------------------------------------------------
    fx_init(&p, 1);
    fx_fire(t0);
    CK(fx_fire(t0 + 20000) == 0, "a fire during the sequence is refused");
    CK(fx_fire(t0 + 95000 + 40000) == 0, "a fire inside the spacing is refused");
    CK(fx_fire(t0 + 95000 + 81000) == 1, "and accepted after the spacing");

    // ---- after-pulses ----------------------------------------------------
    fx_defaults(&p);
    p.enabled = 1; p.hold_ms = 0; p.pulses = 2; p.gap_ms = 40;
    fx_init(&p, 1);
    fx_fire(t0);
    uint64_t full, hold, rum;
    span(t0, t0 + 200000, &full, &hold, &rum);
    CK(full > 3 * 14000 && full < 3 * 16000,
       "three full-drive windows: the strike and two after-pulses");
    CK(hold == 0, "after-pulses carry no hold phase");
    fx_step(t0 + 15000 + 20000, &s, &r);
    CK(s == FX_SOL_OFF, "the gap between pulses is genuinely off");
    fx_step(t0 + 15000 + 40000 + 7000, &s, &r);
    CK(s == FX_SOL_FULL, "the first after-pulse lands one gap after release");

    // ---- rumble co-fire, both polarities ---------------------------------
    fx_defaults(&p);
    p.enabled = 1; p.rum_ms = 60; p.rum_off_ms = 20;
    fx_init(&p, 1);
    fx_fire(t0);
    fx_step(t0 + 10000, &s, &r);
    CK(s == FX_SOL_FULL && r == 0, "positive offset: the strike leads");
    fx_step(t0 + 30000, &s, &r);
    CK(r == 1, "and the rumble follows at the offset");
    p.rum_off_ms = -20;
    fx_init(&p, 1);
    fx_fire(t0);
    fx_step(t0 + 5000, &s, &r);
    CK(s == FX_SOL_OFF && r == 1, "negative offset: the rumble leads");
    fx_step(t0 + 25000, &s, &r);
    CK(s == FX_SOL_FULL, "and the strike waits out the offset");
    // rumble may outlast the solenoid; the sequence must not end early
    p.rum_off_ms = 50; p.rum_ms = 200; p.hold_ms = 0;
    fx_init(&p, 1);
    fx_fire(t0);
    fx_step(t0 + 200000, &s, &r);
    CK(r == 1, "a long rumble keeps running after the solenoid is done");
    // and it must NOT block the next shot: only the solenoid timeline does
    CK(fx_fire(t0 + 150000) == 1,
       "a refire during the rumble tail is accepted once the solenoid is free");
    fx_step(t0 + 150000 + 60000, &s, &r);
    CK(r == 1, "and the new shot restarts the rumble window");

    // ---- jitter: bounded, varied, deterministic, strike exempt -----------
    fx_defaults(&p);
    p.enabled = 1; p.jit_pct = 15; p.hold_ms = 100;
    fx_init(&p, 42);
    uint64_t holds[40];
    uint64_t t = 1000000;
    int varied = 0;
    for (int i = 0; i < 40; ++i) {
        fx_fire(t);
        span(t, t + 400000, &full, &hold, &rum);
        if (full < 14000 || full > 16000) {
            CK(false, "jitter must never touch the strike");
            goto done_jit;
        }
        holds[i] = hold;
        if (hold < 84000 || hold > 116000) {
            CK(false, "a hold escaped the +/-15 percent band");
            goto done_jit;
        }
        if (i > 0 && holds[i] != holds[0]) varied = 1;
        t += 800000;
    }
    CK(true, "40 jittered fires: strike untouched, holds inside the band");
    CK(varied, "and the holds actually vary");
    {
        // same seed, same sequence: the tuning is reproducible
        fx_init(&p, 42);
        fx_fire(1000000);
        span(1000000, 1400000, &full, &hold, &rum);
        CK(hold == holds[0], "the jitter stream is deterministic from the seed");
    }
done_jit:;

    // ---- a retune must change the NEXT shot, never the playing one -------
    fx_defaults(&p);
    p.enabled = 1; p.hold_ms = 80;
    fx_init(&p, 1);
    fx_fire(t0);
    fx_params_t q = p;
    q.drive_ms = 25; q.hold_ms = 0;
    fx_set(&q);                            // retuned mid-sequence
    fx_step(t0 + 20000, &s, &r);
    CK(s == FX_SOL_HOLD, "the playing shot keeps its own strike and hold");
    fx_step(t0 + 16000, &s, &r);
    CK(s != FX_SOL_FULL, "and the strike is not stretched by the new drive");

    // ---- a disabled rumble must not delay the strike ---------------------
    fx_defaults(&p);
    p.enabled = 1; p.rum_off_ms = -20; p.rum_ms = 0;
    fx_init(&p, 1);
    fx_fire(t0);
    fx_step(t0 + 1000, &s, &r);
    CK(s == FX_SOL_FULL && r == 0,
       "rumble off: a negative offset is ignored and the strike is immediate");

    // ---- dry-fire (A/B) mode expiry --------------------------------------
    fx_ab_set(1, t0);
    CK(fx_ab_active(t0 + 1000000), "dry-fire mode is on after arming");
    CK(fx_ab_left_s(t0 + 60000000) == 540,
       "and reports its remaining time in seconds");
    CK(fx_ab_left_s(t0 + FX_AB_TIMEOUT_US + 1) == 0, "which reaches 0 at expiry");
    CK(fx_ab_active(t0 + FX_AB_TIMEOUT_US - 1000), "and stays on through the window");
    CK(!fx_ab_active(t0 + FX_AB_TIMEOUT_US + 1000),
       "but expires by itself: it cannot be left armed");
    fx_ab_set(1, t0);
    fx_ab_set(0, t0 + 5000);
    CK(!fx_ab_active(t0 + 6000), "and switches off on demand");

    // ---- degenerate but legal shapes -------------------------------------
    fx_defaults(&p);
    p.enabled = 1; p.hold_ms = 0; p.pulses = 0;
    fx_init(&p, 1);
    fx_fire(t0);
    span(t0, t0 + 100000, &full, &hold, &rum);
    CK(full > 14000 && full < 16000 && hold == 0,
       "hold 0, pulses 0 is one clean strike");
    CK(fx_fire(0) == 0 || 1, "time zero does not crash");   // exercised, not asserted
    fx_init(&p, 1);
    CK(fx_fire(1ull << 40) == 1, "timestamps beyond 32 bits work");

    // ---- the serial command surface --------------------------------------
    static char cap[512];
    fx_set_reply([](const char* l) {
        strncat(cap, l, sizeof(cap) - strlen(cap) - 1);
    });
    fx_defaults(&p);
    p.enabled = 1;
    fx_init(&p, 1);
    cap[0] = 0;
    CK(fx_command("fx=hold:120,duty:55,pulse:2", 0) == 1,
       "a tuning line is claimed");
    CK(fx_get()->hold_ms == 120 && fx_get()->duty_pct == 55
       && fx_get()->pulses == 2, "and every key lands");
    CK(strstr(cap, "hold=120") && strstr(cap, "duty=55"),
       "the reply echoes what was accepted");
    cap[0] = 0;
    fx_command("fx=drive:999,rumoff:-500", 0);
    CK(fx_get()->drive_ms == 100 && fx_get()->rum_off_ms == -20,
       "out-of-range values are clamped, not applied");
    CK(strstr(cap, "drive=100") != 0, "and the echo shows the CLAMPED value");
    CK(fx_command("cam=thr:60", 0) == 0, "a cam line is left for the cam handler");
    CK(fx_command("fxsave", 0) == 1, "fxsave is claimed");
    cap[0] = 0;
    fx_command("fx=ab:1", 5000000);
    CK(fx_ab_active(6000000), "ab:1 arms dry-fire mode");
    CK(strstr(cap, "dry-fire ON") != 0, "and says so, expiry included");
    fx_command("fx=ab:0", 7000000);
    CK(!fx_ab_active(8000000), "ab:0 disarms it");
    cap[0] = 0;
    fx_command("fx=test:1", 10000000);
    CK(strstr(cap, "test fired") != 0, "test:1 fires a sequence");
    int s2, r2;
    fx_step(10000000 + 1000, &s2, &r2);
    CK(s2 == FX_SOL_FULL, "and the sequence is really playing");
    cap[0] = 0;
    fx_command("fx=test:1", 10005000);
    // a test press behaves like a pull now: mid-sequence it preempts and
    // re-strikes instead of reporting busy -- no press is ever swallowed
    CK(strstr(cap, "test fired") != 0,
       "a test fire during a sequence preempts instead of refusing");
    cap[0] = 0;
    fx_command("fx?", 10000000);
    CK(strstr(cap, "busy=1") != 0 && strstr(cap, "drive=") != 0,
       "fx? reports the parameters and the live state");
    CK(fx_command("fx=bogus:7", 0) == 1,
       "an unknown key is ignored without rejecting the line");
    // garbage in the middle must not derail later keys
    fx_defaults(&p); p.enabled = 1; fx_init(&p, 1);
    fx_command("fx=,,:,hold:200", 0);
    CK(fx_get()->hold_ms == 200, "malformed tokens are skipped, later keys land");

    // ---- a fast second pull is never a lost shot -------------------------
    fx_defaults(&p); p.enabled = 1; p.hold_ms = 200; fx_init(&p, 1);
    fx_fire(t0);
    // pull again mid-HOLD: old tail cut, new strike after the re-latch gap
    CK(fx_fire_preempt(t0 + 50000) == 1, "a pull during the hold is accepted");
    fx_step(t0 + 51000, &s2, &r2);
    CK(s2 == FX_SOL_OFF, "the old hold is cut so the spring can return");
    fx_step(t0 + 50000 + (uint64_t)FX_RELATCH_MS * 1000 + 1000, &s2, &r2);
    CK(s2 == FX_SOL_FULL, "and the new strike lands right after the re-latch gap");
    // pull during the quiet spacing, long after release: instant strike
    fx_defaults(&p); p.enabled = 1; p.space_ms = 400; fx_init(&p, 1);
    fx_fire(t0);
    uint64_t tq = t0 + 200000;                    // well past hold, inside space
    CK(fx_fire_preempt(tq) == 1, "a pull inside the spacing is accepted");
    fx_step(tq + 1000, &s2, &r2);
    CK(s2 == FX_SOL_FULL, "and strikes immediately -- the armature is long back");
    // pull during the STRIKE itself: also relatch, never a dead press
    fx_defaults(&p); p.enabled = 1; fx_init(&p, 1);
    fx_fire(t0);
    CK(fx_fire_preempt(t0 + 5000) == 1, "a pull during the strike is accepted");
    fx_step(t0 + 5000 + (uint64_t)FX_RELATCH_MS * 1000 + 1000, &s2, &r2);
    CK(s2 == FX_SOL_FULL, "with the fresh strike after the re-latch gap");
    // plain fx_fire (autofire path) still refuses politely during a sequence
    fx_defaults(&p); p.enabled = 1; p.hold_ms = 100; fx_init(&p, 1);
    fx_fire(t0);
    CK(fx_fire(t0 + 20000) == 0,
       "the autofire path still waits its turn -- only a pull edge preempts");

    // ---- cancel and the on-switch over serial ----------------------------
    fx_defaults(&p); p.enabled = 1; fx_init(&p, 1);
    fx_fire(t0);
    fx_cancel();
    fx_step(t0 + 1000, &s2, &r2);
    CK(s2 == FX_SOL_OFF, "cancel kills a playing sequence dead");
    CK(fx_fire(t0 + 2000) == 0,
       "but the earned quiet spacing survives the cancel");
    CK(fx_fire(t0 + 400000) == 1, "and firing resumes once it passes");

    // ---- the re-latch floor holds even when space is tuned below it ------
    fx_defaults(&p); p.enabled = 1; p.hold_ms = 0; p.space_ms = 0;
    fx_init(&p, 1);
    fx_fire(t0);
    uint64_t te = t0 + 16000;              // 1 ms after the strike ends
    CK(fx_fire(te) == 0,
       "space 0 cannot strike an armature still in flight (autofire path)");
    CK(fx_fire_preempt(te) == 1, "a pull is still accepted...");
    fx_step(te + 1000, &s2, &r2);
    CK(s2 == FX_SOL_OFF, "...but waits out the re-latch");
    fx_step(t0 + 15000 + (uint64_t)FX_RELATCH_MS * 1000 + 1000, &s2, &r2);
    CK(s2 == FX_SOL_FULL, "and strikes the moment the armature is back");
    fx_defaults(&p); fx_init(&p, 1);   // dormant again
    CK(fx_fire(400) == 0, "a dormant engine still refuses a trigger fire");
    CK(fx_fire_forced(400) == 1,
       "but an explicit test fire works with the engine off");
    fx_step(410, &s2, &r2);
    CK(s2 == FX_SOL_FULL, "and the test sequence really plays");
    CK(fx_get()->enabled == 0, "without switching the engine on behind the user");
    fx_defaults(&p); fx_init(&p, 1);
    fx_command("fx=on:1", 0);
    CK(fx_get()->enabled == 1 && fx_fire(500) == 1,
       "fx=on:1 wakes the engine over serial");
    fx_command("fx=auto:5000", 0);
    CK(fx_get()->auto_ms == 1000, "the autofire wait clamps at one second");
    cap[0] = 0;
    fx_command("fx?", 0);
    CK(strstr(cap, "auto=1000") != 0, "and fx? reports it");
    CK(strstr(cap, "temp=-1") != 0, "temp reads unknown before any trigger pull");
    fx_temp_note(2);
    cap[0] = 0;
    fx_command("fx?", 0);
    CK(strstr(cap, "temp=2") != 0,
       "a fatal temperature is a WORD on screen, not a mystery");
    fx_temp_note(-1);
    cap[0] = 0;
    fx_command("fx?", 0);
    CK(strstr(cap, "on=1") != 0, "and fx? reports the switch");

    // ---- quiet mode: the calibration tools' silence switch ----------------
    // This exists because the old way of "silencing" the gun -- turning the
    // knobs down, rumble time to zero -- HANDS the motor back to OpenFIRE,
    // whose off-screen rumble then fired through every calibration shot. The
    // ownership answers below are the contract the firmware glue reads.
    {
        fx_params_t q; fx_defaults(&q); q.enabled = 1; q.rum_ms = 0;
        fx_init(&q, 1);
        const uint64_t t = 5000000;
        CK(fx_owns_outputs(t) == 1,
           "engine on: the fx branch replaces the stock solenoid path");
        CK(fx_owns_rumble(t) == 0,
           "engine on with rumble time 0: the MOTOR still belongs to OpenFIRE "
           "-- the bug that fired rumble through every calibration");
        CK(fx_fire(t) == 1, "and it fires normally");

        fx_quiet_set(1, t);
        CK(fx_quiet_active(t) == 1, "quiet mode arms");
        int s3, r3;
        fx_step(t + 1000, &s3, &r3);
        CK(s3 == FX_SOL_OFF,
           "and kills the shot that was already playing -- not one strike "
           "inside the first capture");
        CK(fx_owns_outputs(t) == 1 && fx_owns_rumble(t) == 1,
           "quiet claims BOTH outputs, so both stock paths are skipped");
        CK(fx_fire(t + 1000000) == 0, "nothing fires from the trigger path");
        CK(fx_fire_preempt(t + 1000000) == 0,
           "and a fast second pull cannot preempt its way through either");
        CK(fx_fire_forced(t + 1000000) == 0, "nor can an explicit test fire");
        cap[0] = 0;
        fx_command("fx=test:1", t + 1000000);
        CK(strstr(cap, "quiet mode is ON") != 0,
           "and the test says WHY, instead of looking like a dead solenoid");

        // Dormant engine + quiet: this is the case the calibration actually
        // runs in, and the stock paths must still be shut out.
        fx_params_t d; fx_defaults(&d); fx_init(&d, 1);
        fx_quiet_set(1, t);
        CK(fx_owns_outputs(t) == 1 && fx_owns_rumble(t) == 1,
           "quiet silences a gun whose engine was never switched on -- the "
           "stock solenoid and rumble both belong to us while it lasts");
        CK(fx_get()->enabled == 0,
           "without switching the engine on behind the user's back");

        // Expiry: a tool that dies mid-calibration must not leave a gun that
        // never recoils again.
        CK(fx_quiet_active(t + FX_QUIET_TIMEOUT_US - 1) == 1,
           "quiet lasts the whole window");
        CK(fx_quiet_active(t + FX_QUIET_TIMEOUT_US) == 0,
           "then lapses by itself: a crashed tool cannot mute a gun forever");
        CK(fx_owns_rumble(t + FX_QUIET_TIMEOUT_US) == 0,
           "and the outputs go back to OpenFIRE when it does");
        fx_params_t e2; fx_defaults(&e2); e2.enabled = 1; fx_init(&e2, 1);
        fx_quiet_set(1, t);
        CK(fx_fire(t + 10000) == 0, "still refused inside the window");
        CK(fx_fire(t + FX_QUIET_TIMEOUT_US + 10000) == 1,
           "and firing comes back on its own after it");

        // Re-arming pushes the deadline out: the tool holds the switch down.
        fx_quiet_set(1, t);
        fx_quiet_set(1, t + 60000000);
        CK(fx_quiet_active(t + FX_QUIET_TIMEOUT_US + 1000) == 1,
           "re-arming extends the window, so a long calibration stays quiet");

        fx_quiet_set(0, t + 60000000);
        CK(fx_quiet_active(t + 60000001) == 0, "quiet:0 releases it at once");
        CK(fx_quiet_left_s(t + 60000001) == 0, "and the countdown reads zero");

        // The report carries both, and a tool tests for the KEY to learn
        // whether this firmware has quiet mode at all.
        fx_quiet_set(1, t);
        cap[0] = 0;
        fx_command("fx?", t);
        CK(strstr(cap, "quiet=1") != 0, "fx? reports quiet mode");
        CK(strstr(cap, "qleft=") != 0, "with its own countdown");
        cap[0] = 0;
        fx_command("fx=quiet:0", t);
        CK(strstr(cap, "quiet off") != 0, "and ~fx=quiet:0 answers plainly");
        CK(strstr(cap, " left=") == 0,
           "the quiet reply never writes the dry-fire countdown's key");
        fx_init(&q, 1);
        CK(fx_quiet_active(t) == 0, "a re-init clears quiet with everything else");
    }

    printf("\nrecoil engine: %s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
