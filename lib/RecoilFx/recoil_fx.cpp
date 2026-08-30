// Recoil effect engine implementation. One fire builds a timeline of absolute
// timestamps; fx_step answers "what should the pins be right now" from that
// timeline alone, so a delayed or jittery poll loop cannot corrupt a sequence.
#include "recoil_fx.h"

static fx_params_t s_p;
static uint32_t    s_rng = 0x2545f491u;

// The armed timeline, absolute microseconds. Phase boundaries for the strike,
// the hold, and each after-pulse, plus the rumble window and the quiet time.
static uint64_t s_sol0;                     // strike start
static uint64_t s_drive_end;                // strike end -- snapshotted, so a
                                            // live retune cannot stretch or cut
                                            // a strike already playing
static int      s_hold_on;                  // this sequence HAS a hold phase
static uint64_t s_hold_end;                 // strike+hold end (drive end if no hold)
static uint64_t s_pulse_on[FX_MAX_PULSES];  // after-pulse starts
static uint64_t s_pulse_off[FX_MAX_PULSES];
static int      s_pulses;
static uint64_t s_rum_on, s_rum_off;
static uint64_t s_sol_end;                  // last SOLENOID edge -- refire gates
                                            // on this, so a long rumble tail
                                            // cannot silently eat fast shots
static uint64_t s_seq_end;                  // last output edge (rumble included)
static uint64_t s_free_at;                  // sol_end + spacing
static int      s_armed = 0;

static uint64_t s_ab_until = 0;
static uint64_t s_quiet_until = 0;

static int clampi(int v, int lo, int hi, int* hit)
{
    if (v < lo) { *hit = 1; return lo; }
    if (v > hi) { *hit = 1; return hi; }
    return v;
}

void fx_defaults(fx_params_t* p)
{
    // Dormant by default: freshly flashed firmware behaves exactly like the
    // build before it until the engine is switched on by name.
    p->enabled    = 0;
    p->drive_ms   = 15;
    p->hold_ms    = 80;
    p->duty_pct   = 40;
    p->pulses     = 0;
    p->gap_ms     = 40;
    p->jit_pct    = 0;
    p->rum_off_ms = 0;
    p->rum_ms     = 0;
    p->space_ms   = 80;
    p->auto_ms    = 250;
}

int fx_clamp(fx_params_t* p)
{
    int hit = 0;
    p->enabled    = (p->enabled != 0);
    // Up to 100, the same ceiling OpenFIRE's own SolOn slider offers: the
    // strike is the power knob and the whole stock range fits inside it.
    p->drive_ms   = clampi(p->drive_ms,    5, 100, &hit);
    p->hold_ms    = clampi(p->hold_ms,     0, 500, &hit);
    p->duty_pct   = clampi(p->duty_pct,   25,  70, &hit);
    p->pulses     = clampi(p->pulses,      0, FX_MAX_PULSES, &hit);
    p->gap_ms     = clampi(p->gap_ms,     15, 120, &hit);
    p->jit_pct    = clampi(p->jit_pct,     0,  15, &hit);
    p->rum_off_ms = clampi(p->rum_off_ms, -20,  50, &hit);
    p->rum_ms     = clampi(p->rum_ms,      0, 200, &hit);
    p->space_ms   = clampi(p->space_ms,    0, 500, &hit);
    p->auto_ms    = clampi(p->auto_ms,     0, 1000, &hit);
    return hit;
}

void fx_init(const fx_params_t* p, uint32_t seed)
{
    s_p = *p;
    fx_clamp(&s_p);
    if (seed) s_rng = seed;
    s_armed = 0;
    s_free_at = 0;
    s_ab_until = 0;
    s_quiet_until = 0;
}

void fx_set(const fx_params_t* p) { s_p = *p; fx_clamp(&s_p); }
const fx_params_t* fx_get(void)   { return &s_p; }

// Linear congruential step; returns a factor in [1-jit, 1+jit] as microsecond
// scaling. Deterministic from the seed, which is what the tests rely on.
static uint64_t jitter_us(int ms)
{
    uint64_t base = (uint64_t)ms * 1000u;
    if (s_p.jit_pct <= 0) return base;
    s_rng = s_rng * 1664525u + 1013904223u;
    // spread in [-jit, +jit] percent from the top 16 bits
    int64_t spread = (int64_t)(s_rng >> 16 & 0xffff) - 32768;
    int64_t delta = (int64_t)base * spread * s_p.jit_pct / (32768ll * 100ll);
    return base + delta;
}

// The test path: an EXPLICIT test command fires even while the engine is
// dormant -- refusing it made "TEST FIRE with stock recoil" look broken.
// Busy and spacing still apply; only the on-switch is bypassed.
int fx_fire_forced(uint64_t now_us)
{
    const int keep = s_p.enabled;
    s_p.enabled = 1;
    const int r = fx_fire_preempt(now_us);   // a test press behaves like a pull
    s_p.enabled = keep;
    return r;
}

// Builds the whole timeline with the trigger moment at t. Split out so a
// preempting pull can arm at a chosen future start.
static void arm_at(uint64_t now_us)
{
    // The strike is never jittered: trigger-to-strike is the timing the hand
    // aims by. Jitter lands on the hold and the gaps only.
    const uint64_t drive = (uint64_t)s_p.drive_ms * 1000u;
    uint64_t t = now_us;
    if (s_p.rum_off_ms < 0 && s_p.rum_ms > 0) {
        // rumble leads: it starts now and the strike waits out the offset.
        // With the rumble off the offset is ignored -- a disabled rumble must
        // not cost the trigger 20 ms of dead delay.
        s_rum_on = t;
        s_sol0   = t + (uint64_t)(-s_p.rum_off_ms) * 1000u;
    } else {
        s_sol0   = t;
        s_rum_on = t + (uint64_t)s_p.rum_off_ms * 1000u;
    }
    s_rum_off = s_rum_on + (uint64_t)s_p.rum_ms * 1000u;
    if (s_p.rum_ms <= 0) s_rum_on = s_rum_off = 0;

    s_drive_end = s_sol0 + drive;
    s_hold_on   = (s_p.hold_ms > 0);
    s_hold_end  = s_drive_end + (s_hold_on ? jitter_us(s_p.hold_ms) : 0);
    uint64_t cursor = s_hold_end;
    s_pulses = s_p.pulses;
    for (int i = 0; i < s_pulses; ++i) {
        cursor += jitter_us(s_p.gap_ms);
        s_pulse_on[i]  = cursor;
        cursor += drive;                            // after-pulses have no hold
        s_pulse_off[i] = cursor;
    }
    s_sol_end = cursor;
    s_seq_end = cursor;
    if (s_rum_off > s_seq_end) s_seq_end = s_rum_off;
    s_free_at = s_sol_end + (uint64_t)s_p.space_ms * 1000u;
    s_armed = 1;
}

int fx_fire(uint64_t now_us)
{
    if (fx_quiet_active(now_us)) return 0;         // quiet mode: nothing fires
    if (!s_p.enabled) return 0;                    // dormant engine never fires
    // Only the SOLENOID timeline blocks a refire. A rumble tail longer than
    // the strike must not turn fast second shots into dead triggers; a new
    // shot simply restarts the rumble window along with everything else.
    if (s_armed && now_us < s_sol_end) return 0;   // solenoid still playing
    if (now_us < s_free_at) return 0;              // inside the quiet spacing
    // The re-latch time is physics, not spacing: with space tuned below it,
    // a strike this soon lands on an armature still in flight and is weak or
    // silent. The floor applies no matter how small space is set.
    if (s_armed && now_us < s_sol_end + (uint64_t)FX_RELATCH_MS * 1000u)
        return 0;
    arm_at(now_us);
    return 1;
}

// A pull that lands mid-sequence is INTENT, not noise: refusing it read as a
// missed shot. The remainder of the playing sequence is cut, and the new
// strike starts the moment the armature can land again -- immediately if the
// solenoid has been off long enough, else after the re-latch gap.
int fx_fire_preempt(uint64_t now_us)
{
    // Quiet mode outranks the preempt rule: a pull during a calibration must
    // not cut through to the coil the way a normal fast second pull does.
    if (fx_quiet_active(now_us)) return 0;
    if (!s_p.enabled) return 0;
    const uint64_t relatch = (uint64_t)FX_RELATCH_MS * 1000u;
    if (!(s_armed && now_us < s_sol_end) && now_us >= s_free_at
        && (!s_armed || now_us >= s_sol_end + relatch)) {
        arm_at(now_us);                            // engine idle: normal shot
        return 1;
    }
    uint64_t start = now_us;
    if (s_armed && now_us < s_sol0) {
        // strike not begun (rumble-lead window): just rebuild from now
    } else if (s_armed && now_us < s_sol_end) {
        int sol, rum;
        fx_step(now_us, &sol, &rum);
        if (sol != FX_SOL_OFF) {
            start = now_us + relatch;              // energised: spring first
        } else {
            // inside a gap: how long has the coil been off?
            uint64_t off_since = s_hold_end;
            for (int i = 0; i < s_pulses; ++i)
                if (s_pulse_off[i] <= now_us && s_pulse_off[i] > off_since)
                    off_since = s_pulse_off[i];
            if (now_us - off_since < relatch)
                start = off_since + relatch;
        }
    } else {
        // in the quiet spacing: off since the sequence ended
        if (now_us - s_sol_end < relatch && now_us >= s_sol_end)
            start = s_sol_end + relatch;
        if (start < now_us) start = now_us;
    }
    arm_at(start);
    return 1;
}

void fx_step(uint64_t now_us, int* sol, int* rumble)
{
    int s = FX_SOL_OFF, r = 0;
    if (s_armed) {
        // Every boundary read here was snapshotted at fire time: a retune
        // over serial mid-sequence changes the NEXT shot, never this one.
        if (now_us >= s_sol0 && now_us < s_drive_end)
            s = FX_SOL_FULL;
        else if (s_hold_on && now_us >= s_drive_end && now_us < s_hold_end)
            s = FX_SOL_HOLD;
        else {
            for (int i = 0; i < s_pulses; ++i)
                if (now_us >= s_pulse_on[i] && now_us < s_pulse_off[i]) {
                    s = FX_SOL_FULL;
                    break;
                }
        }
        if (s_rum_off > s_rum_on && now_us >= s_rum_on && now_us < s_rum_off)
            r = 1;
        // No disarm here: the outputs are a pure function of the timeline and
        // the clock, so a probe at any time answers the same way. The next
        // fire re-arms the timeline; nothing needs tearing down.
    }
    if (sol)    *sol = s;
    if (rumble) *rumble = r;
}

// Ends a playing sequence immediately -- the shutdown path's hook, so a mode
// change can never leave the timeline re-raising a pin the caller just lowered.
void fx_cancel(void)
{
    // The timeline dies but the quiet spacing it had earned does not: a pull
    // right after a cancel used to fire with no spacing and no re-latch,
    // onto an armature that may have been energised a moment before.
    s_armed = 0;
}

int fx_busy(uint64_t now_us)
{
    return (s_armed && now_us < s_sol_end) || now_us < s_free_at;
}

void fx_ab_set(int on, uint64_t now_us)
{
    s_ab_until = on ? now_us + FX_AB_TIMEOUT_US : 0;
}

int fx_ab_active(uint64_t now_us)
{
    return s_ab_until != 0 && now_us < s_ab_until;
}

// Seconds of dry-fire mode left; 0 when off. The expiry was invisible before
// this -- a mode that disarms itself must say how long it has, or "it stopped
// working" is the user's only possible reading.
int fx_ab_left_s(uint64_t now_us)
{
    if (s_ab_until == 0 || now_us >= s_ab_until) return 0;
    return (int)((s_ab_until - now_us) / 1000000u);
}

void fx_quiet_set(int on, uint64_t now_us)
{
    s_quiet_until = on ? now_us + FX_QUIET_TIMEOUT_US : 0;
    // A shot already in the air stops NOW. Waiting for it to finish would put
    // a strike inside the first capture of the calibration that just asked for
    // silence -- the one frame the whole run is measured from.
    if (on) fx_cancel();
}

int fx_quiet_active(uint64_t now_us)
{
    return s_quiet_until != 0 && now_us < s_quiet_until;
}

int fx_quiet_left_s(uint64_t now_us)
{
    if (s_quiet_until == 0 || now_us >= s_quiet_until) return 0;
    return (int)((s_quiet_until - now_us) / 1000000u);
}

// Quiet mode claims both outputs precisely so the stock paths are skipped:
// that is the whole point, not a side effect. Note what rum_ms means here --
// with the engine merely ON and rum_ms 0, OpenFIRE keeps the motor, which is
// the correct stock behaviour and the reason quiet needs its own switch.
int fx_owns_outputs(uint64_t now_us)
{
    return s_p.enabled || fx_quiet_active(now_us);
}

int fx_owns_rumble(uint64_t now_us)
{
    return (s_p.enabled && s_p.rum_ms > 0) || fx_quiet_active(now_us);
}
