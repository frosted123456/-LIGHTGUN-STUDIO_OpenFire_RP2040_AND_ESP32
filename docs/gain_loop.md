# Gain loop (register 0x08) — implementation spec

Status: **implemented and host-verified** (`hostcheck/check.sh` 44/44). This file
is now the design record, kept because the reasoning is not obvious from the
code. What shipped differs from the original plan in five places, all noted
below under **As built**. Read `lib/WiicamAim/wiicam_aim.cpp` sections
"the hwmax loop" (K1–K7) first: the gain loop is the same controller on a
second register with a different verdict.

## Why

In sun the wiicam's LEDs bloom horizontally (5×2 → 12×3 boxes, intensity
unchanged) and horizontally adjacent pairs merge into one blob. The sensor
has no AGC (2020 outdoor paper; our captures agree). Width is readout smear
along the row, proportional to how far the pixel sits over the lit
threshold. GAIN (0x08, "smaller = more gain") sets that directly. Sensitivity
2 runs 0x08 = 0x0C with GAINLIMIT (0x1A) = 0, the fully open corner; DFRobot
ships 0xC0/0x40. The MAXSIZE loop cannot touch a merge (it acts after the
object is labeled). merge_split (resolver) recovers a merge after the fact;
this loop makes merges rarer at the source.

## Non-negotiables (from the project rules)

- No fixed numbers: the preset byte is the ceiling, this rig's own cut level
  is the floor, evidence is from this frame/dwell only.
- Must never leave the gun blind: the rail is the existing V_CUT verdict and
  the dimmest corner's intensity; worst case is one dwell, then back up.
- OV/ESP32 path untouched (wiicam adapter only).
- Every rule mutation-tested; `hostcheck/check.sh` run once per batch.
- Release comments short: what and why.

## Registers

| Reg  | Name      | sens 0 / 1 / 2 preset | Loop use |
|------|-----------|-----------------------|----------|
| 0x08 | GAIN      | 0xC0 / 0x41 / 0x0C    | driven: raise the BYTE to lower gain, from the preset upward |
| 0x1A | GAINLIMIT | 0x40 / 0x40 / 0x00    | must stay BELOW 0x08's byte. At sens 2 it is 0: always valid. At sens 0/1 (0x40) the byte must never go below 0x41: clamp `val >= limit + 1`. Never written by the loop. |
| 0x06 | MAXSIZE   | 0x90 / 0x90 / 0xFF    | the existing loop |

Writes go through `s_blobreg(reg, val)` from `wiicam_aim_hw_tick()` (pump
core), exactly like 0x06: add `s_hwgain` (int16, `WIICAM_HW_LEAVE` /
`WIICAM_HW_RESTORE` sentinels) next to `s_hwmax`, written when `s_hw_dirty`.
RESTORE = re-select the current sensitivity level (`s_sens_set`), same as
0x06 — the preset rewrites 0x08 too. `wiicam_aim_begin()`/sensitivity change
must mark the gain loop's value stale the way it does for MAXSIZE (grep
`s_hw_dirty = true` near "begin() rewrites 0x06").

Unknown until hardware says: whether 0x08 accepts a live write and how fast
the sensor reacts. Log the byte written and the dimmest-corner intensity
before/after on `~camgain?`; the first capture answers it.

## State (mirror the MAXSIZE loop's names with a `g` prefix)

```
s_gl_on        bool   default true (wiicam), off with cam=gloop:0
s_gl_state     HOLD / LOWER / RAISE / NOSAFE / OFF   (same enum, own variable)
s_gl_val       current 0x08 byte             (preset at boot)
s_gl_lo        highest byte known to CUT an LED   (= lowest gain that lost a corner)   init 0x100 "unknown"
s_gl_hi        lowest byte that still MERGED       init = preset
s_gl_dwell, s_gl_nclean, s_gl_nmerge, s_gl_ncut, s_gl_cut_run, s_gl_dwell_nlock
s_gl_hold_run, s_gl_saved, s_gl_from_flash, s_gl_settle
s_gl_imin      dimmest matched corner's intensity byte this dwell (min over frames)
s_gl_wmed      median single-LED width this dwell
s_gl_curve[N]  (byte, imin, wmed) per settled dwell, N = 8, ring   -- the learning
```

Direction reminder: "LOWER gain" = byte goes UP toward 0xC0. Keep a helper
`gl_write(byte)` that clamps to `[preset, 0xC0]` and `>= limit + 1`, sets
`s_hwgain`, `s_hw_dirty`, `s_gl_settle = LOOP_SETTLE`.

## Per-frame verdict (computed where the MAXSIZE verdict is, same inputs)

| Verdict   | Condition |
|-----------|-----------|
| G_CLEAN   | `an == 4 && r.locked && r.n_real == 4 && r.split == 0` |
| G_MERGE   | `r.split > 0` (merge_split fired) OR `quad_ambig_total()` advanced this frame (ambiguity refusal) — both are the merge fingerprint |
| G_CUT     | exactly the existing V_CUT condition (`an <= 3 && r.locked && r.n_real >= 2 && recent && !vouched`) |
| G_NONE    | otherwise |

Also per frame, when `r.locked && r.count == 4`: `imin = min` over matched
corners of `s_bi[i]` (intensity byte), fold into `s_gl_imin`; `wmed` = median
`s_bw[i]` over matched corners, fold into `s_gl_wmed`.

## Dwell rules (K1–K7 shape)

- K1: judged on the sensor's report and the resolver's verdict, before any
  software gate — same as MAXSIZE.
- K3 RAISE (immediate): `LOOP_RAISE_N` consecutive G_CUT → `s_gl_lo = s_gl_val`
  (this byte cut an LED), write `max(preset, (s_gl_lo + s_gl_hi)/2)` — i.e.
  back toward more gain, bisecting between the byte that merged and the byte
  that cut. Provisional lo exactly like `loop_lo_settle`: confirmed only if
  the following dwell locks ≥ LOOP_DWELL/2.
- LOWER (patient, one per dwell): `ncut == 0 && nmerge >= LOOP_DWELL/4`
  (merges are rarer than strays — a quarter of a dwell is a lot of merges)
  → `s_gl_hi = s_gl_val`; write `(s_gl_val + s_gl_lo)/2` if lo known, else
  `min(0xC0, s_gl_val * 2)` (doubling walks 0x0C→0x18→0x30→0x60→0xC0: five
  steps, the same bisection idea in the other direction).
- Prediction rail (the learning): before any LOWER, if the curve has ≥ 2
  points, fit intensity vs byte linearly through them and refuse the step
  whose predicted `imin` falls below the intensity recorded on the dwell
  that last produced a CUT (`s_gl_icut`, -1 = never). If refused, HOLD and
  count `s_gl_pred_hold` (visible on `~camgain?`). No cut ever seen → no
  rail beyond K3.
- K4 (byte from flash never locks for a whole dwell) → back to preset,
  record lo, same as MAXSIZE.
- K5 NOSAFE: `lo` and `hi` adjacent → preset, until a clean dwell.
- Settled: `LOOP_SETTLED` clean dwells → save `hwg0` (val, lo, hi) via new
  `aim_hwgain_store/load/clear` in `aim_runtime.cpp` (copy `aim_hwloop_*`,
  key `"hwg0"`; add to `aim_nvs` test stubs' `is_u32key` and slot counts:
  `hostcheck/wiicam_adapter_test.cpp`, `wiicam_learn_test.cpp`).
- `camreset` clears `hwg0` and restarts from the preset; `cam=gloop:1`
  restarts the search; `cam=hwgain:N` by hand switches the loop OFF (like
  `hwmax:`); `hwgain:-1` leaves it alone.

## Arbitration with the MAXSIZE loop

One register moves per dwell. Merges drive the gain loop; strays drive the
MAXSIZE loop; they never read the same verdict. While one loop is in LOWER
or RAISE (mid-search) the other holds its dwell counters (do not count, do
not act). A CUT backs off whichever loop moved LAST (`s_last_mover`). Both
share `s_hw_dirty`/settle so writes never overlap.

## Reporting

- `~camgain?` → `CAM: gain on=1 state=HOLD val=12 lo=256 hi=12 dwell=n/50
  clean= merge= cut= imin= wmed= icut= pred_hold= settled= saved=` (same
  shape as `~camloop?`; keep `on=` first so the tools' token parser applies).
- `cam?` tail: append `gloop=… gval=… glo=… ghi=…` after the loop fields.
- Shape CSV / BlobLog: add columns on the END: `gval gstate glo ghi gimin
  gwmed` (tools: `tools/gun_studio.py` COLS + `sample()` + camgain parser;
  `hostcheck/blob_log_test.py` COLS tuple, verbatim-line tests).
- pical: poll `~camgain?` at the same cadence as `~camloop?` while logging.
- README: one row for `~camgain?`, one for `cam=gloop`/`hwgain`; update
  `docs/wiicam_registers.md` 0x08 row.

## Tests (mirror the MAXSIZE loop's blocks in `wiicam_adapter_test.cpp`)

Use the same harness (`arm`, `lock_and_zero`, `run`, `loopq`-style parser
for `camgain?`, `regs06()` → add `regs08()`). Cases, each with a mutant that
must fail:

1. Merges for a dwell (drive 3 blobs with the bottom pair merged; widths
   offered) → LOWER: byte doubles, register 0x08 written once. Mutant: LOWER
   threshold `>= 1` instead of `LOOP_DWELL/4`.
2. Clean dwells → HOLD, no write; four clean → saved `hwg0`.
3. Five G_CUT frames → RAISE at once, lo recorded, byte back toward preset.
   Mutant: lo not recorded.
4. Provisional lo withdrawn when the dwell after the RAISE never locks.
5. lo/hi adjacent → NOSAFE at preset; clean dwell releases.
6. Prediction rail: two curve points with falling intensity → the next LOWER
   whose predicted imin < icut is refused (pred_hold +1, no write). Mutant:
   rail removed.
7. Arbitration: merges + strays in the same dwell → only the gain loop
   moves; MAXSIZE loop's dwell counters untouched. Mutant: both move.
8. `cam=hwgain:64` by hand → loop OFF, register written; `gloop:1` resumes.
9. Clamp: at sens 0 (limit 0x40) the byte never goes below 0x41.
10. camreset → preset, `hwg0` erased; boot from flash with a saved byte →
    K4 if it never locks.
11. `camgain?` line verbatim.

Replay: `hostcheck` already has replay blocks from Pi captures; add one from
`blobs012014353.csv` (sun): merged 30×3 blobs at the bottom pair → the loop
lowers gain within two dwells and never writes below the preset.

## Delivery

SendUserFile then `device_commit_files` (force) to
`C:\Users\Home\Desktop\Francis\Claude\projects\IR sensor\ LIGHTGUN-STUDIO_OpenFire_RP2040_AND_ESP32`
(leading space is real); verify SHA-256 container vs disk with device_bash.
Firmware: `lib/WiicamAim/wiicam_aim.cpp`, `lib/AimPipeline/aim_runtime.{cpp,h}`.
Tools: `tools/gun_studio.py`, `pical/pical.py`. Tests: `hostcheck/*`.

---

## As built — where it differs from the plan above

1. **A merge is no longer read as a cut, for EITHER loop.** Two LEDs reported
   as one blob leave three where four were: the exact shape of a cut, but
   nothing was cut. Left alone it walked MAXSIZE back up for a reason MAXSIZE
   cannot fix and hid from the gain loop the one event it exists for. The
   merge fingerprint is now computed before the verdict and excluded from
   `V_CUT`. This is a fix to the pre-existing hwmax loop as much as to this one.

2. **The two registers share one restore.** Re-selecting the sensitivity level
   is the only restore the driver has and it rewrites `0x06`, `0x08` and
   `0x1A` together, so a restore asked for by one loop wiped the other's byte.
   `wiicam_aim_hw_tick()` now resolves BOTH `WIICAM_HW_RESTORE` sentinels
   before writing either value. Covered by test (10a) — a mutant that reverts
   the ordering fails it.

3. **The prediction rail is reachable in exactly one place**, and it is worth
   knowing which: after a cut, `hi` is known and the bisection already stops
   the loop short, so the rail adds nothing. It bites after **NOSAFE
   recovery**, where the clean dwell that releases NOSAFE throws both bounds
   away — they were about a room that has changed — while `icut` survives,
   because the room changed and the LEDs did not. Without the rail the loop
   would double straight back into the byte it already knows cuts. Tested in
   the learn suite, in full mode.

4. **`imin`/`wmed` are measured on CLEAN frames only and in FULL mode only.**
   A merged or cut frame has a non-LED in the blob list; and outside full mode
   the intensity byte holds whatever the last full frame left there, which
   read as "the dimmest LED is at zero" would arm the rail against a
   measurement that never happened and freeze the loop for the session.
   `~camgain?` reports the last finished dwell's values when the current one
   has none yet, so a tool polling just past a dwell boundary does not read -1.

5. **pical alternates the two controller questions in one poll slot.** Two
   questions in one frame come back as one burst, and a third consumer of the
   wire starves the camfit poll. Each controller is asked at half the old
   cadence; the wire carries exactly what it did before.

### Settings, as answered

| | |
|---|---|
| Evidence | Both a split-and-recovered merge and a refused one count. The split recovers a merge to a couple of px; it does not undo it. |
| Eagerness | `GL_MERGE_N = LOOP_DWELL / 4` — 12 merge frames of 50. Fires in a burst like `blobs012`, ignores the occasional one. |
| Ceiling | `GL_CEIL = 0xC0`, OpenFIRE's own sensitivity-0 gain: the least gain this codebase already ships. |
| Default | On, like the auto light limit loop. `~cam=gloop:0` switches it off. |

### Still unknown until it runs on hardware

- Whether register `0x08` accepts a live write and how fast the sensor reacts.
  `~camgain?` logs the byte and the dimmest-corner intensity either side of a
  step; `gval`/`gimin` in the CSV answer it on the first capture.
- Whether blob width actually tracks gain on this part. `gwmed` against `gval`
  is the plot that says so. If it does not move, the premise is wrong and the
  loop should be switched off rather than tuned.

---

## Field note 1 — the solenoid stutter (capture `blobs004022014`)

**What the capture proved.** The loop stepped once, 0x0C → 0x18, settled and
saved; `ghi` stayed 256, so it never cut an LED getting there. LED box width
fell 5–12 px → 2–3, the size byte 2–5 → 1, intensity 15–30 → 4–14, and merges
went from bursts of ~60/s to nothing. **Both open questions are answered: 0x08
takes a live write, and blob width really does track gain.**

**What it also exposed.** The user felt the solenoid stutter, and the gun's own
clock shows the frame rate halving (200 → 100–140 fps) while polls climb to
400–615/s. That is gun-side and the Pi cannot cause it.

The cause was in this change. Every register write goes through
`aim_blobreg_set()` in the rp2040 patch, which does `fx_glue_shutdown()` —
"this call blocks core 1 for tens of ms", the core the solenoid runs on —
then `aim_cam_take()`, which stops core 0 polling the sensor, then `delay(10)`.
`s_hwmax`/`s_hwgain`/`s_hwmin` are never cleared after a successful write, so
**any** `s_hw_dirty` mark rewrote every register we held, at that price, to put
back bytes the sensor already had. Adding the gain loop gave that tick a second
register and doubled it.

**Fixed, three parts:**

1. A shadow of what the sensor is believed to hold (`s_hwmax_at`,
   `s_hwgain_at`, `s_hwmin_at`). A register is written only when the wanted
   value differs. `hw_shadow_lost()` invalidates all three wherever the preset
   rewrites the block — `camrebuilt`, `wiicam_aim_hw_dirty`, and after the
   `s_sens_set` restore inside the tick.
2. **One register per tick.** Whatever is left keeps `s_hw_dirty` set and goes
   on the next pass, microseconds later, instead of three shutdowns back to
   back.
3. `more` (work deferred) separated from `done` (a write failed). Only a real
   failure arms the one-second dead-sensor backoff; deferred work must not wait
   a second behind the register in front of it.

4. **No write lands mid-shot.** `wiicam_aim_hw_tick()` returns immediately
   while `fx_busy()`. The write hook forces both output pins low before every
   write — it has to, because the core is about to stop for tens of ms and a
   solenoid left energised is a burnt solenoid — so a write during a pulse cuts
   the shot in half, which is exactly what the user felt. Both loops are
   patient by construction (one step per quarter-second dwell), and the dirty
   mark and store requests persist, so the write simply waits for the shot.

`bregw` / `bregf` (writes that landed / were refused) are now on the camblob
line and in the CSV, so the next capture measures this rather than inferring it.

**A method note worth keeping:** the first pass at this analysis bucketed the
counters by pical's log timestamps and concluded the gun was fine. It is not
safe to do that — while pical is backlogged its rows are stale, so gun-side
events get smeared across the wrong seconds. Always re-key on `gun_ms`, which
tracks wall time to ±0.5 s even when a core is blocked.

## Field note 2 — the same capture, read again

A second pass over `blobs004022014`, with the CSV rather than the memory of
it, overturns most of field note 1's diagnosis. The write storm did not
happen: **two** register writes in 135 s (`hwmax` 15 → 7 → 14, both at
02:21:33), `gval` 24 and `HOLD` on all 364 rows. What the capture actually
holds:

| t (s) | bn | hwmax | what the row says |
|---|---|---|---|
| 72–76 | 4 | 15 | steady; `gimin` = **4**, corner intensity median 7 (scale 0–255) |
| 76–78 | **0–1** | 15 | LEDs gone for ~2 s, `bdrop` +228 — the "hang": nothing to see |
| 78 | 4 | **15→7** | LEDs return; resolver past its 1 s no-lock window, so four blobs read as a stray-only room; hwmax LOWERs |
| 79 | **2** | 7 | 7 cuts two LEDs |
| 79 | 4 | **7→14** | K3 raises. Two writes, two `fx_glue_shutdown()` — the stutter |

**Root cause.** The loop settled at 0x18 with the dimmest corner at intensity
4 — the sensor's floor, not a margin above it (p10 of all corner readings is
4). No cut had ever been measured (`ghi` 256), so `icut` stayed −1 and the
prediction rail never armed. A scene shift took the LEDs under threshold;
the re-acquire dwell then fed the hwmax loop a "stray-only room".

**Fixed, in `wiicam_aim.cpp`, each with a mutation-tested assertion:**

1. `s_last_mover` is cleared once the mover's loop finishes a dwell in HOLD.
   Left sticky, one MAXSIZE move disabled the gain loop's cut-raise for the
   rest of the session.
2. The hwmax loop defers to the gain loop mid-search, the same way the gain
   loop already deferred to it.
3. The **re-acquire dwell**: the first dwell after the sensor reported ≤ 1
   blob is not evidence to LOWER on, for either loop. This is the t=78 fix.
4. A cut nobody moved for goes to MAXSIZE first (cannot bring a merge back)
   and to gain only once MAXSIZE has nothing left — at its preset, NOSAFE,
   off, **or vouched**. The gain loop also now *sees* the cut at a vouched
   MAXSIZE: the shared verdict used to drop it there, so gain was never told.
5. The rail from a measured floor: `icut` falls back to the last clean
   dwell's dimmest corner when the cut dwell measured nothing; a settled
   dwell whose dimmest corner is at or under `icut` is treated as a cut
   (hi = this byte, back into the band) instead of kept; and a second of an
   empty sensor at a gain above the preset, with no untested MAXSIZE out, is
   a cut too (`gl_blind`, mirroring `loop_blind`).
6. `~cam=msplit:0/1` — merge_split as a live A/B knob; `QuadResult.merge`
   counts blobs the resolver judged a merged pair whether or not it split
   them.

**Not fixed, on purpose.** A 2 s watchdog was planned. `wiicam_aim_hw_tick()`
only runs in Run mode; Pause has 300 ms delays and Docked has its own
`for(;;)`, so any feed placed in the overlay would reset the gun in the
pause menu or while docked to the Studio. It needs a beat at the top of both
upstream loops in every mode, which is an upstream edit, not an overlay one.

**The stall record.** `~camblob?` (and the Studio/pical CSV) now carry
`c0gap` / `c1gap` (longest gap between camera polls / between passes of the
pump core's loop since the last read, µs), `holdmax` / `holdus` (longest
single camera hold since the last read, total held since boot) and `pfail`
(camera reads the driver refused). A frozen gun cannot leave a clean log
again.

**Two more things the capture says.**

- pical goes quiet for 2–4 s every 15.6–18.3 s, like clockwork, with the
  gun at ~200 fps throughout. `BlobLog` called `os.fsync()` every second on
  the front end's frame loop; a USB stick's housekeeping can hold that for
  seconds. The sync now runs on its own thread, one in flight at a time,
  skipped and counted when the last is still busy. This is the hub freezing.
- `bpolls` sustains ~600/s against `startIrCamTimer(420)`. Upstream's timer
  is a PWM wrap whose divider is computed from `clk_sys` *at init*; OpenFIRE
  overclocks after, so the tick runs proportionally faster. Not a fault, but
  it means ~2/3 of full-mode reads are duplicates of a ~200 Hz sensor.
