# The WiiCam (DFRobot SEN0158 / Wii Remote IR camera) — what we know about its registers

A reference for anyone touching `lib/WiicamAim/` or the DFRobot driver. Everything
here is either taken from a document that names its source, or measured on our own
hardware with the capture files in hand. The **Confidence** column says which.

- *Confirmed* — behaviour observed on our sensor, or in the Wii's own writes.
- *Named only* — the register has an accepted name (WiiBrew), but what it does at
  the edges is not documented anywhere we found.
- *Measured* — inferred from our own captures; the capture is cited.
- *Undocumented* — nobody knows; the value is copied because it works.

I²C address `0x58` (`0xB0 >> 1`). All writes are two bytes: register, value.
The chip is a PixArt object-tracking sensor: the imager finds blobs internally,
the chip reports **at most four** of them per frame. Nothing outside those four
is ever seen by the host, which is the whole reason MAXSIZE/MINSIZE matter —
they are the only controls that act before the four slots are handed out.

## Registers

| Reg | Name | Function | Wii writes | OpenFIRE presets (sens 0 / 1 / 2) | Ours | Confidence |
|---|---|---|---|---|---|---|
| `0x00` | — | Unknown. `0x02` on the Wii's sensitivity levels 1–4, `0x07` on level 5 — the only register that distinguishes the top level. | `0x02` / `0x07` | not written | not written | Undocumented |
| `0x01` `0x02` | — | Unknown. Zero in every known preset. | `0x00` | — | — | Undocumented |
| `0x03` `0x04` | — | Unknown, but *pinned*: the Wii writes `0x71` and `0x01` on all five levels; the community presets leave both at zero and work. | `0x71` / `0x01` | not written | not written | Undocumented |
| `0x06` | **MAXSIZE** | Maximum blob size. Blobs above it are not reported. Whether an oversized blob is discarded, clamped or split is stated nowhere. **`0` blinds the sensor** (no blobs at all) — the loop never writes below 1. | `0x64`–`0xC8` | `0x90` / `0x90` / `0xFF` | the auto light limit loop bisects it between 1 and the preset; `~cam=hwmax:` by hand | Named only; blinding at 0 is Confirmed |
| `0x08` | **GAIN** | Sensor gain. **Smaller value = more gain.** WiiBrew calls the same byte "intensity sensitivity, increasing values reducing the sensitivity". | `0xFE` → `0x20` | `0xC0` / `0x41` / `0x0C` | not written (the preset owns it) | Confirmed |
| `0x1A` | **GAINLIMIT** | Gain limit; must be less than GAIN or the camera does not function. Holds for all eight known presets; equals GAIN − 1 in all five of Nintendo's. | GAIN − 1 | `0x40` / `0x40` / `0x00` | not written | Confirmed |
| `0x1B` | **MINSIZE** | Minimum blob size. Range corroborated by Nintendo's own presets; rejection direction inferred from the name. Never written by the stock driver, so it sits at an unknown default. | `3`–`5` | not written | `~cam=hwmin:` by hand only; never saved | Named only |
| `0x30` | Enable | `0x08` is the only value at which the camera outputs data — at anything else it returns all `0xFF`. The Wii writes `0x01` before changing mode or sensitivity, then `0x08`. | `0x01` then `0x08` | same | same (driver) | Confirmed |
| `0x33` | Mode | Output format: `1` basic (10 B), `3` extended (12 B), `5` full (36 B). | | `1` / `3` | `1` / `3` / **`0x55`** for full | Confirmed |
| `0x33` = `0x55` | — | Also selects full mode on our sensor; what the high nibble does is unknown. `0x05` works too (`~cam=fullreg:5`). We ship `0x55` because that is what was confirmed first; both are accepted. | | — | `0x55` default, not saved | Confirmed on our sensor |
| `0x36` | Data (with header) | Reading from `0x36` yields 37 bytes in full mode: one junk/header byte, then the 36 data bytes. The driver reads from here. | | | | Confirmed |
| `0x37` | Data | Object data proper, 36 bytes. | 36 bytes | | | Confirmed |

The Wii's five levels write a whole block (`0x00`–`0x08`, then `0x1A`–`0x1B`) at
once; the OpenFIRE driver writes only `0x06`, `0x08`, `0x1A`. That is why "Max"
sensitivity in OpenFIRE is MAXSIZE `0xFF`, wide open, where Nintendo never goes
above `0xC8`.

## Report formats

Per object, all formats start with the same three bytes:

| Byte | Content |
|---|---|
| 0 | X low 8 bits |
| 1 | Y low 8 bits |
| 2 | bits 7–6: Y high 2 bits; bits 5–4: X high 2 bits; bits 3–0: **size** (0–15, extended and full only) |

Positions are 10-bit, `0..1023` × `0..767`. `Y > 767` is the driver's "not seen"
test for a slot.

| Format | `0x33` | Bytes | Per object | Extra fields |
|---|---|---|---|---|
| Basic | `1` | 10 | 2.5 (pairs share the high-bits byte) | none |
| Extended | `3` | 12 | 3 | size |
| Full | `5` / `0x55` | 36 (+1 header when read from `0x36`) | 9 | bytes 3–6: box `xmin ymin xmax ymax`, **7-bit, in the sensor's native 128×96 array** (not the 1024×768 the positions use); byte 7: unknown (reads 0); byte 8: intensity, 8-bit |

We keep the box as width = `xmax − xmin` and height = `ymax − ymin` (0 for a
one-pixel blob), plus the origin `xmin, ymin`. The `CAM: blobs` line and the blob
CSV carry `w, h, i, xm, ym` per blob in full mode.

## What we measured on our own hardware

Captures referenced are in the blob CSVs from the September 2026 sessions
(`blobs004094926`, `blobs003102848`, `blobs002114852`) and the shape CSVs beside them.

| Finding | Evidence | Confidence |
|---|---|---|
| Full mode runs at ~160–170 fps on the RP2040 at 1 MHz I²C, 37-byte read. | `hz` column, every capture | Measured |
| An LED on our bar never exceeds **12 columns wide** or **3 rows tall** in the box fields (99%+ of 37,568 blobs; the tail was window contamination from junk locks, since removed). | `shape002102846` | Measured |
| A window looked at straight on reports **flat slabs 13–93 wide, 0–11 tall**, up to four per frame — width separates them from LEDs; height and `size` do not. | `blobs003102848`, `blobs002114852` | Measured |
| The same window at a shallow angle, or a sun patch on the floor, can come in as a **3×0 fragment** — smaller than an LED. No shape gate catches that; only position (the resolver) does. | `blobs004094926` | Measured |
| **MAXSIZE did not remove 60-wide slabs at 96** (nor at 114, 134, 174). Across all captures no blob ever had `size > MAXSIZE / 16` (63→2, 96→5, 174→7, 255→8), so the hypothesis is that MAXSIZE bounds the 4-bit `size` field (≈ intensity-area), not the box. Not proven — few samples at low values. | `blobs003102848`, `blobs004094926` | Measured, hypothesis |
| If that holds, MAXSIZE cannot separate a flat wide slab (`size` 2–5) from a bright LED (`size` 1–3); the firmware's width gate does, which is why it exists. | same | Inference |
| MAXSIZE `0` = no blobs (the gun goes dark until the register is rewritten). | earlier session, hwmax:0 | Confirmed |
| The sensitivity preset write (`sensitivityLevel()`) rewrites `0x06` on every profile switch and pause-menu exit, so anything we put there has to be put back afterwards (`wiicam_aim_hw_dirty`). | firmware patch `OpenFIREcommon.cpp` | Confirmed |
| A full-mode read that fails validation (length or header) must drop the format back, or every subsequent report decodes to nonsense (`wiicam_aim_fmt_fallback`). | firmware | Confirmed |
| Register writes need the camera bus: a write from the other core mid-read corrupts the read. `aim_cam_take/give` serialises them. | earlier session (I²C hangs) | Confirmed |

## Open questions

- What `0x06` actually compares against (box area, pixel count, or the 4-bit
  `size`). A direct test: `~cam=hwmax:40` while aiming at a window with the bar in
  view, then read the blob CSV — if `size ≥ 3` blobs vanish while LEDs (`size` 1–2)
  stay, the field is `size` and the loop should search in 16 steps, not 255.
- Whether MINSIZE (`0x1B`) rejects by the same measure.
- What byte 7 of a full-mode object carries (always 0 on ours).
- The high nibble of `0x33` (`0x55` vs `0x05`).
- Registers `0x00`, `0x03`, `0x04`: the Wii pins them; we have never varied them.

## Where this is used

- `lib/WiicamAim/wiicam_aim.cpp` — `wiicam_aim_full_poll` (unpack), the auto light
  limit loop (`0x06`), `~cam=hwmin:` (`0x1B`), `~cam=fullreg:` (`0x33`).
- `libraries/DFRobotIRPositionEx` (OpenFIRE) — `sensitivityLevel()` (the preset
  table above), `dataFormat()`, `begin()` (`0x30` sequence).
- `README.md` §6 — the serial commands that expose each of these.
