# Lightgun Studio — calibrated aim pipeline for OpenFIRE

One aim pipeline, two supported guns:

| Build | Sensor | Board | Cost class |
|---|---|---|---|
| **ESP32-S3 + OV2640** | camera, blob detection in firmware | Freenove ESP32-S3-WROOM CAM | ~12 € with the board |
| **RP2040 + wiicam** | SEN0158 / Wii camera, blob detection on-chip | stock OpenFIRE RP2040 hardware | reuses what you have |

Both replace OpenFIRE's aiming with a calibrated pipeline. What you gain over
stock: a **two-distance calibration** that solves where the camera really
points and what your LED rectangle really is — so aim holds when you move
around the room and the camera does not need to be aligned with the barrel;
**corner identity** that survives an LED dropping out mid-motion instead of
re-sorting and springing; a tunable **latency lead** (0–50 ms) so the cursor
stops trailing fast swings; an adjustable **smoothing level** (0–10) to trade
cursor jitter for glide, tuned live from the fine-tune screen; a **fine tune** that lines the cursor up with your
iron sights; and a **verify** step that measures the result instead of assuming
it. Studio detects which gun is plugged in and adapts.

Please test. You get extremely accurate aim with snappy aim. No springy effect when fine tunning is properly performed. Give me your feedback!

---

## 1. What you need

**Hardware — ESP32-S3 + OV2640 build**

| Part | Specification |
|---|---|
| Board | Freenove ESP32-S3-WROOM CAM, **N8R8** — 8 MB flash, 8 MB **octal** PSRAM |
| Camera | OV2640 module **without the IR-cut filter** — sold as "NV" / "night vision"  **75 mm** FPC ribbon between camera and board |
| Lens filter | IR long-pass over the lens — **800 nm recommended (non tested)**, 700 nm works |
| Cable | One USB-C cable to the board's **native USB** socket (see below) |

**Why those wavelengths (OV2640 build).** 850 nm LEDs with an 800 nm
long-pass filter is a matched pair: the LEDs pass, room light and screen glow
do not. Do not substitute 940 nm LEDs on THIS build — the OV2640's silicon
sensitivity falls off steeply past 850 nm, so you lose most of the signal you
are filtering for. A 700 nm filter works but lets more of the visible red end
through, which raises the blob noise floor you will be fighting in the camera
tuning step. The wiicam build is the opposite case — see its section below.

**The camera must be the IR-cut-free version.** A normal OV2640 has a filter
glued over the sensor that blocks exactly the light you want. Some modules let
you scrape it off; buying the NV version is easier.

**Hardware — RP2040 + wiicam build**

| Part | Specification |
|---|---|
| Board | Any RP2040 board stock OpenFIRE supports (Pico etc.) |
| Sensor | DFRobot SEN0158 / Wii camera, on the I2C pins your board's OpenFIRE pinout expects (see OpenFIRE's BOARDS.md) |
| Cable | One USB cable |

The wiicam does blob detection on its own chip, so there is no camera to tune —
its three sensitivity levels replace that step, and its stock 33° lens needs no
correction.

**Why 940 nm here.** The Wii camera was designed around the Wii sensor bar,
which is 940 nm, and its built-in filter is matched to it — so on this build
940 nm LEDs are the right choice, not 850 nm.

**Wide and fisheye lenses cost signal.** A fisheye dims off-axis LEDs
(vignetting plus the extra glass) and shrinks every blob, and the wiicam's
on-chip detector silently drops whatever falls below its threshold — LEDs
vanish long before they reach the frame edge. Use at least **High** sensitivity
with a very large FOV, and go up a level if the live preview is choppy; add LED
power or stand closer if dropouts persist. Expect brief dropouts when aiming at
screen corners during play (the resolver coasts through them). Measure reports
the dropout count after every sweep.

**Both builds**

| Part | Specification |
|---|---|
| IR LEDs | 4 ×, one per screen corner, **5 W class recommended** — **850 nm for the ESP32/OV2640 build, 940 nm for the RP2040/wiicam build** (wavelength notes above) |
| Trigger | A button wired per OpenFIRE's pinout |

The LED rectangle does not have to match your screen — the calibration measures
whatever shape it is, so the LEDs can sit at the four corners of the screen
instead of the usual top/bottom bars. It does need to be rigid. Tape sags as
the resistors warm up, and the calibration screen will tell you if it moved
while you were working. Not tested or supported on diamond shape.

**Software**

Fully tested using Visual Studio Code. Procedure, below not fully tested but should work.  

You need Python 3.9+, PlatformIO Core, `git`, and two Python packages.

Windows only — the OpenFIRE desktop app does not support anything else.
Install [Python](https://www.python.org/downloads/) (tick "Add python.exe to
PATH") and [Git for Windows](https://git-scm.com/download/win), then in a new
terminal:

```
python -m pip install --upgrade platformio
python -m pip install pyserial numpy
```

Then confirm all four in one go. Every line must print, with no traceback:

```
git --version
python -m platformio --version
python -c "import serial, numpy, tkinter; print('python deps ok')"
```

If `platformio` is not found afterwards, its scripts folder is not on your PATH;
`python -m platformio` works regardless and every command below can be written
that way.

Also download the [OpenFIRE desktop app](https://github.com/TeamOpenFIRE/OpenFIRE-App).
It is only used to set buttons and pins in step 1 — it is not needed to build. Add it in the root of the project. 

---

## 2. Build and flash

> **Do step 1 first.** This folder does not contain OpenFIRE, so a fresh
> checkout has nothing to compile until the patcher has run. If you build
> first, it stops with a plain message naming what to run.

**1. Get OpenFIRE and apply the overlay — one command per board.**

ESP32-S3:

```
python tools/patch_openfire.py --fetch
```

RP2040:

```
python tools/patch_openfire.py --board rp2040 --fetch
```

Each clones its own upstream next to this project, checks out the exact commit
the overlay is built against, applies the patches, and pulls submodules where
the upstream uses them. The two checkouts can coexist in one folder.

**About the pinned commits.** The overlay replaces a block of OpenFIRE's own
aiming code, so each patch is tied to one upstream revision, recorded with file
fingerprints in `patches/upstream.json`:

| Board | Upstream | Pinned |
|---|---|---|
| esp32 | alessandro-satanassi/OpenFIRE-Firmware-ESP32 | `f8f9bf265c48` (2026-07-17) |
| rp2040 | TeamOpenFIRE/OpenFIRE-Firmware | `8b651a2` (2026-04-19) |

On any other revision the patch **refuses and writes nothing at all**, naming
the line it could not place — upstream can move freely, and can lift the
approach whole if they deem it worth it.

Check it landed before building:

```
python tools/check_setup.py
```

It prints `setup OK` per board found, or names exactly what is still missing.
The build runs the same check itself.

**2. Plug in.**

*ESP32-S3:* the board has two USB-C sockets. Use the one wired straight to the
chip's own USB peripheral — **not** the USB-to-UART bridge. The gun's HID mouse
and the `~` serial channel both live on the native port; flashing over the UART
port will appear to work and then no tool will find the gun. How to tell them
apart: the UART port enumerates as a serial device the moment you plug it in;
the native port only enumerates once firmware is running, and after this
firmware is flashed it also shows up as a mouse.

*RP2040:* one socket, nothing to get wrong. For the very first flash the board
has no firmware to answer on, so hold **BOOTSEL** while plugging in.

Find the port either way:

```
python tools/list_ports.py
```

A flashed gun of either board shows as VID `F143`.

**3. Build and upload.**

ESP32-S3:

```
pio run -e combined_s3_freenove -t upload --upload-port COM8
```

Replace `COM8` with what `list_ports.py` reported. If the upload cannot
start, hold **BOOT**, tap **RESET**, release **BOOT**, and run it again.

RP2040:

```
pio run -c platformio_rp2040.ini -t upload
```

No port needed: the uploader finds the gun, reboots it into BOOTSEL, waits for
the RPI-RP2 drive to mount and copies the UF2 itself. First-ever flash (board
in BOOTSEL by hand): same command, it copies straight away. In VS Code,
Terminal → Run Task has one-click entries for the RP2040 fetch, build and
upload.

**4. Confirm it booted.** Open the serial monitor (`pio device monitor`). Both
boards print an `AIM:` line once they reach Run mode:

```
AIM: pipeline ready (v30) -- send ~ping or ~aimcal?          <- ESP32-S3
AIM: pipeline ready (rp2040-wiicam) -- send ~ping or ~aimcal?  <- RP2040
```

If OpenFIRE starts and no `AIM:` line appears, the overlay is not in the build —
check that the patcher actually ran.

---

## 3. Set it up

```
python tools/gun_studio.py
```

Studio reads which board is connected from the gun itself and adapts. Six
steps, in order. Order matters: aim accuracy is limited by sensor noise, so
checking the noise floor before calibrating is not optional — the app blocks
the calibration step if it is too high.

**1 — Buttons & pins** *(both boards)*. Opens the OpenFIRE desktop app, which needs the serial port
to itself. You do not need to run OpenFIRE's own calibration — this build skips
its first-boot hold and goes straight to Run mode, because the overlay stores
its calibration separately and steps 4 to 6 replace that job. Set your trigger
and pins there, then just close it — Studio waits for it to exit, takes the port
back and reconnects on its own. Reconnect in the header is there if you ever need
to force it.

**2 — Camera tuning.** *ESP32-S3:* exposure, gain and threshold. Aim for a blob
noise floor **under 0.30 px**; 0.60 is the limit. `Auto` sweeps for you and
applies a small safety margin (slightly higher threshold, slightly shorter
exposure) whenever the four-blob rate holds — headroom against background
light creeping up later. If background contamination still shows after Auto
(phantom blobs, jumpy quad), raise `thr` a step and lower `aec` slightly by
hand. The step 3 sweep doubles as a check: whenever fewer than four dots are
detected the preview freezes on the last full frame and says so, making
contamination and dropouts visible at a glance.
*RP2040:* the tab shows the wiicam's three sensitivity levels instead —
Default fits most rigs — and the noise floor is still measured, with limits
scaled for this sensor.

**3 — Lens / FOV** *(skip on both stock lenses)*. The stock 66° OV2640 lens and
the wiicam's 33° lens need nothing here. A wide or fisheye lens bends the LED
quad before it reaches the pipeline; the homography assumes a pinhole and the
calibration has nowhere to put a radially-varying error, so it must be
corrected upstream in the firmware — this step configures that.

- **Preset from FOV** — type the lens listing's field of view. A good first
  approximation for fisheye lenses; Measure beats it.
- **Measure** — a guided 20-second sweep. Stand at a respectable distance —
  whatever keeps all four LEDs in view with the quad still a good size
  (Measure warns if it is too small) — feet planted, and slowly pan/tilt/roll
  so the LEDs travel out to the image edges and corners.
- **Wiicam: raise the sensitivity first.** At least **High** for a very large
  FOV like a fisheye, and go up a level whenever the preview is choppy during
  the sweep — that choppiness is LEDs dropping below the sensor threshold, and
  Measure counts the dropouts and says so.
- Measure fits both distortion models, applies the better one, and refuses
  honestly when the sweep did not cover enough of the frame to pin the answer.
- **The typed FOV cannot sink a Measure.** The focal length is a fitted
  parameter, not a number taken on trust: Measure checks whether the typed
  field of view gives a focal the sweep can be fitted at, and looks for one
  that works if it does not. It reports the field of view it measured, which
  is often the honest answer for a lens whose box says otherwise. Note that
  focal length and distortion strength are not separately identifiable — a
  longer focal with more barrel gives the same mapping scaled by a constant —
  so the fitted numbers may not match the lens spec even when the correction
  is right. The calibration absorbs that scale.
- **Save to gun** persists it; it reloads at every boot.
- **A decentered lens leaves a one-sided error — and Measure now fits it.**
  A clip-on lens that is not perfectly coaxial has its distortion centre off
  the frame centre, which shows up as an offset that grows toward one edge.
  Measure searches for that centre automatically, applies it when it clearly
  improves the fit, and logs *"decentered lens detected"* with the offset.
  Re-seating the lens as centred as possible is still worth doing — the fit
  compensates most of a small decenter, not all of it — and a re-sweep with
  good coverage near the affected edge gives the search its best data.
- **Know the trade — smaller FOV wins.** A wide lens lets you stand closer,
  but every extra degree of lens angle makes each camera pixel cover more
  screen, so jitter grows. Smoothing hides the jitter but adds delay; lead
  can compensate the delay but adds its own overshoot. Each mechanism papers
  over the previous one — the chain works, and Studio tunes all of it, but
  the smallest FOV that still fits your play distance beats every layer of
  compensation. Pick the lens for the distance you actually play at.
- **Dead-band (optional, off by default).** The − / + control below the lens
  buttons. What it does: when the gun is at rest, sensor noise still wiggles
  the cursor by a pixel or two; the dead-band holds the cursor perfectly
  still until it moves MORE than the threshold from the last drawn position,
  then movement passes through instantly — it never adds delay to real
  motion. When to use it: only if the cursor still shimmers at rest after
  tuning smoothing (step 5) — typical with a wide lens. Start at 16, raise
  by 8 until the rest shimmer stops, and stop there: too high and slow
  deliberate drags start to step. **Save to gun** keeps it.
- **Applying or changing a lens correction invalidates the aim calibration** —
  redo step 4, then step 5, after every change here.
- **Smoothing speed sensitivity (`beta`).** The One Euro filter's cutoff is
  `min_cutoff + beta x speed`. The **Smoothing** level sets `min_cutoff` — how
  heavy the filter is **at rest**; `beta` sets how quickly it lets go once the
  gun **moves**. The two are independent, which is why beta is its own knob.
  Default 15 at every smoothing level, useful range roughly 12–35, `-1`
  restores the default. **Save to gun** keeps it.

  **Where it actually bites, in numbers.** At smoothing level 3 the filter lag
  is `1 / (2 pi (3.5 + beta x speed))`, with speed in screen widths per second.
  On a 1.2 m wide screen, going from beta 15 to beta 30:

  | hand speed | lag at beta 15 | lag at beta 30 | trail removed |
  |---|---|---|---|
  | 0.1 m/s (settling on a target) | 34 ms | 27 ms | 0.7 mm |
  | 0.25 m/s (deliberate drag) | 24 ms | 17 ms | 1.9 mm |
  | 1.0 m/s (tracking) | 10 ms | 6 ms | 4.4 mm |
  | 4.0 m/s (fast swipe) | 3 ms | 2 ms | 5.8 mm |

  So beta buys the most **time** on a slow deliberate drag and almost none on a
  swipe, where the filter is already nearly transparent. It never buys more
  than about 6 mm of trail. **The trail you see on a fast swipe is the pipeline
  delay, not the filter** — 2 m/s against a 20 ms delay is 40 mm on its own.
  That is `lead`'s job, not beta's. If 15 and 30 look identical to you on a
  swipe, that is the expected result, not a broken knob.
- **Judging lag: which cursor you are looking at matters.** Studio, the
  fine-tune bar and pical on a desktop all use the **system** cursor, which the
  OS moves straight from the gun's HID report — that one is honest. pical on a
  Pi console has no system cursor to use, so it **draws** one, and a drawn
  crosshair can only move when the app draws a frame: it is sampled once per
  loop and presented a frame later, adding roughly 17–33 ms of *app* latency
  that has nothing to do with the gun (more if the loop misses 60 fps). That is
  fine for aligning iron sights, which is a static task, but it is not a lag
  reference. To compare settings by feel on a Pi: set them, `~camsave`, then
  judge against a game rather than against the calibration screen.
- **A single-FIR temporal mode exists but is compiled out** (`AIM_FIR_MODE`).
  It replaced the One Euro filter and the lead with one least-squares fit, so
  that smoothing and prediction stopped fighting each other. It won in
  simulation and lost on hardware: a fixed window cannot reproduce One Euro's
  speed-adaptive cutoff, which is the property that actually makes the shipped
  filter feel right. The code and its tests are kept — a measured negative
  result is worth keeping — but the shipped build cannot select it.

**4 — Aim calibration** *(both boards)*. Five dots at each of two or three distances. Aim, pull
the trigger four times per dot. Note the live preview refreshes slower than the
gun actually tracks — the display is rate-limited for the serial link and the
GUI; the gun itself runs at full frame rate and every shot uses full-rate data. Stepping back between rounds is **required** —
at one distance the boresight and the screen mapping cannot be separated, and
the fit will refuse. It ends by sending the calibration to the gun and reading
it back to confirm.

**5 — Fine tune** *(both boards)*. Lines the cursor up with your **iron sights**, which is not
where the camera points. It opens showing the gun's SAVED lead, smoothing and beta,
so the first press steps from where you left off. Do it in this order:

1. **Align first.** Shoot the ring with your iron sights, nudge with the
   arrows if needed, step back and repeat — two positions let it separate an
   angular offset (grows with distance) from a parallax one (constant), and
   the wrong correction is worse than none. Short flow: skip the ring, nudge
   until the cursor sits on your notch, press **SAVE NOW**. Mixing them is
   safe: a ring shot only measures what is left after your nudges.
2. **Then SMOOTH ± (0–10)** until the cursor's tracking feels consistent —
   raise it while the cursor jitters, stop when it starts to feel floaty.
   Wide and fisheye lenses usually want a level or two more than stock.
3. **Then BETA ±** if a slow deliberate drag feels sticky after step 2.
   `auto` follows the smoothing table (15); the range is 0–60 and stepping
   below 0 returns to `auto`. Expect a small effect: see the table in step 3
   above for what it is worth in milliseconds. It does **not** shorten the
   trail on a fast swipe.
4. **Then LEAD ±, last** — smoothing changes the total latency, so lead
   tuned before smoothing no longer matches (the screen reminds you). Raise
   it while the cursor trails your swings, stop as soon as it overshoots
   when you reverse direction. This is the knob that shortens a swipe trail.

All three are saved with the fine tune, and the gun's own reply is read back
and reported: **SAVE** confirms what the gun actually wrote, so a refused or
partial write is shown as a failure instead of a hopeful "saved".
You will see a big difference here

**6 — Verify** *(both boards)*. Shoots a nine-point grid and reports the error of the pipeline
alone against the error after the OS. If the two agree, any remaining error is
the calibration's, not your driver's.

**F9** freezes the cursor while Studio is open, so the gun stops fighting you
for the mouse. It is released automatically for steps 4 to 6 and restored when
you quit. It is never saved — a power cycle always gives you the cursor back.

---

## 4. Troubleshooting

**Nothing on the serial port / the tools cannot find the gun**

```
python tools/list_ports.py
python tools/aim_probe.py --port COM5
```

`aim_probe` reports what the gun answers and what it does not. Use one cable, on
the board's **native USB** port. Do not run the OpenFIRE app at the same time —
it takes the port for itself.

**The reticle wanders, or the LEDs are hard to lock onto**

You are probably too close. When an LED is partly off the sensor its reported
centre is pulled inward, so the shape distorts and lock becomes unstable. The
calibration screen shows **TOO CLOSE — STEP BACK** when it happens. A quad span
around 65 px is comfortable.

**Blob noise floor will not come down**

Check the IR-pass filter is fitted and no sunlight or incandescent lamp is in
frame. Then re-run step 2's `Auto`. Bright LEDs with a short exposure beat dim
LEDs with a long one.

**Calibration refused**

- *"all shots were taken at effectively one distance"* — you did not step back far
  enough. Roughly 50 % further is enough.
- *"the LEDs are running off the edge of the camera"* — step back.
- *"fitted LED rectangle is implausible"* — the resolver locked onto something
  other than your four LEDs. Raise the threshold in step 2 and retry.

**`UnknownBoard: Unknown board ID 'ESP32-S3-WROOM-1-DevKitC-1-N8R2'`**

The definition now ships in `boards/` and resolves on its own. If you still see
this, your `platformio.ini` line should read `boards_dir = boards`.

**The build stops with `SETUP INCOMPLETE`**

That is the pre-build guard, and the box names which of the three states you are
in: OpenFIRE not downloaded, downloaded but not patched, or a partly-populated
checkout. Each one is fixed by running the command it prints. `python
tools/check_setup.py` runs the same check without starting a build.

**`undefined reference to 'cam_patch_chunk_cb'` at link time**

Your copy of `lib/esp32-camera-ov2640/` is missing its headers. PlatformIO only
links a private library that something includes, and `esp_camera.h` lives in
that library's `driver/include/` — with the headers gone the whole library is
silently left out of the build, and the first sign of it is this link error.
The folder should hold **17 files**, including `driver/include/`,
`driver/private_include/`, `sensors/private_include/` and `target/`. `python
tools/check_setup.py` reports this before a build starts.

**RP2040: the checkout's submodules are empty / missing `OpenFIREshared.h`** —
TeamOpenFIRE keeps its board definitions and bundled libraries as git
submodules. New fetches pull them automatically; an older checkout is fixed
with `git -C OpenFIRE-Firmware-RP2040 submodule update --init --recursive`.
The setup guard names this state.

**RP2040: upload says no BOOTSEL drive appeared** — hold **BOOTSEL** while
plugging the board in, then run the upload again; it copies straight away.

**RP2040: calibration refused with a negative rectangle width** — the sensor's
X axis is arriving un-mirrored. The default handles the SEN0158; for another
module flip it live with `~cam=mirx:0` (and `miry:1` if needed), then `~camsave`.

**The patcher says "CANNOT PATCH"**

Your OpenFIRE is not the pinned commit. Either use `--fetch` into a fresh
folder, or `git checkout f8f9bf265c48` in the copy you have. Rebasing the
overlay onto a newer `main` is a real (if not huge) piece of work: upstream is
building its own camera abstraction, so a future version may offer a cleaner
seam than patching.

**Calibration is not saved**

The results screen says either `INSTALLED ON THE GUN` or `NOT INSTALLED` with a
reason. It reads the value back from the gun before claiming success. If it
could not send, the line it prints can be pasted into a serial monitor by hand —
note the leading `~`, which is what makes the gun claim the line instead of
passing it to OpenFIRE:

```
~aimcal=0.512161,0.535141,0.401183,1.241383,12.156,9.125
```

**Aim was right, then hours later it is off by about a centimetre**

Almost certainly mechanical, not software — the aim path was measured over two
simulated hours of a motionless gun and drifts under 0.1 screen px, and the
32-bit microsecond clock wrap at ~72 minutes moves the cursor by 0.0000 px.
Both are asserted by `hostcheck/long_run_drift_test.cpp`.

What does move is the hardware, and the boresight is an *angular* reference, so
the master gain (~26 screen px per camera px) multiplies any mechanical shift:

| the camera shifts in its mount by | cursor moves |
|---|---|
| 0.25 sensor px (0.08 deg) | 3.6 px |
| 1 sensor px (0.31 deg) | 14 px |
| 2 sensor px (0.62 deg) | 28 px |

A third of a degree is a centimetre on screen. A camera that can settle in a
taped or printed mount as it warms will do this. The LED rig moving matters far
less — a whole-rig 2 mm shift is 11 px, one LED slipping 5 mm is 14 px — because
the calibration measures the rectangle rather than assuming it.

Fix the camera mount rigidly before chasing this in software. Re-running step 4
takes about a minute and corrects it either way.

**Aim drifts as you move closer or further away**

The fine-tune split needs two positions far enough apart. Redo step 4 and step
well back for the second one.

**A freshly flashed gun blinks orange and never moves the cursor**

Stock OpenFIRE treats a profile whose four edge offsets are all zero as "never
calibrated" and holds at boot until you either pull the trigger into its own
calibration or set offsets from the desktop app. Since the overlay keeps its
calibration in a separate NVS namespace, those zeros mean nothing to it, and
the hold blocked the Run loop that emits the trigger markers step 3 needs — so
a new gun could not calibrate its way out. This build skips that hold. If you
still see it, you are running a firmware built before the fix. A camera that
failed to start still holds, and that one is a real fault: check the ribbon.

**The gun stops moving the cursor**

Studio froze it and did not get to restore it. Reopen Studio and press **F9**,
or unplug and replug — the freeze is never stored.


---

## 5. Verifying a build

```
bash hostcheck/check.sh
```

Compiles every build combination, runs the geometry, protocol and NVS tests,
and renders each GUI screen headlessly. Needs `g++`, and `Xvfb` for the GUI
checks (they are skipped with a notice if it is missing). Every line should end
in `OK`.

---

## 6. Useful serial commands

All are prefixed `~` on the native USB port.

| Command | Effect |
|---|---|
| `~ping` | Alive check — board, calibration state, uptime, boot count, reset reason |
| `~aimcal?` | Print the active calibration |
| `~aimcal=...` | Install and save a calibration |
| `~aimhid=0` / `=1` | Freeze / release the cursor (never saved) |
| `~cam?` | Camera settings, including `lead`, `smooth` and the lens correction |
| `~cam=thr:60,aec:40` | Tune the camera live |
| `~cam=lead:10` | Latency lead in ms, 0–50 |
| `~cam=smooth:5` | Cursor smoothing level 0–10 (0 = off, 3 = default) |
| `~cam=dead:16` | Output dead-band in cursor units, 0 = off (default); swallows rest shimmer, never delays real motion |
| `~cam=lens:2,lfeq:900,lfpx:840` | Set the lens correction live |
| `~cam=sens:1` | wiicam sensitivity 0–2 (RP2040 board only) |
| `~cam=beta:24` | Smoothing speed sensitivity, 0–60; −1 = default (15) |
| `~cam=fmt:1` | Sensor report format (wiicam only): 0 basic, 1 extended (adds each blob's size, 0–15), 2 full (adds the blob's bounding box and an 8-bit intensity). Basic by default; saved by `~camsave`. `~cam=ext:` is the old name and still works |
| `~cam=fullreg:85` | The byte written to the mode register for full mode — 85 (0x55) or 5 (0x05). The driver's working constants for the other two formats are the doubled nibble, so 0x55 is the default; try 5 if full mode returns nonsense. Not saved |
| `~cam=bmin:2,bmax:9` | Blob size window. Blobs outside it are dropped before the quad resolver, so a bright window is refused instead of taking an LED's slot. Needs `fmt:1` or higher; 0–15 accepts everything |
| `~cam=rtol:3` | Odd-one-out gate, in blob-size steps (0–15). Drops a blob whose size is more than this far from the other three in the same frame. Needs no distance tuning; 0 = off |
| `~cam=hwmax:150` | The sensor's OWN maximum blob size, register 0x06. Gates inside the camera, before it allocates its four object slots. −1 leaves the register alone |
| `~cam=hwmin:3` | The sensor's own minimum blob size, register 0x1B — never written by the stock driver, so it otherwise sits at an unknown default. −1 leaves it alone |
| `~camblob?` | Each blob the sensor last reported — position, size, and whether a gate kept it, plus its bounding box and intensity in full mode — the share of recent frames that saw four, three or two LEDs, how many rejected blobs sat nowhere near a corner versus exactly where a missing LED should have been (the false-negative meter for a size window that is too tight), and the gun's frame counter and clock, from which the camera's true frame rate is measured |
| `~camreset` | Undo everything that can stop a gun aiming: lens, lead, the software blob gate and its saved copy, and the sensor's own thresholds, which go back to the sensitivity preset |
| `~camlearn=on:1` | Start measuring what an LED actually looks like on this rig. Two histograms per feature — blobs the quad resolver confirmed as corners, and blobs it placed nowhere near one — so the question "can a window and an LED be told apart at all" is answered from data instead of guessed. Needs `fmt:2` for anything beyond size. Starting clears; nothing is gated on it and nothing is saved |
| `~camlearn?` | The histograms: a summary line, then one line per class and feature |
| `~camlearn=reset` | Clear the capture without stopping it |
| `~camdiag` | Sensor connection test: power, both data wires, swapped lines, the sensor itself, and whether frames actually flow |
| `~camsave` | Persist camera settings, lead, smoothing, beta, dead-band, lens, temporal mode and the software blob gate (`fmt`, `bmin`, `bmax`, `rtol`). The two SENSOR thresholds `hwmax` and `hwmin` are deliberately left out: they are the only settings that can leave a gun dark, so a power cycle has to remain a way out. Replies `CAM: saved ...` (or `CAM: SAVE FAILED ...`) listing the values written, so a tool can verify rather than assume |
| `~fx?` | Recoil engine state: every knob, dry-fire and quiet countdowns, and the trigger path's last temperature reading |
| `~fx=on:1,drive:45,hold:0` | Tune the recoil engine live. `on:0` is stock OpenFIRE behaviour |
| `~fx=quiet:1` / `:0` | Silence the gun: nothing fires, and the engine holds both the solenoid and the rumble motor so OpenFIRE's own recoil cannot run either. Used by the calibration screens; lapses by itself after five minutes |
| `~fx=ab:1` | Dry-fire mode — the trigger fires the solenoid with no IR lock. Expires after ten minutes |
| `~fx=test:1` | Fire one sequence now, no trigger and no IR needed |
| `~fxsave` | Persist the recoil knobs |

## Calibrating without a PC

Studio needs Windows. Moving the gun to another TV — a Pi cabinet, a friend's
living room — normally means carrying a computer to it, so the same
calibration also runs from the TV itself.

**`pical`** (in `pical/`) runs every step of *3. Set it up* from **step 2**
onwards: camera tuning, lens/FOV, aim calibration, fine tune and verify. Only
step 1 — buttons and pins, in the OpenFIRE desktop app — is missing, because
that app is Windows-only, and it only has to be done once when the gun is
first built. Follow section 3 above as written; it applies unchanged, the
screens simply live on the TV instead of in a Windows window.

It is not a second implementation — the capture session, the fits, the
auto-tune and the serial link all come from `tools/`, so both front ends stay
in step.

| | What you need |
|---|---|
| **USB stick** | [**download the latest image**](https://github.com/frosted123456/-LIGHTGUN-STUDIO_OpenFire_RP2040_AND_ESP32/releases/download/latest/pical-latest.img.xz) (or pick a version from Releases), write it with Raspberry Pi Imager (or balenaEtcher / Rufus), plug the stick and the gun into a Pi 3/4/5/Zero 2 W and power on |
| **On a PC** | `python pical/pical.py` — fullscreen, or `--windowed` |
| **Batocera** | copy `pical/` into `roms/ports` — *experimental, untested* |

The stick touches nothing on the machine: your own SD card stays as it is, no
network is needed, and the calibration screen comes up by itself. You aim with
your **iron sights**, so a gun whose aim is completely offset still calibrates
— the shots are read over serial, not from the cursor. A game controller, a
mouse or a keyboard drives the menus.

The app sits on the stick's FAT partition, so a newer version can be dropped
on from any PC without rebuilding the image, and every session is logged
beside it. `pical/README.md` has the details, including the Pi 3B USB-boot
caveat and how to build the image yourself.

On the TV you get the same camera view Studio shows — the four LEDs, the quad
and the frame centre — on the camera, lens and fine-tune screens, and the gun's
cursor is drawn by the app, since a Pi console has no desktop to draw one.

Studio remains the more comfortable place for a long tuning session, and it is
the only place step 1 can be done. Everything else is available in both.

See `NOTICE.md` for third-party code and licences.
