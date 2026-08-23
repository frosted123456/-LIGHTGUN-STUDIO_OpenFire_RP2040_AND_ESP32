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
Note Lightgun studio use a preview at a lower refresh rate (for preview purpose obviously). Actual refresh rate is define by the cam used.   

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
noise floor **under 0.30 px**; 0.60 is the limit. `Auto` sweeps for you.
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
- **Save to gun** persists it; it reloads at every boot.
- **A decentered lens leaves a one-sided error.** Both distortion models are
  radially symmetric about the frame centre — the standard assumption, and
  exactly what a clip-on lens that is not perfectly coaxial breaks. A small
  decenter puts the true distortion centre off to one side, and the residual
  shows up as an offset that grows toward that edge. To verify, re-sweep with
  good coverage close to that edge: if the offset survives a clean sweep it is
  mechanical, and re-sweeping only measures it more precisely. Re-seat the
  lens as centred as possible; the fine tune absorbs part of what remains.
  (Fitting the distortion centre itself is a possible future extension — not
  currently implemented.)
- Know the trade: a wide lens lets you stand closer, but the shorter focal
  magnifies every noise source on screen.
- **Applying or changing a lens correction invalidates the aim calibration** —
  redo step 4, then step 5, after every change here.

**4 — Aim calibration** *(both boards)*. Five dots at each of two or three distances. Aim, pull
the trigger four times per dot. Note the live preview refreshes slower than the
gun actually tracks — the display is rate-limited for the serial link and the
GUI; the gun itself runs at full frame rate and every shot uses full-rate data. Stepping back between rounds is **required** —
at one distance the boresight and the screen mapping cannot be separated, and
the fit will refuse. It ends by sending the calibration to the gun and reading
it back to confirm.

**5 — Fine tune** *(both boards)*. Lines the cursor up with your **iron sights**, which is not
where the camera points. The full flow: shoot the ring, nudge with the arrows if
needed, step back, repeat — two positions let it separate an angular offset
(grows with distance) from a parallax one (constant), and the wrong correction
is worse than none. The short flow: skip the ring entirely, nudge with the
arrows until the cursor sits on your notch, and press **SAVE NOW** — that keeps
exactly what you see as a constant offset from one position. Mixing them is
safe: a ring shot only measures what is left after your nudges, so nothing is
ever counted twice. `LEAD ±` trades latency for overshoot — raise it while the
cursor trails you, stop as soon as it overshoots when you reverse direction.
`SMOOTH ±` (0–10) trades cursor jitter for glide — raise it until the jitter
settles, stop when the cursor starts to feel floaty. Wide and fisheye lenses
magnify sensor noise, so they usually want a level or two more than stock.
Both are saved with the fine tune.
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
| `~cam=lens:2,lfeq:900,lfpx:840` | Set the lens correction live |
| `~cam=sens:1` | wiicam sensitivity 0–2 (RP2040 board only) |
| `~camsave` | Persist camera settings, lead, smoothing and lens |

See `NOTICE.md` for third-party code and licences.
