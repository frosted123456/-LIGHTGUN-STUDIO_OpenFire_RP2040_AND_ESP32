# pical — Studio on the TV, without a PC

Moving the gun to another TV normally means carrying a computer to it. `pical`
runs the Studio steps that do not need Windows, from the TV itself:

Everything from **step 2 of the main README's setup section** onwards is here.
Only step 1 is missing, and only because the app it needs runs on Windows.
Step 4b is pical's own: it is offered at the end of a calibration, it is
skippable with one press, and skipping it leaves the calibration finished.

| Step | In pical | Notes |
|---|---|---|
| 1 buttons & pins | no | the OpenFIRE app is Windows-only; do this once on a PC |
| 2 camera tuning | yes | sliders and auto-tune, or the wiicam's sensitivity |
| 3 lens / FOV | yes | preset, 20 s measured sweep, dead-band |
| 4 aim calibration | yes | five dots at two or three distances |
| 4b room light sweep | yes | optional, ~15 s: measures your LEDs against the room so the gun can gate out a lamp |
| 5 fine tune | yes | iron sights, then smoothing, then lead |
| 6 verify | yes | nine shots, pipeline error vs the OS cursor |

There is no second implementation of anything. `pical.py` is a view; the
capture session, the fits, the auto-tune and the serial link all come from
`tools/`, so a change to the maths reaches Studio and pical together.

## Three ways to run it

| | What you need | Who it is for |
|---|---|---|
| **USB stick** | the released `.img.xz`, any Pi 3/4/5/Zero 2 W | anyone — your own SD card is never touched |
| **On a PC** | `python pical/pical.py` | development, and a lighter alternative to Studio |
| **Batocera port** | copy this folder to `roms/ports` | *experimental, untested* |

## The USB stick

[**Download the latest image**](https://github.com/frosted123456/-LIGHTGUN-STUDIO_OpenFire_RP2040_AND_ESP32/releases/download/latest/pical-latest.img.xz)
— that link always points at the newest build of `main` and never changes.
Tagged versions are on the repository's Releases page. Write it with
**Raspberry Pi Imager** (it reads `.xz` directly), balenaEtcher, or Rufus.
Plug the stick and the gun into the Pi and power it on: the calibration screen
comes up by itself, with no network and nothing installed on the machine.

Pi 3B (the original, not the 3B+) needs USB boot enabled once from an SD card,
or just write the image to an SD card instead.

The app lives on the stick's FAT partition, so a newer `pical.py` can be
dropped on from Windows, macOS or Linux without rebuilding the image. Every
session is logged to `pical/calib_out/` on that same partition.

## What is on the screen

- **Camera view** — the sensor's own picture: the four LEDs, the quad joining
  them, a trail of recent positions and the frame centre. Every number in the
  app comes from those four dots, so a jumping or dropping-out quad explains a
  bad result before you measure anything. It is on the camera, lens and
  fine-tune screens.
- **Cursor** — the gun's pointer, drawn by the app. The Pi has no desktop to
  draw one, so pical draws it. It is deliberately hidden during a lens sweep
  and during dot capture, where the pointer is not the pipeline's own aim.

## Step 3, lens / FOV

- **Field of view** is what the lens is sold as. `Apply preset from FOV` uses
  it directly. `Measure` uses it only as a starting point: if that focal does
  not fit the sweep it finds one that does, and reports the field of view it
  measured. A wrong number in this box no longer costs you the fit.
- Preset and Measure both apply the correction **live**. Nothing is permanent
  until `Save to gun`, which writes whatever is live at that moment — the row
  shows which correction that is.
- The sweep screen shows the three gates the fit will apply — frames,
  coverage, quad span — while you sweep, plus a coverage map with a ring at
  the coverage gate. Push the LEDs past that ring or the fit will refuse.
- A refused sweep prints the numbers, the reason and what to change, and puts
  the correction you had back. Every sweep is saved to `calib_out/` whether it
  passes or not.
- Fitting takes up to two minutes on a Pi. It runs in the background, so the
  screen keeps updating and Esc still works.

## Step 5, fine tune

Five rows, driven like every other screen: **up/down** picks a row,
**left/right** changes it, and any controller button steps through the rows.
The sight offset has one row per axis so that up and down are never taken
away from moving between rows.

| Row | What it does |
|---|---|
| Sight left / right | moves the cursor across, relative to your iron sights |
| Sight up / down | the same, vertically |
| Smoothing (at rest) | raise until rest jitter settles; stop when it feels floaty |
| Speed sensitivity | how fast smoothing lets go once you MOVE; raise if swipes trail |
| Lead | raise while the cursor trails; stop when reversals overshoot |

Do them in that order — smoothing changes the latency that lead is
compensating for, and the screen says so when you change it.

**Which cursor you get depends on the platform.** On a PC the app hands its
crosshair to the **system** cursor, which the OS moves straight from the gun's
reports — no app lag. On the Pi console there is no system cursor, so one is
drawn, and a drawn cursor updates at the app's own frame rate — the HUD's
`app N ms` figure is exactly that interval, measured. Two switch files on the
stick change the Pi behaviour (create them empty next to `pical.py`, delete to
undo):

| file | effect |
|---|---|
| `HWCURSOR` | moves the cursor on the display chip's own cursor layer, at input-pump rate instead of frame rate. Less cursor lag; costs CPU. If the cursor disappears, delete the file |
| `NOKMS` | skips the direct display path and starts the X server, where the server moves the cursor from the gun's reports like a desktop does |

Even so, to compare filter settings by feel, save them to the gun, quit, and
judge on a desktop cursor or in a game — not against a calibration screen.

## Running it

The menu lists the steps in order. Run them in that order the first time: aim
error scales with blob noise, and a lens change invalidates the calibration.

1. Aim at each dot with your **iron sights**. The cursor is irrelevant here —
   the gun is read over serial, so an aim that is completely off still
   calibrates.
2. Pull the trigger four times per dot. With no trigger wired, hold still on
   the dot and it captures itself; a controller button works too.
3. Step back when it asks and do it again. The distances must differ or the
   sight offset cannot be separated from the screen mapping.
4. It writes the result to the gun and says so.

Menus take a game controller, a mouse or a keyboard.

## On a PC

```
python pical/pical.py --windowed        # a window, for development
python pical/pical.py                   # fullscreen, same as the stick
python pical/pical.py --stances 2       # two distances instead of three
```

Studio remains the place for step 1 (buttons and pins, through the OpenFIRE
app) and is the more comfortable environment for a long tuning session.
Every step after that one is available in both.

## Building the image

CI does it: `.github/workflows/pical-image.yml` runs `image/build.sh` on every
push that touches this folder. A push to `main` replaces the image on the
rolling `latest` prerelease, so the download link above is always current; a
`v*` tag cuts a proper release instead. Either way the Actions run page ends
with a clickable download link. To build one by hand on any Linux box with
root:

```
sudo bash pical/image/build.sh          # writes pical.img.xz
sudo bash pical/image/smoke.sh          # packaging test, no network needed
```

`smoke.sh` builds against a synthetic base image, so it proves the grow,
payload, shrink and partition-table steps in about twenty seconds. It exists
because a build can fail in a way that still produces a file: `parted -s
resizepart` answers *no* to its own shrink prompt, which once left a truncated
image with a partition table pointing past the end of the disk.
