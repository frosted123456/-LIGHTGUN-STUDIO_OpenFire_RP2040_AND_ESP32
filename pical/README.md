# pical — calibrate without a PC

Moving the gun to another TV normally means carrying a computer to it. `pical`
is the same calibration, driven from the TV itself: a pygame app over the same
capture, fit and serial dialect the desktop tools use.

There is no second implementation of anything. `pical.py` is a view; the
capture state machine and the fit come from `tools/aim_calib.py` and
`tools/aim_fit.py`, so a change to the maths reaches every front end at once.

## Three ways to run it

| | What you need | Who it is for |
|---|---|---|
| **USB stick** | the released `.img.xz`, any Pi 3/4/5/Zero 2 W | anyone — your own SD card is never touched |
| **On a PC** | `python pical/pical.py` | development, and a lighter alternative to Studio |
| **Batocera port** | copy this folder to `roms/ports` | *experimental, untested* |

## The USB stick

Download `pical-*.img.xz` from the repository's Releases page and write it with
**Raspberry Pi Imager** (it reads `.xz` directly), balenaEtcher, or Rufus.
Plug the stick and the gun into the Pi and power it on: the calibration screen
comes up by itself, with no network and nothing installed on the machine.

Pi 3B (the original, not the 3B+) needs USB boot enabled once from an SD card,
or just write the image to an SD card instead.

The app lives on the stick's FAT partition, so a newer `pical.py` can be
dropped on from Windows, macOS or Linux without rebuilding the image. Every
session is logged to `pical/calib_out/` on that same partition.

## Running it

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

Studio remains the fuller tool on Windows: camera tuning, lens fitting, fine
tune and verify. `pical` covers step 4 only.

## Building the image

CI does it: `.github/workflows/pical-image.yml` runs `image/build.sh` on every
push that touches this folder, and attaches the image to the release when a
`v*` tag is pushed. To build one by hand on any Linux box with root:

```
sudo bash pical/image/build.sh          # writes pical.img.xz
sudo bash pical/image/smoke.sh          # packaging test, no network needed
```

`smoke.sh` builds against a synthetic base image, so it proves the grow,
payload, shrink and partition-table steps in about twenty seconds. It exists
because a build can fail in a way that still produces a file: `parted -s
resizepart` answers *no* to its own shrink prompt, which once left a truncated
image with a partition table pointing past the end of the disk.
