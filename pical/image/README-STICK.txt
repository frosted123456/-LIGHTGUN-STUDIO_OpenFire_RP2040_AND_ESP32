LIGHTGUN CALIBRATION STICK
==========================

Plug this stick and your lightgun into the Pi and power it on. The
calibration screen comes up by itself -- there is no login, no setup
questions and nothing is installed on the machine. Your own SD card is not
touched; take it out first, because the Pi boots the SD card before USB.

  1. Aim at each dot with your IRON SIGHTS, not the cursor.
  2. Pull the trigger four times per dot.
  3. Step back when it asks, then do it again.
  4. It saves the result to the gun by itself.

A game controller, a mouse or a keyboard all drive the menus.

SHUTTING DOWN
  Press Esc to leave the app, then type: poweroff
  Wait for the activity light to stop before pulling the power. Cutting
  power mid-write can damage the stick's filesystem.

FILES ON THIS PARTITION
  pical.py      the calibration app
  tools/        the shared capture and fit code
  calib_out/    every session is logged here, readable from any PC

UPDATING
  Replace pical.py (and tools/) from a newer release. This partition is
  FAT, so Windows, macOS and Linux can all write to it directly.

SWITCH FILES (create them empty, next to pical.py, from any PC)
  NOAUTOSTART   boot to a root shell instead of the app, for a look around.
                From that shell:
                    pical-launch            run the app by hand
                    journalctl -b | tail    what happened during this boot
  HWCURSOR      move the cursor on the display chip's own cursor layer, at
                input rate instead of frame rate. Less cursor lag; costs
                some CPU. If the cursor ever disappears with this on,
                delete the file.
  NOKMS         skip the direct display path and start the X server
                instead. There the SERVER moves the cursor from the gun's
                reports, like a desktop does. Use to compare cursor lag,
                or if the direct path shows no picture.
  Delete a file to undo it. All three survive updates to pical.py.
