LIGHTGUN CALIBRATION STICK
==========================

Plug this stick and your lightgun into the Pi, power it on, and the
calibration screen comes up by itself. Nothing is installed on the machine
and your own SD card is not touched.

  1. Aim at each dot with your IRON SIGHTS, not the cursor.
  2. Pull the trigger four times per dot.
  3. Step back when it asks, then do it again.
  4. It saves the result to the gun by itself.

A game controller, a mouse or a keyboard all drive the menus.

FILES ON THIS PARTITION
  pical.py      the calibration app
  tools/        the shared capture and fit code
  calib_out/    every session is logged here, readable from any PC

UPDATING
  Replace pical.py (and tools/) from a newer release. This partition is
  FAT, so Windows, macOS and Linux can all write to it directly.
