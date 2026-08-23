#!/bin/sh
# Runs the calibration app on the console framebuffer, no desktop.
# The app lives on the FAT boot partition so it can be replaced from any PC.
APP=/boot/firmware/pical
export SDL_VIDEODRIVER=kmsdrm
export SDL_AUDIODRIVER=dummy
export PICAL_OUT="$APP/calib_out"
# A stale lock from an unclean shutdown must not keep the app off the screen.
[ -w "$APP" ] || echo "pical: $APP is not writable; logs will not be saved"
exec /usr/bin/python3 "$APP/pical.py" "$@"
