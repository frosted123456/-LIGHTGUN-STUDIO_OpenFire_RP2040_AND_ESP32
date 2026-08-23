#!/bin/sh
# Starts the calibration app on the console, and leaves evidence behind.
#
# Everything it prints is copied to the FAT boot partition, so when the TV
# shows nothing useful the stick can be read on any PC. SDL is tried on each
# video driver in turn rather than being forced onto one, because which of
# them works depends on the Pi model and the display.
APP=/boot/firmware/pical
LOG="$APP/last-run.log"
export SDL_AUDIODRIVER=dummy
export PICAL_OUT="$APP/calib_out"
export PYTHONUNBUFFERED=1

log() { echo "$*" | tee -a "$LOG"; }

# A fresh log per boot, with enough context to place the failure.
{
    echo "=== pical $(date -u '+%Y-%m-%d %H:%M:%S UTC') ==="
    echo "model:   $(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"
    echo "tty:     $(tty 2>/dev/null)"
    echo "user:    $(id -un) ($(id -u))"
    echo "python:  $(python3 -V 2>&1)"
    echo "drm:     $(ls /dev/dri 2>/dev/null | tr '\n' ' ')"
    echo "serial:  $(ls /dev/ttyACM* /dev/ttyUSB* /dev/lightgun 2>/dev/null | tr '\n' ' ')"
} > "$LOG" 2>&1

if [ ! -f "$APP/pical.py" ]; then
    log "!! $APP/pical.py is missing -- the stick was not built correctly"
    exit 1
fi

python3 -c "import pygame, numpy, serial" >> "$LOG" 2>&1 || {
    log "!! python3 is missing pygame, numpy or pyserial (see $LOG)"
    exit 1
}

# kmsdrm is the normal path on a Pi; the rest are fallbacks so a driver quirk
# shows up as a working screen rather than a blank one.
for DRV in kmsdrm fbcon directfb x11 ""; do
    if [ -n "$DRV" ]; then
        export SDL_VIDEODRIVER="$DRV"
        log "-- trying SDL video driver: $DRV"
    else
        unset SDL_VIDEODRIVER
        log "-- trying SDL's own choice of video driver"
    fi
    python3 "$APP/pical.py" "$@" >> "$LOG" 2>&1
    rc=$?
    if [ "$rc" = "0" ]; then
        log "-- app exited normally"
        exit 0
    fi
    log "-- that driver failed (exit $rc); last lines:"
    tail -n 6 "$LOG" | sed 's/^/     /'
done

log "!! no SDL video driver worked. The full log is on this stick at"
log "!! pical/last-run.log -- read it on any PC."
exit 1
