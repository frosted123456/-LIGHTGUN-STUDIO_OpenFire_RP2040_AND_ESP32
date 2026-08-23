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
    echo "egl:     $(ls /usr/lib/*/libEGL.so.1 /usr/lib/*/dri/*_dri.so 2>/dev/null \
                     | head -4 | tr '\n' ' ')"
    echo "connectors:"
    for st in /sys/class/drm/card*-*/status; do
        [ -e "$st" ] || continue
        echo "         $(basename "$(dirname "$st")") = $(cat "$st" 2>/dev/null)"
    done
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

# SDL2's video drivers are kmsdrm, wayland, x11, offscreen and dummy -- the
# SDL1 names (fbcon, directfb) do not exist and only waste a retry. kmsdrm is
# the normal path on a console-only Pi; the empty entry lets SDL choose.
# The app picks the DRM card with a connected output by itself. If that
# still draws nothing, walk the device indexes explicitly before giving up
# on kmsdrm: on a Pi one card is the v3d render node with no screen on it.
for IDX in "" 0 1 2; do
    [ -n "$IDX" ] || continue
    [ -e "/dev/dri/card$IDX" ] || continue
    KMS_IDXS="${KMS_IDXS:-} $IDX"
done

try_run() {
    python3 "$APP/pical.py" "$@" >> "$LOG" 2>&1
}

for DRV in kmsdrm wayland x11 ""; do
    if [ -n "$DRV" ]; then
        export SDL_VIDEODRIVER="$DRV"
        log "-- trying SDL video driver: $DRV"
    else
        unset SDL_VIDEODRIVER
        log "-- trying SDL's own choice of video driver"
    fi
    if [ "$DRV" = "kmsdrm" ]; then
        # first the card the app chose, then each one in turn
        for IDX in auto $KMS_IDXS; do
            if [ "$IDX" = "auto" ]; then
                unset SDL_KMSDRM_DEVICE_INDEX
                log "   kmsdrm: letting the app choose the card"
            else
                export SDL_KMSDRM_DEVICE_INDEX="$IDX"
                log "   kmsdrm: forcing device index $IDX"
            fi
            try_run "$@"
            rc=$?
            if [ "$rc" = "0" ]; then
                log "-- app exited normally"
                exit 0
            fi
            log "   that index failed (exit $rc)"
        done
        unset SDL_KMSDRM_DEVICE_INDEX
    else
        try_run "$@"
        rc=$?
        if [ "$rc" = "0" ]; then
            log "-- app exited normally"
            exit 0
        fi
    fi
    log "-- that driver failed (exit $rc); last lines:"
    tail -n 6 "$LOG" | sed 's/^/     /'
done

log "!! no SDL video driver worked. The full log is on this stick at"
log "!! pical/last-run.log -- read it on any PC."
if ! ls /usr/lib/*/libEGL.so.1 >/dev/null 2>&1; then
    log "!! libEGL is missing: this image was built without the Mesa stack,"
    log "!! which is what kmsdrm draws through. Rebuild with a newer image."
fi
exit 1
