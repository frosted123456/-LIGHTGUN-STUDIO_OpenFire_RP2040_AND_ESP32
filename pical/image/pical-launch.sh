#!/bin/sh
# Starts the calibration app on the console, and leaves evidence behind.
#
# Two things make this less trivial than "the boot text already shows up".
# The boot text is the kernel's framebuffer console, which is not something an
# application can draw into. To put graphics on a console-only system an app
# has to drive DRM/KMS itself -- pick the card, pick the connector, set a
# mode, allocate buffers through GBM and render through EGL. A desktop
# normally does all of that on the app's behalf, so the easy path is to run a
# minimal X server and let it do that job. kmsdrm is tried after it.
#
# Every line is written to the FAT boot partition AND synced, because a stick
# pulled out of a running Pi otherwise keeps an empty file: the writes are
# still sitting in the page cache.
APP=/boot/firmware/pical
LOG="$APP/last-run.log"
export SDL_AUDIODRIVER=dummy
export PICAL_OUT="$APP/calib_out"
export PYTHONUNBUFFERED=1

log() {
    echo "$*" | tee -a "$LOG"
    sync
}

{
    echo "=== pical $(date -u '+%Y-%m-%d %H:%M:%S UTC') ==="
    echo "model:   $(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"
    echo "tty:     $(tty 2>/dev/null)"
    echo "user:    $(id -un) ($(id -u))"
    echo "python:  $(python3 -V 2>&1)"
    echo "drm:     $(ls /dev/dri 2>/dev/null | tr '\n' ' ')"
    echo "egl:     $(ls /usr/lib/*/libEGL.so.1 2>/dev/null | tr '\n' ' ')"
    echo "xorg:    $(command -v Xorg xinit 2>/dev/null | tr '\n' ' ')"
    echo "connectors:"
    for st in /sys/class/drm/card*-*/status; do
        [ -e "$st" ] || continue
        echo "         $(basename "$(dirname "$st")") = $(cat "$st" 2>/dev/null)"
    done
    echo "serial:  $(ls /dev/ttyACM* /dev/ttyUSB* /dev/lightgun 2>/dev/null | tr '\n' ' ')"
} > "$LOG" 2>&1
sync

if [ ! -f "$APP/pical.py" ]; then
    log "!! $APP/pical.py is missing -- the stick was not built correctly"
    exit 1
fi

python3 -c "import pygame, numpy, serial" >> "$LOG" 2>&1 || {
    log "!! python3 is missing pygame, numpy or pyserial (see $LOG)"
    exit 1
}

run_py() {
    python3 "$APP/pical.py" "$@" >> "$LOG" 2>&1
    RC=$?
    sync
    return $RC
}

# ---- 1: a minimal X server, the most reliable path on a Pi ---------------
if command -v xinit >/dev/null 2>&1 && [ -z "${PICAL_NO_X:-}" ]; then
    log "-- starting the X server"
    SDL_VIDEODRIVER=x11 xinit /usr/bin/python3 "$APP/pical.py" "$@" \
        -- :0 vt1 -keeptty -nocursor >> "$LOG" 2>&1
    RC=$?
    sync
    if [ "$RC" = "0" ]; then
        log "-- app exited normally (X11)"
        exit 0
    fi
    log "-- X path failed (exit $RC); last lines:"
    tail -n 8 "$LOG" | sed 's/^/     /'
    sync
fi

# ---- 2: straight to the hardware through DRM/KMS -------------------------
# One of a Pi's DRM cards is the v3d render node with no screen attached;
# the app picks the connected one by itself, and these are the fallbacks.
for IDX in auto 0 1 2; do
    if [ "$IDX" = "auto" ]; then
        unset SDL_KMSDRM_DEVICE_INDEX
        log "-- kmsdrm, letting the app choose the card"
    else
        [ -e "/dev/dri/card$IDX" ] || continue
        export SDL_KMSDRM_DEVICE_INDEX="$IDX"
        log "-- kmsdrm, forcing device index $IDX"
    fi
    SDL_VIDEODRIVER=kmsdrm run_py "$@" && { log "-- app exited normally"; exit 0; }
    log "   failed (exit $RC)"
done
unset SDL_KMSDRM_DEVICE_INDEX

# ---- 3: whatever SDL can find on its own ---------------------------------
for DRV in wayland ""; do
    if [ -n "$DRV" ]; then
        export SDL_VIDEODRIVER="$DRV"
        log "-- trying SDL video driver: $DRV"
    else
        unset SDL_VIDEODRIVER
        log "-- letting SDL choose"
    fi
    run_py "$@" && { log "-- app exited normally"; exit 0; }
    log "   failed (exit $RC)"
done

log "!! nothing could open a display. The full log is on this stick at"
log "!! pical/last-run.log -- read it on any PC."
if ! ls /usr/lib/*/libEGL.so.1 >/dev/null 2>&1; then
    log "!! libEGL is missing: the image was built without the Mesa stack."
fi
sync
exit 1
