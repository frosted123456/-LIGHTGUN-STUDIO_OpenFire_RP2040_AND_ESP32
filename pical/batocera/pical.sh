#!/bin/bash
# Batocera port entry. Copy the whole pical folder into
# /userdata/roms/ports/ and it appears in EmulationStation as a game.
#
# EXPERIMENTAL: shipped untested on Batocera. The USB image in pical/image/
# is the supported way to run this on a Pi.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-kmsdrm}"
export PICAL_OUT="$DIR/calib_out"
mkdir -p "$PICAL_OUT"
for PY in python3 /usr/bin/python3; do
    command -v "$PY" >/dev/null 2>&1 || continue
    if "$PY" -c "import pygame, numpy, serial" >/dev/null 2>&1; then
        exec "$PY" "$DIR/pical.py" "$@"
    fi
done
echo "pical needs python3 with pygame, numpy and pyserial." >&2
echo "Batocera does not ship all three on every build; use the USB image" >&2
echo "in pical/image/ instead." >&2
sleep 8
