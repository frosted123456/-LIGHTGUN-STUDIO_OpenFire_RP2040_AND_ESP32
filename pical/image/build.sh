#!/usr/bin/env bash
# Builds the pical USB/SD image from Raspberry Pi OS Lite arm64.
#
# One image boots Pi 3 / 4 / 5 / Zero 2 W. The app and the desktop tools it
# imports are placed on the FAT boot partition, so they can be edited from any
# PC without touching the Linux filesystem. Runs on any Ubuntu host with root;
# CI runs it in .github/workflows/pical-image.yml.
#
# Partition edits go through sfdisk, not parted: "parted -s resizepart" answers
# NO to its own shrink prompt and exits 1, which silently leaves the table
# unchanged while the image file is truncated under it.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORK="${WORK:-$REPO/.pical-build}"
OUT="${OUT:-$REPO/pical.img}"
BASE_URL="${BASE_URL:-https://downloads.raspberrypi.com/raspios_lite_arm64_latest}"
GROW_MB="${GROW_MB:-900}"        # headroom for pygame, numpy and their deps
SEC=512                          # bytes per sector
SPB=8                            # sectors per 4 KiB filesystem block
MARGIN_SEC=32768                 # slack left after the shrink, 16 MiB

say() { echo "== $*"; }
die() { echo "!! $*" >&2; exit 1; }

need() { command -v "$1" >/dev/null 2>&1 || die "missing tool: $1"; }
for t in wget xz losetup sfdisk parted resize2fs e2fsck chroot; do need "$t"; done
[ "$(id -u)" = "0" ] || die "run me as root (sudo)"

LOOP=""
cleanup() {
    set +e
    umount -R /mnt/pical-root 2>/dev/null
    [ -n "$LOOP" ] && losetup -d "$LOOP" 2>/dev/null
}
trap cleanup EXIT

# Partition nodes appear asynchronously; touching one before it exists is a
# race that surfaces as "no such file or directory" on a loaded runner.
wait_part() {
    for _ in $(seq 1 40); do
        [ -b "$1" ] && return 0
        partprobe "$LOOP" >/dev/null 2>&1 || true
        partx -u "$LOOP" >/dev/null 2>&1 || true
        command -v udevadm >/dev/null 2>&1 && udevadm settle >/dev/null 2>&1
        sleep 0.25
    done
    # No udev (container-based runners): the kernel knows the partition even
    # when nothing populated /dev, so make the node from what sysfs reports.
    local name dev
    name="$(basename "$1")"
    dev="$(cat "/sys/class/block/$name/dev" 2>/dev/null || true)"
    if [ -n "$dev" ]; then
        mknod "$1" b "${dev%%:*}" "${dev##*:}" 2>/dev/null || true
        [ -b "$1" ] && return 0
    fi
    die "partition node $1 never appeared"
}

attach() {
    LOOP="$(losetup -fP --show "$WORK/work.img")"
    wait_part "${LOOP}p1"
    wait_part "${LOOP}p2"
}

detach() {
    sync
    [ -n "$LOOP" ] && losetup -d "$LOOP"
    LOOP=""
}

part2_start() {
    parted -sm "$LOOP" unit s print | awk -F: '/^2:/{gsub("s","",$2); print $2}'
}

# Sets partition 2's LAST sector, then re-attaches so the kernel sees the new
# geometry (sfdisk cannot re-read the table of a loop device in use).
set_p2_end() {
    local start="$1" last="$2"
    echo ",$(( last - start + 1 ))" | sfdisk --no-reread -N 2 "$LOOP" >/dev/null 2>&1 || true
    detach
    attach
    local got
    got="$(parted -sm "$LOOP" unit s print | awk -F: '/^2:/{gsub("s","",$3); print $3}')"
    [ "$got" = "$last" ] || die "partition 2 end is ${got}s, wanted ${last}s"
}

mkdir -p "$WORK"

# ---------------------------------------------------------------- base image
if [ ! -f "$WORK/base.img" ]; then
    say "fetching Raspberry Pi OS Lite arm64"
    wget -q --show-progress -O "$WORK/base.img.xz" "$BASE_URL"
    say "decompressing"
    xz -T0 -d "$WORK/base.img.xz"
fi
cp --sparse=always "$WORK/base.img" "$WORK/work.img"

# ------------------------------------------------------------ room for pygame
say "growing the root filesystem by ${GROW_MB} MB"
truncate -s "+${GROW_MB}M" "$WORK/work.img"
attach
START="$(part2_start)"
[ -n "$START" ] || die "could not read partition 2 from the base image"
TOTAL=$(( $(stat -c %s "$WORK/work.img") / SEC ))
set_p2_end "$START" $(( TOTAL - 1 ))
e2fsck -pf "${LOOP}p2" || true
resize2fs "${LOOP}p2"

mkdir -p /mnt/pical-root
mount "${LOOP}p2" /mnt/pical-root
mkdir -p /mnt/pical-root/boot/firmware
mount "${LOOP}p1" /mnt/pical-root/boot/firmware

# --------------------------------------------------------------- app payload
say "installing the app onto the FAT boot partition"
APP=/mnt/pical-root/boot/firmware/pical
mkdir -p "$APP/tools" "$APP/calib_out"
install -m 0644 "$REPO/pical/pical.py" "$APP/pical.py"
for f in aim_calib.py aim_fit.py; do
    install -m 0644 "$REPO/tools/$f" "$APP/tools/$f"
done
install -m 0644 "$REPO/pical/image/README-STICK.txt" "$APP/README.txt"

# ------------------------------------------------------------- system pieces
install -m 0644 "$REPO/pical/image/pical.service" \
        /mnt/pical-root/etc/systemd/system/pical.service
install -m 0644 "$REPO/pical/image/99-lightgun.rules" \
        /mnt/pical-root/etc/udev/rules.d/99-lightgun.rules
install -m 0755 "$REPO/pical/image/pical-launch.sh" \
        /mnt/pical-root/usr/local/bin/pical-launch

# -------------------------------------------------------- packages via chroot
if [ "${PICAL_SKIP_CHROOT:-0}" = "1" ]; then
say "SKIPPING the chroot package install (PICAL_SKIP_CHROOT=1)"
else
say "installing python3-pygame, numpy and pyserial into the image"
cp /usr/bin/qemu-aarch64-static /mnt/pical-root/usr/bin/ 2>/dev/null || true
mount -t proc /proc /mnt/pical-root/proc
mount --rbind /sys  /mnt/pical-root/sys
mount --rbind /dev  /mnt/pical-root/dev
cp /etc/resolv.conf /mnt/pical-root/etc/resolv.conf

chroot /mnt/pical-root /bin/bash -eux <<'CHROOT'
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
    python3-pygame python3-numpy python3-serial
apt-get clean
rm -rf /var/lib/apt/lists/*
systemctl enable pical.service
# Nothing else should own the console or the framebuffer.
systemctl disable getty@tty1.service || true
CHROOT

rm -f /mnt/pical-root/usr/bin/qemu-aarch64-static
fi

# ------------------------------------------------------------------ shrink
sync
umount -R /mnt/pical-root
e2fsck -pf "${LOOP}p2" || true
say "shrinking the filesystem back to its contents"
MINB="$(resize2fs -P "${LOOP}p2" | awk '{print $NF}')"
[ -n "$MINB" ] || die "could not read the minimum filesystem size"
resize2fs "${LOOP}p2" "$MINB"
e2fsck -pf "${LOOP}p2" || true
END=$(( START + MINB * SPB + MARGIN_SEC ))
set_p2_end "$START" "$END"
detach
truncate -s $(( (END + 1) * SEC )) "$WORK/work.img"

# ------------------------------------------------------- verify before ship
say "verifying the finished image"
attach
parted -sm "$LOOP" unit s print >/dev/null || die "partition table is unreadable"
e2fsck -pf "${LOOP}p2" || die "root filesystem is not clean"
mount "${LOOP}p2" /mnt/pical-root
[ -x /mnt/pical-root/usr/local/bin/pical-launch ] || die "launcher missing"
[ -f /mnt/pical-root/etc/systemd/system/pical.service ] || die "service missing"
if [ "${PICAL_SKIP_CHROOT:-0}" != "1" ]; then
    [ -L /mnt/pical-root/etc/systemd/system/multi-user.target.wants/pical.service ] \
        || die "pical.service is not enabled"
fi
if [ "${PICAL_SKIP_CHROOT:-0}" != "1" ]; then
    [ -d /mnt/pical-root/usr/lib/python3/dist-packages/pygame ] \
        || die "pygame did not install into the image"
fi
mount "${LOOP}p1" /mnt/pical-root/boot/firmware
[ -f /mnt/pical-root/boot/firmware/pical/pical.py ] || die "app missing from FAT"
[ -f /mnt/pical-root/boot/firmware/pical/tools/aim_calib.py ] || die "tools missing"
umount -R /mnt/pical-root
detach
say "image verified"

mv "$WORK/work.img" "$OUT"
if [ "${PICAL_NO_XZ:-0}" = "1" ]; then
    say "done: $OUT  ($(du -h "$OUT" | cut -f1), left uncompressed)"
    exit 0
fi
say "compressing"
rm -f "$OUT.xz"
xz -T0 -9 "$OUT"
say "done: $OUT.xz  ($(du -h "$OUT.xz" | cut -f1))"
