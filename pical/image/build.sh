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
CHROOT_MOUNTED=0
cleanup() {
    set +e
    chroot_mounts_down /mnt/pical-root
    umount -R /mnt/pical-root 2>/dev/null || umount -R -l /mnt/pical-root 2>/dev/null
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

# Chroot mounts, made explicitly rather than with --rbind. A recursive bind of
# /sys drags the host's cgroup2 hierarchy in, which then refuses to unmount
# ("target is busy"); detaching it lazily instead leaves the root filesystem
# pinned, so the shrink that follows finds the device still in use. A fresh
# sysfs and a NON-recursive /dev bind avoid the whole problem.
chroot_mounts_up() {
    mount -t proc  proc   "$1/proc"
    mount -t sysfs sysfs  "$1/sys"
    mount --bind   /dev   "$1/dev"
    mkdir -p "$1/dev/pts"
    mount -t devpts devpts "$1/dev/pts" 2>/dev/null || true
    CHROOT_MOUNTED=1
}
chroot_mounts_down() {
    [ "${CHROOT_MOUNTED:-0}" = "1" ] || return 0
    local m
    for m in dev/pts dev sys proc; do
        umount "$1/$m" 2>/dev/null || umount -l "$1/$m" 2>/dev/null || true
    done
    CHROOT_MOUNTED=0
}

# Unmounts the image root and PROVES it: e2fsck on a still-mounted filesystem
# refuses, and the shrink after it would then run on stale geometry.
umount_root() {
    local i
    for i in $(seq 1 20); do
        mountpoint -q "$1" || return 0
        sync
        umount -R "$1" 2>/dev/null && continue
        command -v fuser >/dev/null 2>&1 && fuser -k -M -m "$1" 2>/dev/null || true
        sleep 0.5
    done
    if mountpoint -q "$1"; then
        die "$1 is still mounted; something holds it open"
    fi
    return 0
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
# RaspiOS enables the ext4 "orphan_file" feature. Resizing such a filesystem
# with a different e2fsprogs -- here, and again when the Pi expands it on
# first boot -- leaves the orphan inode list inconsistent, and the SECOND
# boot then fails with "ext4_init_orphan_info" and drops to initramfs. The
# feature is a performance optimisation; the image does not need it.
tune2fs -O ^orphan_file "${LOOP}p2" >/dev/null 2>&1 || true
e2fsck -pf "${LOOP}p2" >/dev/null 2>&1 || true
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
# Everything pical imports, and everything those import in turn. A missing
# module here is an app that crashes on the TV with no way to fix it.
for f in aim_calib.py aim_fit.py calib_lens.py aim_finetune.py \
         aim_verify.py gun_studio.py; do
    install -m 0644 "$REPO/tools/$f" "$APP/tools/$f"
done
install -m 0644 "$REPO/pical/image/README-STICK.txt" "$APP/README.txt"

# ------------------------------------------------------------- system pieces
install -m 0644 "$REPO/pical/image/99-lightgun.rules" \
        /mnt/pical-root/etc/udev/rules.d/99-lightgun.rules
install -m 0755 "$REPO/pical/image/pical-launch.sh" \
        /mnt/pical-root/usr/local/bin/pical-launch

# RaspiOS runs a first-boot init that asks for a username and password and
# resizes the root filesystem. This is an appliance: it must come up on the
# calibration screen with no questions, and the resize is what corrupted the
# filesystem on the second boot. Remove it from the kernel command line.
CMDLINE=/mnt/pical-root/boot/firmware/cmdline.txt
if [ -f "$CMDLINE" ]; then
    sed -i -e 's| init=/usr/lib/raspberrypi-sys-mods/firstboot||g' \
           -e 's| systemd.run[^ ]*||g' -e 's| systemd.run_success_action=[^ ]*||g' \
           -e 's| systemd.unit=kernel-command-line[^ ]*||g' "$CMDLINE"
    say "cmdline: $(cat "$CMDLINE")"
fi

# -------------------------------------------------------- packages via chroot
if [ "${PICAL_SKIP_CHROOT:-0}" = "1" ]; then
say "SKIPPING the chroot package install (PICAL_SKIP_CHROOT=1)"
else
say "installing python3-pygame, numpy and pyserial into the image"
cp /usr/bin/qemu-aarch64-static /mnt/pical-root/usr/bin/ 2>/dev/null || true
chroot_mounts_up /mnt/pical-root
cp /etc/resolv.conf /mnt/pical-root/etc/resolv.conf

chroot /mnt/pical-root /bin/bash -eux <<'CHROOT'
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
    python3-pygame python3-numpy python3-serial
apt-get clean
rm -rf /var/lib/apt/lists/*
# A named account exists for diagnostics over SSH or a second console; the
# app itself runs as root from tty1.
if ! id pical >/dev/null 2>&1; then
    useradd -m -s /bin/bash pical
fi
echo 'pical:pical123' | chpasswd
usermod -aG sudo,dialout,video,input,render pical 2>/dev/null ||     usermod -aG sudo,dialout,video,input pical

# tty1 logs straight in as root and runs the app. "systemctl disable" does
# not stop a console getty -- getty.target pulls the instance in on its own --
# so the instance is overridden rather than fought with.
mkdir -p /etc/systemd/system/getty@tty1.service.d
cat > /etc/systemd/system/getty@tty1.service.d/autologin.conf <<'UNIT'
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin root --noclear %I $TERM
UNIT
# Do not rely on the generator having pulled the instance in.
systemctl enable getty@tty1.service 2>/dev/null || true

# The first-boot user wizard must never appear.
systemctl disable userconfig.service 2>/dev/null || true
systemctl mask    userconfig.service 2>/dev/null || true
systemctl disable pical.service 2>/dev/null || true
rm -f /etc/systemd/system/pical.service
systemctl set-default multi-user.target

# tty1 is the appliance console. Any other console -- SSH, tty2 via
# Ctrl+Alt+F2 -- gets an ordinary shell, and dropping a file called
# NOAUTOSTART next to the app on the FAT partition disables the autostart
# from any PC, without reflashing.
cat > /root/.bash_profile <<'PROFILE'
if [ "$(tty)" = "/dev/tty1" ] && [ ! -f /boot/firmware/pical/NOAUTOSTART ]; then
    /usr/local/bin/pical-launch
    echo
    echo "The calibration app exited. Type 'pical-launch' to start it again,"
    echo "or 'poweroff' to shut down cleanly before pulling the power."
fi
PROFILE
CHROOT

rm -f /mnt/pical-root/usr/bin/qemu-aarch64-static
chroot_mounts_down /mnt/pical-root
fi

# ------------------------------------------------------------------ shrink
sync
umount_root /mnt/pical-root
# The shrink is the one step that silently corrupts if it runs against a
# filesystem the kernel still has open.
if mountpoint -q /mnt/pical-root; then
    die "root still mounted before the shrink"
fi
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
if [ -f /mnt/pical-root/boot/firmware/cmdline.txt ]; then
    grep -q "firstboot" /mnt/pical-root/boot/firmware/cmdline.txt \
        && die "the first-boot wizard is still on the kernel command line"
fi
if [ "${PICAL_SKIP_CHROOT:-0}" != "1" ]; then
    grep -q "autologin root" \
        /mnt/pical-root/etc/systemd/system/getty@tty1.service.d/autologin.conf \
        2>/dev/null || die "tty1 autologin was not installed"
    grep -q pical-launch /mnt/pical-root/root/.bash_profile \
        || die "the console does not launch the app"
fi
if [ "${PICAL_SKIP_CHROOT:-0}" != "1" ]; then
    [ -d /mnt/pical-root/usr/lib/python3/dist-packages/pygame ] \
        || die "pygame did not install into the image"
fi
mount "${LOOP}p1" /mnt/pical-root/boot/firmware
[ -f /mnt/pical-root/boot/firmware/pical/pical.py ] || die "app missing from FAT"
for f in aim_calib.py aim_fit.py calib_lens.py aim_finetune.py \
         aim_verify.py gun_studio.py; do
    [ -f "/mnt/pical-root/boot/firmware/pical/tools/$f" ] || die "tools/$f missing"
done
umount_root /mnt/pical-root
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
