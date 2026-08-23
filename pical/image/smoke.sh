#!/usr/bin/env bash
# Runs build.sh against a synthetic base image, with no network and no ARM
# chroot. It proves the packaging path itself: grow, payload install, shrink,
# partition-table rewrite and the final verify.
#
# The bug this exists to catch: "parted -s resizepart" answers NO to its own
# shrink prompt, which left the table unchanged while the file was truncated
# under it -- a corrupt, unbootable image that still built "successfully".
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
W="${W:-/tmp/pical-smoke}"
[ "$(id -u)" = "0" ] || { echo "run me as root (sudo)" >&2; exit 1; }

rm -rf "$W"; mkdir -p "$W"
truncate -s 420M "$W/base.img"
parted -s "$W/base.img" mklabel msdos
parted -s "$W/base.img" mkpart primary fat32 8192s 139263s
parted -s "$W/base.img" mkpart primary ext4 139264s 100%
L1="$(losetup -o $((8192*512)) --sizelimit $((131072*512)) -f --show "$W/base.img")"
L2="$(losetup -o $((139264*512)) -f --show "$W/base.img")"
# vfat where the kernel can MOUNT it; ext2 stands in where the module is
# absent, since the build only cares that partition 1 mounts at all.
if grep -qw vfat /proc/filesystems 2>/dev/null && command -v mkfs.vfat >/dev/null; then
    mkfs.vfat -F32 "$L1" >/dev/null 2>&1
else
    mkfs.ext2 -q -F "$L1"
fi
mkfs.ext4 -q -F "$L2"
mkdir -p /mnt/pical-smoke
mount "$L2" /mnt/pical-smoke
mkdir -p /mnt/pical-smoke/etc/systemd/system/multi-user.target.wants \
         /mnt/pical-smoke/etc/udev/rules.d /mnt/pical-smoke/usr/local/bin \
         /mnt/pical-smoke/boot/firmware
dd if=/dev/zero of=/mnt/pical-smoke/ballast bs=1M count=80 status=none
umount /mnt/pical-smoke
losetup -d "$L1"; losetup -d "$L2"

PICAL_SKIP_CHROOT=1 PICAL_NO_XZ=1 WORK="$W" OUT="$W/out.img" GROW_MB=200 \
    bash "$REPO/pical/image/build.sh"

# The image must still be readable from the outside, with the payload in place.
L1="$(losetup -o $((8192*512)) --sizelimit $((131072*512)) -f --show "$W/out.img")"
mount "$L1" /mnt/pical-smoke
test -f /mnt/pical-smoke/pical/pical.py
test -f /mnt/pical-smoke/pical/tools/aim_calib.py
test -f /mnt/pical-smoke/pical/tools/aim_fit.py
python3 -c "import ast,sys; ast.parse(open('/mnt/pical-smoke/pical/pical.py').read())"
umount /mnt/pical-smoke; losetup -d "$L1"
rm -rf "$W"
echo "pical image smoke: OK"
