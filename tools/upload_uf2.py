#!/usr/bin/env python3
"""UF2 uploader for the RP2040 build: reboot the gun to BOOTSEL over the
1200-baud touch, wait for the RPI-RP2 drive to actually mount, copy the UF2.

    python tools/upload_uf2.py <firmware.uf2> [COMxx]

No port given: probes for the gun. Board already in BOOTSEL: just copies.
"""
import os, shutil, string, sys, time

MOUNT_WAIT_S = 60.0     # Windows can take a while to mount the drive
SETTLE_S     = 1.0      # extra delay after the drive appears


def find_uf2_drive():
    """Return the mount point of a BOOTSEL drive, or None."""
    if os.name == "nt":
        for d in string.ascii_uppercase:
            if os.path.exists("%s:/INFO_UF2.TXT" % d):
                return "%s:/" % d
        return None
    for base in ("/media", "/run/media", "/Volumes"):
        if not os.path.isdir(base):
            continue
        for root, dirs, files in os.walk(base):
            if "INFO_UF2.TXT" in files:
                return root
            if root.count(os.sep) - base.count(os.sep) > 2:
                dirs[:] = []
    return None


def touch_1200(port):
    """Open/close the port at 1200 baud; the core reboots to BOOTSEL."""
    import serial
    try:
        s = serial.Serial(port, 1200)
        s.close()
        return True
    except Exception as e:
        print("  could not touch %s (%s)" % (port, e))
        return False


def find_gun_port():
    """First port that answers ~ping, else the first OpenFIRE VID port."""
    import serial
    from serial.tools import list_ports
    candidates = list(list_ports.comports())
    for p in candidates:
        if p.vid == 0xF143:
            return p.device
    for p in candidates:
        try:
            s = serial.Serial(p.device, 115200, timeout=0.15)
            s.write(b"\n~ping\n")
            time.sleep(0.4)
            ok = b"pong" in s.read(256)
            s.close()
            if ok:
                return p.device
        except Exception:
            pass
    return None


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    uf2 = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] else None
    if not os.path.exists(uf2):
        print("ERROR: %s does not exist -- build first" % uf2)
        return 1

    drive = find_uf2_drive()
    if drive is None:
        if port is None:
            port = find_gun_port()
        if port:
            print("Rebooting the gun on %s into BOOTSEL..." % port)
            touch_1200(port)
        else:
            print("No gun found on any port. Hold BOOTSEL and plug the board in.")
        print("Waiting for the RPI-RP2 drive (up to %.0f s)..." % MOUNT_WAIT_S)
        t0 = time.time()
        while drive is None and time.time() - t0 < MOUNT_WAIT_S:
            time.sleep(0.5)
            drive = find_uf2_drive()
    if drive is None:
        print("ERROR: no BOOTSEL drive appeared. Hold BOOTSEL while plugging the")
        print("board in, then run this again (it will copy straight away).")
        return 1

    print("Copying %s -> %s" % (os.path.basename(uf2), drive))
    time.sleep(SETTLE_S)
    try:
        shutil.copy(uf2, os.path.join(drive, os.path.basename(uf2)))
    except Exception as e:
        # the drive vanishing mid-copy usually means the board already took the
        # file and rebooted -- Windows reports that as an I/O error
        if find_uf2_drive() is None:
            print("Drive vanished during the copy -- the board took the "
                  "firmware and rebooted. Done.")
            return 0
        print("ERROR: copy failed: %s" % e)
        return 1
    print("Done. The board reboots into the new firmware.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
