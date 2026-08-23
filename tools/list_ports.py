#!/usr/bin/env python3
"""List serial ports with enough detail to tell the gun's two ports apart.
Run this when aim_probe.py finds nothing."""
import sys
try:
    from serial.tools import list_ports
except ImportError:
    sys.exit("pip install pyserial")
ps = list(list_ports.comports())
if not ps:
    print("NO SERIAL PORTS AT ALL.")
    print("The gun's HID may still work (mouse moves) while no COM port exists --")
    print("that means the USB CDC is not enumerating, which is a firmware/driver")
    print("matter, not a wiring one.")
    sys.exit(1)
print("%-8s %-34s %-10s %s" % ("port", "description", "VID:PID", "guess"))
print("-"*88)
for p in ps:
    vp = ("%04X:%04X" % (p.vid, p.pid)) if p.vid else "-"
    d = (p.description or "").lower()
    if p.vid == 0x1A86 or "ch340" in d or "ch9102" in d:
        g = "CH340 -- the UART port"
    elif p.vid == 0x303A or "usb serial device" in d or "cdc" in d:
        g = "ESP32 native USB -- try this one"
    else:
        g = ""
    print("%-8s %-34s %-10s %s" % (p.device, (p.description or "")[:34], vp, g))
print("\nESP32-S3 native USB is VID 303A. CH340 is VID 1A86. An OpenFIRE gun")
print("(either board) enumerates as VID F143.")
print("If only the CH340 appears, the native USB CDC is not enumerating -- check")
print("that the build has ARDUINO_USB_CDC_ON_BOOT=1 and that you are using the")
print("USB socket wired to the chip's native pins, not the UART bridge.")
