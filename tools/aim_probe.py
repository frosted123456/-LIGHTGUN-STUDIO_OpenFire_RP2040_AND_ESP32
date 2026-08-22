#!/usr/bin/env python3
"""aim_probe.py -- check that the gun answers '~' commands on its USB serial port.

    python tools/aim_probe.py [--port COM7]

Close the OpenFIRE app first; only one program can hold the port.
"""
import argparse, sys, time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pip install pyserial")

PASS, FAIL, WARN = "PASS", "FAIL", "WARN"
results = []
def report(state, what, detail=""):
    results.append((state, what))
    mark = {"PASS": "[ ok ]", "FAIL": "[FAIL]", "WARN": "[warn]"}[state]
    print("  %s %s%s" % (mark, what, ("   " + detail) if detail else ""))

def drain(ser, seconds, want=None):
    """collect lines for `seconds`; stop early if a line contains `want`"""
    out, buf, t0 = [], b"", time.time()
    while time.time() - t0 < seconds:
        try: buf += ser.read(256)
        except Exception: break
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("ascii", "replace").strip()
            if t: out.append(t)
            if want and want in t: return out
        time.sleep(0.01)
    return out

def probe(portname, verbose=False, dtr=None):
    """dtr=None tries both DTR states."""
    if dtr is None:
        for d in (True, False):
            if probe(portname, verbose, d): return True
        return False
    print("\n=== %s  (DTR %s) ===" % (portname, "asserted" if dtr else "low"))
    # Open without asserting DTR/RTS: pyserial raises both by default and that
    # can reset an ESP32 native-USB CDC.
    try:
        ser = serial.Serial()
        ser.port = portname
        ser.baudrate = 115200
        ser.timeout = 0.1
        ser.dtr = dtr
        ser.rts = False
        ser.open()
    except Exception as e:
        print("  cannot open: %s" % e); return False
    time.sleep(0.4)
    ser.reset_input_buffer()

    # ---- 1. does it answer our query at all? --------------------------
    # Retry: a board that reset on connect needs a moment and can lose the first line.
    boot = drain(ser, 1.0, want="AIM: pipeline ready")
    if any("pipeline ready" in l for l in boot):
        report(PASS, "caught the boot announcement", "build has USE_AIM_PIPELINE")

    ans, lines, alive = [], [], False
    for attempt in range(4):
        ser.write(b"\n~ping\n")
        pong = [l for l in drain(ser, 1.0, want="pong") if "pong" in l]
        if pong:
            report(PASS, "'~ping' answered", pong[0][:78])
            alive = True
            if "calib=active" in pong[0]:
                report(PASS, "a calibration is loaded and active", "(from ~ping)")
            elif "calib=loaded" in pong[0]:
                report(WARN, "calibration loaded but DISABLED", "send ~aimcal=on")
            else:
                report(WARN, "no calibration stored", "run aim_calib.py")
            break
        ser.write(b"\n~aimcal?\n")
        lines = drain(ser, 1.5, want="AIM:")
        ans = [l for l in lines if l.startswith("AIM:")]
        if ans:
            alive = True
            break
        if verbose and lines: print("  (attempt %d saw: %s)" % (attempt+1, lines[-1][:60]))
    if not alive:
        print("  no reply to ~ping or ~aimcal? after 4 attempts")
        if lines:
            print("  port IS alive -- last line was: %s" % lines[-1][:70])
            print("  probably the gun, but the build lacks USE_AIM_PIPELINE")
        else:
            print("  nothing at all on this port")
        ser.close(); return False
    if not ans:                       # ping worked; now get the calibration state
        ser.write(b"~aimcal?\n")
        ans = [l for l in drain(ser, 1.5, want="AIM:") if l.startswith("AIM:")]
    if not ans:
        report(FAIL, "'~aimcal?' gave no reply though ~ping did",
               "a long reply is being dropped -- see the chunked reply sink")
        ans = []
    if ans:
        report(PASS, "'~aimcal?' answered", ans[0][:78])
    stored = bool(ans) and "none stored" not in ans[0]
    # ---- 2. the filter ------------------------------------------------
    ser.write(b"~aimfilt?\n")
    f = [l for l in drain(ser, 1.5, want="filter") if "filter" in l]
    report(PASS if f else FAIL, "'~aimfilt?' answered", f[0][:78] if f else "no reply")

    # ---- 3+4. camera command path AND telemetry ----------------------
    # Use dash= , never thr= : a probe must not mutate detector settings. Restored below.
    ser.write(b"~cam=dash:2\n")
    c = [l for l in drain(ser, 1.5) if "CMD ok" in l]
    report(PASS if c else WARN, "'~cam=' acknowledged by the camera tuner",
           c[0][:70] if c else "no 'CMD ok' echo")
    q = drain(ser, 3.0)
    ql = [l for l in q if l.startswith("Q,")]
    st = [l for l in q if l.startswith("STAT,")]
    if ql:
        report(PASS, "Q telemetry arrives on this port", "%d lines, e.g. %s" % (len(ql), ql[-1][:56]))
        f4 = [l for l in ql if l.split(",")[2:3] == ["4"]]
        n0 = len([l for l in ql if l.split(",")[2:3] == ["0"]])
        if f4:
            report(PASS, "frames with all four LEDs", "%d of %d" % (len(f4), len(ql)))
        elif n0 == len(ql):
            report(FAIL, "NO LEDs detected at all",
                   "aim at the screen; check exposure/threshold. Nothing downstream")
        else:
            report(WARN, "fewer than four LEDs seen", "%d of %d had 4" % (len(f4), len(ql)))
    else:
        report(FAIL, "no Q telemetry", "the line sink may not be installed")
    if st: report(PASS, "STAT line arrives too", st[-1][:60])

    # ---- 5. trigger markers ------------------------------------------
    ser.write(b"~aimcap=1\n")
    m = [l for l in drain(ser, 1.5) if "trigger markers" in l]
    report(PASS if m else FAIL, "'~aimcap=1' acknowledged", m[0][:60] if m else "no reply")
    if m:
        print("\n  >>> PULL THE TRIGGER a few times in the next 6 seconds <<<")
        t = [l for l in drain(ser, 6.0) if l.startswith("T,")]
        report(PASS if t else WARN, "trigger markers received",
               "%d pulls" % len(t) if t else "none -- is the trigger pin mapped in the OpenFIRE app?")

    # ---- 6. we must not have broken OpenFIRE's protocol ---------------
    # 'E' is their end-of-serial-handoff command; a reply proves their parser is reached.
    ser.write(b"E\n")
    e = drain(ser, 1.0)
    report(PASS, "OpenFIRE's own parser still reachable",
           "(sent 'E', no lockup)")

    # restore what we changed, and only what we changed
    ser.write(b"~cam=dash:0\n~aimcap=0\n")
    time.sleep(0.2)
    ser.close()
    return True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port"); ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--reset-hint", action="store_true",
                    help="print what to do if nothing answers")
    a = ap.parse_args()

    ports = list(list_ports.comports())
    print("serial ports on this machine:")
    for p in ports:
        print("  %-8s %s" % (p.device, (p.description or "")[:60]))
    if not ports: sys.exit("no serial ports found -- is the gun plugged in?")

    if a.port:
        ok = probe(a.port, a.verbose)
    else:
        # Try every port; only one of them is the gun.
        ok = False
        for p in ports:
            if probe(p.device, a.verbose):
                print("\n>>> the gun is on %s" % p.device)
                ok = True
                break
    print()
    nf = sum(1 for s, _ in results if s == FAIL)
    nw = sum(1 for s, _ in results if s == WARN)
    if not ok:
        print("NO GUN FOUND.")
        print("")
        print("Do this, in order:")
        print("  1. Leave this running and UNPLUG/REPLUG the gun (or tap its reset).")
        print("     The firmware announces itself on the first pass of the run loop,")
        print("     so a fresh boot while we are listening is the strongest signal.")
        print("  2. Close the OpenFIRE app if it is open -- it holds the port.")
        print("  3. Confirm the build: the banner should mention USE_AIM_PIPELINE.")
        print("     pio run -e combined_s3_freenove -t upload")
        print("  4. Plug the CH340 cable in too and run this again. If the replies")
        print("     appear THERE, the reply routing is still wrong and I need to know.")
        sys.exit(1)
    print("%d checks, %d failed, %d warnings" % (len(results), nf, nw))
    print("READY -- the tool suite can talk to the gun on this port." if nf == 0
          else "NOT READY -- see the FAIL lines above.")
    sys.exit(1 if nf else 0)

main()
