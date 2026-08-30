#!/usr/bin/env python3
"""USB doctor logic against injected snapshots: every wire signature the
verdict table promises must actually come out of the code."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "tools"))
import usb_doctor as U

fails = []


def ck(ok, msg):
    print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        fails.append(msg)


K = ("COM13", 0x2E8A, 0x000A, "SN1")          # a gun board
P = {K: "FIRECon P1"}

# ---- wiggle watch: drops are timestamped ----------------------------------
w = U.Watcher(now=100.0)
w.ports = dict(P)
ev = w.poll(now=103.0, ports={}, problems={})
ck(len(ev) == 1 and ev[0][1] == "drop" and "gun board" in ev[0][2],
   "a vanished gun port is a timestamped drop event")
ev = w.poll(now=104.0, ports=dict(P), problems={})
ck(len(ev) == 1 and ev[0][1] == "add", "and its return is logged too")
s = w.summary()
ck(any("dropped 1 time" in l and "3s" in l for l in s),
   "the summary names the second the link died: %s" % s[0][:60])

# ---- descriptor failure: the not-recognized state --------------------------
w = U.Watcher(now=0.0)
w.ports = {}
w.problems = {}
ev = w.poll(now=2.0, ports={},
            problems={"USB\\VID_0000&PID_0000\\X": (43, "Unknown USB Device")})
ck(any("descriptor request FAILED" in e[2] for e in ev),
   "code 43 is translated to plain words")
ck(any("DATA" in l and "Reflow" in l for l in w.summary()),
   "and the verdict points at the data pair")

# ---- a clean watch says so, with the next step ------------------------------
w = U.Watcher(now=0.0)
w.ports = dict(P)
w.poll(now=5.0, ports=dict(P), problems={})
ck(any("no drops" in l for l in w.summary()), "a quiet watch is honest about it")

# ---- soak judgement ---------------------------------------------------------
healthy = [(i * 0.5, i * 50) for i in range(30)]
r = U.soak_report(healthy)
ck(any("steady" in l for l in r), "a steady stream is called healthy")

stall = [(0.0, 0), (0.5, 50), (1.0, 50), (1.6, 50), (2.1, 120), (2.6, 170)]
r = U.soak_report(stall)
ck(any("stall" in l and "marginal data joint" in l for l in r),
   "stalls without drops are named as the marginal-joint signature")

dead = [(i * 0.5, 0) for i in range(10)]
r = U.soak_report(dead)
ck(any("check the camera first" in l for l in r),
   "zero frames points at the camera before blaming USB")

# ---- problem snapshot parser (format, not the live system) ------------------
ck(U.problem_verdict(28).startswith("driver-side"),
   "driver problem codes do not blame a wire")
ck(len(U.WIRE_TABLE) >= 5, "the which-wire table ships with the module")

print("\nusb doctor: %s (%d failures)" % ("ALL PASS" if not fails else "FAILED",
                                          len(fails)))
sys.exit(1 if fails else 0)
