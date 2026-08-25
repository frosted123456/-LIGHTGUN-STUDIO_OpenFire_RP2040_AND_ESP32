#!/usr/bin/env python3
"""The camera-settings save path, against a fake gun.

Two things are asserted, because two things were wrong. First, that a save is
CONFIRMED rather than assumed: a refusal, a mismatch, a silent gun and an older
firmware that does not report a field must each produce a different, honest
answer. Second, that the beta knob steps the way the screens claim it does,
including the -1 "follow the smoothing table" value at the bottom of the range.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "tools"))
import aim_calib as A
import aim_finetune as F

FAILS = []


def ck(ok, msg):
    print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        FAILS.append(msg)


class FakeGun:
    """Applies ~cam= lines and answers ~camsave with what it is holding."""

    def __init__(self, mode="ok", report_beta=True):
        self.mode = mode                  # ok | fail | silent
        self.report_beta = report_beta
        self.state = {"lead": 0, "smooth": 3, "beta": -1, "dead": 0, "lens": 0}
        self.replies = []
        self.written = []
        self.ser = self
        self._buf = ""

    def write(self, b):
        self.written.append(b)
        self._buf += b.decode()
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip().lstrip("~")
            if line.startswith("cam="):
                for tok in line[4:].split(","):
                    k, sep, v = tok.partition(":")
                    if sep and k in self.state:
                        try:
                            self.state[k] = int(v)
                        except ValueError:
                            pass
            elif line == "camsave":
                if self.mode == "silent":
                    continue
                head = ("CAM: saved" if self.mode == "ok"
                        else "CAM: SAVE FAILED (values out of range?)")
                beta = (" beta=%d" % self.state["beta"]) if self.report_beta else ""
                self.replies.append(
                    "%s thr=110 aec=300 agc=4 boost=0 lead=%dms smooth=%d "
                    "dead=%d%s lens=%d tmode=0 firk=7 firpct=100"
                    % (head, self.state["lead"], self.state["smooth"],
                       self.state["dead"], beta, self.state["lens"]))
        return len(b)


print("camsave verification")

# ---- the happy path: the gun holds what the screen asked for --------------
g = FakeGun()
g.write(b"\n~cam=lead:25,smooth:6,beta:24\n")
ok, msg = A.camsave_verified(g, lead=25, smooth=6, beta=24)
ck(ok, "a matching save is confirmed: %s" % msg)
ck(any(b"camsave" in w for w in g.written), "and ~camsave actually went out")
ck("lead=25" in msg and "beta=24" in msg,
   "the message quotes the gun's own numbers, not the tool's")

# ---- a refusal must not read as success -----------------------------------
g = FakeGun(mode="fail")
ok, msg = A.camsave_verified(g, lead=0)
ck(not ok and "REFUSED" in msg, "a SAVE FAILED reply is reported as a refusal")

# ---- a silent gun is not a save -------------------------------------------
g = FakeGun(mode="silent")
ok, msg = A.camsave_verified(g, lead=0, timeout=0.3)
ck(not ok and "NOT" in msg.upper(), "no reply at all is reported as NOT saved")

# ---- a gun that stored something else --------------------------------------
# This is the case an optimistic toast hides completely: the write returned OK
# but the value that landed is not the one on screen.
g = FakeGun()
g.write(b"\n~cam=beta:9\n")
ok, msg = A.camsave_verified(g, beta=30)
ck(not ok and "DIFFERENT" in msg and "9" in msg,
   "a value the gun did not take is caught: %s" % msg)

# ---- older firmware, no beta in the save reply ------------------------------
g = FakeGun(report_beta=False)
g.write(b"\n~cam=lead:10\n")
ok, msg = A.camsave_verified(g, lead=10, beta=30)
ck(ok and "does not report beta" in msg,
   "a field the firmware never reports is named, not failed: %s" % msg)

# ---- a disconnected gun ------------------------------------------------------
ok, msg = A.camsave_verified(None, lead=0)
ck(not ok, "no port is not a save")

# ---- the beta knob ----------------------------------------------------------
print("beta knob")
t = F.Tuner({"cx": 0.5, "cy": 0.5, "w": 0.3, "h": 1.0, "bx": 0.0, "by": 0.0},
            lead=0, smooth=3, beta=-1)
ck(t.beta == -1 and t.beta_label().startswith("auto"),
   "it starts on the table value the gun reported")
t.nudge_beta(+1)
ck(t.beta == F.BETA_AUTO + F.BETA_STEP,
   "a nudge off auto steps from %d, not from 0" % F.BETA_AUTO)
t.nudge_beta(-1)
ck(t.beta == F.BETA_AUTO, "and steps back down by the same amount")
for _ in range(50):
    t.nudge_beta(+1)
ck(t.beta == F.BETA_MAX, "it clamps at the firmware ceiling of %d" % F.BETA_MAX)
for _ in range(50):
    t.nudge_beta(-1)
ck(t.beta == -1, "and returns to auto below zero, so the default is reachable")

# beta is independent of the other two knobs, which is the whole point of it
t = F.Tuner({"cx": 0.5, "cy": 0.5, "w": 0.3, "h": 1.0, "bx": 0.0, "by": 0.0},
            lead=20, smooth=7, beta=-1)
t.nudge_beta(+1)
ck(t.lead == 20 and t.smooth == 7,
   "changing beta leaves lead and rest smoothing alone")

# a session spent only on feel is still something to save
ck(t.solve_direct() is not None,
   "a beta-only change counts as something to save")
t2 = F.Tuner({"cx": 0.5, "cy": 0.5, "w": 0.3, "h": 1.0, "bx": 0.0, "by": 0.0})
ck(t2.solve_direct() is None, "an untouched screen still has nothing to save")

print("\ncamsave verification: %s (%d failures)"
      % ("ALL PASS" if not FAILS else "FAILED", len(FAILS)))
sys.exit(1 if FAILS else 0)
