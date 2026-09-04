#!/usr/bin/env python3
"""The calibration install path, against a fake gun: the line must go out with the
'~' prefix the firmware requires, and the read-back must catch a gun that stored
different values."""
import sys, os, time, types
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import aim_calib as A

class FakeGun:
    """Mimics the firmware's gatekeeper: only '~'-prefixed lines are claimed."""
    def __init__(self, honour_tilde=True):
        self.honour = honour_tilde
        self.installed = None
        self.replies = []
        self.ser = self
        self._buf = ""
    def write(self, b):
        self._buf += b.decode()
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if not line: continue
            if self.honour and not line.startswith("~"):
                continue                        # passed to OpenFIRE, discarded
            line = line.lstrip("~")
            if line.startswith("aimcal="):
                self.installed = [float(x) for x in line.split("=")[1].split(",")]
            elif line.startswith("aimcal?"):
                if self.installed is None:
                    self.replies.append("AIM: none stored")
                else:
                    v = self.installed
                    self.replies.append(
                        "AIM: ACTIVE cx=%.5f cy=%.5f w=%.5f h=%.5f bx=%.3f by=%.3f "
                        "lever=%.5f rx=%.5f ry=%.5f" % (
                            v[0], v[1], v[2], v[3], v[4], v[5],
                            v[6] if len(v) > 6 else 0.0,
                            v[7] if len(v) > 8 else 0.0,
                            v[8] if len(v) > 8 else 0.0))

C = dict(cx=0.508233, cy=0.569531, w=0.406803, h=1.241522,
         bx=8.109, by=6.188, lever=0.0, rx=0.010513, ry=-0.021610)
cmd = A.aimcal_line(C)
fails = []

g = FakeGun(honour_tilde=True)
msg = A.install_over_serial(g, cmd, C, timeout=1.0)
print("strict gun :", msg.splitlines()[0])
if not msg.startswith("INSTALLED and verified"): fails.append("strict gun refused a correct install")
if g.installed is None or abs(g.installed[7] - C['rx']) > 1e-9:
    fails.append("roll coefficients did not reach the gun")

# the regression: a bare line must be REPORTED as not installed, never as sent
g2 = FakeGun(honour_tilde=True)
def bare(src, cmd, c, timeout=1.0):
    src.ser.write(("\n" + cmd.lstrip("~") + "\n").encode())   # the old behaviour
    src.ser.write(b"\n~aimcal?\n")
    time.sleep(0.05)
    return src.replies[-1] if src.replies else "no reply"
r = bare(g2, cmd, C)
print("bare line  :", r, "-> installed:", g2.installed)
if g2.installed is not None: fails.append("a bare line reached the gun; the fake is wrong")
msg2 = A.install_over_serial(g2, cmd, C, timeout=1.0)
if not msg2.startswith("INSTALLED"): fails.append("prefixed retry still failed")

# and a gun that answers with the WRONG values must be caught
g3 = FakeGun(honour_tilde=True)
A.install_over_serial(g3, cmd, C, timeout=1.0)
g3.installed[7] = 0.0                                  # gun silently dropped the roll term
g3.replies.clear()
msg3 = A.install_over_serial(g3, cmd, dict(C, rx=0.5), timeout=1.0)
print("mismatch   :", msg3.splitlines()[0])
if "DISAGREES" not in msg3: fails.append("a mismatched read-back was not detected")

# The reply buffer itself, on a SerialSource with no port: the reader thread
# appends while a waiter reads, so the list is locked and copied out; it holds
# 80 lines (a 13-line camlearn answer, a blob pair, verdicts and margin); and a
# reader that dies says WHY, so a front end can show more than "lost".
import threading
class RaisingSer:
    def __init__(self, lines):
        self.lines = list(lines)
    @property
    def in_waiting(self):
        if not self.lines: raise OSError("device reports readiness to read but returned no data")
        return len(self.lines[0])
    def read(self, n):
        return self.lines.pop(0)
    def write(self, b): pass
src = A.SerialSource.__new__(A.SerialSource)
threading.Thread.__init__(src, daemon=True)
src.ser = RaisingSer([("CAM: n=%d\n" % i).encode() for i in range(100)] + [b"Q,1,4,1,2,3,4,5,6,7,8,c,15,0,0\n"])
src.stop = False; src.replies = []; src.lock = threading.Lock(); src.dead = False; src.dead_reason = ""
src.q = __import__("queue").Queue(maxsize=4000)
src.via_queue = True; src.want_dash = False # owned by a Link: the re-arm is a request, not a write
src.run()                                   # runs to the raise, inline
if not src.want_dash:
    fails.append("a queue-owned reader did not ask the Link to re-arm the stream")
snap = src.snapshot()
if len(snap) != 80 or snap[0] != "CAM: n=20" or snap[-1] != "CAM: n=99":
    fails.append("the reply buffer did not keep the last 80 lines: %d, %r..%r"
                 % (len(snap), snap[:1], snap[-1:]))
if snap is src.replies:
    fails.append("snapshot() handed out the live list instead of a copy")
if not (src.dead and "readiness" in src.dead_reason):
    fails.append("a reader that died did not say why: dead=%r reason=%r"
                 % (src.dead, src.dead_reason))
src.clear_replies()
if src.snapshot():
    fails.append("clear_replies() left lines behind")
print("reply buffer: 80 kept, copied out under the lock, death reason %r" % src.dead_reason[:40])

for f in fails: print("  [FAIL]", f)
print("install verification: %s" % ("ALL PASS" if not fails else "FAILED"))
sys.exit(1 if fails else 0)
