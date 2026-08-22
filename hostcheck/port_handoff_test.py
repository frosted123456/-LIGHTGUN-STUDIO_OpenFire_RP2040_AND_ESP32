# Step 1 hands the serial port to the OpenFIRE app. Coming back from that is
# ours to do, and it must ALWAYS finish so the status can never stay parked on
# "handed off" until Studio is restarted.
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "tools"))
os.environ.setdefault("MPLBACKEND", "Agg")

import gun_studio as gs                                    # noqa: E402

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  ok   %s" % name)
    else:
        print("  FAIL %s %s" % (name, detail))
        fails.append(name)


class FakeProc(object):
    """Records that wait() happened, and when."""
    def __init__(self, order):
        self.order = order
        self.waited = False

    def wait(self):
        self.waited = True
        self.order.append("wait")


class Clock(object):
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


print("port handoff")

# The port must not be probed before their app has exited: probing early can
# reopen the port under a still-running app.
order = []
clk = Clock()
proc = FakeProc(order)


def probe_ok(port):
    order.append("probe")
    return True


why = gs.take_port_back(proc, "COM8", timeout_s=60, probe=probe_ok,
                        sleep=clk.sleep, clock=clk)
check("waits for their app first", order[0] == "wait", order)
check("free port returns immediately", why == "free", why)

# The launcher case: proc exits at once but the port is still held. It has to
# keep trying, not give up on the first probe.
clk = Clock()
held = {"n": 0}


def probe_late(port):
    held["n"] += 1
    return held["n"] >= 5


why = gs.take_port_back(FakeProc([]), "COM8", timeout_s=60, probe=probe_late,
                        sleep=clk.sleep, clock=clk)
check("keeps probing while the port is held", why == "free" and held["n"] == 5,
      "why=%s n=%d" % (why, held["n"]))
check("does not spin", clk.t >= 4.0, "elapsed=%s" % clk.t)

# Never-released port: it must still return, so the caller reconnects and the
# status stops lying. Hanging here is the original bug in a new costume.
clk = Clock()
why = gs.take_port_back(FakeProc([]), "COM8", timeout_s=10,
                        probe=lambda p: False, sleep=clk.sleep, clock=clk)
check("gives up rather than hanging", why == "timeout", why)
check("respects the timeout", 10.0 <= clk.t <= 11.0, "elapsed=%s" % clk.t)

# No port known: nothing to wait for.
why = gs.take_port_back(FakeProc([]), "", probe=lambda p: False)
check("no port returns at once", why == "no port", why)

# A dead Popen must not take the caller down with it.
class Boom(object):
    def wait(self):
        raise OSError("process already reaped")


why = gs.take_port_back(Boom(), "COM8", timeout_s=5, probe=lambda p: True,
                        sleep=lambda d: None, clock=Clock())
check("survives a broken proc handle", why == "free", why)

# The probe itself must never raise on a port that is not there.
check("port_is_free is false for a missing port",
      gs.port_is_free("/dev/definitely-not-a-port") is False)

print("FAILURES: %d" % len(fails))
print("port handoff: ALL PASS" if not fails else "port handoff: FAILED")
sys.exit(1 if fails else 0)
