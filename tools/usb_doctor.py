"""USB link doctor: which of the four USB wires is bad, from the host side.

The gun cannot probe its own USB -- D+/D- run to the chip's USB block, not to
GPIO, and a broken link cannot carry its own diagnosis. But the HOST sees a
distinct signature per wire, and watching arrivals, drops and Windows problem
codes while the user wiggles the harness localises the fault in time.

The wire signatures this is built on:
  no power light, nothing in Device Manager      -> VBUS or GND (power pair)
  board powers up, PC never sees ANY device      -> D+ (attach is a D+ pull-up)
  "device not recognized" / descriptor failed    -> data pair marginal (D- first)
  works until the cable is flexed                -> a joint; the wiggle log
                                                    says which second, you know
                                                    which section you touched
All pure logic lives in testable functions; only snapshots touch the system.
"""
import re
import subprocess
import time

try:
    from serial.tools import list_ports
except Exception:                       # pyserial absent: watcher still loads
    list_ports = None

# USB vendor ids of the boards this project ships on
GUN_VIDS = {0x2E8A, 0x303A}             # Raspberry Pi (RP2040), Espressif

WIRE_TABLE = (
    "WHICH WIRE -- read the symptoms top to bottom:",
    "  gun completely dead (no LED): VBUS or GND, the outer pair.",
    "  gun LED on, PC never chimes: D+ broken -- attach itself is a D+ pull-up.",
    "  'USB device not recognized': data pair marginal -- reflow D- and D+.",
    "  connects, drops when flexed: cold joint -- the wiggle log names the second.",
    "  drops only when the solenoid fires: power sag or vibration at a joint.",
)


def snapshot_ports():
    """Serial ports right now: {key: description}. Key survives re-plugs of
    the same physical device but distinguishes different ones."""
    out = {}
    if list_ports is None:
        return out
    for p in list_ports.comports():
        key = (p.device, p.vid or 0, p.pid or 0, p.serial_number or "")
        out[key] = p.description or p.device
    return out


def diff_ports(prev, cur):
    """[(kind, text)] for every appearance/disappearance between snapshots."""
    ev = []
    for k in cur:
        if k not in prev:
            gun = " (a gun board)" if k[1] in GUN_VIDS else ""
            ev.append(("add", "%s appeared%s" % (k[0], gun)))
    for k in prev:
        if k not in cur:
            gun = " (a gun board)" if k[1] in GUN_VIDS else ""
            ev.append(("drop", "%s DISAPPEARED%s" % (k[0], gun)))
    return ev


def snapshot_problems():
    """Windows device problem states: {instance_id: (code, description)}.

    Catches the state a port listing cannot: a device that ATTACHED but never
    enumerated ("Device Descriptor Request Failed") has no COM port at all.
    Empty on other platforms or when pnputil is unavailable; parsing is
    tolerant of locale because it keys on the numeric code."""
    try:
        r = subprocess.run(["pnputil", "/enum-devices", "/problem"],
                           capture_output=True, text=True, timeout=10)
    except Exception:
        return {}
    out = {}
    inst, desc = None, ""
    for ln in (r.stdout or "").splitlines():
        ln = ln.strip()
        m = re.match(r"Instance ID:\s*(.+)$", ln)
        if m:
            inst, desc = m.group(1), ""
            continue
        m = re.match(r"Device Description:\s*(.+)$", ln)
        if m:
            desc = m.group(1)
            continue
        m = re.search(r"Problem.*?:\s*.*?(\d+)", ln)
        if m and inst and "USB" in inst.upper():
            out[inst] = (int(m.group(1)), desc)
    return out


def problem_verdict(code, desc=""):
    """One plain-language line per Windows problem code that matters here."""
    if code == 43:
        return ("descriptor request FAILED (code 43): the device attached but "
                "could not talk -- a marginal DATA wire; reflow D- and D+ "
                "first. (%s)" % (desc or "unnamed USB device"))
    if code in (28, 31, 37):
        return ("driver-side failure (code %d) on %s -- less likely a wire; "
                "try another PC port first" % (code, desc or "a USB device"))
    if code == 21:
        return "device is being removed (code 21) -- transient, re-check"
    return "problem code %d on %s" % (code, desc or "a USB device")


class Watcher:
    """Timestamps every port event and problem-state change over a window.

    The user wiggles one harness section at a time and reads back WHICH SECOND
    each drop happened -- the fault is wherever their hand was."""

    PROBLEM_EVERY_S = 2.0           # pnputil is a subprocess; not every tick

    def __init__(self, now=None):
        self.t0 = now if now is not None else time.time()
        self.ports = snapshot_ports()
        self.problems = snapshot_problems()
        self._prob_t = self.t0
        self.events = []                # (t_rel, kind, text)
        self.drops = 0
        self.descriptor_fails = 0

    def poll(self, now=None, ports=None, problems=None):
        """One tick; injectable snapshots so the logic is host-testable."""
        now = now if now is not None else time.time()
        cur = ports if ports is not None else snapshot_ports()
        fresh = []
        for kind, text in diff_ports(self.ports, cur):
            if kind == "drop":
                self.drops += 1
            fresh.append((now - self.t0, kind, text))
        self.ports = cur
        # The live fallback used to reuse the baseline itself, which compared
        # the dict against itself -- the "device not recognized" catcher, the
        # tab's headline feature, silently never fired.
        if problems is not None:
            curp = problems
        elif now - self._prob_t >= self.PROBLEM_EVERY_S:
            self._prob_t = now
            curp = snapshot_problems()
        else:
            curp = self.problems
        for inst, (code, desc) in curp.items():
            if inst not in self.problems:
                if code == 43:
                    self.descriptor_fails += 1
                fresh.append((now - self.t0, "problem",
                              problem_verdict(code, desc)))
        self.problems = curp
        self.events.extend(fresh)
        return fresh

    def summary(self):
        """The verdict lines for everything the watch saw."""
        out = []
        if self.descriptor_fails:
            out.append("VERDICT: enumeration failed %d time(s) -- the DATA "
                       "pair is marginal. Reflow D- and D+ at the connector "
                       "you resoldered; a joint that half-works passes power "
                       "and fails data." % self.descriptor_fails)
        if self.drops:
            secs = ", ".join("%.0fs" % t for t, k, _ in self.events
                             if k == "drop")
            out.append("VERDICT: the link dropped %d time(s) (at %s). The "
                       "wire you were flexing at those moments is the fault."
                       % (self.drops, secs))
        if not self.drops and not self.descriptor_fails:
            out.append("no drops and no enumeration failures during the "
                       "watch. If the gun still misbehaves on a fresh plug, "
                       "run the watch again starting UNPLUGGED, then plug in.")
        return out


def soak_report(samples):
    """Judges a stream soak: [(t, frames_total)] -> verdict lines.

    A healthy link gains frames at a steady rate; a marginal one stalls in
    bursts long before it drops entirely."""
    if len(samples) < 3:
        return ["soak too short to judge"]
    gaps = []
    for (t0, f0), (t1, f1) in zip(samples, samples[1:]):
        if f1 == f0 and (t1 - t0) > 0:
            gaps.append((t0, t1 - t0))
    total = samples[-1][1] - samples[0][1]
    dur = samples[-1][0] - samples[0][0]
    rate = total / dur if dur > 0 else 0.0
    out = ["soak: %d frames in %.1f s (%.0f/s)" % (total, dur, rate)]
    long_gaps = [g for g in gaps if g[1] >= 0.4]
    if total == 0:
        out.append("VERDICT: port open but NO data -- if this is the wiicam "
                   "gun, check the camera first (~camdiag); a dead camera "
                   "sends no frames on a perfect USB link.")
    elif long_gaps:
        out.append("VERDICT: %d stall(s) of %.1f s or longer -- the link "
                   "freezes without dropping, the signature of a marginal "
                   "data joint. Wiggle-watch to localise it."
                   % (len(long_gaps), max(g[1] for g in long_gaps)))
    else:
        out.append("stream is steady -- the USB link is healthy under load.")
    return out
