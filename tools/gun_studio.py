#!/usr/bin/env python3
"""Lightgun Studio -- one window that walks the whole setup, in order:

  1 buttons & pins (OpenFIRE app)   2 camera tuning   3 lens / FOV
  4 aim calibration                 5 fine tune       6 verify

Order matters: aim error scales with blob noise, so the calibration step stays
locked until step 2 reports a usable noise floor. Step 3 only matters when the
camera does not wear the stock 66-degree lens. F9 freezes the cursor while the
window is open; steps that need the gun release it and put it back."""
import os, re, sys, subprocess, threading, queue, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import aim_fit
from aim_calib import (parse_q, is_trigger, sigma_from_hold, find_gun,
                       SerialSource, FRAME_W, FRAME_H)

# Version of the shared Link/serial layer in this file. pical ships as one .py
# beside this tools/ folder and is routinely updated ON ITS OWN -- a stick that
# gets a new pical.py and keeps an old tools/ produces a pical whose features
# silently do nothing, because the parsing they depend on lives HERE. pical
# checks this number at startup and says so out loud instead.
#   1  original                       2  FX: state parsing
#   3  quiet mode, blob gate keys, send failure + liveness reporting
#   4  camera frame-rate meter, blob CSV log, sensor threshold keys
#   5  three-way report format (fmt), the full-mode register, and the box and
#      intensity a full report adds to every blob
#   6  the shape-learning capture: the multi-line 'CAM: hist' reply, held
#      until it is whole, and the CSV both front ends write out of it
#   7  the shape gate (pxmax/armax) and the counters that judge every gate's
#      rejections (bsrej, bfar, bnear). All five are parsed HERE, so a pical
#      copied onto a stick beside an older tools/ would show the two new rows
#      as "--" for ever and never raise the false-negative warning -- with
#      nothing anywhere to say why.
#   8  the height gate (bhmax) and the NINE-field blob tuple: the box origin
#      xmn,ymn joins the width, height and pixel count a full report already
#      carried. An older tools/ drops a nine-field blob on the floor entirely
#      -- parse_blobs only ever accepted 4 and 7 -- so a pical beside one
#      shows "no blobs" on a gun that is sending four of them, which reads as
#      a dead sensor rather than as a version skew.
#   9  '~camfit', the only defensible source of a non-zero gate: the verdict
#      lines are parsed HERE into Link.fit. They are also kept OUT of the
#      key/value store on purpose -- the verdict says "bhmax=11" about a
#      number the gun has NOT set, and an older tools/ reads that token as
#      the live setting, snaps the spinbox to it and sends it back on the
#      next click. A front end beside an older tools/ therefore shows no fit
#      at all AND mis-reports the gate, which is why this one is a version.
#  10  the INERT lines, and the third field of the stored fit. A gate that is
#      SET and cannot act is the one state 'cam?' cannot show -- it reports
#      bhmax and fmt separately and never puts them together -- and the gun
#      now says so at the moment it happens, naming the format it is really
#      in as 'fmt:N' with a COLON, which the key/value sweep cannot see. It is
#      read in pump() and written into last["fmt"], so every front end's idea
#      of the format is corrected by the gun rather than by its own last
#      click. A front end beside an older tools/ goes on showing a gate as
#      working while the gun is telling it otherwise.
LINK_API = 10

SIGMA_GOOD, SIGMA_OK = 0.30, 0.60      # px; see the note above for why this gates
# Board-scaled noise gates: the wiicam's 33-deg lens has ~2.2x less screen
# gain than the OV2640, so the same screen error tolerates 2.2x the sigma.
def sigma_gates(board):
    k = 2.2 if (board and "wiicam" in board) else 1.0
    return SIGMA_GOOD * k, SIGMA_OK * k
APP_PORT_WAIT_S = 60.0                 # how long to keep trying after their app exits
CAM_KEYS = ("thr", "aec", "agc", "boost")
LENS_KEYS = ("lens", "lk1u", "lk2u", "lfpx", "lfeq", "lcxu", "lcyu")
CAM_RANGE = {"thr": (8, 200), "aec": (4, 400), "agc": (0, 30), "boost": (0, 1)}


# ---------------------------------------------------------------------------
# quieting the gun for a measurement
# ---------------------------------------------------------------------------
# A calibration measures where the gun was pointing when the trigger broke. A
# solenoid strike swings the gun and the rumble motor shakes the camera, so
# both have to stop for the duration -- and stopping them is less obvious than
# it looks.
#
# Turning the knobs down does NOT do it. The engine only owns the rumble motor
# while its own rumble time is above zero, so "rumble time 0" -- the setting
# that reads like silence -- hands the motor straight back to OpenFIRE, whose
# off-screen rumble then fires on every calibration shot. That is a real bug
# this code was written to fix, not a hypothetical.
QUIET_REARM_S = 30.0     # firmware quiet mode lapses on its own; hold it down

# The fingerprint of a gun left sitting on the old calibration-quiet values.
LEGACY_QUIET = {"on": 1, "drive": 5, "hold": 0, "pulse": 0}


def is_legacy_quiet(last):
    """Does the gun hold values a calibration put there and never took back?

    A 5 ms strike with no hold, no after-pulses and a silent motor is not a
    setting anyone chooses -- it is what the old quieting wrote. Recognising it
    matters because saving THOSE as 'the user's settings' and restoring them
    afterwards is how a temporary quiet became permanent.
    """
    for k, v in LEGACY_QUIET.items():
        if last.get("fx" + k) != v:
            return False
    return last.get("fxrumms") in (0, 1)


def quiet_plan(last):
    """How this gun can be silenced, from what it has already told us.

    "quiet"  firmware has ~fx=quiet -- one switch: nothing fires and BOTH
             outputs belong to the engine, so the stock rumble cannot run.
    "legacy" older firmware: force the engine on with the smallest strike and
             a 1 ms rumble window. The 1 is what takes the motor away from
             OpenFIRE; a 0 there gives it back.
    "stuck"  the gun is ALREADY sitting on those legacy values. Quiet it the
             same way, but never save and restore them -- that is what makes
             the state permanent.
    "none"   this gun does not speak ~fx; there is nothing to quieten.
    """
    if "fxquiet" in last:
        return "quiet"
    if "fxon" not in last:
        return "none"
    return "stuck" if is_legacy_quiet(last) else "legacy"


# ---------------------------------------------------------------------------
# how fast the camera actually produces frames
# ---------------------------------------------------------------------------
class FrameRate:
    """The camera's TRUE new-frame rate, from the gun's own clock.

    Worth having because nobody knows it. Every figure in circulation for this
    sensor -- "100 Hz" everywhere online, "~300 Hz" in OpenFIRE's own source
    comment -- is unmeasured, and we poll it at 420 Hz, above both. Polling
    faster than the camera produces frames buys nothing: the firmware's
    duplicate-report cache discards repeats, so `bframes` counts only genuinely
    new reports and its rate IS the camera's rate.

    Timed from the gun's millisecond clock rather than the host's, because a
    host-side interval measures this app's poll jitter instead of the camera.
    """

    MIN_MS = 400          # long enough that quantisation is not the answer

    def __init__(self):
        self.frames = None
        self.gun_ms = None
        self.hz = None

    def feed(self, frames, gun_ms):
        if frames is None or gun_ms is None:
            return self.hz
        # A gun that rebooted restarts both counters, so a backwards step is a
        # new baseline rather than a negative rate.
        if (self.frames is None or frames < self.frames
                or gun_ms < self.gun_ms):
            self.frames, self.gun_ms = frames, gun_ms
            return self.hz
        d_f = frames - self.frames
        d_ms = gun_ms - self.gun_ms
        if d_ms >= self.MIN_MS:
            self.frames, self.gun_ms = frames, gun_ms
            self.hz = 1000.0 * d_f / d_ms
        return self.hz


def parse_blobs(line):
    """The per-blob list from a '~camblob?' reply.

    Four fields per blob -- (x, y, size, kept) -- in basic and extended mode,
    and NINE in full mode, where the sensor also reports the blob's bounding
    box and its pixel count:

        (x, y, size, kept, boxw, boxh, px, xmn, ymn)

    Wiibrew calls px an intensity; measured against the box it is plainly a
    count of lit pixels -- it never once exceeded the bounding box across 340
    blobs, and a 2x2 box reads exactly 4 every single time, which a brightness
    would not. The wire name is kept because the CSVs already carry it. The box
    is in the sensor's own 128x96 pixel array; x and y are NOT. The gun
    normalises those out of the sensor's 1024x768 report into the pipeline's
    240x176 space before it records them (wiicam_aim.cpp, x = nx * SX), so a
    box pixel is nearly two units of x or y and the two still cannot be
    compared without scaling -- but by 1.9, not by the 8 that reading them as
    1024x768 would imply.

    SEVEN is the same full report before the box ORIGIN was added to it, and it
    is still accepted: the guns in the field are reflashed one at a time, and a
    tools/ that refused the older report would show "no blobs" on a gun that is
    sending four of them -- which reads as a dead sensor. Everything that needs
    the origin asks for it by length instead (see blob_shape).

    Fields are positional and belong together, so they are parsed as a unit
    rather than scattered into the key/value store. The tuple keeps whatever
    length the gun sent: padding a four-field blob out to nine would hand
    every caller a 0x0 box at the origin that reads exactly like a measured
    one. Any other length is a line the gun's send buffer cut short -- that
    blob is dropped and the whole ones before it are kept, because a truncated
    tail is not a reason to throw away a good frame."""
    out = []
    if not line:
        return out
    for tok in line.replace("CAM: blobs", "").split():
        f = tok.split(",")
        if len(f) not in (4, 7, 9):
            continue
        try:
            out.append(tuple(int(v) for v in f))
        except ValueError:
            pass
    return out


# ---------------------------------------------------------------------------
# what the sensor actually sees, as a shape
# ---------------------------------------------------------------------------
# The sensor's own pixel array. Everything the preview draws lives in it: the
# box fields are 7-bit numbers in this space, and the reported position is
# mapped into it so the two can be compared at all.
NATIVE_W, NATIVE_H = 128.0, 96.0
# How far the box centre may sit from the reported position before the UNITS
# are the suspect rather than the blob. The two are measured differently -- the
# position is an intensity centroid and the centre is the middle of a bounding
# box -- so they never coincide exactly, and a smeared blob at sensitivity 2
# can put a pixel or two between them. Ten times that is not a blob being
# lopsided, it is the box fields not being in this array at all, and the whole
# shape analysis would then be built on a unit nobody has checked.
SHAPE_OFF_MAX = 3.0
# The format the GUN says it is really in, out of either INERT line. Both name
# it with a colon rather than an '=', so the key/value sweep in pump() is blind
# to it. Anchored on "is in" and not on the last 'fmt:' of the line: the fit
# form ends with "set fmt:2 for it to act", which is advice about where to go
# and not a reading of where the gun is.
_INERT_FMT_RE = re.compile(r"is in fmt:(\d+)")
# When the shape gate counts as "refusing heavily", in blobs refused per camera
# FRAME over the window the readout measures.
#
# What is counted is the UNEXPLAINED part: rejections minus the ones the
# resolver vouched for. The first version counted raw rejections at one per
# frame, reasoning that the resolver can name only one stray a frame so one
# rejection a frame is where honest work ends. The log said otherwise: a 92 s
# daylight session with bhmax:8 held four straight seconds at EXACTLY 1.00
# rejected per frame, br4 zero, br3 every frame, bnear zero throughout. That
# is the sun holding one of the four slots, the gate refusing it every frame,
# the gun running on three real corners and a reconstructed fourth -- the gate
# doing its job in the very scene it exists for, and a raw threshold of one
# per frame fired for the whole stretch.
#
# What separates that from a gate eating the bar is already on the wire. Both
# vouching counters are SHAPE-GATE rejections and nothing else -- the gun
# narrowed them: bfar is a blob the shape gate dropped that also sat far from
# every corner (a stray, vouched for), bnear one the shape gate dropped that
# sat where the missing LED had to be (almost certainly a real corner). They
# count on every frame, capture on or off. That matters for the subtraction:
# while bfar counted every far blob it held kept strays too, and one kept
# stray a frame cancelled the threshold and masked a gate eating a real LED,
# and a bnear that could come from the size window sent the reader to switch
# off a height gate that was not the one doing it. Narrowed, bsrej minus the
# two is exactly "shape rejections the resolver could not account for".
#
# That session climbed bsrej by 829 and bfar by 730: 88% vouched for. A gate
# set for a different bar refuses three or four LEDs a frame, the resolver
# then has too few points to lock, and nothing gets vouched for at all -- bfar
# and bnear stay flat while bsrej runs away. So the measure is rejections
# minus what the resolver accounted for, and one of THOSE per frame is
# unambiguous: a whole slot, every frame, that nobody can say was stray light.
#
# pical warns off the same number under the same name, and that is the point of
# the name: this was 0.25 per frame here and 2.0 there, an eight-fold
# disagreement about one measurement, so a gate refusing a third of every blob
# the camera saw warned in Studio and stayed silent in pical on the same gun in
# the same room. One number, one measurement, and a divergence now shows up as
# two different figures in two files that read the same.
#
# A RATE and not a total, because bsrej counts since boot: the last warning
# driven off one of these raw never came off the screen again.
GATE_HEAVY_PER_FRAME = 1.0
# How old the last NEW FRAME may be before the gate warnings stop claiming
# anything. They are computed when a window of frames closes and they describe
# what is happening now, so on a gun that has stopped answering -- unplugged,
# camera dead, port handed to another app -- the last verdict has to go quiet
# rather than sit there being read as current. Without this the warning stayed
# up for as long as nothing arrived, which is the same latch this panel has
# already shipped twice: bvalve read raw, and a delta that never expired.
GATE_WARN_STALE_S = 8.0


def blob_shape(b):
    """One blob's geometry for the preview, in the sensor's native 128x96 array.

    Returns a dict, or None for a blob with no shape in it at all:

      kept     the gun's own verdict, so the drawing can separate the blobs it
               kept from the ones a gate dropped
      cross    the REPORTED position, mapped out of the 240x176 pipeline into
               this array. This is the measurement the preview exists to make:
               if the box fields really are in the sensor's 128x96 array then
               the box centre lands on it, and if they are not, nothing else
               about the shape numbers can be trusted either.
      box      (left, top, w, h) of the bounding box, in the same array. The
               wire carries xmx-xmn, so a one-pixel blob reports 0 and the drawn
               extent is w+1 by h+1 -- the same +1 the density is divided by.
      origin   True when the gun sent xmn,ymn. On the seven-field report there
               is no origin, so the box is centred on the crosshair instead:
               that is an ASSUMPTION and the drawing has to say so, because a
               box parked on the crosshair agrees with it by construction and
               would read as the measurement passing.
      density  px / ((w+1)*(h+1)) -- how much of its own box the blob fills. A
               point source fills nearly all of it; a window reflection or a
               smear does not.
      off      how far the box centre sits from the crosshair, in this array's
               pixels, or None when there is no origin to measure it from.
    """
    if not b or len(b) < 4:
        return None
    cross = (b[0] * NATIVE_W / FRAME_W, b[1] * NATIVE_H / FRAME_H)
    out = {"kept": b[3] == 1, "cross": cross, "box": None, "origin": False,
           "density": None, "off": None}
    if len(b) < 7:
        return out
    w, h, px = b[4], b[5], b[6]
    # Divided by the drawn extent, not by w*h: the wire's width is a
    # difference of two coordinates, so a 1x1 blob arrives as 0x0 and a
    # density of px/0 is either a crash or an infinity, on the single
    # commonest blob this sensor produces.
    area = (w + 1) * (h + 1)
    out["density"] = px / float(area)
    if len(b) >= 9:
        out["box"] = (float(b[7]), float(b[8]), float(w), float(h))
        out["origin"] = True
        cx = b[7] + w / 2.0
        cy = b[8] + h / 2.0
        out["off"] = ((cx - cross[0]) ** 2 + (cy - cross[1]) ** 2) ** 0.5
    else:
        out["box"] = (cross[0] - w / 2.0, cross[1] - h / 2.0,
                      float(w), float(h))
    return out


class BlobLog:
    """One CSV row per NEW camera frame, for reading afterwards on a PC.

    The Pi has no console, and the contamination we are chasing only happens at
    the TV -- so the numbers have to be captured where the gun is and read
    somewhere else. On the stick this lands on the FAT boot partition, next to
    pical.py, which any PC can open.

    Rows are written only when the gun's frame counter has MOVED: polling adds
    samples, not frames, and a file full of repeated frames would overstate
    every rate computed from it."""

    # Anything new goes on the END, even when it belongs beside a column that
    # is already there: the captures taken at the TV are the whole reason this
    # file exists, and they are read months later against whatever this tuple
    # said at the time. Slotting w0,h0,i0 in after k0 would be tidier and would
    # quietly turn every one of those files into nonsense.
    COLS = ("wall", "gun_ms", "bframes", "hz", "bn",
            "ext", "bmin", "bmax", "rtol", "hwmax", "hwmin", "sens",
            "brej", "brrej", "bvalve", "bdrop",
            "br4", "br3", "br2", "br1", "br0",
            "x0", "y0", "s0", "k0", "x1", "y1", "s1", "k1",
            "x2", "y2", "s2", "k2", "x3", "y3", "s3", "k3",
            "fmt", "fullreg",
            "w0", "h0", "i0", "w1", "h1", "i1",
            "w2", "h2", "i2", "w3", "h3", "i3",
            # The two halves of "was that rejection right?", and the shape
            # gate's own count. bfar and bnear reached the firmware after this
            # file was last touched and were never written down, so every
            # capture taken since answers the one question the log exists to
            # answer -- is the gate throwing away real LEDs? -- with silence.
            # bnear is the false-negative meter: it counts blobs a gate
            # dropped that sat exactly where the missing corner should have
            # been, and it moves long before the cursor starts sticking.
            "bfar", "bnear", "bsrej",
            # ...and the settings those counts were taken under. A pixel-count
            # and roundness limit read months later mean nothing unless the
            # row says which limits were in force when it was written.
            "pxmax", "armax",
            # The box ORIGIN, which is what turns w,h from a size into a
            # place. Without it a capture cannot be checked against the
            # position beside it, so "are these fields really in the sensor's
            # 128x96 array?" -- the question every conclusion drawn from the
            # box depends on -- is unanswerable from the file afterwards.
            # On the end, behind the shape-gate columns, for the same reason
            # everything else is: the captures already taken are read by index.
            "xm0", "ym0", "xm1", "ym1", "xm2", "ym2", "xm3", "ym3",
            # The gate that actually works, and therefore the one setting a
            # row most needs to name. It belongs beside pxmax and armax and it
            # is nowhere near them, because moving those two down one column
            # would silently reinterpret every capture on the stick.
            "bhmax")

    def __init__(self, path):
        self.path = path
        self.rows = 0
        self._last_frames = None
        self._f = open(path, "w")
        try:
            self._f.write(",".join(self.COLS) + "\n")
            self._f.flush()
        except Exception:
            # A full stick raises here; without this the handle is orphaned,
            # because __init__ never returns and nothing is left to close it.
            self._f.close()
            raise

    def sample(self, last, blobs_line, hz):
        """Returns True when a row was actually written."""
        frames = last.get("bframes")
        if frames is None or frames == self._last_frames:
            return False
        self._last_frames = frames
        vals = [time.strftime("%H:%M:%S"),
                last.get("bms", ""), frames,
                ("%.1f" % hz) if hz else ""]
        vals.append(last.get("bn", ""))
        for k in ("ext", "bmin", "bmax", "rtol", "hwmax", "hwmin", "sens",
                  "brej", "brrej", "bvalve", "bdrop",
                  "br4", "br3", "br2", "br1", "br0"):
            vals.append(last.get(k, ""))
        blobs = parse_blobs(blobs_line)
        for i in range(4):
            if i < len(blobs):
                vals.extend(blobs[i][:4])
            else:
                vals.extend(("", "", "", ""))
        vals.append(last.get("fmt", ""))
        vals.append(last.get("fullreg", ""))
        # Empty, not zero, when the gun was not in full mode: a 0x0 box and a
        # brightness of 0 are what an unlit blob would look like, and a reader
        # averaging these columns months later has no way to tell a
        # measurement of nothing from a measurement that never happened.
        for i in range(4):
            b = blobs[i] if i < len(blobs) else ()
            vals.extend(b[4:7] if len(b) in (7, 9) else ("", "", ""))
        # Blank, not zero, on a gun too old to send them: a counter that reads
        # 0 for a whole capture is indistinguishable from a gate that never
        # fired, and "bnear stayed at zero" is exactly the conclusion this
        # column is used to draw.
        for k in ("bfar", "bnear", "bsrej", "pxmax", "armax"):
            vals.append(last.get(k, ""))
        # The origin, and blank on the seven-field report for a harder reason
        # than the others: 0,0 is a real corner of the array. Written as zeros,
        # a whole capture off a gun that never sent the origin would pile every
        # blob into the top-left of any plot drawn from it, and the plot would
        # look like a sensor fault rather than like a missing column.
        for i in range(4):
            b = blobs[i] if i < len(blobs) else ()
            vals.extend(b[7:9] if len(b) == 9 else ("", ""))
        vals.append(last.get("bhmax", ""))
        self._f.write(",".join(str(v) for v in vals) + "\n")
        # Flushed every row: a stick pulled out of a running Pi otherwise keeps
        # an empty file, because the writes are still in the page cache.
        self._f.flush()
        self.rows += 1
        return True

    def close(self):
        """Returns False if the final flush failed -- a caller that reports
        "N rows written" should not claim rows that never reached the disk."""
        try:
            self._f.close()
            return True
        except Exception:
            return False


# ---------------------------------------------------------------------------
# the shape-learning histograms
# ---------------------------------------------------------------------------
# '~camlearn?' answers with a summary line and then ONE LINE PER class and
# feature -- twelve of them, because a single reply carrying 384 numbers fits
# no buffer the gun has. The names, the two classes and the bin count are the
# firmware's own (lib/WiicamAim/wiicam_learn.h) and are pinned here rather
# than read off the wire: a build that changes any of them is a different
# generation of this parsing, which is what LINK_API is for.
HIST_FEATS = ("sz", "bw", "bh", "aspect", "area", "irel")
HIST_CLASSES = 2
HIST_BINS = 32
# Class 0 is a blob in a frame where the quad resolver confirmed all four
# corners -- a known-good LED. Class 1 is a blob a gate rejected in one of
# those same frames. Written into the CSV by NAME rather than as 0 and 1: the
# file is opened weeks later in a spreadsheet with nothing beside it to say
# which number meant which, and a distribution read under the wrong label is
# worse than no distribution at all.
CLASS_NAMES = ("led", "rej")
SHAPE_COLS = (("class", "feature", "frames", "led_blobs", "rej_blobs")
              + tuple("b%d" % i for i in range(HIST_BINS)))


class HistSet:
    """The multi-line '~camlearn?' reply, held until it is WHOLE.

    Twelve lines arrive one after another behind a summary line, in among the
    frame stream, and any of them can be missed: a front end can start
    listening half way through a reply, and two replies run together when the
    gun is asked twice. So a set is assembled off to one side and only
    PUBLISHED once every class and feature is present -- nothing half-arrived
    ever replaces what is held.

    That matters because the only thing done with a held set is writing it to
    a file. A CSV with four rows from this capture and eight from the previous
    one is not a measurement of anything, and there is nothing about the file
    afterwards that would look wrong -- the numbers are all plausible, they
    just came from two different rigs.

    `seq` counts published sets, so a caller that has just asked the gun can
    tell a fresh answer from the one that was already there."""

    def __init__(self):
        self.summary = {}       # the counts that arrived WITH the held set
        self.rows = {}          # (class index, feature name) -> bin counts
        self.seq = 0            # bumped only when a whole new set is published
        self._sum = None        # the reply being collected right now
        self._rows = {}

    def reset(self):
        """Forget everything, keeping seq. Used when the port is reopened: a
        histogram belongs to the gun and the light it was measured under, and
        a reconnect may be onto a different gun entirely. seq keeps counting
        so a caller waiting for a fresh set is not fooled by the reset."""
        self.summary, self.rows = {}, {}
        self._sum, self._rows = None, {}

    def feed(self, line):
        """Take one 'CAM: learn' / 'CAM: hist' line. True if it was ours."""
        if line.startswith("CAM: learn "):
            s = {}
            for tok in line.split():
                k, sep, v = tok.partition("=")
                if sep:
                    try:
                        s[k] = int(v)
                    except ValueError:
                        pass
            # 'CAM: learn ON -- ...' and 'CAM: learn cleared' are status
            # messages for a human, not reports; only the report carries the
            # bin count, so that is what a report is recognised by.
            if "bins" not in s:
                return False
            # A report starts here, and it drops whatever was half collected:
            # the counts on THIS line are the ones the rows below it were
            # measured with, and rows from an older reply are not.
            self._sum, self._rows = s, {}
            return True
        if not line.startswith("CAM: hist "):
            return False
        cls, feat, counts = None, None, []
        for tok in line.split()[2:]:
            k, sep, v = tok.partition("=")
            if sep:
                if k == "c":
                    try:
                        cls = int(v)
                    except ValueError:
                        return False
                elif k == "f":
                    feat = v
                continue
            try:
                counts.append(int(tok))
            except ValueError:
                pass
        if feat not in HIST_FEATS or cls is None \
                or not (0 <= cls < HIST_CLASSES):
            return False
        if self._sum is None:
            # The summary says how many frames these bins came from, and it
            # comes first. Rows with no summary in front of them are the tail
            # of a reply we joined half way through; kept, they would be
            # written out under the PREVIOUS capture's counts.
            return False
        key = (cls, feat)
        if key in self._rows:
            # The same feature twice means two replies have run together --
            # one of them lost its summary line, so neither is trustworthy as
            # a whole. The part-built set goes; what is already published is
            # left alone, and the next clean reply is waited for.
            self._sum, self._rows = None, {}
            return False
        self._rows[key] = counts
        if len(self._rows) == HIST_CLASSES * len(HIST_FEATS):
            # Published by rebinding, not by mutating: a caller that took a
            # reference to the last set goes on reading a whole one.
            self.summary, self.rows = self._sum, self._rows
            self.seq += 1
            self._sum, self._rows = None, {}
        return True

    def ready(self):
        """Is a whole set held? False until the gun has answered once."""
        return bool(self.rows)

    def counts(self):
        """(frames, LED blobs, rejected blobs) from the held set's summary."""
        s = self.summary
        return (s.get("frames", 0), s.get("led", 0), s.get("rej", 0))

    def running(self):
        """Whether the GUN said the capture was on, not whether we asked for
        it: '~camreset' stops a capture from anywhere, and a front end that
        believed its own last click would go on claiming to be recording."""
        return bool(self.summary.get("on"))

    def total(self, cls, feat):
        """How many samples landed in one histogram. Zero means the feature
        was never fed -- which is what every box-derived feature looks like
        after a capture taken outside full mode."""
        return sum(self.rows.get((cls, feat), ()))


def write_shape_csv(path, hs):
    """The held histograms, one row per (class, feature). Returns the rows.

    Wide and repetitive on purpose. This file is carried to a PC on the stick
    and opened in a spreadsheet, so the counts are repeated on every row
    rather than kept on a header line of their own: the first sort would put
    such a line in the middle of the data and the rest of the file would no
    longer say what it was measured from."""
    frames, led, rej = hs.counts()
    n = 0
    with open(path, "w") as fh:
        fh.write(",".join(SHAPE_COLS) + "\n")
        for cls in range(HIST_CLASSES):
            for feat in HIST_FEATS:
                counts = hs.rows.get((cls, feat))
                if counts is None:
                    continue
                # Fewer than 32 numbers means the gun's line buffer cut the
                # tail off this feature. Blank, not zero: zero is a bin
                # nothing landed in, which is a measurement, and a missing
                # tail written as zeros is a distribution that leans left.
                bins = [str(c) for c in counts[:HIST_BINS]]
                bins += [""] * (HIST_BINS - len(bins))
                fh.write(",".join([CLASS_NAMES[cls], feat, str(frames),
                                   str(led), str(rej)] + bins) + "\n")
                n += 1
    return n


# ---------------------------------------------------------------------------
# the measured gate: what '~camfit' answered
# ---------------------------------------------------------------------------
# The gun turns the two distributions it has been accumulating -- how tall its
# OWN LEDs come out, and how tall the stray light in the room does -- into a
# ceiling, or says out loud that this rig has none. It is the only defensible
# source of a non-zero bhmax: every figure that used to be printed beside the
# control was measured on one bar with two LEDs per corner, and a bar with five
# LEDs per cluster makes blobs several times larger, so a borrowed ceiling
# blinds that gun and leaves its owner nothing on screen to explain why it
# stopped working.
#
# The targets are the firmware's own and are stated in its own sentences, so
# they are read off the wire when they are there and only fall back to these:
# a build that wants more data than this would otherwise be reported as
# "600 of 500", which is not a progress figure, it is a bug on screen.
FIT_LED_WANT, FIT_STRAY_WANT = 500, 20


class CamFit:
    """The '~camfit' reply, held as an answer rather than as lines of log.

    Several lines, and ANY of them can be absent: three of the four outcomes
    are a single sentence, the stored-provenance line only appears when the
    gun has one, the 'applied' line only follows '=apply', and a gun on older
    firmware answers nothing at all. So every field starts as None and stays
    that way until the gun says otherwise -- a front end asks this object and
    gets "not measured", never a zero that reads like a measurement of zero.

    `seq` counts ANSWERS, bumped on the header line that always comes first.
    A caller that has just asked can tell a fresh reply from the one that was
    already held, which is the only way to tell an old gun's silence from an
    answer that has not arrived yet."""

    def __init__(self):
        self.seq = 0
        self.reset()

    def reset(self):
        """Forget the answer, keeping seq -- used on the header line and on a
        reconnect. A fit belongs to the gun, the bar and the room it was
        measured in, and a reconnect may be onto a different gun entirely."""
        self.led_n = self.stray_n = None      # blobs counted, each class
        self.led_max_h = self.stray_min_h = None   # rows; <0 = not measured
        self.led_want, self.stray_want = FIT_LED_WANT, FIT_STRAY_WANT
        # (led_max_h, stray_min_h, led_max_px) from an earlier capture on
        # this gun, and any of the three may be None: the pixel figure is the
        # newest field and an older gun sends the pair without it.
        self.stored = None
        # "" | "need_led" | "need_stray" | "no_gate" | "ok"
        self.verdict = ""
        self.bhmax = None       # the number, and ONLY on a real verdict
        self.tight = False      # one row between the LEDs and the stray light
        self.applied = None     # True saved, False save failed, None neither
        # (samples, how tall they reached, where the rest stop) for LED
        # samples the gun set aside as contamination -- the sun or a window
        # learned into the LED class while the resolver was locking on it.
        # Held because it is the difference between a ceiling of 7 that means
        # something and one that was measured with a 31-tall blob in the
        # class: the number is the same either way and the trust in it is not.
        self.ignored = None
        # The gun changed format for the gate on the way past. It applies in
        # full mode and persists full mode, so this is a thing that HAPPENED
        # rather than an instruction, and the panel's own format control has
        # to follow it.
        self.switched = False

    def _num(self, rest, after, before):
        """The integer between two phrases of the gun's own sentence, or None.

        The counts are in the header as well, but the TARGETS are only in
        these sentences -- and a target this file pinned instead of read
        would go on saying 500 through a firmware that wanted a thousand."""
        i = rest.find(after)
        if i < 0:
            return None
        j = rest.find(before, i + len(after))
        if j < 0:
            return None
        try:
            return int(rest[i + len(after):j].strip())
        except ValueError:
            return None

    def feed(self, line):
        """Take one 'CAM: fit' line. True if it was ours.

        Nothing here raises on a line it does not recognise: a firmware that
        adds a sentence must leave the held answer alone rather than clearing
        it, so an unknown line is simply claimed and dropped."""
        if not line.startswith("CAM: fit"):
            return False
        rest = line[8:].strip()
        if rest.startswith("ledn="):
            # The header, which both '?' and '=apply' send first: a NEW answer
            # starts here and everything held from the last one goes. Kept, a
            # verdict from the previous ask would sit under this ask's counts
            # and read as this rig's current state.
            self.reset()
            self.seq += 1
            for tok in rest.split():
                k, sep, v = tok.partition("=")
                if not sep:
                    continue
                try:
                    n = int(v)
                except ValueError:
                    continue
                if k == "ledn":
                    self.led_n = n
                elif k == "ledmaxh":
                    self.led_max_h = n
                elif k == "straym":
                    self.stray_n = n
                elif k == "strayminh":
                    self.stray_min_h = n
            return True
        if "LED samples ignored" in rest:
            # 'CAM: fit 32 LED samples ignored -- they reach 31 tall, far
            # above the 7 the rest stop at, ...'. It begins with a NUMBER, so
            # it matches none of the prefixes below and fell through both this
            # parser and the panel: the one line that says the capture had
            # stray light in its LED class was the one line nothing read.
            try:
                n = int(rest.split()[0])
            except (ValueError, IndexError):
                return True
            self.ignored = (n, self._num(rest, "they reach", "tall"),
                            self._num(rest, "far above the", "the rest"))
            return True
        if rest.startswith("STORED"):
            # Read as a whole line, and by NAME: the pixel figure was added to
            # the end of this line after the other two, so a gun on either
            # firmware sends a different number of fields and neither may be
            # taken by position. A field this gun did not send stays None --
            # 0 px is a measurement, and "no measurement" is not one.
            got = {}
            for tok in rest.split():
                k, sep, v = tok.partition("=")
                if not sep:
                    continue
                try:
                    got[k] = int(v)
                except ValueError:
                    pass
            self.stored = (got.get("ledmaxh"), got.get("strayminh"),
                           got.get("ledmaxpx"))
            return True
        if rest.startswith("NEEDS MORE LED DATA"):
            self.verdict = "need_led"
            w = self._num(rest, "blobs so far,", "wanted")
            if w is not None:
                self.led_want = w
            return True
        if rest.startswith("NO STRAY DATA"):
            self.verdict = "need_stray"
            w = self._num(rest, "seen,", "wanted")
            if w is not None:
                self.stray_want = w
            return True
        if rest.startswith("NO SAFE GATE"):
            # No number is held here, deliberately. This rig cannot be gated
            # on size, and a front end that kept the last verdict's figure
            # would offer to apply a ceiling measured before the lamp came on.
            self.verdict = "no_gate"
            return True
        if rest.startswith("bhmax="):
            try:
                self.bhmax = int(rest[6:].split()[0].strip("(,"))
            except (ValueError, IndexError):
                return True
            self.verdict = "ok"
            self.tight = "TIGHT" in rest
            return True
        if rest.startswith("applied and saved"):
            self.applied = True
        elif rest.startswith("applied but SAVE FAILED"):
            self.applied = False
        elif rest.startswith("switched to fmt:2"):
            # An apply from any other format switches the gun to full mode and
            # saves full mode with the gate. The line this replaced said the
            # gate was INERT and left the user to fix it, and both front ends
            # had grown a sentence telling them to press Save as well -- for a
            # command called 'apply' that did not survive a reboot.
            self.switched = True
        return True


# ---------------------------------------------------------------------------
# naming a capture so the next one cannot land on it
# ---------------------------------------------------------------------------
# Files used to be named from the wall clock alone. Two captures inside the
# same second overwrite each other in silence -- and one second is not a
# hypothetical here, because the way these get taken is press, look, press
# again. The rest of the time the clock is merely unhelpful: it does not say
# which capture came first when the run crossed midnight, and it gives the two
# people in this conversation nothing shorter to say than fourteen digits.
#
# So every file also carries a sequence number, scanned off the directory it
# is about to land in: highest existing + 1. That makes the name sort in
# creation order, gives the user a "number 4" to ask for, and -- with the
# existence check below -- cannot name a file that is already there.
SEQ_DIGITS = 3
# Anchored, and no more than six digits before the dash, so the older
# wall-clock names are NOT read as sequence numbers: 'shape-20260901-1131.csv'
# would otherwise be capture number twenty million, and every file after it
# would carry that number too once the widths stopped matching.
_SEQ_RE = r"^%s-(\d{1,6})-"


def seq_path(outdir, prefix, ext=".csv", now=None):
    """(path, number) for the next capture in `outdir`. Never an existing file.

    The clock stays in the name because it is what identifies the moment; the
    number is what orders it. `now` is for the tests, which cannot wait a
    second to prove that two captures do not collide."""
    n = 0
    try:
        for f in os.listdir(outdir):
            m = re.match(_SEQ_RE % re.escape(prefix), f)
            if m:
                n = max(n, int(m.group(1)))
    except OSError:
        pass                    # no directory yet, so this is number 1
    stamp = now if now is not None else time.strftime("%H%M%S")
    # Nothing here may return a path that exists. The scan above is enough on
    # its own for files this app wrote, but the directory is a FAT partition
    # somebody else also copies into, and overwriting a capture that cannot be
    # taken again is the one failure this whole scheme exists to prevent.
    while True:
        n += 1
        p = os.path.join(outdir, "%s-%0*d-%s%s"
                         % (prefix, SEQ_DIGITS, n, stamp, ext))
        if not os.path.exists(p):
            return p, n


def port_is_free(port):
    """Can this port be opened right now? Used only to tell whether another
       process still holds it. Deliberately does NOT touch the Link object --
       that one is owned by the Tk tick loop and opening it from a worker
       thread would race the pump()."""
    try:
        import serial
        s = serial.Serial(port, 115200, timeout=0.2)
        s.close()
        return True
    except Exception:
        return False


def take_port_back(proc, port, timeout_s=APP_PORT_WAIT_S, probe=port_is_free,
                   sleep=time.sleep, clock=time.monotonic):
    """Block until `proc` has exited AND `port` can be opened again.

       Returns "no port", "free" or "timeout". The caller reconnects either way:
       a timeout is a thing to report, not a reason to leave the UI parked on a
       status that will never change on its own.

       The two waits are separate on purpose. Some launchers start the real app
       in a second process and exit immediately, so proc.wait() returning does
       not mean the port is back."""
    try:
        if proc is not None:
            proc.wait()
    except Exception:
        pass
    if not port:
        return "no port"
    deadline = clock() + timeout_s
    while clock() < deadline:
        if probe(port):
            return "free"
        sleep(1.0)
    return "timeout"


# ---------------------------------------------------------------------------
# a serial link that owns the port and can be handed over to a child process
# ---------------------------------------------------------------------------
class Link:
    def __init__(self):
        self.src = None
        self.port = None
        self.frames = 0
        self.hist = []            # recent quads, for the live view and sigma
        self.last = {}
        self.replies = []
        self.hid_on = True        # the gun boots this way; we do not change it uninvited
        self.partial_t = 0.0      # wall clock of the last <4-LED frame
        self.partial_n = 0        # running count of <4-LED frames
        self.full_t = 0.0         # wall clock of the last 4-LED frame
        # Optional consumers, so another front end can drive a capture session
        # from this same stream. Unused by Studio.
        self.gun_t = 0.0          # the gun's own clock, from the last frame
        self.sink = None          # called with (quad, gun_time_s)
        self.trig_sink = None     # called on a trigger marker
        self.blobs = ""           # last "CAM: blobs ..." line, raw
        # The shape-learning histograms, assembled across the twelve lines
        # they arrive on. Kept whole in one object rather than as loose keys
        # for the same reason the blob list is: the bins of one feature are a
        # distribution, and half of one is not a smaller distribution.
        self.hists = HistSet()
        # The gun's own verdict on what a size gate can do on THIS rig, held
        # whole for the same reason: its lines are an answer, not a reading,
        # and the number in one of them is a PROPOSAL rather than a setting.
        self.fit = CamFit()
        # Writes that failed. A serial write throwing is how a gun that
        # rebooted or re-enumerated announces itself, and swallowing it made a
        # dead link look exactly like a screen whose keys had stopped working.
        self.send_fails = 0

    def connect(self, port=None):
        """False on failure, never an exception: opening a stale COM name
        raises inside SerialSource, and that exception used to die in Tk's
        callback with the header stuck on "looking for the gun...". A gun
        that re-enumerated on a NEW port is found by falling back to a scan."""
        for cand in (port or find_gun(), None if port is None else find_gun()):
            if not cand:
                continue
            try:
                self.src = SerialSource(cand)
            except Exception:
                self.src = None
                continue
            self.port = cand
            # Everything we knew belonged to the PREVIOUS gun. Keeping it
            # meant a reconnect onto a different (or reflashed) gun answered
            # questions about it from the old one's replies -- including
            # "does this firmware have quiet mode", which decides whether a
            # calibration runs with a live solenoid. The replies that follow
            # a connect refill all of it within a frame or two.
            self.last.clear()
            self.blobs = ""
            # And the histograms with them. A distribution measured on the
            # previous gun, under the previous light, written out as this
            # one's is exactly the two-rigs-in-one-spread failure the capture
            # clears itself to avoid.
            self.hists.reset()
            # And the fit, which is a statement about one bar in one room. A
            # ceiling measured on the gun that was here a minute ago is the
            # borrowed number this whole command exists to stop.
            self.fit.reset()
            self.replies = []
            self.src.start()
            return True
        return False

    def close(self):
        if self.src:
            try: self.src.close()
            except Exception: pass
            self.src = None

    def alive(self):
        """Is the reader thread still on the port?

        Its run loop breaks out on any serial exception -- which is what a gun
        rebooting or re-enumerating looks like from here -- and the thread then
        ends quietly. Nothing about the window changes when that happens: the
        last values stay on screen and every key still 'works', it just reaches
        a port nobody is reading. Asking this is how a front end tells the
        difference between a frozen gun and a frozen link."""
        if not self.src:
            return False
        return bool(getattr(self.src, "is_alive", lambda: True)())

    def send(self, line):
        """Returns True when the bytes reached the port. Callers that care
        (anything the user pressed a key for) can then say so."""
        if not self.src: return False
        # The '~' matters. aim_runtime_command accepts a bare name, but the
        # gatekeeper on the shared serial only CLAIMS lines starting with '~' --
        # anything else is passed through to OpenFIRE and silently discarded.
        # An auto-installed calibration went missing exactly this way.
        if not line.startswith("~"): line = "~" + line
        try:
            self.src.ser.write(("\n%s\n" % line).encode())
            return True
        except Exception:
            self.send_fails += 1
            return False

    def pointer(self, on, remember=True):
        """Freeze or release the cursor.

        The gun boots with the pointer ON and this app does NOT take it away by
        itself -- freezing is a thing the user asks for, with a key, and the
        window says so. Doing it silently on connect meant opening the app made
        the gun stop working with no visible cause.

        `remember=False` forces the pointer on temporarily (calibration, verify,
        or handing the port to another app) without forgetting that the user had
        chosen frozen, so their choice comes back afterwards.

        RULE: we can only change this while we own the serial port. Every path
        that gives the port away must force it ON first, or the user is left
        frozen until they replug."""
        if remember:
            self.hid_on = on
        self.send("~aimhid=%d" % (1 if on else 0))

    def pump(self):
        """drain the stream; keep the last ~2 s of quads"""
        if not self.src: return
        n = 0
        while n < 400:
            try: line = self.src.q.get_nowait()
            except queue.Empty: break
            n += 1
            if line.startswith("FX:"):
                # recoil engine state: keys stored with an fx prefix so they
                # can never collide with the camera keys
                self.replies.append(line)
                for tok in line.replace("FX:", "").split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        try: self.last["fx" + k] = int(v)
                        except ValueError: pass
                continue
            # The shape-learning reply. Its twelve 'CAM: hist' lines are DATA,
            # and they stop here: Studio's log box is six lines tall and
            # pical's log is a 200-line ring, so one poll of a running capture
            # would push every message the user was told to watch for -- the
            # diag verdict, the save result -- straight off the end of both.
            # The summary line above them is one line a human can read, so it
            # goes on through to the log like any other answer.
            if line.startswith("CAM: hist "):
                self.hists.feed(line)
                continue
            if line.startswith("CAM: learn "):
                self.hists.feed(line)
            # Two lines name the format the GUN is really in, and both name
            # it with a colon -- so the key/value sweep below is blind to
            # both. Read here, into last["fmt"], because the gun's word on its
            # own format beats this app's memory of the last click: a format
            # changed from a serial terminal or from pical on the same gun
            # otherwise leaves this front end showing a gate as working while
            # the gun is telling it, in words, that the gate does nothing.
            #
            # 'bhmax N set but INERT ... this gun is in fmt:N' is the gate
            # being set where it cannot act. 'fit switched to fmt:2' is the
            # opposite: an apply moved the gun INTO full mode and saved it
            # there, so the panel that was showing INERT a moment ago has to
            # stop -- and its format radio has to move with the gun.
            #
            # Both are a fresher reading of the same fact and not sticky
            # flags: the next '~camblob?' answer overwrites them like any
            # other, which is what stops this latching the way bvalve did.
            if "INERT" in line:
                m = _INERT_FMT_RE.search(line)
                if m:
                    try:
                        self.last["fmt"] = int(m.group(1))
                    except ValueError:
                        pass
            elif line.startswith("CAM: fit switched to fmt:2"):
                self.last["fmt"] = 2
            # The fit verdict, taken off the wire BEFORE the key/value sweep
            # below can see it. Its middle line reads 'CAM: fit bhmax=11 (LEDs
            # reach 7...)' -- a number the gun has NOT set and will not set
            # until it is asked again with '=apply'. Fed to that sweep it
            # lands in last["bhmax"], the spinbox snaps to a gate nobody
            # turned on, and the next click sends it. The line still goes to
            # replies: it is the plainest English on the wire and the log is
            # where the long form belongs.
            if line.startswith("CAM: fit"):
                self.fit.feed(line)
                self.replies.append(line)
                continue
            if line.startswith("CAM:") or line.startswith("AIM:") or "CMD ok" in line:
                self.replies.append(line)
                # The per-blob list is kept whole: its fields are positional
                # (x, y, size, kept) and belong together, not as loose keys.
                if line.startswith("CAM: blobs"):
                    self.blobs = line.strip()
                # keep the label honest if the gun disagrees with us
                if "pointer FROZEN" in line: self.hid_on = False
                elif "pointer ON" in line:   self.hid_on = True
                # Any CAM:/pong/ack line may carry k=v state -- the old
                # "CAM: thr=" prefix missed the wiicam's "CAM: board=" readback
                # AND the ping's board tag, so Studio never learned the board
                # and kept showing the ESP32 tuning panel.
                if line.startswith("CAM:") or line.startswith("AIM: pong") \
                        or "CMD ok" in line:
                    for tok in line.replace("|", " ").split():
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            if k == "board":
                                self.last["board"] = v
                            elif k in CAM_KEYS or k in LENS_KEYS \
                                    or k in ("sens", "dead", "lead", "smooth",
                                                 "beta", "tmode",
                                                 "firk", "firpct",
                                                 # blob gate + its counters.
                                                 # fmt is the report format
                                                 # (0/1/2); ext is the same
                                                 # gun saying only whether
                                                 # sizes are available, kept
                                                 # because a gun on older
                                                 # firmware sends nothing else.
                                                 "ext", "fmt", "fullreg",
                                                 "bmin", "bmax", "bn",
                                                 "brej", "bframes", "bdrop",
                                                 "br4", "br3", "br2", "br1",
                                                 "br0", "brrej", "bvalve", "bms",
                                                 "rtol", "hwmax", "hwmin",
                                                 # The shape gate and the two
                                                 # counters that judge every
                                                 # gate's rejections. Left out
                                                 # of this tuple they never
                                                 # reach last[] at all, so the
                                                 # CSV column and the readout
                                                 # would both be permanently
                                                 # blank with nothing on the
                                                 # wire to blame for it.
                                                 # bhmax is the gate that
                                                 # actually works, so it is
                                                 # also the one whose spinbox
                                                 # would sit at 'off' while
                                                 # the gun was gating -- and
                                                 # the next click would then
                                                 # send the gun a setting the
                                                 # user never chose.
                                                 "bhmax", "pxmax", "armax",
                                                 "bsrej", "bfar", "bnear"):
                                try: self.last[k] = int(v)
                                except ValueError: pass
                continue
            if is_trigger(line):
                self.last["trig"] = self.last.get("trig", 0) + 1
                if self.trig_sink:
                    self.trig_sink()
                continue
            pq = parse_q(line)
            if pq is None:
                # a Q line with fewer than four points is a DROPOUT, not
                # noise on the wire -- remember when we last saw one so the
                # live view can say WHY it is not updating
                if line.startswith("Q,"):
                    try:
                        if 0 <= int(line.split(",")[2]) < 4:
                            self.partial_t = time.time()
                            self.partial_n += 1
                    except (ValueError, IndexError):
                        pass
                continue
            q, gt = pq
            self.frames += 1
            self.full_t = time.time()
            self.gun_t = gt
            self.hist.append((gt, q))
            if self.sink:
                self.sink(q, gt)
        # Local snapshot: the auto-tune worker rebinds hist from its own
        # thread, and a read-then-index on the live attribute lost the race --
        # the IndexError killed the tick loop and froze the whole live panel.
        h = self.hist
        cut = h[-1][0] - 2.0 if h else 0
        self.hist = [x for x in h if x[0] >= cut][-400:]

    # ---- measurements ----------------------------------------------------
    def sigma(self):
        """blob noise with hand tremor removed; None until enough frames"""
        h = self.hist                  # snapshot: another thread may rebind it
        if len(h) < 40: return None
        a = np.array([x[1] for x in h[-120:]])
        # only use a stretch where the hand was reasonably still, or tremor
        # dominates and the number means nothing
        cen = a.mean(1)
        if max(np.ptp(cen[:, 0]), np.ptp(cen[:, 1])) > 12.0: return None
        return sigma_from_hold(a)

    def span(self):
        if not self.hist: return 0.0
        return aim_fit.quad_span(self.hist[-1][1])

    def fps(self):
        if len(self.hist) < 10: return 0.0
        dt = self.hist[-1][0] - self.hist[0][0]
        return (len(self.hist) - 1) / dt if dt > 0 else 0.0


# ---------------------------------------------------------------------------
# auto-tune: find an exposure/threshold that gives four stable blobs quietly
# ---------------------------------------------------------------------------
def auto_tune(link, log, stop):
    """Sweep aec x thr, score each point, apply the best.

    Score is deliberately NOT "lowest sigma": a very high threshold gives
    beautiful sigma on two surviving blobs, which is useless. Four blobs, seen on
    essentially every frame, comes first; sigma only breaks ties.
    """
    best = None
    aecs = [20, 30, 40, 60, 90]
    thrs = [40, 60, 80, 110, 150]
    total = len(aecs) * len(thrs)
    done = 0
    for aec in aecs:
        for thr in thrs:
            if stop.is_set(): log("auto-tune cancelled"); return None
            link.send("~cam=aec:%d,thr:%d" % (aec, thr))
            time.sleep(0.35)                       # let the sensor settle
            link.hist = []
            t0 = time.time()
            while time.time() - t0 < 0.9:
                link.pump(); time.sleep(0.02)
            done += 1
            n = len(link.hist)
            if n < 15:
                log("  aec=%-3d thr=%-3d  no frames" % (aec, thr)); continue
            sg = link.sigma()
            spans = [aim_fit.quad_span(h[1]) for h in link.hist]
            # every frame in the stream already has 4 blobs (parse_q drops the
            # rest), so "frames per second" IS the four-blob hit rate
            rate = n / 0.9
            score = (rate, -(sg if sg is not None else 9.9))
            log("  aec=%-3d thr=%-3d  %5.0f fps  span %5.1f  sigma %s"
                % (aec, thr, rate, np.mean(spans),
                   "%.3f" % sg if sg is not None else "  -  (hand moved)"))
            if best is None or score > best[0]:
                best = (score, aec, thr, sg, rate)
    if not best:
        log("auto-tune found NOTHING -- are all four LEDs in view?")
        return None
    _, aec, thr, sg, rate = best
    log("")
    log("best: aec=%d thr=%d  (%0.0f fps with four blobs, sigma %s)"
        % (aec, thr, rate, "%.3f" % sg if sg is not None else "not measured"))
    # Safety margin: nudge AWAY from the contamination edge -- a slightly
    # higher threshold and slightly shorter exposure -- and keep it only if
    # the four-blob rate holds. Headroom against background creep (sun,
    # lamps warming up) is worth more than the last hundredth of a px.
    m_aec, m_thr = max(4, int(round(aec * 0.8))), min(200, thr + 20)
    if (m_aec, m_thr) != (aec, thr) and not stop.is_set():
        link.send("~cam=aec:%d,thr:%d" % (m_aec, m_thr))
        time.sleep(0.35)
        link.hist = []
        t0 = time.time()
        while time.time() - t0 < 0.9:
            link.pump(); time.sleep(0.02)
        m_rate = len(link.hist) / 0.9
        if m_rate >= rate * 0.95:
            log("margin: aec=%d thr=%d holds %0.0f fps -- applied with "
                "headroom against background creep" % (m_aec, m_thr, m_rate))
            aec, thr = m_aec, m_thr
        else:
            log("margin: aec=%d thr=%d drops to %0.0f fps -- no free headroom, "
                "keeping the optimum" % (m_aec, m_thr, m_rate))
    link.send("~cam=aec:%d,thr:%d" % (aec, thr))
    time.sleep(0.3)
    log("applied. Press 'Save to gun' to keep it across power cycles.")
    return (aec, thr)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
def main():
    import tkinter as tk
    from tkinter import ttk, messagebox

    link = Link()
    root = tk.Tk()
    root.title("Lightgun Studio")
    root.configure(bg="#0d1117")
    # Height adapts to the screen. 800 was chosen to fit a 768p laptop and is
    # kept as the floor, but the tab panels have grown and a fixed 800 CLIPS
    # their last lines on machines with room to spare -- silently, with no
    # scrollbar and no sign that anything is missing.
    try:
        _sh = root.winfo_screenheight()
        _sw = root.winfo_screenwidth()
    except Exception:
        _sh, _sw = 800, 1020
    # The tab panels have grown enough that 800 no longer holds the tallest of
    # them. Take what the screen offers, up to 980, and keep 800 as the floor
    # for a 768p laptop -- trimming the panels instead only shrinks the tab
    # area with them, which is a race that cannot be won.
    #
    # WIDTH now adapts the same way, and for a measured reason. On a 768p
    # laptop the tab area stops growing at 313 px and the wiicam camera panel
    # already asks for 268 of them: there is no room on that tab for anything
    # taller than about 25 px, and the shape preview -- a 128x96 sensor frame
    # drawn big enough to see a 12x10 box in -- needs the better part of 230.
    # Height cannot be found. Width can: the window was pinned at 1020 on
    # screens that are 1366 or 1400 wide, so the preview goes in the 320 px
    # that were being thrown away, beside the controls rather than under them.
    # 1020 stays the floor, and on a screen too narrow to widen into, the
    # preview falls back to a tab of its own (see PREVIEW_W below).
    _ww = max(1020, min(1340, _sw - 30))
    root.geometry("%dx%d" % (_ww, max(800, min(980, _sh - 60))))
    # What the widening actually bought, which is what the preview may spend.
    # Measured from the layout that already fitted at 1020 rather than from a
    # fraction of the window, because every pixel under 1020 belongs to a row
    # of the camera panel that is already 562 px wide and cannot give any back.
    #
    # Capped as well as floored, and the cap is a HEIGHT measurement wearing a
    # width's clothes: the canvas keeps the sensor's 4:3 so a round blob draws
    # round, which makes 288 px wide 216 px tall -- and 216 plus the heading
    # and the four-line verdict under it is 299 of the 313 px the tab area
    # allows. Wider would draw a bigger picture and cut the verdict off the
    # bottom of it, and the verdict is the part that says whether any of the
    # numbers on this tab mean anything.
    PREVIEW_W = min(288, _ww - 1020 - 14)
    # Below this the box for a real LED -- 12 wide by 10 tall at sensitivity 2
    # -- draws smaller than a spinbox arrow, and a preview nobody can read is
    # worse than a tab they have to click.
    PREVIEW_MIN = 180

    C_BG, C_FG, C_DIM, C_OK, C_WARN, C_BAD = "#0d1117", "#e6edf3", "#7d8590", "#39c26e", "#d8a13a", "#d24b4b"
    F = ("Segoe UI" if os.name == "nt" else "DejaVu Sans", 10)
    FB = (F[0], 11, "bold")
    FH = (F[0], 15, "bold")

    def lab(parent, text, font=F, fg=C_FG, **kw):
        return tk.Label(parent, text=text, font=font, fg=fg, bg=C_BG, **kw)

    # ---- header ---------------------------------------------------------
    head = tk.Frame(root, bg=C_BG); head.pack(fill="x", padx=16, pady=(14, 6))
    lab(head, "Lightgun Studio", FH).pack(side="left")
    st_conn = lab(head, "not connected", F, C_BAD); st_conn.pack(side="right")
    # The pointer toggle lives in the header because it is a MODE, not an
    # action, and the window has to say which mode it is in -- a gun that has
    # silently stopped moving the cursor is indistinguishable from a broken gun.
    st_hid = lab(head, "", F, C_OK); st_hid.pack(side="right", padx=(0, 18))
    # How many distances to calibrate from. Two is measured to be as good as
    # three (65.7 vs 63.6 px at blob sigma 0.3, identical at 0.6) and saves 20
    # trigger pulls, and not every room lets you take three steps back. What is
    # NOT optional is more than one: at a single distance the boresight and the
    # screen mapping are exactly degenerate and the fit refuses.
    stance_n = tk.IntVar(value=3)

    body = tk.Frame(root, bg=C_BG); body.pack(fill="both", expand=True, padx=16, pady=8)

    # ---- left: the four steps -------------------------------------------
    left = tk.Frame(body, bg=C_BG); left.pack(side="left", fill="y", padx=(0, 18))
    step_rows = {}
    for n, (num, title, sub) in enumerate([
        (1, "Buttons & pins", "opens the OpenFIRE app"),
        (2, "Camera tuning",  "exposure, threshold, noise floor"),
        (3, "Lens / FOV",     "only if your lens is not the stock 66\u00b0"),
        (4, "Aim calibration","five dots x three distances"),
        (5, "Fine tune",      "iron sights to cursor, lead and smoothing"),
        (6, "Verify",         "measures whose error it is"),
        (7, "Recoil feel",    "solenoid strike, hold and after-pulses")]):
        f = tk.Frame(left, bg=C_BG); f.pack(fill="x", pady=5)
        b = tk.Button(f, text="%d.  %s" % (num, title), font=FB, width=22, anchor="w",
                      bg="#161b22", fg=C_FG, activebackground="#21262d",
                      relief="flat", padx=10, pady=8)
        b.pack(fill="x")
        s = lab(f, "   " + sub, (F[0], 9), C_DIM, anchor="w"); s.pack(fill="x")
        step_rows[num] = (b, s)
        if num == 4:
            # Distance count, next to the step it applies to. Two positions is
            # measured to be as good as three and saves 20 trigger pulls; some
            # rooms simply do not allow three.
            rowd = tk.Frame(left, bg=C_BG); rowd.pack(fill="x", pady=(2, 0))
            lab(rowd, "   distances:", (F[0], 9), C_DIM).pack(side="left")
            for nval in (2, 3):
                tk.Radiobutton(rowd, text=str(nval), value=nval, variable=stance_n,
                               font=(F[0], 9), bg=C_BG, fg=C_FG, selectcolor="#161b22",
                               activebackground=C_BG, activeforeground=C_FG,
                               highlightthickness=0, bd=0,
                               command=lambda: step_rows[4][1].config(
                                   text="   five dots x %d distances" % stance_n.get())
                               ).pack(side="left")
            lab(rowd, "(2 is nearly as good and 20 fewer pulls)",
                (F[0], 8), C_DIM).pack(side="left", padx=(6, 0))

    # ---- right: the live panel ------------------------------------------
    right = tk.Frame(body, bg=C_BG); right.pack(side="left", fill="both", expand=True)
    toprow = tk.Frame(right, bg=C_BG); toprow.pack(fill="x")
    cv = tk.Canvas(toprow, width=380, height=280, bg="#010409", highlightthickness=1,
                   highlightbackground="#30363d")
    cv.pack(side="left", anchor="n")
    stats = tk.Frame(toprow, bg=C_BG); stats.pack(side="left", fill="both",
                                                  expand=True, padx=(14, 0))
    stat_vals = {}
    for k in ("frames", "rate", "quad span", "blob noise", "verdict"):
        r = tk.Frame(stats, bg=C_BG); r.pack(fill="x", pady=3)
        lab(r, k, F, C_DIM, width=11, anchor="w").pack(side="left")
        v = lab(r, "-", FB, C_FG, anchor="w", wraplength=230, justify="left")
        v.pack(side="left")
        stat_vals[k] = v

    # height 6, not 8: this box is packed with expand=True, so it absorbs every
    # spare pixel of window height -- including the ones the tab panels need.
    # The log scrolls; the controls above it do not.
    logbox = tk.Text(right, height=6, bg="#010409", fg=C_DIM, font=("Consolas" if os.name=="nt" else "monospace", 9),
                     relief="flat", highlightthickness=1, highlightbackground="#30363d")
    logbox.pack(fill="both", expand=True, pady=(10, 0), side="bottom")
    def log(msg):
        logbox.insert("end", msg + "\n"); logbox.see("end")
    PY_LOG = log

    # ---- step actions ---------------------------------------------------
    def find_openfire_app():
        """Look where it actually tends to live before asking."""
        roots = [os.path.join(HERE, "..", ".."), os.path.expanduser("~")]
        for r in roots:
            for dirpath, dirnames, files in os.walk(os.path.abspath(r)):
                if dirpath.count(os.sep) - os.path.abspath(r).count(os.sep) > 4:
                    dirnames[:] = []
                    continue
                for f in files:
                    if f.lower() == "openfireapp.exe":
                        return os.path.join(dirpath, f)
        return None

    def step1():
        exe = find_openfire_app()
        if not exe:
            log("Could not find OpenFIREapp.exe automatically.")
            from tkinter import filedialog
            exe = filedialog.askopenfilename(title="Find OpenFIREapp.exe",
                                             filetypes=[("OpenFIRE app", "*.exe")])
            if not exe: return
        # Their app needs the port to itself, so hand it over and take it back.
        log("Releasing the port and starting the OpenFIRE app...")
        # Once the port is gone we cannot send ~aimhid any more, so a frozen
        # pointer would be stuck until a replug. Release it while we still can.
        link.pointer(True, remember=False)
        link.close()
        st_conn.config(text="OpenFIRE app has the port -- close it to come back",
                       fg=C_WARN)
        try:
            proc = subprocess.Popen([exe], cwd=os.path.dirname(exe))
        except Exception as e:
            log("could not start it: %s" % e); reconnect(); return
        log("Set your pins and buttons there, then just CLOSE it.")

        # Steps 3-5 come back on their own because they subprocess.call() a tool
        # we wrote. This one launches somebody else's app, so it used to end at
        # Popen and leave the status stuck on 'handed off' until the user found
        # the Reconnect button -- which reads as a hang, not a prompt. Wait for
        # it here instead. Their app may also hand off to a second process and
        # exit at once, so the port can still be held after wait() returns:
        # probe it directly (never through `link`, which the tick loop owns)
        # until it comes free.
        def take_the_port_back():
            why = take_port_back(proc, link.port or "")
            def done():
                if why == "timeout":
                    log("The port is still held after %ds -- something else has it."
                        % int(APP_PORT_WAIT_S))
                else:
                    log("OpenFIRE app closed; taking the port back...")
                reconnect()
            root.after(0, done)
        threading.Thread(target=take_the_port_back, daemon=True).start()
        log("Two settings that matter for us, in the profile:")
        log("  RunMode = Normal        (Average stacks extra smoothing on ours)")
        log("  serialARcorrection off  (it would re-correct an already-correct aim)")

    def step2():
        nb.select(tab_cam)

    def step3():
        nb.select(tab_lens)

    def step4():
        sg = link.sigma()
        s_good, s_ok = sigma_gates(link.last.get("board"))
        if sg is not None and sg > s_ok:
            from tkinter import messagebox
            if not messagebox.askyesno("Noise floor is high",
                    "Blob noise is %.2f px.\n\n"
                    "Aim error scales with this: 0.2 px gives about 16 px of error, "
                    "0.8 px gives about 46 px, and calibrating now bakes that in.\n\n"
                    "Tune the camera first?  (Yes = go to tuning, No = calibrate anyway)"
                    % sg):
                pass
            else:
                nb.select(tab_cam); return
        # On the wiicam, the calibration IS the LED measurement. Every frame
        # where the resolver confirms four corners is a frame that says what
        # this rig's LEDs look like, across every stance the user shoots from
        # -- free data, and until now discarded unless the user knew to press
        # 'Learn LED shape' first. pical arms this itself; Studio hands the
        # port to a separate process for calibration, so it is armed HERE,
        # before the handoff, and left running: the gun does not reboot when
        # the port changes hands, and aim_calib touches neither the format nor
        # the capture. Full detail first, because the box features the gate
        # is derived from need it. Arming clears (off->on edge), which is
        # right: a calibration is a fresh measurement.
        b = link.last.get("board") or ""
        if "wiicam" in b:
            link.send("~cam=fmt:2")
            link.send("~camlearn=on:1")
            learn_state["on"] = True
            log("shape capture armed for the calibration: every confirmed "
                "corner it sees is a measurement of your LEDs at every "
                "distance. When you are back: on the Camera tab, with only "
                "the bar in view, pan slowly so your lamp or window comes "
                "into the picture, then press 'Measure the gate'.")
        log("Handing the port to the calibration window...")
        link.pointer(True, remember=False)   # calibration reads the trigger as a click
        link.close()
        st_conn.config(text="calibrating...", fg=C_WARN)
        def run():
            try:
                subprocess.call([sys.executable, os.path.join(HERE, "aim_calib.py"),
                                 "--port", link.port or "",
                                 "--stances", str(stance_n.get())])
            except Exception as e:
                log("calibration failed to start: %s" % e)
            root.after(0, reconnect)
        threading.Thread(target=run, daemon=True).start()

    def _handoff(tool, label):
        """Every child window drives the cursor with the gun, so the pointer has
           to be live -- and it has to come back to whatever the user chose."""
        log("Handing the port to the %s window..." % label)
        link.pointer(True, remember=False)
        link.close()
        st_conn.config(text="%s..." % label, fg=C_WARN)
        def run():
            try:
                subprocess.call([sys.executable, os.path.join(HERE, tool),
                                 "--port", link.port or ""])
            except Exception as e:
                log("%s failed to start: %s" % (label, e))
            root.after(0, reconnect)
        threading.Thread(target=run, daemon=True).start()

    def step5(): _handoff("aim_finetune.py", "fine tune")
    def step6(): _handoff("aim_verify.py", "verify")

    step_rows[1][0].config(command=step1)
    step_rows[2][0].config(command=step2)
    step_rows[3][0].config(command=step3)
    step_rows[4][0].config(command=step4)
    step_rows[5][0].config(command=step5)
    step_rows[6][0].config(command=step6)

    # ---- tabs: camera tuning lives here ---------------------------------
    nb = ttk.Notebook(right)
    tab_cam = tk.Frame(nb, bg=C_BG)
    nb.add(tab_cam, text="  Camera  ")
    # 6 px above the tab strip, not 12: the notebook is packed WITHOUT expand,
    # so the tab area is only ever as tall as the window has left over -- 307
    # px on a 768p laptop -- and every pixel spent outside it is one the
    # camera panel cannot have.
    nb.pack(fill="x", pady=(6, 0))

    # ---- the shape preview -------------------------------------------------
    # What the gun actually SEES, drawn in the sensor's own 128x96 array. Until
    # this existed the panel could only print coordinates, and coordinates
    # cannot answer the question the whole shape gate rests on -- is the thing
    # in slot 2 a point of light or a window?
    #
    # It goes BESIDE the controls, not under them. Measured on a 768p laptop:
    # the tab area stops growing at 313 px and the wiicam panel already asked
    # for 268 of them, so 25 px was everything on offer in the height -- and a
    # canvas tall enough to see a 12x10 box in needs the better part of 220.
    # Height cannot be bought at any price. Width could: the window was pinned
    # at 1020 on a screen 1366 or 1400 wide, so widening it to 1340 buys 320
    # px, of which the preview column takes 290 and the camera panel keeps the
    # rest. On a screen too narrow to widen into, the preview gets a tab of
    # its own: worse, because the gate spinboxes are then not on screen beside
    # it, but still better than not drawing it.
    if PREVIEW_W >= PREVIEW_MIN:
        prev_host = tk.Frame(tab_cam, bg=C_BG)
        prev_host.pack(side="right", fill="y", padx=(12, 0))
        prev_page = tab_cam
    else:
        prev_host = tk.Frame(nb, bg=C_BG)
        nb.add(prev_host, text="  Shape  ")
        prev_page = prev_host
        PREVIEW_W = 288
    PREVIEW_H = int(PREVIEW_W * NATIVE_H / NATIVE_W)
    # wraplength, not hope. Without it this heading is 303 px of natural width
    # and the COLUMN grows to hold it -- which on a 1280 px screen takes 87 px
    # straight off the camera panel beside it and pushes its widest row off
    # the edge. A label may never be what decides how wide this column is.
    lab(prev_host, "%d x %d sensor pixels, shaded by density"
        % (NATIVE_W, NATIVE_H), (F[0], 8), C_DIM, anchor="w",
        justify="left", wraplength=PREVIEW_W).pack(fill="x")
    shape_cv = tk.Canvas(prev_host, width=PREVIEW_W, height=PREVIEW_H,
                         bg="#010409", highlightthickness=1,
                         highlightbackground="#30363d")
    shape_cv.pack()
    # A FIXED number of lines, so the canvas above cannot jump up and down as
    # the verdict changes length -- a picture that moves while you are reading
    # a number off it is worse than a smaller picture. The count is measured
    # rather than typed: the longest message below is about this many pixels
    # of text, and a narrow column simply gets more lines of it. Too few and
    # the sentence that says the units are wrong is the one cut off.
    SHAPE_MSG_PX = 900
    shape_lbl = lab(prev_host, "", (F[0], 8), C_DIM, justify="left",
                    anchor="nw", wraplength=PREVIEW_W,
                    height=max(3, -(-SHAPE_MSG_PX // PREVIEW_W)))
    shape_lbl.pack(fill="x", pady=(2, 0))

    # Density as a fill colour: dark when a blob rattles around inside its own
    # bounding box, hot when it fills it. A point source fills nearly all of
    # its box; a reflection off a window frame does not, and that difference is
    # invisible in a list of numbers and obvious in a picture.
    SHAPE_RAMP = ((0.0, (22, 42, 66)), (0.55, (31, 111, 235)),
                  (1.0, (216, 161, 58)))

    def heat(d):
        """A density in 0..1 as a hex colour. Clamped, because px can exceed
        the box area -- and when it does, that is itself the units being wrong
        rather than a blob to shade differently."""
        d = 0.0 if d is None else max(0.0, min(1.0, d))
        for (a, ca), (b, cb) in zip(SHAPE_RAMP, SHAPE_RAMP[1:]):
            if d <= b or b >= 1.0:
                t = 0.0 if b <= a else (d - a) / (b - a)
                return "#%02x%02x%02x" % tuple(
                    int(round(ca[i] + (cb[i] - ca[i]) * t)) for i in range(3))
        return "#000000"

    def draw_shapes():
        """Draw the last '~camblob?' reply as shapes.

        The crosshair is a MEASUREMENT, not decoration. We are not certain the
        box fields are in the sensor's 128x96 array -- they are 7-bit numbers
        that could be in some other space entirely -- and the only way to
        settle it from outside the gun is to map the reported position into the
        same array and see whether the box lands on it. Agreeing is quiet;
        disagreeing is drawn as a red line between the two and said in words
        underneath, because a mismatch means every conclusion drawn from the
        box so far is in unknown units."""
        shape_cv.delete("all")
        W, H = PREVIEW_W, PREVIEW_H
        sx, sy = W / NATIVE_W, H / NATIVE_H
        # The array's own edge. Without it a blob has nothing to be large or
        # small against, and "how much of the frame is this thing" is the first
        # question asked of a suspected window.
        shape_cv.create_rectangle(1, 1, W, H, outline="#30363d")
        # The OV2640 build finds its blobs in software and answers no
        # '~camblob?' at all. Left saying "waiting", this canvas would accuse
        # a perfectly healthy ESP32 gun of not answering, for ever.
        board = link.last.get("board") or ""
        if board and "wiicam" not in board:
            shape_cv.create_text(W / 2, H / 2, text="not this board",
                                 fill=C_DIM, font=(F[0], 9))
            shape_lbl.config(text="%s finds its blobs in software and reports "
                                  "no shapes -- this preview is for the "
                                  "wiicam sensor" % board, fg=C_DIM)
            return
        raw = getattr(link, "blobs", "")
        blobs = parse_blobs(raw)
        bh_gate = link.last.get("bhmax") or 0
        kept = drop = 0
        worst = None
        assumed = 0
        boxed = 0
        for b in blobs:
            s = blob_shape(b)
            if s is None:
                continue
            if s["kept"]:
                kept += 1
            else:
                drop += 1
            col = C_OK if s["kept"] else C_BAD
            cx = 1 + s["cross"][0] * sx
            cy = 1 + s["cross"][1] * sy
            if s["box"]:
                boxed += 1
                x0, y0, bw, bh = s["box"]
                # +1 because the wire's width is xmx-xmn: a single-pixel blob
                # reports 0x0, and a rectangle drawn with no extent is a blob
                # that vanishes at exactly the size the sensor sees most often.
                X0, Y0 = 1 + x0 * sx, 1 + y0 * sy
                X1, Y1 = 1 + (x0 + bw + 1) * sx, 1 + (y0 + bh + 1) * sy
                # Kept and dropped differ in TWO ways, not one: a red outline
                # and a hatched fill. Colour alone is the whole verdict on a
                # picture whose other channel is already a colour ramp, and
                # "which of these did the gate throw away" is the question the
                # preview is being looked at to answer.
                box_kw = {"fill": heat(s["density"]), "outline": col,
                          "width": 2}
                if not s["kept"]:
                    box_kw["stipple"] = "gray50"
                if not s["origin"]:
                    # No origin on the wire, so this box is parked on the
                    # crosshair. Dashed, and said in the line below: a box
                    # drawn at the position agrees with the position by
                    # construction, and would otherwise read as the units
                    # having been confirmed when nothing was measured.
                    assumed += 1
                    box_kw["width"] = 1
                    box_kw["dash"] = (3, 2)
                shape_cv.create_rectangle(X0, Y0, X1, Y1, **box_kw)
                # Height, because height is what the gate that works judges.
                # Red when this blob is over the limit the gun is holding, so
                # the picture says WHICH blobs the setting is about to cost.
                shape_cv.create_text(
                    X0, Y0 - 1, text="h%d" % int(bh), anchor="sw",
                    font=(F[0], 7),
                    fill=C_BAD if (bh_gate and bh > bh_gate) else C_DIM)
                if s["off"] is not None and (worst is None or s["off"] > worst):
                    worst = s["off"]
                if s["off"] is not None and s["off"] > SHAPE_OFF_MAX:
                    shape_cv.create_line(1 + (x0 + bw / 2.0) * sx,
                                         1 + (y0 + bh / 2.0) * sy, cx, cy,
                                         fill=C_BAD, width=2)
            # Drawn last so it is never hidden under a box fill.
            shape_cv.create_line(cx - 5, cy, cx + 5, cy, fill="#ffd24a")
            shape_cv.create_line(cx, cy - 5, cx, cy + 5, fill="#ffd24a")
        if not blobs:
            # An empty reply and no reply at all are different faults, and the
            # empty frame looks identical either way. Saying "the sensor sees
            # nothing" before the gun has answered once sends the user hunting
            # for an LED problem that is really a connection problem.
            waiting = not raw
            shape_cv.create_text(W / 2, H / 2,
                                 text="waiting" if waiting else "no blobs",
                                 fill=C_DIM if waiting else C_BAD,
                                 font=(F[0], 9))
            shape_lbl.config(
                text=("waiting for the gun's first blob report"
                      if waiting else
                      "the sensor is reporting nothing at all -- no LEDs in "
                      "view, or the camera is not answering"),
                fg=C_DIM if waiting else C_BAD)
            return
        txt = "%d kept, %d dropped." % (kept, drop)
        col = C_DIM
        if not boxed:
            txt += ("  Positions only -- set blob detail to 'full detail' and "
                    "the boxes appear.")
        elif assumed:
            txt += ("  No box ORIGIN from this gun, so every box is drawn AT "
                    "its crosshair: the shape is real, the placement is not.")
            col = C_WARN
        elif worst is not None and worst > SHAPE_OFF_MAX:
            txt += ("  MISMATCH: box centre %.1f px off the position, so the "
                    "box fields are NOT in this array and the shape numbers "
                    "are in unknown units." % worst)
            col = C_BAD
        elif worst is not None:
            txt += ("  Box centre sits %.1f px from the position, so the box "
                    "fields ARE in this 128x96 array." % worst)
            col = C_OK
        shape_lbl.config(text=txt, fg=col)

    # ESP32 (OV2640) tuning widgets live in one frame, wiicam's in another;
    # the gun's own ~ping board tag decides which is shown. Both take the
    # LEFT of the tab and whatever width the preview beside them leaves.
    frame_esp = tk.Frame(tab_cam, bg=C_BG)
    frame_esp.pack(side="left", fill="both", expand=True)
    frame_wii = tk.Frame(tab_cam, bg=C_BG)   # packed on detect

    # Every line this panel spends is a line the Save row at the bottom does
    # not get: the window is 800 px tall on a 768p laptop and the tab area
    # under it is 313 px, no matter how tall the panel asks to be. It went
    # over once already -- "Save to gun" rendered four pixels high and could
    # not be clicked -- so headings ride ON the row they name rather than
    # above it, and the paddings here are deliberately mean.
    #
    # WHAT IS ON THIS PANEL is now a separate question from how much fits on
    # it. Everything above the Advanced button is something a user touches
    # while a test is running; everything below it is something set once, or
    # not at all. That is not tidying: the two rows that used to sit between
    # 'blob detail' and the live readout carried seven spinboxes and a pair of
    # radios, two of them superseded by the height gate and two of them raw
    # sensor registers, and hunting for the sensitivity buttons through that
    # mid-test is how a session gets abandoned. Measured, the split also buys
    # back the height it needed for the new gate: front alone asks for 179 px
    # of the 313, front plus Advanced 277, and the render test's 20 px margin
    # holds in both.
    #
    # THE ROW GAPS ARE GONE, and they paid for the fit row. The gate the panel
    # now steers people to is one this gun measures for itself, which needs two
    # buttons and a line to answer on -- 21 px -- and the panel had 13 over the
    # 313. Every pady between these rows is 0 or 1 as a result, and the button
    # rows lost a pixel of their own. Measured with the Advanced disclosure
    # open and both readout lines wrapped: 286 px, so the render test's 20 px
    # margin holds with seven to spare. Nothing on the panel was removed to
    # find it -- everything on the front is something a test needs in its hand.
    roww = tk.Frame(frame_wii, bg=C_BG); roww.pack(fill="x")
    lab(roww, "wiicam sensitivity", (F[0], 9), C_DIM).pack(side="left",
                                                           padx=(0, 8))
    for sv, nm in ((0, "Default"), (1, "High"), (2, "Max")):
        tk.Button(roww, text=nm, font=F, bg="#161b22", fg=C_FG, relief="flat",
                  padx=10, pady=1,
                  command=lambda v=sv: (link.send("~cam=sens:%d" % v),
                                        log("sensitivity -> %d" % v))
                  ).pack(side="left", padx=(0, 6))
    # What used to have a line of its own, riding on this row instead: a line
    # here costs 20 px of a panel that has 313, and measured, five words fit
    # in the 254 px the three buttons leave. "Max", because it is: a fresh
    # flash now boots there, and a gun flashed over an older profile keeps
    # whatever it held, which is why the hint says it rather than assuming
    # it. At Default and High the blob reports hit a hard 4-pixel floor and
    # an LED vanishes at about 1.8x the distance it was full brightness at;
    # Max opens the sensor's own size limit and that floor share fell from a
    # quarter of all reports to under one percent. The rest of what that line said
    # -- that the setting lives in the OpenFIRE profile, and that the noise
    # floor above still measures on this board -- goes to the log when the
    # board is detected, which is where this panel already sends the
    # reasoning that will not fit beside a control.
    lab(roww, "Max is the one to use", (F[0], 8), C_DIM).pack(side="left")

    # ---- what the sensor reports, and the gate that judges it --------------
    # The wiicam finds blobs in HARDWARE and reports four slots. A bright
    # window does not add a fifth point: it TAKES one, and an LED goes missing,
    # which is why "too much light" shows up as a quad that keeps breaking.
    # Nothing in software can recover a point the sensor never sent. What CAN
    # be done is refuse the impostor, so the resolver rebuilds the missing
    # corner from the three real ones instead of trusting four points one of
    # which is a lie.
    rowb = tk.Frame(frame_wii, bg=C_BG); rowb.pack(fill="x")
    # Three report formats, not a switch: each one costs a longer read per
    # frame, so the question is how much detail is worth the bus time and not
    # whether sizes are on. Named for what the user gets rather than for the
    # sensor's words -- "extended" and "full" are the datasheet's names for
    # two read lengths, and neither of them says which one anybody wants.
    cam_fmt = tk.IntVar(value=0)

    def send_fmt():
        n = cam_fmt.get()
        # 'ext' for the two formats that predate full mode, 'fmt' only for
        # full. Both firmware generations accept ext:0 and ext:1; the older
        # one has never heard of 'fmt' and drops the whole key without a
        # word, so a gun on it would sit at detail 0 for ever while the
        # control insisted otherwise -- and every gate row under it is dead
        # until sizes arrive. fmt:2 is the one thing only the new firmware
        # can do, so it is the one thing worth asking for by that name.
        link.send("~cam=%s:%d" % ("fmt" if n == 2 else "ext", n))
        link.send("~camblob?")
        if n == 0:
            log("blob detail off -- the sensor reports position only, so the "
                "shape preview has nothing but crosshairs to draw and every "
                "gate has nothing to judge")
            return
        # The full reasoning goes in the log rather than on the panel: it
        # is read once, and the panel has to stay short enough to fit.
        log("blob sizes ON (Save to gun keeps them; without a save they are "
            "back off on the next power-cycle). Aim at the screen and read "
            "the sizes, then swing past the window and read them again. If "
            "the two are DIFFERENT, set the size window to keep the LEDs and "
            "drop the rest. If they are the SAME, no setting here can "
            "separate them -- a curtain, an angle change or moving the LED "
            "bar is the only real fix.")
        if n == 2:
            log("full detail also reports each blob's bounding box, its pixel "
                "count and where the box sits. That is what the preview "
                "beside this panel draws, and what the height gate needs: in "
                "any other format the gun is sent no box and the gate stands "
                "down. The box is in the sensor's own 128x96 pixels while the "
                "positions beside it are already normalised into the gun's "
                "240x176 pipeline space, so if the crosshair in the preview "
                "does not land inside its box, the mode register under "
                "Advanced is the thing to try.")
            log("'Save to gun' keeps this format now, and that is what "
                "makes a saved height gate work: the gate only judges in full "
                "detail, so a gun that came back in any other format would "
                "load the ceiling and never apply it. Save the format and the "
                "gate together.")
            # Said HERE, because this is the only moment the shape controls
            # stop being inert. What this used to say was the measurement
            # behind a recommended height -- 11,996 blobs off ONE bar with two
            # LEDs per corner. A bar with five LEDs per cluster makes blobs
            # several times taller, so that ceiling would have blinded it with
            # nothing on screen to say why, and the figure is gone. The gun
            # measures its own instead.
            log("'Measure the gate' asks the gun what IT has seen: how tall "
                "its own LEDs come out and how tall the stray light does. "
                "Press 'Learn LED shape', point at the bar for a few seconds "
                "with no bright light in view, then pan slowly so a lamp or a "
                "window comes into the picture beside it, then measure. "
                "Right after a calibration is even better -- the gun has "
                "then seen your LEDs from every distance -- but not required. "
                "Nothing here suggests a height: a number measured on "
                "somebody else's LED bar can blind yours.")
    lab(rowb, "blob detail", (F[0], 9), C_DIM).pack(side="left")
    for fv, nm in ((0, "off"), (1, "sizes"), (2, "full detail")):
        tk.Radiobutton(rowb, text=nm, value=fv, variable=cam_fmt,
                       command=send_fmt, font=(F[0], 9), bg=C_BG, fg=C_FG,
                       selectcolor="#161b22", activebackground=C_BG,
                       activeforeground=C_FG, highlightthickness=0, bd=0
                       ).pack(side="left")
    bgate = {}
    bspin = {}

    def gate_send(k, v):
        """Send whatever is in the box, typed or stepped. A value the user
        typed reached nothing before this: only the arrows had a command."""
        try:
            n = int(v.get())
        except Exception:
            return                      # mid-edit; the next event will do it
        link.send("~cam=%s:%d" % (k, n))
        link.send("~camblob?")

    # The firmware refuses a bhmax or a pxmax BELOW WHAT THIS GUN HAS MEASURED
    # ITS OWN LEDs AT, and the refusal names that figure. The floor is the
    # rig's, not a constant: a bar with five LEDs per cluster makes blobs
    # several times larger than one with two, so a fixed floor either refuses
    # the only workable setting for the first or accepts one that blinds the
    # second. Both floors are backed by a measurement that SURVIVES a power
    # cycle -- the gun stores the tallest LED and the largest LED pixel count
    # it has measured and takes the greater of stored and live -- so a refusal
    # can arrive on a gun that has captured nothing this session, naming a
    # figure from a capture taken weeks ago. With no capture at all, live or
    # stored, there is nothing to defend and any value is accepted, which is
    # exactly when a stepped 1, 2, 3 does the most damage: bhmax:1 is a gun
    # that sees nothing and says nothing about it. armax is the one fixed
    # floor left -- refused below 16 as before -- and it is deprecated.
    #
    # So all three step through a fixed list rather than a range: the arrows
    # can only ever land on a value somebody might mean, and the one number
    # worth having comes from 'Measure the gate' and not from this ladder.
    #
    # Shown in the units the user thinks in, not the wire's: armax is eighths
    # of a ratio, and "20" on a panel means nothing at all next to "2.5:1".
    # bhmax is in sensor ROWS to keep it apart from pxmax, which is a count of
    # pixels -- two different things that would otherwise both read "12 px".
    BHMAX_STEPS = (("off", 0), ("8 rows", 8), ("10 rows", 10),
                   ("12 rows", 12), ("16 rows", 16), ("24 rows", 24))
    PXMAX_STEPS = (("off", 0), ("12 px", 12), ("13 px", 13), ("14 px", 14),
                   ("16 px", 16), ("20 px", 20), ("24 px", 24))
    ARMAX_STEPS = (("off", 0), ("2:1", 16), ("2.5:1", 20), ("3:1", 24),
                   ("4:1", 32))
    shape_var = {}          # key -> (StringVar, {shown: wire}, {wire: shown})

    def shape_send(k):
        """Send the value the spinbox is showing, translated back to the wire.

        Nothing here may raise: this runs from a Tk callback, and an exception
        in one leaves the control dead with only a traceback on a stderr the
        user does not have."""
        var, to_wire, _ = shape_var[k]
        try:
            n = to_wire[var.get()]
        except Exception:
            return                      # not one of the rungs; ignore it
        link.send("~cam=%s:%d" % (k, n))
        link.send("~camblob?")
        # There is no room beside these for a hint saying so, and without one
        # a gate set outside full mode does nothing at all with nothing to say
        # why -- the same trap the shape capture has, answered the same way.
        # Only when the gate is being turned ON: saying it while somebody
        # switches one off is noise.
        if n and cam_fmt.get() != 2:
            log("the shape gate only acts in 'full detail'. In the other "
                "formats the gun is sent no bounding box and no pixel count, "
                "so there is nothing to judge and the gate stands down -- set "
                "blob detail to 'full detail' above, or this setting does "
                "nothing.")
        # Said only when it is switched ON, and said at length because the
        # panel has room for two words. At sensitivity 2 the sensor smears
        # horizontally -- the same LEDs went from a 2x2 box to 12x3 when the
        # gain went up, width x5.5 against height x1.5 -- so a roundness limit
        # measures the gain rather than the source, and the LEDs are what it
        # starts refusing. 'biggest blob height' is the same idea on the one
        # axis the smear leaves alone.
        if n and k == "armax":
            log("roundness (armax) is DEPRECATED and NOT recommended. It was "
                "measured at sensitivity 1, where an LED came out round; at "
                "sensitivity 2 -- the default now -- the sensor smears a 2x2 "
                "blob out to 12x3, so the ratio it measures is the gain and "
                "not the LED, and it is the LEDs that get refused. The gun "
                "still loads and applies whatever is stored, so a setting "
                "already in a gun keeps its meaning, but nothing suggests a "
                "value for it and 'Measure the gate' never sets it. Use "
                "'biggest blob height': height is the axis the smear does not "
                "touch.")

    def shape_box(parent, key, name, steps, note=None, width=5):
        """One rung-stepping gate control, label and all.

        The width is in characters and it is not cosmetic: every one of these
        rows is within 30 px of the panel's edge, and a box two characters
        wider than its longest rung is two characters of a row that then runs
        off the side with nothing to say so."""
        lab(parent, name, (F[0], 9), C_DIM).pack(side="left", padx=(12, 2))
        sv = tk.StringVar(value=steps[0][0])
        shape_var[key] = (sv, {s: w for s, w in steps},
                          {w: s for s, w in steps})
        sp = tk.Spinbox(parent, values=tuple(s for s, _ in steps), width=width,
                        textvariable=sv, font=F, bg="#161b22", fg=C_FG,
                        relief="flat", state="readonly",
                        readonlybackground="#161b22",
                        command=lambda k=key: shape_send(k))
        sp.pack(side="left", padx=(2, 0))
        bspin[key] = sp
        if note:
            lab(parent, note, (F[0], 8), C_DIM).pack(side="left", padx=(4, 0))
        return sp

    # THE gate, on the front of the panel, beside the format it needs. Height
    # rather than width, area or pixel count because at sensitivity 2 the
    # sensor smears horizontally -- the same LEDs went from a 2x2 box to 12x3
    # when the gain went up -- so width stops measuring the source and starts
    # measuring the gain. Height is the axis the smear does not touch.
    #
    # No note beside it. The one that used to be here said "10 recommended",
    # and 10 was measured on a bar with two LEDs per corner: on a bar with
    # five per cluster the LEDs themselves are taller than that, so the hint
    # was an instruction to blind the gun. The row under this one carries what
    # the gun measured on ITS OWN bar instead.
    shape_box(rowb, "bhmax", "biggest blob height", BHMAX_STEPS, width=7)

    # ---- the only defensible height: the one this gun measured -------------
    # '~camfit' reads the two distributions the shape capture has been filling
    # -- how tall this rig's own LEDs come out, and how tall the stray light
    # in this room does -- and either names a ceiling between them or says
    # there is no gap to put one in. Asking and APPLYING are two separate
    # presses on purpose: '?' changes nothing, '=apply' writes the gate and
    # saves it, and a single button that did both would set a gate off a
    # half-finished capture the first time somebody pressed it to see what it
    # said.
    #
    # One row, and the label on it is the HEIGHT GATE'S line: when the gate is
    # misbehaving that is what it says, and the fit comes back underneath it
    # once the gate is quiet again. The alternative was a warning on the
    # readout line below, which is already two wrapped lines deep and would
    # have taken a third -- 14 px of a tab area that stops growing at 313.
    #
    # Asked for by hand and never polled. The gun's answer is three sentences
    # of English and they go to the log like every other reply; asked every
    # few seconds while a capture ran, they would push the diag verdict and
    # the save result out of a six-line box before anybody read them.
    rowfit = tk.Frame(frame_wii, bg=C_BG); rowfit.pack(fill="x")
    # seq is what tells an answer from silence: an older gun does not have
    # '~camfit' and says nothing at all, which is indistinguishable from a
    # reply still in flight until the clock runs out.
    fit_state = {"seq": -1, "asked": 0.0}

    def fit_ask(apply=False):
        if not link.src:
            log("not connected -- press Reconnect first")
            return
        fit_state["seq"] = link.fit.seq
        fit_state["asked"] = time.monotonic()
        link.send("~camfit=apply" if apply else "~camfit?")
        if apply:
            log("~camfit=apply sent -- the gun measures again and writes what "
                "it finds NOW, so what lands is never the line this panel was "
                "showing a minute ago. Nothing here chose the number.")
            # Nothing to add about saving. The apply writes the gate AND the
            # full mode it needs, switching the gun into it if it had dropped
            # out, so there is no second press to remember. The sentence that
            # stood here told the user to press 'Save to gun' as well, which
            # was a workaround for a command called 'apply' that did not
            # survive a reboot -- the gun fixed that, and a front end still
            # asking for the extra press teaches a habit that is now wrong.
            return
        # The two conditions that make the answer 'not enough data' are both
        # things the user can fix in one click each, and both are on this
        # panel -- so they are said at the moment the question is asked rather
        # than after the gun has spent a sentence saying no.
        if cam_fmt.get() != 2:
            log("the fit needs 'full detail': the box height it measures is "
                "only reported in that format, so in any other one the gun "
                "has nothing to have measured and the counts stay at zero.")
        if not learn_state["on"]:
            log("the fit only counts blobs while the shape capture is running "
                "-- press 'Learn LED shape', aim at the bar from where you "
                "play with no bright light in view, then pan slowly so a lamp "
                "or a window comes into the picture beside it. It needs both: "
                "the LEDs to find the ceiling, the stray light to know there "
                "is room under it.")

    btn_fit = tk.Button(rowfit, text="Measure the gate",
                        command=lambda: fit_ask(False), font=(F[0], 9),
                        bg="#1f6feb", fg="white", relief="flat", padx=8,
                        pady=0)
    btn_fit.pack(side="left")
    # Disabled until the gun has actually named a number. There is nothing to
    # apply after 'no safe gate' -- that rig cannot be gated on size at all --
    # and a live button there would invite a value the gun has just explained
    # it does not have.
    btn_fit_apply = tk.Button(rowfit, text="Apply", font=(F[0], 9),
                              command=lambda: fit_ask(True), bg="#161b22",
                              fg=C_FG, relief="flat", padx=8, pady=0,
                              state="disabled")
    btn_fit_apply.pack(side="left", padx=(6, 8))
    # ONE line, and every sentence below is measured against it: this label
    # gets 380 px of a 1400 px screen and 369 on the narrow layout, where the
    # preview has taken a tab of its own. Several outgrow it -- either warning
    # once it names more than one limit to switch off (610 px with all three),
    # and any fit answer carrying the samples the gun set aside (671 with the
    # stored pair beside them) -- and each wraps to a SECOND line, which takes
    # the panel to 297 px of the 313 a 768p laptop allows. Two is the budget:
    # a third would be 311 and there is no room to spend on white space, so
    # every clause added here gets measured against 738 px, the narrow
    # layout's two lines.
    #
    # A FIXED two lines was tried first -- the same don't-shuffle reason the
    # preview's verdict has one -- and the panel has 27 px over that 313, so a
    # standing second line spends half of it on white space for the sake of
    # one transient state. Natural height instead: a sentence that outgrows
    # the line wraps and costs its 14 px only while it is up, rather than
    # being clipped with nothing to say it was.
    gate_lbl = lab(rowfit, "", (F[0], 8), C_DIM, justify="left", anchor="nw",
                   wraplength=380)
    gate_lbl.pack(side="left", fill="x", expand=True)

    def gate_line():
        """(text, colour) for the gate line: what is WRONG first, then what
        the gun measured.

        A warning outranks the fit because it is the live thing: a gate that
        is eating corners now matters more than a ceiling somebody measured a
        minute ago, and the fit is in the log as well while this line is the
        only place the warning appears."""
        f = link.fit
        # What the GUN holds, against the ladder beside it. A fit lands on
        # whatever sits between the two measurements -- 9, 11, 13 -- and the
        # spinbox shows its own rungs and nothing else, so without this the
        # panel reads "off" over a gun that is gating at 11.
        held = link.last.get("bhmax") or 0
        odd = bool(held) and held not in shape_var["bhmax"][2]
        # Which of the three shape limits are actually SET, and what the user
        # would have to type to switch each of them off. The shape gate is
        # bhmax OR pxmax OR armax: a warning that says "bhmax:0 turns it off"
        # to somebody whose gate is pxmax is advice that changes nothing, and
        # they have no way to find out it was the wrong advice.
        on = [k for k in ("bhmax", "pxmax", "armax") if link.last.get(k)]
        gate_on = bool(on)
        off_hint = ("%s turn%s it off"
                    % (" and ".join("%s:0" % k for k in on),
                       "" if len(on) > 1 else "s"))
        # SET BUT INERT: a limit the gun is carrying and cannot apply, because
        # the box it judges is only reported in full mode. It is the one state
        # 'cam?' cannot show -- the limits and fmt come back on the same line
        # and nothing there puts them together -- and it is what "I set the
        # gate and nothing happened" turns out to be. The gun says so at the
        # moment it happens, for bhmax and for pxmax, and that word reaches
        # last["fmt"] through pump(); but the STATE outlives the moment, so it
        # is derived here as well. 'ext' is the fallback for a gun too old to
        # report fmt at all -- it can never reach full mode, so its gate can
        # never act either.
        fmt = link.last.get("fmt", link.last.get("ext"))
        full = fmt == 2
        inert = gate_on and fmt is not None and not full
        # A window's verdict speaks for the present or not at all. It is
        # computed when a window of frames closes, so on a gun that has
        # stopped answering -- unplugged, camera dead, port handed to another
        # app -- nothing recomputes it and the last warning would sit here
        # being read as current. That is the latch this panel has already
        # shipped twice.
        seen = blob_state.get("frames_t")
        g = ({} if seen is None or time.monotonic() - seen > GATE_WARN_STALE_S
             else (blob_state.get("gate") or {}))
        near, srej = g.get("bnear", 0), g.get("bsrej", 0)
        frames = g.get("frames", 0)

        def num(v):
            """A figure the gun did not send reads '?', never blank and never
            'None'. A blank where a number belongs is the one thing an older
            firmware must never produce here."""
            return "?" if v is None else v

        # The false-negative meter first: it is the only number on this panel
        # that says a gate is WRONG rather than that it is working, and it
        # moves long before the cursor starts sticking. Both of these are
        # deltas over the window the percentages under them were taken over --
        # never the since-boot totals, which is how "SIZE WINDOW TOO TIGHT"
        # once stayed on the panel for a whole power cycle.
        #
        # bnear now counts SHAPE-gate rejections only, so when it speaks the
        # shape gate is the culprit and the only question left is which of the
        # three limits did it. It is named. The line used to say the height
        # gate was innocent whenever it was inert, which was true while bnear
        # could also come from the size window and is wrong now.
        if near and gate_on:
            return ("GATE MAY BE TAKING REAL LEDs (%d) -- %s"
                    % (near, off_hint)), C_BAD
        if inert:
            if len(on) == 1:
                return ("%s %s is SET BUT INERT -- it needs blob detail "
                        "'full detail'"
                        % (on[0], link.last.get(on[0]))), C_BAD
            return ("%s are SET BUT INERT -- they need blob detail 'full "
                    "detail'" % "+".join(on)), C_BAD
        # Gated on the gate being ON and ACTING, like pical's. Outside full
        # mode the gun reports no box, the shape gate stands down and bsrej
        # cannot move -- so a rate computed there is a rate about nothing, and
        # with every limit off there is nothing the advice could name.
        if (gate_on and full and srej
                and srej >= frames * GATE_HEAVY_PER_FRAME):
            # srej here is already the UNEXPLAINED count -- refusals the
            # resolver did not vouch for as stray light -- see blob_tick.
            return ("SHAPE GATE REFUSING HEAVILY (%d unexplained) -- %s"
                    % (srej, off_hint)), C_WARN
        if f.seq == fit_state["seq"] and fit_state["asked"]:
            # Nothing new since we asked. Either it is still coming, or this
            # gun has never heard of the command and never will answer.
            if time.monotonic() - fit_state["asked"] < 2.0:
                return "gate fit: asking the gun...", C_DIM
            return ("gate fit: no answer -- this gun's firmware has no "
                    "camfit"), C_DIM
        # Contamination rides on whatever the fit is saying rather than
        # replacing it. The gun sets these samples aside BEFORE it derives
        # anything, so the ceiling beside them is the clean one -- and a reader
        # who is shown "32 set aside at 31 tall" learns their rig had the sun
        # in its LED class, which a clean 7 on its own never tells them. Its
        # own line begins with a digit, which is how it fell through both the
        # parser and this panel: the one sentence that says the capture was
        # contaminated was the one sentence nothing read.
        ign = ""
        if f.ignored:
            ign = ("   %d set aside at %s tall, body stops at %s"
                   % (f.ignored[0], num(f.ignored[1]), num(f.ignored[2])))
        if f.verdict == "need_led":
            # The stored pair is what the gun measures its REFUSALS against,
            # so before a capture finishes it is the only explanation on offer
            # for "why will it not take my number?". A stray edge of 0 means
            # no fit has ever applied on this gun -- camsave records the LED
            # edge alone, since a hand-set ceiling has no stray behind it --
            # and it reads exactly like a gun too old to send the field,
            # because to the reader they are the same thing: nothing measured
            # it. Never "stray at 0", which is a measurement of a dark room.
            st = ""
            if f.stored:
                sl, ss, _px = f.stored
                st = ("; stored %s tall, stray %s"
                      % (num(sl), ss if ss and ss > 0 else "not recorded"))
            return ("gate fit: %s of %d LED blobs, %s of %d stray%s%s"
                    % (num(f.led_n), f.led_want, num(f.stray_n),
                       f.stray_want, st, ign)), C_DIM
        if f.verdict == "need_stray":
            return ("gate fit: LEDs reach %s, %s of %d stray -- sweep past a "
                    "lamp%s" % (num(f.led_max_h), num(f.stray_n),
                                f.stray_want, ign)), C_DIM
        if f.verdict == "no_gate":
            # No number, on purpose, and none is offered anywhere else either.
            # This rig's LEDs are as tall as its stray light, so a size gate
            # cannot tell them apart and the honest answer is that there is
            # nothing to set. The sentence saying what CAN be done is in the
            # log, in the gun's own words. The contamination clause matters
            # most here: samples set aside are the commonest reason a rig
            # looks unseparable when it is not.
            return ("gate fit: NO SAFE GATE -- LEDs reach %s, stray starts "
                    "at %s%s" % (num(f.led_max_h), num(f.stray_min_h),
                                 ign)), C_WARN
        if f.verdict == "ok" and f.bhmax is not None:
            if f.applied is True:
                return ("gate fit: bhmax %d applied and saved%s%s"
                        % (f.bhmax, "  no step for it in the box"
                           if odd else "", ign)), C_OK
            if f.applied is False:
                return ("gate fit: bhmax %d set but NOT SAVED -- gone at "
                        "power-off" % f.bhmax), C_BAD
            return ("gate fit: bhmax %d (LEDs %s, stray %s)%s -- Apply saves "
                    "it%s" % (f.bhmax, num(f.led_max_h), num(f.stray_min_h),
                              " TIGHT" if f.tight else "", ign)), C_OK
        if f.seq:
            # A header and no outcome this parsing knows: a firmware that has
            # grown a case since. The counts it did send are still worth
            # having, and the log has the sentence verbatim.
            return ("gate fit: %s LED blobs, %s stray -- see the log"
                    % (num(f.led_n), num(f.stray_n))), C_DIM
        if odd:
            # Said before anyone has asked for a fit, because it is the panel
            # contradicting itself: the gun is gating and the box says 'off'.
            return ("gate fit: the gun holds bhmax %d, no step for it in the "
                    "box" % held), C_WARN
        return "gate fit: press Measure -- it reads this gun's own LEDs", C_DIM

    def fit_tick():
        try:
            txt, col = gate_line()
            if gate_lbl.cget("text") != txt:
                gate_lbl.config(text=txt, fg=col)
            else:
                gate_lbl.config(fg=col)
            # Only ever enabled on a verdict that named a number.
            want = "normal" if (link.fit.verdict == "ok"
                                and link.fit.bhmax is not None) else "disabled"
            if str(btn_fit_apply.cget("state")) != want:
                btn_fit_apply.config(state=want)
        except Exception as e:
            log("gate line hiccup: %s" % e)
        finally:
            root.after(500, fit_tick)

    blob_lbl = lab(frame_wii, "blob readout: set blob detail to 'sizes' to "
                   "fill this line",
                   (F[0], 8), C_DIM, justify="left", anchor="w", wraplength=560)
    blob_lbl.pack(fill="x")
    blob_lbl2 = lab(frame_wii, "", (F[0], 8), C_DIM, justify="left", anchor="w",
                    wraplength=560)
    blob_lbl2.pack(fill="x", pady=(0, 1))
    # Wrap to the width this panel ACTUALLY has, measured, not guessed. These
    # lines carry live numbers of unpredictable length, and a Tk label that
    # overruns its frame is clipped silently -- the reader simply never sees
    # the end of the sentence. Driven from the PARENT's resize, so setting a
    # child's wraplength cannot feed back into the event that set it.
    def wrap_blob(ev):
        w = max(240, ev.width - 12)
        for lb in (blob_lbl, blob_lbl2):
            if lb.cget("wraplength") != w:
                lb.config(wraplength=w)
        # The gate line gets what the two buttons on its row leave, measured
        # off the buttons rather than taken from a constant: they are as wide
        # as their own labels, and a constant would be wrong the day one of
        # those labels changed -- silently, by clipping the end of a warning.
        gw = max(200, w - btn_fit.winfo_reqwidth()
                 - btn_fit_apply.winfo_reqwidth() - 20)
        if gate_lbl.cget("wraplength") != gw:
            gate_lbl.config(wraplength=gw)
    frame_wii.bind("<Configure>", wrap_blob)

    barw = tk.Frame(frame_wii, bg=C_BG); barw.pack(fill="x", pady=(1, 0))
    tk.Button(barw, text="Save to gun", command=lambda: (link.send("~camsave"),
                                     log("~camsave sent -- the gun answers "
                                         "with a CAM: saved / SAVE FAILED line "
                                         "listing what it wrote")),
              font=FB, bg="#238636", fg="white", relief="flat", padx=14, pady=2).pack(side="left")
    tk.Button(barw, text="Read from gun", command=lambda: link.send("~cam?"),
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=2).pack(side="left", padx=8)

    # ---- shape learning ----------------------------------------------------
    # What a confirmed LED actually looks like on this rig, measured instead of
    # guessed: the gun accumulates six per-blob features in two classes, and
    # the label comes from the quad resolver rather than from any gate, so
    # nothing here is a gate being taught by its own decisions.
    #
    # On THIS bar, and not on a row of its own, because there is no room for a
    # row: the panel has to hold the Advanced disclosure under it as well, and
    # a button row costs 30 px of a tab area that stops growing at 313.
    # Everything these two have to explain goes to the log, which is where
    # this panel already sends the reasoning that will not fit beside a
    # control.
    learn_state = {"on": False}

    def learn_label():
        return "Stop learning" if learn_state["on"] else "Learn LED shape"

    def learn_toggle():
        on = not learn_state["on"]
        learn_state["on"] = on
        link.send("~camlearn=on:%d" % (1 if on else 0))
        # Ask straight back, so the button and the counts follow the GUN's
        # answer within a tick instead of this app's memory of the click.
        link.send("~camlearn?")
        btn_learn.config(text=learn_label())
        if not on:
            log("shape capture stopped. Press 'Shape CSV' to write the "
                "histograms out -- starting another capture CLEARS them.")
            return
        log("shape capture ON, starting from empty. It counts only blobs from "
            "frames where the resolver found all four corners, so aim at the "
            "bar from where you actually play and let it run; then press "
            "'Shape CSV'.")
        if cam_fmt.get() != 2:
            # Last, so it is the line still visible in a six-line log box.
            log("WARNING: blob detail is not 'full detail', so the only "
                "histogram that will fill is blob SIZE -- and size is the one "
                "feature already known NOT to separate a window from an LED. "
                "Set 'full detail' above and start the capture again.")

    def shape_write(hs):
        outdir = os.path.join(HERE, "calib_out")
        try:
            os.makedirs(outdir, exist_ok=True)
            # Numbered, not just stamped. Two of these get written inside the
            # same second -- press, look, press again -- and the wall clock
            # alone silently overwrote the first one, which is a capture off a
            # rig that has since been moved.
            path, seq = seq_path(outdir, "shape")
            n = write_shape_csv(path, hs)
        except Exception as e:
            log("could not write the shape CSV: %s" % e)
            return
        frames, led, rej = hs.counts()
        # The NUMBER is said out loud and first: it is the whole point of
        # having one, and it is what the user has to be able to repeat back
        # down a phone line to say which file to send.
        log("capture %d: wrote %d rows to %s -- %d confirmed frames, %d LED "
            "blobs, %d rejected blobs"
            % (seq, n, os.path.basename(path), frames, led, rej))
        if not led:
            log("...and every bin in it is empty: no confirmed LED blobs were "
                "measured. Either the capture was never on, or the resolver "
                "never saw four corners while it was.")
        elif hs.total(0, "aspect") == 0:
            log("...but only the SIZE histogram has anything in it: the box, "
                "aspect, area and brightness features need 'full detail', "
                "which this capture did not run in.")

    def shape_save():
        # Ask, then WAIT for a set the gun sent AFTER the asking. Writing
        # whatever is held would happily produce a file from a capture that
        # ended ten minutes ago, and the file gives no hint of its own age.
        seq0 = link.hists.seq
        link.send("~camlearn?")
        deadline = time.monotonic() + 2.0

        def check():
            hs = link.hists
            if hs.seq != seq0 and hs.ready():
                shape_write(hs)
                return
            if time.monotonic() > deadline:
                log("no complete set of histograms came back in 2 s -- "
                    "nothing written. A gun on older firmware does not have "
                    "'~camlearn' at all and answers nothing.")
                return
            root.after(100, check)
        root.after(100, check)

    tk.Button(barw, text="Shape CSV", command=shape_save, font=F,
              bg="#161b22", fg=C_FG, relief="flat", padx=12,
              pady=2).pack(side="right")
    # A fixed width in characters, so the label swapping between "Learn LED
    # shape" and "Stop learning" cannot change how wide this row is: a row that
    # grows past the panel runs off the right-hand edge with nothing to say so.
    btn_learn = tk.Button(barw, text=learn_label(), command=learn_toggle,
                          font=F, width=15, bg="#161b22", fg=C_FG,
                          relief="flat", padx=6, pady=2)
    btn_learn.pack(side="right", padx=(0, 8))

    # ---- everything set once, or not at all --------------------------------
    # Hidden by default, and not because any of it is dangerous: it is because
    # the panel is read under time pressure with a gun in one hand. Two of the
    # controls below are superseded by the height gate on the front, two are
    # raw sensor registers nobody touches twice, one is the undocumented mode
    # byte, one is a diagnostic run when nothing works at all, and the size
    # window and odd-one-out are set once for a room. Collapsed they cost the
    # 25 px of this button; open they add 98, and the panel still fits the
    # 313 px tab area a 768p laptop allows with the render test's margin to
    # spare -- which the front rows and these together would NOT.
    adv_state = {"on": False}

    def adv_label():
        return ("▾  Advanced settings" if adv_state["on"]
                else "▸  Advanced settings")

    def adv_toggle():
        adv_state["on"] = not adv_state["on"]
        btn_adv.config(text=adv_label())
        if adv_state["on"]:
            frame_adv.pack(fill="x")
        else:
            frame_adv.pack_forget()

    rowadv = tk.Frame(frame_wii, bg=C_BG); rowadv.pack(fill="x", pady=(1, 0))
    btn_adv = tk.Button(rowadv, text=adv_label(), command=adv_toggle,
                        font=(F[0], 9), bg="#161b22", fg=C_DIM, relief="flat",
                        anchor="w", padx=8, pady=1)
    btn_adv.pack(side="left")
    # The list is what makes a disclosure findable rather than a place things
    # go to hide, and it is measured to the width the panel actually has: 154
    # px of button and 8 of gap leave 421, and this is 392 of them.
    lab(rowadv, "size window, odd-one-out, superseded limits, sensor "
        "registers/test", (F[0], 8), C_DIM).pack(side="left", padx=(8, 0))
    # Packed only when the button says so. Built now so nothing here has to be
    # created on a click -- a widget built inside a Tk callback that raises
    # leaves the disclosure permanently empty with only a traceback on a
    # stderr the user does not have.
    frame_adv = tk.Frame(frame_wii, bg=C_BG)

    # No pady between these four rows, unlike every row on the front: 2 px a
    # row is 8 px of the 36 the panel has left over with this open, and these
    # are spinboxes set once rather than a readout scanned mid-test.
    rowsz = tk.Frame(frame_adv, bg=C_BG); rowsz.pack(fill="x")
    for key, name in (("bmin", "keep from size"), ("bmax", "up to")):
        lab(rowsz, name, (F[0], 9), C_DIM).pack(side="left", padx=(0, 2))
        gv = tk.IntVar(value=0 if key == "bmin" else 15)
        bgate[key] = gv
        sp = tk.Spinbox(rowsz, from_=0, to=15, width=3, textvariable=gv, font=F,
                        bg="#161b22", fg=C_FG, relief="flat",
                        command=lambda k=key, v=gv: gate_send(k, v))
        sp.pack(side="left", padx=(0, 10))
        sp.bind("<Return>", lambda _e, k=key, v=gv: gate_send(k, v))
        bspin[key] = sp
    # Odd-one-out: four identical emitters should look alike in one frame, so a
    # blob that does not match the others is the suspect. Needs no distance
    # tuning, which is what defeats the absolute window beside it.
    lab(rowsz, "odd-one-out steps", (F[0], 9), C_DIM).pack(side="left",
                                                           padx=(4, 2))
    gv = tk.IntVar(value=0)
    bgate["rtol"] = gv
    sp = tk.Spinbox(rowsz, from_=0, to=15, increment=1, width=4,
                    textvariable=gv, font=F, bg="#161b22", fg=C_FG,
                    relief="flat",
                    command=lambda k="rtol", v=gv: gate_send(k, v))
    sp.pack(side="left", padx=(2, 0))
    sp.bind("<Return>", lambda _e, k="rtol", v=gv: gate_send(k, v))
    bspin["rtol"] = sp

    # The two shape limits the height gate replaced. Kept because a gun in the
    # field may still be set on them and the panel must be able to say so and
    # turn them off -- not because either is a good idea now.
    #
    # A word each rather than one heading over both: they are not superseded in
    # the same way. pxmax measures the right thing on the wrong scale -- a bar
    # with five LEDs per cluster fills several times the pixels of one with two
    # -- while armax measures a shape the sensor stopped producing when the
    # default sensitivity moved to 2. Measured, one heading plus both verdicts
    # is 660 px of a 602 px panel, and the row runs off the edge in silence.
    rowsh = tk.Frame(frame_adv, bg=C_BG); rowsh.pack(fill="x")
    shape_box(rowsh, "pxmax", "pixel count", PXMAX_STEPS, "superseded")
    # The one short clause there is room for, and DEPRECATED is the half of it
    # that matters: the gun still loads and applies a stored armax so a setting
    # already in the field keeps its meaning, but nothing suggests a value for
    # it and 'Measure the gate' never sets one. WHY it is wrong -- at
    # sensitivity 2 the sensor smears a 2x2 blob out to 12x3, so the ratio
    # measures the gain and not the LED -- goes to the log the moment anybody
    # switches it on, which is the only moment it is worth reading.
    shape_box(rowsh, "armax", "roundness", ARMAX_STEPS,
              "DEPRECATED, not recommended")

    # Which byte selects full mode is the one thing about it nobody can look
    # up: the driver's own working constants for the other two formats are the
    # doubled nibble (0x11, 0x33), which makes 0x55 the consistent choice,
    # while the published tables say the mode is simply 5. Both readings can be
    # right, so the switch is here rather than in a reflash -- a user whose
    # preview shows crosshairs outside their boxes has something to try.
    rowfr = tk.Frame(frame_adv, bg=C_BG); rowfr.pack(fill="x")
    cam_fullreg = tk.IntVar(value=85)

    def send_fullreg():
        v = cam_fullreg.get()
        link.send("~cam=fullreg:%d" % v)
        link.send("~camblob?")
        log("full-mode register -> 0x%02x. Which byte this sensor wants for "
            "full mode is not documented anywhere we trust, so it is a "
            "setting rather than a reflash: if the preview draws boxes that "
            "do not follow the LEDs, or crosshairs that miss their boxes, the "
            "other value is the thing to try. Not saved -- 0x55 comes back on "
            "the next power-cycle." % v)
    lab(rowfr, "full-mode register", (F[0], 9), C_DIM).pack(side="left")
    for rv, nm in ((85, "0x55"), (5, "0x05")):
        tk.Radiobutton(rowfr, text=nm, value=rv, variable=cam_fullreg,
                       command=send_fullreg, font=(F[0], 9), bg=C_BG, fg=C_FG,
                       selectcolor="#161b22", activebackground=C_BG,
                       activeforeground=C_FG, highlightthickness=0, bd=0
                       ).pack(side="left")
    # The two SENSOR thresholds ride on the register row: everything on it is
    # a byte written straight into a camera register, and everything on the
    # rows above it is judged on this side of the wire.
    for key, lo, hi, step in (("hwmax", -1, 255, 5), ("hwmin", -1, 255, 1)):
        lab(rowfr, "sensor %s" % ("max" if key == "hwmax" else "min"),
            (F[0], 9), C_DIM).pack(side="left", padx=(12, 2))
        gv = tk.IntVar(value=lo)
        bgate[key] = gv
        sp = tk.Spinbox(rowfr, from_=lo, to=hi, increment=step, width=4,
                        textvariable=gv, font=F, bg="#161b22", fg=C_FG,
                        relief="flat",
                        command=lambda k=key, v=gv: gate_send(k, v))
        sp.pack(side="left", padx=(2, 0))
        sp.bind("<Return>", lambda _e, k=key, v=gv: gate_send(k, v))
        bspin[key] = sp

    # Sensor connection test: which of power, wiring and the sensor itself is
    # broken, straight from the gun's own pins -- and a live camera restart
    # when the fault turns out to be fixed. Run once, when nothing works at
    # all, which is exactly why it does not belong on the front of the panel.
    rowdg = tk.Frame(frame_adv, bg=C_BG); rowdg.pack(fill="x")
    tk.Button(rowdg, text="Test sensor connection", font=FB, bg="#1f6feb",
              fg="white", relief="flat", padx=12, pady=2,
              command=lambda: (link.send("~camdiag"),
                               log("~camdiag sent -- the gun answers with "
                                   "CAM: diag lines; the VERDICT line names "
                                   "what is broken"))).pack(side="left")
    # One line and a short one: measured, the button takes 229 px of the 590
    # this panel gets and a Tk label that overruns its frame is clipped in
    # silence -- the reader simply never sees the end of the sentence.
    lab(rowdg, "checks power, both wires, swapped lines and the sensor",
        (F[0], 8), C_DIM, justify="left").pack(side="left", padx=8)

    # Which notebook pages the blob poll has to keep running for. Two names
    # when the preview has a tab of its own, the same name twice when it does
    # not -- and the poll is what feeds it, so a page missing from here is a
    # preview frozen on whatever it happened to be showing.
    cam_pages = (str(tab_cam), str(prev_page))
    # Once now, so the canvas says what it is waiting for instead of sitting
    # empty until the first reply arrives on a gun that may never send one.
    draw_shapes()

    sliders = {}
    for k in CAM_KEYS:
        lo, hi = CAM_RANGE[k]
        r = tk.Frame(frame_esp, bg=C_BG); r.pack(fill="x", pady=1)
        lab(r, k, F, C_DIM, width=7, anchor="w").pack(side="left")
        v = tk.IntVar(value=lo)
        s = tk.Scale(r, from_=lo, to=hi, orient="horizontal", variable=v,
                     bg=C_BG, fg=C_FG, troughcolor="#161b22", highlightthickness=0,
                     length=260, showvalue=True)
        s.pack(side="left")
        sliders[k] = v
        def mk(kk, vv):
            def on(_=None): link.send("~cam=%s:%d" % (kk, vv.get()))
            return on
        s.config(command=mk(k, v))

    bar = tk.Frame(frame_esp, bg=C_BG); bar.pack(fill="x", pady=6)
    stop_flag = threading.Event()
    def do_auto():
        stop_flag.clear()
        log("auto-tune: sweeping exposure x threshold, about 30 s...")
        def run():
            r = auto_tune(link, lambda m: root.after(0, log, m), stop_flag)
            if r: root.after(0, lambda: (sliders["aec"].set(r[0]), sliders["thr"].set(r[1])))
        threading.Thread(target=run, daemon=True).start()
    tk.Button(bar, text="Auto-tune", command=do_auto, font=FB, bg="#1f6feb", fg="white",
              relief="flat", padx=14, pady=6).pack(side="left")
    tk.Button(bar, text="Save to gun", command=lambda: (link.send("~camsave"),
                                     log("~camsave sent -- the gun answers "
                                         "with a CAM: saved / SAVE FAILED line "
                                         "listing what it wrote")),
              font=FB, bg="#238636", fg="white", relief="flat", padx=14, pady=6).pack(side="left", padx=8)
    tk.Button(bar, text="Read from gun", command=lambda: link.send("~cam?"),
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=6).pack(side="left")
    tk.Button(bar, text="Cancel", command=stop_flag.set,
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=6).pack(side="right")

    # ---- tab: lens / FOV --------------------------------------------------
    # Only matters when the camera does not wear the stock 66-degree lens. A
    # wide or fisheye lens bends the LED quad; the homography assumes a pinhole,
    # and the calibration has nowhere to put a radially-varying error -- so the
    # correction has to happen upstream, on the blob centroids, in the firmware.
    # This tab sets that correction: a preset from the datasheet FOV, or a
    # measured fit from a 20-second sweep. Save writes it to NVS with ~camsave.
    import calib_lens
    tab_lens = tk.Frame(nb, bg=C_BG)
    nb.add(tab_lens, text="  Lens  ")

    lens_state = lab(tab_lens, "current: unknown -- press Read from gun", (F[0], 9), C_DIM,
                     anchor="w")
    lens_state.pack(fill="x", pady=(6, 2))

    rowp = tk.Frame(tab_lens, bg=C_BG); rowp.pack(fill="x", pady=3)
    lab(rowp, "lens FOV:", F, C_DIM).pack(side="left")
    fov_var = tk.StringVar(value="66")
    tk.Entry(rowp, textvariable=fov_var, width=5, font=F, bg="#161b22", fg=C_FG,
             insertbackground=C_FG, relief="flat").pack(side="left", padx=(4, 2))
    lab(rowp, "deg (full horizontal, from the lens listing)", (F[0], 9), C_DIM).pack(side="left")

    def fov_value():
        try:
            v = float(fov_var.get())
            if 30 <= v <= 200: return v
        except ValueError:
            pass
        log("lens: FOV must be a number between 30 and 200 degrees")
        return None

    def lens_off():
        link.send("~cam=lens:0")
        log("lens: correction OFF (stock lens). Press Save to keep it.")

    def lens_preset():
        fov = fov_value()
        if fov is None: return
        if fov <= 75:
            log("lens: %.0f deg is close enough to a pinhole that no preset is"
                " needed -- the calibration absorbs the focal length itself."
                " Use Measure if the image is visibly bent." % fov)
            return
        r = calib_lens.spec_fisheye(fov)
        link.send("~cam=" + calib_lens.tune_line(dict(r, model="fisheye")))
        log("lens: fisheye preset applied for %.0f deg (feq=%.1f, fpx=%.1f)."
            % (fov, r["feq"], r["fpx"]))
        log("lens: a preset assumes an ideal equidistant lens. Measure beats it.")
        log("Press 'Save to gun' to keep it across power cycles.")

    lens_busy = {"on": False}

    def lens_measure():
        if lens_busy["on"]:
            log("lens: a measurement is already running"); return
        fov = fov_value()
        if fov is None: return
        if not link.src:
            log("lens: not connected"); return
        lens_busy["on"] = True
        # Snapshot the live lens state BEFORE the sweep turns it off, so a
        # refusal can put it back instead of leaving the correction off.
        prev_lens = {k: link.last.get(k, 0) for k in LENS_KEYS}
        # Snapshot the pointer choice BEFORE freezing: the gun's "pointer
        # FROZEN" reply to our own freeze writes hid_on=False through the
        # pump's feedback parser, so reading hid_on at restore time would
        # restore the freeze itself and leave the cursor dead after Measure.
        want_hid = link.hid_on
        # Dropout accounting baseline: full and partial frames seen so far.
        n_full0, n_part0 = link.frames, link.partial_n
        # Freeze the pointer for the sweep: with the resolver off the firmware
        # falls back to OpenFIRE's stock aim, which sends the cursor jumping
        # all over the desktop while you pan. Restored when the fit lands.
        link.pointer(False, remember=False)
        # raw data: resolver off (it invents corners), correction off (fitting
        # corrected data fits garbage), full frame rate
        link.send("~cam=res:0,lens:0,dashhz:0")
        log("lens: MEASURING for 20 s. Stand at a distance that keeps all four")
        log("LEDs in view with the quad still a good size, feet planted, and")
        log("slowly pan/tilt/roll the gun so the LEDs travel across the WHOLE")
        log("image -- push them out to the edges and corners. Wiicam: use at")
        log("least High sensitivity for a fisheye; go up if the view is choppy.")
        t0 = time.time()
        frames = []
        # hist timestamps are the GUN's clock, not ours -- comparing them
        # against time.time() collects garbage. Accumulate by identity instead:
        # every (gun-time, quad) pair not seen before is a new frame.
        seen = set()

        def collect():
            for (gt, q) in link.hist:
                if gt not in seen:
                    seen.add(gt)
                    frames.append(np.asarray(q, float))
            left = 20.0 - (time.time() - t0)
            if left > 0:
                lens_state.config(text="measuring... %2.0f s left, %d frames"
                                  % (left, len(frames)), fg=C_WARN)
                root.after(250, collect)
                return
            link.send("~cam=res:2,dashhz:60")
            lens_state.config(text="fitting...", fg=C_WARN)
            snap = np.array(frames) if frames else np.zeros((0, 4, 2))

            # Save every sweep, pass or fail, in the Q format calib_lens.py
            # reads -- a refusal without data cannot be diagnosed.
            sweep_path = ""
            if len(snap):
                try:
                    outdir = os.path.join(HERE, "calib_out")
                    os.makedirs(outdir, exist_ok=True)
                    # Numbered like the captures, and for the same reason: a
                    # sweep that failed and a sweep repeated straight away are
                    # the pair most worth comparing, and the wall clock alone
                    # let the second one land on the first.
                    sweep_path, _seq = seq_path(outdir, "lenssweep", ".log")
                    with open(sweep_path, "w") as fh:
                        for i, q in enumerate(snap):
                            fh.write("Q,%d,4," % (i * 7) +
                                     ",".join("%d,%d" % (round(pt[0]*10), round(pt[1]*10))
                                              for pt in q) + "\n")
                except Exception:
                    sweep_path = ""

            def fit():
                # never let an exception strand the sweep with the pointer
                # frozen and the lens off -- done() runs whatever happens
                try:
                    r = calib_lens.fit_from_frames(snap, fov)
                except Exception as e:
                    r = dict(ok=False, why="fitter crashed: %r" % e,
                             model=None, rms_px=0.0, coverage=0.0)

                def done():
                    lens_busy["on"] = False
                    # release the pointer back to the user's pre-sweep choice
                    link.pointer(want_hid)
                    # the numbers a refusal needs to be diagnosable
                    if len(snap):
                        rad = np.linalg.norm(snap - (120.0, 88.0), axis=-1)
                        spans = [aim_fit.quad_span(q) for q in snap]
                        log("lens: sweep %d frames, coverage %.0f%%, quad span "
                            "median %.1f px, max radius %.1f px"
                            % (len(snap), r.get("coverage", 0)*100,
                               float(np.median(spans)), float(rad.max())))
                        if float(np.median(spans)) < 25.0:
                            log("lens: the quad is SMALL -- with an ultra-wide "
                                "lens stand ~0.5 m from the rig and sweep again")
                    # Dropout accounting: the sweep runs RAW, so what the
                    # resolver normally hides is visible here. A wide lens
                    # dims off-axis LEDs and the sensor drops them.
                    fulls = link.frames - n_full0
                    parts = link.partial_n - n_part0
                    if parts > fulls:
                        log("lens: DROPOUTS -- %d of %d frames were missing "
                            "LEDs. The lens dims off-axis LEDs below the "
                            "sensor threshold. Raise sensitivity (camera "
                            "tab), add LED power, or stand closer, then "
                            "re-run Measure for a cleaner fit."
                            % (parts, parts + fulls))
                    if sweep_path:
                        log("lens: sweep saved: %s" % sweep_path)
                    if not r["ok"]:
                        # put the pre-sweep correction back -- the sweep
                        # switched it off and a refusal must not leave it off
                        if prev_lens.get("lens"):
                            link.send("~cam=" + ",".join(
                                "%s:%d" % (k, prev_lens[k]) for k in LENS_KEYS))
                            log("lens: previous correction restored")
                        lens_state.config(text="measure failed -- see the log", fg=C_BAD)
                        log("lens: REFUSED: %s" % r["why"])
                        return
                    link.send("~cam=" + calib_lens.tune_line(r))
                    if r["model"] == "none":
                        lens_state.config(
                            text="measured: no correction needed (pinhole "
                                 "within noise)", fg=C_OK)
                        log("lens: this lens shows no measurable distortion "
                            "on this sensor -- correction set to OFF.")
                        log("Press 'Save to gun' to keep it across power cycles.")
                        return
                    lens_state.config(
                        text="measured: %s  rms %.2f px  (coverage %.0f%%)"
                             % (r["model"], r["rms_px"], r["coverage"] * 100), fg=C_OK)
                    log("lens: fitted %s model, residual %.2f px rms." % (r["model"], r["rms_px"]))
                    if r.get("lcx") or r.get("lcy"):
                        log("lens: decentered lens detected -- distortion "
                            "centre offset %+.1f, %+.1f px, compensated."
                            % (r["lcx"], r["lcy"]))
                    if r["rms_px"] > 1.0:
                        log("lens: residual is high -- consider redoing the sweep more slowly.")
                    log("Applied live. Press 'Save to gun' to keep it across power cycles.")
                    log("IMPORTANT: the aim calibration was made under the OLD "
                        "lens mapping -- redo Calibrate (step 4) now, then "
                        "Fine tune, or aim will be warped.")
                root.after(0, done)
            threading.Thread(target=fit, daemon=True).start()
        root.after(250, collect)

    rowb = tk.Frame(tab_lens, bg=C_BG); rowb.pack(fill="x", pady=6)
    tk.Button(rowb, text="Stock lens (off)", command=lens_off, font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=12, pady=6).pack(side="left")
    tk.Button(rowb, text="Preset from FOV", command=lens_preset, font=FB, bg="#1f6feb",
              fg="white", relief="flat", padx=12, pady=6).pack(side="left", padx=8)
    tk.Button(rowb, text="Measure (20 s sweep)", command=lens_measure, font=FB,
              bg="#1f6feb", fg="white", relief="flat", padx=12, pady=6).pack(side="left")
    # Output dead-band: swallows sub-threshold rest shimmer at the cursor,
    # never delays real motion. Set-once-per-lens, so it lives here, not in
    # the fine-tune bar. 0 = off; ~16-32 suits a wide lens.
    def dead_nudge(d):
        v = max(0, min(128, int(link.last.get("dead", 0)) + d))
        link.last["dead"] = v
        link.send("~cam=dead:%d" % v)
        log("dead-band -> %d units (0 = off; Save to gun to keep)" % v)
    rowd = tk.Frame(tab_lens, bg=C_BG); rowd.pack(fill="x", pady=(0, 4))
    lab(rowd, "Dead-band (rest shimmer):", F, C_DIM).pack(side="left")
    tk.Button(rowd, text="-", command=lambda: dead_nudge(-8), font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=10).pack(side="left", padx=(8, 2))
    tk.Button(rowd, text="+", command=lambda: dead_nudge(+8), font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=10).pack(side="left", padx=2)
    dead_lbl = lab(rowd, "off", F, C_DIM); dead_lbl.pack(side="left", padx=8)

    # One Euro speed sensitivity. The cutoff is min_cutoff + beta*speed, so
    # `smooth` sets how heavy the filter is AT REST and beta sets how quickly
    # it lets go once the gun moves. Raising it shortens the trail on a swipe
    # without touching rest stability.
    def beta_nudge(d):
        cur = int(link.last.get("beta", -1))
        v = 15 if cur < 0 else cur
        v = max(0, min(60, v + d * 3))
        link.last["beta"] = v
        link.send("~cam=beta:%d" % v)
        log("beta -> %d  (speed sensitivity; 15 = default. Raise if a SLOW "
            "deliberate drag feels sticky. It is worth a few ms at most and it "
            "does NOT shorten a fast swipe's trail -- that is LEAD. "
            "Save to gun to keep)" % v)
    def beta_reset():
        link.last["beta"] = -1
        link.send("~cam=beta:-1")
        log("beta -> default (15 at every smoothing level)")
    rowt = tk.Frame(tab_lens, bg=C_BG); rowt.pack(fill="x", pady=(0, 4))
    lab(rowt, "Smoothing speed sensitivity (beta):", F, C_DIM).pack(side="left")
    tk.Button(rowt, text="-", command=lambda: beta_nudge(-1), font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=10).pack(side="left", padx=(8, 2))
    tk.Button(rowt, text="+", command=lambda: beta_nudge(+1), font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=10).pack(side="left", padx=2)
    tk.Button(rowt, text="default", command=beta_reset, font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=10).pack(side="left", padx=6)
    tmode_lbl = lab(rowt, "15 (default)", F, C_DIM); tmode_lbl.pack(side="left", padx=8)

    tk.Button(rowb, text="Save to gun", command=lambda: (link.send("~camsave"),
                                     log("~camsave sent -- the gun answers "
                                         "with a CAM: saved / SAVE FAILED line "
                                         "listing what it wrote")),
              font=FB, bg="#238636", fg="white", relief="flat", padx=12, pady=6).pack(side="left", padx=8)
    tk.Button(rowb, text="Read from gun", command=lambda: link.send("~cam?"),
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=6).pack(side="left")
    lab(tab_lens, "Wide lens trade-off: more FOV = stand closer and less edge "
        "clipping, but every\nnoise source is magnified by the shorter focal. "
        "The stock 66\u00b0 lens needs nothing here.", (F[0], 8), C_DIM,
        justify="left", anchor="w").pack(fill="x", pady=(2, 4))

    # ---- recoil tab: the solenoid feel engine ---------------------------
    # Every control is a serial command; the gun echoes what it ACCEPTED and
    # the labels below show that echo, not the button press.
    tab_fx = tk.Frame(nb, bg=C_BG)
    nb.add(tab_fx, text="  Recoil  ")
    def fx_dryfire():
        if not link.src:
            log("not connected -- press Reconnect first")
            return
        # armed/disarmed comes from the gun's own echo -- a local toggle went
        # out of step the moment the 10-minute expiry fired on its own
        if int(link.last.get("fxleft", 0) or 0) > 0:
            link.send("~fx=ab:0")
            link.pointer(True)
            log("dry-fire off; pointer released")
        else:
            link.pointer(False)
            link.send("~fx=ab:1")
            log("DRY-FIRE ON for 10 min: trigger fires with no IR (and sends "
                "no mouse click). Pointer frozen (F9 releases). Click again "
                "to disarm, or just let it expire.")
        link.send("~fx?")

    # actions FIRST: on a fixed-height window whatever packs last is what
    # gets cut, and these are the buttons a bench session cannot live without
    rowfx = tk.Frame(tab_fx, bg=C_BG); rowfx.pack(fill="x", pady=(3, 2))
    tk.Button(rowfx, text="TEST FIRE", font=FB, bg="#1f6feb", fg="white",
              relief="flat", padx=14, pady=5,
              command=lambda: link.send("~fx=test:1")).pack(side="left")
    tk.Button(rowfx, text="Dry-fire", font=FB, bg="#9e6a03", fg="white",
              relief="flat", padx=10, pady=5,
              command=fx_dryfire).pack(side="left", padx=4)
    tk.Button(rowfx, text="Engine ON", font=FB, bg="#238636", fg="white",
              relief="flat", padx=10, pady=5,
              command=lambda: link.send("~fx=on:1")).pack(side="left", padx=4)
    tk.Button(rowfx, text="off (stock)", font=F, bg="#161b22", fg=C_FG,
              relief="flat", padx=8, pady=5,
              command=lambda: link.send("~fx=on:0")).pack(side="left")
    tk.Button(rowfx, text="Save", font=FB, bg="#238636", fg="white",
              relief="flat", padx=10, pady=5,
              command=lambda: link.send("~fxsave")).pack(side="left", padx=4)
    tk.Button(rowfx, text="Read", font=F, bg="#161b22", fg=C_FG,
              relief="flat", padx=8, pady=5,
              command=lambda: link.send("~fx?")).pack(side="left")
    # state gets its own line: appended to the button row it fell off the
    # right edge of the fixed window
    fx_on_lbl = lab(tab_fx, "state: press Read -- engine OFF means stock "
                    "behaviour", (F[0], 8), C_DIM, anchor="w")
    fx_on_lbl.pack(fill="x", pady=(0, 1))

    FX_KNOBS = (
        ("drive",  "Strike ms",   5, 5,  100,  "the hit; 45 = stock default"),
        ("hold",   "Hold ms",     10, 0,  500, "held push after the hit"),
        ("duty",   "Hold pwr %",  5, 25,  70,  "raise until hold stops buzzing"),
        ("pulse",  "Pulses",      1, 0,   3,   "extra clacks after release"),
        ("gap",    "Gap ms",      5, 15,  120, "short=meaty, long=double-clack"),
        ("jit",    "Jitter %",    3, 0,   15,  "randomness; strike stays exact"),
        ("rumoff", "Rum off ms",  5, -20, 50,  "negative = motor leads the hit"),
        ("rumms",  "Rum time ms", 10, 0,  200, "0 = OpenFIRE owns the rumble"),
        ("space",  "Space ms",    10, 0,  500, "quiet time; sets max fire rate"),
        ("auto",   "Auto wait ms",50, 0, 1000, "hold time before autofire kicks in"),
    )
    fx_lbls = {}

    def fx_nudge(key, d, step, lo, hi):
        # A nudge is an absolute set computed from the last echo. Before any
        # echo exists, computing from a fake 0 stomped the gun's saved value
        # (one click turned a stored drive 45 into 5) -- so the first click
        # reads instead of writing.
        cur = link.last.get("fx" + key)
        if cur is None:
            link.send("~fx?")
            log("recoil: reading the gun's current values first -- "
                "nudge again once they show")
            return
        v = max(lo, min(hi, int(cur) + d * step))
        link.send("~fx=%s:%d" % (key, v))     # the FX: echo updates the label

    # two columns: the single column was taller than the fixed window
    grid = tk.Frame(tab_fx, bg=C_BG); grid.pack(fill="x")
    cols = (tk.Frame(grid, bg=C_BG), tk.Frame(grid, bg=C_BG))
    cols[0].pack(side="left", fill="x", expand=True)
    cols[1].pack(side="left", fill="x", expand=True, padx=(12, 0))
    for i, (key, name, step, lo, hi, tip) in enumerate(FX_KNOBS):
        cell = tk.Frame(cols[i % 2], bg=C_BG); cell.pack(fill="x")
        r = tk.Frame(cell, bg=C_BG); r.pack(fill="x")
        lab(r, name + ":", F, C_DIM, width=11, anchor="w").pack(side="left")
        tk.Button(r, text="-", font=F, bg="#161b22", fg=C_FG, relief="flat",
                  padx=8, command=lambda k=key, st=step, l=lo, h=hi:
                  fx_nudge(k, -1, st, l, h)).pack(side="left", padx=(2, 1))
        tk.Button(r, text="+", font=F, bg="#161b22", fg=C_FG, relief="flat",
                  padx=8, command=lambda k=key, st=step, l=lo, h=hi:
                  fx_nudge(k, +1, st, l, h)).pack(side="left", padx=1)
        fx_lbls[key] = lab(r, "?", F, C_FG); fx_lbls[key].pack(side="left", padx=6)
        # one short line of purpose per knob; enough to tune without the docs
        lab(cell, tip, (F[0], 7), C_DIM, anchor="w").pack(fill="x", padx=(4, 0))


    step_rows[7][0].config(command=lambda: nb.select(tab_fx))

    # ---- USB doctor tab: which of the four USB wires is bad ---------------
    import usb_doctor as UD
    tab_usb = tk.Frame(nb, bg=C_BG)
    nb.add(tab_usb, text="  USB  ")
    for line in UD.WIRE_TABLE:
        lab(tab_usb, line, (F[0], 8), C_DIM, anchor="w").pack(fill="x")
    usb_state = {"watch": None, "until": 0.0, "soak": None}
    usb_lbl = lab(tab_usb, "idle", F, C_DIM, anchor="w")

    def usb_watch():
        # 60 seconds of event logging; the user wiggles one harness section
        # at a time and reads back WHICH SECOND each drop happened
        usb_state["watch"] = UD.Watcher()
        usb_state["until"] = time.time() + 60.0
        log("USB watch: 60 s. Wiggle ONE section of the cable at a time -- "
            "the log timestamps every drop. Start at the connector you "
            "resoldered.")

    def usb_soak():
        if not link.src:
            log("USB soak: not connected -- Reconnect first")
            return
        usb_state["soak"] = [(time.time(), link.frames)]
        log("USB soak: 15 s of stream under load -- stalls without drops "
            "are the marginal-joint signature.")

    def usb_tick():
        w = usb_state["watch"]
        if w is not None:
            left = usb_state["until"] - time.time()
            if left <= 0:
                for line in w.summary():
                    log("USB " + line)
                usb_state["watch"] = None
                usb_lbl.config(text="watch done -- verdicts in the log")
            else:
                for t, kind, text in w.poll():
                    log("USB %5.1fs  %s" % (t, text))
                usb_lbl.config(text="watching... %ds left, %d drop(s), "
                               "%d enum failure(s)"
                               % (left, w.drops, w.descriptor_fails))
        sk = usb_state["soak"]
        if sk is not None:
            sk.append((time.time(), link.frames))
            if sk[-1][0] - sk[0][0] >= 15.0:
                for line in UD.soak_report(sk):
                    log("USB " + line)
                usb_state["soak"] = None
        root.after(500, usb_tick)
    root.after(1500, usb_tick)

    rowu = tk.Frame(tab_usb, bg=C_BG); rowu.pack(fill="x", pady=6)
    tk.Button(rowu, text="Watch 60 s (wiggle the cable)", font=FB, bg="#1f6feb",
              fg="white", relief="flat", padx=12, pady=6,
              command=usb_watch).pack(side="left")
    tk.Button(rowu, text="Stream soak 15 s", font=FB, bg="#1f6feb", fg="white",
              relief="flat", padx=12, pady=6,
              command=usb_soak).pack(side="left", padx=8)
    usb_lbl.pack(fill="x", pady=(2, 0))
    lab(tab_usb, "The watch also catches 'device not recognized' states that "
        "never get a COM port\n(Windows problem codes). For a gun that is "
        "completely dead to the PC, start the watch,\nTHEN plug the gun in.",
        (F[0], 8), C_DIM, justify="left", anchor="w").pack(fill="x", pady=(4, 0))

    fx_poll = {"t": 0.0, "left_seen": 0}

    def fx_tick():
        on = link.last.get("fxon")
        left = link.last.get("fxleft", 0)
        # While dry-fire is armed the countdown is re-read from the gun every
        # few seconds; a snapshot froze at "9:59 left" forever, and after the
        # gun's own expiry the pointer stayed frozen with no one to blame.
        # Re-read while EITHER countdown is running. Quiet mode lapses on the
        # gun the same way dry-fire does, and polling only for dry-fire left
        # the quiet banner and its timer frozen on screen long after the gun
        # had gone back to normal.
        if (left or link.last.get("fxquiet")) and link.src \
                and time.time() - fx_poll["t"] > 3.0:
            fx_poll["t"] = time.time()
            link.send("~fx?")
        if fx_poll["left_seen"] and not left:
            link.pointer(True)
            log("dry-fire expired on the gun -- pointer released")
        fx_poll["left_seen"] = left
        if on is None:
            fx_on_lbl.config(text="state: press Read")
        else:
            t = "engine %s" % ("ON" if on else "off (stock)")
            # the dry-fire countdown: the mode disarms itself, and before this
            # was shown "it stopped working" was the only possible reading
            if left:
                t += "   dry-fire %d:%02d left" % (left // 60, left % 60)
            temp = link.last.get("fxtemp", -1)
            if temp == 2:
                t += "   TEMP FATAL -- TMP36 unplugged or reading garbage; " \
                     "autofire is blocked"
            elif temp == 1:
                t += "   temp WARNING -- sustained fire slowed"
            # Quiet mode says so, loudly. A gun deliberately silenced for a
            # calibration is otherwise indistinguishable from a broken one, and
            # every knob on this tab would look like it had stopped working.
            quiet = link.last.get("fxquiet", 0)
            if quiet:
                ql = int(link.last.get("fxqleft", 0) or 0)
                t += ("   QUIET MODE ON -- nothing fires (%d:%02d left); "
                      "leave the calibration screen to end it" % (ql // 60, ql % 60))
            fx_on_lbl.config(text=t,
                             fg=C_BAD if temp == 2
                             else (C_WARN if quiet else (C_OK if on else C_DIM)))
        for key, _n, _s, _l, _h, _t in FX_KNOBS:
            v = link.last.get("fx" + key)
            fx_lbls[key].config(text="?" if v is None else str(v))
        root.after(400, fx_tick)
    root.after(1200, fx_tick)

    # The blob readout, polled only while the Camera tab is on a wiicam: it is
    # a live measurement of what the sensor is handing us, and the whole point
    # is to see it CHANGE as the gun swings past the window.
    # near_said / srej_said: the long explanation behind each gate warning goes
    # to the log the FIRST time it moves and not on every poll -- the log box
    # is six lines tall, and a warning that repeats twice a second is a warning
    # that pushes the diag verdict and the save result off the end of it.
    # gate: the two shape-gate counts over the last closed WINDOW of frames,
    # for the gate line beside the fit buttons to word. Numbers and not a
    # sentence, because whether "bhmax:0 turns it off" is even true depends on
    # whether the gate can act -- and held here rather than written straight to
    # the label, because it is only recomputed when a window closes while the
    # label is redrawn on its own clock.
    # frames_n / frames_t: the gun's frame counter and when it last moved, so
    # a warning computed from a window of frames stops claiming the present
    # once the frames stop arriving.
    blob_state = {"ref": {}, "line": "", "good": True, "near_said": False,
                  "srej_said": False, "gate": {}, "frames_n": None,
                  "frames_t": None}
    blob_rate = FrameRate()

    def blob_tick():
        # Everything is inside a try whose finally reschedules. A Spinbox the
        # user cleared makes IntVar.get() raise TclError, and with the
        # reschedule as the last statement that exception killed this tick --
        # and with it the whole blob readout -- for the rest of the session,
        # leaving only a traceback on a stderr nobody sees.
        try:
            blob_tick_body()
        except Exception as e:
            log("blob readout hiccup: %s" % e)
        finally:
            root.after(700, blob_tick)

    def blob_tick_body():
        b = link.last.get("board") or ""
        # Compared as widget names, not tab indexes: an index silently means a
        # different tab the moment one is added. The preview is on this same
        # page on a wide enough screen and on a page of its own otherwise, and
        # it is fed from here -- left out, the fallback layout would draw one
        # frame and then freeze on it for as long as its own tab was in front.
        if "wiicam" in b and link.src and nb.select() in cam_pages:
            link.send("~camblob?")
            blob_rate.feed(link.last.get("bframes"), link.last.get("bms"))
            # When the gun last sent a NEW frame, which is what the gate
            # warnings are allowed to speak for. Stamped on the frame counter
            # MOVING rather than on a reply arriving: a gun whose camera has
            # died still answers '~camblob?' with the same numbers for ever,
            # and a window computed from them is a window about nothing.
            fr = link.last.get("bframes")
            if fr is not None and fr != blob_state.get("frames_n"):
                blob_state["frames_n"] = fr
                blob_state["frames_t"] = time.monotonic()
            focused = None
            try:
                focused = root.focus_get()
            except Exception:
                pass
            for key, gv in bgate.items():
                v = link.last.get(key)
                if v is None:
                    continue
                # Never overwrite a box the user is typing into: the 700 ms
                # sync snapped a half-typed "120" back mid-keystroke, which
                # made the control impossible to set by hand.
                if focused is not None and focused is bspin.get(key):
                    continue
                try:
                    cur = gv.get()
                except Exception:
                    continue            # mid-edit, not a number yet
                if cur != v:
                    gv.set(v)
            # The shape gate follows the gun too, through its own translation:
            # the wire carries 20, the box shows "2.5:1". A value that is not
            # one of the rungs -- a gun set from a serial terminal, or a
            # firmware that means something else by the key -- is left alone
            # rather than snapped to the nearest rung, because snapping would
            # claim a setting nobody chose and the box would then send it.
            for key, (sv, _to_wire, to_shown) in shape_var.items():
                v = link.last.get(key)
                if v is None or v not in to_shown:
                    continue
                if focused is not None and focused is bspin.get(key):
                    continue
                if sv.get() != to_shown[v]:
                    sv.set(to_shown[v])
            # fmt is what the gun holds; ext is the same answer from a gun on
            # older firmware, which knows only "sizes on" and never sends fmt
            # at all. Falling back keeps the control showing the truth on such
            # a gun instead of insisting the detail is off while sizes arrive.
            e = link.last.get("fmt", link.last.get("ext"))
            if e is not None and cam_fmt.get() != e:
                cam_fmt.set(e)
            fr = link.last.get("fullreg")
            # Only the two the gun accepts: anything else came from a build
            # that means something different by the key, and snapping the
            # control to it would claim a setting the radio cannot even send.
            if fr in (85, 5) and cam_fullreg.get() != fr:
                cam_fullreg.set(fr)
            # The gun's own word on the shape capture beats this app's memory
            # of the last click: '~camreset' stops a capture from anywhere,
            # including from pical on the same gun, and a button still reading
            # "Stop learning" is a claim that something is recording when
            # nothing is. Only ever read off a WHOLE reply, so a set that
            # arrived half way through cannot flip it.
            if link.hists.ready() and link.hists.running() != learn_state["on"]:
                learn_state["on"] = link.hists.running()
                btn_learn.config(text=learn_label())
            raw = getattr(link, "blobs", "")
            if raw:
                # The gun appends this trailer in basic mode, where every size
                # comes back as the placeholder -1. Dropped on the floor, the
                # line read as four blobs genuinely measured at size -1 and
                # the gates under it looked broken rather than simply unfed.
                # pical already surfaces it; this panel did not.
                unfed = "(sizes need fmt:1)" in raw
                shown = []
                for b in parse_blobs(raw):
                    # Box and pixel count only exist in full mode, so they are
                    # shown only when the gun actually sent them -- a "box 0x0"
                    # printed in extended mode reads as a measured shape.
                    txt = "%d,%d size %d" % (b[0], b[1], b[2])
                    if len(b) in (7, 9):
                        txt += " box %dx%d %dpx" % (b[4], b[5], b[6])
                    if b[3] != 1:
                        txt += " DROPPED"
                    shown.append(txt)
                blob_lbl.config(
                    text=(("blobs now: " + "   ".join(shown)) if shown
                          else "blobs now: none -- the sensor sees nothing "
                               "at all")
                    + ("   (size -1: nothing measured it -- set blob detail "
                       "to 'sizes')" if unfed else ""),
                    fg=C_FG if shown else C_BAD)
            # Outside the `if raw`, because a gun that has answered nothing is
            # the state the preview most needs to be honest about.
            draw_shapes()
            keys = ("br4", "br3", "br2", "br1", "br0")
            now = [link.last.get(k) for k in keys]
            if None not in now:
                r = blob_state["ref"]
                d = [n - r.get(k, 0) for n, k in zip(now, keys)]
                if any(x < 0 for x in d):
                    # The gun's counters restart at zero on a reboot, and a
                    # negative delta never reaches the threshold below -- the
                    # panel froze on stale numbers for the rest of the session.
                    blob_state["ref"] = dict(zip(keys, now))
                    d = [0] * len(keys)
                tot = sum(d)
                if tot >= 30:
                    # Over EVERY frame, including the ones that saw nothing:
                    # a share taken only over the frames that went well is not
                    # a measure of how much the light is costing.
                    # The drop counters are since-boot too; spliced raw into
                    # a sentence about "the last N frames" they made a window
                    # that had been fixed still read as dropping thousands.
                    # bvalve is one of them and was missed: read raw, a single
                    # give-back at any point in the power cycle left "SIZE
                    # WINDOW TOO TIGHT" on the panel for good, long after the
                    # user had widened the window it was complaining about.
                    d_rej = {}
                    for k in ("brej", "brrej", "bvalve", "bsrej", "bnear",
                              "bfar"):
                        cur = link.last.get(k, 0)
                        prev = blob_state["ref"].get(k, cur)
                        d_rej[k] = max(0, cur - prev)
                        now.append(cur)
                        keys = keys + (k,)
                    blob_state["ref"] = dict(zip(keys, now))
                    blob_state["good"] = d[0] * 10 >= tot * 8
                    hz = blob_rate.hz
                    # Every word here is rationed. This label wraps at the
                    # panel's own width, a third wrapped line costs 14 px, and
                    # the panel has 21 px of slack in the 313 px tab area a
                    # 768p screen allows. Two counters had to go on, so two
                    # things came off to pay for them: the sentence that
                    # spelled out the give-back warning, and the "(we poll at
                    # 420)" aside -- which was only ever a note about a number
                    # this front end does not even poll at. pical's readout is
                    # not wrapped this tight and still carries it.
                    # Measured at the panel's 573 px: this line with the
                    # give-back warning up is 983 px of a 1100 px two-line
                    # budget. The two SHAPE-gate warnings used to ride here as
                    # well and took it to a third line; they are on the gate
                    # line beside the fit buttons now, where the sentence that
                    # says how to switch the gate off fits beside them.
                    blob_state["line"] = (
                        "last %d frames: %d%% saw all four LEDs, %d%% three, "
                        "%d%% two or fewer   %s   dropped %d size, %d odd, "
                        "%d shape"
                        % (tot, 100 * d[0] // tot, 100 * d[1] // tot,
                           100 * (d[2] + d[3] + d[4]) // tot,
                           ("camera %.0f new frames/s" % hz)
                           if hz else "measuring camera rate...",
                           d_rej["brej"], d_rej["brrej"], d_rej["bsrej"])
                        + ("   SIZE WINDOW TOO TIGHT" if d_rej["bvalve"]
                           else ""))
                    # The two shape-gate numbers, over the window the
                    # percentages above were taken over, handed to the gate
                    # line as NUMBERS rather than as sentences. How they read
                    # is not something this loop knows: it depends on which of
                    # the three shape limits is set -- "bhmax:0 turns it off"
                    # is no use to somebody gating on pxmax -- and on whether
                    # the gate can act at all. All the wording lives in
                    # gate_line(), with the width budget it has to fit.
                    #
                    # Neither is a flag on the wire: they are since-boot
                    # totals, and the last thing to read one raw left "SIZE
                    # WINDOW TOO TIGHT" on the panel for the rest of the power
                    # cycle after a single give-back.
                    # Unexplained = refused minus vouched-for. See the note
                    # on GATE_HEAVY_PER_FRAME for the session that showed the
                    # raw count fires on a gate doing exactly its job.
                    unexplained = max(0, d_rej["bsrej"] - d_rej["bfar"]
                                      - d_rej["bnear"])
                    blob_state["gate"] = {"bnear": d_rej["bnear"],
                                          "bsrej": unexplained,
                                          "frames": tot}
                    if d_rej["bnear"] and not blob_state["near_said"]:
                        blob_state["near_said"] = True
                        log("GATE MAY BE TAKING REAL LEDs: %d blob(s) a gate "
                            "dropped sat exactly where the missing corner had "
                            "to be, so they were almost certainly LEDs. This "
                            "is the false-negative meter -- it moves long "
                            "before the cursor starts sticking. Set 'biggest "
                            "blob height' to off (bhmax:0) to take the shape "
                            "gate out of the picture, then widen whichever "
                            "gate you tightened last: the size window, the "
                            "odd-one-out steps, or the two limits under "
                            "Advanced -- and if the gate line says the height "
                            "gate is INERT, it was one of those and not this "
                            "one. Press 'Measure the gate' for a ceiling "
                            "measured on THIS rig instead of a chosen one."
                            % d_rej["bnear"])
                    if (unexplained >= tot * GATE_HEAVY_PER_FRAME
                            and not d_rej["bnear"]
                            and not blob_state["srej_said"]):
                        blob_state["srej_said"] = True
                        log("the shape gate refused %d blobs in %d frames "
                            "that the resolver could not account for as "
                            "stray light (%d more it could). A gate that is "
                            "carrying the room shows up as refusals the "
                            "resolver vouches for; one set for a different "
                            "LED bar refuses the bar itself, the resolver "
                            "cannot lock, and nothing gets vouched for. The "
                            "symptom is a cursor that sticks or jumps. Set "
                            "'biggest blob height' to off (bhmax:0) to prove "
                            "it either way, then press 'Measure the gate'."
                            % (unexplained, tot, d_rej["bfar"]))
                if blob_state["line"]:
                    blob_lbl2.config(
                        text=blob_state["line"],
                        fg=C_OK if blob_state.get("good") else C_WARN)
    root.after(1400, blob_tick)
    # Started after the readout, because the gate line shows whichever of the
    # two warnings that loop raises before it shows any fit -- and reads
    # blob_state to find out.
    root.after(1600, fit_tick)

    board_state = {"cur": None}

    def board_tick():
        b = link.last.get("board")
        if b and b != board_state["cur"]:
            board_state["cur"] = b
            if "wiicam" in b:
                frame_esp.pack_forget()
                # side/expand, not a bare fill="x": the shape preview owns the
                # right of this tab, and a panel packed to the top would sit
                # under it and take the whole width back.
                frame_wii.pack(side="left", fill="both", expand=True)
                # The step list's subtitle for this step names the OV2640's
                # knobs. On this board they are a different three, and a new
                # user reads the subtitle before anything else.
                step_rows[2][1].config(text="   sensitivity, blob detail, "
                                            "learn the gate")
                log("board: %s -- camera tab switched to sensitivity. Press "
                    "Max; 'Save to gun' writes it into the OpenFIRE profile "
                    "with everything else. The blob noise figure above still "
                    "measures on this board: the sensitivity buttons do not "
                    "replace it." % b)
            else:
                frame_wii.pack_forget()
                frame_esp.pack(side="left", fill="both", expand=True)
                step_rows[2][1].config(text="   exposure, threshold, noise "
                                            "floor")
            # The preview is fed from the blob poll, which never runs on a
            # board that has no blob report -- so this is the only moment it
            # can be told which board it is looking at.
            draw_shapes()
        root.after(500, board_tick)
    root.after(500, board_tick)

    def lens_tick():
        # keep the state line honest from the gun's own cam? replies
        bt = link.last.get("beta", None)
        if bt is None:
            tmode_lbl.config(text="?", fg=C_DIM)
        elif bt < 0:
            tmode_lbl.config(text="15 (default)", fg=C_DIM)
        else:
            tmode_lbl.config(text="%d" % bt, fg=C_OK if bt != 15 else C_DIM)
        d = link.last.get("dead", None)
        if d is not None:
            dead_lbl.config(text=("off" if d == 0 else "%d units" % d),
                            fg=(C_DIM if d == 0 else C_OK))
        if not lens_busy["on"] and "lens" in link.last:
            m = link.last.get("lens", 0)
            if m == 0:
                lens_state.config(text="current: correction OFF (stock lens)", fg=C_DIM)
            elif m == 1:
                lens_state.config(text="current: polynomial  k1=%dppm k2=%dppm fpx=%.1f"
                                  % (link.last.get("lk1u", 0), link.last.get("lk2u", 0),
                                     link.last.get("lfpx", 0) / 10.0), fg=C_OK)
            else:
                lens_state.config(text="current: fisheye  feq=%.1f fpx=%.1f"
                                  % (link.last.get("lfeq", 0) / 10.0,
                                     link.last.get("lfpx", 0) / 10.0), fg=C_OK)
        root.after(500, lens_tick)
    root.after(500, lens_tick)

    # ---- pointer toggle ---------------------------------------------------
    def refresh_hid():
        if link.hid_on:
            st_hid.config(text="aim ON   (F9 to freeze)", fg=C_OK)
        else:
            st_hid.config(text="aim FROZEN   (F9 to release)", fg=C_WARN)

    def toggle_hid(_=None):
        if not link.src:
            log("Not connected -- cannot change the pointer.")
            return
        link.pointer(not link.hid_on)
        refresh_hid()
        log("Pointer %s." % ("released -- the gun drives the cursor again"
                             if link.hid_on else
                             "frozen -- the gun stops driving the cursor. "
                             "Steps 4 to 6 release it automatically."))

    # Window-scoped, not system-wide: a global hook would need an extra package
    # and the ability to swallow F9 from every other application, which is a lot
    # of blast radius for a convenience. Focus the window and press F9.
    root.bind("<F9>", toggle_hid)
    root.bind("<KeyPress-F9>", toggle_hid)

    # ---- connection + live tick ------------------------------------------
    def reconnect():
        link.close()
        st_conn.config(text="looking for the gun...", fg=C_WARN)
        root.update_idletasks()
        if link.connect(link.port):
            st_conn.config(text="connected on %s" % link.port, fg=C_OK)
            log("connected on %s" % link.port)
            link.send("~ping")
            link.send("~cam?")
            link.send("~fx?")
            link.send("~aimcal?")
            link.pointer(link.hid_on)      # the gun boots ON; we own this per session
            refresh_hid()
            refresh_hid()
        else:
            st_conn.config(text="no gun found -- replug and Reconnect", fg=C_BAD)
            st_hid.config(text="", fg=C_DIM)
            st_conn.config(text="no gun found", fg=C_BAD)
            log("No gun found. Close the OpenFIRE app if it is open, then Reconnect.")

    tk.Button(head, text="Reconnect", command=reconnect, font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=10).pack(side="right", padx=10)

    def draw_view():
        cv.delete("all")
        W, H = 380, 280
        cv.create_rectangle(2, 2, W-2, H-2, outline="#30363d")
        now = time.time()
        dropping = (now - link.partial_t) < 1.0 and (now - link.full_t) > 0.5
        if not link.hist:
            # say WHY there is nothing to draw: a stream of partial frames
            # is dropouts (background IR, too far, threshold), not silence
            if dropping:
                cv.create_text(W/2, H/2 - 10, text="seeing LEDs, but not all four",
                               fill=C_WARN, font=FB)
                cv.create_text(W/2, H/2 + 12,
                               text="background IR too high, or too far away",
                               fill=C_DIM, font=(F[0], 9))
            else:
                cv.create_text(W/2, H/2, text="no four-LED frames", fill=C_BAD, font=FB)
            return
        if dropping:
            # the view below is the LAST GOOD frame; the banner says it is old
            cv.create_text(W/2, 14, text="LEDs dropping out -- view is the last "
                           "full frame", fill=C_WARN, font=(F[0], 9))
        q = aim_fit.canon(link.hist[-1][1])
        for i, nm in enumerate(("TL", "TR", "BL", "BR")):
            x = 4 + (q[i][0]/FRAME_W)*(W-8)
            y = 4 + (q[i][1]/FRAME_H)*(H-8)
            cv.create_oval(x-4, y-4, x+4, y+4, fill="#ffd24a", outline="")
            cv.create_text(x+10, y-8, text=nm, fill=C_DIM, font=(F[0], 8), anchor="w")
        pts = [(4 + (q[i][0]/FRAME_W)*(W-8), 4 + (q[i][1]/FRAME_H)*(H-8)) for i in range(4)]
        for a, b in ((0,1),(1,3),(3,2),(2,0)):
            cv.create_line(*pts[a], *pts[b], fill="#3a7fbf")
        cv.create_line(W/2-6, H/2, W/2+6, H/2, fill="#e0803a")
        cv.create_line(W/2, H/2-6, W/2, H/2+6, fill="#e0803a")
        # a trail, so instability is visible rather than inferred
        for _, qq in link.hist[-60:]:
            c = qq.mean(0)
            x = 4 + (c[0]/FRAME_W)*(W-8); y = 4 + (c[1]/FRAME_H)*(H-8)
            cv.create_oval(x-1, y-1, x+1, y+1, outline="", fill="#1f6feb")

    link_state = {"dead": False}
    # Said once per session: see the note in the reply pump below.
    learn_cleared = {"said": False}

    def tick():
        # The reader thread dies silently on an unplug; without this check the
        # header said "connected" forever while every send went to a corpse --
        # and a USB soak then blamed the camera for the silence.
        alive = (getattr(link.src, "is_alive", lambda: True)()
                 if link.src else True)
        if link.src and not alive and not link_state["dead"]:
            link_state["dead"] = True
            st_conn.config(text="link LOST -- replug and Reconnect", fg=C_BAD)
            log("serial link lost (unplugged?) -- press Reconnect")
        elif link.src and alive:
            link_state["dead"] = False
        link.pump()
        while link.replies:
            line = link.replies.pop(0)
            log(line)
            # 'CAM: learn cleared -- <reason>' is the gun throwing a capture
            # away because something that changes what the sensor REPORTS
            # changed under it: the sensitivity, or one of the sensor's own
            # size thresholds. The reply names the setting and stops there,
            # which leaves the consequence to be guessed -- so it is said
            # once, in words. Once, because it fires on every step of a
            # spinbox somebody is holding down, and eight of these would
            # push the fit verdict and the save result out of a six-line box.
            if (line.startswith("CAM: learn cleared -- ")
                    and not learn_cleared["said"]):
                learn_cleared["said"] = True
                log("the shape capture was CLEARED by that change: the "
                    "sensor now reports different sizes, and a capture "
                    "spanning both is a measurement of neither. It is still "
                    "running, from empty -- so 'Measure the gate' will say "
                    "NEEDS MORE LED DATA until the bar and the room have "
                    "been swept again.")
        refresh_hid()      # the gun may have released itself via the escape hatch
        for k in CAM_KEYS:
            if k in link.last and sliders[k].get() != link.last[k]:
                sliders[k].set(link.last[k])
        draw_view()
        sg = link.sigma()
        stat_vals["frames"].config(text="%d" % link.frames)
        stat_vals["rate"].config(text="%.0f Hz" % link.fps())
        stat_vals["quad span"].config(text="%.1f px" % link.span())
        if sg is None:
            stat_vals["blob noise"].config(text="hold still to measure", fg=C_DIM)
            stat_vals["verdict"].config(text="-", fg=C_DIM)
        else:
            s_good, s_ok = sigma_gates(link.last.get("board"))
            col = C_OK if sg <= s_good else (C_WARN if sg <= s_ok else C_BAD)
            stat_vals["blob noise"].config(text="%.3f px" % sg, fg=col)
            # turn sigma into the number the user cares about
            err = 11.4 + (sg/0.05 - 1) * 1.2
            stat_vals["verdict"].config(
                text=("good -- ready to calibrate" if sg <= s_good else
                      "usable, could be better" if sg <= s_ok else
                      "too noisy -- tune before calibrating"), fg=col)
        # the calibration step is gated on step 2, and the label says why
        if sg is not None and sg > s_ok:
            step_rows[4][1].config(text="   blocked: blob noise %.2f px is too high" % sg, fg=C_BAD)
        else:
            step_rows[4][1].config(text="   five dots x %d distances" % stance_n.get(),
                                   fg=C_DIM)
        root.after(50, tick)

    log("Lightgun Studio. Step 1 sets up buttons; steps 2-6 are ours.")
    log("The gun keeps driving the cursor. Press F9 to freeze it while you work")
    log("in here; steps 4 to 6 release it on their own and put it back after.")
    log("Order matters: aim accuracy is limited by blob noise, so tune before you")
    log("calibrate. The app blocks step 4 if the noise floor is too high.")
    reconnect()
    root.after(50, tick)
    try:
        root.mainloop()
    finally:
        # unconditional: a traceback in the GUI must not leave the pointer frozen
        try: link.pointer(True, remember=False)
        except Exception: pass
        link.close()


if __name__ == "__main__":
    main()
