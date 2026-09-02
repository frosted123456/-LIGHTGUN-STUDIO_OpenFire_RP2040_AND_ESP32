#!/usr/bin/env python3
"""pical -- the Lightgun Studio steps that do not need Windows, on the TV.

Studio's step 1 is the OpenFIRE app (buttons and pins); that one stays on a
PC. Everything after it is here:

    2 camera tuning   3 lens / FOV   4 aim calibration
    5 fine tune       6 verify

Nothing is reimplemented. The serial link, the capture session, the fits and
the tuning maths all come from tools/ -- this file is the pygame front end
over them, so a change to the maths reaches Studio and pical together.

The gun is aimed by its IRON SIGHTS and read over serial, so a calibration
that is completely offset never prevents recalibrating. Menus take a game
controller, a mouse or a keyboard.
"""
import math
import os
import re
import sys
import textwrap
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(HERE, "..", "tools"), os.path.join(HERE, "tools")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import pygame

import aim_fit
import calib_lens
from aim_calib import (CaptureSession, aimcal_line, camsave_verified,
                       install_over_serial, make_plan, FRAME_W, FRAME_H)
from aim_finetune import (BETA_MAX, LEAD_MAX, LEAD_STEP, SHOT_FRAMES,
                          SMOOTH_MAX, TARGET, Tuner)
from aim_verify import GRID_3x3
import gun_studio
from gun_studio import (BlobLog, CAM_KEYS, CAM_RANGE, FrameRate, LENS_KEYS,
                        Link, QUIET_REARM_S, auto_tune, parse_blobs,
                        quiet_plan, sigma_gates, write_shape_csv)

# The shared serial layer lives in tools/ beside this file, and this file is
# routinely copied onto the stick ON ITS OWN. When only pical.py is updated,
# every feature whose parsing lives in tools/ stops working -- silently, in a
# way that looks exactly like a broken screen. So it is checked, out loud.
LINK_API_NEEDED = 10
LINK_API_OK = getattr(gun_studio, "LINK_API", 0) >= LINK_API_NEEDED

OUT_DIR = os.environ.get("PICAL_OUT", os.path.join(HERE, "calib_out"))
NO_DATA_S = 5.0
LENS_SWEEP_S = 20.0
VERIFY_FRAMES = 12

# The two gates a lens sweep has to pass, mirrored here so the screen can draw
# them while the sweep is running instead of only reporting them afterwards.
# hostcheck/pical_render_test.py asserts these still match calib_lens.
COV_GATE = 0.55           # fraction of the frame half-extent the LEDs must reach
SPAN_GATE = 25.0          # px, smallest quad that carries usable distortion
CX, CY = calib_lens.CX, calib_lens.CY

# The sensor's OWN pixel array, which is NOT the space a blob's position is
# reported in. A full-mode blob carries both at once: x,y have already been
# scaled into the pipeline's 240x176 by the gun (wiicam_aim.cpp, x = nx * SX),
# while the bounding box -- w, h and the origin xmn,ymn -- is left in the
# 7-bit numbers the sensor produced. Drawing them together means converting
# one of them, and the panel converts the POSITION, because the box is the
# thing under suspicion and a suspect measurement should not set the scale.
SENSOR_W, SENSOR_H = 128.0, 96.0

# How far a box centre may sit from its own reported position before the shape
# panel calls it a disagreement, in sensor pixels. A real blob is lopsided, so
# its centroid and the middle of its bounding box differ by a fraction of a
# pixel honestly. Two pixels is far more than that, and far less than the
# factor of nearly two that reading the box in the wrong array would produce.
BOX_MATCH_PX = 2.0

C_BG = (10, 12, 16)
C_FG = (230, 237, 243)
C_DIM = (125, 133, 144)
C_OK = (57, 194, 110)
C_WARN = (216, 161, 58)
C_BAD = (210, 75, 75)
C_RING = (224, 122, 95)
C_SEL = (31, 111, 235)


def wrap(text, width=72):
    """A refusal reason is a sentence, not a caption: show all of it."""
    return textwrap.wrap(str(text), width) or [""]


# ---------------------------------------------------------------------------
# naming what gets recorded onto the stick
# ---------------------------------------------------------------------------
def next_recording(out_dir):
    """The number the next file written to the stick should carry.

    The Pi has NO real-time clock. Every boot starts its clock at the same
    value, so a capture taken today and one taken next week are both stamped
    with the same second -- and a name built from the clock alone SILENTLY
    replaced the earlier one. The recordings from the TV are the whole reason
    these files exist, and losing one to a name collision leaves nothing
    behind to show it ever happened.

    A number scanned from what is already there only ever counts up, so the
    files also sort in the order they were made, and it is short enough to say
    down a phone: "send me number 4".
    """
    n = 0
    try:
        for f in os.listdir(out_dir):
            # Only our own numbered names. A stick that has been used before
            # still holds files stamped '<name>-20260901-013038.<ext>' from
            # the versions that named recordings from the clock, and read as a
            # sequence number that date would make the next recording number
            # 20,260,902. The digit count is BOUNDED for exactly that reason:
            # at five it cannot reach the eight a date needs, so a datestamped
            # name never matches at any length.
            m = re.match(r"[a-z]+-(\d{1,5})-", f)
            if m:
                n = max(n, int(m.group(1)))
    except OSError:
        # An unreadable OUT_DIR is not a reason to refuse to record. The
        # collision check in recording_path still stops anything being
        # overwritten, which is the part that matters.
        pass
    return n + 1


def recording_path(prefix, ext=".csv"):
    """A path on the stick that is certainly free, and the number in it.

    The clock time stays in the name because it still orders files within one
    boot and costs nothing; the NUMBER is what makes the name unique. Every
    candidate is checked against the directory as well, because with no clock
    and a directory that may have failed to list, "highest + 1" is a good
    guess rather than a proof -- and overwriting a capture taken at the TV is
    the one failure this cannot have.

    For the recordings THIS file writes: the blob logs, the shape CSVs and the
    lens sweeps. A calibration's own two files are numbered by
    CaptureSession.save() in the same scheme, and must not be numbered again
    here -- see finish_calib.
    """
    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)
    n = next_recording(OUT_DIR)
    for k in range(1000):
        path = os.path.join(OUT_DIR, "%s-%03d-%s%s"
                            % (prefix, n + k, time.strftime("%H%M%S"), ext))
        if not os.path.exists(path):
            return path, n + k
    # Both callers already turn an exception here into a toast. A thousand
    # taken names in a row is a stick that needs looking at, not something to
    # paper over by picking a name that would destroy one of them.
    raise IOError("no free name for a %s file in %s" % (prefix, OUT_DIR))


# ---------------------------------------------------------------------------
# what a blob actually looked like, in the sensor's own pixels
# ---------------------------------------------------------------------------
def blob_shape(b):
    """One parsed blob, converted into the sensor's own 128x96 pixels.

    Hands back (x, y, box, px, dens, placed):
      x, y    the REPORTED position, scaled out of the pipeline's 240x176
      box     (x0, y0, w, h) of the bounding box, in sensor pixels, or None
      px      the blob's lit-pixel count, or None outside full mode
      dens    px / box area -- how full the box is -- or None
      placed  True when the box's own origin came off the wire (nine fields),
              False when there was none and the box had to be hung on the
              position instead (seven fields, the previous firmware)

    Sizes are w+1 by h+1 on purpose: the gun sends xmx-xmn, a DIFFERENCE, so a
    blob one pixel across reports 0 and a box drawn 0 wide is an LED that
    vanishes off the panel entirely.
    """
    x = b[0] * SENSOR_W / FRAME_W
    y = b[1] * SENSOR_H / FRAME_H
    if len(b) < 7:
        return x, y, None, None, None, False
    w = max(1, b[4] + 1)
    h = max(1, b[5] + 1)
    px = b[6]
    dens = px / float(w * h)
    if len(b) >= 9:
        return x, y, (float(b[7]), float(b[8]), w, h), px, dens, True
    return x, y, (x - w / 2.0, y - h / 2.0, w, h), px, dens, False


def box_position_gap(blobs):
    """Worst distance between a box centre and its own reported position.

    In sensor pixels, or None when no blob carried a box origin to compare.
    This is the one number that says whether the box fields mean what this
    panel draws them as: the position and the box come from different parts of
    the same report, so if the box really is in the sensor's 128x96 array the
    two land on top of each other. If they do not, the units are wrong and
    every shape gate on this screen is judging a number nobody understands.
    """
    worst = None
    for b in blobs:
        x, y, box, _px, _dens, placed = blob_shape(b)
        if box is None or not placed:
            continue
        g = math.hypot(box[0] + box[2] / 2.0 - x, box[1] + box[3] / 2.0 - y)
        worst = g if worst is None else max(worst, g)
    return worst


# ---------------------------------------------------------------------------
# the gun's own verdict on the shape gate
# ---------------------------------------------------------------------------
# What '~camfit' wants before it will name a ceiling, mirrored here so a
# progress bar can be drawn while the data is still coming in rather than only
# after the gun refuses. Whatever the gun's own reply says wins over these:
# they are only the starting point for a bar that has to be drawn before the
# first answer arrives, and a firmware that moves its floor must not leave the
# bar measuring against a number nothing on the gun believes any more.
LED_WANT = 500
STRAY_WANT = 20

# When the shape gate counts as "refusing heavily": UNEXPLAINED rejections per
# camera frame, where unexplained means the resolver did not confirm the blob
# as stray light. Measured, not chosen -- see Camera.gate_lines for the session
# that set it. Shared with Studio's own warning, which used to fire at 0.25 of
# raw rejections while this one waited for 2.0, an eight-fold disagreement
# about the same number.
GATE_HEAVY_PER_FRAME = 1.0

# The fewest camera frames a gate warning will be drawn from. A rate measured
# over twenty frames is a tenth of a second of camera and swings wildly, and
# this readout has about seven rows: a line that flickers in and out of them
# costs a measurement somebody was reading. Shared by both gate warnings so
# they cannot come and go on different evidence, and the same floor Studio
# takes its own window over.
GATE_WINDOW_FRAMES = 30

# How long a borrow waits for the gun to say whether the shape capture is
# already running before it arms one anyway. A real answer is thirteen lines
# and up to 2.6 KB, so this is a wait for a reply to cross the wire rather
# than for a round trip -- and a gun with no capture at all never answers,
# which is why it has to end.
CAM_ARM_S = 1.5


class FitReport:
    """The last '~camfit' answer, held whole.

    The gun always sends the counters first -- 'CAM: fit ledn=... straym=...'
    -- and then exactly one outcome line. Assembled here rather than read off
    link.last because the two halves mean nothing apart: the counters say how
    far the measurement has got, the outcome says what it came to, and a
    screen that showed one without the other would either report progress
    towards a verdict that already exists or a verdict with nothing behind it.

    Everything degrades to "we do not know". A gun on older firmware never
    answers '~camfit' at all, so `verdict` stays None for ever -- and every
    caller here treats None as "this gun cannot measure it", never as "no safe
    gate" and never as a number.
    """

    def __init__(self):
        self.reset()

    def reset(self, new_gun=True):
        """Forget it all. Used on a reconnect: a verdict is a measurement of
        one gun in one room, and the gun that comes back may be another.

        `new_gun=False` keeps one thing: whether the gun HAS the command.
        Clearing the histograms -- which is what starting a capture does --
        throws away the measurement but changes nothing about the firmware
        underneath, and forgetting that there would set every screen asking
        an old gun all over again. That is the log-flooding the flag exists
        to stop, and it would come back on the one screen most likely to
        clear the histograms."""
        keep = True if new_gun else self.supported
        self.ledn = self.stray_n = 0
        self.led_h = self.stray_h = None
        self.led_want, self.stray_want = LED_WANT, STRAY_WANT
        self.verdict = None      # 'need_led' / 'need_stray' / 'no_gate' / 'gate'
        self.bhmax = None
        self.tight = False
        self.stored = None       # (LED rows, stray rows) from an earlier capture
        self.stored_px = None    # ...and the pixel envelope beside them
        self.ignored = None      # (count, height reached, height of the rest)
                                 # for LED samples the gun set aside as stray
        self.applied = None      # 'saved' / 'unsaved' once the gun has set it
        self.switched = False    # apply put the gun into full mode itself
        self.inert = False       # an OLDER firmware saying the gate cannot act
        self.seq = 0             # whole answers seen, so a caller that has
                                 # just asked can tell a fresh one from the
                                 # one that was already on screen
        self.t = 0.0             # monotonic, for the same reason
        # Whether this gun has the command at all. True until it says
        # otherwise, because a gun that has simply not been asked yet is not
        # the same thing as one that cannot answer.
        self.supported = keep

    # The counters always arrive as a k=v run; the outcomes are sentences with
    # numbers in them. Pulled out by name and by position respectively, and
    # never by a blanket k=v sweep of every 'CAM: fit' line: the STORED line
    # carries ledmaxh= and strayminh= of its own, from an OLD capture, and a
    # sweep would quietly overwrite the live measurement with them.
    _NUM = re.compile(r"-?\d+")

    def _ints(self, s):
        return [int(v) for v in self._NUM.findall(s)]

    def feed(self, line):
        """Take one 'CAM: fit' line. True if it was ours."""
        if "unknown command" in line and "camfit" in line:
            # Firmware older than the fit answers every ask with a refusal.
            # Recorded so the screens stop asking: at one poll every couple of
            # seconds that is a thousand lines an hour into a 200-line log
            # ring, and the log is exactly where a user is sent to read the
            # diag verdict and the result of a save.
            self.supported = False
            return True
        if not line.startswith("CAM: fit"):
            return False
        self.supported = True
        rest = line[8:].strip()
        if rest.startswith("ledn="):
            # A fresh reading starts here, so the previous outcome goes with
            # it. Left standing, a gun that had dropped back to NEEDS MORE
            # LED DATA -- which is what a power cycle does -- would go on
            # showing the ceiling from before it, and that ceiling is the one
            # thing on this screen nobody may guess at.
            self.verdict, self.bhmax, self.applied = None, None, None
            self.tight = self.inert = self.switched = False
            # The provenance and the contamination note go with it. Both are
            # sent BELOW this line when the gun has them, so a reading that
            # does not carry them is a gun that no longer has them -- which
            # '~camreset' now produces by wiping the stored pair. Kept, they
            # would go on showing an erased measurement as this gun's own.
            self.stored = self.stored_px = self.ignored = None
            got = {}
            for tok in rest.split():
                k, sep, v = tok.partition("=")
                if sep and k in ("ledn", "ledmaxh", "straym", "strayminh"):
                    try:
                        got[k] = int(v)
                    except ValueError:
                        pass
            self.ledn = got.get("ledn", 0)
            self.stray_n = got.get("straym", 0)
            # -1 is the firmware's "nothing measured", not a height of minus
            # one row. Kept as None so no caller can print it as a number.
            self.led_h = got.get("ledmaxh") if got.get("ledmaxh", -1) >= 0 else None
            self.stray_h = (got.get("strayminh")
                            if got.get("strayminh", -1) >= 0 else None)
            return True
        if "LED samples ignored" in rest:
            # Samples the gun set aside before working out the ceiling. It
            # starts with a DIGIT after 'CAM: fit ', so it matched no branch
            # here and reached the log only -- and the log is not where
            # anybody reads it. The whole reason the firmware says it out loud
            # is that a user who sees "32 ignored at 31 rows" understands
            # their rig, where one who sees a clean 7 never learns the sun
            # spent the capture in the LED class.
            n = self._ints(rest)
            if len(n) >= 3:
                self.ignored = (n[0], n[1], n[2])
            return True
        if rest.startswith("switched to fmt:"):
            # 'apply' now puts the gun into full mode itself and persists
            # THAT, rather than whatever format happened to be live. Worth
            # carrying: it is a change to the gun the user did not ask for by
            # name, and the screen that asked for the apply should say so.
            self.switched = True
            return True
        if rest.startswith("STORED"):
            # By NAME, not by position. This line gained a third measurement
            # -- the pixel envelope -- after it was first parsed here, and a
            # positional reader would have kept working by luck and then
            # silently mis-read the next field added to it. The height pair is
            # what this screen shows, because height is the gate it can set;
            # the pixel figure is carried so a later screen has it rather than
            # having to ask the gun again.
            got = {}
            for tok in rest.split():
                k, sep, v = tok.partition("=")
                if sep and k in ("ledmaxh", "strayminh", "ledmaxpx"):
                    try:
                        got[k] = int(v)
                    except ValueError:
                        pass
            if "ledmaxh" in got and "strayminh" in got:
                self.stored = (got["ledmaxh"], got["strayminh"])
                self.stored_px = got.get("ledmaxpx")
            return True
        if rest.startswith("INERT"):
            # The gun accepted the ceiling and then said it cannot act on it:
            # the shape gate compares box heights and this gun is not
            # reporting boxes. NOT an outcome -- it arrives after one, in the
            # same answer -- so nothing is published and the verdict it
            # qualifies is left standing.
            self.inert = True
            return True
        if rest.startswith("NEEDS MORE LED DATA"):
            n = self._ints(rest)
            if len(n) >= 2:
                self.ledn, self.led_want = n[0], n[1]
            self.verdict = "need_led"
        elif rest.startswith("NO STRAY DATA"):
            n = self._ints(rest)
            if len(n) >= 2:
                self.stray_n, self.stray_want = n[0], n[1]
            if len(n) >= 3:
                self.led_h = n[2]
            self.verdict = "need_stray"
        elif rest.startswith("NO SAFE GATE"):
            n = self._ints(rest)
            if len(n) >= 2:
                self.led_h, self.stray_h = n[0], n[1]
            self.verdict = "no_gate"
        elif rest.startswith("bhmax="):
            n = self._ints(rest)
            if n:
                self.bhmax = n[0]
            if len(n) >= 3:
                self.led_h, self.stray_h = n[1], n[2]
            self.tight = "TIGHT" in rest
            self.verdict = "gate"
        elif rest.startswith("applied"):
            # The save is reported separately from the setting, because they
            # fail separately: a gate that took but did not reach flash is
            # gone on the next power cycle and nothing else would say so.
            self.applied = "unsaved" if "SAVE FAILED" in rest else "saved"
            return True
        else:
            # 'not applied -- send camfit=apply', and anything a later
            # firmware adds. Not an outcome, so nothing is published: the
            # screen keeps the verdict it already has rather than being
            # blanked by a line it did not understand.
            return True
        self.seq += 1
        self.t = time.monotonic()
        return True

    def stored_words(self):
        """The gun's own record of an earlier capture, in words, or None.

        A stray height of 0 means NO camfit has ever applied on this gun:
        '~camsave' records the LED edge on its own and leaves the stray side
        at zero. Printed as "stray at 0" that reads as a measured room with no
        light in it, which is the opposite of what it says -- and 0 is the one
        value that cannot be a measured height, since the gate rejects only
        what is TALLER than the ceiling.
        """
        if not self.stored:
            return None
        led, stray = self.stored
        return ("LEDs %d rows, room light %s"
                % (led, "not recorded" if not stray else "%d rows" % stray))

    def ignored_words(self):
        """The samples the gun set aside, in words, or None."""
        if not self.ignored:
            return None
        n, high, rest = self.ignored
        return ("%d LED samples reached %d rows against the %d the rest stop "
                "at" % (n, high, rest))

    def enough(self):
        """Has the gun got as far as an answer about the gate itself?"""
        return self.verdict in ("no_gate", "gate")

    def progress(self):
        """(LED fraction, stray fraction) of what the gun says it wants."""
        return (min(1.0, self.ledn / float(max(1, self.led_want))),
                min(1.0, self.stray_n / float(max(1, self.stray_want))))


class Meter:
    """A since-boot counter read as a RATE, over a window of recent replies.

    Every counter on the '~camblob?' line counts from power-on. Read raw, a
    warning built on one can never clear: a single event at any point in the
    session pins it on screen for the rest of the power cycle, which is how a
    'SIZE WINDOW TOO TIGHT' line came to sit permanently over a window the
    user had already widened.

    A one-reply delta clears, and that is what the drop counts use -- but it
    is too short to answer "is this gate refusing HEAVILY?", which is a
    question about a sustained share rather than about one second. So samples
    are kept for a few seconds and the climb is measured across them.

    Samples are keyed on the gun's own frame count, not on being called. This
    is read from draw() at about 60 fps while a reply lands roughly once a
    second: appending per call would fill the window with sixty copies of the
    same reading and the "window" would be a sixtieth of a second long.
    """

    def __init__(self, window_s=8.0):
        self.window = window_s
        self._s = []              # (monotonic, gun frames, {key: value})

    def _prune(self, now):
        # By TIME, and with no floor on how few samples may be left. A gun
        # that has stopped answering must let its warnings expire: keeping the
        # last two samples alive for ever would hold the final climb on screen
        # long after there was anything behind it.
        while self._s and now - self._s[0][0] > self.window:
            self._s.pop(0)

    def feed(self, frames, values):
        """One gun reply's worth of counters, if it really is a new one."""
        if not isinstance(frames, (int, float)):
            return
        now = time.monotonic()
        self._prune(now)
        if self._s and frames == self._s[-1][1]:
            return
        if self._s and frames < self._s[-1][1]:
            # A gun that rebooted restarts every counter at zero, and a
            # negative climb read as a rate is a warning that can never come
            # back. Start again rather than reporting nonsense.
            self._s = []
        self._s.append((now, frames, dict(values)))

    def span(self):
        """(frames, seconds) the window currently covers -- 0 when it holds
        fewer than two samples and nothing can be measured yet."""
        self._prune(time.monotonic())
        if len(self._s) < 2:
            return 0, 0.0
        return self._s[-1][1] - self._s[0][1], self._s[-1][0] - self._s[0][0]

    def climb(self, key):
        """How far `key` has moved across the window. Never negative."""
        self._prune(time.monotonic())
        if len(self._s) < 2:
            return 0
        a, b = self._s[0][2].get(key), self._s[-1][2].get(key)
        if not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
            return 0
        return max(0, b - a)

    def per_frame(self, key):
        """That climb per camera frame, which is the only fair denominator:
        the sensor hands over at most four blobs a frame, so 'two rejected per
        frame' is a share anyone can judge where 'nine hundred rejected' is
        just a number that grows."""
        frames, _s = self.span()
        return self.climb(key) / float(frames) if frames > 0 else 0.0


def heat(t):
    """A density as a colour: cold blue for a box that is mostly empty,
    bright amber for one that is solid.

    A ramp rather than a number, because the boxes are 30 px across on the
    Pi's screen and nobody reads a number that size from a sofa -- but "that
    one is the wrong colour" carries across a room.

    The hot end stops at amber and never reaches white on purpose. The
    crosshair drawn on top of it is white, and a solid blob -- the very case
    this panel is used to look at -- would otherwise swallow the one mark the
    whole measurement depends on.
    """
    try:
        t = max(0.0, min(1.0, float(t)))
    except (TypeError, ValueError):
        return (60, 66, 74)
    stops = ((0.0, (28, 52, 104)), (0.55, (176, 96, 40)),
             (1.0, (236, 176, 64)))
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return tuple(int(a + (b - a) * f) for a, b in zip(c0, c1))
    return stops[-1][1]


# Where a settings screen draws the selected row's hint, as a fraction of the
# screen height, and how far apart the readout above it stacks its lines. Both
# are named rather than typed twice: the camera screen has to work out how
# many readout rows fit BEFORE the hint, and a screen that computes the fit
# from one number while the hint is drawn at another is a screen that overlaps
# them -- which is what a hard-coded slice of six did the day a row was added
# to the list above it.
TIP_Y = 0.90
READOUT_STEP = 1.25


def cols_for(font, px):
    """How many characters of `font` fit across `px` pixels.

    Measured off the face rather than assumed, and it has to be: the faces are
    sized from the screen's HEIGHT while the room a block of text has is a
    fraction of its WIDTH, so the same sentence that sits comfortably on the
    Pi's 4:3 panel runs off the side of a short 16:9 one. Text here is drawn
    CENTRED with no wrapping of its own, so an over-long line does not clip --
    it loses a word off each end, and nothing on screen says it happened.
    """
    sample = "abcdefghijklmnopqrstuvwxyz 0123456789 ,.:-"
    try:
        avg = font.size(sample)[0] / float(len(sample))
        if avg > 0:
            return max(20, int(px / avg))
    except Exception:
        pass
    return 40


def readout_cols(sc):
    """How many characters of the small readout face fit across the screen.

    The camera readout used to wrap at a flat 96 columns whatever the screen
    was, which on the Pi's 1024x768 filled 686 px and left 338 px unused --
    and every column it did not use cost a wrapped row of a readout that is
    already fighting for room. 0.90 of the width, because the lines are
    CENTRED and the widest of them is not known until it is built.
    """
    return max(60, cols_for(sc.f_xs, sc.w * 0.90))


# ---------------------------------------------------------------------------
# drawing
# ---------------------------------------------------------------------------
class Screen:
    """Surface, fonts and the primitives every view uses."""

    def __init__(self, surf):
        self.s = surf
        self.w, self.h = surf.get_size()
        u = max(11, int(self.h / 58))
        self.f_xs = pygame.font.Font(None, int(u * 1.7))
        self.f_s = pygame.font.Font(None, u * 2)
        self.f_m = pygame.font.Font(None, int(u * 2.7))
        self.f_l = pygame.font.Font(None, int(u * 4.0))
        self.f_xl = pygame.font.Font(None, int(u * 5.6))

    def text(self, x, y, msg, font=None, colour=C_FG, centre=True):
        img = (font or self.f_m).render(msg, True, colour)
        r = img.get_rect()
        if centre:
            r.center = (int(x), int(y))
        else:
            r.midleft = (int(x), int(y))
        self.s.blit(img, r)
        return r

    def lines(self, x, y, msgs, font=None, colour=C_DIM, step=1.5):
        font = font or self.f_s
        dy = int(font.get_height() * step)
        for m in msgs:
            self.text(x, y, m, font, colour)
            y += dy
        return y

    def ring(self, x, y, r, colour, width=3):
        pygame.draw.circle(self.s, colour, (int(x), int(y)), int(r), width)

    def arc(self, x, y, r, frac, colour, width=6):
        if frac <= 0.0:
            return
        rect = pygame.Rect(int(x - r), int(y - r), int(2 * r), int(2 * r))
        start = math.pi / 2
        pygame.draw.arc(self.s, colour, rect,
                        start - 2 * math.pi * min(1.0, frac), start, width)

    def bar(self, x, y, w, h, frac, colour):
        pygame.draw.rect(self.s, (40, 46, 54), (int(x), int(y), int(w), int(h)), 1)
        if frac > 0:
            pygame.draw.rect(self.s, colour, (int(x) + 1, int(y) + 1,
                                              int((w - 2) * min(1.0, frac)),
                                              int(h - 2)))

    def crosshair(self, x, y, r, colour, dot=True):
        self.ring(x, y, r, colour, 3)
        pygame.draw.line(self.s, colour, (x - r * 1.9, y), (x - r * 0.4, y), 2)
        pygame.draw.line(self.s, colour, (x + r * 0.4, y), (x + r * 1.9, y), 2)
        pygame.draw.line(self.s, colour, (x, y - r * 1.9), (x, y - r * 0.4), 2)
        pygame.draw.line(self.s, colour, (x, y + r * 0.4), (x, y + r * 1.9), 2)
        if dot:
            pygame.draw.circle(self.s, colour, (int(x), int(y)),
                               max(2, int(r * 0.12)))


def install_cursor(screen_h):
    """Hand our crosshair to the system cursor. True if it took.

    The point is latency, not looks: a system cursor is moved by the compositor
    from the HID report, so it does not wait for this app's frame. The artwork is
    the same crosshair the Pi path blits, so nothing is lost by using it -- the
    stock arrow being hard to see on a dark screen was the reason for drawing one
    in the first place.

    Kept small: a hardware cursor plane is commonly capped at 64x64. The
    surface is n = 2*int(1.9r)+3 across, so r must stay at or under 16 -- at
    17 the surface is 67 px and the kmsdrm backend REFUSES it, which silently
    turns the whole hardware-cursor mode off on the one platform it is for.
    """
    r = int(max(8, min(16, screen_h * 0.022)))
    n = 2 * int(r * 1.9) + 3
    try:
        surf = pygame.Surface((n, n), pygame.SRCALPHA)
        c = (n - 1) // 2
        pygame.draw.circle(surf, C_RING, (c, c), r, 3)
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            pygame.draw.line(surf, C_RING,
                             (c + dx * r * 1.9, c + dy * r * 1.9),
                             (c + dx * r * 0.4, c + dy * r * 0.4), 2)
        pygame.draw.circle(surf, C_RING, (c, c), max(2, int(r * 0.12)))
        pygame.mouse.set_cursor(pygame.cursors.Cursor((c, c), surf))
    except Exception:
        # No colour-cursor support on this platform or this pygame: fall back to
        # blitting, which always works.
        return False
    return True


# ---------------------------------------------------------------------------
# input: keyboard, mouse and any controller reduced to one set of actions
# ---------------------------------------------------------------------------
class Input:
    AX_DEAD = 0.55
    REPEAT_S = 0.22

    def __init__(self):
        pygame.joystick.init()
        self.pads = []
        self.rescan()
        self._t = 0.0
        self._last = (0, 0)

    def rescan(self):
        for p in self.pads:
            try:
                p.quit()
            except Exception:
                pass
        self.pads = []
        for i in range(pygame.joystick.get_count()):
            try:
                j = pygame.joystick.Joystick(i)
                j.init()
                self.pads.append(j)
            except Exception:
                pass
        return len(self.pads)

    def actions(self, events, now):
        out = []
        for e in events:
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_UP:
                    out.append("up")
                elif e.key == pygame.K_DOWN:
                    out.append("down")
                elif e.key == pygame.K_LEFT:
                    out.append("left")
                elif e.key == pygame.K_RIGHT:
                    out.append("right")
                elif e.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_SPACE):
                    out.append("select")
                elif e.key == pygame.K_ESCAPE:
                    out.append("back")
                elif e.key == pygame.K_t:
                    out.append("trigger")
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                out.append("click")
            elif e.type == pygame.JOYBUTTONDOWN:
                out.append("back" if e.button in (1, 6) else "select")
            elif e.type == pygame.JOYHATMOTION:
                if e.value[1] > 0:
                    out.append("up")
                elif e.value[1] < 0:
                    out.append("down")
                if e.value[0] < 0:
                    out.append("left")
                elif e.value[0] > 0:
                    out.append("right")
            elif e.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                self.rescan()
        vx = vy = 0
        for p in self.pads:
            try:
                if p.get_numaxes() > 1:
                    ax, ay = p.get_axis(0), p.get_axis(1)
                    if ax > self.AX_DEAD:
                        vx = 1
                    elif ax < -self.AX_DEAD:
                        vx = -1
                    if ay > self.AX_DEAD:
                        vy = 1
                    elif ay < -self.AX_DEAD:
                        vy = -1
            except Exception:
                continue
        if (vx, vy) == (0, 0):
            self._last = (0, 0)
        elif (vx, vy) != self._last or (now - self._t) > self.REPEAT_S:
            self._t = now
            self._last = (vx, vy)
            if vy > 0:
                out.append("down")
            elif vy < 0:
                out.append("up")
            if vx > 0:
                out.append("right")
            elif vx < 0:
                out.append("left")
        return out


# ---------------------------------------------------------------------------
# a focusable row of controls, so one model serves every settings screen
# ---------------------------------------------------------------------------
class Row:
    """One line on a settings screen.

    kind 'button' fires act(); kind 'spin' adjusts a value with left/right and
    reads it back live from the gun, so the screen never shows a value the
    gun does not actually hold.

    `fmt` may be a callable as well as a %-format, for a setting whose wire
    value is not the number a person thinks in. The shape gate's aspect limit
    is one: it travels as EIGHTHS, so the gun answers 20 and the reader on the
    sofa needs '2.5:1'. A %-format cannot divide, and a screen with no console
    beside it gives nobody any way to work out what 20 meant.
    """

    def __init__(self, label, kind="button", act=None, get=None, set=None,
                 lo=0, hi=100, step=1, fmt="%d", hint="", vals=None):
        self.label = label
        self.kind = kind
        self.act = act
        self.get = get
        self.set = set
        self.lo, self.hi, self.step = lo, hi, step
        self.fmt = fmt
        self.hint = hint
        # An explicit ladder of values, for settings where a linear step lands
        # somewhere harmful. The sensor's MAXSIZE is one: stepping by 5 from
        # -1 reaches 4, a value low enough to blind the camera, and the useful
        # range starts around 100.
        self.vals = tuple(vals) if vals else None
        self.rect = None

    def value(self):
        try:
            return self.get() if self.get else None
        except Exception:
            return None

    def tip(self):
        """A hint may be a callable, so a row can explain what it will do to
        the state the gun is in right now rather than in general."""
        try:
            return self.hint() if callable(self.hint) else self.hint
        except Exception:
            return ""

    def show(self, v):
        """The value as the reader sees it.

        Everything is caught: a fmt callable runs on whatever the gun last
        said, including a number from a firmware that means something else by
        the key, and pical is fullscreen on a Pi with no console -- a
        ZeroDivisionError inside a draw is a black screen, not a bad label."""
        if v is None:
            return "--"
        try:
            return self.fmt(v) if callable(self.fmt) else (self.fmt % v)
        except Exception:
            return str(v)

    def nudge(self, d):
        if self.kind != "spin" or self.set is None:
            return
        v = self.value()
        # Anything that will not do arithmetic is treated as no reading at
        # all. Every value on the wire arrives through an int(), so this
        # should be impossible -- but this runs off a key press on a screen
        # with no console, and an arrow that raises takes the whole front end
        # down with it. "Start from the end you are stepping away from" is
        # already the answer for a gun that has not replied yet.
        if v is not None and not isinstance(v, (int, float)):
            v = None
        if self.vals:
            # Move along the ladder from wherever the gun actually is. With no
            # reading yet, start from the end the user is stepping away from
            # rather than inventing a value in the middle.
            if v in self.vals:
                i = self.vals.index(v)
            elif v is None:
                i = 0 if d > 0 else len(self.vals) - 1
            else:
                # nearest rung, so an odd value from the gun still steps sanely
                i = min(range(len(self.vals)),
                        key=lambda k: abs(self.vals[k] - v))
            i = max(0, min(len(self.vals) - 1, i + d))
            self.set(self.vals[i])
            return
        if v is None:
            v = self.lo
        v = max(self.lo, min(self.hi, v + d * self.step))
        self.set(v)


class RowScreen:
    """Shared behaviour for the settings-style views."""

    title = ""
    subtitle = ""
    preview = False           # show the sensor view beside the rows

    def __init__(self, app):
        self.app = app
        self.sel = 0
        self.rows = []

    def handle(self, acts, mouse):
        # Nothing here may raise: pical runs fullscreen on a Pi with no console,
        # so an IndexError is a black screen with no explanation. sel is only
        # wrapped by the up/down arithmetic, and any rebuild that shortens the
        # list can leave it past the end.
        if self.rows:
            self.sel = max(0, min(self.sel, len(self.rows) - 1))
        for a in acts:
            if not self.rows and a != "back":
                continue
            if a == "up":
                self.sel = (self.sel - 1) % max(1, len(self.rows))
            elif a == "down":
                self.sel = (self.sel + 1) % max(1, len(self.rows))
            elif a == "left":
                self.rows[self.sel].nudge(-1)
            elif a == "right":
                self.rows[self.sel].nudge(+1)
            elif a == "select":
                r = self.rows[self.sel]
                if r.kind == "button" and r.act:
                    r.act()
            elif a == "back":
                self.app.to_menu()
            elif a == "click":
                for i, r in enumerate(self.rows):
                    if r.rect and r.rect.collidepoint(mouse):
                        self.sel = i
                        if r.kind == "button" and r.act:
                            r.act()
                        break

    def draw_rows(self, sc, y0, y1=None):
        # With the camera view on the right the rows move into a narrower
        # left-hand column; without it they keep the full width.
        bx, bw = (0.04, 0.58) if self.preview else (0.14, 0.72)
        lx, vx = bx + 0.03, bx + bw - 0.09
        y = y0
        # Pitch shrinks to fit y1 when there are many rows, so adding one never
        # pushes the last row over whatever the screen draws below the list.
        dy = sc.h * 0.072
        n = max(1, len(self.rows))
        if y1 is not None and n > 1:
            dy = min(dy, (y1 - y0) / float(n - 1))
        for i, r in enumerate(self.rows):
            on = (i == self.sel)
            col = C_FG if on else C_DIM
            if on:
                pygame.draw.rect(sc.s, (22, 28, 36),
                                 (int(sc.w * bx), int(y - dy * 0.36),
                                  int(sc.w * bw), int(dy * 0.72)))
            sc.text(sc.w * lx, y, r.label, sc.f_m, col, centre=False)
            r.rect = pygame.Rect(int(sc.w * bx), int(y - dy * 0.36),
                                 int(sc.w * bw), int(dy * 0.72))
            if r.kind == "spin":
                txt = r.show(r.value())
                sc.text(sc.w * vx, y, ("< %s >" % txt) if on else txt,
                        sc.f_m, C_OK if on else C_DIM)
            elif r.get is not None:
                v = r.value()
                if v is not None:
                    sc.text(sc.w * vx, y, str(v), sc.f_s,
                            C_OK if on else C_DIM)
            y += dy
        if self.rows:
            tip = self.rows[self.sel].tip()
            if tip:
                sc.text(sc.w / 2, sc.h * TIP_Y, tip, sc.f_s, C_DIM)
        return y

    def draw(self, sc):
        sc.text(sc.w / 2, sc.h * 0.09, self.title, sc.f_l, C_FG)
        if self.subtitle:
            sc.text(sc.w / 2, sc.h * 0.155, self.subtitle, sc.f_s, C_DIM)
        self.draw_rows(sc, sc.h * 0.27, sc.h * 0.84)
        if self.preview:
            w = sc.w * 0.30
            self.app.draw_preview(sc, sc.w * 0.66, sc.h * 0.28,
                                  w, w * FRAME_H / FRAME_W)
        sc.text(sc.w / 2, sc.h * 0.96, "Esc goes back", sc.f_xs, C_DIM)


# ---------------------------------------------------------------------------
# 2 -- camera tuning
# ---------------------------------------------------------------------------
class Camera(RowScreen):
    title = "CAMERA TUNING"
    preview = True

    def __init__(self, app):
        RowScreen.__init__(self, app)
        self.tuning = False
        self.stop = threading.Event()
        self._blob_t = 0.0
        self._built_wii = None     # which row set is currently built
        self._blob_ref = {}        # counter values at the last readout
        self._drop_ref = {}        # ...and for the drop counters, as
                                   # key -> (value, bframes, delta shown), so
                                   # the sentence survives the ~60 draws
                                   # between one gun reply and the next
        self._rate = FrameRate()   # the camera's own new-frame rate
        self._log = None           # CSV on the stick, when logging
        self._log_seq = 0          # ...and the number in its name
        self._learn_t = 0.0        # last '~camlearn?' poll, monotonic
        self._fit_t = 0.0          # ...and the last '~camfit?' one
        self._ask_t = 0.0          # the last question of ANY kind, so two of
                                   # them can never be asked in one breath
        self._meter = Meter()      # bsrej as a rate, over seconds of replies
        self._parsed_raw = None    # the blob line these came from...
        self._parsed = []          # ...parsed once, not once per reader
        self._blob_last = ""       # keeps the last percentage on screen while
                                   # the next window fills, so it stops flashing
        self.advanced = False      # which of the two row pages is built
        self.shape_rect = None     # what the sensor panel last covered, so
                                   # hostcheck can measure it against the rows
        self.view_rect = None      # ...and the camera view stacked above it
        self.build()
        self._built_wii = self.wiicam()

    def wiicam(self):
        b = self.app.link.last.get("board", "")
        return "wiicam" in b

    def build(self):
        """The row list for whichever of the two pages is showing.

        Sixteen rows on one page was more than anybody could find anything in
        while a test was running, so the list is split: the things a test
        actually needs stay here, and the gates and registers that are set
        once and left alone move to the second page. Which one is built is
        just a flag, because a real second screen would be a second Camera
        with its own blob log, its own shape-capture poll and its own idea of
        which counters it had already seen -- and the log started on page one
        would go on writing into an object nothing could reach to close.
        """
        link = self.app.link
        self.rows = []
        # An ESP32 gun has no second page. Left set, the flag would send a
        # gun that reconnected as an ESP32 to a page that does not exist.
        if not self.wiicam():
            self.advanced = False
        if self.advanced:
            self.build_advanced()
            return
        self.title = Camera.title
        if self.wiicam():
            self.subtitle = ("the wiicam finds blobs in hardware -- "
                             "sensitivity, shape, and what the sensor sees")
            self.rows.append(Row(
                "Sensitivity", "spin",
                get=lambda: link.last.get("sens", 0),
                set=lambda v: link.send("~cam=sens:%d" % v),
                lo=0, hi=2, step=1,
                hint="0 default, 2 highest. Use at least 1-2 with a wide lens."))
            # Ambient light. The wiicam finds blobs in HARDWARE and reports
            # four slots: a bright window does not add a fifth point, it TAKES
            # one, and an LED goes missing. The only hardware fact that tells
            # them apart is blob SIZE, which the basic report does not carry.
            # Read the sizes first (the line under the preview), THEN set a
            # window -- a gate guessed at is worth nothing.
            #
            # fmt is what the gun reports back; ext is the same answer from a
            # gun on older firmware, which only ever knew two formats and does
            # not send fmt at all. Without the fallback such a gun would show 0
            # here while it was busily reporting sizes.
            #
            # And it is sent as ext: for 0 and 1 for the same reason. Both
            # firmware generations take ext:; the older one has never heard of
            # fmt and drops the key silently, so this row could never move off
            # 0 on such a gun and every gate row below it stayed dead. Only
            # full mode -- which the old firmware cannot do at all -- goes out
            # as fmt:2.
            self.rows.append(Row(
                "Blob detail (sizes)", "spin",
                get=lambda: link.last.get("fmt", link.last.get("ext")),
                set=lambda v: link.send("~cam=%s:%d"
                                        % ("fmt" if v == 2 else "ext", v)),
                lo=0, hi=2, step=1,
                hint="0 position only, 1 adds each blob's size, 2 adds its box "
                     "and pixel count and fills the panel. Save to keep it"))
            # THE gate, and the reason the other two are on the second page.
            # Height is the one axis that still measures the SOURCE at every
            # sensitivity: turn the gain up and the sensor smears a blob
            # sideways -- the same LEDs went from a 2x2 box to 12x3 -- so
            # width, area, pixel count and roundness all start measuring the
            # gain instead. Height does not move.
            #
            # NO NUMBER IS SUGGESTED HERE, and none may be. The figure this
            # row used to recommend was measured on ONE bar with two LEDs per
            # corner; a bar with five LEDs in each cluster makes a blob
            # several times taller, so that ceiling drops every real LED on
            # such a gun -- and the owner sees a cursor that will not lock,
            # with nothing anywhere pointing at a row they set weeks ago. The
            # only defensible source for a non-zero value is '~camfit' on the
            # rig it is going to run on, which is what the room-light sweep
            # exists to feed. The hint points there instead.
            #
            # In ROWS, not pixels, and never "12 px": pxmax on the next page
            # is a count of lit pixels and would read identically. Same units
            # and same ladder as Studio's own row, so the two front ends
            # cannot describe the same gate two different ways.
            #
            # The ladder is a list of plausible values, not a list of values
            # worth choosing, and not a list of values the gun is guaranteed
            # to take either. Its floor is no longer a fixed number: the gun
            # refuses any ceiling BELOW the tallest LED this rig has been
            # measured at, and with nothing measured it accepts anything. So a
            # rung can be refused on a gun whose LEDs are taller than it --
            # which is exactly the five-LED-cluster case -- and the refusal is
            # surfaced as a toast rather than left as a row whose arrows
            # visibly do nothing. The first non-zero rungs are the numbers
            # this ladder was built from when the floor WAS fixed; they are
            # kept only so the step from "off" is not a cliff.
            self.rows.append(Row(
                "Biggest blob (height)", "spin",
                get=lambda: link.last.get("bhmax"),
                set=lambda v: link.send("~cam=bhmax:%d" % v),
                vals=(0, 8, 10, 12, 16, 24),
                fmt=lambda v: "off" if not v else "%d rows" % v,
                hint=self.bhmax_hint))
            # Measuring what an LED actually looks like on this rig, from the
            # couch. The gun does the accumulating; both of these exist
            # because the Pi has no console and the LED bar is at the TV, so
            # a capture that could only be started over a serial terminal
            # could never be taken from where the light actually goes wrong.
            self.rows.append(Row("Learn LED shape", act=self.learn_toggle,
                                 hint=self.learn_hint))
            self.rows.append(Row("Log blobs to the stick", act=self.log_toggle,
                                 hint=self.log_hint))
            self.rows.append(Row("Write shape CSV to the stick",
                                 act=self.shape_save, hint=self.shape_hint))
        else:
            self.subtitle = "aim for a blob noise floor under 0.30 px"
            for k, tip in (("thr", "threshold: raise it if the background shows up"),
                           ("aec", "exposure: lower it if blobs bleed together"),
                           ("agc", "gain: keep it low, brightness beats gain"),
                           ("boost", "sensor boost, on or off")):
                lo, hi = CAM_RANGE[k]
                self.rows.append(Row(
                    k.upper(), "spin",
                    get=(lambda kk=k: link.last.get(kk)),
                    set=(lambda v, kk=k: link.send("~cam=%s:%d" % (kk, v))),
                    lo=lo, hi=hi, step=(1 if k == "boost" else 2), hint=tip))
            self.rows.append(Row(
                "Auto tune", act=self.run_auto,
                hint="sweeps exposure and threshold, then keeps a safety margin"))
        # What ~camsave actually writes, on the board it is written from. The
        # gun stores the blob format, the size window, the odd-one-out
        # tolerance and the shape gate and stops there: the full-mode register
        # and the two SENSOR thresholds are left out on purpose, because they
        # are the settings that can leave a gun dark and a power cycle has to
        # stay a way back.
        # A flat "keeps these settings" sent people away believing an hwmax
        # they had spent an evening on would still be there in the morning.
        self.rows.append(Row(
            "Save to gun", act=self.save,
            hint=("keeps these across power cycles, except the full-mode "
                  "register and the two sensor thresholds") if self.wiicam()
            else "keeps these settings across power cycles"))
        if self.wiicam():
            self.rows.append(Row(
                "Advanced", act=self.enter_advanced,
                hint="the older gates, the sensor's own registers and the "
                     "connection test -- none of them needed for a test"))
        self.rows.append(Row("Back", act=self.app.to_menu))

    def build_advanced(self):
        """Page two: set once, then left alone.

        Everything here either has a working replacement on page one, gates
        INSIDE the sensor where a wrong value leaves the gun dark, or is a
        one-off check. None of it belongs in front of somebody trying to find
        the sensitivity row while an LED bar is dropping corners.
        """
        link = self.app.link
        self.title = "CAMERA TUNING -- ADVANCED"
        self.subtitle = ("set once and left alone -- the height gate on the "
                         "first page replaces most of this")
        self.rows.append(Row(
            "Smallest blob kept", "spin",
            get=lambda: link.last.get("bmin"),
            set=lambda v: link.send("~cam=bmin:%d" % v),
            lo=0, hi=15, step=1,
            hint="drops specks below this size; 0 keeps everything"))
        self.rows.append(Row(
            "Largest blob kept", "spin",
            get=lambda: link.last.get("bmax"),
            set=lambda v: link.send("~cam=bmax:%d" % v),
            lo=0, hi=15, step=1,
            hint="drops blobs bigger than this -- a window is usually "
                 "much larger than an LED; 15 keeps everything"))
        self.rows.append(Row(
            "Odd-one-out (size steps)", "spin",
            get=lambda: link.last.get("rtol"),
            set=lambda v: link.send("~cam=rtol:%d" % v),
            lo=0, hi=15, step=1,
            hint="drops a blob more than this many size steps from the "
                 "other three. Needs no distance tuning; 0 is off, 3 is "
                 "a fair start"))
        # The two older shape gates. The 4-bit size the three rows above judge
        # barely moves on this rig -- 52,624 confirmed LED blobs came in at
        # size 1 or 2 and did not respond to a 1.8x change of distance -- so
        # these were the first real measurement, and the height gate on page
        # one is the second. They are kept because guns in the field are set
        # up with them, not because they are what to reach for now.
        #
        # Both ladders are lists of values the GUN WILL ACCEPT, and nothing
        # more. They used to be an LED envelope with a step of margin, and
        # that envelope came off ONE bar with two LEDs per corner: a cluster
        # of five makes a blob several times bigger in every one of these
        # units, so those rungs blind such a gun and its owner has no way at
        # all to find out why. Every rung is legal, none is a suggestion, and
        # the ceiling that is worth having comes from '~camfit' on the rig it
        # will run on -- nothing here can know it. Their floors are no longer
        # fixed numbers either: the gun refuses a ceiling below what THIS rig
        # has measured its own LEDs at, and accepts anything at all when it
        # has measured nothing. So the ladders cannot promise every rung will
        # be taken; a rung under this gun's own envelope is refused by name,
        # which pical turns into a toast rather than leaving it as a row whose
        # arrows visibly do nothing. The first non-zero rungs are the numbers
        # they were built from back when the floors were fixed, kept only so
        # the step from "off" is not a cliff.
        #
        # Labelled and shown in what they physically reject, not in the wire's
        # units: 'armax 20' is unreadable, '2.5:1' is a shape.
        self.rows.append(Row(
            "Biggest blob (pixels)", "spin",
            get=lambda: link.last.get("pxmax"),
            set=lambda v: link.send("~cam=pxmax:%d" % v),
            vals=(0, 12, 13, 14, 16, 20, 24),
            fmt=lambda v: "off" if not v else "%d px" % v,
            hint="drops a blob of more pixels than this; the height gate is "
                 "the better one now. Needs Blob detail 2"))
        # "Not recommended" in the hint rather than only in a release note.
        # A ratio needs the width, and at sensitivity 2 the sensor smears a
        # blob sideways -- 2x2 became 12x3 on the same LEDs -- so this gate
        # measures the gain and throws away real LEDs, which reaches the user
        # as a cursor that sticks and never as a row they set weeks ago.
        #
        # DEPRECATED as well as not recommended, and the two are different
        # things: it still loads and still applies, so a gun somebody set up
        # with it keeps behaving exactly as it did, but nothing offers a value
        # for it any more and '~camfit' will never set it. The height gate is
        # the one the measurement can actually reach.
        self.rows.append(Row(
            "Roundness limit", "spin",
            get=lambda: link.last.get("armax"),
            set=lambda v: link.send("~cam=armax:%d" % v),
            vals=(0, 16, 20, 24, 32),
            fmt=lambda v: "off" if not v else "%g:1" % (v / 8.0),
            hint="DEPRECATED and NOT recommended -- drops real LEDs at "
                 "sensitivity 2. Needs Blob detail 2"))
        # The sensor's own thresholds. These gate INSIDE the camera, before
        # it hands out its four slots, so they are the only ones that stop
        # a stray source from costing a corner rather than being noticed
        # after it already has.
        #
        # No figure here either. The range this row used to quote was read off
        # somebody else's hardware, and "our LEDs can never be large" is a
        # claim about one bar: on a five-LED cluster they certainly can.
        self.rows.append(Row(
            "Sensor max size (0x06)", "spin",
            get=lambda: link.last.get("hwmax"),
            set=lambda v: link.send("~cam=hwmax:%d" % v),
            vals=(-1, 60, 80, 100, 120, 140, 160, 180, 200, 224, 255),
            hint="the camera's OWN ceiling, inside the sensor; -1 leaves it "
                 "alone, which is how it ships"))
        self.rows.append(Row(
            "Sensor min size (0x1B)", "spin",
            get=lambda: link.last.get("hwmin"),
            set=lambda v: link.send("~cam=hwmin:%d" % v),
            vals=(-1, 0, 1, 2, 3, 4, 5, 6, 8, 10),
            hint="never written by the stock driver, so it sits at an "
                 "unknown default. Risky knob: our LEDs are already small"))
        # Beside the sensor thresholds rather than beside Blob detail, because
        # it is the same kind of thing: a raw register nobody can look up.
        # It is the only move left when full detail comes back as nonsense.
        self.rows.append(Row(
            "Full-mode register (0x33)", "spin",
            get=lambda: link.last.get("fullreg"),
            set=lambda v: link.send("~cam=fullreg:%d" % v),
            vals=(85, 5), fmt="0x%02x",
            # "NOT saved" belongs in the hint, not only in the firmware's
            # camsave reply that nobody on the TV can read: ~camsave writes
            # fmt, the size window, rtol and the shape gates and stops there,
            # so a user who found the byte that works, pressed Save and
            # power-cycled would come back to nonsense and no idea why.
            hint="the byte that asks for full mode -- nobody is sure "
                 "which. Try the other if 2 shows nonsense; NOT saved"))
        self.rows.append(Row(
            "Sensor connection test", act=self.diag,
            hint="checks power, wiring, swapped lines and the sensor "
                 "itself; the log shows the verdict"))
        # Says where it goes AND where Save is. Nothing on this page saves,
        # and a user who has just spent a minute on the size window needs to
        # be told that once rather than find out on the next power cycle.
        self.rows.append(Row(
            "Back", act=self.leave_advanced,
            hint="back to the camera page -- Save to gun is there, and "
                 "nothing on this page is kept until it is pressed"))

    def enter_advanced(self):
        self.advanced = True
        self.build()
        self.sel = 0

    def leave_advanced(self):
        self.advanced = False
        self.build()
        # Back onto the row that opened it rather than at the top of the
        # list: this is a page people step in and out of while a capture is
        # running, and landing on Sensitivity every time invites nudging it.
        self.sel = next((i for i, r in enumerate(self.rows)
                         if r.label == "Advanced"), 0)

    def log_toggle(self):
        """Start or stop a CSV of every new camera frame, on the stick.

        The Pi has no console and the light that causes the trouble is only at
        the TV, so the numbers have to be captured where the gun is and read
        somewhere else. This lands on the FAT partition beside pical.py, which
        any PC can open."""
        if self._log is not None:
            n, path = self._log.rows, self._log.path
            self._log.close()
            self._log = None
            self.app.toast_now("stopped -- %d frames written to %s (recording "
                               "%d)" % (n, os.path.basename(path),
                                        self._log_seq))
            return
        try:
            path, seq = recording_path("blobs")
            self._log = BlobLog(path)
            self._log_seq = seq
        except Exception as e:
            self.app.toast_now("could not open the log: %s" % e)
            return
        # The number, out loud, at the start and again at the end. It is what
        # the user has to say to ask for the file afterwards, and the clock in
        # the rest of the name means nothing on a Pi that has no clock.
        self.app.toast_now("recording %d -- logging to %s, aim at the screen "
                           "then swing past the window"
                           % (self._log_seq,
                              os.path.basename(self._log.path)))

    def log_hint(self):
        if self._log is not None:
            return ("recording %d, %d frames so far -- select again to stop "
                    "and close the file" % (self._log_seq, self._log.rows))
        return "writes one row per camera frame to the stick, for reading on a PC"

    def close_log(self):
        if self._log is not None:
            self._log.close()
            self._log = None

    # ---- shape learning --------------------------------------------------
    def full_mode(self):
        """Is the gun in FULL report mode? fmt is what this firmware answers;
        ext is the same question to a gun too old to have full mode at all,
        and such a gun can never be in it."""
        return self.app.link.last.get("fmt", self.app.link.last.get("ext")) == 2

    def learn_on(self):
        """What the GUN last said, not what we last asked for. A '~camreset'
        stops the capture from anywhere, and a row still offering to 'stop'
        would be claiming something is being measured when nothing is."""
        return self.app.link.hists.running()

    def learn_toggle(self):
        """Start or stop the shape capture.

        Starting CLEARS what was measured -- that is the firmware's rule, and
        it is the right one: a distribution accumulated across a change of
        sensitivity or of the LED bar is two rigs averaged into one wide
        spread, which reads as 'these cannot be separated' when in truth
        neither rig was measured. Both the toast and the hint say so, because
        the CSV is the only copy and nothing warns twice."""
        link = self.app.link
        on = not self.learn_on()
        link.send("~camlearn=on:%d" % (1 if on else 0))
        # Ask straight back so the row follows the gun's own answer rather
        # than this screen's memory of the press.
        link.send("~camlearn?")
        self._learn_t = time.monotonic()
        if not on:
            frames, led, _rej = link.hists.counts()
            self.app.toast_now("shape capture stopped after %d frames and %d "
                               "LED blobs -- write the CSV before starting "
                               "another" % (frames, led))
            return
        if not self.full_mode():
            # Plainly, and short enough to fit the toast at 1024 px: this is
            # the one way to spend a whole capture and have nothing at the end
            # of it. Without full mode the gun feeds only the 4-bit size, and
            # size is the feature we already know does not tell a window from
            # an LED -- so the fix goes in the same sentence as the warning.
            self.app.toast_now("Blob detail is not 2: only SIZE will fill, and "
                               "size cannot tell a window from an LED. Set it "
                               "to 2 and start again")
            return
        self.app.toast_now("shape capture ON, from empty -- aim at the bar "
                           "from where you play and let it run")

    def learn_hint(self):
        frames, led, _rej = self.app.link.hists.counts()
        if self.learn_on():
            return ("measuring: %d confirmed frames, %d LED blobs so far -- "
                    "select again to stop" % (frames, led))
        if led:
            return ("stopped with %d frames and %d LED blobs -- write the CSV, "
                    "starting again CLEARS them" % (frames, led))
        return ("measures what a confirmed LED looks like here; set Blob "
                "detail to 2 first")

    def bhmax_hint(self):
        """What this gate is, and where its number has to come from.

        Never a suggested figure. The one that used to sit here was measured
        on a single bar with two LEDs per corner, and a cluster of five makes
        a blob several times taller: borrowed onto that gun the ceiling drops
        every real LED, and what the owner sees is a cursor that will not
        lock, with nothing on any screen connecting it to this row. So the
        only number this hint will ever show is one THIS gun measured, and
        until it has, it points at the sweep that measures it.

        Short on purpose: hints are drawn centred, on one line, with no
        wrapping anywhere, so a long one loses a word off each end -- and the
        word it loses is at the end, which is where the instruction is.
        """
        fit = self.app.fit
        if fit.verdict == "no_gate":
            return ("NO SAFE GATE here: your LEDs and the room light are the "
                    "same height. Leave it off and fix the light")
        if fit.verdict == "gate" and fit.bhmax is not None:
            return ("drops a blob taller than this. This gun measured %d rows. "
                    "Needs Blob detail 2" % fit.bhmax)
        return ("drops a blob taller than this; 0 is off. Needs Blob detail 2, "
                "and the room sweep to measure it")

    def gate_keys(self):
        """Which of the shape gate's three inputs -- bhmax, pxmax, armax --
        the gun is actually holding non-zero right now.

        The gate is bhmax || pxmax || armax: a ceiling on ANY ONE of the
        three is enough to make it act, so 'is it on' and 'what turns it
        off' both have to look at all three rather than assume the row this
        page shows first. A rig set up from the second page with pxmax or
        the DEPRECATED armax, with Biggest blob (height) never touched,
        runs the gate with bhmax sitting at 0 the whole time -- which is
        exactly the rig every message below used to get wrong.
        """
        last = self.app.link.last
        return [k for k in ("bhmax", "pxmax", "armax") if last.get(k)]

    def gate_off_clause(self, keys):
        """`keys` worded as the settings that zero them: 'bhmax:0', or
        'bhmax:0 and pxmax:0' when more than one is holding the gate up.
        Callers only reach this once `keys` is known to be non-empty."""
        return " and ".join("%s:0" % k for k in keys)

    def gate_lines(self):
        """The shape gate in plain words: the verdict, and any doubt about it.

        Four things belong here and nowhere else on this screen. A gate
        that is set but cannot act, which is invisible from every other row
        on the screen. What the gun set aside as contamination while it was
        measuring, because that is what turns a bare ceiling into one
        somebody can actually trust. The gun's own verdict, because NO SAFE
        GATE is an answer no amount of tuning can improve on and a user who
        has not been told that will spend an evening looking for the
        number. And how much the gate is throwing away, because a gate
        quietly refusing a large share of what the sensor finds is the
        state that ends in "the cursor stopped working". The false-negative
        meter -- the only counter that says a gate is WRONG rather than
        merely busy -- is fed from the same window as the last of these,
        but is drawn ABOVE it, in blob_lines(): it describes the last few
        seconds, and a standing state below it would push it off the screen
        the moment it appeared.

        Every one of them says how to switch the gate off, and by NAME: the
        gate is bhmax || pxmax || armax, so 'bhmax:0' silences it only when
        bhmax is the row actually holding it up. Named from the gun's own
        reply rather than assumed, because a rig gated through pxmax or the
        DEPRECATED armax alone has bhmax sitting at 0 already -- telling
        that owner to zero a row that already is would read as an answer
        while changing nothing.
        """
        out = []
        fit = self.app.fit
        # Read once, off the gun's own reply, for the whole of this call --
        # link.last cannot move mid-draw.
        keys = self.gate_keys()
        gate_on = bool(keys)
        full = self.full_mode()
        if gate_on and not full:
            # A ceiling that cannot act. The shape gate compares box HEIGHTS,
            # and outside full report mode the gun does not report a box at
            # all -- so the gate stands down and the number in the row above
            # judges nothing. This is the exact shape of the bug that made the
            # whole feature a no-op for a release: the saved format was
            # clamped below full mode, so every saved ceiling loaded into a
            # gun that could never execute it. Nothing else on this screen
            # shows it: the row reads "10 rows", the gun agrees, and the gate
            # is doing nothing whatsoever.
            out.append("SHAPE GATE IS SET BUT INERT -- it compares box "
                       "heights and Blob detail is not 2, so it is judging "
                       "nothing. Set Blob detail to 2 and Save, or %s to "
                       "turn the gate off" % self.gate_off_clause(keys))
        told = fit.ignored_words()
        if told:
            # What the gun threw out of its own LED measurement. This reaches
            # the readout rather than only the log because it is the one line
            # that explains a ceiling: an envelope of 7 rows with 32 samples
            # set aside at 31 is a rig with the sun in the LED class, and the
            # user who knows that can fix it. Never phrased as an error -- the
            # gun handled it correctly, and the ceiling it names is measured
            # from the rest.
            out.append("LED CAPTURE PARTLY CONTAMINATED: %s -- almost "
                       "certainly stray light learned as an LED. It is left "
                       "out of the ceiling; block that light and capture "
                       "again for a cleaner one" % told)
        if fit.verdict == "no_gate":
            # First and unconditional. This is the gun saying the thing this
            # whole screen is for cannot be done on this rig, and it must not
            # be crowded out by a counter.
            #
            # The advice names whatever the gun is CURRENTLY holding non-zero
            # rather than bhmax by default: a no_gate verdict says nothing
            # about which of the three rows a user reached for, and a rig
            # gated through pxmax alone already has bhmax at 0 -- so 'keep it
            # off with bhmax:0' would be true and useless in the same breath.
            # Nothing set at all is said plainly too, rather than naming a
            # row that was never the one holding the gate up.
            if keys:
                meanwhile = ("keep it off with %s meanwhile"
                            % self.gate_off_clause(keys))
            else:
                meanwhile = "it is off already"
            out.append("NO SAFE GATE ON THIS RIG -- your LEDs reach %s rows "
                       "and the room light starts at %s, so a size gate "
                       "cannot tell them apart. Move the bar, block the light "
                       "or use brighter LEDs; %s"
                       % (fit.led_h if fit.led_h is not None else "?",
                          fit.stray_h if fit.stray_h is not None else "?",
                          meanwhile))
        # The rate, over seconds rather than since boot -- and what is
        # counted is the part the resolver could NOT explain.
        #
        # The first version of this warned on raw rejections at one per frame,
        # with the argument that the resolver can only ever name one stray a
        # frame, so one rejection a frame is where honest work ends. The real
        # log said otherwise. A 92 s daylight session with bhmax:8 held four
        # straight seconds at EXACTLY 1.00 rejected per frame -- br4 zero,
        # br3 every frame, bnear zero throughout. That is the sun holding one
        # of the four slots, the gate refusing it on every frame, the gun
        # running on three real corners and one reconstructed: the gate doing
        # precisely its job, in the very scene it exists for. A warning at
        # "one per frame" fires for the whole of that stretch.
        #
        # What distinguishes that from a gate eating the bar is already on
        # the wire. bfar counts blobs the SHAPE GATE dropped that the resolver
        # then placed far from every expected corner -- confirmed strays --
        # and bnear the ones it placed exactly where a missing LED should have
        # been. Both count SHAPE-gate rejections only, and both now count on
        # EVERY frame rather than only while a capture is armed: the recording
        # into the histograms is gated on the capture, the classification is
        # not. That matters more than it sounds. While they were gated, they
        # shipped at zero -- nothing arms the capture at boot and both tools
        # switch it off on the way out of the sweep -- so during ordinary play
        # this subtraction degenerated to the raw count it was written to
        # replace, and fired the very warning it exists to prevent.
        #
        # In that session bsrej climbed 829 and bfar 730: 88% of the refusals
        # were vouched for. A gate set for a different bar refuses three or
        # four LEDs a frame, the resolver then has too few points to lock, and
        # NOTHING gets vouched for -- bfar and bnear both stay flat while
        # bsrej runs away. So the number that separates the two is rejections
        # minus what the resolver accounted for, and one of THOSE per frame is
        # unambiguous: a whole slot, every frame, that no one can say was
        # stray light.
        frames, _secs = self._meter.span()
        srej = self._meter.climb("bsrej")
        vouched = self._meter.climb("bfar") + self._meter.climb("bnear")
        unexplained = max(0, srej - vouched)
        per = unexplained / float(frames) if frames > 0 else 0.0
        if (gate_on and full and frames >= GATE_WINDOW_FRAMES
                and per >= GATE_HEAVY_PER_FRAME):
            out.append("SHAPE GATE IS REFUSING HEAVILY: %d blobs thrown away "
                       "over the last %d frames that the resolver could not "
                       "account for as stray light, %.1f of the 4 a frame "
                       "carries. It may be set for a different LED bar; "
                       "%s turns it off"
                       % (unexplained, frames, per,
                          self.gate_off_clause(keys)))
        return out

    def shape_save(self):
        """Write the histograms to the stick, beside the blob logs.

        Asks the gun and WAITS for a set it sent after the asking, rather than
        writing whatever the last poll left behind: the file carries no clue
        about its own age, so one written from a capture that ended ten
        minutes ago is indistinguishable from one taken just now."""
        link = self.app.link
        if not link.src:
            # Said at once rather than after the two-second wait below, which
            # this screen spends blocked: with no port there is nothing that
            # could answer, and a frozen screen is how a Pi looks when it has
            # crashed.
            self.app.toast_now("connect the gun first")
            return
        seq0 = link.hists.seq
        link.send("~camlearn?")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            # Pumped from here, because this runs off a key press rather than
            # from the frame loop and the reply would otherwise not be read
            # until the next draw -- long after this function has given up.
            link.pump()
            if link.hists.seq != seq0 and link.hists.ready():
                break
            time.sleep(0.02)
        if link.hists.seq == seq0 or not link.hists.ready():
            self.app.toast_now("no complete set of histograms came back in "
                               "2 s -- nothing written")
            return
        try:
            path, seq = recording_path("shape")
            n = write_shape_csv(path, link.hists)
        except Exception as e:
            self.app.toast_now("could not write the shape CSV: %s" % e)
            return
        frames, led, rej = link.hists.counts()
        name = os.path.basename(path)
        if not led:
            self.app.toast_now("wrote %s, but every bin in it is EMPTY -- no "
                               "confirmed LED blobs were measured" % name)
        elif link.hists.total(0, "aspect") == 0:
            self.app.toast_now("wrote %s -- SIZE only, %d blobs: the shape "
                               "features need Blob detail 2" % (name, led))
        else:
            self.app.toast_now("wrote %d rows to %s (recording %d) -- %d "
                               "frames, %d LED, %d rejected"
                               % (n, name, seq, frames, led, rej))

    def shape_hint(self):
        frames, led, rej = self.app.link.hists.counts()
        # Only once there is something to write. "writes the 0 LED and 0
        # rejected blobs measured over 0 frames" reads like a broken counter
        # rather than like a capture nobody has started yet.
        if led or rej:
            return ("writes the %d LED and %d rejected blobs over %d frames "
                    "to the stick, as the next shape-NNN.csv"
                    % (led, rej, frames))
        return ("writes the histograms to the stick as the next numbered "
                "shape-NNN.csv, one row per class and feature")

    def diag(self):
        self.app.link.send("~camdiag")
        self.app.toast_now("testing the sensor link -- watch the log for the "
                           "CAM: diag VERDICT line")

    def save(self):
        self.app.save_cam()

    def run_auto(self):
        if self.tuning:
            return
        self.tuning = True
        self.stop.clear()
        self.app.log_lines = ["auto tune running, this takes about a minute..."]

        def work():
            try:
                auto_tune(self.app.link, self.app.log, self.stop)
            except Exception as e:
                self.app.log("auto tune failed: %s" % e)
            finally:
                self.tuning = False

        threading.Thread(target=work, daemon=True).start()

    def handle(self, acts, mouse):
        if self.tuning:
            for a in acts:
                if a == "back":
                    self.stop.set()
            return
        # The row set is chosen from the board tag, which may not have arrived
        # when this screen was opened -- and a reconnect clears it again. Left
        # unwatched, a wiicam gun got the ESP32 sliders and none of the
        # ambient-light controls, with no way out but leaving and returning.
        if self.wiicam() != self._built_wii:
            self.build()
            self._built_wii = self.wiicam()
            self.sel = min(self.sel, max(0, len(self.rows) - 1))
        # Esc on the second page comes back to the first one. Left to
        # RowScreen it would go all the way out to the menu, and the menu
        # closes a blob log that is still recording -- so a glance at the size
        # window would end a capture the user was in the middle of taking.
        if self.advanced and "back" in acts:
            self.leave_advanced()
            acts = [a for a in acts if a != "back"]
        # The blob readout is a live measurement, so it is re-read while this
        # screen is up rather than sampled once on the way in. Faster while
        # logging, because then the samples are the deliverable.
        # monotonic, not wall time: a Pi has no clock until NTP corrects it,
        # and a backward step left the deadline in the future and stopped both
        # the readout and the logging until wall time caught up.
        every = 0.25 if self._log is not None else 1.0
        now_m = time.monotonic()
        if (self.wiicam() and now_m - self._blob_t > every
                and self.ask(now_m, self.BLOB_REPLY_S)):
            self._blob_t = now_m
            self.app.link.send("~camblob?")
            self._rate.feed(self.app.link.last.get("bframes"),
                            self.app.link.last.get("bms"))
            if self._log is not None:
                try:
                    self._log.sample(self.app.link.last,
                                     getattr(self.app.link, "blobs", ""),
                                     self._rate.hz)
                except Exception as e:
                    self.app.toast_now("log write failed: %s" % e)
                    self.close_log()
        # The gate's verdict. One short line plus one sentence, so it can be
        # asked for often enough to be live without taking anything from the
        # preview -- and it is what turns 'Biggest blob (height)' from a
        # number nobody can source into a measurement of this gun. Not asked
        # at all of a gun that has already said it does not know the command.
        if (self.wiicam() and self.app.fit.supported
                and now_m - self._fit_t > 2.5
                and self.ask(now_m, self.FIT_REPLY_S)):
            self._fit_t = now_m
            self.app.link.send("~camfit?")
        # The shape capture's counters, polled on their own clock and far
        # slower than anything else here. That answer is thirteen lines and up
        # to 2.6 KB, which is nearly a quarter of a second of a 115200 wire --
        # a quarter of a second in which not one camera frame can reach the
        # preview. At the two seconds this used to run at, that was an eighth
        # of the link and a visible hitch every other second on the one screen
        # a user sits on while judging whether the light is costing them
        # frames. Five is still a progress line that moves, and it costs a
        # twentieth; eight when nothing is running is enough to notice a
        # capture started from Studio, or one a '~camreset' has stopped.
        every_l = 5.0 if self.learn_on() else 8.0
        if (self.wiicam() and now_m - self._learn_t > every_l
                and self.ask(now_m, self.LEARN_REPLY_S)):
            self._learn_t = now_m
            self.app.link.send("~camlearn?")
        RowScreen.handle(self, acts, mouse)

    # How long each answer occupies the wire, in seconds, at 115200 8N1 --
    # measured from the firmware's own buffer sizes, not guessed: the blob
    # pair is about 470 bytes, the fit verdict about 300, and the thirteen
    # histogram lines up to 2.6 KB.
    #
    # These are not a rate limit, they are how long the gun will be BUSY. Only
    # one thing comes down this wire, so an answer being sent is a camera
    # frame that is not: with periods of 1 s and 2 s the blob poll and the
    # histogram poll lined up exactly, every other second, and the gun replied
    # with one ~2.9 KB burst -- a quarter of a second in which the preview
    # received nothing at all. Holding the next question until the last answer
    # can have finished costs nothing, because a poll that loses the race goes
    # out on the next frame 16 ms later and its own clock is only advanced
    # when the question actually leaves.
    BLOB_REPLY_S = 0.05
    FIT_REPLY_S = 0.03
    LEARN_REPLY_S = 0.25

    def ask(self, now_m, answer_s):
        """May a question go out now? Claims the wire until then if it may."""
        if now_m < self._ask_t:
            return False
        self._ask_t = now_m + answer_s
        return True

    def parsed_blobs(self):
        """The last '~camblob?' blob list, parsed once per reply.

        Three things on this screen read the same line -- the readout, the
        camera view's caption and the sensor panel -- and draw() runs at about
        60 fps while the line itself changes about once a second. Parsing it
        per reader per frame was the same string split nearly two hundred
        times a second for one answer's worth of numbers.
        """
        raw = getattr(self.app.link, "blobs", "")
        if raw != self._parsed_raw:
            self._parsed_raw = raw
            self._parsed = parse_blobs(raw)
        return self._parsed

    def _delta(self, key):
        """How much a since-boot counter moved since the last gun REPLY.

        Keyed on bframes, not on being called. This runs out of blob_lines(),
        which draw() calls at about 60 fps, while link.last only changes when
        a '~camblob?' answer lands -- roughly once a second. Re-baselining on
        every call meant the counter had already moved to its new value by the
        second frame, so "3 dropped by size" was on screen for the one frame
        after each reply and blank for the other fifty-nine: 16 ms in every
        second, which the eye reads as never. The last answer is kept and
        handed back until a genuinely newer frame count arrives.
        """
        last = self.app.link.last
        v = last.get(key)
        # Absent, or anything that will not subtract. Every counter on the
        # wire arrives through an int(), so the second case should be
        # impossible -- but this is called from draw() sixty times a second on
        # a screen with no console, where one TypeError is a black TV.
        if not isinstance(v, (int, float)):
            return 0
        frames = last.get("bframes")
        prev, at, shown = self._drop_ref.get(key, (None, None, 0))
        # frames is None only on a gun that does not report it at all; there
        # is nothing to key on there, so fall through and behave as before
        # rather than freeze on the first sample for ever.
        if frames is not None and frames == at:
            return shown
        d = 0 if (prev is None or v < prev) else v - prev   # or the gun rebooted
        self._drop_ref[key] = (v, frames, d)
        return d

    def blob_lines(self):
        """What the sensor is handing us right now, in words.

        Two numbers matter. The per-blob sizes say whether a window and an LED
        are even separable -- if they land on the same size, no gate can split
        them and the honest answer is a curtain, not a setting. The share of
        frames that lost a corner says how much the light is costing.
        """
        link = self.app.link
        out = []
        # The window the "refusing heavily" warning is measured over. Fed from
        # here rather than from handle() because handle() runs BEFORE the
        # reply it asked for has arrived; Meter itself throws away everything
        # that is not a genuinely new frame count, so being called from a
        # 60 fps draw is harmless.
        self._meter.feed(link.last.get("bframes"),
                         {"bsrej": link.last.get("bsrej"),
                          "bfar": link.last.get("bfar"),
                          "bnear": link.last.get("bnear")})
        raw = getattr(link, "blobs", "")
        if raw:
            parts = raw.replace("CAM: blobs", "").strip()
            if "(sizes need fmt:1)" in parts:
                out.append("blob sizes: set Blob detail to 1 to read them")
                parts = parts.replace("(sizes need fmt:1)", "").strip()
            shown = []
            parsed = self.parsed_blobs()
            for b in parsed:
                txt = "%d,%d size %d" % (b[0], b[1], b[2])
                # The box and the brightness only exist in full mode. Printed
                # as a fixed part of the line they would read as a measured
                # 0x0 box on every gun that is not in it.
                if len(b) >= 7:
                    txt += " box %dx%d %dpx" % (b[4], b[5], b[6])
                if b[3] != 1:
                    txt += " DROPPED"
                shown.append(txt)
            if shown:
                out.append("blobs now: " + "    ".join(shown))
            # Does the box mean what the panel draws it as? The position and
            # the box come out of different halves of the same report, and the
            # box is only believed to be in the sensor's 128x96 array. If it
            # is, the two land on each other; if they pull apart, every shape
            # gate on this screen is judging a number in unknown units and the
            # measurement behind them has to be redone. That is worth saying
            # in words as well as drawing, because it is a conclusion, not a
            # reading -- and it is the answer to the question the panel was
            # added to settle.
            gap = box_position_gap(parsed)
            if gap is not None and gap > BOX_MATCH_PX:
                out.append("BOX AND POSITION DISAGREE by %.1f of 128 px -- "
                           "box units wrong, or a lens correction is moving "
                           "the position" % gap)
            elif gap is not None:
                out.append("box centres land on the reported positions, worst "
                           "%.1f of 128 px -- the box really is in the "
                           "sensor's own array" % gap)
        # First, always: this one means the window is set so tight it would
        # blind the gun, and it was previously last and cut off by the slice.
        # The DELTA, like the drop counts below: bvalve counts since boot, so
        # the raw value pinned this warning on screen for the rest of the
        # power cycle after a single give-back -- spending one of the few rows
        # this readout gets on a window the user had already widened.
        if self._delta("bvalve"):
            out.append("SIZE WINDOW TOO TIGHT -- widen it or set it to 0..15")
        # Second, and for the same reason: this is the only number on the
        # screen that says a gate is WRONG rather than that it is working.
        # bnear counts blobs a gate threw away that sat exactly where the
        # missing corner had to be, so they were almost certainly LEDs -- it
        # is the false-negative meter, and it moves long before the symptom
        # does. Worded as a doubt about the gate, never as another drop count:
        # read as "N dropped" it would look like the gate earning its keep,
        # which is the exact opposite of what it means.
        #
        # It names the way OUT as well as the doubt. "Widen it" is advice
        # somebody has to know which row to act on; bhmax:0 is the one move
        # that always works and it can be read straight off the screen.
        #
        # Over the same WINDOW as the heavy-gate line, not over the last reply
        # alone. A one-reply delta clears after about a second while Studio's
        # is a rate over at least thirty frames, so the two front ends on the
        # same gun disagreed about whether the gate was taking LEDs at any
        # given moment -- and the one that said nothing was the one on the
        # screen with no console beside it. Any climb inside the window
        # raises it, which is Studio's rule too: this counter does not need a
        # threshold, because one blob rejected where a corner had to be is
        # already one too many.
        nframes, _ns = self._meter.span()
        dnear = self._meter.climb("bnear")
        if dnear and nframes >= GATE_WINDOW_FRAMES:
            # bnear counts SHAPE-gate rejections only, so the way out is
            # whichever shape knob is actually up -- named, not assumed to be
            # bhmax. A gun gated on pxmax alone would otherwise be told to
            # zero a knob that is already zero.
            keys = self.gate_keys()
            off = (self.gate_off_clause(keys) if keys
                   else "bhmax:0 (though no shape limit reads as set)")
            out.append("GATE MAY BE TAKING REAL LEDs: %d dropped over the "
                       "last %d frames where a corner should be -- widen it, "
                       "or %s to turn the shape gate off"
                       % (dnear, nframes, off))
        # The gun's own verdict on the gate, and how hard the gate is working.
        # Above the frame percentages because both are conclusions: "no size
        # gate can work here" is not something a percentage will ever say, and
        # it is the answer to the question the percentages raise. BELOW the
        # two warnings above, though, because those describe something
        # happening in the last second and are gone again by the next reply,
        # while these describe a standing state that will still be here on the
        # next draw -- a transient pushed off the bottom by a standing one is
        # a transient nobody ever sees.
        out.extend(self.gate_lines())
        keys = ("br4", "br3", "br2", "br1", "br0")
        now = [link.last.get(k) for k in keys]
        tot = None
        if None not in now:
            d = [n - self._blob_ref.get(k, 0) for n, k in zip(now, keys)]
            if any(x < 0 for x in d):
                # A gun that rebooted restarts its counters at zero, and a
                # negative delta never reaches the threshold below -- the
                # readout would freeze on stale numbers for good.
                self._blob_ref = dict(zip(keys, now))
                d = [0] * len(keys)
            tot = sum(d)
            if tot >= 20:
                # Over EVERY frame, the hopeless ones included: a percentage
                # taken over only the frames that went well is not a measure
                # of anything.
                out.append("last %d frames: %d%% saw all four LEDs, %d%% "
                           "three, %d%% two or fewer"
                           % (tot, 100 * d[0] // tot, 100 * d[1] // tot,
                              100 * (d[2] + d[3] + d[4]) // tot))
                self._blob_ref = dict(zip(keys, now))
                self._blob_last = out[-1]
            elif self._blob_last:
                out.append(self._blob_last)
        bits = []
        if self._rate.hz is not None:
            bits.append("camera %.0f new frames/s (we poll at 420)"
                        % self._rate.hz)
        # Deltas, like the percentages above. Splicing the since-boot totals
        # into a sentence about "the last N frames" meant a window that had
        # been fixed still read as dropping thousands of blobs, forever.
        drej = self._delta("brej")
        drrej = self._delta("brrej")
        dsrej = self._delta("bsrej")
        if drej:
            bits.append("%d dropped by size" % drej)
        if drrej:
            bits.append("%d dropped as odd-one-out" % drrej)
        # The shape gate's own count, beside the other two so the three can be
        # compared at a glance: which gate is doing the work is the first
        # question after tightening one, and it cannot be answered from a
        # total that lumps them together.
        if dsrej:
            bits.append("%d dropped by shape" % dsrej)
        if bits:
            out.append("   ".join(bits))
        # The two "something is being recorded" indicators, on ONE row, and
        # last. A row each would put one more line of content than there is
        # room for, and the one pushed off the bottom would be whichever came
        # second -- which is exactly the failure that widened the readout in
        # the first place: a capture running at the TV with nothing on screen
        # to say so. Last because everything above it is either a measurement
        # or a warning, and those are what a reader short of rows should get.
        rec = []
        if self.learn_on():
            frames, led, _rej = self.app.link.hists.counts()
            rec.append("SHAPE: %d frames %d LED blobs" % (frames, led))
        if self._log is not None:
            rec.append("LOGGING: %d frames -> %s"
                       % (self._log.rows, os.path.basename(self._log.path)))
        if rec:
            out.append("   ".join(rec))
        return out

    def draw_shape(self, sc, x, y, w, h, blobs=None):
        """What the sensor sees, drawn in ITS OWN 128x96 pixels.

        The readout above says where the gun thinks each blob is. It cannot
        say what SHAPE the thing was -- and shape is the only thing the gates
        on this screen judge, so tuning them from a coordinate is tuning
        blind. Each blob is drawn as the box it actually filled, at the size
        the sensor reports, inside the whole frame: one LED said to span
        twelve of 128 columns is either the truth about this lens or a unit
        error, and nobody can tell those apart from a number on its own.

        The crosshair is a MEASUREMENT, not decoration. The position and the
        box arrive in different halves of the same report and only the
        position's units are certain, so if the box fields really are in the
        sensor's array the box centre sits under the crosshair. When they pull
        apart the panel draws the gap as a line rather than tidying it away --
        a wrong unit here invalidates every shape gate and the whole LED
        envelope behind them.

        Returns the rectangle it covered, label and numbers included, so
        hostcheck can measure it against the rows and the readout.
        """
        if blobs is None:
            blobs = self.parsed_blobs()
        xi, yi, wi, hi = int(x), int(y), int(w), int(h)
        panel = pygame.Rect(xi, yi, wi, hi)
        pygame.draw.rect(sc.s, (16, 20, 26), panel)
        # The frame's own edge and a grid every 32 sensor pixels. Half the
        # question this panel answers is how much of the frame one LED takes
        # up, and "a quarter of the way across" is something the eye can judge
        # against a line where it cannot judge it against nothing.
        kx, ky = w / SENSOR_W, h / SENSOR_H
        for gx in range(32, int(SENSOR_W), 32):
            pygame.draw.line(sc.s, (30, 36, 44), (xi + gx * kx, yi),
                             (xi + gx * kx, yi + hi))
        for gy in range(32, int(SENSOR_H), 32):
            pygame.draw.line(sc.s, (30, 36, 44), (xi, yi + gy * ky),
                             (xi + wi, yi + gy * ky))
        pygame.draw.rect(sc.s, (48, 54, 61), panel, 1)
        # The legend is split across the caption and the line under the
        # numbers, because a caption that overhangs its own panel is worse
        # than no caption -- and the panel is now half as wide as it was, so
        # even "SENSOR 128x96 -- fill = how full" no longer fits. What is left
        # is the one thing the caption has to say: which array this is drawn
        # in, because that is what tells it apart from the camera view stacked
        # above it, and the two are different sizes for a real reason.
        cap = "SENSOR %dx%d" % (SENSOR_W, SENSOR_H)
        if sc.f_xs.size(cap)[0] > w:
            # Measured against the panel it belongs to, every time. On a short
            # screen the stacked column is height-limited down to under a
            # hundred pixels across, and the array size -- which is the part
            # a reader can look up elsewhere -- is what goes. What must stay
            # is the word that tells this panel apart from the camera view
            # above it, because the two are drawn in different arrays.
            cap = "SENSOR"
        sc.text(x + w / 2, y - sc.h * 0.026, cap, sc.f_xs, C_DIM)

        told = []                     # the numbers, one line per blob
        boxed = False
        for i, b in enumerate(blobs[:4]):
            tag = "ABCD"[i]
            bx, by, box, npx, dens, placed = blob_shape(b)
            # Kept or gate-dropped, on the outline, because that is the
            # question the gates are being tuned to answer. Never on the fill:
            # the fill is already carrying the density.
            col = C_OK if b[3] == 1 else C_BAD
            cx, cy = x + bx * kx, y + by * ky
            if box is not None:
                boxed = True
                r = pygame.Rect(int(x + box[0] * kx), int(y + box[1] * ky),
                                max(2, int(box[2] * kx)),
                                max(2, int(box[3] * ky))).clip(panel)
                pygame.draw.rect(sc.s, heat(dens), r)
                # Dashed would be better for a box whose origin was never
                # sent, but at 30 px across a dash is noise; one pixel of
                # outline instead of two, and the number says so in words.
                pygame.draw.rect(sc.s, col, r, 2 if placed else 1)
                sc.text(min(max(r.left + 5, xi + 6), xi + wi - 6),
                        max(yi + 7, r.top - 7), tag, sc.f_xs, col)
            if panel.collidepoint(int(cx), int(cy)):
                sc.crosshair(cx, cy, 3, C_FG, dot=True)
            gap = None
            if box is not None and placed:
                mx = x + (box[0] + box[2] / 2.0) * kx
                my = y + (box[1] + box[3] / 2.0) * ky
                gap = math.hypot(box[0] + box[2] / 2.0 - bx,
                                 box[1] + box[3] / 2.0 - by)
                if gap > BOX_MATCH_PX:
                    # Loud on purpose. This is the panel failing its own
                    # check, and a faint hairline would be read as decoration
                    # by the one person who needs to see it.
                    pygame.draw.line(sc.s, C_BAD, (mx, my), (cx, cy), 2)
                    pygame.draw.circle(sc.s, C_BAD, (int(mx), int(my)), 4, 1)
            if box is None:
                told.append(("%s no box" % tag, col))
                continue
            # w and h as the gun REPORTED them, not the w+1 the box is drawn
            # at: these are the numbers the gates compare against and the
            # numbers under suspicion, so printing anything else here would
            # hide exactly what is being checked.
            txt = "%s %dx%d  %dpx  %d%%" % (tag, b[4], b[5], npx,
                                            round(100 * dens))
            if not placed:
                txt += "  no origin"
            elif gap is not None and gap > BOX_MATCH_PX:
                txt += "  off %.1f" % gap
            told.append((txt, col))
        if not blobs:
            sc.text(x + w / 2, y + h / 2, "no blobs reported", sc.f_s, C_WARN)
        elif not boxed:
            # One line, not four. Outside full mode NO blob has a box, and
            # "A no box / B no box / C no box / D no box" is four lines of
            # column saying the one thing the reader has to do about it.
            told = [("boxes need Blob detail 2", C_WARN)]

        ty = y + h + sc.h * 0.012 + sc.f_xs.get_height() / 2.0
        dy = int(sc.f_xs.get_height() * 1.15)
        bottom = y + h
        for txt, col in told:
            sc.text(x + w / 2, ty, txt, sc.f_xs, col)
            bottom = ty + sc.f_xs.get_height() / 2.0
            ty += dy
        if blobs:
            # Shortened with the caption, and for the same reason. What had to
            # survive is the colour key: the outline is the only thing on the
            # panel that says whether a gate kept the blob or threw it away,
            # and that is the whole question the gates are tuned to answer.
            sc.text(x + w / 2, ty, "fill = how full; green kept, red dropped",
                    sc.f_xs, C_DIM)
            bottom = ty + sc.f_xs.get_height() / 2.0
        top = y - sc.h * 0.026 - sc.f_xs.get_height() / 2.0
        return pygame.Rect(xi, int(top), wi, int(bottom - top))

    def draw(self, sc):
        if self.tuning:
            sc.text(sc.w / 2, sc.h * 0.12, "AUTO TUNE", sc.f_l, C_WARN)
            sc.lines(sc.w / 2, sc.h * 0.26, self.app.log_lines[-12:], sc.f_s, C_DIM)
            sc.text(sc.w / 2, sc.h * 0.92, "Esc cancels", sc.f_s, C_DIM)
            return
        if not self.wiicam():
            RowScreen.draw(self, sc)
            self.app.draw_noise(sc, sc.h * 0.74)
            return
        # The wiicam list grew to twelve rows, so a readout at a FIXED height
        # landed on top of the last three of them. It is placed from where the
        # rows actually end instead, and the rows are given a shorter run.
        #
        # It reached SIXTEEN, at which point 1024x768 -- the mode the Pi
        # actually runs in -- left a 27 px pitch for a 24 px line box and the
        # readout was down to its last six rows. The list is now split in two,
        # nine rows and ten, which frees that up again; but draw_rows SPREADS
        # whatever it has over the band it is given, so nine rows in the old
        # sixteen-row band would put 51 px between 24 px labels, push the last
        # row's rectangle 8 px lower and cost the readout a row for nothing.
        # The pitch is capped at 0.050 h instead -- one and a half times the
        # glyph box -- and everything the rows do not need goes to the readout
        # and to the sensor panel beside them. Measured at 1024x768: 38 px of
        # pitch, 14 px of daylight, and eleven readout rows on the first page
        # and nine on the longer second one, where sixteen rows left six.
        sc.text(sc.w / 2, sc.h * 0.09, self.title, sc.f_l, C_FG)
        sc.text(sc.w / 2, sc.h * 0.145, self.subtitle, sc.f_s, C_DIM)
        y1 = min(sc.h * 0.715,
                 sc.h * 0.185 + max(0, len(self.rows) - 1) * sc.h * 0.050)
        y = self.draw_rows(sc, sc.h * 0.185, y1)
        # BOTH views, stacked. The sensor panel used to replace the CAMERA
        # VIEW here, and that was a false choice: they answer different
        # questions and tuning needs both at once. The camera view draws the
        # RESOLVED quad -- four corners the pipeline has already accepted --
        # so it says whether the gun is finding a target at all and how it is
        # moving; the sensor panel draws every blob the sensor handed over,
        # the gate-dropped ones included, in the array the gates measure. With
        # only the second one on screen, a gate that had started throwing away
        # a real corner looked exactly like a gate that was working, because
        # the thing it broke was not being drawn.
        #
        # Sized from the room they HAVE, not from the screen width alone. The
        # readout below is centred and runs nearly the full width, so it has
        # to start under the whole column: at 1024x768 the width cap used to
        # be the binding limit, but two panels one above the other are always
        # limited by the HEIGHT, and a 16:9 mode makes that worse -- the same
        # width fraction is taller there and pushed the readout down far
        # enough to cut rows off the bottom of it.
        #
        # 0.68 h is where the column and its numbers must stop for the readout
        # to keep the rows it needs; the floor stops a short screen from
        # shrinking the panels into something nobody can read. Measured at
        # 1024x768: the two boxes come out 117 and 119 px tall against the 230
        # a single panel had, which is the half each of them was promised, and
        # the readout keeps eight rows where it had nine.
        py = sc.h * 0.205
        # What the column has to carry besides the two boxes: a caption above
        # each of them, and the sensor panel's per-blob numbers underneath.
        cap = sc.h * 0.026 + sc.f_xs.get_height()
        below = 5 * int(sc.f_xs.get_height() * 1.15) + sc.h * 0.012
        room = sc.h * 0.68 - py - cap - below
        # Halved, and then turned into a width through the TALLER of the two
        # aspect ratios, so neither box can exceed its share: the sensor's
        # 128x96 is 4:3 and the pipeline's 240x176 is wider, so the sensor
        # panel is the one that binds.
        each = max(sc.h * 0.06, room / 2.0)
        w = min(sc.w * 0.30, max(sc.w * 0.13, each * SENSOR_W / SENSOR_H))
        vh = w * FRAME_H / FRAME_W
        ph = w * SENSOR_H / SENSOR_W
        # Centred in what is left beside the rows, which draw_rows ends at
        # 0.62 w whenever a preview is up.
        px = (sc.w * 0.62 + sc.w) / 2 - w / 2
        # Measured against the panel, like the sensor panel's own caption: on
        # a short screen this column is height-limited to under a hundred
        # pixels across and the full label overhangs it. Everything the
        # shorter one drops is decoration; the word that says which of the two
        # stacked panels this is stays.
        lab = "CAMERA VIEW"
        if sc.f_xs.size(lab)[0] > w:
            lab = "CAMERA"
        self.view_rect = self.app.draw_preview(sc, px, py, w, vh, label=lab)
        self.shape_rect = self.draw_shape(sc, px, py + vh + cap, w, ph)
        # Wrapped to the width the screen ACTUALLY has. A fixed 96 columns
        # filled 686 px of a 1024 px screen and left 338 px of it empty, which
        # cost the full-mode blob line a third wrapped row it did not need --
        # and that row came straight out of the readout's own budget below.
        cols = readout_cols(sc)
        lines = []
        for ln in self.blob_lines():
            lines.extend(wrap(ln, cols))
        # How many rows FIT, counted, rather than a hard-coded six. Six was
        # what fitted the day it was measured; the next row added to the list
        # above it silently pushed the last of them into the selected row's
        # own hint, which is the collision this arithmetic exists to make
        # impossible. Both ends come from the same numbers the rows and the
        # hint are drawn at, so the three cannot drift apart.
        #
        # The readout is CENTRED and runs nearly the full width, so it has to
        # clear the whole panel column as well as the rows -- the column is
        # taller than the shorter of the two row lists, and taking the rows
        # alone drew the first line of the readout straight through the
        # panel's own numbers.
        bot = max((r.rect.bottom for r in self.rows if r.rect), default=y)
        for panel in (self.view_rect, self.shape_rect):
            if panel is not None:
                bot = max(bot, panel.bottom)
        top = bot + sc.h * 0.016
        gh = sc.f_xs.get_height()
        step = max(1, int(gh * READOUT_STEP))
        # The hint's top edge, less a margin: running the readout to within a
        # pixel of it reads as one paragraph rather than two.
        floor = sc.h * TIP_Y - sc.f_s.get_height() / 2.0 - sc.h * 0.008
        room = int((floor - top - gh) // step) + 1
        sc.lines(sc.w / 2, top + gh / 2.0, lines[:max(1, room)], sc.f_xs,
                 C_DIM, READOUT_STEP)
        sc.text(sc.w / 2, sc.h * 0.96, "Esc goes back", sc.f_xs, C_DIM)


# ---------------------------------------------------------------------------
# 3 -- lens / FOV
# ---------------------------------------------------------------------------
class Lens(RowScreen):
    title = "LENS / FOV"
    subtitle = "skip this on a stock lens"
    preview = True

    def __init__(self, app):
        RowScreen.__init__(self, app)
        self.fov = 160
        self.sweeping = False
        self.t0 = 0.0
        self.frames = []
        self.seen = set()
        self.trail = None          # coverage map, painted as the sweep runs
        self.max_r = 0.0           # furthest any LED has been from frame centre
        self.want_hid = True
        self.prev_lens = {}
        self.n_full0 = self.n_part0 = 0
        self.report = []           # the sweep's own report, shown until dismissed
        self.report_ok = False
        self.fitting = False       # the fit runs off the main loop
        self.fit_result = None
        self.fit_id = 0
        self.fit_t0 = 0.0
        self.snap = None
        self.path = ""
        self.build()

    # ---- what the gun is holding right now -------------------------------
    def live_name(self):
        m = self.app.link.last.get("lens", None)
        if m is None:
            return "?"
        return "OFF" if m == 0 else ("polynomial" if m == 1 else "fisheye")

    def save_hint(self):
        return ("writes the correction currently live on the gun (%s) into its "
                "memory -- Preset and Measure both apply live first"
                % self.live_name())

    def build(self):
        link = self.app.link
        self.rows = [
            Row("Lens field of view", "spin",
                get=lambda: self.fov, set=lambda v: setattr(self, "fov", v),
                lo=20, hi=200, step=5, fmt="%d deg",
                hint="what the lens is sold as. Preset uses it; Measure only "
                     "uses it as a starting point"),
            Row("Apply preset from FOV", act=self.preset,
                hint="applies a textbook fisheye for that FOV, live on the gun"),
            Row("Measure this lens (20 s sweep)", act=self.measure,
                hint="measures the real distortion and applies it, live. "
                     "Beats a preset"),
            Row("Stock lens (correction off)", act=self.off,
                hint="restores the uncorrected pipeline"),
            Row("Dead-band", "spin",
                get=lambda: link.last.get("dead", 0),
                set=lambda v: link.send("~cam=dead:%d" % v),
                lo=0, hi=128, step=8,
                hint="0 is off; 16-32 calms rest shimmer on a wide lens"),
            Row("Save to gun (make permanent)", act=self.save,
                get=self.live_name, hint=self.save_hint),
            Row("Back", act=self.app.to_menu),
        ]

    def save(self):
        # the gun's own confirmation first; the friendly name only if it took
        if self.app.save_cam(lens=int(self.app.link.last.get("lens", 0) or 0)):
            self.app.toast_now("%s -- saved and confirmed by the gun"
                               % self.live_name())

    def off(self):
        self.app.link.send("~cam=lens:0")
        self.app.toast_now("correction off -- press Save to keep it")

    def preset(self):
        if self.fov <= 75:
            self.app.toast_now("%d deg is close enough to a pinhole; use Measure "
                               "only if the image looks bent" % self.fov)
            return
        r = calib_lens.spec_fisheye(self.fov)
        self.app.link.send("~cam=" + calib_lens.tune_line(dict(r, model="fisheye")))
        self.app.toast_now("fisheye preset applied live (feq %.1f) -- press Save "
                           "to keep it" % r["feq"])

    @property
    def hide_cursor(self):
        # The sweep turns the resolver off, so the cursor is the firmware's
        # stock aim wandering around -- not this pipeline's, and not useful.
        return self.sweeping or self.fitting

    # ---- the 20 s sweep --------------------------------------------------
    def measure(self):
        link = self.app.link
        if not link.src:
            self.app.toast_now("connect the gun first")
            return
        self.sweeping = True
        self.frames = []
        self.seen = set()
        self.trail = pygame.Surface((int(FRAME_W), int(FRAME_H)))
        self.trail.fill((0, 0, 0))
        self.trail.set_colorkey((0, 0, 0))
        self.max_r = 0.0
        self.t0 = time.time()
        self.want_hid = link.hid_on
        # Snapshot the live correction BEFORE the sweep turns it off, so a
        # refusal can put it back instead of leaving the gun uncorrected.
        self.prev_lens = {k: link.last.get(k, 0) for k in LENS_KEYS}
        self.n_full0, self.n_part0 = link.frames, link.partial_n
        # The resolver is off for the sweep, so the firmware falls back to the
        # stock aim and would fling the cursor around the screen.
        link.pointer(False, remember=False)
        link.send("~cam=res:0,lens:0,dashhz:0")

    def spans(self):
        if not self.frames:
            return 0.0
        a = np.array(self.frames[-300:])
        return float(np.median(np.linalg.norm(a.max(axis=1) - a.min(axis=1),
                                              axis=1)))

    def coverage(self):
        return self.max_r / min(CX, CY)

    def collect(self):
        for gt, q in self.app.link.hist:
            if gt in self.seen:
                continue
            self.seen.add(gt)
            a = np.asarray(q, float)
            self.frames.append(a)
            r = float(np.linalg.norm(a - (CX, CY), axis=-1).max())
            if r > self.max_r:
                self.max_r = r
            if self.trail is not None:
                for pt in a:
                    px, py = int(pt[0]), int(pt[1])
                    if 0 <= px < FRAME_W and 0 <= py < FRAME_H:
                        self.trail.set_at((px, py), (36, 84, 144))
        if (time.time() - self.t0) < LENS_SWEEP_S:
            return
        self.finish()

    def save_sweep(self, snap):
        """Keep every sweep, pass or fail, in the format calib_lens reads.
        A refusal with no data behind it cannot be diagnosed later.

        Numbered like every other recording. Named from the clock alone, two
        sweeps taken on two different evenings landed on the same file on a Pi
        that has no clock -- and a sweep that was REFUSED is exactly the one
        somebody wants to look at afterwards."""
        if not len(snap):
            return ""
        try:
            path, _seq = recording_path("lenssweep", ".log")
            with open(path, "w") as fh:
                for i, q in enumerate(snap):
                    fh.write("Q,%d,4," % (i * 7) +
                             ",".join("%d,%d" % (round(pt[0] * 10),
                                                 round(pt[1] * 10))
                                      for pt in q) + "\n")
            return path
        except Exception:
            return ""

    def finish(self):
        """Hand the gun back, then start the fit on a worker thread.

        The fit is tens of seconds of numpy on a Pi. Running it inline froze
        the screen with no explanation, which reads as a crash; running it on
        a thread keeps the screen alive. The thread only computes -- every
        serial write still happens on the main loop, so there is never a
        second writer on the port.
        """
        self.sweeping = False
        link = self.app.link
        link.send("~cam=res:2,dashhz:60")
        link.pointer(self.want_hid)
        self.snap = np.array(self.frames) if self.frames else np.zeros((0, 4, 2))
        self.path = self.save_sweep(self.snap)
        self.fitting = True
        self.fit_result = None
        self.fit_t0 = time.time()
        self.fit_id += 1
        me, snap, fov = self.fit_id, self.snap, self.fov

        def work():
            try:
                r = calib_lens.fit_from_frames(snap, fov)
            except Exception as e:
                r = dict(ok=False, why="the fitter crashed: %r" % e, model=None,
                         rms_px=0.0, coverage=0.0)
            if me == self.fit_id:          # a single assignment; no lock needed
                self.fit_result = r

        threading.Thread(target=work, daemon=True).start()

    def collect_fit(self):
        r = self.fit_result
        if r is None:
            return
        self.fitting = False
        self.fit_result = None
        link = self.app.link
        snap, path = self.snap, self.path

        rep = ["%d frames kept   |   coverage %.0f%% (needs %d%%)   |   "
               "quad span %.0f px (needs %.0f)"
               % (len(snap), self.coverage() * 100, COV_GATE * 100,
                  self.spans(), SPAN_GATE)]
        fulls = link.frames - self.n_full0
        parts = link.partial_n - self.n_part0
        if parts > fulls:
            rep += ["", "DROPOUTS: %d of %d frames were missing LEDs. A wide "
                        "lens dims the LEDs off-axis until the sensor stops "
                        "seeing them." % (parts, parts + fulls),
                    "Raise sensitivity (step 2), add LED power, or stand "
                    "closer, then sweep again."]
        self.report_ok = bool(r["ok"])
        if not r["ok"]:
            rep += [""] + wrap("REFUSED: " + r["why"])
            if self.prev_lens.get("lens"):
                link.send("~cam=" + ",".join("%s:%d" % (k, self.prev_lens[k])
                                             for k in LENS_KEYS))
                rep += ["", "The correction you had before has been put back."]
        else:
            link.send("~cam=" + calib_lens.tune_line(r))
            if r["model"] == "none":
                rep += ["", "No correction needed: this lens shows no "
                            "measurable distortion on this sensor.",
                        "Correction set to OFF."]
            else:
                line = "Fitted %s, %.2f px rms." % (r["model"], r["rms_px"])
                if r.get("fpx", 0) > 1.0:
                    line += ("  Measured field of view %.0f deg."
                             % (2.0 * math.degrees(math.atan(
                                 FRAME_W / 2.0 / r["fpx"]))))
                if r.get("lcx") or r.get("lcy"):
                    line += ("  Decentred lens: centre offset %+.1f, %+.1f px, "
                             "compensated." % (r["lcx"], r["lcy"]))
                rep += ["", line]
                if r["rms_px"] > 1.0:
                    rep += ["The residual is high -- consider sweeping again, "
                            "more slowly."]
                rep += ["", "Applied live. Press Save to keep it.",
                        "Then REDO Calibrate (step 4): the aim calibration was "
                        "made under the old lens mapping."]
        if path:
            rep += ["", "sweep saved: %s" % os.path.basename(path)]
        self.report = rep

    # ---- input -----------------------------------------------------------
    def handle(self, acts, mouse):
        if self.report:
            for a in acts:
                if a in ("select", "back", "click", "trigger"):
                    self.report = []
            return
        if self.fitting:
            self.collect_fit()
            for a in acts:
                if a == "back":
                    self.fitting = False
                    self.fit_id += 1        # orphan the worker's result
            return
        if self.sweeping:
            # Collect here rather than in draw(): the frames keep arriving
            # whether or not the last frame rendered, and a slow screen must
            # not cost the fit its data.
            self.collect()
            for a in acts:
                if a == "back":
                    self.sweeping = False
                    self.app.link.send("~cam=res:2,dashhz:60")
                    self.app.link.pointer(self.want_hid)
                    if self.prev_lens.get("lens"):
                        self.app.link.send("~cam=" + ",".join(
                            "%s:%d" % (k, self.prev_lens[k]) for k in LENS_KEYS))
            return
        RowScreen.handle(self, acts, mouse)

    # ---- drawing ---------------------------------------------------------
    def draw_report(self, sc):
        sc.text(sc.w / 2, sc.h * 0.10, "SWEEP RESULT", sc.f_l,
                C_OK if self.report_ok else C_BAD)
        sc.lines(sc.w / 2, sc.h * 0.22, self.report, sc.f_s, C_FG)
        sc.text(sc.w / 2, sc.h * 0.94, "any button continues", sc.f_xs, C_DIM)

    def draw_sweep(self, sc):
        left = max(0.0, LENS_SWEEP_S - (time.time() - self.t0))
        sc.text(sc.w / 2, sc.h * 0.08, "SWEEPING", sc.f_xl, C_WARN)
        sc.bar(sc.w * 0.25, sc.h * 0.15, sc.w * 0.5, sc.h * 0.018,
               1.0 - left / LENS_SWEEP_S, C_WARN)
        sc.lines(sc.w * 0.26, sc.h * 0.26, [
            "Pan, tilt and roll slowly so the LEDs travel",
            "over the WHOLE image -- push them out past",
            "the orange ring, into the corners.",
            "Keep all four in view; feet planted.",
        ], sc.f_s, C_DIM)
        # The three numbers the fit will be judged on, live, with the gate
        # each one has to pass -- so a refusal is never the first news.
        cov, span = self.coverage(), self.spans()
        n = len(self.frames)
        rows = [("%2.0f s left" % left, C_FG),
                ("frames %d / 30 needed" % n, C_OK if n >= 30 else C_WARN),
                ("coverage %3.0f%% / %d%%" % (cov * 100, COV_GATE * 100),
                 C_OK if cov >= COV_GATE else C_WARN),
                ("quad span %3.0f px / %.0f" % (span, SPAN_GATE),
                 C_OK if span >= SPAN_GATE else C_WARN)]
        link = self.app.link
        parts = link.partial_n - self.n_part0
        fulls = link.frames - self.n_full0
        if parts:
            rows.append(("dropouts %d of %d" % (parts, parts + fulls),
                         C_BAD if parts > fulls else C_WARN))
        y = sc.h * 0.50
        for txt, col in rows:
            sc.text(sc.w * 0.08, y, txt, sc.f_m, col, centre=False)
            y += sc.h * 0.055
        w = sc.w * 0.40
        self.app.draw_preview(sc, sc.w * 0.55, sc.h * 0.30, w,
                              w * FRAME_H / FRAME_W, rings=True,
                              trail=self.trail, label="COVERAGE")
        sc.text(sc.w / 2, sc.h * 0.94, "Esc abandons the sweep", sc.f_xs, C_DIM)

    def draw_fitting(self, sc):
        n = len(self.snap) if self.snap is not None else 0
        sc.text(sc.w / 2, sc.h * 0.34, "FITTING", sc.f_xl, C_WARN)
        sc.text(sc.w / 2, sc.h * 0.45, "%d frames, %.0f s elapsed"
                % (n, time.time() - self.fit_t0), sc.f_m, C_FG)
        sc.lines(sc.w / 2, sc.h * 0.56, [
            "Searching for the lens model that explains the sweep.",
            "This takes up to two minutes on a Pi. The gun is yours again.",
        ], sc.f_s, C_DIM)
        sc.bar(sc.w * 0.30, sc.h * 0.68, sc.w * 0.4, sc.h * 0.012,
               (time.time() * 0.5) % 1.0, C_WARN)
        sc.text(sc.w / 2, sc.h * 0.90, "Esc abandons the fit", sc.f_xs, C_DIM)

    def draw(self, sc):
        if self.report:
            self.draw_report(sc)
            return
        if self.fitting:
            self.draw_fitting(sc)
            return
        if self.sweeping:
            self.draw_sweep(sc)
            return
        RowScreen.draw(self, sc)
        m = self.app.link.last.get("lens", None)
        if m is not None:
            sc.text(sc.w * 0.33, sc.h * 0.81,
                    "live on the gun now: %s" % self.live_name(),
                    sc.f_s, C_DIM if m == 0 else C_OK)


# ---------------------------------------------------------------------------
# 4 -- aim calibration
# ---------------------------------------------------------------------------
class Calib:
    hide_cursor = True        # aim with the iron sights; the cursor is noise here

    def __init__(self, app, session):
        self.app = app
        self.session = session

    def handle(self, acts, mouse):
        for a in acts:
            if a == "back":
                self.app.to_menu()
            elif a in ("select", "click", "trigger"):
                self.session.note_trigger()
                self.session.trigger(self.app.link.gun_t or time.time(),
                                     pulled=True)

    def draw(self, sc):
        s = self.session
        app = self.app
        if app.no_data():
            app.draw_no_data(sc)
            return
        if s.state == s.S_STEPBACK:
            self.draw_stepback(sc)
            return

        tx, ty = s.target()
        cx, cy = tx * sc.w, ty * sc.h
        r = sc.h * 0.045
        cap = (s.state == s.S_CAPTURING)
        col = C_OK if cap else C_RING
        sc.crosshair(cx, cy, r, col)
        if cap:
            sc.arc(cx, cy, r * 1.5, s.progress(app.link.gun_t), C_OK, 5)
        elif s.dwell > 0:
            sc.arc(cx, cy, r * 1.5, s.dwell, C_WARN, 5)

        head = ("TILTED STANCE" if s.stance_kind() == "roll" else "DISTANCE")
        sc.text(sc.w / 2, sc.h * 0.06, "%s %d of %d"
                % (head, s.stance + 1, s.stances), sc.f_m, C_FG)
        sc.text(sc.w / 2, sc.h * 0.115, "dot %d of %d   --   pull %d of 4"
                % (s.idx + 1, len(s.dots), len(s.pulls) + 1), sc.f_s, C_DIM)

        if s.state == s.S_REVIEW:
            if s.last_result:
                sc.text(sc.w / 2, sc.h * 0.50, "captured", sc.f_l, C_OK)
            else:
                sc.text(sc.w / 2, sc.h * 0.47, "REJECTED", sc.f_l, C_BAD)
                sc.lines(sc.w / 2, sc.h * 0.55, [s.msg], sc.f_s, C_WARN)
        elif s.auto:
            sc.text(sc.w / 2, sc.h * 0.88,
                    "no trigger seen -- hold still on the dot to capture",
                    sc.f_s, C_WARN)
        else:
            sc.text(sc.w / 2, sc.h * 0.88,
                    "aim with your iron sights and pull the trigger",
                    sc.f_s, C_DIM)
        app.draw_hud(sc, "%d shots" % len(s.shots))

    def draw_stepback(self, sc):
        s = self.session
        if s.stance_kind() == "roll":
            want = s.stance_roll()
            d = s.roll_check()
            sc.text(sc.w / 2, sc.h * 0.32, "TILT THE GUN %s"
                    % ("CLOCKWISE" if want > 0 else "ANTICLOCKWISE"), sc.f_l, C_FG)
            now = ("tilt %+.0f deg" % d) if d is not None else "tilt --"
            good = d is not None and (d * want) > 0 and abs(d) >= 8.0
        else:
            r = s.stepback_check()
            sc.text(sc.w / 2, sc.h * 0.32, "STEP BACK", sc.f_l, C_FG)
            now = ("distance change %.2fx" % r) if r else "distance change --"
            good = r is not None and r >= 1.15
        sc.text(sc.w / 2, sc.h * 0.45, now, sc.f_m, C_OK if good else C_WARN)
        sc.lines(sc.w / 2, sc.h * 0.56, [
            "Hold the new position; it continues on its own.",
            "The two positions must differ or the fit cannot separate",
            "the sight offset from the screen mapping.",
        ], sc.f_s, C_DIM)
        self.app.draw_hud(sc, "%d shots" % len(s.shots))


# ---------------------------------------------------------------------------
# 4b -- the room-light sweep, which nobody has to do
# ---------------------------------------------------------------------------
class RoomSweep:
    """Show the gun the room, so it can tell a lamp from an LED.

    The wiicam finds blobs in HARDWARE and reports exactly four slots. A
    bright window does not add a fifth point, it TAKES one, and a corner goes
    missing -- so the only defence is a gate that can throw the window away
    before the resolver has to choose. The only honest way to set that gate is
    to measure both things on the rig it will run on: how big this bar's LEDs
    actually are, and how small the smallest thing in this room that is not
    one of them actually is. The first half fills itself whenever four corners
    are locked. The second half only fills when something that is NOT an LED
    is in the picture at the same time as they are, which is the one thing
    normal play never does -- hence a step that asks for it on purpose.

    It is OPTIONAL, and it is offered after the calibration rather than during
    it for that reason: by the time this screen appears the calibration has
    already been fitted, installed and written to the stick, so Esc costs
    nothing at all. Nothing here changes a setting on the gun unless the user
    chooses to apply the verdict.

    The gun's own settings are put back on the way out. Full report mode and
    the learning capture are both needed for the measurement and both are
    things a user may have had off for their own reasons.
    """

    # The verdict is two or three short lines -- around 300 bytes -- so it can
    # be asked for once a second without taking anything from the preview
    # beside it, which is the same wire. The 2.6 KB histogram dump is NOT
    # asked for here at all: everything this screen shows is on the fit line.
    POLL_S = 1.0
    # How long the gun gets to answer '~camfit=apply' before the screen stops
    # waiting. A gun on firmware without camfit never answers, and a spinner
    # that turns for ever is indistinguishable from one that has hung.
    APPLY_WAIT_S = 3.0

    def __init__(self, app):
        self.app = app
        self.t0 = time.monotonic()
        # Started from now, not from zero: the first question goes out below,
        # and a poll clock left at zero would send a second one on the very
        # first frame -- two 300-byte answers back to back for one reading.
        self._poll = self.t0
        self._apply_t = 0.0        # when apply was asked for; 0 = not asked
        self._saved = False        # ...and whether the format save has run
        self.applied = False       # a ceiling is live on the gun because of
                                   # this screen, so full mode must STAY
        self.done = ""             # the outcome, latched, once there is one
        link = app.link
        # Through the app, not from here. The calibration this step is offered
        # at the end of has already borrowed the same two settings, and taking
        # them again would re-send '~camlearn=on:1' -- an off->on edge to the
        # firmware, which CLEARS the histograms. That would wipe the five
        # minutes of confirmed LED frames the calibration just banked, at the
        # exact moment this screen set out to use them.
        app.cam_borrow()
        link.send("~camfit?")

    # The cursor is the gun's own aim and the screen stays in view throughout,
    # so it keeps meaning something here. Left visible on purpose: it is how
    # the user knows the gun is still locked onto the bar while they sweep.
    hide_cursor = False

    def restore(self):
        """Give the gun back what this step borrowed -- except full mode, if a
        ceiling was applied.

        Called from the way out AND from the app's own shutdown, because a
        crash or a window close on this screen would otherwise leave a user
        with a report format and a capture they never asked for and no sign
        anywhere of where they came from.

        The exception is not tidiness, it is correctness. The shape gate only
        ACTS in full report mode. This screen is normally entered from a gun
        in fmt:1, so handing the format back after applying a ceiling left the
        number saved in flash and the gate stone dead -- while the screen said
        "the gun will hold it through a power cycle", which was true of the
        number and false of everything the user cared about.
        """
        self.app.cam_restore(keep_full=self.applied)

    def leave(self):
        # to_menu() calls restore() for us, through the same hook the app's
        # own shutdown uses -- so there is ONE way out of this screen and no
        # path that can forget to hand the gun back.
        self.app.to_menu()

    def apply(self):
        """Set and save the ceiling the gun just worked out -- never anything
        else. There is no path here that writes a number this app chose."""
        fit = self.app.fit
        if fit.verdict == "no_gate":
            self.app.toast_now("there is nothing to apply: no height gate can "
                               "separate them on this rig")
            return
        if fit.verdict != "gate" or fit.bhmax is None:
            self.app.toast_now("not measured yet -- keep sweeping, or press "
                               "Esc to skip this step")
            return
        self.app.link.send("~camfit=apply")
        self._apply_t = time.monotonic()
        self._saved = False

    def handle(self, acts, mouse):
        now = time.monotonic()
        fit = self.app.fit
        if self._apply_t:
            # Waiting on the gun's own answer, so the screen reports what the
            # GUN did rather than what this app asked for: setting the gate
            # and getting it into flash fail separately, and a gate that took
            # but was not saved is gone on the next power cycle.
            if fit.applied in ("saved", "unsaved"):
                self.applied = True
                self._apply_t = 0.0
                self.done = self.finish_apply(fit)
            elif now - self._apply_t > self.APPLY_WAIT_S:
                self.done = ("The gun did not answer. Nothing has been "
                             "changed; this firmware may be too old to "
                             "measure it.")
                self._apply_t = 0.0
        elif (not self.done and fit.supported
                and now - self._poll > self.POLL_S):
            self._poll = now
            self.app.link.send("~camfit?")
        for a in acts:
            if a == "back":
                self.leave()
                return
            if a in ("select", "click"):
                if self.done:
                    self.leave()
                    return
                self.apply()

    def finish_apply(self, fit):
        """Check the ceiling really persisted, and say what actually happened.

        '~camfit=apply' now does the whole job itself: it switches the gun
        into full mode if it has dropped out of it and persists FULL rather
        than whatever format happened to be live, so the gate that comes back
        from a power cycle can actually run. This used to have to send
        '~camsave' as well, because apply stored the gate and left the format
        behind -- a command called 'apply' that did not survive a reboot,
        which was the bug rather than something to work around.

        The save stays, as a CHECK rather than as the mechanism. The gun's
        camsave reply carries fmt and bhmax, so it is the one way to read back
        what reached flash instead of trusting that it did -- and on a gun
        still running the firmware where apply left the format alone, it is
        also what makes the ceiling act. Everything this returns is read off
        what the gun said.
        """
        rows = fit.bhmax or 0
        if fit.applied == "unsaved":
            return ("Set to %d rows, but the gun could NOT save it -- it will "
                    "be gone on the next power cycle." % rows)
        ok = self.app.save_cam(fmt=2, bhmax=rows)
        self._saved = bool(ok)
        if not ok:
            return ("Set to %d rows, but full detail could NOT be saved -- "
                    "the gate needs it, so it will not act after a power "
                    "cycle." % rows)
        if fit.inert:
            # An older firmware saying it took the ceiling and cannot act on
            # it. It cannot happen on a build whose apply switches the format
            # itself, and it should not happen from here on any build -- this
            # screen puts the gun in full mode before it ever asks -- so if it
            # does, something refused the format and the user has to be told
            # rather than reassured.
            return ("Set to %d rows and saved, but the gun says the gate is "
                    "INERT: it needs Blob detail 2 and this gun is not in it."
                    % rows)
        if fit.switched:
            # Said because it is a change to the gun nobody asked for by
            # name. It is the right change -- the ceiling was measured in full
            # mode and only acts in full mode -- but a report format that
            # moved on its own is exactly the kind of thing a user finds later
            # and cannot explain.
            return ("Set to %d rows and saved. The gun switched itself to "
                    "full detail, which the gate needs to act, and kept it."
                    % rows)
        return ("Set to %d rows and saved, with full detail kept so the gate "
                "can actually act. It will hold through a power cycle." % rows)

    def bars(self, sc, y):
        """The two halves of the measurement, against what the gun asked for.

        Both are drawn even when one of them is full, because "which half is
        missing" is the whole question: an LED bar sat in front of a blank
        wall fills the first one in a minute and never fills the second at
        all, and without the second bar that reads as a broken step rather
        than as a room with nothing bright in it."""
        fit = self.app.fit
        led_f, stray_f = fit.progress()
        rows = (("your LEDs", fit.ledn, fit.led_want, led_f),
                ("room light", fit.stray_n, fit.stray_want, stray_f))
        for label, n, want, frac in rows:
            col = C_OK if frac >= 1.0 else C_WARN
            sc.text(sc.w * 0.05, y, label, sc.f_m, C_DIM, centre=False)
            sc.text(sc.w * 0.26, y, "%d / %d" % (n, want), sc.f_m, col,
                    centre=False)
            sc.bar(sc.w * 0.05, y + sc.h * 0.028, sc.w * 0.44, sc.h * 0.016,
                   frac, col)
            y += sc.h * 0.085
        return y

    def verdict_lines(self):
        """What the gun has concluded, in the reader's words.

        A firmware with no '~camfit' at all lands on the last branch, which
        says so instead of showing a blank panel -- and never shows a number,
        because on that gun there is no measurement behind one."""
        fit = self.app.fit
        if fit.verdict == "no_gate":
            return ([
                "NO GATE CAN WORK HERE.",
                "Something in your room is the same size as your LEDs.",
                "Move the bar, block that light, or use brighter LEDs.",
            ], C_BAD)
        if fit.verdict == "gate" and fit.bhmax is not None:
            out = ["MEASURED: anything taller than %d rows is not one of "
                   "your LEDs." % fit.bhmax]
            if fit.tight:
                out.append("A TIGHT fit: one step apart. Worth sweeping "
                           "again.")
            out.append("Enter sets it on the gun and saves it.")
            return out, C_OK
        if fit.verdict == "need_stray":
            return (["Keep sweeping: nothing but your LEDs has come into "
                     "the picture yet."], C_WARN)
        # Not a branch of its own: contamination can sit under any verdict,
        # and it is drawn beside whichever one applies (see draw).
        if fit.verdict == "need_led":
            out = ["Keep the screen in view: not enough of your own LEDs "
                   "measured yet."]
            told = fit.stored_words()
            if told:
                # The one number on this screen that may be shown before a
                # verdict, because it is not borrowed: the gun wrote it down
                # the last time IT was measured, and it is the only answer
                # there will ever be to "why is the gate set to what it is?"
                # after a power cycle has emptied the histograms.
                out.append("Measured before on this gun: %s." % told)
            return out, C_WARN
        if not fit.supported or time.monotonic() - self.t0 > 4.0:
            # Either the gun refused the command outright, or four seconds --
            # four polls -- have gone by with no answer of any kind. A gun
            # that has answered none of them is not slow, it is a gun that has
            # never heard of the question, and this screen can do nothing for
            # it but say so.
            return (["This gun's firmware cannot measure the room.",
                     "Skip this step; nothing here applies to it."], C_DIM)
        return (["asking the gun..."], C_DIM)

    def draw(self, sc):
        app = self.app
        # Wrapped to the room each block actually HAS. The header runs the
        # full width; everything below it shares the screen with the preview
        # on the right, so it gets a little over half. Measured, not assumed:
        # a fixed column that fits 1024x768 loses a word off each end of every
        # centred line on a 640-wide one, silently.
        wide = cols_for(sc.f_s, sc.w * 0.90)
        narrow = cols_for(sc.f_s, sc.w * 0.54)
        sc.text(sc.w / 2, sc.h * 0.065, "ROOM LIGHT", sc.f_l, C_FG)
        # WHY, before HOW, and in the reader's terms. Somebody who does not
        # know what this is for will skip it, and they would be right to.
        head = []
        for ln in ("This teaches the gun the difference between your LEDs and "
                   "the lights in your room, so a lamp can never take a corner "
                   "from it.",
                   "Sweep slowly around the room for about 15 seconds, "
                   "KEEPING THE SCREEN IN VIEW, so lamps, windows and "
                   "reflections come into the picture beside your LED bar."):
            head.extend(wrap(ln, wide))
        sc.lines(sc.w / 2, sc.h * 0.125, head, sc.f_s, C_DIM, step=1.35)
        if self.done:
            sc.lines(sc.w / 2, sc.h * 0.58, wrap(self.done, wide), sc.f_m,
                     C_FG, step=1.35)
            sc.text(sc.w / 2, sc.h * 0.90, "press Enter or Esc to finish",
                    sc.f_s, C_DIM)
        else:
            y = self.bars(sc, sc.h * 0.36)
            lines, col = self.verdict_lines()
            told = app.fit.ignored_words()
            if told:
                # Under the verdict rather than instead of it: the ceiling is
                # still the answer, and this is why it is the number it is.
                # A user who has this said to them can go and block the light;
                # one who does not gets a clean-looking 7 with the sun sitting
                # inside it.
                lines = lines + ["Some of it was stray light: %s. Those are "
                                 "left out." % told]
            wrapped = []
            for ln in lines:
                wrapped.extend(wrap(ln, narrow))
            # As many rows as there is room for above the way out, counted
            # from the same numbers both are drawn at. The verdict is the
            # longest thing this screen ever says and the way out is the one
            # thing that must never be pushed off it.
            step = max(1, int(sc.f_s.get_height() * 1.35))
            room = max(1, int((sc.h * 0.88 - sc.f_s.get_height() - y) // step))
            # Centred in the left column, which stops at the preview: these
            # lines are drawn CENTRED, so their x is the middle of the room
            # they have and not its left edge.
            sc.lines(sc.w * 0.30, y, wrapped[:room], sc.f_s, col, step=1.35)
            # The way out, always on screen and always the same key. This step
            # is optional and nobody may be trapped in it: the sentence says
            # both what Esc does and that the calibration is already finished,
            # because "skip" on its own reads like abandoning something.
            sc.text(sc.w / 2, sc.h * 0.92,
                    "Esc SKIPS this -- your calibration is already finished "
                    "and saved", sc.f_s, C_WARN)
        w = sc.w * 0.30
        app.draw_preview(sc, sc.w * 0.62, sc.h * 0.30, w, w * FRAME_H / FRAME_W)
        app.draw_hud(sc, "")


# ---------------------------------------------------------------------------
# 5 -- fine tune
# ---------------------------------------------------------------------------
class FineTune:
    """Iron sights onto the cursor, plus lead and smoothing.

    Order matters and the screen says so: align, then smoothing, then lead --
    smoothing changes the latency that lead is compensating for.
    """

    def __init__(self, app, tuner):
        self.app = app
        self.t = tuner
        self.recent = []
        self.sel = 0
        # One row per adjustable thing, so up/down always MOVES and left/right
        # always ADJUSTS -- the same rule as every other screen. The sight
        # offset used to own all four arrows, which left no way to reach
        # smoothing or lead with a d-pad at all.
        self.controls = ["dx", "dy", "smooth", "beta", "lead"]

    def feed_quad(self, q):
        """Banks a frame for the next ring shot, keeping only what a shot
        uses -- the sink runs for the whole session and an untrimmed list was
        a slow, unbounded leak on the Pi."""
        self.recent.append(np.asarray(q, float))
        del self.recent[:-SHOT_FRAMES]

    def send_preview(self):
        self.app.link.send("~" + aimcal_line(self.t.preview())
                           .replace("aimcal=", "aimcal!=", 1))

    def handle(self, acts, mouse):
        t = self.t
        for a in acts:
            if a == "back":
                self.app.link.send("~" + aimcal_line(t.c0))
                # beta belongs in the revert too: leaving it changed after a
                # cancel is a silent edit the user did not agree to keep.
                self.app.link.send("~cam=lead:%d,smooth:%d,beta:%d"
                                   % (t.lead0, t.smooth0, t.beta0))
                self.app.link.last["beta"] = t.beta0
                self.app.to_menu()
            elif a == "up":
                self.sel = (self.sel - 1) % len(self.controls)
            elif a == "down":
                self.sel = (self.sel + 1) % len(self.controls)
            elif a in ("left", "right"):
                d = -1 if a == "left" else +1
                k = self.controls[self.sel]
                if k == "dx":
                    t.nudge(d, 0); self.send_preview()
                elif k == "dy":
                    t.nudge(0, d); self.send_preview()
                elif k == "lead":
                    t.nudge_lead(d)
                    self.app.link.send("~cam=lead:%d" % t.lead)
                elif k == "beta":
                    # How fast the smoothing lets go once the gun moves.
                    # Independent of how heavy it is at rest. The step and the
                    # -1 "follow the table" rule live in Tuner, so this screen
                    # and Studio's fine-tune bar cannot drift apart.
                    t.nudge_beta(d)
                    self.app.link.last["beta"] = t.beta
                    self.app.link.send("~cam=beta:%d" % t.beta)
                else:
                    t.nudge_smooth(d)
                    self.app.link.send("~cam=smooth:%d" % t.smooth)
                    self.app.toast_now("smoothing %d -- re-check LEAD, its lag "
                                       "changed" % t.smooth)
            elif a == "select":
                self.sel = (self.sel + 1) % len(self.controls)
            elif a in ("click", "trigger"):
                self.shoot()
        return

    def shoot(self):
        t = self.t
        if t.stage > 1 or len(self.recent) < 4:
            return
        q = np.median(np.array(self.recent[-SHOT_FRAMES:]), axis=0)
        before = t.off[t.stage].copy()
        t.note_quad(q)
        d = t.off[t.stage] - before
        if not t.msg:
            self.app.toast_now("measured  %+.0f, %+.0f px"
                               % (d[0] * 1920.0, d[1] * 1200.0))
        self.send_preview()

    def finish_stage(self):
        t = self.t
        if t.stage == 0:
            if not t.measured[0]:
                self.app.toast_now("shoot the ring here first")
                return
            t.stage = 1
            t.msg = ""
        elif t.stage == 1:
            c = t.solve()
            if c is not None:
                self.commit(c, "two stations: angular and parallax separated")
            else:
                self.app.toast_now(t.msg)

    def save_now(self):
        """Two independent things live on this screen, so save both.

        The sight offset only exists once there is something to solve; lead,
        smoothing and beta are always savable. Gating the second on the first
        meant a session spent purely on feel ended with NOTHING written to the
        gun and a toast that only said 'nothing to save yet'."""
        c = self.t.solve_direct()
        if c is None:
            if self.save_cam_settings():
                self.t.lead0 = self.t.lead
                self.t.smooth0 = self.t.smooth
                self.t.beta0 = self.t.beta
            return
        self.commit(c, "saved as a constant offset from one position")

    def save_cam_settings(self):
        return self.app.save_cam(lead=self.t.lead, smooth=self.t.smooth,
                                 beta=self.t.beta)

    def commit(self, c, note):
        link = self.app.link
        try:
            msg = install_over_serial(link.src, aimcal_line(c), c)
        except Exception as e:
            msg = "NOT SENT -- %s" % e
        # The camera settings get their own verified save and their own line on
        # the result screen: the calibration going in says nothing about
        # whether lead, smoothing and beta reached flash.
        ok, cam_msg = camsave_verified(link.src, lead=self.t.lead,
                                       smooth=self.t.smooth, beta=self.t.beta)
        if ok:
            self.t.lead0 = self.t.lead
            self.t.smooth0 = self.t.smooth
            self.t.beta0 = self.t.beta
        self.app.view = Result(self.app, c, None, msg, None,
                               note + "\n" + cam_msg)

    def draw(self, sc):
        t = self.t
        app = self.app
        if app.no_data():
            app.draw_no_data(sc)
            return
        # the ring you line your iron sights up with
        cx, cy = sc.w * TARGET[0], sc.h * TARGET[1]
        r = sc.h * 0.035
        sc.crosshair(cx, cy, r, C_WARN, dot=False)

        sc.text(sc.w / 2, sc.h * 0.06,
                "FINE TUNE -- station %d of 2" % (t.stage + 1), sc.f_m, C_FG)
        # A small sensor view here too: if the quad is drifting or an LED is
        # dropping out, the offsets measured on this screen are meaningless.
        w = sc.w * 0.15
        app.draw_preview(sc, sc.w * 0.02, sc.h * 0.14, w, w * FRAME_H / FRAME_W)
        sc.text(sc.w / 2, sc.h * 0.585,
                "Shoot the ring with your IRON SIGHTS, or move the cursor onto "
                "your notch.", sc.f_s, C_DIM)
        sc.text(sc.w / 2, sc.h * 0.615,
                "Align first, then smoothing, then lead.", sc.f_s, C_DIM)

        d = t.off[t.stage] if t.stage < 2 else np.zeros(2)
        rows = [
            ("Sight  left / right", "%+.0f px" % (d[0] * 1920.0)),
            ("Sight  up / down", "%+.0f px" % (d[1] * 1200.0)),
            ("Smoothing (at rest)", "%d / %d" % (t.smooth, SMOOTH_MAX)),
            ("Speed sensitivity", "%s / %d" % (t.beta_label(), BETA_MAX)),
            ("Lead", "%d ms" % t.lead),
        ]
        y = sc.h * 0.635
        dy = sc.h * 0.050
        for i, (label, val) in enumerate(rows):
            on = (i == self.sel)
            col = C_FG if on else C_DIM
            if on:
                pygame.draw.rect(sc.s, (22, 28, 36),
                                 (int(sc.w * 0.27), int(y - dy * 0.36),
                                  int(sc.w * 0.50), int(dy * 0.72)))
            sc.text(sc.w * 0.30, y, label, sc.f_m, col, centre=False)
            sc.text(sc.w * 0.62, y, ("< %s >" % val) if on else val, sc.f_m,
                    C_OK if on else C_DIM, centre=False)
            y += dy
        hint = ("left / right moves the cursor across, relative to your sights",
                "left / right moves the cursor up and down",
                "raise until rest jitter settles, stop when it feels floaty",
                "raise if a SLOW drag feels sticky; a fast swipe's trail is LEAD",
                "raise while the cursor trails, stop when reversals overshoot")
        sc.text(sc.w / 2, sc.h * 0.885, hint[self.sel], sc.f_xs, C_DIM)
        sc.text(sc.w / 2, sc.h * 0.915,
                "up / down picks a row   |   left / right changes it   |   "
                "any pad button steps through", sc.f_xs, C_DIM)

        sp = t.spread()
        extra = "measured here" if t.measured[t.stage] else "not measured here yet"
        if t.stage == 1 and sp:
            extra += "   distance change %.2fx" % sp
        app.draw_hud(sc, extra)
        sc.text(sc.w / 2, sc.h * 0.955,
                "T shoots   |   N next station / finish   |   S save now   |   Esc cancels",
                sc.f_xs, C_DIM)


# ---------------------------------------------------------------------------
# 6 -- verify
# ---------------------------------------------------------------------------
class Verify:
    """Nine shots, comparing the pipeline's own solve with where the OS
    cursor actually landed. If the two agree, what is left is calibration
    error rather than anything downstream."""

    def __init__(self, app, calib):
        self.app = app
        self.c = calib
        self.idx = 0
        self.buf = []
        self.cur = []
        self.capturing = False
        self.t0 = 0.0
        self.results = []
        self.msg = ""
        self.done = False

    def handle(self, acts, mouse):
        for a in acts:
            if a == "back":
                self.app.to_menu()
            elif a in ("select", "click", "trigger"):
                if self.done:
                    self.app.to_menu()
                elif not self.capturing:
                    self.capturing = True
                    self.t0 = self.app.link.gun_t
                    self.buf = []
                    self.cur = []

    def feed(self, q, gt):
        if not self.capturing:
            return
        self.buf.append(np.asarray(q, float))
        self.cur.append(pygame.mouse.get_pos())
        if (gt - self.t0) > 0.9:
            self.finish()

    def finish(self):
        self.capturing = False
        sc_w, sc_h = self.app.sc.w, self.app.sc.h
        if len(self.buf) < VERIFY_FRAMES:
            self.msg = "only %d frames -- shoot again" % len(self.buf)
            return
        q = np.median(np.array(self.buf), axis=0)
        ours = aim_fit.solve(self.c, q, FRAME_W, FRAME_H)
        tx, ty = GRID_3x3[self.idx]
        cur = np.median(np.array(self.cur), axis=0) if self.cur else None
        self.results.append(dict(
            target=(tx * sc_w, ty * sc_h),
            ours=(ours[0] * sc_w, ours[1] * sc_h) if ours else None,
            actual=(float(cur[0]), float(cur[1])) if cur is not None else None))
        self.msg = ""
        self.idx += 1
        if self.idx >= len(GRID_3x3):
            self.done = True

    def stats(self):
        ours, act = [], []
        for r in self.results:
            if r["ours"]:
                ours.append(math.hypot(r["ours"][0] - r["target"][0],
                                       r["ours"][1] - r["target"][1]))
            if r["actual"]:
                act.append(math.hypot(r["actual"][0] - r["target"][0],
                                      r["actual"][1] - r["target"][1]))
        f = lambda v: (float(np.mean(v)), float(np.max(v))) if v else (0.0, 0.0)
        return f(ours), f(act)

    def draw(self, sc):
        app = self.app
        if app.no_data():
            app.draw_no_data(sc)
            return
        if self.done:
            (om, ox), (am, ax) = self.stats()
            sc.text(sc.w / 2, sc.h * 0.14, "VERIFY", sc.f_xl, C_FG)
            good = om < 25.0
            sc.lines(sc.w / 2, sc.h * 0.30, [
                "pipeline error      mean %.0f px   worst %.0f px" % (om, ox),
                "after the OS        mean %.0f px   worst %.0f px" % (am, ax),
            ], sc.f_m, C_OK if good else C_WARN)
            sc.lines(sc.w / 2, sc.h * 0.50, [
                "If the two rows agree, the remaining error is the",
                "calibration's, not the driver's or the OS's.",
                "" if good else "Over ~25 px: redo Calibrate, then Fine tune.",
            ], sc.f_s, C_DIM)
            sc.text(sc.w / 2, sc.h * 0.80, "press any button to finish", sc.f_m, C_FG)
            return
        for i, (gx, gy) in enumerate(GRID_3x3):
            x, y = gx * sc.w, gy * sc.h
            if i < self.idx:
                sc.ring(x, y, sc.h * 0.012, C_OK, 2)
            elif i == self.idx:
                sc.crosshair(x, y, sc.h * 0.035,
                             C_OK if self.capturing else C_RING)
        for r in self.results:
            if r["ours"]:
                pygame.draw.circle(sc.s, C_SEL,
                                   (int(r["ours"][0]), int(r["ours"][1])), 3)
        sc.text(sc.w / 2, sc.h * 0.06, "VERIFY -- shot %d of %d"
                % (self.idx + 1, len(GRID_3x3)), sc.f_m, C_FG)
        sc.text(sc.w / 2, sc.h * 0.88,
                "aim at the cross with your iron sights and shoot", sc.f_s, C_DIM)
        if self.msg:
            sc.text(sc.w / 2, sc.h * 0.93, self.msg, sc.f_s, C_WARN)
        app.draw_hud(sc, "")


# ---------------------------------------------------------------------------
# result
# ---------------------------------------------------------------------------
class Result:
    def __init__(self, app, calib, why, install, saved, note=""):
        self.app = app
        self.c = calib
        self.why = why
        self.install = install
        self.saved = saved
        self.note = note
        self.sel = 0
        self.rows = [("Done", "menu"), ("Do it again", "again")]
        # The room-light sweep is offered HERE, from the screen that says the
        # calibration is finished, and it is never the first row. That is the
        # whole of its skippability: 'Done' is already selected, it is the
        # obvious thing to press, and pressing it leaves a gun that is
        # calibrated and saved. Only after a fit that actually worked, and
        # only on the sensor that has anything to measure -- offering it after
        # a refusal would read as part of the recovery.
        if calib and "wiicam" in app.link.last.get("board", ""):
            self.rows.append(("Learn the room light (optional)", "sweep"))
        self.hot = []

    def handle(self, acts, mouse):
        for a in acts:
            if a == "up":
                self.sel = (self.sel - 1) % len(self.rows)
            elif a == "down":
                self.sel = (self.sel + 1) % len(self.rows)
            elif a in ("select",):
                self.act(self.rows[self.sel][1])
            elif a == "back":
                self.app.to_menu()
            elif a == "click":
                for i, r in enumerate(self.hot):
                    if r.collidepoint(mouse):
                        self.act(self.rows[i][1])
                        break

    def act(self, what):
        if what == "again":
            self.app.begin_calib()
        elif what == "sweep":
            self.app.begin_room_sweep()
        else:
            self.app.to_menu()

    def draw(self, sc):
        if self.c:
            px = self.c["fit_rms"] * ((1920.0 ** 2 + 1080.0 ** 2) ** 0.5) / (2 ** 0.5)
            good = px < 20.0
            sc.text(sc.w / 2, sc.h * 0.14, "CALIBRATED", sc.f_xl,
                    C_OK if good else C_WARN)
            sc.text(sc.w / 2, sc.h * 0.24, "fit error %.1f screen px%s"
                    % (px, "" if good else "   (high -- consider redoing it)"),
                    sc.f_m, C_OK if good else C_WARN)
            body = ["LED rectangle %.3f x %.3f of the screen"
                    % (self.c["w"], self.c["h"]),
                    "sight offset %+.1f, %+.1f camera px"
                    % (self.c["bx"], self.c["by"])]
            if self.note:
                # the note carries one line per thing saved (calibration, then
                # the camera settings), so it must not be drawn as one string
                body.extend(l[:88] for l in self.note.split("\n") if l)
            sc.lines(sc.w / 2, sc.h * 0.34, body, sc.f_s, C_DIM)
            ok = "INSTALLED" in (self.install or "").upper()
            sc.text(sc.w / 2, sc.h * 0.56,
                    "saved to the gun" if ok else "NOT SAVED TO THE GUN",
                    sc.f_m, C_OK if ok else C_BAD)
            if not ok and self.install:
                sc.lines(sc.w / 2, sc.h * 0.62, [self.install[:78]], sc.f_s, C_WARN)
        else:
            sc.text(sc.w / 2, sc.h * 0.16, "FIT REFUSED", sc.f_xl, C_BAD)
            sc.lines(sc.w / 2, sc.h * 0.30,
                     [self.why or "not enough usable shots"], sc.f_s, C_WARN)
            sc.lines(sc.w / 2, sc.h * 0.44, [
                "Nothing was changed on the gun.",
                "The usual cause is not moving far enough between distances --",
                "the two stances must differ by 1.25x or more.",
            ], sc.f_s, C_DIM)
        if self.saved:
            sc.text(sc.w / 2, sc.h * 0.70, "log: %s" % os.path.basename(self.saved),
                    sc.f_s, C_DIM)
        self.hot = []
        y = sc.h * 0.80
        for i, (label, _k) in enumerate(self.rows):
            on = (i == self.sel)
            r = sc.text(sc.w / 2, y, ("> %s <" % label) if on else label,
                        sc.f_m, C_FG if on else C_DIM)
            self.hot.append(r.inflate(sc.w * 0.4, sc.h * 0.02))
            y += sc.h * 0.075


# ---------------------------------------------------------------------------
# 7 -- recoil feel
# ---------------------------------------------------------------------------
class Recoil(RowScreen):
    """The solenoid feel engine's knobs, from the couch.

    Every row is a serial command; the values shown are the gun's own FX:
    echoes, so a clamp or a refused save is visible, never papered over. The
    engine ships OFF: the gun is stock until the first row says otherwise."""

    title = "RECOIL FEEL"
    subtitle = "engine OFF = stock behaviour. Dry-fire lets the trigger work with no IR."

    KNOBS = (
        ("on",     "Engine",             1, 0,   1,
         "0 = stock OpenFIRE recoil, 1 = this engine owns the solenoid"),
        ("drive",  "Strike (ms)",        5, 5,  100,
         "the full-power hit; 45 with hold 0 is the stock default"),
        ("hold",   "Hold (ms)",          10, 0,  500,
         "separates the strike from the spring return; the two-event feel"),
        ("duty",   "Hold power (%)",     5, 25,  70,
         "raise until the hold stops buzzing, then stop"),
        ("pulse",  "After-pulses",       1, 0,   3,
         "extra clacks after release"),
        ("gap",    "Pulse gap (ms)",     5, 15,  120,
         "short = meatier single hit, long = a distinct double-clack"),
        ("jit",    "Jitter (%)",         3, 0,   15,
         "random stretch on hold and gaps; the strike stays exact"),
        ("rumoff", "Rumble offset (ms)", 5, -20, 50,
         "negative = the motor leads the strike"),
        ("rumms",  "Rumble time (ms)",   10, 0,  200,
         "0 leaves the rumble to OpenFIRE"),
        ("space",  "Re-fire space (ms)", 10, 0,  500,
         "quiet time between shots"),
        ("auto",   "Autofire wait (ms)", 50, 0, 1000,
         "how long the trigger is held before autofire starts"),
    )

    def __init__(self, app):
        RowScreen.__init__(self, app)
        link = app.link
        self.rows = []
        for key, name, step, lo, hi, tip in self.KNOBS:
            self.rows.append(Row(
                name, "spin",
                get=(lambda kk=key: link.last.get("fx" + kk)),
                set=(lambda v, kk=key: link.send("~fx=%s:%d" % (kk, v))),
                lo=lo, hi=hi, step=step, hint=tip))
        self.rows.append(Row("Test fire", act=self.test,
                             hint=self.test_hint))
        self.rows.append(Row("Dry-fire mode (10 min)", act=self.dryfire,
                             hint=self.dry_hint))
        self.rows.append(Row("Save to gun", act=self.save,
                             hint="keeps every knob across power cycles"))
        self.rows.append(Row("Back", act=self.app.to_menu))
        self._poll_t = 0.0
        link.send("~fx?")            # seed the rows from the gun's own state

    def handle(self, acts, mouse):
        # Re-read the gun every couple of seconds while this screen is up:
        # the dry-fire countdown otherwise froze at its armed value, claiming
        # ARMED long after the gun had disarmed itself.
        if time.time() - self._poll_t > 2.0:
            self._poll_t = time.time()
            self.app.link.send("~fx?")
        RowScreen.handle(self, acts, mouse)

    def test(self):
        self.app.link.send("~fx=test:1")

    def test_hint(self):
        if self.app.link.last.get("fxquiet"):
            return ("QUIET MODE is on -- nothing will fire until it is off or "
                    "lapses; leaving a calibration turns it off")
        return "one full sequence, no trigger and no IR needed"

    def draw(self, sc):
        RowScreen.draw(self, sc)
        # Quiet mode has to be VISIBLE here, or a gun that is deliberately
        # silent is indistinguishable from a gun that has broken -- which is
        # exactly how the old quieting was read.
        if self.app.link.last.get("fxquiet"):
            left = int(self.app.link.last.get("fxqleft", 0) or 0)
            sc.text(sc.w / 2, sc.h * 0.205,
                    "QUIET MODE ON -- nothing fires (%d:%02d left)"
                    % (left // 60, left % 60), sc.f_s, C_WARN)

    def dryfire(self):
        self.app.link.send("~fx=ab:1")
        self.app.link.send("~fx?")
        self.app.toast_now("dry-fire ON for 10 minutes -- the trigger now "
                           "fires the solenoid with no IR")

    def dry_hint(self):
        """Named state, not hope: the mode disarms itself after 10 minutes,
        and without a countdown its expiry reads as breakage."""
        left = int(self.app.link.last.get("fxleft", 0) or 0)
        if left > 0:
            return ("ARMED, %d:%02d left -- re-select to re-arm for another "
                    "10 minutes" % (left // 60, left % 60))
        return "the trigger fires with no IR lock; expires by itself"

    def save(self):
        self.app.link.send("~fxsave")
        self.app.toast_now("asked the gun to save -- watch the log for FX: saved")


# ---------------------------------------------------------------------------
# menu
# ---------------------------------------------------------------------------
class Menu(RowScreen):
    title = "LIGHTGUN CALIBRATION"
    subtitle = "aim with your IRON SIGHTS -- the cursor does not matter here"

    def __init__(self, app):
        RowScreen.__init__(self, app)
        self.rows = [
            Row("2   Camera tuning", act=lambda: app.open(Camera(app)),
                hint="exposure, threshold and the blob noise floor"),
            Row("3   Lens / FOV", act=lambda: app.open(Lens(app)),
                hint="only needed on a wide or fisheye lens"),
            Row("4   Aim calibration", act=app.begin_calib,
                hint="five dots at two or three distances"),
            # Offered here as well as on the way out of a calibration, because
            # the way out of a calibration is exactly where it is most likely
            # to be skipped -- and a step that can only ever be reached by
            # redoing the whole calibration would never be done at all.
            Row("4b  Room light sweep (optional)", act=app.begin_room_sweep,
                hint="about 15 s: teaches the gun to tell your LEDs from the "
                     "lamps and windows in the room"),
            Row("5   Fine tune", act=app.begin_finetune,
                hint="iron sights onto the cursor, then smoothing, then lead"),
            Row("6   Verify", act=app.begin_verify,
                hint="nine shots, measures what the calibration achieved"),
            Row("7   Recoil feel", act=lambda: app.open(Recoil(app)),
                hint="solenoid strike, hold and after-pulses (RP2040 build)"),
            Row("Distances for calibration", "spin",
                get=lambda: app.stances, set=lambda v: setattr(app, "stances", v),
                lo=2, hi=3, step=1,
                hint="three separates the sight offset better than two"),
            Row("Reconnect the gun", act=app.connect,
                hint="use this after replugging the gun"),
            Row("Quit to a shell", act=app.quit,
                hint="the console comes back; type pical-launch to return"),
        ]

    def handle(self, acts, mouse):
        acts = [a for a in acts if a != "back"]
        RowScreen.handle(self, acts, mouse)

    def draw(self, sc):
        sc.text(sc.w / 2, sc.h * 0.075, self.title, sc.f_xl, C_FG)
        sc.text(sc.w / 2, sc.h * 0.135, self.subtitle, sc.f_s, C_DIM)
        self.draw_rows(sc, sc.h * 0.24, sc.h * 0.77)
        link = self.app.link
        if link.src:
            b = link.last.get("board", "gun")
            col, msg = C_OK, "connected on %s   (%s)" % (link.port, b)
            if self.app.no_data():
                col, msg = C_BAD, ("port %s open but no camera frames -- all "
                                   "four LEDs in view?" % link.port)
        else:
            col, msg = C_BAD, "NO GUN FOUND -- plug it in and choose Reconnect"
        sc.text(sc.w / 2, sc.h * 0.815, msg, sc.f_s, col)
        self.app.draw_noise(sc, sc.h * 0.86)


# ---------------------------------------------------------------------------
# application
# ---------------------------------------------------------------------------
class App:
    def __init__(self, surf, stances=3):
        self.sc = Screen(surf)
        self.link = Link()
        self.inp = Input()
        self.stances = stances
        self.session = None
        self.running = True
        self.toast = ""
        self.toast_t = 0.0
        # The gun's own verdict on the shape gate. Kept on the app rather than
        # on a screen because two of them need it -- the camera page draws the
        # warnings, the room sweep drives the whole step off it -- and the
        # lines it is built from arrive on the shared reply stream, which only
        # this loop drains.
        self.fit = FitReport()
        # Full report mode and the learning capture, borrowed by a step that
        # needs them and given back afterwards. On the app rather than on a
        # screen because the borrow SPANS screens: a calibration takes it, the
        # result screen holds it, and the room sweep offered from there has to
        # inherit it rather than take it again -- taking it again would clear
        # the very data the calibration just gathered.
        self._cam_borrow = None
        self._cam_arm = None      # a borrow that has asked and not yet armed
        # The gun's own clock as the last session saw it, kept across a
        # reconnect so a gun that RESTARTED can be told from a port that
        # merely re-enumerated. None once the question has been answered.
        self._gun_t0 = None
        self._fx_saved = None
        self._fx_plan = "none"        # which silence this gun supports
        self._fx_quiet_want = False   # we are asking for silence right now
        self._fx_quiet_t = 0.0        # last time we re-armed it
        self._fx_verify = None        # (values, deadline) for a restore check
        self._fx_retry = 0.0          # last attempt at an owed restore
        self.log_lines = []
        self.t_open = 0.0
        # Link liveness. The reader thread ends on any serial error, and
        # nothing on screen changed when it did: values froze, keys still
        # "worked", and it read as a broken screen rather than a dead gun.
        self._link_bad = False
        self._link_t = 0.0
        self._link_retry = 0.0
        # Pointer tracking: draw our own cursor only once the mouse has
        # actually moved, so a gun that is not driving HID does not leave a
        # crosshair parked in the corner pretending to be an aim point.
        self._mpos = None
        self._mseen = False
        # Where the crosshair comes from. On a platform with a real pointer the
        # OS moves it on the HID report, independently of this loop; drawing our
        # own there puts a SECOND crosshair on screen one frame behind the first
        # one, which is visible as lag next to the system cursor on the same
        # display. So on those platforms we hand our own artwork to the system
        # cursor instead of blitting it.
        #
        # kmsdrm (the Pi console) keeps the blit BY DEFAULT: there is no
        # compositor there, and a silently missing cursor on the platform that
        # needs one most is not worth a silent trade. But SDL's kmsdrm backend
        # does drive the DRM cursor plane, and the plane moves the moment
        # motion is PUMPED -- it does not wait for this loop to draw a frame.
        # PICAL_HWCURSOR=1 (or --hwcursor) opts in; paired with pump_wait()
        # below, the cursor then updates faster than the 60 fps draw rate.
        # The headless drivers never take this path.
        self._sys_cursor = False
        self._sys_shown = None
        self._pump_wait = False
        drv = pygame.display.get_driver()
        if drv not in ("kmsdrm", "dummy", "offscreen"):
            self._sys_cursor = install_cursor(self.sc.h)
        elif drv == "kmsdrm" and os.environ.get("PICAL_HWCURSOR") == "1":
            self._sys_cursor = install_cursor(self.sc.h)
            self._pump_wait = self._sys_cursor
            if self._sys_cursor:
                # The success is logged too: without this line a run with the
                # mode on and a run without it produce identical logs, and
                # "no difference on the delay" cannot be told apart from
                # "the switch never took".
                print("pical: hardware cursor on the DRM plane, pump-paced")
            if not self._sys_cursor:
                # The user asked for this by name; a silent fallback would
                # look identical to the mode working. The launcher log keeps
                # this line.
                print("pical: HWCURSOR requested but the cursor plane "
                      "refused the cursor; using the drawn one")
        # The app's own frame interval, measured, so display lag stops being a
        # guess: current and the worst over the last second, in draw_hud.
        self._t_frame = None
        self._frame_hist = []
        self.view = Menu(self)

    # ---- plumbing --------------------------------------------------------
    def log(self, msg):
        self.log_lines.append(str(msg))
        del self.log_lines[:-200]

    def toast_now(self, msg):
        self.toast = str(msg)
        self.toast_t = time.time()
        self.log(msg)

    def reboot_tick(self):
        """Notice a gun that came back as a DIFFERENT boot, and start over.

        A reconnect is not by itself a reboot: a USB port that re-enumerated
        hands back the same gun, still running, with its capture and its
        report format exactly as they were. A gun that RESTARTED comes up with
        the capture off, the format at whatever flash holds, and its
        histograms empty -- and nothing on the sweep screen notices. It goes
        on polling because the gun still answers '~camfit?'; every answer says
        ledn=0 and NEEDS MORE LED DATA; the screen goes on saying "keep
        sweeping" at a gun that is not measuring anything, at 0 of 500, for
        as long as the user is willing to wave it about.

        Worse quietly: the borrow taken before the reboot describes a gun that
        no longer exists. Handing that format back writes a report mode into a
        gun that never had it, and switching "its" capture off switches off
        one this app never started.

        Told apart by the gun's own clock, which restarts near zero on a
        reboot and simply carries on otherwise. Decided once, when a frame has
        actually arrived to decide from -- guessing early would be the same
        mistake as not looking.
        """
        if self._gun_t0 is None:
            return
        gt = self.link.gun_t
        if gt <= 0.0:
            return                    # no frame since the reconnect yet
        # A second of slack, because the clock is read off a frame that may
        # have been in flight across the reconnect.
        rebooted = gt < self._gun_t0 - 1.0
        self._gun_t0 = None
        if not rebooted:
            return
        held = self._cam_borrow is not None or self._cam_arm is not None
        # Dropped, never restored: nothing is SENT to put the old state back,
        # because the gun that would receive it is not the gun the state was
        # recorded from.
        self._cam_borrow = self._cam_arm = None
        self.fit.reset()
        self.link.hists.reset()
        if held:
            # A screen is still asking for the measurement, so start it again
            # rather than leaving it pointed at a gun that has stopped
            # collecting. The bars go back to 0 of 500 either way; the
            # difference is whether they ever move again.
            self.cam_borrow()
            self.toast_now("the gun RESTARTED -- everything it had measured "
                           "is gone, and this is starting over from empty")
        else:
            self.toast_now("the gun RESTARTED -- anything it had measured is "
                           "gone")

    def cam_borrow(self):
        """Take what the LED measurement needs -- after asking what is there.

        Full report mode and the learning capture. The gun's confirmed frames
        are free data: every frame in which the resolver locks all four
        corners is a measurement of what THIS bar's LEDs look like, and a
        calibration is minutes of nothing but such frames. That was being
        thrown away, and both tools instead told the user to switch the
        capture on by hand on another screen -- so in practice the LED side of
        '~camfit' was only ever filled by somebody who already knew what the
        feature was.

        Two steps, and the order is the whole point. The one thing that has to
        be known is whether the capture was ALREADY running, and it can only
        be learnt BEFORE arming: once '~camlearn=on:1' has gone out the gun
        answers on=1 whatever it was doing before, and there is no way left to
        tell whether the capture stopped on the way out was pical's own or the
        user's. link.hists.running() cannot answer it either -- only the
        camera screen ever polls '~camlearn?', so from the menu that flag is
        minutes old or has never been set at all. So this ASKS, and cam_tick
        completes the borrow when the gun answers.

        Nothing here blocks. The answer is thirteen lines and up to 2.6 KB,
        which is a fifth of a second of wire; waiting for it inside a key
        press would freeze the screen, and a frozen screen is how a Pi looks
        when it has crashed.

        Idempotent, and that is correctness rather than tidiness: a
        calibration borrows, and then the room sweep offered at the end of it
        borrows again. A second borrow must not re-remember -- it would record
        the borrowed state as the user's own and never give the real one back.
        """
        link = self.link
        if not link.src or self._cam_arm is not None:
            return
        if self._cam_borrow is not None:
            # Already held. Re-assert the format if something has moved it
            # since, but never the capture: on:1 is safe to repeat (the
            # firmware clears on the off->on edge only) and re-sending it
            # would buy nothing but wire.
            if link.last.get("fmt", link.last.get("ext")) != 2:
                link.send("~cam=fmt:2")
            return
        self._cam_arm = (link.hists.seq, time.monotonic())
        link.send("~camlearn?")

    def cam_tick(self, now_m):
        """Finish a borrow once the gun has said what it was holding.

        Or once it plainly is not going to: a gun on firmware without the
        capture never answers, and a step that waited for ever would collect
        nothing and say nothing about why. Then the borrow goes ahead with
        "was it running" UNKNOWN, and unknown means the capture is left
        running on the way out -- a capture nobody stopped is visible on the
        camera screen and costs nothing, where stopping one that was somebody
        else's measurement is invisible.
        """
        if self._cam_arm is None:
            return
        seq0, asked = self._cam_arm
        link = self.link
        fresh = link.hists.seq != seq0 and link.hists.ready()
        if not fresh and (now_m - asked) < CAM_ARM_S:
            return
        running = link.hists.running() if fresh else None
        self._cam_arm = None
        self._cam_borrow = (link.last.get("fmt", link.last.get("ext")), running)
        if running is False:
            # Arming an idle capture CLEARS it, so whatever was measured
            # before is gone and any verdict drawn from it is now a fiction.
            # Same gun, though, so what it can answer is not forgotten.
            self.fit.reset(new_gun=False)
            _f, led, _r = link.hists.counts()
            if led:
                # Only when there was something to lose. The shape CSV is the
                # only copy of a capture and nothing else warns; said here
                # rather than on each screen that borrows, so no screen added
                # later can forget to say it.
                self.toast_now("the shape capture restarted from empty -- the "
                               "%d LED blobs measured before this are gone"
                               % led)
        if link.last.get("fmt", link.last.get("ext")) != 2:
            link.send("~cam=fmt:2")
        # Sent whatever the answer was, including when there was none: the
        # firmware clears on the off->on edge ONLY, so repeating it at an
        # already-running capture is a no-op rather than a wipe, and a gun too
        # old to have the capture drops the key in silence.
        link.send("~camlearn=on:1")

    def cam_restore(self, keep_full=False):
        """Give back what cam_borrow took. Safe to call twice.

        `keep_full` is for the one case where giving it back would undo the
        thing the user just asked for: a ceiling applied by the room sweep
        only ACTS in full report mode, so restoring a gun that came in on
        fmt:1 left the number in flash and the gate stone dead -- while the
        screen said it would hold through a power cycle, which was true of the
        number and false of everything the user cared about.
        """
        if self._cam_arm is not None and self._cam_borrow is None:
            # Asked, never armed. Nothing was changed, so there is nothing to
            # put back -- and the pending question must not survive into
            # whatever screen comes next.
            self._cam_arm = None
            return
        if self._cam_borrow is None:
            return
        fmt, running = self._cam_borrow
        self._cam_borrow = None
        # Only a capture we can PROVE we started. A capture that was already
        # running is the user's, and one whose state the gun never reported
        # might be: disabling it does not destroy what it has measured, but it
        # does end a measurement somebody may be in the middle of.
        if running is False:
            self.link.send("~camlearn=on:0")
        if keep_full:
            return
        # None means the gun never said what report mode it was in, and a
        # guess written into it is a silent edit. Leave it as it is.
        if fmt is not None and fmt != 2:
            # 'ext:', never 'fmt:': both firmware generations take ext, while
            # the older one drops fmt in silence and would be left in
            # whatever mode this app put it in.
            self.link.send("~cam=ext:%d" % fmt)

    def save_cam(self, **want):
        """Write the live camera settings to the gun and SAY WHAT HAPPENED.

        Every screen that saves goes through here so the answer is the gun's
        own, not this app's optimism: the reply says whether the flash write
        returned OK and carries the values that were written, which are then
        compared against what the screen asked for. `want` is optional -- pass
        the fields the screen actually changed."""
        ok, msg = camsave_verified(self.link.src, **want)
        self.toast_now(msg)
        return ok

    def no_data(self):
        return (self.link.src is not None and self.link.frames == 0
                and (time.time() - self.t_open) > NO_DATA_S)

    def connect(self):
        self.toast_now("looking for the gun...")
        self.draw()
        pygame.display.flip()
        self.link.close()
        # A verdict is a measurement of one gun in one room, and the gun that
        # answers next may be another one entirely -- or the same one after a
        # power cycle, which empties its histograms. Link.connect clears its
        # own state for exactly this reason; this is the same rule.
        self.fit.reset()
        if self.link.connect():
            self.t_open = time.time()
            self.link.send("~ping")
            self.link.send("~cam?")
            self.link.send("~fx?")     # fx_calm and the Recoil screen need it
            self.toast_now("connected on %s" % self.link.port)
        else:
            self.toast_now("no gun answered on any serial port")

    def open(self, view):
        self.view = view

    def to_menu(self):
        self.fx_quiet(False)
        # A screen that was writing a file does not get to keep it open once
        # nobody is looking at it.
        closer = getattr(self.view, "close_log", None)
        if closer:
            closer()
        # ...and one that borrowed a setting from the gun gives it back. The
        # room sweep needs full report mode and the learning capture, and both
        # are things a user may deliberately have had off; left switched on
        # they are a silent edit with nothing anywhere to say where it came
        # from. Same rule, and same shape, as the recoil restore.
        back = getattr(self.view, "restore", None)
        if back:
            back()
        # And the app-level borrow, for the steps that span more than one
        # screen. A no-op when the view above has already handed it back --
        # which is how the room sweep gets to keep full mode when the user
        # applied a ceiling, without this line taking it away again.
        self.cam_restore()
        self.session = None
        self.link.sink = None
        self.link.trig_sink = None
        self.view = Menu(self)

    def quit(self):
        # Leaving must never leave the gun quiet: on firmware that has quiet
        # mode it would lapse by itself, but waiting five minutes for your
        # recoil to come back is not an answer.
        self.fx_quiet(False)
        self.running = False

    # ---- steps -----------------------------------------------------------
    def fx_quiet(self, on):
        """Silence the solenoid AND the rumble motor for a measurement.

        Both matter: a strike swings the gun and the motor shakes the camera,
        and a calibration is a measurement of where the gun was pointing.

        Firmware that has quiet mode does this with one switch that also takes
        the motor away from OpenFIRE's own off-screen rumble -- the half the
        old approach could never do, which is why rumble kept firing through
        calibrations. Older firmware gets the best available imitation, and is
        told so rather than left to look silent while it is not."""
        link = self.link
        if not on:
            # Intent is recorded FIRST and unconditionally. A gun that is
            # mid-reboot when the user leaves a calibration used to keep
            # `wanted` set, and the reconnect then re-armed quiet on a gun
            # nobody was calibrating any more -- silent, with the only
            # indicator on a screen the user had just left.
            self._fx_quiet_want = False
        if not link.src:
            return
        plan = quiet_plan(link.last)
        self._fx_plan = plan
        if plan == "none":
            if on:
                # Not silently: on a gun that should have a recoil engine this
                # means the ~fx? answer never arrived, and the calibration is
                # about to run with a live solenoid.
                self.log("no ~fx answer from this gun -- recoil will NOT be "
                         "quieted for this run")
            return                            # no recoil engine on this gun
        # Arming twice without an intervening restore would re-read the saved
        # values from the gun -- which by then holds the QUIET ones, so the
        # user's real settings would be replaced by them permanently.
        if on and self._fx_quiet_want:
            if plan == "quiet":
                link.send("~fx=quiet:1")
                self._fx_quiet_t = time.time()
            return
        if plan == "quiet":
            ok = link.send("~fx=quiet:%d" % (1 if on else 0))
            if not on:
                pass                          # intent already cleared above
            elif ok:
                self._fx_quiet_want = True
                self._fx_quiet_t = time.time()
            else:
                self.toast_now("could not reach the gun to quieten it")
            return
        # ---- older firmware ------------------------------------------------
        if not on:
            self.fx_restore()
            return
        if plan == "legacy":
            keys = ("on", "drive", "hold", "pulse", "rumms")
            self._fx_saved = {k: link.last.get("fx" + k) for k in keys}
            msg = ("older firmware: recoil quieted the old way -- reflash for "
                   "a fully silent calibration")
        else:
            # "stuck": these ARE quiet values already. Saving and restoring
            # them would write yesterday's calibration state back as if the
            # user had chosen it, which is how it became permanent in the
            # first place. Quieten, keep nothing, and say what is wrong.
            self._fx_saved = None
            msg = ("the gun still holds calibration-quiet recoil values -- "
                   "set them on screen 7 and Save")
        # rumms:1, never 0. A zero rumble time gives the motor BACK to
        # OpenFIRE, whose off-screen rumble is exactly what fired through
        # every calibration shot; 1 ms keeps it ours and is imperceptible.
        link.send("~fx=on:1,drive:5,hold:0,pulse:0,rumms:1")
        self._fx_quiet_want = True
        # One toast, and it is the SPECIFIC one: a second call here overwrote
        # the stuck-values warning with the generic line, so the message that
        # actually named a problem was the one nobody ever saw.
        self.toast_now(msg)

    def fx_restore(self):
        """Put the user's recoil settings back, and CHECK that they landed.

        Returns False when the write could not be sent -- and KEEPS the saved
        values in that case, so a later attempt can still put them back. The
        old version dropped them either way, which turned a write into a dead
        port into a permanently lost recoil setup."""
        if self._fx_saved is None:
            return True
        parts = ["%s:%d" % (k, v) for k, v in self._fx_saved.items()
                 if v is not None and k != "on"]
        on = self._fx_saved.get("on")
        if on is not None:
            parts.append("on:%d" % on)      # the on-switch goes back LAST
        if not parts:
            self._fx_saved = None
            return True
        if not self.link.send("~fx=" + ",".join(parts)):
            self.toast_now("could not reach the gun to put the recoil "
                           "settings back -- they are still remembered")
            return False
        # Ask, then check the answer a moment later. A restore that was
        # written into a dead port looks identical to one that worked, and
        # the cost of not noticing is a gun that never recoils again.
        self.link.send("~fx?")
        self._fx_verify = (dict(self._fx_saved), time.time() + 1.5)
        self._fx_saved = None
        return True

    def fx_tick(self, now):
        """Hold the quiet switch down, and confirm a restore actually took.

        Firmware quiet mode expires by itself -- that is what stops a crashed
        app from muting a gun forever -- so an app that still wants silence has
        to keep asking for it."""
        if (self._fx_quiet_want and self._fx_plan == "quiet"
                and now - self._fx_quiet_t > QUIET_REARM_S):
            self._fx_quiet_t = now
            self.link.send("~fx=quiet:1")
        # A restore that could not be sent is still owed. Retry it once the gun
        # is back, rather than leaving the user's recoil settings sitting in
        # this app's memory waiting for someone to notice.
        if (self._fx_saved is not None and not self._fx_quiet_want
                and self.link.src and now - self._fx_retry > 3.0):
            self._fx_retry = now
            self.fx_restore()
        if self._fx_verify and now > self._fx_verify[1]:
            want, _ = self._fx_verify
            self._fx_verify = None
            bad = [k for k, v in want.items()
                   if v is not None and self.link.last.get("fx" + k) != v]
            if bad:
                self.toast_now("recoil settings did NOT go back (%s) -- check "
                               "screen 7" % ",".join(sorted(bad)))

    def begin_calib(self):
        if not self.link.src:
            self.toast_now("connect the gun first")
            return
        # Only once a gun is truly here: quieting before the guard captured a
        # stale saved-state that later clobbered freshly tuned recoil knobs.
        self.fx_quiet(True)
        # And the free half of the shape measurement. A calibration is minutes
        # of frames in which the resolver has locked all four corners, and
        # every blob in such a frame is a confirmed LED -- which is exactly
        # what '~camfit' needs 500 of before it will name a ceiling. It used
        # to be discarded and the user told to switch the capture on by hand
        # on another screen, so in practice the LED side was only ever filled
        # by somebody who already knew what the feature was.
        self.cam_borrow()
        self.session = CaptureSession(plan=make_plan(self.stances, 0))
        # Fullscreen: a window fraction IS a screen fraction.
        self.session.to_screen = lambda fx, fy: (fx, fy)
        self.session.geom_note = "pical fullscreen %dx%d" % (self.sc.w, self.sc.h)
        self.link.sink = lambda q, gt: self.session.feed(q, gt)
        self.link.trig_sink = self.on_trigger
        self.view = Calib(self, self.session)

    def begin_room_sweep(self):
        """Open the room-light step, or say plainly why there is nothing to
        open. Refusing quietly here would look exactly like a dead row."""
        if not self.link.src:
            self.toast_now("connect the gun first")
            return
        if "wiicam" not in self.link.last.get("board", ""):
            self.toast_now("this step measures the wiicam sensor -- this gun "
                           "has nothing here to measure")
            return
        self.view = RoomSweep(self)

    def begin_finetune(self):
        c = self.read_gun_calib()
        if c is None:
            return
        lead = int(self.link.last.get("lead", 0) or 0)
        smooth = int(self.link.last.get("smooth", 3) or 3)
        beta = int(self.link.last.get("beta", -1))
        t = Tuner(c, lead, smooth, beta)
        view = FineTune(self, t)
        self.link.sink = lambda q, gt: view.feed_quad(q)
        self.link.trig_sink = view.shoot
        self.view = view

    def begin_verify(self):
        c = self.read_gun_calib()
        if c is None:
            return
        # Verify MEASURES the calibration, so the same rule applies: a strike
        # or a rumble during the nine shots is noise added to the number the
        # whole screen exists to report.
        self.fx_quiet(True)
        view = Verify(self, c)
        self.link.sink = view.feed
        self.link.trig_sink = lambda: view.handle(["trigger"], (0, 0))
        self.view = view

    def read_gun_calib(self, timeout=2.5):
        """Ask the gun what calibration it holds; it is the source of truth.

        The reply is taken from the reader thread's own buffer rather than by
        reading the port here -- that thread already owns the port, and a
        second reader would race it for the same bytes.
        """
        src = self.link.src
        if src is None:
            self.toast_now("connect the gun first")
            return None
        try:
            src.replies.clear()
            src.ser.write(b"\n~aimcal?\n")
        except Exception as e:
            self.toast_now("could not ask the gun: %s" % e)
            return None
        t0 = time.time()
        while (time.time() - t0) < timeout:
            for r in list(src.replies):
                if not r.startswith("AIM:") or "cx=" not in r:
                    continue
                g = {}
                for tok in r.replace("AIM:", "").split():
                    if "=" in tok:
                        k, _, v = tok.partition("=")
                        try:
                            g[k] = float(v)
                        except ValueError:
                            pass
                if all(k in g for k in ("cx", "cy", "w", "h", "bx", "by")):
                    return dict(magic=aim_fit.MAGIC, cx=g["cx"], cy=g["cy"],
                                w=g["w"], h=g["h"], bx=g["bx"], by=g["by"],
                                lever=g.get("lever", 0.0), rx=g.get("rx", 0.0),
                                ry=g.get("ry", 0.0))
            time.sleep(0.05)
        self.toast_now("the gun has no calibration yet -- run step 4 first")
        return None

    def on_trigger(self):
        if self.session and self.session.state != self.session.S_DONE:
            self.session.note_trigger()   # or auto-capture decides we have none
            self.session.trigger(self.link.gun_t or time.time(), pulled=True)

    def finish_calib(self):
        self.fx_quiet(False)
        s = self.session
        c, why = s.fit()
        saved = None
        try:
            # The session numbers its own two files now, both under ONE number
            # because they are two halves of one capture. This used to move
            # them onto pical's numbering afterwards, from a time when
            # CaptureSession named them from the clock alone -- and once the
            # session started numbering them, that renaming burned a sequence
            # number on every calibration and the count advanced 2, 4, 6.
            saved = s.save(OUT_DIR)[0]
        except Exception as e:
            self.toast_now("could not write the log: %s" % e)
        install = ""
        if c:
            cmd = aimcal_line(c)
            if self.link.src is not None:
                try:
                    install = install_over_serial(self.link.src, cmd, c)
                except Exception as e:
                    install = "NOT SENT -- %s" % e
            if saved:
                try:
                    with open(os.path.join(os.path.dirname(saved), "aimcal.txt"),
                              "w") as f:
                        f.write(cmd + "\n")
                except Exception:
                    pass
        self.session = None
        self.link.sink = None
        self.link.trig_sink = None
        self.view = Result(self, c, why, install, saved)

    # ---- shared chrome ---------------------------------------------------
    def draw_hud(self, sc, extra=""):
        link = self.link
        parts = ["%d frames" % link.frames, "%.0f Hz" % link.fps()]
        # The app's OWN loop interval -- the gun Hz above is the serial stream,
        # which says nothing about how often this screen redraws. The drawn
        # cursor can never be fresher than this number.
        if self._frame_hist:
            dts = [d for _, d in self._frame_hist]
            parts.append("app %.0f ms (worst %.0f)"
                         % (1000.0 * sum(dts) / len(dts), 1000.0 * max(dts)))
        if link.span():
            parts.append("span %.0f px" % link.span())
        if extra:
            parts.append(extra)
        sc.text(sc.w * 0.02, sc.h * 0.975, "   ".join(parts), sc.f_xs, C_DIM,
                centre=False)
        # Hung by its RIGHT edge, because 0.02 of the width is not enough room
        # for the word at the sizes this actually runs at: drawn from 0.98 w
        # leftwards it ran 5 px off a 1024 px screen -- the Pi's own mode --
        # and 9 px off a 640 px one, losing the 'c' on the display nobody has
        # a console beside.
        sc.text(sc.w * 0.98 - sc.f_xs.size("Esc")[0], sc.h * 0.975, "Esc",
                sc.f_xs, C_DIM, centre=False)

    def draw_preview(self, sc, x, y, w, h, rings=False, trail=None,
                     label="CAMERA VIEW"):
        """What the sensor sees: the four LEDs, in sensor coordinates.

        Every number in this app is derived from these four dots, so showing
        them turns tuning from guesswork into something you can watch. The
        trail of quad centres makes instability visible; the rings mark the
        radius a lens sweep has to push the LEDs past.

        Returns the rectangle it covered, its caption included, so a screen
        that stacks it above something else can be measured rather than
        trusted -- the camera page puts the sensor panel directly underneath.
        """
        link = self.link
        pad = 4
        xi, yi, wi, hi = int(x), int(y), int(w), int(h)
        covered = pygame.Rect(xi, int(y - sc.h * 0.026
                                      - sc.f_xs.get_height() / 2.0),
                              wi, 0)
        covered.height = yi + hi - covered.top
        pygame.draw.rect(sc.s, (16, 20, 26), (xi, yi, wi, hi))
        pygame.draw.rect(sc.s, (48, 54, 61), (xi, yi, wi, hi), 1)
        sc.text(x + w / 2, y - sc.h * 0.026, label, sc.f_xs, C_DIM)
        kx = (w - 2 * pad) / FRAME_W
        ky = (h - 2 * pad) / FRAME_H

        def to_px(p):
            return (x + pad + p[0] * kx, y + pad + p[1] * ky)

        ccx, ccy = to_px((CX, CY))
        if rings:
            # The coverage gate, drawn. The fit refuses a sweep whose LEDs
            # never reached this circle, so put the circle on the screen
            # instead of only mentioning it in the refusal.
            for frac, col in ((COV_GATE, C_WARN), (1.0, (44, 50, 58))):
                rx, ry = frac * min(CX, CY) * kx, frac * min(CX, CY) * ky
                pygame.draw.ellipse(sc.s, col,
                                    pygame.Rect(int(ccx - rx), int(ccy - ry),
                                                int(2 * rx), int(2 * ry)), 1)
        if trail is not None:
            # The coverage map is painted once, point by point, as the sweep
            # runs, then blitted scaled -- redrawing thousands of pixels every
            # frame would cost more than the sweep can afford on a Pi.
            sc.s.blit(pygame.transform.scale(
                trail, (int(w - 2 * pad), int(h - 2 * pad))), (xi + pad, yi + pad))

        now = time.time()
        dropping = (now - link.partial_t) < 1.0 and (now - link.full_t) > 0.5
        if not link.hist:
            msg = ("seeing LEDs, but not all four" if dropping
                   else "no four-LED frames")
            sc.text(x + w / 2, y + h / 2 - sc.h * 0.015, msg, sc.f_s,
                    C_WARN if dropping else C_BAD)
            if dropping:
                sc.text(x + w / 2, y + h / 2 + sc.h * 0.015,
                        "background IR too high, or too far", sc.f_xs, C_DIM)
            return covered
        if dropping:
            sc.text(x + w / 2, y + sc.h * 0.022, "DROPPING OUT -- last good frame",
                    sc.f_xs, C_WARN)
        for _, qq in link.hist[-60:]:
            px, py = to_px(qq.mean(0))
            if xi < px < xi + wi and yi < py < yi + hi:
                sc.s.set_at((int(px), int(py)), (31, 111, 235))
        q = aim_fit.canon(link.hist[-1][1])
        pts = [to_px(q[i]) for i in range(4)]
        for a, b in ((0, 1), (1, 3), (3, 2), (2, 0)):
            pygame.draw.line(sc.s, (58, 127, 191), pts[a], pts[b], 1)
        for i, nm in enumerate(("TL", "TR", "BL", "BR")):
            pygame.draw.circle(sc.s, (255, 210, 74),
                               (int(pts[i][0]), int(pts[i][1])), 4)
            sc.text(pts[i][0] + sc.w * 0.012, pts[i][1], nm, sc.f_xs, C_DIM)
        pygame.draw.line(sc.s, C_RING, (ccx - 6, ccy), (ccx + 6, ccy), 1)
        pygame.draw.line(sc.s, C_RING, (ccx, ccy - 6), (ccx, ccy + 6), 1)
        return covered

    def draw_cursor(self, sc):
        """Show the pointer -- as the system cursor where there is one, else drawn.

        Same pointer either way: the gun drives it over USB HID. The Pi console
        has no cursor of its own to show, so it is drawn there. On a desktop the
        system cursor carries our crosshair artwork instead, because a blitted
        copy can only move when this loop draws a frame and therefore trails the
        real pointer by a frame -- which is exactly what it looks like when both
        are on the same screen.

        Some screens deliberately do not want it: during a lens sweep and a dot
        capture the cursor is driven by the stock aim or by a calibration that is
        about to be replaced, so showing it would be a lie.
        """
        want = (self._mseen and self.link.hid_on
                and not getattr(self.view, "hide_cursor", False))
        if self._sys_cursor:
            # set_visible on every frame would spam SDL; only on a change
            if want != self._sys_shown:
                pygame.mouse.set_visible(bool(want))
                self._sys_shown = want
            return
        if not want:
            return
        x, y = pygame.mouse.get_pos()
        sc.crosshair(x, y, sc.h * 0.022, C_RING, dot=True)

    def draw_noise(self, sc, y):
        """The blob noise floor and what it means, in one line."""
        link = self.link
        sg = link.sigma()
        if sg is None:
            sc.text(sc.w / 2, y, "blob noise: hold the gun still to measure",
                    sc.f_s, C_DIM)
            return
        good, ok = sigma_gates(link.last.get("board"))
        col = C_OK if sg <= good else (C_WARN if sg <= ok else C_BAD)
        verdict = ("good, ready to calibrate" if sg <= good else
                   "usable, could be better" if sg <= ok else
                   "too noisy -- tune before calibrating")
        sc.text(sc.w / 2, y, "blob noise %.3f px   --   %s" % (sg, verdict),
                sc.f_s, col)

    def draw_no_data(self, sc):
        sc.text(sc.w / 2, sc.h * 0.30, "NO DATA FROM THE GUN", sc.f_l, C_BAD)
        sc.lines(sc.w / 2, sc.h * 0.44, [
            "Nothing has arrived on %s for %.0f seconds."
            % (self.link.port, time.time() - self.t_open),
            "",
            "Are all four LEDs in view? Frames with fewer are discarded.",
            "Is another program holding the port?",
            "",
            "Esc returns to the menu.",
        ], sc.f_s, C_DIM)

    def draw(self):
        self.sc.s.fill(C_BG)
        self.view.draw(self.sc)
        self.draw_link_banner()
        self.draw_cursor(self.sc)
        if self.toast and (time.time() - self.toast_t) < 5.0:
            sc = self.sc
            img = sc.f_s.render(self.toast, True, C_WARN)
            r = img.get_rect()
            r.center = (sc.w // 2, int(sc.h * 0.945))
            pygame.draw.rect(sc.s, (18, 22, 28), r.inflate(30, 14))
            sc.s.blit(img, r)

    # ---- the link itself -------------------------------------------------
    def link_tick(self, now):
        """Notice when the gun stops being there, and try to get it back.

        The reader thread breaks out of its loop on any serial error -- a gun
        that reboots, a USB port that re-enumerates -- and then simply ends.
        Everything on screen keeps its last value and every key still appears
        to work, because writes into a dead port fail silently. That is
        indistinguishable from a screen whose controls have broken, and it is
        what "the arrows do not change anything" looks like from the couch.
        """
        if now - self._link_t < 1.0:
            return
        self._link_t = now
        bad = bool(self.link.src) and not self.link.alive()
        if bad and not self._link_bad:
            self.toast_now("lost the gun on %s -- reconnecting" % self.link.port)
        self._link_bad = bad
        if not bad:
            return
        # Retry on a slow cadence: the gun may be mid-reboot, and hammering
        # the port scan makes the screen stutter for no gain.
        if now - self._link_retry < 5.0:
            return
        self._link_retry = now
        # The old port name first, then a rescan: a gun that re-enumerated
        # comes back under a different name, which is the common case here.
        port = self.link.port
        self.link.close()
        self.fit.reset()                  # see connect(): it may be a new gun
        # Taken BEFORE the reconnect, because Link.connect clears everything
        # it knew about the previous gun. The gun's own frame clock is one of
        # the few things it keeps, and it is what says whether the gun that
        # answers next is the same boot.
        was_t = self.link.gun_t
        if self.link.connect(port):
            self.t_open = now
            self.link.send("~ping")
            self.link.send("~cam?")
            self.link.send("~fx?")
            self._link_bad = False
            self._gun_t0 = was_t if was_t > 0 else None
            self.toast_now("gun back on %s" % self.link.port)
            # A gun that rebooted has forgotten it was asked to be quiet, and
            # a reconnect during a calibration must not resume with a live
            # solenoid on the next pull.
            if self._fx_quiet_want:
                self._fx_quiet_t = 0.0

    def draw_link_banner(self):
        """One red line, on every screen, whenever the gun is not reachable.

        Also the place the stale-tools warning lands: a stick that received a
        new pical.py and kept an old tools/ produces features that quietly do
        nothing, and that has cost a whole test session before."""
        sc = self.sc
        msg = None
        if not LINK_API_OK:
            msg = ("tools/ on this stick is OLDER than pical.py -- copy the "
                   "whole tools folder across, several screens will not work")
        elif self._link_bad:
            msg = ("GUN LINK LOST on %s -- nothing you change here is reaching "
                   "the gun. Reconnecting..." % (self.link.port or "?"))
        if not msg:
            return
        img = sc.f_s.render(msg, True, C_BAD)
        r = img.get_rect()
        r.center = (sc.w // 2, int(sc.h * 0.035))
        pygame.draw.rect(sc.s, (40, 14, 16), r.inflate(30, 14))
        pygame.draw.rect(sc.s, C_BAD, r.inflate(30, 14), 1)
        sc.s.blit(img, r)

    # ---- one frame -------------------------------------------------------
    def step(self, events, now):
        if self._t_frame is not None and now > self._t_frame:
            self._frame_hist.append((now, now - self._t_frame))
            while self._frame_hist and self._frame_hist[0][0] < now - 1.0:
                self._frame_hist.pop(0)
            # Hard cap as well: a backward clock step (NTP, a hand-set date)
            # leaves future-stamped entries the window prune never reaches.
            del self._frame_hist[:-240]
        self._t_frame = now
        acts = self.inp.actions(events, now)
        mouse = pygame.mouse.get_pos()
        if self._mpos is not None and mouse != self._mpos:
            if not self._mseen:
                # One line, once: proof in the log that SDL delivers this
                # device's motion at all. Its absence after a session IS the
                # diagnosis -- an absolute mouse classified as a tablet or
                # touchscreen produces no pointer motion on a bare console,
                # and then no cursor of any kind can ever appear.
                print("pical: first pointer motion at %d,%d" % mouse)
            self._mseen = True
        self._mpos = mouse
        # Extra keys that only make sense on the fine-tune screen.
        if isinstance(self.view, FineTune):
            for e in events:
                if e.type == pygame.KEYDOWN and e.key == pygame.K_n:
                    self.view.finish_stage()
                elif e.type == pygame.KEYDOWN and e.key == pygame.K_s:
                    self.view.save_now()
        self.link_tick(now)
        self.link.pump()
        while self.link.replies:
            r = self.link.replies.pop(0)
            self.log(r)
            # The fit verdict arrives as two or three ordinary reply lines,
            # so it is assembled here where they are drained. A gun on older
            # firmware never sends one and FitReport simply stays empty.
            self.fit.feed(r)
            # The lines a user was TOLD to watch for must actually appear:
            # pical has no visible log outside the auto-tune overlay.
            if ("diag VERDICT" in r or "diag stream" in r
                    or r.startswith("FX: saved")
                    or r.startswith("FX: SAVE FAILED")
                    # A refusal must be as visible as a success: both of these
                    # otherwise read as a solenoid that has stopped working.
                    or r.startswith("FX: quiet mode is ON")
                    or r.startswith("FX: busy")
                    # A setting the gun REFUSED. It answers by name and leaves
                    # the old value in place, so the row goes on showing what
                    # is really there -- which looks exactly like an arrow
                    # that does nothing. The gate floors are the gun's OWN
                    # measured LED envelope now, so this is not a case the
                    # ladders can be built to avoid: a ceiling under what this
                    # rig measured is refused by name, and a refusal nobody
                    # sees is a control the user gives up on.
                    or "-- not set" in r
                    # A gate the gun took and cannot act on, which is the one
                    # state invisible from every row on the screen: the row
                    # reads back the value, the gun agrees, and the gate is
                    # judging nothing because it has no boxes to judge.
                    or "set but INERT" in r
                    # The capture starting over. The gun clears it when
                    # sensitivity or either sensor threshold changes, because
                    # a distribution spanning the change is two rigs averaged
                    # into one -- the right call, but it resets the progress
                    # bars under a user who has just nudged a slider, and
                    # without this they are left wondering why 400 LED blobs
                    # became 0. '~camreset' lands here too, for the same
                    # reason.
                    or r.startswith("CAM: learn cleared")
                    # And a save that did not reach flash. Every other save in
                    # this app is verified and reports itself; the one the
                    # room sweep fires to keep the report format is sent into
                    # the stream, so its failure has to be caught here or the
                    # ceiling comes back on the next power cycle with nothing
                    # anywhere to say why.
                    or r.startswith("CAM: SAVE FAILED")
                    or "refused" in r):
                self.toast_now(r.strip()[:110])
        # After the replies are drained, so the restore check reads the gun's
        # freshest answer rather than the one from the previous frame.
        self.fx_tick(now)
        # Before the borrow tick, so a gun that restarted has its stale borrow
        # dropped and a fresh one started in the SAME frame -- otherwise the
        # pending question from before the reboot would complete against the
        # new gun and record its post-boot state as the user's own.
        self.reboot_tick()
        # ...and for the same reason: a pending borrow is waiting on a
        # '~camlearn?' answer that pump() has only just delivered. monotonic,
        # not the wall clock this loop is handed: a Pi has no clock until NTP
        # corrects it, and a backward step would leave the deadline in the
        # future for ever.
        self.cam_tick(time.monotonic())
        if self.session is not None and self.session.state == self.session.S_DONE:
            self.finish_calib()
        self.view.handle(acts, mouse)
        if self.toast and (now - self.toast_t) > 5.0:
            self.toast = ""
        self.draw()


def pick_drm_device():
    """Point SDL at the DRM card that actually drives a screen.

    SDL's kmsdrm backend takes the first /dev/dri/card* it finds. On a Pi
    running vc4-kms-v3d there are two: one is the v3d render node with no
    connectors at all. Picking that one succeeds, draws nothing, and leaves
    the TV black with no error anywhere -- so choose by which card has a
    connector reporting "connected".
    """
    if os.environ.get("SDL_KMSDRM_DEVICE_INDEX"):
        return None                       # an explicit choice wins
    import glob
    cards = sorted(glob.glob("/dev/dri/card*"))
    if len(cards) < 2:
        return None                       # nothing to disambiguate
    for i, dev in enumerate(cards):
        name = os.path.basename(dev)
        for st in sorted(glob.glob("/sys/class/drm/%s-*/status" % name)):
            try:
                with open(st) as fh:
                    if fh.read().strip() != "connected":
                        continue
            except OSError:
                continue
            os.environ["SDL_KMSDRM_DEVICE_INDEX"] = str(i)
            return "%s (index %d), connector %s" % (
                dev, i, os.path.basename(os.path.dirname(st)))
    return None


def pump_wait(clock, pump):
    """Hold the ~60 fps pace -- and keep the pointer moving while holding it.

    SDL only reads the gun's motion when events are pumped, so under kmsdrm the
    hardware cursor moves at PUMP rate, not draw rate. clock.tick sleeps the
    whole gap in one block, freezing the cursor for 16 ms at a time; this
    spreads short sleeps with a pump after each, so the plane tracks the gun
    several times per frame. Pumped events stay queued for event.get() -- pump
    consumes nothing.

    The plain path stays clock.tick: extra pumps cost CPU a Zero 2 W does not
    have, and without the hardware cursor they buy nothing anyone can see.

    The deadline is carried across calls, like clock.tick's own: measured from
    ENTRY it would add a full 16.7 ms on top of each frame's work (and on top
    of flip's vblank wait), dropping the drawn UI to ~30 fps. Carried, the
    frame's work is subtracted from the wait, and a late frame pays no extra
    debt. Monotonic clock: wall time steps backwards when someone sets the
    date, and this loop must never sleep out that jump.
    """
    if not pump:
        clock.tick(60)
        return
    now = time.monotonic()
    deadline = getattr(pump_wait, "_next", now)
    if deadline < now:
        deadline = now                     # late frame: no wait, no debt
    pump_wait._next = deadline + (1.0 / 60.0)
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            return
        time.sleep(left if left < 0.004 else 0.004)
        pygame.event.pump()


def run(stances=3, windowed=False, port=None):
    # The HWCURSOR switch file is read HERE, not only by the launcher: the
    # launcher lives in the image's Linux filesystem, which a PC cannot edit,
    # while this file sits on the FAT partition next to pical.py -- the one
    # place every update already goes. A stick with a new pical.py and an old
    # launcher must still honour the switch.
    if os.path.isfile(os.path.join(HERE, "HWCURSOR")):
        os.environ["PICAL_HWCURSOR"] = "1"
    if not windowed:
        picked = pick_drm_device()
        if picked:
            print("pical: display device %s" % picked)
    pygame.init()
    flags = 0 if windowed else pygame.FULLSCREEN
    size = (1280, 720) if windowed else (0, 0)
    # set_mode FIRST: the mouse and caption calls need an initialised video
    # system, and calling them earlier turns a real driver error into a
    # misleading "video system not initialized".
    surf = pygame.display.set_mode(size, flags)
    pygame.mouse.set_visible(windowed)
    pygame.display.set_caption("Lightgun calibration")
    print("pical: SDL video driver in use: %s" % pygame.display.get_driver())
    app = App(surf, stances)
    if port is None:
        app.connect()
    else:
        if app.link.connect(port):
            app.t_open = time.time()
            app.link.send("~ping")
            app.link.send("~cam?")
            app.link.send("~fx?")
    if not LINK_API_OK:
        # Also on stdout, because the launcher keeps that log and a photo of
        # the TV is not always how this gets reported.
        print("pical: WARNING -- tools/ is older than pical.py (needs LINK_API "
              "%d, found %d); copy the whole tools folder to the stick"
              % (LINK_API_NEEDED, getattr(gun_studio, "LINK_API", 0)))
    clock = pygame.time.Clock()
    try:
        while app.running:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.QUIT:
                    app.running = False
            app.step(events, time.time())
            pygame.display.flip()
            pump_wait(clock, app._pump_wait)
    finally:
        # Whatever ended this run -- the menu, a window close, Ctrl-C, or an
        # exception on the way past -- the gun does not keep the state this app
        # put it in. A crash mid-calibration used to leave the recoil silenced
        # with no way to tell that was what had happened.
        try:
            closer = getattr(app.view, "close_log", None)
            if closer:
                closer()
            # And the settings a screen borrowed, for the same reason: a
            # window closed during the room sweep would otherwise leave the
            # gun in full report mode with a capture running, and the next
            # session would have no idea why.
            back = getattr(app.view, "restore", None)
            if back:
                back()
            app.cam_restore()
            app.fx_quiet(False)
            # Said on stdout, not through a toast: the frame loop has ended,
            # so nothing will ever draw one -- and the launcher keeps this log.
            if not app.fx_restore():
                print("pical: WARNING -- could not put the recoil settings "
                      "back; check screen 7 on the gun")
        except Exception as e:
            print("pical: could not restore the recoil settings: %s" % e)
        app.link.close()
        pygame.quit()
    return 0


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--windowed", action="store_true",
                    help="run in a window instead of fullscreen")
    ap.add_argument("--stances", type=int, default=3, choices=(2, 3),
                    help="distance stances for the calibration (default 3)")
    ap.add_argument("--port", help="serial port; probed when omitted")
    ap.add_argument("--hwcursor", action="store_true",
                    help="on a Pi console, move the cursor on the DRM hardware "
                         "plane at pump rate instead of drawing it at frame "
                         "rate (lower lag; costs CPU)")
    a = ap.parse_args()
    if a.hwcursor:
        os.environ["PICAL_HWCURSOR"] = "1"
    sys.exit(run(a.stances, a.windowed, a.port))


if __name__ == "__main__":
    main()
