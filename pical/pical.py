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
from gun_studio import (CAM_KEYS, CAM_RANGE, LENS_KEYS, Link, QUIET_REARM_S,
                        auto_tune, quiet_plan, sigma_gates)

# The shared serial layer lives in tools/ beside this file, and this file is
# routinely copied onto the stick ON ITS OWN. When only pical.py is updated,
# every feature whose parsing lives in tools/ stops working -- silently, in a
# way that looks exactly like a broken screen. So it is checked, out loud.
LINK_API_NEEDED = 3
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
    """

    def __init__(self, label, kind="button", act=None, get=None, set=None,
                 lo=0, hi=100, step=1, fmt="%d", hint=""):
        self.label = label
        self.kind = kind
        self.act = act
        self.get = get
        self.set = set
        self.lo, self.hi, self.step = lo, hi, step
        self.fmt = fmt
        self.hint = hint
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

    def nudge(self, d):
        if self.kind != "spin" or self.set is None:
            return
        v = self.value()
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
        for a in acts:
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
                v = r.value()
                txt = "--" if v is None else (r.fmt % v)
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
                sc.text(sc.w / 2, sc.h * 0.90, tip, sc.f_s, C_DIM)
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
        self._blob_ref = {}        # counter values at the last readout
        self._blob_last = ""       # keeps the last percentage on screen while
                                   # the next window fills, so it stops flashing
        self.build()

    def wiicam(self):
        b = self.app.link.last.get("board", "")
        return "wiicam" in b

    def build(self):
        link = self.app.link
        self.rows = []
        if self.wiicam():
            self.subtitle = ("the wiicam finds blobs in hardware -- "
                             "sensitivity, and what to do about stray light")
            self.rows.append(Row(
                "Sensitivity", "spin",
                get=lambda: link.last.get("sens", 0),
                set=lambda v: link.send("~cam=sens:%d" % v),
                lo=0, hi=2, step=1,
                hint="0 default, 2 highest. Use at least 1-2 with a wide lens."))
            self.rows.append(Row(
                "Sensor connection test", act=self.diag,
                hint="checks power, wiring, swapped lines and the sensor "
                     "itself; the log shows the verdict"))
            # Ambient light. The wiicam finds blobs in HARDWARE and reports
            # four slots: a bright window does not add a fifth point, it TAKES
            # one, and an LED goes missing. The only hardware fact that tells
            # them apart is blob SIZE, and only the extended report carries it.
            # Read the sizes first (the line under the preview), THEN set a
            # window -- a gate guessed at is worth nothing.
            self.rows.append(Row(
                "Blob detail (sizes)", "spin",
                get=lambda: link.last.get("ext"),
                set=lambda v: link.send("~cam=ext:%d" % v),
                lo=0, hi=1, step=1,
                hint="1 asks the sensor for each blob's size, so the readout "
                     "below can show it. Resets on power-cycle."))
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
        self.rows.append(Row("Save to gun", act=self.save,
                             hint="keeps these settings across power cycles"))
        self.rows.append(Row("Back", act=self.app.to_menu))

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
        # The blob readout is a live measurement, so it is re-read while this
        # screen is up rather than sampled once on the way in.
        if self.wiicam() and time.time() - self._blob_t > 1.0:
            self._blob_t = time.time()
            self.app.link.send("~camblob?")
        RowScreen.handle(self, acts, mouse)

    def blob_lines(self):
        """What the sensor is handing us right now, in words.

        Two numbers matter. The per-blob sizes say whether a window and an LED
        are even separable -- if they land on the same size, no gate can split
        them and the honest answer is a curtain, not a setting. The share of
        frames that lost a corner says how much the light is costing.
        """
        link = self.app.link
        out = []
        raw = getattr(link, "blobs", "")
        if raw:
            parts = raw.replace("CAM: blobs", "").strip()
            if "(sizes need ext:1)" in parts:
                out.append("blob sizes: set Blob detail to 1 to read them")
                parts = parts.replace("(sizes need ext:1)", "").strip()
            shown = []
            for tok in parts.split():
                f = tok.split(",")
                if len(f) == 4:
                    shown.append("%s,%s size %s%s"
                                 % (f[0], f[1], f[2],
                                    "" if f[3] == "1" else " DROPPED"))
            if shown:
                out.append("blobs now: " + "    ".join(shown))
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
        rej = link.last.get("brej")
        if rej:
            out.append("%d blobs dropped by the size window so far" % rej)
        return out

    def draw(self, sc):
        if self.tuning:
            sc.text(sc.w / 2, sc.h * 0.12, "AUTO TUNE", sc.f_l, C_WARN)
            sc.lines(sc.w / 2, sc.h * 0.26, self.app.log_lines[-12:], sc.f_s, C_DIM)
            sc.text(sc.w / 2, sc.h * 0.92, "Esc cancels", sc.f_s, C_DIM)
            return
        RowScreen.draw(self, sc)
        if self.wiicam():
            # Under the whole layout, centred like everything else: beside the
            # preview it ran off the right edge of the screen and over the
            # camera view, which on a TV is simply gone.
            sc.lines(sc.w / 2, sc.h * 0.735, self.blob_lines()[:3],
                     sc.f_xs, C_DIM)
        else:
            self.app.draw_noise(sc, sc.h * 0.74)


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
        A refusal with no data behind it cannot be diagnosed later."""
        if not len(snap):
            return ""
        try:
            os.makedirs(OUT_DIR, exist_ok=True)
            path = os.path.join(OUT_DIR,
                                time.strftime("lenssweep-%Y%m%d-%H%M%S.log"))
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
        self.session = CaptureSession(plan=make_plan(self.stances, 0))
        # Fullscreen: a window fraction IS a screen fraction.
        self.session.to_screen = lambda fx, fy: (fx, fy)
        self.session.geom_note = "pical fullscreen %dx%d" % (self.sc.w, self.sc.h)
        self.link.sink = lambda q, gt: self.session.feed(q, gt)
        self.link.trig_sink = self.on_trigger
        self.view = Calib(self, self.session)

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
        sc.text(sc.w * 0.98, sc.h * 0.975, "Esc", sc.f_xs, C_DIM, centre=False)

    def draw_preview(self, sc, x, y, w, h, rings=False, trail=None,
                     label="CAMERA VIEW"):
        """What the sensor sees: the four LEDs, in sensor coordinates.

        Every number in this app is derived from these four dots, so showing
        them turns tuning from guesswork into something you can watch. The
        trail of quad centres makes instability visible; the rings mark the
        radius a lens sweep has to push the LEDs past.
        """
        link = self.link
        pad = 4
        xi, yi, wi, hi = int(x), int(y), int(w), int(h)
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
            return
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
        if self.link.connect(port):
            self.t_open = now
            self.link.send("~ping")
            self.link.send("~cam?")
            self.link.send("~fx?")
            self._link_bad = False
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
            # The lines a user was TOLD to watch for must actually appear:
            # pical has no visible log outside the auto-tune overlay.
            if ("diag VERDICT" in r or "diag stream" in r
                    or r.startswith("FX: saved")
                    or r.startswith("FX: SAVE FAILED")
                    # A refusal must be as visible as a success: both of these
                    # otherwise read as a solenoid that has stopped working.
                    or r.startswith("FX: quiet mode is ON")
                    or r.startswith("FX: busy")):
                self.toast_now(r.strip()[:110])
        # After the replies are drained, so the restore check reads the gun's
        # freshest answer rather than the one from the previous frame.
        self.fx_tick(now)
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
