#!/usr/bin/env python3
"""pical -- lightgun calibration without a PC.

A pygame front end over the same capture session, fit and serial dialect the
desktop tools use (tools/aim_calib.py, tools/aim_fit.py). It runs on Windows
for development, as a Batocera port, and as the boot target of the USB image
in pical/image/.

The gun is aimed by its IRON SIGHTS and read over serial, so a calibration
that is completely offset does not prevent recalibrating. Menus accept a
game controller, a mouse and a keyboard; the gun's own trigger drives the
capture.
"""
import math
import os
import queue
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# The desktop tools are the single source of truth for capture and fit.
for _p in (os.path.join(HERE, "..", "tools"), os.path.join(HERE, "tools")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import pygame

import aim_fit
from aim_calib import (CaptureSession, SerialSource, aimcal_line, find_gun,
                       install_over_serial, is_trigger, make_plan, parse_q)

# Where sessions are written. On the USB image this is the FAT boot partition,
# so the logs can be read from any PC.
OUT_DIR = os.environ.get("PICAL_OUT", os.path.join(HERE, "calib_out"))
NO_DATA_S = 5.0                 # silence this long on a port -> say so
BAUD = 115200

C_BG = (10, 12, 16)
C_FG = (230, 237, 243)
C_DIM = (125, 133, 144)
C_OK = (57, 194, 110)
C_WARN = (216, 161, 58)
C_BAD = (210, 75, 75)
C_RING = (224, 122, 95)


# ---------------------------------------------------------------------------
# drawing helpers
# ---------------------------------------------------------------------------
class Screen:
    """Surface plus the fonts and primitives every view uses."""

    def __init__(self, surf):
        self.s = surf
        self.w, self.h = surf.get_size()
        u = max(12, int(self.h / 52))
        self.f_s = pygame.font.Font(None, u * 2)
        self.f_m = pygame.font.Font(None, int(u * 2.7))
        self.f_l = pygame.font.Font(None, int(u * 4.2))
        self.f_xl = pygame.font.Font(None, int(u * 6.0))

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
        """Progress arc from 12 o'clock, clockwise."""
        if frac <= 0.0:
            return
        rect = pygame.Rect(int(x - r), int(y - r), int(2 * r), int(2 * r))
        start = math.pi / 2
        pygame.draw.arc(self.s, colour, rect, start - 2 * math.pi * min(1.0, frac),
                        start, width)

    def bar(self, x, y, w, h, frac, colour):
        pygame.draw.rect(self.s, (40, 46, 54), (int(x), int(y), int(w), int(h)), 1)
        if frac > 0:
            pygame.draw.rect(self.s, colour,
                             (int(x) + 1, int(y) + 1,
                              int((w - 2) * min(1.0, frac)), int(h - 2)))


# ---------------------------------------------------------------------------
# input: one event stream from keyboard, mouse and any game controller
# ---------------------------------------------------------------------------
class Input:
    """Normalises keyboard, mouse and controller into up/down/select/back."""

    AX_DEAD = 0.55
    REPEAT_S = 0.22

    def __init__(self):
        pygame.joystick.init()
        self.pads = []
        self.rescan()
        self._ax_t = 0.0
        self._ax_last = 0

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
        """Returns a list of 'up' / 'down' / 'select' / 'back' / 'trigger'."""
        out = []
        for e in events:
            if e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_UP, pygame.K_LEFT):
                    out.append("up")
                elif e.key in (pygame.K_DOWN, pygame.K_RIGHT):
                    out.append("down")
                elif e.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_SPACE):
                    out.append("select")
                elif e.key == pygame.K_ESCAPE:
                    out.append("back")
                elif e.key == pygame.K_t:
                    out.append("trigger")
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                out.append("click")
            elif e.type == pygame.JOYBUTTONDOWN:
                # Any face button selects; the menu is the only place a wrong
                # guess costs anything, and it is always recoverable.
                out.append("back" if e.button in (1, 6) else "select")
            elif e.type == pygame.JOYHATMOTION:
                if e.value[1] > 0 or e.value[0] < 0:
                    out.append("up")
                elif e.value[1] < 0 or e.value[0] > 0:
                    out.append("down")
            elif e.type == pygame.JOYDEVICEADDED or e.type == pygame.JOYDEVICEREMOVED:
                self.rescan()
        # analogue sticks, rate limited so one push is one step
        v = 0
        for p in self.pads:
            for ax in (0, 1):
                try:
                    a = p.get_axis(ax)
                except Exception:
                    continue
                if a > self.AX_DEAD:
                    v = 1
                elif a < -self.AX_DEAD:
                    v = -1
        if v == 0:
            self._ax_last = 0
        elif v != self._ax_last or (now - self._ax_t) > self.REPEAT_S:
            self._ax_t = now
            self._ax_last = v
            out.append("down" if v > 0 else "up")
        return out


# ---------------------------------------------------------------------------
# the gun link
# ---------------------------------------------------------------------------
class Gun:
    """Owns the serial source and reports what has arrived on it."""

    def __init__(self):
        self.src = None
        self.port = None
        self.frames = 0
        self.trigs = 0
        self.gun_t = 0.0
        self.t_open = 0.0
        self.err = ""

    def connect(self, port=None):
        self.close()
        self.err = ""
        try:
            self.port = port or find_gun(BAUD)
        except Exception as e:
            self.port = None
            self.err = "serial scan failed: %s" % e
            return False
        if not self.port:
            self.err = "no gun answered ~ping on any serial port"
            return False
        try:
            self.src = SerialSource(self.port, BAUD)
            self.src.start()
        except Exception as e:
            self.src = None
            self.err = "could not open %s: %s" % (self.port, e)
            return False
        self.frames = self.trigs = 0
        self.t_open = time.time()
        return True

    def close(self):
        if self.src:
            try:
                self.src.close()
            except Exception:
                pass
        self.src = None

    def drain(self, session, on_trigger):
        """Feeds every queued line to the session. Returns lines consumed."""
        if not self.src:
            return 0
        n = 0
        # A backlog means the view fell behind; drop it rather than calibrate
        # from stale frames, except mid-capture where every frame counts.
        if session is None or session.state != session.S_CAPTURING:
            back = self.src.q.qsize()
            if back > 120:
                for _ in range(back - 20):
                    try:
                        self.src.q.get_nowait()
                    except queue.Empty:
                        break
        while n < 600:
            try:
                line = self.src.q.get_nowait()
            except queue.Empty:
                break
            n += 1
            if is_trigger(line):
                self.trigs += 1
                if session is not None:
                    session.note_trigger()
                    on_trigger()
                continue
            pq = parse_q(line)
            if pq is None:
                continue
            self.frames += 1
            q, gt = pq
            self.gun_t = gt
            if session is not None:
                session.feed(q, gt)
        return n

    def send(self, line):
        if not self.src:
            return
        try:
            self.src.ser.write(("\n%s\n" % line).encode())
        except Exception:
            pass


# ---------------------------------------------------------------------------
# views
# ---------------------------------------------------------------------------
class Menu:
    """Start screen: connection state and the run options."""

    def __init__(self, app):
        self.app = app
        self.sel = 0
        self.hot = []

    def items(self):
        g = self.app.gun
        conn = ("connected on %s" % g.port) if g.src else "NOT CONNECTED"
        return [("Calibrate  (%d distances)" % self.app.stances, "start"),
                ("Distances: %d" % self.app.stances, "stances"),
                ("Gun: %s" % conn, "reconnect"),
                ("Quit", "quit")]

    def act(self, what):
        if what == "start":
            if not self.app.gun.src:
                self.app.toast = "connect the gun first"
                return
            self.app.begin_calib()
        elif what == "stances":
            self.app.stances = 2 if self.app.stances >= 3 else 3
        elif what == "reconnect":
            self.app.connect()
        elif what == "quit":
            self.app.running = False

    def handle(self, actions, mouse):
        items = self.items()
        for a in actions:
            if a == "up":
                self.sel = (self.sel - 1) % len(items)
            elif a == "down":
                self.sel = (self.sel + 1) % len(items)
            elif a == "select":
                self.act(items[self.sel][1])
            elif a == "back":
                self.app.running = False
            elif a == "click":
                for i, r in enumerate(self.hot):
                    if r.collidepoint(mouse):
                        self.sel = i
                        self.act(items[i][1])
                        break

    def draw(self, sc):
        g = self.app.gun
        sc.text(sc.w / 2, sc.h * 0.13, "LIGHTGUN CALIBRATION", sc.f_xl, C_FG)
        sc.text(sc.w / 2, sc.h * 0.21,
                "aim with your IRON SIGHTS -- the cursor does not matter here",
                sc.f_s, C_DIM)
        self.hot = []
        y = sc.h * 0.36
        for i, (label, key) in enumerate(self.items()):
            on = (i == self.sel)
            col = C_FG if on else C_DIM
            if key == "reconnect":
                col = (C_OK if g.src else C_BAD) if not on else C_FG
            r = sc.text(sc.w / 2, y, ("> %s <" % label) if on else label,
                        sc.f_m, col)
            self.hot.append(r.inflate(sc.w * 0.5, sc.h * 0.02))
            y += sc.h * 0.085
        msgs = []
        if g.err:
            msgs.append(g.err)
        if g.src and g.frames == 0 and (time.time() - g.t_open) > NO_DATA_S:
            msgs.append("port open but no camera frames -- wrong port, or the")
            msgs.append("gun is not streaming (all four LEDs in view?)")
        elif g.src:
            msgs.append("%d camera frames, %d trigger pulls seen"
                        % (g.frames, g.trigs))
        sc.lines(sc.w / 2, sc.h * 0.76, msgs, sc.f_s,
                 C_BAD if g.err else C_DIM)
        sc.text(sc.w / 2, sc.h * 0.93,
                "controller, mouse or keyboard -- Enter selects, Esc quits",
                sc.f_s, C_DIM)
        if self.app.toast:
            sc.text(sc.w / 2, sc.h * 0.86, self.app.toast, sc.f_s, C_WARN)


class Calib:
    """The capture screen: targets, progress and live feedback."""

    def __init__(self, app, session):
        self.app = app
        self.session = session

    def handle(self, actions, mouse):
        for a in actions:
            if a == "back":
                self.app.to_menu()
            elif a in ("select", "click", "trigger"):
                # A controller button is a manual trigger, for a gun whose
                # trigger is not wired to the firmware yet.
                self.session.note_trigger()
                self.session.trigger(self.app.gun.gun_t or time.time())

    def draw(self, sc):
        s = self.session
        g = self.app.gun
        if g.frames == 0 and (time.time() - g.t_open) > NO_DATA_S:
            sc.text(sc.w / 2, sc.h * 0.30, "NO DATA FROM THE GUN", sc.f_l, C_BAD)
            sc.lines(sc.w / 2, sc.h * 0.44, [
                "Nothing has arrived on %s for %.0f seconds."
                % (g.port, time.time() - g.t_open),
                "",
                "Are all four LEDs in view? Frames with fewer are discarded.",
                "Is another program holding the port?",
                "",
                "Esc returns to the menu.",
            ], sc.f_s, C_DIM)
            return

        if s.state == s.S_STEPBACK:
            self.draw_stepback(sc)
            return

        tx, ty = s.target()
        cx, cy = tx * sc.w, ty * sc.h
        r = sc.h * 0.045
        cap = (s.state == s.S_CAPTURING)
        col = C_OK if cap else C_RING
        sc.ring(cx, cy, r, col, 3)
        pygame.draw.line(sc.s, col, (cx - r * 1.9, cy), (cx - r * 0.4, cy), 2)
        pygame.draw.line(sc.s, col, (cx + r * 0.4, cy), (cx + r * 1.9, cy), 2)
        pygame.draw.line(sc.s, col, (cx, cy - r * 1.9), (cx, cy - r * 0.4), 2)
        pygame.draw.line(sc.s, col, (cx, cy + r * 0.4), (cx, cy + r * 1.9), 2)
        pygame.draw.circle(sc.s, col, (int(cx), int(cy)), max(2, int(r * 0.12)))
        if cap:
            sc.arc(cx, cy, r * 1.5, s.progress(g.gun_t), C_OK, 5)
        elif s.dwell > 0:
            sc.arc(cx, cy, r * 1.5, s.dwell, C_WARN, 5)

        head = "DISTANCE %d of %d" % (s.stance + 1, s.stances)
        if s.stance_kind() == "roll":
            head = "TILTED STANCE %d of %d" % (s.stance + 1, s.stances)
        sc.text(sc.w / 2, sc.h * 0.06, head, sc.f_m, C_FG)
        sc.text(sc.w / 2, sc.h * 0.115,
                "dot %d of %d   --   pull %d of 4"
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

        self.draw_hud(sc)

    def draw_stepback(self, sc):
        s = self.session
        if s.stance_kind() == "roll":
            want = s.stance_roll()
            d = s.roll_check()
            sc.text(sc.w / 2, sc.h * 0.32,
                    "TILT THE GUN %s" % ("CLOCKWISE" if want > 0 else "ANTICLOCKWISE"),
                    sc.f_l, C_FG)
            now = "tilt %+.0f deg" % d if d is not None else "tilt --"
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
        self.draw_hud(sc)

    def draw_hud(self, sc):
        s = self.session
        g = self.app.gun
        parts = ["%d frames" % g.frames, "%.0f fps" % s.fps,
                 "%d shots" % len(s.shots)]
        if s.live_span:
            parts.append("span %.0f px" % s.live_span)
        close = s.too_close()
        if close is not None and close < 5.0:
            parts.append("LEDS AT THE FRAME EDGE -- step back")
        sc.text(sc.w * 0.02, sc.h * 0.975, "   ".join(parts), sc.f_s,
                C_BAD if (close is not None and close < 5.0) else C_DIM,
                centre=False)
        sc.text(sc.w * 0.98, sc.h * 0.975, "Esc to abandon", sc.f_s, C_DIM,
                centre=False)


class Result:
    """What the fit produced, and whether the gun took it."""

    def __init__(self, app, calib, why, install, saved):
        self.app = app
        self.c = calib
        self.why = why
        self.install = install
        self.saved = saved
        self.sel = 0
        self.hot = []

    def items(self):
        return [("Done", "menu"), ("Calibrate again", "again")]

    def act(self, what):
        if what == "again":
            self.app.begin_calib()
        else:
            self.app.to_menu()

    def handle(self, actions, mouse):
        items = self.items()
        for a in actions:
            if a == "up":
                self.sel = (self.sel - 1) % len(items)
            elif a == "down":
                self.sel = (self.sel + 1) % len(items)
            elif a == "select":
                self.act(items[self.sel][1])
            elif a == "back":
                self.app.to_menu()
            elif a == "click":
                for i, r in enumerate(self.hot):
                    if r.collidepoint(mouse):
                        self.act(items[i][1])
                        break

    def draw(self, sc):
        if self.c:
            px = self.c["fit_rms"] * ((1920.0 ** 2 + 1080.0 ** 2) ** 0.5) / (2 ** 0.5)
            good = px < 20.0
            sc.text(sc.w / 2, sc.h * 0.14, "CALIBRATED", sc.f_xl,
                    C_OK if good else C_WARN)
            sc.text(sc.w / 2, sc.h * 0.24,
                    "fit error %.1f screen px %s"
                    % (px, "" if good else "  (high -- consider redoing it)"),
                    sc.f_m, C_OK if good else C_WARN)
            sc.lines(sc.w / 2, sc.h * 0.34, [
                "LED rectangle %.3f x %.3f of the screen" % (self.c["w"], self.c["h"]),
                "sight offset %+.1f, %+.1f camera px" % (self.c["bx"], self.c["by"]),
                "shots used %d" % self.c.get("n_shots", 0),
            ], sc.f_s, C_DIM)
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
                "The most common cause is not moving far enough between",
                "distances -- the two stances must differ by 1.25x or more.",
            ], sc.f_s, C_DIM)
        if self.saved:
            sc.text(sc.w / 2, sc.h * 0.70, "log: %s" % os.path.basename(self.saved),
                    sc.f_s, C_DIM)
        self.hot = []
        y = sc.h * 0.80
        for i, (label, _k) in enumerate(self.items()):
            on = (i == self.sel)
            r = sc.text(sc.w / 2, y, ("> %s <" % label) if on else label,
                        sc.f_m, C_FG if on else C_DIM)
            self.hot.append(r.inflate(sc.w * 0.4, sc.h * 0.02))
            y += sc.h * 0.075


# ---------------------------------------------------------------------------
# application
# ---------------------------------------------------------------------------
class App:
    def __init__(self, surf, stances=3):
        self.sc = Screen(surf)
        self.gun = Gun()
        self.inp = Input()
        self.stances = stances
        self.session = None
        self.running = True
        self.toast = ""
        self.toast_t = 0.0
        self.view = Menu(self)

    def connect(self):
        self.toast = "looking for the gun..."
        self.draw()
        pygame.display.flip()
        if self.gun.connect():
            self.toast = ""
        else:
            self.toast = self.gun.err
        self.toast_t = time.time()

    def begin_calib(self):
        self.session = CaptureSession(plan=make_plan(self.stances, 0))
        # Fullscreen: a window fraction IS a screen fraction.
        self.session.to_screen = lambda fx, fy: (fx, fy)
        self.session.geom_note = "pical fullscreen %dx%d" % (self.sc.w, self.sc.h)
        self.view = Calib(self, self.session)

    def to_menu(self):
        self.session = None
        self.view = Menu(self)

    def finish(self):
        """Fit, save the session, install over serial."""
        s = self.session
        c, why = s.fit()
        saved = None
        try:
            paths = s.save(OUT_DIR)
            saved = paths[0]
        except Exception as e:
            self.toast = "could not write the log: %s" % e
        install = ""
        if c:
            cmd = aimcal_line(c)
            if hasattr(self.gun.src, "ser"):
                try:
                    install = install_over_serial(self.gun.src, cmd, c)
                except Exception as e:
                    install = "NOT SENT -- %s" % e
            else:
                install = "NOT SENT -- this run had no serial port"
            if saved:
                try:
                    with open(os.path.join(os.path.dirname(saved), "aimcal.txt"),
                              "w") as f:
                        f.write(cmd + "\n")
                except Exception:
                    pass
        self.view = Result(self, c, why, install, saved)
        self.session = None

    def on_trigger(self):
        if self.session and self.session.state != self.session.S_DONE:
            self.session.trigger(self.gun.gun_t or time.time())

    def draw(self):
        self.sc.s.fill(C_BG)
        self.view.draw(self.sc)

    def step(self, events, now):
        acts = self.inp.actions(events, now)
        mouse = pygame.mouse.get_pos()
        self.gun.drain(self.session, self.on_trigger)
        if self.session is not None and self.session.state == self.session.S_DONE:
            self.finish()
        self.view.handle(acts, mouse)
        if self.toast and (now - self.toast_t) > 4.0:
            self.toast = ""
        self.draw()


def run(stances=3, windowed=False, port=None):
    pygame.init()
    pygame.mouse.set_visible(windowed)
    flags = 0 if windowed else pygame.FULLSCREEN
    size = (1280, 720) if windowed else (0, 0)
    surf = pygame.display.set_mode(size, flags)
    pygame.display.set_caption("Lightgun calibration")
    app = App(surf, stances)
    if port is None:
        app.connect()
    else:
        app.gun.connect(port)
    clock = pygame.time.Clock()
    while app.running:
        events = pygame.event.get()
        for e in events:
            if e.type == pygame.QUIT:
                app.running = False
        app.step(events, time.time())
        pygame.display.flip()
        clock.tick(60)
    app.gun.close()
    pygame.quit()
    return 0


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--windowed", action="store_true",
                    help="run in a window instead of fullscreen")
    ap.add_argument("--stances", type=int, default=3, choices=(2, 3),
                    help="distance stances (default 3)")
    ap.add_argument("--port", help="serial port; probed when omitted")
    a = ap.parse_args()
    sys.exit(run(a.stances, a.windowed, a.port))


if __name__ == "__main__":
    main()
