#!/usr/bin/env python3
"""pical: every screen renders, and a whole calibration completes end to end.

Runs headless on SDL's dummy driver, so the pygame front end is covered by the
same suite as the desktop tools. Frames come from aim_calib's SimSource model,
fed on a synthetic gun clock so a full multi-stance run takes seconds.
"""
import os
import queue
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "pical"))

import numpy as np
import pygame

import pical
from aim_calib import SimSource

FAILS = []


def ck(ok, msg):
    print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        FAILS.append(msg)


class FakeSer:
    """Just enough serial for the calibration read-back and install paths.

    It also APPLIES ~cam= lines and answers ~camsave with what it is holding,
    so the save path is tested against a gun that can disagree rather than one
    that always says yes."""

    def __init__(self, replies):
        self.replies = replies
        self.written = []
        self.state = {"lead": 0, "smooth": 3, "beta": -1, "dead": 0, "lens": 0}
        self.save_fails = False

    def write(self, b):
        self.written.append(b)
        txt = b.decode("ascii", "replace")
        for line in txt.split("\n"):
            line = line.strip().lstrip("~")
            if not line.startswith("cam="):
                continue
            for tok in line[4:].split(","):
                k, sep, v = tok.partition(":")
                if sep and k in self.state:
                    try:
                        self.state[k] = int(v)
                    except ValueError:
                        pass
        if "aimcal?" in txt:
            self.replies.append(
                "AIM: cx=0.500000 cy=0.500000 w=0.350000 h=1.200000 "
                "bx=5.00 by=-3.00 lever=0.000000 rx=0.000000 ry=0.000000")
        if "camsave" in txt:
            self.replies.append(
                "%s thr=110 aec=300 agc=4 boost=0 lead=%dms smooth=%d dead=%d "
                "beta=%d lens=%d tmode=0 firk=7 firpct=100"
                % ("CAM: SAVE FAILED" if self.save_fails else "CAM: saved",
                   self.state["lead"], self.state["smooth"],
                   self.state["dead"], self.state["beta"], self.state["lens"]))
        return len(b)


class FakeSrc:
    """A SerialSource with no port: the queue is filled by the test."""

    def __init__(self):
        self.q = queue.Queue(maxsize=8000)
        self.replies = []
        self.ser = FakeSer(self.replies)

    def close(self):
        pass


def attach(app):
    app.link.src = FakeSrc()
    app.link.port = "SIM"
    app.t_open = 1e12                     # never trips the no-data screen


def key(k):
    return pygame.event.Event(pygame.KEYDOWN, key=k)


def main():
    pygame.init()
    surf = pygame.display.set_mode((1280, 720))

    app = pical.App(surf, stances=2)
    sc = pical.Screen(surf)
    ck(isinstance(app.view, pical.Menu), "opens on the menu")

    app.step([], 0.0)
    ck(True, "menu renders with no gun connected")
    app.step([key(pygame.K_DOWN)], 0.1)
    ck(app.view.sel == 1, "menu navigates on a key press")

    # every step refuses politely with no gun, rather than crashing
    for i, name in ((2, "calibrate"), (3, "fine tune"), (4, "verify")):
        app.view.sel = i
        app.toast = ""
        app.step([key(pygame.K_RETURN)], 0.2)
        ck(isinstance(app.view, pical.Menu) and app.toast,
           "%s without a gun refuses and says so" % name)

    attach(app)

    # ---- 2 camera tuning -------------------------------------------------
    cam = pical.Camera(app)
    app.open(cam)
    app.step([], 1.0)
    ck(True, "camera screen renders (ESP32 sliders)")
    n_rows = len(cam.rows)
    cam.sel = 0
    app.step([key(pygame.K_RIGHT)], 1.1)
    ck(any(b"cam=" in w for w in app.link.src.ser.written),
       "a slider nudge sends a ~cam= command")
    app.link.last["board"] = "rp2040-wiicam"
    cam2 = pical.Camera(app)
    ck(len(cam2.rows) < n_rows and cam2.wiicam(),
       "wiicam board shows the sensitivity control instead of the sliders")
    app.open(cam2)
    app.step([], 1.2)
    ck(True, "camera screen renders (wiicam)")
    cam2.tuning = True
    app.log_lines = ["sweeping..."]
    app.step([], 1.3)
    ck(True, "auto-tune progress screen renders")
    cam2.tuning = False
    app.link.last.pop("board", None)

    # ---- 3 lens ----------------------------------------------------------
    lens = pical.Lens(app)
    app.open(lens)
    app.step([], 2.0)
    ck(True, "lens screen renders")
    lens.fov = 60
    lens.preset()
    ck("pinhole" in app.toast, "a narrow FOV preset is refused with a reason")
    lens.fov = 160
    lens.preset()
    ck(any(b"lens:2" in w for w in app.link.src.ser.written),
       "a wide FOV preset sends a fisheye correction")
    lens.sweeping = True
    lens.t0 = 1e12                        # never completes during the test
    app.step([], 2.1)
    ck(True, "lens sweep screen renders")
    lens.sweeping = False

    # The gates the sweep screen draws live must be the gates the fitter
    # actually applies, or the coverage ring is a lie.
    csrc = open(os.path.join(ROOT, "tools", "calib_lens.py")).read()
    ck(("cov < %.2f" % pical.COV_GATE) in csrc
       and ("span < %.1f" % pical.SPAN_GATE) in csrc,
       "the drawn sweep gates match calib_lens")

    lens2 = pical.Lens(app)
    app.open(lens2)
    app.link.last["lens"] = 2
    ck(lens2.live_name() == "fisheye" and "fisheye" in lens2.save_hint(),
       "Save names the correction that is live, so it is clear what it writes")
    lens2.measure()
    ck(lens2.sweeping, "Measure starts a sweep")
    app.step([], 2.2)
    ck(True, "sweep screen renders with the live coverage map")
    n_before = len(app.link.src.ser.written)
    lens2.t0 = 0.0                        # the 20 s are up, with no frames
    lens2.handle([], (0, 0))              # collection runs off handle, not draw
    ck(not lens2.sweeping and lens2.fitting,
       "the fit starts without blocking the loop")
    lens2.draw(sc)
    ck(True, "the fitting screen renders while the worker runs")
    for _ in range(400):                  # the worker refuses this in ms
        if lens2.fit_result is not None:
            break
        time.sleep(0.01)
    lens2.handle([], (0, 0))
    ck(not lens2.fitting and lens2.report, "a starved sweep produces a report")
    ck(not lens2.report_ok and any("REFUSED" in l for l in lens2.report),
       "the report says it was refused, and why")
    ck(any(b"lens:2" in w for w in app.link.src.ser.written[n_before:]),
       "the previous correction is restored after a refusal")
    lens2.draw(sc)
    ck(True, "sweep report renders")
    lens2.handle(["select"], (0, 0))
    ck(not lens2.report, "any button dismisses the report")
    app.link.last.pop("lens", None)

    # ---- 4 aim calibration, end to end ----------------------------------
    app.to_menu()
    attach(app)
    app.begin_calib()
    ck(isinstance(app.view, pical.Calib), "calibration starts")
    sess = app.session
    sim = SimSource(sess)

    t = 1.0
    guard = 0
    while app.session is not None and guard < 40000:
        guard += 1
        # No trigger is ever sent: this exercises the hold-still fallback,
        # the path a user with an unwired trigger depends on.
        sess.auto = True
        d, rl = sim.stance_state()
        q = sim._quad(*sess.target(), d, rl, stance=sess.stance, dot=sess.idx)
        v = np.rint(np.asarray(q).reshape(-1) * 10).astype(int)
        app.link.src.q.put("Q,%d,4,%d,%d,%d,%d,%d,%d,%d,%d"
                           % (int(t * 1000), *v))
        t += 1.0 / 60.0
        app.step([], t)

    ck(app.session is None, "the run reached the end")
    ck(isinstance(app.view, pical.Result), "lands on the result view")
    r = app.view
    ck(r.c is not None, "the fit produced a calibration: %s" % (r.why or "ok"))
    if r.c:
        px = r.c["fit_rms"] * ((1920.0 ** 2 + 1080.0 ** 2) ** 0.5) / (2 ** 0.5)
        ck(px < 25.0, "fit error %.1f screen px (want < 25)" % px)
        ck(0.05 < r.c["w"] < 2.0 and 0.05 < r.c["h"] < 2.0,
           "LED rectangle is plausible (%.3f x %.3f)" % (r.c["w"], r.c["h"]))
    app.step([], t)
    ck(True, "result view renders")

    # ---- 5 fine tune -----------------------------------------------------
    app.to_menu()
    app.link.last["lead"] = 20
    app.link.last["smooth"] = 7
    app.begin_finetune()
    ck(isinstance(app.view, pical.FineTune), "fine tune reads the gun's calibration")
    ft = app.view
    ck(ft.t.lead == 20 and ft.t.smooth == 7,
       "it seeds lead and smoothing from the gun, not from defaults")
    app.step([], t + 1)
    ck(True, "fine-tune screen renders")
    ck(ft.controls == ["dx", "dy", "smooth", "beta", "lead"],
       "every adjustable thing has its own row")
    ft.sel = 0
    app.step([key(pygame.K_DOWN)], t + 1.5)
    ck(ft.sel == 1, "up/down moves between rows even on the sight-offset row")
    ft.sel = 4                             # lead
    app.step([key(pygame.K_RIGHT)], t + 2)
    ck(ft.t.lead == 20 + pical.LEAD_STEP, "lead adjusts from the seeded value")
    ft.sel = 2                             # rest smoothing
    app.step([key(pygame.K_RIGHT)], t + 3)
    ck(ft.t.smooth == 8 and "LEAD" in app.toast,
       "changing smoothing warns that lead must be re-checked")
    ft.sel = 0                             # sight, left/right
    before = ft.t.off[0].copy()
    app.step([key(pygame.K_LEFT)], t + 4)
    ck(ft.t.off[0][0] < before[0], "left/right nudges the sight across")
    ft.sel = 1                             # sight, up/down
    before = ft.t.off[0].copy()
    app.step([key(pygame.K_RIGHT)], t + 4.5)
    ck(ft.t.off[0][1] > before[1] and ft.t.off[0][0] == before[0],
       "the second row nudges the sight vertically, and only vertically")
    ft.sel = 0
    app.step([key(pygame.K_RETURN)], t + 4.7)
    ck(ft.sel == 1, "a pad button (select) also steps through the rows")

    # ---- beta: how fast the smoothing lets go once the gun moves ---------
    ck(ft.controls == ["dx", "dy", "smooth", "beta", "lead"],
       "fine tune exposes rest smoothing and speed sensitivity separately")
    ft.sel = 3                             # beta
    n0 = len(app.link.src.ser.written)
    lead_before, smooth_before = ft.t.lead, ft.t.smooth
    app.step([key(pygame.K_RIGHT)], t + 4.9)
    ck(app.link.last["beta"] == 18
       and any(b"beta:18" in w for w in app.link.src.ser.written[n0:]),
       "beta nudges up from the 15 default")
    ck(ft.t.lead == lead_before and ft.t.smooth == smooth_before,
       "and touches neither the lead nor the rest smoothing")
    app.step([], t + 5.0)
    ck(True, "fine-tune screen renders all five rows")
    ft.sel = 4                             # lead is now the fifth row
    app.step([key(pygame.K_RIGHT)], t + 5.1)
    ck(ft.t.lead == lead_before + pical.LEAD_STEP, "the lead row still adjusts lead")

    # ---- save: the feel settings must reach the gun on their own ---------
    # A session spent purely on lead, smoothing and beta used to end with
    # nothing written at all, because SAVE returned early when there was no
    # sight offset to solve.
    ser = app.link.src.ser
    ft.t.off[0][:] = 0.0
    ft.t.measured[0] = False
    # nothing left for the sight-offset solver: this is the state a session
    # spent purely on feel ends in, and it used to save nothing at all
    ft.t.lead0, ft.t.smooth0, ft.t.beta0 = ft.t.lead, ft.t.smooth, ft.t.beta
    ck(ft.t.solve_direct() is None, "there is no sight offset left to solve")
    n0 = len(ser.written)
    ft.save_now()
    ck(any(b"camsave" in w for w in ser.written[n0:]),
       "SAVE writes the feel settings even with no sight offset to solve")
    ck(ser.state["lead"] == ft.t.lead and ser.state["beta"] == ft.t.beta,
       "the gun ends up holding exactly what the screen shows")
    ck("SAVED" in app.toast, "and the toast is the gun's own words: %s" % app.toast)

    ser.save_fails = True
    ft.save_now()
    ck("REFUSED" in app.toast,
       "a refused write is reported as a failure, not as saved: %s" % app.toast)
    ser.save_fails = False

    # cancelling must put back the beta the gun had, not leave a silent edit
    app.to_menu()
    app.link.last["beta"] = 9
    app.begin_finetune()
    ft2 = app.view
    ft2.sel = 3
    app.step([key(pygame.K_RIGHT)], t + 5.4)
    ck(ser.state["beta"] == 12, "beta steps from the value the gun reported")
    ft2.handle(["back"], (0, 0))
    ck(ser.state["beta"] == 9, "cancelling puts the gun's own beta back")

    # and the two-thing save: calibration AND feel settings, each reported
    app.to_menu()
    app.begin_finetune()
    ft3 = app.view
    ft3.sel = 0
    app.step([key(pygame.K_LEFT)], t + 5.6)
    ft3.save_now()
    ck(isinstance(app.view, pical.Result),
       "saving a sight offset lands on the result view")
    ck("SAVED" in (app.view.note or ""),
       "and the result names the camera settings' own outcome: %s"
       % (app.view.note or "").replace("\n", " | "))

    # ---- 6 verify --------------------------------------------------------
    app.to_menu()
    app.begin_verify()
    ck(isinstance(app.view, pical.Verify), "verify reads the gun's calibration")
    vf = app.view
    app.step([], t + 5)
    ck(True, "verify screen renders")
    quad = sim._quad(0.5, 0.5, 1.7, 0.0, stance=0, dot=0)
    for i in range(len(pical.GRID_3x3)):
        vf.capturing = True
        vf.t0 = 0.0
        vf.buf = []
        vf.cur = []
        for k in range(20):
            vf.feed(quad, 0.05 * k)
    ck(vf.done, "nine shots complete the grid")
    ck(len(vf.results) == len(pical.GRID_3x3), "every shot produced a result")
    app.step([], t + 6)
    ck(True, "verify report renders")

    # ---- remaining states ------------------------------------------------
    pical.Result(app, None, "span spread too small", "", None).draw(sc)
    ck(True, "refused-fit view renders")

    app.to_menu()
    attach(app)
    app.begin_calib()
    s2 = app.session
    s2.state = s2.S_STEPBACK
    app.view.draw(sc)
    i = min(1, len(s2.plan) - 1)
    s2.plan[i]["kind"] = "roll"
    s2.plan[i]["roll"] = 1
    s2.stance = i
    app.view.draw(sc)
    ck(True, "step-back and tilt prompts render")

    app.t_open = 0.0
    app.link.frames = 0
    ck(app.no_data(), "a silent port is detected as no-data")
    app.view.draw(sc)
    ck(True, "no-data screen renders")

    # ---- the camera view and the drawn cursor ----------------------------
    def lit(s):
        a = pygame.surfarray.array3d(s)
        return int((a.sum(axis=2) > 60).sum())

    surf.fill(pical.C_BG)
    app.link.hist = []
    app.draw_preview(sc, 100, 100, 320, 235)
    ck(lit(surf) > 0, "the camera view says so when no four-LED frame arrives")

    surf.fill(pical.C_BG)
    app.link.hist = [(1.0, np.asarray(quad, float))]
    app.link.full_t = time.time()
    empty = lit(surf)
    cov = pygame.Surface((240, 176))
    cov.fill((0, 0, 0)); cov.set_colorkey((0, 0, 0))
    for i in range(200):
        cov.set_at((i, i % 176), (36, 84, 144))
    app.draw_preview(sc, 100, 100, 320, 235, rings=True, trail=cov)
    ck(lit(surf) > empty + 200, "the camera view draws the LEDs, quad and rings")

    real_pos = pygame.mouse.get_pos
    pygame.mouse.get_pos = lambda: (640, 360)
    app.view = pical.Menu(app)            # a screen that wants the cursor shown
    surf.fill(pical.C_BG); app._mseen = False; app.link.hid_on = True
    app.draw_cursor(sc); off = lit(surf)
    surf.fill(pical.C_BG); app._mseen = True
    app.draw_cursor(sc); on = lit(surf)
    surf.fill(pical.C_BG); app.link.hid_on = False
    app.draw_cursor(sc); frozen = lit(surf)
    pygame.mouse.get_pos = real_pos
    app.link.hid_on = True
    ck(off == 0, "no cursor is drawn until the pointer has actually moved")
    ck(on > 50, "the gun's cursor is drawn once it moves")
    ck(frozen == 0, "and not while the pointer is frozen")

    pygame.mouse.get_pos = lambda: (640, 360)
    keep = app.view
    app.view = pical.Lens(app)
    app.view.sweeping = True
    surf.fill(pical.C_BG); app.draw_cursor(sc)
    ck(lit(surf) == 0, "the cursor is hidden during a lens sweep")
    app.view.sweeping = False
    surf.fill(pical.C_BG); app.draw_cursor(sc)
    ck(lit(surf) > 50, "and comes back when the sweep ends")
    pygame.mouse.get_pos = real_pos
    app.view = keep

    # ---- DRM device choice -----------------------------------------------
    # A Pi has two DRM cards; one is the v3d render node with no screen on
    # it. Choosing that one draws nothing and reports no error at all.
    import builtins
    import glob as globmod
    import io
    fake = {"/dev/dri/card*": ["/dev/dri/card0", "/dev/dri/card1"],
            "/sys/class/drm/card0-*/status": [],
            "/sys/class/drm/card1-*/status": ["/sys/class/drm/card1-HDMI-A-1/status"]}
    real_glob, real_open = globmod.glob, builtins.open
    globmod.glob = lambda p: fake.get(p, [])
    builtins.open = lambda p, *a, **k: (io.StringIO("connected\n")
                                        if "card1-HDMI" in str(p)
                                        else real_open(p, *a, **k))
    os.environ.pop("SDL_KMSDRM_DEVICE_INDEX", None)
    picked = pical.pick_drm_device()
    idx = os.environ.pop("SDL_KMSDRM_DEVICE_INDEX", None)
    globmod.glob, builtins.open = real_glob, real_open
    ck(idx == "1", "picks the connected card, not the render node (got %s)" % idx)
    ck(picked and "card1" in picked, "and names the device it chose")

    pygame.quit()
    print("\npical: %s (%d failures)" % ("ALL PASS" if not FAILS else "FAILED",
                                         len(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
