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
# Somewhere disposable, set BEFORE pical is imported (it resolves OUT_DIR at
# import time). Without this every run of this test left a ~640 KB simulated
# session in pical/calib_out -- 72 of them had accumulated in the release tree,
# in the folder where a REAL capture from the stick is supposed to land.
import tempfile
os.environ.setdefault("PICAL_OUT", tempfile.mkdtemp(prefix="pical-test-"))
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
        self.state = {"lead": 0, "smooth": 3, "beta": -1, "dead": 0, "lens": 0,
                      # The report format and the shape ceiling ride in the
                      # camsave reply too, and the room sweep VERIFIES both --
                      # a ceiling that reached flash in a format where the
                      # gate cannot run is the no-op this whole feature spent
                      # a release being.
                      "fmt": 1, "bhmax": 0}
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
                "beta=%d lens=%d tmode=0 firk=7 firpct=100 fmt=%d bhmax=%d"
                % ("CAM: SAVE FAILED" if self.save_fails else "CAM: saved",
                   self.state["lead"], self.state["smooth"],
                   self.state["dead"], self.state["beta"], self.state["lens"],
                   self.state["fmt"], self.state["bhmax"]))
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
    # Checked by WHICH controls appear, not by how many: a count says nothing
    # about whether the right screen was built, and broke the moment the
    # ambient-light rows arrived.
    wii_labels = [r.label for r in cam2.rows]
    ck(cam2.wiicam() and "Sensitivity" in wii_labels
       and "THR" not in wii_labels and n_rows > 0,
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

    # ---- the system-cursor path -------------------------------------------
    # On a desktop the OS moves the pointer from the HID report, so blitting a
    # copy puts a second crosshair on screen one frame behind the real one.
    # There we hand our artwork to the system cursor and blit NOTHING; the same
    # show/hide rules still have to apply, through set_visible instead.
    shown = []
    real_vis = pygame.mouse.set_visible
    pygame.mouse.set_visible = lambda v: shown.append(bool(v))
    pygame.mouse.get_pos = lambda: (640, 360)
    app._sys_cursor, app._sys_shown = True, None
    app.view = pical.Menu(app)
    app._mseen, app.link.hid_on = True, True
    surf.fill(pical.C_BG); app.draw_cursor(sc)
    ck(lit(surf) == 0, "with a system cursor nothing is blitted")
    ck(shown[-1:] == [True], "and the system cursor is shown instead")
    n = len(shown)
    surf.fill(pical.C_BG); app.draw_cursor(sc)
    ck(len(shown) == n, "an unchanged state does not re-issue set_visible")
    app.link.hid_on = False
    app.draw_cursor(sc)
    ck(shown[-1:] == [False], "a frozen pointer hides the system cursor too")
    app.link.hid_on = True
    app.view = pical.Lens(app)
    app.view.sweeping = True
    app.draw_cursor(sc)
    ck(shown[-1:] == [False], "and so does a lens sweep")
    app.view.sweeping = False
    app.draw_cursor(sc)
    ck(shown[-1:] == [True], "it comes back when the sweep ends")
    pygame.mouse.set_visible = real_vis
    pygame.mouse.get_pos = real_pos
    app._sys_cursor, app._sys_shown = False, None
    app.view = keep
    # the headless driver must NOT have taken that path by itself, or the Pi
    # console would silently lose its only cursor
    ck(pical.App(surf, stances=2)._sys_cursor is False,
       "the headless driver keeps the blitted cursor")
    ck(pical.install_cursor(720) in (True, False),
       "install_cursor answers rather than raising")
    # The DRM cursor plane is commonly 64x64; one pixel over and the kmsdrm
    # backend refuses the cursor, silently killing the mode it exists for.
    for h in (480, 720, 1080, 2160):
        r = int(max(8, min(16, h * 0.022)))
        n = 2 * int(r * 1.9) + 3
        if n > 64:
            ck(False, "cursor surface is %d px at screen height %d "
                      "-- exceeds the 64 px hardware plane" % (n, h))
            break
    else:
        ck(True, "the cursor surface fits the 64 px hardware plane at "
                 "every screen height")

    # ---- the kmsdrm hardware-cursor opt-in --------------------------------
    # Default on the Pi console is the blit; PICAL_HWCURSOR=1 moves the
    # pointer to the DRM cursor plane and turns on the pump-paced wait.
    real_drv = pygame.display.get_driver
    pygame.display.get_driver = lambda: "kmsdrm"
    os.environ.pop("PICAL_HWCURSOR", None)
    a1 = pical.App(surf, stances=2)
    ck(a1._sys_cursor is False and a1._pump_wait is False,
       "kmsdrm defaults to the blitted cursor, no extra pumping")
    os.environ["PICAL_HWCURSOR"] = "1"
    a2 = pical.App(surf, stances=2)
    ck(a2._pump_wait == a2._sys_cursor,
       "with HWCURSOR, pump pacing follows the cursor install (%s)"
       % a2._sys_cursor)
    os.environ.pop("PICAL_HWCURSOR", None)
    pygame.display.get_driver = real_drv

    # ---- pump_wait --------------------------------------------------------
    class FakeClock:
        def __init__(self): self.ticks = []
        def tick(self, n): self.ticks.append(n)
    fc = FakeClock()
    pical.pump_wait(fc, False)
    ck(fc.ticks == [60], "without pumping it is exactly clock.tick(60)")
    pumps = []
    real_pump = pygame.event.pump
    pygame.event.pump = lambda: pumps.append(1)
    if hasattr(pical.pump_wait, "_next"):
        del pical.pump_wait._next
    t0 = time.time()
    # The deadline is carried across calls like clock.tick's: the first call
    # establishes the pace (no wait), the second holds it.
    pical.pump_wait(fc, True)
    first = len(pumps)
    pical.pump_wait(fc, True)
    dt = time.time() - t0
    pygame.event.pump = real_pump
    ck(fc.ticks == [60], "the pumped path never touches the clock")
    ck(first == 0, "the first call anchors the pace without waiting")
    # Loose bounds on purpose: a loaded CI runner can turn one 4 ms sleep
    # into most of the frame, so demand only that pumping happened and that
    # the pace is neither skipped nor grossly overslept.
    ck(len(pumps) >= 1,
       "the wait pumps events while holding the pace (%d)" % len(pumps))
    ck(0.008 < dt < 0.5,
       "and holds roughly a frame of pace across the pair (%.1f ms)" % (dt * 1e3))
    # a late frame must not accumulate debt: fake being far behind schedule
    pical.pump_wait._next = time.monotonic() - 1.0
    t0 = time.time()
    pical.pump_wait(fc, True)
    ck(time.time() - t0 < 0.05, "a late frame pays no extra wait")
    del pical.pump_wait._next

    # ---- the measured frame interval --------------------------------------
    ah = pical.App(surf, stances=2)
    attach(ah)
    for i in range(5):
        ah.step([], 100.0 + i * 0.016)
    ck(len(ah._frame_hist) >= 3, "the loop interval is being recorded")
    dts = [d for _, d in ah._frame_hist]
    ck(all(abs(d - 0.016) < 1e-9 for d in dts),
       "and records the real step-to-step gap")
    ah.step([], 102.0)
    ck(all(t > 101.0 for t, _ in ah._frame_hist),
       "old samples age out after a second")
    ah.draw_hud(pical.Screen(surf), "")
    ck(True, "the HUD renders with the app-interval figure")

    # ---- 7 recoil feel -----------------------------------------------------
    app.to_menu()
    app.view = pical.Recoil(app)
    ser = app.link.src.ser
    ck(any(b"fx?" in w for w in ser.written),
       "opening the recoil screen asks the gun for its state")
    app.step([], t + 8)
    ck(True, "recoil screen renders with no reply yet (all rows unknown)")
    rv = app.view
    rv.sel = 2                                 # hold
    n0 = len(ser.written)
    app.step([key(pygame.K_RIGHT)], t + 8.2)
    ck(any(b"fx=hold:" in w for w in ser.written[n0:]),
       "a knob row sends its ~fx= command")
    # the FX: echo is what the rows display; feed one through the pump
    app.link.last["fxhold"] = 90
    app.step([], t + 8.4)
    ck(True, "and renders the gun's echoed value")
    rv.sel = len(rv.rows) - 4                  # test fire
    n0 = len(ser.written)
    app.step([key(pygame.K_RETURN)], t + 8.6)
    ck(any(b"fx=test:1" in w for w in ser.written[n0:]),
       "the test-fire row fires over serial")

    # ---- quieting the gun for a measurement -------------------------------
    # A calibration measures where the gun was pointing when the trigger broke,
    # so the solenoid AND the rumble motor have to stop. The second half is the
    # one that was broken: setting the engine's rumble time to zero HANDS the
    # motor back to OpenFIRE, whose off-screen rumble then fired on every
    # calibration shot -- a "silence" that was louder than doing nothing.
    app.link.last.clear()
    app.link.last["board"] = "rp2040-wiicam"
    app._fx_saved = None
    app._fx_quiet_want = False

    app.link.last["fxquiet"] = 0            # firmware that HAS quiet mode
    app.link.last["fxon"] = 1
    n0 = len(ser.written)
    app.fx_quiet(True)
    sent = b"".join(ser.written[n0:])
    ck(b"fx=quiet:1" in sent, "new firmware is quieted with one switch")
    ck(app._fx_saved is None,
       "and nothing is saved and restored -- the state that used to get stuck")
    n0 = len(ser.written)
    app.fx_tick(time.time() + pical.QUIET_REARM_S + 1)
    ck(b"fx=quiet:1" in b"".join(ser.written[n0:]),
       "quiet is re-armed while it is wanted: it expires by itself, so a "
       "crashed app can never mute a gun for good")
    n0 = len(ser.written)
    app.fx_quiet(False)
    ck(b"fx=quiet:0" in b"".join(ser.written[n0:]), "and released on the way out")

    # Older firmware: the best imitation available, and it must never write
    # rumms:0 -- that is the exact value that gives the motor back.
    app.link.last.pop("fxquiet")
    app.link.last.update({"fxon": 0, "fxdrive": 45, "fxhold": 60,
                          "fxpulse": 1, "fxrumms": 0})
    n0 = len(ser.written)
    app.fx_quiet(True)
    sent = b"".join(ser.written[n0:])
    ck(b"rumms:1" in sent and b"rumms:0" not in sent,
       "old firmware keeps the motor with a 1 ms window, never a 0 that "
       "hands it back to OpenFIRE's own rumble")
    ck(app._fx_saved and app._fx_saved.get("drive") == 45,
       "the user's real settings are saved first")
    n0 = len(ser.written)
    app.fx_quiet(False)
    sent = b"".join(ser.written[n0:])
    ck(b"drive:45" in sent and b"on:0" in sent, "and put back afterwards")
    ck(b"fx?" in sent, "with a read-back asked for, so a restore that went "
                       "into a dead port is not mistaken for one that worked")
    app.toast = ""
    app.link.last["fxdrive"] = 5           # the gun did NOT take it
    app.fx_tick(time.time() + 2.0)
    ck("did NOT go back" in app.toast, "a restore that did not land is reported")

    # A gun already sitting on calibration-quiet values must not have them
    # saved as if the user had chosen them: that is how it became permanent.
    app._fx_saved = None
    app.link.last.update({"fxon": 1, "fxdrive": 5, "fxhold": 0, "fxpulse": 0,
                          "fxrumms": 0})
    ck(pical.quiet_plan(app.link.last) == "stuck",
       "values a calibration left behind are recognised for what they are")
    app.toast = ""
    app.fx_quiet(True)
    ck(app._fx_saved is None and "screen 7" in app.toast,
       "so they are never saved and restored, and the user is told what to fix")
    app.fx_quiet(False)

    # Arming twice without an intervening restore must not re-read the saved
    # values from a gun that is already holding the QUIET ones -- doing so
    # replaced the user's real settings with them, permanently. Reachable by
    # pressing "calibrate again" on the result screen.
    app._fx_saved = None
    app._fx_quiet_want = False
    app.link.last.update({"fxon": 0, "fxdrive": 45, "fxhold": 60,
                          "fxpulse": 1, "fxrumms": 0})
    app.link.last.pop("fxquiet", None)
    app.fx_quiet(True)
    app.link.last.update({"fxon": 1, "fxdrive": 5, "fxhold": 0,
                          "fxpulse": 0, "fxrumms": 1})   # the gun's own echo
    app.fx_quiet(True)                                   # ...and arm again
    ck(app._fx_saved and app._fx_saved.get("drive") == 45,
       "arming quiet twice keeps the user's real settings, not the quiet ones")
    app.fx_quiet(False)

    # A restore that could not be sent must KEEP what it was holding, so a
    # later attempt can still put it back.
    app._fx_saved = {"on": 1, "drive": 45, "hold": 60, "pulse": 1, "rumms": 0}
    saved_src, app.link.src = app.link.src, None
    ck(app.fx_restore() is False and app._fx_saved is not None,
       "a restore into a dead link reports failure and forgets nothing")
    app.link.src = saved_src
    ck(app.fx_restore() is True and app._fx_saved is None,
       "and the next attempt, with the gun back, completes it")

    # Leaving with the link down must not leave the app wanting silence: the
    # reconnect would then re-arm quiet on a gun nobody is calibrating.
    app._fx_quiet_want = True
    app._fx_plan = "quiet"
    saved_src, app.link.src = app.link.src, None
    app.fx_quiet(False)
    app.link.src = saved_src
    n0 = len(ser.written)
    app.fx_tick(time.time() + pical.QUIET_REARM_S + 1)
    ck(not app._fx_quiet_want
       and b"quiet:1" not in b"".join(ser.written[n0:]),
       "leaving while the gun is away still ends the wanting of silence")

    # ---- the link itself --------------------------------------------------
    # A reader thread that has died leaves every value frozen and every key
    # apparently working, because writes into a dead port fail silently. That
    # is indistinguishable from a screen whose controls have broken -- and it
    # is what "the arrows do not change anything" looks like from the couch.
    class DeadSrc(FakeSrc):
        def is_alive(self):
            return False
    app.link.src = DeadSrc()
    app.link.port = "SIM"
    app._link_t = 0.0
    app._link_retry = time.time()          # do not actually rescan ports here
    app.toast = ""
    app.link_tick(time.time())
    ck(app._link_bad, "a dead reader thread is NOTICED")
    ck("lost the gun" in app.toast, "and said out loud, not left to be guessed")
    app.step([], t + 9)
    ck(True, "the lost-link banner renders over whatever screen is up")
    ck(pical.LINK_API_OK,
       "pical and the tools/ beside it are the same generation")
    attach(app)
    app._link_bad = False
    ser = app.link.src.ser          # attach() built a NEW fake serial

    # ---- the two pages ----------------------------------------------------
    # Sixteen rows on one page was more than anyone could find anything in
    # while a test was running. What has to hold is not a row COUNT but which
    # page each control is on: the first page is what a test needs, the second
    # is what is set once and left alone, and a control drifting back onto the
    # first page puts the list straight back where it started.
    app.link.last["board"] = "rp2040-wiicam"
    cam3 = pical.Camera(app)
    app.open(cam3)

    def page(cam, advanced):
        """Build the page named and hand back its labels, in order."""
        if bool(cam.advanced) != advanced:
            cam.enter_advanced() if advanced else cam.leave_advanced()
        return [r.label for r in cam.rows]

    FRONT = ["Sensitivity", "Blob detail (sizes)", "Biggest blob (height)",
             "Learn LED shape", "Log blobs to the stick",
             "Write shape CSV to the stick", "Save to gun", "Advanced", "Back"]
    ADV = ["Smallest blob kept", "Largest blob kept",
           "Odd-one-out (size steps)", "Biggest blob (pixels)",
           "Roundness limit", "Sensor max size (0x06)",
           "Sensor min size (0x1B)", "Full-mode register (0x33)",
           "Sensor connection test", "Back"]
    labels = page(cam3, False)
    ck(labels == FRONT,
       "the camera page carries the testing controls, in order (%s)" % labels)
    adv_labels = page(cam3, True)
    ck(adv_labels == ADV,
       "and the second page carries the gates and the registers (%s)"
       % adv_labels)
    ck(not [l for l in adv_labels if l in FRONT and l != "Back"],
       "with nothing on both pages at once (%s)"
       % [l for l in adv_labels if l in FRONT and l != "Back"])
    # Every control the one long list used to carry is still reachable. A
    # split that quietly loses a row is worse than a list that is too long:
    # the setting is simply gone, with nothing on screen to say where.
    WAS = ["Sensitivity", "Sensor connection test", "Blob detail (sizes)",
           "Full-mode register (0x33)", "Smallest blob kept",
           "Largest blob kept", "Odd-one-out (size steps)",
           "Biggest blob (pixels)", "Roundness limit",
           "Sensor max size (0x06)", "Sensor min size (0x1B)",
           "Log blobs to the stick", "Learn LED shape",
           "Write shape CSV to the stick", "Save to gun"]
    lost = [l for l in WAS if l not in FRONT + ADV]
    ck(not lost,
       "and nothing that used to be on the screen was lost (%s)" % lost)

    # Getting there and back. Esc has to come back to the camera page rather
    # than out to the menu: the menu closes a blob log that is still
    # recording, so a glance at the size window would end a capture.
    labels = page(cam3, False)
    cam3.sel = labels.index("Advanced")
    app.step([key(pygame.K_RETURN)], t + 9.15)
    ck(cam3.advanced and isinstance(app.view, pical.Camera),
       "selecting Advanced opens the second page on the same screen")
    ck("ADVANCED" in cam3.title and cam3.title != pical.Camera.title,
       "which says so in its own title: %r" % cam3.title)
    app.step([key(pygame.K_ESCAPE)], t + 9.16)
    ck(not cam3.advanced and isinstance(app.view, pical.Camera),
       "and Esc comes back to the camera page, not out to the menu")
    ck(cam3.rows[cam3.sel].label == "Advanced",
       "landing back on the row that opened it (%s)"
       % cam3.rows[cam3.sel].label)
    # One screen throughout, so a running log survives the trip. Two Camera
    # objects would each have their own, and the one started on page one
    # would go on writing with nothing able to reach it to close it.
    cam3.log_toggle()
    trip = cam3._log
    page(cam3, True)
    page(cam3, False)
    ck(cam3._log is trip and trip is not None,
       "a blob log started on the camera page is still recording after a "
       "visit to the second one")
    cam3.log_toggle()

    labels = page(cam3, True)
    cam3.sel = labels.index("Largest blob kept")
    n0 = len(ser.written)
    app.step([key(pygame.K_LEFT)], t + 9.2)
    ck(any(b"cam=bmax:" in w for w in ser.written[n0:]),
       "and nudging a row on the second page sends the gate to the gun")
    labels = page(cam3, False)
    app.link.blobs = "CAM: blobs 30,40,3,1 200,41,4,1 31,140,3,1 210,150,14,0"
    app.link.last.update({"br4": 800, "br3": 150, "br2": 40, "br1": 10,
                          "br0": 0, "brej": 7})
    lines = "\n".join(cam3.blob_lines())
    ck("size 14 DROPPED" in lines,
       "the readout shows each blob's size and what the gate did with it")
    ck("saw all four LEDs" in lines,
       "and what share of frames is losing a corner -- the number that says "
       "how much the light is actually costing")
    # The gun's counters restart at zero when it reboots. A negative delta
    # never reaches the sample threshold, so the readout used to freeze on
    # stale numbers for the rest of the session.
    app.link.last.update({"br4": 5, "br3": 1, "br2": 0, "br1": 0, "br0": 0})
    cam3.blob_lines()                      # re-baselines on the negative delta
    app.link.last.update({"br4": 300, "br3": 20, "br2": 5, "br1": 0, "br0": 0})
    ck("saw all four LEDs" in "\n".join(cam3.blob_lines()),
       "and it recovers by itself after the gun reboots its counters")
    app.step([], t + 9.4)
    ck(True, "camera screen renders with the blob readout")

    # ---- full detail: seven fields per blob, and the worst-case readout ----
    # This is the state the readout is TALLEST in, and the one the slice at
    # the bottom of Camera.draw has to survive: box and pixel count on every
    # blob, three of them dropped, so the sentence wraps onto three rows and
    # not two.
    app.link.last.update({"fmt": 2, "fullreg": 85})
    app.link.blobs = ("CAM: blobs 118,140,3,0,11,9,214 200,141,4,0,12,10,203 "
                      "131,144,3,0,10,9,198 210,150,14,0,41,38,255")
    cam3.log_toggle()                      # the LOGGING line, drawn last
    # Two replies, because every counter on this screen is a DELTA: the first
    # takes the baseline, the second is the window the user actually reads.
    app.link.last.update({"bframes": 1900, "bms": 19000, "brej": 40,
                          "brrej": 18, "bvalve": 4, "bsrej": 6, "bnear": 0,
                          "br4": 1000, "br3": 60,
                          "br2": 20, "br1": 5, "br0": 0})
    cam3._rate.feed(1900, 19000)
    cam3.blob_lines()
    app.link.last.update({"bframes": 2000, "bms": 20000, "brej": 90,
                          "brrej": 39, "bsrej": 20, "br4": 1200, "br3": 90,
                          "br2": 30, "br1": 8, "br0": 1})
    cam3._rate.feed(2000, 20000)
    full = cam3.blob_lines()
    joined = "\n".join(full)
    ck("box 41x38 255px" in joined,
       "full detail shows each blob's box and pixel count")
    # The readout wraps to the width the SCREEN has, not to a flat 96 columns.
    # 96 filled 686 px of the Pi's 1024 and left 338 px of it empty, which
    # spent a whole wrapped row of a readout that has six -- and the row it
    # spent was the one at the bottom saying a capture was recording.
    pi_cols = pical.readout_cols(pical.Screen(pygame.Surface((1024, 768))))
    ck(pi_cols > 96,
       "1024x768 wraps the readout wider than the old flat 96 columns (%d)"
       % pi_cols)
    # Asserted as "never worse", not as a fixed row count. It used to demand
    # three rows at 96 and two at the measured width, which was true only for
    # the exact length of the label at the time -- relabelling one field from
    # "bright 255" to "255px" shortened every blob by five characters, both
    # widths landed on two rows, and a test that was really about the WIDTH
    # failed over wording. What has to hold is that measuring the screen can
    # never cost a row versus the flat 96, whatever the line happens to say.
    ck(len(pical.wrap(full[0], pi_cols)) <= len(pical.wrap(full[0], 96)),
       "wrapping to the measured width never costs a row versus a flat 96 "
       "(%d -> %d rows)"
       % (len(pical.wrap(full[0], 96)), len(pical.wrap(full[0], pi_cols))))
    wrapped = []
    for ln in full:
        wrapped.extend(pical.wrap(ln, pi_cols))
    ck(len(wrapped) >= 5,
       "the worst-case readout still fills the rows drawn for it, so the fit "
       "arithmetic is actually being tested (%d rows: %s)"
       % (len(wrapped), [r[:18] for r in wrapped]))
    ck("14 dropped by shape" in joined,
       "the shape gate's own drops are a DELTA beside the other two, so the "
       "user can see which gate is doing the work: %s"
       % [l for l in full if "dropped" in l])

    # The drop counts have to STAY on screen between gun replies. blob_lines()
    # runs from draw() at ~60 fps while link.last changes about once a second:
    # re-baselining on every call made "N dropped by size" true for the one
    # frame after each reply and blank for the other fifty-nine.
    ck("50 dropped by size" in joined and "21 dropped as odd-one-out" in joined,
       "the drop counts are the delta since the last reply: %s"
       % [l for l in full if "dropped" in l])
    ck(all("50 dropped by size" in "\n".join(cam3.blob_lines())
           for _ in range(5)),
       "and they survive the draws between one reply and the next, instead "
       "of showing for 16 ms in every second")

    # bvalve counts blobs the floor gave BACK, since boot. Read raw, one
    # give-back at any time in the power cycle pinned this warning on screen
    # for good -- permanently spending one of the six rows the readout has.
    ck("SIZE WINDOW TOO TIGHT" not in joined,
       "a since-boot bvalve is not a give-back happening now")
    app.link.last.update({"bframes": 2100, "bms": 21000, "bvalve": 61,
                          "br4": 1300, "br3": 120, "br2": 40, "br1": 10,
                          "br0": 2})
    ck("SIZE WINDOW TOO TIGHT" in "\n".join(cam3.blob_lines()),
       "a give-back happening now says the size window is too tight")
    app.link.last.update({"bframes": 2200, "bms": 22000,
                          "br4": 1400, "br3": 150, "br2": 50, "br1": 12,
                          "br0": 3})
    ck("SIZE WINDOW TOO TIGHT" not in "\n".join(cam3.blob_lines()),
       "and it clears once the give-backs stop, instead of latching for the "
       "rest of the power cycle")

    # ---- the false-negative meter -----------------------------------------
    # bnear counts blobs the SHAPE gate threw away that sat exactly where the
    # missing corner had to be. It is the only number on this screen that says
    # a gate is WRONG, and worded as one more drop count it would read as the
    # gate earning its keep -- the exact opposite.
    #
    # Measured over the same WINDOW as the heavy-gate line, and that is the
    # point rather than a detail. It was a one-reply delta, so it cleared
    # about a second after the last rejection while Studio's -- a rate over at
    # least thirty frames -- was still showing: two front ends on the same
    # gun, disagreeing at any given moment about whether the gate was taking
    # LEDs, and the one that said nothing was the one on the screen with no
    # console beside it. Driven on a simulated clock, because a window that
    # only expires with real seconds cannot be tested with real seconds.
    ck("GATE MAY BE TAKING REAL LEDs" not in "\n".join(cam3.blob_lines()),
       "a bnear that has not moved raises nothing")
    near_mono, nfake = time.monotonic, {"t": time.monotonic()}
    time.monotonic = lambda: nfake["t"]
    try:
        for i in range(4):
            app.link.last.update({"bframes": 2300 + 100 * i,
                                  "bms": 23000 + 1000 * i,
                                  "bnear": 3 if i else 0,
                                  "br4": 1500, "br3": 190, "br2": 60,
                                  "br1": 15, "br0": 4})
            cam3.blob_lines()
            nfake["t"] += 1.0
        near = "\n".join(cam3.blob_lines())
        ck("GATE MAY BE TAKING REAL LEDs" in near,
           "bnear moving says the gate may be taking real LEDs: %s"
           % [l for l in cam3.blob_lines() if "GATE" in l])
        ck("3 dropped by" not in near and "3 dropped as" not in near,
           "and it is NOT phrased as another drop count -- read as one it "
           "looks like the gate working, which is the opposite of what it "
           "means: %s" % [l for l in cam3.blob_lines() if "GATE" in l])
        ck("corner" in near,
           "it says WHY the blob was probably an LED -- it sat where a corner "
           "should have been")
        ck("frames" in near,
           "and it names the window it was measured over, which is what makes "
           "it the same measurement Studio reports: %s"
           % [l for l in cam3.blob_lines() if "GATE" in l])
        # The reply AFTER the last rejection. Under the old one-reply delta
        # this went silent here while Studio went on warning for its whole
        # window; now both hold.
        app.link.last.update({"bframes": 2700, "bms": 27000,
                              "br4": 1600, "br3": 220, "br2": 70, "br1": 18,
                              "br0": 5})
        nfake["t"] += 1.0
        ck("GATE MAY BE TAKING REAL LEDs" in "\n".join(cam3.blob_lines()),
           "one quiet reply does NOT clear it -- a one-reply delta is why the "
           "two front ends used to disagree about the same gun")
        # It still has to clear, or it is the latched since-boot warning this
        # project has shipped twice.
        nfake["t"] += 20.0
        ck("GATE MAY BE TAKING REAL LEDs" not in "\n".join(cam3.blob_lines()),
           "and it CLEARS once the window has moved past the last rejection, "
           "rather than latching for the rest of the power cycle")
        # ...and it will not be drawn off a handful of frames either, for the
        # same reason the heavy line will not: a rate over a tenth of a second
        # of camera swings wildly, and this readout has about seven rows.
        # Emptied again first: the assertion above ran blob_lines(), which
        # feeds the meter, so the window already holds a sample from before
        # this case and the span would be thousands of frames wide.
        nfake["t"] += 20.0
        app.link.last.update({"bframes": 5000, "bms": 50000, "bnear": 3})
        cam3.blob_lines()
        nfake["t"] += 1.0
        app.link.last.update({"bframes": 5010, "bms": 50100, "bnear": 9})
        cam3.blob_lines()
        ck(cam3._meter.span()[0] == 10,
           "the window really is ten frames wide (%s)"
           % (cam3._meter.span(),))
        ck("GATE MAY BE TAKING REAL LEDs" not in "\n".join(cam3.blob_lines()),
           "ten frames is not a window, however far bnear moved in them")
    finally:
        time.monotonic = near_mono
    # Left where the worst-case readout expects it: bnear climbing, over a
    # window wide enough to be drawn from.
    app.link.last.update({"bframes": 6000, "bms": 60000, "bnear": 0})
    cam3.blob_lines()
    app.link.last.update({"bframes": 6100, "bms": 61000, "bnear": 9})
    cam3.blob_lines()

    # The readout must not be drawn ON TOP of the rows, nor over the selected
    # row's own hint below them. Measured at 1024x768, which is the mode the
    # Pi actually runs in: at the 1280x720 the rest of this test renders at, a
    # seventh row still clears the hint by 2 px and the collision the slice
    # exists to avoid would go unnoticed.
    pi_sc = pical.Screen(pygame.Surface((1024, 768)))

    # ---- the rows themselves have to clear EACH OTHER ---------------------
    # draw_rows shrinks the pitch to fit the band it is given, and it will go
    # on shrinking it past the height of the face it draws in: at fourteen
    # rows the band left 24.8 px of pitch for a 24 px line box, which is
    # 0.8 px of daylight, and two more rows in the same band would have had
    # the labels overlapping with nothing to say so. Measured off the rects
    # draw_rows itself sets, at the resolution the Pi runs at.
    #
    # The band no longer stretches to fill itself either: nine rows spread
    # over the sixteen-row band would put 51 px between 24 px labels and push
    # the readout down for nothing. Both pages are measured, because the
    # longer of the two is the one that decides how much is left below.
    for advanced in (False, True):
        page(cam3, advanced)
        cam3.draw(pi_sc)
        ys = sorted(r.rect.centery for r in cam3.rows if r.rect)
        pitch = min(b - a for a, b in zip(ys, ys[1:]))
        what = "second page" if advanced else "camera page"
        ck(8 <= len(cam3.rows) <= 11,
           "the %s is a list somebody can read from a sofa, not sixteen "
           "rows (%d)" % (what, len(cam3.rows)))
        ck(pitch - pi_sc.f_m.get_height() >= 3.0,
           "and the rows on the %s clear each other by a real margin at "
           "1024x768: %d px of pitch for a %d px line box (%.1f px of "
           "daylight)" % (what, pitch, pi_sc.f_m.get_height(),
                          pitch - pi_sc.f_m.get_height()))
        # ...and clear the subtitle above them, which is the space the band
        # was grown into.
        sub_bottom = pi_sc.h * 0.145 + pi_sc.f_s.get_height() / 2.0
        ck(ys[0] - pi_sc.f_m.get_height() / 2.0 >= sub_bottom,
           "the first row on the %s starts below the subtitle (row top %d, "
           "subtitle ends %d)"
           % (what, ys[0] - pi_sc.f_m.get_height() / 2.0, sub_bottom))
    labels = page(cam3, False)

    def readout_rows():
        """Draw one camera frame and hand back the rect of EVERY readout row.

        Spied through lines() as well as text(), so a sentence that wraps is
        measured on all of its rows rather than only the one starting with
        the words being matched -- and so a check that finds nothing at all
        has something to fail on instead of quietly passing.
        """
        real_text, real_lines = pical.Screen.text, pical.Screen.lines
        got = {"inside": False, "rows": []}

        def spy_text(self, x, y, msg, font=None, colour=pical.C_FG,
                     centre=True):
            r = real_text(self, x, y, msg, font, colour, centre)
            if got["inside"]:
                got["rows"].append((r.top, r.bottom, str(msg)))
            return r

        def spy_lines(self, x, y, msgs, font=None, colour=pical.C_DIM,
                      step=1.5):
            got["inside"] = True
            try:
                return real_lines(self, x, y, msgs, font, colour, step)
            finally:
                got["inside"] = False

        pical.Screen.text, pical.Screen.lines = spy_text, spy_lines
        try:
            cam3.draw(pi_sc)
        finally:
            pical.Screen.text, pical.Screen.lines = real_text, real_lines
        return got["rows"]

    def check_clear(rows, what):
        ck(bool(rows), "there is a readout to measure at all, %s (drew %s)"
                       % (what, [m[:20] for _, _, m in rows]))
        if not rows:
            return
        row_bottom = max((r.rect.bottom for r in cam3.rows if r.rect),
                         default=0)
        top = min(r[0] for r in rows)
        bottom = max(r[1] for r in rows)
        # Where the selected row's own hint starts: draw_rows puts it at
        # TIP_Y, centred, in f_s. Running the readout into it is what a
        # wider slice buys.
        tip_top = pi_sc.h * pical.TIP_Y - pi_sc.f_s.get_height() / 2.0
        # The LAST ROW'S GLYPH, not its highlight rectangle. The rectangle is
        # 0.72 of the pitch and stops well short of the descender, so a
        # readout tucked just under it can still be drawn through the label's
        # bottom -- which is a collision the rect-only check cannot see.
        glyph_bottom = max((r.rect.centery + pi_sc.f_m.get_height() / 2.0
                            for r in cam3.rows if r.rect), default=0)
        ck(top >= row_bottom,
           "the blob readout sits below the last row, not over it, %s (rows "
           "end %d, readout starts %d)" % (what, row_bottom, top))
        # Real margin at both ends, not a pixel of clearance: this list grows,
        # and every previous time it grew the thing under it was found by
        # somebody at a TV rather than here.
        ck(top - glyph_bottom >= 6,
           "and clears the last LABEL by a real margin, %s (label ends %d, "
           "readout starts %d)" % (what, glyph_bottom, top))
        ck(bottom <= tip_top - 6,
           "and its last wrapped row clears the row hint by a real margin, "
           "%s (readout ends %d, hint starts %d)" % (what, bottom, tip_top))

    rows_a = readout_rows()
    ck(any("blobs now" in m for _, _, m in rows_a),
       "the readout was actually drawn, so there is something to measure "
       "(saw %s)" % [m[:20] for _, _, m in rows_a])
    # Read off the rows pical ITSELF chose to draw, not a slice this test
    # takes of its own: at five, a full-mode capture showed no frame rate and
    # no sign at all that it was recording to the stick.
    ck(any("LOGGING" in m for _, _, m in rows_a)
       and any("new frames/s" in m for _, _, m in rows_a),
       "the rows pical draws still reach the frame rate and the LOGGING "
       "indicator (drew %s)" % [m[:18] for _, _, m in rows_a])
    check_clear(rows_a, "with a full readout")

    # And the deepest the readout ever gets: both warnings take a row each on
    # top of the rows the blob line already wraps onto, and the fit
    # arithmetic is the only thing keeping the last of them out of the hint.
    # Above the frames the meter has already seen, because a frame count that
    # goes BACKWARDS is how a gun announces a reboot and the window starts
    # again from nothing -- which would leave this "worst case" a frame short
    # of being drawn at all.
    app.link.last.update({"bframes": 6200, "bms": 62000, "bvalve": 90,
                          "bnear": 12,
                          "br4": 1500, "br3": 190, "br2": 60, "br1": 15,
                          "br0": 4})
    rows_b = readout_rows()
    ck(any("SIZE WINDOW TOO TIGHT" in m for _, _, m in rows_b)
       and any("GATE MAY BE TAKING REAL LEDs" in m for _, _, m in rows_b),
       "the worst case really is the worst case -- both warnings up at once "
       "(drew %s)" % [m[:20] for _, _, m in rows_b])
    check_clear(rows_b, "with both warnings taking a row each")
    # More content than there are rows for is the NORMAL state of this
    # readout, and what it must never do is silently draw the overflow on top
    # of the hint. The count comes from the space that is actually there.
    ck(len(rows_b) >= 5,
       "and it is still drawing a useful number of rows (%d)" % len(rows_b))
    cam3.log_toggle()

    # ---- the sensor shape panel --------------------------------------------
    # A coordinate cannot say what SHAPE a blob was, and shape is the only
    # thing the gates on this screen judge. The panel draws each blob as the
    # box it actually filled, in the sensor's own 128x96 pixels, with the
    # position the gun reported crossed on top of it.
    #
    # That crosshair is a MEASUREMENT, not decoration. The box fields are only
    # BELIEVED to be in the 128x96 array; the position certainly is not (the
    # gun scales it into the pipeline's 240x176 first). If the two land on
    # each other the belief holds; if they pull apart, every shape gate is
    # judging a number in unknown units and the LED envelope behind them has
    # to be measured again. So the panel is tested on both outcomes.
    def texts_of(cam, sc=pi_sc):
        """Every string the screen draws, with its rect and whether it came
        through lines() -- so the panel's own numbers can be told apart from
        the readout underneath it."""
        real_text, real_lines = pical.Screen.text, pical.Screen.lines
        got = {"inside": False, "all": []}

        def spy_text(self, x, y, msg, font=None, colour=pical.C_FG,
                     centre=True):
            r = real_text(self, x, y, msg, font, colour, centre)
            got["all"].append((r, str(msg), got["inside"]))
            return r

        def spy_lines(self, x, y, msgs, font=None, colour=pical.C_DIM,
                      step=1.5):
            got["inside"] = True
            try:
                return real_lines(self, x, y, msgs, font, colour, step)
            finally:
                got["inside"] = False

        pical.Screen.text, pical.Screen.lines = spy_text, spy_lines
        try:
            sc.s.fill(pical.C_BG)
            cam.draw(sc)
        finally:
            pical.Screen.text, pical.Screen.lines = real_text, real_lines
        return got["all"]

    def paint(rect, sc=pi_sc):
        a = pygame.surfarray.array3d(sc.s)
        sub = a[rect.left:rect.right, rect.top:rect.bottom]
        return int((sub.sum(axis=2) > 60).sum())

    def reds(rect, sc=pi_sc):
        a = pygame.surfarray.array3d(sc.s).astype(int)
        sub = a[rect.left:rect.right, rect.top:rect.bottom]
        return int((abs(sub - np.asarray(pical.C_BAD)).sum(axis=2) <= 10).sum())

    def blob9(xp, yp, size, keep, w, h, px, dx=0, dy=0):
        """A nine-field blob whose box sits, by construction, exactly where
        the reported position says it should -- offset by dx,dy when the test
        wants the two to disagree."""
        xn = xp * pical.SENSOR_W / pical.FRAME_W
        yn = yp * pical.SENSOR_H / pical.FRAME_H
        return "%d,%d,%d,%d,%d,%d,%d,%d,%d" % (
            xp, yp, size, keep, w, h, px,
            int(round(xn - (w + 1) / 2.0)) + dx,
            int(round(yn - (h + 1) / 2.0)) + dy)

    # The conversion first, on its own, because everything drawn rests on it.
    x9, y9, box9, px9, dens9, placed9 = pical.blob_shape(
        (120, 88, 3, 1, 11, 9, 84, 58, 43))
    ck(abs(x9 - 64.0) < 0.01 and abs(y9 - 48.0) < 0.01,
       "the reported position converts out of the pipeline's 240x176 into "
       "the sensor's own 128x96 (%.2f, %.2f, want 64, 48)" % (x9, y9))
    ck(box9 == (58.0, 43.0, 12, 10) and placed9,
       "the box is placed by its own origin and sized w+1 by h+1 -- the gun "
       "sends xmx-xmn, so a one-pixel blob reports 0 and a box drawn 0 wide "
       "is an LED that vanishes: %s" % (box9,))
    ck(px9 == 84 and abs(dens9 - 84 / 120.0) < 1e-9,
       "and density is the pixel count over the box it fills (%.3f)" % dens9)
    x7, y7, box7, _p7, _d7, placed7 = pical.blob_shape((120, 88, 3, 1, 11, 9, 84))
    ck(not placed7 and box7 is not None
       and abs(box7[0] + box7[2] / 2.0 - x7) < 1e-9,
       "a seven-field blob has no origin, so its box is hung on the position "
       "and says so rather than being dropped: %s" % (box7,))
    ck(pical.blob_shape((120, 88, 3, 1))[2] is None,
       "and a four-field blob has no box at all, rather than a measured 0x0")
    ck(pical.box_position_gap([(120, 88, 3, 1, 11, 9, 84)]) is None,
       "with nothing to compare, the box/position check answers None rather "
       "than a made-up zero -- 'they agree' is a claim, not a default")

    # Nine fields, boxes where the positions say they should be. Measured
    # against the EMPTY panel rather than against zero: the panel's own
    # background and its 32-pixel grid are lit too, so a bare threshold would
    # pass on a panel that drew no blobs at all.
    app.link.last["fmt"] = 2
    app.link.blobs = "CAM: blobs"
    texts_of(cam3)
    bare = paint(cam3.shape_rect)
    good = " ".join(blob9(*b) for b in
                    ((50, 45, 3, 1, 11, 9, 84), (190, 48, 4, 1, 12, 10, 79),
                     (52, 150, 3, 1, 10, 9, 80), (195, 152, 14, 0, 41, 38, 255)))
    app.link.blobs = "CAM: blobs " + good
    drew = texts_of(cam3)
    pr = cam3.shape_rect
    ck(pr is not None and pr.width > 0 and pr.height > 0,
       "the shape panel is drawn and says what it covered: %s" % (pr,))
    ck(paint(pr) > bare + 1500,
       "and four blobs really put ink on it -- boxes, fills and crosshairs "
       "(%d lit pixels against %d for an empty frame)" % (paint(pr), bare))
    nums = [m for r, m, inside in drew if not inside and pr.colliderect(r)]
    ck(any("11x9" in m and "84px" in m and "%" in m for m in nums),
       "the numbers under it give each blob its w x h, its pixel count and "
       "how full the box is: %s" % nums)
    ck(any("41x38" in m for m in nums),
       "including the one that got dropped, which is the blob a gate is "
       "being tuned to reject: %s" % nums)
    ck("box centres land on the reported positions" in
       "\n".join(cam3.blob_lines()),
       "and the readout says the box really is in the sensor's own array: %s"
       % [l for l in cam3.blob_lines() if "box centres" in l])
    quiet_red = reds(pr)

    # Now break it: the same blobs with their boxes shifted a long way off.
    app.link.blobs = "CAM: blobs " + " ".join(
        blob9(*b, dx=dx, dy=dy) for b, dx, dy in
        (((50, 45, 3, 1, 11, 9, 84), 14, 7),
         ((190, 48, 4, 1, 12, 10, 79), -12, 6),
         ((52, 150, 3, 1, 10, 9, 80), 13, -9),
         ((195, 152, 14, 0, 41, 38, 255), -15, 8)))
    drew = texts_of(cam3)
    said = "\n".join(cam3.blob_lines())
    ck("BOX AND POSITION DISAGREE" in said,
       "a box that does not sit on its own position is called out in words, "
       "because it means the units are wrong: %s"
       % [l for l in cam3.blob_lines() if "DISAGREE" in l])
    ck("128" in said,
       "against the frame it is measured in, so 16 px means something: %r"
       % [l for l in cam3.blob_lines() if "DISAGREE" in l])
    nums = [m for r, m, inside in drew if not inside and pr.colliderect(r)]
    ck(sum(1 for m in nums if "off " in m) >= 3,
       "and every disagreeing blob carries its own gap: %s" % nums)
    ck(reds(cam3.shape_rect) > quiet_red + 60,
       "the gap is DRAWN as well -- a line from the box centre to the "
       "crosshair, so it cannot be missed from a sofa (%d red pixels against "
       "%d when they agree)" % (reds(cam3.shape_rect), quiet_red))

    # Kept and dropped have to be told apart, or a gate cannot be judged.
    app.link.blobs = "CAM: blobs " + " ".join(
        blob9(*b) for b in ((50, 45, 3, 1, 11, 9, 84),
                            (190, 48, 4, 1, 12, 10, 79)))
    texts_of(cam3)
    all_kept = reds(cam3.shape_rect)
    app.link.blobs = "CAM: blobs " + " ".join(
        blob9(*b) for b in ((50, 45, 3, 0, 11, 9, 84),
                            (190, 48, 4, 0, 12, 10, 79)))
    texts_of(cam3)
    ck(reds(cam3.shape_rect) > all_kept + 40,
       "a gate-dropped blob is drawn in a different colour from a kept one "
       "(%d red pixels against %d)" % (reds(cam3.shape_rect), all_kept))

    # The other three report formats, and a frame with nothing in it. None of
    # them may throw: pical is fullscreen on a Pi with no console, so one
    # exception inside draw is a black TV and no way to find out why.
    for what, line, want in (
            ("seven fields, the previous firmware",
             "CAM: blobs 50,45,3,1,11,9,84 190,48,4,1,12,10,79", "no origin"),
            ("four fields, no sizes at all",
             "CAM: blobs 50,45,3,1 190,48,4,1 52,150,3,1 195,152,14,0 "
             "(sizes need fmt:1)", "boxes need Blob detail 2"),
            ("a frame with no blobs in it", "CAM: blobs", "no blobs reported"),
            ("no answer from the gun yet", "", "no blobs reported"),
            ("a line the send buffer cut in half",
             "CAM: blobs 50,45,3,1,11,9,84,58,4 190,48,4,1,12", "11x9"),
            ("numbers the sensor cannot produce",
             "CAM: blobs 999,999,3,1,127,127,9999,127,127 "
             "-5,-9,3,0,-2,-3,-4,-6,-7", None)):
        app.link.blobs = line
        try:
            drew = texts_of(cam3)
            threw = None
        except Exception as e:                 # noqa: BLE001 - that IS the test
            drew, threw = [], e
        ck(threw is None, "the panel survives %s (%r)" % (what, threw))
        if threw is None and want:
            shown = [m for r, m, inside in drew
                     if not inside and cam3.shape_rect.colliderect(r)]
            ck(any(want in m for m in shown),
               "...and says '%s' rather than drawing a measured box it never "
               "got: %s" % (want, shown))
        if threw is None:
            ck(cam3.shape_rect.right <= pi_sc.w
               and cam3.shape_rect.bottom <= pi_sc.h
               and cam3.shape_rect.left >= 0 and cam3.shape_rect.top >= 0,
               "and stays on the screen with %s (%s)"
               % (what, cam3.shape_rect))

    # ---- and it must not be drawn over anything else ----------------------
    # The panel is a whole column tall and the readout under it is CENTRED and
    # runs nearly the full width, so the readout has to clear the panel as
    # well as the rows. Taking the rows alone drew the first line of the
    # readout straight through the panel's own numbers.
    app.link.blobs = "CAM: blobs " + good
    for advanced in (False, True):
        page(cam3, advanced)
        drew = texts_of(cam3)
        pr = cam3.shape_rect
        what = "second page" if advanced else "camera page"
        hit = [r for r in (row.rect for row in cam3.rows if row.rect)
               if pr.colliderect(r)]
        ck(not hit, "the panel clears every row on the %s (%s)" % (what, hit))
        under = [(r, m) for r, m, inside in drew if inside and pr.colliderect(r)]
        ck(not under,
           "and the readout starts below it rather than through it, %s (%s)"
           % (what, [m[:24] for _, m in under]))
        tip_top = pi_sc.h * pical.TIP_Y - pi_sc.f_s.get_height() / 2.0
        ck(pr.bottom <= tip_top - 6,
           "and the panel's own numbers clear the row hint, %s (panel ends "
           "%d, hint starts %d)" % (what, pr.bottom, tip_top))
        wide = [m for r, m, _i in drew if r.left < 0 or r.right > pi_sc.w]
        ck(not wide,
           "with nothing on the %s drawn off the side of a 1024 px screen "
           "(%s)" % (what, [m[:40] for m in wide]))
    labels = page(cam3, False)

    # The sensor's own thresholds, the odd-one-out gate, the shape gates, the
    # report format and the full-mode register all reach the gun -- from
    # whichever page they now live on. fmt: is sent ONLY for full mode: the
    # previous firmware has no such key and drops it in silence, so a gun on
    # it could never be moved off detail 0 at all.
    for label, wire, adv in (("Biggest blob (height)", b"cam=bhmax:", False),
                             ("Odd-one-out (size steps)", b"cam=rtol:", True),
                             ("Biggest blob (pixels)", b"cam=pxmax:", True),
                             ("Roundness limit", b"cam=armax:", True),
                             ("Sensor max size (0x06)", b"cam=hwmax:", True),
                             ("Sensor min size (0x1B)", b"cam=hwmin:", True),
                             ("Full-mode register (0x33)", b"cam=fullreg:",
                              True)):
        here = page(cam3, adv)
        ck(label in here, "the camera screen offers '%s'" % label)
        cam3.sel = here.index(label)
        n0 = len(ser.written)
        app.step([key(pygame.K_RIGHT)], t + 9.5)
        ck(any(wire in w for w in ser.written[n0:]),
           "and nudging it sends %s to the gun" % wire.decode())

    # ---- the shape gates, in the reader's units and inside the firmware's
    # ---- refusal ranges ----------------------------------------------------
    # The gun REFUSES a bhmax of 1..7, a pxmax of 1..11 and an armax of 1..15
    # outright -- it answers by name and leaves the old value alone. On a
    # screen with no console that is an arrow that visibly does nothing, so
    # the ladders are built so it cannot be reached: every rung, stepped from
    # every other rung, in both directions, has to be a value the gun takes.
    for label, floor, adv in (("Biggest blob (height)", 8, False),
                              ("Biggest blob (pixels)", 12, True),
                              ("Roundness limit", 16, True)):
        here = page(cam3, adv)
        row = cam3.rows[here.index(label)]
        sent = []
        real_send, app.link.send = app.link.send, sent.append
        try:
            for start in (None, 0) + tuple(row.vals) + (1, 5, 7, 63):
                held = {"v": start}
                real_get, row.get = row.get, (lambda h=held: h["v"])
                try:
                    for d in (-1, +1):
                        for _ in range(len(row.vals) + 3):
                            row.nudge(d)
                            if sent:
                                held["v"] = int(sent[-1].split(":")[1])
                finally:
                    row.get = real_get
        finally:
            app.link.send = real_send
        got = sorted(set(int(s.split(":")[1]) for s in sent))
        illegal = [v for v in got if v and v < floor]
        ck(got and not illegal,
           "'%s' can only ever emit a value the gun accepts -- 0 or >= %d "
           "(emitted %s, illegal %s)" % (label, floor, got, illegal))
        ck(0 in got, "...including 0, which is how the gate is turned off "
                     "again (%s emitted %s)" % (label, got))

    # And the value on screen is the one a person thinks in. armax travels as
    # EIGHTHS of a ratio: a row showing "20" beside a label saying roundness
    # is a setting nobody at a TV can check, whichever way they set it.
    adv_labels = page(cam3, True)
    ar = cam3.rows[adv_labels.index("Roundness limit")]
    ck(ar.show(20) == "2.5:1" and ar.show(16) == "2:1"
       and ar.show(24) == "3:1" and ar.show(0) == "off",
       "the roundness row shows a RATIO, not the wire's eighths (%s)"
       % [ar.show(v) for v in (0, 16, 20, 24, 32)])
    px = cam3.rows[adv_labels.index("Biggest blob (pixels)")]
    ck(px.show(14) == "14 px" and px.show(0) == "off" and px.show(None) == "--",
       "and the pixel row names its unit, with 'off' for the value that "
       "means off (%s)" % [px.show(v) for v in (None, 0, 14)])
    # armax is kept for guns already set up with it, and it is WRONG here: the
    # ratio needs a width, and at sensitivity 2 the sensor smears a blob
    # sideways, so the gate drops real LEDs. That reaches the user as a cursor
    # that sticks and never as the row they set weeks ago, so the row itself
    # has to say so -- a release note nobody at a TV can read is not a warning.
    ck("NOT recommended" in ar.tip(),
       "and the roundness row says outright that it is not recommended: %r"
       % ar.tip())
    ck("NOT recommended" not in px.tip()
       and "NOT recommended" not in cam3.rows[
           adv_labels.index("Smallest blob kept")].tip(),
       "while the rows that ARE still fit for use do not, so the warning "
       "keeps its meaning")
    # Whatever the gun says, including what it cannot mean. A firmware that
    # means something else by the key, or a value typed at a serial terminal,
    # must not take the screen down: pical is fullscreen with no console, so
    # an exception inside draw is a black TV.
    labels = page(cam3, False)
    bh = cam3.rows[labels.index("Biggest blob (height)")]
    ck(bh.show(10) == "10 rows" and bh.show(0) == "off"
       and bh.show(None) == "--",
       "the height row is in sensor ROWS, which is what keeps it apart from "
       "the pixel count -- both would otherwise read '12 px' (%s)"
       % [bh.show(v) for v in (None, 0, 10)])
    for v in (-1, 7, 255, 10 ** 9):
        for r_ in (ar, px, bh):
            ck(isinstance(r_.show(v), str),
               "'%s' survives a value the gun should never send (%r -> %r)"
               % (r_.label, v, r_.show(v)))
    for label, adv in (("Biggest blob (height)", False),
                       ("Biggest blob (pixels)", True),
                       ("Roundness limit", True)):
        here = page(cam3, adv)
        tip = cam3.rows[here.index(label)].tip()
        ck("Blob detail 2" in tip,
           "'%s' says it needs full detail -- in any other format the gate "
           "stands down and the row does nothing: %r" % (label, tip))
        ck("drops" in tip,
           "and says what it physically throws away, not what it sets: %r"
           % tip)
    labels = page(cam3, False)

    # No hint may be wider than the screen. They are drawn CENTRED and on one
    # line, with no wrapping anywhere -- a hint 1141 px long on the Pi's 1024
    # simply loses a word off each end, and the word this one lost was the
    # "NOT recommended" it was rewritten to carry.
    for advanced in (False, True):
        page(cam3, advanced)
        for r_ in cam3.rows:
            tip = r_.tip()
            if not tip:
                continue
            wide = pi_sc.f_s.size(tip)[0]
            ck(wide <= pi_sc.w - 20,
               "the '%s' hint fits the Pi's screen (%d px of %d): %r"
               % (r_.label, wide, pi_sc.w, tip))
    labels = page(cam3, False)

    # A REFUSAL has to reach the user. The gun answers by name and leaves the
    # old value in place, so the row goes on showing the truth -- which from
    # the sofa is indistinguishable from an arrow that does nothing. pical has
    # no visible log outside the auto-tune overlay, so a refusal that only
    # goes to the log is a refusal nobody will ever see.
    for reply, why in (
            ("CAM: pxmax below 12 would reject measured LEDs -- not set\n",
             "a pxmax under the measured LED envelope"),
            ("CAM: armax below 16 (2:1) would reject measured LEDs -- not "
             "set\n", "an armax under 2:1"),
            ("CAM: hwmax:0 refused -- it blinds the sensor\n",
             "the sensor ceiling that would blind the camera")):
        app.toast = ""
        app.link.src.q.put(reply)
        app.step([], t + 9.55)
        ck(reply.split("\n")[0][:40] in app.toast,
           "the gun refusing %s is said out loud, not left in a log nobody "
           "can see: %r" % (why, app.toast))

    cam3.sel = labels.index("Blob detail (sizes)")
    for k, want, gone in ((pygame.K_LEFT, b"cam=ext:1", b"cam=fmt:1"),
                          (pygame.K_RIGHT, b"cam=fmt:2", None)):
        app.link.last["fmt"] = 2 if k == pygame.K_LEFT else 1
        n0 = len(ser.written)
        app.step([key(k)], t + 9.6)
        sent = b" ".join(ser.written[n0:])
        ck(want in sent and (gone is None or gone not in sent),
           "blob detail sends %s, which BOTH firmware generations take (%s)"
           % (want.decode(), sent))

    # What ~camsave really writes. The gun keeps the format and the software
    # gates and nothing else, so a hint that says "these settings" flatly
    # sends people away believing an hwmax they spent an evening on will
    # still be there in the morning.
    save_hint = cam3.rows[labels.index("Save to gun")].tip()
    ck("full-mode register" in save_hint and "sensor thresholds" in save_hint,
       "the Save hint names what does NOT persist: %r" % save_hint)
    adv_labels = page(cam3, True)
    ck("NOT saved" in cam3.rows[
           adv_labels.index("Full-mode register (0x33)")].tip(),
       "and the register's own row says it too")
    # Nothing on the second page saves, and the row that leaves it is the last
    # thing a user touches after a minute spent on the size window.
    ck("Save" in cam3.rows[adv_labels.index("Back")].tip(),
       "the second page's Back row says where Save is: %r"
       % cam3.rows[adv_labels.index("Back")].tip())
    labels = page(cam3, False)

    # The camera's true frame rate, from the gun's own clock. Nobody has ever
    # measured it on this sensor, and it decides whether full mode is
    # affordable -- so it has to appear on the screen the user is looking at.
    app.link.last.update({"bframes": 1000, "bms": 10000})
    cam3._rate.feed(1000, 10000)
    app.link.last.update({"bframes": 1100, "bms": 11000})
    cam3._rate.feed(1100, 11000)
    ck("new frames/s" in "\n".join(cam3.blob_lines()),
       "the camera's measured frame rate is on screen")

    # Logging to the stick: the Pi has no console and the bad light is only at
    # the TV, so a session has to leave a file behind.
    cam3.log_toggle()
    ck(cam3._log is not None, "logging starts")
    app.link.last["bframes"] = 1101
    app.step([], t + 12.0)                # poll + sample happen in handle()
    logpath = cam3._log.path
    ck("LOGGING" in "\n".join(cam3.blob_lines()),
       "and says so on screen while it runs")
    cam3.log_toggle()
    ck(cam3._log is None and os.path.isfile(logpath),
       "stopping closes the file, and the file is really there")
    with open(logpath) as f:
        head = f.readline().strip()
    ck(head.startswith("wall,gun_ms,bframes,hz"),
       "with a header a PC can read months later")
    # And a screen that was logging must not keep the file open once it is gone.
    cam3.log_toggle()
    app.to_menu()
    ck(cam3._log is None, "leaving the screen closes the log")

    # ---- shape learning ---------------------------------------------------
    # What a confirmed LED actually looks like on this rig, measured by the
    # gun and carried off on the stick. Everything here is driven through the
    # real serial queue rather than by poking link.hists, because the whole
    # difficulty is that the answer arrives as thirteen separate lines and any
    # of them can go missing.
    HFEATS = ("sz", "bw", "bh", "aspect", "area", "irel")

    def learn_lines(on, frames, led, rej, feats):
        """The gun's '~camlearn?' answer. A feature nothing was fed into comes
        back as 32 zeros -- the firmware sends every line every time -- so
        `feats` only names the ones that have something in them."""
        out = ["CAM: learn on=%d frames=%d led=%d rej=%d bins=32\n"
               % (on, frames, led, rej)]
        for c in (0, 1):
            for f in HFEATS:
                b = feats.get((c, f), [0] * 32)
                out.append("CAM: hist c=%d f=%s %s\n"
                           % (c, f, " ".join(str(v) for v in b)))
        return out

    def feed(lines):
        for ln in lines:
            app.link.src.q.put(ln)
        app.link.pump()

    def csv_rows(path):
        with open(path) as fh:
            body = fh.read().strip().split("\n")
        return body[0], [r.split(",") for r in body[1:]]

    app.link.last["board"] = "rp2040-wiicam"
    cam4 = pical.Camera(app)
    app.open(cam4)
    labels = [r.label for r in cam4.rows]
    ck("Learn LED shape" in labels and "Write shape CSV to the stick" in labels,
       "the wiicam camera screen offers the shape capture and its CSV")
    csv_row = cam4.rows[labels.index("Write shape CSV to the stick")]
    ck("shape-NNN.csv" in csv_row.tip() and "0 LED" not in csv_row.tip(),
       "and before anything is captured its hint says what the file is, not "
       "that it holds nothing: %r" % csv_row.tip())

    # Starting it. The row follows the GUN, so nothing claims to be recording
    # until the gun has said it is.
    cam4.sel = labels.index("Learn LED shape")
    n0 = len(ser.written)
    app.step([key(pygame.K_RETURN)], t + 13.0)
    sent = b" ".join(ser.written[n0:])
    ck(b"camlearn=on:1" in sent and b"camlearn?" in sent,
       "selecting it starts the capture and asks straight back (%s)" % sent)
    ck(cam4.learn_on() is False,
       "and the row does not claim to be recording before the gun has "
       "answered -- a gun on older firmware never will")

    # A capture taken OUTSIDE full mode. Only the size histogram fills, and
    # size is the one feature already known not to separate a window from an
    # LED, so this has to be said out loud rather than discovered in the CSV.
    app.link.last["fmt"] = 1
    app.toast = ""
    cam4.learn_toggle()                        # stop
    cam4.learn_toggle()                        # ...and start again, in ext
    ck("SIZE" in app.toast and "Blob detail" in app.toast,
       "starting outside full mode warns that only size will fill: %r"
       % app.toast)

    sz0 = [0, 441, 58, 1] + [0] * 28
    sz1 = [0, 12, 90, 33] + [0] * 28
    feed(learn_lines(1, 1200, 4800, 90, {(0, "sz"): sz0, (1, "sz"): sz1}))
    ck(cam4.learn_on() is True,
       "once the gun answers, the row knows the capture is running")
    ck("1200 confirmed frames, 4800 LED blobs" in cam4.learn_hint(),
       "and the hint shows live progress, like the blob log's does: %r"
       % cam4.learn_hint())
    ck("SHAPE: 1200 frames 4800 LED blobs" in "\n".join(cam4.blob_lines()),
       "the readout says a capture is running and how far it has got")
    ck("4800 LED and 90 rejected" in csv_row.tip(),
       "and the CSV row now says what there is to write: %r" % csv_row.tip())

    # Both indicators on ONE row: six rows of readout are drawn, and a row
    # each would push whichever came last off the bottom -- a capture running
    # at the TV with nothing on screen to say so.
    cam4.log_toggle()
    rec = [l for l in cam4.blob_lines() if "LOGGING" in l]
    ck(len(rec) == 1 and "SHAPE:" in rec[0],
       "the shape capture and the blob log share one row rather than "
       "spending two of the six: %s" % rec)
    cam4.log_toggle()

    # The CSV. Written from a set the gun sent AFTER the asking, so the queue
    # is loaded first -- shape_save pumps the link itself.
    feed([])                                   # drain anything left over
    for ln in learn_lines(1, 1200, 4800, 90,
                          {(0, "sz"): sz0, (1, "sz"): sz1}):
        app.link.src.q.put(ln)
    app.toast = ""
    cam4.shape_save()
    shapes = sorted(f for f in os.listdir(pical.OUT_DIR)
                    if f.startswith("shape-"))
    ck(len(shapes) == 1, "the CSV lands on the stick beside the blob logs "
                         "(%s)" % shapes)
    ext_path = os.path.join(pical.OUT_DIR, shapes[0])
    hdr, rows = csv_rows(ext_path)
    ck(hdr == "class,feature,frames,led_blobs,rej_blobs,"
              + ",".join("b%d" % i for i in range(32)),
       "with the header a spreadsheet needs: %r" % hdr[:60])
    ck(len(rows) == 12 and all(len(r) == 37 for r in rows),
       "one row per class and feature, all the same width (%d rows, widths %s)"
       % (len(rows), sorted(set(len(r) for r in rows))))
    got = {(r[0], r[1]): r for r in rows}
    ck(sorted(got) == sorted((c, f) for c in ("led", "rej") for f in HFEATS),
       "named by class and feature, not by the wire's c=0 and c=1: %s"
       % sorted(k[0] for k in got)[:3])
    ck(all(r[2] == "1200" and r[3] == "4800" and r[4] == "90" for r in rows),
       "and the counts repeat on EVERY row, so a sorted sheet still says "
       "what it was measured from")
    ck(got[("led", "sz")][5:9] == ["0", "441", "58", "1"],
       "the size histogram is written bin for bin: %s"
       % got[("led", "sz")][5:10])
    ck(set(got[("led", "aspect")][5:]) == {"0"},
       "and the features full mode never fed are all-zero rows rather than "
       "missing ones -- an empty histogram is a fact worth having")
    ck("SIZE only" in app.toast,
       "the toast says the capture was size-only rather than letting it be "
       "found in the file: %r" % app.toast)

    # A FULL capture: every feature fed, both classes.
    full = {}
    for c in (0, 1):
        for i, f in enumerate(HFEATS):
            full[(c, f)] = [(i + 1) * (c + 1) + b for b in range(32)]
    for ln in learn_lines(0, 3400, 13120, 268, full):
        app.link.src.q.put(ln)
    app.toast = ""
    # No sleep. The name used to be built from the clock alone, so two
    # captures in the same second landed on the same file and this test had to
    # wait out a second to see two of them -- which is precisely the collision
    # a Pi with no RTC hits on every boot, at every second of the day.
    cam4.shape_save()
    shapes = sorted(f for f in os.listdir(pical.OUT_DIR)
                    if f.startswith("shape-"))
    ck(len(shapes) == 2, "a second capture written in the SAME second is a "
                         "second file, not an overwrite (%s)" % shapes)
    hdr, rows = csv_rows(os.path.join(pical.OUT_DIR, shapes[-1]))
    got = {(r[0], r[1]): r for r in rows}
    ck(len(rows) == 12 and got[("rej", "irel")][5] == "12",
       "a full capture writes all twelve, each with its own bins (%s)"
       % got[("rej", "irel")][5:9])
    ck(all(r[3] == "13120" and r[4] == "268" for r in rows),
       "with the LED and rejected counts on every row")
    ck("13120 LED" in app.toast,
       "and the toast reports what was captured: %r" % app.toast)

    # ---- never overwrite a recording ---------------------------------------
    # The Pi has NO real-time clock: every boot starts it at the same value,
    # so a capture taken today and one taken next week are stamped with the
    # same second. Under the old clock-only names the second one silently
    # replaced the first, and a capture taken at the TV is the only copy there
    # is. The number is scanned from the directory and only ever counts up.
    ck(all(f.split("-")[1].isdigit() and len(f.split("-")[1]) == 3
           for f in shapes),
       "every recording carries a zero-padded number (%s)" % shapes)
    nn = [int(f.split("-")[1]) for f in shapes]
    ck(nn == sorted(nn) and nn[1] > nn[0],
       "counting up, so a plain sort puts them in the order they were taken "
       "-- which is what the clock could not do (%s)" % shapes)
    ck(("recording %d" % nn[-1]) in app.toast,
       "and the toast says the number, which is what a user has to say to "
       "ask for the file: %r" % app.toast)

    seq_dir = tempfile.mkdtemp(prefix="pical-seq-")
    ck(pical.next_recording(seq_dir) == 1, "an empty stick starts at 1")
    for name in ("blobs-001-010101.csv", "shape-002-010101.csv",
                 "blobs-007-235959.csv"):
        open(os.path.join(seq_dir, name), "w").close()
    ck(pical.next_recording(seq_dir) == 8,
       "the next number is the highest ALREADY THERE plus one, whatever wrote "
       "it -- blobs and shape share one count so 'number 4' means one file "
       "(got %d)" % pical.next_recording(seq_dir))
    # A lens sweep is 'lenssweep-20260901-013038.log'. Read as a sequence
    # number that date would make the next recording number 20,260,902, and
    # every name after it unreadable.
    open(os.path.join(seq_dir, "lenssweep-20260901-013038.log"), "w").close()
    ck(pical.next_recording(seq_dir) == 8,
       "a datestamped name is not mistaken for a sequence number (got %d)"
       % pical.next_recording(seq_dir))
    ck(pical.next_recording(os.path.join(seq_dir, "nope")) == 1,
       "and a directory that cannot be listed still records, rather than "
       "refusing")

    # The real thing, hammered: a hundred names in the same second, none of
    # them a name that already exists.
    real_out, pical.OUT_DIR = pical.OUT_DIR, seq_dir
    try:
        seen = set(os.listdir(seq_dir))
        clashed, nums = [], []
        for _ in range(100):
            p, n = pical.recording_path("blobs")
            name = os.path.basename(p)
            if name in seen:
                clashed.append(name)
            seen.add(name)
            nums.append(n)
            open(p, "w").close()
        ck(not clashed,
           "a hundred recordings taken inside one second never reuse a name "
           "(%s)" % clashed[:3])
        ck(nums == sorted(nums) and len(set(nums)) == len(nums),
           "and their numbers go up one at a time, never repeating (%s..%s)"
           % (nums[:3], nums[-3:]))
        ck(nums[0] == 8, "carrying on from what was already there (%d)"
           % nums[0])
        # Belt and braces: a file already sitting on the name the count picks
        # is stepped over rather than written through. With no clock and a
        # directory that may have failed to list, "highest + 1" is a good
        # guess, not a proof -- and this is the one thing that must not be
        # guessed at, because the file it would destroy is the only copy.
        taken = os.path.join(seq_dir, "shape-%03d-%s.csv"
                             % (pical.next_recording(seq_dir),
                                time.strftime("%H%M%S")))
        with open(taken, "w") as fh:
            fh.write("the capture nobody wants to lose\n")
        got, _n = pical.recording_path("shape")
        ck(os.path.abspath(got) != os.path.abspath(taken)
           and not os.path.exists(got),
           "the name the count lands on is checked against the stick as "
           "well, so nothing is ever written through (%s, not %s)"
           % (os.path.basename(got), os.path.basename(taken)))
        ck(open(taken).read().startswith("the capture"),
           "and the file that was already there is still what it was")
    finally:
        pical.OUT_DIR = real_out

    # ---- the twelve lines, out of order and cut short ----------------------
    # They arrive interleaved with the frame stream and any of them can be
    # lost. What must never happen is a file half from this capture and half
    # from the last one: the numbers would all be plausible and nothing about
    # the file afterwards would look wrong.
    held = app.link.hists.counts()
    seq0 = app.link.hists.seq
    shuffled = learn_lines(1, 77, 88, 9, {(0, "sz"): sz0})
    feed([shuffled[0]] + shuffled[:0:-1])      # summary first, rows backwards
    ck(app.link.hists.counts() == (77, 88, 9)
       and app.link.hists.seq == seq0 + 1,
       "the twelve lines land whatever order they arrive in: %s"
       % (app.link.hists.counts(),))
    held = app.link.hists.counts()
    seq0 = app.link.hists.seq
    feed(learn_lines(1, 999, 999, 999, {})[:-4])   # four lines lost
    ck(app.link.hists.counts() == held and app.link.hists.seq == seq0,
       "a set cut short does not replace the one already held: %s"
       % (app.link.hists.counts(),))
    app.toast = ""
    cam4.shape_save()                          # nothing fresh in the queue
    ck("nothing written" in app.toast,
       "and the CSV refuses rather than writing the old capture under "
       "today's date: %r" % app.toast)
    ck(len([f for f in os.listdir(pical.OUT_DIR)
            if f.startswith("shape-")]) == 2,
       "so no third file appeared")
    # Merely LATE is not lost, though. pump() drains whatever has arrived and
    # returns, so the thirteen lines routinely come in over several calls: a
    # set that only counted the lines from one drain would never complete.
    feed(learn_lines(1, 999, 999, 999, {})[-4:])
    ck(app.link.hists.counts() == (999, 999, 999),
       "the last four completing it later is a whole reply, not a partial one")
    # What must be dropped is a tail with no summary in FRONT of it -- the
    # reply we joined half way through. Kept, its bins would be written out
    # under the previous capture's frame and blob counts.
    held = app.link.hists.counts()
    seq0 = app.link.hists.seq
    feed(learn_lines(1, 5, 5, 5, {(0, "sz"): sz1})[1:])
    ck(app.link.hists.counts() == held and app.link.hists.seq == seq0,
       "rows with no summary in front of them are dropped, not labelled with "
       "the last capture's counts: %s" % (app.link.hists.counts(),))

    # With no port open there is nothing that could answer, and this row
    # blocks the frame loop while it waits: two frozen seconds on a Pi with no
    # console is indistinguishable from a crash.
    src = app.link.src
    app.link.src = None
    app.toast = ""
    t_wait = time.monotonic()
    cam4.shape_save()
    app.link.src = src
    ck("connect the gun" in app.toast and time.monotonic() - t_wait < 0.5,
       "with no gun it says so at once rather than freezing the screen: %r"
       % app.toast)

    app.step([], t + 14.0)
    ck(True, "the camera screen renders with the shape rows on it")
    app.to_menu()

    # ---- the gun's own verdict on the shape gate ---------------------------
    # '~camfit' answers with a counter line and then exactly ONE outcome line,
    # and the two mean nothing apart: the counters say how far the measurement
    # has got, the outcome says what it came to. Every branch is exercised
    # here because the whole point of the command is that pical must never
    # invent a ceiling -- a gate borrowed from another rig blinds the gun and
    # its owner has no way to find out why -- so a parse that quietly produced
    # a number where the gun gave none would be the worst failure on the
    # screen, and the least visible.
    HDR = "CAM: fit ledn=%d ledmaxh=%d straym=%d strayminh=%d"
    f = pical.FitReport()
    ck(f.verdict is None and f.bhmax is None,
       "an empty fit report is 'we do not know', never a number")
    ck(not f.feed("CAM: blobs 1,2,3,4") and not f.feed("AIM: pong"),
       "and it only takes its own lines, so the shared reply stream can be "
       "fed to it wholesale")
    f.feed(HDR % (120, -1, 0, -1))
    f.feed("CAM: fit NEEDS MORE LED DATA -- run a calibration with the "
           "capture on; 120 blobs so far, 500 wanted")
    ck(f.verdict == "need_led" and f.ledn == 120 and f.led_want == 500,
       "NEEDS MORE LED DATA carries the count and the target off the gun's "
       "own sentence (%s, %d of %d)" % (f.verdict, f.ledn, f.led_want))
    ck(f.led_h is None and f.stray_h is None,
       "and the firmware's -1 'nothing measured' is None here, so no caller "
       "can print it as a height of minus one row (%s, %s)"
       % (f.led_h, f.stray_h))
    ck(abs(f.progress()[0] - 0.24) < 1e-6 and f.progress()[1] == 0.0,
       "the sweep's two bars read straight off it: %s" % (f.progress(),))
    ck(not f.enough(), "and it knows it has not reached a verdict yet")
    # The STORED line is the gun's record of its own previous capture, and it
    # carries ledmaxh= and strayminh= of its own. A blanket k=v sweep of every
    # 'CAM: fit' line would overwrite the LIVE measurement with those, which
    # is how a screen ends up reporting a stale ceiling as a fresh one.
    f.feed(HDR % (120, -1, 0, -1))
    f.feed("CAM: fit STORED ledmaxh=7 strayminh=12 -- from an earlier capture "
           "on this gun; recapture if the bar, the lens or the room has "
           "changed")
    f.feed("CAM: fit NEEDS MORE LED DATA -- run a calibration with the "
           "capture on; 120 blobs so far, 500 wanted")
    ck(f.stored == (7, 12) and f.led_h is None,
       "an earlier capture's pair is kept APART from the live measurement, "
       "which is still empty (stored %s, live %s)" % (f.stored, f.led_h))
    f.feed(HDR % (900, 7, 2, -1))
    f.feed("CAM: fit NO STRAY DATA -- sweep the room with the screen in view "
           "so a lamp or window enters frame; 2 seen, 20 wanted. Your LEDs "
           "measured 7 tall.")
    ck(f.verdict == "need_stray" and f.stray_n == 2 and f.stray_want == 20
       and f.led_h == 7,
       "NO STRAY DATA is the other half missing, and says so with both "
       "numbers (%s, %d of %d, LEDs %s)"
       % (f.verdict, f.stray_n, f.stray_want, f.led_h))
    f.feed(HDR % (900, 9, 40, 9))
    f.feed("CAM: fit NO SAFE GATE -- your LEDs reach 9 tall and the stray "
           "light starts at 9, so a size gate cannot tell them apart. Move "
           "the bar, block the light, or use brighter LEDs.")
    ck(f.verdict == "no_gate" and f.bhmax is None,
       "NO SAFE GATE offers NO ceiling at all -- there is no number that "
       "works on such a rig and half of one is worse than none (%s, %s)"
       % (f.verdict, f.bhmax))
    ck(f.led_h == 9 and f.stray_h == 9 and f.enough(),
       "but it does carry the two heights, which is what makes the refusal "
       "explainable (%s, %s)" % (f.led_h, f.stray_h))
    f.feed(HDR % (900, 7, 40, 13))
    f.feed("CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)")
    ck(f.verdict == "gate" and f.bhmax == 10 and f.led_h == 7
       and f.stray_h == 13 and not f.tight,
       "a verdict is the ceiling AND the two measurements behind it "
       "(bhmax %s, %s..%s, tight %s)"
       % (f.bhmax, f.led_h, f.stray_h, f.tight))
    f.feed(HDR % (900, 7, 40, 8))
    f.feed("CAM: fit bhmax=8 (LEDs reach 7, stray starts at 8 -- TIGHT, only "
           "one step between them)")
    ck(f.bhmax == 8 and f.tight,
       "and a one-step gap comes through as TIGHT rather than as an ordinary "
       "answer, because it is the one worth re-sweeping")
    # Setting the gate and getting it into flash fail SEPARATELY. A gate that
    # took but was not saved is gone on the next power cycle, and nothing else
    # anywhere would say so.
    f.feed("CAM: fit applied and saved")
    ck(f.applied == "saved", "'applied and saved' lands as a saved apply")
    f.feed(HDR % (900, 7, 40, 8))
    f.feed("CAM: fit bhmax=8 (LEDs reach 7, stray starts at 8)")
    f.feed("CAM: fit applied but SAVE FAILED -- it will be gone on the next "
           "power cycle")
    ck(f.applied == "unsaved",
       "and a failed save is told apart from a good one (%s)" % f.applied)
    # A power cycle empties the histograms, so the gun drops back to NEEDS
    # MORE LED DATA. The counter line has to take the old verdict with it: a
    # ceiling left standing from before the reboot is the one number on this
    # screen nobody may be shown by accident.
    f.feed(HDR % (10, -1, 0, -1))
    ck(f.verdict is None and f.bhmax is None and f.applied is None,
       "a fresh counter line clears the previous outcome instead of leaving "
       "an old ceiling on screen (%s, %s)" % (f.verdict, f.bhmax))
    f.feed(HDR % (900, 7, 40, 13))
    f.feed("CAM: fit SOMETHING A LATER FIRMWARE SAYS")
    ck(f.verdict is None,
       "an outcome this build has never heard of publishes NOTHING rather "
       "than a guess -- the counters stay, the verdict does not appear")
    # pical is fullscreen on a Pi with no console, so one exception anywhere
    # near a draw is a black TV. Every one of these is a line the gun's send
    # buffer could cut short, or a firmware that means something else by a key.
    for junk in ("CAM: fit", "CAM: fit ", "CAM: fit ledn=x ledmaxh= straym",
                 "CAM: fit bhmax=", "CAM: fit NO SAFE GATE --",
                 "CAM: fit STORED", "CAM: fit NEEDS MORE LED DATA",
                 "CAM: fit ledn=9999999999999 ledmaxh=-99 straym=0 strayminh=0",
                 "CAM: fit bhmax=10 (LEDs reach"):
        g = pical.FitReport()
        try:
            g.feed(HDR % (900, 7, 40, 13))
            g.feed(junk)
            threw = None
        except Exception as e:                 # noqa: BLE001 - that IS the test
            threw = e
        ck(threw is None,
           "a mangled fit line does not take the screen down (%r -> %r)"
           % (junk[:38], threw))
        ck(isinstance(g.progress()[0], float),
           "...and the bars still have a number to draw afterwards")
    # Firmware older than the fit answers every ask with a refusal. Left
    # unnoticed that is a thousand lines an hour into a 200-line log ring --
    # and the log is exactly where a user is sent to read the diag verdict.
    # The STORED line gained a third measurement -- the pixel envelope --
    # after this parser was written. Read by NAME for that reason: a
    # positional reader kept working by luck here and would have silently
    # mis-read whatever field was added next.
    f2 = pical.FitReport()
    f2.feed(HDR % (120, -1, 0, -1))
    f2.feed("CAM: fit STORED ledmaxh=7 strayminh=12 ledmaxpx=84 -- from an "
            "earlier capture on this gun; recapture if the bar, the lens or "
            "the room has changed")
    ck(f2.stored == (7, 12) and f2.stored_px == 84,
       "the stored provenance carries all three of the gun's own numbers "
       "(%s, px %s)" % (f2.stored, f2.stored_px))
    f2.feed(HDR % (120, -1, 0, -1))
    f2.feed("CAM: fit STORED strayminh=12 ledmaxpx=84 ledmaxh=7 -- reordered")
    ck(f2.stored == (7, 12),
       "in whatever order they arrive, because they are named on the wire "
       "(%s)" % (f2.stored,))
    f2.feed(HDR % (120, -1, 0, -1))
    f2.feed("CAM: fit STORED ledmaxh=7 -- an older firmware, half a pair")
    ck(f2.stored is None,
       "half a pair fills nothing, and the reading it belongs to shows no "
       "provenance rather than the PREVIOUS reading's (%s)" % (f2.stored,))
    # The gun taking a ceiling and saying it cannot act on it. NOT an outcome
    # -- it qualifies one, in the same answer -- so it must not blank the
    # verdict it arrived with.
    # A stray height of 0 means NO camfit has ever applied on this gun:
    # camsave records the LED edge on its own and leaves the stray side at
    # zero. Shown as "stray at 0" it reads as a measured room with no light in
    # it, which is the opposite of what it says -- and 0 is the one value that
    # cannot be a measured height, since the gate rejects only what is TALLER
    # than the ceiling.
    f2.feed(HDR % (120, -1, 0, -1))
    f2.feed("CAM: fit STORED ledmaxh=7 strayminh=0 ledmaxpx=84 -- from an "
            "earlier capture on this gun")
    ck(f2.stored == (7, 0), "a stray height of 0 still parses")
    ck("not recorded" in f2.stored_words() and "0 rows" not in f2.stored_words(),
       "but it reads as NOT RECORDED, never as a room whose light starts at "
       "zero: %r" % f2.stored_words())
    f2.feed(HDR % (120, -1, 0, -1))
    f2.feed("CAM: fit STORED ledmaxh=7 strayminh=12 ledmaxpx=84 -- earlier")
    ck("12 rows" in f2.stored_words(),
       "and a real stray height reads as itself: %r" % f2.stored_words())

    # ---- the contamination line ----------------------------------------
    # It starts with a DIGIT after 'CAM: fit ', so it matched no branch and
    # reached the log only -- and the log is not where anybody reads it. The
    # firmware says it out loud because a user who sees "32 ignored at 31
    # rows" understands their rig, where one who sees a clean 7 never learns
    # the sun spent the capture inside the LED class.
    f3 = pical.FitReport()
    ck(f3.ignored is None and f3.ignored_words() is None,
       "nothing is contaminated until the gun says so")
    f3.feed(HDR % (900, 7, 40, 13))
    f3.feed("CAM: fit 32 LED samples ignored -- they reach 31 tall, far above "
            "the 7 the rest stop at, and are almost certainly stray light "
            "learned while the resolver locked on it")
    ck(f3.ignored == (32, 31, 7),
       "the count, the height they reached and the height of the rest all "
       "land (%s)" % (f3.ignored,))
    said = f3.ignored_words()
    ck("32" in said and "31" in said and "7" in said,
       "and come back as words with all three numbers in them: %r" % said)
    f3.feed("CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)")
    ck(f3.ignored == (32, 31, 7) and f3.verdict == "gate",
       "it rides WITH the verdict rather than replacing it -- the ceiling is "
       "still the answer and this is why it is that number (%s, %s)"
       % (f3.ignored, f3.verdict))
    f3.feed(HDR % (900, 7, 40, 13))
    ck(f3.ignored is None,
       "and a fresh reading starts clean, so a capture that is no longer "
       "contaminated stops saying it is")
    for junk in ("CAM: fit 32 LED samples ignored",
                 "CAM: fit LED samples ignored -- they reach tall"):
        g3 = pical.FitReport()
        try:
            g3.feed(junk)
            threw = None
        except Exception as e:                 # noqa: BLE001 - that IS the test
            threw = e
        ck(threw is None and g3.ignored_words() is None,
           "a contamination line the buffer cut short says nothing rather "
           "than raising (%r -> %r)" % (junk[:34], threw))

    # ---- apply switches the format itself now --------------------------
    f2.feed(HDR % (900, 7, 40, 13))
    f2.feed("CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)")
    f2.feed("CAM: fit switched to fmt:2 -- the shape gate needs it and the "
            "ceiling was measured in it")
    f2.feed("CAM: fit applied and saved")
    ck(f2.switched and f2.verdict == "gate" and f2.bhmax == 10,
       "the gun switching itself into full mode rides with the verdict "
       "rather than replacing it (%s, %s)" % (f2.verdict, f2.switched))
    f2.feed(HDR % (900, 7, 40, 13))
    ck(not f2.switched, "and a fresh reading starts from not knowing again")

    # 'CAM: fit INERT RIGHT NOW ...' is GONE from current firmware: apply now
    # switches the gun into full mode itself, so the sequence that used to
    # produce it -- apply, then a reply saying the ceiling it just took
    # cannot act -- cannot happen from '=apply' on this build any more. Fed
    # here anyway because a gun that has not been reflashed can still send
    # it from a bare '~camfit?' (bhmax set by hand, outside full mode, on
    # older firmware), and a line this build no longer emits itself must
    # still not break the one that reads it.
    f2.feed(HDR % (900, 7, 40, 13))
    f2.feed("CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)")
    f2.feed("CAM: fit applied and saved")
    ck(not f2.inert, "nothing is inert until the gun says so")
    f2.feed("CAM: fit INERT RIGHT NOW -- the shape gate needs fmt:2 and this "
            "gun is in fmt:1; set fmt:2 for it to act")
    ck(f2.inert and f2.verdict == "gate" and f2.bhmax == 10,
       "an older gun's INERT line still qualifies the verdict rather than "
       "replacing it (%s, %s, inert %s)" % (f2.verdict, f2.bhmax, f2.inert))
    f2.feed(HDR % (900, 7, 40, 13))
    ck(not f2.inert, "and a fresh reading starts from not knowing again")

    g = pical.FitReport()
    ck(g.supported, "a gun that has not been asked yet is not a gun that "
                    "cannot answer")
    ck(g.feed('AIM: unknown command "camfit?"') and not g.supported,
       "the refusal is recognised, so the screens can stop asking")

    # ---- a counter read as a RATE, and a warning that CLEARS ---------------
    # Every counter on the '~camblob?' line counts from POWER-ON. This project
    # has shipped the two ways of getting that wrong twice: a bvalve warning
    # built on the raw total, which pinned itself on screen for the rest of
    # the power cycle after a single give-back, and a _delta() that advanced
    # its reference on every draw, so a count was visible for the one frame
    # after each reply and blank for the other fifty-nine -- 16 ms in every
    # second, which the eye reads as never. Both are tested here, and the
    # clear-down is driven by a SIMULATED clock rather than a sleep so the
    # suite does not pay eight seconds for it.
    real_mono, fake = time.monotonic, {"t": 1000.0}
    time.monotonic = lambda: fake["t"]
    try:
        m = pical.Meter(window_s=8.0)
        ck(m.per_frame("bsrej") == 0.0 and m.span() == (0, 0.0),
           "an empty window is 0, not a division by no frames at all")
        for i in range(6):
            m.feed(1000 + 100 * i, {"bsrej": 5000 + 300 * i})
            fake["t"] += 1.0
        ck(m.span()[0] == 500 and abs(m.per_frame("bsrej") - 3.0) < 1e-9,
           "a climb of 1500 over 500 frames reads as 3.0 a frame whatever the "
           "since-boot total behind it is (%s, %.2f)"
           % (m.span(), m.per_frame("bsrej")))
        held = len(m._s)
        for _ in range(60):
            m.feed(1500, {"bsrej": 6500})
        ck(len(m._s) == held,
           "sixty draws in one frame do not fill the window with copies of "
           "one reply -- samples are keyed on the gun's frame count, not on "
           "being called (%d samples)" % len(m._s))
        fake["t"] += 20.0
        ck(m.per_frame("bsrej") == 0.0 and m.climb("bsrej") == 0
           and m.span() == (0, 0.0),
           "and once the counter stops moving the window EXPIRES, so a "
           "warning built on it cannot latch for the rest of the power cycle "
           "(%.2f a frame)" % m.per_frame("bsrej"))
        m2 = pical.Meter()
        m2.feed(9000, {"bsrej": 9000})
        m2.feed(10, {"bsrej": 3})
        ck(m2.climb("bsrej") == 0,
           "a gun that rebooted restarts the window instead of reporting a "
           "negative rate that could never recover")
        m2.feed(None, {"bsrej": 4})
        m2.feed(20, {"bsrej": None})
        ck(isinstance(m2.per_frame("bsrej"), float),
           "and a counter this firmware does not send is 0, not a TypeError "
           "inside draw()")

        # ---- the gate warnings, on the screen -----------------------------
        app.link.last["board"] = "rp2040-wiicam"
        cam5 = pical.Camera(app)
        app.open(cam5)
        app.link.blobs = "CAM: blobs " + " ".join(
            blob9(*b) for b in ((50, 45, 3, 1, 11, 9, 84),
                                (190, 48, 4, 1, 12, 10, 79)))
        app.link.last.update({"fmt": 2, "bhmax": 10, "bframes": 1000,
                              "bms": 10000, "bsrej": 0, "bnear": 0,
                              "bvalve": 0, "brej": 0, "brrej": 0})
        cam5.blob_lines()
        ck(not [l for l in cam5.blob_lines() if "NO SAFE GATE" in l],
           "a gun that has not answered '~camfit' raises no verdict at all -- "
           "not knowing is not the same as 'no gate can work here'")
        for ln in (HDR % (900, 9, 40, 9),
                   "CAM: fit NO SAFE GATE -- your LEDs reach 9 tall and the "
                   "stray light starts at 9, so a size gate cannot tell them "
                   "apart. Move the bar, block the light, or use brighter "
                   "LEDs."):
            app.link.src.q.put(ln + "\n")
        app.step([], t + 15.0)
        ck(app.fit.verdict == "no_gate",
           "the app's own loop feeds the verdict, so every screen sees it "
           "without asking twice (%s)" % app.fit.verdict)
        gate = [l for l in cam5.blob_lines() if "NO SAFE GATE" in l]
        ck(gate, "NO SAFE GATE reaches the readout in words: %s" % gate)
        ck(gate and "bhmax:0" in gate[0],
           "...and says how to switch the gate off, which is the one action "
           "that always works and is not guessable from the row's label")
        ck(gate and ("Move the bar" in gate[0] or "brighter LEDs" in gate[0]),
           "...and what to do about the room instead of offering a number")
        tip = [r for r in cam5.rows
               if r.label == "Biggest blob (height)"][0].tip()
        ck("NO SAFE GATE" in tip and "rows" not in tip,
           "the gate's own row says it too, and offers no height at all -- "
           "there is no number that works on such a rig, and half of one is "
           "worse than none: %r" % tip)
        # bsrej is what the SHAPE gate threw away, since boot. Read raw it
        # only ever grows; what matters is whether the gate is refusing
        # heavily NOW, which is a share over seconds -- and, since a real
        # session settled the question, a share of the rejections the RESOLVER
        # could not account for.
        #
        # One helper for every case below, because all three counters have to
        # move together. bsrej alone is a state the gun cannot produce: a
        # rejected blob is either placed far from every corner (bfar), or
        # sitting where a missing LED should be (bnear), or neither. Driving
        # one of the three is how the first version of this threshold came to
        # be wrong.
        def window(base, frames, d_srej, d_far=0, d_near=0, steps=5):
            """Drive one measurement window; hand back the heavy-gate lines."""
            fake["t"] += 20.0                  # let the last window expire
            ck(cam5._meter.span() == (0, 0.0),
               "the rate window is empty before the next case (%s)"
               % (cam5._meter.span(),))
            for i in range(steps + 1):
                app.link.last.update({
                    "bframes": base + frames * i // steps,
                    "bsrej": 30000 + d_srej * i // steps,
                    "bfar": 40000 + d_far * i // steps,
                    "bnear": 50000 + d_near * i // steps})
                cam5.blob_lines()
                fake["t"] += 1.0
            # Every case below reads a SILENCE as meaning something, so the
            # window has to have really been driven: under thirty frames the
            # warning stands down whatever the rate is, and a case that never
            # reached that would pass for the wrong reason.
            ck(cam5._meter.span()[0] == frames,
               "the window drove the %d frames it meant to (%s)"
               % (frames, (cam5._meter.span(),)))
            return [l for l in cam5.blob_lines() if "REFUSING HEAVILY" in l]

        heavy = window(2000, 100, 250)
        ck(heavy, "a gate throwing away two and a half UNEXPLAINED blobs on "
                  "every frame is called out: %s" % heavy)
        ck(heavy and "bhmax:0" in heavy[0],
           "...and it too names the way out")
        ck(heavy and "could not account for" in heavy[0],
           "...and says what it counted, so the number can be argued with: "
           "%s" % heavy)

        # THE threshold, and it has to be ONE number across both front ends.
        # pical warned at 2.0 RAW rejections a frame while Studio warned at 25
        # per 100 frames -- an eight-fold disagreement about the same
        # measurement. What both were counting turned out to be the wrong
        # thing: see the daylight session below.
        ck(pical.GATE_HEAVY_PER_FRAME == 1.0,
           "the heavy-gate threshold is one UNEXPLAINED rejection per frame "
           "-- a whole slot of the four, every frame, that nobody can say was "
           "stray light (%s)" % pical.GATE_HEAVY_PER_FRAME)

        # ---- the session that set the number -------------------------------
        # 92 s of daylight with bhmax:8. From 01:32:37 to 01:32:41 the gate
        # rejected EXACTLY 1.00 blobs per frame for four straight seconds --
        # br4 zero, br3 every frame, bnear zero throughout. That is the sun
        # holding one of the four slots, the gate refusing it on every frame,
        # the gun running on three real corners and one reconstructed: the
        # gate doing precisely its job, in the very scene it exists for. Over
        # the whole session bsrej climbed 829 and bfar 730 -- 88% of the
        # refusals vouched for by the resolver, and the worst unexplained rate
        # was 0.06 a frame.
        #
        # A warning on RAW rejections at one per frame fires for the whole of
        # that stretch, on the one screen a user is sitting at while the gate
        # is working. This is the case that has to stay silent.
        ck(not window(12000, 50, 50, d_far=50),
           "the real daylight stretch -- 50 rejections over 50 frames, every "
           "one of them placed as stray by the resolver -- is SILENT: one a "
           "frame is the steady state of one persistent stray correctly "
           "refused, not a gate misbehaving")
        rate = (cam5._meter.climb("bsrej") - cam5._meter.climb("bfar")
                - cam5._meter.climb("bnear")) / float(cam5._meter.span()[0])
        ck(abs(rate) < 1e-9,
           "...because nothing about it is unexplained (%.2f a frame, against "
           "%.2f raw)" % (rate, cam5._meter.per_frame("bsrej")))

        # ---- and the case it has to keep catching --------------------------
        # A gate set for a different LED bar refuses three or four LEDs a
        # frame. The resolver then has too few points to lock, so NOTHING gets
        # vouched for: bfar and bnear stay flat while bsrej runs away. That is
        # the shape this warning exists for, and it is the opposite shape from
        # the daylight session even though both are "the gate rejecting a lot".
        eating = window(13000, 50, 150)
        ck(eating,
           "a gate refusing three blobs a frame with the resolver accounting "
           "for NONE of them warns -- that is a ceiling set for somebody "
           "else's bar: %s" % eating)
        ck(eating and "150 blobs" in eating[0],
           "and the count it names is the unexplained one (%s)" % eating)
        ck(eating and "different LED bar" in eating[0],
           "and it says what that usually means, which is the one thing the "
           "reader can act on: %s" % eating)
        # But not off a handful of frames. A rate measured over twenty frames
        # is a tenth of a second of camera and it swings wildly -- and this
        # readout has about seven rows, so a line that flickers in and out of
        # them costs a measurement somebody was reading.
        ck(not window(13500, 20, 100),
           "five unexplained rejections a frame over only twenty frames is "
           "NOT warned about -- the window has to have seen enough for the "
           "rate to mean anything")

        # ---- partly vouched for --------------------------------------------
        # The interesting middle. Raw rejections identical, and the verdict
        # flips on how much of it the resolver could place.
        ck(not window(14000, 50, 100, d_far=60),
           "100 rejections with 60 placed as stray is 0.8 a frame "
           "unexplained, and stays silent")
        part = window(15000, 50, 100, d_far=40)
        ck(part,
           "the same 100 rejections with only 40 placed is 1.2, and warns: "
           "%s" % part)
        ck(part and "60 blobs" in part[0] and "100 blobs" not in part[0],
           "and the message reports the 60 it could not explain, not the 100 "
           "the gate threw away -- the raw figure is the one that was wrong "
           "(%s)" % part)

        # ---- more vouched for than rejected --------------------------------
        # bfar can move without a rejection behind it: the resolver labels a
        # far blob it declines to associate whether or not a gate touched it.
        # Subtracted raw that is a NEGATIVE rate, which never reaches a
        # threshold again and would silently disable this warning for the rest
        # of the power cycle.
        ck(not window(16000, 50, 10, d_far=80),
           "bfar climbing faster than bsrej clamps to zero rather than going "
           "negative, and stays silent")
        ck(cam5._meter.climb("bfar") > cam5._meter.climb("bsrej"),
           "...with the window really holding the case it claims to (%d far, "
           "%d rejected)" % (cam5._meter.climb("bfar"),
                             cam5._meter.climb("bsrej")))
        after = window(17000, 50, 150)
        ck(after,
           "and the warning still works afterwards, so the clamp did not "
           "leave it stuck: %s" % after)

        # ---- the two warnings are independent ------------------------------
        # bnear is subtracted here, because a blob rejected where a corner
        # should be IS accounted for -- but it is accounted for as the gate
        # being WRONG, which is the other warning's whole subject. Netting it
        # out of one must never silence the other.
        both = window(18000, 50, 10, d_near=10)
        near_line = [l for l in cam5.blob_lines()
                     if "GATE MAY BE TAKING REAL LEDs" in l]
        ck(near_line,
           "bnear moving still raises the false-negative warning on its own "
           "delta: %s" % near_line)
        ck(not both,
           "while the heavy-gate warning stays silent, because those ten "
           "rejections were all accounted for: %s" % both)
        # And bnear really is netted out, not merely present in the sum. A
        # gate whose every mistake is the bnear kind -- refusing blobs exactly
        # where a corner should be -- is a real failure, but it is the OTHER
        # warning's failure; counted here as well it would be reported twice,
        # once as "taking real LEDs" and once as "set for a different bar",
        # and the second is advice that would not help.
        ck(not window(18500, 50, 100, d_near=60),
           "100 rejections with 60 of them landing where a corner should be "
           "is 0.8 a frame unexplained, and the heavy-gate warning stays "
           "silent -- that failure belongs to the false-negative meter")
        ck([l for l in cam5.blob_lines()
            if "GATE MAY BE TAKING REAL LEDs" in l],
           "...which is raising it, so the event is reported once and by the "
           "warning whose advice fits it")

        # ---- and it still clears --------------------------------------------
        # With all three counters in the window, which is the state that
        # actually occurs. A rate that cannot fall back to zero is a warning
        # pinned to the screen for the rest of the power cycle, and this
        # project has shipped that twice.
        ck(window(19000, 50, 150),
           "a heavy window warns...")
        fake["t"] += 20.0
        ck(not [l for l in cam5.blob_lines() if "REFUSING HEAVILY" in l],
           "...and CLEARS once the counters stop moving, with all three of "
           "them in the window")

        # ---- a gate that is set and cannot act -------------------------
        # The shape gate compares box HEIGHTS, and outside full report mode
        # the gun reports no box at all -- so the gate stands down and the
        # number in the row above judges nothing. This is the exact shape of
        # the bug that made the whole feature a no-op for a release: the saved
        # format was clamped below full mode, so every saved ceiling loaded
        # into a gun that could never execute it. Nothing else on this screen
        # shows it -- the row reads "10 rows", the gun agrees.
        app.link.last.update({"bframes": 22000, "bhmax": 10, "fmt": 2})
        cam5.blob_lines()
        ck(not [l for l in cam5.blob_lines() if "INERT" in l],
           "a gate in full mode is not called inert")
        app.link.last["fmt"] = 1
        inert = [l for l in cam5.blob_lines() if "INERT" in l]
        ck(inert, "a ceiling set outside full mode is called out as INERT: "
                  "%s" % inert)
        ck(inert and "Blob detail" in inert[0] and "bhmax:0" in inert[0],
           "...saying both ways out of it -- the format it needs, or off")
        ck(not [l for l in cam5.blob_lines() if "REFUSING HEAVILY" in l],
           "and a gate that cannot act is not also accused of refusing "
           "things, which would be two contradictory warnings at once")
        app.link.last["bhmax"] = 0
        ck(not [l for l in cam5.blob_lines() if "INERT" in l],
           "a gate that is off is not inert, it is off")

        # ---- naming the row that is ACTUALLY holding the gate up --------
        # Every case above happened to be tested with bhmax as the only one
        # of the three ever set. The shape gate is bhmax || pxmax || armax,
        # so a rig set up entirely from the second page -- pxmax or the
        # DEPRECATED armax, with Biggest blob (height) never touched -- runs
        # the gate with bhmax sitting at 0 throughout, and 'bhmax:0 turns it
        # off' on such a rig is a true sentence that changes nothing.
        app.link.last.update({"bhmax": 0, "pxmax": 0, "armax": 0})
        ck(cam5.gate_keys() == [],
           "nothing set on any of the three reads as the gate being off "
           "(%s)" % cam5.gate_keys())
        app.link.last["pxmax"] = 14
        ck(cam5.gate_keys() == ["pxmax"],
           "pxmax alone is read as the key actually holding the gate up, "
           "not assumed to be bhmax just because it is the first row on "
           "the page (%s)" % cam5.gate_keys())
        # fmt is still 1 from the case above, so this pxmax ceiling is
        # inert too -- the gate does not care which row set it -- and the
        # message has to say so by the RIGHT name.
        inert_px = [l for l in cam5.blob_lines() if "INERT" in l]
        ck(inert_px, "a pxmax ceiling outside full mode is inert as well")
        ck(inert_px and "pxmax:0" in inert_px[0]
           and "bhmax:0" not in inert_px[0],
           "...and names pxmax, the row that is actually live, never a "
           "bhmax that was left at 0 the whole time: %s" % inert_px)
        # The NO SAFE GATE verdict fed further up this block is still
        # standing -- nothing between there and here asked the gun again --
        # so the same rig is checked against its advice too.
        no_gate_px = [l for l in cam5.blob_lines() if "NO SAFE GATE" in l]
        ck(no_gate_px and "pxmax:0" in no_gate_px[0]
           and "bhmax:0" not in no_gate_px[0],
           "NO SAFE GATE's own advice names pxmax too, not bhmax: %s"
           % no_gate_px)
        # More than one of the three can be live at once -- they are
        # independent settings, not a radio button -- and naming only the
        # first would leave a user who cleared it still running the gate.
        app.link.last["bhmax"] = 10
        both = [l for l in cam5.blob_lines() if "INERT" in l]
        ck(both and "bhmax:0 and pxmax:0" in both[0],
           "both live settings are named together: %s" % both)
        # And with nothing live at all -- which is this gun's actual state
        # for the whole of the ladder tests further up this file, since
        # those never touch link.last for real -- the advice has to say
        # the gate is off rather than name a row that was never the one
        # holding it up.
        app.link.last.update({"bhmax": 0, "pxmax": 0, "armax": 0})
        no_gate_off = [l for l in cam5.blob_lines() if "NO SAFE GATE" in l]
        ck(no_gate_off and "off already" in no_gate_off[0]
           and "bhmax:0" not in no_gate_off[0],
           "nothing set reads as the gate being off already, not as advice "
           "to zero a row that was never on: %s" % no_gate_off)
        # The heavy-refusal warning names the live key too.
        app.link.last.update({"pxmax": 14, "fmt": 2})
        heavy_px = window(20500, 50, 150)
        ck(heavy_px and "pxmax:0" in heavy_px[0]
           and "bhmax:0" not in heavy_px[0],
           "the heavy-refusal warning names pxmax as well, when pxmax is "
           "what is actually live: %s" % heavy_px)
        # Left exactly as the case above left it, so nothing after this
        # depends on a row this case turned on.
        app.link.last.update({"bhmax": 0, "pxmax": 0, "armax": 0, "fmt": 1})
        ck(not [l for l in cam5.blob_lines() if "INERT" in l],
           "a gate that is off is not inert, it is off")

        # ---- the contamination note, on the readout --------------------
        # This is the one line that explains a ceiling: an envelope of 7 rows
        # with 32 samples set aside at 31 is a rig with the sun in the LED
        # class, and a user told that can go and fix it. It used to reach the
        # log only, which is nowhere.
        app.link.last["bhmax"] = 10
        ck(not [l for l in cam5.blob_lines() if "CONTAMINATED" in l],
           "a clean capture says nothing about contamination")
        for ln in (HDR % (900, 7, 40, 13),
                   "CAM: fit 32 LED samples ignored -- they reach 31 tall, "
                   "far above the 7 the rest stop at, and are almost "
                   "certainly stray light learned while the resolver locked "
                   "on it",
                   "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)"):
            app.link.src.q.put(ln + "\n")
        app.step([], t + 15.5)
        dirty = [l for l in cam5.blob_lines() if "CONTAMINATED" in l]
        ck(dirty, "a contaminated capture is said on the readout: %s" % dirty)
        ck(dirty and "32" in dirty[0] and "31" in dirty[0]
           and "7" in dirty[0],
           "with all three numbers, because '32 ignored at 31 against 7' is "
           "the whole of what makes it understandable: %s" % dirty)
        ck(dirty and "block that light" in dirty[0],
           "and what to do about it: %s" % dirty)
        ck(dirty and "left out" in dirty[0],
           "while saying the ceiling is measured from the rest, so it does "
           "not read as an error the user has to undo: %s" % dirty)
        app.link.last.update({"bhmax": 10, "fmt": 2})
        # A gate that is switched off cannot be refusing anything, however
        # much it threw away before the user turned it off.
        app.link.last["bhmax"] = 0
        ck(not window(23000, 50, 150),
           "a gate the user has already turned off is not accused of "
           "refusing anything")
        app.link.last["bhmax"] = 10
        ck(window(24000, 50, 150),
           "...and turning it back on brings the warning back, so the guard "
           "is on the gate's state and not a latch")
        app.link.last.update({"bframes": 25000, "bnear": 50100})
        near = [l for l in cam5.blob_lines() if "GATE MAY BE TAKING" in l]
        ck(near and "bhmax:0" in near[0],
           "the false-negative warning names the way out as well, so all "
           "three warnings answer the same question: %s" % near)
    finally:
        time.monotonic = real_mono
    app.to_menu()

    # ---- no recommended non-zero value, anywhere ---------------------------
    # Every figure these rows used to suggest was measured on ONE bar with two
    # LEDs per corner. A bar with five LEDs in each cluster makes a blob
    # several times bigger in every one of these units, so a borrowed ceiling
    # silently blinds that gun -- and what its owner sees is a cursor that
    # will not lock, with nothing on any screen connecting it to a row they
    # set weeks ago. The only defensible source for a non-zero value is
    # '~camfit' on the rig it is going to run on.
    app.link.last["board"] = "rp2040-wiicam"
    # From a gun that has said nothing yet, which is the state these hints
    # have to be right in: the height row deliberately CHANGES once the gun
    # has measured a ceiling, and the previous section left a verdict behind.
    app.fit.reset()
    cam6 = pical.Camera(app)
    app.open(cam6)
    GATES = ("Biggest blob (height)", "Biggest blob (pixels)",
             "Roundness limit", "Sensor max size (0x06)")
    suggested = []
    for advanced in (False, True):
        page(cam6, advanced)
        for r_ in cam6.rows:
            tip = r_.tip() or ""
            if r_.label not in GATES:
                continue
            for word in ("recommended", "a fair start", "Nintendo",
                         "is a good", "suggested"):
                if word in tip and "NOT recommended" not in tip:
                    suggested.append((r_.label, tip))
    ck(not suggested,
       "no gate row suggests a figure for itself on either page (%s)"
       % suggested)
    labels = page(cam6, False)
    bh = cam6.rows[labels.index("Biggest blob (height)")]
    ck("sweep" in bh.tip() and "recommended" not in bh.tip(),
       "the height gate points at the room sweep that can measure it, "
       "instead of at a number: %r" % bh.tip())
    ck(0 in bh.vals and bh.show(0) == "off",
       "and it still ships at 0 -- off -- reachable from its own row")
    adv_labels = page(cam6, True)
    ar = cam6.rows[adv_labels.index("Roundness limit")]
    ck("DEPRECATED" in ar.tip() and "NOT recommended" in ar.tip(),
       "roundness is marked DEPRECATED as well as not recommended -- the two "
       "are different things and the row has to carry both: %r" % ar.tip())
    ck(ar.show(20) == "2.5:1" and ar.show(0) == "off" and 0 in ar.vals,
       "and it still LOADS and displays, so a gun already set up with one "
       "keeps its meaning (%s)" % [ar.show(v) for v in (0, 16, 20)])
    hw = cam6.rows[adv_labels.index("Sensor max size (0x06)")]
    ck("Nintendo" not in hw.tip() and "100-200" not in hw.tip(),
       "the sensor ceiling quotes nobody else's hardware: %r" % hw.tip())
    ck(hw.vals[0] == -1 and "leaves it alone" in hw.tip(),
       "and names the value that leaves the sensor alone, which is how it "
       "ships (%s)" % (hw.vals[0],))
    px = cam6.rows[adv_labels.index("Biggest blob (pixels)")]
    ck(0 in px.vals and px.show(0) == "off",
       "every gate can still be turned off from its own row")
    page(cam6, False)
    app.to_menu()

    # ---- 4b: the room-light sweep ------------------------------------------
    # A step nobody has to do, offered after a calibration that is ALREADY
    # finished and saved. Everything here turns on one rule: it must never
    # cost the user something they did not ask for -- not a setting, not a
    # measurement, and not their way out of the screen.
    # ---- borrowing the gun's report format and capture ---------------------
    # The measurement needs full report mode and the learning capture, and
    # both are things a user may deliberately have had off. What has to be
    # right is the ORDER: the one fact that matters -- was the capture already
    # running? -- can only be learnt before arming, because after
    # '~camlearn=on:1' the gun answers on=1 whatever it was doing. So a borrow
    # asks first and arms when the answer lands, and it never blocks: that
    # answer is thirteen lines and up to 2.6 KB, and waiting for it inside a
    # key press freezes a screen that has no console beside it.
    def arm(on, led=0):
        """Let a pending borrow complete, with the gun saying what it held."""
        for ln in learn_lines(on, 40, led, 0, {}):
            app.link.src.q.put(ln)
        app.step([], time.time())

    app.link.last["board"] = "rp2040-wiicam"
    app.link.last["fmt"] = 1
    app.link.hists.reset()
    n0 = len(ser.written)
    app.begin_room_sweep()
    rs = app.view
    ck(isinstance(rs, pical.RoomSweep), "the menu row opens the sweep")
    sent = b" ".join(ser.written[n0:])
    ck(b"camlearn?" in sent and b"camlearn=on:1" not in sent,
       "it ASKS what the gun is holding before it arms anything -- after "
       "on:1 the gun answers on=1 whatever it was doing, and there is no way "
       "left to tell whose capture it is (%s)" % sent)
    n0 = len(ser.written)
    arm(0)                                     # the gun: capture was OFF
    sent = b" ".join(ser.written[n0:])
    ck(b"cam=fmt:2" in sent and b"camlearn=on:1" in sent,
       "and once the gun has answered it takes both -- full report mode and "
       "the capture (%s)" % sent)
    n0 = len(ser.written)
    app.step([key(pygame.K_ESCAPE)], t + 16.0)
    sent = b" ".join(ser.written[n0:])
    ck(b"cam=ext:1" in sent and b"camlearn=on:0" in sent,
       "Esc puts BOTH back exactly as they were -- left on they are a silent "
       "edit with nothing anywhere to say where it came from (%s)" % sent)
    ck(b"cam=fmt:1" not in sent,
       "and goes back as ext:, which both firmware generations take -- fmt: "
       "is dropped in silence by the older one")
    ck(isinstance(app.view, pical.Menu),
       "and lands on the menu, with the calibration untouched")
    # A capture that was ALREADY running is the user's. It is still armed --
    # on:1 at a running capture is a no-op, the firmware clears on the off->on
    # edge only -- but it must not be STOPPED on the way out.
    app.link.last["fmt"] = 2
    app.begin_room_sweep()
    arm(1, led=900)
    n0 = len(ser.written)
    app.to_menu()
    sent = b" ".join(ser.written[n0:])
    ck(b"camlearn=on:0" not in sent,
       "a capture the user already had running is not stopped on the way out "
       "-- pical only ever switches off one it can prove it started (%s)"
       % sent)
    ck(b"cam=ext:" not in sent,
       "and a gun already in full mode is not moved out of it either")
    # A gun that never answers is not waited on for ever, and its capture is
    # left running rather than stopped on a guess.
    app.link.last["fmt"] = 1
    app.link.hists.reset()
    app.begin_room_sweep()
    # Anchored to the real clock, not to a round number: monotonic is seconds
    # since boot, so an absolute value can land either side of the deadline
    # already recorded and the test would pass or fail on the machine's
    # uptime.
    real_mono, fake = time.monotonic, {"t": time.monotonic()}
    time.monotonic = lambda: fake["t"]
    try:
        app.step([], t + 16.2)
        ck(app._cam_borrow is None,
           "a borrow with no answer yet has taken nothing")
        fake["t"] += pical.CAM_ARM_S + 0.1
        n0 = len(ser.written)
        app.step([], t + 16.3)
        sent = b" ".join(ser.written[n0:])
        ck(b"camlearn=on:1" in sent,
           "but it does not wait for ever -- a gun with no capture at all "
           "never answers, and a step that collected nothing and said nothing "
           "about why is worse (%s)" % sent)
        n0 = len(ser.written)
        app.to_menu()
        ck(b"camlearn=on:0" not in b" ".join(ser.written[n0:]),
           "and with the answer unknown the capture is LEFT running: one "
           "nobody stopped shows on the camera screen, one stopped out from "
           "under somebody does not")
    finally:
        time.monotonic = real_mono
    # ...and one that has never said what report mode it holds is not written
    # a guess.
    app.link.last.pop("fmt", None)
    app.link.last.pop("ext", None)
    app.link.hists.reset()
    app.begin_room_sweep()
    arm(0)
    n0 = len(ser.written)
    app.to_menu()
    ck(b"cam=ext:" not in b" ".join(ser.written[n0:]),
       "and a gun that never said what report mode it was in is not written "
       "a guess on the way out")
    app.link.last["fmt"] = 2

    # ---- the calibration banks its own LED data ----------------------------
    # A calibration is minutes of frames in which the resolver has locked all
    # four corners, and every blob in such a frame is a confirmed LED -- which
    # is exactly what '~camfit' needs 500 of. This used to be discarded, and
    # both tools instead told the user to switch the capture on by hand on
    # another screen, so the LED side was only ever filled by somebody who
    # already knew the feature existed.
    app.link.last["fmt"] = 1
    app.link.hists.reset()
    n0 = len(ser.written)
    app.begin_calib()
    ck(isinstance(app.view, pical.Calib), "a calibration starts")
    arm(0)
    sent = b" ".join(ser.written[n0:])
    ck(b"camlearn=on:1" in sent and b"cam=fmt:2" in sent,
       "and it arms the shape capture and full mode, so its confirmed frames "
       "are measured instead of thrown away (%s)" % sent)
    # The borrow SPANS screens: the sweep is offered at the end of the
    # calibration, and taking it again would re-send on:1 -- an off->on edge
    # to a capture that is already running is fine, but re-REMEMBERING would
    # record the borrowed state as the user's own and never give the real one
    # back.
    held = app._cam_borrow
    app.finish_calib()
    ck(isinstance(app.view, pical.Result), "and finishes on the result screen")
    ck(app._cam_borrow == held,
       "which does NOT hand the borrow back -- the sweep offered from that "
       "screen needs the LED blobs the calibration just banked (%s vs %s)"
       % (app._cam_borrow, held))
    # A fitted calibration, so the result screen carries the row that offers
    # the sweep. finish_calib above had no shots to fit, and the sweep is
    # deliberately not offered after a refusal: there it would read as part of
    # the recovery rather than as an optional extra.
    app.view = pical.Result(app, dict(fit_rms=0.004, w=0.35, h=1.2, bx=5.0,
                                      by=-3.0), None, "INSTALLED", None)
    n0 = len(ser.written)
    app.view.sel = [l for l, _k in app.view.rows].index(
        "Learn the room light (optional)")
    app.step([key(pygame.K_RETURN)], t + 16.5)
    ck(isinstance(app.view, pical.RoomSweep),
       "the sweep opens from the result screen")
    ck(app._cam_borrow == held and app._cam_arm is None,
       "and INHERITS the calibration's borrow rather than taking it again -- "
       "a second ask would record fmt:2 as what the user had (%s vs %s)"
       % (app._cam_borrow, held))
    ck(b"camlearn?" not in b" ".join(ser.written[n0:]),
       "so it does not even ask, which is what keeps the calibration's own "
       "LED blobs out of reach of another off->on edge")
    n0 = len(ser.written)
    app.step([key(pygame.K_ESCAPE)], t + 16.6)
    sent = b" ".join(ser.written[n0:])
    ck(b"cam=ext:1" in sent and b"camlearn=on:0" in sent,
       "and ONE give-back at the end covers the whole trip (%s)" % sent)
    app.link.last["fmt"] = 2
    # Refusals, out loud. A row that silently does nothing is a row the user
    # decides is broken.
    app.link.last["board"] = "esp32-ov2640"
    app.toast = ""
    app.begin_room_sweep()
    ck(isinstance(app.view, pical.Menu) and app.toast,
       "an ESP32 gun is told there is nothing here to measure: %r" % app.toast)
    app.link.last["board"] = "rp2040-wiicam"
    src, app.link.src = app.link.src, None
    app.toast = ""
    app.begin_room_sweep()
    app.link.src = src
    ck(isinstance(app.view, pical.Menu) and "connect" in app.toast,
       "and with no gun it refuses politely rather than opening a screen "
       "nothing can answer: %r" % app.toast)

    # Nothing is applied without the user choosing it, and nothing at all is
    # applied when the gun has said no gate can work.
    # From fmt:1, and with the borrow actually COMPLETED, because that is the
    # only state the bug this guards against can be seen in: the sweep is
    # normally reached from a gun in extended mode, and it was handing that
    # mode back on the way out -- killing the ceiling the user had just
    # applied, under a screen that said it would hold through a power cycle.
    app.link.last["fmt"] = 1
    app.link.hists.reset()
    app.begin_room_sweep()
    arm(0)
    rs = app.view
    ck(app._cam_borrow == (1, False),
       "the sweep is entered from a gun in extended mode with the capture "
       "off, which is the normal way in (%s)" % (app._cam_borrow,))
    app.fit.reset()
    # From HERE, because ser.written carries the whole session and the ladder
    # tests further up nudged the gate rows on purpose.
    n_gate = len(ser.written)
    n0 = len(ser.written)
    app.toast = ""
    rs.apply()
    ck(b"camfit=apply" not in b" ".join(ser.written[n0:]) and app.toast,
       "nothing is applied before there is a verdict, and it says why: %r"
       % app.toast)
    for ln in (HDR % (900, 9, 40, 9),
               "CAM: fit NO SAFE GATE -- your LEDs reach 9 tall and the stray "
               "light starts at 9, so a size gate cannot tell them apart."):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 17.0)
    n0 = len(ser.written)
    app.toast = ""
    rs.apply()
    ck(b"camfit=apply" not in b" ".join(ser.written[n0:]),
       "nor when the gun says no size gate can work here: %r" % app.toast)
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 17.1)
    ck(b"cam=bhmax:" not in b" ".join(ser.written[n_gate:]),
       "a verdict sitting on screen has still written NOTHING to the gun by "
       "itself -- the whole screen is a measurement until somebody presses")
    n0 = len(ser.written)
    app.step([key(pygame.K_RETURN)], t + 17.2)
    ck(b"camfit=apply" in b" ".join(ser.written[n0:]),
       "only the user's own press applies it, and it applies the GUN's "
       "number by asking the gun to set it")
    n0 = len(ser.written)
    # The fake gun's camfit answers are queued by hand, so the ceiling it
    # would have stored has to be set by hand too: otherwise its camsave reply
    # echoes bhmax=0 and the verified save reports a disagreement that exists
    # only in the simulation.
    ser.state["bhmax"] = 10
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)",
               "CAM: fit applied and saved"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 17.3)
    ck("saved" in rs.done and "10 rows" in rs.done,
       "and the screen reports what the GUN did, not what pical asked for: "
       "%r" % rs.done)
    # '~camfit=apply' now persists the GATE and the report FORMAT itself --
    # switching the gun into full mode first if it had dropped out of it,
    # because the gate only acts there. The '~camsave' below still goes out,
    # belt-and-braces: it is how pical reads back what the gun says actually
    # reached flash, rather than trusting apply's own reply -- not what makes
    # the format persist any more. Before apply did this itself, the gate
    # could take the number and leave the format behind, so the ceiling came
    # back from a power cycle in a mode where the shape gate cannot run --
    # the number surviving and the gate not, which is the same no-op the
    # firmware's own format clamp used to cause.
    ck(b"camsave" in b" ".join(ser.written[n0:]),
       "applying also SAVES, belt-and-braces, so pical can read back "
       "whether the format the gate needs actually reached flash (%s)"
       % b" ".join(ser.written[n0:])[-60:])
    ck("full detail" in rs.done,
       "and the outcome says the format was kept, not just the number: %r"
       % rs.done)
    ck(rs.applied,
       "the screen knows a ceiling is live because of it, which is what "
       "stops the way out from undoing it")
    # THE defect: restore() used to send ~cam=ext:<pre-sweep> unconditionally,
    # and the sweep is normally entered from a gun in fmt:1 -- so the ceiling
    # the user had just applied was dead before they reached the menu, under a
    # screen that said it would hold through a power cycle.
    n0 = len(ser.written)
    app.step([key(pygame.K_ESCAPE)], t + 17.4)
    sent = b" ".join(ser.written[n0:])
    ck(b"cam=ext:" not in sent and b"cam=fmt:1" not in sent,
       "and leaving does NOT put the old report format back: the shape gate "
       "only acts in fmt:2, so handing it back would kill the ceiling that "
       "was just applied and saved (%s)" % sent)
    ck(app._cam_borrow is None,
       "while the borrow is still closed out, so nothing is left half-held")
    # Nothing applied means the format DOES go back -- the exception is for
    # the one case where giving it back would undo the user's own choice.
    app.link.last["fmt"] = 1
    app.link.hists.reset()
    app.begin_room_sweep()
    arm(0)
    n0 = len(ser.written)
    app.step([key(pygame.K_ESCAPE)], t + 17.5)
    ck(b"cam=ext:1" in b" ".join(ser.written[n0:]),
       "a sweep the user skipped still hands the report format back")
    app.link.last["fmt"] = 2
    # A gun that took the ceiling and says it cannot act on it. Current
    # firmware cannot produce this from '=apply' any more -- apply switches
    # the gun into full mode itself before it can ever answer INERT, and
    # this screen also puts it in full mode first on its own account -- so
    # the INERT line here stands for a gun that has NOT been reflashed and
    # is answering the old way. It should never happen on this build, and if
    # it somehow does, something refused the format and the user must be
    # told rather than reassured.
    app.link.hists.reset()
    app.begin_room_sweep()
    arm(0)
    rs = app.view
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 17.6)
    ser.state["bhmax"] = 10
    rs.apply()
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)",
               "CAM: fit applied and saved",
               "CAM: fit INERT RIGHT NOW -- the shape gate needs fmt:2 and "
               "this gun is in fmt:1; set fmt:2 for it to act"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 17.7)
    ck(app.fit.inert,
       "an older gun's INERT line is read rather than ignored, even though "
       "current firmware no longer sends it")
    ck("INERT" in rs.done and "Blob detail 2" in rs.done,
       "and the outcome says so instead of promising a gate that acts: %r"
       % rs.done)
    app.step([key(pygame.K_ESCAPE)], t + 17.8)
    # And a save that did not reach flash at all.
    ser.save_fails = True
    app.link.hists.reset()
    app.begin_room_sweep()
    arm(0)
    rs = app.view
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 17.9)
    ser.state["bhmax"] = 10
    rs.apply()
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)",
               "CAM: fit applied and saved"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 18.0)
    ck("could NOT be saved" in rs.done and "not act" in rs.done,
       "a format that never reached flash is reported as a gate that will "
       "not act, not as a success: %r" % rs.done)
    ser.save_fails = False
    app.step([key(pygame.K_ESCAPE)], t + 18.05)
    # ---- apply switches the format itself now ------------------------------
    # 'apply' used to store the gate and leave the report format behind, so
    # the ceiling came back from a power cycle in a mode where the shape gate
    # cannot run. It now switches the gun into full mode and persists THAT --
    # a change to the gun nobody asked for by name, and the right one, so the
    # screen has to say it happened rather than let the user find it later.
    app.link.last["fmt"] = 1
    app.link.hists.reset()
    app.begin_room_sweep()
    arm(0)
    rs = app.view
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 18.1)
    ser.state["bhmax"] = 10
    rs.apply()
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)",
               "CAM: fit switched to fmt:2 -- the shape gate needs it and the "
               "ceiling was measured in it",
               "CAM: fit applied and saved"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 18.15)
    ck("switched itself to full detail" in rs.done,
       "the outcome says the gun moved its own report format, and why: %r"
       % rs.done)
    ck("Set to 10 rows and saved" in rs.done,
       "...alongside what was actually set: %r" % rs.done)
    n0 = len(ser.written)
    app.step([key(pygame.K_ESCAPE)], t + 18.2)
    ck(b"cam=ext:1" not in b" ".join(ser.written[n0:]),
       "and leaving still does not undo it -- the gate the gun switched the "
       "format FOR would die with it")
    app.link.last["fmt"] = 2
    # A save that did not reach flash is gone on the next power cycle, and it
    # is the one outcome that looks identical to success from here.
    app.begin_room_sweep()
    rs = app.view
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 18.0)
    rs.apply()
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)",
               "CAM: fit applied but SAVE FAILED -- it will be gone on the "
               "next power cycle"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 18.1)
    ck("NOT save" in rs.done and "power cycle" in rs.done,
       "a gate that took but never reached flash is reported as exactly "
       "that: %r" % rs.done)
    app.step([key(pygame.K_ESCAPE)], t + 18.2)
    # And a gun that answers nothing at all does not leave the screen waiting
    # for ever -- a spinner that never stops is indistinguishable from a hang.
    app.begin_room_sweep()
    rs = app.view
    for ln in (HDR % (900, 7, 40, 13),
               "CAM: fit bhmax=10 (LEDs reach 7, stray starts at 13)"):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 19.0)
    rs.apply()
    rs._apply_t = time.monotonic() - (rs.APPLY_WAIT_S + 1.0)
    app.step([], t + 19.1)
    ck("did not answer" in rs.done and "Nothing has been changed" in rs.done,
       "a gun that never answers the apply is said so, and nothing is "
       "claimed to have changed: %r" % rs.done)
    app.step([key(pygame.K_ESCAPE)], t + 19.2)

    # ---- the capture starting over, out loud -------------------------------
    # The gun clears the capture whenever sensitivity or either sensor
    # threshold changes, because a distribution spanning the change is two
    # rigs averaged into one. That is the right call and it is invisible: a
    # user mid-sweep who nudges sensitivity watches 400 LED blobs become 0
    # with nothing anywhere to say why.
    for reply, why in (
            ("CAM: learn cleared -- sensitivity changed from 1 to 2, and a "
             "capture spanning both is not a measurement of either\n",
             "a capture cleared by a sensitivity change"),
            ("CAM: learn cleared -- hwmax changed\n",
             "one cleared by the sensor's own size ceiling moving"),
            ("CAM: learn cleared -- hwmin changed\n",
             "one cleared by its floor moving"),
            ("CAM: learn cleared\n", "one cleared outright by camreset")):
        app.toast = ""
        app.link.src.q.put(reply)
        app.step([], t + 19.4)
        ck(reply.split("\n")[0][:34] in app.toast,
           "%s reaches the user rather than only the log -- the progress bars "
           "are about to restart and nothing else says so: %r"
           % (why, app.toast))

    # ---- two failures the user could not otherwise see ---------------------
    # Both of these are states in which every row on the screen reads back
    # exactly what was asked for and the thing the user wanted is not
    # happening. pical has no visible log outside the auto-tune overlay, so a
    # line that only reaches the log is a line nobody will ever see.
    for reply, why in (
            ("CAM: bhmax 10 set but INERT -- the shape gate needs fmt:2 and "
             "this gun is in fmt:1\n",
             "a gate the gun took and cannot act on"),
            ("CAM: SAVE FAILED lead=0ms smooth=3 dead=0 beta=-1 lens=0\n",
             "a save that never reached flash")):
        app.toast = ""
        app.link.src.q.put(reply)
        app.step([], t + 19.5)
        ck(reply.split("\n")[0][:36] in app.toast,
           "%s is said out loud rather than left in a log nobody can read: "
           "%r" % (why, app.toast))

    # ---- the optional step is documented -----------------------------------
    # A whole screen missing from the step table is how a feature ends up
    # never being used: the README is the only place that says what pical
    # covers, and 4b is the one step that is skippable and therefore the one
    # most easily forgotten.
    readme = os.path.join(ROOT, "pical", "README.md")
    if os.path.isfile(readme):
        with open(readme) as fh:
            doc = fh.read()
        ck("4b" in doc and "room light" in doc.lower(),
           "the README's step table lists 4b, the room-light sweep")
        ck("skip" in doc.lower(),
           "and says it is skippable, which is the whole of why it is safe "
           "to offer at the end of a calibration")

    # ---- a gun that restarts mid-sweep -------------------------------------
    # A reconnect is not by itself a reboot: a port that re-enumerated hands
    # back the same running gun. A gun that RESTARTED comes up with the
    # capture off, its histograms empty, and its format at whatever flash
    # holds -- and nothing on this screen noticed. It went on polling, because
    # the gun still answers '~camfit?'; every answer said ledn=0 and NEEDS
    # MORE LED DATA; the screen went on saying "keep sweeping" at 0 of 500, at
    # a gun that was not measuring anything, for as long as the user was
    # willing to wave it about. And the borrow taken before the reboot
    # described a gun that no longer existed, so the eventual give-back would
    # have written a report format into a gun that never had it.
    app.link.last["fmt"] = 1
    app.link.hists.reset()
    app.begin_room_sweep()
    arm(0)
    rs = app.view
    ck(app._cam_borrow == (1, False), "a sweep is under way with a borrow held")
    app.link.gun_t = 400.0                     # the gun, 400 s into its boot
    for ln in (HDR % (300, 7, 5, -1),
               "CAM: fit NO STRAY DATA -- sweep the room with the screen in "
               "view so a lamp or window enters frame; 5 seen, 20 wanted. "
               "Your LEDs measured 7 tall."):
        app.link.src.q.put(ln + "\n")
    app.step([], t + 19.0)
    ck(app.fit.verdict == "need_stray" and app.fit.ledn == 300,
       "and it has got somewhere: 300 LED blobs (%s)" % app.fit.ledn)

    # The reboot. link_tick reconnects and records the clock the OLD session
    # had; the gun then answers on a clock that has started again.
    class Rebooted(FakeSrc):
        def is_alive(self):
            return False
    app.link.src = Rebooted()
    app.link.port = "SIM"
    app._link_t = 0.0
    app._link_retry = 0.0
    app.toast = ""
    def fake_connect(port=None):
        """What Link.connect really does on a reconnect: a new port handle,
        and everything it knew about the previous gun thrown away."""
        app.link.src = FakeSrc()
        app.link.last.clear()
        app.link.blobs = ""
        app.link.hists.reset()
        app.link.replies = []
        return True

    real_connect = app.link.connect
    app.link.connect = fake_connect
    try:
        app.link_tick(time.time())
    finally:
        app.link.connect = real_connect
    app.link.last["board"] = "rp2040-wiicam"    # the '~cam?' answer, arriving
    ck(app._gun_t0 == 400.0,
       "the clock the previous session ended on is kept across the reconnect, "
       "because it is the only thing that can tell a reboot from a "
       "re-enumeration (%s)" % app._gun_t0)
    n0 = len(app.link.src.ser.written)
    app.toast = ""
    app.link.gun_t = 0.9                        # a gun 0.9 s into a NEW boot
    app.step([], t + 19.1)
    ck("RESTARTED" in app.toast,
       "a gun whose own clock has gone backwards is called a restart, out "
       "loud: %r" % app.toast)
    ck("starting over" in app.toast,
       "and the user is told the measurement is starting over rather than "
       "left to wonder why the bars stopped moving: %r" % app.toast)
    ck(app.fit.verdict is None and app.fit.ledn == 0,
       "the verdict and the counts from before the reboot are dropped -- they "
       "describe a gun that no longer exists (%s, %s)"
       % (app.fit.verdict, app.fit.ledn))
    ck(app._cam_borrow is None and app._cam_arm is not None,
       "the stale borrow is gone and a fresh one is under way, so the sweep "
       "is pointed at a gun that is collecting again (%s, %s)"
       % (app._cam_borrow, app._cam_arm is not None))
    sent = b" ".join(app.link.src.ser.written[n0:])
    ck(b"cam=ext:1" not in sent and b"camlearn=on:0" not in sent,
       "and NOTHING is sent to put the old state back: the gun that would "
       "receive it is not the gun it was recorded from (%s)" % sent)
    arm(0)
    ck(app._cam_borrow is not None,
       "the fresh borrow completes against the gun that is actually there")
    sent = b" ".join(app.link.src.ser.written[n0:])
    ck(b"cam=fmt:2" in sent and b"camlearn=on:1" in sent,
       "having armed full mode and the capture again (%s)" % sent[-60:])
    # A re-enumeration that was NOT a reboot keeps the borrow, because the
    # gun on the other end is still the one the borrow was recorded from.
    app.link.gun_t = 900.0
    app._gun_t0 = 400.0
    held = app._cam_borrow
    app.toast = ""
    app.step([], t + 19.2)
    ck(app._cam_borrow == held and "RESTARTED" not in app.toast,
       "a clock that simply carried on is not a reboot, and the borrow "
       "survives it (%s vs %s)" % (app._cam_borrow, held))
    ck(app._gun_t0 is None, "and the question is answered once, not re-asked "
                            "every frame")
    # No frame since the reconnect at all: the decision WAITS for evidence
    # rather than guessing either way.
    app._gun_t0 = 400.0
    app.link.gun_t = 0.0
    app.step([], t + 19.3)
    ck(app._gun_t0 == 400.0,
       "with no frame to decide from, nothing is decided -- guessing early is "
       "the same mistake as not looking")
    app.link.gun_t = 900.0
    app.step([], t + 19.35)
    app.to_menu()
    attach(app)
    ser = app.link.src.ser
    app.link.last["board"] = "rp2040-wiicam"
    app.link.last["fmt"] = 2

    # ---- the sweep drawn, at every size, in every state --------------------
    # Brand-new UI, and the state it is in depends entirely on what the gun
    # has said -- so each outcome is drawn at each size. Two things must hold
    # in every one of the forty-nine: nothing is drawn off the screen (these
    # lines are CENTRED with no wrapping of their own, so an over-long one
    # does not clip, it loses a word off each end in silence), and the way out
    # is on screen. A step that cannot be skipped is a step that blocks a
    # calibration somebody has already finished.
    SIZES = ((640, 480), (800, 600), (1024, 768), (1280, 720),
             (1280, 1024), (1366, 768), (1920, 1080))
    STATES = (
        ("no answer yet", ()),
        ("not enough LED data", (HDR % (120, -1, 0, -1),
                                 "CAM: fit STORED ledmaxh=7 strayminh=12 -- "
                                 "from an earlier capture on this gun",
                                 "CAM: fit NEEDS MORE LED DATA -- run a "
                                 "calibration with the capture on; 120 blobs "
                                 "so far, 500 wanted")),
        ("no stray data", (HDR % (900, 7, 2, -1),
                           "CAM: fit NO STRAY DATA -- sweep the room with the "
                           "screen in view so a lamp or window enters frame; "
                           "2 seen, 20 wanted. Your LEDs measured 7 tall.")),
        ("no safe gate", (HDR % (900, 9, 40, 9),
                          "CAM: fit NO SAFE GATE -- your LEDs reach 9 tall "
                          "and the stray light starts at 9, so a size gate "
                          "cannot tell them apart. Move the bar, block the "
                          "light, or use brighter LEDs.")),
        ("a verdict", (HDR % (900, 7, 40, 13),
                       "CAM: fit bhmax=10 (LEDs reach 7, stray starts at "
                       "13)")),
        ("a tight verdict", (HDR % (900, 7, 40, 8),
                             "CAM: fit bhmax=8 (LEDs reach 7, stray starts at "
                             "8 -- TIGHT, only one step between them)")),
        # A verdict with the sun inside the LED measurement. The line the gun
        # sends about it starts with a DIGIT after 'CAM: fit ', which is why
        # it used to match no branch and reach the log alone.
        ("contaminated", (HDR % (900, 7, 40, 13),
                          "CAM: fit 32 LED samples ignored -- they reach 31 "
                          "tall, far above the 7 the rest stop at, and are "
                          "almost certainly stray light learned while the "
                          "resolver locked on it",
                          "CAM: fit bhmax=10 (LEDs reach 7, stray starts at "
                          "13)")),
        # And provenance from a gun that has been saved but never fitted:
        # camsave records the LED edge alone and leaves the stray side at 0.
        ("never fitted before", (HDR % (120, -1, 0, -1),
                                 "CAM: fit STORED ledmaxh=7 strayminh=0 "
                                 "ledmaxpx=84 -- from an earlier capture on "
                                 "this gun",
                                 "CAM: fit NEEDS MORE LED DATA -- run a "
                                 "calibration with the capture on; 120 blobs "
                                 "so far, 500 wanted")),
    )
    screens = [(w, h, pical.Screen(pygame.Surface((w, h)))) for w, h in SIZES]
    app.begin_room_sweep()
    rs = app.view
    n0 = len(ser.written)
    for what, lines in STATES:
        for ln in lines:
            app.link.src.q.put(ln + "\n")
        app.step([], t + 20.0)
        if what == "contaminated":
            # The same note the camera readout carries, on the screen the
            # user is actually looking at while they sweep. Under the verdict,
            # never instead of it: the ceiling is still the answer and this is
            # why it is that number.
            said = " ".join(m for _r, m, _i in texts_of(rs, screens[2][2]))
            ck("stray light" in said and "32" in said and "31" in said,
               "the sweep says which of its LED samples were really stray "
               "light, with the numbers: %s"
               % [m for _r, m, _i in texts_of(rs, screens[2][2])
                  if "stray light" in m or "32" in m])
            ck("left out" in said,
               "and that they are left out, so the ceiling beside it is not "
               "read as wrong")
        if what == "no answer yet":
            # The agreed description promises about fifteen seconds of
            # sweeping. A step with no duration on it reads as open-ended,
            # and an open-ended optional step is one people abandon.
            said = " ".join(m for _r, m, _i in texts_of(rs, screens[2][2]))
            ck("15 second" in said,
               "the sweep says roughly how long it takes: %s"
               % [m for _r, m, _i in texts_of(rs, screens[2][2])
                  if "15" in m])
        for w, h, sc_n in screens:
            try:
                drew = texts_of(rs, sc_n)
                threw = None
            except Exception as e:             # noqa: BLE001 - that IS the test
                drew, threw = [], e
            ck(threw is None, "the sweep survives %s at %dx%d (%r)"
               % (what, w, h, threw))
            if threw is not None:
                continue
            off = [m for r, m, _i in drew
                   if r.left < 0 or r.right > w or r.top < 0 or r.bottom > h]
            ck(not off, "and draws nothing off a %dx%d screen showing %s (%s)"
               % (w, h, what, [m[:36] for m in off]))
            ck(any("Esc" in m and "SKIP" in m for r, m, _i in drew),
               "and the way out is on screen at %dx%d showing %s" % (w, h, what))
            if what == "never fitted before":
                # Joined across the wrapped rows, because the sentence is
                # broken over two of them at these widths. The "never as a
                # zero" half of this claim is pinned on stored_words() itself
                # up above: down here the stray PROGRESS BAR legitimately
                # draws "room light" beside "0 / 20", and a text match cannot
                # tell that apart from a provenance line saying the same
                # thing.
                said = " ".join(m for _r, m, _i in drew)
                ck("Measured before on this gun" in said
                   and "not recorded" in said,
                   "a gun that has been saved but never fitted shows its "
                   "stray side as NOT RECORDED at %dx%d (%s)"
                   % (w, h, [m for _r, m, _i in drew
                             if "Measured" in m or "recorded" in m]))
    # The one state with no bars and no skip line: the outcome, which any
    # button leaves. It still has to fit.
    rs.done = ("Set to 8 rows, but the gun could NOT save it -- it will be "
               "gone on the next power cycle.")
    for w, h, sc_n in screens:
        drew = texts_of(rs, sc_n)
        off = [m for r, m, _i in drew
               if r.left < 0 or r.right > w or r.top < 0 or r.bottom > h]
        ck(not off, "the outcome fits a %dx%d screen too (%s)"
           % (w, h, [m[:36] for m in off]))
        ck(any("finish" in m for r, m, _i in drew),
           "and says how to leave it at %dx%d" % (w, h))
    ck(b"cam=bhmax:" not in b" ".join(ser.written[n0:]),
       "and drawing every state of the sweep at every size never wrote a "
       "gate value to the gun")
    app.step([key(pygame.K_ESCAPE)], t + 20.5)

    # ---- the camera page's two stacked panels ------------------------------
    # The sensor panel used to REPLACE the camera view. They answer different
    # questions -- the view draws the resolved quad, the panel draws every
    # blob the sensor handed over including the ones a gate threw away -- and
    # with only the second on screen a gate that had started taking a real
    # corner looked exactly like a gate that was working. Stacked, they are
    # both height-limited, and the readout underneath is CENTRED and nearly
    # full width, so it has to clear the whole column. Measured at every size,
    # because the faces are chosen from the screen's height and the room the
    # column has is a fraction of its width.
    app.link.last["board"] = "rp2040-wiicam"
    cam7 = pical.Camera(app)
    app.open(cam7)
    app.link.blobs = "CAM: blobs " + " ".join(
        blob9(*b) for b in ((50, 45, 3, 0, 11, 9, 84),
                            (190, 48, 4, 0, 12, 10, 79),
                            (52, 150, 3, 0, 10, 9, 80),
                            (195, 152, 14, 0, 41, 38, 255)))
    app.link.last.update({"fmt": 2, "bhmax": 10, "bframes": 3000, "bms": 30000,
                          "brej": 40, "brrej": 18, "bvalve": 4, "bsrej": 6,
                          "bnear": 0, "br4": 1000, "br3": 60, "br2": 20,
                          "br1": 5, "br0": 0})
    cam7._rate.feed(3000, 30000)
    cam7.blob_lines()
    cam7.log_toggle()
    app.link.last.update({"bframes": 3100, "bms": 31000, "brej": 90,
                          "brrej": 39, "bvalve": 90, "bsrej": 20, "bnear": 9,
                          "br4": 1200, "br3": 90, "br2": 30, "br1": 8,
                          "br0": 1})
    cam7._rate.feed(3100, 31000)
    cam7.blob_lines()
    for w, h, sc_n in screens:
        for advanced in (False, True):
            page(cam7, advanced)
            drew = texts_of(cam7, sc_n)
            what = "%dx%d %s" % (w, h, "second page" if advanced else "page 1")
            vr, pr = cam7.view_rect, cam7.shape_rect
            ck(vr is not None and pr is not None,
               "%s: both panels are drawn and say what they covered" % what)
            ck(not vr.colliderect(pr),
               "%s: and they are stacked, not overlapping (%s over %s)"
               % (what, vr, pr))
            rows = [r.rect for r in cam7.rows if r.rect]
            hit = [r for r in rows if vr.colliderect(r) or pr.colliderect(r)]
            ck(not hit, "%s: the column clears every row beside it (%s)"
               % (what, hit))
            under = [m for r, m, ins in drew
                     if ins and (vr.colliderect(r) or pr.colliderect(r))]
            ck(not under,
               "%s: and the readout starts below the whole column rather "
               "than through it (%s)" % (what, [m[:24] for m in under]))
            tip_top = h * pical.TIP_Y - sc_n.f_s.get_height() / 2.0
            ck(pr.bottom <= tip_top - 6,
               "%s: the column's own numbers clear the row hint (column ends "
               "%d, hint starts %d)" % (what, pr.bottom, tip_top))
            off = [m for r, m, _i in drew
                   if r.left < 0 or r.right > w or r.top < 0 or r.bottom > h]
            ck(not off, "%s: nothing is drawn off the screen (%s)"
               % (what, [m[:36] for m in off]))
            n_rows = len(set(r.top for r, _m, ins in drew if ins))
            ck(n_rows >= 5,
               "%s: and the readout still gets a useful number of rows (%d)"
               % (what, n_rows))
            # A caption wider than the panel it names is worse than no
            # caption, and these panels are half the width they used to be.
            for word, rect in (("SENSOR", pr), ("CAMERA", vr)):
                cap = [m for r, m, ins in drew
                       if not ins and m.startswith(word) and rect.colliderect(r)]
                ck(cap and sc_n.f_xs.size(cap[0])[0] <= rect.width,
                   "%s: the %s caption fits inside its own panel (%s, %d of "
                   "%d px)" % (what, word, cap,
                               sc_n.f_xs.size(cap[0])[0] if cap else -1,
                               rect.width))
    page(cam7, False)
    cam7.close_log()
    app.to_menu()


    # ---- one question at a time --------------------------------------------
    # The gun answers in the order it is asked and there is ONE wire, so an
    # answer being sent is a camera frame that is not. The blob poll ran on a
    # 1 s period and the histogram poll on 2 s, both seeded from zero in
    # __init__ -- exact harmonics, so every second blob poll landed in the
    # same frame as a histogram poll and the gun replied with one ~3.2 KB
    # burst: a quarter of a second in which the preview received nothing at
    # all, every two seconds. That is what "the preview was laggy" was. The
    # screen now holds each question until the last answer can have finished.
    #
    # Driven on a simulated clock, because the periods are seconds long and
    # six hundred real frames go by in a fraction of one.
    app.link.last["board"] = "rp2040-wiicam"
    cam9 = pical.Camera(app)
    app.open(cam9)
    cam9.log_toggle()                      # the fastest the blob poll ever runs
    app.link.hists.summary = {"on": 1, "frames": 9, "led": 9, "rej": 0}
    asked, worst = [], 0
    real_mono, fake = time.monotonic, {"t": 9000.0}
    time.monotonic = lambda: fake["t"]
    real_send = app.link.send
    try:
        for i in range(1800):              # thirty seconds at the real 60 fps
            fake["t"] = 9000.0 + i / 60.0
            batch = []
            app.link.send = lambda ln, b=batch: (b.append(ln), real_send(ln))[1]
            try:
                cam9.handle([], (0, 0))
            finally:
                app.link.send = real_send
            qs = [ln for ln in batch if ln.endswith("?")]
            worst = max(worst, len(qs))
            asked.extend((fake["t"], ln) for ln in qs)
    finally:
        app.link.send = real_send
        time.monotonic = real_mono
    cam9.close_log()
    ck(worst <= 1,
       "no single frame ever asks the gun two questions -- sent together they "
       "come back as one burst, which is the whole failure (worst %d)" % worst)
    counts = {}
    for _tt, ln in asked:
        counts[ln] = counts.get(ln, 0) + 1
    ck(counts.get("~camblob?", 0) >= 105,
       "and spacing the questions does not starve the poll that feeds the "
       "sensor panel and the blob log -- it asks four times a second and a "
       "question it loses goes out on the next frame, not on the next period "
       "(%d in thirty seconds)" % counts.get("~camblob?", 0))
    ck(2 <= counts.get("~camfit?", 0) <= 14
       and 2 <= counts.get("~camlearn?", 0) <= 8,
       "the other two still get through, on their own much slower clocks "
       "(%s)" % counts)
    ck(counts.get("~camlearn?", 0) <= counts.get("~camblob?", 0) / 8,
       "the 2.6 KB histogram answer is asked for FAR less often than the "
       "470-byte one -- at the two seconds it used to run at it was an "
       "eighth of the whole link (%s)" % counts)
    # And specifically: nothing is asked in the window the big answer needs.
    after_learn = [b[0] - a[0] for a, b in zip(asked, asked[1:])
                   if a[1] == "~camlearn?"]
    ck(after_learn and min(after_learn) >= cam9.LEARN_REPLY_S - 1e-9,
       "and nothing else is asked until the histogram dump can have finished, "
       "so its quarter-second of wire is never doubled (closest %.2f s, needs "
       "%.2f)" % (min(after_learn) if after_learn else -1, cam9.LEARN_REPLY_S))
    app.link.hists.summary = {}
    app.to_menu()

    # ---- a gun on firmware older than the fit ------------------------------
    # It answers '~camfit' with 'AIM: unknown command' -- once every couple of
    # seconds, for ever, into a 200-line log ring that is where a user is sent
    # to read the diag verdict and the result of a save. Asked once, then left
    # alone, and every screen that would have shown a verdict says plainly
    # that this gun cannot measure one.
    class OldSer(FakeSer):
        """A gun that knows the blob and histogram commands but not the fit."""

        def __init__(self, replies, q):
            FakeSer.__init__(self, replies)
            self.q = q

        def write(self, b):
            n = FakeSer.write(self, b)
            for line in b.decode("ascii", "replace").split("\n"):
                line = line.strip().lstrip("~")
                if line.startswith("camfit"):
                    self.q.put('AIM: unknown command "%s"\n' % line)
            return n

    old = FakeSrc()
    old.ser = OldSer(old.replies, old.q)
    app.link.src = old
    app.fit.reset()
    app.link.last["board"] = "rp2040-wiicam"
    cam8 = pical.Camera(app)
    app.open(cam8)
    # Half a minute of SCREEN time, on a simulated clock -- the poll runs on
    # a 2.5 s period and sixty real frames go by in a fraction of a second,
    # so a loop paced by the wall clock would never reach a second poll and
    # would pass whether or not the refusal was remembered.
    real_mono, fake = time.monotonic, {"t": 5000.0}
    time.monotonic = lambda: fake["t"]
    try:
        for i in range(60):
            app.step([], t + 21.0 + i * 0.5)
            fake["t"] += 0.5
    finally:
        time.monotonic = real_mono
    asked = sum(1 for w in old.ser.written if b"camfit" in w)
    ck(asked == 1,
       "an old gun is asked for the fit ONCE, not every two and a half "
       "seconds for the rest of the session (%d asks in thirty seconds)"
       % asked)
    ck(not app.fit.supported and app.fit.verdict is None,
       "the refusal is remembered rather than read as a verdict")
    ck(sum(1 for l in app.log_lines if "unknown command" in l) <= 1,
       "so it costs the log one line, not a thousand an hour")
    ck("unknown command" not in app.toast,
       "and it is not shouted at the user, who can do nothing about it")
    ck(not [l for l in cam8.blob_lines()
            if "NO SAFE GATE" in l or "REFUSING HEAVILY" in l],
       "a gun that cannot answer raises no gate verdict -- silence is not "
       "'no gate can work here'")
    tip = [r for r in cam8.rows
           if r.label == "Biggest blob (height)"][0].tip()
    ck("Blob detail 2" in tip and "drops" in tip,
       "and the gate row falls back to what it is rather than to a blank: "
       "%r" % tip)
    app.begin_room_sweep()
    rs = app.view
    said = " ".join(rs.verdict_lines()[0])
    ck("cannot measure" in said and "Skip" in said,
       "the sweep says outright that this firmware cannot do it, instead of "
       "spinning: %r" % said)
    for w, h, sc_n in screens:
        drew = texts_of(rs, sc_n)
        off = [m for r, m, _i in drew
               if r.left < 0 or r.right > w or r.top < 0 or r.bottom > h]
        ck(not off, "and it still fits a %dx%d screen (%s)"
           % (w, h, [m[:36] for m in off]))
    app.step([key(pygame.K_ESCAPE)], t + 26.0)
    ck(isinstance(app.view, pical.Menu),
       "and Esc still leaves cleanly on a gun that answered none of it")
    attach(app)
    ser = app.link.src.ser
    app.link.last["board"] = "rp2040-wiicam"

    # ---- every recording carries a number ----------------------------------
    # The Pi has NO real-time clock, so two captures taken on two different
    # evenings are stamped with the same second. Numbering is what makes the
    # names unique, and there are now TWO places that do it: pical numbers the
    # files it writes itself, and CaptureSession.save() numbers the
    # calibration's own pair. What must hold is that they agree -- one number
    # means one capture, whichever tool wrote it -- and that neither renumbers
    # the other's work.
    rec_dir = tempfile.mkdtemp(prefix="pical-rec-")
    real_out, pical.OUT_DIR = pical.OUT_DIR, rec_dir
    try:
        sess = pical.CaptureSession(plan=pical.make_plan(2, 0))
        sess.shots = [dict(tx=0.5, ty=0.5, q=np.zeros((4, 2)))]
        sess.raw = [np.zeros((4, 2))]
        first = sess.save(rec_dir)
        second = sess.save(rec_dir)
        names = sorted(os.listdir(rec_dir))
        ck(len(names) == 4,
           "two calibrations saved in the SAME second leave four files, not "
           "two (%s)" % names)
        nums = {}
        for n in names:
            nums.setdefault(n.split("-")[1], set()).add(n.split("-")[0])
        ck(all(v == {"shots", "rawquads"} for v in nums.values()),
           "the shot log and the raw quads behind it share ONE number, "
           "because they are two halves of one capture and 'send me number "
           "12' has to fetch both (%s)" % nums)
        # pical used to move these onto its OWN numbering afterwards, from a
        # time when the session named them off the clock. Once the session
        # started numbering them that renaming burned a number per
        # calibration and the count went 2, 4, 6 -- so what has to be true now
        # is that pical does not touch them at all.
        ck(not hasattr(pical, "save_session"),
           "and pical does not renumber them a second time")
        got = sorted(int(n) for n in nums)
        ck(got == [got[0], got[0] + 1],
           "two calibrations advance the count by ONE each, not by two (%s)"
           % got)
        # And pical's own recordings count ABOVE them, so a number still names
        # exactly one capture across both tools.
        blob, nb = pical.recording_path("blobs")
        ck(nb > got[-1],
           "a blob log taken after a calibration counts above it rather than "
           "reusing its number (calibrations %s, blob log %d)" % (got, nb))
        open(blob, "w").close()
        # A sweep that was REFUSED is exactly the one somebody wants to look
        # at afterwards, so those are numbered too.
        lens = pical.Lens.__new__(pical.Lens)
        a = pical.Lens.save_sweep(lens, np.zeros((3, 4, 2)))
        b = pical.Lens.save_sweep(lens, np.zeros((3, 4, 2)))
        ck(a and b and a != b,
           "two lens sweeps in the same second are two files (%s, %s)"
           % (os.path.basename(a), os.path.basename(b)))
        ck(os.path.basename(a).startswith("lenssweep-")
           and os.path.basename(a).split("-")[1].isdigit(),
           "under the same numbering as everything else (%s)"
           % os.path.basename(a))
        ck(pical.Lens.save_sweep(lens, np.zeros((0, 4, 2))) == "",
           "and a sweep with no frames in it writes nothing at all")
        seen, clash = set(os.listdir(rec_dir)), []
        for pref, ext in (("blobs", ".csv"), ("shape", ".csv"),
                          ("lenssweep", ".log")) * 12:
            q, _n = pical.recording_path(pref, ext)
            if os.path.basename(q) in seen:
                clash.append(os.path.basename(q))
            seen.add(os.path.basename(q))
            open(q, "w").close()
        ck(not clash,
           "and thirty-six recordings of three kinds inside one second never "
           "reuse a name (%s)" % clash[:3])
    finally:
        pical.OUT_DIR = real_out

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
