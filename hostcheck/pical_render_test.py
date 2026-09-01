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

    # ---- the ambient-light readout ---------------------------------------
    app.link.last["board"] = "rp2040-wiicam"
    cam3 = pical.Camera(app)
    labels = [r.label for r in cam3.rows]
    ck("Blob detail (sizes)" in labels and "Largest blob kept" in labels,
       "the wiicam camera screen offers the blob size window")
    app.open(cam3)
    cam3.sel = labels.index("Largest blob kept")
    n0 = len(ser.written)
    app.step([key(pygame.K_LEFT)], t + 9.2)
    ck(any(b"cam=bmax:" in w for w in ser.written[n0:]),
       "and nudging it sends the gate to the gun")
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
    # bnear counts blobs a gate threw away that sat exactly where the missing
    # corner had to be. It is the only number on this screen that says a gate
    # is WRONG, and worded as one more drop count it would read as the gate
    # earning its keep -- the exact opposite. It has to be a warning, and it
    # has to be a delta like everything else here.
    ck("GATE MAY BE TAKING REAL LEDs" not in "\n".join(cam3.blob_lines()),
       "a bnear that has not moved raises nothing")
    app.link.last.update({"bframes": 2300, "bms": 23000, "bnear": 3,
                          "br4": 1500, "br3": 190, "br2": 60, "br1": 15,
                          "br0": 4})
    near = "\n".join(cam3.blob_lines())
    ck("GATE MAY BE TAKING REAL LEDs" in near,
       "bnear moving says the gate may be taking real LEDs: %s"
       % [l for l in cam3.blob_lines() if "GATE" in l])
    ck("3 dropped by" not in near and "3 dropped as" not in near,
       "and it is NOT phrased as another drop count -- read as one it looks "
       "like the gate working, which is the opposite of what it means: %s"
       % [l for l in cam3.blob_lines() if "GATE" in l])
    ck("corner" in near,
       "it says WHY the blob was probably an LED -- it sat where a corner "
       "should have been")
    app.link.last.update({"bframes": 2400, "bms": 24000,
                          "br4": 1600, "br3": 220, "br2": 70, "br1": 18,
                          "br0": 5})
    ck("GATE MAY BE TAKING REAL LEDs" not in "\n".join(cam3.blob_lines()),
       "and it clears once the gate stops doing it, rather than latching on "
       "a since-boot total for the rest of the power cycle")

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
    cam3.draw(pi_sc)
    ys = sorted(r.rect.centery for r in cam3.rows if r.rect)
    pitch = min(b - a for a, b in zip(ys, ys[1:]))
    ck(len(cam3.rows) >= 16,
       "the camera screen really is carrying sixteen rows now (%d)"
       % len(cam3.rows))
    ck(pitch - pi_sc.f_m.get_height() >= 3.0,
       "and the rows still clear each other by a real margin at 1024x768: "
       "%d px of pitch for a %d px line box (%.1f px of daylight)"
       % (pitch, pi_sc.f_m.get_height(),
          pitch - pi_sc.f_m.get_height()))
    # ...and clear the subtitle above them, which is the space the band was
    # grown into.
    sub_bottom = pi_sc.h * 0.145 + pi_sc.f_s.get_height() / 2.0
    ck(ys[0] - pi_sc.f_m.get_height() / 2.0 >= sub_bottom,
       "the first row starts below the subtitle (row top %d, subtitle ends "
       "%d)" % (ys[0] - pi_sc.f_m.get_height() / 2.0, sub_bottom))

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
    app.link.last.update({"bframes": 2300, "bms": 23000, "bvalve": 90,
                          "bnear": 9,
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

    # The sensor's own thresholds, the odd-one-out gate, the shape gate, the
    # report format and the full-mode register all reach the gun. fmt: is sent
    # ONLY for full mode: the previous firmware has no such key and drops it
    # in silence, so a gun on it could never be moved off detail 0 at all.
    for label, wire in (("Odd-one-out (size steps)", b"cam=rtol:"),
                        ("Biggest blob (pixels)", b"cam=pxmax:"),
                        ("Roundness limit", b"cam=armax:"),
                        ("Sensor max size (0x06)", b"cam=hwmax:"),
                        ("Sensor min size (0x1B)", b"cam=hwmin:"),
                        ("Full-mode register (0x33)", b"cam=fullreg:")):
        ck(label in labels, "the camera screen offers '%s'" % label)
        cam3.sel = labels.index(label)
        n0 = len(ser.written)
        app.step([key(pygame.K_RIGHT)], t + 9.5)
        ck(any(wire in w for w in ser.written[n0:]),
           "and nudging it sends %s to the gun" % wire.decode())

    # ---- the shape gate, in the reader's units and inside the firmware's
    # ---- refusal ranges ----------------------------------------------------
    # The gun REFUSES a pxmax of 1..11 and an armax of 1..15 outright -- it
    # answers by name and leaves the old value alone. On a screen with no
    # console that is an arrow that visibly does nothing, so the ladders are
    # built so it cannot be reached: every rung, stepped from every other
    # rung, in both directions, has to be a value the firmware takes.
    for label, floor in (("Biggest blob (pixels)", 12),
                         ("Roundness limit", 16)):
        row = cam3.rows[labels.index(label)]
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
    ar = cam3.rows[labels.index("Roundness limit")]
    ck(ar.show(20) == "2.5:1" and ar.show(16) == "2:1"
       and ar.show(24) == "3:1" and ar.show(0) == "off",
       "the roundness row shows a RATIO, not the wire's eighths (%s)"
       % [ar.show(v) for v in (0, 16, 20, 24, 32)])
    px = cam3.rows[labels.index("Biggest blob (pixels)")]
    ck(px.show(14) == "14 px" and px.show(0) == "off" and px.show(None) == "--",
       "and the pixel row names its unit, with 'off' for the value that "
       "means off (%s)" % [px.show(v) for v in (None, 0, 14)])
    # Whatever the gun says, including what it cannot mean. A firmware that
    # means something else by the key, or a value typed at a serial terminal,
    # must not take the screen down: pical is fullscreen with no console, so
    # an exception inside draw is a black TV.
    for v in (-1, 7, 255, 10 ** 9):
        for r_ in (ar, px):
            ck(isinstance(r_.show(v), str),
               "'%s' survives a value the gun should never send (%r -> %r)"
               % (r_.label, v, r_.show(v)))
    for label in ("Biggest blob (pixels)", "Roundness limit"):
        tip = cam3.rows[labels.index(label)].tip()
        ck("Blob detail 2" in tip,
           "'%s' says it needs full detail -- in any other format the gate "
           "stands down and the row does nothing: %r" % (label, tip))
        ck("drops" in tip,
           "and says what it physically throws away, not what it sets: %r"
           % tip)

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
    ck("NOT saved" in cam3.rows[labels.index("Full-mode register (0x33)")].tip(),
       "and the register's own row says it too")

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
    ck("shape-DATE.csv" in csv_row.tip() and "0 LED" not in csv_row.tip(),
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
    time.sleep(1.1)                            # a new second, so a new name
    cam4.shape_save()
    shapes = sorted(f for f in os.listdir(pical.OUT_DIR)
                    if f.startswith("shape-"))
    ck(len(shapes) == 2, "a second capture writes a second file rather than "
                         "overwriting the first (%s)" % shapes)
    hdr, rows = csv_rows(os.path.join(pical.OUT_DIR, shapes[-1]))
    got = {(r[0], r[1]): r for r in rows}
    ck(len(rows) == 12 and got[("rej", "irel")][5] == "12",
       "a full capture writes all twelve, each with its own bins (%s)"
       % got[("rej", "irel")][5:9])
    ck(all(r[3] == "13120" and r[4] == "268" for r in rows),
       "with the LED and rejected counts on every row")
    ck("13120 LED" in app.toast,
       "and the toast reports what was captured: %r" % app.toast)

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
