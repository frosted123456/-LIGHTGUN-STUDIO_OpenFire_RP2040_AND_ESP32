#!/usr/bin/env python3
"""pical: every view renders, and a whole calibration completes end to end.

Runs headless on SDL's dummy driver, so the pygame app is covered by the same
suite as the desktop tools. Frames come from aim_calib's SimSource model, fed
on a synthetic gun clock so a full multi-stance run takes seconds.
"""
import os
import queue
import sys

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


class FakeSrc:
    """A SerialSource with no port: the queue is filled by the test."""

    def __init__(self):
        self.q = queue.Queue(maxsize=8000)

    def close(self):
        pass


def main():
    pygame.init()
    surf = pygame.display.set_mode((1280, 720))

    app = pical.App(surf, stances=2)
    ck(isinstance(app.view, pical.Menu), "opens on the menu")

    # the menu draws and navigates with no gun attached
    app.step([], 0.0)
    ck(True, "menu renders with no gun connected")
    ev = [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_DOWN)]
    app.step(ev, 0.1)
    ck(app.view.sel == 1, "menu navigates on a key press")
    # selecting Start without a gun must refuse, not crash or begin
    app.view.sel = 0
    app.step([pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RETURN)], 0.2)
    ck(isinstance(app.view, pical.Menu) and app.toast,
       "Start without a gun refuses and says so")

    # attach a fake link and run a whole calibration
    app.gun.src = FakeSrc()
    app.gun.port = "SIM"
    app.gun.t_open = 1e12                    # never trips the no-data screen
    app.begin_calib()
    ck(isinstance(app.view, pical.Calib), "Start enters the capture view")
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
        app.gun.src.q.put("Q,%d,4,%d,%d,%d,%d,%d,%d,%d,%d"
                          % (int(t * 1000), *v))
        t += 1.0 / 60.0
        app.step([], t)

    ck(app.session is None, "the run reached the end and left the capture view")
    ck(isinstance(app.view, pical.Result), "lands on the result view")
    r = app.view
    ck(r.c is not None, "the fit produced a calibration: %s"
       % (r.why or "ok"))
    if r.c:
        px = r.c["fit_rms"] * ((1920.0 ** 2 + 1080.0 ** 2) ** 0.5) / (2 ** 0.5)
        ck(px < 25.0, "fit error %.1f screen px (want < 25)" % px)
        ck(0.05 < r.c["w"] < 2.0 and 0.05 < r.c["h"] < 2.0,
           "LED rectangle is plausible (%.3f x %.3f)" % (r.c["w"], r.c["h"]))
        # no serial port on a FakeSrc: the install must be reported, not attempted
        ck("NOT SENT" in (r.install or "").upper() or r.install == "",
           "no-port run does not claim the gun was written")
    app.step([], t)
    ck(True, "result view renders")

    # every remaining view state renders: refused fit, and both stepback kinds
    ref = pical.Result(app, None, "span spread too small", "", None)
    ref.draw(pical.Screen(surf))
    ck(True, "refused-fit view renders")

    app.begin_calib()
    s2 = app.session
    s2.state = s2.S_STEPBACK
    app.view.draw(pical.Screen(surf))
    s2.plan[min(1, len(s2.plan) - 1)]["kind"] = "roll"
    s2.plan[min(1, len(s2.plan) - 1)]["roll"] = 1
    s2.stance = min(1, len(s2.plan) - 1)
    app.view.draw(pical.Screen(surf))
    ck(True, "step-back and tilt prompts render")

    # the no-data screen is what a wrong port looks like
    app.gun.t_open = 0.0
    app.gun.frames = 0
    app.view.draw(pical.Screen(surf))
    ck(True, "no-data screen renders")

    pygame.quit()
    print("\npical: %s (%d failures)" % ("ALL PASS" if not FAILS else "FAILED",
                                         len(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
