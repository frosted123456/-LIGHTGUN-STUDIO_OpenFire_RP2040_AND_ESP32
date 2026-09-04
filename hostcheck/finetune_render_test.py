#!/usr/bin/env python3
"""Renders the fine-tune screen and drives the real two-station flow end to end,
asserting the ring is drawn where it is measured, that nudges preview without
persisting, and that the lead buttons accumulate and clamp.
"""
import sys, os, time, threading, queue, subprocess
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import tkinter as _tk
errors = []
_o = _tk.Tk.report_callback_exception
def catch(self, e, v, tb):
    errors.append("%s: %s" % (e.__name__, v)); _o(self, e, v, tb)
_tk.Tk.report_callback_exception = catch

import aim_calib as A
import aim_fit
import aim_finetune as F

SENT = []
sess = A.CaptureSession(plan=A.make_plan(3, 0))
sim = A.SimSource(sess, blob_sigma=0.3, lever_m=0.03, bore=(11.0, -3.0))

class FakeSrc:
    def __init__(self): self.q = queue.Queue()
    def feed(self, dist):
        q = sim._quad(F.TARGET[0], F.TARGET[1], dist, 0.0, stance=0, dot=0)
        # The wiicam's 14-field form: kind c, every corner measured, no lead.
        self.q.put("Q,0,4," + ",".join(str(int(round(v*10))) for v in q.reshape(-1))
                   + ",c,15,0,0")

src = FakeSrc()
# A calibration actually FITTED to this simulated gun: a hand-made dict would not
# solve its quads anywhere sensible and the ring measurement would refuse them.
_cal_src = A.SimSource(A.CaptureSession(plan=A.make_plan(3, 0)), blob_sigma=0.0,
                       tremor_deg=0.0, lever_m=0.0, bore=(5.0, -3.0))
_cal_src.rng = np.random.default_rng(1)
_cal_src._stance_bias *= 0; _cal_src._dot_bias *= 0
_sh = []
for _si, _d in enumerate((1.2, 1.7, 2.3)):
    for _di, (_tx, _ty) in enumerate(A.DOTS):
        _sh.append(dict(q=_cal_src._quad(_tx, _ty, _d, 0.0, stance=_si, dot=_di),
                        tx=_tx, ty=_ty, stance=_si))
CAL = aim_fit.fit(_sh, A.FRAME_W, A.FRAME_H)[0]
tuner = F.Tuner(CAL)
DIST = {"d": 1.3}

def driver():
    time.sleep(1.5)
    root = _tk._default_root
    def press(label):
        for b in root.children and []:
            pass
        # drive through the same handler the canvas click uses
        root.event_generate("<Return>") if label == "done" else None
    # THE RING MUST BE DRAWN WHERE IT IS MEASURED. Only the real canvas knows
    # where it was painted; a mismatch bakes a fixed error into every fine tune
    # and still looks like a working correction.
    cvs = [w for w in root.winfo_children() if isinstance(w, _tk.Canvas)]
    if not cvs:
        errors.append("no canvas found")
    else:
        c = cvs[0]
        W, H = c.winfo_width(), c.winfo_height()
        # poll: the canvas is cleared and repainted every tick, so a snapshot
        # taken between the delete and the redraw would see nothing
        # ...and the same race sits between find_all() and coords(): the id
        # found can be deleted by the next tick before its coordinates are
        # read, and coords() of a dead item is an empty list. So the poll
        # covers both, and only breaks once it holds four numbers.
        ovals, xy = [], []
        _dl = time.time() + 3.0
        while time.time() < _dl:
            ovals = [i for i in c.find_all() if c.type(i) == "oval"]
            xy = c.coords(ovals[0]) if ovals else []
            if len(xy) == 4: break
            time.sleep(0.03)
        if len(xy) != 4:
            errors.append("the ring is not drawn at all")
        else:
            x0, y0, x1, y1 = xy
            gx, gy = (x0+x1)/2.0/W, (y0+y1)/2.0/H
            if abs(gx - F.TARGET[0]) > 0.005 or abs(gy - F.TARGET[1]) > 0.005:
                errors.append("ring drawn at (%.3f, %.3f) but measured against "
                              "TARGET (%.3f, %.3f)" % (gx, gy, *F.TARGET))
            else:
                print("ring drawn at (%.3f, %.3f), matches TARGET" % (gx, gy))

    # station 1: shoot the ring, then nudge
    for _ in range(20): src.feed(1.3)
    time.sleep(0.4)
    tuner.note_quad(sim._quad(F.TARGET[0], F.TARGET[1], 1.3, 0.0, stance=0, dot=0))
    if not tuner.measured[0]:
        errors.append("shooting the ring did not measure an offset: %s" % tuner.msg)
    for _ in range(6): root.event_generate("<Left>")
    for _ in range(3): root.event_generate("<Up>")
    time.sleep(0.4)
    root.event_generate("<Return>")                 # NEXT
    time.sleep(0.4)
    if tuner.stage != 1: errors.append("NEXT did not advance to station 2")
    # station 2, further away
    DIST["d"] = 2.4
    for _ in range(20): src.feed(2.4)
    time.sleep(0.4)
    tuner.note_quad(sim._quad(F.TARGET[0], F.TARGET[1], 2.4, 0.0, stance=0, dot=0))
    for _ in range(4): root.event_generate("<Left>")
    time.sleep(0.4)
    # screenshot WHILE a flash is live, so the lit state is rendered at least once
    for _ in range(3): root.event_generate("<plus>")
    time.sleep(0.10)
    subprocess.run("import -window root /tmp/finetune_mid.png", shell=True, capture_output=True)
    lead_cmds = [s for s in SENT if s.startswith("~cam=lead:")]
    if len(lead_cmds) != 3:
        errors.append("3 lead presses produced %d commands: %s" % (len(lead_cmds), lead_cmds))
    elif lead_cmds[-1] != "~cam=lead:%d" % (3*F.LEAD_STEP):
        errors.append("lead did not accumulate: %s" % lead_cmds)
    root.event_generate("<minus>"); time.sleep(0.05)
    if SENT[-1] != "~cam=lead:%d" % (2*F.LEAD_STEP):
        errors.append("LEAD - did not step down: %s" % SENT[-1])
    # and it must clamp rather than run away
    for _ in range(20): root.event_generate("<plus>")
    time.sleep(0.05)
    if SENT[-1] != "~cam=lead:%d" % F.LEAD_MAX:
        errors.append("lead did not clamp at %d: %s" % (F.LEAD_MAX, SENT[-1]))
    root.event_generate("<Return>")                 # FINISH
    time.sleep(0.6)
    if tuner.stage != 2:
        errors.append("FINISH did not complete: %s" % tuner.msg)
    subprocess.run("import -window root /tmp/finetune.png", shell=True, capture_output=True)
    # the preview must have gone out as a NON-persisting line
    prev = [s for s in SENT if s.startswith("~aimcal!=")]
    if not prev:
        errors.append("no preview lines were sent (aimcal!= never used)")
    if any(s.startswith("~aimcal=") for s in SENT):
        errors.append("a PERSISTING aimcal= was sent during nudging -- that is a "
                      "flash write per button press")
    if not any(s.startswith("~cam=lead:") for s in SENT if "lead" in s):
        pass    # lead was not exercised here; the maths test covers the value
    print("preview lines sent: %d" % len(prev))
    print("finetune render: %s" % ("OK" if not errors else "FAILED -- %s" % errors[0]))
    time.sleep(0.2)
    os._exit(1 if errors else 0)

threading.Thread(target=driver, daemon=True).start()
out = F.run_gui(src, tuner, lambda l: SENT.append(l))
