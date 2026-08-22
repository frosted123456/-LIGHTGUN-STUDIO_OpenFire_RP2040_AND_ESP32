#!/usr/bin/env python3
"""Renders EVERY screen of the calibration GUI and requires tick() to survive each
one, and asserts the results screen reports the real install outcome. This covers
the event loop, which --selftest never touches. States are driven directly.
"""
import sys, os, time, threading, subprocess, queue
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import aim_calib as A
import numpy as np

# reach into the GUI's state dict once run_gui has created it
def fill_shots():
    for si, d in enumerate((1.2, 1.7, 2.3)):
        for di, (tx, ty) in enumerate(A.DOTS):
            sess.shots.append(dict(q=src._quad(tx, ty, d, 0.0, stance=si, dot=di),
                                   tx=tx, ty=ty, stance=si, sigma=0.31, span=70.0,
                                   frames=90, drift=2.0, pulls=4, hand=0.4,
                                   roll=0.0, edge=30.0))


def A_state_set(msg):
    st = getattr(A, "_LAST_STATE", None)
    if st is not None: st["install"] = msg

sess = A.CaptureSession(plan=A.make_plan(2, 2))
src = A.SimSource(sess, blob_sigma=0.3)
src.start()
os.makedirs('/tmp/rv', exist_ok=True)
errors = []

def driver():
    time.sleep(1.5)
    q = src._quad(0.5, 0.5, 1.2)
    scenes = []
    # every draw branch, including the ones that need specific state
    for name, setup in [
        ("aim, unarmed",       lambda: (setattr(sess,'state',sess.S_AIM), setattr(sess,'armed',False), setattr(sess,'auto',True))),
        ("aim, armed+dwelling",lambda: (setattr(sess,'armed',True), setattr(sess,'dwell',0.5))),
        ("aim, trigger mode",  lambda: (setattr(sess,'auto',False), setattr(sess,'dwell',0.0))),
        ("aim, pull counter",  lambda: sess.pulls.append(dict(q=q,sigma=0.3,span=70.0,frames=80,drift=2.0))),
        ("bottom target",      lambda: setattr(sess,'idx',2)),
        # the TOO CLOSE branch: an LED pushed hard against the frame edge
        ("too close warning",  lambda: setattr(sess,'live_q',
                                   __import__('numpy').array([[1.,1.],[60.,2.],[2.,120.],[61.,121.]]))),
        ("back in range",      lambda: setattr(sess,'live_q',q)),
        ("capturing",          lambda: (setattr(sess,'state',sess.S_CAPTURING), setattr(sess,'t0',sess.live_t))),
        ("review/rejected",    lambda: (setattr(sess,'state',sess.S_REVIEW), setattr(sess,'msg','test rejection'))),
        ("step back",          lambda: (setattr(sess,'state',sess.S_STEPBACK), setattr(sess,'stance',1))),
        # the rolled-stance screen: four distinct branches, all new code
        ("tilt, no reference", lambda: (setattr(sess,'stance',2), setattr(sess,'roll_ref',None),
                                        sess.shots.clear())),
        ("tilt, wrong way",    lambda: setattr(sess,'roll_ref',
                                        A.aim_fit.quad_roll_sin(sess.live_q) + 0.30)),
        ("tilt, partial",      lambda: setattr(sess,'roll_ref',
                                        A.aim_fit.quad_roll_sin(sess.live_q) - 0.17)),
        ("tilt, good",         lambda: setattr(sess,'roll_ref',
                                        A.aim_fit.quad_roll_sin(sess.live_q) - 0.25)),
        ("tilt, other side",   lambda: (setattr(sess,'stance',3), setattr(sess,'roll_ref',
                                        A.aim_fit.quad_roll_sin(sess.live_q) + 0.25))),
        # a fit needs shots, or every "done" scene renders the FAILED branch and
        # the install status is never drawn
        ("fill shots",         lambda: fill_shots()),
        ("done/results",       lambda: setattr(sess,'state',sess.S_DONE)),
        # both install outcomes
        ("done: installed",    lambda: A_state_set("INSTALLED and verified on the gun (read back ...)")),
        ("done: not installed",lambda: A_state_set("NO REPLY from the gun in 2.5s -- probably NOT installed.")),
    ]:
        setup()
        time.sleep(0.55)                      # ~34 ticks at 16 ms
        scenes.append(name)
    # ASSERT what the results screen actually says: a verified install must not
    # still tell the user to send the line themselves.
    import tkinter as tk
    root = tk._default_root
    def screen_text():
        out = []
        for w in root.winfo_children():
            if isinstance(w, tk.Canvas):
                for i in w.find_all():
                    if w.type(i) == "text": out.append(str(w.itemcget(i, "text")))
        return " | ".join(out)
    for msg, want, unwanted in (
            ("INSTALLED and verified on the gun (read back ...)",
             "INSTALLED ON THE GUN", "send this line to the gun"),
            ("NO REPLY from the gun in 2.5s -- probably NOT installed.",
             "NOT INSTALLED", None)):
        A_state_set(msg)
        # POLL rather than sleep a fixed amount: under load the tick loop is
        # starved and a short sleep can pass without a single redraw
        txt = ""
        deadline = time.time() + 5.0
        while time.time() < deadline:
            txt = screen_text()
            if want in txt: break
            time.sleep(0.05)
        if want not in txt:
            errors.append("results screen never says %r (install=%r)" % (want, msg[:30]))
        if unwanted and unwanted in txt:
            errors.append("results screen still says %r after a verified install" % unwanted)
    subprocess.run("import -window root /tmp/rv/last.png", shell=True, capture_output=True)
    print("screens drawn: %s" % ", ".join(scenes))
    print("gui render: %s" % ("OK" if not errors else "FAILED -- %s" % errors[0]))
    time.sleep(0.2)
    os._exit(1 if errors else 0)

# any exception inside tick() lands in Tk's report_callbackexception
import tkinter as _tk
_orig = _tk.Tk.report_callback_exception
def catch(self, exc, val, tb):
    errors.append("%s: %s" % (exc.__name__, val))
    _orig(self, exc, val, tb)
_tk.Tk.report_callback_exception = catch

threading.Thread(target=driver, daemon=True).start()
A.run_gui(src, sess, '/tmp/ro')
