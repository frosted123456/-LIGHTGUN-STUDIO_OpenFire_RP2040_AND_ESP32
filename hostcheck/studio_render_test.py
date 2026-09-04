#!/usr/bin/env python3
"""Renders gun_studio far enough to run its tick loop and draw every widget, and
drives the POINTER TOGGLE to assert the '~aimhid=' lines that reach the wire --
the toggle is the only way back from a frozen cursor.
"""
import sys, os, time, threading, subprocess, queue, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import tkinter as _tk
errs = []
_o = _tk.Tk.report_callback_exception
def catch(self, e, v, tb):
    errs.append("%s: %s" % (e.__name__, v)); _o(self, e, v, tb)
_tk.Tk.report_callback_exception = catch

import gun_studio

WIRE = []
CHUNKS = []                 # one entry per write() call, so "same write" is checkable

class FakeSerial:
    def write(self, b):
        CHUNKS.append(b.decode())
        for ln in b.decode().splitlines():
            if ln.strip(): WIRE.append(ln.strip())
    def close(self): pass

class FakeSource:
    def __init__(self, port, baud=115200):
        self.ser = FakeSerial(); self.q = queue.Queue(); self.replies = []
    def start(self): pass
    def close(self): pass

gun_studio.SerialSource = FakeSource
gun_studio.find_gun = lambda *a, **k: "FAKE1"

# Studio writes its captures into tools/calib_out, beside the lens sweeps.
# Pointed somewhere disposable for the test: the same folder in the release
# tree is where a REAL capture off a rig is supposed to land, and a test that
# drops a simulated one in it every run makes the real ones unfindable.
OUT = tempfile.mkdtemp(prefix="studio-test-")
gun_studio.HERE = OUT

# The Link the app builds, so the test can feed the gun's own replies into it.
# Without this the wiicam half of the Camera tab is never PACKED -- it only
# appears once a board tag says wiicam -- so every widget on it went untested.
LINKS = []
_RealLink = gun_studio.Link
class SpyLink(_RealLink):
    def __init__(self):
        _RealLink.__init__(self)
        LINKS.append(self)
gun_studio.Link = SpyLink


def find_notebook(w):
    import tkinter.ttk as ttk
    if isinstance(w, ttk.Notebook):
        return w
    for c in w.winfo_children():
        r = find_notebook(c)
        if r is not None:
            return r
    return None


def by_text(w, want, cls="Radiobutton", out=None):
    """Every widget of class `cls` whose label is exactly one of `want`.

    The radio buttons are the only way to exercise what a click actually puts
    on the wire, and they are built in a loop with no handle kept. The class
    matters: "off" is also the text of the dead-zone state Label, and calling
    invoke() on that is an AttributeError, not a test failure.
    """
    out = [] if out is None else out
    try:
        if w.winfo_class() == cls and str(w.cget("text")) in want:
            out.append(w)
    except Exception:
        pass
    for c in w.winfo_children():
        by_text(c, want, cls, out)
    return out


def spinboxes(w, out=None):
    """Every Spinbox on the tree.

    The shape-gate pair is built in a loop with no handle kept, and they are
    the only two controls here whose steps are a fixed LIST rather than a
    range -- which is their whole point, because the gun refuses the values a
    range would walk through. That has to be driven, not read.
    """
    out = [] if out is None else out
    try:
        if w.winfo_class() == "Spinbox":
            out.append(w)
    except Exception:
        pass
    for c in w.winfo_children():
        spinboxes(c, out)
    return out


def texts(w, out):
    try:
        t = w.cget("text")
        if t:
            out.append(str(t))
    except Exception:
        pass
    for c in w.winfo_children():
        texts(c, out)
    return out


def canvases(w, out=None):
    """Every Canvas on the tree, in build order.

    The shape preview is one, and nothing about it is readable as text: what
    it drew is only visible as canvas items, so the test has to hold the
    widget itself to ask.
    """
    out = [] if out is None else out
    try:
        if w.winfo_class() == "Canvas":
            out.append(w)
    except Exception:
        pass
    for c in w.winfo_children():
        canvases(c, out)
    return out


def shown(w):
    """Is this widget actually on screen right now?

    The Advanced disclosure is a frame that is packed and unpacked, and a
    widget inside an unpacked frame still answers cget() and still accepts
    invoke() -- so "the control exists" proves nothing about whether it is in
    front of the user. Only winfo_ismapped() does.
    """
    try:
        return bool(w.winfo_ismapped())
    except Exception:
        return False


def logbox_text(w):
    """Everything in the scrolling log. The panel is too short to explain
    itself, so Studio sends its reasoning and its warnings to the log -- which
    makes the log the only place a warning can be checked for."""
    try:
        if w.winfo_class() == "Text":
            return w.get("1.0", "end")
    except Exception:
        return ""
    out = ""
    for c in w.winfo_children():
        out += logbox_text(c)
    return out


def driver():
    # Wait for the window to BE THERE rather than for a fixed three seconds.
    # The app builds every widget before it reaches mainloop(), so the
    # notebook existing is the signal; and this file is now long enough that
    # three seconds of certainty at the top costs more of the runner's
    # timeout than it buys. Still bounded, and still says so if nothing came.
    for _ in range(120):
        root = _tk._default_root
        if root is not None and find_notebook(root) is not None:
            break
        time.sleep(0.05)
    time.sleep(0.3)                        # let the first tick round run
    root = _tk._default_root
    if root is None or find_notebook(root) is None:
        errs.append("no Tk root"); os._exit(1)

    def fire(seq):
        root.event_generate(seq); root.update()

    # Nothing reaches the wire until the tick drains Link's outgoing queue,
    # one line per tick with a hold after each -- so every "what did that
    # click send" check below waits for the queue to empty first.
    def drained(link, secs=4.0):
        t_end = time.time() + secs
        while time.time() < t_end:
            root.update()
            if link.pending() == 0:
                return True
            time.sleep(0.02)
        return False

    # ---- R8: connect must turn the resolver back on ------------------------
    # A prior session that died mid lens-sweep leaves res:0 on the gun; only a
    # fresh connect can put it right again.
    recon_btn = by_text(root, ("Reconnect",), cls="Button")
    if not recon_btn:
        errs.append("no Reconnect button found")
    else:
        WIRE.clear()
        recon_btn[0].invoke()
        # The connect burst is 8 lines and '~camlearn?' waits up to 0.6 s for
        # an answer this fake never gives, so this is the slowest drain here.
        if not drained(LINKS[0], 6.0):
            errs.append("the connect burst never finished draining: %s"
                        % LINKS[0].pending())
        if "~cam=res:2" not in WIRE:
            errs.append("connect did not turn the resolver back on (R8): %s" % WIRE)
        # ...and the resolver line went out through the ONE queue, not as a
        # straight write racing it.
        if LINKS[0].wrote < 8:
            errs.append("the connect burst did not go through Link's queue "
                        "(wrote=%d)" % LINKS[0].wrote)

    WIRE.clear()
    fire("<F9>"); drained(LINKS[0])        # freeze
    fire("<F9>"); drained(LINKS[0])        # release
    fire("<F9>"); drained(LINKS[0])        # freeze again, and LEAVE it frozen
    hid = [w for w in WIRE if w.startswith("~aimhid=")]
    if hid != ["~aimhid=0", "~aimhid=1", "~aimhid=0"]:
        errs.append("F9 did not toggle cleanly: %s" % hid)

    # ---- the live view says which corners are real (C1 C2 C3) ---------------
    # The wiicam's 14-field Q line names the corners the gun filled in and the
    # lead it added; the view must draw those hollow and the lead as an arrow,
    # and go on drawing four filled corners off the OV gun's 10-field line.
    live_cv = [c for c in canvases(root) if int(c.cget("width")) == 380]
    if not live_cv:
        errs.append("no 380-px live view canvas found")
    else:
        live_cv = live_cv[0]
        L0 = LINKS[0]

        def view_after(line):
            L0.hist = []
            L0.src.q.put(line + "\n")
            t_end = time.time() + 1.0
            while time.time() < t_end:
                root.update(); time.sleep(0.03)
                if L0.hist and live_cv.find_withtag("corner"):
                    break
            root.update(); time.sleep(0.12); root.update()
            return (len(live_cv.find_withtag("corner_real")),
                    len(live_cv.find_withtag("corner_hollow")),
                    len(live_cv.find_withtag("lead")),
                    len(live_cv.find_withtag("legend")))

        QUAD = "1000,600,1800,610,1010,1300,1810,1310"
        got = view_after("Q,5000,4,%s,c,11,30,0" % QUAD)
        if got[:3] != (3, 1, 1):
            errs.append("kind c, real=11, lead 30: drew %d filled, %d hollow, "
                        "%d arrow (want 3, 1, 1)" % got[:3])
        if got[3] != 1:
            errs.append("no legend line under a view with a hollow corner")
        if L0.qmeta is None or L0.qmeta.get("kind") != "c" \
                or L0.qmeta.get("real") != 11:
            errs.append("Link.qmeta did not carry the Q line's flags: %r" % (L0.qmeta,))
        # The quad the sigma and the stats use is the one the cursor came
        # from: pre-lead corners plus the lead.
        if L0.hist and abs(L0.hist[-1][1][0][0] - 103.0) > 1e-6:
            errs.append("parse_q did not add the lead back: %r" % (L0.hist[-1][1][0],))
        # The hollow dot follows its corner through the TL,TR,BL,BR sort:
        # BR sent first on the wire, real clearing slot 0.
        PERM = "1810,1310,1000,600,1800,610,1010,1300"
        got = view_after("Q,5500,4,%s,c,14,0,0" % PERM)
        hollow = live_cv.find_withtag("corner_hollow")
        hx = [live_cv.coords(i) for i in hollow]
        if got[:2] != (3, 1) or not hx \
                or not (hx[0][0] > 380 * 0.6 and hx[0][1] > 280 * 0.6):
            errs.append("with BR sent first the hollow dot did not land at BR: "
                        "%r %r" % (got, hx))
        got = view_after("Q,6000,4,%s,c,15,0,0" % QUAD)
        if got != (4, 0, 0, 0):
            errs.append("kind c, all real, no lead: drew %r (want 4 filled, "
                        "nothing else)" % (got,))
        got = view_after("Q,7000,4,%s" % QUAD)
        if got != (4, 0, 0, 0):
            errs.append("the OV gun's 10-field line drew %r (want 4 filled, "
                        "no arrow)" % (got,))
        L0.hist = []; L0.qmeta = None
        # parse_q_ex on the old form: every slot counted as measured, no lead;
        # a padded n=2 line gives the mask the -1,-1 pads imply.
        import aim_calib
        m = aim_calib.parse_q_ex("Q,7000,4,%s" % QUAD)
        if m is None or (m["kind"], m["real"], m["lead"]) != (None, 15, (0.0, 0.0)):
            errs.append("parse_q_ex on a 10-field line: %r" % (m,))
        m = aim_calib.parse_q_ex("Q,7000,2,1000,600,-1,-1,1010,1300,-1,-1")
        if m is None or m["q"] is not None or m["real"] != 5 or m["n"] != 2:
            errs.append("parse_q_ex on a padded n=2 line: %r" % (m,))

    # The "remember" semantics: a temporary release must not erase the user's
    # choice, or returning from calibration would silently un-freeze them.
    # A Link nobody ticks: flush() writes what it queued, in order.
    L = gun_studio.Link(); L.src = FakeSource("X")
    WIRE.clear()
    L.pointer(False)                       # user freezes
    L.pointer(True, remember=False)        # calibration borrows the cursor
    if not L.hid_on is False:
        errs.append("a temporary release overwrote the user's choice")
    L.pointer(L.hid_on)                    # ...and we come back
    L.flush()
    # remember=False is app-side only: Studio must come BACK to the user's choice
    if WIRE != ["~aimhid=0", "~aimhid=1", "~aimhid=0"]:
        errs.append("remember=False sequence wrong on the wire: %s" % WIRE)

    # ---- the outgoing queue itself (L1) --------------------------------------
    # One wire, many askers. Driven on a clock the test owns, so the holds and
    # the '~camlearn?' wait can be checked to the millisecond.
    Q = gun_studio.Link(); Q.src = FakeSource("Q")
    qclock = {"t": 100.0}
    Q.clock = lambda: qclock["t"]
    WIRE.clear()
    Q.send("~cam=bhmax:8")
    Q.pump()
    if WIRE != ["~cam=bhmax:8"]:
        errs.append("a queued line was not written on the first pump: %s" % WIRE)
    Q.send("~cam=bhmax:10")
    qclock["t"] += 0.030; Q.pump()
    if len(WIRE) != 1:
        errs.append("the 40 ms hold after a write was not honoured: %s" % WIRE)
    qclock["t"] += 0.015; Q.pump()
    if WIRE != ["~cam=bhmax:8", "~cam=bhmax:10"]:
        errs.append("the next line did not go out once the hold passed: %s" % WIRE)
    # '~camlearn?' holds the wire until the whole histogram set is in -- or
    # 600 ms have passed on a gun that never answers.
    qclock["t"] += 1.0; WIRE.clear()
    Q.send("~camlearn?"); Q.pump()
    Q.send("~camloop?")
    qclock["t"] += 0.3; Q.pump()
    if WIRE != ["~camlearn?"]:
        errs.append("a line went out while '~camlearn?' was still awaiting "
                    "its answer: %s" % WIRE)
    # Two polls of the same question while it waits: the first is queued,
    # the second is skipped and counted, never piled up behind the first.
    skipped0 = Q.polls_skipped
    a = Q.send("~camblob?", poll=True)
    b = Q.send("~camblob?", poll=True)
    if not (a is True and b is False and Q.polls_skipped == skipped0 + 1):
        errs.append("a repeated poll was not skipped while the wire waited: "
                    "%s %s skipped=%d" % (a, b, Q.polls_skipped - skipped0))
    if Q.pending() != 2:
        errs.append("the queue holds %d lines, expected the loop poll and one "
                    "blob poll" % Q.pending())
    # The learn answer arrives whole: the wait ends and the queue moves on.
    for ln in ["CAM: learn on=1 frames=10 led=40 rej=1 bins=32\n"] + [
            "CAM: hist c=%d f=%s %s\n" % (c, f, " ".join(["0"] * 32))
            for c in (0, 1) for f in gun_studio.HIST_FEATS]:
        Q.src.q.put(ln)
    qclock["t"] += 0.05; Q.pump()          # reads the set; the wait ends
    qclock["t"] += 0.05; Q.pump()
    if WIRE[:2] != ["~camlearn?", "~camloop?"]:
        errs.append("the queue did not resume once the histogram set landed: "
                    "%s" % WIRE)
    # A poll arriving inside a hold is DELAYED, not dropped: the hold only
    # paces the wire. Only a second copy of a waiting poll is dropped.
    skipped0 = Q.polls_skipped
    if Q.send("~fx?", poll=True) is not True or Q.polls_skipped != skipped0:
        errs.append("a poll sent inside the hold after a write was dropped")
    if Q.send("~fx?", poll=True) is not False or Q.polls_skipped != skipped0 + 1:
        errs.append("a second copy of a waiting poll was queued")
    # A user command goes ahead of the waiting polls, never ahead of another
    # user command.
    while Q.pending():
        qclock["t"] += 0.1; Q.pump()
    WIRE.clear()
    Q.send("~camloop?", poll=True); Q.send("~camfit?", poll=True)
    Q.send("~cam=bhmax:12"); Q.send("~cam=bhmax:16")
    for _ in range(4):
        qclock["t"] += 0.1; Q.pump()
    if WIRE != ["~cam=bhmax:12", "~cam=bhmax:16", "~camloop?", "~camfit?"]:
        errs.append("user commands did not go ahead of the waiting polls, in "
                    "their own order: %s" % WIRE)
    # The reader thread's stream re-arm comes through the queue as a poll.
    WIRE.clear()
    Q.src.want_dash = True
    qclock["t"] += 0.1; Q.pump()
    if WIRE != ["~cam=dash:2", "~aimcap=1"] or Q.src.want_dash:
        errs.append("the reader's re-arm request was not sent through the "
                    "queue: %s" % WIRE)
    # The wait also ends on its own after 600 ms on a gun that never answers.
    qclock["t"] += 2.0; WIRE.clear()
    while Q.pending():
        qclock["t"] += 0.1; Q.pump()
    Q.send("~camlearn?"); qclock["t"] += 0.1; Q.pump()
    Q.send("~ping")
    qclock["t"] += 0.5; Q.pump()
    if "~ping" in WIRE:
        errs.append("the '~camlearn?' wait ended before its 600 ms timeout")
    qclock["t"] += 0.2; Q.pump()
    if "~ping" not in WIRE:
        errs.append("the '~camlearn?' wait did not time out at 600 ms: %s" % WIRE)
    # The long holds: a flash write keeps the wire for 300 ms, a diag 1.5 s.
    for cmd, hold in (("~camsave", 0.3), ("~camdiag", 1.5), ("~cam=sens:2", 0.3)):
        qclock["t"] += 3.0; WIRE.clear()
        Q.send(cmd); Q.pump(); Q.send("~ping")
        qclock["t"] += hold - 0.01; Q.pump()
        early = "~ping" in WIRE
        qclock["t"] += 0.02; Q.pump()
        if early or "~ping" not in WIRE:
            errs.append("%s did not hold the wire for %.1f s: %s" % (cmd, hold, WIRE))
    # A user command is never dropped, only queued behind the hold.
    qclock["t"] += 3.0
    if Q.send("~camdiag") is not True or Q.send("~cam=bhmax:8") is not True:
        errs.append("a user command was refused by the queue")
    while Q.pending():
        qclock["t"] += 0.1; Q.pump()
    if gun_studio.Link().hid_on is not True:
        errs.append("Link must start with the pointer ON -- the gun boots that way")

    # Reconnecting must forget the previous gun. Keeping its replies meant a
    # reconnect onto a different or reflashed gun answered questions about it
    # from the old one's state -- including "does this firmware have quiet
    # mode", which decides whether a calibration runs with a live solenoid.
    L2 = gun_studio.Link()
    L2.last["fxquiet"] = 0
    L2.last["board"] = "rp2040-wiicam"
    L2.blobs = "CAM: blobs 1,2,3,1"
    L2.connect("FAKE1")
    if L2.last or L2.blobs:
        errs.append("connect kept state from the previous gun: %r / %r"
                    % (L2.last, L2.blobs))

    # ---- '~camfit', parsed off the wire ------------------------------------
    # The gun's answer to "what height gate can THIS rig have" is several
    # lines, and ANY of them can be absent: three of the four outcomes are a
    # single sentence, the stored-provenance line only appears when the gun
    # has one, the 'applied' line only follows '=apply', and an older gun says
    # nothing at all. Every outcome is walked here rather than through the
    # window, because a parser fault seen through a GUI is a screenshot and
    # seen here is one line -- and because the panel's honesty rests entirely
    # on this: a verdict read wrong is a height nobody measured, offered as
    # though the gun had measured it.
    def fit_of(*lines):
        """A fresh CamFit fed these lines in order."""
        f = gun_studio.CamFit()
        for ln in lines:
            f.feed(ln)
        return f

    HDR = "CAM: fit ledn=900 ledmaxh=7 straym=40 strayminh=15"
    f = fit_of(HDR, "CAM: fit bhmax=11 (LEDs reach 7, stray starts at 15)",
               "CAM: fit not applied -- send camfit=apply to set and save it")
    if (f.verdict, f.bhmax, f.tight, f.applied) != ("ok", 11, False, None):
        errs.append("a plain verdict parsed as %r"
                    % ((f.verdict, f.bhmax, f.tight, f.applied),))
    if (f.led_n, f.led_max_h, f.stray_n, f.stray_min_h) != (900, 7, 40, 15):
        errs.append("the header counts did not reach the answer: %r"
                    % ((f.led_n, f.led_max_h, f.stray_n, f.stray_min_h),))
    if f.seq != 1:
        errs.append("an answer did not bump seq, so a front end cannot tell a "
                    "fresh reply from the one already held: %r" % f.seq)
    # One row between the LEDs and the stray light is a gate that works today
    # and stops working when the lamp warms up. The gun says TIGHT; dropping
    # that word would present a knife-edge as a comfortable margin. The
    # ceiling in that case is the LED maximum ITSELF -- the gate keeps
    # h <= bhmax, so led_max_h + 1 would land on the stray height and reject
    # nothing at all, a "TIGHT" gate that is really a no-op.
    f = fit_of(HDR, "CAM: fit bhmax=7 (LEDs reach 7, stray starts at 8 -- "
                    "TIGHT, only one step between them)",
               "CAM: fit applied and saved")
    if (f.bhmax, f.tight, f.applied) != (7, True, True):
        errs.append("a TIGHT verdict applied and saved parsed as %r"
                    % ((f.bhmax, f.tight, f.applied),))
    # A save that failed must never read as one that worked: the gate is live
    # either way, and the difference only shows at the next power cycle.
    f = fit_of(HDR, "CAM: fit bhmax=11 (LEDs reach 7, stray starts at 15)",
               "CAM: fit applied but SAVE FAILED -- it will be gone on the "
               "next power cycle")
    if f.applied is not False:
        errs.append("a failed save parsed as %r, which the panel would report "
                    "as success" % f.applied)
    # Not enough LED data, with the provenance line the gun sends in front of
    # it. The stored pair is what its refusals are measured against, so losing
    # it loses the only explanation for "why was my setting refused?".
    f = fit_of("CAM: fit ledn=120 ledmaxh=-1 straym=0 strayminh=-1",
               "CAM: fit STORED ledmaxh=7 strayminh=15 ledmaxpx=214 -- "
               "from an earlier capture on this gun; recapture if the bar, "
               "the lens or the room has changed",
               "CAM: fit NEEDS MORE LED DATA -- aim at the bar with the "
               "capture on; 120 blobs so far, 500 wanted")
    if (f.verdict, f.led_n, f.led_want, f.stored) != ("need_led", 120, 500,
                                                      (7, 15, 214)):
        errs.append("'needs more LED data' parsed as %r"
                    % ((f.verdict, f.led_n, f.led_want, f.stored),))
    # The pixel figure was added to the END of the STORED line, so a gun on
    # either firmware sends a different number of fields. The older pair still
    # has to land, with the field it never sent reading None -- 0 px is a
    # measurement, and the pxmax floor is derived from this one.
    f2 = fit_of("CAM: fit ledn=120 ledmaxh=-1 straym=0 strayminh=-1",
                "CAM: fit STORED ledmaxh=7 strayminh=15 -- from an earlier "
                "capture on this gun; recapture if the bar, the lens or the "
                "room has changed",
                "CAM: fit NEEDS MORE LED DATA -- aim at the bar with the "
                "capture on; 120 blobs so far, 500 wanted")
    if f2.stored != (7, 15, None):
        errs.append("a two-field STORED line from an older gun parsed as %r"
                    % (f2.stored,))
    if f.bhmax is not None:
        errs.append("an unfinished measurement still held a height to apply: "
                    "%r" % f.bhmax)
    # The targets come off the wire, not out of this file: a firmware that
    # wants more data would otherwise be reported as "600 of 500", which is
    # not a progress figure, it is a bug on screen.
    f = fit_of("CAM: fit ledn=600 ledmaxh=-1 straym=0 strayminh=-1",
               "CAM: fit NEEDS MORE LED DATA -- aim at the bar with the "
               "capture on; 600 blobs so far, 1000 wanted")
    if f.led_want != 1000:
        errs.append("the target was pinned in tools/ instead of read off the "
                    "gun's own sentence: %r" % f.led_want)
    f = fit_of("CAM: fit ledn=900 ledmaxh=7 straym=3 strayminh=-1",
               "CAM: fit NO STRAY DATA -- sweep the room with the screen in "
               "view so a lamp or window enters frame; 3 seen, 20 wanted. "
               "Your LEDs measured 7 tall.")
    if (f.verdict, f.stray_n, f.stray_want, f.led_max_h) != ("need_stray", 3,
                                                             20, 7):
        errs.append("'no stray data' parsed as %r"
                    % ((f.verdict, f.stray_n, f.stray_want, f.led_max_h),))
    # The one answer that must not produce a number. This rig's LEDs are as
    # tall as its stray light, so a size gate cannot separate them at all --
    # and a held ceiling from the previous ask would be offered for a rig the
    # gun has just finished explaining cannot have one.
    f = fit_of(HDR, "CAM: fit bhmax=11 (LEDs reach 7, stray starts at 15)",
               "CAM: fit ledn=900 ledmaxh=18 straym=40 strayminh=15",
               "CAM: fit NO SAFE GATE -- your LEDs reach 18 tall and the "
               "stray light starts at 15, so a size gate cannot tell them "
               "apart. Move the bar, block the light, or use brighter LEDs.")
    if (f.verdict, f.bhmax, f.led_max_h, f.stray_min_h) != ("no_gate", None,
                                                            18, 15):
        errs.append("'no safe gate' parsed as %r -- a rig that cannot be "
                    "gated must not come with a number"
                    % ((f.verdict, f.bhmax, f.led_max_h, f.stray_min_h),))
    if f.seq != 2:
        errs.append("a second ask did not count as a second answer: %r"
                    % f.seq)
    # Nothing here may raise, and nothing may leave a field looking measured
    # when it was not. These are the lines a gun sends when its send buffer
    # cut the reply short, or when a build says something this parsing has
    # never seen -- both of which reach a user before they reach a test.
    for junk in ("CAM: fit",
                 "CAM: fit ledn=x ledmaxh= straym= strayminh=",
                 "CAM: fit bhmax=",
                 "CAM: fit bhmax=(LEDs",
                 "CAM: fit NEEDS MORE LED DATA -- run a cali",
                 "CAM: fit SOMETHING THIS BUILD INVENTED"):
        try:
            g = fit_of(junk)
        except Exception as e:
            errs.append("a truncated fit line raised %r: %r" % (e, junk))
            continue
        if g.bhmax is not None:
            errs.append("%r left a height to apply: %r" % (junk, g.bhmax))
    g = fit_of("CAM: fit ledn=x ledmaxh= straym= strayminh=")
    if (g.led_n, g.led_max_h) != (None, None):
        errs.append("unparseable counts became numbers: %r"
                    % ((g.led_n, g.led_max_h),))
    g = fit_of("CAM: fit ledn=120 ledmaxh=-1 straym=0 strayminh=-1",
               "CAM: fit NEEDS MORE LED DATA -- run a cali")
    if (g.verdict, g.led_want) != ("need_led", 500):
        errs.append("a sentence cut off before its target fell back to "
                    "nothing usable: %r" % ((g.verdict, g.led_want),))
    if gun_studio.CamFit().feed("CAM: blobs 1,2,3,1"):
        errs.append("CamFit claimed a line that was not a fit reply")

    # ---- a verdict is a PROPOSAL, not a setting ----------------------------
    # NAMED, because a later tidy-up of pump() is exactly what brings this
    # back. 'CAM: fit bhmax=11 (LEDs reach 7...)' is the gun describing a
    # height it has NOT set and will not set until it is asked again with
    # '=apply'. pump()'s generic key/value sweep reads every 'k=v' token on a
    # CAM: line, so with the fit lines left in its path that 11 lands in
    # last["bhmax"]: the height spinbox follows last[] within a tick, snaps to
    # a gate nobody turned on, and the next click on that box sends it to the
    # gun. The lines still have to reach the log -- they are the plainest
    # English on the wire and the log is where the long form belongs.
    L3 = gun_studio.Link()
    L3.src = FakeSource("X")
    L3.last["bhmax"] = 0
    for ln in ("CAM: blob bhmax=0 bn=4 bframes=10",
               "CAM: fit ledn=900 ledmaxh=7 straym=40 strayminh=15",
               "CAM: fit bhmax=11 (LEDs reach 7, stray starts at 15)"):
        L3.src.q.put(ln)
    L3.pump()
    if L3.last.get("bhmax") != 0:
        errs.append("a fit VERDICT became the gun's live setting "
                    "(last[bhmax]=%r): the spinbox would snap to a gate "
                    "nobody turned on and the next click would send it"
                    % L3.last.get("bhmax"))
    if L3.fit.bhmax != 11:
        errs.append("the verdict was swallowed instead of held: %r"
                    % L3.fit.bhmax)
    if not any("bhmax=11" in r for r in L3.replies):
        errs.append("the fit lines never reached the log, which is the only "
                    "place the gun's own wording is shown: %r" % L3.replies)

    # ---- the hwmax loop, off the wire and into words ----------------------
    # The one control that acts BEFORE the sensor hands out its four slots.
    # Both reply forms are walked here rather than through the window for the
    # same reason the fit is: a parser fault seen through a GUI is a
    # screenshot, and the whole row's honesty rests on this. Two shapes have
    # to land -- the five keys 'cam?' now ends with, and the whole '~camloop?'
    # line -- and NEITHER can be read by the generic key/value sweep: 'hws' is
    # a word, 'state' is a word, and 'dwell=3/50' is not a number at all.
    L4 = gun_studio.Link()
    L4.src = FakeSource("X")
    L4.src.q.put("CAM: board=rp2040-wiicam sens=2 ext=1 fmt=2 bmin=2 bmax=9 "
                 "rtol=3 bhmax=0 pxmax=0 armax=0 hwmax=-1 hwmin=-1 "
                 "loop=1 hwv=63 hwlo=48 hwhi=80 hws=HOLD\n")
    L4.pump()
    if [L4.last.get(k) for k in ("loop", "hwv", "hwlo", "hwhi", "hws")] \
            != [1, 63, 48, 80, "HOLD"]:
        errs.append("the five keys 'cam?' now ends with did not all land: %r"
                    % {k: L4.last.get(k) for k in ("loop", "hwv", "hwlo",
                                                   "hwhi", "hws")})
    # hws is the one that would be lost silently: it is a WORD on a line whose
    # every other key is a number, so the int() sweep swallows it and the
    # field that says what the gun is DOING never reaches last[] at all.
    if L4.last.get("hws") != "HOLD":
        errs.append("the loop's state word was dropped by the int-only "
                    "sweep -- the row would read '?' on a working gun")
    # A word this parsing does not know is a different generation of it, and
    # must not be passed through to a panel as though it were a state.
    L4.src.q.put("CAM: loop=1 hws=SOMETHINGNEW\n")
    L4.pump()
    if L4.last.get("hws") != "HOLD":
        errs.append("a state word this build has never seen replaced the one "
                    "it knew: %r" % L4.last.get("hws"))
    # The whole '~camloop?' line.
    L4.src.q.put("CAM: loop on=1 state=RAISE val=70 lo=63 hi=256 dwell=3/50 "
                 "clean=41 stray=0 cut=2 settled=0 saved=0\n")
    L4.pump()
    want = {"on": 1, "state": "RAISE", "val": 70, "lo": 63, "hi": 256,
            "dwell": 3, "dwelln": 50, "clean": 41, "stray": 0, "cut": 2,
            "settled": 0, "saved": 0}
    if L4.loop != want:
        errs.append("the '~camloop?' line parsed as %r" % (L4.loop,))
    # It must NOT reach the log: it is polled once a second, and Studio's box
    # is six lines -- six seconds and the save result the user was told to
    # watch for is gone.
    if any("CAM: loop on=" in r for r in L4.replies):
        errs.append("the loop poll reached the log, which would empty it "
                    "every six seconds: %r" % L4.replies[-3:])
    # And its lo/hi are the bracket the loop is SEARCHING in, which are not
    # the size window's. Landing them in last[] beside bmin/bmax is how a
    # spinbox ends up showing a number nobody set.
    if L4.last.get("lo") is not None or L4.last.get("hi") is not None:
        errs.append("the loop's own bracket leaked into the key store: "
                    "lo=%r hi=%r" % (L4.last.get("lo"), L4.last.get("hi")))
    # A reconnect is a different gun until it says otherwise.
    L4.connect("FAKE1")
    if L4.loop:
        errs.append("connect kept the previous gun's loop: %r" % L4.loop)

    # ---- the five states, in words ----------------------------------------
    # Every one of them, because each says something a user has to act on
    # differently -- and because the two that are NOT a running search
    # (NOSAFE, OFF) are the two most likely to be read as a fault.
    def loop_says(last, loop=None):
        return gun_studio.loop_line(last, loop)[0]

    LOOPBASE = {"on": 1, "val": 63, "lo": 48, "hi": 80, "dwell": 10,
                "dwelln": 50, "clean": 44, "stray": 1, "cut": 0,
                "settled": 1, "saved": 1}
    # OFF gives no reason: a hand-set sensor max and the tool's own 'loop:0'
    # both get there, and the readout cannot tell which.
    cases = [("HOLD", ("holding at 63", "clean 44 stray 1 cut 0", "saved")),
             ("LOWER", ("searching down", "48..80")),
             ("RAISE", ("raising", "LED cut")),
             ("NOSAFE", ("NO SAFE LIMIT", "tell your LEDs from the room light")),
             ("OFF", ("off - limit 63",))]
    for st, wants in cases:
        d = dict(LOOPBASE, state=st)
        if st == "OFF":
            d["on"] = 0
        txt = loop_says({}, d)
        for w in wants:
            if w not in txt:
                errs.append("the %s state does not say %r: %r" % (st, w, txt))
        if "None" in txt:
            errs.append("a 'None' reached the loop row on %s: %r" % (st, txt))
        if "set by hand" in txt:
            errs.append("the %s state claims a reason the gun never sent: %r"
                        % (st, txt))
    # 'saved' is a WORD, both ways round: 'saved=0' on a limit the loop is
    # still hunting is not a failure and must not read as one.
    if "not saved yet" not in loop_says({}, dict(LOOPBASE, state="LOWER",
                                                 saved=0)):
        errs.append("a limit the loop has not saved yet did not say so: %r"
                    % loop_says({}, dict(LOOPBASE, state="LOWER", saved=0)))
    # 256 is the firmware SAYING it has no ceiling yet -- one past the top of
    # the byte it searches in. Printed raw it is a limit nobody set.
    if "48..?" not in loop_says({}, dict(LOOPBASE, state="LOWER", hi=256)):
        errs.append("hwhi=256 was shown as a limit rather than as unknown: %r"
                    % loop_says({}, dict(LOOPBASE, state="LOWER", hi=256)))
    # An OLDER GUN. It sends none of the five keys and never answers
    # '~camloop?', and that must degrade to a question mark -- never blank,
    # never 'None', and never an exception on a screen with no console.
    old = loop_says({"bhmax": 0, "bn": 4, "bframes": 900})
    if "?" not in old or "None" in old or not old.strip():
        errs.append("an older gun's silence did not read as '?': %r" % old)
    for junk in ({}, {"loop": None}, {"hws": "HOLD"}, {"loop": 1},
                 {"loop": 1, "hws": "LOWER", "hwhi": 256}):
        try:
            t, kind = gun_studio.loop_line(junk)
        except Exception as e:
            errs.append("loop_line raised on %r: %r" % (junk, e))
            continue
        if not t or "None" in t or kind not in ("ok", "warn", "bad", "dim"):
            errs.append("loop_line gave %r / %r for %r" % (t, kind, junk))
    # A truncated or invented '~camloop?' line must cost the field, not the
    # screen -- and must not half-replace the answer that was there.
    L5 = gun_studio.Link()
    L5.src = FakeSource("X")
    L5.src.q.put("CAM: loop on=1 state=HOLD val=63 lo=48 hi=80 dwell=10/50 "
                 "clean=44 stray=1 cut=0 settled=1 saved=1\n")
    L5.pump()
    held = dict(L5.loop)
    for junk in ("CAM: loop ", "CAM: loop state=HOLD val=x dwell=/",
                 "CAM: loop val=9 clean=1"):
        L5.src.q.put(junk + "\n")
        L5.pump()
    if L5.loop != held:
        errs.append("a headless or truncated loop line replaced the answer "
                    "that was held: %r" % (L5.loop,))

    # ---- the blob log's two new columns -----------------------------------
    # COLS and sample() are two lists that have to stay the same length for
    # ever, and nothing else checks them: the file is read months later, in a
    # spreadsheet, by column index. A row one field short silently shifts
    # every column after it.
    import csv as _csv
    _p = os.path.join(OUT, "loopcols.csv")
    _bl = gun_studio.BlobLog(_p)
    if gun_studio.BlobLog.COLS[-2:] != ("loopv", "loops"):
        errs.append("the loop columns are not the last two, so every capture "
                    "already on a stick reads shifted: %r"
                    % (gun_studio.BlobLog.COLS[-4:],))
    _bl.sample({"bframes": 1, "hwv": 63, "hws": "HOLD"},
               "CAM: blobs 1,2,3,1", 100.0)
    # ...and the row a gun too old to send them writes: BLANK, not 0 and not
    # 'OFF'. A 0 in loopv is a sensor limit of zero, which is a gun that sees
    # nothing, and 'OFF' is a claim about a loop that is not there.
    _bl.sample({"bframes": 2}, "CAM: blobs 1,2,3,1", 100.0)
    _bl.close()
    with open(_p) as _fh:
        _rows = list(_csv.reader(_fh))
    if len(_rows) != 3 or any(len(r) != len(gun_studio.BlobLog.COLS)
                              for r in _rows):
        errs.append("COLS and sample() are out of step: header %d, rows %s"
                    % (len(gun_studio.BlobLog.COLS),
                       [len(r) for r in _rows[1:]]))
    elif _rows[1][-2:] != ["63", "HOLD"] or _rows[2][-2:] != ["", ""]:
        errs.append("the loop columns wrote %r / %r"
                    % (_rows[1][-2:], _rows[2][-2:]))
    # ...and they FOLLOW the loop during a capture. sample() reads hwv/hws
    # from last[], which 'cam?' fills only on connect and "Read from gun"; the
    # once-a-second '~camloop?' answer has to land there too, or every row of
    # a capture carries the limit the loop had when the app was opened.
    _p2 = os.path.join(OUT, "loopfollow.csv")
    L6 = gun_studio.Link()
    L6.src = FakeSource("X")
    L6.src.q.put("CAM: board=rp2040-wiicam loop=1 hwv=63 hwlo=48 hwhi=80 "
                 "hws=HOLD bframes=10\n")
    L6.pump()
    _bl = gun_studio.BlobLog(_p2)
    _bl.sample(L6.last, "CAM: blobs 1,2,3,1", 100.0)
    L6.src.q.put("CAM: loop on=1 state=LOWER val=40 lo=30 hi=63 dwell=3/50 "
                 "clean=20 stray=4 cut=0 settled=0 saved=0\n")
    L6.pump()
    L6.last["bframes"] = 11
    _bl.sample(L6.last, "CAM: blobs 1,2,3,1", 100.0)
    _bl.close()
    with open(_p2) as _fh:
        _rows = list(_csv.reader(_fh))
    if len(_rows) != 3 or _rows[1][-2:] != ["63", "HOLD"] \
            or _rows[2][-2:] != ["40", "LOWER"]:
        errs.append("loopv/loops did not follow the '~camloop?' answer: %r"
                    % [r[-2:] for r in _rows[1:]])
    if [L6.last.get(k) for k in ("loop", "hwlo", "hwhi")] != [1, 30, 63]:
        errs.append("the '~camloop?' answer did not refresh last[]: %r"
                    % {k: L6.last.get(k) for k in ("loop", "hwlo", "hwhi")})

    # ---- the wiicam camera tab, ambient-light half ------------------------
    # Feed the gun's own replies so the wiicam frame packs and the blob
    # readout runs its parsing on a real line rather than on nothing.
    if LINKS:
        link = LINKS[0]
        nb = find_notebook(root)
        # The replies as THIS firmware sends them: fmt and fullreg beside the
        # older ext, and every counter the panel reads. The line fed here used
        # to predate all three, so the controls that depend on them -- the
        # detail radio, the full-mode register radio, the give-back warning --
        # were exercised against a gun that could not have driven them.
        for ln in (
            "AIM: pong board=rp2040-wiicam\n",
            "CAM: board=rp2040-wiicam sens=1 lens=0 res=2 dash=0 ext=1 fmt=1 "
            "fullreg=5 bmin=2 bmax=9 rtol=3 bhmax=0 pxmax=0 armax=0 hwmax=-1 "
            "hwmin=-1\n",
            "CAM: blob fmt=1 ext=1 fullreg=5 bmin=2 bmax=9 rtol=3 bhmax=0 "
            "pxmax=0 "
            "armax=0 hwmax=-1 "
            "hwmin=-1 bn=4 brej=7 brrej=2 bvalve=4 bframes=900 bms=9000 "
            "bdrop=3 bsrej=0 bfar=0 bnear=0 bsv=0 bcold=20 br4=800 br3=150 "
            "br2=40 br1=10 br0=0\n",
            "CAM: blobs 30,40,3,1 200,41,4,1 31,140,3,1 210,150,14,0\n",
        ):
            link.src.q.put(ln)
        if nb is not None:
            nb.select(0)                     # the Camera tab
        root.update()
        time.sleep(2.2)                      # let board_tick and blob_tick run
        root.update()
        all_text = texts(root, [])
        joined = "\n".join(all_text)
        if "size 14 DROPPED" not in joined:
            errs.append("the blob readout never showed the rejected blob: %s"
                        % [t for t in all_text if "blobs now" in t])
        if "saw all four LEDs" not in joined:
            errs.append("the corner-loss percentages never appeared")
        if link.last.get("bcold") != 20:
            errs.append("bcold did not reach link.last: %r"
                        % link.last.get("bcold"))
        # R8 follow-up: the FIRST sighting of a since-boot counter is not a
        # delta (same rule as brej/bnear/etc above) -- so on this cold start
        # bcold contributes 0 and the bucket is just renamed, not yet bigger.
        # The fold itself is checked once a second reply moves bcold, below.
        if "5% two or fewer, or no lock" not in joined:
            errs.append("the two-or-fewer bucket was not renamed: %s"
                        % [t for t in all_text if "saw all four LEDs" in t])
        if link.last.get("bmax") != 9:
            errs.append("blob gate keys did not reach link.last")
        if link.last.get("fmt") != 1 or link.last.get("fullreg") != 5:
            errs.append("fmt / fullreg did not reach link.last: %r"
                        % {k: link.last.get(k) for k in ("fmt", "fullreg")})
        # A first reading of bvalve is not a give-back happening NOW. It
        # counts since boot, so shown raw the warning latched on and never
        # came off again for the rest of the power cycle.
        if "SIZE WINDOW TOO TIGHT" in joined:
            errs.append("a since-boot bvalve raised the size-window warning "
                        "before anything had been given back")

        # An older gun that has never heard of bcold must still parse cleanly
        # -- the fold treats it as zero, not as a missing field the parser
        # chokes on.
        Lold = gun_studio.Link()
        Lold.src = FakeSource("OLD")
        Lold.src.q.put(
            "CAM: blob fmt=1 ext=1 fullreg=5 bmin=2 bmax=9 rtol=3 bhmax=0 "
            "pxmax=0 armax=0 hwmax=-1 hwmin=-1 bn=4 brej=7 brrej=2 bvalve=4 "
            "bframes=900 bms=9000 bdrop=3 bsrej=0 bfar=0 bnear=0 br4=800 "
            "br3=150 br2=40 br1=10 br0=0\n")
        Lold.pump()
        if "bcold" in Lold.last:
            errs.append("bcold appeared in link.last from a reply that never "
                        "sent it: %r" % Lold.last.get("bcold"))
        if Lold.last.get("bn") != 4 or Lold.last.get("br4") != 800:
            errs.append("an old gun's blob reply, minus bcold, did not parse "
                        "at all: %r" % Lold.last)

        # ---- what a click actually puts on the wire ------------------------
        # ext: for 0 and 1, fmt: only for full. The previous firmware has no
        # fmt key and drops it without a word, so a gun on it could never
        # leave detail 0 -- and every gate row below the radio stays dead
        # until sizes arrive.
        radios = {str(w.cget("text")): w
                  for w in by_text(root, ("off", "sizes", "full detail",
                                          "0x55", "0x05"))}
        missing = [n for n in ("off", "sizes", "full detail", "0x55", "0x05")
                   if n not in radios]
        if missing:
            errs.append("camera tab is missing radio buttons: %s" % missing)
        for name, want in (("off", "~cam=ext:0"), ("sizes", "~cam=ext:1"),
                           ("full detail", "~cam=fmt:2")):
            if name not in radios:
                continue
            WIRE.clear()
            radios[name].invoke()
            drained(link)
            if want not in WIRE:
                errs.append("'%s' sent %s, not %s" % (name, WIRE, want))
        # Full mode is PERSISTED now. The save used to clamp it back to
        # extended, so every saved bhmax loaded into a gun that could never
        # execute it: the gate died on each power cycle while the tools went on
        # reporting it as active. The panel has to say that a save keeps the
        # format, because saving the gate without the format is the same dead
        # gate -- and this sentence is the only place a user is told.
        fmt_log = logbox_text(root)
        if "keeps this format" not in fmt_log:
            errs.append("nothing says a save keeps full detail, so the gate "
                        "it needs looks like it survives a power cycle on its "
                        "own: %r" % fmt_log[-300:])
        if "0x05" in radios:
            WIRE.clear()
            radios["0x05"].invoke()
            drained(link)
            if "~cam=fullreg:5" not in WIRE:
                errs.append("the full-mode register radio sent %s" % WIRE)

        # ---- testing-relevant in front, the rest behind a disclosure -------
        # The panel is read with a gun in one hand and a test running, and
        # eight spinboxes deep it stopped being readable at all. What is in
        # FRONT is what a test needs: the sensitivity, the report format, the
        # height gate, and the buttons that record and save. Everything set
        # once -- or superseded -- is one click away and no closer.
        #
        # Asserted by winfo_ismapped(), not by existence: every control below
        # is built either way, and a disclosure that never actually hides
        # anything is the failure this is guarding against.
        gate_box = {}
        for sp in spinboxes(root):
            vals = str(sp.cget("values"))
            if "2.5:1" in vals:
                gate_box["armax"] = sp
            elif "12 px" in vals:
                gate_box["pxmax"] = sp
            elif "10 rows" in vals:
                gate_box["bhmax"] = sp
        adv_btn = by_text(root, ("▸  Advanced settings",
                                 "▾  Advanced settings"), cls="Button")
        if not adv_btn:
            errs.append("the camera panel has no Advanced disclosure: %s"
                        % [t for t in texts(root, []) if "Advanced" in t])
        elif "bhmax" not in gate_box:
            errs.append("the camera panel has no blob-height control -- the "
                        "one gate measured to work is the one that has to be "
                        "in front (spinboxes: %s)"
                        % [str(s.cget("values")) for s in spinboxes(root)])
        else:
            adv_btn = adv_btn[0]
            # The shape-capture button answers to three labels, because the
            # capture is armed at boot and this app does not know which state
            # it is in until the gun says: "Shape capture ?" is what stands
            # there in between. Pinned as a set rather than as the one label,
            # so a pass that removed the unknown state would still be seen.
            front = {"sensitivity": by_text(root, ("Default",), cls="Button"),
                     "blob detail": by_text(root, ("full detail",)),
                     "learn": by_text(root, ("Learn LED shape",
                                             "Stop learning",
                                             "Shape capture ?"),
                                      cls="Button"),
                     "auto limit": by_text(root, ("auto: on", "auto: off"),
                                           cls="Button"),
                     "CSV": by_text(root, ("Shape CSV",), cls="Button"),
                     "save": by_text(root, ("Save to gun",), cls="Button")}
            missing = [k for k, v in front.items()
                       if not any(shown(w) for w in v)]
            if missing or not shown(gate_box["bhmax"]):
                errs.append("a control a test needs is not on the front of "
                            "the panel: %s"
                            % (missing + ([] if shown(gate_box["bhmax"])
                                          else ["blob height"])))
            # ...and the rest is NOT, until it is asked for.
            hidden = {"bmin/bmax/rtol": [bspin for bspin in spinboxes(root)
                                         if str(bspin.cget("values")) == ""],
                      "pxmax": [gate_box["pxmax"]],
                      "armax": [gate_box["armax"]],
                      "full-mode register": by_text(root, ("0x55",)),
                      "sensor test": by_text(root,
                                             ("Test sensor connection",),
                                             cls="Button")}
            leaked = [k for k, v in hidden.items()
                      if v and any(shown(w) for w in v)]
            if leaked:
                errs.append("the Advanced settings are on the panel with "
                            "everything else -- the split does nothing: %s"
                            % leaked)
            if not hidden["bmin/bmax/rtol"]:
                errs.append("the size window and odd-one-out spinboxes are "
                            "gone entirely, not merely hidden")
            # One click, and every one of them is there.
            adv_btn.invoke()
            root.update()
            still = [k for k, v in hidden.items()
                     if v and not all(shown(w) for w in v)]
            if still:
                errs.append("opening Advanced did not reveal: %s" % still)
            if str(adv_btn.cget("text")) == "▸  Advanced settings":
                errs.append("the disclosure arrow did not turn round, so the "
                            "button cannot say which way it is going to go")
            # armax is actively wrong at sensitivity 2 -- the sensor smears a
            # 2x2 blob out to 12x3, so the ratio measures the gain and not the
            # LED. It stays because a gun in the field may be set on it, and
            # it has to SAY so where it is set.
            adv_text = "\n".join(texts(root, []))
            if ("not recommended" not in adv_text
                    or "DEPRECATED" not in adv_text):
                errs.append("armax sits among the Advanced settings without "
                            "being marked DEPRECATED and not recommended -- "
                            "it still loads and applies, so a gun in the "
                            "field may be set on it, and nothing else on the "
                            "panel says not to")
            # And it closes again, or a disclosure is just a slow way of
            # showing everything.
            adv_btn.invoke()
            root.update()
            if any(shown(w) for w in hidden["armax"]):
                errs.append("the disclosure would not close again")
            adv_btn.invoke()
            root.update()

        # ---- the shape gate ------------------------------------------------
        # The bhmax and pxmax floors are THIS RIG's own measurements now -- the
        # gun refuses anything below the tallest LED and the largest LED pixel
        # count it has ever measured, and names that figure -- so there is no
        # fixed illegal band left to test against. What still has to hold is
        # the LADDER: on a gun that has measured nothing the firmware accepts
        # whatever it is sent, and bhmax:1 accepted is a gun that sees nothing
        # and says nothing about it. So all three step through a fixed LIST of
        # values somebody could mean rather than by 1 from 0, and the list is
        # what is being tested -- every rung, walked in both directions, has
        # to be a value worth sending, and 0 has to stay reachable.
        if len(gate_box) != 3:
            errs.append("the camera tab is missing shape-gate controls (found "
                        "%s of %d spinboxes)"
                        % (sorted(gate_box), len(spinboxes(root))))
        else:
            # The 700 ms sync pulls these boxes back to whatever the gun last
            # said. Forgetting the gun's value first leaves the walk below
            # measuring the ladder and nothing else.
            for k in ("bhmax", "pxmax", "armax"):
                link.last.pop(k, None)
            # And put the detail radio back to 'sizes', which the click test
            # above left on 'full detail'. Two things ride on it: the shape
            # gate does nothing outside full mode, which the panel has to say
            # out loud, and a test that only ever steps these in full mode
            # would never find out whether it does.
            if "sizes" in radios:
                radios["sizes"].invoke()
                root.update()
            # The floors below are the LADDER's lowest useful rungs, not the
            # firmware's -- bhmax and pxmax are refused against what this rig
            # measured, which varies by bar. A ladder that could emit 1, 2 or 3
            # is the danger: on a gun with no capture the firmware takes them,
            # and a bhmax of 1 rejects every blob the sensor can produce.
            for key, floor in (("bhmax", 8), ("pxmax", 12), ("armax", 16)):
                sp = gate_box[key]
                sent = []
                for direction in ("buttonup", "buttondown"):
                    # Twelve clicks queue twenty-four lines; they are read
                    # off the wire once the queue has drained, not per click.
                    WIRE.clear()
                    for _ in range(12):
                        sp.invoke(direction)
                        root.update()
                    drained(link, 8.0)
                    sent += [w for w in WIRE
                             if w.startswith("~cam=%s:" % key)]
                vals = sorted(set(int(w.split(":")[1]) for w in sent))
                illegal = [v for v in vals if v and v < floor]
                if not vals:
                    errs.append("stepping the %s box put nothing on the wire"
                                % key)
                elif illegal:
                    errs.append("the %s ladder can emit %s -- a value no rig "
                                "wants, taken on trust by any gun that has "
                                "measured nothing (emitted %s)"
                                % (key, illegal, vals))
                elif 0 not in vals:
                    errs.append("the %s ladder cannot reach 0, so the gate "
                                "can be turned on but never off: %s"
                                % (key, vals))
            # There is no room beside these two for a hint, and a gate set
            # outside full mode does nothing at all -- so the panel has to say
            # so somewhere. The log is where this panel already puts what will
            # not fit beside a control.
            gate_log = logbox_text(root)
            if "full detail" not in gate_log or "stands down" not in gate_log:
                errs.append("stepping the shape gate outside full mode never "
                            "said the gate would do nothing: %r"
                            % gate_log[-300:])
            # And the box follows the GUN, in the gun's own units translated
            # into the reader's. 20 eighths is 2.5:1, and a panel showing 20
            # beside a label saying roundness is a setting nobody can check.
            # Every counter left exactly where the first reply put it: the
            # readout below this is built from DELTAS, and a reply that moves
            # one to prove a translation would spend the give-back the
            # give-back test is waiting for.
            link.src.q.put(
                "CAM: blob fmt=1 ext=1 fullreg=5 bmin=2 bmax=9 rtol=3 "
                "bhmax=10 pxmax=14 armax=20 hwmax=-1 hwmin=-1 bn=4 brej=7 "
                "brrej=2 "
                "bvalve=4 bframes=900 bms=9000 bdrop=3 bsrej=0 bfar=0 "
                "bnear=0 br4=800 br3=150 br2=40 br1=10 br0=0\n")
            settle_t = time.time() + 1.6
            while time.time() < settle_t:
                root.update()
                time.sleep(0.05)
            shown_box = {k: str(gate_box[k].get()) for k in gate_box}
            if (shown_box.get("armax") != "2.5:1"
                    or shown_box.get("pxmax") != "14 px"
                    or shown_box.get("bhmax") != "10 rows"):
                errs.append("the shape boxes did not follow the gun into the "
                            "reader's units: %s" % shown_box)
            if (link.last.get("pxmax") != 14 or link.last.get("armax") != 20
                    or link.last.get("bhmax") != 10):
                errs.append("bhmax / pxmax / armax never reached link.last: "
                            "%r" % {k: link.last.get(k)
                                    for k in ("bhmax", "pxmax", "armax")})
            for k in ("bsrej", "bfar", "bnear"):
                if k not in link.last:
                    errs.append("'%s' never reached link.last, so the CSV "
                                "column for it can only ever be blank" % k)

        # ---- basic mode: the gun says why every size is -1 -----------------
        # The trailer is the only thing on the line that explains the -1s.
        # Dropped, the readout looked like four blobs measured at size -1 and
        # the gates below it looked broken rather than unfed.
        for ln in (
            "CAM: blob fmt=0 ext=0 fullreg=5 bmin=2 bmax=9 rtol=3 hwmax=-1 "
            "hwmin=-1 bn=4 brej=7 brrej=2 bvalve=4 bframes=1000 bms=10000 "
            "bdrop=3 br4=840 br3=170 br2=45 br1=12 br0=0\n",
            "CAM: blobs 30,40,-1,1 200,41,-1,1 31,140,-1,1 210,150,-1,1 "
            "(sizes need fmt:1)\n",
        ):
            link.src.q.put(ln)
        root.update()
        time.sleep(1.6)
        root.update()
        joined = "\n".join(texts(root, []))
        if "size -1" not in joined or "set blob detail" not in joined:
            errs.append("basic mode showed size -1 with no explanation: %s"
                        % [t for t in texts(root, []) if "blobs now" in t])

        # ---- full detail: NINE fields per blob -----------------------------
        # Box, pixel count and the box origin only exist here, and the readout
        # line is at its longest -- which is the state the panel has to still
        # fit in. The boxes below are the real geometry: a blob reported at
        # 130,140 in the 240x176 pipeline is at 69,76 in the sensor's 128x96
        # array, so a 11x9 box whose origin is 64,72 has its centre within a
        # pixel of the crosshair. That agreement is the measurement.
        for ln in (
            "CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 bhmax=10 "
            "pxmax=14 armax=20 hwmax=-1 "
            "hwmin=-1 bn=4 brej=90 brrej=39 bvalve=61 bframes=1900 bms=19000 "
            "bdrop=3 bsrej=31 bfar=9 bnear=0 br4=1700 br3=250 br2=90 br1=40 "
            "br0=20\n",
            "CAM: blobs 130,140,3,1,11,9,214,64,72 200,141,14,0,12,10,203,"
            "101,72 131,140,13,0,10,9,198,65,72 210,150,14,0,41,38,255,92,63"
            "\n",
        ):
            link.src.q.put(ln)
        root.update()
        time.sleep(1.6)
        root.update()
        all_text = texts(root, [])
        joined = "\n".join(all_text)
        if "box 41x38 255px" not in joined:
            errs.append("full mode never showed the box and pixel count: %s"
                        % [t for t in all_text if "blobs now" in t])
        if "set blob detail" in joined:
            errs.append("the basic-mode 'sizes need fmt:1' hint outlived "
                        "basic mode")
        # bvalve MOVED this time, so the warning is real and has to show...
        if "SIZE WINDOW TOO TIGHT" not in joined:
            errs.append("a give-back happening now raised no size-window "
                        "warning: %s"
                        % [t for t in all_text if "saw all four LEDs" in t])
        # ...and the shape gate's own drops sit beside the other two, so the
        # reader can see WHICH gate is doing the work. A total that lumps
        # them together cannot answer that, and it is the first question
        # after tightening one of them.
        if "31 shape" not in joined:
            errs.append("the shape gate's drops are not in the readout: %s"
                        % [t for t in all_text if "dropped" in t])
        if link.last.get("fullreg") != 85:
            errs.append("the register radio did not follow the gun back to "
                        "0x55: %r" % link.last.get("fullreg"))
        # A Tk label wider than the frame it sits in is CLIPPED, silently: the
        # reader simply never sees the end of the sentence, and these lines
        # carry live numbers whose length is not known in advance. Measured
        # here rather than eyeballed, because it has already shipped once.
        def wide_labels(w, out):
            try:
                t = str(w.cget("text"))
            except Exception:
                t = ""
            if any(m in t for m in ("blobs now", "saw all four LEDs",
                                    "Read the sizes FIRST")):
                m = w.master
                if m is not None and m.winfo_width() > 1 \
                        and w.winfo_reqwidth() > m.winfo_width():
                    out.append("%r needs %dpx in a %dpx panel"
                               % (t[:40], w.winfo_reqwidth(), m.winfo_width()))
            for c in w.winfo_children():
                wide_labels(c, out)
            return out
        clipped = wide_labels(root, [])
        if clipped:
            errs.append("clipped label(s): %s" % "; ".join(clipped))
        # And the same in the other direction: the window does not grow, so a
        # panel taller than the tab area has its last lines cut off the bottom
        # -- which is what wrapping the lines above cost the first time.
        # The TIGHTEST container holding both markers is the panel itself; an
        # ancestor holds them too and would be measured instead.
        #
        # Measured with the Advanced disclosure OPEN, because that is the
        # tallest this panel ever gets and a margin that only holds while
        # something is hidden is not a margin.
        if adv_btn and not shown(gate_box.get("armax")):
            adv_btn.invoke()
            root.update()
        cands = []
        def collect(w):
            t = texts(w, [])
            if (any("blob detail" in s for s in t)
                    and any("wiicam sensitivity" in s for s in t)):
                cands.append(w)
            for c in w.winfo_children():
                collect(c)
        collect(root)
        # By height AND THEN width. When the panel is the tallest thing on the
        # page the page requests exactly the same height, and a tie broken the
        # wrong way measures the whole page -- whose width is the panel plus
        # the preview column, so the width check below would pass on a panel
        # that was running off its own edge.
        wii = (min(cands, key=lambda w: (w.winfo_reqheight(),
                                         w.winfo_reqwidth()))
               if cands else None)
        if wii is not None and nb is not None:
            need = wii.winfo_reqheight()
            # The tab PAGE's own allocated height is the content area exactly.
            # Deriving it from the notebook minus a guessed tab-strip height
            # was off by a pixel or two and reported false failures.
            # A ROW wider than the panel is the same failure seen from the
            # other side: pack does not shrink it, it runs off the right-hand
            # edge with nothing to say so, and the window width does not grow.
            # The panel's own request is the widest of its rows, so one number
            # covers every one of them -- including the ones that put a
            # heading or a hint beside the controls to save a line.
            if wii.winfo_reqwidth() > wii.winfo_width():
                errs.append("the wiicam camera panel needs %dpx of width in a "
                            "%dpx column -- a row runs off the edge"
                            % (wii.winfo_reqwidth(), wii.winfo_width()))
            page = nb.nametowidget(nb.tabs()[0])
            have = page.winfo_height()
            # Printed, not only asserted. The panel's budget is the one number
            # every new row on it is argued against, and a comment quoting a
            # figure measured on somebody else's fonts is how it drifts.
            print("studio: wiicam panel %dpx of a %dpx tab area" % (need, have))
            if need > have:
                errs.append("the wiicam camera panel needs %dpx of a %dpx tab "
                            "area -- its last lines are cut off" % (need, have))
            else:
                # Fitting is not the same as having room. The notebook is
                # packed WITHOUT expand, so the tab page is only ever as tall
                # as the panel asks for: a panel one pixel from being cut off
                # reports need == have, exactly like a comfortable one. Ask
                # for MARGIN more and see whether the window can still supply
                # it -- that is the slack, measured rather than assumed.
                # Run at 768 px of screen height or this proves nothing.
                MARGIN = 20
                pad = _tk.Frame(wii, height=MARGIN, bg=wii.cget("bg"))
                pad.pack(fill="x")
                root.update()
                grown_need, grown_have = wii.winfo_reqheight(), \
                    page.winfo_height()
                pad.destroy()
                root.update()
                print("studio: with a %dpx probe on it, %dpx wanted of %dpx"
                      % (MARGIN, grown_need, grown_have))
                if grown_need > grown_have:
                    errs.append("the wiicam camera panel fits with less than "
                                "%dpx to spare: %dpx of a tab area that stops "
                                "growing at %dpx" % (MARGIN, need, grown_have))
                # ...and the two newest rows on the panel were ON it while
                # that was measured. The fit row is two buttons and a line --
                # 21 px of a tab area that stops growing at 313 -- and it was
                # paid for by taking the gaps out from between every other
                # row; the loop row is a switch and a line, 20 px, and it was
                # paid for by taking the pady off every button that still had
                # one. Measured at 1400x768 with the disclosure open and both
                # readout lines wrapped, the panel asks for 291 px of the 313.
                # A margin taken against a panel that had quietly dropped
                # either row would prove nothing about the layout anybody
                # actually sees.
                for label, want in (("the fit row", ("Measure the gate",)),
                                    ("the loop row", ("auto: on",
                                                      "auto: off"))):
                    if not any(shown(w) for w
                               in by_text(root, want, cls="Button")):
                        errs.append("%s was not on the panel that was just "
                                    "measured (%dpx of %dpx), so the margin "
                                    "says nothing about it"
                                    % (label, need, grown_have))
        else:
            errs.append("could not find the wiicam camera panel to measure")

        # ---- the loop row, on the panel -------------------------------------
        # The parsing and the wording are pinned above; this is the wiring
        # between them -- that the poll actually goes out, that the row
        # follows the gun's answer, and that the switch sends what it says it
        # sends. Run with the Advanced disclosure open, because the note that
        # says why the loop is off lives beside the box that turns it off.
        def wait(secs=1.2):
            t_end = time.time() + secs
            while time.time() < t_end:
                root.update()
                time.sleep(0.05)

        def label_starting(pfx):
            out = []

            def walk(w):
                try:
                    if w.winfo_class() == "Label" \
                            and str(w.cget("text")).startswith(pfx):
                        out.append(w)
                except Exception:
                    pass
                for c in w.winfo_children():
                    walk(c)
            walk(root)
            return out[0] if out else None

        loop_btns = by_text(root, ("auto: on", "auto: off"), cls="Button")
        if not loop_btns:
            errs.append("the camera panel has no auto-light-limit switch")
        else:
            btn_loop = loop_btns[0]
            # The poll. On blob_tick's own cadence and NOT on a timer of its
            # own: a second after() chain on the same port is two questions
            # racing for one wire, and the answer that loses is a camera frame
            # the preview never gets.
            WIRE.clear()
            wait(2.4)
            n_polls = WIRE.count("~camloop?")
            if not 1 <= n_polls <= 4:
                errs.append("'~camloop?' went out %d times in 2.4 s -- the "
                            "poll is meant to be about one a second: %s"
                            % (n_polls, WIRE[:12]))
            # The gun answers, and the row says it in words a player can act
            # on. Fed as a reply, never set locally: what this row shows has
            # to be the gun's state and not this app's memory of a click.
            link.src.q.put("CAM: loop on=1 state=HOLD val=63 lo=48 hi=80 "
                           "dwell=22/50 clean=47 stray=1 cut=0 settled=1 "
                           "saved=1\n")
            wait(1.4)
            lbl = label_starting("Auto light limit")
            if lbl is None:
                errs.append("the loop row is not on the panel at all")
            else:
                txt = str(lbl.cget("text"))
                for w in ("holding at 63", "clean 47 stray 1 cut 0", "saved"):
                    if w not in txt:
                        errs.append("the loop row does not say %r: %r"
                                    % (w, txt))
                if str(btn_loop.cget("text")) != "auto: on":
                    errs.append("the switch did not follow the gun into on: "
                                "%r" % btn_loop.cget("text"))
            # A hand-set sensor max turns the loop off, and the firmware does
            # it silently. The row that changes is on the front of the panel
            # while the box is under Advanced, so it is said in both places.
            if True:
                hw = [sp for sp in spinboxes(root)
                      if str(sp.cget("to")) in ("255", "255.0")
                      and str(sp.cget("from")) in ("-1", "-1.0")
                      and str(sp.cget("increment")) in ("5", "5.0")]
                if not hw:
                    errs.append("the sensor max spinbox is not on the panel")
                else:
                    # Stepped rather than typed: the arrows and the Return
                    # binding run the same gate_send, and event_generate on an
                    # unfocused Spinbox is a no-op that would pass this test
                    # while proving nothing.
                    WIRE.clear()
                    hw[0].invoke("buttonup")
                    drained(link)
                    if not any(w.startswith("~cam=hwmax:") for w in WIRE):
                        errs.append("stepping the sensor max sent %s" % WIRE)
                    if "~camloop?" not in WIRE:
                        errs.append("a hand-set sensor max did not ask the "
                                    "gun what that did to the loop: %s" % WIRE)
                    logged = logbox_text(root)
                    if "auto light limit is now OFF" not in logged:
                        errs.append("nothing said that setting it by hand "
                                    "switches the loop off: %r"
                                    % logged[-300:])
                    if "NOT saved" not in logged:
                        errs.append("...nor that a hand-set one is the kind "
                                    "that is not saved: %r" % logged[-300:])
            # ...and then the gun says so, which is what the panel must
            # actually follow.
            link.src.q.put("CAM: loop on=0 state=OFF val=120 lo=0 hi=256 "
                           "dwell=0/50 clean=0 stray=0 cut=0 settled=0 "
                           "saved=0\n")
            wait(1.4)
            # 'off - limit 120', no reason: the readout cannot tell a hand-set
            # limit from the switch's own 'loop:0'.
            lbl = label_starting("Auto light limit")
            if lbl is not None and "off - limit 120" not in str(
                    lbl.cget("text")):
                errs.append("the loop row did not say it was off, with the "
                            "limit the sensor keeps: %r" % lbl.cget("text"))
            if str(btn_loop.cget("text")) != "auto: off":
                errs.append("the switch did not follow the gun into off: %r"
                            % btn_loop.cget("text"))
            note = label_starting("loop off")
            if note is None:
                errs.append("nothing beside the sensor max box says the loop "
                            "is off: %s"
                            % [t for t in texts(root, []) if "loop" in t])
            elif "hand" in str(note.cget("text")):
                errs.append("the note beside the box claims a reason the gun "
                            "never sent: %r" % note.cget("text"))
            # And the switch puts it back.
            WIRE.clear()
            btn_loop.invoke()
            drained(link)
            if "~cam=loop:1" not in WIRE or "~camloop?" not in WIRE:
                errs.append("the switch did not turn the loop back on and ask "
                            "straight back: %s" % WIRE)
            link.src.q.put("CAM: loop on=1 state=LOWER val=90 lo=48 hi=256 "
                           "dwell=4/50 clean=39 stray=5 cut=0 settled=0 "
                           "saved=0\n")
            wait(1.4)
            WIRE.clear()
            btn_loop.invoke()
            drained(link)
            if "~cam=loop:0" not in WIRE:
                errs.append("the switch did not turn a running loop off: %s"
                            % WIRE)
            # With the state UNKNOWN the switch must not guess: 'loop:1' at a
            # loop that is already running wipes its search and its saved
            # limit, so it asks the gun instead of sending either.
            link.loop = {}
            for k in ("loop", "hwv", "hwlo", "hwhi", "hws"):
                link.last.pop(k, None)
            WIRE.clear()
            btn_loop.invoke()
            drained(link)
            if any(w.startswith("~cam=loop:") for w in WIRE):
                errs.append("the switch sent a loop command before the gun "
                            "had said whether the loop was on: %s" % WIRE)
            if "~camloop?" not in WIRE:
                errs.append("...and did not ask the gun for the state: %s"
                            % WIRE)
            # Put the gun back where the rest of this file expects it.
            link.src.q.put("CAM: loop on=1 state=HOLD val=63 lo=48 hi=80 "
                           "dwell=22/50 clean=47 stray=1 cut=0 settled=1 "
                           "saved=1\n")
            wait(1.0)

        # ---- the shape preview ----------------------------------------------
        # Coordinates cannot say whether the thing in slot 2 is a point of
        # light or a window; a picture can. Nothing here is readable as text,
        # so the canvas items themselves are what gets counted -- and the one
        # number that has to be right is the crosshair, which is a MEASUREMENT
        # of whether the box fields are in the array we believe they are.
        # Found through its heading rather than by size: there are two
        # canvases on this window and picking the wrong one would test the
        # live quad view instead, which draws four dots whatever the blobs did.
        prev_cv = None

        def find_preview(w):
            try:
                if (w.winfo_class() == "Label"
                        and "shaded by density" in str(w.cget("text"))):
                    kids = canvases(w.master)
                    if kids:
                        return kids[0]
            except Exception:
                pass
            for c in w.winfo_children():
                r = find_preview(c)
                if r is not None:
                    return r
            return None
        prev_cv = find_preview(root)
        if prev_cv is None:
            errs.append("the camera tab has no shape preview at all: %s"
                        % [t for t in texts(root, []) if "sensor pixels" in t])
        else:
            def preview():
                """(kinds seen, the verdict line under the canvas)."""
                kinds = {}
                for i in prev_cv.find_all():
                    k = prev_cv.type(i)
                    kinds[k] = kinds.get(k, 0) + 1
                said = [t for t in texts(root, [])
                        if "kept," in t or "reporting nothing" in t
                        or "waiting for" in t]
                return kinds, (said[0] if said else "")

            def feed_blobs(line, secs=1.4):
                # Only the blob list, never the counter line: every rate and
                # warning on this panel is a DELTA, and moving a counter to
                # redraw a picture would spend a give-back the readout test
                # further down is waiting for.
                link.src.q.put(line + "\n")
                t_end = time.time() + secs
                while time.time() < t_end:
                    root.update()
                    time.sleep(0.05)

            kinds, said = preview()
            # One boundary rectangle plus one box per blob; two lines per
            # crosshair; one height label per box.
            if kinds.get("rectangle") != 5:
                errs.append("the preview drew %s rectangles for the frame and "
                            "four boxes" % kinds.get("rectangle"))
            if kinds.get("line", 0) < 8:
                errs.append("the preview drew %s lines -- four crosshairs is "
                            "eight of them, and the crosshair is the whole "
                            "measurement" % kinds.get("line"))
            if kinds.get("text", 0) < 4:
                errs.append("the preview labelled %s of four boxes with the "
                            "height the gate judges" % kinds.get("text"))
            if "ARE in this 128x96 array" not in said:
                errs.append("nine-field blobs whose boxes sit exactly on their "
                            "reported positions did not confirm the units: %r"
                            % said)
            # The height labels are the gate made visible: bhmax is 10, so the
            # 38-row box has to be marked and the 9-row one must not be.
            marks = {}
            for i in prev_cv.find_all():
                if prev_cv.type(i) == "text":
                    marks[str(prev_cv.itemcget(i, "text"))] = \
                        str(prev_cv.itemcget(i, "fill"))
            if marks.get("h38") == marks.get("h9"):
                errs.append("every box is labelled the same colour, so the "
                            "picture does not say which blobs the height "
                            "limit is about to cost: %s" % marks)
            # ...and a box that does NOT sit on its position has to be loud.
            # This is the case the preview exists for: if it ever happens on a
            # real gun, the box fields are not in the array we think they are
            # and every shape number taken off them is meaningless.
            feed_blobs("CAM: blobs 130,140,3,1,11,9,214,10,10 "
                       "200,141,14,0,12,10,203,101,72")
            kinds, said = preview()
            if "MISMATCH" not in said:
                errs.append("a box centre 60 px from its own crosshair was "
                            "reported as agreement: %r" % said)
            # Seven fields: the older full report, with no origin in it. The
            # box is real and its PLACE is not, and a preview that drew it on
            # the crosshair without saying so would look like the units had
            # just been confirmed.
            feed_blobs("CAM: blobs 130,140,3,1,11,9,214 200,141,4,1,12,10,203")
            kinds, said = preview()
            if "ORIGIN" not in said:
                errs.append("a seven-field report drew boxes at the crosshair "
                            "and claimed nothing was wrong: %r" % said)
            if kinds.get("rectangle") != 3:
                errs.append("a seven-field report drew %s rectangles instead "
                            "of the frame and two boxes"
                            % kinds.get("rectangle"))
            # Four fields: no shape at all. Crosshairs only, and it says which
            # setting would fill the boxes in.
            feed_blobs("CAM: blobs 30,40,3,1 200,41,4,1 31,140,3,1")
            kinds, said = preview()
            if kinds.get("rectangle") != 1 or kinds.get("line", 0) < 6:
                errs.append("a four-field report drew %s rectangles and %s "
                            "lines -- it has three crosshairs and no boxes"
                            % (kinds.get("rectangle"), kinds.get("line")))
            if "full detail" not in said:
                errs.append("positions-only did not say which setting brings "
                            "the boxes back: %r" % said)
            # And a sensor that is reporting nothing has to say so rather than
            # leaving the last good frame on the screen for ever.
            feed_blobs("CAM: blobs")
            kinds, said = preview()
            if kinds.get("rectangle") != 1 or "reporting nothing" not in said:
                errs.append("an empty report left %s rectangles up and said "
                            "%r" % (kinds.get("rectangle"), said))
            # Put a real frame back, so the state the screenshot catches and
            # the rows below measure is the one the panel actually lives in.
            feed_blobs("CAM: blobs 130,140,3,1,11,9,214,64,72 "
                       "200,141,14,0,12,10,203,101,72 "
                       "131,140,13,0,10,9,198,65,72 "
                       "210,150,14,0,41,38,255,92,63")

        # ---- the false-negative meter --------------------------------------
        # bnear counts blobs a gate threw away that sat exactly where the
        # missing corner had to be, so they were almost certainly LEDs. It is
        # the only number on this panel that says a gate is WRONG, and read as
        # one more drop count it would look like the gate earning its keep --
        # the exact opposite of what it means.
        for ln in (
            "CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 bhmax=10 "
            "pxmax=14 armax=20 hwmax=-1 "
            "hwmin=-1 bn=4 brej=120 brrej=50 bvalve=90 bframes=2400 "
            "bms=24000 bdrop=3 bsrej=44 bfar=12 bnear=6 br4=2100 br3=280 "
            "br2=95 br1=44 br0=22\n",
            "CAM: blobs 130,140,3,1,11,9,214,64,72 "
            "200,141,14,0,12,10,203,101,72 131,140,13,0,10,9,198,65,72 "
            "210,150,14,0,41,38,255,92,63\n",
        ):
            link.src.q.put(ln)
        root.update()
        time.sleep(1.6)
        root.update()
        near_text = texts(root, [])
        near = "\n".join(near_text)
        if "GATE MAY BE TAKING REAL LEDs" not in near:
            errs.append("bnear moving raised no warning at all: %s"
                        % [t for t in near_text if "dropped" in t])
        # ...and it says how to switch the gate off, in the same breath. The
        # gate is a setting somebody chose; a warning that says only "this may
        # be wrong" leaves them with a doubt and nothing to press. bhmax:0 is
        # the one key that takes the shape gate out of the picture entirely.
        if not any("GATE MAY BE TAKING REAL LEDs" in t and "bhmax:0" in t
                   for t in near_text):
            errs.append("the false-negative warning does not say how to turn "
                        "the gate off: %s"
                        % [t for t in near_text if "GATE MAY" in t])
        if "6 shape" in near or "6 size" in near or "6 odd" in near:
            errs.append("bnear was folded in with the drop counts, where it "
                        "reads as the gate working rather than as the gate "
                        "being wrong: %s"
                        % [t for t in near_text if "dropped" in t])
        # The sentence that will not fit beside the numbers goes to the log,
        # once, which is where this panel already puts its reasoning.
        near_log = logbox_text(root)
        if "corner" not in near_log or "false-negative" not in near_log:
            errs.append("the log never explained what the warning means: %r"
                        % near_log[-300:])
        if near_log.count("GATE MAY BE TAKING REAL LEDs") > 1:
            errs.append("the explanation repeats on every poll, which pushes "
                        "the diag verdict and the save result out of a "
                        "six-line log box")
        # Both warnings up at once is the deepest this readout ever gets: one
        # more wrapped line than the state measured above, and the panel has
        # to survive it without a single row being cut off the bottom. It is a
        # weaker bar than the 20 px margin on purpose -- the margin is for the
        # state the panel sits in, this is for the state it can reach.
        if wii is not None and nb is not None:
            page = nb.nametowidget(nb.tabs()[0])
            if wii.winfo_reqheight() > page.winfo_height():
                errs.append("with both warnings up the wiicam panel needs "
                            "%dpx of a %dpx tab area -- its last rows are cut "
                            "off, which is where 'Save to gun' lives"
                            % (wii.winfo_reqheight(), page.winfo_height()))

        # ...and the give-back warning has to come OFF again once the gun
        # stops giving blobs back. bvalve counts since boot: read raw it
        # latched for the rest of the power cycle, on a panel whose whole job
        # is to say whether the window the user just typed is a good one.
        for ln in (
            "CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 bhmax=10 "
            "pxmax=14 armax=20 hwmax=-1 "
            "hwmin=-1 bn=4 brej=120 brrej=50 bvalve=90 bframes=2900 "
            "bms=29000 bdrop=3 bsrej=44 bfar=12 bnear=6 br4=2600 br3=300 "
            "br2=100 br1=45 br0=25\n",
            "CAM: blobs 130,140,3,1,11,9,214,64,72 "
            "200,141,4,1,12,10,203,101,72 131,140,3,1,10,9,198,65,72 "
            "210,150,3,1,41,38,255,92,63\n",
        ):
            link.src.q.put(ln)
        root.update()
        time.sleep(1.6)
        root.update()
        cleared = "\n".join(texts(root, []))
        if "SIZE WINDOW TOO TIGHT" in cleared:
            errs.append("the size-window warning never cleared after the "
                        "give-backs stopped")
        if "GATE MAY BE TAKING REAL LEDs" in cleared:
            errs.append("the false-negative warning latched on a since-boot "
                        "total instead of clearing when the gate stopped")

        # A REFUSAL has to be visible. The gun answers by name and leaves the
        # old value alone, so the control goes on showing the truth -- which
        # from the user's side is a spinbox that does nothing.
        #
        # The two shape floors are the RIG's own measurements now, not
        # constants, and the refusal names the figure it measured: these are
        # the sentences a gun actually sends, and the reason they have to reach
        # the log verbatim is that the number in them is the only explanation
        # of why a setting would not take. They survive a power cycle too --
        # the gun keeps the tallest LED and the largest LED pixel count it has
        # measured -- so this can arrive on a gun that has captured nothing
        # this session.
        for refusal in (
            "CAM: bhmax 6 is below the tallest LED this rig has been "
            "measured at (9) -- not set\n",
            "CAM: pxmax 40 is below the largest LED this rig has been "
            "measured at (214 px) -- not set\n",
            "CAM: armax 12 rejects blobs rounder than 2:1, which is most "
            "measured LEDs -- not set\n",
        ):
            link.src.q.put(refusal)
        root.update()
        time.sleep(1.0)
        root.update()
        refused_log = logbox_text(root)
        for want in ("tallest LED this rig", "largest LED this rig",
                     "rounder than 2:1"):
            if want not in refused_log:
                errs.append("the gun refusing a setting ('%s') was swallowed "
                            "instead of shown: %r" % (want, refused_log[-300:]))

        # ---- shape learning ------------------------------------------------
        # The capture that measures what a confirmed LED looks like on this
        # rig. Two buttons on the wiicam panel's action bar and a CSV; the
        # answer they work from arrives as thirteen separate lines, which is
        # where all the difficulty is.
        HFEATS = ("sz", "bw", "bh", "aspect", "area", "irel")

        def learn_lines(on, frames, led, rej, feats):
            """The gun's '~camlearn?' answer. The firmware sends every line
            every time, so a feature nothing was fed into comes back as 32
            zeros rather than not at all."""
            out = ["CAM: learn on=%d frames=%d led=%d rej=%d bins=32\n"
                   % (on, frames, led, rej)]
            for c in (0, 1):
                for f in HFEATS:
                    b = feats.get((c, f), [0] * 32)
                    out.append("CAM: hist c=%d f=%s %s\n"
                               % (c, f, " ".join(str(v) for v in b)))
            return out

        def settle(secs=1.2):
            t_end = time.time() + secs
            while time.time() < t_end:
                root.update()
                time.sleep(0.05)

        LEARN_LABELS = ("Learn LED shape", "Stop learning", "Shape capture ?")
        buttons = {str(w.cget("text")): w
                   for w in by_text(root, LEARN_LABELS + ("Shape CSV",),
                                    cls="Button")}
        learn_now = [buttons[k] for k in LEARN_LABELS if k in buttons]
        if not learn_now or "Shape CSV" not in buttons:
            errs.append("the wiicam camera panel has no shape-learning "
                        "controls: %s" % sorted(buttons))
        else:
            btn_learn = learn_now[0]
            # THE CAPTURE IS ARMED AT BOOT. Until the gun answers '~camlearn?'
            # this app does not know what it is doing, and it must not guess
            # OFF: guessing off puts "Learn LED shape" on a button whose press
            # sends on:1 to a capture that is already running -- a no-op the
            # firmware ignores (it clears on the off->on edge only) -- so the
            # counts the user was watching would not move and nothing on the
            # panel would say why.
            if str(btn_learn.cget("text")) != "Shape capture ?":
                errs.append("the shape-capture button assumed a state before "
                            "the gun had said one: %r"
                            % btn_learn.cget("text"))
            # And it follows the gun into 'on' with nobody having pressed it:
            # that is what "armed at boot" looks like from here.
            for ln in learn_lines(1, 12, 40, 1, {(0, "sz"): [1] * 32}):
                link.src.q.put(ln)
            settle(1.4)
            if str(btn_learn.cget("text")) != "Stop learning":
                errs.append("the button did not follow a gun that was already "
                            "capturing at connect: %r"
                            % btn_learn.cget("text"))
            # Put it back off so the click below is still an off->on edge.
            for ln in learn_lines(0, 12, 40, 1, {(0, "sz"): [1] * 32}):
                link.src.q.put(ln)
            settle(1.4)
            # Starting it on a gun that is NOT in full mode: only the size
            # histogram will fill, and size is the one feature already known
            # not to separate a window from an LED. That has to be said, or a
            # whole capture is spent finding it out. Fed as a reply rather
            # than clicked, so the panel and the gun agree on the mode -- the
            # 700 ms sync would otherwise put the radio back mid-test.
            link.src.q.put("CAM: blob fmt=1 ext=1 fullreg=85 bmin=2 bmax=9 "
                           "rtol=3 hwmax=-1 hwmin=-1 bn=4 brej=90 brrej=39 "
                           "bvalve=61 bframes=3900 bms=39000 bdrop=3 "
                           "br4=3600 br3=300 br2=100 br1=45 br0=25\n")
            settle(1.4)
            WIRE.clear()
            n_chunks = len(CHUNKS)
            btn_learn.invoke()
            drained(link)
            # Two lines, two writes: the 'ask back' must not ride the same
            # write as the command, or it lands on a gun still parsing it.
            if any("camlearn=on:1" in c and "camlearn?" in c
                   for c in CHUNKS[n_chunks:]):
                errs.append("'~camlearn=on:1' and '~camlearn?' went out in "
                            "one write")
            if "~camlearn=on:1" not in WIRE or "~camlearn?" not in WIRE:
                errs.append("'Learn LED shape' did not start a capture and "
                            "ask straight back: %s" % WIRE)
            if WIRE.index("~camlearn?") <= WIRE.index("~camlearn=on:1"):
                errs.append("'~camlearn?' went out before the command it "
                            "follows: %s" % WIRE)
            logged = logbox_text(root)
            if "SIZE" not in logged or "full detail" not in logged:
                errs.append("starting outside full mode did not warn that "
                            "only size would fill: %r" % logged[-300:])
            if str(btn_learn.cget("text")) != "Stop learning":
                errs.append("the button did not become a stop: %r"
                            % btn_learn.cget("text"))

            # The gun's own word wins: '~camreset' stops a capture from
            # anywhere, and a button still offering to stop one is a claim
            # that something is recording when nothing is.
            for ln in learn_lines(0, 40, 90, 3, {(0, "sz"): [1] * 32}):
                link.src.q.put(ln)
            settle(1.4)
            if str(btn_learn.cget("text")) != "Learn LED shape":
                errs.append("the button ignored the gun saying the capture "
                            "had stopped: %r" % btn_learn.cget("text"))

            # ---- the twelve lines, out of order and cut short --------------
            hs = link.hists
            seq0, held = hs.seq, hs.counts()
            L = learn_lines(1, 1200, 4800, 90, {(0, "sz"): [2] * 32})
            for ln in [L[0]] + L[:0:-1]:        # summary first, rows backwards
                link.src.q.put(ln)
            settle(1.4)
            if hs.counts() != (1200, 4800, 90) or hs.seq != seq0 + 1:
                errs.append("the twelve histogram lines did not land out of "
                            "order: %s" % (hs.counts(),))
            seq0, held = hs.seq, hs.counts()
            for ln in learn_lines(1, 7, 7, 7, {})[:-5]:   # five lines lost
                link.src.q.put(ln)
            settle(1.4)
            if hs.counts() != held or hs.seq != seq0:
                errs.append("a set cut short replaced the one already held: "
                            "%s" % (hs.counts(),))

            # ---- the CSV ---------------------------------------------------
            # A full capture this time: every feature fed, both classes. The
            # button asks the gun and waits, so the reply goes in AFTER the
            # click or it would be the set that was already held.
            full = {}
            for c in (0, 1):
                for i, f in enumerate(HFEATS):
                    full[(c, f)] = [(i + 1) * (c + 1) + b for b in range(32)]
            buttons["Shape CSV"].invoke()
            for ln in learn_lines(0, 3400, 13120, 268, full):
                link.src.q.put(ln)
            outdir = os.path.join(OUT, "calib_out")
            for _ in range(40):
                settle(0.1)
                if os.path.isdir(outdir) and os.listdir(outdir):
                    break
            got = sorted(os.listdir(outdir)) if os.path.isdir(outdir) else []
            if len(got) != 1 or not got[0].startswith("shape-001-"):
                errs.append("'Shape CSV' wrote no shape-001-TIME.csv: %s"
                            % got)
            else:
                with open(os.path.join(outdir, got[0])) as fh:
                    body = fh.read().strip().split("\n")
                want = ("class,feature,frames,led_blobs,rej_blobs,"
                        + ",".join("b%d" % i for i in range(32)))
                if body[0] != want:
                    errs.append("the CSV header is not the one a spreadsheet "
                                "is expected to open: %r" % body[0][:70])
                rows = [r.split(",") for r in body[1:]]
                if len(rows) != 12 or any(len(r) != 37 for r in rows):
                    errs.append("the CSV is not 12 rows of 37 columns: %d "
                                "rows, widths %s"
                                % (len(rows), sorted(set(len(r) for r in rows))))
                elif not all(r[2] == "3400" and r[3] == "13120"
                             and r[4] == "268" for r in rows):
                    errs.append("the counts do not repeat on every row, so a "
                                "sorted sheet stops saying what it measured")
                elif [r[0] for r in rows] != ["led"] * 6 + ["rej"] * 6:
                    errs.append("the class column is not named: %s"
                                % [r[0] for r in rows][:3])
                if "13120 LED blobs" not in logbox_text(root):
                    errs.append("the log did not say what was written: %r"
                                % logbox_text(root)[-200:])
                if "capture 1:" not in logbox_text(root):
                    errs.append("the log never said which NUMBER the capture "
                                "got, so there is nothing for the user to ask "
                                "for down a phone line: %r"
                                % logbox_text(root)[-200:])

            # ---- and the next one cannot land on it ------------------------
            # The way these get taken is press, look, press again -- so two
            # captures inside the same second is the normal case, and named
            # from the wall clock alone the second one silently overwrote the
            # first. That is a capture off a rig that has since been moved.
            first = sorted(os.listdir(outdir))
            buttons["Shape CSV"].invoke()
            for ln in learn_lines(0, 99, 98, 97, full):
                link.src.q.put(ln)
            for _ in range(40):
                settle(0.1)
                if len(os.listdir(outdir)) > len(first):
                    break
            both = sorted(os.listdir(outdir))
            if len(both) != len(first) + 1:
                errs.append("a second capture in the same second did not make "
                            "a second file: %s" % both)
            elif not both[1].startswith("shape-002-"):
                errs.append("the second capture is not number 2, so the files "
                            "do not sort in the order they were taken: %s"
                            % both)
            else:
                with open(os.path.join(outdir, both[0])) as fh:
                    kept = fh.read()
                if "3400" not in kept:
                    errs.append("the FIRST capture was overwritten by the "
                                "second -- the one thing the numbering exists "
                                "to prevent")

        # ---- the gate this rig can actually have ---------------------------
        # The panel used to print a height beside the control: 10, measured on
        # ONE bar with two LEDs per corner. A bar with five LEDs per cluster
        # makes blobs several times taller, so that hint was an instruction to
        # blind the gun, and its owner had nothing on screen to say why aim had
        # stopped working. What replaces it is the gun's own measurement, and
        # everything below is about it being asked for, read honestly, and
        # written only when a human says so.
        def gate_now():
            """The gate line: the fit answer, or whichever of the gate's own
            states has taken it. Found by its own wording -- it is the only
            label on the panel that ever carries any of them."""
            return [t for t in texts(root, [])
                    if t.startswith("gate fit") or "GATE MAY" in t
                    or "SHAPE GATE" in t or "INERT" in t]

        # Full mode first, and it is not housekeeping: the gate only judges a
        # box height, boxes only arrive in full mode, and this panel puts a
        # gate that CANNOT ACT ahead of anything a fit has to say about it. So
        # a fit answer is only on screen at all once the gate is live -- the
        # INERT half of that is driven further down.
        link.src.q.put("CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 "
                       "rtol=3 bhmax=10 pxmax=14 armax=20 hwmax=-1 hwmin=-1 "
                       "bn=4 brej=90 brrej=39 bvalve=61 bframes=3950 "
                       "bms=39500 bdrop=3 bsrej=44 bfar=12 bnear=6 br4=3600 "
                       "br3=300 br2=100 br1=45 br0=25\n")
        settle(0.8)

        fitb = {str(w.cget("text")): w
                for w in by_text(root, ("Measure the gate", "Apply"),
                                 cls="Button")}
        if sorted(fitb) != ["Apply", "Measure the gate"]:
            errs.append("the camera tab has no way to measure the gate: %s"
                        % sorted(fitb))
        else:
            b_meas, b_apply = fitb["Measure the gate"], fitb["Apply"]
            if not shown(b_meas) or not shown(b_apply):
                errs.append("the fit controls are not in front of the user, "
                            "where the gate they set is")
            if str(b_apply.cget("state")) != "disabled":
                errs.append("Apply is live before the gun has named anything "
                            "-- one click would write a gate off a capture "
                            "that has not happened")
            # Asking must change nothing on the gun. '?' and '=apply' differ
            # by one word on the wire and by everything else: the second one
            # writes flash.
            WIRE.clear()
            b_meas.invoke()
            drained(link)
            if ("~camfit?" not in WIRE
                    or any("camfit=apply" in w for w in WIRE)):
                errs.append("'Measure the gate' put %s on the wire" % WIRE)
            # An older gun has never heard of the command and answers nothing
            # at all. Silence has to turn into a sentence: left as "asking the
            # gun..." it is a spinner that never stops, and left blank it is a
            # panel that looks broken on a gun that is fine.
            # 2 s is the app's own deadline and the line is repainted on a
            # 500 ms clock, so anything under 2.5 would be racing the repaint
            # rather than testing the deadline. Waited for, not slept through.
            t_end = time.time() + 3.2
            while time.time() < t_end:
                root.update()
                if any("no answer" in t for t in gate_now()):
                    break
                time.sleep(0.05)
            if not any("no answer" in t for t in gate_now()):
                errs.append("a gun with no '~camfit' left the gate line "
                            "waiting for ever: %s" % gate_now())

            def fit_reply(*lines, **kw):
                """Feed one fit reply and wait for the gate line to show it.

                `want` is the marker that answer has to produce. The wait ends
                as soon as it appears rather than after a fixed span: there
                are half a dozen of these and the line is repainted on a
                500 ms clock, so fixed sleeps here cost seconds of the
                runner's timeout for nothing."""
                for ln in lines:
                    link.src.q.put(ln + "\n")
                want = kw.get("want")
                t_end = time.time() + kw.get("secs", 1.5)
                while time.time() < t_end:
                    root.update()
                    if want and want in "\n".join(gate_now()):
                        break
                    time.sleep(0.05)
                return "\n".join(gate_now())

            # How far along the measurement is, against the gun's own targets.
            # Without both numbers "not enough data" is a dead end -- there is
            # nothing on screen to say whether one more minute of play will do
            # it or whether the room has no stray light in it at all.
            said = fit_reply("CAM: fit ledn=120 ledmaxh=-1 straym=0 "
                             "strayminh=-1",
                             "CAM: fit NEEDS MORE LED DATA -- aim at the "
                             "bar with the capture on; 120 blobs so "
                             "far, 500 wanted", want="120 of 500")
            if "120 of 500" not in said or "0 of 20" not in said:
                errs.append("the panel does not show how far along the "
                            "measurement is: %r" % said)
            if str(b_apply.cget("state")) != "disabled":
                errs.append("Apply went live on a measurement that is not "
                            "finished")
            # The answer with no number in it. This rig's LEDs are as tall as
            # its stray light, so no size gate can separate them: the honest
            # reply is to say so and offer nothing, and an Apply button live
            # here would invite a height the gun has just explained it has not
            # got.
            said = fit_reply("CAM: fit ledn=900 ledmaxh=18 straym=40 "
                             "strayminh=15",
                             "CAM: fit NO SAFE GATE -- your LEDs reach 18 "
                             "tall and the stray light starts at 15, so a "
                             "size gate cannot tell them apart. Move the bar, "
                             "block the light, or use brighter LEDs.",
                             want="NO SAFE GATE")
            if "NO SAFE GATE" not in said:
                errs.append("'no safe gate' was not said plainly: %r" % said)
            if str(b_apply.cget("state")) != "disabled":
                errs.append("Apply is live on a rig the gun says cannot be "
                            "gated on size at all")
            # A real verdict. The number is the gun's PROPOSAL: it must be
            # readable, it must arm the second press, and it must not have
            # become the setting on the way past -- last[] and the spinbox
            # both still hold the 10 this gun is actually gating at.
            said = fit_reply("CAM: fit ledn=900 ledmaxh=7 straym=40 "
                             "strayminh=15",
                             "CAM: fit bhmax=11 (LEDs reach 7, stray starts "
                             "at 15)",
                             "CAM: fit not applied -- send camfit=apply to "
                             "set and save it", want="bhmax 11")
            if "bhmax 11" not in said:
                errs.append("the verdict never named the height it measured: "
                            "%r" % said)
            if str(b_apply.cget("state")) != "normal":
                errs.append("Apply stayed disabled on a verdict that named a "
                            "number, so the fit cannot be used at all")
            if link.last.get("bhmax") != 10:
                errs.append("the verdict overwrote the gun's live setting: "
                            "last[bhmax]=%r" % link.last.get("bhmax"))
            if str(gate_box["bhmax"].get()) != "10 rows":
                errs.append("the height spinbox followed a verdict instead of "
                            "the gun: %r" % gate_box["bhmax"].get())
            WIRE.clear()
            b_apply.invoke()
            drained(link)
            if "~camfit=apply" not in WIRE:
                errs.append("Apply did not send the apply form: %s" % WIRE)
            said = fit_reply("CAM: fit ledn=900 ledmaxh=7 straym=40 "
                             "strayminh=15",
                             "CAM: fit bhmax=11 (LEDs reach 7, stray starts "
                             "at 15)",
                             "CAM: fit applied and saved",
                             want="applied and saved")
            if "applied and saved" not in said:
                errs.append("the panel never confirmed the write: %r" % said)
            # A save that failed is a gate that works until the next power
            # cycle and then silently does not. Reported as success it is the
            # worst line on the panel.
            said = fit_reply("CAM: fit ledn=900 ledmaxh=7 straym=40 "
                             "strayminh=15",
                             "CAM: fit bhmax=11 (LEDs reach 7, stray starts "
                             "at 15)",
                             "CAM: fit applied but SAVE FAILED -- it will be "
                             "gone on the next power cycle", want="NOT SAVED")
            if "NOT SAVED" not in said:
                errs.append("a failed save was reported as a success: %r"
                            % said)
            # ---- the samples the gun set aside -----------------------------
            # 'CAM: fit 32 LED samples ignored -- they reach 31 tall, far
            # above the 7 the rest stop at, ...' begins with a NUMBER, so it
            # matched none of the parser's prefixes and fell through it and
            # the panel both: the one line that says the capture had the sun
            # in its LED class was the one line nothing read. A ceiling of 7
            # measured with a 31-tall blob in the class is the same number as
            # a clean one and not the same measurement, and the reader has no
            # other way to learn the difference.
            said = fit_reply("CAM: fit ledn=900 ledmaxh=7 straym=40 "
                             "strayminh=15",
                             "CAM: fit 32 LED samples ignored -- they reach "
                             "31 tall, far above the 7 the rest stop at, and "
                             "are almost certainly stray light learned while "
                             "the resolver locked on it",
                             "CAM: fit bhmax=11 (LEDs reach 7, stray starts "
                             "at 15)",
                             "CAM: fit not applied -- send camfit=apply to "
                             "set and save it", want="set aside")
            for want in ("32", "31", "set aside"):
                if want not in said:
                    errs.append("the contamination the gun reported is not on "
                                "the panel (%r missing): %r" % (want, said))
            if "bhmax 11" not in said:
                errs.append("the contamination clause replaced the verdict "
                            "instead of riding with it: %r" % said)
            # ---- the stored pair, and a stray edge of nothing ---------------
            # The stored pair is what the gun measures its REFUSALS against,
            # so before a capture finishes it is the only answer to "why will
            # it not take my number?". A stray edge of 0 is not a dark room:
            # it means no fit has ever applied on this gun, because camsave
            # records the LED edge alone. It has to read the same as a gun too
            # old to send the field at all -- to the reader they are one
            # thing, nothing measured it -- and never as "stray at 0".
            said = fit_reply("CAM: fit ledn=120 ledmaxh=-1 straym=0 "
                             "strayminh=-1",
                             "CAM: fit STORED ledmaxh=9 strayminh=0 "
                             "ledmaxpx=214 -- from an earlier capture on this "
                             "gun",
                             "CAM: fit NEEDS MORE LED DATA -- aim at the "
                             "bar with the capture on; 120 blobs so "
                             "far, 500 wanted", want="stored")
            if "9 tall" not in said or "not recorded" not in said:
                errs.append("a stored LED edge with no stray behind it was "
                            "not reported as unrecorded: %r" % said)
            if "stray 0" in said:
                errs.append("a stray edge of 0 is shown as a measurement of "
                            "zero, which is a dark room and not the truth: "
                            "%r" % said)
            said = fit_reply("CAM: fit ledn=120 ledmaxh=-1 straym=0 "
                             "strayminh=-1",
                             "CAM: fit STORED ledmaxh=9 strayminh=15 -- from "
                             "an earlier capture on this gun",
                             "CAM: fit NEEDS MORE LED DATA -- aim at the "
                             "bar with the capture on; 120 blobs so "
                             # The marker is the VALUE, not the word
                             # "stored": the line before this one carried
                             # that too, and waiting for it would have read
                             # the previous window's answer.
                             "far, 500 wanted", want="stray 15")
            if "9 tall" not in said or "not recorded" in said:
                errs.append("an older gun's two-field STORED line lost the "
                            "stray edge it did send: %r" % said)
            # A reply whose numbers did not survive the wire. Every figure the
            # gun did not send reads '?': a blank where a measurement belongs
            # is indistinguishable from a measurement of nothing, and the word
            # None on a panel is a traceback nobody can act on.
            said = fit_reply("CAM: fit ledn=x ledmaxh= straym= strayminh=",
                             "CAM: fit NO SAFE GATE -- your LEDs reach 18 "
                             "tall and the stray light starts at 15, so a "
                             "size gate cannot tell them apart.",
                             want="reach ?")
            if "?" not in said or "None" in said:
                errs.append("a fit reply with unreadable numbers put %r on "
                            "the panel" % said)

        # ---- the shape gate working too hard --------------------------------
        # bsrej counts every blob the shape gate refused, SINCE BOOT, and the
        # warning has to come off a rate over a window: bvalve read raw pinned
        # "SIZE WINDOW TOO TIGHT" on the panel for a whole power cycle, and a
        # delta that re-based itself on every draw showed its count for about
        # 16 ms in every second.
        #
        # What is counted is the UNEXPLAINED part -- refusals minus the ones
        # the resolver vouched for, bfar (placed far from every corner) and
        # bnear (sitting where a missing LED had to be). A 92 s daylight log
        # settled that: with bhmax:8 the gate held four straight seconds at
        # exactly 1.00 refusals per frame while the sun sat in one of the four
        # slots, and a raw threshold of one per frame fired through the whole
        # stretch -- on a gate doing precisely the job it exists for. Over that
        # session bsrej climbed 829 and bfar 730, 88% vouched for.
        #
        # So every window below states all three counters, and each case is a
        # different answer to "who accounted for these?".
        gw = {"br4": 3600, "bframes": 3950, "bsrej": 44, "bfar": 12,
              "bnear": 6}

        def gate_window(d_srej, d_far, d_near, frames, bh=10, px=0, ar=0,
                        fmt=2, with_bfar=True, secs=2.5):
            """One closed WINDOW of `frames`, and what the gate line says after
            it. The counters are cumulative on the wire and every rule here is
            about how much they MOVED, so this takes deltas and keeps the
            totals. Only br4 moves among the corner counts, so the window is
            exactly `frames` frames.

            Three clocks stand between the reply and the answer: a 50 ms
            pump, a 700 ms poll that closes the window, and a 500 ms repaint
            of the gate line. So the wait ends once the readout names THIS
            window -- its own frame count and its own raw shape count, which
            is proof the poll closed it -- AND the gate line has either
            changed or had a full repaint period to change in.

            Waiting only for the expected outcome was tried and is a trap:
            when a window's verdict equals the one before it the marker is
            already on screen, the wait returns before the repaint, and the
            assertion reads the previous window's line and passes on
            anything. It cost this file a real regression -- with the count
            reverted to the raw one, the window that has to stay silent went
            on passing.

            The "changed" half is measured from the line as it stood WHEN THE
            WINDOW CLOSED, not from before the reply was fed. The limits and
            the format reach last[] a tick before the poll closes the window,
            so the line gets repainted once in between with this window's
            gate and the LAST window's counts -- a real difference, and not
            the one being waited for.
            """
            gw["br4"] += frames
            gw["bframes"] += frames
            gw["bsrej"] += d_srej
            gw["bfar"] += d_far
            gw["bnear"] += d_near
            # Which limits are set and which format the gun is in are part
            # of the fixture, not a constant: the shape gate is bhmax OR
            # pxmax OR armax, it only acts in full mode, and every rule about
            # this warning depends on both.
            link.src.q.put(
                "CAM: blob fmt=%d ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 "
                "bhmax=%d pxmax=%d armax=%d hwmax=-1 hwmin=-1 bn=4 brej=90 "
                "brrej=39 bvalve=61 bframes=%d bms=%d bdrop=3 bsrej=%d %s"
                "bnear=%d br4=%d br3=300 br2=100 br1=45 br0=25\n"
                % (fmt, bh, px, ar,
                   gw["bframes"], gw["bframes"] * 10, gw["bsrej"],
                   ("bfar=%d " % gw["bfar"]) if with_bfar else "",
                   gw["bnear"], gw["br4"]))
            # The separator is load-bearing: "10 shape" is a substring of
            # "110 shape", and without the comma this marker matched the
            # previous window's readout instantly -- the assertions then ran
            # against a panel that had not been fed yet.
            mark = ("last %d frames" % frames, ", %d shape" % d_srej)
            t_end = time.time() + secs
            closed_at = at_close = None
            while time.time() < t_end:
                root.update()
                if closed_at is None and any(all(m in t for m in mark)
                                             for t in texts(root, [])):
                    closed_at = time.time()
                    at_close = "\n".join(gate_now())
                if closed_at is not None:
                    got = "\n".join(gate_now())
                    # 0.55: one repaint period plus the 50 ms this loop can
                    # take to notice the window closed.
                    if got != at_close or time.time() - closed_at > 0.55:
                        return got
                time.sleep(0.05)
            return "\n".join(gate_now())

        # 300 unexplained over 900 frames -- a third of a blob a frame, the
        # room being carried and not the bar being eaten. Studio warned here
        # until this batch, at 0.25 a frame, while pical said nothing until
        # 2.0: one gate refusing a third of everything the camera saw, two
        # tools, two answers. This case pins the number itself.
        got = gate_window(300, 0, 0, 900)
        if "SHAPE GATE REFUSING" in got:
            errs.append("a gate refusing a third of a blob per frame warned "
                        "in Studio -- that is the eight-fold disagreement "
                        "with pical, back again: %r" % got)
        # 01:32:37 to 01:32:41, off the real log: exactly one refusal per
        # frame, and the resolver vouched for every one of them. br4 was zero
        # and br3 was every frame -- the sun holding a slot, the gate refusing
        # it, the gun running on three real corners and a reconstructed
        # fourth. The raw rule fired 14 times replaying that session; this is
        # the case that has to stay silent, and nothing may reach the log.
        log_before = len(logbox_text(root))
        got = gate_window(50, 50, 0, 50)
        if "SHAPE GATE REFUSING" in got:
            errs.append("one refusal a frame, every one of them vouched for "
                        "by the resolver, still warned -- that is the sun in "
                        "a slot and the gate doing its job: %r" % got)
        # The log grows on every reply -- each CAM: line is echoed into it --
        # so what is checked is that the explanation did not fire, not that
        # nothing was written. It is a once-only sentence: spent here, it
        # would never appear for the gate that really was eating the bar.
        if "could not account for" in logbox_text(root)[log_before:]:
            errs.append("a gate carrying the room spent the once-only "
                        "explanation: %r"
                        % logbox_text(root)[log_before:][:200])
        # A gate set for a different LED bar refuses the bar itself: the
        # resolver then has too few points to lock and vouches for nothing, so
        # bfar and bnear stay flat while bsrej runs away. THAT is the shape of
        # a gate eating the bar, and it is what the warning is for.
        got = gate_window(150, 0, 0, 50)
        if "SHAPE GATE REFUSING" not in got:
            errs.append("150 refusals in 50 frames, not one of them "
                        "explained by the resolver, and the panel said "
                        "nothing: %r" % got)
        if "150 unexplained" not in got:
            errs.append("the warning does not say how many refusals were "
                        "unaccounted for, which is the whole measurement it "
                        "is made of: %r" % got)
        if "bhmax:0" not in got:
            errs.append("the shape-gate warning does not say how to turn the "
                        "gate off: %r" % got)
        # The sentence that will not fit beside the numbers goes to the log,
        # once -- and it has to carry BOTH figures, because "150 the resolver
        # could not account for, 0 it could" is the whole argument for
        # believing the warning.
        heavy_log = logbox_text(root)
        if "150" not in heavy_log or "(0 more" not in heavy_log:
            errs.append("the log did not say how many refusals the resolver "
                        "could and could not account for: %r"
                        % heavy_log[-320:])
        if "bhmax:0" not in heavy_log:
            errs.append("the log never carried the escape hatch either: %r"
                        % heavy_log[-320:])
        # Partly vouched for: 100 refused, 60 of them placed far from every
        # corner, so 40 unexplained over 50 frames -- 0.8 a frame, under the
        # line. This is also the CLEAR-DOWN of the window above: bsrej is now
        # a big number that stays big, and a warning that reads it raw never
        # comes off again, which is the exact bug bvalve shipped with.
        got = gate_window(100, 60, 0, 50)
        if "SHAPE GATE REFUSING" in got:
            errs.append("0.8 unexplained a frame warned, or the warning "
                        "latched on a since-boot total instead of clearing "
                        "when the gate stopped: %r" % got)
        if "gate fit" not in got:
            errs.append("nothing came back on the gate line once the warning "
                        "cleared, so the fit is unreachable after one: %r"
                        % got)
        # The same gate with the vouching lagging behind: 110 refused, 50
        # explained, 60 left over 50 frames -- 1.2 a frame, over the line.
        got = gate_window(110, 50, 0, 50)
        if "SHAPE GATE REFUSING" not in got or "60 unexplained" not in got:
            errs.append("1.2 unexplained refusals a frame did not warn: %r"
                        % got)
        # bfar climbing FASTER than bsrej -- the resolver labelling positions
        # with no gate rejection behind them. The rate is deliberately one the
        # RAW rule would have warned on, 120 refusals in 50 frames: over-
        # vouching has to end in silence, not in "well, some of it was
        # explained". The clamp itself cannot be seen from here -- a negative
        # count never reaches the threshold either -- so the second assertion
        # is a guard for the day this figure is printed somewhere that is not
        # behind that comparison.
        got = gate_window(120, 200, 0, 50)
        if "SHAPE GATE REFUSING" in got:
            errs.append("more vouched for than refused still warned: %r" % got)
        if "(-" in got:
            errs.append("a negative unexplained count reached the panel, "
                        "which reads as a gate refusing minus eighty blobs: "
                        "%r" % got)
        # bnear is its own warning and it outranks this one. The numbers are
        # deliberately a heavy window as well -- 200 refused, 190 of them
        # unexplained -- because a bnear case that could not have raised the
        # heavy warning proves nothing about which of the two wins.
        got = gate_window(200, 0, 10, 50)
        if "GATE MAY BE TAKING REAL LEDs" not in got:
            errs.append("bnear moving raised no warning of its own: %r" % got)
        if "SHAPE GATE REFUSING" in got:
            errs.append("the heavy-rejection warning took the line from the "
                        "false-negative meter, which is the only number here "
                        "that says a gate is WRONG rather than busy: %r" % got)
        # The shape gate is bhmax OR pxmax OR armax, and the advice has to
        # name the one that is actually set. "bhmax:0 turns it off" to
        # somebody gating on pxmax is advice that changes nothing at all, and
        # nothing on the panel would ever tell them so.
        got = gate_window(150, 0, 0, 50, bh=0, px=14)
        if "SHAPE GATE REFUSING" not in got or "pxmax:0" not in got:
            errs.append("a pxmax gate refusing heavily was not reported, or "
                        "was reported as bhmax's doing: %r" % got)
        if "bhmax:0" in got:
            errs.append("the warning tells the reader to type bhmax:0 while "
                        "their gate is pxmax: %r" % got)
        # Outside full mode the gun sends no box, the shape gate stands down
        # and bsrej cannot move -- so a rate computed there is a rate about
        # nothing. pical gates its own warning on the gate being on AND
        # acting; this one did not, and would have warned about a gate that
        # was not running. (The INERT state is what the line says instead,
        # which is the truth about that gun.)
        got = gate_window(150, 0, 0, 50, bh=10, fmt=1)
        if "SHAPE GATE REFUSING" in got:
            errs.append("a gate that cannot act at all was reported as "
                        "refusing heavily: %r" % got)
        # And with every limit off there is no gate to warn about and nothing
        # the advice could name.
        got = gate_window(150, 0, 0, 50, bh=0, px=0, ar=0)
        if "SHAPE GATE REFUSING" in got:
            errs.append("the warning fired with every shape limit switched "
                        "off: %r" % got)
        # A gun that predates bfar sends no such key at all. The count then
        # degrades to the raw one -- which is what this warning was before the
        # log corrected it -- rather than blanking, raising, or silently never
        # firing again on the gun most likely to need it. LAST in this block:
        # popping the key resets this test's own reference for it, so a window
        # fed afterwards would see the counter arrive back as one huge delta.
        link.last.pop("bfar", None)
        got = gate_window(300, 0, 0, 50, bh=10, with_bfar=False)
        if "SHAPE GATE REFUSING" not in got:
            errs.append("on a gun with no bfar counter the warning went "
                        "silent instead of falling back to the raw count: %r"
                        % got)
        if "unexplained" not in got:
            errs.append("the fallback dropped the count out of the sentence: "
                        "%r" % got)
        # ---- and a warning speaks for the present or not at all ------------
        # The verdict is computed when a window of frames CLOSES. On a gun
        # that has stopped answering -- unplugged, camera dead, port taken by
        # another app -- nothing recomputes it, and the last warning would sit
        # on the line being read as current. That is the latch this panel has
        # already shipped twice: bvalve read raw, and a delta that never
        # expired. Measured off the warning the window above just raised, and
        # with the staleness bound shortened rather than slept through: eight
        # seconds of nothing, in a file that already runs 50 of the runner's
        # 60, buys exactly one assertion.
        gun_studio.GATE_WARN_STALE_S = 0.5
        try:
            settle(0.9)                 # no reply fed: the gun has gone quiet
            quiet = "\n".join(gate_now())
        finally:
            gun_studio.GATE_WARN_STALE_S = 8.0
        if "SHAPE GATE REFUSING" in quiet:
            errs.append("the warning went on claiming the present after the "
                        "gun stopped sending frames: %r" % quiet)
        if "gate fit" not in quiet:
            errs.append("nothing came back on the gate line once the warning "
                        "went stale: %r" % quiet)

        # ---- a capture thrown away by a setting -----------------------------
        # Changing the sensitivity or either sensor threshold changes what the
        # sensor REPORTS, so the gun clears the shape capture and says which
        # setting did it. What it does not say is the consequence: the capture
        # is empty now and the fit will answer NEEDS MORE LED DATA until the
        # bar and the room have been swept again. Said once, because these
        # fire on every step of a spinbox somebody is holding down and eight
        # of them would push the fit verdict out of a six-line log.
        cleared_before = len(logbox_text(root))
        link.src.q.put("CAM: learn cleared -- sensitivity changed from 1 to "
                       "2, and a capture spanning both is not a measurement "
                       "of either\n")
        settle(0.35)
        cleared = logbox_text(root)[cleared_before:]
        if "sensitivity changed from 1 to 2" not in cleared:
            errs.append("the gun's own reason for clearing the capture never "
                        "reached the log: %r" % cleared)
        if "NEEDS MORE LED DATA" not in cleared:
            errs.append("nothing said what the cleared capture means for the "
                        "fit, which is the whole consequence: %r" % cleared)
        link.src.q.put("CAM: learn cleared -- hwmax changed\n")
        settle(0.35)
        twice = logbox_text(root)[cleared_before:]
        if twice.count("swept again") > 1:
            errs.append("the explanation repeats on every cleared capture, "
                        "and a spinbox held down would fill the log with it")

        # ---- SET BUT INERT --------------------------------------------------
        def wait_gate(want, absent=False, secs=1.5):
            """Spin until the gate line does (or does not) carry `want`.

            Every check below is a state CHANGE -- INERT appearing where it
            was not, or going once the gun is back in full mode -- so waiting
            for the change is both faster than a fixed sleep and stricter
            than one: a sleep one tick short reads the state before it.
            """
            t_end = time.time() + secs
            while time.time() < t_end:
                root.update()
                got = "\n".join(gate_now())
                if (want in got) != absent:
                    return got
                time.sleep(0.05)
            return "\n".join(gate_now())

        # A limit the gun is carrying and cannot apply, because the box it
        # judges is only reported in full mode. This is the state that gutted
        # the feature: 'cam?' reports the limits and fmt on one line and never
        # puts them together, so a gate doing nothing at all looked exactly
        # like one that was working, and the saved format used to come back
        # clamped -- every saved gate loaded inert while the tools called it
        # active. Derived from last[] rather than only believed when the gun
        # says it, because an older gun says nothing and is just as inert.
        link.src.q.put("CAM: blob fmt=1 ext=1 fullreg=85 bmin=2 bmax=9 "
                       "rtol=3 bhmax=10 pxmax=0 armax=0 hwmax=-1 hwmin=-1 "
                       "bn=4 brej=90 brrej=39 bvalve=61 bframes=7000 "
                       "bms=70000 bdrop=3 bsrej=2000 bfar=372 bnear=16 "
                       "br4=6300 br3=300 br2=100 br1=45 br0=25\n")
        said = wait_gate("INERT")
        if "INERT" not in said or "bhmax" not in said:
            errs.append("a gate set outside full mode reads as a working "
                        "gate: %r" % said)
        if "full detail" not in said:
            errs.append("the INERT state does not say what to do about it -- "
                        "'full detail' is the control that fixes it: %r"
                        % said)
        # ...and it is not the HEIGHT gate's state, it is the shape gate's.
        # With bhmax off and pxmax set, the gate is still on and still inert,
        # and the line has to name the limit that is actually set: telling
        # somebody to fix bhmax when their gate is pxmax is advice they cannot
        # act on and cannot tell was wrong.
        link.src.q.put("CAM: blob fmt=1 ext=1 fullreg=85 bmin=2 bmax=9 "
                       "rtol=3 bhmax=0 pxmax=14 armax=0 hwmax=-1 hwmin=-1 "
                       "bn=4 brej=90 brrej=39 bvalve=61 bframes=7010 "
                       "bms=70100 bdrop=3 bsrej=2000 bfar=372 bnear=16 "
                       "br4=6300 br3=300 br2=100 br1=45 br0=25\n")
        said = wait_gate("pxmax")
        if "INERT" not in said or "pxmax" not in said:
            errs.append("a pxmax-only gate outside full mode was reported as "
                        "working, or reported as bhmax's problem: %r" % said)
        if "bhmax" in said:
            errs.append("the INERT line named a limit that is switched off: "
                        "%r" % said)
        # The gun's own word beats this app's memory of the last click. A
        # format changed from a serial terminal or from pical on the same gun
        # leaves last["fmt"] stale, and the gun names the format it is really
        # in inside the INERT line itself -- with a COLON, which the key/value
        # sweep cannot see. Fed here with last["fmt"] deliberately wrong, and
        # with pxmax as the limit, because the gun sends this line for that
        # one too now.
        link.src.q.put("CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 "
                       "rtol=3 bhmax=10 pxmax=14 armax=20 hwmax=-1 hwmin=-1 "
                       "bn=4 brej=90 brrej=39 bvalve=61 bframes=7050 "
                       "bms=70500 bdrop=3 bsrej=2000 bfar=372 bnear=16 "
                       "br4=6300 br3=300 br2=100 br1=45 br0=25\n")
        if "INERT" in wait_gate("INERT", absent=True):
            errs.append("the INERT state stuck after the gun came back into "
                        "full mode: %s" % gate_now())
        link.src.q.put("CAM: pxmax 14 set but INERT -- the shape gate needs "
                       "fmt:2 and this gun is in fmt:1\n")
        wait_gate("INERT")
        if link.last.get("fmt") != 1:
            errs.append("the gun said which format it was really in and the "
                        "front end kept its own: last[fmt]=%r"
                        % link.last.get("fmt"))
        if "INERT" not in "\n".join(gate_now()):
            errs.append("the gun said the gate was inert and the panel went "
                        "on showing it as live: %s" % gate_now())
        if link.last.get("pxmax") != 14:
            errs.append("the INERT line's own '14' was read as a setting: "
                        "last[pxmax]=%r" % link.last.get("pxmax"))
        # An apply now moves the gun INTO full mode and saves it there, and
        # says so with 'switched to fmt:2'. That line is the gun correcting
        # this front end's idea of the format -- also with a colon -- and the
        # INERT state it was showing a moment ago has to go. The line it
        # replaced said the gate was inert and left the user to fix it; both
        # tools had grown a sentence telling them to press Save as well, for a
        # command called 'apply' that did not survive a reboot.
        for ln in ("CAM: fit ledn=900 ledmaxh=7 straym=40 strayminh=15",
                   "CAM: fit bhmax=11 (LEDs reach 7, stray starts at 15)",
                   "CAM: fit switched to fmt:2 -- the shape gate needs it and "
                   "the ceiling was measured in it",
                   "CAM: fit applied and saved"):
            link.src.q.put(ln + "\n")
        wait_gate("INERT", absent=True)
        if link.last.get("fmt") != 2:
            errs.append("the gun switched itself to full mode and the panel "
                        "kept the old format: last[fmt]=%r"
                        % link.last.get("fmt"))
        if not link.fit.switched:
            errs.append("the format switch was not held on the answer")
        if "INERT" in "\n".join(gate_now()):
            errs.append("the panel still calls the gate inert after the gun "
                        "switched into the format it needs: %s" % gate_now())
        if "but NOT the format" in logbox_text(root):
            errs.append("the log still tells the user an apply does not save "
                        "the format -- the gun saves it, and that advice "
                        "teaches a habit that is now wrong")
        # bnear counts SHAPE-gate rejections only now, so when it speaks the
        # shape gate IS the culprit and the only open question is which limit
        # did it. The line used to say the height gate was innocent whenever
        # it was inert, which was true while bnear could also come from the
        # size window and is wrong now: these blobs were thrown away by the
        # shape gate while it was still acting.
        link.src.q.put("CAM: blob fmt=1 ext=1 fullreg=85 bmin=2 bmax=9 "
                       "rtol=3 bhmax=0 pxmax=14 armax=0 hwmax=-1 hwmin=-1 "
                       "bn=4 brej=90 brrej=39 bvalve=61 bframes=8000 "
                       "bms=80000 bdrop=3 bsrej=2000 bfar=372 bnear=40 "
                       "br4=7200 br3=300 br2=100 br1=45 br0=25\n")
        settle(1.4)
        both = "\n".join(gate_now())
        if "GATE MAY BE TAKING REAL LEDs" not in both:
            errs.append("the false-negative meter went quiet just because the "
                        "gate was inert: %r" % both)
        if "pxmax:0" not in both:
            errs.append("the false-negative warning does not name the limit "
                        "that threw the blobs away: %r" % both)
        if "bhmax:0" in both:
            errs.append("the warning tells the reader to switch off a limit "
                        "that is already off: %r" % both)

        subprocess.run("import -window root /tmp/studio_camera_wiicam.png",
                       shell=True, capture_output=True)

    subprocess.run("import -window root /tmp/studio.png", shell=True, capture_output=True)

    # ---- R8 follow-up: bcold folds into "two or fewer, or no lock" --------
    # A resolver with no model yet, or one that refused the offered seed,
    # must not read as "saw all four" just because the sensor still reported
    # four blobs. Values are deliberately far above anything used earlier in
    # this file, so the delta below is against a KNOWN baseline regardless of
    # what board branch ran above.
    link.src.q.put(
        "CAM: blob fmt=1 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 bhmax=0 "
        "pxmax=0 armax=0 hwmax=-1 hwmin=-1 bn=4 brej=5000 brrej=3000 "
        "bvalve=5000 bframes=90000 bms=900000 bdrop=3 bsrej=5000 bfar=1000 "
        "bnear=500 bsv=100 bcold=1000 br4=50000 br3=5000 br2=2000 br1=1000 "
        "br0=500\n")
    root.update(); time.sleep(1.5); root.update()
    link.src.q.put(
        "CAM: blob fmt=1 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 bhmax=0 "
        "pxmax=0 armax=0 hwmax=-1 hwmin=-1 bn=4 brej=5010 brrej=3005 "
        "bvalve=5010 bframes=90100 bms=901000 bdrop=3 bsrej=5010 bfar=1005 "
        "bnear=505 bsv=105 bcold=1040 br4=50060 br3=5005 br2=2001 br1=1001 "
        "br0=501\n")
    root.update(); time.sleep(1.5); root.update()
    joined = "\n".join(texts(root, []))
    if "39% two or fewer, or no lock" not in joined:
        errs.append("bcold was not folded into the two-or-fewer bucket, or "
                    "the bucket was not renamed: %s"
                    % [t for t in texts(root, []) if "saw all four LEDs" in t])

    # ---- R8: shutdown must turn the resolver back on ------------------------
    # A crash or a window close mid-sweep must not strand the gun at res:0.
    # quit() (not destroy()) stops mainloop() without tearing down widgets, so
    # the still-pending tick() callback simply never fires.
    WIRE.clear()
    root.quit()                             # trips gun_studio.main()'s finally
    time.sleep(0.3)
    if "~cam=res:2" not in WIRE:
        errs.append("shutdown did not turn the resolver back on (R8): %s" % WIRE)

    print("wire during toggles: %s" % hid)
    print("studio render: %s" % ("OK" if not errs else "FAILED -- %s" % errs[0]))
    time.sleep(0.2)

# driver() ends by destroying the window, which is what makes gun_studio.main()
# return below (through its own finally, the shutdown path R8 checks) -- so the
# thread that reports the verdict has to be joined before the process exits,
# or main() returning first would tear the process down mid-report.
_driver_thread = threading.Thread(target=driver, daemon=True)
_driver_thread.start()
gun_studio.main()
_driver_thread.join(timeout=10)
sys.stdout.flush()             # os._exit() skips the normal flush on the way out
os._exit(1 if errs else 0)
