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

class FakeSerial:
    def write(self, b):
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

    WIRE.clear()
    fire("<F9>"); time.sleep(0.3)          # freeze
    fire("<F9>"); time.sleep(0.3)          # release
    fire("<F9>"); time.sleep(0.3)          # freeze again, and LEAVE it frozen
    hid = [w for w in WIRE if w.startswith("~aimhid=")]
    if hid != ["~aimhid=0", "~aimhid=1", "~aimhid=0"]:
        errs.append("F9 did not toggle cleanly: %s" % hid)

    # The "remember" semantics: a temporary release must not erase the user's
    # choice, or returning from calibration would silently un-freeze them.
    L = gun_studio.Link(); L.src = FakeSource("X")
    WIRE.clear()
    L.pointer(False)                       # user freezes
    L.pointer(True, remember=False)        # calibration borrows the cursor
    if not L.hid_on is False:
        errs.append("a temporary release overwrote the user's choice")
    L.pointer(L.hid_on)                    # ...and we come back
    # remember=False is app-side only: Studio must come BACK to the user's choice
    if WIRE != ["~aimhid=0", "~aimhid=1", "~aimhid=0"]:
        errs.append("remember=False sequence wrong on the wire: %s" % WIRE)
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
               "CAM: fit NEEDS MORE LED DATA -- run a calibration with the "
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
                "CAM: fit NEEDS MORE LED DATA -- run a calibration with the "
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
               "CAM: fit NEEDS MORE LED DATA -- run a calibration with the "
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
            "bdrop=3 bsrej=0 bfar=0 bnear=0 br4=800 br3=150 br2=40 br1=10 "
            "br0=0\n",
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
            root.update()
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
            root.update()
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
            front = {"sensitivity": by_text(root, ("Default",), cls="Button"),
                     "blob detail": by_text(root, ("full detail",)),
                     "learn": by_text(root, ("Learn LED shape",),
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
                    for _ in range(12):
                        WIRE.clear()
                        sp.invoke(direction)
                        root.update()
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
                if grown_need > grown_have:
                    errs.append("the wiicam camera panel fits with less than "
                                "%dpx to spare: %dpx of a tab area that stops "
                                "growing at %dpx" % (MARGIN, need, grown_have))
                # ...and the newest row on the panel was ON it while that
                # was measured. The fit row is two buttons and a line -- 21 px
                # of a tab area that stops growing at 313 -- and it was paid
                # for by taking the gaps out from between every other row.
                # Measured here with the disclosure open and both readout
                # lines wrapped, the panel asks for 286 px. A margin taken
                # against a panel that had quietly dropped the row would prove
                # nothing about the layout anybody actually sees.
                if not any(shown(w) for w
                           in by_text(root, ("Measure the gate",),
                                      cls="Button")):
                    errs.append("the fit row was not on the panel that was "
                                "just measured (%dpx of %dpx), so the margin "
                                "says nothing about it" % (need, grown_have))
        else:
            errs.append("could not find the wiicam camera panel to measure")

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

        buttons = {str(w.cget("text")): w
                   for w in by_text(root, ("Learn LED shape", "Stop learning",
                                           "Shape CSV"), cls="Button")}
        if "Learn LED shape" not in buttons or "Shape CSV" not in buttons:
            errs.append("the wiicam camera panel has no shape-learning "
                        "controls: %s" % sorted(buttons))
        else:
            btn_learn = buttons["Learn LED shape"]
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
            btn_learn.invoke()
            root.update()
            if "~camlearn=on:1" not in WIRE or "~camlearn?" not in WIRE:
                errs.append("'Learn LED shape' did not start a capture and "
                            "ask straight back: %s" % WIRE)
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
            root.update()
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
                             "CAM: fit NEEDS MORE LED DATA -- run a "
                             "calibration with the capture on; 120 blobs so "
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
            root.update()
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
                             "CAM: fit NEEDS MORE LED DATA -- run a "
                             "calibration with the capture on; 120 blobs so "
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
                             "CAM: fit NEEDS MORE LED DATA -- run a "
                             "calibration with the capture on; 120 blobs so "
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
    print("wire during toggles: %s" % hid)
    print("studio render: %s" % ("OK" if not errs else "FAILED -- %s" % errs[0]))
    time.sleep(0.2)
    os._exit(1 if errs else 0)

threading.Thread(target=driver, daemon=True).start()
gun_studio.main()
