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
    time.sleep(3.0)
    root = _tk._default_root
    if root is None:
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
            if "not recommended" not in adv_text:
                errs.append("armax sits among the Advanced settings with "
                            "nothing marking it as the one that is wrong")
            # And it closes again, or a disclosure is just a slow way of
            # showing everything.
            adv_btn.invoke()
            root.update()
            if any(shown(w) for w in hidden["armax"]):
                errs.append("the disclosure would not close again")
            adv_btn.invoke()
            root.update()

        # ---- the shape gate ------------------------------------------------
        # The gun REFUSES a bhmax of 1..7, a pxmax of 1..11 and an armax of
        # 1..15 outright: it answers by name and leaves the old value alone,
        # so a spinbox stepping by 1 from 0 would spend its first clicks doing
        # nothing visible. All three step through a fixed LIST for that
        # reason, and the list is what is being tested -- every rung, walked
        # in both directions, has to be a value the gun accepts.
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
            # bhmax first, because it is the one the user is now steered to:
            # the firmware refuses 1..7 and 8 is one step above the tallest
            # LED ever measured, so a ladder that could emit 7 would be a gate
            # eating corners the moment somebody held an arrow down.
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
                    errs.append("the %s ladder can emit %s, which the gun "
                                "refuses -- the arrows would do nothing and "
                                "say nothing (emitted %s)"
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
        for refusal in (
            "CAM: bhmax below 8 would reject measured LEDs -- not set\n",
            "CAM: pxmax below 12 would reject measured LEDs -- not set\n",
            "CAM: armax below 16 (2:1) would reject measured LEDs -- not "
            "set\n",
        ):
            link.src.q.put(refusal)
        root.update()
        time.sleep(1.0)
        root.update()
        refused_log = logbox_text(root)
        for want in ("bhmax below 8", "pxmax below 12", "armax below 16"):
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

        subprocess.run("import -window root /tmp/studio_camera_wiicam.png",
                       shell=True, capture_output=True)

    subprocess.run("import -window root /tmp/studio.png", shell=True, capture_output=True)
    print("wire during toggles: %s" % hid)
    print("studio render: %s" % ("OK" if not errs else "FAILED -- %s" % errs[0]))
    time.sleep(0.2)
    os._exit(1 if errs else 0)

threading.Thread(target=driver, daemon=True).start()
gun_studio.main()
