#!/usr/bin/env python3
"""Renders gun_studio far enough to run its tick loop and draw every widget, and
drives the POINTER TOGGLE to assert the '~aimhid=' lines that reach the wire --
the toggle is the only way back from a frozen cursor.
"""
import sys, os, time, threading, subprocess, queue
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
        for ln in (
            "AIM: pong board=rp2040-wiicam\n",
            "CAM: board=rp2040-wiicam sens=1 lens=0 ext=1 bmin=2 bmax=9 "
            "res=2 dash=0\n",
            "CAM: blob ext=1 bmin=2 bmax=9 bn=4 brej=7 bframes=900 bdrop=3 "
            "br4=800 br3=150 br2=40 br1=10 br0=0\n",
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
        cands = []
        def collect(w):
            t = texts(w, [])
            if (any("ambient light" in s for s in t)
                    and any("wiicam sensitivity" in s for s in t)):
                cands.append(w)
            for c in w.winfo_children():
                collect(c)
        collect(root)
        wii = min(cands, key=lambda w: w.winfo_reqheight()) if cands else None
        if wii is not None and nb is not None:
            need = wii.winfo_reqheight()
            # The tab PAGE's own allocated height is the content area exactly.
            # Deriving it from the notebook minus a guessed tab-strip height
            # was off by a pixel or two and reported false failures.
            page = nb.nametowidget(nb.tabs()[0])
            have = page.winfo_height()
            if need > have:
                errs.append("the wiicam camera panel needs %dpx of a %dpx tab "
                            "area -- its last lines are cut off" % (need, have))
        else:
            errs.append("could not find the wiicam camera panel to measure")
        subprocess.run("import -window root /tmp/studio_camera_wiicam.png",
                       shell=True, capture_output=True)

    subprocess.run("import -window root /tmp/studio.png", shell=True, capture_output=True)
    print("wire during toggles: %s" % hid)
    print("studio render: %s" % ("OK" if not errs else "FAILED -- %s" % errs[0]))
    time.sleep(0.2)
    os._exit(1 if errs else 0)

threading.Thread(target=driver, daemon=True).start()
gun_studio.main()
