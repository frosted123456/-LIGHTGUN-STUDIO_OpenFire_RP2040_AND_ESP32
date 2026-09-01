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
            "fullreg=5 bmin=2 bmax=9 rtol=3 hwmax=-1 hwmin=-1\n",
            "CAM: blob fmt=1 ext=1 fullreg=5 bmin=2 bmax=9 rtol=3 hwmax=-1 "
            "hwmin=-1 bn=4 brej=7 brrej=2 bvalve=4 bframes=900 bms=9000 "
            "bdrop=3 br4=800 br3=150 br2=40 br1=10 br0=0\n",
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

        # ---- full detail: seven fields per blob ----------------------------
        # Box and brightness only exist here, and the readout line is at its
        # longest -- which is the state the panel has to still fit in.
        for ln in (
            "CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 hwmax=-1 "
            "hwmin=-1 bn=4 brej=90 brrej=39 bvalve=61 bframes=1900 bms=19000 "
            "bdrop=3 br4=1700 br3=250 br2=90 br1=40 br0=20\n",
            "CAM: blobs 130,140,3,1,11,9,214 200,141,14,0,12,10,203 "
            "131,140,13,0,10,9,198 210,150,14,0,41,38,255\n",
        ):
            link.src.q.put(ln)
        root.update()
        time.sleep(1.6)
        root.update()
        all_text = texts(root, [])
        joined = "\n".join(all_text)
        if "box 41x38 bright 255" not in joined:
            errs.append("full mode never showed the box and brightness: %s"
                        % [t for t in all_text if "blobs now" in t])
        if "set blob detail" in joined:
            errs.append("the basic-mode 'sizes need fmt:1' hint outlived "
                        "basic mode")
        # bvalve MOVED this time, so the warning is real and has to show...
        if "SIZE WINDOW TOO TIGHT" not in joined:
            errs.append("a give-back happening now raised no size-window "
                        "warning: %s"
                        % [t for t in all_text if "saw all four LEDs" in t])
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

        # ...and the give-back warning has to come OFF again once the gun
        # stops giving blobs back. bvalve counts since boot: read raw it
        # latched for the rest of the power cycle, on a panel whose whole job
        # is to say whether the window the user just typed is a good one.
        for ln in (
            "CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 hwmax=-1 "
            "hwmin=-1 bn=4 brej=90 brrej=39 bvalve=61 bframes=2900 bms=29000 "
            "bdrop=3 br4=2600 br3=300 br2=100 br1=45 br0=25\n",
            "CAM: blobs 130,140,3,1,11,9,214 200,141,4,1,12,10,203 "
            "131,140,3,1,10,9,198 210,150,3,1,41,38,255\n",
        ):
            link.src.q.put(ln)
        root.update()
        time.sleep(1.6)
        root.update()
        if "SIZE WINDOW TOO TIGHT" in "\n".join(texts(root, [])):
            errs.append("the size-window warning never cleared after the "
                        "give-backs stopped")

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
            if len(got) != 1 or not got[0].startswith("shape-"):
                errs.append("'Shape CSV' wrote no shape-DATE.csv: %s" % got)
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

        subprocess.run("import -window root /tmp/studio_camera_wiicam.png",
                       shell=True, capture_output=True)

    subprocess.run("import -window root /tmp/studio.png", shell=True, capture_output=True)
    print("wire during toggles: %s" % hid)
    print("studio render: %s" % ("OK" if not errs else "FAILED -- %s" % errs[0]))
    time.sleep(0.2)
    os._exit(1 if errs else 0)

threading.Thread(target=driver, daemon=True).start()
gun_studio.main()
