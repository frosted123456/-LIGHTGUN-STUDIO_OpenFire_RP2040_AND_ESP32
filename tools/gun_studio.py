#!/usr/bin/env python3
"""Lightgun Studio -- one window that walks the whole setup, in order:

  1 buttons & pins (OpenFIRE app)   2 camera tuning   3 lens / FOV
  4 aim calibration                 5 fine tune       6 verify

Order matters: aim error scales with blob noise, so the calibration step stays
locked until step 2 reports a usable noise floor. Step 3 only matters when the
camera does not wear the stock 66-degree lens. F9 freezes the cursor while the
window is open; steps that need the gun release it and put it back."""
import os, sys, subprocess, threading, queue, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import aim_fit
from aim_calib import (parse_q, is_trigger, sigma_from_hold, find_gun,
                       SerialSource, FRAME_W, FRAME_H)

SIGMA_GOOD, SIGMA_OK = 0.30, 0.60      # px; see the note above for why this gates
# Board-scaled noise gates: the wiicam's 33-deg lens has ~2.2x less screen
# gain than the OV2640, so the same screen error tolerates 2.2x the sigma.
def sigma_gates(board):
    k = 2.2 if (board and "wiicam" in board) else 1.0
    return SIGMA_GOOD * k, SIGMA_OK * k
APP_PORT_WAIT_S = 60.0                 # how long to keep trying after their app exits
CAM_KEYS = ("thr", "aec", "agc", "boost")
LENS_KEYS = ("lens", "lk1u", "lk2u", "lfpx", "lfeq")
CAM_RANGE = {"thr": (8, 200), "aec": (4, 400), "agc": (0, 30), "boost": (0, 1)}


def port_is_free(port):
    """Can this port be opened right now? Used only to tell whether another
       process still holds it. Deliberately does NOT touch the Link object --
       that one is owned by the Tk tick loop and opening it from a worker
       thread would race the pump()."""
    try:
        import serial
        s = serial.Serial(port, 115200, timeout=0.2)
        s.close()
        return True
    except Exception:
        return False


def take_port_back(proc, port, timeout_s=APP_PORT_WAIT_S, probe=port_is_free,
                   sleep=time.sleep, clock=time.monotonic):
    """Block until `proc` has exited AND `port` can be opened again.

       Returns "no port", "free" or "timeout". The caller reconnects either way:
       a timeout is a thing to report, not a reason to leave the UI parked on a
       status that will never change on its own.

       The two waits are separate on purpose. Some launchers start the real app
       in a second process and exit immediately, so proc.wait() returning does
       not mean the port is back."""
    try:
        if proc is not None:
            proc.wait()
    except Exception:
        pass
    if not port:
        return "no port"
    deadline = clock() + timeout_s
    while clock() < deadline:
        if probe(port):
            return "free"
        sleep(1.0)
    return "timeout"


# ---------------------------------------------------------------------------
# a serial link that owns the port and can be handed over to a child process
# ---------------------------------------------------------------------------
class Link:
    def __init__(self):
        self.src = None
        self.port = None
        self.frames = 0
        self.hist = []            # recent quads, for the live view and sigma
        self.last = {}
        self.replies = []
        self.hid_on = True        # the gun boots this way; we do not change it uninvited

    def connect(self, port=None):
        self.port = port or find_gun()
        if not self.port:
            return False
        self.src = SerialSource(self.port)
        self.src.start()
        return True

    def close(self):
        if self.src:
            try: self.src.close()
            except Exception: pass
            self.src = None

    def send(self, line):
        if not self.src: return
        # The '~' matters. aim_runtime_command accepts a bare name, but the
        # gatekeeper on the shared serial only CLAIMS lines starting with '~' --
        # anything else is passed through to OpenFIRE and silently discarded.
        # An auto-installed calibration went missing exactly this way.
        if not line.startswith("~"): line = "~" + line
        try: self.src.ser.write(("\n%s\n" % line).encode())
        except Exception: pass

    def pointer(self, on, remember=True):
        """Freeze or release the cursor.

        The gun boots with the pointer ON and this app does NOT take it away by
        itself -- freezing is a thing the user asks for, with a key, and the
        window says so. Doing it silently on connect meant opening the app made
        the gun stop working with no visible cause.

        `remember=False` forces the pointer on temporarily (calibration, verify,
        or handing the port to another app) without forgetting that the user had
        chosen frozen, so their choice comes back afterwards.

        RULE: we can only change this while we own the serial port. Every path
        that gives the port away must force it ON first, or the user is left
        frozen until they replug."""
        if remember:
            self.hid_on = on
        self.send("~aimhid=%d" % (1 if on else 0))

    def pump(self):
        """drain the stream; keep the last ~2 s of quads"""
        if not self.src: return
        n = 0
        while n < 400:
            try: line = self.src.q.get_nowait()
            except queue.Empty: break
            n += 1
            if line.startswith("CAM:") or line.startswith("AIM:") or "CMD ok" in line:
                self.replies.append(line)
                # keep the label honest if the gun disagrees with us
                if "pointer FROZEN" in line: self.hid_on = False
                elif "pointer ON" in line:   self.hid_on = True
                # Any CAM:/pong/ack line may carry k=v state -- the old
                # "CAM: thr=" prefix missed the wiicam's "CAM: board=" readback
                # AND the ping's board tag, so Studio never learned the board
                # and kept showing the ESP32 tuning panel.
                if line.startswith("CAM:") or line.startswith("AIM: pong") \
                        or "CMD ok" in line:
                    for tok in line.replace("|", " ").split():
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            if k == "board":
                                self.last["board"] = v
                            elif k in CAM_KEYS or k in LENS_KEYS or k == "sens":
                                try: self.last[k] = int(v)
                                except ValueError: pass
                continue
            if is_trigger(line):
                self.last["trig"] = self.last.get("trig", 0) + 1
                continue
            pq = parse_q(line)
            if pq is None: continue
            q, gt = pq
            self.frames += 1
            self.hist.append((gt, q))
        cut = self.hist[-1][0] - 2.0 if self.hist else 0
        self.hist = [h for h in self.hist if h[0] >= cut][-400:]

    # ---- measurements ----------------------------------------------------
    def sigma(self):
        """blob noise with hand tremor removed; None until enough frames"""
        if len(self.hist) < 40: return None
        a = np.array([h[1] for h in self.hist[-120:]])
        # only use a stretch where the hand was reasonably still, or tremor
        # dominates and the number means nothing
        cen = a.mean(1)
        if max(np.ptp(cen[:, 0]), np.ptp(cen[:, 1])) > 12.0: return None
        return sigma_from_hold(a)

    def span(self):
        if not self.hist: return 0.0
        return aim_fit.quad_span(self.hist[-1][1])

    def fps(self):
        if len(self.hist) < 10: return 0.0
        dt = self.hist[-1][0] - self.hist[0][0]
        return (len(self.hist) - 1) / dt if dt > 0 else 0.0


# ---------------------------------------------------------------------------
# auto-tune: find an exposure/threshold that gives four stable blobs quietly
# ---------------------------------------------------------------------------
def auto_tune(link, log, stop):
    """Sweep aec x thr, score each point, apply the best.

    Score is deliberately NOT "lowest sigma": a very high threshold gives
    beautiful sigma on two surviving blobs, which is useless. Four blobs, seen on
    essentially every frame, comes first; sigma only breaks ties.
    """
    best = None
    aecs = [20, 30, 40, 60, 90]
    thrs = [40, 60, 80, 110, 150]
    total = len(aecs) * len(thrs)
    done = 0
    for aec in aecs:
        for thr in thrs:
            if stop.is_set(): log("auto-tune cancelled"); return None
            link.send("~cam=aec:%d,thr:%d" % (aec, thr))
            time.sleep(0.35)                       # let the sensor settle
            link.hist = []
            t0 = time.time()
            while time.time() - t0 < 0.9:
                link.pump(); time.sleep(0.02)
            done += 1
            n = len(link.hist)
            if n < 15:
                log("  aec=%-3d thr=%-3d  no frames" % (aec, thr)); continue
            sg = link.sigma()
            spans = [aim_fit.quad_span(h[1]) for h in link.hist]
            # every frame in the stream already has 4 blobs (parse_q drops the
            # rest), so "frames per second" IS the four-blob hit rate
            rate = n / 0.9
            score = (rate, -(sg if sg is not None else 9.9))
            log("  aec=%-3d thr=%-3d  %5.0f fps  span %5.1f  sigma %s"
                % (aec, thr, rate, np.mean(spans),
                   "%.3f" % sg if sg is not None else "  -  (hand moved)"))
            if best is None or score > best[0]:
                best = (score, aec, thr, sg, rate)
    if not best:
        log("auto-tune found NOTHING -- are all four LEDs in view?")
        return None
    _, aec, thr, sg, rate = best
    log("")
    log("best: aec=%d thr=%d  (%0.0f fps with four blobs, sigma %s)"
        % (aec, thr, rate, "%.3f" % sg if sg is not None else "not measured"))
    link.send("~cam=aec:%d,thr:%d" % (aec, thr))
    time.sleep(0.3)
    log("applied. Press 'Save to gun' to keep it across power cycles.")
    return (aec, thr)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
def main():
    import tkinter as tk
    from tkinter import ttk, messagebox

    link = Link()
    root = tk.Tk()
    root.title("Lightgun Studio")
    root.configure(bg="#0d1117")
    root.geometry("1020x800")

    C_BG, C_FG, C_DIM, C_OK, C_WARN, C_BAD = "#0d1117", "#e6edf3", "#7d8590", "#39c26e", "#d8a13a", "#d24b4b"
    F = ("Segoe UI" if os.name == "nt" else "DejaVu Sans", 10)
    FB = (F[0], 11, "bold")
    FH = (F[0], 15, "bold")

    def lab(parent, text, font=F, fg=C_FG, **kw):
        return tk.Label(parent, text=text, font=font, fg=fg, bg=C_BG, **kw)

    # ---- header ---------------------------------------------------------
    head = tk.Frame(root, bg=C_BG); head.pack(fill="x", padx=16, pady=(14, 6))
    lab(head, "Lightgun Studio", FH).pack(side="left")
    st_conn = lab(head, "not connected", F, C_BAD); st_conn.pack(side="right")
    # The pointer toggle lives in the header because it is a MODE, not an
    # action, and the window has to say which mode it is in -- a gun that has
    # silently stopped moving the cursor is indistinguishable from a broken gun.
    st_hid = lab(head, "", F, C_OK); st_hid.pack(side="right", padx=(0, 18))
    # How many distances to calibrate from. Two is measured to be as good as
    # three (65.7 vs 63.6 px at blob sigma 0.3, identical at 0.6) and saves 20
    # trigger pulls, and not every room lets you take three steps back. What is
    # NOT optional is more than one: at a single distance the boresight and the
    # screen mapping are exactly degenerate and the fit refuses.
    stance_n = tk.IntVar(value=3)

    body = tk.Frame(root, bg=C_BG); body.pack(fill="both", expand=True, padx=16, pady=8)

    # ---- left: the four steps -------------------------------------------
    left = tk.Frame(body, bg=C_BG); left.pack(side="left", fill="y", padx=(0, 18))
    step_rows = {}
    for n, (num, title, sub) in enumerate([
        (1, "Buttons & pins", "opens the OpenFIRE app"),
        (2, "Camera tuning",  "exposure, threshold, noise floor"),
        (3, "Lens / FOV",     "only if your lens is not the stock 66\u00b0"),
        (4, "Aim calibration","five dots x three distances"),
        (5, "Fine tune",      "iron sights to cursor, and lead"),
        (6, "Verify",         "measures whose error it is")]):
        f = tk.Frame(left, bg=C_BG); f.pack(fill="x", pady=5)
        b = tk.Button(f, text="%d.  %s" % (num, title), font=FB, width=22, anchor="w",
                      bg="#161b22", fg=C_FG, activebackground="#21262d",
                      relief="flat", padx=10, pady=8)
        b.pack(fill="x")
        s = lab(f, "   " + sub, (F[0], 9), C_DIM, anchor="w"); s.pack(fill="x")
        step_rows[num] = (b, s)
        if num == 4:
            # Distance count, next to the step it applies to. Two positions is
            # measured to be as good as three and saves 20 trigger pulls; some
            # rooms simply do not allow three.
            rowd = tk.Frame(left, bg=C_BG); rowd.pack(fill="x", pady=(2, 0))
            lab(rowd, "   distances:", (F[0], 9), C_DIM).pack(side="left")
            for nval in (2, 3):
                tk.Radiobutton(rowd, text=str(nval), value=nval, variable=stance_n,
                               font=(F[0], 9), bg=C_BG, fg=C_FG, selectcolor="#161b22",
                               activebackground=C_BG, activeforeground=C_FG,
                               highlightthickness=0, bd=0,
                               command=lambda: step_rows[4][1].config(
                                   text="   five dots x %d distances" % stance_n.get())
                               ).pack(side="left")
            lab(rowd, "(2 is nearly as good and 20 fewer pulls)",
                (F[0], 8), C_DIM).pack(side="left", padx=(6, 0))

    # ---- right: the live panel ------------------------------------------
    right = tk.Frame(body, bg=C_BG); right.pack(side="left", fill="both", expand=True)
    toprow = tk.Frame(right, bg=C_BG); toprow.pack(fill="x")
    cv = tk.Canvas(toprow, width=380, height=280, bg="#010409", highlightthickness=1,
                   highlightbackground="#30363d")
    cv.pack(side="left", anchor="n")
    stats = tk.Frame(toprow, bg=C_BG); stats.pack(side="left", fill="both",
                                                  expand=True, padx=(14, 0))
    stat_vals = {}
    for k in ("frames", "rate", "quad span", "blob noise", "verdict"):
        r = tk.Frame(stats, bg=C_BG); r.pack(fill="x", pady=3)
        lab(r, k, F, C_DIM, width=11, anchor="w").pack(side="left")
        v = lab(r, "-", FB, C_FG, anchor="w", wraplength=230, justify="left")
        v.pack(side="left")
        stat_vals[k] = v

    logbox = tk.Text(right, height=8, bg="#010409", fg=C_DIM, font=("Consolas" if os.name=="nt" else "monospace", 9),
                     relief="flat", highlightthickness=1, highlightbackground="#30363d")
    logbox.pack(fill="both", expand=True, pady=(10, 0), side="bottom")
    def log(msg):
        logbox.insert("end", msg + "\n"); logbox.see("end")
    PY_LOG = log

    # ---- step actions ---------------------------------------------------
    def find_openfire_app():
        """Look where it actually tends to live before asking."""
        roots = [os.path.join(HERE, "..", ".."), os.path.expanduser("~")]
        for r in roots:
            for dirpath, dirnames, files in os.walk(os.path.abspath(r)):
                if dirpath.count(os.sep) - os.path.abspath(r).count(os.sep) > 4:
                    dirnames[:] = []
                    continue
                for f in files:
                    if f.lower() == "openfireapp.exe":
                        return os.path.join(dirpath, f)
        return None

    def step1():
        exe = find_openfire_app()
        if not exe:
            log("Could not find OpenFIREapp.exe automatically.")
            from tkinter import filedialog
            exe = filedialog.askopenfilename(title="Find OpenFIREapp.exe",
                                             filetypes=[("OpenFIRE app", "*.exe")])
            if not exe: return
        # Their app needs the port to itself, so hand it over and take it back.
        log("Releasing the port and starting the OpenFIRE app...")
        # Once the port is gone we cannot send ~aimhid any more, so a frozen
        # pointer would be stuck until a replug. Release it while we still can.
        link.pointer(True, remember=False)
        link.close()
        st_conn.config(text="OpenFIRE app has the port -- close it to come back",
                       fg=C_WARN)
        try:
            proc = subprocess.Popen([exe], cwd=os.path.dirname(exe))
        except Exception as e:
            log("could not start it: %s" % e); reconnect(); return
        log("Set your pins and buttons there, then just CLOSE it.")

        # Steps 3-5 come back on their own because they subprocess.call() a tool
        # we wrote. This one launches somebody else's app, so it used to end at
        # Popen and leave the status stuck on 'handed off' until the user found
        # the Reconnect button -- which reads as a hang, not a prompt. Wait for
        # it here instead. Their app may also hand off to a second process and
        # exit at once, so the port can still be held after wait() returns:
        # probe it directly (never through `link`, which the tick loop owns)
        # until it comes free.
        def take_the_port_back():
            why = take_port_back(proc, link.port or "")
            def done():
                if why == "timeout":
                    log("The port is still held after %ds -- something else has it."
                        % int(APP_PORT_WAIT_S))
                else:
                    log("OpenFIRE app closed; taking the port back...")
                reconnect()
            root.after(0, done)
        threading.Thread(target=take_the_port_back, daemon=True).start()
        log("Two settings that matter for us, in the profile:")
        log("  RunMode = Normal        (Average stacks extra smoothing on ours)")
        log("  serialARcorrection off  (it would re-correct an already-correct aim)")

    def step2():
        nb.select(tab_cam)

    def step3():
        nb.select(tab_lens)

    def step4():
        sg = link.sigma()
        s_good, s_ok = sigma_gates(link.last.get("board"))
        if sg is not None and sg > s_ok:
            from tkinter import messagebox
            if not messagebox.askyesno("Noise floor is high",
                    "Blob noise is %.2f px.\n\n"
                    "Aim error scales with this: 0.2 px gives about 16 px of error, "
                    "0.8 px gives about 46 px, and calibrating now bakes that in.\n\n"
                    "Tune the camera first?  (Yes = go to tuning, No = calibrate anyway)"
                    % sg):
                pass
            else:
                nb.select(tab_cam); return
        log("Handing the port to the calibration window...")
        link.pointer(True, remember=False)   # calibration reads the trigger as a click
        link.close()
        st_conn.config(text="calibrating...", fg=C_WARN)
        def run():
            try:
                subprocess.call([sys.executable, os.path.join(HERE, "aim_calib.py"),
                                 "--port", link.port or "",
                                 "--stances", str(stance_n.get())])
            except Exception as e:
                log("calibration failed to start: %s" % e)
            root.after(0, reconnect)
        threading.Thread(target=run, daemon=True).start()

    def _handoff(tool, label):
        """Every child window drives the cursor with the gun, so the pointer has
           to be live -- and it has to come back to whatever the user chose."""
        log("Handing the port to the %s window..." % label)
        link.pointer(True, remember=False)
        link.close()
        st_conn.config(text="%s..." % label, fg=C_WARN)
        def run():
            try:
                subprocess.call([sys.executable, os.path.join(HERE, tool),
                                 "--port", link.port or ""])
            except Exception as e:
                log("%s failed to start: %s" % (label, e))
            root.after(0, reconnect)
        threading.Thread(target=run, daemon=True).start()

    def step5(): _handoff("aim_finetune.py", "fine tune")
    def step6(): _handoff("aim_verify.py", "verify")

    step_rows[1][0].config(command=step1)
    step_rows[2][0].config(command=step2)
    step_rows[3][0].config(command=step3)
    step_rows[4][0].config(command=step4)
    step_rows[5][0].config(command=step5)
    step_rows[6][0].config(command=step6)

    # ---- tabs: camera tuning lives here ---------------------------------
    nb = ttk.Notebook(right)
    tab_cam = tk.Frame(nb, bg=C_BG)
    nb.add(tab_cam, text="  Camera  ")
    nb.pack(fill="x", pady=(12, 0))

    # ESP32 (OV2640) tuning widgets live in one frame, wiicam's in another;
    # the gun's own ~ping board tag decides which is shown.
    frame_esp = tk.Frame(tab_cam, bg=C_BG); frame_esp.pack(fill="x")
    frame_wii = tk.Frame(tab_cam, bg=C_BG)   # packed on detect

    lab(frame_wii, "wiicam sensitivity (persisted in the OpenFIRE profile):",
        (F[0], 9), C_DIM, anchor="w").pack(fill="x", pady=(6, 2))
    roww = tk.Frame(frame_wii, bg=C_BG); roww.pack(fill="x", pady=2)
    for sv, nm in ((0, "Default"), (1, "High"), (2, "Max")):
        tk.Button(roww, text=nm, font=F, bg="#161b22", fg=C_FG, relief="flat",
                  padx=12, pady=6,
                  command=lambda v=sv: (link.send("~cam=sens:%d" % v),
                                        log("sensitivity -> %d" % v))
                  ).pack(side="left", padx=(0, 6))
    lab(frame_wii, "Default fits most rigs (it is OpenFIRE's own default). Watch the\n"
        "noise floor above -- it still measures on this board.",
        (F[0], 8), C_DIM, justify="left", anchor="w").pack(fill="x", pady=(2, 4))
    barw = tk.Frame(frame_wii, bg=C_BG); barw.pack(fill="x", pady=6)
    tk.Button(barw, text="Save to gun", command=lambda: (link.send("~camsave"), log("~camsave sent")),
              font=FB, bg="#238636", fg="white", relief="flat", padx=14, pady=6).pack(side="left")
    tk.Button(barw, text="Read from gun", command=lambda: link.send("~cam?"),
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=6).pack(side="left", padx=8)

    sliders = {}
    for k in CAM_KEYS:
        lo, hi = CAM_RANGE[k]
        r = tk.Frame(frame_esp, bg=C_BG); r.pack(fill="x", pady=1)
        lab(r, k, F, C_DIM, width=7, anchor="w").pack(side="left")
        v = tk.IntVar(value=lo)
        s = tk.Scale(r, from_=lo, to=hi, orient="horizontal", variable=v,
                     bg=C_BG, fg=C_FG, troughcolor="#161b22", highlightthickness=0,
                     length=260, showvalue=True)
        s.pack(side="left")
        sliders[k] = v
        def mk(kk, vv):
            def on(_=None): link.send("~cam=%s:%d" % (kk, vv.get()))
            return on
        s.config(command=mk(k, v))

    bar = tk.Frame(frame_esp, bg=C_BG); bar.pack(fill="x", pady=6)
    stop_flag = threading.Event()
    def do_auto():
        stop_flag.clear()
        log("auto-tune: sweeping exposure x threshold, about 30 s...")
        def run():
            r = auto_tune(link, lambda m: root.after(0, log, m), stop_flag)
            if r: root.after(0, lambda: (sliders["aec"].set(r[0]), sliders["thr"].set(r[1])))
        threading.Thread(target=run, daemon=True).start()
    tk.Button(bar, text="Auto-tune", command=do_auto, font=FB, bg="#1f6feb", fg="white",
              relief="flat", padx=14, pady=6).pack(side="left")
    tk.Button(bar, text="Save to gun", command=lambda: (link.send("~camsave"), log("~camsave sent")),
              font=FB, bg="#238636", fg="white", relief="flat", padx=14, pady=6).pack(side="left", padx=8)
    tk.Button(bar, text="Read from gun", command=lambda: link.send("~cam?"),
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=6).pack(side="left")
    tk.Button(bar, text="Cancel", command=stop_flag.set,
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=6).pack(side="right")

    # ---- tab: lens / FOV --------------------------------------------------
    # Only matters when the camera does not wear the stock 66-degree lens. A
    # wide or fisheye lens bends the LED quad; the homography assumes a pinhole,
    # and the calibration has nowhere to put a radially-varying error -- so the
    # correction has to happen upstream, on the blob centroids, in the firmware.
    # This tab sets that correction: a preset from the datasheet FOV, or a
    # measured fit from a 20-second sweep. Save writes it to NVS with ~camsave.
    import calib_lens
    tab_lens = tk.Frame(nb, bg=C_BG)
    nb.add(tab_lens, text="  Lens  ")

    lens_state = lab(tab_lens, "current: unknown -- press Read from gun", (F[0], 9), C_DIM,
                     anchor="w")
    lens_state.pack(fill="x", pady=(6, 2))

    rowp = tk.Frame(tab_lens, bg=C_BG); rowp.pack(fill="x", pady=3)
    lab(rowp, "lens FOV:", F, C_DIM).pack(side="left")
    fov_var = tk.StringVar(value="66")
    tk.Entry(rowp, textvariable=fov_var, width=5, font=F, bg="#161b22", fg=C_FG,
             insertbackground=C_FG, relief="flat").pack(side="left", padx=(4, 2))
    lab(rowp, "deg (full horizontal, from the lens listing)", (F[0], 9), C_DIM).pack(side="left")

    def fov_value():
        try:
            v = float(fov_var.get())
            if 30 <= v <= 200: return v
        except ValueError:
            pass
        log("lens: FOV must be a number between 30 and 200 degrees")
        return None

    def lens_off():
        link.send("~cam=lens:0")
        log("lens: correction OFF (stock lens). Press Save to keep it.")

    def lens_preset():
        fov = fov_value()
        if fov is None: return
        if fov <= 75:
            log("lens: %.0f deg is close enough to a pinhole that no preset is"
                " needed -- the calibration absorbs the focal length itself."
                " Use Measure if the image is visibly bent." % fov)
            return
        r = calib_lens.spec_fisheye(fov)
        link.send("~cam=" + calib_lens.tune_line(dict(r, model="fisheye")))
        log("lens: fisheye preset applied for %.0f deg (feq=%.1f, fpx=%.1f)."
            % (fov, r["feq"], r["fpx"]))
        log("lens: a preset assumes an ideal equidistant lens. Measure beats it.")
        log("Press 'Save to gun' to keep it across power cycles.")

    lens_busy = {"on": False}

    def lens_measure():
        if lens_busy["on"]:
            log("lens: a measurement is already running"); return
        fov = fov_value()
        if fov is None: return
        if not link.src:
            log("lens: not connected"); return
        lens_busy["on"] = True
        # raw data: resolver off (it invents corners), correction off (fitting
        # corrected data fits garbage), full frame rate
        link.send("~cam=res:0,lens:0,dashhz:0")
        log("lens: MEASURING for 20 s. Stand ~2 m from the rig, feet planted,")
        log("and slowly pan/tilt/roll the gun so the LEDs travel across the")
        log("WHOLE image -- push them out to the edges and corners. Keep all")
        log("four in frame.")
        t0 = time.time()
        frames = []
        # hist timestamps are the GUN's clock, not ours -- comparing them
        # against time.time() collects garbage. Accumulate by identity instead:
        # every (gun-time, quad) pair not seen before is a new frame.
        seen = set()

        def collect():
            for (gt, q) in link.hist:
                if gt not in seen:
                    seen.add(gt)
                    frames.append(np.asarray(q, float))
            left = 20.0 - (time.time() - t0)
            if left > 0:
                lens_state.config(text="measuring... %2.0f s left, %d frames"
                                  % (left, len(frames)), fg=C_WARN)
                root.after(250, collect)
                return
            link.send("~cam=res:2,dashhz:60")
            lens_state.config(text="fitting...", fg=C_WARN)
            snap = np.array(frames) if frames else np.zeros((0, 4, 2))

            def fit():
                r = calib_lens.fit_from_frames(snap, fov)

                def done():
                    lens_busy["on"] = False
                    if not r["ok"]:
                        lens_state.config(text="measure failed -- see the log", fg=C_BAD)
                        log("lens: REFUSED: %s" % r["why"])
                        return
                    link.send("~cam=" + calib_lens.tune_line(r))
                    lens_state.config(
                        text="measured: %s  rms %.2f px  (coverage %.0f%%)"
                             % (r["model"], r["rms_px"], r["coverage"] * 100), fg=C_OK)
                    log("lens: fitted %s model, residual %.2f px rms." % (r["model"], r["rms_px"]))
                    if r["rms_px"] > 1.0:
                        log("lens: residual is high -- consider redoing the sweep more slowly.")
                    log("Applied live. Press 'Save to gun' to keep it across power cycles.")
                root.after(0, done)
            threading.Thread(target=fit, daemon=True).start()
        root.after(250, collect)

    rowb = tk.Frame(tab_lens, bg=C_BG); rowb.pack(fill="x", pady=6)
    tk.Button(rowb, text="Stock lens (off)", command=lens_off, font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=12, pady=6).pack(side="left")
    tk.Button(rowb, text="Preset from FOV", command=lens_preset, font=FB, bg="#1f6feb",
              fg="white", relief="flat", padx=12, pady=6).pack(side="left", padx=8)
    tk.Button(rowb, text="Measure (20 s sweep)", command=lens_measure, font=FB,
              bg="#1f6feb", fg="white", relief="flat", padx=12, pady=6).pack(side="left")
    tk.Button(rowb, text="Save to gun", command=lambda: (link.send("~camsave"), log("~camsave sent")),
              font=FB, bg="#238636", fg="white", relief="flat", padx=12, pady=6).pack(side="left", padx=8)
    tk.Button(rowb, text="Read from gun", command=lambda: link.send("~cam?"),
              font=F, bg="#161b22", fg=C_FG, relief="flat", padx=12, pady=6).pack(side="left")
    lab(tab_lens, "Wide lens trade-off: more FOV = stand closer and less edge "
        "clipping, but every\nnoise source is magnified by the shorter focal. "
        "The stock 66\u00b0 lens needs nothing here.", (F[0], 8), C_DIM,
        justify="left", anchor="w").pack(fill="x", pady=(2, 4))

    board_state = {"cur": None}

    def board_tick():
        b = link.last.get("board")
        if b and b != board_state["cur"]:
            board_state["cur"] = b
            if "wiicam" in b:
                frame_esp.pack_forget()
                frame_wii.pack(fill="x")
                log("board: %s -- camera tab switched to sensitivity" % b)
            else:
                frame_wii.pack_forget()
                frame_esp.pack(fill="x")
        root.after(500, board_tick)
    root.after(500, board_tick)

    def lens_tick():
        # keep the state line honest from the gun's own cam? replies
        if not lens_busy["on"] and "lens" in link.last:
            m = link.last.get("lens", 0)
            if m == 0:
                lens_state.config(text="current: correction OFF (stock lens)", fg=C_DIM)
            elif m == 1:
                lens_state.config(text="current: polynomial  k1=%dppm k2=%dppm fpx=%.1f"
                                  % (link.last.get("lk1u", 0), link.last.get("lk2u", 0),
                                     link.last.get("lfpx", 0) / 10.0), fg=C_OK)
            else:
                lens_state.config(text="current: fisheye  feq=%.1f fpx=%.1f"
                                  % (link.last.get("lfeq", 0) / 10.0,
                                     link.last.get("lfpx", 0) / 10.0), fg=C_OK)
        root.after(500, lens_tick)
    root.after(500, lens_tick)

    # ---- pointer toggle ---------------------------------------------------
    def refresh_hid():
        if link.hid_on:
            st_hid.config(text="aim ON   (F9 to freeze)", fg=C_OK)
        else:
            st_hid.config(text="aim FROZEN   (F9 to release)", fg=C_WARN)

    def toggle_hid(_=None):
        if not link.src:
            log("Not connected -- cannot change the pointer.")
            return
        link.pointer(not link.hid_on)
        refresh_hid()
        log("Pointer %s." % ("released -- the gun drives the cursor again"
                             if link.hid_on else
                             "frozen -- the gun stops driving the cursor. "
                             "Steps 4 to 6 release it automatically."))

    # Window-scoped, not system-wide: a global hook would need an extra package
    # and the ability to swallow F9 from every other application, which is a lot
    # of blast radius for a convenience. Focus the window and press F9.
    root.bind("<F9>", toggle_hid)
    root.bind("<KeyPress-F9>", toggle_hid)

    # ---- connection + live tick ------------------------------------------
    def reconnect():
        link.close()
        st_conn.config(text="looking for the gun...", fg=C_WARN)
        root.update_idletasks()
        if link.connect(link.port):
            st_conn.config(text="connected on %s" % link.port, fg=C_OK)
            log("connected on %s" % link.port)
            link.send("~ping")
            link.send("~cam?")
            link.send("~aimcal?")
            link.pointer(link.hid_on)      # the gun boots ON; we own this per session
            refresh_hid()
            refresh_hid()
        else:
            st_hid.config(text="", fg=C_DIM)
            st_conn.config(text="no gun found", fg=C_BAD)
            log("No gun found. Close the OpenFIRE app if it is open, then Reconnect.")

    tk.Button(head, text="Reconnect", command=reconnect, font=F, bg="#161b22",
              fg=C_FG, relief="flat", padx=10).pack(side="right", padx=10)

    def draw_view():
        cv.delete("all")
        W, H = 380, 280
        cv.create_rectangle(2, 2, W-2, H-2, outline="#30363d")
        if not link.hist:
            cv.create_text(W/2, H/2, text="no four-LED frames", fill=C_BAD, font=FB)
            return
        q = aim_fit.canon(link.hist[-1][1])
        for i, nm in enumerate(("TL", "TR", "BL", "BR")):
            x = 4 + (q[i][0]/FRAME_W)*(W-8)
            y = 4 + (q[i][1]/FRAME_H)*(H-8)
            cv.create_oval(x-4, y-4, x+4, y+4, fill="#ffd24a", outline="")
            cv.create_text(x+10, y-8, text=nm, fill=C_DIM, font=(F[0], 8), anchor="w")
        pts = [(4 + (q[i][0]/FRAME_W)*(W-8), 4 + (q[i][1]/FRAME_H)*(H-8)) for i in range(4)]
        for a, b in ((0,1),(1,3),(3,2),(2,0)):
            cv.create_line(*pts[a], *pts[b], fill="#3a7fbf")
        cv.create_line(W/2-6, H/2, W/2+6, H/2, fill="#e0803a")
        cv.create_line(W/2, H/2-6, W/2, H/2+6, fill="#e0803a")
        # a trail, so instability is visible rather than inferred
        for _, qq in link.hist[-60:]:
            c = qq.mean(0)
            x = 4 + (c[0]/FRAME_W)*(W-8); y = 4 + (c[1]/FRAME_H)*(H-8)
            cv.create_oval(x-1, y-1, x+1, y+1, outline="", fill="#1f6feb")

    def tick():
        link.pump()
        while link.replies:
            log(link.replies.pop(0))
        refresh_hid()      # the gun may have released itself via the escape hatch
        for k in CAM_KEYS:
            if k in link.last and sliders[k].get() != link.last[k]:
                sliders[k].set(link.last[k])
        draw_view()
        sg = link.sigma()
        stat_vals["frames"].config(text="%d" % link.frames)
        stat_vals["rate"].config(text="%.0f Hz" % link.fps())
        stat_vals["quad span"].config(text="%.1f px" % link.span())
        if sg is None:
            stat_vals["blob noise"].config(text="hold still to measure", fg=C_DIM)
            stat_vals["verdict"].config(text="-", fg=C_DIM)
        else:
            s_good, s_ok = sigma_gates(link.last.get("board"))
            col = C_OK if sg <= s_good else (C_WARN if sg <= s_ok else C_BAD)
            stat_vals["blob noise"].config(text="%.3f px" % sg, fg=col)
            # turn sigma into the number the user cares about
            err = 11.4 + (sg/0.05 - 1) * 1.2
            stat_vals["verdict"].config(
                text=("good -- ready to calibrate" if sg <= s_good else
                      "usable, could be better" if sg <= s_ok else
                      "too noisy -- tune before calibrating"), fg=col)
        # the calibration step is gated on step 2, and the label says why
        if sg is not None and sg > s_ok:
            step_rows[4][1].config(text="   blocked: blob noise %.2f px is too high" % sg, fg=C_BAD)
        else:
            step_rows[4][1].config(text="   five dots x %d distances" % stance_n.get(),
                                   fg=C_DIM)
        root.after(50, tick)

    log("Lightgun Studio. Step 1 sets up buttons; steps 2-6 are ours.")
    log("The gun keeps driving the cursor. Press F9 to freeze it while you work")
    log("in here; steps 4 to 6 release it on their own and put it back after.")
    log("Order matters: aim accuracy is limited by blob noise, so tune before you")
    log("calibrate. The app blocks step 4 if the noise floor is too high.")
    reconnect()
    root.after(50, tick)
    try:
        root.mainloop()
    finally:
        # unconditional: a traceback in the GUI must not leave the pointer frozen
        try: link.pointer(True, remember=False)
        except Exception: pass
        link.close()


if __name__ == "__main__":
    main()
