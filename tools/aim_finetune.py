#!/usr/bin/env python3
"""Fine-tune: line the CURSOR up with your IRON SIGHTS.

Two ways to use it. Shoot the ring at two distances and it separates an ANGULAR
sight offset (grows with range) from a PARALLAX one (constant) -- applying the
wrong kind is worse than none. Or skip the shooting: nudge with the arrows
until the cursor sits on your notch and press SAVE NOW, which keeps exactly the
preview you are looking at as a constant offset. Mixing is safe: a ring shot
only ever measures what is left AFTER your nudges.

Nudges go out as "aimcal!=" (applied, not written to flash); saving installs
and persists. LEAD +/- trades latency for overshoot on reversals.
SMOOTH +/- trades cursor jitter for glide (0 = filter off, 10 = heaviest).
BETA +/- sets how fast the smoothing lets go once the gun moves, independently
of how heavy it is at rest; "auto" follows the smoothing table."""
import argparse, os, queue, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import aim_fit
from aim_calib import (parse_q, find_gun, SerialSource, FRAME_W, FRAME_H,
                       install_over_serial, aimcal_line, camsave_verified)

STEP_PX   = 4.0                 # screen px per nudge, at 1920 wide
# 5 ms, not 2. Measured through the real resolver: on a fast sweep one 2 ms
# press is worth about 3 screen px, which is invisible -- you would need five
# presses before anything happened, with nothing on screen saying a press had
# registered. 5 ms is ~25 px on that same sweep, and six presses covers the
# whole 0-50 range.
LEAD_STEP = 5                   # ms per lead nudge
LEAD_MAX  = 50                  # the ceiling the capture layer clamps to anyway
SMOOTH_MAX = 10                 # smoothing level range the firmware accepts
# One Euro speed sensitivity: cutoff = min_cutoff + beta * speed. SMOOTH sets
# how heavy the filter is at REST, BETA how fast it lets go once the gun moves.
# -1 means "follow the smoothing table", which is a flat 15 at every level.
BETA_STEP = 3                   # beta units per nudge
BETA_MAX  = 60                  # AIM_BETA_MAX in the firmware
BETA_AUTO = 15                  # what the table uses, and the base for a nudge off -1
MIN_SPREAD = 1.25               # quad-span ratio between the two stations
# Where the ring sits, as a fraction of the window. ONE definition: the drawing
# and the measurement both read it, so they cannot drift apart -- a ring drawn
# at 0.40 and measured against 0.50 would bake a fixed 120 px error into every
# fine tune and look exactly like a working correction.
TARGET = (0.50, 0.40)
SHOT_FRAMES = 12                # frames medianed into one measurement

C_BG, C_FG, C_DIM = "#0d1117", "#e6edf3", "#7d8590"
C_OK, C_WARN, C_BAD = "#39c26e", "#d8a13a", "#d24b4b"


class Tuner:
    """State and arithmetic. No GUI, so it is testable headlessly."""

    def __init__(self, calib, lead=0, smooth=3, beta=-1):
        self.c0 = dict(calib)          # what calibration produced, untouched
        self.stage = 0                 # 0 = near, 1 = far, 2 = done
        self.off = [np.zeros(2), np.zeros(2)]   # normalised screen nudges
        self.q = [None, None]          # a representative quad per station
        self.measured = [False, False]
        self.span = [None, None]
        # Seeded from the gun so the first nudge steps FROM the saved value
        # instead of stomping it back to a default.
        self.lead = self.lead0 = int(lead)
        self.smooth = self.smooth0 = int(smooth)
        self.beta = self.beta0 = int(beta)
        self.msg = ""

    # ---- the live preview -------------------------------------------------
    def preview(self):
        """calibration with the CURRENT station's nudge applied as a constant.

        Only a constant here: while the user is still nudging we have one
        measurement and cannot honestly split it. The split happens once, at the
        end, when both stations are in."""
        d = self.off[self.stage] if self.stage < 2 else np.zeros(2)
        c = dict(self.c0)
        c['cx'] = self.c0['cx'] + d[0]
        c['cy'] = self.c0['cy'] + d[1]
        return c

    def nudge(self, dx, dy):
        if self.stage > 1: return
        self.off[self.stage] = self.off[self.stage] + np.array(
            [dx * STEP_PX / 1920.0, dy * STEP_PX / 1200.0])

    def nudge_lead(self, d):
        self.lead = int(min(LEAD_MAX, max(0, self.lead + d * LEAD_STEP)))

    def nudge_smooth(self, d):
        self.smooth = int(min(SMOOTH_MAX, max(0, self.smooth + d)))

    def nudge_beta(self, d):
        """Step beta, with -1 ("follow the table") on the low side of zero.

        A nudge off -1 starts from BETA_AUTO rather than from 0, because -1 IS
        BETA_AUTO as far as the filter is concerned -- stepping to 0 would be a
        15-unit jump in the wrong direction. Stepping below 0 lands on -1 and
        STAYS there, so the default is reachable without a separate button and
        holding the button down cannot walk back up the range."""
        if self.beta < 0 and d < 0:
            return
        base = BETA_AUTO if self.beta < 0 else self.beta
        v = base + d * BETA_STEP
        self.beta = -1 if v < 0 else int(min(BETA_MAX, v))

    def beta_label(self):
        return ("auto, %d" % BETA_AUTO) if self.beta < 0 else str(self.beta)

    def note_quad(self, q):
        """A shot on the ring MEASURES the offset; it does not merely record a
           pose.

        Solve the quad through the calibration as it currently stands, and the
        gap between that and the ring is what is left to correct. Because the
        preview already includes everything accumulated so far, the residual is
        additive: shoot again and it converges, nudge by hand and the two
        compose. Shooting when it is already right adds nothing.

        (The first version stored the quad and left the whole offset to the
        arrow buttons, which is why shooting the ring reported zero no matter
        how far off the cursor was.)"""
        if self.stage > 1: return
        q = np.asarray(q)
        self.q[self.stage] = q
        self.span[self.stage] = aim_fit.quad_span(q)
        r = aim_fit.solve(self.preview(), q, FRAME_W, FRAME_H)
        if r is None:
            self.msg = "could not solve that shot -- are all four LEDs visible?"
            return
        resid = np.array([TARGET[0] - r[0], TARGET[1] - r[1]])
        # A shot that lands a whole screen away is a mis-solve or a shot at
        # something else, not a sight offset worth believing.
        if abs(resid[0]) > 0.5 or abs(resid[1]) > 0.5:
            self.msg = ("that shot solved %.0f, %.0f px from the ring -- too far "
                        "to be a sight offset; shoot the ring itself"
                        % (resid[0]*1920.0, resid[1]*1200.0))
            return
        self.off[self.stage] = self.off[self.stage] + resid
        self.measured[self.stage] = True
        self.msg = ""

    # ---- moving between stations -----------------------------------------
    def spread(self):
        if None in self.span: return None
        lo, hi = sorted(self.span)
        return (hi / lo) if lo > 1e-6 else None

    def can_finish(self):
        """Both stations measured, and far enough apart to be separable."""
        if not (self.measured[0] and self.measured[1]):
            return False, "shoot the ring at both distances first"
        sp = self.spread()
        if sp is None or sp < MIN_SPREAD:
            return False, ("the two positions are too close together "
                           "(%.2fx, need %.2fx) -- the angular and constant "
                           "parts cannot be separated" % (sp or 1.0, MIN_SPREAD))
        return True, ""

    def solve(self):
        ok, why = self.can_finish()
        if not ok:
            self.msg = why
            return None
        near, far = (0, 1) if self.span[0] >= self.span[1] else (1, 0)
        r = aim_fit.split_offset(self.c0, self.q[near], self.off[near],
                                 self.q[far], self.off[far], FRAME_W, FRAME_H)
        if r is None:
            self.msg = "the split was refused (degenerate geometry)"
            return None
        return aim_fit.apply_offset(self.c0, *r)

    def solve_direct(self):
        """Save the CURRENT station's offset as-is: a plain constant, no
        angular/parallax split, no measurement required.

        This is the honest version of what one station can support. The nudge
        is exactly what the live preview has been showing, so what you saved is
        what you saw -- nothing is re-measured on the way out, which is also
        why it CANNOT double-stack with a ring shot: a shot only ever measures
        what is left AFTER the preview, and saving applies the preview itself.

        The cost is stated where the caller can show it: a constant is correct
        at every distance only if the true offset is parallax. If it is angular
        it will drift as you move; the two-station flow exists for that."""
        if self.stage > 1:
            return None
        d = self.off[self.stage]
        if (abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9
                and self.lead == self.lead0 and self.smooth == self.smooth0
                and self.beta == self.beta0):
            self.msg = "nothing to save yet -- nudge or shoot the ring first"
            return None
        return self.preview()


# ---------------------------------------------------------------------------
def run_gui(src, tuner, on_send):
    import tkinter as tk
    root = tk.Tk()
    root.title("Fine tune")
    root.configure(bg=C_BG)
    root.attributes("-fullscreen", True)
    SW = root.winfo_screenwidth(); SH = root.winfo_screenheight()
    cv = tk.Canvas(root, width=SW, height=SH, bg=C_BG, highlightthickness=0)
    cv.pack(fill="both", expand=True)

    st = {"last_send": 0.0, "dirty": True, "done": None, "quad": None,
          # Click feedback. Without it there is no way to tell a press that
          # registered from one that missed the button, which is exactly how a
          # working lead control felt like a dead one.
          "flash": None, "flash_t": 0.0, "toast": "", "toast_t": 0.0,
          "prev_q": None, "prev_t": 0.0, "vpx": 0.0, "recent": []}

    def text(x, y, s, size, fill, bold=False, anchor="center"):
        cv.create_text(x, y, text=s, anchor=anchor, fill=fill,
                       font=("DejaVu Sans", int(size), "bold" if bold else "normal"))

    # ---- the buttons, as canvas rectangles we hit-test ourselves ----------
    # BETA sits on the same bar as LEAD and SMOOTH: it is the third thing the
    # cursor's feel depends on, and it was only reachable from Studio's lens tab.
    # Width is derived from the count so adding a pair does not overrun the row.
    DEFS = (("◀", ("nudge", -1, 0)),
            ("▶", ("nudge", +1, 0)),
            ("▲", ("nudge", 0, -1)),
            ("▼", ("nudge", 0, +1)),
            ("LEAD −", ("lead", -1)),
            ("LEAD +", ("lead", +1)),
            ("SMOOTH −", ("smooth", -1)),
            ("SMOOTH +", ("smooth", +1)),
            ("BETA −", ("beta", -1)),
            ("BETA +", ("beta", +1)))
    BH, BY = SH * 0.11, SH * 0.03
    gap = SW * 0.010
    BW = (SW * 0.95 - (len(DEFS) - 1) * gap) / len(DEFS)
    total = len(DEFS) * BW + (len(DEFS) - 1) * gap
    x0 = (SW - total) / 2.0
    BUTTONS = []
    for i, (lab, act) in enumerate(DEFS):
        bx = x0 + i * (BW + gap)
        BUTTONS.append(dict(x0=bx, y0=BY, x1=bx + BW, y1=BY + BH, lab=lab, act=act))
    DONE = dict(x0=SW*0.29, y0=SH*0.86, x1=SW*0.49, y1=SH*0.94, lab="DONE", act=("done",))
    # The single-station escape hatch: nudge by eye against your own notch and
    # keep it, without the ring shot and without the second station.
    SAVE = dict(x0=SW*0.51, y0=SH*0.86, x1=SW*0.71, y1=SH*0.94, lab="SAVE", act=("save",))

    def hit(x, y):
        for b in BUTTONS + [DONE, SAVE]:
            if b["x0"] <= x <= b["x1"] and b["y0"] <= y <= b["y1"]:
                return b
        return None

    def say(msg, b=None):
        st["toast"] = msg; st["toast_t"] = time.time()
        st["flash"] = b; st["flash_t"] = time.time()

    def do(act, b=None):
        if act[0] == "nudge":
            tuner.nudge(act[1], act[2]); st["dirty"] = True
            d = tuner.off[tuner.stage] if tuner.stage < 2 else np.zeros(2)
            say("nudged  ->  %+.0f, %+.0f px" % (d[0]*1920.0, d[1]*1200.0), b)
        elif act[0] == "lead":
            before = tuner.lead
            tuner.nudge_lead(act[1])
            on_send("~cam=lead:%d" % tuner.lead)
            if tuner.lead == before:
                say("lead already at %d ms (range 0-%d)" % (tuner.lead, LEAD_MAX), b)
            else:
                say("lead  %d  ->  %d ms" % (before, tuner.lead), b)
        elif act[0] == "smooth":
            before = tuner.smooth
            tuner.nudge_smooth(act[1])
            on_send("~cam=smooth:%d" % tuner.smooth)
            if tuner.smooth == before:
                say("smoothing already at %d (range 0-%d)" % (tuner.smooth, SMOOTH_MAX), b)
            else:
                # smoothing lag adds AFTER the lead compensation, so a tuned
                # lead no longer matches once the level changes
                say("smoothing  %d  ->  %d   (re-check LEAD: its lag changed)"
                    % (before, tuner.smooth), b)
        elif act[0] == "beta":
            before = tuner.beta
            tuner.nudge_beta(act[1])
            on_send("~cam=beta:%d" % tuner.beta)
            if tuner.beta == before:
                say("speed sensitivity already at %s (range auto, 0-%d)"
                    % (tuner.beta_label(), BETA_MAX), b)
            else:
                say("speed sensitivity  %s  ->  %s   (rest jitter unchanged)"
                    % (("auto" if before < 0 else str(before)),
                       tuner.beta_label()), b)
            st["dirty"] = True
        elif act[0] == "save":
            c = tuner.solve_direct()
            if c is not None:
                tuner.stage = 2
                tuner.msg = ("saved as a constant offset (one position). If aim "
                             "drifts when you change distance, redo this with "
                             "both stations.")
                st["done"] = c
            else:
                # Without this the button looked dead: the refusal was written
                # to tuner.msg, which is only drawn once stage 2 is reached.
                say(tuner.msg or "nothing to save yet", b)
            st["dirty"] = True
        elif act[0] == "done":
            if tuner.stage == 0:
                ok = tuner.measured[0]
                if not ok:
                    tuner.msg = "shoot the ring here first"
                else:
                    tuner.stage = 1; tuner.msg = ""
                st["dirty"] = True
            elif tuner.stage == 1:
                c = tuner.solve()
                if c is not None:
                    tuner.stage = 2
                    st["done"] = c
                st["dirty"] = True

    def click(ev):
        b = hit(ev.x, ev.y)
        if b: do(b["act"], b)
        elif tuner.stage < 2 and len(st["recent"]) >= 4:
            # a shot that is not on a button IS the measurement. Median of the
            # last few frames, because one frame carries the full blob noise and
            # this number goes straight into the correction.
            q = np.median(np.array(st["recent"][-SHOT_FRAMES:]), axis=0)
            before = tuner.off[tuner.stage].copy()
            tuner.note_quad(q)
            d = tuner.off[tuner.stage] - before
            if not tuner.msg:
                say("measured  ->  %+.0f, %+.0f px" % (d[0]*1920.0, d[1]*1200.0))
            st["dirty"] = True

    cv.bind("<Button-1>", click)
    root.bind("<Escape>", lambda e: root.destroy())
    root.bind("<Left>",  lambda e: do(("nudge", -1, 0), BUTTONS[0]))
    root.bind("<Right>", lambda e: do(("nudge", +1, 0), BUTTONS[1]))
    root.bind("<Up>",    lambda e: do(("nudge", 0, -1), BUTTONS[2]))
    root.bind("<Down>",  lambda e: do(("nudge", 0, +1), BUTTONS[3]))
    root.bind("<minus>", lambda e: do(("lead", -1), BUTTONS[4]))
    root.bind("<plus>",  lambda e: do(("lead", +1), BUTTONS[5]))
    root.bind("<equal>", lambda e: do(("lead", +1), BUTTONS[5]))
    root.bind("<bracketleft>",  lambda e: do(("smooth", -1), BUTTONS[6]))
    root.bind("<bracketright>", lambda e: do(("smooth", +1), BUTTONS[7]))
    root.bind("<comma>",  lambda e: do(("beta", -1), BUTTONS[8]))
    root.bind("<period>", lambda e: do(("beta", +1), BUTTONS[9]))
    root.bind("<Return>", lambda e: do(("done",)))

    def push():
        """send the live preview, rate limited -- it is a serial link, not a bus"""
        if not st["dirty"]: return
        now = time.time()
        if now - st["last_send"] < 0.05: return
        st["last_send"] = now; st["dirty"] = False
        on_send("~" + aimcal_line(tuner.preview()).replace("aimcal=", "aimcal!=", 1))

    def draw():
        cv.delete("all")
        lit = (time.time() - st["flash_t"]) < 0.18
        for b in BUTTONS:
            on = lit and st["flash"] is b
            cv.create_rectangle(b["x0"], b["y0"], b["x1"], b["y1"],
                                fill="#2d6a43" if on else "#161b22",
                                outline=C_OK if on else "#30363d", width=3 if on else 2)
            # word labels need a smaller face than the arrows or they overrun
            # the box and the two LEAD buttons run into each other
            fs = SH*0.035 if len(b["lab"]) <= 2 else SH*0.019
            text((b["x0"]+b["x1"])/2, (b["y0"]+b["y1"])/2, b["lab"], fs, C_FG, bold=True)
        # What the lead is WORTH right now, from the gun's own motion. A number
        # in ms means nothing to a hand; "about 40 px at the speed you are
        # moving" is the thing you can actually judge against the cursor.
        worth = st["vpx"] * (tuner.lead / 1000.0) * 1920.0
        text(SW/2, BY + BH + SH*0.026,
             "lead %d ms (worth about %.0f screen px at your current speed)"
             "      smoothing %d / %d      speed sensitivity %s"
             % (tuner.lead, abs(worth), tuner.smooth, SMOOTH_MAX,
                tuner.beta_label()), SH*0.019,
             C_OK if (tuner.lead or tuner.smooth or tuner.beta >= 0) else C_DIM)
        text(SW/2, BY + BH + SH*0.055,
             "LEAD: raise while the cursor trails you, stop when reversals "
             "overshoot.   SMOOTHING: raise until the jitter settles, stop "
             "when the cursor starts to glide.", SH*0.016, C_DIM)
        text(SW/2, BY + BH + SH*0.078,
             "BETA (speed sensitivity): how fast the smoothing lets go once you "
             "move. Raise it if slow drags feel sticky; it does NOT shorten the "
             "trail on a fast swipe -- that is LEAD.", SH*0.016, C_DIM)
        if (time.time() - st["toast_t"]) < 1.6 and st["toast"]:
            text(SW/2, BY + BH + SH*0.112, st["toast"], SH*0.024, C_OK, bold=True)

        if tuner.stage == 2:
            text(SW/2, SH*0.42, "FINE TUNE APPLIED", SH*0.045, C_OK, bold=True)
            text(SW/2, SH*0.50, tuner.msg or "saved to the gun", SH*0.022, C_FG)
            text(SW/2, SH*0.56, "Esc to close", SH*0.018, C_DIM)
            return

        # the target you line your IRON SIGHTS up with
        cx, cy = SW*TARGET[0], SH*TARGET[1]
        r = SH*0.035
        cv.create_oval(cx-r, cy-r, cx+r, cy+r, outline=C_WARN, width=3)
        cv.create_line(cx-r*1.8, cy, cx+r*1.8, cy, fill=C_WARN, width=2)
        cv.create_line(cx, cy-r*1.8, cx, cy+r*1.8, fill=C_WARN, width=2)

        head = ("STATION 1 -- stand where you normally play" if tuner.stage == 0
                else "STATION 2 -- step well back, then do it again")
        text(SW/2, SH*0.53, head, SH*0.030, C_FG, bold=True)
        text(SW/2, SH*0.585,
             "Shoot the ring with your IRON SIGHTS to measure the offset -- or "
             "skip it: nudge with the arrows until the cursor sits on your "
             "notch, then SAVE NOW.", SH*0.019, C_DIM)
        text(SW/2, SH*0.615,
             "Mixing them is safe: a ring shot only measures what is LEFT after "
             "your nudges, so nothing is counted twice.", SH*0.016, C_DIM)
        d = tuner.off[tuner.stage]
        text(SW/2, SH*0.660, "offset %+.0f, %+.0f px"
             % (d[0]*1920.0, d[1]*1200.0), SH*0.024,
             C_OK if tuner.measured[tuner.stage] else C_DIM)
        text(SW/2, SH*0.705,
             "measured here -- shoot again to refine" if tuner.measured[tuner.stage]
             else "not measured here yet -- shoot the ring", SH*0.019,
             C_OK if tuner.measured[tuner.stage] else C_WARN)
        sp = tuner.spread()
        if tuner.stage == 1 and sp is not None:
            good = sp >= MIN_SPREAD
            text(SW/2, SH*0.750, "distance change %.2fx  (need %.2fx)" % (sp, MIN_SPREAD),
                 SH*0.020, C_OK if good else C_BAD)
        if tuner.msg:
            text(SW/2, SH*0.79, tuner.msg, SH*0.019, C_BAD)
        cv.create_rectangle(DONE["x0"], DONE["y0"], DONE["x1"], DONE["y1"],
                            fill="#161b22", outline="#30363d", width=2)
        text((DONE["x0"]+DONE["x1"])/2, (DONE["y0"]+DONE["y1"])/2,
             "NEXT" if tuner.stage == 0 else "FINISH", SH*0.026, C_FG, bold=True)
        cv.create_rectangle(SAVE["x0"], SAVE["y0"], SAVE["x1"], SAVE["y1"],
                            fill="#161b22", outline="#2d6a43", width=2)
        text((SAVE["x0"]+SAVE["x1"])/2, (SAVE["y0"]+SAVE["y1"])/2 - SH*0.011,
             "SAVE NOW", SH*0.022, C_OK, bold=True)
        text((SAVE["x0"]+SAVE["x1"])/2, (SAVE["y0"]+SAVE["y1"])/2 + SH*0.016,
             "keep this as-is, one position", SH*0.011, C_DIM)

    def tick():
        n = 0
        while n < 200:
            try: line = src.q.get_nowait()
            except queue.Empty: break
            n += 1
            pq = parse_q(line)
            if pq is not None:
                q = pq[0]; st["quad"] = q
                st["recent"].append(np.asarray(q)); del st["recent"][:-SHOT_FRAMES]
                # rough screen-widths/sec of the aim point, for the "worth"
                # readout. Rough is fine: it is a feel gauge, not a measurement.
                # Solve BOTH quads through the calibration and difference the
                # SCREEN positions. Differencing camera pixels and scaling by
                # the frame width was wrong by the whole master gain -- about
                # 3x low -- because a camera pixel is worth ~26 screen px here,
                # not 1920/240.
                now = time.time()
                if st["prev_q"] is not None and now > st["prev_t"]:
                    dt = now - st["prev_t"]
                    if 0.002 < dt < 0.2:
                        a = aim_fit.solve(tuner.preview(), st["prev_q"], FRAME_W, FRAME_H)
                        b = aim_fit.solve(tuner.preview(), q, FRAME_W, FRAME_H)
                        if a and b:
                            v = float(np.hypot(b[0]-a[0], b[1]-a[1])) / dt
                            st["vpx"] = 0.8*st["vpx"] + 0.2*v
                st["prev_q"] = q; st["prev_t"] = now
        push()
        draw()
        root.after(16, tick)

    root.after(16, tick)
    root.mainloop()
    return st["done"]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()
    port = a.port or find_gun(a.baud)
    if not port:
        sys.exit("no gun found. Run tools/aim_probe.py for a diagnosis.")
    src = SerialSource(port, a.baud)
    src.start()
    time.sleep(0.6)
    src.replies.clear()
    src.ser.write(b"\n~aimcal?\n")
    time.sleep(1.0)
    cal = None
    for r in src.replies:
        if r.startswith("AIM:") and "cx=" in r:
            g = {}
            for tok in r.replace("AIM:", "").split():
                if "=" in tok:
                    k, _, v = tok.partition("=")
                    try: g[k] = float(v)
                    except ValueError: pass
            if all(k in g for k in ("cx", "cy", "w", "h", "bx", "by")):
                cal = dict(magic=aim_fit.MAGIC, cx=g['cx'], cy=g['cy'], w=g['w'],
                           h=g['h'], bx=g['bx'], by=g['by'],
                           lever=g.get('lever', 0.0), rx=g.get('rx', 0.0),
                           ry=g.get('ry', 0.0))
    if cal is None:
        sys.exit("the gun has no calibration loaded -- run tools/aim_calib.py first.")
    print("current: %s" % aimcal_line(cal))

    # Seed lead and smoothing from the gun, so the first nudge steps from the
    # saved value instead of resetting it.
    src.replies.clear()
    src.ser.write(b"\n~cam?\n")
    time.sleep(0.8)
    lead0, smooth0, beta0 = 0, 3, -1
    for r in src.replies:
        if "lead=" not in r:
            continue
        for tok in r.replace("CAM:", "").split():
            k, _, v = tok.partition("=")
            try:
                if k == "lead":     lead0 = int(float(v.rstrip("ms")))
                elif k == "smooth": smooth0 = int(float(v))
                elif k == "beta":   beta0 = int(float(v))
            except ValueError:
                pass
    print("gun reports lead=%d ms smooth=%d beta=%s"
          % (lead0, smooth0, "auto" if beta0 < 0 else beta0))

    def send(line):
        try: src.ser.write(("\n%s\n" % line).encode())
        except Exception: pass

    tuner = Tuner(cal, lead0, smooth0, beta0)
    out = run_gui(src, tuner, send)
    if out is None:
        send("~" + aimcal_line(cal))         # put the original back, saved
        send("~cam=lead:%d,smooth:%d,beta:%d"
             % (tuner.lead0, tuner.smooth0, tuner.beta0))
        print("cancelled; original calibration, lead, smoothing and beta restored")
        return
    print(install_over_serial(src, aimcal_line(out), out))
    ok, msg = camsave_verified(src, lead=tuner.lead, smooth=tuner.smooth,
                               beta=tuner.beta)
    print(("lead, smoothing and beta: " if ok else "SAVE PROBLEM -- ") + msg)


if __name__ == "__main__":
    main()
