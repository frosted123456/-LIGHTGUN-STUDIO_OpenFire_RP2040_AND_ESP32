#!/usr/bin/env python3
"""aim_calib.py -- on-screen calibration capture for the IR lightgun.

    python3 tools/aim_calib.py --port COM7     # real gun
    python3 tools/aim_calib.py --sim           # synthetic, for testing the UI
    python3 tools/aim_calib.py --selftest      # logic only, no display

Aim at each target and pull the trigger (space or click also work), step back when
asked. Requires pyserial, numpy, tkinter.
"""
import argparse, json, os, queue, sys, threading, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aim_fit

FRAME_W, FRAME_H = 240.0, 176.0     # HQVGA, must match the firmware
CAPTURE_MS   = 1300                 # per dot
DWELL_MS     = 900                  # hold this still and it captures itself
# Native px the quad centroid may range over within the dwell window and still
# count as "still".
DWELL_PX     = 5.0
ARM_PX       = 12.0                 # you must MOVE this far before a dwell can arm
ARM_TIMEOUT  = 3.0                  # ...or just hold this long, for the first target
# Trigger pulls accumulated per target before it advances. Combined by MEDIAN,
# so one flyer cannot drag a dot.
PULLS_PER_DOT = 4
AUTO_AFTER_S = 12.0                 # no trigger by now -> offer hold-still capture
REVIEW_S     = 3.0                  # a rejected capture retries itself after this
# Trigger markers arriving within this long of entering AIM are ignored: they
# were pulled while the previous screen was up.
TRIG_DEAD_S  = 0.7
STEPBACK_S   = 1.5                  # sustained distance change before auto-continue
# Roll stances. The fit's sin(roll) term needs tilt DIVERSITY to be identifiable,
# so ask for a modest tilt each way.
ROLL_TARGET_DEG = 12.0              # what we ask for
ROLL_MIN_DEG    = 8.0               # what we accept before continuing
ROLL_S          = 1.2               # sustained tilt before auto-continue
MIN_FRAMES   = 30                   # of 4-LED frames needed to accept a dot
DRIFT_REJECT = 12.0                 # native px of centroid travel -> "you moved"
# How long after a REAL trigger pull the capture ignores frames: the solenoid
# strike, its after-pulses and the hand's ring-down all land in this window,
# and counting them measured the gun's own recoil as "you moved". Auto-dwell
# captures pull nothing, so they skip the blank entirely.
RECOIL_BLANK_MS = 300.0
# EDGE MARGIN. An LED whose blob is partly outside the sensor has its centroid
# pulled inward, so the quad quietly shrinks on that side and the fit is
# corrupted. Checked on the MEDIAN quad of the pull, plus a mostly-clipped guard.
EDGE_MARGIN = 5.0
EDGE_BAD_FRAC = 0.40                # of the hold's frames below the margin
GAIN_HINT    = 25.6                 # screen px per image px, for the status line
# Sanity-check inputs only. RIG_ASPECT is the physical LED rectangle aspect
# (width/height); RIG_H_CM turns the fitted rectangle into a tape-measurable
# width: implied_width = display_aspect * w/h * height. No screen aspect is
# hardcoded -- it is read from the display at run time.
RIG_H_CM     = 40.2
FOCAL_PX     = 184.7                # measured; used only by the rig-aspect check
RIG_ASPECT   = 0.208/0.402

# Targets: corners pulled in enough to be comfortably aimable, plus the centre.
DOTS = [(0.08,0.08),(0.92,0.08),(0.92,0.92),(0.08,0.92),(0.50,0.50)]
DOT_NAMES = ["top-left","top-right","bottom-right","bottom-left","centre"]

def aimcal_line(c):
    """The one line the whole session exists to produce."""
    base = ("aimcal=%.6f,%.6f,%.6f,%.6f,%.3f,%.3f"
            % (c['cx'], c['cy'], c['w'], c['h'], c['bx'], c['by']))
    rx, ry = c.get('rx', 0.0), c.get('ry', 0.0)
    if rx or ry:
        # position 7 is the lever; the parser needs it present to reach 8 and 9
        base += ",%.6f,%.6f,%.6f" % (c.get('lever', 0.0), rx, ry)
    return base


def _rig_line(session):
    """What the images say about the rig itself, independent of the calibration."""
    med, drift = session.rig_aspect()
    if med is None: return "not enough shots"
    txt = "height/width %.3f" % med
    if drift is not None:
        txt += ",  drifted %+.1f%% during this run" % (100*drift)
        if abs(drift) > 0.015:
            txt += "  <- the rig is MOVING; check the mounting"
    return txt


_LAST_STATE = None


def install_over_serial(src, cmd, c, timeout=2.5):
    """Send the calibration and read it back to confirm the gun took it."""
    try:
        src.replies.clear()
        src.ser.write(("\n~" + cmd.lstrip("~") + "\n").encode())
        time.sleep(0.4)
        src.ser.write(b"\n~aimcal?\n")
    except Exception as e:
        return "Could not auto-send (%s); send it by hand:\n  ~%s" % (e, cmd.lstrip("~"))
    t0 = time.time()
    while time.time() - t0 < timeout:
        for r in list(src.replies):
            # AIM:-only: a CAM: line's "lcxu=" token contains "cx=" and would
            # otherwise be parsed as a calibration read-back
            if not r.startswith("AIM:") or "cx=" not in r: continue
            got = {}
            for tok in r.replace("AIM:", "").split():
                if "=" in tok:
                    k, _, v = tok.partition("=")
                    try: got[k] = float(v)
                    except ValueError: pass
            want_rx, want_ry = c.get('rx', 0.0), c.get('ry', 0.0)
            ok = (abs(got.get('cx', 9e9) - c['cx']) < 2e-4 and
                  abs(got.get('bx', 9e9) - c['bx']) < 2e-2 and
                  abs(got.get('rx', 9e9) - want_rx) < 2e-4 and
                  abs(got.get('ry', 9e9) - want_ry) < 2e-4)
            if ok:
                return ("INSTALLED and verified on the gun (read back cx=%.5f bx=%.3f "
                        "rx=%+.5f ry=%+.5f)" % (got.get('cx', 0), got.get('bx', 0),
                                                got.get('rx', 0), got.get('ry', 0)))
            return ("SENT BUT THE GUN DISAGREES -- it reports %s\n"
                    "  expected cx=%.5f bx=%.3f rx=%+.5f ry=%+.5f\n"
                    "  send it by hand and check:  ~%s"
                    % (r.strip(), c['cx'], c['bx'], want_rx, want_ry, cmd.lstrip("~")))
        time.sleep(0.05)
    return ("NO REPLY from the gun in %.1fs -- the calibration is probably NOT "
            "installed.\n  Send it by hand and confirm with ~aimcal?:\n  ~%s"
            % (timeout, cmd.lstrip("~")))


def camsave_verified(src, timeout=2.5, **want):
    """Persist the live camera settings and CHECK what the gun actually wrote.

    Two separate facts, and the caller is told both. The gun answers ~camsave
    with "CAM: saved ..." or "CAM: SAVE FAILED ...", which is the flash write's
    own result, and that same line carries the values that were written -- so
    an expected lead, smoothing or beta is COMPARED against them instead of
    assumed. An optimistic "saved" message printed over a write that never
    happened is how a whole tuning session gets lost.

    Returns (ok, message). Keys in `want` that the firmware does not report are
    named in the message rather than treated as a mismatch, so an older build
    reports "saved" honestly instead of failing on a field it never had.
    """
    if src is None:
        return False, "no gun connected -- NOT saved"
    try:
        src.replies.clear()
        src.ser.write(b"\n~camsave\n")
    except Exception as e:
        return False, "could not reach the gun (%s) -- NOT saved" % e
    t0 = time.time()
    while time.time() - t0 < timeout:
        for r in list(src.replies):
            if not (r.startswith("CAM: saved")
                    or r.startswith("CAM: SAVE FAILED")):
                continue
            if r.startswith("CAM: SAVE FAILED"):
                return False, "the gun REFUSED the save -- %s" % r.strip()
            got = {}
            for tok in r.split():
                k, sep, v = tok.partition("=")
                if not sep:
                    continue
                # lead is reported with its unit, as "lead=30ms"
                try: got[k] = int(v[:-2] if v.endswith("ms") else v)
                except ValueError: pass
            bad = ["%s is %d, asked %d" % (k, got[k], int(v))
                   for k, v in sorted(want.items())
                   if k in got and got[k] != int(v)]
            if bad:
                return False, ("the gun saved DIFFERENT values -- %s"
                               % "; ".join(bad))
            shown = ", ".join("%s=%d" % (k, got[k])
                              for k in sorted(want) if k in got)
            msg = "SAVED to the gun and read back"
            if shown:
                msg += ": " + shown
            missing = sorted(k for k in want if k not in got)
            if missing:
                msg += ("  (this firmware does not report %s in its save reply)"
                        % ", ".join(missing))
            return True, msg
        time.sleep(0.05)
    return False, ("NO REPLY to ~camsave in %.1fs -- treat the settings as NOT "
                   "saved and try again" % timeout)


def make_plan(n_dist=3, n_roll=2):
    """The stance list: distances first, then the rolled stances."""
    plan = [dict(kind='dist', roll=0) for _ in range(n_dist)]
    rolls = [dict(kind='roll', roll=+1), dict(kind='roll', roll=-1)]
    return plan + rolls[:n_roll]

# What the camera model predicts for the reference rig, so the app can say
# outright whether the model checks out.
PREDICT_W, PREDICT_H = 0.35, 1.20


def sigma_from_hold(frames):
    """Blob centroid sigma with hand tremor removed (similarity fit, dof-corrected)."""
    a = np.asarray(frames)
    if len(a) < 8: return float('nan')
    ref = np.median(a, axis=0); rc = ref - ref.mean(0)
    d = (rc*rc).sum()
    if d < 1e-9: return float('nan')
    res = []
    for f in a:
        fc = f - f.mean(0)
        p = (rc[:,0]*fc[:,0] + rc[:,1]*fc[:,1]).sum()/d
        q = (rc[:,0]*fc[:,1] - rc[:,1]*fc[:,0]).sum()/d
        res.append(fc - np.stack([p*rc[:,0]-q*rc[:,1], q*rc[:,0]+p*rc[:,1]], 1))
    return float(np.concatenate(res).std())/np.sqrt(0.5)


class CaptureSession:
    """All the state and validation. No GUI, so it is testable headlessly."""
    S_AIM, S_CAPTURING, S_REVIEW, S_STEPBACK, S_DONE = range(5)

    def __init__(self, stances=2, dots=None, plan=None):
        # `plan` supersedes `stances`; the int form is kept so older headless
        # tests still construct.
        self.plan = plan if plan is not None else [dict(kind='dist', roll=0)
                                                   for _ in range(stances)]
        self.stances = len(self.plan)
        self.dots = dots or DOTS
        self.stance = 0
        self.idx = 0
        self.state = self.S_AIM
        self.buf = []
        self.pulls = []            # accepted captures for the CURRENT target
        self.t0 = 0.0
        self.shots = []          # accepted: dict(q, tx, ty, sigma, span, stance)
        self.raw = []            # every Q line, saved for offline re-analysis
        self.msg = ""
        self.last_result = None
        self.live_span = None      # EMA of the incoming quad span, for live feedback
        self.live_q = None         # most recent quad, for the on-screen live view
        self.live_t = 0.0
        self.fps = 0.0
        # The trigger is the PRIMARY confirmation. Auto-capture is a FALLBACK,
        # armed only if no trigger has ever arrived.
        self.auto = False
        self.auto_reason = ""
        self.seen_trigger = False
        self.first_t = None
        self.geom_note = ""     # display geometry, saved with the shots for audit
        self.cen_hist = []         # (t, cx, cy) rolling window for dwell detection
        self.dwell = 0.0           # 0..1 progress toward an auto-capture
        self.armed = False         # has the user moved to this target yet?
        self.arm_ref = None        # centroid when this target became active
        self.arm_t = 0.0
        self.state_t = 0.0         # when the current state was entered
        self.sb_ok_since = None
        self.roll_ref = None       # mean sin(roll) of the level stances
        self.marg_hist = []        # recent frame-edge margins, for the live check
        self.trig_deaf_until = 0.0 # ignore trigger markers until this time

    # ---- flow -----------------------------------------------------------
    def target(self):
        """where to DRAW the reticle, as a fraction of the window"""
        return self.dots[self.idx]

    # Set by the GUI to convert window fractions to screen fractions. Identity
    # for the headless selftests.
    to_screen = staticmethod(lambda fx, fy: (fx, fy))

    def target_screen(self):
        """where the reticle actually IS, as a fraction of the screen"""
        return self.to_screen(*self.dots[self.idx])

    def target_name(self):
        return DOT_NAMES[self.idx] if self.dots is DOTS else "dot %d" % (self.idx+1)

    def note_trigger(self):
        """a real trigger arrived -- stop offering auto-capture"""
        self.seen_trigger = True
        self.auto = False
        self.auto_reason = ""

    def consider_auto(self, now):
        """Enable auto-capture only if no trigger has ever been seen."""
        if self.seen_trigger or self.auto: return
        if self.live_t and (self.live_t - self.first_t) > AUTO_AFTER_S:
            self.auto = True
            self.auto_reason = ("no trigger seen in %.0fs -- falling back to "
                                "hold-still capture" % AUTO_AFTER_S)

    def trigger(self, now, pulled=False):
        # Dead time after a state change: a pull made while the previous screen
        # was up arrives just after the flip and would capture un-aimed.
        # `pulled` marks a REAL trigger pull -- one that fires the solenoid --
        # so the capture can blank the recoil transient. The auto-dwell path
        # and the simulators leave it False and behave exactly as before.
        if now < self.trig_deaf_until:
            return
        if self.state == self.S_AIM:
            self.buf = []; self.t0 = now; self.state = self.S_CAPTURING
            self.blank_until = now + (RECOIL_BLANK_MS / 1000.0 if pulled else 0.0)
            self.cen_hist = []; self.dwell = 0.0; self.state_t = now
        elif self.state == self.S_REVIEW:
            self._advance()
        elif self.state == self.S_STEPBACK:
            # Through _enter, so the trigger dead time re-arms: one physical
            # pull arrives twice in pical (serial marker + HID click), and the
            # bare assignment let the second copy start an un-aimed capture.
            self._enter(self.S_AIM, now)

    def feed(self, quad, now):
        """quad: (4,2) native px. Called for every 4-LED frame."""
        self.raw.append(quad)
        if self.first_t is None: self.first_t = now
        self.consider_auto(now)
        # live view + frame rate, maintained regardless of state
        if self.live_t:
            dt = now - self.live_t
            if dt > 0: self.fps = 0.9*self.fps + 0.1*(1.0/dt)
        self.live_t = now
        prev = self.live_q
        self.live_q = quad
        sp = aim_fit.quad_span(quad)
        self.live_span = sp if self.live_span is None else 0.85*self.live_span + 0.15*sp
        self.marg_hist.append(self.edge_margin(quad))
        del self.marg_hist[:-40]
        # AUTO-CAPTURE: holding still on the target starts the capture by itself,
        # so the procedure works with no trigger at all.
        a = np.asarray(quad)
        self.cen_hist.append((now, float(a[:,0].mean()), float(a[:,1].mean())))
        win = DWELL_MS / 1000.0
        # Keep MORE than one window: pruning to exactly `win` means held/win can
        # never reach 1.0 and the capture never fires.
        while self.cen_hist and now - self.cen_hist[0][0] > win * 2.0:
            self.cen_hist.pop(0)
        # ARM ON ARRIVAL, then fire on stillness. Stillness alone is not enough --
        # the gun is still whenever it is held steady, anywhere.
        cen_now = (float(a[:,0].mean()), float(a[:,1].mean()))
        if self.state == self.S_AIM and not self.armed:
            if self.arm_ref is None:
                self.arm_ref = cen_now; self.arm_t = now
            moved = max(abs(cen_now[0]-self.arm_ref[0]), abs(cen_now[1]-self.arm_ref[1]))
            # escape hatch for the first target, where the user may already be on it
            if moved > ARM_PX or (now - self.arm_t) > ARM_TIMEOUT:
                self.armed = True
                self.cen_hist = [c for c in self.cen_hist if now - c[0] <= 0.1]
        if self.state == self.S_AIM and self.auto and self.armed:
            w = [c for c in self.cen_hist if now - c[0] <= win]
            if len(w) >= 8:
                xs = [c[1] for c in w]; ys = [c[2] for c in w]
                rng = max(max(xs) - min(xs), max(ys) - min(ys))
                held = w[-1][0] - w[0][0]
                if rng < DWELL_PX:
                    self.dwell = min(1.0, held / (win * 0.9))
                    if self.dwell >= 1.0:
                        self.cen_hist = []; self.dwell = 0.0
                        self.trigger(now)
                else:
                    self.dwell = 0.0
        # Every state advances itself; the trigger is only ever a shortcut.
        if self.state == self.S_REVIEW and (now - self.state_t) > REVIEW_S:
            self._enter(self.S_AIM, now); return
        if self.state == self.S_STEPBACK:
            if self.stance_kind() == 'roll':
                d = self.roll_check()
                want = self.stance_roll()
                ok = d is not None and (d*want) > 0 and abs(d) >= ROLL_MIN_DEG
                hold = ROLL_S
            else:
                r = self.stepback_check()
                ok = r is not None and r >= 1.15
                hold = STEPBACK_S
            if ok:
                if self.sb_ok_since is None: self.sb_ok_since = now
                elif (now - self.sb_ok_since) > hold:
                    self._enter(self.S_AIM, now)
            else:
                self.sb_ok_since = None
            return
        if self.state != self.S_CAPTURING: return
        # Recoil blanking: frames while the strike shakes the gun are not aim.
        # The window extends past the blank so the frame budget is unchanged.
        if now < getattr(self, "blank_until", 0.0):
            return
        self.buf.append(quad)
        blank = max(0.0, getattr(self, "blank_until", 0.0) - self.t0)
        if (now - self.t0 - blank)*1000.0 >= CAPTURE_MS:
            self._finish(now)

    def progress(self, now):
        if self.state != self.S_CAPTURING: return 0.0
        blank = max(0.0, getattr(self, "blank_until", 0.0) - self.t0)
        return min(1.0, max(0.0, (now - self.t0 - blank))*1000.0/CAPTURE_MS)

    def _finish(self, now):
        a = np.asarray(self.buf)
        # SETTLING MARGIN. Discard the leading fraction of the buffer: frames in
        # flight when the capture began can predate the move to this target.
        if len(a) >= MIN_FRAMES + 8:
            a = a[max(4, len(a)//8):]
        if len(a) < MIN_FRAMES:
            self.msg = ("Only %d frames (need %d). Are all four LEDs visible?"
                        % (len(a), MIN_FRAMES))
            self.last_result = None; self._enter(self.S_REVIEW, now); return
        cen = a.mean(1)
        drift = float(max(np.ptp(cen[:,0]), np.ptp(cen[:,1])))
        med = np.median(a, axis=0)
        sg = sigma_from_hold(a)
        span = aim_fit.quad_span(med)
        if drift > DRIFT_REJECT:
            self.msg = "You moved %.1f px during the capture (limit %.1f)." % (drift, DRIFT_REJECT)
            self.last_result = None; self._enter(self.S_REVIEW, now); return
        marg = self.edge_margin(med)
        frac_bad = float(np.mean([self.edge_margin(f) < EDGE_MARGIN for f in a]))
        if marg < EDGE_MARGIN or frac_bad > EDGE_BAD_FRAC:
            self.msg = ("The LEDs are running off the edge of the camera "
                        "(%.0f px margin, need %.0f; %.0f%% of frames clipped). "
                        "Step back a little." % (marg, EDGE_MARGIN, 100*frac_bad))
            self.last_result = None; self._enter(self.S_REVIEW, now); return
        self.pulls.append(dict(q=med, sigma=sg, span=span, frames=len(a),
                               drift=drift, edge=marg))
        self.last_result = dict(sigma=sg, span=span, drift=drift, frames=len(a),
                                edge=marg, pull=len(self.pulls), of=PULLS_PER_DOT)
        self.msg = ""
        if len(self.pulls) < PULLS_PER_DOT:
            # same target again -- back to waiting, re-armed so the user has to settle
            self._enter(self.S_AIM, now)
            return
        # median across pulls, per corner, per axis
        stack = np.asarray([p['q'] for p in self.pulls])       # (N,4,2)
        med_q = np.median(stack, axis=0)
        tx, ty = self.target_screen()
        self.shots.append(dict(q=med_q, tx=tx, ty=ty,
                               sigma=float(np.median([p['sigma'] for p in self.pulls])),
                               span=float(np.median([p['span'] for p in self.pulls])),
                               stance=self.stance, roll=aim_fit.quad_roll_sin(med_q),
                               frames=sum(p['frames'] for p in self.pulls),
                               drift=max(p['drift'] for p in self.pulls),
                               pulls=len(self.pulls),
                               # spread across pulls IS the user's hand steadiness
                               hand=float(np.max(np.std(stack.reshape(len(stack),-1), axis=0))
                                          if len(stack) > 1 else 0.0)))
        self.pulls = []
        self._advance()

    def _enter(self, st, now=None):
        # NB: does NOT touch self.pulls -- a rejected or retried capture must not
        # discard the pulls already banked for this target.
        self.state = st
        self.state_t = now if now is not None else self.live_t
        self.armed = False; self.arm_ref = None; self.dwell = 0.0
        self.cen_hist = []; self.sb_ok_since = None
        # a pull that was already in flight belongs to the state we just left
        if st == self.S_AIM:
            self.trig_deaf_until = self.state_t + TRIG_DEAD_S

    def _advance(self):
        # If a dot failed we come back here on the next trigger; retry the same dot.
        if self.last_result is None and self.state == self.S_REVIEW:
            self._enter(self.S_AIM); return
        self.pulls = []
        self.idx += 1
        if self.idx < len(self.dots):
            self._enter(self.S_AIM); return
        self.idx = 0; self.stance += 1
        if self.stance >= self.stances:
            self.state = self.S_DONE
        else:
            self._enter(self.S_STEPBACK)

    # ---- live feedback ---------------------------------------------------
    def stepback_check(self):
        """Live distance-change feedback: previous round's mean span vs the span now."""
        if self.stance == 0 or self.live_span is None: return None
        prev = [s['span'] for s in self.shots if s['stance'] == self.stance-1]
        if not prev: return None
        cur = [s['span'] for s in self.shots if s['stance'] == self.stance]
        ref = np.mean(cur) if cur else self.live_span
        if ref < 1e-6: return None
        r = float(np.mean(prev)/ref)
        return r if r >= 1.0 else 1.0/r

    @staticmethod
    def edge_margin(q):
        """smallest distance from any corner to the frame edge, native px"""
        a = np.asarray(q)
        return float(min(a[:,0].min(), FRAME_W-a[:,0].max(),
                         a[:,1].min(), FRAME_H-a[:,1].max()))

    def too_close(self):
        """Live edge-margin check, median of the recent frames."""
        if not self.marg_hist: return None
        return float(np.median(self.marg_hist[-20:]))

    def stance_kind(self, i=None):
        i = self.stance if i is None else i
        return self.plan[i]['kind'] if 0 <= i < len(self.plan) else 'dist'

    def stance_roll(self, i=None):
        i = self.stance if i is None else i
        return self.plan[i]['roll'] if 0 <= i < len(self.plan) else 0

    def roll_check(self):
        """Live tilt in degrees relative to the level stances -- a CHANGE, not an absolute."""
        if self.live_q is None: return None
        if self.roll_ref is None:
            base = [s['roll'] for s in self.shots if self.stance_kind(s['stance']) == 'dist']
            if not base: return None
            self.roll_ref = float(np.mean(base))
        cur = aim_fit.quad_roll_sin(self.live_q)
        return float(np.degrees(np.arcsin(np.clip(cur - self.roll_ref, -1.0, 1.0))))

    def rig_aspect(self):
        """(median, drift) of the LED rig's own height/width; reported, not gated."""
        if len(self.shots) < 4: return None, None
        a = np.array([aim_fit.rect_aspect(s['q'], FOCAL_PX) for s in self.shots])
        a = a[np.isfinite(a)]
        if len(a) < 4: return None, None
        h = len(a)//2
        return float(np.median(a)), float((np.median(a[h:]) - np.median(a[:h])) / np.median(a[:h]))

    def spread(self):
        return aim_fit.span_spread(self.shots) if len(self.shots) >= 2 else 1.0

    def roll_spread(self):
        return aim_fit.roll_spread(self.shots) if len(self.shots) >= 2 else 0.0

    def fit(self):
        return aim_fit.fit(self.shots, FRAME_W, FRAME_H)

    def save(self, outdir):
        outdir = os.path.abspath(outdir)
        os.makedirs(outdir, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        shp = os.path.join(outdir, "shots-%s.txt" % stamp)
        with open(shp, "w") as f:
            f.write("%g %g\n" % (FRAME_W, FRAME_H))
            if self.geom_note: f.write("# %s\n" % self.geom_note)
            for s in self.shots:
                f.write("%.4f %.4f " % (s['tx'], s['ty']) +
                        " ".join("%.3f" % v for v in np.asarray(s['q']).reshape(-1)) + "\n")
        rawp = os.path.join(outdir, "rawquads-%s.csv" % stamp)
        with open(rawp, "w") as f:
            f.write("x0,y0,x1,y1,x2,y2,x3,y3\n")
            for q in self.raw:
                f.write(",".join("%.2f" % v for v in np.asarray(q).reshape(-1)) + "\n")
        return shp, rawp


# ===========================================================================
# line sources
# ===========================================================================
def is_trigger(line):
    """T,<ms> -- the trigger press, in the SAME stream as the quads."""
    return line.startswith('T,')


def parse_q(line):
    if not line.startswith('Q,'): return None
    f = line.strip().split(',')
    if len(f) < 11: return None
    try:
        t = int(f[1])                      # the gun's own millisecond clock
        if int(f[2]) != 4: return None
        v = [int(x) for x in f[3:11]]
    except ValueError:
        return None
    if min(v) < 0: return None
    # Returns (quad, gun_time_seconds). The gun's own clock is used for dwell,
    # capture duration and frame rate -- NOT the PC's wall clock at drain time.
    return np.array(v, float).reshape(4, 2)/10.0, t/1000.0


def find_gun(baud=115200):
    """Ask each serial port '~ping' and return the one that answers."""
    import serial
    from serial.tools import list_ports
    for p in list_ports.comports():
        try:
            s = serial.Serial()
            s.port = p.device; s.baudrate = baud; s.timeout = 0.15
            s.dtr = True; s.rts = False
            s.open()
        except Exception:
            continue
        try:
            time.sleep(0.3)
            s.reset_input_buffer()
            for _ in range(3):
                s.write(b"\n~ping\n")
                t0 = time.time(); buf = b""
                while time.time() - t0 < 0.8:
                    buf += s.read(128)
                    if b"pong" in buf:
                        s.close()
                        return p.device
        except Exception:
            pass
        try: s.close()
        except Exception: pass
    return None


class SerialSource(threading.Thread):
    def __init__(self, port, baud=115200):
        super().__init__(daemon=True)
        import serial
        self.q = queue.Queue(maxsize=4000)
        # DTR asserted, RTS low: a USB-CDC host must assert DTR for the device to
        # consider it connected, but an RTS+DTR toggle can reset the chip.
        self.ser = serial.Serial()
        self.ser.port = port; self.ser.baudrate = baud; self.ser.timeout = 0.2
        self.ser.dtr = True; self.ser.rts = False
        self.ser.open()
        time.sleep(0.3)
        # The '~' dialect, which the firmware claims out of OpenFIRE's Serial
        # stream; a BARE "dash=2" only works on the CH340/UART0 path. Re-sent by
        # run() because a command issued before the gun's reader is ready is dropped.
        self.ser.write(b"~cam=dash:2\n~aimcap=1\n")
        self._armed = 0.0
        self.stop = False
        self.replies = []          # AIM: lines, so an install can be VERIFIED

    def run(self):
        buf = b""
        last_arm = 0.0
        last_q = 0.0
        while not self.stop:
            # keep asking until the stream actually starts. A healthy stream
            # (a Q line in the last 2 s) is left alone -- the old queue-empty
            # test misfired whenever the GUI drained faster than lines arrived,
            # re-sending the command forever.
            now = time.time()
            if (now - last_q) > 2.0 and (now - last_arm) > 2.0:
                last_arm = now
                try: self.ser.write(b"~cam=dash:2\n~aimcap=1\n")
                except Exception: pass
            # Read what has ARRIVED, not a fixed 512 bytes: read(512) blocks
            # until 512 bytes accumulate or the timeout fires, which at Q-line
            # rates is a ~200 ms burst -- the live view then updates at ~5 Hz
            # and feels laggy. Drain in_waiting, with a 1-byte blocking read
            # to idle on when the port is quiet.
            try:
                nwait = self.ser.in_waiting
                buf += self.ser.read(nwait if nwait > 0 else 1)
            except Exception: break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                txt = line.decode("ascii", "replace")
                if txt.startswith("AIM:") or txt.startswith("CAM:"):
                    # CAM: included: fine-tune seeds lead/smoothing from cam?,
                    # and an AIM:-only filter silently dropped that reply --
                    # the tool then started from defaults and the first nudge
                    # stomped the saved values.
                    self.replies.append(txt)
                    del self.replies[:-40]
                elif txt.startswith("Q,"):
                    last_q = time.time()
                try: self.q.put_nowait(txt)
                except queue.Full: pass

    def close(self):
        self.stop = True
        try:
            self.ser.write(b"~aimcap=0\n~cam=dash:0\n"); self.ser.close()
        except Exception: pass


class SimSource(threading.Thread):
    """TEST ONLY. Synthesises Q-lines from a known-truth camera model so the flow
       can be exercised without hardware."""
    def __init__(self, session, blob_sigma=0.6, tremor_deg=0.28,
                 bore=(5.0,-3.0), rig=(0.208,0.402), screen_w=0.597,
                 dists=(1.2, 1.7, 2.3), fpx=184.7, auto_trigger=False,
                 stance_bias_deg=0.35, dot_bias_deg=0.25, lever_m=0.06):
        super().__init__(daemon=True)
        self.q = queue.Queue(maxsize=4000)
        self.s = session; self.bs = blob_sigma; self.tr = tremor_deg
        self.bore = bore; self.rig = rig; self.sw = screen_w
        self.sh = screen_w*9.0/16.0; self.dists = dists; self.fpx = fpx
        self.stop = False; self.drift = np.zeros(2); self.rng = np.random.default_rng(7)
        # SYSTEMATIC aim bias, as opposed to tremor: consistent while stance and
        # dot stay the same, so it does not average out over four pulls.
        self.sb = stance_bias_deg; self.db = dot_bias_deg
        # How far the camera sits off the user's sight line, in metres. 0 is a
        # valid build: then there is no roll error to find.
        self.lever_m = lever_m
        brng = np.random.default_rng(1000 + int(blob_sigma*1000))
        self._stance_bias = brng.normal(0, np.deg2rad(self.sb), (12, 2))
        self._dot_bias    = brng.normal(0, np.deg2rad(self.db), (16, 2))
        self.truth = dict(bore=bore, rig_w_frac=rig[0]/screen_w, rig_h_frac=rig[1]/self.sh)

    def _quad(self, tx, ty, dist, roll=0.0, stance=None, dot=None):
        """roll: radians about the aim axis; stance/dot index the systematic aim bias."""
        hw, hh = self.rig[0]/2, self.rig[1]/2
        L = np.array([[-hw,+hh,0],[+hw,+hh,0],[-hw,-hh,0],[+hw,-hh,0]])
        self.drift = 0.90*self.drift + self.rng.normal(0, 0.35, 2)
        bias = np.zeros(2)
        if stance is not None: bias = bias + self._stance_bias[stance % 12]
        if dot    is not None: bias = bias + self._dot_bias[dot % 16]
        jit = np.tan(np.deg2rad(self.drift*self.tr + self.rng.normal(0, self.tr*0.3, 2))
                     + bias)*dist
        T = np.array([(tx-0.5)*self.sw + jit[0], (0.5-ty)*self.sh + jit[1], 0.0])
        eye = np.array([0.0, 0.0, dist])
        a = np.array([self.bore[0]/self.fpx, -self.bore[1]/self.fpx, -1.0])
        a /= np.linalg.norm(a)
        w = T - eye; w /= np.linalg.norm(w)
        v = np.cross(a, w); s = np.linalg.norm(v); c = float(np.dot(a, w))
        if s < 1e-12: R = np.eye(3)
        else:
            vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
            R = np.eye(3) + vx + vx@vx*((1-c)/(s*s))
        if roll:
            cr, sr = np.cos(roll), np.sin(roll)
            R = R @ np.array([[cr,-sr,0],[sr,cr,0],[0,0,1]])
            # Rolling about the camera's z axis also swings the off-axis boresight,
            # so put it back on target while keeping the roll, as a real user would.
            aw = R @ a
            wv = w
            v2 = np.cross(aw, wv); s2 = np.linalg.norm(v2); c2 = float(np.dot(aw, wv))
            if s2 > 1e-12:
                vx2 = np.array([[0,-v2[2],v2[1]],[v2[2],0,-v2[0]],[-v2[1],v2[0],0]])
                R = (np.eye(3) + vx2 + vx2@vx2*((1-c2)/(s2*s2))) @ R
        C = eye + R@np.array([0,-self.lever_m,0])   # camera below the aim axis
        Pc = (L - C) @ R
        z = -Pc[:,2]
        out = np.stack([FRAME_W/2 + self.fpx*Pc[:,0]/z,
                        FRAME_H/2 - self.fpx*Pc[:,1]/z], 1)
        return out + self.rng.normal(0, self.bs, (4,2))

    def stance_state(self):
        """(distance, roll) the simulated user adopts for the current stance."""
        st = self.s.stance
        if self.s.stance_kind(st) == 'roll':
            # rolled stances happen back at the first distance
            return self.dists[0], np.deg2rad(ROLL_TARGET_DEG * self.s.stance_roll(st))
        return self.dists[min(st, len(self.dists)-1)], 0.0

    def run(self):
        t0 = time.time()
        while not self.stop:
            tx, ty = self.s.target()
            d, rl = self.stance_state()
            q = self._quad(tx, ty, d, rl, stance=self.s.stance, dot=self.s.idx)
            v = np.rint(q.reshape(-1)*10).astype(int)
            # timestamp from real elapsed time, so the gun-clock path sees a clock
            # that behaves like the hardware's
            t = int((time.time() - t0) * 1000.0) + 1000
            self.q.put("Q,%d,4,%d,%d,%d,%d,%d,%d,%d,%d" % (t, *v))
            time.sleep(1.0/60.0)

    def close(self): self.stop = True


# ===========================================================================
# headless self-test: the whole flow, no display
# ===========================================================================
def selftest_auto(stances=2, blob_sigma=0.6, roll_stances=2):
    """Prove the whole run completes with no trigger input of any kind."""
    s = CaptureSession(plan=make_plan(stances, roll_stances))
    s.auto = True                    # exercise the fallback path explicitly
    s.auto_reason = "selftest"
    src = SimSource(s, blob_sigma=blob_sigma)
    now = 0.0; guard = 0; forced = 0
    while s.state != s.S_DONE and guard < 60000:
        guard += 1
        s.auto = True
        d, rl = src.stance_state()
        s.feed(src._quad(*s.target(), d, rl, stance=s.stance, dot=s.idx), now)
        now += 1.0/60.0
    ok = (s.state == s.S_DONE)
    if not ok:
        print("  sigma %.1f: DID NOT COMPLETE -- auto-capture never fired "
              "(%d shots after %.0f simulated seconds)" % (blob_sigma, len(s.shots), now))
        return False
    c, why = s.fit()
    print("  sigma %.1f: %d shots, %d inputs of ANY kind, fit %s"
          % (blob_sigma, len(s.shots), forced,
             "OK bore (%.2f,%.2f) rect %.4f x %.4f" % (c['bx'],c['by'],c['w'],c['h'])
             if c else "FAILED: %s" % why))
    return ok and c is not None


def selftest(stances=2, blob_sigma=0.6, verbose=True, roll_stances=2):
    s = CaptureSession(plan=make_plan(stances, roll_stances))
    src = SimSource(s, blob_sigma=blob_sigma)
    now = 0.0
    guard = 0
    ok_live = [True]
    while s.state != s.S_DONE and guard < 200000:
        guard += 1
        if s.state == s.S_STEPBACK:
            # exercise the real live-feedback path: frames from the new stance
            # arrive BEFORE the trigger
            d, rl = src.stance_state()
            for _ in range(40):
                s.feed(src._quad(*s.target(), d, rl, stance=s.stance, dot=s.idx), now)
                now += 1.0/60.0
            if s.stance_kind() == 'roll':
                dd = s.roll_check()
                if verbose: print("  live tilt feedback: %s"
                                  % ("%+.1f deg" % dd if dd is not None else "n/a"))
                want = s.stance_roll()
                if dd is None or dd*want <= 0 or abs(dd) < ROLL_MIN_DEG:
                    print("  [FAIL] live tilt feedback did not register the roll "
                          "(got %s, wanted %s%.0f deg)"
                          % ("n/a" if dd is None else "%+.1f" % dd,
                             "+" if want > 0 else "-", ROLL_MIN_DEG))
                    ok_live[0] = False
            else:
                rr = s.stepback_check()
                if verbose: print("  live step-back feedback: %s"
                                  % ("%.2fx" % rr if rr else "n/a"))
                if rr is None or rr < 1.3:
                    print("  [FAIL] live step-back feedback did not register the move")
                    ok_live[0] = False
            s.trigger(now); continue
        if s.state in (s.S_AIM, s.S_REVIEW):
            # a real user takes a moment between pulls; advance the clock past the
            # post-state-change dead time
            now += TRIG_DEAD_S + 0.05
            s.trigger(now)          # "user pulls the trigger"
            continue
        tx, ty = s.target()
        d, rl = src.stance_state()
        s.feed(src._quad(tx, ty, d, rl, stance=s.stance, dot=s.idx), now)
        now += 1.0/60.0
    ok = ok_live[0]
    if verbose:
        print("captured %d shots over %d stances (%d rolled)"
              % (len(s.shots), s.stances,
                 sum(1 for p in s.plan if p['kind'] == 'roll')))
        print("  span spread: %.3f   roll spread: %.3f" % (s.spread(), s.roll_spread()))
        sg = np.median([x['sigma'] for x in s.shots])
        print("  blob sigma measured %.3f px (injected %.2f)" % (sg, blob_sigma))
        if abs(sg - blob_sigma) > max(0.15, 0.25*blob_sigma):
            print("  [FAIL] sigma estimate off"); ok = False
    c, why = s.fit()
    if c is None:
        print("  [FAIL] fit refused: %s" % why); return False
    if verbose:
        print("  boresight   (%+.2f, %+.2f)   truth (%+.2f, %+.2f)"
              % (c['bx'], c['by'], *src.truth['bore']))
        print("  LED rect    %.4f x %.4f      truth %.4f x %.4f"
              % (c['w'], c['h'], src.truth['rig_w_frac'], src.truth['rig_h_frac']))
        print("  fit rms     %.5f  (%.0f px on 1920x1080)"
              % (c['fit_rms'], c['fit_rms']*1920))
        print("  rejected    %d of %d" % (c['n_rejected'], len(s.shots)))
    be = np.hypot(c['bx']-src.truth['bore'][0], c['by']-src.truth['bore'][1])
    # The simulated user carries systematic per-stance and per-dot aim bias, which
    # the fit necessarily absorbs partly into the boresight, so allow a few px.
    if be > 10.0:
        print("  [FAIL] boresight off by %.2f px" % be); ok = False
    if abs(c['w']-src.truth['rig_w_frac']) > 0.05:
        print("  [FAIL] rect width off"); ok = False
    return ok


# ===========================================================================
# GUI
# ===========================================================================
def run_gui(src, session, outdir, screenshot=None):
    import tkinter as tk
    root = tk.Tk()
    root.title("Lightgun aim calibration")
    root.attributes("-fullscreen", True)
    root.configure(bg="#000000")
    # Wait for the window to be mapped, then read geometry LIVE on every use: a
    # fresh fullscreen window can report winfo_width()==1.
    for _ in range(60):
        root.update()
        if root.winfo_width() > 64 and root.winfo_height() > 64: break
        time.sleep(0.02)
    SW = root.winfo_screenwidth(); SH = root.winfo_screenheight()

    # On Windows the gun's ABSOLUTE HID coordinate is spread over the whole virtual
    # desktop, not one monitor, so report it.
    VW = VH = None
    try:
        import ctypes
        u = ctypes.windll.user32
        VW, VH = u.GetSystemMetrics(78), u.GetSystemMetrics(79)   # SM_CX/CYVIRTUALSCREEN
    except Exception:
        pass

    def geom():
        """live window geometry -- always correct by the time a shot is recorded"""
        w, h = root.winfo_width(), root.winfo_height()
        if w < 64 or h < 64: w, h = SW, SH
        return root.winfo_rootx(), root.winfo_rooty(), w, h

    def to_screen(fx, fy):
        """Window fraction -> SCREEN fraction: reticles are drawn in window fractions."""
        wx, wy, ww, wh = geom()
        return ((wx + fx*ww) / float(SW), (wy + fy*wh) / float(SH))

    WX, WY, WW, WH = geom()
    covers = (abs(WW-SW) <= 2 and abs(WH-SH) <= 2 and abs(WX) <= 2 and abs(WY) <= 2)
    print("display: screen %dx%d aspect %.3f | window %dx%d at (%d,%d) aspect %.3f"
          % (SW, SH, SW/float(SH), WW, WH, WX, WY, WW/float(max(WH,1))))
    print("         %s" % ("window covers the screen" if covers else
          "WINDOW DOES NOT COVER THE SCREEN -- targets converted to screen fractions"))
    if VW:
        print("         virtual desktop %dx%d%s" % (VW, VH,
              "" if (VW == SW and VH == SH) else
              "   <-- MULTI-MONITOR: the gun's absolute HID coordinate spans this,"
              " not one screen, so aim may over-travel horizontally"))
    session.to_screen = to_screen
    session.geom_note = ("screen %dx%d window %dx%d at (%d,%d) virtual %s"
                         % (SW, SH, WW, WH, WX, WY,
                            ("%dx%d" % (VW, VH)) if VW else "n/a"))

    # the canvas and the input bindings
    cv = tk.Canvas(root, width=SW, height=SH, bg="#000000", highlightthickness=0)
    cv.pack(fill="both", expand=True)

    def on_trigger(_e=None):
        if session.state == session.S_DONE: return
        session.trigger(state["gun_t"], pulled=True)   # gun clock, same as feed()
    root.bind("<Button-1>", on_trigger)
    root.bind("<space>", on_trigger)
    root.bind("<Escape>", lambda e: (src.close(), root.destroy()))

    state = {"fit": None, "why": None, "saved": None, "install": None, "shot": False,
             "t_start": None, "frames": 0, "trigs": 0, "last_line": "",
             "gun_t": 0.0, "arrive_t": 0.0, "dropped": 0}
    global _LAST_STATE
    _LAST_STATE = state          # so hostcheck can drive the results screen

    def on_trigger(_e=None):
        if session.state == session.S_DONE: return
        session.trigger(state["gun_t"], pulled=True)   # gun clock, same as feed()
    root.bind("<Button-1>", on_trigger)
    root.bind("<space>", on_trigger)
    root.bind("<Escape>", lambda e: (src.close(), root.destroy()))

    def draw_hud():
        """Always-on live view, so it is obvious whether the gun is seen at all."""
        BW, BH = SW*0.20, SW*0.20*(FRAME_H/FRAME_W)
        # Right edge, vertically centred: any fixed corner collides with a corner target.
        x0 = SW - BW - 24
        y0 = SH*0.5 - BH*0.5 - 40
        # freshness is the one thing that MUST use the PC clock: it answers
        # "is data still arriving"
        fresh = (time.time() - state["arrive_t"]) < 0.5 if state["arrive_t"] else False
        cv.create_rectangle(x0, y0, x0+BW, y0+BH,
                            outline="#39c26e" if fresh else "#a33",
                            fill="#0a0f14", width=2)
        cv.create_text(x0+BW/2, y0-10, text="camera view  %.0fx%.0f" % (FRAME_W, FRAME_H),
                       fill="#667788", font=("DejaVu Sans", 11))
        q = session.live_q
        if q is not None and fresh:
            # canonicalise for display too, or the panel shows an X whenever the
            # resolver's slot order is not TL/TR/BL/BR
            q = aim_fit.canon(q)
            pts=[]
            for i in range(4):
                px = x0 + (q[i][0]/FRAME_W)*BW
                py = y0 + (q[i][1]/FRAME_H)*BH
                pts.append((px,py))
                cv.create_oval(px-4,py-4,px+4,py+4, fill="#ffd24a", outline="")
                cv.create_text(px+9, py-8, text="TL TR BL BR".split()[i],
                               fill="#8899aa", anchor="w", font=("DejaVu Sans", 9))
            for a,b in ((0,1),(1,3),(3,2),(2,0)):
                cv.create_line(*pts[a],*pts[b], fill="#3a7fbf", width=1)
            # the boresight: where the calibration is evaluated
            cv.create_line(x0+BW/2-6, y0+BH/2, x0+BW/2+6, y0+BH/2, fill="#e0803a")
            cv.create_line(x0+BW/2, y0+BH/2-6, x0+BW/2, y0+BH/2+6, fill="#e0803a")
        else:
            cv.create_text(x0+BW/2, y0+BH/2, text="no 4-LED frames",
                           fill="#a33", font=("DejaVu Sans", 13, "bold"))
        # numbers
        yy = y0+BH+14
        span = aim_fit.quad_span(q) if q is not None else 0.0
        for lbl,val,col in [
            ("frames", "%d" % state["frames"], "#ccc" if state["frames"] else "#a33"),
            ("rate",   "%.0f Hz" % session.fps, "#ccc"),
            ("quad span", "%.1f px" % span, "#ccc"),
            ("triggers", "%d" % state["trigs"],
                         "#39c26e" if state["trigs"] else "#8a7a3a"),
            ("captured", "%d / %d" % (len(session.shots),
                                      len(session.dots)*session.stances), "#ccc"),
            ("pulls this dot", "%d / %d" % (len(session.pulls), PULLS_PER_DOT), "#ccc"),
            ("stale dropped", "%d" % state["dropped"],
                              "#8a7a3a" if state["dropped"] else "#667788"),
        ]:
            cv.create_text(x0, yy, text=lbl, fill="#667788", anchor="w",
                           font=("DejaVu Sans", 11))
            cv.create_text(x0+BW, yy, text=val, fill=col, anchor="e",
                           font=("DejaVu Sans", 11, "bold"))
            yy += 17
        if session.auto:
            cv.create_text(x0+BW/2, yy+8, text="auto-capture armed (no trigger)",
                           fill="#8a7a3a", font=("DejaVu Sans", 10))
        elif state["trigs"] == 0 and state["frames"] > 100:
            cv.create_text(x0+BW/2, yy+8,
                text="waiting for a trigger pull...", fill="#667788",
                font=("DejaVu Sans", 10))
        if state["last_line"]:
            # bottom-LEFT: the bottom-centre is where the instruction text lives
            cv.create_text(24, SH-14, text=state["last_line"][:64],
                           fill="#334455", anchor="w", font=("DejaVu Sans", 10))

    def draw_target(x, y, prog):
        R = max(26, int(min(SW, SH)*0.035))
        cv.create_oval(x-R, y-R, x+R, y+R, outline="#2a5f8f", width=2)
        cv.create_line(x-R*1.7, y, x-R*0.35, y, fill="#4a8fc7", width=2)
        cv.create_line(x+R*0.35, y, x+R*1.7, y, fill="#4a8fc7", width=2)
        cv.create_line(x, y-R*1.7, x, y-R*0.35, fill="#4a8fc7", width=2)
        cv.create_line(x, y+R*0.35, x, y+R*1.7, fill="#4a8fc7", width=2)
        cv.create_oval(x-3, y-3, x+3, y+3, fill="#ffffff", outline="")
        if prog > 0:
            cv.create_arc(x-R, y-R, x+R, y+R, start=90, extent=-359.9*prog,
                          style="arc", outline="#39c26e", width=6)
        elif session.dwell > 0:
            # amber ring = holding still, filling toward an auto-capture
            cv.create_arc(x-R*1.35, y-R*1.35, x+R*1.35, y+R*1.35, start=90,
                          extent=-359.9*session.dwell, style="arc",
                          outline="#e0a03a", width=4)

    def text(x, y, s, size=20, col="#dddddd", anchor="center", bold=False):
        cv.create_text(x, y, text=s, fill=col, anchor=anchor,
                       font=("Segoe UI" if os.name == "nt" else "DejaVu Sans",
                             size, "bold" if bold else "normal"))

    def drain():
        """Read the source, discarding a backlog when not mid-capture (stale frames)."""
        n = 0
        backlog = src.q.qsize()
        if backlog > 120 and session.state != session.S_CAPTURING:
            for _ in range(backlog - 20):
                try: src.q.get_nowait(); state["dropped"] += 1
                except queue.Empty: break
        while n < 600:
            try: line = src.q.get_nowait()
            except queue.Empty: break
            n += 1
            state["last_line"] = line.strip()
            if is_trigger(line):
                state["trigs"] += 1
                session.note_trigger()
                on_trigger()
                continue
            pq = parse_q(line)
            if pq is not None:
                q, gt = pq
                state["frames"] += 1
                state["gun_t"] = gt
                state["arrive_t"] = time.time()
                session.feed(q, gt)

    def tick():
        # No-data guard: the most likely cause is the wrong COM port, since only
        # one of the gun's two USB sockets carries the Q-line stream.
        if state["t_start"] is None:
            state["t_start"] = time.time()
        if state["frames"] == 0 and (time.time() - state["t_start"]) > 4.0:
            cv.delete("all")
            text(SW/2, SH*0.30, "NO DATA FROM THE GUN", 38, "#d24b4b", bold=True)
            msgs = [
              "Nothing has arrived on this port in %.0f seconds." % (time.time()-state["t_start"]),
              "",
              "1. Wrong port? This must be the CH340 / UART socket, NOT the",
              "   native USB socket the OpenFIRE App and HID mouse use.",
              "2. Is dashboard.py still open? Only one program can hold the port.",
              "3. Telemetry: the tool sends '~cam=dash:2' at connect. Run",
              "   tools/aim_probe.py -- it reports whether that is getting through.",
              "4. Are all four LEDs in view? Frames with fewer are discarded.",
              "",
              "Only the CH340 cable is needed now -- the trigger arrives on this",
              "same stream, so the native USB socket can stay unplugged.",
            ]
            yy = SH*0.42
            for m in msgs:
                text(SW/2, yy, m, 17, "#bbbbbb"); yy += SH*0.045
            text(SW/2, SH*0.93, "Esc to quit", 14, "#556677")
            root.after(200, tick); return
        drain()
        cv.delete("all")
        st = session.state
        draw_hud()

        if st == session.S_DONE and state["fit"] is None and state["why"] is None:
            state["fit"], state["why"] = session.fit()
            state["saved"] = session.save(outdir)
            print("\nSaved to: %s" % os.path.dirname(state["saved"][0]))
            for f in state["saved"]: print("   %s" % os.path.basename(f))
            c = state["fit"]
            if c:
                print("boresight %+.2f,%+.2f  LED rect %.4f x %.4f  fit rms %.5f"
                      % (c['bx'], c['by'], c['w'], c['h'], c['fit_rms']))
                cmd = aimcal_line(c)
                print("\nSend this to the gun:\n  %s\n" % cmd)
                with open(os.path.join(os.path.dirname(state["saved"][0]),
                                       "aimcal.txt"), "w") as f:
                    f.write(cmd + "\n")
                # if we own the serial port, install it directly
                if hasattr(src, "ser"):
                    state["install"] = install_over_serial(src, cmd, c)
                    print(state["install"])
                else:
                    state["install"] = ("NOT SENT -- this run had no serial port "
                                        "(simulator or file replay).")
            else:
                print("FIT REFUSED: %s" % state["why"])
            sys.stdout.flush()

        if st in (session.S_AIM, session.S_CAPTURING):
            tx, ty = session.target()
            draw_target(tx*SW, ty*SH, session.progress(state["gun_t"]))
            if st == session.S_CAPTURING:
                head = "HOLD STILL..."
            elif session.auto and not session.armed:
                head = "Move to the %s target" % session.target_name()
            elif session.auto:
                head = "Hold still on the %s target" % session.target_name()
            else:
                head = "Aim at the %s target, then pull the trigger" % session.target_name()
            # Keep the instructions clear of the reticle: use whichever band the
            # target is not in. NB: `band` must be computed BEFORE anything uses it.
            band = 0.78 if ty < 0.55 else 0.10
            # TOO CLOSE, said before the trigger rather than after four pulls.
            marg = session.too_close()
            if marg is not None and marg < EDGE_MARGIN:
                text(SW/2, SH*band, "TOO CLOSE -- STEP BACK", 30, "#e0803a", bold=True)
                text(SW/2, SH*(band+0.05),
                     "an LED is %.0f px from the edge of the camera; captures need %.0f"
                     % (marg, EDGE_MARGIN), 17, "#e0803a")
                text(SW/2, SH*(band+0.09),
                     "blobs that touch the edge report a centroid pulled inward",
                     14, "#8899aa")
            else:
                text(SW/2, SH*band, head, 26, "#ffffff", bold=True)
            if len(session.pulls):
                text(SW/2, SH*(band+0.10), "shot %d of %d on this target"
                     % (len(session.pulls), PULLS_PER_DOT), 18, "#39c26e")
            text(SW/2, SH*(band+0.05), "round %d of %d     target %d of %d"
                 % (session.stance+1, session.stances, session.idx+1, len(session.dots)),
                 16, "#8899aa")
            text(SW/2, SH*0.985,
                 (session.auto_reason if session.auto_reason else
                  "you decide when to shoot -- trigger, space or click. "
                  "hold-still capture arms itself if no trigger appears."),
                 13, "#8a7a3a" if session.auto_reason else "#445566")
            r = session.last_result
            if r:
                text(SW/2, SH*0.94, "last: %d frames   blob sigma %.2f px   quad span %.1f px"
                     "   drift %.1f px   edge %.0f px"
                     % (r['frames'], r['sigma'], r['span'], r['drift'],
                        r.get('edge', float('nan'))), 14, "#667788")
        elif st == session.S_REVIEW:
            left = max(0.0, REVIEW_S - (state["gun_t"] - session.state_t))
            text(SW/2, SH*0.45, "That capture was rejected", 30, "#e0803a", bold=True)
            text(SW/2, SH*0.52, session.msg, 18, "#bbbbbb")
            text(SW/2, SH*0.60, "Retrying this target in %.0f..." % (left + 0.9),
                 20, "#8899aa")
        elif st == session.S_STEPBACK and session.stance_kind() == 'roll':
            want = session.stance_roll()
            side = "CLOCKWISE" if want > 0 else "COUNTER-CLOCKWISE"
            text(SW/2, SH*0.30, "TILT THE GUN " + side, 52, "#ffffff", bold=True)
            text(SW/2, SH*0.40,
                 "Back to your FIRST distance, then roll the gun about %d degrees %s."
                 % (ROLL_TARGET_DEG, side.lower()), 22, "#dddddd")
            text(SW/2, SH*0.45, "Keep aiming normally -- just hold it tilted.",
                 20, "#dddddd")
            text(SW/2, SH*0.53,
                 "Why: your camera is not on your sight line, so rolling the gun swings it",
                 16, "#8899aa")
            text(SW/2, SH*0.57,
                 "around that lever arm -- about 6 screen px per degree for a 10 cm offset.",
                 16, "#8899aa")
            text(SW/2, SH*0.61,
                 "Tilting both ways here is what lets the fit measure and cancel it.",
                 16, "#8899aa")
            d = session.roll_check()
            if d is None:
                text(SW/2, SH*0.72, "waiting for the LEDs...", 22, "#8899aa")
            else:
                signed = d * (1 if want > 0 else -1)
                good = signed >= ROLL_TARGET_DEG * 0.85
                okmin = signed >= ROLL_MIN_DEG
                col = "#39c26e" if good else ("#d8c14a" if okmin else "#d24b4b")
                text(SW/2, SH*0.70, "tilt: %+.1f deg" % d, 34, col, bold=True)
                text(SW/2, SH*0.76,
                     "good -- continuing automatically" if good else
                     ("enough; a little more is better" if okmin else
                      ("tilt the OTHER way" if signed < -2 else "keep tilting")), 18, col)
                bw = SW*0.4
                cv.create_rectangle(SW/2-bw/2, SH*0.80, SW/2+bw/2, SH*0.83,
                                    outline="#334455")
                frac = max(0.0, min(1.0, signed / ROLL_TARGET_DEG))
                cv.create_rectangle(SW/2-bw/2, SH*0.80, SW/2-bw/2 + bw*frac, SH*0.83,
                                    fill=col, outline="")
                # a horizon line that rolls with the gun, so the tilt is a picture
                import math as _m
                a = _m.radians(d); L = SW*0.10
                cx0, cy0 = SW/2, SH*0.90
                cv.create_line(cx0-L, cy0, cx0+L, cy0, fill="#334455", width=2)
                cv.create_line(cx0-L*_m.cos(a), cy0-L*_m.sin(a),
                               cx0+L*_m.cos(a), cy0+L*_m.sin(a), fill=col, width=4)
        elif st == session.S_STEPBACK:
            text(SW/2, SH*0.38, "STEP BACK", 60, "#ffffff", bold=True)
            text(SW/2, SH*0.48,
                 "Move at least 50% further from the screen. It continues by itself.",
                 22, "#dddddd")
            text(SW/2, SH*0.56,
                 "This is required: at a single distance the boresight cannot be",
                 16, "#8899aa")
            text(SW/2, SH*0.60,
                 "separated from the screen mapping, and the fit will be refused.",
                 16, "#8899aa")
            rr = session.stepback_check()
            if rr:
                good = rr >= 1.5
                okmin = rr >= 1.15
                col = "#39c26e" if good else ("#d8c14a" if okmin else "#d24b4b")
                text(SW/2, SH*0.70, "distance change: %.2fx" % rr, 34, col, bold=True)
                text(SW/2, SH*0.76,
                     "good -- continuing automatically" if good else
                     ("just enough; a little further is better" if okmin else
                      "not far enough yet, keep going"), 18, col)
                bw = SW*0.4
                cv.create_rectangle(SW/2-bw/2, SH*0.80, SW/2+bw/2, SH*0.83,
                                    outline="#334455")
                cv.create_rectangle(SW/2-bw/2, SH*0.80,
                                    SW/2-bw/2 + bw*min(1.0, (rr-1.0)/0.6), SH*0.83,
                                    fill=col, outline="")
        else:
            y = SH*0.14
            c, why = state["fit"], state["why"]
            if c is None:
                text(SW/2, y, "CALIBRATION FAILED", 40, "#d24b4b", bold=True); y += SH*0.09
                text(SW/2, y, why or "unknown", 18, "#dddddd"); y += SH*0.08
            else:
                text(SW/2, y, "CALIBRATION COMPLETE", 36, "#39c26e", bold=True); y += SH*0.075
                sg = np.median([x['sigma'] for x in session.shots])
                rows = [
                    ("blob noise floor",   "%.3f px  ->  %.0f screen px of raw jitter"
                                            % (sg, sg*GAIN_HINT)),
                    ("boresight",          "%+.2f, %+.2f native px off frame centre"
                                            % (c['bx'], c['by'])),
                    ("LED rect vs screen", "%.4f wide x %.4f tall" % (c['w'], c['h'])),
                    ("this display",       "%dx%d, aspect %.3f" % (SW, SH, SW/float(SH))),
                    ("implied rig width",  "%.1f cm if it is %.1f cm tall" % (
                        (SW/float(SH)) * c['w']/c['h'] * RIG_H_CM, RIG_H_CM)),
                    ("rig shape (measured)", _rig_line(session)),
                    ("  (go measure it)",  "a mismatch = wrong rig shape or wrong frame"),
                    ("fit residual",       "%.5f  (%.0f px on 1920x1080)"
                                            % (c['fit_rms'], c['fit_rms']*1920)),
                    ("shots used",         "%d, %d rejected as outliers"
                                            % (c['n_shots'], c['n_rejected'])),
                    ("distance spread",    "%.2fx" % c['fit_spread']),
                ]
                for k, v in rows:
                    text(SW*0.30, y, k, 18, "#8899aa", anchor="e")
                    text(SW*0.32, y, v, 17, "#e8e8e8", anchor="w")
                    y += SH*0.046
                # DID IT REACH THE GUN? -- the question the whole session exists
                # to answer, so it goes on screen and not only to the console.
                if c['fit_rms'] > 0.02:
                    text(SW/2, y, "Residual is high -- re-run before trusting this.",
                         17, "#e0803a"); y += SH*0.05
                cmd = aimcal_line(c)
                ins = state.get("install") or ""
                y += SH*0.02
                if ins.startswith("INSTALLED"):
                    text(SW/2, y, "INSTALLED ON THE GUN", 34, "#39c26e", bold=True)
                    y += SH*0.055
                    text(SW/2, y, "read back and verified -- nothing to copy",
                         17, "#39c26e")
                    y += SH*0.045
                    text(SW/2, y, cmd, 15, "#667788")
                    y += SH*0.045
                else:
                    text(SW/2, y, "NOT INSTALLED", 30, "#e0803a", bold=True)
                    y += SH*0.05
                    for ln in (ins or "no serial port").splitlines()[:3]:
                        text(SW/2, y, ln.strip(), 16, "#e0803a"); y += SH*0.035
                    y += SH*0.01
                    text(SW/2, y, "send this line to the gun over serial:", 16, "#8899aa")
                    y += SH*0.045
                    text(SW/2, y, cmd, 20, "#39c26e", bold=True)
                    y += SH*0.05
            if state["saved"]:
                y = max(y + SH*0.015, SH*0.80)      # flows; never clamped back up
                folder = os.path.dirname(state["saved"][0])
                text(SW/2, y, "saved to:  %s" % folder, 14, "#667788"); y += SH*0.028
                text(SW/2, y, "%s   +   %s"
                     % (os.path.basename(state["saved"][0]),
                        os.path.basename(state["saved"][1])), 13, "#556677")
                y += SH*0.030
            # last, so it can never be drawn over
            text(SW/2, min(y, SH*0.975), "Esc to quit", 14, "#556677")


        if screenshot and st == screenshot[0]:
            root.update_idletasks()
            cv.postscript(file=screenshot[1], colormode="color")
            screenshot[0] = None
        root.after(16, tick)

    root.after(16, tick)
    root.mainloop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port"); ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    # distance stances identify the boresight
    ap.add_argument("--stances", type=int, default=3,
                    help="distance stances (default 3)")
    # Rolled stances are OFF by default: the roll term is within noise for most
    # builds. Turn them on only if the camera is genuinely far off the sight line,
    # and check the reported cross-validation before trusting the result.
    ap.add_argument("--roll-stances", type=int, default=0, choices=(0,1,2),
                    help="rolled stances after the distances (default 0; the "
                         "roll term is within noise for most builds)")
    ap.add_argument("--sim-sigma", type=float, default=0.6)
    # Default output next to the SCRIPT, not the shell's working directory.
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "calib_out"))
    a = ap.parse_args()

    if a.selftest:
        print("=== AUTO-CAPTURE ONLY (no trigger input at all) ===")
        allok = True
        for bs in (0.3, 0.6, 1.0):
            allok &= selftest_auto(a.stances, bs, a.roll_stances)
        print()
        for bs in (0.3, 0.6, 1.0):
            print("=== selftest, injected blob sigma %.1f px ===" % bs)
            allok &= selftest(a.stances, bs, roll_stances=a.roll_stances)
            print()
        print("SELFTEST %s" % ("PASS" if allok else "FAIL"))
        sys.exit(0 if allok else 1)

    session = CaptureSession(plan=make_plan(a.stances, a.roll_stances))
    if a.sim:
        src = SimSource(session, blob_sigma=a.sim_sigma)
    else:
        port = a.port
        if not port:
            print("looking for the gun...")
            port = find_gun(a.baud)
            if not port:
                sys.exit("no gun found on any serial port.\n"
                         "Run tools/aim_probe.py for a full diagnosis.")
            print("found it on %s" % port)
        src = SerialSource(port, a.baud)
    src.start()
    try: run_gui(src, session, a.out)
    finally: src.close()


if __name__ == "__main__":
    main()
