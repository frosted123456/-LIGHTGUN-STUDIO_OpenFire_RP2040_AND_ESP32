#!/usr/bin/env python3
"""A rejected capture must be retried by a NEW trigger pull: a stale marker
arriving just after the state flips back to AIM must not start a capture, and the
dead time must stay short enough not to eat a deliberate pull.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import aim_calib as A

fails = []
sess = A.CaptureSession(plan=A.make_plan(3, 0))
src = A.SimSource(sess, blob_sigma=0.3)
q = src._quad(0.5, 0.5, 1.6)

now = 100.0
sess.live_t = now
sess._enter(sess.S_REVIEW, now)
sess.msg = "test rejection"

# a pull made DURING the review screen
sess.trigger(now + 0.1)
if sess.state != sess.S_AIM:
    fails.append("a pull during REVIEW should advance/retry, got state %s" % sess.state)

# ...and the marker for that same pull arriving a moment later must NOT capture
sess.trigger(now + 0.2)
if sess.state == sess.S_CAPTURING:
    fails.append("a stale marker started a capture immediately after the retry began")

# a genuine pull, after the dead time, must work
sess.trigger(now + 0.2 + A.TRIG_DEAD_S + 0.05)
if sess.state != sess.S_CAPTURING:
    fails.append("a real pull after the dead time did not start a capture")
print("dead time %.2f s: stale marker ignored, real pull accepted" % A.TRIG_DEAD_S)

# the dead time must not be so long it eats normal shooting: four pulls on one
# dot happen back to back with no state change between them
sess2 = A.CaptureSession(plan=A.make_plan(3, 0))
sess2.live_t = 0.0
sess2._enter(sess2.S_AIM, 0.0)
t = A.TRIG_DEAD_S + 0.01
sess2.trigger(t)
if sess2.state != sess2.S_CAPTURING:
    fails.append("first pull on a fresh target was swallowed")
if A.TRIG_DEAD_S > 1.0:
    fails.append("dead time %.2f s is long enough to feel like a dropped trigger"
                 % A.TRIG_DEAD_S)

for f in fails:
    print("  [FAIL] %s" % f)
# ---- recoil blanking -------------------------------------------------------
# A REAL pull fires the solenoid, and the strike swings the gun through the
# first part of the capture. Those frames are not aim and must be skipped --
# but only on a real pull: the auto-dwell path and the simulators pull
# nothing and must behave exactly as before.
sess2 = A.CaptureSession(plan=A.make_plan(3, 0))
src2 = A.SimSource(sess2, blob_sigma=0.3)
q2 = src2._quad(0.5, 0.5, 1.6)
t = 200.0
sess2.live_t = t
sess2._enter(sess2.S_AIM, t)
sess2.trig_deaf_until = 0.0
sess2.trigger(t, pulled=True)
if sess2.state != sess2.S_CAPTURING:
    fails.append("a real pull should start the capture")
# recoil frames: violently displaced, inside the blank window
qbad = q2 + 60.0
for i in range(10):
    sess2.feed(qbad, t + 0.01 + i * 0.02)
if len(sess2.buf) != 0:
    fails.append("recoil-window frames were counted (%d)" % len(sess2.buf))
# settled frames after the blank are counted
tb = t + A.RECOIL_BLANK_MS / 1000.0 + 0.01
for i in range(50):
    sess2.feed(q2, tb + i * 0.02)
if len(sess2.buf) < 40:
    fails.append("post-blank frames were not counted (%d)" % len(sess2.buf))
# and the capture window is extended by the blank, not shortened by it
if sess2.state != sess2.S_CAPTURING:
    fails.append("the capture ended before its full post-blank window")
# an auto (dwell) capture skips nothing
sess3 = A.CaptureSession(plan=A.make_plan(3, 0))
sess3.live_t = t
sess3._enter(sess3.S_AIM, t)
sess3.trig_deaf_until = 0.0
sess3.trigger(t)                      # pulled defaults to False
sess3.feed(q2, t + 0.02)
if len(sess3.buf) != 1:
    fails.append("an auto capture must not blank (%d)" % len(sess3.buf))

print("retry trigger: %s" % ("ALL PASS" if not fails else "FAILED"))
sys.exit(1 if fails else 0)
