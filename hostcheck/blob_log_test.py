#!/usr/bin/env python3
"""The frame-rate meter and the blob CSV, on their own.

Both exist to answer questions nobody has measured on this sensor, so their
arithmetic has to be right before any conclusion is drawn from it: the rate is
the number that decides whether a larger read per frame costs us anything, and
the CSV is the only record of a session captured at the TV.
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))

from gun_studio import BlobLog, FrameRate, parse_blobs

FAILS = []


def ck(ok, msg):
    print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        FAILS.append(msg)


def main():
    # ---- the frame-rate meter -------------------------------------------
    r = FrameRate()
    ck(r.feed(None, None) is None, "no answer before the gun has said anything")
    ck(r.feed(1000, 10000) is None, "one sample is not a rate")
    ck(r.feed(1010, 10100) is None,
       "and a 100 ms window is refused -- too short to be quantisation-proof")
    hz = r.feed(1100, 11000)
    ck(hz is not None and abs(hz - 100.0) < 0.01,
       "100 frames in 1000 ms of the GUN's clock reads as 100 Hz")
    hz = r.feed(1400, 12000)
    ck(abs(hz - 300.0) < 0.01, "and 300 in the next second reads as 300 Hz")
    # A gun that reboots restarts both counters. That must re-baseline, not
    # produce a negative rate or freeze the reading for good.
    r.feed(5, 20)
    ck(abs(r.hz - 300.0) < 0.01, "a reboot keeps the last good reading...")
    hz = r.feed(105, 1020)
    ck(abs(hz - 100.0) < 0.01, "...and then measures again from the new zero")

    # ---- the blob line --------------------------------------------------
    b = parse_blobs("CAM: blobs 30,40,3,1 200,41,4,1 31,140,3,1 210,150,14,0")
    ck(len(b) == 4 and b[3] == (210, 150, 14, 0),
       "the per-blob list parses as position, size and kept-flag")
    ck(parse_blobs("CAM: blobs (sizes need ext:1)") == [],
       "and a reply with no blobs in it yields nothing, not junk")
    ck(parse_blobs("") == [] and parse_blobs(None) == [],
       "an absent line is not an error")

    # ---- the CSV --------------------------------------------------------
    d = tempfile.mkdtemp(prefix="bloblog-")
    path = os.path.join(d, "blobs.csv")
    log = BlobLog(path)
    last = {"bframes": 100, "bms": 5000, "bn": 4, "ext": 1, "bmin": 0,
            "bmax": 15, "rtol": 3, "hwmax": 150, "hwmin": -1, "sens": 1,
            "brej": 7, "brrej": 2, "bdrop": 3,
            "br4": 800, "br3": 150, "br2": 40, "br1": 10, "br0": 0}
    line = "CAM: blobs 30,40,3,1 200,41,4,1 31,140,3,1 210,150,14,0"
    ck(log.sample(last, line, 100.0) is True, "the first sample is written")
    ck(log.sample(last, line, 100.0) is False,
       "polling again with the SAME frame writes nothing -- a file full of "
       "repeated frames would overstate every rate taken from it")
    last["bframes"] = 101
    ck(log.sample(last, line, 100.0) is True, "a new frame is written")
    # A row must survive the stick being pulled, so every row is flushed.
    with open(path) as f:
        rows = f.read().strip().split("\n")
    ck(len(rows) == 3, "header plus two rows are on disk already, unflushed "
                       "rows would be lost when the stick is pulled")
    hdr = rows[0].split(",")
    ck(len(hdr) == len(BlobLog.COLS), "the header names every column")
    for r_ in rows[1:]:
        ck(len(r_.split(",")) == len(hdr),
           "every row has exactly as many fields as the header")
    got = dict(zip(hdr, rows[1].split(",")))
    ck(got["hwmax"] == "150" and got["rtol"] == "3" and got["hz"] == "100.0",
       "the settings in force are recorded WITH the data, so a row can still "
       "be interpreted months later")
    ck(got["s3"] == "14" and got["k3"] == "0",
       "and the rejected blob is in the row, with its size and its verdict")

    # A gun that has said nothing yet must not produce a row.
    log2 = BlobLog(os.path.join(d, "empty.csv"))
    ck(log2.sample({}, "", None) is False,
       "no frame counter means no row, rather than a row of blanks")
    log2.close()
    log.close()

    print("\nblob log: %s (%d failures)"
          % ("ALL PASS" if not FAILS else "FAILED", len(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
