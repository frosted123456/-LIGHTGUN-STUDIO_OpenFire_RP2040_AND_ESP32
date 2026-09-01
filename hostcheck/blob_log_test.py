#!/usr/bin/env python3
"""The frame-rate meter, the blob CSV and the shape histograms, on their own.

All three exist to answer questions nobody has measured on this sensor, so
their arithmetic has to be right before any conclusion is drawn from it: the
rate is the number that decides whether a larger read per frame costs us
anything, and the two CSVs are the only record of a session captured at the TV.

Full mode makes the blob CSV harder. Each blob now arrives with three extra
numbers -- box width, box height and intensity -- and the older formats do not
send them at all, so every row has to say which of those two it is rather than
letting a reader guess months later.

The shape histograms are harder again: one answer is thirteen lines, and any
of them can be lost. Assembling them is the only part of that capture running
on this side of the wire, so it is tested here on its own, away from any
window that might be drawing it.
"""
import os
import queue
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))

from gun_studio import (BlobLog, FrameRate, HIST_BINS, HIST_FEATS, HistSet,
                        Link, SHAPE_COLS, parse_blobs, write_shape_csv)

FAILS = []

# The header as it stood before full mode. The new columns are APPENDED to it,
# never inserted, so a spreadsheet or a one-off script written against a
# capture from last week still finds every old column at the same index.
OLD_COLS = ("wall", "gun_ms", "bframes", "hz", "bn",
            "ext", "bmin", "bmax", "rtol", "hwmax", "hwmin", "sens",
            "brej", "brrej", "bvalve", "bdrop",
            "br4", "br3", "br2", "br1", "br0",
            "x0", "y0", "s0", "k0", "x1", "y1", "s1", "k1",
            "x2", "y2", "s2", "k2", "x3", "y3", "s3", "k3")

# ...and as it stood before the shape gate, with full mode's box columns on
# the end of it. Pinned as a whole prefix rather than by counting: the shape
# columns were appended to a header that already carried one generation of
# additions, and "appended" is only true if every column of BOTH earlier
# generations is still at the index it was read at.
FULLMODE_COLS = OLD_COLS + ("fmt", "fullreg",
                            "w0", "h0", "i0", "w1", "h1", "i1",
                            "w2", "h2", "i2", "w3", "h3", "i3")


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
    # Full mode appends box width, box height and intensity to the SAME tuple.
    # The first four fields keep their meaning and their order, so the reader
    # of an extended capture is not reading a box width as a kept-flag.
    f = parse_blobs("CAM: blobs 30,40,3,1,8,12,200 200,41,4,1,4,7,255")
    ck(len(f) == 2 and f[0] == (30, 40, 3, 1, 8, 12, 200),
       "a full-mode tuple keeps all seven fields, the extras last")
    ck(len(f) == 2 and f[1][:4] == (200, 41, 4, 1),
       "and its first four still mean what they meant in extended")
    ck(parse_blobs("CAM: blobs (sizes need fmt:1)") == [],
       "and a reply with no blobs in it yields nothing, not junk")
    ck(parse_blobs("") == [] and parse_blobs(None) == [],
       "an absent line is not an error")
    # The reply is built into a fixed buffer, so the LAST tuple is the one that
    # gets cut short. A half tuple is not a blob at any width: taken as one it
    # puts a box width in the kept-flag column and shifts everything after it.
    t = parse_blobs("CAM: blobs 30,40,3,1,8,12,200 200,41,4,1,4")
    ck(t == [(30, 40, 3, 1, 8, 12, 200)],
       "a truncated tuple is dropped, and the whole ones before it are kept")
    ck(parse_blobs("CAM: blobs 1,2,3 9,9,9,9,9,9") == [],
       "only 4 and 7 are tuple widths this format has; nothing else is a blob")

    # ---- the CSV --------------------------------------------------------
    ck(tuple(BlobLog.COLS[:len(OLD_COLS)]) == OLD_COLS,
       "the new columns are APPENDED: every column a pre-full-mode capture "
       "had is still there, at the same index, under the same name")
    ck(len(BlobLog.COLS) == len(set(BlobLog.COLS)),
       "and no column name is used twice -- a duplicate silently wins the "
       "lookup and the other column can never be read back")
    for name in ("fmt", "fullreg"):
        ck(name in BlobLog.COLS,
           "'%s' is recorded with the data: an extended row and a full row "
           "are not otherwise distinguishable afterwards" % name)
    missing = [p + str(i) for i in range(4) for p in ("w", "h", "i")
               if p + str(i) not in BlobLog.COLS]
    ck(not missing, "every blob gets its own box and intensity columns "
                    "(missing: %s)" % missing)
    ck(tuple(BlobLog.COLS[:len(FULLMODE_COLS)]) == FULLMODE_COLS,
       "the shape-gate columns went on the END too, behind the box columns "
       "a full-mode capture from last week is read at")
    # bfar and bnear reached the firmware after this file was last touched and
    # were never written down; bsrej is new with the shape gate. bnear is the
    # one that matters most: it counts blobs a gate dropped that sat exactly
    # where the missing corner had to be, which is the only evidence in the
    # whole capture that a gate is throwing away REAL LEDs. A log without it
    # cannot answer the question it is taken to answer.
    for name in ("bfar", "bnear", "bsrej"):
        ck(name in BlobLog.COLS,
           "'%s' is written down: without it a capture cannot say whether a "
           "rejection was right" % name)
    for name in ("pxmax", "armax"):
        ck(name in BlobLog.COLS,
           "'%s' rides with the data, like every other setting: a pixel "
           "count read months later means nothing unless the row says what "
           "the limit was" % name)

    d = tempfile.mkdtemp(prefix="bloblog-")
    path = os.path.join(d, "blobs.csv")
    log = BlobLog(path)
    last = {"bframes": 100, "bms": 5000, "bn": 4, "ext": 1, "fmt": 1,
            "fullreg": 85, "bmin": 0, "bmax": 15, "rtol": 3, "hwmax": 150,
            "hwmin": -1, "sens": 1, "brej": 7, "brrej": 2, "bdrop": 3,
            "br4": 800, "br3": 150, "br2": 40, "br1": 10, "br0": 0,
            "bfar": 5, "bnear": 2, "bsrej": 11, "pxmax": 14, "armax": 20}
    line = "CAM: blobs 30,40,3,1 200,41,4,1 31,140,3,1 210,150,14,0"
    ck(log.sample(last, line, 100.0) is True, "the first sample is written")
    ck(log.sample(last, line, 100.0) is False,
       "polling again with the SAME frame writes nothing -- a file full of "
       "repeated frames would overstate every rate taken from it")
    last["bframes"] = 101
    ck(log.sample(last, line, 100.0) is True, "a new frame is written")
    # A row must survive the stick being pulled, so every row is flushed.
    with open(path) as fh:
        rows = fh.read().strip().split("\n")
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
    ck(got["fmt"] == "1" and got["fullreg"] == "85",
       "the report format and the mode byte ride along too: which of the "
       "two candidate bytes was in the register is the whole question full "
       "mode is being tried to answer")
    # Extended sends no box at all. Blank, not zero: zero is a real width --
    # it is what a one-pixel blob measures -- so a zero here would read as a
    # measurement that was never taken.
    ck(got["w0"] == "" and got["h0"] == "" and got["i0"] == "",
       "an extended row leaves the box columns EMPTY rather than zero, which "
       "is a width a real blob can have")
    ck(got["bfar"] == "5" and got["bnear"] == "2" and got["bsrej"] == "11",
       "the two rejection verdicts and the shape gate's own count are FILLED, "
       "not just named: bnear is the false-negative meter and a column of "
       "blanks is the same file as no column at all")
    ck(got["pxmax"] == "14" and got["armax"] == "20",
       "and the shape limits those counts were taken under are on the row "
       "beside them")
    # ---- a full-mode row --------------------------------------------------
    last2 = dict(last, bframes=102, fmt=2, bn=3)
    full_line = ("CAM: blobs 30,40,3,1,8,12,200 200,41,4,1,4,7,255 "
                 "31,140,3,1,0,3,1")
    ck(log.sample(last2, full_line, 100.0) is True, "a full-mode frame writes")
    log.close()
    with open(path) as fh:
        rows = fh.read().strip().split("\n")
    ck(len(rows) == 4, "and lands on disk beside the extended ones -- one file "
                       "per session, whatever the format was switched to "
                       "mid-way through it")
    ck(len(rows[3].split(",")) == len(hdr),
       "with the same field count, so the two formats share one table")
    got = dict(zip(hdr, rows[3].split(",")))
    ck(got["fmt"] == "2", "the row says it is a full-mode row")
    ck(got["w0"] == "8" and got["h0"] == "12" and got["i0"] == "200",
       "blob 0 carries the box the sensor measured and its intensity")
    ck(got["x0"] == "30" and got["s0"] == "3" and got["k0"] == "1",
       "and its first four columns did not shift to make room for them")
    ck(got["w2"] == "0" and got["h2"] == "3" and got["i2"] == "1",
       "a zero-width box is recorded as the zero it is, next to a height "
       "that is not zero -- the pair is the shape test, and only one of them "
       "collapsing is the interesting case")
    # Only three blobs were seen, so the fourth is absent in every column it
    # owns -- old and new alike.
    ck(got["x3"] == "" and got["s3"] == "" and got["w3"] == ""
       and got["h3"] == "" and got["i3"] == "",
       "a slot the sensor did not fill is blank across the whole row, not "
       "half blank and half stale")

    # ---- a gun too old to have a shape gate -------------------------------
    # Blank, not zero. "bnear stayed at 0 for the whole session, so the gate
    # is not taking LEDs" is exactly the conclusion this column is read to
    # draw, and it must not be drawable from a gun that never sent the key.
    log3 = BlobLog(os.path.join(d, "old.csv"))
    old = dict(last, bframes=150)
    for k in ("bfar", "bnear", "bsrej", "pxmax", "armax"):
        old.pop(k)
    ck(log3.sample(old, line, 100.0) is True, "a gun on older firmware writes")
    log3.close()
    with open(os.path.join(d, "old.csv")) as fh:
        old_row = dict(zip(hdr, fh.read().strip().split("\n")[1].split(",")))
    ck(all(old_row[k] == "" for k in ("bfar", "bnear", "bsrej", "pxmax",
                                      "armax")),
       "and leaves the new columns EMPTY rather than 0 -- a counter that "
       "never arrived is not a counter that never moved: %s"
       % {k: old_row[k] for k in ("bfar", "bnear", "bsrej")})
    ck(old_row["brej"] == "7" and old_row["s3"] == "14",
       "while everything that gun DOES send still lands in its own column")

    # ---- wire to column, in one piece -------------------------------------
    # The columns above are only half of it. A key the reply parser does not
    # recognise never reaches last[] at all, so a column can be named, filled
    # from last[] and still be blank in every row ever written -- which is
    # exactly how bfar and bnear were lost: they arrived in the firmware, the
    # parser was never told about them, and nothing anywhere said so.
    def _queue_of(lines):
        q = queue.Queue()
        for ln in lines:
            q.put(ln)
        return q

    blob_line = (
        "CAM: blob fmt=2 ext=1 fullreg=85 bmin=2 bmax=9 rtol=3 pxmax=14 "
        "armax=20 hwmax=-1 hwmin=-1 bn=4 brej=90 brrej=39 bvalve=61 "
        "bframes=1900 bms=19000 bdrop=3 bsrej=27 bfar=13 bnear=4 br4=1700 "
        "br3=250 br2=90 br1=40 br0=20\n")
    link = Link()
    link.src = type("S", (), {"q": _queue_of([blob_line])})()
    link.pump()
    for k, want in (("bsrej", 27), ("bfar", 13), ("bnear", 4),
                    ("pxmax", 14), ("armax", 20)):
        ck(link.last.get(k) == want,
           "'%s=%d' off a real camblob? line reaches last[] (%r)"
           % (k, want, link.last.get(k)))
    log4 = BlobLog(os.path.join(d, "wire.csv"))
    ck(log4.sample(link.last, "CAM: blobs 30,40,3,1,8,12,200", None) is True,
       "and a row written straight from that reply lands on disk")
    log4.close()
    with open(os.path.join(d, "wire.csv")) as fh:
        wire_hdr, wire_row = [r.split(",")
                              for r in fh.read().strip().split("\n")[:2]]
    wire = dict(zip(wire_hdr, wire_row))
    ck(wire["bsrej"] == "27" and wire["bfar"] == "13" and wire["bnear"] == "4"
       and wire["pxmax"] == "14" and wire["armax"] == "20",
       "with every new column carrying the gun's own number: %s"
       % {k: wire[k] for k in ("bsrej", "bfar", "bnear", "pxmax", "armax")})

    # A gun that has said nothing yet must not produce a row.
    log2 = BlobLog(os.path.join(d, "empty.csv"))
    ck(log2.sample({}, "", None) is False,
       "no frame counter means no row, rather than a row of blanks")
    log2.close()

    # ---- the shape histograms --------------------------------------------
    def hist_lines(on=1, frames=100, led=400, rej=9, feats=None, bins=None):
        """One '~camlearn?' answer. The firmware sends all twelve lines every
        time, so a feature nothing was fed into arrives as 32 zeros."""
        feats = feats or {}
        out = ["CAM: learn on=%d frames=%d led=%d rej=%d bins=%d"
               % (on, frames, led, rej, HIST_BINS)]
        for c in (0, 1):
            for f in HIST_FEATS:
                b = feats.get((c, f), [0] * (bins or HIST_BINS))
                out.append("CAM: hist c=%d f=%s %s"
                           % (c, f, " ".join(str(v) for v in b)))
        return out

    def feed_all(hs, lines):
        for ln in lines:
            hs.feed(ln)
        return hs

    sz = {(0, "sz"): [0, 441, 58, 1] + [0] * 28}
    hs = feed_all(HistSet(), hist_lines(feats=sz))
    ck(hs.ready() and hs.seq == 1 and hs.counts() == (100, 400, 9),
       "a whole answer publishes once, with the counts it arrived with")
    ck(hs.total(0, "sz") == 500 and hs.total(0, "bw") == 0,
       "a feature nothing was fed into totals zero, which is what a capture "
       "taken outside full mode looks like")
    ck(hs.running() is True,
       "and the capture's on/off state is read off the GUN's answer, not off "
       "whichever button was pressed last")

    # The lines are interleaved with the frame stream and there is nothing
    # that makes them arrive in order.
    L = hist_lines(frames=7, led=8, rej=1, feats=sz)
    hs = feed_all(HistSet(), [L[0]] + L[:0:-1])
    ck(hs.ready() and hs.counts() == (7, 8, 1),
       "the twelve lines land in whatever order they arrive")

    # Cut short. What is already held must survive untouched: a CSV with four
    # rows from this capture and eight from the last one is not a measurement
    # of anything, and nothing about the file afterwards would look wrong.
    hs = feed_all(HistSet(), hist_lines(frames=100, led=400, feats=sz))
    seq, held = hs.seq, hs.counts()
    feed_all(hs, hist_lines(frames=999, led=999, rej=999)[:-3])
    ck(hs.counts() == held and hs.seq == seq,
       "a set three lines short does not publish, and does not disturb the "
       "one already held")
    ck(hs.rows[(0, "sz")][1] == 441,
       "right down to the bins, which are still the first capture's")
    feed_all(hs, hist_lines(frames=999, led=999, rej=999)[-3:])
    ck(hs.counts() == (999, 999, 999) and hs.seq == seq + 1,
       "and the three arriving late finish it -- pump() drains what has come "
       "in and returns, so a reply routinely spans several calls")

    # Rows with no summary in front of them: the tail of a reply we started
    # listening half way through.
    hs = feed_all(HistSet(), hist_lines(frames=1, led=2, feats=sz))
    seq, held = hs.seq, hs.counts()
    feed_all(hs, hist_lines(frames=5, led=6)[1:])        # rows, no summary
    ck(hs.counts() == held and hs.seq == seq,
       "rows with no summary in front of them are dropped rather than "
       "labelled with the previous capture's counts")

    # Two answers running together, the second one's summary lost. The same
    # feature arriving twice is the only sign of that from this side, and
    # letting the second reply's bins fill the first one's gaps would make a
    # set out of two captures with nothing afterwards to show for it.
    L = hist_lines(frames=5, led=6, rej=6)
    feed_all(hs, L[:7])                                  # summary + 6 rows
    feed_all(hs, L[1:])                                  # summary lost: sz again
    ck(hs.counts() == held and hs.seq == seq,
       "a repeated feature throws away the half-built set instead of "
       "stitching two replies into one: %s" % (hs.counts(),))

    # The status replies to '~camlearn=on:1' and '=reset' are not reports.
    hs = HistSet()
    ck(hs.feed("CAM: learn ON -- feeding only resolver-confirmed frames")
       is False and hs.feed("CAM: learn cleared") is False and not hs.ready(),
       "a one-line status is not mistaken for a report with no bins in it")

    # Reconnecting is onto a possibly different gun under possibly different
    # light. seq keeps counting so a caller waiting for a fresh answer is not
    # fooled into writing the previous gun's capture.
    hs = feed_all(HistSet(), hist_lines(feats=sz))
    seq = hs.seq
    hs.reset()
    ck(not hs.ready() and hs.counts() == (0, 0, 0) and hs.seq == seq,
       "a reset forgets the histograms but not the sequence number")

    # ---- the shape CSV ----------------------------------------------------
    ck(SHAPE_COLS[:5] == ("class", "feature", "frames", "led_blobs",
                          "rej_blobs")
       and len(SHAPE_COLS) == 5 + HIST_BINS,
       "the CSV is wide: the counts, then one column per bin (%d columns)"
       % len(SHAPE_COLS))
    full = {}
    for c in (0, 1):
        for i, f in enumerate(HIST_FEATS):
            full[(c, f)] = [(i + 1) * (c + 1) + b for b in range(HIST_BINS)]
    hs = feed_all(HistSet(), hist_lines(on=0, frames=3400, led=13120, rej=268,
                                        feats=full))
    path = os.path.join(d, "shape.csv")
    n = write_shape_csv(path, hs)
    with open(path) as fh:
        body = fh.read().strip().split("\n")
    ck(n == 12 and len(body) == 13, "one row per class and feature, under one "
                                    "header (%d rows)" % n)
    ck(body[0] == ",".join(SHAPE_COLS), "the header names every column")
    rows = [r.split(",") for r in body[1:]]
    ck(all(len(r) == len(SHAPE_COLS) for r in rows),
       "every row is exactly as wide as the header, so the sheet is a table")
    ck([r[0] for r in rows] == ["led"] * 6 + ["rej"] * 6
       and [r[1] for r in rows[:6]] == list(HIST_FEATS),
       "the class is named rather than numbered, and the features keep the "
       "firmware's own order: %s" % [r[0] + "/" + r[1] for r in rows[:2]])
    ck(all(r[2] == "3400" and r[3] == "13120" and r[4] == "268"
           for r in rows),
       "the counts repeat on EVERY row: the first sort in a spreadsheet would "
       "carry a header line of them off into the middle of the data")
    ck(rows[0][5:8] == ["1", "2", "3"] and rows[6][5:8] == ["2", "3", "4"],
       "and each row carries its own bins, class by class")

    # A line the gun's buffer cut short. The tail is unknown, not zero: zeros
    # are bins nothing landed in, and a missing tail written as zeros is a
    # distribution that leans left for no reason anybody could later find.
    cut = dict(full)
    cut[(0, "sz")] = list(range(28))
    hs = feed_all(HistSet(), hist_lines(frames=5, led=6, rej=7, feats=cut))
    write_shape_csv(path, hs)
    with open(path) as fh:
        first = fh.read().strip().split("\n")[1].split(",")
    ck(first[5 + 27] == "27" and first[5 + 28:] == [""] * 4,
       "a truncated histogram line leaves its missing bins EMPTY rather than "
       "zero: %s" % first[-6:])

    print("\nblob log: %s (%d failures)"
          % ("ALL PASS" if not FAILS else "FAILED", len(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
