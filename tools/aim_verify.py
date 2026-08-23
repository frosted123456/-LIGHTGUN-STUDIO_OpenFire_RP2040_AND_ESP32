#!/usr/bin/env python3
"""aim_verify.py -- shoot a grid and measure the aiming error.

    python tools/aim_verify.py [--grid 3x3|5x3]

Reports OURS (geometry computed on the PC) next to ACTUAL (where the OS cursor
really went), so downstream errors can be told apart from calibration errors.
"""
import os, sys, time, queue, argparse, threading
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import aim_fit
from aim_calib import parse_q, is_trigger, find_gun, SerialSource, FRAME_W, FRAME_H

HOLD_MS   = 500          # how long each shot samples
MIN_FRAMES = 12
GRID_3x3 = [(x, y) for y in (0.12, 0.5, 0.88) for x in (0.12, 0.5, 0.88)]
GRID_5x3 = [(x, y) for y in (0.12, 0.5, 0.88) for x in (0.08, 0.29, 0.5, 0.71, 0.92)]


def read_calib(ser, tries=4):
    """Read the active calibration off the gun."""
    for _ in range(tries):
        ser.write(b"\n~aimcal?\n")
        buf, t0 = "", time.time()
        while time.time() - t0 < 1.5:
            buf += ser.read(256).decode("ascii", "replace")
            for line in buf.splitlines():
                if "cx=" not in line: continue
                d = {}
                for tok in line.replace("AIM:", "").split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        try: d[k] = float(v)
                        except ValueError: pass
                if all(k in d for k in ("cx", "cy", "w", "h", "bx", "by")):
                    return dict(magic=aim_fit.MAGIC, cx=d["cx"], cy=d["cy"],
                                w=d["w"], h=d["h"], bx=d["bx"], by=d["by"],
                                lever=d.get("lever", 0.0))
            time.sleep(0.02)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--grid", choices=("3x3", "5x3"), default="3x3")
    ap.add_argument("--out", default=os.path.join(HERE, "calib_out"))
    a = ap.parse_args()

    port = a.port or find_gun()
    if not port:
        sys.exit("no gun found -- run tools/aim_probe.py")
    src = SerialSource(port)
    calib = read_calib(src.ser)
    if not calib:
        sys.exit("could not read a calibration off the gun ('~aimcal?').\n"
                 "Run the calibration first -- there is nothing to verify yet.")
    src.start()
    print("gun on %s" % port)
    print("calibration in use: w=%.5f h=%.5f bx=%.2f by=%.2f"
          % (calib["w"], calib["h"], calib["bx"], calib["by"]))

    import tkinter as tk
    root = tk.Tk()
    root.title("Verify")
    root.attributes("-fullscreen", True)
    root.configure(bg="#000000")
    for _ in range(60):
        root.update()
        if root.winfo_width() > 64: break
        time.sleep(0.02)
    SW, SH = root.winfo_screenwidth(), root.winfo_screenheight()
    cv = tk.Canvas(root, width=SW, height=SH, bg="#000000", highlightthickness=0)
    cv.pack(fill="both", expand=True)

    targets = GRID_3x3 if a.grid == "3x3" else GRID_5x3
    st = dict(idx=0, capturing=False, t0=0.0, buf=[], cur=[], results=[],
              gun_t=0.0, arrive=0.0, done=False, last="")

    def txt(x, y, s, size=18, col="#dddddd", bold=False, anchor="center"):
        cv.create_text(x, y, text=s, fill=col, anchor=anchor,
                       font=(("Segoe UI" if os.name == "nt" else "DejaVu Sans"),
                             size, "bold" if bold else "normal"))

    def shoot(_e=None):
        if st["done"] or st["capturing"]: return
        st["capturing"] = True; st["t0"] = st["gun_t"]; st["buf"] = []; st["cur"] = []

    root.bind("<Button-1>", shoot)
    root.bind("<space>", shoot)
    root.bind("<Escape>", lambda e: (src.close(), root.destroy()))

    def finish():
        st["capturing"] = False
        if len(st["buf"]) < MIN_FRAMES:
            st["last"] = "only %d frames -- shoot again" % len(st["buf"]); return
        q = np.median(np.array(st["buf"]), axis=0)
        ours = aim_fit.solve(calib, q, FRAME_W, FRAME_H)
        tx, ty = targets[st["idx"]]
        cur = np.median(np.array(st["cur"]), axis=0) if st["cur"] else None
        st["results"].append(dict(
            target=(tx, ty),
            # roll at the moment of the shot, degrees
            roll_deg=float(np.degrees(np.arcsin(
                np.clip(aim_fit.quad_roll_sin(q), -1.0, 1.0)))),
            ours=(ours[0]*SW, ours[1]*SH) if ours else None,
            actual=(cur[0], cur[1]) if cur is not None else None))
        st["last"] = ""
        st["idx"] += 1
        if st["idx"] >= len(targets):
            st["done"] = True
            report()

    def report():
        rows = []
        for r in st["results"]:
            tx, ty = r["target"][0]*SW, r["target"][1]*SH
            eo = ea = None
            if r["ours"]:   eo = (r["ours"][0]-tx, r["ours"][1]-ty)
            if r["actual"]: ea = (r["actual"][0]-tx, r["actual"][1]-ty)
            rows.append((r["target"], eo, ea, r.get("roll_deg", float('nan'))))
        def stats(sel):
            v = [np.hypot(*e) for _, a, b, _r in rows
                 for e in [ (a if sel == 0 else b) ] if e]
            return (np.mean(v), np.percentile(v, 95), max(v)) if v else (float('nan'),)*3
        so, sa = stats(0), stats(1)
        os.makedirs(a.out, exist_ok=True)
        p = os.path.join(a.out, "verify-%s.csv" % time.strftime("%Y%m%d-%H%M%S"))
        with open(p, "w") as f:
            f.write("target_x,target_y,ours_err_x,ours_err_y,"
                    "actual_err_x,actual_err_y,roll_deg\n")
            for (t, eo, ea, rd) in rows:
                f.write("%.4f,%.4f,%s,%s,%s,%s,%.2f\n" % (
                    t[0], t[1],
                    "%.1f" % eo[0] if eo else "", "%.1f" % eo[1] if eo else "",
                    "%.1f" % ea[0] if ea else "", "%.1f" % ea[1] if ea else "",
                    rd))
        st["summary"] = (so, sa, p)
        print("\nOURS   (geometry only) : %.1f px mean, %.1f p95, %.1f worst" % so)
        print("ACTUAL (through the OS): %.1f px mean, %.1f p95, %.1f worst" % sa)
        # flag residuals that still track roll
        rr = [(rd, eo) for _, eo, _a, rd in rows if eo and rd == rd]
        if len(rr) >= 5:
            xs = np.array([a for a, _ in rr]); ys = np.array([b[0] for _, b in rr])
            if xs.std() > 1.0 and ys.std() > 1e-6:
                cc = float(np.corrcoef(xs, ys)[0, 1])
                print("roll spanned %.1f deg; correlation with x error: %+.2f%s"
                      % (xs.max()-xs.min(), cc,
                         "   <- still roll-dependent, recalibrate with the tilt stances"
                         if abs(cc) > 0.6 else ""))
        print("saved %s" % p)

    def draw():
        cv.delete("all")
        if not st["done"]:
            for i, (tx, ty) in enumerate(targets):
                x, y = tx*SW, ty*SH
                if i < st["idx"]:
                    cv.create_oval(x-5, y-5, x+5, y+5, outline="#39c26e")
                elif i == st["idx"]:
                    R = 30
                    cv.create_oval(x-R, y-R, x+R, y+R, outline="#4a8fc7", width=2)
                    cv.create_line(x-R*1.6, y, x-R*0.4, y, fill="#4a8fc7")
                    cv.create_line(x+R*0.4, y, x+R*1.6, y, fill="#4a8fc7")
                    cv.create_line(x, y-R*1.6, x, y-R*0.4, fill="#4a8fc7")
                    cv.create_line(x, y+R*0.4, x, y+R*1.6, fill="#4a8fc7")
                    cv.create_oval(x-3, y-3, x+3, y+3, fill="#fff", outline="")
                    if st["capturing"]:
                        pr = min(1.0, (st["gun_t"]-st["t0"])*1000.0/HOLD_MS)
                        cv.create_arc(x-R, y-R, x+R, y+R, start=90, extent=-359.9*pr,
                                      style="arc", outline="#39c26e", width=5)
                else:
                    cv.create_oval(x-3, y-3, x+3, y+3, outline="#333")
            band = 0.80 if targets[st["idx"]][1] < 0.55 else 0.12
            txt(SW/2, SH*band, "Aim at the ring and pull the trigger", 24, "#fff", True)
            txt(SW/2, SH*(band+0.045), "target %d of %d%s"
                % (st["idx"]+1, len(targets),
                   "   --   " + st["last"] if st["last"] else ""), 15, "#8899aa")
            return
        # ---- the error map -------------------------------------------------
        so, sa, path = st["summary"]
        txt(SW/2, SH*0.06, "VERIFY RESULTS", 34, "#39c26e", True)
        for r in st["results"]:
            tx, ty = r["target"][0]*SW, r["target"][1]*SH
            cv.create_oval(tx-3, ty-3, tx+3, ty+3, outline="#666")
            for key, col, k in (("ours", "#4a8fc7", 6.0), ("actual", "#e0803a", 6.0)):
                if not r[key]: continue
                ex, ey = r[key][0]-tx, r[key][1]-ty
                cv.create_line(tx, ty, tx+ex*k, ty+ey*k, fill=col, width=2)
                cv.create_oval(tx+ex*k-3, ty+ey*k-3, tx+ex*k+3, ty+ey*k+3,
                               fill=col, outline="")
        y = SH*0.14
        txt(SW/2, y, "error vectors exaggerated 6x", 13, "#667788"); y += SH*0.05
        txt(SW/2, y, "OURS (geometry only)      %.1f px mean   %.1f p95   %.1f worst" % so,
            19, "#4a8fc7", True); y += SH*0.04
        txt(SW/2, y, "ACTUAL (through the OS)   %.1f px mean   %.1f p95   %.1f worst" % sa,
            19, "#e0803a", True); y += SH*0.05
        if sa[0] > so[0]*1.5 + 5:
            txt(SW/2, y, "ACTUAL is much worse than OURS: the error is DOWNSTREAM of the aiming maths.",
                17, "#d8a13a"); y += SH*0.035
            txt(SW/2, y, "Check RunMode=Normal and serialARcorrection=off in the OpenFIRE profile.",
                16, "#d8a13a")
        elif so[0] > 40:
            txt(SW/2, y, "Both are poor in the same way: this is the calibration. Re-run step 3.",
                17, "#d8a13a")
        else:
            txt(SW/2, y, "Geometry and the delivered cursor agree. This is as good as the chain gets.",
                17, "#39c26e")
        txt(SW/2, SH*0.95, "saved %s     --     Esc to close" % os.path.basename(path),
            13, "#556677")

    def tick():
        n = 0
        while n < 400:
            try: line = src.q.get_nowait()
            except queue.Empty: break
            n += 1
            if is_trigger(line):
                shoot(); continue
            pq = parse_q(line)
            if pq is None: continue
            q, gt = pq
            st["gun_t"] = gt
            if st["capturing"]:
                st["buf"].append(q)
                st["cur"].append((root.winfo_pointerx(), root.winfo_pointery()))
                if (gt - st["t0"])*1000.0 >= HOLD_MS: finish()
        draw()
        root.after(16, tick)

    src.ser.write(b"~cam=dash:2\n~aimcap=1\n")
    root.after(16, tick)
    root.mainloop()
    try: src.ser.write(b"~aimcap=0\n~cam=dash:0\n")
    except Exception: pass
    src.close()


if __name__ == "__main__":
    main()
