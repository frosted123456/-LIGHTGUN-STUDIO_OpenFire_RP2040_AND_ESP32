#!/usr/bin/env python3
"""Assert tools/aim_fit.py and lib/AimPipeline/aim_core.cpp agree.
Two implementations of one algorithm can drift; this is the guard."""
import subprocess, sys, os, re, numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT,'tools'))
import aim_fit

CLI=os.path.join(ROOT,'tools','aim_fit_cli')
if not os.path.exists(CLI): sys.exit("build tools/aim_fit_cli first")
FW,FH=240.0,176.0
rng=np.random.default_rng(99)
bad=0
NTRIAL=80
for trial in range(NTRIAL):
    # Second half of the trials carry ROLL, so the 3-regressor path is exercised
    # on both sides rather than only the level path.
    rolled = trial >= NTRIAL//2
    shots=[]; ntxt=[]
    nd=rng.integers(8,16) if rolled else rng.integers(6,14)
    for i in range(nd):
        # random but plausible quads at two distinct scales
        s=(0.6 if i%2 else 1.0)*rng.uniform(0.9,1.1)
        cx,cy=rng.uniform(80,160),rng.uniform(60,120)
        w,h=40*s,70*s
        q=np.array([[-w/2,-h/2],[w/2,-h/2],[-w/2,h/2],[w/2,h/2]],float)
        if rolled:
            a=np.deg2rad([-14,0,14][i%3])
            q=q@np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]])
        q=q+np.array([cx,cy])
        q+=rng.normal(0,1.2,(4,2))
        tx,ty=rng.uniform(0.05,0.95),rng.uniform(0.05,0.95)
        shots.append(dict(q=q,tx=tx,ty=ty))
        ntxt.append("%.6f %.6f "%(tx,ty)+" ".join("%.4f"%v for v in q.reshape(-1)))
    txt="%g %g\n"%(FW,FH)+"\n".join(ntxt)+"\n"
    r=subprocess.run([CLI],input=txt,capture_output=True,text=True)
    py,reason=aim_fit.fit(shots,FW,FH)
    c_ok = 'FIT REFUSED' not in r.stdout
    if c_ok != (py is not None):
        print("trial %d: AGREEMENT FAILURE on accept/reject (C ok=%s, py ok=%s: %s)"%(
              trial,c_ok,py is not None,reason)); bad+=1; continue
    if not c_ok: continue
    mroll=re.search(r'rx=([+-][0-9.]+) ry=([+-][0-9.]+)',r.stdout)
    if mroll:
        crc=np.array([float(mroll.group(1)),float(mroll.group(2))])
        prc=np.array([py.get('rx',0.0),py.get('ry',0.0)])
        # the roll term is in normalised screen units; 2e-4 is 0.4 px on 1920
        if np.abs(crc-prc).max() > 2e-4:
            print("trial %d: roll coeff disagree C=%s py=%s"%(trial,crc,prc)); bad+=1; continue
        if rolled and not (crc.any() or prc.any()):
            print("trial %d: rolled data but neither side fitted a roll term"%trial); bad+=1; continue
    m=re.search(r'bx=([+-][0-9.]+)\s+by=([+-][0-9.]+)',r.stdout)
    mr=re.search(r'LED rectangle\s+([0-9.]+) x ([0-9.]+)',r.stdout)
    cb=np.array([float(m.group(1)),float(m.group(2))])
    pb=np.array([py['bx'],py['by']])
    cr=np.array([float(mr.group(1)),float(mr.group(2))])
    pr=np.array([py['w'],py['h']])
    db=np.abs(cb-pb).max(); dr=np.abs(cr-pr).max()
    if db>0.02 or dr>0.001:   # 0.02px; physical significance is ~0.2px (blob sigma is 0.5-1px)
        print("trial %d: boresight d=%.4f px, rect d=%.5f  C=%s py=%s"%(trial,db,dr,cb,pb)); bad+=1
print("\n%s  (%d/%d trials disagreed)"%("CROSSCHECK PASS" if not bad else "CROSSCHECK FAIL", bad, NTRIAL))
sys.exit(1 if bad else 0)
