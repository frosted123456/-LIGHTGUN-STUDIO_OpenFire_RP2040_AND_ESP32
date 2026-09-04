#!/usr/bin/env python3
"""Turn a dash=2 Q-line log into a calibration fit and measure the blob noise floor.

    python3 tools/aim_capture_analyse.py log.txt --dots 5 --stances 2
"""
import sys, argparse, numpy as np

DOTS5 = [(0.08,0.08),(0.92,0.08),(0.08,0.92),(0.92,0.92),(0.50,0.50)]
DOTS9 = [(x,y) for y in (0.08,0.50,0.92) for x in (0.08,0.50,0.92)]

def parse(path):
    ts=[]; qs=[]
    for line in open(path,'r',errors='replace'):
        if not line.startswith('Q,'): continue
        f=line.strip().split(',')
        if len(f)<11: continue
        try:
            t=int(f[1]); n=int(f[2]); v=[int(x) for x in f[3:11]]
        except ValueError: continue
        if n!=4 or min(v)<0: continue        # need all four, no -1 placeholders
        q=[x/10.0 for x in v]                # dash_x10 -> native px
        if len(f)>=15:                       # 14-field form: add the lead back, as parse_q does
            try:
                ldx,ldy=int(f[13])/10.0,int(f[14])/10.0
                q=[c+(ldx if i%2==0 else ldy) for i,c in enumerate(q)]
            except ValueError: pass
        ts.append(t); qs.append(q)
    return np.array(ts), np.array(qs).reshape(-1,4,2)

def similarity_resid(hold):
    """hold: (N,4,2). Per-corner noise after removing the best similarity; sigma in px."""
    ref = np.median(hold,axis=0)                     # (4,2)
    rc  = ref - ref.mean(0)
    res = []
    for f in hold:
        fc = f - f.mean(0)
        # closed-form similarity (scale+rotation) minimising ||s*R*rc - fc||
        num = (rc[:,0]*fc[:,0]+rc[:,1]*fc[:,1]).sum()
        den = (rc[:,0]*fc[:,1]-rc[:,1]*fc[:,0]).sum()
        d   = (rc*rc).sum()
        if d < 1e-9: continue
        a, b = num/d, den/d
        pred = np.stack([a*rc[:,0]-b*rc[:,1], b*rc[:,0]+a*rc[:,1]],1)
        res.append(fc-pred)
    if not res: return float('nan'), float('nan')
    r=np.concatenate(res)
    # dof correction: the similarity fit consumes 4 of a quad's 8 measurements
    DOF = np.sqrt(0.5)
    return float(r.std())/DOF, float(np.abs(r).max())/DOF

def med_filt(a,w):
    n=len(a); o=np.empty_like(a)
    for i in range(n):
        lo=max(0,i-w//2); hi=min(n,i+w//2+1)
        o[i]=np.median(a[lo:hi],axis=0)
    return o

def segment(ts,qs,min_frames,want,verbose=True):
    """Find the `want` longest low-speed dwells in the log."""
    cen=med_filt(qs.mean(1),15)
    v=np.zeros(len(cen))
    v[1:]=np.hypot(cen[1:,0]-cen[:-1,0], cen[1:,1]-cen[:-1,1])
    runs=None
    for thr in np.arange(0.02,3.01,0.02):
        out=[]; i=0
        while i<len(v):
            if v[i]<thr:
                j=i
                while j<len(v) and v[j]<thr: j+=1
                if j-i>=min_frames: out.append((i,j))
                i=j
            else: i+=1
        m=[]
        for r in out:
            if m and r[0]-m[-1][1] <= 10: m[-1]=(m[-1][0],r[1])
            else: m.append((r[0],r[1]))
        if len(m)>=want:
            runs=(thr,m); break
    if runs is None:
        if verbose: print("  WARNING: found fewer dwells than the %d expected."%want)
        return []
    thr,out=runs
    # merge runs split by a brief speed excursion, keep the longest `want` in time
    # order, then trim each end in case a run bled into the sweep either side
    out=sorted(sorted(out,key=lambda r:r[1]-r[0],reverse=True)[:want])
    trimmed=[]
    for i,j in out:
        pad=max(2,(j-i)//8)
        trimmed.append((i+pad, j-pad))
    if verbose:
        print("  smoothed-speed threshold %.2f px/frame -> %d dwells, lengths %s"%(
            thr,len(trimmed),[j-i for i,j in trimmed]))
    return trimmed

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('log'); ap.add_argument('--dots',type=int,default=5)
    ap.add_argument('--stances',type=int,default=2)
    ap.add_argument('--frame',default='240x176')
    ap.add_argument('--min-frames',type=int,default=25)   # ~0.4s at 60Hz
    ap.add_argument('--out',default='shots.txt')
    a=ap.parse_args()
    fw,fh=[float(x) for x in a.frame.split('x')]

    ts,qs=parse(a.log)
    if len(qs)<50: sys.exit("only %d usable Q-lines -- is dash=2 on and all 4 LEDs visible?"%len(qs))
    dur=(ts[-1]-ts[0])/1000.0
    print("%d frames with 4 LEDs over %.1f s (%.0f Hz)"%(len(qs),dur,len(qs)/max(dur,1e-9)))

    want=a.dots*a.stances
    holds=segment(ts,qs,a.min_frames,want)
    print("found %d stationary holds, expected %d (%d dots x %d stances)\n"%(len(holds),want,a.dots,a.stances))
    if len(holds) < want:
        print("  Too few holds. Either pause longer on each dot, or raise --hold-px.")
    if len(holds) > want:
        print("  Extra holds found; keeping the %d longest.\n"%want)
        holds=sorted(sorted(holds,key=lambda h:h[1]-h[0],reverse=True)[:want])

    DOTS = DOTS5 if a.dots==5 else DOTS9
    print("%-6s %-8s %-26s %-10s %-12s %s"%("hold","frames","median quad centre","span px","blob sigma","max|resid|"))
    print("-"*94)
    lines=[]; sigmas=[]
    for k,(i,j) in enumerate(holds[:want]):
        h=qs[i:j]; med=np.median(h,axis=0)
        span=np.hypot(med[:,0].max()-med[:,0].min(), med[:,1].max()-med[:,1].min())
        sg,mx=similarity_resid(h); sigmas.append(sg)
        dot=DOTS[k % a.dots]
        print("%-6d %-8d (%6.2f,%6.2f)             %-10.1f %-12.3f %.2f"%(
            k,j-i,med[:,0].mean(),med[:,1].mean(),span,sg,mx))
        lines.append("%.4f %.4f "%dot + " ".join("%.3f"%x for x in med.reshape(-1)))

    sg=np.array([s for s in sigmas if np.isfinite(s)])
    print("\nBLOB NOISE FLOOR: sigma = %.3f px  (median across holds; hand tremor removed)"%np.median(sg))
    print("  -> at gain 25.6 that is %.0f screen px of raw jitter per axis before filtering"%(np.median(sg)*25.6))

    spans=[np.hypot(np.ptp(np.median(qs[i:j],axis=0)[:,0]),np.ptp(np.median(qs[i:j],axis=0)[:,1]))
           for i,j in holds[:want]]
    if not spans: sys.exit("no usable holds -- see the message above")
    print("  quad span across holds: min %.1f max %.1f  -> spread %.3f %s"%(
        min(spans),max(spans),max(spans)/min(spans),
        "(OK, >=1.15)" if max(spans)/min(spans)>=1.15 else "(TOO LOW -- the step back was not big enough)"))

    # the visiting order is cyclic, so try each rotation and keep the lowest residual
    import subprocess, os, re
    cli=os.path.join(os.path.dirname(os.path.abspath(__file__)),'aim_fit_cli')
    quads=[l.split(' ',2)[2] for l in lines]
    best=None
    if os.path.exists(cli):
        for rot in range(a.dots):
            body=[]
            for k,qd in enumerate(quads):
                d=DOTS[(k+rot) % a.dots]
                body.append("%.4f %.4f %s"%(d[0],d[1],qd))
            txt="%g %g\n"%(fw,fh)+"\n".join(body)+"\n"
            r=subprocess.run([cli],input=txt,capture_output=True,text=True)
            m=re.search(r'fit rms\s+([0-9.]+)',r.stdout)
            rms=float(m.group(1)) if m else 9e9
            if best is None or rms<best[0]: best=(rms,rot,txt,r.stdout)
    if best is None:
        with open(a.out,'w') as f:
            f.write("%g %g\n"%(fw,fh)); f.write("\n".join(lines)+"\n")
        print("\nwrote %s -- now run:  ./tools/aim_fit_cli < %s"%(a.out,a.out)); return
    rms,rot,txt,out = best
    with open(a.out,'w') as f: f.write(txt)
    print("\nbest dot assignment: rotation %d of %d, fit rms %.5f"%(rot,a.dots,rms))
    if rms > 0.02:
        print("  WARNING: residual is high. Either the dwells were mis-segmented, or the")
        print("  aiming reference moved between stances. Re-run the capture before trusting this.")
    print(out)

main()
