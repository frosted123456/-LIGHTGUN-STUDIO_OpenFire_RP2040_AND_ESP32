"""fake gun that answers ~aimcal? with a real calibration and streams quads
aimed at whichever screen point the verifier is currently asking for."""
import os,pty,sys,time,threading,select,math
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
FPX=184.7; FW,FH=240.,176.
RIGW,RIGH=0.208,0.402; SWm=0.581; SHm=SWm*1200/1920.
CAL="AIM: ACTIVE  cx=0.51216 cy=0.53514 w=0.40118 h=1.24138 bx=12.156 by=9.125 lever=0.00000  rms=0.011 spread=2.1 shots=10 rej=0"
L=np.array([[-RIGW/2,RIGH/2,0],[RIGW/2,RIGH/2,0],[-RIGW/2,-RIGH/2,0],[RIGW/2,-RIGH/2,0]])
def ray(u,v):
    d=np.array([(u-FW/2)/FPX,-(v-FH/2)/FPX,-1.0]); return d/np.linalg.norm(d)
A=ray(FW/2+12.156,FH/2+9.125)
def quad(tx,ty,d=1.2,err=0.0):
    T=np.array([(tx-0.5+err)*SWm,(0.5-ty)*SHm,0.0]); eye=np.array([0,0,d])
    w=T-eye; w/=np.linalg.norm(w)
    v=np.cross(A,w); s=np.linalg.norm(v); c=float(np.dot(A,w))
    vx=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    R=np.eye(3)+vx+vx@vx*((1-c)/(s*s)) if s>1e-12 else np.eye(3)
    Pc=(L-eye)@R; z=-Pc[:,2]
    return np.stack([FW/2+FPX*Pc[:,0]/z, FH/2-FPX*Pc[:,1]/z],1)+np.random.normal(0,0.2,(4,2))
m,s=pty.openpty(); print("FAKE_GUN_PORT="+os.ttyname(s),flush=True)
tgt=[0.5,0.5]; dash=[False]; stop=[False]
def rx():
    buf=b""
    while not stop[0]:
        r,_,_=select.select([m],[],[],0.05)
        if not r: continue
        try: buf+=os.read(m,256)
        except OSError: break
        while b"\n" in buf:
            ln,buf=buf.split(b"\n",1); t=ln.decode(errors="replace").strip()
            if not t: continue
            if t.startswith("~"): t=t[1:]
            if t.startswith("aimcal?"): os.write(m,(CAL+"\n").encode())
            elif t.startswith("ping"): os.write(m,b"AIM: pong  calib=active filter=1.00/15.00 capture=off\n")
            elif "dash:2" in t: dash[0]=True; os.write(m,b"CMD ok (tune) | dash=2\n")
            elif "dash:0" in t: dash[0]=False
            elif t.startswith("aimcap"): os.write(m,b"AIM: trigger markers ON\n")
def tx():
    n=0
    while not stop[0]:
        if dash[0]:
            n+=1
            q=quad(tgt[0],tgt[1])
            v=np.rint(q.reshape(-1)*10).astype(int)
            os.write(m,("Q,%d,4,%d,%d,%d,%d,%d,%d,%d,%d\n"%(n*17,*v)).encode())
            time.sleep(1/60.)
        else: time.sleep(0.05)
for f in (rx,tx): threading.Thread(target=f,daemon=True).start()
# expose the target so the driver can move it
import json
open('/tmp/vtgt.json','w').write(json.dumps(tgt))
def watch():
    while not stop[0]:
        try:
            t=json.loads(open('/tmp/vtgt.json').read()); tgt[0],tgt[1]=t
        except Exception: pass
        time.sleep(0.05)
threading.Thread(target=watch,daemon=True).start()
time.sleep(120)
