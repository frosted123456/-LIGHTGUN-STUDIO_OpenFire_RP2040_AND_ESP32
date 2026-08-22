#!/usr/bin/env python3
"""A fake gun on a pty: implements the '~' channel exactly as the firmware does,
by driving the REAL aim_runtime through a tiny C shim. Lets aim_probe.py be
tested end to end with no hardware."""
import os, pty, sys, time, threading, subprocess, select
# build a shim that feeds bytes to aim_serial_rx and prints whatever it emits
open('/tmp/shim.cpp','w').write(r'''
#include "aim_runtime.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
static int g_thr=80, g_saved=-1;
static bool cam(const char* l){
    if(!strncmp(l,"cam?",4)){ printf("CAM: thr=%d aec=40 agc=2 boost=0\n",g_thr); fflush(stdout); return true; }
    if(!strncmp(l,"camsave",7)){ g_saved=g_thr; printf("CAM: saved thr=%d aec=40 agc=2 boost=0\n",g_thr); fflush(stdout); return true; }
    if(!strncmp(l,"camreset",8)){ g_saved=-1; printf("CAM: stored settings cleared\n"); fflush(stdout); return true; }
    if(strncmp(l,"cam=",4)) return false;
    { const char* t=strstr(l,"thr:"); if(t) g_thr=atoi(t+4); }
    printf("CMD ok (tune) | %s\n", l+4);
    if(strstr(l,"dash:2")) puts("__DASH_ON__");
    if(strstr(l,"dash:0")) puts("__DASH_OFF__");
    fflush(stdout); return true;
}
int main(){
    setvbuf(stdout,NULL,_IOLBF,0);
    aim_runtime_begin();
    aim_serial_set_extra(cam);
    int c;
    while((c=getchar())!=EOF){ aim_serial_rx((char)c); fflush(stdout); }
    return 0;
}''')
ROOT=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..")
subprocess.run(["g++","-std=c++17","-O1","-Ilib/AimPipeline","/tmp/shim.cpp",
                "lib/AimPipeline/aim_runtime.cpp","lib/AimPipeline/aim_core.cpp",
                "-o","/tmp/shim","-lm"],check=True,cwd=ROOT)
m,s=pty.openpty()
name=os.ttyname(s)
print("FAKE_GUN_PORT="+name, flush=True)
proc=subprocess.Popen(["/tmp/shim"],stdin=subprocess.PIPE,stdout=subprocess.PIPE,bufsize=0)
dash=[False]; stop=[False]
def pump_out():
    while not stop[0]:
        r,_,_=select.select([proc.stdout],[],[],0.05)
        if r:
            line=proc.stdout.readline()
            if not line: break
            t=line.decode(errors='replace')
            if '__DASH_ON__' in t: dash[0]=True; continue
            if '__DASH_OFF__' in t: dash[0]=False; continue
            os.write(m,t.encode())
def pump_in():
    while not stop[0]:
        r,_,_=select.select([m],[],[],0.05)
        if r:
            try: d=os.read(m,256)
            except OSError: break
            if d: proc.stdin.write(d); proc.stdin.flush()
def stream():
    n=0
    while not stop[0]:
        if dash[0]:
            n+=1
            os.write(m,("Q,%d,4,1476,664,1807,654,1473,1290,1818,1308\n"%(n*17)).encode())
            if n%30==0: os.write(m,b"STAT,1000,135.0,0.2,0,0,thr=60 aec=40 agc=2 boost=0 y8=0\n")
            if n%45==0: os.write(m,b"T,%d\n"%(n*17))   # a simulated trigger pull
            time.sleep(1/60.0)
        else: time.sleep(0.05)
# mimic the firmware's boot announcement
def announce():
    time.sleep(0.5)
    os.write(m,b"AIM: pipeline ready (v30) -- send ~ping or ~aimcal?\n")
for f in (pump_out,pump_in,stream,announce): threading.Thread(target=f,daemon=True).start()
time.sleep(60); stop[0]=True
