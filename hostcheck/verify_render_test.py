# Drives the verify tool against a fake gun over a pty: the whole shoot-a-grid
# -> report -> CSV path must run with no exception reaching Tk's handler.
import sys,os,time,threading,subprocess,json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import tkinter as _tk
errs=[]; _o=_tk.Tk.report_callback_exception
def catch(self,e,v,tb): errs.append("%s: %s"%(e.__name__,v)); _o(self,e,v,tb)
_tk.Tk.report_callback_exception=catch
sys.argv=['aim_verify.py','--port',sys.argv[1],'--grid','3x3','--out','/tmp/vout']
GRID=[(x,y) for y in (0.12,0.5,0.88) for x in (0.12,0.5,0.88)]
def drive():
    time.sleep(3.0)
    for i,(tx,ty) in enumerate(GRID):
        open('/tmp/vtgt.json','w').write(json.dumps([tx,ty]))
        time.sleep(0.9)                      # let the stream settle on the new aim
        os.system("xdotool key space 2>/dev/null || true")
        time.sleep(1.2)
    time.sleep(2.0)
    subprocess.run("import -window root /tmp/verify.png",shell=True,capture_output=True)
    print("verify render: %s"%("OK" if not errs else "FAILED -- %s"%errs[0]))
    time.sleep(0.3); os._exit(1 if errs else 0)
threading.Thread(target=drive,daemon=True).start()
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools', 'aim_verify.py')).read())
