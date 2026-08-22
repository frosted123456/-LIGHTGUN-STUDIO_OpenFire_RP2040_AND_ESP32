# Exercises tools/check_setup.py, the pre-build guard. A build that fails with
# "Unknown board ID" or a missing-header wall teaches the user nothing, so each
# incomplete-setup state must produce its own named message and a non-zero exit.
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
GUARD = os.path.join(ROOT, "tools", "check_setup.py")
SRC = ("OpenFIRE-Firmware-ESP32", "lightgun", "src")
PARTS = ("OpenFIRE-Firmware-ESP32", "shared_boards", "partitions")
CAM = ("lib", "esp32-camera-ov2640")

# Read the required list from the guard itself rather than restating it, so the
# two cannot drift apart.
class _Stub(object):
    def subst(self, _):
        return os.path.join(tempfile.gettempdir(), "ov_guard_absent")

    def Exit(self, _):
        pass


_g = {"__name__": "cs", "Import": lambda *a: None, "env": _Stub()}
_err, sys.stderr = sys.stderr, io.StringIO()
try:
    exec(compile(open(GUARD).read(), GUARD, "exec"), _g)
except SystemExit:
    pass                      # it refuses the stub path; the constant is set
finally:
    sys.stderr = _err
CAM_FILES = _g["CAM_FILES"]

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  ok   %s" % name)
    else:
        print("  FAIL %s %s" % (name, detail))
        fails.append(name)


# Builds a fake project tree; stage says how far setup got.
def tree(stage):
    d = tempfile.mkdtemp(prefix="ov_guard_")
    if stage == "bare":
        return d
    src = os.path.join(d, *SRC)
    os.makedirs(src)
    if stage == "empty_src":
        return d
    body = "int main(){}\n" if stage == "unpatched" else \
           "#ifdef USE_AIM_PIPELINE\n#endif\nint main(){}\n"
    for n in ("main.cpp", "OpenFIREcommon.cpp"):
        open(os.path.join(src, n), "w").write(body)
    if stage == "unpatched" or stage == "half":
        if stage == "half":
            open(os.path.join(src, "main.cpp"), "w").write(
                "#ifdef USE_AIM_PIPELINE\n#endif\n")
            open(os.path.join(src, "OpenFIREcommon.cpp"), "w").write("int x;\n")
        return d
    # The camera driver: "headless" ships only the .c files, which is the state
    # that produced the cam_patch_chunk_cb link failure.
    want = CAM_FILES if stage != "headless" else \
        [f for f in CAM_FILES if f.endswith(".c")]
    for f in want:
        t = os.path.join(d, *CAM)
        t = os.path.join(t, f)
        os.makedirs(os.path.dirname(t), exist_ok=True)
        open(t, "w").write("/* stub */\n")
    if stage == "headless":
        return d
    if stage == "no_parts":
        return d
    p = os.path.join(d, *PARTS)
    os.makedirs(p)
    open(os.path.join(p, "default_OF_8MB.csv"), "w").write("nvs,data,nvs,,16K,\n")
    return d


def run(d):
    r = subprocess.run([sys.executable, GUARD, d], capture_output=True, text=True)
    return r.returncode, (r.stdout + r.stderr)


print("setup guard")

# Each incomplete state must be refused, and the message must name the fix.
for stage, phrase in (("bare", "has not been downloaded"),
                      ("empty_src", "incomplete"),
                      ("unpatched", "not patched"),
                      ("half", "not patched"),
                      ("headless", "camera driver is incomplete"),
                      ("no_parts", "partition table is missing")):
    rc, out = run(tree(stage))
    check("%s refused" % stage, rc != 0, "rc=%d" % rc)
    check("%s names the cause" % stage, phrase in out, repr(out[:200]))
    # Every message must end in a command, but not the same one: an incomplete
    # camera driver is fixed by re-downloading this project, not by the patcher.
    fix = "Re-download" if stage == "headless" else "patch_openfire.py"
    check("%s names its fix" % stage, fix in out)

# The link error the missing headers actually produce must be named, so the
# message matches what the user has on screen.
rc, out = run(tree("headless"))
check("headless names the link symbol", "cam_patch_chunk_cb" in out)
check("headless names a missing header", "esp_camera.h" in out)

# A half-patched tree is the dangerous one: it compiles far enough to confuse.
rc, out = run(tree("half"))
check("half-patched names the unpatched file", "OpenFIREcommon.cpp" in out)

# A complete tree must pass, or the guard blocks working builds.
rc, out = run(tree("ok"))
check("complete setup passes", rc == 0, "rc=%d out=%r" % (rc, out[:200]))

# The path PlatformIO actually uses: module-level exec with an injected Import
# and env. A guard that only works under __main__ would never run in a build.
class FakeEnv(object):
    def __init__(self, d):
        self.d = d
        self.exited = []

    def subst(self, s):
        return self.d if s == "$PROJECT_DIR" else s

    def Exit(self, code):
        self.exited.append(code)


def as_platformio(d):
    env = FakeEnv(d)
    g = {"__name__": "check_setup", "Import": lambda *a: None, "env": env}
    try:
        exec(compile(open(GUARD).read(), GUARD, "exec"), g)
    except SystemExit as e:
        return env, int(getattr(e, "code", 1) or 0)
    return env, 0


env, code = as_platformio(tree("bare"))
check("platformio path calls env.Exit", env.exited == [1], repr(env.exited))
check("platformio path stops the script", code == 1, "code=%r" % code)

env, code = as_platformio(tree("ok"))
check("platformio path allows a good tree", env.exited == [] and code == 0,
      "exited=%r code=%r" % (env.exited, code))

print("FAILURES: %d" % len(fails))
print("ALL PASS" if not fails else "FAILED")
sys.exit(1 if fails else 0)
