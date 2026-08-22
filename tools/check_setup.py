# PlatformIO pre-build guard. Stops the build with a readable message when the
# OpenFIRE checkout is missing or unpatched, instead of an opaque board or
# compiler error 200 lines later.
import os
import sys

MARKER = "USE_AIM_PIPELINE"
# Two boards, two checkouts. Which one a build needs is decided by the
# src_dir PlatformIO hands us; the CLI probes whichever is present.
BOARDS = {
    "esp32": {
        "checkout": "OpenFIRE-Firmware-ESP32",
        "src": os.path.join("OpenFIRE-Firmware-ESP32", "lightgun", "src"),
        "files": ("main.cpp", "OpenFIREcommon.cpp"),
        "fetch": "python tools/patch_openfire.py --fetch",
        "patch": "python tools/patch_openfire.py OpenFIRE-Firmware-ESP32",
    },
    "rp2040": {
        "checkout": "OpenFIRE-Firmware-RP2040",
        "src": os.path.join("OpenFIRE-Firmware-RP2040", "OpenFIREmain"),
        "files": ("OpenFIREmain.ino", "OpenFIREcommon.cpp"),
        "fetch": "python tools/patch_openfire.py --board rp2040 --fetch",
        "patch": "python tools/patch_openfire.py --board rp2040 OpenFIRE-Firmware-RP2040",
    },
}
SRC = BOARDS["esp32"]["src"]
PARTS = os.path.join("OpenFIRE-Firmware-ESP32", "shared_boards",
                     "partitions", "default_OF_8MB.csv")

# The patched camera driver is linked only because something includes a header
# from it. Ship the .c files without driver/include and PlatformIO silently
# leaves the whole library out, and the build dies at link time on
# cam_patch_chunk_cb with nothing pointing at the cause.
CAM = os.path.join("lib", "esp32-camera-ov2640")
CAM_FILES = [
    os.path.join("driver", "cam_hal.c"),
    os.path.join("driver", "include", "esp_camera.h"),
    os.path.join("driver", "include", "sensor.h"),
    os.path.join("driver", "private_include", "cam_hal.h"),
    os.path.join("driver", "private_include", "sccb.h"),
    os.path.join("driver", "private_include", "xclk.h"),
    os.path.join("sensors", "private_include", "ov2640.h"),
    os.path.join("sensors", "private_include", "ov2640_regs.h"),
    os.path.join("sensors", "private_include", "ov2640_settings.h"),
    os.path.join("target", "esp32s3", "ll_cam.c"),
    os.path.join("target", "private_include", "ll_cam.h"),
]


# How to abort. SCons catches a bare sys.exit, so under PlatformIO the build
# is stopped with env.Exit instead; _abort is bound at import time below.
_abort = None


# Prints the box and aborts; every failure path ends here.
def stop(root, title, lines):
    bar = "=" * 72
    out = ["", bar, "SETUP INCOMPLETE: " + title, bar, ""]
    out += lines
    out += ["",
            "Project folder:",
            "  " + root,
            bar, ""]
    sys.stderr.write("\n".join(out) + "\n")
    sys.stderr.flush()
    _abort()
    raise SystemExit(1)


# Runs the checks in the order a first-time user hits them.
def check(root, board="esp32"):
    b = BOARDS[board]
    src = os.path.join(root, b["src"])
    if not os.path.isdir(src):
        stop(root, "OpenFIRE firmware has not been downloaded yet", [
            "This project builds OpenFIRE's own sources with our libraries on",
            "top. OpenFIRE is not redistributed here, so it has to be fetched",
            "once before the first build.",
            "",
            "Run this from the project folder, then build again:",
            "",
            "    " + b["fetch"],
            "",
            "It clones the exact commit this was built against and applies the",
            "two patches. Running it twice is safe.",
        ])

    missing = [n for n in b["files"]
               if not os.path.exists(os.path.join(src, n))]
    if missing:
        stop(root, "the OpenFIRE checkout is incomplete", [
            "Expected to find these under " + b["src"] + ":",
            "",
        ] + ["    " + n for n in missing] + [
            "",
            "Delete the " + b["checkout"] + " folder and run:",
            "",
            "    " + b["fetch"],
        ])

    unpatched = []
    for n in b["files"]:
        with open(os.path.join(src, n), "r", encoding="utf-8",
                  errors="surrogateescape") as f:
            if MARKER not in f.read():
                unpatched.append(n)
    if unpatched:
        stop(root, "OpenFIRE is downloaded but not patched", [
            "These files do not carry the overlay hooks yet:",
            "",
        ] + ["    " + n for n in unpatched] + [
            "",
            "Run this from the project folder, then build again:",
            "",
            "    " + b["patch"],
            "",
            "If it refuses, your checkout is a different upstream commit than",
            "the patches were generated against. The message names the commit;",
            "check it out and run the patcher again. Nothing is written unless",
            "every change can be placed exactly.",
        ])

    if board == "rp2040":
        # Submodules: a plain clone leaves boards/ and the Adafruit libraries
        # empty, and the build dies on missing headers far from the cause.
        shared = os.path.join(root, b["checkout"], "OpenFIREmain", "boards",
                              "OpenFIREshared.h")
        if not os.path.exists(shared):
            stop(root, "the checkout's submodules are empty", [
                "OpenFIREmain/boards/ (and the bundled Adafruit libraries) are",
                "git SUBMODULES, and this clone never fetched them. Run:",
                "",
                "    git -C " + b["checkout"] + " submodule update --init --recursive",
                "",
                "(New fetches do this automatically now.)",
            ])
        return
    if board != "esp32":
        return

    gone = [f for f in CAM_FILES
            if not os.path.exists(os.path.join(root, CAM, f))]
    if gone:
        stop(root, "this project's own camera driver is incomplete", [
            "Missing from " + CAM + ":",
            "",
        ] + ["    " + f.replace(os.sep, "/") for f in gone] + [
            "",
            "Without the headers PlatformIO does not link that library at all,",
            "and the build fails much later with:",
            "",
            "    undefined reference to `cam_patch_chunk_cb'",
            "",
            "Re-download this project; the folder should hold 17 files.",
        ])

    if not os.path.exists(os.path.join(root, PARTS)):
        stop(root, "the OpenFIRE partition table is missing", [
            "Expected: " + PARTS,
            "",
            "Delete the OpenFIRE-Firmware-ESP32 folder and run:",
            "",
            "    python tools/patch_openfire.py --fetch",
        ])


# PlatformIO imports this with an `env`; the __main__ path is for the tests.
def board_of_src(src_dir):
    return "rp2040" if "OpenFIRE-Firmware-RP2040" in src_dir else "esp32"


if __name__ == "__main__":
    _abort = lambda: sys.exit(1)  # noqa: E731
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    boards = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--board=")]
    root = os.path.abspath(argv[0] if argv else ".")
    if boards:
        todo = boards
    else:
        # probe: validate every checkout that exists; a bare tree gets the
        # esp32 message, which names the default fetch.
        todo = [bd for bd in ("esp32", "rp2040")
                if os.path.isdir(os.path.join(root, BOARDS[bd]["checkout"]))] or ["esp32"]
    for bd in todo:
        check(root, bd)
        print("setup OK (%s)" % bd)
else:
    Import("env")  # noqa: F821
    _abort = lambda: env.Exit(1)  # noqa: E731,F821
    check(env.subst("$PROJECT_DIR"),  # noqa: F821
          board_of_src(env.subst("$PROJECT_SRC_DIR")))  # noqa: F821
