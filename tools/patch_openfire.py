#!/usr/bin/env python3
"""Fetch and/or patch the OpenFIRE firmware this overlay builds on.

    python tools/patch_openfire.py --fetch                      ESP32-S3 (default)
    python tools/patch_openfire.py --board rp2040 --fetch       RP2040 + wiicam
    python tools/patch_openfire.py [--board rp2040] <checkout>  patch a copy you have

Two boards, two upstreams, both pinned to one exact commit each (see
patches/upstream.json):
    esp32   alessandro-satanassi/OpenFIRE-Firmware-ESP32  (OV2640 camera stack)
    rp2040  TeamOpenFIRE/OpenFIRE-Firmware                (wiicam / SEN0158)

OpenFIRE's own sources are not redistributed here. Running this twice is safe,
and it never leaves a half-patched tree.
"""
import hashlib, json, os, re, sys

MARKER = "USE_AIM_PIPELINE"


def read(p):
    with open(p, "r", encoding="utf-8", errors="surrogateescape") as f:
        return f.read().splitlines(keepends=True)


def write(p, lines):
    with open(p, "w", encoding="utf-8", errors="surrogateescape", newline="") as f:
        f.write("".join(lines))


def parse_hunks(patch_lines):
    """unified diff -> [(old_start, [(' '|'-'|'+', text), ...]), ...]"""
    hunks, cur = [], None
    for ln in patch_lines:
        if ln.startswith("@@"):
            m = re.match(r"@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@", ln)
            if not m:
                raise ValueError("bad hunk header: %s" % ln.rstrip())
            cur = (int(m.group(1)), [])
            hunks.append(cur)
        elif cur is not None and ln[:1] in (" ", "-", "+"):
            cur[1].append((ln[0], ln[1:]))
        elif cur is not None and ln.rstrip("\n") == "":
            cur[1].append((" ", "\n"))
    return hunks


def apply_hunks(src, hunks):
    """Apply by exact context match, searching near the recorded position.

    Not a fuzzy patcher on purpose: a silently mis-placed hunk in firmware is
    far worse than a refusal that names the line it wanted.
    """
    out = list(src)
    offset = 0
    for start, ops in hunks:
        old = [t for k, t in ops if k in (" ", "-")]
        new = [t for k, t in ops if k in (" ", "+")]
        want = start - 1 + offset
        found = None
        for probe in [want] + [want + d for d in range(1, 400)] + \
                     [want - d for d in range(1, 400)]:
            if probe < 0 or probe + len(old) > len(out):
                continue
            if out[probe:probe + len(old)] == old:
                found = probe
                break
        if found is None:
            return None, ("could not place a change near line %d.\n"
                          "     The file does not match the OpenFIRE version this "
                          "overlay was built against." % start)
        out[found:found + len(old)] = new
        offset += len(new) - len(old)
    return out, None


def fetch(here, meta, dest):
    """Clone upstream at the pinned commit."""
    import subprocess
    url = meta["repo"]
    ref = meta["pinned_commit"]
    if os.path.exists(dest):
        print("ERROR: %s already exists; delete it or patch it directly." % dest)
        return None
    print("Cloning %s at %s ..." % (url, ref))
    try:
        subprocess.check_call(["git", "clone", "--quiet", url, dest])
        subprocess.check_call(["git", "-C", dest, "checkout", "--quiet", ref])
        # Some upstreams keep their board definitions and bundled libraries as
        # SUBMODULES (TeamOpenFIRE does); a plain clone leaves those folders
        # empty and the build dies on missing headers far from the real cause.
        subprocess.check_call(["git", "-C", dest, "submodule", "update",
                               "--init", "--recursive", "--quiet"])
    except FileNotFoundError:
        print("ERROR: git is not installed, or not on PATH.")
        print("       Download the repository manually, check out %s," % ref)
        print("       then run this script again pointing at the folder.")
        return None
    except subprocess.CalledProcessError as e:
        print("ERROR: git failed (%s)." % e)
        return None
    print("Checked out %s into %s\n" % (ref, dest))
    return dest


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    all_meta_path = os.path.join(here, "patches", "upstream.json")
    all_meta = json.load(open(all_meta_path)) if os.path.exists(all_meta_path) else {}

    args = sys.argv[1:]
    board = "esp32"
    if "--board" in args:
        i = args.index("--board")
        if i + 1 >= len(args) or args[i + 1] not in ("esp32", "rp2040"):
            print("ERROR: --board must be esp32 or rp2040")
            return 2
        board = args[i + 1]
        del args[i:i + 2]
    meta = all_meta.get(board, {})
    pdir = os.path.join(here, "patches", board)

    if not args:
        print(__doc__)
        return 2
    if args[0] == "--fetch":
        if not meta:
            print("ERROR: patches/upstream.json is missing; cannot know what to fetch.")
            return 1
        dest = os.path.abspath(args[1]) if len(args) > 1 else \
               os.path.join(here, meta["checkout_dir"])
        if fetch(here, meta, dest) is None:
            return 1
        root = dest
    else:
        root = os.path.abspath(args[0])

    subdir = meta.get("src_subdir", "lightgun/src")
    src = os.path.join(root, *subdir.split("/"))
    if not os.path.isdir(src):
        # tolerate being pointed at the source folder itself
        probe = meta.get("files", ["main.cpp"])[0]
        if os.path.exists(os.path.join(root, probe)):
            src = root
        else:
            print("ERROR: no %s under %s" % (subdir, root))
            print("       Point this at the folder you unpacked the OpenFIRE "
                  "checkout into (board: %s)." % board)
            return 1

    rc = 0

    # Tell the user, up front, whether their OpenFIRE is the one this was built
    # against. Upstream is active, and a change inside one of the regions we
    # touch is the thing most likely to break a build. The patcher refuses
    # rather than mis-places either way; this just names the situation.
    if meta:
        print("Patch is generated against commit %s (%s)." %
              (meta["pinned_commit"], meta["pinned_commit_date"]))
        for name, want in meta.get("pristine", {}).items():
            t = os.path.join(src, name)
            if not os.path.exists(t):
                continue
            with open(t, "rb") as f:
                body = f.read()
            if MARKER.encode() in body:
                continue                       # already patched; nothing to compare
            got = hashlib.sha256(body).hexdigest()
            if got == want["sha256"]:
                print("  %-20s exact match with the version this was built against" % name)
            else:
                print("  %-20s DIFFERENT from the pinned commit" % name)
                print("     yours %d lines / %d bytes, pinned %d / %d"
                      % (body.count(b"\n"), len(body), want["lines"], want["bytes"]))
                print("     Nothing is written unless every change can be placed.")
                print("     To get the version this was built for:")
                print("       git -C <checkout> checkout %s" % meta.get("pinned_commit", "<commit>"))
        print("")

    # The board definition this project builds against may not be present in
    # every OpenFIRE checkout. Install it if it is missing; never overwrite.
    boards = os.path.join(root, "shared_boards")
    bsrc = os.path.join(here, "boards", "ESP32-S3-WROOM-1-DevKitC-1-N8R2.json")
    if board == "esp32" and os.path.isdir(boards) and os.path.exists(bsrc):
        bdst = os.path.join(boards, os.path.basename(bsrc))
        if os.path.exists(bdst):
            print("  %-20s already present" % os.path.basename(bsrc))
        else:
            with open(bsrc, "rb") as f: data = f.read()
            with open(bdst, "wb") as f: f.write(data)
            print("  %-20s installed into shared_boards/" % os.path.basename(bsrc))

    # TWO PHASES. Plan every file before writing any, because a tree with one
    # file patched and the other not does not compile and is not obviously
    # broken -- it looks like a build error somewhere else entirely.
    plan = []
    for name in meta.get("files", ["main.cpp", "OpenFIREcommon.cpp"]):
        target = os.path.join(src, name)
        patch = os.path.join(pdir, name + ".patch")
        if not os.path.exists(target):
            print("  %-20s not found in %s" % (name, src)); rc = 1; continue
        if not os.path.exists(patch):
            print("  %-20s patch missing from patches/" % name); rc = 1; continue
        body = read(target)
        if any(MARKER in ln for ln in body):
            print("  %-20s already patched, left alone" % name)
            continue
        out, err = apply_hunks(body, parse_hunks(read(patch)))
        if out is None:
            print("  %-20s CANNOT PATCH: %s" % (name, err)); rc = 1; continue
        plan.append((name, target, body, out))

    if rc != 0:
        print("\nNothing was written. Fix the above and run this again.")
        return rc
    for name, target, body, out in plan:
        write(target + ".orig", body)
        write(target, out)
        print("  %-20s patched  (original kept as %s.orig)" % (name, name))
    if board == "rp2040":
        print("\nOpenFIRE is ready. Build with:")
        print("  pio run -c platformio_rp2040.ini -e combined_rp2040_wiicam -t upload")
    else:
        print("\nOpenFIRE is ready. Build with:  pio run -e combined_s3_freenove -t upload")
    return 0


if __name__ == "__main__":
    sys.exit(main())
