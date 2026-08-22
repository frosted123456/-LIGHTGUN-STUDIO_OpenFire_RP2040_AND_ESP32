# Licences

This project's own code is **MIT** — see `LICENSE`. Take it, change it, sell it,
no attribution needed. The pieces it builds on are not all MIT, so the rest of
this file records what belongs to whom.

## Third-party code

This project is an overlay. It does not redistribute OpenFIRE.

**OpenFIRE firmware** — not included. Download it yourself; `tools/patch_openfire.py`
applies this overlay's changes to your copy. The two modified files are shipped
here only as unified diffs under `patches/`. OpenFIRE firmware is LGPL-2.1; the
OpenFIRE desktop app and board definitions are GPL-3.0.

**`lib/esp32-camera-ov2640/`** — a trimmed copy of Espressif's `esp32-camera`
driver, Apache-2.0, plus OV2640 and SCCB sources from the OpenMV project, MIT.
Original copyright headers are preserved in those files.

**Everything else** (`lib/AimPipeline`, `lib/OV2640Capture`,
`lib/DFRobotIRPositionEx_OV2640`, `tools/`, `hostcheck/`) is this project's own
code, MIT licensed.

A note on the MIT choice: OpenFIRE's firmware is LGPL-2.1, and this overlay is
compiled into it. MIT is compatible with that — permissive code can be combined
into an LGPL work — but a binary you build from the combination is covered by
the LGPL, and if you distribute that binary you must honour LGPL-2.1 for the
OpenFIRE parts. Distributing *this* repository on its own is plain MIT, which
is one reason OpenFIRE's sources are not vendored here.

The quad-to-square warp follows Heckbert, *Fundamentals of Texture Mapping and
Image Warping* (MSc thesis, UC Berkeley, 1989), §2.2, implemented from the maths
rather than copied. Cursor smoothing uses the One Euro filter (Casiez, Roussel &
Vogel, CHI 2012). The rectangle-aspect estimator follows Zhang & He, *Whiteboard
Scanning and Image Enhancement*.
