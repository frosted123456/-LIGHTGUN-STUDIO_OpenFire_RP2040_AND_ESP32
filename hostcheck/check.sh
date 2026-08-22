#!/bin/sh
# Host-side check suite for the OV2640 lightgun overlay. Run after any edit:
#   sh hostcheck/check.sh
cd "$(dirname "$0")/.." || exit 1
# Syntax-check ov2640_capture.cpp in all four LIGHTGUN_DIAG x USE_AIM_PIPELINE
# combinations: a guard can be wrong in one and compile clean in the others.
for D in 0 1; do
  for A in "" "-DUSE_AIM_PIPELINE"; do
    if g++ -std=c++17 -fsyntax-only -DESP_PLATFORM -DLIGHTGUN_DIAG=$D $A \
        -Ihostcheck/fakeinc -Ilib/QuadResolver -Ilib/OV2640Capture -Ilib/DFRobotIRPositionEx_OV2640 \
        -Ilib/AimPipeline lib/OV2640Capture/ov2640_capture.cpp; then
        echo "LIGHTGUN_DIAG=$D ${A:-(no pipeline)}: OK"
    else
        echo "LIGHTGUN_DIAG=$D ${A:-(no pipeline)}: FAILED"; exit 1
    fi
  done
done

# The signed wire format: out-of-frame corners must survive with their sign.
if g++ -std=c++17 -O1 -fsanitize=undefined -Ilib/DFRobotIRPositionEx_OV2640 \
       hostcheck/shim_signed_test.cpp -o /tmp/ov_shim_signed_test \
   && /tmp/ov_shim_signed_test > /tmp/ov_shim_signed_out 2>&1; then
    echo "shim signed round trip: OK"
else
    cat /tmp/ov_shim_signed_out 2>/dev/null
    echo "shim signed round trip: FAILED"; exit 1
fi

# The native-px members must be reachable from OUTSIDE the class, as GetPosition
# reads them.
if g++ -std=c++17 -O1 -Ilib/DFRobotIRPositionEx_OV2640 -Ilib/AimPipeline \
       hostcheck/native_access_test.cpp lib/AimPipeline/aim_core.cpp \
       -o /tmp/ov_native_access -lm \
   && /tmp/ov_native_access; then
    echo "native member access: OK"
else
    echo "native member access: FAILED"; exit 1
fi

# The aim pipeline's own C++ suites.
for T in aim_core_test aim_runtime_test roll_test lever_test; do
    SRC="lib/AimPipeline/aim_core.cpp"
    [ "$T" = "aim_runtime_test" ] && SRC="$SRC lib/AimPipeline/aim_runtime.cpp"
    if g++ -std=c++17 -O2 -Ilib/AimPipeline hostcheck/$T.cpp $SRC \
           -o /tmp/ov_$T -lm && /tmp/ov_$T > /tmp/ov_$T.out 2>&1 \
       && grep -q "ALL PASS" /tmp/ov_$T.out; then
        echo "$T: OK"
    else
        tail -20 /tmp/ov_$T.out 2>/dev/null
        echo "$T: FAILED"; exit 1
    fi
done

# Compile AND LINK the ESP_PLATFORM branch of aim_runtime.cpp against the fakeinc
# stubs -- a syntax-only check cannot see a linkage mistake.
if g++ -std=c++17 -O2 -DESP_PLATFORM -Ihostcheck/fakeinc -Ilib/AimPipeline \
       hostcheck/esp_link_test.cpp lib/AimPipeline/aim_runtime.cpp \
       lib/AimPipeline/aim_core.cpp -o /tmp/ov_esplink -lm \
   && /tmp/ov_esplink > /tmp/ov_esplink.out 2>&1 \
   && grep -q "ALL PASS" /tmp/ov_esplink.out; then
    echo "esp branch link + nvs round trip: OK"
else
    tail -20 /tmp/ov_esplink.out 2>/dev/null
    echo "esp branch link + nvs round trip: FAILED"; exit 1
fi

# Warning sweep on the aim pipeline, so ESP-toolchain warnings surface here first.
if g++ -std=c++17 -O2 -Wall -Wextra -Wvolatile -Werror -Ilib/AimPipeline \
       -c lib/AimPipeline/aim_core.cpp -o /dev/null \
   && g++ -std=c++17 -O2 -Wall -Wextra -Wvolatile -Werror -Ilib/AimPipeline \
       -c lib/AimPipeline/aim_runtime.cpp -o /dev/null; then
    echo "aim pipeline -Wall -Wextra -Werror: OK"
else
    echo "aim pipeline -Wall -Wextra -Werror: FAILED"; exit 1
fi

# The resolver's corner order is stable but arbitrary: all 24 permutations of the
# same quad must give the identical result.
if g++ -std=c++17 -O2 -Ilib/AimPipeline hostcheck/canon_test.cpp \
       lib/AimPipeline/aim_runtime.cpp lib/AimPipeline/aim_core.cpp \
       -o /tmp/ov_canon -lm && /tmp/ov_canon > /tmp/ov_canon.out 2>&1 \
   && grep -q "identical result" /tmp/ov_canon.out; then
    echo "corner-order invariance (24 permutations): OK"
else
    cat /tmp/ov_canon.out 2>/dev/null
    echo "corner-order invariance: FAILED"; exit 1
fi

# aim_fit.py and aim_core.cpp are two implementations of one algorithm (the GUI
# fits with one, the gun runs the other) and must agree, roll term included.
if g++ -std=c++17 -O2 -Ilib/AimPipeline tools/aim_fit_cli.cpp \
       lib/AimPipeline/aim_core.cpp -o tools/aim_fit_cli -lm \
   && python3 hostcheck/py_c_crosscheck.py > /tmp/ov_xc.out 2>&1 \
   && grep -q "CROSSCHECK PASS" /tmp/ov_xc.out; then
    echo "python/C fitter agreement: OK"
else
    tail -12 /tmp/ov_xc.out 2>/dev/null
    echo "python/C fitter agreement: FAILED"; exit 1
fi

# rect_aspect, self-checked before anything draws a conclusion from it.
if python3 hostcheck/rect_aspect_test.py > /tmp/ov_ra.out 2>&1 \
   && grep -q "rect_aspect: ALL PASS" /tmp/ov_ra.out; then
    echo "rig-aspect estimator self-check: OK"
else
    cat /tmp/ov_ra.out 2>/dev/null
    echo "rig-aspect estimator self-check: FAILED"; exit 1
fi

# Nothing in the aim path may drift over a long motionless run, including across
# the 32-bit microsecond wrap.
if g++ -std=c++17 -O2 -Ilib/QuadResolver -Ilib/OV2640Capture -Ilib/AimPipeline \
       hostcheck/long_run_drift_test.cpp lib/QuadResolver/quad_resolver.cpp \
       lib/AimPipeline/aim_core.cpp lib/AimPipeline/aim_runtime.cpp \
       -o /tmp/ov_drift -lm \
   && /tmp/ov_drift > /tmp/ov_drift.out 2>&1 \
   && grep -q "ALL PASS" /tmp/ov_drift.out; then
    echo "no drift over a two-hour run: OK"
else
    cat /tmp/ov_drift.out 2>/dev/null
    echo "no drift over a two-hour run: FAILED"; exit 1
fi

# The latency lead must actually move the quad, measured through the real
# resolver with the same arithmetic the capture layer uses.
if g++ -std=c++17 -O2 -Ilib/QuadResolver -Ilib/OV2640Capture hostcheck/lead_test.cpp \
       lib/QuadResolver/quad_resolver.cpp -o /tmp/ov_lead -lm \
   && /tmp/ov_lead > /tmp/ov_lead.out 2>&1 \
   && grep -q "ALL PASS" /tmp/ov_lead.out; then
    echo "latency lead measured end to end: OK"
else
    cat /tmp/ov_lead.out 2>/dev/null
    echo "latency lead measured end to end: FAILED"; exit 1
fi

# The fine-tune split: an angular vs parallax sight offset can only be separated
# by two measurements at different distances.
if python3 hostcheck/finetune_split_test.py > /tmp/ov_ft.out 2>&1 \
   && grep -q "finetune split: ALL PASS" /tmp/ov_ft.out; then
    echo "fine-tune angular/parallax split: OK"
else
    cat /tmp/ov_ft.out 2>/dev/null
    echo "fine-tune angular/parallax split: FAILED"; exit 1
fi

# A rejected capture must wait for a NEW trigger pull, not a stale streamed marker.
if python3 hostcheck/retry_trigger_test.py > /tmp/ov_retry.out 2>&1 \
   && grep -q "retry trigger: ALL PASS" /tmp/ov_retry.out; then
    echo "retry waits for a fresh trigger: OK"
else
    cat /tmp/ov_retry.out 2>/dev/null
    echo "retry waits for a fresh trigger: FAILED"; exit 1
fi

# The RP2040/wiicam front end: the stale-slot mask, 240x176 normalisation,
# lock + solve, lead, and the ~cam subset -- all on the host.
if g++ -std=c++17 -O2 -DESP_PLATFORM -Ihostcheck/fakeinc -Ilib/WiicamAim \
       -Ilib/QuadResolver -Ilib/AimPipeline hostcheck/wiicam_adapter_test.cpp \
       lib/WiicamAim/wiicam_aim.cpp lib/QuadResolver/quad_resolver.cpp \
       lib/AimPipeline/aim_runtime.cpp lib/AimPipeline/aim_core.cpp \
       -o /tmp/ov_wiicam -lm \
   && /tmp/ov_wiicam > /tmp/ov_wiicam.out 2>&1 \
   && grep -q "wiicam adapter: ALL PASS" /tmp/ov_wiicam.out; then
    echo "wiicam adapter (rp2040 front end): OK"
else
    tail -20 /tmp/ov_wiicam.out 2>/dev/null
    echo "wiicam adapter (rp2040 front end): FAILED"; exit 1
fi

# Lens fit machinery: recover known synthetic lenses, refuse bad sweeps.
if python3 tools/calib_lens.py selftest > /tmp/ov_lens.out 2>&1 \
   && grep -q "SELFTEST PASSED" /tmp/ov_lens.out; then
    echo "lens fit selftest: OK"
else
    cat /tmp/ov_lens.out 2>/dev/null
    echo "lens fit selftest: FAILED"; exit 1
fi

# Step 1 gives the port to the OpenFIRE app; coming back from that is ours to
# do, and it must always finish so the UI cannot park on 'handed off'.
if python3 hostcheck/port_handoff_test.py > /tmp/ov_hand.out 2>&1 \
   && grep -q "port handoff: ALL PASS" /tmp/ov_hand.out; then
    echo "port handback after the OpenFIRE app: OK"
else
    cat /tmp/ov_hand.out 2>/dev/null
    echo "port handback after the OpenFIRE app: FAILED"; exit 1
fi

# The frame-edge guard, against two real bench sessions kept in hostcheck/data.
if python3 hostcheck/edge_guard_test.py > /tmp/ov_edge.out 2>&1 \
   && grep -q "edge guard: ALL PASS" /tmp/ov_edge.out; then
    echo "frame-edge guard vs real sessions: OK"
else
    cat /tmp/ov_edge.out 2>/dev/null
    echo "frame-edge guard vs real sessions: FAILED"; exit 1
fi

# The pre-build setup guard: every incomplete-setup state must be refused with
# a message that names the cause, not an unrelated board or compiler error.
if python3 hostcheck/setup_guard_test.py > /tmp/ov_guard.out 2>&1 \
   && grep -q "ALL PASS" /tmp/ov_guard.out; then
    echo "pre-build setup guard: OK"
else
    cat /tmp/ov_guard.out 2>/dev/null
    echo "pre-build setup guard: FAILED"; exit 1
fi

# The calibration install path: the '~' prefix the firmware requires, and the
# read-back that proves the values actually landed.
if python3 hostcheck/install_verify_test.py > /tmp/ov_inst.out 2>&1 \
   && grep -q "install verification: ALL PASS" /tmp/ov_inst.out; then
    echo "calibration install + read-back: OK"
else
    cat /tmp/ov_inst.out 2>/dev/null
    echo "calibration install + read-back: FAILED"; exit 1
fi

# The roll term end to end: paired on identical data, and the fitted coefficient
# must recover the simulator's known camera lever.
if python3 hostcheck/roll_end_to_end.py > /tmp/ov_roll.out 2>&1 \
   && grep -q "roll end-to-end: ALL PASS" /tmp/ov_roll.out; then
    echo "roll term end-to-end: OK"
else
    cat /tmp/ov_roll.out 2>/dev/null
    echo "roll term end-to-end: FAILED"; exit 1
fi

# The reticles are drawn in WINDOW fractions but the gun maps to the whole SCREEN;
# the fit must still recover the true rectangle when the two differ.
if python3 hostcheck/window_geometry_test.py > /tmp/ov_geom.out 2>&1 \
   && grep -q "window/screen geometry: OK" /tmp/ov_geom.out; then
    echo "window/screen reference frame: OK"
else
    cat /tmp/ov_geom.out 2>/dev/null
    echo "window/screen reference frame: FAILED"; exit 1
fi

# The '~' channel shares OpenFIRE's Serial: verify byte-for-byte pass-through of
# their S/M/X/F/E commands.
if g++ -std=c++17 -O2 -Ilib/AimPipeline hostcheck/serial_channel_test.cpp \
       lib/AimPipeline/aim_runtime.cpp lib/AimPipeline/aim_core.cpp \
       -o /tmp/ov_sct -lm && /tmp/ov_sct > /tmp/ov_sct.out 2>&1 \
   && grep -q "ALL PASS" /tmp/ov_sct.out; then
    echo "'~' serial channel isolation: OK"
else
    cat /tmp/ov_sct.out 2>/dev/null
    echo "'~' serial channel isolation: FAILED"; exit 1
fi

# Actually RENDER the GUIs: --selftest never runs the event loop. Needs Xvfb and
# an interpreter with tkinter (which may not be plain "python3"); skipped if absent.
OV_PY=""
for c in python3 python3.12 python3.11 python; do
    command -v $c >/dev/null 2>&1 || continue
    if $c -c "import tkinter" >/dev/null 2>&1; then OV_PY=$c; break; fi
done
if [ -z "$OV_PY" ]; then
    echo "gui renders: SKIPPED (no interpreter with tkinter)"
elif command -v xvfb-run >/dev/null 2>&1; then
    if timeout 90 xvfb-run -a -s "-screen 0 1280x800x24" \
         "$OV_PY" hostcheck/finetune_render_test.py > /tmp/ov_ftr.out 2>&1 \
       && grep -q "finetune render: OK" /tmp/ov_ftr.out; then
        echo "fine-tune renders + full two-station flow: OK"
    else
        tail -6 /tmp/ov_ftr.out 2>/dev/null
        echo "fine-tune renders + full two-station flow: FAILED"; exit 1
    fi
    if timeout 60 xvfb-run -a -s "-screen 0 1280x800x24" \
         "$OV_PY" hostcheck/gui_render_test.py > /tmp/ov_gui.out 2>&1 \
       && grep -q "gui render: OK" /tmp/ov_gui.out; then
        echo "gui renders (tick executed): OK"
    else
        tail -12 /tmp/ov_gui.out 2>/dev/null
        echo "gui renders: FAILED"; exit 1
    fi
else
    echo "gui renders: SKIPPED (no xvfb-run)"
fi

# The studio front-end, same rendering guard as the calibration GUI.
if [ -n "$OV_PY" ] && command -v xvfb-run >/dev/null 2>&1; then
    if timeout 60 xvfb-run -a -s "-screen 0 1280x900x24" \
         "$OV_PY" hostcheck/studio_render_test.py > /tmp/ov_studio.out 2>&1 \
       && grep -q "studio render: OK" /tmp/ov_studio.out; then
        echo "studio renders: OK"
    else
        tail -12 /tmp/ov_studio.out 2>/dev/null
        echo "studio renders: FAILED"; exit 1
    fi
else
    echo "studio renders: SKIPPED"
fi

# The verify tool, driven against a fake gun so the whole shoot-a-grid -> report
# -> CSV path runs. Needs xvfb-run and xdotool.
if [ -n "$OV_PY" ] && command -v xvfb-run >/dev/null 2>&1 \
   && command -v xdotool >/dev/null 2>&1; then
    if timeout 130 "$OV_PY" hostcheck/verify_fakegun.py > /tmp/ov_vg.out 2>&1 & then
        sleep 6
        VP=$(grep -o "FAKE_GUN_PORT=.*" /tmp/ov_vg.out | cut -d= -f2)
        if [ -n "$VP" ] && timeout 90 xvfb-run -a -s "-screen 0 1280x800x24" \
             "$OV_PY" hostcheck/verify_render_test.py "$VP" > /tmp/ov_vr.out 2>&1 \
           && grep -q "verify render: OK" /tmp/ov_vr.out; then
            echo "verify tool renders and reports: OK"
        else
            tail -12 /tmp/ov_vr.out 2>/dev/null
            echo "verify tool: FAILED"; exit 1
        fi
    fi
else
    echo "verify tool: SKIPPED (needs xvfb-run and xdotool)"
fi

