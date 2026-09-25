#!/usr/bin/env python3
"""Flag-wiring tests for jetson_one_command_grasp.py. No hardware, no camera.

Why this file exists
--------------------
The launcher went stale without anyone noticing. It kept passing the nav-home
extrinsics (--cam-x/--cam-y/--sign-y/--i-accept-predicted-extrinsics) long after
vision_grasp_bridge.py started requiring a MEASURED homography at the C3 grasp
pose. Nothing catches that statically: both sides parse fine, both sides run
fine, and the mismatch only shows up on the robot — after the model is loaded,
the serial port is open and the arm has already walked to C3 — as

    grasp-home detection requires --homography; nav-home H/theta/cam offsets
    are invalid after the arm moves

which then takes the whole run down with it.

So the two things checked here are exactly the two that failed: the flags handed
to the bridge, and the calibration gate running BEFORE any hardware is touched.

The launcher is also deliberately parseable by the Jetson's system python3
(3.6.9) so a wrong interpreter reports itself instead of raising SyntaxError
inside the import block. The last case pins that down.
"""
from __future__ import annotations

import ast
import json
from pathlib import Path
import sys
import tempfile

import jetson_one_command_grasp as L


checks = 0
failures = []


def check(cond, label):
    global checks
    checks += 1
    if not cond:
        failures.append(label)
        print(f"  [FAIL] {label}")
    else:
        print(f"  [ok]   {label}")


def make_args(**overrides):
    """A parsed-args stand-in with the launcher's own defaults applied."""
    args = L.parse_args_from([])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def write_calibration(path, points):
    """A valid calibration document, built through the real fitter."""
    sys.path.insert(0, str(L.INTEGRATION))
    from grasp_home_homography import make_calibration, save_calibration
    pixels = [(p[0], p[1]) for p in points]
    bases = [(p[2], p[3]) for p in points]
    save_calibration(path, make_calibration(pixels, bases))


GOOD_POINTS = [
    (200.0, 380.0, 0.300, 0.050), (440.0, 380.0, 0.300, -0.050),
    (200.0, 470.0, 0.240, 0.050), (440.0, 470.0, 0.240, -0.050),
    (320.0, 425.0, 0.270, 0.000), (260.0, 500.0, 0.220, 0.030),
    (380.0, 350.0, 0.315, -0.028),
]


print("1. grasp mode passes the homography and none of the nav-home extrinsics")
cmd = L.build_bridge_cmd(make_args(homography="/tmp/h.json"))
check("--homography" in cmd, "--homography is passed")
check(cmd[cmd.index("--homography") + 1] == "/tmp/h.json", "with the given path")
check("--once" in cmd, "--once: one detection, then YOLO's RAM comes back")
for dead in ("--cam-x", "--cam-y", "--sign-y", "--i-accept-predicted-extrinsics"):
    # Not merely unused: at C3 the mapping is the homography, so forwarding these
    # would print numbers that look like they steer the arm while changing nothing.
    check(dead not in cmd, f"{dead} is not passed at the C3 grasp pose")
check("--calibration-only" not in cmd, "not in calibration mode")

print("\n2. calibrate mode collects pixels and sends nothing")
cmd = L.build_bridge_cmd(make_args(calibrate=True, calibration_samples=25))
check("--calibration-only" in cmd, "--calibration-only")
check("--dry-run" in cmd, "--dry-run: the TCP socket is never opened")
check(cmd[cmd.index("--calibration-samples") + 1] == "25", "sample count forwarded")
check("--homography" not in cmd, "no homography required to produce one")
check("--once" not in cmd, "keeps running so several positions can be measured")

print("\n3. --show is opt-in (a headless SSH session has no display)")
check("--show" not in L.build_bridge_cmd(make_args()), "off by default")
check("--show" in L.build_bridge_cmd(make_args(show=True)), "on when asked")

print("\n4. the controller holds C3 long enough to measure from")
ctrl = L.build_ctrl_cmd(make_args(calibrate=True, latch_wait=120.0))
wait = float(ctrl[ctrl.index("--latch-wait") + 1])
check(wait >= 1800.0, f"calibrate stretches --latch-wait to {wait:.0f}s")
ctrl = L.build_ctrl_cmd(make_args(latch_wait=120.0))
check(float(ctrl[ctrl.index("--latch-wait") + 1]) == 120.0,
      "grasp mode keeps the operator's value")
check("--s6-stall-grasp-steps" in ctrl, "S6 stall shortcut (sets the 1 deg hold bias)")
check("--unlock-candidate-real" in ctrl, "manifest is still candidate")

print("\n5. the calibration gate matches the bridge's runtime gate")
check(L.HOMOGRAPHY_MIN_POINTS == 6 and L.HOMOGRAPHY_MAX_ERROR_M == 0.02,
      "6 points / 2 cm, as required by vision_grasp_bridge.resolve_pose")
with tempfile.TemporaryDirectory() as tmp:
    good = str(Path(tmp) / "good.json")
    write_calibration(good, GOOD_POINTS)
    check(L.check_homography(good), "a 7-point fit is accepted")

    check(not L.check_homography(str(Path(tmp) / "absent.json")),
          "a missing file fails before the serial port is opened")

    thin = str(Path(tmp) / "thin.json")
    write_calibration(thin, GOOD_POINTS[:4])
    check(not L.check_homography(thin),
          "the 4-point mathematical minimum is rejected: it fits anything")

    broken = str(Path(tmp) / "broken.json")
    Path(broken).write_text("{not json", encoding="utf-8")
    check(not L.check_homography(broken), "an unparseable file is rejected")

    tampered = str(Path(tmp) / "tampered.json")
    doc = json.loads(Path(good).read_text(encoding="utf-8"))
    doc["homography"][0][0] = doc["homography"][0][0] * 2.0
    Path(tampered).write_text(json.dumps(doc), encoding="utf-8")
    check(not L.check_homography(tampered),
          "a matrix edited away from its own points is rejected")

print("\n6. preflight refuses to start when the camera is already open")
check(L.camera_holders("/dev/does-not-exist-x3plus") == [],
      "a free (or absent) device reports no holder")

print("\n7. the launcher stays parseable by the Jetson's system python3 (3.6.9)")
source = (Path(L.__file__).with_suffix(".py")).read_text(encoding="utf-8")
tree = ast.parse(source)
futures = [alias.name for node in ast.walk(tree)
           if isinstance(node, ast.ImportFrom) and node.module == "__future__"
           for alias in node.names]
check("annotations" not in futures,
      "no `from __future__ import annotations` (3.6 cannot parse it)")
check(not any(isinstance(node, ast.NamedExpr) for node in ast.walk(tree)),
      "no walrus operator")
check("[FATAL]" in source and "3.8" in source,
      "says which interpreter is needed rather than dying with a SyntaxError")

print()
if failures:
    print(f"{len(failures)} of {checks} checks FAILED:")
    for f in failures:
        print(f"  - {f}")
    sys.exit(1)
print(f"all {checks} checks passed — the one-command launcher is wired to the "
      f"bridge the repo actually has")
sys.exit(0)
