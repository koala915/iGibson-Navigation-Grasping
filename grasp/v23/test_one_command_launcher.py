#!/usr/bin/env python3
"""Flag-wiring tests for jetson_one_command_grasp.py (v23/E1). No hardware.

The v23 copy carries one extra job on top of v21's. v23 changes the arm pose, and
the arm camera rides on arm_link4, so EVERY piece of camera wiring that was
correct at C3 is wrong at E1 -- and wrong in the way this repo keeps getting
bitten by: both sides parse, both sides run, and the mismatch appears only after
the arm has powered up and moved. Section 1b pins the three that matter (the pose
stamp, the policy band, the calibration path) plus the weight pair, because a v21
model loaded here would pass every shape and contract check in the codebase.

Why this file exists
--------------------
The launcher went stale without anyone noticing. It kept passing the nav-home
extrinsics (--cam-x/--cam-y/--sign-y/--i-accept-predicted-extrinsics) long after
vision_grasp_bridge.py started requiring a MEASURED homography at the grasp
pose. Nothing catches that statically: both sides parse fine, both sides run
fine, and the mismatch only shows up on the robot — after the model is loaded,
the serial port is open and the arm has already walked to the grasp home — as

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
import hashlib
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


def _sha256(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _controller_config():
    """DeployConfig, read without importing torch or opening PyBullet.

    x3plus_real_grasp imports stable_baselines3 at module level, which is a
    multi-second import and pulls in torch. This file is meant to run on a
    laptop in under a second, so the dataclass defaults are read out of the
    source with ast instead.
    """
    tree = ast.parse((Path(L.HERE) / "x3plus_real_grasp.py").read_text(
        encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "DeployConfig":
            return {stmt.target.id: ast.literal_eval(stmt.value)
                    for stmt in node.body
                    if isinstance(stmt, ast.AnnAssign) and stmt.value is not None
                    and isinstance(stmt.target, ast.Name)
                    and _literal(stmt.value)}
    raise AssertionError("DeployConfig not found in x3plus_real_grasp.py")


def _literal(node):
    try:
        ast.literal_eval(node)
        return True
    except (ValueError, SyntaxError, TypeError):
        return False


def _controller_home_deg():
    return _controller_config()["home_deg"]


def _controller_ranges():
    cfg = _controller_config()
    return cfg["trained_x_range"], cfg["trained_y_range"]


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
check("--allow-top-clipped-grasp-home" not in cmd,
      "top-clipped detections stay fail-closed by default")
opt_in_cmd = L.build_bridge_cmd(make_args(
    homography="/tmp/h.json", allow_top_clipped=True))
check("--allow-top-clipped-grasp-home" in opt_in_cmd,
      "the explicit top-only clipping opt-in reaches the bridge")
for dead in ("--cam-x", "--cam-y", "--sign-y", "--i-accept-predicted-extrinsics"):
    # Not merely unused: at a grasp home the mapping is the homography, so
    # forwarding these would print numbers that look like they steer the arm
    # while changing nothing.
    check(dead not in cmd, f"{dead} is not passed at the E1 grasp pose")
check("--calibration-only" not in cmd, "not in calibration mode")
bridge_source = Path(L.BRIDGE).read_text(encoding="utf-8")
controller_source = Path(L.CONTROLLER).read_text(encoding="utf-8")
check("--grasp-forward-offset-mm" in cmd
      and '"--grasp-forward-offset-mm"' in bridge_source
      and "def apply_grasp_forward_offset(" in bridge_source
      and '"--tcp-forward-error-mm"' in controller_source
      and "def control_tcp_position(" in controller_source,
      "the two X corrections reach compatible bridge/controller files")
check(float(cmd[cmd.index("--grasp-forward-offset-mm") + 1]) == 5.0,
      "the conservative default correction is +5 mm")

print("\n1b. every E1-specific wire, because none of them fail loudly")
sys.path.insert(0, str(L.INTEGRATION))
import arm_cam_geometry as acg          # noqa: E402  (path set just above)

cmd = L.build_bridge_cmd(make_args(homography="/tmp/h.json"))

# The pose stamp. The bridge's grasp-home default is still C3 -- deliberately,
# because modes A/B/C all still run v21 -- so omitting --pose here does not
# error, it stamps C3. The grasp side then compares that stamp against its
# encoders and drops every frame on a 7.1 deg S2 residual.
check("--pose" in cmd, "--pose is passed (the bridge's default is still C3)")
pose_name = cmd[cmd.index("--pose") + 1]
check(pose_name == "v23_e1_grasp_home", f"...and names E1, not C3 ({pose_name})")
check(pose_name in acg.GRASP_HOME_POSE_NAMES,
      "the name is a registered grasp-home pose, not a typo the bridge rejects")

# The stamp has to describe the pose the controller actually parks at. This is
# the check that turns a silent 100%-drop into a red test on a laptop.
ctrl_home = _controller_home_deg()
stamp_deg = acg.get_pose(pose_name).arm_deg
check(max(abs(a - b) for a, b in zip(stamp_deg[:5], ctrl_home[:5])) < 0.05,
      f"the stamped pose {list(stamp_deg[:5])} matches the controller's home "
      f"{list(ctrl_home[:5])}")

# The policy band. E1's FOV is 44% larger than C3's while v23's trained band is
# NARROWER than v21's, so the camera really can see ground the policy has never
# been evaluated on.
check("--policy-x-range" in cmd and "--policy-y-range" in cmd,
      "--policy-x-range/--policy-y-range are passed")
bridge_x = (float(cmd[cmd.index("--policy-x-range") + 1]),
            float(cmd[cmd.index("--policy-x-range") + 2]))
bridge_y = (float(cmd[cmd.index("--policy-y-range") + 1]),
            float(cmd[cmd.index("--policy-y-range") + 2]))
ctrl_x, ctrl_y = _controller_ranges()
check(bridge_x == tuple(ctrl_x),
      f"bridge x band {bridge_x} equals the controller's {tuple(ctrl_x)}")
check(bridge_y == tuple(ctrl_y),
      f"bridge y band {bridge_y} equals the controller's {tuple(ctrl_y)}")

# The calibration path. A C3 file is valid JSON, passes the same >=6-point
# 2 cm gate, and records nothing about which pose it was measured at. Separate
# filenames are the only thing keeping them apart.
check(Path(L.HOMOGRAPHY).name != "grasp_home_homography.json",
      f"the default calibration path is not v21's ({Path(L.HOMOGRAPHY).name})")
check("e1" in Path(L.HOMOGRAPHY).name.lower(), "...and says E1 in its name")

# The weights. v21 and v23 share the 28D/6D obs and the incremental decode, so
# nothing in any shape or contract check can tell the pairs apart -- only the
# manifest sha256 can, and only if the launcher points at the right files.
manifest = json.loads((Path(L.HERE) / "manifest.json").read_text(encoding="utf-8"))
for label, path, key in (("model", L.PPO_MODEL, "model"),
                         ("vecnorm", L.VECNORM, "vecnormalize")):
    want = manifest["artifacts"][key]
    check(Path(path).name == Path(want["file"]).name,
          f"launcher {label} is the one manifest.json documents")
    check(Path(path).is_file() and _sha256(path) == want["sha256"],
          f"...and its sha256 matches ({label})")

print("\n2. calibrate mode collects pixels and sends nothing")
cmd = L.build_bridge_cmd(make_args(calibrate=True, calibration_samples=25))
check("--calibration-only" in cmd, "--calibration-only")
check("--dry-run" in cmd, "--dry-run: the TCP socket is never opened")
check(cmd[cmd.index("--calibration-samples") + 1] == "25", "sample count forwarded")
check("--homography" not in cmd, "no homography required to produce one")
check("--once" not in cmd, "keeps running so several positions can be measured")
check("--grasp-forward-offset-mm" not in cmd,
      "calibration records measured geometry without an operational correction")
check("--allow-top-clipped-grasp-home" not in L.build_bridge_cmd(
    make_args(calibrate=True, allow_top_clipped=True)),
      "calibration never relaxes the clipped-bbox gate")

print("\n3. --show is opt-in (a headless SSH session has no display)")
check("--show" not in L.build_bridge_cmd(make_args()), "off by default")
check("--show" in L.build_bridge_cmd(make_args(show=True)), "on when asked")

print("\n3b. target correction is reversible and bounded")
zero_cmd = L.build_bridge_cmd(make_args(grasp_forward_offset_mm=0.0))
check(float(zero_cmd[zero_cmd.index("--grasp-forward-offset-mm") + 1]) == 0.0,
      "--grasp-forward-offset-mm 0 disables the correction")
check(L.DEFAULT_GRASP_FORWARD_OFFSET_MM == 5.0
      and L.MAX_GRASP_FORWARD_OFFSET_MM == 15.0
      and L.DEFAULT_FLOOR_FINGER_ERROR_MM == 15.0
      and L.DEFAULT_JAW_TRACK_FRACTION == 0.5
      and L.DEFAULT_TCP_FORWARD_ERROR_MM == 20.0
      and L.MAX_TCP_FORWARD_ERROR_MM == 20.0,
      "the 3/3 hardware defaults and hard upper bounds stay explicit")

print("\n4. the controller holds C3 long enough to measure from")
ctrl = L.build_ctrl_cmd(make_args(calibrate=True, latch_wait=120.0))
wait = float(ctrl[ctrl.index("--latch-wait") + 1])
check(wait >= 1800.0, f"calibrate stretches --latch-wait to {wait:.0f}s")
ctrl = L.build_ctrl_cmd(make_args(latch_wait=120.0))
check(float(ctrl[ctrl.index("--latch-wait") + 1]) == 120.0,
      "grasp mode keeps the operator's value")
check("--s6-stall-grasp-steps" in ctrl
      and float(ctrl[ctrl.index("--floor-finger-error-mm") + 1]) == 15.0
      and float(ctrl[ctrl.index("--jaw-track-fraction") + 1]) == 0.5
      and float(ctrl[ctrl.index("--tcp-forward-error-mm") + 1]) == 20.0,
      "the complete 3/3 hardware tuple reaches the controller")
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
print("9. the vision bridge loads in parallel, but still looks only after E1")
# The whole point of --wait-for-go is that the slow ultralytics import overlaps
# the controller's startup. The safety half is that overlapping the LOAD must not
# overlap the LOOK: the arm-camera geometry is only valid at E1, and the pose
# stamp cannot catch a stale picture -- it is read when the detection is sent, so
# it would truthfully say "home" about a frame taken while the arm was moving.
for mode, kwargs in (("grasp", {}), ("calibrate", {"calibrate": True})):
    wcmd = L.build_bridge_cmd(make_args(homography="/tmp/h.json", **kwargs))
    check("--wait-for-go" in wcmd,
          f"the launcher asks the bridge to wait for a go signal ({mode} mode)")
check("--wait-for-go" in bridge_source,
      "...and the bridge actually offers that flag")

gate = bridge_source.find("if args.wait_for_go:")
camera = bridge_source.find('print(f"[bridge] opening camera')
model = bridge_source.find("model = YOLO(args.model)")
check(gate != -1 and camera != -1 and model != -1,
      "the bridge has a go gate, a model load and a camera open to order")
check(model < gate,
      "the model loads BEFORE the gate, so the slow import is what overlaps")
# open_capture_checked keeps its probe frame and the detect loop consumes it as
# frame one, so a camera opened early would feed a pre-home picture into
# home-pose geometry.
check(gate < camera,
      "the gate is BEFORE the camera opens, so the first frame is taken after E1")
gate_body = bridge_source[gate:camera]
check("SystemExit" in gate_body and "sys.stdin.readline()" in gate_body,
      "EOF on stdin aborts instead of detecting at an unverified arm pose")

start = source.find("bridge = subprocess.Popen(")
wait = source.find("while not home_ready.wait(")
release = source.find('bridge.stdin.write("go')
check(start != -1 and wait != -1 and release != -1,
      "the launcher has a bridge start, a home wait and a go signal to order")
check(start < wait,
      "the bridge is started BEFORE the wait for E1, or nothing overlaps")
check(wait < release,
      "the go signal is sent only after E1 is confirmed")
check("stdin=subprocess.PIPE" in source[start:release],
      "the launcher keeps the bridge's stdin so it can release it")
# Otherwise the launcher waits out the whole startup timeout and blames the
# controller for a bridge that died loading.
check("bridge.poll() is not None" in source[wait:release],
      "a bridge that dies while loading is noticed during the home wait")

print()
if failures:
    print(f"{len(failures)} of {checks} checks FAILED:")
    for f in failures:
        print(f"  - {f}")
    sys.exit(1)
print(f"all {checks} checks passed — the v23 launcher is wired to the bridge, "
      f"the E1 pose and the weights the repo actually has")
sys.exit(0)
