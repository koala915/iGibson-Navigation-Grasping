#!/usr/bin/env python3
"""Unified self-driving grasp pipeline for X3Plus.

One process owns the Rosmaster serial link and does the whole loop:

    1. detect object (YOLOv11, dual camera)
    2. drive the mecanum base toward it       (set_car_motion)
    3. stop once it is within arm reach        (hand off, no blind push)
    4. grasp with the PPO arm policy           (vision gives x/y/z + width)
    5. if the grasp failed (object still seen at the same spot), retry from 2
       — up to --max-retries times.

Why one process: the same Rosmaster object drives BOTH the wheels
(`set_car_motion`) and the arm servos (`set_uart_servo_angle_array`). Running
the chassis and the arm from separate processes would fight over the Rosmaster
serial port,
so the orchestrator builds the grasp controller (which opens Rosmaster) and
reuses `controller.servo.device` for the chassis.

Navigation logic (distance/offset geometry, decision thresholds, speed
calibration) is ported from detection/rear_nav/rear_to_arm_blind_handoff.py,
but the actuator is changed from the port-7000 motor server to direct
`set_car_motion`, and the final ARM blind-push is replaced by a hand-off to the
arm policy (which already reaches objects at x≈0.25 m).

HARDWARE PREREQUISITES
    * The port-7000 motor server / ROS base driver must NOT be running
      (they would own the serial port).
    * Camera streams: arm + rear MJPEG on :8080 (see URL_ARM / URL_REAR).

Run:
    python vision_grasp_pipeline.py --selftest          # logic check, no hw/cam
    python vision_grasp_pipeline.py                      # dry-run (no motion), needs cams

The integrated --real path is intentionally refused until measured grasp-home
homography mapping is connected to final alignment and target latching.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_cam_geometry as acg  # noqa: E402  (pure math, no cv2/torch)

# Heavy / platform deps (cv2, ultralytics, the grasp module with torch+pybullet)
# are imported lazily inside the classes/functions that need them, so this file
# imports — and --selftest runs — on a plain dev machine.

# ════════════════════════════════════════════════════════════════════════════
# Calibration constants — measured on the robot 2026-07-08 (Phase 1/2,
# chessboard 6x9 inner corners @20mm; see progress.md / docs/calibration/CALIBRATION_PLAN.md).
# ════════════════════════════════════════════════════════════════════════════

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "127.0.0.1")
URL_REAR = f"http://{JETSON_IP}:8080/stream?topic=/back_cam/image_raw"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic=/arm_cam/image_raw"

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

# Rear camera distance model (rear_cam_intrinsics.json, RMS 0.374px).
# Distortion at the bbox bottom-center is 1.79px -> ignored (Phase 1 decision).
# Ground model validated 0.70–1.50m, max error 0.98cm.
THETA_REAR = 16.35
H_REAR = 0.503
REAR_CAM_TO_FRONT_M = 0.20
FY_REAR = 544.82
CY_REAR = 244.79
FX_REAR = 544.16
CX_REAR = 316.98

# ── Arm camera: everything comes from integration/arm_cam_geometry.py ─────────
# That module is the single source of truth for the arm camera's intrinsics,
# distortion, and per-pose extrinsics, and it also carries the corrected ground
# model. The names below are kept as module-level aliases because callers
# (nav_rl_grasp_pipeline, the tests, --dump-frame) refer to them, but they are
# views onto the registry, not a second copy of the numbers.
#
# ARM_CAM_POSE decides which extrinsic set is in force. It defaults to the pose
# the v21 stack actually starts from (C3), NOT the v17 nav home the numbers were
# originally measured at -- see check_arm_cam_pose() for what happens when the
# arm is somewhere else.
ARM_CAM_POSE = acg.DEFAULT_POSE
THETA_ARM = ARM_CAM_POSE.theta_deg
H_ARM = ARM_CAM_POSE.h_m
ARM_CAM_CALIBRATED_AT_HOME_DEG = ARM_CAM_POSE.arm_deg
ARM_CAM_POSE_TOL_DEG = acg.POSE_TOL_DEG
FY_ARM = acg.FY
CY_ARM = acg.CY
FX_ARM = acg.FX
CX_ARM = acg.CX
DIST_ARM = acg.DIST  # k1 k2 p1 p2 k3

# Chassis speed calibration: vx ≈ KX*speed (m/s), wz ≈ KZ*speed (rad/s)
KX = 0.006995
KZ = 0.02384

YOLO_CONF = 0.3
IMG_SIZE = 640
MAX_FRAME_AGE = 0.8

REAR_DECISION_INTERVAL = 0.5
ARM_DECISION_INTERVAL = 0.5

# REAR control
FAR_VX_MPS = 0.22
MID_VX_MPS = 0.18
NEAR_VX_MPS = 0.13
MIN_SPEED = 28
MAX_SPEED = 48
FAR_DIST_M = 1.60
MID_DIST_M = 1.10
NEAR_DIST_M = 0.90
REAR_BLIND_START_DIST_M = 0.90
ARM_EXPECTED_CAPTURE_DIST_M = 0.70
MIN_REAR_BLIND_DIST_M = 0.12
MAX_REAR_BLIND_DIST_M = 0.38
REAR_BLIND_VX_MPS = 0.13
OFFSET_FORWARD_DEADZONE_M = 0.06
OFFSET_TURN_THRESHOLD_M = 0.28
REAR_BLIND_MAX_OFFSET_M = 0.13
TURN_SPEED_RATIO = 0.85

# ARM control
ARM_FAR_VX_MPS = 0.10
ARM_MID_VX_MPS = 0.07
ARM_NEAR_VX_MPS = 0.045
# The integrated runtime writes SI velocities with set_car_motion(). The old
# TCP/PWM minimum of 24 would turn a requested 0.045 m/s approach into 0.168
# m/s, so it must not be applied to this path. A board that cannot execute the
# lower command must stop rather than silently exceed the safety limit.
ARM_MIN_SPEED = 0
ARM_MAX_SPEED = 38
ARM_FAR_DIST_M = 0.70
ARM_MID_DIST_M = 0.45
ARM_NEAR_DIST_M = 0.30
ARM_BLIND_START_DIST_M = 0.24   # object this close & centered -> hand off to arm policy
ARM_OFFSET_FORWARD_DEADZONE_M = 0.035
ARM_OFFSET_TURN_THRESHOLD_M = 0.12
ARM_KP_WZ = 0.5
ARM_MAX_WZ = 0.35
ARM_MIN_TURN_SPEED = 0

# ── pipeline-specific (new) ──
# Camera -> arm-base frame mapping for the latched grasp target. These now come
# from the active pose in arm_cam_geometry rather than being hard zeros. The old
# CAM_TO_BASE_X = 0.0 was survivable at the nav home, where the ground distance
# is ~45 cm and the offset is a correction; at the C3 pose the ground distance is
# only +-4 cm, so cam_x IS the answer and a zero would put the target on top of
# the arm base.
CAM_TO_BASE_X = ARM_CAM_POSE.cam_x_m
CAM_TO_BASE_Y = ARM_CAM_POSE.cam_y_m
SIGN_Y = ARM_CAM_POSE.sign_y   # +1 or -1: image-right -> grasp +Y
OBJ_Z_FIXED = 0.02      # object centroid z in the grasp frame (m)
# Width gate: an object wider than the gripper can open is physically
# ungraspable — abort immediately instead of wasting retries. This is a pipeline-side
# constant: v21 has no width-sizing config of its own (v17's
# DeployConfig.grip_max_object_width_m is gone along with --width-grip), so this
# number must be maintained here against the gripper's measured max opening.
MAX_GRASP_WIDTH_M = 0.06
VERIFY_TOL_M = 0.06     # re-detected object within this of latched pos ⇒ grasp failed
TARGET_LOST_TIMEOUT_S = 4.0   # give up approach if nothing seen this long
APPROACH_MAX_DURATION_S = 120.0  # hard stop even if detections keep arriving
RETREAT_TIME_S = 0.6          # short reverse before a retry
RETREAT_VX_MPS = 0.12


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def parse_class_z(values: List[str], fallback_z: float) -> Dict[str, float]:
    """Parse repeated --class-z entries like sugarbox=0.0325.

    The value is the object's CENTROID z in the grasp frame, NOT its height --
    see parse_class_height. They coincide only via height = 2*centroid_z, and only
    for a symmetric object resting on the floor.

    NAME must match a class the loaded detection/models/best.pt actually reports
    (check with `YOLO('detection/models/best.pt').names`) -- an unmatched name
    falls back to fallback_z, so warn_unknown_classes() below exists to make a
    stale name from before a model swap visible. best.pt has been, in order:
    bottle-cap/paper-ball (train5), eraser-detect (train11, 2026-07-31), and
    sugarbox (2026-08-01).
    """
    out: Dict[str, float] = {}
    for raw in values:
        if "=" not in raw:
            raise ValueError(f"--class-z must be NAME=CENTROID_Z_M, got: {raw}")
        name, value = raw.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"--class-z has an empty class name: {raw}")
        z = float(value)
        if not math.isfinite(z) or z < 0.0:
            raise ValueError(f"--class-z value must be finite and non-negative: {raw}")
        out[name] = z
    out.setdefault("_fallback", float(fallback_z))
    return out


def parse_class_height(values: List[str]) -> Dict[str, float]:
    """Parse repeated --class-height entries like sugarbox=0.065.

    The value is the object's FULL Z EXTENT (top minus floor) in metres. The grasp
    side uses it for wrist_z_offset and the engagement-depth gates.

    Deliberately has NO fallback. An undeclared class sends no height at all, and
    the grasp side applies its own documented symmetric-object estimate; that is a
    known, inspectable behaviour. A fallback height would instead hand the grasp
    side a number indistinguishable from a measurement.
    """
    out: Dict[str, float] = {}
    for raw in values:
        if "=" not in raw:
            raise ValueError(f"--class-height must be NAME=HEIGHT_M, got: {raw}")
        name, value = raw.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"--class-height has an empty class name: {raw}")
        h = float(value)
        if not math.isfinite(h) or h <= 0.0:
            raise ValueError(f"--class-height must be finite and positive: {raw}")
        out[name] = h
    return out


def check_arm_cam_pose(detect_home_deg, *, real: bool, acknowledged: bool = False) -> bool:
    """Refuse to run --real when detection happens at a pose the camera isn't calibrated for.

    Returns True if the pose matches ARM_CAM_CALIBRATED_AT_HOME_DEG. Under --real a
    mismatch raises SystemExit unless explicitly acknowledged, because there is no
    downstream check that can catch it: the distance model returns a plausible number
    at any pose, and the arm then grasps confidently in the wrong place.
    """
    cur = tuple(float(v) for v in detect_home_deg)
    if ARM_CAM_POSE.matches(cur, ARM_CAM_POSE_TOL_DEG):
        # Right pose — but the extrinsics for it may still be a URDF prediction
        # rather than a measurement, which is its own refusal.
        return ARM_CAM_POSE.require_measured(real=real, acknowledged=acknowledged)

    other = acg.pose_for_arm_deg(cur, ARM_CAM_POSE_TOL_DEG)
    hint = (f"    That pose IS registered as {other.name!r}; select it instead of\n"
            f"    {ARM_CAM_POSE.name!r}.\n" if other is not None else "")
    msg = (
        "arm-camera geometry does not belong to this detection pose.\n"
        f"    detection pose (cfg.home_deg) : {list(cur)}\n"
        f"    active extrinsics            : {ARM_CAM_POSE.describe()}\n"
        + hint +
        "    The camera rides on arm_link4, so theta/H/cam_x/cam_y are mounting\n"
        "    geometry valid at ONE pose only, and nothing downstream can catch a\n"
        "    mismatch: the model returns a confident number at any pose and the arm\n"
        "    then grasps precisely in the wrong place.\n"
        "    Fix: re-measure at the pose you actually detect from (docs/calibration/CALIBRATION_PLAN.md\n"
        "    Phase 1/2 + 3 — intrinsics and distortion carry over unchanged), or drive\n"
        "    the arm to a calibrated pose for detection via --nav-home-deg and set\n"
        "    --grasp-home-deg to the trained C3 pose."
    )
    if real and not acknowledged:
        raise SystemExit("[pipeline] REFUSED: " + msg)
    print("[pipeline] WARN: " + msg)
    return False


def warn_unknown_classes(model, tables) -> List[str]:
    """Warn about class names the loaded model does not predict. Returns the names.

    A typo'd or stale class name is otherwise silent: the lookup misses, the
    fallback is used, and the whole run looks normal while the arm approaches at
    the wrong height. Not fatal -- a mixed-model workflow is legitimate -- but it
    must be visible.
    """
    try:
        names = model.names
        known = set(names.values() if isinstance(names, dict) else names)
    except Exception:
        return []
    unknown = []
    for flag, table in tables:
        for name in table:
            if name != "_fallback" and name not in known:
                unknown.append(name)
                print(f"[pipeline] WARN: {flag} names {name!r}, which this model does "
                      f"not predict. Known classes: {sorted(known)}")
    return unknown


def class_name_for_box(model, box) -> str:
    cls_id = int(box.cls[0].item())
    names = model.names
    if isinstance(names, dict):
        return str(names.get(cls_id, cls_id))
    if 0 <= cls_id < len(names):
        return str(names[cls_id])
    return str(cls_id)


# ════════════════════════════════════════════════════════════════════════════
# Geometry + decision functions (ported, motor-server-free)
# ════════════════════════════════════════════════════════════════════════════

def distort_pixel(u, v, fx, fy, cx, cy, dist):
    """Forward plumb-bob distortion of an ideal pixel (selftest round-trip)."""
    k1, k2, p1, p2, k3 = dist
    x = (u - cx) / fx
    y = (v - cy) / fy
    r2 = x * x + y * y
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return cx + xd * fx, cy + yd * fy


def undistort_pixel(u, v, fx, fy, cx, cy, dist, iters=8):
    """Undistort one pixel coordinate (plumb-bob k1,k2,p1,p2,k3).

    Pure-math equivalent of cv2.undistortPoints(..., P=K) (same fixed-point
    iteration) so this module stays importable without OpenCV for --selftest.
    """
    k1, k2, p1, p2, k3 = dist
    xd = (u - cx) / fx
    yd = (v - cy) / fy
    x, y = xd, yd
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return cx + x * fx, cy + y * fy


def estimate_ground_distance(y2, theta_deg, h_m, fy, cy):
    """Forward ground distance to the box bottom (m); -1 if above horizon."""
    theta = math.radians(theta_deg)
    alpha = math.atan((y2 - cy) / fy)
    total_angle = theta + alpha
    if total_angle <= 0:
        return -1.0
    return h_m / math.tan(total_angle)


def optical_depth_from_ground_distance(dist_m, theta_deg, h_m):
    """Convert horizontal ground distance to pinhole optical-axis depth.

    A pitched camera observes a ground point at
    ``Z_cam = D*cos(theta) + H*sin(theta)``. Horizontal pixel offsets and
    widths must use this optical depth; using D directly under-scales the arm
    camera's lateral geometry by roughly 40% at the Phase-3 working distance.
    """
    if dist_m <= 0:
        return 0.0
    theta = math.radians(theta_deg)
    return dist_m * math.cos(theta) + h_m * math.sin(theta)


def estimate_pitched_offset_x(cx_box, dist_m, fx, cx_cam, theta_deg, h_m):
    """Right-positive lateral offset for a pitched camera."""
    depth_m = optical_depth_from_ground_distance(dist_m, theta_deg, h_m)
    return depth_m * (cx_box - cx_cam) / fx


def estimate_pitched_width(width_px, dist_m, fx, theta_deg, h_m):
    """Metric image-plane width for a pitched camera."""
    if width_px <= 0:
        return 0.0
    depth_m = optical_depth_from_ground_distance(dist_m, theta_deg, h_m)
    return width_px * depth_m / fx


def rear_dist_to_front_dist(dist_rear_m):
    if dist_rear_m <= 0:
        return -1.0
    return max(0.0, dist_rear_m - REAR_CAM_TO_FRONT_M)


def estimate_offset_x(cx_box, dist_m, fx, cx_cam):
    """Lateral offset (m): >0 object to the right, <0 to the left."""
    if dist_m <= 0:
        return 0.0
    return dist_m * (cx_box - cx_cam) / fx


def speed_from_vx(vx_mps, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KX <= 1e-9:
        return min_speed
    return int(round(clamp(vx_mps / KX, min_speed, max_speed)))


def speed_from_wz(wz_rad_s, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KZ <= 1e-9:
        return min_speed
    # Floor instead of round: the reconstructed SI command must never exceed
    # the angular cap the caller just applied.
    return int(math.floor(clamp(abs(wz_rad_s) / KZ, min_speed, max_speed)))


def get_rear_distance_state(dist_front_m):
    if dist_front_m <= 0:
        return "INVALID"
    if dist_front_m <= REAR_BLIND_START_DIST_M:
        return "BLIND_READY"
    if dist_front_m <= NEAR_DIST_M:
        return "NEAR"
    if dist_front_m <= MID_DIST_M:
        return "MID"
    if dist_front_m <= FAR_DIST_M:
        return "FAR"
    return "VERY_FAR"


def choose_rear_base_speed(distance_state):
    if distance_state in ("VERY_FAR", "FAR"):
        return speed_from_vx(FAR_VX_MPS)
    if distance_state == "MID":
        return speed_from_vx(MID_VX_MPS)
    if distance_state in ("NEAR", "BLIND_READY"):
        return speed_from_vx(NEAR_VX_MPS)
    return 0


def compute_rear_blind_plan(dist_front_m):
    blind_dist_m = clamp(dist_front_m - ARM_EXPECTED_CAPTURE_DIST_M,
                         MIN_REAR_BLIND_DIST_M, MAX_REAR_BLIND_DIST_M)
    if REAR_BLIND_VX_MPS <= 1e-6:
        blind_time_s = 1.0
    else:
        blind_time_s = blind_dist_m / REAR_BLIND_VX_MPS
    blind_time_s = clamp(blind_time_s, 0.6, 3.2)
    return blind_dist_m, blind_time_s


def decide_rear_action_by_offset(target_found, offset_x_m, dist_front_m):
    if not target_found:
        return "stop", 0, "TARGET LOST", "NO_TARGET"

    distance_state = get_rear_distance_state(dist_front_m)
    base_speed = choose_rear_base_speed(distance_state)

    if distance_state == "INVALID":
        return "stop", 0, "INVALID DIST", distance_state

    if distance_state == "BLIND_READY":
        if abs(offset_x_m) <= REAR_BLIND_MAX_OFFSET_M:
            return "stop", 0, "ENTER REAR BLIND", distance_state
        turn_speed = int(clamp(round(base_speed * TURN_SPEED_RATIO), MIN_SPEED, MAX_SPEED))
        if offset_x_m < 0:
            return "turn_left", turn_speed, "BLIND NEAR BUT LEFT OFFSET", distance_state
        return "turn_right", turn_speed, "BLIND NEAR BUT RIGHT OFFSET", distance_state

    if abs(offset_x_m) <= OFFSET_FORWARD_DEADZONE_M:
        return "forward", base_speed, "CENTER: FORWARD", distance_state

    if abs(offset_x_m) >= OFFSET_TURN_THRESHOLD_M:
        turn_speed = int(clamp(round(base_speed * TURN_SPEED_RATIO), MIN_SPEED, MAX_SPEED))
        if offset_x_m < 0:
            return "turn_left", turn_speed, "LARGE LEFT OFFSET: TURN LEFT", distance_state
        return "turn_right", turn_speed, "LARGE RIGHT OFFSET: TURN RIGHT", distance_state

    if offset_x_m < 0:
        return "curve_left", base_speed, "LEFT OFFSET: CURVE LEFT", distance_state
    return "curve_right", base_speed, "RIGHT OFFSET: CURVE RIGHT", distance_state


def get_arm_distance_state(dist_arm_m):
    if dist_arm_m <= 0:
        return "INVALID"
    if dist_arm_m <= ARM_BLIND_START_DIST_M:
        return "ARM_BLIND_READY"
    if dist_arm_m <= ARM_NEAR_DIST_M:
        return "ARM_NEAR"
    if dist_arm_m <= ARM_MID_DIST_M:
        return "ARM_MID"
    if dist_arm_m <= ARM_FAR_DIST_M:
        return "ARM_FAR"
    return "ARM_VERY_FAR"


def choose_arm_forward_speed(distance_state):
    if distance_state in ("ARM_VERY_FAR", "ARM_FAR"):
        return speed_from_vx(ARM_FAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)
    if distance_state == "ARM_MID":
        return speed_from_vx(ARM_MID_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)
    if distance_state in ("ARM_NEAR", "ARM_BLIND_READY"):
        return speed_from_vx(ARM_NEAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)
    return 0


def decide_arm_action_by_distance(target_found, offset_x_m, dist_arm_m):
    """Returns (action, speed, status, distance_state, desired_wz).

    action == "handoff" means: centered and within arm reach -> stop & grasp
    (this replaces the old "arm_blind_push").
    """
    if not target_found:
        return "stop", 0, "TARGET LOST", "NO_TARGET", 0.0

    distance_state = get_arm_distance_state(dist_arm_m)

    if distance_state == "INVALID":
        return "stop", 0, "INVALID DIST", distance_state, 0.0

    if distance_state == "ARM_BLIND_READY":
        if abs(offset_x_m) <= ARM_OFFSET_TURN_THRESHOLD_M:
            return "handoff", 0, "IN REACH -> HANDOFF TO ARM", distance_state, 0.0

    angle_error = math.atan2(offset_x_m, dist_arm_m) if dist_arm_m > 0 else 0.0
    desired_wz = clamp(ARM_KP_WZ * angle_error, -ARM_MAX_WZ, ARM_MAX_WZ)

    if abs(offset_x_m) <= ARM_OFFSET_FORWARD_DEADZONE_M:
        return "forward", choose_arm_forward_speed(distance_state), "ARM CENTER: FORWARD", distance_state, desired_wz

    turn_speed = speed_from_wz(desired_wz, ARM_MIN_TURN_SPEED, ARM_MAX_SPEED)
    if offset_x_m < 0:
        return "turn_left", turn_speed, "ARM OFFSET LEFT: TURN LEFT", distance_state, desired_wz
    return "turn_right", turn_speed, "ARM OFFSET RIGHT: TURN RIGHT", distance_state, desired_wz


def action_to_vxyz(action, speed) -> Tuple[float, float, float]:
    """Map a string nav action + speed number to set_car_motion (vx, vy, vz).

    Uses the same KX/KZ calibration the original speed numbers were derived from.
    vy (mecanum strafe) is left at 0 for now — turning is done with yaw (vz),
    matching the original motor-server behaviour.
    """
    spd = float(speed)
    vx = spd * KX
    vz = spd * KZ
    if action in ("forward", "arm_forward"):
        return (vx, 0.0, 0.0)
    if action == "turn_left":
        return (0.0, 0.0, +vz)
    if action == "turn_right":
        return (0.0, 0.0, -vz)
    if action == "curve_left":
        return (vx, 0.0, +0.5 * vz)
    if action == "curve_right":
        return (vx, 0.0, -0.5 * vz)
    # stop / handoff / unknown
    return (0.0, 0.0, 0.0)


def select_largest_box(results):
    boxes = results.boxes
    if boxes is None or len(boxes) == 0:
        return None
    best_box, best_area = None, -1
    for box in boxes:
        x1, y1, x2, y2 = (int(box.xyxy[0][i]) for i in range(4))
        area = max(1, x2 - x1) * max(1, y2 - y1)
        if area > best_area:
            best_area, best_box = area, box
    return best_box


# ════════════════════════════════════════════════════════════════════════════
# Camera reader (lazy cv2)
# ════════════════════════════════════════════════════════════════════════════

class LatestFrameReader:
    def __init__(self, url, width=FRAME_W, height=FRAME_H):
        import cv2
        import threading
        self._cv2 = cv2
        self.url, self.width, self.height = url, width, height
        # A bare integer string ("0") means a local webcam index, not a URL —
        # lets the pipeline dry-run against a dev-machine camera.
        self.cap = cv2.VideoCapture(int(url) if isinstance(url, str) and url.isdigit() else url)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f"Unable to open camera source: {url}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.latest_frame = None
        self.latest_time = 0.0
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                frame = self._cv2.resize(frame, (self.width, self.height))
                with self.lock:
                    self.latest_frame = frame
                    self.latest_time = time.time()
            else:
                time.sleep(0.01)

    def read(self):
        with self.lock:
            if self.latest_frame is None:
                return False, None, 0.0
            return True, self.latest_frame.copy(), self.latest_time

    def release(self):
        self.running = False
        self.cap.release()
        self.thread.join(timeout=2.0)


# ════════════════════════════════════════════════════════════════════════════
# Navigator: dual-camera approach + grasp-target latch + verify, via set_car_motion
# ════════════════════════════════════════════════════════════════════════════

class Navigator:
    def __init__(self, device, model, *, show=False, dry_run=False,
                 rear_url=URL_REAR, arm_url=URL_ARM,
                 class_z_m: Optional[Dict[str, float]] = None,
                 class_height_m: Optional[Dict[str, float]] = None,
                 cam_to_base_x: float = CAM_TO_BASE_X,
                 cam_to_base_y: float = CAM_TO_BASE_Y,
                 sign_y: float = SIGN_Y):
        mapping = (float(cam_to_base_x), float(cam_to_base_y), float(sign_y))
        if not all(math.isfinite(v) for v in mapping):
            raise ValueError("camera-to-base mapping values must be finite")
        if sign_y not in (-1.0, 1.0):
            raise ValueError("camera lateral sign must be +1 or -1")
        self.device = device          # Rosmaster handle (None in dry-run)
        self.model = model            # ultralytics YOLO
        self.show = show
        self.dry_run = dry_run
        self.rear_url, self.arm_url = rear_url, arm_url
        self.class_z_m = class_z_m or {"_fallback": OBJ_Z_FIXED}
        # Per-class object HEIGHT (full z extent). Empty is legitimate: the grasp side
        # then applies its own symmetric-object estimate. Deliberately has no
        # "_fallback" -- a height guessed for an unrecognised class is worse than no
        # height, because the grasp side cannot tell a guess from a measurement.
        self.class_height_m = dict(class_height_m or {})
        self.cam_to_base_x = float(cam_to_base_x)
        self.cam_to_base_y = float(cam_to_base_y)
        self.sign_y = float(sign_y)
        self._rear = None
        self._arm = None
        # last ARM detection captured at hand-off (for latch)
        self._last_arm = None         # dict: dist, offset, box_w_px, class_name, depth
        self._last_arm_hit = None     # arm_cam_geometry.GroundHit from the last detect
        self._last_arm_height = None  # declared height used for that hit, if any
        # Set by every _detect_arm call. True means a fresh frame was processed
        # and either yielded a usable target or a trustworthy no-target result.
        self._arm_observation_valid = False

    # ── camera lifecycle ──
    def open_cameras(self):
        if self._rear is None:
            self._rear = LatestFrameReader(self.rear_url)
        if self._arm is None:
            self._arm = LatestFrameReader(self.arm_url)

    def release(self):
        for r in (self._rear, self._arm):
            if r is not None:
                try:
                    r.release()
                except Exception as e:
                    print(f"[nav][WARN] camera cleanup failed: {e}")
        self._rear = self._arm = None

    # ── chassis ──
    def _drive(self, action, speed):
        vx, vy, vz = action_to_vxyz(action, speed)
        if self.dry_run or self.device is None:
            print(f"[nav][dry] {action:>11} spd={speed:>3} -> set_car_motion({vx:+.3f},{vy:+.3f},{vz:+.3f})")
            return
        self.device.set_car_motion(vx, vy, vz)

    def stop(self):
        if self.dry_run or self.device is None:
            print("[nav][dry] STOP -> set_car_motion(0,0,0)")
            return
        self.device.set_car_motion(0.0, 0.0, 0.0)

    def back_off(self, t=RETREAT_TIME_S):
        """Short reverse to give room before a retry."""
        speed = speed_from_vx(RETREAT_VX_MPS)
        t0 = time.time()
        while time.time() - t0 < t:
            if self.dry_run or self.device is None:
                print(f"[nav][dry] back_off -> set_car_motion({-RETREAT_VX_MPS:+.3f},0,0)")
                break
            self.device.set_car_motion(-RETREAT_VX_MPS, 0.0, 0.0)
            time.sleep(0.05)
        self.stop()

    # ── detection helper ──
    def _detect_arm(self):
        """Return (found, dist_arm_m, offset_x_m, box_w_px, class_name)."""
        self._arm_observation_valid = False
        ok, frame, ts = self._arm.read()
        if not ok or (time.time() - ts) > MAX_FRAME_AGE:
            return False, -1.0, 0.0, 0.0, None
        res = self.model.predict(source=frame, conf=YOLO_CONF, imgsz=IMG_SIZE, verbose=False)[0]
        box = select_largest_box(res)
        if box is None:
            self._arm_observation_valid = True
            return False, -1.0, 0.0, 0.0, None
        if not getattr(self, "_arm_frame_size_checked", False):
            self._arm_frame_size_checked = True
            problem = acg.frame_size_mismatch(frame.shape[1], frame.shape[0])
            if problem:
                raise SystemExit("[pipeline] REFUSED: " + problem)
        class_name = class_name_for_box(self.model, box)
        x1, y1, x2, y2 = (int(box.xyxy[0][i]) for i in range(4))
        clipped = acg.bbox_touches_border(x1, y1, x2, y2)
        if clipped:
            print(f"[nav] arm detection dropped: {clipped}")
            return False, -1.0, 0.0, 0.0, None
        cx_box = (x1 + x2) / 2.0
        box_w_px = max(1, x2 - x1)
        # Locate the object the same way vision_grasp_bridge does. With a declared
        # height that is the silhouette centre against the object's half-height
        # plane; the bbox bottom-centre against the floor is off by -11 to -45 mm
        # at the near-vertical C3 pose, because the lowest pixel of the silhouette
        # is often the object's TOP face (arm_cam_geometry.silhouette_centre_target).
        # Mode A and mode B disagreeing about this is how the 2026-08-01 pose guard
        # ended up in one and not the other.
        obj_h = self.class_height_m.get(class_name)
        try:
            if obj_h is not None:
                hit = acg.silhouette_centre_target(x1, y1, x2, y2, ARM_CAM_POSE, obj_h)
            else:
                hit = acg.ground_hit_from_raw(cx_box, y2, ARM_CAM_POSE)
        except acg.GroundGeometryError as e:
            print(f"[nav] arm detection dropped: {e}")
            return False, -1.0, 0.0, 0.0, None
        self._last_arm_hit = hit
        self._last_arm_height = obj_h
        self._arm_observation_valid = True
        return True, hit.ground_dist_m, hit.lateral_m, box_w_px, class_name

    def _detect_rear(self):
        ok, frame, ts = self._rear.read()
        if not ok or (time.time() - ts) > MAX_FRAME_AGE:
            return False, -1.0, 0.0
        res = self.model.predict(source=frame, conf=YOLO_CONF, imgsz=IMG_SIZE, verbose=False)[0]
        box = select_largest_box(res)
        if box is None:
            return False, -1.0, 0.0
        x1, y1, x2, y2 = (int(box.xyxy[0][i]) for i in range(4))
        cx_box = (x1 + x2) / 2.0
        dist_rear = estimate_ground_distance(y2, THETA_REAR, H_REAR, FY_REAR, CY_REAR)
        dist_front = rear_dist_to_front_dist(dist_rear)
        offset = estimate_offset_x(cx_box, dist_rear, FX_REAR, CX_REAR)
        return True, dist_front, offset

    # ── main approach state machine ──
    def approach(self) -> bool:
        """Drive to the hand-off point. Returns True when ready to grasp."""
        self.open_cameras()
        state = "REAR_FOLLOW"
        last_seen = time.time()
        started_at = last_seen
        print(f"[nav] approach start (state={state})")

        while True:
            now = time.time()
            if now - started_at > APPROACH_MAX_DURATION_S:
                self.stop()
                print("[nav] approach timeout -> stopping")
                return False

            if state == "REAR_FOLLOW":
                found, dist_front, offset = self._detect_rear()
                if found:
                    last_seen = now
                    action, speed, status, dstate = decide_rear_action_by_offset(True, offset, dist_front)
                    print(f"[nav] REAR dist_front={dist_front:.2f} off={offset:+.2f} {action} ({status})")
                    if action == "stop" and dstate == "BLIND_READY":
                        self.stop()
                        _, blind_t = compute_rear_blind_plan(dist_front)
                        state = "REAR_BLIND"
                        print(f"[nav] -> REAR_BLIND for {blind_t:.2f}s")
                        self._rear_blind(blind_t)
                        state = "ARM_ALIGN"
                        # ARM target acquisition gets its own loss window; do not
                        # count the intentional blind-transfer time as target loss.
                        last_seen = time.time()
                        print("[nav] -> ARM_ALIGN")
                    else:
                        self._drive(action, speed)
                else:
                    self.stop()
                    if now - last_seen > TARGET_LOST_TIMEOUT_S:
                        print("[nav] target lost (REAR) — give up approach")
                        return False
                time.sleep(REAR_DECISION_INTERVAL)

            elif state == "ARM_ALIGN":
                found, dist_arm, offset, box_w, class_name = self._detect_arm()
                if found:
                    last_seen = now
                    action, speed, status, dstate, _wz = decide_arm_action_by_distance(True, offset, dist_arm)
                    print(f"[nav] ARM dist={dist_arm:.2f} off={offset:+.2f} {action} ({status})")
                    if action == "handoff":
                        self.stop()
                        self._last_arm = {
                            "dist": dist_arm,
                            "offset": offset,
                            "box_w_px": box_w,
                            "class_name": class_name,
                            "depth": (self._last_arm_hit.depth_m
                                      if self._last_arm_hit is not None else None),
                            "hit": self._last_arm_hit,
                            "obj_h": self._last_arm_height,
                        }
                        print(f"[nav] HANDOFF: class={class_name} dist={dist_arm:.3f} "
                              f"off={offset:+.3f} box_w={box_w}px")
                        return True
                    self._drive(action, speed)
                else:
                    self.stop()
                    if now - last_seen > TARGET_LOST_TIMEOUT_S:
                        print("[nav] target lost (ARM) — give up approach")
                        return False
                time.sleep(ARM_DECISION_INTERVAL)

            if self.show:
                self._maybe_show()

    def _rear_blind(self, blind_time_s):
        t0 = time.time()
        while time.time() - t0 < blind_time_s:
            self._drive("forward", speed_from_vx(REAR_BLIND_VX_MPS))
            time.sleep(0.05)
            if self.dry_run or self.device is None:
                break
        self.stop()

    def _maybe_show(self):
        try:
            import cv2
            ok, frame, _ = self._arm.read()
            if ok:
                cv2.imshow("pipeline arm cam", frame)
                cv2.waitKey(1)
        except Exception:
            pass

    # ── grasp target latch + verify ──
    def latch_arm_object(self) -> Tuple[list, Optional[float], Optional[float]]:
        """Map the hand-off arm detection to a grasp-frame target + width + height.

        Returns (pos, width_m, height_m). height_m is None unless --class-height
        declared one for this class; the grasp side then falls back to its own
        documented symmetric-object estimate rather than being handed a guess.

        Height cannot be derived from the bbox: the arm camera looks down at a fixed
        pitch, so bbox pixel height mixes the object's height with its depth. Width
        can, because it is measured across the image plane at a known distance.
        """
        if self._last_arm is None:
            # fall back to a sane straight-ahead target
            return [0.25 + self.cam_to_base_x, self.cam_to_base_y, OBJ_Z_FIXED], None, None
        d = self._last_arm["dist"]
        off = self._last_arm["offset"]
        class_name = self._last_arm.get("class_name")
        obj_x = d + self.cam_to_base_x
        obj_y = self.sign_y * off + self.cam_to_base_y
        obj_z = self.class_z_m.get(class_name, self.class_z_m.get("_fallback", OBJ_Z_FIXED))
        obj_h = self.class_height_m.get(class_name)
        # Metric width scales with the OPTICAL DEPTH, not the ground distance (using
        # d under-reports by cos(theta+alpha) -- 20% at the nav home, ~100% at C3),
        # and it then needs correcting for the object's height, because the widest
        # part of the silhouette is the TOP face and that is magnified more.
        hit = self._last_arm.get("hit")
        obj_h_seen = self._last_arm.get("obj_h")
        if hit is not None and obj_h_seen:
            width_m = acg.silhouette_width_to_object_width(
                self._last_arm["box_w_px"], hit, ARM_CAM_POSE, obj_h_seen)
        else:
            depth = self._last_arm.get("depth")
            if depth is None:
                depth = abs(d)   # only via a hand-built _last_arm (see --dump-frame)
            width_m = self._last_arm["box_w_px"] * depth / FX_ARM
        pos = [round(obj_x, 4), round(obj_y, 4), round(obj_z, 4)]
        print(f"[nav] latched grasp target class={class_name} pos={pos} "
              f"width={width_m:.3f}m height="
              + (f"{obj_h:.3f}m" if obj_h is not None else "None (grasp side estimates)"))
        return pos, round(width_m, 4), (round(obj_h, 4) if obj_h is not None else None)

    def verify_grasp(self, latched_pos) -> Optional[bool]:
        """Re-detect from the arm cam. Object still at the same spot ⇒ failed.

        Returns True if valid observations show the object gone/moved, False if
        it remains at the latched position, and None when the camera/detection
        geometry did not produce enough usable observations to decide.
        """
        self.open_cameras()
        time.sleep(0.3)
        hits = 0
        usable = 0
        for _ in range(5):
            # No `dist > 0` filter here. Ground distance is measured from the
            # camera's own ground projection and is legitimately negative for
            # anything behind it -- 64% of the image at the C3 pose. Filtering on
            # it would report "object gone" for a still-present object, i.e. turn a
            # failed grasp into a reported success. _detect_arm() already returns
            # found=False when the geometry is genuinely unusable.
            self._arm_observation_valid = False
            found, dist, offset, _w, _class_name = self._detect_arm()
            if getattr(self, "_arm_observation_valid", False):
                usable += 1
            if found:
                obj_x = dist + self.cam_to_base_x
                obj_y = self.sign_y * offset + self.cam_to_base_y
                if (abs(obj_x - latched_pos[0]) <= VERIFY_TOL_M
                        and abs(obj_y - latched_pos[1]) <= VERIFY_TOL_M):
                    hits += 1
            time.sleep(0.1)
        if hits >= 2:
            print(f"[verify] object still at spot ({hits}/5) -> grasp FAILED")
            return False
        if usable < 3:
            print(f"[verify] only {usable}/5 usable observations -> grasp UNKNOWN")
            return None
        print(f"[verify] object gone ({hits}/5 near spot, {usable}/5 usable) -> grasp OK")
        return True


# ════════════════════════════════════════════════════════════════════════════
# Orchestrator
# ════════════════════════════════════════════════════════════════════════════

def _load_grasp_module():
    """Import GraspController/DeployConfig from ../grasp/v21 (added to sys.path).

    2026-07-31: repointed from ../grasp (v17: absolute arm actions, arm_link5 TCP)
    to ../grasp/v21 (v21: incremental actions, gripper_center TCP, contact-based
    grasp confirmation instead of manual width sizing) -- v21 is the stack with
    hardware evidence (first_real_grasp_logged in grasp/v21/manifest.json); v17
    never had a confirmed physical grasp. v21's own sys.path handling (for
    Rosmaster_Lib in the shared grasp/ directory) resolves relative to its own
    __file__, so it works correctly regardless of which script imports it here.
    """
    grasp_dir = Path(__file__).resolve().parent.parent / "grasp" / "v21"
    sys.path.insert(0, str(grasp_dir))
    import x3plus_real_grasp as g
    return g


def run_pipeline(args):
    if args.real:
        raise SystemExit(
            "real integrated grasp is disabled until the final alignment/latch "
            "uses the calibrated grasp-home homography; the existing nav-home "
            "camera geometry can see farther than the arm can reach"
        )
    if args.real and not args.i_confirm_camera_frame:
        raise SystemExit("real grasp requires a confirmed camera->base mapping")
    g = _load_grasp_module()
    from ultralytics import YOLO

    model_path = (Path(__file__).resolve().parent.parent
                  / "detection" / "models" / "best.pt")
    print(f"[pipeline] loading YOLO: {model_path}")
    model = YOLO(str(model_path))

    cfg = g.DeployConfig(serial_port=args.port)
    # v21 has no manual width-based closure sizing (grip_width_control) -- its jaw
    # closes on CONTACT (detected by command-vs-encoder divergence during the close;
    # see grasp/v21/x3plus_real_grasp.py's attempt_close/_park_jaw_hold) and holds at
    # contact + a bias, regardless of the object's width. That is a strict
    # improvement over sizing the angle in advance from a vision-estimated width: it
    # needs no width estimate and adapts to whatever is actually between the
    # fingers. The width from nav.latch_arm_object() below is still used for the
    # too-wide-to-grasp pre-check.
    if args.nav_home_deg is not None:
        cfg.home_deg = g.parse_deg6_csv(args.nav_home_deg)
    if args.grasp_home_deg is not None:
        cfg.grasp_home_deg = g.parse_deg6_csv(args.grasp_home_deg)
    class_z_m = parse_class_z(args.class_z, OBJ_Z_FIXED)
    if args.class_z:
        print(f"[pipeline] class centroid-z overrides: {class_z_m}")
    class_height_m = parse_class_height(args.class_height)
    if class_height_m:
        print(f"[pipeline] class heights (drive v21 wrist_z_offset): {class_height_m}")
    else:
        print("[pipeline] no --class-height given: the grasp side will estimate height "
              "as 2*(centroid_z-ground_z). Correct only for a symmetric object resting "
              "on the floor whose --class-z is its true centroid.")
    warn_unknown_classes(model, [("--class-z", class_z_m),
                                 ("--class-height", class_height_m)])
    # Detection happens at cfg.home_deg (controller.move_home() puts the arm there
    # before nav.approach()), so that is the pose the camera constants must match.
    check_arm_cam_pose(cfg.home_deg, real=args.real,
                       acknowledged=args.i_confirm_arm_cam_pose)
    if args.handoff_dist is not None:
        globals()["ARM_BLIND_START_DIST_M"] = args.handoff_dist
    print(f"[pipeline] camera->base mapping: x={args.cam_x:+.4f}m "
          f"y={args.cam_y:+.4f}m sign_y={args.sign_y:+.0f}")

    controller = g.GraspController(cfg, real_servo=args.real, use_socket=False)
    try:
        nav = Navigator(
            controller.servo.device,
            model,
            show=args.show,
            dry_run=not args.real,
            class_z_m=class_z_m,
            class_height_m=class_height_m,
            cam_to_base_x=args.cam_x,
            cam_to_base_y=args.cam_y,
            sign_y=args.sign_y,
        )
    except Exception:
        controller.close()
        raise

    try:
        success = False
        for attempt in range(1, args.max_retries + 1):
            print("\n" + "=" * 60)
            print(f"[pipeline] ATTEMPT {attempt}/{args.max_retries}")
            print("=" * 60)

            print(f"[pipeline] moving arm to navigation home {list(cfg.home_deg)}")
            # move_home() is guarded (FloorGuard) and blocks until the encoders
            # confirm arrival, unlike v17's fire-and-forget servo.move_to_home() --
            # so no fixed-duration sleep is needed after it.
            controller.move_home()

            if not nav.approach():
                print("[pipeline] no object reachable — stopping.")
                break

            obj_pos, width, height = nav.latch_arm_object()

            # Width gate: if the object is wider than the gripper can open, no
            # arm pose will ever grasp it — abort the whole run (retrying the
            # same too-wide object is pointless).
            if width is not None and width > MAX_GRASP_WIDTH_M:
                print(f"[pipeline] OBJECT TOO LARGE: width={width*100:.1f}cm > "
                      f"max {MAX_GRASP_WIDTH_M*100:.1f}cm — aborting (cannot grasp).")
                break

            # v21's obj_provider second element is the object's HEIGHT in metres --
            # NOT width. It drives wrist_z_offset and the engagement-depth geometry.
            # Comes from --class-height, which the operator measures; None when the
            # class was not declared, in which case v21 applies its own documented
            # symmetric-object estimate (2*(centroid_z-ground_z)) exactly as a bare
            # --obj-x/y/z run does. The bbox-derived `width` must never be passed
            # here: it is an unrelated image-plane quantity and would silently
            # corrupt the approach geometry.
            controller.obj_provider = (lambda p=obj_pos, h=height: (p, h))

            grasp_seq_ok = controller.run(max_steps=args.max_steps)

            if not grasp_seq_ok:
                print("[pipeline] grasp sequence did not complete (stage machine "
                      "never reached done) — counting as failure.")
            elif nav.verify_grasp(obj_pos):
                print(f"[pipeline] OK grasp succeeded on attempt {attempt}.")
                success = True
                break
            print("[pipeline] FAILED grasp — retreating and retrying.")
            controller.move_home()   # guarded; ensures gripper open for retry
            time.sleep(1.0)
            nav.back_off()

        if not success:
            print(f"\n[pipeline] gave up after {args.max_retries} attempt(s).")
    except KeyboardInterrupt:
        print("\n[pipeline] interrupted — emergency stop.")
        try:
            controller.servo.emergency_stop()
        except Exception:
            pass
    finally:
        try:
            nav.stop()
        except Exception as e:
            print(f"[pipeline][WARN] chassis stop failed: {e}")
        try:
            nav.release()
        finally:
            controller.close()


# ════════════════════════════════════════════════════════════════════════════
# Self-test (pure functions, no hardware / cameras / torch)
# ════════════════════════════════════════════════════════════════════════════

def run_selftest():
    print("== action_to_vxyz ==")
    for act, spd in [("forward", 30), ("turn_left", 30), ("turn_right", 40),
                     ("curve_left", 35), ("curve_right", 35), ("handoff", 0), ("stop", 0)]:
        print(f"  {act:>11} spd={spd:>3} -> {tuple(round(v,3) for v in action_to_vxyz(act, spd))}")

    print("\n== rear decision (offset=0.0) ==")
    for d in [2.0, 1.3, 0.95, 0.85]:
        print(f"  dist_front={d:.2f} -> {decide_rear_action_by_offset(True, 0.0, d)}")
    print("== rear decision (offset=±) at MID ==")
    for off in [-0.30, -0.15, 0.0, 0.15, 0.30]:
        print(f"  off={off:+.2f} -> {decide_rear_action_by_offset(True, off, 1.0)}")

    print("\n== arm decision ==")
    for d, off in [(0.60, 0.0), (0.40, 0.10), (0.30, -0.05), (0.22, 0.02), (0.22, 0.20)]:
        print(f"  dist={d:.2f} off={off:+.2f} -> {decide_arm_action_by_distance(True, off, d)}")

    print("\n== latch mapping: a real pixel through the active pose ==")
    # Driven from an actual image row rather than a made-up distance, so the
    # numbers below are the ones the arm would really be handed. The object is
    # placed 60 px right of centre, two thirds of the way down the frame.
    probe = acg.ground_hit_from_raw(acg.CX + 60.0, 320.0, ARM_CAM_POSE)
    d, off, bw = probe.ground_dist_m, probe.lateral_m, 90
    print(f"  pixel(u={acg.CX+60:.0f}, v=320) -> d={d:+.4f}m lateral={off:+.4f}m "
          f"depth={probe.depth_m:.4f}m")
    print(f"  pos=({probe.obj_x:.3f},{probe.obj_y:.3f},{OBJ_Z_FIXED}) "
          f"width={probe.metric_size(bw)*100:.2f}cm")
    print(f"  the same bbox scaled by ground distance instead of depth would read "
          f"{abs(bw * d / FX_ARM)*100:.2f}cm")

    print("\n== class-height plumbing (sugarbox: 6.5cm tall, centroid 3.25cm) ==")
    nav = Navigator(None, None, dry_run=True,
                    class_z_m={"sugarbox": 0.0325, "_fallback": OBJ_Z_FIXED},
                    class_height_m={"sugarbox": 0.065})
    for cname in ("sugarbox", "undeclared-class"):
        nav._last_arm = {"dist": d, "offset": off, "box_w_px": bw,
                         "class_name": cname, "depth": probe.depth_m}
        pos, w, h = nav.latch_arm_object()
        est = 2.0 * (pos[2] - 0.0)   # what the grasp side would assume if h is None
        print(f"  class={cname:18s} z={pos[2]:.4f} height="
              + (f"{h:.4f} (declared)" if h is not None
                 else f"None -> grasp side estimates {est:.4f}"))
    print("  NOTE: with --class-z 0.0325 the estimate happens to equal the declared")
    print("        0.065, because the box is symmetric and floor-resting. That")
    print("        coincidence is not a substitute for declaring the height: it")
    print("        breaks silently for any object that is not both.")

    print("\n== width gate ==")
    print(f"  MAX_GRASP_WIDTH_M = {MAX_GRASP_WIDTH_M:.3f}m "
          f"({MAX_GRASP_WIDTH_M*100:.1f}cm) — measure the object across the jaw's")
    print("  closing axis; anything wider aborts the run instead of retrying.")

    print("\n== rear blind plan ==")
    for d in [0.90, 1.20]:
        print(f"  dist_front={d:.2f} -> (blind_dist, blind_time)={tuple(round(x,3) for x in compute_rear_blind_plan(d))}")

    print("\n== arm-cam undistort round-trip + ground model ==")
    # Phase 2's measured points belong to the NAV HOME, so they are checked against
    # that pose explicitly rather than against whatever pose is currently active.
    # Pinning them to ARM_CAM_POSE would make this assertion "prove" the constants
    # right no matter which pose they came from -- exactly the confusion that let
    # nav-home extrinsics survive the move to C3.
    nav = acg.V17_NAV_HOME
    for d_true, u, v in [(0.25, 286, 428), (0.30, 280, 352), (0.40, 249, 220),
                         (0.50, 238, 126)]:
        uu, vv = undistort_pixel(u, v, FX_ARM, FY_ARM, CX_ARM, CY_ARM, DIST_ARM)
        ur, vr = distort_pixel(uu, vv, FX_ARM, FY_ARM, CX_ARM, CY_ARM, DIST_ARM)
        rt_err = max(abs(ur - u), abs(vr - v))
        assert rt_err < 0.05, f"undistort round-trip {rt_err:.3f}px at ({u},{v})"
        hit = acg.ground_hit(uu, vv, nav)
        d_err = abs(hit.ground_dist_m - d_true)
        assert d_err < 0.01, f"ground model err {d_err*100:.2f}cm at D={d_true}"
        print(f"  D={d_true:.2f} raw({u},{v}) -> undist({uu:6.1f},{vv:6.1f}) "
              f"est={hit.ground_dist_m:.4f}m (err {d_err*100:.2f}cm, rt {rt_err:.4f}px) "
              f"depth={hit.depth_m:.4f}m")
    # The depth/distance gap is the bug the old code had: it used the ground
    # distance as the projection depth for lateral offset and metric width.
    hit = acg.ground_hit(acg.CX, acg.CY, nav)
    ratio = hit.ground_dist_m / hit.depth_m
    print(f"  at the nav home, ground/depth = {ratio:.3f} -> the old lateral and "
          f"width were {100*(1-ratio):.0f}% low")

    print("\n== active arm-cam pose ==")
    print(f"  {ARM_CAM_POSE.describe()}")
    lo = acg.ground_hit(acg.CX, 0.0, ARM_CAM_POSE)
    hi = acg.ground_hit(acg.CX, 479.0, ARM_CAM_POSE)
    xs = sorted((lo.obj_x, hi.obj_x))
    print(f"  visible ground band along base X: {xs[0]:.3f} .. {xs[1]:.3f} m")
    print(f"  ground distance at the image centre: {acg.ground_hit(acg.CX, acg.CY, ARM_CAM_POSE).ground_dist_m:+.4f} m")
    if not ARM_CAM_POSE.fully_measured:
        print("  NOTE: these extrinsics are a URDF prediction, not a measurement.")
    print("\n[selftest] OK")


def parse_args():
    p = argparse.ArgumentParser(description="X3Plus unified self-driving grasp pipeline")
    p.add_argument("--real", action="store_true", help="drive real wheels + servos (default dry-run)")
    p.add_argument("--show", action="store_true", help="show arm camera window")
    p.add_argument("--max-retries", type=int, default=3, help="grasp attempts before giving up")
    p.add_argument("--max-steps", type=int, default=300, help="grasp policy steps per attempt")
    p.add_argument("--port", type=str, default="/dev/myserial", help="Rosmaster serial port")
    p.add_argument("--handoff-dist", type=float, default=None,
                   help="override hand-off distance (m); default ARM_BLIND_START_DIST_M")
    p.add_argument("--nav-home-deg", type=str, default=None,
                   help="Navigation/cruise home servo degrees as S1,...,S6")
    p.add_argument("--grasp-home-deg", type=str, default=None,
                   help="PPO grasp initial servo degrees as S1,...,S6")
    p.add_argument("--class-z", action="append", default=[],
                   help="YOLO class CENTROID-Z override, NAME=CENTROID_Z_M. "
                        "Repeatable; unlisted classes use OBJ_Z_FIXED. Not the height.")
    p.add_argument("--class-height", action="append", default=[],
                   help="YOLO class object HEIGHT (full z extent), NAME=HEIGHT_M. "
                        "Repeatable. Drives the grasp side's wrist_z_offset and "
                        "engagement gates. Cannot be measured from the bbox (the arm "
                        "camera looks down at a fixed pitch, so bbox pixel height mixes "
                        "height with depth) -- measure the object and state it. "
                        "Undeclared classes send no height and the grasp side falls "
                        "back to 2*(centroid_z-ground_z).")
    p.add_argument("--cam-x", type=float, default=CAM_TO_BASE_X,
                   help="calibrated camera-origin to PPO base_link X offset (m)")
    p.add_argument("--cam-y", type=float, default=CAM_TO_BASE_Y,
                   help="calibrated camera-origin to PPO base_link Y offset (m)")
    p.add_argument("--sign-y", type=float, choices=(-1.0, 1.0), default=SIGN_Y,
                   help="camera-right to PPO base_link Y sign")
    p.add_argument("--i-confirm-arm-cam-pose", action="store_true",
                   help="Acknowledge that H_ARM/THETA_ARM were re-measured at the pose "
                        "detection actually runs from (cfg.home_deg). Without this, "
                        "--real refuses when that pose differs from "
                        f"{list(ARM_CAM_CALIBRATED_AT_HOME_DEG)}, the pose they were "
                        "calibrated at on 2026-07-08.")
    p.add_argument("--i-confirm-camera-frame", action="store_true",
                   help="confirm Phase-3 camera->base mapping was measured on the real robot")
    p.add_argument("--selftest", action="store_true", help="run pure-logic self-test and exit")
    return p.parse_args()


def main():
    args = parse_args()
    if args.selftest:
        run_selftest()
        return
    if args.max_retries <= 0:
        raise SystemExit("--max-retries must be > 0")
    if args.max_steps <= 0:
        raise SystemExit("--max-steps must be > 0")
    if args.handoff_dist is not None and (
            not math.isfinite(args.handoff_dist) or args.handoff_dist <= 0.0):
        raise SystemExit("--handoff-dist must be finite and > 0")
    if not all(math.isfinite(v) for v in (args.cam_x, args.cam_y, args.sign_y)):
        raise SystemExit("--cam-x/--cam-y/--sign-y must be finite")
    if args.real and not args.i_confirm_camera_frame:
        raise SystemExit(
            "real grasp refused: Phase-3 camera->PPO base_link mapping is not "
            "proven; pass calibrated --cam-x/--cam-y/--sign-y and "
            "--i-confirm-camera-frame"
        )
    run_pipeline(args)


if __name__ == "__main__":
    main()
