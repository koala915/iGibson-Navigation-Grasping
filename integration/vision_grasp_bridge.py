#!/usr/bin/env python3
"""Vision → Grasp bridge for X3Plus.

Runs YOLOv11 (Ultralytics) on the arm-camera stream, turns the detected
bounding box into an object position + width in the arm/grasp coordinate
frame, and pushes it to the grasp controller over TCP.

Pipeline:
    arm camera ──YOLO──▶ bbox ──geometry──▶ {x, y, z, w, height}(m) ──TCP 5555──▶
        x3plus_real_grasp.py  (run with --real --socket ...)

Intrinsics, distortion and pose stamps live in integration/arm_cam_geometry.py.
The measured legacy nav-home projection remains available for diagnostics. At
the v21 C3 grasp pose, runtime detections require a measured pixel-to-base
homography; nav-home H/theta values are deliberately rejected there.

Works against BOTH grasp stacks — the payload is a superset:

    v21 (current, grasp/v21/x3plus_real_grasp.py)
        python3 grasp/v21/x3plus_real_grasp.py --real --socket \
          --latch-obj --i-confirm-external-frame --unlock-candidate-real \
          --model models/candidate_v21_seed816_ckpt550000.zip \
          --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
          --contract obs_28_incremental
        reads "height" (full z extent), ignores "w".

    v17 (fallback, grasp/x3plus_real_grasp.py)
        python3 grasp/x3plus_real_grasp.py --real --socket \
          --width-grip --latch-obj --i-confirm-external-frame
        reads "w" (width), ignores "height".

── The pose problem, and how the two processes solve it together ──────────────

The arm camera is bolted to arm_link4 and moves with the arm, so this bridge's
extrinsics are valid at exactly ONE arm pose. This process cannot read the
servos (the grasp process owns the serial port), and the grasp process has no
idea how the coordinates it receives were computed. Neither can catch a
mismatch alone.

So every payload carries a "cam_pose" stamp naming the arm pose its geometry
assumes, and the grasp side compares that stamp against its own encoders at
latch time — the one moment both facts exist in the same place. A disagreement
aborts instead of grasping confidently in the wrong spot.

Both sides also require --latch-obj under --real: latching freezes one detection
taken at the home pose for the whole episode. Detections this bridge stops
sending go stale on the grasp side after
DeployConfig.detection_stale_timeout_sec and are withdrawn rather than served as
current.

The grasp side (class DetectionReceiver) accepts one JSON object per TCP
connection and reads until EOF, so this bridge opens a fresh connection for
every message and closes it right after sending.

Calibration status: see arm_cam_geometry.POSES. A predicted v21 C3 pose is not
enough to generate grasp coordinates: collect bottom-centre pixel samples with
``--calibration-only`` and supply the resulting verified calibration with
``--homography``. ``--i-accept-predicted-extrinsics`` applies only to the legacy
nav-home mode and cannot bypass the grasp-home homography gate.

Examples:
    # collect stable undistorted pixels for grasp-home homography calibration
    python vision_grasp_bridge.py --dry-run --calibration-only --show --stream 0

    # continuous grasp-home bridge using a measured calibration
    python vision_grasp_bridge.py --host 192.168.1.11 \
        --homography grasp_home_homography.json \
        --class-height sugarbox=0.065
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_cam_geometry as acg  # noqa: E402
import mission_status           # noqa: E402  (stdlib only)

# cv2 and ultralytics are imported inside main(), not here, so the pure functions
# below (geometry, payload construction, the z/height reconciliation) can be
# imported and tested on a machine with neither installed. The same reason
# vision_grasp_pipeline.py defers its heavy imports.

# ── Default model (relative to this script: integration/ → detection/models/) ──
DEFAULT_MODEL = str(Path(__file__).resolve().parent.parent / "detection" / "models" / "best.pt")

DEFAULT_STREAM = "http://127.0.0.1:8080/stream?topic=/arm_cam/image_raw"
IMG_SIZE = 640
CAMERA_REOPEN_FAILURES = 10
CAMERA_REOPEN_DELAY_SEC = 0.5
CAMERA_OPEN_ATTEMPTS = 3
CAMERA_PROBE_READS = 3
CAMERA_PROBE_DELAY_SEC = 0.05

# Legacy measured nav-home geometry.  Grasp-home never uses these values; it
# requires a measured homography instead.
H = 0.332
FIXED_THETA = 36.40

# Phase-3 camera ground coordinates -> PPO/URDF base_link mapping.
CAM_TO_BASE_X = 0.1639
CAM_TO_BASE_Y = 0.0331
SIGN_X = 1.0
SIGN_Y = -1.0


def resolve_camera_geometry(args) -> None:
    """Resolve the selected pose mapping and fail closed at grasp home.

    Navigation home retains the measured pinhole-ground model.  Grasp home is
    mapped directly by a calibrated pixel-to-base homography, avoiding unsafe
    assumptions about its very different height, pitch and forward direction.
    Calibration-only mode never opens the TCP connection and therefore may
    collect pixels before a homography exists.
    """
    # All four together, before any branch can return early. The hulls used to
    # be set only on the load-succeeded path, so --calibration-only (which
    # returns above without a homography) left them undefined; nothing reads
    # them in that mode today, but that safety was a property of the control
    # flow in the frame loop rather than of this function. Initialising here
    # makes an unmapped read a clear None instead of an AttributeError.
    args.homography_matrix = None
    args.homography_document = None
    args.homography_pixel_hull = None
    args.homography_base_hull = None
    if args.calibration_only and args.camera_pose != "grasp-home":
        raise SystemExit("--calibration-only is only supported at --camera-pose grasp-home")
    if args.camera_pose == "grasp-home":
        if args.calibration_only:
            return
        if not args.homography:
            raise SystemExit(
                "grasp-home detection requires --homography; nav-home H/theta/"
                "cam offsets are invalid after the arm moves"
            )
        try:
            try:
                from .grasp_home_homography import load_calibration
            except ImportError:
                from grasp_home_homography import load_calibration
            document = load_calibration(
                args.homography, min_points=6, max_error_m=0.02
            )
        except Exception as exc:
            raise SystemExit(f"invalid grasp-home homography: {exc}") from exc
        args.homography_document = document
        args.homography_matrix = document["homography"]
        args.homography_pixel_hull = convex_hull(
            [(row["u"], row["v"]) for row in document["points"]]
        )
        args.homography_base_hull = convex_hull(
            [(row["x"], row["y"]) for row in document["points"]]
        )
        return

    defaults = (H, FIXED_THETA, CAM_TO_BASE_X, CAM_TO_BASE_Y, SIGN_X, SIGN_Y)
    values = (
        args.camera_height, args.camera_theta, args.cam_x, args.cam_y,
        args.sign_x, args.sign_y,
    )
    resolved = [default if value is None else value
                for value, default in zip(values, defaults)]
    (args.camera_height, args.camera_theta, args.cam_x, args.cam_y,
     args.sign_x, args.sign_y) = (float(v) for v in resolved)
    numeric = (args.camera_height, args.camera_theta, args.cam_x, args.cam_y)
    if not all(math.isfinite(v) for v in numeric):
        raise SystemExit("camera geometry values must be finite")
    if args.camera_height <= 0.0:
        raise SystemExit("--camera-height must be > 0")
    if not 0.0 < args.camera_theta < 90.0:
        raise SystemExit("--camera-theta must be between 0 and 90 degrees")
    if args.sign_x not in (-1.0, 1.0) or args.sign_y not in (-1.0, 1.0):
        raise SystemExit("--sign-x and --sign-y must be +1 or -1")


def convex_hull(points):
    """Return a counter-clockwise convex hull (monotonic chain)."""
    unique = sorted({(float(x), float(y)) for x, y in points})
    if len(unique) <= 1:
        return unique

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def point_in_convex_hull(point, hull, tolerance: float = 1e-7) -> bool:
    """True when a point is inside/on a counter-clockwise convex hull."""
    if len(hull) < 3:
        return False
    px, py = float(point[0]), float(point[1])
    for index, start in enumerate(hull):
        end = hull[(index + 1) % len(hull)]
        cross = ((end[0] - start[0]) * (py - start[1])
                 - (end[1] - start[1]) * (px - start[0]))
        if cross < -tolerance:
            return False
    return True


def apply_grasp_home_mapping(args, u: float, v: float):
    """Map one undistorted pixel and reject homography extrapolation."""
    if (args.homography_matrix is None or args.homography_pixel_hull is None
            or args.homography_base_hull is None):
        # Reachable only if a future caller maps pixels without a loaded
        # calibration. Say so, rather than dying inside the hull test on None.
        raise ValueError(
            "grasp-home mapping requires a loaded homography; "
            "resolve_camera_geometry did not supply one"
        )
    pixel = (float(u), float(v))
    if not point_in_convex_hull(pixel, args.homography_pixel_hull):
        raise ValueError(
            f"undistorted pixel ({u:.1f},{v:.1f}) is outside calibration hull"
        )
    try:
        try:
            from .grasp_home_homography import apply_homography
        except ImportError:
            from grasp_home_homography import apply_homography
        mapped = apply_homography(args.homography_matrix, pixel)
    except Exception as exc:
        raise ValueError(f"homography mapping failed: {exc}") from exc
    base_xy = (float(mapped[0]), float(mapped[1]))
    if not point_in_convex_hull(base_xy, args.homography_base_hull):
        raise ValueError(
            f"mapped base point ({base_xy[0]:.4f},{base_xy[1]:.4f}) is "
            "outside calibrated base hull"
        )
    return base_xy


def apply_grasp_home_width_endpoint(args, u: float, v: float,
                                    center_xy: Tuple[float, float],
                                    max_distance_m: float):
    """Map one bbox side for width estimation, never as a target position.

    The calibration hull is made from object centres.  A graspable object's
    silhouette may extend a little outside that hull while its bottom-centre is
    safely interpolated.  Keep the target centre on the strict path above and
    bound this width-only extrapolation by the maximum jaw opening.
    """
    if args.homography_matrix is None:
        raise ValueError("grasp-home width mapping requires a loaded homography")
    pixel = (float(u), float(v))
    try:
        try:
            from .grasp_home_homography import apply_homography
        except ImportError:
            from grasp_home_homography import apply_homography
        mapped = apply_homography(args.homography_matrix, pixel)
    except Exception as exc:
        raise ValueError(f"homography width mapping failed: {exc}") from exc
    endpoint_xy = (float(mapped[0]), float(mapped[1]))
    if not all(math.isfinite(value) for value in endpoint_xy):
        raise ValueError(f"non-finite mapped bbox endpoint {endpoint_xy}")
    distance_m = math.hypot(endpoint_xy[0] - center_xy[0],
                            endpoint_xy[1] - center_xy[1])
    if distance_m > max_distance_m:
        raise ValueError(
            f"bbox side maps {distance_m*100:.1f} cm from its centre, past the "
            f"{max_distance_m*100:.1f} cm jaw-width bound"
        )
    return endpoint_xy


def median_calibration_record(records: List[dict]) -> dict:
    """Collapse one stable detection batch to median undistorted pixels."""
    if not records:
        raise ValueError("calibration record batch is empty")

    def med(key: str) -> float:
        return round(float(statistics.median(row[key] for row in records)), 3)

    return {
        "camera_pose": "grasp-home",
        "samples": len(records),
        "class": records[0]["class"],
        "u": med("u"),
        "v": med("v"),
        "left_u": med("left_u"),
        "left_v": med("left_v"),
        "right_u": med("right_u"),
        "right_v": med("right_v"),
        "raw_u": med("raw_u"),
        "raw_v": med("raw_v"),
    }


def calibration_record_from_bbox(
        box_xyxy: Tuple[float, float, float, float], class_name: str):
    """Return one safe calibration sample, or a reason to skip the frame."""
    x1, y1, x2, y2 = box_xyxy
    clipped = acg.bbox_touches_border(x1, y1, x2, y2)
    if clipped:
        return None, clipped
    raw_u = (x1 + x2) / 2.0
    try:
        u, v = acg.undistort_pixel(raw_u, y2)
        left_u, left_v = acg.undistort_pixel(x1, y2)
        right_u, right_v = acg.undistort_pixel(x2, y2)
    except (ValueError, OverflowError) as exc:
        return None, f"unusable calibration geometry: {exc}"
    values = (u, v, left_u, left_v, right_u, right_v)
    if not all(math.isfinite(value) for value in values):
        return None, "unusable calibration geometry: non-finite pixel"
    return {
        "class": class_name, "u": u, "v": v,
        "left_u": left_u, "left_v": left_v,
        "right_u": right_u, "right_v": right_v,
        "raw_u": raw_u, "raw_v": y2,
    }, ""

# An object wider than the jaw can open is physically ungraspable. The gripper's
# measured pad separation is 72.3 mm fully open (S6=30 deg, URDF FK on the finger
# pad links), so anything past 60 mm leaves no approach clearance.
MAX_GRASP_WIDTH_M = 0.06

# The region the v21 policy was actually evaluated in: x(0.20, 0.33), y(+-0.10),
# 97/100 on a held-out seed. The arm camera at C3 sees ground from about x=0.185
# to x=0.318, so it OVERHANGS this at both ends -- a detection can be perfectly
# healthy and still name a spot the policy has never been measured on. Dropping
# those here means the operator sees it while looking at the camera window,
# rather than after the arm has driven to its home pose and refused.
# Kept in sync with DeployConfig.trained_x_range / trained_y_range.
#
# These are v21's numbers and stay the DEFAULT, because every existing caller is
# a v21 caller. v23 is trained on a different, narrower band -- x(0.205, 0.280)
# y(-0.070, +0.065) -- from a pose that sees 44% MORE ground, so at E1 the
# overhang is worse in both directions. A v23 caller passes --policy-x-range /
# --policy-y-range; grasp/v23/jetson_one_command_grasp.py does exactly that.
#
# Getting this wrong is not dangerous, only confusing: the grasp side runs the
# same envelope check itself (DeployConfig.trained_*_range) and refuses. The
# point of the copy here is WHERE the operator finds out.
TRAINED_X_RANGE = (0.20, 0.33)
TRAINED_Y_RANGE = (-0.10, 0.10)
MAX_GRASP_FORWARD_OFFSET_MM = 15.0

# --class-z and --class-height describe the same object. For a symmetric object
# resting on the floor, height = 2 * centroid_z. Declaring both with a bigger
# disagreement than this is a contradiction the operator has to resolve, not
# something to average over.
Z_HEIGHT_CONSISTENCY_TOL_M = 0.005


def parse_class_z(values: List[str], fallback_z: float) -> Dict[str, float]:
    """Parse repeated --class-z entries like sugarbox=0.0325.

    The value is the object's CENTROID z in the grasp frame, not its height. The
    two are only related by height = 2*z for a symmetric object resting on the
    floor; see --class-height for stating the height directly.
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
    fallback_z = float(fallback_z)
    if not math.isfinite(fallback_z) or fallback_z < 0.0:
        raise ValueError(f"--obj-z must be finite and non-negative, got {fallback_z}")
    out.setdefault("_fallback", fallback_z)
    return out


def parse_class_height(values: List[str]) -> Dict[str, float]:
    """Parse repeated --class-height entries like sugarbox=0.065.

    The value is the object's FULL Z EXTENT (top minus floor) in metres — what the
    v21 grasp side calls ``height`` and uses to compute wrist_z_offset. It cannot be
    derived from the bbox: the arm camera looks down at an angle, so bbox pixel
    height mixes the object's height with its depth. Stating it per class is the
    honest option; omitting it lets the grasp side apply its own documented
    symmetric-object estimate instead of trusting a made-up number.
    """
    out: Dict[str, float] = {}
    for raw in values:
        if "=" not in raw:
            raise ValueError(f"--class-height must be NAME=HEIGHT_M, got: {raw}")
        name, value = raw.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"--class-height has an empty class name: {raw}")
        height = float(value)
        if not math.isfinite(height) or height <= 0.0:
            raise ValueError(f"--class-height must be finite and positive: {raw}")
        out[name] = height
    return out


def reconcile_z_and_height(class_z: Dict[str, float],
                           class_height: Dict[str, float]) -> Dict[str, float]:
    """Make the declared centroid-z agree with the declared height.

    Two failure modes this closes, both silent before:

    * A class with --class-height but no --class-z used to fall back to --obj-z
      (default 0.02). Declaring sugarbox=0.065 then shipped z=0.02 with
      height=0.065, and the grasp side computed wrist_z_offset off a centroid
      1.25 cm below where the declared height puts it. Here the height decides:
      z = height/2, the same resting-symmetric-object assumption the grasp side
      documents for the reverse direction.
    * Declaring both, inconsistently, is a contradiction — one of the two numbers
      is wrong and no averaging fixes that. Raise and make the operator pick.

    An explicit --class-z that AGREES with the height is kept as given, so an
    asymmetric object can still be described exactly.
    """
    out = dict(class_z)
    for name, height in class_height.items():
        implied = height / 2.0
        if name in class_z:
            if abs(class_z[name] - implied) > Z_HEIGHT_CONSISTENCY_TOL_M:
                raise ValueError(
                    f"--class-z {name}={class_z[name]} contradicts --class-height "
                    f"{name}={height}: a symmetric object resting on the floor has "
                    f"centroid_z = height/2 = {implied:.4f} m, which is "
                    f"{abs(class_z[name] - implied)*100:.2f} cm away. Fix whichever "
                    "of the two is wrong; the grasp side uses BOTH (height for "
                    "wrist_z_offset, z as the target centroid) and cannot reconcile "
                    "them for you.")
        else:
            out[name] = implied
            print(f"[bridge] {name}: --class-height {height} m implies centroid "
                  f"z={implied:.4f} m (symmetric, resting on the floor)")
    return out


def class_name_for_box(model, box) -> str:
    cls_id = int(box.cls[0].item())
    names = model.names
    if isinstance(names, dict):
        return str(names.get(cls_id, cls_id))
    if 0 <= cls_id < len(names):
        return str(names[cls_id])
    return str(cls_id)


def open_capture(stream: str):
    """Open a cv2 capture. A bare integer string is treated as a webcam index."""
    import cv2
    if stream.isdigit():
        return cv2.VideoCapture(int(stream))
    return cv2.VideoCapture(stream)


def open_capture_checked(stream: str, attempts: int = CAMERA_OPEN_ATTEMPTS,
                         probe_reads: int = CAMERA_PROBE_READS,
                         retry_delay: float = CAMERA_REOPEN_DELAY_SEC,
                         probe_delay: float = CAMERA_PROBE_DELAY_SEC):
    """Open a capture and prove it by reading a real frame.

    ``VideoCapture.isOpened()`` alone is not sufficient for hot-unplugged or
    busy V4L2 devices: some backends briefly report open and then every read
    fails.  A capture is returned only after one non-empty frame.  Failed
    handles are always released and retry count is finite.

    Returns ``(capture, first_frame, error_text)``.  On failure the first two
    values are ``None`` and ``error_text`` explains the last failed attempt.
    """
    attempts = max(1, int(attempts))
    probe_reads = max(1, int(probe_reads))
    last_error = "unknown camera error"

    for attempt in range(1, attempts + 1):
        cap = None
        success = False
        try:
            if stream.isdigit() and Path("/dev").is_dir():
                device_path = Path("/dev") / f"video{int(stream)}"
                if not device_path.exists():
                    last_error = f"{device_path} does not exist"
                    print(f"[bridge] camera open attempt {attempt}/{attempts} "
                          f"failed: {last_error}")
                    if attempt < attempts and retry_delay > 0.0:
                        time.sleep(retry_delay)
                    continue

            cap = open_capture(stream)
            if cap is None or not cap.isOpened():
                last_error = "backend could not open the source"
            else:
                for _ in range(probe_reads):
                    ok, frame = cap.read()
                    if ok and frame is not None and getattr(frame, "size", 0) > 0:
                        success = True
                        return cap, frame, ""
                    if probe_delay > 0.0:
                        time.sleep(probe_delay)
                last_error = (
                    f"source opened but produced no frame in {probe_reads} probe reads"
                )
        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"
        finally:
            # Python executes finally even on return; keep only a capture that
            # has already produced a non-empty frame.
            if cap is not None and not success:
                try:
                    cap.release()
                except Exception:
                    pass

        print(f"[bridge] camera open attempt {attempt}/{attempts} "
              f"failed: {last_error}")
        if attempt < attempts and retry_delay > 0.0:
            time.sleep(retry_delay)

    return None, None, last_error


def send_detection(host: str, port: int, payload: dict, timeout: float = 2.0) -> bool:
    """Open a fresh TCP connection, send one JSON payload, close. True on success."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.sendall(json.dumps(payload).encode("utf-8"))
        return True
    except OSError as e:
        print(f"[bridge] send failed → {host}:{port}: {e}")
        return False


def pick_best_box(result):
    """Return the highest-confidence box, or None."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None
    confs = boxes.conf.tolist()
    best_i = max(range(len(confs)), key=lambda i: confs[i])
    return boxes[best_i]


def build_payload(box_xyxy: Tuple[float, float, float, float],
                  class_name: str,
                  pose: acg.ArmCamPose,
                  class_z: Dict[str, float],
                  class_height: Dict[str, float],
                  max_width_m: float = MAX_GRASP_WIDTH_M) -> Tuple[Optional[dict], str]:
    """Turn one bbox into a stamped detection payload.

    Returns (payload, note). payload is None when the detection is unusable, and
    note always explains what happened — a dropped detection with no reason
    printed is how "the bridge sees nothing" turns into a half-hour of staring at
    a working camera.
    """
    x1, y1, x2, y2 = box_xyxy
    clipped = acg.bbox_touches_border(x1, y1, x2, y2)
    if clipped:
        return None, clipped
    cx_box = (x1 + x2) / 2.0
    box_w_px = max(1.0, x2 - x1)

    obj_z = class_z.get(class_name, class_z["_fallback"])
    obj_h = class_height.get(class_name)

    # The declared height is not just cargo for the grasp side: it is what lets us
    # locate the object properly. With it, the ray through the SILHOUETTE CENTRE
    # meets the object's half-height plane at its centre (~4 mm across the C3
    # workspace). Without it, all that is left is the bbox bottom-centre against
    # the floor, which at this near-vertical pose is off by -11 to -45 mm because
    # the lowest pixel is often the object's TOP face. See
    # arm_cam_geometry.silhouette_centre_target.
    try:
        if obj_h is not None:
            hit = acg.silhouette_centre_target(x1, y1, x2, y2, pose, obj_h)
            method = "centroid"
        else:
            hit = acg.ground_hit_from_raw(cx_box, y2, pose)
            method = "bottom-edge"
    except acg.GroundGeometryError as e:
        return None, f"unusable geometry: {e}"

    if obj_h is not None:
        width_m = acg.silhouette_width_to_object_width(box_w_px, hit, pose, obj_h)
    else:
        width_m = hit.metric_size(box_w_px)

    values = (hit.obj_x, hit.obj_y, obj_z, width_m)
    if not all(math.isfinite(v) for v in values) or width_m < 0.0:
        return None, f"rejected non-finite geometry {values}"
    if width_m > max_width_m:
        return None, (f"object is {width_m*100:.1f} cm wide, past the "
                      f"{max_width_m*100:.1f} cm the jaw can open — ungraspable")
    if not TRAINED_X_RANGE[0] <= hit.obj_x <= TRAINED_X_RANGE[1]:
        return None, (f"x={hit.obj_x:.3f} is outside the policy's evaluated range "
                      f"{TRAINED_X_RANGE[0]}-{TRAINED_X_RANGE[1]} m — the camera can "
                      f"see further than the arm was trained to reach; move the "
                      f"object or the robot")
    if not TRAINED_Y_RANGE[0] <= hit.obj_y <= TRAINED_Y_RANGE[1]:
        return None, (f"y={hit.obj_y:+.3f} is outside the policy's evaluated range "
                      f"{TRAINED_Y_RANGE[0]}..{TRAINED_Y_RANGE[1]} m")

    # Superset payload: v17 reads "w" and ignores "height"; v21 reads "height"
    # and ignores "w". One bridge serves both, and neither side has to guess
    # which one is listening.
    payload = {
        "x": round(hit.obj_x, 4),
        "y": round(hit.obj_y, 4),
        "z": round(obj_z, 4),
        "w": round(width_m, 4),
        "class": class_name,
    }
    if obj_h is not None:
        payload["height"] = round(obj_h, 4)
    acg.stamp_payload(payload, pose)
    note = (f"[{method}] d={hit.ground_dist_m:+.4f} Z={hit.depth_m:.4f} "
            f"total={hit.total_angle_deg:.1f}deg w={width_m*100:.1f}cm")
    return payload, note


def policy_ranges(args) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """The (x, y) band this run will accept, per --policy-x-range/--policy-y-range.

    Falls back to the module constants, so a caller that does not pass them keeps
    exactly the v21 behaviour this file has always had.
    """
    x_range = tuple(getattr(args, "policy_x_range", None) or TRAINED_X_RANGE)
    y_range = tuple(getattr(args, "policy_y_range", None) or TRAINED_Y_RANGE)
    return x_range, y_range


def apply_grasp_forward_offset(
        xy: Tuple[float, float], offset_mm: float) -> Tuple[float, float]:
    """Translate a homography result in base +X without changing its width.

    This is an operational correction for a repeatable real-arm landing bias,
    not part of the measured camera calibration. The translated centre still
    passes through the receiving policy's envelope check.
    """
    value = float(offset_mm)
    if (not math.isfinite(value)
            or not 0.0 <= value <= MAX_GRASP_FORWARD_OFFSET_MM):
        raise ValueError(
            "grasp forward offset must be finite and in [0, {:.0f}] mm; got {!r}"
            .format(MAX_GRASP_FORWARD_OFFSET_MM, offset_mm))
    return float(xy[0]) + value / 1000.0, float(xy[1])


def build_homography_payload(args,
                             box_xyxy: Tuple[float, float, float, float],
                             class_name: str, pose: acg.ArmCamPose,
                             class_z: Dict[str, float],
                             class_height: Dict[str, float],
                             max_width_m: float = MAX_GRASP_WIDTH_M,
                             base_xy_transform=None):
    """Build a grasp-home payload from a measured pixel-to-base map.

    ``base_xy_transform`` is reserved for a rigid base-frame transform applied
    equally to the centre and both width endpoints after the strict reference
    homography hull checks.  The v23 S1-only scanner uses it to rotate E1-mapped
    points about arm_joint1; ordinary bridge callers leave it as ``None``.
    """
    x1, y1, x2, y2 = box_xyxy
    border_hits = acg.bbox_border_hits(x1, y1, x2, y2)
    accepted_top_clip = (
        border_hits == ("top",)
        and bool(getattr(args, "allow_top_clipped_grasp_home", False))
    )
    if border_hits and not accepted_top_clip:
        return None, acg.bbox_touches_border(x1, y1, x2, y2)
    raw_u = (x1 + x2) / 2.0
    try:
        u, v = acg.undistort_pixel(raw_u, y2)
        left_u, left_v = acg.undistort_pixel(x1, y2)
        right_u, right_v = acg.undistort_pixel(x2, y2)
        # The target centre remains strictly inside both calibration hulls.
        # Left/right are silhouette points used only to estimate width; the
        # calibration hull itself contains object centres, not bbox sides.
        obj_x, obj_y = apply_grasp_home_mapping(args, u, v)
        center_xy = (obj_x, obj_y)
        left_xy = apply_grasp_home_width_endpoint(
            args, left_u, left_v, center_xy, max_width_m)
        right_xy = apply_grasp_home_width_endpoint(
            args, right_u, right_v, center_xy, max_width_m)
        pre_transform_width_m = math.hypot(right_xy[0] - left_xy[0],
                                           right_xy[1] - left_xy[1])
        if base_xy_transform is not None:
            center_xy = tuple(float(value) for value in
                              base_xy_transform(center_xy))
            left_xy = tuple(float(value) for value in
                            base_xy_transform(left_xy))
            right_xy = tuple(float(value) for value in
                             base_xy_transform(right_xy))
            obj_x, obj_y = center_xy
    except (TypeError, ValueError) as exc:
        return None, f"unusable homography geometry: {exc}"

    width_m = math.hypot(right_xy[0] - left_xy[0],
                         right_xy[1] - left_xy[1])
    if base_xy_transform is not None:
        # The transform is documented as rigid and the width is measured after
        # it, so the two must agree. Checking rather than trusting: a scale
        # factor slipping into the transform would rescale the width and the
        # bracketing test together, silently, and both would still look
        # perfectly reasonable. A pure rotation cannot change a distance.
        if abs(width_m - pre_transform_width_m) > 1e-6:
            return None, (
                "base_xy_transform is not rigid: width changed from "
                f"{pre_transform_width_m*100:.3f} cm to {width_m*100:.3f} cm. "
                "Only rotations and translations may be applied here.")
    left_delta = (left_xy[0] - obj_x, left_xy[1] - obj_y)
    right_delta = (right_xy[0] - obj_x, right_xy[1] - obj_y)
    if left_delta[0] * right_delta[0] + left_delta[1] * right_delta[1] > 1e-9:
        return None, "mapped bbox sides do not bracket the target centre"
    obj_z = class_z.get(class_name, class_z["_fallback"])
    obj_h = class_height.get(class_name)
    values = (obj_x, obj_y, obj_z, width_m)
    if not all(math.isfinite(value) for value in values):
        return None, f"rejected non-finite homography geometry {values}"
    if width_m > max_width_m:
        return None, (f"object is {width_m*100:.1f} cm wide, past the "
                      f"{max_width_m*100:.1f} cm the jaw can open — ungraspable")
    x_range, y_range = policy_ranges(args)
    if not x_range[0] <= obj_x <= x_range[1]:
        return None, (f"x={obj_x:.3f} is outside the policy's evaluated range "
                      f"{x_range[0]}-{x_range[1]} m")
    if not y_range[0] <= obj_y <= y_range[1]:
        return None, (f"y={obj_y:+.3f} is outside the policy's evaluated range "
                      f"{y_range[0]}..{y_range[1]} m")

    payload = {
        "x": round(obj_x, 4), "y": round(obj_y, 4),
        "z": round(obj_z, 4), "w": round(width_m, 4),
        "class": class_name,
    }
    if obj_h is not None:
        payload["height"] = round(obj_h, 4)
    acg.stamp_payload(payload, pose)
    clip_note = ("; accepted top-only clip (bottom/left/right visible, "
                 "measured-homography opt-in)" if accepted_top_clip else "")
    width_note = (
        "; bounded bbox-side width extrapolation"
        if (not point_in_convex_hull((left_u, left_v),
                                     args.homography_pixel_hull)
            or not point_in_convex_hull((right_u, right_v),
                                        args.homography_pixel_hull))
        else ""
    )
    transform_note = ("; rigid base-XY transform" if base_xy_transform is not None
                      else "")
    return payload, (f"[homography] uv=({u:.1f},{v:.1f}) "
                     f"w={width_m*100:.1f}cm{clip_note}{width_note}"
                     f"{transform_note}")


def parse_args():
    p = argparse.ArgumentParser(description="X3Plus vision → grasp TCP bridge")
    p.add_argument("--host", default="127.0.0.1", help="grasp controller host (default 127.0.0.1)")
    p.add_argument("--port", type=int, default=5555, help="grasp controller TCP port (default 5555)")
    p.add_argument("--model", default=DEFAULT_MODEL, help="YOLO .pt path (default detection/models/best.pt)")
    p.add_argument("--stream", default=DEFAULT_STREAM, help="camera URL or webcam index (e.g. 0)")
    p.add_argument("--conf", type=float, default=0.3, help="YOLO confidence threshold (default 0.3)")
    p.add_argument("--imgsz", type=int, default=IMG_SIZE, help="YOLO inference size (default 640)")
    p.add_argument("--rate", type=float, default=0.3, help="min seconds between sends (default 0.3)")
    p.add_argument("--status-udp", default=None, metavar="HOST:PORT",
                   help="publish detection telemetry to the operator console "
                        f"(e.g. {mission_status.DEFAULT_ENDPOINT}). Fire-and-forget.")
    p.add_argument("--once", action="store_true",
                   help="send/output the first valid detection batch then exit")
    p.add_argument("--wait-for-go", action="store_true",
                   help="load the model, then block on a line from stdin before "
                        "opening the camera. Lets a launcher start this process "
                        "alongside the arm controller so the ultralytics import "
                        "overlaps the controller's own startup instead of queueing "
                        "behind it, while the first frame is still taken only after "
                        "the arm is confirmed at the pose the geometry assumes. "
                        "EOF on stdin aborts rather than detecting at an unknown pose.")
    p.add_argument("--show", action="store_true", help="show annotated camera window")
    p.add_argument("--dry-run", action="store_true",
                   help="compute and print detections without sending. Use this to "
                        "read off numbers during Phase 3 calibration.")
    # ── which arm pose the camera geometry belongs to ──
    p.add_argument("--camera-pose", choices=("grasp-home", "nav-home"),
                   default="grasp-home",
                   help="arm pose used for detection (default grasp-home)")
    p.add_argument("--pose", default=None, choices=sorted(acg.POSES),
                   help="advanced: explicit arm-camera pose registry name")
    p.add_argument("--homography", default=None,
                   help="verified grasp-home pixel-to-base calibration JSON")
    p.add_argument("--allow-top-clipped-grasp-home", action="store_true",
                   help="with a measured grasp-home homography only, accept a bbox "
                        "that touches the TOP edge alone. Its bottom-centre and "
                        "left/right edges must remain visible and inside the "
                        "calibration hull. Left/right/bottom clipping is never allowed.")
    p.add_argument("--policy-x-range", type=float, nargs=2, default=None,
                   metavar=("LO", "HI"),
                   help="base-frame x band the receiving policy was trained on "
                        f"(default {TRAINED_X_RANGE[0]} {TRAINED_X_RANGE[1]}, which "
                        "is v21's). v23 is 0.205 0.280.")
    p.add_argument("--policy-y-range", type=float, nargs=2, default=None,
                   metavar=("LO", "HI"),
                   help="base-frame y band the receiving policy was trained on "
                        f"(default {TRAINED_Y_RANGE[0]} {TRAINED_Y_RANGE[1]}, which "
                        "is v21's). v23 is -0.070 0.065.")
    p.add_argument("--grasp-forward-offset-mm", type=float, default=0.0,
                   help="translate a grasp-home homography target in base +X "
                        "(forward/away from the robot) after calibration and "
                        "before the policy envelope check (default 0; allowed "
                        "0..15 mm). Width is unchanged.")
    p.add_argument("--calibration-only", action="store_true",
                   help="print median undistorted bbox pixels; never connect to TCP")
    p.add_argument("--calibration-samples", type=int, default=10,
                   help="valid frames per median calibration output (default 10)")
    p.add_argument("--camera-theta", "--cam-theta", dest="camera_theta",
                   type=float, default=None,
                   help="measured optical-axis depression (deg), towards base +X. "
                        "At the C3 pose this is near 90, NOT near 36.")
    p.add_argument("--camera-height", "--cam-h", dest="camera_height",
                   type=float, default=None,
                   help="measured camera height above the floor (m)")
    p.add_argument("--cam-x", type=float, default=None,
                   help="measured camera ground projection in the base frame, +X forward (m)")
    p.add_argument("--cam-y", type=float, default=None,
                   help="measured camera ground projection in the base frame, +Y left (m)")
    p.add_argument("--sign-y", type=float, default=None, choices=(-1.0, 1.0),
                   help="sign mapping image-right → grasp +Y (+1 or -1)")
    p.add_argument("--sign-x", type=float, default=None, choices=(-1.0, 1.0),
                   help="nav-home camera-forward to base +X sign")
    p.add_argument("--i-accept-predicted-extrinsics", action="store_true",
                   help="send detections even though the selected pose's extrinsics "
                        "are a URDF prediction rather than a hardware measurement. "
                        "The arm will move to whatever these numbers say.")
    # ── object description ──
    p.add_argument("--obj-z", type=float, default=0.02,
                   help="object CENTROID z in the grasp frame (m, default 0.02), for "
                        "classes with neither --class-z nor --class-height.")
    p.add_argument("--class-z", action="append", default=[],
                   help="YOLO class centroid-z override, NAME=CENTROID_Z_M. Repeatable. "
                        "Usually unnecessary: a class with --class-height gets "
                        "centroid_z = height/2 automatically.")
    p.add_argument("--class-height", action="append", default=[],
                   help="YOLO class object HEIGHT (full z extent), NAME=HEIGHT_M. "
                        "Repeatable. Read by the v21 grasp side to compute "
                        "wrist_z_offset; ignored by v17. Cannot be measured from the "
                        "bbox (the camera looks down at an angle), so state it per "
                        "class. Unlisted classes send no height and the grasp side "
                        "applies its own symmetric-object estimate.")
    p.add_argument("--max-width", type=float, default=MAX_GRASP_WIDTH_M,
                   help=f"refuse to send objects wider than this (m, default "
                        f"{MAX_GRASP_WIDTH_M})")
    return p.parse_args()


def resolve_pose(args) -> acg.ArmCamPose:
    """Select the extrinsic set and apply any measured overrides.

    ``--camera-pose`` names a KIND of pose; ``--pose`` names one row in the
    registry. nav-home has exactly one row, so the two are interchangeable there.
    grasp-home now has one row per deployed policy generation (C3 for v21, E1 for
    v23), so --pose is how a caller says which. The default stays C3: switching
    it would silently restamp every existing mode A/B/C detection with a pose the
    grasp side would then refuse against its encoders.
    """
    if args.camera_pose == "nav-home":
        allowed = {acg.V17_NAV_HOME.name}
        default_name = acg.V17_NAV_HOME.name
    else:
        allowed = set(acg.GRASP_HOME_POSE_NAMES)
        default_name = acg.DEFAULT_POSE.name
    if args.pose is not None and args.pose not in allowed:
        raise SystemExit(
            f"--camera-pose {args.camera_pose} conflicts with --pose {args.pose}; "
            f"expected one of {sorted(allowed)}"
        )
    pose_name = args.pose or default_name
    pose = acg.get_pose(pose_name)
    pose = pose.replace(theta_deg=args.camera_theta, h_m=args.camera_height,
                        cam_x_m=args.cam_x, cam_y_m=args.cam_y,
                        sign_y=args.sign_y)
    print(f"[bridge] arm-camera geometry → {pose.describe()}")
    print(f"[bridge]   source: {pose.source}")
    return pose


def main():
    import cv2
    from ultralytics import YOLO

    args = parse_args()
    resolve_camera_geometry(args)
    if not 0.0 <= args.conf <= 1.0:
        raise SystemExit("--conf must be between 0 and 1")
    if args.imgsz <= 0:
        raise SystemExit("--imgsz must be > 0")
    if not math.isfinite(args.rate) or args.rate < 0.0:
        raise SystemExit("--rate must be finite and >= 0")
    if not math.isfinite(args.max_width) or args.max_width <= 0.0:
        raise SystemExit("--max-width must be finite and positive")
    if args.calibration_samples <= 0:
        raise SystemExit("--calibration-samples must be > 0")
    try:
        apply_grasp_forward_offset((0.0, 0.0),
                                   args.grasp_forward_offset_mm)
    except ValueError as exc:
        raise SystemExit(str(exc))
    if args.calibration_only and args.grasp_forward_offset_mm != 0.0:
        raise SystemExit("--grasp-forward-offset-mm is runtime-only; calibration "
                         "must record the uncorrected measured geometry")
    if args.camera_pose != "grasp-home" and args.grasp_forward_offset_mm != 0.0:
        raise SystemExit("--grasp-forward-offset-mm requires --camera-pose grasp-home")
    if (args.allow_top_clipped_grasp_home
            and (args.camera_pose != "grasp-home" or args.calibration_only)):
        raise SystemExit("--allow-top-clipped-grasp-home is runtime-only and requires "
                         "--camera-pose grasp-home with a measured homography")

    pose = resolve_pose(args)
    # Predicted extrinsics are a fine starting point for --dry-run, but they must
    # not reach a real arm without the operator saying so.
    if (not args.dry_run and not args.calibration_only
            and args.camera_pose == "nav-home"):
        pose.require_measured(real=True,
                              acknowledged=args.i_accept_predicted_extrinsics)

    # --wait-for-go removes the race this warns about: the launcher releases the
    # gate only after the controller has confirmed the home pose, so the single
    # detection cannot be taken or sent early.
    if args.once and not args.wait_for_go:
        print("[bridge] NOTE: --once sends a single detection and exits. The grasp "
              "side latches only AFTER it has driven the arm to the home pose, "
              "which takes several seconds, and treats anything older than "
              "detection_stale_timeout_sec as no detection at all. Use --once for "
              "sanity checks; leave it off for a real grasp run.")

    target_transform = None
    if args.grasp_forward_offset_mm:
        offset_mm = args.grasp_forward_offset_mm
        target_transform = lambda xy: apply_grasp_forward_offset(xy, offset_mm)
        print(f"[bridge] grasp target correction: base +X {offset_mm:+.1f} mm "
              "after homography; policy envelope remains active")

    print(f"[bridge] loading YOLO model: {args.model}")
    model = YOLO(args.model)
    print(f"[bridge] classes: {model.names}")
    class_height = parse_class_height(args.class_height)
    class_z = reconcile_z_and_height(parse_class_z(args.class_z, args.obj_z),
                                     class_height)
    if args.class_z:
        print(f"[bridge] class centroid-z: {class_z}")
    if class_height:
        print(f"[bridge] class heights (for v21 wrist_z_offset): {class_height}")
    else:
        print("[bridge] no --class-height given: payloads omit 'height'. v17 does not "
              "use it; v21 will fall back to its symmetric-object estimate.")
        print("[bridge] WARN: without a declared height the object can only be located "
              "from the bbox bottom edge against the floor. At the C3 pose that is "
              "wrong by -11 to -45 mm, because the lowest pixel of a near-vertical "
              "view is often the object's TOP face. Declare --class-height.")
    known = set(model.names.values() if isinstance(model.names, dict) else model.names)
    for flag, table in (("--class-z", class_z), ("--class-height", class_height)):
        # A typo'd class name is silent otherwise: the lookup misses, the fallback is
        # used, and the run looks normal while grasping at the wrong height.
        for name in table:
            if name != "_fallback" and name not in known:
                print(f"[bridge] WARN: {flag} names {name!r}, which this model does not "
                      f"predict. Known classes: {sorted(known)}")

    if args.wait_for_go:
        # Everything above this line -- the ultralytics import, the YOLO weights, the
        # class tables -- is the slow part of starting up and none of it depends on
        # where the arm is. Waiting HERE lets a launcher start this process at the
        # same time as the arm controller, so this load overlaps the controller's
        # own torch/PPO load and its walk to the grasp home instead of queueing
        # behind them. Peak memory is unchanged: both processes are already resident
        # together by the time a detection is sent.
        #
        # The gate is deliberately BEFORE the camera opens, not after. open_capture
        # keeps the frame it probed with and the detect loop consumes it as its
        # first frame, and the camera driver buffers more behind that -- so a camera
        # opened early would hand the first detection an image taken while the arm
        # was still moving, measured against home-pose geometry. The arm-pose stamp
        # would not catch it either: that is read when the detection is sent, so it
        # would truthfully say "home" about a picture taken somewhere else.
        print("[bridge] loaded; waiting for the go signal on stdin before opening "
              "the camera", flush=True)
        line = sys.stdin.readline()
        if not line:
            # EOF = the launcher is gone. Opening the camera now would detect at an
            # unknown arm pose with nobody left to check the result.
            raise SystemExit("[bridge] ERROR: stdin closed before the go signal; "
                             "refusing to detect at an unverified arm pose.")
        print(f"[bridge] go signal received ({line.strip()!r}); opening the camera",
              flush=True)

    print(f"[bridge] opening camera: {args.stream}")
    cap, pending_frame, camera_error = open_capture_checked(args.stream)
    if cap is None:
        raise SystemExit(
            f"[bridge] ERROR: camera {args.stream!r} failed after "
            f"{CAMERA_OPEN_ATTEMPTS} attempts: {camera_error}. "
            "Check that the device exists and no other process owns it.")
    print(f"[bridge] camera ready: {args.stream} "
          f"frame_shape={tuple(pending_frame.shape)}")

    if args.calibration_only:
        print(f"[bridge] calibration-only: pose={args.camera_pose}, median of "
              f"{args.calibration_samples} valid frames; TCP disabled")
    elif args.dry_run:
        print("[bridge] DRY RUN: computing detections, sending nothing.")
    else:
        print(f"[bridge] streaming detections → {args.host}:{args.port} "
              f"(rate={args.rate}s, conf={args.conf})"
              f"{' [--once]' if args.once else ''}")

    last_send = 0.0
    last_status = 0.0
    last_note = ""
    infer_ema = None
    # Off unless --status-udp was given. The bridge reports in the mission FSM's
    # vocabulary so the console reads the same whichever mode is running.
    report = mission_status.SimpleReporter(getattr(args, "status_udp", None), mode="B")
    checked_frame_size = False
    consecutive_read_failures = 0
    calibration_records: List[dict] = []
    try:
        while True:
            if pending_frame is not None:
                ok, frame = True, pending_frame
                pending_frame = None
            else:
                ok, frame = cap.read()
            if not ok or frame is None:
                consecutive_read_failures += 1
                if consecutive_read_failures in (1, CAMERA_REOPEN_FAILURES):
                    print(f"[bridge] WARN: failed to read frame "
                          f"({consecutive_read_failures}/"
                          f"{CAMERA_REOPEN_FAILURES})")
                if consecutive_read_failures >= CAMERA_REOPEN_FAILURES:
                    print(f"[bridge] reopening camera after "
                          f"{consecutive_read_failures} consecutive read failures")
                    cap.release()
                    cap, pending_frame, camera_error = open_capture_checked(args.stream)
                    if cap is None:
                        raise SystemExit(
                            f"[bridge] ERROR: camera {args.stream!r} could not "
                            f"recover after {CAMERA_OPEN_ATTEMPTS} attempts: "
                            f"{camera_error}. Stopping instead of sending stale data."
                        )
                    consecutive_read_failures = 0
                    # A reopened index/URL can resolve to a different camera or
                    # resolution.  Re-run the calibration-size gate and never
                    # combine samples collected across a broken stream.
                    checked_frame_size = False
                    calibration_records.clear()
                    print(f"[bridge] camera reopened and frame verified: "
                          f"{args.stream} frame_shape={tuple(pending_frame.shape)}")
                else:
                    time.sleep(0.05)
                continue
            consecutive_read_failures = 0

            if not checked_frame_size:
                checked_frame_size = True
                problem = acg.frame_size_mismatch(frame.shape[1], frame.shape[0])
                if problem:
                    print(f"[bridge] FATAL: {problem}")
                    return
                print(f"[bridge] frame {frame.shape[1]}x{frame.shape[0]} matches the "
                      f"calibrated size")

            t_infer = time.monotonic()
            result = model.predict(source=frame, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
            dt_infer = time.monotonic() - t_infer
            infer_ema = dt_infer if infer_ema is None else 0.9 * infer_ema + 0.1 * dt_infer
            annotated = result.plot() if args.show else None

            box = pick_best_box(result)
            payload: Optional[dict] = None
            if box is not None:
                class_name = class_name_for_box(model, box)
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                if args.calibration_only:
                    record, note = calibration_record_from_bbox(
                        (x1, y1, x2, y2), class_name)
                    if record is not None and (calibration_records
                            and calibration_records[0]["class"] != class_name):
                        print("[bridge][calibration] class changed; resetting batch")
                        calibration_records.clear()
                    if record is not None:
                        calibration_records.append(record)
                        note = "collecting calibration samples"
                    if (record is not None
                            and len(calibration_records) >= args.calibration_samples):
                        record = median_calibration_record(calibration_records)
                        print("[bridge][calibration] "
                              + json.dumps(record, sort_keys=True))
                        calibration_records.clear()
                        if args.once:
                            break
                elif args.camera_pose == "grasp-home":
                    payload, note = build_homography_payload(
                        args, (x1, y1, x2, y2), class_name, pose,
                        class_z, class_height, args.max_width,
                        base_xy_transform=target_transform)
                else:
                    payload, note = build_payload(
                        (x1, y1, x2, y2), class_name, pose,
                        class_z, class_height, args.max_width)
                if payload is None and note != last_note:
                    if args.calibration_only and note != "collecting calibration samples":
                        print(f"[bridge][calibration] skipped {class_name}: {note}")
                    elif not args.calibration_only:
                        print(f"[bridge] dropped {class_name}: {note}")
                    last_note = note

                if args.show and annotated is not None:
                    cx_box = (x1 + x2) / 2.0
                    cv2.circle(annotated, (int(cx_box), int(y2)), 6, (0, 0, 255), -1)
                    label = (f"{class_name} x={payload['x']:.3f} y={payload['y']:.3f} "
                             f"z={payload['z']:.3f} w={payload['w']*100:.1f}cm"
                             if payload else f"{class_name} DROPPED")
                    cv2.putText(annotated, label, (int(x1), max(20, int(y1) - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 255) if payload else (0, 0, 255), 2)

            now = time.monotonic()
            if payload is not None and (now - last_send) >= args.rate:
                if args.dry_run:
                    last_send = now
                    print(f"[bridge][dry] would send {payload}   {note}")
                    report.say("ALIGN", "FINE_ALIGN", f"dry: {note or 'detection ready'}",
                               target={"visible": True, "streak": 3,
                                       "dist": payload.get("x"), "age": 0.0})
                    if args.once:
                        break
                elif send_detection(args.host, args.port, payload):
                    last_send = now
                    print(f"[bridge] sent {payload}   {note}")
                    # The bridge only ever locates; the grasp side decides. ALIGN
                    # is what it is really doing, and using the mission's own
                    # name keeps one vocabulary across all three modes.
                    report.say("ALIGN", "FINE_ALIGN", note or "detection sent",
                               target={"visible": True, "streak": 3,
                                       "dist": payload.get("x"), "age": 0.0})
                    if args.once:
                        break
            elif payload is None and report.enabled and (now - last_status) >= 1.0:
                last_status = now
                report.say("INVESTIGATE", "TURN_TO_TARGET", note or "no usable detection",
                           target={"visible": False, "streak": 0,
                                   "dist": None, "age": None})

            if args.show and annotated is not None:
                center = int(acg.CX)
                cv2.line(annotated, (center, 0), (center, annotated.shape[0]), (0, 255, 255), 1)
                cv2.putText(annotated, f"infer {infer_ema*1000:.0f} ms", (8, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.imshow("X3Plus vision -> grasp bridge", annotated)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n[bridge] interrupted.")
    finally:
        report.close()
        if cap is not None:
            cap.release()
        if args.show:
            cv2.destroyAllWindows()
        if infer_ema is not None:
            print(f"[bridge] mean YOLO inference {infer_ema*1000:.0f} ms/frame. "
                  "The grasp side discards detections older than its "
                  "detection_stale_timeout_sec (default 1.0 s), so keep "
                  "inference + --rate comfortably under that or raise it with "
                  "--stale-timeout on the grasp side.")


if __name__ == "__main__":
    main()
