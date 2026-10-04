"""Ground calibration and asset configuration with no ROS/model/hardware imports."""
import json
import math
import os

import cv2
import numpy as np


def resolve_asset_paths(repo_root, environment=None):
    """Repository defaults with deployment overrides and the legacy SAM alias."""
    environment = os.environ if environment is None else environment
    base_dir = environment.get("SUGARBOX_ASSET_DIR", os.path.join(repo_root, "detection"))
    return {
        "base_dir": base_dir,
        "yolo": environment.get("SUGARBOX_YOLO_MODEL", os.path.join(base_dir, "models", "best.pt")),
        "sam": environment.get("SUGARBOX_SAM_MODEL") or environment.get(
            "SUGARBOX_SAM2_MODEL", os.path.join(base_dir, "models", "sam2.1_b.pt")),
        "homography": environment.get("SUGARBOX_HOMOGRAPHY", os.path.join(base_dir, "rear_ground_homography.json")),
    }


def recursive_find_key(obj, keys, default=None):
    if isinstance(obj, dict):
        for key in keys:
            if key in obj:
                return obj[key]
        values = obj.values()
    elif isinstance(obj, list):
        values = obj
    else:
        return default
    for value in values:
        result = recursive_find_key(value, keys, default)
        if result is not default:
            return result
    return default


def _matrix_candidate(value):
    try:
        matrix = np.asarray(value, dtype=np.float64)
        if matrix.size == 9:
            matrix = matrix.reshape(3, 3)
        return matrix if matrix.shape == (3, 3) else None
    except (TypeError, ValueError, OverflowError):
        return None


def recursive_find_3x3(obj):
    if isinstance(obj, dict):
        for key in ("H", "homography", "homography_matrix", "H_pixel_to_ground",
                    "pixel_to_ground", "matrix"):
            if key in obj:
                matrix = _matrix_candidate(obj[key])
                if matrix is not None:
                    return matrix
        for value in obj.values():
            matrix = recursive_find_3x3(value)
            if matrix is not None:
                return matrix
    elif isinstance(obj, list):
        return _matrix_candidate(obj)
    return None


def find_calibration_pixels(obj):
    """Read measured pixels, excluding rejected samples from the fitted range.

    Current calibration files store u/v in samples. Legacy array formats remain
    supported, but malformed or non-finite accepted samples must fail closed.
    """
    missing = object()
    samples = recursive_find_key(obj, ["samples"], default=missing)
    if samples is not missing:
        if not isinstance(samples, list):
            return None
        points = []
        for sample in samples:
            if not isinstance(sample, dict):
                return None
            if sample.get("inlier", True) == False:
                continue
            try:
                point = [sample["u"], sample["v"]]
                # A non-finite measured ground point also invalidates calibration.
                values = point + [sample[key] for key in ("x", "y") if key in sample]
                if any(isinstance(value, bool) or not isinstance(
                        value, (int, float)) or not math.isfinite(value)
                       for value in values):
                    return None
            except (KeyError, TypeError, ValueError):
                return None
            points.append(point)
        value = points
    else:
        value = recursive_find_key(obj, [
            "image_points", "pixel_points", "src_points", "pixels", "pixel_xy"
        ])
        if value is None:
            return None

    try:
        pixels = np.asarray(value, dtype=np.float32).reshape(-1, 2)
    except (TypeError, ValueError, OverflowError):
        return None
    if len(pixels) < 3 or not np.isfinite(pixels).all():
        return None
    return pixels


def _validate_calibration_metadata(data):
    """Reject declared resolution or coordinate contracts for another camera.

    Legacy files without these declarations remain supported. A provided field
    must describe the fixed rear camera's 640x480 ground-plane calibration.
    """
    missing = object()
    for key, expected in (("frame_width", 640), ("frame_height", 480),
                          ("type", "rear_camera_ground_homography")):
        value = recursive_find_key(data, [key], default=missing)
        if value is not missing and value != expected:
            raise RuntimeError("calibration {} must be {!r}".format(key, expected))

    convention = recursive_find_key(data, ["coordinate_convention"], default=missing)
    if convention is missing:
        return
    if not isinstance(convention, dict):
        raise RuntimeError("calibration coordinate_convention must be an object")
    expected_convention = {
        "pixel_input": "bottom_point_u_v",
        "ground_output": "x_forward_y_left",
        "origin": "rear_camera_ground_projection",
        "x_axis": "forward_positive_meters",
        "y_axis": "left_positive_right_negative_meters",
    }
    for key, expected in expected_convention.items():
        if key in convention and convention[key] != expected:
            raise RuntimeError("calibration coordinate_convention.{} must be {!r}".format(
                key, expected))


def load_ground_calibration(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    _validate_calibration_metadata(data)
    H = recursive_find_3x3(data)
    if H is None or not np.isfinite(H).all() or np.linalg.matrix_rank(H) != 3:
        raise RuntimeError(
            "rear_ground_homography.json requires a finite, nonsingular 3x3 Homography matrix"
        )

    rear_cam_to_front_m = recursive_find_key(data, ["rear_cam_to_front_m"])
    if rear_cam_to_front_m is None:
        rear_cam_to_front_m = 0.20
    try:
        rear_cam_to_front_m = float(rear_cam_to_front_m)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeError("rear_cam_to_front_m must be finite") from exc
    if not math.isfinite(rear_cam_to_front_m):
        raise RuntimeError("rear_cam_to_front_m must be finite")

    calibration_pixels = find_calibration_pixels(data)
    if calibration_pixels is None:
        raise RuntimeError("calibration requires at least three finite accepted pixel samples")
    calibration_hull = cv2.convexHull(calibration_pixels.reshape(-1, 1, 2))
    if len(calibration_hull) < 3 or cv2.contourArea(calibration_hull) <= 1e-6:
        raise RuntimeError("calibration pixel hull must have nonzero area")

    return H, rear_cam_to_front_m, calibration_hull


def pixel_to_ground(u, v, H):
    p = np.array([float(u), float(v), 1.0], dtype=np.float64)
    q = H @ p
    if abs(q[2]) < 1e-9:
        return None
    q /= q[2]
    x, y = float(q[0]), float(q[1])
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def inside_calibration_hull(u, v, hull, margin_px=10.0):
    # Missing or degenerate calibration cannot authorize extrapolated targets.
    if hull is None:
        return False
    try:
        u, v = float(u), float(v)
        hull = np.asarray(hull, dtype=np.float32).reshape(-1, 1, 2)
        if (not math.isfinite(u) or not math.isfinite(v)
                or not np.isfinite(hull).all() or len(hull) < 3
                or cv2.contourArea(hull) <= 1e-6):
            return False
        distance = cv2.pointPolygonTest(hull, (u, v), True)
    except (TypeError, ValueError, OverflowError, cv2.error):
        return False
    return math.isfinite(distance) and distance >= -margin_px
