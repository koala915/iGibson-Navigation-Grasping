#!/usr/bin/env python3
"""Three calibrated arm-camera views, then a verified return to v23 E1.

This process owns the camera and Rosmaster serial port only for the search.  It
does not load PPO and never closes the jaw.  A successful result is printed only
after the guarded move back to E1 has been encoder-confirmed; the launcher then
waits for this process to exit (releasing both devices) before it starts the
ordinary v23 controller with a fixed base-frame target.

The three views differ only in S1 yaw.  They share the measured E1 homography in
the arm-fixed reference view, then rotate its base XY result about the measured
S1 axis.  This is valid only for yaw-only poses; the config loader rejects any
S2-S5 change.  Real use additionally requires hardware motion and held-out yaw
mapping validation for both side views.
"""

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace


if sys.version_info < (3, 8):
    sys.exit(
        "[FATAL] 需要 Python >= 3.8，目前是 {}。\n"
        "        先執行 source ~/grasp_venv/bin/activate".format(
            sys.version.split()[0]))

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
INTEGRATION = REPO_ROOT / "integration"
DEFAULT_CONFIG = HERE / "three_pose_scan.json"
DEFAULT_CAMERA = ("/dev/v4l/by-id/"
                  "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0")
DEFAULT_MODEL = REPO_ROOT / "detection" / "models" / "best.pt"
RESULT_PREFIX = "[scan][result] "
SCHEMA = "x3plus.v23.three_pose_scan"
E1_ARM_DEG = (90.0, 74.2, 8.6, 8.6, 90.0, 30.0)
EXPECTED_POSE_NAMES = ("LEFT", "E1", "RIGHT")
# URDF arm_joint1 origin plus deploy_contract.URDF_TO_TRAINING_FRAME XY.
# Verified with FK over S1=60..120 deg on 2026-08-30; max planar residual was
# 1.3e-8 m.  Hardware angle/backlash and axis verticality still need the config's
# separate held-out validation gate.
S1_PIVOT_XY_M = (0.118146, -0.003359)
S1_YAW_SIGN = -1.0

for _path in (str(HERE), str(INTEGRATION)):
    if _path not in sys.path:
        sys.path.insert(0, _path)


class ScanConfigError(ValueError):
    pass


def _finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ScanConfigError("{} must be numeric".format(label))
    if not math.isfinite(result):
        raise ScanConfigError("{} must be finite".format(label))
    return result


def _validate_arm_deg(raw, label):
    if not isinstance(raw, list) or len(raw) != 6:
        raise ScanConfigError("{} arm_deg must contain exactly 6 angles".format(label))
    values = tuple(_finite_float(value, "{} arm_deg".format(label)) for value in raw)
    limits = ((0.0, 180.0), (0.0, 180.0), (0.0, 180.0),
              (0.0, 180.0), (0.0, 270.0), (30.0, 30.0))
    for index, (value, bounds) in enumerate(zip(values, limits), start=1):
        if not bounds[0] <= value <= bounds[1]:
            raise ScanConfigError(
                "{} S{}={} is outside [{}, {}]".format(
                    label, index, value, bounds[0], bounds[1]))
    return values




def announce_unvalidated(config):
    """Say exactly what evidence is missing, every run, before anything moves."""
    missing = [pose for pose in config["poses"]
               if not (pose["hardware_validated"] and pose["yaw_mapping_validated"])]
    if not missing:
        return
    print("=" * 70)
    print("[UNLOCKED] --unlock-unvalidated-scan: running poses whose hardware")
    print("           evidence is INCOMPLETE. This is a supervised experiment.")
    for pose in missing:
        gaps = []
        if not pose["hardware_validated"]:
            gaps.append("motion not confirmed on hardware")
        if not pose["yaw_mapping_validated"]:
            gaps.append("yaw mapping not confirmed against a ruler")
        print("           {:6s} — {}".format(pose["name"], "; ".join(gaps)))
    print("           Every other gate still applies: the S1-only check, the")
    print("           pivot and yaw sign, the reference homography and its hull,")
    print("           sample spread, cross-pose agreement, and the")
    print("           encoder-confirmed E1 return before any target is released.")
    print("           Stay beside the robot with a hand on the power switch.")
    print("=" * 70)



def load_scan_config(path_str, require_homographies=True,
                     calibration_pose=None, unlock_unvalidated=False):
    """Load and validate exactly three distinct, hardware-visited poses.

    ``unlock_unvalidated`` waives ONLY the two per-pose evidence flags,
    hardware_validated and yaw_mapping_validated, for a supervised experiment.
    It is the same shape as --unlock-candidate-real on the controller: the
    evidence is still missing, the config still records that it is missing, and
    the operator is standing next to the robot having decided to try anyway.

    Everything else stays enforced -- three poses, S1-only, the pivot, the yaw
    sign, the +-25 degree limit, the reference homography and its >=6 point /
    2 cm gate, the sample spread, the cross-pose agreement, and the
    encoder-confirmed E1 return. Those are what stop a wrong number reaching the
    policy; these two flags only record whether a person has checked the pose on
    hardware yet."""
    path = Path(path_str).resolve()
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise ScanConfigError("scan config does not exist: {}".format(path))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScanConfigError("cannot read scan config {}: {}".format(path, exc))
    if document.get("schema") != SCHEMA or document.get("version") != 2:
        raise ScanConfigError("scan config must use schema {!r} version 2".format(SCHEMA))
    mapping_raw = document.get("mapping")
    if not isinstance(mapping_raw, dict):
        raise ScanConfigError("scan config mapping must be an object")
    if mapping_raw.get("type") != "s1_yaw_from_reference":
        raise ScanConfigError("mapping type must be s1_yaw_from_reference")
    reference_pose = str(mapping_raw.get("reference_pose", "")).strip()
    reference_s1_deg = _finite_float(
        mapping_raw.get("reference_s1_deg"), "mapping reference_s1_deg")
    pivot_raw = mapping_raw.get("pivot_xy_m")
    if not isinstance(pivot_raw, list) or len(pivot_raw) != 2:
        raise ScanConfigError("mapping pivot_xy_m must contain exactly two values")
    pivot_xy = tuple(_finite_float(v, "mapping pivot_xy_m") for v in pivot_raw)
    yaw_sign = _finite_float(mapping_raw.get("yaw_sign"), "mapping yaw_sign")
    if reference_pose != "E1" or abs(reference_s1_deg - E1_ARM_DEG[0]) > 1e-9:
        raise ScanConfigError("yaw reference must be E1 at S1=90 degrees")
    if any(abs(a - b) > 1e-9 for a, b in zip(pivot_xy, S1_PIVOT_XY_M)):
        raise ScanConfigError(
            "S1 pivot must be the verified training-frame value {}".format(
                list(S1_PIVOT_XY_M)))
    if yaw_sign != S1_YAW_SIGN:
        raise ScanConfigError("yaw_sign must be -1 for the URDF arm_joint1 -Z axis")
    homography_raw = str(mapping_raw.get("homography", "")).strip()
    if not homography_raw:
        raise ScanConfigError("mapping has no reference homography path")
    homography = Path(homography_raw)
    if not homography.is_absolute():
        homography = (path.parent / homography).resolve()
    if require_homographies and not homography.is_file():
        raise ScanConfigError(
            "reference E1 homography does not exist: {}".format(homography))
    poses = document.get("poses")
    if not isinstance(poses, list) or len(poses) != 3:
        raise ScanConfigError("scan config must contain exactly three poses")

    parsed = []
    names = set()
    arm_rows = set()
    for index, raw in enumerate(poses):
        if not isinstance(raw, dict):
            raise ScanConfigError("pose {} must be an object".format(index + 1))
        name = str(raw.get("name", "")).strip()
        if not name or name in names:
            raise ScanConfigError("scan pose names must be non-empty and unique")
        arm_deg = _validate_arm_deg(raw.get("arm_deg"), name)
        rounded = tuple(round(value, 4) for value in arm_deg)
        if rounded in arm_rows:
            raise ScanConfigError("scan poses must use three distinct arm positions")
        if (raw.get("hardware_validated") is not True
                and calibration_pose is None and not unlock_unvalidated):
            raise ScanConfigError(
                "pose {} is not hardware_validated; run --scan-calibrate-pose {} "
                "beside the robot first, or pass --unlock-unvalidated-scan to try "
                "it as a supervised experiment"
                .format(name, name))
        if (name != reference_pose
                and raw.get("yaw_mapping_validated") is not True
                and calibration_pose is None and not unlock_unvalidated):
            raise ScanConfigError(
                "pose {} yaw mapping is not hardware-validated; collect held-out "
                "points before runtime, or pass --unlock-unvalidated-scan to try "
                "it as a supervised experiment".format(name))
        parsed.append({
            "name": name,
            "arm_deg": arm_deg,
            "hardware_validated": raw.get("hardware_validated") is True,
            "yaw_mapping_validated": raw.get("yaw_mapping_validated") is True,
            "note": str(raw.get("note", "")),
        })
        names.add(name)
        arm_rows.add(rounded)

    if tuple(pose["name"] for pose in parsed) != EXPECTED_POSE_NAMES:
        raise ScanConfigError(
            "scan poses must be ordered LEFT, E1, RIGHT")
    reference = next((pose for pose in parsed if pose["name"] == reference_pose), None)
    if reference is None or tuple(reference["arm_deg"]) != E1_ARM_DEG:
        raise ScanConfigError("mapping reference pose must exactly equal v23 E1")
    for pose in parsed:
        if tuple(pose["arm_deg"][1:5]) != tuple(reference["arm_deg"][1:5]):
            raise ScanConfigError(
                "pose {} changes S2-S5; single-homography yaw mapping permits S1 only"
                .format(pose["name"]))
    left_delta = reference_s1_deg - parsed[0]["arm_deg"][0]
    right_delta = parsed[2]["arm_deg"][0] - reference_s1_deg
    if not (5.0 <= left_delta <= 25.0 and 5.0 <= right_delta <= 25.0):
        raise ScanConfigError(
            "LEFT/RIGHT S1 offsets must each be between 5 and 25 degrees")

    return_raw = document.get("return_pose")
    if not isinstance(return_raw, dict):
        raise ScanConfigError("return_pose must be an object")
    return_name = str(return_raw.get("name", "")).strip()
    return_arm = _validate_arm_deg(return_raw.get("arm_deg"), "return_pose")
    matching = [pose for pose in parsed if pose["name"] == return_name]
    if len(matching) != 1 or tuple(matching[0]["arm_deg"]) != tuple(return_arm):
        raise ScanConfigError("return_pose must exactly match one of the three poses")
    if return_name != "E1" or tuple(return_arm) != E1_ARM_DEG:
        raise ScanConfigError(
            "return_pose must be v23 E1 {} before PPO handoff".format(
                list(E1_ARM_DEG)))

    result = {
        "path": path,
        "poses": parsed,
        "return_pose": {"name": return_name, "arm_deg": return_arm},
        "mapping": {
            "type": "s1_yaw_from_reference",
            "reference_pose": reference_pose,
            "reference_s1_deg": reference_s1_deg,
            "pivot_xy_m": pivot_xy,
            "yaw_sign": yaw_sign,
            "homography": homography,
        },
    }
    for key, default, lo, hi in (
            ("samples_per_pose", 5, 1, 50),
            ("per_pose_timeout_sec", 8.0, 0.5, 60.0),
            ("within_pose_spread_m", 0.008, 0.001, 0.03),
            ("cross_pose_tolerance_m", 0.01, 0.001, 0.03)):
        value = document.get(key, default)
        if key == "samples_per_pose":
            if isinstance(value, bool) or int(value) != value:
                raise ScanConfigError("samples_per_pose must be an integer")
            value = int(value)
        else:
            value = _finite_float(value, key)
        if not lo <= value <= hi:
            raise ScanConfigError("{} must be in [{}, {}]".format(key, lo, hi))
        result[key] = value
    if calibration_pose is not None and calibration_pose not in names:
        raise ScanConfigError(
            "unknown calibration pose {!r}; expected one of {}".format(
                calibration_pose, sorted(names)))
    return result


def map_reference_xy_for_pose(reference_xy, pose_spec, mapping):
    """Rotate an E1-mapped ground point into base XY for one S1-only view."""
    x = _finite_float(reference_xy[0], "reference x")
    y = _finite_float(reference_xy[1], "reference y")
    pivot_x, pivot_y = mapping["pivot_xy_m"]
    delta_s1 = float(pose_spec["arm_deg"][0]) - mapping["reference_s1_deg"]
    angle = math.radians(mapping["yaw_sign"] * delta_s1)
    cosine, sine = math.cos(angle), math.sin(angle)
    dx, dy = x - pivot_x, y - pivot_y
    return (pivot_x + cosine * dx - sine * dy,
            pivot_y + sine * dx + cosine * dy)


def aggregate_pose_samples(samples, max_spread_m):
    """Median one view and reject jitter inconsistent with a stationary object."""
    if not samples:
        raise ScanConfigError("pose produced no valid detection samples")
    classes = {sample.get("class") for sample in samples}
    if len(classes) != 1 or None in classes:
        raise ScanConfigError("pose samples disagree on object class")
    xs = [float(sample["x"]) for sample in samples]
    ys = [float(sample["y"]) for sample in samples]
    if not all(math.isfinite(v) for v in xs + ys):
        raise ScanConfigError("pose samples contain non-finite coordinates")
    x = statistics.median(xs)
    y = statistics.median(ys)
    spread = max(math.hypot(px - x, py - y) for px, py in zip(xs, ys))
    if spread > max_spread_m:
        raise ScanConfigError(
            "pose detection spread {:.1f} cm exceeds {:.1f} cm".format(
                spread * 100.0, max_spread_m * 100.0))
    result = dict(samples[len(samples) // 2])
    for key in ("x", "y", "z", "w", "height"):
        values = [float(sample[key]) for sample in samples if sample.get(key) is not None]
        if values:
            result[key] = round(float(statistics.median(values)), 4)
    result["sample_count"] = len(samples)
    result["spread_m"] = round(spread, 5)
    return result


def fuse_pose_results(results, tolerance_m):
    """Fuse agreeing views; a disagreement is safer than picking one blindly."""
    if not results:
        raise ScanConfigError("all three scan poses produced no valid target")
    classes = {item.get("class") for item in results}
    if len(classes) != 1 or None in classes:
        raise ScanConfigError("scan poses disagree on object class")
    for left_index, left in enumerate(results):
        for right in results[left_index + 1:]:
            distance = math.hypot(float(left["x"]) - float(right["x"]),
                                  float(left["y"]) - float(right["y"]))
            if distance > tolerance_m:
                raise ScanConfigError(
                    "scan poses {} and {} disagree by {:.1f} cm (limit {:.1f} cm)"
                    .format(left["pose"], right["pose"], distance * 100.0,
                            tolerance_m * 100.0))
    fused = dict(results[0])
    for key in ("x", "y", "z", "w", "height"):
        values = [float(item[key]) for item in results if item.get(key) is not None]
        if values:
            fused[key] = round(float(statistics.median(values)), 4)
    fused["poses"] = [item["pose"] for item in results]
    fused["pose_count"] = len(results)
    fused.pop("cam_pose", None)
    fused.pop("cam_pose_name", None)
    return fused


def needs_reference_confirmation(results, poses, reference_s1_deg):
    """True when the only successful view is a ROTATED one.

    The cross-pose agreement check is the only thing that tests the S1 rotation
    at runtime. A wrong pivot or a flipped yaw_sign sends LEFT and RIGHT to
    opposite sides -- roughly 20 cm apart at this radius -- so with two views
    the 1 cm gate catches it immediately. With one view there is nothing left:
    the rotation produces a confident, wrong coordinate and every downstream
    check passes it, because a finite in-envelope number is exactly what they
    are looking for.

    Worse, this is not a rare corner. A single view means the object sat at the
    edge of the other views' windows, which is where the rotation arm is longest
    and a mapping error costs the most.

    The reference pose is exempt: at S1=90 the mapping IS the measured
    homography and no rotation is applied.
    """
    if len(results) != 1:
        return False
    by_name = {pose["name"]: pose for pose in poses}
    pose = by_name.get(results[0].get("pose"))
    if pose is None:
        # An unrecognised pose name cannot be shown to be unrotated, so treat it
        # as needing confirmation rather than assuming the safe case.
        return True
    return abs(float(pose["arm_deg"][0]) - float(reference_s1_deg)) > 1e-9


def finalize_scan_result(return_ok, interrupted, run_error, calibration_pose,
                         results, tolerance_m, confirmation_needed=False,
                         confirmation=None, accept_unconfirmed=False):
    """Gate the only scanner-to-controller handoff behind a confirmed E1 return.

    Keep this decision pure so the interruption and failed-return paths can be
    regression-tested without opening the camera or servo port.
    """
    if interrupted:
        return 130, None, "scan was interrupted"
    if run_error is not None:
        return 2, None, str(run_error)
    if not return_ok:
        return 2, None, "E1 return was not encoder-confirmed"
    if calibration_pose:
        return 0, None, "calibration completed after E1 return"
    if confirmation_needed:
        if confirmation is None:
            if not accept_unconfirmed:
                return 2, None, (
                    "only one rotated view saw the object and the reference "
                    "pose could not confirm it after the return; the target "
                    "would rest entirely on the S1 rotation being correct. "
                    "Re-run with the object further inside the shared field of "
                    "view, or pass --accept-single-rotated-view to take it "
                    "anyway")
            print("[scan] WARNING: single rotated view, unconfirmed at the "
                  "reference pose, accepted only because "
                  "--accept-single-rotated-view was given.")
        else:
            # Fold the reference detection in as an ordinary result. It carries
            # no rotation, so fuse_pose_results now has one rotated and one
            # unrotated measurement to disagree about -- which is the check that
            # a single view was missing.
            results = list(results) + [confirmation]
    try:
        fused = fuse_pose_results(results, tolerance_m)
    except ScanConfigError as exc:
        return 2, None, str(exc)
    return 0, fused, "target released after E1 return"


def _geometry_args(homography, policy_x_range, policy_y_range, allow_top):
    return SimpleNamespace(
        camera_pose="grasp-home", pose=None, calibration_only=False,
        homography=str(homography), camera_height=None, camera_theta=None,
        cam_x=None, cam_y=None, sign_x=None, sign_y=None,
        policy_x_range=policy_x_range, policy_y_range=policy_y_range,
        allow_top_clipped_grasp_home=bool(allow_top),
    )


def _dynamic_pose(acg, pose_spec, homography=None):
    """A pose carrying only what the grasp-home path reads: name and arm_deg.

    ``stamp_payload`` is the sole consumer here -- at a grasp home the mapping
    is the measured homography, and the trigonometric distance model in
    arm_cam_geometry is never used. The extrinsics below used to be plausible
    placeholders (theta 90, H 1.0 m, camera at the origin), which is the wrong
    kind of wrong: if any future path did reach ground_hit() with this pose it
    would get a confident, meaningless coordinate rather than an error.

    NaN instead. Every consumer of the trig model already checks for finite
    values and refuses, so a fabricated pose that leaks into it now fails
    closed. Nothing on the current path reads these fields at all.
    """
    poison = float("nan")
    return acg.ArmCamPose(
        name="v23_scan_{}".format(pose_spec["name"].lower()),
        arm_deg=tuple(pose_spec["arm_deg"]),
        theta_deg=poison, h_m=poison, cam_x_m=poison, cam_y_m=poison,
        sign_y=-1.0, distance_model_measured=False,
        base_offset_measured=False,
        # The parsed pose dicts carry name/arm_deg/flags/note and NOTHING else --
        # all three views share the single E1 file under config["mapping"], so
        # reading pose_spec["homography"] here raised KeyError on the first
        # scanned pose and took the whole run down. The calibration path never
        # calls this function, which is why only the runtime scan was broken.
        source="three-pose scan; geometry comes only from {}".format(
            homography if homography is not None
            else pose_spec.get("homography", "the shared reference homography")),
    )


def _grab_after_motion(cap, cv2, settle_s=0.7, flush_frames=10):
    time.sleep(settle_s)
    frame = None
    for _ in range(max(1, flush_frames)):
        ok, candidate = cap.read()
        if ok and candidate is not None and getattr(candidate, "size", 0):
            frame = candidate
    return frame


def _scan_one_pose(args, config, pose_spec, model, cap, vgb, acg, cv2):
    geometry = _geometry_args(
        config["mapping"]["homography"],
        (args.policy_x_lo, args.policy_x_hi),
        (args.policy_y_lo, args.policy_y_hi), args.allow_top_clipped)
    vgb.resolve_camera_geometry(geometry)
    pose = _dynamic_pose(acg, pose_spec, config["mapping"]["homography"])
    class_height = {"sugarbox": args.class_height}
    class_z = {"_fallback": args.class_height / 2.0,
               "sugarbox": args.class_height / 2.0}
    def target_transform(xy):
        rotated = map_reference_xy_for_pose(
            xy, pose_spec, config["mapping"])
        return vgb.apply_grasp_forward_offset(
            rotated, args.grasp_forward_offset_mm)

    samples = []
    deadline = time.monotonic() + config["per_pose_timeout_sec"]
    pending = _grab_after_motion(cap, cv2)
    last_reason = "no box"
    while time.monotonic() < deadline and len(samples) < config["samples_per_pose"]:
        if pending is None:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
        else:
            frame, pending = pending, None
        result = model.predict(frame, imgsz=640, conf=args.conf, verbose=False)[0]
        box = vgb.pick_best_box(result)
        if box is None:
            last_reason = "YOLO found no box"
            continue
        class_name = vgb.class_name_for_box(model, box)
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        payload, reason = vgb.build_homography_payload(
            geometry, (x1, y1, x2, y2), class_name, pose,
            class_z, class_height,
            base_xy_transform=target_transform)
        if payload is None:
            last_reason = reason
            continue
        samples.append(payload)
    if not samples:
        print("[scan] {}: no valid target ({})".format(pose_spec["name"], last_reason))
        return None
    try:
        result = aggregate_pose_samples(samples, config["within_pose_spread_m"])
    except ScanConfigError as exc:
        print("[scan] {} rejected: {}".format(pose_spec["name"], exc))
        return None
    result["pose"] = pose_spec["name"]
    print("[scan] {}: target ({:+.4f},{:+.4f}) from {} samples, spread {:.1f} mm"
          .format(pose_spec["name"], result["x"], result["y"],
                  result["sample_count"], result["spread_m"] * 1000.0))
    return result


def _calibration_loop(args, config, pose_spec, model, cap, vgb,
                      reference_geometry=None):
    records = []
    active_class = None
    print("[scan][calibration] {}: move the object, wait for one median line, "
          "then record its measured base x,y; Ctrl+C returns to E1".format(
              pose_spec["name"]))
    if pose_spec["name"] == config["mapping"]["reference_pose"]:
        print("[scan][calibration] E1 owns the one reference homography: {}".format(
            config["mapping"]["homography"]))
    else:
        print("[scan][calibration] this S1-only side view reuses E1's homography "
              "plus yaw rotation; do NOT fit or copy a second homography")
        print("[scan][calibration] compare predicted_x/y with ruler-measured base "
              "x/y at >=2 held-out points across this view; require each axis <=1 cm")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        result = model.predict(frame, imgsz=640, conf=args.conf, verbose=False)[0]
        box = vgb.pick_best_box(result)
        if box is None:
            continue
        class_name = vgb.class_name_for_box(model, box)
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        record, reason = vgb.calibration_record_from_bbox(
            (x1, y1, x2, y2), class_name)
        if record is None:
            continue
        if active_class is not None and class_name != active_class:
            records = []
        active_class = class_name
        records.append(record)
        if len(records) >= args.calibration_samples:
            median = vgb.median_calibration_record(records)
            median["scan_pose"] = pose_spec["name"]
            if reference_geometry is not None:
                try:
                    reference_xy = vgb.apply_grasp_home_mapping(
                        reference_geometry, median["u"], median["v"])
                    predicted = map_reference_xy_for_pose(
                        reference_xy, pose_spec, config["mapping"])
                    median["predicted_x"] = round(predicted[0], 4)
                    median["predicted_y"] = round(predicted[1], 4)
                    median["mapping"] = "E1_homography_plus_S1_yaw"
                except ValueError as exc:
                    median["prediction_rejected"] = str(exc)
            print("[scan][calibration] {}".format(
                json.dumps(median, ensure_ascii=False)), flush=True)
            records = []


def parse_args_from(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--camera", default=DEFAULT_CAMERA)
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--class-height", type=float, default=0.065)
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--policy-x-range", type=float, nargs=2,
                        default=(0.205, 0.280), metavar=("LO", "HI"))
    parser.add_argument("--policy-y-range", type=float, nargs=2,
                        default=(-0.070, 0.065), metavar=("LO", "HI"))
    parser.add_argument("--grasp-forward-offset-mm", type=float, default=0.0,
                        help="translate the final base-frame target in +X after "
                             "the S1 view rotation (runtime only, 0..15 mm)")
    parser.add_argument("--allow-top-clipped", action="store_true")
    parser.add_argument("--calibrate-pose", default=None)
    parser.add_argument("--calibration-samples", type=int, default=20)
    parser.add_argument("--pose-tol-deg", type=float, default=2.0)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--unlock-unvalidated-scan", action="store_true",
                        help="run poses whose hardware_validated / "
                             "yaw_mapping_validated flags are still false, as a "
                             "supervised experiment. Same shape as "
                             "--unlock-candidate-real: the evidence is still "
                             "missing and the config still says so. Every other "
                             "gate stays enforced.")
    parser.add_argument("--accept-single-rotated-view", action="store_true",
                        help="release a target seen by exactly one rotated "
                             "(LEFT/RIGHT) view even when the reference pose "
                             "cannot confirm it afterwards. Without this the "
                             "scan refuses, because nothing else tests the S1 "
                             "rotation at runtime.")
    parser.add_argument("--i-am-beside-the-robot", action="store_true")
    args = parser.parse_args(argv)
    args.policy_x_lo, args.policy_x_hi = args.policy_x_range
    args.policy_y_lo, args.policy_y_hi = args.policy_y_range
    return args


def parse_args():
    return parse_args_from(None)


def main():
    args = parse_args()
    numeric = (args.class_height, args.conf, args.pose_tol_deg,
               args.grasp_forward_offset_mm,
               args.policy_x_lo, args.policy_x_hi,
               args.policy_y_lo, args.policy_y_hi)
    if not all(math.isfinite(float(value)) for value in numeric):
        print("[FATAL] scan numeric arguments must be finite.")
        return 2
    if args.class_height <= 0.0 or not 0.0 <= args.conf <= 1.0:
        print("[FATAL] --class-height must be positive and --conf must be in [0,1].")
        return 2
    if not 0.0 <= args.grasp_forward_offset_mm <= 15.0:
        print("[FATAL] --grasp-forward-offset-mm must be in [0, 15].")
        return 2
    if args.calibrate_pose is not None and args.grasp_forward_offset_mm != 0.0:
        print("[FATAL] --grasp-forward-offset-mm is runtime-only; scan calibration "
              "must report the uncorrected E1/S1 geometry.")
        return 2
    if not (args.policy_x_lo < args.policy_x_hi
            and args.policy_y_lo < args.policy_y_hi):
        print("[FATAL] policy ranges must be increasing.")
        return 2
    if args.calibration_samples <= 0:
        print("[FATAL] --calibration-samples must be positive.")
        return 2
    try:
        config = load_scan_config(
            args.config, require_homographies=args.calibrate_pose is None,
            calibration_pose=args.calibrate_pose,
            unlock_unvalidated=args.unlock_unvalidated_scan)
        if args.unlock_unvalidated_scan and args.calibrate_pose is None:
            announce_unvalidated(config)
    except ScanConfigError as exc:
        print("[FATAL] {}".format(exc))
        return 2
    pose_by_name = {pose["name"]: pose for pose in config["poses"]}
    if args.calibrate_pose is not None and args.calibrate_pose not in pose_by_name:
        print("[FATAL] unknown --calibrate-pose {!r}; expected {}".format(
            args.calibrate_pose, sorted(pose_by_name)))
        return 2

    # Validate the one reference calibration through the bridge's exact loader before
    # importing camera/servo drivers or opening any hardware.
    if args.calibrate_pose is None or config["mapping"]["homography"].is_file():
        try:
            from grasp_home_homography import load_calibration
            load_calibration(str(config["mapping"]["homography"]), min_points=6,
                             max_error_m=0.02)
            print("[check] shared E1 homography: {}".format(
                config["mapping"]["homography"]))
        except Exception as exc:
            print("[FATAL] scan homography rejected: {}".format(exc))
            return 2

    if not (0.0 < args.pose_tol_deg <= 3.0):
        print("[FATAL] --pose-tol-deg must be in (0, 3].")
        return 2
    if args.check:
        if args.calibrate_pose:
            print("[check] PASS: validation-collection pose {} is configured; no "
                  "camera, serial, or arm opened.".format(args.calibrate_pose))
        else:
            print("[check] PASS: exactly three validated poses; no camera, serial, "
                  "or arm opened.")
        return 0
    if not args.i_am_beside_the_robot:
        print("[REFUSED] three-pose scan moves the real arm. Add "
              "--i-am-beside-the-robot while standing at the power switch.")
        return 3
    if not Path(args.model).is_file():
        print("[FATAL] YOLO model does not exist: {}".format(args.model))
        return 2
    if str(args.camera).startswith("/dev/") and not Path(args.camera).exists():
        print("[FATAL] camera does not exist: {}".format(args.camera))
        return 2
    if not Path(args.port).exists():
        print("[FATAL] Rosmaster port does not exist: {}".format(args.port))
        return 2

    import cv2
    from ultralytics import YOLO
    import arm_cam_geometry as acg
    import vision_grasp_bridge as vgb
    import x3plus_real_grasp as X
    from pose_explorer import Mover, check_mover_contract

    if not getattr(X, "_ROSMASTER_AVAILABLE", False):
        print("[FATAL] Rosmaster_Lib is not importable; refusing real scan motion.")
        return 2
    missing = check_mover_contract(X)
    if missing:
        print("[FATAL] guarded-move Mover contract changed: {}".format(missing))
        return 2
    print("[scan] loading YOLO before opening the servo port: {}".format(args.model))
    model = YOLO(args.model)
    cap, first_frame, error = vgb.open_capture_checked(str(args.camera))
    if cap is None:
        print("[FATAL] camera failed: {}".format(error))
        return 2
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    cfg = X.DeployConfig(serial_port=args.port)
    mapper = X.JointMapper(cfg)
    fk = X.FKComputer(cfg.urdf_path)
    servo = None
    mover = None
    interrupted = False
    run_error = None
    return_ok = False
    results = []
    confirmation = None
    confirmation_needed = False
    reference_geometry = None
    if args.calibrate_pose and config["mapping"]["homography"].is_file():
        reference_geometry = _geometry_args(
            config["mapping"]["homography"],
            (args.policy_x_lo, args.policy_x_hi),
            (args.policy_y_lo, args.policy_y_hi), False)
        vgb.resolve_camera_geometry(reference_geometry)
    try:
        servo = X.ServoController(cfg, dry_run=False)
        reading = servo.read_degrees()
        if not reading.valid:
            print("[FATAL] cannot read servos: {}".format(reading.reason))
            return 2
        mover = Mover(
            cfg, servo, mapper, X.FloorGuard(cfg, fk),
            mapper.hw_deg_to_sim_arm(reading.degrees[:5]),
            mapper.hw_deg_to_sim_grip(reading.degrees[5]))
        # Establish the known reference before any lateral sweep.  Starting from
        # an arbitrary powered-up pose and heading directly to LEFT would make the
        # largest move the first one and would skip the one pose whose geometry is
        # physically calibrated.
        home = config["return_pose"]
        print("[scan] positioning at reference {} {} before scan".format(
            home["name"], list(home["arm_deg"])))
        initial = mover.move_to(X, home["arm_deg"], tol_deg=args.pose_tol_deg)
        print("[scan] initial reference reached={} ({})".format(
            initial["reached"], initial["reason"]))
        if not initial["reached"]:
            raise ScanConfigError("did not confirm E1 before lateral scan")
        scan_poses = ([pose_by_name[args.calibrate_pose]]
                      if args.calibrate_pose else config["poses"])
        for pose in scan_poses:
            print("[scan] moving to {} {}".format(pose["name"], list(pose["arm_deg"])))
            move = mover.move_to(X, pose["arm_deg"], tol_deg=args.pose_tol_deg)
            print("[scan] {} reached={} ({})".format(
                pose["name"], move["reached"], move["reason"]))
            if not move["reached"]:
                raise ScanConfigError("did not confirm pose {}".format(pose["name"]))
            if args.calibrate_pose:
                _grab_after_motion(cap, cv2)
                _calibration_loop(args, config, pose, model, cap, vgb,
                                  reference_geometry)
            else:
                result = _scan_one_pose(
                    args, config, pose, model, cap, vgb, acg, cv2)
                if result is not None:
                    results.append(result)
    except KeyboardInterrupt:
        interrupted = True
        print("\n[scan] interrupted; returning to E1 before exit.")
    except ScanConfigError as exc:
        run_error = str(exc)
        print("[FATAL] {}".format(exc))
    finally:
        if mover is not None:
            home = config["return_pose"]
            print("[scan] returning to {} {}".format(home["name"], list(home["arm_deg"])))
            move = mover.move_to(X, home["arm_deg"], tol_deg=args.pose_tol_deg)
            return_ok = bool(move["reached"])
            print("[scan] return reached={} ({})".format(return_ok, move["reason"]))
        # One zero-rotation look, taken here because the arm is confirmed at the
        # reference pose and the camera is still open. Deliberately inside the
        # finally and behind every one of these conditions: an interrupted or
        # failed scan releases nothing, so confirming it would be wasted motion
        # on a run that is already refused.
        confirmation_needed = (
            not interrupted and run_error is None and return_ok
            and not args.calibrate_pose
            and needs_reference_confirmation(
                results, config["poses"], config["mapping"]["reference_s1_deg"]))
        if confirmation_needed:
            reference = next(
                (pose for pose in config["poses"]
                 if pose["name"] == config["return_pose"]["name"]), None)
            if reference is None:
                print("[scan] reference pose missing from the pose list; "
                      "cannot confirm.")
            else:
                print("[scan] single rotated view ({}); confirming at {} with no "
                      "rotation applied.".format(results[0].get("pose"),
                                                 reference["name"]))
                try:
                    confirmation = _scan_one_pose(
                        args, config, reference, model, cap, vgb, acg, cv2)
                except Exception as exc:            # never mask the return
                    confirmation = None
                    print("[scan] confirmation failed: {}".format(exc))
                if confirmation is None:
                    print("[scan] {} did not see the object.".format(
                        reference["name"]))
                else:
                    print("[scan] {} confirms ({:.4f}, {:.4f}).".format(
                        reference["name"], confirmation["x"], confirmation["y"]))
        cap.release()
        fk.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    code, fused, reason = finalize_scan_result(
        return_ok, interrupted, run_error, args.calibrate_pose, results,
        config["cross_pose_tolerance_m"],
        confirmation_needed=confirmation_needed, confirmation=confirmation,
        accept_unconfirmed=args.accept_single_rotated_view)
    if code != 0:
        if code != 130:
            print("[FATAL] {}; no target will be released.".format(reason))
        return code
    if fused is None:
        return 0
    # This is the sole machine-readable handoff. It happens after E1 return.
    print(RESULT_PREFIX + json.dumps(fused, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
