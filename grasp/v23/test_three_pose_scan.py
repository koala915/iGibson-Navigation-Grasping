#!/usr/bin/env python3
"""Pure-logic checks for the exclusive three-pose search handoff."""

import inspect
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import jetson_one_command_grasp as launcher  # noqa: E402
import three_pose_scan as scan               # noqa: E402


checks = 0
failures = []


def ok(condition, label):
    global checks
    checks += 1
    print("  [{}] {}".format("ok" if condition else "FAIL", label))
    if not condition:
        failures.append(label)


def expect_config_error(fn, text, label):
    try:
        fn()
    except scan.ScanConfigError as exc:
        ok(text in str(exc), label)
    else:
        ok(False, label)


def base_document(directory):
    poses = []
    rows = (
        ("LEFT", [70, 74.2, 8.6, 8.6, 90, 30]),
        ("E1", [90, 74.2, 8.6, 8.6, 90, 30]),
        ("RIGHT", [110, 74.2, 8.6, 8.6, 90, 30]),
    )
    homography = directory / "e1.json"
    homography.write_text("{}", encoding="utf-8")
    for name, arm in rows:
        poses.append({
            "name": name, "arm_deg": list(arm),
            "hardware_validated": True,
            "yaw_mapping_validated": True,
        })
    return {
        "schema": scan.SCHEMA, "version": 2,
        "mapping": {
            "type": "s1_yaw_from_reference",
            "reference_pose": "E1", "reference_s1_deg": 90,
            "homography": homography.name,
            "pivot_xy_m": list(scan.S1_PIVOT_XY_M),
            "yaw_sign": scan.S1_YAW_SIGN,
        },
        "return_pose": {"name": "E1", "arm_deg": list(rows[1][1])},
        "poses": poses,
        "samples_per_pose": 5,
        "per_pose_timeout_sec": 8.0,
        "within_pose_spread_m": 0.008,
        "cross_pose_tolerance_m": 0.01,
    }


def write_document(directory, document):
    path = directory / "scan.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def main():
    print("three-pose scan tests (no camera, serial, PyBullet, YOLO, or PPO)\n")
    with tempfile.TemporaryDirectory() as raw:
        directory = Path(raw)
        doc = base_document(directory)
        path = write_document(directory, doc)
        loaded = scan.load_scan_config(str(path))
        ok([pose["name"] for pose in loaded["poses"]] == ["LEFT", "E1", "RIGHT"],
           "exactly three ordered lateral poses load")
        ok(loaded["return_pose"]["name"] == "E1",
           "return pose is explicit")
        ok(loaded["mapping"]["homography"].is_absolute(),
           "the one reference homography resolves relative to the config")

        missing = base_document(directory)
        (directory / "e1.json").unlink()
        missing_path = write_document(directory, missing)
        expect_config_error(
            lambda: scan.load_scan_config(str(missing_path)),
            "does not exist", "runtime refuses a missing reference homography")
        calibrated = scan.load_scan_config(
            str(missing_path), require_homographies=False, calibration_pose="LEFT")
        ok(calibrated["poses"][0]["name"] == "LEFT",
           "validation collection can run before the reference file is present")

        duplicate = base_document(directory)
        duplicate["poses"][2]["arm_deg"] = duplicate["poses"][0]["arm_deg"]
        duplicate_path = write_document(directory, duplicate)
        expect_config_error(
            lambda: scan.load_scan_config(str(duplicate_path),
                                          require_homographies=False),
            "distinct arm positions", "duplicate arm poses are rejected")

        unvisited = base_document(directory)
        unvisited["poses"][0]["hardware_validated"] = False
        unvisited_path = write_document(directory, unvisited)
        expect_config_error(
            lambda: scan.load_scan_config(str(unvisited_path),
                                          require_homographies=False),
            "not hardware_validated", "unvisited motion cannot enter the scanner")

        unverified_yaw = base_document(directory)
        unverified_yaw["poses"][2]["yaw_mapping_validated"] = False
        unverified_yaw_path = write_document(directory, unverified_yaw)
        expect_config_error(
            lambda: scan.load_scan_config(str(unverified_yaw_path),
                                          require_homographies=False),
            "yaw mapping", "an unmeasured side-view rotation cannot enter runtime")

        changed_pitch = base_document(directory)
        changed_pitch["poses"][0]["arm_deg"][1] = 75.0
        changed_pitch_path = write_document(directory, changed_pitch)
        expect_config_error(
            lambda: scan.load_scan_config(str(changed_pitch_path),
                                          require_homographies=False),
            "S2-S5", "single-homography reuse rejects every non-S1 pose change")

        wrong_pivot = base_document(directory)
        wrong_pivot["mapping"]["pivot_xy_m"][0] += 0.01
        wrong_pivot_path = write_document(directory, wrong_pivot)
        expect_config_error(
            lambda: scan.load_scan_config(str(wrong_pivot_path),
                                          require_homographies=False),
            "S1 pivot", "an edited yaw pivot is rejected")

        wrong_sign = base_document(directory)
        wrong_sign["mapping"]["yaw_sign"] = 1.0
        wrong_sign_path = write_document(directory, wrong_sign)
        expect_config_error(
            lambda: scan.load_scan_config(str(wrong_sign_path),
                                          require_homographies=False),
            "yaw_sign", "the opposite S1 rotation direction is rejected")

        wide_yaw = base_document(directory)
        wide_yaw["poses"][2]["arm_deg"][0] = 120.0
        wide_yaw_path = write_document(directory, wide_yaw)
        expect_config_error(
            lambda: scan.load_scan_config(str(wide_yaw_path),
                                          require_homographies=False),
            "between 5 and 25", "unreviewed yaw beyond 25 degrees is rejected")

        collection = base_document(directory)
        for side in (collection["poses"][0], collection["poses"][2]):
            side["hardware_validated"] = False
            side["yaw_mapping_validated"] = False
        collection_path = write_document(directory, collection)
        collected = scan.load_scan_config(
            str(collection_path), require_homographies=False,
            calibration_pose="LEFT")
        ok(collected["poses"][0]["name"] == "LEFT",
           "explicit validation collection may visit an unapproved side pose")

        wrong_home = base_document(directory)
        wrong_home["return_pose"]["arm_deg"][1] = 75.0
        wrong_home_path = write_document(directory, wrong_home)
        expect_config_error(
            lambda: scan.load_scan_config(str(wrong_home_path),
                                          require_homographies=False),
            "exactly match", "return E1 cannot drift from a configured pose")

        wrong_name = base_document(directory)
        wrong_name["return_pose"] = {
            "name": "LEFT", "arm_deg": list(wrong_name["poses"][0]["arm_deg"])}
        wrong_name_path = write_document(directory, wrong_name)
        expect_config_error(
            lambda: scan.load_scan_config(str(wrong_name_path),
                                          require_homographies=False),
            "must be v23 E1", "a different configured pose cannot replace E1")

        closed_jaw = base_document(directory)
        closed_jaw["poses"][1]["arm_deg"][5] = 90.0
        closed_path = write_document(directory, closed_jaw)
        expect_config_error(
            lambda: scan.load_scan_config(str(closed_path),
                                          require_homographies=False),
            "S6", "every search pose keeps the jaw fully open")

        mapping = loaded["mapping"]
        reference_xy = (0.25, 0.0)
        left_xy = scan.map_reference_xy_for_pose(
            reference_xy, loaded["poses"][0], mapping)
        right_xy = scan.map_reference_xy_for_pose(
            reference_xy, loaded["poses"][2], mapping)
        ok(left_xy[1] > reference_xy[1] and right_xy[1] < reference_xy[1],
           "S1=70 looks left and S1=110 looks right in base coordinates")
        left_radius = ((left_xy[0] - mapping["pivot_xy_m"][0]) ** 2
                       + (left_xy[1] - mapping["pivot_xy_m"][1]) ** 2) ** 0.5
        reference_radius = ((reference_xy[0] - mapping["pivot_xy_m"][0]) ** 2
                            + (reference_xy[1] - mapping["pivot_xy_m"][1]) ** 2) ** 0.5
        ok(abs(left_radius - reference_radius) < 1e-12,
           "S1 mapping is a width- and radius-preserving rigid rotation")

    print("\n1. within-pose stability")
    samples = [
        {"x": 0.250, "y": -0.020, "z": 0.0325, "w": 0.025,
         "height": 0.065, "class": "sugarbox"},
        {"x": 0.252, "y": -0.019, "z": 0.0325, "w": 0.026,
         "height": 0.065, "class": "sugarbox"},
        {"x": 0.251, "y": -0.021, "z": 0.0325, "w": 0.024,
         "height": 0.065, "class": "sugarbox"},
    ]
    aggregate = scan.aggregate_pose_samples(samples, 0.008)
    ok(aggregate["x"] == 0.251 and aggregate["sample_count"] == 3,
       "stable samples collapse to a median")
    expect_config_error(
        lambda: scan.aggregate_pose_samples(
            samples + [dict(samples[0], x=0.280)], 0.008),
        "spread", "a moving or flickering target is rejected")
    expect_config_error(
        lambda: scan.aggregate_pose_samples(
            [samples[0], dict(samples[1], **{"class": "other"})], 0.008),
        "class", "one pose cannot switch object class")

    print("\n2. cross-pose consensus")
    e1 = dict(aggregate, pose="E1")
    f3 = dict(aggregate, pose="RIGHT", x=0.254, y=-0.018)
    single = scan.fuse_pose_results([e1], 0.01)
    ok(single["pose_count"] == 1 and single["poses"] == ["E1"],
       "one exclusive view may supply the target")
    fused = scan.fuse_pose_results([e1, f3], 0.01)
    ok(fused["pose_count"] == 2 and fused["x"] == 0.2525,
       "agreeing views fuse by median")
    expect_config_error(
        lambda: scan.fuse_pose_results(
            [e1, dict(f3, x=0.275, y=0.010)], 0.01),
        "disagree", "different objects or bad calibrations fail closed")
    expect_config_error(
        lambda: scan.fuse_pose_results([], 0.01),
        "no valid target", "three misses never become a default grasp")

    print("\n3. E1 release gate")
    code, released, _ = scan.finalize_scan_result(
        True, False, None, None, [e1, f3], 0.01)
    ok(code == 0 and released is not None,
       "an encoder-confirmed E1 return releases an agreeing target")
    code, released, _ = scan.finalize_scan_result(
        False, False, None, None, [e1], 0.01)
    ok(code == 2 and released is None,
       "an unconfirmed E1 return cannot release a target")
    code, released, _ = scan.finalize_scan_result(
        True, True, None, None, [e1], 0.01)
    ok(code == 130 and released is None,
       "an interrupted scan returns no target even after reaching E1")
    code, released, _ = scan.finalize_scan_result(
        True, False, "pose failed", None, [e1], 0.01)
    ok(code == 2 and released is None,
       "a scan error cannot release a stale partial target")
    code, released, _ = scan.finalize_scan_result(
        True, False, None, "F3", [], 0.01)
    ok(code == 0 and released is None,
       "calibration returns to E1 without starting a grasp")

    print("\n4. launcher handoff")
    args = launcher.parse_args_from(["--three-pose-scan"])
    scan_cmd = launcher.build_scan_cmd(args)
    ok("--i-am-beside-the-robot" in scan_cmd,
       "scanner motion carries an explicit supervised-hardware acknowledgement")
    ok("--config" in scan_cmd and "--policy-x-range" in scan_cmd,
       "scanner receives its config and the v23 policy envelope")
    ok(float(scan_cmd[scan_cmd.index("--grasp-forward-offset-mm") + 1]) == 5.0,
       "scanner receives the same +5 mm base-X correction as the single view")
    calibration_cmd = launcher.build_scan_cmd(
        launcher.parse_args_from(["--scan-calibrate-pose", "LEFT"]))
    ok("--grasp-forward-offset-mm" not in calibration_cmd,
       "scan calibration reports raw geometry without runtime correction")
    target = {
        "x": 0.251, "y": -0.020, "z": 0.0325, "height": 0.065,
        "pose_count": 2, "poses": ["E1", "F3"],
    }
    fixed_cmd = launcher.build_ctrl_cmd(args, fixed_target=target)
    ok("--socket" not in fixed_cmd and "--latch-obj" not in fixed_cmd,
       "post-scan controller has no live camera/socket source")
    ok("--obj-x" in fixed_cmd and "--object-height" in fixed_cmd,
       "post-scan controller receives a frozen base target")
    normal_cmd = launcher.build_ctrl_cmd(launcher.parse_args_from([]))
    ok("--socket" in normal_cmd and "--latch-obj" in normal_cmd,
       "legacy E1 single-view path remains unchanged")

    line = scan.RESULT_PREFIX + json.dumps(target)
    parsed = launcher.parse_scan_result_line(line)
    ok(parsed["x"] == target["x"],
       "launcher parses the one machine-readable result")
    ok(launcher.parse_scan_result_line("[scan] ordinary log") is None,
       "ordinary scanner logs cannot become a target")
    try:
        launcher.parse_scan_result_line(
            scan.RESULT_PREFIX + '{"x": NaN, "y": 0, "z": 0, "height": 1, '
            '"pose_count": 1, "poses": ["E1"]}')
    except ValueError:
        ok(True, "non-finite scan results are refused")
    else:
        ok(False, "non-finite scan results are refused")
    scanner_source = inspect.getsource(launcher.run_scanner)
    ok("start_new_session=SCAN_START_NEW_SESSION" in scanner_source,
       "terminal Ctrl+C cannot hit launcher and scanner simultaneously")
    ok("grace_seconds=scan_interrupt_grace_sec(" in scanner_source,
       "the E1-return window is derived per run, not a flat constant")
    ok(scanner_source.count("stop_process(") == 1,
       "the scanner is signalled from exactly ONE place, so it gets one SIGINT")

    # The window must outlast what actually happens inside that finally: the
    # guarded return, then the confirming reference detection.
    cfg_path = HERE / "three_pose_scan.json"
    document = json.loads(cfg_path.read_text(encoding="utf-8"))
    per_pose = float(document["per_pose_timeout_sec"])
    grace = launcher.scan_interrupt_grace_sec(str(cfg_path))
    ok(grace >= launcher.SCAN_RETURN_BUDGET_SEC + per_pose,
       "grace {:.0f}s covers the worst-case return {:.0f}s plus the {:.0f}s "
       "confirming detection".format(
           grace, launcher.SCAN_RETURN_BUDGET_SEC, per_pose))
    ok(launcher.SCAN_RETURN_BUDGET_SEC >= 40 * 0.35,
       "the return budget is built from max_iters x settle, not guessed")
    ok(launcher.scan_interrupt_grace_sec("does-not-exist.json")
       == launcher.SCAN_RETURN_BUDGET_SEC + launcher.SCAN_TEARDOWN_BUDGET_S,
       "an unreadable config shortens the window rather than inventing one")
    ok(launcher.scan_interrupt_grace_sec(str(cfg_path)) > 
       launcher.scan_interrupt_grace_sec(None),
       "reading the config lengthens the window by the confirmation cost")

    # Behavioural, not a grep. The guarantee is "one SIGINT per process", and
    # the only honest way to test it is to count the signals a fake child
    # receives across the two calls the interrupt path makes.
    class FakeProc(object):
        """A child that stays alive until it has been signalled once."""

        _next_pid = 91000

        def __init__(self, exits_after_signal=True):
            FakeProc._next_pid += 1
            self.pid = FakeProc._next_pid
            self.signals = []
            self.terminated = 0
            self.killed = 0
            self._alive = True
            self._exits = exits_after_signal

        def poll(self):
            return None if self._alive else 0

        def send_signal(self, sig):
            self.signals.append(sig)

        def wait(self, timeout=None):
            if not self._alive:
                return 0
            if self._exits and self.signals:
                self._alive = False
                return 0
            raise subprocess.TimeoutExpired("scanner", timeout)

        def terminate(self):
            self.terminated += 1
            self._alive = False

        def kill(self):
            self.killed += 1
            self._alive = False

    launcher._SIGNALLED_PIDS.clear()
    proc = FakeProc()
    launcher.stop_process(proc, "fake", grace_seconds=0.01)
    launcher.stop_process(proc, "fake", grace_seconds=0.01)
    ok(proc.signals == [signal.SIGINT],
       "two stop_process calls send exactly ONE SIGINT ({})".format(proc.signals))

    # The case that actually bit: a child STILL RUNNING its guarded return when
    # a second call arrives. stop_process's own escalation always ends in kill,
    # so a first call can never leave the child in that state -- the pid record
    # is seeded directly, which is exactly the situation a future second caller
    # would create.
    launcher._SIGNALLED_PIDS.clear()
    slow = FakeProc(exits_after_signal=False)
    launcher._SIGNALLED_PIDS.add(slow.pid)
    launcher.stop_process(slow, "slow", grace_seconds=0.01)
    ok(slow.signals == [],
       "an already-signalled, still-running child is NOT signalled again")
    ok(slow.terminated >= 1,
       "...it is waited for, then escalated once its window expires")

    launcher._SIGNALLED_PIDS.clear()
    done = FakeProc()
    done._alive = False
    launcher.stop_process(done, "already exited", grace_seconds=0.01)
    ok(done.signals == [],
       "an already-exited child is never signalled at all")
    launcher._SIGNALLED_PIDS.clear()

    print("\n11. a lone ROTATED view is confirmed at the reference, or refused")
    # The cross-pose 1 cm gate is the only runtime test of the S1 rotation. One
    # view removes it, so the reference pose -- where no rotation is applied --
    # has to put it back.
    POSES = [{"name": "LEFT", "arm_deg": [70.0, 74.2, 8.6, 8.6, 90.0, 30.0]},
             {"name": "E1", "arm_deg": [90.0, 74.2, 8.6, 8.6, 90.0, 30.0]},
             {"name": "RIGHT", "arm_deg": [110.0, 74.2, 8.6, 8.6, 90.0, 30.0]}]

    def hit(pose, x, y, cls="sugarbox"):
        return {"pose": pose, "x": x, "y": y, "z": 0.015, "w": 0.023,
                "class": cls, "height": 0.03}

    need = scan.needs_reference_confirmation
    ok(need([hit("LEFT", 0.24, 0.02)], POSES, 90.0) is True,
       "one LEFT view needs confirming")
    ok(need([hit("RIGHT", 0.24, -0.02)], POSES, 90.0) is True,
       "one RIGHT view needs confirming")
    ok(need([hit("E1", 0.24, 0.0)], POSES, 90.0) is False,
       "one E1 view does NOT -- at S1=90 no rotation is applied")
    ok(need([hit("LEFT", 0.24, 0.02), hit("E1", 0.24, 0.02)], POSES, 90.0) is False,
       "two views already cross-check each other")
    ok(need([hit("MYSTERY", 0.24, 0.0)], POSES, 90.0) is True,
       "an unrecognised pose name is treated as rotated, not assumed safe")

    def final(results, **kw):
        return scan.finalize_scan_result(
            True, False, None, None, results, 0.01, **kw)

    lone = [hit("LEFT", 0.2400, 0.0200)]
    code, fused, reason = final(lone, confirmation_needed=True, confirmation=None)
    ok(code == 2 and fused is None,
       "unconfirmed lone rotated view is REFUSED by default")
    ok("rest entirely on the S1 rotation" in reason,
       "...and the refusal says why, not just that it failed")

    code, fused, _ = final(lone, confirmation_needed=True, confirmation=None,
                           accept_unconfirmed=True)
    ok(code == 0 and fused is not None,
       "--accept-single-rotated-view is the only way past it")

    agree = hit("E1", 0.2405, 0.0203)
    code, fused, _ = final(lone, confirmation_needed=True, confirmation=agree)
    ok(code == 0 and fused is not None and fused["pose_count"] == 2,
       "a confirming reference detection is fused in as a second view")
    ok(fused is not None and set(fused["poses"]) == {"LEFT", "E1"},
       "...and both poses are recorded in the released target")

    disagree = hit("E1", 0.2400, -0.0800)     # 10 cm away: a flipped yaw sign
    code, fused, reason = final(lone, confirmation_needed=True,
                                confirmation=disagree)
    ok(code == 2 and fused is None and "disagree" in reason,
       "a reference detection that disagrees REFUSES, it does not average")

    code, fused, reason = final(lone, confirmation_needed=True,
                                confirmation=disagree, accept_unconfirmed=True)
    ok(code == 2 and fused is None,
       "--accept-single-rotated-view cannot override an actual disagreement")

    wrong_class = hit("E1", 0.2405, 0.0203, cls="eraser")
    code, fused, _ = final(lone, confirmation_needed=True,
                           confirmation=wrong_class)
    ok(code == 2 and fused is None,
       "the confirmation has to be the same object class")

    code, fused, _ = final(lone, confirmation_needed=False, confirmation=None)
    ok(code == 0 and fused is not None and fused["pose_count"] == 1,
       "when confirmation is not needed the lone view still releases")

    code, _, _ = scan.finalize_scan_result(
        True, True, None, None, lone, 0.01, confirmation_needed=True,
        confirmation=agree)
    ok(code == 130, "an interrupted scan still releases nothing, confirmed or not")
    code, _, _ = scan.finalize_scan_result(
        False, False, None, None, lone, 0.01, confirmation_needed=True,
        confirmation=agree)
    ok(code == 2, "an unconfirmed E1 RETURN still releases nothing")

    scan_source = inspect.getsource(scan.main)
    ok("cap.release()" in scan_source
       and scan_source.index("needs_reference_confirmation")
       < scan_source.index("cap.release()"),
       "the confirming grab happens while the camera is still open")
    ok("not interrupted and run_error is None and return_ok" in scan_source,
       "confirmation is skipped on runs that are already refused")

    print("")
    print("12. the fabricated scan pose poisons the trig model, not feeds it")
    import arm_cam_geometry as acg_mod
    # Built from a REAL parsed pose. The first version of this test passed a
    # hand-written dict with a "homography" key that load_scan_config never
    # emits; _dynamic_pose read that key, so the fixture satisfied it and the
    # KeyError waited until the first hardware scan instead.
    with tempfile.TemporaryDirectory() as _tmp:
        _path = Path(_tmp) / "real.json"
        _path.write_text(json.dumps(base_document(Path(_tmp))), encoding="utf-8")
        _cfg = scan.load_scan_config(str(_path))
    _left = next(p for p in _cfg["poses"] if p["name"] != _cfg["return_pose"]["name"])
    ok("homography" not in _left,
       "a parsed pose carries no homography key -- the shared one is in mapping")
    # Guarded: a raw traceback out of a check is a worse signal than a named
    # failure. Reintroducing the KeyError this pins used to kill the whole suite
    # here, which reads like the tests are broken rather than the code.
    try:
        fake = scan._dynamic_pose(acg_mod, _left, _cfg["mapping"]["homography"])
    except Exception as _exc:
        ok(False, "_dynamic_pose raised {} on a real parsed pose: {}".format(
            type(_exc).__name__, _exc))
        fake = None
    if fake is None:
        fake = acg_mod.ArmCamPose(
            name="unbuilt", arm_deg=tuple(_left["arm_deg"]),
            theta_deg=float("nan"), h_m=float("nan"), cam_x_m=float("nan"),
            cam_y_m=float("nan"), sign_y=-1.0, distance_model_measured=False,
            base_offset_measured=False, source="stand-in after failure")
    ok(not any(math.isfinite(v) for v in
               (fake.theta_deg, fake.h_m, fake.cam_x_m, fake.cam_y_m)),
       "its extrinsics are non-finite, not plausible placeholders")
    ok(tuple(fake.arm_deg) == tuple(_left["arm_deg"]),
       "the arm angles it DOES carry are real -- stamp_payload reads them")
    ok(scan._dynamic_pose(acg_mod, _left) is not None,
       "...and it still builds when no homography path is supplied")
    stamped = acg_mod.stamp_payload({}, fake)
    ok(stamped["cam_pose"] == [70.0, 74.2, 8.6, 8.6, 90.0, 30.0],
       "stamping still works, which is this pose's only real job")
    try:
        acg_mod.ground_hit(300.0, 400.0, fake)
        ok(False, "ground_hit on the fabricated pose must refuse")
    except acg_mod.GroundGeometryError:
        ok(True, "ground_hit REFUSES rather than returning a confident number")
    except Exception as exc:
        ok(False, "ground_hit raised {} instead of GroundGeometryError".format(
            type(exc).__name__))

    print("")
    print("13. --unlock-unvalidated-scan waives the evidence flags and NOTHING else")
    with tempfile.TemporaryDirectory() as tmp:
        raw_doc = base_document(Path(tmp))
        for pose in raw_doc["poses"]:
            if pose["name"] != raw_doc["return_pose"]["name"]:
                pose["hardware_validated"] = False
                pose["yaw_mapping_validated"] = False
        path = Path(tmp) / "unvalidated.json"
        path.write_text(json.dumps(raw_doc), encoding="utf-8")

        expect_config_error(lambda: scan.load_scan_config(str(path)),
                            "not hardware_validated",
                            "without the flag an unvalidated pose is refused")
        cfg = scan.load_scan_config(str(path), unlock_unvalidated=True)
        ok(len(cfg["poses"]) == 3, "with the flag the config loads")
        ok(not any(p["hardware_validated"] for p in cfg["poses"]
                   if p["name"] != cfg["return_pose"]["name"]),
           "...and it still RECORDS that the evidence is missing, not fakes it")

        # The waiver must be narrow. Every structural gate stays fatal.
        for label, mutate, text in (
            ("S1-only",
             lambda d: d["poses"][0]["arm_deg"].__setitem__(1, 80.0),
             "S2"),
            ("the yaw sign",
             lambda d: d["mapping"].__setitem__("yaw_sign", 1.0),
             "yaw_sign"),
            ("the pivot",
             lambda d: d["mapping"].__setitem__("pivot_xy_m", [0.2, 0.0]),
             "pivot"),
            ("the return pose",
             lambda d: d["return_pose"]["arm_deg"].__setitem__(0, 70.0),
             "return_pose"),
            ("the open jaw",
             lambda d: d["poses"][0]["arm_deg"].__setitem__(5, 90.0),
             "30"),
        ):
            doc2 = json.loads(path.read_text(encoding="utf-8"))
            mutate(doc2)
            bad = Path(tmp) / "bad.json"
            bad.write_text(json.dumps(doc2), encoding="utf-8")
            try:
                scan.load_scan_config(str(bad), unlock_unvalidated=True)
                ok(False, "the flag must NOT waive {}".format(label))
            except scan.ScanConfigError:
                ok(True, "the flag does not waive {}".format(label))

    # The waiver must appear in exactly the two conditionals it is for. Any
    # third `not unlock_unvalidated` would be a structural gate quietly opting
    # itself out of the check the operator thinks it is still getting.
    src = inspect.getsource(scan.load_scan_config)
    ok(src.count("not unlock_unvalidated") == 2,
       "the waiver appears in exactly 2 conditionals ({})".format(
           src.count("not unlock_unvalidated")))

    print("")
    print("14. the runtime scan path builds its pose from a REAL parsed config")
    # The calibration path does not call _dynamic_pose, so a scan-only fault
    # survived every offline suite and only appeared once the arm had already
    # travelled to LEFT and come back.
    import inspect as _inspect
    src = _inspect.getsource(scan._scan_one_pose)
    ok('config["mapping"]["homography"]' in src,
       "_scan_one_pose takes the homography from mapping, not from the pose")
    # Parsed, not grepped: the comment explaining the bug mentions the
    # subscript, and a text search cannot tell a comment from code.
    import ast as _ast
    import textwrap as _tw
    _tree = _ast.parse(_tw.dedent(_inspect.getsource(scan._dynamic_pose)))
    _subs = [n for n in _ast.walk(_tree)
             if isinstance(n, _ast.Subscript)
             and isinstance(n.value, _ast.Name) and n.value.id == "pose_spec"
             and isinstance(n.slice, _ast.Constant) and n.slice.value == "homography"]
    ok(not _subs,
       "_dynamic_pose no longer SUBSCRIPTS a key the parser never emits")
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "runtime.json"
        path.write_text(json.dumps(base_document(Path(tmp))), encoding="utf-8")
        cfg = scan.load_scan_config(str(path))
    import arm_cam_geometry as _acg
    for pose in cfg["poses"]:
        try:
            built = scan._dynamic_pose(_acg, pose, cfg["mapping"]["homography"])
            ok(tuple(built.arm_deg) == tuple(pose["arm_deg"]),
               "{} builds a stampable pose from the parsed config".format(
                   pose["name"]))
        except Exception as exc:
            ok(False, "{} raised {}: {}".format(
                pose["name"], type(exc).__name__, exc))

    print()
    if failures:
        print("{} of {} checks FAILED:".format(len(failures), checks))
        for label in failures:
            print("  - {}".format(label))
        return 1
    print("all {} checks passed — scan ownership, calibration, consensus, and "
          "fixed-target handoff stay fail-closed".format(checks))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
