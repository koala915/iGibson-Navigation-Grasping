#!/usr/bin/env python3
"""Run the validated v21 arm-camera grasp entirely on the Jetson.

This launcher owns the process ordering that was previously done by hand on
Windows and the Jetson:

1. start the real-arm controller and wait until the C3 home pose is confirmed;
2. run one local YOLO inference from the arm camera and send it to localhost;
3. release the YOLO process after the detection is latched, then let PPO grasp;
4. forward Ctrl+C and clean up both children on every exit path.

YOLO deliberately uses ``--once``.  The arm-camera geometry is valid only at
the C3 home pose, and freeing YOLO before policy motion also matters on a Jetson
Nano with limited RAM.

At C3 the bridge maps pixels to base coordinates through a MEASURED homography
and rejects everything else -- the old nav-home H/theta/cam offsets included, and
``--i-accept-predicted-extrinsics`` cannot waive it.  So this launcher passes
``--homography`` and validates that file with the bridge's own gate BEFORE the
serial port is opened: the alternative is what used to happen, where the arm
powered up, moved to C3 home, and only then did the bridge exit and take the run
down with it.

``--calibrate`` is the other half of the same problem.  The calibration has to be
measured at C3 home, which means something has to hold the arm there while the
pixels are collected; that something is this launcher.  See
docs/calibration/CALIBRATION_PLAN.md "grasp-home homography".

This file is deliberately kept parseable by the Jetson's SYSTEM python3 (3.6.9):
no ``from __future__ import annotations``, no walrus, no PEP 585 generics.  It is
the command people type by hand, so running it under the wrong interpreter has to
report itself rather than die with a SyntaxError in the import block.  Everything
it launches runs under the 3.8 venv and may use all of them.
"""

import argparse
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import IO, Optional


if sys.version_info < (3, 8):
    sys.exit(
        "[FATAL] 需要 Python >= 3.8，目前是 {}（Jetson 的系統 python3 是 3.6.9）。\n"
        "        先啟用虛擬環境再重跑：source ~/grasp_venv/bin/activate".format(
            sys.version.split()[0]))
if "VIRTUAL_ENV" not in os.environ:
    # Warning, not an error: what actually matters is whether the deps import,
    # and preflight() checks exactly that a few lines below.
    print("[WARN] 沒有偵測到 VIRTUAL_ENV。若下面的 import 檢查失敗，"
          "先執行 source ~/grasp_venv/bin/activate")


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
BRIDGE = REPO_ROOT / "integration" / "vision_grasp_bridge.py"
CONTROLLER = HERE / "x3plus_real_grasp.py"
PPO_MODEL = HERE / "models" / "candidate_v21_seed816_ckpt550000.zip"
VECNORM = HERE / "models" / "candidate_v21_seed816_ckpt550000_vec.pkl"
YOLO_MODEL = REPO_ROOT / "detection" / "models" / "best.pt"
INTEGRATION = REPO_ROOT / "integration"
HOMOGRAPHY = INTEGRATION / "grasp_home_homography.json"
ARM_CAMERA = ("/dev/v4l/by-id/"
              "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0")

# The runtime gate the bridge itself applies (vision_grasp_bridge.resolve_pose).
# Duplicated here on purpose: checking it earlier is the whole point, and a
# looser check here would pass a file the bridge then rejects at C3.
HOMOGRAPHY_MIN_POINTS = 6
HOMOGRAPHY_MAX_ERROR_M = 0.02


def parse_args_from(argv) -> argparse.Namespace:
    """Parse an explicit argv. Separate from parse_args() so the command builders
    below can be exercised with the real defaults, without a hardware run."""
    p = argparse.ArgumentParser(
        description="One-command, Jetson-local v21 vision grasp")
    p.add_argument("--camera", default=ARM_CAMERA,
                   help="stable local arm-camera path (defaults to the non-SN0001 "
                        "Sonix camera, currently /dev/video1)")
    p.add_argument("--port", default="/dev/myserial",
                   help="Rosmaster serial port (default /dev/myserial)")
    # --cam-x/--cam-y/--sign-y are gone, not defaulted: at C3 the bridge maps
    # through the homography alone, so forwarding them would print numbers that
    # look like they steer the arm while changing nothing.
    p.add_argument("--homography", default=str(HOMOGRAPHY),
                   help="measured grasp-home pixel->base calibration JSON "
                        "(default integration/grasp_home_homography.json). "
                        "Produce it with --calibrate; the bridge refuses C3 "
                        "detections without one.")
    p.add_argument("--calibrate", action="store_true",
                   help="hold the arm at C3 home and print median undistorted "
                        "pixels instead of grasping. Nothing is sent to the arm "
                        "and no grasp is attempted.")
    p.add_argument("--calibration-samples", type=int, default=20,
                   help="valid frames per printed median in --calibrate (default 20)")
    p.add_argument("--show", action="store_true",
                   help="show the annotated camera window (needs a display; not "
                        "over a plain SSH session)")
    p.add_argument("--class-height", type=float, default=0.065,
                   help="sugarbox full height in metres (default 0.065)")
    p.add_argument("--entry-xy-mm", type=float, default=10.0)
    p.add_argument("--s6-stall-steps", type=int, default=2)
    p.add_argument("--pose-tol-deg", type=float, default=3.0,
                   help="maximum C3 arm-pose residual accepted by both startup and "
                        "the detection stamp (default 3.0 deg; controller caps "
                        "startup recovery at 3.0 deg)")
    p.add_argument("--latch-wait", type=float, default=120.0)
    p.add_argument("--stale-timeout", type=float, default=1.0)
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--startup-timeout", type=float, default=180.0,
                   help="seconds allowed for model load and confirmed C3 home")
    p.add_argument("--check", action="store_true",
                   help="check files/imports only; do not open hardware or move")
    return p.parse_args(argv)


def parse_args() -> argparse.Namespace:
    return parse_args_from(None)


def check_homography(path_str: str) -> bool:
    """Validate the calibration with the SAME gate the bridge applies at C3.

    Runs before any hardware is opened. A missing or weak calibration is the one
    failure that used to cost a full startup: model load, serial port, arm to C3
    home, and only then the bridge exiting mid-run.
    """
    path = Path(path_str)
    if not path.is_file():
        print(f"[FATAL] missing grasp-home homography: {path}")
        print("        C3 偵測一定要有實測校正檔，nav-home 的 H/theta/offset 不能替代。")
        print("        產生方式：本檔加 --calibrate（詳見下方說明與")
        print("        docs/calibration/CALIBRATION_PLAN.md「grasp-home homography」）")
        return False

    sys.path.insert(0, str(INTEGRATION))
    try:
        from grasp_home_homography import (HomographyCalibrationError,
                                           load_calibration)
    except ImportError as exc:
        print(f"[FATAL] cannot import grasp_home_homography: {exc}")
        return False

    try:
        document = load_calibration(str(path), min_points=HOMOGRAPHY_MIN_POINTS,
                                    max_error_m=HOMOGRAPHY_MAX_ERROR_M)
    except HomographyCalibrationError as exc:
        print(f"[FATAL] grasp-home homography rejected: {exc}")
        print(f"        校正檔要過 runtime 的同一道閘：>= {HOMOGRAPHY_MIN_POINTS} 個不共線的點、"
              f"最大誤差 < {HOMOGRAPHY_MAX_ERROR_M*100:.0f} cm、內容與擬合結果一致。")
        print("        用 --calibrate 重新量測，再用 "
              "integration/grasp_home_homography.py 重解。")
        return False

    fit = document["fit"]
    print(f"[check] homography: {path} ({fit['point_count']} points, "
          f"RMSE {fit['rmse_m']*100:.2f} cm, max {fit['max_error_m']*100:.2f} cm)")
    return True


def preflight(args: argparse.Namespace) -> bool:
    ok = True
    for label, path in (
            ("controller", CONTROLLER), ("vision bridge", BRIDGE),
            ("PPO model", PPO_MODEL), ("VecNormalize", VECNORM),
            ("YOLO model", YOLO_MODEL)):
        if path.is_file():
            print(f"[check] {label}: {path}")
        else:
            print(f"[FATAL] missing {label}: {path}")
            ok = False

    for module in ("cv2", "ultralytics", "stable_baselines3", "pybullet"):
        if importlib.util.find_spec(module) is None:
            print(f"[FATAL] Python module {module!r} is not installed in {sys.executable}")
            ok = False
        else:
            print(f"[check] import {module}: OK")

    camera = str(args.camera)
    if camera.startswith("/dev/") and not Path(camera).exists():
        print(f"[FATAL] camera device does not exist: {camera}")
        ok = False
    elif camera.startswith("/dev/"):
        # This launcher opens the camera itself; there is no separate streaming
        # terminal to start. What there can be is a leftover one still holding
        # the device, and V4L2 hands out no second reader.
        holders = camera_holders(camera)
        if holders:
            print(f"[FATAL] camera {camera} is already open in another process:")
            for pid, cmdline in holders:
                print(f"          pid {pid}: {cmdline}")
            print("        關掉那個終端機／程序後重跑（本檔會自己讀相機，不需要另外開串流）。")
            ok = False
        else:
            print(f"[check] camera: {camera} (free; this launcher reads it directly)")

    if not Path(args.port).exists():
        print(f"[FATAL] Rosmaster port does not exist: {args.port}")
        ok = False
    else:
        print(f"[check] Rosmaster port: {args.port}")

    # Skipped under --calibrate for the obvious reason: that mode exists to
    # create this file, so requiring it first would be a closed loop.
    if not args.calibrate and not check_homography(args.homography):
        ok = False
    return ok


def pump(proc: subprocess.Popen, log: IO[str], *,
         home_ready: Optional[threading.Event] = None,
         detection_sent: Optional[threading.Event] = None):
    assert proc.stdout is not None
    for line in proc.stdout:
        print(line, end="", flush=True)
        log.write(line)
        log.flush()
        if home_ready is not None and "[Start] home: reached=True" in line:
            home_ready.set()
        if detection_sent is not None and line.startswith("[bridge] sent "):
            detection_sent.set()


def stop_process(proc: Optional[subprocess.Popen], label: str) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[launcher] stopping {label} (SIGINT)")
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


def build_ctrl_cmd(args: argparse.Namespace):
    """Arm-controller command line."""
    # The controller holds the arm at C3 by waiting for a detection that
    # --calibrate never sends, so that wait has to outlast a measuring session
    # rather than a one-shot detection.
    latch_wait = max(args.latch_wait, 1800.0) if args.calibrate else args.latch_wait
    return [
        sys.executable, str(CONTROLLER),
        "--real", "--socket", "--latch-obj",
        "--i-confirm-external-frame", "--unlock-candidate-real",
        "--port", str(args.port),
        "--model", str(PPO_MODEL),
        "--vecnorm", str(VECNORM),
        "--contract", "obs_28_incremental",
        "--pose-tol-deg", str(args.pose_tol_deg),
        "--entry-xy-mm", str(args.entry_xy_mm),
        "--s6-stall-grasp-steps", str(args.s6_stall_steps),
        "--latch-wait", str(latch_wait),
        "--stale-timeout", str(args.stale_timeout),
        "--max-steps", str(args.max_steps),
    ]


def build_bridge_cmd(args: argparse.Namespace):
    """Vision-bridge command line.

    Kept separate from main() so the flag wiring is testable without hardware.
    The bug this guards against already happened once: the launcher went on
    passing nav-home extrinsics after the bridge started requiring a measured
    homography at C3, and the mismatch only surfaced on the robot, after the arm
    had powered up and moved.
    """
    cmd = [
        sys.executable, str(BRIDGE),
        "--host", "127.0.0.1", "--port", "5555",
        "--model", str(YOLO_MODEL),
        "--stream", str(args.camera),
        "--class-height", f"sugarbox={args.class_height}",
    ]
    if args.show:
        cmd.append("--show")
    if args.calibrate:
        # --calibration-only never opens the TCP socket, so the controller sits
        # at C3 untouched for the whole session.
        cmd += ["--dry-run", "--calibration-only",
                "--calibration-samples", str(args.calibration_samples)]
    else:
        cmd += ["--homography", str(args.homography), "--once"]
    return cmd


def print_calibration_next_steps(args: argparse.Namespace) -> None:
    print("")
    print("[launcher] 接下來：把每組 (u, v) 與實測 base (x, y) 寫成 JSON，例如")
    print('             [{"u": 318.4, "v": 402.1, "x": 0.247, "y": 0.018}, ...]')
    print("           留至少 2 組不要拿去擬合，之後當驗證點。然後：")
    print(f"             python3 {INTEGRATION / 'grasp_home_homography.py'} \\")
    print("               --points-json grasp_home_points.json \\")
    print(f"               --output {args.homography} --max-rmse-cm 1")
    print("           產生後直接跑 `python3 jetson_one_command_grasp.py --check` 驗收。")


def camera_holders(device: str):
    """(pid, cmdline) of other processes holding the camera, best effort.

    Without this, a leftover stream_cam.py in another terminal surfaces as a
    generic OpenCV open failure, which reads like a broken camera rather than
    "something else already has it".
    """
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return []
    target = os.path.realpath(device)
    mine = os.getpid()
    holders = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == mine:
            continue
        try:
            fds = list((entry / "fd").iterdir())
        except OSError:
            continue          # not ours, or the process just exited
        for fd in fds:
            try:
                if os.path.realpath(str(fd)) != target:
                    continue
                raw = (entry / "cmdline").read_bytes()
            except OSError:
                continue
            cmdline = raw.decode("utf-8", "replace").replace("\0", " ").strip()
            holders.append((entry.name, cmdline or "?"))
            break
    return holders


def main() -> int:
    args = parse_args()
    if not preflight(args):
        return 2
    if args.check:
        print("[check] PASS: files and Python dependencies are present; no hardware opened.")
        return 0

    stamp = time.strftime("%Y%m%d_%H%M%S")
    ctrl_log_path = Path.home() / f"mode_b_integrated_{stamp}_controller.log"
    vision_log_path = Path.home() / f"mode_b_integrated_{stamp}_vision.log"
    if args.calibrate:
        print("[launcher] CALIBRATE: 手臂只會移到 C3 home 並停在那裡，不會夾任何東西。")
    else:
        print("[launcher] REAL supervised grasp: stay beside the robot, hand on power.")
    print(f"[launcher] controller log: {ctrl_log_path}")
    print(f"[launcher] vision log    : {vision_log_path}")
    print("[launcher] 相機由本程序自己開，不需要另外開串流終端機。")

    ctrl_cmd = build_ctrl_cmd(args)
    bridge_cmd = build_bridge_cmd(args)

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    ctrl = None
    bridge = None
    ctrl_thread = None
    bridge_thread = None
    home_ready = threading.Event()
    detection_sent = threading.Event()
    ctrl_log = open(ctrl_log_path, "w", encoding="utf-8")
    vision_log = open(vision_log_path, "w", encoding="utf-8")
    try:
        ctrl = subprocess.Popen(
            ctrl_cmd, cwd=str(HERE), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8",
            errors="replace", bufsize=1)
        ctrl_thread = threading.Thread(
            target=pump, args=(ctrl, ctrl_log),
            kwargs={"home_ready": home_ready}, daemon=True)
        ctrl_thread.start()

        deadline = time.monotonic() + max(1.0, args.startup_timeout)
        while not home_ready.wait(0.1):
            if ctrl.poll() is not None:
                print(f"[FATAL] controller exited before confirming C3 home (exit {ctrl.returncode}).")
                return ctrl.returncode or 2
            if time.monotonic() >= deadline:
                print("[FATAL] timed out waiting for the controller to confirm C3 home.")
                return 2

        if args.calibrate:
            print("[launcher] C3 home confirmed; starting the calibration camera pass.")
            print("[launcher] 每擺一個位置，等下面印出一行 [bridge][calibration] {...}，")
            print("[launcher] 把裡面的 u,v 和你量到的 base 座標 x,y 記成一組。")
            print(f"[launcher] 至少 {HOMOGRAPHY_MIN_POINTS} 組不共線的點，另外多留 2 組"
                  "當驗證點。量完按 Ctrl+C。")
        else:
            print("[launcher] C3 home confirmed; starting one local YOLO detection.")
        bridge = subprocess.Popen(
            bridge_cmd, cwd=str(REPO_ROOT), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8",
            errors="replace", bufsize=1)
        bridge_thread = threading.Thread(
            target=pump, args=(bridge, vision_log),
            kwargs={"detection_sent": detection_sent}, daemon=True)
        bridge_thread.start()

        while ctrl.poll() is None:
            if bridge.poll() is not None:
                # Let the stdout pump consume the final "sent" line before deciding
                # that a clean --once exit produced no detection.
                if bridge_thread is not None:
                    bridge_thread.join(timeout=2.0)
                if bridge.returncode not in (None, 0):
                    print(f"[FATAL] vision bridge exited with {bridge.returncode}; "
                          "stopping arm controller.")
                    stop_process(ctrl, "controller")
                    return bridge.returncode or 2
                if args.calibrate:
                    # Nothing is ever "sent" in this mode, so the detection check
                    # below would read a finished calibration as a failure.
                    print("[launcher] calibration pass finished; stopping the arm.")
                    print_calibration_next_steps(args)
                    stop_process(ctrl, "controller")
                    return 0
                if not detection_sent.is_set():
                    print("[FATAL] vision bridge exited without sending a valid "
                          "detection; stopping arm controller.")
                    stop_process(ctrl, "controller")
                    return 2
            time.sleep(0.1)
        return int(ctrl.returncode or 0)
    except KeyboardInterrupt:
        print("\n[launcher] Ctrl+C received; stopping vision and controller.")
        if args.calibrate:
            # The expected way to end a measuring session, not an error.
            print_calibration_next_steps(args)
        return 130
    finally:
        stop_process(bridge, "vision bridge")
        stop_process(ctrl, "controller")
        if bridge_thread is not None:
            bridge_thread.join(timeout=2.0)
        if ctrl_thread is not None:
            ctrl_thread.join(timeout=2.0)
        vision_log.close()
        ctrl_log.close()


if __name__ == "__main__":
    raise SystemExit(main())
