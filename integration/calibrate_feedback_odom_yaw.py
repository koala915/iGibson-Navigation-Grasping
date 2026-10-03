#!/usr/bin/env python3
"""Calibrate left/right feedback-odometry yaw without a protractor.

Place a straight tape line on the floor along the robot centerline. Run two
complete turns, stop near the same line, then use short left/right trim pulses
until the chassis is exactly aligned again. The known physical angle is then
``turns * 2*pi`` while the program records the raw motion-feedback integral.

This program must be run interactively on the Jetson (for example in its
Jupyter terminal) so the operator beside the robot can press Enter immediately.
It only commands ``wz``; ``vx`` and ``vy`` are always zero.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SERIAL = "/dev/myserial"
DEFAULT_TURN_WZ = 0.40
DEFAULT_TRIM_WZ = 0.20
DEFAULT_TRIM_S = 0.12
DEFAULT_SAMPLE_HZ = 25.0
DEFAULT_MAX_CONTINUOUS_S = 40.0
DEFAULT_HOLDOUT_COARSE_WZ = 0.35
DEFAULT_HOLDOUT_FINE_WZ = 0.20
STOP_REPEAT = 12
STOP_INTERVAL_S = 0.03


def wrap_delta(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def calculate_scales(trials: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    by_direction: Dict[str, List[float]] = {"left": [], "right": []}
    details = []
    for trial in trials:
        direction = str(trial["direction"])
        if direction not in by_direction:
            raise ValueError("invalid direction %r" % direction)
        if trial.get("timed_out") or trial.get("interrupted"):
            raise ValueError("refusing incomplete %s trial" % direction)
        raw = float(trial["raw_wz_integral"])
        turns = int(trial["turns"])
        actual = (1.0 if direction == "left" else -1.0) * turns * 2.0 * math.pi
        if not math.isfinite(raw) or abs(raw) < 1e-6:
            raise ValueError("invalid raw yaw integral for %s" % direction)
        scale = actual / raw
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError(
                "%s trial implies scale %.6f; verify turn direction" %
                (direction, scale))
        by_direction[direction].append(scale)
        details.append({
            "direction": direction,
            "turns": turns,
            "actual_yaw_rad": actual,
            "raw_wz_integral": raw,
            "scale": scale,
            "imu_yaw_integral": trial.get("imu_yaw_integral"),
        })

    missing = [name for name, values in by_direction.items() if not values]
    if missing:
        raise ValueError("missing trial(s): %s" % ", ".join(missing))
    left = statistics.fmean(by_direction["left"])
    right = statistics.fmean(by_direction["right"])
    return {
        "angular_left_scale": left,
        "angular_right_scale": right,
        "left_right_difference_ratio": abs(left - right) / ((left + right) / 2.0),
        "trials": details,
    }


def _load_rosmaster(serial_port: str):
    grasp_dir = ROOT / "grasp"
    if str(grasp_dir) not in sys.path:
        sys.path.insert(0, str(grasp_dir))
    try:
        from Rosmaster_Lib import Rosmaster
    except ImportError as exc:
        raise RuntimeError(
            "Rosmaster_Lib is unavailable; use the deployed Jetson repo") from exc
    return Rosmaster(com=serial_port)


def _stop(device) -> None:
    for _ in range(STOP_REPEAT):
        try:
            device.set_car_motion(0.0, 0.0, 0.0)
        except Exception:
            pass
        time.sleep(STOP_INTERVAL_S)


class YawRecorder:
    def __init__(self, device) -> None:
        self.device = device
        self.started = time.monotonic()
        self.previous_t: Optional[float] = None
        self.previous_wz: Optional[float] = None
        self.previous_imu_yaw: Optional[float] = None
        self.raw_wz_integral = 0.0
        self.imu_yaw_integral = 0.0
        self.samples: List[Dict[str, float]] = []

    def sample(self) -> Tuple[float, float]:
        now = time.monotonic()
        motion = self.device.get_motion_data()
        if motion is None or len(motion) < 3:
            raise RuntimeError("malformed motion feedback: %r" % (motion,))
        raw_wz = float(motion[2])
        attitude = self.device.get_imu_attitude_data(ToAngle=False)
        if attitude is None or len(attitude) < 3:
            raise RuntimeError("malformed IMU attitude: %r" % (attitude,))
        imu_yaw = float(attitude[2])
        if not math.isfinite(raw_wz) or not math.isfinite(imu_yaw):
            raise RuntimeError("non-finite yaw feedback")

        if self.previous_t is not None and self.previous_wz is not None:
            dt = now - self.previous_t
            if 0.0 < dt < 0.5:
                self.raw_wz_integral += 0.5 * (self.previous_wz + raw_wz) * dt
        if self.previous_imu_yaw is not None:
            self.imu_yaw_integral += wrap_delta(imu_yaw - self.previous_imu_yaw)
        self.previous_t = now
        self.previous_wz = raw_wz
        self.previous_imu_yaw = imu_yaw
        self.samples.append({
            "t": now - self.started,
            "raw_wz": raw_wz,
            "imu_yaw": imu_yaw,
        })
        return raw_wz, imu_yaw


def _prepare_device(serial_port: str):
    device = _load_rosmaster(serial_port)
    device.create_receive_threading()
    device.set_auto_report_state(True, False)
    _stop(device)
    time.sleep(0.8)
    return device


def _drive_for(device, recorder: YawRecorder, command_wz: float,
               duration_s: float, sample_hz: float) -> None:
    period = 1.0 / sample_hz
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        tick = time.monotonic()
        device.set_car_motion(0.0, 0.0, command_wz)
        recorder.sample()
        time.sleep(max(0.0, period - (time.monotonic() - tick)))
    _stop(device)
    coast_deadline = time.monotonic() + 0.6
    while time.monotonic() < coast_deadline:
        recorder.sample()
        time.sleep(period)


def probe(serial_port: str, seconds: float = 5.0) -> Dict[str, Any]:
    device = _prepare_device(serial_port)
    recorder = YawRecorder(device)
    raw_values: List[float] = []
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            raw_wz, _imu = recorder.sample()
            raw_values.append(raw_wz)
            time.sleep(0.04)
    finally:
        _stop(device)
    result = {
        "samples": len(raw_values),
        "mean_abs_raw_wz": statistics.fmean(abs(v) for v in raw_values),
        "max_abs_raw_wz": max(abs(v) for v in raw_values),
        "imu_yaw_drift_rad": recorder.imu_yaw_integral,
        "imu_yaw_drift_deg": math.degrees(recorder.imu_yaw_integral),
    }
    print(json.dumps(result, indent=2))
    return result


def run_interactive(serial_port: str, direction: str, turns: int,
                    turn_wz: float, trim_wz: float, trim_s: float,
                    sample_hz: float, max_continuous_s: float,
                    output: Path) -> Dict[str, Any]:
    sign = 1.0 if direction == "left" else -1.0
    turn_wz = abs(float(turn_wz))
    trim_wz = abs(float(trim_wz))
    if not 0.20 <= turn_wz <= 0.80:
        raise ValueError("turn_wz must be between 0.20 and 0.80 rad/s")
    if not 0.10 <= trim_wz <= 0.30:
        raise ValueError("trim_wz must be between 0.10 and 0.30 rad/s")
    if not 0.05 <= trim_s <= 0.30:
        raise ValueError("trim_s must be between 0.05 and 0.30 s")
    if not 1 <= turns <= 3:
        raise ValueError("turns must be between 1 and 3")

    device = _prepare_device(serial_port)
    recorder = YawRecorder(device)
    interrupted = False
    timed_out = False
    aligned = threading.Event()

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True
        aligned.set()

    def wait_for_alignment() -> None:
        input(
            "Watch the chassis line. Press Enter near exactly %d full %s turn(s): "
            % (turns, direction))
        aligned.set()

    input(
        "Robot centered on the tape line, area clear, hand at power switch. "
        "Press Enter to START: ")
    old_sigint = signal.signal(signal.SIGINT, request_stop)
    old_sigterm = signal.signal(signal.SIGTERM, request_stop)
    input_thread = threading.Thread(target=wait_for_alignment, daemon=True)
    input_thread.start()
    command_wz = sign * turn_wz
    period = 1.0 / sample_hz
    continuous_started = time.monotonic()
    try:
        while not aligned.is_set():
            if time.monotonic() - continuous_started >= max_continuous_s:
                timed_out = True
                break
            tick = time.monotonic()
            device.set_car_motion(0.0, 0.0, command_wz)
            recorder.sample()
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
        _stop(device)
        for _ in range(max(1, int(0.8 * sample_hz))):
            recorder.sample()
            time.sleep(period)

        if not interrupted and not timed_out:
            print("\nStopped. Sight along the chassis and tape line.")
            print("Trim commands: l = short left, r = short right, done = aligned.")
            while True:
                command = input("trim [l/r/done]: ").strip().lower()
                if command in ("done", "d", ""):
                    break
                if command not in ("l", "r"):
                    print("Enter l, r, or done.")
                    continue
                trim_sign = 1.0 if command == "l" else -1.0
                _drive_for(device, recorder, trim_sign * trim_wz,
                           trim_s, sample_hz)
    finally:
        _stop(device)
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)

    actual_yaw = sign * turns * 2.0 * math.pi
    scale = actual_yaw / recorder.raw_wz_integral if abs(
        recorder.raw_wz_integral) > 1e-6 else None
    result = {
        "schema": "x3plus-feedback-odom-yaw-trial-v1",
        "direction": direction,
        "turns": turns,
        "actual_yaw_rad": actual_yaw,
        "command_wz_radps": command_wz,
        "raw_wz_integral": recorder.raw_wz_integral,
        "imu_yaw_integral": recorder.imu_yaw_integral,
        "scale": scale,
        "sample_count": len(recorder.samples),
        "timed_out": timed_out,
        "interrupted": interrupted,
        "samples": recorder.samples,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}, indent=2))
    print("saved: %s" % output)
    return result


def _configured_yaw_scale(direction: str) -> float:
    try:
        from feedback_odom import FeedbackOdomConfig
    except ImportError:
        from integration.feedback_odom import FeedbackOdomConfig
    cfg = FeedbackOdomConfig()
    return (cfg.angular_left_scale if direction == "left"
            else cfg.angular_right_scale)


def run_holdout(serial_port: str, direction: str, target_deg: float,
                coarse_wz: float, fine_wz: float, sample_hz: float,
                output: Path) -> Dict[str, Any]:
    """Turn until calibrated feedback odometry reaches a target angle."""
    if not 30.0 <= target_deg <= 180.0:
        raise ValueError("target_deg must be between 30 and 180")
    if not 0.25 <= coarse_wz <= 0.60:
        raise ValueError("coarse_wz must be between 0.25 and 0.60")
    if not 0.15 <= fine_wz <= 0.30:
        raise ValueError("fine_wz must be between 0.15 and 0.30")
    sign = 1.0 if direction == "left" else -1.0
    scale = _configured_yaw_scale(direction)
    target_rad = sign * math.radians(target_deg)
    fine_at_rad = math.radians(15.0)
    stop_at_rad = math.radians(1.0)
    timeout_s = max(12.0, abs(target_rad) / fine_wz * 2.0)

    device = _prepare_device(serial_port)
    recorder = YawRecorder(device)
    started = time.monotonic()
    interrupted = False
    timed_out = False

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    old_sigint = signal.signal(signal.SIGINT, request_stop)
    old_sigterm = signal.signal(signal.SIGTERM, request_stop)
    period = 1.0 / sample_hz
    try:
        while not interrupted:
            estimated = recorder.raw_wz_integral * scale
            remaining = abs(target_rad) - abs(estimated)
            if remaining <= stop_at_rad:
                break
            if time.monotonic() - started >= timeout_s:
                timed_out = True
                break
            command = sign * (coarse_wz if remaining > fine_at_rad else fine_wz)
            tick = time.monotonic()
            device.set_car_motion(0.0, 0.0, command)
            recorder.sample()
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
        _stop(device)
        coast_deadline = time.monotonic() + 1.0
        while time.monotonic() < coast_deadline:
            recorder.sample()
            time.sleep(period)
    finally:
        _stop(device)
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)

    final_odom_yaw = recorder.raw_wz_integral * scale
    result = {
        "schema": "x3plus-feedback-odom-yaw-holdout-v1",
        "direction": direction,
        "target_yaw_deg": sign * target_deg,
        "target_yaw_rad": target_rad,
        "configured_scale": scale,
        "raw_wz_integral": recorder.raw_wz_integral,
        "final_odom_yaw_rad": final_odom_yaw,
        "final_odom_yaw_deg": math.degrees(final_odom_yaw),
        "odom_target_error_deg": math.degrees(final_odom_yaw - target_rad),
        "imu_yaw_integral": recorder.imu_yaw_integral,
        "imu_yaw_integral_deg": math.degrees(recorder.imu_yaw_integral),
        "sample_count": len(recorder.samples),
        "timed_out": timed_out,
        "interrupted": interrupted,
        "samples": recorder.samples,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}, indent=2))
    print("saved: %s" % output)
    return result


def fit_files(paths: Sequence[Path]) -> Dict[str, Any]:
    trials = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
    result = calculate_scales(trials)
    print(json.dumps(result, indent=2))
    return result


def selftest() -> None:
    assert abs(wrap_delta(math.radians(179) - math.radians(-179))
               - math.radians(-2)) < 1e-12
    result = calculate_scales([
        {"direction": "left", "turns": 2,
         "raw_wz_integral": 8.0 * math.pi,
         "imu_yaw_integral": 4.0 * math.pi,
         "timed_out": False, "interrupted": False},
        {"direction": "right", "turns": 2,
         "raw_wz_integral": -10.0 * math.pi,
         "imu_yaw_integral": -4.0 * math.pi,
         "timed_out": False, "interrupted": False},
    ])
    assert abs(result["angular_left_scale"] - 0.5) < 1e-12
    assert abs(result["angular_right_scale"] - 0.4) < 1e-12
    print("calibrate_feedback_odom_yaw selftest: PASS")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--probe", action="store_true")
    mode.add_argument("--run", choices=("left", "right"))
    mode.add_argument("--holdout", choices=("left", "right"))
    mode.add_argument("--fit", action="store_true")
    mode.add_argument("--selftest", action="store_true")
    parser.add_argument("--serial", default=DEFAULT_SERIAL)
    parser.add_argument("--turns", type=int, default=2)
    parser.add_argument("--turn-wz", type=float, default=DEFAULT_TURN_WZ)
    parser.add_argument("--trim-wz", type=float, default=DEFAULT_TRIM_WZ)
    parser.add_argument("--trim-s", type=float, default=DEFAULT_TRIM_S)
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--max-continuous-s", type=float,
                        default=DEFAULT_MAX_CONTINUOUS_S)
    parser.add_argument("--target-deg", type=float, default=90.0)
    parser.add_argument("--coarse-wz", type=float, default=DEFAULT_HOLDOUT_COARSE_WZ)
    parser.add_argument("--fine-wz", type=float, default=DEFAULT_HOLDOUT_FINE_WZ)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--trial", action="append", type=Path, default=[])
    parser.add_argument("--real", action="store_true")
    parser.add_argument("--i-confirm-clear", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.selftest:
        selftest()
        return 0
    if args.fit:
        if len(args.trial) < 2:
            raise SystemExit("--fit requires at least one left and one right trial")
        fit_files(args.trial)
        return 0
    if args.probe:
        probe(args.serial)
        return 0
    if not args.real or not args.i_confirm_clear:
        raise SystemExit(
            "motion refused: pass --real and --i-confirm-clear only after "
            "clearing the full turn radius and standing at the power switch")
    if args.holdout:
        output = args.output or Path(
            "/tmp/odom_yaw_holdout_%s.json" % args.holdout)
        run_holdout(args.serial, args.holdout, args.target_deg,
                    args.coarse_wz, args.fine_wz, args.sample_hz, output)
        return 0
    output = args.output or Path("/tmp/odom_yaw_%s.json" % args.run)
    run_interactive(args.serial, args.run, args.turns, args.turn_wz,
                    args.trim_wz, args.trim_s, args.sample_hz,
                    args.max_continuous_s, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
