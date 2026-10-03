#!/usr/bin/env python3
"""Calibrate straight-line feedback odometry without changing yaw calibration.

The motion mode opens the Rosmaster serial port directly, commands only ``vx``,
and records the time integral of the board's raw ``get_motion_data()[0]``. Run
one forward and one backward trial, measure the actual signed displacement, then
use ``--fit`` to calculate the scale applied by ``FeedbackOdomConfig``.

Examples on the Jetson::

    python3 integration/calibrate_feedback_odom_linear.py --probe
    python3 integration/calibrate_feedback_odom_linear.py \
        --run forward --speed 0.10 --duration 4.0 --real --i-confirm-clear
    python3 integration/calibrate_feedback_odom_linear.py \
        --run backward --speed 0.10 --duration 4.0 --real --i-confirm-clear
    python3 integration/calibrate_feedback_odom_linear.py --fit \
        --trial /tmp/odom_linear_forward.json:0.41 \
        --trial /tmp/odom_linear_backward.json:-0.39

Only one process may own ``/dev/myserial``. Stop the grasp service, motor
server, and mission pipeline before using ``--probe`` or ``--run``.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SERIAL = "/dev/myserial"
DEFAULT_SPEED_MPS = 0.10
DEFAULT_DURATION_S = 4.0
DEFAULT_SAMPLE_HZ = 25.0
MAX_SPEED_MPS = 0.20
MAX_DURATION_S = 6.0
STOP_REPEAT = 12
STOP_INTERVAL_S = 0.03


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("%s must be finite" % name)
    return result


def fit_scale(trials: Sequence[Tuple[float, float]]) -> Dict[str, Any]:
    """Fit ``actual_distance = scale * raw_integral`` through the origin."""
    if len(trials) < 2:
        raise ValueError("at least two trials are required")
    cleaned: List[Tuple[float, float]] = []
    for raw_integral, actual_distance in trials:
        raw = _finite(raw_integral, "raw_integral")
        actual = _finite(actual_distance, "actual_distance")
        if abs(raw) < 1e-6:
            raise ValueError("raw integral is too close to zero")
        if abs(actual) < 1e-3:
            raise ValueError("actual distance is too close to zero")
        cleaned.append((raw, actual))

    individual = [actual / raw for raw, actual in cleaned]
    if any(value * individual[0] <= 0.0 for value in individual[1:]):
        raise ValueError(
            "trials imply conflicting scale signs; verify forward/backward "
            "direction and raw feedback")
    denominator = sum(raw * raw for raw, _actual in cleaned)
    scale = sum(raw * actual for raw, actual in cleaned) / denominator
    if abs(scale) < 1e-6:
        raise ValueError("fitted scale is too close to zero")
    residuals = [actual - scale * raw for raw, actual in cleaned]
    rmse = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    spread = max(individual) - min(individual)
    spread_ratio = spread / abs(scale) if scale else float("inf")
    return {
        "linear_scale": scale,
        "individual_scales": individual,
        "residuals_m": residuals,
        "rmse_m": rmse,
        "spread_ratio": spread_ratio,
        "warning": (
            "forward/backward scales differ by more than 10 percent"
            if spread_ratio > 0.10 else ""
        ),
        "feedback_orientation": (
            "raw vx matches ROS +x" if scale > 0.0
            else "raw vx is inverted relative to ROS +x"
        ),
    }


def _load_rosmaster(serial_port: str):
    grasp_dir = ROOT / "grasp"
    if str(grasp_dir) not in sys.path:
        sys.path.insert(0, str(grasp_dir))
    try:
        from Rosmaster_Lib import Rosmaster
    except ImportError as exc:
        raise RuntimeError(
            "Rosmaster_Lib is unavailable; run this from the deployed repo "
            "where grasp/Rosmaster_Lib is installed") from exc
    return Rosmaster(com=serial_port)


def _stop(device) -> None:
    for _ in range(STOP_REPEAT):
        try:
            device.set_car_motion(0.0, 0.0, 0.0)
        except Exception:
            pass
        time.sleep(STOP_INTERVAL_S)


def _motion(device) -> Tuple[float, float, float]:
    raw = device.get_motion_data()
    if raw is None or len(raw) < 3:
        raise RuntimeError("malformed motion feedback: %r" % (raw,))
    values = tuple(float(v) for v in raw[:3])
    if not all(math.isfinite(v) for v in values):
        raise RuntimeError("non-finite motion feedback: %r" % (values,))
    return values


def _encoders(device) -> Optional[List[int]]:
    try:
        values = device.get_motor_encoder()
        return [int(v) for v in values[:4]]
    except Exception:
        return None


def _prepare_device(serial_port: str):
    device = _load_rosmaster(serial_port)
    device.create_receive_threading()
    device.set_auto_report_state(True, False)
    _stop(device)
    time.sleep(0.8)
    return device


def probe(serial_port: str, seconds: float = 2.0) -> Dict[str, Any]:
    device = _prepare_device(serial_port)
    samples: List[Tuple[float, float, float]] = []
    encoder_start = _encoders(device)
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            samples.append(_motion(device))
            time.sleep(0.04)
        encoder_end = _encoders(device)
    finally:
        _stop(device)
        # Rosmaster_Lib's daemon receiver has no shutdown API. Explicitly
        # closing the fd while it blocks in read() creates a noisy traceback;
        # let process teardown close the descriptor after this function exits.
    result = {
        "samples": len(samples),
        "mean_abs_vx": statistics.fmean(abs(v[0]) for v in samples),
        "mean_abs_vy": statistics.fmean(abs(v[1]) for v in samples),
        "mean_abs_wz": statistics.fmean(abs(v[2]) for v in samples),
        "encoder_start": encoder_start,
        "encoder_end": encoder_end,
    }
    print(json.dumps(result, indent=2))
    return result


def run_trial(serial_port: str, direction: str, speed: float, duration: float,
              sample_hz: float, output: Path) -> Dict[str, Any]:
    speed = _finite(speed, "speed")
    duration = _finite(duration, "duration")
    sample_hz = _finite(sample_hz, "sample_hz")
    if not 0.05 <= speed <= MAX_SPEED_MPS:
        raise ValueError("speed must be between 0.05 and %.2f m/s" % MAX_SPEED_MPS)
    if not 0.5 <= duration <= MAX_DURATION_S:
        raise ValueError("duration must be between 0.5 and %.1f s" % MAX_DURATION_S)
    if not 10.0 <= sample_hz <= 50.0:
        raise ValueError("sample_hz must be between 10 and 50")

    sign = 1.0 if direction == "forward" else -1.0
    command_vx = sign * speed
    period = 1.0 / sample_hz
    device = _prepare_device(serial_port)
    encoder_start = _encoders(device)
    samples: List[Dict[str, float]] = []
    raw_integral = 0.0
    previous_t: Optional[float] = None
    previous_vx: Optional[float] = None
    interrupted = False

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    old_sigint = signal.signal(signal.SIGINT, request_stop)
    old_sigterm = signal.signal(signal.SIGTERM, request_stop)
    started = time.monotonic()
    try:
        while not interrupted:
            now = time.monotonic()
            elapsed = now - started
            if elapsed >= duration:
                break
            device.set_car_motion(command_vx, 0.0, 0.0)
            vx, vy, wz = _motion(device)
            if previous_t is not None and previous_vx is not None:
                raw_integral += 0.5 * (previous_vx + vx) * (now - previous_t)
            samples.append({"t": elapsed, "vx": vx, "vy": vy, "wz": wz})
            previous_t, previous_vx = now, vx
            time.sleep(max(0.0, period - (time.monotonic() - now)))

        _stop(device)
        coast_deadline = time.monotonic() + 1.0
        while time.monotonic() < coast_deadline:
            now = time.monotonic()
            vx, vy, wz = _motion(device)
            if previous_t is not None and previous_vx is not None:
                raw_integral += 0.5 * (previous_vx + vx) * (now - previous_t)
            samples.append({
                "t": now - started, "vx": vx, "vy": vy, "wz": wz,
            })
            previous_t, previous_vx = now, vx
            time.sleep(period)
    finally:
        _stop(device)
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)

    encoder_end = _encoders(device)
    result = {
        "schema": "x3plus-feedback-odom-linear-trial-v1",
        "direction": direction,
        "serial_port": serial_port,
        "command_vx_mps": command_vx,
        "command_duration_s": duration,
        "sample_hz": sample_hz,
        "sample_count": len(samples),
        "raw_vx_integral": raw_integral,
        "raw_vx_min": min(s["vx"] for s in samples),
        "raw_vx_max": max(s["vx"] for s in samples),
        "max_abs_vy": max(abs(s["vy"]) for s in samples),
        "max_abs_wz": max(abs(s["wz"]) for s in samples),
        "encoder_start": encoder_start,
        "encoder_end": encoder_end,
        "encoder_delta": (
            [end - start for start, end in zip(encoder_start, encoder_end)]
            if encoder_start is not None and encoder_end is not None else None
        ),
        "interrupted": interrupted,
        "samples": samples,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}, indent=2))
    print("saved: %s" % output)
    return result


def fit_from_args(entries: Sequence[str]) -> Dict[str, Any]:
    trials: List[Tuple[float, float]] = []
    sources = []
    for entry in entries:
        try:
            path_text, actual_text = entry.rsplit(":", 1)
        except ValueError as exc:
            raise ValueError("trial must be FILE:SIGNED_DISTANCE_M") from exc
        path = Path(path_text)
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("interrupted"):
            raise ValueError("refusing interrupted trial: %s" % path)
        raw = float(data["raw_vx_integral"])
        actual = float(actual_text)
        direction = str(data.get("direction", ""))
        expected_sign = 1.0 if direction == "forward" else -1.0
        if actual * expected_sign <= 0.0:
            raise ValueError(
                "%s needs a %s signed distance" %
                (path, "positive" if expected_sign > 0 else "negative"))
        trials.append((raw, actual))
        sources.append({"file": str(path), "raw_integral": raw,
                        "actual_distance_m": actual})
    result = fit_scale(trials)
    result["trials"] = sources
    print(json.dumps(result, indent=2))
    return result


def selftest() -> None:
    result = fit_scale([(0.50, 0.40), (-0.25, -0.20)])
    assert abs(result["linear_scale"] - 0.8) < 1e-12
    assert result["warning"] == ""
    inverted = fit_scale([(-0.50, 0.40), (0.25, -0.20)])
    assert abs(inverted["linear_scale"] + 0.8) < 1e-12
    assert inverted["feedback_orientation"].startswith("raw vx is inverted")
    try:
        fit_scale([(0.5, 0.4), (-0.5, 0.4)])
    except ValueError:
        pass
    else:
        raise AssertionError("sign mismatch was not rejected")
    print("calibrate_feedback_odom_linear selftest: PASS")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--probe", action="store_true")
    mode.add_argument("--run", choices=("forward", "backward"))
    mode.add_argument("--fit", action="store_true")
    mode.add_argument("--selftest", action="store_true")
    parser.add_argument("--serial", default=DEFAULT_SERIAL)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MPS)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--trial", action="append", default=[],
                        metavar="FILE:SIGNED_DISTANCE_M")
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
            raise SystemExit("--fit requires at least two --trial entries")
        fit_from_args(args.trial)
        return 0
    if args.probe:
        probe(args.serial)
        return 0
    if not args.real or not args.i_confirm_clear:
        raise SystemExit(
            "motion refused: pass both --real and --i-confirm-clear only after "
            "the area is clear and an operator is at the power switch")
    output = args.output or Path(
        "/tmp/odom_linear_%s.json" % args.run)
    run_trial(args.serial, args.run, args.speed, args.duration,
              args.sample_hz, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
