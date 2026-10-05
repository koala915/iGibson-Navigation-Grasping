#!/usr/bin/env python3
"""Navigation-only G2 client: ROS sensors in, TCP 7000 velocity out.

This never opens /dev/myserial.  The resident grasp-service remains the sole
board owner.  ``--probe`` is read-only; ``--real`` is a bounded, straight-line
odom test, NOT a map patrol or grasp launcher.  The 0.42 m forward brake was
measured at motor output 30 with the arm in its travel pose.  Do not generalise
that clearance to other speeds, arm poses, turning, or different obstacles.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

try:
    import nav_rl as nr
except ImportError:
    from . import nav_rl as nr


ROOT = Path(__file__).resolve().parent.parent
GRASPCTL = ROOT / "grasp" / "v23" / "graspctl.py"
SPEED_MPS = 0.15               # measured motor output 30
CONTROL_PERIOD_S = 0.05       # measured 15 cm stop used 20 Hz
STOP_LEAD_M = 0.06            # measured coast 0.050-0.066 m at motor 30
MAX_DISTANCE_M = 0.25         # deliberately short, supervised probe only
MAX_DRIVE_S = 2.5
ODOM_HARD_LIMIT_M = 0.28
SCAN_AGE_LIMIT_S = 0.35
ODOM_AGE_LIMIT_S = 0.20
STAMP_AGE_LIMIT_S = 0.40


@dataclasses.dataclass(frozen=True)
class Scan:
    points: tuple
    received_at: float
    stamp: float
    frame_id: str


@dataclasses.dataclass(frozen=True)
class Odom:
    x: float
    y: float
    yaw: float
    vx: float
    wz: float
    received_at: float
    stamp: float
    frame_id: str
    child_frame_id: str


def _stamp(message) -> float:
    stamp = message.get("header", {}).get("stamp", {})
    value = float(stamp["secs"]) + float(stamp["nsecs"]) * 1e-9
    if not math.isfinite(value) or value <= 0:
        raise ValueError("invalid ROS header stamp")
    return value


def parse_odom(message, *, now=None) -> Odom:
    pose = message["pose"]["pose"]
    pos = pose["position"]
    orient = pose["orientation"]
    twist = message["twist"]["twist"]
    x, y = float(pos["x"]), float(pos["y"])
    qx, qy, qz, qw = (float(orient[k]) for k in ("x", "y", "z", "w"))
    yaw = math.atan2(2 * (qw * qz + qx * qy),
                     1 - 2 * (qy * qy + qz * qz))
    vx = float(twist["linear"]["x"])
    wz = float(twist["angular"]["z"])
    if not all(math.isfinite(v) for v in (x, y, yaw, vx, wz, qx, qy, qz, qw)):
        raise ValueError("non-finite odom")
    return Odom(x, y, yaw, vx, wz, time.monotonic() if now is None else now,
                _stamp(message), str(message["header"]["frame_id"]),
                str(message["child_frame_id"]))


class RosNavSensors:
    """Latest-only subscriptions; no publisher and no access to serial."""

    def __init__(self, host="127.0.0.1", port=9090):
        import roslibpy
        self._lock = threading.Lock()
        self._scan: Optional[Scan] = None
        self._odom: Optional[Odom] = None
        self._scan_info = None
        self._scan_error = ""
        self._odom_error = ""
        self._ros = roslibpy.Ros(host=host, port=port)
        self._topics = []
        try:
            self._ros.run(timeout=5.0)
            if not self._ros.is_connected:
                raise ConnectionError("rosbridge did not connect")
            scan = roslibpy.Topic(self._ros, "/scan", "sensor_msgs/LaserScan",
                                  queue_length=1)
            odom = roslibpy.Topic(self._ros, "/odom_setmotor",
                                  "nav_msgs/Odometry", queue_length=1)
            self._topics = [scan, odom]
            scan.subscribe(self._on_scan)
            odom.subscribe(self._on_odom)
        except Exception:
            self.close()
            raise

    def _on_scan(self, msg):
        try:
            points = tuple(nr.laser_scan_to_points(msg))
            scan = Scan(points, time.monotonic(), _stamp(msg),
                        str(msg["header"]["frame_id"]))
            info = {"header": msg["header"], "angle_min": msg["angle_min"],
                    "angle_increment": msg["angle_increment"],
                    "ranges": [0.0] * len(msg["ranges"])}
        except (KeyError, TypeError, ValueError) as exc:
            with self._lock:
                self._scan_error = "invalid /scan: " + str(exc)
            return
        with self._lock:
            self._scan = scan
            if self._scan_info is None:
                self._scan_info = info
            self._scan_error = ""

    def _on_odom(self, msg):
        try:
            odom = parse_odom(msg)
        except (KeyError, TypeError, ValueError) as exc:
            with self._lock:
                self._odom_error = "invalid /odom_setmotor: " + str(exc)
            return
        with self._lock:
            self._odom = odom
            self._odom_error = ""

    def snapshot(self):
        with self._lock:
            return (self._scan, self._odom, self._scan_info,
                    self._scan_error or self._odom_error)

    def wait_ready(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            scan, odom, _, _ = self.snapshot()
            if scan is not None and odom is not None:
                return
            time.sleep(0.05)
        raise TimeoutError("/scan or /odom_setmotor did not arrive")

    def close(self):
        for topic in getattr(self, "_topics", ()):
            try:
                topic.unsubscribe()
            except Exception:
                pass
        if getattr(self, "_ros", None) is not None:
            try:
                self._ros.terminate()
            except Exception:
                pass


def health(scan: Optional[Scan], odom: Optional[Odom], cfg,
           *, now=None, wall_now=None) -> tuple:
    """Return (reason, front_x).  Any bad input fails closed."""
    now = time.monotonic() if now is None else now
    wall_now = time.time() if wall_now is None else wall_now
    if scan is None or odom is None:
        return "missing_sensor", float("inf")
    if scan.frame_id != "laser" or odom.frame_id != "odom" or \
            odom.child_frame_id != "base_footprint":
        return "wrong_frame", float("inf")
    if now - scan.received_at > SCAN_AGE_LIMIT_S or \
            now - odom.received_at > ODOM_AGE_LIMIT_S:
        return "stale_arrival", float("inf")
    if not (0 <= wall_now - scan.stamp <= STAMP_AGE_LIMIT_S and
            0 <= wall_now - odom.stamp <= STAMP_AGE_LIMIT_S):
        return "stale_stamp", float("inf")
    if not scan.points:
        return "empty_scan", float("inf")
    front = nr.front_min_brake(scan.points, cfg)
    if front < cfg.safety_brake_dist:
        return "lidar_brake", front
    return "", front


def forward_distance(start: Odom, current: Odom) -> float:
    dx, dy = current.x - start.x, current.y - start.y
    return dx * math.cos(start.yaw) + dy * math.sin(start.yaw)


def check_service_status(path=GRASPCTL):
    result = subprocess.run([sys.executable, str(path), "status"],
                            capture_output=True, text=True, timeout=6.0)
    if result.returncode:
        raise RuntimeError("graspctl status failed: " + result.stderr[-250:])
    status = json.loads(result.stdout)
    chassis = status.get("chassis") or {}
    odom = status.get("odom") or {}
    if (not status.get("ok") or chassis.get("moving") or
            chassis.get("arm_busy") or chassis.get("arm_pose") != "travel" or
            chassis.get("client") is not None or not odom.get("valid") or
            chassis.get("board_rx_age_s") is None or
            float(chassis["board_rx_age_s"]) > 0.2):
        raise RuntimeError("G2 service not in idle travel pose with valid feedback")
    motors = chassis.get("motors")
    if motors != [0, 0, 0, 0]:
        raise RuntimeError("G2 service reports nonzero motors")
    return status


def check_one_serial_owner():
    result = subprocess.run(["fuser", "/dev/myserial"],
                            capture_output=True, text=True, timeout=3.0)
    pids = [v for v in result.stdout.split() if v.isdigit()]
    if result.returncode != 0 or len(pids) != 1:
        raise RuntimeError("/dev/myserial must have exactly one owner; fuser: "
                           + (result.stdout + result.stderr)[-250:])
    service = subprocess.run(
        ["systemctl", "show", "-p", "MainPID", "--value", "grasp-service"],
        capture_output=True, text=True, timeout=3.0)
    if service.returncode != 0 or service.stdout.strip() != pids[0]:
        raise RuntimeError("/dev/myserial owner is not grasp-service MainPID")
    return pids[0]


class G2ChassisClient:
    """One TCP client; disconnect and watchdog are the server's final stops."""

    def __init__(self, host="127.0.0.1", port=7000):
        self._socket = socket.create_connection((host, port), timeout=1.0)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def velocity(self, vx: float, wz: float = 0.0):
        if not (math.isfinite(vx) and math.isfinite(wz)):
            raise ValueError("non-finite speed")
        if abs(vx) > SPEED_MPS or wz != 0.0:
            raise ValueError("this short straight probe permits only |vx|<=0.15, wz=0")
        self._socket.sendall((json.dumps({"action": "velocity", "vx": vx,
                                          "wz": wz}) + "\n").encode("ascii"))

    def stop(self):
        for _ in range(3):
            try:
                self._socket.sendall(b'{"action":"stop"}\n')
            except OSError:
                break
            time.sleep(0.03)

    def close(self):
        try:
            self.stop()
        finally:
            self._socket.close()


def run_straight(sensors, cfg, distance_m, *, host="127.0.0.1", port=7000):
    """Bounded motor-30 probe; callers must do physical and service preflight."""
    if not 0.10 <= distance_m <= MAX_DISTANCE_M:
        raise ValueError("distance_m must be 0.10..0.25 m")
    scan, start, _, error = sensors.snapshot()
    reason, initial_front = health(scan, start, cfg)
    if error or reason:
        raise RuntimeError("sensor preflight: " + (error or reason))
    if abs(start.vx) > 0.01 or abs(start.wz) > 0.01:
        raise RuntimeError("odom reports motion before start")
    client = G2ChassisClient(host, port)
    begun = last_progress = time.monotonic()
    last_distance = 0.0
    reason = "unknown"
    try:
        while True:
            now = time.monotonic()
            scan, odom, _, error = sensors.snapshot()
            sensor_reason, front = health(scan, odom, cfg)
            if error or sensor_reason:
                reason = error or sensor_reason
                break
            moved = forward_distance(start, odom)
            lateral = -(odom.x - start.x) * math.sin(start.yaw) + \
                      (odom.y - start.y) * math.cos(start.yaw)
            yaw_error = (odom.yaw - start.yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(lateral) > 0.05 or abs(yaw_error) > math.radians(15):
                reason = "straight_path_deviation"
                break
            if moved < -0.02 or moved - last_distance > 0.05:
                reason = "odom_jump"
                break
            if moved > last_distance + 0.002:
                last_distance = moved
                last_progress = now
            if now - begun > MAX_DRIVE_S or moved >= ODOM_HARD_LIMIT_M:
                reason = "hard_limit"
                break
            if moved >= max(0.0, distance_m - STOP_LEAD_M):
                reason = "distance_target"
                break
            if (now - begun > 0.8 and moved < 0.005) or \
                    now - last_progress > 0.6:
                reason = "no_odom_progress"
                break
            client.velocity(SPEED_MPS, 0.0)
            time.sleep(CONTROL_PERIOD_S)
    finally:
        client.close()
    time.sleep(1.0)
    scan, final, _, _ = sensors.snapshot()
    final_m = forward_distance(start, final) if final is not None else float("nan")
    print("reason=%s initial_front_m=%.3f front_at_end_m=%.3f "
          "final_odom_m=%.3f final_vx_mps=%.3f" %
          (reason, initial_front,
           nr.front_min_brake(scan.points, cfg) if scan else float("inf"),
           final_m, final.vx if final else float("nan")))
    return reason, final_m


def run_selftest():
    cfg = nr.NavRLConfig(lidar_yaw_offset_deg=180.0,
                         lidar_forward_offset_m=0.10,
                         safety_brake_dist=0.42)
    t, w = 100.0, 1000.0
    o = Odom(1, 2, 0, 0, 0, t, w, "odom", "base_footprint")
    clear = Scan(((180.0, 0.50),), t, w, "laser")
    assert health(clear, o, cfg, now=t, wall_now=w)[0] == ""
    board = Scan(tuple((180.0 + a, 0.31) for a in (-0.2, 0, 0.2)),
                 t, w, "laser")
    assert health(board, o, cfg, now=t, wall_now=w)[0] == "lidar_brake"
    assert health(dataclasses.replace(board, received_at=t-1), o, cfg,
                  now=t, wall_now=w)[0] == "stale_arrival"
    assert health(dataclasses.replace(board, stamp=w-1), o, cfg,
                  now=t, wall_now=w)[0] == "stale_stamp"
    assert health(dataclasses.replace(clear, frame_id="other"), o, cfg,
                  now=t, wall_now=w)[0] == "wrong_frame"
    assert health(None, o, cfg, now=t, wall_now=w)[0] == "missing_sensor"
    assert math.isclose(forward_distance(o, dataclasses.replace(o, x=1.15)), 0.15)
    assert math.isclose(forward_distance(dataclasses.replace(o, yaw=math.pi/2),
                                          dataclasses.replace(o, y=2.15)), 0.15)
    print("[g2-nav] selftest OK: brake, stale data, frames, odom projection")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--selftest", action="store_true")
    mode.add_argument("--probe", action="store_true", help="read-only ROS/G2 check")
    mode.add_argument("--real", action="store_true", help="bounded straight probe")
    p.add_argument("--distance-m", type=float, default=0.15)
    p.add_argument("--ros-host", default="127.0.0.1")
    p.add_argument("--ros-port", type=int, default=9090)
    p.add_argument("--chassis-host", default="127.0.0.1")
    p.add_argument("--chassis-port", type=int, default=7000)
    p.add_argument("--lidar-orientation-evidence", default="")
    p.add_argument("--i-confirm-clear-path", action="store_true")
    p.add_argument("--i-confirm-power-cut", action="store_true")
    args = p.parse_args()
    if args.selftest:
        run_selftest()
        return
    cfg = nr.NavRLConfig(lidar_yaw_offset_deg=180.0,
                         lidar_forward_offset_m=0.10,
                         safety_brake_dist=0.42)
    nr.validate_config(cfg)
    if args.real:
        if not (args.i_confirm_clear_path and args.i_confirm_power_cut):
            raise SystemExit("--real needs operator path and power-cut confirmations")
        if not 0.10 <= args.distance_m <= MAX_DISTANCE_M:
            raise SystemExit("--distance-m must be 0.10..0.25")
        if args.chassis_host not in ("127.0.0.1", "localhost"):
            raise SystemExit("real G2 client must run locally on the Jetson")
        if args.ros_host not in ("127.0.0.1", "localhost"):
            raise SystemExit("real ROS bridge must be local for stamp-age checks")
    check_service_status()
    if args.real:
        check_one_serial_owner()
    sensors = RosNavSensors(args.ros_host, args.ros_port)
    try:
        sensors.wait_ready()
        scan, odom, info, error = sensors.snapshot()
        reason, front = health(scan, odom, cfg)
        if error or (reason and not (args.probe and reason == "lidar_brake")):
            raise SystemExit("ROS sensor gate: " + (error or reason))
        details = nr.describe_scan(info, cfg)
        print("[g2-nav] scan_frame=%s front_x_m=%.3f odom=(%.3f,%.3f) "
              "vx=%.3f" % (scan.frame_id, front, odom.x, odom.y, odom.vx))
        for warning in details["warnings"]:
            print("[g2-nav] WARNING: " + warning)
        if args.probe:
            print("[g2-nav] read-only probe OK; brake=%s; no TCP motor command sent"
                  % (reason or "clear"))
            return
        verified, why = nr.validate_orientation_evidence(
            args.lidar_orientation_evidence, cfg, details)
        if not verified:
            raise SystemExit("LiDAR orientation evidence refused: " + why)
        if details["forward_fraction"] < 0.9:
            raise SystemExit("forward LiDAR coverage below 90%")
        stop_reason, final_m = run_straight(
            sensors, cfg, args.distance_m,
            host=args.chassis_host, port=args.chassis_port)
        if stop_reason != "distance_target":
            raise SystemExit("[g2-nav] motion stopped by safety gate: " + stop_reason)
        if not math.isfinite(final_m) or abs(final_m - args.distance_m) > 0.04:
            raise SystemExit("[g2-nav] distance outside 4 cm acceptance")
    finally:
        sensors.close()


if __name__ == "__main__":
    main()
