#!/usr/bin/env python3
"""Supervised, small-angle G2 turn probe; not a side-obstacle avoidance policy.

The resident grasp service remains the only /dev/myserial owner. This client
checks fresh ROS scan/odom, streams a fixed motor-25 turn through TCP 7000,
and sends stop on every exit path. Physical all-around clearance is required:
the robot's own LiDAR returns are not yet separated from side obstacles.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import time

try:
    from . import nav_rl as nr
    from .g2_nav_client import (RosNavSensors, check_one_serial_owner,
                                check_service_status, health)
except ImportError:
    import nav_rl as nr
    from g2_nav_client import (RosNavSensors, check_one_serial_owner,
                               check_service_status, health)


WZ_RADPS = 1.0                # maps to (-25,-25,+25,+25)
CONTROL_PERIOD_S = 0.05
STOP_AT_DEG = 6.0             # measured stop coasting may add a few degrees
HARD_YAW_DEG = 14.0
MAX_TURN_S = 1.0
MAX_TRANSLATION_M = 0.03
SIDE_LIMIT_M = 0.65           # centre distance; includes ~0.12 m half-width
SIDE_SELF_MASK_M = 0.35      # TG30 sees robot parts inside this band


def side_hazard(points, cfg) -> tuple:
    """Find persistent close side returns, not isolated TG30/self speckles.

    The inner 0.35 m is not trusted as obstacle sensing because the installed
    robot produces its own returns there. This gate supplements, rather than
    replaces, an operator's physical all-around clearance confirmation.
    """
    sides = {"left": [], "right": []}
    for angle_deg, distance in points:
        sample = nr._transform_scan_sample(angle_deg, distance, cfg)
        if sample is None:
            continue
        angle, distance = sample
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if (-0.35 < x < 0.35 and
                SIDE_SELF_MASK_M < abs(y) < SIDE_LIMIT_M):
            sides["left" if y > 0 else "right"].append((angle, x, y))

    found = []
    for side, points_on_side in sides.items():
        points_on_side.sort(key=lambda point: point[0])
        cluster = []
        for point in points_on_side:
            if cluster:
                prev = cluster[-1]
                if (point[0] - prev[0] > math.radians(1.0) or
                        math.hypot(point[1] - prev[1], point[2] - prev[2]) > 0.04):
                    if len(cluster) >= 3:
                        found.append((side, min(abs(p[2]) for p in cluster)))
                        break
                    cluster = []
            cluster.append(point)
        else:
            if len(cluster) >= 3:
                found.append((side, min(abs(p[2]) for p in cluster)))
    if found:
        return "+".join("side_" + side for side, _ in found), min(d for _, d in found)
    return "", float("inf")


def yaw_delta(start: float, current: float) -> float:
    return (current - start + math.pi) % (2.0 * math.pi) - math.pi


class TurnClient:
    def __init__(self, direction: int):
        self.sock = socket.create_connection(("127.0.0.1", 7000), timeout=1.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.direction = direction

    def velocity(self):
        self.sock.sendall((json.dumps({"action": "velocity", "vx": 0.0,
                                       "wz": self.direction * WZ_RADPS}) + "\n").encode("ascii"))

    def close(self):
        try:
            for _ in range(3):
                try:
                    self.sock.sendall(b'{"action":"stop"}\n')
                except OSError:
                    break
                time.sleep(0.03)
        finally:
            self.sock.close()  # disconnect is a second server-side stop


def turn_once(sensors: RosNavSensors, cfg, direction: int) -> tuple:
    scan, start, _, error = sensors.snapshot()
    why, front = health(scan, start, cfg)
    if error or why:
        raise RuntimeError("sensor preflight: " + (error or why))
    side, distance = side_hazard(scan.points, cfg)
    if side:
        raise RuntimeError("side preflight: %s at %.3f m" % (side, distance))
    if abs(start.vx) > 0.01 or abs(start.wz) > 0.02:
        raise RuntimeError("odom reports motion before turn")

    begun = time.monotonic()
    reason = "unknown"
    client = TurnClient(direction)
    try:
        while True:
            now = time.monotonic()
            scan, odom, _, error = sensors.snapshot()
            why, _ = health(scan, odom, cfg)
            if error or why:
                reason = error or why
                break
            side, _ = side_hazard(scan.points, cfg)
            if side:
                reason = side
                break
            angle = direction * math.degrees(yaw_delta(start.yaw, odom.yaw))
            drift = math.hypot(odom.x - start.x, odom.y - start.y)
            if angle < -1.5:
                reason = "wrong_yaw_direction"
                break
            if angle > HARD_YAW_DEG or drift > MAX_TRANSLATION_M:
                reason = "yaw_or_translation_limit"
                break
            if angle >= STOP_AT_DEG:
                reason = "angle_target"
                break
            if now - begun >= MAX_TURN_S:
                reason = "time_limit"
                break
            if now - begun >= 0.7 and angle < 0.5:
                reason = "no_odom_progress"
                break
            client.velocity()
            time.sleep(CONTROL_PERIOD_S)
    finally:
        client.close()

    time.sleep(0.7)
    _, final, _, _ = sensors.snapshot()
    if final is None:
        raise RuntimeError("no final odom")
    final_angle = direction * math.degrees(yaw_delta(start.yaw, final.yaw))
    final_drift = math.hypot(final.x - start.x, final.y - start.y)
    print("reason=%s initial_front_m=%.3f final_yaw_deg=%.2f "
          "final_drift_m=%.3f final_wz_radps=%.3f" %
          (reason, front, direction * final_angle, final_drift, final.wz))
    if abs(final.wz) > 0.05 or final_drift > MAX_TRANSLATION_M or \
            not -1.5 <= final_angle <= HARD_YAW_DEG:
        raise RuntimeError("final stopped pose outside limits")
    return reason, final_angle


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--selftest", action="store_true")
    mode.add_argument("--probe", action="store_true", help="read-only ROS check")
    mode.add_argument("--real", action="store_true", help="one supervised turn")
    p.add_argument("--direction", choices=("left", "right"), default="left")
    p.add_argument("--lidar-orientation-evidence", default="")
    p.add_argument("--i-confirm-all-around-clear", action="store_true")
    p.add_argument("--i-confirm-power-cut", action="store_true")
    args = p.parse_args()

    if args.selftest:
        assert abs(math.degrees(yaw_delta(math.radians(179),
                                          math.radians(-179))) - 2) < 1e-9
        assert abs(math.degrees(yaw_delta(0.0, math.radians(-2))) + 2) < 1e-9
        assert abs(-math.degrees(yaw_delta(0.0, math.radians(-2))) - 2) < 1e-9
        test_cfg = nr.NavRLConfig(lidar_yaw_offset_deg=0.0)
        assert side_hazard([(-90.0, 0.5)], test_cfg)[0] == ""
        assert side_hazard([(-90.0, 0.5), (-89.8, 0.5),
                            (-89.6, 0.5)], test_cfg)[0] == "side_right"
        print("[g2-turn] selftest OK: wrapped yaw, direction, side cluster")
        return
    if args.real and not (args.i_confirm_all_around_clear and
                          args.i_confirm_power_cut):
        raise SystemExit("--real needs physical clearance and power-cut confirmations")

    check_service_status()
    if args.real:
        check_one_serial_owner()
    cfg = nr.NavRLConfig(lidar_yaw_offset_deg=180.0,
                         lidar_forward_offset_m=0.10,
                         safety_brake_dist=0.42)
    nr.validate_config(cfg)
    sensors = RosNavSensors()
    try:
        sensors.wait_ready()
        scan, odom, info, error = sensors.snapshot()
        why, front = health(scan, odom, cfg)
        if error or why:
            raise SystemExit("ROS sensor gate: " + (error or why))
        side, side_m = side_hazard(scan.points, cfg)
        print("[g2-turn] read-only front_x_m=%.3f yaw_deg=%.2f wz=%.3f" %
              (front, math.degrees(odom.yaw), odom.wz))
        print("[g2-turn] side_gate=%s distance_m=%.3f" %
              (side or "clear_in_test_zone", side_m))
        if args.probe:
            print("[g2-turn] probe OK; no motor command sent")
            return
        if side:
            raise SystemExit("side obstacle gate: %s at %.3f m" % (side, side_m))
        verified, why = nr.validate_orientation_evidence(
            args.lidar_orientation_evidence, cfg, nr.describe_scan(info, cfg))
        if not verified:
            raise SystemExit("LiDAR orientation evidence refused: " + why)
        direction = 1 if args.direction == "left" else -1
        reason, _ = turn_once(sensors, cfg, direction)
        if reason != "angle_target":
            raise SystemExit("[g2-turn] stopped by safety/time gate: " + reason)
    finally:
        sensors.close()


if __name__ == "__main__":
    main()
