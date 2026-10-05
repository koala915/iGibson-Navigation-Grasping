#!/usr/bin/env python3
"""Supervised 360-degree mapping spin through the resident TCP chassis owner."""
import argparse
import json
import math
import socket
import time

import nav_rl as nr
from g2_nav_client import (RosNavSensors, check_one_serial_owner,
                           check_service_status)

CONTROL_PERIOD_S = 0.05
COMMAND_WZ = 1.20
SEGMENT_DEG = 30.0
STOP_LEAD_DEG = 3.0
TOTAL_DEG = 360.0
HARD_TOTAL_DEG = 375.0
MAX_TRANSLATION_M = 0.08
MAX_TIME_S = 40.0
# Chassis corners are about 0.192 m from base centre. This preserves >10 cm
# beyond that measured envelope while considering every scan direction.
SELF_MASK_M = 0.27
HARD_CLEARANCE_M = 0.30
SCAN_AGE_LIMIT_S = 0.35
ODOM_AGE_LIMIT_S = 0.20
STAMP_AGE_LIMIT_S = 0.40


def wrapped_delta(previous, current):
    return (current - previous + math.pi) % (2 * math.pi) - math.pi


def swept_clearance(points, cfg):
    distances = []
    for source_angle, distance in points:
        sample = nr._transform_scan_sample(source_angle, distance, cfg)
        if sample is None:
            continue
        angle, distance = sample
        # _transform_scan_sample already applies the LiDAR forward offset.
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        radius = math.hypot(x, y)
        if radius > SELF_MASK_M:
            distances.append(radius)
    distances.sort()
    if not distances:
        return float("inf"), float("inf"), 0
    p01 = distances[max(0, int(len(distances) * 0.01) - 1)]
    return distances[0], p01, len(distances)


def sensor_gate(scan, odom, cfg):
    now, wall = time.monotonic(), time.time()
    if scan is None or odom is None:
        return "missing_sensor", None
    if scan.frame_id != "laser" or odom.frame_id != "odom" or odom.child_frame_id != "base_footprint":
        return "wrong_frame", None
    if now - scan.received_at > SCAN_AGE_LIMIT_S or now - odom.received_at > ODOM_AGE_LIMIT_S:
        return "stale_arrival", None
    if not (0 <= wall - scan.stamp <= STAMP_AGE_LIMIT_S and
            0 <= wall - odom.stamp <= STAMP_AGE_LIMIT_S):
        return "stale_stamp", None
    minimum, p01, count = swept_clearance(scan.points, cfg)
    # Use a small cluster percentile for the hard stop so a single speckle does
    # not masquerade as a wall; still report the absolute minimum.
    if p01 < HARD_CLEARANCE_M:
        return "swept_clearance", (minimum, p01, count)
    return "", (minimum, p01, count)


class SpinClient:
    def __init__(self):
        self.sock = socket.create_connection(("127.0.0.1", 7000), timeout=1.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def turn(self):
        self.sock.sendall((json.dumps({"action": "velocity", "vx": 0.0,
                                       "wz": COMMAND_WZ}) + "\n").encode("ascii"))

    def stop(self):
        for _ in range(3):
            try:
                self.sock.sendall(b'{"action":"stop"}\n')
            except OSError:
                break
            time.sleep(0.03)

    def close(self):
        try:
            self.stop()
        finally:
            self.sock.close()


def selftest():
    assert abs(math.degrees(wrapped_delta(math.radians(179), math.radians(-179))) - 2) < 1e-6
    cfg = nr.NavRLConfig(lidar_yaw_offset_deg=0, lidar_forward_offset_m=0.1)
    minimum, p01, count = swept_clearance([(0, 0.5)] * 100, cfg)
    assert count == 100 and minimum > 0.59 and p01 > 0.59
    minimum, p01, _ = swept_clearance([(0, 0.25)] * 100, cfg)
    assert 0.34 < p01 < 0.36
    print("selftest OK")


def run(probe=False):
    check_service_status()
    if not probe:
        check_one_serial_owner()
    cfg = nr.NavRLConfig(lidar_yaw_offset_deg=180.0,
                         lidar_forward_offset_m=0.10,
                         safety_brake_dist=0.42)
    nr.validate_config(cfg)
    sensors = RosNavSensors()
    client = None
    try:
        sensors.wait_ready()
        scan, start, _, error = sensors.snapshot()
        reason, clearance = sensor_gate(scan, start, cfg)
        if error or reason:
            raise RuntimeError(error or "%s: %r" % (reason, clearance))
        print("preflight clearance_min=%.3f p01=%.3f points=%d" % clearance)
        if probe:
            return
        if abs(start.vx) > 0.01 or abs(start.wz) > 0.02:
            raise RuntimeError("odom reports motion before start")
        client = SpinClient()
        begun = time.monotonic()
        accumulated = 0.0
        last_yaw = start.yaw
        next_stop = SEGMENT_DEG
        last_progress = begun
        while accumulated < TOTAL_DEG - 2.0:
            now = time.monotonic()
            scan, odom, _, error = sensors.snapshot()
            reason, clearance = sensor_gate(scan, odom, cfg)
            if error or reason:
                raise RuntimeError(error or "%s: %r" % (reason, clearance))
            delta = math.degrees(wrapped_delta(last_yaw, odom.yaw))
            if delta < -1.0:
                raise RuntimeError("wrong_yaw_direction")
            if delta > 0:
                accumulated += delta
                last_yaw = odom.yaw
                last_progress = now
            drift = math.hypot(odom.x - start.x, odom.y - start.y)
            if accumulated > HARD_TOTAL_DEG or drift > MAX_TRANSLATION_M:
                raise RuntimeError("yaw_or_translation_limit")
            if now - begun > MAX_TIME_S or now - last_progress > 2.5:
                raise RuntimeError("time_or_progress_limit")
            if accumulated >= next_stop - STOP_LEAD_DEG:
                client.stop()
                time.sleep(0.45)
                scan, settled, _, error = sensors.snapshot()
                accumulated += max(0.0, math.degrees(wrapped_delta(last_yaw, settled.yaw)))
                last_yaw = settled.yaw
                reason, clearance = sensor_gate(scan, settled, cfg)
                drift = math.hypot(settled.x - start.x, settled.y - start.y)
                print("checkpoint target=%.0f accumulated=%.2f drift=%.3f clearance_p01=%.3f" %
                      (next_stop, accumulated, drift, clearance[1] if clearance else -1))
                if error or reason or abs(settled.wz) > 0.05:
                    raise RuntimeError(error or reason or "did_not_stop")
                next_stop += SEGMENT_DEG
                continue
            client.turn()
            time.sleep(CONTROL_PERIOD_S)
        client.stop()
        time.sleep(0.8)
        _, final, _, _ = sensors.snapshot()
        accumulated += max(0.0, math.degrees(wrapped_delta(last_yaw, final.yaw)))
        drift = math.hypot(final.x - start.x, final.y - start.y)
        heading_error = abs(((final.yaw - start.yaw + math.pi) % (2*math.pi)) - math.pi)
        print("complete accumulated_deg=%.2f heading_error_deg=%.2f drift_m=%.3f final_wz=%.3f" %
              (accumulated, math.degrees(heading_error), drift, final.wz))
        expected_heading = abs(((math.radians(TOTAL_DEG) + math.pi) % (2*math.pi)) - math.pi)
        heading_tolerance = 15.0
        if (not TOTAL_DEG - 10 <= accumulated <= HARD_TOTAL_DEG or
                abs(math.degrees(heading_error) - math.degrees(expected_heading)) > heading_tolerance or
                drift > MAX_TRANSLATION_M or abs(final.wz) > 0.05):
            raise RuntimeError("final_acceptance_failed")
    finally:
        if client is not None:
            client.close()
        sensors.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--real", action="store_true")
    parser.add_argument("--total-deg", type=float, default=360.0)
    parser.add_argument("--i-confirm-clearance", action="store_true")
    parser.add_argument("--i-confirm-power-cut", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        selftest()
    elif args.probe:
        run(probe=True)
    elif args.real and args.i_confirm_clearance and args.i_confirm_power_cut:
        if not 10.0 <= args.total_deg <= 720.0:
            raise SystemExit("--total-deg must be 10..720")
        TOTAL_DEG = args.total_deg
        HARD_TOTAL_DEG = args.total_deg + 15.0
        run(probe=False)
    else:
        raise SystemExit("real spin needs clearance and power-cut confirmations")
