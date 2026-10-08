"""Offline safety tests for the G2 navigation-only TCP client."""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from integration import g2_nav_client as g2


def odom(x=0.0, y=0.0):
    return g2.Odom(x, y, 0.0, 0.0, 0.0, time.monotonic(), time.time(),
                   "odom", "base_footprint")


def scan(range_m=0.70, clustered=True):
    angles = (-0.2, 0.0, 0.2) if clustered else (0.0,)
    return g2.Scan(tuple((180.0 + a, range_m) for a in angles),
                   time.monotonic(), time.time(), "laser")


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.closed = False

    def setsockopt(self, *_args):
        pass

    def sendall(self, data):
        self.sent.append(json.loads(data))

    def close(self):
        self.closed = True


class FakeSensors:
    def __init__(self, xs, ranges=None):
        self.xs = list(xs)
        self.ranges = ranges or [0.70] * len(xs)
        self.i = 0

    def snapshot(self):
        i = min(self.i, len(self.xs) - 1)
        self.i += 1
        return scan(self.ranges[i]), odom(self.xs[i]), None, ""


class Tests(unittest.TestCase):
    def setUp(self):
        self.cfg = g2.nr.NavRLConfig(lidar_yaw_offset_deg=180.0,
                                     lidar_forward_offset_m=0.10)

    def test_brake_and_freshness(self):
        reason, front = g2.health(scan(0.31), odom(), self.cfg)
        self.assertEqual(reason, "lidar_brake")
        self.assertLess(front, 0.42)
        self.assertEqual(g2.health(scan(0.31, clustered=False), odom(),
                                   self.cfg)[0], "")
        self.assertEqual(g2.health(dataclasses.replace(
            scan(), received_at=time.monotonic()-1), odom(), self.cfg)[0],
            "stale_arrival")
        self.assertEqual(g2.health(dataclasses.replace(
            scan(), stamp=time.time()-1), odom(), self.cfg)[0], "stale_stamp")
        self.assertEqual(g2.health(scan(), dataclasses.replace(
            odom(), child_frame_id="wrong"), self.cfg)[0], "wrong_frame")

    def test_tcp_protocol_and_stop_on_close(self):
        sock = FakeSocket()
        with patch.object(g2.socket, "create_connection", return_value=sock):
            client = g2.G2ChassisClient()
            client.velocity(0.15)
            with self.assertRaises(ValueError):
                client.velocity(0.16)
            with self.assertRaises(ValueError):
                client.velocity(math.nan)
            client.close()
        self.assertEqual(sock.sent[0], {"action": "velocity", "vx": 0.15,
                                        "wz": 0.0})
        self.assertEqual(sock.sent[1:], [{"action": "stop"}] * 3)
        self.assertTrue(sock.closed)

    def test_obstacle_refuses_connection(self):
        sensors = FakeSensors([0.0], [0.31])
        with patch.object(g2, "G2ChassisClient", side_effect=AssertionError(
                "must not connect")):
            with self.assertRaisesRegex(RuntimeError, "lidar_brake"):
                g2.run_straight(sensors, self.cfg, 0.15)

    def test_distance_run_stops_and_closes(self):
        sensors = FakeSensors([0.0, 0.0, 0.05, 0.10, 0.15])
        sock = FakeSocket()
        with patch.object(g2.socket, "create_connection", return_value=sock):
            reason, distance = g2.run_straight(sensors, self.cfg, 0.15)
        self.assertEqual(reason, "distance_target")
        self.assertAlmostEqual(distance, 0.15)
        self.assertGreaterEqual(sum(msg["action"] == "velocity"
                                    for msg in sock.sent), 1)
        self.assertEqual(sock.sent[-3:], [{"action": "stop"}] * 3)
        self.assertTrue(sock.closed)

    def test_obstacle_during_motion_stops_and_reports_safety_reason(self):
        sensors = FakeSensors([0.0, 0.0, 0.02, 0.02],
                              [0.70, 0.70, 0.31, 0.31])
        sock = FakeSocket()
        with patch.object(g2.socket, "create_connection", return_value=sock):
            reason, _ = g2.run_straight(sensors, self.cfg, 0.15)
        self.assertEqual(reason, "lidar_brake")
        self.assertEqual(sock.sent[0]["action"], "velocity")
        self.assertEqual(sock.sent[1:], [{"action": "stop"}] * 3)
        self.assertTrue(sock.closed)

    def test_serial_owner_gate(self):
        class Result:
            def __init__(self, stdout):
                self.returncode = 0
                self.stdout = stdout
                self.stderr = ""
        with patch.object(g2.subprocess, "run", side_effect=[
                Result("4324\n"), Result("4324\n")]):
            self.assertEqual(g2.check_one_serial_owner(), "4324")
        with patch.object(g2.subprocess, "run", return_value=Result("4324 9999\n")):
            with self.assertRaisesRegex(RuntimeError, "exactly one owner"):
                g2.check_one_serial_owner()
        with patch.object(g2.subprocess, "run", side_effect=[
                Result("4324\n"), Result("9999\n")]):
            with self.assertRaisesRegex(RuntimeError, "not grasp-service"):
                g2.check_one_serial_owner()

    def test_brief_noise_waits_then_resumes_original_distance(self):
        sensors = FakeSensors([0,0,.02,.02,.02,.02,.02,.04,.07,.10,.125,.15],
                              [.70,.70,.31,.31,.70,.70,.70,.70,.70,.70,.70,.70])
        sock = FakeSocket()
        with patch.object(g2.socket,'create_connection',return_value=sock):
            reason,distance = g2.run_straight(sensors,self.cfg,.15,recover_sensor_s=3)
        self.assertEqual(reason,'distance_target')
        self.assertGreaterEqual(distance,.15-g2.STOP_LEAD_M)
        self.assertLessEqual(distance,.15)
        actions=[m['action'] for m in sock.sent]
        first_stop=actions.index('stop')
        self.assertIn('velocity',actions[first_stop+1:])
        self.assertEqual(actions[-3:],['stop']*3)

    def test_persistent_noise_has_a_finite_wait(self):
        sensors=FakeSensors([0,0,.02],[.70,.70,.31]);sock=FakeSocket()
        with patch.object(g2.socket,'create_connection',return_value=sock):
            reason,_=g2.run_straight(sensors,self.cfg,.15,recover_sensor_s=.2)
        self.assertEqual(reason,'lidar_brake')
        self.assertEqual(sum(m['action']=='velocity' for m in sock.sent),1)
        self.assertTrue(sock.closed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
