#!/usr/bin/env python3
"""TCP motor bridge for ``sugarbox_rl_approach_final2.py``.

This process is the sole owner of ``/dev/myserial`` while the standalone
Sugarbox approach controller is running.  It accepts newline-delimited JSON
velocity commands on TCP port 7000 and publishes the repository's calibrated
wheel-feedback odometry on ``/odom_setmotor``.

The old ``~/ai_motor_server_RL.py`` duplicated odometry calibration constants
and inverted yaw feedback.  This replacement deliberately delegates every
feedback conversion and integration step to ``feedback_odom.py``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

try:
    from .feedback_odom import FeedbackOdomReader
except ImportError:
    from feedback_odom import FeedbackOdomReader


HOST = "0.0.0.0"
PORT = 7000
WATCHDOG_TIMEOUT_S = 0.50
ODOM_RATE_HZ = 20.0

ABSOLUTE_MAX_MOTOR = 60
RL_MOTOR_LIMIT = 30
MIN_ACTIVE_MOTOR = 20.0
CMD_MAX_FORWARD_VX = 0.15
CMD_MAX_REVERSE_VX = 0.08
CMD_MAX_ABS_WZ = 1.00
CMD_VX_DEADBAND = 0.015
CMD_WZ_DEADBAND = 0.03
LINEAR_MOTOR_PER_MPS = 200.0
ANGULAR_MOTOR_PER_RAD_S = float(os.environ.get("G2_ANGULAR_MOTOR_PER_RAD_S", "25.0"))


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def velocity_to_motor_values(vx: float, wz: float) -> Tuple[int, int, int, int]:
    """Preserve the wheel mapping verified in the previous RL server."""
    vx = float(clamp(vx, -CMD_MAX_REVERSE_VX, CMD_MAX_FORWARD_VX))
    wz = float(clamp(wz, -CMD_MAX_ABS_WZ, CMD_MAX_ABS_WZ))
    if abs(vx) < CMD_VX_DEADBAND:
        vx = 0.0
    if abs(wz) < CMD_WZ_DEADBAND:
        wz = 0.0

    left = vx * LINEAR_MOTOR_PER_MPS - wz * ANGULAR_MOTOR_PER_RAD_S
    right = vx * LINEAR_MOTOR_PER_MPS + wz * ANGULAR_MOTOR_PER_RAD_S
    peak = max(abs(left), abs(right))
    if 0.0 < peak < MIN_ACTIVE_MOTOR:
        scale = MIN_ACTIVE_MOTOR / peak
        left *= scale
        right *= scale
        peak = MIN_ACTIVE_MOTOR
    if peak > RL_MOTOR_LIMIT:
        scale = RL_MOTOR_LIMIT / peak
        left *= scale
        right *= scale
    return tuple(int(round(v)) for v in (left, left, right, right))


class SugarboxRLMotorServer:
    def __init__(self, device, rospy, odom_type, transform_type, tf_type,
                 *, host: str = HOST, port: int = PORT):
        self.device = device
        self.rospy = rospy
        self.Odometry = odom_type
        self.TransformStamped = transform_type
        self.TFMessage = tf_type
        self.host = host
        self.port = int(port)
        self.serial_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.reader = FeedbackOdomReader(device, serial_lock=self.serial_lock)
        self.last_command = 0.0
        self.last_motor_values: Optional[Tuple[int, int, int, int]] = None
        self.shutting_down = threading.Event()
        self.server_socket: Optional[socket.socket] = None
        self.odom_pub = rospy.Publisher("/odom_setmotor", odom_type, queue_size=20)
        self.tf_pub = rospy.Publisher("/tf", tf_type, queue_size=20)

    def set_motors(self, values: Tuple[int, int, int, int]) -> None:
        values = tuple(int(clamp(v, -ABSOLUTE_MAX_MOTOR, ABSOLUTE_MAX_MOTOR))
                       for v in values)
        if values == self.last_motor_values:
            return
        with self.serial_lock:
            self.device.set_motor(*values)
        self.last_motor_values = values
        print("[MOTOR] m1={} m2={} m3={} m4={}".format(*values))

    def stop(self) -> None:
        self.set_motors((0, 0, 0, 0))

    def mark_command(self) -> None:
        with self.state_lock:
            self.last_command = time.monotonic()

    def command_age(self) -> float:
        with self.state_lock:
            stamp = self.last_command
        return float("inf") if stamp <= 0.0 else time.monotonic() - stamp

    def process_command(self, command: object) -> None:
        if not isinstance(command, dict):
            raise ValueError("command must be a JSON object")
        action = str(command.get("action", "stop")).strip().lower()
        if action == "stop":
            self.mark_command()
            self.stop()
            return
        if action != "velocity":
            raise ValueError("unsupported action: {}".format(action))
        vx = float(command.get("vx", 0.0))
        wz = float(command.get("wz", 0.0))
        if not math.isfinite(vx) or not math.isfinite(wz):
            raise ValueError("vx and wz must be finite")
        self.mark_command()
        values = velocity_to_motor_values(vx, wz)
        self.set_motors(values)

    def publish_odom(self, state) -> None:
        stamp = self.rospy.Time.now()
        half_yaw = state.yaw * 0.5
        qz, qw = math.sin(half_yaw), math.cos(half_yaw)

        msg = self.Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = self.reader.odom.covariance()
        msg.twist.twist.linear.x = state.vx
        msg.twist.twist.linear.y = state.vy
        msg.twist.twist.angular.z = state.wz
        msg.twist.covariance = self.reader.odom.covariance()
        self.odom_pub.publish(msg)

        transform = self.TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        tf_msg = self.TFMessage()
        tf_msg.transforms = [transform]
        self.tf_pub.publish(tf_msg)

    def odom_loop(self) -> None:
        rate = self.rospy.Rate(ODOM_RATE_HZ)
        while not self.rospy.is_shutdown() and not self.shutting_down.is_set():
            state = self.reader.poll()
            if state.valid:
                self.publish_odom(state)
            elif self.reader.odom.failure_count % 40 == 1:
                self.rospy.logwarn("feedback odom unavailable: {}".format(state.reason))
            rate.sleep()

    def watchdog_loop(self) -> None:
        while not self.rospy.is_shutdown() and not self.shutting_down.is_set():
            if self.command_age() > WATCHDOG_TIMEOUT_S:
                self.stop()
            time.sleep(0.05)

    def handle_client(self, connection: socket.socket, address) -> None:
        print("[INFO] client connected: {}".format(address))
        connection.settimeout(0.20)
        buffer = ""
        try:
            while not self.rospy.is_shutdown() and not self.shutting_down.is_set():
                try:
                    chunk = connection.recv(4096)
                except socket.timeout:
                    continue
                except (ConnectionError, OSError) as exc:
                    print("[WARN] client receive failed: {}".format(exc))
                    break
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip():
                        continue
                    try:
                        self.process_command(json.loads(line))
                    except Exception as exc:
                        print("[WARN] command error: {}".format(exc))
                        self.stop()
        finally:
            self.stop()
            connection.close()
            print("[INFO] client disconnected: {}".format(address))

    def serve(self) -> None:
        threading.Thread(target=self.odom_loop, daemon=True).start()
        threading.Thread(target=self.watchdog_loop, daemon=True).start()
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket = server
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        server.settimeout(0.5)
        print("[INFO] Sugarbox RL motor server listening on {}:{}".format(
            self.host, self.port))
        try:
            while not self.rospy.is_shutdown() and not self.shutting_down.is_set():
                try:
                    connection, address = server.accept()
                except socket.timeout:
                    continue
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.handle_client(connection, address)
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        if self.shutting_down.is_set():
            return
        self.shutting_down.set()
        try:
            self.stop()
        finally:
            if self.server_socket is not None:
                self.server_socket.close()
            try:
                self.device.set_auto_report_state(False, False)
            except Exception:
                pass
            serial = getattr(self.device, "ser", None)
            if serial is not None:
                try:
                    serial.close()
                except Exception:
                    pass


def load_rosmaster(serial_port: str):
    root = Path(__file__).resolve().parents[1]
    grasp_dir = root / "grasp"
    if str(grasp_dir) not in sys.path:
        sys.path.insert(0, str(grasp_dir))
    from Rosmaster_Lib import Rosmaster
    try:
        return Rosmaster(com=serial_port)
    except TypeError:
        return Rosmaster()


def run_selftest() -> None:
    assert velocity_to_motor_values(0.0, 0.0) == (0, 0, 0, 0)
    assert velocity_to_motor_values(0.15, 0.0) == (30, 30, 30, 30)
    assert velocity_to_motor_values(-0.08, 0.0) == (-20, -20, -20, -20)
    left = velocity_to_motor_values(0.0, 0.4)
    right = velocity_to_motor_values(0.0, -0.4)
    assert left[0] < 0 < left[2]
    assert right[0] > 0 > right[2]
    assert max(map(abs, velocity_to_motor_values(999.0, 999.0))) <= RL_MOTOR_LIMIT
    print("[selftest] Sugarbox RL motor mapping OK")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--serial-port", default="/dev/myserial")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        run_selftest()
        return

    import rospy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    rospy.init_node("sugarbox_rl_motor_server", anonymous=False)
    device = load_rosmaster(args.serial_port)
    device.create_receive_threading()
    try:
        device.set_auto_report_state(True, False)
    except TypeError:
        device.set_auto_report_state(enable=True, forever=False)
    server = SugarboxRLMotorServer(
        device, rospy, Odometry, TransformStamped, TFMessage,
        host=args.host, port=args.port)
    rospy.on_shutdown(server.shutdown)
    server.stop()
    server.serve()


if __name__ == "__main__":
    main()
