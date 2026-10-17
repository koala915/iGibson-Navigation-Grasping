"""Supervised PPO approach -> camera preview at fixed cruise pose. Never grasps."""
import _thread
import argparse
import ipaddress
import json
import os
from pathlib import Path
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "integration"))
from sugarbox_navigation_handoff import HANDOFF_EXIT_CODE, NAV_POSE


class ArmMotionCancelled(RuntimeError):
    pass


def near_pose(angles, pose, tolerance=1):
    return (isinstance(angles, list) and len(angles) == len(pose)
            and all(isinstance(a, (int, float)) and not isinstance(a, bool)
                    and abs(a-b) <= tolerance for a, b in zip(angles, pose)))


class MotorConnection:
    """Only stop, read status, and explicitly initialize the fixed cruise pose."""
    def __init__(self, host, port=7000):
        self.sock = socket.create_connection((host, port), timeout=12)
        self.pending = b""
        self.counter = 0

    def stop(self):
        self.sock.sendall(b'{"action":"stop"}\n')

    def request(self, action, **fields):
        if action not in ("arm_status", "arm_initialize"):
            raise ValueError("Cruise-only camera demo refuses other arm commands")
        self.stop()
        self.counter += 1
        request_id = "cruise-" + str(self.counter)
        command = dict(fields, action=action, request_id=request_id)
        self.sock.sendall((json.dumps(command) + "\n").encode())
        while b"\n" not in self.pending:
            part = self.sock.recv(4096)
            if not part:
                raise RuntimeError("Cruise bridge disconnected")
            self.pending += part
            if len(self.pending) > 8192:
                raise RuntimeError("Oversized cruise bridge response")
        line, self.pending = self.pending.split(b"\n", 1)
        value = json.loads(line)
        if value.get("ok") is not True or value.get("request_id") != request_id:
            raise RuntimeError("Cruise request refused: " + str(value))
        return value

    def initialize(self):
        return self.request("arm_initialize", confirmed=True, empty_gripper=True, workspace_clear=True)

    def close(self):
        try:
            self.stop()
        finally:
            self.sock.close()


def demo_environment(host):
    ipaddress.IPv4Address(host)
    return dict(ROUTE_B_IP=host, SUGARBOX_RUN_MODE="DRIVE",
                SUGARBOX_TRAINING_CONTRACT="1", SUGARBOX_CONTRACT_ALLOW_DRIVE="1",
                SUGARBOX_TARGET_INITIAL_ONLY="1", SUGARBOX_PPO_ONLY="0",
                SUGARBOX_FOOTPRINT_GUARD="0", SUGARBOX_ARM_HANDOFF="1",
                SUGARBOX_REAR_HANDOFF_V="470")


def handoff_preview(result, preview, expired=False):
    if expired or result != HANDOFF_EXIT_CODE:
        print("[DEMO_NO_HANDOFF] No arm movement or grasp after exit/error.", flush=True)
        return False
    preview()
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--drive-seconds", type=float, default=180)
    parser.add_argument("--preview-seconds", type=float, default=60)
    for flag in ("i-am-beside-the-robot", "arm-empty-and-clear"):
        parser.add_argument("--" + flag, action="store_true", required=True)
    args = parser.parse_args()
    if not 0 < args.drive_seconds <= 240 or not 0 < args.preview_seconds <= 120:
        parser.error("Bounded drive/preview durations required")
    os.environ.update(demo_environment(args.host))
    import cv2
    import numpy as np
    import sugarbox_rl_approach_final2 as app

    readers = []
    motor = None
    cancelled = False
    pose_motion = False
    stage = "SETUP - STOPPED"
    names = ("Rear Camera - Recording Setup", "Arm Camera - Recording Preview")

    def frames():
        values = []
        for index, reader in enumerate(readers):
            ok, frame, stamp = reader.read()
            if ok and frame.shape[:2] == (480, 640) and 0 <= time.time()-stamp <= .35:
                values.append((index, frame, stamp))
                panel = frame.copy()
                cv2.putText(panel, stage, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            .5, (255, 255, 255), 1)
                cv2.imshow(names[index], panel)
        return values

    def cancel():
        nonlocal cancelled
        values = frames()
        if pose_motion and len(values) != len(readers):
            cancelled = True
        if (cv2.waitKey(1) & 255 in (27, 32)
                or any(cv2.getWindowProperty(names[index], cv2.WND_PROP_VISIBLE) < 1
                       for index, _ in enumerate(readers))):
            cancelled = True
        return cancelled

    def open_cameras(ports):
        nonlocal cancelled, pose_motion
        cancelled, pose_motion = False, False
        for index, port in enumerate(ports):
            readers.append(app.LatestFrameReader(udp_port=port))
            cv2.namedWindow(names[index], cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(names[index], 30+index*670, 60)
            cv2.imshow(names[index], np.zeros((480, 640, 3), dtype=np.uint8))
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            if cancel():
                raise ArmMotionCancelled("Recording setup cancelled")
            if len(frames()) == len(readers):
                return
            time.sleep(.01)
        raise RuntimeError("Fresh bound camera frames required before arm movement")

    def release_cameras():
        for reader in readers:
            reader.release()
        readers.clear()
        cv2.destroyAllWindows()

    def preview():
        nonlocal motor, stage
        # Preserve the arm-window name/index; both streams stay visible but stopped.
        stage = "HANDOFF - STOPPED - NO GRASP"
        open_cameras((5600, 5602))
        motor = MotorConnection(args.host)
        if not near_pose(motor.request("arm_status").get("servo_deg"), NAV_POSE, 1):
            raise RuntimeError("Cruise pose changed; no alignment or arm movement")
        stage = "ARM CAMERA - STOPPED - NO ALIGN / NO GRASP"
        deadline = time.monotonic()+args.preview_seconds
        while time.monotonic() < deadline and not cancel():
            time.sleep(.01)

    timer = None
    expired = threading.Event()
    try:
        open_cameras((5600, 5602))
        motor = MotorConnection(args.host)
        pose_motion = True
        print("[DEMO_INIT] " + json.dumps(motor.initialize()), flush=True)
        if not near_pose(motor.request("arm_status").get("servo_deg"), NAV_POSE, 1):
            raise RuntimeError("Actual travel pose not confirmed; no PPO")
        print("[DEMO_CRUISE] Fixed (90,140,0,0,90,30); no camera-pose changes.", flush=True)
        motor.close()
        motor = None
        release_cameras()

        def timeout():
            expired.set()
            print("[DEMO_TIMEOUT] Ending PPO; no automatic camera-pose movement.", flush=True)
            _thread.interrupt_main()

        print("[DEMO_START] PPO approach; space estops, ESC exits; no grasp.", flush=True)
        timer = threading.Timer(args.drive_seconds, timeout)
        timer.daemon = True
        timer.start()
        try:
            result = app.main()
        finally:
            timer.cancel()
        print("[DEMO_PPO_EXIT] " + str(result), flush=True)
        handoff_preview(result, preview, expired.is_set())
    except (ArmMotionCancelled, KeyboardInterrupt) as exc:
        print("[DEMO_CANCELLED] " + str(exc), flush=True)
    finally:
        if timer:
            timer.cancel()
        try:
            if motor:
                motor.close()
        finally:
            release_cameras()
        print("[DEMO_DONE] No grasp command or post-handoff chassis alignment.", flush=True)


if __name__ == "__main__":
    main()
