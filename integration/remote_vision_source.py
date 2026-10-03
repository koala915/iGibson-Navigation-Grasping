#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math
import threading
import time

import roslibpy


class RemoteVisionSource(object):
    """
    接收 Windows YOLO + SAM2 發布的：

        /trash_target/detection
        std_msgs/String

    Windows 座標定義：
        x：機器人前方為正
        y：機器人左側為正、右側為負
    """

    def __init__(
        self,
        ros_host="127.0.0.1",
        ros_port=9090,
        topic_name="/trash_target/detection",
        target_class="sugarbox",
        minimum_confidence=0.35,
        minimum_stable_frames=5,
        target_timeout_sec=1.0,
        existing_ros=None,
        auto_connect=True,
    ):
        self.ros_host = str(ros_host)
        self.ros_port = int(ros_port)
        self.topic_name = str(topic_name)

        self.target_class = str(target_class).strip().lower()
        self.minimum_confidence = float(minimum_confidence)
        self.minimum_stable_frames = int(minimum_stable_frames)
        self.target_timeout_sec = float(target_timeout_sec)

        self.lock = threading.Lock()

        self.latest_payload = None
        self.latest_receive_monotonic = 0.0
        self.latest_error = None

        self.owns_ros_connection = existing_ros is None

        if existing_ros is None:
            self.ros = roslibpy.Ros(
                host=self.ros_host,
                port=self.ros_port,
            )
        else:
            self.ros = existing_ros

        self.topic = roslibpy.Topic(
            self.ros,
            self.topic_name,
            "std_msgs/String",
        )

        if auto_connect:
            self.start()

    def start(self):
        if not self.ros.is_connected:
            print(
                "[REMOTE VISION] Connecting to ws://{}:{}".format(
                    self.ros_host,
                    self.ros_port,
                )
            )

            self.ros.run(timeout=10)

        if not self.ros.is_connected:
            raise RuntimeError(
                "RemoteVisionSource 無法連線 rosbridge"
            )

        self.topic.subscribe(self._message_callback)

        print(
            "[REMOTE VISION] Subscribed: {}".format(
                self.topic_name
            )
        )

    def _message_callback(self, message):
        try:
            raw_data = message.get("data", "")

            if isinstance(raw_data, dict):
                payload = raw_data
            else:
                payload = json.loads(raw_data)

            if not isinstance(payload, dict):
                raise TypeError(
                    "payload 不是 JSON object"
                )

            receive_time = time.monotonic()

            with self.lock:
                self.latest_payload = payload
                self.latest_receive_monotonic = receive_time
                self.latest_error = None

        except Exception as exc:
            with self.lock:
                self.latest_error = "{}: {}".format(
                    type(exc).__name__,
                    exc,
                )

            print(
                "[REMOTE VISION ERROR] {}".format(
                    self.latest_error
                )
            )

    def _copy_latest(self):
        with self.lock:
            payload = (
                dict(self.latest_payload)
                if isinstance(self.latest_payload, dict)
                else None
            )

            receive_time = float(
                self.latest_receive_monotonic
            )

            error = self.latest_error

        return payload, receive_time, error

    def get_latest(self):
        """
        回傳整理後的有效目標。

        有效時：
            {
                "valid": True,
                "class_name": "sugarbox",
                "x_forward_m": ...,
                "y_left_m": ...,
                "y_right_m": ...,
                "distance_m": ...,
                "bearing_rad": ...,
                "confidence": ...,
                "stable_frames": ...,
                "age_sec": ...,
                "raw": {...}
            }

        無效時回傳 None。
        """

        payload, receive_time, error = self._copy_latest()

        if error is not None:
            return None

        if payload is None:
            return None

        age_sec = time.monotonic() - receive_time

        if age_sec > self.target_timeout_sec:
            return None

        if not bool(payload.get("valid", False)):
            return None

        class_name = str(
            payload.get("class_name", "")
        ).strip().lower()

        if class_name != self.target_class:
            return None

        stable_frames = int(
            payload.get("stable_frames", 0)
        )

        if stable_frames < self.minimum_stable_frames:
            return None

        confidence = float(
            payload.get("confidence", 0.0)
        )

        if confidence < self.minimum_confidence:
            return None

        coordinate_convention = str(
            payload.get(
                "coordinate_convention",
                "",
            )
        ).strip()

        if coordinate_convention != "x_forward_y_left":
            return None

        try:
            x_forward_m = float(
                payload["object_x_base"]
            )

            y_left_m = float(
                payload["object_y_base"]
            )

        except (KeyError, TypeError, ValueError):
            return None

        if not (
            math.isfinite(x_forward_m)
            and math.isfinite(y_left_m)
        ):
            return None

        if x_forward_m <= 0.0:
            return None

        distance_m = float(
            payload.get(
                "distance_m",
                math.hypot(
                    x_forward_m,
                    y_left_m,
                ),
            )
        )

        bearing_rad = float(
            payload.get(
                "bearing_rad",
                math.atan2(
                    y_left_m,
                    x_forward_m,
                ),
            )
        )

        return {
            "valid": True,
            "class_name": class_name,
            "x_forward_m": x_forward_m,

            # Windows JSON 原始定義
            "y_left_m": y_left_m,

            # 組員舊 GoalTracker 若使用右正，直接使用這個
            "y_right_m": -y_left_m,

            "distance_m": distance_m,
            "bearing_rad": bearing_rad,
            "confidence": confidence,
            "stable_frames": stable_frames,
            "age_sec": age_sec,
            "timestamp_unix": payload.get(
                "timestamp_unix"
            ),
            "reason": payload.get("reason"),
            "raw": payload,
        }

    def get_status(self):
        payload, receive_time, error = self._copy_latest()

        if receive_time > 0.0:
            age_sec = time.monotonic() - receive_time
        else:
            age_sec = None

        return {
            "connected": bool(
                self.ros.is_connected
            ),
            "has_message": payload is not None,
            "age_sec": age_sec,
            "error": error,
            "raw": payload,
        }

    def close(self):
        try:
            self.topic.unsubscribe()
        except Exception:
            pass

        if self.owns_ros_connection:
            try:
                self.ros.terminate()
            except Exception:
                pass


def main():
    source = RemoteVisionSource(
        ros_host="127.0.0.1",
        ros_port=9090,
        target_class="sugarbox",
        minimum_confidence=0.35,
        minimum_stable_frames=5,
        target_timeout_sec=1.0,
    )

    print()
    print("等待 Windows sugarbox 目標...")
    print("Ctrl+C 結束")
    print()

    last_print_time = 0.0

    try:
        while True:
            target = source.get_latest()

            now = time.monotonic()

            if now - last_print_time >= 0.25:
                last_print_time = now

                if target is None:
                    status = source.get_status()

                    raw = status.get("raw") or {}

                    print(
                        "[NO TARGET] reason={} age={}".format(
                            raw.get(
                                "reason",
                                "no_message",
                            ),
                            (
                                "{:.2f}s".format(
                                    status["age_sec"]
                                )
                                if status["age_sec"] is not None
                                else "None"
                            ),
                        )
                    )

                else:
                    print(
                        "[TARGET] class={} "
                        "x_forward={:.3f} m "
                        "y_left={:.3f} m "
                        "y_right={:.3f} m "
                        "dist={:.3f} m "
                        "conf={:.3f} "
                        "stable={}".format(
                            target["class_name"],
                            target["x_forward_m"],
                            target["y_left_m"],
                            target["y_right_m"],
                            target["distance_m"],
                            target["confidence"],
                            target["stable_frames"],
                        )
                    )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[STOP] RemoteVisionSource")

    finally:
        source.close()


if __name__ == "__main__":
    main()
