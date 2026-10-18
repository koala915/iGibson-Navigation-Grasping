"""Newest frame from the rear camera's RTP/H264 stream, on the Windows PC.

The Jetson sends it with deploy/ros/start_rear_approach_stream.sh <PC_IP> 5600.
Only one receiver can hold the UDP port: close this before the approach starts
its own.
"""
import subprocess
import threading
import time

import numpy as np

W, H = 640, 480
# The arm camera sits here in the travel pose and once read as a sugarbox at
# 0.84-0.94 (2026-10-17). Same box as REAR_IGNORE_REGIONS in the approach engine;
# tests/test_mission_loop.py keeps the two equal.
ARM_CAMERA_REGION = (295, 395, 405, 480)


class StreamReader:
    """Keep only the newest decoded frame from the rear UDP/H264 stream."""

    def __init__(self, port, gst):
        self.proc = subprocess.Popen(
            [str(gst), "-q", "udpsrc", f"port={port}",
             "caps=application/x-rtp,media=video,encoding-name=H264,payload=96",
             "!", "rtph264depay", "!", "h264parse", "!", "avdec_h264", "!", "videoconvert", "!",
             f"video/x-raw,format=BGR,width={W},height={H}", "!", "fdsink", "fd=1", "sync=false"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.frame, self.stamp = None, 0.0
        self.lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        size = W * H * 3
        while True:
            buf = self.proc.stdout.read(size)
            if len(buf) < size:
                return
            with self.lock:
                self.frame = np.frombuffer(buf, np.uint8).reshape(H, W, 3).copy()
                self.stamp = time.monotonic()

    def latest(self):
        with self.lock:
            return self.frame, self.stamp

    def close(self):
        self.proc.kill()
        self.proc.wait()
