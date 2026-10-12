"""Capture one rear-camera calibration sample with the runtime's own pixel extractor.

Usage: rear_calib_capture.py LABEL X_WHEEL_M Y_LEFT_M [--frames N]
Appends {label, x_wheel, y, u, v, ...} to rear_calib_samples.jsonl next to this file
and saves an annotated frame calib_LABEL.png. The pixel is the median SAM2 mask
bottom-contact point over N frames, the same point the approach feeds its homography.
"""
import argparse
import importlib.util
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

REPO = Path(r"C:\Users\user\Documents\repo\iGibson-Navigation-Grasping")
HERE = Path(__file__).resolve().parent
GST = REPO / r".venv-approach\Lib\site-packages\gstreamer_cli\bin\gst-launch-1.0.exe"
W, H = 640, 480

ap = argparse.ArgumentParser()
ap.add_argument("label")
ap.add_argument("x_wheel", type=float)
ap.add_argument("y_left", type=float)
ap.add_argument("--frames", type=int, default=12)
args = ap.parse_args()

os.environ["SUGARBOX_YOLO_IMGSZ"] = "1280"
sys.path.insert(0, str(REPO / "integration"))
spec = importlib.util.spec_from_file_location("approach", REPO / "integration/sugarbox_rl_approach_final2.py")
approach = importlib.util.module_from_spec(spec)
spec.loader.exec_module(approach)
from ultralytics import YOLO, SAM

yolo = YOLO(str(approach.YOLO_MODEL_PATH))
sam = SAM(str(approach.SAM2_MODEL_PATH))

proc = subprocess.Popen(
    [str(GST), "-q", "udpsrc", "port=5600",
     "caps=application/x-rtp,media=video,encoding-name=H264,payload=96", "!",
     "rtph264depay", "!", "h264parse", "!", "avdec_h264", "!", "videoconvert", "!",
     f"video/x-raw,format=BGR,width={W},height={H}", "!",
     "queue", "max-size-buffers=1", "max-size-bytes=0", "max-size-time=0", "leaky=downstream", "!",
     "fdsink", "fd=1", "sync=false"],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
latest = {"frame": None, "n": 0}


def reader():
    size = W * H * 3
    buf = bytearray()
    while True:
        chunk = proc.stdout.read(size - len(buf))
        if not chunk:
            return
        buf.extend(chunk)
        if len(buf) == size:
            latest["frame"] = np.frombuffer(bytes(buf), np.uint8).reshape(H, W, 3)
            latest["n"] += 1
            buf = bytearray()


threading.Thread(target=reader, daemon=True).start()
rows, last_frame, last_seen, notes = [], None, -1, []
try:
    deadline = time.time() + 25
    while len(rows) < args.frames and time.time() < deadline:
        if latest["n"] == last_seen or latest["frame"] is None:
            time.sleep(0.02)
            continue
        last_seen, frame = latest["n"], latest["frame"].copy()
        res = yolo.predict(frame, imgsz=1280, conf=approach.YOLO_CONF, verbose=False)
        det = approach.select_best_box(res[0], yolo.names) if res else None
        if det is None:
            notes.append("NO_YOLO")
            continue
        mask = approach.run_sam2_with_bbox(sam, frame, det["bbox"])
        contact = approach.mask_bottom_contact(mask) if mask is not None else None
        if contact is None:
            notes.append("NO_CONTACT")
            continue
        u, v, _ = contact
        x1, y1, x2, y2 = det["bbox"]
        rows.append((float(u), float(v), float(y2), float((x1 + x2) / 2), float(det.get("conf", 0))))
        last_frame = (frame, det["bbox"], (u, v))
finally:
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True)

if not rows:
    print(json.dumps({"label": args.label, "ok": False, "notes": notes[-5:]}))
    sys.exit(1)
arr = np.array(rows)
u, v = float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))
out = {"label": args.label, "ok": True, "x_wheel": args.x_wheel, "y": args.y_left,
       "u": round(u, 1), "v": round(v, 1),
       "u_spread": round(float(arr[:, 0].max() - arr[:, 0].min()), 1),
       "v_spread": round(float(arr[:, 1].max() - arr[:, 1].min()), 1),
       "yolo_bottom_v": round(float(np.median(arr[:, 2])), 1),
       "conf": round(float(np.median(arr[:, 4])), 2), "frames": len(rows),
       "misses": len(notes)}
with open(HERE / "rear_calib_samples.jsonl", "a", encoding="utf-8") as f:
    f.write(json.dumps(out) + "\n")
frame, bbox, (cu, cv_) = last_frame
img = frame.copy()
cv2.rectangle(img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 1)
cv2.circle(img, (int(u), int(v)), 4, (0, 0, 255), -1)
cv2.putText(img, f"{args.label} x={args.x_wheel} y={args.y_left}", (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
cv2.imwrite(str(HERE / f"calib_{args.label}.png"), img)
print(json.dumps(out))
