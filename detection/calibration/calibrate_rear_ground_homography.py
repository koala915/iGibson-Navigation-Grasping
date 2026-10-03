#!/usr/bin/env python3
"""Interactive rear-camera pixel-to-ground homography calibration.

Click the object's floor contact point, enter measured x-forward/y-left metres,
and press S after at least six samples. The output schema is consumed directly
by detection/rear_cam_sam2_publisher.py.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np


def reprojection_errors(homography: np.ndarray, pixels: np.ndarray,
                        ground: np.ndarray) -> np.ndarray:
    projected = cv2.perspectiveTransform(
        pixels.reshape(-1, 1, 2).astype(np.float64), homography).reshape(-1, 2)
    return np.linalg.norm(projected - ground, axis=1)


def solve(points_px, points_ground, *, rear_cam_to_front_m: float,
          camera_url: str) -> dict:
    if len(points_px) < 6 or len(points_px) != len(points_ground):
        raise ValueError("at least six matched pixel/ground samples are required")
    pixels = np.asarray(points_px, dtype=np.float64)
    ground = np.asarray(points_ground, dtype=np.float64)
    if not np.isfinite(pixels).all() or not np.isfinite(ground).all():
        raise ValueError("calibration samples must be finite")
    homography, mask = cv2.findHomography(pixels, ground, cv2.RANSAC, 0.04)
    if homography is None or not np.isfinite(homography).all():
        raise ValueError("homography solve failed")
    inliers = (mask.reshape(-1) != 0) if mask is not None else np.ones(len(pixels), bool)
    if int(inliers.sum()) < 4:
        raise ValueError("homography has fewer than four inliers")
    errors = reprojection_errors(homography, pixels, ground)
    samples = []
    for index, (pixel, xy, error, accepted) in enumerate(
            zip(pixels, ground, errors, inliers), 1):
        samples.append({
            "id": index,
            "u": float(pixel[0]), "v": float(pixel[1]),
            "x": float(xy[0]), "y": float(xy[1]),
            "reprojection_error_m": float(error),
            "inlier": bool(accepted),
        })
    inlier_errors = errors[inliers]
    return {
        "type": "rear_camera_ground_homography",
        "version": 2,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "camera_url": camera_url,
        "frame_width": 640,
        "frame_height": 480,
        "coordinate_convention": {
            "pixel_input": "bottom_point_u_v",
            "ground_output": "x_forward_y_left",
            "origin": "rear_camera_ground_projection",
            "x_axis": "forward_positive_meters",
            "y_axis": "left_positive_right_negative_meters",
        },
        "rear_cam_to_front_m": float(rear_cam_to_front_m),
        "H_pixel_to_ground": homography.astype(float).tolist(),
        "homography": homography.astype(float).tolist(),
        "sample_count": len(samples),
        "inlier_count": int(inliers.sum()),
        "mean_reprojection_error_m": float(np.mean(inlier_errors)),
        "max_reprojection_error_m": float(np.max(inlier_errors)),
        "samples": samples,
    }


def save_result(result: dict, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        backup_dir = output.parent / "homography_backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / (output.stem + "_" +
                               datetime.now().strftime("%Y%m%d_%H%M%S") + output.suffix)
        backup.write_bytes(output.read_bytes())
        print(f"[BACKUP] {backup}")
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                      encoding="utf-8")
    print(f"[SAVED] {output}")


def run(args) -> int:
    capture = cv2.VideoCapture(args.camera_url)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open rear camera: {args.camera_url}")
    points_px: List[List[float]] = []
    points_ground: List[List[float]] = []
    pending: List[Optional[Tuple[int, int]]] = [None]

    def clicked(event, x, y, _flags, _data):
        if event == cv2.EVENT_LBUTTONDOWN:
            pending[0] = (int(x), int(y))

    cv2.namedWindow("Rear ground homography")
    cv2.setMouseCallback("Rear ground homography", clicked)
    print("Click a floor contact point, then enter x-forward and y-left metres.")
    print("Keys: S solve/save, U undo, R reset, Q quit")
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            frame = cv2.resize(frame, (640, 480))
            if pending[0] is not None:
                u, v = pending[0]
                pending[0] = None
                try:
                    raw = input(f"pixel ({u},{v}) -> x y metres: ").strip().split()
                    if len(raw) != 2:
                        raise ValueError("enter exactly two numbers")
                    x, y = map(float, raw)
                    if not (math.isfinite(x) and math.isfinite(y)):
                        raise ValueError("coordinates must be finite")
                    points_px.append([float(u), float(v)])
                    points_ground.append([x, y])
                    print(f"[ADD] {len(points_px)}: ({u},{v}) -> ({x:.3f},{y:+.3f})")
                except ValueError as exc:
                    print(f"[REJECT] {exc}")
            for index, (u, v) in enumerate(points_px, 1):
                cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 255), -1)
                cv2.putText(frame, str(index), (int(u) + 7, int(v) - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(frame, f"samples={len(points_px)}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.imshow("Rear ground homography", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                return 0
            if key in (ord("u"), ord("U")) and points_px:
                points_px.pop(); points_ground.pop()
            if key in (ord("r"), ord("R")):
                points_px.clear(); points_ground.clear()
            if key in (ord("s"), ord("S")):
                result = solve(points_px, points_ground,
                               rear_cam_to_front_m=args.rear_cam_to_front,
                               camera_url=args.camera_url)
                save_result(result, args.output)
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-url", required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]
                        / "rear_ground_homography.json")
    parser.add_argument("--rear-cam-to-front", type=float, default=0.20)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
