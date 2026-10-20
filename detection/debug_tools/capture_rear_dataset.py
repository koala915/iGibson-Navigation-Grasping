"""Collect rear-camera training photos for a new YOLO class, hands free.

The 2026-10-12 placement-box model was trained on phone photos and found nothing
in the rear camera, even at 4x zoom (2026-10-17): the viewpoint is near the floor,
the colour is cooler and the stream is H.264. This saves frames from the same
UDP stream the approach uses, so the training images look like what it will see.

Auto mode (default): move the box, step out of view, and a frame is saved once
the picture has held still for --still-s and differs from the last saved frame.
A beep confirms each save, so nobody has to watch the laptop.

Keys in the window: SPACE save now | A toggle auto | B / N tag "box" / "negative"
(empty scene) | D delete the last save | Q or ESC quit.

Needs the Jetson streaming: deploy/ros/start_rear_approach_stream.sh <PC_IP> 5600.
Lines on the preview mark distances ahead of the front wheels from the rear
homography (dashed beyond its 1.40 m calibration edge).
"""
import argparse
import datetime
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "integration"))
from rear_stream import ARM_CAMERA_REGION, StreamReader  # noqa: E402

W, H = 640, 480
GST = REPO / ".venv-approach" / "Lib" / "site-packages" / "gstreamer_cli" / "bin" / "gst-launch-1.0.exe"
# File x of the rear homography = distance ahead of the front-wheel line + 0.215 m.
WHEEL_TO_FILE_X_M = 0.215
CALIBRATED_MAX_M = 1.40
GUIDE_DISTANCES_M = (0.5, 0.8, 1.0, 1.4, 1.8)
# The arm camera never moves in the travel pose, so it must not count as a scene change.
IGNORE_REGION = ARM_CAMERA_REGION
DIFF_SCALE = 0.5            # compare at 320x240: enough for a far box, cheap
DIFF_PIXEL_LEVEL = 30       # grey-level change that counts as a changed pixel


def change_score(previous_gray, gray):
    """Changed pixels between two DIFF_SCALE grey frames, arm camera excluded."""
    if previous_gray is None or previous_gray.shape != gray.shape:
        return float("inf")
    diff = cv2.absdiff(previous_gray, gray)
    x1, y1, x2, y2 = (int(round(v * DIFF_SCALE)) for v in IGNORE_REGION)
    diff[y1:y2, x1:x2] = 0
    return int(np.count_nonzero(diff > DIFF_PIXEL_LEVEL))


def to_diff_gray(frame):
    small = cv2.resize(frame, None, fx=DIFF_SCALE, fy=DIFF_SCALE, interpolation=cv2.INTER_AREA)
    return cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (5, 5), 0)


def distance_line(homography, wheel_distance_m, half_width_m=0.8, steps=33):
    """Image points of the floor line wheel_distance_m ahead of the front wheels."""
    inverse = np.linalg.inv(homography)
    points = []
    for y in np.linspace(-half_width_m, half_width_m, steps):
        q = inverse @ np.array([wheel_distance_m + WHEEL_TO_FILE_X_M, y, 1.0])
        if abs(q[2]) <= 1e-9:          # homogeneous scale: either sign is a point
            continue
        u, v = q[0] / q[2], q[1] / q[2]
        if -W <= u <= 2 * W and 0 <= v <= H:
            points.append((int(round(u)), int(round(v))))
    return points


def beep():
    try:
        import winsound
        winsound.Beep(1500, 120)
    except Exception:
        print("\a", end="", flush=True)


def main(argv=None):
    today = datetime.date.today().strftime("%Y%m%d")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=Path(
        r"C:\Users\user\Documents\repo\yolo_train\dataset") / f"placement_box_robotcam_{today}" / "raw")
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--still-s", type=float, default=1.5,
                    help="how long the picture must hold still before an auto save")
    ap.add_argument("--still-max-px", type=int, default=40,
                    help="changed pixels (320x240) that still count as 'holding still'")
    ap.add_argument("--min-change-px", type=int, default=60,
                    help="changed pixels against the last save needed for a new save")
    ap.add_argument("--homography", type=Path, default=REPO / "detection/rear_ground_homography.json")
    args = ap.parse_args(argv)

    from sugarbox_ground_calibration import load_ground_calibration
    homography = load_ground_calibration(str(args.homography))[0]
    guides = [(d, distance_line(homography, d)) for d in GUIDE_DISTANCES_M]

    args.out.mkdir(parents=True, exist_ok=True)
    meta_path = args.out / "metadata.jsonl"
    saved = sorted(args.out.glob("*.jpg"))
    print(f"[capture] saving to {args.out} ({len(saved)} frames already there)")

    reader = StreamReader(args.port, GST)
    auto, tag = True, "box"
    previous_gray, last_saved_gray = None, None
    still_since, last_save_path = None, None
    window = "rear dataset capture"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame, stamp = reader.latest()
            if frame is None:
                if cv2.waitKey(100) & 0xFF in (ord("q"), 27):
                    break
                continue
            now = time.monotonic()
            gray = to_diff_gray(frame)
            motion = change_score(previous_gray, gray)
            previous_gray = gray
            still_since = (still_since or now) if motion <= args.still_max_px else None
            novelty = change_score(last_saved_gray, gray)
            stale = now - stamp > 1.0

            key = cv2.waitKey(30) & 0xFF
            want = key == ord(" ")
            if (auto and not stale and still_since is not None
                    and now - still_since >= args.still_s and novelty >= args.min_change_px):
                want = True
            if want and not stale:
                name = f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}_{tag}.jpg"
                path = args.out / name
                cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                with meta_path.open("a", encoding="utf-8") as meta:
                    meta.write(json.dumps({"file": name, "tag": tag, "auto": key != ord(" "),
                                           "stream_port": args.port, "size": [W, H]}) + "\n")
                last_saved_gray, last_save_path = gray, path
                saved.append(path)
                beep()
                print(f"[capture] {len(saved):4d} {name}")
            if key == ord("a"):
                auto = not auto
            elif key == ord("b"):
                tag = "box"
            elif key == ord("n"):
                tag = "negative"
            elif key == ord("d") and last_save_path is not None and last_save_path.exists():
                last_save_path.unlink()
                saved = [p for p in saved if p != last_save_path]
                print(f"[capture] deleted {last_save_path.name}")
                last_save_path, last_saved_gray = None, None
            elif key in (ord("q"), 27):
                break

            view = frame.copy()
            for distance, points in guides:
                if len(points) >= 2:
                    colour = (0, 200, 0) if distance <= CALIBRATED_MAX_M else (0, 160, 255)
                    if distance <= CALIBRATED_MAX_M:
                        cv2.polylines(view, [np.array(points)], False, colour, 1)
                    else:
                        for a, b in zip(points[::2], points[1::2]):
                            cv2.line(view, a, b, colour, 1)
                    visible = [p for p in points if 0 <= p[0] < W - 40]
                    if visible:
                        label_at = min(visible)
                        cv2.putText(view, f"{distance:.1f}m", (label_at[0] + 2, label_at[1] - 3),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
            status = (f"{'AUTO' if auto else 'MANUAL'} tag={tag} saved={len(saved)} "
                      f"motion={motion} new={novelty}{' STALE' if stale else ''}")
            cv2.rectangle(view, (0, 0), (W, 22), (0, 0, 0), -1)
            cv2.putText(view, status, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255) if stale else (255, 255, 255), 1)
            cv2.imshow(window, view)
    finally:
        reader.close()
        cv2.destroyAllWindows()
        print(f"[capture] {len(saved)} frames in {args.out}")


if __name__ == "__main__":
    main()
