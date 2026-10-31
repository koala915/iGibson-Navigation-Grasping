"""Draft YOLO boxes for frames saved by capture_rear_dataset.py; a person then reviews.

The robot stood still while one object was moved around, so within each run of
frames shot from one heading the background is fixed. Per run: the per-pixel
median is the empty scene, the object is what differs from it, and a colour
gate keeps shadows and passing legs out. SAM2, prompted with that box, then
tightens it to the object's outline. Frames it cannot settle are flagged for a
human instead of guessed.

Writes <dataset>/images + labels (class 0) and review sheets with every box
drawn, numbered like the capture order, flagged frames in red.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
THUMB = (160, 120)
# A new heading moves every pixel; the object and auto exposure do not. So the
# test is the MEDIAN grey change between consecutive thumbnails after removing
# the mean: 2026-10-17 it stayed 0.6-4.9 while the box moved and jumped to 6.3-8.1
# when the robot turned or someone walked in.
NEW_HEADING_DIFF = 5.5
# The empty scene for a frame is the median of up to this many neighbours on
# either side within its heading; slow drift then does not blur it.
BACKGROUND_NEIGHBOURS = 5
MIN_RUN_FOR_MEDIAN = 4
DIFF_LEVEL = 40          # per-channel change against the empty scene
MIN_AREA_PX = 40
# Cardboard under this arena's light, measured on 40 good boxes (2026-10-17):
# hue 0..10 with some wrap past 175 (OpenCV 0..180), saturation 49..96, value
# 56..108. The yellow floor tape sits at hue 24..35 and was pulled into boxes
# when the limit was 30. The brick floor past the walls is in range too, which
# is why the colour-only fallback excludes whatever stays brown all run.
HUE_MAX, HUE_WRAP_MIN, MIN_SAT, VAL_RANGE = 14, 170, 45, (25, 230)
IGNORE_REGION = (295, 395, 405, 480)      # the arm camera in travel pose
MIN_MASK_BROWN = 0.25     # a SAM mask that is mostly not cardboard is the wrong object
MAX_BOX_AREA = 0.25       # of the frame; bigger means it grabbed floor or wall


def brown_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (((h <= HUE_MAX) | (h >= HUE_WRAP_MIN)) & (s >= MIN_SAT)
            & (v >= VAL_RANGE[0]) & (v <= VAL_RANGE[1]))


def headings(frames):
    """Split frame indices into runs shot from one heading."""
    thumbs = [cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), THUMB).astype(np.float32)
              for f in frames]
    thumbs = [t - t.mean() for t in thumbs]
    runs, start = [], 0
    for i in range(1, len(frames)):
        if float(np.median(np.abs(thumbs[i] - thumbs[i - 1]))) > NEW_HEADING_DIFF:
            runs.append(list(range(start, i)))
            start = i
    runs.append(list(range(start, len(frames))))
    return runs


def best_blob(mask, colour):
    """Largest blob weighted by cardboard colour: (box, a cardboard pixel in it)."""
    mask = mask.astype(np.uint8)
    x1, y1, x2, y2 = IGNORE_REGION
    mask[y1:y2, x1:x2] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    best, best_score = None, 0.0
    for label in range(1, count):
        x, y, w, h, area = stats[label]
        if area < MIN_AREA_PX:
            continue
        inside = labels == label
        brown = float(np.mean(colour[inside]))
        score = area * brown
        if brown >= 0.25 and score > best_score:
            ys, xs = np.nonzero(inside & colour)
            # The median cardboard pixel, not the blob centre: an open box's
            # centre is its dark inside, and SAM then segments only that.
            point = (int(np.median(xs)), int(np.median(ys)))
            best, best_score = ((int(x), int(y), int(x + w), int(y + h)), point), score
    return best


def mask_box(mask, colour, frame_area):
    """Tight box of a SAM mask if it looks like the cardboard object, else None."""
    ys, xs = np.nonzero(mask)
    if xs.size < MIN_AREA_PX or float(np.mean(colour[mask])) < MIN_MASK_BROWN:
        return None
    box = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)
    if (box[2] - box[0]) * (box[3] - box[1]) > MAX_BOX_AREA * frame_area:
        return None
    return box


def sam_refine(sam, frame, colour, coarse, point):
    """Whole object from SAM2: a point on the cardboard first, the coarse box second."""
    frame_area = frame.shape[0] * frame.shape[1]
    for prompt in ({"points": [list(point)], "labels": [1]}, {"bboxes": [list(coarse)]}):
        result = sam(frame, verbose=False, **prompt)[0]
        if result.masks is None:
            continue
        candidates = [mask_box(m.cpu().numpy() > 0.5, colour, frame_area)
                      for m in result.masks.data]
        candidates = [c for c in candidates if c is not None and iou(c, coarse) > 0.0]
        if candidates:
            # The candidate overlapping the evidence most is the object.
            return max(candidates, key=lambda c: iou(c, coarse))
    return None


def iou(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def yolo_line(box, width, height):
    x1, y1, x2, y2 = box
    return "0 {:.6f} {:.6f} {:.6f} {:.6f}".format(
        (x1 + x2) / 2 / width, (y1 + y2) / 2 / height, (x2 - x1) / width, (y2 - y1) / height)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dataset", type=Path, help="folder holding raw/ from the capture tool")
    ap.add_argument("--sam", type=Path, default=REPO / "detection/models/sam2.1_b.pt")
    args = ap.parse_args(argv)

    files = sorted((args.dataset / "raw").glob("*.jpg"))
    frames = [cv2.imread(str(f)) for f in files]
    tags = {}
    meta_path = args.dataset / "raw" / "metadata.jsonl"
    if meta_path.exists():
        for line in meta_path.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            tags[row["file"]] = row.get("tag", "box")

    from ultralytics import SAM
    sam = SAM(str(args.sam))
    results = []
    runs = headings(frames)
    print("[autolabel] {} frames, {} headings: {}".format(
        len(frames), len(runs), [len(r) for r in runs]))
    for run in runs:
        # Brown in most of the run = part of the scene (brick floor past the
        # walls), never the object, which moves. Keeps the colour fallback honest.
        static_brown = (np.mean([brown_mask(frames[j]) for j in run], axis=0) >= 0.5
                        if len(run) >= 3 else None)
        for position, i in enumerate(run):
            window = run[max(0, position - BACKGROUND_NEIGHBOURS):position + BACKGROUND_NEIGHBOURS + 1]
            if len(window) < MIN_RUN_FOR_MEDIAN:
                # A short run: the other frames alone are the best empty scene
                # (2026-10-17: the last two, one with the box and one without).
                window = [j for j in window if j != i]
            background = (np.median(np.stack([frames[j] for j in window]), axis=0).astype(np.uint8)
                          if window else None)
            frame = frames[i]
            colour = brown_mask(frame)
            found, source = None, "colour"
            if background is not None:
                changed = np.max(cv2.absdiff(frame, background), axis=2) > DIFF_LEVEL
                found, source = best_blob(changed, colour), "background"
            if found is None and static_brown is not None:
                # A box left in one spot for several frames is in the median too.
                found, source = best_blob(colour & ~static_brown, colour), "colour"
            coarse, point = found if found else (None, None)
            box = sam_refine(sam, frame, colour, coarse, point) if coarse else None
            flags = []
            if tags.get(files[i].name) == "negative":
                box, flags = None, ["tagged negative"]
            elif coarse is None:
                flags.append("nothing found")
            elif box is None:
                box = coarse
                flags.append("SAM found no cardboard-like object, coarse box kept")
            if source == "colour" and box is not None:
                flags.append("found by colour only")
            results.append({"index": i, "file": files[i].name, "box": box, "flags": flags})

    images, labels = args.dataset / "images", args.dataset / "labels"
    images.mkdir(exist_ok=True)
    labels.mkdir(exist_ok=True)
    for row in results:
        shutil.copy2(files[row["index"]], images / row["file"])
        height, width = frames[row["index"]].shape[:2]
        (labels / (Path(row["file"]).stem + ".txt")).write_text(
            (yolo_line(row["box"], width, height) + "\n") if row["box"] else "", encoding="utf-8")
    (args.dataset / "autolabel_report.json").write_text(
        json.dumps(results, indent=1, ensure_ascii=False), encoding="utf-8")

    thumbs = []
    for row in results:
        view = frames[row["index"]].copy()
        colour = (0, 0, 255) if row["flags"] else (0, 255, 0)
        if row["box"]:
            cv2.rectangle(view, row["box"][:2], row["box"][2:], colour, 2)
        view = cv2.resize(view, (320, 240))
        cv2.putText(view, str(row["index"]), (4, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
        thumbs.append(view)
    per_sheet, cols = 24, 4
    for sheet_no, start in enumerate(range(0, len(thumbs), per_sheet)):
        chunk = thumbs[start:start + per_sheet]
        while len(chunk) % cols:
            chunk.append(np.zeros_like(thumbs[0]))
        sheet = np.vstack([np.hstack(chunk[r:r + cols]) for r in range(0, len(chunk), cols)])
        cv2.imwrite(str(args.dataset / f"review_{sheet_no + 1}.jpg"), sheet)
    flagged = [r for r in results if r["flags"]]
    print("[autolabel] boxes {} / {}, flagged {}".format(
        sum(1 for r in results if r["box"]), len(results), len(flagged)))
    for row in flagged:
        print("  #{} {} {}".format(row["index"], row["file"], "; ".join(row["flags"])))


if __name__ == "__main__":
    main()
