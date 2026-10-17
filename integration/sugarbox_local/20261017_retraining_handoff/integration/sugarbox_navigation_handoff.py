"""Pure navigation handoff checks; no arm motion, camera, ROS or serial I/O."""
import math


NAV_POSE = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)
HANDOFF_EXIT_CODE = 42


def should_handoff(now, last_rear_usable, distance, clearance_ok, has_memory,
                   rear_near_edge=False):
    return (has_memory and clearance_ok and last_rear_usable > 0
            and (rear_near_edge or now - last_rear_usable >= 0.6)
            and 0 < distance <= 0.85)


def rear_at_handoff_edge(result, stamp, now, boundary):
    if (boundary is None or result is None or not math.isfinite(stamp)
            or not 0 <= now-stamp <= .35):
        return False
    detection = result.get("det") or {}
    box = detection.get("bbox")
    confidence = detection.get("conf", 0)
    if (box is None or len(box) != 4 or not all(math.isfinite(v) for v in box)
            or not math.isfinite(confidence) or confidence < .45):
        return False
    x1, y1, x2, y2 = box
    return 0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480 and y2 >= boundary
