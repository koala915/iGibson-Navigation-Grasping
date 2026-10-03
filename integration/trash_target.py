"""Adapter from the offboard SAM2 publisher to the mission's detection tuple.

``detection/rear_cam_sam2_publisher.py`` runs on a development machine and
publishes ``/trash_target/detection`` (std_msgs/String carrying JSON).  This
module turns one of those payloads into the ``(found, dist_front, offset)``
triple that ``vision_grasp_pipeline._detect_rear`` returns, so the mission loop
can consume either source without knowing which one it has.

THE SIGN FLIP IS THE WHOLE POINT OF THIS FILE.

    publisher   ``coordinate_convention: "x_forward_y_left"``  -> y is LEFT-positive
    pipeline    ``estimate_offset_x``  "> 0 object to the right" -> RIGHT-positive

They are opposite.  Feeding ``object_y_base`` through unchanged makes the robot
turn away from the target every single time, and nothing downstream can detect
it: both are plain floats in the same range, so the value stays plausible while
being backwards.  ``to_rear_detection`` negates, and
``test_trash_target.py`` pins that with a left-of-centre object.

Everything here is pure: no ROS, no camera, no torch.  Run the tests on a
development machine.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Tuple

# The publisher sends at PUBLISH_INTERVAL_SEC = 0.20, so anything older than
# this is several missed frames rather than jitter.  Matches the grasp side's
# detection_stale_timeout_sec (1.0) -- an offboard link that has gone quiet must
# stop the robot acting on the last thing it said, not keep steering by it.
DEFAULT_MAX_AGE_S = 1.0

# The frame the publisher promises.  Anything else is a different robot or a
# miscalibrated homography, and steering by it would be confident nonsense.
EXPECTED_FRAME = "base_footprint"
EXPECTED_CONVENTION = "x_forward_y_left"

NOT_FOUND: Tuple[bool, float, float] = (False, -1.0, 0.0)


@dataclass(frozen=True)
class TrashTarget:
    """One accepted detection, already in the robot's base frame."""

    valid: bool
    x_forward_m: float          # +X ahead of base_footprint
    y_left_m: float             # +Y to the robot's LEFT (publisher convention)
    distance_m: float
    stamp_unix: float
    reason: str = ""
    source: str = ""
    bbox_xyxy: Optional[Tuple[float, float, float, float]] = None
    mask_bottom_u: Optional[float] = None
    mask_bottom_v: Optional[float] = None
    on_floor: Optional[bool] = None
    observation_age_s: float = 0.0

    def age_s(self, now_unix: float) -> float:
        return float(now_unix) - self.stamp_unix


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _bbox(value: Any) -> Optional[Tuple[float, float, float, float]]:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    parsed = tuple(_finite(part) for part in value)
    if any(part is None for part in parsed):
        return None
    x1, y1, x2, y2 = parsed
    if x2 <= x1 or y2 <= y1:
        return None
    return float(x1), float(y1), float(x2), float(y2)


def parse_trash_target(payload: Any) -> Optional[TrashTarget]:
    """Parse one ``/trash_target/detection`` message body.

    Accepts the JSON text, an already-decoded mapping, or the rosbridge
    envelope ``{"data": "<json>"}``.  Returns None for anything malformed --
    a dropped message is a missed frame, and the caller already treats missing
    detections as "keep patrolling".

    A payload with ``valid: false`` is NOT malformed: the publisher emits those
    deliberately (waiting for stability, geometry out of range, shutting down),
    and they are parsed so the caller can see the reason.
    """
    if isinstance(payload, (bytes, bytearray)):
        try:
            payload = payload.decode("utf-8")
        except UnicodeDecodeError:
            return None

    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except (ValueError, TypeError):
            return None

    if not isinstance(payload, Mapping):
        return None

    # rosbridge wraps std_msgs/String as {"data": "..."}.
    if "data" in payload and "valid" not in payload:
        return parse_trash_target(payload.get("data"))

    valid = bool(payload.get("valid", False))
    reason = str(payload.get("reason", ""))
    source = str(payload.get("source", ""))
    stamp = _finite(payload.get("timestamp_unix"))

    if stamp is None:
        return None

    if not valid:
        # No coordinates are promised on an invalid payload; do not invent any.
        return TrashTarget(False, float("nan"), float("nan"), float("nan"),
                           stamp, reason or "invalid", source)

    frame = str(payload.get("frame_id", ""))
    convention = str(payload.get("coordinate_convention", ""))
    if frame != EXPECTED_FRAME or convention != EXPECTED_CONVENTION:
        # Refuse rather than guess. A frame mismatch is exactly the failure this
        # module exists to prevent, and it cannot be detected further downstream.
        return TrashTarget(False, float("nan"), float("nan"), float("nan"),
                           stamp,
                           f"frame mismatch: {frame!r}/{convention!r}", source)

    x = _finite(payload.get("object_x_base"))
    y = _finite(payload.get("object_y_base"))
    if x is None or y is None:
        return TrashTarget(False, float("nan"), float("nan"), float("nan"),
                           stamp, "non-finite coordinates", source)

    distance = _finite(payload.get("distance_m"))
    if distance is None:
        distance = math.hypot(x, y)

    observation_age = _finite(payload.get("observation_age_s"))
    if observation_age is None or observation_age < 0.0:
        observation_age = 0.0
    on_floor_raw = payload.get("on_floor")
    on_floor = on_floor_raw if isinstance(on_floor_raw, bool) else None

    return TrashTarget(
        True, x, y, distance, stamp, reason or "accepted", source,
        bbox_xyxy=_bbox(payload.get("bbox_xyxy")),
        mask_bottom_u=_finite(payload.get("mask_bottom_u")),
        mask_bottom_v=_finite(payload.get("mask_bottom_v")),
        on_floor=on_floor,
        observation_age_s=observation_age,
    )


def to_rear_detection(target: Optional[TrashTarget],
                      now_unix: float,
                      *,
                      max_age_s: float = DEFAULT_MAX_AGE_S,
                      compensate_vx: float = 0.0,
                      compensate_wz: float = 0.0,
                      max_compensation_s: float = 1.2,
                      ) -> Tuple[bool, float, float]:
    """``(found, dist_front_m, offset_m)`` in ``_detect_rear``'s convention.

    ``offset`` comes back RIGHT-positive, negated from the publisher's
    left-positive ``object_y_base``.  See the module docstring.

    Fails closed: no target, an invalid one, a stale one, or one behind the
    robot all report "not found", which the mission loop reads as "keep
    patrolling" rather than as a reason to drive somewhere.
    """
    if target is None or not target.valid:
        return NOT_FOUND

    age = target.age_s(now_unix)
    if not math.isfinite(age) or age > float(max_age_s):
        return NOT_FOUND

    # A clock that disagrees between machines can hand us a future stamp. Accept
    # a little (NTP skew) and reject the rest -- a wildly future stamp would
    # otherwise never go stale.
    if age < -float(max_age_s):
        return NOT_FOUND

    if not math.isfinite(target.x_forward_m) or target.x_forward_m <= 0.0:
        # Behind or level with the base origin. The rear-follow controller has
        # no meaning for that and would drive at a negative distance.
        return NOT_FOUND

    x = float(target.x_forward_m)
    y_left = float(target.y_left_m)

    # The publisher timestamps how long the frame spent waiting/in inference.
    # Propagate that old robot-relative point to now with the latest measured
    # chassis velocity. This is deliberately bounded: a delayed frame is safer
    # to reject through the normal freshness gate than to extrapolate far.
    delay = min(max(0.0, float(target.observation_age_s)) + max(0.0, age),
                max(0.0, float(max_compensation_s)))
    vx = _finite(compensate_vx) or 0.0
    wz = _finite(compensate_wz) or 0.0
    if delay > 0.0 and (vx != 0.0 or wz != 0.0):
        x -= vx * delay
        phi = wz * delay
        c, s = math.cos(phi), math.sin(phi)
        x, y_left = c * x + s * y_left, -s * x + c * y_left
        if x <= 0.0:
            return NOT_FOUND

    return True, x, -y_left
