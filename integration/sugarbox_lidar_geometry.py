"""Pure ROS LaserScan geometry for the standalone sugarbox approach.

The measured TF is base_link -> laser: yaw 180 degrees, x +0.10 m.
PPO point coordinates use that complete transform, with robot x forward and y
left. Safety retains its measured sensor-origin sectors/ranges after applying
only the yaw correction; translation must not relax the existing thresholds.
Unknown returns cannot be filled from a neighbouring hit.
"""
from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class LidarGeometry:
    yaw_offset_deg: float = 180.0
    forward_offset_m: float = 0.10

    def __post_init__(self):
        if not all(math.isfinite(float(value)) for value in
                   (self.yaw_offset_deg, self.forward_offset_m)):
            raise ValueError("LiDAR mounting parameters must be finite")


@dataclass(frozen=True)
class RobotScanSample:
    angle_rad: float
    range_m: Optional[float]
    sensor_range_m: Optional[float] = None
    sensor_angle_rad: Optional[float] = None


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def robot_scan_samples(ranges: Sequence[float], angle_min: float,
                       angle_increment: float,
                       geometry: LidarGeometry) -> Tuple[RobotScanSample, ...]:
    """Rotate and translate scan returns into the measured robot frame.

    Invalid returns retain their nominal rotated beam angle with no range.
    Their unknown range cannot provide the distance needed for translation.
    This preserves no-hit policy samples instead of borrowing valid hits.
    The yaw-corrected sensor angle and sensor range are retained separately:
    safety sectors/thresholds were measured from the sensor. Translation must
    not move a close return between front and side sectors or increase range.
    The 2 cm invalid-return cutoff matches nav_rl's canonical transform.
    """
    if (not math.isfinite(float(angle_min))
            or not math.isfinite(float(angle_increment))
            or float(angle_increment) == 0.0):
        raise ValueError("LaserScan angles must be finite with nonzero increment")
    yaw = math.radians(geometry.yaw_offset_deg)
    samples = []
    for index, value in enumerate(ranges):
        angle = wrap_angle(float(angle_min) + index * float(angle_increment) + yaw)
        try:
            distance = float(value)
        except (TypeError, ValueError):
            distance = float("nan")
        if not math.isfinite(distance) or distance <= 0.02:
            samples.append(RobotScanSample(angle, None, None, angle))
            continue
        x = distance * math.cos(angle) + geometry.forward_offset_m
        y = distance * math.sin(angle)
        samples.append(RobotScanSample(math.atan2(y, x), math.hypot(x, y),
                                      distance, angle))
    return tuple(samples)


def nearest_policy_ranges(samples: Sequence[RobotScanSample],
                          policy_angles_deg: Sequence[float], *,
                          max_range_m: float, distance_scale: float) -> Tuple[float, ...]:
    """Keep nearest-sample policy extraction in right-to-left beam order.

    Samples outside a beam's angular cell do not fill an unobserved cell.
    This matters for incomplete scans; the deployed TG30 supplies a full turn.
    Range scaling remains confined to the PPO observation.
    """
    angles = tuple(math.radians(float(value)) for value in policy_angles_deg)
    half_cell = (abs(angles[1] - angles[0]) / 2.0
                 if len(angles) > 1 else 0.0)
    ordered = sorted(samples, key=lambda sample: sample.angle_rad)
    sample_angles = [sample.angle_rad for sample in ordered]
    output = []
    for target in angles:
        index = bisect_left(sample_angles, target)
        candidates = ([ordered[index % len(ordered)], ordered[index - 1],
                       ordered[0], ordered[-1]] if ordered else [])
        nearest = min(candidates,
                      key=lambda sample: abs(wrap_angle(sample.angle_rad - target)),
                      default=None)
        if (nearest is None or nearest.range_m is None or nearest.range_m <= 0.0
                or abs(wrap_angle(nearest.angle_rad - target)) > half_cell + 1e-12):
            output.append(float(max_range_m))
        else:
            output.append(min(float(nearest.range_m) * distance_scale,
                              float(max_range_m)))
    return tuple(output)
