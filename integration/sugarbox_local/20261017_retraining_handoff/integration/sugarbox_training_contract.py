"""Offline reference, not a motor controller or a physical safety layer.

Source: koala915/igibson_x3_test at
12f53c7d4e0c1ed06ac3a13def711e63aed01288, navigation_env_room.py
and the hash-matched DOORWAY_FT training_config.json.
Physical ray projection and frame verification are deliberately not inferred.
"""
from collections import deque
import math

import numpy as np


SOURCE_COMMIT = "12f53c7d4e0c1ed06ac3a13def711e63aed01288"
# User-reported mounting: lidar is approximately 9cm ahead of body center.
LIDAR_CENTER_OFFSET_X_M = 0.09
GOAL_ORIGIN_OFFSET_X_M = 0.135


def planar_speed(vx, vy):
    if not all(math.isfinite(v) for v in (vx, vy)):
        raise ValueError("Non-finite feedback velocity")
    return math.hypot(vx, vy)


def center_relative_point(x, y, origin_x, origin_y):
    """Translate an explicitly measured origin into the body-center frame."""
    if not all(math.isfinite(v) for v in (x, y, origin_x, origin_y)):
        raise ValueError("Non-finite geometry")
    return x + origin_x, y + origin_y


def center_relative_goal(front_x, left_y):
    return center_relative_point(front_x, left_y, GOAL_ORIGIN_OFFSET_X_M, 0.0)


def center_relative_hit(distance, body_angle, sensor_x=LIDAR_CENTER_OFFSET_X_M, sensor_y=0.0):
    """Do not just add sensor offset to a polar range; angle also changes."""
    if not math.isfinite(distance) or distance <= 0 or not math.isfinite(body_angle):
        raise ValueError("Requires a valid physical hit, not a no-hit sentinel")
    x, y = center_relative_point(distance * math.cos(body_angle),
                                 distance * math.sin(body_angle), sensor_x, sensor_y)
    return math.hypot(x, y), math.atan2(y, x)


def scan_to_center_rays(scan, reflect_front_back):
    """Approximate training rays from the existing, verified physical scan frame.

    Translation changes angles, so reselect angular neighbors in CENTER frame.
    This cannot reconstruct occluded geometry or faithfully emulate rays that
    start outside the training radius. Physical raw safety metrics stay separate.
    """
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    if ranges.ndim != 1 or not ranges.size or not math.isfinite(scan.angle_increment) or scan.angle_increment == 0:
        raise ValueError("Invalid scan geometry")
    if not math.isfinite(scan.angle_min):
        raise ValueError("Invalid scan origin")
    raw_angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    angles = math.pi - raw_angles if reflect_front_back else raw_angles
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    valid = np.isfinite(ranges) & (ranges > max(0.0, scan.range_min))
    valid &= np.abs(angles) <= math.pi / 2 + 1e-9
    if scan.range_max > 0:
        valid &= ranges <= scan.range_max
    # No-hit samples retain angular coverage but always return max distance.
    no_hit = ~valid | (ranges >= 4.0)
    d = np.where(no_hit, 4.0, ranges)
    x = d * np.cos(angles) + LIDAR_CENTER_OFFSET_X_M
    y = d * np.sin(angles)
    center_angles = np.arctan2(y, x)
    center_ranges = np.where(no_hit, 4.0, np.clip(np.hypot(x, y), 0.33, 4.0))
    # Rear chassis returns cannot become policy hits through translation.
    front_coverage = np.abs(angles) <= math.pi / 2 + 1e-9
    rays = np.full(48, 4.0, dtype=np.float32)
    half_spacing = math.pi / 47 / 2
    for i, ray_angle in enumerate(np.linspace(-math.pi / 2, math.pi / 2, 48)):
        errors = np.abs(center_angles - ray_angle)
        candidates = np.flatnonzero(front_coverage & (errors <= half_spacing + 1e-9))
        if candidates.size:
            index = candidates[np.argmin(errors[candidates])]
            rays[i] = center_ranges[index]
    return rays


def training_observation(goal_x, goal_y, vx, vy, wz, previous_raw_action,
                         center_ray_distances, distance_scale=1.0):
    """Input rays must already represent CENTER-frame directions, not raw scan.

    Clipping to the training support here is an offline comparison only. It
    does not emulate rays that START outside radius 0.33m and must never be
    used to discard raw near-obstacle evidence in physical safety checks.
    """
    rays = np.asarray(center_ray_distances, dtype=np.float32)
    if not math.isfinite(distance_scale) or distance_scale <= 0:
        raise ValueError("Invalid policy distance scale")
    previous = np.asarray(previous_raw_action, dtype=np.float32)
    if rays.shape != (48,) or previous.shape != (2,):
        raise ValueError("Expected 48 center rays and two raw actions")
    if not np.isfinite(rays).all() or (rays < 0).any():
        raise ValueError("Invalid center rays; explicitly represent no-hit as 4m")
    if not np.isfinite(previous).all() or not all(math.isfinite(v) for v in (goal_x, goal_y, wz)):
        raise ValueError("Non-finite observation")
    bearing = math.atan2(goal_y, goal_x)
    obs = np.empty(55, dtype=np.float32)
    obs[:7] = [math.hypot(goal_x, goal_y), math.sin(bearing), math.cos(bearing),
               planar_speed(vx, vy), wz, previous[0], previous[1]]
    obs[7:] = np.minimum(np.clip(rays, 0.33, 4.0) * distance_scale, 4.0)
    return obs


class TrainingActionReference:
    """Training shaping with an optional delay override for physical trials."""

    def __init__(self, delay_steps=2):
        if type(delay_steps) is not int or delay_steps < 0:
            raise ValueError("delay_steps must be a nonnegative integer")
        self.delay_steps = delay_steps
        self.reset()

    def reset(self):
        length = self.delay_steps + 1
        self.buffer = deque((np.zeros(2) for _ in range(length)), maxlen=length)

    def step(self, raw_action, min_lidar, goal_distance, bearing, vx, vy):
        raw = np.asarray(raw_action, dtype=np.float64)
        scalars = (min_lidar, goal_distance, bearing, vx, vy)
        if raw.shape != (2,) or not np.isfinite(raw).all():
            raise ValueError("Invalid raw action")
        if not all(math.isfinite(v) for v in scalars) or min_lidar < 0 or goal_distance < 0:
            raise ValueError("Invalid plant inputs")
        self.buffer.append(raw.copy())
        throttle, angular = np.clip(self.buffer[0], -1.0, 1.0)
        assist = (goal_distance > 0.9 and math.cos(bearing) > 0.95
                  and min_lidar > 0.6 and planar_speed(vx, vy) < 0.04
                  and throttle < 0.28)
        if assist:
            throttle = 0.28
        linear_scale = angular_scale = 1.0
        if min_lidar < 0.66:
            ratio = max(0.0, min(1.0, min_lidar / 0.66))
            linear_scale = 0.24 + 0.76 * ratio
            angular_scale = 0.9 + 0.1 * ratio
        return {
            "vx": float(throttle * linear_scale * (0.5 if throttle >= 0 else 0.15)),
            "wz": float(angular * 1.2 * angular_scale),
            "delayed_action": self.buffer[0].copy(),
            "stall_assist": bool(assist),
            "linear_scale": linear_scale,
            "angular_scale": angular_scale,
        }


def limited_reference_twist(vx, wz, forward_limit=0.15, reverse_limit=0.08,
                            yaw_limit=0.78):
    """Offline proportional limiting; not a verified hardware velocity map."""
    values = (vx, wz, forward_limit, reverse_limit, yaw_limit)
    if not all(math.isfinite(v) for v in values) or min(values[2:]) <= 0:
        raise ValueError("Invalid velocity or limits")
    linear_limit = forward_limit if vx >= 0 else reverse_limit
    scale = min(1.0, linear_limit / abs(vx) if vx else 1.0,
                yaw_limit / abs(wz) if wz else 1.0)
    return vx * scale, wz * scale
