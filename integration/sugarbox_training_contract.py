"""Make the doorway PPO see and act the way it was trained (enoch20050427, 2026-10-11).

Source of the contract: koala915/igibson_x3_test at
12f53c7d4e0c1ed06ac3a13def711e63aed01288, navigation_env_room.py and the
hash-matched DOORWAY_FT training_config.json. Audit and physical trials:
docs/operations/sugarbox_local_20261011_policy_contract_audit.md on enoch's
branch codex/sugarbox-local-20261011.

Training differs from the older runtime in four ways this module restores:
the goal is relative to the chassis centre, speed is unsigned planar speed,
the 48 rays start at the chassis centre (floor 0.33 m, no-hit 4 m), and the
plant shapes actions near obstacles (plus a stall assist).

One deliberate change from enoch's snapshot (2026-10-12): rays come from the
team's measured scan geometry (sugarbox_lidar_geometry: TF yaw 180 deg, LiDAR
+0.10 m ahead of base_link). His snapshot mirrored the scan front-to-back
instead of rotating it; the two agree straight ahead and disagree on which
side an obstacle is on.
"""
from collections import deque
import math

import numpy as np


SOURCE_COMMIT = "12f53c7d4e0c1ed06ac3a13def711e63aed01288"
# The approach measures the box from the E1 / gripper origin; the policy was
# trained on goals relative to the chassis centre, 0.135 m further back.
GOAL_ORIGIN_OFFSET_X_M = 0.135
RAY_FLOOR_M = 0.33          # training rays start on a 0.33 m radius
RAY_NO_HIT_M = 4.0
POLICY_RAY_COUNT = 48
# Chassis footprint around the centre (30 cm wheel span, about 26 cm wide, plus a
# margin). Beams behind the sensor that end inside it hit the robot itself: on
# 2026-10-12 the mast, arm base and wiring answered at |y| <= 0.14 m.
CHASSIS_HALF_LENGTH_M = 0.15
CHASSIS_HALF_WIDTH_M = 0.15


def planar_speed(vx, vy):
    if not all(math.isfinite(v) for v in (vx, vy)):
        raise ValueError("Non-finite feedback velocity")
    return math.hypot(vx, vy)


def center_relative_goal(front_x, left_y):
    """E1-origin box position -> chassis-centre position (a pure translation)."""
    if not all(math.isfinite(v) for v in (front_x, left_y)):
        raise ValueError("Non-finite geometry")
    return front_x + GOAL_ORIGIN_OFFSET_X_M, left_y


def center_rays(samples):
    """48 training rays (right -90 deg .. left +90 deg) from robot-frame samples.

    `samples` are sugarbox_lidar_geometry.RobotScanSample: already rotated and
    translated into the chassis frame.

    Beams behind the sensor count too, unless they end on the chassis. The
    sensor sits 0.10 m ahead of the centre, so whatever is beside the robot
    between the two (centre rays near +-90 deg) is only seen by those beams.
    Dropping all of them (the first version) showed a wall 35 cm to the right
    as "nothing there" on 2026-10-12, and the policy steered toward it.

    A ray from the centre stops at the first surface in its direction, so each
    ray takes the CLOSEST hit within half a ray spacing of it, or 4 m if none.
    Not the single nearest beam (enoch's snapshot): once the origin moves back
    0.10 m, near hits are squeezed into a narrower angle and interleave with
    far no-hit beams, and the nearest beam then drops a real obstacle at
    random (tests/test_sugarbox_training_contract.py shows one at 0.56 m).
    """
    half_spacing = math.pi / (POLICY_RAY_COUNT - 1) / 2

    def counts(s):
        if s.range_m is None or not math.isfinite(s.range_m) or s.sensor_angle_rad is None:
            return False
        if abs(s.sensor_angle_rad) <= math.pi / 2 + 1e-9:
            return True
        x, y = s.range_m * math.cos(s.angle_rad), s.range_m * math.sin(s.angle_rad)
        return abs(x) > CHASSIS_HALF_LENGTH_M or abs(y) > CHASSIS_HALF_WIDTH_M

    hits = [(s.angle_rad, float(s.range_m)) for s in samples if counts(s)]
    rays = np.full(POLICY_RAY_COUNT, RAY_NO_HIT_M, dtype=np.float32)
    if not hits:
        return rays
    angles = np.array([a for a, _ in hits])
    ranges = np.array([r for _, r in hits])
    for i, ray_angle in enumerate(np.linspace(-math.pi / 2, math.pi / 2, POLICY_RAY_COUNT)):
        errors = np.abs(np.arctan2(np.sin(angles - ray_angle), np.cos(angles - ray_angle)))
        in_slice = errors <= half_spacing + 1e-9
        if in_slice.any():
            rays[i] = min(RAY_NO_HIT_M, max(RAY_FLOOR_M, float(ranges[in_slice].min())))
    return rays


def training_observation(goal_x, goal_y, vx, vy, wz, previous_raw_action,
                         center_ray_distances, distance_scale=1.0):
    """55-D observation in the training layout; goal and rays in the centre frame."""
    rays = np.asarray(center_ray_distances, dtype=np.float32)
    if not math.isfinite(distance_scale) or distance_scale <= 0:
        raise ValueError("Invalid policy distance scale")
    previous = np.asarray(previous_raw_action, dtype=np.float32)
    if rays.shape != (POLICY_RAY_COUNT,) or previous.shape != (2,):
        raise ValueError("Expected 48 center rays and two raw actions")
    if not np.isfinite(rays).all() or (rays < 0).any():
        raise ValueError("Invalid center rays; explicitly represent no-hit as 4m")
    if not np.isfinite(previous).all() or not all(math.isfinite(v) for v in (goal_x, goal_y, wz)):
        raise ValueError("Non-finite observation")
    bearing = math.atan2(goal_y, goal_x)
    obs = np.empty(55, dtype=np.float32)
    obs[:7] = [math.hypot(goal_x, goal_y), math.sin(bearing), math.cos(bearing),
               planar_speed(vx, vy), wz, previous[0], previous[1]]
    obs[7:] = np.minimum(np.clip(rays, RAY_FLOOR_M, RAY_NO_HIT_M) * distance_scale, RAY_NO_HIT_M)
    return obs


class TrainingActionReference:
    """The training plant's action shaping, with the delay set by the caller.

    enoch's best trial used delay 0 ("avoidance much better, twitching almost
    gone"); training itself used 2.
    """

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
        # Training's stall assist: far, straight ahead, clear and barely moving.
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
    """Scale vx and wz together so the commanded curvature survives the limits."""
    values = (vx, wz, forward_limit, reverse_limit, yaw_limit)
    if not all(math.isfinite(v) for v in values) or min(values[2:]) <= 0:
        raise ValueError("Invalid velocity or limits")
    linear_limit = forward_limit if vx >= 0 else reverse_limit
    scale = min(1.0, linear_limit / abs(vx) if vx else 1.0,
                yaw_limit / abs(wz) if wz else 1.0)
    return vx * scale, wz * scale
