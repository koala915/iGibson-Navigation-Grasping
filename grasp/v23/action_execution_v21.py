"""Stateful v21 policy-action adapter shared with the real-robot runtime.

The PPO action is not sent directly to the joints.  This reproduces the
pre-FK-clamp portion of ``robot_grasp_env._apply_action`` without importing the
simulator:

* the raw action is clipped and retained for observation ``prev_action``;
* a separate filtered action persists across policy steps;
* smoothing and arm rate limits depend on target distance;
* the arm rate limit is additionally reduced near the floor;
* the gripper close command is gated by finger-pad geometry/contact.

A deployment must still run its preventive FK floor guard on the returned arm
and gripper targets before sending either target to hardware.  That hardware
guard is intentionally stricter than the simulator's recoverable clamp.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence, Tuple

import numpy as np

ARM_STEP_RAD = 0.08
GRIP_STEP_RAD = 0.10
GRIP_OPEN_RAD = -1.5
GRIP_CLOSED_RAD = 0.0
SMOOTH_ALPHA_FAR = 0.20
SMOOTH_ALPHA_NEAR = 0.40
SMOOTH_ALPHA_VERY_NEAR = 0.50
NEAR_TARGET_M = 0.05
VERY_NEAR_TARGET_M = 0.03
NEAR_STEP_SCALE = 0.45
VERY_NEAR_STEP_SCALE = 0.25
MIN_CLOSE_ACTION = 0.45
FLOOR_SLOW_BAND_M = 0.05
FLOOR_SLOW_MIN = 0.06


@dataclass
class ActionExecutionState:
    """State that must persist for one grasp episode."""

    filtered_action: np.ndarray = field(
        default_factory=lambda: np.array([0, 0, 0, 0, 0, -1], dtype=np.float32))

    def reset(self, current_grip_rad: float = GRIP_OPEN_RAD) -> None:
        if not np.isfinite(current_grip_rad):
            raise ValueError("current gripper angle must be finite")
        grip_span = GRIP_CLOSED_RAD - GRIP_OPEN_RAD
        grip_action = 2.0 * (float(current_grip_rad) - GRIP_OPEN_RAD) / grip_span - 1.0
        self.filtered_action = np.array(
            [0, 0, 0, 0, 0, np.clip(grip_action, -1.0, 1.0)],
            dtype=np.float32,
        )

    def prepare(
        self,
        raw_action: Sequence[float],
        current_arm_rads: Sequence[float],
        current_grip_rad: float,
        joint_limits: Sequence[Tuple[float, float]],
        *,
        target_distance_m: float,
        floor_clearance_m: float,
        pads_ready: bool,
        contact_detected: bool = False,
    ) -> dict:
        raw = np.asarray(raw_action, dtype=np.float32)
        if raw.shape != (6,):
            raise ValueError(f"expected action shape (6,), got {raw.shape}")
        if not np.all(np.isfinite(raw)):
            raise ValueError("raw action must contain only finite values")
        raw = np.clip(raw, -1.0, 1.0)

        current = np.asarray(current_arm_rads, dtype=np.float64)
        if current.shape != (5,) or len(joint_limits) != 5:
            raise ValueError("current arm pose and joint limits must both be 5D")
        if not np.all(np.isfinite(current)) or not np.isfinite(current_grip_rad):
            raise ValueError("current arm and gripper state must be finite")
        if self.filtered_action.shape != (6,) or not np.all(
                np.isfinite(self.filtered_action)):
            raise ValueError("filtered action state must be a finite 6D vector")

        normalized_limits = []
        for low, high in joint_limits:
            low, high = float(low), float(high)
            if not np.isfinite(low) or not np.isfinite(high) or low > high:
                raise ValueError(f"invalid joint limit ({low}, {high})")
            normalized_limits.append((low, high))

        distance = float(target_distance_m)
        clearance = float(floor_clearance_m)
        if not np.isfinite(distance) or distance < 0.0:
            raise ValueError("target distance must be finite and non-negative")
        if not np.isfinite(clearance):
            raise ValueError("floor clearance must be finite")

        alpha = SMOOTH_ALPHA_FAR
        if distance < NEAR_TARGET_M:
            alpha = SMOOTH_ALPHA_NEAR
        if distance < VERY_NEAR_TARGET_M:
            alpha = SMOOTH_ALPHA_VERY_NEAR
        filtered = (
            alpha * self.filtered_action + (1.0 - alpha) * raw
        ).astype(np.float32)

        distance_step_scale = 1.0
        if distance < NEAR_TARGET_M and not contact_detected:
            distance_step_scale = NEAR_STEP_SCALE
        if distance < VERY_NEAR_TARGET_M and not contact_detected:
            distance_step_scale = VERY_NEAR_STEP_SCALE
        floor_scale = float(np.clip(
            clearance / FLOOR_SLOW_BAND_M, FLOOR_SLOW_MIN, 1.0))
        total_step_scale = distance_step_scale * floor_scale

        arm_target = np.empty(5, dtype=np.float64)
        max_delta = ARM_STEP_RAD * total_step_scale
        for index, (low, high) in enumerate(normalized_limits):
            desired = current[index] + float(filtered[index]) * ARM_STEP_RAD
            desired = float(np.clip(desired, low, high))
            arm_target[index] = current[index] + float(np.clip(
                desired - current[index], -max_delta, max_delta))

        grip_cmd = float(filtered[5])
        if not pads_ready and not contact_detected:
            grip_cmd = min(grip_cmd, 0.0)
        elif pads_ready or contact_detected:
            grip_cmd = max(grip_cmd, MIN_CLOSE_ACTION)
        desired_grip = (
            GRIP_OPEN_RAD
            + (grip_cmd + 1.0) * 0.5 * (GRIP_CLOSED_RAD - GRIP_OPEN_RAD)
        )
        grip_target = float(current_grip_rad) + float(np.clip(
            desired_grip - float(current_grip_rad), -GRIP_STEP_RAD, GRIP_STEP_RAD))
        grip_target = float(np.clip(grip_target, GRIP_OPEN_RAD, GRIP_CLOSED_RAD))

        # Training records the raw policy action in obs[22:28], while this
        # state keeps the filtered action for the next preprocessing step.
        self.filtered_action = filtered.copy()
        return {
            "raw_action_for_observation": raw.copy(),
            "filtered_action": filtered.copy(),
            "smooth_alpha": alpha,
            "distance_step_scale": distance_step_scale,
            "floor_step_scale": floor_scale,
            "total_step_scale": total_step_scale,
            "arm_target_rads_pre_floor_guard": arm_target,
            "grip_target_rad_pre_floor_guard": grip_target,
            "pads_ready": bool(pads_ready),
            "contact_detected": bool(contact_detected),
        }
