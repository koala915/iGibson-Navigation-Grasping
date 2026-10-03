"""Versioned observation / action contracts shared by eval and real-robot deployment.

A *contract* pins together the four things that must agree or the policy is silently
fed garbage:

  1. the observation layout (dimension + what each slice means),
  2. the arm-action semantics (absolute joint target vs. incremental delta),
  3. the TCP definition used for ``tcp_pos`` / ``rel_pos``,
  4. whether a per-episode object height is required.

Two generations exist:

  ``obs_28_absolute``          v18, v19 — 28D obs, arm action = absolute joint target
                               normalised over the full joint range.
  ``obs_28_incremental``       v21      — original deployable 28D observation with
                               higher-resolution incremental arm actions.
  ``obs_32_goal_incremental``  v20      — 32D obs (28D + target_rel + object_height),
                               arm action = delta from the *current* angle.

They are NOT interchangeable. Running a v19 model under the v20 contract produces
plausible-looking motion that is wrong in both magnitude and reference frame, so every
entry point validates and **fails closed** rather than warning and continuing.

This module deliberately has no iGibson / training-env dependency so it can be deployed
to the robot. The geometry constants below are duplicated from
``x3plus_ground_grasp_env.TaskConfig``; ``test_deploy_parity.py`` asserts they still
match, which is what keeps the duplication honest.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


class ContractMismatchError(RuntimeError):
    """A model, VecNormalize, env or config disagree about the contract.

    Always fatal: a mismatch means the policy's inputs or outputs are being
    misinterpreted, which on real hardware means uncontrolled motion.
    """


class MissingObjectHeightError(RuntimeError):
    """The active contract needs a per-object height and detection did not supply one.

    Never substitute a default. See ``DETECTION_CONTRACT_REQUIRED`` below.
    """


# ───────────────────────────────────────────────────────────────────────────
# Geometry constants (mirror of TaskConfig — verified by test_deploy_parity.py)
# ───────────────────────────────────────────────────────────────────────────

GROUND_HEIGHT = 0.0          # m, floor plane in robot base frame
GRASP_PAD_OFFSET = 0.033     # m, finger pads sit this far BELOW gripper_center
GRASP_CLOSE_DROP = 0.0267    # m, closing the jaw drops the pads/gc by this much
GRASP_HOVER_EXTRA = 0.0      # m, additional clearance (0 since v19)
GRASP_DEPTH_RATIO = 0.45     # fraction of object height the pads aim for
SWEET_SPOT_OFFSET = (0.0, 0.0, 0.0)
PREGRASP_TOWARD_ROBOT_XY = 0.0
MIN_WRIST_Z_OFFSET = 0.015   # m, floor of the adaptive hover offset
STAGE1_MIN_ENGAGE_DEPTH = 0.002   # m, how far inside the object the pads must land

GRIPPER_ANGLE_OPEN = -1.5    # rad
GRIPPER_ANGLE_CLOSED = 0.0   # rad

ARM_STEP_RAD = 0.08          # rad per 1.0 action unit, all five arm joints

# Joints driven together by the parallel linkage, and their sign multipliers.
# The gripper is a mimic chain: every joint is grip_joint * multiplier.
GRIPPER_JOINT_MULTIPLIERS: Dict[str, float] = {
    "grip_joint": 1.0,
    "rlink_joint2": -1.0,
    "rlink_joint3": 1.0,
    "llink_joint1": -1.0,
    "llink_joint2": 1.0,
    "llink_joint3": -1.0,
}

# iGibson does not load the URDF the way a bare p.loadURDF() does: it merges fixed
# links (20 joints in the raw file become 15) and settles the robot on its wheels, so
# its root frame does not coincide with base_footprint. The policy consumes ABSOLUTE
# tcp_pos and object_pos, so deployment has to reproduce the training frame or every
# position it reports is offset by ~2.2 cm — comparable to a bottle cap's whole height.
#
# Measured as the best-fit rigid translation (training gripper_center minus raw-URDF
# gripper_center) over 120 random arm+gripper poses:
#   |delta| before = 22.1 mm mean; residual after = 0.68 mm mean, 1.60 mm max.
# The ~1 mm residual is the training base's sub-degree settling tilt, which is not
# corrected. test_deploy_parity.py test 3 re-measures this and fails above 2 mm.
URDF_TO_TRAINING_FRAME = (0.019896, -0.003359, -0.009072)   # metres

# The base pose the offset above was calibrated at: the training robot settled on its
# wheels, measured over 8 resets (std < 0.5 mm). The base is NOT fixed — arm motion
# pushes it — so absolute positions drift with it. Deployment must therefore place its
# FK base at URDF_TO_TRAINING_FRAME plus however far the real base has drifted from
# this nominal pose; adding the raw measured pose instead would double-count the
# structural offset. Use base_position_for_fk().
TRAINING_BASE_NOMINAL = (-0.000464, -0.000031, 0.046395)


def base_position_for_fk(measured_base_position=None):
    """Where to place the deployment FK base, given the robot's measured base pose.

    Pass None (or the nominal pose) when the base is known to be stationary.
    """
    if measured_base_position is None:
        return tuple(URDF_TO_TRAINING_FRAME)
    drift = (np.asarray(measured_base_position, dtype=np.float64).reshape(-1)[:3]
             - np.asarray(TRAINING_BASE_NOMINAL, dtype=np.float64))
    return tuple(np.asarray(URDF_TO_TRAINING_FRAME, dtype=np.float64) + drift)

# gripper_center = midpoint of these two links' COMs (pybullet getLinkState()[0]).
TCP_RIGHT_JOINT = "rlink_joint2"
TCP_LEFT_JOINT = "llink_joint2"
TCP_DEFINITION = (
    "midpoint of the world COM (pybullet getLinkState(...)[0]) of the child links of "
    "rlink_joint2 and llink_joint2 — i.e. between the two finger pads, NOT arm_link5"
)

DETECTION_CONTRACT_REQUIRED = """\
The 32D contract needs the object's HEIGHT (full extent along z, metres) per detection.
The current detection message is {"x":..., "y":..., "z":...} where z is a position, not
a height, so the height cannot be derived without assuming object symmetry and that the
object rests on the floor. Add an explicit field, e.g.

    {"x":0.24, "y":0.01, "z":0.0065, "height":0.013}

with "height" = measured top-of-object minus floor, and "z" = object centroid height.
Do not infer height from z."""


# ───────────────────────────────────────────────────────────────────────────
# Contract definitions
# ───────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class ObsSlice:
    name: str
    start: int
    stop: int
    unit: str
    note: str = ""

    @property
    def width(self) -> int:
        return self.stop - self.start


@dataclass(frozen=True)
class DeployContract:
    name: str
    obs_dim: int
    action_dim: int
    arm_action_mode: str       # "absolute" | "incremental"
    arm_step_rad: float        # rad per action unit; only used when incremental
    gripper_action_mode: str   # "absolute" in every contract so far
    tcp_definition: str
    requires_object_height: bool
    slices: Tuple[ObsSlice, ...]
    generations: Tuple[str, ...]

    def __post_init__(self) -> None:
        covered = sum(s.width for s in self.slices)
        if covered != self.obs_dim:
            raise ContractMismatchError(
                f"contract {self.name!r}: slices cover {covered}D but obs_dim={self.obs_dim}"
            )
        cursor = 0
        for s in self.slices:
            if s.start != cursor:
                raise ContractMismatchError(
                    f"contract {self.name!r}: slice {s.name!r} starts at {s.start}, expected {cursor}"
                )
            cursor = s.stop
        if self.arm_action_mode not in ("absolute", "incremental"):
            raise ContractMismatchError(
                f"contract {self.name!r}: unknown arm_action_mode {self.arm_action_mode!r}"
            )

    def slice_of(self, name: str) -> ObsSlice:
        for s in self.slices:
            if s.name == name:
                return s
        raise KeyError(f"contract {self.name!r} has no slice {name!r}")


_BASE_28_SLICES: Tuple[ObsSlice, ...] = (
    ObsSlice("arm_joints", 0, 5, "rad", "arm_joint1..5"),
    ObsSlice("grip_joint", 5, 6, "rad", ""),
    ObsSlice("tcp_pos", 6, 9, "m", "gripper_center"),
    ObsSlice("tcp_orn", 9, 13, "quat xyzw", "arm_link5 orientation"),
    ObsSlice("object_pos", 13, 16, "m", "object centroid"),
    ObsSlice("rel_pos", 16, 19, "m", "object_pos - tcp_pos"),
    ObsSlice("stage_onehot", 19, 22, "-", "[stage0, stage1, stage2]"),
    ObsSlice("prev_action", 22, 28, "-", "previous 6D action"),
)

OBS_28_ABSOLUTE = DeployContract(
    name="obs_28_absolute",
    obs_dim=28,
    action_dim=6,
    arm_action_mode="absolute",
    arm_step_rad=ARM_STEP_RAD,
    gripper_action_mode="absolute",
    tcp_definition=TCP_DEFINITION,
    requires_object_height=False,
    slices=_BASE_28_SLICES,
    generations=("v18", "v19"),
)

OBS_28_INCREMENTAL = DeployContract(
    name="obs_28_incremental",
    obs_dim=28,
    action_dim=6,
    arm_action_mode="incremental",
    arm_step_rad=ARM_STEP_RAD,
    gripper_action_mode="absolute",
    tcp_definition=TCP_DEFINITION,
    requires_object_height=False,
    slices=_BASE_28_SLICES,
    generations=("v21",),
)

OBS_32_GOAL_INCREMENTAL = DeployContract(
    name="obs_32_goal_incremental",
    obs_dim=32,
    action_dim=6,
    arm_action_mode="incremental",
    arm_step_rad=ARM_STEP_RAD,
    gripper_action_mode="absolute",
    tcp_definition=TCP_DEFINITION,
    requires_object_height=True,
    slices=_BASE_28_SLICES + (
        ObsSlice("target_rel", 28, 31, "m", "stage0_target - tcp_pos"),
        ObsSlice("object_height", 31, 32, "m", "full object height this episode"),
    ),
    generations=("v20",),
)

CONTRACTS: Dict[str, DeployContract] = {
    c.name: c for c in (
        OBS_28_ABSOLUTE, OBS_28_INCREMENTAL, OBS_32_GOAL_INCREMENTAL
    )
}


def get_contract(name: str) -> DeployContract:
    try:
        return CONTRACTS[name]
    except KeyError:
        raise ContractMismatchError(
            f"unknown contract {name!r}; known: {sorted(CONTRACTS)}"
        ) from None


# ───────────────────────────────────────────────────────────────────────────
# Fail-closed validation
# ───────────────────────────────────────────────────────────────────────────

def validate_model(
    contract: DeployContract,
    model,
    *,
    vecnorm_obs_rms=None,
    model_path: str = "<model>",
    vecnorm_path: str = "<vecnormalize>",
) -> None:
    """Check a loaded policy (and its VecNormalize) against ``contract``.

    Raises ContractMismatchError on any disagreement. There is deliberately no
    "warn and continue" path: a 28D policy driven by 32D statistics, or an absolute
    policy decoded as incremental, moves the real arm in ways nobody predicted.

    Note the dimension check cannot distinguish absolute from incremental within the
    same obs_dim — that is why the contract must be named explicitly by the caller and
    recorded in the model manifest, never guessed from the file.
    """
    obs_space = getattr(model, "observation_space", None)
    if obs_space is None or not hasattr(obs_space, "shape"):
        raise ContractMismatchError(f"{model_path}: policy exposes no observation_space")
    got_obs = int(np.prod(obs_space.shape))
    if got_obs != contract.obs_dim:
        raise ContractMismatchError(
            f"{model_path}: policy observation is {got_obs}D but contract "
            f"{contract.name!r} requires {contract.obs_dim}D. This model belongs to a "
            f"different contract generation — check which of {sorted(CONTRACTS)} it was "
            f"trained under and load the matching VecNormalize with it."
        )

    act_space = getattr(model, "action_space", None)
    if act_space is None or not hasattr(act_space, "shape"):
        raise ContractMismatchError(f"{model_path}: policy exposes no action_space")
    got_act = int(np.prod(act_space.shape))
    if got_act != contract.action_dim:
        raise ContractMismatchError(
            f"{model_path}: policy action is {got_act}D but contract "
            f"{contract.name!r} requires {contract.action_dim}D"
        )

    if vecnorm_obs_rms is not None:
        mean = getattr(vecnorm_obs_rms, "mean", None)
        if mean is None:
            raise ContractMismatchError(f"{vecnorm_path}: obs_rms has no .mean")
        got_norm = int(np.asarray(mean).reshape(-1).shape[0])
        if got_norm != contract.obs_dim:
            raise ContractMismatchError(
                f"{vecnorm_path}: VecNormalize statistics are {got_norm}D but contract "
                f"{contract.name!r} requires {contract.obs_dim}D. The model and its "
                f"VecNormalize must come from the same training run."
            )


def validate_env(contract: DeployContract, env, *, label: str = "<env>") -> None:
    """Check a gym env's spaces against ``contract``. Raises on mismatch."""
    got_obs = int(np.prod(env.observation_space.shape))
    if got_obs != contract.obs_dim:
        raise ContractMismatchError(
            f"{label}: env observation is {got_obs}D but contract "
            f"{contract.name!r} requires {contract.obs_dim}D"
        )
    got_act = int(np.prod(env.action_space.shape))
    if got_act != contract.action_dim:
        raise ContractMismatchError(
            f"{label}: env action is {got_act}D but contract "
            f"{contract.name!r} requires {contract.action_dim}D"
        )


def contract_for_env_config(cfg) -> DeployContract:
    """Derive the contract implied by a training TaskConfig.

    Used by tests and by the manifest writer so the recorded contract name is read off
    the config that actually ran, not typed in by hand.
    """
    incremental = bool(getattr(cfg, "arm_action_incremental", False))
    goal = bool(getattr(cfg, "include_goal_in_obs", False))
    if incremental and goal:
        return OBS_32_GOAL_INCREMENTAL
    if incremental and not goal:
        return OBS_28_INCREMENTAL
    if not incremental and not goal:
        return OBS_28_ABSOLUTE
    raise ContractMismatchError(
        "TaskConfig requests absolute arm actions with goal-only observations; "
        "no released contract covers that combination"
    )


# ───────────────────────────────────────────────────────────────────────────
# Shared decoders (single implementation for eval + deployment)
# ───────────────────────────────────────────────────────────────────────────

def decode_arm_action(
    contract: DeployContract,
    action: Sequence[float],
    current_arm_rads: Sequence[float],
    joint_limits: Sequence[Tuple[float, float]],
) -> np.ndarray:
    """Decode the first five action dims into desired arm joint angles (rad).

    Mirrors ``robot_grasp_env._apply_action``: the incremental branch steps from the
    *current* angle by ``action * arm_step_rad``; the absolute branch maps the action
    across the full joint range. Clipping to the joint limits happens here; any
    additional per-step rate limit is a deployment safety layer applied afterwards and
    must be looser than arm_step_rad or it silently changes policy semantics.
    """
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:5], -1.0, 1.0)
    cur = np.asarray(current_arm_rads, dtype=np.float64).reshape(-1)[:5]
    out = np.zeros(5, dtype=np.float64)
    for i in range(5):
        lo, hi = float(joint_limits[i][0]), float(joint_limits[i][1])
        if contract.arm_action_mode == "incremental":
            desired = cur[i] + a[i] * contract.arm_step_rad
        else:
            desired = lo + (a[i] + 1.0) * 0.5 * (hi - lo)
        out[i] = float(np.clip(desired, lo, hi))
    return out


def decode_gripper_action(action: Sequence[float]) -> float:
    """Decode action[5] into a gripper joint angle (rad). Absolute in every contract."""
    a = float(np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[5], -1.0, 1.0))
    return float(GRIPPER_ANGLE_OPEN + (a + 1.0) * 0.5 * (GRIPPER_ANGLE_CLOSED - GRIPPER_ANGLE_OPEN))


def hover_gripper_center_z(object_height: float) -> float:
    """Absolute gripper_center height the policy hovers at, for a given object height.

    Anchored to the FLOOR, not to the object centroid: the graspable band is
    [floor, object top], so a shorter object means a *lower* target. Mirrors
    ``_apply_adaptive_hover_offset``.
    """
    return float(
        GROUND_HEIGHT
        + GRASP_PAD_OFFSET
        + GRASP_CLOSE_DROP
        + GRASP_HOVER_EXTRA
        + GRASP_DEPTH_RATIO * float(object_height)
    )


def episode_wrist_z_offset(object_height: Optional[float], object_rest_z: float) -> float:
    """The per-episode vertical offset, fixed once from the object's RESTING height.

    The training env computes this in ``_apply_adaptive_hover_offset()`` at reset and
    then holds it for the whole episode; ``_get_stage0_target()`` adds it to the
    object's *current* position. Recomputing it each step instead looks equivalent
    while the object sits still and silently diverges the moment it is lifted — a
    ~4 cm target error, which is how this was caught.

    ``object_rest_z`` is the object's centroid height while it rests on the floor
    (i.e. at detection time), not its live position.
    """
    if object_height is None:
        raise MissingObjectHeightError(DETECTION_CONTRACT_REQUIRED)
    hover_z = hover_gripper_center_z(object_height)
    return float(max(MIN_WRIST_Z_OFFSET, hover_z - float(object_rest_z)))


def stage0_target(
    object_pos: Sequence[float],
    wrist_z_offset: float,
    close_drop: float,
) -> np.ndarray:
    """Reproduce ``_get_stage0_target()`` for deployment.

    ``wrist_z_offset`` must come from ``episode_wrist_z_offset()`` computed once per
    episode — see the note there.

    ``close_drop`` is how far gripper_center has already sunk because the jaw is
    partly closed; it must be *measured* by FK at the current grip angle, not assumed,
    because the hover height is defined in fully-open coordinates.
    """
    obj = np.asarray(object_pos, dtype=np.float64).reshape(-1)[:3]
    target = obj - np.asarray(SWEET_SPOT_OFFSET, dtype=np.float64)
    target[2] += float(wrist_z_offset)
    target[2] -= float(close_drop)
    return target.astype(np.float64)


def build_observation(
    contract: DeployContract,
    *,
    arm_rads: Sequence[float],
    grip_rad: float,
    tcp_pos: Sequence[float],
    tcp_quat: Sequence[float],
    object_pos: Sequence[float],
    stage: int,
    prev_action: Sequence[float],
    object_height: Optional[float] = None,
    close_drop: float = 0.0,
    wrist_z_offset: Optional[float] = None,
) -> np.ndarray:
    """Assemble the raw (un-normalised) observation for ``contract``.

    ``tcp_pos`` must be gripper_center, matching ``TCP_DEFINITION``.
    ``wrist_z_offset`` must be the per-episode value from
    ``episode_wrist_z_offset()``; it is required whenever the contract carries
    ``target_rel``.
    """
    arm = np.asarray(arm_rads, dtype=np.float32).reshape(-1)[:5]
    tcp = np.asarray(tcp_pos, dtype=np.float32).reshape(-1)[:3]
    obj = np.asarray(object_pos, dtype=np.float32).reshape(-1)[:3]

    stage_onehot = np.zeros(3, dtype=np.float32)
    stage_onehot[int(np.clip(stage, 0, 2))] = 1.0

    parts = [
        arm,
        np.array([grip_rad], dtype=np.float32),
        tcp,
        np.asarray(tcp_quat, dtype=np.float32).reshape(-1)[:4],
        obj,
        (obj - tcp).astype(np.float32),
        stage_onehot,
        np.asarray(prev_action, dtype=np.float32).reshape(-1)[:6],
    ]

    if contract.requires_object_height:
        if object_height is None:
            raise MissingObjectHeightError(DETECTION_CONTRACT_REQUIRED)
        if wrist_z_offset is None:
            raise ContractMismatchError(
                "wrist_z_offset is required for this contract; compute it once per "
                "episode with episode_wrist_z_offset(object_height, object_rest_z). "
                "Deriving it per step from the object's live position diverges as soon "
                "as the object is lifted."
            )
        target = stage0_target(obj, wrist_z_offset, close_drop)
        parts.append((target.astype(np.float32) - tcp).astype(np.float32))
        parts.append(np.array([object_height], dtype=np.float32))

    obs = np.concatenate(parts).astype(np.float32)
    if obs.shape != (contract.obs_dim,):
        raise ContractMismatchError(
            f"built {obs.shape[0]}D observation but contract {contract.name!r} "
            f"requires {contract.obs_dim}D"
        )
    return obs
