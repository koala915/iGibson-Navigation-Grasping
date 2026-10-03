#!/usr/bin/env python3
"""Standalone Jetson Nano deployment for X3Plus 6D grasp policy.

No ROS required. Uses PyBullet DIRECT (headless) for forward kinematics only.

Features
--------
- Loads trained weights from the same folder automatically
- TCP socket (port 5555) for object detection input — or defaults to 0.26 m front
- Yahboom Rosmaster_Lib servo control via UART
- 28D observation matching student training env exactly
- Automatic 3-stage grasp management

Usage
-----
    python x3plus_real_grasp.py                     # dry-run, default obj at 0.26 m
    python x3plus_real_grasp.py --real              # real servo mode
    python x3plus_real_grasp.py --real --socket     # real servo + listen for detection

Sim-to-real calibration note (HARDWARE-CALIBRATED 2026-05-25; v18 update 2026-07-17)
-----------------------------
  Rosmaster API command = 90 + sim_deg for EVERY arm joint (arm_hw_invert all
  False). The Rosmaster/App mirrors S2/S3/S4 internally, so the teleop App
  DISPLAYS physical = 180 - API for those joints — do not feed App angles here.
  S6 gripper: API 30° = OPEN, 180° = CLOSED (verified with real grasps).

  ⚠ An earlier revision of this file used S2-inverted mapping and open=180 —
  that convention was a double-mirror artifact and moved the policy backwards.

  All mappings are defined in JointMapper and can be adjusted without touching
  the control logic.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, NamedTuple, Optional, Tuple

import numpy as np

if "KMP_DUPLICATE_LIB_OK" not in os.environ:
    os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
os.environ.setdefault("OMP_NUM_THREADS", "1")

import pybullet as p
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import gymnasium as gym

import deploy_contract as dc
from action_execution_v21 import ActionExecutionState
from deploy_contract import ContractMismatchError, MissingObjectHeightError

# ── Yahboom Rosmaster_Lib ───────────────────────────────────────────────────
# X3Plus drives its arm through the Rosmaster expansion board, NOT through the
# DOFBOT-era Arm_Lib this file originally imported. Arm_Lib does not exist on this
# robot, so that import always failed and the controller silently degraded to
# "print only" even under --real. See V21_DEPLOY_REVIEW_AND_PLAN_2026-07-30.md (F1).
#
# On the Jetson the driver is localized beside the deployment scripts, one level up
# at grasp/Rosmaster_Lib (see CLAUDE.md: cp -r /usr/local/lib/python3.6/dist-packages/
# Rosmaster_Lib .). This stack runs from grasp/v21/, so grasp/ has to be importable.
_GRASP_DIR = str(Path(__file__).resolve().parent.parent)
if _GRASP_DIR not in sys.path:
    sys.path.append(_GRASP_DIR)

try:
    from Rosmaster_Lib import Rosmaster as _RosmasterCls
    _ROSMASTER_AVAILABLE = True
except ImportError:
    _RosmasterCls = None
    _ROSMASTER_AVAILABLE = False
    print("[WARNING] Rosmaster_Lib not found. Servo output will be printed only "
          "(dry-run). --real will refuse to start.")


# ═══════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class DeployConfig:
    # ── model paths (relative to this script or absolute) ──
    # v18 (2026-07-17) = "C3 high-FOV grasp-home" model. Episode start = C3 pose
    #   (sim (0,-0.4,-1.4,-1.4,0) = API (90,67.1,9.8,9.8,90)+S6=30): camera at
    #   0.227m sees ground abs x 0.196-0.330m (84% of the graspable band; v17's
    #   low home saw only 0.185-0.274 — objects ~10cm ahead were OUT of frame).
    #   Formal acceptance: 97/100 deterministic, held-out seed, deployable region
    #   x(0.20,0.33) y(±0.10); nominal (0.25,0) 10/10; cracker_box 20/20.
    #   Verify: python training/eval_6d_grasp.py \
    #     --model trained_6d_models_v18/ppo_6d_final_ready_for_real_robot.zip \
    #     --vecnorm trained_6d_models_v18/vecnormalize_6d_final.pkl \
    #     --episodes 30 --randomize --x-lo 0.20 --x-hi 0.33 --y-lo -0.10 --y-hi 0.10
    # ⚠ model 與 VecNormalize 必須成對使用；v18 訓練起點=C3，絕不可拿 v17 權重
    #   從 C3 起跳（OOD），也不可拿 v18 從舊低姿態起跳。
    # ⚠ Stage-2 (return) arm actions are SCRIPTED in sim — on the real robot,
    #   script the return once a stable grasp is detected.
    # Rollback: v17 pair at trained_6d_models_v17/ (its home = API 90,32.7,9.8,32.7,90).
    # No v18 fallback: 28D absolute and 28D incremental models have identical
    # tensor dimensions, so loading the old pair under the v21 contract cannot be
    # detected from the zip. A selected v21 pair must be supplied explicitly.
    model_path: str = "models/candidate_v23_seed23401_ckpt250000.zip"
    vecnorm_path: str = "models/candidate_v23_seed23401_ckpt250000_vec.pkl"
    # Repo integration: this stack lives at grasp/v23/ and reuses the URDF + meshes
    # already tracked at grasp/x3plus/. Verified byte-identical to the ones shipped
    # in the v21 handoff package (git blob sha256
    # 0a43968783b178d2f270572dc2e01c684722864af1bf8a0c52f05a66fe9d2be2; the
    # working-file hash differs on Windows only because core.autocrlf rewrites the
    # line endings). Every mesh the URDF references is present and identical there,
    # so the 151 MB copy from the package is deliberately not duplicated.
    # There is no --urdf flag, so this default is the only place to set it.
    urdf_path: str = "../x3plus/yahboomcar.urdf"

    # ── observation / action contract ──
    # MUST match the generation the weights were trained under. It is not inferable
    # from the .zip: 28D-vs-32D is checked, but absolute-vs-incremental arm semantics
    # are invisible in the file, and decoding one as the other drives the arm to the
    # wrong place at full range. Look it up in the model's manifest.json.
    #   v18, v19 → "obs_28_absolute"
    #   v20       → "obs_32_goal_incremental"  (also requires object height per detection)
    contract_name: str = "obs_28_incremental"

    # Serial port for the Rosmaster board. /dev/myserial is the stable udev symlink
    # to the ch341 device; the raw /dev/ttyUSBn index is not stable across reboots.
    serial_port: str = "/dev/myserial"

    # Floor plane height in the robot base frame; used for the hover target.
    ground_height: float = 0.0

    # ── half-duplex servo bus ──
    # All six servos share one UART line, so a read issued while a write is still on
    # the wire gets no answer and the driver returns -1 after its 30 ms timeout. The
    # v17 stack hid this by substituting the last command whenever a read failed;
    # this stack fails closed instead, which is why the condition became visible.
    bus_quiet_s: float = 0.02    # minimum gap between a write and the next read
    read_attempts: int = 3       # whole-read retries before declaring the pose unknown
    read_retry_s: float = 0.05   # pause between attempts, to let the bus drain

    # ── floor safety (real robot) ──
    # The sim-side guard can restore a violating state; hardware cannot be teleported
    # back, so on the robot the ONLY workable form is prevention: sweep the candidate
    # path with FK before anything is sent, and shorten or refuse the motion. There is
    # no backstop here by design — if this layer fails, the gripper hits the floor.
    floor_guard_enable: bool = True
    # Raises the two finger pads by this much wherever this code reasons about
    # where the pads ARE, undoing the measured URDF finger error (16.7 mm too long
    # at C3, 2026-07-31; 11.9 mm open / 15.4 mm closed at E1, 2026-08-29).
    #
    # TWO consumers, and they fail in opposite directions, which is why the error
    # has to reach both:
    #
    #   FloorGuard          believes the pads are LOWER than they are, so it stops
    #                       early. Conservative. This is what deadlocked run 6.
    #   _grasp_geometry     believes the same thing, and there it means pads_ready
    #                       fires while the real pads are still ABOVE the object.
    #                       Not conservative at all.
    #
    # The second one was found on hardware on 2026-08-30. With a 6.5 cm box the
    # gate went true at z-error 23 mm: modelled pads at 53 mm against a 65 mm
    # object top, real pads at about 68 mm -- above the object. The jaw is forced
    # to at least MIN_CLOSE_ACTION once pads_ready, so it shut 25 steps before the
    # arm was in position, and the closed jaw then jammed the arm short of the
    # target. The run looked like "it grasps too close to the robot"; it had not
    # grasped at all.
    #
    # Default stays 0.0, the training-identical metric. Opt in with
    # --floor-finger-error-mm AFTER measuring the real clearance with a ruler
    # (pose_check.py --real prints both jaw angles); getting it wrong in the other
    # direction drives the fingers into the floor.
    floor_finger_error_m: float = 0.0
    # Hardware run 2026-08-30: the measured E1 homography put the object within
    # 0.9 mm of its ruler position, yet the real gripper still landed toward the
    # robot. This is therefore kept out of object/camera geometry. A positive
    # value says the URDF/FK TCP is this far too far forward: subtract it from
    # the TCP X seen by both the policy and the close gate, so the arm continues
    # forward by the requested amount. Default 0 preserves the training model;
    # the supervised v23 launcher opts into the measured experiment.
    tcp_forward_error_m: float = 0.0
    floor_safety_margin: float = 0.008   # m, deliberately looser than sim's 0.005:
                                         # real servos overshoot more than the model
    floor_sweep_samples: int = 8         # points checked along current → target
    floor_bisect_iters: int = 6          # refinement of the largest safe fraction
    emergency_raise_rad: float = 0.05    # per-attempt lift when already below the line
    emergency_raise_max_iters: int = 12  # bound on repeated raise attempts

    # ── scripted stage 2 (no policy, no tcp_pos) ──
    scripted_lift_steps: int = 6
    scripted_lift_rad_per_step: float = 0.05

    # ── scripted stage 3: release over the bin (ported 2026-08-04) ──
    # Backported from the release branch (the copy under training/), which forked
    # before the 2026-07-31 jaw-contact work and therefore must not be run for the
    # grasp itself. Only the release motion is taken.
    #
    # No aiming and no bin pose: the opening is ~30 cm across, so horizontal
    # placement has ~10 cm of slack in every direction and the object just falls.
    # The one thing that can really fail is opening with the pad BELOW the rim,
    # which drops the object down the outside of the bin -- so the reach stops as
    # soon as FK says the pad would no longer clear it.
    release_enable: bool = False
    release_extend_steps: int = 6
    release_extend_rad_per_step: float = 0.05
    bin_rim_height: float = 0.05      # m, bin rim above the floor -- MEASURE YOURS
    release_clearance: float = 0.03   # m, pad bottom must clear the rim by this much
    release_settle_s: float = 0.5     # pause after opening, before retracting
    # The jaw travels open→closed = 150 hw deg, and send_degrees moves at most
    # jaw_max_delta_deg per command, so a full close needs ~19 iterations at the
    # shipped 8 deg (~13 if --jaw-step-deg raises it to 12). Left at 40 so the bound
    # still holds either way.
    close_max_iters: int = 40
    # Grasp proxy: this robot has no force/tactile sensor. A jaw closing on nothing
    # reaches the fully-closed stop; one closing on an object stalls short of it.
    # Require it to stall by at least this fraction of the open→closed span before
    # believing an object is held. Proxy only — see _grasp_looks_real().
    #
    # 0.06 (9 deg) -> 0.03 (4.5 deg), 2026-07-31: run 6 rejected a real stall at
    # 175 deg (3.3%, 5 deg short) because the contact-detection race (fixed the
    # same day, commit 7617346) parked the hold ON the 180 stop instead of at the
    # actual contact angle, so the readback undersold how short it really stalled.
    # With that race fixed, 175-ish deg from a genuine contact should register
    # correctly even at the old 0.06 -- 0.03 is a smaller, deliberately partial
    # loosening pending hardware confirmation, not a full retreat to the value
    # that produced the false rejection.
    #
    # VALIDATED ON HARDWARE, 2026-07-31: operator confirmed an empty jaw does not
    # misfire as CONFIRMED at this margin. A genuine grasp the same day cleared it
    # with a wide margin in the other direction too -- jaw stalled at 151/153 deg,
    # 18-19% short of closed, nowhere near the 3% line. See manifest.json
    # transport.grasp_stall_margin_tuning.empty_jaw_validation_2026_07_31 (no
    # specific trial count/settled-angle log was captured with the report, so
    # stronger repeat-trial evidence is still worth attaching if it turns up).
    grasp_stall_min_fraction: float = 0.03

    # ── jaw contact handling (stage 1 close) ──
    # This servo produces grip force by holding a commanded position PAST the
    # physical contact angle: torque scales with the position error. Both extremes
    # of that error are wrong. Walking the command all the way to the closed stop
    # against an object leaves the gearbox at near-stall torque for the rest of the
    # run — the grinding heard on hardware 2026-07-31, and a stripped gear waiting
    # to happen. Re-commanding the exact readback drops the error to zero, and with
    # it the grip force, so the object slips out during the lift. Contact is
    # therefore detected as "the command advances but the encoder does not", and the
    # command is then parked jaw_hold_bias_deg past the measured contact angle:
    # enough error for a firm hold, nowhere near the stop-to-stop error that grinds.
    jaw_contact_min_progress_deg: float = 2.0  # encoder moved less than this = not tracking
    jaw_contact_lag_deg: float = 5.0     # command ahead of encoder by more = blocked
    jaw_contact_streak: int = 2          # consecutive blocked iterations before declaring
    jaw_hold_bias_deg: float = 8.0       # grip-force knob: hold this far past contact

    # The absolute progress test above has a blind spot, heard on hardware as a
    # repeated clicking during the close: an object that SLIPS a little under the
    # jaw lets the encoder move more than jaw_contact_min_progress_deg, so
    # `tracking` reads true, the contact streak resets, and the command keeps
    # advancing into an object it has already reached. The jaw is moving, so
    # nothing looks stalled; it is just moving slower than it was told to.
    #
    # Comparing the encoder's step against the COMMANDED step catches that,
    # because it asks the scale-free question -- is the jaw keeping up? A free
    # jaw returns about 1.0, a blocked one about 0.0, and one squeezing a
    # slipping object lands in between.
    #
    # 0.0 disables it, which is the shipped default: the 2026-07-31 grasp that
    # worked used the absolute test alone, and this only ever makes contact fire
    # EARLIER. 0.5 is the value to try for the slipping case.
    jaw_contact_track_fraction: float = 0.0
    # Ratio test is skipped below this commanded step, where the quotient is
    # dominated by encoder quantisation rather than by the object.
    jaw_contact_min_cmd_step_deg: float = 1.0

    # Hard ceiling on how far the jaw command may run past the encoder during a
    # close. This is a bound on FORCE, not a detector: the servo makes grip force
    # out of position error, so capping the error caps the torque by construction
    # and the gearbox can no longer be driven to the stop-to-stop error that
    # grinds -- even if every detector above misses, which is exactly what
    # happened when the object slipped. Sized above jaw_hold_bias_deg so a normal
    # close and the deliberate hold are never throttled; it binds only on the
    # runaway.
    jaw_max_lag_deg: float = 15.0
    # Opt-in hardware shortcut: while Stage 0's policy is already closing the jaw,
    # treat N consecutive blocked readings as contact and lift with only the
    # explicitly configured small hold bias. "Blocked" includes both a stationary
    # encoder and, when jaw_contact_track_fraction is enabled, an encoder that
    # advances too slowly to consume the previous command's outstanding travel.
    # Zero stall steps disables the shortcut.
    # The run loop also requires the jaw to be >50% closed and still clearly short
    # of the empty 180-degree stop, so a stable open or fully-closed jaw cannot pass.
    stage0_s6_stall_steps: int = 0
    stage0_s6_stall_epsilon_deg: float = 0.5
    stage0_s6_stall_min_action: float = 0.5
    stage0_s6_empty_stop_tol_deg: float = 1.0

    # ── object detection ──
    # (dry-run/CLI fallback only; real mode must use fresh detections — fail closed.)
    default_obj_pos: Tuple[float, float, float] = (0.26, 0.0, 0.02)
    # Object height (full z extent, metres). None = detection must supply it.
    # Required by obs_32_goal_incremental; must never be silently defaulted.
    default_obj_height: Optional[float] = None
    socket_host: str = "0.0.0.0"
    socket_port: int = 5555
    # A detection older than this is treated as no detection at all. It also
    # bounds how far the arm can have moved between the frame the sender measured
    # and the moment this side latches, so it must stay SMALL relative to arm
    # motion -- see _verify_detection_pose(). Raise it only when the detector is
    # genuinely slower than this, and expect the pose check to start biting.
    detection_stale_timeout_sec: float = 1.0
    # How far the arm may be from the pose a detection was computed for before the
    # detection is refused. The arm camera rides on arm_link4; a degree of arm
    # motion is a degree of camera motion.
    detection_pose_tol_deg: float = 1.0

    # ── where the policy is allowed to be handed a target ──
    # v21 had two tiers here because it had two different bodies of evidence: a
    # formal held-out evaluation (x 0.20-0.33) and a narrower range the package
    # vouched for (x 0.20-0.28). v23 has neither. It is a 250k checkpoint whose
    # only stated region is the SPAWN BAND it was trained on --
    # x(0.205, 0.280) y(-0.070, +0.065), already inset 2.5 cm for object width so
    # the YOLO box is not clipped at the frame edge (DEPLOY_V23_MANIFEST.md).
    #
    # So both tiers are set to that one band, deliberately. Widening "trained" to
    # v21's numbers would claim an evaluation that was never run at this pose, and
    # inventing a narrower "documented" band would claim a vouching nobody did.
    # When the training side ships a formal v23 evaluation, split them again.
    #
    # ⚠ Known weak corner: grid cell (0.28, -0.070) scored 73.3%, below the 80%
    #   per-cell gate, and the fix ("hotspot added") has NOT been re-run. That is
    #   the far-right-back corner of this band and it is inside the envelope this
    #   guard permits. Treat a failure there as expected, not as a new bug.
    #
    # This envelope matters specifically because of vision. Hand-typed --obj-x
    # stays where you put it, but a detection comes from a camera whose ground
    # footprint does not line up with the band: at E1 the FOV grew 44% (70 -> 101
    # cm^2), so the camera can now see well OUTSIDE the region the policy was
    # trained for. A detector working perfectly can therefore hand the policy a
    # target it has never seen, and nothing downstream would notice: the
    # coordinate is finite, in frame, and geometrically sound.
    trained_x_range: Tuple[float, float] = (0.205, 0.280)
    trained_y_range: Tuple[float, float] = (-0.070, 0.065)
    documented_x_range: Tuple[float, float] = (0.205, 0.280)
    documented_y_range: Tuple[float, float] = (-0.070, 0.065)
    # Slack on the envelope comparison. Targets arrive as float32, so a value
    # written as exactly the boundary is a fraction past it; and a target one
    # micrometre outside is not what this guard is protecting against.
    envelope_tol_m: float = 0.001

    # ── object latching (opt-in via --latch-obj) ──
    # The arm camera is mounted on arm_link4, so it MOVES WITH THE ARM. Its fixed
    # height/pitch distance model is only valid at the navigation/home pose. Without
    # latching, every policy step re-reads a detection taken from a camera that has
    # since swung, and the target position drifts under the policy mid-approach.
    # With latching, the object position + height are captured ONCE at the home pose
    # and frozen for the whole episode.
    # (Modes A and C do the equivalent pipeline-side by freezing obj_provider; this
    # flag is what gives the standalone --socket path the same protection.)
    latch_obj: bool = False
    latch_wait_sec: float = 5.0   # how long to wait for a fresh detection before fallback

    # ── control timing ──
    control_hz: float = 10.0        # inference rate
    servo_run_time_ms: int = 100    # time for servo to reach target (ms)

    # ── scripted-move timing (2026-09-09: cut for a timed demo) ──
    # move_guarded_and_verified sends at most one rate-limited step, waits settle_s,
    # then reads. So a scripted move's wall clock is (travel / step) * settle_s, and
    # these were the dominant cost of a run: the stage-1 close alone was ~15
    # iterations x 0.30 s ~= 6 s of pure waiting.
    #
    # ⚠ THE ONE RULE: settle_s must stay >= run_time_ms, enforced in __post_init__.
    # Read the encoder while the servo is still travelling and it looks like a jaw
    # that is not keeping up with its command -- which is exactly the signature
    # stop_on_jaw_contact treats as CONTACT. Too short a settle does not just slow
    # convergence, it fabricates a grasp and lifts on empty air. Every pair below
    # keeps 20 ms of headroom on top. (The half-duplex bus quiet window is not part
    # of this bound; read_degrees waits that out on its own from _last_write_t.)
    close_run_time_ms: int = 120    # stage-1 jaw close, and the stage-3 open
    close_settle_s: float = 0.14
    lift_run_time_ms: int = 150     # stage-2 lift, 0.05 rad (~2.9 deg) per step
    lift_settle_s: float = 0.17
    home_run_time_ms: int = 200     # startup walk to home, and every return
    home_settle_s: float = 0.22

    # ── stage thresholds ──
    # Mirrors of the training entry condition (_is_centered_for_grasp):
    #   entry_xy = min(xy_alignment_tolerance 0.035, stage0_entry_xy_tolerance 0.022)
    #   entry_z  = min(z_alignment_tolerance 0.030, stage0_entry_z_tolerance 0.018)
    #   radius   = min(stage0_tolerance_radius 0.040, stage0_entry_tolerance_radius 0.026)
    # Deployment must not close on an easier condition than training, or it closes in
    # states the policy never learned to close in.
    entry_xy_tol: float = 0.022
    entry_z_tol: float = 0.018
    entry_radius: float = 0.026

    # ── stage-0 hover fallback ──
    # Run 5, 2026-07-31: the policy closed the jaw on the object mid-stage-0 and then
    # hovered at target_dist 27 mm — ONE millimetre outside the 26 mm entry radius —
    # for 270 straight steps, until max-steps opened the jaw and handed the object
    # back. The primary gate mirrors training and stays untouched; but a sustained
    # hover just outside it is not an approach still in progress, it is an
    # equilibrium, and three seconds of it earns the close. Bounded: xy, z and
    # pads_ready must all be genuinely satisfied — only the radius is relaxed, and
    # only by stage0_hover_extra_m.
    stage0_hover_extra_m: float = 0.005   # radius slack the fallback may forgive
    # 30 -> 15 (3 s -> 1.5 s at 10 Hz), 2026-09-09, for a timed demo. This is a
    # patience setting, not a safety one: xy, z and pads_ready must ALL already be
    # satisfied to be counted, and only the radius is forgiven, by 5 mm. What a
    # shorter wait costs is the chance that an approach still genuinely converging
    # gets read as an equilibrium and closes ~1.5 s early. Run 5's hover sat at a
    # dead-still 27 mm for 270 steps, so it would have been caught at either value.
    stage0_hover_steps: int = 15          # consecutive near-gate steps

    # ── guard deadlock ──
    # Run 6, 2026-07-31: the jaw closed early, close_drop took the modelled pads to
    # 7.4 mm, and the guard's 8 mm line sat between the clearances at two ADJACENT
    # encoder counts on S2. The policy commanded down, the guard raised one count,
    # the policy commanded down again — 42 emergency raises over 150+ steps with
    # z_now bit-identical every time, until the operator interrupted. The per-move
    # emergency_raise_max_iters cannot catch it: a policy step calls the primitive
    # with max_iters=1, so that counter resets every step. This is an episode-level
    # bound, and a deadlock reported honestly beats 300 steps of fake activity.
    guard_deadlock_steps: int = 25        # consecutive intervening steps with no gain
    guard_deadlock_progress_m: float = 0.001   # clearance gain that counts as progress
    # Timeout rescue: a jaw more than this fraction closed at max-steps means the
    # policy was probably holding something. Try the scripted close before opening —
    # an empty jaw walks to the stop and aborts exactly as it always did, but a held
    # object becomes a grasp instead of being silently released.
    timeout_close_min_grip_frac: float = 0.5
    stage2_dist_threshold: float = 0.05   # dist to home for "done"

    # ── safety ──
    max_delta_deg: float = 8.0            # max per-step ARM servo change (deg)
    # S6's own rate limit, separate from the arm's. max_delta_deg is the anti-runaway
    # bound on five joints that swing a 30 cm arm through the world; the jaw is a
    # 150 deg travel between two fingers that cannot reach anything the arm has not
    # already reached, and it pays the rate limit twice per grasp (close, release).
    # Raising it does NOT raise grip force -- force is the command-vs-encoder error,
    # and jaw_max_lag_deg caps that whatever step asks for it.
    #
    # ⚠ Shipped at 8, the SAME as the arm, so the default is a pure no-op. Raising it
    # is opt-in via --jaw-step-deg, and it is not free: 12 roughly halves the close's
    # iteration count but doubles the squeeze past contact on a SLIPPING object
    # (16 deg -> 33 measured), because the force ceiling flattens the commanded step
    # that jaw_contact_track_fraction's ratio test measures against. The one-command
    # launcher enables that slip detector by default (--jaw-track-fraction 0.5, part
    # of the 3/3 hardware run on 2026-08-30), so 12 would trade a protection that is
    # already earning its keep for about 1.6 s. __post_init__ refuses the
    # combination rather than letting it be chosen by accident.
    #
    # The close got its speed from close_run_time_ms/close_settle_s instead, which
    # costs nothing: same commands, less waiting between them.
    jaw_max_delta_deg: float = 8.0
    # v23 policy grasp-home = E1 高視野姿態（訓練 reset 與此完全一致，不可只改一邊）。
    #   sim (0,-0.275,-1.42,-1.42,0) → API (90, 74.2, 8.6, 8.6, 90)；S6=30 開爪。
    #   舊值（v21/C3）：API (90, 67.08, 9.79, 9.79, 90)。
    #
    # E1 相對 C3 的實際幾何（本檔 FKComputer 在 training frame 下算出，非引述）：
    #   相機 (mono_link)   0.2183 → 0.2340 m（+1.58 cm）
    #   gripper_center     0.1439 → 0.1558 m（+1.19 cm，張爪）
    #   指墊最低點          0.1110 → 0.1231 m（+1.21 cm，張爪）
    #   光軸離鉛垂          3.34°（偏後）→ 1.40°（偏前）—— 過了鉛垂線
    # DEPLOY_V23_MANIFEST.md 寫 22.7→24.3 cm，是 URDF 原點座標；本檔一律用 training
    # frame（低 0.907 cm，見 dc.URDF_TO_TRAINING_FRAME）。兩邊差值都是 +1.6 cm，一致。
    #
    # ⚠ 光軸幾乎鉛垂（1.4°）意味著 arm_cam_geometry 的三角測距在 E1 更不可用：
    #   地面距離對 theta 的敏感度趨近奇異。E1 的視覺**只能**走實測 homography，
    #   這也是 vision_grasp_bridge 在 grasp-home 硬性要求校正檔的原因。
    #
    # nav→E1 轉場請走 validate_nav_to_grasp_transition.py 輸出的 waypoint，
    #   不可單段直跳（首次實機需人在旁、手扶急停）。
    home_deg: Tuple[float,...] = (90.0, 74.2, 8.6, 8.6, 90.0, 30.0)

    # PPO training starting pose for run()'s own guarded startup move, if it needs to
    # differ from home_deg. None (default) = use home_deg, which is every existing
    # caller's behaviour (CLI, all three self-tests, every real run so far) and is
    # unchanged by adding this field. Exists for a vision pipeline that drives around
    # with the arm at a different (e.g. lower, camera-friendlier) nav pose and only
    # raises to the trained starting pose right before handing off to the policy —
    # mirrors grasp/x3plus_real_grasp.py's home_deg/grasp_home_deg split.
    grasp_home_deg: Optional[Tuple[float, ...]] = None

    # ── gripper sim range (must match training env) ──
    gripper_sim_open: float = -1.5   # rad
    gripper_sim_closed: float = 0.0  # rad
    # 實機校正（2026-05-25）：API 30°=張開、180°=閉合。舊版 open=180/closed=30 是
    # 過時的顛倒設定，會讓 policy 一開始就把爪子夾死。
    gripper_hw_open: float = 30.0    # degrees (OPEN)
    gripper_hw_closed: float = 180.0 # degrees (CLOSED)

    # ── arm sim range (URDF limits in radians) ──
    arm_sim_limits: Tuple[Tuple[float, float], ...] = (
        (-1.5708, 1.5708),   # S1
        (-1.5708, 1.5708),   # S2
        (-1.5708, 1.5708),   # S3
        (-1.5708, 1.5708),   # S4
        (-1.5708, 3.14159),  # S5
    )
    # Invert flag for each arm joint: True = hw = center - sim_deg (inverted)
    # 實機校正定案（2026-05-25，實抓驗證）：全部 False，API = 90 + sim_deg。
    # 舊版 S2=True 是「把 App 鏡像顯示當物理角」校出來的雙重翻轉，會讓 S2 反向。
    arm_hw_invert: Tuple[bool, ...] = (False, False, False, False, False)
    arm_hw_center: Tuple[float, ...] = (90.0, 90.0, 90.0, 90.0, 90.0)
    arm_hw_range: Tuple[Tuple[float, float], ...] = (
        (0.0, 180.0),   # S1
        (0.0, 180.0),   # S2
        (0.0, 180.0),   # S3
        (0.0, 180.0),   # S4
        (0.0, 270.0),   # S5
    )

    def __post_init__(self) -> None:
        # The settle >= run_time rule, enforced rather than commented. Reading the
        # encoder mid-travel makes a healthy jaw look like one that stopped against
        # something, and stop_on_jaw_contact reads that as a grasp: the arm would
        # lift on empty air and report success. That failure is invisible in a log,
        # so the cheap moment to catch it is here, before anything moves.
        # Enforced for the CLOSE pair only, and that asymmetry is the whole point.
        # attempt_close is the one caller that passes stop_on_jaw_contact, so it is
        # the one place where reading early is not merely slow: a servo still in
        # travel reads as a jaw not keeping up with its command, which is the exact
        # signature of contact, and the run would lift on empty air and report a
        # grasp. A short settle on a lift or home move only costs an extra iteration
        # and fails safe through the existing "no progress" abort, so it is left
        # alone -- the long-shipped home pair (400 ms / 0.35 s) is itself slightly
        # short, and retroactively refusing to start over that would be wrong.
        #
        # The bus quiet window is deliberately not part of this bound: read_degrees
        # tops that up itself from _last_write_t.
        run_s = float(self.close_run_time_ms) / 1000.0
        if float(self.close_settle_s) < run_s:
            raise ValueError(
                f"close_settle_s={self.close_settle_s} s is shorter than "
                f"close_run_time_ms={self.close_run_time_ms} ms. The jaw would be "
                f"read while it is still travelling, which the contact detector "
                f"cannot tell apart from an object -- the run would lift on nothing "
                f"and call it a grasp.")
        # A step the force ceiling would clip on every iteration is not a faster
        # close, it is the same close with a misleading config.
        if self.jaw_max_lag_deg > 0.0 and self.jaw_max_delta_deg > self.jaw_max_lag_deg:
            raise ValueError(
                f"jaw_max_delta_deg={self.jaw_max_delta_deg} exceeds "
                f"jaw_max_lag_deg={self.jaw_max_lag_deg}; the force ceiling would "
                f"clip every step back to the ceiling and the close would not speed up.")

        # A big jaw step and the slip detector cancel each other out, measured:
        # with the ratio test on, an object slipping 3 deg per write is caught as
        # "slipping" 16 deg past contact at an 8 deg step, but only as a late
        # "stalled" 33 deg past it at 12. The reason is that the force ceiling
        # clips the command down to the encoder's own creep rate, and the ratio
        # test then compares the encoder against that clipped step -- so the jaw
        # looks like it is keeping up with a command that is barely moving.
        # Doubling the over-squeeze is the clicking and gear grinding this detector
        # exists to prevent, so the two must not be combined silently. Speed is the
        # right default here because the shipped jaw_contact_track_fraction is 0;
        # anyone turning the slip detector on is asking for the careful close and
        # should get it.
        if self.jaw_contact_track_fraction > 0.0 and self.jaw_max_delta_deg > self.max_delta_deg:
            raise ValueError(
                f"jaw_contact_track_fraction={self.jaw_contact_track_fraction} needs "
                f"jaw_max_delta_deg <= {self.max_delta_deg} (it is "
                f"{self.jaw_max_delta_deg}). A larger jaw step defeats the slip test: "
                f"the force ceiling flattens the commanded step, the ratio test stops "
                f"seeing the slip, and the squeeze past contact roughly doubles. Pick "
                f"one: --jaw-step-deg 8 for the careful close, or drop "
                f"--jaw-track-fraction for the fast one.")


def resolve_object_height_for_contract(
    contract: dc.DeployContract,
    object_pos,
    reported_height: Optional[float],
    ground_height: float,
) -> Tuple[float, str]:
    """Resolve one episode's object height without weakening the 32D contract.

    The 28D observation does not contain height, but deployment still needs height
    for the stage-0 target and finger-pad close gate. If the detector omits it, the
    only geometry available is an explicit resting-object assumption: ``z`` is the
    centroid of a vertically symmetric object resting on the configured floor, hence
    ``height = 2 * (centroid_z - ground_z)``. The value is frozen for the episode.

    The 32D contract observes height directly and therefore never takes this fallback;
    a missing value remains a hard error.
    """
    pos = np.asarray(object_pos, dtype=np.float64)
    if pos.shape != (3,) or not np.all(np.isfinite(pos)):
        raise MissingObjectHeightError(
            "object position must be a finite 3D centroid before height can be resolved"
        )
    ground = float(ground_height)
    if not np.isfinite(ground):
        raise MissingObjectHeightError("ground height must be finite")

    if reported_height is not None:
        height = float(reported_height)
        if not np.isfinite(height) or height <= 0.0:
            raise MissingObjectHeightError(
                f"reported object height must be finite and positive, got {reported_height!r}"
            )
        return height, "reported"

    if contract.requires_object_height:
        raise MissingObjectHeightError(dc.DETECTION_CONTRACT_REQUIRED)

    height = 2.0 * (float(pos[2]) - ground)
    if not np.isfinite(height) or height <= 0.0:
        raise MissingObjectHeightError(
            "28D resting-centroid fallback requires centroid_z > ground_height; "
            f"got centroid_z={float(pos[2]):.6f}, ground_height={ground:.6f}"
        )
    return height, "resting_centroid_geometry"


def control_tcp_position(tcp_pos, forward_error_m: float) -> np.ndarray:
    """Return the TCP position used by the policy/gate after a bounded X correction.

    ``forward_error_m`` is positive when FK reports the gripper farther forward
    than it physically lands. Subtracting it from the reported base +X makes
    both the policy observation and the close gate demand more forward travel.
    Collision/floor FK remains unmodified.
    """
    tcp = np.asarray(tcp_pos, dtype=np.float64).reshape(-1)[:3].copy()
    error = float(forward_error_m)
    if tcp.shape != (3,) or not np.all(np.isfinite(tcp)):
        raise ValueError("TCP position must contain three finite values")
    if not np.isfinite(error) or not 0.0 <= error <= 0.020:
        raise ValueError(
            f"TCP forward error must be finite and in [0, 20] mm; got {error*1000!r}")
    tcp[0] -= error
    return tcp
# ═══════════════════════════════════════════════════════════════════════════
# Joint Mapper — sim radians ↔ hardware degrees
# ═══════════════════════════════════════════════════════════════════════════

class JointMapper:
    def __init__(self, cfg: DeployConfig):
        self.cfg = cfg

    def sim_arm_to_hw_deg(self, sim_angles_rad: np.ndarray) -> List[float]:
        """Convert 5 arm joint angles (radians, sim) → hardware degrees."""
        hw = []
        for i, (rad, invert, center, (hw_lo, hw_hi), (sim_lo, sim_hi)) in enumerate(zip(
            sim_angles_rad,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_hw_range,
            self.cfg.arm_sim_limits,
        )):
            sim_deg = math.degrees(float(rad))
            if invert:
                deg = center - sim_deg
            else:
                deg = center + sim_deg
            hw.append(float(np.clip(deg, hw_lo, hw_hi)))
        return hw

    def sim_grip_to_hw_deg(self, grip_rad: float) -> float:
        """Convert gripper joint angle (radians, sim) → hardware degree."""
        cfg = self.cfg
        grip_rad = float(np.clip(grip_rad, cfg.gripper_sim_open, cfg.gripper_sim_closed))
        ratio = (grip_rad - cfg.gripper_sim_open) / (cfg.gripper_sim_closed - cfg.gripper_sim_open)
        hw = cfg.gripper_hw_open + ratio * (cfg.gripper_hw_closed - cfg.gripper_hw_open)
        return float(np.clip(hw, min(cfg.gripper_hw_open, cfg.gripper_hw_closed),
                                  max(cfg.gripper_hw_open, cfg.gripper_hw_closed)))

    def hw_deg_to_sim_arm(self, hw_degs: List[float]) -> np.ndarray:
        """Convert 5 hardware degrees → sim radians (for observation building)."""
        sim = []
        for i, (deg, invert, center, (hw_lo, hw_hi), (sim_lo, sim_hi)) in enumerate(zip(
            hw_degs,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_hw_range,
            self.cfg.arm_sim_limits,
        )):
            if invert:
                sim_deg = center - deg
            else:
                sim_deg = deg - center
            rad = math.radians(sim_deg)
            sim.append(float(np.clip(rad, sim_lo, sim_hi)))
        return np.array(sim, dtype=np.float32)

    def hw_deg_to_sim_grip(self, hw_deg: float) -> float:
        """Convert hardware gripper degree → sim radians."""
        cfg = self.cfg
        ratio = (hw_deg - cfg.gripper_hw_open) / (cfg.gripper_hw_closed - cfg.gripper_hw_open + 1e-9)
        rad = cfg.gripper_sim_open + ratio * (cfg.gripper_sim_closed - cfg.gripper_sim_open)
        return float(np.clip(rad, cfg.gripper_sim_open, cfg.gripper_sim_closed))

    def norm_action_to_sim_angles(
        self,
        action: np.ndarray,
        current_arm_rads: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, float]:
        """Convert a normalized 6D policy action → (5 arm radians, gripper radian).

        Delegates to ``deploy_contract.decode_arm_action`` so eval and the robot share
        one implementation of the arm semantics. Under the incremental contract the
        action is a delta from ``current_arm_rads``, which is therefore required;
        decoding it as an absolute target instead would command the arm across its full
        range on every step.

        Only dims 0..4 are contract-dependent — the gripper is absolute everywhere.
        """
        contract = dc.get_contract(self.cfg.contract_name)
        if contract.arm_action_mode == "incremental" and current_arm_rads is None:
            raise ContractMismatchError(
                f"contract {contract.name!r} has incremental arm actions but no current "
                "arm pose was supplied to decode the delta against"
            )
        # Validate here, not only in the caller. _apply_policy_action already rejects a
        # non-finite action, but that makes the guarantee a caller convention rather
        # than a property of this method -- and under the incremental contract a NaN is
        # ADDED to the current pose, so it poisons the running joint state instead of
        # just producing one bad command. Clipping does not remove it: np.clip leaves
        # NaN as NaN, and every subsequent bound check compares false against it.
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (6,) or not np.all(np.isfinite(action)):
            raise ValueError(
                f"policy action must be a finite (6,) vector, got shape {action.shape}"
                + ("" if action.shape != (6,) else f" values {action.tolist()}"))
        if current_arm_rads is None:
            current_arm_rads = np.zeros(5, dtype=np.float32)

        arm_rads = dc.decode_arm_action(
            contract, action, current_arm_rads, self.cfg.arm_sim_limits
        ).astype(np.float32)
        grip_rad = dc.decode_gripper_action(action)
        return arm_rads, float(grip_rad)


# ═══════════════════════════════════════════════════════════════════════════
# Servo Controller (Yahboom Rosmaster_Lib wrapper)
# ═══════════════════════════════════════════════════════════════════════════

class ServoReadResult(NamedTuple):
    """Result of reading the servos.

    ``valid`` is False when any servo could not be read. Callers must treat an
    invalid read as "the robot's state is unknown" and stop, not as "assume the last
    command was reached" — that assumption is what lets a controller believe it is
    holding an object it never picked up.
    """
    degrees: List[float]
    valid: bool
    reason: str


def _rosmaster_raw_to_deg(servo_id: int, raw) -> Optional[float]:
    """Rosmaster_Lib's own raw-count → degree conversion, replicated verbatim.

    Rosmaster_Lib.__arm_convert_angle is name-mangled private, and the public
    wrapper that would call it for us — get_uart_servo_angle — is precisely the
    one we cannot use (see ServoController._read_one). So the arithmetic is copied
    exactly, int(x + 0.5) and all, and a reading taken here is identical to one
    taken through the driver. It is also the exact inverse of the conversion
    set_uart_servo_angle_array applies to our commands, which is what keeps the
    rate limiter and the encoder talking about the same numbers.

    Returns None for a value the driver itself would have rejected: out of range
    means the raw count is not a position this joint can actually be in, and
    treating it as one would feed a fabricated angle into FK.
    """
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    if servo_id in (1, 2, 3, 4):
        deg = int((v - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        hi = 180
    elif servo_id == 5:
        deg = int((270 - 0) * (v - 380) / (3700 - 380) + 0 + 0.5)
        hi = 270
    elif servo_id == 6:
        deg = int((180 - 0) * (v - 900) / (3100 - 900) + 0 + 0.5)
        hi = 180
    else:
        return None
    return None if deg < 0 or deg > hi else float(deg)


class ServoController:
    # Consecutive all-invalid array responses before the request is abandoned. More
    # than one, because a single all-invalid response is ambiguous: it looks the same
    # whether the board does not serve the request or a write was still on the wire.
    _ARRAY_DUD_LIMIT = 3

    def __init__(self, cfg: DeployConfig, dry_run: bool = False):
        self.cfg = cfg
        self._device = None
        self._has_servo = False

        # Deliberately NOT "dry_run = dry_run or driver_missing". That fallback is what
        # made a driverless --real look like a successful run while nothing moved. A
        # missing driver leaves dry_run False and _has_servo False, so every write is
        # refused loudly further down instead of being quietly printed. main() also
        # gates --real up front; this is the second line of defence for direct callers.
        if not dry_run and _ROSMASTER_AVAILABLE:
            try:
                self._device = _RosmasterCls(com=cfg.serial_port)
                self._device.create_receive_threading()
                self._has_servo = True
                print(f"[INFO] Rosmaster_Lib ready on {cfg.serial_port}.")
            except Exception as e:
                self._close_device()
                raise RuntimeError(
                    f"--real could not initialize Rosmaster_Lib on {cfg.serial_port}: {e}"
                ) from e

        self.dry_run = dry_run
        self._last_deg = list(cfg.home_deg)   # [S1..S5, S6]
        # When the bus last carried a write, so read_degrees can wait out the echo.
        # 0.0 means "long ago": the first read never needs to wait.
        self._last_write_t = 0.0
        # Consecutive array reads that returned nothing usable. Once this hits the
        # limit the array request is abandoned for good, so a board that does not
        # serve it stops costing a 30 ms timeout on every single read.
        self._array_dud_streak = 0
        self._read_fail_counts = [0] * 6
        # Responses that came back labelled with a servo other than the one asked —
        # the driver's get_uart_servo_value returning whatever landed first. A
        # non-zero count here is the signature of a shifted bus pipeline, and it is
        # the number that tells the operator to raise --bus-quiet-ms rather than go
        # looking for a broken cable.
        self._read_misdirects = 0
        # Dry-run models the plant well enough to exercise the state machine, but its
        # readings are fabricated. Anything that treats a reading as evidence about the
        # physical world has to check this.
        self.readings_are_simulated = self.dry_run

    @property
    def device(self):
        """The underlying Rosmaster handle, or None in dry-run / without hardware."""
        return self._device

    def _close_device(self):
        """Release the board: stop the receive thread, then close the serial handle.

        Leaving either open keeps /dev/myserial claimed, so the next run fails to
        open the port for reasons that look nothing like the real cause.
        """
        device, self._device = self._device, None
        self._has_servo = False
        if device is None:
            return
        try:
            device.cancel_receive_threading()
        except Exception:
            pass
        serial_obj = getattr(device, "ser", None)
        if serial_obj is not None:
            try:
                serial_obj.close()
            except Exception:
                pass

    def send_degrees(self, deg6: List[float], run_time_ms: Optional[int] = None) -> bool:
        """Send a 6-DOF servo command with per-step rate limiting.

        Returns True only if the command actually went out. A failed write used to be
        printed and swallowed, which let the caller carry on updating its internal
        pose as though the arm had moved.

        NOTE: the command is rate-limited to ``max_delta_deg`` per call, so a single
        call does NOT reach a distant target. Use
        ``GraspController.move_guarded_and_verified`` for anything that must arrive.

        deg6: [S1, S2, S3, S4, S5, S6] in hardware degrees
        """
        if run_time_ms is None:
            run_time_ms = self.cfg.servo_run_time_ms
        # Rosmaster_Lib clamps anything above 2000 ms internally. Clamp here too so the
        # logs and the software's own timing describe the command the board really sees.
        run_time_ms = min(int(run_time_ms), 2000)

        # Rate limiting. S6 has its own, larger cap: see jaw_max_delta_deg.
        safe_deg = []
        for i, (target, prev) in enumerate(zip(deg6, self._last_deg)):
            cap = (self.cfg.jaw_max_delta_deg if i == 5 else self.cfg.max_delta_deg)
            delta = float(np.clip(target - prev, -cap, cap))
            safe_deg.append(float(np.clip(prev + delta, 0.0, 270.0)))

        if self.dry_run:
            self._last_deg = safe_deg[:]
            print(f"[DRY] S1={safe_deg[0]:.1f}° S2={safe_deg[1]:.1f}° "
                  f"S3={safe_deg[2]:.1f}° S4={safe_deg[3]:.1f}° "
                  f"S5={safe_deg[4]:.1f}° S6={safe_deg[5]:.1f}° "
                  f"t={run_time_ms}ms")
            return True

        if not self._has_servo:
            # Not dry-run, but there is no board to talk to. Refuse loudly instead of
            # printing as though it worked: the caller must not advance its pose.
            print("[ERROR] Servo write refused: no Rosmaster device "
                  "(driver missing or init failed). The arm has NOT moved.")
            return False

        # set_uart_servo_angle_array validates its own arguments and RETURNS SILENTLY
        # if any angle is out of the board's range (S1-S4 0-180, S5 0-270, S6 0-180) --
        # no exception, no send. Advancing _last_deg after that would leave the rate
        # limiter tracking a pose the arm never took. JointMapper already clamps to
        # these same ranges, so this should never fire; if it ever does, something
        # upstream is wrong and stopping is the only safe answer.
        board_range = list(self.cfg.arm_hw_range) + [(0.0, 180.0)]
        out_of_range = [
            f"S{i+1}={v:.1f} not in [{lo}, {hi}]"
            for i, (v, (lo, hi)) in enumerate(zip(safe_deg, board_range))
            if not (lo <= v <= hi)
        ]
        if out_of_range:
            print("[ERROR] Servo write refused — the board would silently drop this "
                  "command: " + "; ".join(out_of_range))
            return False

        try:
            # run_time unit: milliseconds (verified against Rosmaster_Lib docs)
            self._device.set_uart_servo_angle_array(
                angle_s=[
                    safe_deg[0], safe_deg[1], safe_deg[2],
                    safe_deg[3], safe_deg[4], safe_deg[5],
                ],
                run_time=run_time_ms,
            )
        except Exception as e:
            # Do NOT advance _last_deg: the command never left, so the arm is still
            # wherever it was, and the rate limiter must keep working from there.
            print(f"[ERROR] Servo write failed: {e}")
            return False
        # Stamp even though the write succeeded logically: the bytes are still going
        # out on the shared line, and read_degrees waits out that window.
        self._last_write_t = time.time()
        self._last_deg = safe_deg[:]
        return True

    def read_degrees(self) -> ServoReadResult:
        """Read the servos. Fails closed — no last-command fallback.

        The previous version substituted the last command whenever a read failed,
        which makes an unreachable servo look like a perfectly tracking one. Anything
        downstream that checks "did we arrive?" or "is something in the jaw?" would
        then be reading its own command back.
        """
        if self.dry_run:
            return ServoReadResult(self._last_deg[:], True, "dry-run (simulated)")
        if not self._has_servo:
            # Not dry-run and no board: the pose is genuinely unknown. Returning
            # _last_deg here would hand back our own commands as if they were
            # measurements — exactly the fallback this method exists to avoid.
            return ServoReadResult(
                [float("nan")] * 6, False,
                "no Rosmaster device (driver missing or init failed)")
        # The six servos are daisy-chained on one half-duplex UART: a read issued
        # while the write burst is still on the wire has nowhere to come back and
        # the driver returns -1 after its 30 ms timeout. Give the bus a quiet window
        # first. (Evidence: the startup walk to C3 reads after a 0.35 s settle and
        # succeeds; the first policy step reads with settle 0.0 and S1 came back -1.)
        quiet = self.cfg.bus_quiet_s - (time.time() - self._last_write_t)
        if quiet > 0:
            time.sleep(quiet)

        attempts = max(1, int(self.cfg.read_attempts))
        angles: List[Optional[float]] = [None] * 6

        for attempt in range(attempts):
            missing = [i for i in range(6) if not self._is_angle(angles[i])]
            if not missing:
                break
            if attempt:
                time.sleep(self.cfg.read_retry_s)

            # One transaction can fill several holes at once, so it is only worth
            # issuing while more than one servo is outstanding. Filling the LAST hole
            # with a whole-array request would cost a transaction to re-read five
            # angles already in hand — which is exactly the trap the first version of
            # this fell into: an array response missing only S6 sent it back to read
            # all six individually, 7 transactions where 2 would do.
            if len(missing) > 1 and self._array_dud_streak < self._ARRAY_DUD_LIMIT:
                arr = self._read_angle_array()
                if arr is None:
                    # The API is absent or raised: it will not start working later.
                    self._array_dud_streak = self._ARRAY_DUD_LIMIT
                else:
                    filled = 0
                    for i in missing:
                        if self._is_angle(arr[i]):
                            angles[i] = float(arr[i])
                            filled += 1
                    # All -1 is ambiguous — a board that does not serve this request,
                    # or a collision. Do not condemn it on one sample; a collision
                    # would clear on the next attempt.
                    self._array_dud_streak = 0 if filled else self._array_dud_streak + 1
                    if self._array_dud_streak == self._ARRAY_DUD_LIMIT:
                        print("[INFO] get_uart_servo_angle_array keeps returning "
                              "nothing usable; per-servo reads from here on.")
                missing = [i for i in range(6) if not self._is_angle(angles[i])]

            for i in missing:
                if self._is_angle(angles[i]):
                    continue    # a misdirected answer already filled this one in
                sid, deg = self._read_one(i + 1)
                # Credit the servo that answered, not the one that was asked. On a
                # shifted pipeline these differ, and using the id on the packet is
                # what stops one late response from condemning every read after it.
                if sid and self._is_angle(deg):
                    angles[sid - 1] = float(deg)

        out = [float(a) if self._is_angle(a) else float("nan") for a in angles]
        missing = [i for i in range(6) if not self._is_angle(angles[i])]
        for i in missing:
            self._read_fail_counts[i] += 1
        if missing:
            names = ", ".join(f"S{i+1}" for i in missing)
            suffix = f" (after {attempts} attempts)" if attempts > 1 else ""
            return ServoReadResult(
                out, False, f"unreadable: {names}{suffix}")
        return ServoReadResult(out, True, "ok")

    def read_failure_summary(self) -> str:
        """Per-servo read failures over this run — the evidence for tuning the bus."""
        total = sum(self._read_fail_counts)
        # Worth reporting even when nothing failed: recovered misdirects are the
        # difference between "the bus is fine" and "the bus is shifted but the read
        # layer is absorbing it", and only the second one gets worse under load.
        shifted = (f"; {self._read_misdirects} misdirected response(s) recovered"
                   if self._read_misdirects else "")
        if not total:
            return f"servo reads: no failures{shifted}"
        per = ", ".join(f"S{i+1}×{n}" for i, n in enumerate(self._read_fail_counts) if n)
        return f"servo read failures: {total} ({per}){shifted}"

    def _read_angle_array(self) -> Optional[List]:
        """All six angles in ONE bus transaction, or None if that API is unusable.

        get_uart_servo_angle_array issues a single FUNC_ARM_CTRL request with one
        30 ms budget. The per-servo call is six separate request/response round trips
        on a half-duplex chain, each with its own 30 ms timeout — up to 180 ms, which
        does not fit inside the 100 ms control period at 10 Hz, and six times the
        opportunity for a collision. Same conversion and units either way.

        Note that FUNC_ARM_CTRL is an ARM request: a board that answers it for the
        five arm joints but leaves the gripper empty is a normal outcome, not a
        failure, which is why the caller fills the gaps per-servo instead of
        discarding the whole response.
        """
        getter = getattr(self._device, "get_uart_servo_angle_array", None)
        if getter is None:
            return None
        try:
            angles = getter()
        except Exception:
            return None
        if angles is None or len(angles) != 6:
            return None
        return list(angles)

    def _read_one(self, servo_id: int):
        """Ask one servo for its angle. Returns (answering_id, degrees) or (0, None).

        Deliberately NOT get_uart_servo_angle(). That wrapper calls
        get_uart_servo_value, which hands back the FIRST response to land no matter
        which servo sent it:

            while timeout > 0:
                if self.__read_id > 0:
                    return self.__read_id, self.__read_val

        — there is no comparison against the servo that was asked. The wrapper then
        checks the id itself, finds a mismatch, and returns -1, throwing away a
        perfectly good reading and reporting a failure for a servo that answered.

        On a loaded half-duplex chain one late response shifts the whole pipeline by
        one: S2's answer is collected by S3's request, so S3 reports -1 while its own
        answer is collected by S4, and so on. The symptom is a run of ADJACENT servos
        all unreadable at once — the S3+S4 startup failure of 2026-07-31, and the
        reason retrying did not clear it: each retry re-requests and keeps the
        pipeline shifted.

        Crediting the answer to whoever actually sent it breaks the cascade. A
        shifted read then costs one extra round trip instead of aborting the run,
        and the misdirect count is kept as evidence that this is what is happening.
        """
        getter = getattr(self._device, "get_uart_servo_value", None)
        if getter is None:
            # A driver without the raw call (or a stub). Fall back to the wrapper and
            # accept its id-mismatch blindness — better than not reading at all.
            try:
                return servo_id, self._device.get_uart_servo_angle(servo_id)
            except Exception:
                return 0, None
        try:
            got = getter(servo_id)
            read_id, raw = int(got[0]), got[1]
        except Exception:
            return 0, None
        if read_id < 1 or read_id > 6:
            return 0, None      # -1 (timeout) or -2 (driver exception)
        if read_id != servo_id:
            self._read_misdirects += 1
        return read_id, _rosmaster_raw_to_deg(read_id, raw)

    @staticmethod
    def _is_angle(a) -> bool:
        # NaN must be rejected explicitly: "nan < 0" is False, so a bare comparison
        # would wave it through as a valid measurement.
        return (isinstance(a, (int, float)) and not isinstance(a, bool)
                and math.isfinite(a) and a >= 0)

    def emergency_stop(self):
        """Immediate stop: hold position rather than sweeping to home.

        Commanding a long move to home during an emergency is the opposite of
        stopping — it sends the arm on an unguarded sweep across the workspace while
        something is already wrong. Freeze instead; the operator decides what next.
        """
        print("[SAFETY] Emergency stop — freezing at the current commanded pose.")
        try:
            self.send_degrees(self._last_deg[:], run_time_ms=200)
        except Exception as e:
            print(f"[SAFETY] Could not send freeze command: {e}")


# ═══════════════════════════════════════════════════════════════════════════
# Forward Kinematics via PyBullet DIRECT
# ═══════════════════════════════════════════════════════════════════════════

class FKComputer:
    """Headless PyBullet instance used only for forward kinematics.

    The TCP this returns is ``gripper_center`` — the midpoint of the two finger-pad
    link COMs — which is what the training env feeds into obs[6:9]. It is NOT the
    arm_link5 COM: that point sits 6.7–8.1 cm below gripper_center depending on pose,
    and using it was the deployment-side TCP mismatch reported from the robot.

    Because the gripper is a parallel linkage, every one of its six joints has to be
    driven from ``grip_rad`` through its multiplier before the pad links are read;
    setting ``grip_joint`` alone leaves the pads at the wrong place entirely.
    """

    def __init__(self, urdf_path: str):
        self.physics_client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self.physics_client)

        urdf_abs = Path(__file__).parent / urdf_path
        if not urdf_abs.exists():
            raise FileNotFoundError(f"URDF not found: {urdf_abs}")

        # Prefer the FK-only URDF when it has been generated. It carries collision
        # meshes on the six gripper links only -- the sole links whose AABB anything
        # here reads -- which drops 59.5 MB of STL nobody parses for a reason.
        # Measured on the dev machine: 3.66 s -> 0.14 s, and the Nano reads the same
        # bytes off an SD card. Proven identical over 402 poses by
        # x3plus/make_deploy_urdf.py --verify. Falling back to the full URDF when it
        # is absent costs startup time and nothing else: the two are the same model
        # as far as this class is concerned.
        deploy_urdf = urdf_abs.with_name(urdf_abs.stem + "_deploy" + urdf_abs.suffix)
        chosen = deploy_urdf if deploy_urdf.exists() else urdf_abs

        # Loaded at the training-frame offset, not the origin: iGibson's loader merges
        # links and settles the base, so a bare origin load puts every reported
        # position ~2.2 cm away from what the policy saw in training. See
        # deploy_contract.URDF_TO_TRAINING_FRAME.
        #
        # IGNORE_VISUAL_SHAPES, and deliberately NOT the collision flag. The .obj
        # visual meshes are ~88 MB and nothing here renders: DIRECT mode, no
        # getCameraImage, no getVisualShapeData. Skipping them halves the load with
        # the geometry bit-identical. URDF_IGNORE_COLLISION_SHAPES would be faster
        # still and is a trap: it does not raise, it silently returns a pad_bottom_z
        # 27 mm too high, i.e. 27 mm of clearance the floor guard does not have,
        # against an 8 mm margin.
        self.body_id = p.loadURDF(
            str(chosen),
            basePosition=list(dc.URDF_TO_TRAINING_FRAME),
            useFixedBase=True,
            flags=p.URDF_IGNORE_VISUAL_SHAPES,
            physicsClientId=self.physics_client,
        )
        self.urdf_used = chosen

        # Discover joint indices (same suffix-match logic as x3plus_ground_grasp_env.py)
        target_arm = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        target_grip = list(dc.GRIPPER_JOINT_MULTIPLIERS.keys())
        self.name2id = {}
        self.short2full = {}
        for j in range(p.getNumJoints(self.body_id, physicsClientId=self.physics_client)):
            info = p.getJointInfo(self.body_id, j, physicsClientId=self.physics_client)
            full = info[1].decode("utf-8")
            self.name2id[full] = j
            for t in target_arm + target_grip:
                if full.endswith(t):
                    self.short2full[t] = full

        self.arm_indices = [self.name2id[self.short2full[t]] for t in target_arm if t in self.short2full]
        if len(self.arm_indices) != 5:
            raise RuntimeError(f"URDF exposed {len(self.arm_indices)} arm joints, expected 5")

        # (joint index, multiplier) for the whole linkage, driven together from grip_rad
        self.grip_drive = []
        for short, mult in dc.GRIPPER_JOINT_MULTIPLIERS.items():
            full = self.short2full.get(short)
            if full is not None:
                self.grip_drive.append((self.name2id[full], float(mult)))

        self.tcp_link_r = self.name2id.get(self.short2full.get(dc.TCP_RIGHT_JOINT, ""))
        self.tcp_link_l = self.name2id.get(self.short2full.get(dc.TCP_LEFT_JOINT, ""))
        if self.tcp_link_r is None or self.tcp_link_l is None:
            raise RuntimeError(
                f"URDF is missing {dc.TCP_RIGHT_JOINT}/{dc.TCP_LEFT_JOINT}; cannot compute "
                "gripper_center, and falling back to arm_link5 would silently reintroduce "
                "the TCP mismatch. Refusing to run."
            )

        self.ee_link = self.arm_indices[-1]   # arm_joint5 child link — orientation only

        # The floor guard is made of getAABB on these links, and getAABB does not
        # raise when the collision shape is missing or a placeholder — it quietly
        # returns a smaller box, i.e. clearance the robot does not have. Both ways
        # of losing it (URDF_IGNORE_COLLISION_SHAPES, or a deploy URDF that stripped
        # the wrong link) show up here as "not a mesh", so check once at load rather
        # than discover it with the fingers in the floor.
        missing = []
        for idx in {self.tcp_link_r, self.tcp_link_l, *(i for i, _ in self.grip_drive)}:
            shapes = p.getCollisionShapeData(self.body_id, idx,
                                             physicsClientId=self.physics_client)
            if not shapes or shapes[0][2] != p.GEOM_MESH:
                missing.append(idx)
        if missing:
            raise RuntimeError(
                f"{chosen.name}: gripper links {sorted(missing)} have no collision "
                f"mesh, so pad_bottom_z would report clearance that is not there. "
                f"Regenerate with x3plus/make_deploy_urdf.py --verify, or delete the "
                f"_deploy URDF to fall back to the full one.")

        print(f"[FK] PyBullet DIRECT ready ({chosen.name}). tcp=gripper_center("
              f"{self.tcp_link_r},{self.tcp_link_l}) arm={self.arm_indices} "
              f"grip_linkage={[i for i, _ in self.grip_drive]}")

    def set_base_pose(self, position, orientation) -> None:
        """Move the FK model's base to a measured pose (odometry on the real robot).

        This is NOT cosmetic: the X3Plus sits on a wheeled base that is free to shift,
        and the policy consumes absolute tcp_pos. If the base moves and FK assumes it
        did not, every reported position is wrong by that drift. Measured in sim: 40
        steps of aggressive motion moved the base enough to push observation parity
        from 1.6 mm to 3.4 mm. Either hold the base still while grasping, or feed
        odometry in here.
        """
        p.resetBasePositionAndOrientation(
            self.body_id, list(position), list(orientation),
            physicsClientId=self.physics_client)

    def _set_state(self, arm_rads: np.ndarray, grip_rad: float) -> None:
        for idx, rad in zip(self.arm_indices, arm_rads):
            p.resetJointState(self.body_id, idx, float(rad), physicsClientId=self.physics_client)
        for idx, mult in self.grip_drive:
            p.resetJointState(self.body_id, idx, float(grip_rad) * mult,
                              physicsClientId=self.physics_client)

    def _gripper_center(self) -> np.ndarray:
        pos_r = p.getLinkState(self.body_id, self.tcp_link_r, computeForwardKinematics=True,
                               physicsClientId=self.physics_client)[0]
        pos_l = p.getLinkState(self.body_id, self.tcp_link_l, computeForwardKinematics=True,
                               physicsClientId=self.physics_client)[0]
        return (np.array(pos_r, dtype=np.float64) + np.array(pos_l, dtype=np.float64)) / 2.0

    def compute(self, arm_rads: np.ndarray, grip_rad: float) -> Tuple[np.ndarray, np.ndarray]:
        """Set joint states and return (gripper_center, arm_link5 quaternion xyzw).

        No stepSimulation: resetJointState + computeForwardKinematics is a pure
        kinematic query, whereas stepping would let gravity perturb the pose between
        the reset and the read.
        """
        self._set_state(arm_rads, grip_rad)
        tcp_pos = self._gripper_center().astype(np.float32)
        state = p.getLinkState(
            self.body_id, self.ee_link,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client,
        )
        tcp_quat = np.array(state[1], dtype=np.float32)   # (x, y, z, w)
        return tcp_pos, tcp_quat

    def pad_bottom_z(self, arm_rads: np.ndarray, grip_rad: float) -> float:
        """Lowest finger-pad AABB point, matching training ``_get_pad_bottom_z``."""
        self._set_state(arm_rads, grip_rad)
        bottoms = []
        for idx in (self.tcp_link_r, self.tcp_link_l):
            try:
                bottoms.append(float(p.getAABB(
                    self.body_id, idx, physicsClientId=self.physics_client)[0][2]))
            except Exception:
                continue
        if bottoms:
            return float(min(bottoms))
        return float(self._gripper_center()[2] - dc.GRASP_PAD_OFFSET)

    def min_gripper_link_z(self, arm_rads: np.ndarray, grip_rad: float,
                           finger_error_m: float = 0.0) -> float:
        """Lowest point of any gripper link at this pose (same metric as training).

        ``finger_error_m`` raises ONLY the two finger pads, because that is where the
        measured URDF-vs-hardware discrepancy lives: the modelled finger is 16.7 mm
        longer than the real one, so the real pad bottom sits that much higher, while
        every other linkage part is modelled correctly. Default 0.0 keeps the
        training-identical metric; the floor guard is the only caller that raises it.
        """
        self._set_state(arm_rads, grip_rad)
        lows = []
        for idx, _ in self.grip_drive:
            try:
                z = float(p.getAABB(self.body_id, idx,
                                    physicsClientId=self.physics_client)[0][2])
            except Exception:
                continue
            if finger_error_m and idx in (self.tcp_link_r, self.tcp_link_l):
                z += float(finger_error_m)
            lows.append(z)
        return float(min(lows)) if lows else float(self._gripper_center()[2])

    def lowest_gripper_link(self, arm_rads: np.ndarray, grip_rad: float):
        """(link_index, z) of whichever gripper link is lowest. Diagnostics only.

        Which link is lowest decides whether the known +16.7 mm URDF finger error
        applies to a clearance number: it does if the finger pads (tcp_link_r/l) are
        lowest, and does not if some other linkage part is. Run 6 stalled with the
        guard reporting 7.4 mm while the operator measured about 15 mm of real
        clearance, and there was no way to tell which case it was.
        """
        self._set_state(arm_rads, grip_rad)
        best, best_z = None, 1e9
        for idx, _ in self.grip_drive:
            try:
                z = float(p.getAABB(self.body_id, idx,
                                    physicsClientId=self.physics_client)[0][2])
            except Exception:
                continue
            if z < best_z:
                best, best_z = idx, z
        if best is None:
            return None, float(self._gripper_center()[2])
        return best, best_z

    def is_pad_link(self, idx) -> bool:
        """True if this link is one of the two finger pads."""
        return idx is not None and idx in (self.tcp_link_r, self.tcp_link_l)

    def sweep_min_z(self, arm_from, grip_from, arm_to, grip_to, samples: int = 8,
                    finger_error_m: float = 0.0) -> float:
        """Lowest gripper point along the straight joint-space path from → to.

        Checking only the endpoint is not enough: the arm can dip below the floor
        partway through a move and come back up, and the servo interpolates through
        that path for real. Returns the minimum over the whole sweep.
        """
        a0 = np.asarray(arm_from, dtype=np.float64).reshape(-1)[:5]
        a1 = np.asarray(arm_to, dtype=np.float64).reshape(-1)[:5]
        worst = 1e9
        for k in range(samples + 1):
            t = k / float(samples)
            worst = min(worst, self.min_gripper_link_z(
                a0 + t * (a1 - a0), grip_from + t * (grip_to - grip_from),
                finger_error_m))
        return float(worst)

    def close_drop(self, arm_rads: np.ndarray, grip_rad: float) -> float:
        """How far gripper_center has already sunk because the jaw is partly closed.

        The hover target is defined in fully-open coordinates, so the controller has to
        add this back or it lifts the arm to "correct" the sink and the pads never reach
        the object. Measured by FK at both grip angles rather than assumed constant.
        """
        self._set_state(arm_rads, grip_rad)
        now_z = float(self._gripper_center()[2])
        self._set_state(arm_rads, dc.GRIPPER_ANGLE_OPEN)
        open_z = float(self._gripper_center()[2])
        self._set_state(arm_rads, grip_rad)   # restore
        return float(max(0.0, open_z - now_z))

    def close(self):
        p.disconnect(self.physics_client)


# ═══════════════════════════════════════════════════════════════════════════
# Object Detection Receiver (TCP socket)
# ═══════════════════════════════════════════════════════════════════════════

# ── Detection payload pose stamp ─────────────────────────────────────────────
# Protocol shared with integration/arm_cam_geometry.py. Deliberately duplicated
# rather than imported: this script has to run standalone on the Jetson from
# grasp/v21/ and must not grow a dependency on integration/. What is duplicated
# here is a wire format — two key names and a tolerance — not calibration
# numbers, and test_deploy_controller.py pins the two definitions together so
# they cannot drift apart silently.
DETECTION_POSE_KEY = "cam_pose"
DETECTION_POSE_NAME_KEY = "cam_pose_name"


class DetectionReceiver:
    """Listen for JSON object-position messages on a TCP socket.

    Expected JSON format:
        {"x": 0.24, "y": 0.00, "z": 0.02, "height": 0.013,
         "cam_pose": [90.0, 74.2, 8.6, 8.6, 90.0, 30.0],
         "cam_pose_name": "v23_e1_grasp_home"}

    ``z`` is the object centroid height; ``height`` is its full z extent (top minus
    floor). They are different quantities — ``height`` cannot be derived from ``z``
    without assuming the object is symmetric and floor-resting, so the 32D contract
    treats a missing ``height`` as a hard error rather than guessing.

    ``cam_pose`` is the arm pose the SENDER's camera geometry assumes. The arm
    camera rides on arm_link4, so the sender's height/pitch model is valid at one
    pose only — but the sender cannot read the servos to check, and this process
    cannot see how the coordinates were computed. The stamp is where the two
    facts meet: _latch_object() compares it against the encoders at the moment it
    freezes a detection. Optional, for compatibility with senders that predate
    it; when absent, --i-confirm-external-frame is the only thing standing behind
    the frame.

    The sender (camera node) connects, sends the JSON, and can disconnect.
    The latest received detection is kept until it goes stale, after which the
    receiver falls back to ``default_pos`` and reports height as unknown rather
    than serving a position the camera stopped confirming.
    """

    MAX_MESSAGE_BYTES = 4096

    def __init__(self, host: str, port: int, default_pos: Tuple[float, float, float],
                 default_height: Optional[float] = None,
                 stale_timeout_sec: float = 1.0):
        self._default_pos = np.array(default_pos, dtype=np.float32)
        self._default_height = default_height
        self._pos = self._default_pos.copy()
        self._height = default_height
        self._pose_stamp: Optional[Tuple[Tuple[float, ...], Optional[str]]] = None
        self._stale_timeout_sec = float(max(0.0, stale_timeout_sec))
        self._last_update_ts = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind((host, port))
            self._server.listen(5)
            self._server.settimeout(1.0)
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        except Exception:
            self._server.close()
            raise
        print(f"[DetectionReceiver] Listening on {host}:{port} | default={list(default_pos)}")

    def _loop(self):
        while not self._stop.is_set():
            conn = None
            try:
                conn, addr = self._server.accept()
                conn.settimeout(2.0)
                data = b""
                while True:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    if len(data) + len(chunk) > self.MAX_MESSAGE_BYTES:
                        raise ValueError(
                            f"detection message exceeds {self.MAX_MESSAGE_BYTES} bytes")
                    data += chunk
                pos, height, pose_stamp = self._parse_payload(data)
                with self._lock:
                    self._pos = pos
                    self._height = height
                    self._pose_stamp = pose_stamp
                    self._last_update_ts = time.monotonic()
                stamp_note = ""
                if pose_stamp is not None:
                    name = pose_stamp[1] or "unnamed"
                    stamp_note = f" cam_pose={name}"
                print(f"[DetectionReceiver] New detection from {addr}: "
                      f"pos={pos.tolist()} height={height}{stamp_note}")
            except socket.timeout:
                continue
            except OSError as e:
                if self._stop.is_set():
                    break
                print(f"[DetectionReceiver] Socket error: {e}")
            except Exception as e:
                print(f"[DetectionReceiver] Parse error: {e}")
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except OSError:
                        pass

    @staticmethod
    def _parse_pose_stamp(msg: dict) -> Optional[Tuple[Tuple[float, ...], Optional[str]]]:
        """Read the sender's ``cam_pose`` stamp, or None when it did not send one.

        A stamp that is PRESENT but malformed raises rather than reading as
        absent: otherwise a sender bug quietly downgrades a hard cross-process
        check into a skipped one, which is the failure this stamp exists to
        prevent.
        """
        if DETECTION_POSE_KEY not in msg:
            return None
        raw = msg[DETECTION_POSE_KEY]
        if not isinstance(raw, (list, tuple)):
            raise ValueError(f"{DETECTION_POSE_KEY} must be a list of joint degrees")
        if not 5 <= len(raw) <= 6:
            raise ValueError(
                f"{DETECTION_POSE_KEY} must have 5 or 6 entries, got {len(raw)}")
        try:
            deg = tuple(float(v) for v in raw)
        except (TypeError, ValueError):
            raise ValueError(f"{DETECTION_POSE_KEY} entries must be numbers") from None
        if not all(math.isfinite(v) for v in deg):
            raise ValueError(f"{DETECTION_POSE_KEY} entries must be finite")
        name = msg.get(DETECTION_POSE_NAME_KEY)
        return deg, (str(name) if name is not None else None)

    @staticmethod
    def _parse_payload(
        data: bytes,
    ) -> Tuple[np.ndarray, Optional[float],
               Optional[Tuple[Tuple[float, ...], Optional[str]]]]:
        """Validate one JSON detection message. Raises ValueError on anything unusable.

        Every field is checked before it can reach the policy. A NaN that slips
        through here becomes a NaN target position, and the arm glides at a
        coordinate that compares false against every bound -- so this fails loudly
        instead of passing the value on.

        Returns (position, height, pose_stamp).
        """
        if not data:
            raise ValueError("empty detection message")
        msg = json.loads(data.decode("utf-8"))
        if not isinstance(msg, dict):
            raise ValueError("detection payload must be a JSON object")
        pos = np.array(
            [float(msg["x"]), float(msg["y"]), float(msg.get("z", 0.02))],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(pos)):
            raise ValueError(f"detection position must be finite, got {pos.tolist()}")
        # Absent height stays None — the contract then fails closed (or falls back to
        # the documented symmetric-object estimate) rather than grasping at a hover
        # height computed from a made-up number.
        height_raw = msg.get("height", None)
        height = float(height_raw) if height_raw is not None else None
        if height is not None and (not math.isfinite(height) or height <= 0.0):
            raise ValueError(
                f"detection height must be finite and positive, got {height}")
        return pos, height, DetectionReceiver._parse_pose_stamp(msg)

    def _is_stale(self) -> bool:
        return (self._stale_timeout_sec > 0.0
                and self._last_update_ts > 0.0
                and (time.monotonic() - self._last_update_ts) > self._stale_timeout_sec)

    def get(self) -> np.ndarray:
        with self._lock:
            if self._is_stale():
                return self._default_pos.copy()
            return self._pos.copy()

    def get_height(self) -> Optional[float]:
        with self._lock:
            if self._is_stale():
                return self._default_height
            return self._height

    def snapshot(self) -> Tuple[np.ndarray, Optional[float], bool,
                                Optional[Tuple[Tuple[float, ...], Optional[str]]]]:
        """Atomically return (pos, height, is_fresh, pose_stamp).

        is_fresh is True only when a real detection has been received and has not
        gone stale. The latch logic needs this to tell "the camera told us the
        object is here" apart from "nobody has said anything and this is just the
        default" -- the two are indistinguishable from get() alone.

        The pose stamp comes out of the same locked read as the position, so the
        stamp the latch verifies always belongs to the detection it froze; a
        separate accessor could hand back a stamp from the next message.
        """
        with self._lock:
            fresh = self._last_update_ts > 0.0 and not self._is_stale()
            if fresh:
                return self._pos.copy(), self._height, True, self._pose_stamp
            return self._default_pos.copy(), self._default_height, False, None

    def close(self):
        self._stop.set()
        try:
            self._server.close()
        except OSError:
            pass
        thread = getattr(self, "_thread", None)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)
            if thread.is_alive():
                print("[DetectionReceiver][WARN] listener did not stop within 3.0s")


# ═══════════════════════════════════════════════════════════════════════════
# Observation Builder
# ═══════════════════════════════════════════════════════════════════════════

class FloorGuard:
    """Preventive floor protection for the real robot.

    The simulation guard restores a safe state after a violating step. That is not
    available on hardware — you cannot teleport a robot that has already driven its
    gripper into the floor. So the only sound form here is *prevention*: before a
    command is sent, sweep the joint-space path from the current pose to the target,
    and if any point along it puts a gripper link below the safety line, shorten the
    motion to the largest safe fraction. If nothing is safe, hold position.

    This layer is the last thing between the policy and the servos, and it has no net
    beneath it. It is deliberately more conservative than the sim guard.
    """

    def __init__(self, cfg: DeployConfig, fk: FKComputer):
        self.cfg = cfg
        self.fk = fk
        self.interventions = 0
        self.refusals = 0

    @property
    def line(self) -> float:
        return float(self.cfg.ground_height + self.cfg.floor_safety_margin)

    def project(self, arm_now, grip_now, arm_target, grip_target):
        """Return (safe_arm_target, safe_grip_target, info).

        Never raises: a guard that throws mid-motion leaves the arm wherever it was.
        """
        if not self.cfg.floor_guard_enable:
            return arm_target, grip_target, {"action": "disabled"}

        a_now = np.asarray(arm_now, dtype=np.float64).reshape(-1)[:5]
        a_tgt = np.asarray(arm_target, dtype=np.float64).reshape(-1)[:5]

        err = float(self.cfg.floor_finger_error_m)
        z_now = self.fk.min_gripper_link_z(a_now, grip_now, err)
        if z_now < self.line:
            # Already too low — the only safe move is up. Which joint direction is
            # "up" is pose-dependent: a fixed S2 -= 0.05 was wrong here, and the same
            # hard-coded sign was already caught once in the scripted lift. Search the
            # candidates with FK and take the one that genuinely gains height.
            self.refusals += 1
            step = float(self.cfg.emergency_raise_rad)
            best, best_z = None, z_now
            for joint in (1, 2, 3):          # S2/S3/S4 carry the vertical motion
                for sign in (+1.0, -1.0):
                    cand = a_now.copy()
                    cand[joint] = float(np.clip(cand[joint] + sign * step,
                                                *self.cfg.arm_sim_limits[joint]))
                    z = self.fk.min_gripper_link_z(cand, grip_now, err)
                    if z > best_z:
                        best, best_z = cand, z
            # Which link is lowest decides whether the measured +16.7 mm URDF finger
            # error applies to this number. Run 6 reported z_now 7.4 mm while the
            # operator's ruler said about 15 mm, and the log could not say why.
            low_idx, _ = self.fk.lowest_gripper_link(a_now, grip_now)
            low = {"low_link": low_idx, "low_is_pad": self.fk.is_pad_link(low_idx)}
            if best is None:
                # Nothing gains height from here. Hold still rather than guess — an
                # arbitrary move while already through the floor can only make it worse.
                return (a_now.astype(np.float32), grip_now,
                        dict(action="emergency_hold", z_now=z_now, z_after=z_now, **low))
            return (best.astype(np.float32), grip_now,
                    dict(action="emergency_raise", z_now=z_now, z_after=best_z, **low))

        z_sweep = self.fk.sweep_min_z(a_now, grip_now, a_tgt, grip_target,
                                      self.cfg.floor_sweep_samples, err)
        if z_sweep >= self.line:
            return arm_target, grip_target, {"action": "pass", "z_sweep": z_sweep}

        # Largest fraction of the requested motion that keeps the whole sweep safe.
        lo, hi = 0.0, 1.0
        best_a, best_g = a_now.copy(), grip_now
        for _ in range(self.cfg.floor_bisect_iters):
            mid = 0.5 * (lo + hi)
            cand_a = a_now + mid * (a_tgt - a_now)
            cand_g = grip_now + mid * (grip_target - grip_now)
            if self.fk.sweep_min_z(a_now, grip_now, cand_a, cand_g,
                                   self.cfg.floor_sweep_samples, err) >= self.line:
                best_a, best_g, lo = cand_a, cand_g, mid
            else:
                hi = mid
        self.interventions += 1
        return (best_a.astype(np.float32), float(best_g),
                {"action": "clamped", "fraction": lo, "z_sweep": z_sweep})


class ObsBuilder:
    """Build the observation for the active contract, matching robot_grasp_env.py.

    The layout is owned by ``deploy_contract`` so training-side eval and the robot
    cannot drift apart; see ``DeployContract.slices`` for the authoritative table.
    ``tcp_pos`` is gripper_center (from FKComputer), not arm_link5.
    """

    def __init__(self, fk: FKComputer, mapper: JointMapper,
                 vec_normalize: Optional[VecNormalize],
                 contract: dc.DeployContract,
                 tcp_forward_error_m: float = 0.0):
        self.fk = fk
        self.mapper = mapper
        self.vec_normalize = vec_normalize
        self.contract = contract
        self.tcp_forward_error_m = float(tcp_forward_error_m)

    def build(
        self,
        arm_sim_rads: np.ndarray,
        grip_sim_rad: float,
        obj_pos: np.ndarray,
        stage: int,
        prev_action: np.ndarray,
        obj_height: Optional[float] = None,
        wrist_z_offset: Optional[float] = None,
    ) -> np.ndarray:
        tcp_pos, tcp_quat = self.fk.compute(arm_sim_rads, grip_sim_rad)
        tcp_pos = control_tcp_position(tcp_pos, self.tcp_forward_error_m)

        # Only measured when the contract needs it — it costs two extra FK queries.
        close_drop = (self.fk.close_drop(arm_sim_rads, grip_sim_rad)
                      if self.contract.requires_object_height else 0.0)

        obs = dc.build_observation(
            self.contract,
            arm_rads=arm_sim_rads,
            grip_rad=grip_sim_rad,
            tcp_pos=tcp_pos,
            tcp_quat=tcp_quat,
            object_pos=obj_pos,
            stage=stage,
            prev_action=prev_action,
            object_height=obj_height,
            close_drop=close_drop,
            wrist_z_offset=wrist_z_offset,
        )

        if self.vec_normalize is not None and self.vec_normalize.norm_obs:
            obs = self.vec_normalize.normalize_obs(obs.reshape(1, -1)).reshape(-1)

        return obs.astype(np.float32)


# ═══════════════════════════════════════════════════════════════════════════
# Mock environment for VecNormalize loading (no PyBullet needed)
# ═══════════════════════════════════════════════════════════════════════════

def _make_mock_env(obs_dim: int):
    """VecNormalize.load needs an env with the right spaces, nothing more."""

    class _MockEnv(gym.Env):
        observation_space = gym.spaces.Box(low=-np.inf, high=np.inf,
                                           shape=(obs_dim,), dtype=np.float32)
        action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)

        def step(self, a):
            return np.zeros(obs_dim, dtype=np.float32), 0.0, False, False, {}

        def reset(self, **kw):
            return np.zeros(obs_dim, dtype=np.float32), {}

    return _MockEnv


def _resolve_asset(path: str, script_dir: Path) -> Path:
    """Resolve a weights/vecnorm path the same way for the release gate and the loader.

    Two resolution rules would be worse than none: the gate would hash one file
    while the controller loaded another, and the mismatch it exists to catch would
    sail straight through. One implementation, both callers.
    """
    p_abs = Path(path)
    if not p_abs.is_absolute():
        cand = script_dir / path
        if not cand.exists():
            # repo 佈局：權重目錄在 training/ 的上一層（Jetson 佈局＝腳本旁，優先）。
            cand = script_dir.parent / path
        p_abs = cand
    if not p_abs.exists():
        raise FileNotFoundError(f"File not found: {p_abs}")
    return p_abs


# ═══════════════════════════════════════════════════════════════════════════
# Main Controller
# ═══════════════════════════════════════════════════════════════════════════

class GraspController:
    def __init__(self, cfg: DeployConfig, real_servo: bool, use_socket: bool,
                 obj_provider: Optional[Callable[[], Tuple[np.ndarray, Optional[float]]]] = None):
        self.cfg = cfg
        self.script_dir = Path(__file__).resolve().parent
        # External object source, checked before --socket/--obj-x-y-z. Matches
        # grasp/x3plus_real_grasp.py's shape exactly: a callable returning
        # (position_xyz, reported_height_m_or_None). Settable here or later —
        # a vision pipeline that latches a detection AFTER construction (so it can
        # navigate first) assigns controller.obj_provider = lambda: (pos, height)
        # right before calling run(). The second element is the object's full
        # vertical extent for the episode's wrist_z_offset/engagement-depth
        # geometry, NOT bounding-box width — feeding pixel width through here would
        # corrupt that geometry. Pass None if no height measurement exists; the
        # contract's usual symmetric-object fallback (2*(centroid_z-ground_z))
        # applies exactly as it does for a bare --obj-x/y/z run.
        self.obj_provider = obj_provider

        # ── Contract ──────────────────────────────────────────────────────
        self.contract = dc.get_contract(cfg.contract_name)
        print(f"[Init] Contract: {self.contract.name} "
              f"(obs {self.contract.obs_dim}D, arm action {self.contract.arm_action_mode})")

        # ── Load model ────────────────────────────────────────────────────
        model_path = self._abs(cfg.model_path)
        vecnorm_path = self._abs(cfg.vecnorm_path)
        print(f"[Init] Loading model: {model_path}")
        self.model = PPO.load(str(model_path), device="cpu")

        # Check the policy against the contract BEFORE touching VecNormalize. A
        # mismatched pair otherwise surfaces as "VecNormalize failed to load (spaces
        # must have the same shape)", which sends you looking at the wrong file.
        dc.validate_model(self.contract, self.model, model_path=str(model_path))

        print(f"[Init] Loading VecNormalize: {vecnorm_path}")
        mock_venv = DummyVecEnv([_make_mock_env(self.contract.obs_dim)])
        # A failed VecNormalize load is fatal: the policy was trained on normalised
        # observations, so feeding it raw ones produces confident nonsense. The old
        # warn-and-continue path silently did exactly that.
        try:
            loaded = VecNormalize.load(str(vecnorm_path), mock_venv)
        except Exception as e:
            raise ContractMismatchError(
                f"{vecnorm_path}: VecNormalize failed to load ({e}). Refusing to run "
                "unnormalised — the policy expects normalised observations."
            ) from e
        vec_norm = VecNormalize(mock_venv, norm_obs=True, norm_reward=False)
        vec_norm.obs_rms = loaded.obs_rms
        vec_norm.training = False
        vec_norm.norm_reward = False

        # Fail closed on any model / VecNormalize / contract disagreement.
        dc.validate_model(
            self.contract, self.model,
            vecnorm_obs_rms=vec_norm.obs_rms,
            model_path=str(model_path), vecnorm_path=str(vecnorm_path),
        )

        # ── FK computer ───────────────────────────────────────────────────
        self.fk = FKComputer(cfg.urdf_path)

        # ── Mapper, obs builder, floor guard ──────────────────────────────
        self.mapper = JointMapper(cfg)
        self.obs_builder = ObsBuilder(
            self.fk, self.mapper, vec_norm, self.contract,
            tcp_forward_error_m=cfg.tcp_forward_error_m)
        self.floor_guard = FloorGuard(cfg, self.fk)

        # ── Servo controller ──────────────────────────────────────────────
        self.servo = ServoController(cfg, dry_run=not real_servo)
        # Store the flag, don't just consume the parameter. The fail-closed paths
        # in _latch_object() and _verify_detection_pose() branch on
        # self.real_servo; without this line they raise AttributeError instead of
        # the refusal they were written to produce -- and they only run when
        # something has ALREADY gone wrong, so the substitution stays invisible in
        # every healthy run. Found by integration/smoke_mode_b.py.
        self.real_servo = bool(real_servo)

        # ── Object detection ──────────────────────────────────────────────
        if use_socket:
            self.detection = DetectionReceiver(
                cfg.socket_host, cfg.socket_port,
                cfg.default_obj_pos, cfg.default_obj_height,
                stale_timeout_sec=cfg.detection_stale_timeout_sec)
        else:
            self.detection = None
        # Kept unconditionally: _latch_object() falls back to the fixed object when
        # there is no detection source, and the socket path still needs a defined
        # fallback if a latch times out in dry-run.
        self._fixed_obj = np.array(cfg.default_obj_pos, dtype=np.float32)
        self._fixed_height = cfg.default_obj_height

        if self.contract.requires_object_height and cfg.default_obj_height is None and not use_socket:
            raise MissingObjectHeightError(
                f"contract {self.contract.name!r} needs an object height and none was "
                f"given (--object-height). " + dc.DETECTION_CONTRACT_REQUIRED
            )

        # ── State ─────────────────────────────────────────────────────────
        self._stage = 0
        self._outcome = "not_run"     # confirmed | unverified | dropped | rejected | ...
        self._wrist_z_offset = None   # frozen at first detection; see run()
        self._episode_object_height = None
        self._object_height_source = None
        self._obj_latched = False
        self._latched_obj: Optional[np.ndarray] = None
        self._latched_height: Optional[float] = None
        self._prev_action = np.zeros(6, dtype=np.float32)  # raw, clipped policy action
        self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(list(cfg.home_deg[:5]))
        self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(cfg.home_deg[5])
        # Sim-rad jaw command held while an object is gripped (contact + bias).
        # None whenever nothing is held; stage 2 falls back to the readback then.
        self._grip_hold_rad: Optional[float] = None
        # Consecutive stage-0 steps spent just outside the entry radius (fallback).
        self._hover_count = 0
        # Guard-deadlock detector: consecutive intervening steps, and the best
        # clearance seen so far. See cfg.guard_deadlock_steps.
        self._guard_deadlock_count = 0
        self._guard_deadlock_z: Optional[float] = None
        # Floor-guard actions over the episode's policy steps — the evidence for
        # the guard_margin_8mm hardware gate, printed on every exit path.
        self._guard_tally: dict = {}
        self._action_execution = ActionExecutionState()
        self._action_execution.reset(self._current_grip_rad)

        print("[Init] All systems ready.")

    def _abs(self, path: str) -> Path:
        return _resolve_asset(path, self.script_dir)

    def _latch_object(self):
        """Capture object position + height ONCE at the home pose and freeze them.

        Solves the moving-arm-camera problem: the arm camera rides on arm_link4, so
        its fixed-height/fixed-pitch distance model is only valid before the arm
        moves. We read a single detection here, at the navigation/home pose, and
        reuse it for the whole episode.
        """
        if self.detection is None:
            # No detection source — the fixed object is already static.
            self._latched_obj = self._fixed_obj.copy()
            self._latched_height = self._fixed_height
            self._obj_latched = True
            print(f"[Latch] No detection source; using fixed object "
                  f"{self._latched_obj.tolist()}")
            return

        deadline = time.time() + self.cfg.latch_wait_sec
        pos, height, fresh, stamp = self.detection.snapshot()
        while not fresh and time.time() < deadline:
            time.sleep(0.05)
            pos, height, fresh, stamp = self.detection.snapshot()

        if not fresh and self.real_servo:
            # Never glide the real arm toward a position nobody confirmed. The
            # default is a placeholder, not a detection.
            raise RuntimeError(
                "no fresh socket detection arrived before latch timeout "
                f"({self.cfg.latch_wait_sec}s); refusing to move the real arm "
                "toward the default position")

        if fresh:
            self._verify_detection_pose(stamp)

        self._latched_obj = pos
        self._latched_height = height
        self._obj_latched = True
        if fresh:
            print(f"[Latch] Object latched at home pose: pos={pos.tolist()}"
                  + (f" height={height:.3f}m" if height is not None else " (no height)"))
        else:
            print(f"[Latch] WARNING: no detection within {self.cfg.latch_wait_sec}s; "
                  f"falling back to {pos.tolist()}")

    def _verify_detection_pose(self, stamp):
        """Check the sender's camera pose against the arm's actual encoders.

        The arm camera rides on arm_link4, so the sender's distance model belongs
        to one arm pose. The sender declares which pose in the payload; only this
        process can read where the arm really is. This is the single moment both
        facts exist together, so it is the only place the mismatch is catchable —
        and nothing downstream can catch it afterwards, because a wrong extrinsic
        yields a perfectly plausible coordinate and the arm then grasps
        confidently in the wrong place.

        Raises RuntimeError under real servos; warns in dry-run so the socket
        path stays testable on a desk.
        """
        if stamp is None:
            print("[Latch] NOTE: the detection carries no cam_pose stamp, so the "
                  "arm pose its geometry assumes cannot be checked against the "
                  "encoders. --i-confirm-external-frame is the only thing "
                  "standing behind the coordinate frame.")
            return

        stamp_deg, stamp_name = stamp
        rd = self.servo.read_degrees()
        if not rd.valid:
            msg = (f"cannot read the encoders to verify the detection's cam_pose "
                   f"stamp ({rd.reason})")
            if self.real_servo:
                raise RuntimeError(msg + "; refusing to latch")
            print(f"[Latch] WARN: {msg} (dry-run, continuing)")
            return

        actual = list(rd.degrees[:5])
        tol = self.cfg.detection_pose_tol_deg
        worst_i, worst = -1, 0.0
        for i, (a, b) in enumerate(zip(stamp_deg[:5], actual)):
            delta = abs(float(a) - float(b))
            if delta > worst:
                worst_i, worst = i, delta

        if worst <= tol:
            print(f"[Latch] cam_pose stamp {stamp_name or 'unnamed'} agrees with the "
                  f"encoders (worst joint off by {worst:.2f} deg, tol {tol} deg)")
            return

        msg = (
            f"detection cam_pose does not match the arm.\n"
            f"    sender computed for : {[round(float(v), 2) for v in stamp_deg[:5]]}"
            f"  ({stamp_name or 'unnamed'})\n"
            f"    encoders report     : {[round(v, 2) for v in actual]}\n"
            f"    worst joint S{worst_i + 1} differs by {worst:.2f} deg "
            f"(tolerance {tol} deg)\n"
            "    The arm camera is mounted on arm_link4, so the sender's height and\n"
            "    pitch model is valid at its declared pose only. Either move the arm\n"
            "    to that pose before detecting, or re-measure the sender's extrinsics\n"
            "    at the pose this arm actually starts from."
        )
        if self.real_servo:
            raise RuntimeError("[Latch] REFUSED: " + msg)
        print("[Latch] WARN: " + msg)

    def _check_target_envelope(self, obj_pos) -> bool:
        """Refuse a target outside the region the policy was evaluated in.

        Called once per episode, after the target is fixed and while the arm is
        still parked at home, so a refusal costs nothing.

        The arm camera at C3 sees ground from about x=0.185 to x=0.318, which
        overhangs the trained region at both ends -- so a detector doing its job
        perfectly can still hand over a target the policy has never been measured
        on. Nothing further down catches it: the coordinate is finite, in frame
        and geometrically consistent. The arm simply drives somewhere it was not
        trained to go, and whatever happens next gets attributed to the grasp.
        """
        x, y = float(obj_pos[0]), float(obj_pos[1])
        tx, ty = self.cfg.trained_x_range, self.cfg.trained_y_range
        dx, dy = self.cfg.documented_x_range, self.cfg.documented_y_range
        # Detections arrive as float32, so a target typed or computed as exactly
        # the boundary lands a fraction outside it (float32(0.33) is 0.33000001).
        # A micrometre past the edge is not the failure this guard is for, and
        # refusing it would look like a bug rather than a safeguard.
        tol = self.cfg.envelope_tol_m

        outside = []
        if not tx[0] - tol <= x <= tx[1] + tol:
            outside.append(f"x={x:.4f} outside the evaluated {tx[0]}-{tx[1]}")
        if not ty[0] - tol <= y <= ty[1] + tol:
            outside.append(f"y={y:+.4f} outside the evaluated {ty[0]}-{ty[1]}")
        if outside:
            msg = (
                "target is outside the region this policy was evaluated in.\n"
                f"    {'; '.join(outside)}\n"
                "    The formal acceptance run covered x(0.20, 0.33) y(+-0.10) only, so\n"
                "    there is no measured success rate here -- not a low one, none.\n"
                "    The arm camera at C3 sees x 0.200-0.318, so a detection near the\n"
                "    bottom of the frame can land outside this range while looking\n"
                "    perfectly healthy: move the object into range, or move the robot."
            )
            if self.real_servo:
                print("[SAFETY] REFUSED: " + msg)
                return False
            print("[SAFETY] WARN (dry-run): " + msg)
            return True

        soft = []
        if not dx[0] - tol <= x <= dx[1] + tol:
            soft.append(f"x={x:.4f} beyond the documented {dx[0]}-{dx[1]}")
        if not dy[0] - tol <= y <= dy[1] + tol:
            soft.append(f"y={y:+.4f} beyond the documented {dy[0]}-{dy[1]}")
        if soft:
            print("[SAFETY] NOTE: " + "; ".join(soft) +
                  ". Inside the evaluated region but outside what manifest.json "
                  "calls valid; expect the approach to be less reliable here.")
        return True

    def _get_obj_pos(self) -> np.ndarray:
        if self.obj_provider is not None:
            return np.asarray(self.obj_provider()[0], dtype=np.float32).reshape(-1)
        if self.cfg.latch_obj and self._obj_latched and self._latched_obj is not None:
            return self._latched_obj.copy()
        if self.detection is not None:
            return self.detection.get()
        return self._fixed_obj.copy()

    def _get_obj_height(self, obj_pos) -> float:
        """Resolve and freeze the height used by target/gate geometry this episode."""
        if self._episode_object_height is not None:
            return float(self._episode_object_height)

        if self.obj_provider is not None:
            reported = self.obj_provider()[1]
        elif self.cfg.latch_obj and self._obj_latched:
            reported = self._latched_height
        elif self.detection is not None:
            reported = self.detection.get_height()
        else:
            reported = self._fixed_height
        height, source = resolve_object_height_for_contract(
            self.contract, obj_pos, reported, self.cfg.ground_height)
        self._episode_object_height = float(height)
        self._object_height_source = source
        if source == "resting_centroid_geometry":
            print("[Init] Object height absent under 28D contract; using explicit "
                  "resting symmetric-object geometry: "
                  f"height=2*(centroid_z-ground_z)={height:.4f} m")
        return float(height)

    def move_guarded_and_verified(
        self,
        arm_target_rad,
        grip_target_rad: float,
        *,
        label: str,
        run_time_ms: int = 300,
        settle_s: float = 0.35,
        max_iters: int = 40,
        tol_deg: float = 2.0,
        stop_on_jaw_contact: bool = False,
        grip_is_hold: bool = False,
    ) -> dict:
        """The one way this controller is allowed to move the arm.

        ``stop_on_jaw_contact`` (the stage-1 close): watch for the jaw command
        advancing while the encoder does not — that is an object, and the correct
        response is to park the command a small bias past the contact angle, not to
        keep walking it to the closed stop. The servo grips by holding a position
        error; walking to the stop leaves it at near-stall torque for the rest of
        the run (audible as gear grinding on hardware, 2026-07-31).

        ``grip_is_hold`` (stage-2 moves with an object gripped): the jaw target is a
        deliberate squeeze past the physical contact angle, so the encoder can never
        reach it. Arrival is then judged on the arm joints alone — otherwise every
        lift step would wrongly report a stall on a jaw that is doing its job.

        Every other path — home, stage-1 close, the scripted lift, the return, the
        failure retreat — goes through here. Three things it guarantees that a bare
        ``send_degrees`` call does not:

        1. **It actually arrives.** ``send_degrees`` rate-limits to ``max_delta_deg``
           per call, so one call moves at most 8 deg. Callers used to issue a single
           command for a 40 deg move and then set their internal pose to the target,
           after which every subsequent FK, guard check and observation was computed
           for a pose the arm was nowhere near.
        2. **Every intermediate command is guarded.** The floor guard is applied per
           iteration, against the pose actually read back.
        3. **Internal state follows the encoders, not the command.** If a read fails,
           the move aborts and the state is left untouched.

        Returns a dict with ``reached`` (bool), ``reason``, ``iters``, and the guard
        actions seen. Never raises for hardware trouble — it reports and stops.
        """
        target = np.asarray(arm_target_rad, dtype=np.float64).reshape(-1)[:5]
        guard_actions = []
        last_err = None
        prev_jaw = None
        prev_jaw_cmd = None
        contact_streak = 0

        for it in range(max_iters):
            cur_arm = np.asarray(self._current_arm_rads, dtype=np.float64).reshape(-1)[:5]
            cur_grip = float(self._current_grip_rad)

            safe_arm, safe_grip, ginfo = self.floor_guard.project(
                cur_arm, cur_grip, target, grip_target_rad)
            guard_actions.append(ginfo["action"])

            if ginfo["action"] == "emergency_hold":
                # Nothing gains height from here; holding is the safe action, and it
                # is already what the arm is doing. Report and stop.
                return {"reached": False, "reason": "floor guard emergency_hold",
                        "iters": it, "guard": guard_actions, "guard_info": ginfo}

            if ginfo["action"] == "emergency_raise":
                # The raise is a real command and must actually be SENT — returning
                # here without sending left the arm sitting below the floor line while
                # the log claimed an emergency raise had happened.
                hw = self.mapper.sim_arm_to_hw_deg(safe_arm)
                hw.append(self.mapper.sim_grip_to_hw_deg(safe_grip))
                if not self.servo.send_degrees(hw, run_time_ms=run_time_ms):
                    return {"reached": False, "reason": "servo write failed during "
                                                        "emergency raise",
                            "iters": it, "guard": guard_actions}
                time.sleep(settle_s)
                rd = self.servo.read_degrees()
                if not rd.valid:
                    return {"reached": False,
                            "reason": f"servo read failed during emergency raise: {rd.reason}",
                            "iters": it, "guard": guard_actions}
                self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
                self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])
                # Keep raising until clear of the line, but never silently forever.
                if it + 1 >= self.cfg.emergency_raise_max_iters:
                    return {"reached": False,
                            "reason": "emergency raise exhausted its attempts",
                            "iters": it + 1, "guard": guard_actions, "guard_info": ginfo}
                continue

            hw = self.mapper.sim_arm_to_hw_deg(safe_arm)
            hw.append(self.mapper.sim_grip_to_hw_deg(safe_grip))
            if stop_on_jaw_contact and prev_jaw is not None:
                # Force ceiling. Never ask the jaw to be more than
                # jaw_max_lag_deg past where it actually is; torque is that error.
                limit = float(self.cfg.jaw_max_lag_deg)
                if limit > 0.0:
                    open_ = float(self.cfg.gripper_hw_open)
                    closed = float(self.cfg.gripper_hw_closed)
                    sign = 1.0 if closed >= open_ else -1.0
                    ceiling = prev_jaw + sign * limit
                    if sign > 0.0 and hw[5] > ceiling:
                        hw[5] = ceiling
                    elif sign < 0.0 and hw[5] < ceiling:
                        hw[5] = ceiling
            if not self.servo.send_degrees(hw, run_time_ms=run_time_ms):
                return {"reached": False, "reason": "servo write failed",
                        "iters": it, "guard": guard_actions}

            time.sleep(settle_s)

            rd = self.servo.read_degrees()
            if not rd.valid:
                # Fail closed: we no longer know where the arm is, so we must not
                # update our idea of it and must not keep commanding.
                return {"reached": False, "reason": f"servo read failed: {rd.reason}",
                        "iters": it, "guard": guard_actions}

            self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
            self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])

            want_hw = self.mapper.sim_arm_to_hw_deg(target)
            want_hw.append(self.mapper.sim_grip_to_hw_deg(grip_target_rad))
            # With an object held, the jaw target is a deliberate squeeze the encoder
            # can never satisfy — judge arrival on the arm joints only.
            n_err = 5 if grip_is_hold else 6
            err = max(abs(float(a) - float(b))
                      for a, b in zip(rd.degrees[:n_err], want_hw[:n_err]))
            if err <= tol_deg:
                return {"reached": True, "reason": "arrived", "iters": it + 1,
                        "guard": guard_actions, "residual_deg": err}

            if stop_on_jaw_contact:
                jaw_rd = float(rd.degrees[5])
                # The command actually on the wire is the rate-limited one the servo
                # layer tracks, not the far-off `want_hw` target.
                jaw_cmd = float(self.servo._last_deg[5])
                if prev_jaw is not None and ginfo["action"] == "pass":
                    enc_step = abs(jaw_rd - prev_jaw)
                    tracking = enc_step >= self.cfg.jaw_contact_min_progress_deg
                    lagging = abs(jaw_cmd - jaw_rd) >= self.cfg.jaw_contact_lag_deg
                    # Slipping: the jaw IS moving, but far slower than commanded.
                    # Only meaningful once the command asked for a real step.
                    fraction = float(self.cfg.jaw_contact_track_fraction)
                    cmd_step = (abs(jaw_cmd - prev_jaw_cmd)
                                if prev_jaw_cmd is not None else 0.0)
                    slipping = (
                        fraction > 0.0
                        and cmd_step >= self.cfg.jaw_contact_min_cmd_step_deg
                        and enc_step < fraction * cmd_step)
                    blocked = lagging and (not tracking or slipping)
                    contact_streak = contact_streak + 1 if blocked else 0
                    if contact_streak >= max(1, int(self.cfg.jaw_contact_streak)):
                        hold = self._park_jaw_hold(jaw_rd, grip_target_rad, run_time_ms)
                        return {"reached": False, "jaw_contact": True,
                                "jaw_contact_deg": jaw_rd, "jaw_hold_deg": hold,
                                "jaw_contact_mode": ("slipping" if slipping
                                                     else "stalled"),
                                "reason": (
                                    f"{'slower than commanded' if slipping else 'no progress'}"
                                    f" — jaw contact at {jaw_rd:.1f} deg, "
                                    f"holding at {hold:.1f} deg"),
                                "iters": it + 1, "guard": guard_actions}
                else:
                    contact_streak = 0
                prev_jaw = jaw_rd
                prev_jaw_cmd = jaw_cmd

            # If the guard is clamping, the arm is being deliberately held short of the
            # request — that is not a stall, so only treat lack of progress as failure
            # when the guard is passing the command through untouched.
            if (last_err is not None and abs(last_err - err) < 0.05
                    and ginfo["action"] == "pass"):
                if stop_on_jaw_contact:
                    # Same evidence, one iteration sooner than the contact streak can
                    # confirm it — and this branch used to win the race, which is how
                    # run A ended up "holding at 180.0 deg", i.e. parked on the stop
                    # it was built to avoid. A stalled close IS jaw contact.
                    jaw_rd = float(rd.degrees[5])
                    hold = self._park_jaw_hold(jaw_rd, grip_target_rad, run_time_ms)
                    return {"reached": False, "jaw_contact": True,
                            "jaw_contact_deg": jaw_rd, "jaw_hold_deg": hold,
                            # This branch is reached only by a whole-arm stall, so
                            # it is never the slip case -- but it must still say
                            # so, or a caller reading jaw_contact_mode gets None
                            # from the path that fires FIRST on a hard block.
                            "jaw_contact_mode": "stalled",
                            "reason": (f"no progress — jaw contact at {jaw_rd:.1f} deg, "
                                       f"holding at {hold:.1f} deg"),
                            "iters": it + 1, "guard": guard_actions}
                return {"reached": False,
                        "reason": f"no progress (residual {err:.1f} deg) — servo stalled "
                                  f"or target unreachable",
                        "iters": it + 1, "guard": guard_actions, "residual_deg": err}
            last_err = err

        return {"reached": False, "reason": f"did not arrive in {max_iters} iterations",
                "iters": max_iters, "guard": guard_actions, "residual_deg": last_err}

    def _park_jaw_hold(self, contact_deg: float, grip_target_rad: float,
                       run_time_ms: int) -> float:
        """Command the jaw to contact + bias and record the hold for later moves.

        By the time contact is confirmed the walking command is already a streak's
        worth of degrees past the block, so this always moves the command BACK toward
        open — within the range the guard approved on the way down, with the arm
        unchanged, and a less-closed jaw only ever raises the pads. That is why a
        plain send is sound here without another guard projection.

        The hold never exceeds what the close was asking for in the first place, so
        a --width-grip-style partial close target stays honoured.
        """
        open_ = float(self.cfg.gripper_hw_open)
        closed = float(self.cfg.gripper_hw_closed)
        sign = 1.0 if closed >= open_ else -1.0
        hold = contact_deg + sign * float(self.cfg.jaw_hold_bias_deg)
        want = self.mapper.sim_grip_to_hw_deg(grip_target_rad)
        hold = min(hold, want) if sign > 0 else max(hold, want)
        hold = float(np.clip(hold, min(open_, closed), max(open_, closed)))
        hw = [float(v) for v in self.mapper.sim_arm_to_hw_deg(self._current_arm_rads)]
        hw.append(hold)
        if not self.servo.send_degrees(hw, run_time_ms=run_time_ms):
            # The walking command is still active — worse (more squeeze) than the
            # hold, but not unsafe. Record the hold anyway so stage 2 keeps aiming
            # at it rather than re-tightening.
            print("[WARN] Could not park the jaw hold; the last close command stands.")
        self._grip_hold_rad = self.mapper.hw_deg_to_sim_grip(hold)
        return hold

    def _observe_stage0_jaw(self, jaw_deg: float, jaw_cmd_deg: float,
                            eligible: bool) -> dict:
        """Update Stage-0 contact evidence from one measured S6 sample.

        Stage 0 differs from the scripted Stage-1 close: every PPO action is
        incremental and is re-anchored to the latest encoder reading. Comparing
        two successive command *positions* therefore hides a slipping object --
        the next command follows the slipping encoder and both appear to advance
        together. Instead, remember how much of the previous command was still
        outstanding and ask whether the next encoder sample consumed it.

        A stopped jaw keeps the original absolute detector. A moving jaw is
        classified as contact only when the ratio test is explicitly enabled,
        the previous outstanding request was large enough to be meaningful, and
        measured closing progress is below the configured fraction. Evidence must
        remain consecutive; any ineligible or normally tracking sample resets it.
        """
        jaw = float(jaw_deg)
        cmd = float(jaw_cmd_deg)
        result = {
            "blocked": False,
            "mode": None,
            "count": 0,
            "encoder_progress_deg": 0.0,
            "requested_progress_deg": 0.0,
        }

        prev_jaw = self._stage0_s6_prev_deg
        prev_cmd = self._stage0_s6_prev_cmd_deg
        if not eligible:
            self._stage0_s6_prev_deg = None
            self._stage0_s6_prev_cmd_deg = None
            self._stage0_s6_stall_count = 0
            return result

        if prev_jaw is not None and prev_cmd is not None:
            open_ = float(self.cfg.gripper_hw_open)
            closed = float(self.cfg.gripper_hw_closed)
            close_sign = 1.0 if closed >= open_ else -1.0
            signed_encoder = close_sign * (jaw - float(prev_jaw))
            encoder_progress = max(0.0, signed_encoder)
            # This was the position error left immediately after the preceding
            # policy step. The following sample is the first fair opportunity to
            # see whether the servo consumed it.
            requested_progress = max(
                0.0, close_sign * (float(prev_cmd) - float(prev_jaw)))
            unchanged = (abs(jaw - float(prev_jaw))
                         <= self.cfg.stage0_s6_stall_epsilon_deg)
            fraction = float(self.cfg.jaw_contact_track_fraction)
            minimum_request = max(float(self.cfg.jaw_contact_min_cmd_step_deg),
                                  float(self.cfg.jaw_contact_lag_deg))
            slipping = (
                not unchanged
                and fraction > 0.0
                and requested_progress >= minimum_request
                and encoder_progress < fraction * requested_progress)
            blocked = unchanged or slipping
            self._stage0_s6_stall_count = (
                self._stage0_s6_stall_count + 1 if blocked else 0)
            result.update({
                "blocked": blocked,
                "mode": ("stalled" if unchanged else
                         ("slipping" if slipping else None)),
                "count": self._stage0_s6_stall_count,
                "encoder_progress_deg": encoder_progress,
                "requested_progress_deg": requested_progress,
            })

        self._stage0_s6_prev_deg = jaw
        self._stage0_s6_prev_cmd_deg = cmd
        result["count"] = self._stage0_s6_stall_count
        return result

    def _grasp_geometry(self, obj_pos, obj_height: float) -> dict:
        """Geometry shared by action preprocessing and the stage-0 close gate."""
        height = float(obj_height)
        if not np.isfinite(height) or height <= 0.0:
            raise MissingObjectHeightError(
                f"object height must be finite and positive, got {obj_height!r}")
        if self._wrist_z_offset is None:
            raise ContractMismatchError(
                "episode wrist_z_offset must be frozen before action preprocessing")

        arm = np.asarray(self._current_arm_rads, dtype=np.float64)
        grip = float(self._current_grip_rad)
        tcp, _ = self.fk.compute(arm, grip)
        tcp = control_tcp_position(tcp, self.cfg.tcp_forward_error_m)
        close_drop = self.fk.close_drop(arm, grip)
        target = dc.stage0_target(obj_pos, self._wrist_z_offset, close_drop)

        delta = np.asarray(target, dtype=np.float64) - np.asarray(tcp, dtype=np.float64)
        xy_err = float(np.linalg.norm(delta[:2]))
        z_err = float(abs(delta[2]))
        radius = float(np.linalg.norm(delta))
        centred = (xy_err < self.cfg.entry_xy_tol
                   and z_err < self.cfg.entry_z_tol
                   and radius < self.cfg.entry_radius)

        # Same quantity as training _pads_ready_to_close(): actual pad AABB bottom,
        # minus only the close-drop that has not happened yet -- plus the measured
        # URDF finger error, because this gate asks a question about the REAL pads
        # and fk.pad_bottom_z answers about the modelled ones. Without the
        # correction the gate says "ready" while the real pads are still above the
        # object, and MIN_CLOSE_ACTION then shuts the jaw on the approach.
        # cfg.floor_finger_error_m is 0.0 by default, so this is identical to
        # training unless the operator has measured the finger and opted in.
        remaining_drop = max(0.0, dc.GRASP_CLOSE_DROP - close_drop)
        pad_bottom = self.fk.pad_bottom_z(arm, grip) + self.cfg.floor_finger_error_m
        pad_after_close = pad_bottom - remaining_drop
        object_top = float(obj_pos[2]) + 0.5 * height
        pads_ready = bool(
            pad_after_close <= object_top - dc.STAGE1_MIN_ENGAGE_DEPTH)
        return {
            "target": np.asarray(target, dtype=np.float64),
            "tcp": np.asarray(tcp, dtype=np.float64),
            "target_distance_m": radius,
            "xy_error_m": xy_err,
            "z_error_m": z_err,
            "centred": bool(centred),
            "pads_ready": pads_ready,
            "pad_after_close_m": float(pad_after_close),
            "finger_error_m": float(self.cfg.floor_finger_error_m),
            "object_top_m": float(object_top),
        }

    def _ready_to_close(self, obj_pos, obj_height) -> Tuple[bool, str, dict]:
        """Deployment close gate: centred on target AND finger pads ready."""
        g = self._grasp_geometry(obj_pos, obj_height)
        ok = bool(g["centred"] and g["pads_ready"])
        why = (f"xy={g['xy_error_m']*1000:.1f}/{self.cfg.entry_xy_tol*1000:.0f}mm "
               f"z={g['z_error_m']*1000:.1f}/{self.cfg.entry_z_tol*1000:.0f}mm "
               f"r={g['target_distance_m']*1000:.1f}/{self.cfg.entry_radius*1000:.0f}mm "
               f"centred={g['centred']} pads_ready={g['pads_ready']} "
               f"(pad_after_close={g['pad_after_close_m']*1000:.1f}mm, "
               f"object_top={g['object_top_m']*1000:.1f}mm)")
        return ok, why, g

    def _stage0_hover_check(self, g: dict) -> Tuple[bool, str]:
        """The bounded fallback for a policy parked just outside the entry radius.

        Fires only when xy, z and pads_ready are all genuinely inside their gates
        and ONLY the radius is out, by at most stage0_hover_extra_m, for
        stage0_hover_steps consecutive steps. A policy still approaching moves
        through this band in a step or two and never accumulates the streak; a
        policy in equilibrium sits in it forever, and three seconds of that is
        indistinguishable from arrival for every purpose the close cares about.
        """
        near = (g["pads_ready"]
                and g["xy_error_m"] < self.cfg.entry_xy_tol
                and g["z_error_m"] < self.cfg.entry_z_tol
                and g["target_distance_m"] < (self.cfg.entry_radius
                                              + self.cfg.stage0_hover_extra_m))
        self._hover_count = self._hover_count + 1 if near else 0
        if self._hover_count >= max(1, int(self.cfg.stage0_hover_steps)):
            return True, (f"hover fallback: {self._hover_count} consecutive steps "
                          f"within +{self.cfg.stage0_hover_extra_m*1000:.0f}mm of "
                          f"the entry radius")
        return False, ""

    def _prepare_policy_action(
        self,
        raw_action,
        obj_pos,
        obj_height: float,
        *,
        contact_detected: bool = False,
    ) -> dict:
        """Apply the frozen policy-action semantics once for the current step."""
        geometry = self._grasp_geometry(obj_pos, obj_height)
        raw = np.asarray(raw_action, dtype=np.float32)
        if raw.shape != (6,) or not np.all(np.isfinite(raw)):
            raise ValueError(f"policy action must be a finite (6,) vector, got {raw.shape}")
        raw = np.clip(raw, -1.0, 1.0)

        if self.contract.arm_action_mode == "incremental":
            floor_clearance = (
                self.fk.min_gripper_link_z(
                    self._current_arm_rads, self._current_grip_rad)
                - float(self.cfg.ground_height)
            )
            prepared = self._action_execution.prepare(
                raw,
                self._current_arm_rads,
                self._current_grip_rad,
                self.cfg.arm_sim_limits,
                target_distance_m=geometry["target_distance_m"],
                floor_clearance_m=floor_clearance,
                pads_ready=geometry["pads_ready"],
                contact_detected=contact_detected,
            )
        else:
            # Legacy reference only. v21 uses the stateful incremental path above.
            arm, grip = self.mapper.norm_action_to_sim_angles(
                raw, self._current_arm_rads)
            prepared = {
                "raw_action_for_observation": raw.copy(),
                "filtered_action": raw.copy(),
                "arm_target_rads_pre_floor_guard": arm,
                "grip_target_rad_pre_floor_guard": float(grip),
            }
        prepared["grasp_geometry"] = geometry
        return prepared
    def attempt_close(self) -> dict:
        """Close the jaw and decide whether stage 2 may run. Testable in isolation.

        Only ONE outcome permits a lift: the jaw was driven by an unobstructed,
        fully-fed-back command chain and stopped short of the closed stop because
        something physically blocked it. Everything else — the guard clamping the
        close, an emergency raise/hold, a timeout, a write or read failure, or the jaw
        closing all the way onto nothing — means we do not know that we are holding
        anything, and stage 2 must not run.

        The previous gate advanced whenever the failure reason merely lacked the word
        "failed", so a guard-clamped, never-completed close still went on to lift and
        print "Scripted return complete".
        """
        self._grip_hold_rad = None
        res = self.move_guarded_and_verified(
            self._current_arm_rads, dc.GRIPPER_ANGLE_CLOSED,
            label="stage1-close", run_time_ms=self.cfg.close_run_time_ms,
            settle_s=self.cfg.close_settle_s,
            max_iters=self.cfg.close_max_iters, tol_deg=3.0,
            stop_on_jaw_contact=True)

        actions = set(res.get("guard", []))
        blocking = actions - {"pass", "disabled"}
        out = {"may_lift": False, "close": res, "guard_actions": sorted(actions)}

        if blocking:
            out["reason"] = (f"floor guard intervened during the close ({sorted(blocking)}) "
                             f"— the jaw never reached a known state")
            return out
        if "failed" in res["reason"]:
            out["reason"] = f"hardware fault during the close: {res['reason']}"
            return out
        if res["reached"]:
            out["reason"] = ("jaw reached the fully-closed stop — nothing blocked it, "
                             "so there is no object between the fingers")
            return out
        if res.get("jaw_contact"):
            # The preferred outcome: contact caught early, command parked at
            # contact + bias instead of grinding on toward the stop.
            out["may_lift"] = True
            out["reason"] = (f"jaw contact at {res['jaw_contact_deg']:.1f} deg, "
                             f"holding at {res['jaw_hold_deg']:.1f} deg")
            return out
        if "no progress" not in res["reason"]:
            out["reason"] = f"close did not resolve to a stall: {res['reason']}"
            return out

        # Stalled with the guard passing and feedback valid — a real obstruction the
        # contact detector missed (e.g. an object soft enough to creep through the
        # progress threshold). Late is worse than early but still a grasp; park the
        # hold now so the rest of the run is not spent at near-stall torque.
        rd = self.servo.read_degrees()
        if rd.valid:
            hold = self._park_jaw_hold(float(rd.degrees[5]), dc.GRIPPER_ANGLE_CLOSED,
                                       run_time_ms=self.cfg.close_run_time_ms)
            out["reason"] = (f"jaw stalled against something ({res['reason']}); "
                             f"holding at {hold:.1f} deg")
        else:
            out["reason"] = f"jaw stalled against something ({res['reason']})"
        out["may_lift"] = True
        return out

    def _grasp_looks_real(self) -> Tuple[str, str]:
        """Best available check that something is actually between the fingers.

        This robot has no force or tactile sensor, so a true grasp signal does not
        exist. The one honest proxy available is the gripper servo's settled angle: a
        jaw closing on nothing reaches the fully-closed stop, while a jaw closing on an
        object stalls short of it. We require it to stall by a clear margin.

        This is a proxy, not a measurement. It cannot distinguish an object from a
        finger fouling, and it will call an over-soft object a miss. It exists so the
        controller does not simply assume "I sent a close command, therefore I am
        holding something" and then lift and drive off with nothing.
        """
        if self.servo.readings_are_simulated:
            # Three states, not two. A dry-run reading is fabricated, so it can neither
            # confirm nor refute a grasp. Returning True here previously let the run
            # print "Scripted return complete" as though something had been picked up.
            return "unverified", ("DRY-RUN — simulated readings; grasp can be neither "
                                  "confirmed nor refuted")
        rd = self.servo.read_degrees()
        if not rd.valid:
            return "rejected", (f"gripper servo unreadable ({rd.reason}) — grasp "
                                f"unconfirmed, treating as failure")
        grip_deg = float(rd.degrees[5])
        if not np.isfinite(grip_deg):
            return "rejected", "gripper servo returned a non-finite angle"
        closed = float(self.cfg.gripper_hw_closed)
        open_ = float(self.cfg.gripper_hw_open)
        span = abs(closed - open_)
        stall = abs(closed - grip_deg)
        frac = stall / span if span > 1e-6 else 0.0
        # A jaw that is barely closed is not holding anything either. This check
        # used to have a lower bound only -- "stopped short of fully closed" --
        # so a fully OPEN jaw scored 100% short and read as a confident grasp.
        # That is precisely the state run_release_only() exists to catch: an
        # object lost between the grasp and the bin. Measured on hardware
        # 2026-09-20: release on an empty jaw reported "CONFIRMED - jaw stalled
        # at 30.0deg (100.0% short of closed)" and ran the whole drop motion.
        # The floor is the fraction the stage-0 shortcut already requires before
        # it will call a stall a grasp, so the two paths agree on "closed enough".
        closed_frac = abs(grip_deg - open_) / span if span > 1e-6 else 0.0
        if closed_frac < self.cfg.timeout_close_min_grip_frac:
            return "rejected", (f"jaw sits at {grip_deg:.1f}deg, only {closed_frac*100:.0f}% of "
                                f"the way closed from {open_:.0f}deg — the fingers are not "
                                f"around anything")
        if frac < self.cfg.grasp_stall_min_fraction:
            return "rejected", (f"jaw reached {grip_deg:.1f}deg, only {frac*100:.1f}% short of "
                           f"fully closed ({closed:.0f}deg) — nothing detected between "
                           f"the fingers")
        return "confirmed", (f"jaw stalled at {grip_deg:.1f}deg "
                             f"({frac*100:.1f}% short of closed)")

    def _scripted_lift_and_return(self) -> str:
        """Open-loop lift and return home, mirroring what training scripts.

        Runs without policy input and without trusting tcp_pos, both of which are
        unreliable once the jaw is loaded. Every waypoint still goes through the floor
        guard. Returns True if it believes it delivered an object.
        """
        status, why = self._grasp_looks_real()
        print(f"[Stage 2] grasp check: {status.upper()} — {why}")
        if status == "rejected":
            print("[Stage 2] Not lifting. Opening jaw and returning home empty.")
            self._retreat_home_empty()
            self._outcome = "grasp_rejected"
            return "rejected"

        # Lift straight up first, in small guarded steps, before travelling home —
        # swinging toward home at grasp height would drag the object along the floor.
        arm = np.asarray(self._current_arm_rads, dtype=np.float64).copy()

        # Which way does S2 raise the gripper? Ask FK instead of trusting a sign
        # convention — getting it backwards commands a descent into the floor, which
        # is exactly what happened the first time this was written (the floor guard
        # caught it and clamped every lift step to zero).
        step_rad = float(self.cfg.scripted_lift_rad_per_step)
        probe_up = arm.copy();   probe_up[1] += step_rad
        probe_dn = arm.copy();   probe_dn[1] -= step_rad
        z_up = self.fk.compute(probe_up, self._current_grip_rad)[0][2]
        z_dn = self.fk.compute(probe_dn, self._current_grip_rad)[0][2]
        lift_sign = 1.0 if z_up > z_dn else -1.0
        print(f"[Stage 2] lift direction: S2 {'+' if lift_sign > 0 else '-'}"
              f"{step_rad} rad raises gripper_center "
              f"({max(z_up, z_dn)*100:.1f}cm vs {min(z_up, z_dn)*100:.1f}cm)")

        # Keep squeezing at the recorded hold, not at the encoder readback: the
        # readback IS the contact angle, and re-commanding it would zero the position
        # error the grip force comes from — the object would ride loose on the way up.
        grip_hold = (self._grip_hold_rad if self._grip_hold_rad is not None
                     else self._current_grip_rad)
        holding = self._grip_hold_rad is not None

        # tol_deg 2.5 -> 3.5, 2026-07-31: loosened alongside grasp_stall_min_fraction,
        # same provenance gap -- no commit recorded the change or the reasoning.
        # NOT YET VALIDATED ON HARDWARE. A looser arrival tolerance during the lift
        # means "reached" can be claimed with more residual error, which matters
        # here specifically because run 7's guard-deadlock investigation (same day)
        # found the 8mm floor line sitting between adjacent 1-degree encoder counts
        # on S2 -- the same joint this lift drives. Before trusting 3.5, confirm a
        # lift step never reports reached=True while floor_guard is still clamping
        # it short; see manifest.json transport.grasp_stall_margin_tuning.
        for i in range(self.cfg.scripted_lift_steps):
            target = np.asarray(self._current_arm_rads, dtype=np.float64).copy()
            target[1] += lift_sign * step_rad
            res = self.move_guarded_and_verified(
                target, grip_hold, label=f"lift-{i+1}",
                run_time_ms=self.cfg.lift_run_time_ms,
                settle_s=self.cfg.lift_settle_s, max_iters=8, tol_deg=3.5,
                grip_is_hold=holding)
            print(f"[Stage 2] lift {i+1}/{self.cfg.scripted_lift_steps}: "
                  f"reached={res['reached']} ({res['reason']}) guard={set(res['guard'])}")
            if not res["reached"]:
                print("[SAFETY] Aborting lift: the lift step was not confirmed.")
                self.servo.emergency_stop()
                self._outcome = "lift_not_confirmed"
                return "aborted"

        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, grip_hold, label="return-home",
            run_time_ms=self.cfg.home_run_time_ms,
            settle_s=self.cfg.home_settle_s, grip_is_hold=holding)
        print(f"[Stage 2] return home: reached={res['reached']} ({res['reason']}, "
              f"{res['iters']} steps)")
        if not res["reached"]:
            print("[SAFETY] Did not confirm arrival at home. Refusing to report a "
                  "completed delivery.")
            self.servo.emergency_stop()
            self._outcome = "return_home_not_confirmed"
            return "aborted"

        status2, why2 = self._grasp_looks_real()
        print(f"[Stage 2] post-return check: {status2.upper()} — {why2}")
        if status2 == "unverified":
            # Exercised the whole flow, proved nothing about a physical object.
            print("[Done] Scripted return executed. OUTCOME: UNVERIFIED — simulated "
                  "readings cannot establish that anything was picked up.")
            self._outcome = "unverified"
            return "unverified"
        if status2 == "confirmed":
            print("[Done] Scripted return complete; object still detected in the jaw.")
            self._outcome = "confirmed"
            return "confirmed"
        print("[Done] Scripted return complete, but the object is NO LONGER detected — "
              "it appears to have been dropped en route.")
        self._outcome = "dropped"
        return "dropped"

    def _scripted_release(self) -> str:
        """Reach forward a little, open the jaw, come back home. No policy, no aiming.

        Ported from the release branch under training/ on 2026-08-04. Unchanged in
        substance; the one v21-specific addition is clearing ``_grip_hold_rad``
        before opening, because in this stack the jaw target while carrying is a
        deliberate squeeze past the contact angle and every guarded move judges
        arrival against it. Leaving it set would make the open look like a stall.

        Verification: there is none, by necessity. ``_grasp_looks_real()`` infers a
        grasp from the jaw stalling short of fully closed, so once the jaw is
        commanded open it reads "confirmed" whether or not anything fell out.
        Re-closing on empty air would be a real check; it is deliberately not done,
        to keep the motion minimal. Read the outcome as "the release motion ran",
        not "the bin contains the object".
        """
        arm0 = np.asarray(self._current_arm_rads, dtype=np.float64).copy()
        grip0 = float(self._current_grip_rad)

        # Which way does S2 reach FORWARD? Ask FK rather than trusting a sign --
        # the lift got this backwards the first time it was written.
        step_rad = float(self.cfg.release_extend_rad_per_step)
        probe_p = arm0.copy(); probe_p[1] += step_rad
        probe_n = arm0.copy(); probe_n[1] -= step_rad
        x_p = self.fk.compute(probe_p, grip0)[0][0]
        x_n = self.fk.compute(probe_n, grip0)[0][0]
        fwd_sign = 1.0 if x_p > x_n else -1.0
        print(f"[Stage 3] reach direction: S2 {'+' if fwd_sign > 0 else '-'}{step_rad} rad "
              f"extends forward ({max(x_p, x_n)*100:.1f}cm vs {min(x_p, x_n)*100:.1f}cm)")

        # While carrying, the jaw command is a squeeze the encoder cannot reach, so
        # every reach step must be judged on the arm joints alone.
        hold_rad = float(self._current_grip_rad)
        min_pad_z = float(self.cfg.bin_rim_height) + float(self.cfg.release_clearance)
        for i in range(self.cfg.release_extend_steps):
            cand = np.asarray(self._current_arm_rads, dtype=np.float64).copy()
            cand[1] += fwd_sign * step_rad
            pad_z = self.fk.pad_bottom_z(cand, float(self._current_grip_rad))
            if pad_z < min_pad_z:
                # Reaching further would put the jaw below the rim. Releasing from
                # here is fine -- the bin is ~30 cm wide -- so stop, do not abort.
                print(f"[Stage 3] stopping the reach at step {i+1}: pad would sit at "
                      f"{pad_z*100:.1f}cm, below rim+clearance ({min_pad_z*100:.1f}cm). "
                      f"Releasing from here.")
                break
            res = self.move_guarded_and_verified(
                cand, hold_rad, label=f"release-reach-{i+1}",
                run_time_ms=self.cfg.lift_run_time_ms,
                settle_s=self.cfg.lift_settle_s, max_iters=8, tol_deg=2.5,
                grip_is_hold=True)
            print(f"[Stage 3] reach {i+1}/{self.cfg.release_extend_steps}: "
                  f"reached={res['reached']} ({res['reason']}) guard={set(res['guard'])}")
            if not res["reached"]:
                # Same reasoning: an unfinished reach is not a reason to keep
                # holding the object. Release from wherever the arm actually stopped.
                print("[Stage 3] Reach not confirmed — releasing from the current pose.")
                break

        pad_now = self.fk.pad_bottom_z(
            np.asarray(self._current_arm_rads, dtype=np.float64),
            float(self._current_grip_rad))
        if pad_now < min_pad_z:
            # Opening here would let go with the jaw at or below the rim, dropping
            # the object down the outside of the bin. Keep holding and report --
            # that is recoverable, a misplaced drop is not.
            print(f"[SAFETY] Pad bottom is {pad_now*100:.1f}cm, not clear of the "
                  f"{self.cfg.bin_rim_height*100:.0f}cm rim + "
                  f"{self.cfg.release_clearance*100:.0f}cm clearance. Refusing to open "
                  f"the jaw here; keeping hold of the object.")
            self._outcome = "release_pose_below_rim"
            return "aborted"

        x_now = self.fk.compute(
            np.asarray(self._current_arm_rads, dtype=np.float64),
            float(self._current_grip_rad))[0][0]
        print(f"[Stage 3] releasing at x={x_now*100:.1f}cm "
              f"({(x_now - self.fk.compute(arm0, grip0)[0][0])*100:+.1f}cm forward of the "
              f"post-lift pose), pad bottom {pad_now*100:.1f}cm vs rim "
              f"{self.cfg.bin_rim_height*100:.0f}cm")

        # Drop the squeeze target before commanding open, or arrival is judged
        # against a jaw position that is no longer wanted.
        self._grip_hold_rad = None
        res = self.move_guarded_and_verified(
            np.asarray(self._current_arm_rads, dtype=np.float64).copy(),
            dc.GRIPPER_ANGLE_OPEN, label="release-open",
            run_time_ms=self.cfg.close_run_time_ms,
            settle_s=self.cfg.close_settle_s, max_iters=self.cfg.close_max_iters,
            tol_deg=3.0)
        print(f"[Stage 3] open jaw: reached={res['reached']} ({res['reason']}, "
              f"{res['iters']} steps)")
        if not res["reached"]:
            print("[SAFETY] The jaw was not confirmed open — not retracting with a "
                  "possibly still-held object.")
            self.servo.emergency_stop()
            self._outcome = "release_not_confirmed"
            return "aborted"

        # Let it fall clear before anything moves; retracting mid-open flicks it.
        time.sleep(float(self.cfg.release_settle_s))

        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, self.mapper.hw_deg_to_sim_grip(self.cfg.home_deg[5]),
            label="release-home", run_time_ms=self.cfg.home_run_time_ms,
            settle_s=self.cfg.home_settle_s)
        print(f"[Stage 3] return home: reached={res['reached']} ({res['reason']})")
        if not res["reached"]:
            self._outcome = "release_home_not_confirmed"
            return "aborted"

        print("[Done] Release motion complete (the drop itself is not sensed).")
        self._outcome = "released"
        return "released"

    def run_release_only(self) -> str:
        """Stage 3 on its own, for a machine already standing at the bin.

        Does NOT move to home first: the arm is holding something, and a home move
        under load is a large swing for no benefit. It reads the pose off the
        encoders and reaches from wherever it is.
        """
        print("\n" + "=" * 60)
        print("X3Plus Release — Stage 3 only")
        print(f"  reach forward, open jaw, return home "
              f"(bin rim {self.cfg.bin_rim_height*100:.0f}cm)")
        print("=" * 60 + "\n")

        if not self._sync_joint_state_from_servos():
            print("[SAFETY] Cannot read the arm pose. Refusing to move.")
            self._outcome = "servo_unreadable"
            return "aborted"

        hw = self.mapper.sim_arm_to_hw_deg(self._current_arm_rads)
        hw.append(self.mapper.sim_grip_to_hw_deg(self._current_grip_rad))
        print(f"[Start] Measured pose S1-6 = {[round(d, 1) for d in hw]}")

        status, why = self._grasp_looks_real()
        print(f"[Start] grasp check: {status.upper()} — {why}")
        if status == "rejected":
            # Nothing in the jaw. Opening it here would be theatre, and would hide
            # the fact that the object was lost between the grasp and the bin.
            print("[Start] Nothing detected in the jaw — there is nothing to release.")
            self._outcome = "nothing_to_release"
            return "rejected"

        self._stage = 3
        return self._scripted_release()

    def _retreat_home_empty(self) -> None:
        """Open the jaw and go home, guarded. Used on every abort path."""
        self._grip_hold_rad = None
        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, dc.GRIPPER_ANGLE_OPEN, label="retreat-home",
            run_time_ms=self.cfg.home_run_time_ms,
            settle_s=self.cfg.home_settle_s)
        print(f"[Retreat] home: reached={res['reached']} ({res['reason']})")

    def move_home(self) -> bool:
        """Public, guarded, verified move to cfg.home_deg with the jaw open.

        The equivalent of grasp/x3plus_real_grasp.py's ServoController.move_to_home()
        for an external caller (a vision pipeline driving the chassis between
        attempts), but deliberately NOT placed on ServoController: that class has no
        FloorGuard or FK, so a servo-level move_to_home() would have to bypass the
        guard entirely, in violation of "every motion goes through
        move_guarded_and_verified" (enforced by
        test_deploy_controller.py's every_motion_is_guarded check). This returns only
        once the encoders confirm arrival, so a caller does not need its own
        fixed-duration sleep afterward.
        """
        self._grip_hold_rad = None
        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, dc.GRIPPER_ANGLE_OPEN, label="move-home",
            run_time_ms=self.cfg.home_run_time_ms,
            settle_s=self.cfg.home_settle_s)
        print(f"[Home] reached={res['reached']} ({res['reason']})")
        return bool(res["reached"])

    def _sync_joint_state_from_servos(self) -> bool:
        """Refresh internal joint state from the encoders. False if unreadable."""
        rd = self.servo.read_degrees()
        if not rd.valid:
            print(f"[SAFETY] Servo read failed: {rd.reason}")
            return False
        self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
        self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])
        return True

    def run(self, max_steps: int = 300) -> bool:
        """Run one grasp episode. Returns True only if the outcome is "confirmed"
        (jaw stalled on something, and it was still detected after the return home).
        Every other ending -- nothing to grasp, a safety abort, a guard deadlock, an
        unreadable bus -- returns False. A caller driving multiple attempts (e.g. a
        vision pipeline retrying on failure) can call this repeatedly on the same
        instance; every piece of episode state is reset at the top of this method."""
        print("\n" + "="*60)
        print("X3Plus Real Grasp Controller")
        print(f"  Stage 0 → align gripper above object")
        print(f"  Stage 1 → close gripper (grasp)")
        print(f"  Stage 2 → lift and return home")
        print("="*60 + "\n")

        # Move to the pose the policy was trained to start from. Usually the same as
        # cfg.home_deg (grasp_home_deg defaults to None), but a caller that drives
        # around at a different nav pose (grasp_home_deg set) needs this to be the
        # trained pose specifically -- see DeployConfig.grasp_home_deg.
        start_pose = (self.cfg.grasp_home_deg if self.cfg.grasp_home_deg is not None
                     else self.cfg.home_deg)

        # Move to home first — guarded and verified like every other motion. This is
        # the largest single move the robot makes, so a one-shot 8 deg-limited command
        # gets nowhere near it.
        print(f"[Start] Moving to grasp start pose {list(start_pose)}...")
        rd = self.servo.read_degrees()
        if rd.valid:
            self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
            self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])
        elif not self.servo.dry_run:
            print(f"[SAFETY] Cannot read servos at startup ({rd.reason}). Refusing to move.")
            return False
        home = self.mapper.hw_deg_to_sim_arm(list(start_pose[:5]))
        # The chassis moving over the floor can leave a real hobby servo a little
        # off its calibrated C3 angle.  A bounded pose tolerance is safe here only
        # when the detection stamp uses the SAME tolerance: otherwise startup would
        # accept a camera pose that _verify_detection_pose() rejects moments later.
        # Never inherit more than 3 deg even if a caller weakens the detection gate;
        # larger offsets need a new camera measurement, not optimistic recovery.
        startup_tol_deg = min(3.0, max(2.0, self.cfg.detection_pose_tol_deg))
        res = self.move_guarded_and_verified(
            home, self.mapper.hw_deg_to_sim_grip(start_pose[5]),
            label="startup-home", run_time_ms=self.cfg.home_run_time_ms,
            settle_s=self.cfg.home_settle_s,
            tol_deg=startup_tol_deg)
        print(f"[Start] home: reached={res['reached']} ({res['reason']}, "
              f"{res['iters']} steps; tolerance {startup_tol_deg:.1f} deg)")
        if not res["reached"]:
            print("[SAFETY] Did not confirm the home pose. Aborting before any grasp.")
            return False
        self._stage = 0
        self._wrist_z_offset = None
        self._grip_hold_rad = None
        self._hover_count = 0
        self._guard_tally = {}
        self._guard_deadlock_count = 0
        self._guard_deadlock_z = None
        self._episode_object_height = None
        self._object_height_source = None
        self._prev_action = np.zeros(6, dtype=np.float32)
        self._action_execution.reset(self._current_grip_rad)
        self._stage0_s6_prev_deg = None
        self._stage0_s6_prev_cmd_deg = None
        self._stage0_s6_stall_count = 0

        # Latch AFTER the arm has confirmed the home pose: that is the only pose at
        # which the arm camera's distance model holds, so it is the only moment a
        # detection is worth freezing. Latching before the move would freeze a
        # reading taken from wherever the arm happened to be sitting.
        self._obj_latched = False
        self._latched_obj = None
        self._latched_height = None
        if self.cfg.latch_obj:
            self._latch_object()
        elif self.detection is not None and self.obj_provider is None:
            print("[WARN] --socket without --latch-obj: the arm camera moves with "
                  "arm_link4, so its distance model stops holding the moment the arm "
                  "leaves home. The target will drift under the policy mid-approach. "
                  "Add --latch-obj.")

        dt = 1.0 / self.cfg.control_hz
        obj_pos = self._get_obj_pos()
        print(f"[Start] Object position: {obj_pos.tolist()}")
        # One choke point for every source of a target -- socket latch, the
        # pipelines' obj_provider, and a hand-typed --obj-x. Checked here, with
        # the arm still parked at home, so refusing costs nothing.
        if not self._check_target_envelope(obj_pos):
            return False
        print(f"[Start] Running for up to {max_steps} steps at {self.cfg.control_hz} Hz\n")

        for step in range(max_steps):
            t0 = time.time()

            obj_pos = self._get_obj_pos()
            obj_height = self._get_obj_height(obj_pos)
            # Frozen at the first detection, while the object is still resting on the
            # floor — matching the training env, which fixes it at reset. Recomputing
            # it from the object's live z drifts the target once the object lifts.
            if self._wrist_z_offset is None:
                self._wrist_z_offset = dc.episode_wrist_z_offset(
                    obj_height, float(obj_pos[2]))
                print(f"[Init] episode wrist_z_offset = {self._wrist_z_offset:.4f} m "
                      f"(object height {obj_height}, resting z {float(obj_pos[2]):.4f})")
            obs = self.obs_builder.build(
                self._current_arm_rads,
                self._current_grip_rad,
                obj_pos,
                self._stage,
                self._prev_action,
                obj_height=obj_height,
                wrist_z_offset=self._wrist_z_offset,
            )

            # ── Model inference ───────────────────────────────────────────
            action, _ = self.model.predict(obs, deterministic=True)
            action = np.asarray(action, dtype=np.float32)
            if action.shape != (6,) or not np.all(np.isfinite(action)):
                raise ContractMismatchError(
                    "policy returned a non-finite or wrong-shaped action: "
                    f"{action.shape}"
                )
            prepared = self._prepare_policy_action(
                action, obj_pos, obj_height, contact_detected=False)
            arm_sim = prepared["arm_target_rads_pre_floor_guard"]
            grip_sim = prepared["grip_target_rad_pre_floor_guard"]
            dist_to_target = prepared["grasp_geometry"]["target_distance_m"]

            # ── Stage management ──────────────────────────────────────────


            # Stage transitions
            if self._stage == 0:
                ready, why, g = self._ready_to_close(obj_pos, obj_height)
                if ready:
                    self._hover_count = 0
                    print(f"\n[Stage] 0→1  {why}")
                    self._stage = 1
                else:
                    fire, how = self._stage0_hover_check(g)
                    if fire:
                        print(f"\n[Stage] 0→1 ({how})  {why}")
                        self._stage = 1
                    elif (step + 1) % 25 == 0:
                        # Run 5 hovered at r=27/26mm for 270 steps and the log showed
                        # only target_dist — which gate term was short, and whether
                        # the guard was the limiter, was unrecoverable. Print both.
                        print(f"\n[Gate {step+1:3d}] {why} "
                              f"guard={dict(self._guard_tally)}")

            elif self._stage == 1:
                verdict = self.attempt_close()
                res = verdict["close"]
                print(f"\n[Stage] 1: jaw close — reached={res['reached']} "
                      f"({res['reason']}, {res['iters']} steps) "
                      f"guard={verdict['guard_actions']}")
                if not verdict["may_lift"]:
                    print(f"[Stage] 1 ABORT — {verdict['reason']}")
                    print("[Stage] Not lifting. Opening the jaw and retreating home.")
                    self._retreat_home_empty()
                    self._outcome = "close_not_confirmed"
                    return False
                print(f"[Stage] 1→2  {verdict['reason']}")
                self._prev_action = prepared[
                    "raw_action_for_observation"].copy()
                self._stage = 2
                continue

            elif self._stage == 2:
                # ── SCRIPTED, not policy-driven ───────────────────────────
                # Training scripts the arm from the moment the grasp latches, so the
                # policy was never trained to act here. Worse, this is exactly where
                # deployment FK is least trustworthy: once the jaw squeezes an object
                # the linkage leaves the grip_joint*multiplier relation (measured up to
                # 0.199 rad), and tcp_pos drifts by ~4 cm. Feeding that observation back
                # into the policy would be driving on a broken sensor. So stage 2 runs
                # open-loop, matching training.
                #
                # And then RETURN, not break: the shared end-of-run block re-targets
                # the home pose with the jaw OPEN. Falling into it after a confirmed
                # grasp opened the jaw at home and dropped the object; after an
                # aborted lift it moved an arm that emergency_stop had just frozen.
                # Stage 2 has already homed (or deliberately frozen) on every path.
                outcome = self._scripted_lift_and_return()
                if outcome == "confirmed":
                    print("[End] The jaw keeps its hold — take the object, then "
                          "Ctrl+C. The servos hold position while powered.")
                return self._outcome == "confirmed"

            # ── Convert action → servo degrees ────────────────────────────
            # Last gate before the servos. Prevention only — there is no undo on hardware.
            arm_sim, grip_sim, ginfo = self.floor_guard.project(
                self._current_arm_rads, self._current_grip_rad, arm_sim, grip_sim)
            self._guard_tally[ginfo["action"]] = (
                self._guard_tally.get(ginfo["action"], 0) + 1)
            if ginfo["action"] != "pass":
                print(f"  [floor guard] {ginfo['action']} {ginfo}")

            # Deadlock detector. Intervening is fine; intervening forever without
            # gaining clearance is not — that is the policy and the guard in a stable
            # disagreement, and it will burn every remaining step looking busy.
            if ginfo["action"] in ("emergency_raise", "emergency_hold"):
                z_now = float(ginfo.get("z_now", 0.0))
                gained = (self._guard_deadlock_z is None
                          or z_now > self._guard_deadlock_z
                          + self.cfg.guard_deadlock_progress_m)
                self._guard_deadlock_z = (z_now if self._guard_deadlock_z is None
                                          else max(self._guard_deadlock_z, z_now))
                self._guard_deadlock_count = 0 if gained else self._guard_deadlock_count + 1
                if self._guard_deadlock_count >= self.cfg.guard_deadlock_steps:
                    pad = ("the finger pads" if ginfo.get("low_is_pad")
                           else f"link {ginfo.get('low_link')}")
                    print(f"\n[SAFETY] Floor guard deadlock: "
                          f"{self._guard_deadlock_count} consecutive interventions "
                          f"with no clearance gained (z stuck at {z_now*1000:.1f}mm, "
                          f"line {self.floor_guard.line*1000:.1f}mm, lowest part is "
                          f"{pad}).")
                    print("[SAFETY] The policy keeps commanding down and the guard "
                          "keeps refusing. Continuing would only repeat this.")
                    if ginfo.get("low_is_pad"):
                        print(f"[Hint] The pads are the limiter, and the URDF finger "
                              f"is 16.7mm longer than the real one — real clearance "
                              f"is likely ~{(z_now + 0.0167)*1000:.0f}mm, not "
                              f"{z_now*1000:.1f}mm. Measure it, and if it confirms, "
                              f"--floor-finger-error-mm 16.7 lets the guard use the "
                              f"real geometry.")
                    self._outcome = "guard_deadlock"
                    self._retreat_home_empty()
                    return False
            else:
                self._guard_deadlock_count = 0

            # Through the same primitive as every other motion, with max_iters=1: a
            # policy step is one small increment, so it should not be chased to
            # convergence — but it still gets the identical guard, write-failure and
            # read-failure handling, and its state still comes from the encoders.
            res = self.move_guarded_and_verified(
                arm_sim, grip_sim, label="policy-step",
                run_time_ms=self.cfg.servo_run_time_ms, settle_s=0.0,
                max_iters=1, tol_deg=self.cfg.max_delta_deg + 1.0)
            if "failed" in res["reason"]:
                print(f"[SAFETY] {res['reason']} during approach — stopping.")
                # One aborted step says almost nothing about a flaky bus. The tally
                # says whether it is one servo or all of them, and how often it has
                # been happening while the run looked healthy.
                print(f"[Bus] step {step+1}: {self.servo.read_failure_summary()}")
                self.servo.emergency_stop()
                return False
            self._prev_action = prepared[
                "raw_action_for_observation"].copy()

            # Status — report the MEASURED pose, not the command we sent.
            meas = self.mapper.sim_arm_to_hw_deg(self._current_arm_rads)
            meas.append(self.mapper.sim_grip_to_hw_deg(self._current_grip_rad))

            # Optional hardware contact shortcut. Some policies close the real jaw
            # on the object while still in Stage 0, then park there indefinitely
            # because the modelled XY gate is not satisfied. Waiting for max-steps
            # used to invoke attempt_close(), which added jaw_hold_bias_deg and made
            # the loaded gearbox chatter. Three stable encoder readings at a genuine
            # partially-closed contact are enough evidence to lift directly.
            if (self.real_servo and self._stage == 0
                    and self.cfg.stage0_s6_stall_steps > 0):
                jaw_hw = float(meas[5])
                open_ = float(self.cfg.gripper_hw_open)
                closed = float(self.cfg.gripper_hw_closed)
                span = max(abs(closed - open_), 1e-6)
                closed_frac = abs(jaw_hw - open_) / span
                short_of_stop = abs(closed - jaw_hw) / span
                closing = float(action[5]) >= self.cfg.stage0_s6_stall_min_action
                if (closing and abs(closed - jaw_hw)
                        <= self.cfg.stage0_s6_empty_stop_tol_deg):
                    print(f"\n[Stage] 0 ABORT — S6 reached the empty-jaw stop at "
                          f"{jaw_hw:.1f}deg (closed={closed:.0f}deg); nothing is "
                          "between the fingers. Returning home immediately.")
                    self._outcome = "empty_jaw_closed"
                    self._retreat_home_empty()
                    return False
                eligible = (closing
                            and closed_frac >= self.cfg.timeout_close_min_grip_frac
                            and short_of_stop >= self.cfg.grasp_stall_min_fraction)
                jaw_cmd = float(self.servo._last_deg[5])
                evidence = self._observe_stage0_jaw(jaw_hw, jaw_cmd, eligible)

                if self._stage0_s6_stall_count >= self.cfg.stage0_s6_stall_steps:
                    status, why = self._grasp_looks_real()
                    if status == "confirmed":
                        # Add only a small, bounded position error to maintain grip
                        # force. The old 8-degree bias made the loaded gearbox chatter;
                        # zero bias held too loosely. Setting the field also tells the
                        # lift primitive that S6 is intentionally loaded and must not
                        # be judged by normal arrival error.
                        hold_hw = self._park_jaw_hold(
                            jaw_hw, dc.GRIPPER_ANGLE_CLOSED,
                            self.cfg.servo_run_time_ms)
                        self._stage = 2
                        mode = evidence.get("mode") or "stalled"
                        detail = (f"encoder moved "
                                  f"{evidence['encoder_progress_deg']:.1f}deg of "
                                  f"{evidence['requested_progress_deg']:.1f}deg requested"
                                  if mode == "slipping" else "encoder stopped")
                        print(f"\n[Stage] 0→2  S6 {mode} for "
                              f"{self._stage0_s6_stall_count} steps at {jaw_hw:.1f}deg "
                              f"({detail}); "
                              f"{why}. Holding at {hold_hw:.1f}deg "
                              f"(+{self.cfg.jaw_hold_bias_deg:.0f}deg bias).")
                        outcome = self._scripted_lift_and_return()
                        if outcome == "confirmed":
                            print("[End] The jaw keeps its hold — take the object, then "
                                  "Ctrl+C. The servos hold position while powered.")
                        return self._outcome == "confirmed"
                    # A stop that the normal grasp proxy rejects (notably 180 deg)
                    # must never accumulate into a later success.
                    self._stage0_s6_stall_count = 0
                    self._stage0_s6_prev_deg = None
                    self._stage0_s6_prev_cmd_deg = None
            print(f"\r[Step {step+1:3d}] Stage={self._stage} "
                  f"target_dist={dist_to_target:.3f}m "
                  f"grip_cmd={float(action[5]):.2f} "
                  f"S1-6={[round(d,1) for d in meas]}",
                  end="", flush=True)

            elapsed = time.time() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
        else:
            print(f"\n[End] Max steps ({max_steps}) reached.")
            if max_steps > 0:
                # obj_pos/obj_height hold their last in-loop values here.
                _, why, _g = self._ready_to_close(obj_pos, obj_height)
                print(f"[Gate end] {why}")
                print(f"[Guard] policy-step floor guard: {dict(self._guard_tally)}")

            # Timeout rescue. Run 5 timed out with the jaw at 138 deg and the
            # object IN it — the policy had grasped on its own, one millimetre
            # outside the entry gate — and the end-of-run path opened the jaw and
            # gave the object back. A mostly-closed jaw at timeout earns one
            # attempt at the scripted close: an empty jaw walks to the stop and
            # aborts exactly as before, so the rescue costs a few seconds and can
            # only ever upgrade the outcome.
            open_ = float(self.cfg.gripper_hw_open)
            closed = float(self.cfg.gripper_hw_closed)
            grip_hw = self.mapper.sim_grip_to_hw_deg(self._current_grip_rad)
            frac = abs(grip_hw - open_) / max(abs(closed - open_), 1e-6)
            if self._stage == 0 and frac >= self.cfg.timeout_close_min_grip_frac:
                print(f"[End] Jaw is {frac*100:.0f}% closed at timeout — the policy "
                      f"may be holding something. Trying the close before opening.")
                verdict = self.attempt_close()
                res = verdict["close"]
                print(f"[Stage] 1: jaw close — reached={res['reached']} "
                      f"({res['reason']}, {res['iters']} steps) "
                      f"guard={verdict['guard_actions']}")
                if verdict["may_lift"]:
                    print(f"[Stage] 1→2 (timeout rescue)  {verdict['reason']}")
                    outcome = self._scripted_lift_and_return()
                    if outcome == "confirmed":
                        print("[End] The jaw keeps its hold — take the object, then "
                              "Ctrl+C. The servos hold position while powered.")
                    return self._outcome == "confirmed"
                print(f"[End] Nothing held ({verdict['reason']}).")

        # Return to home — through the guarded primitive like everything else.
        print("\n[End] Moving back to home...")
        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, self.mapper.hw_deg_to_sim_grip(self.cfg.home_deg[5]),
            label="end-home", run_time_ms=self.cfg.home_run_time_ms,
            settle_s=self.cfg.home_settle_s)
        print(f"[End] home: reached={res['reached']} ({res['reason']})")
        # Reached here only via the max-steps timeout with no rescue (stage never
        # got past 0, or the rescue found nothing to hold) -- never a grasp.
        return False

    def close(self):
        # Each resource is independent. One failed cleanup must not keep the
        # serial receive thread alive and leave /dev/myserial claimed.
        for name, obj, method in (
            ("detection", getattr(self, "detection", None), "close"),
            ("servo", getattr(self, "servo", None), "_close_device"),
            ("fk", getattr(self, "fk", None), "close"),
        ):
            if obj is None:
                continue
            try:
                getattr(obj, method)()
            except Exception as exc:
                print(f"[WARN] {name} cleanup failed: {exc}")


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _release_gate_ok(cfg: DeployConfig, unlock: bool,
                     manifest_path: Optional[str] = None) -> bool:
    """Refuse --real unless the weights match manifest.json and the release is cleared.

    Every failure this package can suffer on hardware is silent: a model paired with
    the wrong VecNormalize, the incremental weights decoded as absolute, a stack whose
    hardware motion/grasp gates are still false, so the candidate must not be driven
    as though it were certified. A measured homography alone does not approve motion.
    So the gate is checked here, before the serial port is opened and before anything
    is loaded, and it is checked against manifest.json rather than against belief.

    `unlock` waives the release-status check ONLY. Integrity and contract are never
    waivable — there is no legitimate reason to drive the arm with weights that are
    not the ones this package documents.
    """
    script_dir = Path(__file__).resolve().parent
    man_path = (Path(manifest_path).expanduser().resolve()
                if manifest_path else script_dir / "manifest.json")
    asset_dir = man_path.parent
    if not man_path.exists():
        print(f"[FATAL] --real requires {man_path}, which is missing. Without it the "
              "weights cannot be verified.")
        return False
    try:
        with open(man_path, "r", encoding="utf-8") as f:
            man = json.load(f)
    except Exception as e:
        print(f"[FATAL] {man_path} is unreadable ({e}). Refusing --real.")
        return False

    # 1. Integrity — the model and the VecNormalize are one unit (manifest pairing_rule).
    arts = man.get("artifacts", {})
    checks = [("model", cfg.model_path, arts.get("model", {}).get("sha256")),
              ("vecnormalize", cfg.vecnorm_path, arts.get("vecnormalize", {}).get("sha256"))]
    bad = False
    for label, rel, want in checks:
        if not want:
            print(f"[FATAL] manifest.json carries no sha256 for {label}. Refusing --real.")
            bad = True
            continue
        try:
            got = _sha256(_resolve_asset(rel, asset_dir))
        except FileNotFoundError as e:
            print(f"[FATAL] {label}: {e}")
            bad = True
            continue
        if got != want:
            print(f"[FATAL] {label} sha256 mismatch — this is NOT the file this package "
                  f"documents.\n         path: {rel}\n         got:  {got}\n"
                  f"         want: {want}")
            bad = True
    if bad:
        print("        Never mix a model with another run's VecNormalize "
              f"({arts.get('pairing_rule', '')}).")
        return False

    # 2. Contract — v21 is incremental; decoding it as absolute drives the arm to the
    #    wrong place at full range and no shape check anywhere would notice.
    want_contract = man.get("contract", {}).get("name")
    if want_contract and cfg.contract_name != want_contract:
        print(f"[FATAL] contract mismatch: running as '{cfg.contract_name}' but these "
              f"weights were trained under '{want_contract}'.\n"
              "        Both are 28D/6D, so nothing downstream would catch this. "
              "Drop --contract, or pass the manifest value.")
        return False

    # 3. Release status — hardware evidence, not a formality.
    status = man.get("status")
    if status != "hardware-approved" and not unlock:
        gates = man.get("hardware_gates", {})
        print(f"[REFUSED] --real blocked: package status is '{status}', not "
              "'hardware-approved'.")
        print("          Hardware gates:")
        for k, v in gates.items():
            if k == "note":
                continue
            print(f"            [{'x' if v else ' '}] {k}")
        print("          This is expected for a candidate. To run it anyway as a "
              "supervised experiment, add --unlock-candidate-real, stay beside the "
              "robot and keep a hand on the power switch.")
        return False

    if status != "hardware-approved":
        print(f"[UNLOCKED] status='{status}' overridden by --unlock-candidate-real. "
              "Integrity and contract verified. Supervised experiment — hand on power.")
    return True


def parse_deg6_csv(value: str) -> Tuple[float, ...]:
    """Parse "90,67.08,9.79,9.79,90,30" into a 6-tuple. Matches
    grasp/x3plus_real_grasp.py's parser exactly, so callers that build a
    DeployConfig for either stack (e.g. the vision pipelines) can use either
    module's copy interchangeably."""
    vals = [float(x.strip()) for x in value.split(",") if x.strip()]
    if len(vals) != 6:
        raise argparse.ArgumentTypeError(f"expected 6 comma-separated degrees, got {len(vals)}")
    return tuple(vals)


def parse_args():
    p = argparse.ArgumentParser(description="X3Plus real-robot grasp deployment")
    p.add_argument("--model", type=str, default=None,
                   help="Path to .zip model (default: trained_6d_models_v18/ppo_6d_final_ready_for_real_robot.zip)")
    p.add_argument("--vecnorm", type=str, default=None,
                   help="Path to vecnormalize .pkl")
    p.add_argument("--release-manifest", type=str, default=None,
                   help="Manifest that owns this exact model/VecNormalize pair. "
                        "Defaults to grasp/v23/manifest.json. Versioned wrappers "
                        "may set this to reuse the hardened E1 controller without "
                        "copying it.")
    p.add_argument("--real", action="store_true",
                   help="Actually send commands to servos (default: dry-run print only)")
    p.add_argument("--unlock-candidate-real", action="store_true",
                   help="Permit --real while this package is still a candidate (its "
                        "hardware_gates are not all true). Supervised experiment only: "
                        "stay beside the robot, hand on the power switch. Does NOT "
                        "waive the sha256 or contract checks.")
    p.add_argument("--socket", action="store_true",
                   help="Listen for object detection on TCP socket (port 5555)")
    p.add_argument("--socket-port", type=int, default=None,
                   help="TCP port to listen on (default 5555). Binds 0.0.0.0, so the "
                        "detector may run on another machine.")
    p.add_argument("--stale-timeout", type=float, default=None,
                   help="Seconds after which a detection counts as no detection "
                        "(default 1.0). Also bounds how far the arm can have moved "
                        "since the frame was taken, so keep it small; raise it only "
                        "if the detector is genuinely slower than this.")
    p.add_argument("--latch-wait", type=float, default=None,
                   help="Seconds to wait at the home pose for a fresh detection "
                        "before giving up (default 5.0). Under --real a timeout "
                        "aborts rather than falling back to the default position.")
    p.add_argument("--pose-tol-deg", type=float, default=None,
                   help="How far the arm may be from the pose a detection declares "
                        "it was computed for, before the detection is refused "
                        "(default 1.0 deg).")
    p.add_argument("--entry-xy-mm", type=float, default=None,
                   help="Optional stricter stage-0 XY alignment required before the "
                        "jaw may close, in mm (default 22, allowed range (0, 22]). "
                        "Use 10 for a narrow object during supervised hardware tests. "
                        "Values above the training gate are refused.")
    p.add_argument("--s6-stall-grasp-steps", type=int, default=None,
                   help="Opt in to accepting a Stage-0 jaw stall as a grasp after N "
                        "unchanged S6 readings. Requires a closing policy command, "
                        "jaw >50%% closed, and a stop short of empty-jaw 180 deg. "
                        "Use 3 for the validated 151-degree hardware contact. The "
                        "contact is then held with a small 1-degree squeeze bias.")
    p.add_argument("--latch-obj", action="store_true",
                   help="Capture the object position/height ONCE at the home pose and "
                        "freeze it for the episode. The arm camera rides on arm_link4, "
                        "so its distance model only holds at home. Required with "
                        "--real --socket.")
    p.add_argument("--i-confirm-external-frame", action="store_true",
                   help="Confirm the XYZ arriving on the socket is already expressed in "
                        "the PPO/URDF base_link frame, not raw camera coordinates. "
                        "Required with --real --socket: nothing downstream can detect a "
                        "wrong frame, it just grasps confidently in the wrong place.")
    p.add_argument("--obj-x", type=float, default=0.26,
                   help="Object X position in metres (forward, default 0.26; v18 deployable x 0.20-0.33)")
    p.add_argument("--obj-y", type=float, default=0.00,
                   help="Object Y position in metres (lateral, default 0.00)")
    p.add_argument("--obj-z", type=float, default=0.02,
                   help="Object centroid Z in metres (NOT its height; default 0.02)")
    p.add_argument("--object-height", type=float, default=None,
                   help="Object height = full z extent, metres (e.g. 0.013 for a bottle "
                        "cap). Required by obs_32_goal_incremental. Distinct from --obj-z.")
    p.add_argument("--contract", type=str, default=None,
                   choices=sorted(dc.CONTRACTS),
                   help="Observation/action contract the weights were trained under. "
                        "v18/v19 = obs_28_absolute, v20 = obs_32_goal_incremental. "
                        "Read it off the model's manifest.json — it cannot be inferred.")
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--jaw-track-fraction", type=float, default=None,
                   help="treat the jaw as having reached the object when the "
                        "encoder advances less than this FRACTION of what the "
                        "command asked for (0 disables, 0.5 is the value to try). "
                        "Applies to the Stage-0 policy close and Stage-1 scripted "
                        "close. The absolute test only sees a jaw that has stopped, "
                        "so an object that slips a little keeps the streak reset "
                        "and the command keeps squeezing — the clicking heard "
                        "during a close.")
    p.add_argument("--jaw-max-lag-deg", type=float, default=None,
                   help="hard ceiling on how far the jaw command may run past the "
                        "encoder during a close (default 15). Grip force IS that "
                        "position error, so this caps the torque by construction "
                        "even when no detector fires. 0 disables the ceiling.")
    p.add_argument("--floor-finger-error-mm", type=float, default=0.0,
                   help="Raise the finger pads by this much EVERYWHERE this code "
                        "reasons about where the pads are: the floor guard AND the "
                        "stage-1 pads_ready gate. Undoes the measured URDF finger "
                        "error (16.7mm at C3 2026-07-31; 11.9mm open / 15.4mm "
                        "closed at E1 2026-08-29). Default 0 keeps the "
                        "training-identical metric, under which the guard stops "
                        "~1.5cm early (deadlocked run 6) AND pads_ready fires while "
                        "the real pads are still above the object, shutting the jaw "
                        "on the approach (jammed run, 2026-08-30). Measure the real "
                         "clearance with pose_check.py --real first — too large a "
                         "value drives the fingers into the floor.")
    p.add_argument("--tcp-forward-error-mm", type=float, default=0.0,
                   help="real gripper landing correction: treat the FK TCP as this "
                        "many mm too far forward, so policy and close gate continue "
                        "farther in base +X (default 0; allowed 0..20). This does not "
                        "alter camera/object coordinates or floor-collision FK.")
    p.add_argument("--slow-motion", action="store_true",
                   help="restore the pre-2026-09-09 scripted-move timing (close "
                        "300ms/0.30s, lift 250/0.28, home 400/0.35). "
                        "The defaults are the demo-speed values; this is the one "
                        "flag to reach for if the faster close starts reporting "
                        "contact it should not, with nothing to edit under pressure.")
    p.add_argument("--jaw-step-deg", type=float, default=None,
                   help="per-command S6 travel limit in degrees (default 8, the same "
                        "as the arm). 12 roughly halves the close's iteration count, "
                        "but doubles the squeeze past contact on a SLIPPING object "
                        "(16->33 deg measured) because the force ceiling flattens the "
                        "step the slip detector measures against -- so it is refused "
                        "together with --jaw-track-fraction, which the one-command "
                        "launcher turns on by default. Never above --jaw-max-lag-deg.")
    p.add_argument("--bus-quiet-ms", type=float, default=None,
                   help="Gap between a servo write and the next read, ms (default 20). "
                        "Raise it if the [Bus] tally shows persistent read failures: "
                        "the six servos share one half-duplex line and a read issued "
                        "too soon after a write gets no answer.")
    p.add_argument("--port", type=str, default="/dev/myserial",
                   help="Serial port for the Rosmaster board (stable udev symlink to "
                        "the ch341 device; default /dev/myserial)")
    return p.parse_args()


def main():
    args = parse_args()

    cfg = DeployConfig(
        control_hz=args.hz,
        default_obj_pos=(args.obj_x, args.obj_y, args.obj_z),
        default_obj_height=args.object_height,
        serial_port=args.port,
        latch_obj=args.latch_obj,
    )
    if args.model:
        cfg.model_path = args.model
    if args.vecnorm:
        cfg.vecnorm_path = args.vecnorm
    if args.contract:
        cfg.contract_name = args.contract
    if args.bus_quiet_ms is not None:
        cfg.bus_quiet_s = max(0.0, args.bus_quiet_ms / 1000.0)
    if args.slow_motion:
        (cfg.close_run_time_ms, cfg.close_settle_s) = (300, 0.30)
        (cfg.lift_run_time_ms, cfg.lift_settle_s) = (250, 0.28)
        (cfg.home_run_time_ms, cfg.home_settle_s) = (400, 0.35)
        cfg.jaw_max_delta_deg = cfg.max_delta_deg
        print("[Config] --slow-motion: pre-2026-09-09 scripted-move timing restored.")
    if args.jaw_step_deg is not None:
        cfg.jaw_max_delta_deg = float(args.jaw_step_deg)
    # Socket timing. DeployConfig is a plain dataclass, so a typo'd field name here
    # would silently create a dead attribute -- each of these four is a real field.
    if args.socket_port is not None:
        if not 1 <= args.socket_port <= 65535:
            print(f"[FATAL] --socket-port {args.socket_port} is not a valid TCP port.")
            return 2
        cfg.socket_port = args.socket_port
    if args.stale_timeout is not None:
        if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0.0:
            print("[FATAL] --stale-timeout must be finite and positive.")
            return 2
        cfg.detection_stale_timeout_sec = float(args.stale_timeout)
    if args.latch_wait is not None:
        if not math.isfinite(args.latch_wait) or args.latch_wait <= 0.0:
            print("[FATAL] --latch-wait must be finite and positive.")
            return 2
        cfg.latch_wait_sec = float(args.latch_wait)
    if args.pose_tol_deg is not None:
        # A large tolerance defeats the check it configures: at the C3 pose the
        # camera is near-vertical, so a few degrees of arm error moves the ground
        # footprint by centimetres.
        if not math.isfinite(args.pose_tol_deg) or not 0.0 < args.pose_tol_deg <= 10.0:
            print("[FATAL] --pose-tol-deg must be in (0, 10].")
            return 2
        cfg.detection_pose_tol_deg = float(args.pose_tol_deg)
    if args.entry_xy_mm is not None:
        entry_xy_mm = float(args.entry_xy_mm)
        training_gate_mm = DeployConfig.entry_xy_tol * 1000.0
        if (not math.isfinite(entry_xy_mm)
                or not 0.0 < entry_xy_mm <= training_gate_mm):
            print(f"[FATAL] --entry-xy-mm must be in (0, {training_gate_mm:.0f}]. "
                  "A larger value would permit closing outside the training gate.")
            return 2
        cfg.entry_xy_tol = entry_xy_mm / 1000.0
        print(f"[Init] Stage-0 close gate requires XY < {entry_xy_mm:.1f}mm "
              f"(training/default gate {training_gate_mm:.0f}mm).")
    if args.s6_stall_grasp_steps is not None:
        if not 2 <= args.s6_stall_grasp_steps <= 10:
            print("[FATAL] --s6-stall-grasp-steps must be in [2, 10].")
            return 2
        cfg.stage0_s6_stall_steps = int(args.s6_stall_grasp_steps)
        cfg.jaw_hold_bias_deg = 1.0
        print(f"[Init] Stage-0 S6 contact: {cfg.stage0_s6_stall_steps} consecutive "
              f"blocked readings confirm contact; hold bias "
              f"{cfg.jaw_hold_bias_deg:.0f} deg.")
    if args.jaw_track_fraction is not None:
        value = float(args.jaw_track_fraction)
        if not math.isfinite(value) or not 0.0 <= value < 1.0:
            print(f"[FATAL] --jaw-track-fraction {value} is outside [0, 1). "
                  "1.0 or more would call every close a contact immediately.")
            return 2
        cfg.jaw_contact_track_fraction = value
        if cfg.stage0_s6_stall_steps > 0 and value > 0.0:
            print(f"[Init] Stage-0 slow-tracking detector enabled: encoder must "
                  f"consume at least {value:.2f} of the prior outstanding S6 "
                  "command; otherwise the jaw parks and lifts.")
    if args.jaw_max_lag_deg is not None:
        value = float(args.jaw_max_lag_deg)
        if not math.isfinite(value) or value < 0.0 or value > 90.0:
            print(f"[FATAL] --jaw-max-lag-deg {value} is outside [0, 90].")
            return 2
        if 0.0 < value < cfg.jaw_hold_bias_deg:
            print(f"[FATAL] --jaw-max-lag-deg {value} is below the "
                  f"{cfg.jaw_hold_bias_deg} deg hold bias, so the ceiling would "
                  "throttle the deliberate grip itself.")
            return 2
        cfg.jaw_max_lag_deg = value
    if args.floor_finger_error_mm:
        # Bounded on purpose: this shifts the ONLY thing standing between the policy
        # and the floor, and a typo'd 167 would hand the guard 16.7 cm of imaginary
        # clearance. The measured value is 16.7 mm; 30 leaves room to be wrong.
        err_mm = float(args.floor_finger_error_mm)
        if not (0.0 <= err_mm <= 30.0):
            print(f"[FATAL] --floor-finger-error-mm {err_mm} is outside [0, 30]. "
                  f"The measured URDF finger error is 16.7mm.")
            return 2
        cfg.floor_finger_error_m = err_mm / 1000.0
        print(f"[Init] Floor guard uses finger-pad correction +{err_mm:.1f}mm. "
              f"The guard now believes the pads are where the ruler says they are — "
              f"verify clearance visually on the first descent.")
    tcp_error_mm = float(args.tcp_forward_error_mm)
    if (not math.isfinite(tcp_error_mm)
            or not 0.0 <= tcp_error_mm <= 20.0):
        print(f"[FATAL] --tcp-forward-error-mm {tcp_error_mm!r} is outside [0, 20].")
        return 2
    cfg.tcp_forward_error_m = tcp_error_mm / 1000.0
    if tcp_error_mm:
        print(f"[Init] TCP landing correction: FK/control TCP X {tcp_error_mm:.1f}mm "
              "toward the robot; policy and close gate will travel that much farther "
              "in base +X. Object coordinates and floor FK stay unchanged.")

    # __post_init__ ran on the defaults, and every block above has been mutating the
    # config since. Re-run the same checks over the finished thing -- placed AFTER
    # the last mutation on purpose: --jaw-track-fraction and --jaw-max-lag-deg are
    # both set below where this used to sit, so an earlier call validated a config
    # that no longer existed and let the slip-detector clash through unnoticed.
    try:
        cfg.__post_init__()
    except ValueError as e:
        print(f"[FATAL] {e}")
        return 2

    # --real --socket hands the arm a target computed by another process, in a frame
    # this script cannot verify, from a camera that moves with the arm. Both flags
    # below are acknowledgements, not switches: neither changes any geometry, they
    # just refuse to let the failure be silent.
    if args.real and args.socket and not args.i_confirm_external_frame:
        print("[FATAL] --real --socket requires --i-confirm-external-frame. Confirm the "
              "sender emits PPO/URDF base_link XYZ, not raw camera coordinates — a wrong "
              "frame is undetectable downstream and grasps confidently in the wrong place.")
        return 2
    if args.real and args.socket and not args.latch_obj:
        print("[FATAL] --real --socket requires --latch-obj. The arm camera is mounted on "
              "arm_link4 and moves with the arm, so its fixed height/pitch distance model "
              "only holds at the home pose. Without latching, the target drifts under the "
              "policy mid-approach.")
        return 2

    # Both --real gates run before the model is loaded and before the serial port is
    # opened, so a refusal costs nothing and cannot half-move the arm. Dry-run is
    # deliberately untouched by either: verification must stay free.
    if args.real and not _release_gate_ok(
            cfg, unlock=args.unlock_candidate_real,
            manifest_path=args.release_manifest):
        return 3

    # --real without a driver used to run to completion printing angles, which reads
    # exactly like a successful grasp. Refuse at the entry point, before anything is
    # loaded, so the failure is unmistakable and costs nothing.
    if args.real and not _ROSMASTER_AVAILABLE:
        print("[FATAL] --real requires Rosmaster_Lib, but the driver is not importable.")
        print("        On the Jetson, localize it beside the deployment scripts:")
        print("          cp -r /usr/local/lib/python3.6/dist-packages/Rosmaster_Lib "
              "~/Documents/deploy_jetson2/grasp/")
        print("        Then confirm the board is free (no rosmaster_main.py, no ROS")
        print("        chassis driver, no port 7000 motor server holding the serial).")
        return 2

    controller = GraspController(cfg, real_servo=args.real, use_socket=args.socket)
    try:
        controller.run(max_steps=args.max_steps)
    except KeyboardInterrupt:
        print("\n[Interrupted] Emergency stop.")
        controller.servo.emergency_stop()
    finally:
        # Here rather than at the end of run(), so the tallies survive every exit
        # path — a stage-1 abort, a safety stop and Ctrl+C all return early from
        # run(). The guard tally is the running hardware evidence for the 8 mm
        # margin gate; {} just means no policy steps executed.
        print(f"[Bus] {controller.servo.read_failure_summary()}")
        tally = getattr(controller, "_guard_tally", None)
        if tally:
            print(f"[Guard] policy-step floor guard: {dict(tally)}")
        controller.close()


if __name__ == "__main__":
    sys.exit(main())
