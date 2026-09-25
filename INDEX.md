# X3Plus Code Index

Quick map for finding the right code without scanning the whole project.

## Main Entry Points

| Task | File | What to inspect first |
|---|---|---|
| **PPO grasp deployment** | `grasp/v21/x3plus_real_grasp.py` | `DeployConfig`, `FloorGuard`, `GraspController.run()` |
| v24 E1 replacement candidate | `grasp/v24/run_candidate.py` | locked artifact tuple; delegates to hardened v23 controller |
| Full mission (patrol → grasp → bin) | `integration/mission_pipeline.py` | `MissionRunner`, the tick loop, `_await_operator()` |
| Mission state machine | `integration/mission_fsm.py` | `State`, `Action`, `step()` — 21 states, `--diagram` |
| Operator console | `ui/server.py` | request routing, SSE stream, the two `--allow-real` gates |
| Offboard SAM2 detector | `detection/rear_cam_sam2_publisher.py` | mask bottom point, `GroundProjector`, the published payload |
| Offboard target adapter | `integration/trash_target.py` | `to_rear_detection` — **the left/right sign flip**, staleness, fail-closed |
| One-process navigation + grasp | `integration/vision_grasp_pipeline.py` | `Navigator`, `run_pipeline()`, camera constants |
| RL navigation + grasp | `integration/nav_rl_grasp_pipeline.py` | `RLNavigator`, `_rl_navigate()`, shared Rosmaster device |
| TG30 ROS scan adapter | `integration/nav_rl.py` | `RosLaserScanSource`, `laser_scan_to_points()`, `make_lidar()` |
| set_motor/odom/ROS detailed plan | `integration/SETMOTOR_ODOM_INTEGRATION.md` | serial ownership, actuator contract, odom/TF gates |
| Dynamic Jetson host | `set_jetson_host.ps1` | set `X3PLUS_JETSON_HOST` once per PowerShell session; no source edits needed |
| Arm-camera TCP bridge | `integration/vision_grasp_bridge.py` | `estimate_distance()`, payload `{x,y,z,w,class}` |
| Camera/frame static check | `integration/verify_camera_grasp_frame.py` | URDF `mono_link` pose vs vision constants |
| Deployment preflight | `integration/verify_x3plus_deploy.py` | dependency and real-run checks |

## Data Flow

```text
arm camera / rear camera
  -> YOLO detection
  -> bbox geometry: distance, lateral offset, width
  -> object target `{x, y, z, w}`
  -> grasp controller object provider or TCP socket
  -> 28D PPO observation
  -> Rosmaster_Lib servo commands
```

Two supported integration modes:

| Mode | Command path | Notes |
|---|---|---|
| Unified pipeline | `integration/vision_grasp_pipeline.py` | One process owns Rosmaster for wheels and arm. Preferred full robot loop. |
| TCP bridge | `integration/vision_grasp_bridge.py` + `grasp/v21/x3plus_real_grasp.py --socket` | Separate vision sender to grasp receiver on port 5555. |

The `detection/rear_nav/` programs and older ROS `/cmd_vel` demos are retained
as calibration/history references. They are not compatible with the current
55D observation ordering or Phase-1/2 camera constants, and now require an
explicit `--real` before any non-stop motor command is allowed.

## Grasp Code Landmarks

In `grasp/v21/x3plus_real_grasp.py` — the current deployment script. The
identically-named file at `grasp/x3plus_real_grasp.py` is the v17 fallback: it
has most of the same class names but an **absolute** action contract, so read
carefully which one you are in. See CLAUDE.md before mixing weights.

| Code | Purpose |
|---|---|
| `DeployConfig` | Servo limits, home poses, socket config, thresholds, width grip config |
| `FloorGuard` | Preventive: clamps commands that would drive the jaw through the floor |
| `home_deg` | Navigation/cruise home pose |
| `grasp_home_deg` | PPO training initial pose before policy starts |
| `JointMapper` | sim radians <-> Rosmaster servo degrees, width -> S6 close angle |
| `ServoController` | Rosmaster_Lib wrapper, `move_to_home()`, `move_to_grasp_home()` |
| `FKComputer` | PyBullet/URDF FK for TCP pose |
| `DetectionReceiver` | TCP `{x,y,z,w}` receiver |
| `ObsBuilder.build()` | 28D observation, including `rel_pos = obj_pos - tcp_pos` |
| `GraspController.run()` | Stage 0/1/2 policy loop |

Important current defaults:

```text
home_deg       = (90, 140, 0, 0, 90, 30)   # navigation/cruise home
grasp_home_deg = (90, 32.704, 9.786, 32.704, 90, 30)   # PPO training home
S6 open=30 deg, closed=180 deg
arm_hw_invert = (False, False, False, False, False)
max_delta_deg = 8.0    # v21. The v17 fallback uses 3.0 -- do not mix them up.
```

## Vision And Coordinate Code

| File | Role |
|---|---|
| `detection/arm_cam.py` | Original arm-camera bbox -> distance/offset demo |
| `detection/nav_arm_cam.py` | Arm-camera navigation demo |
| `detection/rear_nav/rear_to_arm_blind_handoff.py` | Source of rear/arm navigation logic ported into pipeline |
| `detection/calibration/calibrate_arm_camera_theta.py` | Arm camera pitch/distance calibration |
| `detection/calibration/arm_camera_calibration_points.csv` | Existing arm camera calibration samples |
| `integration/verify_camera_grasp_frame.py` | Checks whether camera pose assumptions match URDF base frame |

Coordinate assumption to verify before trusting XY:

```text
PPO object frame = URDF/PyBullet base_link frame
vision obj_x = camera ground distance + camera/base X offset
vision obj_y = sign_y * lateral offset + camera/base Y offset
```

Current calibration status: `vision_grasp_bridge.py`、`vision_grasp_pipeline.py` 與
`detection/arm_cam.py` 已同步 2026-07-10 的手臂相機內參／俯角，且先 undistort bbox
底邊中點；Phase 3 已於 2026-07-16 完成四點實機複驗：`CAM_TO_BASE_X=0.1639m`、
`CAM_TO_BASE_Y=0.0331m`、`SIGN_Y=-1`，最大誤差 X=0.39cm、Y=0.33cm。
`z_offset=0.0488m` 已撤銷，因 FK TCP 是 arm_link5 慣性中心而非指間中心；瓶蓋 Z
先保留訓練預設 0.02m，於 Phase 4 實抓微調。

## Models And Assets

| Path | Purpose |
|---|---|
| `grasp/v21/models/candidate_v21_seed816_ckpt550000.zip` + `_vec.pkl` | **Current** grasp policy + VecNormalize. Contract `obs_28_incremental` |
| `grasp/v21/manifest.json` | Single source of truth: weight sha256, contract, hardware gates |
| `grasp/v24/models/*` + `grasp/v24/manifest.json` | E1 replacement candidate (230/235 delivered formal rows); new hash hardware gates still false |
| `grasp/trained_6d_models_v17/*.zip` `.pkl` | v17 fallback policy. Absolute contract — never feed these to v21 |
| `integration/nav_best_model/ppo_nav_281440_steps.zip` + `ppo_nav_vecnormalize_281440_steps.pkl` | Current nav baseline/default |
| `integration/nav_best_model/doorway_ft_final.zip` + `doorway_ft_final_vecnormalize.pkl` | Candidate nav pair; on-robot A/B required before default switch |
| `grasp/x3plus/yahboomcar.urdf` | FK/URDF frame source |
| `detection/models/best.pt` | Current YOLO model, single class `sugarbox` |
| `detection/models/data.yaml` | Class names plus the measured object geometry the grasp side needs |

## Calibration And Verification Commands

```bash
# Static frame sanity check, no hardware deps
python3 integration/verify_camera_grasp_frame.py

# Pure logic self-test, no camera/hardware
python3 integration/vision_grasp_pipeline.py --selftest
python3 integration/nav_rl.py --selftest

# TG30 /scan direction probe (TG.launch + rosbridge must already be running)
python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1

# Deployment preflight
python3 integration/verify_x3plus_deploy.py

# TCP bridge one detection
python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --once --show

# Pre-flight before any --real run: 138/37/641/29, dry-run offset, safety gate
./grasp/v21/jetson_verify.sh

# Same, for the unmerged v23/E1 stack: 163/37/641/62 + three-pose 89
./grasp/v23/jetson_verify.sh

# Static v24 pair/manifest/locked-launcher checks; does not load the model
python3 grasp/v24/test_candidate_package.py

# Measure the E1 pixel->base homography. The C3 one is wrong at E1 by
# construction, and swapping them raises no error -- only the filename differs.
python3 grasp/v23/jetson_one_command_grasp.py --calibrate

# LEFT/E1/RIGHT S1-only search: one E1 homography plus rigid yaw about arm_joint1.
# LEFT/RIGHT motion and >=2 held-out ruler points per side must be validated before
# --three-pose-scan passes preflight; scanner returns to E1 before PPO starts.
python3 grasp/v23/jetson_one_command_grasp.py --scan-calibrate-pose LEFT
python3 grasp/v23/jetson_one_command_grasp.py --scan-calibrate-pose RIGHT
python3 grasp/v23/jetson_one_command_grasp.py --three-pose-scan --allow-top-clipped

# Grasp with a calibrated external XYZ sender (v21). --width-grip and
# --latch-obj do not exist here: the jaw closes on contact rather than on a
# width-derived angle, and the caller's obj_provider owns target latching.
python3 grasp/v21/x3plus_real_grasp.py --real --socket --unlock-candidate-real \
  --model grasp/v21/models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm grasp/v21/models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental --i-confirm-external-frame
```

Useful pose/class arguments:

```bash
--nav-home-deg 90,140,0,0,90,30
--grasp-home-deg 90,140,0,0,90,30
--class-height sugarbox=0.065 --class-z sugarbox=0.0325
```

## Fast Search Keywords

```bash
rg -n "home_deg|grasp_home_deg|move_to_home|move_to_grasp_home" grasp integration
rg -n "DetectionReceiver|obj_provider|snapshot|latch|width_to_close" grasp integration
rg -n "CAM_TO_BASE|SIGN_Y|THETA_ARM|FIXED_THETA|estimate_distance|width_m" integration detection
rg -n "VecNormalize|norm_reward|training = False|shape=\\(28|shape=\\(6" grasp
rg -n "mono_link|mono_joint|arm_joint|base_link" grasp/x3plus/yahboomcar.urdf
```

## Common Gotchas

- Do not average camera-frame XY from different arm poses unless both detections are transformed into the same URDF/base frame first.
- Freeze the target for one PPO episode; do not continuously update object position while the arm camera is moving.
- If `x/y` looks consistently shifted, tune camera-to-base offsets (`CAM_TO_BASE_X/Y` or bridge `--cam-x/--cam-y`).
- If left/right is reversed, flip `SIGN_Y` or bridge `--sign-y`.
- With `--target-source offboard`, left/right is NOT `SIGN_Y`'s job. The publisher
  is left-positive and `estimate_offset_x` is right-positive, and
  `trash_target.to_rear_detection` is the single place that negates. Both are
  plain floats over the same range, so a wrong sign steers away from the object
  while looking entirely reasonable — `tests/test_trash_target.py` is what
  catches it.
- If distance scale changes with range, recalibrate camera `H/theta/FX/FY`.
- `grasp/x3plus_deploy_bridge.py` was deleted 2026-08-01 (never imported; its own
  docstring said so). Recover from git history if ever needed.
- The mission mainline implements feedback odometry: `integration/feedback_odom.py`
  integrates Rosmaster `get_motion_data()`, while `integration/ros_io.py` publishes
  `/odom_setmotor` and the `odom→base_footprint` TF.  The external ROS/TF bringup and
  single-owner deployment still have to be verified on the robot before a real run.
- The real LiDAR is YDLIDAR TG30. `/dev/rplidar` is only a udev alias; use the ROS `/scan`
  backend and `roslibpy`, not `rplidar-roboticia`.
- Never run a separate motor server and the unified grasp/navigation pipeline if both open `/dev/myserial`.
- Real integrated grasp is refused until Phase-3 values are supplied with
  `--cam-x/--cam-y/--sign-y` and acknowledged by `--i-confirm-camera-frame`.
