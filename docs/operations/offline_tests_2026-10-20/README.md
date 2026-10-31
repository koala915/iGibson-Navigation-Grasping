# Offline test handoff — 2026-10-20

The interrupted Claude work was continued against the existing, uncommitted working tree.
No robot connection, deployment, motor command, or physical grasp was performed.

## Final result

- 33 distinct Python suites/selftests passed after the corrections/retries below.
- 545 unittest cases, 1,784 custom checks, and 3 selftest suites.
- The six directly changed feature suites passed 243 unittest cases.
- Changed Python files parsed successfully; the Jetson permit helper also parsed with Python 3.6 syntax rules (syntax compatibility only).
- `bash -n deploy/ros/start_rear_approach_stream.sh` passed.
- `git diff --check` passed.
- All pre-existing modified/new feature files retained the same SHA256 values across testing.

## Test correction made in this continuation

`grasp/v23/test_travel_pose.py` still attempted a wheel creep in E1, which the current chassis implementation correctly refuses. The test now moves the fake board in travel pose, asserts that the simulated motion really occurred, stops it, switches the recorded pose to E1, and verifies the grasp still waits from the last wheel stop. All 51 checks pass. No controller/service implementation was changed by this continuation.

## Environment failures and resolution

The first run failed while creating/cleaning temporary fixtures under the Windows sandbox TEMP directory. Runs were repeated using `docs/operations/offline_tests_2026-10-20/tmp`; all six feature suites then passed without changing feature code.

The sandbox also denied loopback sockets in the chassis and UI tests (WinError 10013). Those two suites were repeated with permission to use localhost; the fake-board chassis passed 59 checks and the simulated UI passed 48 tests. Initial failures are preserved in `regression/results.json`; successful reruns are in `network/results.json`. `final_results.json` selects the final result for each suite.

## Limits

- POSIX-only checks were skipped by their existing Windows conditions (see suite logs); `deploy/systemd/test_helpers.sh` was not run in Linux.
- No live camera/ROS/model dry-run was performed.
- No hardware acceptance: startup latency, physical stop distance, missed-target collision avoidance, grasp, placement, and the complete repeated patrol cycle still need supervised robot tests.
- Existing unrelated working-tree edits were preserved; nothing was committed or pushed.

## Reproduce

```powershell
python docs/operations/offline_tests_2026-10-20/run_suites.py targeted
python docs/operations/offline_tests_2026-10-20/run_suites.py regression
python docs/operations/offline_tests_2026-10-20/run_suites.py network
```

The network round needs localhost access. The regression round contains those tests too. The runner sets TEMP/TMP only for child test processes. Each process runs the repository test script directly under Python 3.12.10; pytest was not required.

## Per-suite results

| Suite | Count | Result |
|---|---:|---|
| `grasp/v21/test_deploy_controller.py` | 138 checks | PASS |
| `grasp/v21/test_deploy_floor_guard.py` | 641 checks | PASS |
| `grasp/v21/test_servo_read.py` | 37 checks | PASS |
| `grasp/v23/test_chassis_server.py` | 59 checks | PASS |
| `grasp/v23/test_deploy_controller.py` | 180 checks | PASS |
| `grasp/v23/test_deploy_floor_guard.py` | 641 checks | PASS |
| `grasp/v23/test_servo_read.py` | 37 checks | PASS |
| `grasp/v23/test_travel_pose.py` | 51 checks | PASS |
| `grasp/v24/test_candidate_package.py` | 9 tests | PASS |
| `integration/mission_fsm.py --selftest` | selftest | PASS |
| `integration/mission_pipeline.py --selftest` | selftest | PASS |
| `integration/vision_grasp_pipeline.py --selftest` | selftest | PASS |
| `tests/test_g2_nav_client.py` | 6 tests | PASS |
| `tests/test_grasp_home_homography.py` | 9 tests | PASS |
| `tests/test_mission_end_to_end.py` | 29 tests | PASS |
| `tests/test_mission_loop.py` | 66 tests | PASS |
| `tests/test_model_package.py` | 11 tests | PASS |
| `tests/test_nav_safety.py` | 12 tests | PASS |
| `tests/test_patrol_vision_permit.py` | 6 tests | PASS |
| `tests/test_placement_dock.py` | 18 tests | PASS |
| `tests/test_rear_ground_homography.py` | 2 tests | PASS |
| `tests/test_safety_guards.py` | 73 tests | PASS |
| `tests/test_stream_cam_capture.py` | 7 tests | PASS |
| `tests/test_sugarbox_approach_grasp.py` | 56 tests | PASS |
| `tests/test_sugarbox_blind_finish.py` | 72 tests | PASS |
| `tests/test_sugarbox_ground_calibration.py` | 21 tests | PASS |
| `tests/test_sugarbox_lidar_geometry.py` | 15 tests | PASS |
| `tests/test_sugarbox_seed_target.py` | 25 tests | PASS |
| `tests/test_sugarbox_training_contract.py` | 13 tests | PASS |
| `tests/test_target_approach.py` | 6 tests | PASS |
| `tests/test_trash_target.py` | 21 tests | PASS |
| `tests/test_vision_grasp_bridge_pose.py` | 20 tests | PASS |
| `ui/test_server.py` | 48 tests | PASS |
