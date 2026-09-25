# Codebase Classification

ACTIVE 表示現行 caller/操作入口或支援資產，**不表示 hardware-approved**。LEGACY 是保留的舊路徑；TEST 包括會動硬體的診斷程式，不能用檔名判定可安全執行；EXPERIMENTAL 是未接上正式整合的候選或本機工作。DEAD 需要證明沒有動態/手動使用，本輪不以 graph 零結果刪檔。

本表覆蓋主 checkout 的187個文字檔（105 Python）；另253個文字檔位於ignored `.claude`資料（主要是三套歷史source snapshots），完整路徑/sha256/AST symbols/imports/calls在 [source_inventory.json](evidence/source_inventory.json)。snapshot中的153 Python亦通過靜態parse，**不納入現行入口**。未追蹤檔不等於已合併。二進位模型/mesh不是Python程式，不以AST分類；模型身份由manifest和測試確認。

讀取層級：所有列入inventory的文字檔完整讀取並作機械化盤點；主線控制/安全/ROS/影像/入口有函式級追蹤與人工深讀。歷史快照與文書生成器以結構、差異與用途辨識為主，**不宣稱每份歷史副本都完成逐行獨立語意審查**。這個範圍限制不能被「258 Python parse passed」掩蓋。

## 個別檔案

| File | Classification | Usage / evidence basis | Git |
|---|---|---|---|
| [.gitignore](../../../.gitignore) | ACTIVE | 配置/輔助資產；沒有足夠證據宣稱DEAD | tracked |
| [AGENTS.md](../../../AGENTS.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [CLAUDE.md](../../../CLAUDE.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [INDEX.md](../../../INDEX.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [README.md](../../../README.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [README.txt](../../../README.txt) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | untracked / ignored |
| [SUMMER_MEETING_PPT.md](../../../SUMMER_MEETING_PPT.md) | EXPERIMENTAL | 本機報告/生成器，不是機器人runtime；保留使用者未提交工作 | untracked / ignored |
| [SUMMER_RESULTS_REPORT.md](../../../SUMMER_RESULTS_REPORT.md) | EXPERIMENTAL | 本機報告/生成器，不是機器人runtime；保留使用者未提交工作 | untracked / ignored |
| [TROUBLESHOOTING.md](../../../TROUBLESHOOTING.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [progress.md](../../../progress.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [requirements-ci.txt](../../../requirements-ci.txt) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [startup_device_check.py](../../../startup_device_check.py) | TEST | 裝置preflight入口；非純離線unit test | tracked |
| [stream_cam.py](../../../stream_cam.py) | ACTIVE | 被文件/操作流程使用的standalone camera服務；非ROS driver | tracked |
| [.github/workflows/tests.yml](../../../.github/workflows/tests.yml) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [detection/arm_cam.py](../../../detection/arm_cam.py) | LEGACY | 較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入 | tracked |
| [detection/arm_center_setmotor.py](../../../detection/arm_center_setmotor.py) | LEGACY | 較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入 | tracked |
| [detection/nav_arm_cam.py](../../../detection/nav_arm_cam.py) | LEGACY | 較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入 | tracked |
| [detection/rear_cam_sam2_publisher.py](../../../detection/rear_cam_sam2_publisher.py) | ACTIVE | 可選offboard視覺publisher；不直接持有motor | tracked |
| [detection/requirements_detection.txt](../../../detection/requirements_detection.txt) | LEGACY | 較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入 | tracked |
| [detection/calibration/action_speed_calibration_result.json](../../../detection/calibration/action_speed_calibration_result.json) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/calibration/calibrate_arm_camera_theta.py](../../../detection/calibration/calibrate_arm_camera_theta.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/calibration/calibrate_intrinsics.py](../../../detection/calibration/calibrate_intrinsics.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/calibration/calibrate_mecanum_setmotor.py](../../../detection/calibration/calibrate_mecanum_setmotor.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/calibration/calibrate_rearcam.py](../../../detection/calibration/calibrate_rearcam.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/calibration/calibrate_vy_only.py](../../../detection/calibration/calibrate_vy_only.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/calibration/calibrate_wx_only.py](../../../detection/calibration/calibrate_wx_only.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/calibration/wz_90deg_calibration_result.json](../../../detection/calibration/wz_90deg_calibration_result.json) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/armcam_photoshot.py](../../../detection/debug_tools/armcam_photoshot.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/check_rl_model.py](../../../detection/debug_tools/check_rl_model.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/detect_video.py](../../../detection/debug_tools/detect_video.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/multiple_ball_detect.py](../../../detection/debug_tools/multiple_ball_detect.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/rear_cam_autophoto.py](../../../detection/debug_tools/rear_cam_autophoto.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/snapshot_sim.py](../../../detection/debug_tools/snapshot_sim.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/test.py](../../../detection/debug_tools/test.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/train_yolo.py](../../../detection/debug_tools/train_yolo.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/debug_tools/yolo_test.py](../../../detection/debug_tools/yolo_test.py) | TEST | 校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe | tracked |
| [detection/models/README.roboflow.txt](../../../detection/models/README.roboflow.txt) | LEGACY | 較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入 | tracked |
| [detection/models/data.yaml](../../../detection/models/data.yaml) | LEGACY | 較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入 | tracked |
| [detection/rear_nav/rear_to_arm_blind_handoff.py](../../../detection/rear_nav/rear_to_arm_blind_handoff.py) | LEGACY | 較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入 | tracked |
| [docs/calibration/CALIBRATION_PLAN.md](../../../docs/calibration/CALIBRATION_PLAN.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/calibration/CAMERA_CALIBRATION_PARAMETERS.md](../../../docs/calibration/CAMERA_CALIBRATION_PARAMETERS.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/calibration/arm_pose.md](../../../docs/calibration/arm_pose.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/calibration/c3_calib_real_20260802.json](../../../docs/calibration/c3_calib_real_20260802.json) | ACTIVE | 配置/輔助資產；沒有足夠證據宣稱DEAD | tracked |
| [docs/calibration/e1_grasp_home_points_20260829.json](../../../docs/calibration/e1_grasp_home_points_20260829.json) | ACTIVE | 配置/輔助資產；沒有足夠證據宣稱DEAD | tracked |
| [docs/handoff/DESKTOP_AI_MODEL_EXPORT_PROMPT.md](../../../docs/handoff/DESKTOP_AI_MODEL_EXPORT_PROMPT.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/handoff/GRASP_HOME_FOV_RETRAINING_HANDOFF.md](../../../docs/handoff/GRASP_HOME_FOV_RETRAINING_HANDOFF.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/handoff/HANDOFF.md](../../../docs/handoff/HANDOFF.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/handoff/HANDOFF_VERIFY_2026-08-04.md](../../../docs/handoff/HANDOFF_VERIFY_2026-08-04.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/handoff/JETSON_RECOVERY_BOOT_TEST_HANDOFF_2026-07-18.md](../../../docs/handoff/JETSON_RECOVERY_BOOT_TEST_HANDOFF_2026-07-18.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/images/session.json](../../../docs/images/session.json) | ACTIVE | 配置/輔助資產；沒有足夠證據宣稱DEAD | tracked |
| [docs/operations/JETSON_DRYRUN_CHECKLIST.md](../../../docs/operations/JETSON_DRYRUN_CHECKLIST.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/operations/MODE_B_TEST_PLAN.md](../../../docs/operations/MODE_B_TEST_PLAN.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/operations/STARTUP_DEVICE_CHECK.md](../../../docs/operations/STARTUP_DEVICE_CHECK.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/planning/GRASP_TRAINING_REQUIREMENTS_2026-07-27.md](../../../docs/planning/GRASP_TRAINING_REQUIREMENTS_2026-07-27.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/planning/MISSION_PLAN.md](../../../docs/planning/MISSION_PLAN.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/planning/MODEL_RELEASE_WORKFLOW.md](../../../docs/planning/MODEL_RELEASE_WORKFLOW.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [docs/planning/PROJECT_EXECUTION_ROADMAP_2026-08-31.md](../../../docs/planning/PROJECT_EXECUTION_ROADMAP_2026-08-31.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | untracked / ignored |
| [docs/planning/V21_DEPLOY_REVIEW_AND_PLAN_2026-07-30.md](../../../docs/planning/V21_DEPLOY_REVIEW_AND_PLAN_2026-07-30.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [document/SRS_iGibson_v2.md](../../../document/SRS_iGibson_v2.md) | EXPERIMENTAL | 本機報告/生成器，不是機器人runtime；保留使用者未提交工作 | untracked / ignored |
| [document/build_srs.py](../../../document/build_srs.py) | EXPERIMENTAL | 本機報告/生成器，不是機器人runtime；保留使用者未提交工作 | untracked / ignored |
| [grasp/Rosmaster_Lib_reference.py](../../../grasp/Rosmaster_Lib_reference.py) | LEGACY | driver參考源碼；runtime import的是外部Rosmaster_Lib，不是此檔 | tracked |
| [grasp/fk_test.py](../../../grasp/fk_test.py) | TEST | FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式 | tracked |
| [grasp/joint_direction_calibration.py](../../../grasp/joint_direction_calibration.py) | TEST | FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式 | tracked |
| [grasp/move_arm.py](../../../grasp/move_arm.py) | TEST | FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式 | tracked |
| [grasp/requirements_jetson.txt](../../../grasp/requirements_jetson.txt) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [grasp/servo_test.py](../../../grasp/servo_test.py) | TEST | FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式 | tracked |
| [grasp/workspace_scan.py](../../../grasp/workspace_scan.py) | TEST | FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式 | tracked |
| [grasp/x3plus_real_grasp.py](../../../grasp/x3plus_real_grasp.py) | LEGACY | v17 absolute備援；model_tools wrapper仍會subprocess呼叫，非DEAD | tracked |
| [grasp/deploy_v23/DEPLOY_V23_MANIFEST.md](../../../grasp/deploy_v23/DEPLOY_V23_MANIFEST.md) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [grasp/deploy_v23/README.md](../../../grasp/deploy_v23/README.md) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [grasp/deploy_v23/action_execution_v21.py](../../../grasp/deploy_v23/action_execution_v21.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [grasp/deploy_v23/deploy_contract.py](../../../grasp/deploy_v23/deploy_contract.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [grasp/deploy_v23/x3plus_real_grasp.py](../../../grasp/deploy_v23/x3plus_real_grasp.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [grasp/deploy_v23/x3plus/__init__.py](../../../grasp/deploy_v23/x3plus/__init__.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [grasp/deploy_v23/x3plus/yahboomcar.urdf](../../../grasp/deploy_v23/x3plus/yahboomcar.urdf) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [grasp/trained_6d_models_v18/README.md](../../../grasp/trained_6d_models_v18/README.md) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [grasp/trained_6d_models_v18/manifest.json](../../../grasp/trained_6d_models_v18/manifest.json) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [grasp/trained_6d_models_v18/verify_v18.py](../../../grasp/trained_6d_models_v18/verify_v18.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [grasp/v21/HANDOFF.md](../../../grasp/v21/HANDOFF.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [grasp/v21/README.md](../../../grasp/v21/README.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [grasp/v21/action_execution_v21.py](../../../grasp/v21/action_execution_v21.py) | ACTIVE | v21 controller使用的incremental safety action execution | tracked |
| [grasp/v21/bus_probe.py](../../../grasp/v21/bus_probe.py) | TEST | FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式 | tracked |
| [grasp/v21/deploy_contract.py](../../../grasp/v21/deploy_contract.py) | ACTIVE | v21 controller與tests匯入的模型契約 | tracked |
| [grasp/v21/jetson_one_command_grasp.py](../../../grasp/v21/jetson_one_command_grasp.py) | ACTIVE | v21正式獨立抓取候選launcher；不是完整mission | tracked |
| [grasp/v21/jetson_verify.sh](../../../grasp/v21/jetson_verify.sh) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [grasp/v21/manifest.json](../../../grasp/v21/manifest.json) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [grasp/v21/pose_check.py](../../../grasp/v21/pose_check.py) | TEST | FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式 | tracked |
| [grasp/v21/test_deploy_controller.py](../../../grasp/v21/test_deploy_controller.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v21/test_deploy_floor_guard.py](../../../grasp/v21/test_deploy_floor_guard.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v21/test_one_command_launcher.py](../../../grasp/v21/test_one_command_launcher.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v21/test_servo_read.py](../../../grasp/v21/test_servo_read.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v21/x3plus_real_grasp.py](../../../grasp/v21/x3plus_real_grasp.py) | ACTIVE | Mission動態載入 + v21 standalone controller | tracked |
| [grasp/v21/evidence/v21_formal_claim.json](../../../grasp/v21/evidence/v21_formal_claim.json) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [grasp/v21/evidence/v21_formal_evaluation_seed90210.json](../../../grasp/v21/evidence/v21_formal_evaluation_seed90210.json) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [grasp/v21/evidence/v21_seed816_training_metadata.json](../../../grasp/v21/evidence/v21_seed816_training_metadata.json) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [grasp/v21/evidence/v21_selection_lock.json](../../../grasp/v21/evidence/v21_selection_lock.json) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [grasp/v21/reference/README.md](../../../grasp/v21/reference/README.md) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | tracked |
| [grasp/v21/reference/x3plus_real_grasp.release_branch.py](../../../grasp/v21/reference/x3plus_real_grasp.release_branch.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | tracked |
| [grasp/v23/README.md](../../../grasp/v23/README.md) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/action_execution_v21.py](../../../grasp/v23/action_execution_v21.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/bus_probe.py](../../../grasp/v23/bus_probe.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/deploy_contract.py](../../../grasp/v23/deploy_contract.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/jetson_one_command_grasp.py](../../../grasp/v23/jetson_one_command_grasp.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/jetson_verify.sh](../../../grasp/v23/jetson_verify.sh) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/manifest.json](../../../grasp/v23/manifest.json) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/pose_check.py](../../../grasp/v23/pose_check.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/pose_explorer.py](../../../grasp/v23/pose_explorer.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/test_deploy_controller.py](../../../grasp/v23/test_deploy_controller.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v23/test_deploy_floor_guard.py](../../../grasp/v23/test_deploy_floor_guard.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v23/test_one_command_launcher.py](../../../grasp/v23/test_one_command_launcher.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v23/test_servo_read.py](../../../grasp/v23/test_servo_read.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v23/test_three_pose_scan.py](../../../grasp/v23/test_three_pose_scan.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [grasp/v23/three_pose_scan.json](../../../grasp/v23/three_pose_scan.json) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/three_pose_scan.py](../../../grasp/v23/three_pose_scan.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/x3plus_real_grasp.py](../../../grasp/v23/x3plus_real_grasp.py) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/handoff/DEPLOY_V23_MANIFEST.md](../../../grasp/v23/handoff/DEPLOY_V23_MANIFEST.md) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/handoff/README.md](../../../grasp/v23/handoff/README.md) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/v23/handoff/README_as_delivered.md](../../../grasp/v23/handoff/README_as_delivered.md) | EXPERIMENTAL | v23 candidate獨立路徑；有實測記錄但mission loader仍選v21 | tracked |
| [grasp/x3plus/__init__.py](../../../grasp/x3plus/__init__.py) | ACTIVE | ACTIVE FK/URDF asset package marker；不是hardware test | tracked |
| [grasp/x3plus/yahboomcar.urdf](../../../grasp/x3plus/yahboomcar.urdf) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [integration/DEPLOY_VERIFY.md](../../../integration/DEPLOY_VERIFY.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [integration/KALMAN_FILTER_DESIGN.md](../../../integration/KALMAN_FILTER_DESIGN.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [integration/MISSION.md](../../../integration/MISSION.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [integration/NAV_RL.md](../../../integration/NAV_RL.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [integration/README.md](../../../integration/README.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [integration/SETMOTOR_ODOM_INTEGRATION.md](../../../integration/SETMOTOR_ODOM_INTEGRATION.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [integration/TEST_PLAN.md](../../../integration/TEST_PLAN.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [integration/arm_cam_geometry.py](../../../integration/arm_cam_geometry.py) | ACTIVE | Bridge/nav/calibration共用幾何與pose身份 | tracked |
| [integration/capture_arm_cam_obs.py](../../../integration/capture_arm_cam_obs.py) | TEST | 部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware | tracked |
| [integration/feedback_odom.py](../../../integration/feedback_odom.py) | ACTIVE | Mission OdomPublisher使用；board-motion feedback積分 | tracked |
| [integration/grasp_home_homography.py](../../../integration/grasp_home_homography.py) | ACTIVE | Bridge/standalone校正投影；mission尚未完整接入 | tracked |
| [integration/grasp_home_homography_e1.json](../../../integration/grasp_home_homography_e1.json) | ACTIVE | 配置/輔助資產；沒有足夠證據宣稱DEAD | tracked |
| [integration/map_goal_provider.py](../../../integration/map_goal_provider.py) | ACTIVE | Mission使用；external route與AMCL→goal | tracked |
| [integration/mission_fsm.py](../../../integration/mission_fsm.py) | ACTIVE | MissionRunner匯入；pure state transitions | tracked |
| [integration/mission_pipeline.py](../../../integration/mission_pipeline.py) | ACTIVE | 正式整合候選入口；build_and_run直接組裝所有adapter | tracked |
| [integration/mission_status.py](../../../integration/mission_status.py) | ACTIVE | Mission/UI間狀態與控制協定 | tracked |
| [integration/nav_rl.py](../../../integration/nav_rl.py) | ACTIVE | Mission/RLNavigator使用；55D PPO與LaserScan adapter | tracked |
| [integration/nav_rl_grasp_pipeline.py](../../../integration/nav_rl_grasp_pipeline.py) | ACTIVE | ACTIVE類別庫；standalone Mode C為EXPERIMENTAL且有B18 | tracked |
| [integration/preflight.py](../../../integration/preflight.py) | TEST | 部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware | tracked |
| [integration/ros_io.py](../../../integration/ros_io.py) | ACTIVE | Mission使用；rosbridge odom/TF/pose/status I/O | tracked |
| [integration/smoke_mode_b.py](../../../integration/smoke_mode_b.py) | TEST | 部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware | tracked |
| [integration/solve_arm_cam_extrinsics.py](../../../integration/solve_arm_cam_extrinsics.py) | TEST | 部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware | tracked |
| [integration/solve_cam_to_base.py](../../../integration/solve_cam_to_base.py) | TEST | 部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware | tracked |
| [integration/trash_target.py](../../../integration/trash_target.py) | ACTIVE | Mission可選offboard target adapter | tracked |
| [integration/verify_camera_grasp_frame.py](../../../integration/verify_camera_grasp_frame.py) | TEST | 部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware | tracked |
| [integration/verify_x3plus_deploy.py](../../../integration/verify_x3plus_deploy.py) | TEST | 部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware | tracked |
| [integration/vision_grasp_bridge.py](../../../integration/vision_grasp_bridge.py) | ACTIVE | Mode B與one-command launcher使用；pose-stamped TCP sender | tracked |
| [integration/vision_grasp_pipeline.py](../../../integration/vision_grasp_pipeline.py) | ACTIVE | ACTIVE Navigator/base helpers；standalone legacy orchestration已限制real | tracked |
| [integration/nav_best_model/P2_J_collision_guard_rewardparams.json](../../../integration/nav_best_model/P2_J_collision_guard_rewardparams.json) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [integration/nav_best_model/doorway_ft_training_config.json](../../../integration/nav_best_model/doorway_ft_training_config.json) | ACTIVE | 模型/幾何/契約資產；需與實際consumer和版本配對 | tracked |
| [model_tools/__init__.py](../../../model_tools/__init__.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [model_tools/export_model_package.py](../../../model_tools/export_model_package.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [model_tools/fetch_model_release.py](../../../model_tools/fetch_model_release.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [model_tools/grasp_v18_manifest.template.json](../../../model_tools/grasp_v18_manifest.template.json) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [model_tools/install_model_package.py](../../../model_tools/install_model_package.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [model_tools/model_package.py](../../../model_tools/model_package.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [model_tools/run_grasp_package.py](../../../model_tools/run_grasp_package.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [model_tools/verify_model_package.py](../../../model_tools/verify_model_package.py) | LEGACY | 舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher | tracked |
| [tests/test_grasp_home_homography.py](../../../tests/test_grasp_home_homography.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [tests/test_mission_end_to_end.py](../../../tests/test_mission_end_to_end.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [tests/test_model_package.py](../../../tests/test_model_package.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [tests/test_safety_guards.py](../../../tests/test_safety_guards.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [tests/test_stream_cam_capture.py](../../../tests/test_stream_cam_capture.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [tests/test_trash_target.py](../../../tests/test_trash_target.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [tests/test_vision_grasp_bridge_pose.py](../../../tests/test_vision_grasp_bridge_pose.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [training/action_execution_v21.py](../../../training/action_execution_v21.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [training/deploy_contract.py](../../../training/deploy_contract.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [training/test_deploy_controller.py](../../../training/test_deploy_controller.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [training/x3plus_real_grasp.py](../../../training/x3plus_real_grasp.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [training/x3plus/__init__.py](../../../training/x3plus/__init__.py) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [training/x3plus/yahboomcar.urdf](../../../training/x3plus/yahboomcar.urdf) | LEGACY | 交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行 | untracked / ignored |
| [ui/README.md](../../../ui/README.md) | ACTIVE | 文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀 | tracked |
| [ui/install_desktop_shortcut.sh](../../../ui/install_desktop_shortcut.sh) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [ui/jetson_check.sh](../../../ui/jetson_check.sh) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [ui/launch_ui.sh](../../../ui/launch_ui.sh) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [ui/launcher.py](../../../ui/launcher.py) | ACTIVE | launch_ui.sh真正exec入口 | tracked |
| [ui/make_qr.py](../../../ui/make_qr.py) | ACTIVE | UI QR helper | tracked |
| [ui/prototype.html](../../../ui/prototype.html) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [ui/server.py](../../../ui/server.py) | ACTIVE | Console entry與child管理；B09/B19尚需修 | tracked |
| [ui/test_server.py](../../../ui/test_server.py) | TEST | 離線suite候選；已執行清單以offline_test_results.json為準 | tracked |
| [ui/static/app.js](../../../ui/static/app.js) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [ui/static/index.html](../../../ui/static/index.html) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [ui/static/style.css](../../../ui/static/style.css) | ACTIVE | UI資產/啟動環境工具或CI；不是獨立motor controller | tracked |
| [yolo/sugarbox_yolo11_release/README.md](../../../yolo/sugarbox_yolo11_release/README.md) | EXPERIMENTAL | 獨立模型demo/資料配置；mission實際載入detection/models/best.pt | tracked |
| [yolo/sugarbox_yolo11_release/data.yaml](../../../yolo/sugarbox_yolo11_release/data.yaml) | EXPERIMENTAL | 獨立模型demo/資料配置；mission實際載入detection/models/best.pt | tracked |
| [yolo/sugarbox_yolo11_release/predict_demo.py](../../../yolo/sugarbox_yolo11_release/predict_demo.py) | EXPERIMENTAL | 獨立模型demo/資料配置；mission實際載入detection/models/best.pt | tracked |
| [yolo/sugarbox_yolo11_release/webcam_demo.py](../../../yolo/sugarbox_yolo11_release/webcam_demo.py) | EXPERIMENTAL | 獨立模型demo/資料配置；mission實際載入detection/models/best.pt | tracked |

## DEAD候選與不能誤判的檔案

- `MapGoalProvider.in_forbidden()` 沒有在mission行進迴圈找到consumer：屬於尚未接上的能力／dead-call candidate，不代表整個map_goal_provider.py可刪。
- `grasp/x3plus_real_grasp.py` 有model_tools subprocess入口，因此不能照「沒有import」分類DEAD。
- v21/v23的測試、entrypoint、dynamic import、socket receiver與utility均可能沒有一般inbound graph edge。
- `.claude/worktrees/{grasp-v18-arm-c3,grasp-v21-candidate,settle-before-close}/x3plus` 是歷史source snapshots；git worktree list本輪只列根目錄。列LEGACY snapshot，保留，不當成另一套正在執行的deployment。
- `grasp/deploy_v23/` 為使用者未追蹤的handoff；其controller與正式 `grasp/v23/`不同且帶舊Arm_Lib，禁止以檔名替換。

## 文件與log的使用規則

CLAUDE為規格入口，INDEX為導覽；progress/manifest/handoff提供歷史實測宣稱，與目前code逐項比對。v21/HANDOFF不是docs/handoff/HANDOFF。Git曾保存的Mode B log只有bridge視覺傳輸，不能證明目前完整patrol→grasp→bin任務已成功。binary模型成對hash不等於模型行為已通過實機驗收。
