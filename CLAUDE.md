# X3Plus 專題 — Sim-to-Real 夾取部署

> **接手這個專案？先讀 [`docs/handoff/HANDOFF_2026-09-20.md`](docs/handoff/HANDOFF_2026-09-20.md)。**
> 那份講的是「現在做到哪裡、能跑什麼指令、卡在哪一步、下一步做什麼」，以及機器上的
> 檔案跟這個 repo 不同步這件事。本檔是規格，那份是現況，兩份都要看。

## 專題概述
將 PyBullet + PPO 訓練的 6D 夾取策略，部署到 Yahboom X3Plus 實體機器人。

- **硬體**：X3Plus（麥克納姆輪底盤 + 5-DOF 機械臂），Rosmaster 擴充板，Jetson Nano
- **框架**：Stable-Baselines3 PPO + PyBullet（FK only）
- **驅動**：Rosmaster_Lib（需本地化到 `grasp/` 目錄）
- **視覺**：YOLOv11（Ultralytics）手臂相機辨識 → 透過 TCP 5555 送物體座標+寬度給夾取端

---

## 檔案結構（四資料夾：夾取 / 辨識 / 整合 / 操作台）

程式碼分成 `grasp/`、`detection/`、`integration/`、`ui/` 四個資料夾，另有
`tests/`（跨模組回歸測試）與 `model_tools/`（模型打包與發佈）。

根目錄只留六份文件：`README.md`（對外門面）、`CLAUDE.md`（本檔）、`AGENTS.md`、
`INDEX.md`（程式碼導覽）、`TROUBLESHOOTING.md`（症狀→原因→解法→教訓）、
`progress.md`（進度與實機紀錄）。

其餘工作文件 2026-08-06 起收在 `docs/` 底下，**引用時要帶路徑**：

| 目錄 | 內容 |
|------|------|
| `docs/calibration/` | 校正計畫、相機參數、`arm_pose.md`、實測校正資料 |
| `docs/operations/` | `JETSON_DRYRUN_CHECKLIST.md`、開機設備檢查、模式 B 測試計畫 |
| `docs/handoff/` | 訓練端／部署端交接文件 |
| `docs/planning/` | 任務規劃、訓練需求、模型發佈流程 |
| `docs/images/` | README 用的操作台截圖 |

⚠️ `grasp/v21/HANDOFF.md` 是 v21 自己的交接文件，**不是** `docs/handoff/HANDOFF.md`。
兩者同名不同檔，改到時別搞混。

### `grasp/`（夾取）

> **★ 分支 `v23-grasp-test` 上另有 `grasp/v23/`（E1 高姿態），尚未合併。**
> main 上不存在。v23 只改了 grasp home（C3 → E1）與權重；契約、`deploy_contract.py`、
> `action_execution_v21.py`、URDF 都與 v21 逐位元組相同。2026-08-29 已完成 E1 實測
> homography、夾取中心高度與最小物體高度；2026-08-30 一鍵 E1 夾取在可辨識範圍內
> 實測 3/3 成功，且一份完整 log 已核對。正式 launcher 預設為 target +5 mm、finger
> correction 15 mm、jaw tracking 0.5、TCP +20 mm。motion envelope 與 dry-run 仍未過。
> E1 runtime 可用明確 opt-in `--allow-top-clipped` 放行「只碰上緣」的 bbox；
> 左／右／下緣與目標中心 homography 凸包 gate 不可繞過。2026-08-30 已加入 LEFT/E1/RIGHT
> S1-only 掃描框架（40 項純邏輯測試）：共用 E1 homography，再繞 S1 training-frame 軸心
> 旋轉 base XY。2026-09-20 **LEFT 首次在真機上走完並夾取成功**（單視角、
> `--accept-single-rotated-view`；S1 逼近時降到 71–74°，證明 yaw 正負號沒反向），
> 但沒有尺量點，**≤1 cm 映射驗證仍未完成、RIGHT 仍未走過**，兩個旗標維持 false、
> 正式掃描照樣在開硬體前拒絕。三視角的用意是擴大可見範圍（E1 單獨可用的縱深只有
> 約 3 cm），不是視角互相驗算，所以單視角可夾是刻意的。細節見 `grasp/v23/README.md`。
> 模式 A / B / C 在兩個分支上都還是接 v21。

> **2026-09-20 起 v23 有常駐服務：`grasp/v23/grasp_service.py` + `graspctl.py`。**
> 一鍵夾取原本 42.3 秒，其中 34 秒是每次重新載入 torch／PyBullet／PPO／YOLO；
> 常駐之後單次 **8.5 秒**。服務用 launcher 自己的 `build_ctrl_cmd()` 組參數，
> 安全閘一道不少，已驗證檔案零修改。systemd 兩個單元開機自動啟動。
> ⚠ **服務活著時獨占 `/dev/myserial` 與相機**，要跑 `move_arm.py`、`bus_probe.py`、
> `jetson_one_command_grasp.py` 前必須先 `sudo systemctl stop grasp-vision grasp-service`。
> 三姿態後備走 `graspscan.sh`（會自己停再還原服務）。

> **2026-09-07 已把 v24 E1 replacement 訓練包封裝成獨立候選。**
> `grasp/v24/run_candidate.py` 鎖定 model、VecNormalize、incremental contract 與 v24
> manifest，實際控制重用加固後的 v23 controller，沒有再 fork 一份硬體程式。
> 55 個 checksum 條目一致，交付 formal rows 重算 230/235；新 hash 尚無真機驗收，
> 且預註冊時間、環境版本與 deployment 8mm guard parity 待釐清。正式流程仍是 v21。
> 詳見 [接收審查](docs/planning/v24_intake_2026-09-07/INTAKE_REVIEW.md)。

> **★ 2026-08-01 起，整合模式的正式夾取流程是 `grasp/v21/`，不是根目錄那支。**
> v21 是第一套實機夾取成功版本（2026-07-31，3cm 物體，完整 log 見
> `grasp/v21/manifest.json` 的 `hardware_gates.first_real_grasp_2026_07_31`）。
> v23 分支另於 2026-08-30 完成 E1 單視角一鍵夾取 3/3，但尚未接入模式 A/B/C。
> 模式 A / 模式 C 兩支自走流程都已改接 v21（透過 `_load_grasp_module()`）。
> 根目錄的 `x3plus_real_grasp.py` + `trained_6d_models_v17/` 原封保留可直接跑，
> 根目錄那支 v17 已無任何程式匯入（`tests/test_safety_guards.py` 也在 2026-08-01
> 改成透過 `vgp._load_grasp_module()` 取得模組，測的就是 pipeline 實際載入的那一份）。

| 檔案 | 說明 |
|------|------|
| `v21/x3plus_real_grasp.py` | ★**現行**主部署腳本（incremental action、`gripper_center` TCP、接觸式夾持、預防式 FloorGuard） |
| `v21/models/candidate_v21_seed816_ckpt550000.zip` + `_vec.pkl` | ★現行 PPO 權重 + VecNormalize（**不可與 v17 混搭**，見下） |
| `v21/deploy_contract.py` / `action_execution_v21.py` | 觀測/動作契約定義；`obs_28_incremental` |
| `v21/manifest.json` | 權重 sha256、契約、硬體 gate、變更紀錄（單一事實來源） |
| `v21/jetson_one_command_grasp.py` | ★**一鍵辨識＋夾取**（C3 home → 相機辨識一次 → PPO 夾取）。自己開相機；`--calibrate` 產生 C3 homography 校正檔 |
| `v21/test_deploy_controller.py` / `test_servo_read.py` / `test_deploy_floor_guard.py` / `test_one_command_launcher.py` | 回歸測試，須 **138 / 37 / 641 / 29** 全過 |
| `v21/jetson_verify.sh` | 上機前一鍵前置檢查（138/37/641/29、dry-run `wrist_z_offset = 0.0564`、安全閘 exit 3） |
| `v21/bus_probe.py` / `pose_check.py` | 唯讀診斷：半雙工伺服匯流排讀取、姿態/FK 核對 |
| `v24/run_candidate.py` / `manifest.json` | v24 E1 replacement 候選入口；鎖定新 pair 並重用 v23 controller；未通過新 hash 真機 gate，不是正式主線 |
| `x3plus_real_grasp.py`（根目錄） | v17 舊版，**保留備援**。契約是 `obs_28_absolute`。無人匯入，只能手動執行 |
| `trained_6d_models_v17/*.zip` `.pkl` | v17 權重（配上面那支） |
| `x3plus/yahboomcar.urdf` + `meshes/` | PyBullet FK 用的 URDF 與模型（v17/v21 共用） |
| `Rosmaster_Lib/` | 硬體驅動（本地化到此，見下方指令） |
| `fk_test.py` / `servo_test.py` / `workspace_scan.py` / `joint_direction_calibration.py` | 測試/校正小工具 |

⚠️ **v21 權重絕對不可餵給根目錄那支 v17 腳本**（反之亦然）。
v21 是 incremental（`desired = current + action × 0.08 rad`），v17 是 absolute；
兩者 shape 都是 28D/6D，**任何 shape 檢查都抓不到**，手臂會直接暴走。
以 `--contract obs_28_incremental` 與 manifest 的 sha256 為準。

### `detection/`（辨識）
| 檔案 | 說明 |
|------|------|
| `arm_cam.py` | ★手臂相機 + YOLO，bbox → 前向距離+左右偏移（橋接幾何來源） |
| `rear_cam_sam2_publisher.py` | 後相機 + YOLO + **SAM2**，取遮罩最低點 → homography → base 座標，經 rosbridge 發 `/trash_target/detection`。**在 Windows 開發機執行**，不佔 Jetson RAM。需自備 `sam2.1_b.pt` 與 `rear_ground_homography.json`（皆不在 repo）|
| `models/best.pt` | ★正式 YOLOv11 模型。**2026-08-01 起為單類別 `sugarbox`（藍色盒子，高 6.5cm）**，sha256 `ca42c3f4…`。前兩代備份於同目錄：`best_eraser_detect_train11.pt.bak`（`eraser-detect`）、`best_trash_identify_train5.pt.bak`（`bottle-cap`/`paper-ball`）|
| `models/data.yaml` / `yolo11n.pt` | 類別定義 / 基底模型 |
| `calibration/` | 相機/底盤校正腳本與量測資料（含 `calibrate_arm_camera_theta.py`） |
| `debug_tools/` | `yolo_test.py`、`detect_video.py` 等測試/擷取工具 |
| `rear_nav/` | 只剩 `rear_to_arm_blind_handoff.py`（模式 A 導航幾何的來源）。其餘 8 支舊實驗 2026-08-01 已刪 |
| `requirements_detection.txt` | 辨識端相依（ultralytics, opencv-python） |

### `integration/`（整合）
| 檔案 | 說明 |
|------|------|
| `mission_pipeline.py` | ★**完整任務**：巡航→辨識→接近→夾取→送垃圾桶→續巡。單一 Py3.8 程序擁有 `/dev/myserial`，ROS 只跑感測/定位。見 `MISSION.md` |
| `mission_fsm.py` | 21 狀態任務機（純邏輯，`--selftest`/`--diagram`）。強制「輪子與手臂不同時動」「換目標來源必重置 nav」 |
| `map_goal_provider.py` | route.yaml 117 waypoint + AMCL pose → `(dist, bearing)`。含 `--validate` 與弧長重取樣（Route C 原檔最小間距只有 0.049 m） |
| `feedback_odom.py` | `get_motion_data()` → odom pose；2026-09-22 直線尺量更新為 linear 0.98，angular 0.501 仍沿用 Route A、待重測 |
| `ros_io.py` | rosbridge：發 `/odom_setmotor` + `odom→base_footprint` TF、收 `/amcl_pose`（含 covariance 發散門檻）與 `/trash_target/detection`（僅 `--target-source offboard` 時訂閱） |
| `trash_target.py` | 離機 SAM2 目標的轉接層。**發布端 y 左為正、pipeline offset 右為正，這裡負號翻轉** —— 兩邊都是同範圍的 float，接錯不會報錯只會轉錯邊。逾時／無效／後方目標一律 fail closed |
| `vision_grasp_pipeline.py` | ★模式A 自走全流程：雙相機導航(set_car_motion)→handoff→PPO 夾取(obj_provider)→驗證/重試(≤3)。含 `--selftest` |
| `vision_grasp_bridge.py` | 模式B（除錯）：辨識→算 x/y/z/寬度/高度→TCP 5555 送夾取端。payload 是 superset，v17 讀 `w`、v21 讀 `height` |
| `nav_rl.py` + `nav_rl_grasp_pipeline.py` | 模式C：RL 導航避障（PPO+48束LiDAR，訓練 plant 復刻+幾何煞停）→精對位→夾取，見 `NAV_RL.md` |
| `nav_best_model/` | 導航 PPO 權重（best + checkpoint 281440，各配 vecnorm pkl，來源 igibson_x3_test） |
| `mission_status.py` | 遙測發布器。`--status-udp` 每 tick 一個 JSON 封包（UDP），fire-and-forget，沒人聽也不影響任務 |
| `README.md` | 兩種模式開啟流程、各檔用途、校正清單 |

### `ui/`（操作台）

手機/筆電網頁介面，讓使用者不用開終端機打指令。**網頁由 Jetson 自己提供**，
伺服器本身不碰硬體（`/dev/myserial` 的唯一持有者仍然是任務程序）。
只用標準函式庫，Jetson Nano 上不需要 pip install 任何東西。詳見 `ui/README.md`。

| 檔案 | 說明 |
|------|------|
| `server.py` | ★HTTP + SSE 伺服器、任務程序監督、安全閘。`--simulate` 可在開發機無硬體執行 |
| `static/` | 前端：連線 / 主控台 / 任務設定 / 地圖 / 紀錄 |
| `make_qr.py` | 開機偵測 IP 並把連線 QR code 畫到桌面（`--watch` 隨 IP 變動重畫） |
| `jetson_check.sh` | ★上機一鍵檢查：連接埠、防火牆、UDP 迴路、序列埠占用、路線 |
| `test_server.py` | 回歸測試，須 **48** 全過 |
| `prototype.html` | 早期靜態原型，單檔雙擊即開、免 Python。功能以 `static/` 為準 |

```bash
./ui/jetson_check.sh                # ★上機前先跑這個
python3 ui/server.py --simulate     # 開發機，無硬體（狀態與位置由模擬器產生）
python3 ui/server.py                # Jetson，唯讀監看（不驅動硬體）
python3 ui/server.py --allow-real   # 只開第一道閘；目前 A/B/C 實機仍 fail-closed
```

⚠️ **操作台刻意不顯示相機畫面。** Jetson Nano 跑這個專案 RAM 已經吃到九成，
在控制迴路裡編 JPEG 是操作台成本最高的一項。2026-08-06 整條路徑（`camera_publish.py`、
MJPEG 端點、`vision_grasp_pipeline` 的預覽掛勾）已全部移除，有測試擋著不讓它回來。
**不要再加相機預覽。**

操作台只做三件事：**下指令、看狀態、緊急停止**，外加地圖上的大概位置。

省下來的成本：
- 遙測**狀態變化立刻送，其餘 0.5 秒一次**（`--status-period`），不是每個 tick 都送
- **操作台完全不跑出發前檢查** —— 那會另外開載入 torch 的程序；改在終端機跑 `preflight.py`
- SSE 連線數上限 4，避免重連累積執行緒
- 伺服器本身約 27 MB（含相機時是 49 MB）

三個模式都會發 `--status-udp`；模式 B/C 沒有狀態機，用 `mission_status.SimpleReporter`
講**同一套 FSM 狀態名稱**，傳沒定義的名字會直接報錯。

操作台右上角顯示機器人**剩餘記憶體**（讀 `/proc/meminfo`），≥88% 轉黃、≥95% 轉紅。

⚠️ `--allow-real` 與「任務設定」確認目前只是操作台的前兩道閘。操作台尚未提供
serial owner、LiDAR 方向與 grasp-home homography 等模式專用證據，因此 A/B/C
實機請求都會被 server 拒絕；dry-run 與 simulation 可用。完整任務的 final
align/latch 接上量測 homography 後，才可重新開放 `mission_pipeline.py --real`。
網頁上的停止鈕送的是 `SIGINT`（走任務自己的關機路徑，才會把輪子歸零），
夾取途中按下會等該次夾取結束才生效 —— **真正的急停是電源開關**。

---

## 環境設定（在 Jetson Nano 執行一次）

```bash
# 步驟 1：本地化驅動到 grasp/（解決 Python 3.8 找不到 Python 3.6 驅動的問題）
cd ~/Documents/deploy_jetson2/grasp
cp -r /usr/local/lib/python3.6/dist-packages/Rosmaster_Lib .

# 步驟 2：確認 Python 3.8 虛擬環境已啟用
source ~/grasp_venv/bin/activate

# 步驟 3：安裝辨識端相依（首次）
pip install -r ~/Documents/deploy_jetson2/detection/requirements_detection.txt
```

---

## 部署指令

夾取端在 `grasp/v21/` 執行（**上機前先跑 `./jetson_verify.sh`**）：

```bash
cd grasp/v21

# 0) 上機前置檢查：135/37/641/29 全過、wrist_z_offset=0.0564、安全閘 exit 3
./jetson_verify.sh

# 1) 空跑測試（不驅動伺服機，確認角度輸出合理）
python3 x3plus_real_grasp.py \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.03 --obj-x 0.2563 --obj-y -0.0035 --obj-z 0.015

# 2) 實機（★2026-07-31 用這條夾取成功，物體置於 P 點前方 1cm）
#    manifest status 還是 candidate，所以必須帶 --unlock-candidate-real，
#    且須有人在場、手放電源開關。
python3 x3plus_real_grasp.py \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.03 --obj-x 0.2563 --obj-y -0.0035 --obj-z 0.015 \
  --real --unlock-candidate-real
```

`--object-height` 在 28D 契約下是必填（沒有它就沒有 `wrist_z_offset`）。
v21 **沒有** `--width-grip` / `--latch-obj` / `--i-confirm-external-frame`
（夾爪改成接觸偵測 + hold，不再靠寬度算閉合角；物體鎖定由呼叫端的
`obj_provider` 負責）。舊 v17 指令請見本節末的備援區塊。

<details><summary>備援：v17 舊流程（僅在 v21 出問題時使用）</summary>

```bash
cd grasp
python3 x3plus_real_grasp.py --real --socket --width-grip --latch-obj --i-confirm-external-frame
python3 x3plus_real_grasp.py --real --obj-x 0.30 --obj-y 0.05 --obj-z 0.02
```
權重固定用 `trained_6d_models_v17/`，**不可**指到 `v21/models/`。
</details>

### ★ 一鍵：辨識＋夾取（定點，Jetson 本機跑完）

`grasp/v21/jetson_one_command_grasp.py` 把「移到 C3 home → 開相機辨識一次 →
放掉 YOLO 記憶體 → PPO 夾取」串成單一指令。**相機由它自己開，不用另外開串流終端機**
（有別的程序佔著 `/dev/video*`，preflight 會把 pid 和指令列印出來）。底盤不在範圍內。

```bash
source ~/grasp_venv/bin/activate     # 系統 python3 是 3.6.9，跑不動
cd grasp/v21
python3 jetson_one_command_grasp.py --check    # 檔案/相依/相機/序列埠/校正檔，不碰硬體
python3 jetson_one_command_grasp.py            # 正式夾取，人要在旁邊、手放電源
```

⚠️ **C3 姿勢的視覺一定要有實測 homography。** bridge 在 C3 只吃
`--homography` 的 pixel→base 校正檔，舊的 nav-home `H/θ/cam_x/cam_y/sign_y` 一律拒收，
`--i-accept-predicted-extrinsics` 也繞不過去。檔案預設放
`integration/grasp_home_homography.json`，launcher 在**開硬體之前**就用 bridge 同一道閘
（≥6 點、最大誤差 <2cm）驗它。

還沒有校正檔就先量：

```bash
python3 jetson_one_command_grasp.py --calibrate
```

手臂會停在 C3 home 不動。每擺一個位置就等它印一行
`[bridge][calibration] {...}`，把裡面的 `u,v` 跟你尺量的 base `x,y` 記成一組；
至少 6 組不共線、另外留 2 組當驗證點，Ctrl+C 後照畫面提示用
`integration/grasp_home_homography.py` 解出 JSON。詳見
`docs/calibration/CALIBRATION_PLAN.md`「grasp-home homography」。

回歸測試 `grasp/v21/test_one_command_launcher.py` 須 **29** 全過（純邏輯，免硬體）。

### 模式 A：自走全流程（推薦，單一程式控車+手臂）

> 2026-08-01 起手臂端已改接 `grasp/v21/`。物體座標用 `controller.obj_provider`
> 注入，在 home 姿勢鎖定一次後整輪凍結（等同舊版 `--latch-obj`，但由 pipeline 負責）。

```bash
# 純邏輯自測（免相機/硬體/torch，可在開發機跑）
python3 integration/vision_grasp_pipeline.py --selftest
# 乾跑（不驅動硬體；需相機串流）
python3 integration/vision_grasp_pipeline.py
# 正式自走夾取（偵測→導航→夾取→失敗重試≤3）
# --real 需 Phase 3 校正值與確認旗標，否則會拒絕啟動
python3 integration/vision_grasp_pipeline.py --real --show   --cam-x <Phase3_X> --cam-y <Phase3_Y> --sign-y <1或-1> --i-confirm-camera-frame
```
⚠️ 執行前確認 Jetson **沒有跑 port 7000 motor server 或 ROS 底盤 driver**（會搶 Rosmaster 序列埠）。

### 模式 B：TCP 橋接（除錯用，底盤需另外處理）

**用途**：把「辨識」跟「夾取」拆成兩個行程，各跑一個終端機。辨識端算出物體座標
用 TCP 5555 送過去，夾取端只管夾。好處是可以單獨換掉任一端 —— 例如手動送一筆
假座標測夾取、或先只看辨識算出來的 x/y/z 合不合理，不用真的動手臂。
底盤導航**不在**模式 B 範圍內，要自己處理。

2026-08-01 起模式 B 也支援 v21。橋接端送的是 superset payload
（`x,y,z,w,height`）：v17 讀 `w` 忽略 `height`，v21 讀 `height` 忽略 `w`，
所以同一支橋接程式兩邊都能接。

```bash
# 終端機 A —— 夾取端（v21）
cd grasp/v21
python3 x3plus_real_grasp.py --real --socket \
  --latch-obj --i-confirm-external-frame --unlock-candidate-real \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental

# 終端機 B —— 辨識端。先 --once 核對座標合理，再連續送
python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --once --show \
  --class-height eraser-detect=0.03 --class-z eraser-detect=0.015
python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --show \
  --class-height eraser-detect=0.03 --class-z eraser-detect=0.015
```

**旗標說明**
- `--socket`：從 TCP 5555 接收 `{x,y,z,height}`（公尺）。
- `--latch-obj`：在 home 姿勢**擷取一次**座標+高度並凍結整輪。
  **手臂相機裝在 `arm_link4` 會跟著手臂動**，`arm_cam` 的固定高度/俯仰角只在 home 成立，
  手臂一離開 home 偵測就失真。沒有 latch，目標會在接近途中從 policy 腳下漂走。
  `--real --socket` **強制**要這個旗標。
- `--i-confirm-external-frame`：確認送來的 XYZ 已經在 PPO/URDF base_link 座標系，
  不是原始相機座標。座標系搞錯**下游偵測不到**，只會很有把握地夾在錯的地方。
  `--real --socket` 也強制要這個。
- 橋接端 `--class-height NAME=公尺`：物體全高（頂到地），v21 用來算 `wrist_z_offset`。
  **不能從 bbox 量**（相機是斜著往下看，bbox 像素高度混了高度和深度），所以按類別填。
  不填就不送 `height`，v21 退回它自己的對稱物體估計。
- 橋接端 `--class-z NAME=公尺`：物體**質心** z，跟高度是兩回事（對稱且貼地才有 `高度 = 2×質心z`）。

夾取端還會做兩件事保護自己：payload 有 NaN/無限大/負高度就整筆丟掉；
超過 `detection_stale_timeout_sec`（預設 1 秒）沒有新偵測，就把舊值撤回改用預設值，
而不是繼續當成現況供應。`--real` 下 latch 逾時會直接 raise，不會朝預設座標移動。
- `--latch-obj`：在 home 姿勢擷取一次 obj 座標+寬度並凍結整輪。**手臂相機裝在 arm_link4 會隨手臂移動**，
  固定高度/俯仰角只在 home 成立，故強烈建議開啟以免移動後失真偵測干擾 policy。

---

## Sim-to-Real 關節映射

**重要**：Rosmaster_Lib API 對 S2/S3/S4 有內部鏡像（API = 180 − 物理角度）。
下表的 hw 值指 **API 值**（程式碼送出的值），非 App 顯示的物理值。

| 伺服機 | 方向 | hw(API) 公式 | 狀態 |
|--------|------|-------------|------|
| S1 | 正向 | hw = 90 + sim_deg | 已確認（無鏡像） |
| S2 | 正向 | hw = 90 + sim_deg | 已確認（API 鏡像抵消原 invert） |
| S3 | 正向 | hw = 90 + sim_deg | 已確認（同上） |
| S4 | 正向 | hw = 90 + sim_deg | 已確認（同上） |
| S5 | 正向 | hw = 90 + sim_deg（0~270°）| 已確認（無鏡像） |
| S6 夾爪 | 特殊 | open=30°, closed=180° | 已確認（2026-05-22）；`--width-grip` 時閉合角依寬度落在 120~180° |

---

## 觀測空間（28D）

```
[0:5]   臂關節角度（sim rad）
[5]     夾爪角度（sim rad）
[6:9]   TCP 位置（m）
[9:13]  TCP 四元數（x, y, z, w）
[13:16] 物體位置（m）
[16:19] 相對位置 = obj − tcp
[19:22] Stage one-hot [s0, s1, s2]
[22:28] 上一步 action（6D）
```

---

## 三段式控制邏輯（★v21 現行）

| Stage | 觸發條件 | 行為 |
|-------|----------|------|
| 0 | 初始 | RL 輸出對位（incremental），夾爪張開。進 Stage 1 要同時滿足 xy / z / 進場半徑 / pads_ready 四項幾何閘；卡在門檻外時有懸停後備（連續 3 秒近失即放行） |
| 1 | 上述幾何閘通過 | 鎖定手臂、S6 限速閉合。**接觸偵測**：連續 2 次「指令走了但角度沒跟上」判定夾到 → 指令停在 `接觸角 + jaw_hold_bias_deg(8°)`，不再往 180 硬推（這是夾持力來源，也是齒輪研磨的防線） |
| 2 | Stage 1 確認夾到 | **全程維持同一個 hold 角**（不可用編碼器回讀重設，否則位置誤差歸零、物體會掉），抬升後回 home |

空夾判定：夾爪一路走到 180° 全閉停點 = 中間沒東西 → ABORT，不抬升。
`grasp_stall_min_fraction = 0.03`（2026-07-31 實機驗證：空夾在此門檻下不會誤判）。

<details><summary>v17 舊版三段式（備援）</summary>

| Stage | 觸發條件 | 行為 |
|-------|----------|------|
| 0 | 初始 | RL 輸出對位（absolute），夾爪張開 |
| 1 | dist < 5 cm（`stage0_dist_threshold`）或 grip_cmd > 0.90（`grip_close_threshold`）或 diverging | 鎖定手臂、S6 限速閉合至目標角（`--width-grip` 時依寬度，否則 180°），到位後轉 Stage 2 |
| 2 | Stage 1 完成後 | 維持夾爪閉合角，回 home |
</details>

---

## Rosmaster_Lib API（與舊版 Arm_Lib 對照）

| 動作 | 舊版 Arm_Lib | 新版 Rosmaster_Lib |
|------|-------------|-------------------|
| 初始化 | `Arm_Lib.Arm_Device()` | `Rosmaster(); create_receive_threading()` |
| 送角度 | `Arm_serial_servo_write6_array(s1..s6, t)` | `set_uart_servo_angle_array(angle_s=[...], run_time=t)` |
| 讀角度 | `Arm_serial_servo_read(i)` | `get_uart_servo_angle(i)` |

---

## 工作流程（放程式碼到此資料夾後）

當使用者放入 `.py` 檔案，Claude 應：
1. 確認使用 Rosmaster_Lib（不是 Arm_Lib）
2. 確認觀測空間 28D，動作空間 6D
3. 確認 VecNormalize 載入時 `training=False`、`norm_reward=False`
4. 直接修改並回報差異

---

## 注意事項
- 首次連接伺服機前，務必先跑一次 dry-run，目視確認角度輸出合理
- arm_hw_invert = (False, False, False, False, False) — API 鏡像（S2/S3/S4）已抵消原 invert，全部改為 False
- 安全限制：v21 `max_delta_deg=8.0`（v17 是 3.0），每步每軸最多動這麼多，防止暴衝。
  限速是從**上一次的指令值**（`_last_deg`）走，不是從編碼器 —— 夾爪被物體擋住時指令仍會往前走，
  這就是第 3 跑齒輪研磨的成因，現在由接觸偵測擋下
- 若 Stage 1 觸發但夾爪與物體相差甚遠，需校正 obj 位置或 FK 偏移量
- **物體座標必須在 base/URDF 座標系**（與 FK 算出的 TCP 同框）；橋接端的相機距離→base 映射需校正
  `--cam-x/--cam-y/--sign-y/--obj-z`，詳見 `integration/README.md` 校正清單
- **手臂相機（URDF `mono_link`）固定在 `arm_link4`，會隨手臂移動**，`arm_cam` 固定高度/俯仰角只在 home 成立 →
  視覺夾取務必在 home 鎖定一次（v21：pipeline 側凍結 `obj_provider`；v17：`--latch-obj`）
- **半雙工伺服匯流排**：六顆伺服機共用一條 UART。`Rosmaster_Lib.get_uart_servo_value` 回傳
  「第一個抵達的回應」而不檢查是不是問的那一顆 —— 一個遲到封包會讓整條管線錯位，症狀是相鄰數顆
  同時「讀不到」。v21 已改為讀 raw 並認回應者的 ID。診斷用 `grasp/v21/bus_probe.py`（唯讀，不寫入）
- **URDF 手指比實體長 16.7mm** — 尚未修（要等 v22 重訓）。所以紙球、瓶蓋這類矮物體目前夾不到；
  可用 `--floor-finger-error-mm` 做部分補償，但那只抬 pad，非 pad link 最低時會被夾住不放大
- v21 尚未通過的硬體 gate：`guard_margin_8mm_validated_on_hardware`、`c3_real_reach_envelope`、
  `object_heights_measured`。所以 manifest `status` 仍是 `candidate`，`--real` 必帶 `--unlock-candidate-real`

<details><summary>v17 專屬設定（備援）</summary>

- 安全限制：`max_delta_deg=3.0`
- `stage0_dist_threshold = 0.05m`；備用觸發：dist 從最小值回升 > 0.05m 也觸發 Stage 1
- `--width-grip` 寬度→夾爪角公式：`close = 180 − (w/grip_max_object_width_m)×(180 − grip_min_close_deg)`
  （預設 `grip_max_object_width_m=0.06`、`grip_min_close_deg=120`，需依夾爪實測校正）
</details>
