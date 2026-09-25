# 完整任務流程：巡航 → 辨識 → 走過去 → 夾取 → 送垃圾桶 → 續巡

`mission_pipeline.py` 把導航（Navigation 交接包的 Route A/B/C）和夾取（v21）接成單一
任務。單一 Python 3.8 程序擁有 `/dev/myserial`，ROS Melodic 只負責感測與定位。

> **目前狀態（2026-09-06）：只允許 dry-run。** final align/latch 尚未接入 C3
> grasp-home 的量測 homography，`--real` 會在載入 YOLO、相機、LiDAR 或序列埠前
> fail-closed。下方實機步驟保留為完成 homography 整合後的驗證順序，現在不可視為
> 可執行的正式啟動程序。

```
本程序（唯一 /dev/myserial owner）          ROS Melodic（不得開底盤序列埠）
  GraspController → 伺服機（v21）             robot_state_publisher
  set_car_motion  → 輪子                      YDLIDAR TG30 → /scan
  get_motion_data → /odom_setmotor + TF       map_server → /map
  導航 PPO 55D→2D、夾取 PPO 28D→6D            AMCL → map→odom
  MissionFSM                                  rosbridge_server
```

## 新增模組

| 檔案 | 職責 | 離線自測 |
|------|------|---------|
| `map_goal_provider.py` | route.yaml 117 waypoint + AMCL pose → `(dist, bearing)`；到點/繞行/中斷續巡/禁區 | `--selftest`、`--validate` |
| `feedback_odom.py` | `get_motion_data()` → odom pose（2026-09-22 尺量 linear 0.98；angular 0.501 待重測） | `--selftest` |
| `ros_io.py` | rosbridge 發 `/odom_setmotor`+TF、收 `/amcl_pose`；`--target-source offboard` 時另收 `/trash_target/detection` | `--selftest`、`--probe` |
| `trash_target.py` | 離機 SAM2 目標 → `(found, dist, offset)`。**負號翻轉**與時效判定 | `tests/test_trash_target.py`（18） |
| `mission_fsm.py` | 21 狀態任務機（純邏輯，含 `--no-deliver` 的終止持物狀態） | `--selftest`、`--diagram` |
| `mission_pipeline.py` | 主程序，接起全部 | `--selftest` |

### 偵測來源（`--target-source`）

| 值 | 來源 | 說明 |
|---|---|---|
| `onboard`（預設） | `vision_grasp_pipeline._detect_rear` | 機上後相機 YOLO bbox，不依賴外部連線 |
| `offboard` | `/trash_target/detection` | 離機 YOLO + SAM2 遮罩最低點；需 `--ros-backend ros`，否則解析參數時就擋下 |

兩者回傳同一組 `(found, 前向距離 m, 右正橫向偏移 m)`，下游的一致性閘、黑名單與
狀態機完全相同。

⚠️ **發布端 `object_y_base` 左為正，`estimate_offset_x` 右為正**，`trash_target.py`
是唯一做負號翻轉的地方。兩邊都是同範圍的 float，接錯不會報錯，只會每次都轉錯邊。

離機來源逾時（`--trash-max-age`，預設 1 秒）即回報「沒偵測到」繼續巡航。時效以
**本地到達時間**計算 —— 發布端在另一台機器上，時鐘不同。

全部離線可測：

```bash
for m in map_goal_provider feedback_odom ros_io mission_fsm mission_pipeline; do python3 integration/$m.py --selftest; done
```

## 手臂在各階段的姿態

兩個 home，全程只切換一次：

| 階段 | 手臂姿態 | 為什麼 |
|---|---|---|
| SELF_CHECK / IDLE | **nav home** `(90, 140, 0, 0, 90, 30)` | 開機時手臂可能停在上一輪的任何地方，先歸位 |
| PATROL / INVESTIGATE / APPROACH | **nav home** | 行進姿態，夾爪收在上方 |
| **ALIGN 進入時** | **切到 C3** `(90, 67.08, 9.79, 9.79, 90, 30)` | 手臂相機從這裡開始用，底盤此時已停 |
| STATIONARY_GATE / LATCH / GRASP | **C3** | v21 的訓練姿態 |
| CARRY_HOME → DELIVER | **nav home**（夾持中） | v21 的 `_scripted_lift_and_return` 自己回 `home_deg` |
| PLACE | 從當下姿態往前伸 | `run_release_only()` 讀編碼器 |
| 放棄目標 → RESUME | **切回 nav home** | 不然會帶著 C3 姿態繼續巡航 |

為什麼不能全程用 C3：

```
底盤最前緣                      base_footprint 前方 11.95 cm
C3 手臂最前緣                   base_footprint 前方 23.28 cm  ← 多伸出 11.3 cm
C3 夾爪離地                     11.1 cm
LiDAR 掃描面                    19.2 cm       ← 比夾爪高 8 cm，看不到它要撞什麼
```

v21 的 `DeployConfig.home_deg` **就是** C3、`grasp_home_deg` 預設 `None`（manifest 明講
「there is no separate nav-home/grasp-home split」）。那對「單獨跑夾取」是對的 —— 那時車子
已經停在物體前。對要先巡航 58 公尺的機器人是錯的。

v21 其實預留了機制：`run()` 第 2586 行 `start_pose = grasp_home_deg if set else home_deg`，
而 `_scripted_lift_and_return` 夾完會回 `home_deg`。所以只要把兩個都設好，
**夾完之後物體會用 nav 姿態被帶去垃圾桶**，不需要額外處理。

改姿態用 `--nav-home-deg` / `--grasp-home-deg`。手臂相機的外參檢查跟的是**夾取** home。

⚠️ **手臂會不會擋到 LiDAR 尚未實測**。URDF 的 AABB 粗估顯示三種姿態下 `arm_link1` 都可能
穿過 19.2 cm 掃描面（AABB 是寬鬆上界，不等於真的擋到）。上機時用 `--probe` 分別在 nav home
和 C3 看正前方讀數就知道 —— 若讀到約 5~19 cm 的固定回波，那就是手臂本身，煞停會被永久觸發。

## 狀態機

```
BOOT → SELF_CHECK → IDLE --start--> PATROL
                                      │ 同一類別連續 3 幀（目標來源 map→相機，重置 nav）
                                      ▼
                            INVESTIGATE →(≤2.5m)→ APPROACH →(≤0.75m)→ ALIGN
                                      │ 丟失/逾時            │ 丟失/逾時      │ 進入 v21 可夾框
                                      └────→ RESUME ←────────┘                ▼
                                               ▲                      STATIONARY_GATE
                                               │                              ▼ 停穩
                                               │                           LATCH → GRASP → VERIFY
                                               │                    失敗 ↙                    ↘ 成功
                                               │                  RETRY(≤3)              CARRY_HOME
                                               │                    ↓                          ▼
                                               │                APPROACH                   DELIVER
                                               │                                              ▼
                                               └──────────────── PLACE ← PLACE_ALIGN
```

任何狀態：感測器 stale → **PAUSED**（只有人能離開）；ESTOP／fault → 鎖定，不自動恢復。

程式強制兩條硬體不變式，測試涵蓋：

1. **輪子和手臂不會同時被允許動**。兩者共用一條 Rosmaster 序列匯流排和同一個重心；
   手臂只在底盤確認停穩後才動（stationary gate）。
2. **切換目標來源（地圖 waypoint ↔ 相機目標）一定重置 `ActionDelay` 和 `GoalTracker`**。
   殘留的延遲命令是瞄準一個已經不存在的目標。

## 上機啟動順序

```bash
# 0. 確認沒有別的程序佔序列埠（必須剛好一個 PID，就是本程序）
sudo fuser -v /dev/myserial
ps -ef | grep -E '[r]osmaster_main|[M]cnamu_driver|[a]i_motor_server_B|[r]oute_a_runtime'
```

```bash
# 1. ROS 端（各自一個終端機）
roscore
roslaunch ydlidar_ros_driver TG.launch
roslaunch <robot model>.launch                 # robot_state_publisher
rosrun map_server map_server site_map.yaml
roslaunch amcl.launch
roslaunch rosbridge_server rosbridge_websocket.launch   # 需含 rosapi
```

> ⚠️ **這一步還不要設 2D Pose Estimate。** `odom → base_footprint` 的唯一發布者是本
> pipeline（`OdomPublisher`，20 Hz），而 `FeedbackOdom` 每次程序啟動都從 (0, 0, 0)
> 重新積分。主程式一開，odom 原點就跳回車子當下的位置，AMCL 手上的 `map→odom`
> 立刻過期 —— 先設好的定位會在啟動主程式的那一刻失效，`variance` 從收斂值彈到
> 0.3～60 m²，self-check 卡在
> `FAIL: AMCL not usable (AMCL position variance … > 0.0625 m^2)`，連 Enter 都沒得按。
> **順序是「先跑主程式（步驟 4），再設 2D Pose Estimate（步驟 5）」**，
> 見 `SETMOTOR_ODOM_INTEGRATION.md` §4.3 與 §10.2。

> **AMCL 只在「有動」的時候更新並發佈 `/amcl_pose`。** 夾取一次會讓底盤靜止 2 分鐘以上，
> 出來就進 DELIVER，而 DELIVER 把過期的 fix 當成阻斷性故障 —— 每夾成功一次就會停在
> PAUSED，按 Enter 重跑 self-check 也會卡在同一個過期 pose。
>
> 本程序自己解決這件事：靜止時呼叫 `/request_nomotion_update`，並在手臂佔住主迴圈之後
> 主動補一次 fix（`--amcl-refresh-timeout`，預設 3 秒）。**不需要**另外跑 Route B 的
> `amcl_nomotion_keepalive.py`（跑了也無害，那支只在 `route_b_startup.sh dryrun` 階段啟動，
> 本流程不走那個階段）。
>
> 前提是 AMCL 真的有提供這個 service。self-check 與 `preflight --onboard` 都會確認，
> `--real` 下確認不到就拒絕啟動。查不到 rosapi 跟查得到但沒有這個 service 是兩件事，
> 訊息會分開講。

```bash
# 2. 確認定位真的在動
python3 integration/ros_io.py --probe --ros-host 127.0.0.1
```

```bash
# 3. 確認 LiDAR 左右沒有反（前/左/右各放實物）
python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1
# The probe is observational only.  Real mode additionally requires the
# Route B four-direction evidence marker.
```

```bash
# 4. 目前只跑 dry-run；--real 會在開啟裝置前拒絕
source ~/grasp_venv/bin/activate
python3 integration/mission_pipeline.py --dry-run --show \
  --route <route.yaml>
```

完成 homography 整合並解除 runtime 封鎖後，實機流程才會先印
`FAIL: AMCL not usable`；那是預期的 —— 現在才輪到定位。**不要關掉它**，
它已經在發新的 `odom → base_footprint` 了，關掉再開就是把 odom 又歸零一次。

**5. 主程式維持運行，回 RViz 設定位**

RViz 用 **2D Pose Estimate 設緊初始化**（std 0.15 m / yaw 7°）：點在車子真實位置、
箭頭朝真實車頭、放開後確認 `/scan` 貼合牆線、粒子雲收斂。寬初始化實測在重複走廊
跳 1.573 m，不會收斂。

```bash
rosservice call /request_nomotion_update     # 靜止時逼 AMCL 重跑一次 filter
rostopic echo -n 1 /amcl_pose                # covariance[0] 與 [7] 都要 < 0.0625
```

門檻 0.0625 m²（= 0.25 m std）定義在 `ros_io.DEFAULT_MAX_POS_VAR`，另有 20° 的 yaw
門檻。**不要為了開跑去調低門檻**：超標數十倍是定位真的丟了，不是誤判。self-check 每
個 tick 都會重跑，所以定位一收斂它自己就會轉成
`AMCL ok at (...)` → `Self-check passed`，這時才按 Enter。

分段驗證見下方測試計畫。`--detection-streak 999` 讓它永遠不離開路線（只驗巡航）；
`--no-deliver` 讓它夾到就停（不送桶）。

## 上機前必須做的量測（程式會拒絕沒做的）

| 項目 | 為什麼 | 怎麼做 |
|------|--------|--------|
| 序列埠唯一擁有者 | 兩個程序開同一條 UART = 命令交錯 | `fuser -v /dev/myserial` |
| LiDAR raw-index 物理方向 | AMCL/TF 正常仍不能證明 policy 前方 | Route B `LIDAR_ORIENTATION_GATE.md` 四方向測試，建立 `scan_orientation_verified` |
| 靜止 feedback 雜訊 | stationary gate 的門檻是暫定值 | 靜止 5 秒記錄 `get_motion_data()` |
| 靜止時 AMCL 保鮮 | AMCL 只在有動時更新，夾取會靜止 2 分鐘以上 | `rosservice list \| grep request_nomotion_update`；self-check 會擋 |

## 已知落差

- **手臂相機 C3 外參是 URDF 預測值**，啟動會警告 `theta/H/cam_x/cam_y PREDICTED`。
  依 2026-08-04 的決定不處理：v21 用這組參數在實機上已經夾成功。若之後夾取落點
  系統性偏移，這裡是第一個要回頭看的地方。
- **巡航中發現物體並前往（PATROL→INVESTIGATE→APPROACH）尚未實車驗證**。這段
  2026-08-04 修掉三個只會在實機上顯現成「它無視垃圾」的邏輯錯誤，見下方。
- **v21 是 `candidate` 不是 approved**。`manifest.json` 的 `protocol_valid: false`
  （正式評估的 home jitter 沒照協議跑）。成績是真的但沒有認證，簡報要照這個講法。
- **`route.yaml` 需要重取樣**。宣告 0.75 m 間距，實際最小 0.049 m（0.049 m/cell 網格上
  的正交 A* 階梯），117 個間隔裡 45 個不到 0.5 m，`patrol_001` 和 `patrol_116` 座標完全
  相同。程式預設 `--resample-m 0.75`（117 → 83 點，最小間距 0.530 m）。取樣點都落在原折線
  上，不會切出 0.35 m 安全走廊。要用原檔就 `--resample-m 0`，但那樣會在載入時被拒絕。
- **地圖來自 GLB 渲染不是 SLAM 建圖**，現場家具/箱子/人不在圖上。48 束 policy 會避障，
  但 AMCL 會被未建模障礙拉偏。
- **丟失定位沒有自動恢復**。AMCL 發散 → covariance 超標 → PAUSED，要人重設初始位姿。
- **`nav_rl_grasp_pipeline.py` / `vision_grasp_pipeline.py` 沒有設 v21 的模型路徑**，
  用預設會死在 `SELECTED_MODEL_REQUIRED.zip`。`mission_pipeline.py` 從
  `grasp/v21/manifest.json` 解析並驗證 SHA256（manifest 明講兩個檔是一組、不可混用；
  混用不會報錯，只會讓 policy 拿到錯誤正規化的觀測然後夾錯地方）。

## 座標系 — 交接距離講的是哪個原點

三個常被混用的原點，數字用 `grasp/v21` 自己的 FK 量出來（2026-08-04），不是推算：

```
policy_x = base_footprint_x + 0.0199
```

| 參考點 | 在 base_footprint 前方 |
|---|---|
| 後輪軸 | −0.080 m |
| **base_footprint / base_link** | **0**（前後輪軸的**正中間**，不是前輪軸） |
| 前輪軸 | +0.080 m |
| 手臂基座 `arm_link1` | +0.098 m（URDF 0.09825，FK 吻合） |

`base_footprint` 是**底盤中心**：前輪 x=+0.08、後輪 x=−0.08，中點就是原點。

## v21 可夾框 — 導航的實際規格

導航要把車停到讓物體落在這個框裡（policy 座標）：

| | policy x | 換算成 base_footprint 前方 | 換算成前輪軸前方 |
|---|---|---|---|
| 訓練區間 | 0.20 ~ 0.33 | 0.180 ~ 0.310 m | 0.100 ~ 0.230 m |
| 正式評估過（程式用這個） | **0.20 ~ 0.28** | **0.180 ~ 0.260 m** | **0.100 ~ 0.180 m** |
| C3 home 的 TCP 落點 | 0.226 | 0.206 m | 0.126 m |
| C3 手臂相機看得到 | 0.205 ~ 0.317 | 0.185 ~ 0.297 m | 0.105 ~ 0.217 m |

y 方向：正式評估過 −0.09 ~ +0.08 m。

**相機看得到不代表 policy 夾得到** —— 可視帶的遠端 0.317 超出訓練區。
`MissionRunner.envelope_ok()` 在 ALIGN 先擋，v21 的 `_check_target_envelope()` 再擋一次。

交接門檻是兩層，兩層都要過：

1. **訓練框**（硬性，v21 的契約，不要動）：policy x ∈ [0.20, 0.28]、y ∈ [−0.09, 0.08]
2. **對準半徑**（可調，上機要調的就是這個）：距瞄準點 `--handoff-aim-x`（預設 0.24，
   即訓練框中心）的直線距離 ≤ `--handoff-radius-m`（預設 0.04 m）

RL 導航停在 0.75 m、到點半徑 0.35 m，和這個 8×17 cm 的框差一個數量級 ——
**視覺精對位（ALIGN）不是可選項，是唯一能收斂到這個框的東西**。

每一則被擋下來的訊息都會同時印出三個座標系的值，避免再對錯原點：

```
policy x=0.240 | base_footprint x=0.220 | front-axle x=0.140 | y=+0.000 m
```

## 丟垃圾 — 最簡化版

到達 `route.yaml` 標註的 `trash_bin.approach` 點之後，直接跑 v21 的
`run_release_only()`：**不看垃圾桶在哪、不瞄準、不需要量測越過桶子的手臂姿態**。

桶口約 30 cm 寬，所以水平方向每邊都有約 10 cm 餘裕，物體直接掉下去就好。唯一會真的失敗的
是「在夾爪低於桶緣時放開」——那會把物體掉在桶子外側 —— 所以往前伸的動作在 FK 判定夾爪
墊片會低於「桶緣 + 餘裕」時就停下，並且如果當下位置已經低於桶緣，**寧可繼續抓著也不放**。

```bash
--bin-rim-height 0         # 預設 0 = 不管桶子多高，直接跑預設動作
--release-clearance 0.03   # 剩下的唯一檢查：不要在貼近地面時開爪
--release-extend-steps 6
```

桶緣預設 0，所以伸出動作不會被提早擋下。只有在你想讓它為了高桶子提早停手時才需要設。

沒有放開的驗證。夾持判定靠「夾爪停在未完全閉合的角度」，一旦命令張開就一定讀成
「已放開」。`released` 的意思是**放開的動作跑完了**，不是「東西在桶子裡」。

`run_release_only()` 會先檢查夾爪裡是否真的有東西：沒有就回 `rejected` 並直接續巡 ——
這同時也抓到「東西在半路掉了」的情況，而不是對著空夾爪演一次放開。
