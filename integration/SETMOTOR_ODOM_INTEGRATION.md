# set_motor＋feedback odom＋RL 導航＋夾爪整合細部規劃

> 狀態：設計／驗收規格 v1.1（2026-07-16）。ROS `/scan`→Python 3.8 adapter 已實作；
> set_motor、feedback odom、TF、AMCL／巡航狀態機仍**尚未實作**。
> 本文件補充 `docs/planning/MISSION_PLAN.md` 的 Phase 8 底層接線細節。真正的
> `/home/jetson/ai_motor_server_B.py` 尚未進入本 repo；取得原檔並核對四輪映射前，
> 不可猜測馬達正負號或直接上實車。

## 1. 已再次確認的現況

- 現行 `integration/nav_rl_grasp_pipeline.py` 以同一個 `Rosmaster` 物件控制底盤與手臂，
  底盤目前呼叫 `set_car_motion(vx, 0, wz)`，導航策略輸出是 6 Hz 的**連續**速度。
- `grasp/x3plus_real_grasp.py` 已使用 Rosmaster_Lib、28D observation、6D action；
  夾取 VecNormalize 在推論時是 frozen semantics，S6 為 30° 張開／180° 閉合，
  每步最大 3°。
- `ServoController` 目前只呼叫 `create_receive_threading()`，尚未呼叫
  `set_auto_report_state(True, False)`；repo 內也沒有 `/odom_setmotor` publisher 或 TF broadcaster。
- 下載資料中的 `ai_motor_server_B.py` 執行紀錄顯示它能直接 `set_motor()`，但目前那份檔案
  grep 不到 ROS node／Odometry／TF 程式碼，因此只能視為 TCP 馬達程序，不能視為已恢復 odom。
- command-based odom 已棄用；正式定位只接受底層 motion feedback（之後可再融合 IMU）。
- YDLIDAR TG30 已由出廠 `TG.launch` 驗證 `/scan` 約 10.17Hz；`nav_rl.py` 已用
  roslibpy latest-only queue=1 訂閱，保留 header timestamp，控制安全 age 使用本機接收時間。
- `X3Plus_部署操作指南.md` 是舊架構：Arm_Lib、22D observation、舊模型名、S6 0/160°、
  ROS topic 控手臂，與本 repo 現行 FK-only／28D／Rosmaster_Lib／S6 30/180° 不相容，
  **不可再作為上機依據**。

## 2. 最終架構決策

### 2.1 單一硬體擁有者

正式 runtime 只允許主 pipeline 開啟 `/dev/myserial`：

```text
主 pipeline（Python 3.8，唯一 /dev/myserial owner）
├─ ChassisActuator.command_velocity(vx, vy, wz)
│  └─ SetMotorActuator → set_motor(m1, m2, m3, m4)
├─ ServoController → set_uart_servo_angle_array(S1..S6)
├─ MotionFeedbackOdom → get_motion_data() → 積分 pose
├─ RosBridgeIO → /odom_setmotor + TF、/scan、/amcl_pose
├─ RL navigation（55D→2D）
└─ PPO grasp（28D→6D）

ROS Melodic 端（不得開 /dev/myserial）
├─ robot_state_publisher
├─ LiDAR driver → /scan
├─ map_server
├─ AMCL → map→odom
└─ rosbridge_server
```

不採用「外部 motor server 開 serial＋夾取程序再開一次 serial」。若一定要保留硬體 daemon，
它必須同時代理底盤和六顆伺服機，等於要重寫已驗證的夾取緊迴圈；目前不推薦。

### 2.2 唯一 odom topic 與 TF ownership

| 輸出 | 唯一 owner | 說明 |
|---|---|---|
| `/odom_setmotor` | 主 pipeline 的 MotionFeedbackOdom／RosBridgeIO | canonical odom topic；需要 `/odom` 的節點以 remap 解決，不再另算一份 odom |
| `map → odom` | AMCL | 自訂 odom 程式不可發布 |
| `odom → base_footprint` | MotionFeedbackOdom／RosBridgeIO | pose 與 TF 必須同 timestamp |
| `base_footprint → base_link` | robot_state_publisher | URDF 正確 z=0.082；不可手動補第二條 static TF |
| `base_link → laser_link` | robot_state_publisher | 由 URDF 決定 |
| `laser_link → laser` | 一個已確認必要的 static TF owner | URDF／launch 已有時不得重複 |

完整鏈固定為：

```text
map → odom → base_footprint → base_link → laser_link → laser
```

### 2.3 導航 actuator 契約

RL 策略必須持續看到與執行連續速度，不能降級成只有
`forward/turn_left/curve_right + speed` 的離散協定：

```text
command_velocity(vx_mps, vy_mps, wz_radps, timestamp)
stop(reason)
last_command_age()
```

`SetMotorActuator` 內部才負責：

1. 以實車已驗證的 4×3 mecanum mixer 將 `(vx, vy, wz)` 轉成 `(m1..m4)`；
2. 套用每輪方向、線性／角速度比例、deadband；
3. 四輪同比例 normalize，禁止單輪 clipping 改變運動方向；
4. 限制加速度與命令範圍；
5. watchdog 逾時立即連續送零；
6. `stop()` 至少重送數次零命令，並以 feedback 確認真的停下。

四輪順序和正負號只能從實際 `ai_motor_server_B.py` 與校正資料取得，不能從通用麥輪公式猜。

### 2.4 serial write 仲裁

「同一個 process」仍不足以保證安全；所有會寫 Rosmaster serial 的呼叫還要經過同一個
`SerialCommandArbiter`／lock：

```text
優先序：ESTOP／底盤 stop > servo safety command > navigation velocity
```

- ROS callback、相機 thread、odom thread 不得直接寫 driver。
- 導航狀態只允許 chassis 非零命令；GRASP／PLACE 狀態 chassis 只能送零。
- watchdog 的零命令和 servo command 可交錯，但每個完整 serial packet 必須在 lock 內寫完。
- 任一 driver exception 先鎖定 nonzero chassis command，再進 FAULT。

### 2.5 可替換目標來源

通用導航迴圈只接受以下契約，不直接知道 YOLO 或 route.yaml：

```text
GoalProvider.get(now) -> {dist, bearing, source_stamp, valid, reason}
GoalProvider.reset()
```

- `VisualGoalProvider`：YOLO fix＋GoalTracker dead-reckoning。
- `MapGoalProvider`：AMCL pose＋waypoint／垃圾桶 map 座標。
- 切換 provider 前先停車，再清 `ActionDelay`／tracker，最後才允許下一個非零 command。

## 3. 兩階段遷移，避免把「找回 odom」和「最終整合」混在一起

### 階段 A：獨立恢復 feedback odom

目的只是在沒有導航、沒有手臂動作時證明底層 feedback／ROS／TF 可用。

1. 從 Jetson 找回真正的 `ai_motor_server_B.py`，存入 repo 並記 SHA256；原檔先只讀備份。
2. 確認它的 motor mapping、停止重送、TCP framing、watchdog 和 serial port。
3. 停止 `rosmaster_main.py`、官方底盤 driver 和任何會開 `/dev/myserial` 的程序。
4. 啟動 receive thread＋auto-report，靜止取樣 5 秒，確認 timestamp／數值持續更新。
5. 只做低速前進、橫移、左轉、右轉，驗證 feedback 軸向符合 ROS：`+x` 前、`+y` 左、`+z yaw` 左轉。
6. 實作並發布 `/odom_setmotor` 和 `odom → base_footprint`。
7. 依 §8 gate A1–A6 驗收。此時不啟動 grasp pipeline。

階段 A 的 ROS node 可以暫時使用能在實機 interpreter 正常 import 的 `rospy`；但它不是最終
雙程序架構的理由。若 Python 3.8 無法 import rospy，直接走 rosbridge／roslibpy。

### 階段 B：合併進正式單程序 runtime

1. 從已驗證 server 抽出不自行建立 `Rosmaster` 的 `SetMotorActuator`。
2. 從已驗證 odom 抽出只接收**既有 device handle** 的 `MotionFeedbackOdom`。
3. `GraspController.servo.device` 成為唯一共享 handle；actuator、feedback、servo 都使用它。
4. 導航器不再直接呼叫 `device.set_car_motion()`，改呼叫 `ChassisActuator` 介面。
5. 加 `RosBridgeIO`，ROS 端不得再啟動會開 Rosmaster serial 的 driver。
6. 重跑夾取 dry-run／導航 selftest／低速 actuator 測試，再准許做完整任務。

## 4. feedback odom 資料流

### 4.1 啟動

1. 建立唯一 `Rosmaster(com=/dev/myserial)`。
2. `create_receive_threading()`。
3. `set_auto_report_state(True, False)`。
4. 清掉啟動前 pose，記錄 `time.monotonic()`。
5. 等待至少 5 筆**內容或時間有更新**的 feedback；逾時即 fail closed，不准移動。

### 4.2 每個 odom tick（目標 20 Hz）

現行校正為：

```text
vx = vx_raw * 0.9663094
vy = vy_raw * 0.65
wz = wz_raw * (0.9857046 if wz_raw >= 0 else 0.9854675)
```

`vx` 是 2026-09-24 在本機以前進 35.5 cm、後退 34.8 cm 的 raw feedback
積分做 through-origin fit；前後個別 scale 差 0.25%。raw `vx` 方向與 ROS `+x`
一致。`vy` 未在該次測試重測，仍保留 Route A 的 0.65。左右旋轉同日以地面
基準線各做兩整圈，scale 分別為 0.9857046 / 0.9854675；IMU 交叉檢查在 720°
內的誤差皆小於約 1.1°。raw 資料與計算見
`docs/calibration/ODOM_LINEAR_CALIBRATION_2026-09-24.md` 與
`docs/calibration/ODOM_YAW_CALIBRATION_2026-09-24.md`。每 tick：

1. 以 monotonic clock 取得 `dt`，拒絕 `dt<=0`；過大的 `dt` 不得直接積分，先標 stale。
2. 讀 cached `get_motion_data()`；raw 非有限值、超過物理上限或連續不更新時標 invalid。
3. 套方向與 scale，得到 base frame 的 `(vx, vy, wz)`。
4. 積分：

   ```text
   dx_world = (vx*cos(yaw) - vy*sin(yaw)) * dt
   dy_world = (vx*sin(yaw) + vy*cos(yaw)) * dt
   yaw      = wrap_pi(yaw + wz*dt)
   ```

5. 組 Odometry：pose 在 `odom`、twist 在 `base_footprint`；quaternion 由 yaw 產生。
6. 同一 timestamp 發 `/odom_setmotor` 與 `odom → base_footprint`。
7. 更新 freshness、loop duration、drop count；不可讓 odom thread 阻塞 6 Hz 控制迴圈。

`get_motor_encoder()` 先作交叉驗證與診斷，不在 ticks-per-rev、輪徑、減速比確認前取代
已校正的 `get_motion_data()`。

### 4.3 covariance 與 reset

- covariance 不可全部填 0（那表示完美確定）；先用保守、可設定的 x/y/yaw 值，實測後再調。
- odom reset 只允許在 IDLE、輪速為零且 AMCL 尚未依賴目前 odom 時執行。
- runtime 中重連 rosbridge 不得偷偷把 odom pose 歸零；先緩存最新 pose，重連後續發。

## 5. ROS bridge 與感測資料契約

| 資料 | 方向 | 必要欄位／規則 | stale 行為 |
|---|---|---|---|
| `/odom_setmotor` | pipeline → ROS | 20 Hz 目標；frame=`odom`、child=`base_footprint` | 立即停車；PATROL/DELIVER 暫停 |
| TF `odom→base_footprint` | pipeline → ROS | 與 odom 同 timestamp | 立即停車 |
| `/scan` | ROS → pipeline | latest-only、queue=1；保留 ROS timestamp | 導航立即停車；逾時退出當前導航 |
| `/amcl_pose` | ROS → pipeline | pose、covariance、timestamp | PATROL/DELIVER 立即停；APPROACH/ALIGN 可完成當前近距離動作後停 |
| `map→odom` | AMCL → TF | AMCL 唯一 owner | 地圖目標全部禁用 |

ROS LaserScan 轉 48 rays 時只做一次座標轉換：REP-103 為逆時針，ROS backend 預設
`lidar-dir=+1`；舊 direct RPLidar backend 才預設 `-1`。`lidar-dir` 修手性、yaw offset
修零點，兩者不可互相代替。

## 6. 底盤停止後才能啟動手臂的 stationary gate

`ALIGN → LATCH → GRASP` 前新增不可略過的 stationary gate：

1. actuator 連續送零。
2. 等 feedback 至少連續 3 筆同時滿足候選門檻：
   `|vx|<0.02 m/s`、`|vy|<0.02 m/s`、`|wz|<0.05 rad/s`。
3. 確認最近 odom／scan 資料仍新鮮。
4. gate timeout 時維持夾爪 navigation home，禁止啟動 PPO 手臂，進 PAUSED/FAULT。

門檻是安全起始值，須由靜止 feedback noise 量測後定案。

## 7. 完整任務狀態機細節

| State | Entry action | 正常出口 | timeout／錯誤出口 |
|---|---|---|---|
| BOOT | 驗版本、權重配對、裝置 path | SELF_CHECK | FAULT |
| SELF_CHECK | serial owner、feedback fresh、ROS/TF/scan health | IDLE | FAULT |
| IDLE | 底盤零命令、手臂 navigation home | PATROL（使用者 start） | ESTOP |
| PATROL | AMCL waypoint→dist/bearing；重置 ActionDelay | waypoint 到達／偵測同一 track N 幀 | AMCL/scan/odom stale→PAUSED |
| INVESTIGATE | 只用可信 bearing 慢速靠近 | 距離進入有效區→APPROACH | lost/timeout→PATROL |
| APPROACH | 視覺目標＋RL 局部避障 | stop distance→ALIGN | lost→PATROL；scan stale→PAUSED |
| ALIGN | 手臂相機低速精對位 | handoff→STATIONARY_GATE | lost/timeout→退避或 PATROL |
| STATIONARY_GATE | 零命令＋feedback 停穩確認 | LATCH | timeout→FAULT |
| LATCH | 同一時刻保存物體 base pose、AMCL pose、width | GRASP | invalid target→退避重試 |
| GRASP | PPO 28D/6D；底盤保持零 | VERIFY | policy timeout／servo error→FAULT |
| VERIFY | 判斷物體是否仍在原位 | 成功→CARRY_HOME；失敗→RETRY | 相機失效→保守視為失敗 |
| RETRY | 手臂 home、低速退避、清 delay/tracker | APPROACH（≤3） | 3 次失敗→BLACKLIST→PATROL |
| CARRY_HOME | 切到已驗證攜帶 pose、套攜帶限速 | DELIVER | 物體掉落／pose error→PAUSED |
| DELIVER | AMCL 地圖目標＋中繼 waypoint 鏈 | 進垃圾桶區域→PLACE_ALIGN | AMCL stale/卡住→PAUSED |
| PLACE_ALIGN | stationary gate＋對準放置 yaw | PLACE | yaw/odom timeout→PAUSED |
| PLACE | 固定放置 pose、張爪、收臂 | RESUME | servo error→FAULT |
| RESUME | 清 delay/tracker，取中斷點的下一 waypoint | PATROL | route invalid→FAULT |
| PAUSED | 立即停車、保持安全手臂 pose、記原因 | 人工確認後 SELF_CHECK | ESTOP |
| FAULT/ESTOP | 馬達零命令；ESTOP 不自動移動手臂 | 只能人工 reset | — |

每次目標來源或狀態改變都要重置 `ActionDelay` 和 `GoalTracker`；任何 stale／exception
處理都先停車，再記錄，再決定是否可恢復。

### 7.1 route.yaml 最小資料契約

先支援單一巡邏迴圈；未來要跨房間再增加 graph edges，不改 waypoint 本體：

```yaml
frame_id: map
arrival_radius_m: 0.25
patrol_mode: loop
waypoints:
  - {id: p0, x: 0.50, y: 0.30, yaw: 0.0}
  - {id: p1, x: 1.20, y: 0.30, yaw: 0.0}
trash_bin:
  center: {x: 0.20, y: -0.40}
  radius_m: 0.35
  place_pose: {x: 0.45, y: -0.40, yaw: 3.14}
```

載入時先檢查 frame 必須是 `map`、id 不重複、相鄰 waypoint 距離大於兩倍 arrival radius、
座標是有限值、place pose 位於手臂可達的垃圾桶邊緣。任一錯誤在 BOOT 就 fail，不進 PATROL。

### 7.2 任務恢復資料

每次切離 PATROL 時保存 `interrupted_waypoint_index`；夾取／投放成功後從**下一個** waypoint
續走。PAUSED 只保留記憶體狀態；重新開機預設回 SELF_CHECK＋人工 start，不自動延續上次
未完成任務，避免在未知姿態下自行移動。

## 8. 實作與上機 gates

### A. 底層恢復

- **A0 原檔 gate**：`ai_motor_server_B.py`、launch、Rosmaster_Lib 版本與 SHA256 入庫。
- **A1 serial gate**：`fuser -v /dev/myserial` 全程只有一個 PID。
- **A2 feedback gate**：靜止 5 秒、前／後／左移／右移／左右轉，raw 值持續更新且軸向正確。
- **A3 短距離 gate**：25.4 cm 直走、±90° 原地轉，重現約 2%／數度級誤差。
- **A4 閉合 gate**：小方形或 S 路線回原點，記 position/yaw residual，不以 command odom 兜底。
- **A5 ROS gate**：`/odom_setmotor` 約 20 Hz，frame/child/timestamp/covariance 正確。
- **A6 TF gate**：只有一條 `odom→base_footprint`，且 `tf_echo odom laser` 穩定。

### B. 單程序整合

- **B1 actuator gate**：`SetMotorActuator` 的前後／橫移／轉向與舊 server 實車行為一致。
- **B2 watchdog gate**：停止送命令後在設定 timeout 內強制零；拔除控制 client 也會停。
- **B3 shared-device gate**：導航、feedback、伺服使用同一 handle；`fuser` 仍只有一個 PID。
- **B4 regression gate**：夾取 dry-run、導航 selftest、模型 bench 全過。
- **B5 stationary gate**：底盤未停穩時手臂絕不開始動作。

### C. ROS 定位與任務

- **C1 scan gate**：ROS adapter 的前／左／右 probe 與 raw front brake 都正確。
- **C2 TF/map gate**：靜止 `/scan` 貼圖，移動後無快速累積漂移。
- **C3 AMCL gate**：推車／低速繞場，pose freshness、covariance、重定位可接受。
- **C4 patrol gate**：無 YOLO，至少 10 個 waypoint 完成率與停點誤差達標。
- **C5 target switch gate**：PATROL↔INVESTIGATE↔APPROACH 不殘留上一目標 action。
- **C6 grasp/deliver gate**：夾取、攜帶、放置、續巡各自先通過，再做完整任務。

## 9. 導航權重 A/B 流程

- 現行預設仍為 `ppo_nav_281440_steps`；先拿它建立實車 baseline。
- `doorway_ft_final` 是候選：55D／2D 契約已驗證、開發機可載入，但不得因模擬成績較好
  就直接改預設。
- 兩者必須各自搭配同名 VecNormalize，使用完全相同場地、起點、目標、障礙與速度限制。
- 每組至少記：成功、碰撞、幾何煞停次數、timeout、到點時間、最小障礙距離、控制週期 p95。
- doorway 候選只有在實車成功率不降、碰撞／煞停介入較少且沒有單側偏轉時才能升預設。

## 10. 啟動順序

### 10.1 階段 A（只驗 odom，不跑手臂）

1. `roscore`
2. robot model（robot_state_publisher；無 Rosmaster 底盤 driver）
3. 唯一的 set_motor＋feedback odom 程序
4. LiDAR driver
5. 依序驗 `/odom_setmotor`、TF、`/scan`；不開 AMCL／SLAM 調參

### 10.2 最終 runtime

1. `roscore`＋rosbridge
2. robot model＋LiDAR driver＋map_server＋AMCL
3. 檢查 `/scan`、`map→odom`、除 `odom→base_footprint` 外的靜態 TF
4. 啟動**唯一**主 pipeline（它才開 `/dev/myserial` 並補 `/odom_setmotor`＋TF）
5. pipeline SELF_CHECK 全綠後保持 IDLE
6. 人工下達 start 才進 PATROL

關閉順序相反；主 pipeline 收到 Ctrl+C／exception 時先底盤零命令，再關閉 ROS bridge、
auto-report 和 serial。LiDAR driver 不因底盤故障被誤殺。

## 11. 必須補入 repo 的實機資料

在寫 actuator／odom 程式前，至少需要：

1. `/home/jetson/ai_motor_server_B.py` 真實內容；
2. `setmotor_model_only.launch`；
3. 實機 Rosmaster_Lib 版本或對應檔案；
4. `fuser -v /dev/myserial`、`ps`、`rosnode list` 的當次輸出；
5. LiDAR 確切型號與 `/dev/serial/by-id/`；
6. 四輪順序、正負號與各軸校正紀錄。

沒有這些資料時可以完成介面與單元測試，但不應送出真實 `set_motor` 命令。
