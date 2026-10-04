# 交接：巡航 → 辨識 → 前往 → 夾取 → 丟垃圾

> 打包日期 2026-08-04；狀態更新 2026-08-07。這份是接手的人第一個要看的檔案。

## 一分鐘版

X3Plus 完整任務的整合。**程式整合已完成並在電腦端全部驗證通過；實車驗證已開始。**
目前已有 v21 夾取核心，以及 T6 最簡化垃圾桶投放動作（伸手 → 開爪 → 回 home、物品目視進桶）
的實機成功紀錄；巡航、導航與交接流程仍待上車驗證。

```bash
# 任何一台電腦都可以跑，不需要機器人。約 100 秒。
python3 integration/preflight.py --offline
# 期望：OFFLINE: 20 pass, 0 warn, 0 fail
```

綠了就代表程式是完整的，接下來只剩實車。實車步驟見 [`integration/TEST_PLAN.md`](integration/TEST_PLAN.md)。

---

## 進度

分兩個數字講，因為它們是不同的事：

| | 進度 | 依據 |
|---|---|---|
| **程式整合** | **~95%** | 124 個測試 + preflight 20 項全綠；完整任務在假硬體上跑通 |
| **實車驗證** | **~30%** | v21 夾取核心與 T6 最簡化垃圾桶投放已在實車成功；巡航、導航及交接仍待驗證 |

約 30% 是依六個上機關卡（T1–T6）的證據粗估：T5 的**夾取核心**已被證實，
T6 的**最簡化投放動作**也已於 2026-08-07 在實車目視確認成功。這不代表從巡航、導航、
交接到投放的完整端到端流程已通過；T1–T4 與 T5/T6 的整合部分仍需逐關驗證。

剩下的 5% 程式落差不是「還沒寫」，是**三件只能在車上決定的事**：

1. LiDAR 的 180°（兩份文件互相矛盾，程式已加自動偵測但正確值要現場量）
2. 手臂會不會被自己的 LiDAR 看成障礙物（URDF 粗估說可能，需實測）
3. AMCL 丟失定位的自動恢復（**未實作**，目前會停下等人）

### 已完成

| 層 | 檔案 | 狀態 |
|---|---|---|
| 地圖/路線 | `integration/map_goal_provider.py` | route.yaml 117 點 → 重取樣 83 點、到點、繞行、中斷續巡、禁區 |
| 里程計 | `integration/feedback_odom.py` | `get_motion_data()` → pose，Route A 校正值，精確弧長積分 |
| ROS 介面 | `integration/ros_io.py` | 發 `/odom_setmotor` + TF，收 `/amcl_pose`（含發散門檻） |
| 任務狀態機 | `integration/mission_fsm.py` | 20 狀態，純邏輯 |
| 主程序 | `integration/mission_pipeline.py` | 單一 Py3.8 程序獨佔 `/dev/myserial` |
| 導航策略 | `integration/nav_rl.py` | 55D obs、訓練 plant 復刻、48 束、幾何煞停（既有） |
| 夾取 | `grasp/v21/` | v21 + 移植進來的放下動作 |
| 起飛前檢查 | `integration/preflight.py` | 離線 20 項 + 上機 5 項 |

### 未完成

| 項目 | 現況 |
|---|---|
| AMCL 丟失定位自動恢復 | **未實作**。發散 → covariance 超標 → PAUSED 等人重設初始位姿 |
| 手臂相機 C3 外參 | 是 URDF **預測值**不是量測值。依 2026-08-04 決定不處理（v21 用這組參數實機夾成功） |
| 放下垃圾的驗證 | 沒有。`released` 只代表動作跑完，不代表東西在桶裡 |
| 靜止 feedback 雜訊門檻 | 暫定值（0.02 m/s、0.05 rad/s），沒實測過 |

---

## 這是什麼架構

單一 Python 3.8 程序擁有 `/dev/myserial`；ROS Melodic 只做感測與定位，**不得開底盤序列埠**。

```
本程序（唯一 /dev/myserial owner）        ROS Melodic（Python 2.7）
  GraspController → 伺服機（v21）           robot_state_publisher
  set_car_motion  → 輪子                    YDLIDAR TG30 → /scan
  get_motion_data → /odom_setmotor + TF     map_server → /map
  導航 PPO 55D→2D、夾取 PPO 28D→6D          AMCL → map→odom
  MissionFSM                                rosbridge_server
```

會這樣切，是因為手臂和輪子**共用同一條 UART**（`Rosmaster` 只開一個 serial port，`set_motor` 和 `set_uart_servo_angle_array` 都寫它）。Navigation 交接包寫「夾取系統使用 I2C（獨立）」是錯的 —— 照那個做，v21 夾取程式連手臂都動不了。

詳細流程、狀態表、手臂姿態切換見 [`integration/MISSION.md`](integration/MISSION.md)。

---

## 給接手的人：四個一定要先讀的坑

**1. 自動回報沒開 = odom 永遠是 0，而 AMCL 會相信它**

v21 的 `ServoController` 呼叫 `create_receive_threading()` 但**沒有** `set_auto_report_state`。沒開的話 `get_motion_data()` 永遠回傳初始的 0，odom 看起來就是一台從不移動的機器人。程式已在自檢補上並會拒絕啟動，但你要知道為什麼有這段。

**2. LiDAR 的 180° 沒有定論**

Jetson 上的 launch 註解說 `/scan` 的 frame 是 `laser`（相對機器人轉了 180°），主 repo 的紀錄說是 `laser_link`。兩邊量的是不同 launch，可以各自為真。AMCL 走 TF 所以永遠正確，但 `nav_rl.py` 直接讀 `/scan` 原始角度、**不碰 TF**。搞錯的話 policy 的「前方」是車後方，前方全讀成淨空、煞停永不觸發，而且**不會有任何錯誤訊息**。

**覆蓋率不能當證據**：360° 掃描前後都涵蓋，所以它永遠通過。frame_id 也不行，AMCL 正常更不行（它走 TF）。唯一能決定 0° 還是 180° 的是 Route B 的四方向板子測試 —— 板子放前／後／左／右各錄一次，verifier 會**自己量出** offset，你填的 `--offset-deg` 跟量測不合就拒發 marker。`--real` 沒有這份 marker 不會啟動。詳見 `TEST_PLAN.md` T2。

**3. AMCL 只在「有動」的時候更新，而夾取會靜止兩分鐘**

AMCL 靜止時不發 `/amcl_pose`，但夾完的下一個狀態（DELIVER）把過期的 fix 當成阻斷性故障。任務層自己呼叫 `/request_nomotion_update` 解決，並在手臂佔住主迴圈之後補等一次新 fix。**前提是 rosbridge 有帶 rosapi、AMCL 真的提供那個 service** —— 沒有的話夾成功一次就停在 PAUSED，而且按 Enter 也救不回來（self-check 卡在同一個過期 pose）。self-check 和 `preflight --onboard` 都會擋，T0 先用 `rosservice list | grep request_nomotion_update` 確認省得白跑。

**4. route.yaml 不能直接用**

Route C 的 `route.yaml` 宣告 0.75 m 間距，實際最小 **0.049 m**（一個地圖格），117 個間隔裡 45 個不到 0.5 m，`patrol_001` 和 `patrol_116` 座標完全相同。程式預設 `--resample-m 0.75` 會重取樣成 83 點、最小間距 0.530 m。取樣點都落在原折線上，不會切出安全走廊。

---

## 怎麼跑

```bash
# 0. 電腦端先確認程式完整
python3 integration/preflight.py --offline

# 1. Jetson：確認沒有別的程序佔序列埠
sudo fuser -v /dev/myserial

# 2. Jetson：ROS 端（各自終端機）
roscore
roslaunch ydlidar_ros_driver TG.launch
roslaunch <robot model>.launch          # robot_state_publisher
rosrun map_server map_server site_map.yaml
roslaunch amcl.launch                   # RViz 用 2D Pose Estimate 設緊初始化
roslaunch rosbridge_server rosbridge_websocket.launch   # 要含 rosapi

# 3. Jetson：兩個一定要有的東西
rosservice list | grep request_nomotion_update    # 沒有 → 夾完會停住，先解決
ls ~/.route_b_runtime/scan_orientation_verified   # 沒有 → 先做 TEST_PLAN.md T2 四方向

# 4. Jetson：上機前檢查
source ~/grasp_venv/bin/activate
python3 integration/preflight.py --onboard --ros-host 127.0.0.1

# 5. 分段驗證（照 TEST_PLAN.md，一關一關來）
#    先只跑巡航，不夾取也不送桶：
python3 integration/mission_pipeline.py --real --show \
  --no-deliver --detection-streak 999 --max-laps 1 \
  --route <route.yaml> \
  --i-confirm-serial-owner \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --i-confirm-arm-cam-pose
```

`--real` 會在自檢通過後**等你按 Enter** 才開始巡航。

---

## 檔案地圖

```
integration/
  preflight.py          ← 先跑這個
  MISSION.md            ← 架構、狀態表、手臂姿態
  TEST_PLAN.md          ← 上機七關 + 紀錄表
  mission_pipeline.py   ← 主程序
  mission_fsm.py        map_goal_provider.py  feedback_odom.py  ros_io.py
  nav_rl.py             vision_grasp_pipeline.py  nav_rl_grasp_pipeline.py
  nav_best_model/       導航 PPO 權重
grasp/v21/              夾取 v21（含 manifest 與 sha256）
detection/models/       YOLO
tests/
  test_safety_guards.py       59 項
  test_mission_end_to_end.py  10 項，整套任務在假硬體上跑一遍
```

外部依賴（**不在這個包裡**）：Route C 的 `route.yaml` 與 `site_map.yaml/png`，在
`Navigation/02_route_c_external_map_handoff/`。

---

## 重要聲明

v21 的 `manifest.json` 標記 `status: candidate`、`protocol_valid: false`（正式評估的 home
jitter 沒照協議跑）。成績是真的但**沒有認證**。報告、簡報、口試都要照這個講法，不要寫成
「已驗證」或「ready for real robot」。
