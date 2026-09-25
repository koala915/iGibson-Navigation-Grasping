# Control Ownership Analysis

目前只有 **單一 MissionRunner 內的 FSM 排程**，沒有涵蓋 ROS/TCP/其他程序的正式 command arbitration。序列埠 owner 的人工確認，不等於作業系統鎖；UI 的一個 child 限制也管不到終端機或外部 ROS launch。

| Source | Command | Destination | Current priority | Conflict risk / 建議 |
|---|---|---|---|---|
| Mission patrol / DRIVE_BIN | PPO shaped vx,vy=0,wz | MissionNavigator → shared Rosmaster.set_car_motion | FSM state 選擇；無外部優先權 | 可與外部 move_base driver / TCP server 抢同 serial；mission mode 必須排除其他 writer |
| Mission FINE_ALIGN / creep | legacy action/speed → SI vx,wz | 同一 shared device | FSM state；額外 stationary/vision/lidar gate | 不是獨立程序 conflict，但速度與 freshness gate 有 B08/B10/B11 |
| Mission retry back_off | timed negative vx | 同一 shared device | recovery 分支直接發送 | rear coverage 與 watchdog 未閉合 B21 |
| Mission stop / fault / UI pause | stop / stdin control / termination | MissionRunner，最後嘗試 set_car_motion(0,0,0) | 只在程序執行到 handler 時有效 | 主迴圈阻塞或 pipe異常不能當 hardware estop；需 motor deadline |
| Mode C RL standalone | PPO → set_car_motion | 自己建立的 GraspController device | 無 | 與 Mode A 同時跑會有 serial ownership conflict；目前 nav-only 也有 B18 |
| vision_grasp_pipeline standalone | camera nav action → set_car_motion | 自己的 grasp device | 無 | 已有 real integrated refusal；不能繞過 gate 用其他入口 |
| v21 / v23 standalone grasp | servo angles / grip / home / servo reads | 各自 Rosmaster handle | 與底盤指令無統一 ownership | 對同一 board 開第二handle，回包也可能互搶；two versions 必須互斥 |
| vision_grasp_bridge / offboard target | TCP object pose / ROS trash target | receiver / mission perception | 無 wheel priority；**不是直接 motor writer** | 可間接觸發 grasp/mission，需 pose stamp、latch、freshness與目標可信度 |
| detection/arm_center_setmotor.py、rear_nav/rear_to_arm_blind_handoff.py | action + speed | TCP motor server :7000（外部） | server政策未知 | 不能假定它有 client lease/watchdog，需取回servercode |
| calibration/calibrate_mecanum_setmotor.py、calibrate_vy_only.py、calibrate_wx_only.py | custom motor / motion payload | 同一外部 TCP service | 未知 | TEST 仍可驅動實體底盤；不能當 offline regression 執行 |
| debug_tools 的 photo / multiple_ball_detect 等 | TCP navigation commands（依腳本） | 外部 TCP service | 未知 | 名稱包含 photo/debug 不代表只讀；逐檔隔離 |
| detection/nav_arm_cam.py | /cmd_vel Twist（含 dry zero） | 外部 ROS driver | ROS topic 無來源優先權 | 最後到達命令可互相覆蓋；本 repo 沒有 cmd_vel mux |
| grasp/move_arm.py、servo_test.py、joint_direction_calibration.py、pose_check/bus_probe | 直接 servo API / bus probes | Rosmaster serial | 無跨程序 lock | 排他 maintenance mode，單獨 owner；不可和任務並跑 |
| v17／reference／training／deploy copies | 舊 servo / Arm_Lib 操作 | legacy board API | 無 | 某些 Arm_Device 在 import 即建立；不可全量 import discovery |
| 外部 keyboard / teleop / move_base / ai_motor_server_B | repo未實作，文件有外部路徑 | 外部底盤 driver | **無法判定** | 需實際 ROS graph/process/serial inventory 才能證明不存在競爭 |

## 應採用的最小 ownership contract

1. 每個啟動 profile 指定 `mission_rl`、`maintenance` 或 `standalone_grasp`，同一實體 board 同時只能一個 profile 持有 serial。USB symlink 不同但指向同裝置也須識別。
2. 初期保留 mission 已有單一 handle，加入薄 MotorAdapter，不先另開 TCP motor server。Adapter 是唯一 `set_car_motion` 出口，接受來源、generation、deadline、SI速度，回報 applied/blocked/reason。
3. 建議優先序：**physical estop / fault latch > stale data / command timeout > operator stop > selected mission source**。manual 如未實作完整模式切換，不允許與 autonomy 同時啟用。
4. Navigator、fine-align、reverse recovery、exception stop 都經 adapter；arm 持有同一 owner 的受控 interface，真實新鮮 stationary 才可離開 travel pose。清 timeout 不自動恢复上個 nonzero command。
5. 若未來改用 move_base，先選定唯一的 cmd_vel consumer 與 arbitration，再接 goal action；不能只把 move_base launch 加進目前 direct-write 模式。

這是建議優先權，**不是目前已存在的功能**。ROS topic 的 queue_size、TCP connect 先後或 mutex 只保證單次呼叫順序，不構成控制權仲裁。Python thread lock 也不解決跨程序 serial ownership。

## 停止與故障驗證矩陣

離線先注入：inference卡住、scan停止、feedback序號停止、AMCL變舊、UI EOF、thread exception、serial write exception、client disconnect。必須驗證「期限後不接受舊命令，重啟不自動恢復 motion」，而非只 assert 呼叫过 stop。板端拔線後停止延遲、process kill、電源急停、制動距離需要後續受控實機測試。

相關證據：[B01–B08、B21、B25](BUG_AND_RISK_REGISTER.md)、[ACTUAL_RUNTIME_ARCHITECTURE](ACTUAL_RUNTIME_ARCHITECTURE.md)。
