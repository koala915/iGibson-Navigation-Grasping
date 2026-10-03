# Master Bringup Plan

**目前沒有唯一且可重現的全系統 roslaunch 入口。** 可確認的是 Python mission entry，以及外部 ROS services 的介面；本 checkout 沒有任何 `.launch`、`package.xml` 或 `CMakeLists.txt`。本輪不重写launch，不創建會自動啟動馬達的shell wrapper。

## 現有入口與真實依賴

| 啟動項 | 現在可確認的入口 | 先決條件 / 不可同跑 |
|---|---|---|
| ROS master / network | 外部部署 | ROS_MASTER_URI、ROS_IP、主機時鐘與ROS版本待收集 |
| LiDAR | 外部 TG30 ROS driver | actual port、driver參數、scan frame、方向；不能用RPLidar backend代替 |
| TF fixed / robot model | 外部 robot_state_publisher + 本repo URDF（是否實際使用待確認） | 固定joint不得與另一static publisher重複；moving arm需joint_states |
| Map / AMCL | 外部 launch + map檔 | map resolution/origin/hash、base/odom/laser frames、AMCL來源唯一 |
| ROS bridge | 外部 rosbridge_server | Python3 client預設9090；topic/types/connection可用 |
| Cameras | 外部ROS攝影機+web_video_server，或 stream_cam.py（不同URL契約） | pipeline預設8080/stream?topic=...；stream_cam預設8081；不要和UI8080同埠 |
| Motor / odometry | integration/mission_pipeline.py 內建立controller與OdomPublisher | serial soleowner；目前不是兩個可各自roslaunch的node |
| Navigation / patrol / bin | 同一mission entry | route與annotations、AMCL、scan、model對、travel/grasp pose；不用再啟另一PPO或move_base控制器 |
| Standalone v21 grasp | grasp/v21/jetson_one_command_grasp.py | 獨立排他模式；manifest與實測校正、既有real gates |
| Standalone v23 candidate | grasp/v23/jetson_one_command_grasp.py | E1/scan配置與候選解鎖；不和mission同開serial |
| UI | bash ui/launch_ui.sh → ui/launcher.py → ui/server.py | Python環境、port和argv問題需先修；B模式不是自足的完整流程 |

## 需取回的 deployment bundle

這些是明確缺件，不是要求先讓機器人動作：

- robot 上實際使用的 LiDAR、camera、map_server、AMCL、navigation/底盤 launch 與include樹；`rosparam dump`、node/topic/service清單、TF authorities。
- 實際 Rosmaster_Lib 源碼/版本、serial symlink到USB實體的映射、board firmware、外部 ai_motor_server_B/Route A 程式。reference.py不是被runtime import的已驗證安裝版本。
- Route C route.yaml、annotations、map YAML+image、resolution/origin與hash；文件提到117waypoints、0.049m/cell不等於本輪已讀到檔案。
- Jetson Python、ROS distro、venv requirements、torch/CUDA/SB3/roslibpy/OpenCV實際版本；相機/雷達校正與四方向orientation evidence。

## 建議啟動順序與readiness

1. 選profile並驗model/map/route/calibration身份；檢查目前誰持有serial、port與TF。初始motion保持DISARMED。
2. 啟動外部ROS master、唯一LiDAR driver、相機與唯一fixedTF/model publisher。收到新鮮有效scan與指定stream資料才算ready。
3. 啟動rosbridge，確認message types/frames與預期一致；做clock/network檢查。
4. 啟動唯一mission runtime並持有serial，只允許zero wheelcommand與必要的受控initialization。啟動feedback reader與odom publication；檢查封包sequence前進、stationary可信、odom不跳動。**目前程式尚未完整提供這個分階段DISARMED bringup介面，需要PhaseB補齊。**
5. 啟動map_server/AMCL；待odom TF與scan齊備、initialpose初始化、covariance與age通過。不能要求AMCL ready在所有odom publisher之前，否則造成啟動相依死結。
6. 裝入route、檢查bin approach/forbidden segments與姿態；驗證相機pose對應校正。
7. 操作員明確start後才啟用選定mission source。EOF、UI失聯、node restart都不自動解除DISARMED/FAULT。
8. UI最後接上狀態與start/pause/stop。UI不是readiness的替代品，不因按鈕可按就推論機器人ready。

ROS Melodic native nodes維持原環境；Python3.8+任務程式走自己的venv與rosbridge。第一步可用薄shell/profile supervisor管理混合runtime，不必為了叫master就把所有Python包進catkin或改成Python2。

## 唯一設定來源建議

以下是**待建立的profile欄位**，不是現存檔名或可直接執行指令。

| 設定 | 現值 / 分散來源 | Canonical source 建議 |
|---|---|---|
| motor port | DeployConfig/CLI，通常/dev/myserial；外部實際裝置未查 | profile serial.device_identity + port，adapter唯一開啟 |
| LiDAR port/backend | ROS主線無需Python直接開port；legacy rplidar_port空 | 外部driver profile；mission只選scan topic/backend |
| frame chain | ros_io map/odom/base_footprint + URDF base_link/laser_link | URDF與external ROS profile，preflight對照唯一TF |
| LiDAR yaw/dir/offset | NavRLConfig 0deg/+1/0m；URDF x=.0435m | measured orientation evidence綁profile，避免Python與TF各自補兩次 |
| odom scale/timeouts | FeedbackOdomConfig .65/.501/.501、.30s、maxdt.5s | 版本化feedback calibration；packet age與command deadline分開 |
| velocity limits | NavRLConfig .5 forward/.15 reverse/1.2yaw；vision KX/KZ/min24 | policyplant保留一份；安全limit另有明確physical envelope，不混PWM與SI |
| safety distances | NavRLConfig brake / near slowdown；vision nav_stop；arm envelopes | 區分感測有效性、煞車、policy shaping、graspreach，各有單一定義與理由 |
| map/route | MapGoalProvider external Route C hint；CLI --route/annotations | mapbundle hash + routebinding，不寫死另一台PC絕對路徑 |
| grasp homes/model | mission NAV_HOME/C3常數、v21/v23config/manifest | deploymentmanifest綁contract/home/URDF/model對 |
| camera/homography | arm_cam_geometry C3/E1、homography檔、stream URLs | calibration identity + stream identity + active pose |
| console/camera/bridgeports | console8080、webvideo8080、stream_cam8081、rosbridge9090 | profile定不同service endpoint，啟動前binding檢查 |
| watchdog | 有多種資料timeout，無已驗證的獨立wheelcommand deadline | adapter deadline + board watchdog兩層，各有測量驗收 |

## 關閉與驗收

先撤銷motion lease、下zero並latch停止，再停止mission來源；確認停止或保持hardware estop後依次關reader/odom、camera、ROS連線、serial；每個cleanup獨立執行。修B07/B20前不能假定normal shutdown已完整清理。

可宣稱正式bringup前，需保存一份「profile與所有artifact hashes、singleowner/TFauthority清單、idle資料紀錄、controlled motion結果與stop latency」。沒有這份證據，目前只能称整合候選主線。
