# RL 導航 → 夾取整合（nav_rl_grasp_pipeline.py）

把另一台電腦訓練的 **PPO 導航策略**（iGibson `Rs_int`、48 束 LiDAR）
接上既有夾取流程。權重與說明來源：`igibson_x3_test` repo `nav_best_model/`（分支
`nav-best-weights-export`），已複製到本 repo `integration/nav_best_model/`。

## 檔案

| 檔案 | 說明 |
|------|------|
| `nav_rl.py` | 執行期核心：55D obs 組裝、**訓練 plant 復刻**（2 步延遲、stall assist、近障調速、油門映射）、LiDAR→48 束轉換、VecNormalize 手動正規化、幾何煞停、`--selftest/--probe/--bench` |
| `nav_rl_grasp_pipeline.py` | 全流程：YOLO 目標→GoalTracker（偵測間 dead-reckoning）→RL 導航避障→視覺精對位（沿用 vision_grasp_pipeline 的 ARM_ALIGN）→latch→PPO 夾取→驗證/重試 |
| `g2_nav_client.py` | G2 常駐服務相容的**導航專用短直線驗收端**：ROS scan/odom + 0.42 m 煞車 + TCP 7000；不持有 serial、不轉彎、不夾取；見 `../docs/operations/G2_NAV_CLIENT_2026-09-28.md` |
| `nav_best_model/ppo_nav_281440_steps.zip` + `ppo_nav_vecnormalize_281440_steps.pkl` | ★預設模型（**必成對**）。訓練機評測（seed 123×30，2026-07-10）：成功 0.600、碰撞 0.267，兩項都贏 |
| `nav_best_model/doorway_ft_final.zip` + `doorway_ft_final_vecnormalize.pkl` | ★候選模型（**尚未升預設**）。標準任務 60 episodes：成功 0.683、碰撞 0.183；doorway 10/10。與現行相同 55D／2D／plant 契約，必須上機 A/B 後才決定 |
| `nav_best_model/best_model.zip` + `best_vecnormalize.pkl` | 備用 checkpoint。同評測：成功 0.467、碰撞 0.367——**上機不要用**，只留作對照 |

## 流程

```
1. 後桅相機 YOLO 找到物體 → GoalTracker 記 (dist, bearing)
2. PPO 策略 6Hz 控制 set_car_motion(vx, 0, wz)，LiDAR 48 束避障
   - 偵測掉幀時 tracker 用指令速度 dead-reckoning（策略轉彎避障時目標可暫離視野）
   - 幾何煞停：正前 ±50° 原始距離 < 0.25m → 禁止前進（策略模擬碰撞率仍 0.2–0.27，煞停必開）。
     上機時必須量測觸發到靜止的停止距離，並保留大於此距離的空地安全裕度。
3. tracker dist ≤ 0.75m → 停車確認 → ARM_ALIGN 精對位（手臂相機，0.7→0.24m）
4. handoff → latch 座標/寬度 → PPO 夾取 → verify，失敗退避重試 ≤3 次
```

> 現行 actuator 仍是 `set_car_motion`。自製 `set_motor` 尚未接入；正式替換時必須保留
> 連續 `(vx,vy,wz)` 契約、6 Hz control period、2-step ActionDelay 和同一 Rosmaster serial owner。
> 只有 forward／turn／curve＋speed 的離散 TCP 命令不足以直接承接此 policy。細節見
> `SETMOTOR_ODOM_INTEGRATION.md`。

## G2 後的模式 C 決定（2026-09-28）

選擇 **(a)：把模式 C 改成 TCP 7000 的速度格式用戶端**，而不是長期保留第二個
`/dev/myserial` 持有者。原因如下：

- G2 的目的就是讓導航、接近與夾取共用一個常駐 serial owner，避免每次切換付出約
  11.6 秒重啟成本；模式 C 若繼續直接 `set_car_motion`，仍會破壞這個目標。
- 模式 C 已經產生連續的 `vx`（m/s）與 `wz`（rad/s），可直接對應既有
  `{"action":"velocity","vx":...,"wz":...}` 協定，不需要改 PPO action、6 Hz 週期、
  2-step delay 或 `velocity_to_motor_values`。
- TCP 7000 的 0.5 秒 watchdog 與常駐服務內的輪／臂互斥規則，會讓用戶端斷線及夾取
  期間都 fail closed。

目前已新增獨立的 `g2_nav_client.py` 驗收端，具備經 G2 常駐服務送速度的
用戶端與幾何煞車；**尚未執行其實車移動，也未接入模式 C 的 PPO／相機流程**。
`nav_rl_grasp_pipeline.py` 維持現狀，仍須與 `grasp-service` 互斥，不能把
`g2_nav_client.py --probe` 當成模式 C 實車驗收。

G2 odom、TF 與 AMCL 的上機驗收步驟見
[`../docs/operations/G2_ODOM_TF_AMCL_ACCEPTANCE_2026-09-28.md`](../docs/operations/G2_ODOM_TF_AMCL_ACCEPTANCE_2026-09-28.md)。

## 指令

```bash
# 開發機/Jetson：純邏輯自測（免 torch/相機/硬體）
python3 integration/nav_rl_grasp_pipeline.py --selftest

# Jetson 終端機 1：TG30 driver（已實測 /scan 約 10.17Hz）
source /opt/ros/melodic/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
roslaunch ydlidar_ros_driver TG.launch

# Jetson 終端機 2：Python 3.8 與 ROS Melodic 的 WebSocket bridge
source /opt/ros/melodic/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
roslaunch rosbridge_server rosbridge_websocket.launch

# Jetson 終端機 3：Route B 四方向 gate（只讀，不會驅動馬達）
cd ~/Documents/deploy_jetson2
source ~/grasp_venv/bin/activate
# 先依 Navigation/03_route_b_localization_navigation_handoff/LIDAR_ORIENTATION_GATE.md
# 完成 front/back/left/right，建立 ~/.route_b_runtime/scan_orientation_verified
python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1

# Jetson：模型載入 + 推論速率
python3 integration/nav_rl.py --bench

# 候選 doorway 權重：只做載入/維度/速率 smoke test
python3 integration/nav_rl.py --bench \
  --model integration/nav_best_model/doorway_ft_final.zip \
  --vecnorm integration/nav_best_model/doorway_ft_final_vecnormalize.pkl

# 乾跑（不驅動硬體；需相機串流；--no-lidar 只給測試，--real 會拒絕）
python3 integration/nav_rl_grasp_pipeline.py --no-lidar

# 正式（TG30 driver 與 rosbridge 要跑；會占用 /dev/myserial 的底盤 driver/port 7000 不可跑）
python3 integration/nav_rl_grasp_pipeline.py --real --show \
  --lidar-backend ros --ros-host 127.0.0.1 \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --cam-x <Phase3_X> --cam-y <Phase3_Y> --sign-y <1或-1> \
  --i-confirm-camera-frame
```

`--bench` 的單筆 action 只用來確認模型可載入、shape 正確和推論夠快，不能由該 action
判斷實車策略好壞。實車先跑預設權重建立 baseline，再用相同起點／障礙／目標以顯式
`--model`＋`--vecnorm` 測 doorway 候選；未通過前不要改 `DEFAULT_MODEL`。

## Jetson 安裝

```bash
source ~/grasp_venv/bin/activate
python3 -m pip install roslibpy

# ROS 端確認；只有找不到時才安裝下一行套件
rospack find rosbridge_server
# sudo apt install ros-melodic-rosbridge-server
```

本機已確認是 **YDLIDAR TG30**（512000 baud、20K sample rate），因此**不需要也不應安裝
`rplidar-roboticia`**。`/dev/rplidar` 只是出廠 udev 別名，不代表硬體是 Slamtec RPLidar。
舊 `--lidar-backend rplidar` 僅保留給其他已確認的 Slamtec 硬體，且現在強制要求顯式
`--lidar-port`，避免誤開 TG30。

## ⚠️ 上機前校正清單（依序）

1. **LiDAR 硬體與資料已確認（2026-07-16）**：TG30、firmware 2.1、health good，
   但這些 metadata 不等於 policy 前方語意已確認。AMCL 使用 TF，policy 可能直接讀 raw
   `/scan` index；必須先完成 Route B 的四方向板子 gate。
2. **orientation gate 通過後才可固定參數**：證據 marker 的 frame、TF yaw、policy offset
   必須與目前 launch/adapter 相符。左右反轉或整體偏轉時，先刪 marker、停車、修正後重測；
   不可邊猜 `--lidar-dir`/`--lidar-yaw-offset-deg` 邊開巡航。LiDAR 不在車中心才另外設定
   `nav_rl.NavRLConfig.lidar_forward_offset_m`。
3. **轉向正負**：`+wz` 必須讓車**左轉**（策略慣例）。dry-run 看 log、實測反了 → `--wz-sign -1`。
4. **控制週期**：預設 1/6s（訓練值 action_repeat 5×1/30 推算）。實車行為抖/遲鈍時微調
   `--control-period`；**不要動 plant 參數**（stall/調速/延遲是訓練動力學的一部分）。
5. **相機幾何**：沿用 vision_grasp_pipeline 的 THETA/H/FX 常數（本來就標注待校正）；
   nav 目標距離 = 相機前向距離，與 sim 的「車中心距離」差半個車長，保守方向（會早停）。
6. **速度上限**：策略全速 0.5 m/s 比現有手調導航（0.22）快。首測建議加大 `--nav-stop-dist`
   （如 1.0）觀察行為，正常再降回 0.75。

## 已在開發機驗證

- `--selftest` 全過（含閉環玩具模擬收斂、煞停優先權）。
- **真權重虛擬 episode 4/4 成功**（正前 2m／左前 3m／右前 1.5m／右急彎 2.5m，
  全鏈路：VecNormalize→policy→2 步延遲→plant 復刻→tracker），推論 4.7 kHz（CPU）。
- 語意煙霧測試：目標偏左→+wz 左轉；右側 0.45m 障礙→倒退+左轉（避障行為存在）。
- **Jetson 軟體組合預驗（2026-07-10）**：在 **Python 3.8.20 + SB3 2.3.2 + torch CPU**
  （= grasp_venv 的組合）下 `--bench` 載入零警告、`--selftest` 全過，
  且清空 2m 目標的動作輸出與 Py3.12/SB3 2.9.0 **逐位一致** `[1.0, 0.43397042]`
  ——上機清單的「SB3 2.3.2 載入測試」已可視為預先通過，上機時跑一次 `--bench` 確認速率即可。
- 權重檔與訓練機 export（igibson_x3_test `nav-best-weights-export`）SHA256 一致。
- **doorway 候選電腦端預驗（2026-07-16）**：55D obs、2D action、VecNormalize 55D
  契約一致，模型可載入且推論速度遠高於 6 Hz；此結果不取代上機 A/B。

## 設計要點（為什麼這樣做）

- **plant 復刻**：策略是在「2 步延遲 + stall assist + 近障調速」的動力學下訓練的，
  部署端不復刻 = 動力學不一致 = 策略失效。這些參數在 `NavRLConfig` 有訓練值，**勿調**。
- **obs 用指令端速度**：sim 用 `resetBaseVelocity` 強制速度，實際速度=指令速度，
  故部署 obs[3:5] 直接用整形後的指令值（與夾取端「obs 建自指令端狀態」同哲學）。
- **煞停用原始點雲**：48 束 obs 下限被 clip 在 0.33m，看不到更近的障礙；
  幾何煞停必須用 LiDAR 原始距離。
- **實機 footprint 煞停（2026-09-27）**：導航／搬運姿態實測約
  `0.32 × 0.24 m`，LiDAR 位於底盤中心前方 `0.10 m`、離地 `0.095 m`；
  motor 30 在送出 stop 後仍滑行約 `0.056 m`。前進煞停因此使用機器人中心前方
  `0.38 m` 的停止面，以及左右各 `0.22 m` 的固定寬掃掠走廊。走廊外的側牆不會
  阻擋直行，但前方仍保有約 10 cm 的目標淨空。TG30 原始 `180°` 才是車頭方向，
  正式 pipeline 預設 `--lidar-yaw-offset-deg 180 --lidar-forward-offset-m 0.10`。
