# 上機測試計畫 — 巡航到丟垃圾

> **執行狀態（2026-09-06）：暫停在離線整合。** `mission_pipeline.py --real`
> 現在會在任何裝置開啟前拒絕，因為 final align/latch 尚未使用量測過的 grasp-home
> homography。本文的 T0–T6 是解除該封鎖後的分段實機驗證順序，不是目前可直接執行
> 的 bringup 指令。

離線能驗的都綠了（124 個測試 + preflight 20 項，跑 `integration/preflight.py --offline` 一次確認）。
這份是**只能在實車上驗**的部分，按依賴順序排。
每一關沒過就不要往下跑：後面的失敗會被前面的問題污染，浪費的是現場時間。

規則：**一次只改一個變因**。任何方向異常立刻斷電，不要讓 policy 自己「恢復」。

---

## T0 — 開機前（每次都做，2 分鐘）

```bash
sudo fuser -v /dev/myserial          # 必須剛好一個 PID，或空的
ps -ef | grep -E '[r]osmaster_main|[M]cnamu_driver|[a]i_motor_server_B|[r]oute_a_runtime'
```

有其他程序佔著就先殺掉。手臂和輪子共用這條 UART，兩個程序開它 = 命令交錯。

ROS 端起來（各自終端機）：`roscore` → TG30 driver → robot_state_publisher → map_server →
AMCL → rosbridge（**要含 rosapi**）。RViz 用 **2D Pose Estimate 設緊初始化**。

```bash
rosservice list | grep request_nomotion_update    # 必須有這一行
```

沒有這個 service 就不要往下跑。AMCL 只在有動的時候更新並發佈 `/amcl_pose`，
夾取一次會讓底盤靜止兩分鐘以上；任務層靠這個 service 在靜止時把 fix 叫回來。
少了它，**第一次夾成功之後就會停在 PAUSED，而且按 Enter 也救不回來**（self-check
會卡在同一個過期 pose）。`--real` 下 self-check 會擋，這裡先確認省得白跑。

**通過條件**：`rostopic hz /scan` 約 10 Hz，RViz 看得到地圖和粒子雲，
`rosservice list` 有 `/request_nomotion_update`。

---

## T1 — odom 活著（風險最高，先驗）

這是整個專案唯一「壞掉但看起來正常」的環節。v21 的 `ServoController` 不會開自動回報，
沒開的話 `get_motion_data()` 永遠回傳 0 —— odom 會顯示成一台從不移動的機器人，**而 AMCL
會相信它**。`mission_pipeline` 的 self-check 補了這個並會拒絕啟動，這關就是驗它有生效。

```bash
source ~/grasp_venv/bin/activate
python3 integration/ros_io.py --probe --ros-host 127.0.0.1     # 先確認 AMCL 有在發
python3 integration/mission_pipeline.py --real --no-deliver --detection-streak 999 \
  --max-laps 0 --route <route.yaml> \
  --i-confirm-serial-owner \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --i-confirm-arm-cam-pose
```

自檢跑完會停在等你按 Enter。**這時先不要按**，另開終端機：

```bash
rostopic hz /odom_setmotor                  # 應該約 20 Hz
rostopic echo -n1 /odom_setmotor/child_frame_id   # 應該是 base_footprint
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map base_footprint        # AMCL 有接上才會有
```

然後**用手推車前進約 50 cm**，看 `/odom_setmotor` 的 x 有沒有跟著長。

**通過條件**
- self-check 印出 `auto-report enabled` + `board is reporting` + `wheel feedback valid`
- `/odom_setmotor` 約 20 Hz，`odom→base_footprint` 只有一個發布者
- 手推 50 cm，odom 走約 50 cm（±10%；Route A 實測方形閉合誤差 7.8 cm）

**失敗代表什麼**
- 沒有 `board is reporting` → 自動回報沒開或主機板沒回話，**不要往下跑**
- odom 完全不動 → 同上
- odom 動但方向反了 → 校正常數或輪序有問題，回頭查 `feedback_odom.py` 的 scale

---

## T2 — LiDAR 方向（最容易「壞掉但看起來正常」的一關）

### 矛盾在哪

只在**一個事實**上：`/scan` 掛在哪個 frame，因此那 180° 補過了沒有。

| 來源 | 說法 |
|---|---|
| Jetson `setmotor_model_only.launch:14` 的註解（Route B 從實機 grep 貼回） | **`/scan` 的 frame_id 是 `laser`** |
| Route B `tf_confirmed.yaml` / TF log / `_yaw180.launch.partial` | `laser_link → laser` yaw = **180°**（同檔舊版是 0，`version_history` 記載改過） |
| Route B `KNOWN_ISSUES` #3 #4 | 「LiDAR 反向 180°」「**TF 與 policy raw-scan offset 都需保留**」 |
| Route B `doorway_ft_contract.yaml` | policy 端要 `source_angle_offset_deg: 180.0` |
| 主 repo `NAV_RL.md` 校正清單第 1 項 | `/scan` 是 **`laser_link`**、角度 −85°~+85°、「已是 CCW/左正」→ 不補 |
| 主 repo `nav_rl.py:110` | `lidar_yaw_offset_deg = 0.0` |

兩邊量的是**不同的 launch**（Route B 用 Yahboom bringup，主 repo 用原廠 `TG.launch`），
所以可以各自為真 —— 但只有你實際要跑的那組算數。

**為什麼致命**：AMCL 走 TF 所以永遠正確；但 `nav_rl.py` 是 rosbridge 直接讀 `/scan` 原始角度、
**完全不碰 TF**。若 `/scan` 真的在 `laser` frame，policy 以為的正前方就是車子正後方。而
NAV_RL.md 記的窗口只有 170°（−85~+85），如果那是感測器自己的零度，那**車子正前方根本不在
資料裡** → 48 束前方全部讀成 no-hit 4.0 m → policy 認為永遠淨空、幾何煞停永遠不觸發 →
直直撞上去，全程沒有任何錯誤訊息。

### 先記錄現況（30 秒，不用碰車）

```bash
rostopic echo -n1 /scan/header
rostopic echo -n1 /scan/angle_min
rostopic echo -n1 /scan/angle_max
rosrun tf tf_echo laser_link laser
python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1
```

**這些都不是決定性檢查，只是把現況存證。** 覆蓋率只證明「資料有沒有取樣到那個角度」，
360° 掃描前後都涵蓋，所以它**永遠通過**，也就永遠證明不了 raw index 0 的物理方向。
frame_id 也不行 —— 兩份交接量的是不同 launch，各自為真。AMCL 更不行，它走 TF。

唯一能決定 0° 還是 180° 的是下面的四方向板子測試。**不要在做完之前先猜一個 offset 去跑。**

覆蓋率若警告（`covered 0%` 之類），那是另一回事：代表驅動的角度上下限根本沒取樣到
車子前方，offset 救不了，要改驅動設定。

### 四方向板子 gate（決定性，唯一能發 marker 的路徑）

先停底盤：`rosservice call /route_a/runtime/stop "{}"`。
把寬板依序放在車體**前／後／左／右** 0.35–0.60 m，每個方向各跑一次：

```bash
cd <route_b_handoff>
python2 scripts/diagnostics/scan_policy_orientation_gate.py \
  --placement front --samples 20 --output /tmp/orientation-front.json
# 依序重跑 back / left / right
```

這支只讀 topic，不會驅動任何東西。四份都有了再跑 verifier：

```bash
python2 scripts/diagnostics/verify_scan_policy_orientation.py \
  --evidence /tmp/orientation-front.json --evidence /tmp/orientation-back.json \
  --evidence /tmp/orientation-left.json  --evidence /tmp/orientation-right.json \
  --operator-pass --tf-yaw-deg <0或180> --offset-deg <0或180> \
  --scan-launch <實際 launch> --operator <你的名字> \
  --output-marker ~/.route_b_runtime/scan_orientation_verified
```

**offset 是量出來的，不是你填的。** verifier 會自己算：板在前方時四個 raw sector
裡最近的那一個決定 offset（`zero_deg`→0°、`raw180_deg`→180°），你填的 `--offset-deg`
跟它不合就拒絕寫 marker。另外它會確認 front 的 policy 中央束比後／左／右近至少 0.25 m
—— 四份讀數一樣代表板子根本沒移動，直接 FAIL。

**marker 只能由 verifier 產生。** 手寫一份也能騙過下游（下游只檢查欄位齊不齊、offset
跟執行時設定合不合），但那份 PASS 背後沒有任何量測。

marker 有了之後才帶進去：

```bash
--lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified
```

舊的 `--i-confirm-lidar-orientation` 旗標還在，但已經**不算證明**，`--real` 不收。
任何 LiDAR launch、TF、adapter 或 offset 變更後，**刪掉 marker 重測**。

### 順便驗：手臂會不會被自己的 LiDAR 看成障礙物

`--probe` 時把手臂分別擺在 nav home 和 C3，看**正前方**的 sector 讀數：

| 讀到 | 代表 |
|---|---|
| 前方 sector 是遠距離（>1 m，隨場地變化） | 正常，手臂不擋 |
| 前方 sector 固定在 **5~19 cm** 且不隨場地變 | **那是手臂本身**，幾何煞停會被永久觸發、車子永遠不會前進 |

URDF 的 AABB 粗估顯示三種姿態都可能有連桿穿過 19.2 cm 掃描面（C3 最嚴重，`arm_link4`
估到 LiDAR 前方 18.9 cm，正好在 25 cm 煞停距離內）。AABB 是寬鬆上界不等於真的擋到，
所以**以實測為準**。真的擋到就要調 nav 姿態，或把該角度區間排除。

### 左右順序驗證（marker 過了才做）

四方向 gate 定的是「哪一邊是前面」，這一步定的是**左右有沒有鏡像**。
在車子**正前方**放箱子 → 中間 sector 變短。**左邊** → 高 index（接近 47）變短。
**右邊** → 低 index（接近 0）變短。

左右反了就是 `lidar_angle_dir` 要改成 −1（`--lidar-dir -1`）。這是獨立的一件事：
offset 對了但左右反了，policy 會把每個障礙物閃向錯的方向。

**失敗**：先保留現場 evidence、停止實機，不能邊猜 `--lidar-dir`/offset 邊開巡航；
刪除 marker，修正 adapter/TF 後重跑四方向 gate。

---

## T3 — 巡航（不夾取、不送桶）

```bash
python3 integration/mission_pipeline.py --real --show \
  --no-deliver --detection-streak 999 --max-laps 1 \
  --route <route.yaml> \
  --i-confirm-serial-owner \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --i-confirm-arm-cam-pose
```

`--detection-streak 999` = 永遠不會離開路線。這關只驗「跟著 83 個 waypoint 走完一圈」。

**先在空曠處驗轉向正負**：起步後如果車子朝**遠離**目標的方向轉，立刻 Ctrl+C，加 `--wz-sign -1`。

**通過條件**
- 走完一圈（58.2 m），不撞牆
- 每個 waypoint 都有 `next waypoint: wp_xxx` 的 log，不會卡住或跳號
- 過程中不會反覆進 PAUSED

**要記的**：到點誤差、幾何煞停觸發次數、走完一圈的時間、AMCL covariance 有沒有發散。

**已知風險**
- 地圖是 GLB 渲染不是 SLAM 建圖，現場家具不在圖上 → AMCL 可能被拉偏
- 28 m 長走廊 + 麥輪旋轉系統性左偏 → 丟失定位風險
- 丟失後**沒有自動恢復**，會進 PAUSED 等人重設初始位姿

---

## T4 — 巡航中發現物體並走過去 ★ 從未實測

**這關是這次最需要盯的**，因為 2026-08-04 在這段修掉三個只會在實機上表現成
「它無視垃圾」的邏輯錯誤。離線測試涵蓋了，但沒有實機證據。

```bash
python3 integration/mission_pipeline.py --real --show \
  --no-deliver --max-laps 1 \
  --route <route.yaml> \
  --i-confirm-serial-owner \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --i-confirm-arm-cam-pose
```

在巡航路線旁邊放一個 sugarbox。

**要逐一確認的狀態轉換**（log 會印狀態名）

| 看到什麼 | 代表 |
|---|---|
| `PATROL ... streak=1,2,3` | 偵測連續幀在累積 |
| `INVESTIGATE  trash seen on 3 consecutive frames` | **有離開路線**（修好前這裡會立刻彈回 PATROL） |
| `APPROACH  target confirmed at X.XX m` | 距離進入 2.5 m |
| `ALIGN  within 0.75 m — fine align` | RL 導航交棒給視覺精對位 |
| `STATIONARY_GATE  object inside the trained envelope` | 過了交接門檻 |

**通過條件**
- 從 PATROL 進到 INVESTIGATE 後**沒有**在一兩個 tick 內彈回 PATROL
- 一路走到 ALIGN

**失敗的判讀**
- `INVESTIGATE → RESUME (target lost during investigate)` 幾乎立刻發生
  → tracker 又被清掉了，回頭看 `reset_nav(clear_tracker=...)`
- 一直停在 PATROL、streak 反覆歸零
  → 偵測不穩，或 `--detection-jump-m`（預設 0.35 m）太緊
- `aligned but outside the v21 envelope: x=...`
  → 車停的位置不對。log 會同時印三個座標系，看 base_footprint 那個值：
    **目標是 policy x 0.20~0.28 = base_footprint 前方 18~26 cm**

**要記的**：從看到到停穩花多久、停下來時物體在 base 座標的實際位置、失敗重試了幾次。

---

## T5 — 夾取（v21 已驗過，這裡驗的是整合沒有破壞它）

拿掉 `--no-deliver` 以外的限制，讓它跑到 VERIFY。

**通過條件**
- `[Init] Contract: obs_28_incremental`、模型 sha256 verified
- 夾爪在未完全閉合的角度停住（接觸判定），不是走到 180° 底
- `grasp VERIFIED — object is off the floor`

**要特別看的**：夾取全程底盤必須**完全不動**。FSM 保證輪子和手臂不同時被允許動，
如果看到底盤在手臂動作時抖動，那是別的程序在下命令 → 回 T0。

**失敗重試**：最多 3 次，之後會放棄、把該地點加入黑名單、回去巡航。
確認第 4 次不會再對同一個物體重試。

**夾完進 DELIVER 那一刻要盯著**：夾取全程底盤靜止，AMCL 在這段時間不會更新，
所以離開 GRASP 時 fix 已經是兩分鐘前的。任務層會主動叫一次
`/request_nomotion_update` 並等新的 pose 進來（預設最多 3 秒）。

- 正常：直接進 DELIVER 開始走，看不到任何暫停
- 看到 `[mission][WARN] the arm held the loop for ...s and AMCL did not refresh`
  → T0 的 service 沒起來，或 AMCL 沒在跑。**這是唯一會讓「夾成功但任務停住」的原因**
- 3 秒不夠（機器慢、rosbridge 塞）→ 加大 `--amcl-refresh-timeout`

---

## T6 — 丟垃圾（最簡化）

```bash
python3 integration/mission_pipeline.py --real --show \
  --route <route.yaml> \
  --i-confirm-serial-owner \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --i-confirm-arm-cam-pose
```

夾到之後會導航到 `route.yaml` 的 `trash_bin.approach`（4.35, 13.00），然後直接跑
`run_release_only()`：往前伸 → 開爪 → 回 home。不看垃圾桶在哪。

**通過條件**
- log 印 `bin approach target: (4.35, 13.00, ...)`，而**不是**某個 patrol waypoint
  （這是修掉的 bug：原本會開去當時的巡航點）
- `[Stage 3] reach 1..6` 有跑
- `[Done] Release motion complete`
- 之後回到 `PATROL`，且是**中斷點的下一個** waypoint

**注意**：沒有放開的驗證。`released` 的意思是**動作跑完了**，不是「東西在桶子裡」。
要確認就自己看。

---

## 致命問題（會讓整套跑不起來，或壞掉但看起來正常）

| # | 問題 | 現況 | 怎麼確認 |
|---|---|---|---|
| 1 | **自動回報沒開 → odom 永遠是 0，AMCL 相信它** | 已在 self-check 補上並會拒絕啟動 | T1 |
| 2 | **LiDAR 180°：`/scan` 在 `laser` 還是 `laser_link`**。若在 `laser` 而 offset 是 0，policy 的「前方」是車後方 → 前方全讀成淨空、煞停永不觸發 | 已加自動檢查（覆蓋率不足時 `--real` 拒絕啟動）；**正確值仍須現場確認** | T2 |
| 3 | **序列埠被別的程序佔住** | 只能人工檢查 | T0 |
| 4 | 轉進 INVESTIGATE 時丟掉觸發它的偵測 → 永遠走不到物體 | 已修 + 回歸測試 | T4 |
| 5 | DELIVER 沒有把目標指向垃圾桶 → 開去巡航點 | 已修 + 回歸測試 | T6 |
| 6 | 放棄物體後旗標沒清 → 下一次 LATCH 用**上一個物體的座標** | 已修 + 回歸測試 | T5 重試後再放一個物體 |
| 7 | 沒有黑名單 → 對夾不起來的東西無限重試 | 已修（給up 點 1 m 內不再觸發） | T5 |
| 7b | 成功夾取後 `arm_at_home` 沒設回 → CARRY_HOME 逾時，**每次夾成功都會 PAUSED** | 已修 + e2e 測試 | T5 |
| 7c | 對不準的物體永遠走不到 LATCH → 重試上限不會觸發，APPROACH↔ALIGN 無限循環 | 已修（改計失敗次數）+ e2e 測試 | T4 |
| 7d | PAUSED 沒有任何地方能解除 → 死路 | 已修（按 Enter 恢復，且強制重跑自檢） | T3 |
| 7e | `GraspController.run()` 回傳 `False` 被當成「尚未完成」→ GRASP 永久重跑 | 已修；失敗會進 VERIFY/RETRY，最多 3 次 | 離線 e2e |
| 7f | RETRY 的 `move_home()` 沒同步 nav-home 旗標 → 下一把不會重新升到 C3 | 已修 + 回歸測試 | 離線 e2e |
| 7g | 進 ALIGN 同一 tick 就抬臂，尚未由 odom 證明底盤停穩 | 已修；先 SETTLE，下一 tick 才允許 FINE_ALIGN | 離線 e2e |
| 7h | FINE_ALIGN 是阻塞迴圈，繞過 FSM 逾時、健康檢查與 LiDAR 前向煞停 | 已修 + 回歸測試 | T4 再實車確認煞停 |
| 7i | 攜物時 PAUSED 後重新自檢會張爪／清掉送桶狀態 | 已修；保留 jaw hold 與原狀態後續跑 | 離線 e2e + T6 |
| 7j | 過寬物件把 3D policy 座標塞進 2D AMCL 黑名單 → 下次查詢解包崩潰 | 已修 + 回歸測試 | 離線 e2e |
| 7k | `--no-deliver` 實際繼續巡航且仍抓著物體 | 已修；進 COMPLETE、停車持物後正常結束 | 離線 e2e |
| 7l | `vision_grasp_bridge.py` CLI 半合併，參數缺失且 grasp-home 誤用 nav-home 幾何 | 已修；grasp-home 強制量測 homography | bridge 測試 + Mode B 實測 |
| 7m | 相機例外不清 detection streak；黑名單半徑 0 仍封鎖精確座標 | 已修 + 回歸測試 | 離線 e2e |
| 7n | 相機重連後沿用舊解析度 gate 與半套校正樣本 | 已修；重連後重新驗解析度並清 batch | bridge 測試 + Mode B 實測 |
| 8 | `route.yaml` 最小間距 0.049 m，到點判定會一次過好幾個點 | 已修（重取樣 117→83） | T3 看 waypoint 有沒有跳號 |
| 9 | AMCL 丟失定位沒有自動恢復 | **未解**，會進 PAUSED 等人 | T3 |
| 10 | 地圖是 GLB 渲染，現場障礙不在圖上 | **未解**，靠 48 束 policy 避障 | T3 |

---

## 仍未解、但這次不處理的

- **手臂相機 C3 外參是 URDF 預測值不是量測值**。依 2026-08-04 決定不處理：v21 用這組
  參數實機夾成功了。若夾取落點系統性偏移，這是第一個要回頭看的地方。
- **v21 是 `candidate` 不是 approved**。`manifest.json` 的 `protocol_valid: false`。
  成績是真的但沒有認證，報告/簡報要照這個講法。
- **靜止 feedback 雜訊沒量過**，stationary gate 的門檻（0.02 m/s、0.05 rad/s）是暫定值。
- **放下是否真的進桶沒有感測器驗證**。`released` 只表示伸出／開爪／回 home 的動作完成；
  T6 必須目視，若要自動判斷需新增桶內相機、重量或其他存在感測器。

---

## 紀錄表（印出來填，或複製到 progress.md）

測試日期 ______  測試者 ______  程式版本（`git rev-parse --short HEAD`）______

### 上機前

| | 項目 | 結果 | 數值 / 備註 |
|---|---|---|---|
| ☐ | `preflight.py --offline` | 20 pass / 0 fail？ | |
| ☐ | `fuser -v /dev/myserial` 只有一個 PID | | |
| ☐ | ROS 六個節點都起來 | | |
| ☐ | `rosservice list \| grep request_nomotion_update` 有東西 | | |
| ☐ | RViz 緊初始化（std 0.15 m / 7°）已設 | | |
| ☐ | `preflight.py --onboard` | | |

### T1 odom

| | 項目 | 結果 | 數值 |
|---|---|---|---|
| ☐ | 自檢印出 `auto-report enabled` | | |
| ☐ | 自檢印出 `board is reporting` | | |
| ☐ | `rostopic hz /odom_setmotor` | | ____ Hz（期望 ~20） |
| ☐ | `child_frame_id` = `base_footprint` | | |
| ☐ | 手推 50 cm，odom 走了多少 | | ____ cm（期望 45–55） |

### T2 LiDAR

| | 項目 | 結果 | 數值 |
|---|---|---|---|
| ☐ | `/scan/header` 的 frame_id | | `________` |
| ☐ | `angle_min` / `angle_max` | | ____ / ____ deg |
| ☐ | `tf_echo laser_link laser` 的 yaw | | ____ deg |
| ☐ | 四方向 gate：front JSON 已存 | | 路徑 ________ |
| ☐ | 四方向 gate：back / left / right JSON 已存 | | |
| ☐ | verifier **量出**的 offset | | ____ deg（不是你填的） |
| ☐ | marker 已由 verifier 產生（非手寫） | | 路徑 ________ |
| ☐ | 最終使用的 `--lidar-yaw-offset-deg` | | ____（須等於量出值） |
| ☐ | probe：正前方箱子 → 中間 sector 變短 | | |
| ☐ | probe：左邊 → 高 index 變短 | | |
| ☐ | probe：右邊 → 低 index 變短 | | |
| ☐ | 左右若相反 → `--lidar-dir -1` | | |
| ☐ | 手臂在 nav home：正前方讀數 | | ____ m（固定 5–19 cm = 擋到了） |
| ☐ | 手臂在 C3：正前方讀數 | | ____ m |

### T3 巡航

| | 項目 | 結果 | 數值 |
|---|---|---|---|
| ☐ | 轉向正負正確（`+wz` 左轉） | | `--wz-sign` = ____ |
| ☐ | 走完一圈 | | 用時 ____ 分 |
| ☐ | waypoint 有沒有跳號或卡住 | | |
| ☐ | 幾何煞停觸發次數 | | ____ |
| ☐ | 進 PAUSED 幾次、原因 | | |

### T4 發現物體並前往 ★ 從未實測

| | 項目 | 結果 | 數值 |
|---|---|---|---|
| ☐ | 有進 `INVESTIGATE` | | |
| ☐ | **沒有**在一兩個 tick 內彈回 PATROL | | |
| ☐ | 有進 `APPROACH` | | 觸發距離 ____ m |
| ☐ | 有進 `ALIGN` | | |
| ☐ | 從看到到停穩 | | ____ 秒 |
| ☐ | 停穩時物體在 base 座標 | | policy x=____ y=____ |
| ☐ | 對位失敗重試次數 | | ____ |

### T5 夾取

| | 項目 | 結果 | 數值 |
|---|---|---|---|
| ☐ | 手臂只在 ALIGN 進入時切一次 C3 | | |
| ☐ | 夾取全程底盤完全不動 | | |
| ☐ | 夾爪停在未完全閉合的角度 | | ____ deg |
| ☐ | `grasp VERIFIED` | | |
| ☐ | 失敗 3 次後有放棄並加入黑名單 | | |

### T6 丟垃圾

| | 項目 | 結果 | 數值 |
|---|---|---|---|
| ☐ | log 印的目標是 `(4.35, 13.00)` 不是巡航點 | | |
| ☐ | `[Stage 3] reach` 有跑 | | ____ / 6 步 |
| ☐ | `Release motion complete` | | |
| ☐ | 東西真的在桶裡（目視） | | |
| ☐ | 回到中斷點的**下一個** waypoint | | |

### 失敗紀錄

一次只改一個變因。每次失敗記：指令、terminal log、當下狀態名、現場照片。

| # | 關卡 | 症狀 | 改了什麼 | 結果 |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
