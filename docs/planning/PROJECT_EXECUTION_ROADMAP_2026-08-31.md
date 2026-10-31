# X3Plus 專案執行路線圖 — 2026-08-31

> 目的：把目前已有的程式、測試與實機證據，收斂成一條可以逐步執行、逐步驗收的路線。
> 本文件是「接下來怎麼做」的執行清單；現行技術規格仍以根目錄 `CLAUDE.md` 為準。

---

## 0. 最終目標與執行原則

### 最終目標

完成一套可重複展示、可追查失敗原因的完整任務：

```text
啟動與自檢
  → AMCL 定位
  → 沿路線巡航
  → 發現目標
  → RL 接近與視覺精對位
  → PPO 夾取
  → 導航到垃圾桶
  → 放開物體
  → 回到巡航路線
```

### 現階段的主線決策

1. **v23/E1 是預定正式版本，成功後取代 v21/C3。** 不是讓兩套版本長期並列，也不是只在
   操作台多放一個選項；最終完整任務、模式 A/B/C 與 UI 預設都應指向 v23。
2. **v21 只保留為過渡驗證基準與緊急回退。** 目前 `mission_pipeline.py`、模式 A/C 與操作台
   都接 v21，因此在 v23 接線完成前先用它驗證共用的導航、定位與任務層；切換後不再替 v21
   增加新功能，但暫時保留可明確啟動的 legacy 路徑。
3. **不因為離線測試通過就翻硬體 gate。** manifest 的硬體欄位只接受 Jetson／實車證據。
4. **不在同一次實驗同時更換導航模型、夾取版本、校正值與控制參數。** 一次只改一個變因。
5. **任何 `--real` 都要求人在機器人旁，手可立即切斷電源。** 網頁停止鈕不是硬體急停。

### 成功分成四個層級

| 層級 | 定義 | 目前狀態 |
|---|---|---|
| L1 離線可驗證 | 單元測試、自測、模型契約與安全閘全過 | v21/v23 都已達成 |
| L2 子系統實車可執行 | 定點辨識＋夾取或導航單獨可執行 | v21 達成；v23 單視角達成；導航未完成 |
| L3 完整任務可執行 | 巡航→夾取→投放→續巡至少成功 1 次 | 尚未達成 |
| L4 可重複展示 | 固定流程 ≥2/3 成功，失敗可分類且安全停止 | 尚未達成 |

### 「v23 成功、可以取代 v21」的明確定義

不能只用目前 E1 單視角 3/3 當作取代條件。以下四層必須全部通過：

1. **v23 自身成立**：Jetson 148/37/641/50/89、dry-run、E1 FOV/reach、LEFT/RIGHT motion
   與 yaw mapping 全有實機證據。
2. **整合接線成立**：完整任務、模式 A/B/C、操作台都能從同一份 v23 stack contract 取得
   權重、E1 pose、homography、envelope 與 runtime correction，不再暗中 import v21。
3. **完整任務成立**：v23 完成「巡航→發現→接近→夾取→投放→續巡」至少 3 次中的 2 次，
   且失敗會安全停止。
4. **回退成立**：切成 v23 預設後，仍能用一個明確參數回到原封不動的 v21；回退演練至少
   成功一次，確定不是只在文件上存在。

四層通過後，v23 才取代 v21 成為預設。切換預設不等於自動把 manifest 從 `candidate`
改成 `hardware-approved`；正式評估協議未補齊前，candidate unlock 仍保留。

---

## 1. 工作包與相依順序

```text
WP0 版本與證據收斂
 ├─→ WP1 v23 獨立硬體認證 ─┐
 └─→ WP2 共用導航／任務基線 ┼─→ WP4 v23 接入並取代 v21
               └─→ WP3 導航模型 A/B ┘

WP4 完成後 → WP5 幾何修正與下一代重訓 → WP6 展示與交接
```

不可顛倒的部分：

- WP0 未完成前，不合併 v23。
- WP2 的巡航、定位與至少一輪 v21 完整循環未通過前，不把完整任務預設切到 v23；它的目的
  是驗證共用基礎，不是把 v21 做成長期正式版本。
- v23 LEFT/RIGHT 驗證未完成前，不執行正式三姿態夾取。
- 導航 baseline 未建立前，不把 `doorway_ft_final` 升為預設。

---

## WP0 — 版本、文件與證據收斂

**目標**：先確保目前的實機成果不會遺失，並讓 main、CI、文件與 manifest 說同一件事。

**是否需要機器人**：否。

### WP0.1 保存目前工作樹

目前 `v23-grasp-test` 有 4 個已修改檔與一個未追蹤交付包。先執行：

```powershell
git status --short --branch
git diff --check
git diff -- CLAUDE.md integration/vision_grasp_bridge.py `
  tests/test_vision_grasp_bridge_pose.py progress.md
```

確認內容包含：

- v23 一鍵夾取 3/3 證據。
- homography 後的 base +X 目標補償。
- 補償仍受 policy envelope 約束。
- calibration 模式不套 runtime correction。
- 相對應的 bridge pose 測試。

建議拆成兩個 commit：

1. `v23: preserve the measured forward landing correction`
2. `docs: record the supervised E1 3-of-3 result`

不要把 `grasp/deploy_v23/` 一起加入。它是原始交付包，包含重複 meshes 與舊部署腳本。
先移到 repo 外的備份資料夾或新增明確 ignore；**不要在未備份前刪除**。

### WP0.2 把 v23 納入 CI

在 `.github/workflows/tests.yml` 加入：

```bash
python grasp/v23/test_deploy_controller.py       # 148
python grasp/v23/test_servo_read.py              # 37
python grasp/v23/test_deploy_floor_guard.py      # 641
python grasp/v23/test_one_command_launcher.py    # 50
python grasp/v23/test_three_pose_scan.py         # 89
```

另把目前未在 CI 的純邏輯自測補入：

```bash
python integration/map_goal_provider.py --selftest
python integration/feedback_odom.py --selftest
python integration/ros_io.py --selftest
python integration/nav_rl.py --selftest
python integration/nav_rl_grasp_pipeline.py --selftest
```

### WP0.3 對齊文件

逐項核對：

- `README.md`：說明 v21 是現行整合 baseline；v23 已單視角 3/3，但尚未接完整任務。
- `CLAUDE.md`：保留同樣說法，不把 v23 寫成 hardware-approved。
- `INDEX.md`：加入 v23 launcher、scanner、manifest 與測試入口。
- `progress.md`：保留歷史更正，不讓早期 v17「狀態機成功」被誤讀為實抓成功。
- `grasp/v21/manifest.json`：不因 Windows 測試通過翻 Jetson gate。
- `grasp/v23/manifest.json`：硬體 gate 只依後續 WP1 的證據更新。

### WP0 驗收標準

- [ ] 工作樹只剩刻意保留的本地檔案。
- [ ] `grasp/deploy_v23/` 不會被 `git add -A` 意外加入。
- [ ] v21 與 v23 離線測試都由 CI 執行並全綠。
- [ ] README、CLAUDE、INDEX、progress 對 v21/v23 的定位一致。
- [ ] v23 的 3/3 與 runtime correction 已進 commit，不只存在本機未提交差異中。

**未達標就停止**：不要開始 merge 或切換完整任務版本。

---

## WP1 — v23/E1 獨立硬體認證

**目標**：關閉 v23 自己的 dry-run、E1 工作區、LEFT/RIGHT 與三姿態 gate；仍不接完整任務。

**是否需要機器人**：是。

**建議原始證據位置**：Jetson `/home/jetson/x3plus_evidence/YYYYMMDD_v23/`。
原始 `.log` 不提交 Git；摘要、量測值與結論寫入 `progress.md` 和 manifest。

### WP1.1 Jetson 版本與離線前置檢查

1. 將已提交版本部署到 Jetson。
2. 確認使用 `~/grasp_venv` 的 Python 3.8。
3. 確認沒有其他程序占用相機或 `/dev/myserial`。
4. 執行：

```bash
source ~/grasp_venv/bin/activate
cd ~/Documents/deploy_jetson2/grasp/v23
./jetson_verify.sh
```

必須看到：

- controller `148/148`
- servo read `37/37`
- FloorGuard `641/641`，clamped 291、refused 0
- launcher `50/50`
- three-pose scan `89/89`
- dry-run `wrist_z_offset = 0.0564`
- release gate 對 candidate 維持拒絕或要求明確 unlock

保存完整 stdout，並記下：Git commit、Python、SB3、torch、模型 SHA256。

### WP1.2 E1 FOV 尺量

目標是關閉 `e1_fov_ruler_check`，不是用預測面積代替實測。

1. 手臂固定 E1、S6=30°。
2. 使用 6.5 cm `sugarbox`。
3. 以 E1 gripper-center 地面投影作量尺基準。
4. 分別找出物體完整可辨識的前、後、左、右邊界。
5. 每個邊界重放兩次，確認不是 bbox 抖動造成的偶然值。
6. 記錄兩種範圍：
   - 完全不碰邊界的安全範圍。
   - 明確使用 `--allow-top-clipped` 時的 top-only 範圍。

輸出表格：

| 邊界 | base x/y | 是否碰邊 | 可否 runtime 接受 | 備註 |
|---|---:|---|---|---|
| 前 | | | | |
| 後 | | | | |
| 左 | | | | |
| 右 | | | | |

驗收：文件宣稱的 FOV 必須改成尺量結果，不能保留預測面積當正式範圍。

### WP1.3 E1 motion/reach envelope

1. 先跑固定座標 dry-run，選擇工作區中央與四個邊界內縮點。
2. 每點核對：
   - `_check_target_envelope` 通過。
   - Stage 0 不撞關節限制。
   - FloorGuard 沒有拒絕。
   - close gate 的 pads-ready 與 finger correction 合理。
3. 再依「中央 → 近側 → 遠側 → 左 → 右」順序做 supervised real run。
4. 每個新位置第一跑只放高且穩定的 sugarbox，不先測矮物體。

不得為了讓邊界點通過而放寬 `x=0.205–0.280`、`y=-0.070–+0.065`。

驗收：至少中央與四個代表點都有明確的「可達／不可達」結果；manifest 的
`e1_real_reach_envelope` 要有對應量測紀錄後才可翻成 true。

### WP1.4 LEFT/RIGHT 動作安全驗證

先只驗姿勢，不夾取：

```bash
python3 jetson_one_command_grasp.py --scan-calibrate-pose LEFT
python3 jetson_one_command_grasp.py --scan-calibrate-pose RIGHT
```

對 LEFT、RIGHT 各確認：

- S1 實際到達 70°／110°，殘差在現行 gate 內。
- S2–S5 沒有漂移到不同姿勢。
- 相機線材、手臂、底盤、地面與環境無碰撞。
- Ctrl+C 後能 guarded return 到 E1。
- 中斷或讀取失敗時不會產生 target。

這一步完成後只能翻 `left_scan_motion_validated`／`right_scan_motion_validated`，
不能順便翻 yaw mapping。

### WP1.5 LEFT/RIGHT 映射驗證

每側至少使用 2 個彼此分散的尺量點，建議 3 點：靠近旋轉後工作區中央、左界、右界。

每一點記錄：

| pose | 實測 base x | 實測 base y | predicted x | predicted y | abs dx | abs dy |
|---|---:|---:|---:|---:|---:|---:|
| LEFT | | | | | | |
| RIGHT | | | | | | |

驗收：每一筆 `abs dx ≤ 1 cm` 且 `abs dy ≤ 1 cm`。不能只看平均值，也不能用同一個點重複量測
冒充空間涵蓋。通過後才翻 `left_yaw_mapping_validated`／`right_yaw_mapping_validated`。

### WP1.6 正式三姿態夾取

前面四個 LEFT/RIGHT gate 都完成後：

```bash
python3 three_pose_scan.py --check
python3 jetson_one_command_grasp.py --three-pose-scan --check
python3 jetson_one_command_grasp.py --three-pose-scan --allow-top-clipped
```

測試矩陣：

- 只在 LEFT 看得到。
- 只在 RIGHT 看得到。
- E1 與一個側面同時看得到。
- 兩個姿態都看到且座標一致。
- 故意讓兩個姿態看到不同物體，確認 fail closed。

每種成功情境至少 3 次。保存 scanner 與 controller 的成對 log，確認 scanner 已釋放
相機和 serial，PPO 才開始。

### WP1 驗收標準

- [ ] Jetson `148/37/641/50/89` 與 dry-run 基準通過。
- [ ] E1 FOV 尺量完成。
- [ ] E1 reach envelope 有代表點證據。
- [ ] LEFT/RIGHT motion gate 通過。
- [ ] LEFT/RIGHT yaw mapping 每軸 ≤1 cm。
- [ ] 正式三姿態成功情境各至少 3 次，失敗情境確實不夾。
- [ ] manifest 仍維持 `candidate`，除非另有正式 sim protocol 與完整硬體批准流程。

---

## WP2 — 驗證共用導航／定位／任務基礎（暫用 v21）

**目標**：利用目前已接線的 v21，證明 ROS、LiDAR、odom、AMCL、路線、FSM、投放與續巡這些
共用基礎可以工作。這是替 v23 清除整合變因，不是要把 v21 繼續發展成正式版本。

**是否需要機器人**：是。

### WP2.1 固定部署輸入

在上機前記錄：

- repo commit。
- v21 model／VecNormalize SHA256。
- YOLO `best.pt` SHA256。
- 使用的 `route.yaml` 與 `map_annotations.yaml` 副本或 SHA256。
- 導航模型 pair SHA256。
- 地圖檔、AMCL 參數與 LiDAR launch 版本。

`route.yaml` 預設來自旁邊的 Navigation repo；部署時必須明確指定實際路徑，不要依賴開發機
剛好存在的相對目錄。

### WP2.2 啟動 ROS 與感測器

依序啟動：

```bash
roscore
roslaunch ydlidar_ros_driver TG.launch
roslaunch <robot model>.launch
rosrun map_server map_server site_map.yaml
roslaunch amcl.launch
roslaunch rosbridge_server rosbridge_websocket.launch
```

此時**先不要**在 RViz 設 2D Pose Estimate。

檢查：

```bash
sudo fuser -v /dev/myserial
python3 integration/ros_io.py --probe --ros-host 127.0.0.1
python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1
rosservice list | grep request_nomotion_update
```

完成 Route B 的前／後／左／右 LiDAR 實物方向證據，建立：

```text
~/.route_b_runtime/scan_orientation_verified
```

### WP2.3 啟動主程式，再初始化 AMCL

```bash
source ~/grasp_venv/bin/activate
python3 integration/mission_pipeline.py --real --show \
  --route <route.yaml> \
  --i-confirm-serial-owner \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --i-confirm-arm-cam-pose \
  --detection-streak 999
```

主程式剛啟動顯示 `AMCL not usable` 是預期現象。保持程序運行，再回 RViz 設緊初始化：

- position std 0.15 m
- yaw std 7°

然後：

```bash
rosservice call /request_nomotion_update
rostopic echo -n 1 /amcl_pose
```

必須確認 covariance[0]、covariance[7] 都小於 0.0625，且 `/scan` 與地圖牆線重合。
不要用放寬 covariance gate 的方式強迫啟動。

### WP2.4 巡航-only 測試

保留 `--detection-streak 999`，讓 FSM 不離開 PATROL。

分三階段：

1. **短直線**：2 m 到點，量 overshoot 與觸發煞停到完全靜止的距離。
2. **單一障礙**：固定障礙放在路徑附近，確認原始點雲幾何煞停優先於 PPO。
3. **完整一圈**：走完重取樣後路線，確認 waypoint advance、AMCL 與 odom 不發散。

每階段記錄：

- 最小障礙距離。
- 最大 AMCL covariance。
- 目標到達誤差。
- PAUSED／FAULT 次數與原因。
- LiDAR stale、odom invalid、AMCL refresh 失敗次數。

巡航-only 未通過，不進入任何會夾取的完整任務測試。

### WP2.5 目標接近＋夾取，但不投放

移除 `--detection-streak 999`，加入 `--no-deliver`：

```bash
python3 integration/mission_pipeline.py --real --show \
  --route <route.yaml> \
  --i-confirm-serial-owner \
  --lidar-orientation-evidence ~/.route_b_runtime/scan_orientation_verified \
  --i-confirm-arm-cam-pose \
  --no-deliver
```

第一輪只放一個已知的 sugarbox，位置選在：

- 路線可看到。
- 接近途中沒有門檻或狹窄通道。
- 最後能落入 v21 policy `x=0.20–0.28`、`y=-0.09–+0.08`。

逐狀態核對：

1. PATROL：正常沿 route。
2. INVESTIGATE：連續偵測達門檻後才換目標。
3. APPROACH：切換目標時 ActionDelay／tracker 已重置。
4. ALIGN：物體落入 policy envelope 和 handoff radius。
5. STATIONARY_GATE：輪速回授有效且底盤真的停穩。
6. LATCH：物體座標、高度與 camera pose stamp 一起凍結。
7. GRASP：v21 接觸夾持，無空夾誤判、無齒輪研磨。
8. VERIFY：物體仍在夾爪中。

先做 3 次；至少要成功 1 次才能證明共用接線，≥2/3 則保留為推薦的 v21 比較基準。每次失敗
必須分類為：辨識、接近、ALIGN、stationary、座標、空夾、掉落、servo read、FloorGuard
或定位。

### WP2.6 完整投放與續巡

移除 `--no-deliver`，其餘參數不變。

檢查：

- CARRY_HOME 後物體仍被確認夾持。
- 靜止期間 `/request_nomotion_update` 能恢復 map fix。
- DELIVER 的導航目標確實是 trash bin approach，不是 patrol waypoint。
- PLACE 前底盤通過 stationary gate。
- release height 高於桶緣與安全餘裕。
- RESUME 從下一個 waypoint 繼續，不回頭重走同一點。

完整任務至少先成功 1 次，證明共用任務層能完成循環；建議仍跑滿 3 次，留下 ≥2/3 的 v21
比較基準。沒有碰撞、FloorGuard 穿透或 serial ownership 衝突。失敗後能進 PAUSED／FAULT
並安全停止，也算安全機制通過，但不算任務成功。

### WP2 驗收標準

- [ ] 巡航-only：短直線、障礙、完整一圈均通過。
- [ ] AMCL 初始化順序可重複，不需要降低 covariance gate。
- [ ] `--no-deliver` 至少成功 1 次；建議留下 ≥2/3 比較數據。
- [ ] 完整巡航→夾取→投放→續巡至少成功 1 次；建議留下 ≥2/3 比較數據。
- [ ] 所有失敗都有狀態、原因、時間與對應 log。
- [ ] 共用導航／定位／FSM 已被證明，v23 接入時只需要隔離夾取 stack 變因。

---

## WP3 — 導航模型實車 A/B

**目標**：用同一套實車條件決定是否把 `doorway_ft_final` 取代目前預設的
`ppo_nav_281440_steps`。

**前置條件**：WP2 巡航-only 已通過。

### WP3.1 固定測試場景

建立三種路段：

1. 直線到點。
2. 單一障礙繞行。
3. 門口／狹窄通道。

固定：起點、目標、障礙位置、速度參數、LiDAR adapter、AMCL 初始化方式與停止距離。
每次只換 model／VecNormalize pair。

### WP3.2 測試兩個模型

模型 A：

```text
ppo_nav_281440_steps.zip
ppo_nav_vecnormalize_281440_steps.pkl
```

模型 B：

```text
doorway_ft_final.zip
doorway_ft_final_vecnormalize.pkl
```

建議每個場景每模型至少 5 次。記錄：

- 成功率。
- 碰撞／幾何煞停次數。
- 到點時間。
- 最小障礙距離。
- stall assist 次數。
- AMCL 發散或人工介入次數。

### WP3.3 決策規則

只有模型 B 同時滿足以下條件才升預設：

- 總成功率不低於 A。
- 碰撞／危險煞停不高於 A。
- 門口場景明顯優於 A。
- 沒有新增左右偏轉、振盪或停不下來的行為。

升級時必須一起修改 model 與 VecNormalize，並更新 SHA256、README、NAV_RL 與測試。

---

## WP4 — 將 v23 接入完整任務並取代 v21 預設

**目標**：先用明確 stack 隔離兩套版本，完成 v23 完整任務驗收後，將所有正式入口的預設
切到 v23；v21 降為只供回退的 legacy stack。

**前置條件**：WP1 與 WP2 都完成。

### WP4.1 設計 stack contract

新增明確選項，例如：

```text
--grasp-stack v21
--grasp-stack v23
```

每個 stack 必須原子綁定：

- module path。
- manifest path。
- model + VecNormalize SHA256。
- contract 名稱。
- grasp-home pose 名稱與 API 角度。
- homography path。
- policy x/y envelope。
- navigation handoff aim／stop adjustment。
- controller runtime correction。

不能只用共同的 `obs_28_incremental` 區分 v21/v23，因兩者 shape 與 action mode 相同。
開發與驗收期間先要求明確帶 stack；切換完成後，省略參數才代表 v23。

### WP4.2 修改載入路徑

需要檢查的入口：

- `integration/mission_pipeline.py`
- `integration/vision_grasp_pipeline.py`
- `integration/nav_rl_grasp_pipeline.py`
- `ui/server.py`

要求：

- 切換前：明確 `--grasp-stack v21` 時，v21 行為完全不變，作為比較與 rollback baseline。
- 切換後：未帶參數時預設 v23；v21 只能透過明確 `--grasp-stack v21` 啟動。
- v23 只能載入 v23 manifest 所列權重。
- C3 權重＋E1 pose、E1 權重＋C3 pose、錯誤 homography 都要在開 hardware 前拒絕。
- v23 導航交接距離的 +1.3 cm 差異要由 stack config 明確表達，不散落成 magic number。
- 模式 A、模式 B、模式 C、`mission_pipeline.py` 與 UI 必須共用同一個 resolver；不能有某一條
  路徑仍偷偷載入 v21。

### WP4.3 自動化測試

至少增加：

- v21 stack 解析與 SHA256。
- v23 stack 解析與 SHA256。
- 模型／VecNormalize 交叉配對拒絕。
- home pose 交叉配對拒絕。
- homography 交叉配對拒絕。
- UI 預覽命令顯示實際 stack。
- 遙測包含 `grasp_stack`、model hash 短碼、home pose。
- 明確選擇 v21 時的回歸行為不變。
- 切換完成後，所有未指定 stack 的入口都斷言載入 v23。

### WP4.4 實車回歸順序

1. 同一 commit 先跑 v21 定點夾取。
2. 再跑 v23 E1 單視角。
3. 再跑 v23 三姿態。
4. 最後才跑完整任務 v23。

v23 完整任務必須達到 ≥2/3，才能在 UI 或主程式中改成預設。

### WP4.5 正式切換與 v21 降級

v23 完整任務驗收通過後，一次完成以下切換：

1. `mission_pipeline.py` 預設 stack 改為 v23。
2. 模式 A、B、C 的預設 stack 改為 v23。
3. UI 啟動命令與預覽顯示 v23；使用者不需要另外勾選才能得到正式版本。
4. README、CLAUDE、INDEX、部署指令全部將 v23 標為現行整合版本。
5. v21 文件改標「legacy rollback」，不再稱為現行主流程。
6. 保留 `grasp/v21/`、權重、manifest 與明確 `--grasp-stack v21`；不要在同一版直接刪除。
7. 執行一次 rollback drill：v23 停止後，以明確參數啟動 v21 定點夾取，確認回退真的可用。

切換後再跑一次全部離線 CI，以及 v23 一次定點夾取、一次三姿態夾取、一次完整任務。三者
都通過才建立 release tag。v21 至少保留到 v23 經過兩個獨立實車 session 且沒有需要回退的
阻斷性問題。

---

## WP5 — 幾何修正與下一代重訓

**目標**：解決目前靠 runtime correction 補償的系統性誤差，並改善矮物體與弱角落。

### WP5.1 重量實體幾何

使用治具或卡尺，不只用手持直尺：

- C3、E1 張爪與閉爪時的 gripper center。
- 真實 finger top、chunky block bottom、blade tip。
- S6 角度對 finger bottom／jaw opening 的曲線。
- S2–S5 到位殘差與重現性。
- TCP 前向落點偏差是否隨 x/y、姿勢或負載改變。

輸出一份機構量測表，判斷 16.7 mm 是固定 finger 長度差、link offset 還是量測參考點問題。

### WP5.2 修正 URDF／訓練環境

修正後重新驗證：

- gripper-center TCP。
- pad bottom 與 close drop。
- floor collision geometry。
- URDF_TO_TRAINING_FRAME。
- object height 與 centroid z 慣例。

不要直接拿修正後 URDF 餵現有 v21/v23 權重；幾何改動後建立新版本，例如 v24。

### WP5.3 重訓資料分布

至少納入：

- 2、3、3.5、6.5 cm 等不同高度。
- 實際部署 x/y envelope。
- v23 弱角落 `(0.28, -0.07)` hotspot。
- home pose jitter 按正式 protocol 真正執行。
- 實體可見範圍與 policy spawn band 的交集。

### WP5.4 可重現模型包

模型包必須附：

- clean git head；若非 clean，附所有訓練來源檔 SHA256。
- seed、checkpoint、Python、SB3、torch。
- model／VecNormalize SHA256。
- 逐格成功率、碰撞、FloorGuard clearance。
- 有效 formal protocol 結果。
- 明確 home pose、TCP、action contract 與 object-height 定義。

完成前維持 `candidate`，不要直接標為 hardware-approved。

---

## WP6 — 展示、操作與交接

**目標**：把已通過的流程變成別人也能照做的展示版本。

### WP6.1 建立唯一上機 runbook

整理成一份不需要翻多份文件的順序：

1. 開機設備檢查。
2. 停止衝突程序。
3. ROS、LiDAR、map、AMCL、rosbridge 啟動。
4. 啟動 mission pipeline。
5. 再設 AMCL 初始位姿。
6. 確認 self-check。
7. 操作台啟動與雙重 real unlock。
8. 任務開始、正常停止與電源急停。

### WP6.2 展示驗收

展示前固定：

- 使用的 grasp stack。
- 導航模型。
- YOLO 模型。
- route、map、物體與垃圾桶位置。
- 成功／失敗定義。

連續執行 3 次，至少 2 次完成整個循環。第三方只依 runbook 操作，不需要修改程式或手動輸入
隱藏參數，才算交接完成。

---

## 2. 建議實際工作日程

| Session | 內容 | 需要硬體 | 完成標誌 |
|---|---|---|---|
| A | WP0 commit、CI、文件對齊 | 否 | CI 全綠、工作樹乾淨 |
| B | v23 Jetson verify、E1 FOV/reach | 是 | dry-run 與尺量完成 |
| C | LEFT/RIGHT motion + mapping | 是 | 四個 side gate 有證據 |
| D | v23 三姿態實抓 | 是 | 成功/失敗矩陣完成 |
| E | ROS、LiDAR、odom、AMCL、巡航-only | 是 | 路線一圈與煞停通過 |
| F | 暫用 v21 驗證共用接近、投放與續巡 | 是 | 至少一輪完整循環 |
| G | 實作 v23 stack resolver 與所有入口接線 | 部分 | 明確選擇 v21/v23 均正確 |
| H | v23 完整任務實車驗收 | 是 | 完整任務 ≥2/3 |
| I | v23 切為預設＋v21 rollback drill | 是 | 所有入口預設 v23、回退可用 |
| J | 導航 A/B、幾何修正與 v24 規格 | 部分 | 模型決策與可重現訓練包 |

Session 可拆開，但每次結束都要留下：commit／版本、執行指令、結果、失敗分類與下一步。

---

## 3. 每次實車測試的紀錄模板

```text
日期／時間：
操作者：
repo commit：
Jetson 部署路徑：
grasp stack：v21 / v23
grasp model sha256：
vecnorm sha256：
nav model sha256：
YOLO sha256：
route/map：
執行指令：
物體尺寸與擺放座標：
AMCL 初始 covariance：
結果：成功 / 安全拒絕 / 失敗 / 人工急停
最後 FSM state：
失敗分類：
FloorGuard summary：
servo-read summary：
最小障礙距離：
是否保留完整 log：
結論與下一步：
```

只有「實體觀察到物體被夾起／投進桶中」才算任務成功；狀態機走到 success 字樣不等於實體成功。

---

## 4. 最近三個立即行動

1. **先完成 WP0**：保存現在尚未提交的 v23 3/3 與前伸補償，並把 v23 測試納入 CI。
2. **下一次碰機器人先做 WP1.1、WP1.2、WP1.4、WP1.5**：完成 Jetson verify、E1 FOV、
   LEFT/RIGHT 動作與各至少 2 點映射，不先新增功能。
3. **之後用 WP2 驗證共用基礎，再直接進 WP4**：暫用 v21 跑通巡航-only 與至少一輪完整
   循環，確認導航／定位／FSM 沒問題；接著把 v23 接入，完成 ≥2/3 後正式取代 v21。

這三項完成前，不建議開始 v24 重訓。v23 在 WP4 完整任務驗收通過後就應切成操作台與主程式
預設；v21 只留下明確的 legacy rollback，不再雙版本並列發展。
