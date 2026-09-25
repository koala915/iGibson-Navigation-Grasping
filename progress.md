# X3Plus 專題進度記錄

## 2026-09-22 — WP2 底盤／LiDAR 實機點動與線性 odom 校正

### 導航硬體資料鏈與互斥服務

- Jetson 的 `x3plus-navigation` 為唯一 `/dev/myserial` owner；測試期間
  `grasp-service` / `grasp-vision` 維持停止，未發現序列埠競爭。
- `ai_motor_server_P0.py` 經 TCP 7000 接收速度命令，具 0.5 秒 watchdog；每次點動後的
  log 都確認 `action=stop` 與四輪 `m1=m2=m3=m4=0`，`/odom_setmotor` 的線速度與角速度
  亦回到 0。
- TG30 `/scan` 約 10 Hz。實物方向測試確認 raw 180° 是車頭，raw +90°／-90° 分別對應
  車體左右；正式 mainline 仍須把 yaw offset 與 angle direction 一起寫入、通過方向 gate，
  不能只靠本次人工點動推定完整自主導航已驗收。
- 直線點動以車身中心左右各 20--25 cm 的投影走廊判定正前方淨空；現場操作員確認兩側
  近距離回波是預期障礙，不在直線 swept path。沒有永久關閉或放寬緊急煞停。

### 線性 odom 尺量校正

低輸出 18 會受靜摩擦影響，因此正式校正使用既有慣用的四輪輸出 30
（TCP `vx=0.15 m/s, wz=0`），相同約 0.8 秒短脈衝並以尺量同一輪位置：

| 線性係數 | odom 位移 | 尺量實際位移 | 結果 |
|----------|-----------|--------------|------|
| `0.65` | 14.6 cm | 約 22 cm | 少算約 34% |
| `0.98` | 20.9 cm | 約 21--22 cm | 誤差約 0.1--1.1 cm |

- `0.98 = 0.65 × 22 / 14.6`（四捨五入）已在 Jetson 的部署腳本
  `/home/jetson/ROS/X3/yahboomcar_ws/src/yahboomcar_bringup/scripts/ai_motor_server_P0.py`
  實測；原 `0.65` 檔案保留為 `*.bak_odom065_20260922`。
- 本次把 repo 主線 `integration/feedback_odom.py` 的預設線性係數同步為 `0.98`。
  角速度係數 `0.501` **未重測、未更動**。
- 目前數據是短距離人工尺量，足以淘汰 `0.65`，但 `0.98` 仍是暫定值；WP2 下一步要做
  較長直線重複量測、左右 90° 轉向校正，再驗證 TF／AMCL，而不是直接宣稱導航全流程通過。

### 安全收尾

- 最後一輪停止後確認 odom 速度為 0、四輪命令為 0；停止導航服務、釋放
  `/dev/myserial`、執行 `sync` 後安全關機。SSH 與 TCP 22 隨即中斷，ping 無回應。
- Jetson 的 `ai_motor_server_P0.py` 本體目前不在此 repo；本提交只同步可版本控制的係數、
  自測期望值與實機證據，沒有假稱部署腳本已納管。

## 2026-09-20 — 常駐夾取服務（42.3 s → 8.5 s）、LEFT 首次真機夾取

### 新系統碟（128 GB）開機檢查：全項通過

- 裝置對應正確：`/dev/myserial`→`ttyUSB0`（ch341 Rosmaster）、`/dev/ydlidar`→`ttyUSB1`
  （CP2102 TG30）、`arm_cam`→`video0`、`rear_cam`→`video1`（SN0001）；兩顆相機都取得
  640×480 畫面，`startup_device_check.py` PASS。
- 根目錄在 `/dev/sda1`（114 GB，用 42 GB），**swap 6 GB 已寫進 `/etc/fstab`**。
- venv 是 Python 3.8.0：torch 2.4.1（**CPU-only**，JetPack 4.6 沒有對應的 CUDA wheel）、
  SB3 2.3.2、gymnasium 0.29.1、cv2 4.13.0、ultralytics 8.4.83、pyserial 3.5。
- v23 權重 sha256 與 manifest 一致；`detection/models/best.pt` = `ca42c3f4…`。
- 匯流排：45 輪讀取 **0 錯位、0 逾時**。過程中 S3 一度連續回報 raw 3664（有效區間
  900–3100），那是**關節被扳到模型範圍外**，不是伺服機故障 —— 讀值穩定到 ±1 才是判斷關鍵，
  壞掉的伺服機會回亂數。扳回範圍後六顆全正常。
- S1 ±5° 實機寫入驗證通過，其他五顆數字一格未動。

### 一次完整夾取的耗時拆解（`jetson_one_command_grasp.py`，42.3 s）

| 階段 | 秒 |
|------|----|
| 程序啟動與模型載入（PyBullet 載 URDF 就佔 11.5 s / 670 MB） | 23.3 |
| 視覺（YOLO 載入、開相機 1.6 s、冷啟動推論 1.5 s） | 10.5 |
| policy 對位 4.3 + 抬升 1.9 + 回 home 1.9 | 8.1 |

**夾爪不是瓶頸。** 正式流程的閉合發生在 stage 0 的 S6 停滯捷徑裡
（`--s6-stall-grasp-steps 2` 會一併把 `jaw_hold_bias_deg` 設成 **1**），
`attempt_close()` 那條 stage-1 路徑根本不會被呼叫。

曾實驗把 stage-1 close 的 `run_time_ms/settle_s` 由 300/0.30 改成 100/0.15
（空夾 5.6 s → 3.06 s，且全程無誤判接觸），但**已還原**：那條路徑不在正式流程上，
而且在真實物體上會讓接觸判定多花 6 步才成立，指令多壓進去 7°。
附帶量到伺服機的真實速度上限：8° 一步最快約 90 ms，300 ms 是我們自己叫它慢的。

### 新增：常駐服務（本次主要成果）

- `grasp/v23/grasp_service.py` — 呼叫 launcher 自己的 `parse_args_from()` 與
  `build_ctrl_cmd()` 取得**完全相同的 argv**，再把 `GraspController.run` 換成 serve
  loop 後呼叫 controller 的 `main()`。因此 sha256、契約、homography、candidate 解鎖
  等每一道閘照跑，**已驗證檔案零修改**；`run()` 本身的 docstring 就寫明可重複呼叫。
- `grasp/v23/graspctl.py` — unix socket 用戶端，系統 python 3.6.9 可直接跑
  （`grasp` / `status` / `quit`）。
- `grasp/v23/graspscan.sh` — 三姿態後備：停服務 → 跑 launcher 掃描流程 →
  `trap EXIT` 把服務拉回來。掃描器設計上獨占相機與序列埠且在 PPO 啟動前就退出，
  無法與常駐服務並存，所以後備會付完整啟動成本。
- systemd：`grasp-service.service`（`Wants/After=dev-myserial.device`，避免搶在 udev
  之前啟動）、`grasp-vision.service`（`BindsTo` 夾取服務），兩者 `enabled`。
  視覺端維持原檔，只是拿掉 `--once`、加上 `--imgsz 320 --rate 1.0`。
- 實測 **單次夾取 8.52 s（confirmed）**。RSS：夾取 985 MB、視覺 301 MB；ROS 全套
  569 MB；三者同時在仍餘約 1 GB，不碰 swap。閒置時 CPU 幾乎是 0（負載 1.09 / 4 核）。
- **E1 優先**：沒有新鮮偵測時 3.2 s 內拒絕且**手臂完全不動**（原本會走到 home 然後
  空等 `--latch-wait` 120 s）。

### imgsz 320 可以直接用

同一場景比較 640／480／320：座標差 **0.4 mm**（640 自身抖動就有 0.2 mm，homography
RMSE 是 3.9 mm），320 的重複性反而更好；推論 1.37 s → 0.62 s（暖機後 0.65 → 0.25 s）。

### LEFT 姿態首次真機夾取成功（單視角，**非驗證**）

- `graspscan.sh --accept-single-rotated-view`：LEFT 取得 `(+0.2569, +0.0388)`，
  5 樣本散布 0.1 mm；E1／RIGHT 未見；回 E1 編碼器確認後釋出目標。PPO 41 步收斂到
  `target_dist = 0.019 m`，S6 判定 slipping 停在 151°（hold bias 1°），抬升 6 段、
  回 home 後仍夾著。
- 逼近過程 S1 由 90° 降到 **71–74°**，方向與物體在左側一致 →
  **yaw 正負號在真機上沒有反向**。方向錯的話 S1 會往 110° 去夾空氣。
- **但這不是 gate 要的驗證**：沒有尺量點、沒有 ≤1 cm 誤差數據，
  `hardware_validated` / `yaw_mapping_validated` 維持 `false`。
- 擦邊觀察：`Gate 25` 時 `pad_after_close = 62.9 mm` 對 `object_top = 65.0 mm`，
  **只差 2.1 mm**（E1 正面那次是 41.6 mm）。LEFT 的落點明顯偏高、夾得較淺，
  這是之後補驗證時要盯的數字。

### 設計意圖釐清

三視角的目的是**擴大可見範圍**，不是讓視角互相驗算。**只要有一個視角看到就允許夾取**
是刻意的設計，因為單視角覆蓋太小。`--accept-single-rotated-view` 因此是常態用法，
不是繞過檢查的捷徑。

### E1 homography 的實際可用窗口只有約 3 cm 縱深

7 個校正點的像素凸包換算回 base 約 `x = 0.249–0.279 m`。今天三次掃描失敗全部卡在這裡：
左緣裁切 ×2、凸包外 ×1。物體必須放進這個窄帶，這是目前整套視覺夾取真正的範圍限制，
與三姿態無關；要擴大就得在更廣的位置補校正點重新擬合。

### 常駐服務：補上 `home` / `release`，連續 6 輪確認不漏記憶體

- 背靠背 6 輪：成功 3 次，**8.05 / 8.34 / 8.53 s**，RSS 三次都是 **986.1 MB**
  （服務啟動 980.7 MB，第一輪後 +5.4 MB 就固定，之後完全走平）。另外 3 輪因為
  `home` 放下的盒子落點跑掉而被拒，每次 3.2 s 且**手臂不動**。
- `home`：原地張爪並回 E1（在 home 時 0.4 s，抬起後 6.1 s）。沒有它，要把物體放回桌上
  得停服務、跑 `move_arm.py`、再啟動 —— 一分鐘的 PyBullet 重載只為了張開夾爪。
- `release`：呼叫 `run_release_only()`，就是任務到垃圾桶後的那段。

### 修正：`_grasp_looks_real()` 把「完全張開的夾爪」判成握著東西

- 實測發現：**空爪**呼叫 release，印出
  `grasp check: CONFIRMED — jaw stalled at 30.0deg (100.0% short of closed)`，
  然後跑完整套伸手、開爪、回 home。
- 原因：這個檢查**只有下界**（離全閉停點多遠），沒有上界。夾爪完全張開時 frac = 1.0，
  分數最高，於是「離全閉越遠 = 越像握著東西」。
- 影響不只在這條路：`mission_pipeline` 把 `"released"` 當成投遞完成，所以**在路上掉了的
  物體會被記成已投遞** —— 正是 `run_release_only()` 不先回 home、宣稱要抓住的那個情況。
- 修法：補上 `closed_frac >= timeout_close_min_grip_frac`（0.5），用的是 stage-0 捷徑
  既有的門檻，兩條路現在對「夠閉合」的定義一致。**v21 與 v23 都修**（該函式逐位元組
  相同，而任務流程跑的是 v21）；Jetson 上只打這個補丁，沒有整檔覆蓋。
- 驗證：同樣的空爪 release **0.04 s 被拒、手臂沒動**。控制器測試 135 → **138**、
  160 → **163**，`jetson_verify.sh` 兩支與 CLAUDE.md 的硬編碼期望值同步更新；
  其餘套件不變（37 / 641 / 29、launcher 62、three-pose 89、safety guards 68）。
- **尚未完成：握著物體的正向 release 測試。** 盒子幾次落在 E1 校正窗口外（下緣裁切），
  今天沒測到。下次接續時把盒子放在機器人正前方 25–28 cm，先 `grasp` 再 `release`。

### 注意：樹上已有一批同目標的未提交工作（2026-09-09）

今天的加速是在不知道這批工作的情況下做的，兩者互補而非重複：

- `grasp/x3plus/make_deploy_urdf.py` + `yahboomcar_deploy.urdf`（未追蹤）：只保留六個
  夾爪連桿的碰撞網格，**URDF 載入 3.66 s → 0.14 s**。今天量到的 23.3 s 啟動裡有
  11.5 s 正是它。
- v23 controller 的 scripted-move 時序欄位（`close_run_time_ms` 等）、`--jaw-step-deg`
  （夾爪步距 8° → 12°）；launcher 的 `--wait-for-go`：bridge 先啟動載 YOLO，與
  controller 載 torch **並行**。
- 關係：那批縮短「單次啟動要多久」，常駐服務讓啟動「只付一次」。兩個都在的話，連
  `graspscan.sh` 這條必須付完整啟動成本的後備路徑也會跟著變快。
- `grasp/deploy_v23/` 是 **205 MB** 的交付包，不該進 git。

---

## 2026-08-30 — v23 三姿態改為 LEFT/E1/RIGHT S1 剛體旋轉

- 新增 `grasp/v23/three_pose_scan.py` + `three_pose_scan.json`；原先 E1／F3／F6 的
  近中遠設計改為 LEFT／E1／RIGHT（S1=70°／90°／110°），因主要缺口在橫向。
  掃描器不載 PPO、S6 永遠保持 30° 張開，所有
  移動直接借用 controller 的 `move_guarded_and_verified`。
- 三姿態 S2–S5 完全相同，只用 E1 homography；映射後繞 training-frame S1 軸心
  `(0.118146,-0.003359)` 旋轉，yaw sign=-1。URDF/FK 在 S1=60–120° 最大平面殘差
  `1.3e-8 m`，但這不取代真機驗證。5 筆有效偵測取中位數且散布 ≤8 mm，多姿態同時看見時
  base XY 須在 1 cm 內一致。三個都看不到、不同姿態疑似選到不同物體、任一 move 未確認，
  都不會產生夾取 target。
- 掃描器只有在 guarded move 回 E1 且編碼器確認後才印 `[scan][result]`，隨即退出並釋放
  相機與 `/dev/myserial`；launcher 之後才啟動原 v23 PPO，以固定 `--obj-x/y/z` 夾取，
  不存在 scanner/controller 同時搶硬體或拿過去姿態的 pose stamp 冒充 E1。
- 純邏輯 `test_three_pose_scan.py` **40/40** 通過，涵蓋 yaw 方向／半徑不變、軸心／方向
  防竄改、角度上限、禁止
  S2–S5 共用 homography、中斷、掃描錯誤與 E1
  回程未確認時不得釋出目標，且 Ctrl+C 只送掃描器一次 SIGINT、保留 45 秒
  guarded E1 回程窗口；既有 launcher **45/45** 不變。
- **尚不能正式三姿態實抓**：LEFT/RIGHT 尚未實際走過，也還沒各用至少 2 個分散尺量點
  確認 `predicted_x/y` 每軸誤差 ≤1 cm；兩個 `hardware_validated` 與
  `yaw_mapping_validated` 目前故意是 false。下一步依序跑
  `--scan-calibrate-pose LEFT`、`RIGHT`，不需要也不應重做兩份 homography。

---

## 2026-08-29 — v23/E1 grasp-home homography 實測完成

- 使用 `sugarbox`、每點 20 幀中位數，在 v23 的 E1 grasp home 收到 10 筆有效
  undistorted `(u,v)`；另有 3 筆因 bbox 碰到上／下／左影像邊界而依安全規則捨棄。
- base 座標以 E1 張爪 `gripper_center` 地面投影 `(0.2287, -0.0035)` 為量尺基準：
  `x = 0.2287 + forward`、`y = -0.0035 + left`。
- 第 1 筆回報為「左 4 cm」，但其 `u=400.311` 與所有其他左右樣本相反；照原標註擬合
  RMSE 會升到 **5.206 cm**，視為疑似左右抄反並排除，沒有把推測寫進正式外參。
- 正式檔用 7 點擬合：RMSE **0.388 cm**、最大擬合誤差 **0.680 cm**。另保留第 4、10
  兩個凸包內點驗證，最大單軸誤差 **0.448 cm**、最大歐氏誤差 **0.603 cm**，均通過
  1 cm gate。
- 原始整理與排除理由在 `docs/calibration/e1_grasp_home_points_20260829.json`；runtime
  校正檔為 `integration/grasp_home_homography_e1.json`。`e1_homography_measured` 已改為
  true；這只解除視覺映射 gate，不代表 dry-run、可達範圍或實抓已通過。
- 同日實抓 log 顯示前 4 cm／正中的 sugarbox 只碰影像**上緣**。新增 opt-in
  `jetson_one_command_grasp.py --allow-top-clipped`：只在 E1 實測 homography 下接受 top-only
  clipping，因底邊中心與左右邊仍可量；左／右／下緣、**目標中心**在凸包外與 calibration
  mode 一律維持 fail closed。後續前 4 cm／右 3 cm 的實抓 log 又證實 bbox 右側輪廓會比
  已校正的中心凸包多伸出約 3 mm base-space；左右端點已改成只做受 6 cm 夾爪開口約束的
  寬度估算，絕不作為目標位置。沒有採用「先移手臂置中」，因為相機隨 `arm_link4` 移動，離開 E1 後同一份
  homography 立即失效。

---

## 2026-08-03 — v21 Jetson 單機辨識＋夾取實機成功

### 啟動姿態小幅回差恢復

- 底盤移動後曾出現 C3 home 殘差 `2.1°`，原本因超過一般動作的 `2.0°` 到位門檻而在
  開相機前安全中止；這與 S6 stall steps 無關，也不是只改 bridge 就能解決。
- Jetson 單指令現在讓「啟動 C3 到位」與「detection cam_pose 核對」共同使用 `3.0°`
  上限。`2.1°` 的小幅機械回差可以繼續；啟動容許值在 controller 內硬性封頂 `3.0°`，
  即使傳入更大的 detection tolerance，也不能讓明顯偏離校正姿態的手臂通過。
- 只有 startup 使用此恢復窗口；return home 與其他受 guard 保護的動作仍使用原本
  `2.0°` 到位判定。讀取失敗、floor guard 異常或殘差超過 `3.0°` 仍會 fail closed。

### 本次結論

- 分支 `worktree-grasp-v21-candidate` 已有實機證據，可判定為
  **hardware-executable candidate（可在操作員全程監督下執行）**。
- 已用 `python3 jetson_one_command_grasp.py` 在 Jetson 上完成單一指令的完整流程：
  本機相機辨識 → 傳入目標座標 → RL 對位與閉爪 → 判定接觸 → 抬升 → 回 home。
- 本次夾取成功不代表已達 `hardware-approved`。在完成下方「合併前仍需處理」前，
  `manifest.json` 應繼續維持 `candidate`，實跑仍需 `--unlock-candidate-real` 並手放電源。

### 成功實測紀錄

- 啟動前檢查通過：controller、vision bridge、PPO、VecNormalize、YOLO 權重皆存在；
  `cv2`、`ultralytics`、`stable_baselines3`、`pybullet` 可載入；手臂相機 stable by-id
  與 `/dev/myserial` 均存在。
- launcher 等待機械臂確認 C3 home 後才開啟一次性 YOLO；無有效 detection 時不允許手臂
  朝預設座標移動，辨識程序結束後會釋放 Jetson Nano 的記憶體。
- 本次有效 detection：`x=0.2974 m`、`y=0.0033 m`、`z=0.0325 m`、
  `height=0.065 m`、物體寬約 `0.0276 m`；YOLO 平均推論約 `1096 ms`。
- S6 實際在約 `153°` 夾住物體；連續 2 個 policy step 不變後直接確認接觸，
  以 `+1°` 小幅保持力固定在約 `154°`，避免先前 `+8°` 加壓造成齒輪咖咖聲。
- Stage 2 六段抬升全部到位，成功回 home；回 home 後仍確認夾持，該次
  `servo reads: no failures`，floor guard 統計為 `pass: 46`。
- 若 S6 到達空夾爪閉合端 `179–180°`，現在不再等待 stall steps，會直接判定沒有物體並
  安全回 home。

### 已固定的實機參數

- C3 home 經驗校正：`cam_x=0.2970`、`cam_y=0.0034`、`sign_y=-1`。
- sugarbox 高度：`0.065 m`，傳給控制器的質心高度為 `z=0.0325 m`。
- Stage 0 XY 閉爪門檻：`10 mm`。
- S6 接觸判定：連續 `2` 個 step 讀值變化不超過 `0.5°`，且必須是閉爪命令、
  已超過半閉合並明顯未到空夾爪端點。
- 夾持保持偏壓：`+1°`。
- 視覺 X 軸已加入實機前伸補償；Y 軸方向依「面向手臂前伸方向」定義，使用
  `sign_y=-1`。右側位置已成功夾取；左側受相機視野限制，安全可見約到 `2.4 cm`，
  約 `2.7 cm` 時物體會碰到影像左緣並被 bridge 拒絕。

### 程式與測試狀態

- 新增 `grasp/v21/jetson_one_command_grasp.py`，不再需要 Windows 與 Jetson 各開一條指令。
- `x3plus_real_grasp.py` 新增 `--entry-xy-mm`、`--s6-stall-grasp-steps`、
  Stage 0 接觸捷徑、`+1°` 保持力與空夾爪端點立即撤退。
- `test_deploy_controller.py` 會在每個 case 後關閉 PyBullet client，避免 Jetson Nano
  在完整 controller suite 中 OOM。
- 已有測試證據：controller `119/119`、servo read `37/37`、floor guard `641/641`；
  dry-run 的 `wrist_z_offset=0.0564 m` 與預期一致；`arm_cam_geometry.py --selftest`
  亦為 PASS。

### 合併前仍需處理

1. 將本次尚未提交的 controller、測試、Jetson launcher 與本進度紀錄整理成 commit，
   更新 PR #14，並確認遠端 CI / PR checks 全綠；本機 log、臨時校正 JSON 與 `.claude/`
   不應混入 commit。
2. `README.md` 目前仍寫「沒有任何實機證據」，必須改成這次的實際狀態與單指令操作方式。
3. C3 的 `theta/H` 仍是 URDF/FK 預測值，`cam_x/cam_y` 是實機經驗補償，不是完整
   Phase 1/2 外參量測；因此保留 predicted-extrinsics 警告與 candidate 安全閘。
4. 成功 detection 的 `x=0.2974 m` 超過文件標示的有效範圍上限 `0.28 m`，但仍在模型
   已評估的 `0.20–0.33 m` 內；文件與 manifest 的範圍應統一後再合併。
5. Jetson 上 YOLO 約 `1.10 s/frame`，接近 controller 預設 `1.0 s` stale timeout；雖然本次
   latch 成功，仍應調整單機 launcher 的 timeout 或加入對應測試，避免負載稍高時誤拒絕。
6. 現有 `119/37/641` 都通過，但 controller suite 尚未直接覆蓋新增的 Stage 0 S6 stall
   成功路徑、`179–180°` 立即撤退路徑，launcher 也沒有自動化 orchestration 測試；
   合併前應補上這些回歸測試。
7. 目前已有一次完整單指令成功，以及先前中央／右側分開流程成功；仍建議至少補做
   中央、右側、左側安全視野內各 3 次，記錄成功率後才升級成 `hardware-approved`。

### 合併判定

- **是否可執行：是。** 可作為 supervised hardware candidate 使用。
- **是否現在直接合併 main：暫不建議。** 先完成上述 1–6 的程式／文件／PR 收尾；
  第 7 項可在 main 合併後繼續作為硬體認證工作，但在完成前不得將 package 標為
  `hardware-approved`。

---

## 現行部署基準（以 `grasp/x3plus_real_grasp.py` 為唯一來源）

- S1–S5：`hw(API) = 90 + sim_deg`，`arm_hw_invert = (False, False, False, False, False)`。
  Rosmaster API 對 S2/S3/S4 的鏡像已抵消舊的 per-joint invert。
- S6：`30° = open`、`180° = closed`；Stage 1 從 30° 往 180° 閉合。
- 本檔較早日期中出現的舊 invert／夾爪方向，僅保留為除錯歷史，不能當成目前部署設定。

## Current Status（2026-07-16）

> ⚠️ **2026-07-27 更新**：本節內容早於「TCP 定義不一致」的發現。實機夾取**從未真正成功過**
> （v17 與 v18 皆是）。根因與後續規格見本檔最末 2026-07-26/07-27 節與
> `docs/planning/GRASP_TRAINING_REQUIREMENTS_2026-07-27.md`。下方的 Phase 完成度僅指電腦端工作。

**Phase 0–7 現有視覺導航＋夾取的電腦端工作已完成，剩下上機實測；Phase 8 的 ROS
`/scan` bridge 已先完成，但 set_motor／odom／ROS 定位／巡航送物仍是規劃，尚未實作。**

已完成：

- Phase 0：環境與設備確認。2026-07-16 實機對應為 Rosmaster
  `/dev/myserial -> /dev/ttyUSB1`（CH341）；TG30 LiDAR 是 `/dev/ttyUSB0`（CP2102，driver 使用
  `/dev/rplidar` udev 別名）。不可依 ttyUSB 編號寫死；相機一律依 `startup_device_check.py`
  的 serial 標籤辨識。
- Phase 1：兩台相機內參（手臂 `RMS=0.495px` 畸變不可忽略、後鏡頭 `RMS=0.374px` 可忽略）。
- Phase 2：地面距離模型（`arm_H=0.332m/θ=36.40°`、`rear_H=0.503m/θ=16.35°`；
  回推誤差手臂 ≤0.43cm、後鏡頭 ≤0.98cm）。手臂這組數值只適用 **nav-home**。
- Phase 1/2 校正值已寫入程式常數＋手臂相機 undistort 接線（見下方 2026-07-10 節）。
- Phase 3A（nav-home）：相機到 base 座標已完成實機四點校正與套用複驗。最終
  `CAM_TO_BASE_X=0.1639m`、`CAM_TO_BASE_Y=0.0331m`、`SIGN_Y=-1`、
  前／左／右／遠四點最大誤差 X=0.39cm、Y=0.33cm。原先以 H_real−TCP z 算出的
  `z_offset=0.0488m` 已撤銷：TCP 是 arm_link5 慣性中心，不是指間中心；物體 Z 改由
  Phase 4 依 policy rollout 與實抓微調，瓶蓋首測維持訓練預設 0.02m。
- **2026-07-16 新發現**：nav-home 看得到的約 25cm 目標，離 grasp-home TCP 約 25cm，
  超過實測約 15cm 工作半徑；物體進入可抓區後 nav-home 又看不完整。Phase 3A 不能再當作
  最後 latch。控制器已改成 grasp-home settle 後才接受新偵測並加入 15cm 硬守門；bridge
  已改為 grasp-home homography fail-closed。Phase 3B（至少 6 點 homography 實標）待上機。
- 模式C RL 導航整合（PR #2）＋統一映射/Phase 3 絕對座標/校正工具修正（PR #3）皆已
  merge 進 main（`f0f2ed8`）。
- 導航權重目前預設 baseline：**`ppo_nav_281440_steps`**（成功 0.600/碰撞 0.267，優於
  best_model 的 0.467/0.367）。2026-07-16 新增候選 **`doorway_ft_final`**（標準任務
  60 episodes：成功 0.683/碰撞 0.183；doorway 10/10），契約仍是 55D/2D；預設刻意不切，
  等 Phase 5 同場地實車 A/B。
- Jetson 軟體組合預驗：Py3.8.20＋SB3 2.3.2 載入零警告、動作與開發機逐位一致。
- LiDAR 已確認為 **YDLIDAR TG30**（firmware 2.1、health good）；ROS `/scan` 約 10.17Hz，
  `laser_link`、-85°～+85°、0.01～50m。已實作 rosbridge `/scan` backend（latest-only、
  queue=1、stale 停車、REP-103 左正映射），不需 `rplidar-roboticia`。
- Phase 8 規劃已複審：統一 canonical `/odom_setmotor`、TF 鏈
  `map→odom→base_footprint→base_link→laser_link→laser`，並明確規定正式 runtime
  只能由主 pipeline 持有 `/dev/myserial`。細部 actuator／feedback odom／ROS bridge／
  stationary gate／故障復原見 `integration/SETMOTOR_ODOM_INTEGRATION.md`；除 `/scan`
  adapter 外尚未實作。
- 已確認下載的 `X3Plus_部署操作指南.md` 是舊 22D／Arm_Lib／S6 0↔160° 架構，與現行
  28D／Rosmaster_Lib／S6 30↔180° 不相容，不可拿來部署。

### 待辦（依序，全部在機器人端）

1. **整包 scp 到 Jetson**（Jetson 上還是舊版程式，必先做）：
   ```powershell
   cd C:\Users\user\Documents\repo\claude\x3plus
   .\set_jetson_host.ps1 yahboom.local
   scp -r .\* "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson2/"
   ```
2. **開機準備**：`grasp_venv` 內 `python3 -m pip install roslibpy`（首次；不要裝
   `rplidar-roboticia`）；`python3 startup_device_check.py` 確認兩相機與 Rosmaster symlink；
   回歸測試（在 `~/Documents/deploy_jetson2/` 下執行）：
   ```bash
   python3 grasp/x3plus_real_grasp.py                       # dry-run
   python3 integration/vision_grasp_pipeline.py --selftest
   python3 integration/nav_rl.py --bench                    # 2.3.2 下應零警告、≥30Hz
   ```
3. **Phase 3B grasp-home 座標對齊**：先 `--real --pose-only grasp-home`，再以 bridge
   `--calibration-only --calibration-samples 10 --once` 收至少 6 個可見／可達的非共線點，
   解 `integration/grasp_home_homography.json`；驗證點 X/Y 誤差各 <2cm。詳見
   CALIBRATION_PLAN Phase 3。
4. **Phase 5 避障測試**（`--nav-only`）：先開 `ydlidar_ros_driver/TG.launch`＋
   `rosbridge_websocket.launch`，再用 `nav_rl.py --probe --lidar-backend ros` 校方向 → `--wz-sign`
   確認 → 2m 直線（`--nav-stop-dist 1.0`→0.75）→ 繞障 → 煞停測試（量觸發到靜止的
   停止距離，首測全速 0.5 m/s 要預留空地裕度）。
5. Phase 3B 過了才做 **Phase 4 近距定點夾取**（≤15cm；含 `--smooth-approach` 實機驗證、
   跨指間隙量測）。Phase 5 `--nav-only` 可獨立進行；完整 pipeline 在 grasp-home final-align
   接妥前會拒絕 `--real`，之後才進 Phase 6 → Phase 7。
6. 備忘：訓練機 per-seed CSV 在其 scratchpad（`eval_best_model.csv`、`eval_ckpt281440.csv`），
   需要逐集失敗分析時再取。

工具備忘：`startup_device_check.py`（開機檢查）、`docs/operations/STARTUP_DEVICE_CHECK.md`（流程）、
`stream_cam.py`（瀏覽器預覽/點擊取 raw x,y）、`solve_cam_to_base.py`（Phase 3 求解）。

---

## 2026-07-10：Phase 1/2 校正值寫入程式碼 + undistort 接線（無機器人，純程式）

把 progress.md / json 裡的校正結果落地到程式常數，Phase 3 才能用正確的幾何跑 latch。

改動（三個檔案同步）：

- `integration/vision_grasp_pipeline.py`
  - 手臂相機：`FX/FY/CX/CY_ARM = 919.08/919.41/212.23/168.42`、`THETA_ARM=36.40`、`H_ARM=0.332`、
    新增 `DIST_ARM`（k1 k2 p1 p2 k3）。
  - 後鏡頭：`FX/FY/CX/CY_REAR = 544.16/544.82/316.98/244.79`、`THETA_REAR=16.35`、`H_REAR=0.503`。
  - 新增 `undistort_pixel()` / `distort_pixel()`：純數學 plumb-bob 去畸變（與
    `cv2.undistortPoints(..., P=K)` 同一套定點迭代），不用 cv2，`--selftest` 在開發機可驗。
  - `_detect_arm()`：bbox 底邊中點 `(cx_box, y2)` 先去畸變再進距離/偏移模型
    （θ 是在去畸變座標上解的，不去畸變會錯 ~9px）。後鏡頭 1.79px 依 Phase 1 決策維持 raw。
  - selftest 新增：用 Phase 2 的 4 個實測點（D=0.25~0.50m raw 點擊座標）回推，
    assert 往返誤差 <0.05px、距離誤差 <1cm。
- `integration/vision_grasp_bridge.py`：同步常數 + `undistort_pixel()`，
  main loop 的 `(cx_box, y2)` 先去畸變；docstring 校正狀態更新（內參/θ 已完成，cam-x/y 等 Phase 3）。
- `detection/arm_cam.py`：同步常數 + `undistort_pixel()`，主迴圈先去畸變再算距離/偏移。
- `docs/calibration/CALIBRATION_PLAN.md` 常數對照表①②畸變決策已填值標記完成。

驗證：`py_compile` 三檔通過；`vision_grasp_pipeline.py --selftest` 全過——
undistort 純數學版往返誤差 0.0000px，Phase 2 四點回推距離誤差最大 **0.33cm**
（確認 runtime 鏈路與校正流程一致）。

注意：**Jetson 上的 deploy_jetson2 還是舊常數**，下次開工先整包 scp 再跑。

### 同日補充 3：導航權重定案 = ppo_nav_281440_steps（訓練機評測結果）

訓練機跑完 `eval_failure_diagnostics.py --base-seed 123 --num-seeds 30`：

| 權重 | 成功率 | 碰撞率 | 超時率 |
|------|--------|--------|--------|
| best_model.zip | 0.467 (14/30) | 0.367 (11/30) | 0.167 |
| **ppo_nav_281440_steps.zip** ✅ | **0.600 (18/30)** | **0.267 (8/30)** | 0.133 |

281440 兩項指標都贏，明確勝出。已改 `nav_rl.py` 的 `DEFAULT_MODEL/DEFAULT_VECNORM`
為 281440 那對（zip+pkl 必成對）；best_model 只留作對照，上機不要用。
訓練機 repo（igibson_x3_test 分支 `nav-best-weights-export`，commit 1a0fe0a）的
README 也已同步標注；per-seed CSV 在訓練機 scratchpad。

### 同日補充 2：Jetson 軟體組合預驗 + Phase 3 求解工具（branch `phase3-prep`）

- **Jetson 組合預驗**：在開發機 conda `py38`（Python 3.8.20 + SB3 2.3.2 + torch 2.4.1+cpu +
  numpy 1.24，= grasp_venv 的組合）跑 `nav_rl.py --bench`：載入**零警告**（SB3 2.9.0 下的
  clip_range/lr_schedule 警告消失 → 2.3.2 就是訓練版本）、清空 2m 目標動作輸出與 Py3.12
  逐位一致 `[1.0, 0.43397042]`、3041 Hz；`--selftest` 全過。上機清單「SB3 2.3.2 載入測試」
  可視為預先通過。
- **新增 `integration/solve_cam_to_base.py`**：Phase 3 量完三、四個點後，
  `--pair base_x,base_y,latch_x,latch_y`（**base_link 絕對座標**；記號相對量測要先加
  `(ref_x, ref_y)` 換算）餵進去自動解 CAM_TO_BASE_X/Y + SIGN_Y，
  內建 2cm gate 判定、theta/cx 隨距離漂移警告、`--applied-*` 反解已生效常數。
  SIGN_Y 不用左右投票（絕對座標下會被相機側向偏移 cam_y 干擾而解錯號），
  改為兩種正負號各算一次 y_offset 散佈、取散佈小者。
  `--selftest`（常數還原、ref_y≠0＋相機偏右的絕對座標案例、漂移警告、gate 失敗）全過。
  CALIBRATION_PLAN Phase 3 步驟 5 已改為用此工具。

### 同日補充：導航權重電腦端驗證 + webcam dry-run 支援

- 驗證 `igibson_x3_test`（分支 `nav-best-weights-export`）的 `best_model.zip` /
  `best_vecnormalize.pkl` 與 x3plus `integration/nav_best_model/` 內的檔案 **SHA256 完全一致**
  ——mode C 用的就是訓練機 export 的那份，不用再搬。
- 電腦端 `nav_rl.py --bench`：權重載入 OK、清空 2m 目標輸出 `[1.0, +0.43]`（全速前進+微轉）、
  推論 4482 Hz（控制迴圈只需 6 Hz）。兩個 selftest 全綠。
- 新增 `--rear-stream / --arm-stream`（`nav_rl_grasp_pipeline.py`）：可用 URL 或 webcam 索引
  覆寫相機來源；`LatestFrameReader` 支援數字字串=本機 webcam。用途：電腦端
  `--nav-only --no-lidar --rear-stream 0` 跑「YOLO→GoalTracker→policy→輪速指令」dry-run。
- 電腦端已 `pip install pybullet`（3.2.7, Py3.12）；dry-run 驗證整條鏈可載入
  （pybullet FK + URDF + YOLO + nav policy），唯本機 webcam 0 無法抓幀（無相機/被佔用）。
- 訓練機待辦（iGibson 只在那台有）：`eval_failure_diagnostics.py --base-seed 123 --num-seeds 30`
  分別跑 best_model 與 ppo_nav_281440_steps，挑穩的那個；預期成功率 0.5–0.6、碰撞 0.2–0.27。

---

## 2026-07-08：Phase 1/2 實機校正進度紀錄

### 開機後設備對應與修正

> 本小節是 2026-07-08 的 USB 枚舉快照，已不是目前編號；正式規則一直是以晶片與
> `/dev/myserial` 辨識。2026-07-16 現況請看文件最上方 Current Status。

- 當次 Rosmaster 是 ch341 `/dev/ttyUSB0`，`/dev/myserial` 當時指向它。
- 當次 CP210x LiDAR 是 `/dev/ttyUSB1`；這些 ttyUSB 編號不可沿用到下一次開機。
- 已把主要入口的預設 Rosmaster port 統一改成 `/dev/myserial`，避免 USB 枚舉順序改變時誤送到 LiDAR。
- 已新增 `startup_device_check.py` 與 `docs/operations/STARTUP_DEVICE_CHECK.md`，開機後先跑設備檢查再進校正/實機測試。
- 相機 USB 重新枚舉後，目前對應為：
  - `/dev/video2`：手臂相機（原本無 SN0001 的 Sonix camera）
  - `/dev/video1`：後鏡頭（SN0001 的 Sonix camera）
- 已新增/更新 `stream_cam.py`，可用瀏覽器預覽相機並點擊畫面取得 raw `x,y` 像素座標。

### Phase 1：兩台相機內參

棋盤規格：外觀 7x10 方格，單格邊長 20 mm；OpenCV 內角點使用 `--cols 9 --rows 6 --square-mm 20`。

手臂相機（arm camera）：

- 採用候選檔：`arm_cam_intrinsics_candidate_rms0495_cx212.json`，並複製為 `arm_cam_intrinsics.json`。
- 結果：
  - `fx=919.08`
  - `fy=919.41`
  - `cx=212.23`
  - `cy=168.42`
  - `dist=[-0.3764, -0.0748, -0.0015, 0.0035, 0.4793]`
  - `RMS reprojection=0.495 px`
  - `worst view=0.914 px`
  - `distortion bbox-bottom-center=9.00 px`
- 判定：
  - RMS 剛好通過，但 principal point 明顯偏左上。
  - 手臂相機畸變不可忽略；後續 YOLO bbox bottom-center 必須先用 `cv2.undistortPoints` 校正後再進距離/座標換算。

後鏡頭（rear camera）：

- 採用檔案：`rear_cam_intrinsics.json`。
- 結果：
  - `fx=544.16`
  - `fy=544.82`
  - `cx=316.98`
  - `cy=244.79`
  - `dist=[0.1172, -0.5603, 0.0049, -0.0103, 0.6489]`
  - `RMS reprojection=0.374 px`
  - `worst view=0.587 px`
  - `distortion bbox-bottom-center=1.79 px`
- 判定：
  - Phase 1 通過。
  - 後鏡頭在 ground-distance model 使用的 bbox-bottom-center 區域畸變可先忽略。

### Phase 2：地面距離模型 H/theta

實測高度：

- `arm_H = 0.332 m`
- `rear_H = 0.503 m`

手臂相機測距點（raw 點擊座標）：

| D (m) | x | y |
|---:|---:|---:|
| 0.25 | 286 | 428 |
| 0.30 | 280 | 352 |
| 0.35 | 268 | 281 |
| 0.40 | 249 | 220 |
| 0.45 | 241 | 173 |
| 0.50 | 238 | 126 |

手臂相機 Phase 2 結果：

- raw `x,y` 先用手臂相機內參與 dist 做 undistort。
- `arm_theta = 36.40 deg`
- theta 標準差約 `0.18 deg`
- 用平均 theta 回推測距點，最大誤差約 `0.0043 m`（0.43 cm）。
- 判定：手臂相機距離模型通過；但實際流程必須先 undistort bbox bottom-center。

後鏡頭測距點（raw 點擊座標）：

| D (m) | x | y |
|---:|---:|---:|
| 0.70 | 320 | 436 |
| 0.90 | 317 | 368 |
| 1.10 | 319 | 323 |
| 1.30 | 320 | 292 |
| 1.50 | 323 | 266 |

備註：`D=0.50 m` 對後鏡頭太近，畫面看不到，因此不納入模型。

後鏡頭 Phase 2 結果：

- `rear_theta = 16.35 deg`
- theta 標準差約 `0.088 deg`
- 用平均 theta 回推測距點，最大誤差約 `0.0098 m`（0.98 cm）。
- 有效實測範圍：`0.70 m ~ 1.50 m`。
- 判定：後鏡頭距離模型通過。

### 目前 gate 狀態

- Phase 1：可視為通過。
  - 手臂相機：可用候選內參，但需在後續流程強制 undistort。
  - 後鏡頭：通過且可先忽略 ground-distance bbox-bottom-center 畸變。
- Phase 2：通過。
  - `arm_H=0.332`, `arm_theta_deg=36.40`
  - `rear_H=0.503`, `rear_theta_deg=16.35`
- 下一步：Phase 3，做 YOLO/相機地面座標到手臂/夾取座標對齊，標定 `cam_x`, `cam_y`, `sign_y`, `z_offset`。

## 2026-05-04

### 完成事項
- 建立專題工作資料夾 `C:\Users\user\Documents\repo\claude\x3plus\`
- 撰寫 `CLAUDE.md`（專題架構、關節映射、API 對照、部署指令）
- 將 `x3plus_real_grasp.py` 從 `Arm_Lib` 移植到 `Rosmaster_Lib`：
  - 替換 import 與裝置初始化（`Rosmaster()` + `create_receive_threading()`）
  - `Arm_serial_servo_write6_array` → `set_uart_servo_angle_array(angle_s=[...], run_time=t)`
  - `Arm_serial_servo_read(i)` → `get_uart_servo_angle(i)`
- Code review 後進行第二輪重構：
  - `DeployConfig` 新增 `serial_port`（預設 `/dev/ttyUSB0`）
  - `ServoController` 改用 `self._device`、`self._has_servo` 實例變數，移除全域 `HAS_ARM_LIB`
  - 裝置連線延遲到 `ServoController.__init__`（不在 import 時建立）
  - 新增 `ServoController.close()` 清理背景執行緒
  - CLI 新增 `--port` 參數
- 將所有部署必要檔案複製到 `x3plus/` 資料夾

### 待辦 / 未解決
- [ ] 在 Jetson 上複製 `Rosmaster_Lib` 驅動（本地化）
- [ ] 空跑測試：確認角度輸出合理
- [ ] 實體驗證 S3、S4 關節方向（`arm_hw_invert`）
- [ ] 確認 `run_time` 單位（毫秒）在 Rosmaster_Lib 上行為正確

### 目前狀態
程式碼修改完成。Jetson Nano 端尚未執行任何步驟，待辦事項全部未開始。

---

## 2026-05-08

### 完成事項
- 用 `scp` 將整個 `x3plus/` 資料夾從 Windows 傳到 Jetson：
  ```
  scp -r "C:\Users\user\Documents\repo\claude\x3plus" "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson"
  ```
- 找到 Rosmaster_Lib 實際位置（不在 python3.6 dist-packages，而在）：
  `/home/jetson/software/py_install/Rosmaster_Lib`
- 將 Rosmaster_Lib 本地化到部署資料夾：
  ```
  cp -r /home/jetson/software/py_install/Rosmaster_Lib .
  ```
- 確認虛擬環境為 `~/grasp_venv`（Python 3.8.0），成功啟動：
  ```
  source ~/grasp_venv/bin/activate
  ```

### 完成事項（補充）
- 安裝套件（`pip install -r requirements_jetson.txt`）：
  - stable-baselines3 2.2.1 → 2.3.2（升級）
  - gymnasium、pybullet、numpy、torch 已在 grasp_venv 中，全部 OK
- 整理資料夾結構（scp 產生多一層 x3plus/x3plus 巢狀，手動 mv 修正）
- 確認 URDF joint index：arm=[8,9,10,11,12]，grip=13，ee_link=12 ✓
- 確認 home TCP 位置：[0.098, 0.018, 0.593]（z=59cm）
- 空跑測試 #1：預設物體 (0.25, 0, 0.02)，policy 振盪，dist 一直在 0.55m

### 發現的 Bug（已修復）
1. **Observation desync（最關鍵）**：`_current_arm_rads` 更新為 policy action target，但 rate limiter 實際只送出 8°/step。導致 policy 下一步以為已到達 target，造成持續振盪。
   - 修法：`_current_arm_rads` 改用 `self.servo._last_deg`（rate-limited 實際值）反算，Stage 0 和 Stage 1 均已修正。

2. **Stage 2 return-home 邏輯錯誤**：Stage 2 把 policy 輸出的 target 角度拿來和 home 比距離，但 policy 在 Stage 2 輸出 S2=180°（極限值），永遠不收斂。
   - 修法：Stage 2 完全繞過 policy，直接下 home 指令，用 rate-limited 實際位置判斷是否到位。

3. **stage2_dist_threshold 未使用**：程式碼寫死 `0.15`，改用 `self.cfg.stage2_dist_threshold`。

### 發現的環境問題
- 預設物體高度 z=0.02m（地板）超出訓練分布：workspace scan 顯示機械臂最低可達 z=-0.06m（硬體上可達），但 policy 從未學會從 home 走到那麼低的位置，導致 Stage 0 卡在 0.55m 收斂。
- 改用 z=0.15m 後 policy 正常收斂。

### 空跑測試 #2 結果（z=0.15m，修復後）
全流程通過：
- Stage 0：dist 0.277 → 0.176m（4步），grip_cmd 超閾值 → 觸發 Stage 1 ✓
- Stage 1：S6 夾爪從 180° 閉合到 30°，共 20 步 ✓
- Stage 2：dist 1.038 → 0.025rad，印出 `Grasp complete!` ✓

### 序列埠確認
- 此處是早期歷史紀錄且裝置判讀已淘汰；目前一律以
  `/dev/myserial`→ch341=Rosmaster、CP2102=TG30 判定，不照抄 ttyUSB 編號。

### 待辦
- [ ] 實體測試：`python3 x3plus_real_grasp.py --real --obj-x 0.25 --obj-y 0.0 --obj-z 0.15`
- [ ] 目視確認 home 位置正常（各關節直立，夾爪全開）
- [ ] 確認運動方向不暴衝、不撞自己
- [ ] 實體驗證 S3、S4 關節方向（`arm_hw_invert`）
- [ ] 逐步調低 obj-z，找出 policy 實際可達的最低高度

### 目前狀態
Dry-run 全流程驗證完成。準備進行第一次實體伺服機測試（`--real`）。

---

## 2026-05-08（實體測試）

### 發現的問題與修復

**問題 1：pyserial 未安裝**
- 症狀：`[WARNING] Rosmaster_Lib not found`，`--real` 模式 fallback 到 dry-run
- 修法：`pip install pyserial`

**問題 2：序列埠用錯**
- 預設 `/dev/ttyUSB0`（cp210x）是其他裝置，Rosmaster 板實際在 `/dev/ttyUSB1`（ch341）
- 確認方式：`ls -la /dev/myserial` → 指向 `ttyUSB1`
- 當時修法曾改成 `/dev/ttyUSB1`；目前已再次修正為穩定別名 `/dev/myserial`，不得沿用
  這個歷史 ttyUSB 編號。

**問題 3：座標系 X 軸反向（最關鍵）**
- 症狀：實體測試 `--obj-x 0.25` 時，S2 往 180° 走，手臂物理上往後撞
- 根本原因：實體機械臂安裝方向與 URDF 假設的「前方」相差 180°，URDF +X = 物理後方
- 診斷過程：
  - 測試 `--obj-x -0.25`：手臂往前走（方向正確），但 policy 從未訓練過負 X 物體，dist 停在 0.370m 無法收斂
  - 確認：物理前方 = URDF -X 方向
- 修法：在 `FKComputer.compute()` 回傳前對 TCP 做 180° Z 軸旋轉（翻轉 X 和 Y）：
  ```python
  tcp_pos[0] = -tcp_pos[0]
  tcp_pos[1] = -tcp_pos[1]
  ```
  讓 policy 的座標系重新對齊物理前方，obj_x 繼續用正值（0.25）

**其他調整**
- `max_delta_deg`：8.0° → 3.0°（提高安全性）

### 待辦
- [ ] 傳更新腳本到 Jetson，測試座標翻轉修正後手臂是否往前走且 dist 收斂
- [ ] 若收斂，觀察 Stage 1 夾爪是否在正確位置閉合
- [ ] 實體驗證 S3、S4 關節方向（`arm_hw_invert`）
- [ ] 確認夾取成功後逐步調低 obj-z，測試實際可達的最低高度
- [ ] max_delta_deg 測試穩定後可視情況調回 8.0°

### 目前狀態
座標系修正已寫入程式碼但尚未在實體上測試。下一步：傳檔案到 Jetson 跑 `--real` 確認修正是否有效。

---

## 2026-05-11

### 實測結果
- 座標翻轉（X 軸）確認有效：手臂向前抓取成功 ✓
- 問題 1：夾爪向旁邊旋轉（S5 異常）
- 問題 2：手臂沒有向下抓取（z 軸不足）

### 問題診斷與修復

**Bug 1：四元數未跟著座標翻轉（夾爪旁轉根本原因）**
- 根本原因：`FKComputer.compute()` 只翻轉了 `tcp_pos`（X、Y 取負），但 `tcp_quat` 仍是 URDF 原始座標系的值。Policy 看到位置已在物理座標系，卻看到 URDF 座標系的方向 → 感知方向誤差 → 命令 S5 旋轉補正。
- 修法：在翻轉 pos 後，對 quaternion 同樣施加 180° Z 旋轉：
  - Rz180 = (0, 0, 1, 0)；Hamilton product 結果 = `(-qy, qx, qw, -qz)`
  - 程式碼：`tcp_quat = np.array([-qy, qx, qw, -qz], dtype=np.float32)`

**Bug 2：Stage 0→1 用 target TCP 而非當前 TCP 判距離（提早夾取）**
- 根本原因：`dist_to_obj` 是從 action 轉換後的 target 角度做 FK，不是目前實際位置。手臂還沒到就觸發 Stage 1 → 夾到空氣。
- 修法：先用 `self._current_arm_rads` / `self._current_grip_rad` 算 current_tcp，再計算距離。

---

## 2026-05-11（第二次實測 + 程式碼稽核）

### 實測結果
- 傳更新腳本（四元數翻轉 + Stage 0→1 距離修正）到 Jetson 實測
- **手臂沒有動**：log 顯示 Rosmaster_Lib 連線成功，但 Stage 0 期間 dist 一直卡在 0.674m（= home 到物體的距離），代表手臂物理上未移動
- Stage 0→1 在第 25 步因 `grip_cmd=0.40 > grip_close_threshold=0.35` 提早觸發（非距離觸發）
- Stage 2 瞬間完成（手臂仍在 home 附近）

### 發現的 Bug 與修復

**Bug 1：grip_close_threshold 過低，Stage 1 提早觸發**
- 根本原因：policy 在離物體 0.674m 時就輸出 grip_cmd=0.40，超過閾值 0.35，Stage 1 在手臂還沒移動前就觸發
- 修法：`grip_close_threshold` 0.35 → 0.90（等同停用 grip 觸發，改為只靠距離觸發）

**Bug 2：move_to_home() / emergency_stop() 通過 rate limiter**
- 根本原因：兩者都呼叫 `send_degrees()`，rate limiter 每次只允許移動 `max_delta_deg=3.0°`，但兩者只呼叫一次，實際只移動 3°，無法真正回 home；緊急停止也失效
- 修法：新增 `_send_direct()` 方法跳過 rate limiter，直接將目標角度送給伺服機（由硬體在指定 run_time 內自行執行），`move_to_home()` 和 `emergency_stop()` 改用此方法

**Bug 3：Stage 1 強制閉合值錯誤**
- 根本原因：`action[5] = 0.9` 對應 hw_grip ≈ 37.5°，而非設定的 `gripper_hw_closed=30°`
- 修法：改為 `action[5] = 1.0`，正確對應 30°

**Bug 4：VecNormalize 載入失敗繼續跑**
- 根本原因：載入失敗只印 warning，以未正規化觀測繼續執行 policy，輸出會失真
- 修法：`--real` 模式下載入失敗直接 raise RuntimeError，中止執行

### 待辦
- [ ] 在 Jetson 執行最小測試腳本，確認 Rosmaster_Lib 能直接驅動伺服機（繞過部署腳本）
- [ ] 確認序列埠（/dev/ttyUSB1）仍正確對應 Rosmaster 板
- [ ] 若硬體確認 OK，傳更新腳本再次實測
- [ ] 確認 Stage 0→1 在 dist < 0.045m 時才觸發
- [ ] 確認夾爪不再旁轉（四元數修正）
- [ ] 驗證 S3/S4 關節方向（`arm_hw_invert`）
- [ ] 逐步調低 obj-z（0.15 → 0.10 → 0.05）

### 目前狀態
手臂實體無動作，懷疑序列埠或伺服機驅動層問題，正在診斷。程式碼已修四個 Bug，待硬體確認後再次實測。

---

## 2026-05-18（Session 1）

### 完成事項
- 取得 Rosmaster_Lib v3.3.9 原始碼，存為 `Rosmaster_Lib_reference.py`（僅供參考，不部署）
- 發現關鍵 silent fail 機制：
  - `set_uart_servo_angle_array` 的角度上限：S1-S4=[0,180]、S5=[0,270]、S6=[0,180]
  - 任一關節超界 → 只印 `angle_s input error!` 然後直接 return，整包指令丟棄，不動作、不報例外
  - 這很可能是手臂實體完全不動的根本原因（rate limiter 每步 +3° 累積後可能超出某關節上限）
- 在 `send_degrees()` 和 `_send_direct()` 新增 hw 角度 log：
  - 每步印出六個關節的實際 hw 角度
  - 若有超界關節，自動標記 `*** OOB S[n] ***`
  - dry-run 和 real 模式均有效

### 實測結果（obj-z 下探，S3 修正前）
跑 `--real` 四個高度，所有情況手臂都收斂到錯誤姿態：
- dist 停在 0.54–0.69m，未收斂（home 到物體距離約 0.27m，代表 FK 算出的 TCP 完全錯誤）
- 夾爪末端向上，未向下
- S3 在所有測試中均收斂到 < 90°（52–74°），物理上與模擬方向相反

### 診斷與修復

**根本原因：S3 關節方向反了**
- `arm_hw_invert` 的 S3 原為 False → hw = 90 + sim_deg
- Policy 對 S3 持續輸出負 sim 角度（希望手臂往下），但 False 設定讓實體往相反方向動
- 此錯誤同時導致 FK 算出的 TCP 位置錯誤 → dist 永遠無法收斂
- 修法：`arm_hw_invert` S3 改為 True → `(False, True, True, False, False)`

### 待辦
- [x] 傳更新腳本到 Jetson（`scp x3plus_real_grasp.py "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson/"`）
- [ ] dry-run 確認 S3 角度從 < 90° 變為 > 90°（方向修正確認）
- [ ] `--real` 跑 z=0.15，確認夾爪向下、dist 收斂
- [ ] 若收斂，進行 z=0.10 → 0.05 → 0.02 下探測試
- [ ] 若夾爪仍向上，將 S4 也改為 True：`(False, True, True, True, False)`

### obj-z 下探步驟（逐步測試最低夾取高度）

**原則**：每次只改 `--obj-z`，其他參數不變。先跑 dry-run 確認 dist 收斂，再跑 `--real`。

**判斷標準**：
- ✅ 成功：Stage 0 dist 持續下降，最終 < 0.045m 觸發 Stage 1
- ⚠️ 退化：Stage 0 dist 停滯（> 0.15m 超過 30 步），policy 找不到路徑
- ❌ 失敗：手臂撞到地板或自身，或 Stage 1 在空中閉合

**測試序列**：

```bash
# Step 1：已知可行的基準
python3 x3plus_real_grasp.py --real --obj-x 0.25 --obj-z 0.15

# Step 2：往下 5cm
python3 x3plus_real_grasp.py --real --obj-x 0.25 --obj-z 0.10

# Step 3：再往下 5cm
python3 x3plus_real_grasp.py --real --obj-x 0.25 --obj-z 0.05

# Step 4：如果 0.05 成功，試 0.02（地板高度）
python3 x3plus_real_grasp.py --real --obj-x 0.25 --obj-z 0.02
```

**每次觀察 log 的重點**：
1. Stage 0 的 dist 趨勢（前 10 步是否有下降）
2. 是否出現 `*** OOB ***`（角度超界）
3. Stage 1 觸發時的實際 dist（應 < 0.045m）

**若某個高度退化**：不要繼續往下，以上一個成功高度作為最低可夾高度。

### 目前狀態
新增 OOB 角度 log，待傳檔到 Jetson 驗證 silent fail 假說。obj-z 下探測試待實體確認 OK 後進行。

---

## 2026-05-18（Session 2）

### 關節方向全確認

| 關節 | arm_hw_invert | 說明 | 狀態 |
|------|-------------|------|------|
| S1 | False | hw = 90 + sim_deg | 已確認 |
| S2 | True | hw = 90 − sim_deg，> 90° = 向前 | 已確認 |
| S3 | True | hw = 90 − sim_deg，> 90° = 向前 | 已確認 |
| S4 | **True** | hw = 90 − sim_deg，> 90° = 向前 | **本次確認** |
| S5 | False | hw = 90 + sim_deg | 已確認 |

**S4 診斷過程**：第一次實測（S4=False）log 顯示 S4 hw 從 90° 一路降到 4°（往錯方向），改為 True 後 hw 升到 173°（正確向前）。

### 座標翻轉最終決定：移除所有翻轉

背景：`FKComputer.compute()` 曾多次修改（X flip、Y flip、四元數 flip 等組合）。
- X+Y flip + 四元數 flip → S3/S4 往後（四元數偏離訓練 distribution）
- X+Y flip，無四元數 flip → S5 快速旋轉（位置與方向座標系不一致）
- X flip 只，無 flip → FK tcp_x = -0.098，VecNormalize 後約 -4σ（OOD）
- **移除全部 flip（當前狀態）→ 手臂正常向前收斂** ✓

結論：最初 S2→180° 問題是 VecNormalize 載入失敗（Bug 4）造成，而非座標系問題；座標系本身不需要翻轉。

### 實測結果（移除所有翻轉 + S4=True 後）

兩次實測（S4 修正前後），dist 軌跡相同（S4 對 TCP 位置影響小）：
- dist: 0.469m → **0.120m**（step 18–19）→ 發散到 0.245m → 卡在 0.239m
- S2/S3/S4 全部到 180° 上限（hw 最大值），手臂過度彎曲後 TCP 反向移動
- grip_cmd 穩定在 0.79，未達 0.90 門檻，Stage 1 從未觸發

**根本問題**：在當前物體位置 (0.25, 0, 0.15) 下，FK 計算的最近距離約 0.12m，遠大於 Stage 1 門檻 0.045m。手臂關節全部達到硬體上限後 TCP 反向，策略卡住。

### 修改內容

1. **arm_hw_invert** S4 改為 True：`(False, True, True, True, False)`
2. **Stage 1 備用觸發器**：
   - 追蹤 `_min_dist_seen`（每步更新最近距離記錄）
   - 若 dist 從最小值回升超過 `min_dist_rebound_m=0.05m` → 自動觸發 Stage 1（`reason=diverging`）
   - 觸發時印出 reason（dist_thresh / grip_thresh / diverging）
3. **stage0_dist_threshold**：0.045m → 0.05m（保持相近，主要靠 rebound trigger）
4. **TCP 位置 log**：
   - 啟動時印 `[Start] Home TCP (URDF): [x, y, z]`
   - 每步印 `tcp=[x, y, z]`（讓後續校正 FK 偏移量時有參考）

### 待辦
- [ ] 傳更新腳本到 Jetson：`scp x3plus_real_grasp.py "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson/"`
- [ ] 執行：`python3 x3plus_real_grasp.py --real --obj-x 0.25 --obj-y 0.0 --obj-z 0.15`
- [ ] 記錄 `[Start] Home TCP (URDF)` 數值（了解 FK 座標系原點）
- [ ] 觀察 Stage 1 觸發時機（reason=diverging，應在 step ~25）
- [ ] 目視確認：Stage 1 觸發時夾爪距物體的物理距離（判斷是否 FK 偏移 vs 真的夠遠）
- [ ] 若夾爪物理上已很近（< 5cm）但 FK 說 0.12m → FK 有偏移，調整 obj 座標或加固定 offset
- [ ] 若夾爪物理上仍很遠 → 物體位置超出機械臂可達範圍，試 obj-x 0.20 或 obj-z 0.10

### 目前狀態
所有關節方向已確認。手臂能正確向前移動（dist 從 0.47m 收斂到 0.12m），但受硬體關節上限制約，無法繼續靠近。Stage 1 rebound trigger 已加入，下次實測可觀察夾爪物理位置來判斷 FK 偏移程度。

---

## 2026-05-18（Session 3）

### 實測結果（rebound trigger 版本）

```
Home TCP (URDF): [0.098, 0.018, 0.593]
物體位置: [0.25, 0.0, 0.02]
```

- Step 1：Policy 第一步輸出極端動作（S5 想從 90° 跳到 152°，S1 跳到 110°），被 rate limiter 截斷
- S2 hw 從 90° 降到 87°（物理往後），S4 hw 從 90° 降到 81°（物理往後）
- S3 hw 從 90° 升到 124.9°（正確向前），但 step 12 後就振盪不再移動
- Step 16 起 dist 卡在 0.548–0.549m，TCP 在兩點之間交替：
  `[0.211, -0.005, 0.567] ↔ [0.210, +0.001, 0.567]`
- **TCP Z 從 0.593m 只降到 0.567m（僅降 2.6cm），物體在 Z=0.02m，差距 0.547m**

### 根本原因發現：訓練初始姿勢 ≠ 實體 Home 姿勢

**症狀**：Policy 第一步就輸出偏離 home 很遠的目標角度（S5 +62°、S1 +20°），顯示 policy 看到 home 的 obs 時完全「不認識」這個狀態。

**根本原因**：PPO 訓練時的 starting pose 很可能不是全關節 90°（直立）。訓練 obs 的統計分布（VecNormalize mean/var）是從另一個初始姿勢展開的，實體從 home 出發的 obs 落在訓練分布之外（out-of-distribution），policy 輸出無意義動作並快速振盪。

**佐證**：
- S5 第一步目標 152°，policy 「想回到熟悉的狀態」
- 前次測試（dist 從 0.47m → 0.12m 成功收斂）可能是因為手臂由不同起始位置啟動
- 現在從 home (TCP Z=0.593m) 出發，Z 幾乎不降，dist 一直卡在 0.548m

### 待辦

- [ ] **（優先）** 在 Jetson 上讀出 VecNormalize obs 均值，確認訓練時的典型關節角度：
  ```bash
  python3 -c "
  import pickle, numpy as np
  with open('trained_6d_models_v5/vecnormalize_6d_final.pkl','rb') as f:
      vn = pickle.load(f)
  mean = vn.obs_rms.mean
  labels = ['S1','S2','S3','S4','S5','grip','tcp_x','tcp_y','tcp_z',
            'q_x','q_y','q_z','q_w','obj_x','obj_y','obj_z',
            'rel_x','rel_y','rel_z','st0','st1','st2',
            'a0','a1','a2','a3','a4','a5']
  for i,(l,m) in enumerate(zip(labels,mean)):
      print(f'[{i:2d}] {l:6s} {m:8.4f}')
  "
  ```
- [ ] 根據 obs 均值中 [0:5]（S1-S5 的 sim 角度均值）換算出 hw 角度，得出「訓練 home pose」
- [ ] 在部署腳本加入「預定位」階段：啟動 policy 前先把手臂移到訓練 home pose，再開始推論
- [ ] 重新實測，觀察 dist 是否恢復前次（0.47m → 0.12m）的收斂行為

### 目前狀態
確認訓練初始姿勢與實體 home（全關節 90°）不一致是 dist 卡住的根本原因。待讀出 VecNormalize 均值 → 換算訓練 home pose → 加入預定位邏輯後再測。

---

## 2026-05-22

### 完成事項
- 確認訓練初始姿勢（hw degrees）：[S1, S2, S3, S4, S5, S6] = [90, 40, 180, 180, 90, 30]
- 加入預定位（preposition）邏輯，解決 OOD 問題：
  1. `DeployConfig` 新增 `train_home_deg = (90, 40, 180, 180, 90, 30)`、`preposition_run_time_ms = 3000`、`preposition_wait_sec = 4.0`
  2. `ServoController.move_to_train_home()` — 以 3000ms 緩慢移至訓練起始姿勢
  3. `GraspController.__init__` 的初始 `_current_arm_rads` / `_current_grip_rad` 改從 `train_home_deg` 算
  4. `GraspController.run()` 在 policy 推論前加兩步：先到 safety home（全 90°），再到 training home pose

### 執行流程（修改後）
```
[Start] Moving to safety home position...  ← 全 90°，run_time=1500ms，等 2.5s
[Start] Pre-positioning to training home pose [90,40,180,180,90,30]...  ← run_time=3000ms，等 4s
[Start] Home TCP (URDF): ...               ← policy 開始推論
```

### 待辦
- [ ] 傳更新腳本到 Jetson：`scp x3plus_real_grasp.py "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson/"`
- [ ] dry-run 確認：S2 hw 從 90° 降到 40°，S3/S4 hw 升到 180°，S6 降到 30°（閉合）
- [ ] `--real` 跑 `--obj-x 0.25 --obj-z 0.15`，觀察 dist 前 10 步是否穩定下降（期望恢復 0.47m → 0.12m 的收斂）
- [ ] 若收斂，進行 obj-z 下探（0.10 → 0.05 → 0.02）

### 目前狀態
預定位邏輯已寫入腳本，尚未在實體上測試。待傳檔並執行 dry-run 確認角度輸出正確。

---

## 2026-05-22（Session 2）

### Dry-run 確認預定位修改成功
- Stage 0：dist 從 0.063m 起步，3 步觸發 `dist_thresh`（reason=dist_thresh，dist=0.045m）
- 對比之前：dist 卡在 0.548m 從未收斂 → **OOD 問題完全解決**
- Home TCP (URDF): `[0.244, 0.018, 0.21]`，與物體位置 `[0.25, 0, 0.15]` 距離 0.063m

### 從 dry-run log 發現的兩個新 Bug

**Bug 1：夾爪 hw open/closed 方向反了**
- 程式碼設定：`gripper_hw_open=180`、`gripper_hw_closed=30`
- 用戶實測：hw=30° = **打開**（fingers spread），hw=180° = **閉合**（fingers together）
- 後果：Stage 1 強制 `action[5]=1.0`（sim_closed=0.0）→ 程式送 hw=30° → 實體上是**保持打開**
- log 證據：Stage 1 期間 S6 一路從 39° 降到 30° 然後鎖死在 30°，根本沒夾
- 修法：swap `DeployConfig` 的 `gripper_hw_open`（→30）和 `gripper_hw_closed`（→180）

**Bug 2：Stage 1 期間手臂被 policy 帶飛**
- Stage 1 只強制 `action[5]=1.0`，`action[0:5]`（arm target）仍是 policy 輸出
- log 證據：dist 從 0.045m → 0.186m，手臂遠離物體
- 修法：Stage 1 第一步快照當前 hw arm 角度（`servo._last_deg[:5]`），整個 Stage 1 鎖住不動，policy 的 arm action 完全忽略
- 新增 `_stage1_arm_hold` 狀態變數

### 連帶修改
- Stage 2 顯式保持夾爪閉合（`home_deg[5]=30°` 在新方向下是「打開」，須以 `gripper_hw_closed=180°` 覆蓋）
- Stage 2 完成訊息改為 `[Done] Returned to home. Object held.`
- 新增 `_grasp_succeeded` 旗標：成功夾取後不再呼叫 `move_to_home()`（會放開物體）
- 失敗（max_steps 用盡）才走 `move_to_home()` 釋放重置

### 修改清單（`x3plus_real_grasp.py`）
1. `DeployConfig.gripper_hw_open` 180→30、`gripper_hw_closed` 30→180
2. Stage 1 改寫：arm 鎖住、印 `Arm locked at [...]` 訊息、log 多印 S6 度數
3. `__init__` 與 `run()` 新增 `_stage1_arm_hold = None`、`_grasp_succeeded = False`
4. Stage 2 改回顯式 `list(home_deg[:5]) + [gripper_hw_closed]`
5. End-of-loop：`_grasp_succeeded=True` 則不呼叫 `move_to_home()`

### 待辦
- [ ] 傳更新腳本到 Jetson：`scp x3plus_real_grasp.py "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson/"`
- [ ] Dry-run 確認：
  - Stage 1 印 `Arm locked at [...]`
  - S6 從 30° 上升到 180°（不再卡在 30°）
  - Stage 1 期間 S1-S5 完全不變
  - Stage 2 期間 S6 = 180°
  - 結束訊息 `[Done] Returned to home. Object held.`
- [ ] dry-run OK 後實體驗證 `--real`

### 目前狀態
五處修改完成。預定位修改已驗證 OOD 解決，Stage 1 / 夾爪方向修正待 dry-run 驗證。

---

## 2026-05-22（Session 3）

### Dry-run 驗證 Stage 1 / 夾爪修正成功
- Stage 1 印出 `Arm locked at [86.3, 49.0, 180.0, 180.0, 81.0]` ✓
- S6 從 30° 一路 +3°/step 升到 180°（51 步）✓
- Stage 1 期間 S1-S5 完全不變 ✓
- Stage 1 期間 dist 穩定在 0.047m ✓
- Stage 2 三步回 home，S6 維持 180° ✓
- `[Done] Returned to home. Object held.` ✓
- `[End] Grasp succeeded — keeping gripper closed, arm already at home.` ✓

### 但實體 `--real` 發現新問題：Rosmaster_Lib API ↔ Yahboom App 鏡像差異

實體跑到 `[REAL-HOME] S1=90.0° S2=40.0° S3=180.0° S4=180.0° S5=90.0° S6=30.0°` 時：
- 手機 App 顯示 `[90, 140, 0, 0, 90, 30]`
- 夾爪實體**朝後**（不是訓練時面向 +X 的姿態）

對照關係：
| 關節 | API 送出 | App 顯示 | 關係 |
|------|---------|---------|------|
| S1 | 90 | 90 | 相同 |
| **S2** | **40** | **140** | 180 − x |
| **S3** | **180** | **0** | 180 − x |
| **S4** | **180** | **0** | 180 − x |
| S5 | 90 | 90 | 相同 |
| S6 | 30 | 30 | 相同 |

**根本原因**：S2/S3/S4 servo 機械安裝方向可能反了，Rosmaster_Lib API 寫入的角度被內部鏡像（180 − x）後才到伺服機；App 讀的是 servo 真實角度。

### 已套用的測試修改

`home_deg` 改為 API convention 鏡像值，使 App 顯示能對到訓練姿勢：
```python
# Before
home_deg = (90.0, 40.0, 180.0, 180.0, 90.0, 30.0)
# After (this session)
home_deg = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)
```

預期：送這組 API 值 → App 顯示鏡像後 `[90, 40, 180, 180, 90, 30]` → 物理上是訓練姿勢，夾爪向前。

### 待辦（休息後接續）

**第一階段：驗證鏡像假說**
- [ ] `scp x3plus_real_grasp.py "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson/"`
- [ ] 跑 `python3 x3plus_real_grasp.py --real --obj-x 0.25 --obj-z 0.15`
- [ ] **重要：在 `[REAL-HOME]` 印出後立刻按 Ctrl+C 中斷**，先別讓 policy 推論
- [ ] 觀察手機 App 是否顯示 `[90, 40, 180, 180, 90, 30]`
- [ ] 目視確認夾爪是否面向前方（+X 方向）

**第二階段（情況 A：假說成立）**
若 App 顯示對且夾爪面向前方，要處理 `arm_hw_invert` 的潛在問題：
- 過去 `arm_hw_invert = (False, True, True, True, False)` 是基於 API 視角校準
- 鏡像若存在，policy 的 sim → API → 物理 motion 方向可能反了
- [ ] 設計小型動作測試：手動讓 S2/S3/S4 各動一個小幅度，比較 API 命令方向 vs App 顯示方向 vs 物理運動方向
- [ ] 決定是否要把 `arm_hw_invert` 翻為 `(False, False, False, False, False)`
- [ ] 若翻 invert，FK 計算的 sim 值會自動保持一致（home_deg 鏡像 + invert 反向 互相抵消），policy 不需重訓

**第二階段（情況 B：假說不成立）**
若 App 顯示不是 `[90, 40, 180, 180, 90, 30]` 或夾爪仍朝後：
- [ ] 把 App 實際顯示的六個數值記下來
- [ ] 重新推導 API ↔ App 的轉換公式（可能不是純鏡像）
- [ ] 看看是否需要為個別關節做不同處理

**第二階段（情況 C：手臂動作異常）**
- [ ] 立刻 Ctrl+C
- [ ] 紀錄錯誤訊息和最後幾行 log
- [ ] 不要再跑 `--real`，先除錯

**最終驗證（所有情況通過後）**
- [ ] dry-run + real 完整跑完，policy 能實際抓到物體
- [ ] obj-z 下探：0.15 → 0.10 → 0.05 → 0.02
- [ ] 更新 CLAUDE.md 的 Sim-to-Real 關節映射表（反映實際 API/App 關係）

### 目前狀態
程式碼 `home_deg` 已改為鏡像值 `(90, 140, 0, 0, 90, 30)`，**尚未在實體驗證**。下次接續時，先做「第一階段」確認 home 姿勢正確，再依結果走「第二階段」對應分支。

---

## 2026-05-25

### API 鏡像假說驗證 → 情況 A 成立

- [x] `home_deg = (90, 140, 0, 0, 90, 30)` → App 顯示 `[90, 40, 180, 180, 90, 30]` ✓
- [x] 夾爪面向前方 ✓
- [x] 但 policy 推論時手臂**往後撞**，dist 從 0.304m 持續上升

### 根本原因：arm_hw_invert 雙重翻轉

`arm_hw_invert = (False, True, True, True, False)` 是在假設 API = 物理角度時校準的。
API 鏡像（API = 180 − 物理）本身就是一次 invert，加上 `invert=True` 等於雙重翻轉。

| 關節 | API值 | invert=True sim | 訓練正確 sim | 差異 |
|------|-------|----------------|-------------|------|
| S2 | 140 | 90−140 = −50° | +50° | 完全反 |
| S3 | 0 | 90−0 = +90° | −90° | 完全反 |
| S4 | 0 | 90−0 = +90° | −90° | 完全反 |

FK 告訴 policy 手臂在鏡像位置 → policy 輸出反方向動作 → dist 越來越大。

### 修法

`arm_hw_invert` 全部改為 `False`：`(False, False, False, False, False)`

驗算：
- S2: sim = 140−90 = +50° ✓
- S3: sim = 0−90 = −90° ✓
- 動作方向也一致（policy 減小 S2 sim → API 減小 → physical 增大 → 前傾 ✓）

### 修正後實測結果

**正前方 (0.25, 0, 0.15)** — 全流程成功：
- dist: 0.063m → 0.049m（2 步觸發 dist_thresh）
- Stage 1：S6 從 30°→180° 閉合 ✓
- Stage 2：3 步回 home ✓
- `Grasp succeeded` ✓

**左 (0.25, +0.05, 0.15)** — 成功：
- S1: 90°→81°（左轉 ✓）
- dist: 0.068m → 0.049m（2 步觸發）✓

**右 (0.25, −0.05, 0.15)** — 成功（但 diverging 觸發）：
- S1: 90°→96.3°（右轉 ✓）
- dist: 0.091m → 0.058m（min），回升到 0.108m 觸發 diverging
- Stage 1 觸發時 dist=0.108m（離物體稍遠）

### obj-z 下探測試（x=0.30）

| z | 初始 dist | min dist | 觸發方式 | 觸發 dist | 結果 |
|---|----------|----------|----------|----------|------|
| 0.17 | 0.071m | 0.071m | diverging | 0.123m | 完成但 dist 偏大 |
| 0.10 | 0.125m | 0.068m | diverging | 0.118m | 完成（手臂到 S2=23°）；Stage 2 太慢，Ctrl+C |
| 0.05 | 0.171m | 0.171m（沒改善）| diverging | 0.232m | **失敗 — 手臂沒往下** |
| 0.02 | 0.199m | 0.199m（沒改善）| diverging | 0.265m | **失敗 — 手臂沒往下** |

### z=0.05/0.02 失敗的根本原因：diverging trigger 太早觸發

- Rate limiter 每步 3°，前幾步 TCP 自然先上偏（因為從 training home 出發的前幾度動作偏向延伸而非下壓）
- z=0.10 也有同樣的初始上偏，但 30 步後成功收斂到 0.068m
- z=0.05 只跑了 7 步就被 diverging（rebound 0.082 > 0.05）殺掉

### 修法：diverging trigger 增加「已靠近」前提

舊邏輯：`step > 5 and min_dist < 0.25 and rebound > 0.05`
新邏輯：`step > 10 and min_dist < initial_dist * 0.6 and rebound > 0.05`

- 要求 min_dist 比初始距離改善 40% 以上才允許觸發
- 避免「手臂還沒開始動就被判定 diverging」

程式碼已修改，**尚未在 Jetson 上測試**。

### 待辦

- [ ] 傳更新腳本到 Jetson：`scp x3plus_real_grasp.py "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson/"`
- [ ] 重測 z=0.05：`python3 x3plus_real_grasp.py --real --obj-x 0.30 --obj-z 0.05`
- [ ] 觀察 dist 前 30 步是否開始下降（不再被 diverging 打斷）
- [ ] 若 z=0.05 收斂，繼續測 z=0.02
- [ ] 若 policy 在 x=0.30 距離下完全無法收斂（> 100 步 dist 不降），考慮回到 x=0.25 測 z 下探
- [x] 更新 CLAUDE.md 的 Sim-to-Real 關節映射表（反映 API 鏡像 + invert 全 False）

### 目前狀態
方向修正完成（invert 全 False），正前方 + 左右三個位置（x=0.25, z=0.15）成功夾取。obj-z 下探在 z≥0.10 可行，z≤0.05 因 diverging trigger 提早觸發而失敗。已修改觸發邏輯，待傳檔重測。

---

## 2026-06-21

### 整合 YOLOv11 垃圾辨識 → 夾取（含寬度控夾爪）+ 專案重組

把 Roboflow YOLOv11 垃圾辨識（手臂相機，類別 `bottle-cap`/`paper-ball`）接到既有 PPO 夾取部署。

**專案重組為三資料夾**（原根目錄散檔整理）：
- `grasp/`（夾取）— 原部署檔**搬移**進來；URDF/模型路徑相對 `__file__`，整組搬移後免改、已驗證可讀到。
- `detection/`（辨識）— 從 Downloads 來源**複製**，分 `models/`、`calibration/`、`debug_tools/`、`rear_nav/`。
  正式模型 = `detection/models/best.pt`（來源 train5）。未複製 `darknet/` 與影像資料集。
- `integration/`（整合）— 新寫橋接程式 + README。

**夾取端寬度控夾爪**（`grasp/x3plus_real_grasp.py`，向後相容、預設關閉）：
- 新增 CLI `--width-grip`；不加時行為與改寫前完全相同（Stage 1 一律全閉 180°）。
- `DetectionReceiver` 多解析 `"w"`（公尺）、新增 `get_width()`（含 stale 判斷）。
- 新增 `JointMapper.width_to_close_deg()`：`close = 180 − (w/max_w)×(180−min_close)`，
  窄物趨近 180°、寬物提早停。預設 `grip_max_object_width_m=0.06`、`grip_min_close_deg=120`。
- Stage 0→1 依寬度設 `_stage1_close_deg` 與步數；Stage 1/2 全程持握該角度。

**整合橋接**（`integration/vision_grasp_bridge.py`，新寫）：
- 沿用 `arm_cam.py` 幾何（H=0.33, FIXED_THETA=41.5, FX/FY=650）：
  `arm_dist=estimate_distance(y2)`、`offset_x=arm_dist*(cx−CX)/FX`、`width_m=box_w_px*arm_dist/FX`。
- 映射 `obj_x=arm_dist+cam_x`、`obj_y=sign_y*offset_x+cam_y`、`obj_z` 固定，
  透過 TCP 5555 每筆開新連線送 `{x,y,z,w}`（送完即關，符合 `DetectionReceiver` 讀到 EOF 才解析）。
- CLI：`--host/--port/--stream/--conf/--rate/--once/--show/--cam-x/--cam-y/--obj-z/--sign-y`。

**路徑修正**：`detection/arm_cam.py`、`detection/debug_tools/detect_video.py` 的 `MODEL_PATH` 改相對路徑。
新增 `detection/requirements_detection.txt`。

### 驗證狀況
- 所有改動/新檔 `py_compile` 通過；幾何/寬度→夾爪角數值核算合理（dist 0.21–0.35m、width 1.3–8.6cm、close 120–167°、單調）。
- 本機（Windows dev）**未裝** ultralytics / stable_baselines3 / pybullet，**無法跑 runtime dry-run**，需到 Jetson 驗證。

### ⚠️ 最大整合風險：手臂相機會動，距離模型只在單一姿勢成立
- URDF 顯示相機（`mono_link`）以 `mono_joint` 固定在 **arm_link4**，即手臂相機**隨手臂移動**。
- `arm_cam.py` 的 `H=0.33`/`FIXED_THETA=41.5` 是**固定常數**，只在「校正當下的手臂姿勢」成立。
- 若橋接端在夾取過程中**持續送**偵測值，手臂一動距離估計就失真，且與 FK（base 座標）的 TCP 不同步
  → policy 觀測的 `rel_pos = obj − tcp` 會錯亂。
- **正解（待做）**：在 home/觀測姿勢**擷取一次並鎖定**物體座標，整個 episode 用固定目標。
  注意 `detection_stale_timeout_sec=1.0`：單筆鎖定 1 秒後會退回 default，需配套處理（見待辦）。

### 待辦（下次接續）
- [x] 夾取端加 `--latch-obj`：開場（move_to_home 後、step 前）`_latch_object()` 讀一次 obj_pos/width 並凍結，
      解決移動相機問題。等候 fresh 偵測逾時（`latch_wait_sec=5s`）會退回預設並印警告。
      新增 `DetectionReceiver.snapshot()`（原子回傳 pos/width/fresh）。py_compile 通過，待 Jetson 實測。
- [ ] scp 三資料夾到 Jetson；`pip install -r detection/requirements_detection.txt`。
- [ ] 夾取端 dry-run（不加 --real）確認載入與角度；確認不加 `--width-grip` 行為照舊。
- [ ] 校正：`FIXED_THETA`（calibrate_arm_camera_theta.py）、`--cam-x/--cam-y`、`--sign-y`、`--obj-z`。
- [ ] 校正寬度→夾爪：實測 `grip_max_object_width_m`、`grip_min_close_deg`。
- [ ] 端到端：bridge `--once` 核對座標 → 連續 → 完整夾取；觀察 Stage 1 閉合角隨寬度變化。

---

## 2026-06-21（續）— 自走夾取 Pipeline（偵測→導航→夾取→重試）

把視覺導航與 PPO 夾取串成自走流程（使用者 5 步）。決策：單一程式統一控制、雙相機、最多重試 3 次。

### 現況調查
- 既有導航程式 `detection/rear_nav/rear_to_arm_blind_handoff.py`、`detection/arm_center_setmotor.py`
  **已有「看到物體往前開」邏輯**，但 (a) 走 port 7000 motor server、(b) 到 READY_GRAB 就停、從不真的夾。
- `Rosmaster_Lib_reference.py` 確認同一 Rosmaster 物件同時有 `set_car_motion(vx,vy,vz)`（麥輪）與
  `set_uart_servo_angle_array`（手臂）→ 一個 process 可同時控車+手臂，無序列埠衝突。

### 微改 `grasp/x3plus_real_grasp.py`（向後相容）
- `GraspController` 加 `obj_provider` 參數：設定後 `_get_obj_pos/_get_obj_width` 直接回它（統一模式不需 socket/latch）。
- `run()` 回傳 `_grasp_succeeded`。
- `ServoController` 加唯讀 `device` property（共用 Rosmaster 給底盤）。
- 預設 `obj_provider=None` → `--socket`/`--obj-x` 行為不變。py_compile 通過。

### 新增 `integration/vision_grasp_pipeline.py`（★主程式）
- **Navigator**：雙相機（後遠+手臂近）YOLO + 移植 handoff 的距離/偏移/決策函式，
  actuator 改 `action_to_vxyz()`→`set_car_motion`（KX/KZ 換算）。狀態機
  `REAR_FOLLOW → REAR_BLIND → ARM_ALIGN → (handoff)`。
  **取消 ARM_BLIND_PUSH**（原 3s 會衝過 0.24m 物體），改在物體置中且 dist≤0.24m 時交棒給手臂 policy。
  `latch_arm_object()` 算 {x,y,z,width} 映射夾取座標；`verify_grasp()` 重新偵測，原位置仍在→判失敗。
  含 `back_off()` 重試前小退。
- **Orchestrator**：建 `GraspController(real_servo)` 取得共用 device → Navigator；
  迴圈 approach→latch→`obj_provider`→`run()`→verify，失敗張爪退回重試，最多 3 次。
- CLI：`--real/--show/--max-retries/--max-steps/--port/--handoff-dist/--selftest`。
- `--selftest`（純函式、免 cv2/torch）在開發機通過：action_to_vxyz、rear/arm 決策、latch、blind plan 數值合理。

### 驗證狀況
- 兩檔 `py_compile` 通過；pipeline `--selftest` 數值核對正確。
- 開發機無 cv2/ultralytics/torch → 完整 dry-run/實機需在 Jetson。

### ⚠️ 風險 / 待辦
- [ ] **執行前確認 Jetson 沒跑 port 7000 motor server / ROS 底盤 driver**（搶序列埠）。
- [ ] actuator 換成 set_car_motion，導航速度/轉向門檻可能要重新微調；**確認 v_z 正負＝左右轉向**。
- [ ] 校正：`THETA_ARM/H_ARM`（與 arm_cam.py 的 41.5/0.33 不一致，需確認現行值）、`CAM_TO_BASE_X/Y`、`SIGN_Y`、`OBJ_Z_FIXED`、`KX/KZ`、handoff 距離。
- [ ] 觀測姿勢：導航/verify 時手臂須停在 arm-cam 校正姿勢（預設＝夾取 home，需確認）。
- [ ] Jetson：先單測 Navigator（只導航不夾）→ 全流程；確認重試與 3 次後停。
- [ ] 效能：Nano 同時兩路串流 + YOLO 的 FPS。

---

## 2026-06-28

### 完成事項
- 新增根目錄 `INDEX.md`，整理主程式入口、資料流、夾取/視覺座標關鍵程式、校正指令與快速搜尋關鍵字，方便後續快速定位程式碼。
- `integration/vision_grasp_bridge.py` 與 `integration/vision_grasp_pipeline.py` 新增 `--class-z NAME=HEIGHT_M`，可用 YOLO 類別指定物體高度 z；送出的資料也會包含 `class`。
- 確認手臂相機輸出物體資料路徑為 `{x, y, z, w}`，其中 `w` 可進入 `DetectionReceiver`，並由 `--width-grip` 轉成夾爪閉合角度。
- 修正 `grasp/x3plus_real_grasp.py` 的 stale detection 行為：socket 資料過期時不再沿用最後一次偵測值鎖定物體。
- 將巡航/靠近物體用的 `home_deg` 與 PPO 夾取初始姿勢 `grasp_home_deg` 分開；預設巡航 home 維持原本姿勢 `(90, 140, 0, 0, 90, 30)`。
- 新增 CLI：`--nav-home-deg`、`--grasp-home-deg`。完整 pipeline 會先回巡航 home 導航/偵測；standalone grasp 會先在巡航 home latch 物體，再切到 PPO grasp home 開始策略。
- 新增 `integration/verify_camera_grasp_frame.py`，不用 PyBullet/cv2/ultralytics 也能靜態檢查 URDF base frame、`mono_link` 位置與手臂相機方向。

### 座標與偵測結論
- PPO 的物體座標基準是 URDF/base frame；觀測使用 `rel_pos = obj_pos - tcp_pos`，所以手臂相機輸出的 x/y/z 必須先轉到同一個 base frame。
- 目前靜態檢查顯示，在預設 home 下 `mono_link` 約在 base `(x=0.1584, y=-0.0022, z=0.2631)`；若假設 `mono_link +Z` 是 optical axis，方向約朝 base +X 並下看 40 度。
- 現有 bridge/pipeline 視覺常數不一致：bridge 使用 `H=0.33, theta=41.5, FX/FY=650`；pipeline 使用 `H_ARM=0.35, THETA_ARM=27.3, FX/FY≈957`，且 `CAM_TO_BASE_X/Y=0`。因此座標方向大致合理，但仍需要實機校正 offset/scale。
- 若要在 `home_deg` 與 `grasp_home_deg` 各偵測一次 XY，可以做，但不能直接平均 raw camera XY；必須先各自轉成同一個 base frame，再用 median/一致性檢查融合。PPO episode 開始後建議凍結 target，避免手臂移動時座標基準漂移。

### 驗證狀況
- `python -m py_compile integration\vision_grasp_bridge.py integration\vision_grasp_pipeline.py grasp\x3plus_real_grasp.py` 通過。
- `python integration\vision_grasp_pipeline.py --selftest --class-z bottle-cap=0.012 --class-z paper-ball=0.035` 通過。
- `python integration\verify_x3plus_deploy.py --skip-selftest` 只有 Windows 開發機缺 `Rosmaster_Lib`、`ultralytics`、`cv2` 的預期警告。
- Windows 上執行 `grasp\x3plus_real_grasp.py` dry-run 仍因本機缺 `pybullet` 無法完整跑；需在 Jetson/已裝環境再驗證。

### 待辦
- [ ] 選定正式視覺路徑：`integration/vision_grasp_pipeline.py` 或 `integration/vision_grasp_bridge.py`，避免兩套相機常數互相混用。
- [ ] 在 Jetson 實測校正 `CAM_TO_BASE_X/Y`、`SIGN_Y`、`H/theta`、`FX/FY`，並用已知 base 座標物體驗證輸出的 x/y/z。
- [ ] 先用 `vision_grasp_bridge.py --once` 或 pipeline dry-run 檢查 `{x,y,z,w,class}` 合理，再開 `--real --socket --width-grip --latch-obj`。
- [ ] 若 `grasp_home_deg` 改成 PPO training home 後與巡航 home 不同，需要重新跑 `verify_camera_grasp_frame.py` 比較兩個姿勢的相機座標差異。

---

## 2026-07-05 / 07-06 — v17 部署驗證 + 接近段平順度（PR #1）

### 完成事項
- 本機 main 同步到遠端 v17（350ab8f）；原未提交修改保留在 `backup-local-20260705` 分支。
- Jetson 環境更新：目前 IP `172.31.28.252`；部署夾 `~/Documents/deploy_jetson`（非 git → scp 整包覆蓋）；
  `Rosmaster_Lib` 從 `~/Documents/x3plus_pipeline_deploy/grasp/` 重新本地化。
- `docs/operations/JETSON_DRYRUN_CHECKLIST.md` **Phase 0–4 全數通過**：
  - v17 模型/VecNormalize 載入正確；`verify_x3plus_deploy.py` 通過（port 7000 無佔用）。
  - `verify_camera_grasp_frame.py`：nav home 相機朝 +X 前下方（俯角 40°）健康；grasp home 相機下俯 75° 屬預期。
  - dry-run + `--real` **三階段狀態機跑完**：grasp home `[90, 32.704, 9.786, 32.704, 90, 30]` →
    Stage 0 兩步觸發（dist 0.086→0.043）→ Stage 1 鎖定 + S6 30→180 → Stage 2 回 home 保持閉合。
    ⚠️ **更正（2026-07-26）：這一輪並沒有真的夾到物體。原文寫「全流程成功」是誤讀。**
    `[End] Grasp succeeded` 只要 Stage 1→2 走完就會印，**不驗證任何實體結果**——它是狀態機旗標，不是夾取成功。
    低物體（瓶蓋，obj-z 0.02）在 v17 姿態下從未真正夾起。可信的成功紀錄只有 z=0.15 那批（見上方 2026-05-25）。
  - → 舊「Z 反向發散」問題（TROUBLESHOOTING #7）由 v17 正式解決。

### 問題：閉合前手臂「突然前伸」（TROUBLESHOOTING #9）
- 根因：Stage 0 的 10Hz 指令流從靜止中 0.2s 送兩段 3° 突衝（前伸姿態 3°≈3.5cm TCP，共 ~7.5cm），停-衝-停。
- 修法三層（branch `worktree-settle-before-close`，**PR #1**）：
  1. `stage1_settle_run_time_ms=600` + 0.5s：鎖定後先到位再閉合。
  2. `stage0_run_time_ms=250`（> 控制週期）：伺服運動中重規劃、消除停-衝-停。
  3. `--smooth-approach`：虛擬 rollout 先算出鎖定姿勢 → 單一規劃軌跡（~15°/s）滑過去。
     依據：obs 建自指令端狀態 + deterministic policy + 物體整輪固定 → 迴圈對硬體開環，指令序列可預先計算，幾何逐步等價。
- 已否決：移除 `max_delta_deg`（policy 原始目標單步差 50–72°，直送＝暴衝）。

### 結論與解釋（TROUBLESHOOTING #10–#12）
- 「先降 Z 再水平前伸」= v17 跨指式設計（開爪從物體兩側掠過），初始高度即抓取高度，抬高＝OOD。
- Z 基準：URDF root=`base_footprint` → **z=0=地面**；grasp home TCP z=0.010 = 模型有意貼地滑行。
- YOLO XY 與手臂 XY **基準不自動相同**，需 `cam_x/cam_y/sign_y` 校正（地面投影標記法）。

### 待辦（下次實測）
- [ ] `--real --smooth-approach` 驗證前伸是否變成單段平滑滑行（`[Smooth] dist@lock` 應 ≈0.008m）。
- [ ] 量：S6=30° 開爪內側間隙、木塊寬 → 單側裕度 ≥2cm 才安全（跨指式撞物風險）。
- [ ] 尺量 grasp home 抓取中心實際離地高 vs FK z=0.010 → 得 z_offset，校正 `--obj-z`。
- [ ] Phase 5 真物體實夾（木塊 x=0.25），回報：夾到／撞倒／夾空。
- [ ] 視覺 XY 基準校正：地面投影標記法 + `vision_grasp_bridge.py --once`（起始常數用 nav home 實算 cam_x≈0.158/H≈0.263/theta≈40°）。

---

## 2026-07-06 — PR #1 平順化架構複審 + code review + 併回 main

### 三層解法是否冗餘？結論：全留（是 fallback 鏈，不是三個重複解）
- `--smooth-approach` 是 opt-in 且**只在靜態物體源成立**；rollout 沒觸發 Stage 1 或物體源動態時退回串流路徑。
- 串流路徑的平順度靠 `stage0_run_time_ms=250`（層2）；閉合前到位保證靠 settle（層1，兩條路徑共用，
  smooth 路徑下兼作滑行寫入失敗的重送保險）。刪任一層＝fallback 退回會前衝的舊行為。

### Code review（xhigh 多 agent + 人工逐行鏡像比對）
- `_virtual_rollout_to_lock` 與真實 Stage-0 迴圈鏡像一致性逐行核過：obs 順序、觸發條件
  （min_dist 先更新、觸發步仍送 action、下輪鎖定）、rate-limit 數學、prev_action 時序全對；
  `_send_direct` 有更新 `_last_deg`，滑行後狀態接管正確。
- 25 個候選 → 4 個屬實已修，餘駁回（端點滑行＝設計語意；settle 重複＝有意且是重送保險；FK×2＝鏡像真實迴圈）：
  1. 滑行速度地板：`smooth_approach_max_ms` 截斷時長會讓大角度跳躍超速 → 加「不快於串流上限
     （`max_delta_deg×control_hz`＝30°/s）」的時長地板；`max_jump` 改含 S6。
  2. `stage1_settle_wait_sec` 0.5→0.7（原本 < 600ms settle 時長，閉爪指令會早 100ms 送出）。
  3. `--smooth-approach` + `--socket` 無 `--latch-obj` 時印警告（單次快照可能拿到 stale/default 座標）。
  4. CLAUDE.md 根目錄檔案清單補列 `TROUBLESHOOTING.md` 等新檔。
- 清理級待辦（不擋合併）：rollout 鏡像程式碼與 Stage 0→1 轉移體可抽共用 helper。

### 併回 main
- 分支 `worktree-settle-before-close`（PR #1）連同本次修正合併回本機 main，主目錄同步至最新。

---

## 2026-07-06（續）— PR #1 正式 merge + RL 導航整合（模式C）

### PR #1 收尾
- `git push origin main` 完成，GitHub 自動標記 PR #1 **MERGED**。

### 新增：RL 導航→夾取整合（branch `nav-rl-integration`）
另一台電腦訓練的 PPO 導航策略（iGibson Rs_int、48 束 LiDAR、hard-fail 成功率 0.600，
來源 `igibson_x3_test` 分支 `nav-best-weights-export`）接上夾取流程：

- **權重進 repo**：`integration/nav_best_model/`（best + checkpoint 281440 各 224KB，
  配對 vecnorm pkl；.gitignore 加白名單）。
- **`integration/nav_rl.py`**：55D obs 組裝、訓練 plant 逐項復刻（**2 步動作延遲、
  stall assist 0.28、近障調速 0.66m→0.24×/0.9×、油門映射 0.5/0.15/1.2**，參數=訓練
  JSON 值勿調）、LiDAR 點雲→48 束（右→左、[0.33,4.0] clip、車中心距離、支援安裝偏移）、
  VecNormalize 手動正規化、幾何煞停（原始點雲 ±50° <0.25m 禁前進——48 束 obs 下限
  0.33m 看不到更近障礙）、`--probe/--bench/--selftest`。
- **`integration/nav_rl_grasp_pipeline.py`**：GoalTracker（相機定位 + 指令速度
  dead-reckoning，策略避障轉彎時目標可暫離視野）→RL 導航→停車確認→沿用
  vision_grasp_pipeline 的 ARM_ALIGN/latch/夾取/verify/重試。`--no-lidar` 配 `--real`
  直接拒絕。
- **開發機驗證**：selftest 全過；**真權重虛擬 episode 4/4 到達**（含右急彎），
  全鏈路 VecNormalize→policy→延遲→plant→tracker；推論 4.7kHz；語意煙霧測試
  （目標偏左→左轉、右障→倒退左轉）正確。SB3 2.9 可載入（lr_schedule 警告無害）。
- 插曲：pip 裝 SB3 時把 torch 升成 CPU 版，已還原 `2.6.0+cu124`。

### 待辦（Jetson 上機，見 `integration/NAV_RL.md` 校正清單）
- [x] LiDAR 確認：YDLIDAR TG30／CP2102 ttyUSB0；ROS `/scan` 約 10.17Hz
- [x] ROS `/scan` adapter 完成；預設 `--lidar-backend ros`，不安裝 `rplidar-roboticia`
- [ ] Jetson 安裝 `roslibpy`、開 rosbridge；`nav_rl.py --probe` 實測前／左／右方向
- [ ] `nav_rl.py --bench` 確認 Jetson 推論速率 ≥6Hz、SB3 2.3.2 能載入權重
- [ ] dry-run 確認 `+wz`=左轉（反了 `--wz-sign -1`）；再 `--real` 首測（`--nav-stop-dist 1.0` 起步）
- [ ] 端到端：偵測→RL 避障導航→精對位→夾取→verify

---

## 2026-07-07 — 校正→整合路線圖 + 內參工具 + gate 切乾淨（PR #2 續）

### 新增：完整校正路線圖 `docs/calibration/CALIBRATION_PLAN.md`（8 個 Phase，含過關標準/失敗排查/常數對照表）
使用者要「一步一步」照做的計畫，順序＝地基相依：
內參是距離模型的地基、距離模型是座標對齊的地基、座標對齊沒過夾取一定夾空。
- Phase 0 環境同步 → 1 相機內參(重投影<0.5px) → 2 地面距離模型(θ/H，手臂±3cm) →
  3 YOLO↔手臂 base 座標(latch 差<2cm) → 4 定點視覺夾取(≥3/5) →
  5 RL 導航單測(--nav-only) → 6 全流程整合(≥2/3) → 7 收尾。
- 每個 Phase 設「沒過不准進下一步」的門檻；末尾附「常數更新對照表」（校到哪填到哪，
  pipeline/bridge/arm_cam 三處要同步）。

### 新增：棋盤格內參校正工具 `detection/calibration/calibrate_intrinsics.py`
repo 原本**沒有真的內參校正**——現有 fx≈957 是「先沿用近似值」的借用值，另一套 650 是猜測值，
兩者矛盾且 θ 校正僅 3 點、解出 24.9°~29.4° 飄移（內參不準的症狀）。
- 棋盤格 9×6 內角點，對串流 URL 或本地 device；自動擷取（角點移動足夠才拍）、逐 view 重投影誤差、
  `--check` 去畸變預覽、Python 3.8 相容。
- **內參 vs 外參結論**（回答使用者「換姿勢有影響嗎」）：內參(fx/fy/cx/cy/畸變)是鏡頭光學性質、
  **完全不受姿勢影響**，Phase 1 校一次永久有效；外參(H/θ)裝在 arm_link4 隨手臂動、
  **換姿勢即作廢**，Phase 2 必須在現用 nav home 姿勢下重量。

### Review 修正（5 點，把 phase gate 切乾淨）
1. **`--nav-only` 改純 RL 導航驗證**：`RLNavigator` 加 `rl_only`，`approach()` 在 `_rl_navigate()`
   成功後直接返回、跳過 `_arm_align()`。原本會先到 1.0m 再用手臂相機 fine-align 到 0.24m，
   把導航和精對位混在一起。
2. **畸變決策 gate**：內參工具印出 bbox 底邊中點的去畸變位移（距離模型實際吃的像素），
   Phase 1 加明確決策：<2px 記「忽略」、≥2px 偵測前 `cv2.undistort` 或對 bbox 點 `undistortPoints`。
3. **Phase 4 改 bridge 模式**：`x3plus_real_grasp.py --socket --latch-obj --smooth-approach`
   + `vision_grasp_bridge.py --once`，只測「相機座標+夾取」，不混進導航/精對位。
4. **`controller.run()` 回傳值**：兩個 pipeline（nav_rl + vision_grasp）改
   `grasp_seq_ok = controller.run(...)`；三段式沒完成直接判失敗，不再只靠視覺 verify。
5. **對照表補 `lidar_forward_offset_m`**：Phase 5 --probe 時尺量 LiDAR 中心↔車中心，
   否則 48 束 ray 從錯的 robot center 出發。

### 順帶
- pipeline `--nav-only` 旗標（Phase 5 用）、內參工具的 check 迴圈空轉/`--out` 目錄/主點偏移警告（Gemini review）一併修。
- 驗證：三檔 py_compile 通過；nav_rl / nav_rl_grasp_pipeline / vision_grasp_pipeline selftest 全過。
- commits：`f4be79e`（路線圖+工具+--nav-only）、`1f5c3d9`（5 點 review 修正），均在 PR #2。

---

## 2026-07-07（續）— Phase 0 完成（Jetson 端）

### 使用者回報：Phase 0 環境準備完成
- **部署夾改為 `~/Documents/deploy_jetson2`**（舊 `deploy_jetson` 的 Rosmaster_Lib 已複製到
  `deploy_jetson2/grasp/`）；記憶 [[jetson-network]] 已更新路徑。
- `calibrate_intrinsics.py` + `docs/calibration/CALIBRATION_PLAN.md` 已在 Jetson；venv 正常、OpenCV 4.13.0。
- **相機身分確認**：`/dev/video0`=手臂相機、`/dev/video1`=後鏡頭（與既有記憶一致）。

### 計畫微調（配合實況）
- Phase 1 步驟改用**本地裝置 `--source 0/1`**（Jetson 上跑，免串流、畫質最好），路徑更新為 deploy_jetson2。
- 明確標注**內參與姿勢無關**：固定姿勢只為校正時相機不飄，fx/fy/cx/cy 對所有姿勢通用
  （呼應上一則的內參 vs 外參結論）。

### 下一步：Phase 1 相機內參
- 待辦：印 9×6 內角點棋盤格（方格 25mm，尺量實際邊長）→ `--source 0` 校手臂相機、`--source 1` 校後鏡頭
  → 看 RMS<0.5px、fx≈fy、畸變位移決策 → 填對照表①、定案 957 vs 650。

---

## 2026-07-26 / 07-27 — C3 上機實測 → 找到夾取從未成功的根因（TCP 定義不一致）

### 這一輪最重要的結論

**訓練端與部署端在 28D 觀測裡取的 TCP 不是同一個點，相差約 8cm。**
這使得 v17 與 v18 在實機上**從來沒有真正夾到過地面上的物體**，與物體高度、
與觸發門檻都無關。詳細規格與移交事項見 `docs/planning/GRASP_TRAINING_REQUIREMENTS_2026-07-27.md`。

| | 觀測裡的 `tcp_pos` | C3 home 離地 |
|---|---|---|
| 訓練 `training/x3plus_ground_grasp_env.py:1144` | `_get_gripper_center()`＝`rlink_joint2`+`llink_joint2` 連桿質心中點（**指尖**） | **14.54 cm** |
| 部署 `grasp/x3plus_real_grasp.py` `FKComputer.compute()` | `getLinkState(ee_link)[0]`＝**`arm_link5` 質心** | **7.86 cm** |

四元數 `tcp_orn` 兩邊一致（都用 `arm_link5` 姿態），**只有位置錯**。
policy 學的是「把指尖帶到物體」，實機卻餵它質心座標 → 質心到位時真正的夾爪
還在物體上方 **10～11.5 cm**，在半空中閉合。

三方獨立佐證 14.5cm：URDF FK 算 14.54、`manifest.json` 記 14.5、實機拿尺量 ~15。

### 舊紀錄更正（重要）

- **「v17 `--real` 全流程成功」是誤讀，v17 從未真正夾到瓶蓋。** 已更正本檔
  2026-07-05 節與 `TROUBLESHOOTING.md` #9。
- 誤讀來源：`[End] Grasp succeeded` **只要狀態機走完 Stage 1→2 就會印，不驗證任何實體結果**。
  它是狀態機旗標，不是夾取成功。日後判讀 log 務必以實體觀察為準。
- 反證：本檔 2026-05-25 「obj-z=0.15 三點成功、z≥0.10 可行、z≤0.05 失敗」的紀錄是**可信的**
  ——15cm 正好是夾爪實際所在高度。當時判為 diverging trigger 提早觸發，**真正原因是夾爪沒下去**。

### Gate 1 / Gate 3 完成（C3 實機量測）

手臂開到 C3（API `90,67.08,9.79,9.79,90,30`），側面輪廓確認正確（大臂立起、前臂抬平、
手腕垂直下垂），App 顯示 S2≈112.9 符合鏡像慣例。

```
G（夾爪正下方地面投影）= base (0.205, +0.015)
   量法：前輪軸 x=+0.08 往前 12.5cm；URDF FK 預測 (0.1993, +0.0182)，誤差 < 6mm
C3 實測 FOV（相對 G）：前後 −4.5 ~ +7.5 cm、左右 −6.5(左) ~ +10.5(右) cm
換算 base：x 0.160~0.280、y −0.090~+0.080
實際可部署區 = FOV ∩ 訓練區 = x 0.200~0.280、y −0.090~+0.080
```

- **manifest 宣稱涵蓋可抓帶 84%，實測 62%**（深度 8cm / 13cm）。
- **README 預測「C3 視窗在夾取中心前 +9~+22cm」是錯的**，實測 −4.5~+7.5cm。
  那個前移量是從 v17 外推來的：v17 相機離地 14.6cm、光軸離垂直 14.8°，角度淺、
  投影對俯角誤差極敏感；C3 離地 22.7cm、離垂直僅 3.3°，視窗就落在相機正下方。
  **不要把 v17 的偏移外推到其他姿態。**
- reach guard `0.15m` 雖是為 v17 量的，但涵蓋整個可部署區（最遠角落 0.147m），**不需修改**。
- 橫向驗算：相機比夾爪偏右 2.0cm（URDF），實測視窗中心偏右 2.0cm，**吻合到 0.02cm**。

### Stage-1 觸發門檻 0.05 → 0.042（有幫助，但不是根因）

第一次實夾失敗後發現：Stage 0→1 只看**純量距離**，C3 的接近路徑是斜向下來的，
會在夾爪還太高時就滿足 0.05。實機鎖定在 TCP 高於物體 5.4cm（收斂值應為 4.0cm）。

- 掃過可部署區（x 0.21/0.24/0.27 × y −0.08/0/+0.018/+0.07），0.042 每點都會觸發，
  鎖定距離從 ~0.044 收緊到 ~0.038。
- 新增 `--stage0-dist-threshold` 參數（避免 scp 覆蓋後失效）；帶 `0.05` 可完全重現舊行為。
- **但只讓指尖降了 0.7cm**，對 10cm 的落差幾乎無用。真正的問題是上面那個 TCP。

### 機構可行性：不是限制（已排除）

URDF 掃 12 萬組關節組合，**夾爪中心（訓練定義）**在部署區可達的最低高度：

| 夾爪中心 x | 最低離地 |
|---|---:|
| 0.20–0.24 | **−1.1 cm** |
| 0.24–0.28 | **−0.3 cm** |
| 0.28–0.33 | +1.3 cm |

手臂搆得到瓶蓋高度，綽綽有餘。訓練環境註解「太矮/太扁的都夾不起來，故剔除」
**很可能是被同一個 TCP 錯誤誤導**，修好後應重新評估。

### 訓練端設定與實機的落差（已寫入移交文件）

- 物體池只有 `wood_block`(3.5×3.5×**6.6**cm) 與 `cracker_box`，**沒有低於 6.6cm 的物體**；
  實際目標瓶蓋只有 1.3cm，完全在訓練分布外。
- `train_spawn_x_range = (0.20, 0.33)`，但實機在 C3 只看得到 **x ≤ 0.280**，
  **38% 的 spawn 範圍在實機上看不見**。
- 成功判準是**物理判準**（`is_grasping` + `lift_height > 0.10m`），模擬端的 97/100 可信；
  失敗純粹發生在 sim→real 的觀測轉換。

### 週邊排除

- 手臂相機一度完全消失（`startup_device_check.py` 只找到 1 顆）。**是 USB 接頭鬆脫**，
  重插即恢復為 `0c45:6340` → `/dev/video1`。
- 期間誤判 `2bc5:060f` 為手臂相機：那其實是 **Orbbec 深度感測器**，vendor-specific 介面
  沒有 `/dev/video` 節點是正常行為，不是故障。車上那顆 Orbbec 目前只用到 RGB 串流。
- 手臂相機線隨手臂活動，**移動後需重新確認 `/dev/video1` 仍在**。

### 未解決 / 下一步

1. **部署端能否複現訓練的 `_get_gripper_center()`** — 訓練端手指關節由物理模擬驅動，
   部署端 FK 只設 `arm_joint1-5` + `grip_joint`，其餘留 0。照抄公式算出 y=−2.8cm 明顯
   不對稱，**尚未釐清**。已在移交文件列為第一優先待確認事項。
2. **實機橫向偏差** — 使用者觀察閉爪時夾爪偏左；但在現行 TCP 下模型算出的指尖水平誤差
   很小（dx −0.5cm、dy +0.2cm），與觀察不符。修好 TCP 後需重新觀察。
3. **`object_z` 慣例** — 仍是 UNRESOLVED。應統一為「物體幾何中心的真實高度」並寫進 manifest。
4. **PR #13（draft）暫不可 merge** — 它啟用 v18/C3，但在 TCP 修好前 v18 一樣夾不到，
   merge 會給人「可以上機」的錯誤印象。

## 2026-08-30 — v23 E1 夾取目標前移補償

- 實機已能穩定夾取，但落點系統性偏物體近側；沒有改寫 7 點實測
  `integration/grasp_home_homography_e1.json`，而是在 homography／S1 視角旋轉完成後、
  policy envelope 檢查前，加可稽核的 base `+X` runtime correction。
- `grasp/v23/jetson_one_command_grasp.py` 預設 `--grasp-forward-offset-mm 5`；可用 `0`
  完全停用，硬限制 `0–15 mm`。修正同時作用於 bbox 中心與兩個寬度端點，因此不改變
  估計寬度；超出 v23 `x=0.205–0.280 m` 的修正後目標仍 fail closed。
- 單 E1 bridge 與 LEFT／E1／RIGHT scanner 使用同一修正；scan 在旋轉回最終 base frame
  後才沿 `+X` 平移。校正模式不帶補償，保證量測資料仍是原始幾何。
- 第二次實抓 log 證明前述 +5 mm 已套用：bridge 送 `x=0.2728`，故 raw homography
  `x=0.2678`，只比「前 4 cm」尺量 `x≈0.2687` 少 0.9 mm。繼續把物體座標加 1 cm
  會成為 `x=0.2828`，越過 v23 `0.280` envelope；因此新增 controller-side
  `--tcp-forward-error-mm 10`。它只把 policy／close gate 使用的 FK TCP X 減 10 mm，
  讓手臂實際再往 base +X 走 1 cm；camera/object 座標與 floor/collision FK 均不變。
- 後續實抓仍落在機器人側，因此以 5 mm 小步幅把 launcher 的預設
  `--tcp-forward-error-mm` 從 10 提高到 15；視覺座標仍維持 +5 mm，合計約 +20 mm，
  controller 硬上限仍為 20 mm。
- 同日成功夾到後仍聽見喀喀聲。完整 log 證明 S6 在 Stage 0 從 117° 被 policy 推到
  179°；147–152° 的物體滑動／回彈讓舊的「完全不動」計數反覆歸零。當時雖帶
  `--jaw-track-fraction 0.5`，該比例判定其實只接到 scripted Stage 1，沒有保護 Stage 0。
- Stage 0 現在也比較上一筆 S6 命令的未完成角度與下一筆 encoder 進度；連續兩筆低於
  0.5 時，以 `slipping` 接觸鎖定接觸角 +1°，直接進 scripted lift。正常追蹤、單筆慢速、
  ineligible 動作與 `fraction=0` 均不觸發；controller 回歸新增 6 項，合計 148 全過。
- 實機修正後操作者在 E1 可辨識範圍內重複 **3/3 成功**。已核對的一份完整 run：
  S6 在 152° 以 2/8° slow-tracking 連續兩筆觸發、hold 153°；Stage 2 初檢 confirmed、
  lift 6/6、return home 成功、回家後仍 confirmed（S6 150°），bus 無失敗、guard 45 次 pass。
- 將該實測組合升為 `jetson_one_command_grasp.py` 正式預設：target +5 mm、
  `floor-finger-error=15 mm`、`jaw-track-fraction=0.5`、`tcp-forward-error=20 mm`。
  `manifest.hardware_gates.first_real_grasp_logged` 改為 true；仍保留 candidate，因 motion
  envelope、dry-run 與 LEFT/RIGHT scan 硬體 gate 尚未完成。
## 2026-09-07 — v24 E1 replacement 候選封裝

- 接收訓練端 `v24_e1_corner_recovery_seed23404_r3` 的 selected BC update 10；完整交付
  55 個 checksum 全部吻合，formal rows 重算 230/235，舊 v23 弱點 `(0.280,-0.070)`
  為 15/15。新 model/VecNormalize hash 分別以 `0ed998...d81`、`6ebb5a...d26` 鎖定。
- 新增 `grasp/v24/run_candidate.py`、`manifest.json`、必要 provenance 與 9 項靜態測試。
  wrapper 重用 `grasp/v23/x3plus_real_grasp.py`，拒絕使用者覆寫 model、VecNormalize、
  contract、release manifest 或既有 E1 3/3 runtime profile，避免產生另一份會漂移的
  motor/servo/safety runtime，也避免驗收時同時改權重與控制參數。
- v23 release gate 現可由版本 wrapper 指定所屬 manifest；hash 與 incremental contract
  仍不可由 `--unlock-candidate-real` 繞過。v24 的新 hash 硬體 gate 全部起始為 false。
- 尚未執行模型反序列化、模擬重播或實機動作。97/235 正式模擬紀錄低於部署 8mm
  clearance，且 evaluator 含 magnet/scripted return；需完成 deployment parity 與新 hash
  Jetson／固定座標／E1 視覺 3/3 後才可考慮切換。完整限制見
  `docs/planning/v24_intake_2026-09-07/INTAKE_REVIEW.md`。
