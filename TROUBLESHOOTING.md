# X3Plus 問題與解決紀錄（Troubleshooting Log）

> 目的：把實測中踩過的坑與解法記下來，避免重複犯錯。新問題請往下追加，格式：**症狀 → 原因 → 解法 → 教訓**。

---

## 環境 / 部署

### 1. Jetson 空間不足，誤以為是安裝 ultralytics 造成
- **症狀**：Jetson Nano 只剩 ~762MB，懷疑裝辨識套件吃太兇。
- **原因**：最肥的 `torch`（2.4.1, ~1GB+）**之前就已安裝**且 grasp 接收端必須用；本次裝 ultralytics/torchvision/opencv/polars 其實只 ~90MB，不是主因。
- **解法**：先 `pip cache purge` + `du -h -d1 ~/` 找真正佔空間者，不要急著刪辨識套件（只回收 90MB，治標不治本）。
- **教訓**：看 `pip` log 的 `Requirement already satisfied` 判斷誰是既有、誰是新增，再下結論。

### 2. 辨識端該裝在哪台機器
- **原因**：Jetson 跑 YOLO 慢、空間緊。模式 B（TCP 橋接）本來就解耦。
- **解法**：bridge（ultralytics+torch）裝在**電腦端**，`--host`/`--stream` 指向 Jetson IP；Jetson 只跑接收端（不需 ultralytics/opencv）＋相機串流。
- **教訓**：接收端 `DetectionReceiver` 只依賴 numpy/pybullet/SB3/gymnasium，先確認 import 再決定哪端裝什麼。

### 3. 電腦端裝 CUDA 版 torch
- **解法**：RTX 3050（驅動支援 CUDA 12.7）→ `pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124`。先 `nvidia-smi` 看驅動支援的 CUDA 版本，選 ≤ 它的 cuXXX build（cu124 最穩）。驗證 `torch.cuda.is_available()` 與 `r[0].boxes.data.device == cuda:0`。
- **教訓**：cu124 channel 會把 torch 降到對應版本（2.12→2.6），ultralytics 仍相容，正常。

### 4. Jetson 執行時出現 `SyntaxError: future feature annotations is not defined`
- **症狀**：在 Jetson 上直接執行 `python3 jetson_one_command_grasp.py` 時報錯 `SyntaxError: future feature annotations is not defined` 或缺少相依套件。
- **原因**：Jetson Nano (JetPack 4.x / Ubuntu 18.04) 系統預設 `python3` 版本為 **Python 3.6.9**。本專案完整的運作環境（Python 3.8、PyTorch、Stable-Baselines3、PyBullet 等）皆建置於 **`~/grasp_venv`** 虛擬環境中。
- **解法**：執行前必須先進入虛擬環境：
  ```bash
  source ~/grasp_venv/bin/activate && cd ~/Documents/iGibson-Navigation-Grasping/grasp/v21 && python3 jetson_one_command_grasp.py --check
  ```
  `jetson_verify.sh` 第 20 行亦有檢查 `VIRTUAL_ENV` 環境變數。
- **教訓**：Jetson 上本專案一律用 `~/grasp_venv` 的 Python 3.8 執行；系統預設 `python3` (3.6.9) 未安裝專案套件且不支援 PEP 563，跑任何腳本均無法運作。

---

## 相機 / 串流

### 4. bridge 連不到相機串流 8080（Error -138）
- **症狀**：`Connection to tcp://<目前Jetson位址>:8080 failed: Error number -138`、`cannot open camera stream`。
- **原因**：Jetson 上**沒有任何 8080 串流伺服器在跑**。`ping` 通但 `Test-NetConnection -Port 8080` = False。進一步發現 ROS 根本沒起（`rostopic list`→Unable to communicate with master）。
- **解法**：不依賴 ROS web_video_server，改用免 ROS 的 `stream_cam.py`（stdlib http.server + cv2 推 MJPEG 到 :8080）。
- **教訓**：相機串流失敗先分三層查：①`ping` 通不通 ②`Test-NetConnection <port>` 開不開 ③Jetson 上服務有沒有真的在跑。

### 5. `/dev/video0`、`/dev/video1` 都開不起來
- **症狀**：`can't open camera by index` / `backend ... can't be used to capture by index`。
- **原因**：相機被出廠服務 `python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py`（pid 由 `sudo fuser /dev/video0 /dev/video1` 查出）佔用。它由 `bash -c "python3 ...; exec bash"` 啟動，**非 systemd**，`kill <pid>` 後不會自動重啟。
- **解法**：`sudo fuser /dev/video*` 找 pid → `kill` → 再用相機。此服務很可能也佔 Rosmaster 序列埠，跑 `--real` 夾取前本來就該停掉（一石二鳥）。
- **教訓**：Linux 相機「開不起來」先用 `fuser`/`v4l2-ctl --list-devices` 確認是「被佔用」還是「裝置不對」，不要瞎試 index。

### 6. 相機裝置對應記錯（重要）
- **事實**：**`/dev/video0` = 手臂相機（YUYV-only）**、**`/dev/video1` = 機器人本體相機**（曾誤以為 video1 是手臂）。
- **教訓**：辨識/夾取用的手臂相機是 **device 0**；`stream_cam.py --device 0`。

---

## Claude 作業注意（給未來的我）
- 背景 session 改 repo 內檔案會被 bgIsolation guard 擋（Edit/Write）。本機工作目錄是扁平結構，但 git 遠端 default 分支是巢狀（`x3plus/`…），`EnterWorktree`(fresh) 會長錯結構。若使用者要求直接寫入其工作目錄的紀錄檔，改用 Bash `cat >>` 寫入。
- Jetson 部署根目錄：**`~/Documents/deploy_jetson2/`**，非 git repo，用 scp 從 Windows 整包覆蓋（`scp -r .\* jetson@<IP>:~/Documents/deploy_jetson2/`，傳「內容」避免巢狀）。`~/Documents/deploy_jetson/` 與 `~/Documents/x3plus_pipeline_deploy/` 均為舊部署夾。
- Jetson IP 由 DHCP 配發、可能每次開機改變。Windows 開工時只需執行
  `.\set_jetson_host.ps1 <目前IP或主機名>`；所有跨機工具讀取 `X3PLUS_JETSON_HOST`。
  直接在 Jetson 執行的主 pipeline 未設定時使用 `127.0.0.1`，不受 DHCP 影響；舊 Windows
  工具才保留 `172.31.28.252` 作相容備援。歷史位址：`10.201.123.252`、`192.168.0.201`。

---

## 夾取策略（PPO sim-to-real）

### 7. policy 部署時發散，手臂往上飛、Stage 卡在 0（✅ 已解 2026-07-05）
- **症狀**：dry-run 下 TCP z 從 home 0.21 衝到 0.573 並飽和，dist 從 ~0.2 漲到 0.55 卡死，Stage 永遠 0，跑滿 300 步沒夾取。
- **診斷線索**：①XY 大致趨近物體、**Z 方向完全相反**（物體 z=0.02 在下方，手臂卻往上）。②`min dist` 在第 1 步就出現 → 第一個 action 起即遠離。③換物體座標最終姿勢幾乎不變 → policy 幾乎不隨物體反應。
- **確認非以下原因**：傳輸（已驗證正確）、物體座標（latch 正確凍結）、stale（連續送無 stale）。
- **解法（2026-07-05）**：**v17 模型 + 對應的新 `grasp_home_deg=(90, 32.704, 9.786, 32.704, 90, 30)`**（repo commit bb51a61）。部署後照 `docs/operations/JETSON_DRYRUN_CHECKLIST.md` Phase 0–4 全數通過：Stage 0 兩步收斂觸發、S6 30→180 閉合、Stage 2 帶物回 home。
- **教訓**：sim-to-real「往固定飽和姿勢衝、不理會目標」是典型的 obs/action 慣例不一致或訓練起始姿勢不符；根治靠模型與部署姿勢成對更新，不要先動相機座標校正（那不是主因）。

### 8. DetectionReceiver 關閉時崩潰（✅ 已修 2026-07-16）
- **症狀**：主程式結束/Ctrl+C 後，`[DetectionReceiver] Parse error: [Errno 9] Bad file descriptor` 無限洗版，最後 `Fatal Python error: could not acquire lock for <stdout> ... daemon threads` → Aborted。
- **原因**：`close()` 關掉 server socket 後，daemon `_loop` 仍在 `accept()` 已關閉 fd，`except Exception` 抓到後 print 並 `continue` → 無限迴圈；直譯器關閉時 daemon thread 搶 stdout 鎖崩潰。
- **解法**：`close()` 先設停止旗標、關閉 server socket、等待接收 thread 結束；`_loop` 遇到關閉後的 `OSError` 會在旗標已設定時 `break`，且每個 client socket 都在 `finally` 關閉。

### 9. 閉合夾爪前手臂「突然向前伸」不平順（✅ 已解 2026-07-05/06，PR #1）
- **症狀**：v17 `--real` 三階段狀態機跑完（**但並未真的夾到物體**，見 progress.md 2026-07-26 更正），Stage 0→1 交界、夾爪即將閉合前，手臂猛地前伸一下。
- **原因（修正過兩次診斷才定案）**：**不是**伺服補償滯後（加 settle 後仍在）；是 **Stage 0 的 10Hz 指令流本身**——從靜止中 0.2 秒內送兩段 3° 突衝，而在前伸姿態下雅可比大、**3°/關節 ≈ TCP 3.5cm**，共 ~7.5cm；每段又是 100ms「衝到停」小軌跡（對比：3 秒 grasp home 大移動是單一條伺服規劃曲線所以平順）。
- **解法（三層，PR #1）**：
  1. `stage1_settle_run_time_ms=600` + 等 0.5s：鎖定後先平順到位再閉合（到位保證）。
  2. `stage0_run_time_ms=250`（> 100ms 控制週期）：伺服機**運動中重新規劃**，消停-衝-停。
  3. **`--smooth-approach`（最終解）**：利用「obs 來自指令端狀態＋deterministic policy＋物體整輪固定 → 迴圈對硬體開環」的特性，先**虛擬 rollout** 算出 Stage-1 鎖定姿勢，再用符合 Rosmaster 單段 2000ms 上限的**連續分段軌跡**（≤15°/s）滑過去。夾取幾何逐步等價（鎖定點同樣 dist≈0.008m）。
- **絕對不要做**：拿掉 `max_delta_deg`。policy 原始目標單步差 50–72°（S5 +72°、S6 接近途中半閉到 87°），直送＝暴衝；sim 是瞬移式關節設定，硬體永遠追不上，限制器是安全手段不是差異來源。
- **教訓**：「不平順」先分清是**指令軌跡本身**還是**執行滯後**——把兩者用一段強制靜止（settle+wait）隔開就能分辨。

### 10. 為何「先降 Z、再水平前伸」？低位接近會不會撞倒物體？
- **現象**：手臂先在 3 秒腳本移動中降到抓取高度（grasp home TCP z≈0.010），再由 policy 做近乎純水平的 ~7.5cm 前伸。
- **原因（設計而非 bug）**：grasp home = v17 **訓練起始姿勢**，固定且與物體無關，高度就設在抓取高度；policy 只學過「從這裡水平前伸」。接近方式是**跨指式（straddle）**：夾爪全開（S6=30°），兩指從物體**左右兩側**掠過、物體進入指間後才閉合。抬高起始姿勢＝OOD（policy 沒看過，會發散，2026-05 已踩過）。
- **真實風險（sim 有 1.4cm 磁吸容差、不罰輕碰，實機須驗證）**：
  1. 側向偏差 > （開爪半寬 − 物體半寬）→ 手指撞物體側面。目前鎖定點 y 偏差僅 0.5–1.3cm。
  2. 物體寬過開爪內側間隙。
  3. 高物體：滑行終點夾爪基座可能推到物體正面。
- **驗證法**：量 S6=30° 開爪內側間隙與物體寬 → 單側裕度=(間隙−寬)/2，≥2cm 安全；實測盯最後 7.5cm 是否乾淨跨過。
- **會撞時的對策順序**：①`--obj-z` 對準物體上半部（如 5.5cm 木塊用 0.04，等效抬高滑行末段；obj-z 是訓練變數、仍在分布內）②加強 y 對位 ③最後才考慮墊高 waypoint。

### 11. sim 與現實的 Z 軸（物體高度）基準怎麼確認？
- **定義（2026-07-16 重新核對 URDF/FK）**：FK/policy 座標的 root 是
  `base_footprint`，但部署 log 的 `TCP` 取自 `arm_link5` 的
  `getLinkState()[0]`（慣性中心），**不是兩指抓取中心**。URDF 的 `grip_joint`
  相對 `arm_link5` 自帶 `xyz="-0.0035 -0.012625 -0.0685"` 固定偏移。
- **禁止的算法**：不可用「實體指間中心高度 H_real − log TCP z」當成 z_offset；
  2026-07-16 曾算出的 `0.0600−0.0112=0.0488m` 主要是不同參考點的幾何差，已撤銷。
- **實測結果**：1.3cm 瓶蓋中心是 0.0065m。該 Z 的 rollout 會讓 S2 到 0° 下限；
  負 Z 更會在 Stage 0 振盪、無法鎖定。訓練預設 `obj-z=0.02m` 則於 22 步鎖定、
  `dist@lock≈0.042m`、S2≈2.7°，所以首輪實抓保留 0.02，再依碰高／碰低微調。
- **教訓**：先確認程式所稱 TCP 是哪個 URDF link/參考點；Z 要用 policy rollout 加
  實抓驗證，不能把不同 link 的高度直接相減。

### 12. YOLO 辨識的 XY 與手臂前往的 XY，基準點一樣嗎？
- **答案：不會自動一樣，而且每個手臂姿態都有不同相機外參。** 手臂端（FK/policy）用
  **URDF base 座標**（原點=`base_footprint`、+X=車頭前方、+Y=左）。2026-07-16 的
  `CAM_TO_BASE_X=0.1639/CAM_TO_BASE_Y=0.0331/SIGN_Y=-1` 四點結果只適用 nav-home。
- **已確認的架構錯誤**：nav-home 看得到的 25cm 目標，離 grasp-home TCP 約 25cm，超過
  實測約 15cm 工作半徑；物體進入可抓範圍後，nav-home 又看不完整。grasp-home 相機相對
  nav-home 連高度、俯角與前後方向都改變，不能只換 `cam_x`，更不能沿用
  `obj_x = arm_dist + cam_x`。
- **目前解法**：
  1. `x3plus_real_grasp.py --real --pose-only grasp-home` 只移到標定姿態。
  2. 固定車體／手臂，在可見且可達區收至少 6 個非共線實擺點；bridge
     `--calibration-only --calibration-samples 10 --once` 只輸出去畸變 `u,v`，不連 TCP。
  3. `grasp_home_homography.py` 解 `u,v -> base_link X,Y`；另留驗證點，X/Y 各 <2cm。
  4. runtime bridge 預設 grasp-home，缺合格 homography 立即拒絕；偵測落在校正 hull 外也拒絕。
  5. controller 到 grasp-home settle 後會丟棄舊 socket 封包，只接受之後的新偵測；目標離
     grasp-home TCP >15cm 時在任何 PPO/glide 前拒絕。
- **教訓**：「相機能看到」不等於「手臂能到」，「同一顆相機」也不等於「不同手臂姿態可
  共用外參」。最後 latch 必須在真正開始 PPO 的 grasp-home 姿態校正與取樣。

### 13. `ModuleNotFoundError: rplidar`，但雷達其實不是 RPLidar（✅ 已解 2026-07-16）
- **症狀**：`from rplidar import RPLidar` 失敗，USB 又是 CP2102，且出廠裝置名叫
  `/dev/rplidar`，一度以為要安裝 `rplidar-roboticia`。
- **實測辨識**：`ydlidar_ros_driver/TG.launch` 回報 **Model TG30**、firmware 2.1、
  health good、512000 baud、20K sample rate；`/scan` 約 10.17Hz，角度 -85°～+85°。
- **原因**：`/dev/rplidar` 只是 udev 別名；實體是 YDLIDAR TG30，Python RPLidar protocol
  不相容。`rostopic` 一開始失敗只是 ROS master 尚未啟動，不是雷達故障。
- **解法**：ROS 端跑 `roslaunch ydlidar_ros_driver TG.launch` 與
  `roslaunch rosbridge_server rosbridge_websocket.launch`；`grasp_venv` 安裝 `roslibpy`，
  導航用 `--lidar-backend ros --ros-host 127.0.0.1`。
- **教訓**：不能用裝置檔名判斷雷達品牌；以 driver 回報的 model/firmware/health 與實際
  `/scan` rate 為準。TG30 不安裝 `rplidar-roboticia`。

### 15. 一鍵夾取走到 C3 才被視覺端擋下：`grasp-home detection requires --homography`（✅ 已解 2026-08-14）
- **症狀**：`jetson_one_command_grasp.py` 一路正常 —— 檔案檢查過、模型載入、序列埠開啟、
  手臂確認到 C3 home —— 然後 bridge 印
  `grasp-home detection requires --homography; nav-home H/theta/cam offsets are invalid
  after the arm moves`、exit 1，launcher 把控制器一起收掉。
- **原因**：launcher 沒跟上 bridge。bridge 後來把 C3 姿勢改成 fail-closed：那裡只接受
  **實測的 pixel→base homography**，舊的 nav-home `H/θ/cam_x/cam_y/sign_y` 一律拒收，
  `--i-accept-predicted-extrinsics` 也擋不掉（那個旗標只對 nav-home 有效）。
  launcher 卻還在傳那組 nav-home 參數、而且沒傳 `--homography`。
  2026-08-03 那次成功是在加閘之前跑的，所以不是「以前會動現在壞了」，是**閘變嚴了**。
- **解法**：launcher 現在傳 `--homography`，並且在**開硬體之前**用 bridge 同一道閘
  （≥6 點、最大誤差 <2cm）驗這個檔；沒有校正檔就直接不啟動。校正檔用同一支的
  `--calibrate` 產生（手臂停在 C3 home，相機連續印 median undistorted 像素）。
  舊的 `--cam-x/--cam-y/--sign-y` 已從 launcher 移除，不是留著沒作用。
- **教訓**：**檢查要放在成本發生之前。** 這個錯誤本身沒問題，錯的是它在手臂已經上電、
  走到 C3 之後才發生。凡是「跑到一半才會發現」的前置條件，都應該在 preflight 就查。
  另外兩個程序各自版本正確、卻對不上彼此的介面，靠讀單邊程式碼是看不出來的
  —— `test_one_command_launcher.py` 就是釘住這條接線的。

## 導航 / 定位

### 14. 先設好 AMCL 再開 mission_pipeline，定位就爛掉（✅ 已解 2026-08-10）
- **症狀**：RViz 2D Pose Estimate 明明已經收斂，一啟動
  `integration/mission_pipeline.py` 就一路
  `FAIL: AMCL not usable (AMCL position variance 0.3434 > 0.0625 m^2)`，
  self-check 永遠不過、`--wait-start` 連 Enter 都沒得按。重設一次定位、再重開主程式，
  同樣的事再發生一次。
- **原因**：`odom → base_footprint` 的**唯一**發布者就是主程式自己
  （`mission_pipeline.OdomPublisher`，20 Hz，走 `ros_io.publish_odom_and_tf`），
  而 `feedback_odom.FeedbackOdom` 每次程序啟動都從 (0, 0, 0) 重新積分。
  主程式一開，odom 原點就跳回車子當下的位置，AMCL 手上的 `map→odom` 立刻對不上，
  粒子被那個假位移推開 → variance 爆掉。這不是門檻太嚴，是定位真的丟了。
- **解法**：把設定位挪到啟動主程式**之後**。主程式維持運行（它一開始印 `FAIL` 是正常的）
  → RViz 設緊初始化 → `rosservice call /request_nomotion_update` →
  確認 `covariance[0]`、`covariance[7]` < 0.0625 → self-check 每 tick 自己重跑，
  轉成 `AMCL ok at (...)` 後才按 Enter。**中途不要重開主程式**，重開就是再歸零一次。
  `integration/MISSION.md`「上機啟動順序」已改成這個順序。
- **教訓**：誰發 odom，誰就決定定位什麼時候可以設。這條專案裡本來就寫在
  `SETMOTOR_ODOM_INTEGRATION.md` §4.3（「odom reset 只允許在 AMCL 尚未依賴目前 odom 時
  執行」）與 §10.2（步驟 4 才啟動主 pipeline），只是 `MISSION.md` 的操作順序沒跟上，
  照著做就一定踩到。

---
