# 避障接近 → E1 重新辨識夾取

來源：enoch20050427 PR #6，commit `cf0b3ad0cb648c9fd1f1609e958ea2250e587e64`。
Windows 跑 `integration/sugarbox_rl_approach_final2.py`；Jetson 的 grasp-service
仍是唯一伺服／底盤序列埠持有者。沒有啟動另一個 motor server。

2026-10-04 更新：PR #7 已補回 `detection/rear_ground_homography.json`；
本次整合修正其 runtime loader，從 `samples` 的 7 個 inlier 建立校正凸包，
拒絕缺失／退化／非有限的矩陣與凸包，以及宣告不符的解析度或座標契約。
缺校正時不會略過範圍檢查。Jetson 10/03 已依使用者要求關機；10/04 已重新連線，
本次只讀 preflight 通過（資產、模型雜湊、Python imports 與服務 status）。
手臂回報 `other`、目標過期，odom 尚未連 rosbridge；輪速為零。
10/04 已完成下列只讀現場測試；有效物體追蹤與實際接近／E1 全鏈路夾取仍待驗收。

## 行為

1. 起跑前夾爪必須空、readback 有效；先停車，guarded stow 並確認 travel。
2. 後相機 YOLO/SAM2 鎖定物體；55D（goal＋odom＋prev action＋48 rays）
   經 doorway PPO 避障、動作延遲與原 LiDAR 安全層，送 TCP7000 velocity。
3. 保留原置中及最後直进控制，最後距離可調但不超過原預設 0.15 m。
4. 到位才停止並退出接近程序；退出／逾時／失敗都不觸發手臂交接。
5. 確認輪速零、odom twist 近零、回饋新鮮且 stopped_for_s ≥0.5，才 home 至 E1。
6. 等待 2.5 s 丟棄巡航畫面，再取兩次新鮮且穩定的 E1 目標，才呼叫 grasp。
   目標仍須通過 E1 服務的 homography 凸包、裁切與 v23 policy 範圍閘門。
7. 完成後保持握物；不自動搬運、倒退或放下。E1 看不到時停止，不盲目前進。

後相機是固定視角，不使用巡航姿態的手臂相機映射。
這份交接以 E1 新鮮目標為條件；尚未串接三姿態搜尋 fallback。
左右映射旗標、PPO 權重及 servo/FloorGuard 參數沒有改動。

## 部署檔案與外部資產

- Windows：`integration/sugarbox_approach_grasp.py`（預設只讀 preflight）、
  `integration/sugarbox_rl_approach_final2.py`（來源＋路徑環境變數＋到位退出＋推論排程兼容）。
- Jetson：`deploy/ros/start_rear_approach_stream.sh`（已複製到 deploy_jetson2），
  僅讀後相機 by-id SN0001，不占手臂相機；未設開機自啟。
- 避障：`integration/nav_best_model/doorway_ft_final.zip`，sha256
  `af5e9e1d5ce0115de8b4bbb321873c6a99686c82c0c69d060f881bbe97b64f35`。
- 正規化：`doorway_ft_final_vecnormalize.pkl`，sha256
  `3cd4ef9ab5b0fa38058b5142b1cc5b5df1ba9c440f325ff81b88d5e6ca7a8cdf`。
- YOLO：沿用 `detection/models/best.pt`。
- SAM2：已由 Ultralytics 官方資產下載 `detection/models/sam2.1_b.pt`，sha256
  `f1a9cf2dd69d84bb463b5ad98246d03e2d47a130a9295db0ec967e6cd95e2e47`，本機載入成功。
- 後相機校正 `detection/rear_ground_homography.json` 已補回，為 2026-08-06 的
  640×480 後相機量測。離線格式與 inlier 範圍檢查通過；現場仍須確認相機裝機／方向
  未變更，再以真實影像 dry-run 驗收。不可用 E1 校正替代。
- Windows 執行環境 `.venv-approach` 使用既有系統 torch/SB3/Ultralytics，另裝 roslibpy。
  roslibpy 2.1.0 已安裝且 Windows ROS 只讀訂閱成功；3 s 收到 scan 27、odom 55。
  GStreamer 採[官方 Python wheels](https://gstreamer.freedesktop.org/download/download.html)
  `gstreamer-bundle==1.28.7` 已安裝。啟動器位於
  `.venv-approach/Scripts/gst-launch-1.0.exe`，會自動設定 wheel 的 DLL／插件路徑。
  `gst-launch` 版本 1.28.7，必要 UDP／RTP／H.264／BGR 插件均通過檢查。
  合成影像經 H.264 編碼→RTP 封包／解包→解碼→BGR 輸出，3 幀共
  2,764,800 bytes（640×480×3×3），exit 0。這不是 Jetson 到 Windows 實際收流驗收。

## 操作

本次 Windows 環境建立方式（系統 Python 已有 torch／SB3／Ultralytics）：

```powershell
python -m venv --system-site-packages .venv-approach
.venv-approach/Scripts/python.exe -m pip install roslibpy==2.1.0 gstreamer-bundle==1.28.7
```

SAM2 權重不納入 Git；從 Ultralytics 官方資產取得 `sam2.1_b.pt` 後放在
`detection/models/`，並核對上列 sha256。後相機校正預設讀 repo 中的 JSON；
可用 `--homography` 指定另一份有效後相機量測。

第一步只檢查；預設載入 repo 後相機 JSON。`--gstreamer` 指向已安裝的
Windows gst-launch-1.0.exe。下列路徑以本次環境為例。

```powershell
.venv-approach/Scripts/python.exe integration/sugarbox_approach_grasp.py `
  --host 10.16.224.252 `
  --gstreamer .venv-approach/Scripts/gst-launch-1.0.exe
```

若要讀取相機，Windows 接收端與後相機 sender 需同時在同網段。啟動 sender
（Windows 本次 IP 是 10.16.224.160；重新連線須重查）：

```bash
ssh jetson@10.16.224.252
cd /home/jetson/Documents/deploy_jetson2
bash deploy/ros/start_rear_approach_stream.sh 10.16.224.160 5600
# 保留終端機；Ctrl+C 停止串流，不影響常駐夾取服務。
```

Windows 在上列 preflight 指令加 `--dry-run`，檢查實際辨識、方向、LiDAR、odom
與 PPO 動作。此模式不收手臂、不送馬達。之後現場監督、已放下物體、前方淨空，
才以 `--real --i-am-beside-the-robot` 啟動一次完整接近與夾取。
真正急停仍是電源；不要在接近過程移動物體或伸手。

## 2026-10-04 現場只讀 dry-run

- PR #7 已合併 main、PR #8 已合併 v23-grasp-test；Windows 已同步 `3eb79ec`。
- 啟動 `start_g2_test_stack.sh` 後，五個 ROS 感測／定位單元 active。
  `/scan` 約 10.29 Hz、`/odom_setmotor` 約 19.72 Hz，資料有限且本地接收新鮮；
  odom vx/vy/wz 與四輪指令全程零。仍由 grasp-service 單獨持有序列埠。
- 固定後相機 SN0001 的 Jetson NVENC／RTP → Windows GStreamer 實際收流成功，
  畫面 640×480；沒有占用 grasp-vision 的手臂相機。
- 45 秒主迴圈的 headless dry-run 正常 ESC 退出，ROS／Windows receiver 正常清理。
  臨時 harness 強制 DRY_RUN 並禁止建立 MotorClient；只保存程式原本的 HUD／辨識畫面。
  此次沒有送速度、stop 或伺服指令。測試完成後停止本次暫時後相機 sender，ROS 保留。
- YOLO 正式設定 conf=0.30/imgsz=640 沒有辨識到物體。離線 1280 診斷僅找到
  confidence 0.121 的藍盒候選，約 16×39 px；沒有降低 runtime 門檻。
  候選盒底 (120,298) 在校正凸包外約 163 px，因此不能視為可接近目標。
  手臂處於 other 姿態並遮住有效區，須先安全收至 travel、調整盒子再測。
- 實測 `base_link→laser` TF 為 yaw=180°、x=+0.10 m。
  原 standalone 的 π−raw 是鏡射，會把左右反向，也漏了平移。
  新 `sugarbox_lidar_geometry.py` 讓 PPO 使用完整旋轉／平移後的 base 座標，
  保留 48 束 nearest 採樣與 1.15 observation scale。
  raw safety 僅旋轉原感測扇區並保留 sensor 原始測距，維持原物理停車距離與側向 coverage；
  不能把正前方 d 換成 d+0.10 後仍宣稱門檻沒有放寬。
  對不同的實測安裝可指定 `SUGARBOX_LIDAR_YAW_OFFSET_DEG`／
  `SUGARBOX_LIDAR_FORWARD_OFFSET_M`；這不是安全閘 override。
- 錄製 scan 的離線重播前方 min 約 0.571 m；側向有極近回波，未新增遮罩或放寬門檻。
  仍須在新的現場 dry-run 確認側向回波與障礙位置。停止時 HUD front=inf
  不表示前方無障礙；PARTIAL 是視覺 odom 延遲補償，無有效目標時也會顯示。
- 同時修正 resident home 首次 FloorGuard 前未同步開機實際姿態的缺口：
  先驗證六軸有限 encoder，再同步 arm／gripper state；讀值無效、缺軸或 NaN/Inf
  一律在第一筆 servo write 前拒絕。保留 FloorGuard、限速、到位與序列埠所有權檢查。

### 使用者允許後的空爪收臂（同日）

- 使用者確認空爪、收臂範圍淨空、人在旁可立即斷電，允許張爪並收臂；底盤不行駛。
- 必要回歸 controller138／servo37／floor641／UI48、FSM 自測全過。
  新 LiDAR geometry13、travel50、chassis58、calibration21、handoff9 亦全過。
- Jetson 只更新 `grasp/v23/grasp_service.py`，使用 service 的 Python3.8 編譯通過、
  LF SHA256 `8117dc3c657bb84d9154e917855cfe68957be0f254f22ea5bb312eb652464bdf`。
  原檔保留為 `grasp_service.py.bak_home_encoder_20261004`，重啟後姿態保持不變。
- 正式 `graspctl home` 正常到位，7 iterations／2.66秒；回讀 `[90,74,8,9,90,30]`。
  `home` 同時張爪回 E1，不是獨立原地開爪命令。
- 正式 `graspctl stow` 經原中繼點，3.42秒；回讀 `[90,139,0,0,90,30]`、
  pose=travel、holding=false、arm_busy=false，輪速與odom twist保持零。
- 這是本次監督操作的單次到位結果，不更改任何 hardware_validated 旗標。
  有效物體追蹤與實車避障接近仍未驗收。

### 收臂後第二輪辨識（同日）

- 使用者確認收手臂正常、無碰撞或卡住，並重新調整藍盒。
- 第二次 bounded DRY_RUN 正常退出，主迴圈 46.03 秒；MotorClient 禁用，
  只有相機／LiDAR／odom 讀取。四輪指令與實測 odom twist 保持零。
- 即時 YOLO 框到畫面最下方的機器人夾爪，最終為 OUTSIDE_CALIBRATION，
  沒有有效目標鎖定。不能把這次檢出稱為已辨識到藍盒。
- 離線診斷：真正藍盒手動 SAM bbox 得接地 (253.5,393)，hull 距離 -4.34 px，
  在既有 10 px 邊界容許值內；此為診斷位置，非 runtime 的有效目標。
  藍盒約 18×57 px、露出窄側面，1280 診斷可檢出，但正式 imgsz=640 尚未有效檢出。
  不修改 conf=0.30、imgsz=640、校正範圍或驗證旗標。
- 使用者已同意場地淨空後的巡航辨識／避障接近／E1 夾取測試。
  下一步先轉寬正面朝相機、稍移畫面中央，再確認正式辨識和追蹤鎖定。
  校正與 LiDAR 安全距離確認前，尚未啟動底盤導航。

### 原始影像確認（同日，第三輪）

- LiDAR 新 13 項回歸及 PR #9 的 Linux CI 通過；raw safety 保留原 sensor 扇區／距離。
  收臂後 49 幀 scan 的離線 safety replay 無 hard-stop／sideguard／vx-cap 事件；
  此為靜態資料驗證，仍須實際避障測試。
- 另外保存未標註 raw frame，第三輪主迴圈 45.06 秒正常退出。
  最終正式 YOLO 為 NO_YOLO，沒有 VALID 或 TARGET 鎖定；MotorClient 全程禁用，
  實測 odom 與四輪指令保持零。
- 藍盒位於畫面中央附近但仍只露出窄側面。等待轉寬正面後的正式辨識驗證，
  未因此降低信心門檻或改動安全閘。第三輪暫時後相機 sender 已停止，ROS 保留。

## 2026-10-04 PR #8 合併前離線回歸

- 23 套 Python 測試／自測全部通過；17 套共 1,093 checks，另 6 套 selftests。
- 新 runtime 校正 21、E1 交接 9、LiDAR 安全層 12 項皆通過。
- systemd helper 在 WSL 以原內容的 LF 暫存副本驗證通過；GitHub Linux CI 另行驗證。
- 保留 v23 原避障、到位停止／退出、15 cm 上限、模型／校正閘及 resident serial owner。

## 2026-10-03 檢查結果（歷史記錄）

- 原接近程式 target-memory 自測通過。
- 真 doorway model/vecnorm 成功載入並推論，55D/2D 契約檢查通過。
  使用不影響推論的排程替代後，舊 Python lambda 反序列化警告消失，測試輸出一致。
- 新交接 7 項無硬體測試通過；涵蓋接近失敗不交接、預設不移動、未停車不動手臂、
  過期／越界目標不夾、home 後移動中止、握物未確認報失敗。
- Jetson 60 幀後相機→NVENC→RTP→fakesink 通過。短測耗時 4.90 s、
  child peak RSS 44.48 MB、CPU 約 8.99% 單核心；不是長時間串流／Windows 解碼驗收。
- 尚未執行實車避障接近、E1 全鏈路夾取或完整視覺 dry-run；機器保持 E1 握物。
- 必要回歸：v21 controller 138、servo 37、floor 641 全過，ui/test_server exit 0、
  mission_fsm --selftest PASSED、git diff --check 通過。
- 只讀預檢目前只拒絕缺少後相機校正；SAM2 與 Windows gst-launch 已補齊。
  另查主 repo 與兩位協作者全部分支、PR #6、release：未找到此 JSON。
  遠端 CLAUDE 明確註記檔案不在 repo；PR #6 原程式從
  `C:\Users\user\Desktop\igibson_sugarbox_vision\rear_ground_homography.json` 載入。
  最後機器讀值 [90,73,8,9,90,149]、E1握物、motors=[0,0,0,0]、odom failures=0，
  grasp-service／grasp-vision active。此次未發送底盤速度或手臂動作指令。

驗證指令：
```bash
python tests/test_sugarbox_approach_grasp.py
python tests/test_sugarbox_ground_calibration.py
python integration/sugarbox_rl_approach_final2.py --selftest-target-memory
python -m py_compile integration/sugarbox_approach_grasp.py integration/sugarbox_rl_approach_final2.py
```
