# 避障接近 → E1 重新辨識夾取

來源：enoch20050427 PR #6，commit `cf0b3ad0cb648c9fd1f1609e958ea2250e587e64`。
Windows 跑 `integration/sugarbox_rl_approach_final2.py`；Jetson 的 grasp-service
仍是唯一伺服／底盤序列埠持有者。沒有啟動另一個 motor server。

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
- **仍缺現場後相機校正 `rear_ground_homography.json`**。不可用 E1 校正替代、
  不可自行捏造矩陣。須確認來源裝機／相機方向、640×480 解析度與這台機器相符。
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
`detection/models/`，並核對上列 sha256。後相機校正須另外向原作者取得。

第一步只檢查；`--homography` 指向已確認的後相機校正檔，`--gstreamer` 指向
Windows gst-launch-1.0.exe。以下指令中尖括號為待替換路徑。

```powershell
.venv-approach/Scripts/python.exe integration/sugarbox_approach_grasp.py `
  --host 10.16.224.252 --homography <後相機校正檔> --gstreamer <gst-launch-1.0.exe>
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

## 2026-10-03 檢查結果

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
python integration/sugarbox_rl_approach_final2.py --selftest-target-memory
python -m py_compile integration/sugarbox_approach_grasp.py integration/sugarbox_rl_approach_final2.py
```
