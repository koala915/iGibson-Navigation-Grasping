# ChatGPT `igibson` 專案脈絡整理

整理日期：2026-09-24
來源：ChatGPT 網頁版 `igibson` project 中目前可見的對話

> 這份文件是歷史脈絡與整合決策的摘要，不是實機現況的唯一事實來源。
> IP、USB 裝置編號、相機編號、延遲量測與服務狀態都可能在重開機後改變，部署前必須重新檢查。
> 程式行為與安全契約仍以 repo 內的 `CLAUDE.md`、manifest、測試及當次實機紀錄為準。

## 1. 系統目標與分工

專題目標是讓 Yahboom X3Plus 在 iGibson／實體環境中完成：

1. 巡航與 LiDAR 避障。
2. 以 YOLO 偵測 `sugarbox`，以 SAM2 取得更精確的地面接觸位置。
3. PPO 導航接近目標。
4. 近距離以影像中心做最終對位，再前進固定距離。
5. 將目標交給手臂相機與夾取流程。

目前採用的運算分工：

- **Jetson Nano**：ROS Melodic、LiDAR、底盤／手臂硬體、定位與任務執行。
- **Windows 開發機**：後相機串流接收、YOLO、SAM2，以及開發與監控。
- **影像路徑**：Jetson `/dev/video*` -> H.264 RTP/UDP -> Windows GStreamer -> YOLO/SAM2。
- **控制路徑**：Windows/Codex -> SSH -> Jetson；Jetson 上以非 root 使用者執行。

## 2. Jetson 歷史環境快照

以下來自 2026-09-19 前後的對話，只能作為重建環境時的參考：

- Yahboom X3Plus，Jetson Nano 4GB。
- Ubuntu 18.04.6 LTS，kernel `4.9.253-tegra`。
- ROS Melodic，系統 Python 3.6.9。
- 當時根目錄位於 `/dev/sda1`，RAM 約 3.9GB，swap 約 6GB。
- 當時 Jetson IP 是 `172.23.173.252`，Windows IP 是 `172.23.173.27`；兩者皆不可視為固定值。
- SSH 使用者為 `jetson`，曾完成金鑰登入。
- SSH 金鑰權限修正紀錄：`/home/jetson` 755、`.ssh` 700、`authorized_keys` 600。
- ROS apt key 曾出現 `EXPKEYSIG F42ED6FBAB17C654`；舊 Jetson 軟體棧不可直接盲目執行全面升級。

曾觀察到的裝置對應：

- CP210x `10c4:ea60`：可能是 YDLIDAR TG30。
- CH340/CH341 `1a86:7523`：可能是 Rosmaster。
- 後相機曾位於 `/dev/video1`。

部署時應以 udev 屬性與 `v4l2-ctl` 重新辨識，不依賴 `/dev/ttyUSB0`、`/dev/ttyUSB1` 或 `/dev/video1` 的歷史編號。

## 3. 影像傳輸決策

原本 HTTP/MJPEG 路徑曾出現影像緩衝持續累積，後來改為 H.264 RTP/UDP + GStreamer，並使用 leaky queue 丟棄舊影格，讓推論優先處理最新畫面。

歷史量測約為：

- 串流：約 30 ms。
- queue：約 14 ms。
- RL 推論：約 170 ms。
- 合計：約 214 ms。

這些數字不是目前效能保證。repo 內的 `detection/rear_cam_sam2_publisher.py` 已支援 HTTP 與 UDP/H.264，但 UDP 路徑仍需在目前硬體與網路上做 live test。

## 4. PPO 導航契約

導航模型的重要契約如下：

- observation 固定為 55 維。
- PPO 使用 48 束 LiDAR，角度為前方 `-90..+90` 度。
- 不可把 PPO 輸入直接擴成 250 度；即使維度不變，角度語意改變也會破壞訓練契約。
- 額外安全層可獨立使用可信的 `-125..+125` 度 LiDAR，不改 PPO observation。
- PPO 前一動作 observation 應保留原始 policy action，不可換成安全層覆寫後的命令。

目前 repo 內的導航模型組：

- `integration/nav_best_model/doorway_ft_final.zip`
- `integration/nav_best_model/doorway_ft_final_vecnormalize.pkl`

歷史核對的 SHA256：

- model：`AF5E9E1D5CE0115DE8B4BBB321873C6A99686C82C0C69D060F881BBE97B64F35`
- VecNormalize：`3CD4EF9AB5B0FA38058B5142B1CC5B5DF1BA9C440F325FF81B88D5E6CA7A8CDF`

這組仍應視為 candidate，而非未經驗證即可上機的預設模型。對話中曾出現 `motor_delay_steps=0` 與 LiDAR 距離乘上 1.15 的獨立設定，但缺乏訓練契約證據，因此目前整合保留既有的 2-step motor delay 與原始公尺距離。

## 5. LiDAR 安全層決策

`integration/nav_safety.py` 對應的歷史參數：

- 前方扇區：正負 22 度。
- robust percentile：5%，最少 8 個有效點。
- hard stop：進入 0.18 m，解除 0.22 m。
- forward block：進入 0.30 m，解除 0.36 m。
- slow start：0.55 m。
- turn hold：進入 0.45 m，解除 0.55 m。
- 轉向方向需連續 2 tick 才可切換。
- 側邊保護範圍：右側 `-125..-25` 度，左側 `+25..+125` 度。
- side guard：進入 0.20 m，解除 0.24 m。
- 側邊小於 0.16 m 時，前進速度上限 0.06 m/s。

hard stop 脫困策略曾有不同版本。最後一版需求是不信任被車體遮住的後方 LiDAR，只在 hard stop 持續 0.30 秒後做一次短距離倒車：

- `vx=-0.08 m/s`
- odometry 距離約 0.06 m
- timeout 1.5 s
- settle 0.20 s
- 同一 hard-stop episode 最多一次

因倒車方向沒有可信後向感測，repo 目前把它保留為明確 opt-in 的 `--hard-stop-recovery`。未完成現場安全驗證前，不應自動開啟。

最新對話希望 PPO 角速度 action 每個 policy step 的最大變化為 0.15，只限制 angular action，linear action 不變；位置在 motor delay 後、action-to-velocity 前。repo 已以 `--ppo-angular-max-delta` 提供 opt-in，預設 0.0。

## 6. Sugarbox 最終接近流程

不能再用 `target.dist <= 0.45` 直接判定抵達。預期流程為：

1. PPO 導航接近 sugarbox。
2. bbox bottom 到達畫面高度的 0.78。
3. 使用 YOLO bbox center 做 pixel P-control，不使用 SAM mask 最低點 x。
4. 中心容許誤差正負 30 px。
5. `Kp=0.0045`。
6. 角速度絕對值限制在 0.50 到 0.78 rad/s。
7. bbox center 單次跳動超過 180 px 視為無效。
8. 連續 2 個 fresh frame 置中才通過。
9. 最終以前進速度 0.08 m/s 前進。
10. 使用 odometry 量測 0.15 m。
11. timeout 3.5 s。
12. 完成後才進入下一個 FSM 階段。

`integration/target_approach.py` 已實作上述階段與參數。

尚未定案的一項差異：舊對話曾建議視覺暫時失效時沿用上一個有效 `wz`；目前 repo 採 fail closed，無效影格或過大跳動會輸出零角速度。初次實機部署應先維持 fail closed，再依 log 決定是否加入有時限的 command hold。

## 7. Repo 目前對應

本次合併工作位於 branch `feature/sugarbox-approach-integration`，截至本文件建立時尚未提交或合併到 `main`。

已落地的主要模組：

- `detection/rear_cam_sam2_publisher.py`：Windows 端 YOLO/SAM2 publisher，支援 HTTP 與 UDP/H.264。
- `detection/calibration/calibrate_rear_ground_homography.py`：後相機地面 homography 校正。
- `integration/trash_target.py`：offboard 目標轉接與座標方向修正。
- `integration/nav_safety.py`：LiDAR 安全層、選配 hard-stop recovery、PPO angular limiter。
- `integration/target_approach.py`：bbox 置中與 odometry 最終前進。
- `integration/mission_pipeline.py`：將 offboard sugarbox 接近流程接入完整任務。
- `integration/mission_fsm.py`：支援新增的接近階段與狀態轉移。
- `tests/`：對應的安全層、homography、target approach 與端對端測試。

模型與執行期檔案的原則：

- `sam2.1_b.pt` 是另行取得的模型權重；`detection/rear_ground_homography.json` 已於 2026-10-03 補入 git，提供本機 2026-08-06 的 640×480 校正資料。相機位置或影像尺寸改變後須重新校正。
- YOLO/SAM2 推論放在 Windows，避免增加 Jetson Nano 記憶體壓力。
- `/dev/myserial` 仍只能由單一任務程序持有。

## 8. 目前風險與待驗證項目

依優先順序：

1. 在實機重新確認 Jetson IP、SSH、ROS、序列埠與相機裝置身份。
2. 驗證 UDP/H.264 的 live stream、fresh-frame 判斷及長時間運作是否仍維持低延遲。
3. 驗證 LiDAR scan 是否會 stale，並確認 stale 時底盤一定停車。
4. 低速馬達的靜摩擦／最低有效輸出仍可能放大左右擺動。
5. 以架高輪子或低風險場地先測 safety layer，不直接進完整任務。
6. 分別測 PPO 接近、bbox 置中、0.15 m odom 前進，再測完整 FSM。
7. 確認影像掉幀或 bbox 跳動時，現行 fail-closed 行為是否符合實機需求。
8. hard-stop 短倒車必須有人在場、後方淨空，且先保持 opt-in。

## 9. 建議的上機驗證順序

1. 執行只讀設備盤點與 repo preflight。
2. 測試 Windows 到 Jetson 的 SSH 與 rosbridge。
3. 只開相機串流與 publisher，不驅動馬達。
4. 驗證 `/trash_target/detection` 的時間戳、frame identity、座標方向與距離。
5. 在模擬／輪子架高情境測 PPO 輸出及 LiDAR safety override。
6. 在低速開放場地測 sugarbox 最終置中與 0.15 m 前進。
7. 最後才啟用完整巡航、接近與夾取任務。

## 10. 已讀取的 ChatGPT 對話

- `桌面版資料串聯方式`
- `UDP RTP與HTTP TCP差異`
- `每週進度報告`
- `合併模型建議`
- `機器人異常關機處理`
- `設定Jetson SSH`

這些對話已整理進本文件；未來若網頁版 project 新增決策，需再次明確要求讀取，桌面任務不會自動持續同步新的對話內容。
