# 夾取切換延遲壓縮計畫（2026-09-21）

## 目標

正式任務會反覆執行「導航 → 夾取 → 導航」。切換時不得重啟 PyTorch、PyBullet、YOLO
或重新開啟 `/dev/myserial`。目標是把每輪切換的冷啟動成本降為 **0 秒**，只保留實際
夾取動作時間；目前暖機後實測成功夾取為 **8.05–8.53 秒**。

## 已量到的基線

| 項目 | 實測 |
|---|---:|
| 舊版單次完整冷啟動夾取 | 42.3 s |
| 控制器／模型啟動 | 23.3 s |
| 視覺冷啟動 | 10.5 s |
| 實際夾取動作 | 8.1 s |
| v23 常駐服務冷啟動至 ready | 約 26 s |
| v23 暖機後單次成功夾取 | 8.05–8.53 s |
| 導航 + ROS + LiDAR 記憶體 used | 約 1.33 GB |
| 再加常駐夾取與視覺 used | 約 2.37–2.61 GB / 3.96 GB |

Jetson Nano 的 RAM 足以讓導航、夾取模型與視覺同時常駐；目前瓶頸不是 RAM，而是
`/dev/myserial` 只能有一個程序擁有，以及每次重啟模型的冷啟動成本。

## 正式架構

只保留一個 mission process 持有 Rosmaster serial handle，並把同一個 handle 傳給：

1. 底盤控制與 feedback odom；
2. 導航狀態機；
3. 手臂與夾爪控制器。

`integration/mission_pipeline.py` 已具備這個骨架：先建立一次 `GraspController`，再把
`controller.servo.device` 傳給 `MissionNavigator` 與 `FeedbackOdomReader`。因此正式
流程不使用 systemd 在 `x3plus-navigation.service` 與 `grasp-service.service` 之間來回
切換；兩個 service 模式只保留給獨立維修與測試。

## 分階段執行

### G0 — WP2 期間保持現況

- 先完成 odom、LiDAR 方向、AMCL 與導航硬體 gate。
- 不在 WP2 中整包覆蓋 Jetson 的 v23；避免同時變更導航與夾取基線。

### G1 — 降低一次性開機成本

- 只部署經 `make_deploy_urdf.py --verify` 通過的 FK-only `yahboomcar_deploy.urdf`。
- 在 Jetson 重量測 PyBullet／URDF 載入時間；開發機的 3.66 → 0.14 秒不能直接當成
  Nano 的結果。
- 讓 YOLO bridge 與 PPO／PyBullet 載入並行（`--wait-for-go`），但仍只在開機支付一次。

### G2 — v23 併入單一 serial owner

- 把已驗證的 v23 controller 接進 mission pipeline，而不是另外啟動一個 serial service。
- 啟動後保持模型、相機與 serial handle 常駐；狀態切換只呼叫函式，不 fork 新程序。
- 導航狀態時手臂維持 E1/home；進入夾取只需要取得最新偵測並執行 episode。

### G3 — 實機耐久驗收

連續完成至少 10 輪「導航 → 夾取 → 導航」，每輪都檢查：

- `/dev/myserial` 恰好一個 owner；
- 沒有 controller、YOLO 或 PyBullet 重載訊息；
- 導航轉夾取不包含約 26 秒冷啟動；
- 成功夾取時間維持約 8.5 秒；
- RSS 無持續增長、可用 RAM 保持安全餘量；
- odom 在夾取前後連續，沒有重新歸零或 TF 跳變。

## 不優先壓縮的項目

伺服機 scripted-move 時序與 jaw step 會直接改變接觸、夾持與碰撞風險。這些調整只有在
G1/G2 完成且保留完整安全 gate 後才逐項做 A/B 實機驗證；不為省下零點幾秒而先修改。

## 預期結果

- 正常任務：開機預載一次，導航 ↔ 夾取切換不再付冷啟動成本。
- 夾取階段：現階段以 **約 8.5 秒**為可驗證目標。
- 系統冷開機：G1 可再縮短，但以 Jetson 實測為準，不先承諾固定秒數。
