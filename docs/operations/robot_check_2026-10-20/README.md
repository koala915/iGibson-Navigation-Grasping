# 車端唯讀檢查：10.53.253.252（2026-10-20）

SSH 連線成功，主機 `yahboom`。本次只查詢既有服務、檔案雜湊及健康資料，沒有部署、
重啟、解鎖、啟動串流、巡航、手臂移動或夾取。

## 觀察

- `grasp-service` 與 `grasp-vision` 均 active；服務 PID 8814。
- 實際部署目錄 `/home/jetson/Documents/deploy_jetson2` 不是 Git checkout，須用檔案雜湊
  與備份確認版本，不能依分支名稱假定與本機同步。
- v23 health 協定相容：獨立 health、holding、maintenance、service PID、TCP readiness
  均存在；狀態為空爪、travel、四輪零輸出、maintenance=true、arm_busy=false。
- 里程回授 valid，ROS bridge 正常，當下 twist 全零；手臂相機 frame report 持續更新。
- E1 `receipt_monotonic`、`sample_monotonic`、`stationary_since_monotonic` 欄位尚未存在。
  新 runner 目前會沿用舊等待方式；若要啟用快速 E1，須對車上 G2 版本選擇性加入 metadata。
- **巡航定位未就緒**：`ready=false`、`REFERENCE_REQUIRED`、`fresh pose seed required`、
  `reference_kind=none`，且 `/dev/shm/g2_localization_seed.json` 不存在。沒有繞過定位檢查。
- 場景中的投放盒位置尚未標記已驗證，預檢有警告；依原規則僅可朝該估計位置轉向，
  不能只憑未驗證地圖位置盲開。

## 新通訊方式實測

用 `RobotSession` 在同一條 SSH 連線依序讀 health、health、status、health。
首次約 0.797 s（含連線建立）；接續約 0.015／0.032／低於計時解析度 s。
這是四筆唯讀查詢樣本，不代表整段機器人流程耗時。原始資料在 `rpc_readonly.json`。

執行未帶 `--real` 的新版總控預檢，正常拒絕巡航：
`[BLOCKED] localization: not localized: fresh pose seed required`。
輸出保留於 `preflight.txt`。

巡航前必須取得新位置參考並確認定位健康；本次沒有依猜測寫入起點，也沒有變更維護鎖。
