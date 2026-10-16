# 決賽 demo 的 v23 執行入口

決賽以 `v23-grasp-test`、v23 E1 模型及常駐 grasp-service 為主。
導航、建圖、監督器、相機與文件留在各自共用目錄；`grasp/v23/demo.py`
統一調用，不複製控制實作。v21 與 v24 保留供回歸／參考，未完成 gate 不改 true。
舊 A/B/C 任務仍是 v21／獨立序列架構，不作決賽實機入口。

## 正式展示目標與準備順序（2026-10-17）

正式主線為「已建圖場地定位 → 巡航尋物 → 視覺接近/E1交接 → 夾取 → 收臂搬運 →
導航至投放點 → 投放 → 接續任務」。定點夾取/原地投放是模組測試與備援。
112303577ncu 的下一個現場主項是先重驗新地板/新濾波下的建圖、AMCL與巡航/投放點路線，
不是把建圖移到完成定點展示之後；離線模組檢查可並行，但不能取代地圖定位驗收。
完整清單及GitHub現況見[10/17核對](../../docs/planning/V23_FINAL_DEMO_REVIEW_2026-10-17.md)。

目前本檔的plan後端只接受home/grasp/stow/release/短直線drive，沒有巡航或導航垃圾桶action；
`mission_pipeline.py`仍為舊獨立序列架構且實機入口拒絕執行，不得直接拿來接替常駐owner。
已具備建圖啟動工具不等於已完成整條v23任務鏈。

10/16部署：固定自回波濾波 `/scan_raw → /scan`、獨立health相容介面、vision心跳、
控制/視覺限定監督器已啟用；AMCL/新路線、冷開機與完整任務仍待驗證。
現行監督器用`v23-control-vision-monitor.service`，不訂閱ROS、不自動重啟owner；
不要直接啟用下方舊全功能monitor配置。部署說明見
[健康介面與恢復](../../docs/operations/HEALTH_COMPAT_RECOVERY_2026-10-16.md)。

## 離線驗證

```bash
python grasp/v23/demo.py --plan grasp/v23/demo_grasp.json
python grasp/v23/demo.py --plan grasp/v23/demo_pick_place.json
python -m pytest tests/test_v23_demo.py tests/test_g2_safety_audit.py tests/test_g2_demo_supervisor.py -q
```

預設只印預覽，不建立實機 backend。grasp 計畫：home/E1 → grasp → stow（保留夾持）。
pick_place 計畫另執行 release → stow，是原地夾取／投放，不代表已找到垃圾桶。

## Jetson 上的統一入口

以下須先部署新版 service health API、vision 心跳設定，於專案根目錄執行：

```bash
# ROS／序列／RAM／相機心跳唯讀檢查
~/grasp_venv/bin/python3 grasp/v23/demo.py check --seconds 30
# 排障後取得 maintenance 鎖、完整驗證再解鎖
~/grasp_venv/bin/python3 grasp/v23/demo.py check --boot-gate --repair-ros --seconds 90
# 現行 /scan（10/16起為固定濾波後）／odom 唯讀探測；原始資料在 /scan_raw
~/grasp_venv/bin/python3 grasp/v23/demo.py probe
# 建圖（先 source ROS Melodic 環境）
source /opt/ros/melodic/setup.bash
~/grasp_venv/bin/python3 grasp/v23/demo.py map --seconds 300
# 僅停車時讀實際編碼器
~/grasp_venv/bin/python3 grasp/v23/demo.py arm status
# 明確執行計畫
~/grasp_venv/bin/python3 grasp/v23/demo.py --plan grasp/v23/demo_grasp.json --execute
```

`supervise` 子命令調用既有有限 UDP 意圖／TCP 7000 監督器。
現場持續監督授權已提供，不重複詢問；開機不自動開始 demo。

## 計畫與後端限制

- 最多 20 步、結尾 stow；不接受任意 shell 或未知 action。
- home 張爪，只允許空夾爪；grasp 需 E1、確認夾持後才可搬運／release。
- drive 僅巡航姿態、0.10–0.25 m 直線，須提供與 scan 相符的方向證據。
  沿用 g2_nav_client 現行 /scan 煞車、進度、偏差、時間及最終停車驗證。
- v23 model pair、唯一 serial owner、零速、有效 odom、無 maintenance 鎖、
  本次 vision PID 的新鮮影像／推論心跳必須正常。
- 步驟前後 RAM ≥1024 MB；導航期間低於 768 MB 中止子程序。
  grasp-service 的 arm 命令亦配置 768 MB 起動閘。
- 只允許一個 demo runner；逾時／未知結果立即中止，不重送夾取、不自行張爪／投放，
  不重播未知 checkpoint。短暫故障先有限等待／修復，完成點可明確 `--resume`；
  無法恢復時降級 parked／diagnostics，不把降級描述成完成原任務。
- `--output` 覆寫單一狀態，不累積影像、scan 或無限事件。

drive 的 JSON 格式為 `{"action":"drive","distance_m":0.15}`；plan 頂層須有
`"version":"v23"`、`"steps":[...]`、`"orientation_evidence":"實際證據.json"`。
證據路徑相對 plan 檔案解析。欲夾取時安排 home、grasp、stow，不能在 E1 行駛。

## 尚待驗收

新版部署與冷開機；新地圖 AMCL、垃圾桶接近點、完整路徑與側向轉向包絡；
車身線材量測／濾波 A/B；USB port3 根因；v23 LEFT/RIGHT 映射、搆及範圍與 8 mm margin。
此入口沒有自主 map patrol、自動尋物／對準，短直線計畫不能代替這些驗收。
RAM 起動閘不能防止其他程序突然分配記憶體，長時與 OOM 壓力測試未完成。
操作台正式頁面現使用 D／v23 同一 plan 後端，預設「定點夾取」，可勾選「原地投放」。
類別與物體高度顯示部署的固定 sugarbox／6.5 cm；在 UI 不改寫 service 校正值。
server 預設唯讀預覽；實機仍需以 --allow-real 啟動，並沿用既有操作台啟動條件。
UI 停止會終止計畫／導航子程序，但已交給 service 的手臂命令可能仍在完成，
不能把它當成伺服機的即時斷電按鈕。

## 等待、恢復與降級

實作與目的、部署界線見 [V23_RECOVERY_2026-10-07.md](../../docs/operations/V23_RECOVERY_2026-10-07.md)。
`demo.py monitor`保留完整候選監督入口；本次實機只部署上述獨立版本bundle的control/vision模式，
由`v23-control-vision-monitor.service`啟動，不直接啟用舊的全功能/owner重啟配置。
完整Serial transport與owner升級重啟仍須另行整合，不能把相容health補丁當成全部API已具備。
