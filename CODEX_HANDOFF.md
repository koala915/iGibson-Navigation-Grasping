# Codex 下一步（2026-10-17）

正式分支是112303577ncu/iGibson-Navigation-Grasping的v23-grasp-test。機器人已依要求關機；最後IP為172.20.10.3，下次開機先確認實際IP。不要為本次GitHub推送重連/開機。

## 正式目標與下一次現場順序

正式展示：已建圖場地定位 → 巡航尋物 → 視覺接近/E1交接 → 夾取 → 收臂搬運 → 投放點導航 → 投放 → 接續任務。
112303577ncu先做基本冷開機確認，再做雷射座標/濾波對照 → 新地板/新濾波建圖與尺度 → AMCL → 巡航/投放點短路線，最後接全隊流程。定點夾取是模組測試/備援，不能取代導航主線。
詳見[10/17 GitHub與決賽目標核對](docs/planning/V23_FINAL_DEMO_REVIEW_2026-10-17.md)、[正式優先順序](docs/planning/V23_FINAL_DEMO_PRIORITIES.md)。只讀所需檔案，不重新掃描整個專案。

## 10/16成果與部署界線

- 直線odom合計88.85cm、尺量約88–89.5cm；左89.94°/右89.23°均現場確認約90°，保留倍率。微動實際0.6–1.0cm；左5°第1輪odom5.75°僅目測相近，使用者無量角器，精密量測暫緩。一般停止距離/超轉仍須處理。
- LiDAR左右方向已做紙板A/B確認。固定自回波profile已接到/scan；原始資料在/scan_raw，開機不自動學習。外物漏檢對照仍未驗收。車體前20/後13/左12/右12cm含線材；雷射實測前13cm，但正式TF/一般客戶端仍10cm，建圖前要核對統一，不直接套舊地圖。
- 實機原稿以最小補丁新增獨立/tmp/grasp_health.sock與推論心跳/tmp/x3plus_vision_health.json。手臂忙碌時73次健康查詢全成功、最慢10.9ms；home/stow時原生odom/TF最大間隔約65ms。
- v23-control-vision-monitor.service已enabled；限定控制/視覺、無ROS訂閱、無owner自動重啟。相機故障約12–15秒可自行恢復。最後維護鎖保留、底盤零速、巡航位、holding=false。冷開機尚未驗證。
- 部署重啟owner一次，PID30006→2518，odom原點已重置；所有後續動作重建reference，不能續用舊量測座標。完整Serial transport與owner故障恢復尚未部署。
- 監督器使用/home/jetson/health_compat_20261016/recovery四模組bundle；原稿備份/manifest在同目錄。vision有效argv已保存為zz-preserve-deployed-20261016.conf，保留組員allow-top-clipped/frame-report等參數。未覆寫組員校正。
- 曾發現ROS背景記錄影響rosbridge，已改預設RAM-only。RAM記錄依使用者要求停止：23分19秒、685筆，最低1749MB；complete=false代表提前停止，非滿30分鐘驗收，期間重啟程序不能證明無洩漏。所有結果已取回本機。
- 相關105項通過、skip1，五組回歸通過；controller/floor仍需測試程序相對URDF路徑轉接，Windows中文絕對路徑問題未修復。

## GitHub核對與操作限制

10/17核對：自己的origin在本次推送前為e980fa8；koala v23為3eb79ec，最新接近分支為d06bba3（新增10/12實測）；enoch獨立快照仍ebad46e。只fetch/review，未合併owner/隊友分支。
現行v23 plan只有home/grasp/stow/release/短直線drive；完整地圖巡航/投放導航入口仍待接通。舊mission_pipeline是獨立Serial且拒絕實機，不得直接啟動取代常駐owner。
不要再問已授權的現場安全；僅量距/量角/擺放物件時提示配合。保留真實資料/狀態閘，不偽造通過，不重播未知夾取/投放。不開第二Serial owner，不覆寫組員，不自行合併owner。

## 記錄入口

- [新地板里程計](docs/operations/ODOM_NEW_FLOOR_2026-10-16.md)
- [左右90°](docs/operations/ODOM_LEFT_90_2026-10-16.md)、[小角度](docs/operations/ODOM_SMALL_ANGLE_2026-10-16.md)
- [LiDAR固定濾波](docs/operations/LIDAR_SELF_FILTER_2026-10-16.md)
- [健康相容介面/自動修復/關機結果](docs/operations/HEALTH_COMPAT_RECOVERY_2026-10-16.md)
- [開機穩定性測試](docs/operations/BOOT_STABILITY_TEST_2026-10-16.md)
- JSON與有限log：docs/calibration/odom_new_floor_2026-10-16/。
- 10/16逐段舊交接已保存在[封存](docs/handoff/CODEX_HANDOFF_2026-10-16_ARCHIVE.md)，其中「等待回覆/未部署/正在記錄」為當時狀態，以本檔為準。
