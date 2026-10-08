# Codex 下一步

決賽主線為 `v23-grasp-test`／v23 常駐服務，入口 `grasp/v23/demo.py`。

- 決賽目的與優先順序：[決賽目標](docs/planning/V23_FINAL_DEMO_PRIORITIES.md)。
- 最新狀態與部署界線：[交接](docs/handoff/V23_FINAL_DEMO_2026-10-07.md)。
- 操作：[FINAL_DEMO.md](grasp/v23/FINAL_DEMO.md)。
- 有限修復／降級／升級重啟：[恢復說明](docs/operations/V23_RECOVERY_2026-10-07.md)。

SSH 172.20.10.2 已恢復。正式服務曾出現控制板長時間無回傳、odom 停止發布、相機啟動失敗。
新版只在本機與 Jetson `/tmp` 假硬體驗證，尚未覆寫正式部署或啟用監督單元。
優先唯讀確認回傳／相機，再比對遠端檔案及組員校正、逐檔備份部署與驗證。
USB issue 暫緩；今天沒有場地。不得啟動第二個 Serial owner、覆寫組員校正、合併 owner，
或把靜態通過當成完整自主導航／AMCL／側向避障驗收。
