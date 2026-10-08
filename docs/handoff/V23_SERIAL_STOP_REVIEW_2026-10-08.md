# v23 更新整合與序列埠停車修正（2026-10-08）

## 本次結果

已將 ncu 的 v23 demo / recovery 更新合併到隔離分支 `codex/v23-serial-stop-fix`，修復「一次序列埠寫入錯誤後，連停車命令都永久拒絕」的問題。未部署到機器人、未執行實機運動、未推送 GitHub，也未向 ncu 發送訊息。

## GitHub 版本與整合範圍

- ncu fork：`112303577ncu/iGibson-Navigation-Grasping`，`v23-grasp-test`，`e980fa8f8612bfb64d40a466df223f17083d1513`。
- 原工作分支：`koala915/iGibson-Navigation-Grasping`，`codex/approach-lidar-frame`，`560e2f3e4f0a56775b6c546bd0de5dece21ec5a5`。
- ncu 相對共同祖先新增 118 個檔案變更、8010 行增加、73 行刪除。這是 fork 的更新，不代表 owner 的 main 已經包含它。
- `27260dc`：先保存原本的轉向側面 LiDAR 防護、禁止自動倒退、無進展停止及規劃文件。
- `b3d6caf`：無衝突合併 ncu 更新。原工作目錄中的 R7 底盤限制及其測試仍保留，沒有納入本次工作。

## ncu 新增什麼

1. `grasp/v23/demo.py` 與 JSON 計畫：E1、夾取、收手、可選原地釋放，以及透過 `g2_nav_client` 的 10–25 cm 直行；支援 checkpoint / resume。
2. 有次數限制的 vision、LiDAR、ROS bridge 修復；持續 serial 故障時限制 grasp-service 重啟；重啟後有不動作診斷模式。
3. 自訂 serial transport、底盤 maintenance lock、板端健康閘、grasp-service health socket 與記憶體檢查、vision heartbeat。
4. UI 的主控台改接 demo 入口，新增 boot / safety audit 與操作文件。

這些新增能力不等於完整自主「遠處尋物 → 避障接近 → 夾取」已驗收。ncu 文件標明新版尚未部署，Jetson 完整測試與實機驗收仍待完成。

## 停車問題及修正

舊 transport 在第一次 exception、short write 或鎖逾時後鎖存 `write_fault`，並在後續每一次 `write()` 一開始直接拋錯。因此埠恢復後，watchdog 的零速封包也不會寫出去。

修正後：

- 每次寫入仍嘗試取得有期限的鎖並送出封包；失敗不再永久封鎖後續停車寫入。
- `write_fault` 仍鎖存，health 持續不健康，現有底盤／手臂健康閘繼續拒絕新的運動。
- `require_write_ok()` 判斷呼叫執行緒最近一次寫入結果；成功送出的停車可更新軟體記錄，其他執行緒的成功不會蓋掉此執行緒的失敗。
- 不自動重開埠，不重播舊運動命令，不自動清除健康故障。

序列埠確實斷線時，軟體無法保證零速命令到達控制板。本次修正保障的是：恢復可寫後，停止命令仍有機會送達；fake board 驗證不能替代實際輪速與硬體測試。

## 驗證

- 新增 5 個測試：一次 exception、一次 short write、鎖逾時後 watchdog 停車、連續停車失敗後恢復、跨執行緒錯誤隔離。這 5 個測試在修正前全部失敗，修正後全部通過。
- serial stop / recovery / demo / safety audit / supervisor pytest：97 通過、1 skipped。
- 完整回歸 32 組：31 組通過。`grasp/v23/test_startup_pose.py` 共 13 項中 10 項通過、3 項因本機 Windows Python 缺少 `socket.AF_UNIX` 而 error；未將這 3 項列為通過，需在 Linux / Jetson 重跑。
- v21 控制器、伺服讀取、floor guard、v23 控制器／底盤、雙相機相關整合、UI、mission selftest 都在上述通過組內。
- Git Bash systemd helper tests 通過。
- 完整回歸逐組結果：`docs/operations/V23_SERIAL_STOP_REGRESSION_2026-10-08.json`。
- CI 已加入新的 serial stop 測試。

## 下一步

1. 在 Linux / Jetson 重跑 startup pose 與完整 CI；先使用 fake serial，確認修正與實際 Rosmaster_Lib 的吞例外行为相容。
2. 審核部署差異與服務啟動設定，先做不動作健康檢查；完成檢查後才安排有人看守的實機停車／轉向驗證。
3. 保留目前禁止自動倒退、側面防護與無進展停止。1.5 m / 2 m 靜態辨識、travel-pose 視野及校正仍未完成。
4. 繼續雙相機計畫前，先取得以上量測，才能決定可靠辨識距離與固定直行距離；不能直接沿用尚未量測的速度／滑行誤差。
