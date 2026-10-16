# 開機後穩定性測試（10/16）

後續已完成獨立健康socket、影像心跳及有限相機修復監督器部署，解決下方當時記錄的介面缺口；
見 [健康介面與恢復](HEALTH_COMPAT_RECOVERY_2026-10-16.md)。此輪部署重啟owner一次，odom原點已重設。

## 已執行

使用者目前沒有量角器，要求小角度下次再做；左5°第1輪僅記目測相近。
改依 `NEXT_BOOT_DEMO_STABILITY_2026-10-12.md` 的相機、共同Serial與RAM基線優先執行。

### 相機恢復

既有vision服務失敗日誌為開啟相機失敗。現在兩台相機by-id均已出現：
SN0001→video0，原服務設定的不帶SN名稱→video1。
不替換相機、校正檔、模型或組員其他選項；先測原設定路徑連續5張640x480，均成功。
reset-failed後啟動原grasp-vision，服務active，日誌確認模型載入、影像尺寸符合校正。
`/dev/shm/grasp_vision_frame.json` 更新，快照時資料約0.49秒前，seen=false僅代表未見目標。
這是手動恢復實測，非自動修復監督器驗收，也未驗證辨識/夾取精度。

systemd磁碟設定與已載入設定有差異；本次沒有daemon-reload或改寫組員配置。
已載入vision含`--allow-top-clipped-grasp-home`及`--frame-report`，未用本機舊參數覆寫。

### 手臂動作期間的共同Serial資料

首次ROSbridge前檢因scan時間過期而未動作；第一次原生home請求因maintenance被明確拒絕，亦未動作。
之後確認停止與唯一owner，解鎖後依序home→stow，完成後重新maintenance-lock。
home RPC約3.50秒，stow約3.87秒；兩次均返回ok=true，stow holding=false。
Serial owner前後均PID30006，未開第二個Serial。

直接ROS觀測結果：

| 動作期間 | odom筆數 | odom最大更新間隔 | odom TF筆數 | TF最大更新間隔 |
|---|---:|---:|---:|---:|
| home | 70 | 58.8ms | 70 | 61.4ms |
| stow | 77 | 63.7ms | 77 | 64.7ms |

原始scan與濾波scan均持續約10Hz，最大時間戳年齡約0.14秒。
證實本輪姿態動作不阻斷輪回傳/odom/TF；未測夾取負載，也不表示所有Serial故障已排除。

## 找到並處理的誤停來源

新增的背景記錄器原先透過rosbridge訂閱raw/filtered scan、odom與TF，完整scan記錄CPU約41%。
降低scan採樣仍會影響導航客戶端資料交付；TF整topic節流還會漏掉混合訊息內的odom TF。
因此不把節流監測的間隔當成真實發布間隔，也不放寬導航資料過期門檻。

| 對照 | 導航檢查樣本 | 過期樣本 | 最大scan時間戳年齡 |
|---|---:|---:|---:|
| ROS背景記錄同時運行 | 125 | 70 | 0.602s |
| 停止ROS背景記錄，相機持續運行 | 104 | 0 | 0.360s |
| 改RAM-only記錄，相機持續運行 | 109 | 0 | 0.373s |

另外直接ROS 8秒量測：raw及filtered各80幀，最大年齡分別0.111s與0.145s。
對照支持「新增診斷訂閱影響rosbridge交付」；尚未將原因細分為橋接節流機制或CPU排程，
不宣稱LiDAR driver故障或根因已完全查明。

`integration/g2_baseline_recorder.py` 已改成預設只記RAM與主要程序RSS，不訂閱ROS、不發動作或修復。
`--ros-topics`僅供隔離診斷，不能與導航客戶端一起當作正式背景監控。

## RAM記錄正在進行

最終執行單元：`x3plus-baseline-ram-20261016.service`（systemd transient，非新增開機常駐）。
程式最多1800秒，外層RuntimeMaxSec=1900；每2秒採樣，日誌1MB輪替1份，上限約2MB。
資料：`/home/jetson/demo_stability_20261016/baseline_ram.jsonl`；
完成/終止後產生`baseline_ram.summary.json`，用complete欄位區分滿30分鐘或提前停止。
初期61秒31筆最低MemAvailable約1754.6MB，grasp-service RSS約507.4MB不變，vision約297–302MB。
這不是30分鐘完成或無洩漏結論；取回的本機log僅是當時快照，後續需再下載完整結果。
先前三個ROS記錄實驗unit已停止，對應summary complete=false，不計入30分鐘驗收。

## 測試與仍待處理

- 記錄器/恢復/demo/audit/supervisor：95通過、1略過。
- 五組既有回歸：controller138、servo37、floor641、UI50（略過1）、mission selftest通過。
  controller/floor使用僅測試程序的相對URDF路徑轉接，中文絕對路徑問題未修復。
- 實機無`/tmp/grasp_health.sock`，新版ResidentBackend/monitor需要該獨立健康介面；
  目前不能直接啟用新監督器或以舊main RPC的忙碌/超時判定owner當機。
- 此輪未覆寫grasp-service核心、未啟用自動owner重啟，原點未重設。
- 記憶體分級/持續時間/恢復門檻仍待實作部署，未透過塞滿實機RAM做測試。
- 後續先核對完整RAM結果，再對齊實機與候選健康介面，保留組員校正與已載入參數，
  最後才測有限自動修復及最小夾取/投放展示。需要擺放目標時再請現場配合。

相關JSON在`docs/calibration/odom_new_floor_2026-10-16/`，包含
`arm_shared_serial_native2.json`、`arm_native_samples2.json`、`native_scan_latency_vision_on.json`、
`rosbridge_nav_latency.json`、`rosbridge_nav_no_recorder.json`、`rosbridge_nav_ram_recorder.json`及
`stability_current_snapshot.json`。
