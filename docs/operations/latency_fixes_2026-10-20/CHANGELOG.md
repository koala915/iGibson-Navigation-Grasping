# 流程等待時間優化（2026-10-20）

本次在上一輪安全修正上增加資源常駐與有條件的 E1 提早接續。沒有連線機器人、部署、
重啟車上服務或執行實車移動。修改前快照 `before/`／雜湊 `baseline.json`，接手時的未提交
修改在 `initial_worktree.patch`；僅本次差異在 `changes.patch`。

## 已完成

1. **模型常駐，控制狀態逐段重建。** 新增 `integration/mission_runtime.py`。任務開始
   預檢兩種目標的模型／校正／PPO 契約，預載並暖機 YOLO（糖果盒／投放盒）、SAM2、PPO。
   巡航辨識也使用同一個模型程序，主程序不再另外載一份 YOLO。切換階段時只保留模型、
   推論用 normalization 與解碼影格；新的模組重新建立目標、seed、各段控制器、視覺
   sequence 與 ROS 里程歷史。禁止 inference-only normalization 被切回 training。
   模型檔大小／mtime 改變會拒絕重用，原有 checksum 預檢仍執行。
2. **後相機單一接收器。** 任務全程保留 GStreamer，用共享記憶體提供每個階段的影格；
   關閉階段的 reader 使用權不會關閉接收器。每次取得使用權都設定新時間界線，界線前的
   影格不可使用。保留同一筆接收時間，不因 IPC 重設新鮮度；每次複製一致快照，不會把
   正在覆寫的影像交給辨識。解碼管線加入只留最新影格的 queue。
3. **常駐狀態／指令通道。** 新增 `integration/robot_rpc.py`，一條 SSH 連線轉送既有
   grasp／health Unix sockets。只接受原有 allowlist，逐筆請求有 ID 與 timeout；讀取或
   動作失敗、回應不符、夾取未確認時關閉通道，**不重新連線重送動作**。沒有啟動第二個
   序列埠持有者；TCP stop 仍有獨立入口，不受該通道阻塞。
4. **E1 條件式提早接續。** receiver 新增原子讀取 position＋receipt；service 增加可選
   metadata，home 回覆能力欄位。runner 只在明確支援時每 0.20 s 查詢，兩筆不同且有效的
   新辨識一致便要求夾取；同一 receipt 不可算兩次，時鐘倒退／停止 epoch 改變會停止。
   第一筆必須晚於停止起點＋既有 1.5 s frame-lag 上限，服務端的 2.5 s grasp settle
   閘與原有姿態／底盤／幾何／地板保護都保留。微調前仍保留舊的 2.5 s 等待；快速模式的
   frame report 必須有不同時間戳，不能用同一張空白 report 累積盲區前進次數。
5. **ROS 生命週期。** 原程式每段 `terminate()` 會終止共用的 Twisted reactor，常駐
   程序無法為下一段重啟。現在常駐模式只解除各段訂閱／斷線，獨立模式保留原行為。
   馬達 socket 仍每段關閉。視覺執行緒最長等 10 s 結束，未結束則標記 poisoned 並終止
   worker，不能讓下一段與前一段同時使用模型。
6. **可追查的時間紀錄。** 每次任務輸出 `timings.jsonl`，包含一次暖機、各段總時間／
   exit code、接近耗時、常駐服務指令耗時與 E1 確認耗時。引擎完整輸出保留在配對的
   `*.engine.log`，也轉印進階段 log／終端。可用 `--isolated-stages` 選原先獨立模式。

正常總控預設啟用常駐；直接單獨跑 `sugarbox_approach_grasp.py` 保留原本獨立模式。
無 `--real` 的預檢不會啟動模型 worker、相機或動作。暖機失敗時不開始巡航；中斷、逾時、
worker 異常或錯誤 generation 都終止任務，沒有自動重播與後退續巡。

## 驗證與實際速度證據

- 新增 `tests/test_mission_runtime.py`，涵蓋跨程序常駐／重用、來源影格時間保留、舊影格
  拒絕、單一 reader／控制使用權、異常後禁止重用、逾時終止、ROS reactor、SSH 不重送、
  E1 重複讀數／停止 epoch／微調 report、總控接線與暖機失敗不啟動巡航。已加入 CI。
  共 27 個新案例；包含 worker 被終止後共享鎖無法取得時，必須有限時失敗，不能讓
  接收器永遠卡住。最終 **35 組離線套件通過：593 個 unittest 案例、1,784 個自訂檢查、
  3 組 selftest**；Windows 不支援的 POSIX 結束訊號案例有明列 skip。
- 使用 `.venv-approach` 的真實模型執行 `model_smoke.py`，沒有相機、ROS 或馬達連線。
  同一個 worker 完成糖果盒／投放盒／糖果盒／投放盒四次辨識＋拒絕 go 測試，全部 exit 5，
  模型保留、控制程式每次重新進入；初次預載約 **45.9 s**，後續四次約 **0.078–0.110 s**。
  這是本機空白圖 YOLO＋引擎準備／拒絕開始的測量，**不包含實際 SAM2 目標分割、ROS
  連線、影像確認、行走或夾取，也不是實車完整切換時間**。目的為確認不再反覆冷載入。
  初始準備有一次成本，實車效益要依循環次數量測。
- 完整回歸與每個來源雜湊在 `final_results.json`，個別輸出在 `targeted/`、
  `regression_final/`、`network/`；首次執行的 `regression/` 保留供追查。
  靜態檢查在 `static_checks.txt`。Windows 的 POSIX signal
  案例明列跳過，本機 loopback 若被 sandbox 擋下則在僅開放本機通訊的環境重跑。

重跑：`python docs/operations/latency_fixes_2026-10-20/run_suites.py targeted`、
`regression`、`network`。真模型測試使用 `.venv-approach/Scripts/python.exe`
執行同目錄的 `model_smoke.py`，不啟動相機或連 ROS。

## 保留與尚需車上確認

- **沒有降低辨識門檻、淨空距離、停穩條件、permit 時效或更改 PPO 權重。** 上一輪修正的
  pre-arrival 失敗停止、略過物體仍避撞及投放轉向淨空全部保留。一般接近失敗仍不自動重試
  動作；目前沒有足夠的來源時間／障礙證據可安全放行，不能靠少停車把這層保護拿掉。
- **E1 車端 metadata 是相容性擴充。** 只選擇性套用 `snapshot_with_receipt()`、service
  status 欄位與能力回覆到車上 ncu G2 分支，不可把 checkout 的舊 arm-only service／整個
  v23 目錄覆蓋上車。未套用時 runner 自動保留舊 2.5 s＋1.1 s 等待，其他常駐優化仍可用。
- receipt 表示收到辨識的時間，**不是相機拍攝來源時間**。仍沿用現有 frame-lag 上限；
  後相機的來源 timestamp／AMCL 同步尚待車端提供，所以巡航移動影格的保守排除規則
  保留。不能把這次改動宣稱為完整拍攝時間同步。
- 常駐兩套 YOLO＋SAM2＋PPO 的記憶體占用、長時間 CUDA 穩定性、ROS 反覆訂閱，以及
  車上 arm／chassis／maintenance 交接仍須實測；先空載檢查、短距離單段，再做多輪。
  需要保留相機、服務、ROS 與控制板健康紀錄；離線測試未更新任何 hardware_validated。

回復本次變更應依 `changes.patch`／`before/` 選擇性處理，保留上一輪與 Claude 已有修改，
不要清空整個工作目錄。
