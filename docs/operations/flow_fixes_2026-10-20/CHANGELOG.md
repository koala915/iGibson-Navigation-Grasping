# 巡航／接近／夾取／投放流程修正（2026-10-20）

本次在既有未提交版本上修正跨階段問題。修改前快照在 `before/`，其 SHA256 在
`baseline.json`；`initial_worktree.patch` 是接手時已存在的修改，不是本次成果。
`fixes.patch` 僅比較本次快照與完成版，避免把 Claude 先前的修改混算進來。

沒有部署、重啟車上服務、連線機器人或執行實機動作。下列結果是離線驗證。

## 第 3 輪反覆前進／後退的證據與處理

使用者確認問題發生於 10/19 第 3 輪。來源為
`logs/mission_20261019_patrol3.out`：

| 接近次數 | 停止原因與估計距離 | 總控隨後退出的里程計距離 |
|---|---|---|
| 1 | `SEED_NOT_CONFIRMED`，0.78 m | −0.220 m |
| 2 | 同上，0.79 m | −0.226 m |
| 3 | 同上，0.80 m | −0.212 m |
| 4 | 同上，0.77 m | −0.217 m |

第一次推論約 2.1–2.4 s，舊流程在相機成功量到前已朝巡航估計位置前進並放棄；總控
將接近端的 exit 2／已知空爪當成「夾空」，執行 `room.leave()` 後續巡。這四次約 20 cm
退出有明確總控紀錄，不能只憑啟動時印出的 hard-stop reverse 設定就歸因為避障倒車。
使用者回報曾撞盒；log 可以證明危險的失敗續巡循環，沒有接觸感測證據可判定撞擊瞬間。

既有尚未上車的 SeedLook 修正（停著看、取得新辨識才前進）予以保留。本次額外封住
總控誤分類：**只有接近已回報到達、之後交接／夾取失敗的 exit 6，且手臂與障礙位置
可確認，才允許依原規則收臂退出。** 其他失敗包含未確認 seed、預檢失敗、逾時、異常
結束，原地停止，不自動後退、重試或重新巡航。未識別的階段結果也停止。

## 本次修改

| 問題 | 修改後行為 | 主要檔案 |
|---|---|---|
| 接近尚未成功就被當夾空退出續巡 | exit 6 為唯一已到達後的空夾恢復入口；其餘停止。連到達後失敗若連障礙位置都無法確認也不恢復巡航 | `integration/mission_loop.py` |
| 1 m 的未確認盒子被略過，累積為淨空影格 | 所有未確認位置保留避撞記憶；略過／失敗盒子仍在前方走道可見時不給許可，不因距離大於 0.90 m 就放行；近距離記憶障礙也擋車 | 同上 |
| 移動中的舊影格用新位姿排除成牆邊 | 新增位姿歷史，只有涵蓋影格可能拍攝至推論完成且近乎靜止才准地圖排除；欠缺證據先停車重看。過期、非有限數與錯誤定位清空快取；收到定位不重置其來源年齡 | 同上 |
| 投放失敗環視直接繞過淨空檢查 | `turn_if_clear()` 共用入口，每段環視重新查定位與明確 `turn_clear=True`；拒絕時仍持物停止 | 上述總控、`integration/sugarbox_approach_grasp.py` |
| 總控指定的校正檔／解碼器未傳入子程序 | 夾取、投放、重試與恢復都傳入同一份 `--homography`／`--gstreamer` 絕對路徑 | `integration/mission_loop.py` |
| 持物恢復投放跳過維護鎖交接 | `--no-patrol --resume-at place` 進階段前也走既有 unlock；服務拒絕不啟動動作 | 同上 |
| client 缺少 health／release-preview；無法確認服務協定 | 增加兩指令、health 獨立 socket 與 2 s timeout、嚴格布林成功回應；總控／單獨接近預檢實際 status＋health 的 G2 欄位，不憑空補值 | `grasp/v23/graspctl.py`、接近 runner、總控 |

沒有更動 PPO 權重、動作縮放、手臂控制器或底層安全閘。新的停止條件刻意保守：AMCL
抖動、位姿歷史不足或新影格確認失敗可能使任務提早停住，不能為消除停車而直接放行。

`tests/test_flow_contracts.py` 新增 21 個跨階段案例。調整既有測試的服務 fixture 與失敗
分類；原先「正前方略過物體不擋車」的案例改成側邊位置，正前方則由新測試明確要求停止。
CI 補入總控、跨階段安全、seed、投放幾何、permit 五組原先未列入的套件。
`CLAUDE.md` 與決策文件同步記錄規格；原檢查報告及修改前重現證據保留。

## 驗證

- 針對性 7 組／264 個 unittest 案例通過，包含實際串接 `patrol_mission()` →
  `fetch_sugarbox()` 的 exit 2 回放：只接近一次、沒有 `room.leave()`、沒有第二次巡航，
  最後送停止。
- 完整離線回歸結果與來源雜湊收在 `final_results.json`，每組原始輸出在
  `targeted/`、`regression/`、`network/`。
- 最終為 **34 組全通過：566 個 unittest 案例、1,784 個自訂檢查、3 組 selftest**。
  自訂檢查含 v21 控制器 138／servo 37／floor guard 641，以及 v23 的 180／37／641、
  travel 51、chassis 59。首次 sandbox 內 loopback 被 WinError 10013 擋下的輸出保留，
  最終結果採同一份程式在准許本機通訊環境的通過結果，沒有為它修改產品程式。
- Python 靜態檢查涵蓋修改的控制流程與測試，沒有未定義名稱；client 另確認 Python 3.6
  語法相容。Windows 上 Unix socket client 以假 socket 測試，不代表已驗證真 socket 服務。
- 本機 loopback 的假底盤／UI 測試需要 sandbox 外執行；沒有連 Jetson。POSIX 專用
  SIGTERM／signal 案例在 Windows 有明列 skip，Linux CI 仍須跑。

重跑入口：`python docs/operations/flow_fixes_2026-10-20/run_suites.py targeted`、
`regression`、`network`。腳本只執行離線測試，不啟動實機任務。

## 車上仍需確認的條件

1. **車上服務與 checkout 不完全相同。** 保留 ncu 的 G2 常駐服務／maintenance／獨立
   health 實作；本次只補 client 與相容性拒絕檢查，沒有把舊 arm-only service 改造成完整
   G2。不可把 checkout 的整個 `grasp/v23/` 覆蓋上車，也不可直接覆蓋車上功能較多的
   `graspctl.py`（其中另有 maintenance 指令）；挑選必要變更並先核對實際服務。
2. 地圖排除仍依賴 AMCL 與 0.30 s 串流延遲上限的假設，沒有真正來源時間同步。須量測
   實際延遲與轉向後定位品質。新邏輯寧可停住，仍不能補救完全錯誤但自稱正常的定位。
3. 實車先確認 seed 未量到時全程停止、前方 1 m 已略過盒子不重新巡航、靠牆持物不環視；
   再驗短距接近、到達後 E1／夾取、投放及續巡，保留影像、停止距離及 log。
4. 盒子低於 LiDAR 且近距離可能被夾爪遮住，軟體離線通過不能代替現場碰撞／視野驗收。
   本次不把任何 `hardware_validated` 狀態改為通過。

回復本次修改時依 `fixes.patch`／`before/` 精準處理，先確認後續是否已有新修改；不要
`git reset --hard`，那會一併刪除接手前 Claude／使用者尚未提交的工作。
