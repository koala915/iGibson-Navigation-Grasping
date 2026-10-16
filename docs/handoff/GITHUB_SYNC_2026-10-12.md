# GitHub 成果核對（2026-10-12）

此為10/12快照；10/17已重新fetch，koala接近分支更新為d06bba3，自己的成果與決賽主序亦有更新。
最新核對見[10/17決賽主線](../planning/V23_FINAL_DEMO_REVIEW_2026-10-17.md)，不要只依本頁舊head判定現況。

機器人未開機／連線。本次只讀 GitHub、fetch 遠端參照、讀取交接及模擬合併；
沒有實機連線、部署、移動、實際合併或推送。現有本機交付計畫完整保留。

## 已抓取的版本

| 來源／分支 | commit | 成果與界線 |
|---|---|---|
| 112303577ncu `v23-grasp-test` | `e980fa8` | 本機 HEAD 與 origin 相同；20 小時計畫及入口連結仍為未提交文件 |
| koala915 `v23-grasp-test` | `3eb79ec` | PR #8 已合入，包含 rear homography、視覺接近與 E1 交接 |
| koala915 `codex/approach-lidar-frame` | `560e2f3` | PR #9 尚未合入，修正接近 LiDAR 幾何與以實際編碼器初始化手臂 |
| enoch20050427 `codex/sugarbox-local-20261011` | `ebad46e` | 10/11 最新獨立快照；以 koala v23 `3eb79ec` 為基底，35 個新增檔案、13,050 行 |
| enoch20050427 `v23-grasp-test` | `fecb0a3` | 舊分支；不能只 fetch 這個分支就宣稱已取得他的最新成果 |

遠端參照分別存為 `origin/*`、`koala/*`、`enoch/*`。目前工作分支仍是自己的
`v23-grasp-test`，沒有將遠端成果直接套入正式入口。
自己的 v23 相對 koala v23 為 ahead 14／behind 7；仍未統一。
對 koala v23、PR #9 分支、enoch 最新快照分別執行 `git merge-tree --write-tree`，
三者皆 exit 0，沒有文字衝突；這不代表三者一起整合後已通過功能測試。

## enoch 新成果

依其 [交接文件](https://github.com/enoch20050427/iGibson-Navigation-Grasping/blob/codex/sugarbox-local-20261011/docs/handoff/SUGARBOX_LOCAL_2026-10-11.md)：

- PPO 訓練契約適配、LiDAR 點轉至底盤中心；雷射偏移 0.09 m、目標原點偏移 0.135 m。
- 零額外 action delay、固定第一次目標鎖定，單次人工觀察前後震盪減少；尚非重複成功證據。
- rear camera → arm camera 的交接、像素接近與分階段控制、姿態讀回及診斷工具。
- forward／travel／down20 三個 S2 姿態實測讀回；沒有完成完整自動夾取。
- 148 項獨立測試，作者記錄全過；本次未重跑，不當作本機新增通過數。
- `test_turn_filter.py` 驗證的是轉向指令換向確認／抑制抖動，
  不是跨 LiDAR 幀確認障礙物的濾波；不能因此宣稱短暫掃描雜訊已排除。

## 必須保留的問題

1. **公開 push 並非完整部署包。** 模型、原始 runtime logs、相機影像與產生的證據 JSON
   未包含在快照；Windows 絕對模型路徑也保留。正式整合前要補資產位置、hash 與環境設定。
2. profile templates 仍 `confirmed=false`；可夾取 pixel goal、控制符號與前方停車校正未完成。
3. PPO → travel camera → down20 camera → 自動 grasp 全鏈尚未執行；不能直接當正式 demo。
4. 快照有 standalone motor server 候選，不能和常駐 grasp-service 同時開 Serial。
   整合沿用常駐唯一持有者，不直接啟動候選 server。
5. 新快照的雷射偏移 0.09 m 與目前 G2 0.10 m 不同；須核對座標契約與量測，不能直接取代。
6. 作者也記錄中文绝對 URDF 路徑失敗，以 runtime-only 相對 loader 完成離線測試；
   production 路徑問題仍未修復，與本機測得的問題相符。

## 20 小時計畫調整

保留原總時數。版本整合 2 小時先納入已合入上游的 PR #8，審查 PR #9，
再選取 enoch 快照中可共用的工具／契約／測試；不把全部 standalone 流程換成正式入口。
LiDAR 3 小時先比對現有群聚、enoch 的 footprint／換向邏輯與已記錄資料，避免重寫已有功能。
完整展示 4 小時先打通最小接近到夾取；未確認的像素控制不占用正式交付主線。
開機後才進行實機基線、部署及共同 Serial 驗證。

來源：[PR #8](https://github.com/koala915/iGibson-Navigation-Grasping/pull/8)、
[PR #9](https://github.com/koala915/iGibson-Navigation-Grasping/pull/9)、
[enoch 最新 commit](https://github.com/enoch20050427/iGibson-Navigation-Grasping/commit/ebad46e2b3f0d2beb5cb5d6daaa5874ee3557cc3)。
