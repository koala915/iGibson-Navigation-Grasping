# Sugarbox PPO 重訓交接：固定巡航姿態

> **已依 PR #10 review 更新：本頁連結的原始碼/ZIP 是歷史快照，不是現行部署版本。請先讀 [GitHub 現行基線與最新重訓決策](SUGARBOX_PPO_CURRENT_BASELINE_2026-10-17.md)。後續以 PR #9 的 `codex/approach-lidar-frame`（本次核對 `eafba63`）為起點，不覆寫回本快照。第一版不預設把方向保持/自動後退救援放進訓練。**

2026-10-17，enoch20050427 的獨立實機接近版本。這次只新增交接快照，不覆寫團隊主線控制器，不合併任務流程，也不啟動實機或訓練。

## 從這裡開始

- [交接包 README](../../integration/sugarbox_local/20261017_retraining_handoff/README.md)
- [完整重訓需求](../../integration/sugarbox_local/20261017_retraining_handoff/PPO_RETRAINING_HANDOFF_2026-10-17.md)
- [機器可讀的契約對齊表](../../integration/sugarbox_local/20261017_retraining_handoff/contract_alignment.json)
- [下載固定巡航版 ZIP](packages/sugarbox_ppo_retraining_handoff_20261017_cruise.zip)

ZIP SHA256 見同目錄 `packages/sugarbox_ppo_retraining_handoff_20261017_cruise.zip.sha256`；快照 README 已補上歷史警告，不能使用早期留言的舊雜湊。

## 最新決定

接近、影像對準與相機交接維持巡航／循線姿態 `(90,140,0,0,90,30)`，不再做朝前、循線、向下的三段式切換。錄影入口只初始化一次巡航姿態，交接時讀回角度、切換相機預覽，不再發送視角切換或夾取命令。

這不修改團隊夾取服務自身的姿態或安全控制。非巡航角度的舊 pixel 校正不能直接沿用；最終可夾位置尚未完成校準。

## 為什麼不只有 final2.py

導航主程式仍是 `sugarbox_rl_approach_final2.py`。目前另需 `sugarbox_training_contract.py` 的訓練契約，以及小型 `sugarbox_navigation_handoff.py` 的純交接判斷。後者從手臂 pixel 模組分離，避免訓練端為了匯入導航而攜帶整套手臂三段式程式。

包內共 13 個檔案；除上述三支導航程式外，其餘為選讀錄影入口、三個離線測試檔、舊訓練設定與交接/雜湊文件。沒有模型權重、VecNormalize 本體、原始 log、影像、憑證或場景資料。

## 驗證與限制

- 獨立本機回歸測試：160 項通過，包含固定姿態、拒絕舊 staged profile 與導航依賴檢查。
- 發布的 ZIP/原始碼快照另跑所附離線子集；結果見 PR 說明。完整本機測試與交接子集不同，不混用數量。
- 尚未實機重測固定巡航版本；沒有啟動 PPO 訓練或重新部署 Jetson。
- 現行 training-contract 模式跳過 anti-jitter 的旗標問題、0.45 m 診斷停止及交接淨空限制，已在交接文件標示，這次沒有順便修改它們。

請組員 Codex 先核對環境、訓練 commit、55D/2D 舊模型配對、新過濾器狀態/維度與動作/感測契約，再提出實驗順序。重訓不是直接執行本包的實機錄影程式。
