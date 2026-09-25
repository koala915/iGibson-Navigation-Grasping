# v24 E1 訓練交付接收審查

日期：2026-09-07。交付來源：
`C:/Users/user/Downloads/v24_e1_replacement_seed23404_r3_COMPLETE/v24_e1_replacement_seed23404_r3`

判定：**已接收為待部署驗證候選包；尚不可取代正式 v21，也不可繼承 v23 的真機成功紀錄。**
初次審查沒有修改下載包、執行交付 Python、反序列化 pickle、推論或重跑模擬；其後把正式
pair 與必要原始證據複製到 `grasp/v24/`，原下載包仍未改動。
[驗證腳本](verify_intake.py) 與 [機器可讀結果](verification.json) 保存此次核對方法與結果。

## 已確認的交付內容

- 共 56 個檔案；SHA256SUMS 列出 55 個，全部雜湊吻合。只有 checksum 清單自身不在清單中。
- 正式 pair：
  - model/v24_e1_model.zip：`0ed9989154ed280971fd4499c8927c198539f37915f6d9191f9cd08eaf100d81`
  - model/v24_e1_vecnormalize.pkl：`6ebb5a64ef967a1fcfca522fe330f71e0eef3f95b452c78a8248b4edb74ecd26`
- 選定 targeted DAgger BC update 10；不是把檔名中的 v24 當成新的 action contract。
- ZIP 的 JSON metadata 是 observation 28D、action 6D；training metadata 指定
  obs_28_incremental / 6D_arm_incremental_gripper_absolute。
- deploy_contract.py 與本 repo grasp/v23/deploy_contract.py 在 LF 正規化後完全相同；
  交付 URDF 與 grasp/x3plus/yahboomcar.urdf 同樣完全相同。
- E1 home：sim rad [0, -0.275, -1.42, -1.42, 0]，與 v23 相同；
  C3 的 v21 不是此權重的起始姿態。
- 工作區 x=[0.205,0.280]、y=[-0.070,0.065] m，與現行 v23 envelope 相符。
- 模型、normalizer、protocol、training metadata、selection lock、formal claim、registry、
  formal evaluator 與 URDF 的內部 hash 引用一致；9 個 training source snapshot 都吻合。
  此為包內一致性驗證，不是外部簽章或執行歷史的獨立證明。
- 訓練時與正式評估時的 eval_v23_e1_grasp.py 不同；包內保留兩份。
  formal_evaluator_source 的 hash 確實吻合 formal evaluation，重播不可誤用 source_bundle 那份。
  finalize_v24_e1_selection.py 是後處理來源，列於 SHA256SUMS，
  並非 formal integrity_start 所聲稱的 episode evaluator。

## 正式結果重算

從 JSON 的逐回合 rows 重算，沒有只採用報告摘要：

| 類別 | 成功 / 回合 |
|---|---|
| Bottle cap | 25 / 25 |
| Mixed heights/shapes | 75 / 75 |
| 9 格 grid | 130 / 135 |
| 合計 | 230 / 235（97.87%） |
| 最差 grid | 13 / 15（86.67%） |
| 舊 hotspot (0.280,-0.070) | 15 / 15 |

235 個 case seed 全部不同；穿地回合 0、記錄的 guard interventions 0。
失敗分布：左近角 (0.205,-0.070) sphere 2 次；
(0.2425,+0.065) cylinder 1 次；右遠角 (0.280,+0.065) cylinder 2 次。
這些是交付模擬紀錄重算，**不是本機重新執行得到的成功率，也不含相機或實體伺服閉迴路**。

## 接入前問題與限制

| ID | Priority | 證據與影響 | 最小處理方式 |
|---|---|---|---|
| V24-01 | P1 / provenance | preregistration.created_at=13:10:00+08:00；training started_at_utc=05:09:22.071333+00:00，即早 37.929 秒。與 preregistered_before_training 標記不一致。 | 訓練端提供原始程序 log、檔案建立紀錄與時間欄位定義；保留原件，新增說明，不能回填時間來消除差異。這不直接否定 230/235 統計。 |
| V24-02 | P1 / environment | requirements.txt 鎖 SB3 2.1.0，metadata 與 ZIP system_info 是 2.2.1；另含 pywin32 與 D:/... 的本機 iGibson 路徑。 | 補實際 train/eval environment lock 與 iGibson commit/version。不得把這份 Windows freeze 直接裝到 Jetson。 |
| V24-03 | P1 / deployment parity | 97/235 回合記錄 min_gripper_z <8mm，最低 0.843mm；正式 gate 是不穿地。部署 guard 是 8mm，且還有全路徑 sweep、finger correction 等差異。 | 比對兩端量測 link/frame、finger geometry、8mm guard 與每步動作。不能判定這 97 回合必然被擋，也不能假定零穿地等於通過部署 guard。不可因此降低現有安全距離。 |
| V24-04 | P1 / controller parity | evaluator 使用 Runner→X3PlusRobotGraspEnv.step，包含 dual-side contact 後 magnet 邏輯與 scripted return；它不是現行 v23 GraspController。 | 補 deployment-equivalent replay／動作 trace，檢查 stage、jaw hold、contact、return 與 observation TCP 補償。需無 magnet 的獨立證據才可把模擬成功解讀為實體接觸保持能力。 |
| V24-05 | P1 / new-weight hardware | 交付 manifest 明載 new-weight physical validation PENDING。 | 新 hash 必須做 Jetson load、dry-run、固定座標、E1 視覺單點與完整 log 驗收，不能沿用舊 v23 3/3。 |
| V24-06 | P2 / reproducibility | source bundle 有 Python 與 URDF，但未附 meshes/YCB/iGibson 本機來源；不是完全獨立的 simulator 重播包。 | 明列外部資產版本、取得方式與 hash；相同 URDF 可復用 repo mesh，但不代表完整 iGibson 環境已具備。 |
| V24-07 | P1 / integration scope | 包內沒有 ROS bringup、motor runtime、回授封包 sequence、navigation map/route 或 mission final homography adapter。 | 原 audit 的 B01/B04/B05/B11/B13/B14/B17 等仍需處理；這份是夾取訓練交付，不是整套機器人整合包。 |

時間線的其餘記錄為 selection lock 13:24:13、formal claim 13:25:16、
formal result 13:26:39；包內順序合理。claim 來源碼以 exclusive create 在 Runner 建構前建立檔案，
但這不獨立證明外部曾未重跑 formal seed。

## 建議接入方式

1. 先把 v24 定位為 E1 候選。之後建立獨立 v24 artifact manifest，保留原交付 hash，
   不修改 v21/v23 manifest 來冒充已驗證版本。
2. 以本 repo 已修過 jaw hold、串口讀取、release gate 與 cleanup 的 v23 controller 為基底；
   不導入訓練 env 取代硬體 runtime。
3. profile 必須同時鎖定 model/VecNormalize、E1 home、incremental contract、URDF、
   相機 calibration identity 與補償參數。v23 的 target +5mm、TCP +20mm、finger +15mm
   是既有實機 tuple，對新 hash 需要重新確認，不能由相同 shape 判定等價。
4. 已建立 `grasp/v24/run_candidate.py` 作為鎖檔候選 launcher；它重用 v23 controller，
   並鎖定 model/VecNormalize/contract/manifest。候選 release gate 仍會拒絕一般 `--real`。
   下一步是 Jetson 離線載入、normalization/finiteness、controller parity，再依實機規則逐項測試。
5. 通過新 hash 的真機驗收後，再安排 mission 的 E1/home/homography 切換；
   完整任務的 real 封鎖不因收到此包而解除。

## 給訓練端的待補資料

- 解釋 preregistration 比 training start 晚 37.929 秒的原始證據。
- 實際 SB3 2.2.1 環境 lock、iGibson 版本／commit 與資產清單。
- selected update 10 的 action/observation trace，特別是 5 個失敗回合與最低 clearance 回合。
- 以部署端 8mm guard、E1 runtime 補償與 jaw contact/hold 規則重播的 parity 結果；
  說明 magnet 在 formal episode 中的使用範圍。現有 JSON 未保留逐步 magnet_active。
