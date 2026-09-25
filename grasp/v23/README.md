# grasp/v23 — E1 高姿態夾取（測試中，**尚未合併**）

這是 v23 的實機測試包。跟 v21 唯一的實質差別是**手臂起始姿勢**與**權重**：

| | v21 | v23 |
|---|---|---|
| grasp home | C3 = API (90, **67.08**, **9.79**, **9.79**, 90, 30) | E1 = API (90, **74.2**, **8.6**, **8.6**, 90, 30) |
| 權重 | `candidate_v21_seed816_ckpt550000` | `candidate_v23_seed23401_ckpt250000` |
| 相機高度（training frame） | 0.2183 m | 0.2340 m（+1.58 cm）|
| gripper_center（張爪） | 0.1439 m | 0.1558 m（+1.19 cm）|
| 光軸離鉛垂 | 3.34°（偏後）| 1.40°（**偏前**，過了鉛垂線）|
| 可用 FOV | ~70 cm² | ~101 cm²（+44%）|
| 訓練 spawn band | x 0.20–0.33、y ±0.10 | x 0.205–0.280、y −0.070–+0.065 |

契約沒變（`obs_28_incremental`、28D/6D、incremental arm + absolute gripper），
`deploy_contract.py`、`action_execution_v21.py`、URDF 都與 v21 逐位元組相同。

---

## ⚠ 這個包的腳本不是交付包裡那一支

`grasp/deploy_v23/x3plus_real_grasp.py` **沒有**被採用。它跟
`grasp/v21/reference/x3plus_real_grasp.release_branch.py` 除了三個 hunk
（model 路徑、`home_deg`、`home_deg` 的註解）以外**逐位元組相同**，而那支在
`grasp/v21/reference/README.md` 裡標的是「**參考副本 — 不要執行**」：它在
2026-07-31 之前分支出去，缺少讓 v21 真的夾得起來的東西。

| 缺什麼 | 後果 |
|---|---|
| `_park_jaw_hold` / `jaw_contact` | 夾爪一路推到 180° 全閉停點並維持近堵轉扭矩（2026-07-31 實機聽得到齒輪研磨）|
| `_check_target_envelope` | 相機看得到、但策略沒訓練過的目標不會被擋 |
| 認 ID 的半雙工匯流排讀取 | 一個遲到封包讓整條讀取管線錯位 |
| manifest 發佈閘 | `--real` 不再驗權重 sha256 |
| 12 個 CLI 旗標 | 其中 `--latch-obj`、`--i-confirm-external-frame`、`--unlock-candidate-real`、`--pose-tol-deg`、`--stale-timeout` 是 `jetson_one_command_grasp.py` 在傳的 —— 沒有它們，一鍵流程根本驅動不了 |

所以本目錄用的是 **v21 那支 3268 行的硬體版**，只改了 `home_deg`、model 路徑、
target envelope 三處。交付包留在 `grasp/deploy_v23/` 原封不動，當對照用。

如果之後要 v23 的 `--release-to-bin` / `--release-only`，請照 2026-08-04 移植
release 動作的做法把旗標搬過來，**不要**整支換回去。

---

## 上機順序

### 0. 離線前置檢查（不碰硬體）

```bash
cd grasp/v23 && ./jetson_verify.sh
```

要看到 **148 / 37 / 641 / 50 / 89** 一字不差，`wrist_z_offset = 0.0564`。

> `wrist_z_offset` 錨在**地板**不是手臂（`dc.hover_gripper_center_z`），所以
> C3→E1 這個值不變。它要是動了，代表變的不是姿勢。

### 1. 量 E1 的外參（**必做，沒有捷徑**）

C3 的校正檔在 E1 是錯的，這是幾何上的必然：相機升高 1.6 cm，光軸還跨過了鉛垂線。
兩個檔案都是合法 JSON、都過同一道 ≥6 點／<2 cm 閘、也都不記錄自己是在哪個姿勢量的，
**唯一**把它們分開的是檔名不同：

- v21 → `integration/grasp_home_homography.json`
- v23 → `integration/grasp_home_homography_e1.json`

```bash
source ~/grasp_venv/bin/activate
cd grasp/v23
python3 jetson_one_command_grasp.py --calibrate
```

手臂會走到 E1 並停在那裡不動，相機持續印
`[bridge][calibration] {"u": ..., "v": ..., ...}`。每擺一個位置就等它印一行，
把裡面的 `u,v` 跟你用尺量的 base `(x, y)` 記成一組。

**至少 6 組不共線**、覆蓋四周與中央，另外留 **2 組**不要拿去擬合當驗證點。
擺放位置要落在 v23 的 band 內：**x 0.205–0.280 m、y −0.070–+0.065 m**。

Ctrl+C 之後：

```bash
python3 ../../integration/grasp_home_homography.py \
  --points-json grasp_home_points_e1.json \
  --output ../../integration/grasp_home_homography_e1.json \
  --max-rmse-cm 1
```

擬合 RMSE 與兩個保留點的 x/y 誤差都要 ≤1 cm。驗收：

```bash
python3 jetson_one_command_grasp.py --check
```

> **為什麼 E1 只能走 homography。** `arm_cam_geometry` 的三角測距要除以光線與地面
> 夾角的正切；相機越接近鉛垂，回推的橫向偏移越是趨近 0、深度對 theta 誤差越敏感。
> C3 的 3.34° 已經在這個區間邊緣，E1 的 1.40° 更深。`vision_grasp_bridge` 在
> grasp-home 直接拒收非 homography 的映射，`--i-accept-predicted-extrinsics` 也繞不過。

### 2. 空跑

```bash
python3 x3plus_real_grasp.py \
  --model models/candidate_v23_seed23401_ckpt250000.zip \
  --vecnorm models/candidate_v23_seed23401_ckpt250000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.065 --obj-x 0.24 --obj-y 0.00 --obj-z 0.0325
```

### 3. 定點實機（先固定座標，還沒接視覺）

同上加 `--real --unlock-candidate-real`。人在旁邊、手放電源開關 ——
**網頁上的停止鈕不是急停，電源開關才是。**

manifest `status` 仍是 `candidate`；以 `manifest.json` 的 hardware gates 為準。
成功實抓完整 log 已於 2026-08-30 通過，且操作者在可辨識範圍內重複 **3/3 成功**；
motion envelope、Jetson dry-run 與左右掃描實測仍未通過，所以尚不能移除 candidate 閘。

### 4. 一鍵：辨識＋夾取

```bash
python3 jetson_one_command_grasp.py --check    # 不碰硬體
python3 jetson_one_command_grasp.py            # 正式
```

目前一鍵 launcher 會在 E1 homography 算完後，把最終目標沿 base `+X`（遠離機器人）
平移 **5 mm**。2026-08-30 第二次實抓確認 bridge 已送出 `x=0.2728`，其未補償值
`0.2678` 與前 4 cm 的尺量 `0.2687` 只差 0.9 mm，但真實夾爪仍落在近側；因此要求的
「再往前」不再扭曲已準確的物體座標，而由 controller 額外套用 TCP landing
correction。10 mm 與 15 mm 的實抓仍落在近側，最後提高到 **20 mm**；此設定搭配
15 mm finger correction 與 0.5 jaw tracking，在可辨識範圍內實測 3/3 成功。
policy 與 close gate 會把 FK TCP X 視為靠後 20 mm，要求手臂繼續往外；
floor/collision FK 則維持原值。

兩項都是可逆的 runtime correction，
`integration/grasp_home_homography_e1.json` 仍保留原始實測值；物體寬度、Y、Z 都不變，
平移後也仍須通過 v23 的 `x=0.205–0.280 m` policy envelope。

```bash
# 回到未補償的原始外參
python3 jetson_one_command_grasp.py \
  --grasp-forward-offset-mm 0 --tcp-forward-error-mm 0

# 若 +5 mm 實測仍偏近，只用小步幅增加；程式硬限制為 0–15 mm
python3 jetson_one_command_grasp.py --grasp-forward-offset-mm 7
```

目前正式 launcher 預設是「物體座標 +5 mm、TCP 再走 +20 mm」，合計約 +25 mm；
`floor-finger-error=15 mm` 與 `jaw-track-fraction=0.5` 也已成為正式一鍵預設。
三者就是 2026-08-30 的 3/3 實機成功組合，因此一般執行不再需要逐一帶旗標。
若要完全取消 TCP 修正可帶 0；TCP 修正硬限制仍為 0–20 mm。

同一參數也會傳給 `--three-pose-scan`，但在 S1 旋轉成最終 base frame **之後**才加
`+X`，所以三個視角使用一致的「遠離機器人」方向。所有 `--calibrate`／
`--scan-calibrate-pose` 輸出刻意不套用此補償，避免把 runtime correction 混進量測資料。

### 夾到後仍喀喀響：Stage 0 慢速接觸

2026-08-30 實抓 log 顯示喀喀聲不是發生在 scripted Stage 1，而是 policy 還停在
**Stage 0** 時已經開始關 S6：編碼器在 147–152° 間因物體滑動而抖動，舊的
「連續不動」計數每次都被歸零，最後一路推到 179°。原本的
`--jaw-track-fraction` 只接到 Stage 1，因此該次即使帶 `0.5` 也保護不到這條路徑。

現在同一個比例判定也接到 Stage 0。它比較「上一筆 S6 命令尚未走完的角度」與下一筆
編碼器實際前進量；連續兩筆低於指定比例才確認接觸，避免單筆 UART／量化抖動誤判。
確認後會立即把 S6 停在**接觸讀值 +1° hold bias**，鎖定該 hold 並進入 scripted lift，
policy 沒有下一拍重新夾緊的機會。正常跟得上命令的空夾仍繼續到 180°並判定失敗。

```bash
# 以下三項已是正式預設：floor=15 mm、jaw tracking=0.5、TCP=20 mm
python3 jetson_one_command_grasp.py
```

成功觸發時 log 應出現 `Stage 0→2  S6 slipping`，並列出 encoder 實走角度與前一筆
尚待走完的角度。`0` 可停用比例判定；0.5 已實測 3/3，若沒有新的硬體證據不要改動，
數值越高會越早把正常但稍慢的伺服判成接觸。

若 log 明確寫的是 **`bbox touches only the top frame edge`**，而左右與下緣都沒有碰框，
可以用以下受限例外：

```bash
python3 jetson_one_command_grasp.py --allow-top-clipped
```

這不是忽略所有 clipping。E1 homography 用的是 bbox **底邊中心**；只有上緣被裁掉時，
底邊中心與左右邊仍可量，所以此旗標只放行 `top` 單一邊界。只要同時碰到 left/right/bottom、
底邊中心落在校正凸包外，仍會 fail closed。校正凸包是用物體中心建立，因此 bbox 左右
輪廓可在中心仍位於凸包內時稍微伸出；左右端點只用於寬度估算，且各自離中心與總寬都受
6 cm 夾爪開口限制，不會成為手臂目標。校正模式永遠不接受 clipped bbox。

不要在改變 S2–S5 後直接套 E1 homography：相機裝在 `arm_link4`，俯仰一變外參就變。
但只改 S1 是可證明的例外：整支手臂與相機繞固定的垂直 `arm_joint1` 剛體旋轉，先用
E1 homography 得到參考 XY，再繞 training-frame `(0.118146, -0.003359)` 旋轉即可。
程式只允許這個 S1-only 特例；任何 S2–S5 差異都會在開硬體前被拒絕。

---

## 三姿態掃描 → 回 E1 夾取

`three_pose_scan.py` 已把搜尋與 PPO 分成兩個**不重疊的硬體所有權階段**：

1. 掃描器獨占相機與 `/dev/myserial`，先確認 E1，再走 LEFT／E1／RIGHT，S1 分別為
   70°／90°／110°；結束後再次回 E1。
2. 三姿態的 S2–S5 完全相同，共用一份實測 E1 homography；左右結果再繞 S1 軸旋轉。
3. S1 軸心來自 URDF joint origin 加 training-frame offset；FK 在 S1=60–120° 的最大
   平面殘差為 `1.3e-8 m`。這只證明模型，不代替真機背隙、軸垂直度與碰撞檢查。
4. 每姿態收 5 個穩定樣本，單姿態散布須 ≤8 mm。
5. 多個姿態看見時，base XY 須在 1 cm 內一致；只有一個姿態看見也可用。
6. 全部看不到、座標不一致、姿態未到位，一律不夾。
7. 掃描器 guarded move 回 E1 並由編碼器確認後才釋出唯一結果；PPO 永遠從 E1 開始。

把目前 E1 的 7 點校正中心凸包旋轉 ±20° 後，三視角聯集約為
`x=0.2248–0.2844 m、y=-0.1064–+0.0696 m`；runtime 仍裁回策略評估帶
`x=0.205–0.280、y=-0.070–+0.065 m`。這表示橫向可覆蓋整個 policy y 帶，但不會藉掃描
偷放寬策略沒驗證過的座標。

E1 homography 已完成；左右不重做 homography，但真機驗證尚未完成。人在電源旁先各走一次，
每個姿態至少放 2 個分散的尺量 base 點。輸出會包含旋轉後的 `predicted_x/y`：

```bash
python3 jetson_one_command_grasp.py --scan-calibrate-pose LEFT
# 比較 predicted_x/y 與尺量 base x/y；每軸誤差都必須 <= 1 cm

python3 jetson_one_command_grasp.py --scan-calibrate-pose RIGHT
```

Ctrl+C 只送掃描器一次 SIGINT，最多等 45 秒完成 guarded E1 回程。左右都確認動作無碰撞、
編碼器到位，且各個驗證點每軸誤差 ≤1 cm 後，才把 `three_pose_scan.json` 中 LEFT／RIGHT
的 `hardware_validated` 與 `yaw_mapping_validated` 改成 `true`。在此之前正式模式故意拒絕。

**2026-09-20：LEFT 第一次在真機上走完並夾取成功（單視角，仍不算驗證）。**
`graspscan.sh --accept-single-rotated-view` 下，LEFT 取得 `(+0.2569, +0.0388)`、
5 樣本散布 0.1 mm，E1／RIGHT 未見；回 E1 編碼器確認後釋出目標，PPO 41 步收斂到
`target_dist = 0.019 m`，回 home 後物體仍在爪中。逼近時 S1 由 90° 降到 71–74°，
方向與物體在左側一致，**所以 yaw 正負號在真機上沒有反向**。

但這次沒有尺量點、沒有 ≤1 cm 誤差數據，兩個旗標**維持 false**。另外值得盯的是
`pad_after_close = 62.9 mm` 對 `object_top = 65.0 mm`，只差 2.1 mm
（E1 正面那次是 41.6 mm）—— LEFT 的落點偏高、夾得較淺，補驗證時要一起看。

三視角的用意是**擴大可見範圍**，不是讓視角互相驗算，所以
`--accept-single-rotated-view` 是常態用法而非捷徑：E1 正面單獨能用的區域只有約
3 cm 縱深（見下節）。

```bash
python3 three_pose_scan.py --check
python3 jetson_one_command_grasp.py --three-pose-scan --check
python3 jetson_one_command_grasp.py --three-pose-scan --allow-top-clipped
```

`--three-pose-scan` 不改變 PPO 的 grasp-home：策略仍從 E1 開始，因此不用重訓。
若改動 S1 以外的關節，這個共用映射立即失效，必須另量 homography；不能把
`yaw_mapping_validated` 留成 true。S1 超過配置允許的 ±25° 也會被拒絕。

---

## 常駐服務：把 23 秒的啟動成本只付一次

一次一鍵夾取量到 42.3 秒，其中**手臂只動了 8.1 秒**：啟動與載模型 23.3 秒
（PyBullet 載 URDF 就佔 11.5 秒、670 MB），視覺 10.5 秒。常駐之後單次夾取 **8.5 秒**。

| 檔案 | 用途 |
|------|------|
| `grasp_service.py` | 常駐夾取服務。用 launcher 自己的 `build_ctrl_cmd()` 組參數，所有安全閘照跑；`GraspController.run` 換成 serve loop |
| `graspctl.py` | 指令列用戶端（`grasp` / `status` / `quit`），系統 python 3.6.9 可跑 |
| `graspscan.sh` | 三姿態後備：停服務 → 跑掃描流程 → `trap EXIT` 還原服務 |

```bash
python3 graspctl.py status           # 服務狀態、伺服角度、目前偵測是否新鮮
python3 graspctl.py grasp            # 一次夾取，約 8.5 秒
sudo systemctl stop grasp-vision grasp-service   # 要用 move_arm/bus_probe 前必須先停
```

開機由 systemd 帶起：`grasp-service.service`（`Wants/After=dev-myserial.device`）與
`grasp-vision.service`（`BindsTo` 前者）。視覺端就是原本的
`vision_grasp_bridge.py`，只是拿掉 `--once`、加上 `--imgsz 320 --rate 1.0`。

**E1 優先，掃描是後備。** 服務收到 `grasp` 時先看有沒有新鮮的 E1 偵測，有就直接夾；
沒有就在 3 秒內拒絕且**不動手臂**，並提示改用 `graspscan.sh`。掃描器獨占相機與
`/dev/myserial` 且在 PPO 啟動前就退出，無法與常駐服務並存，所以後備會付完整啟動成本。

⚠ **服務活著時它獨占 `/dev/myserial` 和相機**，`move_arm.py`、`bus_probe.py`、
`jetson_one_command_grasp.py` 都會開不了，症狀看起來像裝置壞掉。先 `systemctl stop`。

⚠ 記憶體：夾取服務 985 MB、視覺 301 MB。ROS 全套另外 569 MB，三者同時在還剩約 1 GB。

## 姿勢還沒定案 — `pose_explorer.py`

E1 的實測結果是：6.5 cm 的 sugarbox 只能擺在**前 2.0–5.5、左 2.0–右 5.5 cm**
（約 26 cm²），而策略的訓練 band 是 101 cm²。視覺最多只能餵給它三分之一。

這不是校正沒做好，是姿勢的幾何。盒子 6.5 cm 高、相機 23 cm 高幾乎垂直往下看，
盒子**頂面**的投影比底面遠 1.4 倍，所以 `bbox_touches_border` 會在接地點還離
地面視野邊界很遠時就先拒收。**物體高度是這裡最大的變數**，不是相機解析度。

```bash
python3 pose_explorer.py --list                    # 只印候選表，不碰硬體
python3 pose_explorer.py --check                   # 檢查相機/序列埠/借用契約
python3 pose_explorer.py --i-am-beside-the-robot   # 實際走一遍
```

相機**整場只開一次**，手臂在候選姿勢之間移動，每個姿勢存一張標註過的快照到
`~/pose_explorer/`（`--show` 可另開即時視窗，需要顯示器）。快照上疊的是
**預測**的地面格線（2 cm 一格）與光軸十字 —— 光軸在 u=212 而不是 320，這就是
為什麼往右的可用範圍是往左的兩倍。

`k` 記下你看上的姿勢，`q` 離開時會回 E1 並把清單寫進 `~/pose_explorer/session.json`。
把那份清單給訓練端當新的 grasp home。

⚠ **表格裡的面積是預測值，會高估。** E1 預測遠端到 x=0.305，實測前 6.0（x=0.2887）
就撞上緣了，差 2 cm。用眼睛看，不要看數字。

⚠ **換姿勢＝重訓。** 這支只幫你選，選完的姿勢不能直接部署 —— 策略是從特定
home 訓練出來的。

---

## 合併回 main 的條件

這個分支要合併，下面每一項都要成立（也就是 `manifest.json` 的
`hardware_gates` 必須逐項有實測證據）：

- [x] `e1_homography_measured` — 2026-08-29：7 點 RMSE 0.388 cm；2 個保留點最大軸誤差 0.448 cm
- [ ] `e1_fov_ruler_check` — E1 放尺量，確認 x 約 13 cm、y 約 19 cm
- [x] `e1_gripper_center_height_ruler_check` — 張爪實測 15.2 cm，FK 15.58 cm
- [x] `e1_minimum_object_height_measured` — 3 cm 可夾、2 cm 空夾
- [x] `jetson_dry_run_ok` — 2026-09-25：repo 版在 Jetson 上 163/37/641/62/89 全過、`wrist_z_offset = 0.0564`、dry-run exit 0
- [x] `first_real_grasp_logged` — 2026-08-30 完整 log 已核對；可辨識範圍內重複 3/3 成功

沒過就留在分支上。v23 現已有 E1 實機 3/3 紀錄，但其他 gate 尚未全過；v21 完全沒被
動到，模式 A / B / C 也都還指著 v21。

---

## 已知會踩到的東西

**(0.28, −0.070) 這一格訓練只有 73.3%**，低於 80% 的單格門檻，hotspot 加了但 RL 還沒重跑。
那是 band 的右後角，envelope guard **會**放行。在那裡失敗是預期內的，不是新 bug。

**URDF 手指比實體長 16.7 mm**（2026-07-31 在 C3 量的，沒修、也還沒在 E1 重量）。
紙球、瓶蓋這類矮物體照樣夾不到。

**姿勢重現性 ±2°** 會帶來約 1.6 cm 的不裁切深度變動。固定 homography 沒辦法自適應，
所以每筆偵測都會重新核對手臂姿勢（`--pose-tol-deg`）。

**沒有訓練 metadata、沒有 git_head、沒有正式評估**隨包附上，這次訓練無法從
記錄重現。要離開 `candidate` 之前先跟訓練端要 v21 那種
`evidence/*_training_metadata.json`。

**模式 A / C 沒有接 v23。** `integration/vision_grasp_pipeline.py` 與
`nav_rl_grasp_pipeline.py` 仍透過 `_load_grasp_module()` 載入 `grasp/v21/`，
跑的還是 C3 策略。導航停止距離要加的 +1.3 cm 也還沒套進任何導航程式。

細節、每個數字的來源、以及各項 gate 的定義都在 `manifest.json`。
