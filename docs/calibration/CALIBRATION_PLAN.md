# 校正 → 整合完整路線圖（相機內參 → 座標對齊 → RL 導航 → 全流程）

> 依序執行，**每個 Phase 有明確過關標準，沒過不要進下一步**。
> 每完成一個 Phase，把量到的數字填進「常數更新對照表」並更新 progress.md。
> 預估總時數：約 2–3 個工作天（Phase 1–3 一天、Phase 4 半天、Phase 5–6 一天）。

## 總覽

| Phase | 內容 | 過關標準（一句話） | 預估 |
|---|---|---|---|
| 0 | 環境同步與基礎檢查 | 相機串流可看、夾取 dry-run 照舊能跑 | 0.5h |
| 1 | 相機內參（fx/fy/cx/cy+畸變）×2 台 | 重投影誤差 < 0.5px | 1.5h |
| 2 | 手臂相機 C3 外參（θ/H/cam_x/cam_y/sign_y 一起解）+ 桅相機（θ、H） | 求解器殘差 ≤1cm、保留點 ≤1cm；桅相機 ±10cm | 2h |
| 3 | base 基準點 + 校正後驗證 | 保留點誤差 < 1cm、左右方向正確 | 1h |
| 4 | 定點視覺夾取端到端（含 smooth-approach） | 真物體夾取 ≥3/5 成功 | 3h |
| 5 | RL 導航單獨驗證（--nav-only） | 2m 直線到點 + 繞障不碰 + 煞停距離量測合格 | 3h |
| 6 | 全流程整合 | 偵測→導航→夾取 ≥2/3 成功 | 2h |
| 7 | 收尾 | PR merge、紀錄更新 | 0.5h |

> **2026-08-02 重要更正**：v21 的偵測姿勢是 **C3**（`90,67.08,9.79,9.79,90,30`），不是
> v17 的 nav home。手臂相機鎖在 `arm_link4`，θ/H/cam_x/cam_y 只在單一姿勢成立。
> C3 的相機幾乎垂直朝下（URDF 預測 θ≈89.7°、H≈0.215m、cam_x≈0.256m），與 nav home
> 量到的 36.4°/0.332m 完全不同，**舊值在 C3 一個都不能用**。Phase 2 已改寫成 C3 專用
> 流程並與 Phase 3 合併求解。目前登錄在 `integration/arm_cam_geometry.py` 的 C3 外參
> 是 **URDF 推算的預測值，不是實測**，bridge 會拒絕送出直到你量過或明示接受。

> **執行順序調整（2026-07-10 決定）**：下次上機先做 **Phase 3 → Phase 5（避障測試）**，
> 這兩個通過後才回頭做 Phase 4 → 6。兩者互相獨立（Phase 5 只用後鏡頭+LiDAR+底盤，
> 不碰夾取；Phase 3 只用手臂相機+手臂），順序對調不影響過關條件。

---

## Phase 0 — 環境同步與基礎檢查

**目標**：Jetson 上有最新程式碼 + 權重，相機/序列埠可用，舊功能沒壞。

**步驟**
1. 電腦端先同步 GitHub `main` 並確認沒有拿到舊分支；再把 `x3plus/` 內容送上 Jetson：
   ```powershell
   git pull --ff-only origin main
   git log --oneline -3
   # 在 x3plus/ 目錄下傳「內容」，避免多一層 x3plus/x3plus
   .\set_jetson_host.ps1 172.31.28.252
   scp -r .\* "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson2/"
   ```
2. Jetson：先跑 `python3 startup_device_check.py`。相機**一律依 serial/標籤辨識**，記下它輸出的
   `<ARM_N>` 與 `<REAR_N>`；不得假設 `/dev/video0` 或 `/dev/video1` 對應固定相機。
3. 若相機被佔用，對上一步辨識出的裝置執行 `sudo fuser /dev/video<ARM_N> /dev/video<REAR_N>`；確認是出廠程序後才停止它。
4. 起相機串流：`python3 stream_cam.py --device <ARM_N>`（手臂相機）；桅相機另開 `--device <REAR_N>`。
5. 確認：瀏覽器看得到兩路正確影像；`ls -la /dev/ttyUSB*` 記下各埠晶片（ch341=Rosmaster）。
6. 回歸測試：`cd grasp && python3 x3plus_real_grasp.py`（dry-run）跑完無錯。

**過關**：兩路串流正常、dry-run 照舊、序列埠清單記錄下來。
**沒過**：查 TROUBLESHOOTING #4–#6（串流/相機被佔用/裝置對應）。

---

## Phase 1 — 相機內參校正（你指定的起點）

**目標**：量出兩台相機真正的 fx/fy/cx/cy 和畸變係數。
目前程式裡有**兩套互相矛盾的常數**（pipeline 用 fx≈957、arm_cam.py 用 650），
本 Phase 一次定案，之後所有距離估計都建立在這上面。

**準備**：A4 列印棋盤格，貼平板上；尺量實際方格邊長（印表機常縮放）。
**本專案的板 = 7×10 方格（標示 20mm）→ 內角點 6×9**，即工具**預設** `--cols 9 --rows 6`
（校的是內角點＝方格數各減 1，不是方格數），只需 `--square-mm 20`。

**步驟**（每台相機各做一次；在 Jetson 上直接讀本地裝置 = 免串流、畫質最好）
1. 手臂相機（Phase 0 辨識出的 `/dev/video<ARM_N>`）：先把手臂移到**穩定姿勢**（建議直接用 nav home，反正之後就是這個姿勢）：
   ```bash
   cd ~/Documents/deploy_jetson2/grasp && python3 move_arm.py    # -> nav home，送完 servo 保持不動
   ```
   ⚠️ **內參與姿勢無關**——固定姿勢只是讓相機在校正過程中不飄，servo 上電別讓手臂垂下；
   校出來的 fx/fy/cx/cy 對所有姿勢通用。
   ```bash
   cd ~/Documents/deploy_jetson2/detection/calibration
   # 預設 --cols 9 --rows 6 已對應 6×9 內角點；只需給方格邊長
   python3 calibrate_intrinsics.py --source <ARM_N> \
     --square-mm 20 --out arm_cam_intrinsics.json --show
   ```
   （SSH 無螢幕時拿掉 `--show`，靠終端印的 `shot N/M` 計數；或改在**電腦端**對 Jetson 串流 URL 跑，
   這樣 `--show` 有畫面能確認棋盤有被偵測到、覆蓋是否夠。）
2. 拿棋盤在鏡頭前**慢慢移動並在每個位置停一下**（動態模糊會毀了角點精度）：中央、四角、近、遠、
   左右各傾 30°，自動擷取 18 張。
3. 桅相機（Phase 0 辨識出的 `/dev/video<REAR_N>`）同法：`--source <REAR_N> --out rear_cam_intrinsics.json`。
4. 有螢幕的話跑 `--check <json>` 看去畸變後直線是否變直。
5. **畸變決策**（工具會直接印出）：看 `distortion displacement` 的
   bbox-bottom-center 位移——距離模型讀的是**原始** y2/cx_box，這個數字就是誤差來源。
   - **< 2px** → 在本文件記「畸變：忽略」，程式不動。
   - **≥ 2px** → 偵測前對 frame 做 `cv2.undistort`（或至少對 bbox 底邊中點做
     `cv2.undistortPoints` 再進距離公式），列入 Phase 2 前的修改項。

**過關標準**
- RMS 重投影誤差 **< 0.5px**（理想 < 0.3）
- fx ≈ fy（差 < 2%）
- (cx, cy) 落在 (320, 240)±40 內（640×480）
- **畸變決策已記錄**（忽略 or 加 undistort）
- 兩台的 fx 出爐後就知道 957 和 650 哪個對（或都不對）

**沒過**：板不平/方格尺寸量錯/傾角覆蓋不足，重拍。單張誤差特別大的 view 重做。
**產出**：兩份 json → 填入對照表①，更新程式常數（見表）。

---

## Phase 2 — 地面距離模型校正（θ 俯角、H 高度）

> **2026-08-02 重寫。** v21 的偵測姿勢是 C3（`90, 67.08, 9.79, 9.79, 90, 30`），不是
> v17 的 nav home。手臂相機鎖在 `arm_link4` 上，θ/H/cam_x/cam_y 四個外參只在**單一
> 姿勢**成立，換姿勢就全部作廢。舊版這一節的做法在 C3 會直接失效，原因有二：
>
> 1. 舊解法用 `θ = atan(H/D)`。C3 的相機離垂直不到一度，畫面上 D 會跨過 0 變負，
>    `atan(H/D)` 在 D=0 無定義、D<0 取錯分支。正確要用 `atan2(H, D)`。
> 2. 舊版要你在 0.3–0.8 m 貼記號。C3 只看得到 base x ≈ **0.195–0.32 m**（橫向 −5.9～+10.7 cm，不對稱），那些記號
>    全在視野外。
>
> 所以 C3 的 Phase 2 與 Phase 3 **合併成一次量測**，用 `solve_arm_cam_extrinsics.py`
> 一起解 θ / cam_x / cam_y / sign_y。桅相機（rear）不受影響，仍照舊版做。

**目標**：定出 C3 姿勢下的 θ、H、cam_x、cam_y、sign_y。

**先做 Phase 3 步驟 1**（建立 base-frame 地面基準點），因為底下每個位置都要用
base 絕對座標記錄。

**步驟**
1. 手臂開到 C3：
   ```bash
   python3 grasp/v21/x3plus_real_grasp.py --real --unlock-candidate-real      --model models/candidate_v21_seed816_ckpt550000.zip      --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl      --contract obs_28_incremental --max-steps 0
   ```
   （`--max-steps 0` 會走到起始姿勢就停。）
2. **尺量 H**：鏡頭中心到地面的垂直高度。近垂直姿勢下這是四個參數裡最好量、
   最準的一個，務必用尺，不要用擬合。URDF 預測值 **0.2148 m**，差超過 1 cm 要先查姿勢。
3. 擺物體，**至少 5 點（建議 8 點）**。⚠ **可用範圍很窄，而且隨物體高度縮小**——
   採集工具一開始就會印出來（`usable_placement_window`）：

   | 物體高度 | 完整入鏡的 base x | base y（x 中段）|
   |---|---|---|
   | **sugarbox 6.5 cm** | **0.239 – 0.292 m** | **−0.028 – +0.052 m** |
   | 3 cm | 0.227 – 0.299 | −0.036 – +0.068 |
   | 平面標記 | 0.218 – 0.304 | −0.042 – +0.081 |

   近垂直視角下物體的**頂面投影得比底面遠**，輪廓大於落地面積，越高越早出畫面。
   範圍外會被貼邊檢查擋掉、不會記錄。左右也不對稱（主點在 x=212 不是 320）。
   - 前後、左右都要盡量用滿這個範圍，否則 θ 與 cam_x 分不開、`sign_y` 與 `cam_y` 解不出來。
   - **另外留 2 點不參與擬合**，最後拿來驗證。
4. **用採集工具，不要手抄**（手抄最容易把對的像素配到錯的座標，而且事後看不出來）：
   ```bash
   python3 integration/capture_arm_cam_obs.py --h <尺量的H> --solve \
     --stream http://<JETSON_IP>:8080/stream?topic=/arm_cam/image_raw \
     --out c3_calib.json
   ```
   擺好物體 → 輸入 base 絕對座標 → Enter。它取 20 幀中位數像素、每筆存檔、自動留
   保留點、涵蓋不足會提示缺什麼。上機前可先 `--simulate` 在桌上演練一次。
5. `--solve` 會自動解完；要手動重解時（座標一律 base 絕對值，橫向 **±3.5～4.5 cm**）：
   ```bash
   python3 integration/solve_arm_cam_extrinsics.py --h 0.2151 \
     --obs 0.205,0.000,233,413 \
     --obs 0.230,-0.040,64,306 \
     --obs 0.255,0.000,234,200 \
     --obs 0.280,0.040,403,95 \
     --obs 0.300,0.000,233,11 \
     --obs 0.240,0.035,381,264
   ```

> **演練實測**（`--simulate`，1.5 px 雜訊、8 點、6 擬合 2 保留）：擬合殘差 **0.93 mm**、
> 保留點誤差 **0.96 mm**，只佔 22 mm 進場容差的 4%。同一次解出的 θ 差 2.26°、
> cam_x 差 8.5 mm——**參數不準但預測很準**，正是下面那段警告的意思。

**過關標準**
- 求解器殘差 **≤ 10 mm**（≤ 5 mm 為佳）
- 保留的 2 點代入後誤差 **≤ 10 mm**
- `sign_y` 有被資料決定（沒跳「lateral 太集中」警告）

**⚠ 不要拿解出來的 θ 去對 URDF 預測值 89.740。** 可視帶太窄，θ 與 cam_x 高度相關，
兩者會互相補償：模擬中 1 px / 2 mm 雜訊下 θ 只能還原到約 ±5°、cam_x 約 ±2 cm，
但保留點的預測誤差平均只有 0.9 mm。曾有一次擬合解出 θ=79.8（真值 89.3）卻仍把
每個點放在 1.3 mm 內。**看殘差與保留點，不要看參數本身**，也絕對不要外推到量測
帶之外。

**沒過**：誤差隨距離變大 → H 量錯；整體固定偏移 → base 基準點記錯（回 Phase 3 步驟 1）。
**產出**：θ / H / cam_x / cam_y / sign_y。兩種填法擇一：
- 改 `integration/arm_cam_geometry.py` 的 `V21_C3_GRASP_HOME`（把兩個 `_measured`
  旗標改成 `True`）
- 或完全不動程式碼，跑 bridge 時帶 `--cam-theta/--cam-h/--cam-x/--cam-y/--sign-y`

---

## Phase 3 — base-frame 基準點與驗證（C3 已與 Phase 2 合併求解）

**目標**：建立量測用的 base 座標原點，並在 Phase 2 解完後驗證整條視覺→座標鏈。

> **2026-08-02**：C3 姿勢下 cam_x/cam_y 已由 Phase 2 的 `solve_arm_cam_extrinsics.py`
> 與 θ 一起解出，不再需要先假設 θ 正確再單獨解偏移——那個兩階段做法在近垂直
> 姿勢下不成立（θ 與 cam_x 高度相關）。`solve_cam_to_base.py` 仍留給 nav home
> 那類前視姿勢用。

**步驟**
1. **建地面基準點**（Phase 2 開始前就要做）：`--real` 跑到 C3 起始姿勢，從指間
   抓取中心鉛直投影到地面貼記號，並記下 log 的 TCP base 座標 `(ref_x, ref_y)`
   （如 `(0.167, 0.018)`）。**記號只是量尺原點，不是 `base_link` 原點。**
2. **z_offset 順便量**（TROUBLESHOOTING #11）：同一時刻尺量抓取中心離地高 H_real，
   `z_offset = H_real − 0.010`。
3. 之後每個擺放位置都換成 **base-frame 絕對座標**再記錄：
   `(actual_x, actual_y) = (ref_x + forward_from_mark, ref_y + left_from_mark)`。
   例如 `ref=(0.167,0.018)` 時，記號正前 8 cm 是 `(0.247, 0.018)`。
4. **驗證**（homography 解完之後）：把保留、不參與擬合的點擺回去，跑 bridge dry-run
   讀輸出：
   ```bash
   python3 integration/vision_grasp_bridge.py --dry-run --once --show \
     --homography grasp_home_homography.json \
     --class-height sugarbox=0.065
   ```
   印出的 `x`/`y` 與實擺 base 座標比對。

**過關標準**：保留點誤差 **< 1 cm**（x、y 各自）、左右方向正確（物體往左移，y 要變大）。
**沒過**：誤差隨距離變 → H 量錯，回 Phase 2 步驟 2；整體固定偏移 → 基準點記錯，回步驟 1。
**產出**：z_offset → 對照表③（θ/H/cam_x/cam_y/sign_y 已在 Phase 2 產出）。

### grasp-home homography（Mode B 必要 gate）

`vision_grasp_bridge.py` 在 C3 不接受 nav-home 的 H/θ/offset，也不能用
`--i-accept-predicted-extrinsics` 繞過。請在可夾工作區擺至少 **6 個不共線、覆蓋四周與中央**
的位置；每點量 base-frame `(x,y)`，並取得穩定的 undistorted `(u,v)`：

```bash
python3 integration/vision_grasp_bridge.py --dry-run --calibration-only --once \
  --calibration-samples 20 --show --class-height sugarbox=0.065
```

把輸出的 `u,v` 與實測 `x,y` 整理成 JSON `[{"u":...,"v":...,"x":...,"y":...}, ...]`，
保留另外至少 2 點不要拿去擬合，再產生 runtime 檔：

```bash
python3 integration/grasp_home_homography.py \
  --points-json grasp_home_points.json \
  --output grasp_home_homography.json --max-rmse-cm 1
```

現有 `c3_calib_real_20260802.json` 只有 5 筆、且前兩筆是同一個 base 座標，**不能**通過
runtime 的 6 點 gate；需要重新補量。擬合 RMSE 與保留點 x/y 誤差都須 ≤1 cm，且 runtime
只接受 calibration convex hull 內的偵測，不做外推。

### E1 grasp-home homography（v23，分支 `v23-grasp-test`）

**每個 grasp home 各要一份校正檔，不能共用。** 相機裝在 `arm_link4`，姿勢一動相機就動：

| | C3（v21） | E1（v23） |
|---|---|---|
| 手臂 API | (90, 67.08, 9.79, 9.79, 90, 30) | (90, **74.2**, **8.6**, **8.6**, 90, 30) |
| 相機高（training frame） | 0.2183 m | 0.2340 m |
| 光軸 | (−0.0583, 0, −0.9983)，離鉛垂 3.34°**偏後** | (+0.0244, 0, −0.9997)，離鉛垂 1.40°**偏前** |
| 校正檔 | `integration/grasp_home_homography.json` | `integration/grasp_home_homography_e1.json` |
| registry pose | `v21_c3_grasp_home` | `v23_e1_grasp_home` |

⚠ **兩個檔案互換不會報錯。** 都是合法 JSON、都過同一道 ≥6 點／<2 cm 閘、內容也都不記錄
自己是在哪個姿勢量的。唯一把它們分開的是**檔名**，所以 v23 的 launcher 預設指向
`_e1` 那支，而且不會去讀 v21 那支。

量測時手臂要有東西按在 E1 不動 —— 就是 launcher 本身：

```bash
source ~/grasp_venv/bin/activate
cd grasp/v23
python3 jetson_one_command_grasp.py --calibrate --calibration-samples 20 --show
```

物體擺放要落在 v23 的 spawn band 內：**x 0.205–0.280 m、y −0.070–+0.065 m**
（比 v21 窄，但 E1 的 FOV 比 C3 大 44% —— 相機看得到 band 以外的地面，
那裡策略沒訓練過，`_check_target_envelope` 會擋，不要把它放寬去配合 FOV）。

至少 6 組不共線、覆蓋四周與中央，另留 2 組當保留點。Ctrl+C 後：

```bash
python3 integration/grasp_home_homography.py \
  --points-json grasp_home_points_e1.json \
  --output integration/grasp_home_homography_e1.json --max-rmse-cm 1
```

驗收 `python3 jetson_one_command_grasp.py --check`（不碰硬體，用 bridge 同一道閘）。

**為什麼 E1 不能退回三角測距。** `arm_cam_geometry` 的地面距離模型要除以光線與地面
夾角的正切，相機越接近鉛垂，回推的橫向偏移越趨近 0、深度對 θ 誤差越敏感。C3 的 3.34°
已經在這個區間邊緣（該模組自己的註解寫 offsets「collapse to ~0」），E1 的 1.40° 更深，
而且光軸還跨過了鉛垂線（x 分量由負轉正）。`arm_cam_geometry.V23_E1_GRASP_HOME` 那組
θ/H/cam_x/cam_y 存在的用途是**替偵測蓋上姿勢戳記**（grasp 端拿去跟編碼器核對），
不是拿來映射像素。

---

## Phase 4 — 定點視覺夾取端到端（不含導航）

**目標**：只在 grasp-home 可見且 ≤15cm 的工作區，驗證平順化與校正後視覺夾取。

> **前置 gate**：必須先完成上面的 grasp-home homography。舊 nav-home 四點常數不能通過此 gate。

**步驟（依序）**
1. **近距固定座標 dry-run**：用實際基準點前方約 5／10／14cm 分別 dry-run；三點純軟體
   rollout 已通過，但 Jetson 仍需核對 `[Reach]`、lock pose 與關節限制。
   ```bash
   python3 grasp/x3plus_real_grasp.py --smooth-approach \
     --obj-x <base絕對X> --obj-y <base絕對Y> --obj-z 0.02 --max-steps 180
   ```
   超過 15cm 必須在任何 policy/glide 前被拒絕。
2. **跨指間隙量測**（TROUBLESHOOTING #10）：S6=30° 開爪內側間隙 vs 物體寬 →
   單側裕度 ≥2cm 才安全。
3. **固定座標真物體夾取**：木塊放在 Phase 3 基準記號前方 25 cm；命令列的 `--obj-x/--obj-y`
   必須填 base-frame 絕對座標（`ref_x + 0.25`, `ref_y`），不是相對記號的 `(0.25, 0)`。
   `--obj-z` 用 `實際離地高 − z_offset`，跑 3 次。
4. **視覺 latch 夾取**（用 bridge 模式＝模式 B，**不要**跑完整 pipeline——那會把導航/
   精對位混進來，這步只測「相機座標 + 夾取」）：物體隨機放手臂相機視野內（0.2–0.3m），
   ```bash
   # 終端機 1（夾取端，v21）
   cd grasp/v21
   python3 x3plus_real_grasp.py --real --socket \
     --latch-obj --i-confirm-external-frame --unlock-candidate-real \
     --model models/candidate_v21_seed816_ckpt550000.zip \
     --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
     --contract obs_28_incremental
   # 終端機 2（辨識端，先 --once 核對座標再連續送）
   #   --class-height/-z 依實測物體填；sugarbox 為 0.065 / 0.0325
   python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --once --show \
     --class-height sugarbox=0.065 --class-z sugarbox=0.0325
   python3 integration/vision_grasp_bridge.py --host 127.0.0.1 \
     --class-height sugarbox=0.065 --class-z sugarbox=0.0325
   ```
   跑 5 次。
   > v21 沒有 `--width-grip`（改接觸偵測）和 `--smooth-approach`。
   > `--latch-obj` 與 `--i-confirm-external-frame` 在 `--real --socket` 下是**強制**的，
   > 缺任一會 exit 2。`--unlock-candidate-real` 是因為 manifest status 仍為 `candidate`。
   > 舊 v17 指令見 `integration/README.md` 模式 B 段落的備援區塊。

**過關標準**：步驟 3 ≥2/3、步驟 4 **≥3/5** 夾起且 verify 通過；過程無撞地/撞物。
**沒過**：夾空 → 座標差（回 Phase 3）；碰倒 → 看 #10 的對策順序（obj-z 對上半部→y 對位→墊高）。
**產出**：可用的定點夾取；TROUBLESHOOTING 補實測心得。

---

## Phase 5 — RL 導航單獨驗證（--nav-only，不夾取）

**目標**：導航策略上實車，先不碰夾取。細節見 `integration/NAV_RL.md`。

**步驟（依序）**
1. **LiDAR 資料鏈已確認（2026-07-16）**：實機是 YDLIDAR TG30，不是 Slamtec；
   `TG.launch` 已辨識 firmware 2.1／health good，`/scan` 約 10.17Hz。程式已加入
   rosbridge `/scan` adapter，Phase 5 不再卡在 Python `rplidar` 套件；不要安裝
   `rplidar-roboticia`。先開兩個終端機：
   ```bash
   # 終端機 1
   source /opt/ros/melodic/setup.bash
   source /home/jetson/software/library_ws/devel/setup.bash
   unset ROS_IP
   export ROS_HOSTNAME=127.0.0.1
   export ROS_MASTER_URI=http://127.0.0.1:11311
   roslaunch ydlidar_ros_driver TG.launch

   # 終端機 2
   source /opt/ros/melodic/setup.bash
   source /home/jetson/software/library_ws/devel/setup.bash
   unset ROS_IP
   export ROS_HOSTNAME=127.0.0.1
   export ROS_MASTER_URI=http://127.0.0.1:11311
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```
2. **方向校正**（終端機 3，`grasp_venv`）：
   ```bash
   cd ~/Documents/deploy_jetson2
   source ~/grasp_venv/bin/activate
   python3 -m pip install roslibpy  # 首次一次
   python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1
   ```
   手放車**正前** 0.4m → 中間扇區數字應變小；再放**左**、**右**各測。
   ROS 預設已是左正（`--lidar-dir 1`）；若實測左右相反才改 `--lidar-dir -1`。
   整體偏 → `--lidar-yaw-offset-deg`。
   **順便用尺量 LiDAR 中心到車體中心的前後距離** → `nav_rl.NavRLConfig.
   lidar_forward_offset_m`（LiDAR 在中心前方為正；不填的話 48 束距離全部
   以 LiDAR 位置當車中心，近距離判斷會偏）。
3. **推論測試**：先跑目前預設 baseline，再測 doorway 候選；兩組 zip/pkl 不可交叉：
   ```bash
   python3 integration/nav_rl.py --bench
   python3 integration/nav_rl.py --bench \
     --model integration/nav_best_model/doorway_ft_final.zip \
     --vecnorm integration/nav_best_model/doorway_ft_final_vecnormalize.pkl
   ```
   兩組都要能以 SB3 2.3.2 載入且速率 ≥30Hz；bench 不代表實車導航已通過。
4. **轉向正負**：dry-run `python3 integration/nav_rl_grasp_pipeline.py --nav-only`，
   物體放偏左，log 的 `wz` 應為正；`--real` 低速首測若車右轉 → `--wz-sign -1`。
5. **直線到點 baseline**：空地、物體正前 2m，先用目前預設權重：
   ```bash
   python3 integration/nav_rl_grasp_pipeline.py --real --nav-only --nav-stop-dist 1.0 \
     --lidar-backend ros --ros-host 127.0.0.1
   ```
   通過後降 `--nav-stop-dist 0.75` 再測。
6. **doorway 候選 A/B**：在完全相同起點、物體與障礙配置，顯式加：
   ```bash
   python3 integration/nav_rl_grasp_pipeline.py --real --nav-only --nav-stop-dist 1.0 \
     --model integration/nav_best_model/doorway_ft_final.zip \
     --vecnorm integration/nav_best_model/doorway_ft_final_vecnormalize.pkl
   ```
   候選需成功率不降、碰撞／BRAKE 介入不增且無固定單側偏轉，才可考慮升預設。
7. **繞障**：機器人與物體之間放一個紙箱（偏一側），看策略是否繞過去、
   log 有無 `STALL`/`BRAKE`、`minray` 數字合理。
8. **煞停測試**：行進中把紙板伸到車前 <0.25m → log 出現 `BRAKE`、車停；以膠帶記錄
   觸發點與停止點，量出實際停止距離。首測全速 0.5 m/s 時，停止距離必須小於預先保留的
   空地安全裕度，否則降低速度／加大煞停門檻後重測。

**過關標準**
- probe 三方向反應正確；bench ≥30Hz
- 直線 2m 到點停下（停點離物體 ≈ nav-stop-dist ±0.15m）
- 繞障不碰箱子（跑 3 次至少 2 次乾淨繞過——策略本身碰撞率 0.2 左右，煞停要兜底）
- 煞停 100% 有效，且每次量得的停止距離均在預先標出的安全裕度內

**沒過**：完全不動/亂轉 → 檢查 obs（開 log 比對 dist/bearing 合理性）、control period；
行為抖 → `--control-period` 微調（**不要動 plant 參數**）。
**產出**：lidar-dir/yaw-offset/forward-offset/wz-sign 定案 → 對照表④。

> 目前此 Phase 的底盤 backend 仍是 `set_car_motion`。若要改用自製 `set_motor`，必須先通過
> `integration/SETMOTOR_ODOM_INTEGRATION.md` 的 A/B gates；不可在 Phase 5 現場臨時把
> 連續速度改成離散 forward/turn 指令。

---

## Phase 6 — 全流程整合

**目標**：偵測 → RL 導航避障 → 精對位 → 夾取 → 驗證，一條龍。

**步驟**
1. 物體放 2–3m 外、視野內，無障礙跑一次：
   ```bash
   python3 integration/nav_rl_grasp_pipeline.py --real --show \
     --cam-x <Phase3_X> --cam-y <Phase3_Y> --sign-y <1或-1> \
     --i-confirm-camera-frame
   ```
2. 加一個障礙物再跑。
3. 故意讓第一次夾失敗（物體放歪一點）驗 retry：退避→重新接近→再夾。

**過關標準**：≥2/3 端到端成功（夾起 + verify 過）；retry 邏輯運作正常。
**沒過**：分段定位——導航段掛 → 回 Phase 5；精對位/latch 偏 → 回 Phase 3；
夾取段掛 → 回 Phase 4。每段 log 都有自己的前綴（[nav-rl]/[pipeline]/[Stage]）。

---

## Phase 7 — 收尾

- [ ] PR #2 轉正式 + merge；校正後的常數 commit（訊息附量測數據）
- [ ] progress.md 補各 Phase 結果；TROUBLESHOOTING.md 補新踩的坑
- [ ] 錄一段全流程 demo 影片（之後報告/口試用）

---

## 常數更新對照表（校到哪填到哪）

| # | 校正值 | 量到的值 | 要更新的位置 |
|---|--------|---------|-------------|
| ① | 手臂相機 fx/fy/cx/cy | ✅ 919.08 / 919.41 / 212.23 / 168.42（RMS 0.495px）| **2026-08-02 起單一來源**：`integration/arm_cam_geometry.py`（`FX/FY/CX/CY/DIST`）。bridge / pipeline / arm_cam.py 都改成引用，不再各留一份 |
| ① | 桅相機 fx/fy/cx/cy | ✅ 544.16 / 544.82 / 316.98 / 244.79（RMS 0.374px）| **已寫入 2026-07-10**：`vision_grasp_pipeline.py` FX_REAR… |
| ② | θ_ARM / H_ARM **@nav home** | ✅ 36.40° / 0.332 m（回推誤差 ≤0.43cm）| `arm_cam_geometry.V17_NAV_HOME`。**只在 nav home 有效**，v21 不用這組 |
| ② | θ / H / cam_x / cam_y / sign_y **@C3** | ⚠ 目前是 FK 預測值 89.740° / 0.2148 m / +0.2762 / −0.0056 / +1，**未實測** | `arm_cam_geometry.V21_C3_GRASP_HOME`，量到後把兩個 `_measured` 旗標改 True；或跑 bridge 時帶 `--cam-theta/--cam-h/--cam-x/--cam-y/--sign-y` 不動程式碼 |
| ② | θ_REAR / H_REAR | ✅ 16.35° / 0.503 m（回推誤差 ≤0.98cm，有效 0.7–1.5m）| **已寫入 2026-07-10**：`vision_grasp_pipeline.py` |
| ③ | CAM_TO_BASE_X/Y、SIGN_Y | C3 已併入②一起解（`solve_arm_cam_extrinsics.py`）| 不再是獨立項目。前視姿勢（nav home）仍可用 `solve_cam_to_base.py` |
| ③ | z_offset | ＿＿ | 擺物時 `--obj-z = 實際離地高 − z_offset`；`--class-z` 各類別高度 |
| ④ | lidar dir / yaw offset / port | ＿＿ | `nav_rl.NavRLConfig` 預設值或 CLI |
| ④ | lidar forward offset（LiDAR 中心↔車中心，前正） | ＿＿ | `nav_rl.NavRLConfig.lidar_forward_offset_m` |
| ④ | wz_sign | ＿＿ | CLI `--wz-sign`（定案後可改 pipeline 預設） |
| ① | 畸變決策（忽略 / undistort） | ✅ 手臂 9px 不可忽略；後鏡頭 1.79px 忽略 | **已寫入 2026-07-10**：三檔的手臂鏈路都在距離模型前對 bbox 底邊中點做 `undistort_pixel()`（純數學版，與 cv2.undistortPoints 同法）；後鏡頭維持 raw |

> θ 是以「去畸變後」像素解出的（Phase 2 流程如此），所以 runtime 一定要先 undistort
> 再進距離模型——`arm_cam_geometry.ground_hit_from_raw()` 已內建，pipeline selftest 用
> Phase 2 實測點（明確綁 nav home）驗證誤差 ≤0.33cm。
>
> **θ 的量測慣例**：一律從 base **+X 方向**量俯角。C3 的光軸已越過垂直（URDF 報
> 「俯角 86.7°、水平分量朝後」），以 +X 量同一條射線是 **93.34°**——直接拿 86.7 餵進
> 距離模型會把整個工作區左右鏡射。`verify_camera_grasp_frame.py` 現在會直接印正確值
> 並在越過垂直時警告。

**改完常數一律重跑該 Phase 的驗證一次**，確認填對地方。
