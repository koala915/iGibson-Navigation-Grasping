# 巡航起步卡在 `cannot enter turn reserve`（2026-10-29）

放置盒往內移（中心離右牆、上牆各 45 cm）之後，連續三次 `開始 loop` 都在巡航起步時停下，
車子一步都沒走（`pulses 0`、`arrivals []`）。路線本身沒問題：從場地中間起步可完整規劃一圈。

## 三次失敗的數字

| 時間 | 起步位姿 (x, y, 朝向) | 中心到實際盒子 | 原地轉一圈時車身離盒子 | 轉身圈出界的點 | 朝巡航起點 | 前方 27 cm 可走 |
|---|---|---|---|---|---|---|
| 21:45 | (1.04, 0.37, 87°) | 0.420 m | **還有 11.2 cm** | 1 點（舊 6 cm 餘量的地圖） | −0.00 m | 否 |
| 21:55 | (1.03, 0.45, 114°) | 0.361 m | **還有 5.3 cm** | 4 點（22°–90°） | +0.22 m | 否 |
| 22:06 | (1.06, 0.40, 75°) | 0.385 m | **還有 7.7 cm** | 1 點（45°） | −0.18 m | 否 |

「車身離盒子」：車身外框（含右側線材）最遠的角離中心 0.308 m，原地轉一圈時掃過這個半徑；
中心到盒子實際外框的距離減掉它，就是轉身時車身與盒子之間剩下的空間。三次都還有 5–11 cm。

## 為什麼會擋（車上程式）

`deploy/ros/g2_map_patrol.py`（車上 sha256 開頭 `0563fa68a268defc`）約 383–391 行，起步脫困：

```python
for _ in range(3):
    current=runner.ready()['pose']
    if runner.rotation_allowed(current):break
    toward=(points[0][0]-current[0])*math.cos(current[2])+(points[0][1]-current[1])*math.sin(current[2])
    if toward<=0 or not policy.stopping_path_allowed(current,.2725):raise RuntimeError('cannot enter turn reserve')
    runner.pulse(.15,0,.35)
```

1. **`rotation_allowed`（131 行）**：中心周圍 6 cm 的 16 點都要在禁航遮罩內才准轉身。
2. 轉不了就只准**往前直走**，而且只朝 `points[0]` 前進。`points[0]` 是 `patrol_points(--inset)`
   的第一個角（離盒子最遠的那個角，現在在 (0.37, 0.40)），不是實際路線 `outer_patrol_goals.json`
   的入口；前方 0.2725 m 也要全在遮罩內。不准轉、不准倒退。
3. 車頭沒朝向那個角（`toward <= 0`），或前方會穿過盒子的緩衝，就直接 raise。三次都卡在這裡。

## 餘量疊了幾層

轉身檢查要求中心離盒子外框至少約 0.38 m，實際轉身只需要 0.308 m：

| 層 | 量 | 用途 |
|---|---|---|
| 盒子誤差餘量（`move_placement_box.py --slack`） | 0.02 m | 相機驗證後已從 6 cm 縮到 2 cm |
| 盒子緩衝 `center_keepout_buffer_m` | 0.283 m | **已經是車身半徑**（主體角 0.261、含線材 0.308） |
| 遮罩格半對角 | 0.014 m | 整格保守 |
| `rotation_allowed` 轉身圈 | 0.06 m | 原本給 AMCL 誤差 |
| 遮罩 2 cm 格子量化 | 0–0.028 m | |

緩衝 0.283 本身就是「中心在綠區任何一點都能原地轉身」的距離，轉身圈的 6 cm 又再疊一次。
牆邊同理：遮罩 35 cm 已含車身，轉身圈再加 6 cm。

## 可以怎麼改（給 ncu 巡航）

建議擇一或合併，從影響最小的開始：

1. **起步脫困改朝實際路線入口**：用 `g2_navigation_goals.nearest_route_entry` 的入口點取代
   `points[0]`，並允許在 `rotation_allowed` 為假時「先原地轉向入口」——只要轉身時車身外框
   （`BODY_RECTANGLES`）與盒子 `footprint_map`、牆的實際距離大於 0 加上 AMCL 誤差（例如 3 cm）。
   這是幾何檢查，不是遮罩格檢查。
2. **起步允許短距離倒退**（≤ 0.10 m、≤ 0.08 m/s），與投放後退出共用同一條 `back_path_problem` 檢查。
3. **轉身圈不要對盒子重複加 6 cm**：靠近盒子時，以實際車身與盒子外框距離判斷能否轉身；
   遠離盒子時維持原本的遮罩 + 6 cm。
4. 不建議直接把 `center_keepout_buffer_m` 調到 0.283 以下：它管的是整片綠區的「任何姿態都能轉」，
   盒子低於雷達掃描面（6 cm < 9.5 cm），轉身時撞到不會被雷達煞住。要靠近盒子，改上面 1–3 的局部檢查比較安全。

`g2_patrol_policy.py` 在 margin < 0.40 時會檢查 `navigation_mask_reduction_20261021` 的 sha256、
`bin_buffer_m >= 0.283` 與 `turn_reserve_m == 0.06`；改這些數字要同時更新紀錄，否則巡航拒絕啟動。

## 驗證

- 離線：拿上表三個位姿（22:06 的那個是現在的地圖），起步脫困應該都能找到合法動作，
  並且 `resolve_destination(policy, pose, 'patrol-cycle')` 可規劃。
- 從場地中間起步的行為不能變：(0.5, 0.5)、(0.40, 1.18) 都要和現在一樣直接開始。
- 實車：先在 (1.06, 0.40, 75°) 附近起步，人在旁邊、手放電源開關。

## 目前狀態（2026-10-29 22:10）

- 地圖：盒子餘量 2 cm 版已上車（`backups/box_move_20261029/before_slack02/` 是上一版）。
- 路線 `outer_patrol_goals.json`：盒子旁離右牆 0.86 m、上牆 0.89 m，角點 0.78 m。
- 暫時解法：車子起步時放在場地中間，離盒子 50 cm 以上，就不會觸發這段起步脫困。
