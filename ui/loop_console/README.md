# 決賽操作台（loop console）

2×2 m 決賽場地用的操作台：重新定位、開始整個 loop、地圖上看機器人位置與方向、
目前狀態、裝置自檢。**跑在這台電腦上**（`mission_loop.py` 和 YOLO／SAM2 都在這裡），
不是 Jetson；原本在 Jetson 上的 `ui/server.py` 操作台不受影響。樣式直接沿用
`ui/static/style.css`。

```bash
.venv-approach/Scripts/python.exe ui/loop_console/server.py --host 10.53.253.252
```

瀏覽器開 `http://localhost:8090`；同一個網路的手機開 `http://<這台電腦的 IP>:8090`。

沒有機器人時可以重播一次實車紀錄來看畫面（車子位置是模擬的繞圈）：

```bash
python ui/loop_console/server.py --simulate logs/mission_20261021_patrol19/mission.out
```

## 功能

| 區塊 | 內容 |
|------|------|
| 重新定位 | 執行 `integration/relocalize.py`，車子不動。失敗會顯示原因與擺車建議 |
| 開始 loop | 執行 `mission_loop.py --real --i-am-beside-the-robot`，可勾「先重新定位」、填送達上限。要先勾「我在車旁，手放在電源開關上」；自檢有紅燈時不能按 |
| ■ 停止 | 結束任務程序、停掉車上巡航、送輪子停止。**真正的急停是電源開關** |
| 地圖 | 牆、巡航允許帶、放置盒（未驗證為虛線）、牆上標記編號、機器人位置與方向（每 0.5 s）、任務中的軌跡、看到的糖果盒（空心＝候選、實心＝已確認、大圓＝粗估範圍、橘點＝失敗、叉＝略過）。畫面依牆面轉正 |
| 目前狀態 | 階段、已送達、失敗、巡航圈、用時，及最新說明 |
| 裝置自檢 | ssh、6 個車上服務、手臂相機（USB／畫面新鮮度／10 分鐘內斷線次數）、後相機、定位、手臂姿態、夾爪、底盤、Jetson 記憶體、這台電腦的模型與串流接收。閒置時每 10 秒自動檢查 |
| 修復按鍵 | 手臂相機：重啟 `grasp-vision`（不動車）。手臂開機沒收回：home → stow → 重啟 `grasp-service`（會動手臂，需確認） |
| 剛剛發生什麼 | 任務輸出整理成中文事件；看不懂的行只有像錯誤時才列出 |

糖果盒的地圖位置是相機看到當下，用當時的定位把「前方幾 m、左右幾 m」換算過去。
校正範圍外的粗估可能差幾十公分，所以畫成大圓。

不顯示相機畫面（兩個操作台都一樣）。

## 測試

```bash
.venv-approach/Scripts/python.exe ui/loop_console/test_loop_console.py
```

包含：一次完整實車紀錄（第 19 趟）的解析、自檢判定、地圖資料、API 安全閘（未確認不能開始、一次只跑一個工作、停止會送到機器人）。
