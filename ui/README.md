# `ui/` — 操作台

用手機或筆電操作機器人，不用開終端機打指令。網頁由機器人自己提供。

## 建議使用方式：桌面 QR 啟動器

Jetson 初次安裝一次：

```bash
cd ~/Documents/iGibson-Navigation-Grasping
source ~/grasp_venv/bin/activate
python -m pip install qrcode pillow
bash ui/install_desktop_shortcut.sh
```

之後只要雙擊桌面的 **X3Plus 手機操作台**：

1. 畫面提醒「請將機器人與手機連接到同一個 Wi-Fi 網路」。
2. 按「下一步」，操作台會自動啟動並顯示 QR Code。
3. 手機掃描後直接進入 UI，不需要查詢或輸入 IP。

桌面啟動器使用 Tkinter，不會在 Jetson 開啟 Chromium。捷徑預設以
`--allow-real` 啟動伺服器，但這目前只開啟第一道閘；A/B/C 缺少操作台可提交的
模式專用校正證據，實機請求仍會被拒絕。

---

## 終端機啟動方式

```bash
# 開發機（沒有機器人也能跑，狀態由模擬器產生）
python3 ui/server.py --simulate

# Jetson 上，唯讀監看（不會驅動硬體）
python3 ui/server.py

# Jetson 上，開啟第一道實機閘（A/B/C 目前仍 fail-closed）
python3 ui/server.py --allow-real
```

開好之後手機瀏覽器打開 `http://<Jetson 的 IP>:8080`。

---

## 架構

```
手機 / 筆電瀏覽器
    │  GET  /api/events   ← Server-Sent Events，狀態即時推送
    │  POST /api/…        → 開始 / 停止 / 確認 / 檢查
    ▼
ui/server.py                                    ← 不碰硬體
    │  spawn + stdin/stdout          ▲ UDP 8099
    ▼                                │
integration/mission_pipeline.py ─────┘          ← 唯一持有 /dev/myserial
```

**伺服器不碰硬體。** `/dev/myserial` 只能有一個持有者，那個持有者是任務程序。
操作台負責啟動它、讀它的遙測、對它送訊號。如果網頁伺服器自己也開序列埠，就會
變成第二個持有者 —— 半雙工伺服匯流排上兩個持有者的結果是手臂在沒人下令時動作。

**只用標準函式庫。** Jetson Nano 是 aarch64 上的 Python 3.8，`pip install fastapi`
會拖進一個沒有預編譯輪子、要現場編譯 Rust 的 pydantic。在一台 import torch 要四
分鐘的機器上，「pip 一下就好」是展示當天出事的典型原因。`ThreadingHTTPServer` +
SSE 本來就在標準庫裡，而且夠用：狀態只從伺服器流向瀏覽器，指令是普通的 POST。

**遙測走 UDP。** 任務程序絕對不能因為沒人在看而卡住。UDP 送給沒人讀的 socket 會
立刻回傳；管線或 TCP 會塞滿然後停住控制迴圈。掉一格也無所謂 —— 這是 10 Hz 的
重複快照，不是事件日誌，下一格就到，而且帶著完整狀態。

**而且不是每個 tick 都送。** 狀態變化立刻送 —— 那才是資訊；變化之間 0.5 秒
一次心跳（`--status-period`）。控制迴路 10 Hz，但「機器人大概在哪」不需要
一秒十次。

---

## 檔案

| 檔案 | 說明 |
|------|------|
| `server.py` | HTTP + SSE 伺服器、任務程序監督、安全閘。標準庫，無相依 |
| `static/index.html` | Lite 正式介面：連線 / 主控台 / 任務設定 / 地圖 |
| `static/app.js` | 前端。唯一狀態來源是 SSE，不模擬也不推測 |
| `static/style.css` | 樣式（深淺兩色都完整支援） |
| `make_qr.py` | 開機時偵測 IP、把連線 QR code 畫到桌面 |
| `launcher.py` | 兩步驟 Tkinter QR 啟動器；不開 Jetson 瀏覽器 |
| `launch_ui.sh` | 桌面捷徑入口，優先使用 `~/grasp_venv` |
| `install_desktop_shortcut.sh` | 在 Jetson 桌面建立「X3Plus 手機操作台」捷徑 |
| `jetson_check.sh` | ★上機一鍵檢查：連接埠、防火牆、UDP 迴路、序列埠占用、路線 |
| `test_server.py` | 45 項回歸測試，全部離線、不碰硬體 |
| `prototype.html` | 最早的靜態原型。單一檔案、雙擊即開、不需要 Python，適合截圖或沒有機器人時展示版面。**功能以 `static/` 為準** |

搭配的改動在 `integration/`：

| 檔案 | 改了什麼 |
|------|---------|
| `mission_status.py` | 新增。遙測發布器（含節流）、`SimpleReporter`、`--listen` 診斷工具 |
| `mission_pipeline.py` | 新增 `--status-udp HOST:PORT` 與 `--status-period`。沒帶旗標時行為完全不變 |
| `nav_rl_grasp_pipeline.py` | 新增 `--status-udp`（模式 C） |
| `vision_grasp_bridge.py` | 新增 `--status-udp`（模式 B） |
| `preflight.py` | 新增 `--json`。人看的輸出沒有改變 |

**三個模式都會回報，只是話多話少不同。** 模式 B 和 C 沒有狀態機，但它們用
`SimpleReporter` 講的是**同一套狀態名稱**（`APPROACH`、`LATCH`、`GRASP`…）。
操作者不該因為換了一個模式就得學第二套詞彙。傳一個 FSM 沒有定義的名字會直接
報錯，而不是安靜地送出一個介面不會描述的狀態 —— 有測試守著這件事。

---

## 為什麼沒有相機畫面

有做過，2026-08-06 拿掉了。

Jetson Nano 跑這個專案時 RAM 已經在九成上下。在控制迴路裡把每張影格編成 JPEG
是操作台加進去的成本裡最貴的一項，而它換到的東西 —— 看畫面 —— 不是操作機器人
需要的。需要的是：**現在在做什麼、為什麼停下來、怎麼叫它停**。

整條路徑都刪乾淨了（`camera_publish.py`、MJPEG 端點、辨識迴圈裡的掛勾），
而且有測試盯著不讓它回來：`TestResourceGuards` 會掃 `server.py` 和
`vision_grasp_pipeline.py`，出現 `cv2` / `imencode` / `VideoCapture` 就失敗。

要看畫面的時候用模式 B 的 `--show`，那是在機器人的螢幕上開視窗，不經過網路。

---

## 省下來的其他地方

| 做法 | 為什麼 |
|------|--------|
| 狀態變化立刻送，其餘 0.5 秒一次 | 控制迴路 10 Hz，但位置只要「大概」。每個 tick 都送等於一秒十次 dict 建構加 json.dumps，去重畫一個幾乎沒動的數字 |
| 操作台完全不跑出發前檢查 | 檢查會另外開載入 torch 的程序。在記憶體已經吃緊的機器上，那是讓 OOM killer 決定要殺誰。改成在終端機自己跑 `preflight.py` |
| SSE 連線上限 4 | 每條串流佔一條執行緒。手機重連而舊連線還沒被回收時會累積 |
| 任務輸出只留 200 行 | 每一行都是一個 Python 字串，活到任務結束 |

伺服器本身約 **27 MB**（有相機那版是 49 MB）。操作台右上角會顯示機器人剩餘記憶體，
≥88% 轉黃、≥95% 轉紅 —— 這件事本來只能 SSH 進去打 `free -m` 才看得到，
而那正是這個操作台想取代的終端機。

---

## 地圖

`/api/route` 回傳 route.yaml 的航點與垃圾桶位置（地圖框、公尺），前端自動抓範圍
縮放。**路線和機器人位置用的是同一個轉換** —— 對不起來的地圖比沒有地圖更容易
讓人做錯決定。長寬比也保留：走廊就該畫成走廊，不是拉成正方形填滿畫面。

路線讀不到時地圖留白並說明原因，其餘功能不受影響。

---

## 安全設計

`--allow-real` 與介面確認是兩道必要閘：

1. 伺服器是以 `--allow-real` 啟動的 —— 有人站在機器人旁邊、在機器人上打的指令
2. 該次請求帶著操作者的確認 —— 在「任務設定」勾選

單獨一個都不夠。除此之外，各模式仍需 serial owner、LiDAR 方向、相機／homography
等可驗證證據。操作台目前沒有這些欄位，所以 server 會拒絕所有 A/B/C 實機請求；
dry-run 與 `--simulate` 可正常使用。

**停止鈕是軟停止。** 它送的是 `SIGINT`，跟 Ctrl+C 一樣，走任務程序自己的關機
路徑 —— 那條路徑才會把輪子歸零、釋放伺服匯流排。不用 `SIGKILL`，因為直接砍掉
會跳過那些動作，留下一台還在動、而且沒有程序持有序列埠的機器人。

夾取是一個阻塞呼叫。夾取途中按停止，會在那次夾取動作結束後才生效。
**真正的急停是電源開關**，網頁上不是。這句話也直接寫在介面上。

**指令由伺服器組出來。** 前端送的是設定，不是指令字串。每個值在變成 argv 之前
都會被檢查與夾範圍 —— 類別名稱、路線路徑、物體高度、圈數。前端是唯一預期的
呼叫者，但請求是從網路來的，「介面不會送出那種值」不是伺服器可以假設的事。

---

## API

| 方法 | 路徑 | 用途 |
|------|------|------|
| GET | `/api/events` | SSE。事件：`snapshot` `status` `process` `preflight` `estop` `log` `system`（記憶體）|
| GET | `/api/state` | 目前完整快照（SSE 的 `snapshot` 同內容） |
| GET | `/healthz` | 存活檢查 |
| POST | `/api/mission/preview` | 設定 → 指令字串，或拒絕的理由 |
| POST | `/api/mission/start` | 啟動任務程序 |
| POST | `/api/mission/confirm` | 回答任務程序的 `input()`：開始，或清除暫停 |
| POST | `/api/mission/stop` | 送出 SIGINT |
| POST | `/api/estop` | 送出 SIGINT 並鎖定 |
| POST | `/api/estop/clear` | 解除鎖定 |
| POST | `/api/route` | route.yaml 的航點與垃圾桶位置 |

---

## 手動產生 QR code

IP 會隨連到的網路改變，所以 QR code 也要跟著變，否則掃到的是過期的位址。

```bash
python3 ui/make_qr.py            # 產生一次
python3 ui/make_qr.py --watch    # IP 變了就重畫
python3 ui/make_qr.py --print-url
```

需要 `pip3 install qrcode pillow`。一般使用不需要執行這些指令，桌面 QR
啟動器會自動偵測 IP、啟動伺服器並顯示 QR Code。

偵測 IP 用的是「開一個朝外的 UDP socket，讓核心自己選路由介面再讀回來」
（不會真的送封包）。`hostname -I` 會把 docker 橋接、ROS 的虛擬網卡全列出來，
分不出哪一個手機連得到。

### 比賽當天：考慮改用熱點

Yahboom 官方 App 的做法是讓機器人自己開 Wi-Fi 熱點，手機去連它。熱點模式下
位址是固定的，QR code 永遠不會過期，也不依賴場地的網路（可能要密碼、可能有
登入頁、可能根本連不上）。代價是連著熱點時手機沒有網路。

操作台不用改 —— QR code 裡編的就是當下有效的位址。

---

## 測試

```bash
python3 ui/test_server.py                          # 45 項，全部離線
python3 ui/launcher.py --selftest
python3 integration/mission_status.py --selftest
python3 integration/mission_pipeline.py --selftest
```

**上機前務必跑一次**（只有在機器人上才驗證得到的東西）：

```bash
./ui/jetson_check.sh
```

它檢查連接埠綁不綁得起來、防火牆有沒有擋、UDP 遙測迴路通不通、序列埠有沒有
被別人占用、QR code 的網址對不對、route.yaml 讀不讀得到。
不碰馬達、不開伺服匯流排、不啟動任務，隨時可以跑。

沒有機器人時想看遙測長什麼樣：

```bash
python3 integration/mission_status.py --listen 8099
```
