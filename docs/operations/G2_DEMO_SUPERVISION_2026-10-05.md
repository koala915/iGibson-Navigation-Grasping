# Demo 監督與建圖濾波候選（2026-10-05）

本次為本機開發，尚未部署 Jetson 或實車驗收。不是完整自主巡航。

## 速度監督

`integration/g2_demo_supervisor.py` 接 localhost UDP 7002 的新意圖，再以
20 Hz 發到既有 TCP 7000；仍只有 grasp-service 持有 serial。
發送者每 0.1 秒更新 `{"seq":1,"vx":0.15,"ttl":0.3}`，seq 必須遞增。
vx=0 是明確結束；意圖過期也結束，不自動重播舊目標。
一次程序只有一段 0.10–0.25 m 直行，最多 2.5 秒，不支持轉彎、倒車或 AMCL 巡航。
發送端必須依實際任務目標重新計算意圖，不能無限心跳延長失效的任務。

保留 raw /scan 的 0.42 m 立即煞停；LiDAR 混入單幀雜訊也先停，之後
連續淨空 0.5 秒且意圖仍新鮮才恢復，最多兩次。停車等待算入時間限制。
感測過期／錯誤、odom 跳變、偏航／側移、無進展、RAM 低於 768 MB 均結束。
退出送 stop、斷開 TCP；服務既有 0.5 秒 watchdog 保留。
不會自動重啟 grasp-service（重啟會移臂並重設 odom），不把卡輪直接判成通訊故障。
啟動前執行既有姿態／序列 owner／方向證據檢查，行駛中不讀伺服。

```bash
cd /home/jetson/Documents/deploy_jetson2
/home/jetson/grasp_venv/bin/python3 integration/g2_demo_supervisor.py \
  --distance-m 0.15 \
  --lidar-orientation-evidence /home/jetson/.route_b_runtime/scan_orientation_verified \
  --i-confirm-clear-path --i-confirm-power-cut
```

底盤 `_write` 略過相同馬達值，因此 TCP 心跳不代表每次重寫控制板。
這是待量測假說，未改底盤的去重行為。應記錄 command age、板端 age、odom
與實際輪子，再決定是否需要有上限的板端重送；不能把所有停車都當作故障。

## 建圖候選

`deploy/ros/g2_mapping_filter.py` 保留原始 /scan，輸出專供 GMapping 的
/scan_mapping。預設只保留車頭 ±90°（raw 前方為 ±180°），近於 driver range_min
及大於 3 m 的點遮罩。不變更 scan frame、陣列順序、TF 或取得時戳。
近距離不等於雜訊：沒有新增 0.27 m 全周盲區。車體矩形及孤立點濾波需
另以 `_footprint:=true`、`_speckle:=true` 啟用，預設關閉，待線材／紙板對照。
近場真實牆、細物件可能被濾波刪除，所以輸出不可取代安全 scan。

被遮罩的值為 4 m、output range_max 為 3 m。搭配 maxRange=3、maxUrange=3，
OpenSLAM 在建圖／active area 對超限射線直接跳過，不把遮罩區畫成 free。
這是 mapper 專用的超限 sentinel，不能接到把無回波當作空地的安全模組。
GMapping wrapper 將小於 range_min 的值換成 range_max，故不要用零值遮罩。
前視建圖需要轉向及沿各牆取得多視角；不能保證比 360° 圖更準。

`g2_demo_mapping.py` 啟動 filter 和 GMapping，不發底盤命令。
需先 source ROS Melodic／工作空間，且不能同時有 AMCL、map_server 或另一個 GMapping。
預設 5 分鐘，最多 10 分鐘；只保留一筆 scan、10 particles、2 cm grid、3 m range。
開始需 MemAvailable >=1024 MB，執行中低於 768 MB 即停止兩個子程序。
prlimit 虛擬位址上限：filter 256 MiB、GMapping 800 MiB；可能需依 Jetson
實際依賴調整，不能把這些候選值描述成實機已驗證。儲存地圖需在逾時前執行 map_saver。

```bash
/usr/bin/python3 deploy/ros/g2_demo_mapping.py --seconds 300
```

RAM 閘門及配置降低耗盡機率，不能保證其他程序不突然大量分配記憶體。
本次沒有替既有 arm／vision／所有舊入口套上限制；其實測基線約 grasp 505 MB、
vision 298 MB、ROS 全套 561 MB。每個入口逐一量測峰值後再設 systemd MemoryHigh／MemoryMax，
不直接限制 serial owner 而造成 OOM kill。可用 `/proc/meminfo` 和 `systemd-cgtop` 觀察。

## 一手來源與案例

- [ROS laser_filters](https://github.com/ros-perception/laser_filters)：range、angular、box／footprint、speckle 等濾波實作；可重用的機制，不能視為本車驗收。
- [Clearpath Jackal gmapping 配置](https://github.com/jackal/jackal/blob/noetic-devel/jackal_navigation/launch/include/gmapping.launch)：預設 front/scan、10 particles、0.02 m grid；是真實平台配置，未提供與本車等同的成功率。
- [ROS Melodic GMapping wrapper](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/src/slam_gmapping.cpp)：range_min 對應處理、particles、更新門檻。
- [OpenSLAM scanmatcher](https://github.com/ros-perception/openslam_gmapping/blob/master/scanmatcher/scanmatcher.cpp)：超過 laserMaxRange 時略過射線，不做 free-space traversal。

## 驗證與下一步

本機 11 項測試通過：命令過期／重播、非法速度、立即停車、恢復去抖／次數上限、
前方 raw 角度、遮罩值、孤立點 opt-in、MemAvailable。Python 編譯通過。
尚未驗證：Jetson ROS 實際濾波與 TF、prlimit 子程序、低 RAM 全路徑、UDP/TCP
掉線實測、恢復停車距離與新地圖比較。先充滿電後做不動車 probe、bag A/B，
再單段 15 cm 人工受監督試驗；不自動啟動 demo 或開機移動。
