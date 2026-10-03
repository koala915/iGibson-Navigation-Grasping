# Odom 前進／後退校正（2026-09-24）

## 範圍

本次只校正 `get_motion_data()` 的縱向 `vx`，不修改橫移 `vy` 或旋轉 `wz`。

- Jetson：`yahboom`，`/dev/myserial -> /dev/ttyUSB1`
- 命令：`set_car_motion(vx, 0, 0)`
- 命令速度：正負 `0.10 m/s`
- 命令時間：4.0 s
- feedback 取樣：25 Hz，並包含停止後 1 秒
- 車體前後路徑淨空，輪子落地

## 採用資料

第一次試跑只用於確認流程，操作員要求重新量測，因此不納入結果。第二次量測：

| 方向 | raw vx 積分 | 實際有號距離 | 個別 scale |
|---|---:|---:|---:|
| 前進 | `+0.3678271529 m` | `+0.355 m` | `0.9651272268` |
| 後退 | `-0.3596728915 m` | `-0.348 m` | `0.9675458123` |

前進後再後退，最後位於原始起點前方 0.7 cm；這與兩段實測距離差 0.7 cm 一致。

原始資料：

- `odom_linear_2026-09-24/forward.json`
- `odom_linear_2026-09-24/backward.json`

## 計算

以通過原點的 least-squares 擬合：

```text
actual_distance = linear_scale * raw_vx_integral
linear_scale = sum(raw * actual) / sum(raw^2)
```

結果：

```text
linear_scale = 0.9663094139825343
RMSE = 0.0004398 m
forward/backward scale spread = 0.2503%
```

raw `vx` 的符號與 ROS `+x` 一致，所以不加負號。

## 套用方式

`integration/feedback_odom.py` 使用：

```text
vx = vx_raw * 0.966309414
vy = vy_raw * 0.65
```

`vy=0.65` 是未重測的歷史值，刻意獨立保留。本文件完成後，同日另做了左右
旋轉校正；現行 yaw 數值與原始資料見 `ODOM_YAW_CALIBRATION_2026-09-24.md`。

## 後續驗證

更新後應另外做一趟至少 1 m 的直線 holdout，不再用這兩筆擬合資料：

1. odom 與實際距離誤差應在正負 3% 內。
2. 確認 odom x 前進為正、後退為負。
3. 記錄 y 漂移與 yaw 漂移，但不以本次結果調整旋轉 scale。
