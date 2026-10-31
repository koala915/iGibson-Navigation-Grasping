#!/usr/bin/env python3
"""Servo bus diagnostic — reads only, never writes. The arm cannot move.

Why this exists
---------------
Three real runs failed with "invalid reading -1" on three different sets of
servos: S1 at policy step 1, S6 at step 49, and S3+S4 on the very first read at
startup before a single command had been sent. Nothing about a cable or a
position explains that spread, and the startup case rules out contention with
our own writes — there had not been any.

The driver source does explain it. Rosmaster_Lib.get_uart_servo_value:

    self.__read_id = 0
    self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
    timeout = 30
    while timeout > 0:
        if self.__read_id > 0:
            return self.__read_id, self.__read_val      # <-- whoever answered

It returns the first response to land, with no check that it came from the servo
that was asked. get_uart_servo_angle then compares the ids itself, finds a
mismatch, and returns -1 — discarding a good reading and reporting a failure for
a servo that answered perfectly well.

On a busy half-duplex chain that turns one late response into a cascade: S2's
answer is collected by S3's request, so S3 reports -1 while its own answer is
collected by S4, and so on down the chain. The visible symptom is a run of
ADJACENT servos unreadable at once, which is exactly S3+S4.

This probe measures that directly, because it asks for the raw
(responding_id, value) pair instead of the angle wrapper that throws the id away:

  * misdirected  — a response arrived, labelled with a different servo. The
                   pipeline is shifted. This is the defect above, and the read
                   layer in x3plus_real_grasp.py now recovers from it.
  * timeout      — no response at all within the driver's 30 ms budget. This is
                   a wiring, power, or servo fault, and no amount of retiming
                   fixes it.
  * out of range — a response arrived and decoded to an impossible angle. Bad
                   frame or a servo reporting nonsense.

Those three have completely different fixes, and until now they were all just
"-1".

Usage
-----
    python3 bus_probe.py                 # 50 rounds at full speed
    python3 bus_probe.py -n 200          # longer sample
    python3 bus_probe.py --gap-ms 20     # space the reads out; if misdirects
                                         # drop to zero, --bus-quiet-ms is the cure
    python3 bus_probe.py --no-array      # per-servo path only

Run it in whatever pose the arm is already in — especially right after a failure,
before power-cycling. Power-cycling destroys the evidence.
"""
from __future__ import annotations

import argparse
import sys
import time

import x3plus_real_grasp as X


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("-n", "--rounds", type=int, default=50,
                    help="How many times to read all six servos (default 50).")
    ap.add_argument("--gap-ms", type=float, default=0.0,
                    help="Pause between individual reads. Raising this until "
                         "misdirects vanish measures the right --bus-quiet-ms.")
    ap.add_argument("--no-array", action="store_true",
                    help="Skip get_uart_servo_angle_array; probe the per-servo path only.")
    ap.add_argument("--port", type=str, default="/dev/myserial")
    args = ap.parse_args()

    if not X._ROSMASTER_AVAILABLE:
        print("[FATAL] Rosmaster_Lib is not importable — nothing to probe.")
        return 2

    cfg = X.DeployConfig(serial_port=args.port)
    servo = X.ServoController(cfg, dry_run=False)
    dev = servo.device
    if dev is None:
        print("[FATAL] No Rosmaster device after init.")
        return 2

    gap = max(0.0, args.gap_ms / 1000.0)
    rounds = max(1, args.rounds)

    # per-servo tallies, indexed 0..5
    answered_self = [0] * 6      # asked S_i, S_i replied
    got_misdirect = [0] * 6      # asked S_i, someone else replied
    timed_out = [0] * 6          # asked S_i, nobody replied
    out_of_range = [0] * 6       # replied, but the value decodes to an impossible angle
    spoke_for = [0] * 6          # how often S_i's answer turned up on someone else's turn
    last_deg = [None] * 6
    last_raw = [None] * 6

    arr_calls = arr_full = arr_partial = arr_empty = 0
    arr_servo_ok = [0] * 6

    print(f"\nProbing {rounds} rounds on {args.port}"
          f"{'' if gap == 0 else f', {args.gap_ms:.0f} ms between reads'}"
          f"{' (per-servo only)' if args.no_array else ''}. No writes are issued.\n")

    try:
        for _ in range(rounds):
            if not args.no_array:
                arr_calls += 1
                try:
                    arr = dev.get_uart_servo_angle_array()
                except Exception:
                    arr = None
                good = 0
                if arr and len(arr) == 6:
                    for i, a in enumerate(arr):
                        if isinstance(a, (int, float)) and a >= 0:
                            arr_servo_ok[i] += 1
                            good += 1
                if good == 6:
                    arr_full += 1
                elif good:
                    arr_partial += 1
                else:
                    arr_empty += 1
                if gap:
                    time.sleep(gap)

            for sid in range(1, 7):
                try:
                    got = dev.get_uart_servo_value(sid)
                    read_id, raw = int(got[0]), got[1]
                except Exception:
                    read_id, raw = -1, -1
                if read_id < 1 or read_id > 6:
                    timed_out[sid - 1] += 1
                else:
                    deg = X._rosmaster_raw_to_deg(read_id, raw)
                    if deg is None:
                        out_of_range[read_id - 1] += 1
                    else:
                        last_deg[read_id - 1] = deg
                        last_raw[read_id - 1] = raw
                    if read_id == sid:
                        answered_self[sid - 1] += 1
                    else:
                        got_misdirect[sid - 1] += 1
                        spoke_for[read_id - 1] += 1
                if gap:
                    time.sleep(gap)
    except KeyboardInterrupt:
        print("\n[Interrupted] reporting what was collected so far.\n")
    finally:
        closer = getattr(servo, "_close_device", None)
        if closer:
            try:
                closer()
            except Exception:
                pass

    misdirect_total = sum(got_misdirect)
    timeout_total = sum(timed_out)
    range_total = sum(out_of_range)

    print("=" * 74)
    print("每顆伺服機（問誰 → 誰回答）")
    print("=" * 74)
    print(f"{'servo':>6}{'自己回答':>10}{'別人代answer':>14}{'完全沒回應':>12}"
          f"{'超出範圍':>10}{'最後讀值':>12}")
    for i in range(6):
        deg = f"{last_deg[i]:.0f}°" if last_deg[i] is not None else "—"
        raw = f" (raw {last_raw[i]})" if last_raw[i] is not None else ""
        print(f"{'S'+str(i+1):>6}{answered_self[i]:>10}{got_misdirect[i]:>14}"
              f"{timed_out[i]:>12}{out_of_range[i]:>10}{deg+raw:>12}")

    if not args.no_array:
        print()
        print("=" * 74)
        print("整批讀取 get_uart_servo_angle_array")
        print("=" * 74)
        print(f"  呼叫 {arr_calls} 次：六顆全有 {arr_full}、部分 {arr_partial}、全空 {arr_empty}")
        print("  每顆有效次數：" +
              ", ".join(f"S{i+1}={arr_servo_ok[i]}" for i in range(6)))

    print()
    print("=" * 74)
    print("判讀")
    print("=" * 74)
    verdicts = []
    if misdirect_total:
        worst = max(range(6), key=lambda i: spoke_for[i])
        verdicts.append(
            f"▸ 匯流排管線錯位已證實：{misdirect_total} 次回應貼著別顆的 ID 回來"
            f"（最常代答的是 S{worst+1}，{spoke_for[worst]} 次）。\n"
            f"  這就是 get_uart_servo_angle 回 -1 的原因——讀值是好的，只是被丟掉。\n"
            f"  x3plus_real_grasp.py 的新讀取層會把答案記到真正回答的那顆頭上，\n"
            f"  所以這些不再是失敗。若次數很高，再用 --bus-quiet-ms 拉開間隔。")
    if timeout_total:
        dead = [f"S{i+1}×{timed_out[i]}" for i in range(6) if timed_out[i]]
        verdicts.append(
            f"▸ 真正無回應 {timeout_total} 次（{', '.join(dead)}）。\n"
            f"  這不是時序問題，重試也救不回來：查該顆的接線、接頭與供電。")
    if range_total:
        bad = [f"S{i+1}×{out_of_range[i]}" for i in range(6) if out_of_range[i]]
        verdicts.append(
            f"▸ 解碼超出範圍 {range_total} 次（{', '.join(bad)}）——封包壞掉或伺服機回報亂數。")
    if not args.no_array and arr_calls and arr_empty == arr_calls:
        verdicts.append(
            "▸ 整批讀取從頭到尾全空：這塊板子不服務 FUNC_ARM_CTRL，\n"
            "  每次讀取都白花 30 ms 等逾時。讀取層會在三次後自動放棄它。")
    elif not args.no_array and arr_full == arr_calls and arr_calls:
        verdicts.append(
            "▸ 整批讀取每次都完整回來——這是最省匯流排的路徑，維持優先使用。")
    if not verdicts:
        verdicts.append("▸ 這次取樣沒有任何異常。若實跑仍失敗，代表故障只在手臂"
                        "移動／負載時出現，請在失敗當下立刻重跑這支。")
    for v in verdicts:
        print(v)
    print("=" * 74)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
