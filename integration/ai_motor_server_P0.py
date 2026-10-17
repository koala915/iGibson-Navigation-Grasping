#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Snapshot of the Jetson maintenance-mode P0 server copied on 2026-09-28
# from /home/jetson/ROS/X3/yahboomcar_ws/src/yahboomcar_bringup/scripts/.
# Original SHA256: 0e6a926e2991a3939a4c3d40a904fdc14470916ce9abad7c9cf4f5b443963a26.
# TCP 7000 accepts one JSON object per line:
#   velocity: vx (m/s), wz (rad/s), both required; continuous wheel command.
#   stop: no fields; wheel output (0, 0, 0, 0).
#   wheel: m1..m4 integer motor efforts, each limited to +/-30.
#   estop / clear_estop / status: no fields; latch, clear, or report safety state.
#   arm_pose: pose name string; stop wheels before and after the arm movement.
# It does NOT accept the legacy action/speed forward/turn_left format.
# A 0.50 s command watchdog stops the wheels. At 20 Hz it publishes
# /odom_setmotor (nav_msgs/Odometry) and /tf (odom -> base_footprint) from
# Rosmaster.get_motion_data() feedback; commanded velocity is not odometry.
# Keep deploy/systemd/start-navigation.sh pointed at the original Jetson file.
# This snapshot also imports Jetson's route_a_core/motion_state_monitor.py;
# copying this file alone is not a standalone replacement for that deployment.
# Its odom scales are the older 0.98 / 0.501 values. G2's active service uses
# the 2026-09-24 measured feedback_odom.py scales (0.966 / about 0.985).

"""
ai_motor_server_P0.py

用途：
1. TCP 7000 接收 Windows RL 傳來的目標 vx / wz。
2. 將 vx / wz 轉成四輪連續馬達值。
3. 使用 Rosmaster get_motion_data() 取得實際 vx / vy / wz。
4. 發布 /odom_setmotor。
5. 發布 TF：odom -> base_footprint。
6. 具備 0.5 秒 watchdog，Windows 斷線時立即停車。

重要：
- 這是 P0 unified runtime；以 exact ai_motor_server_RL.py 為 base。
- 保留已實車驗證的 continuous vx/wz motor mapping。
- 新增 Route A exact feedback health / stationary semantics 與 ESTOP latch。
- 不修改任何 historical ai_motor_server_B.py / ai_motor_server_RL.py。
"""

import json
import math
import os
import socket
import sys
import threading
import time

import rospy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from Rosmaster_Lib import Rosmaster

# Route A exact health/stationary core. Keep these source files read-only.
ROUTE_A_CORE_DIR = os.environ.get(
    "P0_ROUTE_A_CORE_DIR",
    "/home/jetson/ROS/X3/yahboomcar_ws/src/yahboomcar_bringup/scripts/route_a_core"
)
if ROUTE_A_CORE_DIR not in sys.path:
    sys.path.insert(0, ROUTE_A_CORE_DIR)

from motion_state_monitor import MotionStateMonitor


# ============================================================
# 1. TCP 設定
# ============================================================

HOST = "0.0.0.0"
PORT = 7000

# Windows 超過這段時間沒有送新命令，就停車
WATCHDOG_TIMEOUT_SEC = 0.50


# ============================================================
# 2. 馬達安全限制
# ============================================================

# Rosmaster 理論最大值
ABSOLUTE_MAX_MOTOR = 60

# 第一次 RL 測試先限制到 30
RL_MOTOR_LIMIT = 30

# 四輪都很小時，馬達可能不會啟動
MIN_ACTIVE_MOTOR = 18.0


# ============================================================
# 3. RL 速度命令限制
# ============================================================

CMD_MAX_FORWARD_VX = 0.15
CMD_MAX_REVERSE_VX = 0.08
CMD_MAX_ABS_WZ = 0.40

CMD_VX_DEADBAND = 0.015
CMD_WZ_DEADBAND = 0.03


# ============================================================
# 4. vx / wz 轉馬達初始係數
# ============================================================

# vx=+0.15 m/s 時：
# base_motor 約等於 30
LINEAR_MOTOR_PER_MPS = 200.0

# wz=+0.40 rad/s 時：
# turn_motor 約等於 10
#
# 這是第一版低速測試值，之後要根據：
# cmd wz 與 odom wz 的差異再校正。
ANGULAR_MOTOR_PER_RAD_S = 25.0


# ============================================================
# 5. 組員原本 feedback odom 校正值
# ============================================================

FEEDBACK_LINEAR_SCALE = 0.98

FEEDBACK_ANGULAR_LEFT_SCALE = 0.501
FEEDBACK_ANGULAR_RIGHT_SCALE = 0.501

ODOM_RATE_HZ = 20.0
FEEDBACK_TIMEOUT_SEC = 0.30


# ============================================================
# 6. 全域狀態
# ============================================================

bot = None

motor_lock = threading.Lock()
command_lock = threading.Lock()
odom_lock = threading.Lock()
safety_lock = threading.RLock()

motion_state_monitor = MotionStateMonitor(
    vx_threshold=0.02,
    vy_threshold=0.02,
    wz_threshold=0.05,
    required_consecutive=3,
    feedback_timeout=FEEDBACK_TIMEOUT_SEC
)
estop_latched = False
feedback_motion_blocked = True
last_safety_reason = "startup_waiting_for_feedback"

last_command_time = 0.0
last_motor_values = None

odom_x = 0.0
odom_y = 0.0
odom_yaw = 0.0

shutting_down = False

last_velocity_log_time = 0.0
last_velocity_log_value = None


# ============================================================
# 7. 基本工具
# ============================================================

def clamp(value, minimum, maximum):
    return max(
        minimum,
        min(maximum, value)
    )


def wrap_angle(angle):
    """
    正規化到 [-pi, pi)。
    """

    return (
        angle + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi


def mark_command_received():
    global last_command_time

    with command_lock:
        last_command_time = time.time()


def get_last_command_age():
    with command_lock:
        timestamp = last_command_time

    if timestamp <= 0.0:
        return float("inf")

    return time.time() - timestamp


def invalidate_command_lease():
    global last_command_time
    with command_lock:
        last_command_time = 0.0


def get_safety_status(now=None):
    current = time.time() if now is None else float(now)
    with safety_lock:
        feedback = motion_state_monitor.status(now=current)
        return {
            "estop_latched": bool(estop_latched),
            "feedback_motion_blocked": bool(feedback_motion_blocked),
            "last_safety_reason": last_safety_reason,
            "feedback_valid": bool(feedback["feedback_valid"]),
            "feedback_fresh": bool(feedback["feedback_fresh"]),
            "feedback_age": feedback["feedback_age"],
            "stationary": bool(feedback["stationary"]),
            "consecutive_stationary_samples": feedback["consecutive_stationary_samples"],
            "required_consecutive": feedback["required_consecutive"],
            "last_feedback_error": feedback["last_error"],
        }


def _refresh_feedback_block(now=None):
    global feedback_motion_blocked
    global last_safety_reason
    current = time.time() if now is None else float(now)
    feedback = motion_state_monitor.status(now=current)
    if not feedback["feedback_valid"]:
        feedback_motion_blocked = True
        if feedback["last_error"]:
            last_safety_reason = "feedback_invalid: {}".format(feedback["last_error"])
        else:
            last_safety_reason = "feedback_invalid_or_stale"
    return feedback


def record_feedback_success(vx, vy, wz, stamp=None):
    global feedback_motion_blocked
    global last_safety_reason
    with safety_lock:
        motion_state_monitor.record_feedback(vx, vy, wz, stamp=stamp)
        feedback = motion_state_monitor.status(now=stamp)
        if feedback["feedback_valid"]:
            feedback_motion_blocked = False
            if not estop_latched:
                last_safety_reason = None
        return feedback


def record_feedback_failure(error, stamp=None):
    global feedback_motion_blocked
    global last_safety_reason
    with safety_lock:
        motion_state_monitor.record_failure(error=error, stamp=stamp)
        feedback_motion_blocked = True
        last_safety_reason = "feedback_failure: {}".format(error)
    # Route A feedback failure is not an ESTOP latch, but P0 fails closed:
    # stop immediately and require a later valid feedback + NEW command.
    invalidate_command_lease()
    stop_motors()


def engage_estop(reason="explicit_estop"):
    global estop_latched
    global last_safety_reason
    with safety_lock:
        estop_latched = True
        last_safety_reason = str(reason)
    invalidate_command_lease()
    stop_motors()


def clear_estop():
    global estop_latched
    global last_safety_reason
    with safety_lock:
        estop_latched = False
        _refresh_feedback_block(now=time.time())
        if feedback_motion_blocked:
            last_safety_reason = "estop_cleared_feedback_not_ready"
        else:
            last_safety_reason = "estop_cleared_waiting_for_new_command"
    # Explicit clear never resumes the old command.
    invalidate_command_lease()
    stop_motors()


# ============================================================
# 8. Rosmaster 初始化
# ============================================================

def initialize_rosmaster():
    global bot

    print("[INFO] 初始化 Rosmaster...")

    bot = Rosmaster(com="/dev/myserial")

    # Yahboom Rosmaster_Lib 通常需要接收執行緒，
    # 才能正常取得 get_motion_data()。
    if hasattr(bot, "create_receive_threading"):
        bot.create_receive_threading()

    # 開啟底層自動回報
    try:
        bot.set_auto_report_state(
            enable=True,
            forever=False
        )

    except TypeError:
        # 相容某些舊版 Rosmaster_Lib
        bot.set_auto_report_state(
            True,
            False
        )

    print("[INFO] Rosmaster 初始化完成。")


# ============================================================
# 9. 實際四輪馬達輸出
# ============================================================

def set_motors(m1, m2, m3, m4):
    """
    實際呼叫 Rosmaster.set_motor()。

    M1、M2：左側輪
    M3、M4：右側輪
    """

    global last_motor_values

    m1 = int(round(clamp(
        m1,
        -ABSOLUTE_MAX_MOTOR,
        ABSOLUTE_MAX_MOTOR
    )))

    m2 = int(round(clamp(
        m2,
        -ABSOLUTE_MAX_MOTOR,
        ABSOLUTE_MAX_MOTOR
    )))

    m3 = int(round(clamp(
        m3,
        -ABSOLUTE_MAX_MOTOR,
        ABSOLUTE_MAX_MOTOR
    )))

    m4 = int(round(clamp(
        m4,
        -ABSOLUTE_MAX_MOTOR,
        ABSOLUTE_MAX_MOTOR
    )))

    values = (
        m1,
        m2,
        m3,
        m4
    )

    nonzero = any(value != 0 for value in values)
    if nonzero:
        with safety_lock:
            feedback = _refresh_feedback_block(now=time.time())
            if estop_latched:
                raise RuntimeError("motor command blocked: ESTOP latched")
            if feedback_motion_blocked or not feedback["feedback_valid"]:
                raise RuntimeError("motor command blocked: feedback invalid/stale")

    with motor_lock:
        bot.set_motor(
            m1,
            m2,
            m3,
            m4
        )

    # 只有馬達值改變時才列印，避免一直洗版
    if values != last_motor_values:
        print(
            "[MOTOR] "
            "m1={} m2={} m3={} m4={}".format(
                m1,
                m2,
                m3,
                m4
            )
        )

        last_motor_values = values


def stop_motors():
    set_motors(
        0,
        0,
        0,
        0
    )


# ============================================================
# 10. vx / wz 轉四輪馬達
# ============================================================

def velocity_to_motor_values(vx, wz):
    """
    將目標車體速度轉成四輪馬達命令。

    定義：
        vx > 0：前進
        vx < 0：後退

        wz > 0：左轉
        wz < 0：右轉

    差速混合：
        base_motor = vx 造成的前後動力
        turn_motor = wz 造成的左右輪差

        left  = base - turn
        right = base + turn

    左轉時：
        左輪較慢
        右輪較快

    右轉時：
        左輪較快
        右輪較慢
    """

    vx = float(clamp(
        vx,
        -CMD_MAX_REVERSE_VX,
        CMD_MAX_FORWARD_VX
    ))

    wz = float(clamp(
        wz,
        -CMD_MAX_ABS_WZ,
        CMD_MAX_ABS_WZ
    ))

    if abs(vx) < CMD_VX_DEADBAND:
        vx = 0.0

    if abs(wz) < CMD_WZ_DEADBAND:
        wz = 0.0

    base_motor = (
        vx
        * LINEAR_MOTOR_PER_MPS
    )

    turn_motor = (
        wz
        * ANGULAR_MOTOR_PER_RAD_S
    )

    left_motor = (
        base_motor
        - turn_motor
    )

    right_motor = (
        base_motor
        + turn_motor
    )

    peak = max(
        abs(left_motor),
        abs(right_motor)
    )

    # 若整體命令非零，但兩側都太小，
    # 等比例提高到能啟動馬達的程度。
    #
    # 不會個別強迫內輪變大，
    # 因為內輪接近 0 可能是正常的急彎。
    if (
        peak > 0.0
        and peak < MIN_ACTIVE_MOTOR
    ):
        scale_up = (
            MIN_ACTIVE_MOTOR
            / peak
        )

        left_motor *= scale_up
        right_motor *= scale_up

        peak = max(
            abs(left_motor),
            abs(right_motor)
        )

    # 超過第一次測試限制時，
    # 等比例縮小，保留左右輪比例。
    if peak > RL_MOTOR_LIMIT:
        scale_down = (
            RL_MOTOR_LIMIT
            / peak
        )

        left_motor *= scale_down
        right_motor *= scale_down

    m1 = int(round(
        left_motor
    ))

    m2 = int(round(
        left_motor
    ))

    m3 = int(round(
        right_motor
    ))

    m4 = int(round(
        right_motor
    ))

    return (
        m1,
        m2,
        m3,
        m4
    )


def run_velocity(vx, wz):
    """
    執行連續速度命令。

    注意：
    這裡的 vx/wz 是「目標命令」。

    odom 使用的 vx/wz 仍然來自：
        bot.get_motion_data()

    兩者不是同一個值。
    """

    global last_velocity_log_time
    global last_velocity_log_value

    vx = float(clamp(
        vx,
        -CMD_MAX_REVERSE_VX,
        CMD_MAX_FORWARD_VX
    ))

    wz = float(clamp(
        wz,
        -CMD_MAX_ABS_WZ,
        CMD_MAX_ABS_WZ
    ))

    m1, m2, m3, m4 = (
        velocity_to_motor_values(
            vx,
            wz
        )
    )

    now = time.time()

    log_value = (
        round(vx, 3),
        round(wz, 3),
        m1,
        m2,
        m3,
        m4
    )

    if (
        log_value
        != last_velocity_log_value
        or now
        - last_velocity_log_time
        >= 1.0
    ):
        print(
            "[VELOCITY] "
            "cmd vx={:+.3f} "
            "wz={:+.3f} "
            "-> left={} right={}".format(
                vx,
                wz,
                m1,
                m3
            )
        )

        last_velocity_log_value = (
            log_value
        )

        last_velocity_log_time = now

    set_motors(
        m1,
        m2,
        m3,
        m4
    )


# ============================================================
# 11. Feedback odometry
# ============================================================

def convert_feedback(vx_raw, vy_raw, wz_raw):
    """Preserve the current authoritative RL feedback conversion.

    IMPORTANT: vx sign remains unresolved until P0-D real-robot evidence.
    P0-C intentionally keeps +vx_raw * 0.65 exactly as ai_motor_server_RL.py.
    """
    values = (float(vx_raw), float(vy_raw), float(wz_raw))
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite feedback: {!r}".format(values))

    vx = values[0] * FEEDBACK_LINEAR_SCALE
    vy = -values[1] * FEEDBACK_LINEAR_SCALE
    wz_unscaled = values[2]
    if wz_unscaled >= 0.0:
        wz = wz_unscaled * FEEDBACK_ANGULAR_LEFT_SCALE
    else:
        wz = wz_unscaled * FEEDBACK_ANGULAR_RIGHT_SCALE
    return vx, vy, wz


def parse_motion_data(motion_data):
    if motion_data is None:
        raise ValueError("get_motion_data returned None")
    try:
        if len(motion_data) < 3:
            raise ValueError("get_motion_data returned fewer than 3 values")
        return convert_feedback(motion_data[0], motion_data[1], motion_data[2])
    except TypeError:
        raise ValueError("get_motion_data returned non-sequence {!r}".format(motion_data))


def integrate_feedback(vx, vy, wz, dt):
    global odom_x
    global odom_y
    global odom_yaw

    dt = float(dt)
    if not math.isfinite(dt) or dt < 0.0:
        raise ValueError("invalid odom dt: {!r}".format(dt))
    dt = clamp(dt, 0.0, 0.20)

    with odom_lock:
        delta_x = (vx * math.cos(odom_yaw) - vy * math.sin(odom_yaw)) * dt
        delta_y = (vx * math.sin(odom_yaw) + vy * math.cos(odom_yaw)) * dt
        odom_x += delta_x
        odom_y += delta_y
        odom_yaw = wrap_angle(odom_yaw + wz * dt)
        return odom_x, odom_y, odom_yaw


def build_odom_tf_messages(current_x, current_y, current_yaw, vx, vy, wz, stamp):
    """Build odom + TF with exactly the same stamp."""
    half_yaw = current_yaw * 0.5
    quaternion = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    odom_message = Odometry()
    odom_message.header.stamp = stamp
    odom_message.header.frame_id = "odom"
    odom_message.child_frame_id = "base_footprint"
    odom_message.pose.pose.position.x = current_x
    odom_message.pose.pose.position.y = current_y
    odom_message.pose.pose.position.z = 0.0
    odom_message.pose.pose.orientation.x = quaternion[0]
    odom_message.pose.pose.orientation.y = quaternion[1]
    odom_message.pose.pose.orientation.z = quaternion[2]
    odom_message.pose.pose.orientation.w = quaternion[3]
    odom_message.twist.twist.linear.x = vx
    odom_message.twist.twist.linear.y = vy
    odom_message.twist.twist.angular.z = wz

    transform_message = TransformStamped()
    transform_message.header.stamp = stamp
    transform_message.header.frame_id = "odom"
    transform_message.child_frame_id = "base_footprint"
    transform_message.transform.translation.x = current_x
    transform_message.transform.translation.y = current_y
    transform_message.transform.translation.z = 0.0
    transform_message.transform.rotation.x = quaternion[0]
    transform_message.transform.rotation.y = quaternion[1]
    transform_message.transform.rotation.z = quaternion[2]
    transform_message.transform.rotation.w = quaternion[3]

    tf_message = TFMessage()
    tf_message.transforms = [transform_message]
    return odom_message, tf_message


def feedback_step(dt, stamp=None):
    """One feedback/odom step for runtime and offline regression tests."""
    now = time.time() if stamp is None else float(stamp)
    pose_before = (odom_x, odom_y, odom_yaw)
    try:
        motion_data = bot.get_motion_data()
        vx, vy, wz = parse_motion_data(motion_data)
        record_feedback_success(vx, vy, wz, stamp=now)
        pose = integrate_feedback(vx, vy, wz, dt)
        return {
            "ok": True,
            "raw": motion_data,
            "velocity": (vx, vy, wz),
            "pose": pose,
            "error": None,
            "safety": get_safety_status(now=now),
        }
    except Exception as exc:
        record_feedback_failure(str(exc), stamp=now)
        return {
            "ok": False,
            "raw": None,
            "velocity": (0.0, 0.0, 0.0),
            "pose": pose_before,
            "error": str(exc),
            "safety": get_safety_status(now=now),
        }


def odom_loop(odom_publisher, tf_broadcaster):
    """20 Hz feedback odom. Failed/malformed feedback freezes pose and fails closed."""
    rate = rospy.Rate(ODOM_RATE_HZ)
    previous_time = time.time()

    while not rospy.is_shutdown() and not shutting_down:
        now = time.time()
        dt = now - previous_time
        previous_time = now
        result = feedback_step(dt=dt, stamp=now)

        if not result["ok"]:
            rospy.logwarn_throttle(2.0, "get_motion_data invalid: {}".format(result["error"]))
            rate.sleep()
            continue

        vx, vy, wz = result["velocity"]
        current_x, current_y, current_yaw = result["pose"]
        stamp = rospy.Time.now()
        odom_message, tf_message = build_odom_tf_messages(
            current_x, current_y, current_yaw, vx, vy, wz, stamp
        )
        odom_publisher.publish(odom_message)
        tf_broadcaster.publish(tf_message)
        rate.sleep()


# ============================================================
# 12. Watchdog
# ============================================================

def watchdog_tick():
    """One deterministic watchdog check; this remains the ONLY timeout owner."""
    command_age = get_last_command_age()
    if command_age > WATCHDOG_TIMEOUT_SEC:
        stop_motors()
        return True
    return False


def watchdog_loop():
    """Windows client silence >0.50 s stops the chassis."""
    while not rospy.is_shutdown() and not shutting_down:
        watchdog_tick()
        time.sleep(0.05)



# ============================================================
# 12.5 手臂姿態控制
# ============================================================

# 組員正式使用的兩個姿態
NAV_HOME_DEG = (
    90.0,
    140.0,
    0.0,
    0.0,
    90.0,
    30.0,
)

# C3：arm camera 對位 / grasp 工作姿態
GRASP_HOME_DEG = (
    90.0,
    67.08,
    9.79,
    9.79,
    90.0,
    30.0,
)

# 每一步最多改約 8 度。
# 模仿組員 guarded move 的保守策略。
ARM_MAX_STEP_DEG = 8.0

# 每一步 servo 執行時間
ARM_STEP_TIME_MS = 300

# 每一步結束後多等一下
ARM_STEP_SETTLE_SEC = 0.08

# 我們自己的「已知姿態」。
# 第一次一定先送 nav_home，之後才能安全做插值。
current_arm_pose_deg = None


def _send_arm_array(angles, run_time_ms):
    """
    透過目前唯一的 Rosmaster bot 控制六軸 servo。
    不建立第二個 Rosmaster serial owner。
    """

    angles = [
        float(v)
        for v in angles
    ]

    with motor_lock:
        fn = bot.set_uart_servo_angle_array

        # 相容不同 Rosmaster_Lib 版本
        try:
            fn(
                angle_s=angles,
                run_time=int(run_time_ms)
            )
            return

        except TypeError:
            pass

        try:
            fn(
                angles,
                int(run_time_ms)
            )
            return

        except TypeError:
            pass

        fn(
            angles,
            run_time=int(run_time_ms)
        )


def move_arm_pose(pose_name):
    """
    支援：
        nav_home
        grasp_home / c3

    規則：
    - 車輪一定先停。
    - 第一次若不知道手臂目前在哪裡，
      只允許先回 nav_home。
    - nav_home <-> C3 之後採小步插值。
    """

    global current_arm_pose_deg

    pose_name = str(
        pose_name
    ).strip().lower()

    if pose_name in (
        "nav_home",
        "nav",
        "travel",
    ):
        target = list(
            NAV_HOME_DEG
        )
        canonical = "nav_home"

    elif pose_name in (
        "grasp_home",
        "grasp",
        "c3",
        "arm_check",
    ):
        target = list(
            GRASP_HOME_DEG
        )
        canonical = "grasp_home"

    else:
        raise ValueError(
            "unknown arm pose: {}".format(
                pose_name
            )
        )

    # Wheels-or-arm-never-both
    stop_motors()

    # --------------------------------------------------------
    # 第一次姿態未知
    # --------------------------------------------------------
    if current_arm_pose_deg is None:

        if canonical != "nav_home":
            raise RuntimeError(
                "arm pose unknown; "
                "send nav_home first"
            )

        print(
            "[ARM] first known move -> nav_home"
        )

        # 第一次慢慢回安全 travel pose
        _send_arm_array(
            target,
            1500
        )

        time.sleep(
            1.7
        )

        current_arm_pose_deg = list(
            target
        )

        print(
            "[ARM] reached nav_home "
            "{}".format(
                [
                    round(v, 2)
                    for v in target
                ]
            )
        )

        return


    start = list(
        current_arm_pose_deg
    )

    max_delta = max(
        abs(
            target[i]
            -
            start[i]
        )
        for i in range(6)
    )

    steps = max(
        1,
        int(
            math.ceil(
                max_delta
                /
                ARM_MAX_STEP_DEG
            )
        )
    )

    print(
        "[ARM] move {} -> {} | "
        "steps={} max_delta={:.1f}deg".format(
            [
                round(v, 1)
                for v in start
            ],
            canonical,
            steps,
            max_delta
        )
    )

    for step in range(
        1,
        steps + 1
    ):

        ratio = (
            float(step)
            /
            float(steps)
        )

        pose = [
            start[i]
            +
            (
                target[i]
                -
                start[i]
            )
            *
            ratio
            for i in range(6)
        ]

        _send_arm_array(
            pose,
            ARM_STEP_TIME_MS
        )

        time.sleep(
            ARM_STEP_TIME_MS
            /
            1000.0
            +
            ARM_STEP_SETTLE_SEC
        )

    current_arm_pose_deg = list(
        target
    )

    print(
        "[ARM] reached {} {}".
        format(
            canonical,
            [
                round(v, 2)
                for v in target
            ]
        )
    )



# ============================================================
# 13. JSON 命令解析
# ============================================================

def require_finite_field(command, name):
    if name not in command:
        raise ValueError("missing required field: {}".format(name))
    value = float(command[name])
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))
    return value

def process_command(command):
    """
    支援的命令：

    1. 連續速度
       {
           "action": "velocity",
           "vx": 0.10,
           "wz": 0.20
       }

    2. 停車
       {
           "action": "stop"
       }

    3. 四輪直接測試
       {
           "action": "wheel",
           "m1": 20,
           "m2": 20,
           "m3": 25,
           "m4": 25
       }
    """

    if not isinstance(
        command,
        dict
    ):
        raise ValueError(
            "命令必須是 JSON object"
        )

    if "action" not in command:
        raise ValueError("missing required field: action")

    action = str(command["action"]).strip().lower()

    if action == "velocity":
        vx = require_finite_field(command, "vx")
        wz = require_finite_field(command, "wz")

        mark_command_received()

        print(
            "[CMD] action=velocity "
            "vx={:+.3f} wz={:+.3f}".format(
                vx,
                wz
            )
        )

        run_velocity(
            vx,
            wz
        )

        return

    if action == "stop":
        mark_command_received()

        print(
            "[CMD] action=stop"
        )

        stop_motors()

        return


    if action == "estop":
        print("[CMD] action=estop")
        engage_estop("remote_estop")
        return

    if action == "clear_estop":
        print("[CMD] action=clear_estop")
        clear_estop()
        return

    if action == "status":
        return get_safety_status()

    if action == "arm_pose":

        pose = str(
            command.get(
                "pose",
                ""
            )
        ).strip().lower()

        mark_command_received()

        print(
            "[CMD] action=arm_pose "
            "pose={}".format(
                pose
            )
        )

        # 手臂動之前底盤一定停止
        stop_motors()

        move_arm_pose(
            pose
        )

        # 移完再保證一次 wheels = 0
        stop_motors()

        return

    if action == "wheel":
        m1 = int(
            command.get(
                "m1",
                0
            )
        )

        m2 = int(
            command.get(
                "m2",
                0
            )
        )

        m3 = int(
            command.get(
                "m3",
                0
            )
        )

        m4 = int(
            command.get(
                "m4",
                0
            )
        )

        # RL 測試版 wheel 也限制在 ±30
        m1 = int(clamp(
            m1,
            -RL_MOTOR_LIMIT,
            RL_MOTOR_LIMIT
        ))

        m2 = int(clamp(
            m2,
            -RL_MOTOR_LIMIT,
            RL_MOTOR_LIMIT
        ))

        m3 = int(clamp(
            m3,
            -RL_MOTOR_LIMIT,
            RL_MOTOR_LIMIT
        ))

        m4 = int(clamp(
            m4,
            -RL_MOTOR_LIMIT,
            RL_MOTOR_LIMIT
        ))

        mark_command_received()

        print(
            "[CMD] action=wheel "
            "m1={} m2={} m3={} m4={}".format(
                m1,
                m2,
                m3,
                m4
            )
        )

        set_motors(
            m1,
            m2,
            m3,
            m4
        )

        return

    raise ValueError(
        "不支援的 action：{}".format(
            action
        )
    )


# ============================================================
# 14. TCP client
# ============================================================

def handle_client(
    connection,
    address
):
    print(
        "[INFO] client connected: {}".format(
            address
        )
    )

    connection.settimeout(
        0.20
    )

    receive_buffer = ""

    try:
        while (
            not rospy.is_shutdown()
            and not shutting_down
        ):
            try:
                raw_data = (
                    connection.recv(
                        4096
                    )
                )

            except socket.timeout:
                continue

            if not raw_data:
                break

            receive_buffer += (
                raw_data.decode(
                    "utf-8",
                    errors="replace"
                )
            )

            while "\n" in receive_buffer:
                line, receive_buffer = (
                    receive_buffer.split(
                        "\n",
                        1
                    )
                )

                line = line.strip()

                if not line:
                    continue

                try:
                    command = json.loads(
                        line
                    )

                    process_command(
                        command
                    )

                except Exception as exc:
                    print(
                        "[WARN] command error: {}".format(
                            exc
                        )
                    )

                    # 錯誤命令採安全停止
                    stop_motors()

    finally:
        stop_motors()

        try:
            connection.close()

        except Exception:
            pass

        print(
            "[INFO] client disconnected: {}".format(
                address
            )
        )


# ============================================================
# 15. TCP server
# ============================================================

def tcp_server_loop():
    server_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )

    server_socket.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1
    )

    try:
        server_socket.bind(
            (
                HOST,
                PORT
            )
        )

    except OSError as exc:
        print()
        print(
            "[ERROR] 無法使用 TCP port {}：{}".format(
                PORT,
                exc
            )
        )

        print(
            "[ERROR] 可能 ai_motor_server_B.py "
            "仍在占用 7000。"
        )

        stop_motors()

        raise

    server_socket.listen(
        1
    )

    server_socket.settimeout(
        0.50
    )

    print(
        "[INFO] P0 motor server listening on "
        "{}:{}".format(
            HOST,
            PORT
        )
    )

    try:
        while (
            not rospy.is_shutdown()
            and not shutting_down
        ):
            try:
                connection, address = (
                    server_socket.accept()
                )

            except socket.timeout:
                continue

            try:
                connection.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1
                )

            except Exception:
                pass

            # 一次只允許一個控制 client
            handle_client(
                connection,
                address
            )

    finally:
        try:
            server_socket.close()

        except Exception:
            pass

        stop_motors()


# ============================================================
# 16. 關閉處理
# ============================================================

def shutdown_handler():
    global shutting_down

    if shutting_down:
        return

    shutting_down = True

    print()
    print(
        "[INFO] P0 motor server shutting down..."
    )

    try:
        stop_motors()

    except Exception:
        pass

    if bot is not None:
        try:
            bot.set_auto_report_state(
                enable=False,
                forever=False
            )

        except Exception:
            pass


# ============================================================
# 17. Main
# ============================================================

def main():
    rospy.init_node(
        "ai_motor_server_P0",
        anonymous=False
    )

    rospy.on_shutdown(
        shutdown_handler
    )

    initialize_rosmaster()

    with safety_lock:
        motion_state_monitor.reset()
    stop_motors()

    odom_publisher = rospy.Publisher(
        "/odom_setmotor",
        Odometry,
        queue_size=20
    )

    tf_broadcaster = rospy.Publisher(
        "/tf",
        TFMessage,
        queue_size=20
    )

    odom_thread = threading.Thread(
        target=odom_loop,
        args=(
            odom_publisher,
            tf_broadcaster
        )
    )

    odom_thread.daemon = True
    odom_thread.start()

    watchdog_thread = threading.Thread(
        target=watchdog_loop
    )

    watchdog_thread.daemon = True
    watchdog_thread.start()

    print()
    print(
        "========================================"
    )

    print(
        "ai_motor_server_P0.py"
    )

    print(
        "TCP input : vx / wz"
    )

    print(
        "Odom input: get_motion_data() feedback"
    )

    print(
        "Topic     : /odom_setmotor"
    )

    print(
        "TF        : odom -> base_footprint"
    )

    print(
        "Port      : 7000"
    )

    print(
        "RL limit  : motor ±{}".format(
            RL_MOTOR_LIMIT
        )
    )

    print(
        "========================================"
    )

    try:
        tcp_server_loop()

    except KeyboardInterrupt:
        pass

    finally:
        shutdown_handler()


if __name__ == "__main__":
    main()
