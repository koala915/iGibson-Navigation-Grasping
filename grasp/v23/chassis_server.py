#!/usr/bin/env python3
"""TCP 7000 chassis control inside the resident grasp service (G2).

Why this lives in the grasp service: /dev/myserial can have one owner. Until
now that meant stopping the grasp service (and paying its 11.6 s restart) every
time the robot had to drive. With this module the service that already holds
the port also drives the wheels, so approach -> grasp -> drive needs no restart.
See docs/planning/G2_ARCHITECTURE_2026-09-28.md.

Protocol -- unchanged from integration/sugarbox_rl_motor_server.py, so the
approach controller does not change a line. One JSON object per line:

    {"action": "velocity", "vx": <m/s>, "wz": <rad/s>}
    {"action": "stop"}

A missing action means stop. Anything else stops the wheels and is logged. No
command for 0.5 s stops the wheels. The older {"action": "forward", "speed": N}
format of the detection/ tools is deliberately not accepted: its only definition
is ai_motor_server_P0.py, which lives on the robot and not in this repo.

Safety rules, enforced here rather than trusted to clients:

  R1  An arm command is refused while the wheels turn, and for 0.5 s after they
      stop. "Stopped" means the last non-zero motor write is 0.5 s old; a client
      streaming stop or zero velocity does not count as moving.
  R2  While the arm moves, velocity commands are dropped (counted, not written).
  R3  Every write to the board goes through one lock. Rosmaster_Lib sends each
      command as a single ser.write(), so locking that call keeps arm and wheel
      packets from interleaving when they come from different threads.
  R8  The board must be talking. Writes and reads use separate USB transfers, and
      on 2026-09-28 a hub glitch killed only the read side: the wheels still
      obeyed while every servo read failed and the wheel feedback froze at its
      last value -- which odometry cannot tell from standing still. Every byte
      the driver reads is timestamped (install_rx_monitor); after RX_STALE_S of
      silence the wheels stop and velocity is refused.
  R7  The wheels follow the arm pose the service last verified: full speed in the
      travel pose, a creep (|vx| <= 0.10 m/s, |wz| <= 0.5 rad/s) at E1 for the
      last centimetres into the 3 cm grasp window, and nothing anywhere else --
      the braking corridor was measured with the arm in the travel pose, and an
      arm frozen half-way through an aborted grasp is not something to drive with.

The wheel mapping is imported from sugarbox_rl_motor_server, never copied: two
copies of motor constants is how the old RL server ended up with a mirrored yaw.
"""
from __future__ import annotations

import importlib
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

SETTLE_S = 0.5
ZERO = (0, 0, 0, 0)
MAX_LINE_BYTES = 4096
CREEP_VX = 0.10
CREEP_WZ = 0.5
DRIVE_POSES = ("travel", "e1")
RX_STALE_S = 0.5


def load_motor_mapping(repo_root: Path):
    """(velocity_to_motor_values, ABSOLUTE_MAX_MOTOR, WATCHDOG_TIMEOUT_S)."""
    integration = str(Path(repo_root) / "integration")
    if integration not in sys.path:
        # Appended, not inserted: a grasp/ module must never be shadowed by an
        # integration/ file of the same name.
        sys.path.append(integration)
    mod = importlib.import_module("sugarbox_rl_motor_server")
    return mod.velocity_to_motor_values, mod.ABSOLUTE_MAX_MOTOR, mod.WATCHDOG_TIMEOUT_S


def install_write_lock(device, lock) -> None:
    """R3: serialize every packet this process writes to the board."""
    ser = device.ser
    if getattr(ser, "_g2_write_lock", None) is not None:
        return
    raw_write = ser.write

    def locked_write(data):
        with lock:
            return raw_write(data)

    ser.write = locked_write
    ser._g2_write_lock = lock


class RxMonitor:
    """When the board last sent this process a byte (R8).

    Rosmaster_Lib's receive thread calls ser.read() once per byte, looking the
    method up each time, so wrapping it catches every byte without touching the
    driver. The board streams its auto-report continuously, so silence means the
    read side is dead, not that the robot is idle.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self.last = clock()
        self.bytes = 0

    def age(self) -> float:
        return self._clock() - self.last

    def saw(self, n: int) -> None:
        self.last = self._clock()
        self.bytes += n


def install_rx_monitor(device, clock=time.monotonic) -> RxMonitor:
    ser = device.ser
    mon = getattr(ser, "_g2_rx", None)
    if mon is not None:
        return mon
    mon = RxMonitor(clock)
    raw_read = ser.read

    def monitored_read(*args, **kwargs):
        data = raw_read(*args, **kwargs)
        if data:
            mon.saw(len(data))
        return data

    ser.read = monitored_read
    ser._g2_rx = mon
    return mon


class Chassis:
    """Wheel state plus R1/R2. Every state change happens under one lock, so a
    velocity command can never slip in between the R1 check and the arm start."""

    def __init__(self, device, to_motor, *, max_motor: int = 60,
                 watchdog_s: float = 0.5, clock=time.monotonic, log=print,
                 rx_age=None):
        self.device = device
        self._rx_age = rx_age               # R8; None only in tests that do not need it
        self.rx_blocked = 0
        self._to_motor = to_motor
        self._max_motor = int(max_motor)
        self.watchdog_s = float(watchdog_s)
        self._clock = clock
        self._log = log
        self._lock = threading.Lock()
        self._motors = None                 # unknown until the first write
        self._stopped_since = -math.inf     # when the wheels last went to zero
        self._last_cmd = -math.inf
        self._arm_busy = False
        self._maintenance = False
        # Latched by shutdown(). Without it a command already read off the socket
        # could land after the final stop: seen on the robot 2026-09-28, the
        # wheels went back to 20 for 2 ms after systemctl restart zeroed them.
        self._closed = False
        self._arm_pose = "unknown"          # set by the service after every arm move
        self.pose_blocked = 0               # velocity commands refused by R7, total
        self.dropped = 0                    # velocity commands eaten by R2, total
        self._dropped_this_arm = 0
        self.refused = 0                    # arm commands refused by R1, total
        self.client = None

    # ── writes (caller holds self._lock) ───────────────────────────────────
    def _write(self, values) -> None:
        values = tuple(int(max(-self._max_motor, min(self._max_motor, v)))
                       for v in values)
        if values == self._motors:
            return
        self.device.set_motor(*values)
        transport = getattr(self.device, '__dict__', {}).get('_v23_transport')
        if transport is not None:
            transport.require_write_ok()
        was_moving = self._motors not in (None, ZERO)
        self._motors = values
        if values == ZERO:
            if was_moving:
                self._stopped_since = self._clock()
        else:
            self._stopped_since = None
        self._log("[chassis] m1=%d m2=%d m3=%d m4=%d" % values)

    def _moving_or_settling(self, now: float) -> bool:
        if self._motors not in (None, ZERO):
            return True
        return self._stopped_since is not None and now - self._stopped_since < SETTLE_S

    # ── client side ────────────────────────────────────────────────────────
    def command(self, msg) -> str:
        """Apply one protocol message. Returns what happened; raises on a bad
        message after stopping the wheels, which is what the caller logs."""
        with self._lock:
            if self._closed:
                return "closed"
            self._last_cmd = self._clock()
            if not isinstance(msg, dict):
                self._write(ZERO)
                raise ValueError("command must be a JSON object")
            action = str(msg.get("action", "stop")).strip().lower()
            if action == "stop":
                self._write(ZERO)
                return "stop"
            if action != "velocity":
                self._write(ZERO)
                raise ValueError("unsupported action: %s" % action)
            try:
                vx = float(msg.get("vx", 0.0))
                wz = float(msg.get("wz", 0.0))
            except (TypeError, ValueError):
                vx = wz = math.nan
            if not (math.isfinite(vx) and math.isfinite(wz)):
                self._write(ZERO)
                raise ValueError("vx and wz must be finite numbers")
            if self._arm_busy:
                self.dropped += 1
                self._dropped_this_arm += 1
                return "dropped"
            if self._maintenance:
                self._write(ZERO)
                return "maintenance"
            if self._rx_stale():
                self.rx_blocked += 1
                self._write(ZERO)
                if self.rx_blocked in (1, 10) or self.rx_blocked % 100 == 0:
                    self._log("[chassis] velocity refused (%dx): no data from the board "
                              "for %.1fs" % (self.rx_blocked, self._rx_age()))
                return "board_silent"
            if self._arm_pose not in DRIVE_POSES:
                self.pose_blocked += 1
                self._write(ZERO)
                if self.pose_blocked in (1, 10) or self.pose_blocked % 100 == 0:
                    self._log("[chassis] velocity refused (%dx): arm pose is %s, "
                              "stow or home it first" % (self.pose_blocked, self._arm_pose))
                return "arm_pose"
            if self._arm_pose == "e1":
                vx = max(-CREEP_VX, min(CREEP_VX, vx))
                wz = max(-CREEP_WZ, min(CREEP_WZ, wz))
            self._write(self._to_motor(vx, wz))
            return "ok"

    def stop(self) -> None:
        with self._lock:
            self._write(ZERO)

    def shutdown(self) -> None:
        """Zero the wheels and refuse every later command. Not reversible."""
        with self._lock:
            self._closed = True
            self._write(ZERO)

    def _rx_stale(self) -> bool:
        transport = getattr(self.device, '__dict__', {}).get('_v23_transport')
        return (transport is not None and not transport.status()['healthy']) or \
            (self._rx_age is not None and self._rx_age() > RX_STALE_S)

    def watchdog_tick(self) -> bool:
        """Stop the wheels if the client or the board went quiet. True when it did."""
        with self._lock:
            if self._motors in (None, ZERO):
                return False
            if self._rx_stale():
                why = "no data from the board for %.1fs" % self._rx_age()
            elif self._clock() - self._last_cmd > self.watchdog_s:
                why = "no command for %.1fs" % self.watchdog_s
            else:
                return False
            self._write(ZERO)
        self._log("[chassis] watchdog: %s, wheels stopped" % why)
        return True

    # ── arm side ───────────────────────────────────────────────────────────
    def begin_arm(self):
        """R1 + R2. Returns None when the arm may move, else the refusal reason."""
        with self._lock:
            if self._maintenance:
                return "maintenance"
            if self._rx_stale():
                return "board_silent"
            if self._moving_or_settling(self._clock()):
                self.refused += 1
                return "chassis_moving"
            self._write(ZERO)       # belt and braces: the board is told zero
            self._arm_busy = True
            self._dropped_this_arm = 0
            return None

    def maintenance(self, enable):
        """Atomic idle-only lock: ROS repairs cannot race a new wheel/arm command.

        Failed/abandoned repairs leave the lock set. Only an explicit verified
        unlock or service restart can clear it; never a timeout into motion.
        """
        with self._lock:
            if self._closed or self._arm_busy or self.client is not None or \
                    self._moving_or_settling(self._clock()) or self._rx_stale() or \
                    self._arm_pose != 'travel':
                return False
            self._write(ZERO)
            self._maintenance = bool(enable)
            return True

    def set_arm_pose(self, pose: str) -> None:
        """R7 input: 'travel', 'e1' or anything else (which stops the wheels)."""
        with self._lock:
            self._arm_pose = str(pose)
            if self._arm_pose not in DRIVE_POSES:
                self._write(ZERO)

    def restart_lock(self):
        """Freeze an idle faulty owner atomically, without writing on a broken bus."""
        with self._lock:
            if self._closed or self._arm_busy or self.client is not None \
                    or self._moving_or_settling(self._clock()) or self._motors != ZERO \
                    or self._arm_pose not in ('travel','restart_pending') or not self._rx_stale():
                return False
            self._maintenance = True
            self._arm_pose = 'restart_pending'
            return True

    def still_since(self):
        """Clock time the wheels have been stopped since; None while they turn."""
        with self._lock:
            if self._motors not in (None, ZERO):
                return None
            return self._stopped_since if math.isfinite(self._stopped_since) else 0.0

    def end_arm(self) -> None:
        with self._lock:
            self._arm_busy = False
            dropped = self._dropped_this_arm
        if dropped:
            self._log("[chassis] dropped %d velocity command(s) while the arm moved"
                      % dropped)

    def status(self) -> dict:
        with self._lock:
            now = self._clock()
            since = self._stopped_since
            # None while moving, and before the wheels have ever moved.
            stopped_for = (round(now - since, 2)
                           if since is not None and math.isfinite(since) else None)
            return {"motors": list(self._motors) if self._motors else None,
                    "maintenance": self._maintenance,
                    "moving": self._moving_or_settling(now),
                    "stopped_for_s": stopped_for,
                    "arm_busy": self._arm_busy,
                    "arm_pose": self._arm_pose,
                    "pose_blocked_total": self.pose_blocked,
                    "board_rx_age_s": (round(self._rx_age(), 2)
                                       if self._rx_age is not None else None),
                    "rx_blocked_total": self.rx_blocked,
                    "dropped_total": self.dropped,
                    "refused_total": self.refused,
                    "client": self.client}


class ChassisServer:
    """TCP front end: one client at a time, like the motor server it replaces."""

    def __init__(self, chassis: Chassis, host: str = "0.0.0.0", port: int = 7000,
                 log=print):
        self.chassis = chassis
        self.host = host
        self.port = int(port)
        self._log = log
        self._closing = threading.Event()
        self._sock = None
        self._threads = []

    def start(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        srv.settimeout(0.5)
        self._sock = srv
        self.port = srv.getsockname()[1]
        # The board may still hold a speed from whoever owned the port before us.
        self.chassis.stop()
        for target in (self._accept_loop, self._watchdog_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        self._log("[chassis] TCP %d ready (velocity protocol, %.1fs watchdog)"
                  % (self.port, self.chassis.watchdog_s))

    def close(self) -> None:
        """R6: wheels to zero first, then stop listening."""
        self._closing.set()
        try:
            self.chassis.shutdown()
        finally:
            if self._sock is not None:
                self._sock.close()
        for t in self._threads:
            t.join(timeout=1.0)

    def _watchdog_loop(self) -> None:
        while not self._closing.is_set():
            try:
                self.chassis.watchdog_tick()
            except Exception as exc:          # a write error must not kill the watchdog
                self._log("[chassis] watchdog write failed: %s" % exc)
            time.sleep(0.05)

    def _accept_loop(self) -> None:
        while not self._closing.is_set():
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return                         # socket closed by close()
            try:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._handle(conn, addr)
            finally:
                conn.close()

    def _handle(self, conn, addr) -> None:
        who = "%s:%s" % addr[:2]
        self.chassis.client = who
        self._log("[chassis] client connected: %s" % who)
        conn.settimeout(0.2)
        buf = b""
        errors = 0
        try:
            while not self._closing.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError as exc:
                    self._log("[chassis] receive failed: %s" % exc)
                    break
                if not chunk:
                    break
                buf += chunk
                if b"\n" not in buf and len(buf) > MAX_LINE_BYTES:
                    self._log("[chassis] %s sent %d bytes without a newline, "
                              "dropping it" % (who, len(buf)))
                    break
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        self.chassis.command(json.loads(line.decode("utf-8", "replace")))
                    except Exception as exc:
                        errors += 1
                        # Logged sparingly: a client stuck on a bad format at
                        # 10 Hz would otherwise bury everything else.
                        if errors <= 3 or errors % 50 == 0:
                            self._log("[chassis] bad command #%d from %s: %s"
                                      % (errors, who, exc))
        finally:
            try:
                self.chassis.stop()
            except Exception as exc:
                self._log("[chassis] stop on disconnect failed: %s" % exc)
            self.chassis.client = None
            self._log("[chassis] client disconnected: %s" % who)
