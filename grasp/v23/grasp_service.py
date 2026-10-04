#!/usr/bin/env python3
"""Resident grasp service: pay the 23-second startup once, then grasp on command.

Measured 2026-09-20 on the robot: a one-command grasp takes 42 s wall, of which
34 s is startup the robot spends motionless -- torch and PyBullet imports, the
URDF load (11.5 s by itself), the PPO weights, then YOLO and the camera. Only
about 8 s is the arm moving. Keeping the process alive removes all 34 s.

**It deliberately does not reimplement any of the launcher's setup.** The
controller's configuration lives inside its own ``main()``, behind a hundred
lines of validation -- the sha256 check, the contract check, the homography
gate, the candidate unlock, the E1 pose stamp. Copying that here would mean
maintaining a second copy of the safety gates, and a second copy is how they
drift apart. So this service:

  1. asks ``jetson_one_command_grasp`` to build the exact controller argv it
     would have used (``build_ctrl_cmd``), then
  2. replaces ``GraspController.run`` with a serve loop, and
  3. calls the controller's own ``main()``.

Every gate therefore runs exactly as it does today, at startup, and the first
time ``main()`` reaches ``run()`` we take over the process instead of grasping
once and exiting. Each command then calls the *original* ``run()``, which the
controller documents as safe to call repeatedly: "every piece of episode state
is reset at the top of this method".

The vision side is not touched at all: run ``vision_grasp_bridge.py`` without
``--once`` so a fresh detection is always waiting, and the controller latches it
at the home pose through the same socket path it already uses.

  start:   python3 grasp_service.py [any jetson_one_command_grasp flags]
  drive:   python3 graspctl.py grasp

G2: with GRASP_SERVICE_CHASSIS_PORT=7000 in the environment the service also
drives the wheels over the TCP velocity protocol (chassis_server.py), so the
robot can approach, grasp and drive away without handing /dev/myserial to
another process. Unset, the service is exactly the arm-only one validated on
2026-09-25.

G2 step 3: with GRASP_SERVICE_ODOM_ROSBRIDGE=127.0.0.1:9090 it also publishes
the wheel odometry (/odom_setmotor + odom->base_footprint) over rosbridge
(odom_bridge.py). rosbridge may come up later than the service; publishing
starts when it does.
"""
from __future__ import annotations

import json
import math
import os
import signal
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_SOCKET = "/tmp/grasp_service.sock"
ARM_COMMANDS = ("grasp", "home", "release", "stow")

# The travel pose the robot drives in (docs/calibration/arm_pose.md) and the one
# path between it and E1 that has been checked: S2 swings to WAYPOINT_S2 with
# S3/S4 still at their E1 angles, then everything goes the rest of the way.
# Going direct while holding a box folds the arm before it rises and sweeps the
# box to 10.6 mm of the TG30 (validate_nav_to_grasp_transition.py).
TRAVEL_DEG = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)
WAYPOINT_S2 = 115.0
POSE_TOL_DEG = 4.0

# How old a detection can be relative to the frame it came from. The bridge reads
# the camera flat out (~4 fps with YOLO), so the V4L2 queue hands it frames up to
# about a second old, plus ~0.25 s of inference; it sends the newest result once a
# second. A detection received FRAME_LAG_S after the arm and the wheels stopped
# was therefore shot after they stopped.
FRAME_LAG_S = 1.5
# The grasp starts no sooner than this after the last motion, so every detection
# still fresh (received in the last 1 s) at latch time is newer than FRAME_LAG_S.
GRASP_SETTLE_S = 2.5


def rss_mb() -> float:
    """Resident size of this process, for watching the loop leak (or not)."""
    try:
        with open("/proc/self/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return -1.0


class GraspService:
    """Accepts one client at a time and runs one episode per `grasp` command.

    Single-threaded on purpose: there is one arm, and an episode that overlaps
    another would fight it for the servo bus.
    """

    def __init__(self, controller, run_episode, max_steps: int, sock_path: str,
                 chassis_server=None, odom_bridge=None):
        self.controller = controller
        self._run_episode = run_episode
        self.max_steps = max_steps
        self.sock_path = sock_path
        self.chassis_server = chassis_server
        self.odom_bridge = odom_bridge
        self.chassis = chassis_server.chassis if chassis_server is not None else None
        self.stop_requested = False
        self.episodes = 0
        self.last = None
        self.started = time.time()
        # Monotonic, like the detection receiver's own stamps.
        self._arm_moved_at = time.monotonic()

    # ── detection ───────────────────────────────────────────────────────
    def detection(self):
        """(pos, height, fresh) from the controller's own receiver, or None.

        The receiver only exists on the --socket path, which is the one the
        launcher builds, but a caller could configure a fixed target instead.
        """
        recv = getattr(self.controller, "detection", None)
        if recv is None:
            return None
        pos, height, fresh, _stamp = recv.snapshot()
        return {"pos": [round(float(v), 4) for v in pos],
                "height": height, "fresh": bool(fresh)}

    def detection_received_at(self) -> float:
        """Monotonic receipt time of the latest detection; 0.0 when none/no receiver."""
        recv = getattr(self.controller, "detection", None)
        return float(getattr(recv, "_last_update_ts", 0.0) or 0.0)

    def wait_for_detection(self, timeout_s: float, newer_than: float = 0.0):
        """Wait briefly for a fresh E1 detection.

        Returns the detection, or None when one never arrived. A controller
        configured with a fixed --obj-x/y/z has no receiver at all; that is not
        a missing detection, so `detection()` returning None means "not my
        business" and the caller proceeds.

        Checked BEFORE the episode starts, because `run()` would otherwise walk
        the arm to the home pose and sit there for the whole --latch-wait (120 s
        by default) waiting for a detection that nobody is sending. Refusing up
        front costs nothing and leaves the arm where it is.
        """
        deadline = time.time() + timeout_s
        while True:
            det = self.detection()
            if det is None or (det["fresh"] and self.detection_received_at() >= newer_than):
                return det
            if time.time() >= deadline:
                return None
            time.sleep(0.2)

    # ── commands ────────────────────────────────────────────────────────
    def cmd_status(self) -> dict:
        jaw = None
        try:
            rd = self.controller.servo.read_degrees()
            jaw = rd.degrees if rd.valid else None
        except Exception as exc:                      # diagnostics must not kill the service
            jaw = "read failed: %s" % exc
        return {"ok": True, "episodes": self.episodes, "last": self.last,
                "servo_deg": jaw, "detection": self.detection(),
                "chassis": self.chassis.status() if self.chassis else None,
                "odom": self.odom_bridge.status() if self.odom_bridge else None,
                "rss_mb": round(rss_mb(), 1),
                "uptime_s": round(time.time() - self.started, 1)}

    def run_arm_command(self, fn) -> dict:
        """R1/R2 around one arm command; a plain call when the chassis is off.

        The wheels are claimed before anything else, the detection wait
        included: the grasp latches what the camera sees, so the robot has to
        stand still from that moment on, not only once the arm starts moving.
        """
        if self.chassis is not None:
            why = self.chassis.begin_arm()
            if why is not None:
                return {"ok": False, "reason": why,
                        "hint": "the wheels are turning or stopped less than 0.5 s ago; "
                                "the arm was not moved",
                        "chassis": self.chassis.status()}
        try:
            # The 8 deg rate limit counts from the last COMMAND, not from the arm.
            # After a restart that command is the default home (E1) wherever the
            # arm really is, and on 2026-09-28 a home from the travel pose went
            # out as one unlimited jump. So start every command from the encoders,
            # and do not move an arm whose position cannot be read.
            try:
                deg = self._arm_deg()
            except Exception:
                deg = None
            if deg is None:
                return {"ok": False, "reason": "servo_read_failed",
                        "hint": "the arm was not moved: without the encoders the "
                                "rate limit would count from a guess"}
            self._sync_rate_limit(deg)
            return fn()
        finally:
            self._arm_moved_at = time.monotonic()
            if self.chassis is not None:
                self.chassis.set_arm_pose(self.arm_pose())
                self.chassis.end_arm()

    def _sync_rate_limit(self, deg) -> None:
        """Point the servo rate limiter (S1-S5) at where the arm actually is.

        The jaw is left alone: while holding, its last command is the squeeze
        past contact, and resetting it to the encoder would ease the grip.
        """
        servo = self.controller.servo
        last = list(getattr(servo, "_last_deg", deg))
        servo._last_deg = [float(v) for v in deg[:5]] + [float(last[5])]

    def arm_pose(self) -> str:
        """'travel', 'e1' or 'other' from the encoders ('unreadable' if they fail)."""
        try:
            deg = self._arm_deg()
        except Exception:
            deg = None
        if deg is None:
            return "unreadable"
        if self._near(deg, TRAVEL_DEG):
            return "travel"
        if self._near(deg, self.controller.cfg.home_deg):
            return "e1"
        return "other"

    def still_since(self) -> float:
        """Monotonic time since which both the arm and the wheels have been still."""
        since = self._arm_moved_at
        if self.chassis is not None:
            wheels = self.chassis.still_since()
            since = max(since, wheels if wheels is not None else time.monotonic())
        return since

    # ── travel pose ─────────────────────────────────────────────────────
    def _arm_deg(self):
        rd = self.controller.servo.read_degrees()
        if not rd.valid:
            return None
        try:
            deg = list(rd.degrees)
            if len(deg) != 6 or not all(math.isfinite(v) for v in deg):
                return None
        except (TypeError, ValueError, OverflowError):
            return None
        return deg

    def _sync_joint_state(self, deg) -> None:
        """Give FloorGuard the measured pose before its first projection.

        This changes the controller's physical state, not the jaw's hold command
        or the servo command rate limiter.
        """
        ctl = self.controller
        ctl._current_arm_rads = ctl.mapper.hw_deg_to_sim_arm(deg[:5])
        ctl._current_grip_rad = ctl.mapper.hw_deg_to_sim_grip(deg[5])

    @staticmethod
    def _near(deg, pose) -> bool:
        return deg is not None and all(abs(a - b) <= POSE_TOL_DEG
                                       for a, b in zip(deg[:5], pose[:5]))

    def _waypoint(self):
        e1 = self.controller.cfg.home_deg
        return (e1[0], WAYPOINT_S2, e1[2], e1[3], e1[4])

    def _move_legs(self, legs, label: str, deg) -> dict:
        """Guarded moves through `legs` (S1-S5 each), keeping whatever the jaw holds.

        A held object keeps the controller's hold command, not the encoder angle:
        re-commanding the jaw to where it reads zeroes the squeeze and drops it.
        """
        ctl = self.controller
        holding = ctl._grip_hold_rad is not None
        grip = (ctl._grip_hold_rad if holding
                else ctl.mapper.hw_deg_to_sim_grip(ctl.cfg.gripper_hw_open))
        # Start from the encoders, as run() does, not from the last episode's idea.
        self._sync_joint_state(deg)
        t0 = time.time()
        for i, leg in enumerate(legs):
            try:
                res = ctl.move_guarded_and_verified(
                    ctl.mapper.hw_deg_to_sim_arm(list(leg[:5])), grip,
                    label="%s-%d" % (label, i), run_time_ms=400, settle_s=0.35,
                    tol_deg=3.0, grip_is_hold=holding)
            except Exception as exc:
                return {"ok": False, "reason": "exception: %s" % exc, "leg": i,
                        "elapsed_s": round(time.time() - t0, 2)}
            if not res.get("reached"):
                return {"ok": False, "reason": res.get("reason"), "leg": i,
                        "holding": holding, "elapsed_s": round(time.time() - t0, 2)}
        return {"ok": True, "holding": holding, "elapsed_s": round(time.time() - t0, 2)}

    def _to_e1_from_travel(self, deg):
        """None when the arm is at E1 (or got there); otherwise a refusal reply."""
        if self._near(deg, self.controller.cfg.home_deg):
            return None
        if not self._near(deg, TRAVEL_DEG):
            return {"ok": False, "reason": "arm_not_at_e1_or_travel", "servo_deg": deg,
                    "hint": "only E1 <-> travel is a checked path; run home first"}
        res = self._move_legs([self._waypoint(), self.controller.cfg.home_deg],
                              "unstow", deg)
        return None if res["ok"] else res

    def cmd_stow(self) -> dict:
        """E1 -> travel pose through the checked waypoint. A held object stays held."""
        deg = self._arm_deg()
        if deg is None:
            return {"ok": False, "reason": "servo_read_failed"}
        if self._near(deg, TRAVEL_DEG):
            return {"ok": True, "already": True}
        if not self._near(deg, self.controller.cfg.home_deg):
            return {"ok": False, "reason": "arm_not_at_e1", "servo_deg": deg,
                    "hint": "only E1 -> travel is a checked path; run home first"}
        return self._move_legs([self._waypoint(), TRAVEL_DEG], "stow", deg)

    def cmd_home(self) -> dict:
        """Park at the grasp home pose with the jaw open, releasing anything held.

        Without this the only way to put the object back on the table is to stop
        the service, run move_arm.py and start it again -- a minute of reloading
        PyBullet to open a gripper. Uses the same guarded move run() uses, so the
        floor guard still applies. From the travel pose it first goes back to E1
        through the waypoint with the jaw still closed, so a held object is let go
        at E1 rather than dropped on the chassis.
        """
        ctl = self.controller
        cfg = ctl.cfg
        deg = self._arm_deg()
        if deg is None:
            return {"ok": False, "reason": "servo_read_failed"}
        # A service restart leaves the controller at the configured E1 pose in
        # software, even when the encoders report another pose. FloorGuard must
        # start from the physical arm and jaw before the first home command.
        self._sync_joint_state(deg)
        if self._near(deg, TRAVEL_DEG):
            refused = self._to_e1_from_travel(deg)
            if refused is not None:
                return refused
        arm = ctl.mapper.hw_deg_to_sim_arm(list(cfg.home_deg[:5]))
        grip = ctl.mapper.hw_deg_to_sim_grip(cfg.home_deg[5])
        t0 = time.time()
        try:
            res = ctl.move_guarded_and_verified(
                arm, grip, label="service-home", run_time_ms=400, settle_s=0.35,
                tol_deg=3.0)
        except Exception as exc:
            # A hardware fault must not take the service down with it; the arm
            # holds position while powered, so reporting and staying up is safer
            # than dying and leaving no way to ask what happened.
            return {"ok": False, "reason": "exception: %s" % exc,
                    "elapsed_s": round(time.time() - t0, 2)}
        ctl._grip_hold_rad = None     # nothing is held any more
        return {"ok": bool(res.get("reached")), "reason": res.get("reason"),
                "iters": res.get("iters"), "elapsed_s": round(time.time() - t0, 2)}

    def cmd_release(self) -> dict:
        """Stage 3 on its own: reach forward, open over the bin, come home.

        The controller refuses to open while the pads sit below
        bin_rim_height + release_clearance, and refuses outright when the jaw
        does not look like it is holding anything. Note what the outcome does
        NOT mean: the jaw-stall proxy reads "holding" right up until the jaw is
        commanded open, so "released" says the motion ran, not that the object
        landed in the bin.
        """
        deg = self._arm_deg()
        if deg is None:
            return {"ok": False, "outcome": "servo_read_failed"}
        # The release reach was checked on the robot from E1 (2026-09-25); from the
        # travel pose, carry the object back to E1 first rather than reach from there.
        refused = self._to_e1_from_travel(deg)
        if refused is not None:
            refused["outcome"] = refused.get("reason")
            return refused
        t0 = time.time()
        try:
            outcome = self.controller.run_release_only()
        except Exception as exc:
            return {"ok": False, "outcome": "exception: %s" % exc,
                    "elapsed_s": round(time.time() - t0, 2)}
        return {"ok": outcome == "released", "outcome": outcome,
                "rim_cm": round(self.controller.cfg.bin_rim_height * 100, 1),
                "clearance_cm": round(self.controller.cfg.release_clearance * 100, 1),
                "elapsed_s": round(time.time() - t0, 2),
                "rss_mb": round(rss_mb(), 1)}

    def cmd_grasp(self, wait_s: float = 3.0) -> dict:
        # E1 first: the resident bridge streams a detection every second, so a
        # visible object is already in hand and the episode costs ~8.5 s with no
        # arm rotation at all. Only when E1 sees nothing is the three-pose scan
        # worth its rotations -- and that path stays a separate, manually
        # unlocked flow (graspscan.sh), because a LEFT/RIGHT-only target rests
        # entirely on the S1 rotation the scan config still calls unvalidated.
        #
        # Only from E1. The latch takes any detection received in the last second
        # and checks the camera pose against the encoders at latch time, not at
        # capture time, so a frame shot on the way up from the travel pose would
        # pass and put the grasp somewhere plausible and wrong.
        deg = self._arm_deg()
        if deg is None or not self._near(deg, self.controller.cfg.home_deg):
            return {"ok": False, "reason": "arm_not_at_e1", "servo_deg": deg,
                    "hint": "run home first; it comes up from the travel pose "
                            "through the checked waypoint", "episodes": self.episodes}
        # A frame shot before the arm or the wheels last stopped would latch fine
        # and put the grasp somewhere plausible and wrong, so wait for one shot after.
        since = self.still_since()
        pause = since + GRASP_SETTLE_S - time.monotonic()
        if pause > 0:
            time.sleep(pause)
        if (self.detection() is not None
                and self.wait_for_detection(wait_s, newer_than=since + FRAME_LAG_S) is None):
            return {"ok": False, "reason": "no_fresh_e1_detection",
                    "hint": "nothing visible from E1 in %.0fs — object outside "
                            "the E1 window, or the vision service is down. The "
                            "arm was not moved. Run graspscan.sh to search with "
                            "the three-pose scan." % wait_s,
                    "episodes": self.episodes}
        t0 = time.time()
        self.episodes += 1
        try:
            confirmed = bool(self._run_episode(self.controller,
                                               max_steps=self.max_steps))
            reason = getattr(self.controller, "_outcome", None)
        except Exception as exc:
            confirmed, reason = False, "exception: %s" % exc
        elapsed = time.time() - t0
        self.last = {"confirmed": confirmed, "outcome": reason,
                     "elapsed_s": round(elapsed, 2)}
        # Printed as well as returned: the RSS trend across episodes is the
        # evidence that a long-lived torch + PyBullet process is not leaking.
        print("[service] episode %d: confirmed=%s outcome=%s %.2fs  rss %.0f MB"
              % (self.episodes, confirmed, reason, elapsed, rss_mb()), flush=True)
        return {"ok": True, **self.last, "rss_mb": round(rss_mb(), 1)}

    # ── loop ────────────────────────────────────────────────────────────
    def serve(self) -> None:
        if os.path.exists(self.sock_path):
            os.unlink(self.sock_path)
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self.sock_path)
        os.chmod(self.sock_path, 0o600)
        srv.listen(1)
        try:
            if self.chassis_server is not None:
                self.chassis_server.start()
                # R7 starts closed ("unknown") until the encoders say where the arm is.
                self.chassis.set_arm_pose(self.arm_pose())
            if self.odom_bridge is not None:
                self.odom_bridge.start()
            print("\n[service] ready on %s — startup cost is now paid.\n"
                  "[service] commands: grasp | release | home | stow | status | quit"
                  "   chassis: %s   odom: %s   (rss %.0f MB)\n"
                  % (self.sock_path,
                     "TCP %d" % self.chassis_server.port if self.chassis_server
                     else "off",
                     "rosbridge %s:%d" % (self.odom_bridge.host, self.odom_bridge.port)
                     if self.odom_bridge else "off", rss_mb()), flush=True)
            while not self.stop_requested:
                conn, _ = srv.accept()
                # A client that connects and never sends a line would otherwise
                # block the only thread that can drive the arm.
                conn.settimeout(10.0)
                try:
                    try:
                        with conn.makefile("r") as stream:
                            line = stream.readline().strip()
                    except (socket.timeout, OSError):
                        continue    # a client that said nothing costs us nothing
                    if not line:
                        continue
                    if line == "quit":
                        conn.sendall(b'{"ok": true, "bye": true}\n')
                        print("[service] quit requested", flush=True)
                        return
                    if line == "status":
                        reply = self.cmd_status()
                    elif line in ARM_COMMANDS:
                        reply = self.run_arm_command(
                            {"home": self.cmd_home, "release": self.cmd_release,
                             "grasp": self.cmd_grasp, "stow": self.cmd_stow}[line])
                    else:
                        reply = {"ok": False, "error": "unknown command %r" % line}
                    conn.sendall((json.dumps(reply) + "\n").encode())
                finally:
                    conn.close()
        except KeyboardInterrupt:
            print("\n[service] interrupted — handing back to the controller's "
                  "own shutdown path", flush=True)
        finally:
            # Wheels first: once this process lets go of the port nothing else
            # will stop them.
            if self.chassis_server is not None:
                self.chassis_server.close()
            if self.odom_bridge is not None:
                self.odom_bridge.close()
            srv.close()
            if os.path.exists(self.sock_path):
                os.unlink(self.sock_path)


def install_sigterm_shutdown(service) -> None:
    """Make SIGTERM take the Ctrl+C path. Only installed when the wheels are ours.

    systemd stops a unit with SIGTERM, and Python's default for that is to exit
    on the spot, skipping every finally block. With the wheels turning that
    leaves them turning: the board keeps the last speed it was given. The flag
    covers an episode that swallows the KeyboardInterrupt itself -- the loop
    still ends after it instead of waiting for systemd's SIGKILL.
    """
    def on_sigterm(_signum, _frame):
        service.stop_requested = True
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)


def build_chassis_server(controller, port_text: str):
    """The TCP 7000 chassis front end, or None when it is not asked for.

    Fails loudly when it is asked for and cannot start (port taken, mapping
    missing): a service that quietly came up without wheels would leave the
    approach controller talking to nobody.
    """
    if not port_text:
        return None
    import threading
    import chassis_server as cs

    device = controller.servo.device
    if device is None:
        print("[service] chassis requested but there is no Rosmaster device "
              "(dry run) — chassis stays off", flush=True)
        return None
    to_motor, max_motor, watchdog_s = cs.load_motor_mapping(HERE.parents[1])
    cs.install_write_lock(device, threading.Lock())
    rx = cs.install_rx_monitor(device)
    chassis = cs.Chassis(device, to_motor, max_motor=max_motor, watchdog_s=watchdog_s,
                         log=lambda msg: print(msg, flush=True), rx_age=rx.age)
    return cs.ChassisServer(chassis, port=int(port_text),
                            log=lambda msg: print(msg, flush=True))


def build_odom_bridge(controller, target: str):
    """The rosbridge odometry publisher, or None when it is not asked for.

    Like the chassis, it fails loudly when asked for and impossible (no
    roslibpy, bad address). A rosbridge that is merely not running yet is not
    a failure: the bridge connects when it comes up.
    """
    if not target:
        return None
    import odom_bridge as ob

    device = controller.servo.device
    if device is None:
        print("[service] odometry requested but there is no Rosmaster device "
              "(dry run) — odometry stays off", flush=True)
        return None
    host, _, port = target.rpartition(":")
    if not host or not port.isdigit():
        raise ValueError("GRASP_SERVICE_ODOM_ROSBRIDGE must be host:port, got %r" % target)
    import roslibpy
    feedback_odom, ros_io = ob.load_odom_modules(HERE.parents[1])
    cfg = feedback_odom.FeedbackOdomConfig()
    # Printed so the journal says which calibration a run used: the file on the
    # robot is not always the one in the repo.
    print("[service] odometry calibration: %s" % ", ".join(
        "%s=%s" % (k, getattr(cfg, k)) for k in
        ("linear_scale", "lateral_scale", "angular_left_scale", "angular_right_scale")
        if hasattr(cfg, k)), flush=True)
    reader = feedback_odom.FeedbackOdomReader(device, cfg)
    import chassis_server as cs
    rx = cs.install_rx_monitor(device)          # the same monitor the chassis uses
    return ob.OdomBridge(reader, ros_io, roslibpy, host=host, port=int(port),
                         log=lambda msg: print(msg, flush=True),
                         feedback_age=rx.age, feedback_stale_s=cs.RX_STALE_S)


def main() -> int:
    sys.path.insert(0, str(HERE))
    import jetson_one_command_grasp as launcher   # noqa: E402
    import x3plus_real_grasp as ctrl              # noqa: E402

    # Every flag goes to the launcher's own parser untouched, so this service can
    # never accept a flag the launcher would have rejected. The one setting that
    # is ours alone travels by environment instead of competing for flag space.
    argv = sys.argv[1:]
    sock_path = os.environ.get("GRASP_SERVICE_SOCKET", DEFAULT_SOCKET)
    chassis_port = os.environ.get("GRASP_SERVICE_CHASSIS_PORT", "").strip()
    odom_target = os.environ.get("GRASP_SERVICE_ODOM_ROSBRIDGE", "").strip()

    launcher_args = launcher.parse_args_from(argv)
    ctrl_cmd = launcher.build_ctrl_cmd(launcher_args)
    # build_ctrl_cmd returns [python, script, *flags]; main() parses the flags.
    ctrl_argv = ctrl_cmd[2:]
    print("[service] controller flags (from the launcher, unmodified):")
    print("[service]   " + " ".join(ctrl_argv), flush=True)

    original_run = ctrl.GraspController.run
    state = {}

    def serving_run(self, max_steps: int = 300):
        """Stands in for the first (and only) run() call main() makes.

        Returning True keeps main() on its success path, so when the loop ends
        the controller closes the serial port and PyBullet exactly as it would
        after a normal run.
        """
        if state.get("serving"):
            # Defensive: a nested call would mean main() looped, which it does
            # not. Fall through to a real episode rather than recursing.
            return original_run(self, max_steps=max_steps)
        state["serving"] = True
        chassis_server = build_chassis_server(self, chassis_port)
        odom_bridge = build_odom_bridge(self, odom_target)
        service = GraspService(self, original_run, max_steps, sock_path,
                               chassis_server=chassis_server, odom_bridge=odom_bridge)
        if chassis_server is not None:
            install_sigterm_shutdown(service)
        service.serve()
        return True

    ctrl.GraspController.run = serving_run
    sys.argv = [str(HERE / "x3plus_real_grasp.py")] + ctrl_argv
    return ctrl.main()


if __name__ == "__main__":
    raise SystemExit(main())
