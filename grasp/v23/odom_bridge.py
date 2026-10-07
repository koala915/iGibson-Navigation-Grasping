#!/usr/bin/env python3
"""G2 step 3: wheel odometry from the resident service, published over rosbridge.

Whoever owns /dev/myserial has to publish odometry, because the wheel feedback
comes in on that port. Since G2 that owner is the grasp service, so it now
publishes /odom_setmotor (nav_msgs/Odometry) and the odom -> base_footprint TF
at 20 Hz, the job the motor server used to do with rospy. The service runs in
the Python 3.8 venv, which cannot import rospy (ROS Melodic is Python 2.7), so
it goes through rosbridge the way integration/ros_io.py does.

Reused, not copied: FeedbackOdomReader (integration/feedback_odom.py, whatever
calibration is deployed there) and the message builders of ros_io.py, so the
frames, the covariance and the shared odom/TF stamp are the mission's own.

Not reused: RosBridgeIO's connection handling. It was written for a mission
that starts after ROS; this service starts at boot, usually before ROS. Two
roslibpy 2.0.1 behaviours matter here:
  * On a failed connect RosBridgeIO calls terminate(), which stops the global
    twisted reactor. A stopped reactor can never be restarted, so every later
    attempt in this process would fail silently. Here the one client is created
    once and never terminated; roslibpy's own reconnecting factory retries.
  * Topic.publish() queues messages until the connection is ready. Publishing
    while rosbridge is down would hand AMCL a burst of stale odometry the moment
    it came up. Here nothing is published unless the client is connected.
The wheel feedback is polled at 20 Hz regardless, so the pose keeps integrating
through a rosbridge outage instead of restarting from zero.
"""
from __future__ import annotations

import importlib
import sys
import threading
import time
from pathlib import Path

RATE_HZ = 20.0
RECONNECT_MAX_S = 5.0       # roslibpy's default backoff grows to an hour
FIRST_CONNECT_WAIT_S = 2.0  # at startup only; never blocks the service longer


def load_odom_modules(repo_root: Path):
    """(feedback_odom, ros_io) from integration/, searched after grasp/v23."""
    integration = str(Path(repo_root) / "integration")
    if integration not in sys.path:
        sys.path.append(integration)
    return (importlib.import_module("feedback_odom"),
            importlib.import_module("ros_io"))


class OdomBridge:
    def __init__(self, reader, ros_io, roslibpy, *, host: str = "127.0.0.1",
                 port: int = 9090, rate_hz: float = RATE_HZ, clock=time.time,
                 log=print, feedback_age=None, feedback_stale_s: float = 0.5):
        self.reader = reader
        # get_motion_data() hands back the last cached values forever once the
        # board's data stops arriving -- indistinguishable from standing still.
        # feedback_age (seconds since the board last sent a byte) is what tells.
        self._feedback_age = feedback_age
        self._feedback_stale_s = float(feedback_stale_s)
        self.skipped_silent = 0
        self._ros_io = ros_io
        self._roslibpy = roslibpy
        self.host, self.port = host, int(port)
        self.period = 1.0 / float(rate_hz)
        self._clock = clock
        self._log = log
        self._stop = threading.Event()
        self._thread = None
        self._client = self._odom = self._tf = None
        self._lock = threading.Lock()
        self._state = None
        self._connected = False
        self.published = 0
        self.skipped_invalid = 0
        self.failures = 0

    # ── lifecycle ──────────────────────────────────────────────────────────
    def start(self) -> None:
        rl = self._roslibpy
        self._client = rl.Ros(host=self.host, port=self.port)
        try:
            self._client.factory.set_max_delay(RECONNECT_MAX_S)
        except Exception:
            pass
        try:
            self._client.run(timeout=FIRST_CONNECT_WAIT_S)
        except Exception:
            self._log("[odom] rosbridge %s:%d not up yet; odometry keeps integrating "
                      "and publishing starts when it is" % (self.host, self.port))
        self._odom = rl.Topic(self._client, self._ros_io.ODOM_TOPIC, "nav_msgs/Odometry")
        self._tf = rl.Topic(self._client, self._ros_io.TF_TOPIC, "tf2_msgs/TFMessage")
        self._thread = threading.Thread(target=self._run, name="odom-bridge", daemon=True)
        self._thread.start()
        self._log("[odom] publishing %s + %s->%s at %.0f Hz via rosbridge %s:%d"
                  % (self._ros_io.ODOM_TOPIC, self._ros_io.ODOM_FRAME,
                     self._ros_io.BASE_FRAME, 1.0 / self.period, self.host, self.port))

    def close(self) -> None:
        # No terminate(): it stops the process-wide reactor, and the process is
        # on its way out anyway. The daemon threads go with it.
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    # ── loop ───────────────────────────────────────────────────────────────
    def tick(self) -> None:
        """One 20 Hz step. Public so tests can drive it without a thread."""
        now = self._clock()
        state = self.reader.poll(now=now)
        with self._lock:
            self._state = state
        connected = bool(self._client is not None and self._client.is_connected)
        if connected != self._connected:
            self._connected = connected
            self._log("[odom] rosbridge %s" % ("connected" if connected else
                                               "lost; odometry keeps integrating"))
        if self._feedback_age is not None and self._feedback_age() > self._feedback_stale_s:
            self.skipped_silent += 1
            if self.skipped_silent in (1, 20, 200) or self.skipped_silent % 2000 == 0:
                self._log("[odom] no data from the board for %.1fs (%dx), not published"
                          % (self._feedback_age(), self.skipped_silent))
            return
        if not state.valid:
            self.skipped_invalid += 1
            if self.skipped_invalid in (1, 20, 200) or self.skipped_invalid % 2000 == 0:
                self._log("[odom] feedback invalid (%dx), not published: %s"
                          % (self.skipped_invalid, state.reason))
            return
        if not connected:
            return
        rio, msg = self._ros_io, self._roslibpy.Message
        self._odom.publish(msg(rio.build_odom_message(
            state.x, state.y, state.yaw, state.vx, state.vy, state.wz,
            now, self.reader.odom.covariance(), rio.ODOM_FRAME, rio.BASE_FRAME)))
        self._tf.publish(msg(rio.build_tf_message(
            state.x, state.y, state.yaw, now, rio.ODOM_FRAME, rio.BASE_FRAME)))
        self.published += 1

    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self.tick()
            except Exception as exc:              # never let odometry kill the thread
                self.failures += 1
                if self.failures in (1, 10, 100) or self.failures % 1000 == 0:
                    self._log("[odom] tick failed (%dx): %s" % (self.failures, exc))
            self._stop.wait(max(0.0, self.period - (time.monotonic() - t0)))

    def status(self) -> dict:
        with self._lock:
            st = self._state
        out = {"rosbridge": self._connected, "published": self.published,
               "skipped_invalid": self.skipped_invalid,
               "skipped_board_silent": self.skipped_silent, "failures": self.failures}
        if st is not None:
            out.update({"valid": bool(st.valid),
                        "pose": [round(st.x, 4), round(st.y, 4), round(st.yaw, 4)],
                        "twist": [round(st.vx, 3), round(st.vy, 3), round(st.wz, 3)],
                        "reason": st.reason})
        if self._feedback_age is not None and self._feedback_age() > self._feedback_stale_s:
            out.update(valid=False, reason='board_feedback_stale')
        return out
