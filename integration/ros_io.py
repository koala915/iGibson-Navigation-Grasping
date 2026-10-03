#!/usr/bin/env python3
"""rosbridge I/O for the Python 3.8 mission process.

ROS Melodic is Python 2.7 and the policies are Python 3.8, so the mission
process never imports rospy. It speaks to ROS over the rosbridge WebSocket,
the same way ``nav_rl.RosLaserScanSource`` already reads ``/scan``.

Direction of traffic:

    OUT  /odom_setmotor   nav_msgs/Odometry              -> AMCL
    OUT  /tf              odom -> base_footprint         -> AMCL / TF tree
    IN   /amcl_pose       geometry_msgs/PoseWithCovarianceStamped

Ownership (INTEGRATION_CONTRACT.md section 5, SETMOTOR_ODOM_INTEGRATION.md 2.2):
this process owns ``odom -> base_footprint`` and ``/odom_setmotor`` and nothing
else. ``map -> odom`` belongs to AMCL, ``base_footprint -> base_link`` and
``base_link -> laser_link`` to robot_state_publisher. Publishing a second copy
of any of those makes the TF tree ambiguous and AMCL unusable -- so if the ROS
side is already running a chassis driver, stop it before starting this.

The odometry message and its TF must carry the SAME timestamp; AMCL matches
them by time and a skew shows up as a heading error.

Run:
    python3 integration/ros_io.py --selftest
    python3 integration/ros_io.py --probe --ros-host 127.0.0.1   # live check
"""
from __future__ import annotations

import argparse
import math
import threading
import time
from typing import Any, Mapping, Optional, Tuple

try:  # direct execution vs package import, same pattern as the other modules
    from map_goal_provider import MapPose
    import trash_target
except ImportError:  # pragma: no cover - exercised by the package path
    from .map_goal_provider import MapPose
    from . import trash_target

ODOM_TOPIC = "/odom_setmotor"      # NOT /odom: Route A's canonical name
TF_TOPIC = "/tf"
AMCL_TOPIC = "/amcl_pose"
# Published by detection/rear_cam_sam2_publisher.py, which runs OFF the robot.
TRASH_TOPIC = "/trash_target/detection"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
MAP_FRAME = "map"

# AMCL's tight re-initialisation converged to < 0.038 m / < 1.1 deg. A pose
# whose own covariance is far worse than that is a lost localisation, not a
# usable fix -- the patrol states must stop rather than drive on it.
DEFAULT_MAX_POS_VAR = 0.25 ** 2         # m^2
DEFAULT_MAX_YAW_VAR = math.radians(20.0) ** 2
DEFAULT_MAX_AMCL_AGE_S = 5.0            # local receipt age, not ROS stamp age

# AMCL publishes /amcl_pose only when its filter updates, and it updates on
# MOTION. The mission stands still for up to ~140 s per object (GRASP 120 s +
# CARRY_HOME 20 s) and then enters DELIVER, where a stale pose is a blocking
# fault -- so standing still would end the mission. Route B solves this with
# amcl_nomotion_keepalive.py, but route_b_startup.sh starts that node only in
# start_dryrun, a phase the mission does not use. Ask for the update here
# instead, so the freshness the mission requires is the mission's own business.
NOMOTION_SERVICE = "/request_nomotion_update"
NOMOTION_MIN_PERIOD_S = 0.25            # same rate as the Route B keepalive
NOMOTION_TRIGGER_AGE_S = 0.4            # under MapGoalProvider's 1.0 s window


# ════════════════════════════════════════════════════════════════════════════
# Message helpers
# ════════════════════════════════════════════════════════════════════════════

def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Planar yaw -> (x, y, z, w)."""
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """(x, y, z, w) -> yaw. Full formula, so a non-planar quaternion (AMCL can
    emit tiny roll/pitch noise) still yields the correct heading."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def ros_stamp(seconds: float) -> dict:
    secs = int(seconds)
    return {"secs": secs, "nsecs": int(round((seconds - secs) * 1e9))}


def stamp_seconds(stamp: Mapping[str, Any]) -> Optional[float]:
    try:
        return float(stamp["secs"]) + float(stamp["nsecs"]) * 1e-9
    except (KeyError, TypeError, ValueError):
        return None


def build_odom_message(x: float, y: float, yaw: float,
                       vx: float, vy: float, wz: float,
                       stamp: float, covariance,
                       odom_frame: str = ODOM_FRAME,
                       base_frame: str = BASE_FRAME,
                       twist_covariance=None) -> dict:
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    cov = list(covariance)
    if len(cov) != 36:
        raise ValueError(f"odom covariance must have 36 entries, got {len(cov)}")
    return {
        "header": {"stamp": ros_stamp(stamp), "frame_id": odom_frame},
        "child_frame_id": base_frame,
        "pose": {
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
            "covariance": cov,
        },
        "twist": {
            "twist": {
                "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
            },
            "covariance": list(twist_covariance) if twist_covariance else cov,
        },
    }


def build_tf_message(x: float, y: float, yaw: float, stamp: float,
                     odom_frame: str = ODOM_FRAME,
                     base_frame: str = BASE_FRAME) -> dict:
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    return {"transforms": [{
        "header": {"stamp": ros_stamp(stamp), "frame_id": odom_frame},
        "child_frame_id": base_frame,
        "transform": {
            "translation": {"x": float(x), "y": float(y), "z": 0.0},
            "rotation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
    }]}


def parse_amcl_pose(message: Mapping[str, Any]) -> Tuple[MapPose, float, float]:
    """geometry_msgs/PoseWithCovarianceStamped -> (MapPose, pos_var, yaw_var).

    The stamp is the ROS one; the caller decides whether to trust it or use
    local arrival time (the mission loop uses arrival time for safety ageing,
    exactly like the /scan source does).
    """
    header = message.get("header")
    if not isinstance(header, Mapping):
        raise ValueError("AMCL pose has no header")
    frame = str(header.get("frame_id") or "")
    if frame != MAP_FRAME:
        raise ValueError(f"AMCL pose frame must be {MAP_FRAME!r}, got {frame!r}")

    pose_block = message["pose"]
    pose = pose_block["pose"]
    pos = pose["position"]
    ori = pose["orientation"]
    x, y = float(pos["x"]), float(pos["y"])
    qx, qy, qz, qw = (float(ori["x"]), float(ori["y"]),
                      float(ori["z"]), float(ori["w"]))
    if not all(math.isfinite(v) for v in (qx, qy, qz, qw)):
        raise ValueError("AMCL orientation contains non-finite values")
    qnorm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if qnorm < 1e-9 or abs(qnorm - 1.0) > 0.01:
        raise ValueError(f"AMCL quaternion is not normalized (norm={qnorm:.6f})")
    yaw = quaternion_to_yaw(qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm)
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("AMCL pose contains non-finite values")

    cov = pose_block.get("covariance") or []
    if len(cov) >= 36:
        # Row-major 6x6 over (x y z roll pitch yaw): take the worst planar
        # position variance and the yaw variance.
        pos_var = max(float(cov[0]), float(cov[7]))
        yaw_var = float(cov[35])
        if not all(math.isfinite(v) and v >= 0.0 for v in (pos_var, yaw_var)):
            raise ValueError("AMCL covariance must be finite and non-negative")
    else:
        pos_var = yaw_var = float("nan")

    stamp = stamp_seconds(header.get("stamp") or {})
    return MapPose(x, y, yaw, stamp if stamp is not None else 0.0), pos_var, yaw_var


# ════════════════════════════════════════════════════════════════════════════
# Bridge
# ════════════════════════════════════════════════════════════════════════════

class NullRosIO:
    """Dry-run stand-in. Same surface, publishes nothing, never has a pose."""

    connected = False

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.published = 0

    def publish_odom_and_tf(self, state, covariance, stamp=None) -> None:
        self.published += 1
        if self.verbose:
            print(f"[ros-io][dry] odom+tf x={state.x:+.3f} y={state.y:+.3f} "
                  f"yaw={math.degrees(state.yaw):+.1f}deg")

    def latest_pose(self):
        return None

    def latest_trash_target(self):
        return None

    def pose_age(self, now: Optional[float] = None) -> float:
        return float("inf")

    def pose_quality_ok(self, max_age_s: Optional[float] = None) -> Tuple[bool, str]:
        return False, "dry-run: no AMCL"

    def request_nomotion_update(self, **kw) -> bool:
        return False

    def refresh_pose(self, timeout_s: float = 3.0, poll_s: float = 0.05) -> bool:
        return False

    def rosapi_list(self, what: str, timeout_s: float = 3.0):
        return None

    def close(self) -> None:
        pass


class RosBridgeIO:
    """One rosbridge connection carrying odom out and the AMCL pose in."""

    def __init__(self, host: str = "127.0.0.1", port: int = 9090, *,
                 connect_timeout_s: float = 5.0,
                 odom_topic: str = ODOM_TOPIC,
                 amcl_topic: str = AMCL_TOPIC,
                 trash_topic: Optional[str] = None,
                 tf_topic: str = TF_TOPIC,
                 odom_frame: str = ODOM_FRAME,
                 base_frame: str = BASE_FRAME,
                 max_pos_var: float = DEFAULT_MAX_POS_VAR,
                 max_yaw_var: float = DEFAULT_MAX_YAW_VAR,
                 publish_tf: bool = True,
                 nomotion_service: str = NOMOTION_SERVICE,
                 roslibpy_module=None):
        if not host:
            raise ValueError("ROS bridge host must not be empty")
        if not 1 <= int(port) <= 65535:
            raise ValueError(f"ROS bridge port out of range: {port}")
        for name, topic in (("odom", odom_topic), ("amcl", amcl_topic), ("tf", tf_topic)):
            if not str(topic).startswith("/"):
                raise ValueError(f"{name} topic must be absolute, got {topic!r}")

        self.odom_frame, self.base_frame = odom_frame, base_frame
        self.max_pos_var, self.max_yaw_var = float(max_pos_var), float(max_yaw_var)
        self._publish_tf = bool(publish_tf)

        self._pose: Optional[MapPose] = None
        self._pos_var = self._yaw_var = float("nan")
        self._recv_ts = 0.0
        self._trash: Optional["trash_target.TrashTarget"] = None
        self._trash_recv_ts = 0.0
        self._trash_error = ""
        self._lock = threading.Lock()
        self._last_error = ""
        self.published = 0
        self._nomotion = None
        self._nomotion_ts = 0.0
        self._nomotion_error = ""
        self.nomotion_calls = 0

        roslibpy = roslibpy_module
        if roslibpy is None:
            try:
                import roslibpy  # type: ignore
            except ImportError as exc:
                raise RuntimeError(
                    "roslibpy is required for ROS I/O; run "
                    "'python3 -m pip install roslibpy' inside grasp_venv"
                ) from exc

        self._client = roslibpy.Ros(host=host, port=int(port))
        self._odom = self._tf = self._amcl = self._trash_topic = None
        try:
            self._client.run(timeout=float(connect_timeout_s))
            if not self._client.is_connected:
                raise ConnectionError("rosbridge connection did not become ready")
            self._odom = roslibpy.Topic(self._client, odom_topic, "nav_msgs/Odometry")
            self._odom.advertise()
            if self._publish_tf:
                self._tf = roslibpy.Topic(self._client, tf_topic, "tf2_msgs/TFMessage")
                self._tf.advertise()
            self._amcl = roslibpy.Topic(
                self._client, amcl_topic,
                "geometry_msgs/PoseWithCovarianceStamped", queue_length=1)
            self._amcl.subscribe(self._on_amcl)
            if trash_topic:
                self._trash_topic = roslibpy.Topic(
                    self._client, trash_topic, "std_msgs/String", queue_length=1)
                self._trash_topic.subscribe(self._on_trash)
            # Constructing a Service does not require it to be advertised yet;
            # whether AMCL actually offers it is checked by the preflight.
            self._nomotion = roslibpy.Service(self._client, nomotion_service,
                                              "std_srvs/Empty")
            self._roslibpy = roslibpy
        except Exception as exc:
            self.close()
            raise RuntimeError(f"cannot open ROS bridge at ws://{host}:{port}: {exc}") from exc

    @property
    def connected(self) -> bool:
        client = getattr(self, "_client", None)
        return bool(client is not None and client.is_connected)

    # ── inbound ──
    def _on_amcl(self, message: Mapping[str, Any]) -> None:
        try:
            pose, pos_var, yaw_var = parse_amcl_pose(message)
        except (KeyError, TypeError, ValueError) as exc:
            error = str(exc)
            if error != self._last_error:
                print(f"[ros-io] invalid /amcl_pose: {error}")
                self._last_error = error
            return
        with self._lock:
            self._pose = pose
            self._pos_var, self._yaw_var = pos_var, yaw_var
            self._recv_ts = time.monotonic()
            self._last_error = ""

    def _on_trash(self, message: Mapping[str, Any]) -> None:
        target = trash_target.parse_trash_target(message)
        if target is None:
            error = "unparseable /trash_target/detection payload"
            if error != self._trash_error:
                print(f"[ros-io] {error}")
                self._trash_error = error
            return
        self._trash_error = ""
        with self._lock:
            self._trash = target
            self._trash_recv_ts = time.monotonic()

    def latest_trash_target(self):
        """Latest offboard target, restamped to LOCAL arrival time.

        Same reasoning as ``latest_pose``, and it matters more here: the
        publisher runs on a development machine, so its ``timestamp_unix``
        comes off a different clock entirely. Ageing by it would let a skewed
        laptop clock either hide a dead link or expire a live one. Local
        receipt time is the only clock both ends agree on.
        """
        with self._lock:
            target, ts = self._trash, self._trash_recv_ts
        if target is None:
            return None
        wall = time.time() - (time.monotonic() - ts)
        return trash_target.TrashTarget(
            target.valid, target.x_forward_m, target.y_left_m,
            target.distance_m, wall, target.reason, target.source)

    def latest_pose(self) -> Optional[MapPose]:
        """Latest AMCL pose, restamped to LOCAL arrival time.

        The mission loop ages poses to decide whether it may drive, and the
        Jetson's clock is not necessarily the ROS clock. Same reasoning as
        nav_rl.RosLaserScanSource using local receipt time for safety.
        """
        with self._lock:
            pose, ts = self._pose, self._recv_ts
        if pose is None:
            return None
        wall = time.time() - (time.monotonic() - ts)
        return MapPose(pose.x, pose.y, pose.yaw, wall)

    def pose_age(self, now: Optional[float] = None) -> float:
        with self._lock:
            ts = self._recv_ts
        if not ts:
            return float("inf")
        return time.monotonic() - ts

    def pose_quality_ok(self, max_age_s: Optional[float] = DEFAULT_MAX_AMCL_AGE_S
                        ) -> Tuple[bool, str]:
        """Reject stale or diverged AMCL before it steers the robot.

        The ROS header timestamp cannot be used for safety ageing: the Jetson
        and ROS master may not share a clock.  ``_recv_ts`` is updated only by
        a real callback, never by the nomotion keepalive service.
        """
        with self._lock:
            pose, pos_var, yaw_var = self._pose, self._pos_var, self._yaw_var
            recv_ts = self._recv_ts
        if pose is None:
            return False, "no /amcl_pose received yet"
        if max_age_s is not None:
            try:
                max_age_s = float(max_age_s)
            except (TypeError, ValueError):
                return False, f"invalid AMCL max age {max_age_s!r}"
            if not math.isfinite(max_age_s) or max_age_s <= 0.0:
                return False, f"invalid AMCL max age {max_age_s!r}"
            age = time.monotonic() - recv_ts if recv_ts else float("inf")
            if age > max_age_s:
                return False, (f"AMCL pose stale ({age:.2f}s > "
                               f"{max_age_s:.2f}s); set a fresh 2D Pose Estimate")
        if math.isnan(pos_var) or math.isnan(yaw_var):
            return False, "AMCL pose carried no covariance"
        if pos_var > self.max_pos_var:
            return False, (f"AMCL position variance {pos_var:.4f} > "
                           f"{self.max_pos_var:.4f} m^2 (localisation lost)")
        if yaw_var > self.max_yaw_var:
            return False, (f"AMCL yaw variance {math.degrees(math.sqrt(yaw_var)):.1f}deg "
                           f"> {math.degrees(math.sqrt(self.max_yaw_var)):.1f}deg")
        return True, ""

    def request_nomotion_update(self, *, min_period_s: float = NOMOTION_MIN_PERIOD_S,
                                trigger_age_s: float = NOMOTION_TRIGGER_AGE_S,
                                force: bool = False) -> bool:
        """Ask AMCL to re-run its filter so a standing robot keeps a fresh fix.

        Fire-and-forget: the resulting pose arrives on /amcl_pose like any
        other, so nothing here needs a reply. Only fires when the pose is
        already going stale -- calling it at a flat 4 Hz the way the Route B
        node does would keep resampling the filter against one unchanging scan
        while the robot drives, which is how a particle set over-converges.

        Returns whether a call was actually issued.
        """
        service = self._nomotion
        if service is None:
            return False
        now = time.monotonic()
        if not force:
            if self.pose_age() < float(trigger_age_s):
                return False
            if now - self._nomotion_ts < float(min_period_s):
                return False
        self._nomotion_ts = now

        def _failed(error):
            error = str(error)
            if error != self._nomotion_error:
                print(f"[ros-io] {NOMOTION_SERVICE} failed: {error}. AMCL cannot "
                      "refresh while the robot is stationary; the mission will "
                      "pause when it next needs a map fix.")
                self._nomotion_error = error

        try:
            service.call(self._roslibpy.ServiceRequest({}),
                         callback=lambda _result: None, errback=_failed)
        except Exception as exc:                      # transport died mid-call
            _failed(exc)
            return False
        self.nomotion_calls += 1
        return True

    def refresh_pose(self, timeout_s: float = 3.0, poll_s: float = 0.05) -> bool:
        """Force a fresh fix and wait briefly for it, for use after a long block.

        The mission runs a whole grasp episode inside one tick, so nothing can
        keep the fix alive while it does. Coming back with a minutes-old pose
        means the next map state refuses to drive. The base is stopped at that
        moment, so a bounded synchronous wait costs nothing and is far cheaper
        than pausing and asking a human to re-pose in RViz.
        """
        if not self.request_nomotion_update(force=True):
            return self.pose_age() <= NOMOTION_TRIGGER_AGE_S
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while True:
            if self.pose_age() <= NOMOTION_TRIGGER_AGE_S:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(max(0.0, float(poll_s)))
            self.request_nomotion_update()

    def rosapi_list(self, what: str, timeout_s: float = 3.0) -> Optional[list]:
        """Query /rosapi/nodes or /rosapi/services.

        None means rosapi did not answer -- "cannot verify", which is not the
        same as "not there" and must not be reported as one.
        """
        if what not in ("nodes", "services"):
            raise ValueError(f"rosapi_list takes 'nodes' or 'services', got {what!r}")
        client = getattr(self, "_client", None)
        if client is None or not self.connected:
            return None
        try:
            service = self._roslibpy.Service(client, f"/rosapi/{what}",
                                             f"rosapi/{what.capitalize()}")
            result = service.call(self._roslibpy.ServiceRequest({}),
                                  timeout=float(timeout_s))
        except Exception:
            return None
        if not isinstance(result, Mapping):
            return None
        names = result.get(what)
        return [str(n) for n in names] if isinstance(names, (list, tuple)) else None

    # ── outbound ──
    def publish_odom_and_tf(self, state, covariance, stamp: Optional[float] = None) -> None:
        """Publish /odom_setmotor and the odom->base_footprint TF at one stamp.

        An invalid odom state is dropped rather than published: feeding AMCL a
        frozen pose stamped 'now' would tell it the robot stopped, which is the
        one lie that corrupts localisation silently.
        """
        if not state.valid:
            return
        stamp = time.time() if stamp is None else float(stamp)
        odom_msg = build_odom_message(
            state.x, state.y, state.yaw, state.vx, state.vy, state.wz,
            stamp, covariance, self.odom_frame, self.base_frame)
        self._odom.publish(self._roslibpy.Message(odom_msg))
        if self._tf is not None:
            tf_msg = build_tf_message(state.x, state.y, state.yaw, stamp,
                                      self.odom_frame, self.base_frame)
            self._tf.publish(self._roslibpy.Message(tf_msg))
        self.published += 1

    def close(self) -> None:
        self._nomotion = None
        for attr in ("_amcl", "_trash_topic", "_odom", "_tf"):
            topic = getattr(self, attr, None)
            setattr(self, attr, None)
            if topic is None:
                continue
            try:
                topic.unsubscribe()
            except Exception:
                pass
            try:
                topic.unadvertise()
            except Exception:
                pass
        client, self._client = getattr(self, "_client", None), None
        if client is not None:
            try:
                client.terminate()
            except Exception:
                pass


def make_ros_io(kind: str = "ros", *, host: str = "127.0.0.1", port: int = 9090,
                verbose: bool = False, **kw):
    kind = str(kind).lower()
    if kind in ("none", "null", "dry"):
        print("[ros-io] WARNING: --ros-backend none — no odometry is published, "
              "so AMCL cannot localise and map-frame patrol is unavailable.")
        return NullRosIO(verbose=verbose)
    if kind == "ros":
        return RosBridgeIO(host, port, **kw)
    raise ValueError(f"unsupported ROS backend: {kind!r}")


# ════════════════════════════════════════════════════════════════════════════
# Selftest — a fake roslibpy, so no ROS and no network are needed
# ════════════════════════════════════════════════════════════════════════════

class _FakeTopic:
    def __init__(self, client, name, mtype, queue_length=None):
        self.client, self.name, self.mtype = client, name, mtype
        self.advertised = False
        self.subscriber = None
        self.messages = []
        client.topics.append(self)

    def advertise(self):
        self.advertised = True

    def unadvertise(self):
        self.advertised = False

    def subscribe(self, cb):
        self.subscriber = cb

    def unsubscribe(self):
        self.subscriber = None

    def publish(self, message):
        if not self.advertised:
            raise RuntimeError(f"published to {self.name} without advertising")
        self.messages.append(message.data if hasattr(message, "data") else message)


class _FakeService:
    def __init__(self, client, name, stype):
        self.client, self.name, self.stype = client, name, stype
        client.services.append(self)

    def call(self, request, callback=None, errback=None, timeout=None):
        self.client.calls.append(self.name)
        answer = self.client.answers.get(self.name, {})
        if isinstance(answer, Exception):
            if errback is None:
                raise answer
            errback(answer)
            return None
        if callback is not None:          # async form: fire-and-forget
            callback(answer)
            return None
        return answer


class _FakeRos:
    def __init__(self, host, port):
        self.host, self.port = host, port
        self.is_connected = False
        self.topics = []
        self.services = []
        self.calls = []
        self.answers = {}
        self.terminated = False

    def run(self, timeout=None):
        self.is_connected = True

    def terminate(self):
        self.is_connected = False
        self.terminated = True


class _FakeRoslibpy:
    Ros = _FakeRos
    Topic = _FakeTopic
    Service = _FakeService

    class Message:
        def __init__(self, data):
            self.data = data

    class ServiceRequest(dict):
        pass


def run_selftest() -> None:
    def approx(a, b, tol=1e-9):
        assert abs(a - b) <= tol, f"{a} != {b} (tol {tol})"

    # ── quaternion round trip, including a non-planar input ──
    for deg in (-180.0, -90.0, -0.5, 0.0, 30.0, 90.0, 179.0):
        rad = math.radians(deg)
        approx(quaternion_to_yaw(*yaw_to_quaternion(rad)), rad, 1e-12)
    # tilted quaternion (roll 5 deg, yaw 40 deg) must still give yaw 40
    r, y = math.radians(5.0), math.radians(40.0)
    cr, sr, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(y / 2), math.sin(y / 2)
    approx(quaternion_to_yaw(sr * cy, sr * sy, cr * sy, cr * cy), y, 1e-12)
    print("[ros-io] quaternion helpers OK")

    # ── stamps ──
    s = ros_stamp(1234.5)
    assert s == {"secs": 1234, "nsecs": 500000000}, s
    approx(stamp_seconds(s), 1234.5, 1e-6)
    assert stamp_seconds({"secs": "x"}) is None
    print("[ros-io] stamp helpers OK")

    # ── message shape ──
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.0025
    cov[35] = 0.0076
    odom = build_odom_message(1.0, 2.0, math.pi / 2, 0.3, 0.0, 0.1, 100.25, cov)
    assert odom["header"]["frame_id"] == "odom"
    assert odom["child_frame_id"] == "base_footprint"
    approx(odom["pose"]["pose"]["position"]["x"], 1.0)
    approx(quaternion_to_yaw(**odom["pose"]["pose"]["orientation"]), math.pi / 2, 1e-12)
    assert len(odom["pose"]["covariance"]) == 36
    tf = build_tf_message(1.0, 2.0, math.pi / 2, 100.25)
    assert tf["transforms"][0]["header"]["stamp"] == odom["header"]["stamp"], \
        "odom and TF must share one timestamp"
    try:
        build_odom_message(0, 0, 0, 0, 0, 0, 0.0, [0.0] * 12)
    except ValueError:
        pass
    else:
        raise AssertionError("short covariance should be rejected")
    print("[ros-io] message construction OK")

    # ── amcl parsing ──
    def amcl_msg(x, y, yaw, pvar, yvar, stamp=7.5):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        c = [0.0] * 36
        c[0] = c[7] = pvar
        c[35] = yvar
        return {"header": {"stamp": ros_stamp(stamp), "frame_id": "map"},
                "pose": {"pose": {"position": {"x": x, "y": y, "z": 0.0},
                                  "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}},
                         "covariance": c}}

    pose, pvar, yvar = parse_amcl_pose(amcl_msg(3.45, 11.98, 0.25, 0.01, 0.02))
    approx(pose.x, 3.45); approx(pose.y, 11.98); approx(pose.yaw, 0.25, 1e-12)
    approx(pose.stamp, 7.5, 1e-6); approx(pvar, 0.01); approx(yvar, 0.02)
    try:
        parse_amcl_pose(amcl_msg(float("nan"), 0.0, 0.0, 0.01, 0.02))
    except ValueError:
        pass
    else:
        raise AssertionError("non-finite AMCL pose should be rejected")
    print("[ros-io] /amcl_pose parsing OK")

    # ── full bridge against the fake ──
    fake = _FakeRoslibpy()
    io = RosBridgeIO("127.0.0.1", 9090, roslibpy_module=fake)
    assert io.connected
    names = {t.name: t for t in io._client.topics}
    assert set(names) == {"/odom_setmotor", "/tf", "/amcl_pose"}, names
    assert names["/odom_setmotor"].advertised and names["/tf"].advertised
    assert names["/amcl_pose"].subscriber is not None
    assert names["/odom_setmotor"].mtype == "nav_msgs/Odometry"
    assert names["/tf"].mtype == "tf2_msgs/TFMessage"

    ok, why = io.pose_quality_ok()
    assert not ok and "no /amcl_pose received" in why, why
    assert io.latest_pose() is None and io.pose_age() == float("inf")

    names["/amcl_pose"].subscriber(amcl_msg(3.45, 11.98, 0.25, 0.01, 0.02))
    p = io.latest_pose()
    assert p is not None and abs(p.x - 3.45) < 1e-9
    assert p.stamp > 1e9, "pose must be restamped to local wall time"
    assert io.pose_age() < 1.0
    ok, why = io.pose_quality_ok()
    assert ok, why

    # diverged filter must be refused
    names["/amcl_pose"].subscriber(amcl_msg(3.45, 11.98, 0.25, 1.0, 0.02))
    ok, why = io.pose_quality_ok()
    assert not ok and "position variance" in why, why
    names["/amcl_pose"].subscriber(amcl_msg(3.45, 11.98, 0.25, 0.01, 5.0))
    ok, why = io.pose_quality_ok()
    assert not ok and "yaw variance" in why, why
    # covariance-less pose must be refused, not assumed good
    m = amcl_msg(1, 2, 0, 0.01, 0.01); m["pose"]["covariance"] = []
    names["/amcl_pose"].subscriber(m)
    ok, why = io.pose_quality_ok()
    assert not ok and "no covariance" in why, why
    print("[ros-io] AMCL freshness + covariance gating OK")

    # ── nomotion keepalive: only fires when the fix is actually going stale ──
    fake_ros = io._client
    fake_ros.calls = []
    names["/amcl_pose"].subscriber(amcl_msg(3.45, 11.98, 0.25, 0.01, 0.02))
    assert not io.request_nomotion_update(), \
        "a fresh pose must not trigger a filter update"
    assert io.request_nomotion_update(force=True), "force must always call"
    assert fake_ros.calls == [NOMOTION_SERVICE], fake_ros.calls

    # a pose that has aged past the trigger fires, then rate-limits
    with io._lock:
        io._recv_ts = time.monotonic() - 3.0
    io._nomotion_ts = 0.0
    assert io.request_nomotion_update(), "a stale pose must trigger an update"
    assert not io.request_nomotion_update(), "must be rate limited"
    assert io.request_nomotion_update(min_period_s=0.0), "period 0 always fires"
    assert io.nomotion_calls == 3, io.nomotion_calls    # force + stale + period 0

    # a service that is not there must report once, not crash the mission loop
    fake_ros.answers[NOMOTION_SERVICE] = RuntimeError("service /request_nomotion_update unavailable")
    io._nomotion_ts = 0.0
    assert io.request_nomotion_update(), "an errback failure is still a call attempt"
    # a fix that never arrives must time out, not hang the mission forever
    fake_ros.answers.pop(NOMOTION_SERVICE, None)
    with io._lock:
        io._recv_ts = time.monotonic() - 90.0
    assert not io.refresh_pose(timeout_s=0.05, poll_s=0.01), \
        "refresh_pose must give up rather than block forever"
    names["/amcl_pose"].subscriber(amcl_msg(3.45, 11.98, 0.25, 0.01, 0.02))
    assert io.refresh_pose(timeout_s=0.05), "a fresh pose means the refresh worked"
    print("[ros-io] nomotion keepalive OK")

    # ── rosapi introspection: absent is not the same as unverifiable ──
    fake_ros.answers["/rosapi/services"] = {"services": ["/request_nomotion_update"]}
    assert io.rosapi_list("services") == ["/request_nomotion_update"]
    fake_ros.answers["/rosapi/nodes"] = {"nodes": ["/amcl", "/map_server"]}
    assert io.rosapi_list("nodes") == ["/amcl", "/map_server"]
    fake_ros.answers["/rosapi/services"] = RuntimeError("rosapi not running")
    assert io.rosapi_list("services") is None, "unreachable rosapi means unknown"
    fake_ros.answers["/rosapi/services"] = {"wrong_key": []}
    assert io.rosapi_list("services") is None, "a malformed reply means unknown"
    try:
        io.rosapi_list("topics")
    except ValueError:
        pass
    else:
        raise AssertionError("rosapi_list should reject an unsupported query")
    print("[ros-io] rosapi introspection OK")

    # ── publishing ──
    try:
        from feedback_odom import MotionFeedbackOdom
    except ImportError:
        from .feedback_odom import MotionFeedbackOdom
    odo = MotionFeedbackOdom()
    t = 1000.0
    for _ in range(4):
        t += 0.05
        odo.update(0.5, 0.0, 0.2, 0.05, stamp=t)
    st = odo.state(now=t)
    io.publish_odom_and_tf(st, odo.covariance(), stamp=t)
    assert len(names["/odom_setmotor"].messages) == 1
    assert len(names["/tf"].messages) == 1
    assert (names["/odom_setmotor"].messages[0]["header"]["stamp"]
            == names["/tf"].messages[0]["transforms"][0]["header"]["stamp"]), \
        "the published odom and its TF must share one timestamp"

    bad = odo.record_failure("simulated dropout", stamp=t)
    io.publish_odom_and_tf(bad, odo.covariance(), stamp=t)
    assert len(names["/odom_setmotor"].messages) == 1, \
        "an invalid odom state must NOT be published"
    print("[ros-io] publish + invalid-state suppression OK")

    io.close()
    assert not io.connected and io._client is None
    print("[ros-io] close OK")

    # ── null backend ──
    n = make_ros_io("none")
    n.publish_odom_and_tf(st, odo.covariance())
    assert n.published == 1 and n.latest_pose() is None
    ok, why = n.pose_quality_ok()
    assert not ok and "dry-run" in why
    n.close()
    for bad_kw in (dict(host=""), dict(port=0), dict(odom_topic="odom")):
        try:
            RosBridgeIO(roslibpy_module=fake, **bad_kw)
        except ValueError:
            continue
        raise AssertionError(f"{bad_kw} should have been rejected")
    print("[ros-io] null backend + argument validation OK")

    print("[ros-io] SELFTEST PASSED")


def run_probe(host: str, port: int, seconds: float) -> int:
    """Live check: is AMCL publishing, and can we advertise odom?"""
    io = RosBridgeIO(host, port)
    print(f"[ros-io] connected to ws://{host}:{port}; listening {seconds:.0f}s "
          f"for {AMCL_TOPIC} ...")
    try:
        deadline = time.time() + seconds
        seen = 0
        while time.time() < deadline:
            time.sleep(0.5)
            pose = io.latest_pose()
            if pose is None:
                continue
            seen += 1
            ok, why = io.pose_quality_ok()
            print(f"  pose x={pose.x:+.3f} y={pose.y:+.3f} "
                  f"yaw={math.degrees(pose.yaw):+7.2f}deg age={io.pose_age():.2f}s "
                  f"quality={'OK' if ok else why}")
        if not seen:
            print("[ros-io] NO /amcl_pose seen. Start map_server + AMCL and set the "
                  "initial pose in RViz (tight init: std 0.15 m / 7 deg).")
            return 1
        return 0
    finally:
        io.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--probe", action="store_true", help="live rosbridge check")
    ap.add_argument("--ros-host", default="127.0.0.1")
    ap.add_argument("--ros-port", type=int, default=9090)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    if args.selftest:
        run_selftest()
        return 0
    if args.probe:
        return run_probe(args.ros_host, args.ros_port, args.seconds)
    ap.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
