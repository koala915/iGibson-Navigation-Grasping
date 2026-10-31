#!/usr/bin/env python3
"""Pure-logic checks for G2 step 3: odometry from the resident service.

A fake roslibpy stands in for rosbridge and a fake board for the wheel
feedback; FeedbackOdomReader and ros_io's message builders are the real ones
from integration/, loaded the way the service loads them.
"""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import grasp_service as gs  # noqa: E402
import odom_bridge as ob    # noqa: E402

checks = 0
failures = []


def ok(condition, label):
    global checks
    checks += 1
    print("  [{}] {}".format("ok" if condition else "FAIL", label))
    if not condition:
        failures.append(label)


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class FakeFactory:
    max_delay = None

    def set_max_delay(self, s):
        self.max_delay = s


class FakeRos:
    connect_on_run = False

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.factory = FakeFactory()
        self.is_connected = False
        self.terminated = False

    def run(self, timeout):
        if not FakeRos.connect_on_run:
            raise RuntimeError("Failed to connect to ROS")
        self.is_connected = True

    def terminate(self):
        self.terminated = True


class FakeTopic:
    def __init__(self, ros, name, mtype):
        self.name, self.mtype = name, mtype
        self.sent = []

    def publish(self, message):
        self.sent.append(message)


class FakeRoslibpy:
    Ros = FakeRos
    Topic = FakeTopic

    @staticmethod
    def Message(d):
        return d


class FakeBoard:
    """get_motion_data() like Rosmaster: the last cached (vx, vy, wz)."""

    def __init__(self):
        self.motion = (0.0, 0.0, 0.0)
        self.fail = False

    def get_motion_data(self):
        if self.fail:
            raise OSError("serial read failed")
        return self.motion


def main():
    root = HERE.parents[1]
    fo, rio = ob.load_odom_modules(root)
    ok(hasattr(fo, "FeedbackOdomReader") and hasattr(rio, "build_odom_message"),
       "feedback_odom and ros_io load from integration/")
    ok(sys.path.index(str(root / "integration")) > sys.path.index(str(HERE)),
       "integration/ is searched after grasp/v23")

    print("\n== startup before ROS ==")
    board, clock, logs = FakeBoard(), Clock(), []
    reader = fo.FeedbackOdomReader(board, fo.FeedbackOdomConfig())
    br = ob.OdomBridge(reader, rio, FakeRoslibpy, clock=clock, log=logs.append)
    FakeRos.connect_on_run = False
    br.start()
    br.close()                                  # tests drive tick() by hand
    ok(any("not up yet" in m for m in logs), "a missing rosbridge is logged, not raised")
    ok(br._client.factory.max_delay == ob.RECONNECT_MAX_S,
       "reconnect backoff capped at {:.0f} s instead of roslibpy's hour".format(
           ob.RECONNECT_MAX_S))

    print("\n== nothing is queued while rosbridge is down ==")
    board.motion = (0.1, 0.0, 0.0)              # driving at 0.1 m/s (raw)
    for _ in range(21):                          # 1.0 s at 20 Hz
        clock.t += 0.05
        br.tick()
    ok(br._odom.sent == [] and br._tf.sent == [] and br.published == 0,
       "no odom or TF handed to roslibpy while disconnected")
    x_before = br.status()["pose"][0]
    ok(x_before > 0.05, "but the pose kept integrating (x = {:.3f} m)".format(x_before))

    print("\n== publishing once connected ==")
    br._client.is_connected = True
    clock.t += 0.05
    br.tick()
    ok(len(br._odom.sent) == 1 and len(br._tf.sent) == 1, "one odom and one TF per tick")
    odom, tf = br._odom.sent[0], br._tf.sent[0]["transforms"][0]
    ok(odom["header"]["stamp"] == tf["header"]["stamp"], "odom and TF share one stamp")
    ok(odom["header"]["frame_id"] == "odom" and odom["child_frame_id"] == "base_footprint"
       and tf["header"]["frame_id"] == "odom" and tf["child_frame_id"] == "base_footprint",
       "frames are odom -> base_footprint")
    ok(br._odom.name == "/odom_setmotor" and br._tf.name == "/tf", "topics /odom_setmotor and /tf")
    ok(odom["pose"]["pose"]["position"]["x"] >= x_before,
       "the first published pose continues from the outage, not from zero")
    ok(any("connected" in m for m in logs), "the connection is logged once")

    print("\n== bad feedback is never published ==")
    board.fail = True
    n, skipped = len(br._odom.sent), br.skipped_invalid
    clock.t += 0.05
    br.tick()
    ok(len(br._odom.sent) == n and br.skipped_invalid == skipped + 1,
       "a failed read is skipped, not published as 'stopped'")
    board.fail = False
    clock.t += 0.05
    br.tick()
    clock.t += 0.05
    br.tick()
    ok(len(br._odom.sent) > n, "publishing resumes once the feedback is good again")

    print("\n== a silent board is never published as standing still ==")
    silent = {"age": 2.0}
    br._feedback_age = lambda: silent["age"]
    n = len(br._odom.sent)
    clock.t += 0.05
    br.tick()
    ok(len(br._odom.sent) == n and br.skipped_silent == 1,
       "2 s without a byte from the board: frozen feedback is not published")
    silent["age"] = 0.02
    clock.t += 0.05
    br.tick()
    ok(len(br._odom.sent) == n + 1, "publishing resumes when the board talks again")
    br._feedback_age = None

    print("\n== losing rosbridge ==")
    br._client.is_connected = False
    clock.t += 0.05
    br.tick()
    ok(any("lost" in m for m in logs), "the loss is logged")
    ok(not br._client.terminated, "the client is never terminated (it would stop "
       "the process-wide reactor for good)")

    print("\n== the service wiring ==")

    class Dev:
        pass

    class Ctl:
        detection = None
        servo = type("S", (), {"device": Dev(), "read_degrees": lambda self: None})()

    ok(gs.build_odom_bridge(Ctl(), "") is None, "no GRASP_SERVICE_ODOM_ROSBRIDGE, no odometry")
    try:
        gs.build_odom_bridge(Ctl(), "9090")
        ok(False, "an address without a port is refused")
    except ValueError:
        ok(True, "an address without a port is refused")
    svc = gs.GraspService(Ctl(), None, 1, "/tmp/unused.sock", odom_bridge=br)
    ok(isinstance(svc.cmd_status().get("odom"), dict), "status reports the odometry")
    plain = gs.GraspService(Ctl(), None, 1, "/tmp/unused.sock")
    ok(plain.cmd_status().get("odom") is None, "and null when it is off")

    print()
    if failures:
        print("{} of {} checks FAILED:".format(len(failures), checks))
        for label in failures:
            print("  - {}".format(label))
        return 1
    print("all {} checks passed — odometry integrates through rosbridge outages and "
          "is published only when it can be delivered fresh".format(checks))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
