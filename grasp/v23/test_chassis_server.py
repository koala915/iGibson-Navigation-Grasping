#!/usr/bin/env python3
"""Pure-logic checks for G2: TCP 7000 chassis control inside the grasp service.

No serial port, no torch, no PyBullet: a fake board records set_motor calls and
a fake clock drives the 0.5 s rules. The one real dependency is the wheel
mapping, loaded from integration/sugarbox_rl_motor_server.py the same way the
service loads it, so a change there shows up here.

The SIGTERM and unix-socket checks need POSIX and are skipped (not counted) on
Windows; the Jetson runs all of them.
"""

import json
import queue
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import chassis_server as cs  # noqa: E402
import grasp_service as gs   # noqa: E402

POSIX = os.name == "posix"
checks = 0
failures = []


def ok(condition, label):
    global checks
    checks += 1
    print("  [{}] {}".format("ok" if condition else "FAIL", label))
    if not condition:
        failures.append(label)


def skip(label):
    print("  [skip] {} (needs POSIX)".format(label))


def raises(fn):
    try:
        fn()
    except Exception:
        return True
    return False


def wait_for(pred, timeout=1.5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class FakeSer:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(data)
        return len(data)


class FakeBoard:
    """Rosmaster's shape: every command is one ser.write()."""

    def __init__(self):
        self.ser = FakeSer()
        self.calls = []

    def set_motor(self, *values):
        self.calls.append(tuple(values))
        self.ser.write(b"motor")

    @property
    def last(self):
        return self.calls[-1] if self.calls else None


def make_chassis(to_motor, clock=None):
    board = FakeBoard()
    chassis = cs.Chassis(board, to_motor, max_motor=60, watchdog_s=0.5,
                         clock=clock or time.monotonic, log=lambda _msg: None)
    chassis.set_arm_pose("travel")      # R7: the pose these checks drive in
    return board, chassis


def main():
    root = HERE.parents[1]

    print("== wheel mapping is imported from the motor server, not copied ==")
    to_motor, max_motor, watchdog_s = cs.load_motor_mapping(root)
    ok(to_motor(0.15, 0.0) == (30, 30, 30, 30), "vx 0.15 m/s maps to 30 on all four wheels")
    ok(to_motor(0.0, 0.0) == (0, 0, 0, 0), "zero velocity maps to zero")
    ok(max_motor == 60 and watchdog_s == 0.5, "motor clamp 60, watchdog 0.5 s")
    ok(sys.path.index(str(root / "integration")) > sys.path.index(str(HERE)),
       "integration/ is searched after grasp/v23, so it cannot shadow a grasp module")

    print("\n== protocol: exactly the motor server's ==")
    clock = Clock()
    board, ch = make_chassis(to_motor, clock)
    ok(ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0}) == "ok"
       and board.last == (30, 30, 30, 30), "velocity command drives the wheels")
    n = len(board.calls)
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    ok(len(board.calls) == n, "a repeated command does not rewrite the board")
    ok(ch.command({"action": "stop"}) == "stop" and board.last == (0, 0, 0, 0),
       "stop zeroes the wheels")
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    ok(ch.command({"vx": 0.15, "wz": 0.0}) == "stop" and board.last == (0, 0, 0, 0),
       "a message without action is a stop, even when it carries a speed")
    for bad, label in (({"action": "forward", "speed": 30}, "old action/speed format"),
                       ({"action": "velocity", "vx": float("nan"), "wz": 0}, "NaN vx"),
                       ({"action": "velocity", "vx": "fast", "wz": 0}, "non-numeric vx"),
                       ([0.15, 0.0], "a JSON list")):
        ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
        ok(raises(lambda: ch.command(bad)) and board.last == (0, 0, 0, 0),
           "{} is refused and stops the wheels".format(label))

    print("\n== watchdog ==")
    clock = Clock()
    board, ch = make_chassis(to_motor, clock)
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    clock.advance(0.4)
    ok(not ch.watchdog_tick() and board.last == (30, 30, 30, 30), "0.4 s of silence keeps driving")
    clock.advance(0.2)
    ok(ch.watchdog_tick() and board.last == (0, 0, 0, 0), "0.6 s of silence stops the wheels")
    n = len(board.calls)
    clock.advance(5.0)
    ok(not ch.watchdog_tick() and len(board.calls) == n, "a stopped chassis is not rewritten")

    print("\n== shutdown is final ==")
    # The robot, 2026-09-28: systemctl restart zeroed the wheels, then a command
    # already read off the socket put them back to 20 for 2 ms.
    board, ch = make_chassis(to_motor, Clock())
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    ch.shutdown()
    n = len(board.calls)
    ok(board.last == (0, 0, 0, 0)
       and ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0}) == "closed"
       and len(board.calls) == n,
       "a command that arrives after shutdown cannot restart the wheels")

    print("\n== R1: no arm motion while the wheels turn or settle ==")
    clock = Clock()
    board, ch = make_chassis(to_motor, clock)
    ok(ch.begin_arm() is None, "a chassis that never moved lets the arm go at once")
    ch.end_arm()
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    ok(ch.begin_arm() == "chassis_moving" and not ch.status()["arm_busy"],
       "refused while the wheels turn, and the arm is not claimed")
    ch.command({"action": "stop"})
    clock.advance(0.3)
    ok(ch.begin_arm() == "chassis_moving", "refused 0.3 s after the stop")
    for _ in range(3):                      # a client streaming zeros is not moving
        clock.advance(0.1)
        ch.command({"action": "velocity", "vx": 0.0, "wz": 0.0})
    ok(ch.begin_arm() is None, "allowed 0.6 s after the stop, zeros streaming meanwhile")
    ok(ch.refused == 2, "two refusals counted")
    ch.end_arm()
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    clock.advance(0.7)
    ch.watchdog_tick()
    clock.advance(0.3)
    ok(ch.begin_arm() == "chassis_moving", "a watchdog stop starts the settle window too")
    clock.advance(0.3)
    ok(ch.begin_arm() is None, "and it ends 0.5 s after that stop")
    ch.end_arm()

    print("\n== R2: velocity is dropped while the arm moves ==")
    clock = Clock()
    board, ch = make_chassis(to_motor, clock)
    ch.stop()
    ch.begin_arm()
    n = len(board.calls)
    ok(ch.command({"action": "velocity", "vx": 0.15, "wz": 0.5}) == "dropped"
       and len(board.calls) == n, "velocity during the arm never reaches the board")
    ok(ch.command({"action": "stop"}) == "stop", "stop is still accepted during the arm")
    ok(ch.dropped == 1 and ch.status()["arm_busy"], "the drop is counted")
    ch.end_arm()
    ok(ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0}) == "ok"
       and board.last == (30, 30, 30, 30), "the wheels answer again once the arm is done")

    print("\n== R3: one lock around every packet ==")
    lock = threading.Lock()
    board = FakeBoard()
    held = []
    raw = board.ser.write
    board.ser.write = lambda data: (held.append(lock.locked()), raw(data))[1]
    cs.install_write_lock(board, lock)
    cs.install_write_lock(board, lock)      # idempotent
    board.set_motor(1, 1, 1, 1)
    ok(held == [True], "the write runs with the lock held, once, after a double install")

    class SlowSer:
        def __init__(self):
            self.out = []

        def write(self, data):
            for b in data:                   # byte by byte, yielding in between
                self.out.append(b)
                time.sleep(0)
            return len(data)

    dev = type("Dev", (), {})()
    dev.ser = SlowSer()
    cs.install_write_lock(dev, threading.Lock())

    def spam(tag):
        for _ in range(200):
            dev.ser.write(bytes([tag]) * 8)

    threads = [threading.Thread(target=spam, args=(t,)) for t in (1, 2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    chunks = [dev.ser.out[i:i + 8] for i in range(0, len(dev.ser.out), 8)]
    ok(len(chunks) == 400 and all(len(set(c)) == 1 for c in chunks),
       "two threads writing 400 packets never interleave a byte")

    print("\n== TCP front end (real sockets on localhost) ==")
    board, ch = make_chassis(to_motor)
    srv = cs.ChassisServer(ch, host="127.0.0.1", port=0, log=lambda _msg: None)
    srv.start()
    ok(board.calls[:1] == [(0, 0, 0, 0)], "start() zeroes whatever speed the board held")
    cli = socket.create_connection(("127.0.0.1", srv.port), timeout=2)
    cli.sendall(b'{"action": "velocity", "vx": 0.15, "wz": 0.0}\n')
    ok(wait_for(lambda: board.last == (30, 30, 30, 30)), "a velocity line drives the wheels")
    ok(wait_for(lambda: ch.status()["client"] is not None), "status shows the client")
    cli.sendall(b"not json\n")
    ok(wait_for(lambda: board.last == (0, 0, 0, 0)), "a garbage line stops the wheels")
    cli.sendall(b'{"action": "velocity", "vx": 0.15')
    time.sleep(0.05)
    cli.sendall(b', "wz": 0.0}\n')
    ok(wait_for(lambda: board.last == (30, 30, 30, 30)),
       "the connection survives it, and a line split across packets still parses")
    ok(wait_for(lambda: board.last == (0, 0, 0, 0), timeout=1.5),
       "the watchdog stops a client that goes quiet with the socket open")
    cli.sendall(b'{"action": "velocity", "vx": 0.15, "wz": 0.0}\n')
    wait_for(lambda: board.last == (30, 30, 30, 30))
    cli.close()
    ok(wait_for(lambda: board.last == (0, 0, 0, 0) and ch.status()["client"] is None),
       "a disconnect stops the wheels")
    cli = socket.create_connection(("127.0.0.1", srv.port), timeout=2)
    cli.sendall(b'{"action": "velocity", "vx": 0.15, "wz": 0.0}\n')
    wait_for(lambda: board.last == (30, 30, 30, 30))
    srv.close()
    ok(board.last == (0, 0, 0, 0), "close() zeroes the wheels")
    cli.close()
    ok(raises(lambda: socket.create_connection(("127.0.0.1", srv.port), timeout=0.5)),
       "and stops listening")

    print("\n== R7: the wheels follow the arm pose ==")
    board, ch = make_chassis(to_motor, Clock())
    ch.set_arm_pose("unknown")
    n = len(board.calls)
    ok(ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0}) == "arm_pose"
       and all(c == (0, 0, 0, 0) for c in board.calls[n:]) and ch.pose_blocked == 1,
       "an unknown arm pose refuses velocity and never turns a wheel")
    ch.set_arm_pose("travel")
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    ok(board.last == (30, 30, 30, 30), "the travel pose drives at full speed")
    ch.set_arm_pose("e1")
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    ok(board.last == to_motor(cs.CREEP_VX, 0.0), "at E1 forward speed is capped to the creep")
    ch.command({"action": "velocity", "vx": 0.0, "wz": 1.0})
    ok(board.last == to_motor(0.0, cs.CREEP_WZ), "and so is the turn rate")
    ch.set_arm_pose("other")
    ok(board.last == (0, 0, 0, 0), "an arm left anywhere else stops the wheels at once")

    print("\n== R8: the wheels need the board to be talking ==")
    # The robot, 2026-09-28: a USB hub glitch stopped only the read side. The
    # wheels obeyed, servo reads failed and the wheel feedback froze.
    clock = Clock()
    rx = cs.RxMonitor(clock)
    board = FakeBoard()
    ch = cs.Chassis(board, to_motor, clock=clock, log=lambda _m: None, rx_age=rx.age)
    ch.set_arm_pose("travel")
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    ok(board.last == (30, 30, 30, 30), "a talking board drives")
    clock.advance(0.6)
    ch._last_cmd = clock()                          # a client that keeps sending
    ok(ch.watchdog_tick() and board.last == (0, 0, 0, 0),
       "0.6 s without a byte from the board stops wheels that were turning")
    n = len(board.calls)
    ok(ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0}) == "board_silent"
       and all(c == (0, 0, 0, 0) for c in board.calls[n:]),
       "and velocity is refused while it stays silent")
    rx.saw(12)
    ok(ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0}) == "ok"
       and board.last == (30, 30, 30, 30), "the next byte from the board lets it drive again")

    class ReadSer:
        def __init__(self):
            self.feed = [b"\xff", b"", b"\xfc"]

        def read(self, n=1):
            return self.feed.pop(0) if self.feed else b""

    dev = type("Dev", (), {})()
    dev.ser = ReadSer()
    clock = Clock()
    mon = cs.install_rx_monitor(dev, clock=clock)
    ok(cs.install_rx_monitor(dev) is mon, "installing the monitor twice keeps one")
    clock.advance(2.0)
    dev.ser.read()
    ok(mon.age() == 0.0 and mon.bytes == 1, "a byte read by the driver resets the silence")
    clock.advance(1.0)
    dev.ser.read()
    ok(mon.age() == 1.0, "an empty read does not")

    print("\n== grasp service: R1/R2 around the arm commands ==")
    gs.GRASP_SETTLE_S = 0.0     # the settle wait has its own checks in test_travel_pose.py

    E1 = (90.0, 74.2, 8.6, 8.6, 90.0, 30.0)

    class FakeServo:
        def read_degrees(self):
            return type("R", (), {"valid": True, "degrees": list(E1)})()

    class FakeController:
        detection = None
        servo = FakeServo()
        cfg = type("C", (), {"home_deg": E1})()
        _outcome = "grasped"

    clock = Clock()
    board, ch = make_chassis(to_motor, clock)
    fake_srv = type("S", (), {"chassis": ch})()
    seen = {}

    def episode(controller, max_steps):
        seen["runs"] = seen.get("runs", 0) + 1
        seen["during"] = ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
        return True

    svc = gs.GraspService(FakeController(), episode, 10, "/tmp/unused.sock",
                          chassis_server=fake_srv)
    ch.command({"action": "velocity", "vx": 0.15, "wz": 0.0})
    reply = svc.run_arm_command(svc.cmd_grasp)
    ok(reply.get("reason") == "chassis_moving" and "runs" not in seen and svc.episodes == 0,
       "grasp is refused while driving and the episode never starts")
    ch.command({"action": "stop"})
    clock.advance(0.6)
    reply = svc.run_arm_command(svc.cmd_grasp)
    ok(reply.get("confirmed") is True and seen.get("runs") == 1, "grasp runs once settled")
    ok(seen.get("during") == "dropped", "a velocity command sent mid-grasp was dropped")
    ok(not ch.status()["arm_busy"], "the wheels are released after the grasp")

    def boom():
        raise RuntimeError("servo bus fault")

    ok(raises(lambda: svc.run_arm_command(boom)) and not ch.status()["arm_busy"],
       "an arm command that raises still releases the wheels")
    ok(isinstance(svc.cmd_status().get("chassis"), dict), "status reports the chassis")

    plain = gs.GraspService(FakeController(), episode, 10, "/tmp/unused.sock")
    ok(plain.run_arm_command(lambda: {"ok": True}) == {"ok": True}
       and plain.cmd_status().get("chassis") is None,
       "with the chassis off, arm commands and status are the arm-only service's")
    ok(gs.build_chassis_server(FakeController(), "") is None,
       "no GRASP_SERVICE_CHASSIS_PORT, no chassis")

    print("\n== SIGTERM stops the wheels before the process exits ==")
    if not POSIX:
        skip("systemctl stop while driving")
    else:
        child_src = r'''
import sys, time
sys.path.insert(0, sys.argv[1])
sys.path.insert(0, sys.argv[2])
import chassis_server as cs, grasp_service as gs
from test_chassis_server import FakeBoard
board = FakeBoard()
to_motor, mx, wd = cs.load_motor_mapping(__import__("pathlib").Path(sys.argv[1]).parents[1])
# A 60 s watchdog, so the only thing that can zero the wheels here is SIGTERM.
ch = cs.Chassis(board, to_motor, watchdog_s=60.0, log=lambda m: print(m, flush=True))
srv = cs.ChassisServer(ch, host="127.0.0.1", port=0, log=lambda m: None)
class Servo:
    # serve() asks the encoders where the arm is (R7); say the travel pose.
    def read_degrees(self):
        return type("R", (), {"valid": True, "degrees": list(gs.TRAVEL_DEG)})()
class C:
    detection = None
    servo = Servo()
    cfg = type("Cfg", (), {"home_deg": (90.0, 74.2, 8.6, 8.6, 90.0, 30.0)})()
svc = gs.GraspService(C(), None, 1, sys.argv[3], chassis_server=srv)
gs.install_sigterm_shutdown(svc)
orig = srv.start
def start():
    orig(); print("PORT", srv.port, flush=True)
srv.start = start
try:
    svc.serve()
finally:
    print("LAST", board.last, flush=True)
'''
        with tempfile.TemporaryDirectory() as tmp:
            sock_path = os.path.join(tmp, "svc.sock")
            proc = subprocess.Popen([sys.executable, "-c", child_src, str(HERE), str(HERE),
                                     sock_path], stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, universal_newlines=True)
            # Read the child on a thread with deadlines: a child that never prints
            # what we wait for must fail this check, not hang the whole suite.
            lines = queue.Queue()
            threading.Thread(target=lambda: [lines.put(ln) for ln in proc.stdout],
                             daemon=True).start()
            seen = []

            def wait_line(pred, timeout=15.0):
                deadline = time.time() + timeout
                while time.time() < deadline:
                    try:
                        ln = lines.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    seen.append(ln)
                    if pred(ln):
                        return ln
                return None

            port_line = wait_line(lambda ln: ln.startswith("PORT"))
            driving = False
            if port_line:
                cli = socket.create_connection(("127.0.0.1", int(port_line.split()[1])), timeout=2)
                cli.sendall(b'{"action": "velocity", "vx": 0.15, "wz": 0.0}\n')
                driving = wait_line(lambda ln: "m1=30" in ln) is not None
            ok(driving, "the child service is driving before the signal")
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
            time.sleep(0.3)                         # let the reader drain the pipe
            while not lines.empty():
                seen.append(lines.get())
            out = "".join(seen)
            if port_line:
                cli.close()
            ok(proc.returncode == 0 and "LAST (0, 0, 0, 0)" in out,
               "SIGTERM while driving leaves the wheels at zero (got: {})".format(
                   out.strip().splitlines()[-1] if out.strip() else "no output"))

    print()
    if failures:
        print("{} of {} checks FAILED:".format(len(failures), checks))
        for label in failures:
            print("  - {}".format(label))
        return 1
    print("all {} checks passed — one serial owner, wheels never move with the arm, "
          "and every exit path stops them".format(checks))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
