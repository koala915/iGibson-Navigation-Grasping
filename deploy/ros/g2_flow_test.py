#!/usr/bin/env python3
"""G2 step 4, whole flow with the real poses. Runs on the Jetson (system python3 3.6).

travel pose: drive toward the box -> home (E1) -> creep at E1 until E1 sees the
box -> grasp -> stow (holding) -> reverse (carry) -> release -> stow.

usage: python3 g2_flow_test.py FIRST_M CREEP_MAX_M BACK_M   (e.g. 0.15 0.12 0.25)

Needs the operator at the power switch, the box 25-30 cm ahead, 40 cm clear behind.
"""
import json, math, socket, subprocess, sys, time

FIRST_M, CREEP_MAX_M, BACK_M = [float(v) for v in sys.argv[1:4]]
CTL = ["python3", "/home/jetson/Documents/deploy_jetson2/grasp/v23/graspctl.py"]
T0 = time.time()
marks = {}


def log(msg):
    print("%6.2f %s" % (time.time() - T0, msg), flush=True)


def status():
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect("/tmp/grasp_service.sock")
    s.sendall(b"status\n")
    buf = b""
    while not buf.endswith(b"\n"):
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
    s.close()
    return json.loads(buf.decode())


def pose():
    p = status()["odom"]["pose"]
    return p[0], p[1]


def drive(dist, vx, max_s=8.0, stop_early_m=0.0):
    """Drive to an odometry threshold, then include coasting in the return."""
    x0, y0 = pose()
    c = socket.create_connection(("127.0.0.1", 7000), timeout=3)
    moved, t = 0.0, time.time()
    command_goal = max(0.005, dist - stop_early_m)
    try:
        while moved < command_goal and time.time() - t < max_s:
            c.sendall((json.dumps({"action": "velocity", "vx": vx, "wz": 0.0}) + "\n").encode())
            time.sleep(0.05)
            x, y = pose()
            moved = math.hypot(x - x0, y - y0)
            # Odometry that does not move while the wheels are told to is not a
            # robot standing still; it is feedback that has stopped (2026-09-28).
            if time.time() - t > 1.0 and moved < 0.005:
                raise RuntimeError("commanded for 1 s but odometry did not move")
    finally:
        for _ in range(3):
            c.sendall(b'{"action": "stop"}\n')
            time.sleep(0.02)
        c.close()
    stop_t = time.time()
    time.sleep(0.4)
    x, y = pose()
    return math.hypot(x - x0, y - y0), stop_t


def ctl(cmd):
    t = time.time()
    out = subprocess.run(CTL + [cmd], stdout=subprocess.PIPE, universal_newlines=True).stdout
    try:
        r = json.loads(out)
    except ValueError:
        r = {"raw": out.strip()[:300]}
    return r, time.time() - t


def must(ok, what):
    if not ok:
        log("STOP: " + what)
        sys.exit(2)


st = status()
log("start: arm_pose %s, servo %s, odom %s" % (st["chassis"]["arm_pose"], st["servo_deg"], st["odom"]["pose"]))
must(st["chassis"]["arm_pose"] == "travel", "arm is not in the travel pose")
must(st["servo_deg"] is not None, "servo reads are failing")
rx_age = st["chassis"].get("board_rx_age_s")
must(rx_age is not None and rx_age < 0.5, "the board is not talking")

# The 2026-09-28 run travelled 0.222 m when commanded to stop at 0.15 m.
# Stop commanding early; the guard below still aborts if coast is larger.
first_d, t_stop = drive(FIRST_M, 0.15, stop_early_m=min(0.06, FIRST_M / 2))
log("approach in travel pose: %.3f m" % first_d)
must(first_d <= min(0.18, FIRST_M + 0.03),
     "initial approach exceeded the safe distance; do not creep toward the box")

r, dt = ctl("home")
log("home (travel -> E1 via waypoint) -> ok=%s in %.2f s" % (r.get("ok"), dt))
must(r.get("ok"), "home failed: %s" % r)
t_arm = time.time()

creep = 0.0
while True:
    time.sleep(2.5)                                  # a frame shot after the last motion
    det = status()["detection"]
    if det and det["fresh"]:
        time.sleep(1.0)
        det2 = status()["detection"]
        if det2 and det2["fresh"]:
            log("E1 sees the box: %s" % det2["pos"])
            break
    must(creep + 0.02 + 0.02 <= CREEP_MAX_M, "crept %.3f m at E1 and never saw the box" % creep)
    # The box starts 25-30 cm ahead of the chassis; do not keep creeping when
    # vision is absent and the travelled distance has consumed that clearance.
    must(first_d + creep + 0.03 <= 0.18,
         "forward travel reached the 0.18 m blind-approach limit without a detection")
    d, t_stop = drive(0.005, 0.10)
    creep += d
    log("creep at E1: %.3f m (total %.3f)" % (d, creep))

last_motion = max(t_arm, t_stop)
log("grasp: %.2f s after the last motion" % (time.time() - last_motion))
r, dt = ctl("grasp")
log("grasp -> %s in %.2f s" % (json.dumps(r), dt))
must(r.get("confirmed"), "grasp not confirmed")

r, dt = ctl("stow")
log("stow (holding) -> %s in %.2f s" % (json.dumps(r), dt))
must(r.get("ok"), "stow failed")
st = status()
log("carry: servo %s, arm_pose %s" % (st["servo_deg"], st["chassis"]["arm_pose"]))

d, _ = drive(BACK_M, -0.08, max_s=10.0)
log("carried back %.3f m in the travel pose" % d)

r, dt = ctl("release")
log("release (travel -> E1 holding, then release) -> %s in %.2f s" % (json.dumps(r), dt))
r, dt = ctl("stow")
log("stow (empty) -> ok=%s in %.2f s" % (r.get("ok"), dt))
st = status()
log("end: arm_pose %s, servo %s, odom %s, rss %s MB"
    % (st["chassis"]["arm_pose"], st["servo_deg"], st["odom"]["pose"], st["rss_mb"]))
