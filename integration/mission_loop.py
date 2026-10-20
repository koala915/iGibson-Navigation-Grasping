#!/usr/bin/env python3
"""Competition loop on the Windows PC: patrol -> grasp -> place -> patrol again.

    patrol    ncu's map patrol on the Jetson (deploy/ros/g2_map_patrol.py, PR #5),
              lap after lap, while this PC watches the rear camera for the
              sugarbox, beyond the calibrated strip too (it may stand anywhere)
    grasp     sugarbox_approach_grasp.py --target sugarbox --seed (where the
              patrol saw it), avoidance approach, E1
    place     sugarbox_approach_grasp.py --target placement --turn-to-box: AMCL
              only turns toward the box (its position from ncu's scene layout),
              the camera does the rest
    back out  straight back BACK_OUT_M: the patrol's map mask keeps 0.28 m around
              the placement box and refuses to start inside it

The design was settled with the user on 2026-10-19 (docs/planning/
PATROL_LOOP_DECISIONS_2026-10-19.md). The loop runs until two whole laps find no
sugarbox to fetch. A missed grasp with the arm in a known, empty state backs out
and patrols on (the user, 2026-10-20, option C): 0.20 m, or AT_BOX_BACK_OUT_M when
the approach had reached the box, so the camera sees it again; a spot missed
twice is skipped. A failed placement still
holding the object looks around for the box once and tries again. Anything
leaving the arm's state unknown stops the loop where it is and says why.

One velocity owner at a time: the patrol is stopped with SIGTERM (it brakes and
re-locks the service) before an approach connects, and it is never started while
one runs.

--no-patrol runs one object without the patrol, as on 2026-10-18 when it put the
sugarbox in the placement box twice: grasp the sugarbox in front of the robot,
then carry it to the placement box, which must be in the rear camera's view
after the grasp (finding it from anywhere needs ncu's localization, later).
--resume-at picks a stopped run up where it stopped: grasp-e1 after the arm
camera dropped off USB at the box (replug it first), place once the object is
held. Every stage's output is also saved under logs/.

Default is a read-only preflight. --real needs --i-am-beside-the-robot.

    python integration/mission_loop.py --host <IP> --no-patrol
    python integration/mission_loop.py --host <IP> --no-patrol --real --i-am-beside-the-robot
    python integration/mission_loop.py --host <IP> --real --i-am-beside-the-robot
"""
import argparse
from collections import deque
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import shlex
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from integration import placement_dock as dock
from integration.sugarbox_ground_calibration import (
    inside_calibration_hull, load_ground_calibration, pixel_to_ground, resolve_asset_paths)
from integration.sugarbox_training_contract import GOAL_ORIGIN_OFFSET_X_M
from integration.rear_stream import ARM_CAMERA_REGION

_spec = importlib.util.spec_from_file_location(
    'sugarbox_approach_grasp', ROOT / 'integration/sugarbox_approach_grasp.py')
runner = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(runner)

RUNNER = ROOT / 'integration/sugarbox_approach_grasp.py'
_runtime = None  # owned by one top-level mission, closed on every exit
LOG_ROOT = ROOT / 'logs'
# The E1 camera, on the hub it shares with the motor board; it drops off USB
# after arm moves now and then (twice on 2026-10-18) and may need replugging.
ARM_CAMERA = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0'
JETSON_ROOT = dock.JETSON_ROOT
SCENE = dock.SCENE
PATROL_OUTPUT = '/tmp/mission_patrol.json'
PATROL_LOG = '/tmp/mission_patrol.log'
STREAM_PORT = 5600
# ncu's patrol drives only while this permit says allow (--vision-permit, PR #5,
# 2026-10-19), after it ran into a sugarbox below the LiDAR's scan plane: this PC
# withdraws it on the first sugarbox candidate, confirms with the robot parked,
# and a PC that goes quiet stops the patrol within 1.2 s.
PERMIT_PATH = '/dev/shm/mission_vision_permit.json'
HEALTH_SOCKET = '/tmp/grasp_health.sock'
PERMIT_SCRIPT = ROOT / 'integration/patrol_vision_permit_jetson.py'
# Longest forward pulse of the patrol: ncu's own default. 0.08 s (his first advice
# with the permit) made it 5-7x slower (2026-10-19): each pulse of ~1.2 cm is
# followed by a >= 0.45 s settle. The pulse length does not delay a stop: the patrol
# reads the permit every 0.04 s inside a pulse and stops on the first denial.
PATROL_FORWARD_PULSE_S = 0.6
# Frames in a row without a candidate before a patrol is started: one frame where
# YOLO misses a box right ahead must not set the robot going toward it.
START_CLEAR_FRAMES = 5
# Known sugarboxes the watcher cannot be trusted to see: one a grasp missed with
# the robot at it (backing out 0.20 m leaves it ~0.48 m ahead, inside the
# gripper's blind zone), one the loop gave up on (passed over by the watcher), and
# an unconfirmed candidate that may become close or hidden later. The box is
# below the LiDAR's scan plane, so nothing else stops the patrol:
# one of them in the way ends the mission instead (2026-10-19: the patrol pushed a
# sugarbox and kept going). "In the way", chassis-centre frame: the corridor ahead
# and the turn-in-place circle, each the body (placement_dock.BODY_RECTANGLES:
# y -0.29..+0.14, farthest corner 0.31 m) plus 0.095 m for the box's half size,
# AMCL and the sighting.
GUARD_AHEAD_M = 0.90
GUARD_RIGHT_M = 0.385
GUARD_LEFT_M = 0.30
GUARD_TURN_M = 0.40
# A box the watcher can see stops the patrol itself: only its blind zone needs
# guarding, for a box missed at the robot and for an unconfirmed candidate first
# seen nearer than BLIND_REJECT_M (parked that near, the gripper may have hidden
# it). 2026-10-20 at the robot: a sugarbox 0.60 m from the centre still showed
# above the gripper (rough, conf 0.96), one at 0.73 m was measured.
GUARD_BLIND_M = 0.62
BLIND_REJECT_M = 0.85
# Where a grasp that reached the box leaves it: the approach's stop, ahead of the
# chassis centre (sugarbox_approach_grasp.py --final-x-m default).
AT_BOX_X_M = 0.28
# Backing out after a grasp missed at the box: far enough for the camera to see it
# again (0.28 + 0.45 = 0.73 m, measured there on 2026-10-20), so the next patrol
# frame finds it and the loop tries once more (the user, 2026-10-20: option C).
# A box left nearer or farther (a run stopped on the way in) is backed from to the
# same AT_BOX_SEEN_M, at least BACK_OUT_M and at most 0.60 m; the way back is still
# checked on the map.
AT_BOX_BACK_OUT_M = 0.45
AT_BOX_SEEN_M = AT_BOX_X_M + AT_BOX_BACK_OUT_M
AT_BOX_BACK_MAX_M = 0.60
# Placing a sighting on the map. Each frame arrived at most STREAM_LATENCY_S after
# it was taken; the pose history over that window says how far the robot moved
# and turned meanwhile (beyond it, at the patrol's top speeds). The error bound
# shrinks every exclusion margin, and above SPOT_ERROR_MAX_M nothing is excluded:
# 2026-10-20 review, a frame taken mid-turn placed with the later pose moved a
# real box 0.4 m, into the wall-clip zone.
PATROL_MAX_VX = 0.15
PATROL_MAX_WZ = 1.0
SPOT_ERROR_MAX_M = 0.10
# A far sighting that cannot be placed yet (taken mid-turn) waits this long for a
# frame that can, before it stops the patrol: at 0.15 m/s the robot gains ~13 cm,
# still well short of the blind zone. Nearer ones stop it at once.
DEFER_MIN_M = 1.00
DEFER_S = 0.50
# The robot counts as still once the patrol has been stopped this long: frames are
# then placed from the poses alone, with no allowance for unsampled motion.
STOP_SETTLE_S = 0.60
# A candidate the parked camera did not confirm this near a wall is taken for a
# wall clip (a sugarbox never stands against a wall) and passed over from then on;
# the slack covers the coast between the frame and the stop. A clip passes over
# only sightings inside that same band by the wall: nearer the middle of the field
# a box next to it is still a box (2026-10-20 review).
CLIP_SLACK_M = 0.10
# Without a pose this long the loop gives up (localization lost; re-seeding is
# not built in yet). The patrol's own permit runs stale meanwhile, so it waits.
POSE_LOST_S = 8.0
# Until the Jetson end reports the patrol process, the loop asks over ssh, at
# most this often (one ssh round trip, ~0.5 s, holds the permit renewals back).
PATROL_CHECK_S = 1.0
# The Jetson end reports the patrol process every 0.1 s; a report older than this
# (the link stalled or ended) is not trusted and ssh is asked instead.
PATROL_REPORT_MAX_AGE_S = 0.5
# Rear camera to this PC (encode, Wi-Fi, decode), a generous bound: a frame's
# source stamp is put this much before it arrived, so it is never newer than it is.
STREAM_LATENCY_S = 0.30
# Parked confirmation: frames taken after the stop, for at most CONFIRM_S.
CONFIRM_S = 4.0
POSE_MAX_AGE_S = 0.5
# Candidates the parked camera did not confirm, per lap, before the mission gives
# up: the black wall clips read as sugarboxes at 0.53-0.73 (ncu, 2026-10-19).
MAX_FALSE_STOPS_PER_LAP = 6
# A sugarbox never stands against a wall (the user, 2026-10-19), and the clips
# holding the wall panels read up to 0.73, past WATCH_CONF: a sighting placed on
# the map within this of a wall, or beyond the walls, is passed over.
WALL_EXCLUDE_M = 0.15
# The approach locks a box after 3 consistent frames; the watcher asks the same,
# so it never stops the patrol for a box the approach would not take.
# Below 0.70 a sugarbox sighting is dropped (the user, 2026-10-19): the black wall
# clips read 0.53-0.73. Same threshold as the approach engine's sugarbox target.
WATCH_CONF = 0.70
PLACEMENT_CONF = 0.50
WATCH_IMGSZ = 1280
WATCH_FRAMES = 3
WATCH_MAX_ERROR_M = 0.25
WATCH_MAX_GAP_S = 0.75
# A sighting beyond the calibrated strip is extrapolated, so noisier; the same
# tolerance as the approach's own far seed (FAR_SEED_MAX_ERROR_M).
FAR_WATCH_MAX_ERROR_M = 0.40
STREAM_STALE_S = 1.5
# A sugarbox this close to the placement box's map position is one already
# delivered (it shows over the 6 cm rim), not a new one to fetch: the box's inner
# corner 0.11 m, AMCL ~0.04 m, the rim hiding the sugarbox's foot ~0.07 m, margin.
PLACED_EXCLUDE_M = 0.35
BACK_OUT_M = 0.20
# The patrol refuses to start inside its map mask: back ROOM_STEP_M and ask
# again, at most ROOM_MAX_M since the last stage (the user, 2026-10-19).
ROOM_STEP_M = 0.10
ROOM_MAX_M = 0.30
# Two whole laps with nothing to fetch end the mission (the user, 2026-10-19).
EMPTY_LAPS_TO_END = 2
# Grasping failed this many times within SPOT_RADIUS_M: that spot is left alone.
TRIES_PER_SPOT = 2
SPOT_RADIUS_M = 0.25
# Looking around for the placement box after a failed placement, still holding:
# turns of SEARCH_STEP_RAD, SEARCH_LOOK_S of frames between them, about a circle.
SEARCH_STEP_RAD = math.radians(26)
SEARCH_STEPS = 14
SEARCH_LOOK_S = 2.5
PLACEMENT_IMGSZ = 640


class MissionError(RuntimeError):
    pass


def ssh(host, command, timeout=30):
    return subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                           'jetson@' + host, command],
                          capture_output=True, text=True, timeout=timeout)


def pc_address(host):
    """This PC's address on the robot's network (no packet is sent)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((host, STREAM_PORT))
        return s.getsockname()[0]


def patrol_command(scene=SCENE, output=PATROL_OUTPUT, log=PATROL_LOG):
    """Remote shell line: ncu's patrol-cycle exactly as his delivery navigation
    starts it (native ROS Python), detached, printing its PID."""
    inner = ('source /opt/ros/melodic/setup.bash; '
             'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1; '
             'exec /usr/bin/python deploy/ros/g2_map_patrol.py --scene {} '
             '--destination patrol-cycle --inset .70 --boundary-micro --keep-heading '
             '--vision-permit {} --forward-pulse-max {} '
             '--seconds 600 --output {} --real').format(
                 shlex.quote(scene), PERMIT_PATH, PATROL_FORWARD_PULSE_S, shlex.quote(output))
    # The braces keep `&` on nohup alone: backgrounding the whole `cd && ...` list
    # would make $! a subshell's PID (SIGTERM would miss the patrol) and that
    # subshell would hold the ssh session open.
    return ('cd {} && rm -f {} && {{ nohup /bin/bash -c {} > {} 2>&1 < /dev/null & echo $!; }}'
            .format(JETSON_ROOT, shlex.quote(output), shlex.quote(inner), shlex.quote(log)))


def start_patrol(host, scene):
    result = ssh(host, patrol_command(scene))
    try:
        return int(result.stdout.strip().splitlines()[-1])
    except (IndexError, ValueError):
        raise MissionError('patrol did not start: ' + result.stdout + result.stderr)


def patrol_running(host, pid):
    return ssh(host, 'kill -0 {}'.format(int(pid))).returncode == 0


def patrol_still_running(host, pid, link=None):
    """From `link`'s Jetson end when it has a fresh word on this pid, else over ssh.

    2026-10-21: every stop asked `kill -0` over a new ssh connection each 0.3 s,
    ~0.5-1 s apiece on Windows, while the Jetson end already reported it.
    """
    alive = link.patrol_alive(pid) if link is not None else None
    return patrol_running(host, pid) if alive is None else alive


def stop_patrol(host, pid, timeout=15.0, link=None):
    """SIGTERM: the patrol stops the wheels, re-locks the service, writes its report."""
    ssh(host, 'kill -TERM {}'.format(int(pid)))
    deadline = time.monotonic() + timeout
    while patrol_still_running(host, pid, link):
        if time.monotonic() > deadline:
            raise MissionError('patrol {} did not exit after SIGTERM'.format(pid))
        time.sleep(0.1)


def patrol_report(host):
    result = ssh(host, 'cat ' + PATROL_OUTPUT)
    try:
        return json.loads(result.stdout)
    except ValueError:
        return {'ok': False, 'error': 'no patrol report'}


def patrol_supports_permit(host):
    """The robot's patrol takes --vision-permit (ncu's PR #5 e359fb5 and later)."""
    return ssh(host, 'grep -q -- --vision-permit {}/deploy/ros/g2_map_patrol.py'.format(
        JETSON_ROOT)).returncode == 0


class VisionLink:
    """The patrol's vision permit and the robot's map pose, over one ssh session.

    send() once per processed frame; the Jetson end (patrol_vision_permit_jetson.py)
    stamps it on its own clock and writes the permit. Closing the link, or this PC
    going quiet, stops the patrol.
    """

    def __init__(self, host, popen=subprocess.Popen, clock=time.monotonic):
        command = 'python3 -u -c {} {} {} {}'.format(
            shlex.quote(PERMIT_SCRIPT.read_text(encoding='utf-8')), PERMIT_PATH,
            dock.LOCALIZATION_HEALTH, HEALTH_SOCKET)
        self.clock, self.allowed = clock, False
        self.pose, self.pose_at = None, None
        self.pose_history = deque(maxlen=100)
        self.patrol = None                      # (pid, alive) as the Jetson last saw it
        self.patrol_at = None                   # when that word arrived (self.clock)
        self.lock = threading.Lock()
        self.proc = popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                           'jetson@' + host, command],
                          stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            try:
                record = json.loads(line)
                patrol = record.get('patrol')
                if isinstance(patrol, dict) and isinstance(patrol.get('alive'), bool):
                    with self.lock:
                        self.patrol = (patrol.get('pid'), patrol['alive'])
                        self.patrol_at = self.clock()
                pose = [float(v) for v in record['pose']]
                age = float(record['t']) - float(record['stamp'])
                if (record.get('ready') is not True or len(pose) != 3
                        or not all(math.isfinite(v) for v in pose)
                        or not 0 <= age <= min(dock.LOCALIZATION_MAX_AGE_S, POSE_MAX_AGE_S)):
                    with self.lock:
                        self.pose, self.pose_at = None, None
                        self.pose_history.clear()
                    continue
            except (ValueError, KeyError, TypeError, AttributeError):
                with self.lock:
                    self.pose, self.pose_at = None, None
                    self.pose_history.clear()
                continue
            with self.lock:
                self.pose, self.pose_at = tuple(pose), self.clock() - age
                self.pose_history.append((self.pose_at, self.pose))

    def send(self, allow, age):
        """`age`: seconds from the frame's capture to now (a generous bound)."""
        try:
            self.proc.stdin.write(json.dumps({'allow': bool(allow),
                                              'age': round(max(0.0, age), 3)}) + '\n')
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            raise MissionError('vision permit link to the robot lost: {}'.format(exc))
        self.allowed = bool(allow)

    def pose_now(self):
        with self.lock:
            if self.pose_at is None or self.clock() - self.pose_at > POSE_MAX_AGE_S:
                return None
            return self.pose

    def watch(self, pid):
        """Have the Jetson end report whether this patrol process still runs."""
        try:
            self.proc.stdin.write(json.dumps({'watch': int(pid)}) + '\n')
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            raise MissionError('vision permit link to the robot lost: {}'.format(exc))

    def frame_pose(self, stamp, moving=True):
        """(pose, moved_m, turned_rad) for a frame that arrived at `stamp`.

        The RTP stream carries no capture time, so the frame was taken somewhere in
        [stamp - STREAM_LATENCY_S, stamp]. `pose` is the sample nearest the middle
        of that window; moved/turned bound how far the robot went within it, from
        the samples, and at the patrol's top speeds over any part they do not
        cover (none with the robot known still: `moving` false). None for a stale
        or future frame, without a fresh pose, or with a gap in the samples.
        """
        now = self.clock()
        if not math.isfinite(stamp) or stamp > now or now - stamp > STREAM_STALE_S:
            return None
        with self.lock:
            if (self.pose_at is None or now - self.pose_at > POSE_MAX_AGE_S
                    or not self.pose_history):
                return None
            history = list(self.pose_history)
        start, end = stamp - STREAM_LATENCY_S, stamp
        before = [i for i, (at, _) in enumerate(history) if at <= start]
        after = [i for i, (at, _) in enumerate(history) if at >= end]
        span = history[before[-1] if before else 0:(after[0] + 1) if after else len(history)]
        if not span or span[0][0] > end + POSE_MAX_AGE_S or span[-1][0] < start - POSE_MAX_AGE_S:
            return None
        for (at0, _), (at1, _) in zip(span, span[1:]):
            if not 0 <= at1 - at0 <= POSE_MAX_AGE_S:
                return None
        middle = (start + end) / 2.0
        pose = min(span, key=lambda sample: abs(sample[0] - middle))[1]
        moved = max(math.hypot(p[0] - pose[0], p[1] - pose[1]) for _, p in span)
        turned = max(abs(dock.wrap(p[2] - pose[2])) for _, p in span)
        uncovered = (max(0.0, span[0][0] - start) + max(0.0, end - span[-1][0])
                     if moving else 0.0)
        return pose, moved + PATROL_MAX_VX * uncovered, turned + PATROL_MAX_WZ * uncovered

    def patrol_alive(self, pid):
        """True/False for the watched patrol, None while the Jetson has not said
        or its last word is older than PATROL_REPORT_MAX_AGE_S."""
        with self.lock:
            if (self.patrol is None or self.patrol[0] != pid
                    or self.clock() - self.patrol_at > PATROL_REPORT_MAX_AGE_S):
                return None
            return self.patrol[1]

    def alive(self):
        return self.proc.poll() is None

    def close(self):
        try:
            self.proc.stdin.close()          # the Jetson end writes allow false on EOF
        except (OSError, ValueError):
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def patrol_refused(report):
    """The patrol declined to start where the robot stands (outside its map mask,
    e.g. next to the placement box) and never moved."""
    error = str(report.get('error') or '')
    return (report.get('ok') is not True and not report.get('pulses')
            and ('navigation mask' in error or 'no reachable entry' in error))


def stream_running(host, pc_ip, port=STREAM_PORT):
    # The [u] keeps pgrep from matching this very command line.
    pattern = '[u]dpsink host={} port={}'.format(pc_ip, port)
    return ssh(host, 'pgrep -f {}'.format(shlex.quote(pattern))).returncode == 0


def start_stream(host, pc_ip, port=STREAM_PORT):
    ssh(host, 'cd {} && {{ nohup bash deploy/ros/start_rear_approach_stream.sh {} {} '
              '> /tmp/rear_stream.log 2>&1 < /dev/null & }}'.format(JETSON_ROOT, pc_ip, int(port)))
    time.sleep(2.0)
    if not stream_running(host, pc_ip, port):
        log = ssh(host, 'tail -3 /tmp/rear_stream.log').stdout.strip()
        raise MissionError('rear camera stream did not start: ' + log)


def overlap_fraction(bbox, region):
    x1, y1, x2, y2 = bbox
    rx1, ry1, rx2, ry2 = region
    w = max(0.0, min(x2, rx2) - max(x1, rx1))
    h = max(0.0, min(y2, ry2) - max(y1, ry1))
    area = max(1.0, (x2 - x1) * (y2 - y1))
    return w * h / area


def detection_point(bbox, homography, rear_cam_to_front_m, hull):
    """Box bottom-centre -> chassis-centre frame, as the approach measures it
    (it uses the SAM mask's lowest point; the bbox bottom is close enough to
    decide whether to stop the patrol). None outside the calibrated area."""
    x1, y1, x2, y2 = bbox
    u, v = (x1 + x2) / 2.0, y2
    if not inside_calibration_hull(u, v, hull):
        return None
    ground = pixel_to_ground(u, v, homography)
    if ground is None:
        return None
    return ground[0] - rear_cam_to_front_m + GOAL_ORIGIN_OFFSET_X_M, ground[1]


def in_seed_range(point):
    x, y = point
    return (runner.SEED_MIN_M <= math.hypot(x, y) <= runner.SEED_MAX_M
            and abs(math.degrees(math.atan2(y, x))) <= runner.SEED_MAX_BEARING_DEG)


def far_point(bbox, homography, rear_cam_to_front_m):
    """Extrapolated floor point of a box beyond the calibrated area, centre frame.

    Tens of cm off at times (the engine's coarse_ground): enough to stop the
    patrol and drive toward, which is all it is used for. None outside the
    approach's seed range.
    """
    x1, y1, x2, y2 = bbox
    ground = pixel_to_ground((x1 + x2) / 2.0, y2, homography)
    if ground is None:
        return None
    point = ground[0] - rear_cam_to_front_m + GOAL_ORIGIN_OFFSET_X_M, ground[1]
    return point if in_seed_range(point) else None


def map_point(pose, point):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


class Confirm:
    """WATCH_FRAMES consistent sightings in a row, like the approach's lock."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.points, self.stamp, self.far = [], None, False

    def update(self, point, now, far=False):
        if point is None:
            if self.stamp is not None and now - self.stamp > WATCH_MAX_GAP_S:
                self.reset()
            return False
        tolerance = FAR_WATCH_MAX_ERROR_M if far or self.far else WATCH_MAX_ERROR_M
        if self.points and (now - self.stamp > WATCH_MAX_GAP_S or math.hypot(
                point[0] - self.points[-1][0], point[1] - self.points[-1][1]) > tolerance):
            self.points, self.far = [], False
        self.points.append(point)
        self.far = self.far or far
        self.stamp = now
        return len(self.points) >= WATCH_FRAMES

    def mean(self):
        return (sum(p[0] for p in self.points) / len(self.points),
                sum(p[1] for p in self.points) / len(self.points))


class BoxWatcher:
    """Rear-camera YOLO for one kind of box: the nearest calibrated sighting, else
    the nearest one beyond the calibrated area."""

    def __init__(self, model_path, homography_path, imgsz, conf):
        self.runtime, self.model_path = _runtime, model_path
        self.imgsz, self.conf = imgsz, conf
        if self.runtime is None:
            from ultralytics import YOLO
            self.model = YOLO(str(model_path))
        self.H, self.rear_cam_to_front, self.hull = load_ground_calibration(homography_path)

    def look(self, frame):
        """(point, far) in the chassis-centre frame, or None with no box in view.

        The arm camera's corner of the frame is dropped after detection, so a
        picture of raw YOLO results still shows it boxed.
        """
        if self.runtime is None:
            result = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
            boxes = [box.xyxy[0].tolist() for box in result.boxes]
        else:
            boxes = self.runtime.detect(self.model_path, frame, self.imgsz, self.conf)
        near = far = None
        for box in boxes:
            bbox = tuple(float(v) for v in box)
            if overlap_fraction(bbox, ARM_CAMERA_REGION) >= 0.5:
                continue
            point = detection_point(bbox, self.H, self.rear_cam_to_front, self.hull)
            if point is not None:
                if near is None or math.hypot(*point) < math.hypot(*near):
                    near = point
                continue
            point = far_point(bbox, self.H, self.rear_cam_to_front)
            if point is not None and (far is None or math.hypot(*point) < math.hypot(*far)):
                far = point
        if near is not None:
            return near, False
        return (far, True) if far is not None else None


class SugarboxWatcher(BoxWatcher):
    def __init__(self, homography_path):
        super().__init__(runner.SUGARBOX_MODEL, homography_path, WATCH_IMGSZ, WATCH_CONF)


class PlacementWatcher(BoxWatcher):
    def __init__(self, homography_path):
        super().__init__(runner.PLACEMENT_MODEL, homography_path, PLACEMENT_IMGSZ,
                         PLACEMENT_CONF)


def fresh_frame(reader, last_stamp, now):
    """(frame, stamp) when a new frame is in, (None, last_stamp) while waiting."""
    frame, stamp = reader.latest()
    if now() - stamp > STREAM_STALE_S:
        raise MissionError('rear camera stream lost')
    if stamp == last_stamp:
        time.sleep(0.01)
        return None, last_stamp
    return frame, stamp


def wait_first_frame(reader, now, why, limit=8.0):
    """A receiver just started has no frame for a second or two (start-up and the
    next keyframe), which fresh_frame() would take for a lost stream."""
    deadline = now() + limit
    while reader.latest()[0] is None:
        if now() > deadline:
            raise MissionError(why)
        time.sleep(0.1)


def sighting_on_map(link, point):
    pose = link.pose_now() if point is not None else None
    return map_point(pose, point) if pose is not None else None


def sighting_spot(link, stamp, point, moving=True):
    """(map spot, error_m) for a sighting in the frame that arrived at `stamp`, or
    None when no pose covers that frame. The error grows with the distance for
    whatever the robot turned while the frame may have been taken."""
    found = link.frame_pose(stamp, moving)
    if found is None:
        return None
    pose, moved, turned = found
    return map_point(pose, point), moved + math.hypot(*point) * turned


def passed_over(where, box, skip, clips, walls=None, error=0.0):
    """A sighting not to stop for: against a wall (a wall clip, not a sugarbox), at
    the placement box (delivered), at a spot where grasping failed twice, or at a
    clip the parked camera already did not confirm, in the clip's band by the wall.
    Each test holds even with the spot `error` metres off; without a map position,
    none."""
    if where is None or error > SPOT_ERROR_MAX_M:
        return False
    near = lambda spots, radius: any(
        math.hypot(where[0] - x, where[1] - y) <= radius - error for x, y in spots)
    by_wall = walls is not None and dock.wall_distance(where, walls) <= (
        WALL_EXCLUDE_M + CLIP_SLACK_M - error)
    return ((walls is not None and dock.wall_distance(where, walls) <= WALL_EXCLUDE_M - error)
            or (box is not None and near([(box['x'], box['y'])], PLACED_EXCLUDE_M))
            or near(skip, SPOT_RADIUS_M) or (by_wall and near(clips, SPOT_RADIUS_M)))


def box_in_the_way(pose, spots, ahead=GUARD_AHEAD_M):
    """(spot, x, y) of the first box spot the robot would hit driving up to `ahead`
    or turning in place (x, y: chassis-centre frame), else None."""
    for spot in spots:
        x, y = dock.relative_point(pose, spot[:2])
        if ((0.0 < x <= ahead and -GUARD_RIGHT_M <= y <= GUARD_LEFT_M)
                or math.hypot(x, y) <= GUARD_TURN_M):
            return tuple(spot[:2]), x, y
    return None


def confirm_parked(reader, watcher, link, now):
    """(point, far) once WATCH_FRAMES frames taken after the stop agree, else None.

    The permit stays withdrawn throughout, so the patrol cannot start again.
    """
    confirm, last_stamp = Confirm(), None
    since = now() + STREAM_LATENCY_S          # older frames may predate the stop
    deadline = now() + CONFIRM_S
    while now() < deadline:
        frame, last_stamp = fresh_frame(reader, last_stamp, now)
        if frame is None:
            continue
        link.send(False, now() - last_stamp + STREAM_LATENCY_S)
        if last_stamp < since:
            continue
        sighting = watcher.look(frame)
        point, far = sighting if sighting is not None else (None, False)
        if confirm.update(point, last_stamp, far):
            return confirm.mean(), confirm.far
    return None


def patrol_until_sugarbox(host, scene, reader, watcher, link, box=None, skip=(), guard=(),
                          blind=(), make_room=None, walls=None, now=time.monotonic):
    """Patrol lap after lap until a sugarbox worth fetching is confirmed.

    The patrol drives only while `link` permits it, renewed by every frame
    without a candidate; it starts after START_CLEAR_FRAMES of them, with a pose.
    The first frame with one withdraws the permit, so the patrol stops itself, and
    the patrol is ended; the candidate counts once WATCH_FRAMES frames taken with
    the robot parked agree. Passed over: a sugarbox at the placement box
    (delivered), one at a spot in `skip`, one against a wall of the `walls`
    polygon (map frame) and a clip the parked camera did not confirm, each only
    where the frame's pose places it surely enough. A far sighting that cannot be
    placed yet waits up to DEFER_S for a frame that can.

    Known boxes the watcher will not stop for end the mission when in the robot's
    way: `guard` (given up on: passed over) anywhere up to GUARD_AHEAD_M, `blind`
    (missed at the robot) and unconfirmed candidates first seen nearer than
    BLIND_REJECT_M within the gripper's blind zone. A patrol that refuses to start
    where the robot stands gets make_room() and another try.

    Returns {'point': chassis-centre (x, y), 'map': its map position or None},
    or None after EMPTY_LAPS_TO_END whole laps without one.
    """
    pid, empty_laps, last_stamp, clips, false_stops = None, 0, None, [], 0
    clear, unseen, no_pose_since, checked, unplaced_since = 0, [], None, None, None
    stopped_at, deferred_clear, far_cleared, last_deferred = None, 0, [], None
    wait_first_frame(reader, now, 'no rear camera frames; is the stream reaching this PC?')
    deadline = now() + POSE_LOST_S
    while link.pose_now() is None:             # nor unlocalized: sightings need the map
        if now() > deadline:
            raise MissionError('no localized pose from the robot; the patrol needs one')
        time.sleep(0.1)
    try:
        while True:
            if not link.alive():
                raise MissionError('vision permit link to the robot ended')
            if pid is not None:
                alive = link.patrol_alive(pid)
                if alive is None and now() - checked >= PATROL_CHECK_S:
                    checked = now()            # the Jetson end has not said yet
                    alive = patrol_running(host, pid)
                if alive is False:
                    report, pid, stopped_at = patrol_report(host), None, now()
                    if report.get('ok'):
                        empty_laps += 1
                        false_stops = 0
                        if empty_laps >= EMPTY_LAPS_TO_END:
                            return None
                    elif patrol_refused(report) and make_room is not None and make_room():
                        print('[PATROL] refused to start here ({}); backed off, asking again'
                              .format(report.get('error')), flush=True)
                    else:
                        raise MissionError('patrol stopped: ' + str(report.get('error')))
            frame, last_stamp = fresh_frame(reader, last_stamp, now)
            if frame is None:
                continue
            sighting = watcher.look(frame)
            point, far = sighting if sighting is not None else (None, False)
            pose = link.pose_now()
            if pose is None:
                no_pose_since = now() if no_pose_since is None else no_pose_since
                if now() - no_pose_since > POSE_LOST_S:
                    raise MissionError('no localized pose from the robot for {:.0f} s: '
                                       'localization lost (re-seeding it is not built in yet)'
                                       .format(POSE_LOST_S))
            else:
                no_pose_since = None
            in_way = None
            if pose is not None:
                in_way = (box_in_the_way(pose, guard)
                          or box_in_the_way(pose, list(blind) + unseen, GUARD_BLIND_M))
            if in_way is not None:
                link.send(False, now() - last_stamp + STREAM_LATENCY_S)
                if pid is not None:
                    stop_patrol(host, pid, link=link)
                    pid, stopped_at = None, now()
                (sx, sy), x, y = in_way
                raise MissionError(
                    'a sugarbox the camera cannot vouch for, at map ({:.2f}, {:.2f}), is in the '
                    'patrol\'s way ({:.2f} m ahead, {:+.2f} m left of the centre): below the '
                    'LiDAR and hidden by the gripper, nothing would stop the robot at it. '
                    'Take it away, or set it where the camera sees it, and start again'.format(
                        sx, sy, x, y))
            moving = pid is not None or (stopped_at is not None
                                         and now() - stopped_at < STOP_SETTLE_S)
            spot = (sighting_spot(link, last_stamp, point, moving)
                    if point is not None else None)
            where, error = spot if spot is not None else (None, 0.0)
            placed = where is not None and error <= SPOT_ERROR_MAX_M
            if point is not None and placed and (
                    passed_over(where, box, skip, clips, walls, error) if not far
                    else any(math.hypot(where[0] - x, where[1] - y)
                             <= FAR_WATCH_MAX_ERROR_M - error for x, y in far_cleared)
                    or (walls is not None and dock.wall_distance(where, walls)
                        <= -(FAR_WATCH_MAX_ERROR_M + error))):
                # A far sighting is extrapolated, tens of cm off: only a far spot
                # the parked camera already cleared, or one surely beyond the walls,
                # passes it over (a given-up box in the way is `guard`'s, above).
                point = None
                unplaced_since, deferred_clear = None, 0      # resolved: passed over
            if point is not None and placed:
                unplaced_since, deferred_clear = None, 0      # resolved: a candidate
            elif pid is not None and point is not None and math.hypot(*point) >= DEFER_MIN_M:
                # Taken mid-turn and far: wait, without renewing the permit, for a
                # frame that can place it, a short way on.
                if unplaced_since is None:
                    unplaced_since = now()
                last_deferred, deferred_clear = point, 0
            elif unplaced_since is not None and point is None:
                deferred_clear += 1                            # gone for good?
                if deferred_clear >= START_CLEAR_FRAMES:
                    unplaced_since, deferred_clear = None, 0
            if unplaced_since is not None:
                if now() - unplaced_since <= DEFER_S:
                    clear = 0
                    continue
                if point is None:                              # ran out: stop for it
                    point, far = last_deferred, True
                unplaced_since, deferred_clear = None, 0
            if point is None and pose is None:
                # No permit without a pose to place sightings on the map: the
                # patrol's own permit runs stale and it waits.
                clear = 0
                continue
            link.send(point is None, now() - last_stamp + STREAM_LATENCY_S)
            if point is None:
                clear += 1
                if pid is None and clear >= START_CLEAR_FRAMES:
                    pid = start_patrol(host, scene)
                    link.watch(pid)
                    checked = now()
                    print('[PATROL] lap {} started (pid {})'.format(empty_laps + 1, pid),
                          flush=True)
                continue
            clear = 0
            if pid is not None:
                stop_patrol(host, pid, link=link)   # it stopped itself on the permit already
                pid, stopped_at = None, now()
            print('[PATROL] candidate {:.2f} m ahead, {:+.2f} m left{}: parked to confirm'
                  .format(point[0], point[1], ' (beyond the calibrated area, rough)' if far
                          else ''), flush=True)
            confirmed = confirm_parked(reader, watcher, link, now)
            if confirmed is None:
                false_stops += 1
                # Placed from the parked pose when the frame could not be: off by
                # the coast since, which CLIP_SLACK_M allows for.
                spot = where if placed else sighting_on_map(link, point)
                if spot is None:
                    raise MissionError('an unconfirmed candidate cannot be placed on the map '
                                       '(no localized pose); remaining stopped')
                if far:
                    print('[PATROL] not confirmed with the robot parked (a rough far '
                          'sighting: not remembered)', flush=True)
                elif walls is not None and dock.wall_distance(spot, walls) <= (
                        WALL_EXCLUDE_M + CLIP_SLACK_M):
                    clips.append(spot)
                    print('[PATROL] not confirmed with the robot parked, by a wall: a clip, '
                          'passed over', flush=True)
                else:
                    if math.hypot(*point) < BLIND_REJECT_M:
                        unseen.append(spot)
                    print('[PATROL] not confirmed with the robot parked', flush=True)
                if false_stops > MAX_FALSE_STOPS_PER_LAP:
                    raise MissionError('{} unconfirmed sugarbox candidates in one lap: the '
                                       'camera keeps seeing something that is not one'
                                       .format(false_stops))
                continue
            seen, far = confirmed
            where = sighting_on_map(link, seen)
            if passed_over(where, box, skip, clips, walls):
                # Seen again while driving, it is passed over rather than parked for.
                if far:
                    far_cleared.append(where)
                elif walls is not None and dock.wall_distance(where, walls) <= WALL_EXCLUDE_M:
                    clips.append(where)
                continue
            print('[PATROL] sugarbox {:.2f} m ahead, {:+.2f} m left{}: confirmed'.format(
                seen[0], seen[1], ' (beyond the calibrated area, rough)' if far else ''),
                flush=True)
            return {'point': seen, 'map': where}
    finally:
        if pid is not None and patrol_still_running(host, pid, link):
            stop_patrol(host, pid, link=link)


def tried_out(failures):
    """Map spots where grasping failed TRIES_PER_SPOT times (within SPOT_RADIUS_M)."""
    return [p for p in failures
            if sum(math.hypot(p[0] - q[0], p[1] - q[1]) <= SPOT_RADIUS_M
                   for q in failures) >= TRIES_PER_SPOT]


def approach_seed(host, sighting):
    """Where the sighted sugarbox is now, chassis-centre frame, for --seed.

    The patrol coasts, and may be turning, when it is stopped, so the sighting's
    map position is projected back from the pose now; without a pose the sighting
    itself stands. None outside the approach's seed range.
    """
    point = sighting['point']
    if sighting.get('map') is not None:
        try:
            point = dock.relative_point(dock.localization_pose(dock.read_localization(host)),
                                        sighting['map'])
        except (ValueError, OSError, subprocess.SubprocessError) as exc:
            print('[MISSION] seeding the approach from the raw sighting ({})'.format(exc),
                  flush=True)
    return tuple(point) if in_seed_range(point) else None


def search_placement(host, reader, watcher, turn=None, now=time.monotonic):
    """Turn in place SEARCH_STEP_RAD at a time, looking SEARCH_LOOK_S in between,
    until the camera confirms the placement box (calibrated or not); at most
    SEARCH_STEPS turns, about a full circle. Arm in travel. True when found."""
    turn = turn or runner.turn
    confirm, last_stamp = Confirm(), None
    wait_first_frame(reader, now, 'no rear camera frames to look for the placement box '
                                  'with; still holding')
    for step in range(SEARCH_STEPS + 1):
        if step:
            runner.turn_if_clear(host, SEARCH_STEP_RAD, rotate=turn)
        confirm.reset()
        deadline = now() + SEARCH_LOOK_S
        while now() < deadline:
            frame, last_stamp = fresh_frame(reader, last_stamp, now)
            if frame is None:
                continue
            sighting = watcher.look(frame)
            point, far = sighting if sighting is not None else (None, False)
            if confirm.update(point, last_stamp, far):
                print('[SEARCH] placement box at about ({:.2f}, {:+.2f}) m after {} turn(s)'
                      .format(*confirm.mean(), step), flush=True)
                return True
    return False


def run_stage(host, argv, log_path=None):
    """The approach runner as its own process: the one velocity owner until it exits.

    With log_path its output goes to the terminal and to that file as it comes.
    """
    if _runtime is not None:
        if host != _runtime.host:
            raise MissionError('resident runtime belongs to another robot')
        return _runtime.run_stage(runner, argv, log_path or _runtime.log_dir / 'stage.log')
    command = [sys.executable, '-u', str(RUNNER), '--host', host] + argv
    print('[MISSION] ' + ' '.join(command[2:]), flush=True)
    if log_path is None:
        return subprocess.run(command).returncode
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    with open(log_path, 'w', encoding='utf-8') as log, subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
            text=True, encoding='utf-8', errors='replace', bufsize=1) as proc:
        for line in proc.stdout:
            sys.stdout.write(line)
            log.write(line)
        return proc.wait()


def stage_options(args):
    """The same calibration and decoder for all stages, retries and resumes."""
    options = []
    for option, name in (('--homography', 'homography'), ('--gstreamer', 'gstreamer')):
        value = getattr(args, name, None)
        if value is not None:
            options += [option, str(Path(value).resolve())]
    return options


def arm_camera_state(host):
    """'ok', 'service_down' (the bridge gave up, camera present) or 'unplugged'."""
    words = ssh(host, 'test -e {} && echo camera-present; systemctl is-active grasp-vision'
                .format(shlex.quote(ARM_CAMERA))).stdout.split()
    if 'camera-present' not in words:
        return 'unplugged'
    return 'ok' if 'active' in words else 'service_down'


def ensure_arm_camera(host, fix, look=runner.look, wait_s=25.0):
    """None when E1 reports fresh frames, else why not. With fix, a bridge that
    gave up (it tries the camera three times, then exits) is restarted."""
    state = arm_camera_state(host)
    if state == 'unplugged':
        return 'the arm camera is off the USB bus: replug its cable'

    def fresh():
        record = look(host)
        return record is not None and 0 <= record.get('age', -1) <= runner.FRAME_MAX_AGE_S

    if state == 'ok' and fresh():
        return None
    if not fix:
        return 'the arm-camera bridge is not reporting frames (--real restarts it)'
    print('[MISSION] restarting the arm-camera bridge (grasp-vision)', flush=True)
    ssh(host, 'sudo -n systemctl reset-failed grasp-vision; sudo -n systemctl restart grasp-vision',
        timeout=60)
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        time.sleep(2.0)
        if arm_camera_state(host) == 'ok' and fresh():
            return None
    return 'the arm-camera bridge did not come back: ' + arm_camera_state(host)


class Room:
    """Straight back-outs so the patrol may start again (the user, 2026-10-19).

    leave() after a delivery or a missed grasp backs BACK_OUT_M. While the patrol
    still refuses to start, make() backs ROOM_STEP_M more, up to ROOM_MAX_M in all
    since that stage; the runner checks the way back against the room's walls on
    the map before every step (nothing sees behind the robot). With
    no stage behind it (the mission's start), make() never backs: a robot set
    down in the keep-out goes back into the field by hand, not blindly backward
    (ncu, 2026-10-19: the way out was forward there).
    """

    def __init__(self, host, walls=None, back=None, obstacles=()):
        self.host, self.walls, self.back, self.backed = host, walls, back, None
        self.obstacles = list(obstacles)

    def _back(self, metres):
        if self.back is not None:
            self.back(self.host, metres)
        else:
            runner.back_straight(self.host, metres, self.walls, obstacles=self.obstacles)

    def leave(self, metres=BACK_OUT_M):
        self._back(metres)
        self.backed = metres

    def make(self):
        if self.backed is None or self.backed + ROOM_STEP_M > ROOM_MAX_M + 1e-9:
            return False
        self._back(ROOM_STEP_M)
        self.backed += ROOM_STEP_M
        return True

    def reset(self):
        self.backed = None


def arm_state(host):
    """'holding', 'empty' or 'unknown': anything the service cannot vouch for."""
    try:
        chassis = runner.ctl(host, 'status').get('chassis', {})
        holding = runner.ctl(host, 'health').get('holding')
    except Exception as exc:
        print('[MISSION] arm state unreadable: {}'.format(exc), flush=True)
        return 'unknown'
    if (chassis.get('arm_busy') is not False or chassis.get('moving') is not False
            or chassis.get('arm_pose') not in ('travel', 'e1') or not isinstance(holding, bool)):
        return 'unknown'
    return 'holding' if holding else 'empty'


def to_travel(host):
    if runner.ctl(host, 'status').get('chassis', {}).get('arm_pose') != 'travel':
        runner.ctl(host, 'stow')
        if runner.ctl(host, 'status').get('chassis', {}).get('arm_pose') != 'travel':
            raise MissionError('travel pose not confirmed; wheels not moved')


def unlock(host):
    """ncu's navigation locks the wheels and arm when it ends; the approach needs them."""
    if runner.ctl(host, 'status').get('chassis', {}).get('maintenance') is True:
        runner.service_command(host, 'maintenance-unlock')


def preflight(args):
    issues = []
    resume = getattr(args, 'resume_at', None)
    holding_wanted = resume == 'place'
    poses = ('travel', 'e1') if resume == 'grasp-e1' else ('travel',)
    try:
        status = runner.ctl(args.host, 'status')
        chassis = status.get('chassis', {})
        if chassis.get('arm_pose') not in poses or chassis.get('moving') is not False:
            issues.append('arm not in {} or chassis moving'.format('/'.join(poses)))
        health = runner.ctl(args.host, 'health')
        holding = health.get('holding')
        problem = runner.service_protocol_problem(status, health)
        if problem:
            issues.append(problem)
        if holding_wanted and holding is not True:
            issues.append('nothing is held: --resume-at place needs the object in the jaw')
        elif not holding_wanted and holding is not False:
            issues.append('the jaw holds something; release it first')
    except Exception as exc:
        issues.append('robot: ' + str(exc))
    if not args.no_patrol:
        try:
            dock.localization_pose(dock.read_localization(args.host))
        except (ValueError, OSError, subprocess.SubprocessError) as exc:
            issues.append('localization: ' + str(exc))
        try:
            box = dock.fetch_box(args.host, args.scene)
            if not box['verified']:
                print('[WARN] the placement box position in ncu\'s scene layout is not '
                      'verified: the robot turns toward it but never drives at it unseen, '
                      'and the delivered-sugarbox check uses the unverified position')
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            issues.append('placement box position: ' + str(exc))
        try:
            if not patrol_supports_permit(args.host):
                issues.append('the robot\'s patrol has no --vision-permit: deploy ncu\'s '
                              'g2_map_patrol.py from PR #5 e359fb5 or later')
        except (OSError, subprocess.SubprocessError) as exc:
            issues.append('patrol check: ' + str(exc))
    if resume != 'place':
        try:
            camera = ensure_arm_camera(args.host, fix=False)
        except (OSError, subprocess.SubprocessError) as exc:
            camera = 'arm camera check failed: ' + str(exc)
        if camera and 'restarts' in camera:
            print('[WARN] ' + camera)
        elif camera:
            issues.append(camera)
    for path in (runner.SUGARBOX_MODEL, runner.PLACEMENT_MODEL, args.homography):
        if not Path(path).is_file():
            issues.append('missing ' + str(path))
    if args.gstreamer is None or not Path(args.gstreamer).is_file():
        issues.append('gstreamer receiver missing')
    return issues


def grasp_and_place(args, pc_ip, log_dir):
    """One object, no patrol: the flow that delivered twice on 2026-10-18."""
    if getattr(args, 'resident', False) and _runtime is None:
        return with_runtime(args, pc_ip, log_dir, grasp_and_place)
    log_dir.mkdir(parents=True, exist_ok=True)
    place_argv = ['--target', 'placement', '--real', '--i-am-beside-the-robot',
                  '--obs-log', str(log_dir / 'place_obs.jsonl')] + stage_options(args)
    if args.wall_m is not None:
        place_argv += ['--wall-m', str(args.wall_m)]
    stage, started = 'START', time.monotonic()
    try:
        if not stream_running(args.host, pc_ip):
            start_stream(args.host, pc_ip)
        if args.resume_at != 'place':
            stage = 'GRASP'
            problem = ensure_arm_camera(args.host, fix=True)
            if problem:
                raise MissionError(problem)
            unlock(args.host)
            argv = ['--target', 'sugarbox', '--real', '--i-am-beside-the-robot',
                    '--obs-log', str(log_dir / 'grasp_obs.jsonl')] + stage_options(args)
            if args.resume_at == 'grasp-e1':
                argv.append('--handoff-only')
            code = run_stage(args.host, argv, log_dir / 'grasp.log')
            if code:
                hint = ''
                if arm_camera_state(args.host) == 'unplugged':
                    hint = ('; the arm camera dropped off USB: replug it and rerun with '
                            '--resume-at grasp-e1 (the robot can stay where it is)')
                raise MissionError('grasp runner exit {}{}'.format(code, hint))
            if runner.ctl(args.host, 'health').get('holding') is not True:
                raise MissionError('grasp reported done but nothing is held')
        stage = 'PLACE'
        unlock(args.host)
        code = run_stage(args.host, place_argv, log_dir / 'place.log')
        if code:
            still = runner.ctl(args.host, 'health').get('holding') is True
            raise MissionError('placement runner exit {}{}'.format(code, (
                '; still holding: put the placement box in view and rerun with '
                '--resume-at place') if still else ''))
        stage = 'DONE'
        print('[MISSION] grasped and released in {:.0f} s; logs in {}'.format(
            time.monotonic() - started, log_dir), flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print('[MISSION] stopped in {}: {}'.format(stage, exc), flush=True)
        print('[MISSION] logs in {}'.format(log_dir), flush=True)
        return 2
    finally:
        try:
            runner.stop(args.host)
        except OSError as exc:
            print('[MISSION] final stop not delivered: {}; resident watchdog remains'.format(exc))


def box_left_at(host, ahead=None):
    """Map position of a sugarbox the robot stands at: `ahead` (chassis-centre x, y)
    or the approach's stop AT_BOX_X_M straight ahead. None without a pose."""
    try:
        return map_point(dock.localization_pose(dock.read_localization(host)),
                         ahead or (AT_BOX_X_M, 0.0))
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        print('[MISSION] where the sugarbox stands is unknown: {}'.format(exc), flush=True)
        return None


def at_box_ahead():
    """Where the runner last had the sugarbox when it reported a miss at the box
    (chassis-centre x, y), or None when it could not say."""
    try:
        report = json.loads(runner.AT_BOX_REPORT.read_text(encoding='utf-8'))
        ahead = float(report['x']), float(report['y'])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return ahead if all(math.isfinite(v) for v in ahead) else None


def at_box_back_out(ahead):
    """How far to back from a box `ahead` so the camera measures it again."""
    x = AT_BOX_X_M if ahead is None else ahead[0]
    return round(min(AT_BOX_BACK_MAX_M, max(BACK_OUT_M, AT_BOX_SEEN_M - x)), 3)


def fetch_sugarbox(args, sighting, log_dir, n):
    """GRASP: 'held', 'missed' (nothing held, arm in a known state: back out and
    patrol on) or 'missed_at_box' (the same, but the approach had reached the box).

    Raises MissionError whenever the service cannot vouch for the arm, and when
    the arm camera is off USB (someone has to replug it).
    """
    problem = ensure_arm_camera(args.host, fix=True)
    if problem:
        raise MissionError(problem)
    unlock(args.host)
    argv = ['--target', 'sugarbox', '--real', '--i-am-beside-the-robot',
            '--obs-log', str(log_dir / 'grasp_{}_obs.jsonl'.format(n))] + stage_options(args)
    seed = approach_seed(args.host, sighting)
    if seed is not None:
        argv += ['--seed', '{:.3f}'.format(seed[0]), '{:.3f}'.format(seed[1])]
    code = run_stage(args.host, argv, log_dir / 'grasp_{}.log'.format(n))
    if _runtime is not None and not _runtime.usable():
        # Timed out or faulted: the resident worker was ended and cannot run
        # another stage, so backing out to patrol on would lead nowhere.
        raise MissionError('grasp runner exit {}: the model worker was ended; mission '
                           'stopped where it is'.format(code))
    state = arm_state(args.host)
    if state == 'holding':
        return 'held'
    if state == 'unknown':
        raise MissionError('grasp runner exit {}; the arm state is unknown'.format(code))
    if code == 0:
        raise MissionError('grasp reported done but nothing is held')
    if arm_camera_state(args.host) == 'unplugged':
        raise MissionError('grasp runner exit {}: the arm camera dropped off USB; replug '
                           'it'.format(code))
    to_travel(args.host)
    return 'missed_at_box' if code == runner.AT_BOX_EXIT else 'missed'


def place_held(args, layout_copy, log_dir, n, new_reader, new_watcher):
    """PLACE: turn toward the box and release over it. Still holding after a
    failure: look around for the box once and try again from where it shows."""
    argv = ['--target', 'placement', '--real', '--i-am-beside-the-robot'] + stage_options(args)
    if args.wall_m is not None:
        argv += ['--wall-m', str(args.wall_m)]
    code = run_stage(args.host, argv + [
        '--turn-to-box', '--box-map', str(layout_copy),
        '--obs-log', str(log_dir / 'place_{}_obs.jsonl'.format(n))],
        log_dir / 'place_{}.log'.format(n))
    state = arm_state(args.host)
    if code == 0 and state == 'empty':
        return
    if state != 'holding':
        raise MissionError('placement runner exit {}; the arm state is {}'.format(code, state))
    to_travel(args.host)
    print('[MISSION] placement failed (exit {}) still holding: looking around for the box'
          .format(code), flush=True)
    reader = new_reader()
    try:
        found = search_placement(args.host, reader, new_watcher())
    finally:
        reader.close()            # the approach opens its own receiver on this port
    if not found:
        raise MissionError('placement box not found in a full turn; still holding')
    code = run_stage(args.host, argv + [
        '--obs-log', str(log_dir / 'place_{}_retry_obs.jsonl'.format(n))],
        log_dir / 'place_{}_retry.log'.format(n))
    state = arm_state(args.host)
    if code == 0 and state == 'empty':
        return
    raise MissionError('placement failed twice (exit {}); the arm is {}'.format(code, state))


def patrol_mission(args, pc_ip, log_dir, room=None, new_link=VisionLink):
    """Patrol, fetch, place, back out, patrol on: until two whole laps find nothing."""
    if getattr(args, 'resident', False) and _runtime is None:
        return with_runtime(args, pc_ip, log_dir, patrol_mission, room=room, new_link=new_link)
    from integration.rear_stream import StreamReader
    log_dir.mkdir(parents=True, exist_ok=True)
    stage, reader, link, delivered, failures, n = 'START', None, None, 0, [], 0
    at_box = []
    try:
        layout = dock.fetch_layout(args.host, args.scene)
        layout_copy = log_dir / 'scene_layout.json'
        layout_copy.write_text(json.dumps(layout, indent=1), encoding='utf-8')
        box = dock.box_from_layout(layout, str(layout_copy))
        walls = dock.walls_from_layout(layout)
        if walls is None:
            raise MissionError('scene layout has no room_polygon: wall clips could not be '
                               'told from sugarboxes')
        if not stream_running(args.host, pc_ip):
            start_stream(args.host, pc_ip)
        watcher = SugarboxWatcher(args.homography)
        obstacle = dock.bin_obstacle(box)
        room = room or Room(args.host, walls, obstacles=[obstacle] if obstacle else [])

        def new_reader():
            return _runtime.reader() if _runtime is not None else StreamReader(STREAM_PORT, args.gstreamer)

        while args.max_deliveries is None or delivered < args.max_deliveries:
            n += 1
            stage = 'PATROL {}'.format(n)
            reader, link = new_reader(), new_link(args.host)
            skip = tried_out(failures)
            sighting = patrol_until_sugarbox(args.host, args.scene, reader, watcher, link, box,
                                             skip=skip, guard=skip, blind=at_box,
                                             make_room=room.make, walls=walls)
            link.close()
            link = None
            reader.close()            # the approach opens its own receiver on this port
            reader = None
            if sighting is None:
                print('[MISSION] no sugarbox in {} whole laps: done, {} delivered'.format(
                    EMPTY_LAPS_TO_END, delivered), flush=True)
                return 0
            room.reset()
            stage = 'GRASP {}'.format(n)
            outcome = fetch_sugarbox(args, sighting, log_dir, n)
            if outcome == 'missed_at_box':
                # Right in front: backed out far enough for the camera to see it
                # again, and guarded within the gripper's blind zone meanwhile.
                ahead = at_box_ahead()
                spot = box_left_at(args.host, ahead) or sighting['map']
                if spot is None:
                    raise MissionError('missed sugarbox position unknown; cannot safely '
                                       'back out and restart patrol')
                at_box.append(spot)
                failures.append(spot)
                back = at_box_back_out(ahead)
                print('[MISSION] grasp {} missed with the robot at the box at map '
                      '({:.2f}, {:.2f}): backing out {:.2f} m to see it again'.format(
                          n, spot[0], spot[1], back), flush=True)
                stage = 'BACK_OUT {}'.format(n)
                room.leave(back)
                continue
            if outcome == 'missed':
                spot = sighting['map']
                if spot is not None:
                    failures.append(spot)
                print('[MISSION] grasp {} missed at {}: backing out, patrolling on'.format(
                    n, 'map ({:.2f}, {:.2f})'.format(*spot) if spot is not None
                    else 'an unknown map spot (not counted)'), flush=True)
                stage = 'BACK_OUT {}'.format(n)
                room.leave()
                continue
            if outcome != 'held':
                raise MissionError('unrecognized grasp outcome {}; remaining stopped'
                                   .format(outcome))
            # Grasped: its spot holds no box any more. Where it was grasped (the
            # robot is still there) and where the patrol saw it both count.
            centres = [c for c in (box_left_at(args.host) if at_box or failures else None,
                                   sighting['map']) if c is not None]
            gone = lambda p: any(math.hypot(p[0] - c[0], p[1] - c[1]) <= SPOT_RADIUS_M
                                 for c in centres)
            at_box[:] = [p for p in at_box if not gone(p)]
            failures[:] = [p for p in failures if not gone(p)]
            stage = 'PLACE {}'.format(n)
            place_held(args, layout_copy, log_dir, n, new_reader,
                       lambda: PlacementWatcher(args.homography))
            delivered += 1
            print('[MISSION] {} delivered'.format(delivered), flush=True)
            stage = 'BACK_OUT {}'.format(n)
            room.leave()
        print('[MISSION] {} delivered: done'.format(delivered), flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print('[MISSION] stopped in {}: {}'.format(stage, exc), flush=True)
        print('[MISSION] logs in {}'.format(log_dir), flush=True)
        return 2
    finally:
        if link is not None:
            link.close()
        if reader is not None:
            reader.close()
        try:
            runner.stop(args.host)
        except OSError as exc:
            print('[MISSION] final stop not delivered: {}; resident watchdog remains'.format(exc))


def with_runtime(args, pc_ip, log_dir, mission, **kwargs):
    """One camera/model lifetime, including warm-up, before any patrol starts."""
    from integration.mission_runtime import MissionRuntime, resident_session
    from integration.robot_rpc import RobotSession
    global _runtime
    if _runtime is not None:
        raise MissionError('a mission runtime is already active')
    runtime, robot = None, RobotSession(args.host)
    try:
        runner.stop(args.host)
        if not stream_running(args.host, pc_ip):
            start_stream(args.host, pc_ip)
        runtime = MissionRuntime(args.host, args.gstreamer, log_dir)
        with resident_session(runner, runtime):
            _runtime = runtime
            runner._ROBOT_SESSION = robot
            runtime.reader().close()  # fill the single receiver during warm-up
            assets = resolve_asset_paths(str(ROOT))
            environments = []
            for kind, model, size in [('sugarbox', runner.SUGARBOX_MODEL, WATCH_IMGSZ),
                                      ('placement', runner.PLACEMENT_MODEL, PLACEMENT_IMGSZ)]:
                _paths, issues = runner.preflight(argparse.Namespace(
                    target=kind, sam=Path(assets['sam']), homography=args.homography,
                    gstreamer=args.gstreamer, turn_to_box=False, host=args.host))
                if issues:
                    raise MissionError('resident {} preflight: {}'.format(kind, '; '.join(issues)))
                env = dict(os.environ, SUGARBOX_RUN_MODE='DRY_RUN', SUGARBOX_WAIT_GO='1',
                           SUGARBOX_YOLO_MODEL=str(model.resolve()),
                           SUGARBOX_SAM_MODEL=str(Path(assets['sam']).resolve()),
                           SUGARBOX_HOMOGRAPHY=str(args.homography.resolve()),
                           SUGARBOX_RL_MODEL=str((ROOT / 'integration/nav_best_model/doorway_ft_final.zip').resolve()),
                           SUGARBOX_VECNORMALIZE=str((ROOT / 'integration/nav_best_model/doorway_ft_final_vecnormalize.pkl').resolve()),
                           SUGARBOX_YOLO_IMGSZ=str(size), SUGARBOX_TARGET=kind,
                           GSTREAMER_LAUNCH_EXE=str(Path(args.gstreamer).resolve()))
                environments.append(env)
            print('[MISSION] warming models once; camera remains connected between stages', flush=True)
            runtime.prepare(environments)
            return mission(args, pc_ip, log_dir, **kwargs)
    except (Exception, KeyboardInterrupt) as exc:
        print('[MISSION] resident runtime stopped: {}'.format(exc), flush=True)
        return 2
    finally:
        _runtime = None
        runner._ROBOT_SESSION = None
        robot.close()
        # Even a failure while preparing the models leaves the robot stopped.
        try:
            runner.stop(args.host)
        except OSError:
            pass
        if runtime is not None:
            runtime.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', required=True)
    parser.add_argument('--scene', default=SCENE,
                        help='Jetson-side scene folder: the patrol and the placement box position')
    parser.add_argument('--max-deliveries', type=int, default=None,
                        help='stop after this many (default: until two whole laps find none)')
    parser.add_argument('--wall-m', type=float, default=None,
                        help='taught wall distance for the placement stop (see placement_dock.py)')
    parser.add_argument('--homography', type=Path,
                        default=Path(resolve_asset_paths(str(ROOT))['homography']))
    parser.add_argument('--gstreamer', type=Path, default=runner.find_gstreamer())
    parser.add_argument('--no-patrol', action='store_true',
                        help='one object, no patrol: grasp the sugarbox in front, then carry '
                             'it to the placement box (which must then be in the rear view)')
    parser.add_argument('--resume-at', choices=('grasp-e1', 'place'), default=None,
                        help='with --no-patrol, pick a stopped run up: grasp-e1 (robot at the '
                             'sugarbox, e.g. after an arm-camera dropout) or place (object held)')
    parser.add_argument('--log-dir', type=Path, default=None,
                        help='where each stage writes its output (default logs/mission_<time>)')
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--isolated-stages', action='store_false', dest='resident', default=True,
                        help='fallback: separate model process/receiver per stage')
    parser.add_argument('--i-am-beside-the-robot', action='store_true')
    args = parser.parse_args(argv)
    ipaddress.IPv4Address(args.host)
    if args.max_deliveries is not None and args.max_deliveries < 1:
        parser.error('--max-deliveries must be at least 1')
    if args.no_patrol and args.max_deliveries is not None:
        parser.error('--no-patrol delivers one object; set the next one up and run again')
    if args.resume_at and not args.no_patrol:
        parser.error('--resume-at goes with --no-patrol')
    log_dir = args.log_dir or LOG_ROOT / time.strftime('mission_%Y%m%d_%H%M%S')
    issues = preflight(args)
    for issue in issues:
        print('[BLOCKED] ' + issue)
    if issues:
        return 2
    pc_ip = pc_address(args.host)
    if args.no_patrol:
        print('[MISSION] PC {} | one object, no patrol{}'.format(
            pc_ip, '' if not args.resume_at else ' | resume at ' + args.resume_at))
    else:
        print('[MISSION] PC {} | patrol {} | {}'.format(
            pc_ip, args.scene, 'until two laps find nothing' if args.max_deliveries is None
            else 'at most {} deliveries'.format(args.max_deliveries)))
    if not args.real:
        print('[check] robot, cameras and files passed{}; no motion.'.format(
            '' if args.no_patrol else ', localization, box position'))
        return 0
    if not args.i_am_beside_the_robot:
        print('[REFUSED] --real requires --i-am-beside-the-robot')
        return 3
    if args.no_patrol:
        return grasp_and_place(args, pc_ip, log_dir)
    return patrol_mission(args, pc_ip, log_dir)


if __name__ == '__main__':
    raise SystemExit(main())
