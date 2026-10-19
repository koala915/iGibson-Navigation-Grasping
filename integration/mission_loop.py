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
sugarbox to fetch. A missed grasp with the arm in a known state backs out and
patrols on (a spot that failed twice is left alone); a failed placement still
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
LOG_ROOT = ROOT / 'logs'
# The E1 camera, on the hub it shares with the motor board; it drops off USB
# after arm moves now and then (twice on 2026-10-18) and may need replugging.
ARM_CAMERA = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0'
JETSON_ROOT = dock.JETSON_ROOT
SCENE = dock.SCENE
PATROL_OUTPUT = '/tmp/mission_patrol.json'
PATROL_LOG = '/tmp/mission_patrol.log'
STREAM_PORT = 5600
# The approach locks a box after 3 consistent frames; the watcher asks the same,
# so it never stops the patrol for a box the approach would not take.
WATCH_CONF = 0.50
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
             '--seconds 600 --output {} --real').format(shlex.quote(scene), shlex.quote(output))
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


def stop_patrol(host, pid, timeout=15.0):
    """SIGTERM: the patrol stops the wheels, re-locks the service, writes its report."""
    ssh(host, 'kill -TERM {}'.format(int(pid)))
    deadline = time.monotonic() + timeout
    while patrol_running(host, pid):
        if time.monotonic() > deadline:
            raise MissionError('patrol {} did not exit after SIGTERM'.format(pid))
        time.sleep(0.3)


def patrol_report(host):
    result = ssh(host, 'cat ' + PATROL_OUTPUT)
    try:
        return json.loads(result.stdout)
    except ValueError:
        return {'ok': False, 'error': 'no patrol report'}


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

    def __init__(self, model_path, homography_path, imgsz):
        from ultralytics import YOLO
        self.model, self.imgsz = YOLO(str(model_path)), imgsz
        self.H, self.rear_cam_to_front, self.hull = load_ground_calibration(homography_path)

    def look(self, frame):
        """(point, far) in the chassis-centre frame, or None with no box in view."""
        result = self.model.predict(frame, imgsz=self.imgsz, conf=WATCH_CONF, verbose=False)[0]
        near = far = None
        for box in result.boxes:
            bbox = tuple(float(v) for v in box.xyxy[0].tolist())
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
        super().__init__(runner.SUGARBOX_MODEL, homography_path, WATCH_IMGSZ)


class PlacementWatcher(BoxWatcher):
    def __init__(self, homography_path):
        super().__init__(runner.PLACEMENT_MODEL, homography_path, PLACEMENT_IMGSZ)


def fresh_frame(reader, last_stamp, now):
    """(frame, stamp) when a new frame is in, (None, last_stamp) while waiting."""
    frame, stamp = reader.latest()
    if now() - stamp > STREAM_STALE_S:
        raise MissionError('rear camera stream lost')
    if stamp == last_stamp:
        time.sleep(0.01)
        return None, last_stamp
    return frame, stamp


def patrol_until_sugarbox(host, scene, reader, watcher, box=None, skip=(), make_room=None,
                          now=time.monotonic):
    """Patrol lap after lap until the camera confirms a sugarbox worth fetching.

    Returns {'point': chassis-centre (x, y) when confirmed, 'map': its map position
    or None}, or None after EMPTY_LAPS_TO_END whole laps without one. Passed over:
    a sugarbox at the placement box (delivered) and one at a spot in `skip`. A
    patrol that refuses to start where the robot stands gets make_room() (a short
    straight back-off) and another try. The patrol is stopped before returning,
    and on any error too.
    """
    confirm, pid, empty_laps, last_stamp = Confirm(), None, 0, None
    deadline = now() + 8.0
    while reader.latest()[0] is None:          # never patrol blind
        if now() > deadline:
            raise MissionError('no rear camera frames; is the stream reaching this PC?')
        time.sleep(0.1)
    try:
        while True:
            if pid is None or not patrol_running(host, pid):
                if pid is not None:
                    report, pid = patrol_report(host), None
                    if report.get('ok'):
                        empty_laps += 1
                        if empty_laps >= EMPTY_LAPS_TO_END:
                            return None
                    elif patrol_refused(report) and make_room is not None and make_room():
                        print('[PATROL] refused to start here ({}); backed off, asking again'
                              .format(report.get('error')), flush=True)
                    else:
                        raise MissionError('patrol stopped: ' + str(report.get('error')))
                pid = start_patrol(host, scene)
                print('[PATROL] lap {} started (pid {})'.format(empty_laps + 1, pid), flush=True)
            frame, last_stamp = fresh_frame(reader, last_stamp, now)
            if frame is None:
                continue
            sighting = watcher.look(frame)
            point, far = sighting if sighting is not None else (None, False)
            if not confirm.update(point, last_stamp, far):
                continue
            seen, far = confirm.mean(), confirm.far
            confirm.reset()
            where = None
            try:
                where = map_point(dock.localization_pose(dock.read_localization(host)), seen)
            except (ValueError, OSError, subprocess.SubprocessError) as exc:
                print('[PATROL] cannot place the sighting on the map ({}); taking it'
                      .format(exc), flush=True)
            if where is not None:
                if box is not None and math.hypot(where[0] - box['x'],
                                                  where[1] - box['y']) <= PLACED_EXCLUDE_M:
                    print('[PATROL] sugarbox at the placement box: already delivered',
                          flush=True)
                    continue
                if any(math.hypot(where[0] - x, where[1] - y) <= SPOT_RADIUS_M for x, y in skip):
                    print('[PATROL] sugarbox where grasping failed {} times: left alone'
                          .format(TRIES_PER_SPOT), flush=True)
                    continue
            print('[PATROL] sugarbox {:.2f} m ahead, {:+.2f} m left{}: stopping the patrol'
                  .format(seen[0], seen[1], ' (beyond the calibrated area, rough)' if far else ''),
                  flush=True)
            stop_patrol(host, pid)
            pid = None
            return {'point': seen, 'map': where}
    finally:
        if pid is not None and patrol_running(host, pid):
            stop_patrol(host, pid)


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
    for step in range(SEARCH_STEPS + 1):
        if step:
            turn(host, SEARCH_STEP_RAD)
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
    since the last stage; the runner checks the space behind before every step.
    """

    def __init__(self, host, back=None):
        self.host, self.back, self.backed = host, back, 0.0

    def _back(self, metres):
        (self.back or runner.back_straight)(self.host, metres)

    def leave(self):
        self._back(BACK_OUT_M)
        self.backed = BACK_OUT_M

    def make(self):
        if self.backed + ROOM_STEP_M > ROOM_MAX_M + 1e-9:
            return False
        self._back(ROOM_STEP_M)
        self.backed += ROOM_STEP_M
        return True

    def reset(self):
        self.backed = 0.0


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
        holding = runner.ctl(args.host, 'health').get('holding')
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
    log_dir.mkdir(parents=True, exist_ok=True)
    place_argv = ['--target', 'placement', '--real', '--i-am-beside-the-robot',
                  '--obs-log', str(log_dir / 'place_obs.jsonl')]
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
            if runner.ctl(args.host, 'status').get('chassis', {}).get('maintenance') is True:
                runner.service_command(args.host, 'maintenance-unlock')
            argv = ['--target', 'sugarbox', '--real', '--i-am-beside-the-robot',
                    '--obs-log', str(log_dir / 'grasp_obs.jsonl')]
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


def fetch_sugarbox(args, sighting, log_dir, n):
    """GRASP: 'held', or 'missed' (nothing held, arm in a known state: patrol on).

    Raises MissionError whenever the service cannot vouch for the arm, and when
    the arm camera is off USB (someone has to replug it).
    """
    problem = ensure_arm_camera(args.host, fix=True)
    if problem:
        raise MissionError(problem)
    unlock(args.host)
    argv = ['--target', 'sugarbox', '--real', '--i-am-beside-the-robot',
            '--obs-log', str(log_dir / 'grasp_{}_obs.jsonl'.format(n))]
    seed = approach_seed(args.host, sighting)
    if seed is not None:
        argv += ['--seed', '{:.3f}'.format(seed[0]), '{:.3f}'.format(seed[1])]
    code = run_stage(args.host, argv, log_dir / 'grasp_{}.log'.format(n))
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
    return 'missed'


def place_held(args, layout_copy, log_dir, n, new_reader, new_watcher):
    """PLACE: turn toward the box and release over it. Still holding after a
    failure: look around for the box once and try again from where it shows."""
    argv = ['--target', 'placement', '--real', '--i-am-beside-the-robot']
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


def patrol_mission(args, pc_ip, log_dir, room=None):
    """Patrol, fetch, place, back out, patrol on: until two whole laps find nothing."""
    from integration.rear_stream import StreamReader
    log_dir.mkdir(parents=True, exist_ok=True)
    stage, reader, delivered, failures, n = 'START', None, 0, [], 0
    try:
        layout = dock.fetch_layout(args.host, args.scene)
        layout_copy = log_dir / 'scene_layout.json'
        layout_copy.write_text(json.dumps(layout, indent=1), encoding='utf-8')
        box = dock.box_from_layout(layout, str(layout_copy))
        if not stream_running(args.host, pc_ip):
            start_stream(args.host, pc_ip)
        watcher = SugarboxWatcher(args.homography)
        room = room or Room(args.host)

        def new_reader():
            return StreamReader(STREAM_PORT, args.gstreamer)

        while args.max_deliveries is None or delivered < args.max_deliveries:
            n += 1
            stage = 'PATROL {}'.format(n)
            reader = new_reader()
            sighting = patrol_until_sugarbox(args.host, args.scene, reader, watcher, box,
                                             skip=tried_out(failures), make_room=room.make)
            reader.close()            # the approach opens its own receiver on this port
            reader = None
            if sighting is None:
                print('[MISSION] no sugarbox in {} whole laps: done, {} delivered'.format(
                    EMPTY_LAPS_TO_END, delivered), flush=True)
                return 0
            room.reset()
            stage = 'GRASP {}'.format(n)
            if fetch_sugarbox(args, sighting, log_dir, n) == 'missed':
                if sighting['map'] is not None:
                    failures.append(sighting['map'])
                print('[MISSION] grasp {} missed: backing out, patrolling on'.format(n),
                      flush=True)
                stage = 'BACK_OUT {}'.format(n)
                room.leave()
                continue
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
        if reader is not None:
            reader.close()
        try:
            runner.stop(args.host)
        except OSError as exc:
            print('[MISSION] final stop not delivered: {}; resident watchdog remains'.format(exc))


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
