#!/usr/bin/env python3
"""Windows orchestration: supervised avoidance approach -> stopped E1 grasp,
or (--target placement) -> stopped release over the placement box.

Default is a read-only preflight. The Jetson resident service remains the only
serial owner; this runner never starts a second motor server. It releases only in
placement mode, only after the approach reports a checked stop at the box.
"""
import argparse
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import shlex
import socket
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from integration.sugarbox_ground_calibration import load_ground_calibration, resolve_asset_paths
from integration import placement_dock as dock

APPROACH = ROOT / 'integration/sugarbox_rl_approach_final2.py'
REMOTE_CTL = '/home/jetson/Documents/deploy_jetson2/grasp/v23/graspctl.py'
SERVICE_SOCKET = '/tmp/grasp_service.sock'
SUGARBOX_MODEL = ROOT / 'detection/models/best.pt'
# Retrained on 70 rear-camera frames on 2026-10-18 (yolo_train
# placement_box_yolo11_robotcam_20261018): the phone-photo model found nothing in
# the rear camera. imgsz 640, the camera's own size.
PLACEMENT_MODEL = ROOT / 'detection/models/placement_box.pt'
PLACEMENT_MODEL_SHA256 = '8e256e80b770a6ff7a5a3187ab7cbeb7e30fb84a5d6a1fc1515f84a3025406a0'
# graspctl command -> reply timeout (s). release/release-preview come from E1.
ARM_COMMANDS = {'status': 75, 'health': 30, 'stow': 75, 'home': 75, 'grasp': 200,
                'release': 150, 'release-preview': 150}
# In place the chassis needs >= 1.0 rad/s (wheel speed 20 does not turn it).
TURN_WZ = 1.0
# Stop this much early for odometry latency and coast; to be measured on the robot.
TURN_LEAD_RAD = 0.08
# Written every frame by the E1 bridge (--frame-report in its systemd drop-in).
FRAME_REPORT = '/dev/shm/grasp_vision_frame.json'
FRAME_MAX_AGE_S = 1.5
# E1 grasp band in raw homography x: the measured hull's near edge, and the policy
# limit 0.280 m less the bridge's +5 mm landing offset and 2 mm for frame-to-frame
# jitter (2026-10-16: a box at raw 0.275 flickered between 0.280 and 0.281 and was
# never accepted). The chassis aims mid-band.
E1_NEAR_X, E1_FAR_X = 0.249, 0.273
E1_AIM_X = (E1_NEAR_X + E1_FAR_X) / 2
ADJUST_TRIES = 3
ADJUST_MAX_M = 0.06
ADJUST_BACK_M = 0.02
ADJUST_LEAD_M = 0.012
# E1 working but seeing no box at all: creep forward and look again. 2026-10-18 on
# the white ribbed mats, wheel odometry read 0.105-0.123 m/s at 0.08 commanded
# over the blind last 41 cm and parked 3-6 cm short, the box beyond the top of
# the E1 view. Odometry that over-reads only ever stops the robot short, and a
# box it overshot would show at the bottom edge (that is a seen frame), so
# forward is the only direction this takes.
BLIND_CREEP_M = 0.03
BLIND_CREEP_TRIES = 2
BLIND_CREEP_LOOKS = 4       # fresh frames in a row that saw nothing (E1 runs at 1 Hz)
# Two fresh E1 readings must agree within 8 mm, the second within this long of the first.
AGREE_WITHIN_S = 4.0
MODEL_HASHES = {
    'doorway_ft_final.zip': 'af5e9e1d5ce0115de8b4bbb321873c6a99686c82c0c69d060f881bbe97b64f35',
    'doorway_ft_final_vecnormalize.pkl': '3cd4ef9ab5b0fa38058b5142b1cc5b5df1ba9c440f325ff81b88d5e6ca7a8cdf',
}


def ctl(host, command):
    if command not in ARM_COMMANDS:
        raise ValueError('unsupported arm command')
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
         'jetson@' + host, 'python3 ' + REMOTE_CTL + ' ' + command],
        capture_output=True, text=True, timeout=ARM_COMMANDS[command])
    if result.returncode:
        raise RuntimeError('graspctl {} failed: {}'.format(command, result.stdout + result.stderr))
    value = json.loads(result.stdout)
    if value.get('ok') is not True:
        raise RuntimeError('graspctl refused: ' + str(value))
    return value


def service_command(host, command):
    """ncu's maintenance lock, which graspctl does not forward (PR #5 service).

    His navigation tools lock the wheels and arm when they finish; the next
    approach needs them unlocked. The service grants either only when idle.
    """
    if command not in ('maintenance-lock', 'maintenance-unlock'):
        raise ValueError('unsupported service command')
    script = ("import socket;s=socket.socket(socket.AF_UNIX);s.settimeout(10);"
              "s.connect({!r});s.sendall({!r});print(s.makefile().readline())").format(
                  SERVICE_SOCKET, (command + '\n').encode())
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', 'jetson@' + host,
         'python3 -c ' + shlex.quote(script)],
        capture_output=True, text=True, timeout=30)
    try:
        value = json.loads(result.stdout)
    except ValueError:
        raise RuntimeError('{} failed: {}'.format(command, result.stdout + result.stderr))
    if value.get('ok') is not True:
        raise RuntimeError('{} refused: {}'.format(command, value))
    return value


def stop(host):
    with socket.create_connection((host, 7000), timeout=3) as sock:
        for _ in range(3):
            sock.sendall(b'{"action":"stop"}\n')
            time.sleep(0.05)


def stationary(status):
    chassis, odom = status.get('chassis', {}), status.get('odom', {})
    age = chassis.get('board_rx_age_s')
    held = chassis.get('stopped_for_s')
    twist = odom.get('twist')
    return (chassis.get('moving') is False and chassis.get('arm_busy') is False
            and chassis.get('motors') == [0, 0, 0, 0]
            and isinstance(age, (int, float)) and 0 <= age < 0.5
            and isinstance(held, (int, float)) and held >= 0.5
            and odom.get('valid') is True and odom.get('failures') == 0
            and isinstance(twist, list) and len(twist) == 3
            and all(isinstance(v, (int, float)) and math.isfinite(v)
                    and abs(v) <= 0.01 for v in twist))


def target_position(status):
    detection = status.get('detection', {})
    pos = detection.get('pos')
    if (detection.get('fresh') is not True or not isinstance(pos, list) or len(pos) != 3
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pos)):
        return None
    if not (0.205 <= pos[0] <= 0.280 and -0.070 <= pos[1] <= 0.065 and pos[2] > 0):
        return None
    return pos


def adjust_distance(record):
    """Metres to drive (+ forward) so a box E1 sees but cannot use lands mid grasp band.

    `record` is the bridge's --frame-report for one frame plus its age on the robot's
    clock. None means driving would not help: nothing fresh, already accepted, cut at
    a side (no sideways motion), or in the band but refused for another reason.
    """
    if (not isinstance(record, dict) or not 0 <= record.get('age', -1) <= FRAME_MAX_AGE_S
            or record.get('seen') is not True or record.get('accepted') is not False):
        return None
    border = set(record.get('border') or ())
    if border & {'left', 'right'}:
        return None
    if 'bottom' in border:
        # Its floor contact is below the frame: nearer by an unknown amount.
        return -ADJUST_BACK_M
    estimate = record.get('estimate')
    if not (isinstance(estimate, list) and len(estimate) == 2
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in estimate)):
        return None
    x, y = estimate
    if not -0.070 <= y <= 0.065 or E1_NEAR_X <= x <= E1_FAR_X:
        return None
    return max(-ADJUST_MAX_M, min(ADJUST_MAX_M, x - E1_AIM_X))


def look(host):
    """Latest E1 frame report, aged on the robot's clock; None if unreadable."""
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', 'jetson@' + host,
         'cat {} 2>/dev/null; echo; date +%s.%N'.format(FRAME_REPORT)],
        capture_output=True, text=True, timeout=30)
    lines = result.stdout.strip().splitlines()
    try:
        record = json.loads(lines[0]) if len(lines) == 2 else None
        record['age'] = float(lines[1]) - float(record['t'])
    except (TypeError, ValueError, KeyError):
        return None
    return record


def yaw_of(q):
    return math.atan2(2.0 * (q['w'] * q['z'] + q['x'] * q['y']),
                      1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z']))


class Odometry:
    """Latest wheel-odometry pose (/odom_setmotor via rosbridge) for measured moves."""

    def __init__(self, host):
        import roslibpy
        self.latest = {}
        self.ros = roslibpy.Ros(host=host, port=9090)
        self.ros.run()
        self.topic = roslibpy.Topic(self.ros, '/odom_setmotor', 'nav_msgs/Odometry')
        self.topic.subscribe(self._update)

    def _update(self, message):
        pose = message['pose']['pose']
        self.latest = {'x': pose['position']['x'], 'y': pose['position']['y'],
                       'yaw': yaw_of(pose['orientation']), 't': time.monotonic()}

    def wait(self):
        started = time.monotonic()
        while not self.latest:
            if time.monotonic() - started > 5:
                raise RuntimeError('no odometry; wheels not moved')
            time.sleep(0.05)
        return dict(self.latest)

    def check_fresh(self):
        if time.monotonic() - self.latest['t'] > 0.3:
            raise RuntimeError('odometry went stale while moving')

    def close(self):
        self.topic.unsubscribe()
        self.ros.close()


def drive(host, metres, limit=ADJUST_MAX_M):
    """Straight move measured by wheel odometry; the arm must already be in travel.

    The stop goes out ADJUST_LEAD_M early: on 2026-10-16 a 3.6 cm creep at 0.08 m/s
    sent its stop 0.5 cm late (odometry latency) and coasted another 0.8 cm.
    Backing is 0.08 m/s, the only reverse the team allows (2026-10-12).
    """
    if not (math.isfinite(metres) and 0 < abs(metres) <= limit <= 0.30):
        raise ValueError('refusing a move of {!r} m'.format(metres))
    goal = max(abs(metres) - ADJUST_LEAD_M, 0.003)
    command = json.dumps({'action': 'velocity', 'vx': math.copysign(0.08, metres),
                          'wz': 0.0}).encode() + b'\n'
    odom = Odometry(host)
    try:
        origin = odom.wait()

        def moved():
            return math.hypot(odom.latest['x'] - origin['x'], odom.latest['y'] - origin['y'])

        with socket.create_connection((host, 7000), timeout=3) as sock:
            try:
                started = time.monotonic()
                while moved() < goal:
                    # 2 s for the 6 cm adjustments, as before; longer moves pro rata.
                    if time.monotonic() - started > abs(metres) / 0.08 + 1.25:
                        raise RuntimeError('move did not finish in time')
                    odom.check_fresh()
                    sock.sendall(command)
                    time.sleep(0.05)
            finally:
                for _ in range(3):
                    sock.sendall(b'{"action":"stop"}\n')
                    time.sleep(0.05)
        time.sleep(1.0)
        print('[ADJUST] odometry moved {:+.3f} m (asked {:+.3f})'.format(
            math.copysign(moved(), metres), metres), flush=True)
    finally:
        odom.close()


def turn(host, radians):
    """In-place turn measured by wheel odometry (+ = left); arm in travel."""
    if not (math.isfinite(radians) and 0 < abs(radians) <= math.pi):
        raise ValueError('refusing a turn of {!r} rad'.format(radians))
    goal = max(abs(radians) - TURN_LEAD_RAD, 0.02)
    command = json.dumps({'action': 'velocity', 'vx': 0.0,
                          'wz': math.copysign(TURN_WZ, radians)}).encode() + b'\n'
    odom = Odometry(host)
    try:
        last = odom.wait()['yaw']
        turned = 0.0
        with socket.create_connection((host, 7000), timeout=3) as sock:
            try:
                started = time.monotonic()
                while abs(turned) < goal:
                    if time.monotonic() - started > goal / TURN_WZ * 2.0 + 2.0:
                        raise RuntimeError('turn did not finish in time')
                    odom.check_fresh()
                    sock.sendall(command)
                    time.sleep(0.05)
                    yaw = odom.latest['yaw']
                    turned += dock.wrap(yaw - last)
                    last = yaw
                    if abs(turned) > 0.15 and turned * radians < 0:
                        raise RuntimeError('odometry says it turns the other way; stopped')
                    # 1 rad/s should give ~0.5 rad by now; a yaw that does not move
                    # would otherwise keep the robot spinning until the timeout.
                    if time.monotonic() - started > 1.0 and abs(turned) < 0.10:
                        raise RuntimeError('odometry yaw is not changing; stopped')
            finally:
                for _ in range(3):
                    sock.sendall(b'{"action":"stop"}\n')
                    time.sleep(0.05)
        time.sleep(1.0)
        turned += dock.wrap(odom.latest['yaw'] - last)
        print('[TURN] odometry turned {:+.1f} deg (asked {:+.1f})'.format(
            math.degrees(turned), math.degrees(radians)), flush=True)
        return turned
    finally:
        odom.close()


def settle(host, read, send_stop, sleep, now):
    send_stop(host)
    deadline = now() + 8
    while not stationary(read(host, 'status')):
        if now() >= deadline:
            raise RuntimeError('chassis stop not confirmed; no E1 motion')
        sleep(0.5)


def handoff(host, read=ctl, send_stop=stop, sleep=time.sleep, now=time.monotonic,
            look=None, drive=None):
    """Stop, E1, and grasp on two agreeing fresh readings inside the grasp range.

    With look/drive, a box E1 sees outside its grasp band (twice, same direction)
    moves the chassis with the arm in travel, then E1 measures again, at most
    ADJUST_TRIES times. A working E1 that sees nothing at all for
    BLIND_CREEP_LOOKS frames creeps BLIND_CREEP_M forward instead, at most
    BLIND_CREEP_TRIES times. Only an accepted reading is ever grasped.
    """
    adjustments = 0
    creeps = 0
    while True:
        settle(host, read, send_stop, sleep, now)
        read(host, 'home')
        # Discard travel-pose detections: the E1 camera runs at 1 Hz.
        sleep(2.5)
        previous, previous_at = None, 0.0
        move = None
        creep = False
        hints = []
        blank, saw_box = 0, False
        deadline = now() + 12
        while now() < deadline:
            status = read(host, 'status')
            if not stationary(status) or status.get('chassis', {}).get('arm_pose') != 'e1':
                raise RuntimeError('E1/stationary state lost; no grasp')
            pos = target_position(status)
            if (pos is not None and previous is not None
                    and now() - previous_at <= AGREE_WITHIN_S
                    and math.hypot(pos[0] - previous[0], pos[1] - previous[1]) <= 0.008):
                result = read(host, 'grasp')
                if result.get('confirmed') is not True:
                    raise RuntimeError('grasp was not confirmed')
                return result
            # A poll that lands between two E1 frames reads "not fresh"; it must not
            # wipe the last fresh reading (2026-10-18: E1 sent every 1.4 s, fresh
            # lasts 1 s, and the 1.1 s polls never saw two fresh readings in a row).
            if pos is not None:
                previous, previous_at = pos, now()
            saw_box = saw_box or pos is not None
            if pos is None and look is not None and adjustments < ADJUST_TRIES:
                record = look(host)
                step = adjust_distance(record)
                print('[E1] {} -> {}'.format(
                    'no report' if record is None else
                    'seen={} border={} estimate={} age={:.1f}s'.format(
                        record.get('seen'), record.get('border'), record.get('estimate'),
                        record.get('age', float('nan'))),
                    'no move' if step is None else '{:+.3f} m'.format(step)), flush=True)
                # A box near a frame edge flickers in and out of detection; a frame
                # that saw nothing neither confirms nor cancels the last hint.
                if step is not None:
                    hints = hints[-1:] + [step]
                if (len(hints) == 2 and hints[0] * hints[1] > 0
                        and abs(hints[0] - hints[1]) <= 0.015):
                    move = hints[1]
                    break
                if isinstance(record, dict) and record.get('seen') is True:
                    saw_box = True
                elif (isinstance(record, dict) and record.get('seen') is False
                        and 0 <= record.get('age', -1) <= FRAME_MAX_AGE_S):
                    blank += 1
                if not saw_box and blank >= BLIND_CREEP_LOOKS and creeps < BLIND_CREEP_TRIES:
                    move, creep = BLIND_CREEP_M, True
                    break
            else:
                hints = []
            sleep(1.1)
        if move is None:
            raise RuntimeError('no stable fresh E1 target; stopped without grasp')
        if creep:
            creeps += 1
            print('[CREEP] {}/{}: E1 saw nothing in {} fresh frames; the box is beyond its '
                  'view (odometry parked short); forward {:.3f} m'.format(
                      creeps, BLIND_CREEP_TRIES, blank, move), flush=True)
        else:
            adjustments += 1
            print('[ADJUST] {}/{}: E1 sees the box outside its grasp band; {} {:.3f} m'.format(
                adjustments, ADJUST_TRIES, 'forward' if move > 0 else 'back', abs(move)),
                flush=True)
        read(host, 'stow')
        status = read(host, 'status')
        if status.get('chassis', {}).get('arm_pose') != 'travel' or not stationary(status):
            raise RuntimeError('travel pose not confirmed; wheels not moved')
        drive(host, move)


def face_box(host, box, localize=dock.read_localization, rotate=turn, tries=3):
    """Turn in place until the placement box's map position is straight ahead.

    The resident AMCL pose (ncu's localization service) and the box's map position
    say where it is when the rear camera cannot see it, e.g. right after a grasp.
    Returns the final map pose.
    """
    for attempt in range(tries + 1):
        record = localize(host)
        pose = dock.localization_pose(record)
        bearing, distance = dock.box_bearing(pose, box)
        print('[FACE] placement box {:.2f} m away, bearing {:+.1f} deg'.format(
            distance, math.degrees(bearing)), flush=True)
        if abs(bearing) <= dock.TURN_TOLERANCE_RAD:
            return pose
        if attempt == tries:
            break
        if (record.get('state') or {}).get('turn_clear') is not True:
            raise RuntimeError('localization reports no room to turn here; not turning')
        rotate(host, bearing)
    raise RuntimeError('placement box still not ahead after {} turns'.format(tries))


def place(host, report, final_x, aim_y, empty_test, wall_m=None, read=ctl, send_stop=stop,
          sleep=time.sleep, now=time.monotonic, wall=dock.wall_ahead, drive=drive):
    """Release over the placement box once the approach stopped where it should.

    No camera sees the box at the end (the gripper hides it), so the approach's
    own report is the check. With a taught --wall-m the LiDAR distance to the wall
    behind the box corrects the stop first: odometry carried the last ~40 cm.
    """
    problem = dock.arrival_problem(report, final_x, aim_y)
    if problem:
        raise RuntimeError(problem + '; nothing released')
    settle(host, read, send_stop, sleep, now)
    if wall_m is not None:
        for attempt in range(3):
            measured = wall(host)
            try:
                step = dock.wall_correction(measured, wall_m)
            except ValueError as exc:
                raise RuntimeError(str(exc) + '; nothing released')
            print('[WALL] {:.3f} m ahead, taught {:.3f} m -> {}'.format(
                measured, wall_m, 'ok' if not step else '{:+.3f} m'.format(step)), flush=True)
            if not step:
                break
            if attempt == 2:
                raise RuntimeError('wall distance not reached after 2 corrections; nothing released')
            status = read(host, 'status')
            if status.get('chassis', {}).get('arm_pose') != 'travel':
                raise RuntimeError('travel pose not confirmed; wheels not moved')
            drive(host, step)
            settle(host, read, send_stop, sleep, now)
    result = read(host, 'release-preview' if empty_test else 'release')
    if not empty_test:
        read(host, 'stow')          # release ends at E1; preview stows by itself
    status = read(host, 'status')
    if status.get('chassis', {}).get('arm_pose') != 'travel':
        raise RuntimeError('released, but the travel pose is not confirmed')
    if read(host, 'health').get('holding') is not False:
        raise RuntimeError('released, but the service still reports holding')
    return result


def preflight(args):
    issues = []
    paths = {
        'SUGARBOX_YOLO_MODEL': PLACEMENT_MODEL if args.target == 'placement' else SUGARBOX_MODEL,
        'SUGARBOX_SAM_MODEL': args.sam,
        'SUGARBOX_HOMOGRAPHY': args.homography,
        'SUGARBOX_RL_MODEL': ROOT / 'integration/nav_best_model/doorway_ft_final.zip',
        'SUGARBOX_VECNORMALIZE': ROOT / 'integration/nav_best_model/doorway_ft_final_vecnormalize.pkl',
        'GSTREAMER_LAUNCH_EXE': args.gstreamer,
    }
    for name, path in paths.items():
        if path is None or not Path(path).is_file():
            issues.append('missing {}: {}'.format(name, path))
    if args.homography is not None and Path(args.homography).is_file():
        try:
            load_ground_calibration(args.homography)
        except (OSError, RuntimeError, ValueError) as exc:
            issues.append('invalid rear-camera calibration: ' + str(exc))
    for name, expected in MODEL_HASHES.items():
        path = ROOT / 'integration/nav_best_model' / name
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            issues.append('avoidance model checksum mismatch: ' + name)
    # Both models have one class; the wrong file would chase the wrong box quietly.
    if (args.target == 'placement' and PLACEMENT_MODEL.is_file()
            and hashlib.sha256(PLACEMENT_MODEL.read_bytes()).hexdigest() != PLACEMENT_MODEL_SHA256):
        issues.append('placement model checksum mismatch: ' + str(PLACEMENT_MODEL))
    if getattr(args, 'turn_to_box', False):
        try:
            dock.load_box_map(args.box_map)
        except (OSError, ValueError) as exc:
            issues.append('placement box map: ' + str(exc))
    for module in ('cv2', 'numpy', 'torch', 'gymnasium', 'roslibpy', 'stable_baselines3', 'ultralytics'):
        if importlib.util.find_spec(module) is None:
            issues.append('missing Python dependency: ' + module)
    return paths, issues


def find_gstreamer():
    supplied = os.environ.get('GSTREAMER_LAUNCH_EXE') or shutil.which('gst-launch-1.0')
    if supplied:
        return Path(supplied)
    # The wheel's own binary, not its Scripts launcher: on 2026-10-12 not one UDP
    # byte reached a pipeline started through the launcher, while the binary
    # received and decoded the rear stream (plugins found) on the same port.
    for candidate in (Path(sys.prefix) / 'Lib/site-packages/gstreamer_cli/bin/gst-launch-1.0.exe',
                      Path(sys.prefix) / 'Scripts/gst-launch-1.0.exe'):
        if candidate.is_file():
            return candidate
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='10.16.224.252')
    assets = resolve_asset_paths(str(ROOT))
    parser.add_argument('--sam', type=Path, default=assets['sam'])
    parser.add_argument('--homography', type=Path, default=assets['homography'])
    parser.add_argument('--gstreamer', type=Path, default=find_gstreamer())
    parser.add_argument('--final-travel-m', type=float, default=0.15)
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--dry-run', action='store_true', help='live vision/ROS/PPO, no arm or motor commands')
    parser.add_argument('--i-am-beside-the-robot', action='store_true')
    parser.add_argument('--approach-timeout', type=float, default=120)
    parser.add_argument('--blind-stop-x', type=float, default=None,
                        help='finish blind to this remembered forward x (m) when the gripper '
                             'hides the box; unset keeps the original search')
    parser.add_argument('--yolo-imgsz', type=int, default=None,
                        help='rear-camera YOLO input size (default 1280 with --approach enoch, '
                             '640 with legacy)')
    parser.add_argument('--handoff-m', type=float, default=1.00,
                        help='enoch: box distance from the chassis centre where the policy '
                             'hands over to the slow steer-in (m, 0.60..1.20)')
    parser.add_argument('--final-x-m', type=float, default=None,
                        help='enoch: stop when the box is this far ahead of the chassis '
                             'centre (m; sugarbox 0.25..0.45, default 0.28; placement box '
                             'near edge 0.18..0.45, default {:.3f})'.format(
                                 dock.PLACEMENT_FINAL_X_M))
    parser.add_argument('--obs-log', type=Path, default=None,
                        help='append every policy observation/action as JSON lines '
                             '(for replaying a run offline)')
    parser.add_argument('--approach', choices=('enoch', 'legacy'), default='enoch',
                        help="enoch: training-matched PPO drives to the box on its own and "
                             "may reverse slowly (2026-10-12 primary); legacy: centring, "
                             "fixed final 15 cm and optional blind finish")
    parser.add_argument('--target', choices=('sugarbox', 'placement'), default='sugarbox',
                        help='sugarbox: approach and grasp (E1). placement: carry the held '
                             'object to the placement box and release it over the box')
    parser.add_argument('--empty-test', action='store_true',
                        help='placement: rehearse with an empty jaw (graspctl release-preview)')
    parser.add_argument('--turn-to-box', action='store_true',
                        help='placement: first turn toward the box using the resident AMCL '
                             'pose and --box-map, and drive at its map position until the '
                             'camera finds it (only when the map entry is verified)')
    parser.add_argument('--box-map', type=Path, default=dock.BOX_MAP)
    parser.add_argument('--handoff-only', action='store_true',
                        help='sugarbox: skip the approach and only run E1 (look, fine-adjust, '
                             'grasp) from where the robot stands, e.g. after an arm-camera '
                             'dropout stopped a run at the box')
    parser.add_argument('--wall-m', type=float, default=None,
                        help='placement: taught LiDAR distance to the wall behind the box at '
                             'the right release spot (placement_dock.py --measure-wall); the '
                             'stop is corrected to it before releasing')
    args = parser.parse_args(argv)
    placement = args.target == 'placement'
    if args.final_x_m is None:
        args.final_x_m = dock.PLACEMENT_FINAL_X_M if placement else 0.28
    final_min = 0.18 if placement else 0.25
    if args.real and args.dry_run:
        parser.error('--real and --dry-run are mutually exclusive')
    if args.approach == 'enoch' and args.blind_stop_x is not None:
        parser.error('--blind-stop-x only applies to --approach legacy')
    if args.blind_stop_x is not None and not (
            math.isfinite(args.blind_stop_x) and 0.05 <= args.blind_stop_x <= 0.60):
        parser.error('--blind-stop-x must be 0.05..0.60 m')
    if not (math.isfinite(args.handoff_m) and math.isfinite(args.final_x_m)
            and 0.60 <= args.handoff_m <= 1.20 and final_min <= args.final_x_m <= 0.45
            and args.handoff_m >= args.final_x_m + 0.15):
        parser.error('--handoff-m 0.60..1.20 and --final-x-m {:.2f}..0.45, at least 0.15 m '
                     'apart'.format(final_min))
    if args.yolo_imgsz is not None and (args.yolo_imgsz % 32 or not 320 <= args.yolo_imgsz <= 1920):
        parser.error('--yolo-imgsz must be a multiple of 32 in 320..1920')
    if not placement and (args.empty_test or args.turn_to_box or args.wall_m is not None):
        parser.error('--empty-test, --turn-to-box and --wall-m are for --target placement')
    if args.handoff_only and (placement or args.approach != 'enoch'):
        parser.error('--handoff-only is for --target sugarbox with --approach enoch')
    if placement and args.approach != 'enoch':
        parser.error('--target placement needs --approach enoch (steer-in and arrival report)')
    if args.wall_m is not None and not (math.isfinite(args.wall_m) and 0.20 <= args.wall_m <= 1.50):
        parser.error('--wall-m must be 0.20..1.50 m')
    ipaddress.IPv4Address(args.host)  # fixed-format SSH target, no shell interpolation
    if (not math.isfinite(args.final_travel_m) or not 0 < args.final_travel_m <= 0.15
            or not math.isfinite(args.approach_timeout) or not 0 < args.approach_timeout <= 300):
        parser.error('invalid distance/timeout')
    paths, issues = preflight(args)
    try:
        status = ctl(args.host, 'status')
        angles = status.get('servo_deg')
        if not angles or len(angles) != 6 or not all(math.isfinite(v) for v in angles):
            issues.append('servo readings unavailable')
        elif not placement and angles[5] > 50 and args.real:
            issues.append('jaw may hold an object; release and confirm empty before approach')
        elif placement and args.empty_test and angles[5] > 50:
            issues.append('--empty-test needs an empty, open jaw')
        chassis = status.get('chassis', {})
        if chassis.get('moving') is not False or chassis.get('arm_busy') is not False:
            issues.append('robot is busy')
        if chassis.get('maintenance') is True:
            issues.append('maintenance lock is on (ncu\'s navigation sets it when it ends); '
                          'mission_loop.py unlocks it, or send maintenance-unlock')
        if status.get('odom', {}).get('valid') is not True:
            issues.append('odometry unavailable')
        if placement and not args.empty_test and args.real:
            if ctl(args.host, 'health').get('holding') is not True:
                issues.append('placement needs a held object (or --empty-test)')
    except Exception as exc:
        issues.append('robot preflight: ' + str(exc))
    for issue in issues:
        print('[BLOCKED] ' + issue)
    if issues:
        return 2
    env = os.environ.copy()
    env.update({key: str(value.resolve()) for key, value in paths.items()})
    env.update(ROUTE_B_IP=args.host, SUGARBOX_FINAL_TRAVEL_M=str(args.final_travel_m))
    # Explicit flags replace anything inherited from the shell, so a run's settings
    # are the ones on its own command line.
    for key in ('SUGARBOX_BLIND_STOP_X', 'SUGARBOX_YOLO_IMGSZ', 'SUGARBOX_TRAINING_CONTRACT',
                'SUGARBOX_CONTRACT_ALLOW_DRIVE', 'SUGARBOX_TARGET_INITIAL_ONLY',
                'SUGARBOX_OBS_LOG', 'SUGARBOX_CONTRACT_HANDOFF_M',
                'SUGARBOX_CONTRACT_FINAL_X_M', 'SUGARBOX_TARGET', 'SUGARBOX_CONTRACT_AIM_Y_M',
                'SUGARBOX_SEED_TARGET', 'SUGARBOX_ARRIVAL_REPORT', 'SUGARBOX_FAR_SEED'):
        env.pop(key, None)
    if args.blind_stop_x is not None:
        env['SUGARBOX_BLIND_STOP_X'] = str(args.blind_stop_x)
    if args.obs_log is not None:
        env['SUGARBOX_OBS_LOG'] = str(args.obs_log.resolve())
    # The 2026-10-12 rear calibration reaches 1.40 m; 640 misses the sugarbox beyond
    # ~1 m. The placement box is three times wider and its model was trained at 640.
    yolo_imgsz = args.yolo_imgsz or (1280 if args.approach == 'enoch' and not placement else 640)
    env['SUGARBOX_YOLO_IMGSZ'] = str(yolo_imgsz)
    aim_y = dock.PLACEMENT_AIM_Y_M if placement else None
    arrival_report = Path(tempfile.gettempdir()) / 'sugarbox_arrival_{}.json'.format(os.getpid())
    if placement:
        # The placement box sits in a corner, often beyond the calibrated 1.40 m after
        # a grasp: drive at where the camera roughly sees it (never stop on that).
        env.update(SUGARBOX_TARGET='placement', SUGARBOX_CONTRACT_AIM_Y_M=str(aim_y),
                   SUGARBOX_ARRIVAL_REPORT=str(arrival_report), SUGARBOX_FAR_SEED='1')
    if args.approach == 'enoch':
        # enoch's training-matched inputs and shaping. His "first sighting locks the
        # target" switch is deliberately left off: it was a one-off diagnostic that did
        # not cure the oscillation, and it leaves odometry slip uncorrected (11 cm of
        # sideways error on 2026-10-12). The box is re-measured while visible.
        env['SUGARBOX_TRAINING_CONTRACT'] = '1'
        env['SUGARBOX_CONTRACT_HANDOFF_M'] = str(args.handoff_m)
        env['SUGARBOX_CONTRACT_FINAL_X_M'] = str(args.final_x_m)
        if args.real:
            env['SUGARBOX_CONTRACT_ALLOW_DRIVE'] = '1'
    print('[config] target={} approach={} blind_stop_x={} yolo_imgsz={} final_travel_m={} '
          'handoff_m={} final_x_m={}{}'.format(
              args.target, args.approach, args.blind_stop_x, yolo_imgsz, args.final_travel_m,
              args.handoff_m, args.final_x_m,
              '' if aim_y is None else ' aim_y_m={} wall_m={} empty_test={}'.format(
                  aim_y, args.wall_m, args.empty_test)))
    if args.dry_run:
        env.update(SUGARBOX_RUN_MODE='DRY_RUN', SUGARBOX_EXIT_ON_ARRIVAL='0')
        if args.turn_to_box:
            # No turning in a dry run: show where the map says the box is from here.
            try:
                box = dock.load_box_map(args.box_map)
                pose = dock.localization_pose(dock.read_localization(args.host))
                bearing, distance = dock.box_bearing(pose, box)
                print('[FACE] dry run: placement box {:.2f} m away, bearing {:+.1f} deg '
                      '(the real run turns first)'.format(distance, math.degrees(bearing)))
                if box['verified'] and abs(bearing) <= math.radians(60):
                    env['SUGARBOX_SEED_TARGET'] = '{:.4f},{:.4f}'.format(*dock.seed_point(pose, box))
            except (OSError, ValueError, RuntimeError) as exc:
                print('[FACE] dry run without a map estimate: ' + str(exc))
        return subprocess.run([sys.executable, '-u', str(APPROACH)], env=env).returncode
    if not args.real:
        print('[check] files, model hashes, Python imports and status passed; no motion. '
              'Camera/ROS live dry-run is still required before the first real run.')
        return 0
    if not args.i_am_beside_the_robot:
        print('[REFUSED] --real requires --i-am-beside-the-robot')
        return 3
    env.update(SUGARBOX_RUN_MODE='DRIVE',
               SUGARBOX_EXIT_ON_ARRIVAL='1', SUGARBOX_FINAL_TRAVEL_M=str(args.final_travel_m))
    outcome = 'nothing released' if placement else 'no E1/grasp'
    try:
        stop(args.host)
        time.sleep(0.6)
        if args.handoff_only:
            # handoff() settles, raises the arm to E1 and may fine-adjust from there.
            print(json.dumps(handoff(args.host, look=look, drive=drive), ensure_ascii=False))
            return 0
        ctl(args.host, 'stow')       # after a grasp the held object comes home to travel
        st = ctl(args.host, 'status')
        if st.get('chassis', {}).get('arm_pose') != 'travel':
            raise RuntimeError('travel pose not confirmed')
        if args.turn_to_box:
            box = dock.load_box_map(args.box_map)
            pose = face_box(args.host, box)
            if box['verified']:
                env['SUGARBOX_SEED_TARGET'] = '{:.4f},{:.4f}'.format(*dock.seed_point(pose, box))
            else:
                print('[FACE] placement_box_map.json is not verified: turned toward it only; '
                      'the camera has to find the box from here')
        if arrival_report.exists():
            arrival_report.unlink()
        result = subprocess.run([sys.executable, '-u', str(APPROACH)], env=env,
                                timeout=args.approach_timeout)
        if result.returncode == 4:
            raise RuntimeError('approach stopped: no progress toward the box (the [STOPPED] '
                               'line above says what held it); ' + outcome)
        if result.returncode:
            raise RuntimeError('approach did not report stopped arrival; ' + outcome)
        if placement:
            try:
                report = json.loads(arrival_report.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                report = None
            print('[ARRIVAL] ' + json.dumps(report))
            print(json.dumps(place(args.host, report, args.final_x_m, aim_y, args.empty_test,
                                   wall_m=args.wall_m), ensure_ascii=False))
            return 0
        # Only enoch's approach may reverse, so only it gets the E1 fine adjustment.
        tools = dict(look=look, drive=drive) if args.approach == 'enoch' else {}
        print(json.dumps(handoff(args.host, **tools), ensure_ascii=False))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print('[STOP] ' + str(exc))
        return 2
    finally:
        try:
            stop(args.host)
        except OSError as exc:
            print('[STOP] could not deliver final stop: {}; resident watchdog remains active'.format(exc))


if __name__ == '__main__':
    raise SystemExit(main())
