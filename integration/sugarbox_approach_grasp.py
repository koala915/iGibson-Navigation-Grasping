#!/usr/bin/env python3
"""Windows orchestration: supervised avoidance approach -> stopped E1 grasp.

Default is a read-only preflight. The Jetson resident service remains the only
serial owner. This runner never releases an object or starts a second motor server.
"""
import argparse
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from integration.sugarbox_ground_calibration import load_ground_calibration, resolve_asset_paths

APPROACH = ROOT / 'integration/sugarbox_rl_approach_final2.py'
REMOTE_CTL = '/home/jetson/Documents/deploy_jetson2/grasp/v23/graspctl.py'
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
MODEL_HASHES = {
    'doorway_ft_final.zip': 'af5e9e1d5ce0115de8b4bbb321873c6a99686c82c0c69d060f881bbe97b64f35',
    'doorway_ft_final_vecnormalize.pkl': '3cd4ef9ab5b0fa38058b5142b1cc5b5df1ba9c440f325ff81b88d5e6ca7a8cdf',
}


def ctl(host, command):
    if command not in ('status', 'stow', 'home', 'grasp'):
        raise ValueError('unsupported arm command')
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
         'jetson@' + host, 'python3 ' + REMOTE_CTL + ' ' + command],
        capture_output=True, text=True, timeout=200 if command == 'grasp' else 75)
    if result.returncode:
        raise RuntimeError('graspctl {} failed: {}'.format(command, result.stdout + result.stderr))
    value = json.loads(result.stdout)
    if value.get('ok') is not True:
        raise RuntimeError('graspctl refused: ' + str(value))
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


def drive(host, metres):
    """Straight move measured by wheel odometry; the arm must already be in travel.

    The stop goes out ADJUST_LEAD_M early: on 2026-10-16 a 3.6 cm creep at 0.08 m/s
    sent its stop 0.5 cm late (odometry latency) and coasted another 0.8 cm.
    """
    import roslibpy
    if not (math.isfinite(metres) and 0 < abs(metres) <= ADJUST_MAX_M):
        raise ValueError('refusing adjustment of {!r} m'.format(metres))
    goal = max(abs(metres) - ADJUST_LEAD_M, 0.003)
    command = json.dumps({'action': 'velocity', 'vx': math.copysign(0.08, metres),
                          'wz': 0.0}).encode() + b'\n'
    ros = roslibpy.Ros(host=host, port=9090)
    ros.run()
    latest = {}
    odom = roslibpy.Topic(ros, '/odom_setmotor', 'nav_msgs/Odometry')
    odom.subscribe(lambda m: latest.update(p=m['pose']['pose']['position'], t=time.monotonic()))
    try:
        started = time.monotonic()
        while 'p' not in latest:
            if time.monotonic() - started > 5:
                raise RuntimeError('no odometry; wheels not moved')
            time.sleep(0.05)
        origin = dict(latest['p'])

        def moved():
            return math.hypot(latest['p']['x'] - origin['x'], latest['p']['y'] - origin['y'])

        with socket.create_connection((host, 7000), timeout=3) as sock:
            try:
                started = time.monotonic()
                while moved() < goal:
                    if time.monotonic() - started > 2.0:
                        raise RuntimeError('adjustment did not finish in 2 s')
                    if time.monotonic() - latest['t'] > 0.3:
                        raise RuntimeError('odometry went stale while driving')
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
        odom.unsubscribe()
        ros.close()


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
    ADJUST_TRIES times. Only an accepted reading is ever grasped.
    """
    adjustments = 0
    while True:
        settle(host, read, send_stop, sleep, now)
        read(host, 'home')
        # Discard travel-pose detections: the E1 camera runs at 1 Hz.
        sleep(2.5)
        previous = None
        move = None
        hints = []
        deadline = now() + 12
        while now() < deadline:
            status = read(host, 'status')
            if not stationary(status) or status.get('chassis', {}).get('arm_pose') != 'e1':
                raise RuntimeError('E1/stationary state lost; no grasp')
            pos = target_position(status)
            if (pos is not None and previous is not None
                    and math.hypot(pos[0] - previous[0], pos[1] - previous[1]) <= 0.008):
                result = read(host, 'grasp')
                if result.get('confirmed') is not True:
                    raise RuntimeError('grasp was not confirmed')
                return result
            previous = pos
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
            else:
                hints = []
            sleep(1.1)
        if move is None:
            raise RuntimeError('no stable fresh E1 target; stopped without grasp or blind creep')
        adjustments += 1
        print('[ADJUST] {}/{}: E1 sees the box outside its grasp band; {} {:.3f} m'.format(
            adjustments, ADJUST_TRIES, 'forward' if move > 0 else 'back', abs(move)), flush=True)
        read(host, 'stow')
        status = read(host, 'status')
        if status.get('chassis', {}).get('arm_pose') != 'travel' or not stationary(status):
            raise RuntimeError('travel pose not confirmed; wheels not moved')
        drive(host, move)


def preflight(args):
    issues = []
    paths = {
        'SUGARBOX_YOLO_MODEL': ROOT / 'detection/models/best.pt',
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
    parser.add_argument('--final-x-m', type=float, default=0.28,
                        help='enoch: stop when the box is this far ahead of the chassis '
                             'centre (m, 0.25..0.45)')
    parser.add_argument('--obs-log', type=Path, default=None,
                        help='append every policy observation/action as JSON lines '
                             '(for replaying a run offline)')
    parser.add_argument('--approach', choices=('enoch', 'legacy'), default='enoch',
                        help="enoch: training-matched PPO drives to the box on its own and "
                             "may reverse slowly (2026-10-12 primary); legacy: centring, "
                             "fixed final 15 cm and optional blind finish")
    args = parser.parse_args(argv)
    if args.real and args.dry_run:
        parser.error('--real and --dry-run are mutually exclusive')
    if args.approach == 'enoch' and args.blind_stop_x is not None:
        parser.error('--blind-stop-x only applies to --approach legacy')
    if args.blind_stop_x is not None and not (
            math.isfinite(args.blind_stop_x) and 0.05 <= args.blind_stop_x <= 0.60):
        parser.error('--blind-stop-x must be 0.05..0.60 m')
    if not (math.isfinite(args.handoff_m) and math.isfinite(args.final_x_m)
            and 0.60 <= args.handoff_m <= 1.20 and 0.25 <= args.final_x_m <= 0.45
            and args.handoff_m >= args.final_x_m + 0.15):
        parser.error('--handoff-m 0.60..1.20 and --final-x-m 0.25..0.45, at least 0.15 m apart')
    if args.yolo_imgsz is not None and (args.yolo_imgsz % 32 or not 320 <= args.yolo_imgsz <= 1920):
        parser.error('--yolo-imgsz must be a multiple of 32 in 320..1920')
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
        elif angles[5] > 50 and args.real:
            issues.append('jaw may hold an object; release and confirm empty before approach')
        chassis = status.get('chassis', {})
        if chassis.get('moving') is not False or chassis.get('arm_busy') is not False:
            issues.append('robot is busy')
        if status.get('odom', {}).get('valid') is not True:
            issues.append('odometry unavailable')
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
                'SUGARBOX_CONTRACT_FINAL_X_M'):
        env.pop(key, None)
    if args.blind_stop_x is not None:
        env['SUGARBOX_BLIND_STOP_X'] = str(args.blind_stop_x)
    if args.obs_log is not None:
        env['SUGARBOX_OBS_LOG'] = str(args.obs_log.resolve())
    # The 2026-10-12 rear calibration reaches 1.40 m; 640 misses the box beyond ~1 m.
    yolo_imgsz = args.yolo_imgsz or (1280 if args.approach == 'enoch' else 640)
    env['SUGARBOX_YOLO_IMGSZ'] = str(yolo_imgsz)
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
    print('[config] approach={} blind_stop_x={} yolo_imgsz={} final_travel_m={} '
          'handoff_m={} final_x_m={}'.format(args.approach, args.blind_stop_x, yolo_imgsz,
                                             args.final_travel_m, args.handoff_m,
                                             args.final_x_m))
    if args.dry_run:
        env.update(SUGARBOX_RUN_MODE='DRY_RUN', SUGARBOX_EXIT_ON_ARRIVAL='0')
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
    try:
        stop(args.host)
        time.sleep(0.6)
        ctl(args.host, 'stow')
        st = ctl(args.host, 'status')
        if st.get('chassis', {}).get('arm_pose') != 'travel':
            raise RuntimeError('travel pose not confirmed')
        result = subprocess.run([sys.executable, '-u', str(APPROACH)], env=env,
                                timeout=args.approach_timeout)
        if result.returncode == 4:
            raise RuntimeError('approach stopped: no progress toward the box (the [STOPPED] '
                               'line above says what held it); no E1/grasp')
        if result.returncode:
            raise RuntimeError('approach did not report stopped arrival; no E1/grasp')
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
