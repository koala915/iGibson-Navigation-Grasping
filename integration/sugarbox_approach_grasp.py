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


def handoff(host, read=ctl, send_stop=stop, sleep=time.sleep, now=time.monotonic):
    send_stop(host)
    deadline = now() + 8
    while not stationary(read(host, 'status')):
        if now() >= deadline:
            raise RuntimeError('chassis stop not confirmed; no E1 motion')
        sleep(0.5)
    read(host, 'home')
    # Discard travel-pose detections: the E1 camera runs at 1 Hz.
    sleep(2.5)
    previous = None
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
        sleep(1.1)
    raise RuntimeError('no stable fresh E1 target; stopped without grasp or blind creep')


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
    # The official wheel's Scripts launcher also sets its DLL/plugin paths.
    bundled = Path(sys.prefix) / 'Scripts/gst-launch-1.0.exe'
    return bundled if bundled.is_file() else None


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
    args = parser.parse_args(argv)
    if args.real and args.dry_run:
        parser.error('--real and --dry-run are mutually exclusive')
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
        if result.returncode:
            raise RuntimeError('approach did not report stopped arrival; no E1/grasp')
        print(json.dumps(handoff(args.host), ensure_ascii=False))
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
