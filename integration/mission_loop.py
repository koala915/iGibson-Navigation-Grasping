#!/usr/bin/env python3
"""Competition loop on the Windows PC: patrol -> grasp -> place -> patrol again.

    patrol    ncu's map patrol on the Jetson (deploy/ros/g2_map_patrol.py, PR #5)
              while this PC watches the rear camera for the sugarbox
    grasp     sugarbox_approach_grasp.py --target sugarbox (avoidance approach, E1)
    place     sugarbox_approach_grasp.py --target placement --turn-to-box
    back out  straight back BACK_OUT_M at 0.08 m/s: the patrol's map mask keeps
              0.28 m around the placement box and refuses to start inside it

One velocity owner at a time: the patrol is stopped with SIGTERM (it brakes and
re-locks the service) before an approach connects, and it is never started while
one runs. Any failure stops the loop where it is and says which stage failed.

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
JETSON_ROOT = '/home/jetson/Documents/deploy_jetson2'
SCENE = 'maps/demo_box_20261018_patrol'
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
STREAM_STALE_S = 1.5
# A sugarbox this close to the placement box's map position is one already
# delivered (it shows over the 6 cm rim), not a new one to fetch.
PLACED_EXCLUDE_M = 0.35
BACK_OUT_M = 0.20
BACK_OUT_MARGIN_M = 0.10
MAX_PATROL_CYCLES = 4


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


def map_point(pose, point):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


class Confirm:
    """WATCH_FRAMES consistent sightings in a row, like the approach's lock."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.points, self.stamp = [], None

    def update(self, point, now):
        if point is None:
            if self.stamp is not None and now - self.stamp > WATCH_MAX_GAP_S:
                self.reset()
            return False
        if self.points and (now - self.stamp > WATCH_MAX_GAP_S or math.hypot(
                point[0] - self.points[-1][0], point[1] - self.points[-1][1]) > WATCH_MAX_ERROR_M):
            self.points = []
        self.points.append(point)
        self.stamp = now
        return len(self.points) >= WATCH_FRAMES

    def mean(self):
        return (sum(p[0] for p in self.points) / len(self.points),
                sum(p[1] for p in self.points) / len(self.points))


class SugarboxWatcher:
    """Rear-camera YOLO for the sugarbox while the patrol drives."""

    def __init__(self, homography_path):
        from ultralytics import YOLO
        self.model = YOLO(str(runner.SUGARBOX_MODEL))
        self.H, self.rear_cam_to_front, self.hull = load_ground_calibration(homography_path)

    def look(self, frame):
        result = self.model.predict(frame, imgsz=WATCH_IMGSZ, conf=WATCH_CONF, verbose=False)[0]
        best = None
        for box in result.boxes:
            bbox = tuple(float(v) for v in box.xyxy[0].tolist())
            if overlap_fraction(bbox, ARM_CAMERA_REGION) >= 0.5:
                continue
            point = detection_point(bbox, self.H, self.rear_cam_to_front, self.hull)
            if point is not None and (best is None or math.hypot(*point) < math.hypot(*best)):
                best = point
        return best


def patrol_until_sugarbox(host, scene, reader, watcher, box=None, now=time.monotonic):
    """Patrol (restarting finished cycles) until the camera confirms a sugarbox.

    The patrol is stopped before returning; on any error it is stopped too.
    """
    confirm, pid, cycles, last_stamp = Confirm(), None, 0, None
    deadline = now() + 8.0
    while reader.latest()[0] is None:          # never patrol blind
        if now() > deadline:
            raise MissionError('no rear camera frames; is the stream reaching this PC?')
        time.sleep(0.1)
    try:
        while True:
            if pid is None or not patrol_running(host, pid):
                if pid is not None:
                    report = patrol_report(host)
                    if not report.get('ok'):
                        raise MissionError('patrol stopped: ' + str(report.get('error')))
                cycles += 1
                if cycles > MAX_PATROL_CYCLES:
                    raise MissionError('no sugarbox in {} patrol cycles'.format(MAX_PATROL_CYCLES))
                pid = start_patrol(host, scene)
                print('[PATROL] cycle {} started (pid {})'.format(cycles, pid), flush=True)
            frame, stamp = reader.latest()
            if now() - stamp > STREAM_STALE_S:
                raise MissionError('rear camera stream lost during the patrol')
            if stamp == last_stamp:
                time.sleep(0.01)
                continue
            last_stamp = stamp
            if not confirm.update(watcher.look(frame), stamp):
                continue
            seen = confirm.mean()
            confirm.reset()
            if box is not None and box['verified']:
                try:
                    pose = dock.localization_pose(dock.read_localization(host))
                    where = map_point(pose, seen)
                    if math.hypot(where[0] - box['x'], where[1] - box['y']) <= PLACED_EXCLUDE_M:
                        print('[PATROL] sugarbox at the placement box: already delivered',
                              flush=True)
                        continue
                except ValueError as exc:
                    print('[PATROL] cannot place the sighting on the map ({}); taking it'
                          .format(exc), flush=True)
            print('[PATROL] sugarbox {:.2f} m ahead, {:+.2f} m left: stopping the patrol'
                  .format(*seen), flush=True)
            stop_patrol(host, pid)
            pid = None
            return seen
    finally:
        if pid is not None and patrol_running(host, pid):
            stop_patrol(host, pid)


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


def back_out(host, metres=BACK_OUT_M, scans=dock.read_scans, drive=runner.drive):
    clear = [dock.rear_clearance(m['ranges'], m['angle_min'], m['angle_increment'])
             for m in scans(host)]
    clear = sorted(clear)[len(clear) // 2]
    if clear < metres + BACK_OUT_MARGIN_M:
        raise MissionError('only {:.2f} m free behind; not backing out'.format(clear))
    drive(host, -metres, limit=0.30)


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
            box = dock.load_box_map(args.box_map)
            if not box['verified']:
                print('[WARN] placement box map not verified: the robot turns toward it but '
                      'never drives to it unseen, and a delivered sugarbox cannot be told '
                      'from a new one')
        except (OSError, ValueError) as exc:
            issues.append('placement box map: ' + str(exc))
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', required=True)
    parser.add_argument('--scene', default=SCENE, help='Jetson-side scene folder for the patrol')
    parser.add_argument('--loops', type=int, default=1, help='objects to deliver (1..5)')
    parser.add_argument('--wall-m', type=float, default=None,
                        help='taught wall distance for the placement stop (see placement_dock.py)')
    parser.add_argument('--box-map', type=Path, default=dock.BOX_MAP)
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
    if not 1 <= args.loops <= 5:
        parser.error('--loops must be 1..5')
    if args.no_patrol and args.loops != 1:
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
        print('[MISSION] PC {} | patrol {} | {} loop(s)'.format(pc_ip, args.scene, args.loops))
    if not args.real:
        print('[check] robot, cameras and files passed{}; no motion.'.format(
            '' if args.no_patrol else ', localization, box map'))
        return 0
    if not args.i_am_beside_the_robot:
        print('[REFUSED] --real requires --i-am-beside-the-robot')
        return 3
    if args.no_patrol:
        return grasp_and_place(args, pc_ip, log_dir)
    from integration.rear_stream import StreamReader
    box = dock.load_box_map(args.box_map)
    place_argv = ['--target', 'placement', '--turn-to-box', '--box-map', str(args.box_map),
                  '--real', '--i-am-beside-the-robot']
    if args.wall_m is not None:
        place_argv += ['--wall-m', str(args.wall_m)]
    stage, reader = 'START', None
    try:
        if not stream_running(args.host, pc_ip):
            start_stream(args.host, pc_ip)
        watcher = SugarboxWatcher(args.homography)
        for loop in range(1, args.loops + 1):
            stage = 'PATROL {}'.format(loop)
            reader = StreamReader(STREAM_PORT, args.gstreamer)
            patrol_until_sugarbox(args.host, args.scene, reader, watcher, box)
            reader.close()            # the approach opens its own receiver on this port
            reader = None
            stage = 'GRASP {}'.format(loop)
            runner.service_command(args.host, 'maintenance-unlock')
            code = run_stage(args.host, ['--target', 'sugarbox', '--real',
                                         '--i-am-beside-the-robot'])
            if code:
                raise MissionError('grasp runner exit {}'.format(code))
            if runner.ctl(args.host, 'health').get('holding') is not True:
                raise MissionError('grasp reported done but nothing is held')
            stage = 'PLACE {}'.format(loop)
            code = run_stage(args.host, place_argv)
            if code:
                raise MissionError('placement runner exit {}'.format(code))
            stage = 'BACK_OUT {}'.format(loop)
            back_out(args.host)
            print('[MISSION] loop {} delivered'.format(loop), flush=True)
        stage = 'DONE'
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print('[MISSION] stopped in {}: {}'.format(stage, exc), flush=True)
        return 2
    finally:
        if reader is not None:
            reader.close()
        try:
            runner.stop(args.host)
        except OSError as exc:
            print('[MISSION] final stop not delivered: {}; resident watchdog remains'.format(exc))


if __name__ == '__main__':
    raise SystemExit(main())
