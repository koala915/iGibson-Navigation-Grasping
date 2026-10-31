"""Relocalize AMCL before a mission, with ncu's ArUco corner seed, all on the robot.

Run it when the robot may not be where AMCL thinks (set down by hand, a new
day), on its own or as mission_loop.py --relocalize; the user (2026-10-21) asks
for it rather than paying a minute at every start. That day a seed written
earlier, from a marker that sits on both of two symmetric walls, most likely had
the map turned around: the patrol and the grasp were fine (the walls look the
same either way), and the turn toward the placement box went the wrong way.

With the robot standing still and the rear camera free:
  1. deploy/ros/g2_corner_camera_capture.py   rear frames + LiDAR snapshot
  2. integration/g2_scan_match.py            wall hypotheses (~40 s on the Jetson)
  3. integration/g2_aruco_corner_seed.py --real
                                             keeps the one the markers agree with
                                             and writes the seed
then waits for the localization service to report ready at that seed. Nothing
moves. Steps 2 and 3 are ncu's tools as installed on the robot (PR #5); nothing
of them is needed on this PC. Any answer but LOCALIZED stops the mission start:
AMBIGUOUS means the camera saw only markers that fit several places (ID 0 on
the symmetric walls), and a hand turn toward another marker fixes it.

    python integration/relocalize.py --host 10.53.253.252
"""
import argparse
import json
import math
import shlex
import subprocess
import time

from integration import placement_dock as dock
from integration.sugarbox_lidar_geometry import robot_scan_samples

JETSON_ROOT = dock.JETSON_ROOT
SCENE = dock.SCENE
# The map the localization service runs on (its --map); the scan match must
# use these exact bytes.
ACTIVE_MAP = 'maps/demo_box_20261017/demo_box.yaml'
LANDMARKS = SCENE + '/aruco_corner_landmarks.json'
SNAPSHOT = '/dev/shm/g2_corner_capture.json'
CANDIDATES = '/tmp/mission_corner_candidates.json'
RESULT = '/tmp/mission_corner_result.json'
VENV_PYTHON = '/home/jetson/grasp_venv/bin/python3'
REAR_CAMERA = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
# The capture refuses a camera whose first frames' timestamps are not yet
# steady; on 2026-10-20 the second try went through.
CAPTURE_TRIES = 2
READY_WAIT_S = 20.0
# The service's pose once ready has to be where the seed put it.
SEED_AGREE_M = 0.25
SEED_AGREE_RAD = math.radians(20)
# Looking around (--look, the user 2026-10-29): markers 3 and 4, both on the top
# wall, came back AMBIGUOUS in the symmetric room. Markers on two walls pin the
# map; until the capture sees that, turn in place a step and capture again.
LOOK_STEP_RAD = math.radians(30)
LOOK_MAX_STEPS = 11              # 330 deg: every heading once, no second lap
# A turn sweeps the outline's farthest corner around the centre; every LiDAR
# return outside the outline has to be beyond it by this much. Self returns are
# the body rectangles widened by ncu's 2.5 cm profile tolerance. The scan plane
# is 9.5 cm up: boxes on the floor do not show, so the operator stands by.
LOOK_SWEEP_M = max(math.hypot(x, y) for r in dock.BODY_RECTANGLES
                   for x in (r[0], r[2]) for y in (r[1], r[3]))
LOOK_MARGIN_M = 0.05
SELF_RETURN_TOLERANCE_M = 0.025
# Returns outside ncu's outline that stayed put in the robot frame at two places
# in the arena, with the operator stepped back (2026-10-29): beside the right
# wheel, and two side spots ~0.32 m out just behind the centre. Small boxes only:
# a wall that near still shows at the neighbouring angles (r = d / cos) and refuses.
LOOK_SELF_RETURNS = ((-0.02, -0.185, 0.00, -0.165), (-0.09, 0.28, -0.03, 0.34),
                     (0.02, 0.32, 0.04, 0.35), (-0.06, -0.35, 0.06, -0.31))


class RelocalizeError(RuntimeError):
    pass


def ssh(host, command, timeout):
    return subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                           '-o', 'LogLevel=ERROR', 'jetson@' + host, command],
                          capture_output=True, text=True, timeout=timeout)


def last_json(text):
    """The tools print one JSON report; take the last line that parses."""
    try:
        return json.loads(text)
    except ValueError:
        pass
    for line in reversed(text.strip().splitlines()):
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict):
            return value
    return None


def free_rear_camera(host, wait_s=5.0):
    """The capture opens the rear camera itself: end this PC's stream first.
    The mission starts it again afterwards."""
    ssh(host, "pkill -INT -f '[u]dpsink host='", 15)
    deadline = time.monotonic() + wait_s
    while ssh(host, 'fuser {} >/dev/null 2>&1'.format(REAR_CAMERA), 15).returncode == 0:
        if time.monotonic() > deadline:
            raise RelocalizeError('the rear camera is still in use; stop its reader and try again')
        time.sleep(0.5)


def read_landmarks(host):
    result = ssh(host, 'cat {}/{}'.format(JETSON_ROOT, LANDMARKS), 20)
    try:
        landmarks = json.loads(result.stdout)
        landmarks['registration_id']
        return landmarks
    except (ValueError, KeyError, TypeError):
        raise RelocalizeError('marker registration unreadable on the robot: ' + LANDMARKS)


def registration_id(host):
    return read_landmarks(host)['registration_id']


def marker_walls(ids, landmarks):
    """How many walls the seen markers are on (markers on one wall face one way)."""
    markers = landmarks.get('markers') or {}
    faces = set()
    for marker in ids or ():
        normal = (markers.get(str(marker)) or {}).get('normal_yaw_rad')
        if isinstance(normal, (int, float)) and math.isfinite(normal):
            faces.add(round(math.degrees(dock.wrap(normal))) % 360)
    return len(faces)


def turn_room(scans, cell=0.02, persist=2):
    """Nearest LiDAR return outside the robot's own outline, from the chassis centre.

    A return counts once `persist` scans hit the same 2 cm cell: on 2026-10-29
    single-scan returns at (-0.01..-0.02, -0.17..-0.18), each in 1 of 8 scans
    beside the right wheel, refused every turn. A wall or a person is in all.
    """
    pad = SELF_RETURN_TOLERANCE_M
    seen = {}
    for scan in scans:
        cells = set()
        ranges = [float('nan') if r is None else r for r in scan['ranges']]
        for sample in robot_scan_samples(ranges, scan['angle_min'], scan['angle_increment'],
                                         dock.LIDAR):
            if sample.range_m is None:
                continue
            x = sample.range_m * math.cos(sample.angle_rad)
            y = sample.range_m * math.sin(sample.angle_rad)
            if any(x1 - pad <= x <= x2 + pad and y1 - pad <= y <= y2 + pad
                   for x1, y1, x2, y2 in dock.BODY_RECTANGLES + LOOK_SELF_RETURNS):
                continue
            key =(int(math.floor(x / cell)), int(math.floor(y / cell)))
            if key not in cells:
                cells.add(key)
                hits, nearest = seen.get(key, (0, float('inf')))
                seen[key] = (hits + 1, min(nearest, sample.range_m))
    return min((nearest for hits, nearest in seen.values() if hits >= persist),
               default=float('inf'))


def robot_look(host, step=LOOK_STEP_RAD):
    """A look(log) that turns the robot in place by `step` (right), or refuses.

    The turn itself is the approach runner's: odometry-measured, direction and
    stall checks, stop on every exit. Before it: travel pose, nothing else on the
    chassis, and LiDAR room for the sweep. Localization is not asked: it is
    exactly what is in doubt here.
    """
    from integration import sugarbox_approach_grasp as runner

    def look(log):
        chassis = runner.ctl(host, 'status').get('chassis', {})
        if (chassis.get('arm_pose') != 'travel' or chassis.get('moving') is not False
                or chassis.get('client') is not None):
            raise RelocalizeError('not turning to look: the arm is not in travel or the chassis '
                                  'is busy ({})'.format({k: chassis.get(k) for k in
                                                         ('arm_pose', 'moving', 'client')}))
        if chassis.get('maintenance') is True:
            runner.service_command(host, 'maintenance-unlock')
        room = turn_room(dock.read_scans(host, count=5))
        if room < LOOK_SWEEP_M + LOOK_MARGIN_M:
            raise RelocalizeError(
                'not turning to look: something is {:.2f} m from the centre and a turn needs '
                '{:.2f} m; move the robot toward the middle (or step back from it)'.format(
                    room, LOOK_SWEEP_M + LOOK_MARGIN_M))
        turned = runner.turn(host, -step)
        log('[RELOC] turned {:.0f} deg to look for a marker on another wall'.format(
            math.degrees(turned)))
    return look


def capture(host, registration, log=print):
    for attempt in range(1, CAPTURE_TRIES + 1):
        batch = '/tmp/mission_corner_{}'.format(time.strftime('%H%M%S'))
        result = ssh(host, 'cd {} && {} deploy/ros/g2_corner_camera_capture.py --output-dir {} '
                           '--registration-id {}'.format(JETSON_ROOT, VENV_PYTHON, batch,
                                                         shlex.quote(registration)), 60)
        report = last_json(result.stdout)
        if result.returncode == 0 and report and report.get('ok') is True:
            log('[RELOC] captured: markers {}'.format(report.get('ids')))
            return batch, report
        reason = (result.stderr.strip().splitlines() or ['no output'])[-1]
        log('[RELOC] capture {}/{} failed: {}'.format(attempt, CAPTURE_TRIES, reason))
    raise RelocalizeError('rear camera capture failed {} times: {}'.format(CAPTURE_TRIES, reason))


def match(host):
    result = ssh(host, 'cd {} && /usr/bin/python3 integration/g2_scan_match.py --snapshot {} '
                       '--scene {} --reference-map-yaml {} --output {}'.format(
                           JETSON_ROOT, SNAPSHOT, SCENE, ACTIVE_MAP, CANDIDATES), 240)
    if result.returncode:
        raise RelocalizeError('scan match failed: ' + (
            result.stderr.strip().splitlines() or ['no output'])[-1])


def seed(host, batch):
    result = ssh(host, 'cd {} && {} integration/g2_aruco_corner_seed.py --snapshot {} '
                       '--candidates {} --observations {}/observations.json --landmarks {} '
                       '--output {} --real'.format(JETSON_ROOT, VENV_PYTHON, SNAPSHOT, CANDIDATES,
                                                  batch, LANDMARKS, RESULT), 120)
    report = last_json(result.stdout)
    if not report:
        raise RelocalizeError('corner seed gave no report: ' + (
            result.stderr.strip().splitlines() or ['no output'])[-1])
    return report


def explain(report, seen=None):
    """What went wrong and what to do; `seen`: the marker IDs the capture found
    (the corner seed reports its own only when it localizes)."""
    status = report.get('status')
    reason = report.get('reason') or ''
    markers = report.get('stable_marker_ids') or seen
    if status == 'AMBIGUOUS':
        return ('AMBIGUOUS: markers {} fit more than one place in the room (the walls are '
                'symmetric). Turn or move the robot by hand until the rear camera sees '
                'markers on two different walls, then start again'.format(markers))
    if status == 'NO_MATCH':
        return ('NO_MATCH: no wall hypothesis agrees with markers {}; keep people and '
                'objects away from the robot and use a view with verified markers '
                '(2 and 7 from the start corner)'.format(markers))
    return '{}: {}'.format(status, reason)


def wait_ready(host, seeded, wait_s=None, read=None, sleep=None, now=None):
    wait_s = READY_WAIT_S if wait_s is None else wait_s
    read, sleep = read or dock.read_localization, sleep or time.sleep
    now = now or time.monotonic
    deadline = now() + wait_s
    while True:
        try:
            pose = dock.localization_pose(read(host))
        except ValueError as exc:
            problem = str(exc)
        else:
            off = math.hypot(pose[0] - seeded[0], pose[1] - seeded[1])
            turned = abs(dock.wrap(pose[2] - seeded[2]))
            if off <= SEED_AGREE_M and turned <= SEED_AGREE_RAD:
                return pose
            problem = 'pose ({:.2f}, {:.2f}) is {:.2f} m / {:.0f} deg from the seed'.format(
                pose[0], pose[1], off, math.degrees(turned))
        if now() >= deadline:
            break
        sleep(0.5)
    raise RelocalizeError('localization not ready at the new seed after {:.0f} s: {}'.format(
        wait_s, problem))


def relocalize(host, log=print, look=None):
    """Seed AMCL from the rear camera's markers and a LiDAR snapshot; the map pose.

    With `look` (robot_look): while the capture sees markers on fewer than two
    walls, turn in place a step and capture again, up to LOOK_MAX_STEPS; then
    match whatever the last view has. Without it nothing moves.
    Raises RelocalizeError, with what to do, on anything but a clean LOCALIZED.
    """
    started = time.monotonic()
    free_rear_camera(host)
    landmarks = read_landmarks(host)
    registration = landmarks['registration_id']
    batch, captured = capture(host, registration, log)
    if look is not None:
        for _ in range(LOOK_MAX_STEPS):
            if marker_walls(captured.get('ids'), landmarks) >= 2:
                break
            look(log)
            batch, captured = capture(host, registration, log)
        else:
            log('[RELOC] no view with markers on two walls in a full turn; matching the last one')
    log('[RELOC] matching the walls on the robot (~40 s)')
    match(host)
    report = seed(host, batch)
    if report.get('status') != 'LOCALIZED' or report.get('seed_written') is not True:
        raise RelocalizeError(explain(report, captured.get('ids')))
    seeded = (report.get('seed') or {}).get('pose') or report.get('pose')
    if not (isinstance(seeded, list) and len(seeded) == 3
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in seeded)):
        raise RelocalizeError('LOCALIZED without a usable seed pose: ' + json.dumps(report)[:300])
    pose = wait_ready(host, seeded)
    log('[RELOC] localized at map ({:.2f}, {:.2f}), heading {:.0f} deg, from markers {} '
        '({:.0f} s)'.format(pose[0], pose[1], math.degrees(pose[2]),
                            report.get('stable_marker_ids'), time.monotonic() - started))
    return pose


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', required=True)
    parser.add_argument('--look', action='store_true',
                        help='turn in place until markers on two walls are in view (moves the robot)')
    parser.add_argument('--i-am-beside-the-robot', action='store_true')
    args = parser.parse_args(argv)
    if args.look and not args.i_am_beside_the_robot:
        parser.error('--look turns the robot: it needs --i-am-beside-the-robot')
    try:
        relocalize(args.host, look=robot_look(args.host) if args.look else None)
    except (RelocalizeError, OSError, subprocess.SubprocessError) as exc:
        print('[RELOC] not localized: {}'.format(exc))
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
