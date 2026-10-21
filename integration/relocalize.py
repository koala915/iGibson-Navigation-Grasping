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


def registration_id(host):
    result = ssh(host, 'cat {}/{}'.format(JETSON_ROOT, LANDMARKS), 20)
    try:
        return json.loads(result.stdout)['registration_id']
    except (ValueError, KeyError, TypeError):
        raise RelocalizeError('marker registration unreadable on the robot: ' + LANDMARKS)


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


def explain(report):
    status = report.get('status')
    reason = report.get('reason') or ''
    markers = report.get('stable_marker_ids')
    if status == 'AMBIGUOUS':
        return ('AMBIGUOUS: markers {} fit more than one place in the room (ID 0 sits on '
                'both symmetric walls). Turn the robot by hand until the rear camera sees '
                'another marker, then start again'.format(markers))
    if status == 'NO_MATCH':
        return ('NO_MATCH: no wall hypothesis agrees with markers {}; check that the rear '
                'camera sees a marker and the walls are as mapped'.format(markers))
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


def relocalize(host, log=print):
    """Seed AMCL from the rear camera's markers and a LiDAR snapshot; the map pose.

    Raises RelocalizeError, with what to do, on anything but a clean LOCALIZED.
    """
    started = time.monotonic()
    free_rear_camera(host)
    registration = registration_id(host)
    batch, _ = capture(host, registration, log)
    log('[RELOC] matching the walls on the robot (~40 s)')
    match(host)
    report = seed(host, batch)
    if report.get('status') != 'LOCALIZED' or report.get('seed_written') is not True:
        raise RelocalizeError(explain(report))
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
    args = parser.parse_args(argv)
    try:
        relocalize(args.host)
    except (RelocalizeError, OSError, subprocess.SubprocessError) as exc:
        print('[RELOC] not localized: {}'.format(exc))
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
