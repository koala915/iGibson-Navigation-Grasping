#!/usr/bin/env python3
"""Placement-box docking: geometry and checks for the approach runner (2026-10-18).

The placement box stays put for the whole competition (the user, 2026-10-18), so
its map position and the resident AMCL pose (ncu's localization service, PR #5)
say where it is even when the rear camera cannot see it. After a grasp the runner
turns toward it, seeds the approach with it, and the rear camera takes over as
soon as it finds the box. Nothing here moves the robot; the runner owns motion.

Two tools for the person setting up the field, both read-only:

    python integration/placement_dock.py --host <IP> --where --box-ahead-m 0.50
        map position of the point 0.50 m ahead of the chassis centre (park facing
        the box, tape the distance to its centre, copy the result into
        integration/placement_box_map.json)
    python integration/placement_dock.py --host <IP> --measure-wall
        LiDAR distance to whatever is straight ahead, for --wall-m (park at the
        right release spot first, checked with graspctl release-preview)
"""
import argparse
import ipaddress
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from integration.sugarbox_lidar_geometry import LidarGeometry, robot_scan_samples

BOX_MAP = ROOT / 'integration/placement_box_map.json'
LOCALIZATION_HEALTH = '/dev/shm/g2_localization_health.json'
# ncu's tape measurement: 18 x 13.5 x 6 cm, the long side facing the field.
BOX_DEPTH_M = 0.135
# The scripted release opens the jaw about this far ahead of the chassis centre
# (FK reach 23.9 cm from the arm base). An estimate until release-preview is
# measured on the robot; the runner's --final-x-m overrides it.
RELEASE_CENTER_X_M = 0.28
# SAM measures the box's nearest bottom edge, so that edge stops half a box short.
PLACEMENT_FINAL_X_M = round(RELEASE_CENTER_X_M - BOX_DEPTH_M / 2, 3)
# The arm's centre line in the rear camera's frame: E1 y = rear y + 0.028 m.
PLACEMENT_AIM_Y_M = -0.028
# The jaw drops a ~5 cm box into an 18 cm side: 6 cm of room each way, less margin.
ARRIVAL_MAX_Y_ERROR_M = 0.04
ARRIVAL_MAX_X_OVERSHOOT_M = 0.03
LOCALIZATION_MAX_AGE_S = 1.0
TURN_TOLERANCE_RAD = math.radians(8)
SEED_MIN_M, SEED_MAX_M = 0.30, 3.00
# A taught wall distance is trusted only for small corrections: a bigger error
# means the wrong thing is ahead (or the box moved), not that the robot is off.
WALL_TOLERANCE_M = 0.015
WALL_MAX_CORRECTION_M = 0.06
# Same corridor and robust distance as the approach's front brake in enoch mode.
CORRIDOR_HALF_WIDTH_M = 0.17
CORRIDOR_PERCENTILE = 5.0
CORRIDOR_MIN_POINTS = 8
LIDAR = LidarGeometry(yaw_offset_deg=180.0, forward_offset_m=0.10)


def finite(*values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in values)


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def load_box_map(path=BOX_MAP):
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    if data.get('frame') != 'map' or not finite(data.get('x'), data.get('y')):
        raise ValueError('placement box map needs frame "map" and finite x, y: ' + str(path))
    return {'x': float(data['x']), 'y': float(data['y']),
            'verified': data.get('verified') is True, 'source': str(data.get('source', ''))}


def localization_pose(record):
    """(x, y, yaw) in the map from ncu's health file, or ValueError saying why not.

    `record` carries 'age': seconds since its stamp, on the robot's own clock.
    """
    if not isinstance(record, dict):
        raise ValueError('localization health unreadable')
    if record.get('ready') is not True:
        raise ValueError('not localized: ' + str(record.get('reason', record.get('status'))))
    age = record.get('age')
    if not finite(age) or not 0 <= age <= LOCALIZATION_MAX_AGE_S:
        raise ValueError('localization health is stale ({})'.format(age))
    pose = (record.get('state') or {}).get('pose')
    if not (isinstance(pose, list) and len(pose) == 3 and finite(*pose)):
        raise ValueError('localization pose missing')
    return tuple(float(v) for v in pose)


def relative_point(pose, point):
    """Map point -> chassis-centre frame (x forward, y left)."""
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def point_ahead(pose, metres):
    x, y, yaw = pose
    return x + metres * math.cos(yaw), y + metres * math.sin(yaw)


def box_bearing(pose, box):
    rx, ry = relative_point(pose, (box['x'], box['y']))
    return math.atan2(ry, rx), math.hypot(rx, ry)


def seed_point(pose, box):
    """Where the approach should expect the box's near edge, chassis-centre frame.

    The camera measures the edge nearest the robot, half a box short of the
    centre along the line of sight (the box is square to the robot when the
    robot faces it, which is the only time the seed is used).
    """
    rx, ry = relative_point(pose, (box['x'], box['y']))
    distance = math.hypot(rx, ry)
    if not SEED_MIN_M + BOX_DEPTH_M / 2 <= distance <= SEED_MAX_M + BOX_DEPTH_M / 2:
        raise ValueError('placement box {:.2f} m away: outside the seed range'.format(distance))
    scale = (distance - BOX_DEPTH_M / 2) / distance
    return rx * scale, ry * scale


def arrival_problem(report, final_x, aim_y):
    """None when the approach stopped where a release is safe, else why not."""
    if not isinstance(report, dict):
        return 'no arrival report from the approach'
    if report.get('arrived') is not True:
        return 'approach did not arrive: ' + str(report.get('reason'))
    reason = str(report.get('reason', ''))
    if not reason.startswith('PURSUIT_DONE'):
        # A timeout also ends the pursuit; for a grasp E1 judges it, here nothing would.
        return 'approach ended without reaching the stop distance: ' + reason
    if report.get('seeded') is not False:
        return 'the box was never seen; the stop is a map estimate'
    x, y = report.get('x'), report.get('y')
    if not finite(x, y):
        return 'arrival position missing'
    if x > final_x + 0.005 or x < final_x - ARRIVAL_MAX_X_OVERSHOOT_M:
        return 'stopped with the box edge at {:.3f} m, wanted {:.3f} m'.format(x, final_x)
    if abs(y - aim_y) > ARRIVAL_MAX_Y_ERROR_M:
        return 'box {:+.3f} m sideways of the drop line ({:+.3f} m)'.format(y, aim_y)
    return None


def robust_nearest(values):
    """Few returns: the nearest. Otherwise the 5th percentile, so one stray beam
    neither stops nor starts anything (numpy's default interpolation)."""
    if not values:
        return float('inf')
    values = sorted(values)
    if len(values) < CORRIDOR_MIN_POINTS:
        return values[0]
    rank = (len(values) - 1) * CORRIDOR_PERCENTILE / 100.0
    low = int(math.floor(rank))
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (rank - low)


def corridor_distance(ranges, angle_min, angle_increment,
                      half_width_m=CORRIDOR_HALF_WIDTH_M, geometry=LIDAR):
    """Forward distance from the sensor to the first return in the chassis-wide
    corridor ahead, read the way the approach's front brake reads it."""
    values = []
    for sample in robot_scan_samples(ranges, angle_min, angle_increment, geometry):
        if sample.sensor_range_m is None:
            continue
        forward = sample.sensor_range_m * math.cos(sample.sensor_angle_rad)
        lateral = sample.sensor_range_m * math.sin(sample.sensor_angle_rad)
        if forward > 0.0 and abs(lateral) <= half_width_m:
            values.append(forward)
    return robust_nearest(values)


# Chassis footprint around the centre, as the approach drops self-returns
# (sugarbox_training_contract.CHASSIS_HALF_*: mast, arm base and wiring).
CHASSIS_HALF_LENGTH_M = 0.15
CHASSIS_HALF_WIDTH_M = 0.15


def rear_clearance(ranges, angle_min, angle_increment,
                   half_width_m=CORRIDOR_HALF_WIDTH_M, geometry=LIDAR):
    """Free distance behind the chassis's back edge, in its own-width corridor."""
    values = []
    for sample in robot_scan_samples(ranges, angle_min, angle_increment, geometry):
        if sample.range_m is None:
            continue
        x = sample.range_m * math.cos(sample.angle_rad)
        y = sample.range_m * math.sin(sample.angle_rad)
        if abs(x) <= CHASSIS_HALF_LENGTH_M and abs(y) <= CHASSIS_HALF_WIDTH_M:
            continue                      # the robot itself
        if x < -CHASSIS_HALF_LENGTH_M and abs(y) <= half_width_m:
            values.append(-x - CHASSIS_HALF_LENGTH_M)
    return robust_nearest(values)


def wall_correction(measured_m, taught_m):
    """Metres to drive (+ forward) so the wall ahead reads the taught distance."""
    if not finite(measured_m, taught_m):
        raise ValueError('no wall reading straight ahead')
    error = measured_m - taught_m
    if abs(error) <= WALL_TOLERANCE_M:
        return 0.0
    if abs(error) > WALL_MAX_CORRECTION_M:
        raise ValueError('wall reads {:.3f} m, taught {:.3f} m: too far off to correct '
                         '(something else ahead, or the box moved)'.format(measured_m, taught_m))
    return error


def read_localization(host):
    """ncu's health record, aged on the robot's clock (one ssh round trip)."""
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', 'jetson@' + host,
         'cat {} 2>/dev/null; echo; date +%s.%N'.format(LOCALIZATION_HEALTH)],
        capture_output=True, text=True, timeout=30)
    lines = result.stdout.strip().splitlines()
    try:
        record = json.loads(lines[0]) if len(lines) == 2 else None
        record['age'] = float(lines[1]) - float(record['stamp'])
    except (TypeError, ValueError, KeyError):
        return None
    return record


def read_scans(host, seconds=1.0, count=8):
    """Up to `count` raw /scan messages from rosbridge."""
    import roslibpy
    scans = []
    ros = roslibpy.Ros(host=host, port=9090)
    ros.run()
    topic = roslibpy.Topic(ros, '/scan', 'sensor_msgs/LaserScan')
    topic.subscribe(lambda m: len(scans) < count and scans.append(m))
    try:
        deadline = time.monotonic() + max(seconds, 5.0)
        while len(scans) < count and time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        topic.unsubscribe()
        ros.close()
    if not scans:
        raise RuntimeError('no /scan from rosbridge')
    return scans


def wall_ahead(host):
    """Median corridor distance ahead over a few scans; the robot must be still."""
    readings = [corridor_distance([float('nan') if r is None else r for r in m['ranges']],
                                  m['angle_min'], m['angle_increment'])
                for m in read_scans(host)]
    readings = [r for r in readings if math.isfinite(r)]
    if not readings:
        return float('inf')
    return statistics.median(readings)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', required=True)
    parser.add_argument('--where', action='store_true')
    parser.add_argument('--box-ahead-m', type=float, default=None)
    parser.add_argument('--measure-wall', action='store_true')
    args = parser.parse_args(argv)
    ipaddress.IPv4Address(args.host)
    if not (args.where or args.measure_wall):
        parser.error('choose --where and/or --measure-wall')
    if args.where:
        pose = localization_pose(read_localization(args.host))
        print('robot pose map x={:.3f} y={:.3f} yaw={:+.1f} deg'.format(
            pose[0], pose[1], math.degrees(pose[2])))
        box = load_box_map()
        bearing, distance = box_bearing(pose, box)
        print('placement_box_map.json point: {:.2f} m away, bearing {:+.1f} deg ({})'.format(
            distance, math.degrees(bearing), 'verified' if box['verified'] else 'NOT verified'))
        if args.box_ahead_m is not None:
            x, y = point_ahead(pose, args.box_ahead_m)
            print('point {:.3f} m ahead of the chassis centre: map x={:.4f} y={:.4f}'.format(
                args.box_ahead_m, x, y))
    if args.measure_wall:
        print('wall ahead (LiDAR corridor, from the sensor): {:.3f} m'.format(wall_ahead(args.host)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
