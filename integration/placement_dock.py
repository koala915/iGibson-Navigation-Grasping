#!/usr/bin/env python3
"""Placement-box docking: geometry and checks for the approach runner (2026-10-18).

The placement box stays put for the whole competition (the user, 2026-10-18), so
its map position and the resident AMCL pose (ncu's localization service, PR #5)
say where it is even when the rear camera cannot see it. After a grasp the runner
turns toward it, seeds the approach with it, and the rear camera takes over as
soon as it finds the box. Nothing here moves the robot; the runner owns motion.

The box's map position has one home: ncu's scene_layout.json on the robot, the
same file his patrol plans with (the user, 2026-10-19). It is read from there.

Two tools for the person setting up the field, both read-only:

    python integration/placement_dock.py --host <IP> --where --box-ahead-m 0.50
        map position of the point 0.50 m ahead of the chassis centre (park facing
        the box, tape the distance to its centre, give the result to ncu for
        bin.pose_map in scene_layout.json)
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

JETSON_ROOT = '/home/jetson/Documents/deploy_jetson2'
# ncu's patrol scene on the Jetson (relative to JETSON_ROOT).
SCENE = 'maps/demo_box_20261018_patrol'
LOCALIZATION_HEALTH = '/dev/shm/g2_localization_health.json'
# ncu's tape measurement: 18 x 13.5 x 6 cm, the long side facing the field.
BOX_DEPTH_M = 0.135
# The scripted release opens the jaw about this far ahead of the chassis centre
# (FK reach 23.9 cm from the arm base). An estimate until release-preview is
# measured on the robot; the runner's --final-x-m overrides it.
RELEASE_CENTER_X_M = 0.28
# SAM measures the box's nearest bottom edge, so that edge stops half a box short,
# and PLACEMENT_STOP_BACK_M farther: on 2026-10-21 the robot ran into the box at
# the half-box stop (the user). The jaw then opens 3.75 cm past the near edge
# instead of over the middle.
PLACEMENT_STOP_BACK_M = 0.03
PLACEMENT_FINAL_X_M = round(RELEASE_CENTER_X_M - BOX_DEPTH_M / 2 + PLACEMENT_STOP_BACK_M, 3)
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


def box_from_layout(layout, source='scene_layout.json'):
    """The placement box from ncu's scene layout: {'x', 'y', 'verified', 'source'}.

    bin.pose_map counts once ncu marks it valid for where the box stands now
    (pose_map_valid_for_current_placement). Until then his moved-box candidate
    stands in, unverified: the runner turns toward it but never drives at it.
    On 2026-10-19 pose_map was still the corner the box was moved out of, 20 cm off.
    """
    box = layout.get('bin') if isinstance(layout, dict) else None
    if not isinstance(box, dict) or layout.get('frame_id') != 'map':
        raise ValueError('scene layout has no placement box in the map frame: ' + source)
    if box.get('pose_map_valid_for_current_placement') is True:
        point, verified = box.get('pose_map'), True
    else:
        point, verified = (box.get('placement_update') or {}).get(
            'midpoint_pose_map_candidate'), False
    if not isinstance(point, dict) or not finite(point.get('x'), point.get('y')):
        raise ValueError('scene layout has no placement box position: ' + source)
    return {'x': float(point['x']), 'y': float(point['y']), 'verified': verified,
            'source': source}


def walls_from_layout(layout):
    """The room's wall polygon [(x, y), ...] in the map frame, or None if absent."""
    polygon = layout.get('room_polygon') if isinstance(layout, dict) else None
    if (not isinstance(polygon, list) or len(polygon) < 3
            or not all(isinstance(p, (list, tuple)) and len(p) == 2 and finite(*p)
                       for p in polygon)):
        return None
    return [(float(x), float(y)) for x, y in polygon]


def wall_distance(point, walls):
    """Distance from a map point to the nearest wall: positive inside the wall
    polygon, negative beyond it."""
    x, y = point
    inside, distances = False, []
    for (x1, y1), (x2, y2) in zip(walls, walls[1:] + walls[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy or 1.0)))
        distances.append(math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return min(distances) if inside else -min(distances)


def near_wall(point, walls, margin):
    """True for a map point outside the wall polygon or within `margin` of a wall."""
    return wall_distance(point, walls) <= margin


def load_box_map(path):
    """The placement box from a local copy of scene_layout.json."""
    with open(path, encoding='utf-8') as f:
        return box_from_layout(json.load(f), str(path))


def fetch_layout(host, scene=SCENE):
    """ncu's scene_layout.json as the robot has it."""
    path = '{}/{}/scene_layout.json'.format(JETSON_ROOT, scene)
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', 'jetson@' + host,
         'cat ' + path], capture_output=True, text=True, timeout=30)
    try:
        return json.loads(result.stdout)
    except ValueError:
        raise ValueError('cannot read {} on the robot: {}'.format(path, result.stderr.strip()))


def fetch_box(host, scene=SCENE):
    return box_from_layout(fetch_layout(host, scene), 'robot:{}/scene_layout.json'.format(scene))


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


# The robot's outline (x forward, y left from the chassis centre, x1 y1 x2 y2):
# ncu's surveyed body rectangles in the LiDAR self-return profile (robot file
# deploy/ros/lidar_self_returns_2026-10-18.json). Each one, not the box around
# them all: the right-side cables stick out 0.29 m only mid-body, and a box
# carried that width to a rear corner the robot does not have (2026-10-19).
BODY_RECTANGLES = ((-0.15, -0.14, 0.22, 0.14), (-0.18, -0.22, -0.06, -0.13),
                   (-0.20, 0.10, -0.15, 0.14), (-0.05, -0.29, 0.02, -0.21),
                   (0.04, -0.29, 0.075, -0.21), (-0.23, -0.205, -0.18, -0.135))
# Backing straight must leave the whole outline this far inside the room's walls
# (AMCL ~4 cm, and the margin the old LiDAR check kept).
BACK_WALL_MARGIN_M = 0.08


def to_map(pose, point):
    """Chassis-centre frame point -> map."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


# The placement box is below the LiDAR's scan plane too (0.06 m): backing keeps its
# footprint, a disc of its half diagonal, clear as well; an unverified map
# position (the moved-box candidate) gets BIN_UNVERIFIED_SLACK_M more.
BIN_HALF_DIAGONAL_M = 0.112
BIN_UNVERIFIED_SLACK_M = 0.20
BIN_APPROACH_M = 0.02


def bin_obstacle(box):
    """(x, y, radius) of the placement box for back_path_problem, None without one."""
    if not box:
        return None
    return (box['x'], box['y'],
            BIN_HALF_DIAGONAL_M + (0.0 if box.get('verified') else BIN_UNVERIFIED_SLACK_M))


def segment_distance(point, start, end):
    px, py = point
    (ax, ay), (bx, by) = start, end
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy or 1.0)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def back_path_problem(pose, metres, walls, margin=BACK_WALL_MARGIN_M, obstacles=()):
    """None when backing `metres` straight keeps the robot inside the room, else why not.

    The LiDAR cannot see behind the robot: the body blocks its rear half (the
    user, 2026-10-19; what it returned there was the robot's own right-rear
    corner, read as an obstacle 1 cm away with ~30 cm free). The field has only
    walls (the user, 2026-10-19), so the map and the AMCL pose decide. The room
    is convex: an outline inside it at the start and at the end stays inside on
    the way.
    """
    for x1, y1, x2, y2 in BODY_RECTANGLES:
        for cx, cy in ((x1, y1), (x1, y2), (x2, y1), (x2, y2)):
            start, end = to_map(pose, (cx, cy)), to_map(pose, (cx - metres, cy))
            if near_wall(end, walls, margin):
                return 'backing {:.2f} m would bring the robot within {:.2f} m of a wall'.format(
                    metres, margin)
            for ox, oy, radius in obstacles:
                # Only coming nearer counts: right after a release the front is at
                # the box already, and backing straight away is the way out. A front
                # corner just past the box centre first closes by a hair as it backs
                # by (2026-10-20, second delivery: refused); that is not coming nearer.
                nearest = segment_distance((ox, oy), start, end)
                if (nearest <= radius + margin
                        and nearest < math.hypot(ox - start[0], oy - start[1]) - BIN_APPROACH_M):
                    return ('backing {:.2f} m would sweep the robot within {:.2f} m of the '
                            'placement box'.format(metres, margin))
    return None


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
    parser.add_argument('--scene', default=SCENE)
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
        box = fetch_box(args.host, args.scene)
        bearing, distance = box_bearing(pose, box)
        print('scene_layout box ({:.4f}, {:.4f}): {:.2f} m away, bearing {:+.1f} deg ({})'.format(
            box['x'], box['y'], distance, math.degrees(bearing),
            'verified' if box['verified'] else 'NOT verified'))
        if args.box_ahead_m is not None:
            x, y = point_ahead(pose, args.box_ahead_m)
            print('point {:.3f} m ahead of the chassis centre: map x={:.4f} y={:.4f}'.format(
                args.box_ahead_m, x, y))
    if args.measure_wall:
        print('wall ahead (LiDAR corridor, from the sensor): {:.3f} m'.format(wall_ahead(args.host)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
