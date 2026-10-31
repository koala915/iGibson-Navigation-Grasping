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
# the half-box stop, and again 3 cm farther (the user), so 4 cm. The jaw then
# opens 2.75 cm past the near edge instead of over the middle: about a
# sugarbox's half width.
PLACEMENT_STOP_BACK_M = 0.04
PLACEMENT_FINAL_X_M = round(RELEASE_CENTER_X_M - BOX_DEPTH_M / 2 + PLACEMENT_STOP_BACK_M, 3)
# The arm's centre line in the rear camera's frame: E1 y = rear y + 0.028 m.
PLACEMENT_AIM_Y_M = -0.028
# The jaw drops a ~5 cm box into an 18 cm side: 6 cm of room each way, less margin.
ARRIVAL_MAX_Y_ERROR_M = 0.04
ARRIVAL_MAX_X_OVERSHOOT_M = 0.03
LOCALIZATION_MAX_AGE_S = 1.0
# A short localization outage is waited out with the robot stopped: 2026-10-21
# run 21 ended its placement on "waiting for fresh scan" read once, right after a
# 120 deg turn. The wait ends on LOCALIZATION_STABLE_READS new ready records that
# agree within LOCALIZATION_STABLE_M/RAD.
LOCALIZATION_WAIT_S = 20.0
LOCALIZATION_POLL_S = 0.3
LOCALIZATION_STABLE_READS = 3
LOCALIZATION_STABLE_M = 0.03
LOCALIZATION_STABLE_RAD = math.radians(3)
# Across an outage the map pose may move only as far as odometry did since the
# last good record before it (ncu's recovery gate, 2026-10-22); more is a jump the
# robot did not make, and needs a relocalization. Checked only across an outage:
# against an older record, normal odometry drift alone would trip it.
LOCALIZATION_JUMP_M = 0.15
LOCALIZATION_JUMP_RAD = math.radians(10)
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


class NavMask:
    """ncu's patrol map mask: where the robot's centre may be for the patrol to
    start (g2_patrol_policy.Policy.allowed: a pixel above 250 is allowed)."""

    def __init__(self, width, height, cells, resolution, origin):
        if resolution <= 0 or len(cells) != width * height:
            raise ValueError('navigation mask geometry')
        self.width, self.height, self.cells = width, height, cells
        self.resolution, self.origin = float(resolution), (float(origin[0]), float(origin[1]))

    def allowed(self, x, y):
        if not finite(x, y):
            return False
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = self.height - 1 - int(math.floor((y - self.origin[1]) / self.resolution))
        if not (0 <= col < self.width and 0 <= row < self.height):
            return False
        return self.cells[row * self.width + col] > 250

    def inside(self, point, margin=0.0):
        """The point and a ring of eight around it at `margin` all allowed."""
        x, y = point
        ring = [(x + margin * math.cos(k * math.pi / 4), y + margin * math.sin(k * math.pi / 4))
                for k in range(8)] if margin > 0 else []
        return all(self.allowed(px, py) for px, py in [(x, y)] + ring)


def parse_pgm(data):
    """(width, height, cells) of a binary 8-bit PGM."""
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while end < len(data) and not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise ValueError('navigation mask is not an 8-bit binary PGM')
    width, height = int(tokens[1]), int(tokens[2])
    cells = data[pos + 1:pos + 1 + width * height]
    if len(cells) != width * height:
        raise ValueError('navigation mask is truncated')
    return width, height, cells


def fetch_nav_mask(host, scene=SCENE):
    """The patrol's navigation_center_mask as the robot has it."""
    import base64
    import yaml
    folder = '{}/{}'.format(JETSON_ROOT, scene)
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', 'jetson@' + host,
         'cd {} && cat navigation_center_mask.yaml && echo ===PGM=== && '
         'base64 -w0 navigation_center_mask.pgm'.format(folder)],
        capture_output=True, text=True, timeout=60)
    try:
        meta_text, data_text = result.stdout.split('===PGM===')
        meta = yaml.safe_load(meta_text)
        width, height, cells = parse_pgm(base64.b64decode(data_text.strip()))
        return NavMask(width, height, cells, meta['resolution'], meta['origin'][:2])
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        raise ValueError('cannot read the navigation mask on the robot: {} {}'.format(
            exc, result.stderr.strip()))


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


def localization_jump(before, after):
    """Why `after`'s map pose cannot follow from `before`'s by odometry, or None.

    None too when either record lacks pose and odometry: the supervisor's own
    reference checks then stand alone.
    """
    a, b = [(r.get('state') or {}) if isinstance(r, dict) else {} for r in (before, after)]
    p, o, q, n = a.get('pose'), a.get('odom'), b.get('pose'), b.get('odom')
    if not all(isinstance(v, (list, tuple)) and len(v) == 3 and finite(*v)
               for v in (p, o, q, n)):
        return None
    turn = p[2] - o[2]                   # map <- odom rotation at `before`
    dx, dy = n[0] - o[0], n[1] - o[1]
    x = p[0] + math.cos(turn) * dx - math.sin(turn) * dy
    y = p[1] + math.sin(turn) * dx + math.cos(turn) * dy
    offset = math.hypot(x - q[0], y - q[1])
    rotation = abs(wrap(p[2] + wrap(n[2] - o[2]) - q[2]))
    if offset <= LOCALIZATION_JUMP_M and rotation <= LOCALIZATION_JUMP_RAD:
        return None
    return ('the map pose moved {:.2f} m / {:.0f} deg away from where odometry puts the '
            'robot'.format(offset, math.degrees(rotation)))


def wait_localization(host, read=None, before=None, wait_s=None, sleep=None, now=None):
    """A ready health record; a short outage waited out with the robot stopped.

    Usually one read, as before. After a failed one, LOCALIZATION_STABLE_READS new
    ready records that agree end the wait, if the last does not jump from `before`
    (the last good record before the outage, if the caller has one). Callers keep
    the robot stopped meanwhile. ValueError after `wait_s`, or on a jump.
    """
    read, sleep, now = read or read_localization, sleep or time.sleep, now or time.monotonic
    wait_s = LOCALIZATION_WAIT_S if wait_s is None else wait_s
    record = read(host)
    try:
        localization_pose(record)
        return record
    except ValueError as exc:
        reason = str(exc)
    print('[LOCALIZE] {}: waiting up to {:.0f} s with the robot stopped'.format(
        reason, wait_s), flush=True)
    started = now()
    good = []
    while now() - started < wait_s:
        sleep(LOCALIZATION_POLL_S)
        record = read(host)
        try:
            pose = localization_pose(record)
        except ValueError as exc:
            reason, good = str(exc), []
            continue
        if good and record.get('stamp') == good[-1][0].get('stamp'):
            continue                     # the same record again, not a new one
        if good and (math.hypot(pose[0] - good[0][1][0], pose[1] - good[0][1][1])
                     > LOCALIZATION_STABLE_M
                     or abs(wrap(pose[2] - good[0][1][2])) > LOCALIZATION_STABLE_RAD):
            good = []
        good.append((record, pose))
        if len(good) >= LOCALIZATION_STABLE_READS:
            jump = localization_jump(before, record)
            if jump:
                raise ValueError('not localized: {} during the outage; relocalize before '
                                 'going on'.format(jump))
            print('[LOCALIZE] back after {:.1f} s'.format(now() - started), flush=True)
            return record
    raise ValueError('not localized after waiting {:.0f} s: {}'.format(wait_s, reason))


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
# A corner moving along a wall within the margin is no nearer; rounding aside.
WALL_APPROACH_M = 0.005


def bin_obstacle(box):
    """(x, y, radius) of the placement box for back_path_problem, None without one."""
    if not box:
        return None
    return (box['x'], box['y'],
            BIN_HALF_DIAGONAL_M + (0.0 if box.get('verified') else BIN_UNVERIFIED_SLACK_M))


# Backing toward where the patrol stopped retraces the approach only while that
# spot lies close to the straight line behind the robot.
RETRACE_MAX_LATERAL_M = 0.25
RETRACE_MIN_M = 0.05


def retrace_room(pose, came_from, max_lateral=RETRACE_MAX_LATERAL_M):
    """How far straight back the robot is from `came_from` (the map spot where the
    patrol stopped before the approach), or None when that spot is not behind it
    near the backward line: then backing would not retrace the way it came."""
    if pose is None or came_from is None or not finite(*pose, *came_from):
        return None
    x, y, yaw = pose
    dx, dy = came_from[0] - x, came_from[1] - y
    behind = -(dx * math.cos(yaw) + dy * math.sin(yaw))
    lateral = abs(-dx * math.sin(yaw) + dy * math.cos(yaw))
    if behind < RETRACE_MIN_M or lateral > max_lateral:
        return None
    return behind


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
            # Within the margin, only coming nearer counts: after an angled
            # approach a wheel may already be at the wall, and backing straight
            # takes it away (2026-10-21, first delivery: refused at 0 -> 5.6 cm off
            # the wall). Ending beyond a wall never passes.
            end_m = wall_distance(end, walls)
            if end_m <= 0 or (end_m <= margin
                              and end_m < wall_distance(start, walls) - WALL_APPROACH_M):
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
    except (TypeError, ValueError, KeyError, IndexError):   # IndexError: ssh said nothing
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
