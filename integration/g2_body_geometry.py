"""Shared Python 2/3 body/back-path geometry from koala e8766f0 placement_dock.

No sensor, model, ROS or hardware imports.
"""
import math


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


BODY_RECTANGLES = ((-0.15, -0.14, 0.22, 0.14), (-0.18, -0.22, -0.06, -0.13),
                   (-0.20, 0.10, -0.15, 0.14), (-0.05, -0.29, 0.02, -0.21),
                   (0.04, -0.29, 0.075, -0.21), (-0.23, -0.205, -0.18, -0.135))


BACK_WALL_MARGIN_M = 0.08


def to_map(pose, point):
    """Chassis-centre frame point -> map."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


BIN_APPROACH_M = 0.02


WALL_APPROACH_M = 0.005


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
