"""Where the rear camera can find a sugarbox on one patrol lap, and what would help.

Offline. Footprint: the camera's ground homography. Reach: the farthest first
sightings of the 2026-10-29/30 runs. Route and turn rules: the robot's own files.

    PYTHONPATH="integration;deploy/ros" python docs/operations/vision_coverage_20261030/coverage_map.py
"""
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'integration'), str(ROOT/'deploy/ros')]
from g2_patrol_policy import Policy  # noqa: E402

SCENE = ROOT/'maps/demo_box_20261018_patrol'
H = np.array(json.loads((ROOT/'detection/rear_ground_homography.json').read_text())['H_pixel_to_ground'])
HI = np.linalg.inv(H)
CENTRE_TO_GROUND_M = 0.065      # homography x = chassis-centre x + this
ARM = (295, 405, 412)           # sightings here are dropped as the arm camera's corner
EDGE_PX, BOTTOM_PX = 30, 470    # a box needs its width in frame; lowest bottom edge seen
REACH_M = 1.0                   # 10-29/30: farthest first sighting 0.96 m
STEP_M, TURN_RAD = 0.09, 0.12   # one forward pulse, one turn pulse
CELL = 0.02

layout = json.loads((SCENE/'scene_layout.json').read_text())
a = layout['room_heading_rad']
c, s = math.cos(a), math.sin(a)


def to_uv(x, y):
    return x*c + y*s, -x*s + y*c


def to_map(u, v):
    return u*c - v*s, u*s + v*c


room = [to_uv(*p) for p in layout['room_polygon']]
umin, umax = min(p[0] for p in room), max(p[0] for p in room)
vmin, vmax = min(p[1] for p in room), max(p[1] for p in room)
box = to_uv(layout['bin']['pose_map']['x'], layout['bin']['pose_map']['y'])
box_corners = [to_uv(*p) for p in layout['bin']['footprint_map']]
route = [to_uv(*i['point']) for i in json.loads((SCENE/'outer_patrol_goals.json').read_text())]
policy = Policy(str(SCENE))

us = np.arange(umin+CELL/2, umax, CELL)
vs = np.arange(vmin+CELL/2, vmax, CELL)
U, V = np.meshgrid(us, vs)
eligible = (np.minimum.reduce((U-umin, umax-U, V-vmin, vmax-V)) >= 0.15)
eligible &= np.hypot(U-box[0], V-box[1]) >= 0.35


def footprint(u, v, yaw, reach=REACH_M):
    """Floor cells the camera sees from a pose (room frame)."""
    dx, dy = U-u, V-v
    x = dx*math.cos(yaw) + dy*math.sin(yaw) + CENTRE_TO_GROUND_M
    y = -dx*math.sin(yaw) + dy*math.cos(yaw)
    w = HI[2, 0]*x + HI[2, 1]*y + HI[2, 2]
    px = (HI[0, 0]*x + HI[0, 1]*y + HI[0, 2]) / w
    py = (HI[1, 0]*x + HI[1, 1]*y + HI[1, 2]) / w
    arm = (ARM[0] <= px) & (px <= ARM[1]) & (py >= ARM[2])
    return ((x > 0) & (EDGE_PX <= px) & (px <= 640-EDGE_PX) & (py <= BOTTOM_PX)
            & (np.hypot(x-CENTRE_TO_GROUND_M, y) <= reach) & ~arm)


def wrap(t):
    return (t + math.pi) % (2*math.pi) - math.pi


def drive(points, spin_at=()):
    """Poses of a route as the patrol drives it: (u, v, yaw, is_turning)."""
    poses, here = [], points[0]
    yaw = math.atan2(points[1][1]-here[1], points[1][0]-here[0])
    for goal in points[1:]:
        want = math.atan2(goal[1]-here[1], goal[0]-here[0])
        while abs(wrap(want-yaw)) > TURN_RAD:
            yaw = wrap(yaw + math.copysign(TURN_RAD, wrap(want-yaw)))
            poses.append((here[0], here[1], yaw, True))
        yaw = want
        steps = max(1, int(math.hypot(goal[0]-here[0], goal[1]-here[1])/STEP_M))
        for i in range(1, steps+1):
            poses.append((here[0]+(goal[0]-here[0])*i/steps, here[1]+(goal[1]-here[1])*i/steps,
                          yaw, False))
        here = goal
        if any(math.hypot(here[0]-p[0], here[1]-p[1]) < .02 for p in spin_at):
            for _ in range(int(2*math.pi/TURN_RAD)):
                yaw = wrap(yaw+TURN_RAD)
                poses.append((here[0], here[1], yaw, False))  # stopped, looking on purpose
    return poses


def cover(poses, reach=REACH_M):
    straight, turning = np.zeros_like(U, bool), np.zeros_like(U, bool)
    for u, v, yaw, is_turning in poses:
        if is_turning:
            turning |= footprint(u, v, yaw, reach)
        else:
            straight |= footprint(u, v, yaw, reach)
    return straight, turning


def score(poses, reach=REACH_M):
    straight, turning = cover(poses, reach)
    n = eligible.sum()
    return dict(straight=float(100*(straight & eligible).sum()/n),
                any=float(100*((straight | turning) & eligible).sum()/n),
                pulses=len(poses))


def feasible(points, spins=()):
    """The robot's own rules: every point allowed, every leg plannable, spins allowed."""
    pts = [to_map(*p) for p in points]
    for p in pts:
        if not policy.allowed(*p):
            return 'point outside the mask: {}'.format(to_uv(*p))
    for one, two in zip(pts, pts[1:]):
        try:
            policy.plan(list(one), list(two))
        except Exception as exc:
            return 'leg: {}'.format(exc)
    for p in spins:
        m = to_map(*p)
        for i in range(16):
            if not policy.compact_turn_allowed([m[0], m[1], i*math.pi/8]):
                return 'spin refused at {}'.format(p)
    return 'ok'


centre = (.5*(umin+umax)-.05, .5*(vmin+vmax)-.05)
ring = route
options = {
    '現況': (ring, ()),
    '① 穿過中央兩次': ([ring[0], centre] + ring[1:6] + [centre, ring[0]], ()),
    '② 穿過中央三次': ([ring[0], ring[1], centre] + ring[2:6] + [centre, ring[0]], ()),
    '③ 第二圈反方向': (ring + ring[::-1][1:], ()),
    '④ 中央停下轉一圈': (ring + [centre, ring[0]], (centre,)),
}

if __name__ == '__main__':
    print('room', round(umax-umin, 3), round(vmax-vmin, 3),
          'route', [(round(p[0]-umin, 2), round(p[1]-vmin, 2)) for p in route])
    print('centre', round(centre[0]-umin, 2), round(centre[1]-vmin, 2), 'eligible', int(eligible.sum()))
    results = {}
    for name, (pts, spins) in options.items():
        poses = drive(pts, spins)
        r = score(poses)
        r['reach_1_2'] = score(poses, 1.2)['any']
        r['ok'] = feasible(pts, spins)
        results[name] = (pts, spins, r)
        print(name, {k: (round(v, 1) if isinstance(v, float) else v) for k, v in r.items()})
    base, turning = cover(drive(ring))
    grid = np.full(U.shape, 3)        # 3 never, 2 only while turning, 1 straight, 0 not a box spot
    grid[turning] = 2
    grid[base] = 1
    grid[~eligible] = 0
    np.save(Path(__file__).with_name('grid.npy'), grid)
    Path(__file__).with_name('results.json').write_text(json.dumps(
        {k: v[2] for k, v in results.items()}, ensure_ascii=False, indent=1), encoding='utf-8')
