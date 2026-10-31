"""Move the placement box in the fixed scene, offline. This tool never contacts the robot.

The operator tapes the box centre to the right and top walls as the loop console
draws them (room u max = ncu's "photo_back_wall", v max = "photo_left_wall").
The centre mask is rebuilt from scratch: unlike rebuild_patrol_mask.py it does
not keep the old allowed cells, since those may now sit on the box. Every other
rule is the same: known-free map, 35 cm from walls and map obstacles, and the
original 0.283 m keepout around a slack-widened box envelope. The result must
still carry the patrol ring at `--inset`, or nothing is written.

    python integration/move_placement_box.py --source maps/demo_box_20261018_patrol \
        --output <candidate folder> --right-m 0.45 --top-m 0.45 --long-edge right --inset 0.80
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from g2_patrol_policy import Policy  # noqa: E402

MARGIN_M = 0.35
# Registration allowance kept from 10-18 (0.04) plus 2 cm for a tape measurement.
SLACK_M = 0.06


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(source, destination, right_m, top_m, long_edge, inset, slack=SLACK_M, note=''):
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError('write a separate candidate folder; keep the original for rollback')
    if long_edge not in ('right', 'top'):
        raise ValueError('long edge must be parallel to the right or the top wall')
    layout = json.loads((source/'scene_layout.json').read_text())
    meta = yaml.safe_load((source/'navigation_center_mask.yaml').read_text())
    raw_meta = yaml.safe_load((source/layout['map']).read_text())
    if meta['resolution'] != raw_meta['resolution'] or meta['origin'] != raw_meta['origin']:
        raise ValueError('raw map and mask must have identical geometry')
    raw = cv2.imread(str(source/raw_meta['image']), cv2.IMREAD_GRAYSCALE)
    res = float(meta['resolution'])
    free = raw > 255 * (1.0 - float(raw_meta['free_thresh']))
    distances = cv2.distanceTransform(np.pad(free.astype(np.uint8), 1),
                                      cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    map_clearance = distances * res - math.sqrt(2) * res
    rows, cols = np.indices(raw.shape)
    x = meta['origin'][0] + (cols + .5) * res
    y = meta['origin'][1] + (raw.shape[0] - rows - .5) * res
    a = layout['room_heading_rad']
    c, s = math.cos(a), math.sin(a)
    u, v = x*c + y*s, -x*s + y*c
    polygon = [(px*c + py*s, -px*s + py*c) for px, py in layout['room_polygon']]
    umin, umax = min(p[0] for p in polygon), max(p[0] for p in polygon)
    vmin, vmax = min(p[1] for p in polygon), max(p[1] for p in polygon)
    wall_clearance = np.minimum.reduce((u-umin, umax-u, v-vmin, vmax-v))

    box = layout['bin']
    length, width = box['dimensions_m']['length'], box['dimensions_m']['width']
    half_u, half_v = (width/2, length/2) if long_edge == 'right' else (length/2, width/2)
    cu, cv = umax - right_m, vmax - top_m
    to_map = lambda pu, pv: [pu*c - pv*s, pu*s + pv*c]
    centre = to_map(cu, cv)
    yaw = math.atan2(math.sin(a + (math.pi/2 if long_edge == 'right' else 0.)),
                     math.cos(a + (math.pi/2 if long_edge == 'right' else 0.)))
    envelope = [cu-half_u-slack, cu+half_u+slack, cv-half_v-slack, cv+half_v+slack]
    bin_clearance = np.hypot(
        np.maximum.reduce((envelope[0]-u, u-envelope[1], np.zeros_like(u))),
        np.maximum.reduce((envelope[2]-v, v-envelope[3], np.zeros_like(v))))
    buffer_m = float(box['center_keepout_buffer_m'])
    if buffer_m < .283:
        raise ValueError('requires the original 0.283 m keepout buffer')
    cell_radius = res / math.sqrt(2)
    mask = (free & (map_clearance >= MARGIN_M) & (wall_clearance >= MARGIN_M + cell_radius)
            & (bin_clearance >= buffer_m + cell_radius))

    destination.mkdir(parents=True, exist_ok=True)
    for name in ('navigation_center_mask.yaml', layout['map'], raw_meta['image'],
                 layout['preferred_patrol']['route_file']):
        shutil.copy2(source/name, destination/name)
    output = destination/meta['image']
    if not cv2.imwrite(str(output), np.where(mask, 254, 0).astype(np.uint8)):
        raise ValueError('mask write failed')

    pose = {'x': centre[0], 'y': centre[1], 'yaw': yaw}
    footprint = [to_map(cu+du, cv+dv) for du, dv in
                 ((-half_u, -half_v), (half_u, -half_v), (half_u, half_v), (-half_u, half_v))]
    previous = {'pose_map': box.get('pose_map'),
                'midpoint_pose_map_candidate': (box.get('placement_update') or {}).get(
                    'midpoint_pose_map_candidate'),
                'envelope_room_uv': layout['navigation_mask_validation'].get('envelope_room_uv'),
                'mask_sha256': digest(source/meta['image'])}
    # A verified position stays verified when only the mask is rebuilt around it
    # (2026-10-29: slack cut to 2 cm after the camera check); a moved box is not.
    old = box.get('pose_map') or {}
    same = (box.get('pose_map_valid_for_current_placement') is True
            and math.hypot(old.get('x', 1e9) - pose['x'], old.get('y', 1e9) - pose['y']) < 1e-3)
    box.update(pose_map=pose, footprint_map=footprint, long_edge_parallel_to=(
        'photo_back_wall' if long_edge == 'right' else 'photo_left_wall'),
        wall_clearance_m={'photo_back_wall': round(right_m - half_u, 4),
                          'photo_left_wall': round(top_m - half_v, 4)},
        pose_map_valid_for_current_placement=same, registration_verified=False)
    update = box.setdefault('placement_update', {})
    update.update(midpoint_pose_map_candidate=dict(pose),
                  coordinate_status='candidate_only; taped centre-to-wall, not registered')
    if isinstance(box.get('dock_candidate'), dict):
        box['dock_candidate']['superseded'] = 'box moved; recompute before any use'
    record = dict(right_wall_to_centre_m=right_m, top_wall_to_centre_m=top_m,
                  long_edge_parallel_to=long_edge + ' wall (as the loop console draws it)',
                  slack_m=slack, source='operator tape measurement', note=note,
                  previous=previous)
    box['moves'] = box.get('moves', []) + [record]
    layout['navigation_mask_validation'].update(
        method='rebuilt from scratch: known free, %.0f cm from walls/map, moved-box envelope '
               '+ %.3f m keepout' % (MARGIN_M*100, buffer_m),
        envelope_room_uv=envelope, exact_box_pose_registered=False,
        source='operator tape measurement after moving the box inward')
    layout['navigation_mask_valid_for_current_bin_placement'] = True
    layout['patrol_inset_from_fitted_wall_m'] = inset
    # g2_patrol_policy accepts a sub-40 cm mask only with this record and its hash.
    layout['navigation_mask_reduction_20261021'] = dict(
        schema=1, wall_map_margin_m=MARGIN_M, turn_reserve_m=.06, bin_buffer_m=buffer_m,
        before_sha256=previous['mask_sha256'], sha256=digest(output),
        raw_map_sha256=digest(source/raw_meta['image']), allowed_cells=int(mask.sum()),
        rebuilt_for_box_move=True, physical_validation='offline only; next supervised run required')
    (destination/'scene_layout.json').write_text(json.dumps(layout, indent=2) + '\n')

    policy = Policy(str(destination))      # the robot's own acceptance and ring checks
    ring = policy.patrol_points(inset)
    return dict(box=pose, envelope_room_uv=envelope, allowed_cells=int(mask.sum()),
                mask_sha256=digest(output), layout_sha256=digest(destination/'scene_layout.json'),
                ring=ring, inset=inset)


ROUTE_INSET_M = 0.55             # ncu's outer route: 55 cm inside the fitted walls
# How far the route's box corner stays from the box envelope. A turn needs 0.338 m
# (body sweep 0.308 + 0.03), and the patrol accepts arrivals 6.5 cm off the point,
# then turns where it stopped: 2026-10-30, with 4 cm spare at the corners and 1.1 cm
# mid-leg, a heading fix there could be refused. The chord between points
# comes 3 cm nearer the box than the points themselves.
ROUTE_BOX_CLEAR_M = 0.45
# The corner farthest from the box: left of the next leg, the camera sees the room's
# centre, which no turn of this counterclockwise route shows it (2026-10-30). 60 and
# 90 degrees leave the same 6% unseen; the camera's own width makes up the rest.
ROUTE_LOOK_LEFT_DEG = 60


def write_route(folder, inset=ROUTE_INSET_M, clear=ROUTE_BOX_CLEAR_M):
    """ncu's patrol-cycle route (preferred_patrol.route_file), clipped around the box.

    patrol-cycle follows this file, not patrol_points(--inset). The ring keeps
    `inset` from the walls; at the u/v-max corner it rounds the box envelope at
    `clear`, in three points. Every point must be allowed, every leg plannable and
    turnable, as the robot's g2_navigation_goals and rotation_allowed check.
    """
    folder = Path(folder)
    layout = json.loads((folder/'scene_layout.json').read_text())
    a = layout['room_heading_rad']
    c, s = math.cos(a), math.sin(a)
    polygon = [(px*c + py*s, -px*s + py*c) for px, py in layout['room_polygon']]
    umin, umax = min(p[0] for p in polygon), max(p[0] for p in polygon)
    vmin, vmax = min(p[1] for p in polygon), max(p[1] for p in polygon)
    env = layout['navigation_mask_validation']['envelope_room_uv']
    to_map = lambda pu, pv: [pu*c - pv*s, pu*s + pv*c]
    corner = clear / math.sqrt(2)
    uv = [(umin+inset, vmin+inset), (umax-inset, vmin+inset), (umax-inset, env[2]-clear),
          (env[0]-corner, env[2]-corner), (env[0]-clear, vmax-inset), (umin+inset, vmax-inset),
          (umin+inset, vmin+inset)]
    route = [{'name': 'outer_ccw_%d' % i, 'point': to_map(*p)} for i, p in enumerate(uv)]
    route[0]['look_left_deg'] = ROUTE_LOOK_LEFT_DEG
    name = layout['preferred_patrol']['route_file']
    (folder/name).write_text(json.dumps(route, indent=2) + chr(10))
    policy = Policy(str(folder))
    for item in route:
        if not policy.allowed(*item['point']):
            raise ValueError('route point {} is outside the centre mask'.format(item['name']))
    for one, two in zip(route, route[1:]):
        policy.plan(one['point'], two['point'])
    return dict(route_file=name, points=route, clear_m=clear, sha256=digest(folder/name))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--right-m', type=float)
    parser.add_argument('--top-m', type=float)
    parser.add_argument('--long-edge', choices=('right', 'top'))
    parser.add_argument('--inset', type=float)
    parser.add_argument('--note', default='')
    parser.add_argument('--slack', type=float, default=SLACK_M,
                        help='added to the box size before the keepout: 0.06 taped, 0.02 once '
                             'verified by the camera')
    parser.add_argument('--route-only', action='store_true',
                        help='rewrite only the patrol-cycle route file in --source, in place')
    args = parser.parse_args()
    if args.route_only:
        print(json.dumps(write_route(args.source), indent=2))
    else:
        if None in (args.output, args.right_m, args.top_m, args.long_edge, args.inset):
            parser.error('--output, --right-m, --top-m, --long-edge and --inset are required')
        if not 0.02 <= args.slack <= 0.10:
            parser.error('slack must be 0.02..0.10 m')
        report = build(args.source, args.output, args.right_m, args.top_m, args.long_edge,
                       args.inset, slack=args.slack, note=args.note)
        report['route'] = write_route(args.output)
        print(json.dumps(report, indent=2))
