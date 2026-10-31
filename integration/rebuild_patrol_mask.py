"""Rebuild a fixed-scene centre mask offline. This tool never contacts the robot.

Only the extra wall/map inflation changes. Unknown map cells, measured bin
envelope, LiDAR geometry approval and patrol turn reserve remain enforced.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

MIN_MARGIN_M = 0.35


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(source, destination, margin=MIN_MARGIN_M):
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError('write a separate candidate folder; keep the original for rollback')
    if not math.isfinite(margin) or not MIN_MARGIN_M <= margin <= 0.40:
        raise ValueError('wall/map margin must be between 0.35 and 0.40 m')
    layout = json.loads((source/'scene_layout.json').read_text())
    meta = yaml.safe_load((source/'navigation_center_mask.yaml').read_text())
    raw_meta = yaml.safe_load((source/layout['map']).read_text())
    if (meta['resolution'] != raw_meta['resolution'] or meta['origin'] != raw_meta['origin']
            or meta['origin'][2] != 0 or raw_meta.get('negate', 0) != 0):
        raise ValueError('raw map and mask must have identical, unrotated geometry')
    old = cv2.imread(str(source/meta['image']), cv2.IMREAD_GRAYSCALE)
    raw = cv2.imread(str(source/raw_meta['image']), cv2.IMREAD_GRAYSCALE)
    if raw is None or old is None or raw.shape != old.shape:
        raise ValueError('missing or mismatched map images')
    resolution = float(meta['resolution'])
    if not 0 < resolution <= .02:
        raise ValueError('unsupported map resolution')
    # Free means ROS map-server known-free. Unknown and occupied are both solid.
    free = raw > 255 * (1.0 - float(raw_meta['free_thresh']))
    # Pad the edge as solid. Distance is centre-to-centre; subtract a half-cell
    # diagonal for each cell, so every point in an allowed cell keeps the margin
    # from every point in an occupied/unknown cell, including diagonal corners.
    distances = cv2.distanceTransform(np.pad(free.astype(np.uint8), 1),
                                     cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    map_clearance = distances * resolution - math.sqrt(2) * resolution
    rows, cols = np.indices(raw.shape)
    x = meta['origin'][0] + (cols + .5) * resolution
    y = meta['origin'][1] + (raw.shape[0] - rows - .5) * resolution
    a = layout['room_heading_rad']
    c, s = math.cos(a), math.sin(a)
    u, v = x*c + y*s, -x*s + y*c
    polygon = [(px*c + py*s, -px*s + py*c) for px, py in layout['room_polygon']]
    wall_clearance = np.minimum.reduce((u-min(p[0] for p in polygon),
                                       max(p[0] for p in polygon)-u,
                                       v-min(p[1] for p in polygon),
                                       max(p[1] for p in polygon)-v))
    env = layout['navigation_mask_validation']['envelope_room_uv']
    bin_clearance = np.hypot(np.maximum.reduce((env[0]-u, u-env[1], np.zeros_like(u))),
                             np.maximum.reduce((env[2]-v, v-env[3], np.zeros_like(v))))
    bin_buffer = float(layout['bin']['center_keepout_buffer_m'])
    if bin_buffer < .283 or layout['navigation_mask_valid_for_current_bin_placement'] is not True:
        raise ValueError('requires the approved measured bin envelope and original buffer')
    cell_radius = resolution / math.sqrt(2)
    candidate = (free & (map_clearance >= margin) &
                 (wall_clearance >= margin + cell_radius) &
                 (bin_clearance >= bin_buffer + cell_radius))
    # Preserve already-approved space; only additions must meet the new,
    # whole-cell-conservative geometry. Never remove old route cells by rounding.
    added = candidate & ~(old > 250)
    candidate |= old > 250
    destination.mkdir(parents=True, exist_ok=True)
    for name in ('scene_layout.json', 'navigation_center_mask.yaml', layout['map'],
                 raw_meta['image'], layout['preferred_patrol']['route_file']):
        shutil.copy2(source/name, destination/name)
    output = destination/meta['image']
    if not cv2.imwrite(str(output), np.where(candidate, 254, 0).astype(np.uint8)):
        raise ValueError('mask write failed')
    layout['center_wall_margin_m'] = margin
    bounds = (min(p[0] for p in polygon)+margin, max(p[0] for p in polygon)-margin,
              min(p[1] for p in polygon)+margin, max(p[1] for p in polygon)-margin)
    layout['center_allowed_polygon'] = [[pu*c-pv*s, pu*s+pv*c]
        for pu, pv in ((bounds[0], bounds[2]), (bounds[1], bounds[2]),
                       (bounds[1], bounds[3]), (bounds[0], bounds[3]))]
    layout['preferred_patrol']['general_wall_margin_m'] = margin
    layout['mask_rule'] = ('known free AND >=%.0fcm from occupied/unknown cells and fitted walls '
                          '(whole-cell conservative additions); existing bin envelope/buffer unchanged'
                          % (margin * 100))
    report = dict(schema=1, wall_map_margin_m=margin, turn_reserve_m=.06,
                  bin_buffer_m=bin_buffer, before_sha256=digest(source/meta['image']),
                  sha256=digest(output), raw_map_sha256=digest(source/raw_meta['image']),
                  old_allowed_cells=int((old > 250).sum()),
                  added_cells=int(added.sum()), allowed_cells=int(candidate.sum()),
                  added_area_m2=round(float(added.sum()) * resolution**2, 4),
                  physical_validation='offline only; next supervised run required')
    layout['navigation_mask_reduction_20261021'] = report
    (destination/'scene_layout.json').write_text(json.dumps(layout, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output), indent=2))
