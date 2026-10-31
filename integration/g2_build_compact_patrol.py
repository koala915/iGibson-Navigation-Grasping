"""Build a hashed compact patrol mask offline; never connects to a robot."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

from g2_patrol_policy import Policy,read_pgm,compact_geometry_hash
from g2_patrol_entry import BODY_RADIUS,TURN_MARGIN


def build(scene,raw_scene,output):
    scene,raw_scene,output=map(Path,(scene,raw_scene,output))
    source=Policy(str(scene))
    layout=source.layout
    raw_meta=yaml.safe_load((raw_scene/'demo_box.yaml').read_text())
    if Path(raw_meta['image']).name!=raw_meta['image'] or raw_meta.get('negate',0)!=0:
        raise ValueError('raw map must be scene-relative and non-negated')
    raw_path=raw_scene/raw_meta['image']
    width,height,raw=read_pgm(raw_path)
    resolution=source.resolution
    if ((width,height)!=(source.width,source.height) or raw_meta['resolution']!=resolution
            or tuple(raw_meta['origin'])!=source.origin):
        raise ValueError('raw map geometry mismatch')
    raw_bytes=raw_path.read_bytes()
    expected=layout['navigation_mask_reduction_20261021']['raw_map_sha256']
    if hashlib.sha256(raw_bytes).hexdigest()!=expected:
        raise ValueError('raw map source hash mismatch')
    # The RAW map is inflated once, never the already-inflated centre mask.
    free=(np.array(raw).reshape(height,width)>250).astype(np.uint8)
    clearance=cv2.distanceTransform(free,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
    half=resolution/math.sqrt(2)
    radius=BODY_RADIUS+TURN_MARGIN
    wall_margin=.34
    angle=layout['room_heading_rad'];c,s=math.cos(angle),math.sin(angle)
    uv=[(x*c+y*s,-x*s+y*c) for x,y in layout['room_polygon']]
    left,right=min(x for x,y in uv),max(x for x,y in uv)
    bottom,top=min(y for x,y in uv),max(y for x,y in uv)
    u0,u1,v0,v1=layout['navigation_mask_validation']['envelope_room_uv']
    box_buffer=layout['bin']['center_keepout_buffer_m']
    cells=bytearray(width*height)
    for row in range(height):
        for col in range(width):
            x=source.origin[0]+(col+.5)*resolution
            y=source.origin[1]+(height-row-.5)*resolution
            u,v=x*c+y*s,-x*s+y*c
            wall=min(u-left,right-u,v-bottom,top-v)
            box=math.hypot(max(u0-u,0.,u-u1),max(v0-v,0.,v-v1))
            # One half-diagonal covers any centre in this free cell, the
            # other covers the occupied cell's extent, not another body radius.
            if (wall>=wall_margin+half and box>=box_buffer+half
                    and clearance[row,col]>=radius+2*half):cells[row*width+col]=255
    pgm=('P5\n%d %d\n255\n'%(width,height)).encode('ascii')+bytes(cells)
    layout['center_wall_margin_m']=wall_margin
    layout['boundary_recovery']={'enabled':True,'environment':'walls_and_fixed_bin',
        'body_radius_m':BODY_RADIUS,'error_budget_m':TURN_MARGIN,'hardware_validated':False}
    layout['compact_patrol']={'schema':1,'enabled':True,'body_radius_m':BODY_RADIUS,
        'physical_margin_m':TURN_MARGIN,'extra_turn_ring_m':0,
        'center_wall_margin_m':wall_margin,'bin_buffer_m':box_buffer,
        'mask_sha256':hashlib.sha256(pgm).hexdigest(),'raw_image':raw_meta['image'],
        'raw_sha256':hashlib.sha256(raw_bytes).hexdigest(),
        'grid':[width,height,resolution,list(source.origin)],
        'geometry_sha256':compact_geometry_hash(layout),'hardware_validated':False}
    layout['mask_rule']='raw known-free + swept body/error budget; physical turns, no extra ring'
    layout['preferred_patrol']['command_options']=['--destination','patrol-cycle','--boundary-micro','--keep-heading']
    layout['preferred_patrol']['validation']='offline compact-mask route checked; hardware validation pending'
    layout['state']='compact_patrol_offline_verified_hardware_pending'
    route=json.loads((scene/'outer_patrol_goals.json').read_text())
    # Keep the existing circuit, correcting only its three box-adjacent
    # vertices. They must allow an actual full-body turn, including margin.
    from g2_patrol_entry import EntryGeometry
    geometry=EntryGeometry(type('Scene',(),{'layout':layout})())
    target_clearance=radius+.04
    for item in route:
        x,y=item['point'];u,v=x*c+y*s,-x*s+y*c
        a,b=max(u0,min(u1,u)),max(v0,min(v1,v))
        distance=math.hypot(u-a,v-b)
        if distance<target_clearance:
            if distance==0:raise ValueError('route vertex inside physical box')
            u=a+(u-a)*target_clearance/distance;v=b+(v-b)*target_clearance/distance
            item['point']=[u*c-v*s,u*s+v*c]
    for a,b in zip(route,route[1:]+route[:1]):
        if not geometry.segment_allowed(a['point'],b['point']):
            raise ValueError('compact route does not retain physical swept clearance')
    layout['bin']['navigation_preparation']={
        'point_map':route[3]['point'],
        'face_point_approx_map':[layout['bin']['pose_map'][key] for key in ('x','y')],
        'validated':'offline navigation geometry only; hardware pending',
        'exact_release_dock_verified':False,
        'source':'compact route node; replaces historical preparation measurements'}
    output.mkdir(parents=True,exist_ok=True)
    (output/'navigation_center_mask.pgm').write_bytes(pgm)
    meta={'image':'navigation_center_mask.pgm','resolution':resolution,'origin':list(source.origin),
          'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    (output/'navigation_center_mask.yaml').write_text(yaml.safe_dump(meta),encoding='utf-8')
    (output/'scene_layout.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (output/'outer_patrol_goals.json').write_text(json.dumps(route,indent=2)+'\n',encoding='utf-8')
    if output.resolve()!=raw_scene.resolve():
        (output/raw_meta['image']).write_bytes(raw_bytes)
        shutil.copyfile(raw_scene/'demo_box.yaml',output/'demo_box.yaml')
    policy=Policy(str(output))
    from g2_navigation_goals import resolve_destination
    resolve_destination(policy,(.5,.5),'patrol-cycle')
    return policy


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True)
    parser.add_argument('--raw-scene',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    policy=build(args.scene,args.raw_scene,args.output)
    print(json.dumps({'allowed_cells':sum(v>250 for v in policy.cells),
                      'area_m2':sum(v>250 for v in policy.cells)*policy.resolution**2}))
