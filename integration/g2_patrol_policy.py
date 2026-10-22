"""Small Python 2/3 compatible centre-mask policy for supervised map patrol."""
from __future__ import division
import json
import math
import os
import heapq


def finite(x):
    return not math.isnan(x) and not math.isinf(x)


def wrap(x):
    return (x+math.pi)%(2*math.pi)-math.pi


def read_pgm(path):
    with open(path,'rb') as f:
        def tokens():
            while True:
                line=f.readline()
                if not line:
                    raise ValueError('incomplete PGM header')
                line=line.split(b'#',1)[0]
                for word in line.split():
                    yield word
        words=tokens()
        if next(words)!=b'P5':
            raise ValueError('requires binary PGM')
        width,height,maximum=int(next(words)),int(next(words)),int(next(words))
        if maximum!=255 or not 0<width*height<=4000000:
            raise ValueError('unsupported PGM dimensions/range')
        raw=bytearray(f.read())
    if len(raw)!=width*height:
        raise ValueError('PGM length mismatch')
    return width,height,raw


class Policy:
    def __init__(self,folder):
        import yaml
        with open(os.path.join(folder,'scene_layout.json')) as f:
            self.layout=json.load(f)
        with open(os.path.join(folder,'navigation_center_mask.yaml')) as f:
            meta=yaml.safe_load(f)
        # 2026-10-21: operator requested a smaller no-go region. Below the old
        # 40cm limit accept only the rebuilt, hashed scene, down to 35cm. Keep
        # the runner's independent 6cm turn reserve and all motion-ready gates.
        margin=self.layout.get('center_wall_margin_m',0)
        if self.layout.get('frame_id')!='map' or not finite(margin) or margin<.35:
            raise ValueError('unapproved scene policy')
        self.resolution=float(meta['resolution'])
        self.origin=tuple(float(x) for x in meta['origin'])
        if self.resolution<=0 or self.origin[2]!=0:
            raise ValueError('unsupported mask geometry')
        self.width,self.height,self.cells=read_pgm(os.path.join(folder,meta['image']))
        if margin<.4:
            import hashlib
            record=self.layout.get('navigation_mask_reduction_20261021',{})
            with open(os.path.join(folder,meta['image']),'rb') as f:
                digest=hashlib.sha256(f.read()).hexdigest()
            if (record.get('sha256')!=digest or
                    record.get('wall_map_margin_m')!=margin or
                    record.get('bin_buffer_m')!=self.layout['bin'].get('center_keepout_buffer_m') or
                    record.get('bin_buffer_m',0)<.283 or record.get('turn_reserve_m')!=.06):
                raise ValueError('reduced mask approval/geometry mismatch')
        self.folder=folder;self.dock_cells=None

    def require_geometry(self,current_offset):
        record=self.layout.get('sensor_geometry',{})
        built=record.get('map_built_lidar_forward_m')
        checked=record.get('validation',{})
        if built is not None and abs(float(built)-current_offset)<1e-6:return
        if checked.get('verified') is True and abs(float(checked.get('lidar_forward_m',-1))-current_offset)<1e-6:return
        raise ValueError('map/LiDAR geometry requires revalidation before motion')

    def enable_dock_zone(self,current_offset):
        self.require_geometry(current_offset)
        cfg=self.layout['bin'].get('dock_zone',{})
        if cfg.get('enabled') is not True or self.layout['bin'].get('registration_verified') is not True:
            raise ValueError('local docking zone requires verified physical box registration')
        if cfg.get('resolution')!=self.resolution or tuple(cfg.get('origin',[]))!=self.origin or cfg.get('center_wall_margin_m',0)<.30:
            raise ValueError('dock mask metadata mismatch')
        import hashlib
        with open(os.path.join(self.folder,cfg['image']),'rb') as f:digest=hashlib.sha256(f.read()).hexdigest()
        if digest!=cfg.get('sha256'):raise ValueError('dock mask digest mismatch')
        width,height,cells=read_pgm(os.path.join(self.folder,cfg['image']))
        if (width,height)!=(self.width,self.height):raise ValueError('dock mask geometry mismatch')
        self.dock_cells=cells

    def require_motion_ready(self,current_offset):
        self.require_geometry(current_offset)
        if self.layout.get('navigation_mask_valid_for_current_bin_placement') is False:
            raise ValueError('box moved; re-register box and rebuild navigation mask before motion')

    def allowed(self,x,y):
        if not finite(x) or not finite(y):
            return False
        col=int(math.floor((x-self.origin[0])/self.resolution))
        row=self.height-1-int(math.floor((y-self.origin[1])/self.resolution))
        if not (0<=col<self.width and 0<=row<self.height):return False
        i=row*self.width+col
        return self.cells[i]>250 or (self.dock_cells is not None and self.dock_cells[i]>250)

    def segment_allowed(self,start,end):
        distance=math.hypot(end[0]-start[0],end[1]-start[1])
        count=max(1,int(math.ceil(distance/(self.resolution*.5))))
        return all(self.allowed(start[0]+(end[0]-start[0])*i/count,
                                start[1]+(end[1]-start[1])*i/count)
                   for i in range(count+1))

    def stopping_path_allowed(self,pose,distance):
        if not finite(distance) or distance<0:
            return False
        return self.segment_allowed(pose[:2],(pose[0]+distance*math.cos(pose[2]),
                                                pose[1]+distance*math.sin(pose[2])))

    def plan(self,start,goal):
        """Bounded A* for legal centres, with no diagonal corner cutting."""
        start,goal=tuple(start[:2]),tuple(goal[:2])
        if not self.allowed(*start):raise ValueError('start outside navigation mask')
        if not self.allowed(*goal):raise ValueError('goal outside navigation mask')
        if self.segment_allowed(start,goal):return [start,goal]
        def cell(p):return (int(math.floor((p[0]-self.origin[0])/self.resolution)),int(math.floor((p[1]-self.origin[1])/self.resolution)))
        def point(c):return (self.origin[0]+(c[0]+.5)*self.resolution,self.origin[1]+(c[1]+.5)*self.resolution)
        def good(c):return self.allowed(*point(c))
        first,last=cell(start),cell(goal);queue=[(0,first)];cost={first:0};parent={};closed=set()
        while queue:
            _,current=heapq.heappop(queue)
            if current in closed:continue
            closed.add(current)
            if len(closed)>20000:raise ValueError('path search budget exceeded')
            if current==last:break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(current[0]+dx,current[1]+dy)
                if not good(nxt) or (dx and dy and (not good((current[0]+dx,current[1])) or not good((current[0],current[1]+dy)))):continue
                value=cost[current]+math.hypot(dx,dy)
                if value<cost.get(nxt,float('inf')):
                    cost[nxt]=value;parent[nxt]=current
                    heapq.heappush(queue,(value+math.hypot(nxt[0]-last[0],nxt[1]-last[1]),nxt))
        else:raise ValueError('no legal route')
        cells=[last]
        while cells[-1]!=first:cells.append(parent[cells[-1]])
        raw=[start]+[point(c) for c in reversed(cells)][1:-1]+[goal]
        # Keep only line-of-sight shortcuts that pass every half-cell sample.
        path=[raw[0]];i=0
        while i<len(raw)-1:
            j=len(raw)-1
            while j>i+1 and not self.segment_allowed(raw[i],raw[j]):j-=1
            if not self.segment_allowed(raw[i],raw[j]):raise ValueError('discrete path endpoint mismatch')
            path.append(raw[j]);i=j
        return path


    def patrol_points(self,inset=.60):
        # Photo box is at u/v maximum; the opposite corner is u/v minimum.
        a=self.layout['room_heading_rad']
        c,s=math.cos(a),math.sin(a)
        uv=[(x*c+y*s,-x*s+y*c) for x,y in self.layout['room_polygon']]
        x0,x1=min(p[0] for p in uv)+inset,max(p[0] for p in uv)-inset
        y0,y1=min(p[1] for p in uv)+inset,max(p[1] for p in uv)-inset
        if x0>=x1 or y0>=y1:
            raise ValueError('room too small')
        points=[(x0*c-y0*s,x0*s+y0*c),(x1*c-y0*s,x1*s+y0*c),
                (x1*c-y1*s,x1*s+y1*c),(x0*c-y1*s,x0*s+y1*c)]
        for start,end in zip(points,points[1:]+points[:1]):
            if not self.segment_allowed(start,end):
                raise ValueError('patrol segment outside centre mask')
        return points

def guided_heading(position,start,goal,lookahead=.18):
    """Look ahead on the original segment rather than cutting from a short stop."""
    dx,dy=goal[0]-start[0],goal[1]-start[1];length=math.hypot(dx,dy)
    if length<1e-9:return math.atan2(goal[1]-position[1],goal[0]-position[0])
    along=((position[0]-start[0])*dx+(position[1]-start[1])*dy)/length
    t=min(length,max(0,along)+lookahead)/length
    return math.atan2(start[1]+dy*t-position[1],start[0]+dx*t-position[0])


def stop_reserve(duration):
    # Isolated <=0.15s pulses are much shorter than continuous cruise. The
    # 6cm coast allowance still exceeds the measured 0.08s micro-move by several
    # times; longer pulses retain the original 15cm allowance.
    return .06 if duration<=.15 else .15
