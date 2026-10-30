"""Physical turn and bounded startup reverse checks, Python 2/3, no I/O."""
from __future__ import division
import math

try:
    from .g2_body_geometry import BODY_RECTANGLES,back_path_problem,wall_distance,to_map
except (ImportError, ValueError):  # Python 2: relative import outside a package
    from g2_body_geometry import BODY_RECTANGLES,back_path_problem,wall_distance,to_map

BODY_RADIUS = max(math.hypot(x, y) for x1, y1, x2, y2 in BODY_RECTANGLES
                  for x, y in ((x1, y1), (x1, y2), (x2, y1), (x2, y2)))
# 2cm (3 sigma) localization plus 1cm translation during an isolated turn.
TURN_MARGIN = .03
MAX_POSITION_SIGMA = .02 / 3


class EntryGeometry:
    def __init__(self, policy):
        self.policy = policy
        self.walls = policy.layout['room_polygon']
        cfg = policy.layout.get('boundary_recovery', {})
        if (cfg.get('environment') != 'walls_and_fixed_bin' and
                policy.layout.get('interior_lidar_clusters') != []):
            raise ValueError('entry requires walls_and_fixed_bin environment')
        angle = policy.layout['room_heading_rad']
        self.c, self.s = math.cos(angle), math.sin(angle)
        # This is the measured/candidate physical box envelope including its
        # registration uncertainty, NOT the centre mask's body-radius inflation.
        self.bin = policy.layout['navigation_mask_validation']['envelope_room_uv']
        if len(self.bin) != 4 or not all(not math.isnan(v) and not math.isinf(v) for v in self.bin):
            raise ValueError('invalid box envelope')
        if self.bin[0] >= self.bin[1] or self.bin[2] >= self.bin[3]:
            raise ValueError('invalid box envelope')

    def box_distance(self, pose):
        u, v = pose[0]*self.c + pose[1]*self.s, -pose[0]*self.s + pose[1]*self.c
        u0, u1, v0, v1 = self.bin
        return math.hypot(max(u0-u, 0., u-u1), max(v0-v, 0., v-v1))

    def turn_allowed(self, pose):
        # The exact full-yaw sweep of all body rectangles is their outer circle.
        # No raster quantization and no second 6cm box-ring inflation.
        return (all(not math.isnan(v) and not math.isinf(v) for v in pose)
                and wall_distance(pose[:2], self.walls) > BODY_RADIUS + TURN_MARGIN
                and self.box_distance(pose) > BODY_RADIUS + TURN_MARGIN)

    def box_turn_allowed(self, pose):
        # Only replace the extra BOX ring. Keep the wall reserve and the
        # legacy mask rule everywhere away from the fixed bin.
        wall_reserve=self.policy.layout['center_wall_margin_m']+.06
        return (self.box_distance(pose)<=BODY_RADIUS+.12
                and wall_distance(pose[:2],self.walls)>=wall_reserve
                and self.turn_allowed(pose))

    def reverse_problem(self, pose, distance):
        if not 0 < distance <= .10:
            return 'startup reverse distance limit'
        return self.translation_problem(pose,-distance)

    def translation_problem(self, pose, forward):
        distance=-forward
        u0, u1, v0, v1 = self.bin
        u, v = (u0+u1)/2, (v0+v1)/2
        obstacle = (u*self.c-v*self.s, u*self.s+v*self.c,
                    math.hypot(u1-u0, v1-v0)/2)
        problem = back_path_problem(pose, distance, self.walls, obstacles=[obstacle]) if forward<0 else None
        if problem:return problem
        # Unlike the post-release escape, startup never tolerates an existing
        # intersection. Bound every translated body corner against the box.
        for x1,y1,x2,y2 in BODY_RECTANGLES:
            left,right=x1+min(0.,forward),x2+max(0.,forward)
            swept=[to_map(pose,p) for p in ((left,y1),(right,y1),(right,y2),(left,y2))]
            box=[(a*self.c-b*self.s,a*self.s+b*self.c) for a,b in
                 ((u0,v0),(u1,v0),(u1,v1),(u0,v1))]
            axes=[(self.c,self.s),(-self.s,self.c),(math.cos(pose[2]),math.sin(pose[2])),
                  (-math.sin(pose[2]),math.cos(pose[2]))]
            separated=False
            for ax,ay in axes:
                a=[x*ax+y*ay for x,y in swept];b=[x*ax+y*ay for x,y in box]
                if max(a)+TURN_MARGIN<min(b) or max(b)+TURN_MARGIN<min(a):separated=True;break
            if not separated:return 'startup body path intersects box reserve'
            if any(wall_distance(p,self.walls)<=TURN_MARGIN for p in swept):
                return 'startup body path intersects wall reserve'
        return None

    def segment_allowed(self, pose, end):
        count=max(1,int(math.ceil(math.hypot(end[0]-pose[0],end[1]-pose[1])/.002)))
        # 1mm half-step guard bounds the clearance between centre samples.
        for i in range(count+1):
            p=(pose[0]+(end[0]-pose[0])*i/count,pose[1]+(end[1]-pose[1])*i/count)
            if (wall_distance(p,self.walls)<=BODY_RADIUS+TURN_MARGIN+.001 or
                    self.box_distance(p)<=BODY_RADIUS+TURN_MARGIN+.001):return False
        return True


def uncertainty_allowed(state, directional=False):
    return (all(0 <= v <= MAX_POSITION_SIGMA**2 for v in state['cov'][:2])
            and (not directional or 0<=state['cov'][2]<=(math.radians(1)/3)**2))
