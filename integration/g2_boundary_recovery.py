"""Python 2/3 bounded escape from a SOFT centre mask in a walls-only scene.

No I/O or motion here. The caller must keep localization, sensor freshness,
sole-owner and arm-travel gates. A 25cm body plus a 10cm error budget must
remain inside fitted walls and outside the fixed placement-box envelope.
"""
from __future__ import division
import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class RecoveryPolicy:
    def __init__(self, policy):
        self.policy = policy
        config = policy.layout.get('boundary_recovery', {})
        if config.get('enabled') is not True:
            raise ValueError('boundary recovery is not configured for this scene')
        if config.get('environment') != 'walls_and_fixed_bin':
            raise ValueError('boundary recovery requires the declared empty-room environment')
        self.radius = float(config['body_radius_m']) + float(config['error_budget_m'])
        if not (.25 <= self.radius <= .35 and config['body_radius_m'] == .25
                and config['error_budget_m'] >= .10):
            raise ValueError('unapproved recovery geometry')
        self.limit = .60
        angle = policy.layout['room_heading_rad']
        self.c, self.s = math.cos(angle), math.sin(angle)
        uv = [self.uv(p) for p in policy.layout['room_polygon']]
        self.bounds = (min(p[0] for p in uv), min(p[1] for p in uv),
                       max(p[0] for p in uv), max(p[1] for p in uv))
        u0, u1, v0, v1 = policy.layout['navigation_mask_validation']['envelope_room_uv']
        self.bin = (u0, v0, u1, v1)

    def uv(self, point):
        return (point[0] * self.c + point[1] * self.s,
                -point[0] * self.s + point[1] * self.c)

    def clearance(self, point):
        u, v = self.uv(point)
        u0, v0, u1, v1 = self.bounds
        b0, b1, b2, b3 = self.bin
        box = math.hypot(max(b0 - u, 0, u - b2), max(b1 - v, 0, v - b3))
        return min(u - u0, u1 - u, v - v0, v1 - v, box)

    def hard_allowed(self, point):
        return (all(not math.isnan(x) and not math.isinf(x) for x in point[:2])
                and self.clearance(point) >= self.radius)

    def segment(self, a, b):
        count = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / .005)))
        # Half-step guard covers the clearance function between samples.
        return all(self.clearance((a[0] + (b[0] - a[0]) * i / count,
                                   a[1] + (b[1] - a[1]) * i / count)) >= self.radius + .0025
                   for i in range(count + 1))

    def recovered(self, point):
        # Return far enough inside the soft mask to allow normal turns again.
        return (self.hard_allowed(point) and self.policy.allowed(*point[:2])
                and all(self.policy.allowed(point[0] + .07 * math.cos(k * math.pi / 8),
                                            point[1] + .07 * math.sin(k * math.pi / 8))
                        for k in range(16)))

    def target(self, pose):
        if not self.hard_allowed(pose):
            raise ValueError('physical wall/bin clearance insufficient for automatic escape')
        if self.recovered(pose):
            return tuple(pose[:2])
        candidates = []
        # Nearby known-free CENTRES are destinations. The escape corridor uses
        # fitted physical geometry, so an inflated mask cannot trap the robot.
        for i in range(72):
            angle = i * math.pi / 36
            for j in range(1, 31):
                distance = j * .02
                p = (pose[0] + distance * math.cos(angle), pose[1] + distance * math.sin(angle))
                if self.recovered(p) and self.segment(pose, p):
                    candidates.append((distance, -self.clearance(p), p))
                    break
        if not candidates:
            raise ValueError('no bounded physical escape corridor into the navigation mask')
        return min(candidates)[2]

    def command(self, pose, target, measured_vx=0.):
        if not self.segment(pose, target):
            raise ValueError('escape corridor lost after localization update')
        bearing = math.atan2(target[1] - pose[1], target[0] - pose[0])
        # Forward only: the team allows reversing only in the enoch approach
        # (koala, 2026-10-12), and nothing sees behind the robot. A target
        # behind is faced by turning in place; the clearance circle bounds
        # every heading, so that turn needs no more room than a smaller one.
        sign, error = 1, wrap(bearing - pose[2])
        if abs(error) > math.radians(4):
            if abs(measured_vx) > .01:
                raise ValueError('must stop translation before an escape turn')
            # Circle bounds every heading. The 3cm tracking/stop allocation
            # covers the bounded 0.08s micro-turn; no 6cm soft-mask ring here.
            return 0., (1. if error > 0 else -1.)
        stop_distance = .35 * .08 + .03
        end = (pose[0] + sign * stop_distance * math.cos(pose[2]),
               pose[1] + sign * stop_distance * math.sin(pose[2]))
        if not self.segment(pose, end):
            raise ValueError('escape predicted stop would violate physical geometry')
        if self.clearance(end) + .005 < self.clearance(pose):
            raise ValueError('escape would reduce nearest wall/bin clearance')
        return sign * .08, 0.
