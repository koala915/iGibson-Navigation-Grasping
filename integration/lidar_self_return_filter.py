"""Opt-in, session-local self-return profile for supervised calibration.

Capture only while stopped in operator-confirmed clear surroundings. Never
learn during motion. This does not publish /scan or change navigation defaults.
An object at exactly the learned angle/range is indistinguishable from self.
"""
import math


class SelfReturnFilter:
    def __init__(self, ranges_by_angle, tolerance_m=0.025):
        if not 0 < tolerance_m <= 0.025:
            raise ValueError('tolerance must be within (0, 0.025] m')
        self.ranges_by_angle = dict(ranges_by_angle)
        self.tolerance_m = tolerance_m

    @staticmethod
    def key(angle):
        # One-degree bins tolerate TG30 sample placement jitter. Require range
        # matching as well; this is not a blind angular sector.
        return round(angle)

    @classmethod
    def from_clear_scans(cls, scans, max_range_m=0.75):
        if len(scans) < 10 or not 0 < max_range_m <= 0.75:
            raise ValueError('need >=10 distinct clear scans and bounded range')
        if len({s.stamp for s in scans}) != len(scans):
            raise ValueError('duplicate scans cannot establish persistence')
        bins = {}
        for scan in scans:
            seen = {}
            for angle, distance in scan.points:
                if math.isfinite(angle) and math.isfinite(distance) and 0.1 <= distance <= max_range_m:
                    seen.setdefault(cls.key(angle), []).append(distance)
            for key, distances in seen.items():
                bins.setdefault(key, []).append(distances)
        # Keep separate observed ranges rather than masking the whole min/max
        # interval. A range must recur in >=80% of independent frames.
        profile = {}
        for key, frames in bins.items():
            candidates = sorted({round(d, 3) for frame in frames for d in frame})
            recurring = [d for d in candidates if sum(
                any(abs(d - observed) <= .025 for observed in frame)
                for frame in frames) >= math.ceil(.8 * len(scans))]
            if recurring:
                profile[key] = recurring
        return cls(profile)

    def apply(self, points):
        kept = []
        for angle, distance in points:
            baseline = self.ranges_by_angle.get(self.key(angle))
            values = baseline if isinstance(baseline, list) else [baseline]
            if baseline is None or not math.isfinite(distance) or not any(
                    abs(distance - value) <= self.tolerance_m for value in values):
                kept.append((angle, distance))
        return tuple(kept)
