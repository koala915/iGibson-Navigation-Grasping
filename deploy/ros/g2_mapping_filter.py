#!/usr/bin/env python3
"""Mapping-only TG30 filter; raw /scan remains the motion safety source.

Output /scan_mapping is dedicated to GMapping maxRange=3.0. Masked rays
are 4.0 (> output range_max=3.0) so OpenSLAM skips them, never clears them.
Preserves array order, laser frame and acquisition times (raw forward is pi).
"""
import math
import copy


def filter_ranges(ranges, angle_min, increment, range_min=0.1,
                  front_only=True, footprint=False, speckle=False):
    if not (0 < increment < 0.1 and len(ranges) <= 4096):
        raise ValueError('unsupported scan geometry')
    result = []
    for i, r in enumerate(ranges):
        angle = (angle_min+i*increment+math.pi+math.pi) % (2*math.pi)-math.pi
        valid = math.isfinite(r) and range_min <= r <= 3.0
        if front_only and abs(angle) > math.pi/2:
            valid = False
        if valid and footprint:
            x, y = 0.10+r*math.cos(angle), r*math.sin(angle)
            if -0.15 <= x <= 0.15 and abs(y) <= 0.12:
                valid = False
        if valid and speckle:
            # Mapping only. Thin real objects can also be removed; opt in.
            neighbors = ranges[max(0,i-1):i]+ranges[i+1:i+2]
            if not any(math.isfinite(v) and abs(v-r) <= 0.05 for v in neighbors):
                valid = False
        result.append(float(r) if valid else 4.0)
    return result


def main():
    import rospy
    from sensor_msgs.msg import LaserScan
    from pathlib import Path
    rospy.init_node('g2_mapping_filter')
    reserve = float(rospy.get_param('~min_available_mb', 768))
    if not math.isfinite(reserve) or reserve < 768:
        raise ValueError('RAM reserve must be >=768 MB')
    publisher = rospy.Publisher('/scan_mapping', LaserScan, queue_size=1)

    def callback(scan):
        try:
            fields = dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())
            if int(fields['MemAvailable'].split()[0])/1024 < reserve:
                rospy.signal_shutdown('RAM_pressure')
                return
            age = (rospy.Time.now()-scan.header.stamp).to_sec()
            if scan.header.frame_id != 'laser' or not 0 <= age <= 0.35:
                rospy.logwarn_throttle(2, 'mapping scan wrong frame/stale')
                return
            out = copy.copy(scan)
            out.ranges = filter_ranges(list(scan.ranges), scan.angle_min,
                                       scan.angle_increment, scan.range_min,
                                       rospy.get_param('~front_only', True),
                                       rospy.get_param('~footprint', False),
                                       rospy.get_param('~speckle', False))
            out.range_max = 3.0
            publisher.publish(out)
        except (ValueError, KeyError, OSError):
            rospy.signal_shutdown('invalid scan or memory telemetry')

    rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
