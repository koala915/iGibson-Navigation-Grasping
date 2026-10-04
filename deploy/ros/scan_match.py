#!/usr/bin/env python
"""Is AMCL where the laser says it is? (ROS Melodic, Python 2, run on the Jetson)

AMCL's covariance is the spread of its particles, not the quality of the fit:
after the robot is moved by hand it can stay tight around a pose that is wrong.
This counts the /scan returns that land within 2 cells (~10 cm) of an occupied
map cell, using the live map->laser TF.

  scan_match.py [LABEL]            score at the current AMCL pose
  scan_match.py [LABEL] --sweep    also try heading offsets of -90..+90 deg (and
                                   +-0.2 m shifts); if the best is not offset 0,
                                   the pose is off and needs a new 2D Pose Estimate

2026-09-28: a correct pose scored 77-89 %, and neighbouring headings (+-10 deg)
dropped to about 62 %. One reading of 53 % came from a person standing next to
the robot and recovered to 88 % on the next scan.
"""
import math
import sys

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

label = next((a for a in sys.argv[1:] if not a.startswith("--")), "match")
rospy.init_node("scan_match", anonymous=True, disable_signals=True)
grid = rospy.wait_for_message("/map", OccupancyGrid, 10.0)
W, H, res = grid.info.width, grid.info.height, grid.info.resolution
ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
near = set()
for i, v in enumerate(grid.data):
    if v >= 65:
        cx, cy = i % W, i // W
        for dx in (-2, -1, 0, 1, 2):
            for dy in (-2, -1, 0, 1, 2):
                near.add((cx + dx, cy + dy))
listener = tf.TransformListener()
scan = rospy.wait_for_message("/scan", LaserScan, 5.0)
listener.waitForTransform("map", scan.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
(t, q) = listener.lookupTransform("map", scan.header.frame_id, rospy.Time(0))
yaw0 = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def score(dyaw=0.0, dx=0.0, dy=0.0):
    hit = n = 0
    for k, r in enumerate(scan.ranges):
        if not (scan.range_min < r < min(scan.range_max, 8.0)):
            continue
        a = scan.angle_min + k * scan.angle_increment + yaw0 + dyaw
        cx = int((t[0] + dx + r * math.cos(a) - ox) / res)
        cy = int((t[1] + dy + r * math.sin(a) - oy) / res)
        if 0 <= cx < W and 0 <= cy < H:
            n += 1
            hit += (cx, cy) in near
    return 100.0 * hit / max(n, 1), n


pct, n = score()
print("%s: %.0f%% of %d in-map returns on a wall (laser at %.2f, %.2f, %.1f deg)"
      % (label, pct, n, t[0], t[1], math.degrees(yaw0)))
if "--sweep" in sys.argv:
    rows = []
    for d in range(-90, 91, 5):
        rows.append(max((score(math.radians(d), sx, sy)[0], d, sx, sy)
                        for sx in (-0.2, -0.1, 0.0, 0.1, 0.2)
                        for sy in (-0.2, -0.1, 0.0, 0.1, 0.2)))
    for p, d, sx, sy in sorted(rows, reverse=True)[:5]:
        print("  heading offset %+4d deg, shift %+.1f %+.1f m: %.0f%%" % (d, sx, sy, p))
