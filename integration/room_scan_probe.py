#!/usr/bin/env python3
import json
import math
import rospy
from sensor_msgs.msg import LaserScan

rospy.init_node("codex_room_scan", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message("/scan", LaserScan, timeout=5)
bins = {"front": [], "left": [], "right": [], "rear": []}
points = []
for i, distance in enumerate(msg.ranges):
    if not math.isfinite(distance) or not msg.range_min <= distance <= min(msg.range_max, 3.0):
        continue
    angle = msg.angle_min + i * msg.angle_increment + math.pi
    angle = (angle + math.pi) % (2 * math.pi) - math.pi
    x = 0.10 + distance * math.cos(angle)
    y = distance * math.sin(angle)
    points.append((x, y))
    degrees = math.degrees(angle)
    if abs(degrees) <= 15:
        bins["front"].append(x)
    if abs(abs(degrees) - 180) <= 15:
        bins["rear"].append(-x)
    if abs(degrees - 90) <= 15:
        bins["left"].append(y)
    if abs(degrees + 90) <= 15:
        bins["right"].append(-y)

def stats(values):
    values = sorted(value for value in values if value > 0)
    if not values:
        return None
    return {"min": round(values[0], 3),
            "p10": round(values[max(0, int(len(values) * 0.1) - 1)], 3),
            "n": len(values)}

print(json.dumps({
    "stamp_age": round(rospy.Time.now().to_sec() - msg.header.stamp.to_sec(), 3),
    "points": len(points),
    "bounds": {"xmin": round(min(x for x, _ in points), 3),
               "xmax": round(max(x for x, _ in points), 3),
               "ymin": round(min(y for _, y in points), 3),
               "ymax": round(max(y for _, y in points), 3)},
    "sectors": {name: stats(values) for name, values in bins.items()},
}, sort_keys=True))
