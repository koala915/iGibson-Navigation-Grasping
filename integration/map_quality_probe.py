#!/usr/bin/env python3
import json
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node("codex_map_quality", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=8)
occupied = []
free = 0
unknown = 0
for index, value in enumerate(msg.data):
    if value < 0:
        unknown += 1
    elif value >= 50:
        occupied.append((index % msg.info.width, index // msg.info.width))
    else:
        free += 1
result = {
    "frame": msg.header.frame_id,
    "stamp_age_s": round(rospy.Time.now().to_sec() - msg.header.stamp.to_sec(), 3),
    "resolution_m": msg.info.resolution,
    "width": msg.info.width,
    "height": msg.info.height,
    "occupied_cells": len(occupied),
    "free_cells": free,
    "unknown_cells": unknown,
}
if occupied:
    result["occupied_bbox_m"] = {
        "xmin": round(msg.info.origin.position.x + min(x for x, _ in occupied) * msg.info.resolution, 3),
        "xmax": round(msg.info.origin.position.x + (max(x for x, _ in occupied) + 1) * msg.info.resolution, 3),
        "ymin": round(msg.info.origin.position.y + min(y for _, y in occupied) * msg.info.resolution, 3),
        "ymax": round(msg.info.origin.position.y + (max(y for _, y in occupied) + 1) * msg.info.resolution, 3),
    }
print(json.dumps(result, sort_keys=True))
