#!/usr/bin/env python
"""Snapshot AMCL + odom for the G2 acceptance, section 3 (ROS Melodic, Python 2, on the Jetson).

usage: amcl_probe.py LABEL [N INTERVAL]
AMCL only updates on motion, so each snapshot first asks for a no-motion update
and then takes the NEXT /amcl_pose, never a cached one.
"""
import json, math, sys, time
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

label = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 1
every = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
rospy.init_node("g2_probe", anonymous=True, disable_signals=True)
odom = {}
rospy.Subscriber("/odom_setmotor", Odometry,
                 lambda m: odom.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                                       yaw=yaw(m.pose.pose.orientation), t=m.header.stamp.to_sec()))
rospy.wait_for_service("/request_nomotion_update", 5.0)
nomotion = rospy.ServiceProxy("/request_nomotion_update", Empty)
time.sleep(0.5)
for i in range(n):
    got = []
    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, lambda m: got.append(m))
    t0 = time.time()
    while not got and time.time() - t0 < 5.0:
        try:
            nomotion()
        except Exception:
            pass
        time.sleep(0.3)
    sub.unregister()
    if not got:
        print(json.dumps({"label": label, "i": i, "error": "no fresh /amcl_pose in 5 s"}))
    else:
        m = got[0]
        c = m.pose.covariance
        p = m.pose.pose
        print(json.dumps({"label": label, "i": i, "t": round(time.time(), 2),
                          "amcl": [round(p.position.x, 4), round(p.position.y, 4),
                                   round(math.degrees(yaw(p.orientation)), 2)],
                          "var_xy": [round(c[0], 5), round(c[7], 5)],
                          "var_yaw_deg2": round(c[35] * (180 / math.pi) ** 2, 3),
                          "odom": [round(odom.get("x", float("nan")), 4),
                                   round(odom.get("y", float("nan")), 4),
                                   round(math.degrees(odom.get("yaw", float("nan"))), 2)]}))
    sys.stdout.flush()
    if i + 1 < n:
        time.sleep(every)
