#!/bin/bash
set -eo pipefail
# Reuse only the existing address-discovery function, never its motor-server main.
source /usr/local/libexec/x3plus/start-navigation.sh
# The local ROS graph must also start while Wi-Fi/DHCP is still coming up.
# Remote tools use rosbridge; loopback remains valid for on-board nodes.
ROS_IP="$(discover_ros_ip 2>/dev/null || printf '127.0.0.1')"
export ROS_IP ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME
set +u
source /opt/ros/melodic/setup.bash
source /home/jetson/ydlidar_ws/devel/setup.bash
source /home/jetson/ROS/X3/yahboomcar_ws/devel/setup.bash
case "$1" in
    core) exec roscore ;;
    lidar) exec roslaunch /home/jetson/Documents/deploy_jetson2/deploy/ros/x3plus_tg30_navigation.launch ;;
    bridge) exec roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 ;;
    *) echo "Unknown ROS component: $1" >&2; exit 2 ;;
esac
