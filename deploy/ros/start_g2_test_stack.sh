#!/bin/bash
# Start the ROS stack used for the G2 odom/AMCL acceptance (2026-09-28) on the Jetson.
#
# Each piece runs as a transient systemd unit (g2-roscore, g2-lidar, g2-map, g2-amcl,
# g2-rosbridge): nothing is installed, and they are gone after a reboot or
#   sudo systemctl stop g2-rosbridge g2-amcl g2-map g2-lidar g2-roscore
# No chassis driver and no robot_state_publisher: the grasp service publishes
# odom -> base_footprint, and the launch file below publishes the two static TFs.
#
# Needs passwordless sudo for systemd-run. Run it as the jetson user.
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
LAUNCH="$HERE/x3plus_tg30_navigation.launch"
MAP="${G2_MAP:-/home/jetson/x3plus/route_package/maps/site_map.yaml}"
AMCL="${G2_AMCL:-/home/jetson/ROS/X3/yahboomcar_ws/src/yahboomcar_nav/launch/library/amcl.launch}"
S='source /opt/ros/melodic/setup.bash; source /home/jetson/ydlidar_ws/devel/setup.bash; source /home/jetson/ROS/X3/yahboomcar_ws/devel/setup.bash;'

run() {
  sudo -n systemd-run --unit="$1" -p User=jetson -p Environment=HOME=/home/jetson \
    -p WorkingDirectory=/home/jetson /bin/bash -c "$S exec $2" >/dev/null 2>&1 \
    && echo "started $1" || echo "FAILED $1"
}

run g2-roscore "roscore"
for i in $(seq 1 30); do bash -c "$S rosnode list" >/dev/null 2>&1 && break; sleep 1; done
echo "master up after ${i}s"
run g2-lidar "roslaunch $LAUNCH"
run g2-map "rosrun map_server map_server $MAP"
run g2-amcl "roslaunch $AMCL"
run g2-rosbridge "roslaunch rosbridge_server rosbridge_websocket.launch port:=9090"
sleep 12
bash -c "$S rosnode list"
systemctl is-active g2-roscore g2-lidar g2-map g2-amcl g2-rosbridge
