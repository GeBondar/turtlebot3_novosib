#!/usr/bin/env bash
set -euo pipefail

BASE_DIR=/home/ubuntu/tb3_nav
source "$BASE_DIR/tb3_ros_env.sh"

"$BASE_DIR/stop_tb3_nav_stack.sh"

nohup ros2 launch tb3_rplidar_c1 nav2_slam.launch.py > /tmp/tb3_nav2_slam.log 2>&1 &
echo "slam_nav_pid=$!"

sleep 10
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" \
  >/tmp/tb3_zero_cmd.log 2>&1 || true

echo "SLAM+Nav2 started; logs: /tmp/tb3_nav2_slam.log"
