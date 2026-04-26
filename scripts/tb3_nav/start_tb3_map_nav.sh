#!/usr/bin/env bash
set -euo pipefail

BASE_DIR=/home/ubuntu/tb3_nav
MAP_FILE="${1:-/home/ubuntu/maps/tb3_after_clean_restart.yaml}"
source "$BASE_DIR/tb3_ros_env.sh"

"$BASE_DIR/stop_tb3_nav_stack.sh"

nohup ros2 launch tb3_rplidar_c1 nav2.launch.py map:="$MAP_FILE" > /tmp/tb3_nav2_map.log 2>&1 &
echo "map_nav_pid=$!"

sleep 10
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" \
  >/tmp/tb3_zero_cmd.log 2>&1 || true

echo "Map Nav2 started with $MAP_FILE; logs: /tmp/tb3_nav2_map.log"
