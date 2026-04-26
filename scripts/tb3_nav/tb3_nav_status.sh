#!/usr/bin/env bash
set -euo pipefail

BASE_DIR=/home/ubuntu/tb3_nav
source "$BASE_DIR/tb3_ros_env.sh"

echo "== processes =="
ps -eo pid,etime,args | grep -E '[r]os2 launch turtlebot3|[r]os2 launch tb3_rplidar_c1|[n]av2_|[c]artographer|[s]llidar|[t]urtlebot3_ros|[r]obot_state_publisher' || true

echo
echo "== topics =="
timeout 5 ros2 topic list -t | sort | grep -E '/(scan|odom|tf|map|cmd_vel|navigate|goal|global_costmap|local_costmap)' || true

echo
echo "== actions =="
timeout 5 ros2 action list -t | sort | grep -E 'navigate|follow|compute|spin|backup' || true
