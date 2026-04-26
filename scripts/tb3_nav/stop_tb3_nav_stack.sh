#!/usr/bin/env bash
set -euo pipefail

patterns=(
  'ros2 launch tb3_rplidar_c1 nav2.launch.py'
  'ros2 launch tb3_rplidar_c1 nav2_slam.launch.py'
  'ros2 launch tb3_rplidar_c1 cartographer.launch.py'
  '/opt/ros/jazzy/lib/nav2_'
  '/opt/ros/jazzy/lib/cartographer_ros/'
  '/opt/ros/jazzy/lib/rviz2/rviz2'
)

for pattern in "${patterns[@]}"; do
  pids="$(pgrep -f "$pattern" || true)"
  if [ -n "$pids" ]; then
    kill -INT $pids || true
  fi
done

sleep 4

for pattern in "${patterns[@]}"; do
  pids="$(pgrep -f "$pattern" || true)"
  if [ -n "$pids" ]; then
    kill -TERM $pids || true
  fi
done

sleep 1
ps -eo pid,etime,args | grep -E '[n]av2_|[c]artographer|[r]viz2|ros2 launch tb3_rplidar_c1' || true
