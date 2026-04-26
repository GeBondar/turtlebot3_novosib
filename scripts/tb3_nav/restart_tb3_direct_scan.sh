#!/usr/bin/env bash
set -eo pipefail

patterns=(
  '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch turtlebot3_bringup robot.launch.py'
  '^/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher'
  '^/home/ubuntu/turtlebot3_ws/install/sllidar_ros2/lib/sllidar_ros2/sllidar_node'
  '^/usr/bin/python3 /home/ubuntu/turtlebot3_ws/install/scan_rectifier/lib/scan_rectifier/scan_rectifier_node'
  '^/home/ubuntu/turtlebot3_ws/install/turtlebot3_node/lib/turtlebot3_node/turtlebot3_ros'
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

sleep 2

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=RPILIDAR-C1
export ROS_DOMAIN_ID=148
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash

nohup ros2 launch turtlebot3_bringup robot.launch.py > /tmp/tb3_robot_direct_scan.log 2>&1 &
echo "$!"
