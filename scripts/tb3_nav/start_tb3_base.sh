#!/usr/bin/env bash
set -euo pipefail

BASE_DIR=/home/ubuntu/tb3_nav
source "$BASE_DIR/tb3_ros_env.sh"

for pattern in \
  'ros2 launch turtlebot3_bringup robot.launch.py' \
  '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher' \
  '/home/ubuntu/turtlebot3_ws/install/sllidar_ros2/lib/sllidar_ros2/sllidar_node' \
  '/home/ubuntu/turtlebot3_ws/install/turtlebot3_node/lib/turtlebot3_node/turtlebot3_ros'
do
  pids="$(pgrep -f "$pattern" || true)"
  if [ -n "$pids" ]; then
    kill -INT $pids || true
  fi
done

sleep 4

for pattern in \
  'ros2 launch turtlebot3_bringup robot.launch.py' \
  '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher' \
  '/home/ubuntu/turtlebot3_ws/install/sllidar_ros2/lib/sllidar_ros2/sllidar_node' \
  '/home/ubuntu/turtlebot3_ws/install/turtlebot3_node/lib/turtlebot3_node/turtlebot3_ros'
do
  pids="$(pgrep -f "$pattern" || true)"
  if [ -n "$pids" ]; then
    kill -TERM $pids || true
  fi
done

sleep 2

nohup ros2 launch turtlebot3_bringup robot.launch.py > /tmp/tb3_base.log 2>&1 &
echo "base_pid=$!"

sleep 8
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" \
  >/tmp/tb3_zero_cmd.log 2>&1 || true

echo "base started; logs: /tmp/tb3_base.log"
