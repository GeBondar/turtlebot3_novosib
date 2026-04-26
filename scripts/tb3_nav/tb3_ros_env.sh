#!/usr/bin/env bash
set -eo pipefail
set +u

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=RPILIDAR-C1
export ROS_DOMAIN_ID=148
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Avoid FastDDS shared-memory lock errors that break short-lived ROS CLI commands.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/tb3_nav/fastdds_no_shm.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/home/ubuntu/tb3_nav/fastdds_no_shm.xml

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash
