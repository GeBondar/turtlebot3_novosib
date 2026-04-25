#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/turtlebot3_ws}"
CAMERA_WEB_DIR="${CAMERA_WEB_DIR:-/home/ubuntu/camera_web}"

cd "$WORKSPACE"

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

mkdir -p "$CAMERA_WEB_DIR"
cp robot/camera_web/camera_web.py "$CAMERA_WEB_DIR/camera_web.py"
chmod +x "$CAMERA_WEB_DIR/camera_web.py"

sudo cp systemd/camera-web.service /etc/systemd/system/camera-web.service
sudo cp systemd/yolo-udp-bridge.service /etc/systemd/system/yolo-udp-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable --now camera-web.service
sudo systemctl enable --now yolo-udp-bridge.service

echo "Camera web UI: http://$(hostname -I | awk '{print $1}'):8080/"

