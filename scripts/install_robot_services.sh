#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/turtlebot3_ws}"
CAMERA_WEB_DIR="${CAMERA_WEB_DIR:-/home/ubuntu/camera_web}"
TB3_NAV_DIR="${TB3_NAV_DIR:-/home/ubuntu/tb3_nav}"

cd "$WORKSPACE"

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

mkdir -p "$CAMERA_WEB_DIR"
cp robot/camera_web/camera_web.py "$CAMERA_WEB_DIR/camera_web.py"
cp robot/camera_web/city_map_web.py "$CAMERA_WEB_DIR/city_map_web.py"
chmod +x "$CAMERA_WEB_DIR/camera_web.py"
chmod +x "$CAMERA_WEB_DIR/city_map_web.py"

sudo cp systemd/camera-web.service /etc/systemd/system/camera-web.service
sudo cp systemd/yolo-udp-bridge.service /etc/systemd/system/yolo-udp-bridge.service
sudo cp systemd/web-rviz-state.service /etc/systemd/system/web-rviz-state.service
sudo cp systemd/city-map-web.service /etc/systemd/system/city-map-web.service
sudo systemctl daemon-reload
sudo systemctl enable --now camera-web.service
sudo systemctl enable --now yolo-udp-bridge.service
sudo systemctl enable --now web-rviz-state.service
sudo systemctl enable --now city-map-web.service

mkdir -p "$TB3_NAV_DIR"
install -m 755 scripts/tb3_nav/*.sh "$TB3_NAV_DIR/"
install -m 644 scripts/tb3_nav/fastdds_no_shm.xml "$TB3_NAV_DIR/fastdds_no_shm.xml"

echo "Camera web UI: http://$(hostname -I | awk '{print $1}'):8080/"
echo "City web RViz UI: http://$(hostname -I | awk '{print $1}'):8090/"
