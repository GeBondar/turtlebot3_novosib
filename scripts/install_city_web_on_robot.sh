#!/usr/bin/env bash
set -eo pipefail

DEPLOY_DIR="${1:-/tmp/tb3_city_web_deploy}"
CAMERA_WEB_DIR="/home/ubuntu/camera_web"
WS_DIR="/home/ubuntu/turtlebot3_ws"
PKG_DIR="${WS_DIR}/src/tb3_camera_web"

mkdir -p "${CAMERA_WEB_DIR}"
install -m 755 "${DEPLOY_DIR}/city_map_web.py" "${CAMERA_WEB_DIR}/city_map_web.py"

mkdir -p "${PKG_DIR}/scripts" "${PKG_DIR}/launch" "${PKG_DIR}/resource"
install -m 644 "${DEPLOY_DIR}/ros2_package/CMakeLists.txt" "${PKG_DIR}/CMakeLists.txt"
install -m 644 "${DEPLOY_DIR}/ros2_package/package.xml" "${PKG_DIR}/package.xml"
install -m 644 "${DEPLOY_DIR}/ros2_package/resource/tb3_camera_web" "${PKG_DIR}/resource/tb3_camera_web"
install -m 644 "${DEPLOY_DIR}/ros2_package/launch/camera_web.launch.py" "${PKG_DIR}/launch/camera_web.launch.py"
install -m 644 "${DEPLOY_DIR}/ros2_package/launch/yolo_udp_bridge.launch.py" "${PKG_DIR}/launch/yolo_udp_bridge.launch.py"
install -m 755 "${DEPLOY_DIR}/ros2_package/scripts/camera_web_node.py" "${PKG_DIR}/scripts/camera_web_node.py"
install -m 755 "${DEPLOY_DIR}/ros2_package/scripts/yolo_udp_bridge_node.py" "${PKG_DIR}/scripts/yolo_udp_bridge_node.py"
install -m 755 "${DEPLOY_DIR}/ros2_package/scripts/web_rviz_state_node.py" "${PKG_DIR}/scripts/web_rviz_state_node.py"

source /opt/ros/jazzy/setup.bash
cd "${WS_DIR}"
colcon build --packages-select tb3_camera_web

sudo install -m 644 "${DEPLOY_DIR}/city-map-web.service" /etc/systemd/system/city-map-web.service
sudo install -m 644 "${DEPLOY_DIR}/web-rviz-state.service" /etc/systemd/system/web-rviz-state.service
sudo systemctl daemon-reload
sudo systemctl enable web-rviz-state.service city-map-web.service
sudo systemctl restart web-rviz-state.service city-map-web.service

systemctl --no-pager --full status city-map-web.service web-rviz-state.service || true
echo "Dashboard: http://$(hostname -I | awk '{print $1}'):8090/"
