cmake_minimum_required(VERSION 3.8)
project(tb3_camera_web)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/camera_web_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_web_node
)

install(PROGRAMS
  scripts/yolo_udp_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_udp_bridge
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
