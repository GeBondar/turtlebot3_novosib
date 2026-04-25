from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bind_host = LaunchConfiguration("bind_host")
    udp_port = LaunchConfiguration("udp_port")
    topic = LaunchConfiguration("topic")
    latest_json_path = LaunchConfiguration("latest_json_path")

    return LaunchDescription([
        DeclareLaunchArgument("bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("udp_port", default_value="5005"),
        DeclareLaunchArgument("topic", default_value="/vision/traffic_sign_detections"),
        DeclareLaunchArgument("latest_json_path", default_value="/home/ubuntu/camera_web/latest_detections.json"),
        Node(
            package="tb3_camera_web",
            executable="yolo_udp_bridge",
            name="yolo_udp_bridge",
            output="screen",
            parameters=[{
                "bind_host": bind_host,
                "udp_port": udp_port,
                "topic": topic,
                "latest_json_path": latest_json_path,
            }],
        ),
    ])
