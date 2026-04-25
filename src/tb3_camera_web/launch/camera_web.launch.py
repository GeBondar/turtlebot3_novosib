from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration("device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    port = LaunchConfiguration("port")
    max_clients = LaunchConfiguration("max_clients")

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("max_clients", default_value="8"),
        Node(
            package="tb3_camera_web",
            executable="camera_web_node",
            name="camera_web",
            output="screen",
            parameters=[{
                "camera_device": device,
                "width": width,
                "height": height,
                "fps": fps,
                "port": port,
                "max_clients": max_clients,
            }],
        ),
    ])
