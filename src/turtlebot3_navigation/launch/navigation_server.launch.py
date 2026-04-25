from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_navigation',
            executable='navigation_server',
            name='turtlebot3_navigation_server',
            output='screen',
            parameters=[],
        ),
    ])
