from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    autostart = LaunchConfiguration("autostart", default="True")

    pkg_share = get_package_share_directory("tb3_rplidar_c1")
    params_file = LaunchConfiguration(
        "params_file",
        default=f"{pkg_share}/config/nav2_burger.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("autostart", default_value="True"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg_share}/launch/cartographer.launch.py"),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_rviz": "false",
                "resolution": "0.05",
                "publish_period_sec": "0.5",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg_share}/launch/navigation_core.launch.py"),
            launch_arguments={
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "use_respawn": "False",
            }.items(),
        ),
    ])
