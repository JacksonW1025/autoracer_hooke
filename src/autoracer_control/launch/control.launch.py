from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_speed_mps = LaunchConfiguration("max_speed_mps")

    return LaunchDescription(
        [
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            Node(
                package="autoracer_control",
                executable="pure_pursuit_controller",
                name="pure_pursuit_controller",
                output="screen",
                parameters=[{"max_speed_mps": max_speed_mps}],
            ),
        ]
    )

