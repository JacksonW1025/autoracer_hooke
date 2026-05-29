from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_drive_commands", default_value="false"),
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            Node(
                package="autoracer_safety",
                executable="command_gate",
                name="command_gate",
                output="screen",
                parameters=[
                    {
                        "enable_drive_commands": LaunchConfiguration("enable_drive_commands"),
                        "max_speed_mps": LaunchConfiguration("max_speed_mps"),
                    }
                ],
            ),
        ]
    )

