from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    max_steer_rad = LaunchConfiguration("max_steer_rad")
    max_steer_rate_radps = LaunchConfiguration("max_steer_rate_radps")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_drive_commands", default_value="false"),
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            DeclareLaunchArgument("max_steer_rad", default_value="0.488"),
            DeclareLaunchArgument("max_steer_rate_radps", default_value="0.5"),
            Node(
                package="autoracer_safety",
                executable="command_gate",
                name="command_gate",
                output="screen",
                parameters=[
                    {
                        "enable_drive_commands": LaunchConfiguration("enable_drive_commands"),
                        "max_speed_mps": LaunchConfiguration("max_speed_mps"),
                        "max_steer_rad": ParameterValue(max_steer_rad, value_type=float),
                        "max_steer_rate_radps": ParameterValue(
                            max_steer_rate_radps, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
