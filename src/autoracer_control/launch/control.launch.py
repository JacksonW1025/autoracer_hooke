from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    max_steer_rad = LaunchConfiguration("max_steer_rad")

    return LaunchDescription(
        [
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            DeclareLaunchArgument("wheel_base_m", default_value="1.9"),
            DeclareLaunchArgument("max_steer_rad", default_value="0.488"),
            Node(
                package="autoracer_control",
                executable="pure_pursuit_controller",
                name="pure_pursuit_controller",
                output="screen",
                parameters=[
                    {
                        "max_speed_mps": max_speed_mps,
                        "wheel_base_m": ParameterValue(wheel_base_m, value_type=float),
                        "max_steer_rad": ParameterValue(max_steer_rad, value_type=float),
                    }
                ],
            ),
        ]
    )
