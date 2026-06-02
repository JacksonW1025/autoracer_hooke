from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    trajectory_speed_mps = LaunchConfiguration("trajectory_speed_mps")

    return LaunchDescription(
        [
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            DeclareLaunchArgument("trajectory_speed_mps", default_value="1.5"),
            LogInfo(
                msg=(
                    "Starting autoracer CarMaker stage A: RoadEval trajectory provider, "
                    "pure_pursuit_controller, command_gate(enable_drive_commands=true)."
                )
            ),
            Node(
                package="autoracer_carmaker_sim",
                executable="carmaker_trajectory_provider",
                name="carmaker_trajectory_provider",
                output="screen",
                parameters=[
                    {
                        "centerline_topic": "/carmaker/road/centerline",
                        "pose_topic": "/localization/pose_with_covariance",
                        "trajectory_topic": "/planning/trajectory",
                        "speed_mps": trajectory_speed_mps,
                        "publish_rate_hz": 10.0,
                    }
                ],
            ),
            Node(
                package="autoracer_control",
                executable="pure_pursuit_controller",
                name="pure_pursuit_controller",
                output="screen",
                parameters=[
                    {
                        "max_speed_mps": max_speed_mps,
                        "control_rate_hz": 30.0,
                    }
                ],
            ),
            Node(
                package="autoracer_safety",
                executable="command_gate",
                name="command_gate",
                output="screen",
                parameters=[
                    {
                        "enable_drive_commands": True,
                        "output_topic": "/control/command/control_cmd",
                        "max_speed_mps": max_speed_mps,
                        "command_timeout_sec": 0.7,
                        "localization_timeout_sec": 1.0,
                        "publish_rate_hz": 30.0,
                    }
                ],
            ),
        ]
    )
