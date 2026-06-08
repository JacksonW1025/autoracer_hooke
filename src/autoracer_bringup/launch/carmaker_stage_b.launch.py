from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    max_speed_mps = LaunchConfiguration("max_speed_mps")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value="/opt/ipg/carmaker/linux64-15.1/autoracer_hooke/maps/carmaker_builtin_urban",
            ),
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            LogInfo(
                msg=(
                    "Starting autoracer CarMaker stage B: route goal, lanelet global route, "
                    "local trajectory planner, pure_pursuit_controller, command_gate."
                )
            ),
            Node(
                package="autoracer_carmaker_sim",
                executable="ground_truth_localization_relay",
                name="ground_truth_localization_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/carmaker/ground_truth/pose",
                        "output_topic": "/localization/pose_with_covariance",
                        "frame_id": "map",
                    }
                ],
            ),
            Node(
                package="autoracer_planning",
                executable="route_goal_publisher",
                name="route_goal_publisher",
                output="screen",
                parameters=[
                    {
                        "route_goal_path": PathJoinSubstitution([map_path, "route_goal.yaml"]),
                        "goal_pose_topic": "/goal_pose",
                        "pose_topic": "/localization/pose_with_covariance",
                        "publish_rate_hz": 2.0,
                        "max_publish_count": 20,
                    }
                ],
            ),
            Node(
                package="autoracer_planning",
                executable="lanelet_route_planner",
                name="lanelet_route_planner",
                output="screen",
                parameters=[
                    {
                        "osm_path": PathJoinSubstitution([map_path, "lanelet2_map.osm"]),
                        "map_projector_info_path": PathJoinSubstitution(
                            [map_path, "map_projector_info.yaml"]
                        ),
                        "speed_limit_mps": max_speed_mps,
                        "trajectory_topic": "/planning/global_trajectory",
                    }
                ],
            ),
            Node(
                package="autoracer_planning",
                executable="local_trajectory_planner",
                name="local_trajectory_planner",
                output="screen",
                parameters=[
                    {
                        "global_trajectory_topic": "/planning/global_trajectory",
                        "trajectory_topic": "/planning/trajectory",
                        "lookahead_distance_m": 80.0,
                        "backward_distance_m": 2.0,
                        "resample_interval_m": 0.5,
                        "publish_rate_hz": 10.0,
                        "max_speed_mps": max_speed_mps,
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
                        "trajectory_topic": "/planning/trajectory",
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
                        "require_trajectory": True,
                        "trajectory_topic": "/planning/trajectory",
                        "trajectory_timeout_sec": 1.0,
                        "max_speed_mps": max_speed_mps,
                        "command_timeout_sec": 0.7,
                        "localization_timeout_sec": 1.0,
                        "publish_rate_hz": 30.0,
                    }
                ],
            ),
        ]
    )
