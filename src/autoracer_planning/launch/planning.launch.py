from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    max_speed_mps = LaunchConfiguration("max_speed_mps")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path"),
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
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
                    }
                ],
            ),
        ]
    )

