import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    default_map_path = os.path.join(os.getcwd(), "maps", "whale_map_20251107")
    map_path = LaunchConfiguration("map_path")
    launch_rviz = LaunchConfiguration("launch_rviz")
    load_lanelet = LaunchConfiguration("load_lanelet")
    rviz_config = LaunchConfiguration("rviz_config")
    map_leaf_size = LaunchConfiguration("map_leaf_size")

    map_projector_info = PathJoinSubstitution([map_path, "map_projector_info.yaml"])
    lanelet2_map = PathJoinSubstitution([map_path, "lanelet2_map.osm"])
    pointcloud_map = PathJoinSubstitution([map_path, "pointcloud_map.pcd"])
    pointcloud_metadata = PathJoinSubstitution([map_path, "pointcloud_map_metadata.yaml"])

    default_rviz_config = _pkg_file("autoracer_bringup", "rviz", "map_pointcloud.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=EnvironmentVariable("MAP_PATH", default_value=default_map_path),
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("load_lanelet", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument(
                "map_leaf_size",
                default_value=EnvironmentVariable("MAP_RVIZ_LEAF_SIZE", default_value="0.5"),
            ),
            Node(
                package="autoware_map_projection_loader",
                executable="autoware_map_projection_loader_node",
                name="map_projection_loader",
                output="screen",
                parameters=[
                    {
                        "map_projector_info_path": map_projector_info,
                        "lanelet2_map_path": lanelet2_map,
                        "use_local_projector": False,
                    }
                ],
            ),
            Node(
                package="autoware_map_loader",
                executable="autoware_pointcloud_map_loader",
                name="pointcloud_map_loader_rviz",
                output="screen",
                parameters=[
                    {
                        "enable_whole_load": False,
                        "enable_downsampled_whole_load": True,
                        "enable_partial_load": False,
                        "enable_selected_load": False,
                        "leaf_size": ParameterValue(map_leaf_size, value_type=float),
                        "pcd_paths_or_directory": ParameterValue(
                            [[pointcloud_map]], value_type=list[str]
                        ),
                        "pcd_metadata_path": pointcloud_metadata,
                    }
                ],
                remappings=[
                    ("output/debug/downsampled_pointcloud_map", "/map/pointcloud_map"),
                ],
            ),
            Node(
                package="autoware_map_loader",
                executable="autoware_lanelet2_map_loader",
                name="lanelet2_map_loader",
                output="screen",
                parameters=[
                    {
                        "allow_unsupported_version": True,
                        "center_line_resolution": 5.0,
                        "use_waypoints": True,
                        "lanelet2_map_path": lanelet2_map,
                    }
                ],
                remappings=[("output/lanelet2_map", "/map/vector_map")],
                condition=IfCondition(load_lanelet),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_map",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
