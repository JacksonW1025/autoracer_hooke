import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    default_map_path = os.path.join(os.getcwd(), "maps", "whale_map_20251107")
    map_path = LaunchConfiguration("map_path")
    scenario_dir = LaunchConfiguration("scenario_dir")
    scenario = LaunchConfiguration("scenario")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    map_leaf_size = LaunchConfiguration("map_leaf_size")

    map_projector_info = PathJoinSubstitution([map_path, "map_projector_info.yaml"])
    lanelet2_map = PathJoinSubstitution([map_path, "lanelet2_map.osm"])
    pointcloud_map = PathJoinSubstitution([map_path, "pointcloud_map.pcd"])
    pointcloud_metadata = PathJoinSubstitution([map_path, "pointcloud_map_metadata.yaml"])
    default_scenario_dir = PathJoinSubstitution([map_path, "mock_lidar_scenarios"])

    ndt_param_file = _pkg_file("autoracer_bringup", "config", "hooke2", "ndt_scan_matcher.param.yaml")
    rviz_default = _pkg_file("autoracer_bringup", "rviz", "mock_lidar_ndt.rviz")
    vehicle_model = PathJoinSubstitution(
        [FindPackageShare("hooke2_description"), "urdf", "vehicle.xacro"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=EnvironmentVariable("MAP_PATH", default_value=default_map_path),
            ),
            DeclareLaunchArgument(
                "scenario_dir",
                default_value=EnvironmentVariable(
                    "MOCK_LIDAR_SCENARIO_DIR", default_value=default_scenario_dir
                ),
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value=EnvironmentVariable("MOCK_LIDAR_SCENARIO", default_value="latest"),
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default),
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
                name="pointcloud_map_loader_mock_ndt",
                output="screen",
                parameters=[
                    {
                        "enable_whole_load": False,
                        "enable_downsampled_whole_load": True,
                        "enable_partial_load": True,
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
                    ("service/get_partial_pcd_map", "/map/get_partial_pointcloud_map"),
                    ("service/get_differential_pcd_map", "/map/get_differential_pointcloud_map"),
                    ("service/get_selected_pcd_map", "/map/get_selected_pointcloud_map"),
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [FindExecutable(name="xacro"), " ", vehicle_model]
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="autoracer_sensing",
                executable="mock_lidar_publisher",
                name="mock_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        "scenario_dir": scenario_dir,
                        "scenario": scenario,
                    }
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoware_ndt_scan_matcher"),
                            "launch",
                            "ndt_scan_matcher.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "param_file": ndt_param_file,
                    "input_pointcloud": "/sensing/lidar/concatenated/pointcloud",
                    "input_initial_pose_topic": "/localization/ndt_initial_pose",
                    "input_regularization_pose_topic": "/localization/unused_regularization_pose",
                    "input_service_trigger_node": "/localization/ndt_trigger",
                    "output_pose_topic": "/localization/pose",
                    "output_pose_with_covariance_topic": "/localization/pose_with_covariance",
                    "client_map_loader": "/map/get_differential_pointcloud_map",
                }.items(),
            ),
            Node(
                package="autoracer_localization",
                executable="ndt_startup_helper",
                name="ndt_startup_helper",
                output="screen",
                parameters=[
                    {
                        "initial_pose_topic": "/localization/ndt_initial_pose",
                        "ndt_pose_topic": "/localization/pose_with_covariance",
                        "trigger_service": "/localization/ndt_trigger",
                        "map_service": "/map/get_partial_pointcloud_map",
                        "wait_for_map_service": True,
                        "required_initial_messages": 3,
                        "fresh_initial_pose_sec": 0.5,
                        "ndt_pose_timeout_sec": 30.0,
                        "retrigger_cooldown_sec": 30.0,
                        "max_exe_time_ms": 50000.0,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoracer_localization"),
                            "launch",
                            "pose_tf.launch.py",
                        ]
                    )
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_mock_lidar_ndt",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
