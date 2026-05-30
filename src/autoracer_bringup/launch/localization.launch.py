import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_map_path = os.path.join(os.getcwd(), "maps", "whale_map_20251107")
    map_path = LaunchConfiguration("map_path")

    ndt_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "ndt_scan_matcher.param.yaml",
        ]
    )

    map_projector_info = PathJoinSubstitution([map_path, "map_projector_info.yaml"])
    lanelet2_map = PathJoinSubstitution([map_path, "lanelet2_map.osm"])
    pointcloud_map = PathJoinSubstitution([map_path, "pointcloud_map.pcd"])
    pointcloud_metadata = PathJoinSubstitution([map_path, "pointcloud_map_metadata.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=EnvironmentVariable("MAP_PATH", default_value=default_map_path),
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
                name="pointcloud_map_loader",
                output="screen",
                parameters=[
                    {
                        "enable_whole_load": True,
                        "enable_downsampled_whole_load": False,
                        "enable_partial_load": True,
                        "enable_selected_load": False,
                        "leaf_size": 3.0,
                        "pcd_paths_or_directory": ParameterValue(
                            [[pointcloud_map]], value_type=list[str]
                        ),
                        "pcd_metadata_path": pointcloud_metadata,
                    }
                ],
                remappings=[
                    ("output/pointcloud_map", "/map/pointcloud_map"),
                    ("service/get_partial_pcd_map", "/map/get_partial_pointcloud_map"),
                    ("service/get_differential_pcd_map", "/map/get_differential_pointcloud_map"),
                    ("service/get_selected_pcd_map", "/map/get_selected_pointcloud_map"),
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoware_gnss_poser"),
                            "launch",
                            "gnss_poser.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "input_topic_fix": "/fixposition/fix",
                    "input_topic_orientation": "/fixposition/autoware_orientation",
                    "output_topic_gnss_pose": "/sensing/gnss/pose",
                    "output_topic_gnss_pose_cov": "/sensing/gnss/pose_with_covariance",
                    "output_topic_gnss_fixed": "/sensing/gnss/fixed",
                }.items(),
            ),
            Node(
                package="autoracer_localization",
                executable="fixposition_seed_filter",
                name="fixposition_seed_filter",
                output="screen",
                parameters=[
                    {
                        "input_pose_topic": "/sensing/gnss/pose_with_covariance",
                        "input_status_topic": "/fixposition/fpa/odomstatus",
                        "output_topic": "/localization/fixposition/seed_pose",
                        "map_frame": "map",
                        "max_pose_age_sec": 1.0,
                        "max_xy_stddev_m": 3.0,
                        "max_jump_m": 5.0,
                        "status_timeout_sec": 2.0,
                        "require_status": False,
                        "use_status_when_available": True,
                    }
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="ndt_initial_pose_predictor",
                name="ndt_initial_pose_predictor",
                output="screen",
                parameters=[
                    {
                        "seed_pose_topic": "/localization/fixposition/seed_pose",
                        "ndt_pose_topic": "/localization/pose_with_covariance",
                        "velocity_topic": "/vehicle/status/velocity_status",
                        "steering_topic": "/vehicle/status/steering_status",
                        "output_topic": "/localization/ndt_initial_pose",
                        "map_frame": "map",
                        "publish_rate_hz": 20.0,
                        "wheel_base_m": 1.9,
                        "vehicle_status_timeout_sec": 0.5,
                        "ndt_lost_timeout_sec": 1.0,
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
                    "input_regularization_pose_topic": "/localization/fixposition/seed_pose",
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
                        "ndt_pose_timeout_sec": 2.0,
                        "retrigger_cooldown_sec": 5.0,
                        "min_nvtl_score": 2.3,
                        "max_iteration_num": 30,
                        "max_exe_time_ms": 100.0,
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
        ]
    )
