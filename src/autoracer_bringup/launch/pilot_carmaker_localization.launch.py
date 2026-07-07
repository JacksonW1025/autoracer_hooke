import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def _localization_param(*parts):
    return _pkg_file("autoracer_bringup", "config", "pilot_compatible", "localization", *parts)


def generate_launch_description():
    default_map_path = os.path.join(os.getcwd(), "maps", "whale_map_20251107")
    map_path = LaunchConfiguration("map_path")
    input_pointcloud = LaunchConfiguration("input_pointcloud")
    initial_pose = LaunchConfiguration("initial_pose")
    localization_pointcloud_container_name = LaunchConfiguration(
        "localization_pointcloud_container_name"
    )

    map_projector_info = PathJoinSubstitution([map_path, "map_projector_info.yaml"])
    lanelet2_map = PathJoinSubstitution([map_path, "lanelet2_map.osm"])
    pointcloud_metadata = PathJoinSubstitution([map_path, "pointcloud_map_metadata.yaml"])

    localization_launch = PathJoinSubstitution(
        [
            get_package_share_directory("tier4_localization_launch"),
            "launch",
            "localization.launch.xml",
        ]
    )

    runtime_actions = [
        SetParameter(name="use_sim_time", value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _pkg_file("autoracer_description", "launch", "static_tf.launch.py")
            )
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
            namespace="/map",
            name="pointcloud_map_loader",
            output="screen",
            parameters=[
                {
                    "enable_whole_load": True,
                    "enable_downsampled_whole_load": False,
                    "enable_partial_load": True,
                    "enable_differential_load": True,
                    "enable_selected_load": False,
                    "leaf_size": 3.0,
                    "pcd_paths_or_directory": ParameterValue(
                        [[map_path]], value_type=list[str]
                    ),
                    "pcd_metadata_path": pointcloud_metadata,
                }
            ],
            remappings=[
                ("output/pointcloud_map", "/map/pointcloud_map"),
                ("output/pointcloud_map_metadata", "/map/pointcloud_map_metadata"),
                ("service/get_partial_pcd_map", "/map/get_partial_pointcloud_map"),
                ("service/get_differential_pcd_map", "/map/get_differential_pointcloud_map"),
                ("service/get_selected_pcd_map", "/map/get_selected_pointcloud_map"),
            ],
        ),
        Node(
            package="rclcpp_components",
            executable="component_container_mt",
            name="pointcloud_container",
            output="screen",
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                _pkg_file(
                    "autoware_vehicle_velocity_converter",
                    "launch",
                    "vehicle_velocity_converter.launch.xml",
                )
            ),
            launch_arguments={
                "input_vehicle_velocity_topic": "/vehicle/status/velocity_status",
                "output_twist_with_covariance": (
                    "/sensing/vehicle_velocity_converter/twist_with_covariance"
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                _pkg_file("autoware_gnss_poser", "launch", "gnss_poser.launch.xml")
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
            package="topic_tools",
            executable="relay",
            name="fixposition_rawimu_to_sensing_imu_relay",
            output="screen",
            parameters=[
                {
                    "input_topic": "/fixposition/rawimu",
                    "output_topic": "/sensing/imu/imu_data",
                }
            ],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "pose_source": "ndt",
                "twist_source": "gyro_odom",
                "initial_pose": initial_pose,
                "system_run_mode": "logging_simulation",
                "input_pointcloud": input_pointcloud,
                "localization_pointcloud_container_name": (
                    localization_pointcloud_container_name
                ),
                "ndt_scan_matcher/ndt_scan_matcher_param_path": _localization_param(
                    "ndt_scan_matcher", "ndt_scan_matcher.param.yaml"
                ),
                "ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range_param_path": _localization_param(
                    "ndt_scan_matcher",
                    "pointcloud_preprocessor",
                    "crop_box_filter_measurement_range.param.yaml",
                ),
                "ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_downsample_filter_param_path": _localization_param(
                    "ndt_scan_matcher",
                    "pointcloud_preprocessor",
                    "voxel_grid_filter.param.yaml",
                ),
                "ndt_scan_matcher/pointcloud_preprocessor/random_downsample_filter_param_path": _localization_param(
                    "ndt_scan_matcher",
                    "pointcloud_preprocessor",
                    "random_downsample_filter.param.yaml",
                ),
                "localization_error_monitor_param_path": _localization_param(
                    "localization_error_monitor.param.yaml"
                ),
                "ekf_localizer_param_path": _localization_param(
                    "ekf_localizer.param.yaml"
                ),
                "stop_filter_param_path": _localization_param("stop_filter.param.yaml"),
                "pose_instability_detector_param_path": _localization_param(
                    "pose_instability_detector.param.yaml"
                ),
                "pose_initializer_param_path": _localization_param(
                    "pose_initializer.param.yaml"
                ),
                "twist2accel_param_path": _localization_param(
                    "twist2accel.param.yaml"
                ),
                "eagleye_param_path": _localization_param("eagleye_config.param.yaml"),
                "ar_tag_based_localizer_param_path": _localization_param(
                    "ar_tag_based_localizer.param.yaml"
                ),
                "lidar_marker_localizer/lidar_marker_localizer_param_path": _localization_param(
                    "lidar_marker_localizer", "lidar_marker_localizer.param.yaml"
                ),
                "lidar_marker_localizer/pointcloud_preprocessor/crop_box_filter_measurement_range_param_path": _localization_param(
                    "lidar_marker_localizer",
                    "pointcloud_preprocessor",
                    "crop_box_filter_measurement_range.param.yaml",
                ),
                "lidar_marker_localizer/pointcloud_preprocessor/ring_filter_param_path": _localization_param(
                    "lidar_marker_localizer",
                    "pointcloud_preprocessor",
                    "ring_filter.param.yaml",
                ),
            }.items(),
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=EnvironmentVariable("MAP_PATH", default_value=default_map_path),
            ),
            DeclareLaunchArgument(
                "input_pointcloud",
                default_value="/sensing/lidar/concatenated/pointcloud",
            ),
            DeclareLaunchArgument(
                "localization_pointcloud_container_name",
                default_value="/pointcloud_container",
            ),
            DeclareLaunchArgument("initial_pose", default_value="[]"),
            GroupAction(runtime_actions),
        ]
    )

