import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_map_path = os.path.join(os.getcwd(), "maps", "whale_map_20251107")
    map_path = LaunchConfiguration("map_path")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    launch_fixposition_seed = LaunchConfiguration("launch_fixposition_seed")
    launch_manual_seed = LaunchConfiguration("launch_manual_seed")
    manual_seed_input_topic = LaunchConfiguration("manual_seed_input_topic")
    manual_seed_require_input_pose = LaunchConfiguration("manual_seed_require_input_pose")
    manual_seed_x = LaunchConfiguration("manual_seed_x")
    manual_seed_y = LaunchConfiguration("manual_seed_y")
    manual_seed_z = LaunchConfiguration("manual_seed_z")
    manual_seed_yaw = LaunchConfiguration("manual_seed_yaw")
    manual_seed_xy_variance = LaunchConfiguration("manual_seed_xy_variance")
    manual_seed_z_variance = LaunchConfiguration("manual_seed_z_variance")
    manual_seed_yaw_variance = LaunchConfiguration("manual_seed_yaw_variance")
    launch_kinematic_state_publisher = LaunchConfiguration(
        "launch_kinematic_state_publisher"
    )

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
            DeclareLaunchArgument("wheel_base_m", default_value="1.9"),
            DeclareLaunchArgument("launch_fixposition_seed", default_value="true"),
            DeclareLaunchArgument("launch_manual_seed", default_value="false"),
            DeclareLaunchArgument("manual_seed_input_topic", default_value="/initialpose"),
            DeclareLaunchArgument("manual_seed_require_input_pose", default_value="false"),
            DeclareLaunchArgument("manual_seed_x", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_y", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_z", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_yaw", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_xy_variance", default_value="1.0"),
            DeclareLaunchArgument("manual_seed_z_variance", default_value="0.25"),
            DeclareLaunchArgument("manual_seed_yaw_variance", default_value="0.03"),
            DeclareLaunchArgument("launch_kinematic_state_publisher", default_value="true"),
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
                condition=IfCondition(launch_fixposition_seed),
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
                condition=IfCondition(launch_fixposition_seed),
            ),
            Node(
                package="autoracer_localization",
                executable="manual_seed_pose_publisher",
                name="manual_seed_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "output_topic": "/localization/fixposition/seed_pose",
                        "input_topic": ParameterValue(manual_seed_input_topic, value_type=str),
                        "frame_id": "map",
                        "publish_rate_hz": 20.0,
                        "publish_once": False,
                        "require_input_pose": ParameterValue(
                            manual_seed_require_input_pose, value_type=bool
                        ),
                        "x": ParameterValue(manual_seed_x, value_type=float),
                        "y": ParameterValue(manual_seed_y, value_type=float),
                        "z": ParameterValue(manual_seed_z, value_type=float),
                        "yaw": ParameterValue(manual_seed_yaw, value_type=float),
                        "xy_variance": ParameterValue(
                            manual_seed_xy_variance, value_type=float
                        ),
                        "z_variance": ParameterValue(manual_seed_z_variance, value_type=float),
                        "yaw_variance": ParameterValue(
                            manual_seed_yaw_variance, value_type=float
                        ),
                    }
                ],
                condition=IfCondition(launch_manual_seed),
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
                        "wheel_base_m": ParameterValue(wheel_base_m, value_type=float),
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
            Node(
                package="autoracer_localization",
                executable="kinematic_state_publisher",
                name="kinematic_state_publisher",
                output="screen",
                parameters=[
                    {
                        "ndt_pose_topic": "/localization/pose_with_covariance",
                        "velocity_topic": "/vehicle/status/velocity_status",
                        "steering_topic": "/vehicle/status/steering_status",
                        "output_topic": "/localization/kinematic_state",
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "wheel_base_m": ParameterValue(wheel_base_m, value_type=float),
                        "publish_rate_hz": 50.0,
                        "vehicle_status_timeout_sec": 0.5,
                        "ndt_lost_timeout_sec": 1.0,
                        "max_prediction_step_sec": 0.1,
                    }
                ],
                condition=IfCondition(launch_kinematic_state_publisher),
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
