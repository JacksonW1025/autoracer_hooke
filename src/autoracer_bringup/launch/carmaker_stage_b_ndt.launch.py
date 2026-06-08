from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_map_path = LaunchConfiguration("localization_map_path")
    planning_map_path = LaunchConfiguration("planning_map_path")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ndt_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "ndt_scan_matcher.param.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "localization_map_path",
                default_value=(
                    "/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/logs/"
                    "ndt_tiled_map_route271_20260602_031639/tile20"
                ),
            ),
            DeclareLaunchArgument(
                "planning_map_path",
                default_value=(
                    "/opt/ipg/carmaker/linux64-15.1/autoracer_hooke/maps/"
                    "carmaker_builtin_urban"
                ),
            ),
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            LogInfo(
                msg=(
                    "Starting CarMaker Stage B with replay-verified NDT localization: "
                    "fixposition odom seed, NDT raw pose, axis seed fuser, planning/control."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoracer_description"),
                            "launch",
                            "static_tf.launch.py",
                        ]
                    )
                )
            ),
            Node(
                package="autoware_map_loader",
                executable="autoware_pointcloud_map_loader",
                name="pointcloud_map_loader",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "enable_whole_load": True,
                        "enable_downsampled_whole_load": False,
                        "enable_partial_load": True,
                        "enable_selected_load": False,
                        "pcd_paths_or_directory": ParameterValue(
                            [[localization_map_path]], value_type=list[str]
                        ),
                        "pcd_metadata_path": PathJoinSubstitution(
                            [localization_map_path, "pointcloud_map_metadata.yaml"]
                        ),
                    }
                ],
                remappings=[
                    ("output/pointcloud_map", "/map/pointcloud_map"),
                    ("service/get_partial_pcd_map", "/map/get_partial_pointcloud_map"),
                    ("service/get_differential_pcd_map", "/map/get_differential_pointcloud_map"),
                    ("service/get_selected_pcd_map", "/map/get_selected_pointcloud_map"),
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="fixposition_odom_to_seed_pose",
                name="fixposition_odom_to_seed_pose",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_odom_topic": "/fixposition/odometry_enu",
                        "output_pose_topic": "/fixposition/pose_with_covariance",
                        "map_frame": "map",
                        "base_to_gnss_x": 1.90,
                        "base_to_gnss_y": 0.0,
                        "base_to_gnss_z": 1.037,
                        "base_to_gnss_yaw": -1.57079632679,
                        "reported_xy_sigma_m": 0.1,
                        "reported_z_sigma_m": 0.2,
                        "reported_yaw_sigma_deg": 0.5,
                        "publish_clock": True,
                        "clock_topic": "/clock",
                    }
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="fixposition_seed_filter",
                name="fixposition_seed_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_pose_topic": "/fixposition/pose_with_covariance",
                        "input_status_topic": "/fixposition/fpa/odomstatus",
                        "output_topic": "/localization/fixposition/seed_pose",
                        "map_frame": "map",
                        "max_pose_age_sec": 1.0,
                        "max_xy_stddev_m": 3.0,
                        "max_jump_m": 5.0,
                        "require_status": False,
                        "use_status_when_available": True,
                        "require_rtk": False,
                        "allow_non_rtk_initialized": True,
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
                        "use_sim_time": use_sim_time,
                        "seed_pose_topic": "/localization/fixposition/seed_pose",
                        "ndt_pose_topic": "/localization/pose_with_covariance",
                        "velocity_topic": "/vehicle/status/velocity_status",
                        "steering_topic": "/vehicle/status/steering_status",
                        "output_topic": "/localization/ndt_initial_pose",
                        "regularization_seed_topic": (
                            "/localization/fixposition/startup_regularization_pose"
                        ),
                        "corrected_seed_topic": "/localization/fixposition/corrected_seed_pose",
                        "map_frame": "map",
                        "publish_rate_hz": 20.0,
                        "wheel_base_m": 1.9,
                        "vehicle_status_timeout_sec": 0.5,
                        "ndt_lost_timeout_sec": 3.0,
                        "seed_reset_cooldown_sec": 1.0,
                        "enable_seed_bias_correction": False,
                        "enable_tracking_seed_fusion": True,
                        "max_tracking_seed_stddev_m": 0.75,
                        "max_tracking_seed_age_sec": 0.5,
                        "ndt_seed_deviation_guard_m": 0.0,
                        "log_seed_decisions": True,
                    }
                ],
            ),
            Node(
                package="autoware_pointcloud_preprocessor",
                executable="pickup_based_voxel_grid_downsample_filter_node",
                name="ndt_scan_voxel_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "voxel_size_x": 1.5,
                        "voxel_size_y": 1.5,
                        "voxel_size_z": 1.5,
                    }
                ],
                remappings=[
                    ("input", "/sensing/lidar/concatenated/pointcloud"),
                    ("output", "/sensing/lidar/concatenated/pointcloud_downsampled"),
                ],
            ),
            Node(
                package="autoware_ndt_scan_matcher",
                executable="autoware_ndt_scan_matcher_node",
                name="ndt_scan_matcher",
                output="screen",
                parameters=[
                    ndt_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "ndt.num_threads": 32,
                        "dynamic_map_loading.map_radius": 90.0,
                        "dynamic_map_loading.lidar_radius": 50.0,
                    },
                ],
                remappings=[
                    ("points_raw", "/sensing/lidar/concatenated/pointcloud_downsampled"),
                    ("ekf_pose_with_covariance", "/localization/ndt_initial_pose"),
                    (
                        "regularization_pose_with_covariance",
                        "/localization/fixposition/seed_pose",
                    ),
                    ("trigger_node_srv", "/localization/ndt_trigger"),
                    ("ndt_pose", "/localization/ndt/raw_pose"),
                    (
                        "ndt_pose_with_covariance",
                        "/localization/ndt/raw_pose_with_covariance",
                    ),
                    ("pcd_loader_service", "/map/get_differential_pointcloud_map"),
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="ndt_axis_seed_fuser",
                name="ndt_axis_seed_fuser",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "raw_ndt_pose_topic": "/localization/ndt/raw_pose_with_covariance",
                        "seed_pose_topic": "/localization/fixposition/seed_pose",
                        "output_topic": "/localization/pose_with_covariance",
                        "max_seed_age_sec": 0.5,
                        "max_seed_xy_stddev_m": 0.75,
                        "lateral_gain": 1.0,
                        "yaw_deadband_sigma": 3.0,
                    }
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="ndt_startup_helper",
                name="ndt_startup_helper",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "initial_pose_topic": "/localization/ndt_initial_pose",
                        "ndt_pose_topic": "/localization/pose_with_covariance",
                        "trigger_service": "/localization/ndt_trigger",
                        "map_service": "/map/get_partial_pointcloud_map",
                        "wait_for_map_service": True,
                        "required_initial_messages": 3,
                        "fresh_initial_pose_sec": 0.5,
                        "ndt_pose_timeout_sec": 6.0,
                        "retrigger_cooldown_sec": 5.0,
                        "min_nvtl_score": 2.3,
                        "max_iteration_num": 30,
                        "max_exe_time_ms": 5000.0,
                    }
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="pose_tf_broadcaster",
                name="pose_tf_broadcaster",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_pose_topic": "/localization/pose_with_covariance",
                        "map_frame": "map",
                        "base_frame": "base_link",
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
                        "use_sim_time": use_sim_time,
                        "route_goal_path": PathJoinSubstitution(
                            [planning_map_path, "route_goal.yaml"]
                        ),
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
                        "use_sim_time": use_sim_time,
                        "osm_path": PathJoinSubstitution([planning_map_path, "lanelet2_map.osm"]),
                        "map_projector_info_path": PathJoinSubstitution(
                            [planning_map_path, "map_projector_info.yaml"]
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
                        "use_sim_time": use_sim_time,
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
                        "use_sim_time": use_sim_time,
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
                        "use_sim_time": use_sim_time,
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
