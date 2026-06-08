from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_map_path = LaunchConfiguration("localization_map_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_gnss = LaunchConfiguration("enable_gnss")
    use_gt_initialpose_once = LaunchConfiguration("use_gt_initialpose_once")
    use_gnss_initialpose_once = LaunchConfiguration("use_gnss_initialpose_once")
    gnss_initialpose_method = LaunchConfiguration("gnss_initialpose_method")
    initialpose_min_stamp_sec = LaunchConfiguration("initialpose_min_stamp_sec")
    diagnostic_reinit_min_stamp_sec = LaunchConfiguration("diagnostic_reinit_min_stamp_sec")
    diagnostic_reinit_post_initialization_grace_sec = LaunchConfiguration(
        "diagnostic_reinit_post_initialization_grace_sec"
    )
    diagnostic_reinit_min_trigger_duration_sec = LaunchConfiguration(
        "diagnostic_reinit_min_trigger_duration_sec"
    )
    diagnostic_reinit_max_gnss_pose_age_sec = LaunchConfiguration(
        "diagnostic_reinit_max_gnss_pose_age_sec"
    )
    diagnostic_reinit_method = LaunchConfiguration("diagnostic_reinit_method")
    map_projector_info_path = LaunchConfiguration("map_projector_info_path")
    lanelet2_map_path = LaunchConfiguration("lanelet2_map_path")
    scan_voxel_size = LaunchConfiguration("scan_voxel_size")
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")
    crop_min_z = LaunchConfiguration("crop_min_z")
    crop_max_z = LaunchConfiguration("crop_max_z")
    ndt_regularization_enable = LaunchConfiguration("ndt_regularization_enable")
    ndt_regularization_scale_factor = LaunchConfiguration("ndt_regularization_scale_factor")
    ndt_regularization_pose_topic = LaunchConfiguration("ndt_regularization_pose_topic")
    ndt_runtime_multistart_enable = LaunchConfiguration("ndt_runtime_multistart_enable")
    ndt_runtime_tracking_tier1_period_sec = LaunchConfiguration(
        "ndt_runtime_tracking_tier1_period_sec"
    )
    ndt_runtime_tracking_far_tier_period_sec = LaunchConfiguration(
        "ndt_runtime_tracking_far_tier_period_sec"
    )
    ndt_runtime_raw_score_override_margin = LaunchConfiguration(
        "ndt_runtime_raw_score_override_margin"
    )
    ndt_dynamic_map_radius = LaunchConfiguration("ndt_dynamic_map_radius")
    ndt_dynamic_lidar_radius = LaunchConfiguration("ndt_dynamic_lidar_radius")
    ndt_score_no_ground_points_enable = LaunchConfiguration(
        "ndt_score_no_ground_points_enable"
    )
    ndt_score_no_ground_z_margin = LaunchConfiguration("ndt_score_no_ground_z_margin")
    ndt_converged_param_nearest_voxel_transformation_likelihood = LaunchConfiguration(
        "ndt_converged_param_nearest_voxel_transformation_likelihood"
    )
    ndt_validation_initial_to_result_distance_tolerance_m = LaunchConfiguration(
        "ndt_validation_initial_to_result_distance_tolerance_m"
    )
    ndt_num_threads = LaunchConfiguration("ndt_num_threads")
    ndt_max_iterations = LaunchConfiguration("ndt_max_iterations")
    ndt_initial_pose_particles_num = LaunchConfiguration("ndt_initial_pose_particles_num")
    ndt_initial_pose_startup_trials = LaunchConfiguration("ndt_initial_pose_startup_trials")
    ndt_initial_pose_include_seed_pose = LaunchConfiguration("ndt_initial_pose_include_seed_pose")
    ndt_initial_pose_force_seed_yaw = LaunchConfiguration("ndt_initial_pose_force_seed_yaw")
    ndt_initial_pose_output_seed_yaw = LaunchConfiguration("ndt_initial_pose_output_seed_yaw")
    ndt_initial_pose_use_sensor_points_stamp = LaunchConfiguration(
        "ndt_initial_pose_use_sensor_points_stamp"
    )
    ndt_initial_pose_deterministic_offsets_enable = LaunchConfiguration(
        "ndt_initial_pose_deterministic_offsets_enable"
    )
    ndt_output_pose_time_offset_sec = LaunchConfiguration("ndt_output_pose_time_offset_sec")
    ekf_enable_yaw_bias_estimation = LaunchConfiguration("ekf_enable_yaw_bias_estimation")
    ekf_pose_additional_delay_sec = LaunchConfiguration("ekf_pose_additional_delay_sec")
    ekf_twist_additional_delay_sec = LaunchConfiguration("ekf_twist_additional_delay_sec")
    ekf_proc_stddev_yaw_c = LaunchConfiguration("ekf_proc_stddev_yaw_c")
    ekf_proc_stddev_wz_c = LaunchConfiguration("ekf_proc_stddev_wz_c")
    ekf_pose_smoothing_steps = LaunchConfiguration("ekf_pose_smoothing_steps")
    ekf_max_twist_queue_size = LaunchConfiguration("ekf_max_twist_queue_size")

    ndt_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "ndt_scan_matcher.param.yaml",
        ]
    )
    localization_error_monitor_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoware_localization_error_monitor"),
            "config",
            "localization_error_monitor.param.yaml",
        ]
    )
    pose_instability_detector_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "pose_instability_detector_carmaker.param.yaml",
        ]
    )

    default_map_projector_info = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "carmaker_map_projector_info.yaml",
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
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_gnss", default_value="true"),
            DeclareLaunchArgument("use_gt_initialpose_once", default_value="true"),
            DeclareLaunchArgument("use_gnss_initialpose_once", default_value="false"),
            DeclareLaunchArgument("gnss_initialpose_method", default_value="auto"),
            DeclareLaunchArgument("initialpose_min_stamp_sec", default_value="12.0"),
            DeclareLaunchArgument("diagnostic_reinit_min_stamp_sec", default_value="5.0"),
            DeclareLaunchArgument(
                "diagnostic_reinit_post_initialization_grace_sec", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "diagnostic_reinit_min_trigger_duration_sec", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "diagnostic_reinit_max_gnss_pose_age_sec", default_value="0.5"
            ),
            DeclareLaunchArgument("diagnostic_reinit_method", default_value="direct"),
            DeclareLaunchArgument("map_projector_info_path", default_value=default_map_projector_info),
            DeclareLaunchArgument(
                "lanelet2_map_path",
                default_value=PathJoinSubstitution([localization_map_path, "lanelet2_map.osm"]),
            ),
            DeclareLaunchArgument("scan_voxel_size", default_value="0.2"),
            DeclareLaunchArgument("crop_min_x", default_value="-5.0"),
            DeclareLaunchArgument("crop_max_x", default_value="5.0"),
            DeclareLaunchArgument("crop_min_y", default_value="-5.0"),
            DeclareLaunchArgument("crop_max_y", default_value="5.0"),
            DeclareLaunchArgument("crop_min_z", default_value="-5.0"),
            DeclareLaunchArgument("crop_max_z", default_value="5.0"),
            DeclareLaunchArgument("ndt_regularization_enable", default_value="false"),
            DeclareLaunchArgument("ndt_regularization_scale_factor", default_value="0.010"),
            DeclareLaunchArgument("ndt_regularization_pose_topic", default_value="/sensing/gnss/pose_with_covariance"),
            DeclareLaunchArgument("ndt_runtime_multistart_enable", default_value="false"),
            DeclareLaunchArgument("ndt_runtime_tracking_tier1_period_sec", default_value="1.0"),
            DeclareLaunchArgument("ndt_runtime_tracking_far_tier_period_sec", default_value="0.0"),
            DeclareLaunchArgument("ndt_runtime_raw_score_override_margin", default_value="0.0"),
            DeclareLaunchArgument("ndt_dynamic_map_radius", default_value="150.0"),
            DeclareLaunchArgument("ndt_dynamic_lidar_radius", default_value="70.0"),
            DeclareLaunchArgument("ndt_score_no_ground_points_enable", default_value="false"),
            DeclareLaunchArgument("ndt_score_no_ground_z_margin", default_value="0.8"),
            DeclareLaunchArgument(
                "ndt_converged_param_nearest_voxel_transformation_likelihood",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "ndt_validation_initial_to_result_distance_tolerance_m",
                default_value="3.0",
            ),
            DeclareLaunchArgument("ndt_num_threads", default_value="32"),
            DeclareLaunchArgument("ndt_max_iterations", default_value="40"),
            DeclareLaunchArgument("ndt_initial_pose_particles_num", default_value="200"),
            DeclareLaunchArgument("ndt_initial_pose_startup_trials", default_value="100"),
            DeclareLaunchArgument("ndt_initial_pose_include_seed_pose", default_value="true"),
            DeclareLaunchArgument("ndt_initial_pose_force_seed_yaw", default_value="true"),
            DeclareLaunchArgument("ndt_initial_pose_output_seed_yaw", default_value="true"),
            DeclareLaunchArgument("ndt_initial_pose_use_sensor_points_stamp", default_value="false"),
            DeclareLaunchArgument("ndt_initial_pose_deterministic_offsets_enable", default_value="false"),
            DeclareLaunchArgument("ndt_output_pose_time_offset_sec", default_value="-0.0075"),
            DeclareLaunchArgument("ekf_enable_yaw_bias_estimation", default_value="false"),
            DeclareLaunchArgument("ekf_pose_additional_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("ekf_twist_additional_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("ekf_proc_stddev_yaw_c", default_value="0.005"),
            DeclareLaunchArgument("ekf_proc_stddev_wz_c", default_value="5.0"),
            DeclareLaunchArgument("ekf_pose_smoothing_steps", default_value="5"),
            DeclareLaunchArgument("ekf_max_twist_queue_size", default_value="2"),
            LogInfo(
                msg=(
                    "Starting CarMaker Autoware localization graph in autoracer_hooke: "
                    "map_loader + crop/voxel + NDT + EKF + pose_initializer."
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
                package="autoware_map_projection_loader",
                executable="autoware_map_projection_loader_node",
                name="map_projection_loader",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_projector_info_path": map_projector_info_path,
                        "lanelet2_map_path": lanelet2_map_path,
                    }
                ],
            ),
            Node(
                package="autoware_map_loader",
                executable="autoware_pointcloud_map_loader",
                namespace="map",
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
                    (
                        "service/get_differential_pcd_map",
                        "/map/get_differential_pointcloud_map",
                    ),
                    ("service/get_selected_pcd_map", "/map/get_selected_pointcloud_map"),
                ],
            ),
            Node(
                package="autoware_crop_box_filter",
                executable="crop_box_filter",
                name="ndt_scan_crop_box_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_pointcloud_frame": "lidar_top",
                        "input_frame": "base_link",
                        "output_frame": "base_link",
                        "min_x": ParameterValue(crop_min_x, value_type=float),
                        "max_x": ParameterValue(crop_max_x, value_type=float),
                        "min_y": ParameterValue(crop_min_y, value_type=float),
                        "max_y": ParameterValue(crop_max_y, value_type=float),
                        "min_z": ParameterValue(crop_min_z, value_type=float),
                        "max_z": ParameterValue(crop_max_z, value_type=float),
                        "negative": True,
                        "max_queue_size": 5,
                    }
                ],
                remappings=[
                    ("input", "/sensing/lidar/concatenated/pointcloud"),
                    ("output", "/sensing/lidar/concatenated/pointcloud_self_cropped"),
                ],
            ),
            Node(
                package="autoware_downsample_filters",
                executable="voxel_grid_downsample_filter_node",
                name="ndt_scan_voxel_grid_downsample_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_frame": "base_link",
                        "output_frame": "base_link",
                        "voxel_size_x": ParameterValue(scan_voxel_size, value_type=float),
                        "voxel_size_y": ParameterValue(scan_voxel_size, value_type=float),
                        "voxel_size_z": ParameterValue(scan_voxel_size, value_type=float),
                        "max_queue_size": 5,
                    }
                ],
                remappings=[
                    ("input", "/sensing/lidar/concatenated/pointcloud_self_cropped"),
                    ("output", "/sensing/lidar/concatenated/pointcloud_downsampled"),
                ],
            ),
            Node(
                package="autoware_vehicle_velocity_converter",
                executable="autoware_vehicle_velocity_converter_node",
                name="vehicle_velocity_converter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "speed_scale_factor": 1.0,
                        "velocity_stddev_xx": 0.2,
                        "angular_velocity_stddev_zz": 0.1,
                        "frame_id": "base_link",
                    }
                ],
                remappings=[
                    ("velocity_status", "/vehicle/status/velocity_status"),
                    (
                        "twist_with_covariance",
                        "/sensing/vehicle_velocity_converter/twist_with_covariance",
                    ),
                ],
            ),
            Node(
                package="autoware_gyro_odometer",
                executable="autoware_gyro_odometer_node",
                name="gyro_odometer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "output_frame": "base_link",
                        "message_timeout_sec": 0.2,
                    }
                ],
                remappings=[
                    (
                        "vehicle/twist_with_covariance",
                        "/sensing/vehicle_velocity_converter/twist_with_covariance",
                    ),
                    ("imu", "/fixposition/rawimu"),
                    ("twist_raw", "/sensing/gyro_odometer/twist_raw"),
                    (
                        "twist_with_covariance_raw",
                        "/sensing/gyro_odometer/twist_with_covariance_raw",
                    ),
                    ("twist", "/sensing/gyro_odometer/twist"),
                    (
                        "twist_with_covariance",
                        "/sensing/gyro_odometer/twist_with_covariance",
                    ),
                ],
            ),
            Node(
                package="autoware_gnss_poser",
                executable="gnss_poser",
                name="gnss_poser",
                output="screen",
                condition=IfCondition(enable_gnss),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "base_frame": "base_link",
                        "gnss_base_frame": "gnss_base_link",
                        "map_frame": "map",
                        "buff_epoch": 1,
                        "use_gnss_ins_orientation": True,
                        "gnss_pose_pub_method": 0,
                    }
                ],
                remappings=[
                    ("fix", "/fixposition/fix"),
                    ("autoware_orientation", "/fixposition/autoware_orientation"),
                    ("gnss_pose", "/sensing/gnss/pose"),
                    ("gnss_pose_cov", "/sensing/gnss/pose_with_covariance"),
                    ("gnss_fixed", "/sensing/gnss/fixed"),
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="ground_truth_initialpose_once",
                name="ground_truth_initialpose_once",
                output="screen",
                condition=IfCondition(use_gt_initialpose_once),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_gt_topic": "/carmaker/ground_truth/pose",
                        "output_initialpose_topic": "/initialpose3d",
                        "map_frame": "map",
                        "min_stamp_sec": ParameterValue(
                            initialpose_min_stamp_sec, value_type=float
                        ),
                        "ekf_trigger_service": "/localization/pose_twist_fusion_filter/trigger_node",
                        "ndt_trigger_service": "/localization/pose_estimator/trigger_node",
                    }
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="startup_pose_initializer_once",
                name="startup_pose_initializer_once",
                output="screen",
                condition=IfCondition(use_gnss_initialpose_once),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "gnss_pose_topic": "/sensing/gnss/pose_with_covariance",
                        "initialize_service": "/localization/initialize",
                        "initialize_method": gnss_initialpose_method,
                        "min_gnss_stamp_sec": ParameterValue(
                            initialpose_min_stamp_sec, value_type=float
                        ),
                    }
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
                        "ndt.num_threads": ParameterValue(ndt_num_threads, value_type=int),
                        "ndt.max_iterations": ParameterValue(ndt_max_iterations, value_type=int),
                        "initial_pose_estimation.particles_num": ParameterValue(
                            ndt_initial_pose_particles_num, value_type=int
                        ),
                        "initial_pose_estimation.n_startup_trials": ParameterValue(
                            ndt_initial_pose_startup_trials, value_type=int
                        ),
                        "initial_pose_estimation.include_initial_pose": ParameterValue(
                            ndt_initial_pose_include_seed_pose, value_type=bool
                        ),
                        "initial_pose_estimation.force_initial_yaw": ParameterValue(
                            ndt_initial_pose_force_seed_yaw, value_type=bool
                        ),
                        "initial_pose_estimation.output_initial_yaw": ParameterValue(
                            ndt_initial_pose_output_seed_yaw, value_type=bool
                        ),
                        "initial_pose_estimation.use_sensor_points_stamp": ParameterValue(
                            ndt_initial_pose_use_sensor_points_stamp, value_type=bool
                        ),
                        "initial_pose_estimation.deterministic_offsets.enable": ParameterValue(
                            ndt_initial_pose_deterministic_offsets_enable, value_type=bool
                        ),
                        "initial_pose_estimation.deterministic_offsets.along_m": [
                            0.0,
                            -2.0,
                            -3.0,
                            -4.0,
                            -5.0,
                            1.0,
                        ],
                        "initial_pose_estimation.deterministic_offsets.cross_m": [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        "initial_pose_estimation.deterministic_offsets.yaw_deg": [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        "output_pose_time_offset_sec": ParameterValue(
                            ndt_output_pose_time_offset_sec, value_type=float
                        ),
                        "ndt.regularization.enable": ParameterValue(
                            ndt_regularization_enable, value_type=bool
                        ),
                        "ndt.regularization.scale_factor": ParameterValue(
                            ndt_regularization_scale_factor, value_type=float
                        ),
                        "dynamic_map_loading.map_radius": ParameterValue(
                            ndt_dynamic_map_radius, value_type=float
                        ),
                        "dynamic_map_loading.lidar_radius": ParameterValue(
                            ndt_dynamic_lidar_radius, value_type=float
                        ),
                        "score_estimation.no_ground_points.enable": ParameterValue(
                            ndt_score_no_ground_points_enable, value_type=bool
                        ),
                        "score_estimation.no_ground_points.z_margin_for_ground_removal": ParameterValue(
                            ndt_score_no_ground_z_margin, value_type=float
                        ),
                        "score_estimation.converged_param_nearest_voxel_transformation_likelihood": ParameterValue(
                            ndt_converged_param_nearest_voxel_transformation_likelihood,
                            value_type=float,
                        ),
                        "validation.initial_to_result_distance_tolerance_m": ParameterValue(
                            ndt_validation_initial_to_result_distance_tolerance_m,
                            value_type=float,
                        ),
                        "covariance.covariance_estimation.covariance_estimation_type": 0,
                        "runtime_multistart.enable": ParameterValue(
                            ndt_runtime_multistart_enable, value_type=bool
                        ),
                        "runtime_multistart.debug_topic": "/localization/ndt/runtime_multistart/decision",
                        "runtime_multistart.min_stamp_sec": 2.0,
                        "runtime_multistart.trigger_initial_to_result_distance_m": 0.7,
                        "runtime_multistart.trigger_yaw_delta_deg": 2.0,
                        "runtime_multistart.trigger_score_margin": 0.0,
                        "runtime_multistart.max_prior_innovation_m": 12.0,
                        "runtime_multistart.max_prior_along_m": 0.0,
                        "runtime_multistart.max_prior_cross_m": 0.0,
                        "runtime_multistart.max_prior_yaw_deg": 6.0,
                        "runtime_multistart.min_total_score": 0.0,
                        "runtime_multistart.tracking_tier1_period_sec": ParameterValue(
                            ndt_runtime_tracking_tier1_period_sec, value_type=float
                        ),
                        "runtime_multistart.tracking_far_tier_period_sec": ParameterValue(
                            ndt_runtime_tracking_far_tier_period_sec, value_type=float
                        ),
                        "runtime_multistart.raw_score_override_margin": ParameterValue(
                            ndt_runtime_raw_score_override_margin, value_type=float
                        ),
                        "runtime_multistart.raw_score_override_max_total_score_drop": 0.75,
                        "runtime_multistart.raw_score_override_max_abs_along_m": 3.0,
                        "runtime_multistart.raw_score_override_max_abs_cross_m": 1.5,
                        "runtime_multistart.raw_score_override_max_abs_yaw_deg": 3.0,
                        "runtime_multistart.raw_score_override_max_initial_to_result_distance_m": 3.0,
                        "runtime_multistart.tier1_max_abs_yaw_deg": 2.0,
                        "runtime_multistart.recovery_stable_required_frames": 3,
                        "runtime_multistart.offset_along_m": [
                            0.0,
                            0.5,
                            -0.5,
                            1.0,
                            -1.0,
                            2.0,
                            -2.0,
                            5.0,
                            -5.0,
                            10.0,
                            -10.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            10.0,
                            10.0,
                            12.0,
                            12.0,
                            15.0,
                            15.0,
                            -10.0,
                            -10.0,
                            -12.0,
                            -12.0,
                            -15.0,
                            -15.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        "runtime_multistart.offset_cross_m": [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.75,
                            -0.75,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            -10.0,
                            10.0,
                            -15.0,
                            15.0,
                            -15.0,
                            15.0,
                            -10.0,
                            10.0,
                            -15.0,
                            15.0,
                            -15.0,
                            15.0,
                            -10.0,
                            10.0,
                            -15.0,
                            15.0,
                        ],
                        "runtime_multistart.offset_yaw_deg": [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            2.0,
                            -2.0,
                            5.0,
                            -5.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                    },
                ],
                remappings=[
                    ("points_raw", "/sensing/lidar/concatenated/pointcloud_downsampled"),
                    (
                        "ekf_pose_with_covariance",
                        "/localization/pose_twist_fusion_filter/pose_with_covariance",
                    ),
                    ("regularization_pose_with_covariance", ndt_regularization_pose_topic),
                    ("trigger_node_srv", "/localization/pose_estimator/trigger_node"),
                    ("ndt_align_srv", "/localization/pose_estimator/ndt_align_srv"),
                    ("ndt_pose", "/localization/pose_estimator/pose"),
                    (
                        "ndt_pose_with_covariance",
                        "/localization/pose_estimator/pose_with_covariance",
                    ),
                    ("pcd_loader_service", "/map/get_differential_pointcloud_map"),
                ],
            ),
            Node(
                package="autoware_ekf_localizer",
                executable="autoware_ekf_localizer_node",
                name="ekf_localizer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "node": {
                            "show_debug_info": False,
                            "enable_yaw_bias_estimation": ParameterValue(
                                ekf_enable_yaw_bias_estimation, value_type=bool
                            ),
                            "predict_frequency": 50.0,
                            "tf_rate": 50.0,
                            "extend_state_step": 50,
                        },
                        "pose_measurement": {
                            "pose_additional_delay": ParameterValue(
                                ekf_pose_additional_delay_sec, value_type=float
                            ),
                            "pose_measure_uncertainty_time": 0.01,
                            "pose_smoothing_steps": ParameterValue(
                                ekf_pose_smoothing_steps, value_type=int
                            ),
                            "max_pose_queue_size": 5,
                            "pose_gate_dist": 49.5,
                        },
                        "twist_measurement": {
                            "twist_additional_delay": ParameterValue(
                                ekf_twist_additional_delay_sec, value_type=float
                            ),
                            "twist_smoothing_steps": 2,
                            "max_twist_queue_size": ParameterValue(
                                ekf_max_twist_queue_size, value_type=int
                            ),
                            "twist_gate_dist": 46.1,
                        },
                        "process_noise": {
                            "proc_stddev_yaw_c": ParameterValue(
                                ekf_proc_stddev_yaw_c, value_type=float
                            ),
                            "proc_stddev_vx_c": 10.0,
                            "proc_stddev_wz_c": ParameterValue(
                                ekf_proc_stddev_wz_c, value_type=float
                            ),
                        },
                        "simple_1d_filter_parameters": {
                            "z_filter_proc_dev": 5.0,
                            "roll_filter_proc_dev": 0.1,
                            "pitch_filter_proc_dev": 0.1,
                        },
                        "diagnostics": {
                            "pose_no_update_count_threshold_warn": 50,
                            "pose_no_update_count_threshold_error": 100,
                            "twist_no_update_count_threshold_warn": 50,
                            "twist_no_update_count_threshold_error": 100,
                            "ellipse_scale": 3.0,
                            "error_ellipse_size": 1.5,
                            "warn_ellipse_size": 1.2,
                            "error_ellipse_size_lateral_direction": 0.3,
                            "warn_ellipse_size_lateral_direction": 0.25,
                        },
                        "misc": {
                            "threshold_observable_velocity_mps": 0.0,
                            "pose_frame_id": "map",
                        },
                    }
                ],
                remappings=[
                    (
                        "in_pose_with_covariance",
                        "/localization/pose_estimator/pose_with_covariance",
                    ),
                    (
                        "in_twist_with_covariance",
                        "/sensing/gyro_odometer/twist_with_covariance",
                    ),
                    ("initialpose", "/initialpose3d"),
                    ("trigger_node_srv", "/localization/pose_twist_fusion_filter/trigger_node"),
                    ("ekf_odom", "/localization/kinematic_state"),
                    ("ekf_pose", "/localization/pose_twist_fusion_filter/pose"),
                    (
                        "ekf_pose_with_covariance",
                        "/localization/pose_twist_fusion_filter/pose_with_covariance",
                    ),
                    (
                        "ekf_biased_pose",
                        "/localization/pose_twist_fusion_filter/biased_pose",
                    ),
                    (
                        "ekf_biased_pose_with_covariance",
                        "/localization/pose_twist_fusion_filter/biased_pose_with_covariance",
                    ),
                    ("ekf_twist", "/localization/pose_twist_fusion_filter/twist"),
                    (
                        "ekf_twist_with_covariance",
                        "/localization/pose_twist_fusion_filter/twist_with_covariance",
                    ),
                    (
                        "debug/processing_time_ms",
                        "/localization/pose_twist_fusion_filter/debug/processing_time_ms",
                    ),
                ],
            ),
            Node(
                package="autoware_pose_initializer",
                executable="autoware_pose_initializer_node",
                name="pose_initializer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "user_defined_initial_pose.enable": False,
                        "user_defined_initial_pose.pose": [
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                        ],
                        "gnss_pose_timeout": 3.0,
                        "direct_pose_timeout_sec": ParameterValue(
                            diagnostic_reinit_max_gnss_pose_age_sec,
                            value_type=float,
                        ),
                        "stop_check_duration": 3.0,
                        "pose_error_threshold": 5.0,
                        "pose_error_check_enabled": False,
                        "ekf_enabled": True,
                        "gnss_enabled": ParameterValue(enable_gnss, value_type=bool),
                        "yabloc_enabled": False,
                        "ndt_enabled": True,
                        "stop_check_enabled": False,
                        "gnss_particle_covariance": [
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            10.0,
                        ],
                        "output_pose_covariance": [
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.2,
                        ],
                        "map_height_fitter.map_loader_name": "/map/pointcloud_map_loader",
                        "map_height_fitter.target": "pointcloud_map",
                    }
                ],
                remappings=[
                    ("yabloc_align", "/localization/pose_estimator/yabloc/initializer/yabloc_align_srv"),
                    ("ndt_align", "/localization/pose_estimator/ndt_align_srv"),
                    (
                        "stop_check_twist",
                        "/sensing/vehicle_velocity_converter/twist_with_covariance",
                    ),
                    ("gnss_pose_cov", "/sensing/gnss/pose_with_covariance"),
                    ("pose_reset", "/initialpose3d"),
                    ("ekf_trigger_node", "/localization/pose_twist_fusion_filter/trigger_node"),
                    ("ndt_trigger_node", "/localization/pose_estimator/trigger_node"),
                    ("~/pointcloud_map", "/map/pointcloud_map"),
                    ("~/partial_map_load", "/map/get_partial_pointcloud_map"),
                    ("~/vector_map", "/map/vector_map"),
                ],
            ),
            Node(
                package="autoware_localization_error_monitor",
                executable="autoware_localization_error_monitor_node",
                name="localization_error_monitor",
                output="screen",
                parameters=[
                    localization_error_monitor_param_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("input/odom", "/localization/kinematic_state"),
                ],
            ),
            Node(
                package="autoware_pose_instability_detector",
                executable="autoware_pose_instability_detector_node",
                name="pose_instability_detector",
                output="screen",
                parameters=[
                    pose_instability_detector_param_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("~/input/odometry", "/localization/kinematic_state"),
                    ("~/input/twist", "/sensing/gyro_odometer/twist_with_covariance"),
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="diagnostic_pose_reinitializer",
                name="diagnostic_pose_reinitializer",
                output="screen",
                condition=IfCondition(enable_gnss),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "diagnostics_topic": "/diagnostics",
                        "initialize_service": "/localization/initialize",
                        "direct_pose_topic": "/initialpose3d",
                        "gnss_pose_topic": "/sensing/gnss/pose_with_covariance",
                        "pose_observation_topic": "/localization/pose_estimator/pose_with_covariance",
                        "initialization_state_topic": "/localization/initialization_state",
                        "initialize_method": diagnostic_reinit_method,
                        "cooldown_sec": 10.0,
                        "min_diagnostic_stamp_sec": ParameterValue(
                            diagnostic_reinit_min_stamp_sec, value_type=float
                        ),
                        "post_initialization_grace_sec": ParameterValue(
                            diagnostic_reinit_post_initialization_grace_sec,
                            value_type=float,
                        ),
                        "min_trigger_duration_sec": ParameterValue(
                            diagnostic_reinit_min_trigger_duration_sec,
                            value_type=float,
                        ),
                        "max_gnss_pose_age_sec": ParameterValue(
                            diagnostic_reinit_max_gnss_pose_age_sec,
                            value_type=float,
                        ),
                        "target_status_names": [
                            "localization: pose_instability_detector",
                            "localization: ekf_localizer",
                        ],
                    }
                ],
            ),
        ]
    )
