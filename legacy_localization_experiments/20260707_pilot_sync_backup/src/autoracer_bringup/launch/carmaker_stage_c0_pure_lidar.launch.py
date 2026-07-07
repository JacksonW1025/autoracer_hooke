from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_map_path = LaunchConfiguration("localization_map_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    dynamic_map_radius = LaunchConfiguration("dynamic_map_radius")
    dynamic_lidar_radius = LaunchConfiguration("dynamic_lidar_radius")
    initialpose_min_stamp_sec = LaunchConfiguration("initialpose_min_stamp_sec")
    ndt_resolution = LaunchConfiguration("ndt_resolution")
    scan_voxel_size = LaunchConfiguration("scan_voxel_size")
    initial_pose_correction_gain = LaunchConfiguration("initial_pose_correction_gain")
    initial_pose_along_correction_gain = LaunchConfiguration("initial_pose_along_correction_gain")
    initial_pose_cross_correction_gain = LaunchConfiguration("initial_pose_cross_correction_gain")
    initial_pose_yaw_correction_gain = LaunchConfiguration("initial_pose_yaw_correction_gain")
    ndt_max_iterations = LaunchConfiguration("ndt_max_iterations")
    ndt_covariance_estimation_type = LaunchConfiguration("ndt_covariance_estimation_type")
    enable_robust_initial_update = LaunchConfiguration("enable_robust_initial_update")
    robust_mahalanobis_gate = LaunchConfiguration("robust_mahalanobis_gate")
    robust_measurement_xy_variance_floor_m2 = LaunchConfiguration(
        "robust_measurement_xy_variance_floor_m2"
    )
    robust_measurement_yaw_variance_floor_rad2 = LaunchConfiguration(
        "robust_measurement_yaw_variance_floor_rad2"
    )
    robust_measurement_along_variance_floor_m2 = LaunchConfiguration(
        "robust_measurement_along_variance_floor_m2"
    )
    robust_measurement_cross_variance_floor_m2 = LaunchConfiguration(
        "robust_measurement_cross_variance_floor_m2"
    )
    robust_along_gain = LaunchConfiguration("robust_along_gain")
    robust_cross_gain = LaunchConfiguration("robust_cross_gain")
    robust_yaw_gain = LaunchConfiguration("robust_yaw_gain")
    robust_z_gain = LaunchConfiguration("robust_z_gain")
    robust_roll_pitch_gain = LaunchConfiguration("robust_roll_pitch_gain")
    predictor_update_requires_robust_high_confidence = LaunchConfiguration(
        "predictor_update_requires_robust_high_confidence"
    )
    predictor_update_high_confidence_min_stamp_sec = LaunchConfiguration(
        "predictor_update_high_confidence_min_stamp_sec"
    )
    predictor_update_max_mahalanobis = LaunchConfiguration(
        "predictor_update_max_mahalanobis"
    )
    predictor_update_max_innovation_along_m = LaunchConfiguration(
        "predictor_update_max_innovation_along_m"
    )
    predictor_update_max_innovation_cross_m = LaunchConfiguration(
        "predictor_update_max_innovation_cross_m"
    )
    predictor_update_max_innovation_yaw_deg = LaunchConfiguration(
        "predictor_update_max_innovation_yaw_deg"
    )
    output_along_bias_m = LaunchConfiguration("output_along_bias_m")
    output_cross_bias_m = LaunchConfiguration("output_cross_bias_m")
    robust_hard_reject_correction_m = LaunchConfiguration("robust_hard_reject_correction_m")
    robust_hard_reject_yaw_deg = LaunchConfiguration("robust_hard_reject_yaw_deg")
    enable_prediction_fallback = LaunchConfiguration("enable_prediction_fallback")
    motion_velocity_scale_error = LaunchConfiguration("motion_velocity_scale_error")
    motion_yaw_rate_bias_rad_s = LaunchConfiguration("motion_yaw_rate_bias_rad_s")
    motion_yaw_rate_random_walk_stddev = LaunchConfiguration(
        "motion_yaw_rate_random_walk_stddev_rad_sqrt_s"
    )
    motion_velocity_white_noise_stddev = LaunchConfiguration(
        "motion_velocity_white_noise_stddev_mps"
    )
    enable_motion_scale_correction = LaunchConfiguration("enable_motion_scale_correction")
    motion_scale_correction_alpha = LaunchConfiguration("motion_scale_correction_alpha")
    motion_scale_correction_max_abs = LaunchConfiguration("motion_scale_correction_max_abs")
    motion_scale_correction_min_distance_m = LaunchConfiguration(
        "motion_scale_correction_min_distance_m"
    )
    motion_scale_correction_max_cross_residual_m = LaunchConfiguration(
        "motion_scale_correction_max_cross_residual_m"
    )
    motion_scale_correction_observation_limit = LaunchConfiguration(
        "motion_scale_correction_observation_limit"
    )
    motion_scale_correction_max_step_abs = LaunchConfiguration(
        "motion_scale_correction_max_step_abs"
    )
    motion_scale_correction_bootstrap_min_abs = LaunchConfiguration(
        "motion_scale_correction_bootstrap_min_abs"
    )
    motion_scale_correction_bootstrap_min_updates = LaunchConfiguration(
        "motion_scale_correction_bootstrap_min_updates"
    )
    motion_scale_correction_bootstrap_initial_observation_count = LaunchConfiguration(
        "motion_scale_correction_bootstrap_initial_observation_count"
    )
    motion_scale_correction_min_stamp_sec = LaunchConfiguration(
        "motion_scale_correction_min_stamp_sec"
    )
    motion_scale_correction_require_robust_decision = LaunchConfiguration(
        "motion_scale_correction_require_robust_decision"
    )
    motion_scale_correction_robust_decision_max_age_sec = LaunchConfiguration(
        "motion_scale_correction_robust_decision_max_age_sec"
    )
    motion_scale_correction_max_mahalanobis = LaunchConfiguration(
        "motion_scale_correction_max_mahalanobis"
    )
    motion_scale_correction_max_innovation_along_m = LaunchConfiguration(
        "motion_scale_correction_max_innovation_along_m"
    )
    motion_scale_correction_max_innovation_cross_m = LaunchConfiguration(
        "motion_scale_correction_max_innovation_cross_m"
    )
    motion_scale_correction_max_innovation_yaw_deg = LaunchConfiguration(
        "motion_scale_correction_max_innovation_yaw_deg"
    )
    preserve_tracking_ndt_along = LaunchConfiguration("preserve_tracking_ndt_along")
    runtime_multistart_min_stamp_sec = LaunchConfiguration("runtime_multistart_min_stamp_sec")
    runtime_multistart_trigger_initial_to_result_distance_m = LaunchConfiguration(
        "runtime_multistart_trigger_initial_to_result_distance_m"
    )
    runtime_multistart_trigger_yaw_delta_deg = LaunchConfiguration(
        "runtime_multistart_trigger_yaw_delta_deg"
    )
    runtime_multistart_max_prior_innovation_m = LaunchConfiguration(
        "runtime_multistart_max_prior_innovation_m"
    )
    runtime_multistart_base_candidate_raw_score_margin = LaunchConfiguration(
        "runtime_multistart_base_candidate_raw_score_margin"
    )
    runtime_multistart_tracking_tier1_period_sec = LaunchConfiguration(
        "runtime_multistart_tracking_tier1_period_sec"
    )
    runtime_multistart_tracking_along_health_period_sec = LaunchConfiguration(
        "runtime_multistart_tracking_along_health_period_sec"
    )
    runtime_multistart_raw_score_override_margin = LaunchConfiguration(
        "runtime_multistart_raw_score_override_margin"
    )
    runtime_multistart_raw_score_override_max_total_score_drop = LaunchConfiguration(
        "runtime_multistart_raw_score_override_max_total_score_drop"
    )
    runtime_multistart_raw_score_override_max_abs_along_m = LaunchConfiguration(
        "runtime_multistart_raw_score_override_max_abs_along_m"
    )
    runtime_multistart_raw_score_override_max_abs_cross_m = LaunchConfiguration(
        "runtime_multistart_raw_score_override_max_abs_cross_m"
    )
    runtime_multistart_raw_score_override_max_abs_yaw_deg = LaunchConfiguration(
        "runtime_multistart_raw_score_override_max_abs_yaw_deg"
    )
    runtime_multistart_raw_score_override_max_initial_to_result_distance_m = LaunchConfiguration(
        "runtime_multistart_raw_score_override_max_initial_to_result_distance_m"
    )

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
            DeclareLaunchArgument("localization_map_path"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("dynamic_map_radius", default_value="200.0"),
            DeclareLaunchArgument("dynamic_lidar_radius", default_value="120.0"),
            DeclareLaunchArgument("initialpose_min_stamp_sec", default_value="12.0"),
            DeclareLaunchArgument("ndt_resolution", default_value="2.0"),
            DeclareLaunchArgument("scan_voxel_size", default_value="1.5"),
            DeclareLaunchArgument("initial_pose_correction_gain", default_value="1.0"),
            DeclareLaunchArgument("initial_pose_along_correction_gain", default_value="-1.0"),
            DeclareLaunchArgument("initial_pose_cross_correction_gain", default_value="-1.0"),
            DeclareLaunchArgument("initial_pose_yaw_correction_gain", default_value="-1.0"),
            DeclareLaunchArgument("ndt_max_iterations", default_value="60"),
            DeclareLaunchArgument("ndt_covariance_estimation_type", default_value="1"),
            DeclareLaunchArgument("enable_robust_initial_update", default_value="true"),
            DeclareLaunchArgument("robust_mahalanobis_gate", default_value="4.0"),
            DeclareLaunchArgument("robust_measurement_xy_variance_floor_m2", default_value="1.0"),
            DeclareLaunchArgument("robust_measurement_along_variance_floor_m2", default_value="0.09"),
            DeclareLaunchArgument("robust_measurement_cross_variance_floor_m2", default_value="-1.0"),
            DeclareLaunchArgument("robust_measurement_yaw_variance_floor_rad2", default_value="0.04"),
            DeclareLaunchArgument("robust_along_gain", default_value="0.2"),
            DeclareLaunchArgument("robust_cross_gain", default_value="0.2"),
            DeclareLaunchArgument("robust_yaw_gain", default_value="0.2"),
            DeclareLaunchArgument("robust_z_gain", default_value="1.0"),
            DeclareLaunchArgument("robust_roll_pitch_gain", default_value="1.0"),
            DeclareLaunchArgument("predictor_update_requires_robust_high_confidence", default_value="false"),
            DeclareLaunchArgument("predictor_update_high_confidence_min_stamp_sec", default_value="45.0"),
            DeclareLaunchArgument("predictor_update_max_mahalanobis", default_value="2.0"),
            DeclareLaunchArgument("predictor_update_max_innovation_along_m", default_value="0.8"),
            DeclareLaunchArgument("predictor_update_max_innovation_cross_m", default_value="0.25"),
            DeclareLaunchArgument("predictor_update_max_innovation_yaw_deg", default_value="2.0"),
            DeclareLaunchArgument("output_along_bias_m", default_value="0.0"),
            DeclareLaunchArgument("output_cross_bias_m", default_value="0.0"),
            DeclareLaunchArgument("robust_hard_reject_correction_m", default_value="8.0"),
            DeclareLaunchArgument("robust_hard_reject_yaw_deg", default_value="25.0"),
            DeclareLaunchArgument("enable_prediction_fallback", default_value="false"),
            DeclareLaunchArgument("motion_velocity_scale_error", default_value="0.0"),
            DeclareLaunchArgument("motion_yaw_rate_bias_rad_s", default_value="0.0"),
            DeclareLaunchArgument(
                "motion_yaw_rate_random_walk_stddev_rad_sqrt_s", default_value="0.0"
            ),
            DeclareLaunchArgument("motion_velocity_white_noise_stddev_mps", default_value="0.0"),
            DeclareLaunchArgument("enable_motion_scale_correction", default_value="false"),
            DeclareLaunchArgument("preserve_tracking_ndt_along", default_value="false"),
            DeclareLaunchArgument("motion_scale_correction_alpha", default_value="0.05"),
            DeclareLaunchArgument("motion_scale_correction_max_abs", default_value="0.03"),
            DeclareLaunchArgument("motion_scale_correction_min_distance_m", default_value="20.0"),
            DeclareLaunchArgument(
                "motion_scale_correction_max_cross_residual_m", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "motion_scale_correction_observation_limit", default_value="0.03"
            ),
            DeclareLaunchArgument("motion_scale_correction_max_step_abs", default_value="0.002"),
            DeclareLaunchArgument("motion_scale_correction_bootstrap_min_abs", default_value="0.01"),
            DeclareLaunchArgument("motion_scale_correction_bootstrap_min_updates", default_value="3"),
            DeclareLaunchArgument("motion_scale_correction_bootstrap_initial_observation_count", default_value="1"),
            DeclareLaunchArgument("motion_scale_correction_min_stamp_sec", default_value="45.0"),
            DeclareLaunchArgument(
                "motion_scale_correction_require_robust_decision", default_value="true"
            ),
            DeclareLaunchArgument(
                "motion_scale_correction_robust_decision_max_age_sec", default_value="0.2"
            ),
            DeclareLaunchArgument("motion_scale_correction_max_mahalanobis", default_value="2.0"),
            DeclareLaunchArgument(
                "motion_scale_correction_max_innovation_along_m", default_value="0.8"
            ),
            DeclareLaunchArgument(
                "motion_scale_correction_max_innovation_cross_m", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "motion_scale_correction_max_innovation_yaw_deg", default_value="2.0"
            ),
            DeclareLaunchArgument("runtime_multistart_min_stamp_sec", default_value="12.0"),
            DeclareLaunchArgument(
                "runtime_multistart_trigger_initial_to_result_distance_m",
                default_value="0.7",
            ),
            DeclareLaunchArgument(
                "runtime_multistart_trigger_yaw_delta_deg", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "runtime_multistart_max_prior_innovation_m", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "runtime_multistart_base_candidate_raw_score_margin", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "runtime_multistart_tracking_tier1_period_sec", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "runtime_multistart_tracking_along_health_period_sec", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "runtime_multistart_raw_score_override_margin", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "runtime_multistart_raw_score_override_max_total_score_drop",
                default_value="0.75",
            ),
            DeclareLaunchArgument(
                "runtime_multistart_raw_score_override_max_abs_along_m",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "runtime_multistart_raw_score_override_max_abs_cross_m",
                default_value="0.85",
            ),
            DeclareLaunchArgument(
                "runtime_multistart_raw_score_override_max_abs_yaw_deg",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "runtime_multistart_raw_score_override_max_initial_to_result_distance_m",
                default_value="3.0",
            ),
            LogInfo(
                msg=(
                    "Starting Exp2 pure LiDAR NDT graph: no Fixposition consumers, "
                    "bridge-owned /clock, one-shot GT initialpose, no regularization."
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
                    (
                        "service/get_differential_pcd_map",
                        "/map/get_differential_pointcloud_map",
                    ),
                    ("service/get_selected_pcd_map", "/map/get_selected_pointcloud_map"),
                ],
            ),
            Node(
                package="autoracer_localization",
                executable="ground_truth_initialpose_once",
                name="ground_truth_initialpose_once",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_gt_topic": "/carmaker/ground_truth/pose",
                        "output_initialpose_topic": "/localization/initialpose_once",
                        "map_frame": "map",
                        "min_stamp_sec": ParameterValue(
                            initialpose_min_stamp_sec, value_type=float
                        ),
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
                        "seed_pose_topic": "/localization/initialpose_once",
                        "ndt_pose_topic": "/localization/ndt/accepted_pose_with_covariance",
                        "velocity_topic": "/vehicle/status/velocity_status",
                        "steering_topic": "/vehicle/status/steering_status",
                        "output_topic": "/localization/ndt_initial_pose",
                        "prediction_output_topic": "/localization/predicted_pose",
                        "regularization_seed_topic": "",
                        "corrected_seed_topic": "",
                        "map_frame": "map",
                        "publish_rate_hz": 20.0,
                        "wheel_base_m": 1.9,
                        "vehicle_status_timeout_sec": 0.5,
                        "ndt_lost_timeout_sec": 3.0,
                        "seed_reset_cooldown_sec": 999999.0,
                        "enable_seed_bias_correction": False,
                        "enable_tracking_seed_fusion": False,
                        "max_tracking_seed_stddev_m": 0.0,
                        "max_tracking_seed_age_sec": 0.5,
                        "ndt_seed_deviation_guard_m": 0.0,
                        "log_seed_decisions": True,
                        "enable_lost_recovery_hypotheses": False,
                        "recovery_hypothesis_period_sec": 0.2,
                        "relocalization_decision_topic": "/localization/relocalization/decision",
                        "motion_noise_seed": 424242,
                        "motion_velocity_scale_error": ParameterValue(
                            motion_velocity_scale_error, value_type=float
                        ),
                        "motion_yaw_rate_bias_rad_s": ParameterValue(
                            motion_yaw_rate_bias_rad_s, value_type=float
                        ),
                        "motion_yaw_rate_random_walk_stddev_rad_sqrt_s": ParameterValue(
                            motion_yaw_rate_random_walk_stddev, value_type=float
                        ),
                        "motion_velocity_white_noise_stddev_mps": ParameterValue(
                            motion_velocity_white_noise_stddev, value_type=float
                        ),
                        "enable_motion_scale_correction": ParameterValue(
                            enable_motion_scale_correction, value_type=bool
                        ),
                        "preserve_tracking_ndt_along": ParameterValue(
                            preserve_tracking_ndt_along, value_type=bool
                        ),
                        "tracking_ndt_max_along_correction_m": 0.0,
                        "motion_scale_correction_alpha": ParameterValue(
                            motion_scale_correction_alpha, value_type=float
                        ),
                        "motion_scale_correction_max_abs": ParameterValue(
                            motion_scale_correction_max_abs, value_type=float
                        ),
                        "motion_scale_correction_min_distance_m": ParameterValue(
                            motion_scale_correction_min_distance_m, value_type=float
                        ),
                        "motion_scale_correction_max_cross_residual_m": ParameterValue(
                            motion_scale_correction_max_cross_residual_m, value_type=float
                        ),
                        "motion_scale_correction_observation_limit": ParameterValue(
                            motion_scale_correction_observation_limit, value_type=float
                        ),
                        "motion_scale_correction_max_step_abs": ParameterValue(
                            motion_scale_correction_max_step_abs, value_type=float
                        ),
                        "motion_scale_correction_bootstrap_min_abs": ParameterValue(
                            motion_scale_correction_bootstrap_min_abs, value_type=float
                        ),
                        "motion_scale_correction_bootstrap_min_updates": ParameterValue(
                            motion_scale_correction_bootstrap_min_updates, value_type=int
                        ),
                        "motion_scale_correction_bootstrap_initial_observation_count": ParameterValue(
                            motion_scale_correction_bootstrap_initial_observation_count,
                            value_type=int,
                        ),
                        "motion_scale_correction_min_stamp_sec": ParameterValue(
                            motion_scale_correction_min_stamp_sec, value_type=float
                        ),
                        "motion_scale_correction_robust_decision_topic": "/localization/robust_ndt/decision",
                        "motion_scale_correction_require_robust_decision": ParameterValue(
                            motion_scale_correction_require_robust_decision,
                            value_type=bool,
                        ),
                        "motion_scale_correction_robust_decision_max_age_sec": ParameterValue(
                            motion_scale_correction_robust_decision_max_age_sec,
                            value_type=float,
                        ),
                        "motion_scale_correction_max_mahalanobis": ParameterValue(
                            motion_scale_correction_max_mahalanobis, value_type=float
                        ),
                        "motion_scale_correction_max_innovation_along_m": ParameterValue(
                            motion_scale_correction_max_innovation_along_m,
                            value_type=float,
                        ),
                        "motion_scale_correction_max_innovation_cross_m": ParameterValue(
                            motion_scale_correction_max_innovation_cross_m,
                            value_type=float,
                        ),
                        "motion_scale_correction_max_innovation_yaw_deg": ParameterValue(
                            motion_scale_correction_max_innovation_yaw_deg,
                            value_type=float,
                        ),
                        "recovery_hypothesis_along_offsets_m": [
                            0.0,
                            2.0,
                            -2.0,
                            5.0,
                            -5.0,
                            10.0,
                            -10.0,
                        ],
                        "recovery_hypothesis_cross_offsets_m": [0.0, 0.75, -0.75],
                        "recovery_hypothesis_yaw_offsets_deg": [0.0, 3.0, -3.0],
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
                        "voxel_size_x": ParameterValue(scan_voxel_size, value_type=float),
                        "voxel_size_y": ParameterValue(scan_voxel_size, value_type=float),
                        "voxel_size_z": ParameterValue(scan_voxel_size, value_type=float),
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
                        "ndt.max_iterations": ParameterValue(
                            ndt_max_iterations, value_type=int
                        ),
                        "ndt.resolution": ParameterValue(ndt_resolution, value_type=float),
                        "ndt.regularization.enable": False,
                        "covariance.covariance_estimation.covariance_estimation_type": ParameterValue(
                            ndt_covariance_estimation_type, value_type=int
                        ),
                        "dynamic_map_loading.map_radius": ParameterValue(
                            dynamic_map_radius, value_type=float
                        ),
                        "dynamic_map_loading.lidar_radius": ParameterValue(
                            dynamic_lidar_radius, value_type=float
                        ),
                        "runtime_multistart.enable": True,
                        "runtime_multistart.debug_topic": "/localization/ndt/runtime_multistart/decision",
                        "runtime_multistart.min_stamp_sec": ParameterValue(
                            runtime_multistart_min_stamp_sec, value_type=float
                        ),
                        "runtime_multistart.trigger_initial_to_result_distance_m": ParameterValue(
                            runtime_multistart_trigger_initial_to_result_distance_m,
                            value_type=float,
                        ),
                        "runtime_multistart.trigger_yaw_delta_deg": ParameterValue(
                            runtime_multistart_trigger_yaw_delta_deg, value_type=float
                        ),
                        "runtime_multistart.trigger_score_margin": 0.0,
                        "runtime_multistart.max_prior_innovation_m": ParameterValue(
                            runtime_multistart_max_prior_innovation_m, value_type=float
                        ),
                        "runtime_multistart.max_prior_along_m": 0.0,
                        "runtime_multistart.max_prior_cross_m": 0.0,
                        "runtime_multistart.max_prior_yaw_deg": 6.0,
                        "runtime_multistart.tier1_max_abs_yaw_deg": 2.0,
                        "runtime_multistart.min_total_score": 0.0,
                        "runtime_multistart.base_candidate_raw_score_margin": ParameterValue(
                            runtime_multistart_base_candidate_raw_score_margin,
                            value_type=float,
                        ),
                        "runtime_multistart.tracking_tier1_period_sec": ParameterValue(
                            runtime_multistart_tracking_tier1_period_sec,
                            value_type=float,
                        ),
                        "runtime_multistart.tracking_along_health_period_sec": ParameterValue(
                            runtime_multistart_tracking_along_health_period_sec,
                            value_type=float,
                        ),
                        "runtime_multistart.raw_score_override_margin": ParameterValue(
                            runtime_multistart_raw_score_override_margin,
                            value_type=float,
                        ),
                        "runtime_multistart.raw_score_override_max_total_score_drop": ParameterValue(
                            runtime_multistart_raw_score_override_max_total_score_drop,
                            value_type=float,
                        ),
                        "runtime_multistart.raw_score_override_max_abs_along_m": ParameterValue(
                            runtime_multistart_raw_score_override_max_abs_along_m,
                            value_type=float,
                        ),
                        "runtime_multistart.raw_score_override_max_abs_cross_m": ParameterValue(
                            runtime_multistart_raw_score_override_max_abs_cross_m,
                            value_type=float,
                        ),
                        "runtime_multistart.raw_score_override_max_abs_yaw_deg": ParameterValue(
                            runtime_multistart_raw_score_override_max_abs_yaw_deg,
                            value_type=float,
                        ),
                        "runtime_multistart.raw_score_override_max_initial_to_result_distance_m": ParameterValue(
                            runtime_multistart_raw_score_override_max_initial_to_result_distance_m,
                            value_type=float,
                        ),
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
                    ("ekf_pose_with_covariance", "/localization/ndt_initial_pose"),
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
                        "seed_pose_topic": "/localization/no_seed_available",
                        "initial_pose_topic": "/localization/ndt_initial_pose",
                        "prediction_pose_topic": "/localization/predicted_pose",
                        "max_initial_pose_age_sec": 0.2,
                        "max_ndt_initial_distance_m": 3.0,
                        "max_ndt_initial_yaw_delta_deg": 5.0,
                        "initial_pose_correction_gain": ParameterValue(
                            initial_pose_correction_gain, value_type=float
                        ),
                        "initial_pose_along_correction_gain": ParameterValue(
                            initial_pose_along_correction_gain, value_type=float
                        ),
                        "initial_pose_cross_correction_gain": ParameterValue(
                            initial_pose_cross_correction_gain, value_type=float
                        ),
                        "initial_pose_yaw_correction_gain": ParameterValue(
                            initial_pose_yaw_correction_gain, value_type=float
                        ),
                        "enable_robust_initial_update": ParameterValue(
                            enable_robust_initial_update, value_type=bool
                        ),
                        "robust_update_mode": "ekf",
                        "robust_mahalanobis_gate": ParameterValue(
                            robust_mahalanobis_gate, value_type=float
                        ),
                        "robust_ndt_covariance_estimation_type": ParameterValue(
                            ndt_covariance_estimation_type, value_type=int
                        ),
                        "robust_prior_xy_variance_floor_m2": 0.25,
                        "robust_prior_yaw_variance_floor_rad2": 0.007615435494667714,
                        "robust_measurement_xy_variance_floor_m2": ParameterValue(
                            robust_measurement_xy_variance_floor_m2, value_type=float
                        ),
                        "robust_measurement_along_variance_floor_m2": ParameterValue(
                            robust_measurement_along_variance_floor_m2, value_type=float
                        ),
                        "robust_measurement_cross_variance_floor_m2": ParameterValue(
                            robust_measurement_cross_variance_floor_m2, value_type=float
                        ),
                        "robust_measurement_yaw_variance_floor_rad2": ParameterValue(
                            robust_measurement_yaw_variance_floor_rad2, value_type=float
                        ),
                        "robust_along_gain": ParameterValue(robust_along_gain, value_type=float),
                        "robust_cross_gain": ParameterValue(robust_cross_gain, value_type=float),
                        "robust_yaw_gain": ParameterValue(robust_yaw_gain, value_type=float),
                        "robust_z_gain": ParameterValue(robust_z_gain, value_type=float),
                        "robust_roll_pitch_gain": ParameterValue(
                            robust_roll_pitch_gain, value_type=float
                        ),
                        "robust_max_along_correction_m": 0.35,
                        "robust_max_cross_correction_m": 0.35,
                        "robust_max_yaw_correction_deg": 3.0,
                        "robust_hard_reject_correction_m": ParameterValue(
                            robust_hard_reject_correction_m, value_type=float
                        ),
                        "robust_hard_reject_yaw_deg": ParameterValue(
                            robust_hard_reject_yaw_deg, value_type=float
                        ),
                        "robust_decision_topic": "/localization/robust_ndt/decision",
                        "runtime_multistart_decision_topic": "/localization/ndt/runtime_multistart/decision",
                        "runtime_multistart_decision_max_age_sec": 0.2,
                        "robust_candidate_spread_min_candidate_count": 2,
                        "robust_candidate_spread_along_threshold_m": 1.5,
                        "robust_candidate_spread_along_variance_scale": 0.5,
                        "robust_candidate_spread_max_abs_along_m": 3.0,
                        "robust_candidate_spread_max_abs_cross_m": 0.85,
                        "robust_candidate_spread_score_margin": 0.0,
                        "robust_candidate_spread_yaw_threshold_deg": 3.0,
                        "robust_candidate_spread_yaw_variance_scale": 0.5,
                        "predictor_update_topic": "/localization/ndt/accepted_pose_with_covariance",
                        "predictor_update_requires_robust_high_confidence": ParameterValue(
                            predictor_update_requires_robust_high_confidence, value_type=bool
                        ),
                        "predictor_update_high_confidence_min_stamp_sec": ParameterValue(
                            predictor_update_high_confidence_min_stamp_sec, value_type=float
                        ),
                        "predictor_update_max_mahalanobis": ParameterValue(
                            predictor_update_max_mahalanobis, value_type=float
                        ),
                        "predictor_update_max_innovation_along_m": ParameterValue(
                            predictor_update_max_innovation_along_m, value_type=float
                        ),
                        "predictor_update_max_innovation_cross_m": ParameterValue(
                            predictor_update_max_innovation_cross_m, value_type=float
                        ),
                        "predictor_update_max_innovation_yaw_deg": ParameterValue(
                            predictor_update_max_innovation_yaw_deg, value_type=float
                        ),
                        "enable_prediction_fallback": ParameterValue(
                            enable_prediction_fallback, value_type=bool
                        ),
                        "output_along_bias_m": ParameterValue(
                            output_along_bias_m, value_type=float
                        ),
                        "output_cross_bias_m": ParameterValue(
                            output_cross_bias_m, value_type=float
                        ),
                        "prediction_fallback_min_age_sec": 0.25,
                        "prediction_fallback_xy_variance_floor_m2": 4.0,
                        "prediction_fallback_yaw_variance_floor_rad2": 0.25,
                        "output_topic": "/localization/pose_with_covariance",
                        "max_seed_age_sec": 0.0,
                        "max_seed_xy_stddev_m": 0.0,
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
                        "required_initial_messages": 1,
                        "fresh_initial_pose_sec": 1.0,
                        "ndt_pose_timeout_sec": 6.0,
                        "retrigger_cooldown_sec": 5.0,
                        "min_nvtl_score": 2.3,
                        "max_iteration_num": ParameterValue(ndt_max_iterations, value_type=int),
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
        ]
    )
