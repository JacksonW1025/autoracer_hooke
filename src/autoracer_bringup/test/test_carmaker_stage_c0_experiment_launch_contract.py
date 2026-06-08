from pathlib import Path


NO_RTK_LAUNCH = Path("src/autoracer_bringup/launch/carmaker_stage_c0_ndt_no_rtk.launch.py")
PURE_LIDAR_LAUNCH = Path("src/autoracer_bringup/launch/carmaker_stage_c0_pure_lidar.launch.py")
NDT_HYPER_PARAMETERS = Path(
    "src/external/autoware/core/localization/autoware_ndt_scan_matcher/include/"
    "autoware/ndt_scan_matcher/hyper_parameters.hpp"
)
HOOKE2_NDT_PARAMS = Path("src/autoracer_bringup/config/hooke2/ndt_scan_matcher.param.yaml")


def test_no_rtk_launch_uses_noisy_fixposition_and_correct_axis_split():
    text = NO_RTK_LAUNCH.read_text(encoding="utf-8")

    assert "correlated_fixposition_noise" in text
    assert "/fixposition/noisy_odometry_enu" in text
    assert '"reported_xy_sigma_m": 3.0' in text
    assert '"publish_clock": False' in text
    assert '"publish_clock": True' not in text
    assert '"max_xy_stddev_m": 5.0' in text
    assert '"enable_tracking_seed_fusion": False' in text
    assert '"enable_tracking_seed_along_fusion": True' not in text
    assert '"max_tracking_seed_stddev_m": 5.0' in text
    assert '"fusion_mode": "ndt_cross_yaw_seed_along"' in text
    assert '"along_gain": 0.03' in text
    assert '"max_seed_along_residual_m": 3.0' in text
    assert '"max_seed_xy_stddev_m": 5.0' in text
    assert '"/localization/ndt/accepted_pose_with_covariance"' in text
    assert '"/localization/predicted_pose"' in text
    assert (
        '"predictor_update_topic": "/localization/ndt/accepted_pose_with_covariance"'
        in text
    )
    assert 'executable="vehicle_status_to_twist_covariance"' not in text
    assert 'package="autoware_ekf_localizer"' not in text
    assert 'executable="autoware_ekf_localizer_node"' not in text
    assert 'executable="ekf_feedback_gate"' not in text
    assert '"/vehicle/status/twist_with_covariance"' not in text
    assert '"/localization/ekf/initialpose"' not in text
    assert '"/localization/ekf_trigger"' not in text
    assert '("in_pose_with_covariance", "/localization/pose_with_covariance")' not in text
    assert (
        '"ndt_pose_topic": "/localization/ndt/accepted_pose_with_covariance"'
        in text
    )
    assert '"prediction_pose_topic": "/localization/predicted_pose"' in text
    assert '"robust_cross_gain": 1.0' in text
    assert '"robust_yaw_gain": 0.2' in text
    assert '"enable_prediction_fallback": False' in text
    assert '"relocalization_decision_topic": "/localization/relocalization/decision"' in text
    assert '"ndt.regularization.enable": False' in text
    assert 'DeclareLaunchArgument("ndt_covariance_estimation_type", default_value="1")' in text
    assert '"covariance.covariance_estimation.covariance_estimation_type": ParameterValue(' in text
    assert '"robust_update_mode": "ekf"' in text
    assert '"robust_mahalanobis_gate": 4.0' in text
    assert '"robust_ndt_covariance_estimation_type": ParameterValue(' in text
    assert '"ndt.max_iterations": 60' in text
    assert "regularization_pose_with_covariance" not in text
    assert '"min_nvtl_score": 2.3' in text
    assert '"max_iteration_num": 60' in text


def test_pure_lidar_launch_is_fixposition_free_and_has_independent_clock():
    text = PURE_LIDAR_LAUNCH.read_text(encoding="utf-8")

    forbidden = [
        "fixposition_odom_to_seed_pose",
        "fixposition_seed_filter",
        "/localization/fixposition/seed_pose",
        "/fixposition/pose_with_covariance",
        "/fixposition/noisy_odometry_enu",
        "correlated_fixposition_noise",
        "regularization_pose_with_covariance",
    ]
    for token in forbidden:
        assert token not in text

    assert "vehicle_status_clock_publisher" not in text
    assert "pointcloud_clock_publisher" not in text
    assert "bridge-owned /clock" in text
    assert "ground_truth_initialpose_once" in text
    assert '"/localization/initialpose_once"' in text
    assert 'LaunchConfiguration("initialpose_min_stamp_sec")' in text
    assert 'DeclareLaunchArgument("initialpose_min_stamp_sec", default_value="12.0")' in text
    assert '"min_stamp_sec": ParameterValue(' in text
    assert "initialpose_min_stamp_sec, value_type=float" in text
    assert '"seed_pose_topic": "/localization/initialpose_once"' in text
    assert '"enable_tracking_seed_fusion": False' in text
    assert '"ndt.regularization.enable": False' in text
    assert 'DeclareLaunchArgument("dynamic_map_radius", default_value="200.0")' in text
    assert 'DeclareLaunchArgument("dynamic_lidar_radius", default_value="120.0")' in text
    assert 'DeclareLaunchArgument("ndt_resolution", default_value="2.0")' in text
    assert 'DeclareLaunchArgument("scan_voxel_size", default_value="1.5")' in text
    assert 'DeclareLaunchArgument("initial_pose_correction_gain", default_value="1.0")' in text
    assert 'DeclareLaunchArgument("initial_pose_along_correction_gain", default_value="-1.0")' in text
    assert 'DeclareLaunchArgument("initial_pose_cross_correction_gain", default_value="-1.0")' in text
    assert 'DeclareLaunchArgument("initial_pose_yaw_correction_gain", default_value="-1.0")' in text
    assert '"dynamic_map_loading.map_radius": ParameterValue(' in text
    assert '"dynamic_map_loading.lidar_radius": ParameterValue(' in text
    assert '"ndt.resolution": ParameterValue(ndt_resolution, value_type=float)' in text
    assert '"voxel_size_x": ParameterValue(scan_voxel_size, value_type=float)' in text
    assert "ndt_axis_seed_fuser" in text
    assert '"seed_pose_topic": "/localization/no_seed_available"' in text
    assert '"initial_pose_topic": "/localization/ndt_initial_pose"' in text
    assert 'DeclareLaunchArgument("enable_robust_initial_update", default_value="true")' in text
    assert '"enable_robust_initial_update": ParameterValue(' in text
    assert 'DeclareLaunchArgument("robust_mahalanobis_gate", default_value="4.0")' in text
    assert '"robust_update_mode": "ekf"' in text
    assert '"robust_mahalanobis_gate": ParameterValue(' in text
    assert '"robust_ndt_covariance_estimation_type": ParameterValue(' in text
    assert 'DeclareLaunchArgument("robust_measurement_along_variance_floor_m2", default_value="0.09")' in text
    assert 'DeclareLaunchArgument("robust_measurement_cross_variance_floor_m2", default_value="-1.0")' in text
    assert '"robust_decision_topic": "/localization/robust_ndt/decision"' in text
    assert '"runtime_multistart_decision_topic": "/localization/ndt/runtime_multistart/decision"' in text
    assert '"robust_candidate_spread_along_threshold_m": 1.5' in text
    assert '"robust_candidate_spread_along_variance_scale": 0.5' in text
    assert '"robust_candidate_spread_max_abs_along_m": 3.0' in text
    assert '"robust_candidate_spread_max_abs_cross_m": 0.85' in text
    assert '"robust_candidate_spread_score_margin": 0.0' in text
    assert '"/localization/ndt/accepted_pose_with_covariance"' in text
    assert '"/localization/predicted_pose"' in text
    assert '"predictor_update_topic": "/localization/ndt/accepted_pose_with_covariance"' in text
    assert (
        'DeclareLaunchArgument("predictor_update_requires_robust_high_confidence", default_value="false")'
        in text
    )
    assert (
        'DeclareLaunchArgument("predictor_update_high_confidence_min_stamp_sec", default_value="45.0")'
        in text
    )
    assert 'DeclareLaunchArgument("predictor_update_max_mahalanobis", default_value="2.0")' in text
    assert (
        'DeclareLaunchArgument("predictor_update_max_innovation_along_m", default_value="0.8")'
        in text
    )
    assert (
        'DeclareLaunchArgument("predictor_update_max_innovation_cross_m", default_value="0.25")'
        in text
    )
    assert (
        'DeclareLaunchArgument("predictor_update_max_innovation_yaw_deg", default_value="2.0")'
        in text
    )
    assert '"predictor_update_requires_robust_high_confidence": ParameterValue(' in text
    assert '"predictor_update_high_confidence_min_stamp_sec": ParameterValue(' in text
    assert '"predictor_update_max_innovation_cross_m": ParameterValue(' in text
    assert '"prediction_pose_topic": "/localization/predicted_pose"' in text
    assert 'DeclareLaunchArgument("enable_prediction_fallback", default_value="false")' in text
    assert '"enable_prediction_fallback": ParameterValue(' in text
    assert '"relocalization_decision_topic": "/localization/relocalization/decision"' in text
    assert 'DeclareLaunchArgument("motion_velocity_scale_error", default_value="0.0")' in text
    assert '"motion_velocity_scale_error": ParameterValue(' in text
    assert 'DeclareLaunchArgument("enable_motion_scale_correction", default_value="false")' in text
    assert 'DeclareLaunchArgument("motion_scale_correction_min_distance_m", default_value="20.0")' in text
    assert (
        'DeclareLaunchArgument("motion_scale_correction_bootstrap_min_abs", default_value="0.01")'
        in text
    )
    assert (
        'DeclareLaunchArgument("motion_scale_correction_bootstrap_min_updates", default_value="3")'
        in text
    )
    assert (
        'DeclareLaunchArgument("motion_scale_correction_bootstrap_initial_observation_count", default_value="1")'
        in text
    )
    assert '"motion_scale_correction_bootstrap_initial_observation_count": ParameterValue(' in text
    assert 'DeclareLaunchArgument("preserve_tracking_ndt_along", default_value="false")' in text
    assert '"preserve_tracking_ndt_along": ParameterValue(' in text
    assert '"motion_scale_correction_bootstrap_min_abs": ParameterValue(' in text
    assert '"motion_scale_correction_bootstrap_min_updates": ParameterValue(' in text
    assert '"tracking_ndt_max_along_correction_m": 0.0' in text
    assert 'DeclareLaunchArgument("runtime_multistart_min_stamp_sec", default_value="12.0")' in text
    assert 'DeclareLaunchArgument(' in text
    assert '"runtime_multistart_trigger_initial_to_result_distance_m"' in text
    assert 'default_value="0.7"' in text
    assert '"runtime_multistart_trigger_yaw_delta_deg"' in text
    assert '"runtime_multistart_max_prior_innovation_m", default_value="12.0"' in text
    assert '"runtime_multistart_base_candidate_raw_score_margin", default_value="0.0"' in text
    assert (
        '"runtime_multistart_tracking_tier1_period_sec", default_value="0.0"'
        in text
    )
    assert (
        '"runtime_multistart_tracking_along_health_period_sec", default_value="0.0"'
        in text
    )
    assert '"runtime_multistart_raw_score_override_margin", default_value="0.0"' in text
    assert (
        '"runtime_multistart_raw_score_override_max_total_score_drop",'
        in text
    )
    assert (
        '"runtime_multistart_raw_score_override_max_abs_along_m",'
        in text
    )
    assert (
        '"runtime_multistart_raw_score_override_max_abs_cross_m",'
        in text
    )
    assert (
        '"runtime_multistart_raw_score_override_max_abs_yaw_deg",'
        in text
    )
    assert (
        '"runtime_multistart_raw_score_override_max_initial_to_result_distance_m",'
        in text
    )
    assert '"runtime_multistart.min_stamp_sec": ParameterValue(' in text
    assert '"runtime_multistart.trigger_initial_to_result_distance_m": ParameterValue(' in text
    assert '"runtime_multistart.trigger_yaw_delta_deg": ParameterValue(' in text
    assert '"runtime_multistart.max_prior_innovation_m": ParameterValue(' in text
    assert '"runtime_multistart.base_candidate_raw_score_margin": ParameterValue(' in text
    assert '"runtime_multistart.tracking_tier1_period_sec": ParameterValue(' in text
    assert '"runtime_multistart.tracking_along_health_period_sec": ParameterValue(' in text
    assert '"runtime_multistart.raw_score_override_margin": ParameterValue(' in text
    assert (
        '"runtime_multistart.raw_score_override_max_total_score_drop": ParameterValue('
        in text
    )
    assert '"runtime_multistart.raw_score_override_max_abs_along_m": ParameterValue(' in text
    assert '"runtime_multistart.raw_score_override_max_abs_cross_m": ParameterValue(' in text
    assert '"runtime_multistart.raw_score_override_max_abs_yaw_deg": ParameterValue(' in text
    assert (
        '"runtime_multistart.raw_score_override_max_initial_to_result_distance_m": ParameterValue('
        in text
    )
    assert '"runtime_multistart.recovery_stable_required_frames": 3' in text
    assert "15.0," in text
    assert "-15.0," in text
    assert '"enable_lost_recovery_hypotheses": False' in text
    assert 'DeclareLaunchArgument("ndt_covariance_estimation_type", default_value="1")' in text
    assert '"covariance.covariance_estimation.covariance_estimation_type": ParameterValue(' in text
    assert '"max_ndt_initial_yaw_delta_deg": 5.0' in text
    assert '"max_ndt_initial_distance_m": 3.0' in text
    assert '"initial_pose_correction_gain": ParameterValue(' in text
    assert "initial_pose_correction_gain, value_type=float" in text
    assert '"initial_pose_along_correction_gain": ParameterValue(' in text
    assert '"initial_pose_cross_correction_gain": ParameterValue(' in text
    assert '"initial_pose_yaw_correction_gain": ParameterValue(' in text
    assert '"required_initial_messages": 1' in text
    assert '"min_nvtl_score": 2.3' in text


def test_runtime_multistart_tier_parameters_are_initialized_for_all_bringup_paths():
    hyper_parameters = NDT_HYPER_PARAMETERS.read_text(encoding="utf-8")
    hooke2_params = HOOKE2_NDT_PARAMS.read_text(encoding="utf-8")
    required_defaults = {
        "runtime_multistart.tier1_max_abs_along_m": "1.0",
        "runtime_multistart.tier1_max_abs_cross_m": "0.75",
        "runtime_multistart.tier1_max_abs_yaw_deg": "2.0",
        "runtime_multistart.tracking_tier1_period_sec": "1.0",
        "runtime_multistart.tracking_along_health_period_sec": "0.0",
        "runtime_multistart.innovation_along_penalty_weight": "0.05",
        "runtime_multistart.innovation_cross_penalty_weight": "0.55",
        "runtime_multistart.raw_score_override_margin": "0.0",
        "runtime_multistart.raw_score_override_max_total_score_drop": "0.0",
        "runtime_multistart.raw_score_override_max_abs_along_m": "0.0",
        "runtime_multistart.raw_score_override_max_abs_cross_m": "0.0",
        "runtime_multistart.raw_score_override_max_abs_yaw_deg": "0.0",
        "runtime_multistart.raw_score_override_max_initial_to_result_distance_m": "0.0",
        "runtime_multistart.ambiguity_score_margin": "0.15",
        "runtime_multistart.ambiguity_along_spread_m": "1.5",
        "runtime_multistart.recovery_stable_required_frames": "3",
        "runtime_multistart.recovery_stable_max_innovation_m": "1.0",
        "runtime_multistart.recovery_stable_max_yaw_deg": "5.0",
        "runtime_multistart.recovery_far_tier_period_sec": "1.0",
        "runtime_multistart.recovery_far_tier_min_scan_interval": "10",
    }

    for name, default in required_defaults.items():
        assert f'"{name}", {default}' in hyper_parameters
        assert name.split(".")[-1] in hooke2_params
