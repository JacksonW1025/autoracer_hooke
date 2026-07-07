import re
from pathlib import Path


AUTORACER_ROOT = Path(__file__).resolve().parents[3]

LAUNCH = AUTORACER_ROOT / "src/autoracer_bringup/launch/carmaker_autoware_localization.launch.py"
PACKAGE = AUTORACER_ROOT / "src/autoracer_bringup/package.xml"
LOCALIZATION_SETUP = AUTORACER_ROOT / "src/autoracer_localization/setup.py"
NDT_PARAMS = AUTORACER_ROOT / "src/autoracer_bringup/config/hooke2/ndt_scan_matcher.param.yaml"
PROJECTOR_INFO = (
    AUTORACER_ROOT / "src/autoracer_bringup/config/hooke2/carmaker_map_projector_info.yaml"
)
POSE_INSTABILITY_PARAM = AUTORACER_ROOT / (
    "src/autoracer_bringup/config/hooke2/pose_instability_detector_carmaker.param.yaml"
)
GNSS_POSER = AUTORACER_ROOT / (
    "src/external/autoware/core/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp"
)
POSE_INSTABILITY_SOURCE = AUTORACER_ROOT / (
    "src/external/autoware/universe/localization/autoware_pose_instability_detector/src/"
    "pose_instability_detector.cpp"
)
NDT_CORE = AUTORACER_ROOT / (
    "src/external/autoware/core/localization/autoware_ndt_scan_matcher/src/"
    "ndt_scan_matcher_core.cpp"
)
NDT_HYPER_PARAMETERS = AUTORACER_ROOT / (
    "src/external/autoware/core/localization/autoware_ndt_scan_matcher/include/"
    "autoware/ndt_scan_matcher/hyper_parameters.hpp"
)


def test_autoware_localization_launch_uses_migrated_autoware_stack():
    text = LAUNCH.read_text(encoding="utf-8")

    required_packages = [
        "autoware_map_projection_loader",
        "autoware_map_loader",
        "autoware_crop_box_filter",
        "autoware_downsample_filters",
        "autoware_vehicle_velocity_converter",
        "autoware_gyro_odometer",
        "autoware_localization_error_monitor",
        "autoware_ndt_scan_matcher",
        "autoware_ekf_localizer",
        "autoware_pose_initializer",
        "autoware_pose_instability_detector",
        "autoware_gnss_poser",
    ]
    for package in required_packages:
        assert f'package="{package}"' in text or f'get_package_share_directory("{package}")' in text

    retired_nodes = [
        "ndt_axis_seed_fuser",
        "ndt_initial_pose_predictor",
        "fixposition_seed_filter",
    ]
    for node in retired_nodes:
        assert node not in text

    assert '"/sensing/lidar/concatenated/pointcloud"' in text
    assert '"/sensing/lidar/concatenated/pointcloud_self_cropped"' in text
    assert '"/sensing/lidar/concatenated/pointcloud_downsampled"' in text
    assert '"input_pointcloud_frame": "lidar_top"' in text
    assert '"input_frame": "base_link"' in text
    assert '"output_frame": "base_link"' in text

    assert '"/vehicle/status/velocity_status"' in text
    assert '"/sensing/vehicle_velocity_converter/twist_with_covariance"' in text
    assert '"/fixposition/rawimu"' in text
    assert '"/sensing/gyro_odometer/twist_with_covariance"' in text

    assert '"/localization/pose_estimator/pose_with_covariance"' in text
    assert '"/localization/pose_twist_fusion_filter/pose_with_covariance"' in text
    assert '"/localization/kinematic_state"' in text
    assert '"/initialpose3d"' in text
    assert "ground_truth_initialpose_once" in text
    assert "IfCondition(use_gt_initialpose_once)" in text
    assert '"ekf_trigger_service": "/localization/pose_twist_fusion_filter/trigger_node"' in text
    assert '"ndt_trigger_service": "/localization/pose_estimator/trigger_node"' in text

    assert 'DeclareLaunchArgument("ndt_regularization_enable", default_value="false")' in text
    assert 'DeclareLaunchArgument("ndt_regularization_scale_factor", default_value="0.010")' in text
    assert (
        'DeclareLaunchArgument("ndt_regularization_pose_topic", '
        'default_value="/sensing/gnss/pose_with_covariance")' in text
    )
    assert 'DeclareLaunchArgument("ndt_dynamic_map_radius", default_value="150.0")' in text
    assert 'DeclareLaunchArgument("ndt_dynamic_lidar_radius", default_value="70.0")' in text
    assert 'DeclareLaunchArgument("ndt_score_no_ground_points_enable", default_value="false")' in text
    assert 'DeclareLaunchArgument("ndt_score_no_ground_z_margin", default_value="0.8")' in text
    assert 'DeclareLaunchArgument("ndt_num_threads", default_value="16")' in text
    assert 'DeclareLaunchArgument("ndt_max_iterations", default_value="40")' in text
    assert '"ndt.num_threads": ParameterValue(' in text
    assert "ndt_num_threads, value_type=int" in text
    assert '"ndt.max_iterations": ParameterValue(' in text
    assert "ndt_max_iterations, value_type=int" in text
    assert '"ndt.regularization.enable": ParameterValue(' in text
    assert "ndt_regularization_enable, value_type=bool" in text
    assert '"ndt.regularization.scale_factor": ParameterValue(' in text
    assert "ndt_regularization_scale_factor, value_type=float" in text
    assert '("regularization_pose_with_covariance", ndt_regularization_pose_topic)' in text
    assert '"dynamic_map_loading.map_radius": ParameterValue(' in text
    assert "ndt_dynamic_map_radius, value_type=float" in text
    assert '"dynamic_map_loading.lidar_radius": ParameterValue(' in text
    assert "ndt_dynamic_lidar_radius, value_type=float" in text
    assert '"score_estimation.no_ground_points.enable": ParameterValue(' in text
    assert "ndt_score_no_ground_points_enable, value_type=bool" in text
    assert '"score_estimation.no_ground_points.z_margin_for_ground_removal": ParameterValue(' in text
    assert "ndt_score_no_ground_z_margin, value_type=float" in text
    assert (
        'DeclareLaunchArgument(\n'
        '                "ndt_validation_initial_to_result_distance_tolerance_m",' in text
    )
    assert 'default_value="3.0",' in text
    assert '"validation.initial_to_result_distance_tolerance_m": ParameterValue(' in text
    assert re.search(
        r"ndt_validation_initial_to_result_distance_tolerance_m,\s*value_type=float",
        text,
    )
    assert '"covariance.covariance_estimation.covariance_estimation_type": 0' in text
    assert '"pose_gate_dist": 49.5' in text
    assert '"twist_gate_dist": 46.1' in text
    assert '"pose_smoothing_steps": ParameterValue(' in text
    assert 'DeclareLaunchArgument("ekf_max_twist_queue_size", default_value="2")' in text
    assert '"max_twist_queue_size": ParameterValue(' in text
    assert "ekf_max_twist_queue_size, value_type=int" in text


def test_autoware_localization_launch_exposes_curve_yaw_ekf_controls():
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ekf_enable_yaw_bias_estimation", default_value="false")' in text
    assert 'DeclareLaunchArgument("ekf_pose_additional_delay_sec", default_value="0.0")' in text
    assert 'DeclareLaunchArgument("ekf_twist_additional_delay_sec", default_value="0.0")' in text
    assert 'DeclareLaunchArgument("ekf_proc_stddev_yaw_c", default_value="0.005")' in text
    assert 'DeclareLaunchArgument("ekf_proc_stddev_wz_c", default_value="5.0")' in text
    assert 'DeclareLaunchArgument("ekf_pose_smoothing_steps", default_value="10")' in text
    assert "ekf_enable_yaw_bias_estimation = LaunchConfiguration(" in text
    assert "ekf_pose_smoothing_steps = LaunchConfiguration(" in text
    assert "ekf_twist_additional_delay_sec = LaunchConfiguration(" in text
    assert '"enable_yaw_bias_estimation": ParameterValue(' in text
    assert "ekf_enable_yaw_bias_estimation, value_type=bool" in text
    assert '"pose_additional_delay": ParameterValue(' in text
    assert "ekf_pose_additional_delay_sec, value_type=float" in text
    assert '"twist_additional_delay": ParameterValue(' in text
    assert "ekf_twist_additional_delay_sec, value_type=float" in text
    assert '"proc_stddev_yaw_c": ParameterValue(' in text
    assert "ekf_proc_stddev_yaw_c, value_type=float" in text
    assert '"proc_stddev_wz_c": ParameterValue(' in text
    assert "ekf_proc_stddev_wz_c, value_type=float" in text
    assert '"pose_smoothing_steps": ParameterValue(' in text
    assert "ekf_pose_smoothing_steps, value_type=int" in text


def test_autoware_localization_launch_can_power_pure_lidar_tracker_without_changing_baseline():
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_pure_lidar_fixed_lag_tracker", default_value="false")' in text
    assert (
        'DeclareLaunchArgument(\n'
        '                "ndt_initial_pose_topic",\n'
        '                default_value="/localization/pose_twist_fusion_filter/pose_with_covariance",'
    ) in text
    assert 'executable="pure_lidar_fixed_lag_tracker"' in text
    assert "condition=IfCondition(enable_pure_lidar_fixed_lag_tracker)" in text
    assert '"ekf_pose_with_covariance",\n                        ndt_initial_pose_topic,' in text
    assert '"enable_scan_submap_residual": ParameterValue(' in text
    assert re.search(
        r"pure_lidar_tracker_enable_scan_submap_residual,\s*value_type=bool",
        text,
    )
    assert '"enable_degenerate_along_remap": ParameterValue(' in text
    assert re.search(
        r"pure_lidar_tracker_enable_degenerate_along_remap,\s*value_type=bool",
        text,
    )


def test_autoware_localization_defaults_feed_dense_scans_to_ndt():
    launch = LAUNCH.read_text(encoding="utf-8")
    ndt_params = NDT_PARAMS.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("scan_voxel_size", default_value="0.2")' in launch
    assert 'DeclareLaunchArgument("ndt_dynamic_lidar_radius", default_value="70.0")' in launch
    assert '"ndt_converged_param_nearest_voxel_transformation_likelihood"' in launch
    assert 'default_value="1.0"' in launch
    assert "ndt_converged_param_nearest_voxel_transformation_likelihood = LaunchConfiguration(" in launch
    assert '"score_estimation.converged_param_nearest_voxel_transformation_likelihood": ParameterValue(' in launch
    assert re.search(
        r"ndt_converged_param_nearest_voxel_transformation_likelihood,\s*value_type=float",
        launch,
    )
    assert "      resolution: 1.0" in ndt_params
    assert "        enable: false" in ndt_params
    assert "      converged_param_nearest_voxel_transformation_likelihood: 1.0" in ndt_params
    assert re.search(r"0\.0, 0\.0, 0\.0, 0\.0, 0\.0, 0\.000625,", ndt_params)
    assert "        covariance_estimation_type: 3" in ndt_params
    assert "        scale_factor: 4.0" in ndt_params
    assert "      min_transform_probability: 0.0" in ndt_params
    assert "      min_nearest_voxel_transformation_likelihood: 1.0" in ndt_params
    assert "      lidar_radius: 70.0" in ndt_params


def test_autoware_localization_launch_exposes_ndt_startup_align_controls():
    launch = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ndt_initial_pose_particles_num", default_value="200")' in launch
    assert 'DeclareLaunchArgument("ndt_initial_pose_startup_trials", default_value="100")' in launch
    assert 'DeclareLaunchArgument("ndt_initial_pose_include_seed_pose", default_value="true")' in launch
    assert 'DeclareLaunchArgument("ndt_initial_pose_force_seed_yaw", default_value="true")' in launch
    assert 'DeclareLaunchArgument("ndt_initial_pose_output_seed_yaw", default_value="true")' in launch
    assert 'DeclareLaunchArgument("ndt_initial_pose_use_sensor_points_stamp", default_value="false")' in launch
    assert (
        'DeclareLaunchArgument("ndt_initial_pose_deterministic_offsets_enable", default_value="false")'
        in launch
    )
    assert "ndt_initial_pose_particles_num = LaunchConfiguration(" in launch
    assert "ndt_initial_pose_startup_trials = LaunchConfiguration(" in launch
    assert "ndt_initial_pose_include_seed_pose = LaunchConfiguration(" in launch
    assert "ndt_initial_pose_force_seed_yaw = LaunchConfiguration(" in launch
    assert "ndt_initial_pose_output_seed_yaw = LaunchConfiguration(" in launch
    assert "ndt_initial_pose_use_sensor_points_stamp = LaunchConfiguration(" in launch
    assert "ndt_initial_pose_deterministic_offsets_enable = LaunchConfiguration(" in launch
    assert '"initial_pose_estimation.particles_num": ParameterValue(' in launch
    assert "ndt_initial_pose_particles_num, value_type=int" in launch
    assert '"initial_pose_estimation.n_startup_trials": ParameterValue(' in launch
    assert "ndt_initial_pose_startup_trials, value_type=int" in launch
    assert '"initial_pose_estimation.include_initial_pose": ParameterValue(' in launch
    assert "ndt_initial_pose_include_seed_pose, value_type=bool" in launch
    assert '"initial_pose_estimation.force_initial_yaw": ParameterValue(' in launch
    assert "ndt_initial_pose_force_seed_yaw, value_type=bool" in launch
    assert '"initial_pose_estimation.output_initial_yaw": ParameterValue(' in launch
    assert "ndt_initial_pose_output_seed_yaw, value_type=bool" in launch
    assert '"initial_pose_estimation.use_sensor_points_stamp": ParameterValue(' in launch
    assert "ndt_initial_pose_use_sensor_points_stamp, value_type=bool" in launch
    assert '"initial_pose_estimation.deterministic_offsets.enable": ParameterValue(' in launch
    assert "ndt_initial_pose_deterministic_offsets_enable, value_type=bool" in launch
    assert '"initial_pose_estimation.deterministic_offsets.along_m": [' in launch
    assert '"initial_pose_estimation.deterministic_offsets.cross_m": [' in launch
    assert '"initial_pose_estimation.deterministic_offsets.yaw_deg": [' in launch
    assert "-6.0" in launch
    assert "6.0" in launch
    assert "-4.0" in launch
    assert "4.0" in launch
    assert launch.count("# startup 5m GNSS bounded-search grid") == 1
    assert launch.count("# along=-6") == 5
    assert launch.count("# along=0") == 5
    assert launch.count("# along=6") == 5


def test_ndt_startup_align_can_seed_and_constrain_yaw_for_no_rtk_initialization():
    core = NDT_CORE.read_text(encoding="utf-8")
    hyper_parameters = NDT_HYPER_PARAMETERS.read_text(encoding="utf-8")

    assert "bool include_initial_pose{}" in hyper_parameters
    assert "bool force_initial_yaw{}" in hyper_parameters
    assert "bool output_initial_yaw{}" in hyper_parameters
    assert 'declare_parameter<bool>("initial_pose_estimation.include_initial_pose")' in hyper_parameters
    assert 'declare_parameter<bool>("initial_pose_estimation.force_initial_yaw")' in hyper_parameters
    assert 'declare_parameter<bool>("initial_pose_estimation.output_initial_yaw")' in hyper_parameters
    assert "std::max<int64_t>(1" in core
    assert "param_.initial_pose_estimation.include_initial_pose" in core
    assert "param_.initial_pose_estimation.force_initial_yaw" in core
    assert "param_.initial_pose_estimation.output_initial_yaw" in core
    assert "input[5] = base_rpy.z" in core
    assert "output_quaternion.setRPY(output_rpy.x, output_rpy.y, base_rpy.z)" in core


def test_autoware_localization_launch_exposes_ndt_output_pose_time_offset():
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ndt_output_pose_time_offset_sec", default_value="-0.0075")' in text
    assert "ndt_output_pose_time_offset_sec = LaunchConfiguration(" in text
    assert '"output_pose_time_offset_sec": ParameterValue(' in text
    assert "ndt_output_pose_time_offset_sec, value_type=float" in text


def test_autoware_localization_launch_keeps_ndt_runtime_multistart_disabled_by_default():
    text = LAUNCH.read_text(encoding="utf-8")
    ndt_params = NDT_PARAMS.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ndt_runtime_multistart_enable", default_value="false")' in text
    assert 'DeclareLaunchArgument("ndt_runtime_multistart_observer_enable", default_value="false")' in text
    assert '"runtime_multistart.enable": ParameterValue(' in text
    assert "ndt_runtime_multistart_enable, value_type=bool" in text
    assert '"runtime_multistart.observer_enable": ParameterValue(' in text
    assert "ndt_runtime_multistart_observer_enable, value_type=bool" in text
    assert (
        '"runtime_multistart.debug_topic": "/localization/ndt/runtime_multistart/decision"'
        in text
    )
    assert '"runtime_multistart.observer_topic": ndt_runtime_multistart_observer_topic' in text
    assert (
        '"runtime_multistart.observer_debug_topic": '
        'ndt_runtime_multistart_observer_debug_topic'
    ) in text
    assert '"runtime_multistart.min_total_score": 0.0' in text
    assert '"runtime_multistart.max_prior_innovation_m": 12.0' in text
    assert 'DeclareLaunchArgument("ndt_runtime_tracking_tier1_period_sec", default_value="1.0")' in text
    assert '"runtime_multistart.tracking_tier1_period_sec": ParameterValue(' in text
    assert "ndt_runtime_tracking_tier1_period_sec, value_type=float" in text
    assert 'DeclareLaunchArgument("ndt_runtime_tracking_far_tier_period_sec", default_value="0.0")' in text
    assert '"runtime_multistart.tracking_far_tier_period_sec": ParameterValue(' in text
    assert "ndt_runtime_tracking_far_tier_period_sec, value_type=float" in text
    assert 'DeclareLaunchArgument("ndt_runtime_raw_score_override_margin", default_value="0.0")' in text
    assert '"runtime_multistart.raw_score_override_margin": ParameterValue(' in text
    assert "ndt_runtime_raw_score_override_margin, value_type=float" in text
    assert 'DeclareLaunchArgument("ndt_runtime_enable_gnss_weak_prior", default_value="false")' in text
    assert '"runtime_multistart.enable_gnss_weak_prior": ParameterValue(' in text
    assert "ndt_runtime_enable_gnss_weak_prior, value_type=bool" in text
    assert (
        'DeclareLaunchArgument(\n'
        '                "ndt_runtime_gnss_weak_prior_topic",\n'
        '                default_value="/sensing/gnss/pose_with_covariance",\n'
        "            )"
        in text
    )
    assert '"runtime_multistart.gnss_weak_prior_topic": ndt_runtime_gnss_weak_prior_topic' in text
    assert 'DeclareLaunchArgument("ndt_runtime_gnss_weak_prior_sigma_m", default_value="5.0")' in text
    assert '"runtime_multistart.gnss_weak_prior_sigma_m": ParameterValue(' in text
    assert "ndt_runtime_gnss_weak_prior_sigma_m, value_type=float" in text
    assert 'DeclareLaunchArgument("ndt_runtime_gnss_weak_prior_weight", default_value="1.0")' in text
    assert '"runtime_multistart.gnss_weak_prior_weight": ParameterValue(' in text
    assert "ndt_runtime_gnss_weak_prior_weight, value_type=float" in text
    assert 'DeclareLaunchArgument("ndt_runtime_gnss_weak_prior_max_age_sec", default_value="0.5")' in text
    assert '"runtime_multistart.gnss_weak_prior_max_age_sec": ParameterValue(' in text
    assert "ndt_runtime_gnss_weak_prior_max_age_sec, value_type=float" in text
    assert 'DeclareLaunchArgument("ndt_runtime_gnss_weak_prior_max_penalty", default_value="8.0")' in text
    assert '"runtime_multistart.gnss_weak_prior_max_penalty": ParameterValue(' in text
    assert "ndt_runtime_gnss_weak_prior_max_penalty, value_type=float" in text
    assert '"runtime_multistart.raw_score_override_max_total_score_drop": 0.75' in text
    assert '"runtime_multistart.raw_score_override_max_abs_along_m": 3.0' in text
    assert '"runtime_multistart.raw_score_override_max_abs_cross_m": 1.5' in text
    assert '"runtime_multistart.raw_score_override_max_abs_yaw_deg": 3.0' in text
    assert '"runtime_multistart.raw_score_override_max_initial_to_result_distance_m": 3.0' in text
    assert "      innovation_cross_penalty_weight: 0.15" in ndt_params
    assert "      initial_to_result_penalty_weight: 0.1" in ndt_params
    assert "      tier1_max_abs_along_m: 2.0" in ndt_params
    assert re.search(r"runtime_multistart:\s*\n\s+enable: false", ndt_params)

    offset_vectors = {}
    for key in ["offset_along_m", "offset_cross_m", "offset_yaw_deg"]:
        match = re.search(
            rf'"runtime_multistart\.{key}": \[(?P<body>.*?)\]',
            text,
            flags=re.DOTALL,
        )
        assert match is not None
        offset_vectors[key] = [
            float(value)
            for value in re.findall(r"-?\d+(?:\.\d+)?", match.group("body"))
        ]

    vector_lengths = {len(values) for values in offset_vectors.values()}
    assert vector_lengths == {33}
    assert offset_vectors["offset_along_m"][0] == 0.0
    assert offset_vectors["offset_cross_m"][0] == 0.0
    assert offset_vectors["offset_yaw_deg"][0] == 0.0
    assert 0.5 in offset_vectors["offset_along_m"]
    assert -0.5 in offset_vectors["offset_along_m"]
    assert 2.0 in offset_vectors["offset_along_m"]
    assert -2.0 in offset_vectors["offset_along_m"]
    assert 5.0 in offset_vectors["offset_along_m"]
    assert -5.0 in offset_vectors["offset_along_m"]
    assert 10.0 in offset_vectors["offset_along_m"]
    assert -10.0 in offset_vectors["offset_along_m"]
    assert 0.75 in offset_vectors["offset_cross_m"]
    assert -0.75 in offset_vectors["offset_cross_m"]
    assert 2.0 not in offset_vectors["offset_cross_m"]
    assert -2.0 not in offset_vectors["offset_cross_m"]
    assert 2.0 in offset_vectors["offset_yaw_deg"]
    assert -2.0 in offset_vectors["offset_yaw_deg"]

    assert (
        'DeclareLaunchArgument(\n'
        '                "enable_runtime_candidate_selector_shadow", default_value="false"'
    ) in text
    assert (
        'DeclareLaunchArgument(\n'
        '                "enable_runtime_candidate_selector_gated_takeover", default_value="false"'
    ) in text
    assert 'DeclareLaunchArgument("enable_independent_candidate_observer", default_value="false")' in text
    assert 'default_value="/localization/candidate_observer/candidates"' in text
    assert 'default_value="/localization/candidate_observer/diagnostics"' in text
    assert '"independent_candidate_observer_publish_min_period_sec"' in text
    assert '"independent_candidate_observer_alignment_wall_delay_ms"' in text
    assert '"independent_candidate_observer_include_unaligned_hypotheses"' in text
    assert '"independent_ndt_candidate_observer_initial_pose_topic"' in text
    assert '"independent_ndt_candidate_observer_points_topic"' in text
    assert '"independent_ndt_candidate_observer_max_iterations"' in text
    assert '"independent_ndt_candidate_observer_max_candidates_per_scan"' in text
    assert '"independent_ndt_candidate_observer_scan_voxel_leaf_size_m"' in text
    assert '"independent_ndt_candidate_observer_health_trigger_enable"' in text
    assert '"independent_ndt_candidate_observer_health_i2r_m"' in text
    assert '"independent_ndt_candidate_observer_health_min_nvtl"' in text
    assert '"independent_ndt_candidate_observer_enable_gnss_weak_prior"' in text
    assert '"independent_ndt_candidate_observer_gnss_weak_prior_sigma_m"' in text
    assert '"independent_ndt_candidate_observer_offset_along_m"' in text
    assert 'executable="candidate_observer"' in text
    assert '"enable_independent_ndt_candidate_observer", default_value="false"' in text
    assert 'executable="independent_candidate_observer_node"' in text
    assert "condition=IfCondition(enable_independent_ndt_candidate_observer)" in text
    assert 'prefix="ionice -c 3 nice -n 19"' in text
    assert '"map_source": "pcd_tiles"' in text
    assert '"points_topic": independent_ndt_candidate_observer_points_topic' in text
    assert '"initial_pose_topic": independent_ndt_candidate_observer_initial_pose_topic' in text
    assert '"enable_gnss_weak_prior": ParameterValue(' in text
    assert '"gnss_weak_prior_sigma_m": ParameterValue(' in text
    assert '"map_directory": localization_map_path' in text
    assert '"map_topic": "/debug/loaded_pointcloud_map"' in text
    assert '"map_tile_resolution_m": 20.0' in text
    assert '"max_tiles_per_update": 180' in text
    assert '"scan_voxel_leaf_size_m": ParameterValue(' in text
    assert '"max_candidates_per_scan": ParameterValue(' in text
    assert '"alignment_wall_delay_ms": ParameterValue(' in text
    assert '"health_trigger_enable": ParameterValue(' in text
    assert '"health_trigger_i2r_m": ParameterValue(' in text
    assert '"health_trigger_max_iteration_count": ParameterValue(' in text
    assert '"offset_along_m": ParameterValue(' in text
    assert "independent_ndt_candidate_observer_offset_cross_m," in text
    assert "value_type=str" in text
    assert '"use_sim_time": False' in text
    assert "condition=IfCondition(enable_independent_candidate_observer)" in text
    assert '"runtime_candidate_selector_observer_topic"' in text
    assert '"runtime_candidate_selector_stable_required_frames"' in text
    assert '"runtime_candidate_selector_allow_index0_takeover"' in text
    assert '"observer_topic": runtime_candidate_selector_observer_topic' in text
    assert '"allow_index0_takeover": ParameterValue(' in text
    assert 'executable="runtime_candidate_selector"' in text
    assert '"/localization/selector_shadow/pose_with_covariance"' in text
    assert '"/localization/selector_gated/pose_with_covariance"' in text


def test_ndt_runtime_observer_does_not_modify_base_output_covariance():
    core = NDT_CORE.read_text(encoding="utf-8")

    assert "param_.runtime_multistart.enable && runtime_spread_covariance.ambiguous" in core
    assert "if (runtime_spread_covariance.ambiguous)" not in core
    assert (
        "runtime_observer_only && !param_.runtime_multistart.force_zero_offsets_only"
        in core
    )


def test_autoware_localization_launch_keeps_ndt_pose_source_without_continuous_gnss_fusion():
    text = LAUNCH.read_text(encoding="utf-8")

    assert "autoware_pose_covariance_modifier" not in text
    assert '"/localization/pose_estimator/ndt_scan_matcher/pose_with_covariance"' not in text
    assert '"ndt_pose_with_covariance"' in text
    assert '"in_pose_with_covariance"' in text
    assert (
        '"ndt_pose_with_covariance",\n'
        '                        "/localization/pose_estimator/pose_with_covariance",'
        in text
    )
    assert (
        '"in_pose_with_covariance",\n'
        '                        "/localization/pose_estimator/pose_with_covariance",'
        in text
    ) or (
        '"in_pose_with_covariance",\n'
        '                        ekf_input_pose_topic,'
        in text
    )
    assert (
        'DeclareLaunchArgument(\n'
        '                "ekf_input_pose_topic",\n'
        '                default_value="/localization/pose_estimator/pose_with_covariance",'
        in text
    )
    assert '("gnss_pose_cov", "/sensing/gnss/pose_with_covariance")' in text


def test_autoware_localization_launch_has_runtime_reinitialization_safety_net():
    text = LAUNCH.read_text(encoding="utf-8")
    param_text = POSE_INSTABILITY_PARAM.read_text(encoding="utf-8")
    setup_text = LOCALIZATION_SETUP.read_text(encoding="utf-8")

    assert 'package="autoware_localization_error_monitor"' in text
    assert 'executable="autoware_localization_error_monitor_node"' in text
    assert '"input/odom", "/localization/kinematic_state"' in text

    assert 'package="autoware_pose_instability_detector"' in text
    assert 'executable="autoware_pose_instability_detector_node"' in text
    assert '"~/input/odometry", "/localization/kinematic_state"' in text
    assert '"~/input/twist", "/sensing/gyro_odometer/twist_with_covariance"' in text

    assert 'executable="diagnostic_pose_reinitializer"' in text
    assert "condition=IfCondition(enable_gnss)" in text
    assert '"initialize_service": "/localization/initialize"' in text
    assert '"direct_pose_timeout_sec": ParameterValue(' in text
    assert '"direct_pose_topic": "/initialpose3d"' in text
    assert 'DeclareLaunchArgument("diagnostic_reinit_min_stamp_sec", default_value="5.0")' in text
    assert '"diagnostic_reinit_post_initialization_grace_sec", default_value="5.0"' in text
    assert '"diagnostic_reinit_min_trigger_duration_sec", default_value="1.0"' in text
    assert '"diagnostic_reinit_max_gnss_pose_age_sec", default_value="0.5"' in text
    assert 'DeclareLaunchArgument("diagnostic_reinit_method", default_value="direct")' in text
    assert "diagnostic_reinit_method = LaunchConfiguration(" in text
    assert "diagnostic_reinit_min_trigger_duration_sec = LaunchConfiguration(" in text
    assert '"gnss_pose_topic": "/sensing/gnss/pose_with_covariance"' in text
    assert '"pose_observation_topic": "/localization/pose_estimator/pose_with_covariance"' in text
    assert '"initialization_state_topic": "/localization/initialization_state"' in text
    assert '"initialize_method": diagnostic_reinit_method' in text
    assert "diagnostic_reinit_min_stamp_sec = LaunchConfiguration(" in text
    assert '"min_diagnostic_stamp_sec": ParameterValue(' in text
    assert '"post_initialization_grace_sec": ParameterValue(' in text
    assert '"min_trigger_duration_sec": ParameterValue(' in text
    assert '"max_gnss_pose_age_sec": ParameterValue(' in text
    assert '"target_status_names": [' in text
    assert '"localization: pose_instability_detector"' in text
    assert '"localization: ekf_localizer"' in text
    assert (
        '"ellipse_error_status",\n'
        '                            "localization: pose_instability_detector",'
        not in text
    )

    assert "timer_period: 0.5" in param_text
    assert "pose_estimator_longitudinal_tolerance: 0.5" in param_text
    assert "pose_estimator_lateral_tolerance: 0.5" in param_text

    assert "startup_pose_initializer_once = " in setup_text
    assert 'DeclareLaunchArgument("use_gnss_initialpose_once", default_value="false")' in text
    assert 'DeclareLaunchArgument("gnss_initialpose_method", default_value="auto")' in text
    assert "use_gnss_initialpose_once = LaunchConfiguration(" in text
    assert "gnss_initialpose_method = LaunchConfiguration(" in text
    assert 'executable="startup_pose_initializer_once"' in text
    assert "condition=IfCondition(use_gnss_initialpose_once)" in text
    assert '"gnss_pose_topic": "/sensing/gnss/pose_with_covariance"' in text
    assert '"initialize_service": "/localization/initialize"' in text
    assert '"initialize_method": gnss_initialpose_method' in text
    assert '"min_gnss_stamp_sec": ParameterValue(' in text


def test_gnss_poser_static_tf_lookup_uses_latest_transform_for_sim_time_bridge():
    text = GNSS_POSER.read_text(encoding="utf-8")

    assert "lookupTransform(target_frame, source_frame, tf2::TimePointZero)" in text
    assert "CarMaker publishes /fixposition/fix at the current sim stamp" in text


def test_pose_instability_detector_tolerates_time_jumps_after_reinitialization():
    text = POSE_INSTABILITY_SOURCE.read_text(encoding="utf-8")

    assert "latest_odometry_time <= prev_odometry_time" in text
    assert "Skip pose instability check on non-increasing odometry stamp" in text


def test_autoware_localization_launch_uses_sim_local_cartesian_projector():
    launch_text = LAUNCH.read_text(encoding="utf-8")
    projector_text = PROJECTOR_INFO.read_text(encoding="utf-8")

    assert '"map_projector_info_path": map_projector_info_path' in launch_text
    assert "carmaker_map_projector_info.yaml" in launch_text
    assert "projector_type: LocalCartesian" in projector_text
    assert "vertical_datum: WGS84" in projector_text
    assert "latitude: 29.05466832" in projector_text
    assert "longitude: 110.47991599" in projector_text
    assert "projector_type: Local\n" not in projector_text


def test_bringup_declares_migrated_runtime_dependencies():
    text = PACKAGE.read_text(encoding="utf-8")

    for package in [
        "autoware_crop_box_filter",
        "autoware_downsample_filters",
        "autoware_ekf_localizer",
        "autoware_gyro_odometer",
        "autoware_localization_error_monitor",
        "autoware_map_height_fitter",
        "autoware_pose_initializer",
        "autoware_pose_instability_detector",
        "autoware_vehicle_velocity_converter",
    ]:
        assert f"<exec_depend>{package}</exec_depend>" in text

    assert "<exec_depend>autoware_pose_covariance_modifier</exec_depend>" not in text
