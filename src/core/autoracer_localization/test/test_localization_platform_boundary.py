from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PRODUCT_ROOT = Path(__file__).resolve().parents[4]
LAUNCH_SOURCE = (PACKAGE_ROOT / "launch" / "localization.launch.py").read_text(
    encoding="utf-8"
)
PACKAGE_SOURCES = "\n".join(
    path.read_text(encoding="utf-8")
    for path in PACKAGE_ROOT.rglob("*")
    if path.suffix in {".py", ".xml", ".yaml", ".yml"}
    and "test" not in path.parts
)
MODIFIER_CONFIG = (
    PACKAGE_ROOT
    / "config"
    / "pilot_compatible"
    / "pose_covariance_modifier.param.yaml"
).read_text(encoding="utf-8")
EKF_CONFIG = (
    PACKAGE_ROOT
    / "config"
    / "pilot_compatible"
    / "ekf_localizer.param.yaml"
).read_text(encoding="utf-8")


def test_localization_consumes_only_normalized_sensor_contracts():
    for topic in (
        "/sensing/lidar/concatenated/pointcloud",
        "/sensing/imu/imu_data",
        "/vehicle/status/velocity_status",
        "/sensing/vehicle_velocity_converter/twist_with_covariance",
    ):
        assert topic in PACKAGE_SOURCES


def test_localization_launch_does_not_own_platform_normalization():
    for token in (
        "autoracer_description",
        "autoware_gnss_poser",
        "fixposition",
        "topic_tools",
    ):
        assert token not in LAUNCH_SOURCE


def test_tianmen_launch_uses_one_existing_modifier_and_one_ekf():
    assert '"pose_source": "ndt"' in LAUNCH_SOURCE
    assert '"twist_source": "gyro_odom"' in LAUNCH_SOURCE
    assert '"use_autoware_pose_covariance_modifier"' in LAUNCH_SOURCE
    assert "pose_covariance_modifier_param_path" in LAUNCH_SOURCE
    assert "ekf_localizer_param_path" in LAUNCH_SOURCE
    assert "autoware_ekf_localizer" not in LAUNCH_SOURCE


def test_tianmen_launch_keeps_unqualified_continuous_gnss_fusion_opt_in():
    fusion_default = LAUNCH_SOURCE.split(
        '"CM_LOCALIZATION_USE_GNSS_FUSION"', 1
    )[1].split(")", 1)[0]
    assert 'default_value="false"' in fusion_default


def test_tianmen_modifier_policy_keeps_ndt_primary_and_gnss_position_only():
    assert "fusion_policy: ndt_primary_position" in MODIFIER_CONFIG
    assert "gnss_pose_timeout_sec: 1.0" in MODIFIER_CONFIG
    assert "gnss_position_only_yaw_variance: 1.0" in MODIFIER_CONFIG
    assert "gnss_ndt_max_stamp_delta_sec: 0.0" in MODIFIER_CONFIG
    assert "gnss_ndt_xy_innovation_max_m: 0.0" in MODIFIER_CONFIG


def test_tianmen_ekf_uses_standard_correlated_position_update():
    assert "enable_position_only_state_mask: true" in EKF_CONFIG
    assert "position_only_update_correlated_states: true" in EKF_CONFIG
    assert "position_only_update_velocity_state: false" in EKF_CONFIG
    assert "enable_gnss_bias_estimation: false" in EKF_CONFIG
    assert "enable_position_only_dual_projection: false" in EKF_CONFIG
    assert "position_only_nis_gate: 13.815510557964274" in EKF_CONFIG


def test_modifier_parameter_override_is_explicit_and_default_stays_formal():
    assert "pose_covariance_modifier_param_path = LaunchConfiguration(" in LAUNCH_SOURCE
    assert '"pose_covariance_modifier_param_path"' in LAUNCH_SOURCE
    assert 'ekf_localizer_param_path = LaunchConfiguration(' in LAUNCH_SOURCE
    assert '"ekf_localizer_param_path"' in LAUNCH_SOURCE
    assert '"pose_covariance_modifier.param.yaml"' in LAUNCH_SOURCE
    assert '"CM_LOCALIZATION_MODIFIER_PARAM_PATH"' in LAUNCH_SOURCE
    assert '"CM_LOCALIZATION_EKF_PARAM_PATH"' in LAUNCH_SOURCE
    assert "ndt_scan_matcher_param_path = LaunchConfiguration(" in LAUNCH_SOURCE
    assert '"ndt_scan_matcher_param_path"' in LAUNCH_SOURCE
    assert '"ndt_scan_matcher.param.yaml"' in LAUNCH_SOURCE


def test_ndt_regularization_remains_disabled():
    ndt_config = (
        PACKAGE_ROOT
        / "config"
        / "pilot_compatible"
        / "ndt_scan_matcher"
        / "ndt_scan_matcher.param.yaml"
    ).read_text(encoding="utf-8")
    assert "regularization:" in ndt_config
    assert "enable: false" in ndt_config


def test_modifier_patch_is_part_of_the_dependency_lock():
    lock = (
        PRODUCT_ROOT / "dependencies" / "versions.lock.yaml"
    ).read_text(encoding="utf-8")

    patch_names = (
        "pose_covariance_modifier_ndt_primary.patch",
        "localization_gnss_initializer_switch.patch",
        "pose_covariance_modifier_robust_research_v2.patch",
        "ekf_standard_position_innovation_v85.patch",
        "pose_covariance_modifier_standard_loose_coupling_v86.patch",
        "pose_covariance_modifier_receiver_state_integrity_v108.patch",
    )
    positions = []
    for patch_name in patch_names:
        token = f"dependencies/patches/{patch_name}"
        assert token in lock
        assert (PRODUCT_ROOT / "dependencies" / "patches" / patch_name).is_file()
        positions.append(lock.index(token))
    assert positions == sorted(positions)


def test_dependency_importer_uses_declared_patch_order():
    importer = (PRODUCT_ROOT / "scripts" / "import_dependencies.sh").read_text(
        encoding="utf-8"
    )
    assert "declared_patches" in importer
    assert "VERSION_LOCK" in importer
    assert "recorded_patch_stack_prefix_length" in importer
    assert '"${declared_patch_files[@]:prefix_length}"' in importer
    assert "find \"${PATCH_DIR}\"" not in importer
