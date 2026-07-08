from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_vehicle_mapping_scripts_exist_and_record_required_topics():
    record_script = ROOT / "scripts" / "record_mapping_bag.sh"
    check_script = ROOT / "scripts" / "check_mapping_inputs.sh"
    pull_script = ROOT / "scripts" / "pull_mapping_bag.sh"

    for script in (record_script, check_script, pull_script):
        assert script.exists(), f"missing mapping script: {script}"
        assert script.stat().st_mode & 0o111, f"script is not executable: {script}"

    record_text = record_script.read_text()
    for topic in (
        "/sensing/lidar/concatenated/pointcloud",
        "/imu/data_raw",
        "/imu/data",
        "/tf",
        "/tf_static",
        "/rosout",
    ):
        assert topic in record_text
    assert "--include-unpublished-topics" in record_text
    assert "--polling-interval" in record_text
    assert "-e" in record_text

    check_text = check_script.read_text()
    assert "ring" in check_text
    assert "time" in check_text
    assert "/imu/data" in check_text

    rc_dir = ROOT / "scripts" / "rc"
    public_scripts = {
        "rc_configure_lidar.sh",
        "rc_start_sensors.sh",
        "rc_start_mapping_bag.sh",
        "rc_capture_mapping_bag.sh",
        "rc_stop_mapping_bag.sh",
        "rc_start_localization.sh",
        "rc_start_autoware.sh",
        "rc_stop.sh",
    }
    assert {path.name for path in rc_dir.glob("*.sh")} == public_scripts
    for script_name in public_scripts:
        assert (rc_dir / script_name).stat().st_mode & 0o111

    capture_text = (rc_dir / "rc_capture_mapping_bag.sh").read_text()
    assert "check_mapping_inputs.sh" in capture_text
    assert "record_mapping_bag.sh" in capture_text
    assert "kill -INT" in capture_text
    assert "ros2 bag info" in capture_text
    assert capture_text.count("scripts/ros_env.sh") >= 1
    assert "rc_start_mapping_bag.sh" in capture_text

    start_bag_text = (rc_dir / "rc_start_mapping_bag.sh").read_text()
    stop_bag_text = (rc_dir / "rc_stop_mapping_bag.sh").read_text()
    assert "check_mapping_inputs.sh" in start_bag_text
    assert "record_mapping_bag.sh" in start_bag_text
    assert "mapping_bag.env" in start_bag_text
    assert "kill -INT" in stop_bag_text
    assert "ros2 bag info" in stop_bag_text

    assert "LAUNCH_LOCALIZATION=false" in (rc_dir / "rc_start_sensors.sh").read_text()
    assert "LAUNCH_MAP_PROJECTION_LOADER" in (rc_dir / "rc_start_localization.sh").read_text()
    assert "pkill" in (rc_dir / "rc_stop.sh").read_text()


def test_rc_bringup_exposes_hipnuc_imu_arguments():
    track_launch = ROOT / "src" / "autoracer_bringup" / "launch" / "track_rc_p0.launch.py"
    sensing_launch = ROOT / "src" / "autoracer_bringup" / "launch" / "sensing.launch.py"
    setup_py = ROOT / "src" / "autoracer_sensing" / "setup.py"

    assert "launch_imu" in track_launch.read_text()
    sensing_text = sensing_launch.read_text()
    assert "hipnuc_imu" in sensing_text
    assert "imu_filter_madgwick" in sensing_text
    assert "/imu/data_raw" in sensing_text
    assert "/imu/data" in sensing_text
    assert "pointcloud_voxel_filter" in sensing_text
    assert "pointcloud_voxel_filter" in setup_py.read_text()


def test_track_launch_allows_subsystem_isolation_for_vehicle_debug():
    track_launch = ROOT / "src" / "autoracer_bringup" / "launch" / "track.launch.py"
    rc_launch = ROOT / "src" / "autoracer_bringup" / "launch" / "track_rc_p0.launch.py"
    localization_launch = ROOT / "src" / "autoracer_bringup" / "launch" / "localization.launch.py"
    run_script = ROOT / "scripts" / "run_track.sh"

    track_text = track_launch.read_text()
    for argument in ("launch_planning", "launch_control", "launch_safety"):
        assert f'DeclareLaunchArgument("{argument}"' in track_text
        assert f"IfCondition({argument})" in track_text
        assert argument in rc_launch.read_text()
        assert argument.upper() in run_script.read_text()

    assert "launch_map_projection_loader" in localization_launch.read_text()
    assert "launch_map_projection_loader" in track_text
    rc_text = rc_launch.read_text()
    assert "launch_map_projection_loader" in rc_text
    assert '"manual_seed_require_input_pose": manual_seed_require_input_pose' in rc_text
    assert '"manual_seed_require_input_pose": "true"' not in rc_text
    assert '"ndt_param_file": ndt_param_file' in track_text
    assert '"ndt_param_file": ndt_param_file' in rc_text
    assert '"ndt_initial_pose_stamp_offset_sec": ndt_initial_pose_stamp_offset_sec' in track_text
    assert '"ndt_initial_pose_stamp_offset_sec": ndt_initial_pose_stamp_offset_sec' in rc_text
    assert '"input_pointcloud": localization_pointcloud_topic' in track_text
    assert "launch_pointcloud_filter" in track_text
    assert 'DeclareLaunchArgument("launch_pointcloud_filter", default_value="true")' in rc_text
    assert 'default_value="/sensing/lidar/filtered/pointcloud"' in rc_text
    assert "config\", \"rc\", \"ndt_scan_matcher.param.yaml" in rc_text
    assert 'DeclareLaunchArgument("ndt_initial_pose_stamp_offset_sec", default_value="-0.10")' in rc_text
    assert "rc_autoware.rviz" in track_text
    assert "rc_autoware.rviz" in rc_text
    assert "rc_sensor_extrinsics.yaml" in track_text
    assert 'DeclareLaunchArgument("launch_fixposition", default_value="false")' in track_text
    assert 'DeclareLaunchArgument("launch_manual_seed", default_value="true")' in track_text
    assert 'default_value="/sensing/lidar/filtered/pointcloud"' in track_text
    assert "config\", \"rc\", \"lslidar_cx.yaml" in track_text
    assert "config\", \"rc\", \"ndt_scan_matcher.param.yaml" in track_text
    localization_text = localization_launch.read_text()
    assert 'DeclareLaunchArgument("wheel_base_m", default_value="0.6")' in localization_text
    assert 'DeclareLaunchArgument("launch_fixposition_seed", default_value="false")' in localization_text
    assert 'DeclareLaunchArgument("launch_manual_seed", default_value="true")' in localization_text
    assert 'default_value="/sensing/lidar/filtered/pointcloud"' in localization_text
    assert "config\",\n            \"rc\"" in localization_text
    assert "NDT_PARAM_FILE" in run_script.read_text()
    assert "NDT_INITIAL_POSE_STAMP_OFFSET_SEC" in run_script.read_text()
    assert "LAUNCH_POINTCLOUD_FILTER" in run_script.read_text()
    assert "LOCALIZATION_POINTCLOUD_TOPIC" in run_script.read_text()
    assert "LAUNCH_MAP_PROJECTION_LOADER" in run_script.read_text()
    assert "LAUNCH_IMU" in run_script.read_text()
    assert "IMU_SERIAL_PORT" in run_script.read_text()
    assert "IMU_BAUDRATE" in run_script.read_text()


def test_rc_serial_defaults_match_current_orin_without_guessing_chassis_port():
    defaults_text = (ROOT / "defaults.env").read_text()
    run_script_text = (ROOT / "scripts" / "run_track.sh").read_text()
    assert 'SERIAL_PORT:=}' in defaults_text
    assert 'IMU_SERIAL_PORT:=/dev/ttyUSB0' in defaults_text
    assert 'if [[ -n "${SERIAL_PORT:-}" ]]' in run_script_text
    assert 'LAUNCH_ARGS+=(serial_port:="${SERIAL_PORT}")' in run_script_text

    operator_files = [
        ROOT / "README.md",
        ROOT / "docs" / "operations" / "rc_runbook_zh.md",
        ROOT / "src" / "autoracer_bringup" / "launch" / "track.launch.py",
        ROOT / "src" / "autoracer_bringup" / "launch" / "track_rc_p0.launch.py",
        ROOT / "src" / "autoracer_bringup" / "launch" / "vehicle.launch.py",
        ROOT / "src" / "autoracer_bringup" / "launch" / "bench_verification.launch.py",
    ]
    for path in operator_files:
        assert "/dev/ttyACM0" not in path.read_text(), path


def test_rc_autoware_rviz_exposes_runtime_navigation_tools():
    rviz_text = (
        ROOT / "src" / "autoracer_bringup" / "rviz" / "rc_autoware.rviz"
    ).read_text()

    for topic in (
        "/map/pointcloud_map",
        "/sensing/lidar/concatenated/pointcloud",
        "/sensing/lidar/filtered/pointcloud",
        "/points_aligned",
        "/localization/pose",
        "/localization/kinematic_state",
        "/vehicle/status/steering_status",
        "/vehicle/status/velocity_status",
        "/planning/mission_path",
        "/planning/route_marker",
        "/initialpose",
        "/goal_pose",
    ):
        assert topic in rviz_text

    assert "rviz_default_plugins/SetInitialPose" in rviz_text
    assert "rviz_default_plugins/SetGoal" in rviz_text
    assert "rviz_default_plugins/Odometry" in rviz_text
    assert "rviz_plugins/ControlModeDisplay" in rviz_text
    assert "rviz_plugins::PoseHistory" in rviz_text
    assert "rviz_plugins/SteeringAngle" in rviz_text
    assert "rviz_plugins/VelocityHistory" in rviz_text


def test_mock_lidar_diagnostics_are_not_part_of_rc_flow():
    removed_paths = [
        ROOT / "src" / "autoracer_bringup" / "launch" / "mock_lidar_ndt.launch.py",
        ROOT / "src" / "autoracer_bringup" / "launch" / "mock_lidar_record_scenario.launch.py",
        ROOT / "src" / "autoracer_bringup" / "rviz" / "mock_lidar_ndt.rviz",
        ROOT / "src" / "autoracer_bringup" / "rviz" / "mock_lidar_record.rviz",
        ROOT / "src" / "autoracer_sensing" / "autoracer_sensing" / "mock_lidar_tools.py",
        ROOT / "src" / "autoracer_sensing" / "test" / "test_mock_lidar_tools.py",
    ]
    for path in removed_paths:
        assert not path.exists(), f"mock diagnostic file should stay removed: {path}"

    setup_text = (ROOT / "src" / "autoracer_sensing" / "setup.py").read_text()
    defaults_text = (ROOT / "defaults.env").read_text()
    assert "mock_lidar" not in setup_text
    assert "MOCK_LIDAR" not in defaults_text


def test_official_autoware_rviz_plugins_are_declared():
    repos_text = (ROOT / "autoracer.repos").read_text()
    import_script = (ROOT / "scripts" / "import_dependencies.sh").read_text()
    build_minimal = (ROOT / "scripts" / "build_minimal.sh").read_text()
    build_bench = (ROOT / "scripts" / "build_bench.sh").read_text()
    package_xml = (
        ROOT / "src" / "autoracer_bringup" / "package.xml"
    ).read_text()

    assert "autoware_rviz_plugins.git" in repos_text
    assert "src/autoware/autoware_rviz_plugins" in import_script
    assert "src/external/autoware/autoware_rviz_plugins" in import_script
    autoware_rviz_packages = (
        "autoware_localization_rviz_plugin",
        "autoware_planning_rviz_plugin",
    )
    tier4_rviz_packages = (
        "tier4_control_mode_rviz_plugin",
        "tier4_state_rviz_plugin",
        "tier4_vehicle_rviz_plugin",
        "tier4_planning_factor_rviz_plugin",
    )

    for package in autoware_rviz_packages + tier4_rviz_packages:
        assert package in build_minimal
        assert package in build_bench
        assert package in package_xml

    for package in tier4_rviz_packages:
        assert package in import_script


def test_rc_ndt_parameters_fit_c32_short_range_smoke_test():
    rc_ndt = ROOT / "src" / "autoracer_bringup" / "config" / "rc" / "ndt_scan_matcher.param.yaml"
    hooke_ndt = (
        ROOT / "src" / "autoracer_bringup" / "config" / "hooke2" / "ndt_scan_matcher.param.yaml"
    )

    assert rc_ndt.exists()
    rc_text = rc_ndt.read_text()
    assert 'base_frame: "base_link"' in rc_text
    assert 'map_frame: "map"' in rc_text
    assert "required_distance: 2.0" in rc_text
    assert "required_distance: 10.0" in hooke_ndt.read_text()
