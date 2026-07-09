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
        "/sensing/lidar/filtered/pointcloud",
        "/sensing/imu/imu_data_raw",
        "/sensing/imu/imu_data",
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
    assert "/sensing/lidar/filtered/pointcloud" in check_text
    assert "/sensing/imu/imu_data" in check_text

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
    assert "rc_start_sensors.sh" in capture_text
    assert "kill -INT" in capture_text
    assert "ros2 bag info" in capture_text
    assert capture_text.count("scripts/ros_env.sh") >= 1
    assert "rc_start_mapping_bag.sh" in capture_text
    assert "run_track.sh" not in capture_text

    start_bag_text = (rc_dir / "rc_start_mapping_bag.sh").read_text()
    stop_bag_text = (rc_dir / "rc_stop_mapping_bag.sh").read_text()
    assert "check_mapping_inputs.sh" in start_bag_text
    assert "record_mapping_bag.sh" in start_bag_text
    assert "rc_start_sensors.sh" in start_bag_text
    assert "mapping_bag.env" in start_bag_text
    assert "run_track.sh" not in start_bag_text
    assert "kill -INT" in stop_bag_text
    assert "ros2 bag info" in stop_bag_text

    sensor_start_text = (rc_dir / "rc_start_sensors.sh").read_text()
    assert "run_official_autoware.sh" in sensor_start_text
    for setting in (
        "LAUNCH_MAP=false",
        "LAUNCH_LOCALIZATION=false",
        "LAUNCH_PLANNING=false",
        "LAUNCH_CONTROL=false",
        "LAUNCH_API=false",
        "LAUNCH_VEHICLE_INTERFACE=false",
    ):
        assert setting in sensor_start_text

    localization_start_text = (rc_dir / "rc_start_localization.sh").read_text()
    assert "run_official_autoware.sh" in localization_start_text
    assert "LAUNCH_PLANNING=false" in localization_start_text
    assert "LAUNCH_CONTROL=false" in localization_start_text
    assert "LAUNCH_API=false" in localization_start_text
    assert "LAUNCH_VEHICLE_INTERFACE=false" in localization_start_text
    stop_text = (rc_dir / "rc_stop.sh").read_text()
    assert "pkill" in stop_text
    assert "component_container" in stop_text
    assert "topic_tools/relay" in stop_text


def test_official_sensor_kit_exposes_hipnuc_imu_arguments():
    sensing_launch = (
        ROOT
        / "src"
        / "autoracer_rc_sensor_kit_launch"
        / "launch"
        / "sensing.launch.xml"
    )
    setup_py = ROOT / "src" / "autoracer_sensing" / "setup.py"

    sensing_text = sensing_launch.read_text()
    assert "launch_imu" in sensing_text
    assert "hipnuc_imu" in sensing_text
    assert "imu_filter_madgwick" in sensing_text
    assert "/sensing/imu/imu_data_raw" in sensing_text
    assert "/sensing/imu/imu_data" in sensing_text
    assert "pointcloud_voxel_filter" in sensing_text
    assert "pointcloud_voxel_filter" in setup_py.read_text()


def test_official_branch_removes_legacy_track_entrypoints():
    removed_entrypoints = [
        ROOT / "scripts" / "run_track.sh",
        ROOT / "src" / "autoracer_bringup",
        ROOT / "src" / "autoracer_bringup" / "launch" / "track.launch.py",
        ROOT / "src" / "autoracer_bringup" / "launch" / "track_rc_p0.launch.py",
    ]
    for path in removed_entrypoints:
        assert not path.exists(), f"legacy formal entrypoint should be removed: {path}"

    run_official_text = (ROOT / "scripts" / "run_official_autoware.sh").read_text()
    for setting in (
        "LAUNCH_VEHICLE",
        "LAUNCH_SENSING",
        "LAUNCH_LOCALIZATION",
        "LAUNCH_PLANNING",
        "LAUNCH_CONTROL",
        "LAUNCH_API",
        "LAUNCH_VEHICLE_INTERFACE",
    ):
        assert setting in run_official_text
    assert "ros2 launch autoware_launch autoware.launch.xml" in run_official_text


def test_rc_serial_defaults_match_current_orin_without_guessing_chassis_port():
    defaults_text = (ROOT / "defaults.env").read_text()
    run_script_text = (ROOT / "scripts" / "run_official_autoware.sh").read_text()
    assert 'SERIAL_PORT:=}' in defaults_text
    assert 'IMU_SERIAL_PORT:=/dev/ttyUSB0' in defaults_text
    assert "SERIAL_PORT is required when LAUNCH_VEHICLE_INTERFACE=true" in run_script_text

    operator_files = [
        ROOT / "README.md",
        ROOT / "docs" / "operations" / "rc_runbook_zh.md",
        ROOT / "src" / "autoracer_rc_launch" / "launch" / "vehicle_interface.launch.xml",
        ROOT / "scripts" / "run_official_autoware.sh",
    ]
    for path in operator_files:
        assert "/dev/ttyACM0" not in path.read_text(), path


def test_rc_autoware_rviz_exposes_runtime_navigation_tools():
    rviz_text = (
        ROOT / "src" / "autoracer_rc_launch" / "rviz" / "rc_autoware.rviz"
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
        ROOT / "src" / "autoracer_rc_launch" / "package.xml"
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


def test_localization_parameters_come_from_official_autoware_launch():
    ndt_param = (
        ROOT
        / "src"
        / "external"
        / "autoware"
        / "launcher"
        / "autoware_launch"
        / "config"
        / "localization"
        / "ndt_scan_matcher"
        / "ndt_scan_matcher.param.yaml"
    )

    assert ndt_param.exists()
    ndt_text = ndt_param.read_text()
    assert 'base_frame: "base_link"' in ndt_text
    assert 'map_frame: "map"' in ndt_text
    assert "required_distance:" in ndt_text
