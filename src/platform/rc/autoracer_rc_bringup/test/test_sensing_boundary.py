from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RC_ROOT = PACKAGE_ROOT.parent
REPOSITORY_ROOT = next(
    parent for parent in Path(__file__).resolve().parents if (parent / "dependencies").is_dir()
)
SENSING_SOURCE = (PACKAGE_ROOT / "launch" / "sensing.launch.py").read_text(
    encoding="utf-8"
)
LIDAR_CONFIG_SOURCE = (PACKAGE_ROOT / "config" / "rc" / "lidar.param.yaml").read_text(
    encoding="utf-8"
)
IMU_CONFIG_SOURCE = (PACKAGE_ROOT / "config" / "rc" / "imu.param.yaml").read_text(
    encoding="utf-8"
)
ADAPTER_NODE_SOURCE = (
    RC_ROOT / "autoracer_rc_adapter" / "src" / "c32_pointcloud_adapter_node.cpp"
).read_text(encoding="utf-8")
IMU_ADAPTER_NODE_SOURCE = (
    RC_ROOT / "autoracer_rc_adapter" / "src" / "imu_qos_adapter_node.cpp"
).read_text(encoding="utf-8")
G90_ADAPTER_SOURCE = (
    RC_ROOT
    / "autoracer_rc_adapter"
    / "autoracer_rc_adapter"
    / "g90_adapter_node.py"
).read_text(encoding="utf-8")
G90_PARSER_SOURCE = (
    RC_ROOT / "autoracer_rc_adapter" / "autoracer_rc_adapter" / "nmea_gnss.py"
).read_text(encoding="utf-8")
G90_CONFIG_PATH = PACKAGE_ROOT / "config" / "rc" / "g90.param.yaml"
DEPENDENCY_ROOT = REPOSITORY_ROOT / "dependencies"
RC_REPOSITORIES_PATH = DEPENDENCY_ROOT / "autoracer-rc.repos"
FULL_PACKAGES_PATH = DEPENDENCY_ROOT / "vendor-packages.tsv"
RC_PACKAGES_PATH = DEPENDENCY_ROOT / "vendor-packages-rc.tsv"
LOCK_PATH = DEPENDENCY_ROOT / "versions.lock.yaml"
LIDAR_SHUTDOWN_PATCH_PATH = (
    DEPENDENCY_ROOT / "patches" / "lslidar_ros2_clean_shutdown.patch"
)
LAUNCHER_PATCH_PATH = (
    DEPENDENCY_ROOT / "patches" / "tier4_localization_launch_gnss_enabled.patch"
)
G90_UDEV_PATH = PACKAGE_ROOT / "udev" / "99-autoracer-rc-g90.rules"
IMPORT_SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "import_dependencies.sh"


def _yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_rc_sensing_terminates_at_platform_independent_topics():
    assert '"/sensing/lidar/concatenated/pointcloud"' in SENSING_SOURCE
    assert '"/sensing/imu/imu_data"' in SENSING_SOURCE
    assert '"/sensing/gnss/pose"' in SENSING_SOURCE
    assert '"/sensing/gnss/pose_with_covariance"' in SENSING_SOURCE
    assert '"/sensing/gnss/fixed"' in SENSING_SOURCE
    assert 'package="autoracer_rc_adapter"' in SENSING_SOURCE


def test_g90_uses_pinned_upstream_transport_without_vendor_example_workspace():
    repositories = (REPOSITORY_ROOT / "dependencies" / "autoracer.repos").read_text(
        encoding="utf-8"
    )
    packages = (REPOSITORY_ROOT / "dependencies" / "vendor-packages.tsv").read_text(
        encoding="utf-8"
    )
    assert "420cb44ec980f24216fffa2d491a72578c867efd" in repositories
    assert "6b1195aac34de64e88bcc476fd425900256095f1" in repositories
    assert "nmea_msgs\tvendor/nmea_msgs" in packages
    assert "nmea_navsat_driver\tvendor/nmea_navsat_driver" in packages
    assert "wheeltec_gps" not in repositories


def test_rc_dependency_profile_excludes_only_hooke2_fixposition_packages():
    full_packages = {
        line.split("\t", 1)[0]
        for line in FULL_PACKAGES_PATH.read_text(encoding="utf-8").splitlines()
        if line
    }
    rc_packages = {
        line.split("\t", 1)[0]
        for line in RC_PACKAGES_PATH.read_text(encoding="utf-8").splitlines()
        if line
    }
    excluded = {
        "fixposition_driver_lib",
        "fixposition_driver_msgs",
        "fixposition_driver_ros2",
        "fpsdk_common",
        "fpsdk_ros2",
        "rtcm_msgs",
    }
    assert len(full_packages) == 105
    assert len(rc_packages) == 99
    assert full_packages - rc_packages == excluded
    assert not rc_packages - full_packages

    rc_repositories = _yaml(RC_REPOSITORIES_PATH)["repositories"]
    assert "whale_components" not in rc_repositories
    launcher = rc_repositories["autoware/launcher"]
    assert launcher["url"] == (
        "https://github.com/autowarefoundation/autoware_launch.git"
    )
    assert launcher["version"] == "0f3946af9c5ec1cba491681b6927c6fcf577ed25"

    rc_profile = _yaml(LOCK_PATH)["profiles"]["rc"]
    assert rc_profile["package_count"] == 99
    assert set(rc_profile["excluded_packages"]) == excluded

    import_script = IMPORT_SCRIPT_PATH.read_text(encoding="utf-8")
    assert "--network-rc" in import_script
    assert "--verify-only-rc" in import_script
    assert 'vcs import --shallow "${temporary_checkout}/src"' in import_script


def test_locked_vendor_patches_preserve_launcher_and_clean_lidar_shutdown():
    lock = _yaml(LOCK_PATH)
    assert "dependencies/patches/lslidar_ros2_clean_shutdown.patch" in lock[
        "patches"
    ]
    assert (
        "dependencies/patches/tier4_localization_launch_gnss_enabled.patch"
        in lock["patches"]
    )

    lidar_patch = LIDAR_SHUTDOWN_PATCH_PATH.read_text(encoding="utf-8")
    for token in (
        "+        virtual ~LslidarDriver();",
        "+        return rclcpp::ok() ? 0 : 1;",
        "+            difop_thread_->join();",
        "+            poll_thread_->join();",
        "-    signal(SIGINT, my_handler);",
    ):
        assert token in lidar_patch

    launcher_patch = LAUNCHER_PATCH_PATH.read_text(encoding="utf-8")
    assert '+  <arg name="gnss_enabled" default="true"/>' in launcher_patch
    assert (
        '+      <arg name="gnss_enabled" value="$(var gnss_enabled)"/>'
        in launcher_patch
    )


def test_g90_udev_identity_is_unique_and_does_not_change_access_policy():
    rule = G90_UDEV_PATH.read_text(encoding="utf-8")
    for token in (
        'KERNEL=="ttyCH343USB*"',
        'ATTRS{idVendor}=="1a86"',
        'ATTRS{idProduct}=="55d4"',
        'ATTRS{serial}=="5AA6079369"',
        'SYMLINK+="serial/by-id/',
        "5AA6079369-if00",
    ):
        assert token in rule
    assert "MODE=" not in rule
    assert "GROUP=" not in rule


def test_g90_platform_boundary_matches_fixposition_normalized_outputs():
    for token in (
        'executable="g90_nmea_adapter"',
        'get_package_share_directory("autoware_gnss_poser")',
        '"/g90/raw/nmea_sentence"',
        '"/g90/fix"',
        '"/g90/autoware_orientation"',
        '"/sensing/gnss/pose"',
        '"/sensing/gnss/pose_with_covariance"',
        '"/sensing/gnss/fixed"',
    ):
        assert token in SENSING_SOURCE
    assert "/fixposition/" not in SENSING_SOURCE + G90_ADAPTER_SOURCE


def test_g90_defaults_fail_closed_until_4p_calibration_and_map_binding():
    params = _yaml(G90_CONFIG_PATH)["/g90/g90_nmea_adapter"]["ros__parameters"]
    assert params["frame_id"] == "gnss_link"
    assert params["base_frame"] == "base_link"
    assert params["enable_localization_output"] is False
    assert params["allow_statusless_hdt"] is False
    assert params["yaw_stddev_deg"] == 0.0
    for reason in (
        "localization_output_disabled",
        "yaw_covariance_unconfigured",
        "map_projector_local",
        "base_transform_missing",
    ):
        assert reason in G90_ADAPTER_SOURCE
    assert "covariance_missing" in G90_PARSER_SOURCE
    assert 'sentence_type == "THS"' in G90_PARSER_SOURCE
    assert "heading_status_unavailable" in G90_ADAPTER_SOURCE
    assert '"GB"' in G90_PARSER_SOURCE


def test_g90_launch_is_opt_in_and_requires_discovered_usb_identity():
    assert 'DeclareLaunchArgument("launch_g90", default_value="false")' in SENSING_SOURCE
    assert 'default_value=launch_g90' in SENSING_SOURCE
    assert '"g90_device",\n                default_value=""' in SENSING_SOURCE
    assert 'DeclareLaunchArgument("g90_baud", default_value="115200")' in SENSING_SOURCE
    assert 'executable="nmea_topic_serial_reader"' in SENSING_SOURCE
    assert "respawn=True" in SENSING_SOURCE
    for unstable_name in ("/dev/ttyUSB", "/dev/ttyACM", "/dev/wheeltec_gps"):
        assert unstable_name not in SENSING_SOURCE


def test_rc_sensing_is_sensor_only():
    forbidden = (
        "autoracer_bringup",
        "autoracer_control",
        "localization",
        "map_loader",
        "race.launch",
        "vehicle_cmd",
        "chassis",
    )
    for token in forbidden:
        assert token not in SENSING_SOURCE


def test_lidar_configuration_matches_confirmed_c32_connection_and_axis_mode():
    params = _yaml(PACKAGE_ROOT / "config" / "rc" / "lidar.param.yaml")[
        "/cx/lslidar_driver_node"
    ]["ros__parameters"]
    assert params["device_ip"] == "192.168.1.200"
    assert params["msop_port"] == 2368
    assert params["difop_port"] == 2369
    assert params["frame_id"] == "lidar_top"
    assert params["topic_name"] == "/sensing/lidar/raw/pointcloud"
    assert params["coordinate_opt"] is True
    assert params["pcl_type"] is False


def test_c32_raw_frames_are_reliable_until_normalization():
    assert "raw_qos.reliable().durability_volatile()" in ADAPTER_NODE_SOURCE
    assert '"output", rclcpp::SensorDataQoS()' in ADAPTER_NODE_SOURCE


def test_imu_uses_stable_overridable_usb_identity():
    assert "/dev/serial/by-id/" in SENSING_SOURCE
    assert "0003-if00-port0" in SENSING_SOURCE
    assert 'LaunchConfiguration("imu_device")' in SENSING_SOURCE
    assert "serial_port" not in IMU_CONFIG_SOURCE
    for unstable_name in ("/dev/ttyUSB", "/dev/ttyCH343", "/dev/wheeltec_"):
        assert unstable_name not in SENSING_SOURCE


def test_imu_publishes_native_measurement_without_refusion():
    params = _yaml(PACKAGE_ROOT / "config" / "rc" / "imu.param.yaml")[
        "IMU_publisher"
    ]["ros__parameters"]
    assert params["baud_rate"] == 115200
    assert params["frame_id"] == "imu_link"
    assert params["imu_topic"] == "/sensing/imu/raw/imu_data"
    assert params["imu_switch"] is True
    assert all(
        params[name] is False
        for name in (
            "euler_switch",
            "magnetic_switch",
            "temperature_switch",
            "pressure_switch",
        )
    )
    assert "madgwick" not in (
        SENSING_SOURCE + IMU_CONFIG_SOURCE + IMU_ADAPTER_NODE_SOURCE
    ).lower()


def test_imu_qos_is_normalized_without_changing_message_contents():
    assert '("input", "/sensing/imu/raw/imu_data")' in SENSING_SOURCE
    assert '("output", "/sensing/imu/imu_data")' in SENSING_SOURCE
    assert "input_qos.best_effort().durability_volatile()" in IMU_ADAPTER_NODE_SOURCE
    assert "output_qos.reliable().durability_volatile()" in IMU_ADAPTER_NODE_SOURCE
    assert "publisher_->publish(*message)" in IMU_ADAPTER_NODE_SOURCE


def test_static_sensor_transforms_are_unique_and_match_confirmed_measurements():
    transforms = _yaml(
        RC_ROOT / "autoracer_rc_description" / "config" / "sensor_extrinsics.yaml"
    )["transforms"]
    assert len(transforms) == 2
    assert len({item["child"] for item in transforms}) == 2
    by_child = {item["child"]: item for item in transforms}

    assert by_child["lidar_top"]["parent"] == "base_link"
    assert by_child["lidar_top"]["translation"] == {
        "x": 0.280,
        "y": 0.0,
        "z": 0.3465,
    }
    assert by_child["imu_link"]["parent"] == "base_link"
    assert by_child["imu_link"]["translation"] == {
        "x": 0.200,
        "y": 0.0,
        "z": 0.240,
    }
    for transform in transforms:
        assert transform["rotation_rpy"] == {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
