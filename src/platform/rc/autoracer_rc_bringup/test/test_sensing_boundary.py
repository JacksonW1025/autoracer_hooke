import subprocess
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
RACE_LAUNCH_SOURCE = (PACKAGE_ROOT / "launch" / "race.launch.py").read_text(
    encoding="utf-8"
)
BRINGUP_PACKAGE_SOURCE = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
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
G90_NTRIP_RELAY_SOURCE = (
    RC_ROOT
    / "autoracer_rc_adapter"
    / "autoracer_rc_adapter"
    / "g90_ntrip_relay_node.py"
).read_text(encoding="utf-8")
ADAPTER_PACKAGE_SOURCE = (
    RC_ROOT / "autoracer_rc_adapter" / "package.xml"
).read_text(encoding="utf-8")
ADAPTER_CMAKE_SOURCE = (
    RC_ROOT / "autoracer_rc_adapter" / "CMakeLists.txt"
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
NMEA_RUNTIME_PATCH_PATH = (
    DEPENDENCY_ROOT / "patches" / "nmea_navsat_driver_tf_transformations.patch"
)
LAUNCHER_PATCH_PATH = (
    DEPENDENCY_ROOT / "patches" / "tier4_localization_launch_gnss_enabled.patch"
)
G90_UDEV_PATH = PACKAGE_ROOT / "udev" / "99-autoracer-rc-g90.rules"
IMPORT_SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "import_dependencies.sh"
BUILD_SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "build.sh"
BUILD_VENDOR_SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "build_vendor.sh"
BUILD_PRODUCT_SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "build_product.sh"
INSTALL_ROSDEPS_SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "install_rosdeps.sh"
RC_ENTRY_PATH = REPOSITORY_ROOT / "scripts" / "autoracer_rc.sh"
RC_ENTRY_SOURCE = RC_ENTRY_PATH.read_text(encoding="utf-8")
RUNTIME_STATE_WATCH_PATH = (
    REPOSITORY_ROOT / "scripts" / "autoracer_rc_runtime_state_watch.py"
)
RUNTIME_STATE_WATCH_SOURCE = RUNTIME_STATE_WATCH_PATH.read_text(encoding="utf-8")
MAPPING_RECORDING_PATH = REPOSITORY_ROOT / "scripts" / "autoracer_rc_recording.sh"
MAPPING_RECORDING_SOURCE = MAPPING_RECORDING_PATH.read_text(encoding="utf-8")
MAPPING_STOP_PATH = REPOSITORY_ROOT / "scripts" / "autoracer_rc_recording_stop.sh"
MAPPING_STOP_SOURCE = MAPPING_STOP_PATH.read_text(encoding="utf-8")
MAPPING_PREFLIGHT_PATH = (
    REPOSITORY_ROOT / "scripts" / "autoracer_rc_recording_preflight.py"
)
MAPPING_PREFLIGHT_SOURCE = MAPPING_PREFLIGHT_PATH.read_text(encoding="utf-8")
MAPPING_QOS_PATH = PACKAGE_ROOT / "config" / "rc" / "mapping_recording_qos.yaml"
DRIVING_PROFILES_PATH = PACKAGE_ROOT / "config" / "rc" / "driving_profiles.yaml"


def _yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _bash_function(source, name, following_name):
    start = source.index(f"{name}()")
    end = source.index(f"{following_name}()", start)
    return source[start:end]


def _shell_fit(function_source, function_name, text, columns):
    result = subprocess.run(
        [
            "bash",
            "-c",
            function_source + f'\n{function_name} "$1" "$2"',
            "bash",
            text,
            str(columns),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def _dashboard_width(text):
    return sum(1 if ord(character) <= 127 else 2 for character in text)


def test_rc_sensing_terminates_at_platform_independent_topics():
    assert '"/sensing/lidar/concatenated/pointcloud"' in SENSING_SOURCE
    assert '"/sensing/imu/imu_data"' in SENSING_SOURCE
    assert '"/sensing/gnss/pose"' in SENSING_SOURCE
    assert '"/sensing/gnss/pose_with_covariance"' in SENSING_SOURCE
    assert '"/sensing/gnss/fixed"' in SENSING_SOURCE
    assert 'package="autoracer_rc_adapter"' in SENSING_SOURCE


def test_rc_entry_uses_distro_managed_python_environment():
    python_isolation = "export PYTHONNOUSERSITE=1"
    ros_environment = 'source "${PRODUCT_ROOT}/scripts/ros_env.sh"'
    assert python_isolation in RC_ENTRY_SOURCE
    assert RC_ENTRY_SOURCE.index(python_isolation) < RC_ENTRY_SOURCE.index(
        ros_environment
    )


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


def test_rc_dependency_profile_excludes_hooke2_and_unused_rc_packages():
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
    hooke2_only = {
        "fixposition_driver_lib",
        "fixposition_driver_msgs",
        "fixposition_driver_ros2",
        "fpsdk_common",
        "fpsdk_ros2",
        "rtcm_msgs",
    }
    rc_unused = {
        "nebula_core_common",
        "nebula_core_decoders",
        "nebula_core_hw_interfaces",
        "nebula_core_ros",
        "nebula_hesai",
        "nebula_hesai_common",
        "nebula_hesai_decoders",
        "nebula_hesai_hw_interfaces",
        "nebula_msgs",
        "pandar_msgs",
        "sync_tooling_msgs",
        "tier4_api_msgs",
        "tier4_debug_msgs",
    }
    excluded = hooke2_only | rc_unused
    assert len(full_packages) == 105
    assert len(rc_packages) == 86
    assert full_packages - rc_packages == excluded
    assert not rc_packages - full_packages

    rc_repositories = _yaml(RC_REPOSITORIES_PATH)["repositories"]
    assert "whale_components" not in rc_repositories
    assert "vendor/nebula" not in rc_repositories
    assert "vendor/sync_tooling_msgs" not in rc_repositories
    launcher = rc_repositories["autoware/launcher"]
    assert launcher["url"] == (
        "https://github.com/autowarefoundation/autoware_launch.git"
    )
    assert launcher["version"] == "0f3946af9c5ec1cba491681b6927c6fcf577ed25"

    rc_profile = _yaml(LOCK_PATH)["profiles"]["rc"]
    assert rc_profile["package_count"] == 86
    assert set(rc_profile["excluded_packages"]) == excluded

    import_script = IMPORT_SCRIPT_PATH.read_text(encoding="utf-8")
    assert "--network-rc" in import_script
    assert "--verify-only-rc" in import_script
    assert 'vcs import --shallow "${temporary_checkout}/src"' in import_script

    build_script = BUILD_SCRIPT_PATH.read_text(encoding="utf-8")
    build_vendor_script = BUILD_VENDOR_SCRIPT_PATH.read_text(encoding="utf-8")
    build_product_script = BUILD_PRODUCT_SCRIPT_PATH.read_text(encoding="utf-8")
    install_rosdeps_script = INSTALL_ROSDEPS_SCRIPT_PATH.read_text(encoding="utf-8")
    assert '"${PRODUCT_ROOT}/scripts/import_dependencies.sh" --verify-only-rc' in (
        build_script
    )
    assert '"${PRODUCT_ROOT}/scripts/build_vendor.sh" --rc' in build_script
    assert '"${PRODUCT_ROOT}/scripts/build_product.sh" --rc' in build_script
    assert "vendor-packages-rc.tsv" in build_vendor_script
    assert "--base-paths src/core src/platform/rc" in build_product_script
    assert "export PYTHONNOUSERSITE=1" in build_vendor_script
    assert "export PYTHONNOUSERSITE=1" in build_product_script
    assert "autoracer_hooke2_bringup" not in build_product_script.split(
        'if [[ "${profile}" == "rc" ]]', 1
    )[1].split("else", 1)[0]
    assert '"${ROOT_DIR}/src/platform/rc"' in install_rosdeps_script
    assert "rosdep db >/dev/null" in install_rosdeps_script
    assert "autoware_ar_tag_based_localizer" in install_rosdeps_script
    assert "eagleye_rt" in install_rosdeps_script
    assert "yabloc_common" in install_rosdeps_script
    assert "yabloc_particle_filter" in install_rosdeps_script
    assert "--dependency-types test" in install_rosdeps_script


def test_locked_vendor_patches_preserve_launcher_and_clean_lidar_shutdown():
    lock = _yaml(LOCK_PATH)
    assert "dependencies/patches/lslidar_ros2_clean_shutdown.patch" in lock[
        "patches"
    ]
    assert (
        "dependencies/patches/tier4_localization_launch_gnss_enabled.patch"
        in lock["patches"]
    )
    assert (
        "dependencies/patches/nmea_navsat_driver_tf_transformations.patch"
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

    nmea_runtime_patch = NMEA_RUNTIME_PATCH_PATH.read_text(encoding="utf-8")
    assert "-    <exec_depend>python3-transforms3d</exec_depend>" in (
        nmea_runtime_patch
    )
    assert "+    <exec_depend>tf_transformations</exec_depend>" in (
        nmea_runtime_patch
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
    for token in (
        'KERNEL=="ttyUSB*"',
        'ATTRS{idProduct}=="7523"',
        'SYMLINK+="autoracer_g90_com2"',
    ):
        assert token in rule
    assert "ID_PATH" not in rule
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


def test_g90_formal_pose_uses_the_published_map_heading_contract():
    params = _yaml(G90_CONFIG_PATH)["/g90/g90_nmea_adapter"]["ros__parameters"]
    assert params["frame_id"] == "gnss_link"
    assert params["base_frame"] == "base_link"
    assert params["enable_localization_output"] is True
    assert params["allow_statusless_hdt"] is False
    assert params["heading_mount_offset_deg"] == 90.0
    assert params["yaw_stddev_deg"] == 10.0
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
    assert "/dev/serial/by-id" in SENSING_SOURCE
    assert "/dev/autoracer_rc_g90" not in SENSING_SOURCE
    assert 'DeclareLaunchArgument("g90_baud", default_value="115200")' in SENSING_SOURCE
    assert 'executable="nmea_topic_serial_reader"' in SENSING_SOURCE
    assert "respawn=" not in SENSING_SOURCE
    for unstable_name in (
        "/dev/ttyUSB",
        "/dev/ttyACM",
        "/dev/ttyCH343",
        "/dev/wheeltec_gps",
    ):
        assert unstable_name not in SENSING_SOURCE


def test_g90_corrections_are_project_owned_private_and_launch_scoped():
    for token in (
        'executable="g90_ntrip_relay"',
        'DeclareLaunchArgument(\n                "launch_g90_corrections"',
        'default_value="/dev/autoracer_g90_com2"',
        '"config_file": ParameterValue(',
        'condition=IfCondition(launch_g90_corrections)',
    ):
        assert token in SENSING_SOURCE
    assert 'default_value="false"' in SENSING_SOURCE
    assert '"launch_g90_corrections": "true"' in RACE_LAUNCH_SOURCE
    assert '"g90_ntrip_config_file": g90_ntrip_config_file' in RACE_LAUNCH_SOURCE

    assert "<exec_depend>ntrip_client</exec_depend>" in ADAPTER_PACKAGE_SOURCE
    assert "<exec_depend>python3-serial</exec_depend>" in ADAPTER_PACKAGE_SOURCE
    assert "scripts/g90_ntrip_relay" in ADAPTER_CMAKE_SOURCE
    assert "test/test_g90_ntrip_relay_node.py" in ADAPTER_CMAKE_SOURCE

    for token in (
        "mode must be exactly 0600",
        "NTRIP credentials are expired",
        "G90-COM2",
        "rtcm_fresh",
        "serial_open",
        "caster_connected",
        "client.send_nmea(sentence)",
        "self._serial.write(raw_packet)",
    ):
        assert token in G90_NTRIP_RELAY_SOURCE
    for forbidden in (
        'declare_parameter("username"',
        'declare_parameter("password"',
        "subprocess",
        "str2str",
        "systemctl",
    ):
        assert forbidden not in G90_NTRIP_RELAY_SOURCE

    for token in (
        'G90_COM2_DEVICE="${RC_G90_COM2_DEVICE:-/dev/autoracer_g90_com2}"',
        "validate_g90_correction_inputs",
        "read_g90_correction_snapshot",
        'ros2 topic echo --no-daemon "/diagnostics"',
        "level = ord(level)",
        "observe_g90_corrections false",
        'launch_g90_corrections:="${launch_g90_corrections}"',
        'g90_ntrip_config_file:="${G90_NTRIP_CONFIG_FILE}"',
    ):
        assert token in RC_ENTRY_SOURCE
    assert "systemctl" not in RC_ENTRY_SOURCE
    assert "str2str" not in RC_ENTRY_SOURCE


def test_single_rc_entry_uses_real_devices_and_owned_process_cleanup():
    for token in (
        "test_lidar",
        "test_imu",
        "test_g90",
        "test_chassis",
        "test_all_devices",
        "record_mapping",
        "start_autonomy",
        "/sensing/lidar/raw/pointcloud",
        "/sensing/lidar/concatenated/pointcloud",
        "/sensing/imu/raw/imu_data",
        "/sensing/imu/imu_data",
        "/g90/raw/nmea_sentence",
        "/vehicle/status/velocity_status",
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AA6079369-if00",
        "/dev/autoracer_g90_com2",
        "ros2 topic hz",
        "setsid",
        'kill -INT -- "-${pid}"',
        'return "${result}"',
        "telemetry_only:=true",
        "检查项会采样真实数据",
        "检查底盘串口数据",
        "检查全部设备",
        "录制建图数据",
        "启动自动驾驶",
        "scripts/autoracer_rc.sh [-v|--verbose] check",
        "scripts/autoracer_rc.sh [-v|--verbose] record mapping",
        "scripts/autoracer_rc.sh [-v|--verbose] start autonomy",
    ):
        assert token in RC_ENTRY_SOURCE
    for forbidden in (
        "pkill",
        "killall",
        "rc-stage-work",
        "colcon build",
        "colcon test",
        "vcs import",
        "180",
        "全部传感器持续启动",
        "当前完整系统启动",
        "scripts/rc",
        "test_all_sensors",
        "检查底盘反馈（只读，需授权）",
        "检查全部传感器数据",
        "/dev/autoracer_rc_g90",
        "setup_udev_rules",
        "setup udev",
        "udevadm control --reload-rules",
        "floor1_mapping_101",
        "start_stage4_bench",
        "start stage4-bench",
        "hold_active_profile",
    ):
        assert forbidden not in RC_ENTRY_SOURCE

    assert RC_ENTRY_SOURCE.index('kill -INT -- "-${pid}"') < RC_ENTRY_SOURCE.index(
        'kill -TERM -- "-${pid}"'
    )

    all_devices_start = RC_ENTRY_SOURCE.index("test_all_devices()")
    all_devices_end = RC_ENTRY_SOURCE.index("record_mapping()")
    all_devices = RC_ENTRY_SOURCE[all_devices_start:all_devices_end]
    calls = [
        all_devices.index("test_lidar || result=1"),
        all_devices.index("test_imu || result=1"),
        all_devices.index("test_g90 || result=1"),
        all_devices.index("test_chassis || result=1"),
    ]
    assert calls == sorted(calls)


def test_mapping_recording_is_manual_unlimited_fixed_or_float_and_sensor_only():
    for path in (
        RC_ENTRY_PATH,
        MAPPING_RECORDING_PATH,
        MAPPING_STOP_PATH,
        MAPPING_PREFLIGHT_PATH,
    ):
        assert path.stat().st_mode & 0o100

    for source in (MAPPING_RECORDING_SOURCE, MAPPING_STOP_SOURCE):
        assert "/home/wheeltec/Desktop/work" not in source
        assert 'MAPPING_ROOT="${RC_MAPPING_ROOT:-${WORKSPACE_ROOT}/rc-mapping}"' in (
            source
        )
    assert 'LIDAR_INTERFACE="${RC_LIDAR_INTERFACE:-}"' in MAPPING_RECORDING_SOURCE
    assert "discover_lidar_interface()" in MAPPING_RECORDING_SOURCE
    assert '"root": os.environ["PRODUCT_ROOT"]' in MAPPING_RECORDING_SOURCE
    assert "enP8p1s0" not in MAPPING_RECORDING_SOURCE

    for token in (
        'MAPPING_RECORDING_HELPER="${PRODUCT_ROOT}/scripts/autoracer_rc_recording.sh"',
        "record_mapping()",
        "RC_MAPPING_SITE",
        "RC_MAPPING_LABEL",
        "RC_MAPPING_OPERATOR",
        "RC_MAPPING_ALLOW_DEGRADED_LIDAR",
        "录制建图数据",
        "建图录制会停在 READY 等待 S",
        "record mapping",
    ):
        assert token in RC_ENTRY_SOURCE

    menu_start = RC_ENTRY_SOURCE.index("print_menu()")
    menu_end = RC_ENTRY_SOURCE.index("interactive_menu()")
    menu = RC_ENTRY_SOURCE[menu_start:menu_end]
    assert "6. 录制建图数据" in menu
    assert "7. 启动自动驾驶" in menu
    assert "阶段 4 台架全链路" not in menu

    for token in (
        "RESERVED_FREE_BYTES=$((50 * 1024 * 1024 * 1024))",
        "预检等待：不设超时，按 Q 取消",
        "--watch-pid",
        "[READY]",
        "[S] 开始录制  [Q] 取消",
        "[RECORDING]",
        "read -r -s -n 1 -t 2",
        "disk_reserve_stop_requested.txt",
        "录制时长不限",
        '"maximum_duration": None',
        '"name": "RTK_FIXED_OR_FLOAT"',
        '"accepted_gga_qualities": [4, 5]',
        '"quality_labels_preserved": True',
        'ALLOW_DEGRADED_LIDAR="${RC_MAPPING_ALLOW_DEGRADED_LIDAR:-0}"',
        "PREFLIGHT_RESULT == 3",
        "preflight_override.json",
        '"AUTHORIZED_DEGRADED_CAPTURE"',
        '"operator_start_confirmation_required": True',
        '"scope": "this recording session only"',
        'allowed_failures = {"lidar_rate", "lidar_maximum_gap"}',
        '"starts_chassis": False',
        '"starts_control_chain": False',
        '"sends_vehicle_commands": False',
        "launch_g90_corrections:=true",
        '"transport": "project-owned NTRIP relay; credentials excluded"',
    ):
        assert token in MAPPING_RECORDING_SOURCE

    ready = MAPPING_RECORDING_SOURCE.index("[READY]")
    start_choice = MAPPING_RECORDING_SOURCE.index("s|S)", ready)
    recorder_start = MAPPING_RECORDING_SOURCE.index(
        '"${RECORDER_COMMAND[@]}"', start_choice
    )
    assert ready < start_choice < recorder_start

    assert "WINDOW_SECONDS = 5.0" in MAPPING_PREFLIGHT_SOURCE
    assert "READINESS_STABILITY_SECONDS = 0.0" in MAPPING_PREFLIGHT_SOURCE
    for token in (
        "DiagnosticArray",
        'DIAGNOSTICS_TOPIC = "/diagnostics"',
        'status.hardware_id != "G90-COM2"',
        "diagnostic_level(status.level)",
        '(b"\\x01", 1)',
        "g90_com2_ready",
        "os.get_terminal_size(sys.stdin.fileno()).columns",
        "display_width",
        "render_status_rows",
        "changed_status_items",
        "readiness_summary",
        "status_items",
        "status_snapshot",
        "[设备预检",
        "[质量窗口]",
        "所有必需输入已同时在线；不再增加稳定等待",
    ):
        assert token in MAPPING_PREFLIGHT_SOURCE
    assert 'f"设备：LiDAR {lidar} ｜' not in MAPPING_PREFLIGHT_SOURCE
    assert '"  等待条件：" + "；".join(blockers)' not in MAPPING_PREFLIGHT_SOURCE
    assert "next_window_report" not in MAPPING_PREFLIGHT_SOURCE
    assert (
        '"${PREFLIGHT_COMMAND[@]}" 2>&1 | tee '
        '"${SESSION_DIR}/logs/preflight.log"'
    ) in MAPPING_RECORDING_SOURCE
    assert (
        'MINIMUM_RATE_HZ = {"lidar": 18.0, "imu": 90.0, '
        '"GGA": 9.5, "GST": 9.5, "THS": 9.5}'
    ) in MAPPING_PREFLIGHT_SOURCE
    assert "ACCEPTED_GGA_QUALITIES = frozenset({4, 5})" in MAPPING_PREFLIGHT_SOURCE
    assert "class OperatorCancelInput" in MAPPING_PREFLIGHT_SOURCE
    assert "ensure_process_alive(arguments.watch_pid)" in MAPPING_PREFLIGHT_SOURCE
    assert '"status": "CANCELLED"' in MAPPING_PREFLIGHT_SOURCE
    assert '"failure": "operator_cancelled"' in MAPPING_PREFLIGHT_SOURCE
    assert '"RTK_FIXED_OR_FLOAT"' in MAPPING_PREFLIGHT_SOURCE
    assert '"quality_labels_preserved": True' in MAPPING_PREFLIGHT_SOURCE

    for token in (
        "manual_stop_requested",
        "disk_reserve_stop_requested",
        "runtime_failure_reason_absent",
        "raw_sha256.txt",
        "stop_manifest.json",
        '"preflight_accepted"',
        "an exact audited override limited to lidar_rate/lidar_maximum_gap",
        'kill -INT -- "-${pgid}"',
        'kill -TERM -- "-${pgid}"',
    ):
        assert token in MAPPING_STOP_SOURCE
    assert MAPPING_STOP_SOURCE.index('kill -INT -- "-${pgid}"') < (
        MAPPING_STOP_SOURCE.index('kill -TERM -- "-${pgid}"')
    )

    recording_contract = (
        MAPPING_RECORDING_SOURCE
        + MAPPING_STOP_SOURCE
        + MAPPING_PREFLIGHT_SOURCE
        + MAPPING_QOS_PATH.read_text(encoding="utf-8")
    )
    for forbidden in (
        "--max-minutes",
        "--allow-rtk-float",
        "FIXED_ONLY_GGA_QUALITIES",
        "RTK_FIXED_ONLY",
        "user_accepted_rtk_float",
        "--with-hall",
        "/vehicle/status/",
        "/dev/autoracer_rc_chassis",
        "race_runtime_manager",
        "vehicle_cmd_gate",
        "/control/command/control_cmd",
        "ACQUISITION_TIMEOUT_SECONDS",
        "--acquisition-timeout",
        "acquisition_timeout",
    ):
        assert forbidden not in recording_contract

    assert set(_yaml(MAPPING_QOS_PATH)) == {
        "/sensing/lidar/raw/pointcloud",
        "/sensing/imu/raw/imu_data",
        "/g90/raw/nmea_sentence",
        "/tf_static",
        "/tf",
        "/diagnostics",
    }
    recording_qos = _yaml(MAPPING_QOS_PATH)
    assert (
        recording_qos["/sensing/lidar/raw/pointcloud"]["reliability"]
        == "reliable"
    )
    assert "def subscription_ready(self)" in MAPPING_PREFLIGHT_SOURCE
    assert "while not lidar_probe.subscription_ready()" in MAPPING_PREFLIGHT_SOURCE


def test_mapping_and_autonomy_dashboards_are_fixed_screen_and_width_aware():
    for source, prefix in (
        (MAPPING_RECORDING_SOURCE, "mapping_dashboard"),
        (RC_ENTRY_SOURCE, "terminal_ui"),
    ):
        assert "\\033[?1049h" in source
        assert "\\033[?25l" in source
        assert "\\033[?25h" in source
        assert "\\033[?1049l" in source
        assert "\\033[J" in source
        assert "stty size </dev/tty" in source
        assert f"{prefix}_fit()" in source
        assert f"{prefix}_draw()" in source

    mapping_fit = _bash_function(
        MAPPING_RECORDING_SOURCE,
        "mapping_dashboard_fit",
        "mapping_dashboard_draw",
    )
    autonomy_fit = _bash_function(
        RC_ENTRY_SOURCE,
        "terminal_ui_fit",
        "terminal_ui_draw",
    )
    sample = "状态        这是一条很长的真实故障原因 / rc-map-id-with-long-name"
    for width in (20, 40, 60):
        for function_source, function_name in (
            (mapping_fit, "mapping_dashboard_fit"),
            (autonomy_fit, "terminal_ui_fit"),
        ):
            rendered = _shell_fit(function_source, function_name, sample, width)
            assert _dashboard_width(rendered) <= width
            assert rendered.endswith("…")

    for stage in (1, 3, 4, 5, 6):
        assert f"阶段 {stage}/6" in MAPPING_RECORDING_SOURCE
    assert "stage_number=2" in MAPPING_PREFLIGHT_SOURCE
    assert 'stage_name="固定质量窗口"' in MAPPING_PREFLIGHT_SOURCE
    assert 'DASHBOARD_FD_ENV = "RC_MAPPING_DASHBOARD_FD"' in (
        MAPPING_PREFLIGHT_SOURCE
    )
    assert "class TerminalDashboard" in MAPPING_PREFLIGHT_SOURCE
    assert '"\\x1b[H" + "\\n".join(frame) + "\\x1b[J"' in (
        MAPPING_PREFLIGHT_SOURCE
    )
    assert "render_dashboard_status_rows" in MAPPING_PREFLIGHT_SOURCE
    assert "self-test dashboard changed its fixed row count" in (
        MAPPING_PREFLIGHT_SOURCE
    )
    assert "for columns in (20, 40, 60, 80, 120)" in MAPPING_PREFLIGHT_SOURCE
    assert (
        '"${PREFLIGHT_COMMAND[@]}" >"${SESSION_DIR}/logs/preflight.log" 2>&1'
        in MAPPING_RECORDING_SOURCE
    )
    assert '"${SESSION_DIR}/logs/stop_console.log"' in MAPPING_RECORDING_SOURCE

    mapping_failure = _bash_function(
        MAPPING_RECORDING_SOURCE, "fail", "validate_text"
    )
    assert mapping_failure.index("mapping_dashboard_render_failure") < (
        mapping_failure.index("mapping_dashboard_wait_for_q")
    )
    mapping_finalize = MAPPING_RECORDING_SOURCE[
        MAPPING_RECORDING_SOURCE.index("finalize_recording()"):
        MAPPING_RECORDING_SOURCE.index('RECORDING_STARTED_EPOCH="$(date +%s)"')
    ]
    assert mapping_finalize.index("mapping_dashboard_render_result") < (
        mapping_finalize.index("mapping_dashboard_wait_for_q")
    )

    for stage in range(1, 7):
        assert f"阶段 {stage}/6" in RC_ENTRY_SOURCE
    for renderer in (
        "autonomy_render_preparing",
        "autonomy_render_ready",
        "autonomy_render_start_confirmation",
        "autonomy_render_running",
        "autonomy_render_stopping",
        "autonomy_render_complete",
        "autonomy_render_failure",
    ):
        assert f"{renderer}()" in RC_ENTRY_SOURCE
    assert "runtime 快照未提供数值，不猜测" in RC_ENTRY_SOURCE
    autonomy_renderers = RC_ENTRY_SOURCE[
        RC_ENTRY_SOURCE.index("autonomy_render_preparing()"):
        RC_ENTRY_SOURCE.index("wait_for_q_acknowledgement()")
    ]
    assert "｜" not in autonomy_renderers
    assert "ARRIVED / FINISHED" not in autonomy_renderers
    assert "当前快照未区分到达或人工停车" in autonomy_renderers

    acknowledgement = _bash_function(
        RC_ENTRY_SOURCE,
        "acknowledge_autonomy_failure",
        "wait_for_autonomy_ready",
    )
    assert acknowledgement.index("autonomy_render_failure") < (
        acknowledgement.index("wait_for_q_acknowledgement")
    )
    autonomy_start = RC_ENTRY_SOURCE[
        RC_ENTRY_SOURCE.index("start_autonomy()"):
        RC_ENTRY_SOURCE.index("run_and_report()")
    ]
    success_page = autonomy_start.index("autonomy_render_complete")
    assert autonomy_start.rindex("stop_active", 0, success_page) < success_page
    assert success_page < autonomy_start.index(
        "wait_for_q_acknowledgement", success_page
    )


def test_automatic_driving_entry_uses_the_user_approved_low_speed_profile():
    assert _yaml(DRIVING_PROFILES_PATH) == {
        "schema_version": 1,
        "profiles": {
            "low_speed_1mps": {
                "display_name": "1 m/s 低速验证",
                "approved": True,
                "max_speed_mps": 1.0,
                "max_accel_mps2": 0.4,
                "max_decel_mps2": -0.6,
                "command_latency_sec": 0.2,
                "stopping_margin_m": 5.0,
            }
        },
    }
    assert "<exec_depend>autoracer_bringup</exec_depend>" in BRINGUP_PACKAGE_SOURCE

    for token in (
        'DeclareLaunchArgument("localization_map_path")',
        'DeclareLaunchArgument("course_path")',
        'DeclareLaunchArgument("max_speed_mps")',
        '"departure_speed_mps": "0.3"',
        '"launch_lidar": "true"',
        '"launch_imu": "true"',
        '"launch_g90": "true"',
        '"launch_g90_driver": "true"',
        '"launch_g90_corrections": "true"',
        '"g90_param_file": g90_param_file',
        '"telemetry_only": "false"',
        '"use_sim_time": "false"',
        '"system_run_mode": "online"',
        '"vehicle_info.param.yaml"',
        '"vehicle_cmd_gate.param.yaml"',
        '"race_runtime.param.yaml"',
        '_launch_file("autoracer_bringup", "race.launch.py")',
    ):
        assert token in RACE_LAUNCH_SOURCE

    for token in (
        "discover_autonomy_assets",
        "validate_course_map_contract",
        "discover_driving_profiles",
        "select_autonomy_asset",
        "select_driving_profile",
        "自动使用运行方案",
        "AUTONOMY_COURSE_DIR",
        'course_path:="${AUTONOMY_COURSE_DIR}"',
        "start_autonomy",
        "/autoracer/race/${action}",
        "[S] 开始自动驾驶  [Q] 取消",
        "自动驾驶准备不设超时；故障结论显示后按 Q 清理返回",
        "wait_for_autonomy_start_decision",
        "wait_for_q_acknowledgement",
        "acknowledge_autonomy_failure",
        "monitor_autonomy_run",
        "runtime_fault_is_stopped",
        "start_runtime_state_watch",
        "stop_runtime_state_watch",
        "autoracer_rc_runtime_state_watch.py",
        '${TASK_INDEX}-runtime-state.json',
        "RC_AUTONOMY_AUTHORIZED=1",
        "没有可用于自动驾驶的正式地图",
        "没有获批的自动驾驶运行方案",
        "USER_APPROVED_LOW_SPEED_VALIDATION",
        "地图航向补偿与 G90 运行参数不一致",
        'g90_param_file:="${G90_PARAM_FILE}"',
    ):
        assert token in RC_ENTRY_SOURCE

    assert "7. 启动自动驾驶" in RC_ENTRY_SOURCE
    assert "autonomy) start_autonomy" in RC_ENTRY_SOURCE
    assert "RC_AUTONOMY_MAX_SPEED" not in RC_ENTRY_SOURCE
    assert "请选择运行方案：" not in RC_ENTRY_SOURCE
    assert "AUTONOMY_READY_TIMEOUT_SEC" not in RC_ENTRY_SOURCE
    assert "RC_AUTONOMY_READY_TIMEOUT_SEC" not in RC_ENTRY_SOURCE
    assert "floor1_mapping_101" not in RACE_LAUNCH_SOURCE
    assert "floor1_mapping_101" not in RC_ENTRY_SOURCE
    assert "stage4-bench" not in RC_ENTRY_SOURCE

    ready_start = RC_ENTRY_SOURCE.index("wait_for_autonomy_ready()")
    ready_end = RC_ENTRY_SOURCE.index("wait_for_autonomy_start_decision()")
    ready_contract = RC_ENTRY_SOURCE[ready_start:ready_end]
    assert "deadline" not in ready_contract
    assert "不设超时，按 Q 取消" in ready_contract

    decision_start = ready_end
    decision_end = RC_ENTRY_SOURCE.index("call_race_service()")
    decision_contract = RC_ENTRY_SOURCE[decision_start:decision_end]
    assert "local start_requested=0" in decision_contract
    assert "start_requested=1" in decision_contract
    assert "已收到开始请求；正在进行最终状态确认，按 Q 取消。" in decision_contract
    assert "最终差分确认：等待新的 G90 差分诊断" in decision_contract
    assert "当前状态不是 READY，未执行开始请求" not in decision_contract
    assert decision_contract.index("s|S)") < decision_contract.index("start_requested=1")
    assert decision_contract.index("start_requested=1") < decision_contract.index(
        "read_g90_correction_snapshot 2"
    )
    assert decision_contract.index("read_g90_correction_snapshot 2") < (
        decision_contract.index("return 0")
    )

    runtime_state_start = RC_ENTRY_SOURCE.index("runtime_state_json()")
    runtime_state_end = RC_ENTRY_SOURCE.index("read_runtime_snapshot()")
    runtime_state_contract = RC_ENTRY_SOURCE[
        runtime_state_start:runtime_state_end
    ]
    assert "head -n 1" not in runtime_state_contract
    assert "ros2 topic echo" not in runtime_state_contract
    assert 'read -r output <"${RUNTIME_STATE_FILE}"' in runtime_state_contract

    for token in (
        '"/system/race_runtime/state"',
        "ReliabilityPolicy.RELIABLE",
        "DurabilityPolicy.TRANSIENT_LOCAL",
        "STATE_FIELDS",
        "os.replace",
        "output_path.unlink(missing_ok=True)",
    ):
        assert token in RUNTIME_STATE_WATCH_SOURCE

    autonomy_start = RC_ENTRY_SOURCE.index("start_autonomy()")
    autonomy_end = RC_ENTRY_SOURCE.index("run_and_report()", autonomy_start)
    autonomy_contract = RC_ENTRY_SOURCE[autonomy_start:autonomy_end]
    for conclusion in (
        "自动驾驶资产检查未通过",
        "自动驾驶运行方案检查未通过",
        "自动驾驶设备路径检查未通过",
        "自动驾驶运行图启动失败",
        "自动驾驶未达到 READY",
        "自动驾驶在等待开始时失去可运行状态",
        "自动驾驶开始请求未执行",
        "自动驾驶运行未正常完成",
    ):
        assert conclusion in autonomy_contract


def test_rc_entry_rate_probe_exits_on_the_first_bounded_frequency_result():
    probe_start = RC_ENTRY_SOURCE.index("observe_rate()")
    probe_end = RC_ENTRY_SOURCE.index("observe_field()")
    probe = RC_ENTRY_SOURCE[probe_start:probe_end]

    assert 'RATE_WAIT_SEC="${RC_RATE_WAIT_SEC:-8}"' in RC_ENTRY_SOURCE
    assert 'timeout "${RATE_WAIT_SEC}s"' in probe
    assert "env PYTHONUNBUFFERED=1" in probe
    assert 'ros2 topic hz --window 200 --wall-time "${topic}"' in probe
    assert "awk '/average rate:/ {print; exit}'" in probe
    assert 'timeout "${OBSERVE_SEC}s" ros2 topic hz' not in probe
    assert 'sleep "${RATE_WAIT_SEC}"' not in probe


def test_rc_entry_is_concise_by_default_and_has_one_explicit_verbose_mode():
    for token in (
        "VERBOSE=0",
        "is_verbose()",
        "-v|--verbose",
        "Output is concise by default.",
        "当前使用全量输出模式",
        "LiDAR：raw",
        "IMU：raw",
        "底盘：反馈",
        "设备链路 4/4 正常",
    ):
        assert token in RC_ENTRY_SOURCE
    assert "RC_VERBOSE" not in RC_ENTRY_SOURCE
    assert 'verbose_ok "${topic} (${topic_type})"' in RC_ENTRY_SOURCE
    assert 'verbose_ok "${topic}: ${rate}"' in RC_ENTRY_SOURCE


def test_rc_entry_counts_real_g90_sentences_and_separates_link_from_position():
    probe_start = RC_ENTRY_SOURCE.index("nmea_sentence_count()")
    probe_end = RC_ENTRY_SOURCE.index("telemetry_authorized()")
    probe = RC_ENTRY_SOURCE[probe_start:probe_end]

    assert 'index($0, "sentence: $") && index($0, kind)' in probe
    for kind in ("GGA", "GST", "THS"):
        assert f'nmea_sentence_count "${{capture}}" {kind}' in probe
        assert f'latest_nmea_sentence "${{capture}}" {kind}' in probe
    for token in (
        '[[ "${G90_QUALITY}" =~ ^(4|5)$ ]]',
        '[[ "${G90_GST_STATE}" == "完整" ]]',
        '[[ "${G90_THS_MODE}" == "A" ]]',
        "G90_POSITION_READY=1",
        "observe_g90_fix_sample()",
        'timeout "${TOPIC_WAIT_SEC}s"',
        "ros2 topic echo --no-daemon /g90/fix --once",
        "frame_id: gnss_link",
        "COM1/COM2 与 NMEA/适配链",
        "定位未就绪",
    ):
        assert token in probe
    assert "sentence:.*\\\\$" not in probe
    assert probe.count("ros2 topic echo --no-daemon /g90/fix --once") == 1
    for field in (
        "header",
        "status",
        "latitude",
        "longitude",
        "position_covariance",
    ):
        assert f"observe_field /g90/fix {field}" not in probe


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


def test_imu_qos_and_unknown_gyro_covariance_are_normalized_at_platform_boundary():
    assert '("input", "/sensing/imu/raw/imu_data")' in SENSING_SOURCE
    assert '("output", "/sensing/imu/imu_data")' in SENSING_SOURCE
    assert "input_qos.best_effort().durability_volatile()" in IMU_ADAPTER_NODE_SOURCE
    assert "output_qos.reliable().durability_volatile()" in IMU_ADAPTER_NODE_SOURCE
    params = _yaml(PACKAGE_ROOT / "config" / "rc" / "imu.param.yaml")[
        "imu_qos_adapter"
    ]["ros__parameters"]
    assert params["fallback_angular_velocity_stddev_radps"] == 0.02
    assert "parameters=[ParameterFile(imu_param_file" in SENSING_SOURCE
    assert "auto output = *message" in IMU_ADAPTER_NODE_SOURCE
    assert "normalize_angular_velocity_covariance" in IMU_ADAPTER_NODE_SOURCE
    assert "publisher_->publish(output)" in IMU_ADAPTER_NODE_SOURCE
    for measurement in ("angular_velocity.x", "angular_velocity.y", "angular_velocity.z"):
        assert measurement + " =" not in IMU_ADAPTER_NODE_SOURCE


def test_static_sensor_transforms_are_unique_and_match_confirmed_measurements():
    transforms = _yaml(
        RC_ROOT / "autoracer_rc_description" / "config" / "sensor_extrinsics.yaml"
    )["transforms"]
    assert len(transforms) == 3
    assert len({item["child"] for item in transforms}) == 3
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
    assert by_child["gnss_link"]["parent"] == "base_link"
    assert by_child["gnss_link"]["translation"] == {
        "x": 0.280,
        "y": 0.140,
        "z": 0.240,
    }
    for transform in transforms:
        assert transform["rotation_rpy"] == {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
