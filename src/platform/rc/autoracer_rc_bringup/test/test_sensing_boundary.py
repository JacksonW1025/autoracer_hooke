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


def test_g90_formal_pose_uses_the_field_calibrated_heading_contract():
    params = _yaml(G90_CONFIG_PATH)["/g90/g90_nmea_adapter"]["ros__parameters"]
    assert params["frame_id"] == "gnss_link"
    assert params["base_frame"] == "base_link"
    assert params["enable_localization_output"] is True
    assert params["allow_statusless_hdt"] is False
    assert params["heading_mount_offset_deg"] == 73.0
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
    ):
        assert token in MAPPING_RECORDING_SOURCE

    ready = MAPPING_RECORDING_SOURCE.index("[READY]")
    start_choice = MAPPING_RECORDING_SOURCE.index("s|S)", ready)
    recorder_start = MAPPING_RECORDING_SOURCE.index(
        '"${RECORDER_COMMAND[@]}"', start_choice
    )
    assert ready < start_choice < recorder_start

    assert "WINDOW_SECONDS = 10.0" in MAPPING_PREFLIGHT_SOURCE
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
    ):
        assert token in MAPPING_STOP_SOURCE

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
        '"launch_lidar": "true"',
        '"launch_imu": "true"',
        '"launch_g90": "true"',
        '"launch_g90_driver": "true"',
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
        "ros2 topic echo /g90/fix --once",
        "frame_id: gnss_link",
        "设备链路与 NMEA/适配链",
        "定位未就绪",
    ):
        assert token in probe
    assert "sentence:.*\\\\$" not in probe
    assert probe.count("ros2 topic echo /g90/fix --once") == 1
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
