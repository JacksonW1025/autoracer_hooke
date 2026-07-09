import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def package_name(package_xml: Path) -> str:
    tree = ET.parse(package_xml)
    name = tree.getroot().findtext("name")
    assert name
    return name


def test_autoware_launch_is_pinned_as_the_official_entrypoint():
    repos = read("autoracer.repos")

    assert "src/external/autoware/launcher" in repos
    assert "https://github.com/autowarefoundation/autoware_launch.git" in repos
    assert "version: 0.50.0" in repos


def test_autoracer_vehicle_and_sensor_kit_packages_follow_official_names():
    expected_packages = {
        "src/autoracer_rc_description/package.xml": "autoracer_rc_description",
        "src/autoracer_rc_launch/package.xml": "autoracer_rc_launch",
        "src/autoracer_rc_sensor_kit_description/package.xml": (
            "autoracer_rc_sensor_kit_description"
        ),
        "src/autoracer_rc_sensor_kit_launch/package.xml": (
            "autoracer_rc_sensor_kit_launch"
        ),
    }

    for relative_path, expected_name in expected_packages.items():
        package_xml = ROOT / relative_path
        assert package_xml.exists(), f"missing package.xml: {relative_path}"
        assert package_name(package_xml) == expected_name

    assert "autoracer_bringup" not in read("src/autoracer_rc_launch/package.xml")
    assert "autoracer_bringup" not in read("src/autoracer_rc_sensor_kit_launch/package.xml")


def test_hooke_profile_placeholders_are_disabled_until_real_profile_exists():
    expected_placeholders = {
        "src/autoracer_hooke_description": [
            "vehicle_info.param.yaml",
            "vehicle.xacro",
            "real Hooke vehicle geometry",
        ],
        "src/autoracer_hooke_launch": [
            "vehicle_interface.launch.xml",
            "Hooke CAN adapter",
            "command gate",
        ],
        "src/autoracer_hooke_sensor_kit_description": [
            "sensor_kit_calibration.yaml",
            "sensors_calibration.yaml",
            "real Hooke sensor extrinsics",
        ],
        "src/autoracer_hooke_sensor_kit_launch": [
            "sensing.launch.xml",
            "Hesai",
            "Fixposition",
        ],
    }

    for relative_path, required_terms in expected_placeholders.items():
        placeholder = ROOT / relative_path
        assert placeholder.is_dir(), f"missing Hooke placeholder: {relative_path}"
        assert (placeholder / "COLCON_IGNORE").exists()
        assert not (placeholder / "package.xml").exists()
        assert (placeholder / "README.md").exists()
        assert (placeholder / "profile_requirements.yaml").exists()

        combined = "\n".join(
            (
                (placeholder / "README.md").read_text(encoding="utf-8"),
                (placeholder / "profile_requirements.yaml").read_text(encoding="utf-8"),
            )
        )
        for term in (
            "disabled_placeholder",
            "not runtime ready",
            "Remove COLCON_IGNORE only after",
            "autoracer_hooke",
            "autoracer_hooke_sensor_kit",
            *required_terms,
        ):
            assert term in combined, f"{term!r} missing from {relative_path}"


def test_official_launch_packages_expose_expected_launch_files_and_rc_hardware():
    vehicle_launch = ROOT / "src" / "autoracer_rc_launch" / "launch" / "vehicle_interface.launch.xml"
    sensing_launch = (
        ROOT
        / "src"
        / "autoracer_rc_sensor_kit_launch"
        / "launch"
        / "sensing.launch.xml"
    )
    lidar_config = (
        ROOT / "src" / "autoracer_rc_sensor_kit_launch" / "config" / "lslidar_cx.yaml"
    )
    vehicle_info = (
        ROOT
        / "src"
        / "autoracer_rc_description"
        / "config"
        / "vehicle_info.param.yaml"
    )
    sensor_calibration = (
        ROOT
        / "src"
        / "autoracer_rc_sensor_kit_description"
        / "config"
        / "sensor_kit_calibration.yaml"
    )

    for path in (vehicle_launch, sensing_launch, lidar_config, vehicle_info, sensor_calibration):
        assert path.exists(), f"missing official Autoware integration file: {path}"

    vehicle_text = vehicle_launch.read_text(encoding="utf-8")
    assert "autoracer_vehicle_interface" in vehicle_text
    assert "rc_serial_interface" in vehicle_text
    assert "autoracer_safety" in vehicle_text
    assert "command_gate" in vehicle_text
    assert "ENABLE_DRIVE_COMMANDS" in vehicle_text
    assert "/autoracer/control/safe_control_cmd" in vehicle_text
    assert "/dev/ttyUSB0" not in vehicle_text

    sensing_text = sensing_launch.read_text(encoding="utf-8")
    for term in (
        "lslidar_driver",
        "hipnuc_imu",
        "pointcloud_voxel_filter",
        "/dev/ttyUSB0",
        "lslidar_cx.yaml",
        "/sensing/imu/imu_data_raw",
        "/sensing/imu/imu_data",
    ):
        assert term in sensing_text
    assert '<group if="$(var launch_driver)">' in sensing_text

    lidar_config_text = lidar_config.read_text(encoding="utf-8")
    assert "device_ip: 192.168.1.200" in lidar_config_text
    assert "192.168.1.200" in lidar_config_text

    calibration_text = sensor_calibration.read_text(encoding="utf-8")
    assert "lidar_top" in calibration_text
    assert "-1.5708" in calibration_text


def test_operator_docs_prefer_official_autoware_launch_command():
    readme = read("README.md")
    rc_start = read("scripts/rc/rc_start_autoware.sh")
    run_official = read("scripts/run_official_autoware.sh")

    assert "ros2 launch autoware_launch autoware.launch.xml" in readme
    assert "vehicle_model:=autoracer_rc" in readme
    assert "sensor_model:=autoracer_rc_sensor_kit" in readme
    assert "launch_perception:=false" in readme
    assert "rviz:=false" in readme
    assert "launch_vehicle_interface:=false" in readme
    assert "run_official_autoware.sh" in rc_start
    assert "ros2 launch autoware_launch autoware.launch.xml" in run_official
    assert "AUTORACER_VEHICLE_MODEL:=autoracer_rc" in run_official
    assert "AUTORACER_SENSOR_MODEL:=autoracer_rc_sensor_kit" in run_official
    assert "Only the RC official profile is enabled in this branch" in run_official
    assert "require_active_profile_pair" in run_official
    assert "disabled_placeholder" in run_official
    assert 'vehicle_model:="${AUTORACER_VEHICLE_MODEL}"' in run_official
    assert 'sensor_model:="${AUTORACER_SENSOR_MODEL}"' in run_official
    assert 'launch_perception:="${LAUNCH_PERCEPTION}"' in run_official
    assert "SERIAL_PORT is required when LAUNCH_VEHICLE_INTERFACE=true" in run_official


def test_official_wrapper_rejects_disabled_hooke_profile_before_ros_launch():
    env = os.environ.copy()
    env.update(
        {
            "AUTORACER_VEHICLE_MODEL": "autoracer_hooke",
            "AUTORACER_SENSOR_MODEL": "autoracer_hooke_sensor_kit",
            "MAP_PATH": str(ROOT / "maps"),
            "LAUNCH_VEHICLE_INTERFACE": "false",
            "RC_REQUIRE_LIDAR_LINK": "false",
        }
    )

    result = subprocess.run(
        ["bash", "scripts/run_official_autoware.sh"],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )

    assert result.returncode == 2
    assert "Only the RC official profile is enabled in this branch" in result.stderr
    assert "Requested vehicle_model=autoracer_hooke" in result.stderr
    assert "disabled_placeholder" in result.stderr
    assert "ros2 launch" not in result.stderr


def test_official_localization_contract_uses_upstream_default_pointcloud_topic():
    localization_component = read(
        "src/external/autoware/launcher/autoware_launch/launch/components/tier4_localization_component.launch.xml"
    )
    docs = "\n".join(
        read(path)
        for path in (
            "docs/architecture/runtime_alignment_audit_zh.md",
            "docs/reference/interfaces_and_topics_zh.md",
        )
    )

    assert 'name="input_pointcloud" default="/sensing/lidar/concatenated/pointcloud"' in localization_component
    assert "official localization 默认消费 `/sensing/lidar/concatenated/pointcloud`" in docs
    assert "runtime localization consumes the official default concatenated topic" in docs


def test_official_localization_docs_require_full_map_directory():
    docs = "\n".join(
        read(path)
        for path in (
            "README.md",
            "docs/operations/mapping_workflow_zh.md",
            "docs/operations/rc_runbook_zh.md",
            "docs/operations/rc_full_chain_execution_zh.md",
        )
    )

    assert "pointcloud_map.pcd" in docs
    assert "pointcloud_map_metadata.yaml" in docs
    assert "lanelet2_map.osm" in docs
    assert "map_projector_info.yaml" in docs
    assert "PCD-only" not in docs
    assert "只有 PCD" not in docs
    assert "没有 Lanelet2 地图时只验证 localization-only" not in docs


def test_official_branch_operator_entrypoints_do_not_call_legacy_track_launcher():
    entrypoint_files = [
        "scripts/rc/rc_start_autoware.sh",
        "scripts/rc/rc_start_sensors.sh",
        "scripts/rc/rc_start_mapping_bag.sh",
        "scripts/rc/rc_capture_mapping_bag.sh",
        "scripts/rc/rc_start_localization.sh",
        "scripts/rc/rc_stop.sh",
    ]

    for relative_path in entrypoint_files:
        text = read(relative_path)
        assert "run_track.sh" not in text, relative_path
        assert "autoracer_bringup track.launch.py" not in text, relative_path
        assert "track_rc_p0.launch.py" not in text, relative_path

    assert not (ROOT / "scripts" / "run_track.sh").exists()
    assert not (
        ROOT / "src" / "autoracer_bringup" / "launch" / "track.launch.py"
    ).exists()
    assert not (
        ROOT / "src" / "autoracer_bringup" / "launch" / "track_rc_p0.launch.py"
    ).exists()
