from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RC_ROOT = PACKAGE_ROOT.parent
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


def _yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_rc_sensing_terminates_at_platform_independent_topics():
    assert '"/sensing/lidar/concatenated/pointcloud"' in SENSING_SOURCE
    assert "/sensing/imu/imu_data" in IMU_CONFIG_SOURCE
    assert 'package="autoracer_rc_adapter"' in SENSING_SOURCE


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
    assert params["imu_topic"] == "/sensing/imu/imu_data"
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
    assert "madgwick" not in (SENSING_SOURCE + IMU_CONFIG_SOURCE).lower()


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
