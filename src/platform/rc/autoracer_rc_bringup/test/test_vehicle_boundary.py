from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RC_ROOT = PACKAGE_ROOT.parent
ADAPTER_ROOT = RC_ROOT / "autoracer_rc_adapter"
VEHICLE_LAUNCH_SOURCE = (PACKAGE_ROOT / "launch" / "vehicle.launch.py").read_text(
    encoding="utf-8"
)
VEHICLE_CONFIG_SOURCE = (
    PACKAGE_ROOT / "config" / "rc" / "vehicle.param.yaml"
).read_text(encoding="utf-8")
UDEV_SOURCE = (
    PACKAGE_ROOT / "udev" / "99-autoracer-rc-chassis.rules"
).read_text(encoding="utf-8")
VEHICLE_NODE_SOURCE = (
    ADAPTER_ROOT / "src" / "rc_vehicle_interface_node.cpp"
).read_text(encoding="utf-8")
PROTOCOL_HEADER_SOURCE = (
    ADAPTER_ROOT
    / "include"
    / "autoracer_rc_adapter"
    / "rc_serial_protocol.hpp"
).read_text(encoding="utf-8")
PROTOCOL_SOURCE = (ADAPTER_ROOT / "src" / "rc_serial_protocol.cpp").read_text(
    encoding="utf-8"
)
KINEMATICS_SOURCE = (
    ADAPTER_ROOT / "src" / "rc_vehicle_kinematics.cpp"
).read_text(encoding="utf-8")


def _vehicle_parameters():
    return yaml.safe_load(VEHICLE_CONFIG_SOURCE)["rc_vehicle_interface"][
        "ros__parameters"
    ]


def test_vehicle_launch_uses_a_stable_overridable_chassis_identity():
    assert "/dev/autoracer_rc_chassis" in VEHICLE_LAUNCH_SOURCE
    assert 'LaunchConfiguration("serial_port")' in VEHICLE_LAUNCH_SOURCE
    assert "1a86" in UDEV_SOURCE
    assert "55d4" in UDEV_SOURCE
    assert "0003" in UDEV_SOURCE
    for unstable_name in ("/dev/ttyUSB", "/dev/ttyCH343", "/dev/wheeltec_"):
        assert unstable_name not in VEHICLE_LAUNCH_SOURCE
        assert unstable_name not in VEHICLE_CONFIG_SOURCE


def test_vehicle_parameters_match_frozen_firmware_and_confirmed_boundaries():
    params = _vehicle_parameters()
    assert params["serial_port"] == "/dev/autoracer_rc_chassis"
    assert params["baud_rate"] == 115200
    assert params["maximum_command_speed_mps"] == 3.0
    assert params["minimum_command_speed_mps"] == 0.3
    assert params["max_steering_tire_angle_rad"] == 0.262
    assert params["firmware_command_timeout_ms"] == 250
    assert params["base_frame_id"] == "base_link"
    assert "wheelbase_m" not in params


def test_vehicle_node_uses_direct_standard_control_and_truthful_velocity_contract():
    assert '"/control/command/control_cmd"' in VEHICLE_NODE_SOURCE
    assert '"/vehicle/status/velocity_status"' in VEHICLE_NODE_SOURCE
    assert "autoware_control_msgs::msg::Control" in VEHICLE_NODE_SOURCE
    assert "autoware_vehicle_msgs::msg::VelocityReport" in VEHICLE_NODE_SOURCE
    assert "command.speed_mps = setpoint.speed_mps" in VEHICLE_NODE_SOURCE
    assert (
        "command.steering_tire_angle_rad = setpoint.steering_tire_angle_rad"
        in VEHICLE_NODE_SOURCE
    )
    assert "command.software_stop = false" in VEHICLE_NODE_SOURCE
    assert "feedback.yaw_rate_estimate_rad_s" in VEHICLE_NODE_SOURCE
    assert "command_stamp_is_fresh" in VEHICLE_NODE_SOURCE
    assert "age_ns > maximum_age_ns" in VEHICLE_NODE_SOURCE
    assert "rejected stale Control command" in VEHICLE_NODE_SOURCE
    assert "software_stop.enable = false" in VEHICLE_NODE_SOURCE
    assert "software_stop.software_stop = true" in VEHICLE_NODE_SOURCE

    forbidden = (
        "SteeringReport",
        "GearReport",
        "ControlModeReport",
        "control_mode_request",
        "enable_drive_commands",
        "sensor_msgs::msg::Imu",
    )
    for token in forbidden:
        assert token not in VEHICLE_NODE_SOURCE


def test_protocol_matches_frozen_ackermann_wire_contract():
    combined = PROTOCOL_HEADER_SOURCE + PROTOCOL_SOURCE
    assert "kRcCommandFrameSize = 11U" in PROTOCOL_HEADER_SOURCE
    assert "kRcFeedbackFrameSize = 24U" in PROTOCOL_HEADER_SOURCE
    assert "kRcAckermannCommandId = 0x01U" in PROTOCOL_HEADER_SOURCE
    assert "kRcTelemetryProtocolId = 0xA1U" in PROTOCOL_HEADER_SOURCE
    assert "kRcCommandFlagEnable = 0x01U" in PROTOCOL_HEADER_SOURCE
    assert "kRcCommandFlagSoftwareStop = 0x80U" in PROTOCOL_HEADER_SOURCE
    assert "speed_mps" in combined
    assert "steering_tire_angle_rad" in combined
    assert "frame[7] = 0U" in PROTOCOL_SOURCE
    assert "frame[8] = 0U" in PROTOCOL_SOURCE
    assert "calculate_bcc(frame.data(), 9U)" in PROTOCOL_SOURCE
    assert "calculate_bcc(frame.data(), 22U)" in PROTOCOL_SOURCE
    for legacy_field in ("vx_mps", "vy_mps", "wz_rad_s", "imu_placeholder"):
        assert legacy_field not in combined


def test_command_boundary_is_direct_and_does_not_recreate_yaw_rate_conversion():
    combined = VEHICLE_NODE_SOURCE + KINEMATICS_SOURCE
    assert "maximum_command_speed_mps" in combined
    assert "minimum_command_speed_mps" in combined
    assert "max_steering_tire_angle_rad" in combined
    assert "speed_below_minimum" in combined
    for legacy_conversion in ("wheelbase_m", "std::tan", "wz_rad_s", "vy_mps"):
        assert legacy_conversion not in combined


def test_vehicle_node_does_not_restore_the_rejected_half_meter_product_limit():
    combined = VEHICLE_NODE_SOURCE + VEHICLE_CONFIG_SOURCE + KINEMATICS_SOURCE
    assert "maximum_command_speed_mps: 3.0" in VEHICLE_CONFIG_SOURCE
    assert "0.5 m/s" not in combined
    assert "0.500" not in combined


def test_vehicle_launch_remains_chassis_only():
    forbidden = (
        "sensing.launch",
        "lslidar",
        "hipnuc",
        "map_loader",
        "localization",
        "planning",
        "race.launch",
        "autoracer_control",
    )
    for token in forbidden:
        assert token not in VEHICLE_LAUNCH_SOURCE
