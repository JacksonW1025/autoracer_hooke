from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RC_ROOT = PACKAGE_ROOT.parent
ADAPTER_ROOT = RC_ROOT / "autoracer_rc_adapter"
ADAPTER_CMAKE_SOURCE = (ADAPTER_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
ADAPTER_PACKAGE_SOURCE = (ADAPTER_ROOT / "package.xml").read_text(encoding="utf-8")
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
VEHICLE_STATE_SOURCE = (
    ADAPTER_ROOT / "src" / "rc_vehicle_state.cpp"
).read_text(encoding="utf-8")
VEHICLE_STATE_HEADER_SOURCE = (
    ADAPTER_ROOT
    / "include"
    / "autoracer_rc_adapter"
    / "rc_vehicle_state.hpp"
).read_text(encoding="utf-8")
VEHICLE_INFO_SOURCE = (
    PACKAGE_ROOT / "config" / "rc" / "vehicle_info.param.yaml"
).read_text(encoding="utf-8")
VEHICLE_CMD_GATE_SOURCE = (
    PACKAGE_ROOT / "config" / "rc" / "vehicle_cmd_gate.param.yaml"
).read_text(encoding="utf-8")
RACE_RUNTIME_SOURCE = (
    PACKAGE_ROOT / "config" / "rc" / "race_runtime.param.yaml"
).read_text(encoding="utf-8")


def _vehicle_parameters():
    return yaml.safe_load(VEHICLE_CONFIG_SOURCE)["rc_vehicle_interface"][
        "ros__parameters"
    ]


def _global_parameters(source):
    return yaml.safe_load(source)["/**"]["ros__parameters"]


def _runtime_parameters():
    return yaml.safe_load(RACE_RUNTIME_SOURCE)["race_runtime_manager"][
        "ros__parameters"
    ]


def test_vehicle_launch_uses_a_stable_overridable_chassis_identity():
    assert "/dev/autoracer_rc_chassis" in VEHICLE_LAUNCH_SOURCE
    assert 'LaunchConfiguration("serial_port")' in VEHICLE_LAUNCH_SOURCE
    assert '"serial_port": ParameterValue(serial_port, value_type=str)' in (
        VEHICLE_LAUNCH_SOURCE
    )
    assert "serial_port" not in _vehicle_parameters()
    assert "1a86" in UDEV_SOURCE
    assert "55d4" in UDEV_SOURCE
    assert "0003" in UDEV_SOURCE
    for unstable_name in ("/dev/ttyUSB", "/dev/ttyCH343", "/dev/wheeltec_"):
        assert unstable_name not in VEHICLE_LAUNCH_SOURCE
        assert unstable_name not in VEHICLE_CONFIG_SOURCE


def test_vehicle_parameters_match_frozen_firmware_and_confirmed_boundaries():
    params = _vehicle_parameters()
    assert params["baud_rate"] == 115200
    assert params["maximum_command_speed_mps"] == 3.0
    assert params["minimum_command_speed_mps"] == 0.3
    assert params["max_steering_tire_angle_rad"] == 0.349
    assert params["firmware_command_timeout_ms"] == 250
    assert params["emergency_status_timeout_ms"] == 250
    assert params["hall_feedback_acquisition_timeout_ms"] == 1500
    assert params["hall_feedback_loss_timeout_ms"] == 250
    assert params["base_frame_id"] == "base_link"
    assert "wheelbase_m" not in params


def test_vehicle_node_exposes_the_standard_control_and_status_contract():
    assert '"/control/command/control_cmd"' in VEHICLE_NODE_SOURCE
    assert '"/control/command/gear_cmd"' in VEHICLE_NODE_SOURCE
    assert '"/control/command/emergency_cmd"' in VEHICLE_NODE_SOURCE
    assert '"/vehicle/status/velocity_status"' in VEHICLE_NODE_SOURCE
    assert '"/vehicle/status/steering_status"' in VEHICLE_NODE_SOURCE
    assert '"/vehicle/status/gear_status"' in VEHICLE_NODE_SOURCE
    assert '"/vehicle/status/control_mode"' in VEHICLE_NODE_SOURCE
    assert '"/control/control_mode_request"' in VEHICLE_NODE_SOURCE
    assert "autoware_control_msgs::msg::Control" in VEHICLE_NODE_SOURCE
    assert "autoware_vehicle_msgs::msg::VelocityReport" in VEHICLE_NODE_SOURCE
    assert "autoware_vehicle_msgs::msg::SteeringReport" in VEHICLE_NODE_SOURCE
    assert "autoware_vehicle_msgs::msg::GearReport" in VEHICLE_NODE_SOURCE
    assert "autoware_vehicle_msgs::msg::ControlModeReport" in VEHICLE_NODE_SOURCE
    assert "autoware_vehicle_msgs::srv::ControlModeCommand" in VEHICLE_NODE_SOURCE
    assert "tier4_vehicle_msgs::msg::VehicleEmergencyStamped" in VEHICLE_NODE_SOURCE
    assert "logical_gear_allows_speed" in VEHICLE_NODE_SOURCE
    assert "autonomous_requested_" in VEHICLE_NODE_SOURCE
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
        "enable_drive_commands",
        "sensor_msgs::msg::Imu",
    )
    for token in forbidden:
        assert token not in VEHICLE_NODE_SOURCE


def test_formal_emergency_and_hall_feedback_fail_closed_contract():
    combined_state = VEHICLE_STATE_HEADER_SOURCE + VEHICLE_STATE_SOURCE
    assert "EmergencyStatusMonitor" in combined_state
    assert "fresh_and_clear" in combined_state
    assert "HallFeedbackMonitor" in combined_state
    assert "HallFeedbackDecision::kFault" in combined_state
    assert "loss_timeout_ms_" in combined_state
    assert "manual_stop_is_required" in combined_state

    assert "on_emergency_status" in VEHICLE_NODE_SOURCE
    assert "latch_safety_stop" in VEHICLE_NODE_SOURCE
    assert "safety_stop_latched_" in VEHICLE_NODE_SOURCE
    assert "hall_feedback_fault_latched_" not in VEHICLE_NODE_SOURCE
    assert "missing or stale formal emergency status" in VEHICLE_NODE_SOURCE
    assert "persistent Hall feedback loss" in VEHICLE_NODE_SOURCE
    assert "stop_frame=%s" in VEHICLE_NODE_SOURCE
    assert "velocity_report_is_publishable" in VEHICLE_NODE_SOURCE
    assert "VelocityFeedbackState::kMeasuredMotion" in combined_state
    assert "VelocityFeedbackState::kConfirmedStandstill" in combined_state
    assert "kRcStatusHallStandstillConfirmed" in VEHICLE_STATE_SOURCE
    assert "kConfirmedAutomaticStop" not in VEHICLE_STATE_SOURCE
    assert "tier4_vehicle_msgs" in ADAPTER_CMAKE_SOURCE
    assert "<depend>tier4_vehicle_msgs</depend>" in ADAPTER_PACKAGE_SOURCE


def test_standard_status_fields_keep_their_actual_sources_explicit():
    assert "feedback.hall_speed_command_signed_mps" in VEHICLE_NODE_SOURCE
    assert "feedback.yaw_rate_estimate_rad_s" in VEHICLE_NODE_SOURCE
    assert "feedback.steering_angle_estimate_rad" in VEHICLE_NODE_SOURCE
    assert "steering_estimate_is_valid(feedback)" in VEHICLE_NODE_SOURCE
    assert "gear_report.report = logical_gear_report_" in VEHICLE_NODE_SOURCE
    assert "reported_control_mode(feedback)" in VEHICLE_NODE_SOURCE

    assert "kRcStatusSteeringEstimateValid" in VEHICLE_STATE_SOURCE
    assert "kRcStatusRcOverrideActive" in VEHICLE_STATE_SOURCE
    assert "kRcStatusAutoEnabled" in VEHICLE_STATE_SOURCE
    assert "kRcStatusCommandTimeout" in VEHICLE_STATE_SOURCE
    assert "kRcStatusStopOverrideActive" in VEHICLE_STATE_SOURCE
    assert "kRcStatusHallStandstillConfirmed" in VEHICLE_STATE_SOURCE
    assert "kRcStatusSteeringIsMeasured" not in VEHICLE_STATE_SOURCE


def test_protocol_matches_frozen_ackermann_wire_contract():
    combined = PROTOCOL_HEADER_SOURCE + PROTOCOL_SOURCE
    assert "kRcCommandFrameSize = 11U" in PROTOCOL_HEADER_SOURCE
    assert "kRcFeedbackFrameSize = 24U" in PROTOCOL_HEADER_SOURCE
    assert "kRcAckermannCommandId = 0x01U" in PROTOCOL_HEADER_SOURCE
    assert "kRcTelemetryProtocolId = 0xA1U" in PROTOCOL_HEADER_SOURCE
    assert "kRcCommandFlagEnable = 0x01U" in PROTOCOL_HEADER_SOURCE
    assert "kRcCommandFlagSoftwareStop = 0x80U" in PROTOCOL_HEADER_SOURCE
    assert "kRcStatusHallStandstillConfirmed = 1UL << 12U" in PROTOCOL_HEADER_SOURCE
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


def test_rc_vehicle_info_matches_the_confirmed_geometry():
    params = _global_parameters(VEHICLE_INFO_SOURCE)
    assert params == {
        "wheel_radius": 0.115,
        "wheel_width": 0.100,
        "wheel_base": 0.600,
        "wheel_tread": 0.500,
        "front_overhang": 0.150,
        "rear_overhang": 0.130,
        "left_overhang": 0.050,
        "right_overhang": 0.050,
        "vehicle_height": 0.515,
        "max_steer_angle": 0.349,
    }
    assert round(
        params["wheel_base"]
        + params["front_overhang"]
        + params["rear_overhang"],
        3,
    ) == 0.880
    assert round(
        params["wheel_tread"]
        + params["left_overhang"]
        + params["right_overhang"],
        3,
    ) == 0.600


def test_rc_gate_uses_rc_speed_and_steering_boundaries():
    params = _global_parameters(VEHICLE_CMD_GATE_SOURCE)
    for profile_name in ("nominal", "on_transition"):
        profile = params[profile_name]
        assert profile["vel_lim"] == 3.0
        assert profile["steer_cmd_lim"] == [0.349] * 4
        assert profile["steer_cmd_diff_lim_from_current_steer"] == [0.349] * 4
        assert max(profile["steer_rate_lim_for_steer_cmd"]) <= 0.9
        assert profile["lon_acc_lim_for_lon_vel"] == [0.6] * 4
    assert "0.488" not in VEHICLE_CMD_GATE_SOURCE


def test_rc_runtime_grants_only_velocity_the_hall_acquisition_window():
    params = _runtime_parameters()
    vehicle_params = _vehicle_parameters()
    assert params["velocity_status_timeout_sec"] == 1.75
    assert params["vehicle_status_timeout_sec"] == 0.25
    assert params["velocity_status_timeout_sec"] > (
        vehicle_params["hall_feedback_acquisition_timeout_ms"] / 1000.0
    )


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
