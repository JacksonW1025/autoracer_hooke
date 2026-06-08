from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BENCH_LAUNCH = PACKAGE_ROOT / "launch" / "race_control_bench.launch.py"
FIXTURE = PACKAGE_ROOT / "autoracer_control" / "race_bench_fixture_publisher.py"
MONITOR = PACKAGE_ROOT / "autoracer_control" / "race_bench_monitor.py"
SAFETY_PACKAGE_XML = PACKAGE_ROOT.parents[0] / "autoracer_safety" / "package.xml"


def test_bench_launch_wires_controller_fixture_gate_and_monitor_to_control_bench():
    source = BENCH_LAUNCH.read_text(encoding="utf-8")

    for required_node in (
        "race_bench_fixture_publisher",
        "race_control.launch.py",
        "command_gate",
        "race_bench_monitor",
    ):
        assert required_node in source

    for topic in (
        "/control_bench/planning/trajectory",
        "/control_bench/localization/kinematic_state",
        "/control_bench/vehicle/status/steering_status",
        "/control_bench/localization/acceleration",
        "/control_bench/system/operation_mode/state",
        "/control_bench/autoracer/control/raw_control_cmd",
        "/control_bench/control/command/control_cmd",
        "/control_bench/control/command/gear_cmd",
        "/control_bench/control/command/hazard_lights_cmd",
        "/control_bench/control/command/turn_indicators_cmd",
        "/control_bench/autoracer/safety/state",
    ):
        assert topic in source

    assert '"enable_drive_commands": True' in source
    assert '"require_trajectory": True' in source
    assert "\"output_topic\": \"/control/command/control_cmd\"" not in source
    assert "pure_pursuit_controller" not in source


def test_fixture_publishes_all_required_synthetic_inputs_and_scenarios():
    source = FIXTURE.read_text(encoding="utf-8")

    for message_type in (
        "Trajectory",
        "Odometry",
        "SteeringReport",
        "AccelWithCovarianceStamped",
        "OperationModeState",
        "PoseWithCovarianceStamped",
        "TransformStamped",
    ):
        assert message_type in source

    for topic in (
        "/control_bench/planning/trajectory",
        "/control_bench/localization/kinematic_state",
        "/control_bench/vehicle/status/steering_status",
        "/control_bench/localization/acceleration",
        "/control_bench/system/operation_mode/state",
        "/control_bench/localization/pose_with_covariance",
    ):
        assert topic in source

    assert "transient_local" in source
    assert "OperationModeState.AUTONOMOUS" in source
    assert "is_autoware_control_enabled = True" in source

    for scenario in (
        "straight",
        "left_curve",
        "right_curve",
        "current_speed_low",
        "current_speed_high",
        "missing_trajectory",
        "missing_odometry",
        "missing_steering",
        "missing_acceleration",
        "missing_operation_mode",
        "stale_pose",
        "raw_timeout",
    ):
        assert scenario in source


def test_monitor_summary_contract_and_no_stage_b_claims():
    source = MONITOR.read_text(encoding="utf-8")

    for field in (
        "race_control_bench_ros_only",
        "controller_under_test",
        "autoware_trajectory_follower_node/controller_node_exe",
        "lateral_controller_mode",
        "longitudinal_controller_mode",
        "pure_pursuit_started",
        "raw_control_publisher_count",
        "final_control_publisher_count",
        "default_final_topic_publisher_count",
        "operation_mode_qos",
        "namespace_only_isolation_sufficient",
        "does_not_validate",
        "CarMaker closed-loop",
        "Stage B planner",
        "real vehicle calibration",
        "race performance",
    ):
        assert field in source

    assert "/control/command/control_cmd" in source
    assert "/control_bench/control/command/control_cmd" in source
    assert "pure_pursuit_controller" not in source


def test_autoracer_safety_declares_command_gate_planning_dependency():
    source = SAFETY_PACKAGE_XML.read_text(encoding="utf-8")

    assert "<exec_depend>autoware_planning_msgs</exec_depend>" in source
