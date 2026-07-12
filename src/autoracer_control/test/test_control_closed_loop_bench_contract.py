from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / "launch" / "control_closed_loop_bench.launch.py"
FIXTURE = PACKAGE_ROOT / "autoracer_control" / "control_closed_loop_fixture_publisher.py"
PLANT = PACKAGE_ROOT / "autoracer_control" / "virtual_chassis_node.py"
MONITOR = PACKAGE_ROOT / "autoracer_control" / "control_closed_loop_monitor.py"
SCENARIOS = PACKAGE_ROOT / "autoracer_control" / "control_closed_loop_scenarios.py"
SETUP = PACKAGE_ROOT / "setup.py"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"

ALLOWED_CONTROL_BENCH_TOPICS = {
    "/control_bench/planning/trajectory",
    "/control_bench/system/operation_mode/state",
    "/control_bench/localization/kinematic_state",
    "/control_bench/vehicle/status/steering_status",
    "/control_bench/localization/acceleration",
    "/control_bench/autoracer/control/raw_control_cmd",
}


def test_closed_loop_launch_wires_real_controller_fixture_plant_and_monitor():
    source = LAUNCH.read_text(encoding="utf-8")

    for required in (
        "race_control.launch.py",
        "control_closed_loop_fixture_publisher",
        "virtual_chassis_node",
        "control_closed_loop_monitor",
        "race_controller.closed_loop_candidate.param.yaml",
        "RegisterEventHandler",
        "OnProcessExit",
        "scenario_type",
        "max_duration_sec",
        "completion_threshold",
        'DeclareLaunchArgument("use_sim_time", default_value="false")',
        '"use_sim_time": use_sim_time',
    ):
        assert required in source

    monitor_node_section = source[
        source.index("monitor_node = Node(") : source.index("parameters=[", source.index("monitor_node = Node("))
    ]
    assert "on_exit=[Shutdown(" in monitor_node_section

    assert "command_gate" not in source
    assert "carmaker" not in source.lower()
    assert "/control_bench/localization/pose_with_covariance" not in source
    assert "/control_bench/control/command/control_cmd" not in source


def test_closed_loop_launch_uses_only_allowed_main_control_bench_topics():
    combined_source = "\n".join(
        path.read_text(encoding="utf-8") for path in (LAUNCH, FIXTURE, PLANT, MONITOR)
    )

    discovered_topics = {
        token.strip("\"',")
        for token in combined_source.replace("(", " ").replace(")", " ").split()
        if token.strip("\"',").startswith("/control_bench/")
    }

    assert discovered_topics <= ALLOWED_CONTROL_BENCH_TOPICS
    assert ALLOWED_CONTROL_BENCH_TOPICS <= discovered_topics


def test_virtual_chassis_consumes_raw_control_and_never_final_control():
    source = PLANT.read_text(encoding="utf-8")

    assert "/control_bench/autoracer/control/raw_control_cmd" in source
    assert "create_subscription" in source
    assert "final_control" not in source
    assert "/control/command/control_cmd" not in source
    assert "/control_bench/control/command/control_cmd" not in source


def test_monitor_writes_required_closed_loop_summary_contract():
    source = MONITOR.read_text(encoding="utf-8")

    for required in (
        "closed_loop_summary.json",
        "control_closed_loop_tuning_ros_only",
        "autoware_trajectory_follower_node/controller_node_exe",
        "virtual_chassis",
        "does_not_validate",
        "CarMaker closed-loop",
        "Stage B planner",
        "real vehicle calibration",
        "race performance",
        "rms_lateral_error_m",
        "rms_velocity_error_mps",
        "max_estimated_lat_acc_mps2",
        "oscillation_score",
        "scenario_type",
        "path_length_m",
        "progress_distance_m",
        "trajectory_progress_ratio",
        "completed_trajectory",
        "end_condition",
        "max_duration_sec",
        "completion_threshold",
        "segments",
    ):
        assert required in source


def test_monitor_writes_per_sample_trace_for_root_cause_analysis():
    source = MONITOR.read_text(encoding="utf-8")

    for required in (
        "closed_loop_trace.jsonl",
        "nearest_idx",
        "lateral_error_m",
        "heading_error_rad",
        "reference_x_m",
        "reference_y_m",
        "vehicle_x_m",
        "vehicle_y_m",
        "reference_curvature_1pm",
        "progress_distance_m",
        "trajectory_progress_ratio",
        "nearest_segment_idx",
    ):
        assert required in source


def test_v15_full_validation_scenarios_are_declared_without_yaml_dsl():
    combined_source = "\n".join(
        path.read_text(encoding="utf-8") for path in (LAUNCH, FIXTURE, MONITOR, SCENARIOS)
    )

    for scenario in (
        "straight_120m_v1",
        "arc_r20_90deg_v1",
        "s_curve_100m_v1",
        "speed_step_120m_v1",
    ):
        assert scenario in combined_source

    assert "full_validation" in combined_source
    assert "ScenarioSpec" in combined_source
    assert "SegmentSpec" in combined_source
    assert ".yaml" not in FIXTURE.read_text(encoding="utf-8")


def test_package_declares_closed_loop_assets_without_custom_interfaces():
    setup_source = SETUP.read_text(encoding="utf-8")
    package_source = PACKAGE_XML.read_text(encoding="utf-8")

    for asset in (
        "control_closed_loop_bench.launch.py",
        "race_controller.closed_loop_candidate.param.yaml",
    ):
        assert asset in setup_source

    for entry_point in (
        "control_closed_loop_fixture_publisher",
        "virtual_chassis_node",
        "control_closed_loop_monitor",
    ):
        assert entry_point in setup_source

    for forbidden in (
        "<member_of_group>rosidl_interface_packages</member_of_group>",
        "rosidl_default_generators",
        "message_generation",
        "<build_depend>rosidl",
    ):
        assert forbidden not in package_source

    for interface_dir in ("msg", "srv", "action"):
        assert not (PACKAGE_ROOT / interface_dir).exists()
