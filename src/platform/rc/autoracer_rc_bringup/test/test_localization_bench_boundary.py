from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BENCH_SOURCE = (
    PACKAGE_ROOT / "launch" / "localization_bench.launch.py"
).read_text(encoding="utf-8")
PACKAGE_SOURCE = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")


def test_bench_launch_composes_only_stage4_production_inputs_and_consumers():
    for token in (
        '_python_launch("autoracer_rc_bringup", "sensing.launch.py")',
        '_python_launch("autoracer_rc_bringup", "vehicle.launch.py")',
        '_python_launch("autoracer_localization", "localization.launch.py")',
        'package="autoracer_planning"',
        'executable="fixed_course_publisher"',
        '"trajectory_topic": "/planning/global_trajectory"',
    ):
        assert token in BENCH_SOURCE

    forbidden = (
        "local_trajectory_planner",
        "autoracer_control",
        "race_runtime_manager",
        "vehicle_cmd_gate",
        "race.launch",
        "/planning/trajectory",
        "/control/",
    )
    for token in forbidden:
        assert token not in BENCH_SOURCE


def test_bench_launch_is_wall_clock_uninitialized_and_online():
    assert 'SetParameter(name="use_sim_time", value=False)' in BENCH_SOURCE
    assert '"use_sim_time": "false"' in BENCH_SOURCE
    assert '"system_run_mode": "online"' in BENCH_SOURCE
    assert '"initial_pose": "[]"' in BENCH_SOURCE
    assert '"gnss_enabled": "false"' in BENCH_SOURCE
    assert '"input_pointcloud": "/sensing/lidar/concatenated/pointcloud"' in (
        BENCH_SOURCE
    )


def test_vehicle_feedback_is_opt_in_and_forced_receive_only():
    assert '"launch_vehicle_telemetry",\n                default_value="false"' in (
        BENCH_SOURCE
    )
    assert 'condition=IfCondition(launch_vehicle_telemetry)' in BENCH_SOURCE
    assert '"telemetry_only": "true"' in BENCH_SOURCE
    assert '"telemetry_only": "false"' not in BENCH_SOURCE


def test_bench_excludes_user_skipped_g90_from_runtime():
    assert "/dev/serial/by-id/" in BENCH_SOURCE
    assert "5AA6079369-if00" in BENCH_SOURCE
    assert '"launch_g90": "false"' in BENCH_SOURCE
    assert '"launch_g90_driver": "false"' in BENCH_SOURCE
    for unstable_name in ("/dev/ttyCH343", "/dev/ttyUSB", "/dev/wheeltec_"):
        assert unstable_name not in BENCH_SOURCE


def test_bench_runtime_dependencies_are_declared():
    assert "<exec_depend>autoracer_localization</exec_depend>" in PACKAGE_SOURCE
    assert "<exec_depend>autoracer_planning</exec_depend>" in PACKAGE_SOURCE
