from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from autoracer_safety.race_contract import TimedInput
from autoracer_safety.race_runtime_manager import RuntimePhase, desired_gear_command

from autoware_vehicle_msgs.msg import GearCommand


def _stamp(seconds: float):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int((seconds - whole) * 1.0e9))


def test_timed_input_rejects_future_and_out_of_order_commands():
    tracker = TimedInput()
    assert tracker.update("first", 10.0, _stamp(10.0))
    assert not tracker.update("old", 10.1, _stamp(9.9))
    assert not tracker.update("future", 10.1, _stamp(10.2))
    assert tracker.message == "first"
    assert tracker.rejected_out_of_order == 1
    assert tracker.rejected_future == 1
    assert tracker.fresh(10.19, 0.20)
    assert not tracker.fresh(10.21, 0.20)

    delayed = TimedInput()
    assert delayed.update("delayed", 10.19, _stamp(10.0))
    assert not delayed.fresh(10.21, 0.20)
    assert delayed.fresh(10.21, 0.20, 0.25)
    # A distinct source-age budget must not weaken delivery-loss detection.
    assert not delayed.fresh(10.40, 0.20, 0.50)


def test_timed_input_update_and_fresh_are_safe_for_concurrent_executor_callbacks():
    tracker = TimedInput()
    writer_done = threading.Event()
    reader_failures = []

    def writer():
        for index in range(1, 2001):
            stamp = 10.0 + index * 0.001
            assert tracker.update(index, stamp, _stamp(stamp))
        writer_done.set()

    def reader():
        while not writer_done.is_set():
            try:
                tracker.fresh(12.1, 3.0)
            except Exception as error:  # pragma: no cover - assertion witness
                reader_failures.append(error)
                return

    writer_thread = threading.Thread(target=writer)
    reader_thread = threading.Thread(target=reader)
    writer_thread.start()
    reader_thread.start()
    writer_thread.join(timeout=2.0)
    reader_thread.join(timeout=2.0)

    assert not writer_thread.is_alive()
    assert not reader_thread.is_alive()
    assert reader_failures == []
    assert tracker.sequence == 2000
    assert tracker.message == 2000
    assert tracker.source_stamp_sec == pytest.approx(12.0)


def test_gate_safe_profile_has_complete_equal_length_filter_arrays():
    path = PACKAGE / "config/race/vehicle_cmd_gate.safe.param.yaml"
    params = yaml.safe_load(path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
    assert params["use_emergency_handling"] is True
    assert params["check_external_emergency_heartbeat"] is False
    for profile_name in ("nominal", "on_transition"):
        profile = params[profile_name]
        count = len(profile["reference_speed_points"])
        assert profile["vel_lim"] == pytest.approx(100.0)
        for key in (
            "steer_cmd_lim",
            "steer_rate_lim_for_steer_cmd",
            "lon_acc_lim_for_lon_vel",
            "lon_jerk_lim_for_lon_acc",
            "lat_acc_lim_for_steer_cmd",
            "lat_jerk_lim_for_steer_cmd",
            "steer_cmd_diff_lim_from_current_steer",
        ):
            assert len(profile[key]) == count


def test_production_launch_uses_autoware_gate_and_not_legacy_command_gate():
    source = (PACKAGE / "launch/race_safety.launch.py").read_text(encoding="utf-8")
    assert 'package="autoware_vehicle_cmd_gate"' in source
    assert 'executable="vehicle_cmd_gate_exe"' in source
    assert 'executable="command_gate"' not in source
    assert 'executable="race_runtime_manager"' in source
    assert 'executable="race_guard"' not in source
    assert 'executable="race_supervisor"' not in source
    assert '"/control/command/control_cmd"' in source
    assert '"/control/command/emergency_cmd"' in source


def test_stopping_keeps_drive_while_moving_and_selects_park_at_rest():
    assert desired_gear_command(RuntimePhase.ACTIVE, 0.0, 0.10) == GearCommand.DRIVE
    assert desired_gear_command(RuntimePhase.STOPPING, 0.20, 0.10) == GearCommand.DRIVE
    assert desired_gear_command(RuntimePhase.STOPPING, 0.05, 0.10) == GearCommand.PARK
    assert desired_gear_command(RuntimePhase.FAULT, 5.0, 0.10) == GearCommand.DRIVE
    assert desired_gear_command(RuntimePhase.FAULT, 0.0, 0.10) == GearCommand.PARK
    assert desired_gear_command(RuntimePhase.FINISHED, 0.0, 0.10) == GearCommand.PARK


def test_runtime_manager_is_single_state_and_mrm_owner():
    source = (
        PACKAGE / "autoracer_safety/race_runtime_manager.py"
    ).read_text(encoding="utf-8")
    assert '"/system/race_runtime/state"' in source
    assert '"/system/fail_safe/mrm_state"' in source
    assert '"/system/emergency/control_cmd"' in source
    assert "guard_timeout_sec" not in source
    assert "/system/race_guard/state" not in source
    assert "/system/race_supervisor/state" not in source


def test_runtime_manager_can_require_fresh_raw_ndt_without_adding_an_estimator():
    source = (
        PACKAGE / "autoracer_safety/race_runtime_manager.py"
    ).read_text(encoding="utf-8")
    default_params = yaml.safe_load(
        (PACKAGE / "config/race/race_runtime.safe.param.yaml").read_text(
            encoding="utf-8"
        )
    )["race_runtime_manager"]["ros__parameters"]

    assert '"require_ndt_pose_health": False' in source
    assert '"ndt_pose_timeout_sec": 0.50' in source
    assert '"ndt_pose_source_timeout_sec": 0.50' in source
    assert '"/localization/pose_estimator/ndt_scan_matcher/pose_with_covariance"' in source
    assert 'return "NDT_POSE_STALE"' in source
    assert '"ndt_pose_fresh"' in source
    assert default_params["require_ndt_pose_health"] is False
    assert default_params["ndt_pose_timeout_sec"] == pytest.approx(0.50)
    assert default_params["ndt_pose_source_timeout_sec"] == pytest.approx(0.50)


def test_runtime_manager_logs_machine_readable_stale_input_evidence_without_debounce():
    source = (
        PACKAGE / "autoracer_safety/race_runtime_manager.py"
    ).read_text(encoding="utf-8")

    assert "def _stale_input_witness" in source
    assert '"receipt_timeout_sec": receipt_timeout' in source
    assert '"source_timeout_sec": source_timeout' in source
    assert '"ndt_pose_source_timeout_sec"' in source
    assert '"source_age_sec": now - tracker.source_stamp_sec' in source
    assert '"receipt_age_sec": now - tracker.receipt_sec' in source
    assert "race runtime stale-input witness" in source
    assert "race runtime stale-input witness instrumentation=V1" in source
    assert "debounce" not in source.split("def _stale_input_witness", 1)[1]


def test_runtime_manager_drains_inputs_separately_from_serialized_watchdog_state():
    source = (
        PACKAGE / "autoracer_safety/race_runtime_manager.py"
    ).read_text(encoding="utf-8")

    assert "MutuallyExclusiveCallbackGroup" in source
    assert "self._input_callback_group = ReentrantCallbackGroup()" in source
    assert "self._state_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert source.count("self.create_subscription(") == source.count(
        "callback_group=self._input_callback_group"
    )
    assert "callback_group=self._state_callback_group" in source
    assert "executor = MultiThreadedExecutor(num_threads=4)" in source
    assert "executor.add_node(node)" in source
    assert "executor.spin()" in source
    assert "rclpy.spin(node)" not in source
    assert "executor.shutdown()" in source


def test_planner_has_no_dead_runtime_guard_channel():
    planner_package = PACKAGE.parent / "autoracer_planning"
    source = (
        planner_package / "autoracer_planning/local_trajectory_planner.py"
    ).read_text(encoding="utf-8")
    manifest = (planner_package / "package.xml").read_text(encoding="utf-8")
    assert "/planning/race_guard/" not in source
    assert "VelocityLimitClearCommand" not in source
    assert "autoware_internal_planning_msgs" not in manifest
