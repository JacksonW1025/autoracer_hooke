from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOCAL_PLANNER = ROOT / "src" / "autoracer_planning" / "autoracer_planning" / "local_trajectory_planner.py"
PURE_PURSUIT = ROOT / "src" / "autoracer_control" / "autoracer_control" / "pure_pursuit_controller.py"


def test_local_trajectory_planner_exposes_stage_b_diagnostics():
    source = LOCAL_PLANNER.read_text(encoding="utf-8")

    assert 'declare_parameter("diagnostic_log_period_sec", 1.0)' in source
    assert "nearest_index=" in source
    assert "trajectory_points=" in source
    assert "speed_range_mps=" in source


def test_pure_pursuit_controller_exposes_stage_b_diagnostics():
    source = PURE_PURSUIT.read_text(encoding="utf-8")

    assert 'declare_parameter("diagnostic_log_period_sec", 1.0)' in source
    assert "target_index=" in source
    assert "target_speed_mps=" in source
    assert "steer_rad=" in source
