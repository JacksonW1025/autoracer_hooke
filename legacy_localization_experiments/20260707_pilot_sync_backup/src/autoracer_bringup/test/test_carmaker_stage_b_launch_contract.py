from pathlib import Path


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "carmaker_stage_b.launch.py"
)


def test_stage_b_launch_uses_local_planner_and_canonical_control_topic():
    launch_py = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "ground_truth_localization_relay" in launch_py
    assert '"/carmaker/ground_truth/pose"' in launch_py
    assert '"/localization/pose_with_covariance"' in launch_py
    assert "route_goal_publisher" in launch_py
    assert "lanelet_route_planner" in launch_py
    assert '"/planning/global_trajectory"' in launch_py
    assert "local_trajectory_planner" in launch_py
    assert '"/planning/trajectory"' in launch_py
    assert '"/control/command/control_cmd"' in launch_py
    assert '"require_trajectory": True' in launch_py
    assert '"trajectory_timeout_sec": 1.0' in launch_py
    assert "carmaker_trajectory_provider" not in launch_py
