from pathlib import Path


COMMAND_GATE = (
    Path(__file__).resolve().parents[1]
    / "autoracer_safety"
    / "command_gate.py"
)


def test_command_gate_exposes_optional_trajectory_freshness_guard():
    source = COMMAND_GATE.read_text(encoding="utf-8")

    assert 'declare_parameter("require_trajectory", False)' in source
    assert 'declare_parameter("trajectory_topic", "/planning/trajectory")' in source
    assert 'declare_parameter("trajectory_timeout_sec", 1.0)' in source
    assert "Trajectory" in source
    assert '"trajectory_timeout"' in source

