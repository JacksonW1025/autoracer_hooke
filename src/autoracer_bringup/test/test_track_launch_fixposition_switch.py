from pathlib import Path


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "track.launch.py"


def test_track_launch_can_disable_real_fixposition_driver_for_carmaker_sim():
    launch_py = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'LaunchConfiguration("launch_fixposition")' in launch_py
    assert 'DeclareLaunchArgument("launch_fixposition", default_value="true")' in launch_py
    assert '"launch_fixposition": launch_fixposition' in launch_py
