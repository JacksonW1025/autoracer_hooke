from pathlib import Path


def test_localization_launch_passes_pointcloud_map_directory():
    launch_file = Path("src/autoracer_bringup/launch/localization.launch.py")
    text = launch_file.read_text(encoding="utf-8")

    assert 'PathJoinSubstitution([map_path, "pointcloud_map.pcd"])' not in text
    assert "[[map_path]]" in text
    assert "pcd_paths_or_directory" in text
