from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[4]
INSPECT = ROOT / "tools" / "mapping" / "inspect_bag_topics.sh"
REPLAY = ROOT / "tools" / "mapping" / "run_super_lio_course_replay.sh"


def test_mapping_topic_inspection_selects_legacy_rc_inputs():
    text = INSPECT.read_text(encoding="utf-8")

    assert "/sensing/lidar/concatenated/pointcloud" in text
    assert "/imu/data" in text
    assert "Count: [1-9][0-9]*" in text
    assert "OUTPUT_ENV_FILE" in text


def test_replay_records_super_lio_odometry_without_mutating_source_maps():
    text = REPLAY.read_text(encoding="utf-8")

    assert '[[ ! -e "${REPLAY_DIR}" ]]' in text
    assert 'ros2 bag record -o "${ODOM_BAG}" /lio/odom' in text
    assert 'ros2 bag info "${ODOM_BAG}"' in text
    assert "setsid" in text
    assert "kill -INT" in text
    assert "map.pcd" not in text
    assert "rm -f" not in text
    assert "rm -rf" not in text


def test_mapping_scripts_are_shell_syntax_valid():
    for path in (INSPECT, REPLAY):
        result = subprocess.run(
            ["bash", "-n", str(path)], text=True, capture_output=True, check=False
        )
        assert result.returncode == 0, result.stderr
