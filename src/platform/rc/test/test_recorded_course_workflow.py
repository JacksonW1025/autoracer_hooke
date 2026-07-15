from pathlib import Path
import json
import os
import subprocess


ROOT = Path(__file__).resolve().parents[4]
INSPECT = ROOT / "tools" / "mapping" / "inspect_bag_topics.sh"
REPLAY = ROOT / "tools" / "mapping" / "run_super_lio_course_replay.sh"
SOURCE_FILE = ROOT / "src" / "platform" / "rc" / "tools" / "recorded_course_sources.json"
BUILD_CLI = ROOT / "src" / "platform" / "rc" / "tools" / "build_recorded_course.py"


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


def test_source_descriptors_pair_each_recording_with_its_own_map():
    sources = json.loads(SOURCE_FILE.read_text(encoding="utf-8"))
    assert set(sources) == {f"floor1_mapping_{index}" for index in range(101, 105)}
    for map_id, source in sources.items():
        assert source["source_bag"] == f"rc_mapping_data/bags/raw/{map_id}"
        assert source["odometry_bag"] == f"rc_mapping_data/course_replays/{map_id}/lio_odom"
        assert source["map_path"] == f"rc_mapping_data/autoware_maps/{map_id}"
        assert source["super_lio_config"] == f"rc_mapping_data/runs/{map_id}/rc_c32_super_lio.yaml"
        assert source["source_frame"] == "world"
        assert "lanelet" not in json.dumps(source).lower()


def test_build_cli_is_atomic_and_refuses_replacement():
    text = BUILD_CLI.read_text(encoding="utf-8")
    assert "rosbag2_py.SequentialReader" in text
    assert 'topic_type != "nav_msgs/msg/Odometry"' in text
    assert "output_dir.exists()" in text
    assert "temporary.replace(output_dir)" in text
    assert "sha256" in text
    assert "lanelet" not in text.lower()

    result = subprocess.run(
        ["python3", str(BUILD_CLI), "--help"],
        cwd=ROOT,
        env={
            **os.environ,
            "PYTHONPATH": str(ROOT / "src" / "core" / "autoracer_planning"),
        },
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
