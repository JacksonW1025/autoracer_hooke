import math
from dataclasses import asdict
import json

import pytest

from tools.recorded_course import (
    PoseSample,
    RecordedCourseConfig,
    build_recorded_course,
)
from tools.build_recorded_course import write_asset


def pose(stamp, x, y=0.0, z=0.0, yaw=0.0):
    return PoseSample(
        stamp=float(stamp),
        x=float(x),
        y=float(y),
        z=float(z),
        qx=0.0,
        qy=0.0,
        qz=math.sin(0.5 * yaw),
        qw=math.cos(0.5 * yaw),
    )


def config(**overrides):
    values = {
        "sample_interval_m": 0.5,
        "stationary_step_m": 0.02,
        "minimum_point_spacing_m": 0.01,
        "maximum_step_m": 2.0,
        "smoothing_radius": 1,
        "maximum_smoothing_displacement_m": 0.05,
        "max_speed_mps": 0.5,
        "max_lateral_accel_mps2": 0.4,
        "max_accel_mps2": 0.4,
        "max_decel_mps2": -0.8,
        "departure_speed_mps": 0.1,
        "left_offset_m": 0.4,
        "right_offset_m": 0.4,
    }
    values.update(overrides)
    return RecordedCourseConfig(**values)


def test_complete_single_direction_recording_is_resampled_without_route_selection():
    poses = [pose(0, 0), pose(1, 0), pose(2, 0)]
    poses += [pose(3 + i, 0.5 * i) for i in range(11)]
    poses += [pose(14, 5), pose(15, 5)]

    samples, report = build_recorded_course(poses, config(smoothing_radius=0))

    assert samples[0].s == 0.0
    assert samples[0].x == pytest.approx(0.0)
    assert samples[-1].x == pytest.approx(5.0)
    assert all(b.s > a.s for a, b in zip(samples, samples[1:]))
    assert report["internal_segments_removed"] == 0
    assert report["stationary_prefix_removed"] == 3
    assert report["stationary_suffix_removed"] == 2


def test_invalid_points_are_rejected_but_timestamp_reversal_fails():
    poses = [pose(0, 0), pose(1, 0.5), pose(2, float("nan")), pose(3, 1.0)]
    samples, report = build_recorded_course(poses, config(smoothing_radius=0))
    assert samples[-1].x == pytest.approx(1.0)
    assert report["invalid_points_removed"] == 1

    reversed_time = [pose(0, 0), pose(2, 0.5), pose(1, 1.0)]
    with pytest.raises(ValueError, match="timestamps are not strictly increasing"):
        build_recorded_course(reversed_time, config())


def test_invalid_quaternion_and_large_pose_jump_fail_closed():
    invalid = pose(1, 0.5)
    invalid = PoseSample(**{**invalid.__dict__, "qw": 0.0})
    with pytest.raises(ValueError, match="fewer than two usable poses"):
        build_recorded_course([pose(0, 0), invalid], config())

    with pytest.raises(ValueError, match="pose jump"):
        build_recorded_course(
            [pose(0, 0), pose(1, 0.5), pose(2, 10.0)],
            config(maximum_step_m=1.0),
        )


def test_resampling_interpolates_z_and_recomputes_forward_yaw():
    samples, _ = build_recorded_course(
        [pose(0, 0, 0, 0), pose(1, 1, 0, 1), pose(2, 2, 0, 2)],
        config(smoothing_radius=0),
    )
    assert [sample.s for sample in samples] == pytest.approx([0, 0.5, 1, 1.5, 2])
    assert [sample.z for sample in samples] == pytest.approx([0, 0.5, 1, 1.5, 2])
    assert all(sample.yaw == pytest.approx(0.0) for sample in samples)


def test_smoothing_is_bounded_and_reported():
    poses = [pose(i, i * 0.5, 0.03 if i % 2 else -0.03) for i in range(9)]
    _, report = build_recorded_course(poses, config())
    assert 0.0 < report["maximum_smoothing_displacement_m"] <= 0.05

    with pytest.raises(ValueError, match="smoothing displacement"):
        build_recorded_course(
            poses,
            config(maximum_smoothing_displacement_m=0.001),
        )


def test_speed_profile_obeys_rc_limits_and_stops_at_goal():
    poses = [pose(i, 0.25 * i, 0.2 * math.sin(0.25 * i)) for i in range(41)]
    samples, report = build_recorded_course(poses, config(smoothing_radius=0))

    assert samples[0].target_velocity <= 0.1 + 1e-9
    assert samples[-1].target_velocity == 0.0
    assert all(0.0 <= sample.target_velocity <= 0.5 for sample in samples)
    assert all(-0.8 - 1e-6 <= sample.target_acceleration <= 0.4 + 1e-6 for sample in samples)
    assert all(math.isfinite(sample.curvature) for sample in samples)
    assert report["terminal_stop"] is True


def test_asset_write_is_atomic_and_refuses_replacement(tmp_path):
    data_root = tmp_path / "data"
    source = {
        "source_bag": "bags/source",
        "odometry_bag": "replays/odom",
        "map_path": "maps/test_map",
        "super_lio_config": "runs/test/config.yaml",
        "source_frame": "world",
        "processing": asdict(config(smoothing_radius=0)),
    }
    for relative in (
        "bags/source/metadata.yaml",
        "replays/odom/metadata.yaml",
        "maps/test_map/pointcloud_map_metadata.yaml",
        "maps/test_map/map_projector_info.yaml",
        "runs/test/config.yaml",
    ):
        path = data_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(relative + "\n", encoding="utf-8")

    output = tmp_path / "course"
    manifest = write_asset(
        output,
        "test_map",
        source,
        data_root,
        [pose(0, 0), pose(1, 0.5), pose(2, 1.0)],
    )
    assert manifest["production_method"] == "rc_recorded_super_lio"
    assert manifest["validation"]["status"] == "PASS"
    assert (output / "course.csv").is_file()
    assert json.loads((output / "manifest.json").read_text())["map_id"] == "test_map"

    with pytest.raises(FileExistsError, match="refusing to replace"):
        write_asset(
            output,
            "test_map",
            source,
            data_root,
            [pose(0, 0), pose(1, 0.5)],
        )
