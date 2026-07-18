import csv
from dataclasses import replace
import json
import math
from pathlib import Path

import pytest

from autoracer_planning.fixed_course import (
    CourseBuildConfig,
    CourseSample,
    RoadExtentSample,
    _accelerations,
    apply_road_extents,
    build_asset,
    curvature_speed_envelope,
    holdout_errors,
    load_course_asset,
    load_course_csv,
    sha256_file,
    validate_course_map_contract,
)


def test_curvature_speed_envelope_uses_spatial_vehicle_window():
    envelope = curvature_speed_envelope(
        [0.01, -0.20, 0.0, 0.15, 0.01],
        [0.0, 0.5, 1.0, 1.5, 2.0],
        1.0,
    )

    assert envelope == pytest.approx([0.20, 0.20, 0.20, 0.15, 0.15])


def test_acceleration_is_assigned_to_outgoing_segment_at_speed_valley():
    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]

    accelerations = _accelerations(points, [2.0, 1.0, 2.0])

    assert accelerations == pytest.approx([-1.5, 1.5, 0.0])


def _course_sample(s, x, y):
    return CourseSample(
        s=s,
        x=x,
        y=y,
        z=0.0,
        yaw=0.0,
        curvature=0.0,
        left_offset=1.0,
        right_offset=1.0,
        target_velocity=1.0,
        target_acceleration=0.0,
    )


def test_local_holdout_projection_removes_only_longitudinal_phase_error():
    points = [
        (
            index * 0.5,
            0.5 * math.sin(2.0 * math.pi * index * 0.5 / 5.0)
            if index * 0.5 <= 50.0
            else 0.0,
        )
        for index in range(201)
    ]
    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        distances.append(
            distances[-1]
            + math.hypot(current[0] - previous[0], current[1] - previous[1])
        )
    course = [
        _course_sample(distance, point[0], point[1])
        for distance, point in zip(distances, points)
    ]
    holdout = [(index * 0.5, 0.0, 0.0) for index in range(201)]

    normalized = holdout_errors(course, holdout)
    projected = holdout_errors(
        course,
        holdout,
        alignment_method="local_path_projection",
        projection_window_m=10.0,
        projection_sample_interval_m=0.25,
    )

    assert max(normalized) > 1.0
    assert max(projected) <= 0.5 + 1e-9


def test_road_extents_create_conservative_functional_boundaries():
    samples = [
        CourseSample(
            s=float(index),
            x=float(index),
            y=0.0,
            z=0.0,
            yaw=0.0,
            curvature=0.0,
            left_offset=1.8,
            right_offset=1.8,
            target_velocity=1.0,
            target_acceleration=0.0,
        )
        for index in range(2)
    ]
    extents = [
        RoadExtentSample(
            index, float(index), float(index), 0.0, 0.0, True, 1.4, 1.65
        )
        for index in range(2)
    ]

    bounded, metrics = apply_road_extents(samples, extents, CourseBuildConfig())

    assert bounded[0].left_offset == pytest.approx(1.2)
    assert bounded[0].right_offset == pytest.approx(1.45)
    assert metrics["road_eval_minimum_outward_clearance_m"] == pytest.approx(0.2)
    assert metrics["vehicle_footprint_minimum_margin_m"] == pytest.approx(0.2995)


def _write_vehicle_state(path: Path, lateral_offset: float):
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("stamp", "x", "y", "z", "roll", "pitch", "yaw"))
        for index in range(121):
            x = index * 0.1
            writer.writerow((index * 0.1, x, lateral_offset, 0.3, 0.0, 0.0, 0.0))


def _write_progress(path: Path):
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "sim_time",
                "road_s",
                "lateral_t",
                "distance",
                "speed",
                "on_road",
                "roadeval_ok",
                "link_s",
                "link_t",
            )
        )
        for index in range(13):
            writer.writerow((index, index, 0.0, index, 1.0, 1, 1, index, 0.0))


def test_build_asset_uses_continuous_gt_and_independent_holdout(tmp_path):
    build_dir = tmp_path / "build"
    holdout_dir = tmp_path / "holdout"
    build_dir.mkdir()
    holdout_dir.mkdir()
    build_vehicle = build_dir / "vehicle_state.csv"
    build_progress = build_dir / "route_progress.csv"
    holdout_vehicle = holdout_dir / "vehicle_state.csv"
    holdout_progress = holdout_dir / "route_progress.csv"
    _write_vehicle_state(build_vehicle, 0.0)
    _write_progress(build_progress)
    _write_vehicle_state(holdout_vehicle, 0.1)
    _write_progress(holdout_progress)
    testrun = tmp_path / "TestRun"
    testrun.write_text("route 271\n", encoding="ascii")
    map_path = tmp_path / "test_map"
    map_path.mkdir()
    for filename in (
        "input_contract.json",
        "pointcloud_map_metadata.yaml",
        "map_projector_info.yaml",
    ):
        (map_path / filename).write_text(filename + "\n", encoding="ascii")
    tile = map_path / "pointcloud_map_x0_y0.pcd"
    tile.write_bytes(b"pcd\n")
    release = {
        "status": "PASS",
        "tiled_map": {
            "tile_count": 1,
            "metadata": {
                "path": "pointcloud_map_metadata.yaml",
                "size_bytes": (map_path / "pointcloud_map_metadata.yaml").stat().st_size,
                "sha256": sha256_file(map_path / "pointcloud_map_metadata.yaml"),
            },
            "tiles": [
                {
                    "path": tile.name,
                    "size_bytes": tile.stat().st_size,
                    "sha256": sha256_file(tile),
                }
            ],
        },
    }
    (map_path / "release_manifest.json").write_text(
        json.dumps(release), encoding="ascii"
    )
    config = CourseBuildConfig(
        map_id="test_map",
        carmaker_route_length_m=10.0,
        sample_interval_m=0.5,
        max_smoothing_offset_m=0.01,
        holdout_p95_limit_m=0.2,
        holdout_max_limit_m=0.2,
        require_roadeval_boundaries=False,
    )
    preview = tmp_path / "course_preview"
    build_asset(
        build_vehicle,
        build_progress,
        holdout_vehicle,
        holdout_progress,
        testrun,
        map_path,
        preview,
        config,
    )
    preview_samples = load_course_csv(preview / "course.csv")
    road_file = tmp_path / "road.rd5"
    road_file.write_text("synthetic road\n", encoding="ascii")
    road_extents = tmp_path / "road_extents.csv"
    with road_extents.open("w", encoding="ascii", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "index",
                "s",
                "x",
                "y",
                "yaw",
                "center_on_road",
                "left_road_extent",
                "right_road_extent",
            )
        )
        for index, sample in enumerate(preview_samples):
            writer.writerow(
                (index, sample.s, sample.x, sample.y, sample.yaw, 1, 2.0, 2.0)
            )

    output = tmp_path / "course"
    manifest = build_asset(
        build_vehicle,
        build_progress,
        holdout_vehicle,
        holdout_progress,
        testrun,
        map_path,
        output,
        replace(config, require_roadeval_boundaries=True),
        road_file,
        road_extents,
    )
    loaded_manifest, samples = load_course_asset(output)

    assert manifest["validation"]["status"] == "PASS"
    validate_course_map_contract(loaded_manifest, map_path)
    assert loaded_manifest["assets"]["course.csv"]["rows"] == len(samples)
    assert abs(samples[-1].s - 10.0) < 1e-6
    assert samples[-1].target_velocity == 0.0
    assert abs(samples[-1].yaw) < 1e-9
    assert manifest["validation"]["metrics"]["holdout_xy_p95_m"] < 0.11

    (map_path / "input_contract.json").write_text("changed\n", encoding="ascii")
    with pytest.raises(ValueError, match="input_contract.json"):
        validate_course_map_contract(loaded_manifest, map_path)

    (map_path / "input_contract.json").write_text(
        "input_contract.json\n", encoding="ascii"
    )
    tile.write_bytes(b"tampered\n")
    with pytest.raises(ValueError, match="pointcloud tile"):
        validate_course_map_contract(loaded_manifest, map_path)
