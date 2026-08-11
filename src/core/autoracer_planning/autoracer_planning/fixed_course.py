from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
from typing import Iterable, Sequence


COURSE_COLUMNS = (
    "s",
    "x",
    "y",
    "z",
    "yaw",
    "curvature",
    "left_offset",
    "right_offset",
    "target_velocity",
    "target_acceleration",
)
MAP_CONTRACT_FILES = {
    "input_contract_sha256": "input_contract.json",
    "pointcloud_map_metadata_sha256": "pointcloud_map_metadata.yaml",
    "release_manifest_sha256": "release_manifest.json",
    "map_projector_info_sha256": "map_projector_info.yaml",
}
ROAD_EXTENT_COLUMNS = (
    "index",
    "s",
    "x",
    "y",
    "yaw",
    "center_on_road",
    "left_road_extent",
    "right_road_extent",
)


@dataclass(frozen=True)
class RawState:
    stamp: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ProgressState:
    stamp: float
    distance: float
    on_road: bool
    road_eval_ok: bool


@dataclass(frozen=True)
class CourseSample:
    s: float
    x: float
    y: float
    z: float
    yaw: float
    curvature: float
    left_offset: float
    right_offset: float
    target_velocity: float
    target_acceleration: float


@dataclass(frozen=True)
class RoadExtentSample:
    index: int
    s: float
    x: float
    y: float
    yaw: float
    center_on_road: bool
    left_road_extent: float
    right_road_extent: float


@dataclass(frozen=True)
class CourseBuildConfig:
    route_id: str = "271"
    map_id: str = "urbanroad_route271_20260710"
    frame_id: str = "map"
    version: str = "fixed_course_candidate"
    carmaker_route_length_m: float = 10803.772
    sample_interval_m: float = 0.5
    min_raw_point_distance_m: float = 0.02
    # Collection runs hold the vehicle still while sensors and localization
    # become ready.  Tiny tyre/solver motion during that hold is not a route
    # tangent and must not become the first controller segment.  Zero keeps
    # the historical behaviour; qualified cross-scene builds opt in to a
    # small, progress-defined trim.
    startup_trim_distance_m: float = 0.0
    smoothing_radius: int = 2
    max_smoothing_offset_m: float = 0.05
    left_offset_m: float = 1.8
    right_offset_m: float = 1.8
    road_edge_margin_m: float = 0.2
    vehicle_width_m: float = 1.801
    minimum_footprint_margin_m: float = 0.2
    # Optional, deterministic road-corridor centering.  Zero preserves every
    # historical course byte-for-byte.  When enabled, the construction path
    # may move only laterally, within measured RoadEval extents, to move away
    # from a locally narrow edge.  The shift is smoothed and rate limited so
    # that it cannot create a new steering discontinuity.
    road_corridor_centering_max_shift_m: float = 0.0
    road_corridor_centering_smoothing_radius: int = 20
    road_corridor_centering_max_step_m: float = 0.0125
    require_roadeval_boundaries: bool = True
    max_speed_mps: float = 5.0
    max_lateral_accel_mps2: float = 1.5
    # Optional vehicle/controller trackability limit.  A zero value preserves
    # all historical profiles.  When enabled, v * |curvature| is bounded so a
    # fixed course cannot demand a yaw rate beyond the validated closed-loop
    # bandwidth even when its centripetal-acceleration limit is satisfied.
    max_course_yaw_rate_radps: float = 0.0
    max_accel_mps2: float = 0.8
    max_decel_mps2: float = -1.5
    holdout_p95_limit_m: float = 0.75
    holdout_max_limit_m: float = 1.0
    holdout_alignment_method: str = "normalized_arc_length"
    holdout_projection_window_m: float = 10.0
    holdout_projection_sample_interval_m: float = 0.25


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_vehicle_states(path: Path) -> list[RawState]:
    rows: list[RawState] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            values = tuple(float(row[name]) for name in ("stamp", "x", "y", "z"))
            if all(math.isfinite(value) for value in values):
                rows.append(RawState(*values))
    if len(rows) < 2:
        raise ValueError(f"vehicle state input has fewer than two finite rows: {path}")
    return rows


def load_progress_states(path: Path) -> list[ProgressState]:
    rows: list[ProgressState] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            stamp = float(row["sim_time"])
            distance = float(row["distance"])
            if not math.isfinite(stamp) or not math.isfinite(distance):
                continue
            rows.append(
                ProgressState(
                    stamp=stamp,
                    distance=distance,
                    on_road=int(row["on_road"]) == 1,
                    road_eval_ok=int(row["roadeval_ok"]) == 1,
                )
            )
    if len(rows) < 2:
        raise ValueError(f"route progress input has fewer than two finite rows: {path}")
    return rows


def cutoff_time_for_distance(rows: Sequence[ProgressState], distance_m: float) -> float:
    for previous, current in zip(rows, rows[1:]):
        if current.distance < distance_m:
            continue
        delta = current.distance - previous.distance
        ratio = 0.0 if delta <= 1e-9 else (distance_m - previous.distance) / delta
        ratio = min(1.0, max(0.0, ratio))
        return previous.stamp + ratio * (current.stamp - previous.stamp)
    raise ValueError(
        f"route progress ends at {rows[-1].distance:.3f} m before {distance_m:.3f} m"
    )


def extract_continuous_path(
    states: Sequence[RawState],
    cutoff_time: float,
    min_point_distance_m: float,
    start_time: float | None = None,
) -> list[tuple[float, float, float]]:
    stamps = [state.stamp for state in states]
    if start_time is None:
        start_time = stamps[0]
    if not math.isfinite(start_time) or start_time >= cutoff_time:
        raise ValueError("path start time must be finite and before cutoff")
    start_index = bisect.bisect_left(stamps, start_time)
    end_index = bisect.bisect_left(stamps, cutoff_time)
    if end_index == 0:
        raise ValueError("cutoff occurs before the first vehicle state")

    candidates: list[RawState] = []
    if start_index <= 0:
        candidates.append(states[0])
        start_index = 1
    elif start_index >= len(states):
        raise ValueError("path start occurs after the last vehicle state")
    else:
        previous = states[start_index - 1]
        current = states[start_index]
        dt = current.stamp - previous.stamp
        ratio = 0.0 if dt <= 1e-9 else (start_time - previous.stamp) / dt
        ratio = min(1.0, max(0.0, ratio))
        candidates.append(
            RawState(
                start_time,
                _lerp(previous.x, current.x, ratio),
                _lerp(previous.y, current.y, ratio),
                _lerp(previous.z, current.z, ratio),
            )
        )
    candidates.extend(
        state for state in states[start_index:end_index] if state.stamp > start_time
    )
    if end_index < len(states):
        previous = states[end_index - 1]
        current = states[end_index]
        dt = current.stamp - previous.stamp
        ratio = 0.0 if dt <= 1e-9 else (cutoff_time - previous.stamp) / dt
        ratio = min(1.0, max(0.0, ratio))
        candidates.append(
            RawState(
                cutoff_time,
                _lerp(previous.x, current.x, ratio),
                _lerp(previous.y, current.y, ratio),
                _lerp(previous.z, current.z, ratio),
            )
        )

    path: list[tuple[float, float, float]] = []
    for state in candidates:
        point = (state.x, state.y, state.z)
        if path and _distance_xy(path[-1], point) < min_point_distance_m:
            continue
        path.append(point)
    if len(path) < 2:
        raise ValueError("continuous path has fewer than two unique points")
    return path


def cumulative_distances(points: Sequence[tuple[float, float, float]]) -> list[float]:
    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        distances.append(distances[-1] + _distance_xy(previous, current))
    return distances


def curvature_speed_envelope(
    curvatures: Sequence[float], distances: Sequence[float], window_m: float
) -> list[float]:
    """Return the worst absolute curvature seen across a spatial window."""
    if len(curvatures) != len(distances) or not curvatures:
        raise ValueError("curvature envelope inputs must be non-empty and aligned")
    if window_m < 0.0:
        raise ValueError("curvature envelope window must be non-negative")
    if any(current < previous for previous, current in zip(distances, distances[1:])):
        raise ValueError("curvature envelope distances must be monotonic")
    half_window_m = 0.5 * window_m
    return [
        max(
            abs(curvatures[index])
            for index in range(
                bisect.bisect_left(distances, distance - half_window_m),
                bisect.bisect_right(distances, distance + half_window_m),
            )
        )
        for distance in distances
    ]


def interpolate_path(
    points: Sequence[tuple[float, float, float]],
    distances: Sequence[float],
    target_s: float,
) -> tuple[float, float, float]:
    index = bisect.bisect_right(distances, target_s) - 1
    index = min(max(index, 0), len(points) - 2)
    start_s = distances[index]
    end_s = distances[index + 1]
    ratio = 0.0 if end_s <= start_s else (target_s - start_s) / (end_s - start_s)
    return tuple(_lerp(points[index][axis], points[index + 1][axis], ratio) for axis in range(3))


def resample_path(
    points: Sequence[tuple[float, float, float]], interval_m: float
) -> list[tuple[float, float, float]]:
    if interval_m <= 0.0:
        raise ValueError("sample interval must be positive")
    distances = cumulative_distances(points)
    total = distances[-1]
    targets = [index * interval_m for index in range(int(total / interval_m) + 1)]
    if total - targets[-1] > 1e-6:
        targets.append(total)
    else:
        targets[-1] = total
    return [interpolate_path(points, distances, target) for target in targets]


def bounded_smooth(
    points: Sequence[tuple[float, float, float]], radius: int, max_offset_m: float
) -> list[tuple[float, float, float]]:
    if radius <= 0 or max_offset_m <= 0.0 or len(points) < 2 * radius + 1:
        return list(points)
    output = list(points)
    for index in range(radius, len(points) - radius):
        window = points[index - radius : index + radius + 1]
        average = tuple(sum(point[axis] for point in window) / len(window) for axis in range(3))
        delta = tuple(average[axis] - points[index][axis] for axis in range(3))
        magnitude = math.sqrt(sum(value * value for value in delta))
        scale = 1.0 if magnitude <= max_offset_m else max_offset_m / magnitude
        output[index] = tuple(points[index][axis] + scale * delta[axis] for axis in range(3))
    return output


def build_course_samples(
    raw_path: Sequence[tuple[float, float, float]], config: CourseBuildConfig
) -> tuple[list[CourseSample], dict[str, float]]:
    resampled = resample_path(raw_path, config.sample_interval_m)
    smoothed = bounded_smooth(
        resampled, config.smoothing_radius, config.max_smoothing_offset_m
    )
    distances = cumulative_distances(smoothed)
    yaws = _forward_yaws(smoothed)
    curvatures = [_curvature(smoothed, index) for index in range(len(smoothed))]
    velocities = _velocity_profile(smoothed, curvatures, config)
    accelerations = _accelerations(smoothed, velocities)
    samples = [
        CourseSample(
            s=distances[index],
            x=point[0],
            y=point[1],
            z=point[2],
            yaw=yaws[index],
            curvature=curvatures[index],
            left_offset=config.left_offset_m,
            right_offset=config.right_offset_m,
            target_velocity=velocities[index],
            target_acceleration=accelerations[index],
        )
        for index, point in enumerate(smoothed)
    ]
    deviations = [_distance_3d(before, after) for before, after in zip(resampled, smoothed)]
    return samples, {
        "raw_path_length_m": cumulative_distances(raw_path)[-1],
        "course_length_m": samples[-1].s,
        "smoothing_max_deviation_m": max(deviations, default=0.0),
    }


def _point_segment_distance_xy(
    point: tuple[float, float],
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    projection_x = start[0] + ratio * dx
    projection_y = start[1] + ratio * dy
    return math.hypot(point[0] - projection_x, point[1] - projection_y)


def holdout_errors(
    course: Sequence[CourseSample],
    holdout_path: Sequence[tuple[float, float, float]],
    *,
    alignment_method: str = "normalized_arc_length",
    projection_window_m: float = 10.0,
    projection_sample_interval_m: float = 0.25,
) -> list[float]:
    if alignment_method not in {"normalized_arc_length", "local_path_projection"}:
        raise ValueError(f"unsupported holdout alignment method: {alignment_method}")
    holdout_distances = cumulative_distances(holdout_path)
    course_length = course[-1].s
    holdout_length = holdout_distances[-1]
    if alignment_method == "local_path_projection":
        if projection_window_m <= 0.0 or projection_sample_interval_m <= 0.0:
            raise ValueError("holdout projection window and interval must be positive")
        projected_path = resample_path(
            holdout_path, projection_sample_interval_m
        )
        projected_distances = cumulative_distances(projected_path)
        errors = []
        for sample in course:
            fraction = 0.0 if course_length <= 0.0 else sample.s / course_length
            expected = fraction * holdout_length
            lower_distance = max(0.0, expected - projection_window_m)
            upper_distance = min(holdout_length, expected + projection_window_m)
            first = max(0, bisect.bisect_left(projected_distances, lower_distance) - 1)
            last = min(
                len(projected_path) - 1,
                bisect.bisect_right(projected_distances, upper_distance),
            )
            errors.append(
                min(
                    _point_segment_distance_xy(
                        (sample.x, sample.y),
                        projected_path[index],
                        projected_path[index + 1],
                    )
                    for index in range(first, last)
                )
            )
        return errors

    errors = []
    for sample in course:
        fraction = 0.0 if course_length <= 0.0 else sample.s / course_length
        holdout = interpolate_path(holdout_path, holdout_distances, fraction * holdout_length)
        errors.append(math.hypot(sample.x - holdout[0], sample.y - holdout[1]))
    return errors


def validate_course(
    samples: Sequence[CourseSample],
    holdout_xy_errors: Sequence[float],
    progress_rows: Sequence[ProgressState],
    cutoff_time: float,
    config: CourseBuildConfig,
    road_extent_metrics: dict[str, float] | None = None,
) -> dict:
    if len(samples) < 2:
        raise ValueError("course has fewer than two samples")
    spacings = [current.s - previous.s for previous, current in zip(samples, samples[1:])]
    yaw_errors = []
    for index, sample in enumerate(samples):
        if index < len(samples) - 1:
            other = samples[index + 1]
            tangent = math.atan2(other.y - sample.y, other.x - sample.x)
        else:
            previous = samples[index - 1]
            tangent = math.atan2(sample.y - previous.y, sample.x - previous.x)
        yaw_errors.append(abs(_normalize_angle(sample.yaw - tangent)))

    lateral_accels = [
        sample.target_velocity * sample.target_velocity * abs(sample.curvature)
        for sample in samples
    ]
    progress_before_cutoff = [row for row in progress_rows if row.stamp <= cutoff_time + 1e-6]
    holdout_sorted = sorted(holdout_xy_errors)
    metrics = {
        "point_count": len(samples),
        "course_length_m": samples[-1].s,
        "spacing_min_m": min(spacings),
        "spacing_max_m": max(spacings),
        "yaw_tangent_error_max_rad": max(yaw_errors),
        "lateral_accel_max_mps2": max(lateral_accels),
        "target_acceleration_min_mps2": min(sample.target_acceleration for sample in samples),
        "target_acceleration_max_mps2": max(sample.target_acceleration for sample in samples),
        "holdout_xy_mean_m": sum(holdout_xy_errors) / len(holdout_xy_errors),
        "holdout_xy_p95_m": _percentile(holdout_sorted, 0.95),
        "holdout_xy_max_m": max(holdout_xy_errors),
    }
    if road_extent_metrics is not None:
        metrics.update(road_extent_metrics)
    checks = {
        "finite": all(
            math.isfinite(value)
            for sample in samples
            for value in asdict(sample).values()
        ),
        "s_strictly_increasing": all(spacing > 0.0 for spacing in spacings),
        "spacing_bounded": max(spacings) <= config.sample_interval_m * 1.25,
        "forward_yaw": metrics["yaw_tangent_error_max_rad"] <= 1e-6,
        "positive_offsets": all(
            sample.left_offset > 0.0 and sample.right_offset > 0.0 for sample in samples
        ),
        "speed_bounded": all(
            0.0 <= sample.target_velocity <= config.max_speed_mps + 1e-6
            for sample in samples
        ),
        "terminal_stop": samples[-1].target_velocity == 0.0,
        "lateral_accel_bounded": (
            metrics["lateral_accel_max_mps2"] <= config.max_lateral_accel_mps2 + 1e-5
        ),
        "longitudinal_accel_bounded": (
            metrics["target_acceleration_min_mps2"] >= config.max_decel_mps2 - 1e-5
            and metrics["target_acceleration_max_mps2"] <= config.max_accel_mps2 + 1e-5
        ),
        "build_center_on_road": bool(progress_before_cutoff)
        and all(row.on_road and row.road_eval_ok for row in progress_before_cutoff),
        "independent_holdout_p95": (
            metrics["holdout_xy_p95_m"] <= config.holdout_p95_limit_m
        ),
        "independent_holdout_max": (
            metrics["holdout_xy_max_m"] <= config.holdout_max_limit_m
        ),
    }
    if config.require_roadeval_boundaries:
        checks["road_eval_functional_corridor"] = road_extent_metrics is not None and (
            metrics["road_eval_minimum_outward_clearance_m"]
            >= config.road_edge_margin_m - 1e-6
            and metrics["vehicle_footprint_minimum_margin_m"]
            >= config.minimum_footprint_margin_m - 1e-6
        )
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "metrics": metrics,
    }


def write_course_csv(path: Path, samples: Iterable[CourseSample]) -> None:
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=COURSE_COLUMNS)
        writer.writeheader()
        for sample in samples:
            writer.writerow({name: f"{getattr(sample, name):.9f}" for name in COURSE_COLUMNS})


def load_course_csv(path: Path) -> list[CourseSample]:
    samples: list[CourseSample] = []
    with path.open("r", encoding="ascii", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != COURSE_COLUMNS:
            raise ValueError(f"unexpected course schema in {path}: {reader.fieldnames}")
        for row in reader:
            samples.append(CourseSample(**{name: float(row[name]) for name in COURSE_COLUMNS}))
    if len(samples) < 2:
        raise ValueError(f"course has fewer than two points: {path}")
    return samples


def load_road_extents(path: Path) -> list[RoadExtentSample]:
    extents: list[RoadExtentSample] = []
    with path.open("r", encoding="ascii", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != ROAD_EXTENT_COLUMNS:
            raise ValueError(f"unexpected RoadEval extent schema in {path}: {reader.fieldnames}")
        for row in reader:
            extents.append(
                RoadExtentSample(
                    index=int(row["index"]),
                    s=float(row["s"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    yaw=float(row["yaw"]),
                    center_on_road=int(row["center_on_road"]) == 1,
                    left_road_extent=float(row["left_road_extent"]),
                    right_road_extent=float(row["right_road_extent"]),
                )
            )
    if len(extents) < 2:
        raise ValueError(f"RoadEval extent input has fewer than two points: {path}")
    return extents


def write_road_extents(path: Path, extents: Iterable[RoadExtentSample]) -> None:
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(ROAD_EXTENT_COLUMNS)
        for extent in extents:
            writer.writerow(
                (
                    extent.index,
                    f"{extent.s:.12g}",
                    f"{extent.x:.12g}",
                    f"{extent.y:.12g}",
                    f"{extent.yaw:.12g}",
                    int(extent.center_on_road),
                    f"{extent.left_road_extent:.12g}",
                    f"{extent.right_road_extent:.12g}",
                )
            )


def center_course_in_road_corridor(
    samples: Sequence[CourseSample],
    extents: Sequence[RoadExtentSample],
    config: CourseBuildConfig,
) -> tuple[list[CourseSample], list[RoadExtentSample], dict[str, float]]:
    """Move an opt-in course laterally toward the measured road corridor center.

    Positive shift is to the left of the original course tangent.  Therefore
    the measured left extent decreases by the shift and the right extent
    increases by it.  Every target is projected into the physical feasibility
    interval before and after smoothing; no extrapolated road area is used.
    """
    if len(samples) != len(extents):
        raise ValueError(
            f"RoadEval extent count mismatch: course={len(samples)}, extents={len(extents)}"
        )
    max_shift = config.road_corridor_centering_max_shift_m
    if not math.isfinite(max_shift) or max_shift < 0.0:
        raise ValueError("road corridor centering maximum shift must be finite and nonnegative")
    if config.road_corridor_centering_smoothing_radius < 0:
        raise ValueError("road corridor centering smoothing radius must be nonnegative")
    if (
        not math.isfinite(config.road_corridor_centering_max_step_m)
        or config.road_corridor_centering_max_step_m <= 0.0
    ):
        raise ValueError("road corridor centering maximum step must be finite and positive")

    for index, (sample, extent) in enumerate(zip(samples, extents)):
        if extent.index != index:
            raise ValueError(
                f"RoadEval extent index mismatch at row {index}: {extent.index}"
            )
        geometry_error = max(
            abs(sample.s - extent.s),
            abs(sample.x - extent.x),
            abs(sample.y - extent.y),
            abs(_normalize_angle(sample.yaw - extent.yaw)),
        )
        if geometry_error > 1e-6:
            raise ValueError(
                f"RoadEval extent geometry mismatch at row {index}: {geometry_error:.3e}"
            )

    if max_shift == 0.0:
        return list(samples), list(extents), {
            "road_corridor_centering_enabled": 0.0,
            "road_corridor_centering_min_shift_m": 0.0,
            "road_corridor_centering_max_shift_m": 0.0,
            "road_corridor_centering_max_step_m": 0.0,
        }

    required_extent = (
        0.5 * config.vehicle_width_m
        + config.minimum_footprint_margin_m
        + config.road_edge_margin_m
    )
    bounds: list[tuple[float, float]] = []
    targets: list[float] = []
    for index, extent in enumerate(extents):
        if not extent.center_on_road:
            raise ValueError(f"course center is off-road at row {index}")
        lower = max(-max_shift, required_extent - extent.right_road_extent)
        upper = min(max_shift, extent.left_road_extent - required_extent)
        if lower > upper + 1e-9:
            raise ValueError(
                f"road corridor cannot contain the configured footprint at row {index}"
            )
        target = 0.5 * (extent.left_road_extent - extent.right_road_extent)
        bounds.append((lower, upper))
        targets.append(min(upper, max(lower, target)))

    radius = config.road_corridor_centering_smoothing_radius
    smoothed: list[float] = []
    for index, (lower, upper) in enumerate(bounds):
        begin = max(0, index - radius)
        end = min(len(targets), index + radius + 1)
        target = sum(targets[begin:end]) / (end - begin)
        smoothed.append(min(upper, max(lower, target)))

    step = config.road_corridor_centering_max_step_m
    for index in range(1, len(smoothed)):
        lower, upper = bounds[index]
        target = min(smoothed[index - 1] + step, max(smoothed[index - 1] - step, smoothed[index]))
        smoothed[index] = min(upper, max(lower, target))
    for index in range(len(smoothed) - 2, -1, -1):
        lower, upper = bounds[index]
        target = min(smoothed[index + 1] + step, max(smoothed[index + 1] - step, smoothed[index]))
        smoothed[index] = min(upper, max(lower, target))

    maximum_step = max(
        (abs(current - previous) for previous, current in zip(smoothed, smoothed[1:])),
        default=0.0,
    )
    if maximum_step > step + 1e-9:
        raise ValueError(
            "road corridor feasibility changes faster than the configured shift-rate bound"
        )

    points = [
        (
            sample.x - math.sin(sample.yaw) * shift,
            sample.y + math.cos(sample.yaw) * shift,
            sample.z,
        )
        for sample, shift in zip(samples, smoothed)
    ]
    distances = cumulative_distances(points)
    yaws = _forward_yaws(points)
    curvatures = [_curvature(points, index) for index in range(len(points))]
    velocities = _velocity_profile(points, curvatures, config)
    accelerations = _accelerations(points, velocities)
    centered = [
        CourseSample(
            s=distances[index],
            x=point[0],
            y=point[1],
            z=point[2],
            yaw=yaws[index],
            curvature=curvatures[index],
            left_offset=config.left_offset_m,
            right_offset=config.right_offset_m,
            target_velocity=velocities[index],
            target_acceleration=accelerations[index],
        )
        for index, point in enumerate(points)
    ]
    centered_extents = [
        RoadExtentSample(
            index=index,
            s=centered[index].s,
            x=centered[index].x,
            y=centered[index].y,
            yaw=centered[index].yaw,
            center_on_road=extent.center_on_road,
            left_road_extent=extent.left_road_extent - smoothed[index],
            right_road_extent=extent.right_road_extent + smoothed[index],
        )
        for index, extent in enumerate(extents)
    ]
    return centered, centered_extents, {
        "road_corridor_centering_enabled": 1.0,
        "road_corridor_centering_min_shift_m": min(smoothed),
        "road_corridor_centering_max_shift_m": max(smoothed),
        "road_corridor_centering_max_abs_shift_m": max(map(abs, smoothed)),
        "road_corridor_centering_max_step_m": maximum_step,
    }


def apply_road_extents(
    samples: Sequence[CourseSample],
    extents: Sequence[RoadExtentSample],
    config: CourseBuildConfig,
) -> tuple[list[CourseSample], dict[str, float]]:
    if len(samples) != len(extents):
        raise ValueError(
            f"RoadEval extent count mismatch: course={len(samples)}, extents={len(extents)}"
        )
    if config.road_edge_margin_m <= 0.0:
        raise ValueError("road edge margin must be positive")
    if config.vehicle_width_m <= 0.0:
        raise ValueError("vehicle width must be positive")

    bounded: list[CourseSample] = []
    minimum_left_extent = math.inf
    minimum_right_extent = math.inf
    minimum_outward_clearance = math.inf
    minimum_footprint_margin = math.inf
    maximum_offset_step = 0.0
    half_width = 0.5 * config.vehicle_width_m
    for index, (sample, extent) in enumerate(zip(samples, extents)):
        if extent.index != index:
            raise ValueError(
                f"RoadEval extent index mismatch at row {index}: {extent.index}"
            )
        geometry_error = max(
            abs(sample.s - extent.s),
            abs(sample.x - extent.x),
            abs(sample.y - extent.y),
            abs(_normalize_angle(sample.yaw - extent.yaw)),
        )
        if geometry_error > 1e-6:
            raise ValueError(
                f"RoadEval extent geometry mismatch at row {index}: {geometry_error:.3e}"
            )
        if not extent.center_on_road:
            raise ValueError(f"course center is off-road at row {index}")
        if min(extent.left_road_extent, extent.right_road_extent) <= 0.0:
            raise ValueError(f"invalid RoadEval road extent at row {index}")

        left_offset = min(
            config.left_offset_m,
            extent.left_road_extent - config.road_edge_margin_m,
        )
        right_offset = min(
            config.right_offset_m,
            extent.right_road_extent - config.road_edge_margin_m,
        )
        bounded.append(
            replace(sample, left_offset=left_offset, right_offset=right_offset)
        )
        minimum_left_extent = min(minimum_left_extent, extent.left_road_extent)
        minimum_right_extent = min(minimum_right_extent, extent.right_road_extent)
        minimum_outward_clearance = min(
            minimum_outward_clearance,
            extent.left_road_extent - left_offset,
            extent.right_road_extent - right_offset,
        )
        minimum_footprint_margin = min(
            minimum_footprint_margin,
            left_offset - half_width,
            right_offset - half_width,
        )
        if index > 0:
            maximum_offset_step = max(
                maximum_offset_step,
                abs(left_offset - bounded[index - 1].left_offset),
                abs(right_offset - bounded[index - 1].right_offset),
            )

    return bounded, {
        "road_eval_minimum_left_extent_m": minimum_left_extent,
        "road_eval_minimum_right_extent_m": minimum_right_extent,
        "road_eval_minimum_outward_clearance_m": minimum_outward_clearance,
        "functional_offset_minimum_left_m": min(
            sample.left_offset for sample in bounded
        ),
        "functional_offset_minimum_right_m": min(
            sample.right_offset for sample in bounded
        ),
        "functional_offset_maximum_step_m": maximum_offset_step,
        "vehicle_footprint_minimum_margin_m": minimum_footprint_margin,
    }


def load_course_asset(asset_dir: Path) -> tuple[dict, list[CourseSample]]:
    manifest_path = asset_dir / "manifest.json"
    course_path = asset_dir / "course.csv"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 2:
        raise ValueError("course manifest schema version is not 2")
    for filename, contract in manifest.get("assets", {}).items():
        asset_path = asset_dir / filename
        if not asset_path.is_file():
            raise ValueError(f"course asset is missing: {asset_path}")
        actual_hash = sha256_file(asset_path)
        expected_hash = contract.get("sha256")
        if actual_hash != expected_hash:
            raise ValueError(
                f"course asset hash mismatch for {filename}: "
                f"expected {expected_hash!r}, got {actual_hash!r}"
            )
    if manifest.get("validation", {}).get("status") != "PASS":
        raise ValueError("course manifest validation status is not PASS")
    if (
        manifest.get("validation", {})
        .get("checks", {})
        .get("road_eval_functional_corridor")
        is not True
        or "road_extents.csv" not in manifest.get("assets", {})
    ):
        raise ValueError("course manifest has no validated RoadEval functional corridor")
    return manifest, load_course_csv(course_path)


def validate_course_map_contract(manifest: dict, map_path: Path) -> None:
    contract = manifest.get("map")
    if not isinstance(contract, dict):
        raise ValueError("course manifest has no map hash contract")
    expected_id = contract.get("id")
    if expected_id != manifest.get("map_id") or map_path.name != expected_id:
        raise ValueError(
            f"course/map ID mismatch: expected {expected_id!r}, got {map_path.name!r}"
        )
    for key, filename in MAP_CONTRACT_FILES.items():
        path = map_path / filename
        if not path.is_file():
            raise ValueError(f"course map contract file is missing: {path}")
        actual = sha256_file(path)
        expected = contract.get(key)
        if actual != expected:
            raise ValueError(
                f"course map contract mismatch for {filename}: "
                f"expected {expected!r}, got {actual!r}"
            )
    release = json.loads((map_path / "release_manifest.json").read_text(encoding="utf-8"))
    tiled_map = release.get("tiled_map")
    if release.get("status") != "PASS" or not isinstance(tiled_map, dict):
        raise ValueError("pointcloud release manifest is not a PASS tiled-map release")
    tile_contracts = tiled_map.get("tiles")
    if not isinstance(tile_contracts, list) or not tile_contracts:
        raise ValueError("pointcloud release manifest has no tile contracts")
    expected_tiles = {Path(item["path"]).name: item for item in tile_contracts}
    if int(tiled_map.get("tile_count", -1)) != len(tile_contracts):
        raise ValueError("pointcloud release manifest tile count is inconsistent")
    metadata_contract = tiled_map.get("metadata", {})
    if (
        Path(str(metadata_contract.get("path", ""))).name
        != "pointcloud_map_metadata.yaml"
        or metadata_contract.get("sha256")
        != contract["pointcloud_map_metadata_sha256"]
    ):
        raise ValueError("pointcloud release metadata contract is inconsistent")
    actual_tiles = {path.name: path for path in map_path.glob("*.pcd")}
    if len(expected_tiles) != len(tile_contracts) or set(actual_tiles) != set(
        expected_tiles
    ):
        raise ValueError(
            "pointcloud tile set mismatch: "
            f"expected={len(expected_tiles)}, actual={len(actual_tiles)}"
        )
    for filename, item in expected_tiles.items():
        path = actual_tiles[filename]
        if path.stat().st_size != int(item["size_bytes"]):
            raise ValueError(f"pointcloud tile size mismatch: {filename}")
        if sha256_file(path) != item["sha256"]:
            raise ValueError(f"pointcloud tile hash mismatch: {filename}")


def build_asset(
    build_vehicle_state: Path,
    build_route_progress: Path,
    holdout_vehicle_state: Path,
    holdout_route_progress: Path,
    source_testrun: Path,
    map_path: Path,
    output_dir: Path,
    config: CourseBuildConfig,
    road_file: Path | None = None,
    road_extents: Path | None = None,
) -> dict:
    if (road_file is None) != (road_extents is None):
        raise ValueError("road file and RoadEval extents must be provided together")
    if config.require_roadeval_boundaries and road_extents is None:
        raise ValueError("RoadEval extents are required for a production course asset")
    if map_path.name != config.map_id:
        raise ValueError(
            f"map path name must match map_id {config.map_id!r}: {map_path}"
        )
    map_contract = {}
    for key, filename in MAP_CONTRACT_FILES.items():
        path = map_path / filename
        if not path.is_file():
            raise ValueError(f"required map contract file is missing: {path}")
        map_contract[key] = sha256_file(path)
    build_states = load_vehicle_states(build_vehicle_state)
    build_progress = load_progress_states(build_route_progress)
    holdout_states = load_vehicle_states(holdout_vehicle_state)
    holdout_progress = load_progress_states(holdout_route_progress)
    build_cutoff = cutoff_time_for_distance(build_progress, config.carmaker_route_length_m)
    holdout_cutoff = cutoff_time_for_distance(holdout_progress, config.carmaker_route_length_m)
    build_start = cutoff_time_for_distance(
        build_progress, config.startup_trim_distance_m
    )
    holdout_start = cutoff_time_for_distance(
        holdout_progress, config.startup_trim_distance_m
    )
    build_path = extract_continuous_path(
        build_states,
        build_cutoff,
        config.min_raw_point_distance_m,
        build_start,
    )
    holdout_path = extract_continuous_path(
        holdout_states,
        holdout_cutoff,
        config.min_raw_point_distance_m,
        holdout_start,
    )
    samples, build_metrics = build_course_samples(build_path, config)
    road_extent_metrics = None
    output_road_extents = None
    if road_extents is not None:
        source_road_extents = load_road_extents(road_extents)
        samples, output_road_extents, centering_metrics = center_course_in_road_corridor(
            samples, source_road_extents, config
        )
        samples, road_extent_metrics = apply_road_extents(
            samples, output_road_extents, config
        )
        road_extent_metrics.update(centering_metrics)
        build_metrics["course_length_m"] = samples[-1].s
    errors = holdout_errors(
        samples,
        holdout_path,
        alignment_method=config.holdout_alignment_method,
        projection_window_m=config.holdout_projection_window_m,
        projection_sample_interval_m=config.holdout_projection_sample_interval_m,
    )
    validation = validate_course(
        samples,
        errors,
        build_progress,
        build_cutoff,
        config,
        road_extent_metrics,
    )
    validation["metrics"].update(build_metrics)
    validation["metrics"]["holdout_raw_path_length_m"] = cumulative_distances(holdout_path)[-1]

    temporary = output_dir.with_name(f".{output_dir.name}.tmp-{os.getpid()}")
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir(parents=True)
    course_path = temporary / "course.csv"
    write_course_csv(course_path, samples)
    extents_asset = None
    if road_extents is not None:
        extents_asset = temporary / "road_extents.csv"
        if config.road_corridor_centering_max_shift_m > 0.0:
            write_road_extents(extents_asset, output_road_extents or ())
        else:
            shutil.copyfile(road_extents, extents_asset)
    manifest = {
        "schema_version": 2,
        "route_id": config.route_id,
        "map_id": config.map_id,
        "map": {"id": config.map_id, **map_contract},
        "frame_id": config.frame_id,
        "version": config.version,
        "source": {
            "testrun": source_testrun.name,
            "testrun_sha256": sha256_file(source_testrun),
            "build_collection": build_vehicle_state.parent.name,
            "build_vehicle_state_sha256": sha256_file(build_vehicle_state),
            "build_route_progress_sha256": sha256_file(build_route_progress),
            "holdout_collection": holdout_vehicle_state.parent.name,
            "holdout_vehicle_state_sha256": sha256_file(holdout_vehicle_state),
            "holdout_route_progress_sha256": sha256_file(holdout_route_progress),
            **(
                {
                    "road_file": road_file.name,
                    "road_file_sha256": sha256_file(road_file),
                    "road_extents_sha256": sha256_file(road_extents),
                }
                if road_file is not None and road_extents is not None
                else {}
            ),
        },
        "geometry": {
            "carmaker_route_length_m": config.carmaker_route_length_m,
            "course_length_m": samples[-1].s,
            "sample_interval_m": config.sample_interval_m,
            "startup_trim_distance_m": config.startup_trim_distance_m,
            "frame_contract": "CarMaker Fr0 == localization map",
            "maximum_left_offset_m": config.left_offset_m,
            "maximum_right_offset_m": config.right_offset_m,
            "road_edge_margin_m": config.road_edge_margin_m,
            "vehicle_width_m": config.vehicle_width_m,
            "minimum_footprint_margin_m": config.minimum_footprint_margin_m,
            "road_corridor_centering_max_shift_m": (
                config.road_corridor_centering_max_shift_m
            ),
            "road_corridor_centering_smoothing_radius": (
                config.road_corridor_centering_smoothing_radius
            ),
            "road_corridor_centering_max_step_m": (
                config.road_corridor_centering_max_step_m
            ),
            "boundary_semantics": (
                "progress-indexed functional Frenet offsets, capped at 1.8 m, "
                "inset from RoadEval road extents, with optional bounded lateral "
                "centering inside the same measured corridor"
            ),
            "road_eval_evidence": (
                "course center and both functional offsets independently evaluated "
                "against the configured CarMaker road"
            ),
        },
        "speed_profile": {
            "max_speed_mps": config.max_speed_mps,
            "max_lateral_accel_mps2": config.max_lateral_accel_mps2,
            "max_accel_mps2": config.max_accel_mps2,
            "max_decel_mps2": config.max_decel_mps2,
        },
        "smoothing": {
            "radius": config.smoothing_radius,
            "max_offset_m": config.max_smoothing_offset_m,
        },
        "holdout_alignment": {
            "method": config.holdout_alignment_method,
            "projection_window_m": config.holdout_projection_window_m,
            "projection_sample_interval_m": config.holdout_projection_sample_interval_m,
            "semantics": (
                "local_path_projection removes bounded longitudinal phase error "
                "without allowing matches to remote switchback segments"
                if config.holdout_alignment_method == "local_path_projection"
                else "equal normalized cumulative arc length"
            ),
        },
        "validation": validation,
        "assets": {
            "course.csv": {
                "sha256": sha256_file(course_path),
                "rows": len(samples),
            },
            **(
                {
                    "road_extents.csv": {
                        "sha256": sha256_file(extents_asset),
                        "rows": len(samples),
                    }
                }
                if extents_asset is not None
                else {}
            ),
        },
        "limitations": [
            "The functional corridor is capped at 3.6 m and narrows where RoadEval requires it; it is not a surveyed curb polygon.",
            "GT and RoadEval evidence are offline-only and are not runtime planner inputs.",
        ],
    }
    (temporary / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if validation["status"] != "PASS":
        raise ValueError(f"course validation failed: {validation['checks']}")
    if output_dir.exists():
        raise FileExistsError(f"refusing to replace existing course asset: {output_dir}")
    temporary.replace(output_dir)
    return manifest


def _velocity_profile(
    points: Sequence[tuple[float, float, float]],
    curvatures: Sequence[float],
    config: CourseBuildConfig,
) -> list[float]:
    if config.max_course_yaw_rate_radps < 0.0:
        raise ValueError("maximum course yaw rate must be non-negative")
    speeds = []
    for curvature in curvatures:
        limit = config.max_speed_mps
        if abs(curvature) > 1e-9:
            limit = min(limit, math.sqrt(config.max_lateral_accel_mps2 / abs(curvature)))
            if config.max_course_yaw_rate_radps > 0.0:
                limit = min(limit, config.max_course_yaw_rate_radps / abs(curvature))
        speeds.append(max(0.0, limit))
    speeds[0] = 0.0
    speeds[-1] = 0.0
    decel = abs(config.max_decel_mps2)
    for _ in range(3):
        for index in range(1, len(points)):
            ds = _distance_xy(points[index - 1], points[index])
            limit = math.sqrt(max(0.0, speeds[index - 1] ** 2 + 2.0 * config.max_accel_mps2 * ds))
            speeds[index] = min(speeds[index], limit)
        speeds[-1] = 0.0
        for index in range(len(points) - 2, -1, -1):
            ds = _distance_xy(points[index], points[index + 1])
            limit = math.sqrt(max(0.0, speeds[index + 1] ** 2 + 2.0 * decel * ds))
            speeds[index] = min(speeds[index], limit)
        speeds[0] = 0.0
    return speeds


def _accelerations(
    points: Sequence[tuple[float, float, float]], speeds: Sequence[float]
) -> list[float]:
    """Associate feedforward acceleration with each point's outgoing segment."""
    accelerations = [0.0] * len(points)
    for index in range(len(points) - 1):
        ds = max(_distance_xy(points[index], points[index + 1]), 1e-6)
        accelerations[index] = (
            speeds[index + 1] ** 2 - speeds[index] ** 2
        ) / (2.0 * ds)
    return accelerations


def _forward_yaws(points: Sequence[tuple[float, float, float]]) -> list[float]:
    yaws = []
    for index, point in enumerate(points):
        if index < len(points) - 1:
            other = points[index + 1]
            dx = other[0] - point[0]
            dy = other[1] - point[1]
        else:
            other = points[index - 1]
            dx = point[0] - other[0]
            dy = point[1] - other[1]
        yaws.append(math.atan2(dy, dx))
    return yaws


def _curvature(points: Sequence[tuple[float, float, float]], index: int) -> float:
    if index <= 0 or index >= len(points) - 1:
        return 0.0
    previous = points[index - 1]
    current = points[index]
    following = points[index + 1]
    a = _distance_xy(previous, current)
    b = _distance_xy(current, following)
    c = _distance_xy(previous, following)
    if min(a, b, c) < 1e-9:
        return 0.0
    cross = (current[0] - previous[0]) * (following[1] - previous[1]) - (
        current[1] - previous[1]
    ) * (following[0] - previous[0])
    return 2.0 * cross / (a * b * c)


def _percentile(sorted_values: Sequence[float], fraction: float) -> float:
    if not sorted_values:
        raise ValueError("cannot calculate percentile of an empty sequence")
    position = min(1.0, max(0.0, fraction)) * (len(sorted_values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    return _lerp(sorted_values[lower], sorted_values[upper], position - lower)


def _distance_xy(left: Sequence[float], right: Sequence[float]) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def _distance_3d(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((left[index] - right[index]) ** 2 for index in range(3)))


def _lerp(start: float, end: float, ratio: float) -> float:
    return start + (end - start) * ratio


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    parser = argparse.ArgumentParser(description="Build and validate a fixed CarMaker course")
    parser.add_argument("--build-vehicle-state", type=Path, required=True)
    parser.add_argument("--build-route-progress", type=Path, required=True)
    parser.add_argument("--holdout-vehicle-state", type=Path, required=True)
    parser.add_argument("--holdout-route-progress", type=Path, required=True)
    parser.add_argument("--source-testrun", type=Path, required=True)
    parser.add_argument("--map-path", type=Path, required=True)
    parser.add_argument("--road-file", type=Path, required=True)
    parser.add_argument("--road-extents", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--version", default=CourseBuildConfig().version)
    parser.add_argument(
        "--max-lateral-accel-mps2",
        type=float,
        default=CourseBuildConfig().max_lateral_accel_mps2,
    )
    args = parser.parse_args()
    manifest = build_asset(
        args.build_vehicle_state,
        args.build_route_progress,
        args.holdout_vehicle_state,
        args.holdout_route_progress,
        args.source_testrun,
        args.map_path,
        args.output,
        CourseBuildConfig(
            version=args.version,
            max_lateral_accel_mps2=args.max_lateral_accel_mps2,
        ),
        args.road_file,
        args.road_extents,
    )
    print(json.dumps(manifest["validation"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
