from __future__ import annotations

from dataclasses import dataclass
import bisect
import math
from typing import Sequence


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


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
class RecordedCourseConfig:
    sample_interval_m: float = 0.2
    stationary_step_m: float = 0.02
    minimum_point_spacing_m: float = 0.01
    maximum_step_m: float = 2.0
    smoothing_radius: int = 2
    maximum_smoothing_displacement_m: float = 0.05
    max_speed_mps: float = 0.5
    max_lateral_accel_mps2: float = 0.4
    max_accel_mps2: float = 0.4
    max_decel_mps2: float = -0.8
    departure_speed_mps: float = 0.1
    left_offset_m: float = 0.4
    right_offset_m: float = 0.4


def build_recorded_course(
    source_poses: Sequence[PoseSample], config: RecordedCourseConfig
) -> tuple[list[CourseSample], dict[str, float | int | bool]]:
    _validate_config(config)
    _validate_timestamps(source_poses)
    poses, duplicate_timestamp_count = _collapse_equal_timestamps(source_poses)
    poses, invalid_count = _finite_poses(poses)
    if len(poses) < 2:
        raise ValueError("fewer than two usable poses")

    poses, prefix_removed, suffix_removed = _trim_stationary_ends(poses, config)
    poses, duplicate_count = _collapse_duplicates(poses, config.minimum_point_spacing_m)
    if len(poses) < 2:
        raise ValueError("fewer than two usable poses after stationary trimming")
    _reject_pose_jumps(poses, config.maximum_step_m)

    raw_xyz = [(pose.x, pose.y, pose.z) for pose in poses]
    raw_s = _cumulative_distances(raw_xyz)
    resampled = _resample(raw_xyz, raw_s, config.sample_interval_m)
    smoothed, maximum_displacement = _smooth(resampled, config.smoothing_radius)
    if maximum_displacement > config.maximum_smoothing_displacement_m + 1e-12:
        raise ValueError(
            "smoothing displacement exceeds limit: "
            f"{maximum_displacement:.6f} > {config.maximum_smoothing_displacement_m:.6f}"
        )

    distances = _cumulative_distances(smoothed)
    if len(distances) < 2 or not all(b > a for a, b in zip(distances, distances[1:])):
        raise ValueError("resampled trajectory distance is not strictly increasing")
    yaws = _forward_yaws(smoothed)
    curvatures = _curvatures(smoothed)
    speeds = _speed_profile(distances, curvatures, config)

    samples: list[CourseSample] = []
    for index, ((x, y, z), s, yaw, curvature, speed) in enumerate(
        zip(smoothed, distances, yaws, curvatures, speeds)
    ):
        acceleration = 0.0
        if index > 0:
            ds = distances[index] - distances[index - 1]
            acceleration = (speed * speed - speeds[index - 1] ** 2) / (2.0 * ds)
        samples.append(
            CourseSample(
                s=s,
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                curvature=curvature,
                left_offset=config.left_offset_m,
                right_offset=config.right_offset_m,
                target_velocity=speed,
                target_acceleration=acceleration,
            )
        )

    report: dict[str, float | int | bool] = {
        "source_pose_count": len(source_poses),
        "invalid_points_removed": invalid_count,
        "duplicate_timestamps_removed": duplicate_timestamp_count,
        "stationary_prefix_removed": prefix_removed,
        "stationary_suffix_removed": suffix_removed,
        "duplicate_points_removed": duplicate_count,
        "internal_segments_removed": 0,
        "output_point_count": len(samples),
        "course_length_m": samples[-1].s,
        "maximum_smoothing_displacement_m": maximum_displacement,
        "terminal_stop": samples[-1].target_velocity == 0.0,
    }
    return samples, report


def _validate_config(config: RecordedCourseConfig) -> None:
    positive = (
        config.sample_interval_m,
        config.stationary_step_m,
        config.minimum_point_spacing_m,
        config.maximum_step_m,
        config.maximum_smoothing_displacement_m,
        config.max_speed_mps,
        config.max_lateral_accel_mps2,
        config.max_accel_mps2,
        config.departure_speed_mps,
        config.left_offset_m,
        config.right_offset_m,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError("recorded-course positive limits must be finite and positive")
    if not math.isfinite(config.max_decel_mps2) or config.max_decel_mps2 >= 0.0:
        raise ValueError("max_decel_mps2 must be finite and negative")
    if config.smoothing_radius < 0:
        raise ValueError("smoothing_radius must be non-negative")


def _validate_timestamps(poses: Sequence[PoseSample]) -> None:
    stamps = [pose.stamp for pose in poses]
    if not all(math.isfinite(stamp) for stamp in stamps):
        raise ValueError("pose timestamps must be finite")
    if not all(current >= previous for previous, current in zip(stamps, stamps[1:])):
        raise ValueError("pose timestamps are not strictly increasing")


def _collapse_equal_timestamps(
    poses: Sequence[PoseSample],
) -> tuple[list[PoseSample], int]:
    if not poses:
        return [], 0
    output = [poses[0]]
    removed = 0
    for pose in poses[1:]:
        if pose.stamp == output[-1].stamp:
            output[-1] = pose
            removed += 1
        else:
            output.append(pose)
    return output, removed


def _finite_poses(poses: Sequence[PoseSample]) -> tuple[list[PoseSample], int]:
    usable = []
    for pose in poses:
        values = (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if all(math.isfinite(value) for value in values) and norm > 1e-6:
            usable.append(pose)
    return usable, len(poses) - len(usable)


def _trim_stationary_ends(
    poses: Sequence[PoseSample], config: RecordedCourseConfig
) -> tuple[list[PoseSample], int, int]:
    steps = [_pose_distance(a, b) for a, b in zip(poses, poses[1:])]
    moving = [index for index, distance in enumerate(steps) if distance > config.stationary_step_m]
    if not moving:
        raise ValueError("recording contains no moving trajectory")
    start = moving[0]
    end = moving[-1] + 1
    return list(poses[start : end + 1]), start, len(poses) - end - 1


def _collapse_duplicates(
    poses: Sequence[PoseSample], minimum_spacing: float
) -> tuple[list[PoseSample], int]:
    output = [poses[0]]
    removed = 0
    for pose in poses[1:-1]:
        if _pose_distance(output[-1], pose) < minimum_spacing:
            removed += 1
        else:
            output.append(pose)
    if _pose_distance(output[-1], poses[-1]) < minimum_spacing:
        output[-1] = poses[-1]
        removed += 1
    else:
        output.append(poses[-1])
    return output, removed


def _reject_pose_jumps(poses: Sequence[PoseSample], maximum_step: float) -> None:
    for index, (previous, current) in enumerate(zip(poses, poses[1:]), start=1):
        distance = _pose_distance(previous, current)
        if distance > maximum_step:
            raise ValueError(f"pose jump at index {index}: {distance:.3f} m")


def _pose_distance(a: PoseSample, b: PoseSample) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def _cumulative_distances(points: Sequence[tuple[float, float, float]]) -> list[float]:
    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        distances.append(
            distances[-1] + math.hypot(current[0] - previous[0], current[1] - previous[1])
        )
    return distances


def _resample(
    points: Sequence[tuple[float, float, float]], distances: Sequence[float], interval: float
) -> list[tuple[float, float, float]]:
    length = distances[-1]
    targets = [index * interval for index in range(int(length / interval) + 1)]
    if length - targets[-1] > 1e-9:
        targets.append(length)
    else:
        targets[-1] = length

    output = []
    for target in targets:
        upper = min(max(bisect.bisect_left(distances, target), 1), len(points) - 1)
        lower = upper - 1
        span = distances[upper] - distances[lower]
        ratio = 0.0 if span <= 1e-12 else (target - distances[lower]) / span
        output.append(
            tuple(
                points[lower][axis] + ratio * (points[upper][axis] - points[lower][axis])
                for axis in range(3)
            )
        )
    return output


def _smooth(
    points: Sequence[tuple[float, float, float]], radius: int
) -> tuple[list[tuple[float, float, float]], float]:
    if radius == 0 or len(points) < 3:
        return list(points), 0.0
    output = [points[0]]
    maximum = 0.0
    for index in range(1, len(points) - 1):
        start = max(0, index - radius)
        end = min(len(points), index + radius + 1)
        count = end - start
        candidate_x = sum(point[0] for point in points[start:end]) / count
        candidate_y = sum(point[1] for point in points[start:end]) / count
        tangent_x = points[index + 1][0] - points[index - 1][0]
        tangent_y = points[index + 1][1] - points[index - 1][1]
        tangent_norm = math.hypot(tangent_x, tangent_y)
        if tangent_norm <= 1e-12:
            raise ValueError(f"degenerate smoothing tangent at index {index}")
        normal_x = -tangent_y / tangent_norm
        normal_y = tangent_x / tangent_norm
        lateral_offset = (
            (candidate_x - points[index][0]) * normal_x
            + (candidate_y - points[index][1]) * normal_y
        )
        x = points[index][0] + lateral_offset * normal_x
        y = points[index][1] + lateral_offset * normal_y
        z = points[index][2]
        maximum = max(maximum, math.hypot(x - points[index][0], y - points[index][1]))
        output.append((x, y, z))
    output.append(points[-1])
    return output, maximum


def _forward_yaws(points: Sequence[tuple[float, float, float]]) -> list[float]:
    yaws = []
    for index, point in enumerate(points):
        if index < len(points) - 1:
            other = points[index + 1]
        else:
            other = point
            point = points[index - 1]
        yaws.append(math.atan2(other[1] - point[1], other[0] - point[0]))
    return yaws


def _curvatures(points: Sequence[tuple[float, float, float]]) -> list[float]:
    result = [0.0]
    for first, middle, last in zip(points, points[1:], points[2:]):
        a = math.hypot(middle[0] - first[0], middle[1] - first[1])
        b = math.hypot(last[0] - middle[0], last[1] - middle[1])
        c = math.hypot(last[0] - first[0], last[1] - first[1])
        cross = (middle[0] - first[0]) * (last[1] - middle[1]) - (
            middle[1] - first[1]
        ) * (last[0] - middle[0])
        denominator = a * b * c
        result.append(0.0 if denominator <= 1e-12 else 2.0 * cross / denominator)
    result.append(0.0)
    return result


def _speed_profile(
    distances: Sequence[float], curvatures: Sequence[float], config: RecordedCourseConfig
) -> list[float]:
    speeds = []
    for curvature in curvatures:
        limit = config.max_speed_mps
        if abs(curvature) > 1e-9:
            limit = min(limit, math.sqrt(config.max_lateral_accel_mps2 / abs(curvature)))
        speeds.append(limit)
    speeds[0] = min(speeds[0], config.departure_speed_mps)
    speeds[-1] = 0.0
    for _ in range(2):
        for index in range(1, len(speeds)):
            ds = distances[index] - distances[index - 1]
            speeds[index] = min(
                speeds[index],
                math.sqrt(max(0.0, speeds[index - 1] ** 2 + 2 * config.max_accel_mps2 * ds)),
            )
        speeds[-1] = 0.0
        deceleration = abs(config.max_decel_mps2)
        for index in range(len(speeds) - 2, -1, -1):
            ds = distances[index + 1] - distances[index]
            speeds[index] = min(
                speeds[index],
                math.sqrt(max(0.0, speeds[index + 1] ** 2 + 2 * deceleration * ds)),
            )
    return speeds
