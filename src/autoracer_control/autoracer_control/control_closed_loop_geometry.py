"""Path station and projection helpers for the closed-loop validation bench."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PathPoint:
    x_m: float
    y_m: float


@dataclass(frozen=True)
class PathProjection:
    nearest_idx: int
    nearest_segment_idx: int
    progress_distance_m: float
    projected_x_m: float
    projected_y_m: float
    segment_yaw_rad: float
    distance_m: float


def compute_stations(points: list[PathPoint]) -> list[float]:
    if not points:
        return []

    stations = [0.0]
    for prev, current in zip(points, points[1:]):
        stations.append(
            stations[-1] + math.hypot(current.x_m - prev.x_m, current.y_m - prev.y_m)
        )
    return stations


def project_to_path(points: list[PathPoint], stations: list[float], x_m: float, y_m: float) -> PathProjection:
    if len(points) < 2:
        raise ValueError("at least two path points are required for projection")
    if len(points) != len(stations):
        raise ValueError("path points and station arrays must have the same length")

    best: PathProjection | None = None
    for index, (start, end) in enumerate(zip(points, points[1:])):
        vx = end.x_m - start.x_m
        vy = end.y_m - start.y_m
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-12:
            continue

        t = ((x_m - start.x_m) * vx + (y_m - start.y_m) * vy) / length_sq
        t = min(1.0, max(0.0, t))
        projected_x = start.x_m + t * vx
        projected_y = start.y_m + t * vy
        segment_length = math.sqrt(length_sq)
        progress = stations[index] + t * segment_length
        distance = math.hypot(x_m - projected_x, y_m - projected_y)
        nearest_idx = index if t < 0.5 else index + 1
        projection = PathProjection(
            nearest_idx=nearest_idx,
            nearest_segment_idx=index,
            progress_distance_m=progress,
            projected_x_m=projected_x,
            projected_y_m=projected_y,
            segment_yaw_rad=math.atan2(vy, vx),
            distance_m=distance,
        )
        if best is None or projection.distance_m < best.distance_m:
            best = projection

    if best is None:
        raise ValueError("path projection failed because all segments are degenerate")
    return best


def monotonic_progress(previous_progress_m: float, projected_progress_m: float) -> float:
    return max(float(previous_progress_m), float(projected_progress_m))
