"""Causal LiDAR scan-to-scan relative odometry helpers.

The functions in this module are dependency-light and deliberately offline
testable.  They estimate only the relative 2D motion between two consecutive
scans using the previous scan as a causal local submap.  They do not use GNSS,
GT, map truth, or future frames.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


Point2D = tuple[float, float]


@dataclass(frozen=True)
class RelativeOdometryEstimate:
    forward_m: float
    lateral_m: float
    yaw_rad: float
    residual_m: float
    quality: float
    forward_variance_m2: float
    lateral_variance_m2: float
    yaw_variance_rad2: float
    along_degenerate: bool
    is_valid: bool
    reason: str = ""


def _as_array(points: Iterable[Point2D], max_points: int) -> np.ndarray:
    rows: list[tuple[float, float]] = []
    for point in points:
        if len(point) < 2:
            continue
        x = float(point[0])
        y = float(point[1])
        if math.isfinite(x) and math.isfinite(y):
            rows.append((x, y))
    if max_points > 0 and len(rows) > max_points:
        stride = max(1, len(rows) // max_points)
        rows = rows[::stride][:max_points]
    return np.asarray(rows, dtype=float)


def _rotation(yaw_rad: float) -> np.ndarray:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return np.asarray([[c, -s], [s, c]], dtype=float)


def _nearest_indices(
    reference: np.ndarray,
    query: np.ndarray,
    *,
    max_match_distance_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    # Brute-force nearest neighbor is sufficient for the sampled W10 factor and
    # keeps this helper independent from scipy/sklearn runtime availability.
    diff = query[:, None, :] - reference[None, :, :]
    distances = np.linalg.norm(diff, axis=2)
    indices = np.argmin(distances, axis=1)
    best = distances[np.arange(len(query)), indices]
    mask = best <= max_match_distance_m
    return indices[mask], mask


def _fit_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def _yaw_from_rotation(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _shape_degeneracy(points: np.ndarray, yaw_rad: float) -> tuple[bool, float, float]:
    if len(points) < 3:
        return True, 100.0, 100.0
    centered = points - points.mean(axis=0)
    covariance = centered.T @ centered / max(len(points) - 1, 1)
    eigvals, eigvecs = np.linalg.eigh(covariance)
    order = np.argsort(eigvals)
    small = max(float(eigvals[order[0]]), 1e-9)
    large = max(float(eigvals[order[1]]), small)
    major = eigvecs[:, order[1]]
    forward_axis = np.asarray([math.cos(yaw_rad), math.sin(yaw_rad)])
    major_forward_alignment = abs(float(np.dot(major, forward_axis)))
    # A long thin structure parallel to the travel direction cannot certify
    # along progress; nearest-neighbor ICP can slide along it.
    along_degenerate = (small / large) < 0.03 and major_forward_alignment > 0.75
    condition = large / small
    return along_degenerate, condition, major_forward_alignment


def estimate_scan_to_scan_motion_2d(
    previous_points: Iterable[Point2D],
    current_points: Iterable[Point2D],
    *,
    initial_forward_m: float,
    initial_lateral_m: float = 0.0,
    initial_yaw_rad: float = 0.0,
    max_iterations: int = 8,
    max_match_distance_m: float = 2.5,
    min_match_count: int = 8,
    min_quality: float = 0.35,
    max_residual_m: float = 1.0,
    max_points: int = 512,
    reject_degenerate: bool = False,
) -> RelativeOdometryEstimate:
    """Estimate the current-scan-to-previous-scan transform.

    Returned `forward_m/lateral_m/yaw_rad` are vehicle motion from the previous
    scan to the current scan, expressed in the previous scan frame.  The current
    scan is transformed by that motion to overlap the previous scan.
    """

    previous = _as_array(previous_points, max_points)
    current = _as_array(current_points, max_points)
    if len(previous) < min_match_count or len(current) < min_match_count:
        return RelativeOdometryEstimate(
            initial_forward_m,
            initial_lateral_m,
            initial_yaw_rad,
            residual_m=math.inf,
            quality=0.0,
            forward_variance_m2=100.0,
            lateral_variance_m2=100.0,
            yaw_variance_rad2=1.0,
            along_degenerate=True,
            is_valid=False,
            reason="insufficient_points",
        )

    yaw = float(initial_yaw_rad)
    translation = np.asarray([float(initial_forward_m), float(initial_lateral_m)], dtype=float)
    previous_shape_degenerate, condition, _ = _shape_degeneracy(previous, yaw)

    matched_count = 0
    residual_m = math.inf
    for _ in range(max(1, int(max_iterations))):
        transformed = current @ _rotation(yaw).T + translation
        nearest, mask = _nearest_indices(
            previous,
            transformed,
            max_match_distance_m=max_match_distance_m,
        )
        matched_count = int(mask.sum())
        if matched_count < min_match_count:
            break
        source = current[mask]
        target = previous[nearest]
        rotation, translation = _fit_transform(source, target)
        yaw = _yaw_from_rotation(rotation)
        residuals = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
        residual_m = float(np.mean(residuals)) if len(residuals) else math.inf

    quality = matched_count / float(max(len(current), 1))
    along_degenerate, condition, _ = _shape_degeneracy(previous, yaw)
    along_degenerate = along_degenerate or previous_shape_degenerate
    forward_variance = max(0.05, residual_m * residual_m + 0.01)
    lateral_variance = forward_variance
    if along_degenerate:
        forward_variance = max(forward_variance, min(100.0, condition * 0.05))
    yaw_variance = max(math.radians(0.5) ** 2, residual_m * residual_m * 0.01)
    valid = (
        matched_count >= min_match_count
        and quality >= min_quality
        and residual_m <= max_residual_m
        and not (reject_degenerate and along_degenerate)
    )
    reason = ""
    if matched_count < min_match_count:
        reason = "insufficient_matches"
    elif quality < min_quality:
        reason = "low_quality"
    elif residual_m > max_residual_m:
        reason = "high_residual"
    elif reject_degenerate and along_degenerate:
        reason = "along_degenerate"
    return RelativeOdometryEstimate(
        forward_m=float(translation[0]),
        lateral_m=float(translation[1]),
        yaw_rad=float(yaw),
        residual_m=residual_m,
        quality=quality,
        forward_variance_m2=forward_variance,
        lateral_variance_m2=lateral_variance,
        yaw_variance_rad2=yaw_variance,
        along_degenerate=along_degenerate,
        is_valid=valid,
        reason=reason,
    )
