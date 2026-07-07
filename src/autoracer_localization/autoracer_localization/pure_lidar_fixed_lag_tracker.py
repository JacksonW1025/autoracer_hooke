"""Small pure-LiDAR fixed-lag multi-hypothesis tracker core.

The runtime ROS node can be built around this module, but the core is kept
dependency-free so W10 basin-selection behavior is unit-testable without ROS.
It intentionally does not use GNSS, GT, or future frames.
"""

from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
from typing import Any

from .lidar_relative_odometry import (
    RelativeOdometryEstimate,
    estimate_scan_to_scan_motion_2d,
)


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def planar_delta(from_pose: "Pose2D", to_pose: "Pose2D") -> tuple[float, float, float]:
    dx = to_pose.x - from_pose.x
    dy = to_pose.y - from_pose.y
    cy = math.cos(from_pose.yaw)
    sy = math.sin(from_pose.yaw)
    forward = cy * dx + sy * dy
    lateral = -sy * dx + cy * dy
    return forward, lateral, normalize_angle(to_pose.yaw - from_pose.yaw)


def scan_point_count_indicates_degeneracy(
    *,
    sampled_point_count: int,
    enabled: bool,
    min_sampled_points: int,
) -> bool:
    if not enabled:
        return False
    return int(sampled_point_count) < max(1, int(min_sampled_points))


def needs_pointcloud_subscription(
    *,
    enable_scan_submap_residual: bool,
    enable_scan_point_count_degeneracy: bool,
    enable_lro_forward_correction: bool,
    enable_scan_geometry_degeneracy: bool = False,
) -> bool:
    return (
        bool(enable_scan_submap_residual)
        or bool(enable_scan_point_count_degeneracy)
        or bool(enable_lro_forward_correction)
        or bool(enable_scan_geometry_degeneracy)
    )


def scan_submap_anchor_is_valid(
    *,
    enabled: bool,
    anchor_stamp_sec: float | None,
    current_stamp_sec: float | None,
    max_age_sec: float,
) -> bool:
    """Return whether a frozen high-confidence submap can be used causally."""

    if not enabled or anchor_stamp_sec is None or current_stamp_sec is None:
        return False
    if not math.isfinite(anchor_stamp_sec) or not math.isfinite(current_stamp_sec):
        return False
    return 0.0 <= current_stamp_sec - anchor_stamp_sec <= max(0.0, float(max_age_sec))


def apply_degenerate_velocity_scale(
    velocity_mps: float,
    *,
    degenerate: bool,
    enabled: bool,
    scale: float,
) -> float:
    """Scale dead-reckoning velocity only while a causal scan degeneracy is active."""

    if not enabled or not degenerate:
        return velocity_mps
    if not math.isfinite(velocity_mps):
        return 0.0
    bounded_scale = max(0.0, min(1.0, float(scale)))
    return velocity_mps * bounded_scale


def motion_dt_is_usable(dt_sec: float, max_dt_sec: float) -> bool:
    """Accept causal motion increments while tolerating replay timestamp jitter."""

    if not math.isfinite(dt_sec) or not math.isfinite(max_dt_sec):
        return False
    return 0.0 < dt_sec <= max(0.0, float(max_dt_sec))


def apply_lro_forward_correction(
    pose: "Pose2D",
    estimate: RelativeOdometryEstimate,
    *,
    predicted_forward_m: float,
    enabled: bool,
    gain: float,
    max_correction_m: float,
) -> "Pose2D":
    """Apply only the bounded along delta between LRO and wheel prediction."""

    if not enabled or not estimate.is_valid or estimate.along_degenerate:
        return pose
    if not math.isfinite(estimate.forward_m) or not math.isfinite(predicted_forward_m):
        return pose
    correction_m = (estimate.forward_m - predicted_forward_m) * max(0.0, min(1.0, gain))
    bound_m = abs(float(max_correction_m))
    correction_m = max(-bound_m, min(bound_m, correction_m))
    if abs(correction_m) <= 1e-9:
        return pose
    return Pose2D(
        stamp_sec=pose.stamp_sec,
        x=pose.x + math.cos(pose.yaw) * correction_m,
        y=pose.y + math.sin(pose.yaw) * correction_m,
        yaw=pose.yaw,
    )


def update_twist_bias_estimate(
    *,
    current_bias_mps: float,
    anchor_pose: "Pose2D",
    current_pose: "Pose2D",
    integrated_forward_m: float,
    dt_sec: float,
    alpha: float,
    max_abs_mps: float,
    max_step_mps: float,
    min_dt_sec: float,
    max_dt_sec: float,
    max_lateral_m: float,
    max_yaw_rad: float,
) -> TwistBiasLearningResult:
    """Estimate longitudinal velocity bias from two trusted causal pose anchors."""

    if dt_sec < min_dt_sec:
        return TwistBiasLearningResult(current_bias_mps, False, "dt_too_short")
    if dt_sec > max_dt_sec:
        return TwistBiasLearningResult(current_bias_mps, False, "dt_too_long")
    observed_forward_m, lateral_m, yaw_delta = planar_delta(anchor_pose, current_pose)
    if abs(lateral_m) > max_lateral_m:
        return TwistBiasLearningResult(current_bias_mps, False, "lateral_too_large")
    if abs(yaw_delta) > max_yaw_rad:
        return TwistBiasLearningResult(current_bias_mps, False, "yaw_too_large")
    bias_estimate_mps = (observed_forward_m - integrated_forward_m) / max(dt_sec, 1.0e-9)
    if not math.isfinite(bias_estimate_mps):
        return TwistBiasLearningResult(current_bias_mps, False, "nonfinite_estimate")
    limit = abs(float(max_abs_mps))
    bias_estimate_mps = max(-limit, min(limit, bias_estimate_mps))
    gain = max(0.0, min(1.0, float(alpha)))
    target = (1.0 - gain) * current_bias_mps + gain * bias_estimate_mps
    step = target - current_bias_mps
    step_limit = abs(float(max_step_mps))
    if step_limit > 0.0:
        step = max(-step_limit, min(step_limit, step))
    new_bias = max(-limit, min(limit, current_bias_mps + step))
    return TwistBiasLearningResult(new_bias, True, "updated", bias_estimate_mps)


@dataclass(frozen=True)
class ScanGeometryCertificate:
    point_count: int
    along_span_m: float
    cross_span_m: float
    left_count: int
    right_count: int
    one_sided: bool
    along_to_cross_ratio: float
    along_degenerate: bool


def scan_geometry_certificate(
    points: list[tuple[float, ...]],
    *,
    min_points: int = 20,
    min_cross_side_points: int = 5,
    min_along_to_cross_ratio: float = 3.0,
    min_along_span_m: float = 20.0,
    max_cross_span_m: float = 8.0,
) -> ScanGeometryCertificate:
    """Diagnose one-sided route-frame scan geometry from the current scan only."""

    coords: list[tuple[float, float]] = []
    for point in points:
        if len(point) < 2:
            continue
        x = float(point[0])
        y = float(point[1])
        if math.isfinite(x) and math.isfinite(y):
            coords.append((x, y))
    if not coords:
        return ScanGeometryCertificate(0, 0.0, 0.0, 0, 0, True, math.inf, True)
    xs = [item[0] for item in coords]
    ys = [item[1] for item in coords]
    along_span = max(xs) - min(xs)
    cross_span = max(ys) - min(ys)
    left_count = sum(1 for _, y in coords if y > 0.5)
    right_count = sum(1 for _, y in coords if y < -0.5)
    one_sided = min(left_count, right_count) < max(0, int(min_cross_side_points))
    ratio = along_span / max(cross_span, 1.0e-6)
    one_sided_low_along_evidence = one_sided and along_span <= min_along_span_m
    one_sided_corridor_like = (
        one_sided
        and along_span >= min_along_span_m
        and cross_span <= max_cross_span_m
        and ratio >= min_along_to_cross_ratio
    )
    degenerate = (
        len(coords) < max(1, int(min_points))
        or one_sided_low_along_evidence
        or one_sided_corridor_like
    )
    return ScanGeometryCertificate(
        point_count=len(coords),
        along_span_m=along_span,
        cross_span_m=cross_span,
        left_count=left_count,
        right_count=right_count,
        one_sided=one_sided,
        along_to_cross_ratio=ratio,
        along_degenerate=degenerate,
    )


@dataclass(frozen=True)
class Pose2D:
    stamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MotionDelta:
    dt_sec: float
    forward_m: float
    lateral_m: float
    yaw_rad: float
    covariance_scale: float = 1.0


@dataclass(frozen=True)
class TwistBiasLearningResult:
    bias_mps: float
    updated: bool
    reason: str
    estimate_mps: float | None = None


@dataclass(frozen=True)
class NdtCandidate:
    pose: Pose2D
    score: float
    converged: bool = True
    initial_to_result_m: float = 0.0
    initial_to_result_yaw_rad: float = 0.0
    rejection_reason: str = ""
    localizability_along_variance_m2: float = 0.0
    localizability_cross_variance_m2: float = 0.0
    nearest_voxel_transformation_likelihood: float | None = None
    source: str = "ndt"


@dataclass(frozen=True)
class RoutePrior:
    progress_m: float
    cross_track_m: float
    yaw_error_rad: float
    is_valid: bool = True


@dataclass(frozen=True)
class WeakPriorPenalty:
    penalty: float
    distance_m: float
    is_valid: bool = True


@dataclass(frozen=True)
class RouteProjection:
    progress_m: float
    cross_track_m: float
    yaw_error_rad: float
    route_yaw_rad: float
    distance_m: float
    is_valid: bool = True


class RouteCrossTargetLearner:
    """Causal IIR estimator for lane offset relative to the map route."""

    def __init__(self, *, alpha: float, gate_m: float, abs_limit_m: float) -> None:
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.gate_m = abs(float(gate_m))
        self.abs_limit_m = abs(float(abs_limit_m))
        self.value: float | None = None
        self.count = 0

    @property
    def has_value(self) -> bool:
        return self.value is not None

    def value_or_none(self) -> float | None:
        return self.value

    def update(self, cross_track_m: float, *, along_degenerate: bool) -> float | None:
        if along_degenerate or not math.isfinite(cross_track_m):
            return self.value
        if math.isfinite(self.gate_m) and abs(cross_track_m) > self.gate_m:
            return self.value
        sample = cross_track_m
        if math.isfinite(self.abs_limit_m):
            sample = max(-self.abs_limit_m, min(self.abs_limit_m, sample))
        if self.value is None:
            self.value = sample
        else:
            self.value = (1.0 - self.alpha) * self.value + self.alpha * sample
        self.count += 1
        return self.value


@dataclass(frozen=True)
class RelativeResidual:
    xy_m: float
    yaw_rad: float
    quality: float
    is_valid: bool = True


Point2D = tuple[float, float]


class LightweightScanSubmap:
    """Causal 2D scan-to-submap consistency factor.

    This is intentionally small: it is not a SLAM backend and it does not
    estimate a correction.  It only scores whether a candidate pose makes the
    current LiDAR scan overlap with a rolling map built from past accepted
    tracker poses.  That is the missing evidence needed to distinguish W10
    along-progress basins without GNSS/GT/future frames.
    """

    def __init__(
        self,
        *,
        voxel_size_m: float = 0.8,
        max_cells: int = 20000,
        neighbor_radius_cells: int = 1,
        unmatched_penalty_m: float = 4.0,
        enable_3d_points: bool = False,
    ) -> None:
        self.voxel_size_m = max(float(voxel_size_m), 0.05)
        self.max_cells = max(int(max_cells), 1)
        self.neighbor_radius_cells = max(int(neighbor_radius_cells), 0)
        self.unmatched_penalty_m = max(float(unmatched_penalty_m), self.voxel_size_m)
        self.enable_3d_points = bool(enable_3d_points)
        self._cells: dict[tuple[int, int], int] = {}
        self._fifo: deque[tuple[int, int]] = deque()
        self._points2d: deque[tuple[float, float]] = deque()
        self._points3d: deque[tuple[float, float, float]] = deque()
        self._gaussian_cell_cache: dict[
            tuple[int, int], dict[tuple[int, int], tuple[object, object]]
        ] = {}

    @property
    def cell_count(self) -> int:
        return len(self._cells)

    def clone(self) -> "LightweightScanSubmap":
        other = LightweightScanSubmap(
            voxel_size_m=self.voxel_size_m,
            max_cells=self.max_cells,
            neighbor_radius_cells=self.neighbor_radius_cells,
            unmatched_penalty_m=self.unmatched_penalty_m,
            enable_3d_points=self.enable_3d_points,
        )
        other._cells = dict(self._cells)
        other._fifo = deque(self._fifo)
        other._points2d = deque(self._points2d)
        other._points3d = deque(self._points3d)
        return other

    def add_scan(self, points: list[Point2D], pose: Pose2D) -> int:
        added = 0
        for point in points:
            world = self._transform(point, pose)
            added += self._add_world_point(world)
            if self.enable_3d_points and len(point) >= 3:
                self._points3d.append((world[0], world[1], float(point[2])))
                while len(self._points3d) > self.max_cells:
                    self._points3d.popleft()
        return added

    def add_world_points(self, points: list[Point2D]) -> int:
        added = 0
        for point in points:
            added += self._add_world_point((float(point[0]), float(point[1])))
        return added

    def _add_world_point(self, world: Point2D) -> int:
        self._gaussian_cell_cache.clear()
        cell = self._cell_for_point(world)
        added = 0
        if cell not in self._cells:
            self._fifo.append(cell)
            added = 1
        self._cells[cell] = self._cells.get(cell, 0) + 1
        self._points2d.append((world[0], world[1]))
        while len(self._points2d) > self.max_cells:
            self._points2d.popleft()
        while len(self._cells) > self.max_cells and self._fifo:
            old = self._fifo.popleft()
            count = self._cells.get(old, 0)
            if count <= 1:
                self._cells.pop(old, None)
            else:
                self._cells[old] = count - 1
        return added

    def _gaussian_cells(
        self,
        *,
        min_cell_points: int,
        max_map_points: int,
    ) -> dict[tuple[int, int], tuple[object, object]]:
        """Return cached Gaussian map cells for local-NDT candidate scoring."""

        key = (int(min_cell_points), int(max_map_points))
        cached = self._gaussian_cell_cache.get(key)
        if cached is not None:
            return cached

        try:
            import numpy as np
        except Exception:
            return {}

        map_items = list(self._points2d)
        map_stride = max(1, len(map_items) // max(int(max_map_points), 1))
        map_points = map_items[::map_stride][:max_map_points]
        if len(map_points) < max(8, min_cell_points):
            return {}

        cell_points: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for wx, wy in map_points:
            cell = self._cell_for_point((wx, wy))
            cell_points.setdefault(cell, []).append((wx, wy))

        gaussian_cells: dict[tuple[int, int], tuple[object, object]] = {}
        reg = max(self.voxel_size_m * 0.35, 0.05) ** 2
        for cell, values in cell_points.items():
            if len(values) < min_cell_points:
                continue
            array = np.asarray(values, dtype=float)
            mean = array.mean(axis=0)
            centered = array - mean
            cov = (centered.T @ centered) / max(len(values) - 1, 1)
            cov = cov + np.eye(2) * reg
            try:
                inv_cov = np.linalg.inv(cov)
            except Exception:
                continue
            gaussian_cells[cell] = (mean, inv_cov)
        self._gaussian_cell_cache[key] = gaussian_cells
        return gaussian_cells

    def residual(self, points: list[Point2D], pose: Pose2D, *, max_points: int = 256) -> RelativeResidual:
        if not points or not self._cells:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)
        stride = max(1, len(points) // max(int(max_points), 1))
        selected = points[::stride][:max_points]
        if not selected:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)

        total = 0.0
        matched = 0
        for point in selected:
            world = self._transform(point, pose)
            distance = self._nearest_cell_distance(world)
            if distance is None:
                total += self.unmatched_penalty_m
                continue
            matched += 1
            total += distance
        quality = matched / float(len(selected))
        return RelativeResidual(total / float(len(selected)), 0.0, quality, True)

    def longitudinal_profile_residual(
        self,
        points: list[Point2D],
        pose: Pose2D,
        *,
        max_points: int = 256,
        max_profile_cells: int = 4000,
        lateral_bin_m: float = 0.8,
        unmatched_penalty_m: float | None = None,
    ) -> RelativeResidual:
        """Score along-progress consistency against the rolling submap.

        Nearest occupied-cell distance is intentionally tolerant to sliding
        along a wall/edge, which is exactly W10's failure mode.  This residual
        transforms submap cells into the candidate local frame, buckets them by
        lateral position, and compares the current scan's forward coordinate
        against the closest historical forward coordinate in the same lateral
        band.  It remains causal because the submap only contains past scans.
        """

        if not points or not self._cells:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)
        lateral_bin_m = max(float(lateral_bin_m), 0.1)
        penalty = float(unmatched_penalty_m) if unmatched_penalty_m is not None else self.unmatched_penalty_m
        profile: dict[int, list[float]] = {}
        cell_items = list(self._cells.keys())
        stride_cells = max(1, len(cell_items) // max(int(max_profile_cells), 1))
        cy = math.cos(pose.yaw)
        sy = math.sin(pose.yaw)
        for cell in cell_items[::stride_cells][:max_profile_cells]:
            wx = (cell[0] + 0.5) * self.voxel_size_m
            wy = (cell[1] + 0.5) * self.voxel_size_m
            dx = wx - pose.x
            dy = wy - pose.y
            forward = cy * dx + sy * dy
            lateral = -sy * dx + cy * dy
            lateral_key = int(round(lateral / lateral_bin_m))
            profile.setdefault(lateral_key, []).append(forward)
        if not profile:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)
        for values in profile.values():
            values.sort()

        stride = max(1, len(points) // max(int(max_points), 1))
        selected = points[::stride][:max_points]
        if not selected:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)

        total = 0.0
        matched = 0
        for forward, lateral in selected:
            lateral_key = int(round(lateral / lateral_bin_m))
            candidates: list[float] = []
            for key in (lateral_key - 1, lateral_key, lateral_key + 1):
                values = profile.get(key)
                if values:
                    candidates.extend(values)
            if not candidates:
                total += penalty
                continue
            best = min(abs(value - forward) for value in candidates)
            total += min(best, penalty)
            matched += 1
        quality = matched / float(len(selected))
        return RelativeResidual(total / float(len(selected)), 0.0, quality, True)

    def local_icp_residual(
        self,
        points: list[Point2D],
        pose: Pose2D,
        *,
        max_points: int = 256,
        max_map_points: int = 4000,
        max_match_distance_m: float = 3.0,
        correction_penalty_weight: float = 0.5,
    ) -> RelativeResidual:
        """One-shot point-to-point ICP residual against the causal submap.

        The residual is used only for scoring candidates.  It does not mutate
        the pose.  Wrong along basins can look locally plausible under nearest
        occupied-cell scoring; requiring a large ICP correction is therefore
        penalized explicitly.
        """

        if not points or not self._cells:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)
        try:
            import numpy as np
            from scipy.spatial import cKDTree
        except Exception:
            return self.residual(points, pose, max_points=max_points)

        scan_stride = max(1, len(points) // max(int(max_points), 1))
        selected_scan = points[::scan_stride][:max_points]
        if len(selected_scan) < 4:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)

        cell_items = list(self._cells.keys())
        map_stride = max(1, len(cell_items) // max(int(max_map_points), 1))
        centers = [
            ((cell[0] + 0.5) * self.voxel_size_m, (cell[1] + 0.5) * self.voxel_size_m)
            for cell in cell_items[::map_stride][:max_map_points]
        ]
        if len(centers) < 4:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)

        scan_world = np.array([self._transform(point, pose) for point in selected_scan], dtype=float)
        map_points = np.array(centers, dtype=float)
        distances, indices = cKDTree(map_points).query(scan_world, k=1)
        mask = distances <= max(float(max_match_distance_m), self.voxel_size_m)
        if int(mask.sum()) < 4:
            quality = float(mask.mean()) if len(mask) else 0.0
            return RelativeResidual(self.unmatched_penalty_m, 0.0, quality, False)

        src = scan_world[mask]
        dst = map_points[indices[mask]]
        src_centroid = src.mean(axis=0)
        dst_centroid = dst.mean(axis=0)
        src_centered = src - src_centroid
        dst_centered = dst - dst_centroid
        covariance = src_centered.T @ dst_centered
        try:
            u_matrix, _singular_values, vt_matrix = np.linalg.svd(covariance)
        except Exception:
            return RelativeResidual(self.unmatched_penalty_m, 0.0, float(mask.mean()), False)
        rotation = vt_matrix.T @ u_matrix.T
        if np.linalg.det(rotation) < 0.0:
            vt_matrix[-1, :] *= -1.0
            rotation = vt_matrix.T @ u_matrix.T
        translation = dst_centroid - rotation @ src_centroid
        transformed = (rotation @ src.T).T + translation
        residuals = np.linalg.norm(transformed - dst, axis=1)
        rmse = float(math.sqrt(float(np.mean(residuals * residuals))))
        correction_xy = float(np.linalg.norm(translation))
        yaw_correction = float(math.atan2(rotation[1, 0], rotation[0, 0]))
        score = rmse + max(0.0, float(correction_penalty_weight)) * (
            correction_xy + abs(yaw_correction)
        )
        return RelativeResidual(
            xy_m=score,
            yaw_rad=abs(yaw_correction),
            quality=float(mask.mean()),
            is_valid=True,
        )

    def local_icp_pose_candidate(
        self,
        points: list[Point2D],
        seed_pose: Pose2D,
        *,
        max_points: int = 256,
        max_map_points: int = 4000,
        max_match_distance_m: float = 3.0,
        max_correction_m: float = 1.0,
        max_yaw_correction_rad: float = math.radians(2.0),
        min_quality: float = 0.35,
        correction_penalty_weight: float = 0.5,
        match_in_3d: bool = False,
    ) -> tuple[Pose2D, RelativeResidual] | None:
        """Estimate a causal scan-to-submap pose candidate.

        Unlike :meth:`local_icp_residual`, this returns a corrected pose that
        can be scored by the fixed-lag tracker as an independent local-submap
        measurement.  The correction is tightly gated because applying a bad
        ICP solution directly was previously catastrophic in W10.
        """

        if not points or not self._cells:
            return None
        try:
            import numpy as np
            from scipy.spatial import cKDTree
        except Exception:
            return None

        scan_stride = max(1, len(points) // max(int(max_points), 1))
        selected_scan = points[::scan_stride][:max_points]
        if len(selected_scan) < 4:
            return None

        use_3d = bool(match_in_3d and self._points3d and len(selected_scan[0]) >= 3)
        if use_3d:
            map_items = list(self._points3d)
            map_stride = max(1, len(map_items) // max(int(max_map_points), 1))
            centers = map_items[::map_stride][:max_map_points]
            scan_world = np.array(
                [
                    (*self._transform(point, seed_pose), float(point[2]))
                    for point in selected_scan
                    if len(point) >= 3
                ],
                dtype=float,
            )
        else:
            cell_items = list(self._cells.keys())
            map_stride = max(1, len(cell_items) // max(int(max_map_points), 1))
            centers = [
                ((cell[0] + 0.5) * self.voxel_size_m, (cell[1] + 0.5) * self.voxel_size_m)
                for cell in cell_items[::map_stride][:max_map_points]
            ]
            scan_world = np.array([self._transform(point, seed_pose) for point in selected_scan], dtype=float)
        if len(centers) < 4 or len(scan_world) < 4:
            return None

        map_points = np.array(centers, dtype=float)
        distances, indices = cKDTree(map_points).query(scan_world, k=1)
        mask = distances <= max(float(max_match_distance_m), self.voxel_size_m)
        quality = float(mask.mean()) if len(mask) else 0.0
        if int(mask.sum()) < 4 or quality < min_quality:
            return None

        src = scan_world[mask]
        dst = map_points[indices[mask]]
        src_xy = src[:, :2] if use_3d else src
        dst_xy = dst[:, :2] if use_3d else dst
        src_centroid = src_xy.mean(axis=0)
        dst_centroid = dst_xy.mean(axis=0)
        src_centered = src_xy - src_centroid
        dst_centered = dst_xy - dst_centroid
        covariance = src_centered.T @ dst_centered
        try:
            u_matrix, _singular_values, vt_matrix = np.linalg.svd(covariance)
        except Exception:
            return None
        rotation = vt_matrix.T @ u_matrix.T
        if np.linalg.det(rotation) < 0.0:
            vt_matrix[-1, :] *= -1.0
            rotation = vt_matrix.T @ u_matrix.T
        translation = dst_centroid - rotation @ src_centroid

        origin = np.array([seed_pose.x, seed_pose.y], dtype=float)
        corrected_origin = rotation @ origin + translation
        yaw_correction = float(math.atan2(rotation[1, 0], rotation[0, 0]))
        correction_xy = float(np.linalg.norm(corrected_origin - origin))
        if correction_xy > max(float(max_correction_m), 0.0):
            return None
        if abs(yaw_correction) > abs(float(max_yaw_correction_rad)):
            return None

        transformed_xy = (rotation @ src_xy.T).T + translation
        if use_3d:
            residuals = np.linalg.norm(
                np.column_stack((transformed_xy, src[:, 2])) - dst,
                axis=1,
            )
        else:
            residuals = np.linalg.norm(transformed_xy - dst_xy, axis=1)
        rmse = float(math.sqrt(float(np.mean(residuals * residuals))))
        score = rmse + max(0.0, float(correction_penalty_weight)) * (
            correction_xy + abs(yaw_correction)
        )
        pose = Pose2D(
            stamp_sec=seed_pose.stamp_sec,
            x=float(corrected_origin[0]),
            y=float(corrected_origin[1]),
            yaw=normalize_angle(seed_pose.yaw + yaw_correction),
        )
        return pose, RelativeResidual(
            xy_m=score,
            yaw_rad=abs(yaw_correction),
            quality=quality,
            is_valid=True,
        )

    def local_ndt_pose_candidates(
        self,
        points: list[Point2D],
        seed_pose: Pose2D,
        *,
        forward_offsets_m: tuple[float, ...],
        lateral_offsets_m: tuple[float, ...],
        yaw_offsets_rad: tuple[float, ...],
        max_points: int = 256,
        max_map_points: int = 6000,
        min_quality: float = 0.35,
        min_cell_points: int = 5,
        max_candidates: int = 5,
        reject_boundary_best: bool = False,
        min_second_best_score_margin: float = 0.0,
        profile_score_weight: float = 0.0,
    ) -> list[tuple[Pose2D, RelativeResidual]]:
        """Return bounded local-NDT scan-to-submap candidates.

        This is still intentionally small, but it is stronger than nearest
        occupancy or one-shot ICP: the rolling submap is converted into local
        Gaussian cells, and candidate offsets are scored by a regularized
        Mahalanobis distance.  It stays causal because the submap only contains
        past scans.
        """

        if not points or len(self._points2d) < max(8, min_cell_points):
            return []
        try:
            import numpy as np
        except Exception:
            return []

        gaussian_cells = self._gaussian_cells(
            min_cell_points=min_cell_points,
            max_map_points=max_map_points,
        )
        if not gaussian_cells:
            return []

        scan_stride = max(1, len(points) // max(int(max_points), 1))
        selected_scan = points[::scan_stride][:max_points]
        if len(selected_scan) < 4:
            return []

        def score_pose(pose: Pose2D) -> RelativeResidual:
            total = 0.0
            matched = 0
            for point in selected_scan:
                world = self._transform(point, pose)
                base_cell = self._cell_for_point(world)
                best = None
                for dx in range(-self.neighbor_radius_cells, self.neighbor_radius_cells + 1):
                    for dy in range(-self.neighbor_radius_cells, self.neighbor_radius_cells + 1):
                        gaussian = gaussian_cells.get((base_cell[0] + dx, base_cell[1] + dy))
                        if gaussian is None:
                            continue
                        mean, inv_cov = gaussian
                        delta = np.asarray(world, dtype=float) - mean
                        value = float(delta.T @ inv_cov @ delta)
                        if best is None or value < best:
                            best = value
                if best is None:
                    total += 9.0
                    continue
                matched += 1
                total += min(best, 9.0)
            quality = matched / float(len(selected_scan))
            if quality <= 0.0:
                return RelativeResidual(self.unmatched_penalty_m, 0.0, 0.0, False)
            # sqrt keeps the residual in meter-like scale for tracker scoring.
            return RelativeResidual(
                xy_m=math.sqrt(total / float(len(selected_scan))) * self.voxel_size_m,
                yaw_rad=0.0,
                quality=quality,
                is_valid=quality >= min_quality,
            )

        scored: list[tuple[float, Pose2D, RelativeResidual, float, float, float]] = []
        cy = math.cos(seed_pose.yaw)
        sy = math.sin(seed_pose.yaw)
        for forward in forward_offsets_m:
            for lateral in lateral_offsets_m:
                for yaw_offset in yaw_offsets_rad:
                    pose = Pose2D(
                        stamp_sec=seed_pose.stamp_sec,
                        x=seed_pose.x + forward * cy - lateral * sy,
                        y=seed_pose.y + forward * sy + lateral * cy,
                        yaw=normalize_angle(seed_pose.yaw + yaw_offset),
                    )
                    residual = score_pose(pose)
                    if not residual.is_valid:
                        continue
                    profile_weight = max(0.0, float(profile_score_weight))
                    if profile_weight > 0.0:
                        profile_residual = self.longitudinal_profile_residual(
                            points,
                            pose,
                            max_points=max_points,
                        )
                        if profile_residual.is_valid:
                            residual = RelativeResidual(
                                xy_m=residual.xy_m
                                + profile_weight * profile_residual.xy_m,
                                yaw_rad=max(residual.yaw_rad, profile_residual.yaw_rad),
                                quality=min(residual.quality, profile_residual.quality),
                                is_valid=True,
                            )
                    correction_penalty = 0.2 * (
                        math.hypot(forward, lateral) + abs(yaw_offset)
                    )
                    score = residual.xy_m - min(1.0, residual.quality) + correction_penalty
                    scored.append((score, pose, residual, float(forward), float(lateral), float(yaw_offset)))
        scored.sort(key=lambda item: item[0])
        if not scored:
            return []
        best_score, _best_pose, _best_residual, best_forward, best_lateral, best_yaw = scored[0]
        if reject_boundary_best:
            forward_values = sorted(set(float(value) for value in forward_offsets_m))
            lateral_values = sorted(set(float(value) for value in lateral_offsets_m))
            yaw_values = sorted(set(float(value) for value in yaw_offsets_rad))
            forward_on_boundary = (
                len(forward_values) > 1
                and (best_forward <= forward_values[0] or best_forward >= forward_values[-1])
            )
            lateral_on_boundary = (
                len(lateral_values) > 1
                and (best_lateral <= lateral_values[0] or best_lateral >= lateral_values[-1])
            )
            yaw_on_boundary = (
                len(yaw_values) > 1 and (best_yaw <= yaw_values[0] or best_yaw >= yaw_values[-1])
            )
            if forward_on_boundary or lateral_on_boundary or yaw_on_boundary:
                return []
        if min_second_best_score_margin > 0.0:
            if len(scored) < 2:
                return []
            second_best_score = scored[1][0]
            if second_best_score - best_score < min_second_best_score_margin:
                return []
        return [
            (pose, residual)
            for _score, pose, residual, _forward, _lateral, _yaw in scored[: max(1, int(max_candidates))]
        ]

    def refine_pose(
        self,
        points: list[Point2D],
        seed_pose: Pose2D,
        *,
        forward_offsets_m: tuple[float, ...],
        lateral_offsets_m: tuple[float, ...],
        yaw_offsets_rad: tuple[float, ...],
        max_points: int = 256,
        profile_score_weight: float = 0.0,
    ) -> tuple[Pose2D, RelativeResidual]:
        """Return the local pose with the best causal scan-to-submap residual."""

        best_pose = seed_pose
        profile_weight = max(0.0, float(profile_score_weight))

        def combined_residual(pose: Pose2D) -> RelativeResidual:
            residual = self.residual(points, pose, max_points=max_points)
            if profile_weight <= 0.0:
                return residual
            profile = self.longitudinal_profile_residual(
                points,
                pose,
                max_points=max_points,
            )
            if not profile.is_valid:
                return residual
            if not residual.is_valid:
                return profile
            return RelativeResidual(
                xy_m=residual.xy_m + profile_weight * profile.xy_m,
                yaw_rad=max(residual.yaw_rad, profile.yaw_rad),
                quality=min(residual.quality, profile.quality),
                is_valid=True,
            )

        best_residual = combined_residual(seed_pose)
        best_score = self._residual_score(best_residual)
        cy = math.cos(seed_pose.yaw)
        sy = math.sin(seed_pose.yaw)
        for forward_m in forward_offsets_m:
            for lateral_m in lateral_offsets_m:
                for yaw_rad in yaw_offsets_rad:
                    if forward_m == 0.0 and lateral_m == 0.0 and yaw_rad == 0.0:
                        continue
                    candidate_pose = Pose2D(
                        stamp_sec=seed_pose.stamp_sec,
                        x=seed_pose.x + cy * forward_m - sy * lateral_m,
                        y=seed_pose.y + sy * forward_m + cy * lateral_m,
                        yaw=normalize_angle(seed_pose.yaw + yaw_rad),
                    )
                    residual = combined_residual(candidate_pose)
                    score = self._residual_score(residual)
                    if score < best_score:
                        best_score = score
                        best_pose = candidate_pose
                best_residual = residual
        return best_pose, best_residual

    def profile_along_correction(
        self,
        points: list[Point2D],
        seed_pose: Pose2D,
        *,
        forward_offsets_m: tuple[float, ...],
        max_points: int = 256,
        min_quality: float = 0.0,
        min_second_best_margin_m: float = 0.0,
    ) -> tuple[Pose2D, RelativeResidual] | None:
        """Return a certified along-only correction from the causal profile residual.

        The correction is accepted only when the best offset is inside the search
        interval and clearly better than the second-best offset.  This prevents
        the unsafe boundary latching observed when direct scan-submap correction
        is used in W10.
        """

        offsets = tuple(float(value) for value in forward_offsets_m)
        if len(offsets) < 3:
            return None
        sorted_offsets = sorted(set(offsets))
        if len(sorted_offsets) < 3:
            return None
        cy = math.cos(seed_pose.yaw)
        sy = math.sin(seed_pose.yaw)
        scored: list[tuple[float, Pose2D, RelativeResidual, float]] = []
        for offset in sorted_offsets:
            pose = Pose2D(
                stamp_sec=seed_pose.stamp_sec,
                x=seed_pose.x + cy * offset,
                y=seed_pose.y + sy * offset,
                yaw=seed_pose.yaw,
            )
            residual = self.longitudinal_profile_residual(
                points,
                pose,
                max_points=max_points,
            )
            if not residual.is_valid or residual.quality < min_quality:
                continue
            scored.append((residual.xy_m, pose, residual, offset))
        if len(scored) < 2:
            return None
        scored.sort(key=lambda item: item[0])
        best_score, best_pose, best_residual, best_offset = scored[0]
        if best_offset <= sorted_offsets[0] or best_offset >= sorted_offsets[-1]:
            return None
        second_score = scored[1][0]
        if second_score - best_score < max(0.0, float(min_second_best_margin_m)):
            return None
        return best_pose, best_residual

    def _residual_score(self, residual: RelativeResidual) -> float:
        if not residual.is_valid:
            return self.unmatched_penalty_m
        return residual.xy_m - 0.5 * self.voxel_size_m * max(0.0, min(1.0, residual.quality))

    def _nearest_cell_distance(self, point: Point2D) -> float | None:
        cx, cy = self._cell_for_point(point)
        best: float | None = None
        for dx in range(-self.neighbor_radius_cells, self.neighbor_radius_cells + 1):
            for dy in range(-self.neighbor_radius_cells, self.neighbor_radius_cells + 1):
                cell = (cx + dx, cy + dy)
                if cell not in self._cells:
                    continue
                center = ((cell[0] + 0.5) * self.voxel_size_m, (cell[1] + 0.5) * self.voxel_size_m)
                distance = math.hypot(point[0] - center[0], point[1] - center[1])
                if best is None or distance < best:
                    best = distance
        return best

    def _cell_for_point(self, point: Point2D) -> tuple[int, int]:
        return (
            int(math.floor(point[0] / self.voxel_size_m)),
            int(math.floor(point[1] / self.voxel_size_m)),
        )

    @staticmethod
    def _transform(point: Point2D, pose: Pose2D) -> Point2D:
        cy = math.cos(pose.yaw)
        sy = math.sin(pose.yaw)
        return (
            pose.x + cy * point[0] - sy * point[1],
            pose.y + sy * point[0] + cy * point[1],
        )


@dataclass
class Hypothesis:
    id: int
    parent_id: int | None
    pose: Pose2D
    route_progress_m: float
    covariance_scale: float = 1.0
    score: float = 0.0
    age_frames: int = 0
    missed_updates: int = 0
    start_stamp_sec: float = 0.0
    history: tuple[Pose2D, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class TrackerConfig:
    max_hypotheses: int = 7
    fixed_lag_frames: int = 12
    propagation_covariance_growth: float = 0.15
    missed_update_penalty: float = 0.4
    ndt_score_weight: float = 1.0
    initial_to_result_penalty_weight: float = 0.35
    relative_residual_weight: float = 1.0
    route_cross_weight: float = 0.8
    route_yaw_weight: float = 0.2
    route_progress_weight: float = 0.0
    gnss_weak_prior_weight: float = 1.0
    route_progress_update_gain: float = 1.0
    route_progress_update_max_age_sec: float = math.inf
    enable_degenerate_along_remap: bool = False
    enable_candidate_localizability_along_remap: bool = False
    candidate_localizability_along_min_variance_m2: float = 1.0
    candidate_localizability_along_min_ratio: float = 4.0
    enable_candidate_low_score_along_remap: bool = False
    candidate_low_score_along_remap_threshold: float = 2.3
    degenerate_remap_use_route_frame: bool = False
    degenerate_remap_keep_predicted_yaw: bool = False
    degenerate_keep_predicted_yaw_only: bool = False
    degenerate_skip_ndt_candidates: bool = False
    degenerate_max_lateral_m: float = math.inf
    degenerate_max_yaw_rad: float = math.pi
    enable_submap_candidate_consistency_gate: bool = False
    submap_candidate_max_lateral_m: float = 1.5
    submap_candidate_max_yaw_rad: float = math.radians(3.0)
    submap_candidate_max_progress_innovation_m: float = 2.0
    submap_candidate_max_residual_m: float = 2.0
    submap_candidate_min_residual_improvement_m: float = 0.0
    enable_candidate_residual_consistency_gate: bool = False
    candidate_residual_max_m: float = 2.0
    candidate_residual_min_improvement_m: float = 0.2
    non_monotonic_progress_penalty: float = 8.0
    yaw_jump_weight: float = 0.5
    lateral_jump_weight: float = 0.5
    max_candidate_initial_to_result_m: float = 6.0
    max_candidate_yaw_rad: float = math.radians(12.0)
    min_candidate_score: float = -math.inf
    enable_not_converged_partial_candidates: bool = False
    not_converged_partial_min_nvtl: float = -math.inf
    max_route_progress_regression_m: float = 1.0
    best_switch_score_margin: float = 0.0


def candidate_is_usable(candidate: NdtCandidate, config: TrackerConfig) -> bool:
    """Return whether an NDT candidate is allowed to affect tracker state."""

    reason = str(candidate.rejection_reason or "")
    if not candidate.converged or reason:
        if not (
            config.enable_not_converged_partial_candidates
            and not candidate.converged
            and reason == "not_converged"
        ):
            return False
        nvtl = candidate.nearest_voxel_transformation_likelihood
        if nvtl is None or not math.isfinite(float(nvtl)):
            return False
        if float(nvtl) < float(config.not_converged_partial_min_nvtl):
            return False
    if candidate.rejection_reason and candidate.rejection_reason != "not_converged":
        return False
    if candidate.initial_to_result_m > config.max_candidate_initial_to_result_m:
        return False
    if candidate.score < config.min_candidate_score:
        return False
    return abs(candidate.initial_to_result_yaw_rad) <= config.max_candidate_yaw_rad


def candidate_can_refresh_scan_submap_anchor(candidate: NdtCandidate, config: TrackerConfig) -> bool:
    """Only fully converged usable NDT may freeze a stable anchor submap."""

    return bool(candidate.converged) and candidate_is_usable(candidate, config)


def candidate_indicates_along_degeneracy(candidate: NdtCandidate, config: TrackerConfig) -> bool:
    if config.enable_candidate_low_score_along_remap:
        score = (
            float(candidate.nearest_voxel_transformation_likelihood)
            if candidate.nearest_voxel_transformation_likelihood is not None
            else float(candidate.score)
        )
        threshold = float(config.candidate_low_score_along_remap_threshold)
        if math.isfinite(score) and math.isfinite(threshold) and score < threshold:
            return True
    if not config.enable_candidate_localizability_along_remap:
        return False
    along = float(candidate.localizability_along_variance_m2)
    cross = float(candidate.localizability_cross_variance_m2)
    if not math.isfinite(along) or along < config.candidate_localizability_along_min_variance_m2:
        return False
    if not math.isfinite(cross):
        return False
    return along / max(cross, 1.0e-6) >= config.candidate_localizability_along_min_ratio


def candidate_confidence_summary(
    candidates: list[NdtCandidate],
    config: TrackerConfig,
) -> dict[str, float | int]:
    """Summarize candidate evidence without changing tracker behavior.

    W10 debugging showed that scalar NDT likelihood alone is not a safe update
    gate.  This summary is intentionally diagnostic-only: it exposes the
    evidence distribution needed to design a later confidence certificate while
    keeping the restored baseline untouched.
    """

    usable_count = 0
    along_degenerate_count = 0
    converged_count = 0
    rejected_count = 0
    source_counts: dict[str, int] = {}
    reject_reason_counts: dict[str, int] = {}
    scores: list[float] = []
    nvtls: list[float] = []
    i2r_values: list[float] = []
    yaw_values: list[float] = []
    for candidate in candidates:
        source_counts[candidate.source] = source_counts.get(candidate.source, 0) + 1
        if candidate.converged:
            converged_count += 1
        reason = str(candidate.rejection_reason or "")
        if reason:
            rejected_count += 1
            reject_reason_counts[reason] = reject_reason_counts.get(reason, 0) + 1
        if candidate_is_usable(candidate, config):
            usable_count += 1
        if candidate_indicates_along_degeneracy(candidate, config):
            along_degenerate_count += 1
        score = float(candidate.score)
        if math.isfinite(score):
            scores.append(score)
        if candidate.nearest_voxel_transformation_likelihood is not None:
            nvtl = float(candidate.nearest_voxel_transformation_likelihood)
            if math.isfinite(nvtl):
                nvtls.append(nvtl)
        i2r = float(candidate.initial_to_result_m)
        if math.isfinite(i2r):
            i2r_values.append(i2r)
        yaw = abs(float(candidate.initial_to_result_yaw_rad))
        if math.isfinite(yaw):
            yaw_values.append(yaw)

    def min_or_nan(values: list[float]) -> float:
        return min(values) if values else math.nan

    def max_or_nan(values: list[float]) -> float:
        return max(values) if values else math.nan

    return {
        "candidate_usable_count": usable_count,
        "candidate_converged_count": converged_count,
        "candidate_rejected_count": rejected_count,
        "candidate_reject_reason_count": len(reject_reason_counts),
        "candidate_top_reject_reason": max(
            reject_reason_counts.items(), key=lambda item: item[1]
        )[0]
        if reject_reason_counts
        else "",
        "candidate_top_reject_reason_count": max(reject_reason_counts.values())
        if reject_reason_counts
        else 0,
        "candidate_along_degenerate_count": along_degenerate_count,
        "candidate_source_ndt_count": source_counts.get("ndt", 0),
        "candidate_source_scan_submap_icp_count": source_counts.get("scan_submap_icp", 0),
        "candidate_source_scan_submap_local_ndt_count": source_counts.get(
            "scan_submap_local_ndt", 0
        ),
        "candidate_source_global_map_local_ndt_count": source_counts.get(
            "global_map_local_ndt", 0
        ),
        "candidate_score_min": min_or_nan(scores),
        "candidate_score_max": max_or_nan(scores),
        "candidate_nvtl_min": min_or_nan(nvtls),
        "candidate_nvtl_max": max_or_nan(nvtls),
        "candidate_i2r_min_m": min_or_nan(i2r_values),
        "candidate_i2r_max_m": max_or_nan(i2r_values),
        "candidate_yaw_delta_max_deg": math.degrees(max_or_nan(yaw_values)),
    }


def is_submap_candidate(candidate: NdtCandidate) -> bool:
    return str(candidate.source) in {
        "scan_submap_icp",
        "scan_submap_local_ndt",
        "global_map_local_ndt",
    }


def submap_candidate_consistency_is_valid(
    hypothesis: Hypothesis,
    candidate: NdtCandidate,
    *,
    route_prior: RoutePrior | None,
    relative_residual: RelativeResidual | None,
    config: TrackerConfig,
    baseline_residual: RelativeResidual | None = None,
) -> bool:
    """Return whether a synthetic submap candidate has causal support."""

    if not config.enable_submap_candidate_consistency_gate:
        return True
    if not is_submap_candidate(candidate):
        return True
    _forward, lateral, yaw_delta = planar_delta(hypothesis.pose, candidate.pose)
    if abs(lateral) > config.submap_candidate_max_lateral_m:
        return False
    if abs(yaw_delta) > config.submap_candidate_max_yaw_rad:
        return False
    if relative_residual is None or not relative_residual.is_valid:
        return False
    residual_score = relative_residual.xy_m + abs(relative_residual.yaw_rad)
    if residual_score > config.submap_candidate_max_residual_m:
        return False
    min_improvement = max(0.0, config.submap_candidate_min_residual_improvement_m)
    if min_improvement > 0.0:
        if baseline_residual is None or not baseline_residual.is_valid:
            return False
        baseline_score = baseline_residual.xy_m + abs(baseline_residual.yaw_rad)
        if baseline_score - residual_score < min_improvement:
            return False
    if route_prior is not None and route_prior.is_valid:
        predicted_progress = hypothesis.route_progress_m + _forward
        if (
            abs(route_prior.progress_m - predicted_progress)
            > config.submap_candidate_max_progress_innovation_m
        ):
            return False
    return True


def candidate_residual_consistency_is_valid(
    *,
    relative_residual: RelativeResidual | None,
    baseline_residual: RelativeResidual | None,
    config: TrackerConfig,
) -> bool:
    """Gate arbitrary candidates by causal scan-to-submap residual evidence."""

    if not config.enable_candidate_residual_consistency_gate:
        return True
    if relative_residual is None or not relative_residual.is_valid:
        return False
    residual_score = float(relative_residual.xy_m) + abs(float(relative_residual.yaw_rad))
    if residual_score > float(config.candidate_residual_max_m):
        return False
    min_improvement = max(0.0, float(config.candidate_residual_min_improvement_m))
    if min_improvement <= 0.0:
        return True
    if baseline_residual is None or not baseline_residual.is_valid:
        return False
    baseline_score = float(baseline_residual.xy_m) + abs(float(baseline_residual.yaw_rad))
    return baseline_score - residual_score >= min_improvement


def payload_indicates_along_degeneracy(
    payload: dict[str, Any],
    *,
    min_along_variance_m2: float,
    min_along_to_cross_ratio: float,
) -> bool:
    """Detect an along-degenerate NDT candidate set from causal NDT diagnostics."""

    variance_pairs: list[tuple[float, float]] = []
    if bool(payload.get("spread_covariance_ambiguous", False)):
        try:
            variance_pairs.append(
                (
                    float(payload.get("spread_covariance_along_m2", 0.0) or 0.0),
                    float(payload.get("spread_covariance_cross_m2", 0.0) or 0.0),
                )
            )
        except (TypeError, ValueError):
            pass
    try:
        variance_pairs.append(
            (
                float(payload.get("selected_output_covariance_along_m2", 0.0) or 0.0),
                float(payload.get("selected_output_covariance_cross_m2", 0.0) or 0.0),
            )
        )
    except (TypeError, ValueError):
        pass
    for item in payload.get("candidates") or []:
        try:
            variance_pairs.append(
                (
                    float(item.get("localizability_along_variance_m2", 0.0) or 0.0),
                    float(item.get("localizability_cross_variance_m2", 0.0) or 0.0),
                )
            )
        except (AttributeError, TypeError, ValueError):
            continue
    innovations: list[tuple[float, float]] = []
    for item in payload.get("candidates") or []:
        try:
            innovations.append(
                (
                    float(item.get("innovation_along_m", 0.0) or 0.0),
                    float(item.get("innovation_cross_m", 0.0) or 0.0),
                )
            )
        except (AttributeError, TypeError, ValueError):
            continue
    if len(innovations) >= 3:
        along_values = [item[0] for item in innovations]
        cross_values = [item[1] for item in innovations]
        along_span = max(along_values) - min(along_values)
        cross_span = max(cross_values) - min(cross_values)
        variance_pairs.append((0.25 * along_span * along_span, 0.25 * cross_span * cross_span))

    for along_m2, cross_m2 in variance_pairs:
        if along_m2 < min_along_variance_m2:
            continue
        denominator = max(cross_m2, 1.0e-6)
        if along_m2 / denominator >= min_along_to_cross_ratio:
            return True
    return False


def payload_candidate_localizability_summary(payload: dict[str, Any]) -> dict[str, float]:
    """Return max candidate localizability variances for diagnostics."""

    max_along = 0.0
    max_cross = 0.0
    for item in payload.get("candidates") or []:
        try:
            max_along = max(
                max_along,
                float(item.get("localizability_along_variance_m2", 0.0) or 0.0),
            )
            max_cross = max(
                max_cross,
                float(item.get("localizability_cross_variance_m2", 0.0) or 0.0),
            )
        except (AttributeError, TypeError, ValueError):
            continue
    return {
        "candidate_localizability_along_m2_max": max_along,
        "candidate_localizability_cross_m2_max": max_cross,
        "selected_output_covariance_along_m2": float(
            payload.get("selected_output_covariance_along_m2", 0.0) or 0.0
        ),
        "selected_output_covariance_cross_m2": float(
            payload.get("selected_output_covariance_cross_m2", 0.0) or 0.0
        ),
        "spread_covariance_ambiguous": bool(payload.get("spread_covariance_ambiguous", False)),
        "spread_covariance_contender_count": int(
            payload.get("spread_covariance_contender_count", 0) or 0
        ),
        "spread_covariance_along_m2": float(
            payload.get("spread_covariance_along_m2", 0.0) or 0.0
        ),
        "spread_covariance_cross_m2": float(
            payload.get("spread_covariance_cross_m2", 0.0) or 0.0
        ),
    }


class RoutePath:
    """Polyline route prior loaded from a non-GNSS map asset."""

    def __init__(self, points: list[tuple[float, float]]) -> None:
        if len(points) < 2:
            raise ValueError("route path requires at least two points")
        self._points = tuple(points)
        cumulative = [0.0]
        for prev, cur in zip(self._points, self._points[1:]):
            cumulative.append(cumulative[-1] + math.hypot(cur[0] - prev[0], cur[1] - prev[1]))
        self._progress = tuple(cumulative)

    @classmethod
    def from_csv(cls, path: str | Path) -> "RoutePath":
        points: list[tuple[float, float]] = []
        with Path(path).open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    points.append((float(row["x"]), float(row["y"])))
                except (KeyError, TypeError, ValueError):
                    continue
        return cls(points)

    def project(
        self,
        pose: Pose2D,
        predicted_progress_m: float | None = None,
        search_radius_m: float = 30.0,
    ) -> RouteProjection:
        candidates = self.project_candidates(
            pose,
            predicted_progress_m=predicted_progress_m,
            search_radius_m=search_radius_m,
            max_distance_m=math.inf,
            max_candidates=1,
        )
        return candidates[0] if candidates else RouteProjection(0.0, 0.0, 0.0, 0.0, math.inf, False)

    def project_candidates(
        self,
        pose: Pose2D,
        predicted_progress_m: float | None = None,
        search_radius_m: float = 30.0,
        max_distance_m: float = math.inf,
        max_candidates: int = 5,
    ) -> list[RouteProjection]:
        candidates: list[RouteProjection] = []
        for index, (start, end) in enumerate(zip(self._points, self._points[1:])):
            segment_start_progress = self._progress[index]
            segment_end_progress = self._progress[index + 1]
            if predicted_progress_m is not None and search_radius_m > 0.0:
                if segment_end_progress < predicted_progress_m - search_radius_m:
                    continue
                if segment_start_progress > predicted_progress_m + search_radius_m:
                    continue

            sx, sy = start
            ex, ey = end
            vx = ex - sx
            vy = ey - sy
            length2 = vx * vx + vy * vy
            if length2 <= 1e-9:
                continue
            t = max(0.0, min(1.0, ((pose.x - sx) * vx + (pose.y - sy) * vy) / length2))
            px = sx + t * vx
            py = sy + t * vy
            dx = pose.x - px
            dy = pose.y - py
            distance = math.hypot(dx, dy)
            yaw = math.atan2(vy, vx)
            cross = -math.sin(yaw) * dx + math.cos(yaw) * dy
            progress = segment_start_progress + math.sqrt(length2) * t
            projection = RouteProjection(
                progress_m=progress,
                cross_track_m=cross,
                yaw_error_rad=normalize_angle(pose.yaw - yaw),
                route_yaw_rad=yaw,
                distance_m=distance,
            )
            if projection.distance_m <= max_distance_m:
                candidates.append(projection)

        candidates.sort(key=lambda item: item.distance_m)
        return candidates[: max(1, int(max_candidates))]

    def center_at_progress(self, progress_m: float) -> tuple[float, float, float]:
        progress_m = max(self._progress[0], min(self._progress[-1], float(progress_m)))
        for index, (start, end) in enumerate(zip(self._points, self._points[1:])):
            segment_start = self._progress[index]
            segment_end = self._progress[index + 1]
            if progress_m > segment_end and index < len(self._points) - 2:
                continue
            sx, sy = start
            ex, ey = end
            length = max(segment_end - segment_start, 1e-9)
            ratio = max(0.0, min(1.0, (progress_m - segment_start) / length))
            x = sx + (ex - sx) * ratio
            y = sy + (ey - sy) * ratio
            yaw = math.atan2(ey - sy, ex - sx)
            return x, y, yaw
        yaw = 0.0
        if len(self._points) >= 2:
            sx, sy = self._points[-2]
            ex, ey = self._points[-1]
            yaw = math.atan2(ey - sy, ex - sx)
        return self._points[-1][0], self._points[-1][1], yaw


def route_frame_scan_geometry_certificate(
    points: list[tuple[float, ...]],
    pose: Pose2D,
    route_path: RoutePath,
    *,
    route_progress_m: float,
    search_radius_m: float = 120.0,
    min_points: int = 20,
    min_cross_side_points: int = 5,
    min_along_to_cross_ratio: float = 3.0,
    min_along_span_m: float = 20.0,
    max_cross_span_m: float = 8.0,
) -> ScanGeometryCertificate:
    """Diagnose scan geometry in the route frame using only causal pose/route state."""

    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    route_points: list[tuple[float, float]] = []
    for point in points:
        if len(point) < 2:
            continue
        local_x = float(point[0])
        local_y = float(point[1])
        if not (math.isfinite(local_x) and math.isfinite(local_y)):
            continue
        global_x = pose.x + cos_yaw * local_x - sin_yaw * local_y
        global_y = pose.y + sin_yaw * local_x + cos_yaw * local_y
        projection = route_path.project(
            Pose2D(pose.stamp_sec, global_x, global_y, pose.yaw),
            predicted_progress_m=route_progress_m + local_x,
            search_radius_m=search_radius_m,
        )
        if not projection.is_valid:
            continue
        route_points.append((projection.progress_m - route_progress_m, projection.cross_track_m))
    return scan_geometry_certificate(
        route_points,
        min_points=min_points,
        min_cross_side_points=min_cross_side_points,
        min_along_to_cross_ratio=min_along_to_cross_ratio,
        min_along_span_m=min_along_span_m,
        max_cross_span_m=max_cross_span_m,
    )


def route_offset_pose(
    route_path: RoutePath,
    pose: Pose2D,
    *,
    target_cross_m: float,
    gain: float,
    yaw_gain: float,
    gate_m: float,
    predicted_progress_m: float | None,
    search_radius_m: float,
) -> Pose2D | None:
    projection = route_path.project(
        pose,
        predicted_progress_m=predicted_progress_m,
        search_radius_m=search_radius_m,
    )
    if not projection.is_valid:
        return None
    center_x, center_y, center_yaw = route_path.center_at_progress(projection.progress_m)
    target_x = center_x - math.sin(center_yaw) * target_cross_m
    target_y = center_y + math.cos(center_yaw) * target_cross_m
    distance = math.hypot(target_x - pose.x, target_y - pose.y)
    if distance > max(0.0, gate_m):
        return None
    route_gain = max(0.0, min(1.0, gain))
    route_yaw_gain = max(0.0, min(1.0, yaw_gain))
    return Pose2D(
        stamp_sec=pose.stamp_sec,
        x=pose.x + route_gain * (target_x - pose.x),
        y=pose.y + route_gain * (target_y - pose.y),
        yaw=normalize_angle(
            pose.yaw + route_yaw_gain * normalize_angle(center_yaw - pose.yaw)
        ),
    )


def route_remap_candidate_along_to_prediction(
    route_path: RoutePath,
    predicted_pose: Pose2D,
    ndt_pose: Pose2D,
    *,
    keep_predicted_yaw: bool,
    search_radius_m: float,
) -> Pose2D | None:
    """Keep predicted route progress but use NDT's route-frame cross evidence."""

    predicted_projection = route_path.project(
        predicted_pose,
        predicted_progress_m=None,
        search_radius_m=search_radius_m,
    )
    if not predicted_projection.is_valid:
        return None
    ndt_projection = route_path.project(
        ndt_pose,
        predicted_progress_m=predicted_projection.progress_m,
        search_radius_m=search_radius_m,
    )
    if not ndt_projection.is_valid:
        return None
    center_x, center_y, center_yaw = route_path.center_at_progress(
        predicted_projection.progress_m
    )
    return Pose2D(
        stamp_sec=ndt_pose.stamp_sec,
        x=center_x - math.sin(center_yaw) * ndt_projection.cross_track_m,
        y=center_y + math.cos(center_yaw) * ndt_projection.cross_track_m,
        yaw=predicted_pose.yaw if keep_predicted_yaw else ndt_pose.yaw,
    )


def route_filter_candidates(
    candidates: list[NdtCandidate],
    route_path: RoutePath | None,
    *,
    cross_gate_m: float = math.inf,
    cross_target_m: float = 0.0,
    yaw_gate_rad: float = math.pi,
    predicted_progress_m: float | None = None,
    search_radius_m: float = 30.0,
) -> list[NdtCandidate]:
    """Filter NDT candidates with a causal route-progress neighborhood.

    Hairpins make global nearest-route projection unsafe: a geometrically close
    candidate can belong to the wrong branch.  The caller should pass the
    current tracker progress estimate so filtering is local along the route.
    """

    if route_path is None:
        return candidates

    filtered: list[NdtCandidate] = []
    for candidate in candidates:
        projection = route_path.project(
            candidate.pose,
            predicted_progress_m=predicted_progress_m,
            search_radius_m=search_radius_m,
        )
        if not projection.is_valid:
            filtered.append(candidate)
            continue
        if abs(projection.cross_track_m - cross_target_m) > cross_gate_m:
            continue
        if abs(projection.yaw_error_rad) > yaw_gate_rad:
            continue
        filtered.append(candidate)
    return filtered


class FixedLagMultiHypothesisTracker:
    """Causal fixed-lag scorer for pure-LiDAR localization hypotheses."""

    def __init__(
        self,
        initial_pose: Pose2D,
        config: TrackerConfig | None = None,
        initial_route_progress_m: float = 0.0,
        initial_score: float = 0.0,
        route_path: RoutePath | None = None,
    ) -> None:
        self.config = config or TrackerConfig()
        self._route_path = route_path
        self._next_id = 1
        self._selected_hypothesis_id = 0
        self._hypotheses = [
            Hypothesis(
                id=0,
                parent_id=None,
                pose=initial_pose,
                route_progress_m=initial_route_progress_m,
                score=initial_score,
                start_stamp_sec=initial_pose.stamp_sec,
                history=(initial_pose,),
            )
        ]

    @property
    def hypotheses(self) -> tuple[Hypothesis, ...]:
        return tuple(self._hypotheses)

    def best(self) -> Hypothesis:
        ranked = sorted(self._hypotheses, key=lambda item: item.score, reverse=True)
        top = ranked[0]
        selected = next(
            (item for item in ranked if item.id == self._selected_hypothesis_id),
            None,
        )
        if selected is None:
            self._selected_hypothesis_id = top.id
            return top
        margin = max(0.0, self.config.best_switch_score_margin)
        if top.id != selected.id and top.score > selected.score + margin:
            self._selected_hypothesis_id = top.id
            return top
        return selected

    def propagate(self, motion: MotionDelta) -> None:
        propagated: list[Hypothesis] = []
        for hypothesis in self._hypotheses:
            pose = self._propagate_pose(hypothesis.pose, motion)
            history = (*hypothesis.history, pose)[-self.config.fixed_lag_frames :]
            propagated.append(
                replace(
                    hypothesis,
                    parent_id=hypothesis.id,
                    pose=pose,
                    route_progress_m=hypothesis.route_progress_m + motion.forward_m,
                    covariance_scale=hypothesis.covariance_scale
                    + self.config.propagation_covariance_growth * motion.covariance_scale,
                    score=hypothesis.score
                    - self.config.missed_update_penalty * max(motion.dt_sec, 0.0),
                    age_frames=hypothesis.age_frames + 1,
                    missed_updates=hypothesis.missed_updates + 1,
                    history=history,
                )
            )
        self._hypotheses = self._prune(propagated)

    def update(
        self,
        candidates: list[NdtCandidate],
        route_priors: dict[int | tuple[int, int], RoutePrior] | None = None,
        relative_residuals: dict[int | tuple[int, int], RelativeResidual] | None = None,
        candidate_penalties: dict[int | tuple[int, int], WeakPriorPenalty] | None = None,
        along_degenerate: bool = False,
    ) -> None:
        route_priors = route_priors or {}
        relative_residuals = relative_residuals or {}
        candidate_penalties = candidate_penalties or {}
        updated: list[Hypothesis] = []
        valid_candidates = [candidate for candidate in candidates if self._candidate_is_usable(candidate)]

        if along_degenerate and self.config.degenerate_skip_ndt_candidates:
            valid_candidates = []

        if not valid_candidates:
            self._hypotheses = self._prune(
                [
                    replace(
                        hypothesis,
                        parent_id=hypothesis.id,
                        score=hypothesis.score - self.config.missed_update_penalty,
                        missed_updates=hypothesis.missed_updates + 1,
                    )
                    for hypothesis in self._hypotheses
                ]
            )
            return

        for hypothesis in self._hypotheses:
            # Keep the propagated/coasting branch even when NDT candidates are
            # available.  W10 failures are frequently caused by a single
            # plausible-looking wrong basin; a fixed-lag tracker must be able
            # to delay commitment instead of replacing every live hypothesis
            # with that frame's NDT result.
            updated.append(
                replace(
                    hypothesis,
                    score=hypothesis.score - self.config.missed_update_penalty,
                    missed_updates=hypothesis.missed_updates + 1,
                )
            )
            for candidate_index, candidate in enumerate(valid_candidates):
                if along_degenerate and not self._degenerate_candidate_is_consistent(
                    hypothesis, candidate
                ):
                    continue
                route_prior = route_priors.get((hypothesis.id, candidate_index))
                if route_prior is None:
                    route_prior = route_priors.get(hypothesis.id)
                residual = relative_residuals.get((hypothesis.id, candidate_index))
                if residual is None:
                    residual = relative_residuals.get(hypothesis.id)
                baseline_residual = relative_residuals.get((hypothesis.id, -1))
                weak_penalty = candidate_penalties.get((hypothesis.id, candidate_index))
                if weak_penalty is None:
                    weak_penalty = candidate_penalties.get(candidate_index)
                if not submap_candidate_consistency_is_valid(
                    hypothesis,
                    candidate,
                    route_prior=route_prior,
                    relative_residual=residual,
                    config=self.config,
                    baseline_residual=baseline_residual,
                ):
                    continue
                if not candidate_residual_consistency_is_valid(
                    relative_residual=residual,
                    baseline_residual=baseline_residual,
                    config=self.config,
                ):
                    continue
                updated.append(
                    self._apply_candidate(
                        hypothesis,
                        candidate,
                        route_prior,
                        residual,
                        weak_penalty,
                        along_degenerate=along_degenerate,
                    )
                )

        self._hypotheses = self._prune(updated)

    def _apply_candidate(
        self,
        hypothesis: Hypothesis,
        candidate: NdtCandidate,
        route_prior: RoutePrior | None,
        relative_residual: RelativeResidual | None,
        weak_penalty: WeakPriorPenalty | None,
        *,
        along_degenerate: bool = False,
    ) -> Hypothesis:
        forward, lateral, yaw_delta = planar_delta(hypothesis.pose, candidate.pose)
        candidate_pose = candidate.pose
        candidate_along_degenerate = along_degenerate or candidate_indicates_along_degeneracy(
            candidate, self.config
        )
        if candidate_along_degenerate and self.config.degenerate_keep_predicted_yaw_only:
            candidate_pose = Pose2D(
                stamp_sec=candidate.pose.stamp_sec,
                x=candidate.pose.x,
                y=candidate.pose.y,
                yaw=hypothesis.pose.yaw,
            )
            forward, lateral, yaw_delta = planar_delta(hypothesis.pose, candidate_pose)
        elif candidate_along_degenerate and self.config.enable_degenerate_along_remap:
            if self.config.degenerate_remap_use_route_frame and self._route_path is not None:
                candidate_pose = route_remap_candidate_along_to_prediction(
                    self._route_path,
                    hypothesis.pose,
                    candidate.pose,
                    keep_predicted_yaw=self.config.degenerate_remap_keep_predicted_yaw,
                    search_radius_m=30.0,
                ) or candidate.pose
            else:
                candidate_pose = self._remap_candidate_along_to_prediction(
                    hypothesis.pose,
                    candidate.pose,
                    keep_predicted_yaw=self.config.degenerate_remap_keep_predicted_yaw,
                )
            forward, lateral, yaw_delta = planar_delta(hypothesis.pose, candidate_pose)
        predicted_progress = hypothesis.route_progress_m + forward
        if route_prior and route_prior.is_valid:
            update_gain = max(0.0, min(1.0, self.config.route_progress_update_gain))
            update_age_sec = candidate.pose.stamp_sec - hypothesis.start_stamp_sec
            if update_age_sec > self.config.route_progress_update_max_age_sec:
                update_gain = 0.0
            progress = predicted_progress + update_gain * (route_prior.progress_m - predicted_progress)
        else:
            progress = predicted_progress
        score = hypothesis.score
        score += self.config.ndt_score_weight * candidate.score
        score -= self.config.initial_to_result_penalty_weight * candidate.initial_to_result_m
        score -= self.config.yaw_jump_weight * abs(yaw_delta)
        score -= self.config.lateral_jump_weight * abs(lateral)

        if progress < hypothesis.route_progress_m - self.config.max_route_progress_regression_m:
            score -= self.config.non_monotonic_progress_penalty

        if route_prior and route_prior.is_valid:
            score -= self.config.route_cross_weight * abs(route_prior.cross_track_m)
            score -= self.config.route_yaw_weight * abs(route_prior.yaw_error_rad)
            score -= self.config.route_progress_weight * abs(progress - predicted_progress)

        if relative_residual and relative_residual.is_valid:
            residual_penalty = relative_residual.xy_m + abs(relative_residual.yaw_rad)
            score -= self.config.relative_residual_weight * residual_penalty
            score += max(0.0, min(1.0, relative_residual.quality))

        if weak_penalty is not None and weak_penalty.is_valid:
            score -= self.config.gnss_weak_prior_weight * max(0.0, weak_penalty.penalty)

        history = (*hypothesis.history, candidate_pose)[-self.config.fixed_lag_frames :]
        return Hypothesis(
            id=self._allocate_id(),
            parent_id=hypothesis.id,
            pose=candidate_pose,
            route_progress_m=progress,
            covariance_scale=max(0.5, hypothesis.covariance_scale * 0.7),
            score=score,
            age_frames=hypothesis.age_frames + 1,
            missed_updates=0,
            start_stamp_sec=hypothesis.start_stamp_sec,
            history=history,
        )

    def _candidate_is_usable(self, candidate: NdtCandidate) -> bool:
        return candidate_is_usable(candidate, self.config)

    def _degenerate_candidate_is_consistent(
        self, hypothesis: Hypothesis, candidate: NdtCandidate
    ) -> bool:
        _, lateral, yaw_delta = planar_delta(hypothesis.pose, candidate.pose)
        return (
            abs(lateral) <= self.config.degenerate_max_lateral_m
            and abs(yaw_delta) <= self.config.degenerate_max_yaw_rad
        )

    @staticmethod
    def _remap_candidate_along_to_prediction(
        predicted_pose: Pose2D,
        ndt_pose: Pose2D,
        *,
        keep_predicted_yaw: bool = False,
    ) -> Pose2D:
        _, lateral, _ = planar_delta(predicted_pose, ndt_pose)
        cy = math.cos(predicted_pose.yaw)
        sy = math.sin(predicted_pose.yaw)
        return Pose2D(
            stamp_sec=ndt_pose.stamp_sec,
            x=predicted_pose.x - sy * lateral,
            y=predicted_pose.y + cy * lateral,
            yaw=predicted_pose.yaw if keep_predicted_yaw else ndt_pose.yaw,
        )

    def _prune(self, hypotheses: list[Hypothesis]) -> list[Hypothesis]:
        ranked = sorted(hypotheses, key=lambda item: item.score, reverse=True)
        return ranked[: max(1, self.config.max_hypotheses)]

    def apply_pose_correction(self, corrected: dict[int, Pose2D]) -> None:
        if not corrected:
            return
        updated = []
        for hypothesis in self._hypotheses:
            pose = corrected.get(hypothesis.id)
            if pose is None:
                updated.append(hypothesis)
                continue
            history = (*hypothesis.history[:-1], pose) if hypothesis.history else (pose,)
            updated.append(replace(hypothesis, pose=pose, history=history))
        self._hypotheses = updated

    def add_startup_hypothesis(
        self,
        pose: Pose2D,
        *,
        route_progress_m: float,
        score: float = 0.0,
        min_separation_m: float = 1.0,
        min_route_progress_separation_m: float = 0.0,
    ) -> None:
        for hypothesis in self._hypotheses:
            pose_close = math.hypot(hypothesis.pose.x - pose.x, hypothesis.pose.y - pose.y) < min_separation_m
            progress_close = (
                abs(hypothesis.route_progress_m - route_progress_m)
                < max(0.0, float(min_route_progress_separation_m))
            )
            if pose_close and progress_close:
                return
        self._hypotheses = self._prune(
            [
                *self._hypotheses,
                Hypothesis(
                    id=self._allocate_id(),
                    parent_id=None,
                    pose=pose,
                    route_progress_m=route_progress_m,
                    score=score,
                    start_stamp_sec=pose.stamp_sec,
                    history=(pose,),
                ),
            ]
        )

    def _allocate_id(self) -> int:
        value = self._next_id
        self._next_id += 1
        return value

    @staticmethod
    def _propagate_pose(pose: Pose2D, motion: MotionDelta) -> Pose2D:
        cy = math.cos(pose.yaw)
        sy = math.sin(pose.yaw)
        x = pose.x + cy * motion.forward_m - sy * motion.lateral_m
        y = pose.y + sy * motion.forward_m + cy * motion.lateral_m
        return Pose2D(
            stamp_sec=pose.stamp_sec + motion.dt_sec,
            x=x,
            y=y,
            yaw=normalize_angle(pose.yaw + motion.yaw_rad),
        )


def candidates_from_runtime_multistart(payload: dict[str, Any]) -> list[NdtCandidate]:
    stamp_sec = float(payload.get("stamp_sec", 0.0) or 0.0)
    candidates: list[NdtCandidate] = []
    for item in payload.get("candidates") or []:
        try:
            yaw_rad = math.radians(float(item.get("result_yaw_deg", 0.0) or 0.0))
            score_value = item.get("total_score")
            if score_value is None:
                score_value = item.get("nearest_voxel_transformation_likelihood", 0.0)
            candidates.append(
                NdtCandidate(
                    pose=Pose2D(
                        stamp_sec=stamp_sec,
                        x=float(item["result_x"]),
                        y=float(item["result_y"]),
                        yaw=yaw_rad,
                    ),
                    score=float(score_value),
                    converged=bool(item.get("converged", False)),
                    initial_to_result_m=float(item.get("initial_to_result_distance_m", 0.0) or 0.0),
                    initial_to_result_yaw_rad=math.radians(
                        float(item.get("innovation_yaw_deg", 0.0) or 0.0)
                    ),
                    rejection_reason=str(item.get("reject_reason", "") or ""),
                    localizability_along_variance_m2=float(
                        item.get("localizability_along_variance_m2", 0.0) or 0.0
                    ),
                    localizability_cross_variance_m2=float(
                        item.get("localizability_cross_variance_m2", 0.0) or 0.0
                    ),
                    nearest_voxel_transformation_likelihood=float(
                        item.get("nearest_voxel_transformation_likelihood", math.nan)
                        or math.nan
                    ),
                )
            )
        except (KeyError, TypeError, ValueError):
            continue
    return candidates


def main() -> None:
    import json

    import rclpy
    from autoware_vehicle_msgs.msg import VelocityReport
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String

    class PureLidarFixedLagTrackerNode(Node):
        def __init__(self) -> None:
            super().__init__("pure_lidar_fixed_lag_tracker")
            self._frame_id = self.declare_parameter("frame_id", "map").value
            self._input_topic = self.declare_parameter(
                "runtime_multistart_topic",
                "/localization/ndt/runtime_multistart/decision",
            ).value
            self._output_pose_topic = self.declare_parameter(
                "output_pose_topic",
                "/localization/pure_lidar/pose_with_covariance",
            ).value
            self._output_initial_topic = self.declare_parameter(
                "output_initial_pose_topic",
                "/localization/pure_lidar/initial_pose",
            ).value
            self._diagnostics_topic = self.declare_parameter(
                "diagnostics_topic",
                "/localization/pure_lidar/tracker_diagnostics",
            ).value
            self._fallback_initial_topic = self.declare_parameter(
                "fallback_initial_pose_topic",
                "/localization/ndt_initial_pose",
            ).value
            self._velocity_topic = self.declare_parameter(
                "velocity_topic",
                "/vehicle/status/velocity_status",
            ).value
            self._velocity_scale = float(
                self.declare_parameter("twist_linear_x_scale", 1.0).value
            )
            self._velocity_bias_mps = float(
                self.declare_parameter("twist_linear_x_bias_mps", 0.0).value
            )
            self._motion_max_dt_sec = float(
                self.declare_parameter("motion_max_dt_sec", 1.0).value
            )
            self._enable_twist_bias_learning = bool(
                self.declare_parameter("enable_twist_bias_learning", False).value
            )
            self._twist_bias_learning_alpha = float(
                self.declare_parameter("twist_bias_learning_alpha", 0.05).value
            )
            self._twist_bias_learning_max_step_mps = abs(
                float(self.declare_parameter("twist_bias_learning_max_step_mps", 0.02).value)
            )
            self._twist_bias_learning_max_abs_mps = abs(
                float(self.declare_parameter("twist_bias_learning_max_abs_mps", 0.15).value)
            )
            self._twist_bias_learning_min_dt_sec = float(
                self.declare_parameter("twist_bias_learning_min_dt_sec", 0.3).value
            )
            self._twist_bias_learning_max_dt_sec = float(
                self.declare_parameter("twist_bias_learning_max_dt_sec", 3.0).value
            )
            self._twist_bias_learning_max_lateral_m = abs(
                float(self.declare_parameter("twist_bias_learning_max_lateral_m", 0.5).value)
            )
            self._twist_bias_learning_max_yaw_deg = abs(
                float(self.declare_parameter("twist_bias_learning_max_yaw_deg", 3.0).value)
            )
            self._enable_degenerate_velocity_scale = bool(
                self.declare_parameter("enable_degenerate_velocity_scale", False).value
            )
            self._degenerate_velocity_scale = float(
                self.declare_parameter("degenerate_velocity_scale", 1.0).value
            )
            self._pointcloud_topic = self.declare_parameter(
                "pointcloud_topic",
                "/sensing/lidar/concatenated/pointcloud",
            ).value
            self._enable_gnss_weak_prior = bool(
                self.declare_parameter("enable_gnss_weak_prior", False).value
            )
            self._gnss_weak_prior_topic = str(
                self.declare_parameter("gnss_weak_prior_topic", "/fixposition/odometry_enu").value
            )
            self._gnss_weak_prior_msg_type = str(
                self.declare_parameter("gnss_weak_prior_msg_type", "odometry").value
            )
            self._gnss_weak_prior_sigma_m = max(
                0.1, float(self.declare_parameter("gnss_weak_prior_sigma_m", 5.0).value)
            )
            self._gnss_weak_prior_max_age_sec = max(
                0.0, float(self.declare_parameter("gnss_weak_prior_max_age_sec", 0.5).value)
            )
            self._gnss_weak_prior_max_penalty = max(
                0.0, float(self.declare_parameter("gnss_weak_prior_max_penalty", 8.0).value)
            )
            self._enable_gnss_weak_prior_initial_hint = bool(
                self.declare_parameter("enable_gnss_weak_prior_initial_hint", False).value
            )
            self._gnss_weak_prior_initial_hint_gate_m = max(
                0.0, float(self.declare_parameter("gnss_weak_prior_initial_hint_gate_m", 8.0).value)
            )
            self._gnss_weak_prior_initial_hint_max_step_m = max(
                0.0, float(self.declare_parameter("gnss_weak_prior_initial_hint_max_step_m", 4.0).value)
            )
            self._gnss_weak_prior_initial_hint_yaw_gain = max(
                0.0, min(1.0, float(self.declare_parameter("gnss_weak_prior_initial_hint_yaw_gain", 0.0).value))
            )
            self._enable_scan_submap_residual = bool(
                self.declare_parameter("enable_scan_submap_residual", False).value
            )
            self._scan_submap_add_only_with_usable_ndt = bool(
                self.declare_parameter("scan_submap_add_only_with_usable_ndt", False).value
            )
            self._enable_scan_submap_anchor = bool(
                self.declare_parameter("enable_scan_submap_anchor", False).value
            )
            self._scan_submap_anchor_min_usable_streak = max(
                1, int(self.declare_parameter("scan_submap_anchor_min_usable_streak", 3).value)
            )
            self._scan_submap_anchor_max_age_sec = float(
                self.declare_parameter("scan_submap_anchor_max_age_sec", 8.0).value
            )
            self._scan_submap_max_age_sec = float(
                self.declare_parameter("scan_submap_max_age_sec", 0.35).value
            )
            self._scan_submap_sample_stride = max(
                1, int(self.declare_parameter("scan_submap_sample_stride", 20).value)
            )
            self._scan_submap_max_points = max(
                1, int(self.declare_parameter("scan_submap_max_points", 256).value)
            )
            self._scan_submap_min_range_m = float(
                self.declare_parameter("scan_submap_min_range_m", 3.0).value
            )
            self._scan_submap_max_range_m = float(
                self.declare_parameter("scan_submap_max_range_m", 80.0).value
            )
            self._enable_scan_point_count_degeneracy = bool(
                self.declare_parameter("enable_scan_point_count_degeneracy", False).value
            )
            self._scan_point_count_degeneracy_min_sampled_points = max(
                1,
                int(
                    self.declare_parameter(
                        "scan_point_count_degeneracy_min_sampled_points",
                        350,
                    ).value
                ),
            )
            self._enable_scan_geometry_degeneracy = bool(
                self.declare_parameter("enable_scan_geometry_degeneracy", False).value
            )
            self._scan_geometry_use_as_degenerate = bool(
                self.declare_parameter("scan_geometry_use_as_degenerate", False).value
            )
            self._scan_geometry_min_points = max(
                1, int(self.declare_parameter("scan_geometry_min_points", 20).value)
            )
            self._scan_geometry_min_cross_side_points = max(
                0, int(self.declare_parameter("scan_geometry_min_cross_side_points", 5).value)
            )
            self._scan_geometry_min_along_to_cross_ratio = float(
                self.declare_parameter("scan_geometry_min_along_to_cross_ratio", 3.0).value
            )
            self._scan_geometry_min_along_span_m = float(
                self.declare_parameter("scan_geometry_min_along_span_m", 20.0).value
            )
            self._scan_geometry_max_cross_span_m = float(
                self.declare_parameter("scan_geometry_max_cross_span_m", 8.0).value
            )
            self._scan_geometry_route_search_radius_m = float(
                self.declare_parameter("scan_geometry_route_search_radius_m", 120.0).value
            )
            self._enable_lro_forward_correction = bool(
                self.declare_parameter("enable_lro_forward_correction", False).value
            )
            self._lro_forward_correction_gain = float(
                self.declare_parameter("lro_forward_correction_gain", 0.5).value
            )
            self._lro_forward_correction_max_m = float(
                self.declare_parameter("lro_forward_correction_max_m", 0.5).value
            )
            self._lro_max_match_distance_m = float(
                self.declare_parameter("lro_max_match_distance_m", 2.5).value
            )
            self._lro_min_quality = float(
                self.declare_parameter("lro_min_quality", 0.35).value
            )
            self._lro_max_residual_m = float(
                self.declare_parameter("lro_max_residual_m", 1.0).value
            )
            self._lro_reject_degenerate = bool(
                self.declare_parameter("lro_reject_degenerate", True).value
            )
            self._scan_submap_min_quality = float(
                self.declare_parameter("scan_submap_min_quality", 0.0).value
            )
            self._scan_submap_residual_mode = str(
                self.declare_parameter("scan_submap_residual_mode", "nearest").value
            ).strip().lower()
            self._scan_submap_profile_lateral_bin_m = float(
                self.declare_parameter("scan_submap_profile_lateral_bin_m", 0.8).value
            )
            self._scan_submap_profile_max_cells = max(
                1, int(self.declare_parameter("scan_submap_profile_max_cells", 4000).value)
            )
            self._scan_submap_icp_max_map_points = max(
                1, int(self.declare_parameter("scan_submap_icp_max_map_points", 4000).value)
            )
            self._scan_submap_icp_max_match_distance_m = float(
                self.declare_parameter("scan_submap_icp_max_match_distance_m", 3.0).value
            )
            self._scan_submap_icp_correction_penalty_weight = float(
                self.declare_parameter("scan_submap_icp_correction_penalty_weight", 0.5).value
            )
            self._enable_scan_submap_icp_candidates = bool(
                self.declare_parameter("enable_scan_submap_icp_candidates", False).value
            )
            self._scan_submap_icp_candidates_only_when_ndt_empty = bool(
                self.declare_parameter("scan_submap_icp_candidates_only_when_ndt_empty", True).value
            )
            self._scan_submap_icp_candidate_max_correction_m = float(
                self.declare_parameter("scan_submap_icp_candidate_max_correction_m", 1.0).value
            )
            self._scan_submap_icp_candidate_max_yaw_deg = float(
                self.declare_parameter("scan_submap_icp_candidate_max_yaw_deg", 2.0).value
            )
            self._scan_submap_icp_candidate_min_quality = float(
                self.declare_parameter("scan_submap_icp_candidate_min_quality", 0.35).value
            )
            self._scan_submap_icp_candidate_score = float(
                self.declare_parameter("scan_submap_icp_candidate_score", 1.2).value
            )
            self._scan_submap_icp_candidate_match_in_3d = bool(
                self.declare_parameter("scan_submap_icp_candidate_match_in_3d", False).value
            )
            self._enable_scan_submap_local_ndt_candidates = bool(
                self.declare_parameter("enable_scan_submap_local_ndt_candidates", False).value
            )
            self._scan_submap_local_ndt_candidates_only_when_ndt_empty = bool(
                self.declare_parameter(
                    "scan_submap_local_ndt_candidates_only_when_ndt_empty",
                    True,
                ).value
            )
            self._scan_submap_local_ndt_candidate_score = float(
                self.declare_parameter("scan_submap_local_ndt_candidate_score", 1.0).value
            )
            self._scan_submap_local_ndt_candidate_min_quality = float(
                self.declare_parameter("scan_submap_local_ndt_candidate_min_quality", 0.35).value
            )
            self._scan_submap_local_ndt_min_cell_points = max(
                2, int(self.declare_parameter("scan_submap_local_ndt_min_cell_points", 5).value)
            )
            self._scan_submap_local_ndt_max_candidates = max(
                1, int(self.declare_parameter("scan_submap_local_ndt_max_candidates", 5).value)
            )
            self._scan_submap_local_ndt_reject_boundary_best = bool(
                self.declare_parameter("scan_submap_local_ndt_reject_boundary_best", True).value
            )
            self._scan_submap_local_ndt_min_second_best_score_margin = float(
                self.declare_parameter(
                    "scan_submap_local_ndt_min_second_best_score_margin",
                    0.10,
                ).value
            )
            self._scan_submap_local_ndt_profile_score_weight = float(
                self.declare_parameter("scan_submap_local_ndt_profile_score_weight", 0.0).value
            )
            self._scan_submap_local_ndt_forward_offsets = self._parse_float_tuple(
                str(
                    self.declare_parameter(
                        "scan_submap_local_ndt_forward_offsets_m",
                        "-1.0,-0.5,0.0,0.5,1.0",
                    ).value
                )
            )
            self._scan_submap_local_ndt_lateral_offsets = self._parse_float_tuple(
                str(
                    self.declare_parameter(
                        "scan_submap_local_ndt_lateral_offsets_m",
                        "-0.5,0.0,0.5",
                    ).value
                )
            )
            self._scan_submap_local_ndt_yaw_offsets = tuple(
                math.radians(value)
                for value in self._parse_float_tuple(
                    str(
                        self.declare_parameter(
                            "scan_submap_local_ndt_yaw_offsets_deg",
                            "-1.0,0.0,1.0",
                        ).value
                    )
                )
            )
            self._enable_global_map_local_ndt_candidates = bool(
                self.declare_parameter("enable_global_map_local_ndt_candidates", False).value
            )
            self._global_map_local_ndt_candidates_only_when_ndt_empty = bool(
                self.declare_parameter(
                    "global_map_local_ndt_candidates_only_when_ndt_empty",
                    True,
                ).value
            )
            self._global_map_pcd_dir = str(
                self.declare_parameter("global_map_pcd_dir", "").value or ""
            )
            self._global_map_pcd_stride = max(
                1, int(self.declare_parameter("global_map_pcd_stride", 20).value)
            )
            self._global_map_max_points = max(
                1, int(self.declare_parameter("global_map_max_points", 300000).value)
            )
            self._global_map_voxel_size_m = float(
                self.declare_parameter("global_map_voxel_size_m", 0.8).value
            )
            self._global_map_max_cells = max(
                1, int(self.declare_parameter("global_map_max_cells", 300000).value)
            )
            self._global_map_local_ndt_use_route_seed = bool(
                self.declare_parameter("global_map_local_ndt_use_route_seed", False).value
            )
            self._global_map_local_ndt_route_seed_yaw_gain = float(
                self.declare_parameter("global_map_local_ndt_route_seed_yaw_gain", 0.0).value
            )
            self._global_map_local_ndt_use_gnss_weak_prior_seed = bool(
                self.declare_parameter(
                    "global_map_local_ndt_use_gnss_weak_prior_seed",
                    False,
                ).value
            )
            self._global_map_local_ndt_gnss_seed_max_age_sec = max(
                0.0,
                float(
                    self.declare_parameter(
                        "global_map_local_ndt_gnss_seed_max_age_sec",
                        0.5,
                    ).value
                ),
            )
            self._global_map_local_ndt_gnss_seed_xy_gate_m = max(
                0.0,
                float(
                    self.declare_parameter(
                        "global_map_local_ndt_gnss_seed_xy_gate_m",
                        15.0,
                    ).value
                ),
            )
            self._enable_scan_submap_correction = bool(
                self.declare_parameter("enable_scan_submap_correction", False).value
            )
            self._scan_submap_correction_min_quality = float(
                self.declare_parameter("scan_submap_correction_min_quality", 0.0).value
            )
            self._scan_submap_correction_min_improvement_m = float(
                self.declare_parameter("scan_submap_correction_min_improvement_m", 0.05).value
            )
            self._scan_submap_correction_profile_score_weight = float(
                self.declare_parameter("scan_submap_correction_profile_score_weight", 0.0).value
            )
            self._enable_scan_submap_profile_along_correction = bool(
                self.declare_parameter("enable_scan_submap_profile_along_correction", False).value
            )
            self._scan_submap_profile_along_min_margin_m = float(
                self.declare_parameter("scan_submap_profile_along_min_margin_m", 0.10).value
            )
            self._scan_submap_correction_forward_offsets = self._parse_float_tuple(
                str(
                    self.declare_parameter(
                        "scan_submap_correction_forward_offsets_m",
                        "-1.0,-0.5,0.0,0.5,1.0",
                    ).value
                )
            )
            self._scan_submap_correction_lateral_offsets = self._parse_float_tuple(
                str(
                    self.declare_parameter(
                        "scan_submap_correction_lateral_offsets_m",
                        "-1.0,-0.5,0.0,0.5,1.0",
                    ).value
                )
            )
            self._scan_submap_correction_yaw_offsets = tuple(
                math.radians(value)
                for value in self._parse_float_tuple(
                    str(
                        self.declare_parameter(
                            "scan_submap_correction_yaw_offsets_deg",
                            "-2.0,0.0,2.0",
                        ).value
                    )
                )
            )
            route_samples_csv = str(self.declare_parameter("route_samples_csv", "").value or "")
            self._route_search_radius_m = float(
                self.declare_parameter("route_search_radius_m", 35.0).value
            )
            self._fallback_route_cross_gate_m = float(
                self.declare_parameter("fallback_route_cross_gate_m", 8.0).value
            )
            self._enable_startup_route_branch_hypotheses = bool(
                self.declare_parameter("enable_startup_route_branch_hypotheses", False).value
            )
            self._startup_route_branch_max_distance_m = float(
                self.declare_parameter("startup_route_branch_max_distance_m", 3.0).value
            )
            self._startup_route_branch_max_candidates = max(
                1, int(self.declare_parameter("startup_route_branch_max_candidates", 4).value)
            )
            self._startup_route_branch_min_progress_separation_m = float(
                self.declare_parameter("startup_route_branch_min_progress_separation_m", 30.0).value
            )
            self._route_cross_correction_gain = float(
                self.declare_parameter("route_cross_correction_gain", 0.0).value
            )
            self._route_progress_correction_gain = float(
                self.declare_parameter("route_progress_correction_gain", 0.0).value
            )
            self._route_yaw_correction_gain = float(
                self.declare_parameter("route_yaw_correction_gain", 0.0).value
            )
            self._route_yaw_correction_gate_rad = math.radians(
                float(self.declare_parameter("route_yaw_correction_gate_deg", 8.0).value)
            )
            self._route_cross_correction_gate_m = float(
                self.declare_parameter("route_cross_correction_gate_m", 6.0).value
            )
            route_cross_correction_target_param = float(
                self.declare_parameter("route_cross_correction_target_m", math.nan).value
            )
            self._route_cross_correction_target_m: float | None = (
                route_cross_correction_target_param
                if math.isfinite(route_cross_correction_target_param)
                else None
            )
            self._learn_correction_target_from_startup = bool(
                self.declare_parameter("learn_correction_target_from_startup", False).value
            )
            self._learned_correction_target_abs_limit_m = float(
                self.declare_parameter(
                    "learned_correction_target_abs_limit_m",
                    math.inf,
                ).value
            )
            self._learned_correction_target_count = 0
            self._learn_correction_target_from_ndt = bool(
                self.declare_parameter("learn_correction_target_from_ndt", False).value
            )
            self._route_cross_target_learner = RouteCrossTargetLearner(
                alpha=float(self.declare_parameter("route_cross_target_learning_alpha", 0.1).value),
                gate_m=float(self.declare_parameter("route_cross_target_learning_gate_m", 2.0).value),
                abs_limit_m=self._learned_correction_target_abs_limit_m,
            )
            route_cross_target_param = float(
                self.declare_parameter("route_cross_target_m", 0.0).value
            )
            self._route_cross_target_m: float | None = (
                route_cross_target_param if math.isfinite(route_cross_target_param) else None
            )
            self._candidate_route_cross_gate_m = float(
                self.declare_parameter("candidate_route_cross_gate_m", math.inf).value
            )
            self._candidate_route_yaw_gate_rad = math.radians(
                float(self.declare_parameter("candidate_route_yaw_gate_deg", 180.0).value)
            )
            self._candidate_route_search_radius_m = float(
                self.declare_parameter("candidate_route_search_radius_m", 30.0).value
            )
            self._forward_fallback_initial = bool(
                self.declare_parameter("forward_fallback_initial_pose", True).value
            )
            self._forward_fallback_after_tracker_init = bool(
                self.declare_parameter("forward_fallback_after_tracker_init", False).value
            )
            self._publish_pose_before_first_candidate = bool(
                self.declare_parameter("publish_pose_before_first_candidate", True).value
            )
            self._initialize_from_first_usable_candidate = bool(
                self.declare_parameter("initialize_from_first_usable_candidate", False).value
            )
            self._refresh_from_fallback_until_first_candidate = bool(
                self.declare_parameter("refresh_from_fallback_until_first_candidate", False).value
            )
            self._final_pose_use_internal_stamp = bool(
                self.declare_parameter("final_pose_use_internal_stamp", False).value
            )
            self._final_pose_warmup_sec = float(
                self.declare_parameter("final_pose_warmup_sec", 0.0).value
            )
            self._final_pose_smoothing_alpha = float(
                self.declare_parameter("final_pose_smoothing_alpha", 1.0).value
            )
            self._final_route_projection_gain = float(
                self.declare_parameter("final_route_projection_gain", 0.0).value
            )
            self._final_route_projection_use_pose_projection = bool(
                self.declare_parameter("final_route_projection_use_pose_projection", False).value
            )
            self._final_route_projection_yaw_gain = float(
                self.declare_parameter("final_route_projection_yaw_gain", 0.0).value
            )
            self._final_route_projection_gate_m = float(
                self.declare_parameter("final_route_projection_gate_m", 2.0).value
            )
            max_hypotheses = int(self.declare_parameter("max_hypotheses", 7).value)
            max_candidate_i2r_m = float(
                self.declare_parameter("max_candidate_initial_to_result_m", 6.0).value
            )
            min_candidate_score = float(self.declare_parameter("min_candidate_score", -math.inf).value)
            enable_not_converged_partial_candidates = bool(
                self.declare_parameter("enable_not_converged_partial_candidates", False).value
            )
            not_converged_partial_min_nvtl = float(
                self.declare_parameter("not_converged_partial_min_nvtl", -math.inf).value
            )
            best_switch_score_margin = float(
                self.declare_parameter("best_switch_score_margin", 0.0).value
            )
            relative_residual_weight = float(
                self.declare_parameter("relative_residual_weight", 1.0).value
            )
            route_cross_weight = float(
                self.declare_parameter("route_cross_weight", 0.8).value
            )
            route_yaw_weight = float(
                self.declare_parameter("route_yaw_weight", 0.2).value
            )
            route_progress_weight = float(
                self.declare_parameter("route_progress_weight", 0.0).value
            )
            gnss_weak_prior_weight = float(
                self.declare_parameter("gnss_weak_prior_weight", 1.0).value
            )
            route_progress_update_gain = float(
                self.declare_parameter("route_progress_update_gain", 1.0).value
            )
            route_progress_update_max_age_sec = float(
                self.declare_parameter("route_progress_update_max_age_sec", math.inf).value
            )
            enable_degenerate_along_remap = bool(
                self.declare_parameter("enable_degenerate_along_remap", False).value
            )
            enable_candidate_localizability_along_remap = bool(
                self.declare_parameter("enable_candidate_localizability_along_remap", False).value
            )
            enable_candidate_low_score_along_remap = bool(
                self.declare_parameter("enable_candidate_low_score_along_remap", False).value
            )
            candidate_low_score_along_remap_threshold = float(
                self.declare_parameter("candidate_low_score_along_remap_threshold", 2.3).value
            )
            candidate_localizability_along_min_variance_m2 = float(
                self.declare_parameter(
                    "candidate_localizability_along_min_variance_m2",
                    1.0,
                ).value
            )
            candidate_localizability_along_min_ratio = float(
                self.declare_parameter(
                    "candidate_localizability_along_min_ratio",
                    4.0,
                ).value
            )
            degenerate_remap_use_route_frame = bool(
                self.declare_parameter("degenerate_remap_use_route_frame", False).value
            )
            degenerate_remap_keep_predicted_yaw = bool(
                self.declare_parameter("degenerate_remap_keep_predicted_yaw", False).value
            )
            degenerate_keep_predicted_yaw_only = bool(
                self.declare_parameter("degenerate_keep_predicted_yaw_only", False).value
            )
            degenerate_skip_ndt_candidates = bool(
                self.declare_parameter("degenerate_skip_ndt_candidates", False).value
            )
            self._degenerate_along_min_variance_m2 = float(
                self.declare_parameter("degenerate_along_min_variance_m2", 4.0).value
            )
            self._degenerate_along_min_ratio = float(
                self.declare_parameter("degenerate_along_min_ratio", 4.0).value
            )
            self._degenerate_along_hold_sec = max(
                0.0, float(self.declare_parameter("degenerate_along_hold_sec", 0.0).value)
            )
            self._degenerate_along_until_sec = -math.inf
            degenerate_max_lateral_m = float(
                self.declare_parameter("degenerate_max_lateral_m", math.inf).value
            )
            degenerate_max_yaw_rad = math.radians(
                float(self.declare_parameter("degenerate_max_yaw_deg", 180.0).value)
            )
            enable_submap_candidate_consistency_gate = bool(
                self.declare_parameter("enable_submap_candidate_consistency_gate", False).value
            )
            submap_candidate_max_lateral_m = float(
                self.declare_parameter("submap_candidate_max_lateral_m", 1.5).value
            )
            submap_candidate_max_yaw_rad = math.radians(
                float(self.declare_parameter("submap_candidate_max_yaw_deg", 3.0).value)
            )
            submap_candidate_max_progress_innovation_m = float(
                self.declare_parameter("submap_candidate_max_progress_innovation_m", 2.0).value
            )
            submap_candidate_max_residual_m = float(
                self.declare_parameter("submap_candidate_max_residual_m", 2.0).value
            )
            submap_candidate_min_residual_improvement_m = float(
                self.declare_parameter("submap_candidate_min_residual_improvement_m", 0.0).value
            )
            enable_candidate_residual_consistency_gate = bool(
                self.declare_parameter("enable_candidate_residual_consistency_gate", False).value
            )
            candidate_residual_max_m = float(
                self.declare_parameter("candidate_residual_max_m", 2.0).value
            )
            candidate_residual_min_improvement_m = float(
                self.declare_parameter("candidate_residual_min_improvement_m", 0.2).value
            )
            self._xy_covariance = float(self.declare_parameter("xy_covariance_m2", 0.25).value)
            self._yaw_covariance = float(
                self.declare_parameter("yaw_covariance_rad2", math.radians(3.0) ** 2).value
            )
            self._max_initial_deviation_from_fallback_m = float(
                self.declare_parameter("max_initial_deviation_from_fallback_m", 2.0).value
            )
            self._config = TrackerConfig(
                max_hypotheses=max_hypotheses,
                max_candidate_initial_to_result_m=max_candidate_i2r_m,
                min_candidate_score=min_candidate_score,
                enable_not_converged_partial_candidates=(
                    enable_not_converged_partial_candidates
                ),
                not_converged_partial_min_nvtl=not_converged_partial_min_nvtl,
                relative_residual_weight=relative_residual_weight,
                route_cross_weight=route_cross_weight,
                route_yaw_weight=route_yaw_weight,
                route_progress_weight=route_progress_weight,
                gnss_weak_prior_weight=gnss_weak_prior_weight,
                route_progress_update_gain=route_progress_update_gain,
                route_progress_update_max_age_sec=route_progress_update_max_age_sec,
                enable_degenerate_along_remap=enable_degenerate_along_remap,
                enable_candidate_localizability_along_remap=(
                    enable_candidate_localizability_along_remap
                ),
                candidate_localizability_along_min_variance_m2=(
                    candidate_localizability_along_min_variance_m2
                ),
                candidate_localizability_along_min_ratio=(
                    candidate_localizability_along_min_ratio
                ),
                enable_candidate_low_score_along_remap=enable_candidate_low_score_along_remap,
                candidate_low_score_along_remap_threshold=(
                    candidate_low_score_along_remap_threshold
                ),
                degenerate_remap_use_route_frame=degenerate_remap_use_route_frame,
                degenerate_remap_keep_predicted_yaw=degenerate_remap_keep_predicted_yaw,
                degenerate_keep_predicted_yaw_only=degenerate_keep_predicted_yaw_only,
                degenerate_skip_ndt_candidates=degenerate_skip_ndt_candidates,
                degenerate_max_lateral_m=degenerate_max_lateral_m,
                degenerate_max_yaw_rad=degenerate_max_yaw_rad,
                enable_submap_candidate_consistency_gate=(
                    enable_submap_candidate_consistency_gate
                ),
                submap_candidate_max_lateral_m=submap_candidate_max_lateral_m,
                submap_candidate_max_yaw_rad=submap_candidate_max_yaw_rad,
                submap_candidate_max_progress_innovation_m=(
                    submap_candidate_max_progress_innovation_m
                ),
                submap_candidate_max_residual_m=submap_candidate_max_residual_m,
                submap_candidate_min_residual_improvement_m=(
                    submap_candidate_min_residual_improvement_m
                ),
                enable_candidate_residual_consistency_gate=(
                    enable_candidate_residual_consistency_gate
                ),
                candidate_residual_max_m=candidate_residual_max_m,
                candidate_residual_min_improvement_m=(
                    candidate_residual_min_improvement_m
                ),
                best_switch_score_margin=best_switch_score_margin,
            )
            self._scan_submap_voxel_size_m = float(
                self.declare_parameter("scan_submap_voxel_size_m", 0.8).value
            )
            self._scan_submap_max_cells = int(
                self.declare_parameter("scan_submap_max_cells", 20000).value
            )
            self._scan_submap_neighbor_radius_cells = int(
                self.declare_parameter("scan_submap_neighbor_radius_cells", 1).value
            )
            self._scan_submap_unmatched_penalty_m = float(
                self.declare_parameter("scan_submap_unmatched_penalty_m", 4.0).value
            )
            self._hypothesis_submaps: dict[int, LightweightScanSubmap] = {}
            self._last_submap_added_by_hypothesis: dict[int, float] = {}
            self._last_scan_corrected_by_hypothesis: dict[int, float] = {}
            self._anchor_submap: LightweightScanSubmap | None = None
            self._anchor_submap_stamp_sec: float | None = None
            self._anchor_submap_source_hypothesis_id: int | None = None
            self._usable_ndt_streak: int = 0
            self._latest_scan_points: list[Point2D] = []
            self._latest_scan_sampled_point_count: int = 0
            self._latest_scan_stamp_sec: float | None = None
            self._latest_gnss_weak_prior_pose: Pose2D | None = None
            self._last_gnss_weak_prior_summary: dict[str, float | int | bool] = {
                "gnss_weak_prior_active": False,
                "gnss_weak_prior_candidate_count": 0,
                "gnss_weak_prior_initial_hint_active": False,
            }
            self._latest_scan_geometry_certificate = ScanGeometryCertificate(
                0, 0.0, 0.0, 0, 0, True, math.inf, False
            )
            self._previous_lro_scan_points: list[Point2D] = []
            self._previous_lro_scan_stamp_sec: float | None = None
            self._predicted_forward_since_lro_scan: float = 0.0
            self._route_path: RoutePath | None = None
            if route_samples_csv:
                try:
                    self._route_path = RoutePath.from_csv(route_samples_csv)
                except Exception as exc:
                    self.get_logger().warning(f"failed to load route_samples_csv: {exc}")
            self._global_map_submap: LightweightScanSubmap | None = None
            if self._enable_global_map_local_ndt_candidates and self._global_map_pcd_dir:
                self._global_map_submap = self._load_global_map_submap(self._global_map_pcd_dir)
            self._tracker: FixedLagMultiHypothesisTracker | None = None
            self._has_candidate_update = False
            self._tracker_init_stamp_sec: float | None = None
            self._debug_initial_publish_count = 0
            self._debug_final_publish_count = 0
            self._last_runtime_update_had_usable_ndt = False
            self._last_runtime_update_had_converged_ndt = False
            self._smoothed_final_pose: Pose2D | None = None
            self._latest_fallback_initial: Pose2D | None = None
            self._latest_fallback_z: float = 0.0
            self._last_motion_stamp_sec: float | None = None
            self._bias_anchor_pose: Pose2D | None = None
            self._bias_anchor_stamp_sec: float | None = None
            self._bias_integrated_forward_m: float = 0.0
            self._learned_twist_bias_mps: float = 0.0
            self._twist_bias_update_count: int = 0
            self._twist_bias_last_reason: str = "not_started"
            self._twist_bias_last_estimate_mps: float | None = None
            self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, self._output_pose_topic, 10)
            self._initial_pub = self.create_publisher(
                PoseWithCovarianceStamped, self._output_initial_topic, 10
            )
            self._diag_pub = self.create_publisher(String, self._diagnostics_topic, 10)
            self.create_subscription(String, self._input_topic, self._on_runtime_decision, 10)
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._fallback_initial_topic,
                self._on_fallback_initial_pose,
                10,
            )
            self.create_subscription(VelocityReport, self._velocity_topic, self._on_velocity, 50)
            if self._enable_gnss_weak_prior:
                if self._gnss_weak_prior_msg_type == "pose_with_covariance":
                    self.create_subscription(
                        PoseWithCovarianceStamped,
                        self._gnss_weak_prior_topic,
                        self._on_gnss_weak_prior_pose,
                        20,
                    )
                else:
                    self.create_subscription(
                        Odometry,
                        self._gnss_weak_prior_topic,
                        self._on_gnss_weak_prior,
                        20,
                    )
            if needs_pointcloud_subscription(
                enable_scan_submap_residual=self._enable_scan_submap_residual,
                enable_scan_point_count_degeneracy=self._enable_scan_point_count_degeneracy,
                enable_lro_forward_correction=self._enable_lro_forward_correction,
                enable_scan_geometry_degeneracy=self._enable_scan_geometry_degeneracy,
            ):
                self.create_subscription(
                    PointCloud2,
                    self._pointcloud_topic,
                    self._on_pointcloud,
                    qos_profile_sensor_data,
                )

        def _load_global_map_submap(self, map_dir: str) -> LightweightScanSubmap | None:
            path = Path(map_dir)
            if not path.exists():
                self.get_logger().warning(f"global_map_pcd_dir does not exist: {map_dir}")
                return None
            submap = LightweightScanSubmap(
                voxel_size_m=self._global_map_voxel_size_m,
                max_cells=self._global_map_max_cells,
                neighbor_radius_cells=self._scan_submap_neighbor_radius_cells,
                unmatched_penalty_m=self._scan_submap_unmatched_penalty_m,
            )
            added = 0
            seen_lines = 0
            for pcd_path in sorted(path.glob("*.pcd")):
                data = False
                batch: list[Point2D] = []
                try:
                    with pcd_path.open("r", encoding="utf-8", errors="ignore") as handle:
                        for line in handle:
                            if not data:
                                if line.strip().lower().startswith("data"):
                                    data = True
                                continue
                            seen_lines += 1
                            if seen_lines % self._global_map_pcd_stride != 0:
                                continue
                            fields = line.split()
                            if len(fields) < 2:
                                continue
                            try:
                                batch.append((float(fields[0]), float(fields[1])))
                            except ValueError:
                                continue
                            if len(batch) >= 2048:
                                added += submap.add_world_points(batch)
                                batch.clear()
                                if len(submap._points2d) >= self._global_map_max_points:
                                    break
                    if batch:
                        added += submap.add_world_points(batch)
                    if len(submap._points2d) >= self._global_map_max_points:
                        break
                except Exception as exc:
                    self.get_logger().warning(f"failed to read map pcd {pcd_path}: {exc}")
            self.get_logger().info(
                "global_map_local_ndt_loaded "
                f"points={len(submap._points2d)} cells={submap.cell_count} added={added}"
            )
            return submap if submap.cell_count > 0 else None

        def _on_pointcloud(self, msg: PointCloud2) -> None:
            points: list[Point2D] = []
            sampled_point_count = 0
            try:
                field_names = (
                    ("x", "y", "z")
                    if self._scan_submap_icp_candidate_match_in_3d
                    else ("x", "y")
                )
                rows = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
                for index, row in enumerate(rows):
                    if index % self._scan_submap_sample_stride != 0:
                        continue
                    x = float(row[0])
                    y = float(row[1])
                    distance = math.hypot(x, y)
                    if distance < self._scan_submap_min_range_m:
                        continue
                    if distance > self._scan_submap_max_range_m:
                        continue
                    sampled_point_count += 1
                    if len(points) < self._scan_submap_max_points:
                        if self._scan_submap_icp_candidate_match_in_3d and len(row) >= 3:
                            points.append((x, y, float(row[2])))
                        else:
                            points.append((x, y))
            except Exception as exc:
                self._publish_diag({"status": "SCAN_ERROR", "reason": str(exc)})
                return
            self._latest_scan_points = points
            self._latest_scan_sampled_point_count = sampled_point_count
            self._latest_scan_stamp_sec = stamp_to_float(msg.header.stamp)
            self._latest_scan_geometry_certificate = scan_geometry_certificate(
                points,
                min_points=self._scan_geometry_min_points,
                min_cross_side_points=self._scan_geometry_min_cross_side_points,
                min_along_to_cross_ratio=self._scan_geometry_min_along_to_cross_ratio,
                min_along_span_m=self._scan_geometry_min_along_span_m,
                max_cross_span_m=self._scan_geometry_max_cross_span_m,
            )
            self._apply_lro_scan_to_scan_correction()
            self._previous_lro_scan_points = list(points)
            self._previous_lro_scan_stamp_sec = self._latest_scan_stamp_sec
            self._predicted_forward_since_lro_scan = 0.0

        def _apply_lro_scan_to_scan_correction(self) -> None:
            if (
                not self._enable_lro_forward_correction
                or self._tracker is None
                or not self._previous_lro_scan_points
                or not self._latest_scan_points
                or self._latest_scan_stamp_sec is None
                or self._previous_lro_scan_stamp_sec is None
            ):
                return
            estimate = estimate_scan_to_scan_motion_2d(
                self._previous_lro_scan_points,
                self._latest_scan_points,
                initial_forward_m=self._predicted_forward_since_lro_scan,
                initial_lateral_m=0.0,
                initial_yaw_rad=0.0,
                max_match_distance_m=self._lro_max_match_distance_m,
                min_quality=self._lro_min_quality,
                max_residual_m=self._lro_max_residual_m,
                max_points=self._scan_submap_max_points,
                reject_degenerate=self._lro_reject_degenerate,
            )
            if (
                not estimate.is_valid
                or estimate.quality < self._lro_min_quality
                or estimate.residual_m > self._lro_max_residual_m
            ):
                self._publish_diag(
                    {
                        "status": "LRO_REJECTED",
                        "stamp_sec": self._latest_scan_stamp_sec,
                        "reason": estimate.reason,
                        "lro_quality": estimate.quality,
                        "lro_residual_m": estimate.residual_m,
                        "lro_along_degenerate": estimate.along_degenerate,
                        "predicted_forward_m": self._predicted_forward_since_lro_scan,
                        "uses_gnss_or_gt": False,
                    }
                )
                return
            corrected: dict[int, Pose2D] = {}
            for hypothesis in self._tracker.hypotheses:
                pose = apply_lro_forward_correction(
                    hypothesis.pose,
                    estimate,
                    predicted_forward_m=self._predicted_forward_since_lro_scan,
                    enabled=True,
                    gain=self._lro_forward_correction_gain,
                    max_correction_m=self._lro_forward_correction_max_m,
                )
                if pose != hypothesis.pose:
                    corrected[hypothesis.id] = pose
            if corrected:
                self._tracker.apply_pose_correction(corrected)
            self._publish_diag(
                {
                    "status": "LRO_APPLIED" if corrected else "LRO_NOOP",
                    "stamp_sec": self._latest_scan_stamp_sec,
                    "lro_forward_m": estimate.forward_m,
                    "predicted_forward_m": self._predicted_forward_since_lro_scan,
                    "lro_quality": estimate.quality,
                    "lro_residual_m": estimate.residual_m,
                    "lro_along_degenerate": estimate.along_degenerate,
                    "corrected_hypothesis_count": len(corrected),
                    "uses_gnss_or_gt": False,
                }
            )

        def _on_fallback_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
            pose = self._pose_from_msg(msg)
            fallback_projection = self._project_route(pose)
            if (
                fallback_projection is not None
                and fallback_projection.is_valid
                and abs(fallback_projection.cross_track_m) > self._fallback_route_cross_gate_m
            ):
                self._publish_diag(
                    {
                        "status": "FALLBACK_REJECTED",
                        "reason": "route_cross_gate",
                        "stamp_sec": pose.stamp_sec,
                        "route_cross_track_m": fallback_projection.cross_track_m,
                        "route_progress_m": fallback_projection.progress_m,
                        "uses_gnss_or_gt": False,
                    }
                )
                return
            if self._route_cross_target_m is None and fallback_projection is not None and fallback_projection.is_valid:
                self._route_cross_target_m = fallback_projection.cross_track_m
            if (
                self._learn_correction_target_from_startup
                and self._route_cross_correction_target_m is None
                and fallback_projection is not None
                and fallback_projection.is_valid
                and abs(fallback_projection.cross_track_m) <= self._fallback_route_cross_gate_m
            ):
                limit_m = abs(self._learned_correction_target_abs_limit_m)
                target_m = fallback_projection.cross_track_m
                if math.isfinite(limit_m):
                    target_m = max(-limit_m, min(limit_m, target_m))
                self._route_cross_correction_target_m = target_m
                self._learned_correction_target_count += 1
            self._latest_fallback_initial = pose
            self._latest_fallback_z = float(msg.pose.pose.position.z)
            initialized_now = False
            if self._tracker is None:
                self._tracker = FixedLagMultiHypothesisTracker(
                    pose,
                    self._config,
                    initial_route_progress_m=self._initial_route_progress(
                        pose, fallback_projection
                    ),
                    initial_score=self._fallback_startup_score(fallback_projection),
                    route_path=self._route_path,
                )
                if self._last_motion_stamp_sec is None:
                    self._last_motion_stamp_sec = pose.stamp_sec
                self._tracker_init_stamp_sec = pose.stamp_sec
                self._sync_hypothesis_submaps()
                initialized_now = True
                self._add_startup_route_branch_hypotheses(pose)
            elif (
                self._refresh_from_fallback_until_first_candidate
                and not self._has_candidate_update
            ):
                # Before the first accepted LiDAR/NDT update, the predictor
                # fallback is the only causal startup-only prior available to
                # keep NDT initial poses current.  Do not let an older startup
                # pose coast for seconds and create an artificial first-frame
                # outlier; once LiDAR candidates have been incorporated, stop
                # refreshing from this side channel.
                self._tracker = FixedLagMultiHypothesisTracker(
                    pose,
                    self._config,
                    initial_route_progress_m=self._initial_route_progress(
                        pose, fallback_projection
                    ),
                    initial_score=self._fallback_startup_score(fallback_projection),
                    route_path=self._route_path,
                )
                self._last_motion_stamp_sec = pose.stamp_sec
                self._sync_hypothesis_submaps()
                self._add_startup_route_branch_hypotheses(pose)
            elif not self._has_candidate_update:
                self._tracker.add_startup_hypothesis(
                    pose,
                    route_progress_m=self._initial_route_progress(pose, fallback_projection),
                    score=self._fallback_startup_score(fallback_projection),
                )
                self._add_startup_route_branch_hypotheses(pose)
                self._sync_hypothesis_submaps()
            if self._forward_fallback_initial and (
                initialized_now or self._forward_fallback_after_tracker_init
            ):
                self._initial_pub.publish(msg)
            if self._tracker is not None and not self._has_candidate_update:
                best = self._tracker.best()
                self._publish_pose(
                    self._select_initial_pose(best.pose),
                    self._initial_pub,
                    use_pose_stamp=False,
                )
                if self._publish_pose_before_first_candidate:
                    self._publish_final_pose(best.pose)
            self._publish_diag(
                {
                    "status": "FALLBACK_INITIAL",
                    "stamp_sec": pose.stamp_sec,
                    "tracker_initialized": self._tracker is not None,
                    "forwarded_as_initial": self._forward_fallback_initial,
                    "initialized_now": initialized_now,
                    "startup_hypothesis_count": (
                        len(self._tracker.hypotheses) if self._tracker is not None else 0
                    ),
                    "startup_route_branch_hypotheses_enabled": (
                        self._enable_startup_route_branch_hypotheses
                    ),
                    "route_cross_correction_target_m": self._route_cross_correction_target(),
                    "learned_correction_target_count": self._learned_correction_target_count,
                    "uses_gnss_or_gt": False,
                }
            )

        def _on_gnss_weak_prior(self, msg: Odometry) -> None:
            q = msg.pose.pose.orientation
            gnss_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            base_yaw = normalize_angle(gnss_yaw - (-1.57079632679))
            offset_x = math.cos(base_yaw) * 1.90
            offset_y = math.sin(base_yaw) * 1.90
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
            self._latest_gnss_weak_prior_pose = Pose2D(
                stamp_sec=stamp_sec,
                x=float(msg.pose.pose.position.x) - offset_x,
                y=float(msg.pose.pose.position.y) - offset_y,
                yaw=base_yaw,
            )

        def _on_gnss_weak_prior_pose(self, msg: PoseWithCovarianceStamped) -> None:
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
            self._latest_gnss_weak_prior_pose = Pose2D(
                stamp_sec=stamp_sec,
                x=float(msg.pose.pose.position.x),
                y=float(msg.pose.pose.position.y),
                yaw=yaw,
            )

        def _gnss_weak_prior_penalties(
            self, candidates: list[NdtCandidate], stamp_sec: float
        ) -> dict[int, WeakPriorPenalty]:
            self._last_gnss_weak_prior_summary = {
                "gnss_weak_prior_active": False,
                "gnss_weak_prior_candidate_count": 0,
            }
            if (
                not self._enable_gnss_weak_prior
                or self._latest_gnss_weak_prior_pose is None
                or not candidates
            ):
                return {}
            age_sec = abs(stamp_sec - self._latest_gnss_weak_prior_pose.stamp_sec)
            if age_sec > self._gnss_weak_prior_max_age_sec:
                self._last_gnss_weak_prior_summary = {
                    "gnss_weak_prior_active": False,
                    "gnss_weak_prior_age_sec": age_sec,
                    "gnss_weak_prior_candidate_count": 0,
                }
                return {}
            sigma = self._gnss_weak_prior_sigma_m
            penalties: dict[int, WeakPriorPenalty] = {}
            distances: list[float] = []
            penalty_values: list[float] = []
            for index, candidate in enumerate(candidates):
                distance_m = math.hypot(
                    candidate.pose.x - self._latest_gnss_weak_prior_pose.x,
                    candidate.pose.y - self._latest_gnss_weak_prior_pose.y,
                )
                penalty = min(
                    self._gnss_weak_prior_max_penalty,
                    (distance_m * distance_m) / (2.0 * sigma * sigma),
                )
                penalties[index] = WeakPriorPenalty(
                    penalty=penalty,
                    distance_m=distance_m,
                    is_valid=True,
                )
                distances.append(distance_m)
                penalty_values.append(penalty)
            self._last_gnss_weak_prior_summary = {
                "gnss_weak_prior_active": True,
                "gnss_weak_prior_age_sec": age_sec,
                "gnss_weak_prior_sigma_m": sigma,
                "gnss_weak_prior_candidate_count": len(penalties),
                "gnss_weak_prior_distance_min_m": min(distances),
                "gnss_weak_prior_distance_max_m": max(distances),
                "gnss_weak_prior_penalty_min": min(penalty_values),
                "gnss_weak_prior_penalty_max": max(penalty_values),
            }
            return penalties

        def _on_velocity(self, msg: VelocityReport) -> None:
            if self._tracker is None:
                return
            stamp_msg = msg.header.stamp if hasattr(msg, "header") else getattr(msg, "stamp", None)
            stamp_sec = (
                float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
                if stamp_msg is not None
                else 0.0
            )
            if stamp_sec <= 0.0:
                stamp_sec = self.get_clock().now().nanoseconds * 1e-9
            if self._last_motion_stamp_sec is None:
                self._last_motion_stamp_sec = stamp_sec
                return
            dt_sec = stamp_sec - self._last_motion_stamp_sec
            if not motion_dt_is_usable(dt_sec, self._motion_max_dt_sec):
                self._last_motion_stamp_sec = stamp_sec
                return
            self._last_motion_stamp_sec = stamp_sec
            velocity_mps = float(msg.longitudinal_velocity) * self._velocity_scale + self._velocity_bias_mps
            if self._enable_twist_bias_learning:
                velocity_mps += self._learned_twist_bias_mps
            velocity_mps = apply_degenerate_velocity_scale(
                velocity_mps,
                degenerate=scan_point_count_indicates_degeneracy(
                    sampled_point_count=self._latest_scan_sampled_point_count,
                    enabled=self._enable_scan_point_count_degeneracy,
                    min_sampled_points=self._scan_point_count_degeneracy_min_sampled_points,
                ),
                enabled=self._enable_degenerate_velocity_scale,
                scale=self._degenerate_velocity_scale,
            )
            yaw_rate_radps = float(msg.heading_rate)
            if not math.isfinite(velocity_mps):
                velocity_mps = 0.0
            if not math.isfinite(yaw_rate_radps):
                yaw_rate_radps = 0.0
            self._predicted_forward_since_lro_scan += velocity_mps * dt_sec
            self._tracker.propagate(
                MotionDelta(
                    dt_sec=dt_sec,
                    forward_m=velocity_mps * dt_sec,
                    lateral_m=0.0,
                    yaw_rad=yaw_rate_radps * dt_sec,
                )
            )
            if self._enable_twist_bias_learning:
                raw_velocity_mps = float(msg.longitudinal_velocity) * self._velocity_scale
                if math.isfinite(raw_velocity_mps):
                    self._bias_integrated_forward_m += raw_velocity_mps * dt_sec
            self._sync_hypothesis_submaps()
            self._apply_route_cross_correction()
            self._apply_scan_submap_correction()
            best = self._tracker.best()
            self._publish_pose(
                self._select_initial_pose(best.pose),
                self._initial_pub,
                use_pose_stamp=False,
            )
            if self._has_candidate_update or self._publish_pose_before_first_candidate:
                self._publish_final_pose(best.pose)

        def _on_runtime_decision(self, msg: String) -> None:
            try:
                payload = json.loads(msg.data)
            except Exception as exc:
                self._publish_diag({"status": "ERROR", "reason": f"invalid_json:{exc}"})
                return
            along_degenerate = payload_indicates_along_degeneracy(
                payload,
                min_along_variance_m2=self._degenerate_along_min_variance_m2,
                min_along_to_cross_ratio=self._degenerate_along_min_ratio,
            )
            stamp_sec = float(payload.get("stamp_sec", 0.0) or 0.0)
            if along_degenerate and self._degenerate_along_hold_sec > 0.0:
                self._degenerate_along_until_sec = max(
                    self._degenerate_along_until_sec,
                    stamp_sec + self._degenerate_along_hold_sec,
                )
            if stamp_sec <= self._degenerate_along_until_sec:
                along_degenerate = True
            scan_point_count_degenerate = scan_point_count_indicates_degeneracy(
                sampled_point_count=self._latest_scan_sampled_point_count,
                enabled=self._enable_scan_point_count_degeneracy,
                min_sampled_points=self._scan_point_count_degeneracy_min_sampled_points,
            )
            if scan_point_count_degenerate:
                along_degenerate = True
            if (
                self._enable_scan_geometry_degeneracy
                and self._route_path is not None
                and self._tracker is not None
                and self._latest_scan_points
            ):
                best_for_geometry = self._tracker.best()
                self._latest_scan_geometry_certificate = route_frame_scan_geometry_certificate(
                    self._latest_scan_points,
                    best_for_geometry.pose,
                    self._route_path,
                    route_progress_m=best_for_geometry.route_progress_m,
                    search_radius_m=self._scan_geometry_route_search_radius_m,
                    min_points=self._scan_geometry_min_points,
                    min_cross_side_points=self._scan_geometry_min_cross_side_points,
                    min_along_to_cross_ratio=self._scan_geometry_min_along_to_cross_ratio,
                    min_along_span_m=self._scan_geometry_min_along_span_m,
                    max_cross_span_m=self._scan_geometry_max_cross_span_m,
                )
            scan_geometry_degenerate = (
                self._enable_scan_geometry_degeneracy
                and self._latest_scan_geometry_certificate.along_degenerate
            )
            if self._scan_geometry_use_as_degenerate and scan_geometry_degenerate:
                along_degenerate = True
            localizability_summary = payload_candidate_localizability_summary(payload)
            candidates = candidates_from_runtime_multistart(payload)
            candidates = self._route_filter_candidates(candidates)
            icp_candidate_count = 0
            local_ndt_candidate_count = 0
            global_ndt_candidate_count = 0
            usable = [candidate for candidate in candidates if candidate_is_usable(candidate, self._config)]
            self._last_runtime_update_had_usable_ndt = bool(usable)
            converged_usable = [
                candidate
                for candidate in candidates
                if candidate_can_refresh_scan_submap_anchor(candidate, self._config)
            ]
            self._last_runtime_update_had_converged_ndt = bool(converged_usable)
            self._usable_ndt_streak = self._usable_ndt_streak + 1 if converged_usable else 0
            if self._tracker is None:
                if not usable:
                    self._publish_diag(
                        {
                            "status": "WAITING",
                            "reason": "no_converged_candidate_for_initialization",
                            "candidate_count": len(candidates),
                        }
                    )
                    return
                self._tracker = FixedLagMultiHypothesisTracker(
                    usable[0].pose,
                    self._config,
                    initial_route_progress_m=self._initial_route_progress(usable[0].pose),
                    route_path=self._route_path,
                )
                self._tracker_init_stamp_sec = usable[0].pose.stamp_sec
                self._has_candidate_update = True

            icp_candidates = (
                self._scan_submap_icp_candidates()
                if (not usable or not self._scan_submap_icp_candidates_only_when_ndt_empty)
                else []
            )
            if icp_candidates:
                candidates = [*candidates, *icp_candidates]
                icp_candidate_count = len(icp_candidates)
            local_ndt_candidates = (
                self._scan_submap_local_ndt_candidates()
                if (not usable or not self._scan_submap_local_ndt_candidates_only_when_ndt_empty)
                else []
            )
            if local_ndt_candidates:
                candidates = [*candidates, *local_ndt_candidates]
                local_ndt_candidate_count = len(local_ndt_candidates)
            global_ndt_candidates = (
                self._global_map_local_ndt_candidates()
                if (not usable or not self._global_map_local_ndt_candidates_only_when_ndt_empty)
                else []
            )
            if global_ndt_candidates:
                candidates = [*candidates, *global_ndt_candidates]
                global_ndt_candidate_count = len(global_ndt_candidates)
            route_priors = self._route_priors_for_candidates(candidates)
            valid_candidates = [
                candidate for candidate in candidates if self._tracker._candidate_is_usable(candidate)
            ]
            initialized_from_first_usable_candidate = False
            if (
                self._initialize_from_first_usable_candidate
                and usable
                and not self._has_candidate_update
            ):
                self._tracker = FixedLagMultiHypothesisTracker(
                    usable[0].pose,
                    self._config,
                    initial_route_progress_m=self._initial_route_progress(usable[0].pose),
                    route_path=self._route_path,
                )
                self._tracker_init_stamp_sec = usable[0].pose.stamp_sec
                self._last_motion_stamp_sec = usable[0].pose.stamp_sec
                self._sync_hypothesis_submaps()
                initialized_from_first_usable_candidate = True
            gnss_weak_prior_penalties = self._gnss_weak_prior_penalties(
                valid_candidates,
                stamp_sec,
            )
            residuals = self._scan_submap_residuals_for_candidates(valid_candidates)
            self._tracker.update(
                candidates,
                route_priors=route_priors,
                relative_residuals=residuals,
                candidate_penalties=gnss_weak_prior_penalties,
                along_degenerate=along_degenerate,
            )
            self._sync_hypothesis_submaps()
            self._apply_scan_submap_correction()
            if usable:
                self._has_candidate_update = True
            best = self._tracker.best()
            if self._learn_correction_target_from_ndt and usable and not along_degenerate:
                self._update_route_cross_target_from_pose(
                    best.pose,
                    along_degenerate=along_degenerate,
                )
            self._maybe_update_twist_bias(best, has_usable_candidate=bool(usable))
            self._publish_final_pose(best.pose)
            self._maybe_add_current_scan_to_hypothesis_submaps()
            initial_pose = self._select_initial_pose(best.pose)
            self._publish_pose(initial_pose, self._initial_pub, use_pose_stamp=False)
            self._publish_diag(
                {
                    "status": "OK",
                    "stamp_sec": best.pose.stamp_sec,
                    "candidate_count": len(candidates),
                    **candidate_confidence_summary(candidates, self._config),
                    **localizability_summary,
                    "along_degenerate": along_degenerate,
                    "scan_point_count_degenerate": scan_point_count_degenerate,
                    "scan_geometry_degenerate": scan_geometry_degenerate,
                    "scan_sampled_point_count": self._latest_scan_sampled_point_count,
                    "scan_geometry_point_count": (
                        self._latest_scan_geometry_certificate.point_count
                    ),
                    "scan_geometry_along_span_m": (
                        self._latest_scan_geometry_certificate.along_span_m
                    ),
                    "scan_geometry_cross_span_m": (
                        self._latest_scan_geometry_certificate.cross_span_m
                    ),
                    "scan_geometry_left_count": (
                        self._latest_scan_geometry_certificate.left_count
                    ),
                    "scan_geometry_right_count": (
                        self._latest_scan_geometry_certificate.right_count
                    ),
                    "scan_geometry_one_sided": (
                        self._latest_scan_geometry_certificate.one_sided
                    ),
                    "scan_geometry_along_to_cross_ratio": (
                        self._latest_scan_geometry_certificate.along_to_cross_ratio
                    ),
                    "scan_submap_icp_candidate_count": icp_candidate_count,
                    "scan_submap_local_ndt_candidate_count": local_ndt_candidate_count,
                    "global_map_local_ndt_candidate_count": global_ndt_candidate_count,
                    "hypothesis_count": len(self._tracker.hypotheses),
                    "best_hypothesis_id": best.id,
                    "best_route_progress_m": best.route_progress_m,
                    "best_score": best.score,
                    "learned_twist_bias_mps": self._learned_twist_bias_mps,
                    "twist_bias_update_count": self._twist_bias_update_count,
                    "twist_bias_last_reason": self._twist_bias_last_reason,
                    "twist_bias_last_estimate_mps": self._twist_bias_last_estimate_mps,
                    "route_cross_correction_target_m": self._route_cross_correction_target(),
                    "learned_correction_target_count": self._learned_correction_target_count,
                    "initial_pose_source": (
                        "fallback_guard" if initial_pose is self._latest_fallback_initial else "tracker"
                    ),
                    "scan_submap_cells": self._best_submap_cell_count(),
                    "scan_submap_anchor_active": self._anchor_submap_is_active(),
                    "scan_submap_anchor_cells": (
                        self._anchor_submap.cell_count if self._anchor_submap is not None else 0
                    ),
                    "scan_submap_anchor_age_sec": self._anchor_submap_age_sec(),
                    "scan_submap_anchor_source_hypothesis_id": self._anchor_submap_source_hypothesis_id,
                    "usable_ndt_streak": self._usable_ndt_streak,
                    "converged_ndt_update": self._last_runtime_update_had_converged_ndt,
                    "initialized_from_first_usable_candidate": (
                        initialized_from_first_usable_candidate
                    ),
                    **self._last_gnss_weak_prior_summary,
                    "scan_submap_residual_count": len(residuals),
                    "uses_gnss_or_gt": False,
                }
            )

        def _publish_pose(self, pose: Pose2D, publisher, *, use_pose_stamp: bool) -> None:
            msg = PoseWithCovarianceStamped()
            if use_pose_stamp:
                stamp_sec = max(float(pose.stamp_sec), 0.0)
                msg.header.stamp.sec = int(math.floor(stamp_sec))
                msg.header.stamp.nanosec = int(round((stamp_sec - math.floor(stamp_sec)) * 1.0e9))
                if msg.header.stamp.nanosec >= 1_000_000_000:
                    msg.header.stamp.sec += 1
                    msg.header.stamp.nanosec -= 1_000_000_000
            else:
                msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(self._frame_id)
            msg.pose.pose.position.x = pose.x
            msg.pose.pose.position.y = pose.y
            msg.pose.pose.position.z = self._latest_fallback_z
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = math.sin(0.5 * pose.yaw)
            msg.pose.pose.orientation.w = math.cos(0.5 * pose.yaw)
            msg.pose.covariance[0] = self._xy_covariance
            msg.pose.covariance[7] = self._xy_covariance
            msg.pose.covariance[35] = self._yaw_covariance
            publisher.publish(msg)
            if publisher is self._initial_pub:
                self._debug_initial_publish_count += 1
                if self._debug_initial_publish_count <= 20:
                    self.get_logger().info(
                        "tracker_initial_publish "
                        f"count={self._debug_initial_publish_count} "
                        f"pose_stamp={pose.stamp_sec:.3f} "
                        f"header={stamp_to_float(msg.header.stamp):.3f} "
                        f"x={pose.x:.3f} y={pose.y:.3f} z={self._latest_fallback_z:.3f} "
                        f"yaw={pose.yaw:.6f}"
                    )

        def _publish_final_pose(self, pose: Pose2D) -> None:
            if self._tracker_init_stamp_sec is not None:
                age_sec = pose.stamp_sec - self._tracker_init_stamp_sec
                if age_sec < self._final_pose_warmup_sec:
                    return
            pose_to_publish = self._project_final_pose_to_route(pose)
            pose_to_publish = self._smooth_final_pose(pose_to_publish)
            self._publish_pose(
                pose_to_publish,
                self._pose_pub,
                use_pose_stamp=self._final_pose_use_internal_stamp,
            )
            self._debug_final_publish_count += 1
            if self._debug_final_publish_count <= 20:
                self.get_logger().info(
                    "tracker_final_publish "
                    f"count={self._debug_final_publish_count} "
                    f"pose_stamp={pose_to_publish.stamp_sec:.3f} "
                    f"x={pose_to_publish.x:.3f} y={pose_to_publish.y:.3f} "
                    f"yaw={pose_to_publish.yaw:.6f} has_candidate={self._has_candidate_update}"
                )

        def _project_final_pose_to_route(self, pose: Pose2D) -> Pose2D:
            gain = max(0.0, min(1.0, self._final_route_projection_gain))
            yaw_gain = max(0.0, min(1.0, self._final_route_projection_yaw_gain))
            if (
                self._route_path is None
                or self._tracker is None
                or (gain <= 0.0 and yaw_gain <= 0.0)
            ):
                return pose
            best = self._tracker.best()
            predicted_progress_m = None if self._final_route_projection_use_pose_projection else best.route_progress_m
            corrected = route_offset_pose(
                self._route_path,
                pose,
                target_cross_m=self._route_cross_correction_target(),
                gain=gain,
                yaw_gain=yaw_gain,
                gate_m=self._final_route_projection_gate_m,
                predicted_progress_m=predicted_progress_m,
                search_radius_m=self._route_search_radius_m,
            )
            return corrected if corrected is not None else pose

        def _smooth_final_pose(self, pose: Pose2D) -> Pose2D:
            alpha = max(0.0, min(1.0, self._final_pose_smoothing_alpha))
            if alpha >= 1.0 or self._smoothed_final_pose is None:
                self._smoothed_final_pose = pose
                return pose
            prev = self._smoothed_final_pose
            smoothed = Pose2D(
                stamp_sec=pose.stamp_sec,
                x=prev.x + alpha * (pose.x - prev.x),
                y=prev.y + alpha * (pose.y - prev.y),
                yaw=normalize_angle(prev.yaw + alpha * normalize_angle(pose.yaw - prev.yaw)),
            )
            self._smoothed_final_pose = smoothed
            return smoothed

        def _publish_diag(self, payload: dict[str, Any]) -> None:
            msg = String()
            msg.data = json.dumps(payload, sort_keys=True)
            self._diag_pub.publish(msg)

        def _select_initial_pose(self, tracker_pose: Pose2D) -> Pose2D:
            gnss_hint = self._gnss_weak_prior_initial_hint(tracker_pose)
            if gnss_hint is not None:
                return gnss_hint
            fallback = self._latest_fallback_initial
            if fallback is None:
                return tracker_pose
            deviation_m = math.hypot(tracker_pose.x - fallback.x, tracker_pose.y - fallback.y)
            if deviation_m > self._max_initial_deviation_from_fallback_m:
                return fallback
            return tracker_pose

        def _gnss_weak_prior_initial_hint(self, tracker_pose: Pose2D) -> Pose2D | None:
            if (
                not self._enable_gnss_weak_prior_initial_hint
                or self._latest_gnss_weak_prior_pose is None
            ):
                self._last_gnss_weak_prior_summary["gnss_weak_prior_initial_hint_active"] = False
                return None
            age_sec = abs(tracker_pose.stamp_sec - self._latest_gnss_weak_prior_pose.stamp_sec)
            if age_sec > self._gnss_weak_prior_max_age_sec:
                self._last_gnss_weak_prior_summary.update(
                    {
                        "gnss_weak_prior_initial_hint_active": False,
                        "gnss_weak_prior_initial_hint_age_sec": age_sec,
                    }
                )
                return None
            dx = self._latest_gnss_weak_prior_pose.x - tracker_pose.x
            dy = self._latest_gnss_weak_prior_pose.y - tracker_pose.y
            distance_m = math.hypot(dx, dy)
            if distance_m <= self._gnss_weak_prior_initial_hint_gate_m:
                self._last_gnss_weak_prior_summary.update(
                    {
                        "gnss_weak_prior_initial_hint_active": False,
                        "gnss_weak_prior_initial_hint_distance_m": distance_m,
                    }
                )
                return None
            max_step = self._gnss_weak_prior_initial_hint_max_step_m
            if max_step <= 0.0:
                self._last_gnss_weak_prior_summary.update(
                    {
                        "gnss_weak_prior_initial_hint_active": False,
                        "gnss_weak_prior_initial_hint_distance_m": distance_m,
                    }
                )
                return None
            step = min(max_step, distance_m)
            scale = step / max(distance_m, 1.0e-9)
            yaw_gain = self._gnss_weak_prior_initial_hint_yaw_gain
            hinted = Pose2D(
                stamp_sec=tracker_pose.stamp_sec,
                x=tracker_pose.x + dx * scale,
                y=tracker_pose.y + dy * scale,
                yaw=normalize_angle(
                    tracker_pose.yaw
                    + yaw_gain
                    * normalize_angle(self._latest_gnss_weak_prior_pose.yaw - tracker_pose.yaw)
                ),
            )
            self._last_gnss_weak_prior_summary.update(
                {
                    "gnss_weak_prior_initial_hint_active": True,
                    "gnss_weak_prior_initial_hint_age_sec": age_sec,
                    "gnss_weak_prior_initial_hint_distance_m": distance_m,
                    "gnss_weak_prior_initial_hint_step_m": step,
                    "gnss_weak_prior_initial_hint_yaw_gain": yaw_gain,
                }
            )
            return hinted

        def _maybe_update_twist_bias(
            self, best: Hypothesis, *, has_usable_candidate: bool
        ) -> None:
            if not self._enable_twist_bias_learning or not has_usable_candidate:
                self._twist_bias_last_reason = (
                    "disabled" if not self._enable_twist_bias_learning else "no_usable_candidate"
                )
                return
            if self._bias_anchor_pose is None or self._bias_anchor_stamp_sec is None:
                self._bias_anchor_pose = best.pose
                self._bias_anchor_stamp_sec = best.pose.stamp_sec
                self._bias_integrated_forward_m = 0.0
                self._twist_bias_last_reason = "anchor_initialized"
                return
            dt_sec = best.pose.stamp_sec - self._bias_anchor_stamp_sec
            result = update_twist_bias_estimate(
                current_bias_mps=self._learned_twist_bias_mps,
                anchor_pose=self._bias_anchor_pose,
                current_pose=best.pose,
                integrated_forward_m=self._bias_integrated_forward_m,
                dt_sec=dt_sec,
                alpha=self._twist_bias_learning_alpha,
                max_abs_mps=self._twist_bias_learning_max_abs_mps,
                max_step_mps=self._twist_bias_learning_max_step_mps,
                min_dt_sec=self._twist_bias_learning_min_dt_sec,
                max_dt_sec=self._twist_bias_learning_max_dt_sec,
                max_lateral_m=self._twist_bias_learning_max_lateral_m,
                max_yaw_rad=math.radians(self._twist_bias_learning_max_yaw_deg),
            )
            self._twist_bias_last_reason = result.reason
            self._twist_bias_last_estimate_mps = result.estimate_mps
            if result.reason == "dt_too_short":
                return
            if result.updated:
                self._learned_twist_bias_mps = result.bias_mps
                self._twist_bias_update_count += 1
            self._bias_anchor_pose = best.pose
            self._bias_anchor_stamp_sec = best.pose.stamp_sec
            self._bias_integrated_forward_m = 0.0

        def _initial_route_progress(
            self, pose: Pose2D, projection: RouteProjection | None = None
        ) -> float:
            if projection is None:
                projection = self._project_route(pose)
            return projection.progress_m if projection and projection.is_valid else 0.0

        def _add_startup_route_branch_hypotheses(self, pose: Pose2D) -> None:
            if (
                not self._enable_startup_route_branch_hypotheses
                or self._tracker is None
                or self._route_path is None
            ):
                return
            projections = self._route_path.project_candidates(
                pose,
                predicted_progress_m=None,
                search_radius_m=self._route_search_radius_m,
                max_distance_m=self._startup_route_branch_max_distance_m,
                max_candidates=self._startup_route_branch_max_candidates,
            )
            for projection in projections:
                if not projection.is_valid:
                    continue
                self._tracker.add_startup_hypothesis(
                    pose,
                    route_progress_m=projection.progress_m,
                    score=self._fallback_startup_score(projection),
                    min_route_progress_separation_m=(
                        self._startup_route_branch_min_progress_separation_m
                    ),
                )

        def _fallback_startup_score(self, projection: RouteProjection | None) -> float:
            if projection is None or not projection.is_valid:
                return -10.0
            return -(
                self._config.route_cross_weight * abs(
                    projection.cross_track_m - self._route_cross_target()
                )
                + self._config.route_yaw_weight * abs(projection.yaw_error_rad)
            )

        def _project_route(self, pose: Pose2D) -> RouteProjection | None:
            if self._route_path is None:
                return None
            projection = self._route_path.project(pose, predicted_progress_m=None)
            return projection

        def _route_filter_candidates(self, candidates: list[NdtCandidate]) -> list[NdtCandidate]:
            predicted_progress_m = self._tracker.best().route_progress_m if self._tracker else None
            return route_filter_candidates(
                candidates,
                self._route_path,
                cross_gate_m=self._candidate_route_cross_gate_m,
                cross_target_m=self._route_cross_target(),
                yaw_gate_rad=self._candidate_route_yaw_gate_rad,
                predicted_progress_m=predicted_progress_m,
                search_radius_m=self._candidate_route_search_radius_m,
            )

        def _apply_route_cross_correction(self) -> None:
            if (
                self._tracker is None
                or self._route_path is None
                or (
                    self._route_cross_correction_gain <= 0.0
                    and self._route_progress_correction_gain <= 0.0
                    and self._route_yaw_correction_gain <= 0.0
                )
            ):
                return
            gain = max(0.0, min(1.0, self._route_cross_correction_gain))
            progress_gain = max(0.0, min(1.0, self._route_progress_correction_gain))
            yaw_gain = max(0.0, min(1.0, self._route_yaw_correction_gain))
            if (
                self._learn_correction_target_from_ndt
                and self._route_cross_correction_target_m is None
                and not self._route_cross_target_learner.has_value
            ):
                return
            target_m = self._route_cross_correction_target()
            corrected: dict[int, Pose2D] = {}
            for hypothesis in self._tracker.hypotheses:
                projection = self._route_path.project(
                    hypothesis.pose,
                    predicted_progress_m=hypothesis.route_progress_m,
                    search_radius_m=self._route_search_radius_m,
                )
                if (
                    not projection.is_valid
                    or abs(projection.cross_track_m - target_m) > self._route_cross_correction_gate_m
                ):
                    continue
                route_yaw = projection.route_yaw_rad
                cross_error_m = projection.cross_track_m - target_m
                target_x = hypothesis.pose.x
                target_y = hypothesis.pose.y
                if progress_gain > 0.0:
                    center_x, center_y, center_yaw = self._route_path.center_at_progress(
                        hypothesis.route_progress_m
                    )
                    target_x = center_x - math.sin(center_yaw) * target_m
                    target_y = center_y + math.cos(center_yaw) * target_m
                target_yaw = hypothesis.pose.yaw
                if yaw_gain > 0.0 and abs(projection.yaw_error_rad) <= self._route_yaw_correction_gate_rad:
                    target_yaw = normalize_angle(
                        hypothesis.pose.yaw - yaw_gain * projection.yaw_error_rad
                    )
                corrected[hypothesis.id] = Pose2D(
                    stamp_sec=hypothesis.pose.stamp_sec,
                    x=(
                        hypothesis.pose.x
                        + gain * cross_error_m * math.sin(route_yaw)
                        + progress_gain * (target_x - hypothesis.pose.x)
                    ),
                    y=(
                        hypothesis.pose.y
                        - gain * cross_error_m * math.cos(route_yaw)
                        + progress_gain * (target_y - hypothesis.pose.y)
                    ),
                    yaw=target_yaw,
                )
            self._tracker.apply_pose_correction(corrected)

        def _route_priors_for_candidates(
            self, candidates: list[NdtCandidate]
        ) -> dict[tuple[int, int], RoutePrior]:
            if self._tracker is None or self._route_path is None:
                return {}
            valid_candidates = [
                candidate for candidate in candidates if self._tracker._candidate_is_usable(candidate)
            ]
            route_priors: dict[tuple[int, int], RoutePrior] = {}
            for hypothesis in self._tracker.hypotheses:
                for candidate_index, candidate in enumerate(valid_candidates):
                    projection = self._route_path.project(
                        candidate.pose,
                        predicted_progress_m=hypothesis.route_progress_m,
                        search_radius_m=self._route_search_radius_m,
                    )
                    route_priors[(hypothesis.id, candidate_index)] = RoutePrior(
                        progress_m=projection.progress_m,
                        cross_track_m=projection.cross_track_m - self._route_cross_target(),
                        yaw_error_rad=projection.yaw_error_rad,
                        is_valid=projection.is_valid,
                    )
            return route_priors

        def _route_cross_target(self) -> float:
            return self._route_cross_target_m if self._route_cross_target_m is not None else 0.0

        def _route_cross_correction_target(self) -> float:
            if self._route_cross_correction_target_m is not None:
                return self._route_cross_correction_target_m
            return self._route_cross_target()

        def _update_route_cross_target_from_pose(
            self, pose: Pose2D, *, along_degenerate: bool
        ) -> None:
            if self._route_path is None:
                return
            predicted_progress_m = self._tracker.best().route_progress_m if self._tracker else None
            projection = self._route_path.project(
                pose,
                predicted_progress_m=predicted_progress_m,
                search_radius_m=self._route_search_radius_m,
            )
            if not projection.is_valid:
                return
            value = self._route_cross_target_learner.update(
                projection.cross_track_m,
                along_degenerate=along_degenerate,
            )
            if value is not None:
                self._route_cross_correction_target_m = value
                self._learned_correction_target_count = self._route_cross_target_learner.count

        def _scan_submap_residuals_for_candidates(
            self, candidates: list[NdtCandidate]
        ) -> dict[tuple[int, int], RelativeResidual]:
            if (
                not self._enable_scan_submap_residual
                or self._tracker is None
                or not self._latest_scan_points
                or self._latest_scan_stamp_sec is None
            ):
                return {}
            residuals: dict[tuple[int, int], RelativeResidual] = {}
            for hypothesis in self._tracker.hypotheses:
                submap = self._submap_for_hypothesis(hypothesis.id)
                if submap is None or submap.cell_count <= 0:
                    continue
                def residual_for_pose(pose: Pose2D) -> RelativeResidual:
                    residual = submap.residual(
                        self._latest_scan_points,
                        pose,
                        max_points=self._scan_submap_max_points,
                    )
                    if self._scan_submap_residual_mode in ("profile", "longitudinal", "combined"):
                        profile_residual = submap.longitudinal_profile_residual(
                            self._latest_scan_points,
                            pose,
                            max_points=self._scan_submap_max_points,
                            max_profile_cells=self._scan_submap_profile_max_cells,
                            lateral_bin_m=self._scan_submap_profile_lateral_bin_m,
                        )
                        if self._scan_submap_residual_mode in ("profile", "longitudinal"):
                            residual = profile_residual
                        elif residual.is_valid and profile_residual.is_valid:
                            residual = RelativeResidual(
                                xy_m=0.5 * residual.xy_m + 0.5 * profile_residual.xy_m,
                                yaw_rad=residual.yaw_rad,
                                quality=min(residual.quality, profile_residual.quality),
                                is_valid=True,
                            )
                    if self._scan_submap_residual_mode in ("icp", "gicp", "combined_icp"):
                        icp_residual = submap.local_icp_residual(
                            self._latest_scan_points,
                            pose,
                            max_points=self._scan_submap_max_points,
                            max_map_points=self._scan_submap_icp_max_map_points,
                            max_match_distance_m=self._scan_submap_icp_max_match_distance_m,
                            correction_penalty_weight=(
                                self._scan_submap_icp_correction_penalty_weight
                            ),
                        )
                        if self._scan_submap_residual_mode in ("icp", "gicp"):
                            residual = icp_residual
                        elif residual.is_valid and icp_residual.is_valid:
                            residual = RelativeResidual(
                                xy_m=0.5 * residual.xy_m + 0.5 * icp_residual.xy_m,
                                yaw_rad=max(residual.yaw_rad, icp_residual.yaw_rad),
                                quality=min(residual.quality, icp_residual.quality),
                                is_valid=True,
                            )
                    return residual

                baseline = residual_for_pose(hypothesis.pose)
                if baseline.is_valid and baseline.quality >= self._scan_submap_min_quality:
                    residuals[(hypothesis.id, -1)] = baseline
                for candidate_index, candidate in enumerate(candidates):
                    age_sec = abs(candidate.pose.stamp_sec - self._latest_scan_stamp_sec)
                    if age_sec > self._scan_submap_max_age_sec:
                        continue
                    residual = residual_for_pose(candidate.pose)
                    if residual.is_valid and residual.quality >= self._scan_submap_min_quality:
                        residuals[(hypothesis.id, candidate_index)] = residual
            return residuals

        def _scan_submap_icp_candidates(self) -> list[NdtCandidate]:
            if (
                not self._enable_scan_submap_residual
                or not self._enable_scan_submap_icp_candidates
                or self._tracker is None
                or not self._latest_scan_points
                or self._latest_scan_stamp_sec is None
            ):
                return []
            candidates: list[NdtCandidate] = []
            seen: set[tuple[int, int, int]] = set()
            max_yaw_rad = math.radians(self._scan_submap_icp_candidate_max_yaw_deg)
            for hypothesis in self._tracker.hypotheses:
                submap = self._submap_for_hypothesis(hypothesis.id)
                if submap is None or submap.cell_count <= 0:
                    continue
                if abs(hypothesis.pose.stamp_sec - self._latest_scan_stamp_sec) > self._scan_submap_max_age_sec:
                    continue
                result = submap.local_icp_pose_candidate(
                    self._latest_scan_points,
                    hypothesis.pose,
                    max_points=self._scan_submap_max_points,
                    max_map_points=self._scan_submap_icp_max_map_points,
                    max_match_distance_m=self._scan_submap_icp_max_match_distance_m,
                    max_correction_m=self._scan_submap_icp_candidate_max_correction_m,
                    max_yaw_correction_rad=max_yaw_rad,
                    min_quality=self._scan_submap_icp_candidate_min_quality,
                    correction_penalty_weight=self._scan_submap_icp_correction_penalty_weight,
                    match_in_3d=self._scan_submap_icp_candidate_match_in_3d,
                )
                if result is None:
                    continue
                pose, residual = result
                key = (
                    int(round(pose.x * 20.0)),
                    int(round(pose.y * 20.0)),
                    int(round(pose.yaw * 1000.0)),
                )
                if key in seen:
                    continue
                seen.add(key)
                candidates.append(
                    NdtCandidate(
                        pose=pose,
                        score=self._scan_submap_icp_candidate_score - residual.xy_m,
                        converged=True,
                        initial_to_result_m=0.0,
                        initial_to_result_yaw_rad=residual.yaw_rad,
                        rejection_reason="",
                        source="scan_submap_icp",
                    )
                )
            return candidates

        def _scan_submap_local_ndt_candidates(self) -> list[NdtCandidate]:
            if (
                not self._enable_scan_submap_residual
                or not self._enable_scan_submap_local_ndt_candidates
                or self._tracker is None
                or not self._latest_scan_points
                or self._latest_scan_stamp_sec is None
            ):
                return []
            candidates: list[NdtCandidate] = []
            seen: set[tuple[int, int, int]] = set()
            for hypothesis in self._tracker.hypotheses:
                submap = self._submap_for_hypothesis(hypothesis.id)
                if submap is None or submap.cell_count <= 0:
                    continue
                if abs(hypothesis.pose.stamp_sec - self._latest_scan_stamp_sec) > self._scan_submap_max_age_sec:
                    continue
                results = submap.local_ndt_pose_candidates(
                    self._latest_scan_points,
                    hypothesis.pose,
                    forward_offsets_m=self._scan_submap_local_ndt_forward_offsets,
                    lateral_offsets_m=self._scan_submap_local_ndt_lateral_offsets,
                    yaw_offsets_rad=self._scan_submap_local_ndt_yaw_offsets,
                    max_points=self._scan_submap_max_points,
                    max_map_points=self._scan_submap_icp_max_map_points,
                    min_quality=self._scan_submap_local_ndt_candidate_min_quality,
                    min_cell_points=self._scan_submap_local_ndt_min_cell_points,
                    max_candidates=self._scan_submap_local_ndt_max_candidates,
                    reject_boundary_best=self._scan_submap_local_ndt_reject_boundary_best,
                    min_second_best_score_margin=(
                        self._scan_submap_local_ndt_min_second_best_score_margin
                    ),
                    profile_score_weight=self._scan_submap_local_ndt_profile_score_weight,
                )
                for pose, residual in results:
                    key = (
                        int(round(pose.x * 20.0)),
                        int(round(pose.y * 20.0)),
                        int(round(pose.yaw * 1000.0)),
                    )
                    if key in seen:
                        continue
                    seen.add(key)
                    candidates.append(
                        NdtCandidate(
                            pose=pose,
                            score=self._scan_submap_local_ndt_candidate_score
                            - residual.xy_m
                            + min(1.0, residual.quality),
                            converged=True,
                            initial_to_result_m=0.0,
                            initial_to_result_yaw_rad=residual.yaw_rad,
                            rejection_reason="",
                            source="scan_submap_local_ndt",
                        )
                    )
            return candidates

        def _global_map_local_ndt_candidates(self) -> list[NdtCandidate]:
            if (
                not self._enable_scan_submap_residual
                or not self._enable_global_map_local_ndt_candidates
                or self._global_map_submap is None
                or self._tracker is None
                or not self._latest_scan_points
                or self._latest_scan_stamp_sec is None
            ):
                return []
            candidates: list[NdtCandidate] = []
            seen: set[tuple[int, int, int]] = set()
            for hypothesis in self._tracker.hypotheses:
                if abs(hypothesis.pose.stamp_sec - self._latest_scan_stamp_sec) > self._scan_submap_max_age_sec:
                    continue
                seed_poses = [hypothesis.pose]
                if self._global_map_local_ndt_use_route_seed and self._route_path is not None:
                    route_seed = route_offset_pose(
                        self._route_path,
                        hypothesis.pose,
                        target_cross_m=self._route_cross_correction_target(),
                        gain=1.0,
                        yaw_gain=self._global_map_local_ndt_route_seed_yaw_gain,
                        gate_m=self._final_route_projection_gate_m,
                        predicted_progress_m=hypothesis.route_progress_m,
                        search_radius_m=self._route_search_radius_m,
                    )
                    if route_seed is not None:
                        seed_poses.append(route_seed)
                gnss_seed = self._global_map_local_ndt_gnss_seed_pose(hypothesis.pose)
                if gnss_seed is not None:
                    seed_poses.append(gnss_seed)
                for seed_pose in seed_poses:
                    results = self._global_map_submap.local_ndt_pose_candidates(
                        self._latest_scan_points,
                        seed_pose,
                        forward_offsets_m=self._scan_submap_local_ndt_forward_offsets,
                        lateral_offsets_m=self._scan_submap_local_ndt_lateral_offsets,
                        yaw_offsets_rad=self._scan_submap_local_ndt_yaw_offsets,
                        max_points=self._scan_submap_max_points,
                        max_map_points=self._scan_submap_icp_max_map_points,
                        min_quality=self._scan_submap_local_ndt_candidate_min_quality,
                        min_cell_points=self._scan_submap_local_ndt_min_cell_points,
                        max_candidates=self._scan_submap_local_ndt_max_candidates,
                        reject_boundary_best=self._scan_submap_local_ndt_reject_boundary_best,
                        min_second_best_score_margin=(
                            self._scan_submap_local_ndt_min_second_best_score_margin
                        ),
                        profile_score_weight=self._scan_submap_local_ndt_profile_score_weight,
                    )
                    for pose, residual in results:
                        key = (
                            int(round(pose.x * 20.0)),
                            int(round(pose.y * 20.0)),
                            int(round(pose.yaw * 1000.0)),
                        )
                        if key in seen:
                            continue
                        seen.add(key)
                        candidates.append(
                            NdtCandidate(
                                pose=pose,
                                score=self._scan_submap_local_ndt_candidate_score
                                - residual.xy_m
                                + min(1.0, residual.quality),
                                converged=True,
                                initial_to_result_m=0.0,
                                initial_to_result_yaw_rad=residual.yaw_rad,
                                rejection_reason="",
                                source="global_map_local_ndt",
                            )
                        )
            return candidates

        def _global_map_local_ndt_gnss_seed_pose(
            self, tracker_pose: Pose2D
        ) -> Pose2D | None:
            if (
                not self._global_map_local_ndt_use_gnss_weak_prior_seed
                or self._latest_gnss_weak_prior_pose is None
                or self._latest_scan_stamp_sec is None
            ):
                return None
            age_sec = abs(
                self._latest_scan_stamp_sec - self._latest_gnss_weak_prior_pose.stamp_sec
            )
            if age_sec > self._global_map_local_ndt_gnss_seed_max_age_sec:
                return None
            distance_m = math.hypot(
                tracker_pose.x - self._latest_gnss_weak_prior_pose.x,
                tracker_pose.y - self._latest_gnss_weak_prior_pose.y,
            )
            if distance_m > self._global_map_local_ndt_gnss_seed_xy_gate_m:
                return None
            return Pose2D(
                stamp_sec=tracker_pose.stamp_sec,
                x=self._latest_gnss_weak_prior_pose.x,
                y=self._latest_gnss_weak_prior_pose.y,
                yaw=tracker_pose.yaw,
            )

        def _anchor_submap_is_active(self) -> bool:
            return (
                self._anchor_submap is not None
                and scan_submap_anchor_is_valid(
                    enabled=self._enable_scan_submap_anchor,
                    anchor_stamp_sec=self._anchor_submap_stamp_sec,
                    current_stamp_sec=self._latest_scan_stamp_sec,
                    max_age_sec=self._scan_submap_anchor_max_age_sec,
                )
            )

        def _anchor_submap_age_sec(self) -> float | None:
            if self._anchor_submap_stamp_sec is None or self._latest_scan_stamp_sec is None:
                return None
            return max(0.0, self._latest_scan_stamp_sec - self._anchor_submap_stamp_sec)

        def _submap_for_hypothesis(self, hypothesis_id: int) -> LightweightScanSubmap | None:
            if self._anchor_submap_is_active() and not self._last_runtime_update_had_converged_ndt:
                return self._anchor_submap
            return self._hypothesis_submaps.get(hypothesis_id)

        def _maybe_refresh_anchor_submap(self) -> None:
            if (
                not self._enable_scan_submap_anchor
                or self._tracker is None
                or not self._last_runtime_update_had_converged_ndt
                or self._usable_ndt_streak < self._scan_submap_anchor_min_usable_streak
                or self._latest_scan_stamp_sec is None
            ):
                return
            best = self._tracker.best()
            best_submap = self._hypothesis_submaps.get(best.id)
            if best_submap is None or best_submap.cell_count <= 0:
                return
            self._anchor_submap = best_submap.clone()
            self._anchor_submap_stamp_sec = self._latest_scan_stamp_sec
            self._anchor_submap_source_hypothesis_id = best.id

        def _maybe_add_current_scan_to_hypothesis_submaps(self) -> None:
            if (
                not self._enable_scan_submap_residual
                or self._tracker is None
                or not self._latest_scan_points
                or self._latest_scan_stamp_sec is None
            ):
                return
            if self._scan_submap_add_only_with_usable_ndt and not self._last_runtime_update_had_usable_ndt:
                return
            self._sync_hypothesis_submaps()
            for hypothesis in self._tracker.hypotheses:
                if abs(hypothesis.pose.stamp_sec - self._latest_scan_stamp_sec) > self._scan_submap_max_age_sec:
                    continue
                if self._last_submap_added_by_hypothesis.get(hypothesis.id) == self._latest_scan_stamp_sec:
                    continue
                submap = self._hypothesis_submaps.get(hypothesis.id)
                if submap is None:
                    continue
                submap.add_scan(self._latest_scan_points, hypothesis.pose)
                self._last_submap_added_by_hypothesis[hypothesis.id] = self._latest_scan_stamp_sec
            self._maybe_refresh_anchor_submap()

        def _apply_scan_submap_correction(self) -> None:
            if (
                not self._enable_scan_submap_residual
                or not self._enable_scan_submap_correction
                or self._tracker is None
                or not self._latest_scan_points
                or self._latest_scan_stamp_sec is None
            ):
                return
            corrected: dict[int, Pose2D] = {}
            for hypothesis in self._tracker.hypotheses:
                submap = self._submap_for_hypothesis(hypothesis.id)
                if submap is None or submap.cell_count <= 0:
                    continue
                if abs(hypothesis.pose.stamp_sec - self._latest_scan_stamp_sec) > self._scan_submap_max_age_sec:
                    continue
                if (
                    self._last_scan_corrected_by_hypothesis.get(hypothesis.id)
                    == self._latest_scan_stamp_sec
                ):
                    continue
                if self._enable_scan_submap_profile_along_correction:
                    result = submap.profile_along_correction(
                        self._latest_scan_points,
                        hypothesis.pose,
                        forward_offsets_m=self._scan_submap_correction_forward_offsets,
                        max_points=self._scan_submap_max_points,
                        min_quality=self._scan_submap_correction_min_quality,
                        min_second_best_margin_m=self._scan_submap_profile_along_min_margin_m,
                    )
                    if result is None:
                        continue
                    pose, residual = result
                    corrected[hypothesis.id] = pose
                    self._last_scan_corrected_by_hypothesis[hypothesis.id] = (
                        self._latest_scan_stamp_sec
                    )
                    continue
                seed_residual = submap.residual(
                    self._latest_scan_points,
                    hypothesis.pose,
                    max_points=self._scan_submap_max_points,
                )
                pose, residual = submap.refine_pose(
                    self._latest_scan_points,
                    hypothesis.pose,
                    forward_offsets_m=self._scan_submap_correction_forward_offsets,
                    lateral_offsets_m=self._scan_submap_correction_lateral_offsets,
                    yaw_offsets_rad=self._scan_submap_correction_yaw_offsets,
                    max_points=self._scan_submap_max_points,
                    profile_score_weight=self._scan_submap_correction_profile_score_weight,
                )
                if not residual.is_valid or residual.quality < self._scan_submap_correction_min_quality:
                    continue
                improvement_m = seed_residual.xy_m - residual.xy_m
                if improvement_m < self._scan_submap_correction_min_improvement_m:
                    continue
                corrected[hypothesis.id] = pose
                self._last_scan_corrected_by_hypothesis[hypothesis.id] = self._latest_scan_stamp_sec
            self._tracker.apply_pose_correction(corrected)

        def _sync_hypothesis_submaps(self) -> None:
            if self._tracker is None:
                self._hypothesis_submaps.clear()
                self._last_submap_added_by_hypothesis.clear()
                self._last_scan_corrected_by_hypothesis.clear()
                self._anchor_submap = None
                self._anchor_submap_stamp_sec = None
                self._anchor_submap_source_hypothesis_id = None
                self._usable_ndt_streak = 0
                self._last_runtime_update_had_converged_ndt = False
                return
            previous = self._hypothesis_submaps
            synced: dict[int, LightweightScanSubmap] = {}
            synced_stamps: dict[int, float] = {}
            synced_correction_stamps: dict[int, float] = {}
            live_ids = {hypothesis.id for hypothesis in self._tracker.hypotheses}
            for hypothesis in self._tracker.hypotheses:
                if hypothesis.id in previous:
                    synced[hypothesis.id] = previous[hypothesis.id]
                    if hypothesis.id in self._last_submap_added_by_hypothesis:
                        synced_stamps[hypothesis.id] = self._last_submap_added_by_hypothesis[hypothesis.id]
                    if hypothesis.id in self._last_scan_corrected_by_hypothesis:
                        synced_correction_stamps[hypothesis.id] = (
                            self._last_scan_corrected_by_hypothesis[hypothesis.id]
                        )
                    continue
                parent_map = (
                    previous.get(hypothesis.parent_id)
                    if hypothesis.parent_id is not None
                    else None
                )
                if parent_map is not None:
                    synced[hypothesis.id] = parent_map.clone()
                    parent_stamp = self._last_submap_added_by_hypothesis.get(hypothesis.parent_id)
                    if parent_stamp is not None:
                        synced_stamps[hypothesis.id] = parent_stamp
                    parent_correction_stamp = self._last_scan_corrected_by_hypothesis.get(
                        hypothesis.parent_id
                    )
                    if parent_correction_stamp is not None:
                        synced_correction_stamps[hypothesis.id] = parent_correction_stamp
                else:
                    synced[hypothesis.id] = self._new_scan_submap()
            self._hypothesis_submaps = synced
            self._last_submap_added_by_hypothesis = {
                hyp_id: stamp for hyp_id, stamp in synced_stamps.items() if hyp_id in live_ids
            }
            self._last_scan_corrected_by_hypothesis = {
                hyp_id: stamp
                for hyp_id, stamp in synced_correction_stamps.items()
                if hyp_id in live_ids
            }

        def _new_scan_submap(self) -> LightweightScanSubmap:
            return LightweightScanSubmap(
                voxel_size_m=self._scan_submap_voxel_size_m,
                max_cells=self._scan_submap_max_cells,
                neighbor_radius_cells=self._scan_submap_neighbor_radius_cells,
                unmatched_penalty_m=self._scan_submap_unmatched_penalty_m,
                enable_3d_points=self._scan_submap_icp_candidate_match_in_3d,
            )

        def _best_submap_cell_count(self) -> int:
            if self._tracker is None:
                return 0
            submap = self._hypothesis_submaps.get(self._tracker.best().id)
            return submap.cell_count if submap is not None else 0

        @staticmethod
        def _parse_float_tuple(value: str) -> tuple[float, ...]:
            parsed = tuple(
                float(part.strip())
                for part in value.split(",")
                if part.strip()
            )
            return parsed if parsed else (0.0,)

        @staticmethod
        def _pose_from_msg(msg: PoseWithCovarianceStamped) -> Pose2D:
            orientation = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
            )
            return Pose2D(
                stamp_sec=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
                x=float(msg.pose.pose.position.x),
                y=float(msg.pose.pose.position.y),
                yaw=yaw,
            )

    def stamp_to_float(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    rclpy.init()
    node = PureLidarFixedLagTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
