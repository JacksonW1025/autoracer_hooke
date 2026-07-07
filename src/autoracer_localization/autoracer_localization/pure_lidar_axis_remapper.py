import copy
import json
import math
import statistics
from collections import deque

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from .ndt_initial_pose_predictor import _rpy_from_quaternion, _rpy_to_quaternion, _yaw_from_quaternion
from .pure_lidar_fixed_lag_tracker import (
    LightweightScanSubmap,
    MotionDelta,
    Point2D,
    Pose2D,
    RoutePath,
    normalize_angle,
    planar_delta,
    payload_candidate_localizability_summary,
    payload_indicates_along_degeneracy,
    route_offset_pose,
    route_remap_candidate_along_to_prediction,
)


def pose2d_from_msg(msg: PoseWithCovarianceStamped) -> Pose2D:
    pose = msg.pose.pose
    return Pose2D(
        stamp_sec=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=_yaw_from_quaternion(pose.orientation),
    )


def apply_pose2d_to_msg(
    source: PoseWithCovarianceStamped,
    pose2d: Pose2D,
) -> PoseWithCovarianceStamped:
    out = copy.deepcopy(source)
    out.pose.pose.position.x = pose2d.x
    out.pose.pose.position.y = pose2d.y
    roll, pitch, _ = _rpy_from_quaternion(out.pose.pose.orientation)
    out.pose.pose.orientation = _rpy_to_quaternion(roll, pitch, pose2d.yaw)
    return out


def propagate_pose(pose: Pose2D, motion: MotionDelta) -> Pose2D:
    cy = math.cos(pose.yaw)
    sy = math.sin(pose.yaw)
    return Pose2D(
        stamp_sec=pose.stamp_sec + motion.dt_sec,
        x=pose.x + cy * motion.forward_m - sy * motion.lateral_m,
        y=pose.y + sy * motion.forward_m + cy * motion.lateral_m,
        yaw=pose.yaw + motion.yaw_rad,
    )


def planar_covariance_variance(covariance: list[float] | tuple[float, ...], yaw_rad: float) -> float:
    if len(covariance) < 8:
        return math.inf
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    xx = float(covariance[0])
    xy = float(covariance[1])
    yx = float(covariance[6])
    yy = float(covariance[7])
    return c * c * xx + c * s * (xy + yx) + s * s * yy


def reject_ndt_pose_for_consistency(
    *,
    predicted_pose: Pose2D,
    ndt_pose: Pose2D,
    max_xy_innovation_m: float,
    max_yaw_innovation_rad: float,
) -> tuple[bool, dict[str, float | str]]:
    forward_m, lateral_m, yaw_rad = planar_delta(predicted_pose, ndt_pose)
    xy_m = math.hypot(forward_m, lateral_m)
    yaw_abs_rad = abs(normalize_angle(yaw_rad))
    diag: dict[str, float | str] = {
        "consistency_forward_innovation_m": forward_m,
        "consistency_lateral_innovation_m": lateral_m,
        "consistency_xy_innovation_m": xy_m,
        "consistency_yaw_innovation_deg": math.degrees(yaw_abs_rad),
        "consistency_reject_reason": "",
    }
    if math.isfinite(max_yaw_innovation_rad) and yaw_abs_rad > max_yaw_innovation_rad:
        diag["consistency_reject_reason"] = "yaw_innovation"
        return True, diag
    if math.isfinite(max_xy_innovation_m) and xy_m > max_xy_innovation_m:
        diag["consistency_reject_reason"] = "xy_innovation"
        return True, diag
    return False, diag


def scan_submap_yaw_corrected_pose(
    *,
    submap: LightweightScanSubmap,
    scan_points: list[Point2D],
    predicted_pose: Pose2D,
    yaw_offsets_rad: tuple[float, ...],
    max_points: int,
    min_quality: float,
    max_residual_m: float,
    min_improvement_m: float,
) -> tuple[Pose2D, dict[str, float | int]] | None:
    """Conservatively correct yaw using only a causal rolling scan submap."""

    if not scan_points or submap.cell_count <= 0:
        return None
    offsets = tuple(float(offset) for offset in yaw_offsets_rad)
    if not offsets:
        return None

    def score(pose: Pose2D) -> tuple[float, float] | None:
        residual = submap.residual(scan_points, pose, max_points=max_points)
        if not residual.is_valid:
            return None
        return residual.xy_m, residual.quality

    base_score = score(predicted_pose)
    if base_score is None:
        return None
    candidates: list[tuple[float, Pose2D, float, float]] = []
    for offset in offsets:
        pose = Pose2D(
            stamp_sec=predicted_pose.stamp_sec,
            x=predicted_pose.x,
            y=predicted_pose.y,
            yaw=normalize_angle(predicted_pose.yaw + offset),
        )
        candidate_score = score(pose)
        if candidate_score is None:
            continue
        residual_m, quality = candidate_score
        if quality < min_quality or residual_m > max_residual_m:
            continue
        candidates.append((residual_m, pose, offset, quality))
    if not candidates:
        return None
    best_residual, best_pose, best_offset, best_quality = min(candidates, key=lambda item: item[0])
    if best_residual > base_score[0] - max(0.0, min_improvement_m):
        return None
    return best_pose, {
        "scan_submap_yaw_candidate_count": len(candidates),
        "scan_submap_yaw_base_residual_m": base_score[0],
        "scan_submap_yaw_best_residual_m": best_residual,
        "scan_submap_yaw_best_offset_deg": math.degrees(best_offset),
        "scan_submap_yaw_best_quality": best_quality,
    }


def scan_submap_along_corrected_pose(
    *,
    submap: LightweightScanSubmap,
    scan_points: list[Point2D],
    predicted_pose: Pose2D,
    forward_offsets_m: tuple[float, ...],
    max_points: int,
    max_profile_cells: int,
    lateral_bin_m: float,
    min_quality: float,
    max_residual_m: float,
    min_improvement_m: float,
    reject_boundary_best: bool = False,
    min_second_best_margin_m: float = 0.0,
) -> tuple[Pose2D, dict[str, float | int]] | None:
    """Conservatively correct along/progress using a causal scan profile."""

    if not scan_points or submap.cell_count <= 0:
        return None
    offsets = tuple(float(offset) for offset in forward_offsets_m)
    if not offsets:
        return None

    def pose_at(offset: float) -> Pose2D:
        return Pose2D(
            stamp_sec=predicted_pose.stamp_sec,
            x=predicted_pose.x + math.cos(predicted_pose.yaw) * offset,
            y=predicted_pose.y + math.sin(predicted_pose.yaw) * offset,
            yaw=predicted_pose.yaw,
        )

    def score(pose: Pose2D) -> tuple[float, float] | None:
        residual = submap.longitudinal_profile_residual(
            scan_points,
            pose,
            max_points=max_points,
            max_profile_cells=max_profile_cells,
            lateral_bin_m=lateral_bin_m,
        )
        if not residual.is_valid:
            return None
        return residual.xy_m, residual.quality

    base_score = score(predicted_pose)
    if base_score is None:
        return None
    candidates: list[tuple[float, Pose2D, float, float]] = []
    for offset in offsets:
        pose = pose_at(offset)
        candidate_score = score(pose)
        if candidate_score is None:
            continue
        residual_m, quality = candidate_score
        if quality < min_quality or residual_m > max_residual_m:
            continue
        candidates.append((residual_m, pose, offset, quality))
    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    best_residual, best_pose, best_offset, best_quality = candidates[0]
    unique_offsets = sorted(set(offsets))
    if reject_boundary_best and len(unique_offsets) > 1:
        if best_offset <= unique_offsets[0] or best_offset >= unique_offsets[-1]:
            return None
    second_best_residual = candidates[1][0] if len(candidates) > 1 else math.inf
    if min_second_best_margin_m > 0.0:
        if not math.isfinite(second_best_residual):
            return None
        if second_best_residual - best_residual < min_second_best_margin_m:
            return None
    if best_residual > base_score[0] - max(0.0, min_improvement_m):
        return None
    return best_pose, {
        "scan_submap_along_candidate_count": len(candidates),
        "scan_submap_along_base_residual_m": base_score[0],
        "scan_submap_along_best_residual_m": best_residual,
        "scan_submap_along_second_best_residual_m": second_best_residual,
        "scan_submap_along_best_offset_m": best_offset,
        "scan_submap_along_best_quality": best_quality,
    }


def route_cross_held_pose(
    *,
    route_path: RoutePath,
    pose: Pose2D,
    target_cross_m: float,
    gain: float,
    yaw_gain: float,
    gate_m: float,
    predicted_progress_m: float | None,
    search_radius_m: float,
) -> tuple[Pose2D | None, dict[str, float | bool]]:
    projection = route_path.project(
        pose,
        predicted_progress_m=predicted_progress_m,
        search_radius_m=search_radius_m,
    )
    diag: dict[str, float | bool] = {
        "route_cross_hold_applied": False,
        "route_cross_before_m": projection.cross_track_m,
        "route_cross_target_m": target_cross_m,
        "route_cross_progress_m": projection.progress_m,
    }
    corrected = route_offset_pose(
        route_path,
        pose,
        target_cross_m=target_cross_m,
        gain=gain,
        yaw_gain=yaw_gain,
        gate_m=gate_m,
        predicted_progress_m=predicted_progress_m,
        search_radius_m=search_radius_m,
    )
    if corrected is None:
        return None, diag
    corrected_projection = route_path.project(
        corrected,
        predicted_progress_m=projection.progress_m if projection.is_valid else predicted_progress_m,
        search_radius_m=search_radius_m,
    )
    diag.update(
        {
            "route_cross_hold_applied": True,
            "route_cross_after_m": corrected_projection.cross_track_m,
            "route_cross_progress_m": corrected_projection.progress_m,
        }
    )
    return corrected, diag


def route_progress_held_pose(
    *,
    route_path: RoutePath,
    pose: Pose2D,
    progress_m: float,
    target_cross_m: float,
    gain: float,
    yaw_gain: float,
    gate_m: float,
) -> tuple[Pose2D | None, dict[str, float | bool]]:
    """Hold pose on a causal route progress instead of nearest route branch."""

    center_x, center_y, center_yaw = route_path.center_at_progress(progress_m)
    target_x = center_x - math.sin(center_yaw) * target_cross_m
    target_y = center_y + math.cos(center_yaw) * target_cross_m
    distance = math.hypot(target_x - pose.x, target_y - pose.y)
    diag: dict[str, float | bool] = {
        "route_progress_hold_applied": False,
        "route_progress_hold_progress_m": progress_m,
        "route_progress_hold_distance_m": distance,
        "route_progress_hold_target_cross_m": target_cross_m,
    }
    if distance > max(0.0, gate_m):
        return None, diag
    route_gain = max(0.0, min(1.0, gain))
    route_yaw_gain = max(0.0, min(1.0, yaw_gain))
    corrected = Pose2D(
        stamp_sec=pose.stamp_sec,
        x=pose.x + route_gain * (target_x - pose.x),
        y=pose.y + route_gain * (target_y - pose.y),
        yaw=normalize_angle(
            pose.yaw + route_yaw_gain * normalize_angle(center_yaw - pose.yaw)
        ),
    )
    diag["route_progress_hold_applied"] = True
    return corrected, diag


def should_apply_route_hold(
    *,
    pose_stamp_sec: float,
    last_ndt_update_stamp_sec: float | None,
    degenerate_until_sec: float,
    only_when_stale_or_degenerate: bool,
    stale_sec: float,
) -> bool:
    """Gate route prior so it only helps in degenerate/no-NDT gaps when requested."""

    if not only_when_stale_or_degenerate:
        return True
    if pose_stamp_sec <= degenerate_until_sec:
        return True
    if last_ndt_update_stamp_sec is None:
        return True
    return (pose_stamp_sec - last_ndt_update_stamp_sec) >= max(0.0, stale_sec)


def update_route_cross_target(
    *,
    current_target_m: float,
    observed_cross_m: float,
    alpha: float,
    max_step_m: float,
    max_abs_m: float,
) -> float:
    if not math.isfinite(observed_cross_m):
        return current_target_m
    if not math.isfinite(current_target_m):
        current_target_m = observed_cross_m
    blend = max(0.0, min(1.0, alpha))
    step_limit = max(0.0, max_step_m)
    delta = (observed_cross_m - current_target_m) * blend
    delta = max(-step_limit, min(step_limit, delta))
    limit = max(0.0, max_abs_m)
    return max(-limit, min(limit, current_target_m + delta))


def stable_route_cross_target_candidate(
    *,
    observations: tuple[float, ...],
    min_count: int,
    max_range_m: float,
) -> float | None:
    finite = [value for value in observations if math.isfinite(value)]
    required = max(1, min_count)
    if len(finite) < required:
        return None
    recent = finite[-required:]
    if max(recent) - min(recent) > max(0.0, max_range_m):
        return None
    return float(statistics.median(recent))


def route_progress_innovation_m(
    *,
    route_path: RoutePath,
    predicted_progress_m: float,
    ndt_pose: Pose2D,
    search_radius_m: float,
) -> float | None:
    if not math.isfinite(predicted_progress_m):
        return None
    projection = route_path.project(
        ndt_pose,
        predicted_progress_m=predicted_progress_m,
        search_radius_m=search_radius_m,
    )
    if not projection.is_valid:
        return None
    return projection.progress_m - predicted_progress_m


def update_route_progress_anchor(
    *,
    current_progress_m: float,
    observed_progress_m: float,
    gate_m: float,
    gain: float,
    max_step_m: float,
) -> tuple[float, bool]:
    """Blend integrated route progress toward a trusted NDT route observation."""

    if not math.isfinite(current_progress_m) or not math.isfinite(observed_progress_m):
        return current_progress_m, False
    innovation = observed_progress_m - current_progress_m
    if abs(innovation) > max(0.0, gate_m):
        return current_progress_m, False
    bounded_gain = max(0.0, min(1.0, gain))
    step_limit = max(0.0, max_step_m)
    step = innovation * bounded_gain
    step = max(-step_limit, min(step_limit, step))
    return current_progress_m + step, True


def update_twist_bias_from_progress_innovation(
    *,
    current_bias_mps: float,
    progress_innovation_m: float,
    dt_sec: float,
    alpha: float,
    max_step_mps: float,
    max_abs_mps: float,
    max_progress_innovation_m: float,
    min_bias_mps: float | None = None,
    max_bias_mps: float | None = None,
) -> tuple[float, float | None, bool]:
    if (
        not math.isfinite(current_bias_mps)
        or not math.isfinite(progress_innovation_m)
        or not math.isfinite(dt_sec)
        or dt_sec <= 0.0
    ):
        return current_bias_mps, None, False
    if abs(progress_innovation_m) > max(0.0, max_progress_innovation_m):
        return current_bias_mps, None, False
    desired_delta_mps = progress_innovation_m / dt_sec
    step_limit = max(0.0, max_step_mps)
    desired_delta_mps = max(-step_limit, min(step_limit, desired_delta_mps))
    update = max(0.0, min(1.0, alpha)) * desired_delta_mps
    max_abs = max(0.0, max_abs_mps)
    lower = -max_abs if min_bias_mps is None else max(-max_abs, min_bias_mps)
    upper = max_abs if max_bias_mps is None else min(max_abs, max_bias_mps)
    if lower > upper:
        lower, upper = upper, lower
    updated = max(lower, min(upper, current_bias_mps + update))
    return updated, updated - current_bias_mps, True


def update_twist_bias_from_progress_delta(
    *,
    current_bias_mps: float,
    observed_progress_delta_m: float,
    raw_forward_delta_m: float,
    dt_sec: float,
    alpha: float,
    max_step_mps: float,
    max_abs_mps: float,
    max_progress_residual_m: float,
    min_bias_mps: float | None = None,
    max_bias_mps: float | None = None,
) -> tuple[float, float | None, bool]:
    if (
        not math.isfinite(current_bias_mps)
        or not math.isfinite(observed_progress_delta_m)
        or not math.isfinite(raw_forward_delta_m)
        or not math.isfinite(dt_sec)
        or dt_sec <= 0.0
    ):
        return current_bias_mps, None, False
    progress_residual_m = observed_progress_delta_m - raw_forward_delta_m
    if abs(progress_residual_m) > max(0.0, max_progress_residual_m):
        return current_bias_mps, None, False
    desired_bias_mps = progress_residual_m / dt_sec
    max_abs = max(0.0, max_abs_mps)
    lower = -max_abs if min_bias_mps is None else max(-max_abs, min_bias_mps)
    upper = max_abs if max_bias_mps is None else min(max_abs, max_bias_mps)
    if lower > upper:
        lower, upper = upper, lower
    desired_bias_mps = max(lower, min(upper, desired_bias_mps))
    desired_update = desired_bias_mps - current_bias_mps
    step_limit = max(0.0, max_step_mps)
    limited_update = max(-step_limit, min(step_limit, desired_update))
    update = max(0.0, min(1.0, alpha)) * limited_update
    updated = max(lower, min(upper, current_bias_mps + update))
    return updated, updated - current_bias_mps, True


def scan_geometry_degeneracy_metrics(
    points: list[Point2D],
    *,
    min_total_side_points: int,
    min_side_points: int,
    min_abs_lateral_m: float,
    min_forward_m: float,
    max_forward_m: float,
    min_side_fraction: float,
) -> dict[str, float | int | bool]:
    left_count = 0
    right_count = 0
    min_abs_lateral = max(0.0, min_abs_lateral_m)
    min_forward = max(0.0, min_forward_m)
    max_forward = max(min_forward, max_forward_m)
    for x, y in points:
        if x < min_forward or x > max_forward:
            continue
        if y >= min_abs_lateral:
            left_count += 1
        elif y <= -min_abs_lateral:
            right_count += 1
    total = left_count + right_count
    weak_side = min(left_count, right_count)
    weak_fraction = weak_side / max(total, 1)
    enough_points = total >= max(0, min_total_side_points)
    degenerate = bool(
        enough_points
        and (
            weak_side <= max(0, min_side_points)
            or weak_fraction <= max(0.0, min_side_fraction)
        )
    )
    return {
        "scan_geometry_degenerate": degenerate,
        "scan_geometry_left_count": left_count,
        "scan_geometry_right_count": right_count,
        "scan_geometry_total_side_points": total,
        "scan_geometry_weak_side_fraction": weak_fraction,
    }


def scan_point_count_degeneracy_metrics(
    point_count: int,
    *,
    max_points: int,
) -> dict[str, float | int | bool]:
    degenerate = point_count > 0 and point_count <= max(0, max_points)
    return {
        "scan_point_count_degenerate": degenerate,
        "scan_point_count": int(point_count),
        "scan_point_count_max_points": int(max_points),
    }


def scan_point_count_persistence_metrics(
    *,
    is_degenerate: bool,
    stamp_sec: float,
    first_degenerate_stamp_sec: float | None,
    min_duration_sec: float,
) -> dict[str, float | bool | None]:
    """Require sparse-scan evidence to persist before it can trigger route hold."""

    required_sec = max(0.0, min_duration_sec)
    if not is_degenerate:
        return {
            "scan_point_count_first_degenerate_stamp_sec": None,
            "scan_point_count_degenerate_duration_sec": 0.0,
            "scan_point_count_persistent_degenerate": False,
        }
    start_sec = stamp_sec if first_degenerate_stamp_sec is None else first_degenerate_stamp_sec
    duration_sec = max(0.0, stamp_sec - start_sec)
    return {
        "scan_point_count_first_degenerate_stamp_sec": start_sec,
        "scan_point_count_degenerate_duration_sec": duration_sec,
        "scan_point_count_persistent_degenerate": duration_sec >= required_sec,
    }


def runtime_localizability_degeneracy_metrics(
    payload: dict,
    *,
    min_along_variance_m2: float,
    min_along_to_cross_ratio: float,
    hold_sec: float,
    current_degenerate_until_sec: float,
    runtime_localizability_remaps_ndt: bool = True,
) -> dict[str, float | int | bool]:
    """Summarize NDT runtime localizability and update the causal hold horizon."""

    summary = payload_candidate_localizability_summary(payload)
    along_degenerate = payload_indicates_along_degeneracy(
        payload,
        min_along_variance_m2=min_along_variance_m2,
        min_along_to_cross_ratio=min_along_to_cross_ratio,
    )
    stamp_sec = float(payload.get("stamp_sec", 0.0) or 0.0)
    until_sec = current_degenerate_until_sec
    if along_degenerate and hold_sec > 0.0 and runtime_localizability_remaps_ndt:
        until_sec = max(until_sec, stamp_sec + hold_sec)
    route_hold_until_sec = 0.0
    if along_degenerate and hold_sec > 0.0:
        route_hold_until_sec = stamp_sec + hold_sec
    return {
        **summary,
        "runtime_localizability_along_degenerate": along_degenerate,
        "runtime_localizability_degenerate_until_sec": until_sec,
        "runtime_localizability_route_hold_until_sec": route_hold_until_sec,
    }


def scan_route_geometry_degeneracy_metrics(
    points: list[Point2D],
    *,
    route_path: RoutePath,
    pose: Pose2D,
    predicted_progress_m: float | None,
    search_radius_m: float,
    min_total_side_points: int,
    min_side_points: int,
    min_abs_cross_m: float,
    min_side_fraction: float,
) -> dict[str, float | int | bool]:
    left_count = 0
    right_count = 0
    min_abs_cross = max(0.0, min_abs_cross_m)
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    for x, y in points:
        world_pose = Pose2D(
            stamp_sec=pose.stamp_sec,
            x=pose.x + c * x - s * y,
            y=pose.y + s * x + c * y,
            yaw=pose.yaw,
        )
        projection = route_path.project(
            world_pose,
            predicted_progress_m=predicted_progress_m,
            search_radius_m=search_radius_m,
        )
        if not projection.is_valid:
            continue
        if projection.cross_track_m >= min_abs_cross:
            left_count += 1
        elif projection.cross_track_m <= -min_abs_cross:
            right_count += 1
    total = left_count + right_count
    weak_side = min(left_count, right_count)
    weak_fraction = weak_side / max(total, 1)
    enough_points = total >= max(0, min_total_side_points)
    degenerate = bool(
        enough_points
        and (
            weak_side <= max(0, min_side_points)
            or weak_fraction <= max(0.0, min_side_fraction)
        )
    )
    return {
        "scan_route_geometry_degenerate": degenerate,
        "scan_route_left_count": left_count,
        "scan_route_right_count": right_count,
        "scan_route_total_side_points": total,
        "scan_route_weak_side_fraction": weak_fraction,
    }


def remap_ndt_pose_for_degeneracy(
    *,
    route_path: RoutePath,
    predicted_pose: Pose2D,
    ndt_pose: Pose2D,
    along_degenerate: bool,
    keep_predicted_yaw: bool,
    search_radius_m: float,
) -> Pose2D:
    if not along_degenerate:
        return ndt_pose
    remapped = route_remap_candidate_along_to_prediction(
        route_path,
        predicted_pose,
        ndt_pose,
        keep_predicted_yaw=keep_predicted_yaw,
        search_radius_m=search_radius_m,
    )
    return remapped if remapped is not None else predicted_pose


def parse_float_tuple(text: str, default: tuple[float, ...]) -> tuple[float, ...]:
    values: list[float] = []
    for item in str(text or "").split(","):
        item = item.strip()
        if not item:
            continue
        try:
            values.append(float(item))
        except ValueError:
            continue
    return tuple(values) if values else default


def route_pose_at_progress_with_cross(
    route_path: RoutePath,
    *,
    stamp_sec: float,
    progress_m: float,
    cross_track_m: float,
    yaw_rad: float,
) -> Pose2D:
    center_x, center_y, center_yaw = route_path.center_at_progress(progress_m)
    return Pose2D(
        stamp_sec=stamp_sec,
        x=center_x - math.sin(center_yaw) * cross_track_m,
        y=center_y + math.cos(center_yaw) * cross_track_m,
        yaw=yaw_rad,
    )


def profile_corrected_along_pose(
    *,
    route_path: RoutePath,
    map_submap: LightweightScanSubmap,
    scan_points: list[Point2D],
    predicted_pose: Pose2D,
    ndt_pose: Pose2D,
    forward_offsets_m: tuple[float, ...],
    search_radius_m: float,
    max_points: int,
    max_profile_cells: int,
    lateral_bin_m: float,
    min_quality: float,
    max_residual_m: float,
    min_improvement_m: float,
) -> tuple[Pose2D, dict[str, float | int | bool]] | None:
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
    base_residual = map_submap.longitudinal_profile_residual(
        scan_points,
        route_pose_at_progress_with_cross(
            route_path,
            stamp_sec=ndt_pose.stamp_sec,
            progress_m=predicted_projection.progress_m,
            cross_track_m=ndt_projection.cross_track_m,
            yaw_rad=ndt_pose.yaw,
        ),
        max_points=max_points,
        max_profile_cells=max_profile_cells,
        lateral_bin_m=lateral_bin_m,
    )
    candidates: list[tuple[float, Pose2D, float, float]] = []
    for offset in forward_offsets_m:
        pose = route_pose_at_progress_with_cross(
            route_path,
            stamp_sec=ndt_pose.stamp_sec,
            progress_m=predicted_projection.progress_m + offset,
            cross_track_m=ndt_projection.cross_track_m,
            yaw_rad=ndt_pose.yaw,
        )
        residual = map_submap.longitudinal_profile_residual(
            scan_points,
            pose,
            max_points=max_points,
            max_profile_cells=max_profile_cells,
            lateral_bin_m=lateral_bin_m,
        )
        if not residual.is_valid or residual.quality < min_quality:
            continue
        candidates.append((residual.xy_m, pose, float(offset), residual.quality))
    if not candidates:
        return None
    best_score, best_pose, best_offset, best_quality = min(candidates, key=lambda item: item[0])
    if best_score > max_residual_m:
        return None
    if base_residual.is_valid and best_score > base_residual.xy_m - max(0.0, min_improvement_m):
        return None
    return best_pose, {
        "profile_candidate_count": len(candidates),
        "profile_best_residual_m": best_score,
        "profile_best_offset_m": best_offset,
        "profile_best_quality": best_quality,
        "profile_base_residual_m": base_residual.xy_m if base_residual.is_valid else math.inf,
    }


def elevation_corrected_along_pose(
    *,
    route_path: RoutePath,
    map_xyz: list[tuple[float, float, float]],
    scan_xyz: list[tuple[float, float, float]],
    predicted_pose: Pose2D,
    ndt_pose: Pose2D,
    ndt_z_m: float,
    forward_offsets_m: tuple[float, ...],
    search_radius_m: float,
    max_points: int,
    max_map_xy_distance_m: float,
    min_quality: float,
    max_rmse_m: float,
    min_improvement_m: float,
) -> tuple[Pose2D, dict[str, float | int | bool]] | None:
    """Choose along progress by matching scan height profile to the 3D map.

    This is intentionally one-dimensional: NDT keeps cross/yaw, route progress
    candidates move only along the route.  It is still causal and pure LiDAR:
    map points are static, scan points are current-frame, no GNSS/GT/future.
    """

    if not map_xyz or not scan_xyz:
        return None
    try:
        import numpy as np
        from scipy.spatial import cKDTree
    except Exception:
        return None

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

    map_array = np.asarray(map_xyz, dtype=float)
    if len(map_array) < 8:
        return None
    tree = cKDTree(map_array[:, :2])
    stride = max(1, len(scan_xyz) // max(1, int(max_points)))
    scan = scan_xyz[::stride][:max_points]
    if len(scan) < 8:
        return None

    def score_pose(pose: Pose2D) -> tuple[float, float] | None:
        cy = math.cos(pose.yaw)
        sy = math.sin(pose.yaw)
        world: list[tuple[float, float, float]] = []
        for x, y, z in scan:
            world.append(
                (
                    pose.x + cy * x - sy * y,
                    pose.y + sy * x + cy * y,
                    ndt_z_m + z,
                )
            )
        world_array = np.asarray(world, dtype=float)
        distances, indices = tree.query(world_array[:, :2], k=1)
        mask = distances <= max(float(max_map_xy_distance_m), 0.05)
        quality = float(mask.mean()) if len(mask) else 0.0
        if int(mask.sum()) < 8 or quality < min_quality:
            return None
        dz = world_array[mask, 2] - map_array[indices[mask], 2]
        rmse = float(math.sqrt(float(np.mean(dz * dz))))
        return rmse, quality

    base_pose = route_pose_at_progress_with_cross(
        route_path,
        stamp_sec=ndt_pose.stamp_sec,
        progress_m=predicted_projection.progress_m,
        cross_track_m=ndt_projection.cross_track_m,
        yaw_rad=ndt_pose.yaw,
    )
    base_score = score_pose(base_pose)
    candidates: list[tuple[float, Pose2D, float, float]] = []
    for offset in forward_offsets_m:
        pose = route_pose_at_progress_with_cross(
            route_path,
            stamp_sec=ndt_pose.stamp_sec,
            progress_m=predicted_projection.progress_m + offset,
            cross_track_m=ndt_projection.cross_track_m,
            yaw_rad=ndt_pose.yaw,
        )
        score = score_pose(pose)
        if score is None:
            continue
        rmse, quality = score
        candidates.append((rmse, pose, float(offset), quality))
    if not candidates:
        return None
    best_rmse, best_pose, best_offset, best_quality = min(candidates, key=lambda item: item[0])
    if best_rmse > max_rmse_m:
        return None
    if base_score is not None and best_rmse > base_score[0] - max(0.0, min_improvement_m):
        return None
    return best_pose, {
        "elevation_candidate_count": len(candidates),
        "elevation_best_rmse_m": best_rmse,
        "elevation_best_offset_m": best_offset,
        "elevation_best_quality": best_quality,
        "elevation_base_rmse_m": base_score[0] if base_score is not None else math.inf,
    }


def intensity_corrected_along_pose(
    *,
    route_path: RoutePath,
    map_xyi: list[tuple[float, float, float]],
    scan_xyi: list[tuple[float, float, float]],
    predicted_pose: Pose2D,
    ndt_pose: Pose2D,
    forward_offsets_m: tuple[float, ...],
    search_radius_m: float,
    max_points: int,
    max_map_xy_distance_m: float,
    min_quality: float,
    max_residual: float,
    min_improvement: float,
) -> tuple[Pose2D, dict[str, float | int | bool]] | None:
    """Choose along progress from intensity/reflectivity consistency."""

    if not map_xyi or not scan_xyi:
        return None
    try:
        import numpy as np
        from scipy.spatial import cKDTree
    except Exception:
        return None

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

    map_array = np.asarray(map_xyi, dtype=float)
    if len(map_array) < 8:
        return None
    tree = cKDTree(map_array[:, :2])
    stride = max(1, len(scan_xyi) // max(1, int(max_points)))
    scan = scan_xyi[::stride][:max_points]
    if len(scan) < 8:
        return None

    def score_pose(pose: Pose2D) -> tuple[float, float] | None:
        cy = math.cos(pose.yaw)
        sy = math.sin(pose.yaw)
        world_xy: list[tuple[float, float]] = []
        scan_i: list[float] = []
        for x, y, intensity in scan:
            world_xy.append((pose.x + cy * x - sy * y, pose.y + sy * x + cy * y))
            scan_i.append(math.log1p(max(0.0, intensity)))
        world_array = np.asarray(world_xy, dtype=float)
        distances, indices = tree.query(world_array, k=1)
        mask = distances <= max(float(max_map_xy_distance_m), 0.05)
        quality = float(mask.mean()) if len(mask) else 0.0
        if int(mask.sum()) < 8 or quality < min_quality:
            return None
        scan_values = np.asarray(scan_i, dtype=float)[mask]
        map_values = np.log1p(np.maximum(0.0, map_array[indices[mask], 2]))
        # Remove a scalar gain/exposure mismatch; only the local fingerprint
        # shape should influence along progress.
        diff = (scan_values - float(np.median(scan_values))) - (
            map_values - float(np.median(map_values))
        )
        residual = float(np.median(np.abs(diff)))
        return residual, quality

    base_pose = route_pose_at_progress_with_cross(
        route_path,
        stamp_sec=ndt_pose.stamp_sec,
        progress_m=predicted_projection.progress_m,
        cross_track_m=ndt_projection.cross_track_m,
        yaw_rad=ndt_pose.yaw,
    )
    base_score = score_pose(base_pose)
    candidates: list[tuple[float, Pose2D, float, float]] = []
    for offset in forward_offsets_m:
        pose = route_pose_at_progress_with_cross(
            route_path,
            stamp_sec=ndt_pose.stamp_sec,
            progress_m=predicted_projection.progress_m + offset,
            cross_track_m=ndt_projection.cross_track_m,
            yaw_rad=ndt_pose.yaw,
        )
        score = score_pose(pose)
        if score is None:
            continue
        residual, quality = score
        candidates.append((residual, pose, float(offset), quality))
    if not candidates:
        return None
    best_residual, best_pose, best_offset, best_quality = min(candidates, key=lambda item: item[0])
    if best_residual > max_residual:
        return None
    if base_score is not None and best_residual > base_score[0] - max(0.0, min_improvement):
        return None
    return best_pose, {
        "intensity_candidate_count": len(candidates),
        "intensity_best_residual": best_residual,
        "intensity_best_offset_m": best_offset,
        "intensity_best_quality": best_quality,
        "intensity_base_residual": base_score[0] if base_score is not None else math.inf,
    }


def reflector_spatial_corrected_along_pose(
    *,
    route_path: RoutePath,
    map_xyi: list[tuple[float, float, float]],
    scan_xyi: list[tuple[float, float, float]],
    predicted_pose: Pose2D,
    ndt_pose: Pose2D,
    forward_offsets_m: tuple[float, ...],
    search_radius_m: float,
    max_points: int,
    min_map_intensity: float,
    min_scan_intensity: float,
    max_match_distance_m: float,
    min_quality: float,
    min_improvement_m: float,
) -> tuple[Pose2D, dict[str, float | int | bool]] | None:
    """Use sparse high-reflectivity points as longitudinal landmarks."""

    map_points = [(x, y) for x, y, intensity in map_xyi if intensity >= min_map_intensity]
    scan_points = [
        (x, y)
        for x, y, intensity in scan_xyi
        if intensity >= min_scan_intensity
    ][: max(1, int(max_points))]
    if len(map_points) < 8 or len(scan_points) < 8:
        return None
    try:
        import numpy as np
        from scipy.spatial import cKDTree
    except Exception:
        return None

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

    tree = cKDTree(np.asarray(map_points, dtype=float))

    def score_pose(pose: Pose2D) -> tuple[float, float] | None:
        cy = math.cos(pose.yaw)
        sy = math.sin(pose.yaw)
        world = np.asarray(
            [(pose.x + cy * x - sy * y, pose.y + sy * x + cy * y) for x, y in scan_points],
            dtype=float,
        )
        distances, _indices = tree.query(world, k=1)
        quality = float((distances <= max_match_distance_m).mean()) if len(distances) else 0.0
        if quality < min_quality:
            return None
        clipped = np.minimum(distances, max_match_distance_m)
        return float(np.median(clipped)), quality

    base_pose = route_pose_at_progress_with_cross(
        route_path,
        stamp_sec=ndt_pose.stamp_sec,
        progress_m=predicted_projection.progress_m,
        cross_track_m=ndt_projection.cross_track_m,
        yaw_rad=ndt_pose.yaw,
    )
    base_score = score_pose(base_pose)
    candidates: list[tuple[float, Pose2D, float, float]] = []
    for offset in forward_offsets_m:
        pose = route_pose_at_progress_with_cross(
            route_path,
            stamp_sec=ndt_pose.stamp_sec,
            progress_m=predicted_projection.progress_m + offset,
            cross_track_m=ndt_projection.cross_track_m,
            yaw_rad=ndt_pose.yaw,
        )
        score = score_pose(pose)
        if score is None:
            continue
        residual, quality = score
        candidates.append((residual, pose, float(offset), quality))
    if not candidates:
        return None
    best_residual, best_pose, best_offset, best_quality = min(candidates, key=lambda item: item[0])
    if base_score is not None and best_residual > base_score[0] - max(0.0, min_improvement_m):
        return None
    return best_pose, {
        "reflector_candidate_count": len(candidates),
        "reflector_best_residual_m": best_residual,
        "reflector_best_offset_m": best_offset,
        "reflector_best_quality": best_quality,
        "reflector_base_residual_m": base_score[0] if base_score is not None else math.inf,
        "reflector_scan_point_count": len(scan_points),
        "reflector_map_point_count": len(map_points),
    }


class PureLidarAxisRemapper(Node):
    def __init__(self) -> None:
        super().__init__("pure_lidar_axis_remapper")
        self.declare_parameter("raw_ndt_pose_topic", "/localization/ndt/raw_pose_with_covariance")
        self.declare_parameter("runtime_multistart_topic", "/localization/ndt/runtime_multistart/decision")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("startup_initial_pose_topic", "/localization/ndt_initial_pose")
        self.declare_parameter("output_pose_topic", "/localization/pure_lidar_axis/pose_with_covariance")
        self.declare_parameter("diagnostics_topic", "/localization/pure_lidar_axis/diagnostics")
        self.declare_parameter("route_samples_csv", "")
        self.declare_parameter("keep_predicted_yaw_when_degenerate", False)
        self.declare_parameter("route_search_radius_m", 30.0)
        self.declare_parameter("degenerate_along_min_variance_m2", 0.1)
        self.declare_parameter("covariance_along_degenerate_variance_m2", 1.0)
        self.declare_parameter("covariance_degeneracy_remaps_ndt", True)
        self.declare_parameter("runtime_localizability_remaps_ndt", True)
        self.declare_parameter("covariance_route_hold_requires_scan_point_count_degenerate", False)
        self.declare_parameter("degenerate_along_min_ratio", 1.5)
        self.declare_parameter("degenerate_along_hold_sec", 3.0)
        self.declare_parameter("twist_linear_x_scale", 1.0)
        self.declare_parameter("twist_linear_x_bias_mps", 0.0)
        self.declare_parameter("enable_twist_bias_learning", False)
        self.declare_parameter("twist_bias_learning_alpha", 0.05)
        self.declare_parameter("twist_bias_learning_max_step_mps", 0.02)
        self.declare_parameter("twist_bias_learning_max_abs_mps", 0.25)
        self.declare_parameter("twist_bias_learning_min_mps", math.nan)
        self.declare_parameter("twist_bias_learning_max_mps", math.nan)
        self.declare_parameter("twist_bias_learning_max_progress_innovation_m", 1.0)
        self.declare_parameter("enable_twist_bias_window_learning", False)
        self.declare_parameter("twist_bias_window_learning_min_dt_sec", 2.0)
        self.declare_parameter("twist_bias_window_learning_max_dt_sec", 8.0)
        self.declare_parameter("twist_bias_window_learning_max_progress_residual_m", 2.0)
        self.declare_parameter("publish_prediction_on_velocity", False)
        self.declare_parameter("enable_startup_initial_pose_init", False)
        self.declare_parameter("startup_initial_pose_cutoff_sec", 2.0)
        self.declare_parameter("enable_map_profile_along_correction", False)
        self.declare_parameter("pointcloud_topic", "/sensing/lidar/concatenated/pointcloud")
        self.declare_parameter("map_pcd_dir", "")
        self.declare_parameter("map_profile_voxel_size_m", 0.8)
        self.declare_parameter("map_profile_max_points", 200000)
        self.declare_parameter("map_profile_pcd_stride", 40)
        self.declare_parameter("map_profile_scan_sample_stride", 20)
        self.declare_parameter("map_profile_scan_max_points", 256)
        self.declare_parameter("map_profile_forward_offsets_m", "-2.0,-1.0,0.0,1.0,2.0")
        self.declare_parameter("map_profile_max_profile_cells", 4000)
        self.declare_parameter("map_profile_lateral_bin_m", 0.8)
        self.declare_parameter("map_profile_min_quality", 0.25)
        self.declare_parameter("map_profile_max_residual_m", 3.0)
        self.declare_parameter("map_profile_min_improvement_m", 0.5)
        self.declare_parameter("map_profile_max_scan_age_sec", 0.35)
        self.declare_parameter("enable_map_elevation_along_correction", False)
        self.declare_parameter("map_elevation_pcd_stride", 40)
        self.declare_parameter("map_elevation_scan_max_points", 512)
        self.declare_parameter("map_elevation_forward_offsets_m", "-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0")
        self.declare_parameter("map_elevation_max_xy_distance_m", 0.8)
        self.declare_parameter("map_elevation_min_quality", 0.2)
        self.declare_parameter("map_elevation_max_rmse_m", 1.5)
        self.declare_parameter("map_elevation_min_improvement_m", 0.2)
        self.declare_parameter("enable_map_intensity_along_correction", False)
        self.declare_parameter("map_intensity_pcd_stride", 20)
        self.declare_parameter("map_intensity_scan_max_points", 512)
        self.declare_parameter("map_intensity_forward_offsets_m", "-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0")
        self.declare_parameter("map_intensity_max_xy_distance_m", 0.8)
        self.declare_parameter("map_intensity_min_quality", 0.2)
        self.declare_parameter("map_intensity_max_residual", 1.0)
        self.declare_parameter("map_intensity_min_improvement", 0.05)
        self.declare_parameter("enable_reflector_spatial_along_correction", False)
        self.declare_parameter("reflector_forward_offsets_m", "-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0")
        self.declare_parameter("reflector_map_min_intensity", 200.0)
        self.declare_parameter("reflector_scan_min_intensity", 200.0)
        self.declare_parameter("reflector_max_match_distance_m", 1.2)
        self.declare_parameter("reflector_min_quality", 0.15)
        self.declare_parameter("reflector_min_improvement_m", 0.2)
        self.declare_parameter("reflector_scan_max_points", 512)
        self.declare_parameter("enable_ndt_consistency_rejection", False)
        self.declare_parameter("consistency_max_xy_innovation_m", math.inf)
        self.declare_parameter("consistency_max_yaw_innovation_deg", 8.0)
        self.declare_parameter("enable_route_cross_hold", False)
        self.declare_parameter("route_cross_target_m", math.nan)
        self.declare_parameter("route_cross_hold_gain", 1.0)
        self.declare_parameter("route_cross_hold_yaw_gain", 0.0)
        self.declare_parameter("route_cross_hold_gate_m", 6.0)
        self.declare_parameter("route_hold_only_when_stale_or_degenerate", False)
        self.declare_parameter("route_hold_min_ndt_stale_sec", 0.5)
        self.declare_parameter("enable_route_cross_target_learning", False)
        self.declare_parameter("route_cross_target_learning_alpha", 0.05)
        self.declare_parameter("route_cross_target_learning_max_step_m", 0.2)
        self.declare_parameter("route_cross_target_learning_max_abs_m", 3.0)
        self.declare_parameter("enable_route_cross_target_stable_init", False)
        self.declare_parameter("route_cross_target_stable_min_count", 8)
        self.declare_parameter("route_cross_target_stable_max_range_m", 0.5)
        self.declare_parameter("enable_route_progress_hold", False)
        self.declare_parameter("route_progress_hold_gain", 1.0)
        self.declare_parameter("route_progress_hold_yaw_gain", 0.0)
        self.declare_parameter("route_progress_hold_gate_m", 10.0)
        self.declare_parameter("route_progress_velocity_scale", 1.0)
        self.declare_parameter("enable_route_progress_innovation_gate", False)
        self.declare_parameter("route_progress_innovation_gate_m", 3.0)
        self.declare_parameter("enable_route_progress_ndt_anchor", False)
        self.declare_parameter("route_progress_ndt_anchor_gate_m", 1.0)
        self.declare_parameter("route_progress_ndt_anchor_gain", 0.5)
        self.declare_parameter("route_progress_ndt_anchor_max_step_m", 0.3)
        self.declare_parameter("enable_scan_geometry_degeneracy", False)
        self.declare_parameter("enable_scan_point_count_degeneracy", False)
        self.declare_parameter("scan_point_count_triggers_route_hold", True)
        self.declare_parameter("scan_point_count_max_points", 9000)
        self.declare_parameter("scan_point_count_hold_sec", 3.0)
        self.declare_parameter("scan_point_count_min_duration_sec", 0.0)
        self.declare_parameter("scan_geometry_min_total_side_points", 80)
        self.declare_parameter("scan_geometry_min_side_points", 8)
        self.declare_parameter("scan_geometry_min_abs_lateral_m", 2.0)
        self.declare_parameter("scan_geometry_min_forward_m", 2.0)
        self.declare_parameter("scan_geometry_max_forward_m", 80.0)
        self.declare_parameter("scan_geometry_min_side_fraction", 0.08)
        self.declare_parameter("scan_geometry_hold_sec", 1.0)
        self.declare_parameter("enable_scan_submap_yaw_correction", False)
        self.declare_parameter("scan_submap_yaw_offsets_deg", "-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0")
        self.declare_parameter("scan_submap_voxel_size_m", 0.3)
        self.declare_parameter("scan_submap_max_cells", 50000)
        self.declare_parameter("scan_submap_min_cells", 200)
        self.declare_parameter("scan_submap_yaw_min_quality", 0.35)
        self.declare_parameter("scan_submap_yaw_max_residual_m", 0.8)
        self.declare_parameter("scan_submap_yaw_min_improvement_m", 0.15)
        self.declare_parameter("enable_scan_submap_along_correction", False)
        self.declare_parameter("scan_submap_along_forward_offsets_m", "-2.0,-1.0,0.0,1.0,2.0")
        self.declare_parameter("scan_submap_along_min_quality", 0.35)
        self.declare_parameter("scan_submap_along_max_residual_m", 0.8)
        self.declare_parameter("scan_submap_along_min_improvement_m", 0.15)
        self.declare_parameter("scan_submap_along_reject_boundary_best", True)
        self.declare_parameter("scan_submap_along_min_second_best_margin_m", 0.10)

        route_csv = str(self.get_parameter("route_samples_csv").value or "")
        self._route_path = RoutePath.from_csv(route_csv) if route_csv else None
        self._keep_predicted_yaw = bool(
            self.get_parameter("keep_predicted_yaw_when_degenerate").value
        )
        self._route_search_radius_m = float(self.get_parameter("route_search_radius_m").value)
        self._degenerate_along_min_variance_m2 = float(
            self.get_parameter("degenerate_along_min_variance_m2").value
        )
        self._covariance_along_degenerate_variance_m2 = float(
            self.get_parameter("covariance_along_degenerate_variance_m2").value
        )
        self._covariance_degeneracy_remaps_ndt = bool(
            self.get_parameter("covariance_degeneracy_remaps_ndt").value
        )
        self._runtime_localizability_remaps_ndt = bool(
            self.get_parameter("runtime_localizability_remaps_ndt").value
        )
        self._covariance_route_hold_requires_scan_point_count_degenerate = bool(
            self.get_parameter("covariance_route_hold_requires_scan_point_count_degenerate").value
        )
        self._degenerate_along_min_ratio = float(
            self.get_parameter("degenerate_along_min_ratio").value
        )
        self._degenerate_along_hold_sec = max(
            0.0, float(self.get_parameter("degenerate_along_hold_sec").value)
        )
        self._velocity_scale = float(self.get_parameter("twist_linear_x_scale").value)
        self._velocity_bias_mps = float(self.get_parameter("twist_linear_x_bias_mps").value)
        self._enable_twist_bias_learning = bool(
            self.get_parameter("enable_twist_bias_learning").value
        )
        self._twist_bias_learning_alpha = max(
            0.0, min(1.0, float(self.get_parameter("twist_bias_learning_alpha").value))
        )
        self._twist_bias_learning_max_step_mps = max(
            0.0, float(self.get_parameter("twist_bias_learning_max_step_mps").value)
        )
        self._twist_bias_learning_max_abs_mps = max(
            0.0, float(self.get_parameter("twist_bias_learning_max_abs_mps").value)
        )
        configured_min_bias = float(self.get_parameter("twist_bias_learning_min_mps").value)
        configured_max_bias = float(self.get_parameter("twist_bias_learning_max_mps").value)
        self._twist_bias_learning_min_mps = (
            configured_min_bias if math.isfinite(configured_min_bias) else None
        )
        self._twist_bias_learning_max_mps = (
            configured_max_bias if math.isfinite(configured_max_bias) else None
        )
        self._twist_bias_learning_max_progress_innovation_m = max(
            0.0,
            float(self.get_parameter("twist_bias_learning_max_progress_innovation_m").value),
        )
        self._enable_twist_bias_window_learning = bool(
            self.get_parameter("enable_twist_bias_window_learning").value
        )
        self._twist_bias_window_learning_min_dt_sec = max(
            0.0,
            float(self.get_parameter("twist_bias_window_learning_min_dt_sec").value),
        )
        self._twist_bias_window_learning_max_dt_sec = max(
            self._twist_bias_window_learning_min_dt_sec,
            float(self.get_parameter("twist_bias_window_learning_max_dt_sec").value),
        )
        self._twist_bias_window_learning_max_progress_residual_m = max(
            0.0,
            float(self.get_parameter("twist_bias_window_learning_max_progress_residual_m").value),
        )
        self._publish_prediction_on_velocity = bool(
            self.get_parameter("publish_prediction_on_velocity").value
        )
        self._enable_startup_initial_pose_init = bool(
            self.get_parameter("enable_startup_initial_pose_init").value
        )
        self._startup_initial_pose_cutoff_sec = max(
            0.0, float(self.get_parameter("startup_initial_pose_cutoff_sec").value)
        )
        self._enable_map_profile_along_correction = bool(
            self.get_parameter("enable_map_profile_along_correction").value
        )
        self._map_profile_scan_sample_stride = max(
            1, int(self.get_parameter("map_profile_scan_sample_stride").value)
        )
        self._map_profile_scan_max_points = max(
            1, int(self.get_parameter("map_profile_scan_max_points").value)
        )
        self._map_profile_forward_offsets = parse_float_tuple(
            str(self.get_parameter("map_profile_forward_offsets_m").value),
            (-2.0, -1.0, 0.0, 1.0, 2.0),
        )
        self._map_profile_max_profile_cells = max(
            1, int(self.get_parameter("map_profile_max_profile_cells").value)
        )
        self._map_profile_lateral_bin_m = float(
            self.get_parameter("map_profile_lateral_bin_m").value
        )
        self._map_profile_min_quality = float(self.get_parameter("map_profile_min_quality").value)
        self._map_profile_max_residual_m = float(
            self.get_parameter("map_profile_max_residual_m").value
        )
        self._map_profile_min_improvement_m = float(
            self.get_parameter("map_profile_min_improvement_m").value
        )
        self._map_profile_max_scan_age_sec = float(
            self.get_parameter("map_profile_max_scan_age_sec").value
        )
        self._enable_map_elevation_along_correction = bool(
            self.get_parameter("enable_map_elevation_along_correction").value
        )
        self._map_elevation_scan_max_points = max(
            1, int(self.get_parameter("map_elevation_scan_max_points").value)
        )
        self._map_elevation_forward_offsets = parse_float_tuple(
            str(self.get_parameter("map_elevation_forward_offsets_m").value),
            (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0),
        )
        self._map_elevation_max_xy_distance_m = float(
            self.get_parameter("map_elevation_max_xy_distance_m").value
        )
        self._map_elevation_min_quality = float(
            self.get_parameter("map_elevation_min_quality").value
        )
        self._map_elevation_max_rmse_m = float(
            self.get_parameter("map_elevation_max_rmse_m").value
        )
        self._map_elevation_min_improvement_m = float(
            self.get_parameter("map_elevation_min_improvement_m").value
        )
        self._enable_map_intensity_along_correction = bool(
            self.get_parameter("enable_map_intensity_along_correction").value
        )
        self._map_intensity_scan_max_points = max(
            1, int(self.get_parameter("map_intensity_scan_max_points").value)
        )
        self._map_intensity_forward_offsets = parse_float_tuple(
            str(self.get_parameter("map_intensity_forward_offsets_m").value),
            (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0),
        )
        self._map_intensity_max_xy_distance_m = float(
            self.get_parameter("map_intensity_max_xy_distance_m").value
        )
        self._map_intensity_min_quality = float(
            self.get_parameter("map_intensity_min_quality").value
        )
        self._map_intensity_max_residual = float(
            self.get_parameter("map_intensity_max_residual").value
        )
        self._map_intensity_min_improvement = float(
            self.get_parameter("map_intensity_min_improvement").value
        )
        self._enable_reflector_spatial_along_correction = bool(
            self.get_parameter("enable_reflector_spatial_along_correction").value
        )
        self._reflector_forward_offsets = parse_float_tuple(
            str(self.get_parameter("reflector_forward_offsets_m").value),
            (-4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0),
        )
        self._reflector_map_min_intensity = float(
            self.get_parameter("reflector_map_min_intensity").value
        )
        self._reflector_scan_min_intensity = float(
            self.get_parameter("reflector_scan_min_intensity").value
        )
        self._reflector_max_match_distance_m = float(
            self.get_parameter("reflector_max_match_distance_m").value
        )
        self._reflector_min_quality = float(self.get_parameter("reflector_min_quality").value)
        self._reflector_min_improvement_m = float(
            self.get_parameter("reflector_min_improvement_m").value
        )
        self._reflector_scan_max_points = max(
            1, int(self.get_parameter("reflector_scan_max_points").value)
        )
        self._enable_ndt_consistency_rejection = bool(
            self.get_parameter("enable_ndt_consistency_rejection").value
        )
        self._consistency_max_xy_innovation_m = float(
            self.get_parameter("consistency_max_xy_innovation_m").value
        )
        self._consistency_max_yaw_innovation_rad = math.radians(
            float(self.get_parameter("consistency_max_yaw_innovation_deg").value)
        )
        self._enable_route_cross_hold = bool(self.get_parameter("enable_route_cross_hold").value)
        configured_cross_target = float(self.get_parameter("route_cross_target_m").value)
        self._route_cross_target_m = (
            configured_cross_target if math.isfinite(configured_cross_target) else None
        )
        self._route_cross_hold_gain = float(self.get_parameter("route_cross_hold_gain").value)
        self._route_cross_hold_yaw_gain = float(
            self.get_parameter("route_cross_hold_yaw_gain").value
        )
        self._route_cross_hold_gate_m = float(self.get_parameter("route_cross_hold_gate_m").value)
        self._route_hold_only_when_stale_or_degenerate = bool(
            self.get_parameter("route_hold_only_when_stale_or_degenerate").value
        )
        self._route_hold_min_ndt_stale_sec = max(
            0.0, float(self.get_parameter("route_hold_min_ndt_stale_sec").value)
        )
        self._enable_route_cross_target_learning = bool(
            self.get_parameter("enable_route_cross_target_learning").value
        )
        self._route_cross_target_learning_alpha = float(
            self.get_parameter("route_cross_target_learning_alpha").value
        )
        self._route_cross_target_learning_max_step_m = float(
            self.get_parameter("route_cross_target_learning_max_step_m").value
        )
        self._route_cross_target_learning_max_abs_m = float(
            self.get_parameter("route_cross_target_learning_max_abs_m").value
        )
        self._enable_route_cross_target_stable_init = bool(
            self.get_parameter("enable_route_cross_target_stable_init").value
        )
        self._route_cross_target_stable_min_count = max(
            1, int(self.get_parameter("route_cross_target_stable_min_count").value)
        )
        self._route_cross_target_stable_max_range_m = max(
            0.0, float(self.get_parameter("route_cross_target_stable_max_range_m").value)
        )
        self._enable_route_progress_hold = bool(
            self.get_parameter("enable_route_progress_hold").value
        )
        self._route_progress_hold_gain = float(self.get_parameter("route_progress_hold_gain").value)
        self._route_progress_hold_yaw_gain = float(
            self.get_parameter("route_progress_hold_yaw_gain").value
        )
        self._route_progress_hold_gate_m = float(
            self.get_parameter("route_progress_hold_gate_m").value
        )
        self._route_progress_velocity_scale = float(
            self.get_parameter("route_progress_velocity_scale").value
        )
        self._enable_route_progress_innovation_gate = bool(
            self.get_parameter("enable_route_progress_innovation_gate").value
        )
        self._route_progress_innovation_gate_m = max(
            0.0, float(self.get_parameter("route_progress_innovation_gate_m").value)
        )
        self._enable_route_progress_ndt_anchor = bool(
            self.get_parameter("enable_route_progress_ndt_anchor").value
        )
        self._route_progress_ndt_anchor_gate_m = max(
            0.0, float(self.get_parameter("route_progress_ndt_anchor_gate_m").value)
        )
        self._route_progress_ndt_anchor_gain = max(
            0.0, min(1.0, float(self.get_parameter("route_progress_ndt_anchor_gain").value))
        )
        self._route_progress_ndt_anchor_max_step_m = max(
            0.0, float(self.get_parameter("route_progress_ndt_anchor_max_step_m").value)
        )
        self._enable_scan_geometry_degeneracy = bool(
            self.get_parameter("enable_scan_geometry_degeneracy").value
        )
        self._enable_scan_point_count_degeneracy = bool(
            self.get_parameter("enable_scan_point_count_degeneracy").value
        )
        self._scan_point_count_triggers_route_hold = bool(
            self.get_parameter("scan_point_count_triggers_route_hold").value
        )
        self._scan_point_count_max_points = max(
            0, int(self.get_parameter("scan_point_count_max_points").value)
        )
        self._scan_point_count_hold_sec = max(
            0.0, float(self.get_parameter("scan_point_count_hold_sec").value)
        )
        self._scan_point_count_min_duration_sec = max(
            0.0, float(self.get_parameter("scan_point_count_min_duration_sec").value)
        )
        self._scan_geometry_min_total_side_points = max(
            0, int(self.get_parameter("scan_geometry_min_total_side_points").value)
        )
        self._scan_geometry_min_side_points = max(
            0, int(self.get_parameter("scan_geometry_min_side_points").value)
        )
        self._scan_geometry_min_abs_lateral_m = max(
            0.0, float(self.get_parameter("scan_geometry_min_abs_lateral_m").value)
        )
        self._scan_geometry_min_forward_m = max(
            0.0, float(self.get_parameter("scan_geometry_min_forward_m").value)
        )
        self._scan_geometry_max_forward_m = max(
            self._scan_geometry_min_forward_m,
            float(self.get_parameter("scan_geometry_max_forward_m").value),
        )
        self._scan_geometry_min_side_fraction = max(
            0.0, float(self.get_parameter("scan_geometry_min_side_fraction").value)
        )
        self._scan_geometry_hold_sec = max(
            0.0, float(self.get_parameter("scan_geometry_hold_sec").value)
        )
        self._enable_scan_submap_yaw_correction = bool(
            self.get_parameter("enable_scan_submap_yaw_correction").value
        )
        self._enable_scan_submap_along_correction = bool(
            self.get_parameter("enable_scan_submap_along_correction").value
        )
        self._scan_submap_yaw_offsets = tuple(
            math.radians(offset)
            for offset in parse_float_tuple(
                str(self.get_parameter("scan_submap_yaw_offsets_deg").value),
                (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0),
            )
        )
        self._scan_submap_min_cells = max(
            1, int(self.get_parameter("scan_submap_min_cells").value)
        )
        self._scan_submap_yaw_min_quality = float(
            self.get_parameter("scan_submap_yaw_min_quality").value
        )
        self._scan_submap_yaw_max_residual_m = float(
            self.get_parameter("scan_submap_yaw_max_residual_m").value
        )
        self._scan_submap_yaw_min_improvement_m = float(
            self.get_parameter("scan_submap_yaw_min_improvement_m").value
        )
        self._scan_submap_along_forward_offsets = parse_float_tuple(
            str(self.get_parameter("scan_submap_along_forward_offsets_m").value),
            (-2.0, -1.0, 0.0, 1.0, 2.0),
        )
        self._scan_submap_along_min_quality = float(
            self.get_parameter("scan_submap_along_min_quality").value
        )
        self._scan_submap_along_max_residual_m = float(
            self.get_parameter("scan_submap_along_max_residual_m").value
        )
        self._scan_submap_along_min_improvement_m = float(
            self.get_parameter("scan_submap_along_min_improvement_m").value
        )
        self._scan_submap_along_reject_boundary_best = bool(
            self.get_parameter("scan_submap_along_reject_boundary_best").value
        )
        self._scan_submap_along_min_second_best_margin_m = float(
            self.get_parameter("scan_submap_along_min_second_best_margin_m").value
        )
        self._scan_submap = (
            LightweightScanSubmap(
                voxel_size_m=float(self.get_parameter("scan_submap_voxel_size_m").value),
                max_cells=max(1, int(self.get_parameter("scan_submap_max_cells").value)),
                unmatched_penalty_m=max(
                    1.0, float(self.get_parameter("scan_submap_voxel_size_m").value) * 4.0
                ),
            )
            if (self._enable_scan_submap_yaw_correction or self._enable_scan_submap_along_correction)
            else None
        )
        self._map_submap = (
            self._load_map_submap(str(self.get_parameter("map_pcd_dir").value or ""))
            if self._enable_map_profile_along_correction
            else None
        )
        self._map_elevation_xyz = (
            self._load_map_elevation_points(str(self.get_parameter("map_pcd_dir").value or ""))
            if self._enable_map_elevation_along_correction
            else []
        )
        self._map_intensity_xyi = (
            self._load_map_intensity_points(str(self.get_parameter("map_pcd_dir").value or ""))
            if (
                self._enable_map_intensity_along_correction
                or self._enable_reflector_spatial_along_correction
            )
            else []
        )
        self._latest_scan_points: list[Point2D] = []
        self._latest_scan_xyz: list[tuple[float, float, float]] = []
        self._latest_scan_xyi: list[tuple[float, float, float]] = []
        self._latest_scan_stamp_sec: float | None = None
        self._latest_scan_geometry_diag: dict[str, float | int | bool] = {}
        self._degenerate_until_sec = -math.inf
        self._route_hold_until_sec = -math.inf
        self._scan_point_count_first_degenerate_stamp_sec: float | None = None
        self._latest_runtime_localizability_diag: dict[str, float | int | bool] = {}
        self._predicted_pose: Pose2D | None = None
        self._last_motion_stamp_sec: float | None = None
        self._last_output_msg: PoseWithCovarianceStamped | None = None
        self._last_ndt_update_stamp_sec: float | None = None
        self._last_route_progress_m: float | None = None
        self._integrated_route_progress_m: float | None = None
        self._raw_forward_integral_m = 0.0
        self._bias_window_anchor_stamp_sec: float | None = None
        self._bias_window_anchor_progress_m: float | None = None
        self._bias_window_anchor_raw_forward_m: float | None = None
        self._route_cross_target_observations: deque[float] = deque(
            maxlen=self._route_cross_target_stable_min_count
        )
        self._startup_initial_pose_first_stamp_sec: float | None = None

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("output_pose_topic").value),
            10,
        )
        self._diag_pub = self.create_publisher(
            String,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("raw_ndt_pose_topic").value),
            self._on_ndt_pose,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("runtime_multistart_topic").value),
            self._on_runtime_decision,
            10,
        )
        self.create_subscription(
            VelocityReport,
            str(self.get_parameter("velocity_topic").value),
            self._on_velocity,
            50,
        )
        if self._enable_startup_initial_pose_init:
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("startup_initial_pose_topic").value),
                self._on_startup_initial_pose,
                10,
            )
        if (
            self._enable_map_profile_along_correction
            or self._enable_map_elevation_along_correction
            or self._enable_map_intensity_along_correction
            or self._enable_reflector_spatial_along_correction
            or self._enable_scan_submap_yaw_correction
            or self._enable_scan_submap_along_correction
            or self._enable_scan_geometry_degeneracy
            or self._enable_scan_point_count_degeneracy
        ):
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("pointcloud_topic").value),
                self._on_pointcloud,
                qos_profile_sensor_data,
            )

    def _on_startup_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._enable_startup_initial_pose_init:
            return
        if self._last_ndt_update_stamp_sec is not None:
            return
        pose = pose2d_from_msg(msg)
        if self._startup_initial_pose_first_stamp_sec is None:
            self._startup_initial_pose_first_stamp_sec = pose.stamp_sec
        if pose.stamp_sec > self._startup_initial_pose_first_stamp_sec + self._startup_initial_pose_cutoff_sec:
            return
        self._predicted_pose = pose
        self._last_motion_stamp_sec = pose.stamp_sec
        self._learn_route_state_from_pose(pose, allow_target_update=False)

    def _load_map_submap(self, map_dir: str) -> LightweightScanSubmap | None:
        if not map_dir:
            return None
        from pathlib import Path

        path = Path(map_dir)
        if not path.exists():
            self.get_logger().warning(f"map_pcd_dir does not exist: {map_dir}")
            return None
        submap = LightweightScanSubmap(
            voxel_size_m=float(self.get_parameter("map_profile_voxel_size_m").value),
            max_cells=max(1, int(self.get_parameter("map_profile_max_points").value)),
            unmatched_penalty_m=max(1.0, float(self.get_parameter("map_profile_voxel_size_m").value) * 4.0),
        )
        stride = max(1, int(self.get_parameter("map_profile_pcd_stride").value))
        added = 0
        seen = 0
        batch: list[Point2D] = []
        for pcd_path in sorted(path.glob("*.pcd")):
            data = False
            try:
                with pcd_path.open("r", encoding="utf-8", errors="ignore") as handle:
                    for line in handle:
                        if not data:
                            if line.strip().lower().startswith("data"):
                                data = True
                            continue
                        seen += 1
                        if seen % stride != 0:
                            continue
                        parts = line.split()
                        if len(parts) < 2:
                            continue
                        try:
                            batch.append((float(parts[0]), float(parts[1])))
                        except ValueError:
                            continue
                        if len(batch) >= 2048:
                            added += submap.add_world_points(batch)
                            batch.clear()
            except Exception as exc:
                self.get_logger().warning(f"failed to read map pcd {pcd_path}: {exc}")
        if batch:
            added += submap.add_world_points(batch)
        self.get_logger().info(
            f"map_profile_loaded points={len(submap._points2d)} cells={submap.cell_count} added={added}"
        )
        return submap if submap.cell_count > 0 else None

    def _load_map_elevation_points(self, map_dir: str) -> list[tuple[float, float, float]]:
        if not map_dir:
            return []
        from pathlib import Path

        path = Path(map_dir)
        if not path.exists():
            self.get_logger().warning(f"map_pcd_dir does not exist: {map_dir}")
            return []
        stride = max(1, int(self.get_parameter("map_elevation_pcd_stride").value))
        points: list[tuple[float, float, float]] = []
        seen = 0
        for pcd_path in sorted(path.glob("*.pcd")):
            data = False
            try:
                with pcd_path.open("r", encoding="utf-8", errors="ignore") as handle:
                    for line in handle:
                        if not data:
                            if line.strip().lower().startswith("data"):
                                data = True
                            continue
                        seen += 1
                        if seen % stride != 0:
                            continue
                        parts = line.split()
                        if len(parts) < 3:
                            continue
                        try:
                            points.append((float(parts[0]), float(parts[1]), float(parts[2])))
                        except ValueError:
                            continue
            except Exception as exc:
                self.get_logger().warning(f"failed to read elevation pcd {pcd_path}: {exc}")
        self.get_logger().info(f"map_elevation_loaded points={len(points)}")
        return points

    def _load_map_intensity_points(self, map_dir: str) -> list[tuple[float, float, float]]:
        if not map_dir:
            return []
        from pathlib import Path

        path = Path(map_dir)
        if not path.exists():
            self.get_logger().warning(f"map_pcd_dir does not exist: {map_dir}")
            return []
        stride = max(1, int(self.get_parameter("map_intensity_pcd_stride").value))
        points: list[tuple[float, float, float]] = []
        seen = 0
        for pcd_path in sorted(path.glob("*.pcd")):
            data = False
            try:
                with pcd_path.open("r", encoding="utf-8", errors="ignore") as handle:
                    for line in handle:
                        if not data:
                            if line.strip().lower().startswith("data"):
                                data = True
                            continue
                        seen += 1
                        if seen % stride != 0:
                            continue
                        parts = line.split()
                        if len(parts) < 4:
                            continue
                        try:
                            points.append((float(parts[0]), float(parts[1]), float(parts[3])))
                        except ValueError:
                            continue
            except Exception as exc:
                self.get_logger().warning(f"failed to read intensity pcd {pcd_path}: {exc}")
        self.get_logger().info(f"map_intensity_loaded points={len(points)}")
        return points

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        points: list[tuple[float, float, float, float]] = []
        try:
            rows = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            for index, row in enumerate(rows):
                if index % self._map_profile_scan_sample_stride != 0:
                    continue
                points.append((float(row[0]), float(row[1]), float(row[2]), float(row[3])))
                if len(points) >= self._map_profile_scan_max_points:
                    break
        except Exception as exc:
            self.get_logger().warning(f"failed to read pointcloud: {exc}")
            return
        self._latest_scan_points = [(p[0], p[1]) for p in points]
        self._latest_scan_xyz = [(p[0], p[1], p[2]) for p in points]
        self._latest_scan_xyi = [(p[0], p[1], p[3]) for p in points]
        self._latest_scan_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self._enable_scan_point_count_degeneracy:
            raw_point_count = int(msg.width) * int(msg.height)
            point_count_diag = scan_point_count_degeneracy_metrics(
                raw_point_count,
                max_points=self._scan_point_count_max_points,
            )
            persistence_diag = scan_point_count_persistence_metrics(
                is_degenerate=bool(point_count_diag["scan_point_count_degenerate"]),
                stamp_sec=self._latest_scan_stamp_sec,
                first_degenerate_stamp_sec=self._scan_point_count_first_degenerate_stamp_sec,
                min_duration_sec=self._scan_point_count_min_duration_sec,
            )
            self._scan_point_count_first_degenerate_stamp_sec = persistence_diag[
                "scan_point_count_first_degenerate_stamp_sec"
            ]
            point_count_diag["scan_sampled_point_count"] = len(points)
            point_count_diag.update(persistence_diag)
            self._latest_scan_geometry_diag.update(point_count_diag)
            if (
                self._scan_point_count_triggers_route_hold
                and point_count_diag["scan_point_count_persistent_degenerate"]
            ):
                self._route_hold_until_sec = max(
                    self._route_hold_until_sec,
                    self._latest_scan_stamp_sec + self._scan_point_count_hold_sec,
                )
        if self._enable_scan_geometry_degeneracy:
            geometry_diag = scan_geometry_degeneracy_metrics(
                self._latest_scan_points,
                min_total_side_points=self._scan_geometry_min_total_side_points,
                min_side_points=self._scan_geometry_min_side_points,
                min_abs_lateral_m=self._scan_geometry_min_abs_lateral_m,
                min_forward_m=self._scan_geometry_min_forward_m,
                max_forward_m=self._scan_geometry_max_forward_m,
                min_side_fraction=self._scan_geometry_min_side_fraction,
            )
            self._latest_scan_geometry_diag.update(geometry_diag)
            if geometry_diag.get("scan_geometry_degenerate"):
                self._route_hold_until_sec = max(
                    self._route_hold_until_sec,
                    self._latest_scan_stamp_sec + self._scan_geometry_hold_sec,
                )

    def _apply_scan_submap_yaw_correction(
        self,
        pose: Pose2D,
    ) -> tuple[Pose2D, dict[str, float | int | bool]]:
        scan_age_sec = (
            abs(pose.stamp_sec - self._latest_scan_stamp_sec)
            if self._latest_scan_stamp_sec is not None
            else math.inf
        )
        diag: dict[str, float | int | bool] = {
            "scan_submap_yaw_applied": False,
            "scan_submap_cell_count": self._scan_submap.cell_count if self._scan_submap else 0,
            "scan_submap_yaw_scan_age_sec": scan_age_sec,
        }
        if not self._enable_scan_submap_yaw_correction:
            diag["scan_submap_yaw_skip_reason"] = "disabled"
            return pose, diag
        if self._scan_submap is None:
            diag["scan_submap_yaw_skip_reason"] = "no_submap"
            return pose, diag
        if self._scan_submap.cell_count < self._scan_submap_min_cells:
            diag["scan_submap_yaw_skip_reason"] = "submap_too_small"
            return pose, diag
        if not self._latest_scan_points:
            diag["scan_submap_yaw_skip_reason"] = "no_scan_points"
            return pose, diag
        if self._latest_scan_stamp_sec is None:
            diag["scan_submap_yaw_skip_reason"] = "no_scan_stamp"
            return pose, diag
        if scan_age_sec > self._map_profile_max_scan_age_sec:
            diag["scan_submap_yaw_skip_reason"] = "scan_too_old"
            return pose, diag
        result = scan_submap_yaw_corrected_pose(
            submap=self._scan_submap,
            scan_points=self._latest_scan_points,
            predicted_pose=pose,
            yaw_offsets_rad=self._scan_submap_yaw_offsets,
            max_points=self._map_profile_scan_max_points,
            min_quality=self._scan_submap_yaw_min_quality,
            max_residual_m=self._scan_submap_yaw_max_residual_m,
            min_improvement_m=self._scan_submap_yaw_min_improvement_m,
        )
        if result is None:
            return pose, diag
        corrected, correction_diag = result
        return corrected, {**diag, **correction_diag, "scan_submap_yaw_applied": True}

    def _apply_scan_submap_along_correction(
        self,
        pose: Pose2D,
    ) -> tuple[Pose2D, dict[str, float | int | bool]]:
        scan_age_sec = (
            abs(pose.stamp_sec - self._latest_scan_stamp_sec)
            if self._latest_scan_stamp_sec is not None
            else math.inf
        )
        diag: dict[str, float | int | bool] = {
            "scan_submap_along_applied": False,
            "scan_submap_cell_count": self._scan_submap.cell_count if self._scan_submap else 0,
            "scan_submap_along_scan_age_sec": scan_age_sec,
        }
        if not self._enable_scan_submap_along_correction:
            diag["scan_submap_along_skip_reason"] = "disabled"
            return pose, diag
        if self._scan_submap is None:
            diag["scan_submap_along_skip_reason"] = "no_submap"
            return pose, diag
        if self._scan_submap.cell_count < self._scan_submap_min_cells:
            diag["scan_submap_along_skip_reason"] = "submap_too_small"
            return pose, diag
        if not self._latest_scan_points:
            diag["scan_submap_along_skip_reason"] = "no_scan_points"
            return pose, diag
        if self._latest_scan_stamp_sec is None:
            diag["scan_submap_along_skip_reason"] = "no_scan_stamp"
            return pose, diag
        if scan_age_sec > self._map_profile_max_scan_age_sec:
            diag["scan_submap_along_skip_reason"] = "scan_too_old"
            return pose, diag
        result = scan_submap_along_corrected_pose(
            submap=self._scan_submap,
            scan_points=self._latest_scan_points,
            predicted_pose=pose,
            forward_offsets_m=self._scan_submap_along_forward_offsets,
            max_points=self._map_profile_scan_max_points,
            max_profile_cells=self._map_profile_max_profile_cells,
            lateral_bin_m=self._map_profile_lateral_bin_m,
            min_quality=self._scan_submap_along_min_quality,
            max_residual_m=self._scan_submap_along_max_residual_m,
            min_improvement_m=self._scan_submap_along_min_improvement_m,
            reject_boundary_best=self._scan_submap_along_reject_boundary_best,
            min_second_best_margin_m=self._scan_submap_along_min_second_best_margin_m,
        )
        if result is None:
            return pose, diag
        corrected, correction_diag = result
        return corrected, {**diag, **correction_diag, "scan_submap_along_applied": True}

    def _update_scan_route_geometry_degeneracy(
        self,
        pose: Pose2D,
    ) -> dict[str, float | int | bool]:
        if (
            not self._enable_scan_geometry_degeneracy
            or self._route_path is None
            or not self._latest_scan_points
            or self._latest_scan_stamp_sec is None
            or abs(pose.stamp_sec - self._latest_scan_stamp_sec)
            > self._map_profile_max_scan_age_sec
        ):
            return {}
        metrics = scan_route_geometry_degeneracy_metrics(
            self._latest_scan_points,
            route_path=self._route_path,
            pose=pose,
            predicted_progress_m=self._integrated_route_progress_m
            if self._integrated_route_progress_m is not None
            else self._last_route_progress_m,
            search_radius_m=self._route_search_radius_m,
            min_total_side_points=self._scan_geometry_min_total_side_points,
            min_side_points=self._scan_geometry_min_side_points,
            min_abs_cross_m=self._scan_geometry_min_abs_lateral_m,
            min_side_fraction=self._scan_geometry_min_side_fraction,
        )
        self._latest_scan_geometry_diag.update(metrics)
        if metrics.get("scan_route_geometry_degenerate"):
            self._route_hold_until_sec = max(
                self._route_hold_until_sec,
                pose.stamp_sec + self._scan_geometry_hold_sec,
            )
        return metrics

    def _add_latest_scan_to_submap(self, pose: Pose2D) -> None:
        if (
            not (self._enable_scan_submap_yaw_correction or self._enable_scan_submap_along_correction)
            or self._scan_submap is None
            or not self._latest_scan_points
            or self._latest_scan_stamp_sec is None
            or abs(pose.stamp_sec - self._latest_scan_stamp_sec) > self._map_profile_max_scan_age_sec
        ):
            return
        self._scan_submap.add_scan(self._latest_scan_points, pose)

    def _on_velocity(self, msg: VelocityReport) -> None:
        if self._predicted_pose is None:
            return
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec <= 0.0:
            return
        if self._last_motion_stamp_sec is None:
            self._last_motion_stamp_sec = stamp_sec
            return
        dt_sec = stamp_sec - self._last_motion_stamp_sec
        self._last_motion_stamp_sec = stamp_sec
        if dt_sec <= 0.0 or dt_sec > 1.0:
            return
        raw_velocity_mps = float(msg.longitudinal_velocity) * self._velocity_scale
        velocity_mps = raw_velocity_mps + self._velocity_bias_mps
        yaw_rate_radps = float(msg.heading_rate)
        if not math.isfinite(velocity_mps):
            velocity_mps = 0.0
        if math.isfinite(raw_velocity_mps):
            self._raw_forward_integral_m += max(0.0, raw_velocity_mps * dt_sec)
        if not math.isfinite(yaw_rate_radps):
            yaw_rate_radps = 0.0
        if self._enable_route_progress_hold and self._integrated_route_progress_m is not None:
            progress_step = max(0.0, velocity_mps * self._route_progress_velocity_scale * dt_sec)
            if math.isfinite(progress_step):
                self._integrated_route_progress_m += progress_step
                self._last_route_progress_m = self._integrated_route_progress_m
        self._predicted_pose = propagate_pose(
            self._predicted_pose,
            MotionDelta(
                dt_sec=dt_sec,
                forward_m=velocity_mps * dt_sec,
                lateral_m=0.0,
                yaw_rad=yaw_rate_radps * dt_sec,
            ),
        )
        if self._publish_prediction_on_velocity and self._last_output_msg is not None:
            publish_pose, scan_submap_diag = self._apply_scan_submap_yaw_correction(
                self._predicted_pose
            )
            scan_geometry_diag = self._update_scan_route_geometry_degeneracy(publish_pose)
            # Stabilize lateral placement before evaluating longitudinal scan-profile evidence.
            # In W10 the profile is single-sided; scoring along offsets from a laterally biased
            # pose makes the causal submap evidence look invalid even when route cross can be held.
            route_pose, route_diag = self._apply_route_cross_hold(publish_pose)
            if route_pose is not None:
                publish_pose = route_pose
            publish_pose, scan_submap_along_diag = self._apply_scan_submap_along_correction(
                publish_pose
            )
            if publish_pose is not None:
                self._predicted_pose = publish_pose
            out = apply_pose2d_to_msg(self._last_output_msg, self._predicted_pose)
            out.header.stamp = msg.header.stamp
            self._pose_pub.publish(out)
            self._publish_diag(
                self._predicted_pose,
                self._predicted_pose.stamp_sec <= self._degenerate_until_sec,
                covariance_along_var=None,
                degenerate_until_sec=self._degenerate_until_sec,
                source="velocity_prediction",
                extra={
                    **scan_submap_diag,
                    **scan_submap_along_diag,
                    **scan_geometry_diag,
                    **route_diag,
                },
            )

    def _on_runtime_decision(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        metrics = runtime_localizability_degeneracy_metrics(
            payload,
            min_along_variance_m2=self._degenerate_along_min_variance_m2,
            min_along_to_cross_ratio=self._degenerate_along_min_ratio,
            hold_sec=self._degenerate_along_hold_sec,
            current_degenerate_until_sec=self._degenerate_until_sec,
            runtime_localizability_remaps_ndt=self._runtime_localizability_remaps_ndt,
        )
        self._degenerate_until_sec = float(
            metrics["runtime_localizability_degenerate_until_sec"]
        )
        self._route_hold_until_sec = max(
            self._route_hold_until_sec,
            float(metrics["runtime_localizability_route_hold_until_sec"]),
        )
        self._latest_runtime_localizability_diag = metrics

    def _on_ndt_pose(self, msg: PoseWithCovarianceStamped) -> None:
        ndt_pose = pose2d_from_msg(msg)
        if self._predicted_pose is None or self._route_path is None:
            self._predicted_pose = ndt_pose
            self._learn_route_state_from_pose(ndt_pose, allow_target_update=True)
            self._last_motion_stamp_sec = ndt_pose.stamp_sec
            self._last_ndt_update_stamp_sec = ndt_pose.stamp_sec
            self._last_output_msg = copy.deepcopy(msg)
            self._pose_pub.publish(msg)
            self._add_latest_scan_to_submap(ndt_pose)
            self._publish_diag(
                ndt_pose,
                False,
                covariance_along_var=None,
                degenerate_until_sec=self._degenerate_until_sec,
                source="ndt_passthrough",
            )
            return

        covariance_along_var = planar_covariance_variance(
            list(msg.pose.covariance),
            ndt_pose.yaw,
        )
        scan_geometry_diag = self._update_scan_route_geometry_degeneracy(self._predicted_pose)
        covariance_degenerate = (
            covariance_along_var >= self._covariance_along_degenerate_variance_m2
        )
        along_degenerate = ndt_pose.stamp_sec <= self._degenerate_until_sec
        # NDT covariance is a reliable signal that route-progress hold should protect
        # velocity prediction through a degenerate segment.  Remapping the NDT along
        # component itself is kept as a separate switch: in W10, covariance-triggered
        # remap can discard useful NDT outputs and make the early segment worse.
        if covariance_degenerate:
            covariance_route_hold_allowed = (
                not self._covariance_route_hold_requires_scan_point_count_degenerate
                or bool(
                    self._latest_scan_geometry_diag.get(
                        "scan_point_count_persistent_degenerate",
                        self._latest_scan_geometry_diag.get("scan_point_count_degenerate"),
                    )
                )
            )
            if covariance_route_hold_allowed:
                self._route_hold_until_sec = max(
                    self._route_hold_until_sec,
                    ndt_pose.stamp_sec + self._degenerate_along_hold_sec,
                )
            if self._covariance_degeneracy_remaps_ndt:
                along_degenerate = True
                self._degenerate_until_sec = max(
                    self._degenerate_until_sec,
                    ndt_pose.stamp_sec + self._degenerate_along_hold_sec,
                )
        route_progress_innovation: float | None = None
        if (
            self._enable_route_progress_innovation_gate
            and self._route_path is not None
            and self._integrated_route_progress_m is not None
        ):
            route_progress_innovation = route_progress_innovation_m(
                route_path=self._route_path,
                predicted_progress_m=self._integrated_route_progress_m,
                ndt_pose=ndt_pose,
                search_radius_m=self._route_search_radius_m,
            )
            if (
                route_progress_innovation is not None
                and abs(route_progress_innovation) > self._route_progress_innovation_gate_m
            ):
                along_degenerate = True
        consistency_diag: dict[str, float | str] = {}
        if self._enable_ndt_consistency_rejection:
            rejected, consistency_diag = reject_ndt_pose_for_consistency(
                predicted_pose=self._predicted_pose,
                ndt_pose=ndt_pose,
                max_xy_innovation_m=self._consistency_max_xy_innovation_m,
                max_yaw_innovation_rad=self._consistency_max_yaw_innovation_rad,
            )
            if rejected:
                output_pose, route_diag = self._apply_route_cross_hold(self._predicted_pose)
                if output_pose is None:
                    output_pose = self._predicted_pose
                output_msg = apply_pose2d_to_msg(msg, output_pose)
                self._last_output_msg = copy.deepcopy(output_msg)
                self._pose_pub.publish(output_msg)
                self._publish_diag(
                    output_pose,
                    along_degenerate,
                    covariance_along_var=covariance_along_var,
                    degenerate_until_sec=self._degenerate_until_sec,
                    source="ndt_consistency_rejected_prediction",
                    extra={
                        **consistency_diag,
                        **scan_geometry_diag,
                        **route_diag,
                        "twist_bias_mps": self._velocity_bias_mps,
                    },
                )
                return
        learned_bias_update_mps: float | None = None
        learned_bias_update_mps = self._learn_twist_bias_from_ndt(
            predicted_pose=self._predicted_pose,
            ndt_pose=ndt_pose,
        )
        learned_bias_window_update_mps = self._learn_twist_bias_from_progress_window(
            ndt_pose=ndt_pose,
            along_degenerate=along_degenerate,
        )
        profile_diag: dict[str, float | int | bool] = {}
        output_pose = remap_ndt_pose_for_degeneracy(
            route_path=self._route_path,
            predicted_pose=self._predicted_pose,
            ndt_pose=ndt_pose,
            along_degenerate=along_degenerate,
            keep_predicted_yaw=self._keep_predicted_yaw,
            search_radius_m=self._route_search_radius_m,
        )
        source = "ndt_remapped" if along_degenerate else "ndt_passthrough"
        if (
            along_degenerate
            and self._enable_map_profile_along_correction
            and self._map_submap is not None
            and self._latest_scan_points
            and self._latest_scan_stamp_sec is not None
            and abs(ndt_pose.stamp_sec - self._latest_scan_stamp_sec)
            <= self._map_profile_max_scan_age_sec
        ):
            profile = profile_corrected_along_pose(
                route_path=self._route_path,
                map_submap=self._map_submap,
                scan_points=self._latest_scan_points,
                predicted_pose=self._predicted_pose,
                ndt_pose=ndt_pose,
                forward_offsets_m=self._map_profile_forward_offsets,
                search_radius_m=self._route_search_radius_m,
                max_points=self._map_profile_scan_max_points,
                max_profile_cells=self._map_profile_max_profile_cells,
                lateral_bin_m=self._map_profile_lateral_bin_m,
                min_quality=self._map_profile_min_quality,
                max_residual_m=self._map_profile_max_residual_m,
                min_improvement_m=self._map_profile_min_improvement_m,
            )
            if profile is not None:
                output_pose, profile_diag = profile
                source = "map_profile_along"
        if (
            along_degenerate
            and self._enable_map_elevation_along_correction
            and self._map_elevation_xyz
            and self._latest_scan_xyz
            and self._latest_scan_stamp_sec is not None
            and abs(ndt_pose.stamp_sec - self._latest_scan_stamp_sec)
            <= self._map_profile_max_scan_age_sec
        ):
            elevation = elevation_corrected_along_pose(
                route_path=self._route_path,
                map_xyz=self._map_elevation_xyz,
                scan_xyz=self._latest_scan_xyz,
                predicted_pose=self._predicted_pose,
                ndt_pose=ndt_pose,
                ndt_z_m=float(msg.pose.pose.position.z),
                forward_offsets_m=self._map_elevation_forward_offsets,
                search_radius_m=self._route_search_radius_m,
                max_points=self._map_elevation_scan_max_points,
                max_map_xy_distance_m=self._map_elevation_max_xy_distance_m,
                min_quality=self._map_elevation_min_quality,
                max_rmse_m=self._map_elevation_max_rmse_m,
                min_improvement_m=self._map_elevation_min_improvement_m,
            )
            if elevation is not None:
                output_pose, profile_diag = elevation
                source = "map_elevation_along"
        if (
            along_degenerate
            and self._enable_map_intensity_along_correction
            and self._map_intensity_xyi
            and self._latest_scan_xyi
            and self._latest_scan_stamp_sec is not None
            and abs(ndt_pose.stamp_sec - self._latest_scan_stamp_sec)
            <= self._map_profile_max_scan_age_sec
        ):
            intensity = intensity_corrected_along_pose(
                route_path=self._route_path,
                map_xyi=self._map_intensity_xyi,
                scan_xyi=self._latest_scan_xyi,
                predicted_pose=self._predicted_pose,
                ndt_pose=ndt_pose,
                forward_offsets_m=self._map_intensity_forward_offsets,
                search_radius_m=self._route_search_radius_m,
                max_points=self._map_intensity_scan_max_points,
                max_map_xy_distance_m=self._map_intensity_max_xy_distance_m,
                min_quality=self._map_intensity_min_quality,
                max_residual=self._map_intensity_max_residual,
                min_improvement=self._map_intensity_min_improvement,
            )
            if intensity is not None:
                output_pose, profile_diag = intensity
                source = "map_intensity_along"
        if (
            along_degenerate
            and self._enable_reflector_spatial_along_correction
            and self._map_intensity_xyi
            and self._latest_scan_xyi
            and self._latest_scan_stamp_sec is not None
            and abs(ndt_pose.stamp_sec - self._latest_scan_stamp_sec)
            <= self._map_profile_max_scan_age_sec
        ):
            reflector = reflector_spatial_corrected_along_pose(
                route_path=self._route_path,
                map_xyi=self._map_intensity_xyi,
                scan_xyi=self._latest_scan_xyi,
                predicted_pose=self._predicted_pose,
                ndt_pose=ndt_pose,
                forward_offsets_m=self._reflector_forward_offsets,
                search_radius_m=self._route_search_radius_m,
                max_points=self._reflector_scan_max_points,
                min_map_intensity=self._reflector_map_min_intensity,
                min_scan_intensity=self._reflector_scan_min_intensity,
                max_match_distance_m=self._reflector_max_match_distance_m,
                min_quality=self._reflector_min_quality,
                min_improvement_m=self._reflector_min_improvement_m,
            )
            if reflector is not None:
                output_pose, profile_diag = reflector
                source = "reflector_spatial_along"
        route_diag: dict[str, float | bool] = {}
        route_progress_anchor_diag: dict[str, float | bool] = {}
        if source in ("velocity_prediction", "ndt_consistency_rejected_prediction"):
            route_output, route_diag = self._apply_route_cross_hold(output_pose)
            if route_output is not None:
                output_pose = route_output
        else:
            if (
                self._enable_route_progress_ndt_anchor
                and source == "ndt_passthrough"
                and not along_degenerate
                and self._route_path is not None
                and self._integrated_route_progress_m is not None
            ):
                projection = self._route_path.project(
                    output_pose,
                    predicted_progress_m=self._integrated_route_progress_m,
                    search_radius_m=self._route_search_radius_m,
                )
                if projection.is_valid:
                    updated_progress, applied = update_route_progress_anchor(
                        current_progress_m=self._integrated_route_progress_m,
                        observed_progress_m=projection.progress_m,
                        gate_m=self._route_progress_ndt_anchor_gate_m,
                        gain=self._route_progress_ndt_anchor_gain,
                        max_step_m=self._route_progress_ndt_anchor_max_step_m,
                    )
                    route_progress_anchor_diag = {
                        "route_progress_ndt_anchor_applied": applied,
                        "route_progress_ndt_anchor_observed_m": projection.progress_m,
                        "route_progress_ndt_anchor_innovation_m": (
                            projection.progress_m - self._integrated_route_progress_m
                        ),
                    }
                    if applied:
                        self._integrated_route_progress_m = updated_progress
                        self._last_route_progress_m = updated_progress
            self._learn_route_state_from_pose(
                output_pose,
                allow_target_update=source in ("ndt_passthrough", "ndt_remapped"),
            )
        self._predicted_pose = output_pose
        self._last_motion_stamp_sec = ndt_pose.stamp_sec
        self._last_ndt_update_stamp_sec = ndt_pose.stamp_sec
        output_msg = apply_pose2d_to_msg(msg, output_pose)
        self._last_output_msg = copy.deepcopy(output_msg)
        self._pose_pub.publish(output_msg)
        if source != "ndt_consistency_rejected_prediction":
            self._add_latest_scan_to_submap(output_pose)
        self._publish_diag(
            output_pose,
            along_degenerate,
            covariance_along_var=covariance_along_var,
            degenerate_until_sec=self._degenerate_until_sec,
            source=source,
                extra={
                    **consistency_diag,
                    **scan_geometry_diag,
                    **route_diag,
                    **route_progress_anchor_diag,
                    **profile_diag,
                "twist_bias_mps": self._velocity_bias_mps,
                "learned_bias_update_mps": learned_bias_update_mps,
                "learned_bias_window_update_mps": learned_bias_window_update_mps,
                "route_progress_innovation_m": route_progress_innovation,
            },
        )

    def _learn_twist_bias_from_ndt(
        self,
        *,
        predicted_pose: Pose2D,
        ndt_pose: Pose2D,
    ) -> float | None:
        if not self._enable_twist_bias_learning or self._route_path is None:
            return None
        if self._last_ndt_update_stamp_sec is None:
            return None
        dt_sec = ndt_pose.stamp_sec - self._last_ndt_update_stamp_sec
        if dt_sec <= 0.05 or dt_sec > 1.0:
            return None
        predicted_projection = self._route_path.project(
            predicted_pose,
            predicted_progress_m=self._integrated_route_progress_m,
            search_radius_m=self._route_search_radius_m,
        )
        ndt_projection = self._route_path.project(
            ndt_pose,
            predicted_progress_m=predicted_projection.progress_m,
            search_radius_m=self._route_search_radius_m,
        )
        if not predicted_projection.is_valid or not ndt_projection.is_valid:
            return None
        progress_innovation_m = ndt_projection.progress_m - predicted_projection.progress_m
        updated_bias, update, applied = update_twist_bias_from_progress_innovation(
            current_bias_mps=self._velocity_bias_mps,
            progress_innovation_m=progress_innovation_m,
            dt_sec=dt_sec,
            alpha=self._twist_bias_learning_alpha,
            max_step_mps=self._twist_bias_learning_max_step_mps,
            max_abs_mps=self._twist_bias_learning_max_abs_mps,
            max_progress_innovation_m=self._twist_bias_learning_max_progress_innovation_m,
            min_bias_mps=self._twist_bias_learning_min_mps,
            max_bias_mps=self._twist_bias_learning_max_mps,
        )
        if not applied:
            return None
        self._velocity_bias_mps = updated_bias
        return update

    def _learn_twist_bias_from_progress_window(
        self,
        *,
        ndt_pose: Pose2D,
        along_degenerate: bool,
    ) -> float | None:
        if (
            not self._enable_twist_bias_window_learning
            or self._route_path is None
            or along_degenerate
        ):
            return None
        projection = self._route_path.project(
            ndt_pose,
            predicted_progress_m=self._last_route_progress_m,
            search_radius_m=self._route_search_radius_m,
        )
        if not projection.is_valid:
            return None
        if (
            self._bias_window_anchor_stamp_sec is None
            or self._bias_window_anchor_progress_m is None
            or self._bias_window_anchor_raw_forward_m is None
        ):
            self._bias_window_anchor_stamp_sec = ndt_pose.stamp_sec
            self._bias_window_anchor_progress_m = projection.progress_m
            self._bias_window_anchor_raw_forward_m = self._raw_forward_integral_m
            return None
        dt_sec = ndt_pose.stamp_sec - self._bias_window_anchor_stamp_sec
        if dt_sec < self._twist_bias_window_learning_min_dt_sec:
            return None
        observed_progress_delta_m = projection.progress_m - self._bias_window_anchor_progress_m
        raw_forward_delta_m = self._raw_forward_integral_m - self._bias_window_anchor_raw_forward_m
        if dt_sec > self._twist_bias_window_learning_max_dt_sec:
            self._bias_window_anchor_stamp_sec = ndt_pose.stamp_sec
            self._bias_window_anchor_progress_m = projection.progress_m
            self._bias_window_anchor_raw_forward_m = self._raw_forward_integral_m
            return None
        updated_bias, update, applied = update_twist_bias_from_progress_delta(
            current_bias_mps=self._velocity_bias_mps,
            observed_progress_delta_m=observed_progress_delta_m,
            raw_forward_delta_m=raw_forward_delta_m,
            dt_sec=dt_sec,
            alpha=self._twist_bias_learning_alpha,
            max_step_mps=self._twist_bias_learning_max_step_mps,
            max_abs_mps=self._twist_bias_learning_max_abs_mps,
            max_progress_residual_m=self._twist_bias_window_learning_max_progress_residual_m,
            min_bias_mps=self._twist_bias_learning_min_mps,
            max_bias_mps=self._twist_bias_learning_max_mps,
        )
        self._bias_window_anchor_stamp_sec = ndt_pose.stamp_sec
        self._bias_window_anchor_progress_m = projection.progress_m
        self._bias_window_anchor_raw_forward_m = self._raw_forward_integral_m
        if not applied:
            return None
        self._velocity_bias_mps = updated_bias
        return update

    def _learn_route_state_from_pose(
        self,
        pose: Pose2D,
        *,
        allow_target_update: bool,
    ) -> None:
        if self._route_path is None:
            return
        projection = self._route_path.project(
            pose,
            predicted_progress_m=self._last_route_progress_m,
            search_radius_m=self._route_search_radius_m,
        )
        if not projection.is_valid:
            return
        self._last_route_progress_m = projection.progress_m
        if self._integrated_route_progress_m is None:
            self._integrated_route_progress_m = projection.progress_m
        if not allow_target_update:
            return
        if self._route_cross_target_m is None:
            if self._enable_route_cross_target_stable_init:
                self._route_cross_target_observations.append(projection.cross_track_m)
                candidate = stable_route_cross_target_candidate(
                    observations=tuple(self._route_cross_target_observations),
                    min_count=self._route_cross_target_stable_min_count,
                    max_range_m=self._route_cross_target_stable_max_range_m,
                )
                if candidate is not None:
                    self._route_cross_target_m = candidate
            else:
                self._route_cross_target_m = projection.cross_track_m
        elif self._enable_route_cross_target_learning:
            self._route_cross_target_m = update_route_cross_target(
                current_target_m=self._route_cross_target_m,
                observed_cross_m=projection.cross_track_m,
                alpha=self._route_cross_target_learning_alpha,
                max_step_m=self._route_cross_target_learning_max_step_m,
                max_abs_m=self._route_cross_target_learning_max_abs_m,
            )

    def _apply_route_cross_hold(self, pose: Pose2D) -> tuple[Pose2D | None, dict[str, float | bool]]:
        if (
            not self._enable_route_cross_hold
            or self._route_path is None
            or self._route_cross_target_m is None
        ):
            return None, {}
        if not should_apply_route_hold(
            pose_stamp_sec=pose.stamp_sec,
            last_ndt_update_stamp_sec=self._last_ndt_update_stamp_sec,
            degenerate_until_sec=max(self._degenerate_until_sec, self._route_hold_until_sec),
            only_when_stale_or_degenerate=self._route_hold_only_when_stale_or_degenerate,
            stale_sec=self._route_hold_min_ndt_stale_sec,
        ):
            return None, {
                "route_hold_stale_gate_skipped": True,
                "route_hold_ndt_age_sec": pose.stamp_sec - self._last_ndt_update_stamp_sec
                if self._last_ndt_update_stamp_sec is not None
                else math.inf,
            }
        if self._enable_route_progress_hold and self._integrated_route_progress_m is not None:
            corrected, diag = route_progress_held_pose(
                route_path=self._route_path,
                pose=pose,
                progress_m=self._integrated_route_progress_m,
                target_cross_m=self._route_cross_target_m,
                gain=self._route_progress_hold_gain,
                yaw_gain=self._route_progress_hold_yaw_gain,
                gate_m=self._route_progress_hold_gate_m,
            )
            if corrected is not None:
                self._last_route_progress_m = self._integrated_route_progress_m
                return corrected, diag
        corrected, diag = route_cross_held_pose(
            route_path=self._route_path,
            pose=pose,
            target_cross_m=self._route_cross_target_m,
            gain=self._route_cross_hold_gain,
            yaw_gain=self._route_cross_hold_yaw_gain,
            gate_m=self._route_cross_hold_gate_m,
            predicted_progress_m=self._last_route_progress_m,
            search_radius_m=self._route_search_radius_m,
        )
        if corrected is not None:
            projection = self._route_path.project(
                corrected,
                predicted_progress_m=self._last_route_progress_m,
                search_radius_m=self._route_search_radius_m,
            )
            if projection.is_valid:
                self._last_route_progress_m = projection.progress_m
        return corrected, diag

    def _publish_diag(
        self,
        pose: Pose2D,
        along_degenerate: bool,
        *,
        covariance_along_var: float | None,
        degenerate_until_sec: float,
        source: str,
        extra: dict[str, float | int | bool] | None = None,
    ) -> None:
        msg = String()
        payload = {
                "stamp_sec": pose.stamp_sec,
                "along_degenerate": along_degenerate,
                "covariance_along_var_m2": covariance_along_var,
                "degenerate_until_sec": degenerate_until_sec,
                "route_hold_until_sec": self._route_hold_until_sec,
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
                "source": source,
                "uses_gnss_or_gt": False,
                "route_cross_target_m": self._route_cross_target_m,
	                "route_cross_target_observation_count": len(
	                    self._route_cross_target_observations
	                ),
	            }
        if self._latest_scan_geometry_diag:
            payload.update(self._latest_scan_geometry_diag)
        if self._latest_runtime_localizability_diag:
            payload.update(self._latest_runtime_localizability_diag)
        if extra:
            payload.update(extra)
        msg.data = json.dumps(payload, sort_keys=True)
        self._diag_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PureLidarAxisRemapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
