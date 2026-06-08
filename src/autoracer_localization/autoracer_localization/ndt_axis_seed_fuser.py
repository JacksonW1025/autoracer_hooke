import copy
import json
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from .ndt_initial_pose_predictor import (
    _message_time,
    _normalize_angle,
    _rpy_from_quaternion,
    _rpy_to_quaternion,
    _variance_gain,
    _xy_variance,
    _yaw_from_quaternion,
    _yaw_to_quaternion,
    _yaw_variance,
)


def _fuse_axis_specific_pose(
    ndt_msg,
    seed_msg,
    *,
    lateral_gain=1.0,
    yaw_deadband_sigma=3.0,
):
    fused = copy.deepcopy(ndt_msg)
    ndt_pose = fused.pose.pose
    seed_pose = seed_msg.pose.pose
    seed_yaw = _yaw_from_quaternion(seed_pose.orientation)

    dx = float(seed_pose.position.x) - float(ndt_pose.position.x)
    dy = float(seed_pose.position.y) - float(ndt_pose.position.y)
    lateral_x = -math.sin(seed_yaw)
    lateral_y = math.cos(seed_yaw)
    cross = dx * lateral_x + dy * lateral_y

    applied = False
    gain = min(1.0, max(0.0, float(lateral_gain)))
    if abs(cross) >= 1e-4 and gain > 0.0:
        ndt_pose.position.x += lateral_x * cross * gain
        ndt_pose.position.y += lateral_y * cross * gain
        applied = True

    seed_yaw_var = _yaw_variance(seed_msg.pose.covariance)
    seed_yaw_stddev = math.sqrt(seed_yaw_var) if math.isfinite(seed_yaw_var) else math.inf
    yaw_deadband = max(float(yaw_deadband_sigma) * seed_yaw_stddev, 1e-4)
    yaw_error = _normalize_angle(seed_yaw - _yaw_from_quaternion(ndt_pose.orientation))
    yaw_gain = _variance_gain(_yaw_variance(ndt_msg.pose.covariance), seed_yaw_var)
    if abs(yaw_error) >= yaw_deadband and yaw_gain > 0.0:
        roll, pitch, yaw = _rpy_from_quaternion(ndt_pose.orientation)
        ndt_pose.orientation = _rpy_to_quaternion(
            roll,
            pitch,
            _normalize_angle(yaw + yaw_error * yaw_gain),
        )
        applied = True

    return fused, applied


def _fuse_ndt_cross_yaw_seed_along_pose(
    ndt_msg,
    seed_msg,
    *,
    along_gain=0.03,
    max_seed_along_residual_m=0.0,
):
    fused = copy.deepcopy(ndt_msg)
    ndt_pose = fused.pose.pose
    seed_pose = seed_msg.pose.pose
    ndt_yaw = _yaw_from_quaternion(ndt_pose.orientation)
    forward_x = math.cos(ndt_yaw)
    forward_y = math.sin(ndt_yaw)

    dx = float(seed_pose.position.x) - float(ndt_pose.position.x)
    dy = float(seed_pose.position.y) - float(ndt_pose.position.y)
    along = dx * forward_x + dy * forward_y
    gain = _clamp01(along_gain)
    max_residual = max(0.0, float(max_seed_along_residual_m))
    if gain <= 0.0 and max_residual <= 0.0:
        return fused, False
    if abs(along) < 1e-4:
        return fused, False

    applied_along = along * gain
    if max_residual > 0.0:
        residual_after_gain = along - applied_along
        bounded_residual = _clip_axis(residual_after_gain, max_residual)
        applied_along = along - bounded_residual
    if abs(applied_along) < 1e-4:
        return fused, False

    ndt_pose.position.x += forward_x * applied_along
    ndt_pose.position.y += forward_y * applied_along
    return fused, True


def _clamp01(value):
    return min(1.0, max(0.0, float(value)))


def _ndt_is_consistent_with_initial_pose(
    ndt_msg,
    initial_msg,
    *,
    max_distance_m=0.0,
    max_yaw_delta_deg=0.0,
):
    ndt_pose = ndt_msg.pose.pose
    initial_pose = initial_msg.pose.pose
    distance = math.hypot(
        float(ndt_pose.position.x) - float(initial_pose.position.x),
        float(ndt_pose.position.y) - float(initial_pose.position.y),
    )
    yaw_delta = abs(
        math.degrees(
            _normalize_angle(
                _yaw_from_quaternion(ndt_pose.orientation)
                - _yaw_from_quaternion(initial_pose.orientation)
            )
        )
    )
    details = {
        "distance_m": distance,
        "yaw_delta_deg": yaw_delta,
        "reason": "ok",
    }
    if float(max_distance_m) > 0.0 and distance > float(max_distance_m):
        details["reason"] = "distance"
        return False, details
    if float(max_yaw_delta_deg) > 0.0 and yaw_delta > float(max_yaw_delta_deg):
        details["reason"] = "yaw_delta"
        return False, details
    return True, details


def _apply_initial_pose_correction_gain(ndt_msg, initial_msg, *, correction_gain=1.0):
    gain = _clamp01(correction_gain)
    return _apply_initial_pose_axis_correction_gain(
        ndt_msg,
        initial_msg,
        along_gain=gain,
        cross_gain=gain,
        yaw_gain=gain,
        roll_pitch_gain=gain,
        z_gain=gain,
    )


def _apply_initial_pose_axis_correction_gain(
    ndt_msg,
    initial_msg,
    *,
    along_gain=1.0,
    cross_gain=1.0,
    yaw_gain=1.0,
    roll_pitch_gain=1.0,
    z_gain=1.0,
):
    along_gain = _clamp01(along_gain)
    cross_gain = _clamp01(cross_gain)
    yaw_gain = _clamp01(yaw_gain)
    roll_pitch_gain = _clamp01(roll_pitch_gain)
    z_gain = _clamp01(z_gain)
    if (
        along_gain >= 1.0
        and cross_gain >= 1.0
        and yaw_gain >= 1.0
        and roll_pitch_gain >= 1.0
        and z_gain >= 1.0
    ):
        return copy.deepcopy(ndt_msg)

    fused = copy.deepcopy(ndt_msg)
    ndt_pose = ndt_msg.pose.pose
    initial_pose = initial_msg.pose.pose
    fused_pose = fused.pose.pose
    initial_yaw = _yaw_from_quaternion(initial_pose.orientation)
    forward_x = math.cos(initial_yaw)
    forward_y = math.sin(initial_yaw)
    lateral_x = -math.sin(initial_yaw)
    lateral_y = math.cos(initial_yaw)

    dx = float(ndt_pose.position.x) - float(initial_pose.position.x)
    dy = float(ndt_pose.position.y) - float(initial_pose.position.y)
    along = dx * forward_x + dy * forward_y
    cross = dx * lateral_x + dy * lateral_y
    fused_pose.position.x = (
        float(initial_pose.position.x)
        + forward_x * along * along_gain
        + lateral_x * cross * cross_gain
    )
    fused_pose.position.y = (
        float(initial_pose.position.y)
        + forward_y * along * along_gain
        + lateral_y * cross * cross_gain
    )
    fused_pose.position.z = float(initial_pose.position.z) + (
        float(ndt_pose.position.z) - float(initial_pose.position.z)
    ) * z_gain

    initial_roll, initial_pitch, _ = _rpy_from_quaternion(initial_pose.orientation)
    ndt_roll, ndt_pitch, ndt_yaw = _rpy_from_quaternion(ndt_pose.orientation)
    fused_pose.orientation = _rpy_to_quaternion(
        initial_roll + _normalize_angle(ndt_roll - initial_roll) * roll_pitch_gain,
        initial_pitch + _normalize_angle(ndt_pitch - initial_pitch) * roll_pitch_gain,
        _normalize_angle(initial_yaw + _normalize_angle(ndt_yaw - initial_yaw) * yaw_gain),
    )
    return fused


def _clip_axis(value, limit):
    limit = float(limit)
    if limit <= 0.0:
        return float(value)
    return min(limit, max(-limit, float(value)))


def _covariance_source_name(covariance_estimation_type):
    try:
        cov_type = int(covariance_estimation_type)
    except (TypeError, ValueError):
        return "unknown"
    return {
        0: "fixed_cov0_only",
        1: "laplace",
        2: "multi_ndt",
        3: "multi_ndt_score",
    }.get(cov_type, f"type_{cov_type}")


def _axis_variance_from_xy_covariance(covariance, axis_x, axis_y, floor):
    cov_xx = float(covariance[0])
    cov_xy = float(covariance[1])
    cov_yy = float(covariance[7])
    variance = axis_x * axis_x * cov_xx + 2.0 * axis_x * axis_y * cov_xy + axis_y * axis_y * cov_yy
    if not math.isfinite(variance) or variance <= 0.0:
        variance = float(floor)
    return max(float(variance), float(floor))


def _runtime_candidate_spread_variance_inflation(
    decision,
    *,
    min_candidate_count=2,
    along_spread_threshold_m=1.5,
    along_variance_scale=0.5,
    max_abs_along_m=0.0,
    max_abs_cross_m=0.85,
    selected_score_margin=0.0,
    yaw_spread_threshold_deg=3.0,
    yaw_variance_scale=0.5,
):
    metadata = {
        "runtime_candidate_spread_count": 0,
        "runtime_candidate_along_spread_m": 0.0,
        "runtime_candidate_yaw_spread_deg": 0.0,
        "runtime_candidate_spread_inflation": False,
        "runtime_candidate_spread_score_margin": float(selected_score_margin),
    }
    if not isinstance(decision, dict):
        return [0.0, 0.0, 0.0], metadata
    if int(decision.get("candidate_count") or 0) < int(min_candidate_count):
        return [0.0, 0.0, 0.0], metadata

    selected_total_score = None
    if float(selected_score_margin) > 0.0:
        try:
            selected_index = int(decision.get("selected_candidate_index"))
        except (TypeError, ValueError):
            selected_index = None
        for candidate in decision.get("candidates") or []:
            if not isinstance(candidate, dict):
                continue
            try:
                if int(candidate.get("index")) != selected_index:
                    continue
                total_score = float(candidate.get("total_score"))
            except (TypeError, ValueError):
                continue
            if math.isfinite(total_score):
                selected_total_score = total_score
                break
    metadata["runtime_candidate_spread_selected_total_score"] = selected_total_score

    along_values = []
    yaw_values = []
    for candidate in decision.get("candidates") or []:
        if not isinstance(candidate, dict):
            continue
        if not bool(candidate.get("converged", False)):
            continue
        if str(candidate.get("reject_reason") or ""):
            continue
        try:
            total_score = float(candidate.get("total_score", 0.0))
            cross = float(candidate.get("innovation_cross_m", 0.0))
            along = float(candidate.get("innovation_along_m", 0.0))
            yaw_deg = float(candidate.get("innovation_yaw_deg", 0.0))
        except (TypeError, ValueError):
            continue
        if not (math.isfinite(along) and math.isfinite(cross) and math.isfinite(yaw_deg)):
            continue
        if (
            selected_total_score is not None
            and math.isfinite(total_score)
            and total_score < selected_total_score - float(selected_score_margin)
        ):
            continue
        if max_abs_along_m > 0.0 and abs(along) > float(max_abs_along_m):
            continue
        if max_abs_cross_m > 0.0 and abs(cross) > float(max_abs_cross_m):
            continue
        along_values.append(along)
        yaw_values.append(yaw_deg)

    metadata["runtime_candidate_spread_count"] = len(along_values)
    if len(along_values) < int(min_candidate_count):
        return [0.0, 0.0, 0.0], metadata

    along_spread = max(along_values) - min(along_values)
    yaw_spread = max(yaw_values) - min(yaw_values)
    metadata["runtime_candidate_along_spread_m"] = along_spread
    metadata["runtime_candidate_yaw_spread_deg"] = yaw_spread

    inflation = [0.0, 0.0, 0.0]
    if along_spread > float(along_spread_threshold_m) and along_variance_scale > 0.0:
        inflation[0] = (along_spread * float(along_variance_scale)) ** 2
    if yaw_spread > float(yaw_spread_threshold_deg) and yaw_variance_scale > 0.0:
        yaw_inflation_rad = math.radians(yaw_spread * float(yaw_variance_scale))
        inflation[2] = yaw_inflation_rad * yaw_inflation_rad
    metadata["runtime_candidate_spread_inflation"] = any(value > 0.0 for value in inflation)
    return inflation, metadata


def _ekf_initial_pose_update(
    ndt_msg,
    initial_msg,
    *,
    mahalanobis_gate=4.0,
    ndt_covariance_estimation_type=1,
    process_noise_diag=None,
    prior_xy_variance_floor_m2=0.25,
    prior_yaw_variance_floor_rad2=0.007615435494667714,
    measurement_xy_variance_floor_m2=0.09,
    measurement_along_variance_floor_m2=None,
    measurement_cross_variance_floor_m2=None,
    measurement_yaw_variance_floor_rad2=0.00030461741978670857,
    measurement_variance_inflation_diag=None,
    z_gain=1.0,
    roll_pitch_gain=1.0,
):
    process_noise = list(process_noise_diag or [0.0, 0.0, 0.0])
    while len(process_noise) < 3:
        process_noise.append(0.0)
    process_noise = [max(0.0, float(value)) for value in process_noise[:3]]

    initial_pose = initial_msg.pose.pose
    ndt_pose = ndt_msg.pose.pose
    initial_yaw = _yaw_from_quaternion(initial_pose.orientation)
    forward_x = math.cos(initial_yaw)
    forward_y = math.sin(initial_yaw)
    lateral_x = -math.sin(initial_yaw)
    lateral_y = math.cos(initial_yaw)

    dx = float(ndt_pose.position.x) - float(initial_pose.position.x)
    dy = float(ndt_pose.position.y) - float(initial_pose.position.y)
    along = dx * forward_x + dy * forward_y
    cross = dx * lateral_x + dy * lateral_y
    yaw_delta = _normalize_angle(_yaw_from_quaternion(ndt_pose.orientation) - initial_yaw)

    prior_cov = initial_msg.pose.covariance
    meas_cov = ndt_msg.pose.covariance
    prior_along_var = _axis_variance_from_xy_covariance(
        prior_cov, forward_x, forward_y, prior_xy_variance_floor_m2
    ) + process_noise[0]
    prior_cross_var = _axis_variance_from_xy_covariance(
        prior_cov, lateral_x, lateral_y, prior_xy_variance_floor_m2
    ) + process_noise[1]
    prior_yaw_var = max(float(prior_cov[35]), float(prior_yaw_variance_floor_rad2)) + process_noise[2]
    if not math.isfinite(prior_yaw_var) or prior_yaw_var <= 0.0:
        prior_yaw_var = float(prior_yaw_variance_floor_rad2) + process_noise[2]

    meas_along_floor = (
        measurement_xy_variance_floor_m2
        if measurement_along_variance_floor_m2 is None
        else measurement_along_variance_floor_m2
    )
    meas_cross_floor = (
        measurement_xy_variance_floor_m2
        if measurement_cross_variance_floor_m2 is None
        else measurement_cross_variance_floor_m2
    )
    meas_along_var = _axis_variance_from_xy_covariance(
        meas_cov, forward_x, forward_y, meas_along_floor
    )
    meas_cross_var = _axis_variance_from_xy_covariance(
        meas_cov, lateral_x, lateral_y, meas_cross_floor
    )
    meas_yaw_var = max(float(meas_cov[35]), float(measurement_yaw_variance_floor_rad2))
    if not math.isfinite(meas_yaw_var) or meas_yaw_var <= 0.0:
        meas_yaw_var = float(measurement_yaw_variance_floor_rad2)
    measurement_inflation = list(measurement_variance_inflation_diag or [0.0, 0.0, 0.0])
    while len(measurement_inflation) < 3:
        measurement_inflation.append(0.0)
    measurement_inflation = [max(0.0, float(value)) for value in measurement_inflation[:3]]
    meas_along_var += measurement_inflation[0]
    meas_cross_var += measurement_inflation[1]
    meas_yaw_var += measurement_inflation[2]

    innovation = [along, cross, yaw_delta]
    innovation_variance = [
        prior_along_var + meas_along_var,
        prior_cross_var + meas_cross_var,
        prior_yaw_var + meas_yaw_var,
    ]
    mahalanobis2 = sum(
        (value * value) / max(variance, 1e-12)
        for value, variance in zip(innovation, innovation_variance)
    )
    mahalanobis = math.sqrt(max(0.0, mahalanobis2))
    gate = float(mahalanobis_gate)
    gate_enabled = gate > 0.0
    if gate_enabled and mahalanobis > gate:
        return copy.deepcopy(initial_msg), {
            "accepted": False,
            "reason": "mahalanobis_reject",
            "clipped": False,
            "gate_enabled": True,
            "mahalanobis": mahalanobis,
            "mahalanobis_gate": gate,
            "innovation_along_m": along,
            "innovation_cross_m": cross,
            "innovation_yaw_deg": math.degrees(yaw_delta),
            "innovation_norm_m": math.hypot(along, cross),
            "innovation_variance_diag": innovation_variance,
            "process_noise_diag": process_noise,
            "ndt_covariance_source": _covariance_source_name(ndt_covariance_estimation_type),
            "prior_variance_diag": [prior_along_var, prior_cross_var, prior_yaw_var],
            "measurement_variance_diag": [meas_along_var, meas_cross_var, meas_yaw_var],
            "measurement_variance_inflation_diag": measurement_inflation,
            "applied_along_m": 0.0,
            "applied_cross_m": 0.0,
            "applied_yaw_deg": 0.0,
        }

    gains = [
        prior_along_var / max(innovation_variance[0], 1e-12),
        prior_cross_var / max(innovation_variance[1], 1e-12),
        prior_yaw_var / max(innovation_variance[2], 1e-12),
    ]
    applied_along = gains[0] * along
    applied_cross = gains[1] * cross
    applied_yaw = gains[2] * yaw_delta

    fused = copy.deepcopy(ndt_msg)
    fused_pose = fused.pose.pose
    fused_pose.position.x = (
        float(initial_pose.position.x)
        + forward_x * applied_along
        + lateral_x * applied_cross
    )
    fused_pose.position.y = (
        float(initial_pose.position.y)
        + forward_y * applied_along
        + lateral_y * applied_cross
    )
    fused_pose.position.z = float(initial_pose.position.z) + (
        float(ndt_pose.position.z) - float(initial_pose.position.z)
    ) * _clamp01(z_gain)
    initial_roll, initial_pitch, _ = _rpy_from_quaternion(initial_pose.orientation)
    ndt_roll, ndt_pitch, _ = _rpy_from_quaternion(ndt_pose.orientation)
    fused_pose.orientation = _rpy_to_quaternion(
        initial_roll + _normalize_angle(ndt_roll - initial_roll) * _clamp01(roll_pitch_gain),
        initial_pitch + _normalize_angle(ndt_pitch - initial_pitch) * _clamp01(roll_pitch_gain),
        _normalize_angle(initial_yaw + applied_yaw),
    )

    posterior_along_var = (1.0 - gains[0]) * prior_along_var
    posterior_cross_var = (1.0 - gains[1]) * prior_cross_var
    posterior_yaw_var = (1.0 - gains[2]) * prior_yaw_var
    fused.pose.covariance[0] = (
        posterior_along_var * forward_x * forward_x
        + posterior_cross_var * lateral_x * lateral_x
    )
    fused.pose.covariance[1] = (
        posterior_along_var * forward_x * forward_y
        + posterior_cross_var * lateral_x * lateral_y
    )
    fused.pose.covariance[6] = fused.pose.covariance[1]
    fused.pose.covariance[7] = (
        posterior_along_var * forward_y * forward_y
        + posterior_cross_var * lateral_y * lateral_y
    )
    fused.pose.covariance[35] = posterior_yaw_var

    return fused, {
        "accepted": True,
        "reason": "ekf_measurement_update",
        "clipped": False,
        "gate_enabled": gate_enabled,
        "mahalanobis": mahalanobis,
        "mahalanobis_gate": gate,
        "innovation_along_m": along,
        "innovation_cross_m": cross,
        "innovation_yaw_deg": math.degrees(yaw_delta),
        "innovation_norm_m": math.hypot(along, cross),
        "innovation_variance_diag": innovation_variance,
        "kalman_gain_diag": gains,
        "process_noise_diag": process_noise,
        "ndt_covariance_source": _covariance_source_name(ndt_covariance_estimation_type),
        "prior_variance_diag": [prior_along_var, prior_cross_var, prior_yaw_var],
        "measurement_variance_diag": [meas_along_var, meas_cross_var, meas_yaw_var],
        "measurement_variance_inflation_diag": measurement_inflation,
        "applied_along_m": applied_along,
        "applied_cross_m": applied_cross,
        "applied_yaw_deg": math.degrees(applied_yaw),
    }


def _robust_initial_pose_update(
    ndt_msg,
    initial_msg,
    *,
    along_gain=1.0,
    cross_gain=1.0,
    yaw_gain=1.0,
    z_gain=1.0,
    roll_pitch_gain=1.0,
    max_along_correction_m=0.35,
    max_cross_correction_m=0.35,
    max_yaw_correction_deg=3.0,
    hard_reject_correction_m=8.0,
    hard_reject_yaw_deg=25.0,
    covariance_inflation_scale=1.0,
):
    initial_pose = initial_msg.pose.pose
    ndt_pose = ndt_msg.pose.pose
    initial_yaw = _yaw_from_quaternion(initial_pose.orientation)
    forward_x = math.cos(initial_yaw)
    forward_y = math.sin(initial_yaw)
    lateral_x = -math.sin(initial_yaw)
    lateral_y = math.cos(initial_yaw)

    dx = float(ndt_pose.position.x) - float(initial_pose.position.x)
    dy = float(ndt_pose.position.y) - float(initial_pose.position.y)
    along = dx * forward_x + dy * forward_y
    cross = dx * lateral_x + dy * lateral_y
    yaw_delta = _normalize_angle(_yaw_from_quaternion(ndt_pose.orientation) - initial_yaw)
    correction_norm = math.hypot(along, cross)

    hard_reject = False
    if float(hard_reject_correction_m) > 0.0 and correction_norm > float(hard_reject_correction_m):
        hard_reject = True
    if float(hard_reject_yaw_deg) > 0.0 and abs(math.degrees(yaw_delta)) > float(
        hard_reject_yaw_deg
    ):
        hard_reject = True

    if hard_reject:
        decision = {
            "accepted": False,
            "reason": "hard_reject",
            "clipped": False,
            "innovation_along_m": along,
            "innovation_cross_m": cross,
            "innovation_yaw_deg": math.degrees(yaw_delta),
            "applied_along_m": 0.0,
            "applied_cross_m": 0.0,
            "applied_yaw_deg": 0.0,
            "innovation_norm_m": correction_norm,
        }
        return copy.deepcopy(initial_msg), decision

    scaled_along = along * _clamp01(along_gain)
    scaled_cross = cross * _clamp01(cross_gain)
    scaled_yaw = math.radians(math.degrees(yaw_delta) * _clamp01(yaw_gain))
    applied_along = _clip_axis(scaled_along, max_along_correction_m)
    applied_cross = _clip_axis(scaled_cross, max_cross_correction_m)
    applied_yaw = math.radians(_clip_axis(math.degrees(scaled_yaw), max_yaw_correction_deg))
    clipped = (
        abs(applied_along - scaled_along) > 1e-9
        or abs(applied_cross - scaled_cross) > 1e-9
        or abs(applied_yaw - scaled_yaw) > 1e-9
    )

    fused = copy.deepcopy(ndt_msg)
    fused_pose = fused.pose.pose
    fused_pose.position.x = (
        float(initial_pose.position.x)
        + forward_x * applied_along
        + lateral_x * applied_cross
    )
    fused_pose.position.y = (
        float(initial_pose.position.y)
        + forward_y * applied_along
        + lateral_y * applied_cross
    )
    fused_pose.position.z = float(initial_pose.position.z) + (
        float(ndt_pose.position.z) - float(initial_pose.position.z)
    ) * _clamp01(z_gain)
    initial_roll, initial_pitch, _ = _rpy_from_quaternion(initial_pose.orientation)
    ndt_roll, ndt_pitch, _ = _rpy_from_quaternion(ndt_pose.orientation)
    fused_pose.orientation = _rpy_to_quaternion(
        initial_roll + _normalize_angle(ndt_roll - initial_roll) * _clamp01(roll_pitch_gain),
        initial_pitch + _normalize_angle(ndt_pitch - initial_pitch) * _clamp01(roll_pitch_gain),
        _normalize_angle(initial_yaw + applied_yaw),
    )

    if clipped:
        inflation = max(
            1.0,
            abs(along) / max(abs(applied_along), 1e-3),
            abs(cross) / max(abs(applied_cross), 1e-3),
            abs(yaw_delta) / max(abs(applied_yaw), 1e-4),
        )
        inflation *= max(1.0, float(covariance_inflation_scale))
        fused.pose.covariance[0] = max(float(fused.pose.covariance[0]), 0.0225 * inflation)
        fused.pose.covariance[7] = max(float(fused.pose.covariance[7]), 0.0225 * inflation)
        fused.pose.covariance[35] = max(float(fused.pose.covariance[35]), 0.000625 * inflation)

    decision = {
        "accepted": True,
        "reason": "bounded_innovation",
        "clipped": clipped,
        "innovation_along_m": along,
        "innovation_cross_m": cross,
        "innovation_yaw_deg": math.degrees(yaw_delta),
        "applied_along_m": applied_along,
        "applied_cross_m": applied_cross,
        "applied_yaw_deg": math.degrees(applied_yaw),
        "innovation_norm_m": correction_norm,
        "along_gain": _clamp01(along_gain),
        "cross_gain": _clamp01(cross_gain),
        "yaw_gain": _clamp01(yaw_gain),
        "z_gain": _clamp01(z_gain),
        "roll_pitch_gain": _clamp01(roll_pitch_gain),
    }
    return fused, decision


def _prediction_fallback_due(
    *,
    first_accepted_seen,
    last_accepted_stamp,
    current_stamp,
    min_age_sec,
):
    if not first_accepted_seen or last_accepted_stamp is None:
        return False
    if float(min_age_sec) <= 0.0:
        return True
    age = (current_stamp - last_accepted_stamp).nanoseconds / 1e9
    return age >= float(min_age_sec)


def _make_prediction_fallback_msg(
    initial_msg,
    *,
    xy_variance_floor=4.0,
    yaw_variance_floor=0.25,
):
    fallback = copy.deepcopy(initial_msg)
    fallback.pose.covariance[0] = max(
        float(fallback.pose.covariance[0]), float(xy_variance_floor)
    )
    fallback.pose.covariance[7] = max(
        float(fallback.pose.covariance[7]), float(xy_variance_floor)
    )
    fallback.pose.covariance[35] = max(
        float(fallback.pose.covariance[35]), float(yaw_variance_floor)
    )
    return fallback


def _axis_basis(seed_msg):
    seed_yaw = _yaw_from_quaternion(seed_msg.pose.pose.orientation)
    forward_x = math.cos(seed_yaw)
    forward_y = math.sin(seed_yaw)
    lateral_x = -math.sin(seed_yaw)
    lateral_y = math.cos(seed_yaw)
    return seed_yaw, forward_x, forward_y, lateral_x, lateral_y


def _axis_projection(msg, seed_msg):
    _, forward_x, forward_y, lateral_x, lateral_y = _axis_basis(seed_msg)
    dx = float(msg.pose.pose.position.x) - float(seed_msg.pose.pose.position.x)
    dy = float(msg.pose.pose.position.y) - float(seed_msg.pose.pose.position.y)
    return {
        "along": dx * forward_x + dy * forward_y,
        "cross": dx * lateral_x + dy * lateral_y,
        "yaw": _yaw_from_quaternion(msg.pose.pose.orientation),
    }


def _temporal_filter_axis_pose(
    current_msg,
    seed_msg,
    previous_msg,
    *,
    lateral_alpha=1.0,
    yaw_alpha=1.0,
    mahalanobis_gate=0.0,
    lateral_innovation_stddev_m=0.5,
    yaw_innovation_stddev_rad=0.1,
):
    if previous_msg is None:
        return copy.deepcopy(current_msg), {"rejected": False, "mahalanobis": None}

    current = _axis_projection(current_msg, seed_msg)
    previous = _axis_projection(previous_msg, seed_msg)
    cross_delta = current["cross"] - previous["cross"]
    yaw_delta = _normalize_angle(current["yaw"] - previous["yaw"])

    mahalanobis = None
    rejected = False
    if float(mahalanobis_gate) > 0.0:
        lateral_sigma = max(float(lateral_innovation_stddev_m), 1e-6)
        yaw_sigma = max(float(yaw_innovation_stddev_rad), 1e-6)
        mahalanobis = math.sqrt((cross_delta / lateral_sigma) ** 2 + (yaw_delta / yaw_sigma) ** 2)
        rejected = mahalanobis > float(mahalanobis_gate)

    if rejected:
        target_cross = previous["cross"]
        target_yaw = previous["yaw"]
    else:
        target_cross = previous["cross"] + _clamp01(lateral_alpha) * cross_delta
        target_yaw = _normalize_angle(previous["yaw"] + _clamp01(yaw_alpha) * yaw_delta)

    filtered = copy.deepcopy(current_msg)
    _, forward_x, forward_y, lateral_x, lateral_y = _axis_basis(seed_msg)
    seed_pose = seed_msg.pose.pose
    filtered.pose.pose.position.x = (
        float(seed_pose.position.x) + forward_x * current["along"] + lateral_x * target_cross
    )
    filtered.pose.pose.position.y = (
        float(seed_pose.position.y) + forward_y * current["along"] + lateral_y * target_cross
    )
    roll, pitch, _ = _rpy_from_quaternion(current_msg.pose.pose.orientation)
    filtered.pose.pose.orientation = _rpy_to_quaternion(roll, pitch, target_yaw)
    return filtered, {"rejected": rejected, "mahalanobis": mahalanobis}


def _apply_body_frame_position_bias(msg, *, along_bias_m=0.0, cross_bias_m=0.0):
    along_bias_m = float(along_bias_m)
    cross_bias_m = float(cross_bias_m)
    if abs(along_bias_m) < 1e-12 and abs(cross_bias_m) < 1e-12:
        return msg
    shifted = copy.deepcopy(msg)
    pose = shifted.pose.pose
    yaw = _yaw_from_quaternion(pose.orientation)
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    lateral_x = -math.sin(yaw)
    lateral_y = math.cos(yaw)
    pose.position.x = float(pose.position.x) + forward_x * along_bias_m + lateral_x * cross_bias_m
    pose.position.y = float(pose.position.y) + forward_y * along_bias_m + lateral_y * cross_bias_m
    return shifted


class NdtAxisSeedFuser(Node):
    def __init__(self):
        super().__init__("ndt_axis_seed_fuser")
        self.declare_parameter("raw_ndt_pose_topic", "/localization/ndt/raw_pose_with_covariance")
        self.declare_parameter("seed_pose_topic", "/localization/fixposition/seed_pose")
        self.declare_parameter("output_topic", "/localization/pose_with_covariance")
        self.declare_parameter("initial_pose_topic", "")
        self.declare_parameter("prediction_pose_topic", "")
        self.declare_parameter("max_initial_pose_age_sec", 0.2)
        self.declare_parameter("max_ndt_initial_distance_m", 0.0)
        self.declare_parameter("max_ndt_initial_yaw_delta_deg", 0.0)
        self.declare_parameter("initial_pose_correction_gain", 1.0)
        self.declare_parameter("initial_pose_along_correction_gain", -1.0)
        self.declare_parameter("initial_pose_cross_correction_gain", -1.0)
        self.declare_parameter("initial_pose_yaw_correction_gain", -1.0)
        self.declare_parameter("max_seed_age_sec", 0.5)
        self.declare_parameter("seed_history_duration_sec", 10.0)
        self.declare_parameter("seed_history_max_samples", 20000)
        self.declare_parameter("max_seed_xy_stddev_m", 0.75)
        self.declare_parameter("fusion_mode", "seed_cross_yaw")
        self.declare_parameter("lateral_gain", 1.0)
        self.declare_parameter("along_gain", 0.03)
        self.declare_parameter("max_seed_along_residual_m", 0.0)
        self.declare_parameter("yaw_deadband_sigma", 3.0)
        self.declare_parameter("enable_temporal_filter", False)
        self.declare_parameter("temporal_lateral_alpha", 1.0)
        self.declare_parameter("temporal_yaw_alpha", 1.0)
        self.declare_parameter("temporal_mahalanobis_gate", 0.0)
        self.declare_parameter("temporal_lateral_innovation_stddev_m", 0.5)
        self.declare_parameter("temporal_yaw_innovation_stddev_rad", 0.1)
        self.declare_parameter("enable_robust_initial_update", False)
        self.declare_parameter("robust_update_mode", "ekf")
        self.declare_parameter("robust_mahalanobis_gate", 4.0)
        self.declare_parameter("robust_ndt_covariance_estimation_type", 1)
        self.declare_parameter("robust_prior_xy_variance_floor_m2", 0.25)
        self.declare_parameter("robust_prior_yaw_variance_floor_rad2", 0.007615435494667714)
        self.declare_parameter("robust_measurement_xy_variance_floor_m2", 0.09)
        self.declare_parameter("robust_measurement_along_variance_floor_m2", -1.0)
        self.declare_parameter("robust_measurement_cross_variance_floor_m2", -1.0)
        self.declare_parameter("robust_measurement_yaw_variance_floor_rad2", 0.00030461741978670857)
        self.declare_parameter("robust_along_gain", 1.0)
        self.declare_parameter("robust_cross_gain", 1.0)
        self.declare_parameter("robust_yaw_gain", 1.0)
        self.declare_parameter("robust_z_gain", 1.0)
        self.declare_parameter("robust_roll_pitch_gain", 1.0)
        self.declare_parameter("robust_max_along_correction_m", 0.35)
        self.declare_parameter("robust_max_cross_correction_m", 0.35)
        self.declare_parameter("robust_max_yaw_correction_deg", 3.0)
        self.declare_parameter("robust_hard_reject_correction_m", 8.0)
        self.declare_parameter("robust_hard_reject_yaw_deg", 25.0)
        self.declare_parameter("robust_covariance_inflation_scale", 1.0)
        self.declare_parameter("robust_decision_topic", "/localization/robust_ndt/decision")
        self.declare_parameter("runtime_multistart_decision_topic", "")
        self.declare_parameter("runtime_multistart_decision_max_age_sec", 0.2)
        self.declare_parameter("robust_candidate_spread_min_candidate_count", 2)
        self.declare_parameter("robust_candidate_spread_along_threshold_m", 1.5)
        self.declare_parameter("robust_candidate_spread_along_variance_scale", 0.5)
        self.declare_parameter("robust_candidate_spread_max_abs_along_m", 3.0)
        self.declare_parameter("robust_candidate_spread_max_abs_cross_m", 0.85)
        self.declare_parameter("robust_candidate_spread_score_margin", 0.0)
        self.declare_parameter("robust_candidate_spread_yaw_threshold_deg", 3.0)
        self.declare_parameter("robust_candidate_spread_yaw_variance_scale", 0.5)
        self.declare_parameter("predictor_update_topic", "")
        self.declare_parameter("predictor_update_requires_robust_high_confidence", False)
        self.declare_parameter("predictor_update_high_confidence_min_stamp_sec", 0.0)
        self.declare_parameter("predictor_update_max_mahalanobis", 0.0)
        self.declare_parameter("predictor_update_max_innovation_along_m", 0.0)
        self.declare_parameter("predictor_update_max_innovation_cross_m", 0.0)
        self.declare_parameter("predictor_update_max_innovation_yaw_deg", 0.0)
        self.declare_parameter("enable_prediction_fallback", False)
        self.declare_parameter("prediction_fallback_min_age_sec", 0.25)
        self.declare_parameter("prediction_fallback_xy_variance_floor_m2", 4.0)
        self.declare_parameter("prediction_fallback_yaw_variance_floor_rad2", 0.25)
        self.declare_parameter("output_along_bias_m", 0.0)
        self.declare_parameter("output_cross_bias_m", 0.0)

        self._max_seed_age = float(self.get_parameter("max_seed_age_sec").value)
        self._seed_history_duration = max(
            0.0, float(self.get_parameter("seed_history_duration_sec").value)
        )
        self._seed_history_max_samples = max(
            1, int(self.get_parameter("seed_history_max_samples").value)
        )
        self._max_seed_xy_stddev = float(self.get_parameter("max_seed_xy_stddev_m").value)
        self._initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self._prediction_pose_topic = str(self.get_parameter("prediction_pose_topic").value)
        self._max_initial_pose_age = float(self.get_parameter("max_initial_pose_age_sec").value)
        self._max_ndt_initial_distance = float(
            self.get_parameter("max_ndt_initial_distance_m").value
        )
        self._max_ndt_initial_yaw_delta = float(
            self.get_parameter("max_ndt_initial_yaw_delta_deg").value
        )
        self._initial_pose_correction_gain = float(
            self.get_parameter("initial_pose_correction_gain").value
        )
        self._initial_pose_along_correction_gain = float(
            self.get_parameter("initial_pose_along_correction_gain").value
        )
        self._initial_pose_cross_correction_gain = float(
            self.get_parameter("initial_pose_cross_correction_gain").value
        )
        self._initial_pose_yaw_correction_gain = float(
            self.get_parameter("initial_pose_yaw_correction_gain").value
        )
        self._fusion_mode = str(self.get_parameter("fusion_mode").value)
        self._lateral_gain = float(self.get_parameter("lateral_gain").value)
        self._along_gain = float(self.get_parameter("along_gain").value)
        self._max_seed_along_residual = float(
            self.get_parameter("max_seed_along_residual_m").value
        )
        self._yaw_deadband_sigma = float(self.get_parameter("yaw_deadband_sigma").value)
        self._enable_temporal_filter = bool(self.get_parameter("enable_temporal_filter").value)
        self._temporal_lateral_alpha = float(self.get_parameter("temporal_lateral_alpha").value)
        self._temporal_yaw_alpha = float(self.get_parameter("temporal_yaw_alpha").value)
        self._temporal_mahalanobis_gate = float(self.get_parameter("temporal_mahalanobis_gate").value)
        self._temporal_lateral_innovation_stddev = float(
            self.get_parameter("temporal_lateral_innovation_stddev_m").value
        )
        self._temporal_yaw_innovation_stddev = float(
            self.get_parameter("temporal_yaw_innovation_stddev_rad").value
        )
        self._enable_robust_initial_update = bool(
            self.get_parameter("enable_robust_initial_update").value
        )
        self._robust_update_mode = str(self.get_parameter("robust_update_mode").value)
        self._robust_mahalanobis_gate = float(
            self.get_parameter("robust_mahalanobis_gate").value
        )
        self._robust_ndt_covariance_estimation_type = int(
            self.get_parameter("robust_ndt_covariance_estimation_type").value
        )
        self._robust_prior_xy_variance_floor = float(
            self.get_parameter("robust_prior_xy_variance_floor_m2").value
        )
        self._robust_prior_yaw_variance_floor = float(
            self.get_parameter("robust_prior_yaw_variance_floor_rad2").value
        )
        self._robust_measurement_xy_variance_floor = float(
            self.get_parameter("robust_measurement_xy_variance_floor_m2").value
        )
        self._robust_measurement_along_variance_floor = float(
            self.get_parameter("robust_measurement_along_variance_floor_m2").value
        )
        self._robust_measurement_cross_variance_floor = float(
            self.get_parameter("robust_measurement_cross_variance_floor_m2").value
        )
        self._robust_measurement_yaw_variance_floor = float(
            self.get_parameter("robust_measurement_yaw_variance_floor_rad2").value
        )
        self._robust_along_gain = float(self.get_parameter("robust_along_gain").value)
        self._robust_cross_gain = float(self.get_parameter("robust_cross_gain").value)
        self._robust_yaw_gain = float(self.get_parameter("robust_yaw_gain").value)
        self._robust_z_gain = float(self.get_parameter("robust_z_gain").value)
        self._robust_roll_pitch_gain = float(
            self.get_parameter("robust_roll_pitch_gain").value
        )
        self._robust_max_along_correction = float(
            self.get_parameter("robust_max_along_correction_m").value
        )
        self._robust_max_cross_correction = float(
            self.get_parameter("robust_max_cross_correction_m").value
        )
        self._robust_max_yaw_correction_deg = float(
            self.get_parameter("robust_max_yaw_correction_deg").value
        )
        self._robust_hard_reject_correction = float(
            self.get_parameter("robust_hard_reject_correction_m").value
        )
        self._robust_hard_reject_yaw_deg = float(
            self.get_parameter("robust_hard_reject_yaw_deg").value
        )
        self._robust_covariance_inflation_scale = float(
            self.get_parameter("robust_covariance_inflation_scale").value
        )
        self._robust_decision_topic = str(self.get_parameter("robust_decision_topic").value)
        self._runtime_multistart_decision_topic = str(
            self.get_parameter("runtime_multistart_decision_topic").value
        )
        self._runtime_multistart_decision_max_age = float(
            self.get_parameter("runtime_multistart_decision_max_age_sec").value
        )
        self._robust_candidate_spread_min_candidate_count = int(
            self.get_parameter("robust_candidate_spread_min_candidate_count").value
        )
        self._robust_candidate_spread_along_threshold = float(
            self.get_parameter("robust_candidate_spread_along_threshold_m").value
        )
        self._robust_candidate_spread_along_variance_scale = float(
            self.get_parameter("robust_candidate_spread_along_variance_scale").value
        )
        self._robust_candidate_spread_max_abs_along = float(
            self.get_parameter("robust_candidate_spread_max_abs_along_m").value
        )
        self._robust_candidate_spread_max_abs_cross = float(
            self.get_parameter("robust_candidate_spread_max_abs_cross_m").value
        )
        self._robust_candidate_spread_score_margin = float(
            self.get_parameter("robust_candidate_spread_score_margin").value
        )
        self._robust_candidate_spread_yaw_threshold = float(
            self.get_parameter("robust_candidate_spread_yaw_threshold_deg").value
        )
        self._robust_candidate_spread_yaw_variance_scale = float(
            self.get_parameter("robust_candidate_spread_yaw_variance_scale").value
        )
        self._predictor_update_topic = str(self.get_parameter("predictor_update_topic").value)
        self._predictor_update_requires_robust_high_confidence = bool(
            self.get_parameter("predictor_update_requires_robust_high_confidence").value
        )
        self._predictor_update_high_confidence_min_stamp_sec = float(
            self.get_parameter("predictor_update_high_confidence_min_stamp_sec").value
        )
        self._predictor_update_max_mahalanobis = float(
            self.get_parameter("predictor_update_max_mahalanobis").value
        )
        self._predictor_update_max_innovation_along = float(
            self.get_parameter("predictor_update_max_innovation_along_m").value
        )
        self._predictor_update_max_innovation_cross = float(
            self.get_parameter("predictor_update_max_innovation_cross_m").value
        )
        self._predictor_update_max_innovation_yaw_deg = float(
            self.get_parameter("predictor_update_max_innovation_yaw_deg").value
        )
        self._enable_prediction_fallback = bool(
            self.get_parameter("enable_prediction_fallback").value
        )
        self._prediction_fallback_min_age = float(
            self.get_parameter("prediction_fallback_min_age_sec").value
        )
        self._output_along_bias_m = float(self.get_parameter("output_along_bias_m").value)
        self._output_cross_bias_m = float(self.get_parameter("output_cross_bias_m").value)
        self._prediction_fallback_xy_variance_floor = float(
            self.get_parameter("prediction_fallback_xy_variance_floor_m2").value
        )
        self._prediction_fallback_yaw_variance_floor = float(
            self.get_parameter("prediction_fallback_yaw_variance_floor_rad2").value
        )
        self._last_seed = None
        self._last_seed_stamp = None
        self._seed_history = deque()
        self._last_initial_pose = None
        self._last_initial_stamp = None
        self._last_prediction_pose = None
        self._last_prediction_stamp = None
        self._last_runtime_multistart_decision = None
        self._initial_pose_buffer = []
        self._max_initial_pose_buffer_size = 2000
        self._initial_consistency_rejected_count = 0
        self._robust_accepted_count = 0
        self._robust_rejected_count = 0
        self._robust_clipped_count = 0
        self._accepted_update_count = 0
        self._prediction_fallback_count = 0
        self._last_accepted_update_stamp = None
        self._last_prediction_fallback_stamp = None
        self._last_output = None
        self._last_predictor_update_allowed = True

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_topic").value,
            10,
        )
        self._decision_publisher = (
            self.create_publisher(String, self._robust_decision_topic, 10)
            if self._robust_decision_topic
            else None
        )
        self._predictor_update_publisher = (
            self.create_publisher(PoseWithCovarianceStamped, self._predictor_update_topic, 10)
            if self._predictor_update_topic
            else None
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("seed_pose_topic").value,
            self._on_seed,
            10,
        )
        if self._initial_pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._initial_pose_topic,
                self._on_initial_pose,
                10,
            )
        if self._prediction_pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._prediction_pose_topic,
                self._on_prediction_pose,
                10,
            )
        if self._runtime_multistart_decision_topic:
            self.create_subscription(
                String,
                self._runtime_multistart_decision_topic,
                self._on_runtime_multistart_decision,
                10,
            )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("raw_ndt_pose_topic").value,
            self._on_ndt,
            10,
        )
        self.get_logger().info(
            f"Publishing axis-fused NDT pose on {self.get_parameter('output_topic').value}"
        )

    def _on_seed(self, msg):
        self._last_seed = msg
        self._last_seed_stamp = _message_time(msg, self.get_clock().now())
        self._seed_history.append((self._last_seed_stamp, msg))
        self._prune_seed_history(self._last_seed_stamp)

    def _on_initial_pose(self, msg):
        self._last_initial_pose = msg
        self._last_initial_stamp = _message_time(msg, self.get_clock().now())
        self._store_initial_pose(msg, self._last_initial_stamp)
        self._publish_prediction_fallback_if_due(self._fallback_prediction_msg(msg), self._last_initial_stamp)

    def _on_prediction_pose(self, msg):
        self._last_prediction_pose = msg
        self._last_prediction_stamp = _message_time(msg, self.get_clock().now())

    def _on_runtime_multistart_decision(self, msg):
        try:
            decision = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(decision, dict):
            self._last_runtime_multistart_decision = decision

    def _runtime_measurement_inflation_for_stamp(self, stamp):
        metadata = {
            "runtime_multistart_decision_age_sec": None,
            "runtime_candidate_spread_count": 0,
            "runtime_candidate_along_spread_m": 0.0,
            "runtime_candidate_yaw_spread_deg": 0.0,
            "runtime_candidate_spread_inflation": False,
        }
        decision = self._last_runtime_multistart_decision
        if not isinstance(decision, dict):
            return [0.0, 0.0, 0.0], metadata
        try:
            decision_stamp = float(decision.get("stamp_sec"))
        except (TypeError, ValueError):
            return [0.0, 0.0, 0.0], metadata
        stamp_sec = stamp.nanoseconds / 1e9
        age = abs(stamp_sec - decision_stamp)
        metadata["runtime_multistart_decision_age_sec"] = age
        if self._runtime_multistart_decision_max_age > 0.0 and (
            age > self._runtime_multistart_decision_max_age
        ):
            return [0.0, 0.0, 0.0], metadata
        inflation, spread_metadata = _runtime_candidate_spread_variance_inflation(
            decision,
            min_candidate_count=self._robust_candidate_spread_min_candidate_count,
            along_spread_threshold_m=self._robust_candidate_spread_along_threshold,
            along_variance_scale=self._robust_candidate_spread_along_variance_scale,
            max_abs_along_m=self._robust_candidate_spread_max_abs_along,
            max_abs_cross_m=self._robust_candidate_spread_max_abs_cross,
            selected_score_margin=self._robust_candidate_spread_score_margin,
            yaw_spread_threshold_deg=self._robust_candidate_spread_yaw_threshold,
            yaw_variance_scale=self._robust_candidate_spread_yaw_variance_scale,
        )
        metadata.update(spread_metadata)
        return inflation, metadata

    def _runtime_predictor_update_suppression_reason(self, stamp):
        decision = self._last_runtime_multistart_decision
        if not isinstance(decision, dict):
            return ""
        try:
            decision_stamp = float(decision.get("stamp_sec"))
        except (TypeError, ValueError):
            return ""
        stamp_sec = stamp.nanoseconds / 1e9
        age = abs(stamp_sec - decision_stamp)
        if self._runtime_multistart_decision_max_age > 0.0 and (
            age > self._runtime_multistart_decision_max_age
        ):
            return ""
        if bool(decision.get("recovery_active", False)):
            return "runtime_recovery"
        if bool(decision.get("tier2_evaluated", False)):
            return "runtime_tier2"
        if bool(decision.get("small_tier_ambiguous", False)):
            return "runtime_ambiguous"
        if not bool(decision.get("has_selected_candidate", True)):
            return "runtime_no_selected_candidate"
        return ""

    @staticmethod
    def _decision_abs_float(decision, key):
        try:
            value = abs(float(decision.get(key, 0.0)))
        except (TypeError, ValueError):
            return 0.0
        return value if math.isfinite(value) else math.inf

    def _robust_predictor_update_suppression_reason(self, decision, stamp):
        if not self._predictor_update_requires_robust_high_confidence:
            return ""
        if (
            self._predictor_update_high_confidence_min_stamp_sec > 0.0
            and stamp.nanoseconds * 1e-9
            < self._predictor_update_high_confidence_min_stamp_sec
        ):
            return ""
        if not bool(decision.get("accepted", False)):
            return "robust_reject"
        if str(decision.get("reason", "")) not in {
            "ekf_measurement_update",
            "bounded_innovation",
        }:
            return "robust_reason"
        if (
            self._predictor_update_max_mahalanobis > 0.0
            and self._decision_abs_float(decision, "mahalanobis")
            > self._predictor_update_max_mahalanobis
        ):
            return "robust_mahalanobis"
        if (
            self._predictor_update_max_innovation_along > 0.0
            and self._decision_abs_float(decision, "innovation_along_m")
            > self._predictor_update_max_innovation_along
        ):
            return "robust_along"
        if (
            self._predictor_update_max_innovation_cross > 0.0
            and self._decision_abs_float(decision, "innovation_cross_m")
            > self._predictor_update_max_innovation_cross
        ):
            return "robust_cross"
        if (
            self._predictor_update_max_innovation_yaw_deg > 0.0
            and self._decision_abs_float(decision, "innovation_yaw_deg")
            > self._predictor_update_max_innovation_yaw_deg
        ):
            return "robust_yaw"
        return self._runtime_predictor_update_suppression_reason(stamp)

    def _unverified_runtime_recovery_reject_metadata(self, stamp):
        decision = self._last_runtime_multistart_decision
        if not isinstance(decision, dict):
            return None
        try:
            decision_stamp = float(decision.get("stamp_sec"))
        except (TypeError, ValueError):
            return None
        stamp_sec = stamp.nanoseconds / 1e9
        age = abs(stamp_sec - decision_stamp)
        if self._runtime_multistart_decision_max_age > 0.0 and (
            age > self._runtime_multistart_decision_max_age
        ):
            return None
        recovery_context = bool(decision.get("tier2_evaluated", False)) or bool(
            decision.get("recovery_active", False)
        )
        if not recovery_context or bool(decision.get("recovery_verified", False)):
            return None
        try:
            selected_index = int(decision.get("selected_candidate_index"))
        except (TypeError, ValueError):
            return None
        selected_tier = ""
        for candidate in decision.get("candidates") or []:
            if not isinstance(candidate, dict):
                continue
            try:
                if int(candidate.get("index")) != selected_index:
                    continue
            except (TypeError, ValueError):
                continue
            selected_tier = str(candidate.get("tier", ""))
            break
        if selected_tier != "far":
            return None
        return {
            "runtime_multistart_decision_age_sec": age,
            "runtime_recovery_context": True,
            "runtime_recovery_verified": False,
            "runtime_tier2_evaluated": bool(decision.get("tier2_evaluated", False)),
            "runtime_recovery_active": bool(decision.get("recovery_active", False)),
            "runtime_selected_candidate_index": selected_index,
            "runtime_selected_candidate_tier": selected_tier,
        }

    def _verified_runtime_recovery_metadata(self, stamp):
        decision = self._last_runtime_multistart_decision
        if not isinstance(decision, dict) or not bool(decision.get("recovery_verified", False)):
            return None
        try:
            decision_stamp = float(decision.get("stamp_sec"))
        except (TypeError, ValueError):
            return None
        stamp_sec = stamp.nanoseconds / 1e9
        age = abs(stamp_sec - decision_stamp)
        if self._runtime_multistart_decision_max_age > 0.0 and (
            age > self._runtime_multistart_decision_max_age
        ):
            return None
        try:
            selected_index = int(decision.get("selected_candidate_index"))
        except (TypeError, ValueError):
            return None
        selected_tier = ""
        for candidate in decision.get("candidates") or []:
            if not isinstance(candidate, dict):
                continue
            try:
                if int(candidate.get("index")) != selected_index:
                    continue
            except (TypeError, ValueError):
                continue
            selected_tier = str(candidate.get("tier", ""))
            break
        if selected_tier != "far":
            return None
        return {
            "runtime_multistart_decision_age_sec": age,
            "runtime_recovery_context": True,
            "runtime_recovery_verified": True,
            "runtime_tier2_evaluated": bool(decision.get("tier2_evaluated", False)),
            "runtime_recovery_active": bool(decision.get("recovery_active", False)),
            "runtime_selected_candidate_index": selected_index,
            "runtime_selected_candidate_tier": selected_tier,
            "runtime_recovery_verified_stable_frames": int(
                decision.get("recovery_verified_stable_frames", 0) or 0
            ),
        }

    def _fallback_prediction_msg(self, initial_msg):
        if self._last_prediction_pose is None or self._last_prediction_stamp is None:
            return initial_msg
        initial_stamp = _message_time(initial_msg, self.get_clock().now())
        age = abs((initial_stamp - self._last_prediction_stamp).nanoseconds / 1e9)
        if self._max_initial_pose_age > 0.0 and age > self._max_initial_pose_age:
            return initial_msg
        return self._last_prediction_pose

    def _store_initial_pose(self, msg, stamp):
        sample = (stamp.nanoseconds, msg, stamp)
        if not self._initial_pose_buffer or sample[0] >= self._initial_pose_buffer[-1][0]:
            self._initial_pose_buffer.append(sample)
        else:
            index = 0
            while index < len(self._initial_pose_buffer) and (
                self._initial_pose_buffer[index][0] <= sample[0]
            ):
                index += 1
            self._initial_pose_buffer.insert(index, sample)
        if len(self._initial_pose_buffer) > self._max_initial_pose_buffer_size:
            self._initial_pose_buffer = self._initial_pose_buffer[-self._max_initial_pose_buffer_size :]

    def _initial_pose_for_stamp(self, stamp):
        if not self._initial_pose_buffer:
            return None
        stamp_ns = stamp.nanoseconds
        best = min(self._initial_pose_buffer, key=lambda item: abs(item[0] - stamp_ns))
        age = abs((stamp - best[2]).nanoseconds / 1e9)
        if self._max_initial_pose_age > 0.0 and age > self._max_initial_pose_age:
            return None
        return best[1]

    def _ndt_passes_initial_consistency(self, msg, stamp):
        if self._max_ndt_initial_distance <= 0.0 and self._max_ndt_initial_yaw_delta <= 0.0:
            return True
        initial_pose = self._initial_pose_for_stamp(stamp)
        if initial_pose is None:
            return True
        ok, details = _ndt_is_consistent_with_initial_pose(
            msg,
            initial_pose,
            max_distance_m=self._max_ndt_initial_distance,
            max_yaw_delta_deg=self._max_ndt_initial_yaw_delta,
        )
        if ok:
            return True
        self._initial_consistency_rejected_count += 1
        self.get_logger().warn(
            "Rejecting NDT pose inconsistent with dead-reckon initial pose: "
            f"reason={details['reason']} distance={details['distance_m']:.2f}m "
            f"yaw_delta={details['yaw_delta_deg']:.2f}deg "
            f"count={self._initial_consistency_rejected_count}",
            throttle_duration_sec=1.0,
        )
        return False

    def _apply_initial_correction_gain_if_available(self, msg, stamp):
        axis_gains = (
            self._initial_pose_along_correction_gain,
            self._initial_pose_cross_correction_gain,
            self._initial_pose_yaw_correction_gain,
        )
        axis_gain_enabled = any(gain >= 0.0 for gain in axis_gains)
        if self._initial_pose_correction_gain >= 1.0 and not axis_gain_enabled:
            return msg
        initial_pose = self._initial_pose_for_stamp(stamp)
        if initial_pose is None:
            return msg
        if not axis_gain_enabled:
            return _apply_initial_pose_correction_gain(
                msg,
                initial_pose,
                correction_gain=self._initial_pose_correction_gain,
            )
        fallback_gain = _clamp01(self._initial_pose_correction_gain)
        along_gain = (
            fallback_gain
            if self._initial_pose_along_correction_gain < 0.0
            else self._initial_pose_along_correction_gain
        )
        cross_gain = (
            fallback_gain
            if self._initial_pose_cross_correction_gain < 0.0
            else self._initial_pose_cross_correction_gain
        )
        yaw_gain = (
            fallback_gain
            if self._initial_pose_yaw_correction_gain < 0.0
            else self._initial_pose_yaw_correction_gain
        )
        return _apply_initial_pose_axis_correction_gain(
            msg,
            initial_pose,
            along_gain=along_gain,
            cross_gain=cross_gain,
            yaw_gain=yaw_gain,
        )

    def _apply_robust_initial_update_if_available(self, msg, stamp):
        if not self._enable_robust_initial_update:
            return msg, True
        initial_pose = self._initial_pose_for_stamp(stamp)
        if initial_pose is None:
            decision = {
                "accepted": True,
                "reason": "no_time_matched_initial_pose",
                "clipped": False,
                "gate_enabled": self._robust_mahalanobis_gate > 0.0,
                "mahalanobis": 0.0,
                "mahalanobis_gate": self._robust_mahalanobis_gate,
                "ndt_covariance_source": _covariance_source_name(
                    self._robust_ndt_covariance_estimation_type
                ),
                "process_noise_diag": [
                    self._process_noise_xy(),
                    self._process_noise_xy(),
                    self._process_noise_yaw(),
                ],
            }
            suppression_reason = self._robust_predictor_update_suppression_reason(
                decision, stamp
            )
            self._last_predictor_update_allowed = not suppression_reason
            decision["predictor_update_allowed"] = self._last_predictor_update_allowed
            decision["predictor_update_suppressed_reason"] = suppression_reason
            self._publish_decision(decision, stamp)
            return msg, True
        if self._robust_update_mode.lower() == "bounded":
            fused, decision = _robust_initial_pose_update(
                msg,
                initial_pose,
                along_gain=self._robust_along_gain,
                cross_gain=self._robust_cross_gain,
                yaw_gain=self._robust_yaw_gain,
                z_gain=self._robust_z_gain,
                roll_pitch_gain=self._robust_roll_pitch_gain,
                max_along_correction_m=self._robust_max_along_correction,
                max_cross_correction_m=self._robust_max_cross_correction,
                max_yaw_correction_deg=self._robust_max_yaw_correction_deg,
                hard_reject_correction_m=self._robust_hard_reject_correction,
                hard_reject_yaw_deg=self._robust_hard_reject_yaw_deg,
                covariance_inflation_scale=self._robust_covariance_inflation_scale,
            )
            decision.setdefault("gate_enabled", False)
            decision.setdefault("mahalanobis_gate", 0.0)
            decision.setdefault("ndt_covariance_source", "bounded_without_covariance")
            decision.setdefault("process_noise_diag", [])
        else:
            verified_recovery_metadata = self._verified_runtime_recovery_metadata(stamp)
            if verified_recovery_metadata is not None:
                initial = initial_pose.pose.pose
                measured = msg.pose.pose
                initial_yaw = _yaw_from_quaternion(initial.orientation)
                forward_x = math.cos(initial_yaw)
                forward_y = math.sin(initial_yaw)
                lateral_x = -math.sin(initial_yaw)
                lateral_y = math.cos(initial_yaw)
                dx = float(measured.position.x) - float(initial.position.x)
                dy = float(measured.position.y) - float(initial.position.y)
                innovation_yaw_deg = math.degrees(
                    _normalize_angle(_yaw_from_quaternion(measured.orientation) - initial_yaw)
                )
                decision = {
                    "accepted": True,
                    "reason": "runtime_verified_recovery_reset",
                    "clipped": False,
                    "gate_enabled": self._robust_mahalanobis_gate > 0.0,
                    "mahalanobis": 0.0,
                    "mahalanobis_gate": self._robust_mahalanobis_gate,
                    "ndt_covariance_source": _covariance_source_name(
                        self._robust_ndt_covariance_estimation_type
                    ),
                    "process_noise_diag": [
                        self._process_noise_xy(),
                        self._process_noise_xy(),
                        self._process_noise_yaw(),
                    ],
                    "innovation_norm_m": math.hypot(dx, dy),
                    "innovation_along_m": dx * forward_x + dy * forward_y,
                    "innovation_cross_m": dx * lateral_x + dy * lateral_y,
                    "innovation_yaw_deg": innovation_yaw_deg,
                    "applied_along_m": dx * forward_x + dy * forward_y,
                    "applied_cross_m": dx * lateral_x + dy * lateral_y,
                    "applied_yaw_deg": innovation_yaw_deg,
                }
                decision.update(verified_recovery_metadata)
                suppression_reason = self._robust_predictor_update_suppression_reason(
                    decision, stamp
                )
                self._last_predictor_update_allowed = not suppression_reason
                decision["predictor_update_allowed"] = self._last_predictor_update_allowed
                decision["predictor_update_suppressed_reason"] = suppression_reason
                decision["stamp_sec"] = stamp.nanoseconds / 1e9
                self._robust_accepted_count += 1
                decision["accepted_count"] = self._robust_accepted_count
                decision["rejected_count"] = self._robust_rejected_count
                decision["clipped_count"] = self._robust_clipped_count
                self._publish_decision(decision, stamp)
                return msg, True
            measurement_inflation, spread_metadata = self._runtime_measurement_inflation_for_stamp(
                stamp
            )
            fused, decision = _ekf_initial_pose_update(
                msg,
                initial_pose,
                mahalanobis_gate=self._robust_mahalanobis_gate,
                ndt_covariance_estimation_type=self._robust_ndt_covariance_estimation_type,
                process_noise_diag=[
                    self._process_noise_xy(),
                    self._process_noise_xy(),
                    self._process_noise_yaw(),
                ],
                prior_xy_variance_floor_m2=self._robust_prior_xy_variance_floor,
                prior_yaw_variance_floor_rad2=self._robust_prior_yaw_variance_floor,
                measurement_xy_variance_floor_m2=self._robust_measurement_xy_variance_floor,
                measurement_along_variance_floor_m2=(
                    None
                    if self._robust_measurement_along_variance_floor < 0.0
                    else self._robust_measurement_along_variance_floor
                ),
                measurement_cross_variance_floor_m2=(
                    None
                    if self._robust_measurement_cross_variance_floor < 0.0
                    else self._robust_measurement_cross_variance_floor
                ),
                measurement_yaw_variance_floor_rad2=self._robust_measurement_yaw_variance_floor,
                measurement_variance_inflation_diag=measurement_inflation,
                z_gain=self._robust_z_gain,
                roll_pitch_gain=self._robust_roll_pitch_gain,
            )
            decision.update(spread_metadata)
        recovery_reject_metadata = self._unverified_runtime_recovery_reject_metadata(stamp)
        if decision["accepted"] and recovery_reject_metadata is not None:
            decision.update(recovery_reject_metadata)
            decision.update(
                {
                    "accepted": False,
                    "reason": "unverified_runtime_recovery_candidate",
                    "applied_along_m": 0.0,
                    "applied_cross_m": 0.0,
                    "applied_yaw_deg": 0.0,
                }
            )
            fused = copy.deepcopy(initial_pose)
        suppression_reason = self._robust_predictor_update_suppression_reason(decision, stamp)
        self._last_predictor_update_allowed = bool(decision["accepted"]) and not suppression_reason
        decision["predictor_update_allowed"] = self._last_predictor_update_allowed
        decision["predictor_update_suppressed_reason"] = suppression_reason
        decision["stamp_sec"] = stamp.nanoseconds / 1e9
        if decision["accepted"]:
            self._robust_accepted_count += 1
            if decision.get("clipped"):
                self._robust_clipped_count += 1
        else:
            self._robust_rejected_count += 1
        decision["accepted_count"] = self._robust_accepted_count
        decision["rejected_count"] = self._robust_rejected_count
        decision["clipped_count"] = self._robust_clipped_count
        self._publish_decision(decision, stamp)
        if not decision["accepted"]:
            self.get_logger().warn(
                "Rejecting NDT pose by robust innovation gate: "
                f"reason={decision['reason']} norm={decision.get('innovation_norm_m', 0.0):.2f}m "
                f"yaw={decision.get('innovation_yaw_deg', 0.0):.2f}deg "
                f"count={self._robust_rejected_count}",
                throttle_duration_sec=1.0,
            )
        return fused, bool(decision["accepted"])

    def _process_noise_xy(self):
        return max(0.0, self._robust_prior_xy_variance_floor * 0.1)

    def _process_noise_yaw(self):
        return max(0.0, self._robust_prior_yaw_variance_floor * 0.1)

    def _publish_decision(self, decision, stamp):
        if self._decision_publisher is None:
            return
        payload = dict(decision)
        payload.setdefault("stamp_sec", stamp.nanoseconds / 1e9)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._decision_publisher.publish(msg)

    def _publish_final_pose(self, msg, stamp, *, predictor_update=False):
        public_msg = _apply_body_frame_position_bias(
            msg,
            along_bias_m=self._output_along_bias_m,
            cross_bias_m=self._output_cross_bias_m,
        )
        self._publisher.publish(public_msg)
        self._last_output = public_msg
        if predictor_update:
            self._accepted_update_count += 1
            self._last_accepted_update_stamp = stamp
            if self._predictor_update_publisher is not None:
                self._predictor_update_publisher.publish(msg)

    def _publish_prediction_fallback_if_due(self, initial_msg, stamp):
        if not self._enable_prediction_fallback:
            return
        if not _prediction_fallback_due(
            first_accepted_seen=self._accepted_update_count > 0,
            last_accepted_stamp=self._last_accepted_update_stamp,
            current_stamp=stamp,
            min_age_sec=self._prediction_fallback_min_age,
        ):
            return
        if self._last_prediction_fallback_stamp is not None:
            if stamp.nanoseconds <= self._last_prediction_fallback_stamp.nanoseconds:
                return
        fallback = _make_prediction_fallback_msg(
            initial_msg,
            xy_variance_floor=self._prediction_fallback_xy_variance_floor,
            yaw_variance_floor=self._prediction_fallback_yaw_variance_floor,
        )
        self._prediction_fallback_count += 1
        self._last_prediction_fallback_stamp = stamp
        self._publish_decision(
            {
                "accepted": True,
                "reason": "prediction_fallback",
                "clipped": False,
                "prediction_fallback_count": self._prediction_fallback_count,
                "accepted_update_count": self._accepted_update_count,
                "predictor_update": False,
            },
            stamp,
        )
        self._publish_final_pose(fallback, stamp, predictor_update=False)

    def _prune_seed_history(self, stamp):
        while len(self._seed_history) > self._seed_history_max_samples:
            self._seed_history.popleft()
        if self._seed_history_duration <= 0.0:
            return
        cutoff_ns = stamp.nanoseconds - int(self._seed_history_duration * 1e9)
        while self._seed_history and self._seed_history[0][0].nanoseconds < cutoff_ns:
            self._seed_history.popleft()

    def _seed_covariance_is_usable(self, seed_msg):
        seed_var = _xy_variance(seed_msg.pose.covariance)
        seed_stddev = math.sqrt(seed_var) if math.isfinite(seed_var) else math.inf
        return self._max_seed_xy_stddev <= 0.0 or seed_stddev <= self._max_seed_xy_stddev

    def _seed_for_stamp(self, stamp):
        if not self._seed_history:
            if self._last_seed is None or self._last_seed_stamp is None:
                return None
            age = abs((stamp - self._last_seed_stamp).nanoseconds / 1e9)
            if self._max_seed_age > 0.0 and age > self._max_seed_age:
                return None
            return self._last_seed if self._seed_covariance_is_usable(self._last_seed) else None
        best_seed = None
        best_age = math.inf
        stamp_ns = stamp.nanoseconds
        for seed_stamp, seed_msg in reversed(self._seed_history):
            age = abs((stamp - seed_stamp).nanoseconds / 1e9)
            if self._max_seed_age > 0.0 and age > self._max_seed_age:
                if seed_stamp.nanoseconds < stamp_ns:
                    break
                continue
            if age >= best_age:
                continue
            if not self._seed_covariance_is_usable(seed_msg):
                continue
            best_age = age
            best_seed = seed_msg
        return best_seed

    def _seed_is_usable(self, stamp):
        return self._seed_for_stamp(stamp) is not None

    def _on_ndt(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        predictor_update = True
        if self._enable_robust_initial_update:
            msg, accepted = self._apply_robust_initial_update_if_available(msg, stamp)
            if not accepted:
                return
            predictor_update = self._last_predictor_update_allowed
        else:
            if not self._ndt_passes_initial_consistency(msg, stamp):
                return
            msg = self._apply_initial_correction_gain_if_available(msg, stamp)
        seed_msg = self._seed_for_stamp(stamp)
        if seed_msg is None:
            self._publish_final_pose(msg, stamp, predictor_update=predictor_update)
            return
        if self._fusion_mode == "ndt_cross_yaw_seed_along":
            fused, applied = _fuse_ndt_cross_yaw_seed_along_pose(
                msg,
                seed_msg,
                along_gain=self._along_gain,
                max_seed_along_residual_m=self._max_seed_along_residual,
            )
        else:
            fused, applied = _fuse_axis_specific_pose(
                msg,
                seed_msg,
                lateral_gain=self._lateral_gain,
                yaw_deadband_sigma=self._yaw_deadband_sigma,
            )
        if self._enable_temporal_filter:
            fused, details = _temporal_filter_axis_pose(
                fused,
                seed_msg,
                self._last_output,
                lateral_alpha=self._temporal_lateral_alpha,
                yaw_alpha=self._temporal_yaw_alpha,
                mahalanobis_gate=self._temporal_mahalanobis_gate,
                lateral_innovation_stddev_m=self._temporal_lateral_innovation_stddev,
                yaw_innovation_stddev_rad=self._temporal_yaw_innovation_stddev,
            )
            applied = applied or bool(details["rejected"])
        if applied:
            self.get_logger().debug(f"Published axis-fused NDT pose mode={self._fusion_mode}")
        self._publish_final_pose(fused, stamp, predictor_update=predictor_update)


def main(args=None):
    rclpy.init(args=args)
    node = NdtAxisSeedFuser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
