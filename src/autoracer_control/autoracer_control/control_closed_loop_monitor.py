"""Metrics monitor for the ROS-only control closed-loop bench."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import time

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import AccelWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from autoracer_control.control_closed_loop_geometry import (
    PathPoint,
    compute_stations,
    monotonic_progress,
    project_to_path,
)
from autoracer_control.control_closed_loop_scenarios import FULL_VALIDATION, get_scenario_spec


SUMMARY_FILE = "closed_loop_summary.json"
TRACE_FILE = "closed_loop_trace.jsonl"
STAGE = "control_closed_loop_tuning_ros_only"
CONTROLLER_UNDER_TEST = "autoware_trajectory_follower_node/controller_node_exe"
DATA_SOURCE = "virtual_chassis"
SUMMARY_KEYS = ["does_not_validate"]
DOES_NOT_VALIDATE = [
    "CarMaker closed-loop",
    "Stage B planner",
    "real vehicle calibration",
    "race performance",
]
METRIC_FIELDS = [
    "rms_lateral_error_m",
    "max_abs_lateral_error_m",
    "final_abs_lateral_error_m",
    "rms_heading_error_rad",
    "rms_velocity_error_mps",
    "max_abs_velocity_error_mps",
    "max_abs_acc_actual_mps2",
    "max_abs_jerk_actual_mps3",
    "max_abs_steer_cmd_rad",
    "max_abs_steer_actual_rad",
    "max_abs_steer_rate_radps",
    "max_estimated_lat_acc_mps2",
    "steer_cmd_saturation_ratio",
    "steer_rate_saturation_ratio",
    "speed_regime",
    "reference_velocity_mps",
    "actual_velocity_mps",
    "reference_curvature_1pm",
    "longitudinal_validated",
    "oscillation_score",
]

TRAJECTORY_TOPIC = "/control_bench/planning/trajectory"
OPERATION_MODE_TOPIC = "/control_bench/system/operation_mode/state"
ODOMETRY_TOPIC = "/control_bench/localization/kinematic_state"
STEERING_TOPIC = "/control_bench/vehicle/status/steering_status"
ACCELERATION_TOPIC = "/control_bench/localization/acceleration"
RAW_CONTROL_TOPIC = "/control_bench/autoracer/control/raw_control_cmd"


@dataclass
class Sample:
    lateral_error: float
    heading_error: float
    velocity_error: float
    actual_velocity: float
    reference_velocity: float
    reference_curvature: float
    estimated_lat_acc: float
    progress_distance: float
    trajectory_progress_ratio: float


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _max_abs(values: list[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def _curvature(points, index: int) -> float:
    if index <= 0 or index >= len(points) - 1:
        return 0.0
    p1 = points[index - 1].pose.position
    p2 = points[index].pose.position
    p3 = points[index + 1].pose.position
    a = math.hypot(p2.x - p1.x, p2.y - p1.y)
    b = math.hypot(p3.x - p2.x, p3.y - p2.y)
    c = math.hypot(p3.x - p1.x, p3.y - p1.y)
    denom = a * b * c
    if denom <= 1e-9:
        return 0.0
    signed_area2 = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)
    return 2.0 * signed_area2 / denom


class ControlClosedLoopMonitor(Node):
    def __init__(self) -> None:
        super().__init__("control_closed_loop_monitor")
        self.declare_parameter("scenario", "straight_lateral_offset")
        self.declare_parameter("scenario_type", "smoke")
        self.declare_parameter("summary_root", "logs/control_closed_loop")
        self.declare_parameter("run_duration_sec", 12.0)
        self.declare_parameter("max_duration_sec", 12.0)
        self.declare_parameter("completion_threshold", 0.98)
        self.declare_parameter("max_lat_acc_guardrail_mps2", 1.5)
        self.declare_parameter("max_lateral_error_hard_m", 2.0)
        self.declare_parameter("max_steer", 0.488)
        self.declare_parameter("max_steer_rate", 1.0)
        self.declare_parameter("max_acc", 1.0)
        self.declare_parameter("min_acc", -2.0)
        self.declare_parameter("max_jerk", 2.0)
        self.declare_parameter("min_jerk", -4.0)
        self.declare_parameter("longitudinal_validated", True)
        self.declare_parameter("trajectory_topic", TRAJECTORY_TOPIC)
        self.declare_parameter("operation_mode_topic", OPERATION_MODE_TOPIC)
        self.declare_parameter("odometry_topic", ODOMETRY_TOPIC)
        self.declare_parameter("steering_topic", STEERING_TOPIC)
        self.declare_parameter("acceleration_topic", ACCELERATION_TOPIC)
        self.declare_parameter("raw_control_topic", RAW_CONTROL_TOPIC)

        self._scenario = str(self.get_parameter("scenario").value)
        self._scenario_type = str(self.get_parameter("scenario_type").value)
        self._scenario_spec = get_scenario_spec(self._scenario)
        self._start_time = self.get_clock().now()
        self._samples: list[Sample] = []
        self._trace_rows: list[dict] = []
        self._trajectory: Trajectory | None = None
        self._path_points: list[PathPoint] = []
        self._stations: list[float] = []
        self._path_length_m = 0.0
        self._progress_distance_m = 0.0
        self._last_progress_distance_m = 0.0
        self._acc_values: list[float] = []
        self._jerk_values: list[float] = []
        self._steer_actual_values: list[float] = []
        self._steer_rate_values: list[float] = []
        self._steer_cmd_values: list[float] = []
        self._acc_by_progress: list[tuple[float, float]] = []
        self._jerk_by_progress: list[tuple[float, float]] = []
        self._steer_rate_by_progress: list[tuple[float, float]] = []
        self._last_acc: tuple[float, float] | None = None
        self._last_steer: tuple[float, float] | None = None
        self._last_raw_control_seen: float | None = None
        self._invalid_seen = False
        self._completed = False
        self._end_condition = "running"
        self._exit_code = 1
        self._summary_path: Path | None = None
        self._trace_path: Path | None = None
        self._first_seen: dict[str, float | None] = {
            "trajectory": None,
            "operation_mode": None,
            "odometry": None,
            "steering": None,
            "acceleration": None,
            "raw_control": None,
        }

        self.create_subscription(
            Trajectory,
            str(self.get_parameter("trajectory_topic").value),
            self._on_trajectory,
            10,
        )
        operation_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OperationModeState,
            str(self.get_parameter("operation_mode_topic").value),
            self._on_operation_mode,
            operation_qos,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odometry_topic").value), self._on_odometry, 10
        )
        self.create_subscription(
            SteeringReport,
            str(self.get_parameter("steering_topic").value),
            self._on_steering,
            10,
        )
        self.create_subscription(
            AccelWithCovarianceStamped,
            str(self.get_parameter("acceleration_topic").value),
            self._on_acceleration,
            10,
        )
        self.create_subscription(
            Control, str(self.get_parameter("raw_control_topic").value), self._on_raw_control, 10
        )
        self.create_timer(0.2, self._check_done)

    @property
    def exit_code(self) -> int:
        return self._exit_code

    @property
    def summary_path(self) -> Path | None:
        return self._summary_path

    @property
    def completed(self) -> bool:
        return self._completed

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _mark_seen(self, key: str) -> None:
        if self._first_seen[key] is None:
            self._first_seen[key] = self._elapsed()

    def _on_trajectory(self, msg: Trajectory) -> None:
        self._mark_seen("trajectory")
        self._trajectory = msg
        self._invalid_seen |= not self._trajectory_is_finite(msg)
        self._path_points = [
            PathPoint(float(point.pose.position.x), float(point.pose.position.y))
            for point in msg.points
        ]
        self._stations = compute_stations(self._path_points)
        self._path_length_m = self._stations[-1] if self._stations else 0.0
        if self._path_length_m > 0.0:
            self._progress_distance_m = min(self._progress_distance_m, self._path_length_m)

    def _on_operation_mode(self, msg: OperationModeState) -> None:
        self._mark_seen("operation_mode")

    def _on_odometry(self, msg: Odometry) -> None:
        self._mark_seen("odometry")
        if self._trajectory is None or not self._trajectory.points:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        actual_velocity = float(msg.twist.twist.linear.x)
        projection = self._project_progress(x, y)
        self._progress_distance_m = monotonic_progress(
            self._progress_distance_m, projection.progress_distance_m
        )
        self._progress_distance_m = min(self._progress_distance_m, self._path_length_m)
        self._last_progress_distance_m = self._progress_distance_m
        trajectory_progress_ratio = self._trajectory_progress_ratio()
        nearest_idx = projection.nearest_idx
        point = self._trajectory.points[nearest_idx]
        ref_x = projection.projected_x_m
        ref_y = projection.projected_y_m
        ref_yaw = projection.segment_yaw_rad
        dx = x - ref_x
        dy = y - ref_y
        lateral_error = -math.sin(ref_yaw) * dx + math.cos(ref_yaw) * dy
        heading_error = _wrap_to_pi(yaw - ref_yaw)
        reference_velocity = float(point.longitudinal_velocity_mps)
        reference_curvature = _curvature(self._trajectory.points, nearest_idx)
        velocity_error = actual_velocity - reference_velocity
        estimated_lat_acc = actual_velocity * actual_velocity * abs(reference_curvature)
        self._trace_rows.append(
            {
                "elapsed_sec": self._elapsed(),
                "nearest_idx": nearest_idx,
                "nearest_segment_idx": projection.nearest_segment_idx,
                "progress_distance_m": self._progress_distance_m,
                "trajectory_progress_ratio": trajectory_progress_ratio,
                "vehicle_x_m": x,
                "vehicle_y_m": y,
                "vehicle_yaw_rad": yaw,
                "reference_x_m": ref_x,
                "reference_y_m": ref_y,
                "reference_yaw_rad": ref_yaw,
                "lateral_error_m": lateral_error,
                "heading_error_rad": heading_error,
                "actual_velocity_mps": actual_velocity,
                "reference_velocity_mps": reference_velocity,
                "velocity_error_mps": velocity_error,
                "reference_curvature_1pm": reference_curvature,
                "estimated_lateral_acc_mps2": estimated_lat_acc,
            }
        )
        sample = Sample(
            lateral_error=lateral_error,
            heading_error=heading_error,
            velocity_error=velocity_error,
            actual_velocity=actual_velocity,
            reference_velocity=reference_velocity,
            reference_curvature=reference_curvature,
            estimated_lat_acc=estimated_lat_acc,
            progress_distance=self._progress_distance_m,
            trajectory_progress_ratio=trajectory_progress_ratio,
        )
        self._invalid_seen |= not self._sample_is_finite(sample)
        self._samples.append(sample)

    def _on_steering(self, msg: SteeringReport) -> None:
        self._mark_seen("steering")
        now = self._elapsed()
        steering = float(msg.steering_tire_angle)
        self._steer_actual_values.append(steering)
        if self._last_steer is not None:
            prev_time, prev_steer = self._last_steer
            dt = max(now - prev_time, 1e-6)
            steer_rate = (steering - prev_steer) / dt
            self._steer_rate_values.append(steer_rate)
            self._steer_rate_by_progress.append((self._last_progress_distance_m, steer_rate))
        self._last_steer = (now, steering)
        self._invalid_seen |= not math.isfinite(steering)

    def _on_acceleration(self, msg: AccelWithCovarianceStamped) -> None:
        self._mark_seen("acceleration")
        now = self._elapsed()
        acceleration = float(msg.accel.accel.linear.x)
        self._acc_values.append(acceleration)
        self._acc_by_progress.append((self._last_progress_distance_m, acceleration))
        if self._last_acc is not None:
            prev_time, prev_acc = self._last_acc
            dt = max(now - prev_time, 1e-6)
            jerk = (acceleration - prev_acc) / dt
            self._jerk_values.append(jerk)
            self._jerk_by_progress.append((self._last_progress_distance_m, jerk))
        self._last_acc = (now, acceleration)
        self._invalid_seen |= not math.isfinite(acceleration)

    def _on_raw_control(self, msg: Control) -> None:
        self._mark_seen("raw_control")
        self._last_raw_control_seen = self._elapsed()
        steer_cmd = float(msg.lateral.steering_tire_angle)
        self._steer_cmd_values.append(steer_cmd)
        self._invalid_seen |= not math.isfinite(steer_cmd)
        self._invalid_seen |= not math.isfinite(float(msg.longitudinal.acceleration))

    def _nearest_trajectory_index(self, x: float, y: float) -> int:
        assert self._trajectory is not None
        distances = [
            (point.pose.position.x - x) ** 2 + (point.pose.position.y - y) ** 2
            for point in self._trajectory.points
        ]
        return min(range(len(distances)), key=distances.__getitem__)

    def _project_progress(self, x: float, y: float):
        if len(self._path_points) >= 2 and len(self._path_points) == len(self._stations):
            return project_to_path(self._path_points, self._stations, x, y)

        nearest_idx = self._nearest_trajectory_index(x, y)
        point = self._trajectory.points[nearest_idx]
        return project_to_path(
            [
                PathPoint(float(point.pose.position.x), float(point.pose.position.y)),
                PathPoint(float(point.pose.position.x) + 1e-3, float(point.pose.position.y)),
            ],
            [0.0, 1e-3],
            x,
            y,
        )

    def _trajectory_progress_ratio(self) -> float:
        if self._path_length_m <= 1e-9:
            return 0.0
        return min(1.0, max(0.0, self._progress_distance_m / self._path_length_m))

    def _completed_trajectory(self) -> bool:
        return self._trajectory_progress_ratio() >= float(
            self.get_parameter("completion_threshold").value
        )

    def _check_done(self) -> None:
        if self._completed:
            return
        end_condition = self._current_end_condition()
        if end_condition is None:
            return
        self._end_condition = end_condition
        summary = self._make_summary(end_condition)
        self._write_summary(summary)
        self._exit_code = 0 if summary["result"] == "PASS" else 1
        self._completed = True
        self.get_logger().info(f"closed-loop summary: {self._summary_path}")

    def _current_end_condition(self) -> str | None:
        early_reasons = self._early_failure_reasons()
        if early_reasons:
            return "failure"

        elapsed = self._elapsed()
        if self._scenario_type == FULL_VALIDATION:
            if self._completed_trajectory():
                return "trajectory_complete"
            if elapsed >= float(self.get_parameter("max_duration_sec").value):
                return "timeout"
            return None

        if elapsed >= float(self.get_parameter("run_duration_sec").value):
            return "duration_complete"
        if elapsed >= float(self.get_parameter("max_duration_sec").value):
            return "timeout"
        return None

    def _early_failure_reasons(self) -> list[str]:
        reasons = []
        elapsed = self._elapsed()
        if elapsed > 10.0 and (
            self._last_raw_control_seen is None or elapsed - self._last_raw_control_seen > 10.0
        ):
            reasons.append("raw_control_timeout")
        if self._invalid_seen:
            reasons.append("nan_or_inf_detected")
        if self._samples and _max_abs([sample.lateral_error for sample in self._samples]) > float(
            self.get_parameter("max_lateral_error_hard_m").value
        ):
            reasons.append("lateral_error_hard_limit")
        return reasons

    def _make_summary(self, end_condition: str) -> dict:
        metrics = self._calculate_metrics()
        failure_reasons = self._failure_reasons(metrics, end_condition)
        result = "PASS" if not failure_reasons else "FAIL"
        if result == "FAIL" and end_condition not in {"timeout"}:
            end_condition = "failure"
        return {
            "stage": STAGE,
            "result": result,
            "scenario": self._scenario,
            "scenario_type": self._scenario_type,
            "controller_under_test": CONTROLLER_UNDER_TEST,
            "data_source": DATA_SOURCE,
            "does_not_validate": DOES_NOT_VALIDATE,
            "trace_file": TRACE_FILE,
            "path_length_m": self._path_length_m,
            "progress_distance_m": self._progress_distance_m,
            "trajectory_progress_ratio": self._trajectory_progress_ratio(),
            "completed_trajectory": self._completed_trajectory(),
            "end_condition": end_condition,
            "max_duration_sec": float(self.get_parameter("max_duration_sec").value),
            "completion_threshold": float(self.get_parameter("completion_threshold").value),
            "first_sample_times_sec": self._first_seen,
            "failure_reasons": failure_reasons,
            "metrics": metrics,
            "segments": self._calculate_segments(),
        }

    def _calculate_metrics(self) -> dict:
        lateral = [sample.lateral_error for sample in self._samples]
        heading = [sample.heading_error for sample in self._samples]
        velocity = [sample.velocity_error for sample in self._samples]
        ref_velocities = [sample.reference_velocity for sample in self._samples]
        actual_velocities = [sample.actual_velocity for sample in self._samples]
        curvatures = [sample.reference_curvature for sample in self._samples]
        lat_acc = [sample.estimated_lat_acc for sample in self._samples]

        initial_abs_lateral_error = abs(lateral[0]) if lateral else 0.0
        final_abs_lateral_error = abs(lateral[-1]) if lateral else 0.0
        reference_velocity = statistics.fmean(ref_velocities) if ref_velocities else 0.0
        actual_velocity = statistics.fmean(actual_velocities) if actual_velocities else 0.0
        reference_curvature = max((abs(value) for value in curvatures), default=0.0)
        speed_regime = self._speed_regime(max(ref_velocities, default=0.0))
        max_steer = float(self.get_parameter("max_steer").value)
        max_steer_rate = float(self.get_parameter("max_steer_rate").value)
        steer_cmd_saturation_ratio = self._ratio_at_limit(self._steer_cmd_values, max_steer)
        steer_rate_saturation_ratio = self._ratio_at_limit(self._steer_rate_values, max_steer_rate)

        return {
            "rms_lateral_error_m": _rms(lateral),
            "max_abs_lateral_error_m": _max_abs(lateral),
            "initial_abs_lateral_error_m": initial_abs_lateral_error,
            "final_abs_lateral_error_m": final_abs_lateral_error,
            "rms_heading_error_rad": _rms(heading),
            "rms_velocity_error_mps": _rms(velocity),
            "max_abs_velocity_error_mps": _max_abs(velocity),
            "max_abs_acc_actual_mps2": _max_abs(self._acc_values),
            "max_abs_jerk_actual_mps3": _max_abs(self._jerk_values),
            "max_abs_steer_cmd_rad": _max_abs(self._steer_cmd_values),
            "max_abs_steer_actual_rad": _max_abs(self._steer_actual_values),
            "max_abs_steer_rate_radps": _max_abs(self._steer_rate_values),
            "max_estimated_lat_acc_mps2": max(lat_acc, default=0.0),
            "steer_cmd_saturation_ratio": steer_cmd_saturation_ratio,
            "steer_rate_saturation_ratio": steer_rate_saturation_ratio,
            "speed_regime": speed_regime,
            "reference_velocity_mps": reference_velocity,
            "actual_velocity_mps": actual_velocity,
            "reference_curvature_1pm": reference_curvature,
            "longitudinal_validated": bool(self.get_parameter("longitudinal_validated").value),
            "oscillation_score": self._oscillation_score(),
        }

    def _failure_reasons(self, metrics: dict, end_condition: str) -> list[str]:
        reasons = self._early_failure_reasons()
        received_inputs = [
            self._first_seen["trajectory"],
            self._first_seen["operation_mode"],
            self._first_seen["odometry"],
            self._first_seen["steering"],
            self._first_seen["acceleration"],
        ]
        if any(value is None or value > 10.0 for value in received_inputs):
            reasons.append("missing_controller_input_within_10s")
        if (
            self._first_seen["raw_control"] is None
            or self._first_seen["raw_control"] > 10.0
            or self._last_raw_control_seen is None
        ):
            reasons.append("missing_raw_control_within_10s")
        if len(self._samples) < 10:
            reasons.append("insufficient_odometry_samples")
        if self._invalid_seen or not self._metrics_are_finite(metrics):
            reasons.append("nan_or_inf_detected")
        if metrics["max_abs_lateral_error_m"] > float(
            self.get_parameter("max_lateral_error_hard_m").value
        ):
            reasons.append("lateral_error_hard_limit")
        if self._scenario_type == FULL_VALIDATION:
            if end_condition == "timeout":
                reasons.append("trajectory_timeout")
            if not self._completed_trajectory():
                reasons.append("trajectory_not_completed")
        if metrics["max_abs_steer_actual_rad"] > float(self.get_parameter("max_steer").value) + 1e-6:
            reasons.append("steer_actual_limit")
        max_abs_acc = max(
            abs(float(self.get_parameter("max_acc").value)),
            abs(float(self.get_parameter("min_acc").value)),
        )
        if metrics["max_abs_acc_actual_mps2"] > max_abs_acc + 1e-6:
            reasons.append("acceleration_limit")
        max_abs_jerk = max(
            abs(float(self.get_parameter("max_jerk").value)),
            abs(float(self.get_parameter("min_jerk").value)),
        )
        if metrics["max_abs_jerk_actual_mps3"] > max_abs_jerk + 0.25:
            reasons.append("jerk_limit")
        if metrics["max_estimated_lat_acc_mps2"] > float(
            self.get_parameter("max_lat_acc_guardrail_mps2").value
        ):
            reasons.append("lateral_acc_guardrail")
        if metrics["steer_cmd_saturation_ratio"] > 0.20:
            reasons.append("steer_cmd_saturation")
        if metrics["steer_rate_saturation_ratio"] > 0.20:
            reasons.append("steer_rate_saturation")
        if metrics["oscillation_score"] > 5.0:
            reasons.append("sustained_oscillation")
        return sorted(set(reasons))

    def _calculate_segments(self) -> list[dict]:
        segments = []
        for segment in self._scenario_spec.segments:
            start_s, end_s = segment.s_range_m
            samples = [
                sample
                for sample in self._samples
                if self._progress_in_range(sample.progress_distance, start_s, end_s)
            ]
            steer_rates = self._values_in_progress_range(
                self._steer_rate_by_progress, start_s, end_s
            )
            accelerations = self._values_in_progress_range(self._acc_by_progress, start_s, end_s)
            jerks = self._values_in_progress_range(self._jerk_by_progress, start_s, end_s)
            segment_failure_reasons = []
            if not samples:
                segment_failure_reasons.append("no_samples")

            segments.append(
                {
                    "name": segment.name,
                    "s_range_m": [start_s, end_s],
                    "reference_velocity_mps": segment.reference_velocity_mps,
                    "sample_count": len(samples),
                    "rms_lateral_error_m": _rms(
                        [sample.lateral_error for sample in samples]
                    ),
                    "rms_velocity_error_mps": _rms(
                        [sample.velocity_error for sample in samples]
                    ),
                    "max_abs_steer_rate_radps": _max_abs(steer_rates),
                    "max_abs_acc_actual_mps2": _max_abs(accelerations),
                    "max_abs_jerk_actual_mps3": _max_abs(jerks),
                    "failure_reasons": segment_failure_reasons,
                }
            )
        return segments

    @staticmethod
    def _progress_in_range(progress_m: float, start_s: float, end_s: float) -> bool:
        if math.isclose(progress_m, end_s, abs_tol=1e-9):
            return True
        return start_s <= progress_m < end_s

    def _values_in_progress_range(
        self, values_by_progress: list[tuple[float, float]], start_s: float, end_s: float
    ) -> list[float]:
        return [
            value
            for progress_m, value in values_by_progress
            if self._progress_in_range(progress_m, start_s, end_s)
        ]

    def _write_summary(self, summary: dict) -> None:
        root = Path(str(self.get_parameter("summary_root").value))
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        directory = root / f"{timestamp}-{self._scenario}"
        directory.mkdir(parents=True, exist_ok=True)
        self._summary_path = directory / SUMMARY_FILE
        self._summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        self._trace_path = directory / TRACE_FILE
        with self._trace_path.open("w", encoding="utf-8") as stream:
            for row in self._trace_rows:
                stream.write(json.dumps(row, sort_keys=True) + "\n")

    @staticmethod
    def _speed_regime(reference_velocity: float) -> str:
        if reference_velocity <= 0.3:
            return "stop_crawl"
        if reference_velocity <= 2.0:
            return "low_speed_tuning"
        return "guarded_speed_probe"

    @staticmethod
    def _ratio_at_limit(values: list[float], limit: float) -> float:
        if not values or limit <= 0.0:
            return 0.0
        count = sum(1 for value in values if abs(value) >= 0.99 * limit)
        return count / len(values)

    def _oscillation_score(self) -> float:
        values = [value for value in self._steer_actual_values if abs(value) > 1e-3]
        if len(values) < 3:
            return 0.0
        sign_changes = 0
        prev_sign = math.copysign(1.0, values[0])
        for value in values[1:]:
            sign = math.copysign(1.0, value)
            if sign != prev_sign:
                sign_changes += 1
            prev_sign = sign
        duration = max(self._elapsed(), 1e-6)
        return sign_changes / duration

    @staticmethod
    def _trajectory_is_finite(msg: Trajectory) -> bool:
        for point in msg.points:
            values = [
                point.pose.position.x,
                point.pose.position.y,
                point.pose.position.z,
                point.pose.orientation.x,
                point.pose.orientation.y,
                point.pose.orientation.z,
                point.pose.orientation.w,
                point.longitudinal_velocity_mps,
                point.lateral_velocity_mps,
                point.acceleration_mps2,
                point.heading_rate_rps,
                point.front_wheel_angle_rad,
                point.rear_wheel_angle_rad,
            ]
            if not all(math.isfinite(float(value)) for value in values):
                return False
        return True

    @staticmethod
    def _sample_is_finite(sample: Sample) -> bool:
        return all(math.isfinite(float(value)) for value in sample.__dict__.values())

    @staticmethod
    def _metrics_are_finite(metrics: dict) -> bool:
        for value in metrics.values():
            if isinstance(value, bool) or isinstance(value, str):
                continue
            if not math.isfinite(float(value)):
                return False
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlClosedLoopMonitor()
    try:
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    exit_code = node.exit_code
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)
