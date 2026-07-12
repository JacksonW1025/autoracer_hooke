import copy
import json
import math
import statistics
from collections import deque
from dataclasses import dataclass

import rclpy
from autoware_internal_localization_msgs.srv import (
    PoseWithCovarianceStamped as AlignPose,
)
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _rpy_from_quaternion(q) -> tuple[float, float, float]:
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch, _yaw_from_quaternion(q)


def _set_rpy(q, roll: float, pitch: float, yaw: float) -> None:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PoseCorrection2D:
    x: float
    y: float
    yaw: float

    def apply(self, pose: Pose2D) -> Pose2D:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return Pose2D(
            x=self.x + cos_yaw * pose.x - sin_yaw * pose.y,
            y=self.y + sin_yaw * pose.x + cos_yaw * pose.y,
            yaw=_normalize_angle(self.yaw + pose.yaw),
        )


def _correction_between(source: Pose2D, target: Pose2D) -> PoseCorrection2D:
    yaw = _normalize_angle(target.yaw - source.yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return PoseCorrection2D(
        x=target.x - cos_yaw * source.x + sin_yaw * source.y,
        y=target.y - sin_yaw * source.x - cos_yaw * source.y,
        yaw=yaw,
    )


@dataclass(frozen=True)
class GuardDecision:
    degraded: bool
    state_changed: bool
    xy_innovation_m: float
    yaw_innovation_rad: float
    reason: str


class PoseConsistencyGuardCore:
    """Detect slow NDT drift against a causal wheel/IMU motion prediction."""

    def __init__(
        self,
        *,
        anchor_interval_sec: float,
        anchor_history_sec: float,
        max_xy_innovation_m: float,
        max_yaw_innovation_rad: float,
        gross_xy_innovation_m: float,
        gross_yaw_innovation_rad: float,
        yaw_rate_allowance_sec: float,
        violation_hold_sec: float,
        recovery_xy_innovation_m: float,
        recovery_yaw_innovation_rad: float,
        recovery_hold_sec: float,
        recovery_consistency_window_sec: float,
        recovery_consistency_min_samples: int,
        recovery_consistency_max_xy_spread_m: float,
        recovery_consistency_max_yaw_spread_rad: float,
        max_twist_age_sec: float,
    ) -> None:
        self.anchor_interval_sec = anchor_interval_sec
        self.anchor_history_sec = anchor_history_sec
        self.max_xy_innovation_m = max_xy_innovation_m
        self.max_yaw_innovation_rad = max_yaw_innovation_rad
        self.gross_xy_innovation_m = gross_xy_innovation_m
        self.gross_yaw_innovation_rad = gross_yaw_innovation_rad
        self.yaw_rate_allowance_sec = yaw_rate_allowance_sec
        self.violation_hold_sec = violation_hold_sec
        self.recovery_xy_innovation_m = recovery_xy_innovation_m
        self.recovery_yaw_innovation_rad = recovery_yaw_innovation_rad
        self.recovery_hold_sec = recovery_hold_sec
        self.recovery_consistency_window_sec = recovery_consistency_window_sec
        self.recovery_consistency_min_samples = recovery_consistency_min_samples
        self.recovery_consistency_max_xy_spread_m = recovery_consistency_max_xy_spread_m
        self.recovery_consistency_max_yaw_spread_rad = (
            recovery_consistency_max_yaw_spread_rad
        )
        self.max_twist_age_sec = max_twist_age_sec

        self.predicted_pose: Pose2D | None = None
        self.predicted_stamp_sec: float | None = None
        self.anchor_stamp_sec: float | None = None
        self.prediction_tracks: deque[tuple[float, Pose2D]] = deque()
        self.latest_twist_stamp_sec: float | None = None
        self.latest_vx_mps = 0.0
        self.latest_wz_radps = 0.0
        self.degraded = False
        self.violation_start_sec: float | None = None
        self.recovery_start_sec: float | None = None
        self.recovery_observations = deque()

    def _reset_prediction(self, stamp_sec: float, pose: Pose2D) -> None:
        self.prediction_tracks.clear()
        self.prediction_tracks.append((stamp_sec, pose))
        self.predicted_pose = pose
        self.predicted_stamp_sec = stamp_sec
        self.anchor_stamp_sec = stamp_sec

    def _median_prediction(self) -> Pose2D:
        poses = [item[1] for item in self.prediction_tracks]
        reference_yaw = poses[0].yaw
        return Pose2D(
            x=statistics.median(item.x for item in poses),
            y=statistics.median(item.y for item in poses),
            yaw=_normalize_angle(
                statistics.median(
                    reference_yaw + _normalize_angle(item.yaw - reference_yaw)
                    for item in poses
                )
            ),
        )

    def _add_trusted_anchor(self, stamp_sec: float, pose: Pose2D) -> None:
        self.prediction_tracks.append((stamp_sec, pose))
        while (
            len(self.prediction_tracks) > 1
            and stamp_sec - self.prediction_tracks[0][0] > self.anchor_history_sec
        ):
            self.prediction_tracks.popleft()
        self.predicted_pose = self._median_prediction()
        self.anchor_stamp_sec = stamp_sec

    def _propagate_to(self, stamp_sec: float) -> None:
        if not self.prediction_tracks or self.predicted_stamp_sec is None:
            return
        dt = stamp_sec - self.predicted_stamp_sec
        if dt <= 0.0:
            return
        propagated_tracks = deque()
        for created_stamp_sec, pose in self.prediction_tracks:
            heading_mid = pose.yaw + 0.5 * self.latest_wz_radps * dt
            propagated_tracks.append(
                (
                    created_stamp_sec,
                    Pose2D(
                        x=pose.x + self.latest_vx_mps * math.cos(heading_mid) * dt,
                        y=pose.y + self.latest_vx_mps * math.sin(heading_mid) * dt,
                        yaw=_normalize_angle(
                            pose.yaw + self.latest_wz_radps * dt
                        ),
                    ),
                )
            )
        self.prediction_tracks = propagated_tracks
        self.predicted_pose = self._median_prediction()
        self.predicted_stamp_sec = stamp_sec

    def update_twist(self, stamp_sec: float, vx_mps: float, wz_radps: float) -> None:
        if not all(math.isfinite(value) for value in (stamp_sec, vx_mps, wz_radps)):
            return
        if (
            self.latest_twist_stamp_sec is not None
            and stamp_sec < self.latest_twist_stamp_sec
        ):
            self.predicted_pose = None
            self.predicted_stamp_sec = None
            self.anchor_stamp_sec = None
            self.prediction_tracks.clear()
            self.degraded = False
            self.violation_start_sec = None
            self.recovery_start_sec = None
            self.recovery_observations.clear()
        self.latest_twist_stamp_sec = stamp_sec
        self.latest_vx_mps = vx_mps
        self.latest_wz_radps = wz_radps
        self._propagate_to(stamp_sec)

    def evaluate_pose(self, stamp_sec: float, pose: Pose2D) -> GuardDecision:
        if self.predicted_pose is None:
            self._reset_prediction(stamp_sec, pose)
            return GuardDecision(False, False, 0.0, 0.0, "initialized")

        twist_age_sec = (
            math.inf
            if self.latest_twist_stamp_sec is None
            else abs(stamp_sec - self.latest_twist_stamp_sec)
        )
        if twist_age_sec > self.max_twist_age_sec:
            changed = self.degraded
            self.degraded = False
            self.violation_start_sec = None
            self.recovery_start_sec = None
            self.recovery_observations.clear()
            self._reset_prediction(stamp_sec, pose)
            return GuardDecision(False, changed, 0.0, 0.0, "twist_unavailable")

        self._propagate_to(stamp_sec)
        assert self.predicted_pose is not None
        xy_innovation_m = math.hypot(
            pose.x - self.predicted_pose.x,
            pose.y - self.predicted_pose.y,
        )
        yaw_innovation_rad = abs(_normalize_angle(pose.yaw - self.predicted_pose.yaw))
        state_changed = False
        reason = "healthy"

        if not self.degraded:
            effective_yaw_threshold_rad = self.max_yaw_innovation_rad + (
                abs(self.latest_wz_radps) * self.yaw_rate_allowance_sec
            )
            violates = (
                xy_innovation_m > self.gross_xy_innovation_m
                or yaw_innovation_rad > self.gross_yaw_innovation_rad
                or (
                    xy_innovation_m > self.max_xy_innovation_m
                    and yaw_innovation_rad > effective_yaw_threshold_rad
                )
            )
            trusted_for_anchor = (
                xy_innovation_m <= self.max_xy_innovation_m
                and yaw_innovation_rad <= effective_yaw_threshold_rad
            )
            if violates:
                if self.violation_start_sec is None:
                    self.violation_start_sec = stamp_sec
                if stamp_sec - self.violation_start_sec >= self.violation_hold_sec:
                    self.degraded = True
                    state_changed = True
                    self.recovery_start_sec = None
                    self.recovery_observations.clear()
                    reason = "persistent_motion_inconsistency"
            else:
                self.violation_start_sec = None
                if (
                    trusted_for_anchor
                    and (
                        self.anchor_stamp_sec is None
                        or stamp_sec - self.anchor_stamp_sec >= self.anchor_interval_sec
                    )
                ):
                    self._add_trusted_anchor(stamp_sec, pose)
                    reason = "trusted_anchor_refresh"
                elif not trusted_for_anchor:
                    reason = "trusted_anchor_refresh_withheld"
        else:
            offset_x = pose.x - self.predicted_pose.x
            offset_y = pose.y - self.predicted_pose.y
            offset_yaw = _normalize_angle(pose.yaw - self.predicted_pose.yaw)
            self.recovery_observations.append(
                (stamp_sec, offset_x, offset_y, offset_yaw)
            )
            while (
                self.recovery_observations
                and stamp_sec - self.recovery_observations[0][0]
                > self.recovery_consistency_window_sec
            ):
                self.recovery_observations.popleft()

            recovers = (
                xy_innovation_m <= self.recovery_xy_innovation_m
                and yaw_innovation_rad <= self.recovery_yaw_innovation_rad
            )
            if recovers:
                if self.recovery_start_sec is None:
                    self.recovery_start_sec = stamp_sec
                if stamp_sec - self.recovery_start_sec >= self.recovery_hold_sec:
                    self.degraded = False
                    state_changed = True
                    self.violation_start_sec = None
                    self.recovery_start_sec = None
                    self.recovery_observations.clear()
                    self._reset_prediction(stamp_sec, pose)
                    reason = "motion_consistency_recovered"
            else:
                self.recovery_start_sec = None
                reason = "motion_inconsistency_active"

            if (
                self.degraded
                and self._bounded_recovery_window(
                    xy_innovation_m, yaw_innovation_rad
                )
            ):
                self.degraded = False
                state_changed = True
                self.violation_start_sec = None
                self.recovery_start_sec = None
                self.recovery_observations.clear()
                self._reset_prediction(stamp_sec, pose)
                reason = "bounded_window_recovered"

        return GuardDecision(
            self.degraded,
            state_changed,
            xy_innovation_m,
            yaw_innovation_rad,
            reason,
        )

    def _bounded_recovery_window(
        self,
        xy_innovation_m: float,
        yaw_innovation_rad: float,
    ) -> bool:
        if (
            xy_innovation_m > self.recovery_xy_innovation_m
            or yaw_innovation_rad > self.recovery_yaw_innovation_rad
        ):
            return False

        return self.recovery_window_is_consistent(
            self.recovery_consistency_max_xy_spread_m,
            self.recovery_consistency_max_yaw_spread_rad,
        )

    def recovery_window_is_consistent(
        self,
        xy_spread_limit_m: float,
        yaw_spread_limit_rad: float,
    ) -> bool:
        observations = self.recovery_observations
        if len(observations) < self.recovery_consistency_min_samples:
            return False
        if (
            observations[-1][0] - observations[0][0]
            < self.recovery_consistency_window_sec * 0.95
        ):
            return False

        mean_x = statistics.fmean(item[1] for item in observations)
        mean_y = statistics.fmean(item[2] for item in observations)
        observed_xy_spread_m = max(
            math.hypot(item[1] - mean_x, item[2] - mean_y) for item in observations
        )
        yaw_offsets = [item[3] for item in observations]
        yaw_spread_rad = max(yaw_offsets) - min(yaw_offsets)
        return (
            observed_xy_spread_m <= xy_spread_limit_m
            and yaw_spread_rad <= yaw_spread_limit_rad
        )

    def accept_relocalization(self, stamp_sec: float, pose: Pose2D) -> bool:
        changed = self.degraded
        self.degraded = False
        self.violation_start_sec = None
        self.recovery_start_sec = None
        self.recovery_observations.clear()
        self._reset_prediction(stamp_sec, pose)
        return changed

    def effective_yaw_threshold_rad(self) -> float:
        return self.max_yaw_innovation_rad + (
            abs(self.latest_wz_radps) * self.yaw_rate_allowance_sec
        )


def predicted_pose_message(
    template: PoseWithCovarianceStamped,
    predicted_pose: Pose2D,
) -> PoseWithCovarianceStamped:
    output = copy.deepcopy(template)
    output.pose.pose.position.x = predicted_pose.x
    output.pose.pose.position.y = predicted_pose.y
    orientation = output.pose.pose.orientation
    roll, pitch, _ = _rpy_from_quaternion(orientation)
    _set_rpy(orientation, roll, pitch, predicted_pose.yaw)
    return output


class NdtPoseConsistencyGuard(Node):
    def __init__(self) -> None:
        super().__init__("ndt_pose_consistency_guard")
        self.declare_parameter(
            "input_pose_topic", "/localization/pose_estimator/pose_with_covariance"
        )
        self.declare_parameter(
            "input_twist_topic", "/localization/twist_estimator/twist_with_covariance"
        )
        self.declare_parameter(
            "output_pose_topic",
            "/localization/pose_estimator/guarded_pose_with_covariance",
        )
        self.declare_parameter(
            "status_topic", "/localization/ndt_pose_consistency_guard/status"
        )
        self.declare_parameter("anchor_interval_sec", 5.0)
        self.declare_parameter("anchor_history_sec", 20.0)
        self.declare_parameter("max_xy_innovation_m", 0.2)
        self.declare_parameter("max_yaw_innovation_deg", 0.15)
        self.declare_parameter("gross_xy_innovation_m", 0.6)
        self.declare_parameter("gross_yaw_innovation_deg", 2.0)
        self.declare_parameter("yaw_rate_allowance_sec", 0.06)
        self.declare_parameter("violation_hold_sec", 0.2)
        self.declare_parameter("recovery_xy_innovation_m", 0.12)
        self.declare_parameter("recovery_yaw_innovation_deg", 0.2)
        self.declare_parameter("recovery_hold_sec", 5.0)
        self.declare_parameter("recovery_consistency_window_sec", 5.0)
        self.declare_parameter("recovery_consistency_min_samples", 30)
        self.declare_parameter("recovery_consistency_max_xy_spread_m", 0.15)
        self.declare_parameter("recovery_consistency_max_yaw_spread_deg", 0.5)
        self.declare_parameter("enable_ndt_relocalization", False)
        self.declare_parameter(
            "ndt_align_service_topic",
            "/localization/pose_estimator/ndt_align_srv",
        )
        self.declare_parameter("relocalization_min_degraded_sec", 5.0)
        self.declare_parameter("relocalization_retry_sec", 15.0)
        self.declare_parameter("relocalization_window_max_xy_spread_m", 0.1)
        self.declare_parameter("relocalization_window_max_yaw_spread_deg", 0.2)
        self.declare_parameter("relocalization_search_xy_stddev_m", 0.5)
        self.declare_parameter("relocalization_search_z_stddev_m", 0.15)
        self.declare_parameter("relocalization_search_rp_stddev_deg", 1.5)
        self.declare_parameter("relocalization_search_yaw_stddev_deg", 15.0)
        self.declare_parameter("relocalization_max_correction_m", 0.75)
        self.declare_parameter("relocalization_max_correction_yaw_deg", 1.0)
        self.declare_parameter("relocalization_confirmation_xy_m", 0.15)
        self.declare_parameter("relocalization_confirmation_yaw_deg", 0.2)
        self.declare_parameter("relocalization_confirmation_hold_sec", 0.5)
        self.declare_parameter("relocalization_confirmation_timeout_sec", 3.0)
        self.declare_parameter("max_twist_age_sec", 0.25)
        self._core = PoseConsistencyGuardCore(
            anchor_interval_sec=float(self.get_parameter("anchor_interval_sec").value),
            anchor_history_sec=float(self.get_parameter("anchor_history_sec").value),
            max_xy_innovation_m=float(self.get_parameter("max_xy_innovation_m").value),
            max_yaw_innovation_rad=math.radians(
                float(self.get_parameter("max_yaw_innovation_deg").value)
            ),
            gross_xy_innovation_m=float(
                self.get_parameter("gross_xy_innovation_m").value
            ),
            gross_yaw_innovation_rad=math.radians(
                float(self.get_parameter("gross_yaw_innovation_deg").value)
            ),
            yaw_rate_allowance_sec=float(
                self.get_parameter("yaw_rate_allowance_sec").value
            ),
            violation_hold_sec=float(self.get_parameter("violation_hold_sec").value),
            recovery_xy_innovation_m=float(
                self.get_parameter("recovery_xy_innovation_m").value
            ),
            recovery_yaw_innovation_rad=math.radians(
                float(self.get_parameter("recovery_yaw_innovation_deg").value)
            ),
            recovery_hold_sec=float(self.get_parameter("recovery_hold_sec").value),
            recovery_consistency_window_sec=float(
                self.get_parameter("recovery_consistency_window_sec").value
            ),
            recovery_consistency_min_samples=int(
                self.get_parameter("recovery_consistency_min_samples").value
            ),
            recovery_consistency_max_xy_spread_m=float(
                self.get_parameter("recovery_consistency_max_xy_spread_m").value
            ),
            recovery_consistency_max_yaw_spread_rad=math.radians(
                float(
                    self.get_parameter("recovery_consistency_max_yaw_spread_deg").value
                )
            ),
            max_twist_age_sec=float(self.get_parameter("max_twist_age_sec").value),
        )
        self._relocalization_enabled = bool(
            self.get_parameter("enable_ndt_relocalization").value
        )
        self._relocalization_min_degraded_sec = float(
            self.get_parameter("relocalization_min_degraded_sec").value
        )
        self._relocalization_retry_sec = float(
            self.get_parameter("relocalization_retry_sec").value
        )
        self._relocalization_window_max_xy_spread_m = float(
            self.get_parameter("relocalization_window_max_xy_spread_m").value
        )
        self._relocalization_window_max_yaw_spread_rad = math.radians(
            float(
                self.get_parameter("relocalization_window_max_yaw_spread_deg").value
            )
        )
        self._relocalization_search_xy_stddev_m = float(
            self.get_parameter("relocalization_search_xy_stddev_m").value
        )
        self._relocalization_search_z_stddev_m = float(
            self.get_parameter("relocalization_search_z_stddev_m").value
        )
        self._relocalization_search_rp_stddev_rad = math.radians(
            float(self.get_parameter("relocalization_search_rp_stddev_deg").value)
        )
        self._relocalization_search_yaw_stddev_rad = math.radians(
            float(self.get_parameter("relocalization_search_yaw_stddev_deg").value)
        )
        self._relocalization_max_correction_m = float(
            self.get_parameter("relocalization_max_correction_m").value
        )
        self._relocalization_max_correction_yaw_rad = math.radians(
            float(
                self.get_parameter("relocalization_max_correction_yaw_deg").value
            )
        )
        self._relocalization_confirmation_xy_m = float(
            self.get_parameter("relocalization_confirmation_xy_m").value
        )
        self._relocalization_confirmation_yaw_rad = math.radians(
            float(self.get_parameter("relocalization_confirmation_yaw_deg").value)
        )
        self._relocalization_confirmation_hold_sec = float(
            self.get_parameter("relocalization_confirmation_hold_sec").value
        )
        self._relocalization_confirmation_timeout_sec = float(
            self.get_parameter("relocalization_confirmation_timeout_sec").value
        )
        self._align_client = (
            self.create_client(
                AlignPose,
                self.get_parameter("ndt_align_service_topic").value,
            )
            if self._relocalization_enabled
            else None
        )
        self._relocalization_generation = 0
        self._relocalization_future = None
        self._relocalization_request_prediction: Pose2D | None = None
        self._relocalization_request_stamp_sec: float | None = None
        self._relocalization_correction: PoseCorrection2D | None = None
        self._relocalization_response_stamp_sec: float | None = None
        self._relocalization_confirmation_start_sec: float | None = None
        self._relocalization_last_attempt_sec: float | None = None
        self._degraded_since_sec: float | None = None
        self._pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_pose_topic").value,
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            10,
        )
        self._latest_pose_template = None
        self.create_subscription(
            TwistWithCovarianceStamped,
            self.get_parameter("input_twist_topic").value,
            self._on_twist,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_pose_topic").value,
            self._on_pose,
            10,
        )

    def _on_twist(self, msg: TwistWithCovarianceStamped) -> None:
        twist = msg.twist.twist
        self._core.update_twist(
            _stamp_sec(msg.header.stamp),
            float(twist.linear.x),
            float(twist.angular.z),
        )
        if self._core.degraded:
            self._publish_prediction(msg.header.stamp)

    def _publish_prediction(self, stamp) -> None:
        if self._latest_pose_template is None or self._core.predicted_pose is None:
            return
        output = predicted_pose_message(
            self._latest_pose_template,
            self._core.predicted_pose,
        )
        output.header.stamp = stamp
        self._pose_publisher.publish(output)

    def _clear_relocalization_state(self) -> None:
        self._relocalization_generation += 1
        self._relocalization_future = None
        self._relocalization_request_prediction = None
        self._relocalization_request_stamp_sec = None
        self._relocalization_correction = None
        self._relocalization_response_stamp_sec = None
        self._relocalization_confirmation_start_sec = None

    def _relocalization_state(self) -> str:
        if not self._relocalization_enabled:
            return "disabled"
        if self._relocalization_future is not None:
            return "aligning"
        if self._relocalization_correction is not None:
            return "confirming"
        return "idle"

    def _maybe_start_relocalization(
        self,
        stamp_sec: float,
        template: PoseWithCovarianceStamped,
    ) -> None:
        if (
            not self._relocalization_enabled
            or self._align_client is None
            or not self._core.degraded
            or self._core.predicted_pose is None
            or self._relocalization_future is not None
            or self._relocalization_correction is not None
        ):
            return
        if self._degraded_since_sec is None:
            self._degraded_since_sec = stamp_sec
        if stamp_sec - self._degraded_since_sec < self._relocalization_min_degraded_sec:
            return
        if (
            self._relocalization_last_attempt_sec is not None
            and stamp_sec - self._relocalization_last_attempt_sec
            < self._relocalization_retry_sec
        ):
            return
        if not self._core.recovery_window_is_consistent(
            self._relocalization_window_max_xy_spread_m,
            self._relocalization_window_max_yaw_spread_rad,
        ):
            return
        if not self._align_client.service_is_ready():
            return

        request_prediction = self._core.predicted_pose
        request_pose = predicted_pose_message(template, request_prediction)
        request_pose.header.stamp = template.header.stamp
        covariance = [0.0] * 36
        covariance[0] = self._relocalization_search_xy_stddev_m**2
        covariance[7] = self._relocalization_search_xy_stddev_m**2
        covariance[14] = self._relocalization_search_z_stddev_m**2
        covariance[21] = self._relocalization_search_rp_stddev_rad**2
        covariance[28] = self._relocalization_search_rp_stddev_rad**2
        covariance[35] = self._relocalization_search_yaw_stddev_rad**2
        request_pose.pose.covariance = covariance

        request = AlignPose.Request()
        request.pose_with_covariance = request_pose
        self._relocalization_generation += 1
        generation = self._relocalization_generation
        self._relocalization_request_prediction = request_prediction
        self._relocalization_request_stamp_sec = stamp_sec
        self._relocalization_last_attempt_sec = stamp_sec
        future = self._align_client.call_async(request)
        self._relocalization_future = future
        future.add_done_callback(
            lambda completed, token=generation: self._on_relocalization_response(
                completed, token
            )
        )
        self.get_logger().info(
            "Started 200-particle NDT relocalization at %.3f s" % stamp_sec
        )

    def _on_relocalization_response(self, future, generation: int) -> None:
        if generation != self._relocalization_generation:
            return
        self._relocalization_future = None
        request_prediction = self._relocalization_request_prediction
        request_stamp_sec = self._relocalization_request_stamp_sec
        if request_prediction is None or request_stamp_sec is None or not self._core.degraded:
            self._clear_relocalization_state()
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"NDT relocalization service failed: {exc}")
            self._relocalization_request_prediction = None
            self._relocalization_request_stamp_sec = None
            return
        if response is None or not response.success or not response.reliable:
            self.get_logger().warn("NDT relocalization returned an unreliable result")
            self._relocalization_request_prediction = None
            self._relocalization_request_stamp_sec = None
            return

        result = response.pose_with_covariance.pose.pose
        result_pose = Pose2D(
            float(result.position.x),
            float(result.position.y),
            _yaw_from_quaternion(result.orientation),
        )
        correction_xy_m = math.hypot(
            result_pose.x - request_prediction.x,
            result_pose.y - request_prediction.y,
        )
        correction_yaw_rad = abs(
            _normalize_angle(result_pose.yaw - request_prediction.yaw)
        )
        if (
            correction_xy_m > self._relocalization_max_correction_m
            or correction_yaw_rad > self._relocalization_max_correction_yaw_rad
        ):
            self.get_logger().warn(
                "Rejected NDT relocalization correction: xy=%.3f m yaw=%.3f deg"
                % (correction_xy_m, math.degrees(correction_yaw_rad))
            )
            self._relocalization_request_prediction = None
            self._relocalization_request_stamp_sec = None
            return

        self._relocalization_correction = _correction_between(
            request_prediction, result_pose
        )
        self._relocalization_response_stamp_sec = (
            self._core.latest_twist_stamp_sec or request_stamp_sec
        )
        self._relocalization_confirmation_start_sec = None
        self.get_logger().info(
            "NDT relocalization candidate ready: xy=%.3f m yaw=%.3f deg"
            % (correction_xy_m, math.degrees(correction_yaw_rad))
        )

    def _try_confirm_relocalization(self, stamp_sec: float, raw_pose: Pose2D) -> bool:
        correction = self._relocalization_correction
        request_stamp_sec = self._relocalization_request_stamp_sec
        response_stamp_sec = self._relocalization_response_stamp_sec
        predicted_pose = self._core.predicted_pose
        if (
            correction is None
            or request_stamp_sec is None
            or response_stamp_sec is None
            or predicted_pose is None
            or not self._core.degraded
        ):
            return False
        if stamp_sec <= request_stamp_sec + 0.05:
            return False
        if stamp_sec - response_stamp_sec > self._relocalization_confirmation_timeout_sec:
            self.get_logger().warn("NDT relocalization raw confirmation timed out")
            self._relocalization_correction = None
            self._relocalization_response_stamp_sec = None
            self._relocalization_confirmation_start_sec = None
            return False

        candidate = correction.apply(predicted_pose)
        xy_m = math.hypot(raw_pose.x - candidate.x, raw_pose.y - candidate.y)
        yaw_rad = abs(_normalize_angle(raw_pose.yaw - candidate.yaw))
        agrees = (
            xy_m <= self._relocalization_confirmation_xy_m
            and yaw_rad <= self._relocalization_confirmation_yaw_rad
        )
        if not agrees:
            self._relocalization_confirmation_start_sec = None
            return False
        if self._relocalization_confirmation_start_sec is None:
            self._relocalization_confirmation_start_sec = stamp_sec
            return False
        if (
            stamp_sec - self._relocalization_confirmation_start_sec
            < self._relocalization_confirmation_hold_sec
        ):
            return False

        self._core.accept_relocalization(stamp_sec, raw_pose)
        self._degraded_since_sec = None
        self._clear_relocalization_state()
        self.get_logger().info(
            "Accepted NDT relocalization after raw confirmation: "
            "xy=%.3f m yaw=%.3f deg" % (xy_m, math.degrees(yaw_rad))
        )
        return True

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_pose_template = msg
        pose = msg.pose.pose
        stamp_sec = _stamp_sec(msg.header.stamp)
        raw_pose = Pose2D(
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quaternion(pose.orientation),
        )
        decision = self._core.evaluate_pose(
            stamp_sec,
            raw_pose,
        )
        if decision.state_changed:
            if decision.degraded:
                self._degraded_since_sec = stamp_sec
            else:
                self._degraded_since_sec = None
            self._clear_relocalization_state()

        if decision.degraded and self._try_confirm_relocalization(
            stamp_sec, raw_pose
        ):
            decision = GuardDecision(
                degraded=False,
                state_changed=True,
                xy_innovation_m=decision.xy_innovation_m,
                yaw_innovation_rad=decision.yaw_innovation_rad,
                reason="ndt_relocalization_recovered",
            )
        elif decision.degraded:
            self._maybe_start_relocalization(stamp_sec, msg)

        if not decision.degraded:
            self._pose_publisher.publish(msg)

        status = {
            "degraded": decision.degraded,
            "reason": decision.reason,
            "stamp_sec": stamp_sec,
            "xy_innovation_m": decision.xy_innovation_m,
            "yaw_innovation_deg": math.degrees(decision.yaw_innovation_rad),
            "effective_yaw_threshold_deg": math.degrees(
                self._core.effective_yaw_threshold_rad()
            ),
            "recovery_policy": "strict_hold_or_bounded_window_or_ndt_align",
            "relocalization_state": self._relocalization_state(),
        }
        if self._core.predicted_pose is not None:
            status["predicted_x"] = self._core.predicted_pose.x
            status["predicted_y"] = self._core.predicted_pose.y
            status["predicted_yaw_rad"] = self._core.predicted_pose.yaw
        if self._core.anchor_stamp_sec is not None:
            status["prediction_anchor_age_sec"] = max(
                0.0, _stamp_sec(msg.header.stamp) - self._core.anchor_stamp_sec
            )
        status["prediction_anchor_count"] = len(self._core.prediction_tracks)
        status_msg = String()
        status_msg.data = json.dumps(status, sort_keys=True, separators=(",", ":"))
        self._status_publisher.publish(status_msg)
        if decision.state_changed:
            message = "NDT motion consistency %s: xy=%.3f m yaw=%.3f deg" % (
                "degraded" if decision.degraded else "recovered",
                decision.xy_innovation_m,
                math.degrees(decision.yaw_innovation_rad),
            )
            if decision.degraded:
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NdtPoseConsistencyGuard()
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
