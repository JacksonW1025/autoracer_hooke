#!/usr/bin/env python3
"""Route-progress based NDT initial-pose provider.

This node is intentionally narrow:
- it never publishes final localization;
- it does not use GT or future frames;
- GNSS is used only after projection onto the route as a weak 1D progress
  measurement;
- yaw is inherited from the current predictor/EKF pose, not from the route.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .ndt_initial_pose_predictor import _rpy_from_quaternion, _rpy_to_quaternion
from .pose_stream_qos import latest_pose_qos
from .pure_lidar_fixed_lag_tracker import Pose2D, RoutePath


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class ProgressFilter:
    progress_m: float
    variance_m2: float = 1.0
    last_stamp_sec: float | None = None
    velocity_mps: float = 0.0


def propagate_progress(
    state: ProgressFilter,
    *,
    stamp_sec: float,
    process_noise_m2ps: float,
) -> ProgressFilter:
    if state.last_stamp_sec is None:
        state.last_stamp_sec = stamp_sec
        return state
    dt = stamp_sec - state.last_stamp_sec
    if not math.isfinite(dt) or dt <= 0.0:
        return state
    dt = min(dt, 0.2)
    state.progress_m += state.velocity_mps * dt
    state.variance_m2 += max(0.0, process_noise_m2ps) * dt
    state.last_stamp_sec = stamp_sec
    return state


def update_progress_measurement(
    state: ProgressFilter,
    *,
    observed_progress_m: float,
    measurement_variance_m2: float,
    innovation_gate_m: float,
) -> tuple[ProgressFilter, bool]:
    innovation = observed_progress_m - state.progress_m
    if not math.isfinite(innovation) or abs(innovation) > max(0.0, innovation_gate_m):
        return state, False
    variance = max(1e-6, measurement_variance_m2)
    gain = state.variance_m2 / (state.variance_m2 + variance)
    state.progress_m += gain * innovation
    state.variance_m2 = max(1e-6, (1.0 - gain) * state.variance_m2)
    return state, True


def route_progress_pose_from_base(
    *,
    route_path: RoutePath,
    base_pose: Pose2D,
    progress_m: float,
    predicted_progress_m: float | None,
    route_search_radius_m: float,
    max_abs_cross_m: float,
) -> Pose2D:
    projection = route_path.project(
        base_pose,
        predicted_progress_m=predicted_progress_m,
        search_radius_m=route_search_radius_m,
    )
    cross = projection.cross_track_m if projection.is_valid else 0.0
    cross = clamp(cross, -abs(max_abs_cross_m), abs(max_abs_cross_m))
    center_x, center_y, route_yaw = route_path.center_at_progress(progress_m)
    return Pose2D(
        stamp_sec=base_pose.stamp_sec,
        x=center_x - math.sin(route_yaw) * cross,
        y=center_y + math.cos(route_yaw) * cross,
        yaw=base_pose.yaw,
    )


def pose_msg_to_pose2d(msg: PoseWithCovarianceStamped) -> Pose2D:
    roll, pitch, yaw = _rpy_from_quaternion(msg.pose.pose.orientation)
    del roll, pitch
    return Pose2D(
        stamp_sec=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9,
        x=float(msg.pose.pose.position.x),
        y=float(msg.pose.pose.position.y),
        yaw=yaw,
    )


def pose2d_to_msg(
    pose: Pose2D,
    *,
    template: PoseWithCovarianceStamped,
    xy_variance_m2: float,
    yaw_variance_rad2: float,
) -> PoseWithCovarianceStamped:
    msg = PoseWithCovarianceStamped()
    msg.header = template.header
    msg.header.stamp.sec = int(math.floor(pose.stamp_sec))
    msg.header.stamp.nanosec = int(round((pose.stamp_sec - msg.header.stamp.sec) * 1.0e9))
    msg.pose.pose.position.x = pose.x
    msg.pose.pose.position.y = pose.y
    msg.pose.pose.position.z = template.pose.pose.position.z
    msg.pose.pose.orientation = _rpy_to_quaternion(0.0, 0.0, pose.yaw)
    msg.pose.covariance = list(template.pose.covariance)
    msg.pose.covariance[0] = max(float(msg.pose.covariance[0]), xy_variance_m2)
    msg.pose.covariance[7] = max(float(msg.pose.covariance[7]), xy_variance_m2)
    msg.pose.covariance[35] = max(float(msg.pose.covariance[35]), yaw_variance_rad2)
    return msg


class RouteProgressInitialPoseProvider(Node):
    def __init__(self) -> None:
        super().__init__("route_progress_initial_pose_provider")
        self.declare_parameter("base_pose_topic", "/localization/pose_estimator/pose_with_covariance")
        self.declare_parameter("ndt_pose_topic", "/localization/pose_estimator/pose_with_covariance")
        self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("output_initial_pose_topic", "/localization/route_progress_initial_pose")
        self.declare_parameter("route_samples_csv", "")
        self.declare_parameter("enable_progress_filter", False)
        self.declare_parameter("ndt_gap_threshold_sec", 0.8)
        self.declare_parameter("process_noise_m2ps", 0.05)
        self.declare_parameter("gnss_progress_sigma_m", 5.0)
        self.declare_parameter("gnss_progress_innovation_gate_m", 12.0)
        self.declare_parameter("startup_gnss_passthrough_until_sec", 0.0)
        self.declare_parameter("route_search_radius_m", 60.0)
        self.declare_parameter("max_abs_cross_m", 3.0)
        self.declare_parameter("output_xy_variance_m2", 4.0)
        self.declare_parameter("output_yaw_variance_rad2", 0.20)

        route_csv = str(self.get_parameter("route_samples_csv").value)
        self._route_path = RoutePath.from_csv(route_csv) if route_csv else None
        self._enable = bool(self.get_parameter("enable_progress_filter").value)
        self._ndt_gap_threshold_sec = float(self.get_parameter("ndt_gap_threshold_sec").value)
        self._process_noise_m2ps = float(self.get_parameter("process_noise_m2ps").value)
        self._gnss_variance_m2 = float(self.get_parameter("gnss_progress_sigma_m").value) ** 2
        self._gnss_gate_m = float(self.get_parameter("gnss_progress_innovation_gate_m").value)
        self._startup_gnss_passthrough_until_sec = float(
            self.get_parameter("startup_gnss_passthrough_until_sec").value
        )
        self._route_search_radius_m = float(self.get_parameter("route_search_radius_m").value)
        self._max_abs_cross_m = float(self.get_parameter("max_abs_cross_m").value)
        self._output_xy_variance_m2 = float(self.get_parameter("output_xy_variance_m2").value)
        self._output_yaw_variance_rad2 = float(self.get_parameter("output_yaw_variance_rad2").value)

        self._latest_base: PoseWithCovarianceStamped | None = None
        self._latest_base_pose: Pose2D | None = None
        self._latest_ndt_stamp_sec: float | None = None
        self._progress: ProgressFilter | None = None

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("output_initial_pose_topic").value),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("base_pose_topic").value),
            self._on_base_pose,
            latest_pose_qos(),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("ndt_pose_topic").value),
            self._on_ndt_pose,
            latest_pose_qos(),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("gnss_pose_topic").value),
            self._on_gnss_pose,
            latest_pose_qos(),
        )
        self.create_subscription(
            VelocityReport,
            str(self.get_parameter("velocity_topic").value),
            self._on_velocity,
            10,
        )

    @staticmethod
    def _stamp_sec(msg: PoseWithCovarianceStamped) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9

    def _progress_from_pose(self, pose: Pose2D, predicted_progress_m: float | None) -> float | None:
        if self._route_path is None:
            return None
        projection = self._route_path.project(
            pose,
            predicted_progress_m=predicted_progress_m,
            search_radius_m=self._route_search_radius_m,
        )
        return projection.progress_m if projection.is_valid else None

    def _publish(self, msg: PoseWithCovarianceStamped) -> None:
        self._publisher.publish(msg)

    def _on_base_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_base = msg
        self._latest_base_pose = pose_msg_to_pose2d(msg)
        if not self._enable or self._route_path is None:
            self._publish(msg)
            return

        stamp_sec = self._stamp_sec(msg)
        is_gap = (
            self._latest_ndt_stamp_sec is None
            or stamp_sec - self._latest_ndt_stamp_sec > self._ndt_gap_threshold_sec
        )
        progress = self._progress_from_pose(
            self._latest_base_pose,
            self._progress.progress_m if self._progress is not None else None,
        )
        if self._progress is None and progress is not None:
            self._progress = ProgressFilter(progress_m=progress, last_stamp_sec=stamp_sec)
        elif not is_gap and progress is not None:
            self._progress = ProgressFilter(progress_m=progress, last_stamp_sec=stamp_sec)

        if not is_gap or self._progress is None:
            self._publish(msg)
            return

        propagate_progress(
            self._progress,
            stamp_sec=stamp_sec,
            process_noise_m2ps=self._process_noise_m2ps,
        )
        corrected = route_progress_pose_from_base(
            route_path=self._route_path,
            base_pose=self._latest_base_pose,
            progress_m=self._progress.progress_m,
            predicted_progress_m=self._progress.progress_m,
            route_search_radius_m=self._route_search_radius_m,
            max_abs_cross_m=self._max_abs_cross_m,
        )
        self._publish(
            pose2d_to_msg(
                corrected,
                template=msg,
                xy_variance_m2=self._output_xy_variance_m2,
                yaw_variance_rad2=self._output_yaw_variance_rad2,
            )
        )

    def _on_ndt_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_ndt_stamp_sec = self._stamp_sec(msg)

    def _on_velocity(self, msg: VelocityReport) -> None:
        if self._progress is None:
            return
        self._progress.velocity_mps = float(msg.longitudinal_velocity)

    def _on_gnss_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if (
            self._enable
            and self._latest_base is None
            and self._startup_gnss_passthrough_until_sec > 0.0
            and self._stamp_sec(msg) <= self._startup_gnss_passthrough_until_sec
        ):
            self._publish(msg)
        if not self._enable or self._route_path is None or self._progress is None:
            return
        pose = pose_msg_to_pose2d(msg)
        projection = self._route_path.project(
            pose,
            predicted_progress_m=self._progress.progress_m,
            search_radius_m=self._route_search_radius_m,
        )
        if not projection.is_valid:
            return
        update_progress_measurement(
            self._progress,
            observed_progress_m=projection.progress_m,
            measurement_variance_m2=self._gnss_variance_m2,
            innovation_gate_m=self._gnss_gate_m,
        )


def main() -> None:
    rclpy.init()
    node = RouteProgressInitialPoseProvider()
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
