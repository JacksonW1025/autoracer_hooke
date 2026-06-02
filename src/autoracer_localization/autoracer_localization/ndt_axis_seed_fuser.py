import copy
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from .ndt_initial_pose_predictor import (
    _message_time,
    _normalize_angle,
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
        ndt_pose.orientation = _yaw_to_quaternion(
            _normalize_angle(_yaw_from_quaternion(ndt_pose.orientation) + yaw_error * yaw_gain)
        )
        applied = True

    return fused, applied


def _clamp01(value):
    return min(1.0, max(0.0, float(value)))


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
    filtered.pose.pose.orientation = _yaw_to_quaternion(target_yaw)
    return filtered, {"rejected": rejected, "mahalanobis": mahalanobis}


class NdtAxisSeedFuser(Node):
    def __init__(self):
        super().__init__("ndt_axis_seed_fuser")
        self.declare_parameter("raw_ndt_pose_topic", "/localization/ndt/raw_pose_with_covariance")
        self.declare_parameter("seed_pose_topic", "/localization/fixposition/seed_pose")
        self.declare_parameter("output_topic", "/localization/pose_with_covariance")
        self.declare_parameter("max_seed_age_sec", 0.5)
        self.declare_parameter("max_seed_xy_stddev_m", 0.75)
        self.declare_parameter("lateral_gain", 1.0)
        self.declare_parameter("yaw_deadband_sigma", 3.0)
        self.declare_parameter("enable_temporal_filter", False)
        self.declare_parameter("temporal_lateral_alpha", 1.0)
        self.declare_parameter("temporal_yaw_alpha", 1.0)
        self.declare_parameter("temporal_mahalanobis_gate", 0.0)
        self.declare_parameter("temporal_lateral_innovation_stddev_m", 0.5)
        self.declare_parameter("temporal_yaw_innovation_stddev_rad", 0.1)

        self._max_seed_age = float(self.get_parameter("max_seed_age_sec").value)
        self._max_seed_xy_stddev = float(self.get_parameter("max_seed_xy_stddev_m").value)
        self._lateral_gain = float(self.get_parameter("lateral_gain").value)
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
        self._last_seed = None
        self._last_seed_stamp = None
        self._last_output = None

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("seed_pose_topic").value,
            self._on_seed,
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

    def _seed_is_usable(self, stamp):
        if self._last_seed is None or self._last_seed_stamp is None:
            return False
        age = abs((stamp - self._last_seed_stamp).nanoseconds / 1e9)
        if self._max_seed_age > 0.0 and age > self._max_seed_age:
            return False
        seed_var = _xy_variance(self._last_seed.pose.covariance)
        seed_stddev = math.sqrt(seed_var) if math.isfinite(seed_var) else math.inf
        return self._max_seed_xy_stddev <= 0.0 or seed_stddev <= self._max_seed_xy_stddev

    def _on_ndt(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        if not self._seed_is_usable(stamp):
            self._publisher.publish(msg)
            self._last_output = None
            return
        fused, applied = _fuse_axis_specific_pose(
            msg,
            self._last_seed,
            lateral_gain=self._lateral_gain,
            yaw_deadband_sigma=self._yaw_deadband_sigma,
        )
        if self._enable_temporal_filter:
            fused, details = _temporal_filter_axis_pose(
                fused,
                self._last_seed,
                self._last_output,
                lateral_alpha=self._temporal_lateral_alpha,
                yaw_alpha=self._temporal_yaw_alpha,
                mahalanobis_gate=self._temporal_mahalanobis_gate,
                lateral_innovation_stddev_m=self._temporal_lateral_innovation_stddev,
                yaw_innovation_stddev_rad=self._temporal_yaw_innovation_stddev,
            )
            applied = applied or bool(details["rejected"])
        if applied:
            self.get_logger().debug("Published cross/yaw axis-fused NDT pose")
        self._publisher.publish(fused)
        self._last_output = fused


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
