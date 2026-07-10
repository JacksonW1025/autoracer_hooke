import copy
import math

from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from .ndt_initial_pose_predictor import (
    NdtInitialPosePredictor,
    _propagate,
    _yaw_from_quaternion,
    _yaw_to_quaternion,
)


def _finite_or_zero(value):
    value = float(value)
    return value if math.isfinite(value) else 0.0


class KinematicStatePublisher(Node):
    """Publish Autoware's kinematic_state from NDT pose and vehicle feedback."""

    def __init__(self):
        super().__init__("kinematic_state_publisher")

        self.declare_parameter("ndt_pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("steering_topic", "/vehicle/status/steering_status")
        self.declare_parameter("output_topic", "/localization/kinematic_state")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_base_m", 1.9)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("vehicle_status_timeout_sec", 0.5)
        self.declare_parameter("ndt_lost_timeout_sec", 1.0)
        self.declare_parameter("max_prediction_step_sec", 0.1)
        self.declare_parameter("process_xy_noise_per_m", 0.02)
        self.declare_parameter("process_yaw_noise_per_s", 0.0025)
        self.declare_parameter("twist_linear_variance", 0.25)
        self.declare_parameter("twist_yaw_rate_variance", 0.04)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._wheel_base = float(self.get_parameter("wheel_base_m").value)
        self._vehicle_status_timeout = float(
            self.get_parameter("vehicle_status_timeout_sec").value
        )
        self._ndt_lost_timeout = float(self.get_parameter("ndt_lost_timeout_sec").value)
        self._max_prediction_step = float(
            self.get_parameter("max_prediction_step_sec").value
        )
        self._process_xy_noise_per_m = float(
            self.get_parameter("process_xy_noise_per_m").value
        )
        self._process_yaw_noise_per_s = float(
            self.get_parameter("process_yaw_noise_per_s").value
        )
        self._twist_linear_variance = float(
            self.get_parameter("twist_linear_variance").value
        )
        self._twist_yaw_rate_variance = float(
            self.get_parameter("twist_yaw_rate_variance").value
        )

        self._state = None
        self._last_ndt_receipt = None
        self._last_velocity = None
        self._last_velocity_receipt = None
        self._last_steering = None
        self._last_steering_receipt = None

        self._publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("ndt_pose_topic").value,
            self._on_ndt_pose,
            10,
        )
        self.create_subscription(
            VelocityReport,
            self.get_parameter("velocity_topic").value,
            self._on_velocity,
            10,
        )
        self.create_subscription(
            SteeringReport,
            self.get_parameter("steering_topic").value,
            self._on_steering,
            10,
        )

        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"Publishing kinematic state on {self.get_parameter('output_topic').value}"
        )

    def _on_ndt_pose(self, msg):
        if not NdtInitialPosePredictor._pose_is_usable(msg):
            self.get_logger().warn("Ignoring unusable NDT pose", throttle_duration_sec=1.0)
            return

        now = self.get_clock().now()
        self._last_ndt_receipt = now
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds <= 0:
            stamp = now

        pose = msg.pose.pose
        self._state = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": _yaw_from_quaternion(pose.orientation),
            "covariance": list(msg.pose.covariance),
            "stamp": stamp,
        }

    def _on_velocity(self, msg):
        self._last_velocity = msg
        self._last_velocity_receipt = self.get_clock().now()

    def _on_steering(self, msg):
        self._last_steering = msg
        self._last_steering_receipt = self.get_clock().now()

    def _on_timer(self):
        if self._state is None:
            return

        now = self.get_clock().now()
        if self._ndt_is_lost(now):
            self.get_logger().warn(
                "NDT pose is stale; publishing predicted kinematic state",
                throttle_duration_sec=1.0,
            )
        self._advance_state(now)
        odom = self._state_to_odom(now)
        if odom is not None:
            self._publisher.publish(odom)

    def _advance_state(self, now):
        if self._state is None:
            return

        dt = (now - self._state["stamp"]).nanoseconds / 1e9
        if dt <= 0.0:
            self._state["stamp"] = now
            return

        velocity, _, yaw_rate = self._motion(now)
        remaining = dt
        max_step = max(self._max_prediction_step, 1e-3)
        while remaining > 1e-9:
            step = min(remaining, max_step)
            x, y, yaw = _propagate(
                self._state["x"],
                self._state["y"],
                self._state["yaw"],
                velocity,
                yaw_rate,
                step,
            )
            self._state["x"] = x
            self._state["y"] = y
            self._state["yaw"] = yaw
            self._increase_covariance(abs(velocity) * step, step)
            remaining -= step

        self._state["stamp"] = now

    def _increase_covariance(self, distance, dt):
        cov = self._state["covariance"]
        cov[0] = float(cov[0]) + self._process_xy_noise_per_m * distance
        cov[7] = float(cov[7]) + self._process_xy_noise_per_m * distance
        cov[35] = float(cov[35]) + self._process_yaw_noise_per_s * dt

    def _motion(self, now):
        if self._last_velocity is None or self._last_velocity_receipt is None:
            return 0.0, 0.0, 0.0

        age = (now - self._last_velocity_receipt).nanoseconds / 1e9
        if age > self._vehicle_status_timeout:
            return 0.0, 0.0, 0.0

        velocity = _finite_or_zero(self._last_velocity.longitudinal_velocity)
        lateral_velocity = _finite_or_zero(self._last_velocity.lateral_velocity)

        yaw_rate = float(self._last_velocity.heading_rate)
        if math.isfinite(yaw_rate):
            return velocity, lateral_velocity, yaw_rate
        return velocity, lateral_velocity, self._yaw_rate_from_steering(now, velocity)

    def _yaw_rate_from_steering(self, now, velocity):
        if self._last_steering is None or self._last_steering_receipt is None:
            return 0.0

        age = (now - self._last_steering_receipt).nanoseconds / 1e9
        if age > self._vehicle_status_timeout:
            return 0.0

        steering = float(self._last_steering.steering_tire_angle)
        if not math.isfinite(steering) or self._wheel_base <= 0.0:
            return 0.0
        return velocity * math.tan(steering) / self._wheel_base

    def _state_to_odom(self, stamp):
        if self._state is None:
            return None

        velocity, lateral_velocity, yaw_rate = self._motion(stamp)
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._map_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._state["x"]
        msg.pose.pose.position.y = self._state["y"]
        msg.pose.pose.position.z = self._state["z"]
        msg.pose.pose.orientation = _yaw_to_quaternion(self._state["yaw"])
        msg.pose.covariance = copy.copy(self._state["covariance"])
        msg.twist.twist.linear.x = velocity
        msg.twist.twist.linear.y = lateral_velocity
        msg.twist.twist.angular.z = yaw_rate
        msg.twist.covariance[0] = self._twist_linear_variance
        msg.twist.covariance[7] = self._twist_linear_variance
        msg.twist.covariance[35] = self._twist_yaw_rate_variance
        return msg

    def _ndt_is_lost(self, now):
        if self._last_ndt_receipt is None:
            return False
        return (now - self._last_ndt_receipt).nanoseconds / 1e9 > self._ndt_lost_timeout


def main(args=None):
    rclpy.init(args=args)
    node = KinematicStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
