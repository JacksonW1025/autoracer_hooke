import copy
import math

import rclpy
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw):
    q = PoseWithCovarianceStamped().pose.pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _propagate(x, y, yaw, velocity, yaw_rate, dt):
    if abs(yaw_rate) < 1e-5:
        return (
            x + velocity * math.cos(yaw) * dt,
            y + velocity * math.sin(yaw) * dt,
            _normalize_angle(yaw),
        )

    next_yaw = _normalize_angle(yaw + yaw_rate * dt)
    radius = velocity / yaw_rate
    return (
        x + radius * (math.sin(next_yaw) - math.sin(yaw)),
        y - radius * (math.cos(next_yaw) - math.cos(yaw)),
        next_yaw,
    )


class NdtInitialPosePredictor(Node):
    def __init__(self):
        super().__init__("ndt_initial_pose_predictor")

        self.declare_parameter("seed_pose_topic", "/localization/fixposition/seed_pose")
        self.declare_parameter("ndt_pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("steering_topic", "/vehicle/status/steering_status")
        self.declare_parameter("output_topic", "/localization/ndt_initial_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("wheel_base_m", 1.9)
        self.declare_parameter("vehicle_status_timeout_sec", 0.5)
        self.declare_parameter("ndt_lost_timeout_sec", 1.0)
        self.declare_parameter("max_prediction_step_sec", 0.2)
        self.declare_parameter("process_xy_noise_per_m", 0.02)
        self.declare_parameter("process_yaw_noise_per_s", 0.0025)

        self._map_frame = self.get_parameter("map_frame").value
        self._wheel_base = float(self.get_parameter("wheel_base_m").value)
        self._vehicle_status_timeout = float(
            self.get_parameter("vehicle_status_timeout_sec").value
        )
        self._ndt_lost_timeout = float(self.get_parameter("ndt_lost_timeout_sec").value)
        self._max_prediction_step = float(self.get_parameter("max_prediction_step_sec").value)
        self._process_xy_noise_per_m = float(
            self.get_parameter("process_xy_noise_per_m").value
        )
        self._process_yaw_noise_per_s = float(
            self.get_parameter("process_yaw_noise_per_s").value
        )

        self._state = None
        self._last_ndt_receipt = None
        self._last_velocity = None
        self._last_velocity_receipt = None
        self._last_steering = None
        self._last_steering_receipt = None

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("seed_pose_topic").value,
            self._on_seed_pose,
            10,
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
            f"Publishing NDT initial pose on {self.get_parameter('output_topic').value}"
        )

    def _on_seed_pose(self, msg):
        now = self.get_clock().now()
        if self._state is None or self._ndt_is_lost(now):
            self._set_state_from_pose(msg, now, source="Fixposition seed")

    def _on_ndt_pose(self, msg):
        now = self.get_clock().now()
        self._last_ndt_receipt = now
        self._set_state_from_pose(msg, rclpy.time.Time.from_msg(msg.header.stamp), source="NDT")

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
        self._advance_state(now)
        self._publisher.publish(self._state_to_msg(now))

    def _set_state_from_pose(self, msg, stamp, source):
        pose = msg.pose.pose
        if not self._pose_is_usable(msg):
            self.get_logger().warn(f"Ignoring unusable {source} pose", throttle_duration_sec=1.0)
            return

        self._state = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": _yaw_from_quaternion(pose.orientation),
            "covariance": list(msg.pose.covariance),
            "stamp": stamp,
        }
        self.get_logger().info(f"Initial-pose predictor corrected from {source}")

    def _advance_state(self, now):
        dt = (now - self._state["stamp"]).nanoseconds / 1e9
        if dt <= 0.0:
            self._state["stamp"] = now
            return
        dt = min(dt, self._max_prediction_step)

        velocity, yaw_rate = self._motion(now)
        x, y, yaw = _propagate(
            self._state["x"], self._state["y"], self._state["yaw"], velocity, yaw_rate, dt
        )
        self._state["x"] = x
        self._state["y"] = y
        self._state["yaw"] = yaw
        self._state["stamp"] = now

        distance = abs(velocity) * dt
        cov = self._state["covariance"]
        cov[0] = float(cov[0]) + self._process_xy_noise_per_m * distance
        cov[7] = float(cov[7]) + self._process_xy_noise_per_m * distance
        cov[35] = float(cov[35]) + self._process_yaw_noise_per_s * dt

    def _motion(self, now):
        if self._last_velocity is None or self._last_velocity_receipt is None:
            return 0.0, 0.0

        velocity_age = (now - self._last_velocity_receipt).nanoseconds / 1e9
        if velocity_age > self._vehicle_status_timeout:
            return 0.0, 0.0

        velocity = float(self._last_velocity.longitudinal_velocity)
        if not math.isfinite(velocity):
            velocity = 0.0

        yaw_rate = float(self._last_velocity.heading_rate)
        if math.isfinite(yaw_rate):
            return velocity, yaw_rate

        return velocity, self._yaw_rate_from_steering(now, velocity)

    def _yaw_rate_from_steering(self, now, velocity):
        if self._last_steering is None or self._last_steering_receipt is None:
            return 0.0

        steering_age = (now - self._last_steering_receipt).nanoseconds / 1e9
        if steering_age > self._vehicle_status_timeout:
            return 0.0

        steering = float(self._last_steering.steering_tire_angle)
        if not math.isfinite(steering) or self._wheel_base <= 0.0:
            return 0.0
        return velocity * math.tan(steering) / self._wheel_base

    def _state_to_msg(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = self._state["x"]
        msg.pose.pose.position.y = self._state["y"]
        msg.pose.pose.position.z = self._state["z"]
        msg.pose.pose.orientation = _yaw_to_quaternion(self._state["yaw"])
        msg.pose.covariance = copy.copy(self._state["covariance"])
        return msg

    def _ndt_is_lost(self, now):
        if self._last_ndt_receipt is None:
            return False
        return (now - self._last_ndt_receipt).nanoseconds / 1e9 > self._ndt_lost_timeout

    @staticmethod
    def _pose_is_usable(msg):
        pose = msg.pose.pose
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not all(math.isfinite(float(value)) for value in values):
            return False
        norm = math.sqrt(
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w
        )
        return math.isfinite(norm) and norm > 0.1


def main(args=None):
    rclpy.init(args=args)
    node = NdtInitialPosePredictor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
