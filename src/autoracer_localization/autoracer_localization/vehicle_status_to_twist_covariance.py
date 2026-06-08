import math

import rclpy
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .ndt_initial_pose_predictor import _message_time


def twist_covariance_from_status(
    velocity_msg,
    steering_msg,
    *,
    wheel_base_m,
    longitudinal_variance_m2ps2,
    lateral_variance_m2ps2,
    yaw_rate_variance_rad2ps2,
    steering_timeout_sec,
):
    out = TwistWithCovarianceStamped()
    out.header.stamp = velocity_msg.header.stamp
    out.header.frame_id = velocity_msg.header.frame_id or "base_link"
    out.twist.twist.linear.x = float(velocity_msg.longitudinal_velocity)
    out.twist.twist.linear.y = float(velocity_msg.lateral_velocity)

    yaw_rate = float(velocity_msg.heading_rate)
    if abs(yaw_rate) < 1e-9 and steering_msg is not None:
        velocity_stamp = _message_time(velocity_msg, rclpy.time.Time())
        steering_stamp = _message_time(steering_msg, rclpy.time.Time())
        steering_age = abs((velocity_stamp - steering_stamp).nanoseconds / 1e9)
        if steering_timeout_sec <= 0.0 or steering_age <= steering_timeout_sec:
            wheel_base = max(float(wheel_base_m), 1e-6)
            yaw_rate = float(velocity_msg.longitudinal_velocity) * math.tan(
                float(steering_msg.steering_tire_angle)
            ) / wheel_base
    out.twist.twist.angular.z = yaw_rate

    out.twist.covariance[0] = max(0.0, float(longitudinal_variance_m2ps2))
    out.twist.covariance[7] = max(0.0, float(lateral_variance_m2ps2))
    out.twist.covariance[35] = max(0.0, float(yaw_rate_variance_rad2ps2))
    return out


class VehicleStatusToTwistCovariance(Node):
    def __init__(self):
        super().__init__("vehicle_status_to_twist_covariance")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("steering_topic", "/vehicle/status/steering_status")
        self.declare_parameter("output_topic", "/vehicle/status/twist_with_covariance")
        self.declare_parameter("wheel_base_m", 1.9)
        self.declare_parameter("steering_timeout_sec", 0.1)
        self.declare_parameter("longitudinal_variance_m2ps2", 0.04)
        self.declare_parameter("lateral_variance_m2ps2", 0.25)
        self.declare_parameter("yaw_rate_variance_rad2ps2", 0.04)

        self._wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self._steering_timeout_sec = float(
            self.get_parameter("steering_timeout_sec").value
        )
        self._longitudinal_variance = float(
            self.get_parameter("longitudinal_variance_m2ps2").value
        )
        self._lateral_variance = float(
            self.get_parameter("lateral_variance_m2ps2").value
        )
        self._yaw_rate_variance = float(
            self.get_parameter("yaw_rate_variance_rad2ps2").value
        )
        self._last_steering = None
        self._publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            self.get_parameter("output_topic").value,
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

    def _on_steering(self, msg):
        self._last_steering = msg

    def _on_velocity(self, msg):
        self._publisher.publish(
            twist_covariance_from_status(
                msg,
                self._last_steering,
                wheel_base_m=self._wheel_base_m,
                longitudinal_variance_m2ps2=self._longitudinal_variance,
                lateral_variance_m2ps2=self._lateral_variance,
                yaw_rate_variance_rad2ps2=self._yaw_rate_variance,
                steering_timeout_sec=self._steering_timeout_sec,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusToTwistCovariance()
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
