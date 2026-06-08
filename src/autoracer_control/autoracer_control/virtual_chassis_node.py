"""ROS wrapper for the virtual chassis plant."""

from __future__ import annotations

import math

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import AccelWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from autoracer_control.virtual_chassis_model import (
    RawControlCommand,
    VirtualChassisConfig,
    VirtualChassisModel,
    VirtualChassisState,
)


RAW_CONTROL_TOPIC = "/control_bench/autoracer/control/raw_control_cmd"
ODOMETRY_TOPIC = "/control_bench/localization/kinematic_state"
STEERING_TOPIC = "/control_bench/vehicle/status/steering_status"
ACCELERATION_TOPIC = "/control_bench/localization/acceleration"


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class VirtualChassisNode(Node):
    def __init__(self) -> None:
        super().__init__("virtual_chassis_node")

        self.declare_parameter("raw_control_topic", RAW_CONTROL_TOPIC)
        self.declare_parameter("odometry_topic", ODOMETRY_TOPIC)
        self.declare_parameter("steering_topic", STEERING_TOPIC)
        self.declare_parameter("acceleration_topic", ACCELERATION_TOPIC)

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("initial_v", 0.0)
        self.declare_parameter("initial_delta", 0.0)
        self.declare_parameter("initial_a", 0.0)

        self.declare_parameter("wheel_base", 1.9)
        self.declare_parameter("max_steer", 0.488)
        self.declare_parameter("max_steer_rate", 1.0)
        self.declare_parameter("steer_tau", 0.15)
        self.declare_parameter("actuator_input_delay", 0.15)
        self.declare_parameter("max_speed", 3.0)
        self.declare_parameter("max_acc", 1.0)
        self.declare_parameter("min_acc", -2.0)
        self.declare_parameter("max_jerk", 2.0)
        self.declare_parameter("min_jerk", -4.0)
        self.declare_parameter("acc_tau", 0.20)
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("fixed_speed_mode", False)
        self.declare_parameter("fixed_speed", 1.0)

        config = VirtualChassisConfig(
            wheel_base=float(self.get_parameter("wheel_base").value),
            max_steer=float(self.get_parameter("max_steer").value),
            max_steer_rate=float(self.get_parameter("max_steer_rate").value),
            steer_tau=float(self.get_parameter("steer_tau").value),
            actuator_input_delay=float(self.get_parameter("actuator_input_delay").value),
            max_speed=float(self.get_parameter("max_speed").value),
            max_acc=float(self.get_parameter("max_acc").value),
            min_acc=float(self.get_parameter("min_acc").value),
            max_jerk=float(self.get_parameter("max_jerk").value),
            min_jerk=float(self.get_parameter("min_jerk").value),
            acc_tau=float(self.get_parameter("acc_tau").value),
            dt=float(self.get_parameter("dt").value),
            fixed_speed_mode=bool(self.get_parameter("fixed_speed_mode").value),
            fixed_speed=float(self.get_parameter("fixed_speed").value),
        )
        initial_state = VirtualChassisState(
            x=float(self.get_parameter("initial_x").value),
            y=float(self.get_parameter("initial_y").value),
            yaw=float(self.get_parameter("initial_yaw").value),
            v=float(self.get_parameter("initial_v").value),
            delta_actual=float(self.get_parameter("initial_delta").value),
            a_actual=float(self.get_parameter("initial_a").value),
        )
        self._model = VirtualChassisModel(config, initial_state)
        self._last_command = RawControlCommand()

        self.create_subscription(
            Control,
            str(self.get_parameter("raw_control_topic").value),
            self._on_control,
            10,
        )
        self._odometry_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odometry_topic").value), 10
        )
        self._steering_pub = self.create_publisher(
            SteeringReport, str(self.get_parameter("steering_topic").value), 10
        )
        self._accel_pub = self.create_publisher(
            AccelWithCovarianceStamped,
            str(self.get_parameter("acceleration_topic").value),
            10,
        )
        self.create_timer(config.dt, self._on_timer)
        self._publish_feedback(self._model.state)

    def _on_control(self, msg: Control) -> None:
        self._last_command = RawControlCommand(
            steering_tire_angle=float(msg.lateral.steering_tire_angle),
            velocity=float(msg.longitudinal.velocity),
            acceleration=float(msg.longitudinal.acceleration),
        )

    def _on_timer(self) -> None:
        self._publish_feedback(self._model.step(self._last_command))

    def _publish_feedback(self, state: VirtualChassisState) -> None:
        now = self.get_clock().now().to_msg()

        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = "map"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = state.x
        odometry.pose.pose.position.y = state.y
        odometry.pose.pose.orientation = _yaw_to_quaternion(state.yaw)
        odometry.twist.twist.linear.x = state.v
        odometry.twist.twist.angular.z = (
            state.v
            / float(self.get_parameter("wheel_base").value)
            * math.tan(state.delta_actual)
        )
        self._odometry_pub.publish(odometry)

        steering = SteeringReport()
        steering.stamp = now
        steering.steering_tire_angle = state.delta_actual
        self._steering_pub.publish(steering)

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = now
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear.x = state.a_actual
        self._accel_pub.publish(accel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VirtualChassisNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
