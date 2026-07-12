from __future__ import annotations

import math

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import RouteState, Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    SteeringReport,
    VelocityReport,
)
from autoware_vehicle_msgs.srv import ControlModeCommand
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from autoracer_safety.race_contract import COMMAND_QOS, STATE_QOS


class RaceStackFixture(Node):
    def __init__(self) -> None:
        super().__init__("race_stack_fixture")
        self.declare_parameter("drop_topic", "")
        self.declare_parameter("drop_after_sec", 5.0)
        self.declare_parameter("drop_duration_sec", 0.0)
        self._drop_topic = str(self.get_parameter("drop_topic").value)
        self._drop_after = float(self.get_parameter("drop_after_sec").value)
        self._drop_duration = float(self.get_parameter("drop_duration_sec").value)
        self._start = self._now()
        self._tick = 0
        self._control_mode = ControlModeReport.MANUAL
        self._gear_command = GearCommand.PARK

        self._loc_state_pub = self.create_publisher(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            STATE_QOS,
        )
        self._odom_pub = self.create_publisher(
            Odometry, "/localization/kinematic_state", COMMAND_QOS
        )
        self._trajectory_pub = self.create_publisher(
            Trajectory, "/planning/trajectory", COMMAND_QOS
        )
        self._route_pub = self.create_publisher(
            RouteState, "/planning/route_state", STATE_QOS
        )
        self._raw_control_pub = self.create_publisher(
            Control, "/control/trajectory_follower/control_cmd", COMMAND_QOS
        )
        self._velocity_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", COMMAND_QOS
        )
        self._steering_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", COMMAND_QOS
        )
        self._accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", COMMAND_QOS
        )
        self._gear_pub = self.create_publisher(
            GearReport, "/vehicle/status/gear_status", COMMAND_QOS
        )
        self._control_mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", COMMAND_QOS
        )
        self.create_subscription(
            GearCommand, "/control/command/gear_cmd", self._on_gear, COMMAND_QOS
        )
        self.create_service(
            ControlModeCommand,
            "/control/control_mode_request",
            self._on_control_mode_request,
        )
        self.create_timer(0.02, self._on_timer)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _dropped(self, topic: str) -> bool:
        configured_topic = {
            "controller_recover": "controller",
            "controller_reset": "controller",
        }.get(self._drop_topic, self._drop_topic)
        elapsed = self._now() - self._start
        if configured_topic != topic or elapsed < self._drop_after:
            return False
        return self._drop_duration <= 0.0 or elapsed < self._drop_after + self._drop_duration

    def _on_gear(self, msg: GearCommand) -> None:
        self._gear_command = int(msg.command)

    def _on_control_mode_request(self, request, response):
        if request.mode == ControlModeCommand.Request.AUTONOMOUS:
            self._control_mode = ControlModeReport.AUTONOMOUS
            response.success = True
        elif request.mode in (
            ControlModeCommand.Request.MANUAL,
            ControlModeCommand.Request.NO_COMMAND,
        ):
            self._control_mode = ControlModeReport.MANUAL
            response.success = True
        else:
            response.success = False
        return response

    def _on_timer(self) -> None:
        self._tick += 1
        stamp = self.get_clock().now().to_msg()
        if not self._dropped("controller"):
            control = Control()
            control.stamp = stamp
            control.lateral.stamp = stamp
            control.longitudinal.stamp = stamp
            control.lateral.steering_tire_angle = 0.0
            control.lateral.steering_tire_rotation_rate = 0.0
            control.lateral.is_defined_steering_tire_rotation_rate = True
            control.longitudinal.velocity = 2.0
            control.longitudinal.acceleration = 0.0
            control.longitudinal.jerk = 0.0
            control.longitudinal.is_defined_acceleration = True
            control.longitudinal.is_defined_jerk = True
            self._raw_control_pub.publish(control)

        if self._tick % 2 == 0:
            if not self._dropped("localization"):
                state = LocalizationInitializationState()
                state.stamp = stamp
                state.state = LocalizationInitializationState.INITIALIZED
                self._loc_state_pub.publish(state)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "map"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = -187.736495237
                odom.pose.pose.position.y = 2.65
                odom.pose.pose.position.z = 0.309955232
                odom.pose.pose.orientation.w = 1.0
                self._odom_pub.publish(odom)

            if not self._dropped("trajectory"):
                self._trajectory_pub.publish(self._trajectory(stamp))
                route = RouteState()
                route.stamp = stamp
                route.state = RouteState.SET
                self._route_pub.publish(route)

            if not self._dropped("vehicle_status"):
                velocity = VelocityReport()
                velocity.header.stamp = stamp
                velocity.header.frame_id = "base_link"
                self._velocity_pub.publish(velocity)
                steering = SteeringReport()
                steering.stamp = stamp
                self._steering_pub.publish(steering)
                accel = AccelWithCovarianceStamped()
                accel.header.stamp = stamp
                self._accel_pub.publish(accel)
                gear = GearReport()
                gear.stamp = stamp
                gear.report = int(self._gear_command)
                self._gear_pub.publish(gear)
                mode = ControlModeReport()
                mode.stamp = stamp
                mode.mode = self._control_mode
                self._control_mode_pub.publish(mode)

    @staticmethod
    def _trajectory(stamp) -> Trajectory:
        trajectory = Trajectory()
        trajectory.header.stamp = stamp
        trajectory.header.frame_id = "map"
        for index in range(81):
            point = TrajectoryPoint()
            point.pose.position.x = -187.736495237 + 0.5 * index
            point.pose.position.y = 2.65
            point.pose.position.z = 0.309955232
            point.pose.orientation.w = 1.0
            point.longitudinal_velocity_mps = 2.0
            point.acceleration_mps2 = 0.0
            point.time_from_start.sec = int(math.floor(index * 0.25))
            point.time_from_start.nanosec = int((index * 0.25 % 1.0) * 1.0e9)
            trajectory.points.append(point)
        return trajectory


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceStackFixture()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
