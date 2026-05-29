from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, HazardLightsCommand, TurnIndicatorsCommand
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes", "on")


class CommandGate(Node):
    def __init__(self):
        super().__init__("command_gate")
        self.declare_parameter("enable_drive_commands", False)
        self.declare_parameter("input_topic", "/autoracer/control/raw_control_cmd")
        self.declare_parameter("output_topic", "/control/command/control_cmd")
        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("gear_topic", "/control/command/gear_cmd")
        self.declare_parameter("hazard_topic", "/control/command/hazard_lights_cmd")
        self.declare_parameter("turn_topic", "/control/command/turn_indicators_cmd")
        self.declare_parameter("state_topic", "/autoracer/safety/state")
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("localization_timeout_sec", 1.0)
        self.declare_parameter("max_speed_mps", 1.5)
        self.declare_parameter("max_accel_mps2", 0.8)
        self.declare_parameter("max_decel_mps2", -1.5)
        self.declare_parameter("max_steer_rad", 0.488)
        self.declare_parameter("max_steer_rate_radps", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)

        self._enabled = _as_bool(self.get_parameter("enable_drive_commands").value)
        self._command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self._localization_timeout = float(self.get_parameter("localization_timeout_sec").value)
        self._max_speed = float(self.get_parameter("max_speed_mps").value)
        self._max_accel = float(self.get_parameter("max_accel_mps2").value)
        self._max_decel = float(self.get_parameter("max_decel_mps2").value)
        self._max_steer = float(self.get_parameter("max_steer_rad").value)
        self._max_steer_rate = float(self.get_parameter("max_steer_rate_radps").value)

        self._raw_command = None
        self._last_raw_time = None
        self._last_pose_time = None
        self._last_output_time = None
        self._last_steer = 0.0

        self._command_pub = self.create_publisher(
            Control, self.get_parameter("output_topic").value, 10
        )
        self._gear_pub = self.create_publisher(
            GearCommand, self.get_parameter("gear_topic").value, 10
        )
        self._hazard_pub = self.create_publisher(
            HazardLightsCommand, self.get_parameter("hazard_topic").value, 10
        )
        self._turn_pub = self.create_publisher(
            TurnIndicatorsCommand, self.get_parameter("turn_topic").value, 10
        )
        self._state_pub = self.create_publisher(
            String, self.get_parameter("state_topic").value, 10
        )

        self.create_subscription(
            Control, self.get_parameter("input_topic").value, self._on_raw_command, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._on_pose, 10
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._on_timer)
        self.get_logger().warn(f"enable_drive_commands={self._enabled}")

    def _on_raw_command(self, msg):
        self._raw_command = msg
        self._last_raw_time = self.get_clock().now()

    def _on_pose(self, _msg):
        self._last_pose_time = self.get_clock().now()

    def _age_sec(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _on_timer(self):
        reason = self._unsafe_reason()
        if reason is None:
            command = self._limited_command(self._raw_command)
            state = "drive_enabled"
        else:
            command = self._stop_command()
            state = reason

        self._publish_support_commands(reason is None)
        self._command_pub.publish(command)
        self._state_pub.publish(String(data=state))

    def _unsafe_reason(self):
        if not self._enabled:
            return "drive_disabled"
        raw_age = self._age_sec(self._last_raw_time)
        if raw_age is None or raw_age > self._command_timeout:
            return "raw_command_timeout"
        pose_age = self._age_sec(self._last_pose_time)
        if pose_age is None or pose_age > self._localization_timeout:
            return "localization_timeout"
        return None

    def _limited_command(self, raw):
        command = Control()
        now = self.get_clock().now()
        stamp = now.to_msg()
        command.stamp = stamp
        command.lateral.stamp = stamp
        command.longitudinal.stamp = stamp

        command.longitudinal.velocity = _clamp(
            raw.longitudinal.velocity, 0.0, self._max_speed
        )
        command.longitudinal.acceleration = _clamp(
            raw.longitudinal.acceleration, self._max_decel, self._max_accel
        )
        command.longitudinal.jerk = raw.longitudinal.jerk
        command.longitudinal.is_defined_acceleration = True
        command.longitudinal.is_defined_jerk = raw.longitudinal.is_defined_jerk

        target_steer = _clamp(
            raw.lateral.steering_tire_angle, -self._max_steer, self._max_steer
        )
        if self._last_output_time is not None:
            dt = max((now - self._last_output_time).nanoseconds * 1e-9, 0.001)
            steer_delta = self._max_steer_rate * dt
            target_steer = _clamp(
                target_steer, self._last_steer - steer_delta, self._last_steer + steer_delta
            )

        command.lateral.steering_tire_angle = target_steer
        command.lateral.steering_tire_rotation_rate = raw.lateral.steering_tire_rotation_rate
        command.lateral.is_defined_steering_tire_rotation_rate = (
            raw.lateral.is_defined_steering_tire_rotation_rate
        )

        self._last_output_time = now
        self._last_steer = target_steer
        return command

    def _stop_command(self):
        command = Control()
        now = self.get_clock().now()
        stamp = now.to_msg()
        command.stamp = stamp
        command.lateral.stamp = stamp
        command.longitudinal.stamp = stamp
        command.longitudinal.velocity = 0.0
        command.longitudinal.acceleration = self._max_decel
        command.longitudinal.is_defined_acceleration = True
        command.lateral.steering_tire_angle = 0.0
        self._last_output_time = now
        self._last_steer = 0.0
        return command

    def _publish_support_commands(self, safe):
        stamp = self.get_clock().now().to_msg()

        gear = GearCommand()
        gear.stamp = stamp
        gear.command = GearCommand.DRIVE if safe else GearCommand.NEUTRAL
        self._gear_pub.publish(gear)

        hazard = HazardLightsCommand()
        hazard.stamp = stamp
        hazard.command = HazardLightsCommand.DISABLE if safe else HazardLightsCommand.ENABLE
        self._hazard_pub.publish(hazard)

        turn = TurnIndicatorsCommand()
        turn.stamp = stamp
        turn.command = TurnIndicatorsCommand.DISABLE
        self._turn_pub.publish(turn)


def main():
    rclpy.init()
    node = CommandGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

