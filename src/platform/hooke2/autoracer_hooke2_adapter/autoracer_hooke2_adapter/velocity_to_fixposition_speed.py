import math

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from fixposition_driver_msgs.msg import Speed, WheelSensor
from rclpy.node import Node


def _clamp_to_int32(value):
    return max(-(2**31), min((2**31) - 1, value))


class VelocityToFixpositionSpeed(Node):
    def __init__(self):
        super().__init__("velocity_to_fixposition_speed")

        self.declare_parameter("input_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("output_topic", "/fixposition/speed")
        self.declare_parameter("sensor_location", "RC")
        self.declare_parameter("max_abs_speed_mps", 40.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._sensor_location = self.get_parameter("sensor_location").value
        self._max_abs_speed = float(self.get_parameter("max_abs_speed_mps").value)

        self._publisher = self.create_publisher(Speed, output_topic, 10)
        self.create_subscription(VelocityReport, input_topic, self._on_velocity, 10)

        self.get_logger().info(
            f"Bridge {input_topic} -> {output_topic} as Fixposition {self._sensor_location} speed"
        )

    def _on_velocity(self, msg):
        velocity = float(msg.longitudinal_velocity)
        valid = math.isfinite(velocity) and abs(velocity) <= self._max_abs_speed

        if not valid:
            self.get_logger().warn(
                f"Invalid vehicle speed for Fixposition: {velocity}",
                throttle_duration_sec=1.0,
            )
            velocity = 0.0

        sensor = WheelSensor()
        sensor.header = msg.header
        sensor.location = self._sensor_location
        sensor.vx = _clamp_to_int32(int(round(velocity * 1000.0)))
        sensor.vy = 0
        sensor.vz = 0
        sensor.vx_valid = valid
        sensor.vy_valid = False
        sensor.vz_valid = False

        speed = Speed()
        speed.sensors.append(sensor)
        self._publisher.publish(speed)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityToFixpositionSpeed()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
