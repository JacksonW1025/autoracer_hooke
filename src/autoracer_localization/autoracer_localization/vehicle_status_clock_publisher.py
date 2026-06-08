import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def clock_from_velocity_status(msg: VelocityReport) -> Clock:
    clock = Clock()
    clock.clock = msg.header.stamp
    return clock


class VehicleStatusClockPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__("vehicle_status_clock_publisher", **kwargs)
        self.declare_parameter("input_velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("clock_topic", "/clock")
        self._publisher = self.create_publisher(Clock, self.get_parameter("clock_topic").value, 10)
        self.create_subscription(
            VelocityReport,
            self.get_parameter("input_velocity_topic").value,
            self._on_velocity,
            10,
        )
        self.get_logger().info(
            "Publishing /clock from vehicle-status header stamps; no Fixposition/GT seed dependency"
        )

    def _on_velocity(self, msg: VelocityReport):
        self._publisher.publish(clock_from_velocity_status(msg))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusClockPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
