import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


def clock_from_pointcloud(msg: PointCloud2) -> Clock:
    clock = Clock()
    clock.clock = msg.header.stamp
    return clock


class PointcloudClockPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__("pointcloud_clock_publisher", **kwargs)
        self.declare_parameter("input_pointcloud_topic", "/sensing/lidar/concatenated/pointcloud")
        self.declare_parameter("clock_topic", "/clock")
        self._publisher = self.create_publisher(Clock, self.get_parameter("clock_topic").value, 10)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            PointCloud2,
            self.get_parameter("input_pointcloud_topic").value,
            self._on_pointcloud,
            qos,
        )
        self.get_logger().info(
            "Publishing /clock from pointcloud header stamps; no Fixposition/GT dependency"
        )

    def _on_pointcloud(self, msg: PointCloud2):
        self._publisher.publish(clock_from_pointcloud(msg))


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudClockPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
