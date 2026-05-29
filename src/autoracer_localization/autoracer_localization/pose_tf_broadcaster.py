import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseTfBroadcaster(Node):
    def __init__(self):
        super().__init__("pose_tf_broadcaster")
        self.declare_parameter("input_pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "base_link")

        self._parent_frame = self.get_parameter("parent_frame").value
        self._child_frame = self.get_parameter("child_frame").value
        input_topic = self.get_parameter("input_pose_topic").value

        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped, input_topic, self._on_pose, 10)
        self.get_logger().info(
            f"Broadcasting {self._parent_frame} -> {self._child_frame} from {input_topic}"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = PoseTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

