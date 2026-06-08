import copy

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def clone_pose(msg: PoseWithCovarianceStamped, frame_id: str) -> PoseWithCovarianceStamped:
    output = copy.deepcopy(msg)
    if frame_id:
        output.header.frame_id = frame_id
    return output


class GroundTruthLocalizationRelay(Node):
    def __init__(self):
        super().__init__("ground_truth_localization_relay")
        self.declare_parameter("input_topic", "/carmaker/ground_truth/pose")
        self.declare_parameter("output_topic", "/localization/pose_with_covariance")
        self.declare_parameter("frame_id", "map")

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_topic").value,
            self._on_pose,
            10,
        )
        self.get_logger().info(
            "Stage B ground-truth localization relay: "
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def _on_pose(self, msg):
        self._publisher.publish(clone_pose(msg, self._frame_id))


def _shutdown_if_context_ok():
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError as exc:
        if not _is_shutdown_rcl_error(exc):
            raise


def _is_shutdown_rcl_error(exc: RCLError) -> bool:
    text = str(exc)
    return (
        "rcl_shutdown already called" in text
        or "context is not valid" in text
        or "rcl_init() was not called or rcl_shutdown() was called" in text
    )


def main():
    rclpy.init()
    node = GroundTruthLocalizationRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as exc:
        if not _is_shutdown_rcl_error(exc):
            raise
    finally:
        node.destroy_node()
        _shutdown_if_context_ok()


if __name__ == "__main__":
    main()
