import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .ndt_initial_pose_predictor import _message_time


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


class FixpositionStartupSeedGate(Node):
    def __init__(self):
        super().__init__("fixposition_startup_seed_gate")
        self.declare_parameter("input_seed_topic", "/localization/fixposition/seed_pose")
        self.declare_parameter("lock_topic", "/localization/ndt/raw_pose_with_covariance")
        self.declare_parameter(
            "output_topic", "/localization/fixposition/startup_only_seed_pose"
        )

        self._locked = False
        self._first_lock_stamp = None
        self._last_forwarded_seed_stamp = None
        self._forwarded_count = 0
        self._suppressed_count = 0
        self._ignored_unusable_seed_count = 0
        self._ignored_unusable_lock_count = 0

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_seed_topic").value,
            self._on_seed,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("lock_topic").value,
            self._on_lock,
            10,
        )
        self.get_logger().info(
            "Forwarding Fixposition seed only until first valid NDT lock: "
            f"{self.get_parameter('input_seed_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def _on_seed(self, msg):
        if self._locked:
            self._suppressed_count += 1
            return
        if not _pose_is_usable(msg):
            self._ignored_unusable_seed_count += 1
            self.get_logger().warn("Ignoring unusable startup seed", throttle_duration_sec=1.0)
            return
        stamp = _message_time(msg, self.get_clock().now())
        self._publisher.publish(msg)
        self._last_forwarded_seed_stamp = stamp
        self._forwarded_count += 1

    def _on_lock(self, msg):
        if self._locked:
            return
        if not _pose_is_usable(msg):
            self._ignored_unusable_lock_count += 1
            return
        self._first_lock_stamp = _message_time(msg, self.get_clock().now())
        self._locked = True
        self.get_logger().info(
            "Startup-only Fixposition seed gate locked after first valid NDT pose; "
            f"forwarded={self._forwarded_count}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FixpositionStartupSeedGate()
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
