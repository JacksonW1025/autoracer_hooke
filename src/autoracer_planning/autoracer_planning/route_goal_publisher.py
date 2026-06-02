import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml


def goal_pose_from_yaml(path) -> PoseStamped:
    route_goal = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    goal = route_goal["goal"]
    position = goal["position"]
    yaw = float(goal.get("yaw", 0.0))

    pose = PoseStamped()
    pose.header.frame_id = route_goal.get("frame_id", "map")
    pose.pose.position.x = float(position["x"])
    pose.pose.position.y = float(position["y"])
    pose.pose.position.z = float(position.get("z", 0.0))
    pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.orientation.w = math.cos(yaw * 0.5)
    return pose


class RouteGoalPublisher(Node):
    def __init__(self):
        super().__init__("route_goal_publisher")
        self.declare_parameter("route_goal_path", "")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("max_publish_count", 10)

        route_goal_path = self.get_parameter("route_goal_path").value
        if not route_goal_path:
            raise RuntimeError("route_goal_path parameter is required")
        self._goal_pose = goal_pose_from_yaml(route_goal_path)
        self._pose_seen = False
        self._publish_count = 0
        self._max_publish_count = int(self.get_parameter("max_publish_count").value)

        self._publisher = self.create_publisher(
            PoseStamped, self.get_parameter("goal_pose_topic").value, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            10,
        )

        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(period, self._on_timer)

    def _on_pose(self, _msg):
        self._pose_seen = True

    def _on_timer(self):
        if not self._pose_seen:
            return
        if self._publish_count >= self._max_publish_count:
            return
        if self._publisher.get_subscription_count() == 0:
            return

        self._goal_pose.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._goal_pose)
        self._publish_count += 1
        if self._publish_count == 1:
            position = self._goal_pose.pose.position
            self.get_logger().info(
                "Published route goal: "
                f"frame={self._goal_pose.header.frame_id} "
                f"position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
            )


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
    node = RouteGoalPublisher()
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
