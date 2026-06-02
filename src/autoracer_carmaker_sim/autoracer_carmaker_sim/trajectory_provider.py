import math

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def build_trajectory(centerline: Path, speed_mps: float) -> Trajectory:
    if len(centerline.poses) < 2:
        raise ValueError("centerline path has fewer than two poses")

    trajectory = Trajectory()
    trajectory.header = centerline.header

    elapsed = 0.0
    previous = None
    for pose_stamped in centerline.poses:
        point = TrajectoryPoint()
        point.pose = pose_stamped.pose
        if previous is not None:
            elapsed += _distance(previous.pose.position, pose_stamped.pose.position) / max(speed_mps, 0.1)
        point.time_from_start = _duration(elapsed)
        point.longitudinal_velocity_mps = speed_mps
        point.acceleration_mps2 = 0.0
        trajectory.points.append(point)
        previous = pose_stamped

    return trajectory


class RoadEvalTrajectoryProvider(Node):
    def __init__(self):
        super().__init__("carmaker_trajectory_provider")
        self.declare_parameter("centerline_topic", "/carmaker/road/centerline")
        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("trajectory_topic", "/planning/trajectory")
        self.declare_parameter("speed_mps", 1.5)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._speed_mps = float(self.get_parameter("speed_mps").value)
        self._latest_centerline = None
        self._latest_pose = None
        self._publish_count = 0

        self._trajectory_pub = self.create_publisher(
            Trajectory, self.get_parameter("trajectory_topic").value, 1
        )
        self.create_subscription(
            Path,
            self.get_parameter("centerline_topic").value,
            self._on_centerline,
            1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            10,
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "CarMaker-aligned trajectory provider source=/carmaker/road/centerline "
            f"frame=map speed_mps={self._speed_mps:.3f}"
        )

    def _on_centerline(self, msg):
        self._latest_centerline = msg

    def _on_pose(self, msg):
        self._latest_pose = msg.pose.pose

    def _on_timer(self):
        if self._latest_centerline is None:
            return

        trajectory = build_trajectory(self._latest_centerline, self._speed_mps)
        trajectory.header.stamp = self.get_clock().now().to_msg()
        self._trajectory_pub.publish(trajectory)
        self._publish_count += 1

        if self._publish_count % 20 == 1:
            first = trajectory.points[0].pose.position
            last = trajectory.points[-1].pose.position
            pose_distance = self._distance_to_vehicle(first)
            self.get_logger().info(
                "Published /planning/trajectory from CarMaker RoadEval: "
                f"points={len(trajectory.points)} "
                f"start=({first.x:.2f}, {first.y:.2f}, {first.z:.2f}) "
                f"end=({last.x:.2f}, {last.y:.2f}, {last.z:.2f}) "
                f"start_vehicle_distance_m={pose_distance:.2f}"
            )

    def _distance_to_vehicle(self, point):
        if self._latest_pose is None:
            return math.nan
        return _distance(point, self._latest_pose.position)


def _distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _duration(seconds: float) -> Duration:
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int((seconds - duration.sec) * 1_000_000_000)
    return duration


def _is_shutdown_rcl_error(exc: RCLError) -> bool:
    text = str(exc)
    return (
        "rcl_shutdown already called" in text
        or "context is not valid" in text
        or "rcl_init() was not called or rcl_shutdown() was called" in text
    )


def _shutdown_if_context_ok():
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError as exc:
        if not _is_shutdown_rcl_error(exc):
            raise


def main():
    rclpy.init()
    node = RoadEvalTrajectoryProvider()
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
