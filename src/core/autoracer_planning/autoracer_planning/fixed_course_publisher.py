import math
from pathlib import Path

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autoracer_planning.course_asset import load_runtime_course_asset


def _duration_from_seconds(seconds: float) -> Duration:
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1_000_000_000))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000
    return duration


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)
    return quaternion


def course_to_trajectory(manifest: dict, samples) -> Trajectory:
    trajectory = Trajectory()
    trajectory.header.frame_id = manifest["frame_id"]
    elapsed = 0.0
    for index, sample in enumerate(samples):
        if index > 0:
            previous = samples[index - 1]
            ds = sample.s - previous.s
            average_speed = 0.5 * (
                previous.target_velocity + sample.target_velocity
            )
            elapsed += ds / max(average_speed, 0.1)
        point = TrajectoryPoint()
        point.time_from_start = _duration_from_seconds(elapsed)
        point.pose.position.x = sample.x
        point.pose.position.y = sample.y
        point.pose.position.z = sample.z
        point.pose.orientation = _yaw_to_quaternion(sample.yaw)
        point.longitudinal_velocity_mps = sample.target_velocity
        point.acceleration_mps2 = sample.target_acceleration
        trajectory.points.append(point)
    return trajectory


class FixedCoursePublisher(Node):
    def __init__(self):
        super().__init__("fixed_course_publisher")
        self.declare_parameter("course_path", "")
        self.declare_parameter("map_path", "")
        self.declare_parameter("trajectory_topic", "/planning/global_trajectory")

        course_path_value = str(self.get_parameter("course_path").value)
        if not course_path_value:
            raise RuntimeError("course_path parameter is required")
        course_path = Path(course_path_value)
        map_path_value = str(self.get_parameter("map_path").value)
        if not map_path_value:
            raise RuntimeError("map_path parameter is required")
        manifest, samples = load_runtime_course_asset(course_path, Path(map_path_value))
        if manifest.get("frame_id") != "map":
            raise RuntimeError(f"fixed course frame must be map: {manifest.get('frame_id')!r}")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            Trajectory, str(self.get_parameter("trajectory_topic").value), qos
        )
        self._trajectory = course_to_trajectory(manifest, samples)
        self._trajectory.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._trajectory)
        self.get_logger().info(
            "Published validated fixed course: "
            f"version={manifest['version']} points={len(samples)} "
            f"length_m={samples[-1].s:.3f} sha256="
            f"{manifest['assets']['course.csv']['sha256']}"
        )


def _shutdown_if_context_ok():
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError as exc:
        if "rcl_shutdown already called" not in str(exc):
            raise


def main():
    rclpy.init()
    node = FixedCoursePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        _shutdown_if_context_ok()


if __name__ == "__main__":
    main()
