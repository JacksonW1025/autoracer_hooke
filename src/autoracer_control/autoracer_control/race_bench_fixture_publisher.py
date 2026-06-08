import math

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import (
    AccelWithCovarianceStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster


OPERATION_MODE_QOS = "transient_local"

SCENARIOS = {
    "straight",
    "left_curve",
    "right_curve",
    "current_speed_low",
    "current_speed_high",
    "missing_trajectory",
    "missing_odometry",
    "missing_steering",
    "missing_acceleration",
    "missing_operation_mode",
    "stale_pose",
    "raw_timeout",
}


def _yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _trajectory_point(x, y, yaw, velocity):
    point = TrajectoryPoint()
    point.pose.position.x = float(x)
    point.pose.position.y = float(y)
    point.pose.orientation = _yaw_to_quaternion(yaw)
    point.longitudinal_velocity_mps = float(velocity)
    return point


def _yaw_for_index(points, index):
    if index == 0:
        x0, y0 = points[0]
        x1, y1 = points[1]
    elif index == len(points) - 1:
        x0, y0 = points[-2]
        x1, y1 = points[-1]
    else:
        x0, y0 = points[index - 1]
        x1, y1 = points[index + 1]
    return math.atan2(y1 - y0, x1 - x0)


class RaceBenchFixturePublisher(Node):
    def __init__(self):
        super().__init__("race_bench_fixture_publisher")
        self.declare_parameter("scenario", "straight")
        self.declare_parameter(
            "reference_trajectory_topic", "/control_bench/planning/trajectory"
        )
        self.declare_parameter(
            "odometry_topic", "/control_bench/localization/kinematic_state"
        )
        self.declare_parameter(
            "steering_topic", "/control_bench/vehicle/status/steering_status"
        )
        self.declare_parameter("accel_topic", "/control_bench/localization/acceleration")
        self.declare_parameter(
            "operation_mode_topic", "/control_bench/system/operation_mode/state"
        )
        self.declare_parameter(
            "pose_topic", "/control_bench/localization/pose_with_covariance"
        )
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("raw_timeout_warmup_sec", 1.2)

        self._scenario = str(self.get_parameter("scenario").value)
        if self._scenario not in SCENARIOS:
            raise ValueError(f"unsupported race bench scenario: {self._scenario}")

        self._start_time = self.get_clock().now()
        self._raw_timeout_warmup = float(
            self.get_parameter("raw_timeout_warmup_sec").value
        )

        self._trajectory_pub = self.create_publisher(
            Trajectory, self.get_parameter("reference_trajectory_topic").value, 10
        )
        self._odometry_pub = self.create_publisher(
            Odometry, self.get_parameter("odometry_topic").value, 10
        )
        self._steering_pub = self.create_publisher(
            SteeringReport, self.get_parameter("steering_topic").value, 10
        )
        self._accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, self.get_parameter("accel_topic").value, 10
        )
        operation_mode_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._operation_mode_pub = self.create_publisher(
            OperationModeState,
            self.get_parameter("operation_mode_topic").value,
            operation_mode_qos,
        )
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(f"race bench fixture scenario={self._scenario}")

    def _on_timer(self):
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        pose_x, pose_y, pose_yaw = 0.0, 0.0, 0.0
        odom_x, odom_y, odom_yaw = pose_x, pose_y, pose_yaw

        publish_trajectory = self._scenario != "missing_trajectory"
        publish_odometry = self._scenario != "missing_odometry"
        publish_steering = self._scenario != "missing_steering"
        publish_accel = self._scenario != "missing_acceleration"
        publish_operation_mode = self._scenario != "missing_operation_mode"
        publish_pose = self._scenario != "stale_pose"
        publish_tf = True

        if self._scenario == "raw_timeout" and elapsed > self._raw_timeout_warmup:
            odom_x, odom_y = 1000.0, 1000.0

        if publish_trajectory:
            self._trajectory_pub.publish(self._make_trajectory(now))
        if publish_odometry:
            self._odometry_pub.publish(self._make_odometry(now, odom_x, odom_y, odom_yaw))
        if publish_steering:
            self._steering_pub.publish(self._make_steering(now))
        if publish_accel:
            self._accel_pub.publish(self._make_accel(now))
        if publish_operation_mode:
            self._operation_mode_pub.publish(self._make_operation_mode(now))
        if publish_pose:
            self._pose_pub.publish(self._make_pose(now, pose_x, pose_y, pose_yaw))
        if publish_tf:
            self._tf_broadcaster.sendTransform(
                self._make_transform(now, pose_x, pose_y, pose_yaw)
            )

    def _make_trajectory(self, now):
        trajectory = Trajectory()
        trajectory.header.stamp = now.to_msg()
        trajectory.header.frame_id = "map"

        target_speed = self._target_speed()
        if self._scenario == "left_curve":
            points = [(-1.0, 1.0), (0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]
        elif self._scenario == "right_curve":
            points = [(-1.0, -1.0), (0.0, 0.0), (1.0, -1.0), (2.0, -2.0)]
        else:
            points = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0), (150.0, 0.0)]

        for index, (x, y) in enumerate(points):
            trajectory.points.append(
                _trajectory_point(x, y, _yaw_for_index(points, index), target_speed)
            )
        return trajectory

    def _make_odometry(self, now, x, y, yaw):
        odometry = Odometry()
        odometry.header.stamp = now.to_msg()
        odometry.header.frame_id = "map"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation = _yaw_to_quaternion(yaw)
        odometry.twist.twist.linear.x = self._current_speed()
        return odometry

    def _make_steering(self, now):
        steering = SteeringReport()
        steering.stamp = now.to_msg()
        steering.steering_tire_angle = 0.0
        return steering

    def _make_accel(self, now):
        accel = AccelWithCovarianceStamped()
        accel.header.stamp = now.to_msg()
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear.x = 0.0
        return accel

    def _make_operation_mode(self, now):
        operation_mode = OperationModeState()
        operation_mode.stamp = now.to_msg()
        operation_mode.mode = OperationModeState.AUTONOMOUS
        operation_mode.is_autoware_control_enabled = True
        return operation_mode

    def _make_pose(self, now, x, y, yaw):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation = _yaw_to_quaternion(yaw)
        return pose

    def _make_transform(self, now, x, y, yaw):
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = _yaw_to_quaternion(yaw)
        return transform

    def _target_speed(self):
        if self._scenario == "current_speed_high":
            return 0.5
        return 1.0

    def _current_speed(self):
        if self._scenario == "current_speed_low":
            return 0.5
        if self._scenario == "current_speed_high":
            return 1.0
        return 1.0


def _shutdown_if_context_ok():
    if rclpy.ok():
        rclpy.shutdown()


def _is_shutdown_rcl_error(exc):
    text = str(exc)
    return (
        "rcl_shutdown already called" in text
        or "context is not valid" in text
        or "rcl_init() was not called or rcl_shutdown() was called" in text
    )


def main():
    rclpy.init()
    node = RaceBenchFixturePublisher()
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
