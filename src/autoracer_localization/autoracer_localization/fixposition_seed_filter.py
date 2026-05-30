import math

import rclpy
from fixposition_driver_msgs.msg import FpaOdomstatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def _distance_xy(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _xy_stddev(covariance):
    var_x = float(covariance[0])
    var_y = float(covariance[7])
    if not math.isfinite(var_x) or not math.isfinite(var_y) or var_x < 0.0 or var_y < 0.0:
        return math.inf
    return math.sqrt(max(var_x, var_y))


def _pose_is_finite(msg):
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
    q_norm = math.sqrt(
        pose.orientation.x * pose.orientation.x
        + pose.orientation.y * pose.orientation.y
        + pose.orientation.z * pose.orientation.z
        + pose.orientation.w * pose.orientation.w
    )
    return math.isfinite(q_norm) and q_norm > 0.1


def _status_is_good(status):
    consts = status.consts
    if status.init_status != consts.INIT_STATUS_GLOBAL_INIT:
        return False

    good_gnss = {
        consts.GNSS_STATUS_RTK_FLOAT,
        consts.GNSS_STATUS_RTK_FIXED,
    }
    return status.gnss1_status in good_gnss or status.gnss2_status in good_gnss


class FixpositionSeedFilter(Node):
    def __init__(self):
        super().__init__("fixposition_seed_filter")

        self.declare_parameter("input_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("input_status_topic", "/fixposition/fpa/odomstatus")
        self.declare_parameter("output_topic", "/localization/fixposition/seed_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_pose_age_sec", 1.0)
        self.declare_parameter("max_xy_stddev_m", 3.0)
        self.declare_parameter("max_jump_m", 5.0)
        self.declare_parameter("status_timeout_sec", 2.0)
        self.declare_parameter("require_status", False)
        self.declare_parameter("use_status_when_available", True)

        input_pose_topic = self.get_parameter("input_pose_topic").value
        input_status_topic = self.get_parameter("input_status_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._map_frame = self.get_parameter("map_frame").value
        self._max_pose_age = float(self.get_parameter("max_pose_age_sec").value)
        self._max_xy_stddev = float(self.get_parameter("max_xy_stddev_m").value)
        self._max_jump = float(self.get_parameter("max_jump_m").value)
        self._status_timeout = float(self.get_parameter("status_timeout_sec").value)
        self._require_status = bool(self.get_parameter("require_status").value)
        self._use_status_when_available = bool(
            self.get_parameter("use_status_when_available").value
        )

        self._last_status = None
        self._last_status_receipt = None
        self._last_published_pose = None

        self._publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, input_pose_topic, self._on_pose, 10)
        self.create_subscription(FpaOdomstatus, input_status_topic, self._on_status, 10)

        self.get_logger().info(
            f"Filtering Fixposition seed poses {input_pose_topic} -> {output_topic}"
        )

    def _on_status(self, msg):
        self._last_status = msg
        self._last_status_receipt = self.get_clock().now()

    def _on_pose(self, msg):
        ok, reason = self._validate_pose(msg)
        if not ok:
            self.get_logger().warn(
                f"Reject Fixposition seed pose: {reason}",
                throttle_duration_sec=1.0,
            )
            return

        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self._map_frame
        out.pose = msg.pose
        self._publisher.publish(out)
        self._last_published_pose = out.pose.pose

    def _validate_pose(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self._map_frame:
            return False, f"frame_id is {msg.header.frame_id}, expected {self._map_frame}"

        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age = abs((now - stamp).nanoseconds) / 1e9
        if self._max_pose_age > 0.0 and age > self._max_pose_age:
            return False, f"pose age {age:.3f}s > {self._max_pose_age:.3f}s"

        if not _pose_is_finite(msg):
            return False, "pose contains non-finite values or invalid quaternion"

        stddev = _xy_stddev(msg.pose.covariance)
        if self._max_xy_stddev > 0.0 and stddev > self._max_xy_stddev:
            return False, f"xy covariance stddev {stddev:.3f}m > {self._max_xy_stddev:.3f}m"

        if self._last_published_pose is not None and self._max_jump > 0.0:
            jump = _distance_xy(msg.pose.pose.position, self._last_published_pose.position)
            if jump > self._max_jump:
                return False, f"xy jump {jump:.3f}m > {self._max_jump:.3f}m"

        status_ok, status_reason = self._validate_status(now)
        if not status_ok:
            return False, status_reason

        return True, ""

    def _validate_status(self, now):
        if not self._use_status_when_available and not self._require_status:
            return True, ""

        if self._last_status is None or self._last_status_receipt is None:
            if self._require_status:
                return False, "no odomstatus received"
            return True, ""

        age = (now - self._last_status_receipt).nanoseconds / 1e9
        if age > self._status_timeout:
            if self._require_status:
                return False, f"odomstatus age {age:.3f}s > {self._status_timeout:.3f}s"
            return True, ""

        if not _status_is_good(self._last_status):
            return False, (
                "odomstatus is not globally initialized with RTK float/fixed "
                f"(init={self._last_status.init_status}, "
                f"gnss1={self._last_status.gnss1_status}, "
                f"gnss2={self._last_status.gnss2_status})"
            )

        return True, ""


def main(args=None):
    rclpy.init(args=args)
    node = FixpositionSeedFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
