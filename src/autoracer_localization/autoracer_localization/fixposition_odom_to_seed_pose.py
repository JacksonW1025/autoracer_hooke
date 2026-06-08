import copy
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

BASE_TO_GNSS_TRANSLATION = (1.90, 0.0, 1.037)
BASE_TO_GNSS_YAW = -1.57079632679


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    q = PoseWithCovarianceStamped().pose.pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def gnss_pose_to_base_pose(x, y, z, yaw, *, translation=None, yaw_offset=BASE_TO_GNSS_YAW):
    tx, ty, tz = translation if translation is not None else BASE_TO_GNSS_TRANSLATION
    base_yaw = normalize_angle(float(yaw) - float(yaw_offset))
    cos_yaw = math.cos(base_yaw)
    sin_yaw = math.sin(base_yaw)
    offset_x = cos_yaw * float(tx) - sin_yaw * float(ty)
    offset_y = sin_yaw * float(tx) + cos_yaw * float(ty)
    return (
        float(x) - offset_x,
        float(y) - offset_y,
        float(z) - float(tz),
        base_yaw,
    )


def odometry_to_seed_pose(
    odom_msg,
    *,
    map_frame="map",
    translation=None,
    yaw_offset=BASE_TO_GNSS_YAW,
    reported_xy_sigma_m=None,
    reported_z_sigma_m=None,
    reported_yaw_sigma_deg=None,
):
    out = PoseWithCovarianceStamped()
    out.header.stamp = odom_msg.header.stamp
    out.header.frame_id = map_frame
    pose = odom_msg.pose.pose
    base_x, base_y, base_z, base_yaw = gnss_pose_to_base_pose(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        yaw_from_quaternion(pose.orientation),
        translation=translation,
        yaw_offset=yaw_offset,
    )
    out.pose = copy.deepcopy(odom_msg.pose)
    out.pose.pose.position.x = base_x
    out.pose.pose.position.y = base_y
    out.pose.pose.position.z = base_z
    out.pose.pose.orientation = yaw_to_quaternion(base_yaw)
    _apply_reported_covariance(
        out,
        reported_xy_sigma_m=reported_xy_sigma_m,
        reported_z_sigma_m=reported_z_sigma_m,
        reported_yaw_sigma_deg=reported_yaw_sigma_deg,
    )
    return out


def _apply_reported_covariance(
    msg,
    *,
    reported_xy_sigma_m=None,
    reported_z_sigma_m=None,
    reported_yaw_sigma_deg=None,
):
    if reported_xy_sigma_m is not None and float(reported_xy_sigma_m) >= 0.0:
        variance = float(reported_xy_sigma_m) ** 2
        msg.pose.covariance[0] = variance
        msg.pose.covariance[7] = variance
    if reported_z_sigma_m is not None and float(reported_z_sigma_m) >= 0.0:
        msg.pose.covariance[14] = float(reported_z_sigma_m) ** 2
    if reported_yaw_sigma_deg is not None and float(reported_yaw_sigma_deg) >= 0.0:
        msg.pose.covariance[35] = math.radians(float(reported_yaw_sigma_deg)) ** 2


class FixpositionOdomToSeedPose(Node):
    def __init__(self):
        super().__init__("fixposition_odom_to_seed_pose")

        self.declare_parameter("input_odom_topic", "/fixposition/odometry_enu")
        self.declare_parameter("output_pose_topic", "/fixposition/pose_with_covariance")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_to_gnss_x", BASE_TO_GNSS_TRANSLATION[0])
        self.declare_parameter("base_to_gnss_y", BASE_TO_GNSS_TRANSLATION[1])
        self.declare_parameter("base_to_gnss_z", BASE_TO_GNSS_TRANSLATION[2])
        self.declare_parameter("base_to_gnss_yaw", BASE_TO_GNSS_YAW)
        self.declare_parameter("reported_xy_sigma_m", -1.0)
        self.declare_parameter("reported_z_sigma_m", -1.0)
        self.declare_parameter("reported_yaw_sigma_deg", -1.0)
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("clock_topic", "/clock")

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._translation = (
            float(self.get_parameter("base_to_gnss_x").value),
            float(self.get_parameter("base_to_gnss_y").value),
            float(self.get_parameter("base_to_gnss_z").value),
        )
        self._yaw_offset = float(self.get_parameter("base_to_gnss_yaw").value)
        self._reported_xy_sigma_m = float(self.get_parameter("reported_xy_sigma_m").value)
        self._reported_z_sigma_m = float(self.get_parameter("reported_z_sigma_m").value)
        self._reported_yaw_sigma_deg = float(
            self.get_parameter("reported_yaw_sigma_deg").value
        )
        self._pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_pose_topic").value,
            10,
        )
        self._clock_publisher = None
        if bool(self.get_parameter("publish_clock").value):
            self._clock_publisher = self.create_publisher(
                Clock,
                self.get_parameter("clock_topic").value,
                10,
            )
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry,
            self.get_parameter("input_odom_topic").value,
            self._on_odom,
            odom_qos,
        )
        self.get_logger().info(
            "Converting Fixposition odometry to base_link seed pose "
            f"{self.get_parameter('input_odom_topic').value} -> "
            f"{self.get_parameter('output_pose_topic').value}"
        )

    def _on_odom(self, msg):
        if self._clock_publisher is not None:
            clock = Clock()
            clock.clock = msg.header.stamp
            self._clock_publisher.publish(clock)
        self._pose_publisher.publish(
            odometry_to_seed_pose(
                msg,
                map_frame=self._map_frame,
                translation=self._translation,
                yaw_offset=self._yaw_offset,
                reported_xy_sigma_m=self._reported_xy_sigma_m,
                reported_z_sigma_m=self._reported_z_sigma_m,
                reported_yaw_sigma_deg=self._reported_yaw_sigma_deg,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = FixpositionOdomToSeedPose()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
