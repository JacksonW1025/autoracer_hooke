import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def _yaw_to_quaternion(yaw):
    q = PoseWithCovarianceStamped().pose.pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def build_seed_pose(
    *,
    stamp,
    frame_id,
    x,
    y,
    z,
    yaw,
    xy_variance,
    z_variance,
    yaw_variance,
    roll_pitch_variance=0.01,
):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    msg.pose.pose.orientation = _yaw_to_quaternion(float(yaw))
    msg.pose.covariance[0] = float(xy_variance)
    msg.pose.covariance[7] = float(xy_variance)
    msg.pose.covariance[14] = float(z_variance)
    msg.pose.covariance[21] = float(roll_pitch_variance)
    msg.pose.covariance[28] = float(roll_pitch_variance)
    msg.pose.covariance[35] = float(yaw_variance)
    return msg


def seed_pose_from_initialpose(msg, stamp, fallback_frame_id):
    seed = deepcopy(msg)
    seed.header.stamp = stamp.to_msg()
    if not seed.header.frame_id:
        seed.header.frame_id = fallback_frame_id
    return seed


class ManualSeedPosePublisher(Node):
    def __init__(self):
        super().__init__("manual_seed_pose_publisher")

        self.declare_parameter("output_topic", "/localization/fixposition/seed_pose")
        self.declare_parameter("input_topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("publish_once", False)
        self.declare_parameter("require_input_pose", False)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("xy_variance", 1.0)
        self.declare_parameter("z_variance", 0.25)
        self.declare_parameter("yaw_variance", 0.03)
        self.declare_parameter("roll_pitch_variance", 0.01)

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("output_topic").value, 10
        )
        self._publish_once = bool(self.get_parameter("publish_once").value)
        self._require_input_pose = bool(self.get_parameter("require_input_pose").value)
        self._latest_input_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_topic").value,
            self._on_initialpose,
            10,
        )

        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"Publishing manual NDT seed on {self.get_parameter('output_topic').value}"
        )

    def _on_initialpose(self, msg):
        self._latest_input_pose = msg

    def _on_timer(self):
        msg = self._make_msg(self.get_clock().now())
        if msg is None:
            return
        self._publisher.publish(msg)
        if self._publish_once:
            self.destroy_timer(self._timer)

    def _make_msg(self, stamp):
        if self._latest_input_pose is not None:
            return seed_pose_from_initialpose(
                self._latest_input_pose,
                stamp,
                self.get_parameter("frame_id").value,
            )
        if self._require_input_pose:
            return None
        return build_seed_pose(
            stamp=stamp,
            frame_id=self.get_parameter("frame_id").value,
            x=self.get_parameter("x").value,
            y=self.get_parameter("y").value,
            z=self.get_parameter("z").value,
            yaw=self.get_parameter("yaw").value,
            xy_variance=self.get_parameter("xy_variance").value,
            z_variance=self.get_parameter("z_variance").value,
            yaw_variance=self.get_parameter("yaw_variance").value,
            roll_pitch_variance=self.get_parameter("roll_pitch_variance").value,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ManualSeedPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
