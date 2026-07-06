import math
import unittest

import rclpy
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration

from autoracer_localization.kinematic_state_publisher import KinematicStatePublisher
from autoracer_localization.ndt_initial_pose_predictor import (
    _yaw_from_quaternion,
    _yaw_to_quaternion,
)


def make_pose(stamp, *, x=0.0, y=0.0, yaw=0.0, xy_variance=0.25):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = _yaw_to_quaternion(yaw)
    msg.pose.covariance[0] = xy_variance
    msg.pose.covariance[7] = xy_variance
    msg.pose.covariance[35] = 0.01
    return msg


class KinematicStatePublisherTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._owns_rclpy_context = not rclpy.ok()
        if cls._owns_rclpy_context:
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if cls._owns_rclpy_context and rclpy.ok():
            rclpy.shutdown()

    def test_waits_for_ndt_before_publishing_odometry(self):
        node = KinematicStatePublisher()
        try:
            self.assertIsNone(node._state_to_odom(node.get_clock().now()))
        finally:
            node.destroy_node()

    def test_ndt_pose_and_velocity_form_standard_odometry(self):
        node = KinematicStatePublisher()
        try:
            now = node.get_clock().now()
            node._on_ndt_pose(make_pose(now, x=1.0, y=2.0, yaw=0.5))

            velocity = VelocityReport()
            velocity.longitudinal_velocity = 1.2
            velocity.lateral_velocity = 0.1
            velocity.heading_rate = 0.3
            node._on_velocity(velocity)

            odom = node._state_to_odom(now)
            self.assertEqual(odom.header.frame_id, "map")
            self.assertEqual(odom.child_frame_id, "base_link")
            self.assertAlmostEqual(odom.pose.pose.position.x, 1.0)
            self.assertAlmostEqual(odom.pose.pose.position.y, 2.0)
            self.assertAlmostEqual(_yaw_from_quaternion(odom.pose.pose.orientation), 0.5)
            self.assertAlmostEqual(odom.twist.twist.linear.x, 1.2)
            self.assertAlmostEqual(odom.twist.twist.linear.y, 0.1)
            self.assertAlmostEqual(odom.twist.twist.angular.z, 0.3)
        finally:
            node.destroy_node()

    def test_prediction_uses_heading_rate_then_steering_fallback(self):
        node = KinematicStatePublisher()
        try:
            now = node.get_clock().now()
            node._on_ndt_pose(make_pose(now, yaw=0.0))

            velocity = VelocityReport()
            velocity.longitudinal_velocity = 2.0
            velocity.heading_rate = 0.4
            node._on_velocity(velocity)

            motion = node._motion(now)
            self.assertEqual(motion, (2.0, 0.0, 0.4))

            velocity.heading_rate = math.nan
            steering = SteeringReport()
            steering.steering_tire_angle = 0.1
            node._wheel_base = 2.0
            node._on_steering(steering)

            motion = node._motion(now)
            self.assertAlmostEqual(motion[0], 2.0)
            self.assertAlmostEqual(motion[1], 0.0)
            self.assertAlmostEqual(motion[2], math.tan(0.1))
        finally:
            node.destroy_node()

    def test_prediction_advances_pose_and_covariance_between_ndt_updates(self):
        node = KinematicStatePublisher()
        try:
            now = node.get_clock().now()
            node._on_ndt_pose(make_pose(now, xy_variance=0.25))

            velocity = VelocityReport()
            velocity.longitudinal_velocity = 1.0
            velocity.heading_rate = 0.0
            node._on_velocity(velocity)

            node._state["stamp"] = now - Duration(seconds=1.0)
            old_cov = node._state["covariance"][0]
            node._advance_state(now)

            self.assertAlmostEqual(node._state["x"], 1.0)
            self.assertAlmostEqual(node._state["y"], 0.0)
            self.assertGreater(node._state["covariance"][0], old_cov)
        finally:
            node.destroy_node()

    def test_stale_vehicle_status_zeroes_twist_without_moving_pose(self):
        node = KinematicStatePublisher()
        try:
            now = node.get_clock().now()
            node._on_ndt_pose(make_pose(now, x=4.0))

            velocity = VelocityReport()
            velocity.longitudinal_velocity = 3.0
            velocity.heading_rate = 1.0
            node._on_velocity(velocity)
            node._last_velocity_receipt = now - Duration(seconds=2.0)

            node._state["stamp"] = now - Duration(seconds=1.0)
            node._advance_state(now)
            odom = node._state_to_odom(now)

            self.assertAlmostEqual(node._state["x"], 4.0)
            self.assertAlmostEqual(odom.twist.twist.linear.x, 0.0)
            self.assertAlmostEqual(odom.twist.twist.angular.z, 0.0)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
