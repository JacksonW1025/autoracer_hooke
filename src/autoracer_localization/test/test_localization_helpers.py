import math
import unittest

import rclpy
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from fixposition_driver_msgs.msg import FpaOdomstatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration

from autoracer_localization.fixposition_seed_filter import (
    FixpositionSeedFilter,
    _status_is_good,
    _xy_stddev,
)
from autoracer_localization.ndt_initial_pose_predictor import (
    NdtInitialPosePredictor,
    _propagate,
    _yaw_from_quaternion,
    _yaw_to_quaternion,
)


def make_pose(stamp, *, x=0.0, y=0.0, yaw=0.0, xy_variance=1.0):
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


def make_status(*, init=True, rtk=True):
    msg = FpaOdomstatus()
    consts = msg.consts
    msg.init_status = (
        consts.INIT_STATUS_GLOBAL_INIT if init else consts.INIT_STATUS_LOCAL_INIT
    )
    msg.gnss1_status = consts.GNSS_STATUS_RTK_FIXED if rtk else consts.GNSS_STATUS_SPP
    msg.gnss2_status = consts.GNSS_STATUS_NO_FIX
    return msg


class LocalizationHelperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_fixposition_status_gate(self):
        self.assertTrue(_status_is_good(make_status(init=True, rtk=True)))
        self.assertFalse(_status_is_good(make_status(init=False, rtk=True)))
        self.assertFalse(_status_is_good(make_status(init=True, rtk=False)))

    def test_fixposition_covariance_gate(self):
        node = FixpositionSeedFilter()
        try:
            now = node.get_clock().now()
            self.assertEqual(_xy_stddev(make_pose(now, xy_variance=9.0).pose.covariance), 3.0)

            ok, _ = node._validate_pose(make_pose(now, xy_variance=9.0))
            self.assertTrue(ok)

            ok, reason = node._validate_pose(make_pose(now, xy_variance=16.0))
            self.assertFalse(ok)
            self.assertIn("xy covariance", reason)
        finally:
            node.destroy_node()

    def test_fixposition_status_and_jump_gate(self):
        node = FixpositionSeedFilter()
        try:
            now = node.get_clock().now()

            node._last_status = make_status(init=True, rtk=False)
            node._last_status_receipt = now
            ok, reason = node._validate_pose(make_pose(now))
            self.assertFalse(ok)
            self.assertIn("odomstatus", reason)

            node._last_status = make_status(init=True, rtk=True)
            node._last_published_pose = make_pose(now, x=0.0).pose.pose
            ok, reason = node._validate_pose(make_pose(now, x=6.0))
            self.assertFalse(ok)
            self.assertIn("xy jump", reason)
        finally:
            node.destroy_node()

    def test_predictor_straight_and_turn_propagation(self):
        x, y, yaw = _propagate(0.0, 0.0, 0.0, 2.0, 0.0, 1.0)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

        x, y, yaw = _propagate(0.0, 0.0, 0.0, 2.0, 1.0, 1.0)
        self.assertAlmostEqual(x, 2.0 * math.sin(1.0))
        self.assertAlmostEqual(y, 2.0 * (1.0 - math.cos(1.0)))
        self.assertAlmostEqual(yaw, 1.0)

    def test_predictor_uses_heading_rate_then_steering_fallback(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()

            velocity = VelocityReport()
            velocity.longitudinal_velocity = 2.0
            velocity.heading_rate = 0.3
            node._last_velocity = velocity
            node._last_velocity_receipt = now
            self.assertEqual(node._motion(now), (2.0, 0.3))

            velocity.heading_rate = math.nan
            steering = SteeringReport()
            steering.steering_tire_angle = 0.1
            node._wheel_base = 2.0
            node._last_steering = steering
            node._last_steering_receipt = now
            motion = node._motion(now)
            self.assertAlmostEqual(motion[0], 2.0)
            self.assertAlmostEqual(motion[1], math.tan(0.1))
        finally:
            node.destroy_node()

    def test_predictor_ndt_correction_and_fixposition_reset(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()

            node._on_seed_pose(make_pose(now, x=1.0))
            self.assertAlmostEqual(node._state["x"], 1.0)

            node._on_ndt_pose(make_pose(now, x=10.0, yaw=0.5))
            self.assertAlmostEqual(node._state["x"], 10.0)
            self.assertAlmostEqual(node._state["yaw"], 0.5)

            node._on_seed_pose(make_pose(now, x=20.0))
            self.assertAlmostEqual(node._state["x"], 10.0)

            node._last_ndt_receipt = node.get_clock().now() - Duration(seconds=2.0)
            node._on_seed_pose(make_pose(now, x=30.0))
            self.assertAlmostEqual(node._state["x"], 30.0)

            msg = node._state_to_msg(now)
            self.assertAlmostEqual(_yaw_from_quaternion(msg.pose.pose.orientation), 0.0)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
