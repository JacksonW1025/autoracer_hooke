import math
import unittest

import rclpy
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from fixposition_driver_msgs.msg import FpaOdomstatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.time import Time

from autoracer_localization.fixposition_seed_filter import (
    FixpositionSeedFilter,
    _status_is_good,
    _xy_stddev,
)
from autoracer_localization.ndt_axis_seed_fuser import _fuse_axis_specific_pose
from autoracer_localization.ndt_axis_seed_fuser import _temporal_filter_axis_pose
from autoracer_localization.ndt_initial_pose_predictor import (
    NdtInitialPosePredictor,
    STATE_LOST_RECOVERY,
    STATE_STARTUP,
    STATE_TRACKING,
    _propagate,
    _yaw_from_quaternion,
    _yaw_to_quaternion,
)


def make_pose(stamp, *, x=0.0, y=0.0, yaw=0.0, xy_variance=1.0, yaw_variance=0.01):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = _yaw_to_quaternion(yaw)
    msg.pose.covariance[0] = xy_variance
    msg.pose.covariance[7] = xy_variance
    msg.pose.covariance[35] = yaw_variance
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


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class LocalizationHelperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_fixposition_status_gate_allows_initialized_spp_by_default(self):
        self.assertTrue(_status_is_good(make_status(init=True, rtk=True)))
        self.assertTrue(_status_is_good(make_status(init=True, rtk=False)))
        self.assertFalse(_status_is_good(make_status(init=False, rtk=True)))

    def test_fixposition_status_gate_rejects_spp_when_rtk_required(self):
        self.assertFalse(_status_is_good(make_status(init=True, rtk=False), require_rtk=True))
        self.assertTrue(_status_is_good(make_status(init=True, rtk=True), require_rtk=True))

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

    def test_fixposition_status_covariance_and_jump_gate(self):
        node = FixpositionSeedFilter()
        try:
            now = node.get_clock().now()

            node._last_status = make_status(init=True, rtk=False)
            node._last_status_receipt = now
            ok, reason = node._validate_pose(make_pose(now))
            self.assertTrue(ok)

            ok, reason = node._validate_pose(make_pose(now, xy_variance=16.0))
            self.assertFalse(ok)
            self.assertIn("xy covariance", reason)

            node._last_status = make_status(init=False, rtk=False)
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

    def test_fixposition_require_rtk_parameter_rejects_spp_status(self):
        node = FixpositionSeedFilter()
        try:
            node._require_rtk = True
            now = node.get_clock().now()
            node._last_status = make_status(init=True, rtk=False)
            node._last_status_receipt = now

            ok, reason = node._validate_pose(make_pose(now))

            self.assertFalse(ok)
            self.assertIn("RTK", reason)
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

    def test_predictor_advances_full_elapsed_time_in_bounded_steps(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()
            node._max_prediction_step = 0.2
            node._state = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "covariance": [0.0] * 36,
                "stamp": now - Duration(seconds=1.0),
            }
            velocity = VelocityReport()
            velocity.longitudinal_velocity = 2.0
            velocity.heading_rate = 0.0
            node._last_velocity = velocity
            node._last_velocity_receipt = now

            node._advance_state(now)

            self.assertAlmostEqual(node._state["x"], 2.0)
            self.assertAlmostEqual(node._state["y"], 0.0)
            self.assertEqual(node._state["stamp"], now)
        finally:
            node.destroy_node()

    def test_predictor_integrates_motion_piecewise_when_velocity_updates(self):
        node = NdtInitialPosePredictor()
        try:
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            update = Time(seconds=1.0, clock_type=clock_type)
            end = Time(seconds=2.0, clock_type=clock_type)
            node._vehicle_status_timeout = 2.0
            node._state = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "covariance": [0.0] * 36,
                "stamp": start,
            }

            initial_velocity = VelocityReport()
            initial_velocity.header.stamp = start.to_msg()
            initial_velocity.longitudinal_velocity = 1.0
            initial_velocity.heading_rate = 0.0
            node._last_velocity = initial_velocity
            node._last_velocity_receipt = start

            updated_velocity = VelocityReport()
            updated_velocity.header.stamp = update.to_msg()
            updated_velocity.longitudinal_velocity = 1.0
            updated_velocity.heading_rate = 1.0
            node._on_velocity(updated_velocity)
            node._advance_state(end)

            self.assertAlmostEqual(node._state["x"], 1.0 + math.sin(1.0))
            self.assertAlmostEqual(node._state["y"], 1.0 - math.cos(1.0))
            self.assertAlmostEqual(node._state["yaw"], 1.0)
        finally:
            node.destroy_node()

    def test_predictor_ndt_correction_and_fixposition_reset(self):
        node = NdtInitialPosePredictor()
        try:
            node._ndt_seed_deviation_guard = 0.0
            node._enable_seed_bias_correction = False
            now = node.get_clock().now()

            node._on_seed_pose(make_pose(now, x=1.0))
            self.assertAlmostEqual(node._state["x"], 1.0)
            self.assertEqual(node._predictor_state, STATE_STARTUP)

            node._on_seed_pose(make_pose(now, x=2.0))
            self.assertAlmostEqual(node._state["x"], 1.0)
            self.assertEqual(node._startup_seed_cooldown_ignored_count, 1)

            node._on_ndt_pose(make_pose(now, x=10.0, yaw=0.5))
            self.assertAlmostEqual(node._state["x"], 10.0)
            self.assertAlmostEqual(node._state["yaw"], 0.5)
            self.assertEqual(node._predictor_state, STATE_TRACKING)

            node._on_seed_pose(make_pose(now, x=20.0))
            self.assertAlmostEqual(node._state["x"], 10.0)
            self.assertEqual(node._tracking_seed_ignored_count, 1)

            node._last_ndt_receipt = node.get_clock().now() - Duration(seconds=2.0)
            node._on_seed_pose(make_pose(now, x=30.0))
            self.assertAlmostEqual(node._state["x"], 30.0)
            self.assertEqual(node._predictor_state, STATE_LOST_RECOVERY)
            self.assertEqual(node._lost_recovery_seed_reset_count, 1)

            node._on_seed_pose(make_pose(now, x=31.0))
            self.assertAlmostEqual(node._state["x"], 30.0)
            self.assertEqual(node._lost_recovery_seed_ignored_count, 1)

            node._on_ndt_pose(make_pose(now, x=35.0))
            self.assertEqual(node._predictor_state, STATE_TRACKING)
            node._last_ndt_receipt = node.get_clock().now() - Duration(seconds=2.0)
            node._on_seed_pose(make_pose(now, x=40.0))
            self.assertAlmostEqual(node._state["x"], 40.0)
            self.assertEqual(node._lost_recovery_seed_reset_count, 2)

            msg = node._state_to_msg(now)
            self.assertAlmostEqual(_yaw_from_quaternion(msg.pose.pose.orientation), 0.0)
        finally:
            node.destroy_node()

    def test_predictor_refreshes_startup_seed_before_first_ndt_lock_on_cooldown(self):
        node = NdtInitialPosePredictor()
        try:
            node._seed_reset_cooldown = 1.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(start, x=0.0))
            self.assertAlmostEqual(node._state["x"], 0.0)

            early_seed = Time(seconds=0.5, clock_type=clock_type)
            node._on_seed_pose(make_pose(early_seed, x=10.0))
            self.assertAlmostEqual(node._state["x"], 0.0)

            cooldown_seed = Time(seconds=1.1, clock_type=clock_type)
            node._on_seed_pose(make_pose(cooldown_seed, x=20.0))
            self.assertAlmostEqual(node._state["x"], 20.0)
            self.assertEqual(node._startup_seed_refresh_count, 1)

            node._on_ndt_pose(make_pose(Time(seconds=1.2, clock_type=clock_type), x=30.0))
            fresh_seed_after_lock = Time(seconds=1.5, clock_type=clock_type)
            node._on_seed_pose(make_pose(fresh_seed_after_lock, x=40.0))
            self.assertAlmostEqual(node._state["x"], 30.0)
            self.assertEqual(node._tracking_seed_ignored_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_tracking_seed_fusion_corrects_cross_track_only(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_tracking_seed_fusion = True
            node._max_tracking_seed_stddev = 0.5
            node._max_tracking_seed_age = 0.5
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=1.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(stamp, x=0.0, y=0.0, xy_variance=0.01))
            node._on_ndt_pose(make_pose(stamp, x=0.0, y=2.0, yaw=0.0, xy_variance=0.09))

            node._on_seed_pose(make_pose(stamp, x=5.0, y=0.0, xy_variance=0.01))

            expected_gain = 0.09 / (0.09 + 0.01)
            self.assertAlmostEqual(node._state["x"], 0.0)
            self.assertAlmostEqual(node._state["y"], 2.0 - 2.0 * expected_gain)
            self.assertEqual(node._tracking_seed_fusion_count, 1)
            self.assertEqual(node._tracking_seed_ignored_count, 0)
        finally:
            node.destroy_node()

    def test_predictor_tracking_seed_fusion_corrects_yaw_without_along_shift(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_tracking_seed_fusion = True
            node._max_tracking_seed_stddev = 0.5
            node._max_tracking_seed_age = 0.5
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=1.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(stamp, x=0.0, y=0.0, yaw=0.0))
            node._on_ndt_pose(
                make_pose(
                    stamp,
                    x=10.0,
                    y=2.0,
                    yaw=0.4,
                    xy_variance=0.09,
                    yaw_variance=0.01,
                )
            )

            node._on_seed_pose(
                make_pose(
                    stamp,
                    x=10.0,
                    y=2.0,
                    yaw=0.0,
                    xy_variance=0.01,
                    yaw_variance=0.0001,
                )
            )

            expected_gain = 0.01 / (0.01 + 0.0001)
            self.assertAlmostEqual(node._state["x"], 10.0)
            self.assertAlmostEqual(node._state["y"], 2.0)
            self.assertAlmostEqual(node._state["yaw"], 0.4 - 0.4 * expected_gain)
            self.assertEqual(node._tracking_seed_fusion_count, 1)
            self.assertEqual(node._tracking_seed_ignored_count, 0)
        finally:
            node.destroy_node()

    def test_predictor_tracking_seed_fusion_ignores_yaw_inside_seed_noise_floor(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_tracking_seed_fusion = True
            node._max_tracking_seed_stddev = 0.5
            node._max_tracking_seed_age = 0.5
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=1.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(stamp, x=0.0, y=0.0, yaw=0.0))
            node._on_ndt_pose(
                make_pose(
                    stamp,
                    x=10.0,
                    y=2.0,
                    yaw=0.02,
                    xy_variance=0.09,
                    yaw_variance=0.01,
                )
            )

            node._on_seed_pose(
                make_pose(
                    stamp,
                    x=10.0,
                    y=2.0,
                    yaw=0.0,
                    xy_variance=0.01,
                    yaw_variance=0.0001,
                )
            )

            self.assertAlmostEqual(node._state["x"], 10.0)
            self.assertAlmostEqual(node._state["y"], 2.0)
            self.assertAlmostEqual(node._state["yaw"], 0.02)
            self.assertEqual(node._tracking_seed_fusion_count, 0)
            self.assertEqual(node._tracking_seed_ignored_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_tracking_seed_fusion_rejects_poor_covariance(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_tracking_seed_fusion = True
            node._max_tracking_seed_stddev = 0.5
            node._max_tracking_seed_age = 0.5
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=1.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(stamp, x=0.0, y=0.0, xy_variance=0.01))
            node._on_ndt_pose(make_pose(stamp, x=0.0, y=2.0, yaw=0.0, xy_variance=0.09))
            node._on_seed_pose(make_pose(stamp, x=0.0, y=0.0, xy_variance=4.0))

            self.assertAlmostEqual(node._state["x"], 0.0)
            self.assertAlmostEqual(node._state["y"], 2.0)
            self.assertEqual(node._tracking_seed_fusion_count, 0)
            self.assertEqual(node._tracking_seed_ignored_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_allows_only_one_seed_recovery_until_ndt_relocks(self):
        node = NdtInitialPosePredictor()
        try:
            node._ndt_seed_deviation_guard = 0.0
            node._enable_seed_bias_correction = False
            node._ndt_lost_timeout = 1.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(start, x=0.0))
            node._on_ndt_pose(make_pose(start, x=10.0))

            first_reset = Time(seconds=2.0, clock_type=clock_type)
            node._on_seed_pose(make_pose(first_reset, x=20.0))
            self.assertAlmostEqual(node._state["x"], 20.0)
            self.assertEqual(node._lost_recovery_seed_reset_count, 1)
            self.assertEqual(node._predictor_state, STATE_LOST_RECOVERY)

            repeated_seed = Time(seconds=3.1, clock_type=clock_type)
            node._on_seed_pose(make_pose(repeated_seed, x=30.0))
            self.assertAlmostEqual(node._state["x"], 20.0)
            self.assertEqual(node._lost_recovery_seed_ignored_count, 1)

            node._on_ndt_pose(make_pose(Time(seconds=3.2, clock_type=clock_type), x=40.0))
            self.assertAlmostEqual(node._state["x"], 40.0)
            self.assertEqual(node._predictor_state, STATE_TRACKING)

            second_lost = Time(seconds=5.0, clock_type=clock_type)
            node._on_seed_pose(make_pose(second_lost, x=50.0))
            self.assertAlmostEqual(node._state["x"], 50.0)
            self.assertEqual(node._lost_recovery_seed_reset_count, 2)
        finally:
            node.destroy_node()

    def test_predictor_accepts_ndt_correction_far_from_noisy_fresh_seed_by_default(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()

            node._on_seed_pose(make_pose(now, x=0.0))
            node._on_ndt_pose(make_pose(now, x=0.1))
            self.assertAlmostEqual(node._state["x"], 0.1)

            fresh_seed_stamp = now + Duration(seconds=0.1)
            node._on_seed_pose(make_pose(fresh_seed_stamp, x=0.2))
            node._on_ndt_pose(make_pose(now + Duration(seconds=0.2), x=3.0))
            self.assertAlmostEqual(node._state["x"], 3.0)

            node._on_ndt_pose(make_pose(now + Duration(seconds=0.3), x=0.3))
            self.assertAlmostEqual(node._state["x"], 0.3)
        finally:
            node.destroy_node()

    def test_predictor_publishes_regularization_seed_only_for_startup_or_lost_recovery(self):
        node = NdtInitialPosePredictor()
        try:
            publisher = RecordingPublisher()
            node._regularization_seed_publisher = publisher
            node._ndt_lost_timeout = 1.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(start, x=1.0))
            self.assertEqual(len(publisher.messages), 1)

            node._on_ndt_pose(make_pose(Time(seconds=0.1, clock_type=clock_type), x=1.1))
            node._on_seed_pose(make_pose(Time(seconds=0.2, clock_type=clock_type), x=2.0))
            self.assertEqual(len(publisher.messages), 1)

            node._on_seed_pose(make_pose(Time(seconds=2.0, clock_type=clock_type), x=3.0))
            self.assertEqual(len(publisher.messages), 2)
            self.assertEqual(node._predictor_state, STATE_LOST_RECOVERY)
        finally:
            node.destroy_node()

    def test_predictor_learns_seed_bias_correction_for_lost_recovery(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_seed_bias_correction = True
            node._seed_bias_correction_alpha = 1.0
            node._ndt_lost_timeout = 1.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)

            startup_seed = make_pose(start, x=1.0, y=0.0, yaw=0.1)
            startup_seed.pose.pose.position.z = 2.0
            node._on_seed_pose(startup_seed)
            ndt = make_pose(Time(seconds=0.1, clock_type=clock_type), x=0.0, y=0.0, yaw=0.0)
            ndt.pose.pose.position.z = 0.0
            node._on_ndt_pose(ndt)

            lost_seed = make_pose(Time(seconds=2.0, clock_type=clock_type), x=11.0, y=0.0, yaw=0.1)
            lost_seed.pose.pose.position.z = 2.0
            node._on_seed_pose(lost_seed)

            self.assertEqual(node._predictor_state, STATE_LOST_RECOVERY)
            self.assertAlmostEqual(node._state["x"], 10.0)
            self.assertAlmostEqual(node._state["z"], 0.0)
            self.assertAlmostEqual(node._state["yaw"], 0.0)
        finally:
            node.destroy_node()

    def test_predictor_publishes_corrected_seed_regularization_pose(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_seed_bias_correction = True
            node._seed_bias_correction_alpha = 1.0
            publisher = RecordingPublisher()
            node._corrected_seed_publisher = publisher
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)

            first_seed = make_pose(start, x=1.0, y=0.0, yaw=0.1)
            first_seed.pose.pose.position.z = 2.0
            node._on_seed_pose(first_seed)
            ndt = make_pose(Time(seconds=0.1, clock_type=clock_type), x=0.0, y=0.0, yaw=0.0)
            ndt.pose.pose.position.z = 0.0
            node._on_ndt_pose(ndt)

            tracking_seed = make_pose(Time(seconds=0.2, clock_type=clock_type), x=2.0, y=0.0, yaw=0.1)
            tracking_seed.pose.pose.position.z = 2.0
            node._on_seed_pose(tracking_seed)

            corrected = publisher.messages[-1]
            self.assertAlmostEqual(corrected.pose.pose.position.x, 1.0)
            self.assertAlmostEqual(corrected.pose.pose.position.z, 0.0)
            self.assertAlmostEqual(_yaw_from_quaternion(corrected.pose.pose.orientation), 0.0)
        finally:
            node.destroy_node()

    def test_predictor_uses_message_stamps_for_pose_and_motion_inputs(self):
        node = NdtInitialPosePredictor()
        try:
            node._ndt_seed_deviation_guard = 0.0
            seed_stamp = Time(seconds=12.3)
            node._on_seed_pose(make_pose(seed_stamp, x=1.0))
            self.assertEqual(node._state["stamp"].nanoseconds, seed_stamp.nanoseconds)

            velocity_stamp = Time(seconds=12.4)
            velocity = VelocityReport()
            velocity.header.stamp = velocity_stamp.to_msg()
            velocity.longitudinal_velocity = 2.0
            velocity.heading_rate = 0.0
            node._on_velocity(velocity)
            self.assertEqual(node._last_velocity_receipt.nanoseconds, velocity_stamp.nanoseconds)

            steering_stamp = Time(seconds=12.5)
            steering = SteeringReport()
            steering.stamp = steering_stamp.to_msg()
            steering.steering_tire_angle = 0.1
            node._on_steering(steering)
            self.assertEqual(node._last_steering_receipt.nanoseconds, steering_stamp.nanoseconds)

            ndt_stamp = Time(seconds=12.6)
            node._on_ndt_pose(make_pose(ndt_stamp, x=10.0))
            self.assertEqual(node._last_ndt_receipt.nanoseconds, ndt_stamp.nanoseconds)
            self.assertEqual(node._state["stamp"].nanoseconds, ndt_stamp.nanoseconds)
        finally:
            node.destroy_node()

    def test_axis_seed_fuser_corrects_cross_track_without_along_track_shift(self):
        stamp = Time(seconds=1.0)
        ndt = make_pose(stamp, x=5.0, y=2.0, yaw=0.0, xy_variance=0.09)
        seed = make_pose(stamp, x=10.0, y=0.0, yaw=0.0, xy_variance=0.01)

        fused, applied = _fuse_axis_specific_pose(ndt, seed)

        self.assertTrue(applied)
        self.assertAlmostEqual(fused.pose.pose.position.x, 5.0)
        self.assertAlmostEqual(fused.pose.pose.position.y, 0.0)

    def test_axis_seed_fuser_ignores_yaw_inside_seed_noise_floor(self):
        stamp = Time(seconds=1.0)
        ndt = make_pose(stamp, x=0.0, y=0.0, yaw=0.02, yaw_variance=0.01)
        seed = make_pose(stamp, x=0.0, y=0.0, yaw=0.0, yaw_variance=0.0001)

        fused, applied = _fuse_axis_specific_pose(ndt, seed)

        self.assertFalse(applied)
        self.assertAlmostEqual(_yaw_from_quaternion(fused.pose.pose.orientation), 0.02)

    def test_axis_seed_fuser_corrects_large_yaw_residual(self):
        stamp = Time(seconds=1.0)
        ndt = make_pose(stamp, x=0.0, y=0.0, yaw=0.4, yaw_variance=0.01)
        seed = make_pose(stamp, x=0.0, y=0.0, yaw=0.0, yaw_variance=0.0001)

        fused, applied = _fuse_axis_specific_pose(ndt, seed)

        expected_gain = 0.01 / (0.01 + 0.0001)
        self.assertTrue(applied)
        self.assertAlmostEqual(
            _yaw_from_quaternion(fused.pose.pose.orientation),
            0.4 - 0.4 * expected_gain,
        )

    def test_axis_temporal_filter_smooths_cross_and_yaw_without_along_shift(self):
        stamp = Time(seconds=1.0)
        seed = make_pose(stamp, x=0.0, y=0.0, yaw=0.0)
        previous = make_pose(stamp, x=8.0, y=0.0, yaw=0.0)
        current = make_pose(stamp, x=10.0, y=2.0, yaw=0.2)

        filtered, details = _temporal_filter_axis_pose(
            current,
            seed,
            previous,
            lateral_alpha=0.25,
            yaw_alpha=0.5,
            mahalanobis_gate=0.0,
        )

        self.assertFalse(details["rejected"])
        self.assertAlmostEqual(filtered.pose.pose.position.x, 10.0)
        self.assertAlmostEqual(filtered.pose.pose.position.y, 0.5)
        self.assertAlmostEqual(_yaw_from_quaternion(filtered.pose.pose.orientation), 0.1)

    def test_axis_temporal_filter_rejects_mahalanobis_outlier_without_along_shift(self):
        stamp = Time(seconds=1.0)
        seed = make_pose(stamp, x=0.0, y=0.0, yaw=0.0)
        previous = make_pose(stamp, x=8.0, y=0.0, yaw=0.0)
        current = make_pose(stamp, x=10.0, y=5.0, yaw=1.0)

        filtered, details = _temporal_filter_axis_pose(
            current,
            seed,
            previous,
            lateral_alpha=0.25,
            yaw_alpha=0.5,
            mahalanobis_gate=3.0,
            lateral_innovation_stddev_m=0.5,
            yaw_innovation_stddev_rad=0.1,
        )

        self.assertTrue(details["rejected"])
        self.assertAlmostEqual(filtered.pose.pose.position.x, 10.0)
        self.assertAlmostEqual(filtered.pose.pose.position.y, 0.0)
        self.assertAlmostEqual(_yaw_from_quaternion(filtered.pose.pose.orientation), 0.0)


if __name__ == "__main__":
    unittest.main()
