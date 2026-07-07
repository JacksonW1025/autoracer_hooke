import json
import math
import unittest

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from fixposition_driver_msgs.msg import FpaOdomstatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from rclpy.duration import Duration
from rclpy.time import Time

from autoracer_localization.correlated_fixposition_noise import (
    CorrelatedFixpositionNoise,
    CorrelatedNoiseModel,
    apply_planar_noise_to_odometry,
)
from autoracer_localization.fixposition_seed_filter import (
    FixpositionSeedFilter,
    _status_is_good,
    _xy_stddev,
)
from autoracer_localization.ground_truth_initialpose_once import (
    GroundTruthInitialposeOnce,
    clone_initialpose,
)
from autoracer_localization.diagnostic_pose_reinitializer import (
    DiagnosticPoseReinitializer,
    InitializeLocalization as DiagnosticInitializeLocalization,
    diagnostic_ekf_status_has_pose_no_update_error,
    diagnostic_initialization_state_allows_reinitialization,
    diagnostic_measurement_after_initialization_allows_lost_reinitialization,
    diagnostic_pose_stamp_is_fresh,
    diagnostic_pose_instability_status_has_planar_error,
    diagnostic_post_initialization_grace_allows_reinitialization,
    diagnostic_reinitializer_has_required_seed,
    diagnostic_stamp_allows_reinitialization,
    diagnostic_status_should_trigger_reinitialization,
    diagnostic_sustained_trigger_allows_reinitialization,
    diagnostic_update_sustained_trigger_start,
)
from autoracer_localization.startup_pose_initializer_once import (
    InitializeLocalization as StartupInitializeLocalization,
    replace_startup_pose_yaw_from_route,
    route_heading_for_xy,
    startup_initialize_method_to_request,
    startup_initialize_should_attempt,
)
from autoracer_localization.pointcloud_clock_publisher import (
    PointcloudClockPublisher,
    clock_from_pointcloud,
)
from autoracer_localization.vehicle_status_clock_publisher import clock_from_velocity_status
from autoracer_localization.vehicle_status_to_twist_covariance import (
    VehicleStatusToTwistCovariance,
    twist_covariance_from_status,
)
from autoracer_localization.ekf_feedback_gate import (
    EkfFeedbackGate,
    feedback_pose_is_measurement_backed,
)
from autoracer_localization.fixposition_startup_seed_gate import FixpositionStartupSeedGate
from autoracer_localization.fixposition_odom_to_seed_pose import (
    BASE_TO_GNSS_TRANSLATION,
    BASE_TO_GNSS_YAW,
    odometry_to_seed_pose,
)
from autoracer_localization.ndt_axis_seed_fuser import _fuse_axis_specific_pose
from autoracer_localization.ndt_axis_seed_fuser import _fuse_ndt_cross_yaw_seed_along_pose
from autoracer_localization.ndt_axis_seed_fuser import _apply_initial_pose_correction_gain
from autoracer_localization.ndt_axis_seed_fuser import _apply_initial_pose_axis_correction_gain
from autoracer_localization.ndt_axis_seed_fuser import _make_prediction_fallback_msg
from autoracer_localization.ndt_axis_seed_fuser import _ndt_is_consistent_with_initial_pose
from autoracer_localization.ndt_axis_seed_fuser import _prediction_fallback_due
from autoracer_localization.ndt_axis_seed_fuser import _ekf_initial_pose_update
from autoracer_localization.ndt_axis_seed_fuser import _robust_initial_pose_update
from autoracer_localization.ndt_axis_seed_fuser import _runtime_candidate_spread_variance_inflation
from autoracer_localization.ndt_axis_seed_fuser import _apply_body_frame_position_bias
from autoracer_localization.ndt_axis_seed_fuser import _temporal_filter_axis_pose
from autoracer_localization.ndt_axis_seed_fuser import NdtAxisSeedFuser
from autoracer_localization.ndt_initial_pose_predictor import (
    NdtInitialPosePredictor,
    STATE_LOST_RECOVERY,
    STATE_STARTUP,
    STATE_TRACKING,
    _propagate,
    _rpy_from_quaternion,
    _rpy_to_quaternion,
    _yaw_from_quaternion,
    _yaw_to_quaternion,
)


def make_pose(
    stamp,
    *,
    x=0.0,
    y=0.0,
    z=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
    xy_variance=1.0,
    yaw_variance=0.01,
):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation = _rpy_to_quaternion(roll, pitch, yaw)
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


class RecordingSetBoolClient:
    def __init__(self, *, ready=True):
        self.ready = ready
        self.wait_timeouts = []
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        self.wait_timeouts.append(timeout_sec)
        return self.ready

    def call_async(self, request):
        self.requests.append(request)


class LocalizationHelperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_fixposition_status_gate_allows_initialized_spp_by_default(self):
        self.assertTrue(_status_is_good(make_status(init=True, rtk=True)))

    def test_vehicle_status_to_twist_uses_steering_when_heading_rate_missing(self):
        velocity = VelocityReport()
        velocity.header.stamp = Time(seconds=12.3).to_msg()
        velocity.header.frame_id = "base_link"
        velocity.longitudinal_velocity = 10.0
        velocity.lateral_velocity = 0.2
        velocity.heading_rate = 0.0

        steering = SteeringReport()
        steering.stamp = Time(seconds=12.28).to_msg()
        steering.steering_tire_angle = 0.1

        twist = twist_covariance_from_status(
            velocity,
            steering,
            wheel_base_m=2.0,
            longitudinal_variance_m2ps2=0.04,
            lateral_variance_m2ps2=0.09,
            yaw_rate_variance_rad2ps2=0.16,
            steering_timeout_sec=0.05,
        )

        self.assertEqual(twist.header.frame_id, "base_link")
        self.assertAlmostEqual(twist.twist.twist.linear.x, 10.0)
        self.assertAlmostEqual(twist.twist.twist.linear.y, 0.2)
        self.assertAlmostEqual(twist.twist.twist.angular.z, 10.0 * math.tan(0.1) / 2.0)
        self.assertAlmostEqual(twist.twist.covariance[0], 0.04)
        self.assertAlmostEqual(twist.twist.covariance[7], 0.09)
        self.assertAlmostEqual(twist.twist.covariance[35], 0.16)

    def test_vehicle_status_to_twist_keeps_nonzero_reported_heading_rate(self):
        velocity = VelocityReport()
        velocity.header.stamp = Time(seconds=20.0).to_msg()
        velocity.header.frame_id = ""
        velocity.longitudinal_velocity = 4.0
        velocity.heading_rate = 0.3

        steering = SteeringReport()
        steering.stamp = Time(seconds=19.99).to_msg()
        steering.steering_tire_angle = 0.5

        twist = twist_covariance_from_status(
            velocity,
            steering,
            wheel_base_m=2.0,
            longitudinal_variance_m2ps2=0.04,
            lateral_variance_m2ps2=0.09,
            yaw_rate_variance_rad2ps2=0.16,
            steering_timeout_sec=0.05,
        )

        self.assertEqual(twist.header.frame_id, "base_link")
        self.assertAlmostEqual(twist.twist.twist.angular.z, 0.3)

    def test_vehicle_status_to_twist_node_publishes_when_velocity_arrives(self):
        node = VehicleStatusToTwistCovariance()
        try:
            publisher = RecordingPublisher()
            node._publisher = publisher

            steering = SteeringReport()
            steering.stamp = Time(seconds=30.0).to_msg()
            steering.steering_tire_angle = 0.05
            node._on_steering(steering)

            velocity = VelocityReport()
            velocity.header.stamp = Time(seconds=30.01).to_msg()
            velocity.longitudinal_velocity = 6.0
            velocity.heading_rate = 0.0
            node._on_velocity(velocity)

            self.assertEqual(len(publisher.messages), 1)
            self.assertGreater(publisher.messages[0].twist.twist.angular.z, 0.0)
        finally:
            node.destroy_node()
        self.assertTrue(_status_is_good(make_status(init=True, rtk=False)))
        self.assertFalse(_status_is_good(make_status(init=False, rtk=True)))

    def test_diagnostic_reinitializer_triggers_only_on_target_error_statuses(self):
        targets = ["localization: pose_instability_detector"]

        status = DiagnosticStatus()
        status.name = "localization: pose_instability_detector"
        status.level = DiagnosticStatus.ERROR
        status.values = [KeyValue(key="diff_position_y:status", value="ERROR")]
        self.assertTrue(diagnostic_status_should_trigger_reinitialization(status, targets))

        status.name = "ellipse_error_status"
        status.level = DiagnosticStatus.ERROR
        self.assertFalse(diagnostic_status_should_trigger_reinitialization(status, targets))

        status.name = "localization_error_monitor: ellipse_error_status"
        status.level = DiagnosticStatus.ERROR
        self.assertFalse(diagnostic_status_should_trigger_reinitialization(status, targets))

        status.level = DiagnosticStatus.WARN
        self.assertFalse(diagnostic_status_should_trigger_reinitialization(status, targets))

        status.name = "unrelated"
        status.level = DiagnosticStatus.ERROR
        self.assertFalse(diagnostic_status_should_trigger_reinitialization(status, targets))

    def test_diagnostic_reinitializer_ignores_pose_instability_z_only_error(self):
        status = DiagnosticStatus()
        status.name = "localization: pose_instability_detector"
        status.level = DiagnosticStatus.ERROR
        status.values = [
            KeyValue(key="diff_position_x:status", value="OK"),
            KeyValue(key="diff_position_y:status", value="OK"),
            KeyValue(key="diff_position_z:status", value="ERROR"),
            KeyValue(key="diff_angle_z:status", value="OK"),
        ]

        self.assertFalse(diagnostic_pose_instability_status_has_planar_error(status))
        self.assertFalse(
            diagnostic_status_should_trigger_reinitialization(
                status, ["localization: pose_instability_detector"]
            )
        )

        status.values = [
            KeyValue(key="diff_position_x:status", value="OK"),
            KeyValue(key="diff_position_y:status", value="ERROR"),
            KeyValue(key="diff_position_z:status", value="OK"),
            KeyValue(key="diff_angle_z:status", value="OK"),
        ]

        self.assertTrue(diagnostic_pose_instability_status_has_planar_error(status))
        self.assertTrue(
            diagnostic_status_should_trigger_reinitialization(
                status, ["localization: pose_instability_detector"]
            )
        )

    def test_diagnostic_reinitializer_uses_ekf_pose_no_update_not_covariance_error(self):
        status = DiagnosticStatus()
        status.name = "localization: ekf_localizer"
        status.level = DiagnosticStatus.ERROR
        status.message = "[ERROR]cov_ellipse_long_axis is large"
        status.values = [
            KeyValue(key="pose_no_update_count", value="99"),
            KeyValue(key="pose_no_update_count_threshold_error", value="100"),
        ]

        self.assertFalse(diagnostic_ekf_status_has_pose_no_update_error(status))
        self.assertFalse(
            diagnostic_status_should_trigger_reinitialization(
                status, ["localization: ekf_localizer"]
            )
        )

        status.message = "[ERROR]pose is not updated; [ERROR]cov_ellipse_long_axis is large"
        status.values = [
            KeyValue(key="pose_no_update_count", value="100"),
            KeyValue(key="pose_no_update_count_threshold_error", value="100"),
        ]

        self.assertTrue(diagnostic_ekf_status_has_pose_no_update_error(status))
        self.assertTrue(
            diagnostic_status_should_trigger_reinitialization(
                status, ["localization: ekf_localizer"]
            )
        )

    def test_diagnostic_reinitializer_requires_measurement_before_lost_reinitialization(self):
        self.assertFalse(
            diagnostic_measurement_after_initialization_allows_lost_reinitialization(
                last_measurement_stamp_sec=None,
                last_initialized_stamp_sec=12.0,
            )
        )
        self.assertFalse(
            diagnostic_measurement_after_initialization_allows_lost_reinitialization(
                last_measurement_stamp_sec=11.9,
                last_initialized_stamp_sec=12.0,
            )
        )
        self.assertTrue(
            diagnostic_measurement_after_initialization_allows_lost_reinitialization(
                last_measurement_stamp_sec=15.9,
                last_initialized_stamp_sec=12.0,
            )
        )

    def test_reinitializer_uses_pose_initializer_service_type_and_declares_numeric_level(self):
        self.assertTrue(
            DiagnosticInitializeLocalization.__module__.startswith("autoware_localization_msgs.")
        )
        self.assertTrue(
            StartupInitializeLocalization.__module__.startswith("autoware_localization_msgs.")
        )

        node = DiagnosticPoseReinitializer()
        try:
            self.assertEqual(node.get_parameter("min_level").value, 2)
            self.assertEqual(node.get_parameter("post_initialization_grace_sec").value, 5.0)
            self.assertEqual(node.get_parameter("min_trigger_duration_sec").value, 1.0)
        finally:
            node.destroy_node()

    def test_diagnostic_reinitializer_ignores_startup_transient_before_min_stamp(self):
        self.assertFalse(
            diagnostic_stamp_allows_reinitialization(
                diagnostic_stamp_sec=4.99, min_diagnostic_stamp_sec=5.0
            )
        )
        self.assertTrue(
            diagnostic_stamp_allows_reinitialization(
                diagnostic_stamp_sec=5.0, min_diagnostic_stamp_sec=5.0
            )
        )

    def test_diagnostic_reinitializer_direct_method_requires_gnss_seed(self):
        self.assertFalse(
            diagnostic_reinitializer_has_required_seed(
                initialize_method=DiagnosticInitializeLocalization.Request.DIRECT,
                has_latest_gnss_pose=False,
            )
        )
        self.assertTrue(
            diagnostic_reinitializer_has_required_seed(
                initialize_method=DiagnosticInitializeLocalization.Request.DIRECT,
                has_latest_gnss_pose=True,
            )
        )
        self.assertTrue(
            diagnostic_reinitializer_has_required_seed(
                initialize_method=DiagnosticInitializeLocalization.Request.AUTO,
                has_latest_gnss_pose=False,
            )
        )

    def test_diagnostic_reinitializer_direct_seed_must_be_fresh(self):
        self.assertTrue(
            diagnostic_pose_stamp_is_fresh(
                pose_stamp_sec=23.45,
                reference_stamp_sec=23.50,
                max_age_sec=0.5,
            )
        )
        self.assertFalse(
            diagnostic_pose_stamp_is_fresh(
                pose_stamp_sec=19.381,
                reference_stamp_sec=23.501,
                max_age_sec=0.5,
            )
        )

    def test_diagnostic_reinitializer_declares_gnss_freshness_guard(self):
        node = DiagnosticPoseReinitializer()
        try:
            self.assertEqual(node.get_parameter("max_gnss_pose_age_sec").value, 0.5)
            self.assertEqual(node.get_parameter("direct_pose_topic").value, "/initialpose3d")
        finally:
            node.destroy_node()

    def test_diagnostic_reinitializer_direct_reset_publishes_latest_gnss_without_service_delay(self):
        node = DiagnosticPoseReinitializer()
        try:
            publisher = RecordingPublisher()
            pose = make_pose(Time(seconds=25.7), x=-115.5, y=309.4)
            node._direct_pose_publisher = publisher
            node._latest_gnss_pose = pose

            self.assertTrue(node._publish_direct_initialpose())
            self.assertEqual(publisher.messages, [pose])
        finally:
            node.destroy_node()

    def test_diagnostic_reinitializer_only_arms_after_initialized_state(self):
        self.assertFalse(diagnostic_initialization_state_allows_reinitialization(None))
        self.assertFalse(
            diagnostic_initialization_state_allows_reinitialization(
                LocalizationInitializationState.UNINITIALIZED
            )
        )
        self.assertFalse(
            diagnostic_initialization_state_allows_reinitialization(
                LocalizationInitializationState.INITIALIZING
            )
        )
        self.assertTrue(
            diagnostic_initialization_state_allows_reinitialization(
                LocalizationInitializationState.INITIALIZED
            )
        )

    def test_diagnostic_reinitializer_respects_post_initialization_grace(self):
        self.assertFalse(
            diagnostic_post_initialization_grace_allows_reinitialization(
                diagnostic_stamp_sec=14.9,
                last_initialized_stamp_sec=12.0,
                post_initialization_grace_sec=3.0,
            )
        )
        self.assertTrue(
            diagnostic_post_initialization_grace_allows_reinitialization(
                diagnostic_stamp_sec=15.0,
                last_initialized_stamp_sec=12.0,
                post_initialization_grace_sec=3.0,
            )
        )
        self.assertTrue(
            diagnostic_post_initialization_grace_allows_reinitialization(
                diagnostic_stamp_sec=2.0,
                last_initialized_stamp_sec=None,
                post_initialization_grace_sec=3.0,
            )
        )

    def test_diagnostic_reinitializer_requires_sustained_trigger(self):
        self.assertFalse(
            diagnostic_sustained_trigger_allows_reinitialization(
                diagnostic_stamp_sec=21.9,
                first_trigger_stamp_sec=21.0,
                min_trigger_duration_sec=1.0,
            )
        )
        self.assertTrue(
            diagnostic_sustained_trigger_allows_reinitialization(
                diagnostic_stamp_sec=22.0,
                first_trigger_stamp_sec=21.0,
                min_trigger_duration_sec=1.0,
            )
        )
        self.assertTrue(
            diagnostic_sustained_trigger_allows_reinitialization(
                diagnostic_stamp_sec=21.0,
                first_trigger_stamp_sec=21.0,
                min_trigger_duration_sec=0.0,
            )
        )

    def test_diagnostic_reinitializer_tracks_sustained_triggers_per_status_name(self):
        trigger_starts = {"localization: ekf_localizer": 21.0}

        self.assertIsNone(
            diagnostic_update_sustained_trigger_start(
                trigger_starts,
                trigger_name="localization: pose_instability_detector",
                is_triggering=False,
                diagnostic_stamp_sec=21.5,
            )
        )
        self.assertEqual(trigger_starts["localization: ekf_localizer"], 21.0)

        self.assertEqual(
            diagnostic_update_sustained_trigger_start(
                trigger_starts,
                trigger_name="localization: ekf_localizer",
                is_triggering=True,
                diagnostic_stamp_sec=22.0,
            ),
            21.0,
        )

    def test_startup_initializer_attempts_once_after_gnss_stamp_threshold(self):
        self.assertFalse(
            startup_initialize_should_attempt(
                gnss_stamp_sec=1.9,
                min_gnss_stamp_sec=2.0,
                request_in_flight=False,
                initialized=False,
            )
        )
        self.assertFalse(
            startup_initialize_should_attempt(
                gnss_stamp_sec=2.1,
                min_gnss_stamp_sec=2.0,
                request_in_flight=True,
                initialized=False,
            )
        )
        self.assertFalse(
            startup_initialize_should_attempt(
                gnss_stamp_sec=2.1,
                min_gnss_stamp_sec=2.0,
                request_in_flight=False,
                initialized=True,
            )
        )
        self.assertTrue(
            startup_initialize_should_attempt(
                gnss_stamp_sec=2.1,
                min_gnss_stamp_sec=2.0,
                request_in_flight=False,
                initialized=False,
            )
        )

    def test_startup_initializer_supports_direct_and_auto_pose_initializer_methods(self):
        self.assertEqual(
            startup_initialize_method_to_request("auto"),
            StartupInitializeLocalization.Request.AUTO,
        )
        self.assertEqual(
            startup_initialize_method_to_request("DIRECT"),
            StartupInitializeLocalization.Request.DIRECT,
        )

    def test_route_heading_for_xy_uses_nearest_route_segment(self):
        route = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]

        result = route_heading_for_xy(
            route,
            x=1.2,
            y=0.3,
            max_distance_m=1.0,
            neighbor_stride=1,
        )

        self.assertIsNotNone(result)
        yaw, distance, index = result
        self.assertAlmostEqual(yaw, 0.0)
        self.assertAlmostEqual(distance, math.hypot(0.2, 0.3))
        self.assertEqual(index, 1)

    def test_route_heading_for_xy_rejects_far_gnss_seed(self):
        self.assertIsNone(
            route_heading_for_xy(
                [(0.0, 0.0), (1.0, 0.0)],
                x=20.0,
                y=0.0,
                max_distance_m=2.0,
                neighbor_stride=1,
            )
        )

    def test_route_heading_for_xy_can_prefer_route_start_near_startup_seed(self):
        result = route_heading_for_xy(
            [(0.0, 0.0), (1.0, 0.0), (100.0, 0.0)],
            x=1.5,
            y=0.0,
            max_distance_m=10.0,
            neighbor_stride=1,
            prefer_start_within_m=2.0,
        )

        self.assertIsNotNone(result)
        _, distance, index = result
        self.assertEqual(index, 0)
        self.assertAlmostEqual(distance, 1.5)

    def test_startup_route_heading_replaces_yaw_only_when_enabled_and_near_route(self):
        msg = make_pose(
            Time(seconds=2.0),
            x=0.2,
            y=1.0,
            yaw=1.5,
            xy_variance=25.0,
            yaw_variance=9.0,
        )
        route = [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)]

        updated, applied, distance, index = replace_startup_pose_yaw_from_route(
            msg,
            route,
            enabled=True,
            max_distance_m=1.0,
            neighbor_stride=1,
            yaw_variance=0.02,
        )

        self.assertTrue(applied)
        self.assertAlmostEqual(distance, 0.2)
        self.assertEqual(index, 1)
        self.assertAlmostEqual(updated.pose.pose.position.x, msg.pose.pose.position.x)
        self.assertAlmostEqual(updated.pose.pose.position.y, msg.pose.pose.position.y)
        self.assertAlmostEqual(updated.pose.covariance[0], 25.0)
        self.assertAlmostEqual(updated.pose.covariance[35], 0.02)
        self.assertAlmostEqual(_yaw_from_quaternion(updated.pose.pose.orientation), math.pi / 2.0)
        self.assertAlmostEqual(_yaw_from_quaternion(msg.pose.pose.orientation), 1.5)

    def test_startup_route_heading_can_snap_xy_to_nearest_route_sample(self):
        msg = make_pose(Time(seconds=2.0), x=0.2, y=1.0, yaw=1.5)
        route = [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)]

        updated, applied, _, index = replace_startup_pose_yaw_from_route(
            msg,
            route,
            enabled=True,
            max_distance_m=1.0,
            neighbor_stride=1,
            yaw_variance=0.02,
            snap_xy_to_route=True,
        )

        self.assertTrue(applied)
        self.assertEqual(index, 1)
        self.assertAlmostEqual(updated.pose.pose.position.x, 0.0)
        self.assertAlmostEqual(updated.pose.pose.position.y, 1.0)
        self.assertAlmostEqual(_yaw_from_quaternion(updated.pose.pose.orientation), math.pi / 2.0)

    def test_startup_route_heading_disabled_keeps_original_pose_object(self):
        msg = make_pose(Time(seconds=2.0), x=0.0, y=0.0, yaw=1.0)

        updated, applied, distance, index = replace_startup_pose_yaw_from_route(
            msg,
            [(0.0, 0.0), (1.0, 0.0)],
            enabled=False,
            max_distance_m=1.0,
            neighbor_stride=1,
            yaw_variance=0.02,
        )

        self.assertIs(updated, msg)
        self.assertFalse(applied)
        self.assertIsNone(distance)
        self.assertIsNone(index)

    def test_ekf_feedback_requires_fresh_measurement_backing(self):
        ekf_stamp = Time(seconds=40.0)
        fresh_measurement = Time(seconds=39.9)
        stale_measurement = Time(seconds=39.0)

        self.assertTrue(
            feedback_pose_is_measurement_backed(
                ekf_stamp, fresh_measurement, max_age_sec=0.2
            )
        )
        self.assertFalse(
            feedback_pose_is_measurement_backed(
                ekf_stamp, stale_measurement, max_age_sec=0.2
            )
        )
        self.assertFalse(
            feedback_pose_is_measurement_backed(ekf_stamp, None, max_age_sec=0.2)
        )

    def test_ekf_feedback_gate_does_not_mask_lost_with_open_loop_ekf(self):
        node = EkfFeedbackGate()
        try:
            publisher = RecordingPublisher()
            node._publisher = publisher
            node._max_measurement_age_sec = 0.2

            node._on_ekf_pose(make_pose(Time(seconds=50.0), x=1.0))
            self.assertEqual(publisher.messages, [])

            node._on_measurement_pose(make_pose(Time(seconds=50.1), x=2.0))
            ekf_pose = make_pose(Time(seconds=50.2), x=3.0, xy_variance=0.3)
            node._on_ekf_pose(ekf_pose)
            self.assertEqual(publisher.messages, [ekf_pose])
            self.assertAlmostEqual(publisher.messages[0].pose.pose.position.x, 3.0)
            self.assertAlmostEqual(publisher.messages[0].pose.covariance[0], 0.3)

            node._on_ekf_pose(make_pose(Time(seconds=51.0), x=4.0))
            self.assertEqual(len(publisher.messages), 1)
        finally:
            node.destroy_node()

    def test_ekf_feedback_gate_publishes_once_per_measurement(self):
        node = EkfFeedbackGate()
        try:
            publisher = RecordingPublisher()
            node._publisher = publisher
            node._max_measurement_age_sec = 0.2

            node._on_measurement_pose(make_pose(Time(seconds=60.0), x=1.0))
            first_ekf = make_pose(Time(seconds=60.01), x=2.0)
            node._on_ekf_pose(first_ekf)
            node._on_ekf_pose(make_pose(Time(seconds=60.02), x=3.0))
            self.assertEqual(publisher.messages, [first_ekf])

            second_measurement = make_pose(Time(seconds=60.05), x=4.0)
            node._on_measurement_pose(second_measurement)
            second_ekf = make_pose(Time(seconds=60.06), x=5.0)
            node._on_ekf_pose(second_ekf)
            self.assertEqual(publisher.messages, [first_ekf, second_ekf])
        finally:
            node.destroy_node()

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

        x, y, yaw = _propagate(0.0, 0.0, 0.0, 2.0, 0.0, 1.0, lateral_velocity=1.0)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(yaw, 0.0)

        x, y, yaw = _propagate(0.0, 0.0, 0.0, 2.0, 1.0, 1.0)
        self.assertAlmostEqual(x, 2.0 * math.sin(1.0))
        self.assertAlmostEqual(y, 2.0 * (1.0 - math.cos(1.0)))
        self.assertAlmostEqual(yaw, 1.0)

    def test_fixposition_odometry_seed_applies_gnss_lever_arm_to_base_link(self):
        stamp = Time(seconds=12.3).to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "gnss_base_link"
        msg.pose.pose.position.x = 10.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.position.z = 3.0
        msg.pose.pose.orientation = _yaw_to_quaternion(BASE_TO_GNSS_YAW)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.36
        msg.pose.covariance[35] = 0.04

        seed = odometry_to_seed_pose(msg)

        self.assertEqual(seed.header.stamp, stamp)
        self.assertEqual(seed.header.frame_id, "map")
        self.assertAlmostEqual(seed.pose.pose.position.x, 10.0 - BASE_TO_GNSS_TRANSLATION[0])
        self.assertAlmostEqual(seed.pose.pose.position.y, 2.0)
        self.assertAlmostEqual(seed.pose.pose.position.z, 3.0 - BASE_TO_GNSS_TRANSLATION[2])
        self.assertAlmostEqual(_yaw_from_quaternion(seed.pose.pose.orientation), 0.0)
        self.assertEqual(seed.pose.covariance[0], 0.25)
        self.assertEqual(seed.pose.covariance[7], 0.36)
        self.assertEqual(seed.pose.covariance[35], 0.04)

    def test_fixposition_odometry_seed_can_override_reported_covariance(self):
        msg = Odometry()
        msg.header.stamp = Time(seconds=12.3).to_msg()
        msg.pose.pose.orientation = _yaw_to_quaternion(BASE_TO_GNSS_YAW)
        msg.pose.covariance[0] = 1.0
        msg.pose.covariance[7] = 1.0
        msg.pose.covariance[14] = 4.0
        msg.pose.covariance[35] = 0.25

        seed = odometry_to_seed_pose(
            msg,
            reported_xy_sigma_m=0.1,
            reported_z_sigma_m=0.2,
            reported_yaw_sigma_deg=0.5,
        )

        self.assertAlmostEqual(seed.pose.covariance[0], 0.01)
        self.assertAlmostEqual(seed.pose.covariance[7], 0.01)
        self.assertAlmostEqual(seed.pose.covariance[14], 0.04)
        self.assertAlmostEqual(seed.pose.covariance[35], math.radians(0.5) ** 2)

    def test_predictor_uses_heading_rate_then_steering_fallback(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()

            velocity = VelocityReport()
            velocity.longitudinal_velocity = 2.0
            velocity.lateral_velocity = 0.5
            velocity.heading_rate = 0.3
            node._last_velocity = velocity
            node._last_velocity_receipt = now
            self.assertEqual(node._motion(now), (2.0, 0.5, 0.3))

            velocity.heading_rate = math.nan
            steering = SteeringReport()
            steering.steering_tire_angle = 0.1
            node._wheel_base = 2.0
            node._last_steering = steering
            node._last_steering_receipt = now
            motion = node._motion(now)
            self.assertAlmostEqual(motion[0], 2.0)
            self.assertAlmostEqual(motion[1], 0.5)
            self.assertAlmostEqual(motion[2], math.tan(0.1))
        finally:
            node.destroy_node()

    def test_predictor_integrates_lateral_velocity_from_vehicle_status(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()
            node._vehicle_status_timeout = 10.0
            node._state = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "covariance": [0.0] * 36,
                "stamp": now - Duration(seconds=1.0),
            }
            velocity = VelocityReport()
            velocity.longitudinal_velocity = 0.0
            velocity.lateral_velocity = 1.25
            velocity.heading_rate = 0.0
            node._last_velocity = velocity
            node._last_velocity_receipt = now

            node._advance_state(now)

            self.assertAlmostEqual(node._state["x"], 0.0)
            self.assertAlmostEqual(node._state["y"], 1.25)
            self.assertEqual(node._state["stamp"], now)
        finally:
            node.destroy_node()

    def test_predictor_preserves_roll_pitch_in_initial_pose_output(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()
            node._on_seed_pose(make_pose(now, roll=0.12, pitch=-0.08, yaw=0.3))

            output = node._state_to_msg(now)
            roll, pitch, yaw = _rpy_from_quaternion(output.pose.pose.orientation)

            self.assertAlmostEqual(roll, 0.12)
            self.assertAlmostEqual(pitch, -0.08)
            self.assertAlmostEqual(yaw, 0.3)
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

    def test_predictor_ignores_out_of_order_time_without_rewinding_state_stamp(self):
        node = NdtInitialPosePredictor()
        try:
            clock_type = node.get_clock().clock_type
            start = Time(seconds=10.0, clock_type=clock_type)
            older = Time(seconds=9.9, clock_type=clock_type)
            later = Time(seconds=10.1, clock_type=clock_type)
            node._vehicle_status_timeout = 10.0
            node._state = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "covariance": [0.0] * 36,
                "stamp": start,
            }
            velocity = VelocityReport()
            velocity.longitudinal_velocity = 10.0
            velocity.heading_rate = 0.0
            node._last_velocity = velocity
            node._last_velocity_receipt = start

            node._advance_state(older)

            self.assertAlmostEqual(node._state["x"], 0.0)
            self.assertEqual(node._state["stamp"], start)

            node._advance_state(later)

            self.assertAlmostEqual(node._state["x"], 1.0)
            self.assertEqual(node._state["stamp"], later)
        finally:
            node.destroy_node()

    def test_predictor_does_not_advance_state_past_node_clock_on_status_callback(self):
        node = NdtInitialPosePredictor()
        try:
            now = node.get_clock().now()
            future = now + Duration(seconds=1.0)
            node._vehicle_status_timeout = 10.0
            node._state = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "covariance": [0.0] * 36,
                "stamp": now,
            }

            velocity = VelocityReport()
            velocity.header.stamp = future.to_msg()
            velocity.longitudinal_velocity = 10.0
            velocity.heading_rate = 0.0

            node._on_velocity(velocity)

            self.assertAlmostEqual(node._state["x"], 0.0)
            self.assertEqual(node._state["stamp"].nanoseconds, now.nanoseconds)
            self.assertEqual(node._last_velocity_receipt.nanoseconds, future.nanoseconds)
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

    def test_predictor_cycles_recovery_hypotheses_without_mutating_state(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_lost_recovery_hypotheses = True
            node._recovery_hypothesis_period = 0.5
            node._recovery_hypotheses = [
                (0.0, 0.0, 0.0),
                (3.0, 1.0, math.radians(5.0)),
            ]
            node._ndt_lost_timeout = 1.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            lost = Time(seconds=1.6, clock_type=clock_type)
            node._state = {
                "x": 10.0,
                "y": 20.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": math.pi / 2.0,
                "covariance": [0.0] * 36,
                "stamp": start,
            }
            node._last_ndt_receipt = start

            recovery_msg = node._state_to_msg(lost)
            prediction_msg = node._state_to_msg(lost, include_recovery_hypothesis=False)

            self.assertAlmostEqual(recovery_msg.pose.pose.position.x, 9.0)
            self.assertAlmostEqual(recovery_msg.pose.pose.position.y, 23.0)
            self.assertAlmostEqual(
                _yaw_from_quaternion(recovery_msg.pose.pose.orientation),
                math.pi / 2.0 + math.radians(5.0),
            )
            self.assertAlmostEqual(node._state["x"], 10.0)
            self.assertAlmostEqual(node._state["y"], 20.0)
            self.assertAlmostEqual(node._state["yaw"], math.pi / 2.0)
            self.assertAlmostEqual(prediction_msg.pose.pose.position.x, 10.0)
            self.assertAlmostEqual(prediction_msg.pose.pose.position.y, 20.0)
            self.assertAlmostEqual(
                _yaw_from_quaternion(prediction_msg.pose.pose.orientation),
                math.pi / 2.0,
            )
        finally:
            node.destroy_node()

    def test_predictor_publishes_relocalization_decision_for_recovery_hypothesis(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_lost_recovery_hypotheses = True
            node._recovery_hypothesis_period = 0.5
            node._recovery_hypotheses = [(2.0, -0.5, math.radians(-3.0))]
            node._ndt_lost_timeout = 1.0
            node._relocalization_decision_publisher = RecordingPublisher()
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            lost = Time(seconds=1.6, clock_type=clock_type)
            node._state = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "covariance": [0.0] * 36,
                "stamp": start,
            }
            node._last_ndt_receipt = start

            node._state_to_msg(lost)

            self.assertEqual(len(node._relocalization_decision_publisher.messages), 1)
            payload = json.loads(node._relocalization_decision_publisher.messages[0].data)
            self.assertEqual(payload["reason"], "lost_recovery_hypothesis")
            self.assertFalse(payload["uses_gnss_or_gt"])
            self.assertAlmostEqual(payload["along_offset_m"], 2.0)
            self.assertAlmostEqual(payload["cross_offset_m"], -0.5)
            self.assertAlmostEqual(payload["yaw_offset_deg"], -3.0)
        finally:
            node.destroy_node()

    def test_predictor_can_inject_reproducible_motion_model_error(self):
        node = NdtInitialPosePredictor()
        try:
            stamp = Time(seconds=1.0)
            node._motion_velocity_scale_error = 0.10
            node._motion_longitudinal_velocity_bias = -0.2
            node._motion_velocity_white_noise_stddev = 0.0
            node._motion_yaw_rate_bias = 0.01
            node._motion_yaw_rate_random_walk_stddev = 0.0

            velocity, lateral_velocity, yaw_rate = node._apply_motion_noise(
                stamp,
                10.0,
                0.5,
                0.2,
            )

            self.assertAlmostEqual(velocity, 10.8)
            self.assertAlmostEqual(lateral_velocity, 0.5)
            self.assertAlmostEqual(yaw_rate, 0.21)
        finally:
            node.destroy_node()

    def test_predictor_applies_learned_motion_scale_correction(self):
        node = NdtInitialPosePredictor()
        try:
            node._motion_velocity_scale_error = 0.01
            node._motion_velocity_scale_correction = -0.01

            velocity, _lateral_velocity, _yaw_rate = node._apply_motion_noise(
                Time(seconds=1.0),
                10.0,
                0.0,
                0.0,
            )

            self.assertAlmostEqual(velocity, 10.0 * 1.01 * 0.99)
        finally:
            node.destroy_node()

    def test_predictor_learns_motion_scale_correction_from_accepted_ndt_displacements(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.0), end)

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                10.0 / 10.1 - 1.0,
            )
        finally:
            node.destroy_node()

    def test_predictor_keeps_motion_scale_anchor_until_min_distance(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 20.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._vehicle_status_timeout = 3.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            mid = Time(seconds=1.0, clock_type=clock_type)
            end = Time(seconds=2.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": mid.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]

            node._update_motion_scale_correction_from_pose(make_pose(mid, x=10.0), mid)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._last_motion_scale_pose_sample["stamp_ns"], mid.nanoseconds)

            node._update_motion_scale_correction_from_pose(make_pose(end, x=20.0), end)

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                20.0 / 20.2 - 1.0,
            )
            self.assertEqual(node._last_motion_scale_pose_sample["stamp_ns"], end.nanoseconds)
        finally:
            node.destroy_node()

    def test_predictor_motion_scale_correction_tracks_observation_without_integrating_runaway(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 0.5
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._vehicle_status_timeout = 3.0
            clock_type = node.get_clock().clock_type
            t0 = Time(seconds=0.0, clock_type=clock_type)
            t1 = Time(seconds=1.0, clock_type=clock_type)
            t2 = Time(seconds=2.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": t0.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": t0.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": t1.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": t2.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            observation = 10.0 / 10.1 - 1.0

            node._update_motion_scale_correction_from_pose(make_pose(t1, x=10.0), t1)
            node._update_motion_scale_correction_from_pose(make_pose(t2, x=20.0), t2)

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                0.75 * observation,
            )
        finally:
            node.destroy_node()

    def test_predictor_defers_motion_scale_learning_until_min_stamp(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_min_stamp_sec = 10.0
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                }
            ]

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.2), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertIsNone(node._last_motion_scale_pose_sample)
        finally:
            node.destroy_node()

    def test_predictor_skips_motion_scale_learning_on_ambiguous_runtime_multistart(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_runtime_decision_max_age = 0.2
            node._motion_scale_correction_skip_ambiguous_runtime = True
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_runtime_multistart_decision = {
                "stamp_sec": 1.0,
                "small_tier_ambiguous": True,
                "tier2_evaluated": True,
                "recovery_active": False,
                "candidate_count": 17,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.0), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertIsNone(node._last_motion_scale_pose_sample)
        finally:
            node.destroy_node()

    def test_predictor_allows_motion_scale_learning_when_runtime_multistart_selects_candidate(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_runtime_decision_max_age = 0.2
            node._motion_scale_correction_skip_ambiguous_runtime = True
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_runtime_multistart_decision = {
                "stamp_sec": 1.0,
                "candidate_count": 11,
                "selected_candidate_index": 4,
                "small_tier_ambiguous": False,
                "tier2_evaluated": False,
                "recovery_active": False,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.0), end)

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                10.0 / 10.1 - 1.0,
            )
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_allows_motion_scale_learning_when_runtime_multistart_keeps_base(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_runtime_decision_max_age = 0.2
            node._motion_scale_correction_skip_ambiguous_runtime = True
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_runtime_multistart_decision = {
                "stamp_sec": 1.0,
                "candidate_count": 9,
                "selected_candidate_index": 0,
                "small_tier_ambiguous": False,
                "tier2_evaluated": False,
                "recovery_active": False,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.0), end)

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                10.0 / 10.1 - 1.0,
            )
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_preserves_trusted_motion_scale_accumulator_across_candidate_freeze(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 20.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_runtime_decision_max_age = 0.2
            node._motion_scale_correction_skip_ambiguous_runtime = True
            node._vehicle_status_timeout = 5.0
            clock_type = node.get_clock().clock_type
            t0 = Time(seconds=0.0, clock_type=clock_type)
            t1 = Time(seconds=1.0, clock_type=clock_type)
            frozen = Time(seconds=2.0, clock_type=clock_type)
            t3 = Time(seconds=3.0, clock_type=clock_type)
            t4 = Time(seconds=4.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": t0.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": stamp.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                }
                for stamp in (t0, t1, frozen, t3, t4)
            ]
            node._last_runtime_multistart_decision = {
                "stamp_sec": 1.0,
                "candidate_count": 1,
                "selected_candidate_index": 0,
                "small_tier_ambiguous": False,
                "tier2_evaluated": False,
                "recovery_active": False,
            }

            node._update_motion_scale_correction_from_pose(make_pose(t1, x=10.0), t1)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertGreater(node._motion_scale_correction_motion_accum, 0.0)

            node._last_runtime_multistart_decision = {
                "stamp_sec": 2.0,
                "candidate_count": 11,
                "selected_candidate_index": 4,
                "small_tier_ambiguous": True,
                "tier2_evaluated": False,
                "recovery_active": False,
            }

            node._update_motion_scale_correction_from_pose(
                make_pose(frozen, x=100.0), frozen
            )

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertIsNone(node._last_motion_scale_pose_sample)

            node._last_runtime_multistart_decision = {
                "stamp_sec": 3.0,
                "candidate_count": 1,
                "selected_candidate_index": 0,
                "small_tier_ambiguous": False,
                "tier2_evaluated": False,
                "recovery_active": False,
            }
            node._update_motion_scale_correction_from_pose(make_pose(t3, x=20.0), t3)

            node._last_runtime_multistart_decision = {
                "stamp_sec": 4.0,
                "candidate_count": 1,
                "selected_candidate_index": 0,
                "small_tier_ambiguous": False,
                "tier2_evaluated": False,
                "recovery_active": False,
            }
            node._update_motion_scale_correction_from_pose(make_pose(t4, x=30.0), t4)

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                20.0 / 20.2 - 1.0,
            )
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_motion_scale_learning_requires_accepted_robust_decision(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.5
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 1.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            rejected = Time(seconds=1.0, clock_type=clock_type)
            accepted = Time(seconds=2.0, clock_type=clock_type)
            next_accepted = Time(seconds=3.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": rejected.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": accepted.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": next_accepted.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": False,
                "reason": "mahalanobis_reject",
                "mahalanobis": 6.0,
                "innovation_along_m": 1.8,
                "innovation_cross_m": 0.1,
                "innovation_yaw_deg": 0.2,
            }

            node._update_motion_scale_correction_from_pose(
                make_pose(rejected, x=10.0), rejected
            )

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertIsNone(node._last_motion_scale_pose_sample)

            node._last_robust_ndt_decision = {
                "stamp_sec": 2.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.1,
                "innovation_cross_m": 0.05,
                "innovation_yaw_deg": 0.2,
            }

            node._update_motion_scale_correction_from_pose(
                make_pose(accepted, x=20.0), accepted
            )

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertEqual(
                node._last_motion_scale_pose_sample["stamp_ns"],
                accepted.nanoseconds,
            )

            node._last_robust_ndt_decision = {
                "stamp_sec": 3.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.1,
                "innovation_cross_m": 0.05,
                "innovation_yaw_deg": 0.2,
            }

            node._update_motion_scale_correction_from_pose(
                make_pose(next_accepted, x=30.0), next_accepted
            )

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                10.0 / 10.1 - 1.0,
            )
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_requires_stable_initial_motion_scale_observations(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.5
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 1.0
            node._motion_scale_correction_min_stamp_sec = 0.0
            node._motion_scale_correction_bootstrap_initial_observation_count = 3
            node._vehicle_status_timeout = 3.0
            clock_type = node.get_clock().clock_type
            stamps = [Time(seconds=float(sec), clock_type=clock_type) for sec in (0, 2, 4, 6)]
            node._last_motion_scale_pose_sample = {
                "stamp_ns": stamps[0].nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": stamp.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                }
                for stamp in stamps
            ]

            for index, stamp in enumerate(stamps[1:], start=1):
                node._last_robust_ndt_decision = {
                    "stamp_sec": float(index * 2),
                    "accepted": True,
                    "reason": "ekf_measurement_update",
                    "mahalanobis": 0.8,
                    "innovation_along_m": -0.2,
                    "innovation_cross_m": 0.05,
                    "innovation_yaw_deg": 0.2,
                }
                node._update_motion_scale_correction_from_pose(
                    make_pose(stamp, x=19.8 * index), stamp
                )
                if index < 3:
                    self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
                    self.assertEqual(node._motion_scale_correction_update_count, 0)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.002)
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_rejected_motion_scale_sample_does_not_replace_learning_anchor(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.5
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 1.0
            node._vehicle_status_timeout = 3.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            rejected = Time(seconds=1.0, clock_type=clock_type)
            accepted = Time(seconds=2.0, clock_type=clock_type)
            next_accepted = Time(seconds=3.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": rejected.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": accepted.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": next_accepted.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": False,
                "reason": "mahalanobis_reject",
                "mahalanobis": 8.0,
                "innovation_along_m": 9.0,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(
                make_pose(rejected, x=100.0), rejected
            )

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertIsNone(node._last_motion_scale_pose_sample)

            node._last_robust_ndt_decision = {
                "stamp_sec": 2.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.1,
                "innovation_cross_m": 0.05,
                "innovation_yaw_deg": 0.2,
            }

            node._update_motion_scale_correction_from_pose(
                make_pose(accepted, x=20.0), accepted
            )

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertEqual(
                node._last_motion_scale_pose_sample["stamp_ns"],
                accepted.nanoseconds,
            )

            node._last_robust_ndt_decision = {
                "stamp_sec": 3.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.1,
                "innovation_cross_m": 0.05,
                "innovation_yaw_deg": 0.2,
            }

            node._update_motion_scale_correction_from_pose(
                make_pose(next_accepted, x=30.0), next_accepted
            )

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                10.0 / 10.1 - 1.0,
            )
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_rejects_motion_scale_learning_when_robust_innovation_disagrees(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=11.0), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.0)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
        finally:
            node.destroy_node()

    def test_predictor_rejects_robust_disagreement_catchup_before_bootstrap(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_velocity_scale_correction = -0.002
            node._motion_scale_correction_update_count = 1
            node._motion_scale_correction_bootstrap_min_abs = 0.01
            node._motion_scale_correction_bootstrap_min_updates = 3
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.2), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.002)
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_uses_robust_disagreement_catchup_after_bootstrap(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_velocity_scale_correction = -0.006
            node._motion_scale_correction_update_count = 3
            node._motion_scale_correction_bootstrap_min_abs = 0.01
            node._motion_scale_correction_bootstrap_min_updates = 3
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.2), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.008)
            self.assertEqual(node._motion_scale_correction_update_count, 4)
            self.assertAlmostEqual(node._motion_scale_correction_motion_accum, 0.0)
        finally:
            node.destroy_node()

    def test_predictor_uses_accepted_displacement_scale_when_robust_innovation_agrees(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.1,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.0), end)

            self.assertAlmostEqual(
                node._motion_velocity_scale_correction,
                10.0 / 10.1 - 1.0,
            )
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_allows_robust_agreed_opposite_motion_scale_observation(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_velocity_scale_correction = -0.012
            node._motion_scale_correction_bootstrap_min_abs = 0.01
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._motion_scale_correction_opposite_observation_required_count = 3
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": 0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.2), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.010)
            self.assertEqual(node._motion_scale_correction_update_count, 1)
            self.assertEqual(
                node._last_motion_scale_pose_sample["stamp_ns"],
                end.nanoseconds,
            )
            self.assertAlmostEqual(node._motion_scale_correction_motion_accum, 0.0)
        finally:
            node.destroy_node()

    def test_predictor_blocks_opposite_motion_scale_observation_during_bootstrap(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_velocity_scale_correction = -0.002
            node._motion_scale_correction_update_count = 1
            node._motion_scale_correction_bootstrap_min_abs = 0.01
            node._motion_scale_correction_bootstrap_min_updates = 3
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": 0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.2), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.002)
            self.assertEqual(node._motion_scale_correction_update_count, 1)
            self.assertEqual(
                node._last_motion_scale_pose_sample["stamp_ns"],
                end.nanoseconds,
            )
            self.assertAlmostEqual(node._motion_scale_correction_motion_accum, 0.0)
        finally:
            node.destroy_node()

    def test_predictor_allows_opposite_motion_scale_observation_after_bootstrap_updates(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_velocity_scale_correction = -0.006
            node._motion_scale_correction_update_count = 3
            node._motion_scale_correction_bootstrap_min_abs = 0.01
            node._motion_scale_correction_bootstrap_min_updates = 3
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": 0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.2), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.004)
            self.assertEqual(node._motion_scale_correction_update_count, 4)
            self.assertAlmostEqual(node._motion_scale_correction_motion_accum, 0.0)
        finally:
            node.destroy_node()

    def test_predictor_preserves_tracking_along_when_ndt_pose_jumps_basin(self):
        node = NdtInitialPosePredictor()
        try:
            node._preserve_tracking_ndt_along = True
            node._tracking_ndt_max_along_correction = 0.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=10.0, clock_type=clock_type)
            update = Time(seconds=11.0, clock_type=clock_type)
            node._set_state_from_pose(make_pose(start, x=10.0, y=0.0, yaw=0.0), start, "test")
            node._predictor_state = STATE_TRACKING

            self.assertTrue(
                node._set_state_from_tracking_ndt_pose(
                    make_pose(update, x=12.0, y=0.4, yaw=0.1),
                    update,
                    "NDT",
                )
            )

            forward_x = math.cos(0.1)
            forward_y = math.sin(0.1)
            lateral_x = -math.sin(0.1)
            lateral_y = math.cos(0.1)
            state_dx = node._state["x"] - 10.0
            state_dy = node._state["y"] - 0.0
            ndt_dx = 12.0 - 10.0
            ndt_dy = 0.4 - 0.0
            self.assertAlmostEqual(state_dx * forward_x + state_dy * forward_y, 0.0)
            self.assertAlmostEqual(
                state_dx * lateral_x + state_dy * lateral_y,
                ndt_dx * lateral_x + ndt_dy * lateral_y,
            )
            self.assertAlmostEqual(node._state["yaw"], 0.1, places=6)
            self.assertEqual(node._state["stamp"].nanoseconds, update.nanoseconds)
        finally:
            node.destroy_node()

    def test_predictor_uses_robust_scale_observation_when_tracking_along_is_preserved(self):
        node = NdtInitialPosePredictor()
        try:
            node._preserve_tracking_ndt_along = True
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_velocity_scale_correction = 0.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": -0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.2), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.002)
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_predictor_preserves_motion_scale_correction_on_neutral_scale_observation(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_velocity_scale_correction = -0.012
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._motion_scale_correction_require_robust_decision = True
            node._motion_scale_correction_robust_decision_max_age = 0.2
            node._motion_scale_correction_max_mahalanobis = 2.0
            node._motion_scale_correction_max_innovation_along = 0.8
            node._motion_scale_correction_max_innovation_cross = 0.25
            node._motion_scale_correction_max_innovation_yaw_deg = 2.0
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]
            node._last_robust_ndt_decision = {
                "stamp_sec": 1.0,
                "accepted": True,
                "reason": "ekf_measurement_update",
                "mahalanobis": 0.8,
                "innovation_along_m": 0.2,
                "innovation_cross_m": 0.0,
                "innovation_yaw_deg": 0.0,
            }

            node._update_motion_scale_correction_from_pose(make_pose(end, x=10.0), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, -0.012)
            self.assertEqual(node._motion_scale_correction_update_count, 0)
            self.assertEqual(node._motion_scale_correction_opposite_observation_streak, 0)
        finally:
            node.destroy_node()

    def test_predictor_motion_scale_learning_clamps_each_update_step(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_motion_scale_correction = True
            node._motion_scale_correction_alpha = 1.0
            node._motion_scale_correction_max_abs = 0.03
            node._motion_scale_correction_max_step_abs = 0.002
            node._motion_scale_correction_observation_limit = 0.05
            node._motion_scale_correction_min_distance = 1.0
            node._motion_scale_correction_max_cross_residual = 0.5
            node._vehicle_status_timeout = 2.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            end = Time(seconds=1.0, clock_type=clock_type)
            node._last_motion_scale_pose_sample = {
                "stamp_ns": start.nanoseconds,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            }
            node._motion_history = [
                {
                    "stamp_ns": start.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
                {
                    "stamp_ns": end.nanoseconds,
                    "velocity": 10.0,
                    "lateral_velocity": 0.0,
                    "yaw_rate": 0.0,
                },
            ]

            node._update_motion_scale_correction_from_pose(make_pose(end, x=11.0), end)

            self.assertAlmostEqual(node._motion_velocity_scale_correction, 0.002)
            self.assertEqual(node._motion_scale_correction_update_count, 1)
        finally:
            node.destroy_node()

    def test_prediction_fallback_requires_first_accepted_update_and_min_age(self):
        accepted = Time(seconds=10.0)
        early = Time(seconds=10.1)
        due = Time(seconds=10.3)

        self.assertFalse(
            _prediction_fallback_due(
                first_accepted_seen=False,
                last_accepted_stamp=accepted,
                current_stamp=due,
                min_age_sec=0.2,
            )
        )
        self.assertFalse(
            _prediction_fallback_due(
                first_accepted_seen=True,
                last_accepted_stamp=accepted,
                current_stamp=early,
                min_age_sec=0.2,
            )
        )
        self.assertTrue(
            _prediction_fallback_due(
                first_accepted_seen=True,
                last_accepted_stamp=accepted,
                current_stamp=due,
                min_age_sec=0.2,
            )
        )

    def test_prediction_fallback_inflates_covariance_floor(self):
        stamp = Time(seconds=1.0)
        initial = make_pose(stamp, xy_variance=0.01, yaw_variance=0.001)

        fallback = _make_prediction_fallback_msg(
            initial,
            xy_variance_floor=4.0,
            yaw_variance_floor=0.25,
        )

        self.assertAlmostEqual(fallback.pose.covariance[0], 4.0)
        self.assertAlmostEqual(fallback.pose.covariance[7], 4.0)
        self.assertAlmostEqual(fallback.pose.covariance[35], 0.25)
        self.assertAlmostEqual(initial.pose.covariance[0], 0.01)

    def test_predictor_replays_vehicle_history_after_delayed_ndt_correction(self):
        node = NdtInitialPosePredictor()
        try:
            clock_type = node.get_clock().clock_type
            low_speed = Time(seconds=0.0, clock_type=clock_type)
            correction = Time(seconds=0.5, clock_type=clock_type)
            high_speed = Time(seconds=1.0, clock_type=clock_type)
            target = Time(seconds=2.0, clock_type=clock_type)
            node._vehicle_status_timeout = 10.0

            velocity = VelocityReport()
            velocity.header.stamp = low_speed.to_msg()
            velocity.longitudinal_velocity = 1.0
            velocity.heading_rate = 0.0
            node._on_velocity(velocity)

            velocity = VelocityReport()
            velocity.header.stamp = high_speed.to_msg()
            velocity.longitudinal_velocity = 3.0
            velocity.heading_rate = 0.0
            node._on_velocity(velocity)

            node._on_ndt_pose(make_pose(correction, x=0.5, yaw=0.0))
            node._advance_state(target)

            self.assertAlmostEqual(node._state["x"], 4.0)
            self.assertAlmostEqual(node._state["y"], 0.0)
            self.assertEqual(node._state["stamp"], target)
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

    def test_predictor_lost_recovery_seed_preserves_small_dead_reckoned_along(self):
        node = NdtInitialPosePredictor()
        try:
            node._ndt_lost_timeout = 1.0
            node._vehicle_status_timeout = 10.0
            node._max_lost_recovery_along_residual = 1.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            recovery = Time(seconds=2.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(start, x=0.0, y=0.0, yaw=0.0))
            node._on_ndt_pose(make_pose(start, x=10.0, y=0.0, yaw=0.0))

            velocity = VelocityReport()
            velocity.header.stamp = start.to_msg()
            velocity.longitudinal_velocity = 2.0
            velocity.heading_rate = 0.0
            node._last_velocity = velocity
            node._last_velocity_receipt = start

            node._on_seed_pose(make_pose(recovery, x=13.5, y=5.0, yaw=0.0))

            self.assertEqual(node._predictor_state, STATE_LOST_RECOVERY)
            self.assertEqual(node._lost_recovery_seed_reset_count, 1)
            self.assertAlmostEqual(node._state["x"], 14.0)
            self.assertAlmostEqual(node._state["y"], 5.0)
            self.assertAlmostEqual(_yaw_from_quaternion(node._state_to_msg(recovery).pose.pose.orientation), 0.0)
        finally:
            node.destroy_node()

    def test_predictor_lost_recovery_seed_drops_large_dead_reckoned_along(self):
        node = NdtInitialPosePredictor()
        try:
            node._ndt_lost_timeout = 1.0
            node._vehicle_status_timeout = 10.0
            node._max_lost_recovery_along_residual = 1.0
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)
            recovery = Time(seconds=2.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(start, x=0.0, y=0.0, yaw=0.0))
            node._on_ndt_pose(make_pose(start, x=10.0, y=0.0, yaw=0.0))

            velocity = VelocityReport()
            velocity.header.stamp = start.to_msg()
            velocity.longitudinal_velocity = 2.0
            velocity.heading_rate = 0.0
            node._last_velocity = velocity
            node._last_velocity_receipt = start

            node._on_seed_pose(make_pose(recovery, x=30.0, y=5.0, yaw=0.0))

            self.assertEqual(node._predictor_state, STATE_LOST_RECOVERY)
            self.assertEqual(node._lost_recovery_seed_reset_count, 1)
            self.assertAlmostEqual(node._state["x"], 30.0)
            self.assertAlmostEqual(node._state["y"], 5.0)
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

    def test_predictor_tracking_seed_along_fusion_does_not_change_cross_or_yaw(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_tracking_seed_fusion = False
            node._enable_tracking_seed_along_fusion = True
            node._tracking_seed_along_gain = 0.1
            node._tracking_seed_along_min_interval = 0.1
            node._max_tracking_seed_stddev = 5.0
            node._max_tracking_seed_age = 0.5
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=1.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(stamp, x=0.0, y=0.0, xy_variance=9.0))
            node._on_ndt_pose(make_pose(stamp, x=10.0, y=2.0, yaw=0.0, xy_variance=0.09))

            node._on_seed_pose(make_pose(stamp, x=20.0, y=100.0, yaw=1.0, xy_variance=9.0))

            self.assertAlmostEqual(node._state["x"], 11.0)
            self.assertAlmostEqual(node._state["y"], 2.0)
            self.assertAlmostEqual(node._state["yaw"], 0.0)
            self.assertEqual(node._tracking_seed_along_fusion_count, 1)
            self.assertEqual(node._tracking_seed_ignored_count, 0)
        finally:
            node.destroy_node()

    def test_predictor_tracking_seed_along_fusion_is_rate_limited(self):
        node = NdtInitialPosePredictor()
        try:
            node._enable_tracking_seed_fusion = False
            node._enable_tracking_seed_along_fusion = True
            node._tracking_seed_along_gain = 0.5
            node._tracking_seed_along_min_interval = 0.1
            node._max_tracking_seed_stddev = 5.0
            node._max_tracking_seed_age = 0.5
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=1.0, clock_type=clock_type)

            node._on_seed_pose(make_pose(stamp, x=0.0, xy_variance=9.0))
            node._on_ndt_pose(make_pose(stamp, x=10.0, yaw=0.0, xy_variance=0.09))
            node._on_seed_pose(make_pose(stamp, x=20.0, xy_variance=9.0))
            self.assertAlmostEqual(node._state["x"], 15.0)

            node._on_seed_pose(make_pose(Time(seconds=1.05, clock_type=clock_type), x=30.0, xy_variance=9.0))
            self.assertAlmostEqual(node._state["x"], 15.0)
            self.assertEqual(node._tracking_seed_along_fusion_count, 1)
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

    def test_axis_fuser_no_rtk_mode_keeps_ndt_cross_yaw_and_softens_seed_along(self):
        stamp = Time(seconds=1.0)
        ndt = make_pose(stamp, x=10.0, y=2.0, yaw=0.0)
        seed = make_pose(stamp, x=14.0, y=-5.0, yaw=1.2, xy_variance=9.0)

        fused, applied = _fuse_ndt_cross_yaw_seed_along_pose(ndt, seed, along_gain=0.05)

        self.assertTrue(applied)
        self.assertAlmostEqual(fused.pose.pose.position.x, 10.2)
        self.assertAlmostEqual(fused.pose.pose.position.y, 2.0)
        self.assertAlmostEqual(_yaw_from_quaternion(fused.pose.pose.orientation), 0.0)

    def test_axis_fuser_no_rtk_mode_uses_ndt_yaw_for_projection(self):
        stamp = Time(seconds=1.0)
        ndt = make_pose(stamp, x=1.0, y=2.0, yaw=math.pi / 2.0)
        seed = make_pose(stamp, x=6.0, y=8.0, yaw=0.0, xy_variance=9.0)

        fused, applied = _fuse_ndt_cross_yaw_seed_along_pose(ndt, seed, along_gain=0.1)

        self.assertTrue(applied)
        self.assertAlmostEqual(fused.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(fused.pose.pose.position.y, 2.6)
        self.assertAlmostEqual(_yaw_from_quaternion(fused.pose.pose.orientation), math.pi / 2.0)

    def test_axis_fuser_no_rtk_mode_caps_along_residual_without_cross_yaw_pollution(self):
        stamp = Time(seconds=1.0)
        ndt = make_pose(stamp, x=10.0, y=2.0, yaw=0.0)
        seed = make_pose(stamp, x=0.0, y=-5.0, yaw=1.2, xy_variance=9.0)

        fused, applied = _fuse_ndt_cross_yaw_seed_along_pose(
            ndt,
            seed,
            along_gain=0.03,
            max_seed_along_residual_m=3.0,
        )

        self.assertTrue(applied)
        self.assertAlmostEqual(fused.pose.pose.position.x, 3.0)
        self.assertAlmostEqual(fused.pose.pose.position.y, 2.0)
        self.assertAlmostEqual(_yaw_from_quaternion(fused.pose.pose.orientation), 0.0)

    def test_correlated_fixposition_noise_is_reproducible_and_sets_covariance(self):
        model_a = CorrelatedNoiseModel(
            seed=123,
            tau_sec=40.0,
            planar_stddev_m=3.0,
            white_stddev_m=0.3,
            yaw_stddev_deg=1.0,
        )
        model_b = CorrelatedNoiseModel(
            seed=123,
            tau_sec=40.0,
            planar_stddev_m=3.0,
            white_stddev_m=0.3,
            yaw_stddev_deg=1.0,
        )

        first_a = model_a.sample(10.0)
        second_a = model_a.sample(10.1)
        first_b = model_b.sample(10.0)
        second_b = model_b.sample(10.1)

        self.assertEqual(first_a, first_b)
        self.assertEqual(second_a, second_b)
        self.assertNotEqual(first_a.xy, second_a.xy)

        odom = Odometry()
        odom.header.stamp = Time(seconds=10.1).to_msg()
        odom.pose.pose.position.x = 100.0
        odom.pose.pose.position.y = 200.0
        odom.pose.pose.orientation = _yaw_to_quaternion(0.0)
        noisy = apply_planar_noise_to_odometry(
            odom,
            second_a,
            reported_xy_sigma_m=3.0,
            reported_z_sigma_m=1.5,
            reported_yaw_sigma_deg=1.0,
        )

        self.assertAlmostEqual(noisy.pose.pose.position.x, 100.0 + second_a.xy[0])
        self.assertAlmostEqual(noisy.pose.pose.position.y, 200.0 + second_a.xy[1])
        self.assertAlmostEqual(noisy.pose.covariance[0], 9.0)
        self.assertAlmostEqual(noisy.pose.covariance[7], 9.0)
        self.assertAlmostEqual(noisy.pose.covariance[14], 2.25)
        self.assertAlmostEqual(noisy.pose.covariance[35], math.radians(1.0) ** 2)

    def test_pointcloud_clock_uses_pointcloud_header_stamp(self):
        msg = PointCloud2()
        msg.header.stamp = Time(seconds=12.34).to_msg()

        clock = clock_from_pointcloud(msg)

        self.assertEqual(clock.clock, msg.header.stamp)

    def test_vehicle_status_clock_uses_velocity_header_stamp(self):
        msg = VelocityReport()
        msg.header.stamp = Time(seconds=12.34).to_msg()

        clock = clock_from_velocity_status(msg)

        self.assertEqual(clock.clock, msg.header.stamp)

    def test_ground_truth_initialpose_clone_preserves_pose_and_sets_topic_frame(self):
        stamp = Time(seconds=4.2)
        gt = make_pose(stamp, x=3.0, y=4.0, yaw=0.7)
        gt.header.frame_id = "world"

        initial = clone_initialpose(gt, frame_id="map")

        self.assertEqual(initial.header.stamp, gt.header.stamp)
        self.assertEqual(initial.header.frame_id, "map")
        self.assertAlmostEqual(initial.pose.pose.position.x, 3.0)
        self.assertAlmostEqual(initial.pose.pose.position.y, 4.0)
        self.assertAlmostEqual(_yaw_from_quaternion(initial.pose.pose.orientation), 0.7)

    def test_ground_truth_initialpose_waits_for_min_stamp(self):
        overrides = [Parameter("min_stamp_sec", Parameter.Type.DOUBLE, 8.0)]
        node = GroundTruthInitialposeOnce(parameter_overrides=overrides)
        try:
            publisher = RecordingPublisher()
            node._publisher = publisher

            node._on_gt(make_pose(Time(seconds=4.0), x=1.0))
            self.assertEqual(len(publisher.messages), 0)

            node._on_gt(make_pose(Time(seconds=8.1), x=2.0))
            self.assertEqual(len(publisher.messages), 1)
            self.assertAlmostEqual(publisher.messages[0].pose.pose.position.x, 2.0)

            node._on_gt(make_pose(Time(seconds=9.0), x=3.0))
            self.assertEqual(len(publisher.messages), 1)
        finally:
            node.destroy_node()

    def test_ground_truth_initialpose_triggers_autoware_localization_once(self):
        node = GroundTruthInitialposeOnce()
        try:
            publisher = RecordingPublisher()
            ekf_client = RecordingSetBoolClient()
            ndt_client = RecordingSetBoolClient()
            node._publisher = publisher
            node._trigger_clients = [ekf_client, ndt_client]

            node._on_gt(make_pose(Time(seconds=2.0), x=2.0))
            node._on_gt(make_pose(Time(seconds=3.0), x=3.0))

            self.assertEqual(len(publisher.messages), 1)
            for client in (ekf_client, ndt_client):
                self.assertEqual(len(client.requests), 1)
                self.assertTrue(client.requests[0].data)
        finally:
            node.destroy_node()

    def test_experiment_nodes_accept_use_sim_time_override(self):
        overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        nodes = [
            PointcloudClockPublisher(parameter_overrides=overrides),
            GroundTruthInitialposeOnce(parameter_overrides=overrides),
            CorrelatedFixpositionNoise(parameter_overrides=overrides),
        ]

        for node in nodes:
            try:
                self.assertTrue(node.get_parameter("use_sim_time").value)
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

    def test_startup_seed_gate_forwards_only_until_first_ndt_lock(self):
        node = FixpositionStartupSeedGate()
        try:
            publisher = RecordingPublisher()
            node._publisher = publisher
            clock_type = node.get_clock().clock_type
            start = Time(seconds=0.0, clock_type=clock_type)

            node._on_seed(make_pose(start, x=1.0))
            self.assertEqual(len(publisher.messages), 1)
            self.assertEqual(node._forwarded_count, 1)
            self.assertFalse(node._locked)

            node._on_lock(make_pose(Time(seconds=0.1, clock_type=clock_type), x=2.0))
            node._on_seed(make_pose(Time(seconds=0.2, clock_type=clock_type), x=3.0))

            self.assertEqual(len(publisher.messages), 1)
            self.assertEqual(node._suppressed_count, 1)
            self.assertTrue(node._locked)
            self.assertIsNotNone(node._first_lock_stamp)
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

    def test_ndt_initial_consistency_gate_rejects_large_yaw_correction(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, y=0.0, yaw=0.0)
        ndt = make_pose(stamp, x=1.0, y=0.0, yaw=math.radians(6.0))

        ok, details = _ndt_is_consistent_with_initial_pose(
            ndt,
            initial,
            max_distance_m=3.0,
            max_yaw_delta_deg=5.0,
        )

        self.assertFalse(ok)
        self.assertEqual(details["reason"], "yaw_delta")
        self.assertAlmostEqual(details["yaw_delta_deg"], 6.0)

    def test_ndt_initial_consistency_gate_accepts_small_correction(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, y=0.0, yaw=0.0)
        ndt = make_pose(stamp, x=1.0, y=0.0, yaw=math.radians(2.0))

        ok, details = _ndt_is_consistent_with_initial_pose(
            ndt,
            initial,
            max_distance_m=3.0,
            max_yaw_delta_deg=5.0,
        )

        self.assertTrue(ok)
        self.assertEqual(details["reason"], "ok")

    def test_fuser_uses_time_matched_initial_pose_for_delayed_ndt_result(self):
        node = NdtAxisSeedFuser()
        try:
            clock_type = node.get_clock().clock_type
            ndt_stamp = Time(seconds=1.0, clock_type=clock_type)
            future_stamp = Time(seconds=1.15, clock_type=clock_type)
            node._max_initial_pose_age = 0.2
            node._max_ndt_initial_distance = 0.5

            node._on_initial_pose(make_pose(ndt_stamp, x=1.0, y=0.0))
            node._on_initial_pose(make_pose(future_stamp, x=5.0, y=0.0))

            self.assertTrue(
                node._ndt_passes_initial_consistency(
                    make_pose(ndt_stamp, x=1.1, y=0.0),
                    ndt_stamp,
                )
            )
        finally:
            node.destroy_node()

    def test_fuser_uses_time_matched_seed_when_latest_seed_is_future_of_ndt(self):
        node = NdtAxisSeedFuser()
        try:
            node._publisher = RecordingPublisher()
            node._predictor_update_publisher = RecordingPublisher()
            node._enable_robust_initial_update = False
            node._fusion_mode = "ndt_cross_yaw_seed_along"
            node._along_gain = 0.03
            node._max_seed_along_residual = 3.0
            node._max_seed_age = 0.5
            node._max_seed_xy_stddev = 5.0
            clock_type = node.get_clock().clock_type
            ndt_stamp = Time(seconds=1.0, clock_type=clock_type)
            future_stamp = Time(seconds=3.0, clock_type=clock_type)

            node._on_seed(make_pose(ndt_stamp, x=10.0, y=0.0, yaw=0.0, xy_variance=9.0))
            node._on_seed(make_pose(future_stamp, x=30.0, y=0.0, yaw=0.0, xy_variance=9.0))
            node._on_ndt(make_pose(ndt_stamp, x=0.0, y=0.0, yaw=0.0))

            self.assertEqual(len(node._publisher.messages), 1)
            fused = node._publisher.messages[-1]
            self.assertAlmostEqual(fused.pose.pose.position.x, 7.0)
            self.assertAlmostEqual(fused.pose.pose.position.y, 0.0)
        finally:
            node.destroy_node()

    def test_initial_pose_correction_gain_applies_fractional_ndt_update(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=10.0, y=0.0, roll=0.1, pitch=-0.2, yaw=0.0)
        ndt = make_pose(
            stamp,
            x=14.0,
            y=2.0,
            roll=0.5,
            pitch=0.2,
            yaw=math.radians(10.0),
        )

        fused = _apply_initial_pose_correction_gain(ndt, initial, correction_gain=0.25)
        roll, pitch, yaw = _rpy_from_quaternion(fused.pose.pose.orientation)

        self.assertAlmostEqual(fused.pose.pose.position.x, 11.0)
        self.assertAlmostEqual(fused.pose.pose.position.y, 0.5)
        self.assertAlmostEqual(roll, 0.2)
        self.assertAlmostEqual(pitch, -0.1)
        self.assertAlmostEqual(yaw, math.radians(2.5))

    def test_initial_pose_axis_correction_gain_can_dampen_along_without_cross_yaw(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, y=0.0, yaw=math.pi / 2.0)
        ndt = make_pose(stamp, x=-2.0, y=10.0, yaw=math.radians(100.0))

        fused = _apply_initial_pose_axis_correction_gain(
            ndt,
            initial,
            along_gain=0.2,
            cross_gain=1.0,
            yaw_gain=1.0,
        )

        self.assertAlmostEqual(fused.pose.pose.position.x, -2.0, places=6)
        self.assertAlmostEqual(fused.pose.pose.position.y, 2.0, places=6)
        self.assertAlmostEqual(
            _yaw_from_quaternion(fused.pose.pose.orientation),
            math.radians(100.0),
        )

    def test_robust_initial_pose_update_clips_large_ndt_innovation_by_axis(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=10.0, y=0.0, yaw=0.0)
        ndt = make_pose(stamp, x=12.0, y=1.0, yaw=math.radians(10.0))

        fused, decision = _robust_initial_pose_update(
            ndt,
            initial,
            max_along_correction_m=0.3,
            max_cross_correction_m=0.2,
            max_yaw_correction_deg=2.0,
            hard_reject_correction_m=0.0,
            hard_reject_yaw_deg=0.0,
        )

        self.assertTrue(decision["accepted"])
        self.assertTrue(decision["clipped"])
        self.assertAlmostEqual(fused.pose.pose.position.x, 10.3)
        self.assertAlmostEqual(fused.pose.pose.position.y, 0.2)
        self.assertAlmostEqual(
            _yaw_from_quaternion(fused.pose.pose.orientation),
            math.radians(2.0),
        )
        self.assertAlmostEqual(decision["innovation_along_m"], 2.0)
        self.assertAlmostEqual(decision["applied_along_m"], 0.3)

    def test_ekf_initial_pose_update_accepts_covariance_weighted_measurement(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=10.0, y=0.0, yaw=0.0, xy_variance=1.0, yaw_variance=0.04)
        ndt = make_pose(
            stamp,
            x=10.4,
            y=0.2,
            yaw=math.radians(2.0),
            xy_variance=0.25,
            yaw_variance=math.radians(1.0) ** 2,
        )

        fused, decision = _ekf_initial_pose_update(
            ndt,
            initial,
            mahalanobis_gate=5.0,
            ndt_covariance_estimation_type=1,
            process_noise_diag=[0.2, 0.2, math.radians(2.0) ** 2],
        )

        self.assertTrue(decision["accepted"])
        self.assertTrue(decision["gate_enabled"])
        self.assertEqual(decision["ndt_covariance_source"], "laplace")
        self.assertLess(decision["mahalanobis"], 5.0)
        self.assertGreater(fused.pose.pose.position.x, 10.0)
        self.assertLess(fused.pose.pose.position.x, 10.4)
        self.assertLess(fused.pose.covariance[0], initial.pose.covariance[0] + 0.2)
        self.assertLess(fused.pose.covariance[0], 0.25)

    def test_ekf_initial_pose_update_supports_axis_specific_measurement_floors(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, y=0.0, yaw=0.0, xy_variance=0.25, yaw_variance=0.04)
        ndt = make_pose(
            stamp,
            x=1.0,
            y=1.0,
            yaw=0.0,
            xy_variance=0.01,
            yaw_variance=math.radians(1.0) ** 2,
        )

        fused, decision = _ekf_initial_pose_update(
            ndt,
            initial,
            mahalanobis_gate=10.0,
            ndt_covariance_estimation_type=1,
            process_noise_diag=[0.0, 0.0, 0.0],
            measurement_xy_variance_floor_m2=1.0,
            measurement_along_variance_floor_m2=0.05,
            measurement_cross_variance_floor_m2=1.0,
        )

        self.assertTrue(decision["accepted"])
        self.assertGreater(decision["kalman_gain_diag"][0], decision["kalman_gain_diag"][1])
        self.assertGreater(fused.pose.pose.position.x, 0.7)
        self.assertLess(fused.pose.pose.position.y, 0.3)

    def test_ekf_initial_pose_update_inflates_along_measurement_from_candidate_spread(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, y=0.0, yaw=0.0, xy_variance=0.25, yaw_variance=0.04)
        ndt = make_pose(
            stamp,
            x=1.0,
            y=0.0,
            yaw=0.0,
            xy_variance=0.01,
            yaw_variance=math.radians(1.0) ** 2,
        )

        fused, decision = _ekf_initial_pose_update(
            ndt,
            initial,
            mahalanobis_gate=10.0,
            ndt_covariance_estimation_type=1,
            process_noise_diag=[0.0, 0.0, 0.0],
            measurement_xy_variance_floor_m2=0.09,
            measurement_variance_inflation_diag=[4.0, 0.0, 0.0],
        )

        self.assertTrue(decision["accepted"])
        self.assertEqual(decision["measurement_variance_inflation_diag"], [4.0, 0.0, 0.0])
        self.assertGreater(decision["measurement_variance_diag"][0], 4.0)
        self.assertLess(decision["kalman_gain_diag"][0], 0.1)
        self.assertLess(fused.pose.pose.position.x, 0.1)

    def test_runtime_candidate_spread_variance_inflation_uses_bounded_converged_candidates(self):
        decision = {
            "candidate_count": 4,
            "candidates": [
                {
                    "converged": True,
                    "reject_reason": "",
                    "innovation_along_m": -1.0,
                    "innovation_cross_m": 0.1,
                    "innovation_yaw_deg": 0.2,
                },
                {
                    "converged": True,
                    "reject_reason": "",
                    "innovation_along_m": 2.0,
                    "innovation_cross_m": -0.1,
                    "innovation_yaw_deg": 0.4,
                },
                {
                    "converged": False,
                    "reject_reason": "not_converged",
                    "innovation_along_m": 20.0,
                    "innovation_cross_m": 0.0,
                    "innovation_yaw_deg": 0.0,
                },
                {
                    "converged": True,
                    "reject_reason": "",
                    "innovation_along_m": 8.0,
                    "innovation_cross_m": 0.2,
                    "innovation_yaw_deg": 0.0,
                },
            ],
        }

        inflation, metadata = _runtime_candidate_spread_variance_inflation(
            decision,
            min_candidate_count=2,
            along_spread_threshold_m=1.5,
            along_variance_scale=0.5,
            max_abs_along_m=3.0,
            max_abs_cross_m=0.85,
            yaw_spread_threshold_deg=3.0,
            yaw_variance_scale=0.5,
        )

        self.assertGreater(inflation[0], 2.0)
        self.assertEqual(inflation[1], 0.0)
        self.assertEqual(inflation[2], 0.0)
        self.assertEqual(metadata["runtime_candidate_spread_count"], 2)
        self.assertAlmostEqual(metadata["runtime_candidate_along_spread_m"], 3.0)

    def test_runtime_candidate_spread_variance_inflation_uses_selected_score_neighborhood(self):
        decision = {
            "candidate_count": 3,
            "selected_candidate_index": 0,
            "candidates": [
                {
                    "index": 0,
                    "converged": True,
                    "reject_reason": "",
                    "total_score": 3.50,
                    "innovation_along_m": -1.10,
                    "innovation_cross_m": 0.05,
                    "innovation_yaw_deg": 0.1,
                },
                {
                    "index": 1,
                    "converged": True,
                    "reject_reason": "",
                    "total_score": 3.35,
                    "innovation_along_m": -0.95,
                    "innovation_cross_m": 0.02,
                    "innovation_yaw_deg": 0.2,
                },
                {
                    "index": 2,
                    "converged": True,
                    "reject_reason": "",
                    "total_score": 3.05,
                    "innovation_along_m": 1.30,
                    "innovation_cross_m": 0.10,
                    "innovation_yaw_deg": 0.3,
                },
            ],
        }

        inflation, metadata = _runtime_candidate_spread_variance_inflation(
            decision,
            min_candidate_count=2,
            along_spread_threshold_m=1.5,
            along_variance_scale=0.5,
            max_abs_along_m=3.0,
            max_abs_cross_m=0.85,
            selected_score_margin=0.30,
            yaw_spread_threshold_deg=3.0,
            yaw_variance_scale=0.5,
        )

        self.assertEqual(inflation, [0.0, 0.0, 0.0])
        self.assertEqual(metadata["runtime_candidate_spread_count"], 2)
        self.assertAlmostEqual(metadata["runtime_candidate_along_spread_m"], 0.15)

    def test_ekf_initial_pose_update_rejects_high_mahalanobis_wrong_basin(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, y=0.0, yaw=0.0, xy_variance=0.25, yaw_variance=0.01)
        ndt = make_pose(stamp, x=8.0, y=2.0, yaw=math.radians(20.0), xy_variance=0.09, yaw_variance=0.0025)

        fused, decision = _ekf_initial_pose_update(
            ndt,
            initial,
            mahalanobis_gate=3.0,
            ndt_covariance_estimation_type=1,
            process_noise_diag=[0.05, 0.05, 0.0025],
        )

        self.assertFalse(decision["accepted"])
        self.assertEqual(decision["reason"], "mahalanobis_reject")
        self.assertGreater(decision["mahalanobis"], 3.0)
        self.assertAlmostEqual(fused.pose.pose.position.x, 0.0)
        self.assertAlmostEqual(fused.pose.pose.position.y, 0.0)

    def test_fuser_rejects_unverified_far_tier_recovery_candidate(self):
        node = NdtAxisSeedFuser()
        try:
            node._decision_publisher = RecordingPublisher()
            node._enable_robust_initial_update = True
            node._robust_update_mode = "ekf"
            node._robust_mahalanobis_gate = 10.0
            node._runtime_multistart_decision_max_age = 0.2
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=41.6, clock_type=clock_type)
            initial = make_pose(
                stamp,
                x=10.0,
                y=0.0,
                yaw=0.0,
                xy_variance=0.25,
                yaw_variance=0.01,
            )
            ndt = make_pose(
                stamp,
                x=7.4,
                y=0.18,
                yaw=math.radians(-0.1),
                xy_variance=0.09,
                yaw_variance=0.0025,
            )
            node._store_initial_pose(initial, stamp)
            node._last_runtime_multistart_decision = {
                "stamp_sec": 41.6,
                "candidate_count": 17,
                "tier2_evaluated": True,
                "recovery_active": False,
                "recovery_verified": False,
                "selected_candidate_index": 6,
                "candidates": [
                    {"index": 0, "tier": "small"},
                    {"index": 6, "tier": "far"},
                ],
            }

            fused, accepted = node._apply_robust_initial_update_if_available(ndt, stamp)

            self.assertFalse(accepted)
            self.assertAlmostEqual(fused.pose.pose.position.x, 10.0)
            self.assertAlmostEqual(fused.pose.pose.position.y, 0.0)
            decision = json.loads(node._decision_publisher.messages[-1].data)
            self.assertFalse(decision["accepted"])
            self.assertEqual(decision["reason"], "unverified_runtime_recovery_candidate")
            self.assertEqual(decision["runtime_selected_candidate_tier"], "far")
        finally:
            node.destroy_node()

    def test_fuser_accepts_verified_far_tier_recovery_as_predictor_reset(self):
        node = NdtAxisSeedFuser()
        try:
            node._publisher = RecordingPublisher()
            node._predictor_update_publisher = RecordingPublisher()
            node._decision_publisher = RecordingPublisher()
            node._enable_robust_initial_update = True
            node._robust_update_mode = "ekf"
            node._robust_mahalanobis_gate = 4.0
            node._runtime_multistart_decision_max_age = 0.2
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=145.0, clock_type=clock_type)
            initial = make_pose(
                stamp,
                x=0.0,
                y=0.0,
                yaw=0.0,
                xy_variance=0.25,
                yaw_variance=0.01,
            )
            ndt = make_pose(
                stamp,
                x=12.0,
                y=-14.0,
                yaw=math.radians(0.5),
                xy_variance=0.09,
                yaw_variance=0.0025,
            )
            node._store_initial_pose(initial, stamp)
            node._last_runtime_multistart_decision = {
                "stamp_sec": 145.0,
                "candidate_count": 21,
                "tier2_evaluated": True,
                "recovery_active": False,
                "recovery_verified": True,
                "recovery_verified_stable_frames": 3,
                "selected_candidate_index": 17,
                "candidates": [
                    {"index": 0, "tier": "small"},
                    {"index": 17, "tier": "far"},
                ],
            }

            node._on_ndt(ndt)

            self.assertEqual(len(node._publisher.messages), 1)
            self.assertEqual(len(node._predictor_update_publisher.messages), 1)
            decision = json.loads(node._decision_publisher.messages[-1].data)
            self.assertTrue(decision["accepted"])
            self.assertEqual(decision["reason"], "runtime_verified_recovery_reset")
            self.assertTrue(decision["predictor_update_allowed"])
            predictor_msg = node._predictor_update_publisher.messages[-1]
            self.assertAlmostEqual(predictor_msg.pose.pose.position.x, 12.0)
            self.assertAlmostEqual(predictor_msg.pose.pose.position.y, -14.0)
        finally:
            node.destroy_node()

    def test_fuser_suppresses_predictor_update_for_low_confidence_robust_accept(self):
        node = NdtAxisSeedFuser()
        try:
            node._publisher = RecordingPublisher()
            node._predictor_update_publisher = RecordingPublisher()
            node._decision_publisher = RecordingPublisher()
            node._enable_robust_initial_update = True
            node._robust_update_mode = "ekf"
            node._robust_mahalanobis_gate = 10.0
            node._predictor_update_requires_robust_high_confidence = True
            node._predictor_update_max_mahalanobis = 2.0
            node._predictor_update_max_innovation_along = 0.8
            node._predictor_update_max_innovation_cross = 0.25
            node._predictor_update_max_innovation_yaw_deg = 2.0
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=65.6, clock_type=clock_type)
            initial = make_pose(
                stamp,
                x=0.0,
                y=0.0,
                yaw=0.0,
                xy_variance=0.25,
                yaw_variance=0.01,
            )
            ndt = make_pose(
                stamp,
                x=0.2,
                y=0.45,
                yaw=0.0,
                xy_variance=0.09,
                yaw_variance=0.0025,
            )
            node._store_initial_pose(initial, stamp)

            node._on_ndt(ndt)

            self.assertEqual(len(node._publisher.messages), 1)
            self.assertEqual(len(node._predictor_update_publisher.messages), 0)
            decision = json.loads(node._decision_publisher.messages[-1].data)
            self.assertTrue(decision["accepted"])
            self.assertFalse(decision["predictor_update_allowed"])
            self.assertEqual(decision["predictor_update_suppressed_reason"], "robust_cross")
        finally:
            node.destroy_node()

    def test_fuser_allows_low_confidence_predictor_update_during_startup_grace(self):
        node = NdtAxisSeedFuser()
        try:
            node._publisher = RecordingPublisher()
            node._predictor_update_publisher = RecordingPublisher()
            node._decision_publisher = RecordingPublisher()
            node._enable_robust_initial_update = True
            node._robust_update_mode = "ekf"
            node._robust_mahalanobis_gate = 10.0
            node._predictor_update_requires_robust_high_confidence = True
            node._predictor_update_high_confidence_min_stamp_sec = 45.0
            node._predictor_update_max_innovation_cross = 0.25
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=20.0, clock_type=clock_type)
            initial = make_pose(
                stamp,
                x=0.0,
                y=0.0,
                yaw=0.0,
                xy_variance=0.25,
                yaw_variance=0.01,
            )
            ndt = make_pose(
                stamp,
                x=0.2,
                y=0.45,
                yaw=0.0,
                xy_variance=0.09,
                yaw_variance=0.0025,
            )
            node._store_initial_pose(initial, stamp)

            node._on_ndt(ndt)

            self.assertEqual(len(node._publisher.messages), 1)
            self.assertEqual(len(node._predictor_update_publisher.messages), 1)
            decision = json.loads(node._decision_publisher.messages[-1].data)
            self.assertTrue(decision["accepted"])
            self.assertTrue(decision["predictor_update_allowed"])
            self.assertEqual(decision["predictor_update_suppressed_reason"], "")
        finally:
            node.destroy_node()

    def test_fuser_writes_predictor_update_for_high_confidence_robust_accept(self):
        node = NdtAxisSeedFuser()
        try:
            node._publisher = RecordingPublisher()
            node._predictor_update_publisher = RecordingPublisher()
            node._decision_publisher = RecordingPublisher()
            node._enable_robust_initial_update = True
            node._robust_update_mode = "ekf"
            node._robust_mahalanobis_gate = 10.0
            node._predictor_update_requires_robust_high_confidence = True
            node._predictor_update_max_mahalanobis = 2.0
            node._predictor_update_max_innovation_along = 0.8
            node._predictor_update_max_innovation_cross = 0.25
            node._predictor_update_max_innovation_yaw_deg = 2.0
            clock_type = node.get_clock().clock_type
            stamp = Time(seconds=65.8, clock_type=clock_type)
            initial = make_pose(
                stamp,
                x=0.0,
                y=0.0,
                yaw=0.0,
                xy_variance=0.25,
                yaw_variance=0.01,
            )
            ndt = make_pose(
                stamp,
                x=0.2,
                y=0.05,
                yaw=math.radians(0.2),
                xy_variance=0.09,
                yaw_variance=0.0025,
            )
            node._store_initial_pose(initial, stamp)

            node._on_ndt(ndt)

            self.assertEqual(len(node._publisher.messages), 1)
            self.assertEqual(len(node._predictor_update_publisher.messages), 1)
            decision = json.loads(node._decision_publisher.messages[-1].data)
            self.assertTrue(decision["accepted"])
            self.assertTrue(decision["predictor_update_allowed"])
            self.assertEqual(decision["predictor_update_suppressed_reason"], "")
        finally:
            node.destroy_node()

    def test_body_frame_position_bias_shifts_pose_along_current_yaw(self):
        msg = make_pose(Time(seconds=1.0), x=10.0, y=20.0, yaw=math.pi / 2.0)

        shifted = _apply_body_frame_position_bias(msg, along_bias_m=2.0, cross_bias_m=0.5)

        self.assertAlmostEqual(shifted.pose.pose.position.x, 9.5)
        self.assertAlmostEqual(shifted.pose.pose.position.y, 22.0)
        self.assertAlmostEqual(msg.pose.pose.position.x, 10.0)
        self.assertAlmostEqual(msg.pose.pose.position.y, 20.0)

    def test_output_position_bias_does_not_feed_back_to_predictor_update(self):
        node = NdtAxisSeedFuser()
        try:
            node._publisher = RecordingPublisher()
            node._predictor_update_publisher = RecordingPublisher()
            node._output_along_bias_m = 2.0
            node._output_cross_bias_m = 0.0
            stamp = Time(seconds=2.0)
            msg = make_pose(stamp, x=10.0, y=20.0, yaw=0.0)

            node._publish_final_pose(msg, stamp, predictor_update=True)

            public_msg = node._publisher.messages[-1]
            predictor_msg = node._predictor_update_publisher.messages[-1]
            self.assertAlmostEqual(public_msg.pose.pose.position.x, 12.0)
            self.assertAlmostEqual(public_msg.pose.pose.position.y, 20.0)
            self.assertAlmostEqual(predictor_msg.pose.pose.position.x, 10.0)
            self.assertAlmostEqual(predictor_msg.pose.pose.position.y, 20.0)
        finally:
            node.destroy_node()

    def test_robust_initial_pose_update_rejects_extreme_wrong_basin(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, y=0.0, yaw=0.0)
        ndt = make_pose(stamp, x=20.0, y=0.0, yaw=math.radians(30.0))

        fused, decision = _robust_initial_pose_update(
            ndt,
            initial,
            max_along_correction_m=0.3,
            max_cross_correction_m=0.2,
            max_yaw_correction_deg=2.0,
            hard_reject_correction_m=8.0,
            hard_reject_yaw_deg=20.0,
        )

        self.assertFalse(decision["accepted"])
        self.assertEqual(decision["reason"], "hard_reject")
        self.assertAlmostEqual(fused.pose.pose.position.x, 0.0)
        self.assertAlmostEqual(fused.pose.pose.position.y, 0.0)

    def test_robust_initial_pose_update_applies_axis_gains_before_clipping(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=10.0, y=0.0, yaw=0.0)
        ndt = make_pose(stamp, x=11.0, y=0.5, yaw=math.radians(10.0))

        fused, decision = _robust_initial_pose_update(
            ndt,
            initial,
            along_gain=0.2,
            cross_gain=0.4,
            yaw_gain=0.1,
            max_along_correction_m=1.0,
            max_cross_correction_m=1.0,
            max_yaw_correction_deg=5.0,
        )

        self.assertFalse(decision["clipped"])
        self.assertAlmostEqual(fused.pose.pose.position.x, 10.2)
        self.assertAlmostEqual(fused.pose.pose.position.y, 0.2)
        self.assertAlmostEqual(
            _yaw_from_quaternion(fused.pose.pose.orientation),
            math.radians(1.0),
        )

    def test_robust_initial_pose_update_preserves_vertical_and_attitude_tracking(self):
        stamp = Time(seconds=20.6)
        initial = make_pose(stamp, x=0.0, z=100.0, roll=0.1, pitch=0.2, yaw=0.0)
        ndt = make_pose(stamp, x=0.0, z=104.0, roll=0.3, pitch=-0.2, yaw=0.0)

        fused, _ = _robust_initial_pose_update(
            ndt,
            initial,
            z_gain=0.5,
            roll_pitch_gain=0.5,
        )
        roll, pitch, _ = _rpy_from_quaternion(fused.pose.pose.orientation)

        self.assertAlmostEqual(fused.pose.pose.position.z, 102.0)
        self.assertAlmostEqual(roll, 0.2)
        self.assertAlmostEqual(pitch, 0.0)

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
