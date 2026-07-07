from geometry_msgs.msg import PoseWithCovarianceStamped

from autoracer_localization.gnss5m_weak_pose_bridge import (
    choose_bridge_source,
    clamp_gnss_covariance,
)


def test_clamp_gnss_covariance_never_understates_xy_or_yaw():
    msg = PoseWithCovarianceStamped()
    msg.pose.covariance[0] = 0.3
    msg.pose.covariance[7] = 9.0
    msg.pose.covariance[35] = 0.01

    out = clamp_gnss_covariance(msg, min_xy_variance=25.0, yaw_variance=999.0)

    assert out.pose.covariance[0] == 25.0
    assert out.pose.covariance[7] == 25.0
    assert out.pose.covariance[35] == 999.0


def test_clamp_gnss_covariance_preserves_larger_covariance():
    msg = PoseWithCovarianceStamped()
    msg.pose.covariance[0] = 36.0
    msg.pose.covariance[7] = 49.0
    msg.pose.covariance[35] = 1000.0

    out = clamp_gnss_covariance(msg, min_xy_variance=25.0, yaw_variance=999.0)

    assert out.pose.covariance[0] == 36.0
    assert out.pose.covariance[7] == 49.0
    assert out.pose.covariance[35] == 1000.0


def test_choose_bridge_source_prefers_recent_ndt():
    decision = choose_bridge_source(now_sec=10.0, last_ndt_sec=9.6, ndt_gap_threshold_sec=0.7)

    assert decision.source == "ndt"
    assert decision.reason == "ndt_recent"


def test_choose_bridge_source_uses_gnss_when_ndt_gap_is_large():
    decision = choose_bridge_source(now_sec=10.0, last_ndt_sec=8.0, ndt_gap_threshold_sec=0.7)

    assert decision.source == "gnss"
    assert decision.reason == "ndt_gap"


def test_choose_bridge_source_uses_gnss_before_first_ndt():
    decision = choose_bridge_source(now_sec=10.0, last_ndt_sec=None, ndt_gap_threshold_sec=0.7)

    assert decision.source == "gnss"
    assert decision.reason == "no_ndt_yet"
