from geometry_msgs.msg import PoseWithCovarianceStamped

from autoracer_carmaker_sim.ground_truth_localization_relay import clone_pose


def test_clone_pose_preserves_ground_truth_pose_and_updates_frame():
    source = PoseWithCovarianceStamped()
    source.header.stamp.sec = 12
    source.header.stamp.nanosec = 345
    source.header.frame_id = "carmaker_map"
    source.pose.pose.position.x = 1.25
    source.pose.pose.position.y = -2.5
    source.pose.pose.orientation.w = 1.0
    source.pose.covariance[0] = 0.42

    cloned = clone_pose(source, "map")

    assert cloned is not source
    assert cloned.header.stamp.sec == 12
    assert cloned.header.stamp.nanosec == 345
    assert cloned.header.frame_id == "map"
    assert cloned.pose.pose.position.x == 1.25
    assert cloned.pose.pose.position.y == -2.5
    assert cloned.pose.pose.orientation.w == 1.0
    assert cloned.pose.covariance[0] == 0.42
