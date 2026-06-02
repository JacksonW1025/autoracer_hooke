from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from autoracer_carmaker_sim.trajectory_provider import build_trajectory


def _path(points):
    msg = Path()
    msg.header.frame_id = "map"
    for x, y, z in points:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def test_build_trajectory_preserves_carmaker_road_eval_points():
    trajectory = build_trajectory(_path([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]), 1.5)

    assert trajectory.header.frame_id == "map"
    assert len(trajectory.points) == 2
    assert trajectory.points[0].pose.position.x == 1.0
    assert trajectory.points[0].pose.position.y == 2.0
    assert trajectory.points[0].longitudinal_velocity_mps == 1.5
    assert trajectory.points[-1].longitudinal_velocity_mps == 1.5


def test_build_trajectory_rejects_empty_centerline():
    try:
        build_trajectory(_path([]), 1.5)
    except ValueError as exc:
        assert "centerline path has fewer than two poses" in str(exc)
    else:
        raise AssertionError("expected ValueError")
