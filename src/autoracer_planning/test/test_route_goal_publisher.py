import math

import yaml

from autoracer_planning.route_goal_publisher import goal_pose_from_yaml


def test_goal_pose_from_yaml_uses_route_goal_frame_position_and_yaw(tmp_path):
    route_goal = {
        "frame_id": "map",
        "goal": {
            "position": {"x": 10.0, "y": 2.5, "z": 0.0},
            "yaw": math.pi / 2.0,
        },
    }
    path = tmp_path / "route_goal.yaml"
    path.write_text(yaml.safe_dump(route_goal), encoding="utf-8")

    pose = goal_pose_from_yaml(path)

    assert pose.header.frame_id == "map"
    assert pose.pose.position.x == 10.0
    assert pose.pose.position.y == 2.5
    assert pose.pose.position.z == 0.0
    assert round(pose.pose.orientation.z, 6) == round(math.sin(math.pi / 4.0), 6)
    assert round(pose.pose.orientation.w, 6) == round(math.cos(math.pi / 4.0), 6)

