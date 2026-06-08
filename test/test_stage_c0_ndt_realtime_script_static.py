from pathlib import Path


SCRIPT = Path(
    "/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/"
    "run_stage_c0_ndt_realtime.sh"
)


def test_stage_c0_script_defaults_to_open_loop_collection_testrun():
    source = SCRIPT.read_text(encoding="utf-8")

    assert 'TESTRUN="${TESTRUN:-AutoracerCollection_UrbanRoad}"' in source
    assert 'TACCEL="${TACCEL:-1}"' in source
    assert 'TSTOP="${TSTOP:-520}"' in source
    assert "carmaker_stage_c0_ndt.launch.py" in source
    assert "carmaker_stage_b_ndt.launch.py" not in source
    assert "MAX_SPEED_MPS" not in source
    assert "PLANNING_MAP_PATH" not in source


def test_stage_c0_script_never_waits_for_or_records_control_nodes():
    source = SCRIPT.read_text(encoding="utf-8")

    forbidden = [
        "/route_goal_publisher",
        "/lanelet_route_planner",
        "/local_trajectory_planner",
        "/pure_pursuit_controller",
        "/command_gate",
        "/control/command/control_cmd",
        "/planning/trajectory",
        "/planning/global_trajectory",
    ]
    for token in forbidden:
        assert token not in source


def test_stage_c0_script_records_localization_and_gt_topics():
    source = SCRIPT.read_text(encoding="utf-8")

    required = [
        "/localization/pose_with_covariance",
        "/localization/ndt/raw_pose_with_covariance",
        "/localization/fixposition/seed_pose",
        "/carmaker/ground_truth/pose",
        "/sensing/lidar/concatenated/pointcloud",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/fixposition/odometry_enu",
        "/fixposition/fpa/odomstatus",
    ]
    for topic in required:
        assert topic in source

    assert '"stage": "stage_c0_ndt_realtime"' in source
    assert "movie_restart_count" in source
    assert "seed_covariance_xy" in source
