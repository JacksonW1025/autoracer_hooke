from pathlib import Path


LAUNCH_FILE = Path("src/autoracer_bringup/launch/carmaker_stage_c0_ndt.launch.py")
STARTUP_ONLY_LAUNCH_FILE = Path(
    "src/autoracer_bringup/launch/carmaker_stage_c0_ndt_startup_only.launch.py"
)


def test_stage_c0_ndt_launch_is_localization_only():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    forbidden = [
        "ground_truth_localization_relay",
        "route_goal_publisher",
        "lanelet_route_planner",
        "local_trajectory_planner",
        "pure_pursuit_controller",
        "command_gate",
        "/control/command/control_cmd",
        "/planning/trajectory",
    ]
    for token in forbidden:
        assert token not in text


def test_stage_c0_ndt_launch_uses_replay_verified_localization_graph():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "autoware_gnss_poser" not in text
    assert "map_projection_loader" not in text
    assert "static_tf.launch.py" in text
    assert "autoware_pointcloud_map_loader" in text
    assert "fixposition_odom_to_seed_pose" in text
    assert '"reported_xy_sigma_m": 0.1' in text
    assert "fixposition_seed_filter" in text
    assert "ndt_initial_pose_predictor" in text
    assert "pickup_based_voxel_grid_downsample_filter_node" in text
    assert '"voxel_size_x": 1.5' in text
    assert "/sensing/lidar/concatenated/pointcloud_downsampled" in text
    assert "autoware_ndt_scan_matcher_node" in text
    assert '"ndt.num_threads": 32' in text
    assert '"ndt.max_iterations": 60' in text
    assert '"dynamic_map_loading.map_radius": 90.0' in text
    assert '"dynamic_map_loading.lidar_radius": 50.0' in text
    assert "ndt_axis_seed_fuser" in text
    assert "/localization/ndt/raw_pose_with_covariance" in text
    assert '"/localization/pose_with_covariance"' in text
    assert "ndt_startup_helper" in text
    assert "pose_tf_broadcaster" in text


def test_stage_c0_ndt_launch_keeps_quality_gate_and_matches_iteration_budget():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"min_nvtl_score": 2.3' in text
    assert '"max_iteration_num": 60' in text


def test_stage_c0_ndt_regularization_keeps_replay_verified_seed_source():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert (
        '("regularization_pose_with_covariance", "/localization/fixposition/seed_pose")'
        in text
    )


def test_stage_c0_ndt_launch_uses_tile20_only_for_localization_map():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"localization_map_path"' in text
    assert "pointcloud_map_metadata.yaml" in text
    assert "pcd_metadata_path" in text
    assert "lanelet2_map.osm" not in text
    assert "map_projector_info.yaml" not in text


def test_stage_c0_startup_only_launch_gates_fixposition_after_first_ndt_lock():
    aided_text = LAUNCH_FILE.read_text(encoding="utf-8")
    text = STARTUP_ONLY_LAUNCH_FILE.read_text(encoding="utf-8")

    assert "fixposition_startup_seed_gate" not in aided_text
    assert "fixposition_startup_seed_gate" in text
    assert "/localization/fixposition/startup_only_seed_pose" in text
    assert '"seed_pose_topic": "/localization/fixposition/startup_only_seed_pose"' in text
    assert (
        '("regularization_pose_with_covariance", "/localization/fixposition/startup_only_seed_pose")'
        in text
    )
    assert '"enable_tracking_seed_fusion": False' in text
    assert '"output_topic": "/localization/pose_with_covariance"' in text
    assert "route_goal_publisher" not in text
    assert "pure_pursuit_controller" not in text
    assert "command_gate" not in text
