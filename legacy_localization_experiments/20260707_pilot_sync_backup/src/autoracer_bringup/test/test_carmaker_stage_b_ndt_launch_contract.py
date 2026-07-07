from pathlib import Path


def test_stage_b_ndt_launch_uses_replay_verified_localization_graph():
    launch_file = Path("src/autoracer_bringup/launch/carmaker_stage_b_ndt.launch.py")
    text = launch_file.read_text(encoding="utf-8")

    assert "ground_truth_localization_relay" not in text
    assert "autoware_gnss_poser" not in text
    assert "map_projection_loader" not in text
    assert "fixposition_odom_to_seed_pose" in text
    assert "fixposition_seed_filter" in text
    assert "ndt_initial_pose_predictor" in text
    assert "pickup_based_voxel_grid_downsample_filter_node" in text
    assert '"voxel_grid_downsample_filter_node"' not in text
    assert '"voxel_size_x": 1.5' in text
    assert "/sensing/lidar/concatenated/pointcloud_downsampled" in text
    assert "ndt_axis_seed_fuser" in text
    assert "/localization/ndt/raw_pose_with_covariance" in text
    assert '"ndt.num_threads": 32' in text
    assert '"dynamic_map_loading.map_radius": 90.0' in text
    assert '"dynamic_map_loading.lidar_radius": 50.0' in text


def test_stage_b_ndt_launch_keeps_planning_and_localization_maps_separate():
    text = Path("src/autoracer_bringup/launch/carmaker_stage_b_ndt.launch.py").read_text(
        encoding="utf-8"
    )

    assert "DeclareLaunchArgument" in text
    assert '"localization_map_path"' in text
    assert '"planning_map_path"' in text
    assert 'PathJoinSubstitution([planning_map_path, "lanelet2_map.osm"])' in text
    assert "pointcloud_map_metadata.yaml" in text
    assert "pcd_metadata_path" in text
