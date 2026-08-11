from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RACE_LAUNCH = PACKAGE_ROOT / "launch" / "race_control.launch.py"
RACE_PARAM = PACKAGE_ROOT / "config" / "race_controller.param.yaml"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"
PRODUCT_ROOT = PACKAGE_ROOT.parents[2]


def test_race_control_launch_uses_autoware_mpc_pid_controller_only():
    source = RACE_LAUNCH.read_text(encoding="utf-8")

    assert 'package="autoware_trajectory_follower_node"' in source
    assert 'executable="controller_node_exe"' in source
    assert '"lateral_controller_mode": "mpc"' in source
    assert '"longitudinal_controller_mode": "pid"' in source
    assert "pure_pursuit_controller" not in source
    assert "/control_bench" not in source
    assert "command_gate" not in source


def test_race_control_launch_declares_contract_arguments_and_remaps_all_io():
    source = RACE_LAUNCH.read_text(encoding="utf-8")

    expected_arguments = {
        "reference_trajectory_topic": "/planning/trajectory",
        "odometry_topic": "/localization/kinematic_state",
        "steering_topic": "/vehicle/status/steering_status",
        "accel_topic": "/localization/acceleration",
        "operation_mode_topic": "/system/operation_mode/state",
        "raw_control_topic": "/control/trajectory_follower/control_cmd",
    }
    for name, default in expected_arguments.items():
        assert name in source
        assert default in source

    assert 'DeclareLaunchArgument(\n                "vehicle_info_param_file"' in source
    assert '"hooke2"' not in source
    assert '"race_controller.param.yaml"' in source

    expected_remaps = {
        "~/input/reference_trajectory": "reference_trajectory_topic",
        "~/input/current_odometry": "odometry_topic",
        "~/input/current_steering": "steering_topic",
        "~/input/current_accel": "accel_topic",
        "~/input/current_operation_mode": "operation_mode_topic",
        "~/output/control_cmd": "raw_control_topic",
    }
    for private_topic, launch_argument in expected_remaps.items():
        assert private_topic in source
        assert launch_argument in source

    for package_name in (
        "autoware_mpc_lateral_controller",
        "autoware_pid_longitudinal_controller",
        "autoware_trajectory_follower_node",
    ):
        assert package_name in source

    expected_param_order = [
        "lateral_defaults",
        "longitudinal_defaults",
        "trajectory_follower_defaults",
        "vehicle_info_param_file",
        "race_param_file",
    ]
    parameter_section = source[source.index("parameters=[") : source.index("remappings=[")]
    last_index = -1
    for token in expected_param_order:
        index = parameter_section.index(token)
        assert index > last_index
        last_index = index


def test_race_controller_param_overlay_has_required_bench_contract_values():
    source = RACE_PARAM.read_text(encoding="utf-8")

    assert 'lateral_controller_mode: "mpc"' in source
    assert 'longitudinal_controller_mode: "pid"' in source
    assert "ego_nearest_dist_threshold:" in source
    assert "ego_nearest_yaw_threshold:" in source
    assert "enable_control_cmd_horizon_pub:" in source
    assert "mpc_weight_lat_error: 5.0" in source
    assert "mpc_weight_terminal_lat_error: 5.0" in source
    assert "mpc_low_curvature_" not in source


def test_autoracer_control_packaging_installs_only_runtime_assets_and_dependencies():
    cmake_source = CMAKE.read_text(encoding="utf-8")
    package_source = PACKAGE_XML.read_text(encoding="utf-8")

    for asset in (
        "race_control.launch.py",
        "race_controller.param.yaml",
    ):
        relative = asset if "/" in asset else (
            f"launch/{asset}" if asset.endswith(".py") else f"config/{asset}"
        )
        assert (PACKAGE_ROOT / relative).is_file()

    assert "DIRECTORY config launch" in cmake_source

    for dependency in (
        "ament_index_python",
        "autoware_mpc_lateral_controller",
        "autoware_pid_longitudinal_controller",
        "autoware_trajectory_follower_node",
        "launch",
        "launch_ros",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_source


def test_dynamic_mpc_prediction_uses_supported_coordinate_without_log_flood():
    lock = (PRODUCT_ROOT / "dependencies" / "versions.lock.yaml").read_text(
        encoding="utf-8"
    )
    patch = PRODUCT_ROOT / "dependencies" / "patches" / (
        "mpc_dynamic_prediction_coordinate.patch"
    )
    source = PRODUCT_ROOT / "vendor_ws" / "src" / "autoware" / "universe" / (
        "control/autoware_mpc_lateral_controller/src/mpc.cpp"
    )

    assert "dependencies/patches/mpc_dynamic_prediction_coordinate.patch" in lock
    assert patch.is_file()
    source_text = source.read_text(encoding="utf-8")
    assert 'm_vehicle_model_ptr->modelName() == "dynamics" ? "frenet" : "world"' in source_text
    assert "prediction_dt,\n    prediction_coordinate);" in source_text
    assert 'prediction_dt, "world");' not in source_text
