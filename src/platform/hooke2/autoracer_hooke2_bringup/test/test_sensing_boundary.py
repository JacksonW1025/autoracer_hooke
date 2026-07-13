from pathlib import Path


SENSING_SOURCE = (
    Path(__file__).resolve().parents[1] / "launch" / "sensing.launch.py"
).read_text(encoding="utf-8")
PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_hooke2_sensing_owns_static_sensor_transforms():
    assert 'get_package_share_directory("hooke2_description")' in SENSING_SOURCE
    assert '"static_tf.launch.py"' in SENSING_SOURCE


def test_fixposition_normalization_terminates_at_standard_topics():
    for token in (
        'get_package_share_directory("autoware_gnss_poser")',
        'package="topic_tools"',
        '"/sensing/gnss/pose_with_covariance"',
        '"/sensing/imu/imu_data"',
    ):
        assert token in SENSING_SOURCE


def test_hardware_driver_switch_does_not_disable_normalization():
    assert 'launch_fixposition_driver = LaunchConfiguration(' in SENSING_SOURCE
    assert SENSING_SOURCE.count("condition=IfCondition(launch_fixposition_driver)") == 2
    assert "fixposition_gnss_normalization," in SENSING_SOURCE
    assert "fixposition_imu_normalization," in SENSING_SOURCE


def test_hooke2_race_injects_platform_parameter_files():
    race_source = (PACKAGE_ROOT / "launch" / "race.launch.py").read_text(
        encoding="utf-8"
    )

    for filename in (
        "controller.param.yaml",
        "vehicle_cmd_gate.param.yaml",
        "race_runtime.param.yaml",
    ):
        assert (PACKAGE_ROOT / "config" / "hooke2" / filename).is_file()
        assert filename in race_source

    for argument in (
        "control_param_file",
        "gate_param_file",
        "runtime_param_file",
        "max_accel_mps2",
        "max_decel_mps2",
        "command_latency_sec",
        "stopping_margin_m",
    ):
        assert f'"{argument}"' in race_source


def test_hooke2_overlays_preserve_pre_migration_effective_values():
    repository_root = next(
        parent
        for parent in Path(__file__).resolve().parents
        if (parent / "src" / "core").is_dir()
    )
    expected_pairs = (
        (
            "controller.param.yaml",
            repository_root
            / "src/core/autoracer_control/config/race_controller.param.yaml",
        ),
        (
            "vehicle_cmd_gate.param.yaml",
            repository_root
            / "src/core/autoracer_safety/config/race/vehicle_cmd_gate.safe.param.yaml",
        ),
        (
            "race_runtime.param.yaml",
            repository_root
            / "src/core/autoracer_safety/config/race/race_runtime.safe.param.yaml",
        ),
    )

    for hooke2_name, shared_path in expected_pairs:
        hooke2_path = PACKAGE_ROOT / "config" / "hooke2" / hooke2_name
        assert hooke2_path.read_text(encoding="utf-8") == shared_path.read_text(
            encoding="utf-8"
        )
