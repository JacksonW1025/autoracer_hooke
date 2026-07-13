from pathlib import Path
import math

import yaml


RC_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = RC_ROOT / "autoracer_rc_description"


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_vehicle_reference_geometry_is_explicit_and_safe():
    data = load_yaml(DESCRIPTION / "config/vehicle_info.param.yaml")
    params = data["/**"]["ros__parameters"]
    assert params["wheel_radius"] == 0.115
    assert params["wheel_base"] == 0.600
    assert params["wheel_tread"] == 0.440
    assert params["max_steer_angle"] == 0.262
    for name, value in params.items():
        assert math.isfinite(value), name
        assert value > 0.0, name
    assert 0.0 < params["max_steer_angle"] < math.pi / 2.0


def test_extrinsics_define_each_rc_sensor_once():
    data = load_yaml(DESCRIPTION / "config/sensor_extrinsics.yaml")
    transforms = data["transforms"]
    assert [(item["parent"], item["child"]) for item in transforms] == [
        ("base_link", "lidar_top"),
        ("base_link", "imu_link"),
    ]
    for item in transforms:
        values = list(item["translation"].values()) + list(item["rotation_rpy"].values())
        assert all(math.isfinite(value) for value in values)


def test_static_tf_launch_reads_the_single_extrinsics_file():
    source = (DESCRIPTION / "launch/static_tf.launch.py").read_text(encoding="utf-8")
    assert "sensor_extrinsics.yaml" in source
    assert "yaml.safe_load" in source
    assert "static_transform_publisher" in source


def test_urdf_does_not_duplicate_extrinsic_numbers():
    source = (DESCRIPTION / "urdf/rc_sensor_mounts.urdf.xacro").read_text(encoding="utf-8")
    for frame in ("base_link", "lidar_top", "imu_link"):
        assert f'name="{frame}"' in source
    assert "<origin" not in source
