from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_SOURCE = (PACKAGE_ROOT / "launch" / "localization.launch.py").read_text(
    encoding="utf-8"
)
PACKAGE_SOURCES = "\n".join(
    path.read_text(encoding="utf-8")
    for path in PACKAGE_ROOT.rglob("*")
    if path.suffix in {".py", ".xml", ".yaml", ".yml"}
    and "test" not in path.parts
)


def test_localization_consumes_only_normalized_sensor_contracts():
    for topic in (
        "/sensing/lidar/concatenated/pointcloud",
        "/sensing/imu/imu_data",
        "/vehicle/status/velocity_status",
        "/sensing/vehicle_velocity_converter/twist_with_covariance",
    ):
        assert topic in PACKAGE_SOURCES


def test_localization_launch_does_not_own_platform_normalization():
    for token in (
        "autoracer_description",
        "autoware_gnss_poser",
        "fixposition",
        "topic_tools",
    ):
        assert token not in LAUNCH_SOURCE


def test_tiled_map_loader_publishes_metadata():
    assert '"enable_selected_load": True' in LAUNCH_SOURCE
    assert (
        '("output/pointcloud_map_metadata", "/map/pointcloud_map_metadata")'
        in LAUNCH_SOURCE
    )
