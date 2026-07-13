from pathlib import Path


SENSING_SOURCE = (
    Path(__file__).resolve().parents[1] / "launch" / "sensing.launch.py"
).read_text(encoding="utf-8")


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
