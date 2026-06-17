import math
from pathlib import Path

from autoracer_localization.scan_accumulator import (
    PlanarOdomState,
    SlidingPointCloudAccumulator,
    transform_points_between_states,
)


def test_transform_points_between_states_uses_relative_planar_odometry():
    source = PlanarOdomState(stamp_sec=0.0, x=0.0, y=0.0, yaw=0.0)
    target = PlanarOdomState(stamp_sec=1.0, x=1.0, y=0.0, yaw=0.0)

    transformed = transform_points_between_states(
        [(5.0, 0.0, 1.0, 7.0)],
        source,
        target,
    )

    assert transformed == [(4.0, 0.0, 1.0, 7.0)]


def test_transform_points_between_states_handles_current_yaw():
    source = PlanarOdomState(stamp_sec=0.0, x=0.0, y=0.0, yaw=0.0)
    target = PlanarOdomState(stamp_sec=1.0, x=0.0, y=0.0, yaw=math.pi / 2.0)

    transformed = transform_points_between_states(
        [(1.0, 0.0, 0.0, 1.0)],
        source,
        target,
    )

    x, y, z, intensity = transformed[0]
    assert abs(x) < 1e-9
    assert abs(y + 1.0) < 1e-9
    assert z == 0.0
    assert intensity == 1.0


def test_sliding_accumulator_keeps_recent_frames_and_voxelizes_in_current_frame():
    accumulator = SlidingPointCloudAccumulator(max_frames=3, max_age_sec=0.25, voxel_size_m=0.5)

    accumulator.add_frame(
        stamp_sec=0.0,
        odom_state=PlanarOdomState(stamp_sec=0.0, x=0.0, y=0.0, yaw=0.0),
        points=[(5.0, 0.0, 0.0, 1.0)],
    )
    accumulator.add_frame(
        stamp_sec=0.1,
        odom_state=PlanarOdomState(stamp_sec=0.1, x=1.0, y=0.0, yaw=0.0),
        points=[(4.1, 0.0, 0.0, 2.0), (4.2, 0.0, 0.0, 3.0)],
    )
    accumulator.add_frame(
        stamp_sec=0.4,
        odom_state=PlanarOdomState(stamp_sec=0.4, x=2.0, y=0.0, yaw=0.0),
        points=[(3.0, 0.0, 0.0, 4.0)],
    )

    output = accumulator.accumulated_points(
        current_stamp_sec=0.4,
        current_odom_state=PlanarOdomState(stamp_sec=0.4, x=2.0, y=0.0, yaw=0.0),
    )

    assert len(output) == 1
    assert output[0] == (3.0, 0.0, 0.0, 4.0)


def test_sliding_accumulator_can_select_longer_history_per_publish():
    accumulator = SlidingPointCloudAccumulator(max_frames=5, max_age_sec=1.0, voxel_size_m=0.0)
    state = PlanarOdomState(stamp_sec=0.0, x=0.0, y=0.0, yaw=0.0)
    for index in range(5):
        stamp = index * 0.1
        accumulator.add_frame(
            stamp_sec=stamp,
            odom_state=PlanarOdomState(stamp_sec=stamp, x=0.0, y=0.0, yaw=0.0),
            points=[(float(index), 0.0, 0.0, float(index))],
        )

    short_output = accumulator.accumulated_points(
        current_stamp_sec=0.4,
        current_odom_state=state,
        max_frames=2,
        max_age_sec=0.25,
    )
    long_output = accumulator.accumulated_points(
        current_stamp_sec=0.4,
        current_odom_state=state,
        max_frames=5,
        max_age_sec=1.0,
    )

    assert [point[0] for point in short_output] == [3.0, 4.0]
    assert [point[0] for point in long_output] == [0.0, 1.0, 2.0, 3.0, 4.0]


def test_scan_accumulator_uses_sensor_data_qos_for_pointcloud_io():
    source = Path(__file__).resolve().parents[1] / "autoracer_localization" / "scan_accumulator.py"
    text = source.read_text(encoding="utf-8")

    assert "qos_profile_sensor_data" in text
    assert "create_subscription(PointCloud2, input_topic, self._on_pointcloud, qos_profile_sensor_data)" in text
    assert "create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)" in text
