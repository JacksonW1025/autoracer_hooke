import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

from autoracer_sensing.pointcloud_voxel_filter import (
    pointcloud2_to_xyzi,
    voxel_downsample_xyzi,
    xyzi_to_pointcloud2,
)


def make_cloud(points):
    msg = PointCloud2()
    msg.header.stamp = rclpy.time.Time(seconds=42.0).to_msg()
    msg.header.frame_id = "lidar_top"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = np.asarray(points, dtype=np.float32).tobytes()
    return msg


def test_pointcloud2_round_trip_xyzi():
    points = np.array(
        [
            [0.2, 0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 2.0],
            [5.0, 0.0, 0.0, 3.0],
        ],
        dtype=np.float32,
    )

    cloud = make_cloud(points)
    filtered = pointcloud2_to_xyzi(cloud, min_range_m=0.5, max_range_m=3.0)
    out = xyzi_to_pointcloud2(filtered, cloud.header)

    assert out.header.frame_id == "lidar_top"
    assert out.width == 1
    np.testing.assert_allclose(pointcloud2_to_xyzi(out), points[1:2])


def test_voxel_downsample_and_point_limit():
    points = np.array(
        [
            [0.01, 0.01, 0.01, 1.0],
            [0.02, 0.02, 0.02, 2.0],
            [1.0, 0.0, 0.0, 3.0],
            [2.0, 0.0, 0.0, 4.0],
        ],
        dtype=np.float32,
    )

    sampled = voxel_downsample_xyzi(points, leaf_size_m=0.5, max_points=2)

    assert len(sampled) == 2
    np.testing.assert_allclose(sampled[0], points[0])
