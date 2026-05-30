import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from autoracer_sensing.mock_lidar_tools import (
    CropConfig,
    Pose2D,
    crop_mock_scan,
    offset_initial_pose,
    read_pcd_xyzi,
    transform_lidar_to_map,
    transform_map_to_lidar,
    voxel_downsample,
    write_pcd_xyzi,
)


class MockLidarToolsTest(unittest.TestCase):
    def test_pcd_binary_xyzi_round_trip(self):
        points = np.array(
            [
                [1.0, 2.0, 3.0, 10.0],
                [-1.5, 0.5, 0.0, 20.0],
            ],
            dtype=np.float32,
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scan.pcd"
            write_pcd_xyzi(path, points)
            loaded = read_pcd_xyzi(path)

        self.assertEqual(loaded.dtype, np.float32)
        np.testing.assert_allclose(loaded, points)

    def test_initial_pose_offset_is_in_vehicle_frame(self):
        truth = Pose2D(10.0, 20.0, 0.0, math.pi * 0.5)
        initial = offset_initial_pose(truth, dx=1.0, dy=0.5, dyaw=math.radians(5.0))

        self.assertAlmostEqual(initial.x, 9.5)
        self.assertAlmostEqual(initial.y, 21.0)
        self.assertAlmostEqual(initial.yaw, math.pi * 0.5 + math.radians(5.0))

    def test_map_lidar_transform_round_trip(self):
        truth = Pose2D(2.0, -3.0, 0.0, math.radians(30.0))
        lidar_points = np.array(
            [
                [10.0, 0.0, 0.0, 1.0],
                [2.0, -4.0, 1.0, 2.0],
            ],
            dtype=np.float32,
        )

        map_points = transform_lidar_to_map(lidar_points, truth)
        round_trip = transform_map_to_lidar(map_points, truth)
        np.testing.assert_allclose(round_trip, lidar_points, atol=1e-5)

    def test_crop_filters_by_pandar_view_and_downsamples(self):
        truth = Pose2D(0.0, 0.0, 0.0, 0.0)
        in_view_lidar = np.array(
            [
                [10.0, 0.0, 0.0, 1.0],
                [10.1, 0.0, 0.0, 2.0],
                [20.0, 0.0, 1.0, 3.0],
            ],
            dtype=np.float32,
        )
        out_of_view_lidar = np.array(
            [
                [100.0, 0.0, 0.0, 4.0],
                [1.0, 0.0, 10.0, 5.0],
            ],
            dtype=np.float32,
        )
        map_points = transform_lidar_to_map(
            np.vstack((in_view_lidar, out_of_view_lidar)), truth
        )
        scan = crop_mock_scan(
            map_points,
            truth,
            CropConfig(max_range_m=60.0, voxel_leaf_m=0.3),
        )

        self.assertEqual(len(scan), 2)
        self.assertTrue(np.all(scan[:, 0] < 25.0))
        self.assertTrue(np.all(scan[:, 2] < 2.0))

    def test_voxel_downsample_keeps_one_point_per_voxel(self):
        points = np.array(
            [
                [0.01, 0.01, 0.01, 1.0],
                [0.02, 0.02, 0.02, 2.0],
                [1.0, 0.0, 0.0, 3.0],
            ],
            dtype=np.float32,
        )
        sampled = voxel_downsample(points, 0.5)
        self.assertEqual(len(sampled), 2)


if __name__ == "__main__":
    unittest.main()
