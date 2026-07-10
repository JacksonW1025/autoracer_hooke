import math
import unittest

import rclpy

from autoracer_localization.manual_seed_pose_publisher import build_seed_pose
from autoracer_localization.manual_seed_pose_publisher import seed_pose_from_initialpose


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ManualSeedPosePublisherTest(unittest.TestCase):
    def test_build_seed_pose_uses_configured_map_pose_and_covariance(self):
        stamp = rclpy.time.Time(seconds=42.0)

        msg = build_seed_pose(
            stamp=stamp,
            frame_id="map",
            x=1.2,
            y=-0.4,
            z=0.05,
            yaw=0.75,
            xy_variance=0.8,
            z_variance=0.12,
            yaw_variance=0.04,
        )

        self.assertEqual(msg.header.frame_id, "map")
        self.assertEqual(msg.header.stamp.sec, 42)
        self.assertAlmostEqual(msg.pose.pose.position.x, 1.2, places=6)
        self.assertAlmostEqual(msg.pose.pose.position.y, -0.4, places=6)
        self.assertAlmostEqual(msg.pose.pose.position.z, 0.05, places=6)
        self.assertAlmostEqual(
            yaw_from_quaternion(msg.pose.pose.orientation), 0.75, places=6
        )
        self.assertAlmostEqual(msg.pose.covariance[0], 0.8, places=6)
        self.assertAlmostEqual(msg.pose.covariance[7], 0.8, places=6)
        self.assertAlmostEqual(msg.pose.covariance[14], 0.12, places=6)
        self.assertAlmostEqual(msg.pose.covariance[35], 0.04, places=6)

    def test_seed_pose_from_initialpose_refreshes_stamp_and_preserves_pose(self):
        source_stamp = rclpy.time.Time(seconds=10.0)
        publish_stamp = rclpy.time.Time(seconds=43.0)
        source = build_seed_pose(
            stamp=source_stamp,
            frame_id="map",
            x=2.0,
            y=3.0,
            z=0.0,
            yaw=-0.25,
            xy_variance=0.2,
            z_variance=0.1,
            yaw_variance=0.03,
        )

        msg = seed_pose_from_initialpose(source, publish_stamp, fallback_frame_id="map")

        self.assertEqual(msg.header.frame_id, "map")
        self.assertEqual(msg.header.stamp.sec, 43)
        self.assertAlmostEqual(msg.pose.pose.position.x, 2.0, places=6)
        self.assertAlmostEqual(msg.pose.pose.position.y, 3.0, places=6)
        self.assertAlmostEqual(
            yaw_from_quaternion(msg.pose.pose.orientation), -0.25, places=6
        )
        self.assertAlmostEqual(msg.pose.covariance[0], 0.2, places=6)


if __name__ == "__main__":
    unittest.main()
