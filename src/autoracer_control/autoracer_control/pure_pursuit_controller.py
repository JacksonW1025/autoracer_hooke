import math

from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitController(Node):
    def __init__(self):
        super().__init__("pure_pursuit_controller")
        self.declare_parameter("trajectory_topic", "/planning/trajectory")
        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("output_topic", "/autoracer/control/raw_control_cmd")
        self.declare_parameter("wheel_base_m", 1.9)
        self.declare_parameter("max_speed_mps", 1.5)
        self.declare_parameter("max_steer_rad", 0.488)
        self.declare_parameter("min_lookahead_m", 4.0)
        self.declare_parameter("lookahead_gain", 1.5)
        self.declare_parameter("goal_tolerance_m", 1.0)
        self.declare_parameter("longitudinal_kp", 0.8)
        self.declare_parameter("max_accel_mps2", 0.8)
        self.declare_parameter("max_decel_mps2", -1.5)
        self.declare_parameter("control_rate_hz", 30.0)

        self._wheel_base = float(self.get_parameter("wheel_base_m").value)
        self._max_speed = float(self.get_parameter("max_speed_mps").value)
        self._max_steer = float(self.get_parameter("max_steer_rad").value)
        self._min_lookahead = float(self.get_parameter("min_lookahead_m").value)
        self._lookahead_gain = float(self.get_parameter("lookahead_gain").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self._kp = float(self.get_parameter("longitudinal_kp").value)
        self._max_accel = float(self.get_parameter("max_accel_mps2").value)
        self._max_decel = float(self.get_parameter("max_decel_mps2").value)

        self._trajectory = []
        self._pose = None
        self._speed = 0.0

        self._control_pub = self.create_publisher(
            Control, self.get_parameter("output_topic").value, 10
        )

        self.create_subscription(
            Trajectory, self.get_parameter("trajectory_topic").value, self._on_trajectory, 1
        )
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._on_pose, 10
        )
        self.create_subscription(
            VelocityReport, self.get_parameter("velocity_topic").value, self._on_velocity, 10
        )

        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.create_timer(period, self._on_timer)

    def _on_trajectory(self, msg):
        self._trajectory = [
            (
                point.pose.position.x,
                point.pose.position.y,
                point.longitudinal_velocity_mps,
            )
            for point in msg.points
        ]

    def _on_pose(self, msg):
        pose = msg.pose.pose
        self._pose = (
            pose.position.x,
            pose.position.y,
            _yaw_from_quaternion(pose.orientation),
        )

    def _on_velocity(self, msg):
        self._speed = float(msg.longitudinal_velocity)

    def _on_timer(self):
        command = Control()
        now = self.get_clock().now().to_msg()
        command.stamp = now
        command.lateral.stamp = now
        command.longitudinal.stamp = now

        if self._pose is None or len(self._trajectory) < 2:
            self._publish_stop(command)
            return

        target_index = self._find_target_index()
        if target_index is None:
            self._publish_stop(command)
            return

        target = self._trajectory[target_index]
        steer = self._compute_steer(target)
        goal_distance = math.hypot(
            self._trajectory[-1][0] - self._pose[0],
            self._trajectory[-1][1] - self._pose[1],
        )
        target_speed = 0.0 if goal_distance < self._goal_tolerance else target[2]
        target_speed = _clamp(target_speed, 0.0, self._max_speed)
        accel = _clamp(self._kp * (target_speed - self._speed), self._max_decel, self._max_accel)

        command.lateral.steering_tire_angle = steer
        command.lateral.steering_tire_rotation_rate = 0.0
        command.lateral.is_defined_steering_tire_rotation_rate = False
        command.longitudinal.velocity = target_speed
        command.longitudinal.acceleration = accel
        command.longitudinal.jerk = 0.0
        command.longitudinal.is_defined_acceleration = True
        command.longitudinal.is_defined_jerk = False
        self._control_pub.publish(command)

    def _find_target_index(self):
        nearest_index = min(
            range(len(self._trajectory)),
            key=lambda i: math.hypot(
                self._trajectory[i][0] - self._pose[0],
                self._trajectory[i][1] - self._pose[1],
            ),
        )
        lookahead = max(self._min_lookahead, abs(self._speed) * self._lookahead_gain)
        for index in range(nearest_index, len(self._trajectory)):
            dx = self._trajectory[index][0] - self._pose[0]
            dy = self._trajectory[index][1] - self._pose[1]
            local_x = math.cos(self._pose[2]) * dx + math.sin(self._pose[2]) * dy
            distance = math.hypot(dx, dy)
            if local_x > 0.0 and distance >= lookahead:
                return index
        return len(self._trajectory) - 1

    def _compute_steer(self, target):
        dx = target[0] - self._pose[0]
        dy = target[1] - self._pose[1]
        yaw = self._pose[2]
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        lookahead_sq = max(local_x * local_x + local_y * local_y, 0.01)
        curvature = 2.0 * local_y / lookahead_sq
        steer = math.atan(self._wheel_base * curvature)
        return _clamp(steer, -self._max_steer, self._max_steer)

    def _publish_stop(self, command):
        command.longitudinal.velocity = 0.0
        command.longitudinal.acceleration = self._max_decel
        command.longitudinal.is_defined_acceleration = True
        command.lateral.steering_tire_angle = 0.0
        self._control_pub.publish(command)


def main():
    rclpy.init()
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

