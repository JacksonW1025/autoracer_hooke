import rclpy
from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped
from autoware_map_msgs.srv import GetPartialPointCloudMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import SetBool


class NdtStartupHelper(Node):
    def __init__(self):
        super().__init__("ndt_startup_helper")

        self.declare_parameter("initial_pose_topic", "/localization/ndt_initial_pose")
        self.declare_parameter("ndt_pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("trigger_service", "/localization/ndt_trigger")
        self.declare_parameter("map_service", "/map/get_partial_pointcloud_map")
        self.declare_parameter("wait_for_map_service", True)
        self.declare_parameter("required_initial_messages", 3)
        self.declare_parameter("fresh_initial_pose_sec", 0.5)
        self.declare_parameter("ndt_pose_timeout_sec", 2.0)
        self.declare_parameter("retrigger_cooldown_sec", 5.0)
        self.declare_parameter("min_nvtl_score", 2.3)
        self.declare_parameter("max_iteration_num", 30)
        self.declare_parameter("max_exe_time_ms", 100.0)

        self._required_initial_messages = int(
            self.get_parameter("required_initial_messages").value
        )
        self._fresh_initial_pose_sec = float(
            self.get_parameter("fresh_initial_pose_sec").value
        )
        self._ndt_pose_timeout_sec = float(self.get_parameter("ndt_pose_timeout_sec").value)
        self._retrigger_cooldown_sec = float(
            self.get_parameter("retrigger_cooldown_sec").value
        )
        self._min_nvtl_score = float(self.get_parameter("min_nvtl_score").value)
        self._max_iteration_num = int(self.get_parameter("max_iteration_num").value)
        self._max_exe_time_ms = float(self.get_parameter("max_exe_time_ms").value)
        self._wait_for_map_service = bool(self.get_parameter("wait_for_map_service").value)

        self._initial_count = 0
        self._last_initial_receipt = None
        self._last_ndt_receipt = None
        self._last_trigger_attempt = None
        self._activation_in_flight = False
        self._activated_once = False
        self._latest_score = None
        self._latest_iteration = None
        self._latest_exe_time = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initial_pose_topic").value,
            self._on_initial_pose,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("ndt_pose_topic").value,
            self._on_ndt_pose,
            10,
        )
        self.create_subscription(
            Float32Stamped,
            "/nearest_voxel_transformation_likelihood",
            self._on_score,
            10,
        )
        self.create_subscription(Int32Stamped, "/iteration_num", self._on_iteration, 10)
        self.create_subscription(Float32Stamped, "/exe_time_ms", self._on_exe_time, 10)

        self._trigger_client = self.create_client(
            SetBool, self.get_parameter("trigger_service").value
        )
        self._map_client = self.create_client(
            GetPartialPointCloudMap, self.get_parameter("map_service").value
        )
        self.create_timer(1.0, self._on_timer)

    def _on_initial_pose(self, _msg):
        self._initial_count += 1
        self._last_initial_receipt = self.get_clock().now()

    def _on_ndt_pose(self, _msg):
        self._last_ndt_receipt = self.get_clock().now()

    def _on_score(self, msg):
        self._latest_score = float(msg.data)

    def _on_iteration(self, msg):
        self._latest_iteration = int(msg.data)

    def _on_exe_time(self, msg):
        self._latest_exe_time = float(msg.data)

    def _on_timer(self):
        now = self.get_clock().now()
        self._warn_on_quality()

        if not self._initial_pose_is_ready(now):
            return

        if not self._trigger_client.service_is_ready():
            self._trigger_client.wait_for_service(timeout_sec=0.0)
            return

        if self._wait_for_map_service and not self._map_client.service_is_ready():
            self._map_client.wait_for_service(timeout_sec=0.0)
            return

        if not self._activated_once:
            self._trigger(now, "startup")
            return

        if self._ndt_is_stale(now) and self._cooldown_elapsed(now):
            self._trigger(now, "NDT pose stale")

    def _trigger(self, now, reason):
        if self._activation_in_flight:
            return

        request = SetBool.Request()
        request.data = True
        future = self._trigger_client.call_async(request)
        future.add_done_callback(lambda done: self._on_trigger_response(done, reason))
        self._activation_in_flight = True
        self._last_trigger_attempt = now
        self._initial_count = 0
        self.get_logger().info(f"Calling NDT trigger service: {reason}")

    def _on_trigger_response(self, future, reason):
        self._activation_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"NDT trigger failed for {reason}: {exc}")
            return

        if response.success:
            self._activated_once = True
            self.get_logger().info(f"NDT trigger accepted: {reason}")
        else:
            self.get_logger().warn(f"NDT trigger rejected: {response.message}")

    def _initial_pose_is_ready(self, now):
        if self._last_initial_receipt is None:
            return False
        age = (now - self._last_initial_receipt).nanoseconds / 1e9
        return (
            self._initial_count >= self._required_initial_messages
            and age <= self._fresh_initial_pose_sec
        )

    def _ndt_is_stale(self, now):
        if self._last_ndt_receipt is None:
            return self._activated_once
        age = (now - self._last_ndt_receipt).nanoseconds / 1e9
        if age > self._ndt_pose_timeout_sec:
            self.get_logger().warn(
                f"NDT pose stale for {age:.2f}s",
                throttle_duration_sec=2.0,
            )
            return True
        return False

    def _cooldown_elapsed(self, now):
        if self._last_trigger_attempt is None:
            return True
        return (now - self._last_trigger_attempt).nanoseconds / 1e9 >= self._retrigger_cooldown_sec

    def _warn_on_quality(self):
        if self._latest_score is not None and self._latest_score < self._min_nvtl_score:
            self.get_logger().warn(
                f"NDT score low: {self._latest_score:.3f} < {self._min_nvtl_score:.3f}",
                throttle_duration_sec=2.0,
            )
        if self._latest_iteration is not None and self._latest_iteration >= self._max_iteration_num:
            self.get_logger().warn(
                f"NDT iteration reached limit: {self._latest_iteration}",
                throttle_duration_sec=2.0,
            )
        if self._latest_exe_time is not None and self._latest_exe_time > self._max_exe_time_ms:
            self.get_logger().warn(
                f"NDT execution slow: {self._latest_exe_time:.1f} ms",
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = NdtStartupHelper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
