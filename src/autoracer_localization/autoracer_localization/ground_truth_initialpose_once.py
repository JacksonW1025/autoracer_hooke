import copy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool


def clone_initialpose(msg: PoseWithCovarianceStamped, *, frame_id: str) -> PoseWithCovarianceStamped:
    out = copy.deepcopy(msg)
    out.header.frame_id = frame_id
    return out


class GroundTruthInitialposeOnce(Node):
    def __init__(self, **kwargs):
        super().__init__("ground_truth_initialpose_once", **kwargs)
        self.declare_parameter("input_gt_topic", "/carmaker/ground_truth/pose")
        self.declare_parameter("output_initialpose_topic", "/localization/initialpose_once")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("min_stamp_sec", 0.0)
        self.declare_parameter("ekf_trigger_service", "")
        self.declare_parameter("ndt_trigger_service", "")
        self.declare_parameter("trigger_service_wait_sec", 0.25)
        self._published = False
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_initialpose_topic").value,
            10,
        )
        self._trigger_clients = [
            self.create_client(SetBool, service_name)
            for service_name in (
                str(self.get_parameter("ekf_trigger_service").value),
                str(self.get_parameter("ndt_trigger_service").value),
            )
            if service_name
        ]
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_gt_topic").value,
            self._on_gt,
            10,
        )
        self.get_logger().info("Waiting for one GT pose to seed pure LiDAR NDT startup")

    def _on_gt(self, msg: PoseWithCovarianceStamped):
        if self._published:
            return
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        min_stamp_sec = float(self.get_parameter("min_stamp_sec").value)
        if stamp_sec < min_stamp_sec:
            return
        self._publisher.publish(
            clone_initialpose(msg, frame_id=str(self.get_parameter("map_frame").value))
        )
        self._trigger_startup_services()
        self._published = True
        self.get_logger().info(
            f"Published one GT-derived initialpose at stamp={stamp_sec:.3f}; future GT messages ignored"
        )

    def _trigger_startup_services(self):
        wait_sec = float(self.get_parameter("trigger_service_wait_sec").value)
        for client in self._trigger_clients:
            if not client.wait_for_service(timeout_sec=wait_sec):
                self.get_logger().warn("Localization trigger service is not ready; skipping startup trigger")
                continue
            request = SetBool.Request()
            request.data = True
            client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthInitialposeOnce()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
