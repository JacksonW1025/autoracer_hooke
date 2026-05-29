import math
from pathlib import Path

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml

try:
    import lanelet2
except ImportError:
    lanelet2 = None


def _yaw_to_quaternion(yaw):
    pose = Pose()
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose.orientation


def _duration_from_seconds(seconds):
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int((seconds - duration.sec) * 1_000_000_000)
    return duration


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class LaneletRoutePlanner(Node):
    def __init__(self):
        super().__init__("lanelet_route_planner")
        self.declare_parameter("osm_path", "")
        self.declare_parameter("map_projector_info_path", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("current_pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/planning/mission_path")
        self.declare_parameter("trajectory_topic", "/planning/trajectory")
        self.declare_parameter("marker_topic", "/planning/route_marker")
        self.declare_parameter("speed_limit_mps", 1.5)
        self.declare_parameter("min_point_distance_m", 1.0)

        self._frame_id = self.get_parameter("frame_id").value
        self._speed_limit = float(self.get_parameter("speed_limit_mps").value)
        self._min_point_distance = float(self.get_parameter("min_point_distance_m").value)
        self._current_pose = None
        self._lanelet_map = None
        self._routing_graph = None

        self._path_pub = self.create_publisher(
            NavPath, self.get_parameter("path_topic").value, 1
        )
        self._trajectory_pub = self.create_publisher(
            Trajectory, self.get_parameter("trajectory_topic").value, 1
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 1
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("current_pose_topic").value,
            self._on_current_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            self._on_goal_pose,
            10,
        )

        self._load_map()

    def _load_map(self):
        if lanelet2 is None:
            self.get_logger().error("lanelet2 Python bindings are not available")
            return

        osm_path = Path(self.get_parameter("osm_path").value)
        projector_path = Path(self.get_parameter("map_projector_info_path").value)
        if not osm_path.exists():
            self.get_logger().error(f"Lanelet2 OSM file does not exist: {osm_path}")
            return
        if not projector_path.exists():
            self.get_logger().error(f"map_projector_info.yaml does not exist: {projector_path}")
            return

        with projector_path.open("r", encoding="utf-8") as stream:
            projector_info = yaml.safe_load(stream) or {}
        origin_info = projector_info.get("map_origin", {})
        origin = lanelet2.io.Origin(
            float(origin_info.get("latitude", 0.0)),
            float(origin_info.get("longitude", 0.0)),
        )

        if hasattr(lanelet2.projection, "LocalCartesianProjector"):
            projector = lanelet2.projection.LocalCartesianProjector(origin)
        else:
            projector = lanelet2.projection.UtmProjector(origin)

        self._lanelet_map = lanelet2.io.load(str(osm_path), projector)
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        self._routing_graph = lanelet2.routing.RoutingGraph(self._lanelet_map, traffic_rules)
        self.get_logger().info(f"Loaded Lanelet2 map: {osm_path}")

    def _on_current_pose(self, msg):
        self._current_pose = msg.pose.pose

    def _on_goal_pose(self, msg):
        if self._current_pose is None:
            self.get_logger().warn("Goal received before localization pose is available")
            return
        if self._lanelet_map is None or self._routing_graph is None:
            self.get_logger().error("Lanelet2 map is not ready")
            return

        start = (self._current_pose.position.x, self._current_pose.position.y)
        goal = (msg.pose.position.x, msg.pose.position.y)
        points = self._plan_centerline(start, goal)
        if len(points) < 2:
            self.get_logger().error("Route planning produced fewer than two points")
            return

        self._publish_path(points)
        self._publish_trajectory(points)
        self._publish_marker(points)
        self.get_logger().info(f"Published route with {len(points)} centerline points")

    def _nearest_lanelet(self, xy):
        point = lanelet2.core.BasicPoint2d(float(xy[0]), float(xy[1]))
        nearest = lanelet2.geometry.findNearest(self._lanelet_map.laneletLayer, point, 5)
        if not nearest:
            return None
        return nearest[0][1]

    def _plan_centerline(self, start_xy, goal_xy):
        start_lanelet = self._nearest_lanelet(start_xy)
        goal_lanelet = self._nearest_lanelet(goal_xy)
        if start_lanelet is None or goal_lanelet is None:
            return []

        route = self._routing_graph.getRoute(start_lanelet, goal_lanelet)
        if route is None:
            try:
                lanelet_path = self._routing_graph.shortestPath(start_lanelet, goal_lanelet)
            except TypeError:
                lanelet_path = self._routing_graph.shortestPath(start_lanelet, goal_lanelet, 0)
        else:
            lanelet_path = route.shortestPath()
        if lanelet_path is None:
            return []

        points = []
        for lanelet in lanelet_path:
            for point in lanelet.centerline:
                xyz = (float(point.x), float(point.y), float(point.z))
                if not points or _distance(points[-1], xyz) >= self._min_point_distance:
                    points.append(xyz)
        return points

    def _pose_for_index(self, points, index):
        current = points[index]
        if index < len(points) - 1:
            other = points[index + 1]
        else:
            other = points[index - 1]
        yaw = math.atan2(other[1] - current[1], other[0] - current[0])

        pose = Pose()
        pose.position.x = current[0]
        pose.position.y = current[1]
        pose.position.z = current[2]
        pose.orientation = _yaw_to_quaternion(yaw)
        return pose

    def _publish_path(self, points):
        path = NavPath()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id
        for index, _ in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose = self._pose_for_index(points, index)
            path.poses.append(pose)
        self._path_pub.publish(path)

    def _publish_trajectory(self, points):
        trajectory = Trajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = self._frame_id

        elapsed = 0.0
        for index, _ in enumerate(points):
            if index > 0:
                elapsed += _distance(points[index - 1], points[index]) / max(self._speed_limit, 0.1)
            point = TrajectoryPoint()
            point.time_from_start = _duration_from_seconds(elapsed)
            point.pose = self._pose_for_index(points, index)
            point.longitudinal_velocity_mps = self._speed_limit
            point.acceleration_mps2 = 0.0
            trajectory.points.append(point)

        trajectory.points[-1].longitudinal_velocity_mps = 0.0
        self._trajectory_pub.publish(trajectory)

    def _publish_marker(self, points):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = "autoracer_route"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.95
        for x, y, z in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = z + 0.2
            marker.points.append(p)
        self._marker_pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init()
    node = LaneletRoutePlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
