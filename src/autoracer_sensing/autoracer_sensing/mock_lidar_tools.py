import argparse
import copy
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml


DEFAULT_MAP_NAME = "whale_map_20251107"
DEFAULT_SCENARIO_DIR_NAME = "mock_lidar_scenarios"
DEFAULT_SCAN_TOPIC = "/sensing/lidar/concatenated/pointcloud"
DEFAULT_INITIAL_POSE_TOPIC = "/localization/ndt_initial_pose"
DEFAULT_BASE_TO_LIDAR_X = 1.90
DEFAULT_BASE_TO_LIDAR_Y = 0.0
DEFAULT_BASE_TO_LIDAR_Z = 1.384
DEFAULT_BASE_TO_LIDAR_YAW = math.pi * 0.5
DEFAULT_MIN_RANGE_M = 0.3
DEFAULT_MAX_RANGE_M = 60.0
DEFAULT_MIN_ELEVATION_DEG = -24.985
DEFAULT_MAX_ELEVATION_DEG = 14.794
DEFAULT_VOXEL_LEAF_M = 0.3
DEFAULT_INITIAL_OFFSET_X = 1.0
DEFAULT_INITIAL_OFFSET_Y = 0.5
DEFAULT_INITIAL_OFFSET_YAW_DEG = 5.0


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class CropConfig:
    min_range_m: float = DEFAULT_MIN_RANGE_M
    max_range_m: float = DEFAULT_MAX_RANGE_M
    min_elevation_deg: float = DEFAULT_MIN_ELEVATION_DEG
    max_elevation_deg: float = DEFAULT_MAX_ELEVATION_DEG
    voxel_leaf_m: float = DEFAULT_VOXEL_LEAF_M


def default_map_path() -> Path:
    root = Path(os.environ.get("ROOT_DIR", os.getcwd()))
    return Path(os.environ.get("MAP_PATH", root / "maps" / DEFAULT_MAP_NAME))


def default_scenario_dir(map_path: Path) -> Path:
    return Path(os.environ.get("MOCK_LIDAR_SCENARIO_DIR", map_path / DEFAULT_SCENARIO_DIR_NAME))


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float):
    try:
        from geometry_msgs.msg import Quaternion
    except ImportError as exc:
        raise RuntimeError("geometry_msgs is required for quaternion conversion") from exc

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_msg(msg) -> Pose2D:
    pose = msg.pose.pose
    return Pose2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        z=float(pose.position.z),
        yaw=yaw_from_quaternion(pose.orientation),
    )


def pose_to_dict(pose: Pose2D) -> dict:
    return {
        "x": float(pose.x),
        "y": float(pose.y),
        "z": float(pose.z),
        "yaw": float(normalize_angle(pose.yaw)),
    }


def pose_from_dict(data: dict) -> Pose2D:
    return Pose2D(
        x=float(data.get("x", 0.0)),
        y=float(data.get("y", 0.0)),
        z=float(data.get("z", 0.0)),
        yaw=float(data.get("yaw", 0.0)),
    )


def offset_initial_pose(
    truth_pose: Pose2D,
    dx: float = DEFAULT_INITIAL_OFFSET_X,
    dy: float = DEFAULT_INITIAL_OFFSET_Y,
    dyaw: float = math.radians(DEFAULT_INITIAL_OFFSET_YAW_DEG),
) -> Pose2D:
    cos_yaw = math.cos(truth_pose.yaw)
    sin_yaw = math.sin(truth_pose.yaw)
    return Pose2D(
        x=truth_pose.x + cos_yaw * dx - sin_yaw * dy,
        y=truth_pose.y + sin_yaw * dx + cos_yaw * dy,
        z=truth_pose.z,
        yaw=normalize_angle(truth_pose.yaw + dyaw),
    )


def covariance_6d(xy_variance: float = 1.0, z_variance: float = 0.25, yaw_variance: float = 0.03):
    covariance = [0.0] * 36
    covariance[0] = float(xy_variance)
    covariance[7] = float(xy_variance)
    covariance[14] = float(z_variance)
    covariance[21] = 0.01
    covariance[28] = 0.01
    covariance[35] = float(yaw_variance)
    return covariance


def _rotation_z(yaw: float) -> np.ndarray:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )


def transform_map_to_lidar(points_xyzi: np.ndarray, base_pose: Pose2D) -> np.ndarray:
    points = np.asarray(points_xyzi, dtype=np.float32)
    xyz = points[:, :3]
    map_to_base_rot = _rotation_z(base_pose.yaw)
    map_to_base_translation = np.array([base_pose.x, base_pose.y, base_pose.z], dtype=np.float32)
    base_xyz = (xyz - map_to_base_translation) @ map_to_base_rot

    base_to_lidar_rot = _rotation_z(DEFAULT_BASE_TO_LIDAR_YAW)
    base_to_lidar_translation = np.array(
        [DEFAULT_BASE_TO_LIDAR_X, DEFAULT_BASE_TO_LIDAR_Y, DEFAULT_BASE_TO_LIDAR_Z],
        dtype=np.float32,
    )
    lidar_xyz = (base_xyz - base_to_lidar_translation) @ base_to_lidar_rot
    return np.column_stack((lidar_xyz, points[:, 3])).astype(np.float32, copy=False)


def transform_lidar_to_map(points_xyzi: np.ndarray, base_pose: Pose2D) -> np.ndarray:
    points = np.asarray(points_xyzi, dtype=np.float32)
    xyz = points[:, :3]
    base_to_lidar_rot = _rotation_z(DEFAULT_BASE_TO_LIDAR_YAW)
    base_to_lidar_translation = np.array(
        [DEFAULT_BASE_TO_LIDAR_X, DEFAULT_BASE_TO_LIDAR_Y, DEFAULT_BASE_TO_LIDAR_Z],
        dtype=np.float32,
    )
    base_xyz = xyz @ base_to_lidar_rot.T + base_to_lidar_translation

    map_to_base_rot = _rotation_z(base_pose.yaw)
    map_to_base_translation = np.array([base_pose.x, base_pose.y, base_pose.z], dtype=np.float32)
    map_xyz = base_xyz @ map_to_base_rot.T + map_to_base_translation
    return np.column_stack((map_xyz, points[:, 3])).astype(np.float32, copy=False)


def filter_lidar_view(points_xyzi: np.ndarray, crop: CropConfig) -> np.ndarray:
    if len(points_xyzi) == 0:
        return points_xyzi

    xyz = points_xyzi[:, :3]
    distance_xy = np.hypot(xyz[:, 0], xyz[:, 1])
    distance = np.linalg.norm(xyz, axis=1)
    elevation = np.degrees(np.arctan2(xyz[:, 2], distance_xy))
    mask = (
        (distance >= float(crop.min_range_m))
        & (distance <= float(crop.max_range_m))
        & (elevation >= float(crop.min_elevation_deg))
        & (elevation <= float(crop.max_elevation_deg))
        & np.isfinite(distance)
    )
    return points_xyzi[mask]


def voxel_downsample(points_xyzi: np.ndarray, leaf_size: float) -> np.ndarray:
    leaf = float(leaf_size)
    if leaf <= 0.0 or len(points_xyzi) <= 1:
        return points_xyzi.astype(np.float32, copy=False)
    keys = np.floor(points_xyzi[:, :3] / leaf).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points_xyzi[np.sort(indices)].astype(np.float32, copy=False)


def crop_mock_scan(map_points_xyzi: np.ndarray, truth_pose: Pose2D, crop: CropConfig) -> np.ndarray:
    lidar_points = transform_map_to_lidar(map_points_xyzi, truth_pose)
    visible_points = filter_lidar_view(lidar_points, crop)
    return voxel_downsample(visible_points, crop.voxel_leaf_m)


def _dtype_for_field(field_name: str, field_type: str, field_size: int, count: int):
    if field_type == "F" and field_size == 4:
        base = "<f4"
    elif field_type == "F" and field_size == 8:
        base = "<f8"
    elif field_type == "U" and field_size == 1:
        base = "u1"
    elif field_type == "U" and field_size == 2:
        base = "<u2"
    elif field_type == "U" and field_size == 4:
        base = "<u4"
    elif field_type == "I" and field_size == 1:
        base = "i1"
    elif field_type == "I" and field_size == 2:
        base = "<i2"
    elif field_type == "I" and field_size == 4:
        base = "<i4"
    else:
        raise ValueError(f"Unsupported PCD field {field_name}: {field_type}{field_size}")

    if count == 1:
        return (field_name, base)
    return (field_name, base, (count,))


def _parse_pcd_header(path: Path):
    header_lines = []
    with Path(path).open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD DATA line not found: {path}")
            header_lines.append(line.decode("ascii", errors="replace").strip())
            if line.startswith(b"DATA"):
                break
        data_offset = stream.tell()

    metadata = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        metadata[parts[0]] = parts[1:]
    return metadata, data_offset, len(header_lines)


def read_pcd_xyzi(path: Path) -> np.ndarray:
    pcd_path = Path(path)
    metadata, data_offset, header_line_count = _parse_pcd_header(pcd_path)
    fields = metadata["FIELDS"]
    sizes = [int(value) for value in metadata["SIZE"]]
    types = metadata["TYPE"]
    counts = [int(value) for value in metadata.get("COUNT", ["1"] * len(fields))]
    points_count = int(metadata.get("POINTS", metadata["WIDTH"])[0])
    data_mode = metadata["DATA"][0].lower()

    if not {"x", "y", "z"}.issubset(set(fields)):
        raise ValueError(f"PCD must contain x/y/z fields: {pcd_path}")

    if data_mode == "binary":
        dtype = np.dtype(
            [
                _dtype_for_field(name, field_type, size, count)
                for name, size, field_type, count in zip(fields, sizes, types, counts)
            ]
        )
        data = np.memmap(pcd_path, dtype=dtype, mode="r", offset=data_offset, shape=(points_count,))
        x = np.asarray(data["x"], dtype=np.float32)
        y = np.asarray(data["y"], dtype=np.float32)
        z = np.asarray(data["z"], dtype=np.float32)
        if "intensity" in fields:
            intensity = np.asarray(data["intensity"], dtype=np.float32)
        else:
            intensity = np.zeros(points_count, dtype=np.float32)
        return np.column_stack((x, y, z, intensity)).astype(np.float32, copy=False)

    if data_mode == "ascii":
        body = np.loadtxt(pcd_path, skiprows=header_line_count, dtype=np.float32)
        if body.ndim == 1:
            body = body.reshape((1, -1))
        index = {name: fields.index(name) for name in fields}
        intensity = (
            body[:, index["intensity"]]
            if "intensity" in index
            else np.zeros(body.shape[0], dtype=np.float32)
        )
        return np.column_stack((body[:, index["x"]], body[:, index["y"]], body[:, index["z"]], intensity)).astype(
            np.float32,
            copy=False,
        )

    raise ValueError(f"Unsupported PCD DATA mode {data_mode}: {pcd_path}")


def write_pcd_xyzi(path: Path, points_xyzi: np.ndarray) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points_xyzi, dtype="<f4")
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError("PCD writer expects an Nx4 x/y/z/intensity array")

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with output_path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(points.tobytes())


def load_manifest(scenario_dir: Path) -> dict:
    manifest_path = Path(scenario_dir) / "scenarios.yaml"
    if not manifest_path.exists():
        return {"version": 1, "scenarios": []}
    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream) or {}
    manifest.setdefault("version", 1)
    manifest.setdefault("scenarios", [])
    return manifest


def save_manifest(scenario_dir: Path, manifest: dict) -> None:
    manifest_path = Path(scenario_dir) / "scenarios.yaml"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest["updated_at"] = datetime.now(timezone.utc).isoformat()
    with manifest_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest, stream, sort_keys=False)


def select_scenario(scenario_dir: Path, scenario_name: str) -> dict:
    manifest = load_manifest(scenario_dir)
    scenarios = list(manifest.get("scenarios", []))
    if not scenarios:
        raise RuntimeError(
            f"No mock LiDAR scenarios found in {scenario_dir}. "
            "Run mock_lidar_record_scenario.launch.py and choose a pose in RViz first."
        )

    if scenario_name == "latest":
        return scenarios[-1]

    for scenario in scenarios:
        if scenario.get("name") == scenario_name:
            return scenario
    names = ", ".join(str(item.get("name")) for item in scenarios)
    raise RuntimeError(f"Mock LiDAR scenario '{scenario_name}' not found. Available: {names}")


def create_scenario(
    *,
    map_pcd_path: Path,
    scenario_dir: Path,
    name: str,
    truth_pose: Pose2D,
    crop: CropConfig,
    initial_offset_x: float = DEFAULT_INITIAL_OFFSET_X,
    initial_offset_y: float = DEFAULT_INITIAL_OFFSET_Y,
    initial_offset_yaw: float = math.radians(DEFAULT_INITIAL_OFFSET_YAW_DEG),
) -> dict:
    map_points = read_pcd_xyzi(map_pcd_path)
    scan_points = crop_mock_scan(map_points, truth_pose, crop)
    if len(scan_points) == 0:
        raise RuntimeError(
            f"Mock LiDAR crop for scenario '{name}' produced no points. "
            "Choose a pose inside the PCD map or increase max_range_m."
        )

    scenario_path = Path(scenario_dir) / name
    scan_path = scenario_path / "scan.pcd"
    write_pcd_xyzi(scan_path, scan_points)

    initial_pose = offset_initial_pose(
        truth_pose, dx=initial_offset_x, dy=initial_offset_y, dyaw=initial_offset_yaw
    )
    entry = {
        "name": name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "scan_pcd": str(scan_path.relative_to(scenario_dir)),
        "frame_id": "lidar_top",
        "truth_pose": pose_to_dict(truth_pose),
        "initial_pose": pose_to_dict(initial_pose),
        "initial_offset": {
            "dx": float(initial_offset_x),
            "dy": float(initial_offset_y),
            "dyaw": float(initial_offset_yaw),
        },
        "crop": {
            "min_range_m": float(crop.min_range_m),
            "max_range_m": float(crop.max_range_m),
            "min_elevation_deg": float(crop.min_elevation_deg),
            "max_elevation_deg": float(crop.max_elevation_deg),
            "voxel_leaf_m": float(crop.voxel_leaf_m),
            "points": int(len(scan_points)),
        },
    }

    manifest = load_manifest(scenario_dir)
    manifest["scenarios"] = [
        item for item in manifest.get("scenarios", []) if item.get("name") != name
    ]
    manifest["scenarios"].append(entry)
    save_manifest(scenario_dir, manifest)
    return entry


def _crop_config_from_params(node) -> CropConfig:
    return CropConfig(
        min_range_m=float(node.get_parameter("min_range_m").value),
        max_range_m=float(node.get_parameter("max_range_m").value),
        min_elevation_deg=float(node.get_parameter("min_elevation_deg").value),
        max_elevation_deg=float(node.get_parameter("max_elevation_deg").value),
        voxel_leaf_m=float(node.get_parameter("voxel_leaf_m").value),
    )


def _declare_common_crop_params(node) -> None:
    node.declare_parameter("min_range_m", DEFAULT_MIN_RANGE_M)
    node.declare_parameter("max_range_m", DEFAULT_MAX_RANGE_M)
    node.declare_parameter("min_elevation_deg", DEFAULT_MIN_ELEVATION_DEG)
    node.declare_parameter("max_elevation_deg", DEFAULT_MAX_ELEVATION_DEG)
    node.declare_parameter("voxel_leaf_m", DEFAULT_VOXEL_LEAF_M)
    node.declare_parameter("initial_offset_x", DEFAULT_INITIAL_OFFSET_X)
    node.declare_parameter("initial_offset_y", DEFAULT_INITIAL_OFFSET_Y)
    node.declare_parameter("initial_offset_yaw_deg", DEFAULT_INITIAL_OFFSET_YAW_DEG)


class MockLidarScenarioRecorder:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.node import Node

        class RecorderNode(Node):
            def __init__(inner_self):
                super().__init__("mock_lidar_scenario_recorder")
                map_path = default_map_path()
                inner_self.declare_parameter("map_path", str(map_path))
                inner_self.declare_parameter("map_pcd_path", "")
                inner_self.declare_parameter("scenario_dir", "")
                inner_self.declare_parameter("input_topic", "/initialpose")
                inner_self.declare_parameter("scenario_name", "")
                _declare_common_crop_params(inner_self)

                resolved_map_path = Path(inner_self.get_parameter("map_path").value)
                map_pcd = inner_self.get_parameter("map_pcd_path").value
                scenario_dir = inner_self.get_parameter("scenario_dir").value
                inner_self._map_pcd_path = Path(map_pcd) if map_pcd else resolved_map_path / "pointcloud_map.pcd"
                inner_self._scenario_dir = Path(scenario_dir) if scenario_dir else default_scenario_dir(resolved_map_path)
                inner_self._input_topic = inner_self.get_parameter("input_topic").value
                inner_self.create_subscription(
                    PoseWithCovarianceStamped,
                    inner_self._input_topic,
                    inner_self._on_initialpose,
                    10,
                )
                inner_self.get_logger().info(
                    f"Recording mock LiDAR scenarios from {inner_self._input_topic}; "
                    f"map={inner_self._map_pcd_path}; scenario_dir={inner_self._scenario_dir}"
                )

            def _on_initialpose(inner_self, msg):
                truth_pose = pose_from_msg(msg)
                scenario_name = inner_self.get_parameter("scenario_name").value
                name = scenario_name if scenario_name else f"rviz_{utc_stamp()}"
                crop = _crop_config_from_params(inner_self)
                try:
                    entry = create_scenario(
                        map_pcd_path=inner_self._map_pcd_path,
                        scenario_dir=inner_self._scenario_dir,
                        name=name,
                        truth_pose=truth_pose,
                        crop=crop,
                        initial_offset_x=float(inner_self.get_parameter("initial_offset_x").value),
                        initial_offset_y=float(inner_self.get_parameter("initial_offset_y").value),
                        initial_offset_yaw=math.radians(
                            float(inner_self.get_parameter("initial_offset_yaw_deg").value)
                        ),
                    )
                except Exception as exc:  # noqa: BLE001
                    inner_self.get_logger().error(f"Failed to create mock LiDAR scenario: {exc}")
                    return
                inner_self.get_logger().info(
                    f"Saved scenario {entry['name']} with {entry['crop']['points']} points"
                )

        self._rclpy = rclpy
        self.node = RecorderNode()


class MockLidarPublisher:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField

        class PublisherNode(Node):
            def __init__(inner_self):
                super().__init__("mock_lidar_publisher")
                map_path = default_map_path()
                inner_self.declare_parameter("scenario_dir", str(default_scenario_dir(map_path)))
                inner_self.declare_parameter("scenario", "latest")
                inner_self.declare_parameter("publish_rate_hz", 10.0)
                inner_self.declare_parameter("initial_pose_rate_hz", 20.0)
                inner_self.declare_parameter("pointcloud_topic", DEFAULT_SCAN_TOPIC)
                inner_self.declare_parameter("initial_pose_topic", DEFAULT_INITIAL_POSE_TOPIC)
                inner_self.declare_parameter("frame_id", "lidar_top")
                inner_self.declare_parameter("map_frame", "map")

                scenario_dir = Path(inner_self.get_parameter("scenario_dir").value)
                scenario_name = inner_self.get_parameter("scenario").value
                inner_self._scenario_dir = scenario_dir
                inner_self._scenario = select_scenario(scenario_dir, scenario_name)
                scan_path = scenario_dir / inner_self._scenario["scan_pcd"]
                points = read_pcd_xyzi(scan_path)
                inner_self._cloud_msg = inner_self._make_cloud_msg(points)
                inner_self._initial_pose = pose_from_dict(inner_self._scenario["initial_pose"])
                inner_self._map_frame = inner_self.get_parameter("map_frame").value

                sensor_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                )
                inner_self._cloud_pub = inner_self.create_publisher(
                    PointCloud2, inner_self.get_parameter("pointcloud_topic").value, sensor_qos
                )
                inner_self._pose_pub = inner_self.create_publisher(
                    PoseWithCovarianceStamped,
                    inner_self.get_parameter("initial_pose_topic").value,
                    10,
                )

                scan_rate = max(float(inner_self.get_parameter("publish_rate_hz").value), 0.1)
                pose_rate = max(float(inner_self.get_parameter("initial_pose_rate_hz").value), 0.1)
                inner_self.create_timer(1.0 / scan_rate, inner_self._on_scan_timer)
                inner_self.create_timer(1.0 / pose_rate, inner_self._on_pose_timer)
                inner_self.get_logger().info(
                    f"Publishing mock LiDAR scenario {inner_self._scenario['name']} "
                    f"({len(points)} points) from {scan_path}"
                )

            def _make_cloud_msg(inner_self, points):
                msg = PointCloud2()
                msg.header.frame_id = inner_self.get_parameter("frame_id").value
                msg.height = 1
                msg.width = int(len(points))
                msg.fields = [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                msg.is_bigendian = False
                msg.point_step = 16
                msg.row_step = msg.point_step * msg.width
                msg.data = np.asarray(points, dtype="<f4").tobytes()
                msg.is_dense = True
                return msg

            def _make_initial_pose_msg(inner_self, stamp):
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = inner_self._map_frame
                msg.pose.pose.position.x = float(inner_self._initial_pose.x)
                msg.pose.pose.position.y = float(inner_self._initial_pose.y)
                msg.pose.pose.position.z = float(inner_self._initial_pose.z)
                msg.pose.pose.orientation = yaw_to_quaternion(inner_self._initial_pose.yaw)
                msg.pose.covariance = covariance_6d()
                return msg

            def _publish_initial_pose(inner_self, stamp):
                inner_self._pose_pub.publish(inner_self._make_initial_pose_msg(stamp))

            def _on_scan_timer(inner_self):
                stamp = inner_self.get_clock().now().to_msg()
                inner_self._publish_initial_pose(stamp)
                msg = copy.copy(inner_self._cloud_msg)
                msg.header.stamp = stamp
                inner_self._cloud_pub.publish(msg)

            def _on_pose_timer(inner_self):
                inner_self._publish_initial_pose(inner_self.get_clock().now().to_msg())

        self._rclpy = rclpy
        self.node = PublisherNode()


def recorder_main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    wrapper = MockLidarScenarioRecorder()
    try:
        rclpy.spin(wrapper.node)
    except ExternalShutdownException:
        pass
    finally:
        wrapper.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def publisher_main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    wrapper = MockLidarPublisher()
    try:
        rclpy.spin(wrapper.node)
    except ExternalShutdownException:
        pass
    finally:
        wrapper.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def generate_main(argv=None):
    parser = argparse.ArgumentParser(description="Generate a mock LiDAR scan scenario from a map PCD.")
    parser.add_argument("--map-path", default=str(default_map_path()))
    parser.add_argument("--scenario-dir", default="")
    parser.add_argument("--name", default="")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=None, help="Yaw in radians.")
    parser.add_argument("--yaw-deg", type=float, default=None, help="Yaw in degrees.")
    parser.add_argument("--min-range", type=float, default=DEFAULT_MIN_RANGE_M)
    parser.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE_M)
    parser.add_argument("--voxel-leaf", type=float, default=DEFAULT_VOXEL_LEAF_M)
    parser.add_argument("--initial-offset-x", type=float, default=DEFAULT_INITIAL_OFFSET_X)
    parser.add_argument("--initial-offset-y", type=float, default=DEFAULT_INITIAL_OFFSET_Y)
    parser.add_argument("--initial-offset-yaw-deg", type=float, default=DEFAULT_INITIAL_OFFSET_YAW_DEG)
    args = parser.parse_args(argv)

    map_path = Path(args.map_path)
    scenario_dir = Path(args.scenario_dir) if args.scenario_dir else default_scenario_dir(map_path)
    yaw = math.radians(args.yaw_deg) if args.yaw_deg is not None else args.yaw
    if yaw is None:
        parser.error("Provide either --yaw or --yaw-deg")

    name = args.name if args.name else f"cli_{utc_stamp()}"
    entry = create_scenario(
        map_pcd_path=map_path / "pointcloud_map.pcd",
        scenario_dir=scenario_dir,
        name=name,
        truth_pose=Pose2D(args.x, args.y, args.z, yaw),
        crop=CropConfig(
            min_range_m=args.min_range,
            max_range_m=args.max_range,
            voxel_leaf_m=args.voxel_leaf,
        ),
        initial_offset_x=args.initial_offset_x,
        initial_offset_y=args.initial_offset_y,
        initial_offset_yaw=math.radians(args.initial_offset_yaw_deg),
    )
    print(
        f"Saved {entry['name']} to {scenario_dir} "
        f"with {entry['crop']['points']} points"
    )


if __name__ == "__main__":
    generate_main()
