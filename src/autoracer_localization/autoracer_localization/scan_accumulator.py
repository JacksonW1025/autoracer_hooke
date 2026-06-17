import math
from collections import deque
from dataclasses import dataclass
from typing import Iterable

import rclpy
import sensor_msgs_py.point_cloud2 as point_cloud2
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


PointRow = tuple[float, float, float, float]


@dataclass(frozen=True)
class PlanarOdomState:
    stamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class AccumulatedFrame:
    stamp_sec: float
    odom_state: PlanarOdomState
    points: tuple[PointRow, ...]


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_body_twist(
    state: PlanarOdomState,
    *,
    stamp_sec: float,
    vx_mps: float,
    vy_mps: float,
    wz_radps: float,
) -> PlanarOdomState:
    dt = float(stamp_sec) - float(state.stamp_sec)
    if dt <= 0.0:
        return state
    cos_yaw = math.cos(state.yaw)
    sin_yaw = math.sin(state.yaw)
    dx = (cos_yaw * float(vx_mps) - sin_yaw * float(vy_mps)) * dt
    dy = (sin_yaw * float(vx_mps) + cos_yaw * float(vy_mps)) * dt
    return PlanarOdomState(
        stamp_sec=float(stamp_sec),
        x=state.x + dx,
        y=state.y + dy,
        yaw=_normalize_angle(state.yaw + float(wz_radps) * dt),
    )


def transform_points_between_states(
    points: Iterable[PointRow],
    source: PlanarOdomState,
    target: PlanarOdomState,
) -> list[PointRow]:
    source_cos = math.cos(source.yaw)
    source_sin = math.sin(source.yaw)
    target_cos = math.cos(target.yaw)
    target_sin = math.sin(target.yaw)

    transformed: list[PointRow] = []
    for x, y, z, intensity in points:
        map_x = source.x + source_cos * x - source_sin * y
        map_y = source.y + source_sin * x + source_cos * y
        dx = map_x - target.x
        dy = map_y - target.y
        target_x = target_cos * dx + target_sin * dy
        target_y = -target_sin * dx + target_cos * dy
        transformed.append((target_x, target_y, z, intensity))
    return transformed


def voxel_downsample_points(points: Iterable[PointRow], voxel_size_m: float) -> list[PointRow]:
    if voxel_size_m <= 0.0:
        return list(points)
    selected: dict[tuple[int, int, int], PointRow] = {}
    inv = 1.0 / float(voxel_size_m)
    for point in points:
        x, y, z, _ = point
        key = (math.floor(x * inv), math.floor(y * inv), math.floor(z * inv))
        selected.setdefault(key, point)
    return list(selected.values())


class SlidingPointCloudAccumulator:
    def __init__(self, *, max_frames: int, max_age_sec: float, voxel_size_m: float):
        if max_frames <= 0:
            raise ValueError("max_frames must be positive")
        if max_age_sec < 0.0:
            raise ValueError("max_age_sec must be non-negative")
        self._max_frames = int(max_frames)
        self._max_age_sec = float(max_age_sec)
        self._voxel_size_m = float(voxel_size_m)
        self._frames: deque[AccumulatedFrame] = deque()

    def add_frame(
        self,
        *,
        stamp_sec: float,
        odom_state: PlanarOdomState,
        points: Iterable[PointRow],
    ) -> None:
        self._frames.append(
            AccumulatedFrame(
                stamp_sec=float(stamp_sec),
                odom_state=odom_state,
                points=tuple(points),
            )
        )
        while len(self._frames) > self._max_frames:
            self._frames.popleft()

    def accumulated_points(
        self,
        *,
        current_stamp_sec: float,
        current_odom_state: PlanarOdomState,
        max_frames: int | None = None,
        max_age_sec: float | None = None,
    ) -> list[PointRow]:
        query_max_frames = self._max_frames if max_frames is None else int(max_frames)
        query_max_age_sec = self._max_age_sec if max_age_sec is None else float(max_age_sec)
        if query_max_frames <= 0:
            raise ValueError("max_frames must be positive")
        if query_max_age_sec < 0.0:
            raise ValueError("max_age_sec must be non-negative")

        retained_frames = [
            frame
            for frame in self._frames
            if float(current_stamp_sec) - frame.stamp_sec <= self._max_age_sec + 1e-9
        ]
        self._frames = deque(retained_frames)
        selected_frames = [
            frame
            for frame in self._frames
            if float(current_stamp_sec) - frame.stamp_sec <= query_max_age_sec + 1e-9
        ]
        if len(selected_frames) > query_max_frames:
            selected_frames = selected_frames[-query_max_frames:]

        transformed: list[PointRow] = []
        for frame in selected_frames:
            transformed.extend(
                transform_points_between_states(
                    frame.points,
                    frame.odom_state,
                    current_odom_state,
                )
            )
        return voxel_downsample_points(transformed, self._voxel_size_m)


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _read_xyzi_points(msg: PointCloud2) -> list[PointRow]:
    field_names = [field.name for field in msg.fields]
    if "intensity" in field_names:
        rows = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True,
        )
        return [(float(x), float(y), float(z), float(intensity)) for x, y, z, intensity in rows]
    rows = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return [(float(x), float(y), float(z), 0.0) for x, y, z in rows]


def _make_xyzi_cloud(template: PointCloud2, points: Iterable[PointRow]) -> PointCloud2:
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud = point_cloud2.create_cloud(template.header, fields, list(points))
    cloud.header.frame_id = template.header.frame_id
    return cloud


class ScanAccumulator(Node):
    def __init__(self):
        super().__init__("scan_accumulator")
        self.declare_parameter(
            "input_pointcloud_topic",
            "/sensing/lidar/concatenated/pointcloud_downsampled",
        )
        self.declare_parameter(
            "output_pointcloud_topic",
            "/sensing/lidar/concatenated/pointcloud_accumulated",
        )
        self.declare_parameter(
            "twist_topic",
            "/sensing/gyro_odometer/twist_with_covariance",
        )
        self.declare_parameter("max_frames", 8)
        self.declare_parameter("max_age_sec", 0.8)
        self.declare_parameter("voxel_size_m", 0.2)
        self.declare_parameter("min_points_to_publish", 1)
        self.declare_parameter("max_twist_dt_sec", 0.2)
        self.declare_parameter("adaptive_min_input_points", 0)
        self.declare_parameter("adaptive_max_frames", 8)
        self.declare_parameter("adaptive_max_age_sec", 0.8)

        base_max_frames = int(self.get_parameter("max_frames").value)
        base_max_age_sec = float(self.get_parameter("max_age_sec").value)
        adaptive_max_frames = int(self.get_parameter("adaptive_max_frames").value)
        adaptive_max_age_sec = float(self.get_parameter("adaptive_max_age_sec").value)
        self._base_max_frames = base_max_frames
        self._base_max_age_sec = base_max_age_sec
        self._adaptive_min_input_points = int(
            self.get_parameter("adaptive_min_input_points").value
        )
        adaptive_enabled = self._adaptive_min_input_points > 0
        self._adaptive_max_frames = max(base_max_frames, adaptive_max_frames)
        self._adaptive_max_age_sec = max(base_max_age_sec, adaptive_max_age_sec)
        retention_max_frames = self._adaptive_max_frames if adaptive_enabled else base_max_frames
        retention_max_age_sec = self._adaptive_max_age_sec if adaptive_enabled else base_max_age_sec
        self._accumulator = SlidingPointCloudAccumulator(
            max_frames=retention_max_frames,
            max_age_sec=retention_max_age_sec,
            voxel_size_m=float(self.get_parameter("voxel_size_m").value),
        )
        self._min_points_to_publish = int(self.get_parameter("min_points_to_publish").value)
        self._max_twist_dt_sec = float(self.get_parameter("max_twist_dt_sec").value)
        self._odom_state: PlanarOdomState | None = None
        self._last_twist_stamp_sec: float | None = None

        output_topic = str(self.get_parameter("output_pointcloud_topic").value)
        input_topic = str(self.get_parameter("input_pointcloud_topic").value)
        twist_topic = str(self.get_parameter("twist_topic").value)
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, input_topic, self._on_pointcloud, qos_profile_sensor_data)
        self.create_subscription(TwistWithCovarianceStamped, twist_topic, self._on_twist, 50)

    def _accumulation_policy(self, input_point_count: int) -> tuple[int, float]:
        if (
            self._adaptive_min_input_points > 0
            and input_point_count < self._adaptive_min_input_points
        ):
            return self._adaptive_max_frames, self._adaptive_max_age_sec
        return self._base_max_frames, self._base_max_age_sec

    def _on_twist(self, msg: TwistWithCovarianceStamped) -> None:
        stamp_sec = _stamp_to_sec(msg.header.stamp)
        twist = msg.twist.twist
        if self._odom_state is None:
            self._odom_state = PlanarOdomState(stamp_sec=stamp_sec, x=0.0, y=0.0, yaw=0.0)
            self._last_twist_stamp_sec = stamp_sec
            return
        if self._last_twist_stamp_sec is not None:
            dt = stamp_sec - self._last_twist_stamp_sec
            if dt > self._max_twist_dt_sec:
                self._odom_state = PlanarOdomState(
                    stamp_sec=stamp_sec,
                    x=self._odom_state.x,
                    y=self._odom_state.y,
                    yaw=self._odom_state.yaw,
                )
                self._last_twist_stamp_sec = stamp_sec
                return
        self._odom_state = integrate_body_twist(
            self._odom_state,
            stamp_sec=stamp_sec,
            vx_mps=twist.linear.x,
            vy_mps=twist.linear.y,
            wz_radps=twist.angular.z,
        )
        self._last_twist_stamp_sec = stamp_sec

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        stamp_sec = _stamp_to_sec(msg.header.stamp)
        odom_state = self._odom_state or PlanarOdomState(
            stamp_sec=stamp_sec,
            x=0.0,
            y=0.0,
            yaw=0.0,
        )
        points = _read_xyzi_points(msg)
        self._accumulator.add_frame(stamp_sec=stamp_sec, odom_state=odom_state, points=points)
        max_frames, max_age_sec = self._accumulation_policy(len(points))
        accumulated = self._accumulator.accumulated_points(
            current_stamp_sec=stamp_sec,
            current_odom_state=odom_state,
            max_frames=max_frames,
            max_age_sec=max_age_sec,
        )
        if len(accumulated) < self._min_points_to_publish:
            return
        self._publisher.publish(_make_xyzi_cloud(msg, accumulated))


def main(args=None):
    rclpy.init(args=args)
    node = ScanAccumulator()
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
