import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


_DATATYPES = {
    PointField.INT8: np.dtype("i1"),
    PointField.UINT8: np.dtype("u1"),
    PointField.INT16: np.dtype("i2"),
    PointField.UINT16: np.dtype("u2"),
    PointField.INT32: np.dtype("i4"),
    PointField.UINT32: np.dtype("u4"),
    PointField.FLOAT32: np.dtype("f4"),
    PointField.FLOAT64: np.dtype("f8"),
}


class PointCloudXyziToXyzirc(Node):
    def __init__(self):
        super().__init__("pointcloud_xyzi_to_xyzirc")
        self.declare_parameter("input_topic", "/sensing/lidar/concatenated/pointcloud")
        self.declare_parameter(
            "output_topic", "/sensing/lidar/concatenated/pointcloud_xyzirc"
        )

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._pub = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        count = msg.width * msg.height
        if count == 0:
            self._pub.publish(self._empty_output(msg))
            return

        fields = {field.name: field for field in msg.fields}
        try:
            x = self._read_field(msg, fields, "x", count).astype(np.float32, copy=False)
            y = self._read_field(msg, fields, "y", count).astype(np.float32, copy=False)
            z = self._read_field(msg, fields, "z", count).astype(np.float32, copy=False)
        except (KeyError, ValueError) as exc:
            self.get_logger().error(f"invalid PointCloud2 XYZ layout: {exc}")
            return

        out_dtype = np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("intensity", "u1"),
                ("return_type", "u1"),
                ("channel", "<u2"),
            ]
        )
        out = np.empty(count, dtype=out_dtype)
        out["x"] = x
        out["y"] = y
        out["z"] = z
        out["intensity"] = self._intensity(msg, fields, count)
        out["return_type"] = 1
        out["channel"] = 0

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = msg.height
        cloud.width = msg.width
        cloud.fields = self._xyzirc_fields()
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = out.tobytes()
        cloud.is_dense = msg.is_dense
        self._pub.publish(cloud)

    def _empty_output(self, msg: PointCloud2) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = msg.height
        cloud.width = msg.width
        cloud.fields = self._xyzirc_fields()
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = b""
        cloud.is_dense = msg.is_dense
        return cloud

    def _read_field(self, msg: PointCloud2, fields, name: str, count: int) -> np.ndarray:
        field = fields[name]
        if field.count != 1:
            raise ValueError(f"{name} count={field.count}, expected 1")
        dtype = _DATATYPES.get(field.datatype)
        if dtype is None:
            raise ValueError(f"{name} unsupported datatype={field.datatype}")
        dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
        return np.ndarray(
            shape=(count,),
            dtype=dtype,
            buffer=memoryview(msg.data),
            offset=field.offset,
            strides=(msg.point_step,),
        )

    def _intensity(self, msg: PointCloud2, fields, count: int) -> np.ndarray:
        if "intensity" not in fields:
            return np.zeros(count, dtype=np.uint8)
        values = self._read_field(msg, fields, "intensity", count)
        return np.rint(np.clip(values.astype(np.float32, copy=False), 0.0, 255.0)).astype(
            np.uint8
        )

    def _xyzirc_fields(self):
        return [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudXyziToXyzirc()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
