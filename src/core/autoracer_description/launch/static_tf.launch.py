from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _as_float(value):
    return str(float(value))


def _launch_setup(context, *args, **kwargs):
    config_path = Path(LaunchConfiguration("extrinsics_file").perform(context))
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    nodes = []
    for item in data.get("transforms", []):
        translation = item.get("translation", {})
        rotation = item.get("rotation_rpy", {})
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{item['parent']}_to_{item['child']}",
                arguments=[
                    "--x",
                    _as_float(translation.get("x", 0.0)),
                    "--y",
                    _as_float(translation.get("y", 0.0)),
                    "--z",
                    _as_float(translation.get("z", 0.0)),
                    "--roll",
                    _as_float(rotation.get("roll", 0.0)),
                    "--pitch",
                    _as_float(rotation.get("pitch", 0.0)),
                    "--yaw",
                    _as_float(rotation.get("yaw", 0.0)),
                    "--frame-id",
                    item["parent"],
                    "--child-frame-id",
                    item["child"],
                ],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("autoracer_description"))
        / "config"
        / "hooke2_sensor_extrinsics.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("extrinsics_file", default_value=str(default_config)),
            OpaqueFunction(function=_launch_setup),
        ]
    )

