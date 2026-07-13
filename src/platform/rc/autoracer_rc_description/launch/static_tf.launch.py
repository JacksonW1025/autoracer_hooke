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
        translation = item["translation"]
        rotation = item["rotation_rpy"]
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{item['parent']}_to_{item['child']}",
                arguments=[
                    "--x",
                    _as_float(translation["x"]),
                    "--y",
                    _as_float(translation["y"]),
                    "--z",
                    _as_float(translation["z"]),
                    "--roll",
                    _as_float(rotation["roll"]),
                    "--pitch",
                    _as_float(rotation["pitch"]),
                    "--yaw",
                    _as_float(rotation["yaw"]),
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
        Path(get_package_share_directory("autoracer_rc_description"))
        / "config"
        / "sensor_extrinsics.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("extrinsics_file", default_value=str(default_config)),
            OpaqueFunction(function=_launch_setup),
        ]
    )
