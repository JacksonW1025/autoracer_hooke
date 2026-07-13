from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory("autoracer_description"))
    model_path = share_dir / "urdf" / "hooke2_sensor_mounts.urdf.xacro"
    rviz_config = share_dir / "rviz" / "hooke2_sensor_mounts.rviz"

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(["xacro ", str(model_path)]),
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
            ),
        ]
    )
