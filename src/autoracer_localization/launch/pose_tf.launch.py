from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autoracer_localization",
                executable="pose_tf_broadcaster",
                name="pose_tf_broadcaster",
                output="screen",
            )
        ]
    )

