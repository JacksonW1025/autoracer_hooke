from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_model = PathJoinSubstitution(
        [FindPackageShare("hooke2_description"), "urdf", "vehicle.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("hooke2_description"), "rviz", "vehicle.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [FindExecutable(name="xacro"), " ", vehicle_model]
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
