from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    can_channel_id = LaunchConfiguration("can_channel_id")
    can_baudrate = LaunchConfiguration("can_baudrate")

    hooke2_param = PathJoinSubstitution(
        [get_package_share_directory("autoracer_bringup"), "config", "hooke2", "hooke2.param.yaml"]
    )
    vehicle_info = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "vehicle_info.param.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_channel_id", default_value="0"),
            DeclareLaunchArgument("can_baudrate", default_value="500000"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("hooke2_interface"),
                            "launch",
                            "hooke2_interface.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "hooke2_param_path": hooke2_param,
                    "vehicle_info_param_file": vehicle_info,
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("can_driver"),
                            "launch",
                            "can_driver_node_socket_can0.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "channel_id": can_channel_id,
                    "baudrate": can_baudrate,
                    "can_tx_topic": "/can_tx_to_autoware",
                    "can_rx_topic": "/can_rx_from_autoware",
                }.items(),
            ),
        ]
    )
