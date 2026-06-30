from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    max_steer_rad = LaunchConfiguration("max_steer_rad")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("serial_baudrate", default_value="115200"),
            DeclareLaunchArgument("wheel_base_m", default_value="0.6"),
            DeclareLaunchArgument("max_speed_mps", default_value="3.0"),
            DeclareLaunchArgument("max_steer_rad", default_value="0.262"),
            Node(
                package="autoracer_vehicle_interface",
                executable="rc_serial_interface",
                name="rc_serial_interface",
                output="screen",
                parameters=[
                    {
                        "port": serial_port,
                        "baudrate": serial_baudrate,
                        "wheel_base_m": wheel_base_m,
                        "max_speed_mps": max_speed_mps,
                        "max_steer_rad": max_steer_rad,
                    }
                ],
            ),
        ]
    )
