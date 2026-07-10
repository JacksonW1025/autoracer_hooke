from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    lidar_sensor_model = LaunchConfiguration("lidar_sensor_model")
    rviz_config = LaunchConfiguration("rviz_config")

    vehicle_model = PathJoinSubstitution(
        [FindPackageShare("hooke2_description"), "urdf", "vehicle.xacro"]
    )
    default_rviz_config = _pkg_file("autoracer_bringup", "rviz", "lidar_vehicle.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.120"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.130"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("lidar_sensor_model", default_value="Pandar40P"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "sensing.launch.py")
                ),
                launch_arguments={
                    "launch_lidar": "true",
                    "launch_fixposition": "false",
                    "lidar_host_ip": lidar_host_ip,
                    "lidar_sensor_ip": lidar_sensor_ip,
                    "lidar_data_port": lidar_data_port,
                    "sensor_model": lidar_sensor_model,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
