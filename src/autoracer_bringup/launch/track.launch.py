from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    launch_sensing = LaunchConfiguration("launch_sensing")
    launch_localization = LaunchConfiguration("launch_localization")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_rviz = LaunchConfiguration("launch_rviz")
    enable_drive_commands = LaunchConfiguration("enable_drive_commands")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    can_channel_id = LaunchConfiguration("can_channel_id")
    can_baudrate = LaunchConfiguration("can_baudrate")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    lidar_sensor_model = LaunchConfiguration("lidar_sensor_model")
    fixposition_stream = LaunchConfiguration("fixposition_stream")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path"),
            DeclareLaunchArgument("launch_sensing", default_value="true"),
            DeclareLaunchArgument("launch_localization", default_value="true"),
            DeclareLaunchArgument("launch_vehicle", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("enable_drive_commands", default_value="false"),
            DeclareLaunchArgument("max_speed_mps", default_value="1.5"),
            DeclareLaunchArgument("can_channel_id", default_value="0"),
            DeclareLaunchArgument("can_baudrate", default_value="500000"),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.120"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.130"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("lidar_sensor_model", default_value="Pandar64"),
            DeclareLaunchArgument(
                "fixposition_stream", default_value="tcpcli://192.168.1.200:21000"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_description", "launch", "static_tf.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "sensing.launch.py")
                ),
                launch_arguments={
                    "lidar_host_ip": lidar_host_ip,
                    "lidar_sensor_ip": lidar_sensor_ip,
                    "lidar_data_port": lidar_data_port,
                    "sensor_model": lidar_sensor_model,
                    "fixposition_stream": fixposition_stream,
                }.items(),
                condition=IfCondition(launch_sensing),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "localization.launch.py")
                ),
                launch_arguments={"map_path": map_path}.items(),
                condition=IfCondition(launch_localization),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_planning", "launch", "planning.launch.py")
                ),
                launch_arguments={
                    "map_path": map_path,
                    "max_speed_mps": max_speed_mps,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_control", "launch", "control.launch.py")
                ),
                launch_arguments={"max_speed_mps": max_speed_mps}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_safety", "launch", "safety.launch.py")
                ),
                launch_arguments={
                    "enable_drive_commands": enable_drive_commands,
                    "max_speed_mps": max_speed_mps,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "vehicle.launch.py")
                ),
                launch_arguments={
                    "can_channel_id": can_channel_id,
                    "can_baudrate": can_baudrate,
                }.items(),
                condition=IfCondition(launch_vehicle),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
