from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    launch_static_tf = LaunchConfiguration("launch_static_tf")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_fixposition = LaunchConfiguration("launch_fixposition")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    lidar_sensor_model = LaunchConfiguration("lidar_sensor_model")
    fixposition_stream = LaunchConfiguration("fixposition_stream")
    can_channel_id = LaunchConfiguration("can_channel_id")
    can_baudrate = LaunchConfiguration("can_baudrate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_static_tf", default_value="true"),
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_fixposition", default_value="true"),
            DeclareLaunchArgument("launch_vehicle", default_value="true"),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.120"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.130"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("lidar_sensor_model", default_value="Pandar64"),
            DeclareLaunchArgument(
                "fixposition_stream", default_value="tcpcli://192.168.1.200:21000"
            ),
            DeclareLaunchArgument("can_channel_id", default_value="0"),
            DeclareLaunchArgument("can_baudrate", default_value="500000"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_description", "launch", "static_tf.launch.py")
                ),
                condition=IfCondition(launch_static_tf),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "sensing.launch.py")
                ),
                launch_arguments={
                    "launch_lidar": launch_lidar,
                    "launch_fixposition": launch_fixposition,
                    "lidar_host_ip": lidar_host_ip,
                    "lidar_sensor_ip": lidar_sensor_ip,
                    "lidar_data_port": lidar_data_port,
                    "sensor_model": lidar_sensor_model,
                    "fixposition_stream": fixposition_stream,
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
        ]
    )
