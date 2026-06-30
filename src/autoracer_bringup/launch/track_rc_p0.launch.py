import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    default_map_path = os.path.join(os.getcwd(), "maps", "whale_map_20251107")

    map_path = LaunchConfiguration("map_path")
    launch_sensing = LaunchConfiguration("launch_sensing")
    launch_localization = LaunchConfiguration("launch_localization")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_drive_commands = LaunchConfiguration("enable_drive_commands")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    manual_seed_x = LaunchConfiguration("manual_seed_x")
    manual_seed_y = LaunchConfiguration("manual_seed_y")
    manual_seed_z = LaunchConfiguration("manual_seed_z")
    manual_seed_yaw = LaunchConfiguration("manual_seed_yaw")
    manual_seed_xy_variance = LaunchConfiguration("manual_seed_xy_variance")
    manual_seed_z_variance = LaunchConfiguration("manual_seed_z_variance")
    manual_seed_yaw_variance = LaunchConfiguration("manual_seed_yaw_variance")

    default_rviz_config = _pkg_file("autoracer_bringup", "rviz", "lidar_pointcloud.rviz")
    rc_extrinsics_file = _pkg_file(
        "autoracer_description", "config", "rc_sensor_extrinsics.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=EnvironmentVariable("MAP_PATH", default_value=default_map_path),
            ),
            DeclareLaunchArgument("launch_sensing", default_value="true"),
            DeclareLaunchArgument("launch_localization", default_value="true"),
            DeclareLaunchArgument("launch_vehicle", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("enable_drive_commands", default_value="false"),
            DeclareLaunchArgument("max_speed_mps", default_value="3.0"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("serial_baudrate", default_value="115200"),
            DeclareLaunchArgument(
                "lidar_host_ip",
                default_value=EnvironmentVariable("LIDAR_HOST_IP", default_value=""),
            ),
            DeclareLaunchArgument(
                "lidar_param_file",
                default_value=_pkg_file("autoracer_bringup", "config", "rc", "lslidar_cx.yaml"),
            ),
            DeclareLaunchArgument("manual_seed_x", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_y", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_z", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_yaw", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_xy_variance", default_value="1.0"),
            DeclareLaunchArgument("manual_seed_z_variance", default_value="0.25"),
            DeclareLaunchArgument("manual_seed_yaw_variance", default_value="0.03"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "track.launch.py")
                ),
                launch_arguments={
                    "map_path": map_path,
                    "launch_sensing": launch_sensing,
                    "launch_localization": launch_localization,
                    "launch_vehicle": launch_vehicle,
                    "launch_rviz": launch_rviz,
                    "rviz_config": rviz_config,
                    "extrinsics_file": rc_extrinsics_file,
                    "enable_drive_commands": enable_drive_commands,
                    "max_speed_mps": max_speed_mps,
                    "serial_port": serial_port,
                    "serial_baudrate": serial_baudrate,
                    "wheel_base_m": "0.6",
                    "max_steer_rad": "0.262",
                    "lidar_driver": "lslidar_c32",
                    "lidar_param_file": lidar_param_file,
                    "lidar_host_ip": lidar_host_ip,
                    "lidar_sensor_ip": "192.168.1.200",
                    "lidar_data_port": "2368",
                    "lidar_sensor_model": "C32",
                    "launch_fixposition": "false",
                    "launch_fixposition_seed": "false",
                    "launch_manual_seed": "true",
                    "manual_seed_input_topic": "/initialpose",
                    "manual_seed_require_input_pose": "true",
                    "manual_seed_x": manual_seed_x,
                    "manual_seed_y": manual_seed_y,
                    "manual_seed_z": manual_seed_z,
                    "manual_seed_yaw": manual_seed_yaw,
                    "manual_seed_xy_variance": manual_seed_xy_variance,
                    "manual_seed_z_variance": manual_seed_z_variance,
                    "manual_seed_yaw_variance": manual_seed_yaw_variance,
                }.items(),
            ),
        ]
    )
