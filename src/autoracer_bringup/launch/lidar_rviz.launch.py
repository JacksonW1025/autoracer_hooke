from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    extrinsics_file = LaunchConfiguration("extrinsics_file")
    lidar_driver = LaunchConfiguration("lidar_driver")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    lidar_sensor_model = LaunchConfiguration("lidar_sensor_model")
    rviz_config = LaunchConfiguration("rviz_config")

    default_rviz_config = _pkg_file("autoracer_bringup", "rviz", "lidar_pointcloud.rviz")
    default_extrinsics_file = _pkg_file(
        "autoracer_description", "config", "rc_sensor_extrinsics.yaml"
    )
    default_lidar_param_file = _pkg_file(
        "autoracer_bringup", "config", "rc", "lslidar_cx.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("extrinsics_file", default_value=default_extrinsics_file),
            DeclareLaunchArgument("lidar_driver", default_value="lslidar_c32"),
            DeclareLaunchArgument("lidar_param_file", default_value=default_lidar_param_file),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.102"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.200"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("lidar_sensor_model", default_value="C32"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_description", "launch", "static_tf.launch.py")
                ),
                launch_arguments={"extrinsics_file": extrinsics_file}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "sensing.launch.py")
                ),
                launch_arguments={
                    "launch_lidar": "true",
                    "launch_fixposition": "false",
                    "lidar_driver": lidar_driver,
                    "lidar_param_file": lidar_param_file,
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
