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
    launch_static_tf = LaunchConfiguration("launch_static_tf")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_fixposition = LaunchConfiguration("launch_fixposition")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_rviz = LaunchConfiguration("launch_rviz")
    extrinsics_file = LaunchConfiguration("extrinsics_file")
    lidar_driver = LaunchConfiguration("lidar_driver")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    lidar_sensor_model = LaunchConfiguration("lidar_sensor_model")
    fixposition_stream = LaunchConfiguration("fixposition_stream")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    max_steer_rad = LaunchConfiguration("max_steer_rad")
    rviz_config = LaunchConfiguration("rviz_config")

    default_rviz_config = _pkg_file("autoracer_bringup", "rviz", "lidar_pointcloud.rviz")
    default_extrinsics_file = _pkg_file(
        "autoracer_description", "config", "rc_sensor_extrinsics.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_static_tf", default_value="true"),
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_fixposition", default_value="false"),
            DeclareLaunchArgument("launch_vehicle", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("extrinsics_file", default_value=default_extrinsics_file),
            DeclareLaunchArgument("lidar_driver", default_value="lslidar_c32"),
            DeclareLaunchArgument(
                "lidar_param_file",
                default_value=_pkg_file(
                    "autoracer_bringup", "config", "rc", "lslidar_cx.yaml"
                ),
            ),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.120"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.200"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("lidar_sensor_model", default_value="C32"),
            DeclareLaunchArgument(
                "fixposition_stream", default_value="tcpcli://192.168.1.200:21000"
            ),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("serial_baudrate", default_value="115200"),
            DeclareLaunchArgument("wheel_base_m", default_value="0.6"),
            DeclareLaunchArgument("max_speed_mps", default_value="3.0"),
            DeclareLaunchArgument("max_steer_rad", default_value="0.262"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_description", "launch", "static_tf.launch.py")
                ),
                launch_arguments={"extrinsics_file": extrinsics_file}.items(),
                condition=IfCondition(launch_static_tf),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "sensing.launch.py")
                ),
                launch_arguments={
                    "launch_lidar": launch_lidar,
                    "launch_fixposition": launch_fixposition,
                    "lidar_driver": lidar_driver,
                    "lidar_param_file": lidar_param_file,
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
                    "serial_port": serial_port,
                    "serial_baudrate": serial_baudrate,
                    "wheel_base_m": wheel_base_m,
                    "max_speed_mps": max_speed_mps,
                    "max_steer_rad": max_steer_rad,
                }.items(),
                condition=IfCondition(launch_vehicle),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
