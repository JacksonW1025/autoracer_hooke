import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _pkg_file(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    default_map_path = os.path.join(os.getcwd(), "maps", "whale_map_20251107")
    map_path = LaunchConfiguration("map_path")
    launch_sensing = LaunchConfiguration("launch_sensing")
    launch_imu = LaunchConfiguration("launch_imu")
    launch_localization = LaunchConfiguration("launch_localization")
    launch_planning = LaunchConfiguration("launch_planning")
    launch_control = LaunchConfiguration("launch_control")
    launch_safety = LaunchConfiguration("launch_safety")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    extrinsics_file = LaunchConfiguration("extrinsics_file")
    enable_drive_commands = LaunchConfiguration("enable_drive_commands")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    control_min_lookahead_m = LaunchConfiguration("control_min_lookahead_m")
    control_lookahead_gain = LaunchConfiguration("control_lookahead_gain")
    control_goal_tolerance_m = LaunchConfiguration("control_goal_tolerance_m")
    control_max_steer_rate_radps = LaunchConfiguration("control_max_steer_rate_radps")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    max_steer_rad = LaunchConfiguration("max_steer_rad")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    lidar_driver = LaunchConfiguration("lidar_driver")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    lidar_sensor_model = LaunchConfiguration("lidar_sensor_model")
    launch_pointcloud_filter = LaunchConfiguration("launch_pointcloud_filter")
    pointcloud_filter_input_topic = LaunchConfiguration("pointcloud_filter_input_topic")
    pointcloud_filter_output_topic = LaunchConfiguration("pointcloud_filter_output_topic")
    pointcloud_filter_leaf_size_m = LaunchConfiguration("pointcloud_filter_leaf_size_m")
    pointcloud_filter_min_range_m = LaunchConfiguration("pointcloud_filter_min_range_m")
    pointcloud_filter_max_range_m = LaunchConfiguration("pointcloud_filter_max_range_m")
    pointcloud_filter_max_points = LaunchConfiguration("pointcloud_filter_max_points")
    localization_pointcloud_topic = LaunchConfiguration("localization_pointcloud_topic")
    launch_fixposition = LaunchConfiguration("launch_fixposition")
    launch_fixposition_seed = LaunchConfiguration("launch_fixposition_seed")
    launch_manual_seed = LaunchConfiguration("launch_manual_seed")
    launch_map_projection_loader = LaunchConfiguration("launch_map_projection_loader")
    manual_seed_input_topic = LaunchConfiguration("manual_seed_input_topic")
    manual_seed_require_input_pose = LaunchConfiguration("manual_seed_require_input_pose")
    manual_seed_x = LaunchConfiguration("manual_seed_x")
    manual_seed_y = LaunchConfiguration("manual_seed_y")
    manual_seed_z = LaunchConfiguration("manual_seed_z")
    manual_seed_yaw = LaunchConfiguration("manual_seed_yaw")
    manual_seed_xy_variance = LaunchConfiguration("manual_seed_xy_variance")
    manual_seed_z_variance = LaunchConfiguration("manual_seed_z_variance")
    manual_seed_yaw_variance = LaunchConfiguration("manual_seed_yaw_variance")
    ndt_param_file = LaunchConfiguration("ndt_param_file")
    ndt_initial_pose_stamp_offset_sec = LaunchConfiguration(
        "ndt_initial_pose_stamp_offset_sec"
    )
    fixposition_stream = LaunchConfiguration("fixposition_stream")
    imu_serial_port = LaunchConfiguration("imu_serial_port")
    imu_baudrate = LaunchConfiguration("imu_baudrate")

    default_rviz_config = _pkg_file("autoracer_bringup", "rviz", "rc_autoware.rviz")
    default_extrinsics_file = _pkg_file(
        "autoracer_description", "config", "rc_sensor_extrinsics.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=EnvironmentVariable("MAP_PATH", default_value=default_map_path),
            ),
            DeclareLaunchArgument("launch_sensing", default_value="true"),
            DeclareLaunchArgument("launch_imu", default_value="true"),
            DeclareLaunchArgument("launch_localization", default_value="true"),
            DeclareLaunchArgument("launch_planning", default_value="true"),
            DeclareLaunchArgument("launch_control", default_value="true"),
            DeclareLaunchArgument("launch_safety", default_value="true"),
            DeclareLaunchArgument("launch_vehicle", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("extrinsics_file", default_value=default_extrinsics_file),
            DeclareLaunchArgument("enable_drive_commands", default_value="false"),
            DeclareLaunchArgument("max_speed_mps", default_value="3.0"),
            DeclareLaunchArgument("control_min_lookahead_m", default_value="1.0"),
            DeclareLaunchArgument("control_lookahead_gain", default_value="1.0"),
            DeclareLaunchArgument("control_goal_tolerance_m", default_value="0.35"),
            DeclareLaunchArgument("control_max_steer_rate_radps", default_value="1.5"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("serial_baudrate", default_value="115200"),
            DeclareLaunchArgument("wheel_base_m", default_value="0.6"),
            DeclareLaunchArgument("max_steer_rad", default_value="0.262"),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.102"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.200"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("lidar_driver", default_value="lslidar_c32"),
            DeclareLaunchArgument(
                "lidar_param_file",
                default_value=_pkg_file(
                    "autoracer_bringup", "config", "rc", "lslidar_cx.yaml"
                ),
            ),
            DeclareLaunchArgument("lidar_sensor_model", default_value="C32"),
            DeclareLaunchArgument("launch_pointcloud_filter", default_value="true"),
            DeclareLaunchArgument(
                "pointcloud_filter_input_topic",
                default_value="/sensing/lidar/concatenated/pointcloud",
            ),
            DeclareLaunchArgument(
                "pointcloud_filter_output_topic",
                default_value="/sensing/lidar/filtered/pointcloud",
            ),
            DeclareLaunchArgument("pointcloud_filter_leaf_size_m", default_value="0.25"),
            DeclareLaunchArgument("pointcloud_filter_min_range_m", default_value="0.15"),
            DeclareLaunchArgument("pointcloud_filter_max_range_m", default_value="60.0"),
            DeclareLaunchArgument("pointcloud_filter_max_points", default_value="1500"),
            DeclareLaunchArgument(
                "localization_pointcloud_topic",
                default_value="/sensing/lidar/filtered/pointcloud",
            ),
            DeclareLaunchArgument("launch_fixposition", default_value="false"),
            DeclareLaunchArgument("launch_fixposition_seed", default_value="false"),
            DeclareLaunchArgument("launch_manual_seed", default_value="true"),
            DeclareLaunchArgument("launch_map_projection_loader", default_value="true"),
            DeclareLaunchArgument("manual_seed_input_topic", default_value="/initialpose"),
            DeclareLaunchArgument("manual_seed_require_input_pose", default_value="true"),
            DeclareLaunchArgument("manual_seed_x", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_y", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_z", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_yaw", default_value="0.0"),
            DeclareLaunchArgument("manual_seed_xy_variance", default_value="1.0"),
            DeclareLaunchArgument("manual_seed_z_variance", default_value="0.25"),
            DeclareLaunchArgument("manual_seed_yaw_variance", default_value="0.03"),
            DeclareLaunchArgument(
                "ndt_param_file",
                default_value=_pkg_file(
                    "autoracer_bringup", "config", "rc", "ndt_scan_matcher.param.yaml"
                ),
            ),
            DeclareLaunchArgument("ndt_initial_pose_stamp_offset_sec", default_value="-0.10"),
            DeclareLaunchArgument(
                "fixposition_stream", default_value="tcpcli://192.168.1.200:21000"
            ),
            DeclareLaunchArgument("imu_serial_port", default_value="/dev/autoracer_imu"),
            DeclareLaunchArgument("imu_baudrate", default_value="115200"),
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
                    "lidar_driver": lidar_driver,
                    "lidar_param_file": lidar_param_file,
                    "lidar_host_ip": lidar_host_ip,
                    "lidar_sensor_ip": lidar_sensor_ip,
                    "lidar_data_port": lidar_data_port,
                    "sensor_model": lidar_sensor_model,
                    "launch_fixposition": launch_fixposition,
                    "fixposition_stream": fixposition_stream,
                    "launch_imu": launch_imu,
                    "imu_serial_port": imu_serial_port,
                    "imu_baudrate": imu_baudrate,
                    "launch_pointcloud_filter": launch_pointcloud_filter,
                    "pointcloud_filter_input_topic": pointcloud_filter_input_topic,
                    "pointcloud_filter_output_topic": pointcloud_filter_output_topic,
                    "pointcloud_filter_leaf_size_m": pointcloud_filter_leaf_size_m,
                    "pointcloud_filter_min_range_m": pointcloud_filter_min_range_m,
                    "pointcloud_filter_max_range_m": pointcloud_filter_max_range_m,
                    "pointcloud_filter_max_points": pointcloud_filter_max_points,
                }.items(),
                condition=IfCondition(launch_sensing),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_bringup", "launch", "localization.launch.py")
                ),
                launch_arguments={
                    "map_path": map_path,
                    "wheel_base_m": wheel_base_m,
                    "input_pointcloud": localization_pointcloud_topic,
                    "launch_fixposition_seed": launch_fixposition_seed,
                    "launch_manual_seed": launch_manual_seed,
                    "launch_map_projection_loader": launch_map_projection_loader,
                    "manual_seed_input_topic": manual_seed_input_topic,
                    "manual_seed_require_input_pose": manual_seed_require_input_pose,
                    "manual_seed_x": manual_seed_x,
                    "manual_seed_y": manual_seed_y,
                    "manual_seed_z": manual_seed_z,
                    "manual_seed_yaw": manual_seed_yaw,
                    "manual_seed_xy_variance": manual_seed_xy_variance,
                    "manual_seed_z_variance": manual_seed_z_variance,
                    "manual_seed_yaw_variance": manual_seed_yaw_variance,
                    "ndt_param_file": ndt_param_file,
                    "ndt_initial_pose_stamp_offset_sec": ndt_initial_pose_stamp_offset_sec,
                }.items(),
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
                condition=IfCondition(launch_planning),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_control", "launch", "control.launch.py")
                ),
                launch_arguments={
                    "max_speed_mps": max_speed_mps,
                    "wheel_base_m": wheel_base_m,
                    "max_steer_rad": max_steer_rad,
                    "min_lookahead_m": control_min_lookahead_m,
                    "lookahead_gain": control_lookahead_gain,
                    "goal_tolerance_m": control_goal_tolerance_m,
                }.items(),
                condition=IfCondition(launch_control),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file("autoracer_safety", "launch", "safety.launch.py")
                ),
                launch_arguments={
                    "enable_drive_commands": enable_drive_commands,
                    "max_speed_mps": max_speed_mps,
                    "max_steer_rad": max_steer_rad,
                    "max_steer_rate_radps": control_max_steer_rate_radps,
                }.items(),
                condition=IfCondition(launch_safety),
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
