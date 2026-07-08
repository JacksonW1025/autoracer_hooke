from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _driver_condition(launch_lidar, lidar_driver, expected_driver):
    return IfCondition(
        PythonExpression(
            [
                "'",
                launch_lidar,
                "'.lower() in ['1', 'true', 'yes', 'on'] and '",
                lidar_driver,
                "' == '",
                expected_driver,
                "'",
            ]
        )
    )


def generate_launch_description():
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_imu = LaunchConfiguration("launch_imu")
    launch_fixposition = LaunchConfiguration("launch_fixposition")
    lidar_driver = LaunchConfiguration("lidar_driver")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    sensor_model = LaunchConfiguration("sensor_model")
    fixposition_param_file = LaunchConfiguration("fixposition_param_file")
    fixposition_stream = LaunchConfiguration("fixposition_stream")
    fixposition_speed_topic = LaunchConfiguration("fixposition_speed_topic")
    imu_serial_port = LaunchConfiguration("imu_serial_port")
    imu_baudrate = LaunchConfiguration("imu_baudrate")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_raw_topic = LaunchConfiguration("imu_raw_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_filter_param_file = LaunchConfiguration("imu_filter_param_file")
    launch_pointcloud_filter = LaunchConfiguration("launch_pointcloud_filter")
    pointcloud_filter_input_topic = LaunchConfiguration("pointcloud_filter_input_topic")
    pointcloud_filter_output_topic = LaunchConfiguration("pointcloud_filter_output_topic")
    pointcloud_filter_leaf_size_m = LaunchConfiguration("pointcloud_filter_leaf_size_m")
    pointcloud_filter_min_range_m = LaunchConfiguration("pointcloud_filter_min_range_m")
    pointcloud_filter_max_range_m = LaunchConfiguration("pointcloud_filter_max_range_m")
    pointcloud_filter_max_points = LaunchConfiguration("pointcloud_filter_max_points")

    fixposition_node = Node(
        package="fixposition_driver_ros2",
        executable="fixposition_driver_ros2_exec",
        name="fixposition_driver",
        output="screen",
        parameters=[
            ParameterFile(fixposition_param_file, allow_substs=True),
            {
                "stream": ParameterValue(fixposition_stream, value_type=str),
                "speed_topic": ParameterValue(fixposition_speed_topic, value_type=str),
            },
        ],
        condition=IfCondition(launch_fixposition),
    )

    fixposition_speed_bridge = Node(
        package="autoracer_sensing",
        executable="velocity_to_fixposition_speed",
        name="velocity_to_fixposition_speed",
        output="screen",
        parameters=[
            {
                "input_topic": "/vehicle/status/velocity_status",
                "output_topic": ParameterValue(fixposition_speed_topic, value_type=str),
                "sensor_location": "RC",
            }
        ],
        condition=IfCondition(launch_fixposition),
    )

    lidar_container = ComposableNodeContainer(
        name="autoracer_lidar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="nebula_hesai",
                plugin="HesaiRosWrapper",
                name="hesai_ros_wrapper_node",
                parameters=[
                    ParameterFile(lidar_param_file, allow_substs=True),
                    {
                        "host_ip": ParameterValue(lidar_host_ip, value_type=str),
                        "sensor_ip": ParameterValue(lidar_sensor_ip, value_type=str),
                        "data_port": ParameterValue(lidar_data_port, value_type=int),
                        "sensor_model": ParameterValue(sensor_model, value_type=str),
                    },
                ],
                remappings=[
                    ("pandar_points", "/sensing/lidar/concatenated/pointcloud"),
                    ("velodyne_points", "/sensing/lidar/concatenated/pointcloud"),
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        ],
        output="screen",
        condition=_driver_condition(launch_lidar, lidar_driver, "hesai"),
    )

    lslidar_c32_node = Node(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        namespace="cx",
        name="lslidar_driver_node",
        output="screen",
        parameters=[ParameterFile(lidar_param_file, allow_substs=True)],
        condition=_driver_condition(launch_lidar, lidar_driver, "lslidar_c32"),
    )

    hipnuc_imu_node = Node(
        package="hipnuc_imu",
        executable="talker",
        name="IMU_publisher",
        output="screen",
        parameters=[
            {
                "serial_port": ParameterValue(imu_serial_port, value_type=str),
                "baud_rate": ParameterValue(imu_baudrate, value_type=int),
                "frame_id": ParameterValue(imu_frame_id, value_type=str),
                "imu_topic": ParameterValue(imu_raw_topic, value_type=str),
            }
        ],
        condition=IfCondition(launch_imu),
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        output="screen",
        parameters=[ParameterFile(imu_filter_param_file, allow_substs=True)],
        remappings=[
            ("imu/data_raw", imu_raw_topic),
            ("imu/data", imu_topic),
        ],
        condition=IfCondition(launch_imu),
    )

    pointcloud_filter_node = Node(
        package="autoracer_sensing",
        executable="pointcloud_voxel_filter",
        name="pointcloud_voxel_filter",
        output="screen",
        parameters=[
            {
                "input_topic": ParameterValue(pointcloud_filter_input_topic, value_type=str),
                "output_topic": ParameterValue(pointcloud_filter_output_topic, value_type=str),
                "leaf_size_m": ParameterValue(pointcloud_filter_leaf_size_m, value_type=float),
                "min_range_m": ParameterValue(pointcloud_filter_min_range_m, value_type=float),
                "max_range_m": ParameterValue(pointcloud_filter_max_range_m, value_type=float),
                "max_points": ParameterValue(pointcloud_filter_max_points, value_type=int),
            }
        ],
        condition=IfCondition(launch_pointcloud_filter),
    )

    default_lidar_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "rc",
            "lslidar_cx.yaml",
        ]
    )
    default_fixposition_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "fixposition.param.yaml",
        ]
    )
    default_imu_filter_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "rc",
            "imu_filter_madgwick.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_imu", default_value="true"),
            DeclareLaunchArgument("launch_fixposition", default_value="false"),
            DeclareLaunchArgument("lidar_driver", default_value="lslidar_c32"),
            DeclareLaunchArgument("lidar_param_file", default_value=default_lidar_param_file),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.102"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.200"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("sensor_model", default_value="C32"),
            DeclareLaunchArgument(
                "fixposition_param_file", default_value=default_fixposition_param_file
            ),
            DeclareLaunchArgument(
                "fixposition_stream", default_value="tcpcli://192.168.1.200:21000"
            ),
            DeclareLaunchArgument("fixposition_speed_topic", default_value="/fixposition/speed"),
            DeclareLaunchArgument("imu_serial_port", default_value="/dev/autoracer_imu"),
            DeclareLaunchArgument("imu_baudrate", default_value="115200"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_raw_topic", default_value="/imu/data_raw"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("imu_filter_param_file", default_value=default_imu_filter_param_file),
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
            fixposition_node,
            fixposition_speed_bridge,
            lidar_container,
            lslidar_c32_node,
            pointcloud_filter_node,
            hipnuc_imu_node,
            imu_filter_node,
        ]
    )
