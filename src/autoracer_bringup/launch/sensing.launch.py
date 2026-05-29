from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_fixposition = LaunchConfiguration("launch_fixposition")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_data_port = LaunchConfiguration("lidar_data_port")
    sensor_model = LaunchConfiguration("sensor_model")
    fixposition_param_file = LaunchConfiguration("fixposition_param_file")
    fixposition_stream = LaunchConfiguration("fixposition_stream")
    fixposition_speed_topic = LaunchConfiguration("fixposition_speed_topic")

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
        condition=IfCondition(launch_lidar),
    )

    default_lidar_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "lidar_top.param.yaml",
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_fixposition", default_value="true"),
            DeclareLaunchArgument("lidar_param_file", default_value=default_lidar_param_file),
            DeclareLaunchArgument("lidar_host_ip", default_value="192.168.1.120"),
            DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.1.130"),
            DeclareLaunchArgument("lidar_data_port", default_value="2368"),
            DeclareLaunchArgument("sensor_model", default_value="Pandar64"),
            DeclareLaunchArgument(
                "fixposition_param_file", default_value=default_fixposition_param_file
            ),
            DeclareLaunchArgument(
                "fixposition_stream", default_value="tcpcli://192.168.1.200:21000"
            ),
            DeclareLaunchArgument("fixposition_speed_topic", default_value="/fixposition/speed"),
            fixposition_node,
            fixposition_speed_bridge,
            lidar_container,
        ]
    )
