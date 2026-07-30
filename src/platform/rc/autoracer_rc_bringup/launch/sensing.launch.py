from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    launch_static_tf = LaunchConfiguration("launch_static_tf")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_imu = LaunchConfiguration("launch_imu")
    launch_g90 = LaunchConfiguration("launch_g90")
    launch_g90_driver = LaunchConfiguration("launch_g90_driver")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    imu_param_file = LaunchConfiguration("imu_param_file")
    imu_device = LaunchConfiguration("imu_device")
    g90_param_file = LaunchConfiguration("g90_param_file")
    g90_device = LaunchConfiguration("g90_device")
    g90_baud = LaunchConfiguration("g90_baud")

    package_share = get_package_share_directory("autoracer_rc_bringup")
    default_lidar_param_file = PathJoinSubstitution(
        [package_share, "config", "rc", "lidar.param.yaml"]
    )
    default_imu_param_file = PathJoinSubstitution(
        [package_share, "config", "rc", "imu.param.yaml"]
    )
    default_g90_param_file = PathJoinSubstitution(
        [package_share, "config", "rc", "g90.param.yaml"]
    )

    g90_serial_reader = Node(
        package="nmea_navsat_driver",
        executable="nmea_topic_serial_reader",
        namespace="g90/raw",
        name="nmea_topic_serial_reader",
        output="screen",
        parameters=[
            {
                "port": ParameterValue(g90_device, value_type=str),
                "baud": ParameterValue(g90_baud, value_type=int),
                "frame_id": "gnss_link",
            }
        ],
        remappings=[("nmea_sentence", "/g90/raw/nmea_sentence")],
        condition=IfCondition(launch_g90_driver),
    )

    g90_adapter = Node(
        package="autoracer_rc_adapter",
        executable="g90_nmea_adapter",
        namespace="g90",
        name="g90_nmea_adapter",
        output="screen",
        parameters=[ParameterFile(g90_param_file, allow_substs=True)],
        remappings=[
            ("nmea_sentence", "/g90/raw/nmea_sentence"),
            ("fix", "/g90/fix"),
            ("autoware_orientation", "/g90/autoware_orientation"),
        ],
        condition=IfCondition(launch_g90),
    )

    g90_gnss_normalization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("autoware_gnss_poser"),
                    "launch",
                    "gnss_poser.launch.xml",
                ]
            )
        ),
        launch_arguments={
            "input_topic_fix": "/g90/fix",
            "input_topic_orientation": "/g90/autoware_orientation",
            "output_topic_gnss_pose": "/sensing/gnss/pose",
            "output_topic_gnss_pose_cov": "/sensing/gnss/pose_with_covariance",
            "output_topic_gnss_fixed": "/sensing/gnss/fixed",
        }.items(),
        condition=IfCondition(launch_g90),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_static_tf", default_value="true"),
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_imu", default_value="true"),
            DeclareLaunchArgument("launch_g90", default_value="false"),
            DeclareLaunchArgument(
                "launch_g90_driver",
                default_value=launch_g90,
                description=(
                    "Start the physical G90 serial transport. Disable it for "
                    "validated raw NMEA replay."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_param_file", default_value=default_lidar_param_file
            ),
            DeclareLaunchArgument("imu_param_file", default_value=default_imu_param_file),
            DeclareLaunchArgument(
                "g90_param_file", default_value=default_g90_param_file
            ),
            DeclareLaunchArgument(
                "g90_device",
                default_value="",
                description=(
                    "Measured stable /dev/serial/by-id path for the G90; no "
                    "ttyUSB fallback is permitted."
                ),
            ),
            DeclareLaunchArgument("g90_baud", default_value="115200"),
            DeclareLaunchArgument(
                "imu_device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
                    "0003-if00-port0"
                ),
                description=(
                    "Stable USB IMU path; override this argument when replacing the "
                    "USB bridge."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoracer_rc_description"),
                            "launch",
                            "static_tf.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(launch_static_tf),
            ),
            Node(
                package="lslidar_driver",
                executable="lslidar_driver_node",
                namespace="cx",
                name="lslidar_driver_node",
                output="screen",
                parameters=[ParameterFile(lidar_param_file, allow_substs=True)],
                condition=IfCondition(launch_lidar),
            ),
            Node(
                package="autoracer_rc_adapter",
                executable="c32_pointcloud_adapter_node",
                name="c32_pointcloud_adapter",
                output="screen",
                remappings=[
                    ("input", "/sensing/lidar/raw/pointcloud"),
                    ("output", "/sensing/lidar/concatenated/pointcloud"),
                ],
                condition=IfCondition(launch_lidar),
            ),
            Node(
                package="hipnuc_imu",
                executable="talker",
                name="IMU_publisher",
                output="screen",
                parameters=[
                    ParameterFile(imu_param_file, allow_substs=True),
                    {"serial_port": ParameterValue(imu_device, value_type=str)},
                ],
                condition=IfCondition(launch_imu),
            ),
            Node(
                package="autoracer_rc_adapter",
                executable="imu_qos_adapter_node",
                name="imu_qos_adapter",
                output="screen",
                parameters=[ParameterFile(imu_param_file, allow_substs=True)],
                remappings=[
                    ("input", "/sensing/imu/raw/imu_data"),
                    ("output", "/sensing/imu/imu_data"),
                ],
                condition=IfCondition(launch_imu),
            ),
            g90_serial_reader,
            g90_adapter,
            g90_gnss_normalization,
        ]
    )
