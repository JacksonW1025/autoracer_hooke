from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    launch_static_tf = LaunchConfiguration("launch_static_tf")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_imu = LaunchConfiguration("launch_imu")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    imu_param_file = LaunchConfiguration("imu_param_file")
    imu_device = LaunchConfiguration("imu_device")

    package_share = get_package_share_directory("autoracer_rc_bringup")
    default_lidar_param_file = PathJoinSubstitution(
        [package_share, "config", "rc", "lidar.param.yaml"]
    )
    default_imu_param_file = PathJoinSubstitution(
        [package_share, "config", "rc", "imu.param.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_static_tf", default_value="true"),
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_imu", default_value="true"),
            DeclareLaunchArgument(
                "lidar_param_file", default_value=default_lidar_param_file
            ),
            DeclareLaunchArgument("imu_param_file", default_value=default_imu_param_file),
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
        ]
    )
