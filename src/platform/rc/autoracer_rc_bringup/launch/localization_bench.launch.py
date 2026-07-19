from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def _python_launch(package, filename):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [get_package_share_directory(package), "launch", filename]
        )
    )


def generate_launch_description():
    localization_map_path = LaunchConfiguration("localization_map_path")
    course_path = LaunchConfiguration("course_path")
    launch_vehicle_telemetry = LaunchConfiguration("launch_vehicle_telemetry")
    chassis_serial_port = LaunchConfiguration("chassis_serial_port")
    g90_device = LaunchConfiguration("g90_device")

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_map_path"),
            DeclareLaunchArgument("course_path"),
            DeclareLaunchArgument(
                "launch_vehicle_telemetry",
                default_value="false",
                description=(
                    "Open the chassis receive path. The included vehicle launch is "
                    "always forced to telemetry_only=true."
                ),
            ),
            DeclareLaunchArgument(
                "chassis_serial_port",
                default_value="/dev/autoracer_rc_chassis",
            ),
            DeclareLaunchArgument(
                "g90_device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5AA6079369-if00"
                ),
            ),
            SetParameter(name="use_sim_time", value=False),
            IncludeLaunchDescription(
                _python_launch("autoracer_rc_bringup", "sensing.launch.py"),
                launch_arguments={
                    "launch_static_tf": "true",
                    "launch_lidar": "true",
                    "launch_imu": "true",
                    "launch_g90": "false",
                    "launch_g90_driver": "false",
                    "g90_device": g90_device,
                    "g90_baud": "115200",
                }.items(),
            ),
            IncludeLaunchDescription(
                _python_launch("autoracer_rc_bringup", "vehicle.launch.py"),
                launch_arguments={
                    "serial_port": chassis_serial_port,
                    "telemetry_only": "true",
                    "use_sim_time": "false",
                }.items(),
                condition=IfCondition(launch_vehicle_telemetry),
            ),
            IncludeLaunchDescription(
                _python_launch("autoracer_localization", "localization.launch.py"),
                launch_arguments={
                    "localization_map_path": localization_map_path,
                    "use_sim_time": "false",
                    "system_run_mode": "online",
                    "input_pointcloud": "/sensing/lidar/concatenated/pointcloud",
                    "initial_pose": "[]",
                    "gnss_enabled": "false",
                }.items(),
            ),
            Node(
                package="autoracer_planning",
                executable="fixed_course_publisher",
                name="fixed_course_publisher",
                output="screen",
                parameters=[
                    {
                        "course_path": course_path,
                        "map_path": localization_map_path,
                        "trajectory_topic": "/planning/global_trajectory",
                        "use_sim_time": False,
                    }
                ],
            ),
        ]
    )
