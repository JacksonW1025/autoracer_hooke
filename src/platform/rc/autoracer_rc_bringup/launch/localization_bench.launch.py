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
    package_share = get_package_share_directory("autoracer_rc_bringup")
    localization_map_path = LaunchConfiguration("localization_map_path")
    course_path = LaunchConfiguration("course_path")
    launch_vehicle_telemetry = LaunchConfiguration("launch_vehicle_telemetry")
    chassis_serial_port = LaunchConfiguration("chassis_serial_port")
    launch_g90 = LaunchConfiguration("launch_g90")
    launch_g90_driver = LaunchConfiguration("launch_g90_driver")
    launch_g90_corrections = LaunchConfiguration("launch_g90_corrections")
    g90_device = LaunchConfiguration("g90_device")
    g90_com2_device = LaunchConfiguration("g90_com2_device")
    g90_ntrip_config_file = LaunchConfiguration("g90_ntrip_config_file")
    g90_param_file = LaunchConfiguration("g90_param_file")
    gnss_enabled = LaunchConfiguration("gnss_enabled")

    default_g90_param_file = PathJoinSubstitution(
        [package_share, "config", "rc", "g90.param.yaml"]
    )

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
                "launch_g90",
                default_value="false",
                description=(
                    "Start the RC G90 platform adapter. Keep false until the "
                    "current run is explicitly using GNSS."
                ),
            ),
            DeclareLaunchArgument(
                "launch_g90_driver",
                default_value=launch_g90,
                description=(
                    "Start the physical G90 serial reader. Set false only for "
                    "an explicitly identified fixture publisher."
                ),
            ),
            DeclareLaunchArgument(
                "launch_g90_corrections",
                default_value="false",
                description="Start the project-owned G90 COM2 correction relay.",
            ),
            DeclareLaunchArgument(
                "g90_device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5AA6079369-if00"
                ),
            ),
            DeclareLaunchArgument(
                "g90_param_file",
                default_value=default_g90_param_file,
                description=(
                    "G90 adapter parameters. Outdoor calibration may supply a "
                    "generated file without editing the product repository."
                ),
            ),
            DeclareLaunchArgument(
                "g90_com2_device",
                default_value="/dev/autoracer_g90_com2",
            ),
            DeclareLaunchArgument("g90_ntrip_config_file", default_value=""),
            DeclareLaunchArgument(
                "gnss_enabled",
                default_value="false",
                description=(
                    "Enable the existing Core GNSS initialization path. This "
                    "does not bypass or replace the Core pose initializer."
                ),
            ),
            SetParameter(name="use_sim_time", value=False),
            IncludeLaunchDescription(
                _python_launch("autoracer_rc_bringup", "sensing.launch.py"),
                launch_arguments={
                    "launch_static_tf": "true",
                    "launch_lidar": "true",
                    "launch_imu": "true",
                    "launch_g90": launch_g90,
                    "launch_g90_driver": launch_g90_driver,
                    "launch_g90_corrections": launch_g90_corrections,
                    "g90_device": g90_device,
                    "g90_baud": "115200",
                    "g90_param_file": g90_param_file,
                    "g90_com2_device": g90_com2_device,
                    "g90_ntrip_config_file": g90_ntrip_config_file,
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
                    "gnss_enabled": gnss_enabled,
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
