from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _launch_file(package, filename):
    return PathJoinSubstitution(
        [get_package_share_directory(package), "launch", filename]
    )


def _rc_config(filename):
    return PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_rc_bringup"),
            "config",
            "rc",
            filename,
        ]
    )


def generate_launch_description():
    localization_map_path = LaunchConfiguration("localization_map_path")
    course_path = LaunchConfiguration("course_path")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    max_accel_mps2 = LaunchConfiguration("max_accel_mps2")
    max_decel_mps2 = LaunchConfiguration("max_decel_mps2")
    command_latency_sec = LaunchConfiguration("command_latency_sec")
    stopping_margin_m = LaunchConfiguration("stopping_margin_m")
    chassis_serial_port = LaunchConfiguration("chassis_serial_port")
    imu_device = LaunchConfiguration("imu_device")
    g90_device = LaunchConfiguration("g90_device")

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_map_path"),
            DeclareLaunchArgument("course_path"),
            DeclareLaunchArgument("max_speed_mps"),
            DeclareLaunchArgument("max_accel_mps2"),
            DeclareLaunchArgument("max_decel_mps2"),
            DeclareLaunchArgument("command_latency_sec"),
            DeclareLaunchArgument("stopping_margin_m"),
            DeclareLaunchArgument(
                "chassis_serial_port", default_value="/dev/autoracer_rc_chassis"
            ),
            DeclareLaunchArgument(
                "imu_device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
                    "0003-if00-port0"
                ),
            ),
            DeclareLaunchArgument(
                "g90_device",
                description="Stable /dev/serial/by-id identity for the G90 receiver.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_rc_bringup", "sensing.launch.py")
                ),
                launch_arguments={
                    "launch_static_tf": "true",
                    "launch_lidar": "true",
                    "launch_imu": "true",
                    "launch_g90": "true",
                    "launch_g90_driver": "true",
                    "imu_device": imu_device,
                    "g90_device": g90_device,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_rc_bringup", "vehicle.launch.py")
                ),
                launch_arguments={
                    "serial_port": chassis_serial_port,
                    "telemetry_only": "false",
                    "use_sim_time": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_bringup", "race.launch.py")
                ),
                launch_arguments={
                    "localization_map_path": localization_map_path,
                    "course_path": course_path,
                    "max_speed_mps": max_speed_mps,
                    "max_accel_mps2": max_accel_mps2,
                    "max_decel_mps2": max_decel_mps2,
                    "command_latency_sec": command_latency_sec,
                    "stopping_margin_m": stopping_margin_m,
                    "vehicle_info_param_file": _rc_config(
                        "vehicle_info.param.yaml"
                    ),
                    "gate_param_file": _rc_config(
                        "vehicle_cmd_gate.param.yaml"
                    ),
                    "runtime_param_file": _rc_config(
                        "race_runtime.param.yaml"
                    ),
                    "use_sim_time": "false",
                    "system_run_mode": "online",
                }.items(),
            ),
        ]
    )
