from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _launch_file(package, filename):
    return PathJoinSubstitution(
        [get_package_share_directory(package), "launch", filename]
    )


def generate_launch_description():
    localization_map_path = LaunchConfiguration("localization_map_path")
    course_path = LaunchConfiguration("course_path")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    vehicle_info_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_hooke2_bringup"),
            "config",
            "hooke2",
            "vehicle_info.param.yaml",
        ]
    )
    control_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_hooke2_bringup"),
            "config",
            "hooke2",
            "controller.param.yaml",
        ]
    )
    gate_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_hooke2_bringup"),
            "config",
            "hooke2",
            "vehicle_cmd_gate.param.yaml",
        ]
    )
    runtime_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_hooke2_bringup"),
            "config",
            "hooke2",
            "race_runtime.param.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_map_path"),
            DeclareLaunchArgument("course_path"),
            DeclareLaunchArgument("max_speed_mps", default_value="100.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_hooke2_bringup", "sensing.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_hooke2_bringup", "vehicle.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_bringup", "race.launch.py")
                ),
                launch_arguments={
                    "localization_map_path": localization_map_path,
                    "course_path": course_path,
                    "max_speed_mps": max_speed_mps,
                    "vehicle_info_param_file": vehicle_info_param_file,
                    "control_param_file": control_param_file,
                    "gate_param_file": gate_param_file,
                    "runtime_param_file": runtime_param_file,
                    "max_accel_mps2": "0.8",
                    "max_decel_mps2": "-1.5",
                    "command_latency_sec": "0.2",
                    "stopping_margin_m": "5.0",
                    "use_sim_time": "false",
                    "system_run_mode": "online",
                }.items(),
            ),
        ]
    )
