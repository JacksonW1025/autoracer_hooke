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
    use_sim_time = LaunchConfiguration("use_sim_time")
    system_run_mode = LaunchConfiguration("system_run_mode")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    vehicle_info_param_file = LaunchConfiguration("vehicle_info_param_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_map_path"),
            DeclareLaunchArgument("course_path"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "system_run_mode",
                default_value="logging_simulation",
                choices=["online", "logging_simulation"],
            ),
            DeclareLaunchArgument("max_speed_mps", default_value="100.0"),
            DeclareLaunchArgument("vehicle_info_param_file"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_bringup", "planning.launch.py")
                ),
                launch_arguments={
                    "localization_map_path": localization_map_path,
                    "course_path": course_path,
                    "use_sim_time": use_sim_time,
                    "system_run_mode": system_run_mode,
                    "max_speed_mps": max_speed_mps,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_control", "race_control.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "vehicle_info_param_file": vehicle_info_param_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_safety", "race_safety.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "vehicle_info_param_file": vehicle_info_param_file,
                }.items(),
            ),
        ]
    )
