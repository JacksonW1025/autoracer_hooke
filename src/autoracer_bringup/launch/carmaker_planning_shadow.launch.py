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

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_map_path"),
            DeclareLaunchArgument("course_path"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file(
                        "autoracer_bringup", "pilot_carmaker_localization.launch.py"
                    )
                ),
                launch_arguments={
                    "localization_map_path": localization_map_path,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file(
                        "autoracer_planning", "fixed_course_planning.launch.py"
                    )
                ),
                launch_arguments={
                    "course_path": course_path,
                    "map_path": localization_map_path,
                    "use_sim_time": use_sim_time,
                    "max_speed_mps": "5.0",
                }.items(),
            ),
        ]
    )
