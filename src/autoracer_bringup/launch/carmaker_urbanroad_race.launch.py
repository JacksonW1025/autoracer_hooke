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
    enable_ndt_pose_guard = LaunchConfiguration("enable_ndt_pose_guard")
    max_speed_mps = LaunchConfiguration("max_speed_mps")

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_map_path"),
            DeclareLaunchArgument("course_path"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("max_speed_mps", default_value="100.0"),
            DeclareLaunchArgument(
                "enable_ndt_pose_guard",
                default_value="false",
                choices=["true", "false"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_bringup", "carmaker_planning_shadow.launch.py")
                ),
                launch_arguments={
                    "localization_map_path": localization_map_path,
                    "course_path": course_path,
                    "use_sim_time": use_sim_time,
                    "enable_ndt_pose_guard": enable_ndt_pose_guard,
                    "max_speed_mps": max_speed_mps,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_control", "race_control.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file("autoracer_safety", "race_safety.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
