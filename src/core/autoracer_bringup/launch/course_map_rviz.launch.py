from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pcd_path = LaunchConfiguration("pcd_path")
    course_path = LaunchConfiguration("course_path")
    map_path = LaunchConfiguration("map_path")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_config = PathJoinSubstitution(
        [get_package_share_directory("autoracer_bringup"), "rviz", "course_map.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("pcd_path"),
            DeclareLaunchArgument("course_path"),
            DeclareLaunchArgument("map_path"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="pcl_ros",
                executable="pcd_to_pointcloud",
                name="course_map_pcd_publisher",
                output="screen",
                parameters=[
                    {
                        "file_name": pcd_path,
                        "tf_frame": "map",
                        "publishing_period_ms": 10000,
                    }
                ],
                remappings=[("cloud_pcd", "/map/pointcloud_map")],
            ),
            Node(
                package="autoracer_planning",
                executable="fixed_course_publisher",
                name="course_map_fixed_course_publisher",
                output="screen",
                parameters=[
                    {
                        "course_path": course_path,
                        "map_path": map_path,
                        "trajectory_topic": "/planning/global_trajectory",
                        "visualization_topic": "/planning/course_markers",
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="course_map_rviz",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
