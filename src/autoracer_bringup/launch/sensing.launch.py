from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_fixposition = LaunchConfiguration("launch_fixposition")
    lidar_param_file = LaunchConfiguration("lidar_param_file")

    fixposition_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("fixposition_driver_ros2"), "launch", "tcp.launch"]
            )
        ),
        condition=IfCondition(launch_fixposition),
    )

    lidar_container = ComposableNodeContainer(
        name="autoracer_lidar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="nebula_hesai",
                plugin="HesaiRosWrapper",
                name="hesai_ros_wrapper_node",
                parameters=[ParameterFile(lidar_param_file, allow_substs=True)],
                remappings=[
                    ("pandar_points", "/sensing/lidar/concatenated/pointcloud"),
                    ("velodyne_points", "/sensing/lidar/concatenated/pointcloud"),
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        ],
        output="screen",
        condition=IfCondition(launch_lidar),
    )

    default_lidar_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "lidar_top.param.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_fixposition", default_value="true"),
            DeclareLaunchArgument("lidar_param_file", default_value=default_lidar_param_file),
            fixposition_launch,
            lidar_container,
        ]
    )

