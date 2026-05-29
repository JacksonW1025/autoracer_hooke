from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")

    ndt_param_file = PathJoinSubstitution(
        [
            get_package_share_directory("autoracer_bringup"),
            "config",
            "hooke2",
            "ndt_scan_matcher.param.yaml",
        ]
    )

    map_projector_info = PathJoinSubstitution([map_path, "map_projector_info.yaml"])
    lanelet2_map = PathJoinSubstitution([map_path, "lanelet2_map.osm"])
    pointcloud_map = PathJoinSubstitution([map_path, "pointcloud_map.pcd"])
    pointcloud_metadata = PathJoinSubstitution([map_path, "pointcloud_map_metadata.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path"),
            Node(
                package="autoware_map_projection_loader",
                executable="autoware_map_projection_loader_node",
                name="map_projection_loader",
                output="screen",
                parameters=[
                    {
                        "map_projector_info_path": map_projector_info,
                        "lanelet2_map_path": lanelet2_map,
                        "use_local_projector": False,
                    }
                ],
            ),
            Node(
                package="autoware_map_loader",
                executable="autoware_pointcloud_map_loader",
                name="pointcloud_map_loader",
                output="screen",
                parameters=[
                    {
                        "enable_whole_load": True,
                        "enable_downsampled_whole_load": False,
                        "enable_partial_load": True,
                        "enable_selected_load": False,
                        "leaf_size": 3.0,
                        "pcd_paths_or_directory": [pointcloud_map],
                        "pcd_metadata_path": pointcloud_metadata,
                    }
                ],
                remappings=[
                    ("output/pointcloud_map", "/map/pointcloud_map"),
                    ("service/get_partial_pcd_map", "/map/get_partial_pointcloud_map"),
                    ("service/get_selected_pcd_map", "/map/get_selected_pointcloud_map"),
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoware_gnss_poser"),
                            "launch",
                            "gnss_poser.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "input_topic_fix": "/fixposition/fix",
                    "input_topic_orientation": "/fixposition/autoware_orientation",
                    "output_topic_gnss_pose": "/sensing/gnss/pose",
                    "output_topic_gnss_pose_cov": "/sensing/gnss/pose_with_covariance",
                    "output_topic_gnss_fixed": "/sensing/gnss/fixed",
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoware_ndt_scan_matcher"),
                            "launch",
                            "ndt_scan_matcher.launch.xml",
                        ]
                    )
                ),
                launch_arguments={
                    "param_file": ndt_param_file,
                    "input_pointcloud": "/sensing/lidar/concatenated/pointcloud",
                    "input_initial_pose_topic": "/sensing/gnss/pose_with_covariance",
                    "input_regularization_pose_topic": "/sensing/gnss/pose_with_covariance",
                    "output_pose_topic": "/localization/pose",
                    "output_pose_with_covariance_topic": "/localization/pose_with_covariance",
                    "client_map_loader": "/map/get_partial_pointcloud_map",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("autoracer_localization"),
                            "launch",
                            "pose_tf.launch.py",
                        ]
                    )
                )
            ),
        ]
    )
