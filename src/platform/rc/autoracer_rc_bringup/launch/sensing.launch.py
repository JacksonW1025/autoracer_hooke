# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_imu = LaunchConfiguration("launch_imu")
    lidar_param_file = LaunchConfiguration("lidar_param_file")
    imu_param_file = LaunchConfiguration("imu_param_file")
    imu_filter_param_file = LaunchConfiguration("imu_filter_param_file")
    imu_device = LaunchConfiguration("imu_device")

    lidar = LifecycleNode(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        namespace="cx",
        name="lslidar_driver_node",
        output="screen",
        parameters=[
            ParameterFile(lidar_param_file, allow_substs=True),
            {
                "frame_id": "lidar_top",
                "topic_name": "/sensing/lidar/raw/pointcloud",
            },
        ],
        condition=IfCondition(launch_lidar),
    )
    configure_lidar = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(launch_lidar),
    )
    activate_lidar = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    defaults = PathJoinSubstitution(
        [get_package_share_directory("autoracer_rc_bringup"), "config", "rc"]
    )
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("autoracer_rc_description"),
                    "launch",
                    "static_tf.launch.py",
                ]
            )
        )
    )
    pointcloud_adapter = Node(
        package="autoracer_rc_adapter",
        executable="c32_pointcloud_adapter",
        name="c32_pointcloud_adapter",
        output="screen",
        remappings=[
            ("input", "/sensing/lidar/raw/pointcloud"),
            ("output", "/sensing/lidar/concatenated/pointcloud"),
        ],
        condition=IfCondition(launch_lidar),
    )
    imu = Node(
        package="hipnuc_imu",
        executable="talker",
        name="IMU_publisher",
        output="screen",
        parameters=[
            ParameterFile(imu_param_file, allow_substs=True),
            {"serial_port": ParameterValue(imu_device, value_type=str)},
        ],
        condition=IfCondition(launch_imu),
    )
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[ParameterFile(imu_filter_param_file, allow_substs=True)],
        remappings=[
            ("imu/data_raw", "/sensing/imu/raw"),
            ("imu/data", "/sensing/imu/imu_data"),
        ],
        condition=IfCondition(launch_imu),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_lidar", default_value="true"),
            DeclareLaunchArgument("launch_imu", default_value="true"),
            DeclareLaunchArgument("imu_device"),
            DeclareLaunchArgument(
                "lidar_param_file",
                default_value=PathJoinSubstitution([defaults, "lidar.param.yaml"]),
            ),
            DeclareLaunchArgument(
                "imu_param_file",
                default_value=PathJoinSubstitution([defaults, "imu.param.yaml"]),
            ),
            DeclareLaunchArgument(
                "imu_filter_param_file",
                default_value=PathJoinSubstitution([defaults, "imu_filter.param.yaml"]),
            ),
            static_tf,
            lidar,
            configure_lidar,
            activate_lidar,
            pointcloud_adapter,
            imu,
            imu_filter,
        ]
    )
