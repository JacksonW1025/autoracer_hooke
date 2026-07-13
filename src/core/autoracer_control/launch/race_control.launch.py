from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    reference_trajectory_topic = LaunchConfiguration("reference_trajectory_topic")
    odometry_topic = LaunchConfiguration("odometry_topic")
    steering_topic = LaunchConfiguration("steering_topic")
    accel_topic = LaunchConfiguration("accel_topic")
    operation_mode_topic = LaunchConfiguration("operation_mode_topic")
    raw_control_topic = LaunchConfiguration("raw_control_topic")
    vehicle_info_param_file = LaunchConfiguration("vehicle_info_param_file")
    race_param_file = LaunchConfiguration("race_param_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    lateral_defaults = PathJoinSubstitution(
        [
            FindPackageShare("autoware_mpc_lateral_controller"),
            "param",
            "lateral_controller_defaults.param.yaml",
        ]
    )
    longitudinal_defaults = PathJoinSubstitution(
        [
            FindPackageShare("autoware_pid_longitudinal_controller"),
            "config",
            "autoware_pid_longitudinal_controller.param.yaml",
        ]
    )
    trajectory_follower_defaults = PathJoinSubstitution(
        [
            FindPackageShare("autoware_trajectory_follower_node"),
            "param",
            "trajectory_follower_node.param.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "reference_trajectory_topic",
                default_value="/planning/trajectory",
            ),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="/localization/kinematic_state",
            ),
            DeclareLaunchArgument(
                "steering_topic",
                default_value="/vehicle/status/steering_status",
            ),
            DeclareLaunchArgument(
                "accel_topic",
                default_value="/localization/acceleration",
            ),
            DeclareLaunchArgument(
                "operation_mode_topic",
                default_value="/system/operation_mode/state",
            ),
            DeclareLaunchArgument(
                "raw_control_topic",
                default_value="/control/trajectory_follower/control_cmd",
            ),
            DeclareLaunchArgument(
                "vehicle_info_param_file",
            ),
            DeclareLaunchArgument(
                "race_param_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("autoracer_control"),
                        "config",
                        "race_controller.param.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="autoware_trajectory_follower_node",
                executable="controller_node_exe",
                name="controller",
                output="screen",
                parameters=[
                    lateral_defaults,
                    longitudinal_defaults,
                    trajectory_follower_defaults,
                    vehicle_info_param_file,
                    race_param_file,
                    {
                        "lateral_controller_mode": "mpc",
                        "longitudinal_controller_mode": "pid",
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
                remappings=[
                    ("~/input/reference_trajectory", reference_trajectory_topic),
                    ("~/input/current_odometry", odometry_topic),
                    ("~/input/current_steering", steering_topic),
                    ("~/input/current_accel", accel_topic),
                    ("~/input/current_operation_mode", operation_mode_topic),
                    ("~/output/control_cmd", raw_control_topic),
                ],
            ),
        ]
    )
