from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gate_param_file = LaunchConfiguration("gate_param_file")
    runtime_param_file = LaunchConfiguration("runtime_param_file")
    vehicle_info_param_file = LaunchConfiguration("vehicle_info_param_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "gate_param_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("autoracer_safety"),
                        "config",
                        "race",
                        "vehicle_cmd_gate.safe.param.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "runtime_param_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("autoracer_safety"),
                        "config",
                        "race",
                        "race_runtime.safe.param.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "vehicle_info_param_file",
            ),
            Node(
                package="autoracer_safety",
                executable="race_runtime_manager",
                name="race_runtime_manager",
                output="screen",
                parameters=[
                    runtime_param_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="autoware_vehicle_cmd_gate",
                executable="vehicle_cmd_gate_exe",
                name="vehicle_cmd_gate",
                output="screen",
                parameters=[
                    gate_param_file,
                    vehicle_info_param_file,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[
                    ("input/steering", "/vehicle/status/steering_status"),
                    ("input/acceleration", "/localization/acceleration"),
                    ("input/operation_mode", "/system/operation_mode/state"),
                    ("input/auto/control_cmd", "/control/trajectory_follower/control_cmd"),
                    (
                        "input/auto/turn_indicators_cmd",
                        "/control/race_runtime/turn_indicators_cmd",
                    ),
                    (
                        "input/auto/hazard_lights_cmd",
                        "/control/race_runtime/hazard_lights_cmd",
                    ),
                    ("input/auto/gear_cmd", "/control/race_runtime/gear_cmd"),
                    ("input/gate_mode", "/control/gate_mode_cmd"),
                    ("input/emergency/control_cmd", "/system/emergency/control_cmd"),
                    (
                        "input/emergency/turn_indicators_cmd",
                        "/system/emergency/turn_indicators_cmd",
                    ),
                    (
                        "input/emergency/hazard_lights_cmd",
                        "/system/emergency/hazard_lights_cmd",
                    ),
                    ("input/emergency/gear_cmd", "/system/emergency/gear_cmd"),
                    ("input/mrm_state", "/system/fail_safe/mrm_state"),
                    ("output/vehicle_cmd_emergency", "/control/command/emergency_cmd"),
                    ("output/control_cmd", "/control/command/control_cmd"),
                    ("output/gear_cmd", "/control/command/gear_cmd"),
                    (
                        "output/turn_indicators_cmd",
                        "/control/command/turn_indicators_cmd",
                    ),
                    ("output/hazard_lights_cmd", "/control/command/hazard_lights_cmd"),
                    ("output/gate_mode", "/control/current_gate_mode"),
                    ("output/engage", "/api/autoware/get/engage"),
                    ("output/external_emergency", "/api/autoware/get/emergency"),
                    ("output/operation_mode", "/control/vehicle_cmd_gate/operation_mode"),
                    ("~/service/engage", "/api/autoware/set/engage"),
                    ("~/service/external_emergency", "/api/autoware/set/emergency"),
                ],
            ),
        ]
    )
