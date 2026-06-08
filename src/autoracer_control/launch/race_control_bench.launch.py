from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


BENCH_TOPICS = {
    "reference_trajectory_topic": "/control_bench/planning/trajectory",
    "odometry_topic": "/control_bench/localization/kinematic_state",
    "steering_topic": "/control_bench/vehicle/status/steering_status",
    "accel_topic": "/control_bench/localization/acceleration",
    "operation_mode_topic": "/control_bench/system/operation_mode/state",
    "raw_control_topic": "/control_bench/autoracer/control/raw_control_cmd",
    "pose_topic": "/control_bench/localization/pose_with_covariance",
    "final_control_topic": "/control_bench/control/command/control_cmd",
    "gear_topic": "/control_bench/control/command/gear_cmd",
    "hazard_topic": "/control_bench/control/command/hazard_lights_cmd",
    "turn_topic": "/control_bench/control/command/turn_indicators_cmd",
    "state_topic": "/control_bench/autoracer/safety/state",
}

COMMAND_GATE_BOOLEAN_CONTRACT = {
    "enable_drive_commands": True,
    "require_trajectory": True,
}


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    summary_root = LaunchConfiguration("summary_root")
    monitor_timeout_sec = LaunchConfiguration("monitor_timeout_sec")

    race_control_launch = PathJoinSubstitution(
        [
            FindPackageShare("autoracer_control"),
            "launch",
            "race_control.launch.py",
        ]
    )
    race_param_file = PathJoinSubstitution(
        [
            FindPackageShare("autoracer_control"),
            "config",
            "race_controller.param.yaml",
        ]
    )

    fixture_node = Node(
        package="autoracer_control",
        executable="race_bench_fixture_publisher",
        name="race_bench_fixture_publisher",
        output="screen",
        parameters=[
            {
                "scenario": scenario,
                "reference_trajectory_topic": BENCH_TOPICS["reference_trajectory_topic"],
                "odometry_topic": BENCH_TOPICS["odometry_topic"],
                "steering_topic": BENCH_TOPICS["steering_topic"],
                "accel_topic": BENCH_TOPICS["accel_topic"],
                "operation_mode_topic": BENCH_TOPICS["operation_mode_topic"],
                "pose_topic": BENCH_TOPICS["pose_topic"],
            }
        ],
    )

    command_gate_node = Node(
        package="autoracer_safety",
        executable="command_gate",
        name="command_gate",
        output="screen",
        parameters=[
            {
                "enable_drive_commands": ParameterValue(
                    COMMAND_GATE_BOOLEAN_CONTRACT["enable_drive_commands"], value_type=bool
                ),
                "input_topic": BENCH_TOPICS["raw_control_topic"],
                "output_topic": BENCH_TOPICS["final_control_topic"],
                "pose_topic": BENCH_TOPICS["pose_topic"],
                "trajectory_topic": BENCH_TOPICS["reference_trajectory_topic"],
                "require_trajectory": ParameterValue(
                    COMMAND_GATE_BOOLEAN_CONTRACT["require_trajectory"], value_type=bool
                ),
                "gear_topic": BENCH_TOPICS["gear_topic"],
                "hazard_topic": BENCH_TOPICS["hazard_topic"],
                "turn_topic": BENCH_TOPICS["turn_topic"],
                "state_topic": BENCH_TOPICS["state_topic"],
                "command_timeout_sec": 0.5,
                "localization_timeout_sec": 0.5,
                "trajectory_timeout_sec": 1.0,
            }
        ],
    )

    monitor_node = Node(
        package="autoracer_control",
        executable="race_bench_monitor",
        name="race_bench_monitor",
        output="screen",
        parameters=[
            {
                "scenario": scenario,
                "summary_root": summary_root,
                "monitor_timeout_sec": ParameterValue(monitor_timeout_sec, value_type=float),
                "namespace": "/control_bench",
                "raw_control_topic": BENCH_TOPICS["raw_control_topic"],
                "final_control_topic": BENCH_TOPICS["final_control_topic"],
                "default_final_topic": "/control/command/control_cmd",
                "operation_mode_topic": BENCH_TOPICS["operation_mode_topic"],
                "state_topic": BENCH_TOPICS["state_topic"],
                "command_gate_enable_drive_commands": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="straight"),
            DeclareLaunchArgument("summary_root", default_value="logs/race_control_bench"),
            DeclareLaunchArgument("monitor_timeout_sec", default_value="4.0"),
            GroupAction(
                [
                    PushRosNamespace("control_bench"),
                    fixture_node,
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(race_control_launch),
                        launch_arguments={
                            "reference_trajectory_topic": BENCH_TOPICS[
                                "reference_trajectory_topic"
                            ],
                            "odometry_topic": BENCH_TOPICS["odometry_topic"],
                            "steering_topic": BENCH_TOPICS["steering_topic"],
                            "accel_topic": BENCH_TOPICS["accel_topic"],
                            "operation_mode_topic": BENCH_TOPICS["operation_mode_topic"],
                            "raw_control_topic": BENCH_TOPICS["raw_control_topic"],
                            "race_param_file": race_param_file,
                        }.items(),
                    ),
                    command_gate_node,
                    monitor_node,
                ]
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=monitor_node,
                    on_exit=[Shutdown(reason="race_bench_monitor completed")],
                )
            ),
        ]
    )
