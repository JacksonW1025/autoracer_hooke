from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from autoracer_control.control_closed_loop_scenarios import SCENARIO_SPECS, get_scenario_spec


BENCH_TOPICS = {
    "reference_trajectory_topic": "/control_bench/planning/trajectory",
    "operation_mode_topic": "/control_bench/system/operation_mode/state",
    "odometry_topic": "/control_bench/localization/kinematic_state",
    "steering_topic": "/control_bench/vehicle/status/steering_status",
    "accel_topic": "/control_bench/localization/acceleration",
    "raw_control_topic": "/control_bench/autoracer/control/raw_control_cmd",
}


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="straight_lateral_offset"),
            DeclareLaunchArgument("summary_root", default_value="logs/control_closed_loop"),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context, *args, **kwargs):
    scenario_name = LaunchConfiguration("scenario").perform(context)
    summary_root = LaunchConfiguration("summary_root")
    if scenario_name not in SCENARIO_SPECS:
        raise ValueError(f"unsupported closed-loop scenario: {scenario_name}")
    scenario_spec = get_scenario_spec(scenario_name)

    race_control_launch = PathJoinSubstitution(
        [FindPackageShare("autoracer_control"), "launch", "race_control.launch.py"]
    )
    race_param_file = PathJoinSubstitution(
        [
            FindPackageShare("autoracer_control"),
            "config",
            "race_controller.closed_loop_candidate.param.yaml",
        ]
    )

    fixture_node = Node(
        package="autoracer_control",
        executable="control_closed_loop_fixture_publisher",
        name="control_closed_loop_fixture_publisher",
        output="screen",
        parameters=[
            {
                "scenario": scenario_name,
                "reference_trajectory_topic": BENCH_TOPICS["reference_trajectory_topic"],
                "operation_mode_topic": BENCH_TOPICS["operation_mode_topic"],
            }
        ],
    )
    plant_node = Node(
        package="autoracer_control",
        executable="virtual_chassis_node",
        name="virtual_chassis_node",
        output="screen",
        parameters=[
            {
                "raw_control_topic": BENCH_TOPICS["raw_control_topic"],
                "odometry_topic": BENCH_TOPICS["odometry_topic"],
                "steering_topic": BENCH_TOPICS["steering_topic"],
                "acceleration_topic": BENCH_TOPICS["accel_topic"],
                "initial_x": 0.0,
                "initial_y": scenario_spec.initial_y,
                "initial_yaw": scenario_spec.initial_yaw,
                "initial_v": scenario_spec.initial_v,
                "initial_delta": 0.0,
                "initial_a": 0.0,
                "wheel_base": 1.9,
                "max_steer": 0.488,
                "max_steer_rate": 1.0,
                "steer_tau": 0.15,
                "actuator_input_delay": 0.15,
                "max_speed": scenario_spec.max_speed,
                "max_acc": 1.0,
                "min_acc": -2.0,
                "max_jerk": 2.0,
                "min_jerk": -4.0,
                "acc_tau": 0.20,
                "dt": 0.05,
                "fixed_speed_mode": False,
                "fixed_speed": scenario_spec.initial_v,
            }
        ],
    )
    monitor_node = Node(
        package="autoracer_control",
        executable="control_closed_loop_monitor",
        name="control_closed_loop_monitor",
        output="screen",
        on_exit=[Shutdown(reason="control_closed_loop_monitor completed")],
        parameters=[
            {
                "scenario": scenario_name,
                "scenario_type": scenario_spec.scenario_type,
                "summary_root": summary_root,
                "run_duration_sec": ParameterValue(
                    scenario_spec.max_duration_sec, value_type=float
                ),
                "max_duration_sec": ParameterValue(
                    scenario_spec.max_duration_sec, value_type=float
                ),
                "completion_threshold": ParameterValue(
                    scenario_spec.completion_threshold, value_type=float
                ),
                "trajectory_topic": BENCH_TOPICS["reference_trajectory_topic"],
                "operation_mode_topic": BENCH_TOPICS["operation_mode_topic"],
                "odometry_topic": BENCH_TOPICS["odometry_topic"],
                "steering_topic": BENCH_TOPICS["steering_topic"],
                "acceleration_topic": BENCH_TOPICS["accel_topic"],
                "raw_control_topic": BENCH_TOPICS["raw_control_topic"],
                "max_lat_acc_guardrail_mps2": 1.5,
                "max_steer": 0.488,
                "max_steer_rate": 1.0,
                "max_acc": 1.0,
                "min_acc": -2.0,
                "max_jerk": 2.0,
                "min_jerk": -4.0,
                "max_lateral_error_hard_m": 2.0,
                "longitudinal_validated": True,
            }
        ],
    )

    return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(race_control_launch),
                launch_arguments={
                    "reference_trajectory_topic": BENCH_TOPICS["reference_trajectory_topic"],
                    "odometry_topic": BENCH_TOPICS["odometry_topic"],
                    "steering_topic": BENCH_TOPICS["steering_topic"],
                    "accel_topic": BENCH_TOPICS["accel_topic"],
                    "operation_mode_topic": BENCH_TOPICS["operation_mode_topic"],
                    "raw_control_topic": BENCH_TOPICS["raw_control_topic"],
                    "race_param_file": race_param_file,
                }.items(),
            ),
            fixture_node,
            plant_node,
            monitor_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=monitor_node,
                    on_exit=[Shutdown(reason="control_closed_loop_monitor completed")],
                )
            ),
    ]
