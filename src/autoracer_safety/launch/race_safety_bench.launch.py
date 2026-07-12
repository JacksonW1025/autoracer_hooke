import signal

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events.process import SignalProcess, matches_executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    summary_path = LaunchConfiguration("summary_path")
    drop_after_sec = LaunchConfiguration("drop_after_sec")
    drop_duration_sec = LaunchConfiguration("drop_duration_sec")
    safety_launch = PathJoinSubstitution(
        [FindPackageShare("autoracer_safety"), "launch", "race_safety.launch.py"]
    )
    fixture = Node(
        package="autoracer_safety",
        executable="race_stack_fixture",
        name="race_stack_fixture",
        output="screen",
        parameters=[
            {
                "drop_topic": scenario,
                "drop_after_sec": ParameterValue(drop_after_sec, value_type=float),
                "drop_duration_sec": ParameterValue(drop_duration_sec, value_type=float),
                "use_sim_time": False,
            }
        ],
    )
    monitor = Node(
        package="autoracer_safety",
        executable="race_stack_monitor",
        name="race_stack_monitor",
        output="screen",
        parameters=[
            {
                "scenario": scenario,
                "summary_path": summary_path,
                "drop_after_sec": ParameterValue(drop_after_sec, value_type=float),
                "drop_duration_sec": ParameterValue(drop_duration_sec, value_type=float),
                "use_sim_time": False,
            }
        ],
    )

    def fault_process(context):
        process_names = {"manager": "race_runtime_manager"}
        process_name = process_names.get(scenario.perform(context))
        if process_name is None:
            return []
        return [
            TimerAction(
                period=float(drop_after_sec.perform(context)),
                actions=[
                    EmitEvent(
                        event=SignalProcess(
                            signal_number=signal.SIGTERM,
                            process_matcher=matches_executable(process_name),
                        )
                    )
                ],
            )
        ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="normal"),
            DeclareLaunchArgument("summary_path", default_value="/tmp/race_stack_summary.json"),
            DeclareLaunchArgument("drop_after_sec", default_value="5.0"),
            DeclareLaunchArgument("drop_duration_sec", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(safety_launch),
                launch_arguments={"use_sim_time": "false"}.items(),
            ),
            fixture,
            monitor,
            OpaqueFunction(function=fault_process),
            RegisterEventHandler(
                OnProcessExit(target_action=monitor, on_exit=[Shutdown(reason="bench completed")])
            ),
        ]
    )
