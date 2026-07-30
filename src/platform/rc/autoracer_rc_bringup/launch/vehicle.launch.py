from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    telemetry_only = LaunchConfiguration("telemetry_only")
    vehicle_param_file = LaunchConfiguration("vehicle_param_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    package_share = get_package_share_directory("autoracer_rc_bringup")
    default_vehicle_param_file = PathJoinSubstitution(
        [package_share, "config", "rc", "vehicle.param.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/autoracer_rc_chassis",
                description="Stable udev alias for the RC chassis UART.",
            ),
            DeclareLaunchArgument(
                "vehicle_param_file", default_value=default_vehicle_param_file
            ),
            DeclareLaunchArgument(
                "telemetry_only",
                default_value="false",
                description="Read chassis telemetry without any command endpoint.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="autoracer_rc_adapter",
                executable="rc_vehicle_interface_node",
                name="rc_vehicle_interface",
                output="screen",
                parameters=[
                    ParameterFile(vehicle_param_file, allow_substs=True),
                    {
                        "serial_port": ParameterValue(serial_port, value_type=str),
                        "telemetry_only": ParameterValue(
                            telemetry_only, value_type=bool
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]
    )
