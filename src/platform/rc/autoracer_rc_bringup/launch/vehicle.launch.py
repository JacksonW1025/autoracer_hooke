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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    command_timeout_sec = LaunchConfiguration("command_timeout_sec")
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    feedback_rate_hz = LaunchConfiguration("feedback_rate_hz")
    enable_drive_commands = LaunchConfiguration("enable_drive_commands")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("command_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("command_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("feedback_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("enable_drive_commands", default_value="false"),
            Node(
                package="autoracer_rc_adapter",
                executable="rc_serial_interface",
                name="rc_serial_interface",
                output="screen",
                parameters=[
                    {
                        "device": ParameterValue(serial_port, value_type=str),
                        "baud": ParameterValue(baud, value_type=int),
                        "command_timeout_sec": ParameterValue(
                            command_timeout_sec, value_type=float
                        ),
                        "command_rate_hz": ParameterValue(command_rate_hz, value_type=float),
                        "feedback_rate_hz": ParameterValue(
                            feedback_rate_hz, value_type=float
                        ),
                        "enable_drive_commands": ParameterValue(
                            enable_drive_commands, value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
