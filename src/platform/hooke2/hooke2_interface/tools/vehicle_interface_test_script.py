# Copyright 2024 The Shezhen Whaledynamic Ltc. Co. Authors. All Rights Reserved.
#
#  Software version: 2.7
#   
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess
import importlib
import requests
import sys
import os
import time

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[31m'
    ENDC = '\033[0m'  # reset color

def install_missing_libraries(library_name):
    try:
        importlib.import_module(library_name)
    except ModuleNotFoundError:
        print(f"library{bcolors.FAIL}---[{library_name}]---{bcolors.ENDC} not found. {bcolors.OKGREEN}Installing automatically...{bcolors.ENDC}")
        
        if is_in_china():
            subprocess.call(['pip', 'install', '-i', 'https://pypi.tuna.tsinghua.edu.cn/simple/', library_name])
        else:
            subprocess.call(['pip', 'install', library_name])

def is_in_china():
    try:
        response = requests.get('http://httpbin.org/ip')
        ip = response.json()['origin']
        if 'China' in response.json()['origin']:
            return True
        else:
            return False
    except:
        return False

# check and install missing libraries
install_missing_libraries("prettytable")
install_missing_libraries("pynput")

from enum import Enum
import time
import asyncio
import rclpy
import threading
import queue

# visualization data table
from prettytable import PrettyTable

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.duration import Duration

#   Driving mode : AUTO or MANUAL
from autoware_vehicle_msgs.srv import ControlModeCommand as ControlModeCommand_srv
#   Gear position : D, P, R, N
from autoware_vehicle_msgs.msg import GearCommand
#   
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_vehicle_msgs.msg import HazardLightsCommand

# Vehicle Status Messages - Add vehicle status message imports
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from autoware_vehicle_msgs.msg import HazardLightsReport

#   actuationCmd
from tier4_debug_msgs.msg import Float32Stamped
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped

# import keyboard
from pynput import keyboard

# Change the following params according to your own vehicle
FORWARD_CMD_INTERVAL = 0.02         # [0.0, 1.0]
BRAKE_CMD_INTERVAL = 0.02           # [0.0, 1.0]
STEERING_CMD_INTERVAL = 0.02        # [0.0, 1.0]
STEERING_SPD_CMD_INTERVAL = 0.02    # [0.0, 1.0]
MAX_STEERING_ANGLE = 7.98	    # unit: rad
STEERING_RATIO = 16.6		    # ratio value from steering wheel to ground wheel angle
MAX_SPEED_LIMIT = 5.0               # unit: m/s
MAX_ACC_LIMIT = 1.50                # unit: m/s^2
EMERGENCY_STOP_BRAKE = 0.70         # (0.0, 1.0]

# light status filter configuration parameters
LIGHT_FILTER_WINDOW_DURATION = 1.0   # detection time window(s)
LIGHT_FILTER_MIN_TOGGLES = 2         # minimum number of toggles to be considered as flickering


class ControlModeCommand_Constants(Enum):
    NO_COMMAND = 0
    AUTONOMOUS = 1
    AUTONOMOUS_STEER_ONLY = 2
    AUTONOMOUS_VELOCITY_ONLY = 3
    MANUAL = 4

class GearCommand_Constants(Enum):
    # Autoware Gear Status: NONE-0, N-1, D-(2~19), R-(20~21), P-22, LOW-(23~24)
    INVALID = 0
    NEUTRAL = 1
    DRIVE = 2
    REVERSE = 20
    PARK = 22

class TurnIndicatorsCommand_Constants(Enum):
    DISABLE = 1
    ENABLE_LEFT = 2
    ENABLE_RIGHT = 3


class ThreeColumnTablePrinter:
    def __init__(self):
        self.table = PrettyTable()
        self.table.field_names = ["Command", "Value", "Vehicle Status"]
        self.last_data = None           # cache last data, avoid unnecessary redraw
        self.needs_redraw = True        # mark if need to redraw
        self.clear_screen()

    def update_table(self, auto_mode, gear_cmd, forward_cmd, brake_cmd, steering_cmd, steering_spd_cmd, park_cmd, turn_signal, emergency, service_info, vehicle_status):
        # create hash of current data, check if there is any change
        current_data = (auto_mode, gear_cmd, forward_cmd, brake_cmd, steering_cmd, steering_spd_cmd, park_cmd, turn_signal, emergency, service_info, vehicle_status)
        
        # if data is not changed, do not redraw
        # if self.last_data == current_data:
        #     return False
        
        self.last_data = current_data
        self.needs_redraw = True
        self.table.clear_rows()
        
        # format command display data
        auto_mode_str = f"{auto_mode.name}"
        if auto_mode == ControlModeCommand_Constants.MANUAL:
            auto_mode_str = f"{bcolors.OKGREEN}{auto_mode_str}{bcolors.ENDC}"
        else:
            auto_mode_str = f"{bcolors.OKBLUE}{auto_mode_str}{bcolors.ENDC}"
            
        gear_str = f"{gear_cmd.name}"
        
        # optimize steering display
        if steering_cmd > 0:
            steering_cmd_str = f"{steering_cmd:.2f}"
        elif steering_cmd < 0:
            steering_cmd_str = f"{steering_cmd:.2f}"
        else:
            steering_cmd_str = "0.00"
        
        # turn signal display
        turn_signal_str = f"{turn_signal.name}"
        
        # emergency status display
        emergency_str = f"{bcolors.FAIL}True{bcolors.ENDC}" if emergency else f"{bcolors.OKGREEN}False{bcolors.ENDC}"
        
        # calculate speed command for display
        speed_cmd = forward_cmd * MAX_SPEED_LIMIT  # use actual MAX_SPEED_LIMIT constant
        
        # format turn signal status (fixed display when active, not affected by blinking)
        turn_signal_status = vehicle_status["turn_indicators"]
        if turn_signal_status == "Left":
            turn_signal_status_display = f"{bcolors.OKBLUE}Left{bcolors.ENDC}"
        elif turn_signal_status == "Right": 
            turn_signal_status_display = f"{bcolors.OKBLUE}Right{bcolors.ENDC}"
        else:
            turn_signal_status_display = turn_signal_status
        
        # add rows with command, value, and vehicle status
        self.table.add_row(["Auto Mode", auto_mode_str, vehicle_status["control_mode"]])
        self.table.add_row(["Gear", gear_str, vehicle_status["gear_status"]])
        self.table.add_row(["Speed", f"{speed_cmd:.2f} m/s", f"{vehicle_status['velocity']:.2f} m/s"])
        self.table.add_row(["Throttle", f"{forward_cmd:.2f}", f"{vehicle_status['accel_percentage']:.2f}"])
        self.table.add_row(["Brake", f"{brake_cmd:.2f}", f"{vehicle_status['brake_percentage']:.2f}"])
        self.table.add_row(["Steering", steering_cmd_str, f"{vehicle_status['steering_angle']*180.0/3.1415926:.2f}°"])
        self.table.add_row(["Steering Speed", f"{steering_spd_cmd:.2f}", f"{vehicle_status['steering_speed']:.2f}"])
        self.table.add_row(["Parking", park_cmd, vehicle_status["gear_status"]])
        self.table.add_row(["Turn Signal", turn_signal_str, turn_signal_status_display])
        self.table.add_row(["Emergency", emergency_str, vehicle_status["hazard_lights"]])
        self.table.add_row(["Service Status", service_info, "-"])
        
        return True  # data is updated

    def print_table(self):
        if not self.needs_redraw:
            return
            
        # clear screen and redraw
        self.clear_screen()
        print(self.table)
        print(f"\n{bcolors.HEADER}+--------+--------------------------------------------------+{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} Key    {bcolors.HEADER}|{bcolors.ENDC} Action Description                               {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}+--------+--------------------------------------------------+{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} Space  {bcolors.HEADER}|{bcolors.ENDC} Emergency stop                                   {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} W/w    {bcolors.HEADER}|{bcolors.ENDC} Increase throttle [0.0-1.0]                      {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} S/s    {bcolors.HEADER}|{bcolors.ENDC} Increase brake [0.0-1.0]                         {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} A/a    {bcolors.HEADER}|{bcolors.ENDC} Turn left                                        {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} D/d    {bcolors.HEADER}|{bcolors.ENDC} Turn right                                       {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} M/m    {bcolors.HEADER}|{bcolors.ENDC} Switch AUTO/MANUAL                               {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} G/g    {bcolors.HEADER}|{bcolors.ENDC} Switch gear: N→D→R→N                             {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} P/p    {bcolors.HEADER}|{bcolors.ENDC} Toggle parking                                   {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} Q/q    {bcolors.HEADER}|{bcolors.ENDC} Cycle turn signals                               {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} X/x    {bcolors.HEADER}|{bcolors.ENDC} Toggle debug mode                                {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}|{bcolors.ENDC} ESC    {bcolors.HEADER}|{bcolors.ENDC} Exit program                                     {bcolors.HEADER}|{bcolors.ENDC}")
        print(f"{bcolors.HEADER}+--------+--------------------------------------------------+{bcolors.ENDC}")
        
        self.needs_redraw = False

    def clear_screen(self):
        os.system('cls' if os.name == 'nt' else 'clear')

class VehicleInterfaceTestScript:
    def __init__(self):
        self.table_printer = ThreeColumnTablePrinter()
        self.keyboard_listener_run = True
        self.service_requests_interval = 5.0   #   unit[s]
        self.timer_period = 0.05        #   unit[s] reduce timer frequency to improve response
        self.user_input_key = None      
        self.auto_mode = ControlModeCommand_Constants.MANUAL
        self.gear_cmd = GearCommand_Constants.DRIVE
        self.forward_cmd = 0.00          #   uint[%], for throttle/acceleration/speed cmd, 0.0 - 1.0
        self.brake_cmd = 0.10            #   uint[%], for brakePadel/decceleration/hydPressure cmd, 0.0 - 1.0 (fix: initial value is 0.1)
        self.steering_cmd = 0.00         #   uint[%], for steering angle cmd, [0, 100]%
        self.steering_spd_cmd = 1.00     #   unit[%], for steering speed cmd, [0, 100]%
        self.park_cmd = "PARKING"        #   bool, for parking brake cmd
        self.turn_signal = TurnIndicatorsCommand_Constants.DISABLE
        self.harzad = HazardLightsCommand.DISABLE
        self.emergency = False           #   bool, for emergency stop cmd
        self.service_info = "Service not connected."
        self.debug_mode = False          #   bool, for debugging light status
        
        # Vehicle status data
        self.vehicle_status = {
            "control_mode": "Manual",
            "gear_status": "P", 
            "velocity": 0.0,
            "steering_angle": 0.0,
            "steering_speed": 1.0,
            "turn_indicators": "Disable",
            "hazard_lights": f"{bcolors.OKGREEN}False{bcolors.ENDC}",
            "accel_percentage": 0.0,
            "brake_percentage": 0.0,
            "steer_percentage": 0.0,
            "data_update_status": f"{bcolors.FAIL}No Data{bcolors.ENDC}"
        }
        
        # add state change flag, optimize publish frequency
        self.state_changed = True
        self.last_publish_time = time.time()
        
        # add vehicle status update flag for real-time display
        self.vehicle_status_changed = True
        self.last_display_update_time = time.time()

        # add light status filter variables - simplified approach
        self.light_status_filter = {
            "turn_indicators": {
                "display_status": "Disable",       # current display status
                "status_count": {"Left": 0, "Right": 0, "Disable": 0},  # count of each status in recent window
                "last_reset_time": time.time()     # last time counters were reset
            },
            "hazard_lights": {
                "display_status": f"{bcolors.OKGREEN}False{bcolors.ENDC}",
                "status_count": {f"{bcolors.FAIL}True{bcolors.ENDC}": 0, f"{bcolors.OKGREEN}False{bcolors.ENDC}": 0},
                "last_reset_time": time.time()
            }
        }

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.node = rclpy.create_node("Chassis_Testing_Script")

        #   create visualization table
        self.table = PrettyTable()
        self.table.field_names = ["Command", "Value"]

        # self.sub = self.node.create_subscription(
        #     ControlModeCommand,
        #     "/control/command/control_mode_cmd",
        #     self.control_mode_callback,
        #     10,
        # )
        # self.sub_gear = self.node.create_subscription(
        #     GearCommand,
        #     "/control/command/gear_cmd",
        #     self.gear_command_callback,
        #     10,
        # )
        # self.sub_gear = self.node.create_subscription(
        #     AckermannControlCommand,
        #     "/control/command/control_cmd",
        #     self.control_command_callback,
        #     10,
        # )

        self.pub_gear = self.node.create_publisher(
            GearCommand, "/control/command/gear_cmd", qos_profile
        )
        self.pub_control = self.node.create_publisher(
            Control, "/control/command/control_cmd", qos_profile
        )
        self.pub_actuation = self.node.create_publisher(
            ActuationCommandStamped, "/control/command/actuation_cmd", qos_profile
        )
        self.pub_turn_light = self.node.create_publisher(
            TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", qos_profile
        )
        self.pub_harzad_light = self.node.create_publisher(
            HazardLightsCommand, "/control/command/hazard_lights_cmd", qos_profile
        )
        self.pub_emergency = self.node.create_publisher(
            VehicleEmergencyStamped, "/control/command/emergency_cmd", qos_profile
        )

        # Vehicle Status Subscribers
        self.sub_velocity = self.node.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.velocity_status_callback, 10
        )
        self.sub_steering = self.node.create_subscription(
            SteeringReport, "/vehicle/status/steering_status", self.steering_status_callback, 10
        )
        self.sub_gear_status = self.node.create_subscription(
            GearReport, "/vehicle/status/gear_status", self.gear_status_callback, 10
        )
        self.sub_control_mode_status = self.node.create_subscription(
            ControlModeReport, "/vehicle/status/control_mode", self.control_mode_status_callback, 10
        )
        self.sub_turn_indicators_status = self.node.create_subscription(
            TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", self.turn_indicators_status_callback, 10
        )
        self.sub_hazard_lights_status = self.node.create_subscription(
            HazardLightsReport, "/vehicle/status/hazard_lights_status", self.hazard_lights_status_callback, 10
        )
        self.sub_actuation_status = self.node.create_subscription(
            ActuationStatusStamped, "/vehicle/status/actuation_status", self.actuation_status_callback, 10
        )

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        self.timer = self.node.create_timer(self.timer_period, self.keyboard_timer_callback)
        self.control_mode_client = self.node.create_client(ControlModeCommand_srv, "/control/control_mode_request")
    
    def keyboard_timer_callback(self):
        # handle keyboard input
        user_input_key = self.user_input_key
        self.user_input_key = None

        if user_input_key is not None:
            self._handle_key_input(user_input_key)
            
        # check if display needs update (either command changed or vehicle status changed or periodic refresh)
        current_time = time.time()
        should_update_display = (current_time - self.last_display_update_time > 0.1)  # force refresh every 0.1 second

        if should_update_display:
            # check and update light status filter (handle timeout)
            self._update_light_status_filters()
            
            # update display
            data_changed = self.table_printer.update_table(
                self.auto_mode, self.gear_cmd, self.forward_cmd, self.brake_cmd, 
                self.steering_cmd, self.steering_spd_cmd, self.park_cmd, 
                self.turn_signal, self.emergency, self.service_info, self.vehicle_status
            )
            # redraw table when data is changed or display needs refresh
            if data_changed:
                self.table_printer.print_table()
                self.last_display_update_time = current_time
                self.vehicle_status_changed = False

    def _handle_key_input(self, key):
        """handle keyboard input - independent method to improve response"""
        if key == keyboard.Key.esc:
            self.keyboard_listener_run = False
            print("ESC pressed, exiting...")
            return
        elif key == keyboard.Key.space:  # space key
            self.switch_emergency_stop()
            self.state_changed = True
        elif not self.emergency:
            key_str = str(key)
            if key_str == "'w'":
                self.forward_cmd = min(self.forward_cmd + FORWARD_CMD_INTERVAL, 1.0)
                self.brake_cmd = 0.0
                self.state_changed = True
            elif key_str == "'s'":
                self.forward_cmd = 0.0
                self.brake_cmd = min(self.brake_cmd + BRAKE_CMD_INTERVAL, 1.0)
                self.state_changed = True
            elif key_str == "'a'":
                self.steering_cmd = min(self.steering_cmd + STEERING_CMD_INTERVAL, 1.0)
                self.state_changed = True
            elif key_str == "'d'":
                self.steering_cmd = max(self.steering_cmd - STEERING_CMD_INTERVAL, -1.0)
                self.state_changed = True
            elif key_str == "'m'":
                self.switch_auto_mode()
                self.state_changed = True
            elif key_str == "'g'":
                self.switch_gear_position()
                self.state_changed = True
            elif key_str == "'p'":
                self.set_parking()
                self.state_changed = True
            elif key_str == "'q'":
                self.switch_turn_signal()
                self.state_changed = True
            elif key_str == "'x'":
                self.debug_mode = not self.debug_mode
                print(f"Debug mode: {self.debug_mode}")
                self.state_changed = True
                # elif user_input_key == 'l':
                #     self.toggle_low_beam()
                # elif user_input_key == '0':
                #     self.reset_action()
                # elif user_input_key == '1':
                #     self.start_action()
                # elif user_input_key == '2':
                #     self.vin_req_action()

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.keyboard_listener_run = False
            print("ESC pressed, exiting...")
            return False
        if not self.keyboard_listener_run:
            return False
    
    def on_press(self, key):
        try:
            self.user_input_key = key
        except AttributeError:
            print('special key {0} pressed'.format(
                key))

    def switch_gear_position(self):
        #   only switch between N, D, R
        gear_cmd = self.gear_cmd
        if gear_cmd == GearCommand_Constants.NEUTRAL:
            self.gear_cmd = GearCommand_Constants.DRIVE
        elif gear_cmd == GearCommand_Constants.DRIVE:
            self.gear_cmd = GearCommand_Constants.REVERSE
        elif gear_cmd == GearCommand_Constants.REVERSE:
            self.gear_cmd = GearCommand_Constants.NEUTRAL
        else:
            self.gear_cmd = GearCommand_Constants.NEUTRAL
    
    def switch_auto_mode(self):
        #   only switch between auto/manual
        auto_mode_cmd = self.auto_mode
        if auto_mode_cmd == ControlModeCommand_Constants.MANUAL:
            self.auto_mode = ControlModeCommand_Constants.AUTONOMOUS
        elif auto_mode_cmd == ControlModeCommand_Constants.AUTONOMOUS:
            self.auto_mode = ControlModeCommand_Constants.MANUAL
    
    def set_parking(self):
        #   do nothing, just toggle parking
        if self.gear_cmd != GearCommand_Constants.PARK:
            self.park_cmd = "PARKING"
            self.gear_cmd = GearCommand_Constants.PARK
        else:
            self.park_cmd = "PARKING_RELEASE"
            self.gear_cmd = GearCommand_Constants.NEUTRAL
    
    def switch_turn_signal(self):
        turn_signal_cmd = self.turn_signal
        if turn_signal_cmd == TurnIndicatorsCommand_Constants.DISABLE:
            self.turn_signal = TurnIndicatorsCommand_Constants.ENABLE_LEFT
        elif turn_signal_cmd == TurnIndicatorsCommand_Constants.ENABLE_LEFT:
            self.turn_signal = TurnIndicatorsCommand_Constants.ENABLE_RIGHT
        elif turn_signal_cmd == TurnIndicatorsCommand_Constants.ENABLE_RIGHT:
            self.turn_signal = TurnIndicatorsCommand_Constants.DISABLE
        else:
            self.turn_signal = TurnIndicatorsCommand_Constants.DISABLE

    def switch_emergency_stop(self):
        if self.emergency == False :
            self.forward_cmd = 0.0
            self.brake_cmd = EMERGENCY_STOP_BRAKE
            self.emergency = True
            self.harzad = HazardLightsCommand.ENABLE
        else :
            self.forward_cmd = 0.0
            self.brake_cmd = 0.0
            self.emergency = False
            self.harzad = HazardLightsCommand.DISABLE

    def _filter_light_status(self, light_type, new_status):
        """
        simplified light status filter using counting approach
        Args:
            light_type: "turn_indicators" or "hazard_lights"
            new_status: new status value
        Returns:
            filtered_status: filtered display status
        """
        current_time = time.time()
        filter_info = self.light_status_filter[light_type]
        
        # reset counters every second
        if current_time - filter_info["last_reset_time"] >= 1.0:
            for key in filter_info["status_count"]:
                filter_info["status_count"][key] = 0
            filter_info["last_reset_time"] = current_time
        
        # increment counter for current status
        if new_status in filter_info["status_count"]:
            filter_info["status_count"][new_status] += 1
        
        # determine display status based on counts
        if light_type == "turn_indicators":
            left_count = filter_info["status_count"]["Left"]
            right_count = filter_info["status_count"]["Right"]
            disable_count = filter_info["status_count"]["Disable"]
            
            # if we have any left signals and no right signals, show left
            if left_count > 0 and right_count == 0:
                filter_info["display_status"] = "Left"
            # if we have any right signals and no left signals, show right
            elif right_count > 0 and left_count == 0:
                filter_info["display_status"] = "Right"
            # if we have only disable signals, show disable
            elif disable_count > 0 and left_count == 0 and right_count == 0:
                filter_info["display_status"] = "Disable"
            # for mixed signals, keep the current status (don't change)
            
        elif light_type == "hazard_lights":
            true_status = f"{bcolors.FAIL}True{bcolors.ENDC}"
            false_status = f"{bcolors.OKGREEN}False{bcolors.ENDC}"
            true_count = filter_info["status_count"][true_status]
            false_count = filter_info["status_count"][false_status]
            
            # if we have any true signals, hazard is on
            if true_count > 0:
                filter_info["display_status"] = true_status
            # if we only have false signals, hazard is off
            elif false_count > 0 and true_count == 0:
                filter_info["display_status"] = false_status
        
        return filter_info["display_status"]



    def _update_light_status_filters(self):
        """
        cleanup - no longer needed with simplified counting approach
        """
        pass

    # Vehicle Status Callback Functions
    def velocity_status_callback(self, msg):
        """Callback for vehicle velocity status"""
        self.vehicle_status["velocity"] = msg.longitudinal_velocity
        self.vehicle_status["data_update_status"] = f"{bcolors.OKGREEN}Data OK{bcolors.ENDC}"
        self.vehicle_status_changed = True
    
    def steering_status_callback(self, msg):
        """Callback for vehicle steering status"""
        self.vehicle_status["steering_angle"] = msg.steering_tire_angle
        self.vehicle_status["data_update_status"] = f"{bcolors.OKGREEN}Data OK{bcolors.ENDC}"
        self.vehicle_status_changed = True
    
    def gear_status_callback(self, msg):
        """Callback for vehicle gear status"""
        gear_map = {
            0: "None", 1: "Neutral", 2: "Drive", 20: "Reverse", 22: "Park"
        }
        self.vehicle_status["gear_status"] = gear_map.get(msg.report, "None")
        self.vehicle_status["data_update_status"] = f"{bcolors.OKGREEN}Data OK{bcolors.ENDC}"
        self.vehicle_status_changed = True
    
    def control_mode_status_callback(self, msg):
        """Callback for vehicle control mode status"""
        mode_map = {
            4: "Manual", 1: "Autonomous"
        }
        self.vehicle_status["control_mode"] = mode_map.get(msg.mode, "Manual")
        self.vehicle_status["data_update_status"] = f"{bcolors.OKGREEN}Data OK{bcolors.ENDC}"
        self.vehicle_status_changed = True
    
    def turn_indicators_status_callback(self, msg):
        """Callback for vehicle turn indicators status"""
        # determine the original status
        if msg.report == 2:  # Left turn signal active (including blinking state)
            raw_status = "Left"
        elif msg.report == 3:  # Right turn signal active (including blinking state)
            raw_status = "Right" 
        elif msg.report == 1:  # Disabled
            raw_status = "Disable"
        else:
            raw_status = "Disable"
        
        # debug output
        if self.debug_mode:
            print(f"DEBUG - Turn signal raw: {msg.report} -> {raw_status}")
        
        # use the status filter to get the stable status
        filtered_status = self._filter_light_status("turn_indicators", raw_status)
        
        # debug output
        if self.debug_mode:
            counts = self.light_status_filter["turn_indicators"]["status_count"]
            print(f"DEBUG - Turn signal filtered: {filtered_status}, counts: {counts}")
        
        # update vehicle status
        self.vehicle_status["turn_indicators"] = filtered_status
        self.vehicle_status_changed = True
        self.vehicle_status["data_update_status"] = f"{bcolors.OKGREEN}Data OK{bcolors.ENDC}"
    
    def hazard_lights_status_callback(self, msg):
        """Callback for vehicle hazard lights status"""
        # determine the original status
        raw_status = f"{bcolors.FAIL}True{bcolors.ENDC}" if msg.report == 2 else f"{bcolors.OKGREEN}False{bcolors.ENDC}"
        
        # use the status filter to get the stable status
        filtered_status = self._filter_light_status("hazard_lights", raw_status)
        
        # update vehicle status
        self.vehicle_status["hazard_lights"] = filtered_status
        self.vehicle_status_changed = True
        self.vehicle_status["data_update_status"] = f"{bcolors.OKGREEN}Data OK{bcolors.ENDC}"
    
    def actuation_status_callback(self, msg):
        """Callback for vehicle actuation status"""
        self.vehicle_status["accel_percentage"] = msg.status.accel_status
        self.vehicle_status["brake_percentage"] = msg.status.brake_status  
        self.vehicle_status["steer_percentage"] = msg.status.steer_status
        self.vehicle_status["data_update_status"] = f"{bcolors.OKGREEN}Data OK{bcolors.ENDC}"
        self.vehicle_status_changed = True

    def set_gear_command(self, gear_mode):
        msg = self.generate_gear_cmd_msg(gear_mode)
        self.pub_gear.publish(msg)

    def set_control_command(self, control_cmd):
        msg = self.generate_control_msg(control_cmd)
        self.pub_control.publish(msg)

    def set_actuation_command(self, accel_cmd, brake_cmd, steer_cmd):
        msg_actuation = self.generate_actuation_cmd_msg(accel_cmd, brake_cmd, steer_cmd)
        self.pub_actuation.publish(msg_actuation)
    
    def set_turn_light_command(self, turn_signal):
        msg_turn_signal = self.generate_turn_light_cmd_msg(turn_signal)
        self.pub_turn_light.publish(msg_turn_signal)

    def set_emergency(self):
        msg_hazard = self.generate_hazard_light_cmd_msg(self.harzad)
        self.pub_harzad_light.publish(msg_hazard)

        msg_emergency = self.generate_emergency_cmd_msg(self.emergency)
        self.pub_emergency.publish(msg_emergency)

    def generate_actuation_cmd_msg(self, accel_cmd, brake_cmd, steer_cmd):
        msg_actuation_cmd = ActuationCommandStamped()
        msg_actuation_cmd.header.stamp = self.node.get_clock().now().to_msg()
        msg_actuation_cmd.actuation.accel_cmd = accel_cmd
        msg_actuation_cmd.actuation.brake_cmd = brake_cmd
        msg_actuation_cmd.actuation.steer_cmd = steer_cmd
        return msg_actuation_cmd
    
    def generate_turn_light_cmd_msg(self, turn_signal):
        msg_turn_signal = TurnIndicatorsCommand()
        msg_turn_signal.stamp = self.node.get_clock().now().to_msg()
        if turn_signal == TurnIndicatorsCommand_Constants.ENABLE_LEFT:
            msg_turn_signal.command = TurnIndicatorsCommand.ENABLE_LEFT
        elif turn_signal == TurnIndicatorsCommand_Constants.ENABLE_RIGHT:
            msg_turn_signal.command = TurnIndicatorsCommand.ENABLE_RIGHT
        else:
            msg_turn_signal.command = TurnIndicatorsCommand.DISABLE
        return msg_turn_signal
    
    def generate_hazard_light_cmd_msg(self, hazard_signal):
        msg_hazard_signal = HazardLightsCommand()
        msg_hazard_signal.stamp = self.node.get_clock().now().to_msg()
        msg_hazard_signal.command = hazard_signal
        return msg_hazard_signal

    def generate_emergency_cmd_msg(self, emergency):
        msg_emergency = VehicleEmergencyStamped()
        msg_emergency.stamp = self.node.get_clock().now().to_msg()
        msg_emergency.emergency = emergency
        return msg_emergency
    
    def generate_gear_cmd_msg(self, gear_mode):
        stamp = self.node.get_clock().now().to_msg()
        msg = GearCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = gear_mode.value
        return msg
    
    def generate_control_msg(self, control_cmd):
        stamp = self.node.get_clock().now().to_msg()
        msg = Control()
        lateral_cmd = Control().lateral
        longitudinal_cmd = Control().longitudinal

        lateral_cmd.stamp.sec = stamp.sec
        lateral_cmd.stamp.nanosec = stamp.nanosec
        lateral_cmd.steering_tire_angle = control_cmd["lateral"]["steering_tire_angle"] * MAX_STEERING_ANGLE / STEERING_RATIO
        lateral_cmd.steering_tire_rotation_rate = control_cmd["lateral"]["steering_tire_rotation_rate"]
        
        longitudinal_cmd.stamp.sec = stamp.sec
        longitudinal_cmd.stamp.nanosec = stamp.nanosec
        longitudinal_cmd.velocity = control_cmd["longitudinal"]["velocity"]
        longitudinal_cmd.acceleration = control_cmd["longitudinal"]["acceleration"]
        longitudinal_cmd.jerk = control_cmd["longitudinal"]["jerk"]

        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.lateral = lateral_cmd
        msg.longitudinal = longitudinal_cmd
        return msg
    
    def call_control_mode_sync(self, mode):
        """synchronize call control mode service - fix thread problem"""
        try:
            request = ControlModeCommand_srv.Request()
            request.mode = mode
            future = self.control_mode_client.call_async(request)
            
            # use rclpy.spin_until_future_complete instead of manual thread management
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)

            if future.result() is not None:
                # self.node.get_logger().info('Service call success: {}'.format(future.result()))
                return True
            else:
                self.node.get_logger().warn('Service call failed: no result')
                return False
        except Exception as e:
            self.node.get_logger().error('Service call exception: {}'.format(str(e)))
            return False

    def call_mode_request_periodically(self, mode):
        """call mode request periodically - add exception handling"""
        try:
            if self.control_mode_client.service_is_ready():
                self.call_control_mode_sync(mode.value)
            else:
                self.node.get_logger().debug('Control mode service not ready')
        except Exception as e:
            self.node.get_logger().error('Mode request failed: {}'.format(str(e)))

    def close_control_mode_client(self):
        self.control_mode_client.destroy()

def main(args=None):
    rclpy.init(args=args)

    test_instance = None  # initialize test_instance
    test_instance = VehicleInterfaceTestScript()
    
    # timer for periodic service requests
    service_timer = None
    def timer_callback():
        if test_instance and test_instance.keyboard_listener_run:
            test_instance.call_mode_request_periodically(test_instance.auto_mode)
            service_timer = threading.Timer(test_instance.service_requests_interval, timer_callback)
            service_timer.start()
    
    # start timer
    service_timer = threading.Timer(test_instance.service_requests_interval, timer_callback)
    service_timer.start()

    # get user keyboard input
    key_listener = keyboard.Listener(
            on_press=test_instance.on_press, on_release=test_instance.on_release)
    key_listener.start()

    try:
        service_check_counter = 0
        while rclpy.ok() and test_instance.keyboard_listener_run:
            # reduce service check frequency, check every 50 loops
            service_check_counter += 1
            if service_check_counter >= 50:
                if not test_instance.control_mode_client.wait_for_service(timeout_sec=0.1):
                    test_instance.service_info = f"{bcolors.FAIL}vehicle canbus not available{bcolors.ENDC}"
                else:
                    test_instance.service_info = f"{bcolors.OKGREEN}vehicle canbus connected!{bcolors.ENDC}"
                service_check_counter = 0

            # check if we should exit
            if not test_instance.keyboard_listener_run:
                break
                
            # ROS node processing
            rclpy.spin_once(test_instance.node, timeout_sec=0.001)

            # only send message when state changes or periodically publishes
            current_time = time.time()
            should_publish = test_instance.state_changed or (current_time - test_instance.last_publish_time > 0.1)
            
            if should_publish:
                #   ROS2 publisher, send GEAR cmd
                test_instance.set_gear_command(test_instance.gear_cmd)

                #   ROS2 publisher, send lateral/longitudinal cmd
                control_cmd = {
                    "lateral": {
                        "steering_tire_angle": test_instance.steering_cmd,
                        "steering_tire_rotation_rate": test_instance.steering_spd_cmd,
                    },
                    "longitudinal": {
                        "velocity": test_instance.forward_cmd * MAX_SPEED_LIMIT,
                        "acceleration": test_instance.forward_cmd * MAX_ACC_LIMIT,
                        "jerk": 0.1,
                    }
                }
                test_instance.set_control_command(control_cmd)
                
                #   ROS2 publisher, send actuation cmd
                test_instance.set_actuation_command(test_instance.forward_cmd, test_instance.brake_cmd, test_instance.steering_cmd)

                test_instance.set_turn_light_command(test_instance.turn_signal)
                test_instance.set_emergency()
                
                test_instance.state_changed = False
                test_instance.last_publish_time = current_time

            time.sleep(0.005)  # reduce to 5ms, improve response speed

    except KeyboardInterrupt:
        test_instance.keyboard_listener_run = False
        key_listener.stop()
        if service_timer is not None and service_timer.is_alive():
            service_timer.cancel()  # cancel timer execution
        if test_instance is not None:
            test_instance.node.destroy_node()  # destroy node
        print("Keyboard interrupt, cleaning up and exiting...")
    except Exception as e:
        print(f"An exception occurred: {e}")
    finally:
        if test_instance is not None:
            test_instance.close_control_mode_client()  # explicitly close control_mode_client

        if rclpy.ok():
            rclpy.shutdown()  # manually close ROS node

if __name__ == '__main__':
    main()
