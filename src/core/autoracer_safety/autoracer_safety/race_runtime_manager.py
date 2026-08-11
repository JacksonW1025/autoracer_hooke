from __future__ import annotations

from enum import IntEnum
import json
import time

from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    MrmState,
    OperationModeState,
)
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import RouteState, Trajectory
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    Engage,
    GearCommand,
    GearReport,
    HazardLightsCommand,
    SteeringReport,
    TurnIndicatorsCommand,
    VelocityReport,
)
from autoware_vehicle_msgs.srv import ControlModeCommand
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tier4_control_msgs.msg import GateMode
from tier4_external_api_msgs.srv import Engage as EngageService

from autoracer_safety.race_contract import COMMAND_QOS, STATE_QOS, TimedInput


class RuntimePhase(IntEnum):
    IDLE = 0
    READY = 1
    ARMING = 2
    ACTIVE = 3
    STOPPING = 4
    FINISHED = 5
    FAULT = 6


def desired_gear_command(
    phase: RuntimePhase, speed_mps: float, stop_speed_mps: float
) -> int:
    driving = phase in (RuntimePhase.ARMING, RuntimePhase.ACTIVE) or abs(
        speed_mps
    ) >= stop_speed_mps
    return GearCommand.DRIVE if driving else GearCommand.PARK


class RaceRuntimeManager(Node):
    def __init__(self) -> None:
        super().__init__("race_runtime_manager")
        defaults = {
            "update_rate_hz": 20.0,
            "auto_start": True,
            "startup_timeout_sec": 90.0,
            "localization_timeout_sec": 0.20,
            "require_ndt_pose_health": False,
            "ndt_pose_timeout_sec": 0.50,
            "ndt_pose_source_timeout_sec": 0.50,
            "trajectory_timeout_sec": 0.35,
            "control_timeout_sec": 0.20,
            "vehicle_status_timeout_sec": 0.25,
            "stop_speed_mps": 0.10,
            "service_retry_sec": 0.25,
            "emergency_acceleration_mps2": -2.4,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._phase = RuntimePhase.IDLE
        self._phase_since = self._now()
        self._start_sec = self._phase_since
        self._last_now = -1.0
        self._last_timer_wall_sec = -1.0
        self._last_timer_gap_wall_sec = 0.0
        self._max_timer_gap_wall_sec = 0.0
        self._last_timer_duration_wall_sec = 0.0
        self._max_timer_duration_wall_sec = 0.0
        self._timer_overrun_count = 0
        self._start_requested = bool(self.get_parameter("auto_start").value)
        self._stop_requested = False
        self._activation_armed = False
        self._activation_stamp = -1.0
        self._last_service_call = {"control_mode": -1.0, "engage": -1.0}
        self._pending = {"control_mode": None, "engage": None}
        self._reason = "WAITING_FOR_START"

        self._localization_state = TimedInput()
        self._odometry = TimedInput()
        self._ndt_pose = TimedInput()
        self._trajectory = TimedInput()
        self._route_state = TimedInput()
        self._raw_control = TimedInput()
        self._final_control = TimedInput()
        self._control_mode = TimedInput()
        self._gear = TimedInput()
        self._velocity = TimedInput()
        self._steering = TimedInput()
        self._engage = TimedInput()
        self._timed_inputs = (
            self._localization_state,
            self._odometry,
            self._ndt_pose,
            self._trajectory,
            self._route_state,
            self._raw_control,
            self._final_control,
            self._control_mode,
            self._gear,
            self._velocity,
            self._steering,
            self._engage,
        )
        # Input callbacks must be able to drain while the health timer evaluates
        # the previous snapshot.  Keeping the state machine in a separate
        # mutually-exclusive group preserves serialized transitions and service
        # handling while eliminating timer-before-input false stale decisions.
        self._input_callback_group = ReentrantCallbackGroup()
        self._state_callback_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            lambda message: self._localization_state.update(
                message, self._now(), message.stamp
            ),
            STATE_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            lambda message: self._odometry.update(
                message, self._now(), message.header.stamp
            ),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        # The EKF can continue publishing a numerically fresh prediction from
        # IMU/wheel speed after absolute NDT updates disappear.  Track the raw
        # NDT observation separately so the existing MRM path can fail safe;
        # this is health supervision, not a second estimator or pose output.
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/ndt_scan_matcher/pose_with_covariance",
            lambda message: self._ndt_pose.update(
                message, self._now(), message.header.stamp
            ),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            Trajectory,
            "/planning/trajectory",
            lambda message: self._trajectory.update(
                message, self._now(), message.header.stamp
            ),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            RouteState,
            "/planning/route_state",
            lambda message: self._route_state.update(message, self._now(), message.stamp),
            STATE_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            Control,
            "/control/trajectory_follower/control_cmd",
            lambda message: self._raw_control.update(message, self._now(), message.stamp),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            Control,
            "/control/command/control_cmd",
            lambda message: self._final_control.update(
                message, self._now(), message.stamp
            ),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            lambda message: self._control_mode.update(
                message, self._now(), message.stamp
            ),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            GearReport,
            "/vehicle/status/gear_status",
            lambda message: self._gear.update(message, self._now(), message.stamp),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            lambda message: self._velocity.update(
                message, self._now(), message.header.stamp
            ),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            SteeringReport,
            "/vehicle/status/steering_status",
            lambda message: self._steering.update(
                message, self._now(), message.stamp
            ),
            COMMAND_QOS,
            callback_group=self._input_callback_group,
        )
        self.create_subscription(
            Engage,
            "/api/autoware/get/engage",
            lambda message: self._engage.update(message, self._now(), message.stamp),
            STATE_QOS,
            callback_group=self._input_callback_group,
        )

        self._operation_mode_pub = self.create_publisher(
            OperationModeState, "/system/operation_mode/state", STATE_QOS
        )
        self._state_pub = self.create_publisher(
            String, "/system/race_runtime/state", STATE_QOS
        )
        self._gate_mode_pub = self.create_publisher(
            GateMode, "/control/gate_mode_cmd", STATE_QOS
        )
        self._gear_pub = self.create_publisher(
            GearCommand, "/control/race_runtime/gear_cmd", COMMAND_QOS
        )
        self._turn_pub = self.create_publisher(
            TurnIndicatorsCommand,
            "/control/race_runtime/turn_indicators_cmd",
            COMMAND_QOS,
        )
        self._hazard_pub = self.create_publisher(
            HazardLightsCommand,
            "/control/race_runtime/hazard_lights_cmd",
            COMMAND_QOS,
        )
        self._mrm_pub = self.create_publisher(
            MrmState, "/system/fail_safe/mrm_state", COMMAND_QOS
        )
        self._emergency_control_pub = self.create_publisher(
            Control, "/system/emergency/control_cmd", COMMAND_QOS
        )
        self._emergency_gear_pub = self.create_publisher(
            GearCommand, "/system/emergency/gear_cmd", COMMAND_QOS
        )
        self._emergency_turn_pub = self.create_publisher(
            TurnIndicatorsCommand,
            "/system/emergency/turn_indicators_cmd",
            COMMAND_QOS,
        )
        self._emergency_hazard_pub = self.create_publisher(
            HazardLightsCommand,
            "/system/emergency/hazard_lights_cmd",
            COMMAND_QOS,
        )

        self._control_mode_client = self.create_client(
            ControlModeCommand,
            "/control/control_mode_request",
            callback_group=self._state_callback_group,
        )
        self._engage_client = self.create_client(
            EngageService,
            "/api/autoware/set/engage",
            callback_group=self._state_callback_group,
        )
        self.create_service(
            Trigger,
            "/autoracer/race/start",
            self._on_start,
            callback_group=self._state_callback_group,
        )
        self.create_service(
            Trigger,
            "/autoracer/race/stop",
            self._on_stop,
            callback_group=self._state_callback_group,
        )
        self.create_service(
            Trigger,
            "/autoracer/race/reset",
            self._on_reset,
            callback_group=self._state_callback_group,
        )
        self.create_timer(
            1.0 / float(self.get_parameter("update_rate_hz").value),
            self._on_timer,
            callback_group=self._state_callback_group,
        )
        self.get_logger().info(
            "race runtime stale-input witness instrumentation=V1 "
            "executor=multi_threaded_4 callbacks=state_serial_input_reentrant"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _fresh(
        self,
        tracker: TimedInput,
        receipt_timeout_parameter: str,
        source_timeout_parameter: str | None = None,
    ) -> bool:
        receipt_timeout = float(
            self.get_parameter(receipt_timeout_parameter).value
        )
        source_timeout = (
            None
            if source_timeout_parameter is None
            else float(self.get_parameter(source_timeout_parameter).value)
        )
        return tracker.fresh(self._now(), receipt_timeout, source_timeout)

    def _ndt_fresh(self) -> bool:
        return self._fresh(
            self._ndt_pose,
            "ndt_pose_timeout_sec",
            "ndt_pose_source_timeout_sec",
        )

    def _on_start(self, request, response):
        del request
        if self._phase not in (RuntimePhase.IDLE, RuntimePhase.READY):
            response.success = False
            response.message = f"cannot start from {self._phase.name}"
            return response
        self._start_requested = True
        response.success = True
        response.message = "start accepted"
        return response

    def _on_stop(self, request, response):
        del request
        self._stop_requested = True
        response.success = True
        response.message = "stop accepted"
        return response

    def _on_reset(self, request, response):
        del request
        if abs(self._speed_mps()) >= float(self.get_parameter("stop_speed_mps").value):
            response.success = False
            response.message = "vehicle must be stopped before reset"
            return response
        if self._control_mode.message is not None and self._control_mode.message.mode not in (
            ControlModeReport.MANUAL,
            ControlModeReport.NO_COMMAND,
        ):
            response.success = False
            response.message = "control must be manual before reset"
            return response
        if self._engage.message is not None and bool(self._engage.message.engage):
            response.success = False
            response.message = "vehicle must be disengaged before reset"
            return response
        self._reset_state(clear_inputs=False)
        response.success = True
        response.message = "reset accepted"
        return response

    def _reset_state(self, clear_inputs: bool) -> None:
        self._phase = RuntimePhase.IDLE
        self._phase_since = self._now()
        self._start_sec = self._phase_since
        self._start_requested = bool(self.get_parameter("auto_start").value)
        self._stop_requested = False
        self._activation_armed = False
        self._activation_stamp = -1.0
        self._pending = {"control_mode": None, "engage": None}
        self._reason = "RESET"
        if clear_inputs:
            for tracker in self._timed_inputs:
                tracker.clear()

    def _transition(self, phase: RuntimePhase, reason: str) -> None:
        if phase != self._phase:
            self.get_logger().info(
                f"race runtime {self._phase.name} -> {phase.name}: {reason}"
            )
            self._phase = phase
            self._phase_since = self._now()
        self._reason = reason

    def _stale_input_witness(
        self,
        tracker: TimedInput,
        receipt_timeout_parameter: str,
        source_timeout_parameter: str | None = None,
    ) -> str:
        """Serialize the exact evidence used by a stale-input fault decision."""
        now = self._now()
        receipt_timeout = float(
            self.get_parameter(receipt_timeout_parameter).value
        )
        source_timeout = (
            receipt_timeout
            if source_timeout_parameter is None
            else float(self.get_parameter(source_timeout_parameter).value)
        )
        return json.dumps(
            {
                "now_sec": now,
                # Keep the legacy field for existing evidence readers.
                "timeout_sec": receipt_timeout,
                "receipt_timeout_sec": receipt_timeout,
                "source_timeout_sec": source_timeout,
                "receipt_sec": tracker.receipt_sec,
                "source_stamp_sec": tracker.source_stamp_sec,
                "receipt_age_sec": now - tracker.receipt_sec,
                "source_age_sec": now - tracker.source_stamp_sec,
                "sequence": tracker.sequence,
                "rejected_future": tracker.rejected_future,
                "last_future_offset_sec": tracker.last_future_offset_sec,
                "max_future_offset_sec": tracker.max_future_offset_sec,
                "rejected_out_of_order": tracker.rejected_out_of_order,
                "last_timer_gap_wall_sec": self._last_timer_gap_wall_sec,
                "max_timer_gap_wall_sec": self._max_timer_gap_wall_sec,
                "last_timer_duration_wall_sec": self._last_timer_duration_wall_sec,
                "max_timer_duration_wall_sec": self._max_timer_duration_wall_sec,
                "timer_overrun_count": self._timer_overrun_count,
            },
            sort_keys=True,
        )

    def _fault(self, reason: str) -> None:
        if self._phase == RuntimePhase.FAULT:
            return
        stale_inputs = {
            "LOCALIZATION_STALE": (
                self._odometry,
                "localization_timeout_sec",
                None,
            ),
            "NDT_POSE_STALE": (
                self._ndt_pose,
                "ndt_pose_timeout_sec",
                "ndt_pose_source_timeout_sec",
            ),
            "TRAJECTORY_STALE": (self._trajectory, "trajectory_timeout_sec", None),
            "RAW_CONTROL_STALE": (self._raw_control, "control_timeout_sec", None),
            "FINAL_CONTROL_STALE": (self._final_control, "control_timeout_sec", None),
            "VELOCITY_STALE": (
                self._velocity,
                "vehicle_status_timeout_sec",
                None,
            ),
            "STEERING_STALE": (
                self._steering,
                "vehicle_status_timeout_sec",
                None,
            ),
            "GEAR_STALE": (self._gear, "vehicle_status_timeout_sec", None),
            "CONTROL_MODE_STALE": (
                self._control_mode,
                "vehicle_status_timeout_sec",
                None,
            ),
        }
        if reason in stale_inputs:
            tracker, receipt_parameter, source_parameter = stale_inputs[reason]
            self.get_logger().error(
                f"race runtime stale-input witness: reason={reason} "
                f"{self._stale_input_witness(tracker, receipt_parameter, source_parameter)}"
            )
        self.get_logger().error(f"race runtime fault latched: {reason}")
        self._transition(RuntimePhase.FAULT, reason)

    def _speed_mps(self) -> float:
        if self._velocity.message is None:
            return 0.0
        return float(self._velocity.message.longitudinal_velocity)

    def _base_failure(self) -> str | None:
        if self._localization_state.message is None or (
            self._localization_state.message.state
            != LocalizationInitializationState.INITIALIZED
        ):
            return "LOCALIZATION_NOT_INITIALIZED"
        if not self._fresh(self._odometry, "localization_timeout_sec"):
            return "LOCALIZATION_STALE"
        if bool(self.get_parameter("require_ndt_pose_health").value) and not self._ndt_fresh():
            return "NDT_POSE_STALE"
        if not self._fresh(self._trajectory, "trajectory_timeout_sec"):
            return "TRAJECTORY_STALE"
        if self._trajectory.message is None or len(self._trajectory.message.points) < 2:
            return "TRAJECTORY_INVALID"
        for tracker, reason in (
            (self._velocity, "VELOCITY_STALE"),
            (self._steering, "STEERING_STALE"),
            (self._gear, "GEAR_STALE"),
            (self._control_mode, "CONTROL_MODE_STALE"),
        ):
            if not self._fresh(tracker, "vehicle_status_timeout_sec"):
                return reason
        return None

    def _active_failure(self) -> str | None:
        failure = self._base_failure()
        if failure is not None:
            return failure
        if not self._fresh(self._raw_control, "control_timeout_sec"):
            return "RAW_CONTROL_STALE"
        if not self._fresh(self._final_control, "control_timeout_sec"):
            return "FINAL_CONTROL_STALE"
        if (
            self._control_mode.message.mode != ControlModeReport.AUTONOMOUS
            and not self._expected_control_handover()
        ):
            return "AUTONOMOUS_CONTROL_LOST"
        return None

    def _expected_control_handover(self) -> bool:
        return self._phase in (RuntimePhase.STOPPING, RuntimePhase.FAULT) and abs(
            self._speed_mps()
        ) < float(self.get_parameter("stop_speed_mps").value)

    def _final_is_stop(self) -> bool:
        if not self._fresh(self._final_control, "control_timeout_sec"):
            return False
        command = self._final_control.message
        return (
            command.longitudinal.velocity <= 0.05
            and command.longitudinal.acceleration <= 0.1
        )

    def _call_control_mode(self, mode: int) -> None:
        self._call_service(
            "control_mode",
            self._control_mode_client,
            ControlModeCommand.Request(mode=mode),
        )

    def _call_engage(self, engage: bool) -> None:
        request = EngageService.Request()
        request.engage = engage
        self._call_service("engage", self._engage_client, request)

    def _call_service(self, key: str, client, request) -> None:
        now = self._now()
        pending = self._pending[key]
        if pending is not None and not pending.done():
            return
        retry = float(self.get_parameter("service_retry_sec").value)
        if now - self._last_service_call[key] < retry or not client.service_is_ready():
            return
        self._pending[key] = client.call_async(request)
        self._last_service_call[key] = now

    def _on_timer(self) -> None:
        timer_start_wall = time.monotonic()
        update_period = 1.0 / float(self.get_parameter("update_rate_hz").value)
        if self._last_timer_wall_sec >= 0.0:
            self._last_timer_gap_wall_sec = (
                timer_start_wall - self._last_timer_wall_sec
            )
            self._max_timer_gap_wall_sec = max(
                self._max_timer_gap_wall_sec, self._last_timer_gap_wall_sec
            )
            if self._last_timer_gap_wall_sec > 2.0 * update_period:
                self._timer_overrun_count += 1
                self.get_logger().warning(
                    "race runtime timer scheduling overrun: "
                    f"gap_wall_sec={self._last_timer_gap_wall_sec:.6f} "
                    f"period_sec={update_period:.6f} "
                    f"count={self._timer_overrun_count}"
                )
        self._last_timer_wall_sec = timer_start_wall
        now = self._now()
        if self._last_now >= 0.0 and now + 0.05 < self._last_now:
            self._reset_state(clear_inputs=True)
            self._reason = "TIME_ROLLBACK_RESET"
        self._last_now = now
        base_failure = self._base_failure()

        if self._phase == RuntimePhase.IDLE:
            if not self._start_requested:
                self._reason = "WAITING_FOR_START"
            elif base_failure is None:
                self._transition(RuntimePhase.READY, "READINESS_COMPLETE")
            elif now - self._start_sec > float(
                self.get_parameter("startup_timeout_sec").value
            ):
                self._fault(base_failure)
            else:
                self._reason = base_failure
        elif self._phase == RuntimePhase.READY:
            if base_failure is not None:
                self._fault(base_failure)
            else:
                self._call_control_mode(ControlModeCommand.Request.AUTONOMOUS)
                pending = self._pending["control_mode"]
                if (
                    pending is not None
                    and pending.done()
                    and pending.result() is not None
                    and pending.result().success
                ):
                    self._transition(RuntimePhase.ARMING, "AUTO_REQUEST_ACCEPTED")
        elif self._phase == RuntimePhase.ARMING:
            if base_failure is not None:
                self._fault(base_failure)
            elif not self._fresh(self._raw_control, "control_timeout_sec"):
                self._reason = "WAITING_RAW_CONTROL"
            elif (
                self._activation_armed
                and self._engage.message is not None
                and bool(self._engage.message.engage)
            ):
                self._transition(RuntimePhase.ACTIVE, "ENGAGE_CONFIRMED")
            elif (
                self._control_mode.message.mode == ControlModeReport.AUTONOMOUS
                and self._gear.message.report == GearReport.DRIVE
                and self._final_is_stop()
            ):
                if not self._activation_armed:
                    self._activation_armed = True
                    self._activation_stamp = now
                    self._reason = "WAITING_POST_ENABLE_CONTROL"
                elif self._raw_control.receipt_sec > self._activation_stamp:
                    self._call_engage(True)
            else:
                self._reason = "WAITING_AUTO_GEAR_STOP"
        elif self._phase == RuntimePhase.ACTIVE:
            failure = self._active_failure()
            if failure is not None:
                self._fault(failure)
            elif self._stop_requested or (
                self._route_state.message is not None
                and self._route_state.message.state == RouteState.ARRIVED
            ):
                self._transition(RuntimePhase.STOPPING, "STOP_REQUESTED_OR_ARRIVED")
        elif self._phase == RuntimePhase.STOPPING:
            if abs(self._speed_mps()) >= float(
                self.get_parameter("stop_speed_mps").value
            ):
                failure = self._active_failure()
                if failure is not None:
                    self._fault(failure)
            else:
                self._call_engage(False)
                self._call_control_mode(ControlModeCommand.Request.MANUAL)
                if (
                    self._control_mode.message is not None
                    and self._control_mode.message.mode
                    in (ControlModeReport.MANUAL, ControlModeReport.NO_COMMAND)
                    and self._gear.message is not None
                    and self._gear.message.report in (GearReport.PARK, GearReport.NEUTRAL)
                    and self._engage.message is not None
                    and not bool(self._engage.message.engage)
                ):
                    self._transition(RuntimePhase.FINISHED, "STOP_CONFIRMED")
        elif self._phase == RuntimePhase.FAULT:
            if abs(self._speed_mps()) < float(
                self.get_parameter("stop_speed_mps").value
            ):
                self._call_engage(False)
                self._call_control_mode(ControlModeCommand.Request.MANUAL)

        self._publish(base_failure is None)
        self._last_timer_duration_wall_sec = time.monotonic() - timer_start_wall
        self._max_timer_duration_wall_sec = max(
            self._max_timer_duration_wall_sec,
            self._last_timer_duration_wall_sec,
        )
        if self._last_timer_duration_wall_sec > update_period:
            self.get_logger().warning(
                "race runtime timer execution overrun: "
                f"duration_wall_sec={self._last_timer_duration_wall_sec:.6f} "
                f"period_sec={update_period:.6f}"
            )

    def _publish(self, ready: bool) -> None:
        stamp = self.get_clock().now().to_msg()
        speed = self._speed_mps()
        fault = self._phase == RuntimePhase.FAULT

        gate_mode = GateMode()
        gate_mode.data = GateMode.AUTO
        self._gate_mode_pub.publish(gate_mode)

        gear = GearCommand()
        gear.stamp = stamp
        gear.command = desired_gear_command(
            self._phase,
            speed,
            float(self.get_parameter("stop_speed_mps").value),
        )
        self._gear_pub.publish(gear)

        turn = TurnIndicatorsCommand()
        turn.stamp = stamp
        turn.command = TurnIndicatorsCommand.DISABLE
        self._turn_pub.publish(turn)
        hazard = HazardLightsCommand()
        hazard.stamp = stamp
        hazard.command = (
            HazardLightsCommand.ENABLE if fault else HazardLightsCommand.DISABLE
        )
        self._hazard_pub.publish(hazard)

        operation = OperationModeState()
        operation.stamp = stamp
        autonomous_intent = self._phase in (
            RuntimePhase.READY,
            RuntimePhase.ARMING,
            RuntimePhase.ACTIVE,
            RuntimePhase.STOPPING,
        )
        operation.mode = (
            OperationModeState.AUTONOMOUS if autonomous_intent else OperationModeState.STOP
        )
        operation.is_autoware_control_enabled = self._phase in (
            RuntimePhase.ACTIVE,
            RuntimePhase.STOPPING,
        ) or (self._phase == RuntimePhase.ARMING and self._activation_armed)
        operation.is_in_transition = self._phase in (
            RuntimePhase.READY,
            RuntimePhase.ARMING,
            RuntimePhase.STOPPING,
        )
        operation.is_stop_mode_available = True
        operation.is_autonomous_mode_available = ready and not fault
        self._operation_mode_pub.publish(operation)

        mrm = MrmState()
        mrm.stamp = stamp
        mrm.state = MrmState.MRM_OPERATING if fault else MrmState.NORMAL
        mrm.behavior = MrmState.EMERGENCY_STOP if fault else MrmState.NONE
        self._mrm_pub.publish(mrm)

        emergency_control = Control()
        emergency_control.stamp = stamp
        emergency_control.lateral.stamp = stamp
        emergency_control.longitudinal.stamp = stamp
        emergency_control.lateral.steering_tire_angle = (
            float(self._steering.message.steering_tire_angle)
            if self._steering.message is not None
            else 0.0
        )
        emergency_control.lateral.steering_tire_rotation_rate = 0.0
        emergency_control.lateral.is_defined_steering_tire_rotation_rate = True
        emergency_control.longitudinal.velocity = 0.0
        emergency_control.longitudinal.acceleration = float(
            self.get_parameter("emergency_acceleration_mps2").value
        )
        emergency_control.longitudinal.jerk = -4.0
        emergency_control.longitudinal.is_defined_acceleration = True
        emergency_control.longitudinal.is_defined_jerk = True
        self._emergency_control_pub.publish(emergency_control)

        emergency_gear = GearCommand()
        emergency_gear.stamp = stamp
        emergency_gear.command = (
            GearCommand.DRIVE
            if abs(speed) >= float(self.get_parameter("stop_speed_mps").value)
            else GearCommand.PARK
        )
        self._emergency_gear_pub.publish(emergency_gear)
        emergency_turn = TurnIndicatorsCommand()
        emergency_turn.stamp = stamp
        emergency_turn.command = TurnIndicatorsCommand.NO_COMMAND
        self._emergency_turn_pub.publish(emergency_turn)
        emergency_hazard = HazardLightsCommand()
        emergency_hazard.stamp = stamp
        emergency_hazard.command = (
            HazardLightsCommand.ENABLE if fault else HazardLightsCommand.DISABLE
        )
        self._emergency_hazard_pub.publish(emergency_hazard)

        state = String()
        state.data = json.dumps(
            {
                "state": self._phase.name,
                "state_id": int(self._phase),
                "ready": ready and not fault,
                "reason": self._reason,
                "control_enabled": operation.is_autoware_control_enabled,
                "control_mode": None
                if self._control_mode.message is None
                else int(self._control_mode.message.mode),
                "gear": None
                if self._gear.message is None
                else int(self._gear.message.report),
                "engaged": bool(self._engage.message.engage)
                if self._engage.message is not None
                else False,
                "raw_control_fresh": self._fresh(
                    self._raw_control, "control_timeout_sec"
                ),
                "final_control_fresh": self._fresh(
                    self._final_control, "control_timeout_sec"
                ),
                "ndt_pose_health_required": bool(
                    self.get_parameter("require_ndt_pose_health").value
                ),
                "ndt_pose_fresh": self._ndt_fresh(),
                "stamp": self._now(),
            },
            sort_keys=True,
        )
        self._state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceRuntimeManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
