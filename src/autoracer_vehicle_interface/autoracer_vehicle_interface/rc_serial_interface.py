import math
import os
import select
import termios

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    SteeringReport,
    VelocityReport,
)
from autoware_vehicle_msgs.srv import ControlModeCommand
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .rc_serial_protocol import encode_control_frame, pop_telemetry_frame


_BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes", "on")


class PosixSerial:
    def __init__(self, device, baudrate):
        if baudrate not in _BAUD_RATES:
            raise ValueError(f"unsupported baudrate: {baudrate}")
        self.device = device
        self.baudrate = baudrate
        self.fd = None

    @property
    def is_open(self):
        return self.fd is not None

    def open(self):
        fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        baud = _BAUD_RATES[self.baudrate]
        flow_control = getattr(termios, "CRTSCTS", 0)

        attrs[0] = termios.IGNPAR
        attrs[1] = 0
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | flow_control)
        attrs[2] |= termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud
        attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self.fd = fd

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read(self, size=256):
        if self.fd is None:
            return b""
        readable, _, _ = select.select([self.fd], [], [], 0.0)
        if not readable:
            return b""
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def write(self, data):
        if self.fd is None:
            raise OSError("serial port is not open")
        os.write(self.fd, data)


class RcSerialInterface(Node):
    def __init__(self):
        super().__init__("rc_serial_interface")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("wheel_base_m", 0.54)
        self.declare_parameter("max_speed_mps", 1.5)
        self.declare_parameter("max_steer_rad", 0.393)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("command_rate_hz", 30.0)
        self.declare_parameter("feedback_rate_hz", 50.0)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("start_in_autonomous", True)
        self.declare_parameter("control_topic", "/control/command/control_cmd")
        self.declare_parameter("gear_command_topic", "/control/command/gear_cmd")
        self.declare_parameter("state_topic", "/autoracer/vehicle_interface/state")

        self._port = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._frame_id = str(self.get_parameter("base_frame_id").value)
        self._wheel_base = float(self.get_parameter("wheel_base_m").value)
        self._max_speed = float(self.get_parameter("max_speed_mps").value)
        self._max_steer = float(self.get_parameter("max_steer_rad").value)
        self._command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self._reconnect_interval = float(self.get_parameter("reconnect_interval_sec").value)
        self._autonomous_requested = _as_bool(self.get_parameter("start_in_autonomous").value)

        self._serial = PosixSerial(self._port, self._baudrate)
        self._rx_buffer = bytearray()
        self._last_command = None
        self._last_command_time = None
        self._last_connect_attempt = None
        self._last_telemetry_time = None
        self._last_steer_cmd = 0.0
        self._last_gear = GearReport.DRIVE
        self._last_state = None

        self.create_subscription(
            Control,
            self.get_parameter("control_topic").value,
            self._on_control_command,
            10,
        )
        self.create_subscription(
            GearCommand,
            self.get_parameter("gear_command_topic").value,
            self._on_gear_command,
            10,
        )

        self._velocity_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10
        )
        self._steering_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10
        )
        self._gear_pub = self.create_publisher(GearReport, "/vehicle/status/gear_status", 10)
        self._control_mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 10
        )
        self._state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
        self.create_service(ControlModeCommand, "/control/control_mode_request", self._on_control_mode)

        self.create_timer(
            1.0 / float(self.get_parameter("command_rate_hz").value),
            self._on_command_timer,
        )
        self.create_timer(
            1.0 / float(self.get_parameter("feedback_rate_hz").value),
            self._on_feedback_timer,
        )

    def destroy_node(self):
        self._serial.close()
        super().destroy_node()

    def _now(self):
        return self.get_clock().now()

    def _age_sec(self, stamp):
        if stamp is None:
            return None
        return (self._now() - stamp).nanoseconds * 1e-9

    def _try_connect(self):
        if self._serial.is_open:
            return True
        if self._last_connect_attempt is not None:
            if self._age_sec(self._last_connect_attempt) < self._reconnect_interval:
                return False
        self._last_connect_attempt = self._now()
        try:
            self._serial.open()
        except OSError as exc:
            self._publish_state(f"serial_disconnected:{exc}")
            return False
        self.get_logger().info(f"opened RC serial port {self._port} at {self._baudrate}")
        self._publish_state("serial_connected")
        return True

    def _drop_connection(self, reason):
        self.get_logger().warn(f"closing RC serial port: {reason}")
        self._serial.close()
        self._publish_state(f"serial_error:{reason}")

    def _publish_state(self, state):
        if state == self._last_state:
            return
        self._last_state = state
        self._state_pub.publish(String(data=state))

    def _on_control_mode(self, request, response):
        if request.mode in (
            ControlModeCommand.Request.AUTONOMOUS,
            ControlModeCommand.Request.AUTONOMOUS_STEER_ONLY,
            ControlModeCommand.Request.AUTONOMOUS_VELOCITY_ONLY,
        ):
            self._autonomous_requested = True
            response.success = True
        elif request.mode in (
            ControlModeCommand.Request.MANUAL,
            ControlModeCommand.Request.NO_COMMAND,
        ):
            self._autonomous_requested = False
            response.success = True
        else:
            response.success = False
        return response

    def _on_control_command(self, msg):
        self._last_command = msg
        self._last_command_time = self._now()
        self._last_steer_cmd = _clamp(
            float(msg.lateral.steering_tire_angle), -self._max_steer, self._max_steer
        )

    def _on_gear_command(self, msg):
        if msg.command == GearCommand.REVERSE:
            self._last_gear = GearReport.REVERSE
        elif msg.command == GearCommand.NEUTRAL:
            self._last_gear = GearReport.NEUTRAL
        else:
            self._last_gear = GearReport.DRIVE

    def _control_to_motion(self):
        if self._last_command is None:
            return 0.0, 0.0, 0.0, True
        age = self._age_sec(self._last_command_time)
        if age is None or age > self._command_timeout:
            return 0.0, 0.0, 0.0, True

        velocity = _clamp(
            float(self._last_command.longitudinal.velocity), -self._max_speed, self._max_speed
        )
        steer = _clamp(
            float(self._last_command.lateral.steering_tire_angle),
            -self._max_steer,
            self._max_steer,
        )
        wz = velocity * math.tan(steer) / max(self._wheel_base, 1e-3)
        return velocity, 0.0, wz, abs(velocity) < 1e-4 and abs(wz) < 1e-4

    def _on_command_timer(self):
        self._publish_control_mode()
        if not self._try_connect():
            return

        vx, vy, wz, stop = self._control_to_motion()
        try:
            self._serial.write(encode_control_frame(vx, vy, wz, stop=stop))
        except OSError as exc:
            self._drop_connection(str(exc))

    def _on_feedback_timer(self):
        if not self._try_connect():
            return
        try:
            data = self._serial.read(512)
        except OSError as exc:
            self._drop_connection(str(exc))
            return
        if data:
            self._rx_buffer.extend(data)

        while True:
            try:
                telemetry = pop_telemetry_frame(self._rx_buffer)
            except ValueError as exc:
                self.get_logger().warn(f"dropped invalid telemetry frame: {exc}")
                continue
            if telemetry is None:
                break
            self._last_telemetry_time = self._now()
            self._publish_telemetry(telemetry)

    def _publish_telemetry(self, telemetry):
        now = self._now().to_msg()

        velocity = VelocityReport()
        velocity.header.stamp = now
        velocity.header.frame_id = self._frame_id
        velocity.longitudinal_velocity = float(telemetry.vx_mps)
        velocity.lateral_velocity = float(telemetry.vy_mps)
        velocity.heading_rate = float(telemetry.wz_rad_s)
        self._velocity_pub.publish(velocity)

        steering = SteeringReport()
        steering.stamp = now
        if abs(telemetry.vx_mps) > 0.05:
            steering.steering_tire_angle = _clamp(
                math.atan(telemetry.wz_rad_s * self._wheel_base / telemetry.vx_mps),
                -self._max_steer,
                self._max_steer,
            )
        else:
            steering.steering_tire_angle = self._last_steer_cmd
        self._steering_pub.publish(steering)

        gear = GearReport()
        gear.stamp = now
        if telemetry.vx_mps < -0.05:
            gear.report = GearReport.REVERSE
        elif self._last_gear == GearReport.NEUTRAL:
            gear.report = GearReport.NEUTRAL
        else:
            gear.report = GearReport.DRIVE
        self._gear_pub.publish(gear)

    def _publish_control_mode(self):
        msg = ControlModeReport()
        msg.stamp = self._now().to_msg()
        if not self._serial.is_open:
            msg.mode = ControlModeReport.NOT_READY
        elif self._autonomous_requested:
            msg.mode = ControlModeReport.AUTONOMOUS
        else:
            msg.mode = ControlModeReport.MANUAL
        self._control_mode_pub.publish(msg)


def main():
    rclpy.init()
    node = RcSerialInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
