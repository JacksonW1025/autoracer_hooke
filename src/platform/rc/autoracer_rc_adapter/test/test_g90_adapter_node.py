import math

from diagnostic_msgs.msg import DiagnosticStatus
from nmea_msgs.msg import Sentence
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from autoracer_rc_adapter.g90_adapter_node import G90NmeaAdapter


def sentence(payload: str, stamp: float) -> Sentence:
    checksum = 0
    for byte in payload.encode("ascii"):
        checksum ^= byte
    message = Sentence()
    nanoseconds = round(stamp * 1e9)
    message.header.stamp.sec = nanoseconds // 1_000_000_000
    message.header.stamp.nanosec = nanoseconds % 1_000_000_000
    message.sentence = f"${payload}*{checksum:02X}"
    return message


class Capture:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, message):
        self.events.append((self.name, message))


@pytest.fixture
def node():
    rclpy.init()
    adapter = G90NmeaAdapter()
    try:
        yield adapter
    finally:
        adapter.destroy_node()
        rclpy.shutdown()


def capture_publishers(node):
    events = []
    node._orientation_publisher = Capture("orientation", events)
    node._fix_publisher = Capture("fix", events)
    node._diagnostic_publisher = Capture("diagnostics", events)
    return events


def send_complete_fixed_epoch(node):
    node._on_sentence(sentence("GNTHS,90.0,A", 10.0))
    node._on_sentence(
        sentence("GPGST,092750.00,0.4,0.3,0.2,45.0,0.10,0.20,0.30", 10.05)
    )
    node._on_sentence(
        sentence(
            "GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
            "12.3,M,-2.4,M,0.5,0001",
            10.1,
        )
    )


def test_stage4_default_never_exposes_unapproved_fixed_to_core(node):
    events = capture_publishers(node)
    send_complete_fixed_epoch(node)

    assert [name for name, _ in events] == ["fix", "diagnostics"]
    fix = events[0][1]
    assert fix.status.status == NavSatStatus.STATUS_NO_FIX
    values = {item.key: item.value for item in events[1][1].status[0].values}
    assert values["receiver_rtk_fixed"] == "true"
    assert values["core_boundary_fixed"] == "false"
    assert values["reason"] == "localization_output_disabled"


def test_calibrated_georeferenced_epoch_matches_standard_gnss_contract(node):
    events = capture_publishers(node)
    node._enable_localization_output = True
    node._yaw_stddev_rad = math.radians(1.5)
    node._map_projector_type = "LocalCartesianUTM"
    node._tf_ready = lambda: True

    send_complete_fixed_epoch(node)

    assert [name for name, _ in events] == ["orientation", "fix", "diagnostics"]
    orientation = events[0][1]
    fix = events[1][1]
    assert orientation.header.frame_id == "gnss_link"
    assert orientation.orientation.orientation.z == pytest.approx(0.0)
    assert orientation.orientation.orientation.w == pytest.approx(1.0)
    assert orientation.orientation.rmse_rotation_z == pytest.approx(
        math.radians(1.5)
    )
    assert isinstance(fix, NavSatFix)
    assert fix.status.status == NavSatStatus.STATUS_GBAS_FIX
    assert fix.altitude == pytest.approx(9.9)
    assert list(fix.position_covariance)[0:9:4] == pytest.approx(
        [0.04, 0.01, 0.09]
    )
    assert fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN


def test_local_map_projector_blocks_even_receiver_rtk_fixed(node):
    events = capture_publishers(node)
    node._enable_localization_output = True
    node._yaw_stddev_rad = math.radians(1.5)
    node._map_projector_type = "Local"
    node._tf_ready = lambda: True

    send_complete_fixed_epoch(node)

    assert [name for name, _ in events] == ["fix", "diagnostics"]
    assert events[0][1].status.status == NavSatStatus.STATUS_NO_FIX
    values = {item.key: item.value for item in events[1][1].status[0].values}
    assert values["reason"] == "map_projector_local"


def test_missing_tf_or_yaw_covariance_blocks_output(node):
    node._enable_localization_output = True
    node._map_projector_type = "MGRS"
    assert node._runtime_prerequisite_reason() == "yaw_covariance_unconfigured"

    node._yaw_stddev_rad = math.radians(1.0)
    node._tf_ready = lambda: False
    assert node._runtime_prerequisite_reason() == "base_transform_missing"


def test_missing_gst_never_falls_back_to_invented_covariance(node):
    events = capture_publishers(node)
    node._on_sentence(sentence("GNTHS,90.0,A", 10.0))
    node._on_sentence(
        sentence(
            "GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
            "12.3,M,-2.4,M,0.5,0001",
            10.1,
        )
    )
    assert events[0][1].status.status == NavSatStatus.STATUS_NO_FIX
    assert events[0][1].position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
    values = {item.key: item.value for item in events[1][1].status[0].values}
    assert values["reason"] == "covariance_missing"


def test_statusless_hdt_is_diagnostic_only_by_default(node):
    events = capture_publishers(node)
    node._on_sentence(sentence("GPHDT,90.0,T", 10.0))
    node._on_sentence(
        sentence("GPGST,092750.00,0.4,0.3,0.2,45.0,0.10,0.20,0.30", 10.05)
    )
    node._on_sentence(
        sentence(
            "GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
            "12.3,M,-2.4,M,0.5,0001",
            10.1,
        )
    )

    assert events[0][1].status.status == NavSatStatus.STATUS_NO_FIX
    values = {item.key: item.value for item in events[1][1].status[0].values}
    assert values["reason"] == "heading_status_unavailable"
    assert values["heading_source"] == "HDT"


def test_unavailable_ths_revokes_previous_heading(node):
    events = capture_publishers(node)
    node._on_sentence(sentence("GNTHS,90.0,A", 10.0))
    node._on_sentence(sentence("GNTHS,,V", 10.01))
    node._on_sentence(
        sentence("GPGST,092750.00,0.4,0.3,0.2,45.0,0.10,0.20,0.30", 10.05)
    )
    node._on_sentence(
        sentence(
            "GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
            "12.3,M,-2.4,M,0.5,0001",
            10.1,
        )
    )

    assert events[0][1].status.status == NavSatStatus.STATUS_NO_FIX
    values = {item.key: item.value for item in events[1][1].status[0].values}
    assert values["reason"] == "heading_mode_v"


def test_invalid_gga_immediately_revokes_previous_fixed(node):
    events = capture_publishers(node)
    node._enable_localization_output = True
    node._yaw_stddev_rad = math.radians(1.5)
    node._map_projector_type = "MGRS"
    node._tf_ready = lambda: True
    send_complete_fixed_epoch(node)
    events.clear()

    invalid = sentence(
        "GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
        "12.3,M,-2.4,M,0.5,0001",
        10.2,
    )
    invalid.sentence = f"{invalid.sentence[:-2]}00"
    node._on_sentence(invalid)

    assert [name for name, _ in events] == ["fix", "diagnostics"]
    assert events[0][1].status.status == NavSatStatus.STATUS_NO_FIX
    assert events[1][1].status[0].level == DiagnosticStatus.WARN
    values = {item.key: item.value for item in events[1][1].status[0].values}
    assert values["reason"] == "invalid_gga"
