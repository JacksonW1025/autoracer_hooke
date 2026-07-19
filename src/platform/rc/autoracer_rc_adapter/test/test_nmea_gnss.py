import math

import pytest

from autoracer_rc_adapter.nmea_gnss import (
    Gga,
    Gst,
    Hdt,
    NmeaGnssGate,
    NmeaParseError,
    Ths,
    heading_to_enu_yaw,
    parse_nmea_sentence,
)


def sentence(payload: str) -> str:
    checksum = 0
    for byte in payload.encode("ascii"):
        checksum ^= byte
    return f"${payload}*{checksum:02X}"


def gga(quality=4):
    return Gga(
        quality=quality,
        satellites=18,
        hdop=0.7,
        latitude_deg=22.53539,
        longitude_deg=114.02613,
        altitude_ellipsoid_m=9.9,
    )


@pytest.mark.parametrize("talker", ("GP", "GN", "GB", "BD", "GL", "GA"))
def test_documented_and_standard_gnss_talkers_are_accepted(talker):
    parsed = parse_nmea_sentence(
        sentence(
            f"{talker}GGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
            "12.3,M,-2.4,M,0.5,0001"
        )
    )
    assert isinstance(parsed, Gga)
    assert parsed.quality == 4


def test_gga_preserves_quality_coordinates_and_ellipsoid_height():
    parsed = parse_nmea_sentence(
        sentence(
            "GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
            "12.3,M,-2.4,M,0.5,0001"
        )
    )
    assert parsed.latitude_deg == pytest.approx(22.53539)
    assert parsed.longitude_deg == pytest.approx(114.02613)
    assert parsed.altitude_ellipsoid_m == pytest.approx(9.9)
    assert parsed.satellites == 18


def test_no_fix_gga_allows_empty_position():
    parsed = parse_nmea_sentence(
        sentence("GNGGA,092750.00,,,,,0,00,99.9,,,,,,")
    )
    assert parsed == Gga(0, 0, 99.9, None, None, None)


@pytest.mark.parametrize(
    "payload",
    (
        "GNGGA,092750.00,2260.0,N,11401.0,E,4,18,0.7,12.3,M,-2.4,M,,",
        "GNGGA,092750.00,nan,N,11401.0,E,4,18,0.7,12.3,M,-2.4,M,,",
        "GNGGA,092750.00,2232.0,N,11401.0,E,4,18,0.7,12.3,,,-2.4,,",
    ),
)
def test_invalid_fixed_position_is_rejected(payload):
    with pytest.raises(NmeaParseError):
        parse_nmea_sentence(sentence(payload))


def test_checksum_is_mandatory_and_verified():
    valid = sentence(
        "GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,"
        "12.3,M,-2.4,M,,"
    )
    with pytest.raises(NmeaParseError, match="checksum"):
        parse_nmea_sentence(valid[:-2] + "00")


def test_gst_is_interpreted_as_standard_deviation():
    parsed = parse_nmea_sentence(
        sentence("GPGST,092750.00,0.4,0.3,0.2,45.0,0.10,0.20,0.30")
    )
    assert parsed == Gst(0.10, 0.20, 0.30)


@pytest.mark.parametrize("invalid", ("0", "-0.1", "nan", "inf", ""))
def test_gst_requires_positive_finite_uncertainty(invalid):
    with pytest.raises(NmeaParseError):
        parse_nmea_sentence(
            sentence(f"GPGST,092750.00,0.4,0.3,0.2,45.0,0.10,{invalid},0.30")
        )


@pytest.mark.parametrize(
    ("heading", "expected"),
    ((0.0, math.pi / 2.0), (90.0, 0.0), (180.0, -math.pi / 2.0)),
)
def test_true_north_heading_converts_to_ros_enu(heading, expected):
    assert heading_to_enu_yaw(heading, 0.0) == pytest.approx(expected)


def test_heading_mount_offset_has_ros_positive_sign():
    assert heading_to_enu_yaw(90.0, 90.0) == pytest.approx(math.pi / 2.0)


def test_hdt_requires_true_north_reference():
    assert parse_nmea_sentence(sentence("GPHDT,360.0,T")) == Hdt(0.0)
    with pytest.raises(NmeaParseError):
        parse_nmea_sentence(sentence("GPHDT,90.0,M"))


@pytest.mark.parametrize("talker", ("GP", "GN", "GB", "BD", "GL", "GA"))
def test_ths_preserves_heading_validity_mode(talker):
    assert parse_nmea_sentence(sentence(f"{talker}THS,90.0,A")) == Ths(90.0, "A")


def test_ths_unavailable_mode_can_omit_heading():
    assert parse_nmea_sentence(sentence("GNTHS,,V")) == Ths(None, "V")
    with pytest.raises(NmeaParseError):
        parse_nmea_sentence(sentence("GNTHS,,A"))


def test_invalid_ths_mode_is_rejected():
    with pytest.raises(NmeaParseError, match="THS mode"):
        parse_nmea_sentence(sentence("GNTHS,90.0,D"))


def test_fixed_requires_fresh_heading_and_fresh_gst():
    gate = NmeaGnssGate(heading_max_age_sec=0.3, gst_max_age_sec=0.5)
    assert gate.accept_gga(gga(), 1.0).reason == "heading_missing"
    gate.accept_heading(90.0, 1.1)
    assert gate.accept_gga(gga(), 1.2).reason == "covariance_missing"
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 1.25)
    decision = gate.accept_gga(gga(), 1.3)
    assert decision.accepted is True
    assert decision.orientation.yaw_enu_rad == pytest.approx(0.0)


def test_explicit_heading_rejection_revokes_previous_heading():
    gate = NmeaGnssGate()
    gate.accept_heading(90.0, 1.0)
    gate.reject_heading("heading_mode_v")
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 1.0)
    decision = gate.accept_gga(gga(), 1.1)
    assert decision.accepted is False
    assert decision.reason == "heading_mode_v"


@pytest.mark.parametrize("quality", (0, 1, 2, 5, 9))
def test_only_gga_quality_four_is_accepted(quality):
    gate = NmeaGnssGate()
    gate.accept_heading(90.0, 1.0)
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 1.0)
    decision = gate.accept_gga(gga(quality), 1.1)
    assert decision.accepted is False
    assert decision.reason == f"quality_{quality}"


def test_gst_covariance_is_mapped_to_enu_without_hdop_scaling():
    gate = NmeaGnssGate()
    gate.accept_heading(90.0, 1.0)
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 1.0)
    decision = gate.accept_gga(gga(), 1.1)
    assert decision.east_variance_m2 == pytest.approx(0.2**2)
    assert decision.north_variance_m2 == pytest.approx(0.1**2)
    assert decision.up_variance_m2 == pytest.approx(0.3**2)


def test_stale_or_future_supporting_samples_fail_closed():
    gate = NmeaGnssGate(heading_max_age_sec=0.3, gst_max_age_sec=0.5)
    gate.accept_heading(90.0, 1.0)
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 1.0)
    assert gate.accept_gga(gga(), 1.6).reason == "heading_stale"

    gate.accept_heading(90.0, 2.0)
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 2.0)
    assert gate.accept_gga(gga(), 1.9).reason == "heading_stale"


def test_fixed_silence_and_clock_reset_each_revoke_once():
    gate = NmeaGnssGate(fix_timeout_sec=0.5)
    gate.accept_heading(90.0, 1.0)
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 1.0)
    assert gate.accept_gga(gga(), 1.1).accepted
    assert gate.expire(1.6) is None
    assert gate.expire(1.600001).reason == "timeout"
    assert gate.expire(2.0) is None

    gate.accept_heading(90.0, 100.0)
    gate.accept_gst(Gst(0.1, 0.2, 0.3), 100.0)
    assert gate.accept_gga(gga(), 100.1).accepted
    assert gate.expire(1.0).reason == "clock_reset"


def test_unneeded_valid_sentence_is_ignored():
    assert parse_nmea_sentence(
        sentence("GNRMC,092750.00,A,2232.1,N,11401.5,E,0.0,0.0,190726,,,A")
    ) is None
