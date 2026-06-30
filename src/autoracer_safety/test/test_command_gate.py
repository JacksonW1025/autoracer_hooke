from autoware_vehicle_msgs.msg import GearCommand

from autoracer_safety.command_gate import (
    gear_command_for_velocity,
    limit_signed_velocity,
)


def test_limit_signed_velocity_allows_reverse_with_symmetric_cap():
    assert limit_signed_velocity(4.0, 3.0) == 3.0
    assert limit_signed_velocity(-4.0, 3.0) == -3.0
    assert limit_signed_velocity(-1.25, 3.0) == -1.25


def test_gear_command_follows_safe_signed_velocity():
    assert gear_command_for_velocity(safe=False, velocity=1.0) == GearCommand.NEUTRAL
    assert gear_command_for_velocity(safe=True, velocity=0.0) == GearCommand.DRIVE
    assert gear_command_for_velocity(safe=True, velocity=0.2) == GearCommand.DRIVE
    assert gear_command_for_velocity(safe=True, velocity=-0.2) == GearCommand.REVERSE
