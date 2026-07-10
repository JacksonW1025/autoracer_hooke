from autoracer_control.pure_pursuit_controller import (
    clamp_target_speed,
    steering_for_local_target,
    target_is_in_motion_direction,
)


def test_clamp_target_speed_preserves_signed_trajectory_velocity():
    assert clamp_target_speed(4.2, 3.0) == 3.0
    assert clamp_target_speed(-4.2, 3.0) == -3.0
    assert clamp_target_speed(-1.0, 3.0) == -1.0


def test_target_direction_uses_velocity_sign():
    assert target_is_in_motion_direction(local_x=1.0, target_speed=1.0)
    assert not target_is_in_motion_direction(local_x=-1.0, target_speed=1.0)
    assert target_is_in_motion_direction(local_x=-1.0, target_speed=-1.0)
    assert not target_is_in_motion_direction(local_x=1.0, target_speed=-1.0)


def test_reverse_steering_uses_reverse_kinematic_sign():
    forward = steering_for_local_target(
        local_x=2.0,
        local_y=0.5,
        target_speed=1.0,
        wheel_base_m=0.6,
        max_steer_rad=0.262,
    )
    reverse = steering_for_local_target(
        local_x=-2.0,
        local_y=0.5,
        target_speed=-1.0,
        wheel_base_m=0.6,
        max_steer_rad=0.262,
    )

    assert forward > 0.0
    assert reverse < 0.0
