# scripts/hooke

Hooke profile is disabled.

The Hooke operator surface exists only as a handoff boundary. It is not runtime
ready because the real Hooke vehicle description, sensor-kit description,
vehicle launch, and sensing launch packages are still disabled placeholders
under `src/autoracer_hooke_*`.

Do not point this script layer at the RC profile, RC C32 LiDAR launch, or RC UART
adapter. The Hooke path becomes runnable only after the real
`autoracer_hooke` and `autoracer_hooke_sensor_kit` profiles are complete and
their `COLCON_IGNORE` guards are removed.
