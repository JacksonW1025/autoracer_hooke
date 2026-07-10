# Calibration Checklist

Run these checks before setting `ENABLE_DRIVE_COMMANDS=true`.

## Frames

- `base_link -> lidar_top` measured on the real chassis.
- `base_link -> gnss_base_link` measured to the Fixposition antenna reference.
- `base_link -> imu_link` or Fixposition IMU frame measured and yaw-aligned.
- `map -> base_link` moves smoothly while driving slowly on the mapped track.

## Vehicle Parameters

- Wheel base.
- Wheel radius.
- Steering tire angle sign.
- Steering ratio or VGR parameters inside `hooke2_interface`.
- Maximum steering angle.
- Longitudinal velocity sign and scale.
- Brake/throttle command sign and scale.

## Low-Speed Test

1. Keep wheels lifted or vehicle secured.
2. Run with `ENABLE_DRIVE_COMMANDS=false`.
3. Confirm route, trajectory, and raw control direction in RViz.
4. Enable drive commands at `MAX_SPEED_MPS=0.5`.
5. Verify stop on localization loss, raw command timeout, and route completion.

