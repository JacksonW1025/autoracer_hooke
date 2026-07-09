# RC/Hooke 标定与车辆参数参考

用途：集中记录车辆几何、传感器外参、速度/转角尺度和首次低速测试需要复核的事实。非用途：不替代现场运行手册，不记录每次测试日志。

## RC 车辆参数

| 参数 | 值 |
| --- | --- |
| 车长 | `0.83 m` |
| 车宽 | `0.58 m` |
| 车高 | `0.50 m` |
| 轮径 | `0.23 m` |
| 轮半径 / `base_link` 高度 | `0.115 m` |
| 轴距 | `0.6 m` |
| 最大前轮转角 | `0.262 rad` |

这些值需要同步到 vehicle profile、URDF/TF、controller 参数、vehicle adapter 限幅和低速测试记录。参数不能在多个文件里各自漂移。

## Frames

- `base_link -> lidar_top` measured on the active platform.
- Hooke profile: `base_link -> gnss_base_link` measured to the Fixposition antenna reference.
- Hooke profile: `base_link -> imu_link` or Fixposition IMU frame measured and yaw-aligned.
- RC profile: `base_link -> lidar_top` starts from the legacy C32 value but must be rechecked on the car.
- RC profile: `base_link -> imu_link` starts as zero pose for Hipnuc/N300 Pro and must be updated after physical measurement.
- `map -> base_link` moves smoothly while driving slowly on the mapped track.

## Vehicle Parameters

- Wheel base from the active vehicle profile.
- Wheel radius or wheel diameter from the active vehicle profile.
- Steering tire angle sign.
- Steering ratio or VGR parameters inside `hooke2_interface`.
- Maximum steering angle.
- Longitudinal velocity sign and scale.
- Brake/throttle command sign and scale.
- RC profile: UART feedback velocity, steering angle, gear, and control mode match `/vehicle/status/*` semantics.

## Low-Speed Test

Run these checks before setting `ENABLE_DRIVE_COMMANDS=true`.

1. Keep wheels lifted or vehicle secured.
2. Run with `ENABLE_DRIVE_COMMANDS=false`.
3. Confirm route, trajectory, official control, and gated safe control direction in RViz.
4. Enable drive commands at `MAX_SPEED_MPS=0.5` or another explicitly chosen low-speed limit.
5. Verify stop on localization loss, control command timeout, and route completion.
