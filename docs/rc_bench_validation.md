# RC Bench and Low-Speed Validation

本文档是硬件证据记录模板。没有安全操作员、可用 E-stop 或固定/架空车辆时，停止运动
测试。单元测试和无硬件 launch 不能填写为实车通过。

## 1. 硬件身份

记录：日期、操作员、车辆编号、STM32 固件 hash、C32 型号/序列号、HiPNUC 型号/序列号、
USB VID:PID、稳定 `/dev/serial/by-id/*` 路径、E-stop 方法、车轮架空/车辆约束状态。

## 2. Sensing-only

```bash
ros2 launch autoracer_rc_bringup sensing.launch.py \
  imu_device:=/dev/serial/by-id/hipnuc-imu
ros2 topic hz /sensing/lidar/raw/pointcloud
ros2 topic hz /sensing/lidar/concatenated/pointcloud
ros2 topic hz /sensing/imu/imu_data
ros2 topic echo --once /sensing/lidar/concatenated/pointcloud
ros2 topic echo --once /sensing/imu/imu_data
ros2 run tf2_ros tf2_echo base_link lidar_top
ros2 run tf2_ros tf2_echo base_link imu_link
```

记录 raw 字段/offset、输出 XYZIRC offset、frame、频率、静止重力、角速度正负号、丢包和
driver error。若 live C32 布局不同，先增加失败 fixture，再改 adapter。

## 3. Vehicle-only, drive disabled

```bash
ros2 launch autoracer_rc_bringup vehicle.launch.py \
  serial_port:=/dev/serial/by-id/rc-controller \
  enable_drive_commands:=false
ros2 topic echo --once /vehicle/status/velocity_status
ros2 topic echo --once /vehicle/status/steering_status
ros2 topic echo --once /vehicle/status/gear_status
ros2 topic echo --once /vehicle/status/control_mode
```

保存一帧 24-byte telemetry，核对 `0x7B/0x7D`、长度、BCC、字节序、单位和 flag。
断开串口后必须发布 `NOT_READY`；任何差异先改协议测试，再改实现。

## 4. Secured motion

在操作员握持 E-stop 且车辆固定/车轮架空时，才设置
`enable_drive_commands:=true`。依次验证 stop、`0.2 m/s` 直行、`+0.05 rad`、
`-0.05 rad`、命令超时和串口断开。记录发出的 11-byte frame、轮向、转角方向、status
符号/比例、停止时间和异常。

## 5. Geometry and extrinsics

测量 wheelbase、tread、wheel radius、最大转角以及 `base_link -> lidar_top/imu_link` 六自由度。
记录测量工具、重复次数、误差范围和照片/原始记录路径。只修改 description 中两个 YAML，
然后重跑参数、TF 和低速回归测试。

## 6. Full race, low speed

```bash
ros2 launch autoracer_rc_bringup race.launch.py \
  localization_map_path:="$RC_MAP_PATH" \
  course_path:="$RC_COURSE_PATH" \
  serial_port:=/dev/serial/by-id/rc-controller \
  imu_device:=/dev/serial/by-id/hipnuc-imu \
  enable_drive_commands:=false
```

先在 drive disabled 下确认节点、topic、TF、人工初始化和 NDT/EKF 收敛。之后在受控区域、
E-stop 就绪时单独启用 drive，速度不得超过 `0.5 m/s`。记录初始化、轨迹跟踪、终点停车、
定位丢失停车、status timeout 和串口断开停车证据。

## 结果表

| 项目 | Pass/Fail/Blocked | 证据路径 | 备注 |
|---|---|---|---|
| 固件与设备身份 |  |  |  |
| C32 schema/rate/frame |  |  |  |
| IMU rate/frame/sign |  |  |  |
| UART telemetry/control |  |  |  |
| timeout/disconnect stop |  |  |  |
| 实测 geometry/extrinsics |  |  |  |
| 低速定位与跟踪 |  |  |  |
| 终点与故障停车 |  |  |  |
