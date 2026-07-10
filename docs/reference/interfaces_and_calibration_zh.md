# 接口、Topic 与标定事实

用途：集中记录长期接口事实，包括 topic、消息类型、frame、LiDAR/IMU 输入、Hooke CAN adapter、RC UART adapter、车辆几何和低速标定检查。非用途：不写现场启动步骤，不记录一次性排查命令。

## Topic 契约

| Topic | 类型 | 生产者 | 消费者 | 语义 |
| --- | --- | --- | --- | --- |
| `/sensing/lidar/concatenated/pointcloud` | `sensor_msgs/msg/PointCloud2` | Hooke Hesai/Nebula 或 RC C32/lslidar | pointcloud filter、建图录包、official localization | LiDAR driver 输出，frame 为 `lidar_top`。 |
| `/sensing/lidar/filtered/pointcloud` | `sensor_msgs/msg/PointCloud2` | `pointcloud_voxel_filter` | 建图录包、诊断、后续可选预处理输入 | 当前 official localization 默认不消费该 topic。 |
| `/localization/fixposition/seed_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Hooke Fixposition seed 或 RC manual seed | NDT initial pose predictor、NDT regularization input | 名字带 Fixposition，但语义是 localization seed。 |
| `/localization/ndt_initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `ndt_initial_pose_predictor` | NDT scan matcher | NDT 启动/重定位初始位姿。 |
| `/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | NDT scan matcher | official Autoware planning/control、gate、状态诊断 | map frame 定位输出。 |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | official localization/control surface | official Autoware planning/control、诊断 | 必须使用 Autoware 单位和 frame。 |
| `/planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | official Autoware planning | official Autoware control | 上层规划结果，包含目标速度。 |
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | official Autoware control | `command_gate` | 官方控制输出，进入实车 adapter 前必须经过 gate。 |
| `/autoracer/control/safe_control_cmd` | `autoware_control_msgs/msg/Control` | `command_gate` | RC serial adapter，未来 gated Hooke adapter | adapter-facing safe control；禁用、超时或定位丢失时为 stop。 |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | `command_gate` | Hooke2 CAN adapter 或需要挡位的 vehicle adapter | 挡位命令。 |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | Hooke2 CAN adapter 或 RC serial adapter | localization、control | 纵向速度和 yaw rate，单位必须符合 Autoware。 |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | Hooke2 CAN adapter 或 RC serial adapter | localization、diagnostics | 前轮转角反馈。 |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | Hooke2 CAN adapter 或 RC serial adapter | diagnostics、gate/adapter 检查 | 实际挡位反馈。 |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | Hooke2 CAN adapter 或 RC serial adapter | diagnostics、接管状态检查 | 底盘控制模式反馈。 |

## LiDAR

LiDAR driver output contract:

```text
/sensing/lidar/concatenated/pointcloud  sensor_msgs/msg/PointCloud2
```

Filtered pointcloud output for mapping bag capture and diagnosis:

```text
/sensing/lidar/filtered/pointcloud  sensor_msgs/msg/PointCloud2
```

Mapping bag capture keeps both `/sensing/lidar/concatenated/pointcloud` and `/sensing/lidar/filtered/pointcloud`; offline mapping can choose either raw driver output or filtered diagnostic output. Current official localization uses the upstream default input topic, so runtime localization consumes the official default concatenated topic unless a later profile explicitly overrides it. 当前 official localization 默认消费 `/sensing/lidar/concatenated/pointcloud`。

Hooke uses Hesai Pandar through `nebula_hesai`. A future Hooke deployment must provide the exact model, frame and network facts through a dedicated official sensor-kit profile.

The RC official sensor-kit profile uses Leishen C32 through `lslidar_driver` with `src/autoracer_rc_sensor_kit_launch/config/lslidar_cx.yaml`: `device_ip=192.168.1.200`, `msop_port=2368`, `difop_port=2369`. It publishes directly to `/sensing/lidar/concatenated/pointcloud` in frame `lidar_top`.

The underlying helper uses `192.168.1.102/32` on the LiDAR-facing Ethernet link and a host route to `192.168.1.200/32`, keeping the normal LAN/WiFi route separate. Hostnames, SSH identities, and operator-machine addresses are runtime environment details, not architecture constants.

Do not use Nav2 `/scan` localization as an RC replacement for this point cloud contract.

## Fixposition 与 RC Seed

Hooke launches the Fixposition ROS 2 driver directly as `fixposition_driver_ros2_exec`. The localization-relevant topics are:

```text
/fixposition/fix                    sensor_msgs/msg/NavSatFix
/fixposition/autoware_orientation   autoware_sensing_msgs/msg/GnssInsOrientationStamped
/fixposition/rawimu                 sensor_msgs/msg/Imu
/fixposition/odometry_enu           nav_msgs/msg/Odometry
/fixposition/speed                  fixposition_driver_msgs/msg/Speed
```

`/fixposition/fix` and `/fixposition/autoware_orientation` feed `autoware_gnss_poser`, which publishes `/sensing/gnss/pose_with_covariance` for NDT initialization and regularization.

The RC profile disables Fixposition. RViz/ROS `/initialpose` is republished as `/localization/fixposition/seed_pose`, preserving the NDT startup contract without pretending the RC has a Fixposition device.

## Hooke2 CAN Adapter

Hooke2 bottom-control chain:

```text
/control/command/control_cmd
/control/command/gear_cmd
/control/command/turn_indicators_cmd
/control/command/hazard_lights_cmd
/control/command/emergency_cmd
        |
        v
hooke2_interface
        |
        | publishes /can_rx_from_autoware
        v
can_driver
        |
        | SocketCAN can0, 500000 bps
        v
Hooke2 chassis CAN bus
```

Feedback returns on `/can_tx_to_autoware`, then `hooke2_interface` republishes vehicle status topics:

```text
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/steering_wheel_status
/vehicle/status/gear_status
/vehicle/status/control_mode
```

Do not consume raw Hooke chassis reports outside tiny adapters and debugging tools. Use `/vehicle/status/velocity_status` and `/vehicle/status/steering_status` for localization, planning, and control consumers.

## RC UART Adapter

RC replaces only the Hooke2 CAN transport:

```text
/control/command/control_cmd
  -> autoracer_safety/command_gate
  -> /autoracer/control/safe_control_cmd
  -> autoracer_vehicle_interface/rc_serial_interface
  -> 0x7B cmd1 cmd2 vx vy wz bcc 0x7D
  -> STM32 UART4
```

RC vehicle interface keeps the official Autoware control topic on the upstream side of `command_gate`; the serial adapter consumes the gated safe command. `SERIAL_PORT` and `SERIAL_BAUDRATE` are runtime settings, not architecture constants.

The adapter publishes the same `/vehicle/status/*` surface as Hooke2. Wheel speed, velocity sign, steering angle, and `wz` definitions must be checked against the STM32 protocol and `rc_serial_interface` implementation before dynamic tests.

The only adapter added around Hooke Fixposition compatibility is `velocity_to_fixposition_speed`, which bridges `/vehicle/status/velocity_status` to `/fixposition/speed` as a single `RC` wheelspeed measurement in millimeters per second.

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
- RC profile: `base_link -> lidar_top` starts from the current C32 value but must be rechecked on the car.
- RC profile: `base_link -> imu_link` starts as zero pose for Hipnuc/N300 Pro and must be updated after physical measurement.
- `map -> base_link` moves smoothly while driving slowly on the mapped track.

## 低速标定检查

Run these checks before setting `ENABLE_DRIVE_COMMANDS=true`.

1. Keep wheels lifted or vehicle secured.
2. Run with `ENABLE_DRIVE_COMMANDS=false`.
3. Confirm route, trajectory, official control, and gated safe control direction in RViz.
4. Enable drive commands at `MAX_SPEED_MPS=0.5` or another explicitly chosen low-speed limit.
5. Verify stop on localization loss, control command timeout, and route completion.
