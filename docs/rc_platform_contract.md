# RC Platform Contract

RC 是共享 Autoracer race core 的薄平台目标，不是第二套自动驾驶栈。平台代码只负责把
真实传感器转换成共享输入、把共享控制命令转换成底盘 UART，以及提供车辆物理参数。

## 组合边界

```text
LeiShen C32 -> lslidar_driver -> c32_pointcloud_adapter ┐
HiPNUC IMU -> hipnuc_imu -> imu_filter_madgwick        ├-> shared core
RC geometry/extrinsics --------------------------------┘
shared /control/command/* -> rc_serial_interface -> STM32 UART
STM32 telemetry -> rc_serial_interface -> shared /vehicle/status/*
```

`autoracer_rc_bringup/race.launch.py` 只 include RC sensing、RC vehicle 和
`autoracer_bringup/race.launch.py`。定位、规划、控制、VehicleCmdGate 和运行管理只由
`src/core` 提供。

## 标准 ROS 接口

| 方向 | Topic | 类型/语义 | Frame |
|---|---|---|---|
| core 输入 | `/sensing/lidar/concatenated/pointcloud` | `sensor_msgs/PointCloud2`, 16-byte XYZIRC | `lidar_top` |
| core 输入 | `/sensing/imu/imu_data` | `sensor_msgs/Imu`, 滤波后 IMU | `imu_link` |
| core 输入 | `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | `base_link` |
| core 输入 | `/vehicle/status/steering_status` | `autoware_vehicle_msgs/SteeringReport` | vehicle time |
| core 输入 | `/vehicle/status/gear_status` | `autoware_vehicle_msgs/GearReport` | vehicle time |
| core 输入 | `/vehicle/status/control_mode` | `ControlModeReport`; 串口断开为 `NOT_READY` | vehicle time |
| 平台输入 | `/control/command/control_cmd` | `autoware_control_msgs/Control` | — |
| 平台输入 | `/control/command/gear_cmd` | `autoware_vehicle_msgs/GearCommand` | — |
| 平台服务 | `/control/control_mode_request` | `ControlModeCommand` | — |

RC 不发布伪 GNSS。初始位姿由共享 AD API/人工初始化流程提供。

## Frame 与参数所有权

- `base_link -> lidar_top` 和 `base_link -> imu_link` 只存在于
  `autoracer_rc_description/config/sensor_extrinsics.yaml`。
- 车辆尺寸只存在于 `autoracer_rc_description/config/vehicle_info.param.yaml`。
- 控制、gate、runtime 的 RC 差异只存在于 `autoracer_rc_bringup/config/rc/` overlays。
- core launch 只声明参数 schema，不包含 RC 设备名、串口、传感器品牌或静态 TF。

当前 `wheel_radius=0.115`、`wheel_base=0.600`、`wheel_tread=0.440`、
`max_steer_angle=0.262` 以及零外参均为 `reference_unverified`。它们必须由当前车辆测量
记录替换后，才能把硬件集成标为完成。

## 固件与第三方依赖

- STM32 协议依据：`RCCar-Firmware` commit
  `4113141f1ac5ba1af276db3c2bace81b5bcf1d16`。
- LeiShen：`Lslidar_ROS2_driver` commit
  `08d692c2adf62f29b991fe44313b17840e4bea8b`。
- HiPNUC：`hipnuc/products` commit
  `5a4380272cd70402e7f8928b05a6af4bfa659807`。

串口控制为 11 字节，遥测为 24 字节，115200 baud，big-endian signed milliscale，XOR
BCC。命令超过 `0.5 s`、drive 未启用或串口断开时，平台必须 fail closed。

## 明确禁止

- 不把 legacy 仓库加入依赖、CMake、环境或 launch。
- 不复制 legacy profile、顶层 launch、第二个 VehicleCmdGate 或历史
  `safe_control_cmd` 路径。
- 不在 core 增加 `if rc` 分支。
- 不用字段重命名伪装不兼容的 PointCloud2 字节布局。
- 不用 Hooke2/CarMaker 地图或赛道替代未经验证的 RC 资产。
