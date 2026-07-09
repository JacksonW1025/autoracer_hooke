# 架构图与 Launch 对齐审计

定位：这是 RC 跑通 Hooke/Autoware 全链路之前的前置审计，必须保留。

用途：把 `platform_and_stack_zh.md` 中 Hooke/RC 架构图的每个模块映射到实际 launch、node、topic 和参数，并把差异分成两类：一类是传感器、车辆接口、车辆参数这些允许存在的平台差异；另一类是文档、launch、topic 或参数之间的实现缺口。非用途：不修改算法，不引入新的定位设计，不替代现场运行手册。

## 审计结论

- 当前分支的正式入口改为官方 `autoware_launch`；本仓库提供 RC vehicle profile 和 sensor-kit profile。
- RC 默认 sensing profile 使用 C32、Hipnuc IMU 和 pointcloud filter；UART adapter 通过 official vehicle profile 接入。
- 未来 Hooke profile 应通过同一套官方 profile 模式接入 Hesai/Nebula、Fixposition seed、NDT 和 Hooke2 CAN adapter。
- 当前 official localization 默认消费 `/sensing/lidar/concatenated/pointcloud`；`/sensing/lidar/filtered/pointcloud` 是本仓库 sensor-kit profile 的降采样输出，保留给建图录包、诊断和后续可选预处理。

## 差异处理原则

- 允许差异：LiDAR/IMU 驱动、seed 来源、车辆 URDF/外参/参数、Hooke CAN adapter、RC UART adapter、建图资产生产流程。
- 不允许漂移：shared upper stack 的模块边界、topic 契约、planning/control/gate 链路、地图和定位输入语义。
- 审计发现的允许差异写入本文件；审计发现的实现缺口必须进入后续执行计划或运行手册，不能用口头结论代替。

## RC Launch 链路

| 架构图模块 | launch / script | package / executable | 关键输入 | 关键输出 | 状态 |
| --- | --- | --- | --- | --- | --- |
| RC entrypoint | `scripts/rc/rc_start_autoware.sh` | `scripts/run_official_autoware.sh` | `MAP_PATH` | `autoware_launch/autoware.launch.xml` | 已映射 |
| RC vehicle description / TF | `tier4_vehicle_launch` + `autoracer_rc_description` | `robot_state_publisher` | vehicle/sensor xacro | `/tf_static` | 已映射 |
| RC sensing | `autoracer_rc_sensor_kit_launch/launch/sensing.launch.xml` | `lslidar_driver/lslidar_driver_node` | C32 UDP | `/sensing/lidar/concatenated/pointcloud` | 已映射 |
| RC pointcloud filter | `autoracer_rc_sensor_kit_launch/launch/sensing.launch.xml` | `autoracer_sensing/pointcloud_voxel_filter` | `/sensing/lidar/concatenated/pointcloud` | `/sensing/lidar/filtered/pointcloud` | 已映射 |
| RC IMU raw | `autoracer_rc_sensor_kit_launch/launch/sensing.launch.xml` | `hipnuc_imu/talker` | serial | `/sensing/imu/imu_data_raw` | 已映射 |
| RC IMU filtered | `autoracer_rc_sensor_kit_launch/launch/sensing.launch.xml` | `imu_filter_madgwick/imu_filter_madgwick_node` | `/sensing/imu/imu_data_raw` | `/sensing/imu/imu_data` | 已映射 |
| RC localization | `autoware_launch` localization component | official localization packages | map + `/sensing/lidar/concatenated/pointcloud` + initial pose | `/localization/pose_with_covariance` | 待车端验证 |
| RC kinematic state | official localization/control surface | Autoware components | localization + `/vehicle/status/*` | `/localization/kinematic_state` | 待车端验证 |
| Official planning | `autoware_launch` planning component | official Autoware planning packages | map + localization + route/goal | `/planning/trajectory` and official planning surface | 待车端验证 |
| Official control | `autoware_launch` control component | official Autoware control packages | trajectory + kinematic state + vehicle info | `/control/command/control_cmd` | 待车端验证 |
| Vehicle gate | `autoracer_rc_launch/launch/vehicle_interface.launch.xml` | `autoracer_safety/command_gate` | `/control/command/control_cmd` | `/autoracer/control/safe_control_cmd` | RC adapter 前保留 |
| RC adapter | `autoracer_rc_launch/launch/vehicle_interface.launch.xml` | `autoracer_safety/command_gate` -> `autoracer_vehicle_interface/rc_serial_interface` | `/control/command/*` | UART + `/vehicle/status/*` | 已映射 |

## Hooke Launch 链路

| 架构图模块 | launch / package | package / executable | 关键输入 | 关键输出 | 状态 |
| --- | --- | --- | --- | --- | --- |
| Hooke LiDAR | future Hooke sensor-kit profile | `nebula_hesai/HesaiRosWrapper` | Hesai UDP | `/sensing/lidar/concatenated/pointcloud` | 待按 official profile 建立 |
| Hooke pointcloud filter | future Hooke sensor-kit profile | `autoracer_sensing/pointcloud_voxel_filter` | `/sensing/lidar/concatenated/pointcloud` | `/sensing/lidar/filtered/pointcloud` | 待按 official profile 建立 |
| Hooke Fixposition | future Hooke sensor-kit/localization profile | `fixposition_driver_ros2/fixposition_driver_ros2_exec` | Fixposition stream | `/fixposition/*` | 待按 official profile 建立 |
| Hooke seed | future Hooke localization profile | `autoware_gnss_poser` + `fixposition_seed_filter` | `/fixposition/fix`, `/fixposition/autoware_orientation` | `/localization/fixposition/seed_pose` | 待按 official profile 建立 |
| Hooke NDT | `autoware_launch` localization component | `autoware_ndt_scan_matcher` | pointcloud + seed + map | `/localization/pose_with_covariance` | 待车端验证 |
| Hooke upper stack | `autoware_launch` planning/control components | official Autoware packages or explicit local plugins | localization + map + route | gated control command | 与 RC 共享接口 |
| Hooke adapter | `src/hooke2_vehicle/vehicle_launcher/hooke2_launch/launch/vehicle_interface.launch.xml` | `hooke2_interface` + SocketCAN | `/control/command/*` | CAN + `/vehicle/status/*` | 作为 Hooke reference material 保留；official Hooke profile 仍禁用 |

## RC 入口脚本与 Launch 参数

| 脚本 | 目的 | 强制输入 | 默认关闭项 | 默认开启项 |
| --- | --- | --- | --- | --- |
| `scripts/rc/rc_start_sensors.sh` | 传感器和 TF 输入检查、建图录包准备 | 无 | map、localization、perception、planning、control、API、vehicle interface、RViz、drive commands | vehicle description、sensing driver |
| `scripts/rc/rc_start_localization.sh` | official map localization-only 验证 | `MAP_PATH` | perception、planning、control、API、vehicle interface、drive commands | vehicle description、sensing、map、localization、RViz |
| `scripts/rc/rc_start_autoware.sh` | 完整 RC Autoware 链路启动 | `MAP_PATH` | drive commands | sensing、localization、planning、control、API、vehicle description、vehicle interface |

## 审计后续

1. Hooke profile 的真实部署入口需要继续贴合官方 vehicle/sensor profile 约定，不能恢复本仓库自定义总控或旧 bringup 包。
2. 如果后续关闭 `pointcloud_voxel_filter`，需要同步更新 NDT input topic、架构图和接口契约。
3. 运行静态测试防止旧文档结构回退。
