# 架构图与 Launch 对齐审计

定位：这是 RC 跑通 Hooke/Autoware 全链路之前的前置审计，必须保留。

用途：把 `platform_and_stack_zh.md` 中 Hooke/RC 架构图的每个模块映射到实际 launch、node、topic 和参数，并把差异分成两类：一类是传感器、车辆接口、车辆参数这些允许存在的平台差异；另一类是文档、launch、topic 或参数之间的实现缺口。非用途：不修改算法，不引入新的定位设计，不替代现场运行手册。

## 审计结论

- 当前 shared upper stack 由 `autoracer_planning`、`autoracer_control`、`autoracer_safety` 组成。
- RC 默认 sensing profile 使用 C32、Hipnuc IMU、pointcloud filter、manual seed、NDT、UART adapter。
- Hooke profile 使用 Hesai/Nebula、Fixposition seed、NDT、Hooke2 CAN adapter。
- 当前 shared launch 默认让 NDT 消费 `/sensing/lidar/filtered/pointcloud`；`/sensing/lidar/concatenated/pointcloud` 是 LiDAR driver 输出和建图录包输入。

## 差异处理原则

- 允许差异：LiDAR/IMU 驱动、seed 来源、车辆 URDF/外参/参数、Hooke CAN adapter、RC UART adapter、建图资产生产流程。
- 不允许漂移：shared upper stack 的模块边界、topic 契约、planning/control/gate 链路、地图和定位输入语义。
- 审计发现的允许差异写入本文件；审计发现的实现缺口必须进入后续执行计划或运行手册，不能用口头结论代替。

## RC Launch 链路

| 架构图模块 | launch / script | package / executable | 关键输入 | 关键输出 | 状态 |
| --- | --- | --- | --- | --- | --- |
| RC entrypoint | `scripts/rc/rc_start_autoware.sh` | `scripts/run_track.sh` | `MAP_PATH` | `track.launch.py` | 已映射 |
| RC sensing | `src/autoracer_bringup/launch/sensing.launch.py` | `lslidar_driver/lslidar_driver_node` | C32 UDP | `/sensing/lidar/concatenated/pointcloud` | 已映射 |
| RC pointcloud filter | `src/autoracer_bringup/launch/sensing.launch.py` | `autoracer_sensing/pointcloud_voxel_filter` | `/sensing/lidar/concatenated/pointcloud` | `/sensing/lidar/filtered/pointcloud` | 已映射 |
| RC IMU raw | `src/autoracer_bringup/launch/sensing.launch.py` | `hipnuc_imu/talker` | serial | `/imu/data_raw` | 已映射 |
| RC IMU filtered | `src/autoracer_bringup/launch/sensing.launch.py` | `imu_filter_madgwick/imu_filter_madgwick_node` | `/imu/data_raw` | `/imu/data` | 已映射 |
| RC manual seed | `src/autoracer_bringup/launch/localization.launch.py` | `autoracer_localization/manual_seed_pose_publisher` | `/initialpose` | `/localization/fixposition/seed_pose` | 已映射 |
| RC NDT input | `src/autoracer_bringup/launch/localization.launch.py` | `autoware_ndt_scan_matcher` | `/sensing/lidar/filtered/pointcloud` + `/localization/ndt_initial_pose` | `/localization/pose_with_covariance` | 已映射 |
| RC kinematic state | `src/autoracer_bringup/launch/localization.launch.py` | `autoracer_localization/kinematic_state_publisher` | NDT pose + `/vehicle/status/*` | `/localization/kinematic_state` | 已映射 |
| Shared planning | `src/autoracer_planning/launch/planning.launch.py` | `autoracer_planning/lanelet_route_planner` | map + localization pose + goal | `/planning/trajectory` | 已映射 |
| Shared control | `src/autoracer_control/launch/control.launch.py` | `autoracer_control/pure_pursuit_controller` | trajectory + pose + velocity | `/autoracer/control/raw_control_cmd` | 已映射 |
| Shared gate | `src/autoracer_safety/launch/safety.launch.py` | `autoracer_safety/command_gate` | raw control + localization | `/control/command/control_cmd`, `/control/command/gear_cmd` | 已映射 |
| RC adapter | `src/autoracer_bringup/launch/vehicle.launch.py` | `autoracer_vehicle_interface/rc_serial_interface` | `/control/command/*` | UART + `/vehicle/status/*` | 已映射 |

## Hooke Launch 链路

| 架构图模块 | launch / package | package / executable | 关键输入 | 关键输出 | 状态 |
| --- | --- | --- | --- | --- | --- |
| Hooke LiDAR | `src/autoracer_bringup/launch/sensing.launch.py` | `nebula_hesai/HesaiRosWrapper` | Hesai UDP | `/sensing/lidar/concatenated/pointcloud` | 已映射到 shared launch |
| Hooke pointcloud filter | `src/autoracer_bringup/launch/sensing.launch.py` | `autoracer_sensing/pointcloud_voxel_filter` | `/sensing/lidar/concatenated/pointcloud` | `/sensing/lidar/filtered/pointcloud` | 已映射到 shared launch |
| Hooke Fixposition | `src/autoracer_bringup/launch/sensing.launch.py` | `fixposition_driver_ros2/fixposition_driver_ros2_exec` | Fixposition stream | `/fixposition/*` | 已映射到 conditional launch |
| Hooke seed | `src/autoracer_bringup/launch/localization.launch.py` | `autoware_gnss_poser` + `fixposition_seed_filter` | `/fixposition/fix`, `/fixposition/autoware_orientation` | `/localization/fixposition/seed_pose` | 已映射到 conditional launch |
| Hooke NDT | `src/autoracer_bringup/launch/localization.launch.py` | `autoware_ndt_scan_matcher` | `/sensing/lidar/filtered/pointcloud` + seed + map | `/localization/pose_with_covariance` | 已映射到 shared launch |
| Hooke upper stack | planning/control/safety launch files | local shared packages | localization + map + trajectory | gated control command | 与 RC 共享 |
| Hooke adapter | `src/hooke2_vehicle/vehicle_launcher/hooke2_launch/launch/vehicle_interface.launch.xml` | `hooke2_interface` + SocketCAN | `/control/command/*` | CAN + `/vehicle/status/*` | 已映射到 Hooke2 launch |

## RC 入口脚本与 Launch 参数

| 脚本 | 目的 | 强制输入 | 默认关闭项 | 默认开启项 |
| --- | --- | --- | --- | --- |
| `scripts/rc/rc_start_sensors.sh` | 传感器和 TF 输入检查、建图录包准备 | 无 | localization、planning、control、safety、vehicle、RViz、drive commands | sensing |
| `scripts/rc/rc_start_localization.sh` | PCD/localization-only 验证 | `MAP_PATH` | planning、control、safety、vehicle、drive commands | sensing、localization、RViz |
| `scripts/rc/rc_start_autoware.sh` | 完整 RC Autoware 链路启动 | `MAP_PATH` | drive commands | sensing、localization、planning、control、safety、vehicle、RViz |

## 审计后续

1. Hooke profile 的真实部署入口如果不走当前 shared `track.launch.py`，需要补充对应启动文件并更新 Hooke 表格。
2. 如果后续关闭 `pointcloud_voxel_filter`，需要同步更新 NDT input topic、架构图和接口契约。
3. 运行静态测试防止旧文档结构回退。
