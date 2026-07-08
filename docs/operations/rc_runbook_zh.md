# RC 上车运行手册

用途：给现场运行者一个可重复的上车检查、启动、定位、规划、控制和低速验证流程。非用途：不讨论官方 Autoware 迁移，不记录一次性小测试。

目标：用 RC 验证 Hooke/RC 共享 upper stack，而不是先替换官方组件。

全链路进度、缺口和跨文档依赖以 `docs/operations/rc_full_chain_execution_zh.md` 为准；本文件只写现场操作步骤。

## 1. 主机与网络

- 车端仓库在 `rc-car-migration` 分支。
- 车端已构建或至少能 source workspace。
- LiDAR-facing 网口配置完成：

```bash
sudo -E ./scripts/rc/rc_configure_lidar.sh
```

- 临时主机信息不写入仓库。

## 2. 传感器与 TF

启动 mapping/定位所需传感器：

```bash
IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_start_sensors.sh
```

检查输入：

```bash
./scripts/check_mapping_inputs.sh
```

验收项：

- C32 输出 `/sensing/lidar/concatenated/pointcloud`。
- 点云 frame 为 `lidar_top`。
- Hipnuc IMU 输出 `/sensing/imu/imu_data_raw` 和 `/sensing/imu/imu_data`。
- `base_link -> lidar_top` 和 `base_link -> imu_link` 可查。
- RC 第一版外参可用 legacy 值，实车后必须复核。

## 3. 底盘反馈

串口节点应发布：

- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/vehicle/status/gear_status`
- `/vehicle/status/control_mode`

若串口协议未变，现有固件可先测链路。物理标定前必须确认固件版本、车身参数、速度尺度、转角尺度和 deadband。

## 4. 地图

完整地图目录必须包含：

```text
pointcloud_map.pcd
pointcloud_map_metadata.yaml
lanelet2_map.osm
map_projector_info.yaml
```

official localization-only 也需要完整地图目录；缺少 Lanelet2 或 projector 资产时先补齐地图，不声明车端 localization 已可验证。

## 5. Localization

启动 localization-only：

```bash
MAP_PATH=/home/milesli/autoracer_maps/<map_name> \
IMU_SERIAL_PORT=/dev/ttyUSB0 \
./scripts/rc/rc_start_localization.sh
```

验收项：

- RViz/ROS 发布 `/initialpose`。
- `/localization/pose_with_covariance` 持续更新。
- `/localization/kinematic_state` 有输出。
- `map -> base_link` TF 稳定。

不使用 AMCL/slam_toolbox。

## 6. Planning

- `lanelet_route_planner` 读取 `lanelet2_map.osm`。
- 输入 `/localization/pose_with_covariance` 和 `/goal_pose`。
- 输出 `/planning/trajectory`。
- 第一轮只用简单短路线。

这是共享 upper stack 的 planning 部分，先验证，不先替换 official mission planner。

## 7. Control/Gate/Adapter

- `pure_pursuit_controller` 输出 `/autoracer/control/raw_control_cmd`。
- `command_gate` 输出 `/control/command/control_cmd` 和 `/control/command/gear_cmd`。
- `rc_serial_interface` 把 Autoware command 转成 STM32 串口帧。
- 默认 `ENABLE_DRIVE_COMMANDS=false`。
- 第一轮实车建议 `MAX_SPEED_MPS=0.5~0.8`。

完整启动：

```bash
MAP_PATH=/path/to/map ./scripts/rc/rc_start_autoware.sh
```

低速使能前必须完成标定检查：

```bash
MAP_PATH=/path/to/map SERIAL_PORT=/dev/<actual_chassis_tty> ENABLE_DRIVE_COMMANDS=true ./scripts/rc/rc_start_autoware.sh
./scripts/request_autonomous_mode.sh
```

停止 RC Autoware 相关节点：

```bash
./scripts/rc/rc_stop.sh
```

## 8. 实际跑车顺序

1. 接 LiDAR、控制板、底盘动力。
2. 烧录最新 STM32 固件，或确认当前固件协议和车身参数与源码一致。
3. 配置 LiDAR 网口。
4. 启动 sensors，跑 `check_mapping_inputs.sh`。
5. 设置 `MAP_PATH`。
6. 启动 official Autoware wrapper，保持 `ENABLE_DRIVE_COMMANDS=false`。
7. 给 `/initialpose`，确认 NDT 和 TF。
8. 给短 `/goal_pose`，确认 trajectory 和 raw control。
9. 架空或低速场地设置 `ENABLE_DRIVE_COMMANDS=true`。
10. 验证速度符号、转角方向、停止行为，再提高测试复杂度。
