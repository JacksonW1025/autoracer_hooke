# RC 建图到 Autoware 回灌流程

用途：定义可重复的 RC 建图工作流：车端采集 bag，工作机用 Foxglove 检查 bag，用 Super-LIO 建图并打包 Autoware 地图，再回灌车端验证；现场需要时，也可用 Foxglove Bridge 实时查看车端 ROS 2 topic。非用途：不承载车端运行态 Autoware/RViz 配置。

全链路进度、缺口和上下游依赖以 `docs/operations/rc_full_chain_execution_zh.md` 为准；本文件只写建图和地图回灌流程。

## 工作区

```text
/home/milesli/Desktop/RC/rc_mapping_ws
/home/milesli/Desktop/RC/rc_mapping_data
```

数据目录：

```text
rc_mapping_data/
  bags/raw/
  bags/checked/
  runs/<run_id>/
  autoware_maps/<map_name>/
  reports/<run_id>.md
```

## 现场扫图速查

本节是上车采集 ROS bag 的操作入口。扫图过程不在工作机上完成；车端只负责启动传感器、发布 TF、录制 bag。工作机在 bag 拉回后再检查、可视化、离线建图和打包地图。

车端进入 `autoracer_hooke` 工作区：

```bash
cd <autoracer_hooke工作区>
```

默认车端 bag 保存位置：

```text
~/autoracer_mapping_bags/<RUN_ID>/
```

这是 ROS 2 bag 目录，至少应包含：

```text
metadata.yaml
*.db3
```

### 1. 录制前输入检查

先启动传感器和静态 TF：

```bash
IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_start_sensors.sh
```

另开一个终端，在同一个工作区检查输入：

```bash
./scripts/check_mapping_inputs.sh
```

期望看到：

```text
[mapping-check] OK topic data: /sensing/lidar/concatenated/pointcloud
[mapping-check] OK topic data: /sensing/imu/imu_data_raw
[mapping-check] OK topic data: /sensing/imu/imu_data
[mapping-check] OK topic data: /tf_static
[mapping-check] OK pointcloud fields: x y z intensity ring time
[mapping-check] mapping inputs look usable
```

检查完成后停止传感器：

```bash
./scripts/rc/rc_stop.sh
```

### 2. 短 bag 试录

正式扫楼层前，先录 30 到 60 秒短 bag。这个步骤用于验证 C32、IMU、TF、bag 字段和时间轴，不用于最终地图质量判断。

```bash
BAG_DURATION_SEC=60 \
RUN_ID=floor_test_001 \
IMU_SERIAL_PORT=/dev/ttyUSB0 \
./scripts/rc/rc_capture_mapping_bag.sh
```

脚本会自动启动传感器、检查输入、录制、停止录制，并打印 bag 信息和路径：

```text
[rc-capture] bag path: /home/<user>/autoracer_mapping_bags/floor_test_001
```

### 3. 正式开放时长扫图

正式扫图建议使用开始/停止两步命令，方便人工推车或低速人工驾驶完成完整路径。

开始录制：

```bash
RUN_ID=floor1_mapping_001 \
IMU_SERIAL_PORT=/dev/ttyUSB0 \
./scripts/rc/rc_start_mapping_bag.sh
```

开始命令会启动：

- C32 LiDAR
- Hipnuc IMU
- 点云过滤节点
- 静态 TF
- ROS bag recorder

不会启动 localization、planning、control、vehicle interface；`ENABLE_DRIVE_COMMANDS=false`，不会向底盘输出有效自动驾驶命令。

结束录制：

```bash
./scripts/rc/rc_stop_mapping_bag.sh
```

结束命令会优雅停止 recorder，停止传感器栈，打印 `ros2 bag info`，并清理本次录包状态文件。

### 4. 正式 bag 话题

录包脚本会录：

```text
/sensing/lidar/concatenated/pointcloud
/sensing/lidar/filtered/pointcloud
/sensing/imu/imu_data_raw
/sensing/imu/imu_data
/tf
/tf_static
/rosout
```

后处理必须有：

```text
/sensing/lidar/concatenated/pointcloud
/sensing/imu/imu_data_raw
/sensing/imu/imu_data
/tf_static
/rosout
```

`/tf` 是推荐项。有动态 TF publisher 时应存在；如果当前启动图只有静态 TF，bag 中没有 `/tf` 只作为警告处理。

### 5. 把 bag 拉回工作机

在工作机执行：

```bash
cd /home/milesli/Desktop/RC/autoracer_hooke

VEHICLE_HOST=<user@vehicle-host> \
VEHICLE_BAG=/home/<user>/autoracer_mapping_bags/floor1_mapping_001 \
./scripts/pull_mapping_bag.sh
```

拉回后默认保存到：

```text
/home/milesli/Desktop/RC/rc_mapping_data/bags/raw/floor1_mapping_001/
```

后续工作机检查和建图都以这个目录为输入。

## 车端采集

启动传感器：

```bash
IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_start_sensors.sh
```

检查输入：

```bash
./scripts/check_mapping_inputs.sh
```

定长录包：

```bash
BAG_DURATION_SEC=60 IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_capture_mapping_bag.sh
```

开放式录包：

```bash
IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_start_mapping_bag.sh
./scripts/rc/rc_stop_mapping_bag.sh
```

建图必录 topic：`/sensing/lidar/concatenated/pointcloud`、`/sensing/lidar/filtered/pointcloud`、`/sensing/imu/imu_data_raw`、`/sensing/imu/imu_data`、`/tf`、`/tf_static`、`/rosout`。

诊断可选 topic：`/vehicle/status/velocity_status`、`/vehicle/status/steering_status`、`/vehicle/status/gear_status`、`/autoracer/vehicle_interface/state`。

## 车端实时 Foxglove 监控

实时监控用于现场查看车端正在发布的点云、IMU、TF 和诊断 topic；它不替代 ROS bag 录制，也不是 Autoware 运行界面。客户端机器只需要安装 Foxglove Studio，不需要 ROS/Autoware 环境。

车端首次准备 Foxglove Bridge：

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

启动传感器后，在车端另开终端启动 bridge：

```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

默认会监听 `0.0.0.0:8765`。车端确认端口：

```bash
ss -tulpen | grep 8765
```

客户端打开 Foxglove Studio，选择 Foxglove WebSocket，连接：

```text
ws://<vehicle-ip>:8765/
```

当前现场树莓派常用地址是 `192.168.1.136`，但实际连接地址以车端 `ip -br addr` 输出为准。104 笔记本只作为 Foxglove 客户端，不需要预设连接参数；也可以手动输入：

```text
ws://192.168.1.136:8765/
```

需要停止实时监控时，在运行 bridge 的终端按 `Ctrl-C`。如果 bridge 是后台启动的，先确认进程再停止：

```bash
pgrep -af 'foxglove_bridge|ros2 launch foxglove_bridge'
```

## 工作机检查与建图

进入建图工作区：

```bash
cd /home/milesli/Desktop/RC/rc_mapping_ws
./bootstrap_mapping_ws.sh
```

用 Foxglove 查看 ROS bag：

```bash
./view_bag_foxglove.sh /home/milesli/Desktop/RC/rc_mapping_data/bags/raw/<bag>
```

Foxglove 只属于建图工作流，用来快速查看录下来的 ROS bag 是否完整、时间轴是否连续、点云/IMU/TF 是否存在；它不是 Autoware 运行界面。没有 Foxglove CLI 时，手动打开 Foxglove Studio 并加载 bag；RViz 备用入口：

```bash
./view_bag_rviz.sh /home/milesli/Desktop/RC/rc_mapping_data/bags/raw/<bag>
```

检查一个 bag 是否满足建图输入：

```bash
./inspect_bag_topics.sh /home/milesli/Desktop/RC/rc_mapping_data/bags/raw/<bag>
```

一键检查 bag 并离线运行 Super-LIO：

```bash
./run_mapping_pipeline.sh /home/milesli/Desktop/RC/rc_mapping_data/bags/raw/<bag> <run_id>
```

带 `<map_name>` 时，pipeline 会继续打包官方 Autoware 地图目录：

```bash
./run_mapping_pipeline.sh /home/milesli/Desktop/RC/rc_mapping_data/bags/raw/<bag> <run_id> <map_name>
```

静止 bag 只用于验证流程，不代表有效导航地图。有效地图必须来自运动 bag。

## Autoware 地图目录

完整地图目录：

```text
<map_name>/
  pointcloud_map.pcd
  pointcloud_map_metadata.yaml
  lanelet2_map.osm
  map_projector_info.yaml
```

打包完整地图：

```bash
./package_autoware_map.sh <run_id> <map_name> /path/to/lanelet2_map.osm /path/to/map_projector_info.yaml
```

官方 `autoware_launch` 的 map component 会加载 `lanelet2_map.osm` 和 `map_projector_info.yaml`。只有点云地图、暂时没有 Lanelet/projector 资产时，先补齐地图资产，不在车端声明 localization-only 已可验证。

## 回灌和验证

同步地图回车端：

```bash
VEHICLE_HOST=user@host ./sync_map_to_vehicle.sh <map_name>
```

验证顺序：

1. 设置 `MAP_PATH=<map_dir>`。
2. 启动 localization-only。
3. RViz 发布 `/initialpose`。
4. 确认 NDT pose 和 `map -> base_link` TF。
5. 低速前再启动完整 official Autoware 链路。
6. 底盘供电后做低速 dynamic check。

## 缺口与验证口径

当前缺口：有效运动 bag、有效 PCD 地图、`lanelet2_map.osm`、`map_projector_info.yaml`、C32/IMU 外参实测复核、固件协议/车身参数确认。

x86 只验证脚本、建图工具、bag 检查和地图打包；ARM 车端验证传感器、NDT、共享 upper stack 和 vehicle adapter。无底盘动力阶段只验证 topic、节点状态和零速输出。
