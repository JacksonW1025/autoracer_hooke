# RC 建图到 Autoware 回灌流程

用途：定义可重复的 RC 建图工作流：车端采集 bag，工作机用 Foxglove 检查 bag，用 Super-LIO 建图并打包 Autoware 地图，再回灌车端验证。非用途：不承载车端运行态 Autoware/RViz 配置。

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

建图必录 topic：`/sensing/lidar/concatenated/pointcloud`、`/imu/data_raw`、`/imu/data`、`/tf`、`/tf_static`、`/rosout`。

诊断可选 topic：`/vehicle/status/velocity_status`、`/vehicle/status/steering_status`、`/vehicle/status/gear_status`、`/autoracer/vehicle_interface/state`。

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

带 `<map_name>` 时，pipeline 会继续打包 localization-only 地图：

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

只有点云地图、暂时没有 Lanelet/projector 资产时，只能做 NDT/localization-only 验证：

```bash
LOCALIZATION_ONLY=true ./package_autoware_map.sh <run_id> <map_name>
```

没有 `lanelet2_map.osm` 时，只做 localization/NDT 验证，不声明规划导航完成。

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
5. OSM 到位后再启动完整 track 链路。
6. 底盘供电后做低速 dynamic check。

## 缺口与验证口径

当前缺口：有效运动 bag、有效 PCD 地图、`lanelet2_map.osm`、`map_projector_info.yaml`、C32/IMU 外参实测复核、固件协议/车身参数确认。

x86 只验证脚本、建图工具、bag 检查和地图打包；ARM 车端验证传感器、NDT、共享 upper stack 和 vehicle adapter。无底盘动力阶段只验证 topic、节点状态和零速输出。
