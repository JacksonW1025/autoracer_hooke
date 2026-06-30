# RC 跑车剩余清单

目标是让 RC 车按 Autoracer/Autoware 链路跑起来，而不是给 RC 另做一套独立算法。
本清单按链路模块列出“已验证”“还缺什么”“跑车前验收口径”。

## 1. 车载主机与网络

已验证：
- 树莓派仓库已同步到 `rc-car-migration` 分支。
- SSH 走 `wlan0=192.168.1.136/24`。
- C32 雷达走唯一有线口 `eth0`。
- C32 实际 UDP 目的地址是 `192.168.1.102:2368`，不是 `192.168.1.120`。
- `scripts/configure_rc_lidar_link.sh` 应配置 `eth0=192.168.1.102/32`，并保留
  `192.168.1.200/32` host route。这样回开发机的路由仍走 `wlan0`。

还缺：
- 跑车前在 Pi 上执行一次：

```bash
sudo -E ./scripts/configure_rc_lidar_link.sh
ip -br addr
ip route get 192.168.1.103
ip route get 192.168.1.200
```

验收口径：
- `ip route get 192.168.1.103` 走 `wlan0`。
- `ip route get 192.168.1.200` 走 `eth0 src 192.168.1.102`。

## 2. 传感器输入

已验证：
- C32 LiDAR 已能发布 `/sensing/lidar/concatenated/pointcloud`。
- 最新 bench 结果：点云约 `20 Hz`，有 `PointCloud2` sample。
- RC LiDAR 外参当前使用旧 Nav2 值：`base_link -> lidar_top` 为
  `x=0.24, y=0.0, z=0.39, yaw=-1.5708`。
- RC 没有 Fixposition/ZED；当前链路不依赖这两个设备。

还缺：
- LiDAR 外参后续需要实测复核，但当前值可以作为第一轮跑车初值。
- IMU 可以接入但当前定位链路暂不使用。NDT + 轮速运动预测是当前可跑路径。

验收口径：

```bash
LAUNCH_LIDAR=true LAUNCH_FIXPOSITION=false LAUNCH_VEHICLE=false ./scripts/verify_sensing_feedback.sh
```

必须看到：
- `/sensing/lidar/concatenated/pointcloud` topic exists。
- rate 有估计值。
- sample 成功。

## 3. 底盘反馈与串口接口

已验证：
- Pi 上控制板实际串口是 `/dev/ttyACM0`。
- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/vehicle/status/gear_status`
- `/vehicle/status/control_mode`

这些状态 topic 均能出样本。

还缺：
- 固件还没烧录最新车身参数时，速度数值只能证明链路通，不能作为物理标定结果。
- 下次拿到烧录器后，应烧录 `RCCar-Firmware` 最新版本，或明确确认当前固件已包含：
  `wheelbase=0.6 m`、`wheel_diameter=0.23 m`、速度/转角限幅与死区。
- 底盘动力未上电时，不能验证实际电机响应、舵机角度闭环和速度符号。

验收口径：

```bash
SERIAL_PORT=/dev/ttyACM0 LAUNCH_LIDAR=false LAUNCH_FIXPOSITION=false CHECK_HOOKE2_RAW=false ./scripts/verify_sensing_feedback.sh
```

必须四个 `/vehicle/status/*` 全部有 sample。

## 4. 地图资产

当前状态：
- `maps/` 目录还没有实地图资产。
- 这是完整跑车的最大缺口。

需要导入的文件：
- `pointcloud_map.pcd`
- `pointcloud_map_metadata.yaml`
- `lanelet2_map.osm`
- `map_projector_info.yaml`

要求：
- PCD 和 OSM 必须在同一个 `map` 坐标系下对齐。
- `map_projector_info.yaml` 必须和 OSM/点云生成时的投影一致。
- 轨迹车道中心线要能被 `lanelet_route_planner` 取到 centerline。

验收口径：
- map loader 能发布 `/map/pointcloud_map` 或提供 partial map service。
- NDT 能从 `/sensing/lidar/concatenated/pointcloud` 匹配到地图。
- 规划收到 `/goal_pose` 后能发布 `/planning/trajectory`。

## 5. 定位链路

当前链路：
- RViz/ROS `/initialpose` 或手动参数输入。
- `manual_seed_pose_publisher` 发布 `/localization/fixposition/seed_pose`。
- `ndt_initial_pose_predictor` 用 seed + 轮速/转角预测
  `/localization/ndt_initial_pose`。
- `autoware_ndt_scan_matcher` 输出 `/localization/pose_with_covariance`。
- `kinematic_state_publisher` 输出 `/localization/kinematic_state`。

还缺：
- 真实地图导入后，必须做 NDT 初始位姿验证。
- NDT 参数目前沿用 Hooke2 默认：
  `resolution=2.0`、`required_distance=10.0`、`map_radius=150.0`。
  RC/C32 地图上如果收敛慢或误匹配，需要基于实测日志调整。
- 当前不是 EKF/IMU 融合定位。IMU 后续要做滤波器时再进入链路。

验收口径：
- 给定初始位姿后，`/localization/pose_with_covariance` 持续更新。
- `/tf map -> base_link` 稳定。
- `/localization/kinematic_state` 有 50Hz 左右输出。

## 6. 规划链路

当前链路：
- `lanelet_route_planner` 读取 `lanelet2_map.osm`。
- 输入 `/localization/pose_with_covariance` 和 `/goal_pose`。
- 输出 `/planning/mission_path`、`/planning/trajectory`。

还缺：
- 地图里的 lanelet centerline 必须覆盖测试路线。
- 第一轮应选择简单闭环/短路线，不要直接验证复杂路口或倒车场景。
- 当前 planner 是最小 centerline planner，不是完整 Autoware mission planner。

验收口径：
- RViz 发 `/goal_pose` 后，`/planning/trajectory` 有点列。
- 轨迹速度由 `MAX_SPEED_MPS` 限制。

## 7. 控制与安全门

当前链路：
- `pure_pursuit_controller` 输出 `/autoracer/control/raw_control_cmd`。
- `command_gate` 校验 localization 和 raw command freshness 后输出：
  `/control/command/control_cmd`、`/control/command/gear_cmd`。
- `rc_serial_interface` 把 Autoware `Control + GearCommand` 转成 STM32 串口帧。

已调整：
- RC 默认 `wheel_base_m=0.6`。
- RC 默认 `max_steer_rad=0.262`。
- RC 默认 `max_speed_mps=3.0`。
- RC 默认 `control_min_lookahead_m=1.0`、`control_lookahead_gain=1.0`、
  `control_goal_tolerance_m=0.35`。
- `enable_drive_commands=false` 仍是默认值，防止启动即给动力。

还缺：
- 真车/RC 第一轮实际行驶前，建议把 `MAX_SPEED_MPS` 临时降到 `0.5~0.8`
  做闭环验证；这不是改算法，只是首次现场验收限速。
- 后续如果切换到 Autoware 标准 MPC/PID，要补齐标准控制器需要的
  vehicle_info、trajectory、kinematic_state/odometry 契约。
- 当前 `vehicle_info.param.yaml` 仍是 Hooke2 尺寸，不应拿它代表 RC。

验收口径：
- 定位正常后，`command_gate` state 从 `localization_timeout` 变为
  `drive_enabled`。
- `ENABLE_DRIVE_COMMANDS=true` 前必须确认车架架空或现场安全。

## 8. 车辆模型与 URDF

已有：
- `rc_minimal.urdf.xacro` 已有 RC 尺寸：
  长 `0.83 m`、宽 `0.58 m`、高 `0.50 m`、轮半径 `0.115 m`。
- 静态 TF 使用 `rc_sensor_extrinsics.yaml`，不依赖 Hooke2 传感器。

还缺：
- track 链路当前主要使用 static TF，不完整依赖 robot_state_publisher。
- 若后续需要 Autoware 标准 vehicle_info 或更完整可视化，应新增 RC
  `vehicle_info.param.yaml`，不要继续用 Hooke2 的 `wheel_base=1.9` 等参数。

验收口径：
- `base_link -> lidar_top` TF 正确。
- 后续切换标准 Autoware 控制/规划时，vehicle_info 和 URDF 数值一致。

## 9. 实际跑车顺序

建议按这个顺序推进：

1. 接好 LiDAR、控制板、底盘动力；烧录或确认最新 STM32 固件。
2. Pi 上执行 `sudo -E ./scripts/configure_rc_lidar_link.sh`。
3. 跑 sensing + feedback bench，确认 LiDAR 和 `/vehicle/status/*` 全通过。
4. 导入地图目录，设置 `MAP_PATH`。
5. 启动 track，但保持 `ENABLE_DRIVE_COMMANDS=false`。
6. RViz 给初始位姿，确认 NDT pose 和 TF 稳定。
7. RViz 给短目标点，确认 `/planning/trajectory` 和 raw control 输出。
8. 架空或低速场地内设置 `ENABLE_DRIVE_COMMANDS=true`，先限速 `0.5~0.8 m/s`。
9. 确认速度符号、转角方向、急停/停止命令，再提高速度或进入完整路线。
