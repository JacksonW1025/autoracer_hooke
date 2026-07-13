# Autoracer Hooke

精简的 ROS 2 封闭赛道竞速栈。产品仓只版本化自研算法、Hooke2 平台适配、已验证资源和
依赖定义；Autoware 等第三方源码位于仓内可再生的 `vendor_ws` underlay，不混入产品源码树。

```text
CarMaker / Hooke2 sensors
  -> pilot-compatible NDT + EKF localization
  -> validated fixed-course local trajectory
  -> Autoware MPC lateral + PID longitudinal control
  -> thin race runtime manager + Autoware vehicle command gate
  -> CarMaker Bridge / Hooke2 CAN interface
```

## 目录边界

```text
autoracer_hooke/
  courses/                         已验证固定赛道轨迹
  maps/                            当前点云地图
  dependencies/                    精确依赖清单、版本锁和最小补丁
  scripts/                         导入、构建、环境和硬件诊断脚本
  src/core/                        平台无关的定位、规划、控制和运行管理
  src/platform/hooke2/             Hooke2 CAN、车辆接口、消息和实车启动
  vendor_ws/                       99 个第三方 ROS 包的可再生 underlay（Git 忽略）
```

`src/core` 不依赖 CarMaker 或 Hooke2 私有实现。仿真和实车共用同一套定位、规划、
控制和命令门控，仅在传感器输入与车辆执行边界处切换平台适配。

## 依赖与构建

默认复用实车仓 `pilot-auto.x1` 中与版本锁一致的源码：

```bash
cd /opt/ipg/carmaker/linux64-15.1/autoracer_hooke
./scripts/import_dependencies.sh --refresh
./scripts/install_rosdeps.sh
./scripts/build.sh
source ./scripts/ros_env.sh
```

需要重新从实车仓同步依赖时：

```bash
./scripts/import_dependencies.sh --refresh
```

只有明确需要联网重建第三方工作区时才使用：

```bash
./scripts/import_dependencies.sh --network
```

`dependencies/versions.lock.yaml` 固定上游提交；
`dependencies/patches/vehicle_cmd_gate_volatile_commands.patch` 是唯一产品所需上游补丁。

## 运行入口

- 仿真：双击桌面 `10km` 启动器，或运行
  `../SimProject_TianmenRace/run_pilot_localization_gui.sh`。该入口管理 CarMaker、Bridge、
  定位、规划、控制、运行管理、命令门控和 IPGMovie。
- 实车：

  ```bash
  ros2 launch autoracer_hooke2_bringup race.launch.py \
    localization_map_path:=/path/to/map \
    course_path:=/path/to/course
  ```

当前仿真资源为 `maps/urbanroad_route271_20260710` 和
`courses/urbanroad_route271_unlimited`。RViz 与硬件 smoke 脚本只用于诊断，不构成第二套运行栈。
