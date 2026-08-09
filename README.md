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
  vendor_ws/                       第三方 ROS 包的可再生 underlay（Git 忽略）
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

`dependencies/versions.lock.yaml` 固定上游提交，并以其中的 `patches` 列表作为
当前产品所需上游补丁的唯一清单。

Jetson Orin 的 RC 正式依赖 profile 只导入当前运行闭包中的 86 个包，不构建
Hooke2/Fixposition 或未使用的 Nebula 驱动：

```bash
cd /path/to/autoracer_hooke
./scripts/import_dependencies.sh --network-rc
./scripts/install_rosdeps.sh --rc
./scripts/build.sh --rc
source ./scripts/ros_env.sh
```

`build.sh --rc` 仍会先核对锁定的 86 包源码集合；不会在现场启动时重新导入、
安装依赖或构建。`install_rosdeps.sh --rc` 会尝试更新 rosdep 索引；网络不可达时，
只有现有本地缓存仍能正常读取才会继续。

RC G90 使用两个串口：COM1 的稳定 `by-id` 路径提供正式 NMEA，COM2 的
`/dev/autoracer_g90_com2` 接收差分。把
`src/platform/rc/autoracer_rc_bringup/udev/99-autoracer-rc-g90.rules`
安装到 `/etc/udev/rules.d/` 后重新加载 udev；COM2 规则只匹配当前唯一的
`1a86:7523` CH340，不绑定 USB 物理插口，也不修改系统设备权限。

NTRIP 凭据只保存在运行用户的
`~/.config/autoracer-rc/g90-ntrip.env`，文件权限必须严格为 `0600`。必填键为
`NTRIP_USERNAME`、`NTRIP_PASSWORD`、`NTRIP_HOST`、`NTRIP_PORT`、
`NTRIP_MOUNTPOINT` 和带 IANA 时区的 `NTRIP_EXPIRES_AT_LOCAL`。可保留已验证旧配置中的
`G90_COM2_DEVICE`、`G90_COM2_RTKLIB_PORT`、`G90_COM2_BAUD` 元数据；正式节点不会
执行或 source 这个文件。用户名、密码和认证串不会进入 ROS 参数、命令行或日志。

G90 差分 relay 由 RC 正式 launch 管理，不安装或启用 systemd/`str2str` 常驻服务。
菜单任务退出时，relay 与该任务的其他子进程一起被回收。

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

- RC 车：只使用统一入口，菜单 3/5 会随 G90 检查启动并停止差分 relay，菜单 6/7
  使用同一条项目内链路：

  ```bash
  ./scripts/autoracer_rc.sh
  ```

当前仿真资源为 `maps/urbanroad_route271_20260710` 和
`courses/urbanroad_route271_unlimited`。RViz 与硬件 smoke 脚本只用于诊断，不构成第二套运行栈。
