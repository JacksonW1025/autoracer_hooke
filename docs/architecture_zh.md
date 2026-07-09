# RC/Hooke 官方架构说明

用途：定义当前仓库的长期架构边界，并说明从旧自定义启动框架迁移到官方 Autoware profile 框架后，开发者应该怎样继续改代码。非用途：不记录一次性排查日志，不保存过渡计划。

## 平台原则

RC 不是一套新的自动驾驶栈。RC 是 Hooke/Autoware upper stack 的缩比验证平台，用来先验证 localization、official Autoware planning/control、gate 和 adapter 边界，再迁移到真实 Hooke 底盘。

长期边界：

- Hooke 和 RC 共享 official Autoware planning/control、地图、定位 topic、控制 topic 和 vehicle status 语义。
- CAN 与串口差异停留在 chassis/transport adapter 内；Hooke 用 CAN，RC 用 serial。
- 车辆尺寸、传感器外参、驱动参数、底盘 adapter 放在对应 vehicle/sensor profile 和 adapter 包里。
- 自研 planning/control 候选可以保留，但只能显式替换官方接口，不作为隐藏默认链路。
- `src/external/autoware` 是 pinned upstream 依赖；不在 `src/external/autoware` 里做隐形修改。

## 官方 Launch 结构

当前启动总控是官方 `autoware_launch/autoware.launch.xml`。本仓库按官方约定提供 vehicle profile 和 sensor-kit profile：

```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=autoracer_rc \
  sensor_model:=autoracer_rc_sensor_kit
```

官方 launch 会按名字寻找这些包：

| 参数 | 官方查找规则 | 当前 RC 包 | 责任 |
| --- | --- | --- | --- |
| `vehicle_model:=autoracer_rc` | `$(vehicle_model)_description` | `autoracer_rc_description` | 车辆尺寸、vehicle info、vehicle xacro。 |
| `vehicle_model:=autoracer_rc` | `$(vehicle_model)_launch` | `autoracer_rc_launch` | vehicle interface、safety gate、RViz profile。 |
| `sensor_model:=autoracer_rc_sensor_kit` | `$(sensor_model)_description` | `autoracer_rc_sensor_kit_description` | sensor kit xacro、LiDAR/IMU 外参。 |
| `sensor_model:=autoracer_rc_sensor_kit` | `$(sensor_model)_launch` | `autoracer_rc_sensor_kit_launch` | C32、Hipnuc IMU、Madgwick、pointcloud filter。 |

`scripts/rc/` 是操作者入口，只做薄封装、检查和运行时参数传递。`scripts/common/` 只能放不依赖具体车辆硬件事实的 helper。`scripts/hooke/` 在 Hooke profile 完成前只能 fail-fast。

## 给旧框架开发者的迁移说明

旧框架是本仓库自定义总控：操作者脚本把环境变量拼成 launch 参数，然后由仓库自己的总 launch 决定 sensing、localization、planning、control、safety 和 vehicle interface 怎么串起来。它的优点是直接，缺点是车辆事实、传感器事实、算法选择和现场参数容易混在一个启动层里。

新框架是官方 Autoware 总控：`autoware_launch` 负责装配 map、localization、planning、control、system、API、vehicle、sensing 等官方分层；本仓库只提供 RC 或 Hooke 的 official profile、少量 adapter 和操作脚本。

这次重构后的理解方式：

| 旧开发习惯 | 现在应该改哪里 |
| --- | --- |
| 在自定义总 launch 里加传感器节点 | 对应 sensor-kit launch，例如 `autoracer_rc_sensor_kit_launch`。 |
| 在自定义 description 或脚本里调车辆尺寸 | 对应 vehicle description，例如 `autoracer_rc_description/config/vehicle_info.param.yaml`。 |
| 在总 launch 里改 LiDAR/IMU 外参 | 对应 sensor-kit description，例如 `autoracer_rc_sensor_kit_description/config/sensor_kit_calibration.yaml`。 |
| 在总 launch 里接底盘节点 | 对应 vehicle launch，例如 `autoracer_rc_launch/launch/vehicle_interface.launch.xml`。 |
| 直接让本地 planning/control 成为默认 | 只能作为自研 planning/control 候选，在 profile/launch 层显式替换，并遵守官方 topic/message/frame contract。 |
| 在脚本里写死现场参数 | 脚本只传 `MAP_PATH`、`SERIAL_PORT`、是否开 RViz、是否开 vehicle interface、是否允许 drive commands。 |

以后怎么判断应该改哪里：

- 车身尺寸、轴距、最大转角：改 vehicle description。
- LiDAR/IMU 相对 `base_link` 的外参：改 sensor-kit description。
- C32 驱动参数、盲区、距离裁剪、输出 frame/topic：改 sensor-kit launch/config。
- IMU 串口、滤波、点云降采样：改 sensor-kit launch/config。
- RC 串口车辆接口、安全门、真实底盘串口：改 vehicle launch 或 `autoracer_vehicle_interface`。
- 地图路径、是否开 RViz、是否开车辆接口：用运行时环境变量，不写死进 profile。
- official planning/control 不满足需求：新增或启用自研模块，但保持 `/planning/*`、`/control/command/*`、`/vehicle/status/*` 合约。

## Profile 状态

| 平台 | `vehicle_model` | `sensor_model` | 状态 | 源码位置 | 脚本入口 | 当前责任 |
| --- | --- | --- | --- | --- | --- | --- |
| RC Ackermann | `autoracer_rc` | `autoracer_rc_sensor_kit` | active runtime baseline | `src/autoracer_rc_*` | `scripts/rc/` | 当前唯一可运行开发基线，继续用于 Orin 上的 RC 实车验证。 |
| Hooke | `autoracer_hooke` | `autoracer_hooke_sensor_kit` | `disabled_placeholder` / not runtime ready | `src/autoracer_hooke_*` | `scripts/hooke/` | 交给 Hooke 负责人补真实 vehicle、sensor、CAN adapter、sensing launch。 |

Hooke official profile 当前只保留占位目录。每个 `src/autoracer_hooke_*` 目录都必须保留 `COLCON_IGNORE`，直到真实 Hooke 配置完成。占位目录没有 `package.xml`，目的就是避免 `colcon`、`autoware_launch` 或操作者误认为 Hooke 已经可运行。

Hooke 负责人可以参考 vendored Hooke reference：

```text
src/hooke2_vehicle
src/hardware_drivers
src/wd_msgs
```

但不要让 RC wrapper 指向这些 reference package，也不要用 RC 参数伪装 Hooke profile 通过。

## Hooke 底盘架构图

```mermaid
flowchart LR
  subgraph H_Sensing["Hooke sensing / profile"]
    H_Lidar["Hesai/Pandar LiDAR\nnebula_hesai"]
    H_Fix["Fixposition\nGNSS/INS/IMU"]
    H_Map["Autoware map\nPCD + Lanelet2 + projector"]
  end

  subgraph H_Localization["Localization"]
    H_PC["/sensing/lidar/concatenated/pointcloud"]
    H_Filter["pointcloud_voxel_filter\n/sensing/lidar/filtered/pointcloud"]
    H_Seed["/localization/fixposition/seed_pose\nor GNSS pose seed"]
    H_NDT["autoware_ndt_scan_matcher"]
    H_Pose["/localization/pose_with_covariance"]
    H_State["/localization/kinematic_state"]
  end

  subgraph Upper["Shared official Autoware upper stack"]
    Planner["official Autoware planning\nroute + behavior + motion"]
    Traj["/planning/trajectory"]
    Ctrl["official Autoware control"]
    Cmd["/control/command/control_cmd"]
    Gate["autoracer_safety\ncommand_gate"]
    Safe["/autoracer/control/safe_control_cmd"]
  end

  subgraph H_Chassis["Hooke vehicle adapter"]
    HookeIf["hooke2_interface"]
    CAN["SocketCAN can0\n500000 bps"]
    HookeChassis["Hooke2 chassis"]
    H_Status["/vehicle/status/*"]
  end

  H_Lidar --> H_PC --> H_Filter --> H_NDT
  H_Fix --> H_Seed --> H_NDT
  H_Map --> H_NDT
  H_NDT --> H_Pose --> Planner --> Traj --> Ctrl --> Cmd --> Gate --> Safe
  H_NDT --> H_State --> Planner
  H_State --> Ctrl
  Safe --> HookeIf --> CAN --> HookeChassis
  HookeChassis --> CAN --> HookeIf --> H_Status
  H_Status --> H_State
  H_Status --> Ctrl
```

## RC 底盘架构图

```mermaid
flowchart LR
  subgraph R_Sensing["RC sensing / profile"]
    R_Lidar["Leishen C32\nlslidar_driver"]
    R_Imu["Hipnuc / N300 Pro\nimu_filter_madgwick"]
    R_SeedSrc["RViz / ROS /initialpose"]
    R_Map["Autoware map\nSuper-LIO PCD + Lanelet2 + projector"]
  end

  subgraph R_Localization["Localization"]
    R_PC["/sensing/lidar/concatenated/pointcloud"]
    R_Filter["pointcloud_voxel_filter\n/sensing/lidar/filtered/pointcloud"]
    R_ImuTopic["/sensing/imu/imu_data_raw\n/sensing/imu/imu_data"]
    R_Seed["manual_seed_pose_publisher\n/localization/fixposition/seed_pose"]
    R_NDT["autoware_ndt_scan_matcher"]
    R_Pose["/localization/pose_with_covariance"]
    R_State["/localization/kinematic_state"]
  end

  subgraph Upper2["Shared official Autoware upper stack"]
    Planner2["official Autoware planning\nroute + behavior + motion"]
    Traj2["/planning/trajectory"]
    Ctrl2["official Autoware control"]
    Cmd2["/control/command/control_cmd"]
    Gate2["autoracer_safety\ncommand_gate"]
    Safe2["/autoracer/control/safe_control_cmd"]
  end

  subgraph R_Chassis["RC vehicle adapter"]
    Serial["rc_serial_interface"]
    UART["UART 11-byte command frame"]
    STM32["STM32 firmware\nAckermann PWM"]
    R_Status["/vehicle/status/*"]
  end

  R_Lidar --> R_PC --> R_Filter --> R_NDT
  R_Imu --> R_ImuTopic
  R_ImuTopic -.-> R_NDT
  R_SeedSrc --> R_Seed --> R_NDT
  R_Map --> R_NDT
  R_NDT --> R_Pose --> Planner2 --> Traj2 --> Ctrl2 --> Cmd2 --> Gate2 --> Safe2
  R_NDT --> R_State --> Planner2
  R_State --> Ctrl2
  Safe2 --> Serial --> UART --> STM32
  STM32 --> Serial --> R_Status
  R_Status --> R_State
  R_Status --> Ctrl2
```

两张图的中间 `Shared official Autoware upper stack` 必须保持一致。RC 与 Hooke 的差异只允许出现在 sensing/profile、seed 来源、vehicle profile、map asset 生产流程和 vehicle adapter。

## 算法替换边界

当前默认上层链路是 official Autoware planning/control。以下本地包只作为自研 planning/control 候选存在：

```text
src/autoracer_planning
src/autoracer_control
```

启用自研候选的条件：

- 输入输出 topic、message、frame、单位遵守 official Autoware contract。
- 在 launch/profile 层显式替换，不通过脚本暗中切换。
- 保留 `autoracer_safety` gate 和 vehicle adapter 的清晰边界。
- Hooke/RC 共同维护接口，不给 RC 单独分叉 upper stack。

## 验收顺序

1. Sensing/TF：点云、IMU、静态 TF、车辆状态 topic 可用。
2. Localization：地图加载、manual seed、NDT pose、`kinematic_state` 可用。
3. Planning：官方 planning route/goal 后生成 `/planning/trajectory`。
4. Control：官方 control 输出 `/control/command/control_cmd`，方向、速度、转角合理。
5. Gate：禁用时输出 stop，使能后输出 `/autoracer/control/safe_control_cmd` 和必要 support command。
6. Vehicle adapter：串口/CAN 命令和反馈符号、单位、频率正确。
7. Low-speed drive：低速闭环验证，形成 bag/log 和问题清单。
