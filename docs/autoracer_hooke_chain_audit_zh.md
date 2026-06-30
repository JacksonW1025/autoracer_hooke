# autoracer_hooke 链路审计与 RC 验证迁移基线

这份文档是后续迁移工作的中文基线。它说明当前 `autoracer_hooke`
工作区的链路如何组织、上层算法消费哪些数据、RC 相关改动应该被限制在哪些边界内。

核心目标不是给 RC 车重新设计一套单独的自动驾驶栈，而是让 RC 车尽量满足
Hooke/Autoware 链路已经使用的接口契约，从而在真车运行前验证定位、规划和控制算法。

## 1. 基本原则

- `ackermann-nav2-legacy` 只能作为 RC 硬件事实参考，不能作为 Autoware
  定位、规划、控制链路的设计来源。
- 上层算法要保持可迁移：LiDAR 地图定位、Lanelet 规划、pure pursuit 控制、
  command gate 都不应该变成 RC 专用版本。
- 底盘差异必须放在 vehicle adapter 边界内。Hooke 是 CAN，RC 是
  UART/STM32，但两者都应该对上层暴露 Autoware vehicle status，并消费
  Autoware control command。
- 地图契约保持 Autoware 原链路：地图目录应包含 `pointcloud_map.pcd`、
  `pointcloud_map_metadata.yaml`、`lanelet2_map.osm`、`map_projector_info.yaml`。
- RC 没有 Fixposition，也没有 ZED。这是定位输入/初始位姿来源缺口，不是回退到
  AMCL、slam_toolbox 或其他 Nav2 行为的理由。
- 开发机只用于源码修改、静态检查和不依赖硬件的单元测试。雷达、串口、车身反馈、
  NDT 实时性和整车启动必须在当前车载计算平台上验证；目标链路仍按 AGX Orin
  车载计算平台保持兼容，临时运行主机不构成新的架构边界。
- 临时主机 IP、SSH 密码、本机串口名等运行现场信息不进入仓库配置；通过启动参数或
  环境变量传入。

依据文件：

- `README.md`
- `docs/minimal_stack.md`
- `docs/sensing_feedback_topics.md`
- `src/autoracer_bringup/launch/track.launch.py`
- `src/autoracer_bringup/launch/sensing.launch.py`
- `src/autoracer_bringup/launch/localization.launch.py`
- `src/autoracer_planning/launch/planning.launch.py`
- `src/autoracer_control/launch/control.launch.py`
- `src/autoracer_safety/launch/safety.launch.py`
- `src/autoracer_bringup/launch/vehicle.launch.py`

## 2. 迁移纠偏矩阵

这些是迁移护栏。它们不是零散注意事项，而是防止 RC 验证逐渐偏离
Hooke/Autoware 原链路的约束。

| 领域 | 错误捷径 | 正确边界 | 具体后续动作 |
| --- | --- | --- | --- |
| 参考仓库 | 把 Nav2 legacy 的行为当成 Autoware 设计来源。 | Nav2 legacy 只用于确认 RC 硬件事实和历史接线。 | 不把 AMCL、slam_toolbox、`/scan`、`/wheel_odom` 或自定义控制 topic 复制进上层。 |
| 地图契约 | 重新讨论地图来源，或者切到 Nav2 地图。 | 保持 Autoware 地图目录契约：PCD、Lanelet2、projector info、metadata。 | RC 定位验证也要基于同一类 PCD/Lanelet2 地图资产。 |
| 定位 seed | 因为没有 Fixposition/ZED 就更换定位算法。 | 保留基于 PCD 地图的 NDT 定位，只替换 seed/initial pose 来源。 | 明确 RC 上谁发布 `/localization/ndt_initial_pose` 或等价语义的 seed pose。 |
| LiDAR 输入 | 因为旧 RC 栈用 `/scan`，就改成 2D scan 定位。 | NDT 输入必须是 `/sensing/lidar/concatenated/pointcloud` 上的 `PointCloud2`。 | 将 RC LiDAR driver/remap/frame 规范化到 Autoware 点云契约。 |
| 车辆几何 | 只改某一个 wheelbase 或 steering 参数。 | 把几何参数作为一个统一 RC vehicle profile，所有运动学消费者共用。 | 将 0.600 m 轴距这组参数同步到 TF/URDF、predictor、controller、adapter、firmware。 |
| 底盘反馈 | 让旧 `/wheel_odom`、`/chassis_state`、`/ackermann_cmd` 泄漏到上层。 | 底盘 adapter 只向上发布 Autoware `/vehicle/status/*`。 | 验证 RC UART telemetry 能正确给出 velocity、steering、gear、control mode。 |
| 车辆传输 | 让 Hooke CAN 与 RC UART 的差异影响定位/规划/控制。 | CAN/UART 差异只能存在于 vehicle adapter 内部。 | RC serial 和 Hooke2 CAN 都必须满足同一 command/status 边界。 |
| 固件行为 | 围绕 STM32 deadband 或 PWM 细节改上层算法。 | deadband、最小速度、RC 接管、PWM 映射都是 adapter/firmware 事实。 | 在 adapter 边界测试低速命令和反馈行为。 |
| safety gate 与倒车 | 通过改 gate 的一个速度 clamp 来解决倒车或速度语义。 | 倒车要拆成 trajectory intent、control output、gear command、adapter 行为、底盘限制。 | 前向闭环验证和未来倒车设计分开处理。 |
| 验证顺序 | 看到单个 topic 有数据就直接使能实车。 | 每一层先证明自己，再让下一层消费。 | sensing/TF -> NDT -> route -> raw control -> gate -> physical drive。 |

safety gate/倒车只是一个已经暴露出来的跨层误改案例，不是唯一核心问题。
同样的规则适用于矩阵里的其他行：不能通过让无关上层算法 RC 专用化来掩盖某个接口契约缺口。

## 3. 端到端运行链路

原链路可以概括为：

```text
传感器 + 车辆反馈
  -> 标准化 Autoware sensor/status topics
  -> PCD/Lanelet2 地图加载
  -> GNSS/Fixposition seed path
  -> NDT 基于 PCD 地图匹配定位
  -> /localization/pose_with_covariance
  -> Lanelet 中心线 route 和 trajectory
  -> pure pursuit + 纵向控制
  -> safety command gate
  -> /control/command/control_cmd + support commands
  -> vehicle adapter
  -> 底盘传输
```

`track.launch.py` 在一个顶层 launch 中启动这些模块：

| 层级 | Launch 入口 | 主要职责 |
| --- | --- | --- |
| Static TF | `autoracer_description/launch/static_tf.launch.py` | 发布 `base_link` 到 LiDAR/GNSS/IMU frame 的静态 TF。 |
| Sensing | `autoracer_bringup/launch/sensing.launch.py` | 启动 Hesai LiDAR 和 Fixposition，并把 vehicle speed bridge 给 Fixposition。 |
| Localization | `autoracer_bringup/launch/localization.launch.py` | 加载地图，生成 GNSS seed，预测 NDT initial pose，运行 NDT，发布 map-to-base pose。 |
| Planning | `autoracer_planning/launch/planning.launch.py` | 加载 Lanelet2 map，收到 `/goal_pose` 后发布 route/trajectory。 |
| Control | `autoracer_control/launch/control.launch.py` | 用 pure pursuit 跟踪 `/planning/trajectory`，并做纵向 P 控制。 |
| Safety | `autoracer_safety/launch/safety.launch.py` | 将 raw control gate 到 Autoware vehicle command surface。 |
| Vehicle adapter | `autoracer_bringup/launch/vehicle.launch.py` 或 Hooke2 launch | 在 Autoware command/status 和实际底盘传输之间转换。 |

关键边界：

- 上层算法链路到 `/control/command/control_cmd` 和 support commands 为止。
- 真车 Hooke adapter 是 `hooke2_interface`：它消费同一组 Autoware control topic，
  并从 CAN 反馈发布 `/vehicle/status/*`。
- 当前 minimal launch 使用 `rc_serial_interface`。这应该被视为 vehicle adapter
  的一个实现，而不是上层算法应该变成 RC 专用的证据。

## 4. Topic 与数据契约

下面这些契约决定迁移是否成立。只要 RC 能以正确 frame、时序、单位产出同样契约，
上层算法就可以尽量保持不变。

| Topic | Type | Producer | Consumer | 契约角色 |
| --- | --- | --- | --- | --- |
| `/sensing/lidar/concatenated/pointcloud` | `sensor_msgs/msg/PointCloud2` | `nebula_hesai` | NDT scan matcher | 地图定位用 LiDAR 输入。frame 必须通过 TF 接到 `base_link`。 |
| `/fixposition/fix` | `sensor_msgs/msg/NavSatFix` | Fixposition driver | `autoware_gnss_poser` | Hooke 链路中的 GNSS seed 来源。RC 没有这个设备。 |
| `/fixposition/autoware_orientation` | `autoware_sensing_msgs/msg/GnssInsOrientationStamped` | Fixposition driver | `autoware_gnss_poser` | GNSS seed 配套方向信息。RC 没有这个设备。 |
| `/fixposition/fpa/odomstatus` | `fixposition_driver_msgs/msg/FpaOdomstatus` | Fixposition driver | `fixposition_seed_filter` | 可选 seed 质量/状态检查；当前 launch `require_status=false`。 |
| `/fixposition/speed` | `fixposition_driver_msgs/msg/Speed` | `velocity_to_fixposition_speed` | Fixposition driver | 给 Fixposition 的轮速反馈；只有 Fixposition 存在时才相关。 |
| `/sensing/gnss/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `autoware_gnss_poser` | `fixposition_seed_filter` | map frame GNSS pose candidate。RC 无 Fixposition 时需要替代 seed path。 |
| `/localization/fixposition/seed_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `fixposition_seed_filter` | `ndt_initial_pose_predictor`、NDT regularization input | 过滤后的 initial/recovery seed。名字带 Fixposition，但语义是 localization seed。 |
| `/localization/ndt_initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `ndt_initial_pose_predictor` | NDT scan matcher、startup helper | NDT 初始位姿流。RC 必须补齐的关键契约。 |
| `/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | NDT scan matcher | planning、control、safety、pose TF、initial-pose predictor | 上层使用的权威定位 pose。 |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | NDT scan matcher | diagnostics/RViz | 无 covariance 的 pose 输出，不是主要 planning/control 契约。 |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | Hooke2 CAN adapter 或 RC serial adapter | speed bridge、initial-pose predictor、pure pursuit | 车辆纵向速度和 heading rate，必须使用 Autoware 单位。 |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | Hooke2 CAN adapter 或 RC serial adapter | initial-pose predictor | 前轮转角反馈。 |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | Hooke2 CAN adapter 或 RC serial adapter | vehicle/system consumers | 底盘挡位状态，对倒车语义和 adapter 正确性重要。 |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | Hooke2 CAN adapter 或 RC serial adapter | system/operator consumers | 手动/自动 readiness 状态。 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | operator/RViz/test tool | `lanelet_route_planner` | 任务目标点。 |
| `/planning/mission_path` | `nav_msgs/msg/Path` | `lanelet_route_planner` | RViz/debug | 方便查看的路径。 |
| `/planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | `lanelet_route_planner` | `pure_pursuit_controller` | 控制用 trajectory，包含目标速度。 |
| `/autoracer/control/raw_control_cmd` | `autoware_control_msgs/msg/Control` | `pure_pursuit_controller` | `command_gate` | 未经过 gate 的控制输出。 |
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | `command_gate` | Hooke2 CAN adapter 或 RC serial adapter | 给底盘 adapter 的最终 Autoware control command。 |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | `command_gate` | Hooke2 CAN adapter 或 RC serial adapter | 挡位请求。当前 gate safe 时发 DRIVE，unsafe 时发 NEUTRAL。 |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | `command_gate` | Hooke2 CAN adapter | 安全/辅助命令。 |
| `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | `command_gate` | Hooke2 CAN adapter | 辅助命令。 |

## 5. 传感器与 Frame 链路

### LiDAR

LiDAR 链路很明确：

```text
Hesai/Pandar UDP packets
  -> nebula_hesai decoder
  -> /sensing/lidar/concatenated/pointcloud
  -> autoware_ndt_scan_matcher
```

默认配置是 `src/autoracer_bringup/config/hooke2/lidar_top.param.yaml`。
`track.launch.py` 传入的默认 sensor model 是 `Pandar40P`。

RC 验证要求：

- RC LiDAR driver 必须发布同样的 `PointCloud2` 契约，或者 remap 到该契约。
- frame 必须和 TF tree 一致。如果 RC LiDAR frame 不是 `lidar_top`，要么在
  adapter/launch 中标准化 frame 名，要么一致更新 RC TF profile。
- 不能用 2D `/scan` 栈替代 NDT。如果 RC LiDAR 用于验证，就必须满足点云定位契约。

### Static TF 与车辆几何

当前 Hooke 外参在 `src/autoracer_description/config/hooke2_sensor_extrinsics.yaml`，
发布：

```text
base_link -> lidar_top_base_link -> lidar_top
base_link -> gnss_base_link
base_link -> imu_link
```

当前 Hooke 车辆尺寸在 `src/autoracer_bringup/config/hooke2/vehicle_info.param.yaml`：

```text
wheel_base: 1.9
wheel_tread: 1.55
wheel_radius: 0.313
max_steer_angle: 0.488
```

RC 验证要求：

- RC vehicle profile 必须整体替换这些几何和外参。
- 当前已确认的 RC 固件默认值为：

```text
wheelbase: 0.600 m
track width: 0.470 m
wheel radius: 0.115 m
wheel diameter: 0.230 m
max steering: 0.262 rad
```

- 这些值必须一致传入 localization prediction、controller、vehicle adapter
  limits、URDF/TF。只改一部分会制造互相矛盾的运动学。

## 6. 定位链路

定位链路是这次迁移的关键。

### 地图加载

`localization.launch.py` 加载：

```text
map_projector_info.yaml
lanelet2_map.osm
pointcloud_map.pcd
pointcloud_map_metadata.yaml
```

pointcloud map loader 发布/提供：

```text
/map/pointcloud_map
/map/get_partial_pointcloud_map
/map/get_differential_pointcloud_map
/map/get_selected_pointcloud_map
```

NDT 使用 `client_map_loader=/map/get_differential_pointcloud_map` 配置的
differential pointcloud map service。

所以地图来源并不模糊。RC 验证也应使用同一套 Autoware map directory contract。

### Fixposition Seed Path

当前 Hooke seed path：

```text
/fixposition/fix
/fixposition/autoware_orientation
  -> autoware_gnss_poser
  -> /sensing/gnss/pose_with_covariance
  -> fixposition_seed_filter
  -> /localization/fixposition/seed_pose
```

`fixposition_seed_filter` 检查：

- frame 是 `map`
- pose 新鲜度
- pose/quaternion 是有限值
- covariance 阈值
- jump 阈值
- 可选 Fixposition status

launch 当前设置：

```text
require_status: false
use_status_when_available: true
```

所以 `/fixposition/fpa/odomstatus` 可用于增强过滤，但不是当前 launch 的强制依赖。

### NDT Initial Pose Predictor

当前 NDT initial pose path：

```text
/localization/fixposition/seed_pose
/localization/pose_with_covariance
/vehicle/status/velocity_status
/vehicle/status/steering_status
  -> ndt_initial_pose_predictor
  -> /localization/ndt_initial_pose
```

行为：

- 如果没有 state，seed pose 初始化 predictor。
- 如果 NDT lost，seed pose 可重新初始化。
- 一旦 NDT 开始发布，NDT pose 会校正 predictor state。
- 两次校正之间，用车辆速度、yaw rate/steering 推进 pose。

这意味着 RC 不需要伪装成 Fixposition 设备。RC 需要提供当前名为
`/localization/fixposition/seed_pose` 的语义契约，或者用 Autoware 兼容方式直接提供
`/localization/ndt_initial_pose`。

### NDT Scan Matcher

当前 NDT 输入：

```text
pointcloud: /sensing/lidar/concatenated/pointcloud
initial pose: /localization/ndt_initial_pose
regularization pose: /localization/fixposition/seed_pose
map service: /map/get_differential_pointcloud_map
trigger service: /localization/ndt_trigger
```

关键参数：

```text
ndt.regularization.enable: false
```

所以当前 launch 虽然接了 regularization topic，但配置禁用了 NDT regularization。
Fixposition 不是 NDT scan matching 算法本身的硬依赖，它是当前 startup/recovery seed pose 的来源。

`ndt_startup_helper` 等待：

- 足够数量的新鲜 `/localization/ndt_initial_pose`
- map service ready
- NDT trigger service ready

然后调用 `/localization/ndt_trigger`，后续在 NDT pose stale 或质量诊断差时重新触发。

### RC 定位缺口

RC 没有 Fixposition，也没有 ZED。真正缺的是：

```text
稳定的 map-frame NDT initial/recovery pose
```

后续可以评估的方向：

- 初期验证使用人工/operator initial pose，如果封闭场地流程允许。
- 做一个轻量 RC seed adapter，从已知起点或外部测量发布同样的 seed-pose 契约。
- 如果 Autoware NDT startup tooling 可以满足 `/localization/ndt_initial_pose` 契约，则直接复用。

不应作为默认方案：

- 替换为 Nav2 AMCL/slam_toolbox。
- 把 `/scan` 定位当成 Hooke NDT 链路的等价替代。
- 通过修改 planning/control 来掩盖缺失 localization seed。

## 7. 规划链路

`lanelet_route_planner` 消费：

```text
/localization/pose_with_covariance
/goal_pose
lanelet2_map.osm
map_projector_info.yaml
```

它发布：

```text
/planning/mission_path
/planning/trajectory
/planning/route_marker
```

planner 行为：

- 加载 Lanelet2。
- 根据当前 pose 和 goal 查找 nearest lanelet。
- 计算 route/shortest path。
- 采样 lanelet centerline points。
- 给 trajectory points 赋值 `speed_limit_mps`。
- 最后一个点速度置零。

RC 验证要求：

- 不替换成 Nav2 planner。
- 保持同样 Lanelet map 和 route topic 契约。
- RC 专用速度限制可以是 launch/profile 参数，不应 fork 算法。

## 8. 控制链路

`pure_pursuit_controller` 消费：

```text
/planning/trajectory
/localization/pose_with_covariance
/vehicle/status/velocity_status
```

它发布：

```text
/autoracer/control/raw_control_cmd
```

controller 行为：

- 根据 nearest point 和 lookahead 找 target point。
- 根据 curvature 和 `wheel_base_m` 计算 Ackermann tire steering angle。
- 用 `max_steer_rad` clamp steering。
- 从 trajectory 取 target speed。
- 将 target speed clamp 到 `[0.0, max_speed_mps]`。
- 用纵向 P control 计算 acceleration。
- pose 或 trajectory 不可用时发布 stop。

P0 已把 `wheel_base_m` 和 `max_steer_rad` 从顶层 launch 接到
`control.launch.py`，使 pure pursuit、NDT initial-pose predictor、safety gate
和 RC serial adapter 可以共用同一组 RC 车辆几何。单独启动
`autoracer_control/launch/control.launch.py` 时仍保留 Hooke 默认值。

## 9. Safety 与 Command Gate

`command_gate` 消费：

```text
/autoracer/control/raw_control_cmd
/localization/pose_with_covariance
```

它发布：

```text
/control/command/control_cmd
/control/command/gear_cmd
/control/command/hazard_lights_cmd
/control/command/turn_indicators_cmd
/autoracer/safety/state
```

安全条件：

- `enable_drive_commands` 必须为 true。
- raw command 必须新鲜。
- localization pose 必须新鲜。

unsafe 时，它发布 stop command，以及 NEUTRAL 和 hazard lights 等辅助命令。

safe 时，它限制：

```text
velocity: [0.0, max_speed_mps]
acceleration: [max_decel_mps2, max_accel_mps2]
steering_tire_angle: [-max_steer_rad, max_steer_rad]
steering rate: limited by max_steer_rate_radps
```

迁移说明：

- gate clamp 只是一个跨层迁移风险，不是唯一风险。
- 不要通过把 gate velocity range 改成 `[-3.0, 3.0]` 来解决倒车。
- 倒车语义涉及 planner/control intent、`GearCommand`、adapter behavior、chassis limits，
  需要单独设计。
- 当前前向封闭场地验证下，planner 和 pure pursuit 链路是 forward-speed oriented。

## 10. Vehicle Adapter 边界

### Hooke2 CAN Adapter

vendored Hooke2 interface 消费 Autoware commands：

```text
/control/command/control_cmd
/control/command/gear_cmd
/control/command/turn_indicators_cmd
/control/command/hazard_lights_cmd
/control/command/actuation_cmd
/control/command/emergency_cmd
```

它发布 Autoware vehicle status：

```text
/vehicle/status/control_mode
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/gear_status
/vehicle/status/turn_indicators_status
/vehicle/status/hazard_lights_status
/vehicle/status/actuation_status
/vehicle/status/steering_wheel_status
/vehicle/status/door_status
/vehicle/status/battery_charge
```

它还负责和 raw `/hooke2/*` CAN-level topics 互相转换。这些 raw topics 应该停留在
adapter/debug 边界内。

### RC UART Adapter

当前 Python RC adapter 消费：

```text
/control/command/control_cmd
/control/command/gear_cmd
```

它发布：

```text
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/gear_status
/vehicle/status/control_mode
```

它发送 11-byte UART command frame：

```text
0x7B cmd1 cmd2 vx vy wz bcc 0x7D
```

它解析 24-byte telemetry frame：

```text
0x7B flag vx vy wz reserved... battery bcc 0x7D
```

迁移时这个 adapter 必须停留在 Autoware 边界下面。上层不应该消费旧 RC topic，
例如 `/wheel_odom`、`/chassis_state` 或自定义 `/ackermann_cmd`。

### RC 固件事实

`RCCar-Firmware` 当前已确认 profile：

```text
APP_ORIN_ACKERMANN_WHEELBASE_MM: 600
APP_ORIN_ACKERMANN_TRACK_WIDTH_MM: 470
APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM: 115
APP_ORIN_ACKERMANN_MAX_STEERING_MRAD: 262
APP_ORIN_ACKERMANN_MIN_VX_MMPS: 50
APP_ORIN_VX_DEADBAND_MMPS: 50
APP_ORIN_VX_FORWARD_CAP_MMPS: 3000
APP_ORIN_VX_REVERSE_CAP_MMPS: 3000
```

与 adapter 相关的固件行为：

- UART parser 接受 `vx`、`vy`、`wz` 和 stop flag。
- Ackermann 模式下 `vy` 被忽略。
- 小于 neutral threshold 的速度会被转换为 neutral output。
- steering command mapping 需要足够的 `|vx|` 才能从 yaw rate 计算 Ackermann steering。
- telemetry 用 Hall speed 给出 `vx`，并根据 steering PWM 和 measured speed 估算 `wz`。

这些事实用于 adapter 验证。除非 Autoware 契约确实无法满足，否则不应该倒逼上层算法变化。

## 11. RC 验证映射

| Hooke/Autoware 契约 | RC 来源或动作 | 缺口类型 | 状态 |
| --- | --- | --- | --- |
| PCD/Lanelet2 map directory | 使用与 Hooke 链路相同的 map contract | Map contract | 必须保持 |
| `/sensing/lidar/concatenated/pointcloud` | RC LiDAR driver/remap/profile | Sensor input | 必须实现/验证 |
| `base_link -> lidar_top` TF | RC URDF/static TF profile | Sensor/TF profile | 必须替换 Hooke 外参 |
| `/fixposition/fix` 和 `/fixposition/autoware_orientation` | RC 无该设备 | Missing sensor source | 不能伪装成设备 |
| `/sensing/gnss/pose_with_covariance` | 只有保留 GNSS seed path 时才需要 | Localization seed | 除非重设 seed path，否则是缺口 |
| `/localization/fixposition/seed_pose` | 可以由其他来源发布同等语义 seed pose | Localization seed | 必须重新设计，不能回退 Nav2 |
| `/localization/ndt_initial_pose` | Initial-pose predictor 或 direct seed publisher | Localization startup | 必须为 NDT startup 提供 |
| `/localization/pose_with_covariance` | NDT 输出 | Upper-layer localization contract | 必须作为 planning/control pose source |
| `/vehicle/status/velocity_status` | RC UART telemetry 经 adapter 发布 | Vehicle status | 必须验证单位、频率、符号、deadband 影响 |
| `/vehicle/status/steering_status` | RC UART telemetry/last steering command 经 adapter 发布 | Vehicle status | 必须验证 steering 符号和物理标定 |
| `/vehicle/status/gear_status` 和 `/vehicle/status/control_mode` | RC adapter 发布 Autoware status | Vehicle status | 使能实车前必须验证 |
| `/control/command/control_cmd` | safety gate 输出，RC adapter 消费 | Command boundary | 必须保持 |
| `/control/command/gear_cmd` | gate 输出，RC adapter 消费 | Command boundary/design decision | 必须保持，尤其倒车设计前 |
| controller wheelbase/max steer | RC vehicle profile | Parameter consistency | 必须一致接入 |
| localization predictor wheelbase | RC vehicle profile | Parameter consistency | 必须一致接入 |
| firmware velocity deadband | adapter/firmware fact | Adapter behavior | 记录并测试，不能泄漏到上层 |
| RC override/guard behavior | STM32 firmware + serial adapter state | Adapter behavior | 作为底盘安全行为测试，不作为 planning logic |

## 12. P0 当前实现

P0 只解决 RC 验证链路的硬件入口和参数一致性，不引入 RC 专用定位、规划或控制算法。

已实施：

- 新增 `autoracer_description/config/rc_sensor_extrinsics.yaml`，发布
  `base_link -> lidar_top`，外参为 `x=0.24, y=0.0, z=0.39, yaw=-1.5708`。
- 新增 `track_rc_p0.launch.py` 作为薄 profile wrapper。它复用
  `track.launch.py`，只填入 RC 参数，不复制算法链路。
- `track_rc_p0.launch.py` 使用雷神 C32 driver，保持旧 C32 网络参数：
  `device_ip=192.168.1.200`、`msop_port=2368`、`difop_port=2369`。
  C32 点云直接发布到 `/sensing/lidar/concatenated/pointcloud`，frame 为
  `lidar_top`。
- `track_rc_p0.launch.py` 关闭 Fixposition driver 和 Fixposition seed path，
  打开人工 seed publisher。
- `manual_seed_pose_publisher` 订阅 RViz/ROS `/initialpose`，把人工标定的
  map-frame pose 发布到 `/localization/fixposition/seed_pose`，再由原有
  `ndt_initial_pose_predictor` 输出 `/localization/ndt_initial_pose`。RC profile
  默认 `require_input_pose=true`，不会发布 `0,0,0` 假 seed。
- `track.launch.py` 现在把 `wheel_base_m`、`max_steer_rad` 接给
  localization、control、safety、vehicle adapter。
- RC P0 车辆几何采用固件确认的 `wheel_base_m=0.6`、`max_steer_rad=0.262`；
  固件侧还有 `track_width=0.470`、`wheel_radius=0.115`、`wheel_diameter=0.230`。
- controller、command gate、RC serial adapter 已按 Autoware signed velocity/gear
  契约修正：不再把速度硬夹到 `[0, max]`；gate 根据 signed velocity 发布
  `DRIVE` 或 `REVERSE`；serial adapter 对 gear/速度方向矛盾做最后 stop 保护。
- 固件前/后向速度 cap 已同步到 `3.0 m/s`，低速 deadband 保留 `0.05 m/s`。

仍未在 P0 解决：

- 地图资产还没有导入。运行完整 localization/planning 前，`MAP_PATH` 或
  `map_path` 必须指向包含 PCD/Lanelet2/projector/metadata 的 Autoware map 目录。
- IMU 暂未接入上层消费者。当前 P0 NDT 链路不依赖 IMU。
- 完整倒车规划/行为还没有作为 P0 验证目标；当前只修正 control/gate/adapter
  契约，避免硬编码 forward-only。
- 固件低速死区没有被绕过。当前固件事实是 `APP_ORIN_ACKERMANN_MIN_VX_MMPS=50`
  和 `APP_ORIN_VX_DEADBAND_MMPS=50`，前/后向固件 cap 为 3.0 m/s。

P0 运行入口示例：

```bash
ros2 launch autoracer_bringup track_rc_p0.launch.py \
  map_path:=/path/to/autoware_map
```

启动后在 RViz/ROS 中发布 `/initialpose`。该 pose 是人工标定的等价 seed pose；
不要把默认零值当作真实定位初值。

## 13. 后续实施顺序

下一阶段按这个顺序做：

1. 继续实车校准 RC vehicle profile：
   - C32 外参先用旧 Nav2 值，后续实车标定。
   - IMU 可接入但 P0 不作为定位消费者。
   - launch wiring 保证 controller、localization prediction、vehicle adapter 读取同一组几何。

2. 验证 RC sensing：
   - 确保 C32 产生 `/sensing/lidar/concatenated/pointcloud`。
   - 验证 frame ID 和 TF。
   - 记录 IMU 可用性，但只有明确引入消费者时才接入链路。

3. 验证 NDT startup seed：
   - 不使用 Nav2 AMCL/slam_toolbox。
   - 使用 `/initialpose -> /localization/fixposition/seed_pose -> /localization/ndt_initial_pose`。
   - 在启用 planning/control 前，先验证 NDT 能基于 PCD map 正常工作。

4. 验证 vehicle adapter：
   - UART command/telemetry frame 兼容性。
   - speed sign、steering sign、yaw-rate reconstruction。
   - low-speed deadband 行为。
   - Autoware `/vehicle/status/*` 发布频率和新鲜度。

5. 最后运行上层栈：
   - 先 localization。
   - 再 planning route generation。
   - 再 raw control output。
   - 再 safety-gated output。
   - dry-run topic 检查干净后才进行实车物理驱动。

## 14. 暂时不要改什么

- 不要新增 RC 专用 planning/control 算法。
- 不要把 Nav2 localization 重新作为默认方案。
- 不要用 2D `/scan` localization 替代 pointcloud NDT 验证。
- 不要让 `/wheel_odom`、`/chassis_state` 或自定义 `/ackermann_cmd` 被上层 Autoware 消费。
- 不要只通过改 final gate 来修改速度限制或倒车支持。
- 不要把缺 Fixposition 当成地图来源问题。
- 不要相信局部车辆参数更新；几何必须在 TF、localization prediction、control、adapter、firmware 中一致。
- 不要把 firmware deadband 或 UART PWM 细节当成 fork 上层定位、规划、控制算法的理由。

## 15. 验收清单

开始下一阶段代码迁移前，必须满足：

- 每个上层输入 topic 都有已知 RC producer，或者明确标注为缺口。
- 所有 adapter-only topic 都停留在 vehicle/sensor 边界内。
- 纠偏矩阵中没有任何一行被算法 fork 绕过去。
- 地图契约仍然是 Autoware PCD/Lanelet2 契约。
- NDT startup 使用明确的非 Nav2 seed strategy：RViz/ROS `/initialpose`。
- RC 车辆几何作为一个 coherent profile 存在，并接入每个消费者。
- firmware low-speed/deadband 行为在 adapter 边界被测试。
- `autoracer_hooke` 仍然可以被理解为一套 Autoware-style stack，而不是无关 RC legacy 行为的混合体。
