# Autoracer Race Controller Upgrade Bench Plan

> 状态：已吸收冷启动 review 的可执行版控制器升级测试计划。本文只覆盖 **Stage B 结束前** 能完成的工作：实现升级后的 race controller ROS-only bench，并用机器可判定的 PASS / FAIL 闭环。旧 `pure_pursuit_controller` 只作为 baseline，不是本轮主测对象。

原则：**先升级控制器，再隔离测试；主测 race controller；不启动 CarMaker；不等待 Stage B；不污染当前 EKF / CarMaker 仿真；synthetic bench 不等于车辆性能或真车标定。Stage B 结束后的 CarMaker / bridge / 闭环验证另行设计，不放入本文交付范围。**

---

## 1. 当前目标

当前项目已有旧控制链路：

```text
/planning/trajectory
  -> pure_pursuit_controller
  -> /autoracer/control/raw_control_cmd
  -> command_gate
  -> /control/command/control_cmd
```

这条链路是 baseline。本文的任务不是继续测试它，而是新增并测试升级后的 race controller：

```text
synthetic trajectory / odometry / steering / acceleration / operation mode / pose / tf
  -> race controller
       package:    autoware_trajectory_follower_node
       executable: controller_node_exe
       lateral_controller_mode = mpc
       longitudinal_controller_mode = pid
  -> /control_bench/autoracer/control/raw_control_cmd
  -> command_gate
  -> /control_bench/control/command/control_cmd
  -> race_bench_monitor / runtime_summary.json
```

本轮只回答：

1. `controller_node_exe` 能否在本项目中作为 race controller 启动；
2. MPC/PID、trajectory follower、vehicle info、nearest search 和 Autoracer overlay 参数能否完整加载；
3. synthetic 输入齐全时，race controller 是否发布 raw control；
4. raw control 是否 remap 到 bench topic，并经过 `command_gate` 输出 final control；
5. straight / left / right trajectory 下 steering 符号是否可解释；
6. current speed 低于 / 高于 trajectory target speed 时纵向输出方向是否可解释；
7. 缺关键输入或 gate timeout 时是否 fail closed；
8. bench 是否完全隔离于当前 EKF / CarMaker 仿真。

本轮不回答：

- Stage B planner 是否通过；
- CarMaker closed-loop 是否通过；
- race controller 是否优于 pure pursuit；
- MPC/PID 是否完成 Hooke2 真车标定；
- 真车是否可驾驶；
- race performance 是否达标。

---

## 2. 不可变更的设计决策

| 决策 | 当前结论 |
|---|---|
| 本轮主测 controller | `autoware_trajectory_follower_node/controller_node_exe` |
| 横向模式 | `lateral_controller_mode = mpc` |
| 纵向模式 | `longitudinal_controller_mode = pid` |
| 旧 controller | `pure_pursuit_controller` 只作为 baseline / 后续对比，不作为本轮主测对象 |
| race output | `~/output/control_cmd -> raw_control_topic`，bench 中为 `/control_bench/autoracer/control/raw_control_cmd` |
| final output | 只能由 `command_gate` 发布 `/control_bench/control/command/control_cmd` |
| CarMaker | 本轮不启动、不读取、不修改 |
| 隔离 | 独立 `ROS_DOMAIN_ID` + `/control_bench` explicit remap |
| external Autoware | 不修改 `src/external/autoware/**` |

如果实现时发现需要偏离上述决策，必须停止并重新确认。

---

## 3. 已核实源码事实

### 3.1 race controller 输入输出

`controller_node_exe` 的源码输入在：

```text
src/external/autoware/universe/control/autoware_trajectory_follower_node/include/autoware/trajectory_follower_node/controller_node.hpp
```

实际订阅 topic：

| Controller private topic | Type | Bench topic |
|---|---|---|
| `~/input/reference_trajectory` | `autoware_planning_msgs/msg/Trajectory` | `/control_bench/planning/trajectory` |
| `~/input/current_odometry` | `nav_msgs/msg/Odometry` | `/control_bench/localization/kinematic_state` |
| `~/input/current_steering` | `autoware_vehicle_msgs/msg/SteeringReport` | `/control_bench/vehicle/status/steering_status` |
| `~/input/current_accel` | `geometry_msgs/msg/AccelWithCovarianceStamped` | `/control_bench/localization/acceleration` |
| `~/input/current_operation_mode` | `autoware_adapi_v1_msgs/msg/OperationModeState` | `/control_bench/system/operation_mode/state` |

`current_operation_mode` 使用：

```text
QoS{1}.transient_local()
```

输出：

| Controller private topic | Type | Bench topic |
|---|---|---|
| `~/output/control_cmd` | `autoware_control_msgs/msg/Control` | `/control_bench/autoracer/control/raw_control_cmd` |

### 3.2 Autoware 自测中的必要输入模式

`test_controller_node.cpp` 证明 valid case 至少需要：

- trajectory；
- odometry；
- steering report；
- acceleration；
- autonomous operation mode；
- `map -> base_link` dummy transform / equivalent TF；
- MPC / PID / trajectory follower / vehicle info / nearest search 参数。

因此，bench fixture 不能只发布 topic，还必须保证 frame / stamp / TF 自洽。

### 3.3 command_gate 默认不会放行

`command_gate.py` 默认：

```text
enable_drive_commands = false
```

valid scenario 必须显式设置：

```text
enable_drive_commands = true
```

否则 final command 永远是 stop，只能验证 fail closed，不能验证 race controller raw command 经 gate 后的正常链路。

`command_gate` 还会发布 gear / hazard / turn / safety state。bench 必须显式 remap 这些 support topic，不能只 remap final control。

---

## 4. 本轮必须新增 / 修改的文件

### 4.1 新增文件

```text
src/autoracer_control/launch/race_control.launch.py
src/autoracer_control/launch/race_control_bench.launch.py
src/autoracer_control/config/race_controller.param.yaml
src/autoracer_control/autoracer_control/race_bench_fixture_publisher.py
src/autoracer_control/autoracer_control/race_bench_monitor.py
```

### 4.2 可选但推荐测试文件

```text
src/autoracer_control/test/test_race_control_launch.py
src/autoracer_control/test/test_race_control_bench_contract.py
```

### 4.3 需要同步更新

```text
src/autoracer_control/setup.py
src/autoracer_control/package.xml
src/autoracer_safety/package.xml
```

`setup.py` 需要安装新增 launch / config，并增加 fixture / monitor entry points。

`autoracer_control/package.xml` 至少补齐新 bench 需要的运行依赖：

```text
nav_msgs
autoware_adapi_v1_msgs
std_msgs
launch
launch_ros
ament_index_python
tf2_ros
```

如实现中直接依赖 package share lookup 或其他消息包，应同步声明。`autoracer_safety/package.xml` 当前代码已 import `autoware_planning_msgs.msg.Trajectory`，应补 `autoware_planning_msgs` 依赖。

---

## 5. `race_control.launch.py` 合同

`race_control.launch.py` 只负责启动 race controller，不启动 fixture，不启动 command_gate，不启动 pure pursuit。

### 5.1 必须提供 launch arguments

```text
reference_trajectory_topic default = /planning/trajectory
odometry_topic             default = /localization/kinematic_state
steering_topic             default = /vehicle/status/steering_status
accel_topic                default = /localization/acceleration
operation_mode_topic       default = /system/operation_mode/state
raw_control_topic          default = /autoracer/control/raw_control_cmd
vehicle_info_param_file    default = src/autoracer_bringup/config/hooke2/vehicle_info.param.yaml
race_param_file            default = src/autoracer_control/config/race_controller.param.yaml
```

Bench launch 必须覆盖这些 topic 为 `/control_bench/**`。

### 5.2 必须设置 controller mode

```text
lateral_controller_mode = mpc
longitudinal_controller_mode = pid
```

可以通过 `race_controller.param.yaml` 或 launch parameter override 设置，但 summary 必须证明最终值为上述两项。

### 5.3 必须 remap 完整输入输出

```text
~/input/reference_trajectory    -> reference_trajectory_topic
~/input/current_odometry        -> odometry_topic
~/input/current_steering        -> steering_topic
~/input/current_accel           -> accel_topic
~/input/current_operation_mode  -> operation_mode_topic
~/output/control_cmd            -> raw_control_topic
```

只 remap raw output 不够；如果漏掉任一 input remap，fixture 数据不会进入 controller。

### 5.4 不得做的事

`race_control.launch.py` 禁止：

- 启动 `pure_pursuit_controller`；
- 启动 `command_gate`；
- 写死 `/control_bench/**`；
- 写死生产 raw topic 导致 bench 无法覆盖；
- 修改 external Autoware 参数文件。

---

## 6. 参数加载合同

`controller_node_exe` 不能只靠 `lateral_controller_mode` / `longitudinal_controller_mode` 启动。实现必须加载完整参数链。

### 6.1 最小参数加载顺序

参数文件按以下顺序加载，后加载覆盖前加载：

```text
1. src/external/autoware/universe/control/autoware_mpc_lateral_controller/param/lateral_controller_defaults.param.yaml
2. src/external/autoware/universe/control/autoware_pid_longitudinal_controller/config/autoware_pid_longitudinal_controller.param.yaml
3. src/external/autoware/universe/control/autoware_trajectory_follower_node/param/trajectory_follower_node.param.yaml
4. src/autoracer_bringup/config/hooke2/vehicle_info.param.yaml
5. src/autoracer_control/config/race_controller.param.yaml
```

`race_controller.param.yaml` 必须最后加载。

### 6.2 `race_controller.param.yaml` 最小职责

`race_controller.param.yaml` 至少提供：

```text
lateral_controller_mode: "mpc"
longitudinal_controller_mode: "pid"
ego_nearest_dist_threshold: bench-compatible value
ego_nearest_yaw_threshold: bench-compatible value
```

如果 Autoware controller 启动还需要额外无默认参数，优先在 `race_controller.param.yaml` 里提供 bench 初值。不要复制 external 默认 YAML；不要把 bench 初值写成 Hooke2 真车标定。

### 6.3 参数来源边界

- Autoware 默认参数：只作为 controller package 默认值；
- Hooke2 vehicle info：只作为几何初值；
- HOOKE 手册：只作为 sanity check；
- bench overlay：只保证 ROS-only bench 可运行；
- 任何上述值都不能称为真车标定参数。

---

## 7. `race_control_bench.launch.py` 合同

`race_control_bench.launch.py` 负责把所有节点接到 `/control_bench`。默认运行命令：

```bash
ROS_DOMAIN_ID=97 ros2 launch autoracer_control race_control_bench.launch.py scenario:=straight
```

### 7.1 必须启动的节点

```text
race_bench_fixture_publisher
race_control.launch.py   # included with bench topic args
command_gate
race_bench_monitor
```

### 7.2 必须传给 `race_control.launch.py` 的 bench topic

```text
reference_trajectory_topic = /control_bench/planning/trajectory
odometry_topic             = /control_bench/localization/kinematic_state
steering_topic             = /control_bench/vehicle/status/steering_status
accel_topic                = /control_bench/localization/acceleration
operation_mode_topic       = /control_bench/system/operation_mode/state
raw_control_topic          = /control_bench/autoracer/control/raw_control_cmd
```

### 7.3 command_gate bench 参数

valid scenario 中 `command_gate` 参数必须为：

```text
enable_drive_commands = true
input_topic           = /control_bench/autoracer/control/raw_control_cmd
output_topic          = /control_bench/control/command/control_cmd
pose_topic            = /control_bench/localization/pose_with_covariance
trajectory_topic      = /control_bench/planning/trajectory
require_trajectory    = true
gear_topic            = /control_bench/control/command/gear_cmd
hazard_topic          = /control_bench/control/command/hazard_lights_cmd
turn_topic            = /control_bench/control/command/turn_indicators_cmd
state_topic           = /control_bench/autoracer/safety/state
```

negative scenario 可以通过缺输入 / stale 输入触发 fail closed，但不能通过忘记设置 `enable_drive_commands` 来伪造 PASS。

### 7.4 隔离要求

bench 必须同时使用：

```text
ROS_DOMAIN_ID = 独立值，例如 97
explicit /control_bench topic remap
```

`/control_bench` namespace 不是硬隔离，因为已有节点可使用绝对 topic。硬隔离依赖 `ROS_DOMAIN_ID`。summary 必须记录：

```text
namespace_only_isolation_sufficient = false
```

---

## 8. Synthetic fixture 合同

### 8.1 必须发布的 topic

| Topic | Type | 用途 |
|---|---|---|
| `/control_bench/planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | controller reference trajectory；gate trajectory freshness |
| `/control_bench/localization/kinematic_state` | `nav_msgs/msg/Odometry` | controller current odometry |
| `/control_bench/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | controller steering feedback |
| `/control_bench/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | controller acceleration input |
| `/control_bench/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` | controller operation mode input |
| `/control_bench/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | command_gate localization freshness |
| `/tf` or `/tf_static` in bench ROS domain | `tf2_msgs/msg/TFMessage` | `map -> base_link` transform matching odometry |

### 8.2 最小字段要求

Fixture 不能发布空消息。valid scenario 至少满足：

```text
Trajectory:
  header.frame_id = "map"
  header.stamp = now
  points >= 3
  each point:
    pose.position.x/y set
    pose.orientation valid quaternion, yaw aligned with local path direction
    longitudinal_velocity_mps set by scenario

Odometry:
  header.frame_id = "map"
  child_frame_id = "base_link"
  header.stamp = now
  pose.pose close to trajectory
  pose.pose.orientation valid quaternion
  twist.twist.linear.x = current_speed_mps for scenario

SteeringReport:
  stamp = now
  steering_tire_angle = scenario feedback value, default 0.0

AccelWithCovarianceStamped:
  header.frame_id = "base_link"
  header.stamp = now
  accel.accel.linear.x = scenario acceleration feedback, default 0.0

OperationModeState:
  stamp = now
  mode = AUTONOMOUS
  is_autoware_control_enabled = true
  publisher QoS = transient local

PoseWithCovarianceStamped:
  header.frame_id = "map"
  header.stamp = now
  pose.pose matches odometry pose

TF:
  map -> base_link matches odometry pose and yaw
```

### 8.3 Operation mode 边界

Bench valid scenario 可以 synthetic 发布 autonomous operation mode，因为这是 controller contract test。

生产 / 真车 launch 禁止 synthetic operation mode。

Negative operation mode 分两类：

```text
missing_operation_mode:
  期望：controller not ready 或无当前 scenario raw output

non_autonomous / control_disabled:
  只记录行为并检查输出是否安全/可解释；
  不默认要求 controller 无 raw output；
  当前 command_gate 不消费 OperationModeState，不能靠 gate 自动根据 operation mode fail closed。
```

---

## 9. Scenario 合同

每个 scenario 应作为独立 bench run 执行。推荐：

```bash
ROS_DOMAIN_ID=97 ros2 launch autoracer_control race_control_bench.launch.py scenario:=straight
ROS_DOMAIN_ID=97 ros2 launch autoracer_control race_control_bench.launch.py scenario:=missing_odometry
```

不要在同一个 controller 进程中连续跑 valid -> missing input，因为 Autoware polling subscriber 可能缓存上一帧有效数据。

如果实现选择一个进程内跑多个 scenario，必须实现明确 reset，并且 monitor 只接受 `scenario_start_time` 之后的新 raw / final message。

### 9.1 最小场景集合

| 场景 | 输入设置 | 机器判据 |
|---|---|---|
| `straight` | 直线 trajectory，current speed = target speed | raw 发布；`abs(steer) <= straight_abs_steer_max_rad` |
| `left_curve` | 左弯 trajectory | raw 发布；steer sign 为左转方向 |
| `right_curve` | 右弯 trajectory | raw 发布；steer sign 与 `left_curve` 相反 |
| `current_speed_low` | odom speed < trajectory target speed | raw 发布；`velocity > current_speed` 且/或 `acceleration > 0` |
| `current_speed_high` | odom speed > trajectory target speed | raw 发布；`velocity < current_speed` 且/或 `acceleration < 0` |
| `missing_trajectory` | 不发布当前 scenario trajectory | 当前 scenario 无可驾驶 final command；gate fail closed |
| `missing_odometry` | 不发布当前 scenario odometry | 当前 scenario 无 raw command或无可驾驶 final command |
| `missing_steering` | 不发布 current steering | 当前 scenario 无 raw command或无可驾驶 final command |
| `missing_acceleration` | 不发布 current acceleration | 当前 scenario 无 raw command或无可驾驶 final command |
| `missing_operation_mode` | 不发布 operation mode | 当前 scenario 无 raw command或无可驾驶 final command |
| `stale_pose` | gate pose stamp / receive time 超过 timeout | final command stop；safety state = `localization_timeout` |
| `raw_timeout` | valid 后停止 raw input 等待 timeout | final command stop；safety state = `raw_command_timeout` |

### 9.2 默认数值阈值

```text
straight_abs_steer_max_rad = 0.02
valid_raw_timeout_sec = 2.0
valid_final_timeout_sec = 2.0
publisher_count_expected = 1
```

如运行中发现 Autoware MPC 在 synthetic 直线下输出微小非零 steering，可调整阈值，但必须记录原因。

---

## 10. PASS / FAIL 合同

### 10.1 全局 PASS 必须满足

```text
controller_under_test == autoware_trajectory_follower_node/controller_node_exe
lateral_controller_mode == mpc
longitudinal_controller_mode == pid
pure_pursuit_started == false
ros_domain_id != 当前 EKF / CarMaker domain
namespace == /control_bench
namespace_only_isolation_sufficient == false
raw_control_topic == /control_bench/autoracer/control/raw_control_cmd
final_control_topic == /control_bench/control/command/control_cmd
raw_control_publisher_count == 1
final_control_publisher_count == 1
final_control_publisher_nodes contains command_gate, and final_control_publisher_count == 1
default_final_topic_publisher_count == 0
operation_mode_qos == transient_local
command_gate_used == true
command_gate_enable_drive_commands == true for valid scenario
scenario_results[*] all PASS
```

### 10.2 任一情况必须 FAIL

- 实际启动 `pure_pursuit_controller` 作为主测 controller；
- race controller input remap 缺失；
- raw output 发布到默认 `/autoracer/control/raw_control_cmd` 而不是 bench topic；
- final output 发布到默认 `/control/command/control_cmd`；
- controller 直接发布 final control，绕过 `command_gate`；
- `command_gate` support topics 未 remap 且与当前仿真 domain 重合；
- valid scenario 下 `command_gate` 因 `drive_disabled` 输出 stop；
- raw / final command 含 NaN / Inf；
- missing input scenario 复用旧缓存消息并误判 PASS；
- summary 把 synthetic 结果写成 CarMaker 或真车证据。

---

## 11. `race_bench_monitor` summary 合同

推荐路径：

```text
logs/race_control_bench/<timestamp>/runtime_summary.json
```

最小 JSON 字段：

```json
{
  "stage": "race_control_bench_ros_only",
  "result": "PASS",
  "ros_domain_id": "97",
  "namespace": "/control_bench",
  "namespace_only_isolation_sufficient": false,
  "controller_under_test": "autoware_trajectory_follower_node/controller_node_exe",
  "controller_node_names": ["/control_bench/controller"],
  "pure_pursuit_started": false,
  "lateral_controller_mode": "mpc",
  "longitudinal_controller_mode": "pid",
  "data_source": "synthetic_fixture",
  "command_gate_used": true,
  "command_gate_enable_drive_commands": true,
  "raw_control_topic": "/control_bench/autoracer/control/raw_control_cmd",
  "final_control_topic": "/control_bench/control/command/control_cmd",
  "raw_control_publisher_count": 1,
  "final_control_publisher_count": 1,
  "final_control_publisher_nodes": ["/control_bench/command_gate"],
  "default_final_topic_publisher_count": 0,
  "operation_mode_qos": "transient_local",
  "scenario_results": {
    "straight": "PASS",
    "left_curve": "PASS",
    "right_curve": "PASS",
    "current_speed_low": "PASS",
    "current_speed_high": "PASS",
    "missing_trajectory": "PASS",
    "missing_odometry": "PASS",
    "missing_steering": "PASS",
    "missing_acceleration": "PASS",
    "missing_operation_mode": "PASS",
    "stale_pose": "PASS",
    "raw_timeout": "PASS"
  },
  "numeric_checks": {
    "straight_abs_steer_max_rad": 0.02,
    "left_right_sign_opposite": true,
    "low_speed_accel_positive_or_velocity_gt_current": true,
    "high_speed_accel_negative_or_velocity_lt_current": true,
    "raw_no_nan_inf": true,
    "final_no_nan_inf": true
  },
  "fail_closed_checks": {
    "stale_pose_final_stop": true,
    "raw_timeout_final_stop": true,
    "safety_states_seen": ["localization_timeout", "raw_command_timeout"]
  },
  "does_not_validate": [
    "CarMaker closed-loop",
    "Stage B planner",
    "real vehicle calibration",
    "race performance"
  ]
}
```

实现可以增加字段，但不得删除上述核心证据。

---

## 12. Stage B 前执行边界

本文只交付 **Stage B 完成前** 可完成的 ROS-only synthetic bench。Stage B 完成后的 CarMaker bridge、closed-loop、真车性能验证不在本文执行范围内。

当前交付范围：

```text
synthetic trajectory / odometry / steering / acceleration / operation mode / pose / tf
  -> race controller
  -> command_gate
  -> monitor
  -> runtime_summary.json
```

当前结束条件：

1. upgraded race controller 可以启动；
2. MPC/PID 参数链路可以加载；
3. synthetic valid scenarios 下 raw / final command 可观测；
4. negative scenarios 下 `command_gate` fail closed；
5. bench 与当前 EKF / CarMaker 仿真隔离；
6. 所有 PASS / FAIL 都由 `runtime_summary.json` 和命令输出支撑。

Stage B 完成后的事项不在本文实现，也不要求本文提前规定：

- 如何从 CarMaker bridge 拉 odometry / steering / acceleration / operation mode；
- Stage B planner trajectory 的真实 topic 和质量判据；
- CarMaker closed-loop 指标；
- 真车或 race performance 指标；
- 是否复用、扩展或重写 monitor。

唯一保留的接口约束是：本文新增的 `race_control.launch.py` 必须通过 launch arguments 接收输入/输出 topic，不能把 synthetic topic 写死。Stage B 结束后应另起文档，根据当时已有的 CarMaker bridge 案例决定真实数据接入方式。

---

## 13. 当前不阻塞 Stage B 前 bench 的事项

以下问题需要真实数据、Stage B 或 CarMaker bridge 后才能关闭，但不阻塞本文交付：

- MPC/PID 是否完成 Hooke2 真车标定；
- steering feedback 符号、比例、延迟；
- acceleration feedback 语义；
- CarMaker bridge 的 odometry / steering / acceleration / operation mode 真实 topic；
- Stage B planner trajectory 质量；
- 车辆闭环完成率和 lateral error；
- race controller 是否优于 `pure_pursuit_controller`；
- race performance。

本文 bench 的结论只能是：

```text
race controller software contract passed under synthetic inputs
```

不能写成：

```text
CarMaker closed-loop passed
real vehicle calibrated
vehicle drivable
race performance improved
```

---

## 14. AI 执行边界

### 14.1 可以做

- 新增 `race_control.launch.py`；
- 新增 `race_control_bench.launch.py`；
- 新增 `race_controller.param.yaml`；
- 新增 `race_bench_fixture_publisher.py`；
- 新增 `race_bench_monitor.py`；
- 更新 `autoracer_control/setup.py`；
- 补齐 `package.xml` 依赖；
- 添加 launch / contract smoke tests；
- 只运行 ROS-only bench。

### 14.2 禁止做

- 不得把 `pure_pursuit_controller` 当成本轮主测对象；
- 不得修改 `src/external/autoware/**`；
- 不得修改 `carmaker_stage_b.launch.py`；
- 不得启动 CarMaker 或 `carmaker_ros_bridge`；
- 不得修改 `SimProject_TianmenRace`；
- 不得向默认 `/control/command/control_cmd` 发布 final control；
- 不得把 synthetic operation mode 写入生产 launch；
- 不得声称 synthetic bench 证明车辆性能、真车可驾驶或 race performance。

### 14.3 完成后必须提供证据

执行 AI 必须自己完成以下验证并在最终报告中给出命令、退出码和关键输出摘要。不得要求用户在执行过程中人工盯守这些项目。

```text
build/test command and result
ros2 node list: no pure_pursuit_controller
ros2 topic info --verbose /control_bench/autoracer/control/raw_control_cmd
ros2 topic info --verbose /control_bench/control/command/control_cmd
ros2 topic info --verbose /control/command/control_cmd
parameter evidence: lateral_controller_mode=mpc, longitudinal_controller_mode=pid
operation mode publisher QoS evidence: transient_local
runtime_summary.json with scenario PASS/FAIL details
```

### 14.4 执行中异常处理

如果实现中遇到本文未预料到的依赖、参数或运行时问题，执行 AI 不得自行降级目标或绕过合同。必须按以下规则处理：

```text
1. 先证明问题：给出失败命令、错误日志和涉及文件。
2. 判断类别：
   - 缺 package / package.xml 依赖；
   - 缺 launch remap / 参数文件；
   - Autoware controller 额外无默认参数；
   - fixture 字段或 TF 不满足 controller；
   - command_gate 配置不完整；
   - 环境/构建问题。
3. 若问题属于 ROS-only bench 范围，直接修复并重新验证。
4. 若修复需要修改禁止范围，例如 `src/external/autoware/**`、CarMaker、Stage B 或生产 operation mode，必须停止并报告为 blocked，不得绕过。
5. 如果某个 PASS 判据暂时无法机器化，必须在 summary 中标为 `UNKNOWN` 或 `NOT_IMPLEMENTED`，不得写成 `PASS`。
```

允许 AI 自行修复的例子：

```text
补 package.xml 依赖
补 setup.py entry point / launch install
补 race_controller.param.yaml 中缺失的 bench 参数
补 race_control.launch.py remap
补 fixture 消息字段、stamp、frame 或 TF
补 monitor 的 publisher count / summary 字段
```

不允许 AI 自行扩大范围的例子：

```text
把测试对象降级回 pure_pursuit_controller
启动 CarMaker 或 carmaker_ros_bridge
修改 SimProject_TianmenRace
修改 carmaker_stage_b.launch.py
修改 src/external/autoware/**
把 synthetic operation mode 放进生产 launch
把 synthetic bench 结果写成车辆性能或真车可驾驶
```

最终验收只看证据，不看口头说明。没有命令输出、summary 字段或日志支撑的项目，不得声称完成。

---

## 15. 最小实施顺序

1. 新增 `race_controller.param.yaml`，提供 mode、nearest search 和必要 bench override；
2. 新增 `race_control.launch.py`，完整 remap `~/input/*` 和 `~/output/control_cmd`；
3. 新增 `race_bench_fixture_publisher.py`，发布 trajectory / odometry / steering / acceleration / operation mode / pose / tf；
4. 新增 `race_control_bench.launch.py`，设置 `/control_bench` topic、command_gate 参数和独立 summary 路径；
5. 新增 `race_bench_monitor.py`，实现机器可判定 PASS / FAIL；
6. 更新 `setup.py` 和 `package.xml`；
7. 添加 launch / contract tests，至少验证不启动 pure pursuit、input remap 完整、gate topic 完整；
8. 构建；
9. 用独立 `ROS_DOMAIN_ID` 分 scenario 运行 bench；
10. 检查 summary；
11. 确认最终报告没有声称 Stage B、CarMaker closed-loop、真车标定或 race performance 已通过。

---

## 16. 建议验证命令

文档检查：

```bash
cd /opt/ipg/carmaker/linux64-15.1/autoracer_hooke
rg -n "~/input/reference_trajectory|~/input/current_odometry|~/input/current_steering|~/input/current_accel|~/input/current_operation_mode" docs/autoracer_control_redesign_plan.md
rg -n "enable_drive_commands|gear_topic|hazard_topic|turn_topic|state_topic" docs/autoracer_control_redesign_plan.md
rg -n "namespace_only_isolation_sufficient|scenario_start_time|transient_local|default_final_topic_publisher_count" docs/autoracer_control_redesign_plan.md
```

实现后构建建议：

```bash
cd /opt/ipg/carmaker/linux64-15.1/autoracer_hooke
colcon build --symlink-install --packages-up-to autoracer_control autoracer_safety autoware_trajectory_follower_node
```

bench 运行示例：

```bash
ROS_DOMAIN_ID=97 ros2 launch autoracer_control race_control_bench.launch.py scenario:=straight
ROS_DOMAIN_ID=97 ros2 launch autoracer_control race_control_bench.launch.py scenario:=missing_odometry
```

summary 检查：

```bash
rg -n "controller_under_test|lateral_controller_mode|longitudinal_controller_mode|pure_pursuit_started|data_source|result|does_not_validate" logs/race_control_bench
```

这些命令只验证 ROS-only race controller bench，不代表 Stage B、CarMaker closed-loop、真车标定或 race performance 已通过。
