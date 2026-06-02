# Autoracer Race Controller Upgrade Bench Plan

> 状态：重置后的控制器升级与测试计划。本文的目标不是继续测试旧的 `pure_pursuit_controller`，而是在 Stage B / CarMaker 闭环尚未完成、且不能影响当前 EKF / CarMaker 仿真的前提下，先把**升级后的 race controller** 独立接入并测试一遍。Stage B 完成后，再把 synthetic 数据源替换为 Stage B planner + CarMaker bridge 数据源，在 CarMaker 中做闭环验证。

原则：**先升级控制器，再隔离测试；不依赖 CarMaker；不污染当前仿真；不把 synthetic 结果伪装成车辆性能。**

---

## 1. 当前目标

当前项目已经有旧控制链路：

```text
/planning/trajectory
  -> pure_pursuit_controller
  -> /autoracer/control/raw_control_cmd
  -> command_gate
  -> /control/command/control_cmd
```

这条链路是 baseline，不是本轮目标。本轮目标是新增并测试升级后的控制链路：

```text
synthetic trajectory / odometry / steering / acceleration / operation mode
  -> race controller
       autoware_trajectory_follower_node/controller_node_exe
       lateral_controller_mode = mpc
       longitudinal_controller_mode = pid
  -> /autoracer/control/raw_control_cmd
  -> command_gate
  -> /control/command/control_cmd
  -> bench_monitor / runtime_summary.json
```

本轮先回答：

1. race controller 能不能在本项目中启动；
2. MPC/PID 参数和 remap 能不能加载；
3. synthetic 输入齐全时，race controller 能不能发布 raw control；
4. straight / left / right trajectory 下，steering 输出方向是否符合预期；
5. current speed 低于 / 高于 trajectory target speed 时，纵向输出方向是否符合预期；
6. 缺 trajectory、odometry、steering、acceleration 或 operation mode 时，controller / gate 是否 fail closed；
7. raw control 是否仍走 `command_gate`，final control 是否只有 gate 发布；
8. 这个测试是否完全不影响当前 EKF / CarMaker 仿真。

当前不回答：

- 车辆闭环能不能跑完赛道；
- race controller 是否优于 pure pursuit；
- MPC/PID 是否已经完成真车标定；
- Stage B planner 是否通过；
- CarMaker bridge 数据是否已经满足生产输入契约。

---

## 2. 测试对象必须现在确定

### 2.1 本轮主测对象

本轮主测对象是升级后的 race controller：

```text
package:    autoware_trajectory_follower_node
executable: controller_node_exe
mode:
  lateral_controller_mode: mpc
  longitudinal_controller_mode: pid
output:
  ~/output/control_cmd -> /autoracer/control/raw_control_cmd
```

这才是控制器升级工作本身。

### 2.2 pure pursuit 的位置

`pure_pursuit_controller` 只保留为：

- Stage A / Stage B baseline；
- 后续 CarMaker 对比基线；
- 回归参考。

它不是本轮 Control Bench 的主测对象。不要把“先不依赖 CarMaker 测一遍”理解成“测旧 controller”。本轮 bench 必须测试新增 race controller，否则对升级任务没有价值。

---

## 3. 当前仓库事实

### 3.1 旧 baseline 已存在

当前旧 controller：

```text
src/autoracer_control/autoracer_control/pure_pursuit_controller.py
src/autoracer_control/launch/control.launch.py
```

当前 Stage B baseline 仍启动 pure pursuit：

```text
src/autoracer_bringup/launch/carmaker_stage_b.launch.py
```

Stage B baseline 不应被静默替换。race controller 应该有独立 launch。

### 3.2 升级 controller 源码已在仓库中

目标 controller 源码位于：

```text
src/external/autoware/universe/control/autoware_trajectory_follower_node
```

本地源码确认其 controller 输入为：

| 输入 | Type | bench 来源 |
|---|---|---|
| `~/input/reference_trajectory` | `autoware_planning_msgs/msg/Trajectory` | synthetic trajectory |
| `~/input/current_odometry` | `nav_msgs/msg/Odometry` | synthetic odometry |
| `~/input/current_steering` | `autoware_vehicle_msgs/msg/SteeringReport` | synthetic steering feedback |
| `~/input/current_accel` | `geometry_msgs/msg/AccelWithCovarianceStamped` | synthetic acceleration |
| `~/input/current_operation_mode` | `autoware_adapi_v1_msgs/msg/OperationModeState` | synthetic operation mode, transient local QoS |

输出为：

| 输出 | Type | remap |
|---|---|---|
| `~/output/control_cmd` | `autoware_control_msgs/msg/Control` | `/autoracer/control/raw_control_cmd` |

Autoware 自带测试中，operation mode 至少需要：

```text
mode = OperationModeState::AUTONOMOUS
is_autoware_control_enabled = true
QoS = transient local
```

bench 可以 synthetic 发布这个状态，因为它只验证 controller contract。生产 / 真车 launch 不得伪造 operation mode。

### 3.3 command_gate 仍是最终输出门

`command_gate` 参数化输入输出：

```text
input_topic  = /autoracer/control/raw_control_cmd
output_topic = /control/command/control_cmd
pose_topic   = /localization/pose_with_covariance
trajectory_topic = /planning/trajectory
```

bench 中需要 remap 到 `/control_bench/**`，并发布 gate 所需 pose / trajectory，才能验证 gate 链路和 fail closed。

---

## 4. 本轮需要新增的升级入口

### 4.1 race control launch

新增：

```text
src/autoracer_control/launch/race_control.launch.py
```

职责：

- 启动 `autoware_trajectory_follower_node/controller_node_exe`；
- 加载 Autoware controller 默认参数和 Autoracer overlay；
- 设置：

```text
lateral_controller_mode = mpc
longitudinal_controller_mode = pid
```

- remap 输出：

```text
~/output/control_cmd -> /autoracer/control/raw_control_cmd
```

- 不启动 `pure_pursuit_controller`；
- 不启动 `command_gate`，由上层 bench / bringup 决定是否接 gate。

### 4.2 race controller 参数 overlay

新增：

```text
src/autoracer_control/config/race_controller.param.yaml
```

职责：

- 只保存 Autoracer 对 race controller 的覆盖参数；
- 明确 controller mode 为 `mpc` + `pid`；
- 使用 bench 可运行的 conservative 初值；
- 不把这些初值写成 Hooke2 真车标定参数；
- 不复制 external Autoware 默认 YAML；
- 不修改 `src/external/autoware/**`。

当前没有真车响应日志，因此 overlay 中的 MPC/PID 动态参数只能是 bench 初值或手册边界约束，不能称为已标定。

---

## 5. ROS-only Race Controller Bench

### 5.1 数据流

本轮 bench 数据流固定为：

```text
race_bench_fixture_publisher
  -> /control_bench/planning/trajectory
  -> /control_bench/localization/kinematic_state
  -> /control_bench/vehicle/status/steering_status
  -> /control_bench/localization/acceleration
  -> /control_bench/system/operation_mode/state
  -> /control_bench/localization/pose_with_covariance   # for command_gate
  -> race_control.launch.py
  -> /control_bench/autoracer/control/raw_control_cmd
  -> command_gate
  -> /control_bench/control/command/control_cmd
  -> race_bench_monitor
```

### 5.2 新增文件

新增最小文件集合：

```text
src/autoracer_control/launch/race_control.launch.py
src/autoracer_control/launch/race_control_bench.launch.py
src/autoracer_control/config/race_controller.param.yaml
src/autoracer_control/autoracer_control/race_bench_fixture_publisher.py
src/autoracer_control/autoracer_control/race_bench_monitor.py
```

必要时新增轻量测试：

```text
src/autoracer_control/test/test_race_control_launch.py
src/autoracer_control/test/test_race_control_bench_contract.py
```

不新增车辆动力学 test double。本轮不模拟车辆闭环，只测升级 controller 的输入输出合同和 gate 链路。Stage B 完成后直接用 CarMaker bridge 数据替换 synthetic fixture。

---

## 6. 隔离规则

bench 必须和当前 EKF / CarMaker 仿真隔离。

### 6.1 ROS_DOMAIN_ID 隔离

bench 默认使用独立 domain，例如：

```bash
ROS_DOMAIN_ID=97 ros2 launch autoracer_control race_control_bench.launch.py
```

如果当前 EKF / CarMaker 使用其他 domain，bench 不得复用该 domain。

### 6.2 Namespace / remap 隔离

bench 默认 namespace：

```text
/control_bench
```

bench topic：

```text
/control_bench/planning/trajectory
/control_bench/localization/kinematic_state
/control_bench/localization/pose_with_covariance
/control_bench/localization/acceleration
/control_bench/vehicle/status/steering_status
/control_bench/system/operation_mode/state
/control_bench/autoracer/control/raw_control_cmd
/control_bench/control/command/control_cmd
/control_bench/autoracer/safety/state
```

如果某个节点因为绝对 topic 无法完全 namespace，必须依赖 `ROS_DOMAIN_ID` 做硬隔离，并在 summary 中记录该限制。

### 6.3 禁止影响当前仿真

bench 禁止：

- 启动 CarMaker；
- 启动 `carmaker_ros_bridge`；
- 修改 `SimProject_TianmenRace`；
- 修改 EKF / localization launch；
- 修改 `carmaker_stage_b.launch.py`；
- 向当前 EKF / CarMaker 所在 ROS domain 发布 topic；
- 在默认生产 topic 上发布 `/control/command/control_cmd`；
- 用 synthetic operation mode 冒充生产授权状态。

---

## 7. Synthetic fixture 合同

### 7.1 必须发布的输入

| Topic | Type | 用途 |
|---|---|---|
| `/control_bench/planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | race controller reference trajectory；command_gate trajectory freshness |
| `/control_bench/localization/kinematic_state` | `nav_msgs/msg/Odometry` | race controller current odometry |
| `/control_bench/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | race controller steering feedback |
| `/control_bench/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | race controller acceleration input |
| `/control_bench/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` | race controller readiness gate；bench synthetic only |
| `/control_bench/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | command_gate localization freshness |

### 7.2 OperationModeState 合同

bench 的 valid scenario 中，fixture 应发布：

```text
mode = AUTONOMOUS
is_autoware_control_enabled = true
stamp = now
QoS = transient local
```

negative scenario 中，应覆盖：

- missing operation mode；
- non-autonomous mode；
- control disabled。

这些只用于验证 race controller readiness，不允许进入生产 launch。

### 7.3 场景集合

最小场景：

| 场景 | 目的 | 预期 |
|---|---|---|
| `straight` | 直线轨迹 | steering 约为 0，raw control 发布 |
| `left_curve` | 左弯轨迹 | steering 为左转方向 |
| `right_curve` | 右弯轨迹 | steering 为右转方向，且与 left_curve 符号相反 |
| `current_speed_low` | 当前速度低于目标速度 | longitudinal acceleration / velocity command 倾向加速 |
| `current_speed_high` | 当前速度高于目标速度 | longitudinal acceleration / velocity command 倾向减速 |
| `missing_trajectory` | 缺 reference trajectory | race controller 不应输出可驾驶 command，gate fail closed |
| `missing_odometry` | 缺 odometry | race controller not ready 或无 raw command |
| `missing_steering` | 缺 steering feedback | race controller not ready 或无 raw command |
| `missing_acceleration` | 缺 acceleration | race controller not ready 或无 raw command |
| `missing_operation_mode` | 缺 operation mode | race controller not ready 或无 raw command |
| `stale_pose` | command_gate pose 过期 | final command fail closed |
| `raw_timeout` | raw control 超时 | command_gate fail closed |

场景只验证控制器合同和符号，不验证车辆轨迹跟踪性能。

---

## 8. PASS / FAIL 判据

### 8.1 PASS 条件

一个 bench run 只有同时满足以下条件才可 PASS：

- 使用独立 `ROS_DOMAIN_ID`；
- `controller_under_test = autoware_trajectory_follower_node/controller_node_exe`；
- `lateral_controller_mode = mpc`；
- `longitudinal_controller_mode = pid`；
- 未启动 `pure_pursuit_controller`；
- valid scenario 下 race controller 发布 raw control；
- raw control remap 到 `/control_bench/autoracer/control/raw_control_cmd`；
- final control 由 `command_gate` 发布到 `/control_bench/control/command/control_cmd`；
- raw control 和 final control 都只有一个 publisher；
- straight steering 接近 0；
- left / right steering 符号相反；
- current_speed_low / current_speed_high 的纵向输出方向可解释；
- raw / final control 无 NaN / Inf；
- missing input / stale / timeout scenario fail closed；
- summary 明确 `data_source = synthetic_fixture`；
- summary 明确 `does_not_validate` 包含 CarMaker closed-loop、Stage B planner、real vehicle calibration、race performance。

### 8.2 FAIL 条件

出现以下任一情况必须 FAIL：

- 实际启动的是 `pure_pursuit_controller` 而不是 race controller；
- 同一 bench 中存在多个 raw control publisher；
- race controller 直接发布 final control，绕过 `command_gate`；
- synthetic operation mode 被写入生产 launch；
- 缺关键输入时仍持续输出可驾驶 final command；
- raw / final command 有 NaN / Inf；
- steering 符号和场景不一致；
- 纵向输出方向和 current speed / target speed 关系不可解释；
- bench 发布到当前 EKF / CarMaker domain；
- summary 把 synthetic 结果写成 CarMaker 或真车证据。

---

## 9. Bench monitor 合同

`race_bench_monitor` 订阅：

```text
/control_bench/autoracer/control/raw_control_cmd
/control_bench/control/command/control_cmd
/control_bench/autoracer/safety/state
```

推荐 summary 路径：

```text
logs/race_control_bench/<timestamp>/runtime_summary.json
```

summary 至少包含：

```text
stage = race_control_bench_ros_only
controller_under_test = autoware_trajectory_follower_node/controller_node_exe
lateral_controller_mode = mpc
longitudinal_controller_mode = pid
data_source = synthetic_fixture
ros_domain_id
namespace
pure_pursuit_started = false
race_controller_started
raw_control_published
final_control_published
raw_control_single_publisher
final_control_single_publisher
command_gate_used
straight_steer_near_zero
left_right_steer_sign
current_speed_low_high_longitudinal_direction
no_nan_control
missing_inputs_fail_closed
stale_pose_fail_closed
raw_timeout_fail_closed
does_not_validate = [
  "CarMaker closed-loop",
  "Stage B planner",
  "real vehicle calibration",
  "race performance"
]
```

---

## 10. Stage B 完成后的替换规则

Stage B 完成后，不重写 race controller，也不重写 monitor。只替换 fixture 数据源。

当前 ROS-only bench：

```text
synthetic trajectory / odometry / steering / acceleration / operation mode / pose
  -> race controller
  -> command_gate
  -> monitor
```

Stage B + CarMaker bridge bench：

```text
Stage B planner trajectory
CarMaker bridge / adapter odometry
CarMaker bridge / adapter steering feedback
CarMaker bridge / adapter acceleration
CarMaker bridge / adapter operation mode or approved test mode source
CarMaker bridge pose
  -> same race controller
  -> same command_gate
  -> same monitor
```

替换动作：

1. 禁用 `race_bench_fixture_publisher`；
2. 保留 `race_control.launch.py`；
3. controller trajectory 输入接 Stage B planner；
4. controller 状态输入接 CarMaker bridge / adapter 数据；
5. `command_gate` 仍是 final command 唯一 publisher；
6. summary 中写：

```text
data_source = carmaker_bridge
trajectory_source = stage_b_planner
```

7. 新增闭环指标：

```text
distance
route_progress
lateral_error
heading_error
speed_error
steering_saturation_count
accel_decel_saturation_count
gate_stop_count
offroad_or_abort
completion_time
```

Stage B 后如何从 CarMaker 拉数据，以后按已有案例接入；本文现在不提前虚构 bridge 细节。

---

## 11. HOOKE 手册的使用边界

HOOKE 手册可用于给 race controller 参数设置边界，但不能替代标定。

可用于：

- wheelbase 初始值；
- 最大前轮转角量级；
- 速度上限 sanity check；
- 加速 / 制动能力 sanity check；
- steering response 上限 sanity check。

不可用于直接声称：

- MPC/PID 已标定；
- steering delay / tau 已知；
- steering feedback 符号 / 比例已知；
- acceleration feedback 已验证；
- 真车 operation mode 联动已验证；
- race controller 可驾驶。

因此，本轮 bench 可以使用保守参数让 controller 启动和输出，但 summary 必须写成 bench 初值，不得写成真车标定值。

---

## 12. 当前还缺什么

### 12.1 现在必须补的实现

| 项 | 状态 |
|---|---|
| `race_control.launch.py` | 缺失 |
| `race_controller.param.yaml` | 缺失 |
| `race_control_bench.launch.py` | 缺失 |
| `race_bench_fixture_publisher.py` | 缺失 |
| `race_bench_monitor.py` | 缺失 |
| setup.py entry points | 缺失 |
| launch / contract smoke test | 缺失 |

### 12.2 现在需要确认但不需要真车数据

| 项 | 当前结论 |
|---|---|
| 主测 controller | `autoware_trajectory_follower_node/controller_node_exe` |
| 横向模式 | `mpc` |
| 纵向模式 | `pid` |
| 输出链路 | `race controller -> /autoracer/control/raw_control_cmd -> command_gate -> /control/command/control_cmd` |
| bench 数据源 | synthetic fixture |
| bench 隔离 | 独立 `ROS_DOMAIN_ID` + `/control_bench` remap |
| 是否测试旧 pure pursuit | 不作为本轮主测试；只作为后续 baseline 对比 |

### 12.3 Stage B / CarMaker 后续需要

- Stage B planner trajectory PASS 证据；
- CarMaker bridge topic 列表和消息语义；
- 如何从 CarMaker 拉数据的参考案例；
- odometry / acceleration / steering / operation mode 的 CarMaker bridge 或 adapter 来源；
- 车辆响应指标计算方法。

### 12.4 真车阶段后续需要

- steering ratio / VGR；
- steering feedback 符号、比例、延迟；
- acceleration response；
- operation mode / command gate / 人工接管联动；
- MPC/PID 调参日志。

这些不阻塞 ROS-only race controller bench，但会阻塞真车部署和 race performance 结论。

---

## 13. AI 执行边界

AI 可以直接实现：

- race controller launch；
- race controller 参数 overlay；
- ROS-only race controller bench；
- synthetic fixture；
- monitor 和 summary；
- launch / contract tests；
- 文档同步更新。

AI 不得：

- 把 pure pursuit 当成本轮升级测试对象；
- 修改 `src/external/autoware/**`；
- 修改当前 EKF / CarMaker 仿真入口；
- 启动 CarMaker；
- 伪造 CarMaker bridge 数据；
- 把 synthetic operation mode 放进生产 launch；
- 宣称 synthetic bench 证明车辆性能或真车可驾驶。

---

## 14. 最小实施顺序

1. 新增 `race_controller.param.yaml`，明确 `mpc` + `pid` bench 初值；
2. 新增 `race_control.launch.py`，启动 `controller_node_exe` 并 remap raw output；
3. 新增 `race_bench_fixture_publisher.py`，发布 trajectory / odometry / steering / acceleration / operation mode / pose；
4. 新增 `race_control_bench.launch.py`，在 `/control_bench` 下启动 fixture、race controller、command_gate、monitor；
5. 新增 `race_bench_monitor.py`，生成 `runtime_summary.json`；
6. 添加 launch / contract tests，确保不会启动 pure pursuit，不会绕过 command_gate；
7. 用独立 `ROS_DOMAIN_ID` 运行 bench；
8. 检查 summary；
9. Stage B 完成后，禁用 fixture，接 Stage B planner + CarMaker bridge 数据复跑。

---

## 15. 建议验证命令

文档检查：

```bash
cd /opt/ipg/carmaker/linux64-15.1/autoracer_hooke
rg -n "controller_node_exe|lateral_controller_mode = mpc|longitudinal_controller_mode = pid|race_control_bench" docs/autoracer_control_redesign_plan.md
```

实现后构建：

```bash
cd /opt/ipg/carmaker/linux64-15.1/autoracer_hooke
colcon build --symlink-install --packages-select autoracer_control autoracer_safety autoware_trajectory_follower_node
```

bench 运行：

```bash
ROS_DOMAIN_ID=97 ros2 launch autoracer_control race_control_bench.launch.py
```

summary 检查：

```bash
rg -n "controller_under_test|lateral_controller_mode|longitudinal_controller_mode|pure_pursuit_started|data_source|PASS|FAIL" logs/race_control_bench
```

这些命令只验证 ROS-only race controller bench，不代表 Stage B、CarMaker closed-loop、真车标定或 race performance 已通过。
