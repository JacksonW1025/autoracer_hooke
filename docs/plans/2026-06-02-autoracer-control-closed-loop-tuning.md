# Autoracer 控制器 ROS-only 闭环调参方案

> **给 Claude:** 执行本文计划时必须使用 `superpowers:executing-plans`，按任务逐项实现和验证。

**目标：** 新增一个不依赖 CarMaker 的 ROS-only 闭环控制调参 bench，用真实 Autoware `controller_node_exe` 加一个小型虚拟底盘模型，先收敛出一版低速可用的 MPC/PID 参数候选 overlay。

**架构：** 保留已经执行过的 `race_control_bench` 作为 wiring / software contract 检查，不继续把它扩展成调参工具。新增独立闭环 bench：真实 controller 发布 raw control，虚拟底盘根据 raw control 积分车辆状态并反馈 odometry / steering / acceleration，monitor 评估跟踪误差、转角饱和和振荡。CarMaker 明确不属于本阶段。

**技术栈：** ROS 2、Python `rclpy`、Autoware `autoware_trajectory_follower_node/controller_node_exe`、`autoware_control_msgs/msg/Control`、`autoware_planning_msgs/msg/Trajectory`、`nav_msgs/msg/Odometry`、`autoware_vehicle_msgs/msg/SteeringReport`、`geometry_msgs`、现有 `autoracer_control` 包。

## 0. 当前共识和执行状态

本方案的控制策略范围已经收敛：

- 不自研 lateral / longitudinal controller。
- 控制器只采用 Autoware `controller_node_exe`，模式固定为 MPC lateral + PID longitudinal。
- 本仓库自己的代码只做 bench fixture、virtual chassis、monitor、report 和参数 overlay。
- 速度/曲率相关内容只作为 Autoware 现有参数和诊断维度处理，不实现新的速度分段控制器、在线参数切换器或横向加速度控制器。
- CarMaker 不是默认路径，也不是本阶段交付条件；本地 ROS-only bench 的目的只是给实车前提供一个可复现的初始调参基准。

截至 `logs/control_closed_loop/20260602-autoracer-closed-loop-tuning/tuning_report.md` 中的记录：

| 项 | 状态 |
| --- | --- |
| controller I/O 合同核验 | PASS；5 个输入和 1 个 raw control 输出与 Autoware 源码一致 |
| unit / static contract | PASS |
| ROS node / launch | PASS；closed-loop bench 已接通 |
| smoke / baseline metrics | PASS；六个 baseline 场景都有 summary 且 metrics finite |
| MPC/PID 参数调优 | 未收敛；最多 5 轮后仍有 `s_curve` terminal gate failure |
| V1.5 validation harness | PASS；已增加 progress / completion / segment metrics，验证平台升级完成，但不等价于参数已交付 |

因此当前真实结论是：

```text
Autoware MPC+PID 控制器链路和 ROS-only closed-loop bench 已经能跑；
当前 candidate overlay 是低速候选，不是已交付的最终参数；
剩余主要问题是场景验收门槛和参数收敛证据不足，不是 CarMaker 缺失。
```

## 1. 为什么要把旧方案放在一边

旧方案 `docs/autoracer_control_redesign_plan.md` 应当视为已经执行过的 **software-contract bench 方案**，而不是继续推进的控制参数调参方案。

它已经产出的有效部分：

- 新增了 `race_control.launch.py`，用于启动 `autoware_trajectory_follower_node/controller_node_exe`。
- 控制器模式已经设为 `lateral_controller_mode = mpc`、`longitudinal_controller_mode = pid`。
- 引入了 `/control_bench/**` topic remap，用于 ROS-only 隔离。
- 新增了 `race_bench_fixture_publisher.py`、`race_bench_monitor.py`、`race_control_bench.launch.py`。
- `command_gate` 已经被增强到满足 bench 需要，包括 trajectory freshness 和 support command topic。
- runtime summary 证明了 controller chain 能启动、能发布 raw control、能经过 `command_gate` 输出 final control，且没有启动 `pure_pursuit_controller`。

从 `logs/race_control_bench/*/runtime_summary.json` 看到的结果：

| 场景 | 最新观察状态 |
| --- | --- |
| `straight` | PASS |
| `left_curve` | PASS |
| `right_curve` | PASS |
| `current_speed_low` | PASS |
| `current_speed_high` | PASS |
| `missing_trajectory` | PASS |
| `missing_odometry` | PASS |
| `missing_steering` | PASS |
| `missing_acceleration` | PASS |
| `missing_operation_mode` | PASS |
| `stale_pose` | PASS |
| `raw_timeout` | 曾经 FAIL，后在 `20260602-180132` PASS |

这些证据足够支持一个窄结论：

```text
升级后的 race controller software chain 在 ROS-only synthetic input 下已经接通并可启动。
```

但这些证据不能支持：

```text
MPC/PID 参数已经调好。
车辆能够稳定跟踪轨迹。
CarMaker closed-loop 已通过。
实车行为已经验证。
```

因此，新方案不再继续扩展旧 contract bench，而是新增独立的闭环调参 bench。

## 2. 本方案边界

本方案不使用 CarMaker。

理由：

- 控制链路已经通过 contract bench 验证过，不需要 CarMaker 再证明 wiring。
- 后续真实有效的标定大概率发生在实车；CarMaker 即使使用，也只是可选的中间仿真补充，不是默认路线。
- 现在把 CarMaker 加进来，会把 control tuning 和 bridge、仿真时钟、定位、planner 输出、场景质量混在一起。
- 当前真正需要的是一版低速可用的初始控制参数，而不是最终性能证明。

明确不做：

- 不启动 CarMaker 或 `carmaker_ros_bridge`。
- 不修改 CarMaker TestRun、车辆数据集或 Stage B launch。
- 不声明 Stage B planner 质量。
- 不声明 CarMaker closed-loop 性能。
- 不声明实车标定完成。
- 不修改 `src/external/autoware/**`。
- 不用自写控制器替换 Autoware MPC。
- 不设计新的速度分段控制器、横向加速度控制器、在线参数切换器或控制器切换框架。
- 不在 virtual chassis 内部修改 controller 输入的 trajectory target speed。
- 不新增自定义 ROS message / service / action。
- 不新增自定义控制接口 topic；bench 主接口只使用 `controller_node_exe` 已有输入输出接口在 `/control_bench/**` 下的 remap。
- 不计划、不执行 git commit / branch 操作。

bench 必须依赖的 `/control_bench/**` 主接口 topic 只有以下列表；`/rosout`、`/parameter_events` 等 ROS 系统内建 topic 不计入本约束：

```text
/control_bench/planning/trajectory
/control_bench/system/operation_mode/state
/control_bench/localization/kinematic_state
/control_bench/vehicle/status/steering_status
/control_bench/localization/acceleration
/control_bench/autoracer/control/raw_control_cmd
```

注意：Autoware controller 自身会创建若干内建 debug publisher，例如 processing time、debug marker、MPC/PID diagnostic 或 predicted trajectory。它们不是本文新增的 bench 控制接口。计划约束的重点是：不为 bench 引入新的业务消息格式或新的控制链路 topic。

允许使用的消息类型仅限现有标准类型：

```text
autoware_planning_msgs/msg/Trajectory
autoware_adapi_v1_msgs/msg/OperationModeState
nav_msgs/msg/Odometry
autoware_vehicle_msgs/msg/SteeringReport
geometry_msgs/msg/AccelWithCovarianceStamped
autoware_control_msgs/msg/Control
```

`closed_loop_summary.json` 和 `tuning_report.md` 是文件产物，不是 ROS topic / msg。

### 2.1 `controller_node_exe` 真实 I/O contract

执行本计划前，必须先按源码合同理解控制器，而不是按“mock 输入大概够用”理解控制器。

已核对的源码入口：

```text
autoracer_hooke/src/autoracer_control/launch/race_control.launch.py
pilot-auto.x1/src/autoware/universe/control/autoware_trajectory_follower_node/src/controller_node.cpp
pilot-auto.x1/src/autoware/universe/control/autoware_trajectory_follower_node/include/autoware/trajectory_follower_node/controller_node.hpp
pilot-auto.x1/src/autoware/universe/control/autoware_mpc_lateral_controller/src/mpc_lateral_controller.cpp
pilot-auto.x1/src/autoware/universe/control/autoware_pid_longitudinal_controller/src/pid_longitudinal_controller.cpp
```

`race_control.launch.py` 启动真实 Autoware `controller_node_exe`，并显式设置：

```yaml
lateral_controller_mode: mpc
longitudinal_controller_mode: pid
```

因此本 bench 不是单独横向控制模块测试，而是 MPC 横向 + PID 纵向组合控制器的模块级闭环测试。

控制器正常进入计算需要 5 个输入：

| Controller 订阅名 | bench topic | 类型 | 控制器实际使用语义 |
| --- | --- | --- | --- |
| `~/input/reference_trajectory` | `/control_bench/planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | 参考轨迹、目标速度、目标加速度、航向/曲率相关字段 |
| `~/input/current_odometry` | `/control_bench/localization/kinematic_state` | `nav_msgs/msg/Odometry` | 当前位姿 `pose.pose` 和当前速度 `twist.twist.linear.x` |
| `~/input/current_steering` | `/control_bench/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | 当前实际前轮转角 `steering_tire_angle` |
| `~/input/current_accel` | `/control_bench/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | 当前实际纵向加速度 `accel.accel.linear.x` |
| `~/input/current_operation_mode` | `/control_bench/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` | 是否处于 AUTONOMOUS 且 Autoware control enabled |

控制器输出只有一个主控制接口：

| Controller 发布名 | bench topic | 类型 | plant 使用方式 |
| --- | --- | --- | --- |
| `~/output/control_cmd` | `/control_bench/autoracer/control/raw_control_cmd` | `autoware_control_msgs/msg/Control` | 横向使用 `lateral.steering_tire_angle`；纵向使用 `longitudinal.acceleration`，`longitudinal.velocity` 只作为目标速度/监控量 |

关键语义约束：

- mock 可以替代真实来源，但不能改变接口语义。
- 位姿输入来自 `nav_msgs/Odometry.pose.pose`，不是单独 pose topic。
- 当前速度来自 `nav_msgs/Odometry.twist.twist.linear.x`。
- 当前转角反馈必须是实际前轮转角，不是上一帧 steering command。
- 当前加速度反馈必须是虚拟底盘积分状态中的实际加速度。
- trajectory、odometry、steering、acceleration 必须使用一致的时间节奏和坐标语义。
- operation mode 必须让 controller 处于可输出状态，否则 MPC/PID 可能进入非控制分支。

Trajectory fixture 不允许只填 x/y/v。每个 `TrajectoryPoint` 必须显式填充有限值：

```text
pose.position.x/y/z
pose.orientation.x/y/z/w
longitudinal_velocity_mps
lateral_velocity_mps
acceleration_mps2
heading_rate_rps
front_wheel_angle_rad
rear_wheel_angle_rad
time_from_start
```

MPC 至少需要 3 个点，PID 至少需要 2 个点。用于调参时还必须满足预测窗口长度，轨迹长度应覆盖 `mpc_prediction_horizon * mpc_prediction_dt * v_ref` 对应的预测距离并留余量，不能用最小点数代替调参轨迹。

当前控制计算路径未发现必须使用 TF lookup。bench 不新增 `/control_bench/localization/pose_with_covariance`，也不预设 TF 依赖；若后续 smoke test 证明运行时确实缺 TF，应先记录具体日志和调用路径，再补标准 TF，而不是凭空增加接口。

### 2.2 AI 执行协议和门禁

本计划可以交给 AI 执行，但只能按下面的门禁推进。AI 不得把“实现了代码”或“参数看起来合理”当成完成证据。

执行代理必须遵守：

```text
先证据，后结论。
先源码合同，后实现。
先 smoke test，后调参。
先 low_speed_tuning，后 guarded_speed_probe。
失败即停机分析，不盲目扩大范围。
```

允许 AI 自主推进的范围：

| 阶段 | 可自主执行任务 | 进入下一阶段的硬门禁 |
| --- | --- | --- |
| Phase 0: contract audit | Task 0 | `tuning_report.md` 已记录源码合同核验；若源码和本文不一致，必须先修本文并停止后续实现 |
| Phase 1: unit/contract implementation | Task 1、Task 2 | pytest 通过；contract test 证明没有 CarMaker、自定义 msg、final control 依赖或额外 `/control_bench/**` 控制接口 |
| Phase 2: ROS node/launch implementation | Task 3、Task 4、Task 5、Task 6 | `colcon build --packages-select autoracer_control --symlink-install` 通过 |
| Phase 3: controller smoke | 最小直线场景 smoke | controller 在 10 s 内收到 5 类输入并发布 `/control_bench/autoracer/control/raw_control_cmd`；virtual chassis 同步发布 odometry/steering/acceleration |
| Phase 4: baseline metrics | 全部最小场景 baseline | 每个场景都有 `closed_loop_summary.json`，且 metrics 字段完整、无 NaN/Inf |
| Phase 5: tuning loop | Task 7、Task 8 | 每轮只改一类 controller 参数；before/after metrics 写入 `tuning_report.md`；最多 5 轮 |

任何阶段不满足门禁时，AI 必须停止推进并输出：

```text
blocked_phase
failed_command
observed_error
relevant_log_path
why_the_next_phase_is_not_allowed
minimal_next_action
```

禁止 AI 做以下事情：

- 跳过 Task 0 直接写 node / launch。
- 没有 smoke test 就开始调 MPC/PID 参数。
- 没有 `closed_loop_summary.json` 就宣布参数收敛。
- 为了让场景 PASS 去放宽 `virtual_chassis_node` 的 plant 参数或 guardrail。
- 把 `/control_bench/localization/pose_with_covariance`、EKF/NDT pose 或 TF 作为默认 bench 输入。
- 使用 `final control` 调 controller，或把 `command_gate` 重新塞回本 bench。
- 修改 `src/external/autoware/**`。
- 新增自定义 msg / srv / action。
- 把 Autoware 内部 debug topic 当成 bench 控制接口。
- 一轮同时修改 MPC delay/tau、prediction window、MPC weights、PID gains 多类参数。
- 在没有重跑失败场景和至少一个已通过场景的情况下声称修复。
- 计划或执行 git commit / branch / push。

AI 每完成一个 Phase，必须在 `tuning_report.md` 追加一节：

```text
phase
commands_run
files_changed
evidence_paths
pass_fail_result
next_phase_allowed: true|false
```

交给 AI 的最小执行指令应是：

```text
按 docs/plans/2026-06-02-autoracer-control-closed-loop-tuning.md 执行。
先执行 2.2 节 Phase 0 门禁。
未满足当前 Phase 门禁时停止并报告，不要继续实现后续任务。
不得引入 CarMaker、EKF pose topic、自定义消息或 final control 闭环。
最终只能用 closed_loop_summary.json 和 tuning_report.md 证明结果。
```

### 2.3 可托管执行定义

本节用于把“可以交给 AI 去做”定义成可执行状态机，而不是一句口头授权。

从现在开始，执行 AI 可以在 **当前 Phase 门禁通过后自动进入下一 Phase**，不需要每个 Phase 都回头询问人类是否继续。但自动推进只在以下条件同时满足时允许：

```text
1. 当前 Phase 的命令已经真实执行。
2. 当前 Phase 的证据文件已经生成或测试命令 exit code 为 0。
3. `tuning_report.md` 已追加当前 Phase 记录。
4. 当前 Phase 记录中 `next_phase_allowed: true`。
5. 没有触发 2.2 节禁止项。
```

执行 AI 的主循环必须等价于：

```text
current_phase = first incomplete phase

while current_phase is not complete:
  read this plan
  run only tasks allowed by current_phase
  run the verification commands for current_phase
  append evidence to tuning_report.md

  if gate_passed:
    set next_phase_allowed: true
    move to next phase
  else:
    set next_phase_allowed: false
    print blocked_phase report
    stop
```

执行 AI 不允许因为“不确定”而自行改目标范围。遇到不确定时按下面规则处理：

| 不确定项 | 正确处理 |
| --- | --- |
| 源码合同和本文冲突 | 停在 Phase 0，列出冲突文件和行号，先修 plan |
| controller 不出 raw control | 停在 Phase 3，保留日志，定位缺失输入 / QoS / parameter namespace |
| 某场景 metrics 缺字段或 NaN | 停在 Phase 4，修 monitor 或 fixture，不调参数 |
| 低速场景失败 | 进入 Phase 5 调参闭环，但每轮只改一类 controller 参数 |
| 3.0 m/s guarded probe 失败 | 记录 remaining failure，不强行调到 PASS |
| 需要 CarMaker、EKF、NDT、TF 或 final control 才能继续 | 默认判断为跑偏；只有源码日志证明 controller 主计算路径确实需要时，才作为 blocked report 提交，不自行引入 |

可交付给 AI 的完整提示词如下，后续应直接复制这一段：

```text
你在 /opt/ipg/carmaker/linux64-15.1/autoracer_hooke 工作。

执行 docs/plans/2026-06-02-autoracer-control-closed-loop-tuning.md。

严格按 2.2 和 2.3 节的 Phase gate 状态机执行：
- 从第一个未完成 Phase 开始。
- 当前 Phase 通过门禁后，可以自动进入下一 Phase。
- 当前 Phase 不通过门禁时，必须停止，不得继续后续任务。
- 停止时输出 blocked_phase / failed_command / observed_error / relevant_log_path / why_the_next_phase_is_not_allowed / minimal_next_action。

禁止：
- 引入 CarMaker。
- 引入 EKF/NDT pose topic 或把 TF 作为默认输入。
- 自研控制器、速度分段在线切换器或横向加速度控制器。
- 在 virtual chassis 内部改写 trajectory target speed。
- 新增自定义 msg/srv/action。
- 使用 final control 闭环。
- 修改 src/external/autoware/**。
- 通过放宽 virtual chassis plant 参数或 guardrail 让场景 PASS。
- 没有 closed_loop_summary.json 和 tuning_report.md 就宣布参数收敛。
- 计划或执行 git commit / branch / push。

每个 Phase 完成后必须追加 tuning_report.md，记录：
phase / commands_run / files_changed / evidence_paths / pass_fail_result / next_phase_allowed。

最终完成只能以 closed_loop_summary.json、tuning_report.md、pytest/build/launch 命令输出作为证据。
```

### 2.4 Autoware 控制器范围锁定

本方案现在明确锁定为 **用好 Autoware controller**，不是重新设计控制器。

允许修改的内容：

- `race_controller.closed_loop_candidate.param.yaml` 中 Autoware 已声明并被 `controller_node_exe` / MPC / PID 消费的参数。
- closed-loop bench 的 fixture、virtual chassis、monitor、scenario spec、report 逻辑。
- 项目 planning 侧已有的 trajectory target speed 生成参数；例如现有 `local_trajectory_planner` 中的横向加速度限速参数，可以作为规划输出速度的来源之一单独讨论和调优。

不允许修改或新增的内容：

- 不新增 controller 输入输出消息格式。
- 不新增 controller topic contract。
- 不把 EKF/NDT pose topic、TF 或 final control 作为默认控制输入。
- 不写自研 MPC、Pure Pursuit、Stanley、LQR 或纵向 PID 替代 Autoware controller。
- 不实现多套 controller overlay 按速度硬切换。
- 不实现自定义速度/曲率调度器；若需要速度/曲率相关控制约束，优先使用 Autoware MPC 已有参数字段。
- 不让 virtual chassis 生成、修改或裁剪 trajectory target speed；virtual chassis 只模拟被控对象。

速度和曲率的语义必须保持清楚：

```text
trajectory.longitudinal_velocity_mps:
  controller 的目标速度输入，由 fixture 或上游 planner 生成。

trajectory 几何 / yaw / 曲率:
  controller 的参考路径输入；曲率可以由 trajectory 点序列估计或由 Autoware 内部预处理得到。

odometry.twist.twist.linear.x:
  当前实际速度反馈，由 virtual chassis 或实车状态给出。

odometry.pose.pose:
  当前实际位姿反馈，由 virtual chassis 或实车定位结果给出。
```

MPC lateral 和 PID longitudinal 的关系：

```text
MPC lateral:
  使用参考 trajectory、当前 odometry、当前 steering、operation mode 计算 steering_tire_angle。
  trajectory velocity 会影响预测和速度相关权重/限幅，但它不是 MPC 输出。

PID longitudinal:
  使用 trajectory target velocity / acceleration、当前 odometry speed、当前 acceleration 计算 acceleration command。

virtual chassis:
  消费 raw_control_cmd 中的 steering_tire_angle 和 acceleration；
  积分得到下一帧 odometry、steering、acceleration；
  不反向修改 trajectory。
```

CarMaker 的定位：

```text
not required for this plan
not a completion gate
not the expected primary tuning path
```

若后续使用 CarMaker，只能作为另一个 plant fidelity 更高的验证环境；不能把本计划改回“为了接 CarMaker 而调 controller”。

## 3. 控制参数范围

本阶段主要处理以下几类参数。实施时必须区分 **controller 参数** 和 **bench plant 参数**，不要把虚拟底盘私有参数写进 Autoware controller overlay。

### 3.1 车辆参数

车辆参数优先级最高。它们如果错了，prediction horizon 和权重调得再多也只是掩盖问题。

但本 bench 不是用来“调”车辆物理参数的。车辆物理参数应由底盘手册和现有 `vehicle_info.param.yaml` 固定；bench 只验证这些参数进入 controller 和 virtual chassis 后，MPC/PID 参数候选是否能在低速闭环中稳定工作。

底盘手册已经在仓库中：

```text
docs/PIXLOOP线 控 底 盘 HOOKE.pdf
```

实施者不需要再自行查手册。本文直接列出本阶段需要用到的手册数据。

手册第 V 章 `底盘性能参数 Performance Parameter of Chassis` 给出的关键值：

```yaml
# 车身整体参数
dimensions_m: [2.510, 1.700, 0.646]
wheel_base_m: 1.900
front_rear_wheel_track_m: [1.465, 1.465]
front_rear_overhang_m: [0.237, 0.237]
min_turning_radius_m: [3.0, 6.0]
normal_speed_kmh: 40.0
max_speed_no_load_kmh: [60.0, 55.0]
accel_time_0_30_kmh_s_no_load: [4.5, 9.0]

# 转向系统参数
inner_outer_wheel_steering_angle_deg: [30.0, 27.0]
steering_wheel_range_deg: [-450.0, 450.0]
steering_type: "Ackerman"
steering_mode: "Four-wheel/Wedge Steering"
steering_accuracy_deg: 1.0
steering_response_time_ms: 150.0
steering_steady_state_error_deg: 1.0
steering_overshoot_deg_max: 15.0
steering_control_method: "Target Steering Wheel Angle & Angular Velocity"
steering_protocol: "CAN2.0B"
communication_speed_kbps: 500.0

# 制动/动力系统参数
max_braking_pressure_mpa: 7.0
max_pressure_build_up_time_ms: 300.0
max_braking_distance_m_full_load_30kmh: 6.0
power_speed_control_accuracy_kmh: 0.1
power_response_time_ms: 200.0
power_steady_state_error_kmh: 0.2
```

从手册数据得到的 bench 初始参数：

```yaml
wheel_base: 1.9

# 物理内轮最大角约 30 deg = 0.524 rad。现有项目配置 max_steer_angle = 0.488 rad，
# 更保守，第一版继续使用 0.488 rad 作为软件限幅。
max_front_wheel_angle_rad: 0.488
manual_inner_wheel_angle_rad: 0.524
manual_outer_wheel_angle_rad: 0.471

# 手册给 steering response time <=150 ms。若按 0.488 rad / 0.15 s 粗算，
# 物理响应上界约 3.25 rad/s；本地低速 bench 先使用更保守的软件角速度限幅。
manual_response_based_steer_rate_upper_bound_radps: 3.25
max_front_wheel_angle_rate_radps: 1.0

# 手册给 steering wheel range ±450 deg、内轮角 30 deg。
# 初始近似 steering_ratio = 450 / 30 = 15。
steering_ratio: 15.0

# 手册给常规车速 <= 40 km/h；本地调参只做低速。
max_safe_speed_mps: 2.0

# 由手册 response time 作为初始估计来源。
# 这些不是最终实车标定，后续上车可更新。
input_delay: 0.15
vehicle_model_steer_tau: 0.27
actuator_input_delay: 0.15
steering_actuator_tau: 0.15
longitudinal_actuator_tau: 0.20

# 由 0-30 km/h 加速时间推导。30 km/h = 8.33 m/s。
# 4.5 s 版本约 1.85 m/s^2，9 s 版本约 0.93 m/s^2。
# 本地低速 bench 先用保守值。
max_acc_mps2: 1.0
min_acc_mps2: -2.0
max_jerk_mps3: 2.0
min_jerk_mps3: -4.0

control_command_period: 0.05
```

参数归属：

| 参数 | 用途 | 写入位置 |
| --- | --- | --- |
| `wheel_base` | Autoware vehicle model / 虚拟底盘模型 | 现有 `src/autoracer_bringup/config/hooke2/vehicle_info.param.yaml` 已有；虚拟底盘也声明同名 node parameter |
| `max_steer_angle` | Autoware controller 车辆限幅 | 现有 `vehicle_info.param.yaml` 已有 |
| `max_front_wheel_angle_rad` | 虚拟底盘内部限幅命名 | 只作为 `virtual_chassis_node` 参数或文档说明，不写进 controller overlay |
| `steering_ratio` | 方向盘角和前轮角换算 | 只用于后续 chassis interface 或报告说明；当前 Autoware controller overlay 不写该参数 |
| `input_delay`、`vehicle_model_steer_tau` | MPC 模型参数 | 写入 controller candidate overlay |
| `actuator_input_delay` | 虚拟底盘命令延迟 | 只作为 `virtual_chassis_node` 参数；用于模拟 plant，不能通过放宽它换 PASS |
| `steering_actuator_tau`、`longitudinal_actuator_tau` | 虚拟底盘执行器模型 | 只作为 `virtual_chassis_node` 参数 |

如果后续 chassis interface 使用方向盘角，转换关系固定为：

```text
front_wheel_angle = steering_wheel_angle / steering_ratio
```

虚拟底盘和 Autoware controller 内部都使用前轮转角，单位为 rad。

### 3.2 预测窗口

低速初始建议：

```yaml
mpc_prediction_dt: 0.1
mpc_prediction_horizon: 30-50
```

对应 3-5 秒预测窗口。本地阶段不要过度追求最优，只需要达到稳定、不发散、不剧烈抖动。

### 3.3 MPC 权重

只优先调整与低速路径跟踪和转向平滑直接相关的少量参数：

```yaml
mpc_weight_lat_error
mpc_weight_heading_error
mpc_weight_heading_error_squared_vel
mpc_weight_steering_input
mpc_weight_steering_input_squared_vel
mpc_weight_steer_rate
mpc_weight_steer_acc
mpc_weight_terminal_lat_error
mpc_weight_terminal_heading_error
```

调参规则：

- 跟踪误差大：优先增加 lateral / heading error 权重。
- 转向太激进：增加 steering input 和 steering rate 权重。
- 转向振荡：增加 steering rate / steering acceleration 权重，或先检查 delay / tau 是否明显不匹配。
- 入弯太晚：先检查 `input_delay` 和 `vehicle_model_steer_tau`，再考虑窗口和权重。
- 弯道持续内切或外飘：先检查 wheelbase、转向符号、轨迹曲率和 delay，再动权重。

### 3.4 纵向 PID 参数

本 bench 不能只看横向。真实 `controller_node_exe` 同时运行：

```text
lateral_controller_mode = mpc
longitudinal_controller_mode = pid
```

因此虚拟底盘必须消费 raw control 里的纵向输出，并至少验证低速速度跟踪不发散。

Autoware 纵向 PID 默认参数中，本阶段重点关注：

```yaml
delay_compensation_time
kp
ki
kd
max_out
min_out
max_acc
min_acc
max_jerk
min_jerk
max_acc_cmd_diff
```

第一版不做复杂纵向标定，只做低速 guardrail：

- 能从低速初始值跟随 trajectory velocity。
- 加速度、减速度、jerk 不超出本地安全限幅。
- 不出现持续速度发散或频繁 emergency state。
- 若横向调参需要隔离变量，可临时启用 fixed-speed 模式，但 summary 必须标明该场景不验证纵向 PID。

### 3.5 速度/曲率诊断和 Autoware 内置调度

本方案不实现新的速度分段控制器。这里保留“速度分段”的唯一目的，是把测试结果按速度区间归因，判断同一套 Autoware controller overlay 在不同速度下的边界。

速度/曲率相关处理按三层拆开：

1. **trajectory target speed**：上游 planner 或 fixture 给出的目标速度，是 controller 输入。
2. **Autoware controller 内置参数**：MPC/PID 根据 trajectory、当前状态和参数计算 raw control。
3. **monitor guardrail**：bench 只记录和判定是否超出横向加速度、转角、转角速度、acc/jerk 边界。

因此，“速度分段”只作用于 **bench 场景标签、报告维度和 Autoware 既有参数项**，不引入 CarMaker，不在虚拟底盘里修改 trajectory target speed，也不产出多套在线切换配置。

边界必须明确：

```text
trajectory target speed:
  由 fixture 场景直接给定，作为 controller 输入。

MPC lateral:
  使用 trajectory pose/yaw/curvature/velocity 计算 steering。

PID longitudinal:
  跟踪 trajectory.longitudinal_velocity_mps 和 acceleration_mps2。

virtual_chassis:
  只消费 raw_control_cmd，模拟实际 steering/acceleration 响应。
  不生成速度策略，不改 trajectory target speed。
  启动后立即发布初始 odometry / steering / acceleration；没有收到 raw_control_cmd 前使用零 steering / 零 acceleration 保持状态，避免 controller 等状态、plant 等命令的启动死锁。

monitor:
  记录 speed_regime、误差、饱和、加速度、jerk、横向加速度。
```

暂定速度分段：

```text
stop_crawl:
  v_ref <= 0.3 m/s
  只验证静止/极低速状态处理，不作为主要跟踪性能调参区间。

low_speed_tuning:
  0.5 m/s <= v_ref <= 2.0 m/s
  本地 ROS-only 主要收敛范围。

guarded_speed_probe:
  v_ref = 3.0 m/s
  只做 guardrail check，不声明性能可用。
```

暂定参数策略：

```text
stop_crawl:
  重点检查 PID 积分、stopped state、steering convergence，不以 RMS lateral error 作为主指标。

low_speed_tuning:
  使用本文车辆参数、预测窗口和 MPC/PID 权重收敛候选 overlay。

guarded_speed_probe:
  继续使用同一候选 overlay，但以 steering 饱和、steering rate、acc/jerk、横向加速度为主验收项。
```

第一版候选参数可以使用 Autoware 已有速度/曲率相关字段；这些字段属于 Autoware controller 自身参数，不是本文新增的调度框架：

```yaml
steer_rate_lim_dps_list_by_curvature
curvature_list_for_steer_rate_lim
steer_rate_lim_dps_list_by_velocity
velocity_list_for_steer_rate_lim
mpc_weight_heading_error_squared_vel
mpc_weight_steering_input_squared_vel
```

横向加速度包络在本 bench 中只作为 monitor guardrail，不参与速度命令生成：

```text
a_lat = v_actual^2 * abs(curvature_reference)
max_lat_acc_guardrail_mps2: 1.5
```

如果要让车辆跑更快，优先路径不是自己发明速度调度，而是：

```text
1. 上游 trajectory 先给出符合曲率/横向加速度约束的 target speed。
2. PID longitudinal 跟踪该 target speed。
3. MPC lateral 使用同一条 trajectory 和 velocity 做横向控制。
4. monitor 用 a_lat、steering、steer_rate、acc/jerk 判断边界。
5. 只有 Autoware 已有参数无法覆盖时，才另开设计讨论；本计划内不做。
```

## 4. 新 bench 数据流

新 bench 必须使用真实 race controller，不写 mock controller 函数。

```text
trajectory publisher
  -> /control_bench/planning/trajectory

operation mode publisher
  -> /control_bench/system/operation_mode/state

virtual_chassis_node
  -> /control_bench/localization/kinematic_state
  -> /control_bench/vehicle/status/steering_status
  -> /control_bench/localization/acceleration

controller_node_exe
  -> /control_bench/autoracer/control/raw_control_cmd

virtual_chassis_node
  subscribes raw_control_cmd
  uses lateral steering command and longitudinal command
  integrates next vehicle state

closed_loop_monitor
  records tracking metrics and writes closed_loop_summary.json
```

虚拟底盘应订阅 raw control，而不是 final control。

原因：

- `raw_control_cmd` 反映 controller 的真实输出。
- `command_gate` 可能因为 safety contract 进行限幅或 stop。
- 通过 final control 调 MPC，会把控制器行为和 gate policy 混在一起。

第一版闭环调参 bench 可以不启动 `command_gate`。`command_gate` 已经由旧 contract bench 覆盖。

bench 的节点职责：

| 节点 | 职责 |
| --- | --- |
| `control_closed_loop_fixture_publisher` | 发布 deterministic trajectory 和 transient-local operation mode |
| `race_control.launch.py` included controller | 启动真实 `controller_node_exe`，加载 MPC/PID 参数 |
| `virtual_chassis_node` | 订阅 raw control，模拟转向/纵向执行器和车辆运动，发布反馈状态 |
| `control_closed_loop_monitor` | 订阅轨迹、raw control、虚拟车辆状态，计算误差和 PASS/FAIL |

数据闭环是：

```text
trajectory + operation_mode + virtual vehicle state
  -> controller_node_exe
  -> raw_control_cmd
  -> virtual_chassis_node
  -> new odometry / steering / accel
  -> controller_node_exe
```

## 5. 虚拟底盘模型

虚拟底盘不是 controller，而是被控对象 plant。它接收 controller 输出，并生成下一帧车辆状态。

上一版计划只写横向-only，是过窄的。修订后第一版虚拟底盘包含横向和纵向两部分：

- 横向：转向执行器 + 运动学自行车模型。
- 纵向：加速度执行器 + 速度积分。
- 调横向时允许 fixed-speed mode 隔离变量，但默认 bench 应实现完整横纵向状态。

状态：

```text
x
y
yaw
v
delta_actual
a_actual
```

输入：

```text
delta_cmd = raw_control_cmd.lateral.steering_tire_angle
v_cmd = raw_control_cmd.longitudinal.velocity
acc_cmd = raw_control_cmd.longitudinal.acceleration
```

`v_cmd` 只用于 monitor 记录和速度误差分析。虚拟底盘不能把实际速度直接设置为 `v_cmd`；实际速度只由 `acc_cmd` 经过纵向执行器模型积分得到。

不要依赖 `raw_control_cmd.longitudinal.jerk` 作为 plant 输入。当前纵向 PID 源码主要写入 `velocity` 和 `acceleration`；bench 的 jerk 限制应由虚拟底盘根据连续两帧实际加速度变化自行计算和约束。

参数：

```text
wheel_base
max_steer
max_steer_rate
steer_tau
actuator_input_delay
max_speed
max_acc
min_acc
max_jerk
min_jerk
acc_tau
dt
```

更新逻辑：

```text
delayed_delta_cmd = delay_buffer(delta_cmd, actuator_input_delay)
delayed_acc_cmd = delay_buffer(acc_cmd, actuator_input_delay)

# steering actuator
delta_target = clamp(delayed_delta_cmd, -max_steer, max_steer)
delta_desired = delta_actual + dt / steer_tau * (delta_target - delta_actual)
delta_step = clamp(
  delta_desired - delta_actual,
  -max_steer_rate * dt,
  max_steer_rate * dt
)
delta_actual_next = delta_actual + delta_step

# longitudinal actuator
acc_target = clamp(delayed_acc_cmd, min_acc, max_acc)
acc_desired = a_actual + dt / acc_tau * (acc_target - a_actual)
acc_step = clamp(
  acc_desired - a_actual,
  min_jerk * dt,
  max_jerk * dt
)
a_actual_next = a_actual + acc_step
v_next = clamp(v + a_actual_next * dt, 0.0, max_speed)

# bicycle kinematics
x_next = x + v_next * cos(yaw) * dt
y_next = y + v_next * sin(yaw) * dt
yaw_next = yaw + v_next / wheel_base * tan(delta_actual_next) * dt
```

fixed-speed mode 只作为横向隔离测试开关：

```text
if fixed_speed_mode:
  v_next = fixed_speed
  a_actual_next = 0.0
```

当 fixed-speed mode 开启时，monitor 必须写：

```json
"longitudinal_validated": false
```

默认闭环调参场景应使用 longitudinal actuator，让 PID 输出参与车辆速度更新。

## 6. 场景集合

场景分三类：

- `smoke`：短时长，检查 controller wiring、plant feedback、raw control、metrics 是否工作；可用于快速调参反馈，但不能单独证明完整路线可用。
- `full_validation`：按路径进度跑完整轨迹，正常结束条件是 `trajectory_progress_ratio >= 0.98` 或 `trajectory_complete`；用于证明某组参数能跑完一条路线。
- `diagnostic / sweep`：按速度、曲率、初始误差或 segment 输出矩阵，用于归因，不自动推出新控制策略。

默认闭环场景必须使用 raw control 中的 longitudinal output 更新速度，用于同时观察 PID 速度跟踪。只有为了隔离 MPC steering 行为时，才允许临时使用 fixed-speed mode，并且 summary 必须标明 `longitudinal_validated=false`。

### 6.1 直线横向偏差收敛

参考轨迹：

```text
y_ref = 0
yaw_ref = 0
v_ref = fixed_speed
```

初始状态：

```text
x = 0
y = -0.5
yaw = 0
delta_actual = 0
```

期望：

- 横向误差向 0 收敛。
- 航向保持有界。
- steering 不持续饱和。
- 若使用完整横纵向模式，速度误差保持有界。

### 6.2 直线航向偏差收敛

初始状态：

```text
x = 0
y = 0
yaw = +5 deg
delta_actual = 0
```

期望：

- heading error 向 0 收敛。
- lateral error 不发散。

### 6.3 固定半径左弯

参考轨迹：

```text
R = 20 m
v_ref = fixed_speed
```

理论稳态前轮角：

```text
delta_ref = atan(wheel_base / R)
```

当 `wheel_base = 1.9`、`R = 20` 时：

```text
delta_ref ~= 0.095 rad
```

期望：

- lateral error 有界。
- `delta_actual` 接近合理稳态值。
- 不持续内切或外飘。

### 6.4 S 弯

参考轨迹：

```text
y = A * sin(2*pi*x / wavelength)
```

初始建议：

```text
A = 1.0 m
wavelength = 30-40 m
v_ref = 1.0 m/s
```

期望：

- steering 符号切换平滑。
- 没有高频 steering chatter。
- 曲率变号时 lateral error 仍保持有界。

### 6.5 纵向速度阶跃

参考轨迹保持直线，trajectory velocity 从低速切换到目标低速：

```text
v_ref: 0.5 m/s -> 1.0 m/s -> 2.0 m/s
```

初始状态：

```text
x = 0
y = 0
yaw = 0
v = 0.5
delta_actual = 0
a_actual = 0
```

期望：

- `v` 向 `v_ref` 收敛。
- acceleration / jerk 不超限。
- 不触发持续 emergency 或 stopped 状态。
- 不影响直线横向稳定性。

### 6.6 速度分段约束验证

同一条直线和 20m 圆弧，分别以以下速度运行：

```text
v_ref = 0.5 m/s
v_ref = 1.0 m/s
v_ref = 2.0 m/s
v_ref = 3.0 m/s
```

同时记录每个组合的：

```text
speed_regime
reference_velocity
actual_velocity
estimated_lateral_acceleration = v_actual^2 * abs(curvature_reference)
```

期望：

- `0.5-2.0 m/s` 内跟踪误差满足本地低速调参 PASS 标准。
- `3.0 m/s` 只做 guarded check：不发散、不持续饱和、不超过横向加速度 guardrail。
- 速度越高，steering rate 和 steering saturation 约束越严格。
- 不能通过高 steering rate 强行压误差。

### 6.7 full-validation 场景

当前 V1.5 harness 已有以下完整路线场景：

| 场景 | 类型 | 目的 |
| --- | --- | --- |
| `straight_120m_v1` | full_validation | 直线长距离稳定性和 completion gate |
| `arc_r20_90deg_v1` | full_validation | 20m 半径 90 度圆弧稳定跟踪 |
| `s_curve_100m_v1` | full_validation | S 弯完整路径稳定性 |
| `speed_step_120m_v1` | full_validation | 速度阶跃下的纵向 PID 和横向稳定性 |

full-validation 的 PASS 语义不同于短 smoke：

```text
必须完成 trajectory progress。
必须无 NaN/Inf、无 raw-control timeout、无 hard lateral error。
必须不超 steering、steer_rate、acc、jerk、lat_acc guardrail。
RMS / terminal lateral error 作为报告指标和调参依据，但不要用 near-zero initial error 的单一比较制造 gate-sensitive failure。
```

## 7. 指标和 PASS 标准

monitor 必须写出：

```text
logs/control_closed_loop/<timestamp>/closed_loop_summary.json
```

必要字段：

```json
{
  "stage": "control_closed_loop_tuning_ros_only",
  "result": "PASS|FAIL",
  "scenario": "straight_lateral_offset",
  "scenario_type": "smoke|full_validation",
  "controller_under_test": "autoware_trajectory_follower_node/controller_node_exe",
  "data_source": "virtual_chassis",
  "end_condition": "duration_complete|trajectory_complete|timeout|failure",
  "path_length_m": 0.0,
  "progress_distance_m": 0.0,
  "trajectory_progress_ratio": 0.0,
  "completed_trajectory": false,
  "does_not_validate": [
    "CarMaker closed-loop",
    "Stage B planner",
    "real vehicle calibration",
    "race performance"
  ],
  "metrics": {
    "rms_lateral_error_m": 0.0,
    "max_abs_lateral_error_m": 0.0,
    "initial_abs_lateral_error_m": 0.0,
    "final_abs_lateral_error_m": 0.0,
    "rms_heading_error_rad": 0.0,
    "rms_velocity_error_mps": 0.0,
    "max_abs_velocity_error_mps": 0.0,
    "max_abs_acc_actual_mps2": 0.0,
    "max_abs_jerk_actual_mps3": 0.0,
    "max_abs_steer_cmd_rad": 0.0,
    "max_abs_steer_actual_rad": 0.0,
    "max_abs_steer_rate_radps": 0.0,
    "max_estimated_lat_acc_mps2": 0.0,
    "steer_cmd_saturation_ratio": 0.0,
    "steer_rate_saturation_ratio": 0.0,
    "speed_regime": "stop_crawl|low_speed_tuning|guarded_speed_probe",
    "reference_velocity_mps": 0.0,
    "actual_velocity_mps": 0.0,
    "reference_curvature_1pm": 0.0,
    "longitudinal_validated": true,
    "oscillation_score": 0.0
  },
  "segments": []
}
```

通用 hard guardrail：

```text
max_abs_steer_actual_rad <= max_steer
max_abs_acc_actual_mps2 <= max_abs_configured_acc
max_abs_jerk_actual_mps3 <= max_abs_configured_jerk
max_estimated_lat_acc_mps2 <= max_lat_acc_guardrail_mps2
steer_cmd_saturation_ratio <= 0.20
steer_rate_saturation_ratio <= 0.20
no sustained oscillation
no NaN/Inf in state or commands
```

调参用 soft criteria：

```text
rms_lateral_error_m <= 0.30
rms_velocity_error_mps <= 0.30 when longitudinal_validated == true
final_abs_lateral_error_m <= max(initial_abs_lateral_error_m, terminal_error_floor_m)
terminal_error_floor_m 默认 0.03 m，仅用于避免 near-zero initial error 场景产生无意义的 gate-sensitive failure
```

full-validation 额外要求：

```text
completed_trajectory == true
trajectory_progress_ratio >= 0.98
end_condition == trajectory_complete
```

这些是本地工程 guardrail 和调参证据，不是最终比赛性能指标。若 hard guardrail 通过、RMS 很低、但 terminal gate 因 near-zero initial error 失败，报告必须标记为 `gate_sensitive` 或 remaining diagnostic，不能继续盲目加大 MPC 权重。

## 8. 参数输出方式

调参输出先放到独立 overlay：

```text
src/autoracer_control/config/race_controller.closed_loop_candidate.param.yaml
```

不要直接覆盖现有 `race_controller.param.yaml`。

候选文件只写 **Autoware controller 已声明并消费的参数**，且只写相对 Autoware 默认值或项目车辆默认值有意变更的参数，禁止复制整份 external 默认 YAML。

为避免非必要 topic，candidate overlay 应关闭 Autoware MPC debug trajectory publisher：

```yaml
publish_debug_trajectories: false
```

这只关闭 MPC 可配置的 debug trajectory 输出；`controller_node_exe` 和 PID/MPC 内部固定存在的 diagnostic/debug publisher 不作为 bench 接口依赖，也不作为本文新增 topic。

不要在 controller candidate overlay 里写：

```yaml
steering_ratio
max_front_wheel_angle_rad
steering_actuator_tau
longitudinal_actuator_tau
max_lat_acc_guardrail_mps2
```

这些是 bench plant / report / 后续 chassis interface 参数，不是当前 `controller_node_exe` 的调参产物。

每个候选值建议带来源注释：

```yaml
# seeded from manual response time; needs vehicle validation
input_delay: 0.15
vehicle_model_steer_tau: 0.27

# tuned: ROS-only closed-loop tuning; low-speed candidate
mpc_weight_lat_error: 1.5
```

辅助产物：

```text
logs/control_closed_loop/<timestamp>/closed_loop_summary.json
logs/control_closed_loop/<timestamp>/tuning_report.md
```

`tuning_report.md` 记录每轮参数、场景结果、失败原因和调整理由。最终结论必须说明这份 controller overlay 在各速度段的表现，而不是产出多套在线切换参数。

当前已有候选 overlay：

```text
src/autoracer_control/config/race_controller.closed_loop_candidate.param.yaml
```

它的语义是：

```text
Autoware MPC/PID low-speed candidate overlay
not final vehicle calibration
not multi-speed online switching config
not CarMaker-tuned config
```

`tuning_report.md` 已记录 5 轮 MPC weight tuning 后仍有 `s_curve` terminal gate remaining failure。因此本文后续执行不应继续盲调权重；应先把验收门槛、full-validation 覆盖和 trace-based failure attribution 做清楚，再决定是否继续调整 Autoware controller 参数。

## 8.1 调参执行闭环

AI 执行本文时不能只写一份 seed 参数然后结束。必须按下面闭环收敛：

调参只允许修改 `race_controller.closed_loop_candidate.param.yaml` 中的 controller 参数。`virtual_chassis_node` 参数是测试 plant 定义，除非发现与本文手册数据或单位明显不一致，否则不能通过放宽 plant 参数让场景 PASS。

```text
1. 生成初始 candidate overlay
   来源：Autoware 默认值 + 现有项目车辆参数 + 本文手册估计。

2. 运行最小场景集合
   straight_lateral_offset
   straight_heading_offset
   constant_radius_left
   s_curve
   longitudinal_speed_step
   speed_regime_sweep

3. 读取 closed_loop_summary.json
   判断 fail 原因：横向误差、航向误差、速度误差、转角饱和、转角速度饱和、acc/jerk 超限、振荡。

4. 一次只调整一类 controller 参数
   MPC 内部模型参数（如 `input_delay`、`vehicle_model_steer_tau`） -> prediction window -> MPC 权重 -> PID 参数。

5. 重跑失败场景和至少一个已通过场景
   防止修一个场景破坏另一个场景。

6. 记录调整理由
   每轮写入 tuning_report.md，不允许无依据改参数。

7. 停止条件
   low_speed_tuning 场景全部 PASS；
   guarded_speed_probe 不发散、不持续饱和、不超 guardrail；
   或达到 5 轮仍失败，报告 remaining failure，不伪造收敛。
```

当前记录已经达到“5 轮仍失败”的停止条件。后续若继续执行，不能从“继续加大/减小 MPC 权重”开始，而必须先选择一个窄目标：

```text
1. 修正 near-zero initial error 下的 terminal gate 语义，并重跑 s_curve + 至少两个回归场景；
2. 或保持 gate 不变，做一因素实验：prediction window、input_delay、vehicle_model_steer_tau 三者一次只动一类；
3. 或转入 full-validation 覆盖：arc_r20_90deg_v1、s_curve_100m_v1、speed_step_120m_v1。
```

任一路径都仍然只允许修改 Autoware controller 参数或 monitor/report 的验收语义，不允许引入自研 controller 或 CarMaker 依赖。

允许的调参动作保持最小：

| 现象 | 优先动作 |
| --- | --- |
| 入弯晚、外飘 | 先检查 `input_delay`、`vehicle_model_steer_tau`，再看 horizon |
| 横向误差大但转向不饱和 | 增加 `mpc_weight_lat_error` / heading 相关权重 |
| 转向过猛或频繁饱和 | 增加 steering input / steer rate 权重，或降低 steer rate limit |
| 高频抖动 | 增加 `mpc_weight_steer_rate` / `mpc_weight_steer_acc`，检查 delay/tau |
| 速度误差持续大 | 调整 PID `kp/ki/kd`，先保持 acc/jerk 限幅不放大 |
| acc/jerk 超限 | 调整 PID limit / PID gains；不能放宽虚拟底盘 limit 换 PASS |
| 只在 `3.0 m/s` 表现差 | 不强行调到 PASS，只报告 guarded probe 结果 |

## 9. 实施任务

### Task 0: 源码合同核验和 smoke 准入

**目的：** 防止实现者在没搞清 controller 真实 I/O 的情况下继续堆 bench。

**步骤：**

1. 核对 `race_control.launch.py` 的 remap 和参数加载顺序，确认 controller 输入输出仍为 2.1 节列出的 5 输入 + 1 输出。
2. 核对 `controller_node.cpp` 的 `processData()`，确认 trajectory、odometry、steering、acceleration、operation mode 都是进入控制计算前的必要输入。
3. 核对 MPC lateral 使用：
   - `current_trajectory`
   - `current_odometry.pose.pose`
   - `current_odometry.twist.twist.linear.x`
   - `current_steering.steering_tire_angle`
   - `current_operation_mode`
4. 核对 PID longitudinal 使用：
   - `current_trajectory.longitudinal_velocity_mps`
   - `current_trajectory.acceleration_mps2`
   - `current_odometry.pose.pose`
   - `current_odometry.twist.twist.linear.x`
   - `current_accel.accel.accel.linear.x`
   - `current_operation_mode`
5. 在 `tuning_report.md` 的第一节记录本次源码核验结果和 commit/worktree 状态；若源码合同与本文不一致，先修本文，不继续实现。

### Task 1: 添加 closed-loop bench 静态合同测试

**文件：**

- 新增：`src/autoracer_control/test/test_control_closed_loop_bench_contract.py`

**步骤：**

1. 写测试，断言未来 launch 文件启动：
   - `race_control.launch.py`
   - `control_closed_loop_fixture_publisher`
   - `virtual_chassis_node`
   - `control_closed_loop_monitor`
2. 断言 launch 文件不包含 CarMaker 相关启动项。
3. 断言 plant 订阅 `/control_bench/autoracer/control/raw_control_cmd`，不是 final control。
4. 断言 monitor 写出 `closed_loop_summary.json`。
5. 断言没有新增自定义 msg / srv / action 依赖。
6. 断言 bench 自建节点不使用 2.1 节主接口之外的 `/control_bench/**` 控制链路 topic。
7. 允许 Autoware controller 内部已有 debug publisher 存在，但不得把这些 debug topic 作为 virtual chassis 或 fixture 的输入输出依赖。
8. 运行：

```bash
python3 -m pytest src/autoracer_control/test/test_control_closed_loop_bench_contract.py -q
```

实现前预期：失败，因为文件尚不存在。

### Task 2: 添加纯 Python 虚拟底盘模型和单测

**文件：**

- 新增：`src/autoracer_control/autoracer_control/virtual_chassis_model.py`
- 新增：`src/autoracer_control/test/test_virtual_chassis_model.py`

**步骤：**

1. 测 steering limit。
2. 测 steering rate limit。
3. 测一阶转向响应。
4. 测 `actuator_input_delay`。
5. 测 acceleration limit。
6. 测 jerk limit。
7. 测速度积分，确认实际速度由 acceleration 积分得到，不直接跳到 `v_cmd`。
8. 测直线积分。
9. 测正转角会增加 yaw。
10. 实现最小 dataclass 和 `step()`。
11. 运行：

```bash
python3 -m pytest src/autoracer_control/test/test_virtual_chassis_model.py -q
```

实现后预期：通过。

### Task 3: 添加 `virtual_chassis_node`

**文件：**

- 新增：`src/autoracer_control/autoracer_control/virtual_chassis_node.py`
- 修改：`src/autoracer_control/setup.py`
- 如新增 runtime dependency，修改：`src/autoracer_control/package.xml`

**步骤：**

1. 订阅 raw control topic。
2. 维护 input delay buffer。
3. 固定 timer rate 调用模型 step。
   - 启动后即按 `initial_*` 状态发布反馈。
   - 尚未收到 raw control 时使用零 steering / 零 acceleration 命令。
4. 发布：
   - `nav_msgs/msg/Odometry`
   - `autoware_vehicle_msgs/msg/SteeringReport`
   - `geometry_msgs/msg/AccelWithCovarianceStamped`
5. 默认使用 raw control 的 lateral steering 和 longitudinal acceleration 更新车辆；longitudinal velocity 只记录，不直接覆盖实际速度，不依赖 longitudinal jerk。
6. 提供 `fixed_speed_mode` 参数，仅用于横向隔离测试。
7. 声明初始状态参数：
   - `initial_x`
   - `initial_y`
   - `initial_yaw`
   - `initial_v`
   - `initial_delta`
   - `initial_a`
8. 声明虚拟底盘参数：
   - `wheel_base`
   - `max_steer`
   - `max_steer_rate`
   - `steer_tau`
   - `actuator_input_delay`
   - `max_speed`
   - `max_acc`
   - `min_acc`
   - `max_jerk`
   - `min_jerk`
   - `acc_tau`
   - `dt`
9. 不订阅 final control。

### Task 4: 添加轨迹和 operation mode publisher

**文件：**

- 新增：`src/autoracer_control/autoracer_control/control_closed_loop_fixture_publisher.py`
- 修改：`src/autoracer_control/setup.py`

**步骤：**

1. 发布 `autoware_planning_msgs/msg/Trajectory`。
2. 用 transient local QoS 发布 `autoware_adapi_v1_msgs/msg/OperationModeState`。
3. 实现场景：
   - `straight_lateral_offset`
   - `straight_heading_offset`
   - `constant_radius_left`
   - `s_curve`
   - `longitudinal_speed_step`
   - `speed_regime_sweep`
4. 轨迹生成必须 deterministic。
5. 每个 `TrajectoryPoint` 必须显式设置以下字段，且值必须 finite：
   - `pose.position.x/y/z`
   - `pose.orientation.x/y/z/w`
   - `time_from_start`
   - `longitudinal_velocity_mps`
   - `lateral_velocity_mps`
   - `acceleration_mps2`
   - `heading_rate_rps`
   - `front_wheel_angle_rad`
   - `rear_wheel_angle_rad`
6. 轨迹点数和长度必须满足 controller 计算：
   - MPC 至少 3 点，PID 至少 2 点；
   - 调参场景必须提供足够长、足够密的轨迹，不能用最小点数代替预测窗口轨迹。
7. fixture 只生成 trajectory 和 operation mode，不发布虚拟车辆状态。
8. 初始车辆状态由 `virtual_chassis_node` 参数给出，launch 根据 scenario 设置。

### Task 5: 添加 closed-loop monitor

**文件：**

- 新增：`src/autoracer_control/autoracer_control/control_closed_loop_monitor.py`
- 修改：`src/autoracer_control/setup.py`

**步骤：**

1. 订阅 odometry、steering、acceleration、raw control、trajectory。
2. 基于最近轨迹点计算 lateral error 和 heading error。
   - `dx = x_vehicle - x_ref`
   - `dy = y_vehicle - y_ref`
   - `lateral_error = -sin(yaw_ref) * dx + cos(yaw_ref) * dy`
   - `heading_error = wrap_to_pi(yaw_vehicle - yaw_ref)`
3. 计算 velocity error、acceleration、jerk。
4. 计算 estimated lateral acceleration。
   - 曲率用相邻 trajectory points 估计；直线为 0。
   - `estimated_lateral_acceleration = v_actual^2 * abs(curvature_reference)`
5. 计算 speed regime。
6. 计算 saturation ratio 和最大 steering rate。
7. 检查 NaN/Inf。
8. 写出 `closed_loop_summary.json`。
9. 写出或追加 `tuning_report.md` 所需原始 metrics。
10. 声明 monitor 参数：
   - `max_lat_acc_guardrail_mps2`
11. 只有 PASS 标准满足时 exit code 为 0。

### Task 6: 添加 closed-loop launch

**文件：**

- 新增：`src/autoracer_control/launch/control_closed_loop_bench.launch.py`
- 修改：`src/autoracer_control/setup.py`

**步骤：**

1. include `race_control.launch.py`。
2. 启动 fixture publisher。
3. 启动 virtual chassis。
4. 启动 monitor。
5. 使用 `/control_bench/**` topic。
6. 不启动 `command_gate`。
7. 不启动 CarMaker。
8. 将 `race_param_file` 指向 `race_controller.closed_loop_candidate.param.yaml`。
9. launch 根据 scenario 设置 `virtual_chassis_node` 的初始状态和 fixed-speed 参数。
10. launch 将 `max_lat_acc_guardrail_mps2` 传给 monitor，不写入 controller overlay。
11. 使用 `RegisterEventHandler(OnProcessExit(... Shutdown ...))` 在 monitor 退出后关闭 bench。
12. 不发布、不订阅 `/control_bench/localization/pose_with_covariance`；位姿通过 `/control_bench/localization/kinematic_state` 的 `Odometry.pose.pose` 提供。
13. 默认不发布 TF，因为当前 MPC/PID 控制计算路径未发现 TF lookup；若 smoke test 证明 runtime 需要 TF，必须先记录具体缺失日志和源码调用路径，再补标准 TF，且不得把 TF 当成新的控制接口。
14. 文档命令使用独立 `ROS_DOMAIN_ID`。

### Task 7: 添加候选参数 overlay

**文件：**

- 新增：`src/autoracer_control/config/race_controller.closed_loop_candidate.param.yaml`

**步骤：**

1. 只写有意覆盖项。
2. 只写本文明确属于 controller 的初始估计和待收敛参数：
   - `input_delay`
   - `vehicle_model_steer_tau`
   - steer rate limits
   - PID longitudinal acceleration / jerk output limits
   - PID gains / limits
   - `publish_debug_trajectories: false`
3. prediction window 保持 3-5 秒范围。
4. 只添加让本地闭环稳定所需的最小 MPC/PID overrides。
5. 不使用离散硬切换参数文件；速度分段只作为验证维度。
6. 不把 `steering_ratio`、`max_front_wheel_angle_rad`、虚拟底盘 tau 或 guardrail 写入 controller overlay。

### Task 8: 场景验证和参数调优闭环

**命令：**

```bash
cd /opt/ipg/carmaker/linux64-15.1/autoracer_hooke
python3 -m pytest src/autoracer_control/test/test_virtual_chassis_model.py -q
python3 -m pytest src/autoracer_control/test/test_control_closed_loop_bench_contract.py -q
```

build/install 后：

```bash
colcon build --packages-select autoracer_control --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=logs/control_closed_loop/ros2_launch_logs
```

```bash
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=straight_lateral_offset
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=straight_heading_offset
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=constant_radius_left
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=s_curve
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=longitudinal_speed_step
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=speed_regime_sweep
```

full-validation 覆盖：

```bash
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=straight_120m_v1
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=arc_r20_90deg_v1
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=s_curve_100m_v1
ROS_DOMAIN_ID=98 ros2 launch autoracer_control control_closed_loop_bench.launch.py scenario:=speed_step_120m_v1
```

证据查询：

```bash
rg -n "stage|result|scenario|scenario_type|end_condition|trajectory_progress_ratio|completed_trajectory|speed_regime|rms_lateral_error_m|rms_velocity_error_mps|max_abs_steer|max_abs_acc|max_estimated_lat_acc|saturation|does_not_validate" logs/control_closed_loop
```

调参循环：

1. 先运行全部场景，生成 baseline summary。
2. 若有失败，按 `8.1 调参执行闭环` 调整 candidate overlay。
3. 每轮最多改一类参数。
4. 每轮必须在 `tuning_report.md` 记录：
   - changed parameters
   - reason
   - before metrics
   - after metrics
   - remaining failures
5. 最多 5 轮。5 轮后仍失败时停止并报告，不继续盲调。
6. 若报告已经显示达到 5 轮停止条件，后续只能执行 gate/trace/full-validation 诊断，不得继续普通权重盲调。

## 10. 完成标准

满足以下条件才算完成：

- `tuning_report.md` 已按 2.2 节记录每个 Phase 的 commands、files、evidence、pass/fail 和 next-phase gate。
- `tuning_report.md` 已记录 Task 0 源码合同核验结果，且 bench I/O 与 `controller_node_exe` 真实输入输出一致。
- 现有 `race_control_bench` 仍然只是 contract bench，没有被扩展成调参工具。
- 新增独立 closed-loop tuning bench，且不使用 CarMaker。
- 虚拟底盘根据 raw controller output 积分车辆状态。
- 至少六个最小场景生成 `closed_loop_summary.json`。
- 至少四个 full-validation 场景具备 path progress / completion / segment metrics 字段；未跑完的场景必须在 `tuning_report.md` 说明未交付原因。
- 候选参数写入独立 controller overlay 文件，且不是多套分段在线切换参数。
- 候选参数只包含 Autoware controller 已声明并消费的参数；bench plant、monitor guardrail、chassis interface 参数不得混入 controller overlay。
- 生成 `tuning_report.md`，说明参数来源、每轮调整理由、各速度段表现和 remaining failures。
- 最终报告明确说明：结果不验证 CarMaker、Stage B planner、实车标定或 race performance；真实最终标定仍需要低速实车开始逐步确认。

当前状态不满足“参数可交付”：

```text
原因：Phase 5 达到 5 轮调参上限后，s_curve 仍有 terminal gate remaining failure；
V1.5 harness 已经升级，但只证明验证平台可用，不证明 controller overlay 已收敛；
因此当前可交付的是 bench + report + candidate overlay，不是最终控制参数。
```
