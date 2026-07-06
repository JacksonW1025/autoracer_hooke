# 官方 Autoware 迁移评估备忘

用途：记录当前本地 upper stack 与官方 Autoware Core/Universe planning/control/gate 的差异、迁移前提和风险。非用途：不作为当前 RC 主链路，不指导 RC 单独替换上层算法。

## 使用前提

只有满足下面条件，才进入 official 迁移评估：

- RC 已跑通 sensing -> localization -> planning -> control -> gate -> vehicle adapter。
- 已有 bag/log，能说明当前 upper stack 的实际问题。
- Hooke 和 RC 都同意作为共同 upper stack 迁移，不做 RC 单独替换。
- 迁移候选来自当前 `autoracer.repos` pin，不能混拉最新版。

## 当前共同实现

| 层 | 当前实现 | 说明 |
| --- | --- | --- |
| Planning | `autoracer_planning/lanelet_route_planner.py` | 直接从 OSM 和 `/goal_pose` 生成 `/planning/trajectory`。 |
| Control | `autoracer_control/pure_pursuit_controller.py` | 消费 `/planning/trajectory`、定位 pose、vehicle velocity，输出 raw control。 |
| Gate | `autoracer_safety/command_gate.py` | 做 enable、timeout、限幅、gear/hazard/turn 输出。 |
| Vehicle adapter | Hooke2 CAN 或 RC UART | 平台适配层，长期允许不同。 |

## 官方候选与缺口

| 层 | 官方候选 | 不能直接替换的原因 |
| --- | --- | --- |
| Mission route | `autoware_mission_planner` | 需要 `LaneletMapBin`、`/localization/kinematic_state`、operation mode，输出 `LaneletRoute`，不是 final trajectory。 |
| Path generation | `autoware_path_generator` | 需要 route + odometry + vector map，输出 `PathWithLaneId`，当前本地 planner 没有这一级。 |
| Velocity profile | `autoware_velocity_smoother` | 需要 reference trajectory、kinematic state、operation mode、acceleration、velocity limit。 |
| Control | `autoware_trajectory_follower_node` + MPC/PID 或 pure pursuit plugin | 需要稳定 `nav_msgs/Odometry`、vehicle info 和控制器参数。 |
| Gate | `autoware_vehicle_cmd_gate` 或 `autoware_control_command_gate` | 需要 steering status、engage、operation mode、gate mode、heartbeat、emergency inputs。 |

## 迁移原则

- 先验证当前 upper stack，再讨论替换。
- 若替换，Hooke/RC 共同替换；RC 分支不得单独切换上层算法。
- 先做 isolated launch 和构建闭包，不直接改默认 `track.launch.py`。
- 官方 gate 属于安全边界，缺少 operation mode、engage、heartbeat 时不能设为默认。
- 当前本地 upper stack 在 official-chain 完成 ARM 侧验证前必须保留为回退。
