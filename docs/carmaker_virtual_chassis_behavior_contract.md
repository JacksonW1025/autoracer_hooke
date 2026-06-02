# CarMaker Control Command Adapter MVP Plan

本文定义当前阶段要解决的问题：让 AutoRacer 经过 `command_gate` 后输出的
`/control/command/control_cmd`，在 CarMaker 中被转换成车辆可执行的油门、刹车、
转向等控制量，并让仿真车在行为上产生正确响应。

当前任务不是重建真实底盘，不是做 CAN 协议仿真，也不是把 CarMaker 改成完全等价的
真车动力学模型。当前任务是做一个最小可用的 CarMaker 侧 control command adapter。

## 1. 核心共识

AutoRacer 侧输出形式已经确定，不允许为了适配 CarMaker 修改 AutoRacer 输出消息、
`command_gate` 语义或话题结构。

真车链路中，`command_gate` 之后的控制命令会继续进入底盘接口，最终通过 CAN 被底盘执行。
CarMaker 链路中，不需要模拟 CAN，也不需要完整实现真车底盘接口；分叉点应放在
`command_gate` 之后：

```text
AutoRacer control
  -> /autoracer/control/raw_control_cmd
  -> command_gate
  -> /control/command/control_cmd
  -> CarMaker control command adapter
  -> CarMaker VehicleControl / vehicle behavior
```

验收关注的是行为闭环：

- 给定速度目标，CarMaker 车辆速度应朝该目标变化并稳定在合理误差内。
- 给定停止或降速目标，CarMaker 车辆应减速或停车。
- 给定转向目标，CarMaker 车辆应产生方向正确、量级合理的转向响应。
- AutoRacer 不应为了 CarMaker 做额外适配。

## 2. 不可破坏边界

必须遵守：

1. 不修改 AutoRacer controller 的输出格式。
2. 不修改 `command_gate` 的输入/输出契约。
3. 不要求 AutoRacer 输出 CarMaker 专用字段。
4. 不要求 AutoRacer 输出 `gear_cmd`、油门、刹车或方向盘角。
5. CarMaker 侧只消费当前任务需要的 command，不因为真车 CAN 链路存在其他消息就强行全量对接。
6. 不通过直接写 `Vehicle.v`、车辆位姿或其他状态量来“伪造”响应。
7. 所有改动优先限制在 CarMaker 工程侧，尤其是 ROS2 bridge 到 `VehicleControl` 的适配逻辑。

明确不做：

- 不做 CAN 协议级仿真。
- 不做完整电驱、电池、制动液压、底盘控制器高保真建模。
- 不把真实底盘所有辅助功能搬进 CarMaker。
- 不为了通过测试修改上游规划、控制或安全门。

## 3. 当前链路和问题位置

当前代码已有基本链路：

- `ROS2Bridge.cpp` 接收并缓存 `/control/command/control_cmd`。
- `User.cpp` 中的 `User_VehicleControl_Calc()` 把 command 转换为 CarMaker 的车辆控制量。
- 当前实现是低速 Stage A 可用的硬编码转换。

当前主要问题不是 ROS2 数据链路，而是 command 到车辆行为的转换太粗糙：

- 速度上限硬编码偏低，不能覆盖任务所需的常规速度区间。
- 油门/刹车映射是简单低速闭环，未按底盘手册给出合理默认边界。
- 转向映射应明确使用常规转向/阿克曼转向，不引入四轮转向或楔形转向。
- 测试标准需要能覆盖连续速度区间，而不是只看某一个示例速度。

## 4. `command_gate` 相关输入输出边界

本任务只要求 CarMaker adapter 消费：

```text
/control/command/control_cmd
```

该消息是 `command_gate` 安全检查后放行的最终控制命令，消息格式与
`/autoracer/control/raw_control_cmd` 一致，均为 Autoware `Control` 类型。

本任务不要求 CarMaker adapter 消费：

- `/control/command/gear_cmd`
- `/control/command/turn_indicators_cmd`
- `/control/command/hazard_lights_cmd`
- `/control/command/emergency_cmd`
- `/control/command/actuation_cmd`

原因：

- `gear_cmd` 不是 AutoRacer controller 的原始输出，而是 `command_gate` 或下游辅助逻辑生成/转发的支持命令。
- 真车下游 CAN 接口需要这些辅助消息，不代表 CarMaker 行为层 MVP 必须接入。
- CarMaker 当前任务只验证车辆运动行为，不验证灯光、档位协议、驻车协议或 CAN 帧格式。

如果 CarMaker 内部为了让车辆能动必须设置档位，应在 CarMaker adapter 内部固定为可行驶状态，
例如 `GearNo = 1`、`Clutch = 0`、`BrakePark = 0`。这属于 CarMaker 内部执行条件，
不是要求 AutoRacer 输出档位命令。

## 5. 底盘手册信息的使用方式

本文只提取 MVP adapter 直接使用的手册信息。不要把整本手册搬进 plan，也不要因为手册包含
电机、制动、供电、CAN 等章节，就把当前任务扩展成高保真车辆建模。

| MVP 用途 | 手册信息 | 本文用法 |
| --- | --- | --- |
| 车辆形态边界 | 后驱 / 两驱，常规转向 / 阿克曼转向 | 不建四驱、四轮转向、楔形转向 |
| 常规速度范围 | 常规车速 `<= 40 km/h` | `V_manual_normal = 11.1 m/s`，默认测试上限 |
| 适配器速度上限 | 空载最高车速 `55 km/h` | `V_manual_max = 15.3 m/s`，adapter clamp 上限候选 |
| 加速度默认值 | `0-30 km/h` 加速时间 `9 s` | 平均约 `0.93 m/s2`，MVP 初值取 `max_accel_mps2 = 1.2` |
| 转向比默认值 | 方向机范围 `±450 deg`，轮角约 `30 deg` | `steering_ratio = 15.0` |
| 轮胎角限幅 | 内/外轮转角 `30 / 27 deg` | `max_tire_angle_rad = 0.524` |
| 响应迟滞参考 | 动力响应 `<= 200 ms`，转向响应 `<= 150 ms` | 仅用于发现多秒级异常迟滞，不要求实现 delay model |

当前 MVP 不使用以下手册信息：

- 电机额定功率、额定扭矩、峰值扭矩。
- 电池容量、供电系统。
- 制动压力、建压时间、制动压力闭环。
- 悬架参数、防护等级。
- CAN 通讯细节。

这些信息后续可用于高保真车辆建模，但不是当前最小可用 adapter 的输入。

## 6. MVP adapter 设计

### 6.1 输入

从 `/control/command/control_cmd` 读取：

- `longitudinal.velocity`
- `longitudinal.acceleration`
- `lateral.steering_tire_angle`

只使用这些字段即可完成当前 MVP。

### 6.2 速度控制

目标是让 CarMaker 车辆速度跟随 `control_cmd.longitudinal.velocity`。

MVP 使用简单纵向闭环：

```text
target_speed = clamp(cmd.velocity, 0, max_target_speed_mps)
speed_error = target_speed - measured_vehicle_speed

requested_accel =
    clamp(cmd.acceleration, max_decel_mps2, max_accel_mps2)
    + speed_kp * speed_error

requested_accel = clamp(requested_accel, max_decel_mps2, max_accel_mps2)
```

然后映射到 CarMaker 的 Gas / Brake：

```text
if target_speed <= stop_speed_epsilon and measured_vehicle_speed <= stop_speed_hold:
    Gas = 0
    Brake = stop_hold_brake
elif requested_accel >= 0:
    Gas = requested_accel / max_accel_mps2
    Brake = 0
else:
    Gas = 0
    Brake = abs(requested_accel) / abs(max_decel_mps2)
```

要求：

- `Gas` 和 `Brake` 限幅到 `[0, 1]`。
- 不允许长期同时给明显的 `Gas` 和 `Brake`。
- 不允许通过直接写车辆速度来代替油门/刹车执行。
- 允许后续把 P 控制改为 PI/PID，但 MVP 初版不需要积分项，避免低速振荡和积分饱和。

### 6.3 转向控制

CarMaker 使用常规转向模式：

```text
tire_angle = clamp(cmd.steering_tire_angle, -max_tire_angle_rad, max_tire_angle_rad)
steering_wheel_angle = tire_angle * steering_ratio
steering_wheel_angle = clamp(
    steering_wheel_angle,
    -max_steering_wheel_angle_rad,
    max_steering_wheel_angle_rad
)
```

要求：

- 单位使用 rad。
- 符号方向必须通过左右转测试确认。
- 不引入四轮转向、楔形转向或后轮转向逻辑。

## 7. 推荐初始参数

这些参数是 MVP 初值，不是高保真车辆标定结果。允许在验证失败时按本文闭环规则调整。

```text
max_target_speed_mps = 15.3
normal_speed_mps = 11.1

max_accel_mps2 = 1.2
max_decel_mps2 = -4.0
speed_kp = 0.8
speed_ki = 0.0
accel_ff_gain = 0.0

stop_speed_epsilon = 0.1
stop_speed_hold = 0.2
stop_hold_brake = 0.2

max_tire_angle_rad = 0.524
steering_ratio = 15.0
max_steering_wheel_angle_rad = 7.85
max_steering_wheel_rate_radps = 4.0

command_timeout_sec = 1.0
```

说明：

- `max_target_speed_mps = 15.3` 来自手册空载最高车速，可作为 adapter clamp 上限候选。
- `normal_speed_mps = 11.1` 来自手册常规车速范围，可作为默认测试范围上限。
- 若当前 CarMaker 场景、道路或任务配置有更低速度限制，测试上限应取更低值。
- 不要把任意一个速度值写成唯一验收点。

## 8. 测试速度生成规则

为了避免只验证单点速度，测试点应从测试上限自动生成。

定义：

```text
V_manual_normal = 11.1 m/s
V_manual_max = 15.3 m/s
V_config_max = 当前 CarMaker adapter 配置的 max_target_speed_mps
V_scenario_max = 当前场景或任务允许的最高目标速度；如果没有显式配置，取 V_manual_normal

V_test_max = min(V_config_max, V_scenario_max)
```

基础速度测试点：

```text
0
0.25 * V_test_max
0.50 * V_test_max
0.75 * V_test_max
1.00 * V_test_max
```

转换测试：

```text
0 -> 0.25 * V_test_max
0.25 * V_test_max -> 0.75 * V_test_max
0.75 * V_test_max -> 0.50 * V_test_max
0.50 * V_test_max -> 0
```

如果需要专门验证 adapter 的配置上限，可把 `V_scenario_max` 显式设置为 `V_manual_max`。
否则默认以 `V_manual_normal` 覆盖常规速度区间。

## 9. 通过/失败判据

### 9.1 速度响应 PASS

对每个非零目标速度：

```text
abs(measured_vehicle_speed - target_speed)
    <= max(0.5 m/s, 0.15 * target_speed)
```

并且满足：

- 目标速度升高后，车辆速度应整体上升。
- 目标速度降低后，车辆速度应整体下降。
- 稳态前允许短暂过渡，但不能持续发散。
- 不应被旧的低速硬编码上限卡住。

### 9.2 停车 PASS

目标速度为 0 后：

```text
measured_vehicle_speed < 0.2 m/s
```

并且：

- `Gas = 0`
- `Brake > 0` 或车辆可稳定保持停止

### 9.3 转向 PASS

给定正/负转向目标后：

- 方向盘角或前轮角方向正确。
- 车辆横摆方向与命令一致。
- 响应量级不应明显过小或被异常限幅。
- 不出现单位错误导致的过大转向。

### 9.4 FAIL 条件

任一情况视为失败：

- command 已收到，但车辆长期不动。
- 目标速度变化后，`Vehicle.v` 没有对应趋势。
- 速度被固定卡在旧上限附近。
- 加速时 Brake 长期非零，或减速时 Gas 长期非零。
- Gas/Brake 长期同时明显非零。
- 转向符号反了。
- 车辆速度、控制量出现 NaN/inf。
- 通过直接写车辆速度或位姿绕过车辆执行。

## 10. AI 自检与修正闭环

实现 AI 必须按以下顺序工作，不能只改一次就结束。

### 10.1 修改前记录

记录当前关键参数和行为：

- adapter 输入话题。
- `max_target_speed_mps` 或等价速度限幅。
- Gas/Brake 映射参数。
- Steering 映射参数。
- Stage A 低速测试是否仍可运行。

### 10.2 一次只改一类问题

优先顺序：

1. 确认 `/control/command/control_cmd` 被 CarMaker 收到。
2. 确认 command 没有 timeout。
3. 确认 CarMaker 车辆处于可行驶状态。
4. 调整纵向速度 adapter。
5. 调整转向 adapter。
6. 做 Stage A 回归。

不要同时大改桥接、车辆模型、控制器和场景配置。

### 10.3 验证日志至少包含

每个测试点至少记录：

```text
target_speed
measured_vehicle_speed
speed_error
cmd_acceleration
requested_accel
Gas
Brake
steering_tire_angle
steering_wheel_angle
command_age
```

如果失败，必须根据日志归类后再改。

## 11. 常见失败分类与修正方向

### A. 收到 command 但车不动

优先检查：

- CarMaker 是否启用了 `User_VehicleControl_Calc()` 的控制输出。
- `GearNo` 是否处于可行驶状态。
- `Clutch` 是否导致动力断开。
- `BrakePark` 或常规 Brake 是否一直生效。
- command 是否被 timeout 逻辑清零。

修正方向：

- 在 CarMaker adapter 内部设置可行驶条件，例如 `GearNo = 1`、`Clutch = 0`、`BrakePark = 0`。
- 确认 timeout 使用仿真时间或正确的 ROS 时间。
- 确认非零目标速度时 `Gas > 0` 且 `Brake = 0`。

### B. 速度上不去

优先检查：

- 是否仍存在旧的 `5.0 m/s` 或其他低速硬编码限幅。
- `max_target_speed_mps` 是否低于测试目标。
- `max_accel_mps2` 是否过小。
- Brake 是否误触发。
- CarMaker 车辆模型是否有额外限速或动力限制。

修正方向：

- 移除旧低速硬编码，改为配置化限幅。
- 使用手册导出的默认范围。
- 适度提高 `max_accel_mps2` 或 Gas 映射增益。
- 如果只有接近上限测试失败，先确认场景/车辆模型是否允许该速度。

### C. 速度振荡

优先检查：

- `speed_kp` 是否过大。
- 是否加入了积分项。
- Gas/Brake 是否频繁切换。
- command 输入是否抖动。

修正方向：

- 降低 `speed_kp`。
- MVP 阶段保持 `speed_ki = 0`。
- 增加小死区，避免误差很小时频繁切换 Gas/Brake。
- 限制 `requested_accel` 变化率。

### D. 停车不可靠

优先检查：

- 目标速度为 0 时 Gas 是否清零。
- Brake 是否足够。
- stop hold 逻辑是否被速度误差 P 项覆盖。

修正方向：

- 明确目标速度为 0 时进入 stop branch。
- 提高 `stop_hold_brake` 或 `abs(max_decel_mps2)`。
- 确保停车后不会重新给 Gas。

### E. 转向方向或量级错误

优先检查：

- `steering_tire_angle` 单位是否为 rad。
- CarMaker 方向盘角符号与 Autoware 轮胎角符号是否一致。
- `steering_ratio` 是否误用倒数。
- 角度限幅是否过小。

修正方向：

- 用固定正/负小角度测试符号。
- 如符号相反，只在 CarMaker adapter 内修正符号。
- 使用 `steering_ratio = 15.0` 作为初值。
- 使用 `max_tire_angle_rad = 0.524` 做输入限幅。

### F. command timeout 误触发

优先检查：

- `ROS2Bridge.cpp` 是否持续收到 `/control/command/control_cmd`。
- command 时间戳是否与 CarMaker 仿真时间一致。
- `command_age` 是否异常增长。

修正方向：

- 调整 timeout 时间基准。
- 只在真实 command 中断时清零控制。
- 测试时确保 command 发布频率高于 timeout 要求。

## 12. Stage A 回归要求

修改后必须确认 Stage A 低速行为没有退化：

- 低速起步仍能正常运动。
- 低速停车仍能停住。
- 小角度转向仍正确。
- 原有低速场景不因新速度上限或参数化改动失效。

如果 Stage A 回归失败，优先恢复低速稳定性，再继续扩展速度范围。

## 13. 成功定义

当前 MVP 完成的定义：

1. CarMaker adapter 从 `/control/command/control_cmd` 读取速度、加速度、转向字段。
2. 不修改 AutoRacer 和 `command_gate`。
3. 车辆对生成的速度测试点都有正确加速、稳态、减速、停车响应。
4. 转向方向和量级合理。
5. 失败时可通过日志定位到 adapter、车辆可行驶状态、timeout、限幅或转向映射问题。
6. Stage A 低速回归通过。

## 14. 后续非 MVP 工作

以下工作可以以后做，不阻塞当前最小可用目标：

- 更高保真电驱动力学。
- 更真实的制动压力模型。
- 电池、电机热管理或功率限制。
- 真车 CAN 协议仿真。
- 真实底盘响应延迟建模。
- 更完整的车辆参数标定。

## 15. 给实现 AI 的执行提示

实现时只做 CarMaker 侧最小改动：

1. 找到 `ROS2Bridge.cpp` 中 `/control/command/control_cmd` 的接收和缓存位置。
2. 找到 `User.cpp` 中 command 转 Gas/Brake/Steering 的位置。
3. 移除旧低速硬编码限幅，改为本文参数化限幅。
4. 保持输入只依赖 `control_cmd`。
5. 在 CarMaker 内部保证车辆处于可行驶状态，不要求 AutoRacer 输出档位。
6. 按 `V_test_max` 规则生成测试点。
7. 记录第 10.3 节日志字段。
8. 按第 11 节分类修正失败。
9. 最后做 Stage A 低速回归。

一句话目标：

```text
AutoRacer 输出什么合理的 control_cmd，CarMaker 车辆就应在行为上做出对应的速度和转向响应；
接口差异只能由 CarMaker 侧 adapter 消化，不能反向污染 AutoRacer。
```
