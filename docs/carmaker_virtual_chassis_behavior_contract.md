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

验收关注的是组件级行为响应：

- 给定速度目标，CarMaker 车辆速度应朝该目标变化并稳定在合理误差内。
- 给定停止或降速目标，CarMaker 车辆应减速或停车。
- 给定转向目标，CarMaker 车辆应产生方向正确、量级合理的转向响应。
- AutoRacer 不应为了 CarMaker 做额外适配。

本任务不验证完整 Stage A/Stage B 自动驾驶闭环。完整闭环会在后续联调流程中覆盖；
当前只验证 `control_cmd` 到 CarMaker 车辆行为的转换是否正常。

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
- 当前实现是低速场景可用的硬编码转换。

当前主要问题不是 ROS2 数据链路，而是 command 到车辆行为的转换太粗糙：

- 速度上限硬编码偏低，不能覆盖任务所需的常规速度区间。
- 油门/刹车映射是简单低速闭环，未按底盘手册给出合理默认边界。
- 转向映射应明确使用常规转向/阿克曼转向，不引入四轮转向或楔形转向。
- 验证方式应保持组件级：给少量代表性 `control_cmd`，观察 CarMaker 车速和转向行为。

当前项目中，控制验证相关 TestRun 默认使用 `Small.car`：

```text
Data/TestRun/ExternalControl        -> Vehicle = Small.car
Data/TestRun/ExternalControl_10km   -> Vehicle = Small.car
Data/TestRun/AutoracerStageA_10km   -> Vehicle = Small.car
Data/TestRun/AutoracerStageB_UrbanRoute271 -> Vehicle = Small.car
```

`AutoracerCollection_UrbanRoad*` 使用 `AutoracerCollection_UrbanSensorCar`，更偏采集/传感器场景，
不是当前 adapter 组件验证的默认车辆。

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

即使当前代码里仍存在 `/control/command/gear_cmd` 订阅，当前 MVP 的验收目标仍然是：

```text
只发布 /control/command/control_cmd，CarMaker 车辆也能进入可行驶状态并响应速度/转向命令。
```

如果实现需要档位、离合、驻车条件，应在 CarMaker adapter 内部根据当前 control command 和
测试目标设置，不允许把“额外发布 gear_cmd”作为组件测试通过条件。

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

输入防御要求：

- 如果 `cmd.velocity`、`cmd.acceleration` 或 `cmd.steering_tire_angle` 不是有限值，adapter 必须进入安全降级。
- 安全降级行为：

```text
target_speed = 0
Gas = 0
Brake = stop_hold_brake
steering_wheel_angle = 0
```

- 不允许把 NaN/inf 继续传入 CarMaker 控制量。

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

这些参数是 MVP 初值，不是高保真车辆标定结果。允许在验证失败时按本文调试规则调整。

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

## 8. 组件级验证方式

本任务只验证 CarMaker adapter 本身，不要求跑完整 AutoRacer 闭环。

验证时允许使用测试 publisher 直接发布：

```text
/control/command/control_cmd
```

这只用于验证：

```text
control_cmd -> CarMaker adapter -> Gas / Brake / Steering -> Vehicle behavior
```

不代表生产链路。后续联调再接真实 AutoRacer + `command_gate` 输出。

### 8.1 组件测试执行约束

默认测试场景：

```text
首选：Data/TestRun/ExternalControl_10km
备选：Data/TestRun/ExternalControl
车辆：Small.car
```

约束：

- 组件测试时，`/control/command/control_cmd` 应只有测试 publisher 一个来源。
- 不要同时运行会发布同一话题的 AutoRacer/controller/command_gate 节点，避免命令混杂。
- 测试 publisher 必须持续发布命令，建议 `10 Hz`，不允许只 publish 一次。
- 每个命令必须保持指定时间，让 adapter 避免被 `command_timeout_sec` 误判为超时。
- 测试 publisher 只发布 `/control/command/control_cmd`，不额外发布 `gear_cmd`。
- 如果 AI 无法启动 CarMaker、无法观察 `Vehicle.v/Gas/Brake/Steering` 或无法读取相关日志，
  只能声明“代码修改完成，等待 CarMaker 组件验证”，不能声明任务通过。

### 8.2 最小速度验证

在简单直线路段或当前可用 CarMaker 场景中，依次发布少量代表命令即可。

建议最小命令集：

```text
Command A: hold 2 s,  velocity = 0.0 m/s, acceleration = 0.0 m/s2, steering_tire_angle = 0.0 rad
Command B: hold 6 s,  velocity = 2.0 m/s, acceleration = 0.0 m/s2, steering_tire_angle = 0.0 rad
Command C: hold 12 s, velocity = 8.0 m/s, acceleration = 0.0 m/s2, steering_tire_angle = 0.0 rad
Command D: hold 10 s, velocity = 0.0 m/s, acceleration = 0.0 m/s2, steering_tire_angle = 0.0 rad
```

说明：

- `2.0 m/s` 用于确认低速起步转换正常。
- `8.0 m/s` 低于手册常规速度上限 `11.1 m/s`，但高于当前旧实现的 `5.0 m/s` 限幅，
  用于确认不再被旧的低速硬编码明显卡住。
- 本任务不要求覆盖完整连续速度区间。
- 如果当前场景或车辆模型确实不允许 `8.0 m/s`，可以换成场景允许的代表速度，但必须记录原因。

### 8.3 最小转向验证

在低速下发布：

```text
Command E: hold 5 s, velocity = 2.0 m/s, acceleration = 0.0 m/s2, steering_tire_angle = +0.05 rad
Command F: hold 5 s, velocity = 2.0 m/s, acceleration = 0.0 m/s2, steering_tire_angle = -0.05 rad
```

只验证方向和量级是否合理，不要求做轨迹跟踪精度评估。

## 9. 通过/失败判据

### 9.1 command 接收 PASS

满足：

- CarMaker adapter 能收到 `/control/command/control_cmd`。
- 日志中能看到目标速度、目标加速度、目标转向角。
- command 未被错误 timeout 清零。
- 日志或 `ros2 topic info` 能确认组件测试期间没有其他 publisher 干扰 `/control/command/control_cmd`。

### 9.2 速度转换 PASS

发布 `Command B` 后：

- `Gas > 0`。
- `Brake = 0` 或接近 0。
- `Vehicle.v` 随时间上升。
- `5 s` 内 `Vehicle.v > 1.0 m/s`。

发布 `Command C` 后：

- `Gas > 0`，直到车辆接近目标速度前不应长期为 0。
- `Brake = 0` 或接近 0。
- `Vehicle.v` 继续上升。
- `12 s` 内 `Vehicle.v > 5.5 m/s`，用于排除旧的 `5.0 m/s` 硬编码限幅。

发布 `Command D` 后：

- `Gas = 0`。
- `Brake > 0`，或车辆能稳定减速并停住。
- `Vehicle.v` 随时间下降。
- `10 s` 内 `Vehicle.v < 0.3 m/s`。

这里不要求精确标定加速曲线，也不要求达到真实底盘的动力学响应。

### 9.3 转向转换 PASS

发布正/负小转角命令后：

- CarMaker 方向盘角或前轮角方向正确。
- `Command E` 和 `Command F` 必须产生相反符号的方向盘角/前轮角响应。
- `Command E` 和 `Command F` 必须产生相反方向的横摆响应。
- 响应量级合理，不出现单位错误导致的异常大角度或几乎无响应。
- 如果项目已有明确左右转符号约定，按该约定判断正负号；如果没有，至少必须证明正负命令
  对应的车辆响应方向相反，并在实现记录中写明实际采用的符号映射。

### 9.4 FAIL 条件

任一情况视为失败：

- command 已发布，但 CarMaker adapter 收不到。
- command 已收到，但车辆长期不动。
- 正速度命令下 `Gas` 不上升，或 Brake 长期生效。
- 停止命令下 `Gas` 仍长期非零。
- `Vehicle.v` 与目标速度变化方向相反或无趋势。
- 转向符号反了。
- 车辆速度、控制量出现 NaN/inf。
- 通过直接写车辆速度或位姿绕过车辆执行。

## 10. AI 自检与修正流程

实现 AI 必须按以下顺序工作，不能只改一次就结束。

### 10.1 修改前记录

记录当前关键参数和行为：

- adapter 输入话题。
- `max_target_speed_mps` 或等价速度限幅。
- Gas/Brake 映射参数。
- Steering 映射参数。
- 当前 CarMaker 测试场景和车辆文件。

### 10.2 一次只改一类问题

优先顺序：

1. 确认 `/control/command/control_cmd` 被 CarMaker 收到。
2. 确认 command 没有 timeout。
3. 确认 CarMaker 车辆处于可行驶状态。
4. 调整纵向速度 adapter。
5. 调整转向 adapter。
6. 重新运行组件级速度/转向验证。

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

日志采样频率建议 `10 Hz`。高频 debug 日志必须可开关，默认关闭，避免刷爆 CarMaker 日志。

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

## 12. 非本任务验收项

以下内容不作为当前 adapter 组件任务的完成条件：

- 完整 Stage A / Stage B 闭环跑通。
- planner、controller、localization、sensing feedback 是否正常。
- `command_gate` 的安全策略是否正确。
- 真实底盘 CAN 接口是否正确。
- 车辆动力学是否高保真匹配 HOOKE 真车。

这些会在后续集成流程中验证。当前任务只要求：

```text
给定 control_cmd，CarMaker 车辆在速度和转向行为上做出对应响应。
```

## 13. 成功定义

当前 MVP 完成的定义：

1. CarMaker adapter 从 `/control/command/control_cmd` 读取速度、加速度、转向字段。
2. 不修改 AutoRacer 和 `command_gate`。
3. 使用测试 publisher 直接发布代表性 `control_cmd` 时，CarMaker adapter 能收到并转换。
4. 不发布 `gear_cmd` 时，CarMaker adapter 仍能在内部保证车辆处于可行驶状态。
5. 速度命令满足第 9.2 节最小数值阈值，停止命令满足停车阈值。
6. 正/负转向命令会产生方向相反、量级合理的转向行为。
7. 如果无法运行 CarMaker 组件验证，不能声明当前 MVP 已通过。
8. 失败时可通过日志定位到 adapter、车辆可行驶状态、timeout、限幅或转向映射问题。

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
6. 优先不改 CarMaker Vehicle Data / Powertrain 参数文件。
7. 只有确认 adapter 输出正确但车辆模型物理限制导致无法响应时，才讨论车辆模型参数。
8. 使用测试 publisher 以 `10 Hz` 直接发布少量代表性 `control_cmd`，每个命令保持指定时间。
9. 记录第 10.3 节日志字段。
10. 按第 11 节分类修正失败。
11. 不把完整 Stage A/Stage B 闭环作为当前任务完成条件。

一句话目标：

```text
AutoRacer 输出什么合理的 control_cmd，CarMaker 车辆就应在行为上做出对应的速度和转向响应；
接口差异只能由 CarMaker 侧 adapter 消化，不能反向污染 AutoRacer。
```
