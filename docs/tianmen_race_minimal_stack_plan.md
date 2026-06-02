# 天门山封闭赛道竞速最小自动驾驶栈方案

本文记录 `autoracer_hooke` 面向天门山九十九道弯封闭竞速场景的最小必要软件方案，同时明确当前阶段的最高优先级：**先在 CarMaker 仿真环境中把定位链路调通，再接规划和控制闭环**。

当前不能假设各模块天然能协同工作。`autoracer_hooke` 是由选定 Autoware 基础模块、Hooke2 实车接口、自研 planning / control / safety 节点和 CarMaker 适配层组合出来的系统。第一阶段的目标不是直接追求天门山成绩，而是在没有实车、场地和真实传感器的条件下，用 CarMaker 搭建可复现的车、路、场景、传感器和执行器闭环，证明每个模块的输入输出契约正确，整个自动驾驶框架可以端到端工作。

更详细的 CarMaker 接入接口、阶段门禁和运行证据要求见：

```text
SimProject_TianmenRace/AUTORACER_HOOKE_CARMAKER_SIMULATION_PLAN.md
SimProject_TianmenRace/CARMAKER_BUILTIN_URBAN_COLLECTION_PLAN.md
```

本文是上层方向约束。具体实现必须以当前仓库源码、CarMaker 工程、launch 参数、日志和 ROS2 topic 行为为准。

## 执行摘要：当前先做定位

当前最大技术风险是定位，不是规划闭环，也不是 racing line。因此后续执行入口应按下面顺序推进：

```text
保留 urban_collection_20260531_171753 / route 271 / 10.8km 作为长路线回归基线
  -> 使用已建立的 1km short-route 快速验证基线
     仍用同一张 UrbanRoad_RuralRoad_Expressway.rd5，route 4226 / 996.319m
     目的只是缩短 PCD / scan-map overlap / NDT replay 迭代时间，不改变地图构建原则
  -> Stage C0 CarMaker localization-only validation
     可隔离 NDT，只启动点云地图、CarMaker LiDAR、NDT 初始位姿、NDT 和 ground-truth 对比
     也可并行验证 CarMaker 模拟 Fixposition 原生话题是否满足 AutoRacer localization 输入契约
  -> 明确 localization topic ownership
     ground truth 只能做诊断，NDT/EKF 才能发布算法定位输出
  -> 用 short-route 已通过候选回归 route 271 长路线 PCD / overlap / NDT
  -> 定位-only 证据可复现后，再做 Stage B planning closed-loop
     Stage B 默认 route 271；MIN_DISTANCE_M=500 只是 first-500m smoke gate，不是 500m route
  -> 最后做 Stage C full sensor-localization closed-loop 和 racing line
     LiDAR/NDT、CarMaker Fixposition 原生话题、vehicle status 进入 EKF / localization 链路
```

第一步不是重采集、不是新增 `autoracer_race_planning`、不是继续堆控制闭环，而是建立一个最小定位验证入口，回答：

```text
CarMaker LiDAR scan + 当前 PCD map + GT initial pose 能不能让 NDT 稳定输出 pose？
CarMaker 模拟的 Fixposition 原生话题能不能让 AutoRacer 的 GNSS/INS/IMU 订阅链路正常工作？
```

## 0. 当前仓库事实和门禁状态

下面状态用于防止把“已有资产”误读成“链路已通过”。状态基于当前仓库源码和已有运行证据，应随新的 `runtime_summary.json` 更新。更新时间点：2026-06-01 当前工作树。

| 验证线 | 当前结论 | 关键证据 | 不能外推的内容 |
| --- | --- | --- | --- |
| 天门山 / Stage A 控制闭环 | 已有低速 `PASS` 证据 | `SimProject_TianmenRace/logs/autoracer_stage_a_20260531_092233/runtime_summary.json` 显示 RoadEval trajectory provider、pure pursuit、command_gate、`VehicleControl` 和车辆响应均可观测 | 只证明 ground-truth pose + RoadEval trajectory + 低速控制闭环，不证明 Lanelet2、NDT、EKF、高速竞速或最终安全门 |
| CarMaker 内置 urban collection | 已有 route 271 和 route 4226 两类 `PASS` 证据 | route 271 长路线：`SimProject_TianmenRace/logs/urban_collection_20260531_171753/runtime_summary.json` 和 `urban_collection_route271_phase1_userdefined_pandar40_20260601_104102_20260601_104102/runtime_summary.json` 均显示 `route_id = 271`、`route_progress_m = 10798.475`；route 4226 短路线：`urban_collection_short1km_phase1_userdefined_pandar40_loc_20260601_100844_20260601_100844/runtime_summary.json` 显示 `route_id = 4226`、`route_progress_m = 991.451`、LiDAR `frame_id = lidar_top`、`points = 23054850` | collection PASS 只证明采集输入事实可用；不证明正式 map 资产、NDT 或闭环已经通过 |
| CarMaker 内置 urban route / planner assets | route / planner 资产可用于 Stage B 候选验证，但 Stage B 闭环尚未验收 | `SimProject_TianmenRace/logs/urban_map_build_20260531_174150/runtime_summary.json` 中 `lanelet_count = 1`、`trajectory_points = 3097`、`trajectory_lateral_error_max_m = 1.3e-6`、`trajectory_heading_error_max_rad = 8.5e-7`，说明 route 271 corridor、goal 和 planner validation 自洽；当前 `carmaker_stage_b.launch.py` 已串接 `route_goal_publisher -> lanelet_route_planner -> local_trajectory_planner -> pure_pursuit_controller -> command_gate` | 只能说明 route / planner 资产可加载、可出轨迹；不能说明 ego closed-loop Stage B 已 PASS，也不能说明 NDT / localization 可用 |
| CarMaker 内置 urban pointcloud / static geometry | 0.07m static filter 下静态几何 `PASS`，点云与 CarMaker 静态表面对齐误差很小 | `SimProject_TianmenRace/logs/urban_map_build_20260531_174150/runtime_summary.json` 显示 `pcd_points = 75704`、`static_geometry_validation_status = PASS`、`point_to_static_surface_median_m = 0.03478m`、`mean = 0.03494m`、`rmse = 0.04046m`、`p95 = 0.06661m`、`p99 = 0.06931m`、`outlier_gt_1m_ratio = 0.0`、route progress coverage `1.0` | 这些指标证明点云几何位置基本正确，尤其是与仿真建筑、道路、地形等静态 surface 的误差很小；但不证明 NDT 一定收敛，几何精度和 NDT 可用密度是两个不同门禁 |
| CarMaker 内置 urban NDT replay / Stage 2B | 已从旧 FAIL 进入候选 PASS/回归阶段，但正式发布资产仍不能混淆 | 旧正式目录 `autoracer_hooke/maps/carmaker_builtin_urban/runtime_summary.json` 仍是 `FAIL`、`raw_ndt_pose_count = 0`；route 4226 最新候选 `urban_map_build_short1km_phase1_userdefined_pandar40_lidar_built_diag_20260601_070843/ndt_replay_validation.json` 为 `PASS`，`raw_ndt_pose_count = 53`、`ndt_convergence_ratio = 0.7067`、`ndt_error_mean_m = 0.149m`、`ndt_error_max_m = 0.529m`；route 271 新候选已有 PCD density 和 independent scan-map overlap `PASS`，但缺完整 runtime/NDT 回归 summary | short-route NDT PASS 不能覆盖 route 271 正式目录；候选结果必须经 Stage C0 或等价 localization-only 入口复现后再发布 |
| Stage C0 CarMaker localization-only validation | 当前第一优先级，部分离线诊断已通过，端到端入口尚未最终验收 | 输入事实已具备：urban collection rosbag、CarMaker hit-derived `PointCloud2`、PCD map、ground-truth pose、vehicle status；CarMaker Bridge 已可模拟 Fixposition 原生 `/fixposition/*` 话题；short-route NDT replay 已有 PASS 候选 | 仍需独立 Stage C0 summary 证明 NDT isolated 与 Fixposition contract 的 runtime topic 契约；不启动 planning/control/VehicleControl，避免闭环现象掩盖定位问题 |
| Stage B 规划闭环 | 入口已存在，尚未最终验收 | `SimProject_TianmenRace/run_autoracer_stage_b_headless.sh` 默认 `TESTRUN=AutoracerStageB_UrbanRoute271`、`MIN_DISTANCE_M=500`；`autoracer_hooke/src/autoracer_bringup/launch/carmaker_stage_b.launch.py` 已包含 Local Planning；最近 `autoracer_stage_b_20260531_235645/carmaker.log` 因 `GPUSensor A1:localhost.1: Timeout during initialization` 后 `SIM_ABORT`，且无 `runtime_summary.json` | `MIN_DISTANCE_M=500` 是 route 271 上的 first-500m smoke gate，不是 500m route；不能用 Stage A RoadEval trajectory provider 兜底后标记为规划闭环通过 |
| Stage C 传感器定位闭环 | 尚未最终验收 | 需要 LiDAR -> preprocessing -> NDT/EKF -> `/localization/pose_with_covariance` -> planning/control 的闭环证据 | 当前 Bridge ground-truth pose 与 NDT 输出的 topic ownership 需要先明确，否则会出现同一 localization topic 多发布源；Stage C 前必须先通过 Stage C0 |

因此，当前第一优先级不是新增 `autoracer_race_planning`，也不是先做 Stage B 规划闭环，而是：

```text
保留 urban_collection_20260531_171753 / route 271 / 10.8km 作为长路线回归基线
  -> 使用已建立的 1km short-route 快速验证基线：同一 road，route id 4226，route length 996.319m
  -> 建立 Stage C0 localization-only replay / bringup
  -> 准备 NDT 所需最小输入：PCD map、PointCloud2、GT initial pose、map service、ground-truth diagnostic pose
  -> 验证 CarMaker 模拟 Fixposition 原生输入：/fixposition/fix、/fixposition/autoware_orientation、/fixposition/rawimu、/fixposition/odometry_enu、/fixposition/fpa/odomstatus
  -> 明确 Stage C localization topic ownership
  -> 使用已实现的 density / scan-map overlap / NDT 诊断定位 frame / stamp / initial pose / score 问题
  -> 对已实现的 integrated voxel/static-nearest 代表点策略做 short-route 到 route 271 回归
  -> Stage C0 定位-only PASS 后，再做 Stage B planning closed-loop
  -> 再接 Stage C NDT / EKF 传感器定位闭环
  -> 最后进入 racing line / speed profile
```

## 1. 项目背景与第一原则

### 1.1 最终比赛场景

最终比赛场景是天门山九十九道弯封闭赛道：

- 从山顶到山底的计时赛，目标是用时最短；
- 赛道封闭，无红绿灯、无行人、无其他车辆、无动态障碍物；
- 道路主体是山路、路沿、山体、树木，行驶区域本身是干净道路；
- 山体遮挡严重，GNSS / RTK 不可靠；
- 可用核心传感器是 LiDAR、Fixposition / IMU、车辆底盘反馈。

因此，最终问题不是城市道路自动驾驶，而是：

```text
在已知封闭赛道上稳定定位、稳定寻迹，并尽可能快地通过弯道。
```

### 1.2 当前开发现实

当前更紧急的问题不是 racing line 是否最优，而是：

```text
这套由多个来源拼装出来的自动驾驶框架，是否能在一个可控环境中正常工作。
```

现实约束：

- 目前没有可随时使用的实车；
- 没有稳定可控的封闭测试场地；
- 没有完整真实传感器数据流；
- 不能把未验证的控制、定位、规划链路直接放到实车上调；
- 各模块来源不同，topic、frame、时间戳、消息类型、参数和生命周期很可能不一致。

所以第一原则是：

```text
先在 CarMaker 中构建可复现的数字测试场，把 autoracer_hooke 调通；
再把同一套算法边界迁移到实车；
最后再进入天门山真实赛道测试和竞速调参。
```

## 2. 仿真优先的正向验证路线

### 2.1 为什么仿真是第一步

CarMaker 仿真环境在本项目中不是辅助演示工具，而是算法框架正确性的第一道门禁：

- 它提供可重复运行的道路、地形、建筑物、路沿和静态环境；
- 它提供可控的 ego 车辆动力学、执行器响应和底盘状态；
- 它可以产生 ground-truth pose、vehicle status、RoadEval 几何数据；
- 它可以通过 LidarRSI / GPUSensor 提供仿真 LiDAR hit；
- 它允许在没有实车和真实传感器的情况下，先验证 ROS2 topic、frame、时间戳、地图、定位、规划、控制、安全门和车辆接口。

换句话说，仿真阶段要解决的问题是：

```text
车是否存在、路是否存在、传感器是否存在、数据是否可信、
算法是否能消费这些数据、控制命令是否能驱动车辆响应。
```

只有这些事实成立，后续讨论天门山 racing line、速度剖面和实车部署才有工程基础。

### 2.2 场景选择原则

调通 `autoracer_hooke` 框架不要求第一步就使用天门山场景。任何成熟 CarMaker 场景都可以作为正向验证场，只要满足：

- road / terrain / buildings / static objects 能被 CarMaker 正常加载；
- 可以清理或禁用其他车辆、行人和动态交通参与者；
- ego 车可以在场景中运行；
- 可以挂载或模拟项目所需传感器；
- 可以导出或生成 `autoracer_hooke` 需要的地图、route、trajectory 和验证报告。

当前优先使用 CarMaker 15.1 内置场景作为通用验证场：

```text
Data/TestRun/Examples/Powertrain/DrivingScenarios/CityDriving
Data/Road/Examples/Synthetic/Scenario/UrbanRoad_RuralRoad_Expressway.rd5
```

这类成熟场景比直接从天门山开始更适合调通框架，因为它能先把问题拆开：

```text
CarMaker 标准场景可运行
  -> 采集或生成传感器 / ground truth / route 数据
  -> 生成 PCD / Lanelet2 / projector / route goal / extrinsic
  -> autoracer_hooke 在同一场景中闭环验证
  -> 再迁移到天门山专用资产和竞速策略
```

#### 2.2.1 使用已建立的 1km short-route 做定位快速迭代

当前 `UrbanRoad_RuralRoad_Expressway.rd5` 已经证明可采集、可发布 CarMaker LiDAR、可生成 route samples 和静态几何验证输入。不要因为 route 271 太长就立刻换场景。更简单、风险更低的方式是：**同一张 road，换短 route**。

已在该 road 中确认的路线长度：

```text
Road:
  Data/Road/Examples/Synthetic/Scenario/UrbanRoad_RuralRoad_Expressway.rd5

长路线回归基线:
  Route.6.ID = 271
  Route.6.Length = 10803.772m

短路线快速验证基线:
  Route.1.ID = 4226
  Route.1.Length = 996.319m
```

当前仓库中两条路线应明确分工：

- `271 / 10803.772m`：长路线回归和 Stage B 默认闭环路线；
- `4226 / 996.319m`：short-route collection、PCD / overlap / NDT 快速迭代路线；
- Stage B 脚本中的 `MIN_DISTANCE_M=500` 是 route 271 上的 first-500m smoke gate，不是另一条 500m CarMaker route，也不与 route 4226 重复。

短路线用途：

- 快速迭代 PCD 代表点策略、点云密度、scan-map overlap 和 NDT replay；
- 减少一次 collection / map build / NDT replay 的等待时间；
- 降低调试成本，同时保持 road、terrain、buildings、static objects、LidarRSI、RoadEval 和静态几何来源不变；
- 先在 1km 上证明定位链路能 work，再回到 route 271 / 10.8km 做回归。

短路线不改变验收原则：

- PCD 仍必须来自 CarMaker LidarRSI / GPUSensor 的真实仿真 `PointCloud2`；
- Lanelet2 / route corridor 仍必须来自同一 CarMaker road / route samples；
- `map_projector_info.yaml`、pose、PCD、Lanelet2 仍必须使用同一 `map` / CarMaker Fr0 坐标；
- `Traffic.N` 应保持 `0`，避免动态交通污染定位地图；
- short-route 的所有 map / log / summary 必须与 long-route 基线分开，不能覆盖 long-route 的正式资产；
- short-route PASS 只能证明快速验证链路通过，不能外推为 route 271 / 10.8km 已通过。

当前已建立独立 TestRun 和输出目录；后续必须继续保持 short-route 与 long-route 资产隔离：

```text
TestRun:
  SimProject_TianmenRace/Data/TestRun/AutoracerCollection_UrbanRoad_Short1km

关键配置:
  Road.FName = Examples/Synthetic/Scenario/UrbanRoad_RuralRoad_Expressway.rd5
  Vehicle.Routing.ObjId = 4226
  Vehicle.StartPos.ObjId = 4226
  Vehicle.StartPos = 0.00 0
  Traffic.N = 0

运行参数:
  ROUTE_ID = 4226
  ROUTE_LENGTH_M = 996.319
  MIN_DISTANCE_M = 850 或 900
  ROUTE_STOP_DISTANCE_M = 1050 左右
  TSTOP / TIMEOUT_SEC 按 1km 路线缩短

已存在证据:
  logs/urban_collection_short1km_phase1_userdefined_pandar40_loc_20260601_100844_20260601_100844/runtime_summary.json
  logs/urban_map_build_short1km_phase1_userdefined_pandar40_lidar_built_diag_20260601_070843/pcd_density_validation.json
  logs/urban_map_build_short1km_phase1_userdefined_pandar40_lidar_built_diag_20260601_070843/scan_map_overlap_validation.json
  logs/urban_map_build_short1km_phase1_userdefined_pandar40_lidar_built_diag_20260601_070843/ndt_replay_validation.json

候选 map/log:
  logs/urban_collection_short1km_<timestamp>/
  logs/urban_map_build_short1km_<timestamp>/map_candidate/
  必要时正式发布到独立目录，例如 autoracer_hooke/maps/carmaker_builtin_urban_short1km/
```

不建议当前优先使用 `Data/TestRun/Examples/HDScenarios/*`。这些 TestRun 在本机安装中多为 downloadable scenario placeholder，文件只有少量描述行，不是已经完整安装并可复现的本地场景。现在为了定位调试，应优先减少变量，而不是引入新的 external scene 下载和资产完整性风险。

### 2.3 CarMaker 在本项目中的角色

CarMaker 是 plant，不是自动驾驶算法：

```text
CarMaker road / terrain / static scene
  -> CarMaker ego vehicle dynamics
  -> CarMaker sensors / RoadEval / ground truth
  -> ROS2 Bridge / simulation adapter
  -> autoracer_hooke localization / planning / control / safety
  -> simulation vehicle interface
  -> CarMaker VehicleControl
  -> Brake / PowerTrain / Steering
  -> vehicle dynamics
```

关键边界：

1. CarMaker 负责虚拟世界、传感器、车辆动力学和执行器响应；
2. `autoracer_hooke` 负责自动驾驶算法；
3. ROS2 Bridge / simulation adapter 只做状态、传感器、控制命令和执行器接口转换；
4. `VehicleControl` 是 CarMaker 侧闭环主入口；
5. IPGDriver 可以用于采集车或车辆上电 / 挂挡状态管理，但不能替代 ego 自动驾驶控制；
6. 不允许直接改写车辆位姿制造“跑起来”的假闭环。

当前 CarMaker 主循环中的关键语义应保持清楚：

- `User_In()` / Bridge 发布 CarMaker 状态、RoadEval、仿真传感器数据；
- ROS2 节点消费这些数据并输出控制；
- `User_Calc()` 接收 ROS2 控制命令；
- `User_VehicleControl_Calc()` 把安全门之后的控制命令转换为 CarMaker `VehicleControl` 输入；
- CarMaker 再计算 Brake / PowerTrain / Steering / vehicle dynamics。

### 2.4 通用 CarMaker 操作基线

后续无论由 AI 还是人类继续实现本方案，都应把下面这套方式作为 CarMaker 仿真操作基线。它不是某一次实验日志的复述，而是当前工程已经验证出的通用使用方法。

#### 2.4.1 一律优先用 headless CLI 验证

CarMaker GUI 只能用于人工排障，不应作为验收方式。可复现验证入口必须用脚本启动 CarMaker，并把命令、环境、日志、rosbag 和判定摘要全部落盘。

通用命令形态：

```bash
SimProject_TianmenRace/src/CarMaker.linux64 \
  -screen \
  -projectdir /opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace \
  -ngpu 1 \
  -taccel 1 \
  -tstop <duration_sec> \
  <testrun_name>
```

要求：

- `-projectdir` 必须指向当前 CarMaker project；
- `<testrun_name>` 是 `Data/TestRun/` 下可加载的 TestRun 名称或 CarMaker 可解析入口；
- `-screen` 用于无 GUI 运行；
- `-ngpu 1` 用于启用 GPU sensor 相关路径；
- `TACCEL=1` 是闭环验证默认值，高倍率只用于非闭环探索；
- 每次运行都必须写出 `carmaker_command.txt`、`environment_snapshot.txt`、`carmaker.log` 和最终 `runtime_summary.json`；
- 脚本应清理 Python / ROS / library 环境，优先使用 `/usr/bin/python3`，避免继承 Anaconda、旧 workspace 或错误 `libstdc++`。

当前工程中的参考实现是：

```text
SimProject_TianmenRace/run_urban_collection_headless.sh
SimProject_TianmenRace/run_autoracer_stage_a_headless.sh
SimProject_TianmenRace/run_autoracer_stage_b_headless.sh
SimProject_TianmenRace/run_urban_map_build.sh
```

#### 2.4.2 使用 LidarRSI / GPUSensor 时必须同时启动 MovieEGL

CarMaker 的 LidarRSI / GPUSensor 不只是 `CarMaker.linux64` 一个进程。headless 采集 LiDAR 时，需要在 CarMaker 启动后启动 MovieEGL 的 GPUSensor 实例。

通用命令形态：

```bash
/opt/ipg/carmaker/linux64-15.1/GUI/MovieEGL.exe \
  -headless \
  -mode GPUSensor \
  -instance 1 \
  -apphost localhost \
  -projectdir /opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace \
  -exitatsimend
```

要求：

- MovieEGL 通常在 CarMaker 启动后延迟短时间启动；
- MovieEGL 日志中应能看到 headless EGL device、RSI Sensors、Lidar RSI、CUDA device、Scene loaded、Simulation starts 等事实；
- 如果 MovieEGL 在 `SIM_END` 后退出清理不干净，不能只看退出码下结论，必须结合 CarMaker `SIM_END`、LiDAR 帧数、rosbag 完整性和 `runtime_summary.json` 判定；
- 如果日志中出现 license、No GPU、EGL、CUDA、GPUSensor、MovieNX、MovieEGL 相关错误且无 LiDAR 点云，应判为 `BLOCKED` 或 `FAIL`，不能 fallback 到假点云。

#### 2.4.3 传感器车和 TestRun 必须是独立资产

采集车、闭环测试车和原始 CarMaker example 应分离。不要直接覆盖 CarMaker 自带车辆或原始 TestRun。

一个可用于 LiDAR 采集的 vehicle 至少应包含：

```text
Sensor.Param.<n>.Type = LidarRSI
Sensor.Param.<n>.Beams.FName = LidarRSI_Default
SensorCluster.<m>.Type = LidarRSI
SensorCluster.<m>.UseMovieNX = 1
SensorCluster.N >= 1
```

同时应挂载或发布：

- ground-truth pose；
- velocity / steering status；
- RoadEval / route progress / centerline；
- LiDAR extrinsic；
- 必要时的 GroundTruthSensor。

TestRun 应明确：

- road 文件；
- ego vehicle；
- route id / start pose；
- 是否禁用动态交通参与者；
- 采集阶段是否允许 IPGDriver 驾驶；
- 闭环阶段是否由 `autoracer_hooke` 控制 ego 车。

采集阶段可以用 IPGDriver 当采集车司机；闭环验证阶段不允许 IPGDriver 替代自动驾驶算法。

#### 2.4.4 LidarRSI 到 PointCloud2 的转换必须遵守 CarMaker 语义

`tScanPoint.Origin` 不是 hit point。它是 ray origin。点云 hit 应按 beam 和 range 还原：

```text
beam_direction_sensor = direction(BeamID azimuth / elevation)
hit_sensor = scan_point.Origin + beam_direction_sensor * scan_point.LengthOF
hit_base = sensor_extrinsic * hit_sensor
```

然后再发布：

```text
/sensing/lidar/concatenated/pointcloud  sensor_msgs/msg/PointCloud2
/carmaker/sensor/lidar_extrinsic        geometry_msgs/msg/PoseStamped
```

最低要求：

- `PointCloud2.header.frame_id` 必须明确，例如 `base_link`；
- 点字段至少包含 `x/y/z/intensity`；
- 点数、帧率、时间戳域、range 过滤和 skipped 点数必须可解释；
- 不能用随机点、离线 PCD 重播或 `Origin` 伪装 CarMaker 原生 LiDAR hit；
- 进入 NDT 前，必须先用 hit count、frame、点数、频率和简单几何位置证明点云语义正确。

当前代码路径中，CarMaker Bridge 将 hit 转到 `base_link` 后发布 `/sensing/lidar/concatenated/pointcloud`，并通过 `/carmaker/sensor/lidar_extrinsic` 记录 `base_link -> lidar_link` 外参。这个约定可以支撑当前 map build / NDT replay 调试，但 Stage C 进入最终闭环前必须显式二选一：

1. 保持 `PointCloud2.header.frame_id = base_link`，则下游 launch、NDT 参数和报告必须说明点云已经在 base frame 中，不再依赖 LiDAR TF；
2. 改为 `PointCloud2.header.frame_id = lidar_link`，并提供稳定的 `base_link -> lidar_link` static TF；这更接近实车传感器链路，也更利于迁移。

无论选择哪一种，PCD、Lanelet2、ground-truth pose、NDT 输入和 TF 必须共用同一坐标契约。不能在不同脚本中混用 `base_link` 点云和 `lidar_link` 外参后再用经验误差兜底。

#### 2.4.5 数据采集应使用 rosbag / mcap 固化输入事实

仿真采集不是看屏幕，而是生成可回放数据包。采集脚本应至少记录：

```text
/sensing/lidar/concatenated/pointcloud
/localization/pose_with_covariance
/vehicle/status/velocity_status
/vehicle/status/steering_status
/carmaker/road/centerline
/carmaker/sensor/lidar_extrinsic
```

LiDAR 话题通常需要 `best_effort` QoS override。采集后必须写出：

```text
rosbag_info.txt
lidar_summary.json
route_progress.csv
vehicle_state.csv
sensor_extrinsic.yaml
runtime_summary.json
```

`runtime_summary.json` 至少应包含：

- `stage`；
- `status: PASS | FAIL | BLOCKED`；
- TestRun、road、route id、route length；
- `headless_cli`、`ngpu`、`taccel`、`tstop_sec`；
- CarMaker / MovieEGL exit status；
- 行驶距离和 route progress；
- LiDAR frames、points、rate、frame；
- rosbag 路径和 topic 完整性；
- 逐项 checks 和 failure reason。

#### 2.4.6 地图和路线资产必须从同一坐标链生成

用于调通 `autoracer_hooke` 的 PCD、Lanelet2、projector、goal、extrinsic 应来自同一次或同一套可解释的 CarMaker 场景 / route / pose / LiDAR 数据。

通用生成链路：

```text
CarMaker road / terrain / static scene
  -> clean TestRun
  -> headless collection rosbag
  -> extract_collection
  -> pointcloud_map.pcd
  -> lanelet2_map.osm / route corridor
  -> map_projector_info.yaml
  -> route_goal.yaml
  -> sensor_extrinsic.yaml
  -> alignment_report.txt
  -> asset_manifest.json
  -> runtime_summary.json
```

实际落盘时应区分“候选资产”和“正式基线”：

```text
logs/urban_map_build_<timestamp>/map_candidate/
  -> 全部 validation PASS
  -> 原子发布到 autoracer_hooke/maps/carmaker_builtin_urban/
  -> 写入 asset_manifest.json / runtime_summary.json / alignment_report.txt
```

要求：

- PCD 必须来自 CarMaker LidarRSI / GPUSensor 采集的 `PointCloud2`；
- Lanelet2 / route corridor 必须来自当前 CarMaker road / RoadEval / route samples；
- `map_projector_info.yaml`、pose、PCD、Lanelet2 必须使用同一坐标约定；
- `route_goal.yaml` 应由脚本生成，不能依赖 RViz 人工点选；
- 文件存在不等于 PASS，必须再做 loader、planner、NDT replay 或闭环验证；
- map build 不应在验证完成前直接覆盖正式 `autoracer_hooke/maps/...` 目录；如果脚本中途失败，正式目录必须保持上一套完整基线，或者显式标记为 `BLOCKED` / `DIRTY`；
- `pointcloud_map.pcd`、`pointcloud_map_build.json`、`lanelet2_route_build.json`、`alignment_report.txt`、`runtime_summary.json` 和 `asset_manifest.json` 必须来自同一次 build；点数、输入 bag、route id、生成时间和 validation log dir 必须能互相对上；
- NDT replay、route bbox coverage、trajectory 自洽不能替代静态几何对齐验证；如果 static geometry validation 失败，应如实输出 `FAIL` 或 `BLOCKED`。
- NDT replay 的验收不能只看误差均值；还必须看输出 pose 数量、pose rate、覆盖时间段、覆盖距离、初始化方式和失败原因。少量收敛 pose 可以证明链路可调试，但不能证明 Stage C 闭环定位稳定。

当前 Stage 2 应拆成两个子门禁，避免把 route / planner 资产和 PCD / NDT 定位资产混成一个不可解释的总状态：

```text
Stage 2A route / planner assets
  -> lanelet2_map.osm / map_projector_info.yaml / route_goal.yaml
  -> loader + lanelet_route_planner validation
  -> 可作为 Stage B closed-loop 输入

Stage 2B pointcloud / NDT localization assets
  -> pointcloud_map.pcd / pointcloud_map_metadata.yaml / sensor_extrinsic.yaml
  -> static geometry validation + NDT replay validation
  -> 可作为 Stage C localization 输入
```

Stage 2A `PASS` 不能外推为 Stage 2B `PASS`。从技术依赖看，Stage 2B 正式资产尚未发布不一定阻止 Stage B ground-truth planning closed-loop；但当前项目最大风险是定位，所以执行优先级应先做 Stage C0 localization-only validation。正式“完整 urban map build PASS 基线”仍必须同时包含 Stage 2A 和 Stage 2B 的可追溯结果，并通过原子发布写入 `asset_manifest.json`。

旧正式 urban map 目录仍是 `FAIL`：`autoracer_hooke/maps/carmaker_builtin_urban/runtime_summary.json` 来自 `urban_map_build_20260531_174150`，`raw_ndt_pose_count = 0`、`ndt_pose_rate_hz = 0.0`，不能作为当前定位 PASS 基线。它的价值是证明旧 `0.07m` static surface filter 的静态几何位置基本正确：`median = 0.03478m`、`mean = 0.03494m`、`rmse = 0.04046m`、`p95 = 0.06661m`、`p99 = 0.06931m`、`outlier_gt_1m_ratio = 0.0`。这说明点云与 CarMaker 静态道路、地形、建筑和墙面等 surface 的误差很小；当 NDT `Score = 0` 时，问题不应简单描述成“点云不准”，而应描述成：

```text
旧 PCD 表示 / 密度 / 局部 scan-map overlap 不满足 NDT。
```

后续实现已经补齐两类关键能力：

```text
build_pcd_map.py:
  --voxel-selection first / center_nearest / static_nearest / static_nearest_topk
  --points-per-voxel
  --voxel-candidate-limit

run_urban_map_build.sh:
  PCD_VOXEL_SELECTION 默认 static_nearest

diagnostics:
  validate_pcd_density.py
  validate_scan_map_overlap.py
  replay_ndt_validation.py
```

当前更准确的状态是“候选资产推进中，而不是从零修建图脚本”：

- route 4226 short-route 已有 localization 候选：`urban_map_build_short1km_phase1_userdefined_pandar40_lidar_built_diag_20260601_070843/pcd_density_validation.json`、`scan_map_overlap_validation.json`、`ndt_replay_validation.json` 均为 `PASS`；NDT 关键指标为 `raw_ndt_pose_count = 53`、`ndt_convergence_ratio = 0.7067`、`ndt_error_mean_m = 0.149m`、`ndt_error_max_m = 0.529m`；
- route 271 长路线新候选 `urban_map_build_route271_phase1_userdefined_pandar40_lidar_built_l055_20260601_105446` 已有 PCD density `PASS` 和 independent scan-map overlap `PASS`，overlap 关键指标为 `median_nn_distance_m = 0.228m`、`p95_nn_distance_m = 0.369m`、`overlap_ratio_lt_1_0m = 0.996`；但该目录尚缺完整 `runtime_summary.json` 和 NDT 回归 summary；
- `urban_map_build_d009_20260531_200856` 对 route 271 有 nominal `PASS`，但 NDT 只有 `raw_ndt_pose_count = 12`、`ndt_pose_rate_hz = 0.2195Hz`、`ndt_error_mean_m = 1.087m`、`ndt_error_max_m = 2.238m`，且缺严格 `quality_gate` 字段，不应替代 Stage C0 端到端定位验收。

因此后续重点不是再“新增 scan-map overlap 诊断”或“实现代表点策略”，而是把已实现的 density / overlap / NDT 诊断和 integrated voxel selection 固化为可重复回归：先确认 route 4226 的 localization-only PASS 能从干净输入复现，再把同一参数回归 route 271，并通过原子发布生成带 `asset_manifest.json` 的正式 map 目录。

为缩短迭代周期，PCD / scan-map overlap / NDT 参数和代表点策略的第一批候选应先在 `UrbanRoad_RuralRoad_Expressway.rd5` 的 short-route `4226 / 996.319m` 上验证。short-route 通过后，再把同一策略回归到 long-route `271 / 10803.772m`。不能只因为 short-route 通过就覆盖 long-route 正式基线。

当前工程中的参考实现是：

```text
SimProject_TianmenRace/run_urban_map_build.sh
SimProject_TianmenRace/tools/urban_map_build/
```

#### 2.4.7 PASS / FAIL 必须由运行事实判定

CarMaker 相关验收不得只看进程退出码、节点是否启动或文件是否生成。最小判定依据应同时包含：

- `carmaker.log` 中有 `SIM_END`，没有 `SIM_ABORT`；
- road、vehicle、sensor、terrain 没有 load error；
- ego 车有 route progress，且没有离路；
- LiDAR frames 和 points 非零，频率合理；
- rosbag 中关键 topic count 非零；
- map / route / projector / extrinsic 可被 `autoracer_hooke` 加载；
- planning 能产生 `/planning/trajectory`；
- control command 能经过 safety gate；
- `VehicleControl` 能驱动车辆动力学响应；
- 所有失败都能在 `runtime_summary.json` 中定位到 scene、sensor、map、localization、planning、control、safety 或 vehicle interface。

因此，后续实现者不需要反复证明“CarMaker 能不能被 AI 使用”。当前基线已经说明正确使用 CarMaker 的方式。下一步的重点是沿这条基线把 `autoracer_hooke` 的定位、规划、控制、安全门和车辆接口逐段接通，并用同样的 `runtime_summary.json` 机制证明每一段是否通过。

## 3. 软件需求

### 3.1 调通框架阶段真正需要的能力

在通用 CarMaker 场景中调通 `autoracer_hooke` 时，优先需要：

- 可复现的 CarMaker TestRun、road、vehicle、sensor 配置；
- 清理动态交通参与者后的静态场景；
- ground-truth pose、velocity、steering status；
- CarMaker 模拟 Fixposition 原生 ROS2 driver 话题，用于替代实车 Fixposition driver 输入；
- RoadEval / route / centerline 数据；
- 仿真 LiDAR `PointCloud2`，以及可解释的 frame / extrinsic；
- PCD map、Lanelet2 / route corridor、`map_projector_info.yaml`、`route_goal.yaml`；
- CarMaker 专用 simulation launch，禁用真实硬件节点；
- CarMaker 仿真车辆接口，把安全门输出送入 `VehicleControl`；
- headless CLI 验证脚本和 `runtime_summary.json`；
- 失败时能定位到 scene、sensor、map、localization、planning、control、safety、vehicle interface 中的具体链路。

当前 CarMaker Bridge 对 Fixposition 的仿真输出应按 AutoRacer 实车 localization 订阅契约直接提供，而不是通过旧 CHC430 话题转换：

```text
/fixposition/fix                    sensor_msgs/msg/NavSatFix
/fixposition/autoware_orientation   autoware_sensing_msgs/msg/GnssInsOrientationStamped
/fixposition/rawimu                 sensor_msgs/msg/Imu
/fixposition/odometry_enu           nav_msgs/msg/Odometry
/fixposition/fpa/odomstatus         fixposition_driver_msgs/msg/FpaOdomstatus
```

这些话题的用途是支撑 `autoware_gnss_poser`、`fixposition_seed_filter`、EKF / localization 诊断和状态质量判断。旧 `/chcnav/devpvt` 只能作为 legacy 兼容输出，不能作为新的定位设计依据。

### 3.2 最终竞速阶段需要的能力

面向天门山封闭赛道，最终还需要：

- 高精点云地图和赛道资产；
- LiDAR / NDT 主定位；
- Fixposition / IMU / 车辆状态辅助定位；
- EKF 状态融合，输出连续稳定的 pose / twist；
- 离线生成 racing line；
- 离线生成 speed profile；
- Local Planning：把 `/planning/global_trajectory` 或离线 racing line 裁剪、重采样并发布局部 `/planning/trajectory`；
- 高速轨迹跟踪控制；
- 简单、硬约束、可解释的安全门；
- Hooke2 实车接口和 CarMaker 仿真车辆接口。

### 3.3 明确不需要的城市栈能力

封闭赛道竞速不需要引入：

- 行为规划；
- 变道；
- 城市 mission planner；
- 交通灯、路口、人行横道规则；
- 行人 / 车辆检测、预测、跟踪；
- 复杂障碍物避让；
- 大而全的 `autoware_launch` / `tier4_*_launch`。

这些城市自动驾驶模块对封闭赛道竞速不是核心能力，反而会增加复杂度、降低可维护性。

## 4. 总体技术路线

### 4.1 仿真调通路线

当前阶段推荐先按以下路线调通系统：

```text
CarMaker cleaned built-in scene
  -> ground truth / RoadEval / vehicle status / LiDAR / Fixposition native topics
  -> map and route asset generation
  -> Stage C0 localization-only validation
     PointCloud2 + PCD map + GT initial pose + NDT -> pose
     /fixposition/* -> GNSS/INS/IMU input contract validation
  -> Stage B planning closed-loop
     route / goal / trajectory -> controller -> command_gate -> VehicleControl
  -> Stage C full sensor-localization closed-loop
     NDT / EKF localization -> planning / control / safety
```

这条路线的目标是证明框架能工作，不是证明天门山速度最优。

需要把两条验证线分开理解：

- 天门山 Stage A 控制闭环用于证明 `trajectory -> control -> safety -> VehicleControl -> vehicle dynamics` 已经可以低速工作；
- CarMaker 内置 urban 采集 / 建图线用于生成更通用的 LiDAR、PCD、Lanelet2、projector、goal、extrinsic 输入事实；
- 当前最高优先级是 Stage C0 localization-only validation：先让 CarMaker LiDAR scan、PCD map、GT initial pose 和 NDT 在离线 / 旁路模式下稳定输出定位；同时验证 CarMaker 模拟的 `/fixposition/*` 能满足 `autoracer_hooke` localization 的订阅契约；
- Stage C0 不启动 planning、controller、command_gate 或 VehicleControl，避免定位问题被控制闭环现象掩盖；
- 只有 urban map build 的正式资产目录、summary 和 manifest 原子一致，且 static geometry、planner validation、NDT replay 和后续 closed-loop 都通过后，才能说内置场景中的规划 / 定位链路通过；
- 不能用天门山 Stage A 的 PASS 代替 urban map build PASS，也不能用 urban collection PASS 代替 Stage B / C closed-loop PASS。
- planner asset validation 和少量 NDT replay pose 只能证明资产可加载、链路可调试，不能替代 Stage B / Stage C 在线闭环验收。

### 4.2 最终竞速路线

最终面向天门山的链路为：

```text
LiDAR + CarMaker 模拟 Fixposition 原生话题 + vehicle status
  -> pointcloud_preprocessor
  -> NDT / LiDAR localization
  -> autoware_gnss_poser / Fixposition seed / IMU input
  -> EKF state estimation
  -> race_trajectory_provider
  -> MPC lateral + PID longitudinal control
  -> command_gate / race_safety
  -> hooke2_interface 或 CarMaker simulation vehicle interface
  -> vehicle dynamics
```

CarMaker 中保持相同算法边界：

```text
CarMaker sensors / ground-truth / vehicle state
  -> ROS2 Bridge / simulation adapter
     (/fixposition/*, /sensing/lidar/concatenated/pointcloud, vehicle status, diagnostic ground truth)
  -> autoracer_hooke
  -> command_gate / race_safety
  -> CarMaker simulation vehicle interface
  -> VehicleControl
```

定位话题所有权必须随阶段切换显式定义：

- Stage A / Stage B：CarMaker Bridge 可以作为 ground-truth localization 源，发布 `/localization/pose_with_covariance`；
- Stage C0：可以拆成两个互不混淆的子模式：
  - NDT isolated：只验证 LiDAR/PCD/NDT，CarMaker ground truth 只用于 `/localization/ndt_initial_pose` 和误差评估；
  - Fixposition contract：只验证 CarMaker 模拟 `/fixposition/*` 能驱动 `autoware_gnss_poser`、seed filter 和 EKF 输入契约；
- Stage C：NDT / EKF 必须成为 `/localization/pose_with_covariance` 的唯一算法定位源；Fixposition 原生话题作为 GNSS/INS/IMU 输入或初始位姿 / 状态质量来源；
- Stage C0 / Stage C 中 CarMaker ground truth 应改发到 `/carmaker/ground_truth/pose` 或等价诊断 topic，只用于 initial pose、误差评估和 replay 对齐，不得与 NDT / EKF 同时发布同一个 localization topic；
- 如果同一 topic 出现多个 localization publisher，本阶段判为 `FAIL`，不能用“看起来能跑”作为验收依据。

允许的定位源组合必须写入每次 `runtime_summary.json`：

| 阶段 / 模式 | `/localization/pose_with_covariance` 发布源 | `/fixposition/*` 用途 | CarMaker ground truth 用途 |
| --- | --- | --- | --- |
| Stage A / B ground-truth closed-loop | CarMaker Bridge | 可关闭或只做旁路诊断 | 主定位源 + 诊断 |
| Stage C0 NDT isolated | NDT | 可关闭，避免增加变量 | NDT initial pose + 误差评估 |
| Stage C0 Fixposition contract | 不要求最终定位输出；可验证 `/sensing/gnss/pose_with_covariance` | 主测试对象，必须来自 CarMaker 模拟 Fixposition 原生话题 | 诊断 / 对齐参考 |
| Stage C full localization | NDT / EKF 唯一算法源 | GNSS/INS/IMU/状态质量输入 | 只做诊断，不抢 localization topic |

控制话题必须显式对齐：

- 实车侧 canonical output：`/control/command/control_cmd`；
- 当前 CarMaker Bridge input 已对齐到 `/control/command/control_cmd`；
- `/control/control_cmd` 是旧路径，不能再作为新计划或新验收的默认控制入口。

### 4.3 核心原则

1. 仿真是第一阶段硬门槛，不是可选项；
2. 先证明模块契约和闭环事实，再追求赛道速度；
3. CarMaker 负责 plant，`autoracer_hooke` 负责算法；
4. 不用 IPGDriver 替代 ego 自动驾驶；
5. 不直接改写车辆位姿；
6. 不搬完整城市 Autoware 栈；
7. 只引入对调通框架和竞速必要的成熟基础模块；
8. 每个阶段都必须用日志、topic、指标和运行结果证明。

## 5. 已引入的必要基础模块

以下模块已经作为成熟基础能力引入或纳入构建范围。它们只是基础能力，不改变 `autoracer_hooke` 面向封闭赛道竞速的整体架构。

定位 / 地图：

```text
autoware_ndt_scan_matcher
autoware_ekf_localizer
autoware_kalman_filter
autoware_gnss_poser
autoware_map_loader
autoware_map_projection_loader
autoware_localization_util
```

高速控制：

```text
autoware_trajectory_follower_node
autoware_trajectory_follower_base
autoware_mpc_lateral_controller
autoware_pid_longitudinal_controller
autoware_interpolation
autoware_motion_utils
autoware_osqp_interface
autoware_pure_pursuit
autoware_vehicle_info_utils
```

点云预处理：

```text
autoware_pointcloud_preprocessor
autoware_pcl_extensions
autoware_point_types
managed_transform_buffer
```

当前状态需要分清：

- 源码存在或能构建，不等于已经在目标 launch 中正确接线；
- 节点能启动，不等于 topic、frame、时间戳和消息契约正确；
- 仿真 low-speed smoke test 通过，不等于高速竞速闭环通过；
- NDT、EKF、MPC/PID、安全门质量判定仍需要逐项接入和验证。
- 当前 Python pure pursuit、Stage A trajectory provider 和 command_gate 是低速闭环验证工具，不是最终竞速控制 / 规划 / 安全方案。
- 当前 `autoracer_hooke/maps/carmaker_builtin_urban/` 下资产可用于排障和后续修正；但其 `runtime_summary.json` 显式为 `FAIL` 且 `raw_ndt_pose_count = 0`。在重新发布原子一致的 `PASS` 基线前，不能作为已验收地图基线。

## 6. 仍需新增或补齐的关键能力

### 6.1 先补齐仿真调通能力

在新增比赛专用 planner 之前，必须先补齐仿真调通链路：

```text
CarMaker scene / vehicle / sensors
  -> data collection and map generation
  -> simulation launch
  -> simulation vehicle interface
  -> stage-by-stage validation
```

需要的能力包括：

- CarMaker 内置场景清理和独立 TestRun；
- 传感器车配置，至少包含 ground truth、vehicle status、LiDAR；
- LidarRSI / GPUSensor 到 `PointCloud2` 的正确转换；
- PCD / Lanelet2 / projector / goal / extrinsic 生成；
- route / map / pointcloud / static geometry 对齐报告；
- CarMaker 专用 launch，显式关闭真实 LiDAR、真实 Fixposition driver、CAN、Hooke2 硬件 IO；Fixposition 输入由 CarMaker Bridge 的 `/fixposition/*` 模拟源提供；
- 仿真车辆接口，按 Autoware / Hooke2 控制语义进入 `VehicleControl`；
- headless 验证脚本，输出 `runtime_summary.json`。

这部分仍然是 CarMaker 验证基础，但当前最高优先级应收敛到定位：先把 Stage C0 localization-only validation 做通。

结合当前代码和日志，下一步应优先补齐：

1. 建立 Stage C0 localization-only 验证入口：只启动 `pointcloud_map_loader`、CarMaker `PointCloud2` replay/live 输入、GT initial pose publisher、NDT 和 ground-truth 对比统计，不启动 planning/control/VehicleControl；
2. 建立 Stage C0 Fixposition contract 验证入口：关闭真实 Fixposition driver，由 CarMaker Bridge 发布 `/fixposition/fix`、`/fixposition/autoware_orientation`、`/fixposition/rawimu`、`/fixposition/odometry_enu`、`/fixposition/fpa/odomstatus`，验证 `autoware_gnss_poser`、`fixposition_seed_filter` 和 EKF 输入侧 topic 均可消费；
3. 明确定位输入契约：`pointcloud_map.pcd`、`pointcloud_map_metadata.yaml`、`/sensing/lidar/concatenated/pointcloud`、`/localization/ndt_initial_pose`、`/map/get_differential_pointcloud_map`、`/carmaker/ground_truth/pose`、`/fixposition/*` 必须存在且 frame / stamp 可解释；
4. 先解决 localization topic ownership：CarMaker ground truth 在 Stage C0/C 中只能发诊断 topic，NDT/EKF 才能发布 `/localization/pose_with_covariance`，避免同一 topic 多发布源；如果临时使用 CarMaker ground truth 作为定位源，必须显式标记为 ground-truth mode；
5. 保留 `SimProject_TianmenRace/logs/urban_collection_20260531_171753/rosbag` 作为 route 271 / 10.8km 长路线回归基线；已建立的 route 4226 / 996.319m short-route 继续用于快速迭代，输出必须保持在独立 `urban_collection_short1km_*` / `urban_map_build_short1km_*` 目录；
6. 使用已实现的 scan-map overlap 和 PCD density 诊断：在 ground-truth pose 下把 replay 的 LiDAR scan 转到 `map` frame，统计 scan 点到 PCD map 的最近邻距离、有效点比例、距离分布和失败帧，先判断 NDT `Score = 0` 是 overlap 问题还是 NDT / PCD representation 问题；
7. 针对 Stage 2B，继续验证和调参已实现的 `build_pcd_map.py` integrated voxel/static-nearest 代表点策略，目标是同时满足 static geometry gate、scan-map overlap 和 NDT replay gate；
8. 定位-only 证据可复现后，稳定现有 Stage B headless closed-loop 入口：Stage B 默认 `AutoracerStageB_UrbanRoute271`，route 271 / 10803.772m；`MIN_DISTANCE_M=500` 是 first-500m smoke gate，不是 route；必须证明 `/planning/trajectory` 来自 `local_trajectory_planner` 而不是 Stage A RoadEval trajectory provider；
9. Stage B 已启用 trajectory freshness / planning output timeout；后续补齐 NDT / EKF quality、横向误差、曲率限速和 overspeed 判定；
10. 修正 map build 的发布方式：所有资产先写入 `logs/urban_map_build_<timestamp>/map_candidate/` 或 `logs/urban_map_build_short1km_<timestamp>/map_candidate/`，候选目录完成对应 validation 后再原子发布；short-route 如需正式发布应使用独立目录，例如 `autoracer_hooke/maps/carmaker_builtin_urban_short1km/`，不能覆盖 long-route 的 `autoracer_hooke/maps/carmaker_builtin_urban/`；
11. 把 Bridge / `User.cpp` 中的车辆参数、转向比、限幅、timeout、topic 名称、Fixposition topic 开关和 LiDAR 外参逐步参数化，避免硬编码扩散到最终竞速链路。

### 6.2 再新增比赛专用模块

后续最关键的比赛专用模块是：

```text
autoracer_race_planning
```

它不应从大仓搬运，而应在 `autoracer_hooke` 中按比赛需求干净实现。注意当前 `autoracer_planning` 已经新增 `local_trajectory_planner`，负责把 `/planning/global_trajectory` 裁剪、重采样为局部 `/planning/trajectory`；后续 `autoracer_race_planning` 不应重复实现这条通用 Local Planning 链路，而应专注于 racing line / speed profile 的比赛资产来源。

但该模块应在 Stage A/B/C 的基础契约通过后再实现。若定位、地图、trajectory 来源、安全门或车辆接口仍未验收，提前实现 racing line provider 会把问题从接口正确性转移到策略调参，反而增加排障成本。

职责：

```text
离线 racing line + speed profile
  -> /planning/global_trajectory 或等价 race global path
  -> local_trajectory_planner
  -> 在线发布局部 /planning/trajectory
```

它应包含：

- 读取预生成 racing line；
- 读取预生成 speed profile；
- 根据当前定位找到最近赛道进度；
- 裁剪前方局部 horizon；
- 发布 Autoware `Trajectory`；
- 每个 trajectory point 携带位置、航向、曲率相关速度；
- 支持按定位质量、横向误差、曲率进行简单速度降级。

它不应变成新的城市规划器，也不应实现变道、避障、交通规则等与比赛无关的逻辑。

## 7. 资产生成思路

### 7.1 通用仿真资产

为了先调通框架，应先从成熟 CarMaker 场景生成通用验证资产：

```text
CarMaker road / terrain / buildings / static objects
  -> clean TestRun
  -> IPGDriver collection vehicle 或 scripted collection vehicle
  -> ground truth pose / vehicle status / LiDAR / RoadEval
  -> rosbag / mcap
  -> PCD map
  -> Lanelet2 route corridor
  -> map_projector_info.yaml
  -> route_goal.yaml
  -> sensor_extrinsic.yaml
  -> alignment_report.txt
  -> runtime_summary.json
```

采集阶段允许 IPGDriver 驾驶，因为它只是采集车司机。闭环验证阶段不允许 IPGDriver 接管 ego 控制。

资产生成必须满足：

- PCD 来自 CarMaker 原生传感器数据，不能复制其他场景地图；
- Lanelet2 / route corridor 来自当前 CarMaker road / RoadEval / route samples；
- `map_projector_info.yaml` 与 pose、Lanelet2、PCD 使用同一坐标约定；
- `route_goal.yaml` 可复现，不能依赖 RViz 人工点选；
- `alignment_report.txt` 必须说明坐标、误差、采样、失败原因；
- `asset_manifest.json` 必须记录输入 collection log dir、输入 bag、map build log dir、生成时间、PCD 点数、route id、validation 状态和正式发布目标；
- 正式 map 目录只能接收完整 `PASS` 资产包，不应被失败或中断的 map build 部分覆盖；
- 文件生成成功不等于 PASS，必须通过加载、规划、定位或闭环验证。

### 7.2 天门山竞速资产

竞速能力主要来自离线资产，而不是复杂在线规划：

```text
天门山点云 / RoadEval / 路沿 / 轨迹采集数据
  -> 赛道中心线和边界
  -> racing line
  -> 曲率
  -> 坡度
  -> speed profile
  -> 在线 trajectory provider
```

推荐资产：

```text
race_line.csv
speed_profile.csv
track_boundary.csv
curvature_profile.csv
race_asset_validation.json
```

速度剖面应至少考虑：

- 曲率；
- 最大横向加速度；
- 最大纵向加速度；
- 制动能力；
- 坡度；
- 弯前减速距离；
- 出弯加速。

在线运行时不应重新做复杂优化，只应读取和裁剪离线结果，保持系统简单、稳定、可验证。

## 8. 控制与安全策略

最终控制链路应使用：

```text
/planning/trajectory
  -> autoware_trajectory_follower_node
  -> MPC lateral controller
  -> PID longitudinal controller
  -> /autoracer/control/raw_control_cmd
  -> command_gate / race_safety
  -> /control/command/control_cmd
```

当前 Python pure pursuit 可以保留为低速 smoke test，但不应作为最终竞速主控制器。推荐推进顺序是：

1. 先用 Python pure pursuit 验证 trajectory、pose、vehicle interface 和 safety gate；
2. 再接入 `autoware_trajectory_follower_node`；
3. 再调 MPC lateral / PID longitudinal；
4. 最后提高速度并引入竞速 speed profile。

当前代码中的 pure pursuit 只消费 `/planning/trajectory`、`/localization/pose_with_covariance` 和 `/vehicle/status/velocity_status`，输出 `/autoracer/control/raw_control_cmd`。它适合证明消息契约、方向、速度和 `VehicleControl` 接入，但不应承担高速弯道稳定性、横向加速度约束或最终竞速控制调参。

安全门保持简单：

- trajectory 丢失：停车；
- localization 超时：停车；
- control command 超时：停车；
- NDT / EKF 质量差：降速或停车；
- 横向误差过大：降速或停车；
- 当前速度超过曲率允许速度：限速；
- steering / acceleration 超限：硬限幅。

安全门不应隐藏问题，不应写复杂 fallback，不应自动切换到未知控制源。每次降速或停车都必须能从日志解释原因。

当前 `command_gate` 的已实现能力主要是 drive enable、raw command timeout、localization timeout、trajectory freshness / planning output timeout、速度 / 加速度 / 转角 / 转角速率限幅和 gear / hazard 支持命令。它还没有完整实现 NDT / EKF 质量、横向误差、曲率限速和 overspeed 判定。因此：

- Stage A 可以用当前 `command_gate` 作为低速安全门；
- Stage B 必须保持 `require_trajectory=true` 和 `trajectory_timeout_sec` 生效，planner 停止后应停车并在 `/autoracer/safety/state` 中输出 `trajectory_timeout`；
- Stage C 需要接入定位质量和定位源状态；
- 高速竞速前必须增加横向误差、曲率速度上限和车辆动态限幅；
- 每一项安全门新增逻辑都必须有状态 topic、日志和 `runtime_summary.json` 指标，不能只在控制输出中静默限速。

## 9. CarMaker 验证策略

CarMaker 验证是当前最重要的第一阶段。当前排序以定位风险为核心：Stage A 已证明低速控制接口可用；下一步优先做 Stage C0 定位-only 验证；定位链路清楚后再做 Stage B 规划闭环和 Stage C 完整传感器定位闭环。

### 9.1 阶段 0：场景和传感器可用性

目标：

```text
证明 CarMaker 场景、道路、车辆、传感器和数据发布链路可运行。
```

验收：

- CarMaker 能 headless 加载 TestRun / road / vehicle / terrain；
- 动态交通参与者已禁用或不会影响采集；
- ego 车可以沿 route 运行；
- ground truth pose、velocity、steering status 可持续发布；
- LiDAR hit count 非零且 `PointCloud2` frame / 点数 / 频率可解释；
- 无 `SIM_ABORT`、资源缺失、license 或 sensor init 错误。

### 9.2 阶段 A：控制闭环

目标：

```text
CarMaker ground-truth pose
  -> CarMaker-aligned trajectory
  -> controller
  -> safety gate
  -> simulation vehicle interface
  -> VehicleControl
  -> CarMaker vehicle dynamics
```

此阶段不验证 Lanelet2、NDT、EKF、Fixposition 或真实 LiDAR 定位。它只验证控制、安全门、车辆接口和动力学闭环。

当前已有低速 PASS 证据，但其范围必须保持清楚：

- trajectory 来源是 CarMaker RoadEval rolling centerline；
- localization 来源是 CarMaker ground truth；
- 控制器是 Python pure pursuit；
- safety gate 输出到当前 CarMaker Bridge input `/control/command/control_cmd`；
- 通过不代表最终 MPC/PID、高速控制或传感器定位通过。

### 9.3 阶段 C0：CarMaker 定位-only 验证

目标：

```text
模式 A：NDT isolated
  CarMaker LiDAR / GPUSensor 或已采集 rosbag
    -> /sensing/lidar/concatenated/pointcloud
    -> pointcloud_map_loader
    -> GT initial pose
    -> NDT
    -> /localization/pose_with_covariance
    -> 与 CarMaker ground truth 对比

模式 B：Fixposition contract
  CarMaker runtime vehicle state
    -> CarMaker Bridge 模拟 Fixposition 原生话题
    -> autoware_gnss_poser / fixposition_seed_filter / EKF input
    -> /sensing/gnss/pose_with_covariance 或等价 localization 输入链路可观测
```

这是当前第一优先级。它只验证定位输入和定位算法，不启动 planning、controller、command_gate、VehicleControl 或 closed-loop ego 控制。允许先隔离 NDT 问题单独测试；也允许在另一个子模式中只验证 CarMaker 模拟 Fixposition 原生话题是否满足 AutoRacer localization 订阅契约。两种模式的结果都必须写入 `runtime_summary.json`，不能互相替代。

进入 Stage C0 的最低输入是：

```text
SimProject_TianmenRace/logs/urban_collection_20260531_171753/rosbag
autoracer_hooke/maps/carmaker_builtin_urban/pointcloud_map.pcd
autoracer_hooke/maps/carmaker_builtin_urban/pointcloud_map_metadata.yaml
/sensing/lidar/concatenated/pointcloud
/localization/ndt_initial_pose
/map/get_differential_pointcloud_map
/carmaker/ground_truth/pose
/fixposition/fix
/fixposition/autoware_orientation
/fixposition/rawimu
/fixposition/odometry_enu
/fixposition/fpa/odomstatus
```

最小实现原则：

- 直接用 CarMaker ground truth pose 生成 `/localization/ndt_initial_pose`，先证明 scan-to-map NDT 能工作；
- ground truth 不得与 NDT 同时发布 `/localization/pose_with_covariance`；
- NDT 输出可以先只接 `/localization/pose_with_covariance` 和 `/localization/pose`，不接规划 / 控制；
- Fixposition contract 模式中，真实 `fixposition_driver_ros2_exec` 必须关闭，`/fixposition/*` 必须只有 CarMaker Bridge 一个模拟发布源；
- Fixposition contract 模式的最低验收不是 NDT 收敛，而是 AutoRacer localization 侧订阅链路能看到并消费 `/fixposition/fix`、`/fixposition/autoware_orientation` 和 `/fixposition/fpa/odomstatus`，并能产出 `/sensing/gnss/pose_with_covariance` 或对应 seed / EKF 输入；
- 每次 replay 必须记录 NDT 输出 pose 数量、rate、覆盖时间段、覆盖距离、误差统计和失败原因；
- 如果 NDT `Score = 0` 或无输出，先做 scan-map overlap 诊断，不直接调松阈值。

Stage C0 PASS 的最低标准：

- `/sensing/lidar/concatenated/pointcloud` frame、stamp、点数、频率可解释；
- `pointcloud_map_loader` map service 可用；
- `/localization/ndt_initial_pose` 使用 `map` frame，时间戳与 replay / sim time 一致；
- `/localization/pose_with_covariance` 只有 NDT 一个算法发布源；
- `/fixposition/*` 话题在 Fixposition contract 模式中只有 CarMaker Bridge 一个发布源，frame、stamp、频率和消息类型与 AutoRacer localization 订阅配置一致；
- `autoware_gnss_poser`、`fixposition_seed_filter` 或 EKF 输入侧 topic 至少有一个可观测输出，且其来源在 summary 中标记为 `carmaker_fixposition_sim`；
- NDT pose 连续输出，pose rate、时间覆盖、空间覆盖和误差指标写入 `runtime_summary.json`；
- NDT 与 CarMaker ground truth 的误差在低速 / replay 条件下可解释。

如果 Stage C0 失败，优先定位失败层：

```text
输入 topic 缺失 / stamp 错误
  -> frame 或 TF 错误
  -> initial pose 错误
  -> scan-map overlap 不足
  -> PCD 代表点 / 密度 / 结构不适合 NDT
  -> NDT 参数再微调
```

不要跳过这个阶段直接做 Stage B 或 Stage C full closed-loop，否则规划和控制现象会掩盖定位根因。

### 9.4 阶段 B：规划闭环

目标：

```text
CarMaker ground-truth pose
  -> route_goal_publisher
  -> lanelet_route_planner
  -> /planning/global_trajectory
  -> local_trajectory_planner
  -> /planning/trajectory
  -> controller
  -> safety gate
  -> VehicleControl
```

此阶段必须证明 trajectory 来源明确，且与当前 CarMaker road 同坐标。不能用阶段 A 的简单 trajectory provider 冒充规划闭环通过。

Stage B 依赖的是 route / planner 资产，不依赖 NDT 定位收敛。因此从技术依赖看，即使正式 route 271 NDT 资产尚未发布，也不必阻塞 Stage B ground-truth planning closed-loop。但当前项目最大风险仍是定位，所以执行优先级上应先完成 Stage C0，再回来做 Stage B closed-loop。进入 Stage B 的最低输入是：

```text
Stage 2A route / planner assets validation PASS
  -> lanelet2_map.osm
  -> map_projector_info.yaml
  -> route_goal.yaml
  -> lanelet_route_planner 能发布 /planning/global_trajectory
  -> local_trajectory_planner 能发布 /planning/trajectory
  -> global/local trajectory 与 CarMaker route samples 同坐标、同方向、长度可解释
```

当前 `urban_map_build_20260531_174150` 已经给出很强的 Stage 2A 候选证据：trajectory 点数、横向误差、航向误差和 route coverage 均正常。但这仍只是 planner asset validation，不是 Stage B closed-loop PASS。

推荐最小实现，不引入比赛 planner：

```text
carmaker_stage_b.launch.py
  -> route_goal.yaml 自动发布器
  -> lanelet_route_planner
  -> /planning/global_trajectory
  -> local_trajectory_planner
  -> /planning/trajectory
  -> pure_pursuit_controller
  -> command_gate(output 到 /control/command/control_cmd)
```

Stage B 中不得启动 `carmaker_trajectory_provider`。它可以继续使用 CarMaker ground-truth pose，但 `/planning/global_trajectory` 必须来自 map / route / goal 规划入口，`/planning/trajectory` 必须来自 `local_trajectory_planner`。

当前 Stage B 入口已存在：

```text
SimProject_TianmenRace/run_autoracer_stage_b_headless.sh
  TESTRUN 默认 AutoracerStageB_UrbanRoute271
  EXPECTED_ROUTE_LENGTH_M 默认 10803.772
  MIN_DISTANCE_M 默认 500

SimProject_TianmenRace/Data/TestRun/AutoracerStageB_UrbanRoute271
  Vehicle.Routing.ObjId = 271
  Vehicle.StartPos.ObjId = 271
```

这里 `MIN_DISTANCE_M=500` 是 route 271 上的 first-500m smoke gate，用于降低首次闭环验收成本；它不是 CarMaker 的 500m route，也不替代 route 4226 / 996.319m short-route 定位基线。

验收时必须证明：

- `/planning/global_trajectory` 来自 `lanelet_route_planner`；
- `/planning/trajectory` 来自 `local_trajectory_planner`；
- `map_projector_info.yaml`、Lanelet2、route goal 和 ground-truth pose 共用同一 CarMaker / ROS `map` 坐标；
- 自动发布或加载的 goal 可复现，不依赖 RViz 手点；
- global/local trajectory 的起点、终点、方向、长度和速度字段可解释；
- Stage A 已通过的 control / safety / vehicle interface 未被替换成新的未验证路径。
- `runtime_summary.json` 应显式检查没有 `/carmaker_trajectory_provider` 节点，且 topic sample / log 能证明来源是 `lanelet_route_planner` 和 `local_trajectory_planner`。
- 最近一次 `autoracer_stage_b_20260531_235645` 因 `GPUSensor A1:localhost.1: Timeout during initialization` 后 `SIM_ABORT`，且没有 `runtime_summary.json`；它只能作为失败样本，不能作为 Stage B 验收证据。

如果 planner asset validation 通过但闭环未运行，只能说“规划资产可加载 / 可出轨迹”，不能说 Stage B PASS。
如果正式 route 271 NDT 回归未完成但 ground-truth pose、route assets 和 planner validation 均正常，技术上可以继续做 Stage B；但按当前定位优先策略不应把它放在 Stage C0 之前。若后续因排期选择先做 Stage B，`runtime_summary.json` 必须明确写出定位来源是 CarMaker ground truth，不能暗示传感器定位已经通过。

### 9.5 阶段 C：完整传感器定位闭环

目标：

```text
CarMaker LiDAR / GPUSensor
  -> PointCloud2
  -> pointcloud_preprocessor
  -> NDT
CarMaker Fixposition native topics
  -> /fixposition/fix
  -> /fixposition/autoware_orientation
  -> /fixposition/rawimu
  -> /fixposition/odometry_enu
  -> /fixposition/fpa/odomstatus
  -> EKF
  -> /localization/pose_with_covariance
  -> planning / control / safety
```

此阶段把 Stage C0 已验证的定位链路接入在线闭环，不应推翻阶段 A 已验证的车辆接口，也不应推翻阶段 B 已验证的规划契约。

Stage C 进入闭环前必须先通过 Stage C0 旁路验证：

- 保留 ground-truth pose 作为评估参考；
- LiDAR / GPUSensor 发布真实 hit-derived `PointCloud2`；
- CarMaker Bridge 发布 Fixposition 原生话题，关闭真实 Fixposition driver，避免同名 topic 多发布源；
- NDT / EKF 输出独立 topic，并与 ground truth 对比；
- 只有当 pose 数量、rate、时间覆盖、空间覆盖和误差指标稳定后，才把 planner / controller 的 localization 输入切到 NDT / EKF；
- 切换后 `/localization/pose_with_covariance` 只能有 NDT / EKF 一个算法发布源，ground truth 只能作为诊断 topic；Fixposition 原生话题只能作为 GNSS/INS/IMU/状态质量输入，不直接抢最终 localization topic。

Bridge 需要支持显式定位源模式：

```text
Stage A/B:
  /carmaker/ground_truth/pose              诊断
  /localization/pose_with_covariance       ground-truth localization

Stage C0/C:
  /carmaker/ground_truth/pose              诊断 / 误差评估
  /fixposition/*                           CarMaker 模拟 Fixposition 原生输入
  /sensing/gnss/pose_with_covariance       autoware_gnss_poser 输出，可作 seed / EKF 输入
  /localization/pose_with_covariance       NDT / EKF 唯一算法定位输出
```

当前 Stage 2B 的真实卡点已经从“没有代表点策略 / 没有 overlap 诊断”推进为“候选资产回归和正式发布”：route 4226 short-route 的 density / overlap / NDT replay 已有 PASS 候选，route 271 新候选已有 density / independent overlap PASS，但仍需要完整 NDT 回归和 Stage C0 runtime summary。Stage C 前置工作应优先复现这些候选结果、固定参数和发布流程，而不是调松门禁。

少量 NDT replay pose 或离线误差统计不能替代在线闭环。Stage C PASS 必须同时看到 LiDAR、preprocessing、NDT/EKF、planning、raw control、safety gate、VehicleControl 和车辆响应。

### 9.6 阶段 D：天门山竞速专项

在通用 CarMaker 场景中证明框架工作后，再进入天门山专项：

- 建立天门山 CarMaker scene / road / terrain / sensor 资产；
- 生成天门山 PCD / route / boundary；
- 生成 racing line 和 speed profile；
- 在仿真中验证稳定定位、稳定跟踪和速度策略；
- 再迁移到实车和真实天门山测试。

### 9.6 验收原则

验收不看节点是否启动，而看闭环事实：

- `/localization/pose_with_covariance` 稳定；
- `/planning/trajectory` 稳定；
- raw control 稳定；
- safety gate 放行；
- 控制命令进入车辆接口；
- `VehicleControl` 收到可解释控制输入；
- CarMaker / 实车车辆实际响应；
- 车辆没有离路；
- 日志能解释定位、轨迹、控制、限速、停车原因；
- `runtime_summary.json` 能给出 PASS / FAIL / BLOCKED 结论。

每个 `runtime_summary.json` 的 failure reason 必须与指标一致。例如某指标实际通过时，不应仍出现在 failed checks 中；否则会误导下一位实现者回到错误链路排障。

## 10. 最终期望状态

`autoracer_hooke` 最终应是一个干净的封闭赛道竞速栈，并且先在仿真中被证明可运行：

```text
CarMaker or real sensors
  -> pointcloud preprocessing
  -> NDT
  -> EKF
  -> racing line trajectory provider
  -> MPC + PID trajectory follower
  -> race safety gate
  -> Hooke2 / CarMaker vehicle interface
  -> vehicle dynamics
```

判断方案是否正确的标准只有一个：

```text
先在 CarMaker 中用可复现场景证明 autoracer_hooke 的每个模块和端到端闭环能正常工作；
再迁移到实车；
最后在天门山场景中稳定定位、稳定跟踪 racing line，并以尽可能高的速度安全完成赛道。
```
