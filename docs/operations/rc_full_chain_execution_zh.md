# RC 全链路执行总控

定位：这是 RC 验证 Hooke/Autoware 链路的执行控制文件。它不替代架构审计、建图流程或上车手册，而是把这些工作按依赖顺序串起来，确保每一步都有入口、产物和验收标准。

目标：让 RC 车尽可能跑通 Hooke 共享 upper stack，形成可复现的传感器采集、建图、地图回灌、定位、规划、控制、gate 和底盘适配验证流程。

## 执行原则

- 先审计差异，再执行链路；不在未确认接口和 topic 的情况下继续堆临时流程。
- RC 不新增独立上层算法；差异收敛在 sensing/profile、车辆参数、外参、地图资产生产和 vehicle adapter。
- 固化脚本只服务长期全流程；一次性排查命令不升格为正式脚本。
- x86 工作机负责 bag 检查、Super-LIO 建图、地图打包和静态检查；ARM 车端主机负责传感器、定位、upper stack、vehicle adapter 和实车验证。
- 没有 Lanelet2 地图时只验证 localization-only；没有底盘动力时只验证节点、topic、TF 和零速输出。

## 总链路

```text
架构/Launch 差距审计
  -> 车端传感器和 TF
  -> 建图 bag 录制
  -> 工作机 bag 检查和 Foxglove 查看
  -> Super-LIO 生成 PCD
  -> Autoware 地图打包
  -> 地图回灌车端
  -> NDT localization-only
  -> planning/control/gate dry-run
  -> RC UART adapter
  -> 低速动态验证
```

## 执行控制表

| 阶段 | 正式入口 | 必须产物 | 通过标准 | 当前状态 |
| --- | --- | --- | --- | --- |
| 差距审计 | `docs/architecture/runtime_alignment_audit_zh.md` | Hooke/RC 模块、launch、topic、平台差异表 | 允许差异和实现缺口分清；NDT 输入、adapter 边界、shared upper stack 无歧义 | 已建立，后续随 launch 变化维护 |
| 车辆参数和接口事实 | `docs/reference/calibration_zh.md`、`docs/reference/interfaces_and_topics_zh.md` | 车辆尺寸、轮径、轴距、外参、topic 契约 | RC 参数和 Autoware-facing topic 有唯一来源 | 已建立，实测后补标定值 |
| 车端传感器启动 | `./scripts/rc/rc_start_sensors.sh` | 点云、IMU、TF | `/sensing/lidar/concatenated/pointcloud`、`/imu/data_raw`、`/imu/data`、`/tf_static` 可用 | 待车端 live 验证 |
| 传感器输入检查 | `./scripts/check_mapping_inputs.sh` | 输入检查日志 | 点云 frame、字段、频率和 IMU/TF 满足建图输入 | 待车端 live 验证 |
| 建图 bag 录制 | `./scripts/rc/rc_capture_mapping_bag.sh` 或 start/stop bag 脚本 | 原始 bag | 必录 topic 完整，时间戳连续，原始 bag 不覆盖 | 待录制 |
| 工作机 bag 查看 | `rc_mapping_ws/view_bag_foxglove.sh` | 人工检查结论 | Foxglove 可看到点云、IMU、TF 时间轴；异常先回到采集阶段 | 工具入口已规划，待实包验证 |
| Bag 结构检查 | `rc_mapping_ws/inspect_bag_topics.sh` | checked bag 或检查报告 | 建图必录 topic、消息类型、频率通过检查 | 待实包验证 |
| Super-LIO 建图 | `rc_mapping_ws/run_mapping_pipeline.sh <bag> <run_id>` | `runs/<run_id>/`、PCD、日志、报告 | 离线 replay 不崩溃；运动 bag 能输出有效 PCD | 待运动 bag |
| Autoware 地图打包 | `rc_mapping_ws/package_autoware_map.sh` | `autoware_maps/<map_name>/` | PCD metadata、Lanelet2、projector 文件齐全；localization-only 地图不得冒充导航地图 | 待地图资产 |
| 地图回灌 | `rc_mapping_ws/sync_map_to_vehicle.sh <map_name>` | 车端地图目录 | `MAP_PATH` 指向可加载地图目录 | 待地图资产 |
| Localization-only | `./scripts/rc/rc_start_localization.sh` | NDT pose、`map -> base_link` TF | `/initialpose` 后 `/localization/pose_with_covariance` 和 `/localization/kinematic_state` 稳定 | 待地图和车端验证 |
| Full Autoware dry-run | `./scripts/rc/rc_start_autoware.sh` | trajectory、raw control、gated command | `ENABLE_DRIVE_COMMANDS=false` 时 upper stack 连通且底盘输出保持安全禁用 | 待地图和车端验证 |
| 底盘适配验证 | `rc_serial_interface`、`./scripts/request_autonomous_mode.sh` | `/vehicle/status/*`、UART command | 速度符号、转角方向、gear、control mode 与 STM32 协议一致 | 待底盘供电验证 |
| 低速动态验证 | `docs/operations/rc_runbook_zh.md` | bag/log/问题清单 | 低速短路线能停车、转向、跟踪，问题能归属到 sensing/localization/planning/control/gate/adapter | 待场地和动力条件 |

## 当前缺口

| 模块 | 缺口 | 解除条件 |
| --- | --- | --- |
| 地图资产 | 有效运动 bag、PCD、`lanelet2_map.osm`、`map_projector_info.yaml` 尚未形成 | 完成运动采集、Super-LIO 建图、Lanelet2 标注或导入 |
| 车端 live 数据 | 当前文档任务未重新采集 live topic 证据 | 上车启动 sensors 并保存检查日志 |
| C32/IMU 外参 | 第一版可用 legacy 外参，未完成实测闭环 | 用实测标定结果更新 URDF/TF 和建图配置 |
| STM32/串口协议 | 固件可先按现有协议测，但动态前必须复核速度、转角、deadband 和状态反馈 | 对照固件和 `rc_serial_interface`，完成低速无载/架空验证 |
| Full stack runtime | 静态文档和 launch 对齐已做，车端全链路 runtime 未完成 | 地图到位后按 localization-only -> full dry-run -> low-speed 顺序验证 |

## 产物归档

每次完整验证至少保留：

```text
rc_mapping_data/bags/raw/<bag>/
rc_mapping_data/runs/<run_id>/
rc_mapping_data/autoware_maps/<map_name>/
rc_mapping_data/reports/<run_id>.md
车端 launch 日志
低速验证 bag 或 topic 日志
问题清单
```

归档规则：

- 原始 bag 不覆盖、不移动。
- 每个 `run_id` 记录输入 bag、配置快照、Super-LIO 版本、输出 PCD、地图目录和失败原因。
- 大文件不进 git；脚本、配置、文档和小型报告进 git。

## 文档边界

- 架构和差异边界：`docs/architecture/platform_and_stack_zh.md`。
- 架构到 launch/topic 的前置审计：`docs/architecture/runtime_alignment_audit_zh.md`。
- 建图、Foxglove、Super-LIO、地图打包：`docs/operations/mapping_workflow_zh.md`。
- 现场启动、定位、控制、低速验证：`docs/operations/rc_runbook_zh.md`。
- topic 和 vehicle adapter 事实：`docs/reference/interfaces_and_topics_zh.md`。
- 车辆几何、URDF、TF、外参：`docs/reference/calibration_zh.md`。
