# RC 全链路实施计划

目标：把当前文档基线推进到可重复的车端运行证据，用 RC 验证 Hooke/Autoware 共享 upper stack。

本文件是工程实施计划，不是现场操作手册。现场命令放在 `docs/operations/`，架构事实放在 `docs/architecture/` 和 `docs/reference/`。

## 范围

范围内：

- 保持 Hooke/RC 共享 `localization -> planning -> control -> gate` 为主链路。
- 完成 RC sensing/profile、车辆参数、建图流程、地图回灌、localization 启动和 UART adapter 验证。
- 每个阶段产出可追溯证据：日志、bag metadata、地图产物、topic 检查、launch 状态、低速验证记录。

范围外：

- 在 RC 形成运行证据前替换 shared upper stack。
- 新增 RC 专用 planning、control、localization 或 safety 算法。
- 把一次性诊断命令固化成长期脚本。

## 工作包

| 工作包 | 目标 | 入口 | 完成证据 |
| --- | --- | --- | --- |
| 阶段 1 文档基线 | 保持架构、审计、总控、runbook、建图流程和事实表一致 | `docs/README_zh.md`、`docs/architecture/*`、`docs/operations/*`、`docs/reference/*` | 文档契约测试通过；静态架构图被标注为预览；无旧文档名、主机凭据或当前阶段禁用定位描述 |
| 阶段 2 传感器 runtime | 证明 C32、Hipnuc IMU、TF 在车端可用 | `scripts/rc/rc_start_sensors.sh`、`scripts/check_mapping_inputs.sh` | topic list、topic hz、字段/frame 检查和传感器检查日志 |
| 阶段 3 建图 bag 采集 | 按约定 topic 录制可复用 bag | `scripts/rc/rc_capture_mapping_bag.sh`、start/stop bag 脚本 | `ros2 bag info` 包含必录 topic；原始 bag 归档到 `rc_mapping_data/bags/raw/` |
| 阶段 4 Bag 检查和可视化 | 在运行 LIO 前确认 bag 质量 | `rc_mapping_ws/view_bag_foxglove.sh`、`rc_mapping_ws/inspect_bag_topics.sh` | Foxglove 检查结论和 topic/frequency 报告 |
| 阶段 5 Super-LIO 建图 | 从运动 bag 生成 PCD 并保留过程证据 | `rc_mapping_ws/run_mapping_pipeline.sh` | `runs/<run_id>/` 包含日志、配置快照、Super-LIO 版本、PCD 和报告 |
| 阶段 6 Autoware 地图打包 | 生成 Autoware 可加载地图目录 | `rc_mapping_ws/package_autoware_map.sh` | PCD、metadata、Lanelet2 OSM、projector info 齐全；localization-only 地图有明确标记 |
| 阶段 7 地图回灌和 localization-only | 车端加载地图并验证 NDT 启动 | `rc_mapping_ws/sync_map_to_vehicle.sh`、`scripts/rc/rc_start_localization.sh` | `/initialpose` 后有 NDT pose、`/localization/kinematic_state` 和 `map -> base_link` |
| 阶段 8 Full stack dry-run | 不使能底盘输出时验证 planning/control/gate | `scripts/rc/rc_start_autoware.sh` | goal 生成 trajectory；raw/gated command 存在；drive output 保持禁用 |
| 阶段 9 UART adapter 和底盘反馈 | 验证 Autoware command/status 契约与 STM32 行为一致 | `rc_serial_interface`、`scripts/request_autonomous_mode.sh` | 速度符号、转角方向、gear、control mode、deadband、stop 行为完成核对 |
| 阶段 10 低速动态验证 | 跑短路线并收集可归因问题 | `docs/operations/rc_runbook_zh.md` | 动态 bag/log；问题归属到 sensing、localization、planning、control、gate、adapter、map 或 calibration |

## 执行顺序

1. 固化文档基线，清理过时计划和未标注图像。
2. 车端跑阶段 2；C32/IMU/TF 不通过时不进入正式录包。
3. 先录短静止 bag 验证采集链路，再录运动 bag 做有效建图。
4. 每个 bag 先检查再跑 Super-LIO，不完整 bag 直接退回采集阶段。
5. 在 x86 工作机生成和打包地图资产。
6. 地图回灌车端后先跑 localization-only，再跑完整 Autoware。
7. 完整链路先保持 drive disabled，再进入底盘输出。
8. 底盘输出前必须完成 adapter 状态和 stop 行为验证。

## 质量门槛

- 没有保存产物或命令输出的阶段不能声明完成。
- 任何长期脚本入口必须出现在 `docs/operations/rc_full_chain_execution_zh.md`。
- 临时诊断命令保存在日志或报告中，不新增长期脚本。
- 失败如果改变架构假设，先更新 `runtime_alignment_audit_zh.md`，再改操作文档。
- 失败如果只影响执行顺序，更新 `rc_full_chain_execution_zh.md` 或对应 operation 文档。

## 停止条件

- 静态阶段完成：文档和脚本描述同一条链路，文档检查通过。
- 车端静态阶段完成：传感器、TF、bag、地图加载、NDT、planning/control/gate dry-run 在不驱动底盘时通过。
- 动态阶段完成：低速路线可重复运行，并形成分类问题清单。

## 延后决策

- 官方 Autoware Core/Universe 替换等 RC 验证当前 shared upper stack 后再讨论。
- 后续定位融合是独立定位设计任务，不属于本计划。
- 固件修改只在现有协议无法满足 Autoware-facing adapter 契约时再做。
