# RC/Hooke 文档入口

当前优先级：保持 Hooke/RC 共享 official Autoware 启动结构，用 vehicle profile、sensor-kit profile 和官方 topic/message/frame 合约固定边界。自研模块只能作为显式候选接入，不作为隐藏默认链路。

## 文档结构

`architecture/`

- `architecture/platform_and_stack_zh.md`：Hooke/RC 两套 Autoware 数据流架构图、共享 upper stack、平台差异边界。
- `architecture/image.png`：Hooke/RC 架构图静态预览，用于汇报和快速浏览；事实以 Mermaid 源码和审计表为准。
- `architecture/runtime_alignment_audit_zh.md`：全链路执行前置审计；把架构图模块映射到 launch、node、topic、参数，并记录 Hooke/RC 差异和当前实现缺口。
- `architecture/official_launch_structure_zh.md`：旧自定义 launch 链路与官方 `autoware_launch` + vehicle/sensor profile 结构的差异说明和 Mermaid 图。
- `architecture/official_migration_zh.md`：本地候选 planning/control/gate 与官方 Autoware Core/Universe 组件的差异和后续替换约束。

`operations/`

- `operations/rc_full_chain_execution_zh.md`：RC 验证 Hooke/Autoware 的全链路执行总控，串联前置审计、建图、地图回灌、定位、规划、控制和底盘验证。
- `operations/rc_runbook_zh.md`：RC 上车运行流程，包括传感器、地图、localization、planning、control、gate、adapter 验收顺序。
- `operations/mapping_workflow_zh.md`：车端录 bag、Foxglove 查看 bag、工作机 Super-LIO 建图、Autoware 地图打包、地图回灌。

`reference/`

- `reference/interfaces_and_topics_zh.md`：topic、消息类型、frame、Hooke CAN adapter、RC UART adapter、底盘反馈语义。
- `reference/calibration_zh.md`：RC 车辆尺寸、轮径、轴距、最大转角、传感器外参和低速标定项。

## 阅读入口

- 架构或分支维护问题先看 `architecture/platform_and_stack_zh.md`。
- 执行全链路前先过 `architecture/runtime_alignment_audit_zh.md`，确认架构、launch、topic 和平台差异没有漂移。
- 想理解旧启动结构和官方启动结构的差别，看 `architecture/official_launch_structure_zh.md`。
- 全链路进度、缺口和验收标准看 `operations/rc_full_chain_execution_zh.md`。
- 现场跑车先看 `operations/rc_runbook_zh.md`。
- 建图和地图回灌先看 `operations/mapping_workflow_zh.md`。
- topic、frame、底盘状态含义先看 `reference/interfaces_and_topics_zh.md`。
- 车辆几何和标定先看 `reference/calibration_zh.md`。

## 维护规则

- 正式入口只写全流程会反复使用的内容，不为零散小测试新建长期文档。
- 现场 IP、账号、密码、本机串口名不进入文档；用环境变量或命令参数描述。
- 如果文档内容和当前目标冲突，以 `architecture/platform_and_stack_zh.md` 的平台边界为准。
- official Core/Universe 和自研候选算法的替换讨论只能作为后续共同迁移参考，不能覆盖当前 official baseline。
- 过时计划和一次性执行记录不保留在正式 `docs/` 结构里；需要看历史时用 git log。
