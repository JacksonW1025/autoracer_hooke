# RC 文档结构设计规格

目标：把 `docs/` 组织成一套围绕“RC 跑通 Hooke/Autoware 共享链路”的工程文档，而不是一组按历史问题堆叠的记录。

## 设计原则

- 一个目标：RC 验证 Hooke/Autoware shared upper stack。
- 一个总控：`operations/rc_full_chain_execution_zh.md` 追踪全链路进度、缺口和验收。
- 一个前置审计：`architecture/runtime_alignment_audit_zh.md` 记录架构图、launch、topic 和平台差异。
- 静态图只作预览，Mermaid 源码和审计表是可维护依据。
- 现场命令放 `operations/`，事实表放 `reference/`，设计和实施计划放 `superpowers/`。
- 不为一次性排查新增长期文档或长期脚本。

## 当前目录

```text
docs/
  README_zh.md
  architecture/
    image.png
    platform_and_stack_zh.md
    runtime_alignment_audit_zh.md
    official_migration_zh.md
  operations/
    rc_full_chain_execution_zh.md
    rc_runbook_zh.md
    mapping_workflow_zh.md
  reference/
    interfaces_and_topics_zh.md
    calibration_zh.md
  superpowers/
    specs/2026-07-01-docs-structure-design.md
    plans/2026-07-01-rc-full-chain-implementation.md
```

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `docs/README_zh.md` | 文档入口和维护规则。 |
| `architecture/platform_and_stack_zh.md` | Hooke/RC 架构图、共享 upper stack、平台差异边界。 |
| `architecture/image.png` | 架构图静态预览，用于汇报；不作为事实源。 |
| `architecture/runtime_alignment_audit_zh.md` | 架构模块到 launch、node、topic、参数的前置审计。 |
| `architecture/official_migration_zh.md` | 后续官方 Autoware 迁移评估，不作为当前 RC 主链路。 |
| `operations/rc_full_chain_execution_zh.md` | 全链路执行总控，记录阶段、入口、产物、通过标准和缺口。 |
| `operations/rc_runbook_zh.md` | 上车运行步骤。 |
| `operations/mapping_workflow_zh.md` | bag、Foxglove、Super-LIO、地图打包和回灌流程。 |
| `reference/interfaces_and_topics_zh.md` | topic、消息类型、frame、vehicle adapter 和底盘反馈事实。 |
| `reference/calibration_zh.md` | 车辆几何、URDF/TF、外参、速度和转角标定事实。 |
| `superpowers/plans/2026-07-01-rc-full-chain-implementation.md` | 后续工程实施计划和验收证据要求。 |

## 维护规则

- 同一 topic、参数或平台边界只保留一个权威位置，其他文档引用它。
- 修改 launch、topic 或架构图时，同步更新 `platform_and_stack_zh.md` 和 `runtime_alignment_audit_zh.md`。
- 新增长期脚本前，先确认它服务 `rc_full_chain_execution_zh.md` 中的正式阶段。
- 过时计划删除，不保留会误导后续执行的历史大段落。
- 大文件和运行产物不进文档目录；`image.png` 是唯一允许保留的架构静态预览图。

## 验收标准

- `docs/README_zh.md` 能索引所有正式文档。
- 架构、操作、参考和计划四类文档职责互不重叠。
- 当前阶段不描述未来定位融合设计。
- 文档不包含现场账号、密码、临时主机细节或旧文档入口。
- 文档契约测试和 `git diff --check -- README.md docs test` 通过。
