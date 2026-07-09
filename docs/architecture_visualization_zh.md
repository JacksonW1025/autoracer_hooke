# 架构可视化说明

用途：说明本仓库怎样用 Mermaid/ROS graph 这类可视化文档帮助理解系统结构。非用途：不定义运行时 HMI，不替代 RViz、Foxglove 或 Autoware launch。

## dear_ros_node_viewer 是什么

`dear_ros_node_viewer` 不是上车操作界面。它更接近一套离线架构浏览文档：把 Autoware 的 node、topic、ADAPI、planning、control、sensing 等关系导出成 HTML/Mermaid，方便人在浏览器里看系统拓扑。

它的价值是：

- 给不熟悉 Autoware 的人快速建立系统地图。
- 看清 sensing、localization、planning、control、vehicle adapter 的边界。
- 在重构后说明哪些是官方链路，哪些是本仓库 profile/adapter。
- 辅助 code review 和交接，不参与实车运行。

它不应该做的事：

- 不作为车辆 HMI。
- 不写入启动链路。
- 不保存现场 IP、串口、密码或一次性排查日志。
- 不替代 `autoware_launch`、RViz 工具栏、route/goal/initialpose 交互。

## 当前可视化文档入口

当前仓库的人工维护架构图在：

```text
docs/architecture_zh.md
```

更偏“图形入口”的静态 Mermaid 图在：

```text
docs/architecture/generated/rc_official_runtime_graph.mmd
```

查看方式：

- GitHub/GitLab Markdown 预览 Mermaid。
- VS Code 安装 Mermaid 预览插件。
- 用 Mermaid CLI 转成 SVG/PNG 后放进评审材料。

## 运行界面边界

Autoware 的默认交互入口仍然是 RViz2。我们使用官方 `autoware_launch` 加载官方 Autoware/Tier IV RViz 插件：

```text
autoware_launch/rviz/autoware.rviz
```

这和 `dear_ros_node_viewer` 是两类东西：

| 类型 | 用途 | 是否参与实车运行 |
| --- | --- | --- |
| RViz2 + Autoware RViz 插件 | 初始位姿、目标点、路线、状态、点云、轨迹交互 | 是 |
| Foxglove | 实时/离线 topic、bag、点云检查 | 可选 |
| Mermaid/HTML 架构文档 | 讲清系统结构和开发边界 | 否 |

## 维护规则

- Mermaid 图只表达稳定结构，不记录一次性现场问题。
- 运行链路以 `scripts/rc/`、`autoware_launch`、vehicle/sensor profile 为准。
- 如果实际 topic、frame、module 边界变化，先更新源码和 launch，再同步 Mermaid 图。
- 自动生成工具可以后续补，但生成结果必须进入 `docs/architecture/generated/`，不要散落在仓库根目录。
