# 开发入口与包责任表

用途：给第一次接手仓库的人一个可执行入口，说明当前能跑什么、`src/` 里每类包是什么、继续开发 RC/Hooke/算法时应该从哪里开始。非用途：不替代现场运行手册，不记录一次性测试日志。

## 当前基线

当前唯一可运行基线是 `feature/official-autoware-launch` 上的 RC official Autoware 链路：

```text
vehicle_model:=autoracer_rc
sensor_model:=autoracer_rc_sensor_kit
```

运行入口是 `scripts/rc/`。根 wrapper `scripts/run_official_autoware.sh` 只允许这组 active official RC profile，防止环境变量误切到未完成 profile。

Hooke official profile 当前是 disabled Hooke official profile：`src/autoracer_hooke_*` 只保留占位、`COLCON_IGNORE` 和交接 checklist。Hooke 相关旧资料和 vendored 包仍在仓库里，但它们是 vendored Hooke reference，不是当前可启动的 official profile。

## `src/` 包责任表

| 类别 | 路径 | 当前状态 | 什么时候改 |
| --- | --- | --- | --- |
| active official RC profile | `src/autoracer_rc_description` | 可构建、可被 `autoware_launch` 按 `autoracer_rc` 发现 | RC 车身尺寸、vehicle URDF、vehicle info 变化。 |
| active official RC profile | `src/autoracer_rc_launch` | 可构建、RC vehicle interface 入口 | RC 串口 adapter、安全 gate、RViz profile、vehicle interface launch 变化。 |
| active official RC profile | `src/autoracer_rc_sensor_kit_description` | 可构建、可被 `autoware_launch` 按 `autoracer_rc_sensor_kit` 发现 | RC LiDAR/IMU 外参、sensor kit URDF 变化。 |
| active official RC profile | `src/autoracer_rc_sensor_kit_launch` | 可构建、RC sensing 入口 | C32、Hipnuc IMU、Madgwick、点云降采样、topic/frame 参数变化。 |
| disabled Hooke official profile | `src/autoracer_hooke_description` | 不构建；无 `package.xml` | Hooke 车身尺寸、vehicle info、official vehicle xacro 准备齐全后启用。 |
| disabled Hooke official profile | `src/autoracer_hooke_launch` | 不构建；无 `package.xml` | Hooke CAN adapter、gate 接线、vehicle interface launch 准备齐全后启用。 |
| disabled Hooke official profile | `src/autoracer_hooke_sensor_kit_description` | 不构建；无 `package.xml` | Hooke Hesai/Fixposition/IMU 外参和 sensor kit xacro 准备齐全后启用。 |
| disabled Hooke official profile | `src/autoracer_hooke_sensor_kit_launch` | 不构建；无 `package.xml` | Hooke Hesai/Fixposition sensing launch 和 topic contract 准备齐全后启用。 |
| shared adapter | `src/autoracer_vehicle_interface` | 可构建 | RC UART adapter、STM32 协议、`/vehicle/status/*` 语义变化。 |
| shared sensing adapter | `src/autoracer_sensing` | 可构建 | 点云降采样、Fixposition speed bridge、小型 sensor adapter 变化。 |
| shared localization adapter | `src/autoracer_localization` | 可构建 | manual seed、NDT startup helper、pose/TF bridge 等 topic contract 适配变化。 |
| shared safety boundary | `src/autoracer_safety` | 可构建 | command gate、drive enable、限幅、超时保护变化。 |
| local algorithm candidates | `src/autoracer_planning` | 可构建但不是默认主链路 | 需要显式评估或替换官方 planning 时改；不能暗中接回默认启动。 |
| local algorithm candidates | `src/autoracer_control` | 可构建但不是默认主链路 | 需要显式评估或替换官方 control 时改；不能暗中接回默认启动。 |
| shared frame/reference assets | `src/autoracer_description` | 可构建 | 跨平台辅助 URDF、静态 TF 查看、历史 Hooke/RC frame 参考。 |
| RC sensor vendor package | `src/hipnuc_imu` | 可构建 | Hipnuc IMU driver 本身需要修复时改。 |
| vendored Hooke reference | `src/hooke2_vehicle` | 可构建 reference packages | 迁移 Hooke official profile 时参考；不要让 RC wrapper 指向这里。 |
| vendored Hooke reference | `src/hardware_drivers` | 可构建 CAN driver | Hooke CAN driver 迁移或底层驱动修复时改。 |
| vendored Hooke reference | `src/wd_msgs` | 可构建 Hooke messages | Hooke CAN message definitions 变化时改。 |
| pinned upstream | `src/external/autoware` | 可构建依赖集合 | 只在明确升级/patch Autoware 时改；必须记录来源、原因和验证。 |

## 继续开发 RC

1. 先确认分支和远端同步：

```bash
git status --short --branch
git rev-parse --short HEAD
```

2. 改 RC profile 时优先找对应 official 位置：

```text
车辆参数       -> src/autoracer_rc_description/config/vehicle_info.param.yaml
LiDAR/IMU 外参 -> src/autoracer_rc_sensor_kit_description/config/sensor_kit_calibration.yaml
C32 参数       -> src/autoracer_rc_sensor_kit_launch/config/lslidar_cx.yaml
RC sensing     -> src/autoracer_rc_sensor_kit_launch/launch/sensing.launch.xml
RC vehicle     -> src/autoracer_rc_launch/launch/vehicle_interface.launch.xml
UART adapter   -> src/autoracer_vehicle_interface
```

3. 改完至少跑：

```bash
python3 -m pytest test -q
colcon list --names-only | grep -E '^(autoracer_rc|autoracer_hooke)' || true
```

期望：`autoracer_rc_*` 四个 active profile 可见；`autoracer_hooke_*` 不可见。

## 迁移 Hooke

Hooke 负责人不要从空目录硬写。先从这些 reference material 取事实，再转写到 official profile：

```text
Hooke 车身参数       -> src/hooke2_vehicle/vehicle_launcher/hooke2_description/config/vehicle_info.param.yaml
Hooke vehicle xacro  -> src/hooke2_vehicle/vehicle_launcher/hooke2_description/urdf/vehicle.xacro
Hooke CAN launch     -> src/hooke2_vehicle/vehicle_launcher/hooke2_launch/launch/vehicle_interface.launch.xml
Hooke CAN 参数       -> src/hooke2_vehicle/interfaces/hooke2_interface/config/hooke2.param.yaml
Hooke sensor 外参    -> src/autoracer_description/config/hooke2_sensor_extrinsics.yaml
```

启用顺序：

1. 在 `src/autoracer_hooke_description` 补真实 `package.xml`、`CMakeLists.txt`、`config/`、`urdf/`。
2. 在 `src/autoracer_hooke_sensor_kit_description` 补 Hesai/Fixposition/IMU 的 official sensor-kit description。
3. 在 `src/autoracer_hooke_sensor_kit_launch` 补 Hesai/Fixposition sensing launch。
4. 在 `src/autoracer_hooke_launch` 接入 Hooke CAN adapter 和安全 gate。
5. 移除对应 `COLCON_IGNORE` 前，先补测试证明 Hooke package 能被发现且 RC wrapper 仍不会误启动 Hooke。

## 替换或新增自研算法

`src/autoracer_planning` 和 `src/autoracer_control` 是 local algorithm candidates。它们可以继续存在，但启用规则是：

- 遵守 official topic、message、frame、parameter、diagnostics contract。
- 在 launch/profile 层显式替换，不通过脚本或环境变量暗中切换。
- 不修改 `src/external/autoware` 来绕过接口问题。
- 保留 `autoracer_safety` 在 vehicle adapter 前的安全边界。

算法开发前先写清楚输入/输出表，再加测试覆盖 message contract。

## 修改文档

文档只保留长期入口和事实，不保留一次性日志。

| 要改的问题 | 应改文档 |
| --- | --- |
| 新人如何接手、包责任、开发路线 | 本文件 |
| RC/Hooke profile 状态 | `docs/architecture/profile_matrix_zh.md` |
| 架构边界和上层链路 | `docs/architecture/platform_and_stack_zh.md` |
| launch/topic 是否对齐 | `docs/architecture/runtime_alignment_audit_zh.md` |
| 上车操作 | `docs/operations/rc_runbook_zh.md` |
| 建图和地图回灌 | `docs/operations/mapping_workflow_zh.md` |
| topic/frame/adapter 事实 | `docs/reference/interfaces_and_topics_zh.md` |
| 车辆参数和标定 | `docs/reference/calibration_zh.md` |

每次改 docs 后跑：

```bash
python3 -m pytest test/test_docs_architecture_contract.py -q
```
