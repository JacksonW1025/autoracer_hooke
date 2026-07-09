# 多车型 Profile 矩阵

用途：作为同一仓库维护 RC 和 Hooke official Autoware profiles 的 authoritative profile matrix。非用途：不记录现场临时命令，不把占位目录描述成可运行实现。

## 当前结论

RC 是当前 active runtime baseline。Hooke 只保留受控占位，状态是 `disabled_placeholder`，not runtime ready。

| 平台 | `vehicle_model` | `sensor_model` | 状态 | 源码位置 | 脚本入口 | 当前责任 |
| --- | --- | --- | --- | --- | --- | --- |
| RC Ackermann | `autoracer_rc` | `autoracer_rc_sensor_kit` | active runtime baseline | `src/autoracer_rc_*` | `scripts/rc/` | 当前唯一可运行开发基线，继续用于 Orin 上的 RC 实车验证。 |
| Hooke | `autoracer_hooke` | `autoracer_hooke_sensor_kit` | `disabled_placeholder` / not runtime ready | `src/autoracer_hooke_*` | `scripts/hooke/` | 交给 Hooke 负责人补真实 vehicle、sensor、CAN adapter、sensing launch。 |

`scripts/common/` 是共享脚本边界。它只能放不依赖具体车辆硬件事实的 helper 说明或逻辑；RC 串口、C32 雷达、Hooke CAN、Hesai/Fixposition 这类事实不能放进 common 层。

## RC 基线

RC profile 已经进入官方 Autoware 命名结构：

```text
src/autoracer_rc_description
src/autoracer_rc_launch
src/autoracer_rc_sensor_kit_description
src/autoracer_rc_sensor_kit_launch
```

启动时使用：

```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=autoracer_rc \
  sensor_model:=autoracer_rc_sensor_kit
```

RC 的运行入口只看 `scripts/rc/`。这些脚本可以调用根目录已有 helper，例如 `scripts/run_official_autoware.sh` 和 `scripts/ros_env.sh`，但不应该重新引入旧的自定义 bringup 总控。当前 `scripts/run_official_autoware.sh` 只允许 `autoracer_rc` / `autoracer_rc_sensor_kit` 这一组 active profile，防止环境变量把 RC 入口切到 Hooke 占位或其他未验证 profile。

## Hooke 占位

Hooke 当前只创建下面四个目录：

```text
src/autoracer_hooke_description
src/autoracer_hooke_launch
src/autoracer_hooke_sensor_kit_description
src/autoracer_hooke_sensor_kit_launch
```

每个目录都必须保留：

```text
COLCON_IGNORE
README.md
profile_requirements.yaml
```

不要把 Hooke 空壳包当成可运行 profile。占位目录没有 `package.xml`，目的就是避免 `colcon`、`autoware_launch` 或操作者误认为 Hooke 已经可运行。

只有在真实 Hooke 配置完成后，才可以移除 `COLCON_IGNORE` 并补正式 ROS package 文件。启用前至少需要：

| 目录 | 必须补齐 |
| --- | --- |
| `autoracer_hooke_description` | Hooke 车身尺寸、轴距、转角限制、`vehicle_info.param.yaml`、`vehicle.xacro`。 |
| `autoracer_hooke_launch` | Hooke CAN adapter、车辆接口 launch、安全 gate 接线、车辆状态 topic。 |
| `autoracer_hooke_sensor_kit_description` | Hesai/Fixposition/IMU 相对 `base_link` 的真实外参和 sensor kit URDF。 |
| `autoracer_hooke_sensor_kit_launch` | Hesai driver、Fixposition/GNSS/INS/IMU launch、点云和 IMU official topic contract。 |

Hooke 脚本入口 `scripts/hooke/hooke_start_autoware.sh` 必须在占位阶段 fail-fast，不能偷偷指向 RC profile 或旧链路。

## 算法边界

当前默认上层链路仍是 official Autoware planning/control。以下本地包只作为自研算法候选存在：

```text
src/autoracer_planning
src/autoracer_control
```

这些包不能作为隐藏默认链路，也不能绕过 official profile 结构。以后如果需要启用自研算法候选，要求是：

- 输入输出 topic、message、frame、单位遵守 official Autoware contract。
- 在 launch/profile 层显式替换，不通过脚本暗中切换。
- 仍保留 `autoracer_safety` gate 和 vehicle adapter 的清晰边界。
- 不在 `src/external/autoware` 里做隐形修改。

## 维护规则

- 分支用于开发隔离，不用于表达车型事实；车型事实放在 profile、adapter、sensor kit 和脚本入口。
- RC profile 是当前可运行基线；Hooke profile 未完成前不能参与构建、启动或 release 说明。
- 新增 shared helper 前先确认它不包含具体车辆硬件事实。
- 删除或重命名 profile 目录前必须同时更新本文件、`official_launch_structure_zh.md`、测试和 operator README。
