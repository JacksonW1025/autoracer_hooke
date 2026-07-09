# 平台开发契约

用途：定义 Ackermann 平台 profile、sensor kit、底盘 adapter、自研算法模块和文档的维护规则。非用途：不写现场操作步骤，不记录调试过程，不解释项目历史。

当前开发分支：

```text
feature/official-autoware-launch
```

## 平台状态

| Platform | Status | Vehicle model | Sensor model | Source boundary |
| --- | --- | --- | --- | --- |
| RC | active platform profile | `autoracer_rc` | `autoracer_rc_sensor_kit` | `src/autoracer_rc_*` |
| Hooke | integration pending | `autoracer_hooke` | `autoracer_hooke_sensor_kit` | `src/autoracer_hooke_*` |

RC and Hooke are first-class platform target entries. The difference is current
integration status, not project priority. Platform-specific behavior must stay
in profiles, sensor kits, calibration files, adapters, and operation runbooks.
Shared autonomy behavior must stay behind official Autoware contracts.

Platform status is commit-scoped. During development, one commit may make only
RC runnable, only Hooke runnable, both runnable, or neither runnable while a
shared contract is being changed. That is acceptable when the status is explicit
and the non-runnable profile is guarded from accidental launch.

Current branch status:

- RC: active runtime profile and primary development path.
- Hooke: integration pending; on-car validation is not available in the current development environment.

## Package Responsibilities

| Category | Path | Status | Responsibility |
| --- | --- | --- | --- |
| active platform profile | `src/autoracer_rc_description` | buildable | RC vehicle geometry, `vehicle_info`, vehicle URDF/xacro. |
| active platform profile | `src/autoracer_rc_launch` | buildable | RC vehicle interface launch, command gate wiring, RViz profile. |
| active platform profile | `src/autoracer_rc_sensor_kit_description` | buildable | RC LiDAR/IMU extrinsics and sensor-kit URDF/xacro. |
| active platform profile | `src/autoracer_rc_sensor_kit_launch` | buildable | C32 LiDAR, Hipnuc IMU, Madgwick filter, pointcloud filtering. |
| integration pending | `src/autoracer_hooke_description` | guarded by `COLCON_IGNORE` | Hooke vehicle geometry, `vehicle_info`, vehicle URDF/xacro. |
| integration pending | `src/autoracer_hooke_launch` | guarded by `COLCON_IGNORE` | Hooke vehicle interface launch and command gate wiring. |
| integration pending | `src/autoracer_hooke_sensor_kit_description` | guarded by `COLCON_IGNORE` | Hooke sensor extrinsics and sensor-kit URDF/xacro. |
| integration pending | `src/autoracer_hooke_sensor_kit_launch` | guarded by `COLCON_IGNORE` | Hooke sensing launch and driver configuration. |
| chassis adapter | `src/autoracer_vehicle_interface` | buildable | RC UART adapter, Hooke adapter integration surface, `/vehicle/status/*`. |
| sensor adapter | `src/autoracer_sensing` | buildable | Small sensor conversion/filter nodes that preserve official topics. |
| localization adapter | `src/autoracer_localization` | buildable | Seed-pose and localization helper nodes that preserve official topics. |
| safety boundary | `src/autoracer_safety` | buildable | Command gate, drive-enable policy, timeout and stop behavior. |
| local algorithm package | `src/autoracer_planning` | buildable | Planning candidates that must be explicitly wired into official contracts. |
| local algorithm package | `src/autoracer_control` | buildable | Control candidates that must be explicitly wired into official contracts. |
| shared description assets | `src/autoracer_description` | buildable | Shared frame/URDF helpers and static TF reference assets. |
| vendor driver | `src/hipnuc_imu` | buildable | Hipnuc IMU driver material. |
| reference material | `src/hooke2_vehicle` | buildable reference packages | Hooke vehicle and CAN reference material. |
| reference material | `src/hardware_drivers` | buildable | SocketCAN driver material. |
| reference material | `src/wd_msgs` | buildable | Hooke chassis messages and byte helpers. |
| upstream/pinned dependency | `src/external/autoware` | pinned dependency | Selected upstream Autoware packages. Patch only with source, reason, and verification. |

## 新增或修改平台 Profile

Each platform profile must expose the official Autoware naming contract:

```text
<vehicle_model>_description
<vehicle_model>_launch
<sensor_model>_description
<sensor_model>_launch
```

For RC:

```text
vehicle_model:=autoracer_rc
sensor_model:=autoracer_rc_sensor_kit
```

For Hooke:

```text
vehicle_model:=autoracer_hooke
sensor_model:=autoracer_hooke_sensor_kit
```

Profile content rules:

- Vehicle dimensions, wheelbase, steering limits, and vehicle URDF belong in the vehicle description package.
- Sensor poses relative to `base_link` belong in the sensor-kit description package.
- Sensor driver parameters, filter parameters, and sensing launch files belong in the sensor-kit launch package.
- Chassis transport details belong in the vehicle launch package and adapter package.
- Runtime facts such as map path, serial device, LiDAR interface, and RViz enablement stay in environment variables or operation runbooks.

Minimum checks after changing a profile:

```bash
python3 -m pytest test -q
colcon list --names-only | grep -E '^(autoracer_rc|autoracer_hooke)' || true
```

Expected current result: RC profile packages are discoverable; Hooke packages
remain hidden while they are guarded by `COLCON_IGNORE`.

## 底盘 Adapter

Adapter input/output boundaries:

```text
/control/command/control_cmd
  -> autoracer_safety/command_gate
  -> /autoracer/control/safe_control_cmd
  -> platform adapter
  -> chassis transport
```

Adapter requirements:

- Consume gated control commands, not raw upstream control as a direct chassis command.
- Publish official `/vehicle/status/*` topics with Autoware units and frames.
- Keep CAN, UART, byte layout, checksum, and device-specific details inside the adapter package.
- Preserve stop behavior when `ENABLE_DRIVE_COMMANDS=false`, command timeout, localization loss, or route completion occurs.
- Keep transport configuration as runtime settings unless it is a platform hardware constant.

## 自研算法模块

`src/autoracer_planning` and `src/autoracer_control` are local algorithm package
areas. They may replace or augment upstream modules only through explicit launch
wiring and official interface contracts.

Rules:

- Define input/output topics before implementation.
- Use official Autoware message types, frames, units, and diagnostics wherever possible.
- Keep adapters thin when an interface conversion is unavoidable.
- Do not patch `src/external/autoware` to hide a contract mismatch.
- Do not make a local algorithm the default by adding hidden script logic.

## Upstream Dependency Changes

`src/external/autoware` is an upstream/pinned dependency area. Treat changes
there as dependency patches, not ordinary project code.

Required record for a patch:

```text
Upstream package:
Upstream source/revision:
Reason:
Rejected alternatives:
Verification:
Reversal path:
```

Prefer upstream configuration, profile parameters, adapter nodes, or local
packages before modifying pinned upstream code.

## 文档维护规则

| Content | Document |
| --- | --- |
| Project purpose, target platforms, repository layout | `README.md` |
| Platform profile and package development rules | `docs/development_guide_zh.md` |
| Runtime system architecture, data flow, platform boundaries | `docs/architecture_zh.md` |
| On-car procedures and commands | `docs/operations/rc_runbook_zh.md` |
| Mapping, bag capture, map packaging | `docs/operations/mapping_workflow_zh.md` |
| Topic, frame, adapter, and calibration facts | `docs/reference/interfaces_and_calibration_zh.md` |

Documentation rules:

- Describe project facts and contracts directly.
- Do not keep historical transition notes in formal docs.
- Do not explain how a reader should think about the project.
- Keep operation commands in `docs/operations/`.
- Keep interface tables and calibration facts in `docs/reference/`.
- Keep node/topic/dataflow diagrams in `docs/architecture/`.

Documentation checks:

```bash
python3 -m pytest test/test_docs_architecture_contract.py -q
python3 -m pytest test -q
```
