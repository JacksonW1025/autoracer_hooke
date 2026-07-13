# RC Platform Integration Design

**Status:** Approved architecture, implementation pending

**Baseline:** `pilot-localization-sync-20260707` at `f214db3`

**Repository:** `/home/milesli/Desktop/RC/autoracer_hooke`

**Primary goal:** Add the RC race car as a first-class platform without creating a second autonomy stack or importing the legacy repository's composition model.

## 1. Decision

The repository will use one shared race core and one thin composition root per platform:

```text
src/core
  -> shared localization, planning, control, safety, and race composition

src/platform/hooke2
  -> Hooke2 sensors, CAN/chassis adapter, calibration, and thin composition

src/platform/rc
  -> RC sensors, UART/chassis adapter, calibration, and thin composition
```

“Minimal platform difference” means minimizing duplicated responsibility and the spread of hardware knowledge. It does not mean minimizing file count. A small platform-specific launch file is desirable when it gives one responsibility a clear owner and permits isolated diagnosis.

The legacy tree at `../legacy-refference/autoracer_hooke-autoware-legacy` is reference material only. It is not a build dependency, launch dependency, source workspace, or architecture template. Useful hardware facts may be re-derived from it and implemented under the new contracts after tests establish their behavior.

## 2. Design principles

### 2.1 One behavior owner

Each shared behavior has exactly one production owner:

| Behavior | Owner |
| --- | --- |
| NDT/EKF localization composition | `src/core/autoracer_localization` |
| Fixed-course planning and local trajectory generation | `src/core/autoracer_planning` |
| MPC lateral and PID longitudinal control | `src/core/autoracer_control` |
| Race state, readiness, engage, stop, and fault handling | `src/core/autoracer_safety` |
| Final command arbitration and limiting | Autoware VehicleCmdGate launched by `autoracer_safety` |
| Full shared race composition | `src/core/autoracer_bringup` |
| Hardware protocol translation | The corresponding package under `src/platform/hooke2` or `src/platform/rc` |

No RC-specific copy of localization, planning, control, safety, runtime management, or VehicleCmdGate is permitted.

### 2.2 Stable middle contracts

Core consumes and produces only standard ROS/Autoware contracts:

```text
Inputs to core
  /sensing/lidar/concatenated/pointcloud
  /sensing/imu/imu_data
  /sensing/vehicle_velocity_converter/twist_with_covariance
  /vehicle/status/velocity_status
  /vehicle/status/steering_status
  /vehicle/status/gear_status
  /vehicle/status/control_mode
  /initialpose3d or /sensing/gnss/pose_with_covariance

Outputs from core
  /control/command/control_cmd
  /control/command/gear_cmd
  /control/command/turn_indicators_cmd
  /control/command/hazard_lights_cmd
  /control/command/emergency_cmd
  /control/control_mode_request service calls
```

Core must not contain the strings or types `hooke2`, `rc_serial`, `STM32`, `CAN`, `lslidar`, `C32`, `Hipnuc`, `Pandar`, or `Fixposition`, except in migration tests that assert their absence.

### 2.3 Platform facts are injected

The following are platform facts and must be supplied by platform packages:

- sensor drivers and native device settings;
- raw sensor format conversion;
- sensor frames and calibrated extrinsics;
- vehicle geometry;
- chassis protocol and device connection;
- steering/velocity sign and scale;
- controller tuning deltas;
- command limits and status timeouts;
- whether localization receives a GNSS seed or an operator-provided initial pose.

Shared launch files define argument names and defaults. Platform launch files provide values. Platform files must not reproduce shared node graphs.

### 2.4 Simple, real, verifiable

Adapters translate only what the race stack consumes. They do not emulate unused private protocols or auxiliary vehicle functions. Every adapter must have unit tests for encoding, decoding, units, signs, clamping, malformed input, and timeout behavior. Hardware validation is separate from unit validation and cannot be claimed without real device evidence.

### 2.5 Reproducible dependencies

Third-party sources remain outside the product source tree in the ignored `vendor_ws` underlay. Each source has an upstream URL, immutable revision, license, and explicit package selection in `dependencies/`.

The legacy repository is never listed in `dependencies/versions.lock.yaml`.

## 3. Target repository structure

```text
src/
├── core/
│   ├── autoracer_localization/
│   ├── autoracer_planning/
│   ├── autoracer_control/
│   ├── autoracer_safety/
│   └── autoracer_bringup/
│       └── launch/race.launch.py
└── platform/
    ├── hooke2/
    │   ├── autoracer_hooke2_bringup/
    │   │   ├── launch/sensing.launch.py
    │   │   ├── launch/vehicle.launch.py
    │   │   ├── launch/race.launch.py
    │   │   └── config/hooke2/
    │   ├── autoracer_hooke2_adapter/
    │   ├── hooke2_description/
    │   ├── hooke2_interface/
    │   ├── hooke2_msgs/
    │   └── can_driver/
    └── rc/
        ├── autoracer_rc_bringup/
        │   ├── launch/sensing.launch.py
        │   ├── launch/vehicle.launch.py
        │   ├── launch/race.launch.py
        │   └── config/rc/
        ├── autoracer_rc_adapter/
        │   ├── autoracer_rc_adapter/rc_serial_protocol.py
        │   ├── autoracer_rc_adapter/rc_serial_interface.py
        │   ├── autoracer_rc_adapter/c32_pointcloud_adapter.py
        │   └── test/
        └── autoracer_rc_description/
            ├── config/
            ├── launch/static_tf.launch.py
            └── urdf/
```

The existing `src/core/autoracer_description` is removed after its Hooke2 assets are moved into `src/platform/hooke2/hooke2_description`. Platform TF is launched by platform sensing composition before core localization starts.

The existing `src/core/autoracer_sensing` remains temporarily as the platform-neutral localization AD API bridge. Renaming or folding it into `autoracer_localization` is explicitly outside this integration because it does not block RC and would add unrelated packaging churn. Its package description and tests must continue to state that it is platform neutral.

## 4. Composition model

### 4.1 Shared race composition

`autoracer_bringup/race.launch.py` owns only this graph:

```text
autoracer_localization
  -> autoracer_planning
  -> autoracer_control
  -> autoracer_safety
```

It accepts paths or values for:

- localization map;
- course asset;
- simulation clock and run mode;
- vehicle information;
- controller overlay;
- VehicleCmdGate overlay;
- runtime safety overlay;
- planning speed, acceleration, and deceleration limits.

It does not launch sensor drivers, platform TF, chassis drivers, or protocol adapters.

### 4.2 Hooke2 composition

Hooke2 `sensing.launch.py` owns:

- Hooke2 static sensor TF;
- Nebula/Pandar driver;
- Fixposition driver;
- Fixposition velocity bridge;
- Fixposition-to-standard GNSS pose conversion;
- Fixposition raw IMU normalization to `/sensing/imu/imu_data`.

Hooke2 `vehicle.launch.py` owns Hooke2 interface and CAN driver.

Hooke2 `race.launch.py` includes Hooke2 sensing, Hooke2 vehicle, and the shared race launch. It contains no localization, planner, controller, runtime-manager, or gate node definitions.

### 4.3 RC composition

RC `sensing.launch.py` owns:

- RC static sensor TF;
- the pinned LeiShen C32 driver;
- C32 raw-field normalization to the standard point-cloud topic;
- the pinned HiPNUC IMU driver;
- optional Madgwick filtering when the driver output requires it;
- manual initialization mode, with no fake GNSS publisher.

RC `vehicle.launch.py` owns only the RC serial adapter. It does not launch a second command gate or vehicle-velocity converter.

RC `race.launch.py` includes RC sensing, RC vehicle, and the shared race launch. Its default is `use_sim_time=false`, `system_run_mode=online`, and conservative RC limits.

### 4.4 Diagnostic independence

The three platform launch files serve distinct operations:

- `sensing.launch.py`: validate sensors and TF without enabling chassis control;
- `vehicle.launch.py`: validate serial/CAN feedback with the autonomy stack stopped;
- `race.launch.py`: run the complete closed loop.

This separation is intentional and must not be collapsed into one conditional launch file.

## 5. RC contracts

### 5.1 Sensor contract

| Source | Native boundary | Normalized output |
| --- | --- | --- |
| LeiShen C32 | vendor driver `PointCloud2` | `/sensing/lidar/concatenated/pointcloud`, frame `lidar_top` |
| HiPNUC IMU | vendor driver `sensor_msgs/Imu` | `/sensing/imu/imu_data`, frame `imu_link` |
| Operator | RViz/AD API initial pose | `/initialpose3d`, frame `map` |

Point-cloud normalization preserves at least `x`, `y`, `z`, `intensity`, `return_type`, `channel`, and a valid timestamp where the driver provides them. The adapter fails closed on unsupported layouts rather than silently relabeling fields.

No RC GNSS input is assumed. If GNSS hardware is added later, it must first normalize to `/sensing/gnss/pose_with_covariance`; core behavior does not change.

### 5.2 Chassis command contract

The RC adapter consumes:

```text
/control/command/control_cmd  autoware_control_msgs/msg/Control
/control/command/gear_cmd     autoware_vehicle_msgs/msg/GearCommand
/control/control_mode_request autoware_vehicle_msgs/srv/ControlModeCommand
```

It publishes:

```text
/vehicle/status/velocity_status  autoware_vehicle_msgs/msg/VelocityReport
/vehicle/status/steering_status  autoware_vehicle_msgs/msg/SteeringReport
/vehicle/status/gear_status      autoware_vehicle_msgs/msg/GearReport
/vehicle/status/control_mode     autoware_vehicle_msgs/msg/ControlModeReport
```

The adapter owns UART framing, checksum, reconnect behavior, unit conversion, Ackermann steering-to-yaw-rate conversion when required by firmware, and command timeout. Core never sees the private serial frame.

The protocol source of truth is the sibling firmware repository `/home/milesli/Desktop/RC/RCCar-Firmware`, currently `main` at `4113141f1ac5ba1af276db3c2bace81b5bcf1d16`. Its `docs/protocols/serial-protocol.md`, `WHEELTEC_APP/SerialControl_task.c`, and `WHEELTEC_APP/data_task.c` define an 11-byte downlink control frame, a 24-byte uplink telemetry frame, and UART4 at 115200 baud. Unit tests must be derived from those current firmware definitions, not from the legacy ROS repository. A bench capture is still required to prove that the running STM32 image matches the checked-in firmware.

### 5.3 Runtime semantics

The shared runtime manager expects real or faithfully derived velocity, steering, gear, and control-mode status. If the RC firmware does not report gear or front-wheel angle directly, the adapter may derive them only when the derivation is documented and tested. It must publish `NOT_READY` when the serial device is disconnected and must transition to a stop command when input commands are stale.

## 6. Parameter ownership

### 6.1 Single facts

`autoracer_rc_description/config/vehicle_info.param.yaml` is the single source for:

- wheel radius and width;
- wheelbase and tread;
- overhangs and height;
- maximum steering angle.

The starting reference values are wheel radius `0.115 m`, wheelbase `0.600 m`, tread `0.440 m`, and maximum steering angle `0.262 rad`. They are unverified until measured on the active car.

`autoracer_rc_description/config/sensor_extrinsics.yaml` is the single source for `base_link -> lidar_top` and `base_link -> imu_link`. The static-TF launch reads this file; no duplicate numeric transforms are allowed in launch code.

### 6.2 Shared defaults and platform overlays

Core owns parameter schemas and conservative shared defaults. Platform bringup owns only platform-specific values:

| Overlay | RC-owned values |
| --- | --- |
| `controller.param.yaml` | steering dynamics and gains proven different from shared defaults |
| `planning.param.yaml` | maximum speed, acceleration, deceleration, latency, stopping margin |
| `vehicle_cmd_gate.param.yaml` | velocity, steering, acceleration, jerk, lateral limits |
| `race_runtime.param.yaml` | feedback timeout and emergency deceleration |

Launch parameter order is upstream defaults, core defaults, vehicle information, then platform overlay. Later files override earlier files. A platform file must not copy unchanged upstream/default values.

### 6.3 Planning assets

Planning code is shared; map and course assets are run inputs. RC uses a map and fixed course generated in the same `map` frame. The course speed profile must be rebuilt or validated against RC acceleration, deceleration, and lateral-acceleration limits. Reusing Hooke2/CarMaker course coordinates or speed profiles without validation is forbidden.

## 7. Dependency sources

The implementation starts from these independently sourced revisions, subject to build and license verification:

| Dependency | Upstream | Revision |
| --- | --- | --- |
| LeiShen ROS 2 driver, C32 V4 branch | `https://github.com/Lslidar/Lslidar_ROS2_driver.git` | `08d692c2adf62f29b991fe44313b17840e4bea8b` |
| HiPNUC products/ROS2 examples | `https://github.com/hipnuc/products.git` | `5a4380272cd70402e7f8928b05a6af4bfa659807` |

Only required ROS packages are selected into `vendor_ws`. If either revision fails the target ROS distribution build or does not match the physical device protocol, the implementation records the incompatibility and adds the smallest reviewed patch under `dependencies/patches/`; it does not fall back to importing the legacy workspace.

## 8. Safety defaults

RC integration is fail-closed:

- drive commands disabled until a deliberate bench/vehicle procedure enables them;
- initial dynamic speed limit no greater than `0.5 m/s`;
- command timeout no greater than `0.5 s`;
- serial disconnect reports `NOT_READY` and sends no nonzero motion command;
- stale localization, trajectory, raw control, final control, or vehicle status latches the existing shared fault path;
- current Hooke2 values such as `100 m/s` and `0.488 rad` are never inherited by RC.

Raising speed limits is a calibration change with recorded test evidence, not an integration shortcut.

## 9. Test strategy

### 9.1 Static architecture tests

Tests prove:

- core package manifests and launch files do not depend on platform packages;
- core launch contains no platform brand/protocol strings;
- platform race launch files include the shared race launch and do not instantiate shared algorithm nodes;
- exactly one VehicleCmdGate and one runtime manager exist in a full composition;
- no `/autoracer/control/safe_control_cmd` production path exists.

### 9.2 Unit tests

Tests cover:

- RC serial encode/decode and checksum;
- fragmented, concatenated, malformed, and out-of-range telemetry;
- velocity/gear/steering conversion and timeout stop;
- C32 point-field normalization and unsupported layouts;
- parameter consistency between vehicle information, gate limits, and adapter limits;
- manual localization initialization with GNSS absent;
- RC planning limit propagation and conservative defaults.

### 9.3 No-hardware launch tests

With drivers disabled, launch descriptions must resolve, package dependencies must be discoverable, and core composition must not require Fixposition, Pandar, CAN, C32, HiPNUC, or UART nodes.

### 9.4 Bench validation

Bench validation is staged:

1. C32 packet reception, field layout, frame, timestamp, and point rate.
2. IMU message rate, units, axis signs, and stationary gravity.
3. Serial connection and telemetry with drive output disabled.
4. Wheels lifted or chassis secured; `0`, small positive velocity, and small positive/negative steering commands.
5. Command timeout, serial disconnect, localization loss, and emergency-stop behavior.

### 9.5 Low-speed vehicle validation

Only after bench evidence passes:

- manual initialization and NDT convergence;
- straight-line velocity and stop response at or below `0.5 m/s`;
- positive/negative steering sign;
- fixed-course tracking at conservative speed;
- route completion and automatic stop;
- fault injection for stale vehicle status and serial disconnect.

## 10. Migration and non-goals

### 10.1 Allowed reference extraction

The implementation must not consult the legacy tree by default. If current firmware, upstream driver, physical measurement, and live capture evidence are all unavailable, the legacy tree may be used only to identify a historical assumption for later verification, limited to:

- initial calibration measurements;
- historical hardware device settings;
- historical status behavior that is explicitly labelled unverified.

Each historical assumption must be labelled `reference_unverified` in the validation record and replaced by current evidence before hardware completion. No legacy source, launch file, or package graph is copied.

When the same fact is present in `RCCar-Firmware`, the current firmware repository wins and the legacy copy is not consulted.

### 10.2 Explicitly forbidden

- copying the legacy RC launch/profile structure;
- adding a second project command gate;
- restoring `/autoracer/control/safe_control_cmd`;
- copying localization, planning, control, or safety packages for RC;
- placing RC conditionals in Hooke2 launch files;
- placing Hooke2/RC conditionals inside core algorithms;
- making the legacy repository a dependency;
- claiming hardware verification from unit or simulation results;
- carrying Hooke2 sensor TF or Fixposition wiring inside core.

### 10.3 Outside this scope

- dynamic obstacle perception;
- general-road mission/behavior planning;
- high-speed RC tuning;
- new GNSS hardware;
- firmware changes;
- protocol emulation beyond the commands and feedback used by the race stack.

## 11. Completion criteria

The integration is complete only when:

1. Hooke2 still builds and its platform composition still reaches the same shared core.
2. RC product packages build from the pinned dependency underlay.
3. Core contains no platform-specific sensor, TF, transport, or protocol wiring.
4. RC can launch sensing and vehicle boundaries independently.
5. RC full composition contains one localization chain, one planner, one controller, one runtime manager, and one VehicleCmdGate.
6. All architecture, unit, launch, lint, and package tests pass.
7. Bench evidence validates active C32, IMU, and STM32 protocol behavior.
8. Low-speed vehicle evidence validates initialization, tracking, stopping, and fault handling.
9. Documentation records exact hardware, firmware, calibration, dependency revisions, commands, results, and remaining assumptions.
