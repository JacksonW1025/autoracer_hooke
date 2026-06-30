# autoracer_hooke Chain Audit For RC Validation

This document is the migration baseline. It describes how the current
`autoracer_hooke` workspace is wired, what data the upper stack consumes, and
where RC-specific work is allowed to live.

The goal is not to design a separate RC autonomy stack. The RC car should fill
the same Autoware-facing contracts that the Hooke vehicle stack expects, so the
localization, planning, and control algorithms can be validated before running
on the real chassis.

## 1. Ground Rules

- Treat `ackermann-nav2-legacy` only as a source of RC hardware facts. Do not
  copy its localization, planning, control, or topic model into Autoware.
- Keep upper-layer algorithms portable. LiDAR localization, Lanelet planning,
  pure pursuit control, and the command gate should not become RC-specific.
- Put chassis differences behind the vehicle adapter boundary. Hooke uses CAN;
  RC uses UART/STM32. Both must expose Autoware vehicle status and consume
  Autoware control commands.
- Use the existing Autoware map contract. The stack expects a directory with
  `pointcloud_map.pcd`, `pointcloud_map_metadata.yaml`, `lanelet2_map.osm`, and
  `map_projector_info.yaml`.
- RC has no Fixposition and no ZED. That is a data-source gap to solve around
  the localization input/initial-pose contract, not a reason to fall back to
  AMCL, slam_toolbox, or other Nav2 behavior.
- A development computer is only for source edits, static checks, and
  hardware-independent unit tests. LiDAR, serial transport, chassis feedback,
  NDT real-time behavior, and full vehicle launch must be validated on the
  current onboard compute. Target deployments should remain compatible with the
  AGX Orin vehicle-compute path; a temporary bringup host is not a new
  architecture boundary.
- Do not commit temporary host IPs, SSH credentials, or local serial-device
  names. Pass them through launch arguments or environment variables.

Evidence files:

- `README.md`
- `docs/minimal_stack.md`
- `docs/sensing_feedback_topics.md`
- `src/autoracer_bringup/launch/track.launch.py`
- `src/autoracer_bringup/launch/sensing.launch.py`
- `src/autoracer_bringup/launch/localization.launch.py`
- `src/autoracer_planning/launch/planning.launch.py`
- `src/autoracer_control/launch/control.launch.py`
- `src/autoracer_safety/launch/safety.launch.py`
- `src/autoracer_bringup/launch/vehicle.launch.py`

## 2. Migration Correction Matrix

These items are the migration guardrails. They are not independent trivia: each
row identifies a way the RC validation effort can accidentally drift away from
the Hooke/Autoware stack that it is supposed to validate.

| Area | Wrong shortcut | Correct boundary | Concrete follow-up |
| --- | --- | --- | --- |
| Reference use | Treat Nav2 legacy behavior as an Autoware design source. | Use Nav2 legacy only for RC hardware facts and historical wiring clues. | Do not copy AMCL, slam_toolbox, `/scan`, `/wheel_odom`, or custom command topics into the upper stack. |
| Map contract | Re-open the question of map source or switch to a Nav2 map. | Keep the Autoware map directory contract: PCD, Lanelet2, projector info, metadata. | Validate RC localization against the same PCD/Lanelet2 map assets. |
| Localization seed | Treat missing Fixposition/ZED as a reason to change localization algorithms. | Preserve NDT-on-PCD localization; replace only the seed/initial-pose source. | Define who publishes `/localization/ndt_initial_pose` or a semantic seed pose on RC. |
| LiDAR input | Use 2D `/scan` because the old RC stack did. | Feed NDT with `PointCloud2` on `/sensing/lidar/concatenated/pointcloud`. | Normalize the RC LiDAR driver/remap/frame into the Autoware pointcloud contract. |
| Vehicle geometry | Update only one wheelbase or steering parameter. | Treat geometry as a single RC vehicle profile used by every kinematic consumer. | Wire the 0.600 m wheelbase set through TF/URDF, predictor, controller, adapter, and firmware. |
| Vehicle feedback | Let old `/wheel_odom`, `/chassis_state`, or `/ackermann_cmd` leak upward. | Publish Autoware `/vehicle/status/*` from the chassis adapter. | Verify velocity, steering, gear, and control-mode status from RC UART telemetry. |
| Vehicle transport | Let Hooke CAN vs RC UART reshape localization/planning/control. | Keep CAN/UART differences inside vehicle adapters. | Make RC serial and Hooke2 CAN both satisfy the same command/status boundary. |
| Firmware behavior | Change upper algorithms around STM32 deadband or PWM details. | Treat deadband, minimum speed, RC override, and PWM mapping as adapter/firmware facts. | Test low-speed command/feedback behavior at the adapter boundary. |
| Safety gate and reverse | Solve reverse or speed semantics by changing one clamp in the gate. | Split reverse support across trajectory intent, control output, gear command, adapter behavior, and chassis limits. | Keep forward validation separate from any later reverse-driving design. |
| Validation order | Enable drive because individual topics appear alive. | Prove each layer before the next one consumes it. | Validate sensing/TF, then NDT, then route, then raw control, then gate, then physical drive. |

The safety-gate/reverse item is only one exposed example of a cross-layer
mistake. The same rule applies to the other rows: do not fix a missing contract
by making an unrelated upper-layer algorithm RC-specific.

## 3. End-To-End Runtime Chain

The intended runtime chain is:

```text
Sensor + vehicle feedback
  -> normalized Autoware sensor/status topics
  -> PCD/Lanelet2 map loaders
  -> GNSS/Fixposition seed path
  -> NDT scan matching against PCD map
  -> /localization/pose_with_covariance
  -> Lanelet centerline route and trajectory
  -> pure pursuit + longitudinal control
  -> safety command gate
  -> /control/command/control_cmd + support commands
  -> vehicle adapter
  -> chassis transport
```

`track.launch.py` starts these pieces in one launch:

| Layer | Launch owner | Main responsibility |
| --- | --- | --- |
| Static TF | `autoracer_description/launch/static_tf.launch.py` | Publish fixed transforms from `base_link` to LiDAR/GNSS/IMU frames. |
| Sensing | `autoracer_bringup/launch/sensing.launch.py` | Start Hesai LiDAR and Fixposition; bridge vehicle speed to Fixposition speed input. |
| Localization | `autoracer_bringup/launch/localization.launch.py` | Load map, build GNSS seed, predict NDT initial pose, run NDT, publish map-to-base pose. |
| Planning | `autoracer_planning/launch/planning.launch.py` | Load Lanelet2 map and publish route/trajectory after `/goal_pose`. |
| Control | `autoracer_control/launch/control.launch.py` | Track `/planning/trajectory` with pure pursuit and longitudinal P control. |
| Safety | `autoracer_safety/launch/safety.launch.py` | Gate raw control into the Autoware vehicle command surface. |
| Vehicle adapter | `autoracer_bringup/launch/vehicle.launch.py` or Hooke2 launch | Convert Autoware commands/status to the actual chassis transport. |

Important separation:

- The upper algorithm chain ends at `/control/command/control_cmd` plus support
  commands.
- The real Hooke adapter is `hooke2_interface`: it consumes the same Autoware
  control topics and publishes `/vehicle/status/*` from CAN feedback.
- The current minimal launch uses `rc_serial_interface`. That should be treated
  as a vehicle adapter implementation, not as evidence that the upper stack
  should become RC-specific.

## 4. Topic And Data Contract

These are the contracts that matter for migration. If RC can produce the same
contracts with correct frame/timing/units, the upper stack can stay mostly
unchanged.

| Topic | Type | Producer | Consumer | Contract role |
| --- | --- | --- | --- | --- |
| `/sensing/lidar/concatenated/pointcloud` | `sensor_msgs/msg/PointCloud2` | `nebula_hesai` | NDT scan matcher | LiDAR input for map localization. Must be in the LiDAR frame connected to `base_link` by TF. |
| `/fixposition/fix` | `sensor_msgs/msg/NavSatFix` | Fixposition driver | `autoware_gnss_poser` | GNSS seed source in Hooke chain. RC does not have this source. |
| `/fixposition/autoware_orientation` | `autoware_sensing_msgs/msg/GnssInsOrientationStamped` | Fixposition driver | `autoware_gnss_poser` | Orientation paired with GNSS seed. RC does not have this source. |
| `/fixposition/fpa/odomstatus` | `fixposition_driver_msgs/msg/FpaOdomstatus` | Fixposition driver | `fixposition_seed_filter` | Optional seed quality/status check; current launch sets `require_status=false`. |
| `/fixposition/speed` | `fixposition_driver_msgs/msg/Speed` | `velocity_to_fixposition_speed` | Fixposition driver | Wheel-speed feedback for Fixposition. Only relevant if Fixposition is present. |
| `/sensing/gnss/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `autoware_gnss_poser` | `fixposition_seed_filter` | Map-frame GNSS pose candidate. RC needs an alternative seed path if no Fixposition. |
| `/localization/fixposition/seed_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `fixposition_seed_filter` | `ndt_initial_pose_predictor`, NDT regularization input | Filtered initial/recovery seed. Name is Fixposition-specific but semantic role is localization seed. |
| `/localization/ndt_initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `ndt_initial_pose_predictor` | NDT scan matcher and startup helper | Initial pose stream for NDT. This is the key contract to fill on RC. |
| `/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | NDT scan matcher | planning, control, safety, pose TF, initial-pose predictor | Authoritative localized pose for upper stack. |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | NDT scan matcher | diagnostics/RViz | Pose output without covariance. Not the main planning/control contract. |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | Hooke2 CAN adapter or RC serial adapter | speed bridge, initial-pose predictor, pure pursuit | Vehicle longitudinal velocity and heading rate. Must use Autoware units. |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | Hooke2 CAN adapter or RC serial adapter | initial-pose predictor | Steering tire angle feedback. |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | Hooke2 CAN adapter or RC serial adapter | vehicle/system consumers | Chassis gear state. Important for reverse semantics and adapter correctness. |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | Hooke2 CAN adapter or RC serial adapter | system/operator consumers | Manual/autonomous readiness state. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | operator/RViz/test tool | `lanelet_route_planner` | Mission goal. |
| `/planning/mission_path` | `nav_msgs/msg/Path` | `lanelet_route_planner` | RViz/debug | Human-readable planned path. |
| `/planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | `lanelet_route_planner` | `pure_pursuit_controller` | Control trajectory with target velocities. |
| `/autoracer/control/raw_control_cmd` | `autoware_control_msgs/msg/Control` | `pure_pursuit_controller` | `command_gate` | Ungated controller output. |
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | `command_gate` | Hooke2 CAN adapter or RC serial adapter | Final Autoware control command to the chassis adapter. |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | `command_gate` | Hooke2 CAN adapter or RC serial adapter | Gear request. Current gate publishes DRIVE when safe, NEUTRAL when unsafe. |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | `command_gate` | Hooke2 CAN adapter | Safety/support command. |
| `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | `command_gate` | Hooke2 CAN adapter | Support command. |

## 5. Sensor And Frame Chain

### LiDAR

The LiDAR path is straightforward:

```text
Hesai/Pandar UDP packets
  -> nebula_hesai decoder
  -> /sensing/lidar/concatenated/pointcloud
  -> autoware_ndt_scan_matcher
```

The default config is `src/autoracer_bringup/config/hooke2/lidar_top.param.yaml`.
The default sensor model passed from `track.launch.py` is `Pandar40P`.

For RC validation:

- The RC LiDAR driver must publish the same `PointCloud2` contract or be
  remapped into it.
- The frame must match the TF tree. If the RC LiDAR frame is not `lidar_top`,
  either normalize the frame name in the adapter/launch or update the RC TF
  profile consistently.
- Do not replace NDT with a 2D `/scan` stack. If RC LiDAR is used for validation,
  it must support the pointcloud localization contract.

### Static TF And Vehicle Geometry

Current Hooke extrinsics live in
`src/autoracer_description/config/hooke2_sensor_extrinsics.yaml` and publish:

```text
base_link -> lidar_top_base_link -> lidar_top
base_link -> gnss_base_link
base_link -> imu_link
```

Current Hooke vehicle dimensions live in
`src/autoracer_bringup/config/hooke2/vehicle_info.param.yaml`:

```text
wheel_base: 1.9
wheel_tread: 1.55
wheel_radius: 0.313
max_steer_angle: 0.488
```

For RC validation:

- The RC vehicle profile must replace these geometry/extrinsics as a coherent
  profile.
- The current confirmed RC firmware defaults are:

```text
wheelbase: 0.600 m
track width: 0.470 m
wheel radius: 0.115 m
wheel diameter: 0.230 m
max steering: 0.262 rad
```

- These values must be propagated consistently to localization prediction,
  controller parameters, vehicle adapter limits, and URDF/TF. A partial update
  is worse than no update because it creates contradictory kinematics.

## 6. Localization Chain

The localization chain is the critical part of this migration.

### Map Loading

`localization.launch.py` loads:

```text
map_projector_info.yaml
lanelet2_map.osm
pointcloud_map.pcd
pointcloud_map_metadata.yaml
```

The pointcloud map loader publishes and serves:

```text
/map/pointcloud_map
/map/get_partial_pointcloud_map
/map/get_differential_pointcloud_map
/map/get_selected_pointcloud_map
```

NDT uses the differential pointcloud map service configured as
`client_map_loader=/map/get_differential_pointcloud_map`.

This means the map source is not ambiguous. RC validation should use the same
Autoware map directory contract.

### Fixposition Seed Path

Current Hooke seed path:

```text
/fixposition/fix
/fixposition/autoware_orientation
  -> autoware_gnss_poser
  -> /sensing/gnss/pose_with_covariance
  -> fixposition_seed_filter
  -> /localization/fixposition/seed_pose
```

`fixposition_seed_filter` checks:

- frame is `map`
- pose freshness
- finite pose/quaternion
- covariance threshold
- jump threshold
- optional Fixposition status

The launch sets:

```text
require_status: false
use_status_when_available: true
```

So `/fixposition/fpa/odomstatus` can improve filtering when available, but it is
not mandatory in the current launch.

### NDT Initial Pose Predictor

Current NDT initial pose path:

```text
/localization/fixposition/seed_pose
/localization/pose_with_covariance
/vehicle/status/velocity_status
/vehicle/status/steering_status
  -> ndt_initial_pose_predictor
  -> /localization/ndt_initial_pose
```

Behavior:

- If no state exists, seed pose initializes the predictor.
- If NDT is lost, seed pose can reinitialize it.
- Once NDT is publishing, NDT pose corrects the predictor state.
- Between corrections, vehicle velocity and yaw rate/steering propagate the pose.

This means RC does not need to imitate Fixposition as a device. RC needs to
provide a reliable source for the semantic contract currently named
`/localization/fixposition/seed_pose`, or otherwise provide
`/localization/ndt_initial_pose` directly through an Autoware-compatible
initial-pose workflow.

### NDT Scan Matcher

Current NDT inputs:

```text
pointcloud: /sensing/lidar/concatenated/pointcloud
initial pose: /localization/ndt_initial_pose
regularization pose: /localization/fixposition/seed_pose
map service: /map/get_differential_pointcloud_map
trigger service: /localization/ndt_trigger
```

Important parameter:

```text
ndt.regularization.enable: false
```

So the current launch wires a regularization topic, but the config disables NDT
regularization. Fixposition is therefore not a hard dependency of the NDT scan
matching algorithm. It is the current source of startup/recovery seed poses.

`ndt_startup_helper` waits for:

- enough fresh `/localization/ndt_initial_pose` messages
- map service readiness
- NDT trigger service readiness

Then it calls `/localization/ndt_trigger`, and later retriggers when NDT pose is
stale or quality diagnostics are poor.

### RC Localization Gap

RC has no Fixposition and no ZED. The key missing item is:

```text
stable map-frame initial/recovery pose for NDT
```

Acceptable directions to evaluate after this audit:

- Manual/operator initial pose for first validation runs, if the closed track
  workflow can tolerate it.
- A lightweight RC seed adapter that publishes the same seed-pose contract from
  a known start pose or external measurement.
- Reuse Autoware NDT startup tooling directly if it can satisfy the same
  `/localization/ndt_initial_pose` contract.

Not acceptable as default:

- Replacing this with Nav2 AMCL/slam_toolbox.
- Treating `/scan` localization as equivalent to the Hooke NDT chain.
- Changing planning/control to hide the missing localization seed.

## 7. Planning Chain

`lanelet_route_planner` consumes:

```text
/localization/pose_with_covariance
/goal_pose
lanelet2_map.osm
map_projector_info.yaml
```

It publishes:

```text
/planning/mission_path
/planning/trajectory
/planning/route_marker
```

The planner:

- loads Lanelet2
- finds nearest lanelets to current pose and goal
- computes a route/shortest path
- samples lanelet centerline points
- assigns `speed_limit_mps` to trajectory points
- sets the final point velocity to zero

For RC validation:

- Do not replace this with Nav2 planners.
- The same Lanelet map and route topic contract should be preserved.
- RC-specific speed limits may be launch/profile parameters, not algorithm
  forks.

## 8. Control Chain

`pure_pursuit_controller` consumes:

```text
/planning/trajectory
/localization/pose_with_covariance
/vehicle/status/velocity_status
```

It publishes:

```text
/autoracer/control/raw_control_cmd
```

The controller:

- computes target point by nearest point plus lookahead
- computes Ackermann tire steering angle from curvature and `wheel_base_m`
- clamps steering by `max_steer_rad`
- computes target speed from trajectory
- clamps current target speed to `[0.0, max_speed_mps]`
- computes acceleration by longitudinal proportional control
- publishes stop when pose or trajectory is unavailable

P0 now wires `wheel_base_m` and `max_steer_rad` from the top-level launch into
`control.launch.py`, so pure pursuit, the NDT initial-pose predictor, the
safety gate, and the RC serial adapter can use the same RC vehicle geometry.
Launching `autoracer_control/launch/control.launch.py` by itself still preserves
the Hooke defaults.

## 9. Safety And Command Gate

`command_gate` consumes:

```text
/autoracer/control/raw_control_cmd
/localization/pose_with_covariance
```

It publishes:

```text
/control/command/control_cmd
/control/command/gear_cmd
/control/command/hazard_lights_cmd
/control/command/turn_indicators_cmd
/autoracer/safety/state
```

Safety conditions:

- `enable_drive_commands` must be true
- raw command must be fresh
- localization pose must be fresh

When unsafe, it publishes a stop command and support commands for NEUTRAL and
hazard lights.

When safe, it limits:

```text
velocity: [0.0, max_speed_mps]
acceleration: [max_decel_mps2, max_accel_mps2]
steering_tire_angle: [-max_steer_rad, max_steer_rad]
steering rate: limited by max_steer_rate_radps
```

Migration note:

- The gate clamp is one cross-layer migration risk, not the only one.
- Do not solve reverse driving by blindly changing the safety gate velocity
  range to `[-3.0, 3.0]`.
- Reverse semantics involve planner/control intent, `GearCommand`, adapter
  behavior, and chassis limits. They need a separate design decision.
- For forward closed-track validation, the current planner and pure pursuit
  chain are forward-speed oriented.

## 10. Vehicle Adapter Boundary

### Hooke2 CAN Adapter

The vendored Hooke2 interface consumes Autoware commands:

```text
/control/command/control_cmd
/control/command/gear_cmd
/control/command/turn_indicators_cmd
/control/command/hazard_lights_cmd
/control/command/actuation_cmd
/control/command/emergency_cmd
```

It publishes Autoware vehicle status:

```text
/vehicle/status/control_mode
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/gear_status
/vehicle/status/turn_indicators_status
/vehicle/status/hazard_lights_status
/vehicle/status/actuation_status
/vehicle/status/steering_wheel_status
/vehicle/status/door_status
/vehicle/status/battery_charge
```

It also translates to and from raw `/hooke2/*` CAN-level topics. Those raw topics
should stay inside the adapter/debug boundary.

### RC UART Adapter

The current Python RC adapter consumes:

```text
/control/command/control_cmd
/control/command/gear_cmd
```

It publishes:

```text
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/gear_status
/vehicle/status/control_mode
```

It sends an 11-byte UART command frame:

```text
0x7B cmd1 cmd2 vx vy wz bcc 0x7D
```

It parses a 24-byte telemetry frame:

```text
0x7B flag vx vy wz reserved... battery bcc 0x7D
```

For migration, this adapter must remain below the Autoware boundary. The upper
stack should not consume legacy RC topics such as `/wheel_odom`, `/chassis_state`,
or custom `/ackermann_cmd`.

### RC Firmware Facts

Current confirmed firmware profile in `RCCar-Firmware`:

```text
APP_ORIN_ACKERMANN_WHEELBASE_MM: 600
APP_ORIN_ACKERMANN_TRACK_WIDTH_MM: 470
APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM: 115
APP_ORIN_ACKERMANN_MAX_STEERING_MRAD: 262
APP_ORIN_ACKERMANN_MIN_VX_MMPS: 50
APP_ORIN_VX_DEADBAND_MMPS: 50
APP_ORIN_VX_FORWARD_CAP_MMPS: 3000
APP_ORIN_VX_REVERSE_CAP_MMPS: 3000
```

Firmware behavior relevant to the adapter:

- UART parser accepts `vx`, `vy`, `wz`, and stop flag.
- `vy` is ignored in Ackermann mode.
- Small velocities below the neutral threshold are converted to neutral output.
- Steering command mapping needs enough `|vx|` to compute Ackermann steering
  from yaw rate.
- Telemetry uses Hall speed for `vx` and estimates `wz` from steering PWM and
  measured speed.

These facts matter for adapter validation. They should not force upper-layer
algorithm changes unless the Autoware contract cannot be satisfied.

## 11. RC Validation Mapping

| Hooke/autoware contract | RC source or action | Gap class | Status |
| --- | --- | --- | --- |
| PCD/Lanelet2 map directory | Use same map contract as Hooke chain | Map contract | Must preserve |
| `/sensing/lidar/concatenated/pointcloud` | RC LiDAR driver/remap/profile | Sensor input | Must implement/verify |
| `base_link -> lidar_top` TF | RC URDF/static TF profile | Sensor/TF profile | Must replace Hooke extrinsics |
| `/fixposition/fix` and `/fixposition/autoware_orientation` | No RC device | Missing sensor source | Must not be faked as a device |
| `/sensing/gnss/pose_with_covariance` | Only needed if keeping GNSS seed path | Localization seed | Gap unless seed path is redesigned |
| `/localization/fixposition/seed_pose` | Semantic seed pose can be produced by another source | Localization seed | Must redesign without Nav2 fallback |
| `/localization/ndt_initial_pose` | Initial-pose predictor or direct seed publisher | Localization startup | Must provide for NDT startup |
| `/localization/pose_with_covariance` | NDT output | Upper-layer localization contract | Must stay the planning/control pose source |
| `/vehicle/status/velocity_status` | RC UART telemetry via adapter | Vehicle status | Must verify units, rate, sign, and deadband effects |
| `/vehicle/status/steering_status` | RC UART telemetry/last steering command via adapter | Vehicle status | Must verify steering sign and physical calibration |
| `/vehicle/status/gear_status` and `/vehicle/status/control_mode` | RC adapter reports Autoware status | Vehicle status | Must verify before enabling drive |
| `/control/command/control_cmd` | Safety gate output consumed by RC adapter | Command boundary | Must preserve |
| `/control/command/gear_cmd` | Gate output consumed by RC adapter | Command boundary/design decision | Must preserve, especially before reverse design |
| Controller wheelbase/max steer | RC vehicle profile | Parameter consistency | Must wire consistently |
| Localization predictor wheelbase | RC vehicle profile | Parameter consistency | Must wire consistently |
| Firmware velocity deadband | Adapter/firmware fact | Adapter behavior | Record and test; do not leak upward |
| RC override/guard behavior | STM32 firmware + serial adapter state | Adapter behavior | Test as chassis safety behavior, not planning logic |

## 12. Current P0 Implementation

P0 only fills the RC hardware/configuration entry points. It does not introduce
RC-specific localization, planning, or control algorithms.

Implemented:

- Added `autoracer_description/config/rc_sensor_extrinsics.yaml` for
  `base_link -> lidar_top` with `x=0.24, y=0.0, z=0.39, yaw=-1.5708`.
- Added `track_rc_p0.launch.py` as a thin profile wrapper around
  `track.launch.py`; it does not duplicate the algorithm chain.
- Uses the Leishen C32 profile with the legacy C32 network settings:
  `device_ip=192.168.1.200`, `msop_port=2368`, and `difop_port=2369`.
  The C32 cloud is published directly as
  `/sensing/lidar/concatenated/pointcloud` in frame `lidar_top`.
- Disabled Fixposition and enabled a manual seed publisher for the RC profile.
- `manual_seed_pose_publisher` subscribes to RViz/ROS `/initialpose`, publishes
  the calibrated map-frame seed to `/localization/fixposition/seed_pose`, and
  keeps the original `ndt_initial_pose_predictor` producing
  `/localization/ndt_initial_pose`. The RC profile sets
  `require_input_pose=true`, so it does not publish a fake `0,0,0` seed.
- Wired `wheel_base_m` and `max_steer_rad` through top-level launch to
  localization, control, safety, and the vehicle adapter.
- Set the RC P0 geometry to `wheel_base_m=0.6` and `max_steer_rad=0.262`.
  Firmware also records `track_width=0.470`, `wheel_radius=0.115`, and
  `wheel_diameter=0.230`.
- Controller, command gate, and RC serial adapter now follow the Autoware signed
  velocity/gear contract: speed is no longer hard-clamped to `[0, max]`, the
  gate publishes `DRIVE` or `REVERSE` from signed velocity, and the serial
  adapter stops on gear/velocity direction mismatches.
- Firmware forward/reverse speed caps are synchronized to `3.0 m/s`; the
  `0.05 m/s` low-speed deadband remains.

Still outside P0:

- Map assets are not included. Full localization/planning still requires
  `MAP_PATH` or `map_path` to point at an Autoware PCD/Lanelet2 map directory.
- IMU is not connected because the P0 NDT chain has no IMU consumer.
- Full reverse planning/behavior is still outside P0; P0 only fixes the
  control/gate/adapter contract so it is not hard-coded forward-only.
- Firmware low-speed behavior is not bypassed. Current firmware has
  `APP_ORIN_ACKERMANN_MIN_VX_MMPS=50`, `APP_ORIN_VX_DEADBAND_MMPS=50`, and
  3.0 m/s forward/reverse caps.

Example P0 entry point:

```bash
ros2 launch autoracer_bringup track_rc_p0.launch.py \
  map_path:=/path/to/autoware_map
```

Publish `/initialpose` from RViz/ROS after launch. That pose is the calibrated
semantic seed; default zero values are not used as a real localization initial
pose in the RC profile.

## 13. Implementation Sequence From This Baseline

Do the next phase in this order:

1. Continue calibrating the RC vehicle profile:
   - C32 extrinsics start from the legacy Nav2 values and must be calibrated on the car.
   - IMU can be connected but is not a P0 localization consumer.
   - launch wiring so controller, localization prediction, and vehicle adapter
     read the same geometry.

2. Validate RC sensing:
   - ensure C32 produces `/sensing/lidar/concatenated/pointcloud`
   - verify frame IDs and TF
   - record IMU availability, but only connect it to the chain if a consumer is
     explicitly introduced.

3. Validate NDT startup seed:
   - do not use Nav2 AMCL/slam_toolbox
   - use `/initialpose -> /localization/fixposition/seed_pose -> /localization/ndt_initial_pose`
   - validate NDT against the PCD map before enabling planning/control

4. Validate vehicle adapter:
   - UART command/telemetry frame compatibility
   - speed sign, steering sign, and yaw-rate reconstruction
   - low-speed deadband behavior
   - Autoware `/vehicle/status/*` publication rate and freshness

5. Only then run upper stack:
   - localization first
   - planning route generation second
   - raw control output third
   - safety-gated output last
   - physical drive only after dry-run topic checks are clean

## 14. What Not To Change Yet

- Do not add RC-specific planning/control algorithms.
- Do not reintroduce Nav2 localization as the default.
- Do not replace pointcloud NDT validation with a 2D `/scan` localization stack.
- Do not consume `/wheel_odom`, `/chassis_state`, or custom `/ackermann_cmd` in
  the Autoware upper stack.
- Do not change speed limits or reverse support by only editing the final gate.
- Do not treat missing Fixposition as a map-source problem.
- Do not trust a partial vehicle parameter update; geometry must be consistent
  across TF, localization prediction, control, adapter, and firmware.
- Do not treat firmware deadband or UART PWM details as reasons to fork upper
  localization, planning, or control algorithms.

## 15. Acceptance Checklist

Before coding the next migration phase, the following must be true:

- Every upper-layer input topic has a known RC producer or an explicit gap.
- Every adapter-only topic stays behind the vehicle/sensor boundary.
- The correction matrix has no unresolved row that is being bypassed by an
  algorithm fork.
- The map contract remains the Autoware PCD/Lanelet2 contract.
- NDT startup uses the concrete non-Nav2 seed strategy: RViz/ROS `/initialpose`.
- RC vehicle geometry is represented once as a coherent profile and wired into
  every consumer.
- The firmware low-speed/deadband behavior is tested at the adapter boundary.
- `autoracer_hooke` can still be reasoned about as one Autoware-style stack, not
  a mixture of unrelated RC legacy behaviors.
