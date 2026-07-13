# RC Platform Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the RC race car into the compact Autoracer repository as a thin platform target while preserving one shared localization, planning, control, safety, and runtime stack.

**Architecture:** Repair the existing core/platform boundary first, then add RC-only sensor and UART adapters, RC description/configuration, and a thin RC composition root. The legacy repository is never a dependency or launch source; only explicitly tested hardware facts may be re-derived from it. Hooke2 and RC both terminate at the same standard Autoware topics and the same `autoracer_bringup/race.launch.py`.

**Tech Stack:** ROS 2, Python 3, CMake/ament, pytest, launch/launch_ros, Autoware messages and VehicleCmdGate, LeiShen C32 ROS 2 driver, HiPNUC ROS 2 driver, serial UART.

**Authoritative design:** `docs/superpowers/specs/2026-07-14-rc-platform-integration-design.md`

---

## Execution rules

1. Work from repository root `/home/milesli/Desktop/RC/autoracer_hooke` at baseline `f214db3` or a descendant containing only reviewed changes.
2. Create an isolated worktree before implementation. Do not implement directly in the archived legacy tree.
3. Never add `../legacy-refference/autoracer_hooke-autoware-legacy` to CMake, package manifests, scripts, environment hooks, dependency locks, or launch files.
4. Use the legacy tree only when a task explicitly names a hardware fact to verify. Rewrite that fact under the new package boundary and lock it with a new test.
5. Do not copy any legacy launch file, profile, command gate, top-level package graph, or `/autoracer/control/safe_control_cmd` wiring.
6. Treat `/home/milesli/Desktop/RC/RCCar-Firmware` `main` at `4113141f1ac5ba1af276db3c2bace81b5bcf1d16` as the serial protocol source of truth. If its revision changes, re-read its protocol document and C implementation before continuing.
7. Preserve user changes. Before every commit, inspect `git status --short` and stage only files from the current task.
8. All commits must follow the repository Lore Commit Protocol. Every commit below includes an acceptable template.
9. Stop hardware execution if an emergency stop operator, secured vehicle, or required device is unavailable. Unit and no-hardware results must never be reported as on-car validation.

## Final file map

### Create

- `src/core/autoracer_bringup/test/test_platform_boundary.py`
- `src/core/autoracer_localization/test/test_localization_platform_boundary.py`
- `src/platform/hooke2/autoracer_hooke2_bringup/config/hooke2/controller.param.yaml`
- `src/platform/hooke2/autoracer_hooke2_bringup/config/hooke2/vehicle_cmd_gate.param.yaml`
- `src/platform/hooke2/autoracer_hooke2_bringup/config/hooke2/race_runtime.param.yaml`
- `src/platform/rc/autoracer_rc_adapter/{package.xml,setup.py,setup.cfg}`
- `src/platform/rc/autoracer_rc_adapter/autoracer_rc_adapter/{__init__.py,rc_serial_protocol.py,rc_serial_interface.py,c32_pointcloud_adapter.py}`
- `src/platform/rc/autoracer_rc_adapter/test/{test_rc_serial_protocol.py,test_rc_vehicle_contract.py,test_c32_pointcloud_adapter.py}`
- `src/platform/rc/autoracer_rc_description/{package.xml,CMakeLists.txt}`
- `src/platform/rc/autoracer_rc_description/config/{vehicle_info.param.yaml,sensor_extrinsics.yaml}`
- `src/platform/rc/autoracer_rc_description/launch/static_tf.launch.py`
- `src/platform/rc/autoracer_rc_description/urdf/rc_sensor_mounts.urdf.xacro`
- `src/platform/rc/autoracer_rc_bringup/{package.xml,CMakeLists.txt}`
- `src/platform/rc/autoracer_rc_bringup/launch/{sensing.launch.py,vehicle.launch.py,race.launch.py}`
- `src/platform/rc/autoracer_rc_bringup/config/rc/{lidar.param.yaml,imu.param.yaml,imu_filter.param.yaml,planning.param.yaml,controller.param.yaml,vehicle_cmd_gate.param.yaml,race_runtime.param.yaml}`
- `src/platform/rc/autoracer_rc_bringup/test/{test_rc_launch_contract.py,test_rc_parameter_contract.py}`
- `docs/rc_platform_contract.md`
- `docs/rc_bench_validation.md`

### Modify

- `src/core/autoracer_bringup/{CMakeLists.txt,package.xml}`
- `src/core/autoracer_bringup/launch/{race.launch.py,planning.launch.py}`
- `src/core/autoracer_localization/{CMakeLists.txt,package.xml}`
- `src/core/autoracer_localization/launch/localization.launch.py`
- `src/core/autoracer_control/launch/race_control.launch.py`
- `src/core/autoracer_safety/launch/race_safety.launch.py`
- `src/core/autoracer_planning/launch/fixed_course_planning.launch.py`
- `src/platform/hooke2/hooke2_description/{CMakeLists.txt,package.xml}`
- `src/platform/hooke2/autoracer_hooke2_bringup/{CMakeLists.txt,package.xml}`
- `src/platform/hooke2/autoracer_hooke2_bringup/launch/{sensing.launch.py,race.launch.py}`
- `dependencies/{vendor-packages.tsv,versions.lock.yaml}`
- `scripts/{build_product.sh,install_rosdeps.sh}`
- `README.md`

### Remove after migration

- `src/core/autoracer_description/`

---

## Task 1: Lock the architecture boundary with failing tests

**Files:**
- Create: `src/core/autoracer_bringup/test/test_platform_boundary.py`
- Create: `src/core/autoracer_localization/test/test_localization_platform_boundary.py`
- Modify: `src/core/autoracer_bringup/CMakeLists.txt`
- Modify: `src/core/autoracer_bringup/package.xml`
- Modify: `src/core/autoracer_localization/CMakeLists.txt`
- Modify: `src/core/autoracer_localization/package.xml`

- [ ] **Step 1: Add source-boundary tests that initially fail**

Implement tests that locate the repository from `Path(__file__).resolve()` and assert:

```python
FORBIDDEN_CORE_TOKENS = (
    "hooke2",
    "fixposition",
    "pandar",
    "nebula",
    "lslidar",
    "hipnuc",
    "rc_serial",
    "safe_control_cmd",
)

def test_core_launches_have_no_platform_tokens():
    core = REPO_ROOT / "src/core"
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in core.rglob("*")
        if path.suffix in {".py", ".xml", ".yaml"}
        and "/test/" not in path.as_posix()
    ).lower()
    for token in FORBIDDEN_CORE_TOKENS:
        assert token not in sources

def test_core_manifests_do_not_depend_on_platform_packages():
    manifests = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (REPO_ROOT / "src/core").glob("*/package.xml")
    ).lower()
    assert "autoracer_hooke2" not in manifests
    assert "autoracer_rc" not in manifests
    assert "hooke2_" not in manifests
```

In the localization test, assert that `localization.launch.py` contains the standard input topics but does not instantiate `autoware_gnss_poser`, `topic_tools` relay, or `autoracer_description`.

- [ ] **Step 2: Register both pytest suites with ament**

Under `if(BUILD_TESTING)`, use `find_package(ament_cmake_pytest REQUIRED)`. Register `ament_add_pytest_test(autoracer_platform_boundary test/test_platform_boundary.py)` in `autoracer_bringup` and `ament_add_pytest_test(autoracer_localization_platform_boundary test/test_localization_platform_boundary.py)` in `autoracer_localization`. Add `<test_depend>ament_cmake_pytest</test_depend>` to both manifests.

- [ ] **Step 3: Run the tests and verify the expected failures**

Run:

```bash
colcon test --base-paths src/core \
  --packages-select autoracer_bringup autoracer_localization \
  --event-handlers console_direct+
```

Expected: failures naming `hooke2_sensor_extrinsics.yaml`, `fixposition`, `autoware_gnss_poser`, `topic_tools`, or `autoracer_description`. A test collection or syntax error is not the expected failure and must be fixed first.

- [ ] **Step 4: Commit the regression tests only**

```text
Protect the platform-neutral core before adding another chassis

The RC integration must not turn core launches into conditional hardware
graphs, so source-level boundary tests establish the prohibited dependencies
before the existing leaks are removed.

Constraint: Legacy launch and package composition are reference-only
Confidence: high
Scope-risk: narrow
Directive: Keep platform sensor, TF, and transport names out of src/core
Tested: Tests execute and fail on the known Hooke2/Fixposition leaks
Not-tested: Runtime launch behavior
```

---

## Task 2: Move Hooke2 description ownership out of core

**Files:**
- Move: `src/core/autoracer_description/config/hooke2_sensor_extrinsics.yaml` -> `src/platform/hooke2/hooke2_description/config/sensor_extrinsics.yaml`
- Move: `src/core/autoracer_description/urdf/hooke2_sensor_mounts.urdf.xacro` -> `src/platform/hooke2/hooke2_description/urdf/sensor_mounts.urdf.xacro`
- Move/adapt: `src/core/autoracer_description/launch/static_tf.launch.py` -> `src/platform/hooke2/hooke2_description/launch/static_tf.launch.py`
- Move/adapt diagnostic assets from `src/core/autoracer_description/{rviz,launch}` into `hooke2_description`
- Remove: `src/core/autoracer_description/`
- Modify: `src/platform/hooke2/hooke2_description/{CMakeLists.txt,package.xml}`
- Modify: `src/platform/hooke2/autoracer_hooke2_bringup/{package.xml,launch/sensing.launch.py}`

- [ ] **Step 1: Extend the failing test with ownership assertions**

Assert that `src/core/autoracer_description` does not exist, `hooke2_description/config/sensor_extrinsics.yaml` does exist, and Hooke2 sensing includes `hooke2_description/launch/static_tf.launch.py`.

- [ ] **Step 2: Move only Hooke2 assets and preserve calibration values**

Use repository-aware moves. Update package-relative paths and xacro names. Do not change numeric transforms in this task.

- [ ] **Step 3: Launch Hooke2 static TF in Hooke2 sensing**

Add one include near the start of Hooke2 `sensing.launch.py`. The include belongs to the Hooke2 platform even when LiDAR or Fixposition drivers are individually disabled.

- [ ] **Step 4: Remove core references and dependencies**

Remove `autoracer_description` from core manifests and localization launch. Remove the old package after all assets have owners.

- [ ] **Step 5: Verify package and launch syntax**

Run:

```bash
python3 -m compileall -q src/platform/hooke2/hooke2_description/launch \
  src/platform/hooke2/autoracer_hooke2_bringup/launch
colcon build --base-paths src/core src/platform \
  --packages-up-to hooke2_description autoracer_hooke2_bringup \
  --symlink-install
```

Expected: all selected packages finish successfully.

- [ ] **Step 6: Commit the ownership move**

```text
Keep calibrated sensor geometry with the platform that owns it

Hooke2 transforms are hardware facts and must not make the shared
localization graph depend on a Hooke2 description package.

Constraint: Preserve the validated transform values exactly
Confidence: high
Scope-risk: moderate
Directive: Platform static TF must be started by platform sensing bringup
Tested: Python launch compilation and selected colcon build
Not-tested: Live TF values on Hooke2 hardware
```

---

## Task 3: Normalize localization inputs and move Fixposition wiring to Hooke2

**Files:**
- Modify: `src/core/autoracer_localization/launch/localization.launch.py`
- Modify: `src/core/autoracer_localization/package.xml`
- Modify: `src/platform/hooke2/autoracer_hooke2_bringup/launch/sensing.launch.py`
- Modify: `src/platform/hooke2/autoracer_hooke2_bringup/package.xml`
- Test: `src/core/autoracer_localization/test/test_localization_platform_boundary.py`

- [ ] **Step 1: Add exact normalized-topic assertions**

The localization test must require:

```python
for topic in (
    "/sensing/lidar/concatenated/pointcloud",
    "/sensing/imu/imu_data",
    "/vehicle/status/velocity_status",
    "/sensing/vehicle_velocity_converter/twist_with_covariance",
):
    assert topic in source
```

It must also assert that GNSS absence does not prevent the AD API bridge from accepting initialization when the GNSS consistency thresholds are zero; use the existing `_auto_retry_enabled` behavior as the contract.

- [ ] **Step 2: Remove platform normalization nodes from core localization**

Delete the `autoware_gnss_poser` include and Fixposition raw-IMU relay from `localization.launch.py`. Keep the vehicle-velocity converter because every platform provides the same `VelocityReport` contract and localization consumes its normalized twist.

- [ ] **Step 3: Add the normalization nodes to Hooke2 sensing**

After the Fixposition driver, instantiate:

- `autoware_gnss_poser` mapping Fixposition fix/orientation to `/sensing/gnss/pose_with_covariance`;
- a `topic_tools relay` mapping `/fixposition/rawimu` to `/sensing/imu/imu_data`.

Both nodes use the existing `launch_fixposition` condition. No Fixposition node is started when that argument is false.

- [ ] **Step 4: Correct package dependencies**

Remove `autoware_gnss_poser` and `topic_tools` from `autoracer_localization/package.xml`. Add them to `autoracer_hooke2_bringup/package.xml`.

- [ ] **Step 5: Run the boundary and existing localization tests**

Run:

```bash
colcon test --base-paths src/core src/platform \
  --packages-select autoracer_localization autoracer_sensing autoracer_hooke2_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failures and no core Fixposition dependency.

- [ ] **Step 6: Commit the normalized boundary**

```text
Let localization depend on sensor meaning instead of sensor brands

Fixposition conversion now terminates inside Hooke2 sensing, leaving the
shared NDT/EKF composition to consume only normalized topics.

Constraint: Preserve Hooke2 topic behavior while allowing GNSS-free RC initialization
Rejected: Conditional Fixposition branches in core | spreads platform knowledge
Confidence: high
Scope-risk: moderate
Directive: New positioning hardware must normalize inside its platform package
Tested: Core boundary, localization, and Hooke2 bringup tests
Not-tested: Live Fixposition initialization
```

---

## Task 4: Make platform dynamics injectable without duplicating core graphs

**Files:**
- Modify: `src/core/autoracer_bringup/launch/{race.launch.py,planning.launch.py}`
- Modify: `src/core/autoracer_planning/launch/fixed_course_planning.launch.py`
- Modify: `src/core/autoracer_control/launch/race_control.launch.py`
- Modify: `src/core/autoracer_safety/launch/race_safety.launch.py`
- Create: three Hooke2 overlay files under `src/platform/hooke2/autoracer_hooke2_bringup/config/hooke2/`
- Modify: `src/platform/hooke2/autoracer_hooke2_bringup/launch/race.launch.py`
- Test: existing control/planning/safety tests and `test_platform_boundary.py`

- [ ] **Step 1: Extend launch-contract tests before changing launches**

Require shared race launch arguments:

```text
vehicle_info_param_file
control_param_file
gate_param_file
runtime_param_file
max_speed_mps
max_accel_mps2
max_decel_mps2
command_latency_sec
stopping_margin_m
```

Require that these values are forwarded exactly once to their owning child launch.

- [ ] **Step 2: Expose planning dynamics**

Replace hard-coded `0.8`, `-1.5`, `0.2`, and `5.0` in the local planner launch dictionary with typed launch arguments for maximum acceleration, maximum deceleration, command latency, and stopping margin. Preserve the current values as shared defaults.

- [ ] **Step 3: Forward controller and safety files**

Add `control_param_file`, `gate_param_file`, and `runtime_param_file` arguments to shared race composition and forward them to existing `race_param_file`, `gate_param_file`, and `runtime_param_file` child arguments. Shared core retains conservative default files so simulation launch remains valid.

- [ ] **Step 4: Move current high-speed values into Hooke2 overlays**

Create Hooke2 overlays using the currently validated values. Set core gate defaults to conservative values no greater than `0.5 m/s`; Hooke2 race explicitly passes its high-speed overlay. Do not silently change Hooke2 effective parameters.

- [ ] **Step 5: Verify effective launch contracts**

Run:

```bash
pytest -q src/core/autoracer_control/test \
  src/core/autoracer_planning/test \
  src/core/autoracer_safety/test \
  src/core/autoracer_bringup/test
python3 -m compileall -q src/core src/platform/hooke2/autoracer_hooke2_bringup/launch
```

Expected: all tests pass. Inspect Hooke2 `race.launch.py` and confirm it only includes two platform launches and one shared race launch.

- [ ] **Step 6: Commit parameter ownership**

```text
Inject vehicle dynamics without multiplying race compositions

Shared launches own parameter schemas while each platform supplies only its
geometry, tuning, and safety values at the thin composition boundary.

Constraint: Hooke2 effective high-speed parameters must remain unchanged
Rejected: Copy shared control and safety launches per platform | duplicate behavior owners
Confidence: high
Scope-risk: moderate
Directive: Add new platform values through existing arguments, not new core branches
Tested: Planning, control, safety, bringup tests and launch compilation
Not-tested: Closed-loop dynamic equivalence
```

---

## Task 5: Pin independently sourced RC sensor dependencies

**Files:**
- Modify: `dependencies/vendor-packages.tsv`
- Modify: `dependencies/versions.lock.yaml`
- Add only when the named upstream build failure is reproduced: `dependencies/patches/lslidar_ros2_humble_compat.patch`
- Add only when the named upstream build failure is reproduced: `dependencies/patches/hipnuc_ros2_humble_compat.patch`

- [ ] **Step 1: Add immutable upstream repositories**

Add these lock entries exactly:

```yaml
- path: vendor/lslidar_ros2
  url: https://github.com/Lslidar/Lslidar_ROS2_driver.git
  revision: 08d692c2adf62f29b991fe44313b17840e4bea8b
- path: vendor/hipnuc_products
  url: https://github.com/hipnuc/products.git
  revision: 5a4380272cd70402e7f8928b05a6af4bfa659807
```

Update `package_count` to match the manifest after adding packages.

- [ ] **Step 2: Select only required packages**

Append sorted manifest entries:

```text
hipnuc_imu	vendor/hipnuc_products/examples/ROS2/hipnuc_ws/src/hipnuc_imu
hipnuc_lib_package	vendor/hipnuc_products/examples/ROS2/hipnuc_ws/src/hipnuc_lib_package
lslidar_driver	vendor/lslidar_ros2/lslidar_driver
lslidar_msgs	vendor/lslidar_ros2/lslidar_msgs
```

Do not select HiPNUC CAN/GNSS packages or unrelated LeiShen branches.

- [ ] **Step 3: Rebuild the dependency underlay**

Run:

```bash
./scripts/import_dependencies.sh --network
./scripts/import_dependencies.sh --verify-only
./scripts/install_rosdeps.sh
./scripts/build_vendor.sh
```

Expected: the four selected RC packages are discoverable and the complete selected underlay builds.

- [ ] **Step 4: Handle incompatibilities only with minimal recorded patches**

If the pinned packages fail on the target ROS distribution, patch only the concrete build/API incompatibility. Record the upstream file, exact failure, and removal condition in the patch header and `versions.lock.yaml`. Do not import the legacy copies.

- [ ] **Step 5: Commit reproducible dependencies**

```text
Make RC sensor drivers reproducible without depending on legacy sources

The product underlay selects only the C32 and serial HiPNUC packages from
independent upstream revisions.

Constraint: Third-party sources stay in the ignored vendor underlay
Rejected: Legacy repository as a dependency | couples the new architecture to obsolete composition
Confidence: medium
Scope-risk: moderate
Directive: Update revisions only with build and device regression evidence
Tested: Dependency verification, rosdep, and vendor build
Not-tested: Live sensor compatibility
```

---

## Task 6: Implement the RC serial codec and vehicle adapter test-first

**Files:**
- Create: `src/platform/rc/autoracer_rc_adapter/` package files
- Create: `autoracer_rc_adapter/rc_serial_protocol.py`
- Create: `autoracer_rc_adapter/rc_serial_interface.py`
- Create: `test/test_rc_serial_protocol.py`
- Create: `test/test_rc_vehicle_contract.py`

- [ ] **Step 1: Scaffold one small Python adapter package**

Declare runtime dependencies on `rclpy`, `autoware_control_msgs`, `autoware_vehicle_msgs`, `sensor_msgs`, `std_msgs`, and `python3-numpy`. Register console scripts `rc_serial_interface` and `c32_pointcloud_adapter`.

- [ ] **Step 2: Write failing serial codec tests**

Read and cross-check these current firmware sources before writing the fixtures:

```text
/home/milesli/Desktop/RC/RCCar-Firmware/docs/protocols/serial-protocol.md
/home/milesli/Desktop/RC/RCCar-Firmware/WHEELTEC_APP/SerialControl_task.c
/home/milesli/Desktop/RC/RCCar-Firmware/WHEELTEC_APP/data_task.c
```

Confirm the firmware repository is still at `4113141f1ac5ba1af276db3c2bace81b5bcf1d16`; if not, record the new hash and reconcile every byte-level assertion with the new source. Tests must define the checked-in firmware contract explicitly:

```python
def test_control_frame_contract():
    frame = encode_control_frame(1.0, 0.0, -0.25, stop=False)
    assert len(frame) == 11
    assert frame[0] == 0x7B
    assert frame[-1] == 0x7D
    assert frame[3:5] == b"\x03\xe8"
    assert frame[5:7] == b"\x00\x00"
    assert frame[7:9] == b"\xff\x06"
    assert frame[9] == bcc(frame[:9])

def test_stop_bit_is_explicit():
    assert encode_control_frame(0.0, 0.0, 0.0, stop=True)[2] == 0x80
```

Also test signed saturation, telemetry length, delimiters, XOR checksum, fragmented buffer, garbage-prefix resynchronization, concatenated frames, and bad checksum rejection.

- [ ] **Step 3: Implement the minimal codec**

Implement constants `FRAME_HEAD=0x7B`, `FRAME_TAIL=0x7D`, `CONTROL_FRAME_LEN=11`, `TELEMETRY_FRAME_LEN=24`, big-endian signed milliscale fields, and XOR BCC. Do not add protocol fields not consumed by the race stack.

- [ ] **Step 4: Write failing pure conversion tests for the vehicle adapter**

Extract pure functions so tests do not need a serial device. Use the exact signatures `control_to_motion(target_velocity, steering_tire_angle, gear, wheelbase, max_speed, max_steer)` and `telemetry_to_status(telemetry, last_steer, wheelbase, max_steer)`. The first returns `(vx_mps, vy_mps, wz_rad_s, stop)`; the second returns normalized velocity, steering, and gear values used to populate Autoware reports.

Cover drive/reverse/neutral, clamping, positive and negative steering, zero-speed steering fallback, implausible telemetry, and timeout-to-stop.

- [ ] **Step 5: Implement the ROS node around the pure functions**

The node must consume the standard control/gear topics, provide `/control/control_mode_request`, publish all four required status topics, report `NOT_READY` while disconnected, reconnect without blocking callbacks, and send a stop frame when the command age exceeds `command_timeout_sec`.

Defaults are `115200` baud, `0.5 s` timeout, `30 Hz` command, and `50 Hz` feedback. The device path is a required launch argument; do not hide `/dev/ttyUSB0` as a production constant.

- [ ] **Step 6: Run adapter tests**

Run:

```bash
pytest -q src/platform/rc/autoracer_rc_adapter/test/test_rc_serial_protocol.py \
  src/platform/rc/autoracer_rc_adapter/test/test_rc_vehicle_contract.py
```

Expected: all codec and pure conversion tests pass without hardware.

- [ ] **Step 7: Commit the vehicle boundary**

```text
Translate the shared control contract at the RC UART boundary

The adapter owns the minimum serial framing and status translation required
by the shared runtime while keeping private bytes out of core.

Constraint: Codec follows RCCar-Firmware 4113141; the flashed image still requires a bench check
Rejected: Reuse legacy vehicle launch and command gate | violates the single-core contract
Confidence: medium
Scope-risk: moderate
Directive: Update codec fixtures from current firmware sources, never from legacy ROS code
Tested: Codec, corruption, conversion, clamping, and timeout unit tests
Not-tested: Active STM32 firmware
```

---

## Task 7: Implement C32 normalization test-first

**Files:**
- Create: `src/platform/rc/autoracer_rc_adapter/autoracer_rc_adapter/c32_pointcloud_adapter.py`
- Create: `src/platform/rc/autoracer_rc_adapter/test/test_c32_pointcloud_adapter.py`

- [ ] **Step 1: Write synthetic PointCloud2 tests**

Construct little-endian messages with `x`, `y`, `z`, `intensity`, and `ring`. Require output fields and offsets:

```text
x float32 @ 0
y float32 @ 4
z float32 @ 8
intensity uint8 @ 12
return_type uint8 @ 13
channel uint16 @ 14
point_step = 16
```

Test header/stamp preservation, intensity clamping, ring-to-channel conversion, organized width/height preservation, missing XYZ rejection, and unsupported datatype rejection.

- [ ] **Step 2: Implement one pure conversion function and one thin node**

Implement `c32_to_point_xyzirc(msg, default_return_type)` using NumPy structured dtypes. Subscribe to `/sensing/lidar/raw/pointcloud` with sensor-data QoS and publish `/sensing/lidar/concatenated/pointcloud`. On malformed layouts, log a throttled warning and publish nothing.

- [ ] **Step 3: Run the complete adapter test suite**

Run:

```bash
pytest -q src/platform/rc/autoracer_rc_adapter/test
```

Expected: all tests pass without LiDAR or serial hardware.

- [ ] **Step 4: Commit sensor normalization**

```text
Normalize C32 data once at the RC sensing boundary

The shared localization stack now receives the same point contract regardless
of whether the platform driver is LeiShen or Nebula.

Constraint: Never relabel an incompatible byte layout as a standard point type
Confidence: high
Scope-risk: narrow
Directive: Extend the adapter only when a captured driver message proves a new layout
Tested: Synthetic layouts, conversion, clamping, and malformed-input rejection
Not-tested: Live C32 packet stream
```

---

## Task 8: Add RC description with one source for geometry and extrinsics

**Files:**
- Create: `src/platform/rc/autoracer_rc_description/{package.xml,CMakeLists.txt}`
- Create: `config/vehicle_info.param.yaml`
- Create: `config/sensor_extrinsics.yaml`
- Create: `launch/static_tf.launch.py`
- Create: `urdf/rc_sensor_mounts.urdf.xacro`
- Create or extend: `src/platform/rc/autoracer_rc_bringup/test/test_rc_parameter_contract.py`

- [ ] **Step 1: Write parameter consistency tests**

Load YAML and assert finite positive dimensions, `max_steer_angle` in `(0, pi/2)`, and exact starting reference values:

```text
wheel_radius: 0.115
wheel_base: 0.600
wheel_tread: 0.440
max_steer_angle: 0.262
```

Mark these values as `reference_unverified` in documentation, not in ROS parameters.

- [ ] **Step 2: Create the description package**

Install `config`, `launch`, and `urdf`. Define `base_link`, `lidar_top`, and `imu_link`. Read all translation/rotation values from `sensor_extrinsics.yaml`; do not repeat numeric transforms in Python and xacro.

- [ ] **Step 3: Validate files and transforms statically**

Run:

```bash
python3 -c 'import yaml; yaml.safe_load(open("src/platform/rc/autoracer_rc_description/config/vehicle_info.param.yaml")); yaml.safe_load(open("src/platform/rc/autoracer_rc_description/config/sensor_extrinsics.yaml"))'
python3 -m compileall -q src/platform/rc/autoracer_rc_description/launch
```

Expected: zero parse/compile errors.

- [ ] **Step 4: Commit RC physical facts**

```text
Give RC geometry and sensor placement one auditable owner

The description package supplies shared control and localization with one
vehicle-information file and one extrinsics file.

Constraint: Initial dimensions are reference values pending physical measurement
Confidence: medium
Scope-risk: narrow
Directive: Update geometry only with a measurement record and low-speed regression
Tested: YAML, bounds, package, and launch syntax checks
Not-tested: Physical dimensions and sensor alignment
```

---

## Task 9: Add the thin RC platform composition

**Files:**
- Create: `src/platform/rc/autoracer_rc_bringup/{package.xml,CMakeLists.txt}`
- Create: `launch/{sensing.launch.py,vehicle.launch.py,race.launch.py}`
- Create: `config/rc/*.param.yaml`
- Create: `test/{test_rc_launch_contract.py,test_rc_parameter_contract.py}`

- [ ] **Step 1: Write launch source-contract tests first**

Require:

```python
def test_rc_race_is_only_a_composition_root():
    source = RACE_LAUNCH.read_text(encoding="utf-8")
    for launch_name in ("sensing.launch.py", "vehicle.launch.py"):
        assert launch_name in source
    assert '"autoracer_bringup", "race.launch.py"' in source
    for forbidden in (
        "autoware_trajectory_follower_node",
        "autoware_vehicle_cmd_gate",
        "race_runtime_manager",
        "ndt_scan_matcher",
        "safe_control_cmd",
    ):
        assert forbidden not in source
```

Require `vehicle.launch.py` to contain only `autoracer_rc_adapter/rc_serial_interface`, and require sensing launch to contain static TF, C32 driver, pointcloud adapter, HiPNUC, and IMU filter wiring but no core algorithm nodes.

- [ ] **Step 2: Implement `sensing.launch.py`**

Declare independently switchable `launch_lidar` and `launch_imu` arguments. Include RC static TF unconditionally. Launch the vendor drivers with platform config and normalize outputs to the standard LiDAR and IMU topics. Do not launch GNSS or create a fake GNSS pose.

- [ ] **Step 3: Implement `vehicle.launch.py`**

Declare required serial port plus baud, command timeout, command rate, feedback rate, and drive-enable arguments. Launch one `rc_serial_interface`. Do not launch VehicleCmdGate or vehicle-velocity converter because core owns both shared behaviors.

- [ ] **Step 4: Implement `race.launch.py`**

Include RC sensing, RC vehicle, and shared race. Pass:

```text
use_sim_time=false
system_run_mode=online
vehicle_info_param_file=PathJoinSubstitution([FindPackageShare("autoracer_rc_description"), "config", "vehicle_info.param.yaml"])
control_param_file=PathJoinSubstitution([FindPackageShare("autoracer_rc_bringup"), "config", "rc", "controller.param.yaml"])
gate_param_file=PathJoinSubstitution([FindPackageShare("autoracer_rc_bringup"), "config", "rc", "vehicle_cmd_gate.param.yaml"])
runtime_param_file=PathJoinSubstitution([FindPackageShare("autoracer_rc_bringup"), "config", "rc", "race_runtime.param.yaml"])
max_speed_mps=0.5
max_accel_mps2=0.4
max_decel_mps2=-0.8
```

Map and course remain required runtime arguments. Do not default them to Hooke2/CarMaker assets.

- [ ] **Step 5: Add minimal RC overlays**

Set maximum steering to `0.262 rad`, gate velocity to `0.5 m/s`, command timeout to no more than `0.5 s`, and emergency deceleration to `-0.8 m/s²` initially. Include only values that differ from core/upstream defaults. Parameter tests must assert that RC limits never exceed vehicle information and that no value equals the Hooke2 `100 m/s` default.

- [ ] **Step 6: Build and test all RC product packages**

Run:

```bash
colcon build --base-paths src/core src/platform \
  --packages-up-to autoracer_rc_bringup --symlink-install
colcon test --base-paths src/core src/platform \
  --packages-up-to autoracer_rc_bringup --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build success and zero test failures without hardware.

- [ ] **Step 7: Commit the RC composition**

```text
Compose RC hardware around the single shared race core

Three thin platform launches isolate sensing, chassis, and full-race diagnosis
without copying any localization, planning, control, or safety graph.

Constraint: RC starts with conservative limits and manual localization initialization
Rejected: Legacy RC profile and second command gate | create a second autonomy stack
Confidence: high
Scope-risk: moderate
Directive: Keep RC race launch as includes plus parameter injection only
Tested: RC package build, launch contracts, parameters, and no-hardware tests
Not-tested: Physical sensors or chassis
```

---

## Task 10: Make product build target-selectable and document the contracts

**Files:**
- Modify: `scripts/build_product.sh`
- Modify: `README.md`
- Create: `docs/rc_platform_contract.md`
- Create: `docs/rc_bench_validation.md`

- [ ] **Step 1: Add a platform build selector**

Support:

```bash
AUTORACER_PLATFORM=hooke2 ./scripts/build_product.sh
AUTORACER_PLATFORM=rc ./scripts/build_product.sh
AUTORACER_PLATFORM=all ./scripts/build_product.sh
```

Map values to package targets without conditional source paths:

```text
hooke2 -> autoracer_bringup autoracer_hooke2_bringup
rc     -> autoracer_bringup autoracer_rc_bringup
all    -> all three targets
```

Unknown values exit nonzero with one-line usage. Default remains `hooke2` to preserve current behavior.

- [ ] **Step 2: Document architecture and runtime commands**

`README.md` must describe one core and two platform targets. `docs/rc_platform_contract.md` must list all standard topics, frame semantics, parameter ownership, dependency revisions, and explicit legacy prohibitions. `docs/rc_bench_validation.md` must contain exact commands for sensing-only, vehicle-only, and full-race launch plus evidence fields for rates, frames, firmware, serial frames, calibration, stop behavior, and operator/E-stop presence.

- [ ] **Step 3: Test both build selections**

Run:

```bash
AUTORACER_PLATFORM=hooke2 ./scripts/build_product.sh
AUTORACER_PLATFORM=rc ./scripts/build_product.sh
bash -n scripts/build_product.sh
```

Expected: both platform selections build; shell syntax passes.

- [ ] **Step 4: Commit build and documentation**

```text
Make platform selection explicit while keeping one product architecture

Build and operator documentation now select a thin platform target without
changing the shared source graph or relying on long-lived platform branches.

Constraint: Hooke2 remains the default build target
Confidence: high
Scope-risk: narrow
Directive: Platform selection belongs in build/bringup, never in core algorithms
Tested: Hooke2 build, RC build, and shell syntax
Not-tested: Hardware launch
```

---

## Task 11: Run repository-wide no-hardware verification

**Files:**
- Modify only files needed to fix failures caused by Tasks 1-10

- [ ] **Step 1: Verify no legacy architecture leaked into production**

Run:

```bash
rg -n 'legacy-refference|safe_control_cmd|autoracer_safety.*command_gate' \
  src scripts dependencies README.md docs/rc_platform_contract.md
```

Expected: no production reference; explanatory documentation may mention forbidden terms only in explicit non-goal sections.

- [ ] **Step 2: Verify core neutrality**

Run:

```bash
rg -n -i 'hooke2|fixposition|pandar|nebula|lslidar|hipnuc|rc_serial|stm32|can_' src/core
```

Expected: no runtime core matches. Test files may contain negative assertions.

- [ ] **Step 3: Run formatting and static checks**

Run the repository's available linters plus:

```bash
python3 -m compileall -q src/core src/platform
git diff --check
```

Expected: zero errors.

- [ ] **Step 4: Run all product tests**

Run:

```bash
./scripts/import_dependencies.sh --verify-only
AUTORACER_PLATFORM=all ./scripts/build_product.sh
colcon test --base-paths src/core src/platform --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all product packages build; zero failed tests.

- [ ] **Step 5: Inspect the resolved graph**

With hardware drivers disabled, launch RC full composition and inspect nodes/topics. The executor must export paths to the validated assets and the guards must pass before launch:

```bash
test -n "${RC_MAP_PATH:-}" && test -d "${RC_MAP_PATH}"
test -n "${RC_COURSE_PATH:-}" && test -d "${RC_COURSE_PATH}"
ros2 launch autoracer_rc_bringup race.launch.py \
  localization_map_path:="${RC_MAP_PATH}" \
  course_path:="${RC_COURSE_PATH}" \
  launch_lidar:=false launch_imu:=false enable_drive_commands:=false
```

Use actual validated paths available in the execution environment. Confirm exactly one controller, one `vehicle_cmd_gate`, one `race_runtime_manager`, and one localization chain. Absence of a validated RC map/course blocks this launch check but does not justify substituting Hooke2 assets; record it under `Not-tested`.

- [ ] **Step 6: Commit only verification-driven corrections**

```text
Close integration gaps found by full product verification

Repository-wide checks confirm that both platform compositions terminate at
one shared race core and that no obsolete command path remains.

Constraint: Corrections are limited to failures introduced by RC integration
Confidence: high
Scope-risk: narrow
Directive: Preserve the architecture tests as release gates
Tested: Dependency verification, all-platform build, full tests, static checks
Not-tested: Hardware items listed in the verification report
```

---

## Task 12: Validate current RC hardware before raising limits

**Files:**
- Update: `docs/rc_bench_validation.md`
- Update only with measured evidence: RC description and overlay YAML files

- [ ] **Step 1: Record hardware identity before starting nodes**

Record date, operator, vehicle identifier, STM32 firmware hash/version, C32 model/serial, HiPNUC model/serial, USB IDs, serial device symlinks, E-stop method, and whether the wheels are lifted or vehicle is restrained.

- [ ] **Step 2: Validate sensing only**

Run RC sensing launch. Record `ros2 topic hz`, one message schema, frame IDs, TF tree, point-field offsets, IMU stationary gravity, angular-rate sign, and dropped/error counts. Compare the live C32 layout against the unit-test fixture; change the adapter only through a new failing fixture if it differs.

- [ ] **Step 3: Validate UART with drive disabled**

Run RC vehicle launch with `enable_drive_commands=false`. Capture one current telemetry frame and compare delimiters, length, checksum, byte order, units, and flag meaning to the codec tests. If any field differs, update the protocol test first, then implementation.

- [ ] **Step 4: Validate secured motion commands**

With an operator and working E-stop, test stop, `0.2 m/s` straight, `+0.05 rad`, `-0.05 rad`, command timeout, and serial disconnect. Confirm standard status topics reflect sign, scale, control mode, and freshness.

- [ ] **Step 5: Measure geometry and extrinsics**

Measure wheelbase, tread, wheel radius, maximum steering, and sensor transforms. Update the single source YAML files and rerun parameter and TF tests. Never tune duplicate constants in launch code.

- [ ] **Step 6: Run low-speed closed loop**

Using a validated RC map and course, initialize manually, verify NDT/EKF convergence, then run at or below `0.5 m/s`. Confirm route tracking, terminal stop, localization-loss stop, status-timeout stop, and serial-disconnect stop.

- [ ] **Step 7: Commit measured calibration and evidence**

```text
Replace RC reference assumptions with measured vehicle evidence

Current hardware captures validate protocol, geometry, sensor frames, and the
shared fail-closed race path at conservative speed.

Constraint: Dynamic tests require a safety operator, E-stop, and controlled area
Confidence: high
Scope-risk: moderate
Directive: Raise speed only in a separate calibration change with fresh evidence
Tested: List exact bench and low-speed scenarios from the completed record
Not-tested: Any speed or condition not present in the completed record
```

---

## Completion audit

Before declaring completion, answer every item with evidence:

- [ ] `git status --short` contains no accidental legacy, vendor workspace, build, install, or log files.
- [ ] `src/core` has no platform sensor, transport, or static-TF dependency.
- [ ] Hooke2 and RC race launches each include one shared core race launch.
- [ ] No production `/autoracer/control/safe_control_cmd` path exists.
- [ ] One VehicleCmdGate and one runtime manager own final safety behavior.
- [ ] Both platform builds pass.
- [ ] All unit, launch, architecture, lint, and static tests pass.
- [ ] Dependency revisions and licenses are recorded.
- [ ] RC sensor and UART behavior are verified against current hardware, or explicitly reported as not tested.
- [ ] RC geometry and extrinsics are measured, or remain clearly marked as reference assumptions.
- [ ] RC low-speed initialization, tracking, stopping, and injected faults are verified before any speed increase.

If any unchecked item affects hardware safety or architectural uniqueness, the implementation is not complete.
