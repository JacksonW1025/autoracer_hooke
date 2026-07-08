# Ackermann Autoware

ROS 2 workspace for using the RC Ackermann car to validate the Autoracer
Hooke/Autoware chain.

The current priority is to keep the Hooke and RC branches on the same upper stack
and validate that stack on the RC platform. RC-specific work belongs in sensor
profiles, vehicle geometry, serial vehicle interface, and mapping workflow:

```text
Leishen C32 LiDAR + RViz/manual NDT seed + STM32 UART feedback
  -> PCD/Lanelet2 map
  -> LiDAR/NDT localization
  -> Lanelet centerline route
  -> Pure pursuit + longitudinal PID
  -> safety command gate
  -> /control/command/control_cmd
  -> rc_serial_interface
  -> UART4 11-byte command frame
  -> STM32 Ackermann PWM
```

## Runtime Boundary

This checkout may be edited and statically tested on a development computer, but
hardware bringup must be validated on the current onboard compute. Target
deployments should stay compatible with the AGX Orin vehicle-compute path; any
temporary RC host is only a bringup host, not a new architecture boundary.

Do not commit temporary host IPs, SSH credentials, or machine-local serial device
names. Pass those at launch time through environment variables such as
`SERIAL_PORT`, `MAP_PATH`, and the `LIDAR_*` settings.

## Repository Layout

```text
autoracer.repos            Dependency manifest for selected external packages.
defaults.env               Runtime defaults used by scripts/run_track.sh.
docs/                      Bringup and calibration notes.
maps/                      Local map directory placeholder.
scripts/                   Import, build, run, and smoke-test helpers.
src/autoracer_bringup      Top-level launches and Hooke2 configuration.
src/autoracer_description  Hooke2/RC frames, URDFs, and static TF launch.
src/autoracer_localization Localization helper nodes.
src/autoracer_sensing      Minimal sensor/vehicle feedback adapters.
src/autoracer_planning     Transitional local lanelet route and trajectory node.
src/autoracer_control      Transitional local pure pursuit controller.
src/autoracer_safety       Transitional local command gate before the vehicle interface.
src/autoracer_vehicle_interface RC UART vehicle interface and Autoware status bridge.
src/hardware_drivers       Vendored SocketCAN driver used by Hooke2.
src/hooke2_vehicle         Vendored Hooke2 interface, launch, and description.
src/wd_msgs                Vendored Hooke2 chassis messages and byte helpers.
```

## Documentation

Start here:

```text
docs/README_zh.md
```

The docs are organized by task:

```text
docs/architecture/  Shared stack boundary and official Autoware migration notes.
docs/operations/    RC field runbook and mapping workflow.
docs/reference/     Topic, frame, vehicle, feedback, and calibration facts.
```

## First Bringup

```bash
cd /home/corage/workspace/project/autoracer-hooke
./scripts/import_dependencies.sh
./scripts/install_rosdeps.sh
./scripts/build_minimal.sh
source ./scripts/ros_env.sh
```

The RC Autoware RViz profile uses official Autoware/Tier IV RViz plugins.
Install the desktop/build dependencies before building those packages:

```bash
sudo apt install -y \
  libpng++-dev \
  libpng-dev \
  nlohmann-json3-dev \
  ros-humble-autoware-motion-utils \
  ros-humble-foxglove-bridge \
  ros-humble-rviz-2d-overlay-msgs \
  ros-humble-rviz-2d-overlay-plugins
```

On a resource-constrained onboard host, lower build parallelism without changing
the workspace:

```bash
COLCON_PARALLEL_WORKERS=1 MAKEFLAGS="-j2 -l2" ./scripts/build_minimal.sh
```

When developing beside the old repository, dependencies can be copied locally instead
of fetched:

```bash
IMPORT_FROM_PILOT=true ./scripts/import_dependencies.sh
```

## RC Flow Entrypoints

RC vehicle-side flow scripts live under `scripts/rc/`. These are the stable
operator-facing entry points; lower-level scripts in `scripts/` are helpers used
by those flows.

Prepare the C32 Ethernet link on the vehicle host:

```bash
sudo -E ./scripts/rc/rc_configure_lidar.sh
```

Mapping is performed on the development workstation, not on the vehicle host:

```text
/home/milesli/Desktop/RC/rc_mapping_ws
/home/milesli/Desktop/RC/rc_mapping_data
```

Start only the mapping sensors and TF:

```bash
IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_start_sensors.sh
```

Capture a mapping bag with one command:

```bash
BAG_DURATION_SEC=60 IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_capture_mapping_bag.sh
```

For open-ended field capture, start and finish recording explicitly:

```bash
IMU_SERIAL_PORT=/dev/ttyUSB0 ./scripts/rc/rc_start_mapping_bag.sh
./scripts/rc/rc_stop_mapping_bag.sh
```

Start localization-only against a PCD-only map:

```bash
MAP_PATH=/home/milesli/autoracer_maps/<map_name> \
IMU_SERIAL_PORT=/dev/ttyUSB0 \
./scripts/rc/rc_start_localization.sh
```

Prepare a map directory containing:

```text
lanelet2_map.osm
pointcloud_map.pcd
pointcloud_map_metadata.yaml
map_projector_info.yaml
```

Dry run, without sending effective drive commands. Autoware startup uses the
runtime visualization config `rc_autoware.rviz`; this is separate from the
mapping-workstation Foxglove bag viewer. The RViz config shows map, TF, C32
pointclouds, NDT pose, kinematic state, planning path, and route markers, and
provides RViz tools for `/initialpose` and `/goal_pose`. The RC profile waits
for a map-frame `/initialpose` from RViz/ROS before publishing the NDT seed, so
it will not inject a fake `0,0,0` initial pose by default:

```bash
MAP_PATH=/path/to/map ./scripts/rc/rc_start_autoware.sh
```

Low-speed vehicle run after calibration and bench validation:

```bash
MAP_PATH=/path/to/map SERIAL_PORT=/dev/<actual_chassis_tty> ENABLE_DRIVE_COMMANDS=true ./scripts/rc/rc_start_autoware.sh
./scripts/request_autonomous_mode.sh
```

## Default Safety Position

The default launch keeps `enable_drive_commands` false. The controller and planner
will run, but the safety gate publishes stop commands to the real vehicle command topic.
Switch it to true only after TF, steering, velocity, localization, serial direction, and
RC takeover behavior are verified.

The helper scripts source `install/local_setup.bash` through `scripts/ros_env.sh` so this
workspace does not accidentally run packages from `/home/corage/workspace/project/pilot-auto.x1`.

## RC Serial Vehicle Interface

The RC vehicle interface keeps the Autoware-facing topics unchanged and replaces only the
Hooke2 CAN transport:

```text
/control/command/control_cmd
  -> autoracer_vehicle_interface/rc_serial_interface
  -> 0x7B cmd1 cmd2 vx vy wz bcc 0x7D
  -> STM32 UART4
```

Default serial parameters:

```text
SERIAL_PORT=/dev/<actual_chassis_tty>
SERIAL_BAUDRATE=115200
IMU_SERIAL_PORT=/dev/ttyUSB0
IMU_BAUDRATE=115200
WHEEL_BASE_M=0.6
MAX_STEER_RAD=0.262
MAX_SPEED_MPS=3.0
LIDAR_DRIVER=lslidar_c32
LIDAR_HOST_IP=192.168.1.102
LIDAR_SENSOR_IP=192.168.1.200
```

On the vehicle host, configure the C32 Ethernet link with
`sudo -E ./scripts/rc/rc_configure_lidar.sh`. It assigns
`192.168.1.102/32` on `enP8p1s0` plus a host route to `192.168.1.200/32`, keeping
WiFi as the normal `192.168.1.0/24` route.

Runtime validation for the Autoware stack is ARM-side. The x86 development machine is
used for mapping tools, scripts, static checks, and map packaging; it is not required to
complete a full Autoware runtime build.

The stable operator-facing shell entry points are the `scripts/rc/rc_*.sh` scripts.
Ad-hoc diagnostics should stay as tests or documented commands unless they are part of
the normal run flow.

The STM32 firmware source uses the 11-byte ROS UART command frame and the
24-byte telemetry frame. Flash the matching firmware before trusting vehicle
feedback on `/vehicle/status/*`. Current RC constants use `0.600 m` wheelbase,
`0.230 m` wheel diameter, `0.262 rad` max steering, `3.0 m/s` command caps, and
a `0.05 m/s` low-speed deadband.
