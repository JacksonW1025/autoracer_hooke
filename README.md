# Ackermann Autoware

Minimal ROS 2 workspace for closed-track autonomous driving on the RC Ackermann car.

This workspace intentionally does not launch the full Autoware stack. It uses selected
Autoware packages as libraries and keeps the vehicle task small:

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
src/autoracer_planning     Lanelet route and trajectory node.
src/autoracer_control      Pure pursuit controller.
src/autoracer_safety       Final command gate before the vehicle interface.
src/autoracer_vehicle_interface RC UART vehicle interface and Autoware status bridge.
src/hardware_drivers       Vendored SocketCAN driver used by Hooke2.
src/hooke2_vehicle         Vendored Hooke2 interface, launch, and description.
src/wd_msgs                Vendored Hooke2 chassis messages and byte helpers.
```

## First Bringup

```bash
cd /home/corage/workspace/project/autoracer-hooke
./scripts/import_dependencies.sh
./scripts/install_rosdeps.sh
./scripts/build_minimal.sh
source ./scripts/ros_env.sh
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

Bench validation for the current hardware stage:

```bash
IMPORT_FROM_PILOT=true ./scripts/import_dependencies.sh
./scripts/build_bench.sh
sudo -E ./scripts/configure_rc_lidar_link.sh
./scripts/verify_sensing_feedback.sh
```

RC run readiness checklist:

```text
docs/rc_run_readiness_checklist_zh.md
```

Lightweight LiDAR visualization:

```bash
sudo -E ./scripts/configure_rc_lidar_link.sh
./scripts/run_lidar_rviz.sh
```

Map-only RViz check:

```bash
./scripts/run_map_rviz.sh
```

Mock LiDAR NDT localization check:

```bash
./scripts/run_mock_lidar_record_scenario.sh
./scripts/run_mock_lidar_ndt_rviz.sh
```

Prepare a map directory containing:

```text
lanelet2_map.osm
pointcloud_map.pcd
pointcloud_map_metadata.yaml
map_projector_info.yaml
```

Dry run, without sending effective drive commands. The RC profile waits for a
map-frame `/initialpose` from RViz/ROS before publishing the NDT seed, so it will
not inject a fake `0,0,0` initial pose by default:

```bash
MAP_PATH=/path/to/map ./scripts/run_track.sh
```

Low-speed vehicle run after calibration and bench validation:

```bash
MAP_PATH=/path/to/map SERIAL_PORT=/dev/ttyUSB0 ENABLE_DRIVE_COMMANDS=true ./scripts/run_track.sh
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
SERIAL_PORT=/dev/ttyUSB0
SERIAL_BAUDRATE=115200
WHEEL_BASE_M=0.6
MAX_STEER_RAD=0.262
MAX_SPEED_MPS=3.0
LIDAR_DRIVER=lslidar_c32
LIDAR_HOST_IP=192.168.1.102
LIDAR_SENSOR_IP=192.168.1.200
```

On the Raspberry Pi, configure the C32 Ethernet link with
`sudo -E ./scripts/configure_rc_lidar_link.sh`. It intentionally uses
`192.168.1.102/32` plus a host route to `192.168.1.200`; do not configure
`eth0` as `192.168.1.102/24` when WiFi is also on `192.168.1.0/24`, because
that can break SSH return traffic. If that temporary `/24` address was added by
mistake, reboot the Raspberry Pi or run
`sudo ip addr del 192.168.1.102/24 dev eth0` locally on the Pi. The helper also
clears the earlier incorrect `192.168.1.120` test address if it is still present.

The STM32 firmware already accepts the 11-byte ROS UART command frame and publishes the
24-byte telemetry frame. Current RC constants use `0.600 m` wheelbase, `0.230 m`
wheel diameter, `0.262 rad` max steering, `3.0 m/s` forward/reverse caps, and a
`0.05 m/s` low-speed deadband.
