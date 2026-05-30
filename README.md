# Autoracer Hooke

Minimal ROS 2 workspace for closed-track autonomous driving on the Hooke2 chassis.

This workspace intentionally does not launch the full Autoware stack. It uses selected
Autoware packages as libraries and keeps the vehicle task small:

```text
Pandar LiDAR + Fixposition + Hooke2 vehicle feedback
  -> PCD/Lanelet2 map
  -> LiDAR/GNSS localization
  -> Lanelet centerline route
  -> Pure pursuit + longitudinal PID
  -> safety command gate
  -> /control/command/control_cmd
  -> hooke2_interface
  -> CAN
```

## Repository Layout

```text
autoracer.repos            Dependency manifest for selected external packages.
defaults.env               Runtime defaults used by scripts/run_track.sh.
docs/                      Bringup and calibration notes.
maps/                      Local map directory placeholder.
scripts/                   Import, build, run, and smoke-test helpers.
src/autoracer_bringup      Top-level launches and Hooke2 configuration.
src/autoracer_description  Minimal Hooke2 frames and static TF launch.
src/autoracer_localization Localization helper nodes.
src/autoracer_sensing      Minimal sensor/vehicle feedback adapters.
src/autoracer_planning     Lanelet route and trajectory node.
src/autoracer_control      Pure pursuit controller.
src/autoracer_safety       Final command gate before the vehicle interface.
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

When developing beside the old repository, dependencies can be copied locally instead
of fetched:

```bash
IMPORT_FROM_PILOT=true ./scripts/import_dependencies.sh
```

Bench validation for the current hardware stage:

```bash
IMPORT_FROM_PILOT=true ./scripts/import_dependencies.sh
./scripts/build_bench.sh
./scripts/verify_sensing_feedback.sh
```

Lightweight LiDAR visualization:

```bash
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

Dry run, without sending effective drive commands:

```bash
MAP_PATH=/path/to/map ./scripts/run_track.sh
```

Low-speed vehicle run after calibration and bench validation:

```bash
MAP_PATH=/path/to/map ENABLE_DRIVE_COMMANDS=true MAX_SPEED_MPS=1.5 ./scripts/run_track.sh
./scripts/request_autonomous_mode.sh
```

## Default Safety Position

The default launch keeps `enable_drive_commands` false. The controller and planner
will run, but the safety gate publishes stop commands to the real vehicle command topic.
Switch it to true only after TF, steering, velocity, localization, and CAN direction are
verified.

The helper scripts source `install/local_setup.bash` through `scripts/ros_env.sh` so this
workspace does not accidentally run packages from `/home/corage/workspace/project/pilot-auto.x1`.
