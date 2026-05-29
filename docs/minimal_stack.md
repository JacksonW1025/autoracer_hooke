# Minimal Stack

## Runtime Order

1. Vehicle interface and CAN status.
2. Static TF for `base_link`, `lidar_top`, `gnss_base_link`, and IMU frames.
3. LiDAR and Fixposition drivers, plus the vehicle-speed bridge for Fixposition.
4. PCD and Lanelet2 map loading.
5. Localization, using Fixposition as initial/regularization pose and NDT as map pose.
6. Lanelet route planner and trajectory generator.
7. Pure pursuit controller.
8. Safety command gate.

## Topics

Input:

```text
/sensing/lidar/concatenated/pointcloud
/fixposition/fix
/fixposition/autoware_orientation
/sensing/gnss/pose_with_covariance
/vehicle/status/velocity_status
/vehicle/status/steering_status
/goal_pose
```

Internal:

```text
/fixposition/speed
/localization/pose_with_covariance
/planning/mission_path
/planning/trajectory
/autoracer/control/raw_control_cmd
```

Vehicle output:

```text
/control/command/control_cmd
/control/command/gear_cmd
/control/command/hazard_lights_cmd
```

## Excluded From MVP

The closed-track MVP excludes object recognition, prediction, behavior planning,
traffic lights, lane changes, complex scenario planning, and full Autoware AD API.

## Current Bench Step

Before map localization and route following, validate the live input layer only:

```bash
./scripts/verify_sensing_feedback.sh
```

This uses `bench_verification.launch.py`, so no map, planner, controller, or drive
command gate is launched.
