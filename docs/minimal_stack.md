# Minimal Stack

## Runtime Order

1. Vehicle interface and Autoware `/vehicle/status/*` feedback.
2. Static TF for `base_link` and the active sensor frames.
3. LiDAR driver, plus Fixposition only when that seed source is enabled.
4. PCD and Lanelet2 map loading.
5. Localization, using manual seed or Fixposition seed as NDT startup input.
6. Kinematic state publisher for Autoware controller state input.
7. Lanelet route planner and trajectory generator.
8. Pure pursuit controller.
9. Safety command gate.

## Topics

Input:

```text
/sensing/lidar/concatenated/pointcloud
/localization/fixposition/seed_pose
/vehicle/status/velocity_status
/vehicle/status/steering_status
/goal_pose
```

Internal:

```text
/localization/pose_with_covariance
/localization/kinematic_state
/planning/mission_path
/planning/trajectory
/autoracer/control/raw_control_cmd
```

Fixposition topics such as `/fixposition/fix`, `/fixposition/autoware_orientation`,
`/sensing/gnss/pose_with_covariance`, and `/fixposition/speed` are only present
when the Fixposition seed source is enabled. The RC profile disables that path and
uses RViz/ROS `/initialpose` as the semantic seed source for
`/localization/fixposition/seed_pose`.

The RC profile uses the Leishen C32 driver and publishes the point cloud directly
as `/sensing/lidar/concatenated/pointcloud`. Nav2 AMCL, slam_toolbox, and the old
Nav2 EKF are not part of this stack.

`/localization/kinematic_state` is `nav_msgs/Odometry` for the Autoware control
interface. It is a NDT-corrected kinematic state output: pose is corrected by
`/localization/pose_with_covariance`, while vehicle velocity and steering feedback
fill the twist and short-term prediction between NDT updates. It is not a full EKF.

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
