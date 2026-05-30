# Sensing And Feedback Topics

This repository keeps the closed-track input surface small.

## LiDAR

Hesai Pandar is launched through `nebula_hesai` and normalized to one Autoware
point cloud topic:

```text
/sensing/lidar/concatenated/pointcloud  sensor_msgs/msg/PointCloud2
```

The default parameter file is `src/autoracer_bringup/config/hooke2/lidar_top.param.yaml`.
The current Pandar 60 unit is decoded with Nebula's `Pandar40P` model and `lidar_top`
frame; this was the live configuration that produced point clouds. Override with
`LIDAR_SENSOR_MODEL` when the unit reports a different Nebula-supported model.

## Fixposition

The Fixposition ROS 2 driver is launched directly as `fixposition_driver_ros2_exec`.
Only the topics needed by localization are part of the minimal contract:

```text
/fixposition/fix                    sensor_msgs/msg/NavSatFix
/fixposition/autoware_orientation   autoware_sensing_msgs/msg/GnssInsOrientationStamped
/fixposition/rawimu                 sensor_msgs/msg/Imu
/fixposition/speed                  fixposition_driver_msgs/msg/Speed
```

`/fixposition/fix` and `/fixposition/autoware_orientation` feed
`autoware_gnss_poser`, which publishes `/sensing/gnss/pose_with_covariance` for NDT
initialization and regularization.

## Hooke2 Feedback

Do not consume raw `/hooke2/*` chassis reports outside tiny adapters and debugging
tools. `hooke2_interface` already converts CAN feedback to Autoware vehicle status:

```text
/vehicle/status/velocity_status        autoware_vehicle_msgs/msg/VelocityReport
/vehicle/status/steering_status        autoware_vehicle_msgs/msg/SteeringReport
/vehicle/status/steering_wheel_status  tier4_vehicle_msgs/msg/SteeringWheelStatusStamped
/vehicle/status/gear_status            autoware_vehicle_msgs/msg/GearReport
/vehicle/status/control_mode           autoware_vehicle_msgs/msg/ControlModeReport
```

The only adapter added here is `velocity_to_fixposition_speed`, which bridges
`/vehicle/status/velocity_status` to `/fixposition/speed` as a single `RC`
wheelspeed measurement in millimeters per second.

## Bench Verification

Use the standalone bench launch when the goal is only to prove the live data sources:

```bash
./scripts/verify_sensing_feedback.sh
```

It starts `autoracer_bringup bench_verification.launch.py`, checks LiDAR point cloud
rate, Fixposition samples, Hooke2 Autoware status topics, and raw CAN frames, then
writes artifacts under `log/bench_verify_*`.

For the lightest visual check, use:

```bash
./scripts/run_lidar_rviz.sh
```

This launches only static TF, the Hesai driver, and RViz with
`src/autoracer_bringup/rviz/lidar_pointcloud.rviz`.
