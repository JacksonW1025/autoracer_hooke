# Sensing And Feedback Topics

This repository keeps the closed-track input surface small.

## LiDAR

The Autoware-facing contract is one point cloud topic:

```text
/sensing/lidar/concatenated/pointcloud  sensor_msgs/msg/PointCloud2
```

Hooke uses Hesai Pandar through `nebula_hesai`. The default parameter file is
`src/autoracer_bringup/config/hooke2/lidar_top.param.yaml`; the historical live
configuration used Nebula's `Pandar40P` model and `lidar_top` frame.

The RC profile uses Leishen C32 through `lslidar_driver` with the legacy C32
network settings: `device_ip=192.168.1.200`, `msop_port=2368`,
`difop_port=2369`. It publishes directly to
`/sensing/lidar/concatenated/pointcloud` in frame `lidar_top`.

On the Raspberry Pi, configure the C32 Ethernet link with:

```bash
sudo -E ./scripts/configure_rc_lidar_link.sh
```

The helper uses `192.168.1.120/32` on `eth0` and a host route to
`192.168.1.200/32`. Do not configure `eth0` as `192.168.1.120/24` while WiFi is
also on `192.168.1.0/24`; that connected route can steal SSH return traffic. If
that temporary `/24` address was added by mistake, reboot the Raspberry Pi or
run `sudo ip addr del 192.168.1.120/24 dev eth0` locally on the Pi.

Do not use Nav2 `/scan` localization as an RC replacement for this point cloud
contract.

## Fixposition

The Fixposition ROS 2 driver is launched directly as `fixposition_driver_ros2_exec`.
Only the topics needed by localization are part of the minimal contract:

```text
/fixposition/fix                    sensor_msgs/msg/NavSatFix
/fixposition/autoware_orientation   autoware_sensing_msgs/msg/GnssInsOrientationStamped
/fixposition/rawimu                 sensor_msgs/msg/Imu
/fixposition/odometry_enu           nav_msgs/msg/Odometry
/fixposition/speed                  fixposition_driver_msgs/msg/Speed
```

`/fixposition/fix` and `/fixposition/autoware_orientation` feed
`autoware_gnss_poser`, which publishes `/sensing/gnss/pose_with_covariance` for NDT
initialization and regularization.

The driver may also advertise diagnostic FPA topics such as
`/fixposition/fpa/odomstatus`. They are useful for status inspection, but are not
required for the first localization contract above.

The RC profile disables Fixposition. RViz/ROS `/initialpose` is republished as
`/localization/fixposition/seed_pose`, preserving the NDT startup contract without
pretending the RC has a Fixposition device.

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
rate and Autoware vehicle status topics, then writes artifacts under
`log/bench_verify_*`. Fixposition checks run only when the Fixposition path is
enabled. Raw Hooke2 CAN checks run only when the Hooke2 vehicle interface is the
active profile.

For the lightest visual check, use:

```bash
./scripts/run_lidar_rviz.sh
```

This launches only static TF, the active LiDAR driver, and RViz with
`src/autoracer_bringup/rviz/lidar_pointcloud.rviz`.
