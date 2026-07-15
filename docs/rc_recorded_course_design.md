# RC Recorded Course Asset Design

## Objective

Generate one fixed course asset for each RC point-cloud map by replaying the
original, single-direction mapping recording and preserving the vehicle's
Super-LIO trajectory. The hand-authored Lanelet2 files are not an input,
validation oracle, or runtime dependency for these course assets.

The four source recordings are external data under
`rc_mapping_data/bags/raw/floor1_mapping_{101,102,103,104}`. Each recording
contains the LiDAR point cloud and both filtered and raw IMU topics. It does
not contain a dynamic TF, pose, path, or odometry topic, so the trajectory must
be reproduced with the same Super-LIO configuration that generated the PCD.

## Boundary

The production pipeline belongs to the RC platform and offline tooling. Core
continues to consume a platform-neutral fixed-course asset and must not import
Super-LIO, rosbag2, RC topic names, or an `if rc` branch.

CarMaker and RC use different evidence to produce the same runtime concept:

- CarMaker may use simulator truth, RoadEval, and road files.
- RC uses the LiDAR-inertial trajectory produced while building its PCD.
- Neither production method is a runtime planner dependency.

Hooke2 and its validated CarMaker launch composition are outside the change
scope.

## Source-of-truth data flow

For each recording:

1. Replay the original PointCloud2 and IMU topics with recorded timestamps.
2. Run the pinned Super-LIO build using the saved per-run configuration and
   calibrated LiDAR-to-IMU extrinsics.
3. Record `/lio/odom`, whose `world` coordinates are the coordinates used to
   build that run's `map.pcd`.
4. Treat the complete moving portion as one forward course. The recordings are
   single-direction; route selection, Lanelet matching, and branch selection
   are prohibited.
5. Convert `world` numerically to the runtime `map` frame without applying a
   coordinate transform. The frame rename records the established identity
   contract between the saved PCD and its generating trajectory.
6. Produce a deterministic fixed-course asset and validation report.

## Trajectory processing

Processing may remove measurement defects but must not redesign the driven
line:

- reject non-finite poses and invalid quaternions;
- remove only the stationary prefix and suffix using a documented movement
  threshold with hysteresis;
- collapse consecutive points separated by less than the minimum spacing;
- reject isolated pose jumps that violate a documented distance/speed gate;
- resample by planar arc length at a fixed interval;
- apply bounded smoothing whose maximum lateral displacement is reported and
  enforced;
- recompute yaw from the forward path tangent;
- compute signed planar curvature from the resampled geometry;
- preserve Z by interpolation along arc length;
- set cumulative `s` in metres, strictly increasing from zero.

The tool must fail rather than silently repair a reversal, self-inconsistent
timestamp sequence, large discontinuity, or trajectory with fewer than two
usable points.

## Speed profile

The initial RC profile is deliberately conservative:

- maximum speed: `0.5 m/s`;
- maximum acceleration: `0.4 m/s^2`;
- maximum deceleration: `-0.8 m/s^2`;
- speed is additionally bounded by curvature using an explicit lateral
  acceleration limit;
- the first point uses a safe departure speed and the final point is exactly
  zero;
- forward and backward passes enforce acceleration and stopping feasibility.

The course geometry is independent of these limits. Changing a verified
vehicle limit regenerates only the speed fields, not the driven line.

## Runtime asset contract

`course.csv` uses the existing columns and SI units:

```text
s,x,y,z,yaw,curvature,left_offset,right_offset,target_velocity,target_acceleration
```

The recorded trajectory does not establish surveyed road boundaries.
Consequently, `left_offset` and `right_offset` must not claim RoadEval or
Lanelet-derived free space. They are conservative RC corridor values derived
from an explicit configured assumption and identified as such in the manifest.
They are validation metadata in the current planner; they do not cause online
lateral path optimization.

The manifest records at minimum:

- schema and generator versions;
- map ID and `map` frame;
- source bag metadata hash and Super-LIO commit/config hashes;
- source and output trajectory metrics;
- PCD/map metadata hashes needed to prevent cross-map pairing;
- processing thresholds and maximum smoothing displacement;
- speed-profile constraints;
- validation status and individual checks;
- the limitation that the route is a recorded driven line, not a surveyed
  drivable-area model.

The generic runtime loader validates integrity and platform-neutral invariants.
It must not require CarMaker RoadEval evidence for an RC asset. Existing
CarMaker assets retain their stronger CarMaker-specific evidence.

## Verification gates

An asset is accepted only when all gates pass:

1. Replay completes without Super-LIO errors and produces finite, monotonic
   odometry over the moving interval.
2. CSV schema, hashes, SI units, frame, row count, and manifest are consistent.
3. `s` is strictly increasing; spacing, yaw tangent error, curvature, speed,
   acceleration, and terminal stop satisfy configured bounds.
4. No reversal or unapproved internal segment deletion is detected.
5. Course bounds overlap the corresponding PCD bounds and the course/map IDs
   and hashes match.
6. A reproducible overlay artifact shows the course over its corresponding PCD
   for 101, 102, 103, and 104.
7. Core loads each asset and publishes `/planning/global_trajectory` with
   `header.frame_id == map` while vehicle drive commands remain disabled.

Live vehicle motion is not part of asset generation. Any later hardware test
keeps drive output disabled until sensing, localization, trajectory publication,
vehicle geometry, and emergency-stop conditions are independently verified.

## Repository placement

The implementation will follow existing package boundaries:

- reusable offline conversion and validation logic stays with course tooling;
- RC replay configuration and source descriptors stay under `src/platform/rc`;
- generated small course assets may be tracked under an RC-specific course
  directory;
- source bags, monolithic PCDs, replay recordings, and generated visualization
  caches remain external data and are never committed;
- Core changes are limited to making the runtime asset contract genuinely
  platform-neutral, protected by regression tests for the existing CarMaker
  asset.

## Explicit exclusions

- No hand-authored Lanelet2 input.
- No online route search or full Autoware behavior-planning stack.
- No IMU-only position integration.
- No CarMaker file, RoadEval result, or simulated truth for RC.
- No CAN interface in the RC path.
- No modification of Hooke2 sensing, vehicle, or race launch behavior.
