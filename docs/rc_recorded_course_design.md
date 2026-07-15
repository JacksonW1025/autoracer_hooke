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

Core owns exactly one `CourseSample` type and one CSV reader/writer. Offline
CarMaker and RC generators may derive samples differently, but they must emit
the same runtime representation instead of copying parsers or adding a second
execution path.

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

Schema-3 manifests declare `runtime_contract: fixed_course_v1`. Their
`producer` section is opaque provenance for audit and regeneration; Core must
not branch on its method, evidence fields, topic names, or simulator. The
manifest records at minimum:

- schema and generator versions;
- map ID and `map` frame;
- source bag metadata hash and Super-LIO commit/config hashes;
- source and output trajectory metrics;
- the SHA256 of the corresponding generic `map_manifest.json`;
- processing thresholds and maximum smoothing displacement;
- speed-profile constraints;
- validation status and individual checks;
- the limitation that the route is a recorded driven line, not a surveyed
  drivable-area model.

The generic map manifest binds `map_projector_info.yaml`,
`pointcloud_map_metadata.yaml`, and every PCD tile by relative path, byte size,
point count, and SHA256. Runtime loading rejects a missing, changed, added, or
cross-paired tile instead of trusting only metadata.

The generic runtime loader validates integrity and platform-neutral invariants.
It neither requires CarMaker RoadEval evidence for an RC asset nor understands
Super-LIO evidence. Existing schema-2 CarMaker assets remain supported through
their strict compatibility validator, including RoadEval, release-manifest,
and tile-hash checks; this compatibility path does not define the schema-3
runtime model.

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

## Verification evidence (2026-07-15)

All four tracked courses load through the same Core entry point while
validating every file named by the corresponding external map manifest:

| Map | Samples | Length (m) | `course.csv` SHA256 | `map_manifest.json` SHA256 |
| --- | ---: | ---: | --- | --- |
| 101 | 1,052 | 210.008641470 | `47552aa7a89ce2d5e01fea8c2cc54a49ec9e094bd135379e4fa8c4dde4652113` | `7d4c2c214100ed82676a0c7b295bad57e829fcd30e95ced3c3bc7de5c9e278d4` |
| 102 | 1,052 | 209.957118235 | `15742492abf83bb52dc0888c050e4c18ddd600bb2b025faf65a890d4dd4e233c` | `691400a5f8440e5e33f69add2452ef368e6246e29ad18d66f9996b1d10b45552` |
| 103 | 876 | 174.686177038 | `eb39b6bb1b01ac5395f12b8a501783a7655428200a6e689afb808938ce45561d` | `2d94d6c0a788ea8346891333cc6a61527efc20cc0d947e53345aed0497f3a762` |
| 104 | 2,508 | 500.849978722 | `5f18ec201433cf180345a1b70a369536e38d398783f075bb5347490570b87737` | `830dbfdee85ae716b3e022b0ec85356a6f26a7803ca621283c2b7d347676ed81` |

The four CSV and validation files are byte-identical to their pre-contract
versions; only their manifests were repackaged. The map manifests cover 101,
97, 110, and 206 PCD tiles respectively and regenerate byte-for-byte.

The Core planning/bringup and RC course suites pass 42 tests, including the
schema-2 CarMaker compatibility regression and corrupt/unlisted PCD rejection.
Fresh external builds of `autoracer_planning` and `autoracer_bringup` pass.
An inspection-only ROS launch for map 101 published 471,532 downsampled PCD
points, 1,052 trajectory points in `map`, three course markers, and an exact
zero terminal speed. The captured RViz evidence is external at
`rc_mapping_data/course_validation/floor1_mapping_101_fixed_course_v1_rviz.png`.

The map directories and `map_manifest.json` files remain external deployment
assets and must be copied together to the Orin. Live RC LiDAR, localization,
control, chassis output, and vehicle-geometry validation remain untested in
this x86 inspection run; no Hooke/CarMaker runtime was started or changed.

## Explicit exclusions

- No hand-authored Lanelet2 input.
- No online route search or full Autoware behavior-planning stack.
- No IMU-only position integration.
- No CarMaker file, RoadEval result, or simulated truth for RC.
- No CAN interface in the RC path.
- No modification of Hooke2 sensing, vehicle, or race launch behavior.
