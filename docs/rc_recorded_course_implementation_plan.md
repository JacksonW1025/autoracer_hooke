# RC Recorded Course Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reproduce the four mapping-time Super-LIO trajectories and turn each complete single-direction recording into a validated RC fixed-course asset that the platform-neutral Core can load.

**Architecture:** Offline RC tooling replays each source bag through the pinned mapping workspace, records `/lio/odom`, and converts that odometry into the existing fixed-course CSV geometry and speed fields. Core receives one platform-neutral `fixed_course_v1` integrity contract and never dispatches on producer type. CarMaker schema-2 assets retain their strict compatibility validator, while every schema-3 asset binds the complete map through a generic map manifest.

**Tech Stack:** ROS 2 Humble, rosbag2/`rosbag2_py`, Super-LIO, Python 3 standard library, pytest, Bash, Autoware planning messages.

---

## File structure

- `tools/mapping/inspect_bag_topics.sh`: select and validate recorded LiDAR/IMU topics.
- `tools/mapping/run_super_lio_course_replay.sh`: isolated replay orchestration that records `/lio/odom` without replacing maps.
- `src/platform/rc/tools/recorded_course.py`: pure odometry cleanup, resampling, bounded smoothing, curvature, speed profile, CSV and manifest generation.
- `src/platform/rc/tools/build_recorded_course.py`: rosbag2 reader and command-line entry point.
- `src/platform/rc/tools/recorded_course_sources.json`: immutable mapping of RC map IDs to external bags, maps, configurations, and processing limits.
- `src/platform/rc/test/test_recorded_course.py`: pure conversion and validation regression tests.
- `src/platform/rc/test/test_recorded_course_workflow.py`: replay/source/CLI contract tests.
- `src/core/autoracer_planning/autoracer_planning/course_asset.py`: platform-neutral course loader and map integrity validation.
- `src/core/autoracer_planning/autoracer_planning/map_manifest.py`: generic metadata/projector/all-PCD-tile integrity contract.
- `src/core/autoracer_planning/autoracer_planning/fixed_course.py`: retain CarMaker production code and delegate common runtime loading.
- `src/core/autoracer_planning/test/test_course_asset.py`: RC and CarMaker runtime contract tests.
- `courses/rc/floor1_mapping_{101,102,103,104}/`: generated small CSV/manifest assets.
- `docs/rc_recorded_course_design.md`: final implementation evidence and limitations update.

### Task 1: Restore the reproducible mapping replay boundary

**Files:**
- Create: `tools/mapping/inspect_bag_topics.sh`
- Create: `tools/mapping/run_super_lio_course_replay.sh`
- Create: `src/platform/rc/test/test_recorded_course_workflow.py`

- [ ] **Step 1: Write failing workflow contract tests**

Add tests that require the inspection script to select `/sensing/lidar/concatenated/pointcloud` and `/imu/data` for the four legacy bags, require non-empty messages, and require the replay script to use a new output directory, source the existing mapping workspace, run Super-LIO with saved configuration, record only `/lio/odom`, use process groups, and fail if the output bag has no odometry.

```python
def test_replay_records_super_lio_odometry_without_mutating_source_maps():
    text = (ROOT / "tools/mapping/run_super_lio_course_replay.sh").read_text()
    assert 'ros2 bag record -o "${ODOM_BAG}" /lio/odom' in text
    assert '[[ ! -e "${REPLAY_DIR}" ]]' in text
    assert "map.pcd" not in text
    assert 'ros2 bag info "${ODOM_BAG}"' in text
```

- [ ] **Step 2: Run the tests and verify the expected missing-file failure**

Run:

```bash
python3 -m pytest src/platform/rc/test/test_recorded_course_workflow.py -q
```

Expected: FAIL because both mapping scripts do not exist.

- [ ] **Step 3: Implement the minimal scripts**

Implement topic inspection using `ros2 bag info`. Implement replay with `set -euo pipefail`, explicit external workspace/data roots, saved config copy and hashes, isolated ROS domain, tracked process groups, graceful SIGINT/TERM escalation, `ros2 bag play --clock`, `/lio/odom` recording, and a non-empty odometry postcondition. Never remove or overwrite an existing map/run/replay directory.

- [ ] **Step 4: Run shell syntax and workflow tests**

Run:

```bash
bash -n tools/mapping/inspect_bag_topics.sh
bash -n tools/mapping/run_super_lio_course_replay.sh
python3 -m pytest src/platform/rc/test/test_recorded_course_workflow.py -q
```

Expected: all commands exit zero.

- [ ] **Step 5: Commit the replay boundary using the Lore protocol**

Commit only the two scripts and their test, recording that live replay has not yet run.

### Task 2: Implement deterministic recorded-trajectory conversion

**Files:**
- Create: `src/platform/rc/tools/__init__.py`
- Create: `src/platform/rc/tools/recorded_course.py`
- Create: `src/platform/rc/test/test_recorded_course.py`

- [ ] **Step 1: Write failing tests for cleanup and resampling**

Cover non-finite rejection, invalid quaternion rejection, stationary prefix/suffix removal, duplicate collapse, timestamp reversal failure, isolated jump failure, complete single-direction preservation, fixed-distance resampling, Z interpolation, yaw tangent consistency, and strict cumulative distance.

```python
def test_complete_single_direction_recording_is_resampled_without_route_selection():
    poses = straight_poses(stationary_prefix=3, moving_count=11, stationary_suffix=2)
    samples, report = build_recorded_course(poses, config(interval_m=0.5))
    assert samples[0].s == 0.0
    assert samples[-1].x == pytest.approx(5.0)
    assert all(b.s > a.s for a, b in zip(samples, samples[1:]))
    assert report["internal_segments_removed"] == 0
```

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/platform/rc python3 -m pytest src/platform/rc/test/test_recorded_course.py -q
```

Expected: FAIL because `recorded_course` is missing.

- [ ] **Step 3: Implement cleanup and geometry generation**

Use immutable dataclasses for source poses, config, and course samples. Use planar arc length, linear interpolation, a bounded symmetric moving average, forward tangent yaw, signed three-point curvature, and explicit validation exceptions. Report every rejected point and the maximum smoothing displacement.

- [ ] **Step 4: Write failing speed-profile tests**

Require `0 <= velocity <= 0.5`, exact terminal stop, curvature limit, `+0.4 m/s^2` acceleration bound, `-0.8 m/s^2` deceleration bound, and finite `target_acceleration` values.

- [ ] **Step 5: Implement the minimal forward/backward speed envelope**

Compute curvature speed ceilings, apply forward acceleration and backward braking passes, set a conservative departure speed, force the final speed to zero, and derive acceleration from `v^2` over distance.

- [ ] **Step 6: Run conversion tests and commit**

Run the focused test twice to prove determinism, then commit the pure converter and tests using Lore trailers.

### Task 3: Add rosbag2 input and source descriptors

**Files:**
- Create: `src/platform/rc/tools/build_recorded_course.py`
- Create: `src/platform/rc/tools/recorded_course_sources.json`
- Modify: `src/platform/rc/test/test_recorded_course_workflow.py`

- [ ] **Step 1: Write failing CLI/source tests**

Require exactly four IDs, external paths relative to the repository parent, SHA256 capture for bag metadata/config/map metadata, `/lio/odom` type validation, atomic output creation, refusal to replace output, and no Lanelet path or dependency.

- [ ] **Step 2: Verify RED**

Run the workflow test and confirm failure because the CLI/source file is absent.

- [ ] **Step 3: Implement rosbag2 odometry reader and atomic writer**

Read `nav_msgs/msg/Odometry` with `rosbag2_py.SequentialReader`; use header timestamps and poses; require `world` or the configured source frame; call the pure converter; write `course.csv`, `manifest.json`, and `validation.json` into a temporary sibling directory; rename only after every check passes.

- [ ] **Step 4: Populate exact 101–104 source descriptors**

Point each map ID at its corresponding raw bag, saved run config, Autoware map directory, replay directory, and conservative processing/speed limits. Store no machine-specific secret and no generated large data in Git.

- [ ] **Step 5: Run CLI/source tests and commit**

Run pytest, `python3 ... --help`, JSON parsing, and `git diff --check`, then commit.

### Task 4: Separate the common runtime asset contract from CarMaker evidence

**Files:**
- Create: `src/core/autoracer_planning/autoracer_planning/course_asset.py`
- Create: `src/core/autoracer_planning/test/test_course_asset.py`
- Modify: `src/core/autoracer_planning/autoracer_planning/fixed_course.py`
- Modify: `src/core/autoracer_planning/autoracer_planning/fixed_course_publisher.py`

- [ ] **Step 1: Lock existing CarMaker behavior with regression tests**

Require the tracked CarMaker asset to retain schema 2, RoadEval corridor evidence, all existing asset/map hashes, and rejection when its release manifest or tile hashes change.

- [ ] **Step 2: Write failing RC contract tests**

Create a minimal temporary RC map and recorded-course manifest. Require generic loading to accept its CSV/map hashes without RoadEval, while rejecting a bad frame, mismatched map ID, modified CSV, missing metadata, failed validation, non-finite samples, or missing terminal stop.

- [ ] **Step 3: Verify RED**

Run both old and new course tests. Existing tests must pass; new RC tests must fail because the common loader does not exist.

- [ ] **Step 4: Extract the common loader with the smallest Core diff**

Move the single CSV schema/reader/writer, generic integrity checks, and trajectory sample type into `course_asset.py`. Schema-3 assets declare only `runtime_contract: fixed_course_v1`; producer method and evidence are opaque provenance and must not control runtime behavior. Bind the course to `map_manifest.json`, which hashes the projector, point-cloud metadata, and every PCD tile. Retain the schema-2 CarMaker compatibility path and its strict RoadEval/release checks unchanged in meaning. Do not import RC code or add a platform conditional to launch files.

- [ ] **Step 5: Run Core planning tests**

Run:

```bash
PYTHONPATH=src/core/autoracer_planning python3 -m pytest \
  src/core/autoracer_planning/test/test_course_asset.py \
  src/core/autoracer_planning/test/test_fixed_course.py \
  src/core/autoracer_planning/test/test_local_trajectory_planner.py -q
```

Expected: all pass and the tracked CarMaker asset still loads.

- [ ] **Step 6: Commit the platform-neutral contract**

Use Lore trailers to record why RoadEval is production-method evidence rather than a universal runtime dependency.

### Task 5: Replay all four source bags

**Files:**
- External outputs only: `../rc_mapping_data/course_replays/floor1_mapping_{101,102,103,104}/`

- [ ] **Step 1: Verify prerequisites without changing data**

Check the four source bags with `ros2 bag info`, the saved per-run configs, the existing mapping workspace install, Super-LIO commit/status, free disk space, and absence of ROS business nodes in the selected domain.

- [ ] **Step 2: Replay 101 and validate odometry**

Run the replay script at `PLAYBACK_RATE=1.0`. Require a complete exit, non-empty `/lio/odom`, finite poses, matching time range, and no Super-LIO NaN/synchronization error.

- [ ] **Step 3: Replay 102, 103, and 104 sequentially**

Use separate immutable output directories. A failed replay stops downstream generation for that map but does not modify prior results.

- [ ] **Step 4: Record replay evidence**

Capture bag info, selected topics, command/config/Super-LIO hashes, message counts, start/end timestamps, log error scan, and disk paths in each external replay directory.

### Task 6: Generate and verify four course assets

**Files:**
- Create: `courses/rc/floor1_mapping_101/course.csv`
- Create: `courses/rc/floor1_mapping_101/manifest.json`
- Repeat for `102`, `103`, and `104`
- External: overlay and validation artifacts under `../rc_mapping_data/course_validation/`

- [ ] **Step 1: Generate 101 and inspect all validation metrics**

Generate the generic map manifest, then run the course CLI from its odometry replay. Require no internal segment deletion, bounded smoothing, PCD-bound overlap, strict spacing, finite yaw/curvature, bounded speed/acceleration, terminal stop, and a course-to-map-manifest hash binding.

- [ ] **Step 2: Repeat for 102–104**

Use the exact corresponding bag/map/config hashes. Never reuse an asset across map IDs.

- [ ] **Step 3: Produce deterministic overlays**

Create external top-down PCD/course overlays with fixed axes and map ID labels. Verify the complete recorded line lies on the mapped corridor; if not, fail that asset and investigate replay/config mismatch rather than editing the CSV manually.

- [ ] **Step 4: Prove deterministic regeneration**

Generate each asset into a second temporary directory and compare SHA256 for `course.csv` and normalized manifest content.

- [ ] **Step 5: Commit only small validated assets**

Confirm no bag, replay database, monolithic PCD, cache, or overlay enters Git. Commit four course directories with Lore evidence and honest remaining hardware limits.

### Task 7: Runtime publication verification with drive disabled

**Files:**
- Modify: `docs/rc_recorded_course_design.md`

- [ ] **Step 1: Build the affected product packages**

Build Core planning and RC platform packages in a clean external product workspace. Do not source legacy or mapping underlays into the product build.

- [ ] **Step 2: Load every map/course pair through Core**

For each ID, start the fixed-course publisher with the matching external map and tracked course, with vehicle drive commands disabled. Verify `/planning/global_trajectory`, `header.frame_id == map`, point count, first/last coordinates, and terminal zero speed.

- [ ] **Step 3: Run the full non-hardware test suite**

Run Core planning, RC adapter, RC bringup, dependency, boundary, launch syntax, lint/static checks, and `git diff --check`. Search RC source and generated assets for CAN/SocketCAN and Lanelet runtime dependencies; expect none.

- [ ] **Step 4: Update evidence and commit**

Record exact replay IDs, asset hashes, validation metrics, commands, test counts, and the remaining untested live LiDAR/chassis/vehicle-geometry items in the design document. Commit using the Lore protocol.

- [ ] **Step 5: Independent completion review**

Run code review, verification-before-completion, and adversarial QA over corrupted manifests, wrong map pairing, truncated odometry, stationary recordings, reversal, and replay interruption. Fix any findings before reporting completion.
