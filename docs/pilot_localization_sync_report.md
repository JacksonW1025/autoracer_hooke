# pilot CarMaker localization sync report

Date: 2026-07-07
Branch: `pilot-localization-sync-20260707`
Plan source: `/opt/ipg/carmaker/linux64-15.1/plan.md`
Pilot source (read-only): `/opt/ipg/carmaker/linux64-15.1/pilot-auto.x1`

## Phase 0 audit

Initial git status was recorded on source branch `baseline-cleanup-20260613` before creating this branch.
The worktree already contained localization experiment changes in `autoracer_bringup`,
`autoracer_localization`, and modified Autoware localization packages. These are treated as legacy
experiment state and are archived/replaced instead of reverted.

Package scan:

- Command: `source /opt/ros/humble/setup.bash && colcon list --base-paths src --names-only | sort`
- Duplicate package check: `colcon list --base-paths src --names-only | sort | uniq -d`
- Result: no duplicate package names.

Pilot diff audit for existing localization packages (`diff -qr -x __pycache__`):

| package | workspace path class | diff entries |
| --- | --- | ---: |
| `autoware_ndt_scan_matcher` | `core/localization` | 16 |
| `autoware_ekf_localizer` | `core/localization` | 3 |
| `autoware_pose_initializer` | `core/localization` | 7 |
| `autoware_gnss_poser` | `core/sensing` | 1 |
| `autoware_pose_instability_detector` | `universe/localization` | 1 |
| `autoware_gyro_odometer` | `core/localization` | 0 |
| `autoware_localization_util` | `core/localization` | 0 |
| `autoware_vehicle_velocity_converter` | `core/sensing` | 0 |
| `autoware_pointcloud_preprocessor` | `universe/sensing` | 0 |
| `autoware_map_loader` | `core/map` | 0 |
| `autoware_map_projection_loader` | `core/map` | 0 |
| `autoware_localization_error_monitor` | `universe/localization` | 0 |

Missing pilot packages confirmed present in pilot and absent in workspace:

- `core/localization/autoware_stop_filter`
- `core/localization/autoware_twist2accel`
- `core/common/autoware_signal_processing`
- `universe/system/autoware_default_adapi_helpers/autoware_automatic_pose_initializer`
- `core/api/autoware_adapi_specs`
- `universe/common/autoware_component_interface_utils`
- `launcher/tier4_universe_launch/tier4_localization_launch`

Legacy launch files identified for archival:

- `carmaker_autoware_localization.launch.py`
- `carmaker_stage_a.launch.py`
- `carmaker_stage_b.launch.py`
- `carmaker_stage_b_ndt.launch.py`
- `carmaker_stage_c0_ndt.launch.py`
- `carmaker_stage_c0_ndt_no_rtk.launch.py`
- `carmaker_stage_c0_ndt_startup_only.launch.py`
- `carmaker_stage_c0_pure_lidar.launch.py`

Legacy bringup tests identified for archival:

- `test_carmaker_autoware_localization_launch_contract.py`
- `test_carmaker_stage_b_launch_contract.py`
- `test_carmaker_stage_b_ndt_launch_contract.py`
- `test_carmaker_stage_c0_experiment_launch_contract.py`
- `test_carmaker_stage_c0_ndt_launch_contract.py`

Legacy experiment nodes confirmed by `rg`:

- `ground_truth_initialpose_once`
- `startup_pose_initializer_once`
- `diagnostic_pose_reinitializer`
- `scan_accumulator`
- `ndt_axis_seed_fuser`
- `runtime_candidate_selector`
- `pure_lidar_fixed_lag_tracker`
- `gnss5m_weak_pose_bridge`
- `route_progress_initial_pose_provider`
- `independent_candidate_observer`
- `independent_ndt_candidate_observer`

Plan-specific launch constraints carried into implementation:

- Wrapper must not consume `/carmaker/ground_truth/pose`.
- `autoware_map_loader` must run as `/map/pointcloud_map_loader`.
- Both partial and differential pointcloud map services must be enabled.
- `/fixposition/rawimu` must be relayed to `/sensing/imu/imu_data`.
- `use_sim_time` is distributed through launch-level `SetParameter`.
- NDT must use pilot parameters with `ndt.regularization.enable=false`.

## Phase commits

To be filled after each phase commit.

## Phase 1 archive

Archive root:

- `legacy_localization_experiments/20260707_pilot_sync_backup/`
- `legacy_localization_experiments/COLCON_IGNORE`

Archived launch files:

- `src/autoracer_bringup/launch/carmaker_autoware_localization.launch.py`
- `src/autoracer_bringup/launch/carmaker_stage_a.launch.py`
- `src/autoracer_bringup/launch/carmaker_stage_b.launch.py`
- `src/autoracer_bringup/launch/carmaker_stage_b_ndt.launch.py`
- `src/autoracer_bringup/launch/carmaker_stage_c0_ndt.launch.py`
- `src/autoracer_bringup/launch/carmaker_stage_c0_ndt_no_rtk.launch.py`
- `src/autoracer_bringup/launch/carmaker_stage_c0_ndt_startup_only.launch.py`
- `src/autoracer_bringup/launch/carmaker_stage_c0_pure_lidar.launch.py`

Archived coupled bringup tests:

- `src/autoracer_bringup/test/test_carmaker_autoware_localization_launch_contract.py`
- `src/autoracer_bringup/test/test_carmaker_stage_b_launch_contract.py`
- `src/autoracer_bringup/test/test_carmaker_stage_b_ndt_launch_contract.py`
- `src/autoracer_bringup/test/test_carmaker_stage_c0_experiment_launch_contract.py`
- `src/autoracer_bringup/test/test_carmaker_stage_c0_ndt_launch_contract.py`

Archived modified algorithm packages before pilot replacement:

- `src/external/autoware/core/localization/autoware_ndt_scan_matcher`
- `src/external/autoware/core/localization/autoware_ekf_localizer`
- `src/external/autoware/core/localization/autoware_pose_initializer`
- `src/external/autoware/core/sensing/autoware_gnss_poser`
- `src/external/autoware/universe/localization/autoware_pose_instability_detector`

Phase 1 checks:

- `find src/autoracer_bringup/launch -maxdepth 1 -type f -name 'carmaker*.launch.py'` produced no output.
- `find src/autoracer_bringup/test -maxdepth 1 -type f -name 'test_carmaker*.py'` produced no output.
- `colcon list --base-paths src --names-only | sort | uniq -d` produced no output.

## Verification summary

To be filled after Phase 6/7.
