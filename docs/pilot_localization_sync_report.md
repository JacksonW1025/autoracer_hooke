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

- Phase 0: `50236b5` - audit pilot localization sync baseline.
- Phase 1: `f8df6d5` - archive legacy localization experiments.
- Phase 2: `4ab9086` - sync pilot localization packages.
- Phase 3: `fb625cd` - copy pilot localization parameters.
- Phase 4: `52e8a15` - add pilot CarMaker localization wrapper.
- Phase 5: `625b329` - declare pilot localization dependencies.
- Phase 6: `3b4bc08` - validate pilot localization build and launch.
- Phase 7: final commit in this phase; see `git log --oneline -1` after commit.

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

## Phase 2 pilot package sync

Replaced by byte-for-byte pilot copies:

- `src/external/autoware/core/localization/autoware_ndt_scan_matcher`
- `src/external/autoware/core/localization/autoware_ekf_localizer`
- `src/external/autoware/core/localization/autoware_pose_initializer`
- `src/external/autoware/core/sensing/autoware_gnss_poser`
- `src/external/autoware/universe/localization/autoware_pose_instability_detector`

Added missing pilot packages:

- `src/external/autoware/core/localization/autoware_stop_filter`
- `src/external/autoware/core/localization/autoware_twist2accel`
- `src/external/autoware/core/common/autoware_signal_processing`
- `src/external/autoware/universe/system/autoware_default_adapi_helpers/autoware_automatic_pose_initializer`
- `src/external/autoware/core/api/autoware_adapi_specs`
- `src/external/autoware/universe/common/autoware_component_interface_utils`
- `src/external/autoware/launcher/tier4_universe_launch/tier4_localization_launch`

Phase 2 checks:

- `colcon list --base-paths src --names-only | sort | uniq -d` produced no output.
- `colcon list --base-paths src --names-only | grep -E "stop_filter|twist2accel|tier4_localization_launch|automatic_pose_initializer"` found all four required names.
- `diff -r -x __pycache__` against pilot returned empty for all five replaced packages.

## Phase 3 pilot localization parameters

Copied from `pilot-auto.x1/src/autoware/launcher/autoware_launch/config/localization` to
`src/autoracer_bringup/config/pilot_compatible/localization`:

- `ekf_localizer.param.yaml`
- `stop_filter.param.yaml`
- `twist2accel.param.yaml`
- `pose_initializer.param.yaml`
- `localization_error_monitor.param.yaml`
- `pose_instability_detector.param.yaml`
- `eagleye_config.param.yaml`
- `ar_tag_based_localizer.param.yaml`
- `ndt_scan_matcher/ndt_scan_matcher.param.yaml`
- `ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range.param.yaml`
- `ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml`
- `ndt_scan_matcher/pointcloud_preprocessor/random_downsample_filter.param.yaml`
- `lidar_marker_localizer/lidar_marker_localizer.param.yaml`
- `lidar_marker_localizer/pointcloud_preprocessor/crop_box_filter_measurement_range.param.yaml`
- `lidar_marker_localizer/pointcloud_preprocessor/ring_filter.param.yaml`

Phase 3 checks:

- `cmp -s` matched every copied file against the pilot source file.
- `ndt_scan_matcher.param.yaml` contains `ndt.regularization.enable: false`.

## Phase 4 wrapper launch

Added:

- `src/autoracer_bringup/launch/pilot_carmaker_localization.launch.py`

Wrapper responsibilities implemented:

- Launch-level `SetParameter(name="use_sim_time", value=True)`.
- Existing `autoracer_description/launch/static_tf.launch.py`.
- `autoware_map_projection_loader`.
- `/map/pointcloud_map_loader` via `autoware_map_loader/autoware_pointcloud_map_loader`.
- `rclcpp_components/component_container_mt` as `/pointcloud_container`.
- `autoware_vehicle_velocity_converter` from `/vehicle/status/velocity_status`.
- `autoware_gnss_poser` from `/fixposition/fix` and `/fixposition/autoware_orientation`.
- `topic_tools/relay` from `/fixposition/rawimu` to `/sensing/imu/imu_data`.
- `tier4_localization_launch/launch/localization.launch.xml` with `pose_source=ndt`,
  `twist_source=gyro_odom`, `system_run_mode=logging_simulation`, all required parameter paths,
  and explicit `twist2accel_param_path`.

Phase 4 checks:

- `/usr/bin/python3 -m py_compile src/autoracer_bringup/launch/pilot_carmaker_localization.launch.py` passed.
- `rg` against the wrapper for the Phase 7 forbidden node list and `/carmaker/ground_truth/pose` produced no output.
- `rg` confirmed `twist2accel_param_path`, `eagleye_param_path`,
  `ar_tag_based_localizer_param_path`, `lidar_marker_localizer/*`, IMU relay, map services,
  and `/pointcloud_container` are present.

## Phase 5 package dependencies

Updated `src/autoracer_bringup/package.xml` with runtime dependencies:

- `autoware_adapi_specs`
- `autoware_automatic_pose_initializer`
- `autoware_component_interface_utils`
- `autoware_stop_filter`
- `autoware_twist2accel`
- `tier4_localization_launch`
- `topic_tools`

Existing `rclcpp_components` dependency was preserved.

Phase 5 checks:

- `/usr/bin/python3 -c "import xml.etree.ElementTree as ET; ET.parse('src/autoracer_bringup/package.xml')"` passed.
- `rg` confirmed all added dependency names in `package.xml`.

## Phase 6 build and static validation

Package scan:

- `colcon list --base-paths src --names-only | sort | uniq -d`: no output.
- `colcon list --base-paths src --names-only | grep -E "stop_filter|twist2accel|tier4_localization_launch|automatic_pose_initializer"`:
  - `autoware_automatic_pose_initializer`
  - `autoware_stop_filter`
  - `autoware_twist2accel`
  - `tier4_localization_launch`

Build:

- Command: `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to autoracer_bringup`
- First attempt failed because the pre-existing `build/autoware_ndt_scan_matcher` cache still referenced
  archived experiment targets (`runtime_multistart_selection`, `ndt_validation`,
  `independent_candidate_observer_node`). Source was already pilot-clean; the fix was to remove only
  `build/autoware_ndt_scan_matcher` and `install/autoware_ndt_scan_matcher`.
- Retried the same build command successfully.
- Final summary: `104 packages finished [1min 24s]`.
- Final rebuild after Phase 7 wrapper compatibility fixes: `104 packages finished [10.0s]`.
- Non-fatal warnings observed:
  - colcon underlay override warning for selected Autoware packages already present in `/opt/ros/humble`;
  - ament scoped header install deprecation warnings during packages rebuilt on the first attempt;
  - `autoware_crop_box_filter`: pcap-related IO features disabled.

Installed package prefix check:

- `autoracer_bringup`: `install/autoracer_bringup`
- `autoware_ndt_scan_matcher`: `install/autoware_ndt_scan_matcher`
- `tier4_localization_launch`: `install/tier4_localization_launch`
- `autoware_stop_filter`: `install/autoware_stop_filter`
- `autoware_twist2accel`: `install/autoware_twist2accel`
- `autoware_automatic_pose_initializer`: `install/autoware_automatic_pose_initializer`

Pilot diff self-proof:

- `diff -r -x __pycache__` returned empty for:
  - `autoware_ndt_scan_matcher`
  - `autoware_ekf_localizer`
  - `autoware_pose_initializer`
  - `autoware_gnss_poser`
  - `autoware_pose_instability_detector`

Static launch expansion:

- Command: `source install/setup.bash && ros2 launch autoracer_bringup pilot_carmaker_localization.launch.py --print`
- Result: exit code 0, no missing launch argument errors.
- Humble `--print` prints Python `GroupAction` / `IncludeLaunchDescription` objects without recursively expanding nested XML content; therefore key wiring was checked directly against the wrapper plus copied pilot XML launch files.

Key wiring checks:

- NDT input: `/localization/util/downsample/pointcloud`
  (`tier4_localization_launch/launch/pose_twist_estimator/ndt_scan_matcher.launch.xml`).
- NDT initial pose: `/localization/pose_twist_fusion_filter/biased_pose_with_covariance`.
- NDT map client: `/map/get_differential_pointcloud_map`.
- EKF pose input: `/localization/pose_estimator/pose_with_covariance`.
- EKF twist input: `/localization/twist_estimator/twist_with_covariance`.
- stop_filter input: `/localization/pose_twist_fusion_filter/kinematic_state`.
- stop_filter output: `/localization/kinematic_state`.
- twist2accel uses explicit `twist2accel_param_path`.
- gyro_odometer vehicle twist input: `/sensing/vehicle_velocity_converter/twist_with_covariance`.
- gyro_odometer IMU input default: `/sensing/imu/imu_data`; wrapper relays `/fixposition/rawimu` to it.
- Forbidden node list and `/carmaker/ground_truth/pose` were absent from the wrapper and copied `tier4_localization_launch` files.

## Phase 7 runtime readiness

The full CarMaker closed-loop run was not executed in this session. Reasons:

- No CarMaker process was already running.
- Starting the full `SimProject_TianmenRace/run_stage_c0_ndt_realtime.sh` path would start
  CarMaker/MovieEGL and use desktop/GPU state.
- The script's defaults are wired for the archived stage-C0 experiment chain, so using it with the
  new wrapper requires explicit node/topic overrides.

Wrapper compatibility fixes made during Phase 7:

- Added `localization_map_path` as an alias for `map_path` because the reference script passes
  `localization_map_path:=...`.
- Added declared `use_sim_time` argument and kept launch-level `SetParameter`.
- Scoped the `autoware_vehicle_velocity_converter` and `autoware_gnss_poser` XML includes so generic
  launch args (`config_file`, `param_file`) cannot leak into `tier4_localization_launch`. This fixes
  a launch-only smoke issue where `gyro_odometer` inherited the vehicle velocity converter config.

Launch-only smoke test, without CarMaker/GPU:

- Environment sanitized to avoid Anaconda `libstdc++` overriding ROS Humble runtime libraries.
- Map used: `src/autoracer_bringup` wrapper argument
  `localization_map_path:=/opt/ipg/carmaker/linux64-15.1/autoracer_hooke/maps/carmaker_builtin_urban`
  because this directory contains `map_projector_info.yaml`.
- Result: wrapper started and was interrupted intentionally (`SIGTERM`, status 143) after inspection.
- Nodes observed:
  - `/map/pointcloud_map_loader`
  - `/pointcloud_container`
  - `/vehicle_velocity_converter`
  - `/gnss_poser`
  - `/fixposition_rawimu_to_sensing_imu_relay`
  - `/localization/pose_estimator/ndt_scan_matcher`
  - `/localization/twist_estimator/gyro_odometer`
  - `/localization/util/pose_initializer`
  - `/localization/util/default_adapi/helpers/autoware_automatic_pose_initializer`
  - `/localization/pose_twist_fusion_filter/ekf_localizer`
  - `/localization/pose_twist_fusion_filter/stop_filter`
  - `/localization/pose_twist_fusion_filter/twist2accel`
  - `/localization/pose_twist_fusion_filter/pose_instability_detector`
  - `/localization/localization_error_monitor`
- Topic/service checks:
  - `/localization/kinematic_state`: publisher `/localization/pose_twist_fusion_filter/stop_filter`.
  - `/localization/pose_twist_fusion_filter/kinematic_state`: publisher EKF, subscriber stop_filter.
  - `/localization/util/downsample/pointcloud`: publisher random downsample filter, subscriber NDT.
  - `/localization/pose_estimator/pose_with_covariance`: publisher NDT, subscriber EKF.
  - `/localization/twist_estimator/twist_with_covariance`: publisher gyro_odometer, subscribers EKF/twist2accel/pose_instability_detector.
  - `/sensing/gnss/pose_with_covariance`: publisher gnss_poser, subscriber pose_initializer.
  - `/sensing/imu/imu_data`: gyro_odometer subscriber present; publisher absent in launch-only smoke because no `/fixposition/rawimu` source was running for `topic_tools` relay discovery.
  - Services present: `/localization/initialize`, `/api/localization/initialize`,
    `/localization/pose_estimator/ndt_align_srv`, `/localization/pose_estimator/trigger_node`,
    `/localization/pose_twist_fusion_filter/trigger_node`,
    `/map/get_differential_pointcloud_map`, `/map/get_partial_pointcloud_map`.

Full CarMaker closed-loop commands for manual execution:

```bash
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
export PYTHONNOUSERSITE=1
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu"
source /opt/ros/humble/setup.bash
cd /opt/ipg/carmaker/linux64-15.1/autoracer_hooke
source install/setup.bash
```

Standalone wrapper smoke with CarMaker already publishing bridge topics:

```bash
ros2 launch autoracer_bringup pilot_carmaker_localization.launch.py \
  localization_map_path:=/opt/ipg/carmaker/linux64-15.1/autoracer_hooke/maps/carmaker_builtin_urban \
  use_sim_time:=true
```

Reference-script style run, with old stage-C0 checks overridden:

```bash
cd /opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace
LAUNCH_FILE=pilot_carmaker_localization.launch.py \
MAP_PATH=/opt/ipg/carmaker/linux64-15.1/autoracer_hooke/maps/carmaker_builtin_urban \
ENABLE_ROS_CLI_INTROSPECTION=1 \
RUN_COMPARE=0 \
REQUIRE_COMPARISON=0 \
BASE_REQUIRED_NODES="/map/pointcloud_map_loader /pointcloud_container /vehicle_velocity_converter /gnss_poser /fixposition_rawimu_to_sensing_imu_relay /localization/pose_estimator/ndt_scan_matcher /localization/twist_estimator/gyro_odometer /localization/util/pose_initializer /localization/pose_twist_fusion_filter/ekf_localizer /localization/pose_twist_fusion_filter/stop_filter" \
REQUIRED_SMOKE_TOPICS="/sensing/lidar/concatenated/pointcloud /vehicle/status/velocity_status /fixposition/rawimu /fixposition/fix /sensing/vehicle_velocity_converter/twist_with_covariance /sensing/imu/imu_data /sensing/gnss/pose_with_covariance /localization/util/downsample/pointcloud /localization/pose_estimator/pose_with_covariance /localization/twist_estimator/twist_with_covariance /localization/pose_twist_fusion_filter/kinematic_state /localization/kinematic_state" \
BASE_RECORD_TOPICS="/clock /sensing/lidar/concatenated/pointcloud /vehicle/status/velocity_status /fixposition/rawimu /fixposition/fix /fixposition/autoware_orientation /sensing/vehicle_velocity_converter/twist_with_covariance /sensing/imu/imu_data /sensing/gnss/pose_with_covariance /localization/util/downsample/pointcloud /localization/pose_estimator/pose_with_covariance /localization/twist_estimator/twist_with_covariance /localization/pose_twist_fusion_filter/kinematic_state /localization/kinematic_state" \
./run_stage_c0_ndt_realtime.sh
```

Post-launch checks:

```bash
ros2 topic info /localization/kinematic_state -v
ros2 topic info /localization/pose_twist_fusion_filter/kinematic_state -v
ros2 topic info /localization/util/downsample/pointcloud -v
ros2 topic info /localization/pose_estimator/pose_with_covariance -v
ros2 topic info /localization/twist_estimator/twist_with_covariance -v
ros2 topic info /sensing/gnss/pose_with_covariance -v
ros2 topic info /sensing/imu/imu_data -v
ros2 service list | rg "initialize|ndt_align|trigger_node|get_differential|get_partial"
```

## Decisions and risks

- `autoracer_localization` remains in `src` as required, but the new wrapper does not depend on it.
- The wrapper keeps both `map_path` and `localization_map_path`; this is only a launch-script
  compatibility alias and does not affect algorithm code.
- `autoware_map_loader` in the copied source does not declare an `enable_differential_load`
  parameter; it constructs the differential map service unconditionally. The wrapper remaps and
  verifies `/map/get_differential_pointcloud_map`.
- Current checkout does not contain `maps/whale_map_20251107`; use `MAP_PATH`, `map_path`, or
  `localization_map_path` to point at a map directory containing `map_projector_info.yaml`,
  `lanelet2_map.osm`, and pointcloud map metadata.
- Full closed-loop remains pending on a CarMaker/GPU session with the correct map and Bridge topics.

## Verification summary

- Package scan: pass; no duplicate package names.
- Build: pass; final `colcon build --symlink-install --packages-up-to autoracer_bringup` succeeded.
- Algorithm package copy proof: pass; five replaced packages diff clean against pilot.
- Static launch `--print`: pass; no missing launch arguments.
- Launch-only smoke: pass for node/service/topic ownership that does not require active CarMaker data.
- Full CarMaker closed-loop: pending manual execution.

---

## 2026-07-07 独立复审(review)发现与修复记录

复审方式：逐项对照 implement prompt 验收标准 + 二进制级证据核查。

### 复审确认合格的项

- 8 个 Phase commit、工作区干净、归档目录含 `COLCON_IGNORE`；
- 5 个替换包与 7 个新增包对 pilot `diff -r` 全部为空；
- 15 个参数文件与 pilot 原版逐一 diff 为空（含必填的 eagleye/ar_tag/lidar_marker）；
- wrapper 传齐了全部必填 arg（含 `twist2accel_param_path`）；map_loader 命名
  `/map/pointcloud_map_loader` 与服务 remap 正确；vvc/gnss_poser/relay 接线与
  pilot 原版 launch 接口核对一致；旧 launch 家族与耦合契约测试已一并归档；
- `colcon list` 无重名包；`ros2 launch --print` exit 0。

### 缺陷 1（严重）：4 个替换包源码换了但二进制未重编

- 现象：`build/` 下 EKF(.so 5/31)、gnss_poser(5/30)、pose_initializer(6/7)、
  pose_instability_detector(6/7) 均早于同步日期；EKF 二进制内仍含魔改符号
  `output_time_offset`，gnss_poser 仍含 CarMaker latest-TF 注释字符串。
- 根因：`cp -a` 保留了 pilot 源文件的旧 mtime（4/8），make 按 mtime 判定产物较新而
  跳过编译。只有 NDT 因构建缓存引用了已删除源文件而报错、被 rm 后真正重建。
- 修复：`rm -rf build/<pkg> install/<pkg>` × 5 后强制重建，二进制全部为 7/7 新产物，
  魔改符号 strings 检查归零。

### 缺陷 2（中等）：重建未跟随工作区构建约定

- 现象：Phase 6 中 NDT 的重建用裸 `colcon build`，`CMAKE_BUILD_TYPE` 为空（无优化）；
  工作区基准（scripts/build_minimal.sh 及存量包）为 `Release` + `-DBUILD_TESTING=OFF`。
  NDT 无优化构建会直接损害实时定位性能。
- 附带发现：`BUILD_TESTING=ON` 时 gnss_poser 测试目标链接失败
  （GLIBCXX_3.4.30，anaconda 旧 libstdc++ 污染链接环境），主库不受影响；
  按工作区约定关闭测试构建即可规避。
- 修复：5 个同步包统一以 `-DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release` 重建，
  CMakeCache 确认全部 Release。

### 运行时前置条件提醒（非缺陷）

- wrapper 默认地图路径 `maps/whale_map_20251107` 在本机不存在（maps/ 下仅有
  TM99、carmaker_builtin_urban）。闭环运行前必须通过 `MAP_PATH` 环境变量或
  `localization_map_path` launch 参数指向真实地图目录，且该目录需包含
  `pointcloud_map_metadata.yaml`、`map_projector_info.yaml`、`lanelet2_map.osm` 与 PCD 分块。
