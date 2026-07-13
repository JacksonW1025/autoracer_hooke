# CarMaker 传感器建模实施契约：Pandar40P + Fixposition VRTK2

本文是给 AI 一次性执行的工程契约，不是背景长文。目标是把 CarMaker 中的 LiDAR、Fixposition、TF、topic 合同一次性建模到能支持定位调通的状态。

原则：**简单、真实、可验证**。不要为了“更像硬件”实现复杂私有协议；也不要用错误 frame / 宽松 gate 制造 PASS。

---

## 1. 一句话目标

把 CarMaker 仿真输出改到和真车自动驾驶栈看到的数据一致：

```text
Pandar40P 点云：
  frame_id = lidar_top
  点坐标也在 lidar_top frame
  10Hz, 40 lines, 0.3~200m, Pandar40P 非均匀线束

Fixposition VRTK2：
  /fixposition/* topic、frame、covariance、status 与定位链路契约一致
  dual GNSS RTK fixed + baseline passing 状态自洽

定位 ownership：
  CarMaker truth 只做 diagnostic
  /localization/pose_with_covariance 只能由 NDT / EKF 发布
```

---

## 2. 必须读取的事实源

### 2.1 真车外参

```text
autoracer_hooke/src/core/autoracer_description/urdf/hooke2_sensor_mounts.urdf.xacro
autoracer_hooke/src/core/autoracer_description/config/hooke2_sensor_extrinsics.yaml
```

当前真车外参：

```text
base_link -> lidar_top_base_link
  xyz = 1.90, 0.0, 1.337
  rpy = 0.0, 0.0, +1.57079632679

lidar_top_base_link -> lidar_top
  xyz = 0.0, 0.0, 0.047
  rpy = 0.0, 0.0, 0.0

base_link -> lidar_top
  xyz = 1.90, 0.0, 1.384
  rpy = 0.0, 0.0, +1.57079632679

base_link -> gnss_base_link
  xyz = 1.90, 0.0, 1.037
  rpy = 0.0, 0.0, -1.57079632679

base_link -> imu_link
  xyz = 1.90, 0.0, 1.037
  rpy = 0.0, 0.0, -1.57079632679
```

### 2.2 Pandar40P

```text
autoracer_hooke/docs/Pandar40P_产品手册.pdf
autoracer_hooke/src/core/autoracer_bringup/config/hooke2/lidar_top.param.yaml
```

必须按这些关键参数建模：

```text
model: Pandar40P
frame_id: lidar_top
rotation_speed: 600 rpm = 10Hz
horizontal FoV: 360 deg
horizontal resolution: 0.2 deg @ 10Hz
vertical channels: 40
vertical FoV: -25 deg ~ +15 deg
range: 0.3 m ~ 200 m
single return point rate: 720k pts/s
dual return point rate: 1.44M pts/s
range accuracy: about 2cm for 1~200m
```

40 线垂直设计角使用手册附录 I：

```text
15.00, 11.00, 8.00, 5.00, 3.00, 2.00,
1.67, 1.33, 1.00, 0.67, 0.33, 0.00,
-0.33, -0.67, -1.00, -1.33, -1.67, -2.00,
-2.33, -2.67, -3.00, -3.33, -3.67, -4.00,
-4.33, -4.67, -5.00, -5.33, -5.67, -6.00,
-7.00, -8.00, -9.00, -10.00, -11.00, -12.00,
-13.00, -14.00, -19.00, -25.00
```

每线水平角 offset 使用手册设计值，主要为：

```text
-1.042 deg
+3.125 deg
-5.208 deg
```

如果仓库里有该台实车的 Pandar calibration csv，优先使用 calibration csv；没有就用手册设计值。

当前仓库已有 Nebula 默认 Pandar40P calibration：

```text
autoracer_hooke/vendor_ws/src/vendor/nebula/src/nebula_hesai/nebula_hesai_decoders/calibration/Pandar40P.csv
```

该 CSV 可作为 C0 beam 角度来源，但仍必须验证 CarMaker beam azimuth 正方向与 ROS `lidar_top` 坐标轴、Nebula 解码约定一致。

### 2.3 Fixposition VRTK2

```text
autoracer_hooke/docs/VRTK2_Datasheet_v1.1 1.pdf
autoracer_hooke/docs/VRTK2_integration_manual_v2.2.9(1).pdf
```

必须保留这些接口语义：

```text
VRTK2 = dual GNSS + IMU + camera + optional wheel speed
sensor frame origin 在 Fixposition logo 附近
GNSS antenna 外参需要相对 VRTK2 sensor frame 测量
nominal RTK position accuracy: about 1cm + 1ppm
velocity accuracy: about 0.1 m/s
attitude accuracy: < 0.4 deg
solution latency: about 50 ms
IMU output: about 200Hz
```

常见设备 frame 语义：

```text
FP_VRTK   # VRTK2 body / sensor frame
FP_POI    # configured output point
FP_ENU0   # local ENU origin
GNSS1     # antenna 1
GNSS2     # antenna 2
```

---

## 3. 当前已知错误

这些是必须修掉的硬错误：

```text
SimProject_TianmenRace/Data/Vehicle/AutoracerCollection_UrbanSensorCar
  Sensor.2.pos = 4.3 0.0 0.40      # 错，和真车 URDF 不一致
  Sensor.2.rot = 0.0 0.0 0.0       # 错，和真车 yaw +90deg 不一致；CarMaker 这里单位是 deg

SimProject_TianmenRace/src/User.cpp
  kLidarPosBase = {6.73, 0.0, 1.00} # 当前实际值，来自 Body.pos + Sensor.2.pos，坐标语义未被证明，不能作为正确基线
  kLidarMaxRangeM = 300.0          # 不应作为 Pandar40P 物理 range

SimProject_TianmenRace/src/ROS2Bridge.cpp
  PointCloud2.header.frame_id = "base_link" # 不符合真车 Pandar driver
  /fixposition/fix frame_id = "gnss_link"   # URDF/YAML 没有这个 frame
```

当前 `Data/Sensor/LidarRSI_Pandar40` 只是 Pandar-like 均匀 40 线，不是最终 Pandar40P beam model。

当前测试里如果存在下面假设，也必须同步删除或改写：

```text
Beams.Type == Regular
kLidarPosBase == Body.pos + Sensor.2.pos
```

这些测试固化的是当前近似/错误实现，不是最终坐标合同。

---

## 4. 最终 frame 合同

推荐最小 frame 树：

```text
map
  -> base_link
       -> lidar_top_base_link
            -> lidar_top
       -> gnss_base_link
            -> gnss1_link
            -> gnss2_link
       -> imu_link
```

frame 语义：

| 语义 | 项目 frame | 设备语义 | 要求 |
| --- | --- | --- | --- |
| 车辆基准 | `base_link` | Autoware base | 定位最终输出对象 |
| Pandar40P 点云 | `lidar_top` | LiDAR sensor frame | 点云坐标和 header 都必须在此 frame |
| VRTK2 body / output reference | `gnss_base_link` | `FP_VRTK` 或 `FP_POI` | C0 可复用现有 frame，但要记录语义 |
| GNSS1 antenna | `gnss1_link` | `GNSS1` | 没有实测时可 temporary assumption |
| GNSS2 antenna | `gnss2_link` | `GNSS2` | 没有实测时可 temporary assumption |
| IMU | `imu_link` | IMU / VRTK body | 当前和 `gnss_base_link` 同位同姿态 |

硬规则：

```text
message.header.frame_id 必须表示消息数据实际所在的 frame。
不能把 base_link 坐标点云只改 header 成 lidar_top。
不能继续使用未定义的 gnss_link。
```

CarMaker / ROS 坐标硬规则：

```text
ROS base_link:
  Autoware 约定为车辆后轴中心，x forward, y left, z up。

CarMaker Sensor.<no>.pos:
  transceiver frame 相对挂载 vehicle frame 的位置，单位 m。

CarMaker Sensor.<no>.rot:
  transceiver frame 相对挂载 vehicle frame 的姿态，单位 deg，rotation order = zyx。
  注意：这和 C++/ROS 里的 rpy rad 不是同一个单位。

LidarRSI ScanPoint:
  Origin 是 FrSensor 中的 ray origin。
  LengthOF 是 ray path length。
  如果 Sensor.2 已经正确安装成 lidar_top，Bridge 默认应发布 FrSensor/lidar_top 中的 hit point。
```

禁止在未证明 `Vehicle.PoI` / `Fr1A` / `base_link` 关系前，把 `Body.pos + Sensor.2.pos` 当作 `base_link -> lidar_top`。

Pandar azimuth 符号验证规则：

```text
Pandar40P 手册定义水平角度顶视图顺时针为正。
CarMaker Beam file 使用 azimuth/elevation 描述 beam direction，但不能默认其正方向与 Pandar UDP/CSV 完全一致。

生成 LidarRSI_Pandar40 时：
  可以读取 Nebula Pandar40P.csv 的 Elevation/Azimuth；
  但必须用静态目标或单墙验证左右方向、前向 0deg、yaw +90deg 是否正确；
  如果出现左右镜像，再对 horizontal offset 做符号转换。
```

这里是 P0 验证项，不是预设结论：不要在未验证前声称 Pandar azimuth 符号一定正确或一定错误。

---

## 5. 一次性实施要求

AI 应一次性完成下面所有工作。可以按顺序探索和验证，但不要只做其中一小段就停。

### 5.1 固化 CarMaker frame 换算

先用代码和运行时证据确认：

```text
CarMaker Fr1A <-> ROS base_link
Vehicle.PoI <-> ROS base_link
CarMaker LidarRSI ScanPoint / Origin / LengthOF 的坐标语义
```

不要直接假设 `Sensor.2.pos = base_link -> lidar_top`。如果 `Vehicle.PoI` 不是 `base_link`，先修 truth、sensor、Fixposition 发布点的一致性。

必须显式输出最终采用的换算链：

```text
Fr1A -> base_link
base_link -> lidar_top
Fr1A -> lidar_top
Vehicle.PoI -> base_link
base_link -> gnss_base_link
Fr0/map -> base_link
Fr0/map -> gnss_base_link
```

当前代码里的 `Body.pos + Sensor.2.pos` 只能作为待验证历史实现，不允许作为默认答案。

如果运行时证据确认 CarMaker Fr1A 轴向与 ROS `base_link` 平行，且 ROS `base_link` 是后轴中心，则 LiDAR 的一个待验证初始候选为：

```text
Fr1A -> base_link ≈ rear axle center in Fr1A
Fr1A -> lidar_top = Fr1A -> base_link + base_link -> lidar_top
Sensor.2.rot ≈ 0 0 90 deg
```

这个候选必须通过静态目标或简单墙面验证 yaw、lever arm 和高度方向后才能落地。

### 5.2 更新 URDF / YAML

更新：

```text
autoracer_hooke/src/core/autoracer_description/urdf/hooke2_sensor_mounts.urdf.xacro
autoracer_hooke/src/core/autoracer_description/config/hooke2_sensor_extrinsics.yaml
```

要求：

```text
保留 base_link -> lidar_top
保留 base_link -> gnss_base_link
保留 base_link -> imu_link
新增 gnss_base_link -> gnss1_link
新增 gnss_base_link -> gnss2_link
```

如果没有 GNSS antenna 实测外参，可以先填 temporary assumption，但必须在注释和 summary 里写清楚，不能声称最终实车等价。

### 5.3 更新 CarMaker LiDAR sensor

更新：

```text
SimProject_TianmenRace/Data/Sensor/LidarRSI_Pandar40
SimProject_TianmenRace/Data/Vehicle/AutoracerCollection_UrbanSensorCar
```

要求：

```text
LidarRSI_Pandar40:
  1800 horizontal samples
  40 vertical channels
  10Hz
  0.3~200m
  Pandar40P 非均匀垂直角
  Pandar40P 通道水平角 offset
  使用 Beams.Type = UserDefined 或 CarMaker 等价能力表达非均匀 beam；不能继续用 Regular uniform grid 声称 Pandar40P 已完成
  使用 Pandar40P.csv 或手册角度时，必须验证 horizontal offset 符号与 CarMaker/ROS 坐标一致

Vehicle sensor instance:
  Sensor.2.pos / Sensor.2.rot 与真车 base_link -> lidar_top 经 CarMaker frame 换算后一致
  Sensor.2.rot 写入 CarMaker vehicle file 时使用 deg，不是 rad
  Sensor.2.pos 不得无证明地叠加 Body.pos
  Sensor.Param.2.Range = 0.3 200.0
  SensorCluster.0.CycleTime = 100
```

C0 允许简化：

```text
single-return approximation 可接受
firing time / motion distortion 可暂不实现
```

但必须在 `runtime_summary.json` 里记录：

```text
lidar_return_mode_model
lidar_motion_distortion_model
lidar_firing_time_model
```

### 5.4 更新 CarMaker Bridge 点云输出

更新：

```text
SimProject_TianmenRace/src/User.cpp
SimProject_TianmenRace/src/ROS2Bridge.cpp
```

目标合同：

```text
/sensing/lidar/concatenated/pointcloud
  type: sensor_msgs/msg/PointCloud2
  header.frame_id: lidar_top
  points: lidar_top frame 坐标
  fields: x, y, z, intensity
  stamp: CarMaker sim time
  rate: about 10Hz
  range: 0.3~200m
```

默认实现规则：

```text
hit_sensor = ScanPoint.Origin + beam_direction(BeamID) * ScanPoint.LengthOF
publish points = hit_sensor
publish frame_id = lidar_top
```

不要在默认路径里调用 `SensorPointToBase()` 后再发布正式定位点云。`base_link` 预变换点云只允许作为 debug topic / debug mode。

如果保留 `base_link` 预变换输出，只能作为显式 debug mode，默认必须是：

```text
pointcloud_frame_mode = lidar_top_tf
```

### 5.5 更新 Fixposition topic 输出

CarMaker Bridge 在关闭真实 `fixposition_driver_ros2_exec` 时发布：

```text
/fixposition/fix                    sensor_msgs/msg/NavSatFix
/fixposition/autoware_orientation   autoware_sensing_msgs/msg/GnssInsOrientationStamped
/fixposition/rawimu                 sensor_msgs/msg/Imu
/fixposition/odometry_enu           nav_msgs/msg/Odometry
/fixposition/fpa/odomstatus         fixposition_driver_msgs/msg/FpaOdomstatus
```

可选：

```text
/fixposition/speed                  fixposition_driver_msgs/msg/Speed
```

`/fixposition/fix` 规则：

```text
NavSatFix 经纬高代表哪个物理点，header.frame_id 就写哪个 frame。

如果模拟 GNSS1 antenna:
  frame_id = gnss1_link

如果模拟 VRTK2 fused POI/body:
  frame_id = gnss_base_link
  语义 = FP_POI / FP_VRTK 的项目简化
```

C0 推荐：

```text
/fixposition/fix.header.frame_id = gnss_base_link
```

因此 C0 不能只把 `Vehicle.PoI_Pos` 转成经纬高后填 `frame_id=gnss_base_link`。必须先计算 `gnss_base_link` 对应物理点在 map/Fr0 中的位置：

```text
p_map_gnss = p_map_base + R_map_base * t_base_gnss
NavSatFix(gnss_base_link) = LLA(p_map_gnss)
```

如果最终选择发布 GNSS1 antenna：

```text
p_map_gnss1 = p_map_base + R_map_base * t_base_gnss1
frame_id = gnss1_link
```

Autoware `gnss_poser` 会根据 `NavSatFix.header.frame_id` 查该 frame 到 `base_link` 的静态 TF。如果 header 和经纬高实际物理点不一致，会引入约 1~2m lever-arm 级定位偏差。

同时保留 `gnss1_link` / `gnss2_link` 用于 dual GNSS status、baseline 和后续原始 GNSS 扩展。

`/fixposition/autoware_orientation`：

```text
输出 VRTK2 / GNSS-INS 姿态。
orientation 必须和 /fixposition/fix.header.frame_id 代表的物理 frame 一致。

如果 /fixposition/fix.header.frame_id = gnss_base_link:
  orientation = map -> gnss_base_link
  q_map_gnss = q_map_base * q_base_gnss
  不能直接把 Vehicle.Roll/Pitch/Yaw 当成 gnss_base_link 姿态发布

如果 /fixposition/fix.header.frame_id = gnss1_link:
  orientation 仍必须与 gnss1_link 物理点/语义自洽；不确定时不要使用 gnss1_link 作为 fused INS 输出 frame。

attitude std 不应比 0.4deg 明显更乐观
```

`/fixposition/rawimu`：

```text
header.frame_id = imu_link
stamp = CarMaker sim time
angular_velocity / linear_acceleration 转到 imu_link
orientation 如果发布，也必须是 map -> imu_link 或明确标记为 unavailable；不能把 base_link 姿态直接塞进 imu_link
明确 acceleration 是否包含 gravity
如果不是 200Hz，runtime_summary 必须记录实际 IMU 频率
```

`/fixposition/odometry_enu`：

```text
可以继续使用 FP_ENU0 / FP_POI，但必须解释和 map / gnss_base_link 的关系；
也可以在仿真模式使用 header.frame_id=map, child_frame_id=gnss_base_link。
无论哪种，child_frame_id 必须和 pose 实际代表的点一致。
velocity covariance 不应比 0.1m/s 明显更乐观，除非标记 idealized truth mode。
```

`/fixposition/fpa/odomstatus` nominal C0：

```text
init_status = GLOBAL_INIT
fusion_imu = USED
fusion_gnss1 = USED
fusion_gnss2 = USED
fusion_corr = USED
fusion_cam1 = NOT_USED
imu_status = FINE_CONVERGED
imu_noise = LOW_NOISE
gnss1_status = RTK_FIXED
gnss2_status = RTK_FIXED
baseline_status = PASSING
corr_status = GOOD_CORR
```

wheel speed 必须自洽：

```text
如果不建模 wheel speed:
  fusion_ws = NOT_USED
  不发布 /fixposition/speed

如果建模 wheel speed:
  fusion_ws = USED
  /fixposition/speed 来源于 CarMaker wheel / vehicle speed
```

---

## 6. 验证要求

实现后必须用短 route 验证，不要直接跑 10.8km route。

必须输出或更新：

```text
runtime_summary.json:
  pointcloud_frame_id
  pointcloud_frame_mode
  pointcloud_rate_hz
  pointcloud_avg_points_per_frame
  lidar_model
  lidar_beams_file
  lidar_return_mode_model
  lidar_motion_distortion_model
  lidar_firing_time_model
  lidar_azimuth_sign_validation
  carmaker_sensor_pose
  carmaker_to_ros_frame_chain
  ros_lidar_static_tf
  fixposition_topics
  fixposition_fix_frame_semantics
  fixposition_fix_physical_point
  fixposition_orientation_frame_semantics
  fixposition_status_counts
  imu_publish_rate_hz
  gnss_poser_output_count
  ndt_output_count
```

验收 gate：

```text
LiDAR:
  PointCloud2.header.frame_id == lidar_top
  PointCloud2 点坐标实际在 lidar_top frame
  rate 9.5~10.5Hz
  range 0.3~200m
  base_link -> lidar_top static TF 与 URDF/YAML 一致
  points/frame 与 Pandar40P single/dual approximation 一致
  Pandar40P beam 不能是 Regular uniform vertical grid，除非 summary 明确标记为 temporary approximation 且不得作为最终 PASS
  Pandar horizontal offset 符号通过静态目标/单墙验证，不能只靠肉眼或假设

Fixposition:
  /fixposition/fix present
  /fixposition/autoware_orientation present
  /fixposition/rawimu present
  /fixposition/odometry_enu present
  /fixposition/fpa/odomstatus present
  /fixposition/* 只有 CarMaker Bridge 一个发布源
  /fixposition/fix.frame_id 与 NavSatFix 实际代表物理点一致
  frame_id=gnss_base_link 时，LLA 必须来自 map/Fr0 中的 gnss_base_link 点，不是直接复用 Vehicle.PoI_Pos
  /fixposition/autoware_orientation 必须表示 map -> fix.header.frame_id 对应 frame，不得直接复用 base_link 姿态
  /fixposition/rawimu 的 orientation / angular_velocity / linear_acceleration 必须在 imu_link 语义下自洽
  GNSS1/GNSS2 RTK fixed + baseline passing 自洽
  fusion_ws 与 wheel speed 来源自洽
  gnss_poser 能输出 /sensing/gnss/pose_with_covariance

Map / localization:
  PCD 由当前 CarMaker LidarRSI PointCloud2 生成
  PCD / scan / ground truth / Lanelet2 / projector 共用同一 map 坐标
  PCD density 与真实录制 PCD 同量级
  scan-map overlap 分布可解释
  /localization/pose_with_covariance 只允许 NDT / EKF 发布
  CarMaker truth 只能发布 diagnostic topic

Tests:
  不允许测试继续断言 Beams.Type == Regular 作为最终 Pandar40P
  不允许测试继续断言 kLidarPosBase == Body.pos + Sensor.2.pos
  必须增加或更新坐标一致性检查：PointCloud2 header、点坐标 frame、base_link->lidar_top TF、NavSatFix frame/物理点一致
```

---

## 7. 禁止的捷径

不要做这些事：

```text
继续用 Sensor.2.pos = 4.3 0 0.40 做正式基线
继续用 kLidarPosBase = Body.pos + Sensor.2.pos 作为未验证基线
继续用 PointCloud2.frame_id = base_link 冒充 Pandar 输出
把 base_link 坐标点云只改 header 成 lidar_top
继续使用未定义的 gnss_link
继续用 Regular uniform vertical grid 声称已完成 Pandar40P 建模
未验证 CarMaker/Pandar/Nebula azimuth 正方向就声称 horizontal offset 已正确
把 /fixposition/fix 无条件写成 gnss1_link，但实际发布 fused POI/body 经纬高
把 /fixposition/fix 写成 gnss_base_link，但实际经纬高仍然来自 Vehicle.PoI/base_link
把 /fixposition/autoware_orientation 写成 gnss_base_link 语义，但实际仍发布 base_link 姿态
在 frame 统一前继续调 NDT 参数
放宽 validation gate 制造 PASS
C0 首轮引入 GNSS outage / camera VIO / complex noise model
模拟 Fixposition 私有 TCP 协议或硬件 Web UI
```

---

## 8. 给执行 AI 的指令

请在当前代码空间一次性完成 CarMaker Pandar40P + Fixposition VRTK2 建模修复。

工作方式：

1. 先读取本文列出的事实源和相关 CarMaker / ROS 代码。
2. 用运行时或代码证据确认 CarMaker Fr1A、Vehicle.PoI、ROS `base_link`、LidarRSI `ScanPoint` 坐标语义。
3. 然后一次性修改 URDF/YAML、CarMaker sensor、LidarRSI beams、User.cpp、ROS2Bridge.cpp、runtime summary 和必要验证脚本。
4. 默认目标是 `lidar_top_tf` 点云模式和 VRTK2 nominal RTK fixed 模式。
5. 用同一张 CarMaker road 的短 route 完成快速验证。
6. 输出最终状态：改了什么、为什么、验证命令、验证结果、仍然存在的假设。

如果某个格式或 CarMaker 能力不支持完整 Pandar40P 物理建模，使用最接近的简单实现，但必须在代码注释和 `runtime_summary.json` 中记录 approximation。不要为了追求复杂真实度引入不可控模块。
