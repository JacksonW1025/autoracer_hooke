#!/usr/bin/env bash
set -euo pipefail

BAG_PATH="${1:?usage: inspect_bag_topics.sh <bag_path>}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

[[ -e "${BAG_PATH}" ]] || { echo "ERROR: bag does not exist: ${BAG_PATH}" >&2; exit 1; }

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

info="$(ros2 bag info "${BAG_PATH}")"
printf '%s\n' "${info}"

has_messages() {
  grep -Eq "Topic: $1 \\|.*Count: [1-9][0-9]*" <<< "${info}"
}

if [[ -z "${LIDAR_TOPIC:-}" ]]; then
  if has_messages /sensing/lidar/raw/pointcloud; then
    LIDAR_TOPIC=/sensing/lidar/raw/pointcloud
  elif has_messages /sensing/lidar/concatenated/pointcloud; then
    LIDAR_TOPIC=/sensing/lidar/concatenated/pointcloud
  else
    echo "ERROR: no non-empty RC pointcloud topic in bag" >&2
    exit 1
  fi
fi

if [[ -z "${IMU_TOPIC:-}" ]]; then
  if has_messages /sensing/imu/imu_data; then
    IMU_TOPIC=/sensing/imu/imu_data
  elif has_messages /imu/data; then
    IMU_TOPIC=/imu/data
  else
    echo "ERROR: no non-empty filtered IMU topic in bag" >&2
    exit 1
  fi
fi

for topic in "${LIDAR_TOPIC}" "${IMU_TOPIC}"; do
  has_messages "${topic}" || {
    echo "ERROR: required mapping topic is missing or empty: ${topic}" >&2
    exit 1
  }
done

echo "[course-replay] selected lidar topic: ${LIDAR_TOPIC}"
echo "[course-replay] selected IMU topic: ${IMU_TOPIC}"

if [[ -n "${OUTPUT_ENV_FILE:-}" ]]; then
  printf 'LIDAR_TOPIC=%q\nIMU_TOPIC=%q\n' "${LIDAR_TOPIC}" "${IMU_TOPIC}" \
    > "${OUTPUT_ENV_FILE}"
fi
