#!/usr/bin/env bash
set -Eeuo pipefail

MAPPING_ROOT="/home/wheeltec/Desktop/work/rc-mapping"
PRODUCT_ROOT="/home/wheeltec/Desktop/work/autoracer_hooke"
RECORDINGS_ROOT="${MAPPING_ROOT}/recordings"
RUNTIME_ROOT="${MAPPING_ROOT}/.runtime"
ACTIVE_ROOT="${RUNTIME_ROOT}/active"
QOS_FILE="${PRODUCT_ROOT}/src/platform/rc/autoracer_rc_bringup/config/rc/mapping_recording_qos.yaml"
PREFLIGHT_TOOL="${PRODUCT_ROOT}/scripts/autoracer_rc_recording_preflight.py"
STOP_SCRIPT="${PRODUCT_ROOT}/scripts/autoracer_rc_recording_stop.sh"

G90_DEVICE="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AA6079369-if00"
IMU_DEVICE="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0003-if00-port0"
LIDAR_INTERFACE="enP8p1s0"
LIDAR_HOST_ADDRESS="192.168.1.102/32"
LIDAR_DEVICE_ADDRESS="192.168.1.200"

LOCKED_PILOT_HEAD="98064d37638d1d515b5db2ffd68ba078c35df7b2"
FIRMWARE_BASELINE="997d23aa7543a01041d33f62c561d3e3b927bc06"
RESERVED_FREE_BYTES=$((50 * 1024 * 1024 * 1024))
SPLIT_BYTES=$((4 * 1024 * 1024 * 1024))
CACHE_BYTES=$((256 * 1024 * 1024))

LABEL=""
SITE=""
OPERATOR_NAME="$(id -un)"
DRY_RUN="false"
VERBOSE="false"
ALLOW_DEGRADED_LIDAR="${RC_MAPPING_ALLOW_DEGRADED_LIDAR:-0}"
START_SUCCEEDED="false"
SESSION_DIR=""

usage() {
  cat <<'EOF'
内部用法：
  autoracer_rc_recording.sh --site <场地> [--label <简短标签>] \
    [--operator <姓名>] [--verbose] [--dry-run]

说明：
  - 正式操作请使用 scripts/autoracer_rc.sh 的“录制建图数据”。
  - 录制接受 RTK Fixed/Float、完整 GST、THS A，并执行固定 10 秒预检。
  - GGA quality 4/5 按原值记录，绝不把 Float 重标为 Fixed。
  - 只录 LiDAR/IMU/G90 原始数据；不启动底盘、Planning、Control 或车辆输出。
  - RC_MAPPING_ALLOW_DEGRADED_LIDAR=1 只为本会话审计放行 lidar_rate/
    lidar_maximum_gap；原始 FAIL 和实测值仍写入证据。
  - --dry-run 只检查命令、安装和静态条件，不启动节点、不打开设备、不创建会话。
EOF
}

fail() {
  printf '错误：%s\n' "$*" >&2
  exit 1
}

validate_text() {
  local field_name="$1"
  local value="$2"
  [[ -n "${value}" ]] || fail "${field_name} 不能为空"
  [[ ${#value} -le 128 ]] || fail "${field_name} 不能超过 128 个字符"
  [[ "${value}" != *$'\n'* && "${value}" != *$'\r'* && "${value}" != *$'\t'* ]] ||
    fail "${field_name} 不能包含控制字符"
}

while (($# > 0)); do
  case "$1" in
    --label)
      (($# >= 2)) || fail "--label 缺少值"
      LABEL="$2"
      shift 2
      ;;
    --site)
      (($# >= 2)) || fail "--site 缺少值"
      SITE="$2"
      shift 2
      ;;
    --operator)
      (($# >= 2)) || fail "--operator 缺少值"
      OPERATOR_NAME="$2"
      shift 2
      ;;
    --verbose)
      VERBOSE="true"
      shift
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      fail "未知参数 $1"
      ;;
    *)
      fail "未知位置参数 $1"
      ;;
  esac
done

if [[ -n "${LABEL}" ]]; then
  [[ "${LABEL}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] ||
    fail "标签必须以字母或数字开头，只能包含字母、数字、点、下划线和连字符，最长 64 字符"
fi
validate_text "场地" "${SITE}"
validate_text "操作者" "${OPERATOR_NAME}"
[[ "${ALLOW_DEGRADED_LIDAR}" == "0" || "${ALLOW_DEGRADED_LIDAR}" == "1" ]] ||
  fail "RC_MAPPING_ALLOW_DEGRADED_LIDAR 只能为 0 或 1"

for required_file in "${QOS_FILE}" "${PREFLIGHT_TOOL}" "${STOP_SCRIPT}"; do
  [[ -f "${required_file}" ]] || fail "缺少 ${required_file}"
done
[[ -d "${PRODUCT_ROOT}/.git" ]] || fail "产品仓库不存在：${PRODUCT_ROOT}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PRODUCT_ROOT}/vendor_ws/install/local_setup.bash"
# shellcheck disable=SC1091
source "${PRODUCT_ROOT}/install/local_setup.bash" 2> >(
  grep -Fv -- \
    "not found: \"${PRODUCT_ROOT}/install/hooke2_interface/share/hooke2_interface/local_setup.bash\"" >&2 || true
)
set -u

for command_name in ros2 python3 git setsid timeout sha256sum ip df ps awk grep tr tail wc cp date readlink tee; do
  command -v "${command_name}" >/dev/null 2>&1 || fail "缺少命令 ${command_name}"
done

BRINGUP_PREFIX="$(ros2 pkg prefix autoracer_rc_bringup 2>/dev/null)" ||
  fail "当前 install 中找不到 autoracer_rc_bringup"
DESCRIPTION_PREFIX="$(ros2 pkg prefix autoracer_rc_description 2>/dev/null)" ||
  fail "当前 install 中找不到 autoracer_rc_description"
ros2 pkg prefix lslidar_driver >/dev/null 2>&1 || fail "当前环境找不到 lslidar_driver"
ros2 pkg prefix hipnuc_imu >/dev/null 2>&1 || fail "当前环境找不到 hipnuc_imu"
ros2 pkg prefix nmea_navsat_driver >/dev/null 2>&1 || fail "当前环境找不到 nmea_navsat_driver"
python3 "${PREFLIGHT_TOOL}" --self-test >/dev/null || fail "建图预检工具自测失败"

AVAILABLE_BYTES="$(df --output=avail -B1 "${MAPPING_ROOT}" | tail -n 1 | tr -d ' ')"
[[ "${AVAILABLE_BYTES}" =~ ^[0-9]+$ ]] || fail "无法读取剩余磁盘空间"
REQUIRED_AVAILABLE_BYTES="${RESERVED_FREE_BYTES}"

lidar_network_status() {
  local failures=0
  ip link show dev "${LIDAR_INTERFACE}" >/dev/null 2>&1 || failures=$((failures + 1))
  [[ -r "/sys/class/net/${LIDAR_INTERFACE}/operstate" ]] || failures=$((failures + 1))
  if [[ -r "/sys/class/net/${LIDAR_INTERFACE}/operstate" ]]; then
    [[ "$(<"/sys/class/net/${LIDAR_INTERFACE}/operstate")" == "up" ]] || failures=$((failures + 1))
  fi
  ip -4 addr show dev "${LIDAR_INTERFACE}" 2>/dev/null | grep -Fq "inet ${LIDAR_HOST_ADDRESS}" ||
    failures=$((failures + 1))
  ip -4 route get "${LIDAR_DEVICE_ADDRESS}" 2>/dev/null |
    grep -Eq "dev ${LIDAR_INTERFACE}([[:space:]]|$).*src ${LIDAR_HOST_ADDRESS%/*}([[:space:]]|$)" ||
    failures=$((failures + 1))
  ((failures == 0))
}

if [[ "${DRY_RUN}" == "true" ]]; then
  printf 'DRY RUN：不会启动 ROS 节点、不会打开设备、不会创建录制目录。\n'
  printf '标签：%s\n场地：%s\n操作者：%s\n' \
    "${LABEL:-无}" "${SITE}" "${OPERATOR_NAME}"
  printf '录制时长：不限；磁盘保留线：%s bytes；当前可用：%s bytes\n' \
    "${RESERVED_FREE_BYTES}" "${AVAILABLE_BYTES}"
  [[ -e "${G90_DEVICE}" ]] && printf 'G90 稳定设备路径：存在\n' || printf 'G90 稳定设备路径：缺失\n'
  [[ -e "${IMU_DEVICE}" ]] && printf 'IMU 稳定设备路径：存在\n' || printf 'IMU 稳定设备路径：缺失\n'
  lidar_network_status && printf 'LiDAR 网络：就绪\n' ||
    printf 'LiDAR 网络：未就绪（LiDAR 关闭时这是预期结果）\n'
  printf '底盘：不启动、不订阅\n'
  printf 'GNSS 录制准入：RTK Fixed（4）或 Float（5），保留真实 quality\n'
  printf 'LiDAR 降级放行：%s（仅限 rate/maximum_gap，按原值留证）\n' \
    "${ALLOW_DEGRADED_LIDAR}"
  printf '预检等待：不设超时，按 Q 取消；固定质量窗口：10 秒\n'
  printf 'DRY RUN：静态命令检查完成。\n'
  START_SUCCEEDED="true"
  exit 0
fi

[[ -t 0 ]] || fail "正式录制必须在交互终端运行，以便用 S 开始、Q 停止"
((AVAILABLE_BYTES >= REQUIRED_AVAILABLE_BYTES)) ||
  fail "磁盘空间不足：必须至少保留 50 GiB"

[[ -e "${G90_DEVICE}" && -r "${G90_DEVICE}" && -w "${G90_DEVICE}" ]] ||
  fail "G90 稳定设备路径不存在或当前用户无读写权限：${G90_DEVICE}"
[[ -e "${IMU_DEVICE}" && -r "${IMU_DEVICE}" && -w "${IMU_DEVICE}" ]] ||
  fail "IMU 稳定设备路径不存在或当前用户无读写权限：${IMU_DEVICE}"
lidar_network_status || fail \
  "LiDAR 网络未就绪；必须先让 ${LIDAR_INTERFACE} 为 UP、配置 ${LIDAR_HOST_ADDRESS}，并确保 ${LIDAR_DEVICE_ADDRESS} 经该接口路由"

if command -v fuser >/dev/null 2>&1; then
  fuser "$(readlink -f "${G90_DEVICE}")" >/dev/null 2>&1 &&
    fail "G90 串口已被其他进程占用；先停止旧 sensing/reader"
  fuser "$(readlink -f "${IMU_DEVICE}")" >/dev/null 2>&1 &&
    fail "IMU 串口已被其他进程占用；先停止旧 sensing/driver"
fi

mkdir -p "${RUNTIME_ROOT}" "${RECORDINGS_ROOT}"
mkdir "${ACTIVE_ROOT}" 2>/dev/null ||
  fail "已有活动建图会话；请先运行 ${STOP_SCRIPT}，不要启动第二套设备图"

SESSION_ID="$(date +%Y%m%dT%H%M%S%z)"
if [[ -n "${LABEL}" ]]; then
  SESSION_ID+="-${LABEL}"
fi
SESSION_DIR="${RECORDINGS_ROOT}/${SESSION_ID}"
if ! mkdir "${SESSION_DIR}"; then
  rmdir "${ACTIVE_ROOT}" 2>/dev/null || true
  fail "无法创建会话目录 ${SESSION_DIR}"
fi
mkdir -p \
  "${SESSION_DIR}/raw" \
  "${SESSION_DIR}/logs" \
  "${SESSION_DIR}/quality" \
  "${SESSION_DIR}/manifests/config_snapshot"

printf '%s\n' "${SESSION_ID}" >"${ACTIVE_ROOT}/session_id"
printf '%s\n' "${SESSION_DIR}" >"${ACTIVE_ROOT}/session_dir"
printf 'false\n' >"${ACTIVE_ROOT}/recording_started"

cleanup_failed_start() {
  local exit_code=$?
  if [[ "${START_SUCCEEDED}" != "true" && -n "${SESSION_DIR}" && -d "${ACTIVE_ROOT}" ]]; then
    if [[ ! -f "${ACTIVE_ROOT}/failure_reason" ]]; then
      printf 'autoracer_rc_recording.sh exited before recording became active (code %s)\n' "${exit_code}" \
        >"${ACTIVE_ROOT}/failure_reason"
    fi
    "${STOP_SCRIPT}" --automatic >/dev/null 2>&1 || true
  fi
}
trap cleanup_failed_start EXIT
trap 'exit 130' INT TERM

git -C "${PRODUCT_ROOT}" status --porcelain=v1 --untracked-files=all \
  >"${SESSION_DIR}/manifests/product_git_status.txt"
PRODUCT_BRANCH="$(git -C "${PRODUCT_ROOT}" branch --show-current)"
PRODUCT_HEAD="$(git -C "${PRODUCT_ROOT}" rev-parse HEAD)"
if git -C "${PRODUCT_ROOT}" merge-base --is-ancestor "${LOCKED_PILOT_HEAD}" "${PRODUCT_HEAD}"; then
  LOCKED_ANCESTOR="true"
else
  LOCKED_ANCESTOR="false"
fi
PRODUCT_DIRTY_COUNT="$(wc -l <"${SESSION_DIR}/manifests/product_git_status.txt" | tr -d ' ')"

cp -- "${BRINGUP_PREFIX}/share/autoracer_rc_bringup/launch/sensing.launch.py" \
  "${SESSION_DIR}/manifests/config_snapshot/"
cp -- "${BRINGUP_PREFIX}/share/autoracer_rc_bringup/config/rc/lidar.param.yaml" \
  "${SESSION_DIR}/manifests/config_snapshot/"
cp -- "${BRINGUP_PREFIX}/share/autoracer_rc_bringup/config/rc/imu.param.yaml" \
  "${SESSION_DIR}/manifests/config_snapshot/"
cp -- "${BRINGUP_PREFIX}/share/autoracer_rc_bringup/config/rc/g90.param.yaml" \
  "${SESSION_DIR}/manifests/config_snapshot/"
cp -- "${DESCRIPTION_PREFIX}/share/autoracer_rc_description/config/sensor_extrinsics.yaml" \
  "${SESSION_DIR}/manifests/config_snapshot/"
cp -- "${QOS_FILE}" "${SESSION_DIR}/manifests/config_snapshot/recording_qos.yaml"
cp -- "${PREFLIGHT_TOOL}" "${SESSION_DIR}/manifests/config_snapshot/preflight_recording.py"
(
  cd "${SESSION_DIR}/manifests/config_snapshot"
  sha256sum ./* >../config_snapshot_sha256.txt
)

cat >"${SESSION_DIR}/manifests/required_topics.tsv" <<'EOF'
/sensing/lidar/raw/pointcloud	sensor_msgs/msg/PointCloud2
/sensing/imu/raw/imu_data	sensor_msgs/msg/Imu
/g90/raw/nmea_sentence	nmea_msgs/msg/Sentence
/tf_static	tf2_msgs/msg/TFMessage
EOF
cat >"${SESSION_DIR}/manifests/optional_topics.tsv" <<'EOF'
/tf	tf2_msgs/msg/TFMessage
/diagnostics	diagnostic_msgs/msg/DiagnosticArray
EOF

export SESSION_ID SESSION_DIR LABEL SITE OPERATOR_NAME
export PRODUCT_BRANCH PRODUCT_HEAD PRODUCT_DIRTY_COUNT LOCKED_PILOT_HEAD LOCKED_ANCESTOR
export FIRMWARE_BASELINE G90_DEVICE IMU_DEVICE LIDAR_INTERFACE LIDAR_HOST_ADDRESS
export LIDAR_DEVICE_ADDRESS AVAILABLE_BYTES REQUIRED_AVAILABLE_BYTES ALLOW_DEGRADED_LIDAR
python3 - <<'PY'
import json
import os
from pathlib import Path
from datetime import datetime

session = Path(os.environ["SESSION_DIR"])
required = []
optional = []
for destination, values in ((required, "required_topics.tsv"), (optional, "optional_topics.tsv")):
    for line in (session / "manifests" / values).read_text(encoding="utf-8").splitlines():
        topic, message_type = line.split("\t", 1)
        destination.append({"topic": topic, "type": message_type})
payload = {
    "schema_version": 1,
    "kind": "rc_mapping_recording_session",
    "status": "PREPARING",
    "session_id": os.environ["SESSION_ID"],
    "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    "label": os.environ["LABEL"],
    "site": os.environ["SITE"],
    "operator": os.environ["OPERATOR_NAME"],
    "clock": {"use_sim_time": False, "source": "ROS wall clock"},
    "capture": {
        "storage": "sqlite3",
        "compression": "none during capture",
        "split_bytes": 4294967296,
        "cache_bytes": 268435456,
        "maximum_duration": None,
        "raw_only": True,
        "degraded_lidar_override_requested": os.environ["ALLOW_DEGRADED_LIDAR"] == "1",
        "degraded_lidar_override_scope": [
            "lidar_rate",
            "lidar_maximum_gap",
        ],
        "required_topics": required,
        "optional_topics": optional,
    },
    "gnss_solution_policy": {
        "name": "RTK_FIXED_OR_FLOAT",
        "accepted_gga_qualities": [4, 5],
        "quality_labels_preserved": True,
        "rtk_float_is_not_relabelled_as_fixed": True,
        "scope": "raw mapping recording input only",
    },
    "devices": {
        "g90": {"path": os.environ["G90_DEVICE"], "baud": 115200, "expected_output": "GGA/GST/THS at 10 Hz; script sends no configuration commands"},
        "imu": {"path": os.environ["IMU_DEVICE"]},
        "lidar": {"interface": os.environ["LIDAR_INTERFACE"], "host_address": os.environ["LIDAR_HOST_ADDRESS"], "device_address": os.environ["LIDAR_DEVICE_ADDRESS"]},
    },
    "product": {
        "root": "/home/wheeltec/Desktop/work/autoracer_hooke",
        "branch": os.environ["PRODUCT_BRANCH"],
        "head": os.environ["PRODUCT_HEAD"],
        "locked_pilot_head": os.environ["LOCKED_PILOT_HEAD"],
        "locked_pilot_is_ancestor": os.environ["LOCKED_ANCESTOR"] == "true",
        "dirty_entry_count": int(os.environ["PRODUCT_DIRTY_COUNT"]),
        "status_file": "manifests/product_git_status.txt",
    },
    "firmware_baseline": os.environ["FIRMWARE_BASELINE"],
    "disk_budget": {
        "available_bytes_at_start": int(os.environ["AVAILABLE_BYTES"]),
        "required_available_bytes": int(os.environ["REQUIRED_AVAILABLE_BYTES"]),
        "reserved_free_bytes": 50 * 1024**3,
        "continuous_monitoring": True,
    },
    "safety": {
        "starts_chassis": False,
        "starts_control_chain": False,
        "sends_vehicle_commands": False,
        "g90_credentials_recorded": False,
    },
}
target = session / "manifests" / "session_manifest.json"
target.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

write_process_state() {
  local prefix="$1"
  local process_id="$2"
  local process_group=""
  local start_ticks
  local attempt
  for ((attempt = 0; attempt < 20; attempt += 1)); do
    process_group="$(ps -o pgid= -p "${process_id}" 2>/dev/null | tr -d ' ')"
    [[ "${process_group}" == "${process_id}" ]] && break
    sleep 0.05
  done
  [[ "${process_group}" == "${process_id}" ]] ||
    fail "${prefix} 没有形成独立且可安全停止的 process group"
  start_ticks="$(awk '{print $22}' "/proc/${process_id}/stat")"
  [[ "${process_group}" =~ ^[0-9]+$ && "${start_ticks}" =~ ^[0-9]+$ ]] ||
    fail "无法记录 ${prefix} 进程身份"
  printf '%s\n' "${process_id}" >"${ACTIVE_ROOT}/${prefix}_pid"
  printf '%s\n' "${process_group}" >"${ACTIVE_ROOT}/${prefix}_pgid"
  printf '%s\n' "${start_ticks}" >"${ACTIVE_ROOT}/${prefix}_start_ticks"
}

SENSING_COMMAND=(
  ros2 launch autoracer_rc_bringup sensing.launch.py
  launch_static_tf:=true
  launch_lidar:=true
  launch_imu:=true
  launch_g90:=false
  launch_g90_driver:=true
  "g90_device:=${G90_DEVICE}"
  g90_baud:=115200
  "imu_device:=${IMU_DEVICE}"
)
printf '%q ' "${SENSING_COMMAND[@]}" >"${SESSION_DIR}/manifests/sensing_command.txt"
printf '\n' >>"${SESSION_DIR}/manifests/sensing_command.txt"
setsid python3 -c '
import os
import signal
import sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])
' "${SENSING_COMMAND[@]}" \
  >>"${SESSION_DIR}/logs/sensing.log" 2>&1 &
SENSING_PID=$!
write_process_state "sensing" "${SENSING_PID}"
sleep 2
kill -0 "${SENSING_PID}" 2>/dev/null || fail \
  "RC sensing 启动即退出；查看 ${SESSION_DIR}/logs/sensing.log"

PREFLIGHT_COMMAND=(
  python3 "${PREFLIGHT_TOOL}"
  --output "${SESSION_DIR}/quality/preflight.json"
  --watch-pid "${SENSING_PID}"
  --lidar-probe-dir "${SESSION_DIR}/quality/lidar_probe_bag"
  --lidar-probe-log "${SESSION_DIR}/logs/lidar_probe.log"
  --lidar-qos-file "${QOS_FILE}"
)
printf '正在检查 LiDAR、IMU、RTK Fixed/Float、GST、THS A 和静态 TF。\n'
printf '持续等待 READY，不设超时；按 Q 取消。满足条件后固定观察 10 秒。详细状态：%s\n' \
  "${SESSION_DIR}/logs/preflight.log"
printf '%q ' "${PREFLIGHT_COMMAND[@]}" >"${SESSION_DIR}/manifests/preflight_command.txt"
printf '\n' >>"${SESSION_DIR}/manifests/preflight_command.txt"
set +e
if [[ "${VERBOSE}" == "true" ]]; then
  "${PREFLIGHT_COMMAND[@]}" 2>&1 | tee "${SESSION_DIR}/logs/preflight.log"
  PREFLIGHT_RESULT=${PIPESTATUS[0]}
else
  "${PREFLIGHT_COMMAND[@]}" >"${SESSION_DIR}/logs/preflight.log" 2>&1
  PREFLIGHT_RESULT=$?
fi
set -e
if ((PREFLIGHT_RESULT == 2)); then
  printf 'operator cancelled before mapping preflight reached READY\n' \
    >"${ACTIVE_ROOT}/failure_reason"
  "${STOP_SCRIPT}" --automatic >/dev/null 2>&1 || true
  START_SUCCEEDED="true"
  printf '已取消；正式 rosbag 未开始。\n'
  exit 0
fi
PREFLIGHT_OVERRIDE_ACCEPTED="false"
if ((PREFLIGHT_RESULT == 3)) && [[ "${ALLOW_DEGRADED_LIDAR}" == "1" ]]; then
  if python3 - \
    "${SESSION_DIR}/quality/preflight.json" \
    "${SESSION_DIR}/quality/preflight_override.json" \
    "${SESSION_ID}" "${SITE}" "${OPERATOR_NAME}" <<'PY'
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

preflight_path = Path(sys.argv[1])
override_path = Path(sys.argv[2])
session_id, site, operator = sys.argv[3:]
allowed_failures = {"lidar_rate", "lidar_maximum_gap"}

try:
    preflight_bytes = preflight_path.read_bytes()
    preflight = json.loads(preflight_bytes)
except Exception as error:
    print(f"LiDAR 降级放行拒绝：无法读取预检证据：{error}", file=sys.stderr)
    raise SystemExit(1)

failures = {str(item) for item in preflight.get("failures") or []}
failed_checks = {
    str(check.get("name"))
    for check in preflight.get("checks") or []
    if not bool(check.get("passed"))
}
if (
    preflight.get("status") != "FAIL"
    or not failures
    or failures != failed_checks
    or not failures <= allowed_failures
    or preflight.get("readiness_blockers")
    or preflight.get("details")
    or preflight.get("failure")
):
    print(
        "LiDAR 降级放行拒绝：只允许完整预检中 lidar_rate/"
        "lidar_maximum_gap 单独失败。",
        file=sys.stderr,
    )
    raise SystemExit(1)

checks = {
    str(check.get("name")): check
    for check in preflight.get("checks") or []
}
observed = {
    name: {
        "observed": checks[name].get("observed"),
        "required": checks[name].get("required"),
    }
    for name in sorted(failures)
}
payload = {
    "schema_version": 1,
    "kind": "rc_mapping_degraded_lidar_override",
    "status": "AUTHORIZED_DEGRADED_CAPTURE",
    "scope": "this recording session only",
    "session_id": session_id,
    "site": site,
    "operator": operator,
    "authorized_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    "authorization_source": "RC_MAPPING_ALLOW_DEGRADED_LIDAR=1",
    "reason": (
        "operator-authorized known LiDAR throughput limitation; "
        "preserve actual LiDAR quality evidence"
    ),
    "original_preflight_status": "FAIL",
    "allowed_failures": sorted(allowed_failures),
    "accepted_failures": sorted(failures),
    "observed": observed,
    "preflight_sha256": hashlib.sha256(preflight_bytes).hexdigest(),
    "operator_start_confirmation_required": True,
}
temporary = override_path.with_name(f".{override_path.name}.tmp")
temporary.write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
temporary.replace(override_path)
PY
  then
    PREFLIGHT_OVERRIDE_ACCEPTED="true"
    printf '警告：本会话仅放行已留证的 LiDAR 频率/最大间隔失败；其余预检均已通过。\n'
  fi
fi
if ((PREFLIGHT_RESULT != 0)) && [[ "${PREFLIGHT_OVERRIDE_ACCEPTED}" != "true" ]]; then
  printf 'preflight failed with code %s\n' "${PREFLIGHT_RESULT}" >"${ACTIVE_ROOT}/failure_reason"
  python3 - "${SESSION_DIR}/quality/preflight.json" <<'PY' || true
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
if path.is_file():
    payload = json.loads(path.read_text(encoding="utf-8"))
    blockers = payload.get("readiness_blockers") or []
    failures = payload.get("failures") or []
    if blockers:
        print("未就绪原因：" + "；".join(str(item) for item in blockers), file=sys.stderr)
    if failures:
        print("质量失败：" + "；".join(str(item) for item in failures), file=sys.stderr)
    if payload.get("details"):
        print("未就绪原因：" + str(payload["details"]), file=sys.stderr)
PY
  printf '错误：建图级预检失败，正式 rosbag 没有开始。\n' >&2
  printf '证据：%s\n' "${SESSION_DIR}" >&2
  while true; do
    if ! IFS= read -r -s -n 1 -p '按 Q 确认结论并清理返回：' FAILURE_CHOICE; then
      printf '\n' >&2
      fail "无法读取失败确认"
    fi
    printf '\n'
    case "${FAILURE_CHOICE}" in
      q|Q) break ;;
      *) printf '请输入 Q。\n' ;;
    esac
  done
  "${STOP_SCRIPT}" --automatic >/dev/null 2>&1 || true
  START_SUCCEEDED="true"
  exit 1
fi

if [[ "${PREFLIGHT_OVERRIDE_ACCEPTED}" == "true" ]]; then
  printf '\n[READY] 建图输入已就绪；LiDAR 降级项已按本会话授权留证，尚未开始录制。\n'
else
  printf '\n[READY] 建图输入和固定质量窗口检查通过，尚未开始录制。\n'
fi
while true; do
  if ! IFS= read -r -s -n 1 -p '[S] 开始录制  [Q] 取消：' READY_CHOICE; then
    printf '\n' >&2
    fail "无法读取开始确认"
  fi
  printf '\n'
  case "${READY_CHOICE}" in
    s|S)
      break
      ;;
    q|Q)
      printf 'operator cancelled after READY and before rosbag start\n' \
        >"${ACTIVE_ROOT}/failure_reason"
      "${STOP_SCRIPT}" --automatic >/dev/null 2>&1 || true
      START_SUCCEEDED="true"
      printf '已取消；正式 rosbag 未开始。\n'
      exit 0
      ;;
    *)
      printf '请输入 S 或 Q。\n'
      ;;
  esac
done

BAG_DIR="${SESSION_DIR}/raw/rosbag2"
TOPICS=(
  "/sensing/lidar/raw/pointcloud"
  "/sensing/imu/raw/imu_data"
  "/g90/raw/nmea_sentence"
  "/tf_static"
  "/tf"
  "/diagnostics"
)
RECORDER_COMMAND=(
  ros2 bag record
  --storage sqlite3
  --output "${BAG_DIR}"
  --max-bag-size "${SPLIT_BYTES}"
  --max-cache-size "${CACHE_BYTES}"
  --compression-mode none
  --qos-profile-overrides-path "${QOS_FILE}"
  "${TOPICS[@]}"
)
printf '%q ' "${RECORDER_COMMAND[@]}" >"${SESSION_DIR}/manifests/record_command.txt"
printf '\n' >>"${SESSION_DIR}/manifests/record_command.txt"
printf 'true\n' >"${ACTIVE_ROOT}/recording_started"
date --iso-8601=seconds >"${SESSION_DIR}/manifests/recording_started_at.txt"
DISK_STOP_MARKER="${SESSION_DIR}/logs/disk_reserve_stop_requested.txt"
RECORDER_UNEXPECTED_MARKER="${SESSION_DIR}/logs/recorder_unexpected_exit.txt"
setsid python3 -c '
import os
import signal
import sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])
' "${RECORDER_COMMAND[@]}" \
  >>"${SESSION_DIR}/logs/recorder.log" 2>&1 &
RECORDER_PID=$!
write_process_state "recorder" "${RECORDER_PID}"
sleep 3
kill -0 "${RECORDER_PID}" 2>/dev/null || fail \
  "rosbag recorder 启动即退出；查看 ${SESSION_DIR}/logs/recorder.log"

recorder_identity_matches() {
  [[ -r "/proc/${RECORDER_PID}/stat" ]] || return 1
  local expected_ticks
  local expected_pgid
  local current_ticks
  local current_pgid
  IFS= read -r expected_ticks <"${ACTIVE_ROOT}/recorder_start_ticks"
  IFS= read -r expected_pgid <"${ACTIVE_ROOT}/recorder_pgid"
  current_ticks="$(awk '{print $22}' "/proc/${RECORDER_PID}/stat" 2>/dev/null || true)"
  current_pgid="$(ps -o pgid= -p "${RECORDER_PID}" 2>/dev/null | tr -d ' ')"
  [[ "${current_ticks}" == "${expected_ticks}" && "${current_pgid}" == "${expected_pgid}" ]]
}

printf '\n[RECORDING] 正式原始 rosbag 已开始。\n'
printf '会话：%s\n' "${SESSION_ID}"
printf '目录：%s\n' "${SESSION_DIR}"
printf '录制时长不限；按 Q 正常停止并封存。磁盘可用空间达到 50 GiB 保留线时会自动安全停止。\n'

while recorder_identity_matches; do
  if IFS= read -r -s -n 1 -t 2 RECORDING_CHOICE; then
    if [[ "${RECORDING_CHOICE}" =~ ^[qQ]$ ]]; then
      printf '\n正在停止、落盘、校验并计算 SHA-256...\n'
      set +e
      "${STOP_SCRIPT}"
      STOP_RESULT=$?
      set -e
      START_SUCCEEDED="true"
      exit "${STOP_RESULT}"
    fi
  fi

  CURRENT_AVAILABLE_BYTES="$(
    df --output=avail -B1 "${MAPPING_ROOT}" 2>/dev/null | tail -n 1 | tr -d ' '
  )"
  if [[ ! "${CURRENT_AVAILABLE_BYTES}" =~ ^[0-9]+$ ]]; then
    printf 'disk monitor could not read available bytes\n' >"${ACTIVE_ROOT}/failure_reason"
    date --iso-8601=seconds >"${DISK_STOP_MARKER}"
    printf '\n[WARN] 无法继续监控磁盘，正在安全停止并封存。\n' >&2
    set +e
    "${STOP_SCRIPT}" --automatic
    STOP_RESULT=$?
    set -e
    START_SUCCEEDED="true"
    exit "${STOP_RESULT}"
  fi
  if (( CURRENT_AVAILABLE_BYTES <= RESERVED_FREE_BYTES )); then
    printf '%s\n' "${CURRENT_AVAILABLE_BYTES}" >"${DISK_STOP_MARKER}"
    printf '\n[WARN] 磁盘达到 50 GiB 保留线，正在自动安全停止并封存。\n' >&2
    set +e
    "${STOP_SCRIPT}" --automatic
    STOP_RESULT=$?
    set -e
    START_SUCCEEDED="true"
    exit "${STOP_RESULT}"
  fi
done

date --iso-8601=seconds >"${RECORDER_UNEXPECTED_MARKER}"
printf 'recorder exited before an operator or disk-guard stop request\n' \
  >"${ACTIVE_ROOT}/failure_reason"
printf '\n[FAIL] rosbag recorder 意外退出，正在保留并校验现有数据。\n' >&2
set +e
"${STOP_SCRIPT}" --automatic
STOP_RESULT=$?
set -e
START_SUCCEEDED="true"
if ((STOP_RESULT == 0)); then
  STOP_RESULT=1
fi
exit "${STOP_RESULT}"
