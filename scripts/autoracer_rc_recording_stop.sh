#!/usr/bin/env bash
set -Eeuo pipefail

PRODUCT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(dirname "${PRODUCT_ROOT}")"
MAPPING_ROOT="${RC_MAPPING_ROOT:-${WORKSPACE_ROOT}/rc-mapping}"
RUNTIME_ROOT="${MAPPING_ROOT}/.runtime"
ACTIVE_ROOT="${RUNTIME_ROOT}/active"
STOP_LOCK="${RUNTIME_ROOT}/stop.lock"
RECORDINGS_ROOT="${MAPPING_ROOT}/recordings"
AUTOMATIC="false"

usage() {
  cat <<'EOF'
内部用法：
  autoracer_rc_recording_stop.sh [--automatic]

正式操作请使用 scripts/autoracer_rc.sh 的“录制建图数据”，并在录制中按 Q。
本脚本依次停止 rosbag 和 sensing，校验 metadata、必须 topic、消息数和进程退出，
再为原始 bag 生成 SHA-256 与最终 manifest；无论通过或失败都不删除原始数据。
EOF
}

fail() {
  printf '错误：%s\n' "$*" >&2
  exit 1
}

if (($# > 0)); then
  case "$1" in
    --automatic)
      AUTOMATIC="true"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "未知参数 $1"
      ;;
  esac
fi
[[ $# -le 1 ]] || fail "参数过多"

mkdir -p "${RUNTIME_ROOT}"
if ! mkdir "${STOP_LOCK}" 2>/dev/null; then
  if [[ "${AUTOMATIC}" == "true" ]]; then
    exit 0
  fi
  fail "另一个停止/封存过程正在运行，请等待其完成"
fi
release_stop_lock() {
  rmdir "${STOP_LOCK}" 2>/dev/null || true
}
trap release_stop_lock EXIT

if [[ ! -d "${ACTIVE_ROOT}" ]]; then
  if [[ "${AUTOMATIC}" == "true" ]]; then
    exit 0
  fi
  fail "当前没有活动建图录制会话"
fi

read_state() {
  local name="$1"
  [[ -f "${ACTIVE_ROOT}/${name}" ]] || return 1
  local value
  IFS= read -r value <"${ACTIVE_ROOT}/${name}"
  printf '%s' "${value}"
}

SESSION_ID="$(read_state session_id)" || fail "活动状态缺少 session_id"
SESSION_DIR="$(read_state session_dir)" || fail "活动状态缺少 session_dir"
case "${SESSION_DIR}" in
  "${RECORDINGS_ROOT}"/*) ;;
  *) fail "拒绝处理录制根目录以外的会话路径：${SESSION_DIR}" ;;
esac
[[ -d "${SESSION_DIR}" ]] || fail "会话目录不存在：${SESSION_DIR}"
RECORDING_STARTED="$(read_state recording_started 2>/dev/null || printf 'false')"

PROCESS_REMAINS="false"
LAST_OUTCOME="not_started"

identity_matches() {
  local prefix="$1"
  local pid_file="${ACTIVE_ROOT}/${prefix}_pid"
  local pgid_file="${ACTIVE_ROOT}/${prefix}_pgid"
  local ticks_file="${ACTIVE_ROOT}/${prefix}_start_ticks"
  [[ -f "${pid_file}" && -f "${pgid_file}" && -f "${ticks_file}" ]] || return 2
  local pid pgid expected_ticks current_ticks current_pgid own_pgid
  IFS= read -r pid <"${pid_file}"
  IFS= read -r pgid <"${pgid_file}"
  IFS= read -r expected_ticks <"${ticks_file}"
  [[ "${pid}" =~ ^[0-9]+$ && "${pgid}" =~ ^[0-9]+$ && "${expected_ticks}" =~ ^[0-9]+$ ]] ||
    return 3
  [[ -r "/proc/${pid}/stat" ]] || return 1
  current_ticks="$(awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true)"
  current_pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
  own_pgid="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
  [[ "${current_ticks}" == "${expected_ticks}" && "${current_pgid}" == "${pgid}" ]] || return 3
  [[ "${pgid}" != "${own_pgid}" ]] || return 3
  return 0
}

process_group_has_members() {
  local prefix="$1"
  local pgid_file="${ACTIVE_ROOT}/${prefix}_pgid"
  [[ -f "${pgid_file}" ]] || return 1
  local pgid own_pgid
  IFS= read -r pgid <"${pgid_file}"
  own_pgid="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
  [[ "${pgid}" =~ ^[0-9]+$ && "${pgid}" != "${own_pgid}" ]] || return 1
  ps -eo pgid= | awk -v target="${pgid}" '$1 == target {found = 1} END {exit !found}'
}

wait_for_identity_to_exit() {
  local prefix="$1"
  local maximum_seconds="$2"
  local elapsed=0
  while ((elapsed < maximum_seconds)); do
    if identity_matches "${prefix}"; then
      :
    else
      local result=$?
      if [[ ${result} -eq 1 ]]; then
        if process_group_has_members "${prefix}"; then
          sleep 1
          elapsed=$((elapsed + 1))
          continue
        fi
        return 0
      fi
      [[ ${result} -eq 2 ]] && return 0
      return 2
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

stop_named_process() {
  local prefix="$1"
  local interrupt_grace="$2"
  local terminate_grace="$3"
  local identity_result
  if identity_matches "${prefix}"; then
    identity_result=0
  else
    identity_result=$?
  fi
  case "${identity_result}" in
    0) ;;
    1)
      if process_group_has_members "${prefix}"; then
        identity_result=0
      else
        LAST_OUTCOME="already_exited"
        return 0
      fi
      ;;
    2)
      LAST_OUTCOME="not_started"
      return 0
      ;;
    *)
      LAST_OUTCOME="identity_mismatch"
      PROCESS_REMAINS="true"
      return 1
      ;;
  esac

  local pgid
  IFS= read -r pgid <"${ACTIVE_ROOT}/${prefix}_pgid"
  # Humble launch does not forward an interactive SIGINT because it expects
  # the terminal to have signalled the whole foreground group.  The recorder
  # created this audited PGID with setsid, so address that owned group once.
  printf '正在以 SIGINT 停止 %s 生产进程组 %s...\n' "${prefix}" "${pgid}"
  kill -INT -- "-${pgid}" 2>/dev/null || true
  local wait_result
  if wait_for_identity_to_exit "${prefix}" "${interrupt_grace}"; then
    wait_result=0
  else
    wait_result=$?
  fi
  if ((wait_result == 0)); then
    LAST_OUTCOME="sigint"
    return 0
  fi
  if ((wait_result == 2)); then
    LAST_OUTCOME="identity_mismatch"
    PROCESS_REMAINS="true"
    return 1
  fi
  printf '%s 在 %s 秒内未退出，升级为 SIGTERM。\n' "${prefix}" "${interrupt_grace}" >&2
  kill -TERM -- "-${pgid}" 2>/dev/null || true
  if wait_for_identity_to_exit "${prefix}" "${terminate_grace}"; then
    wait_result=0
  else
    wait_result=$?
  fi
  if ((wait_result == 0)); then
    LAST_OUTCOME="sigterm"
    return 0
  fi
  if ((wait_result == 2)); then
    LAST_OUTCOME="identity_mismatch"
    PROCESS_REMAINS="true"
    return 1
  fi
  LAST_OUTCOME="still_running"
  PROCESS_REMAINS="true"
  return 1
}

if [[ "${AUTOMATIC}" == "false" ]]; then
  if [[ "${RECORDING_STARTED}" == "true" ]] && identity_matches "recorder"; then
    date --iso-8601=seconds >"${SESSION_DIR}/logs/manual_stop_requested.txt"
  else
    date --iso-8601=seconds >"${SESSION_DIR}/logs/manual_stop_found_recorder_inactive.txt"
  fi
fi

RECORDER_OUTCOME="not_started"
if [[ "${RECORDING_STARTED}" == "true" ]]; then
  stop_named_process "recorder" 90 15 || true
  RECORDER_OUTCOME="${LAST_OUTCOME}"
fi

# Freeze the sensing log boundary after rosbag is sealed but before sensor shutdown.
# Runtime failures invalidate capture; shutdown-only vendor diagnostics are retained separately.
SENSING_LOG="${SESSION_DIR}/logs/sensing.log"
if [[ -f "${SENSING_LOG}" ]]; then
  stat -c '%s' "${SENSING_LOG}" >"${SESSION_DIR}/manifests/sensing_log_bytes_before_shutdown.txt"
else
  printf '0\n' >"${SESSION_DIR}/manifests/sensing_log_bytes_before_shutdown.txt"
fi

SENSING_OUTCOME="not_started"
stop_named_process "sensing" 30 15 || true
SENSING_OUTCOME="${LAST_OUTCOME}"
date --iso-8601=seconds >"${SESSION_DIR}/manifests/stop_requested_at.txt"

# rosbag info only reads the sealed bag. It never reindexes or rewrites an incomplete bag.
BAG_DIR="${SESSION_DIR}/raw/rosbag2"
BAG_INFO_RESULT="not_run"
if [[ "${RECORDING_STARTED}" == "true" && -d "${BAG_DIR}" ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  set +e
  timeout 60s ros2 bag info "${BAG_DIR}" >"${SESSION_DIR}/logs/bag_info.txt" 2>&1
  BAG_INFO_CODE=$?
  set -e
  BAG_INFO_RESULT="${BAG_INFO_CODE}"
fi

RECORDER_EXIT_CODE="unknown"
SENSING_EXIT_CODE="unknown"
FAILURE_REASON=""
if [[ -f "${ACTIVE_ROOT}/failure_reason" ]]; then
  IFS= read -r FAILURE_REASON <"${ACTIVE_ROOT}/failure_reason"
fi

export SESSION_ID SESSION_DIR BAG_DIR RECORDING_STARTED RECORDER_OUTCOME SENSING_OUTCOME
export RECORDER_EXIT_CODE SENSING_EXIT_CODE BAG_INFO_RESULT FAILURE_REASON PROCESS_REMAINS
printf '正在核对 metadata、topic 和日志，并计算原始 bag SHA-256；大录制会话可能需要数分钟。\n'
set +e
python3 - <<'PY'
from __future__ import annotations

from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from typing import Any

import yaml

session = Path(os.environ["SESSION_DIR"])
bag = Path(os.environ["BAG_DIR"])
recording_started = os.environ["RECORDING_STARTED"] == "true"


def read_topics(name: str) -> list[dict[str, str]]:
    result = []
    path = session / "manifests" / name
    if not path.is_file():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        topic, message_type = line.split("\t", 1)
        result.append({"topic": topic, "type": message_type})
    return result


def parse_code(value: str) -> int | None:
    try:
        return int(value)
    except ValueError:
        return None


def duration_ns(metadata: dict[str, Any]) -> int:
    value = metadata.get("duration", 0)
    if isinstance(value, dict):
        value = value.get("nanoseconds", 0)
    return int(value or 0)


checks: list[dict[str, Any]] = []


def add_check(name: str, passed: bool, observed: Any, required: Any) -> None:
    checks.append({"name": name, "passed": bool(passed), "observed": observed, "required": required})


required_topics = read_topics("required_topics.tsv")
optional_topics = read_topics("optional_topics.tsv")
mandatory_topic_names = {
    "/sensing/lidar/raw/pointcloud",
    "/sensing/imu/raw/imu_data",
    "/g90/raw/nmea_sentence",
    "/tf_static",
}
metadata_path = bag / "metadata.yaml"
metadata: dict[str, Any] = {}
bag_info: dict[str, Any] = {}
topic_records: dict[str, dict[str, Any]] = {}
preflight_status: Any = None
preflight_failures: set[str] = set()
preflight_override: dict[str, Any] = {}
preflight_override_error = ""
degraded_lidar_override = False

if recording_started:
    add_check(
        "required_topics_manifest",
        mandatory_topic_names <= {item["topic"] for item in required_topics},
        required_topics,
        sorted(mandatory_topic_names),
    )
    preflight_path = session / "quality" / "preflight.json"
    preflight_override_path = session / "quality" / "preflight_override.json"
    preflight_payload: dict[str, Any] = {}
    if preflight_path.is_file():
        try:
            preflight_payload = json.loads(preflight_path.read_text(encoding="utf-8"))
            preflight_status = preflight_payload.get("status")
            preflight_failures = {
                str(item) for item in preflight_payload.get("failures") or []
            }
        except Exception as error:
            preflight_status = f"parse error: {error}"
    preflight_accepted = preflight_status == "PASS" and not preflight_failures
    if preflight_status == "FAIL":
        try:
            preflight_override = json.loads(
                preflight_override_path.read_text(encoding="utf-8")
            )
            allowed_failures = {"lidar_rate", "lidar_maximum_gap"}
            accepted_failures = {
                str(item)
                for item in preflight_override.get("accepted_failures") or []
            }
            declared_allowed_failures = {
                str(item)
                for item in preflight_override.get("allowed_failures") or []
            }
            failed_checks = {
                str(check.get("name"))
                for check in preflight_payload.get("checks") or []
                if not bool(check.get("passed"))
            }
            actual_preflight_sha256 = hashlib.sha256(
                preflight_path.read_bytes()
            ).hexdigest()
            override_errors = []
            expected_fields = {
                "schema_version": 1,
                "kind": "rc_mapping_degraded_lidar_override",
                "status": "AUTHORIZED_DEGRADED_CAPTURE",
                "scope": "this recording session only",
                "session_id": os.environ["SESSION_ID"],
                "authorization_source": "RC_MAPPING_ALLOW_DEGRADED_LIDAR=1",
                "original_preflight_status": "FAIL",
                "operator_start_confirmation_required": True,
                "preflight_sha256": actual_preflight_sha256,
            }
            for key, expected in expected_fields.items():
                if preflight_override.get(key) != expected:
                    override_errors.append(
                        f"{key}={preflight_override.get(key)!r}, expected {expected!r}"
                    )
            if (
                not preflight_failures
                or preflight_failures != failed_checks
                or preflight_failures != accepted_failures
                or not preflight_failures <= allowed_failures
            ):
                override_errors.append(
                    "preflight/failed-check/accepted failure sets do not match"
                )
            if declared_allowed_failures != allowed_failures:
                override_errors.append("allowed failure set is not exact")
            if (
                preflight_payload.get("readiness_blockers")
                or preflight_payload.get("details")
                or preflight_payload.get("failure")
            ):
                override_errors.append("preflight has a non-LiDAR blocker or internal failure")
            if override_errors:
                preflight_override_error = "; ".join(override_errors)
            else:
                preflight_accepted = True
                degraded_lidar_override = True
        except Exception as error:
            preflight_override_error = str(error)
    add_check(
        "preflight_accepted",
        preflight_accepted,
        {
            "status": preflight_status,
            "failures": sorted(preflight_failures),
            "degraded_lidar_override": degraded_lidar_override,
            "override_error": preflight_override_error,
        },
        "PASS, or an exact audited override limited to lidar_rate/lidar_maximum_gap",
    )
    add_check("metadata_exists", metadata_path.is_file(), str(metadata_path), "existing metadata.yaml")
    if metadata_path.is_file():
        try:
            loaded = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
            bag_info = loaded.get("rosbag2_bagfile_information", loaded)
            metadata = loaded
            add_check("metadata_parse", bool(bag_info), sorted(bag_info.keys()), "non-empty rosbag2_bagfile_information")
        except Exception as error:  # preserve evidence and report the parse failure
            add_check("metadata_parse", False, str(error), "valid YAML")
    if bag_info:
        add_check("storage_identifier", bag_info.get("storage_identifier") == "sqlite3", bag_info.get("storage_identifier"), "sqlite3")
        add_check("duration_positive", duration_ns(bag_info) > 0, duration_ns(bag_info), "> 0 ns")
        add_check("total_message_count_positive", int(bag_info.get("message_count", 0)) > 0, int(bag_info.get("message_count", 0)), "> 0")
        for item in bag_info.get("topics_with_message_count", []):
            topic_metadata = item.get("topic_metadata", {})
            topic_records[str(topic_metadata.get("name", ""))] = {
                "type": topic_metadata.get("type"),
                "message_count": int(item.get("message_count", 0)),
                "serialization_format": topic_metadata.get("serialization_format"),
            }
        for expected in required_topics:
            observed = topic_records.get(expected["topic"])
            add_check(
                f"required_topic:{expected['topic']}",
                observed is not None and observed.get("type") == expected["type"] and int(observed.get("message_count", 0)) > 0,
                observed,
                {"type": expected["type"], "message_count": "> 0"},
            )
        relative_paths = bag_info.get("relative_file_paths", [])
        missing_files = [name for name in relative_paths if not (bag / name).is_file() or (bag / name).stat().st_size <= 0]
        add_check("storage_files", bool(relative_paths) and not missing_files, {"relative_paths": relative_paths, "missing_or_empty": missing_files}, "at least one non-empty storage file")
else:
    add_check("recording_started", False, os.environ.get("FAILURE_REASON", ""), "preflight must pass before capture")

recorder_outcome = os.environ["RECORDER_OUTCOME"]
sensing_outcome = os.environ["SENSING_OUTCOME"]
recorder_exit = parse_code(os.environ["RECORDER_EXIT_CODE"])
sensing_exit = parse_code(os.environ["SENSING_EXIT_CODE"])
process_remains = os.environ["PROCESS_REMAINS"] == "true"
manual_stop_requested = (session / "logs" / "manual_stop_requested.txt").is_file()
disk_reserve_stop_requested = (session / "logs" / "disk_reserve_stop_requested.txt").is_file()
recorder_unexpected_exit = (session / "logs" / "recorder_unexpected_exit.txt").is_file()
failure_reason = os.environ.get("FAILURE_REASON", "")

add_check("no_process_identity_or_exit_failure", not process_remains and recorder_outcome not in {"identity_mismatch", "still_running", "sigterm"} and sensing_outcome not in {"identity_mismatch", "still_running", "sigterm"}, {"process_remains": process_remains, "recorder": recorder_outcome, "sensing": sensing_outcome}, "no identity mismatch, SIGTERM fallback, or remaining process")

if recording_started:
    add_check("bag_info_command", parse_code(os.environ["BAG_INFO_RESULT"]) == 0, os.environ["BAG_INFO_RESULT"], 0)
    add_check("runtime_failure_reason_absent", not failure_reason, failure_reason, "")
    recorder_stop_expected = (
        recorder_outcome in {"sigint", "already_exited"}
        and (manual_stop_requested or disk_reserve_stop_requested)
    ) and not recorder_unexpected_exit
    add_check(
        "recorder_exit",
        recorder_stop_expected,
        {
            "code": os.environ["RECORDER_EXIT_CODE"],
            "outcome": recorder_outcome,
            "manual_stop_requested": manual_stop_requested,
            "disk_reserve_stop_requested": disk_reserve_stop_requested,
            "unexpected_exit": recorder_unexpected_exit,
        },
        "clean SIGINT after a recorded manual or disk-reserve stop request",
    )
sensing_exit_ok = sensing_exit in {0, 130} or (sensing_exit is None and sensing_outcome == "sigint")
add_check("sensing_exit", sensing_exit_ok, {"code": os.environ["SENSING_EXIT_CODE"], "outcome": sensing_outcome}, "code 0/130, or inferred clean SIGINT")

log_findings: dict[str, list[str]] = {
    "sensing_runtime": [],
    "sensing_shutdown": [],
    "recorder": [],
}
sensing_pattern = re.compile(
    r"process has died|terminate called|Aborted|Traceback|segmentation fault",
    re.IGNORECASE,
)
recorder_pattern = re.compile(
    r"messages? (?:were )?(?:lost|dropped)|failed to write|write[^\n]*error|serialization[^\n]*error",
    re.IGNORECASE,
)

sensing_path = session / "logs" / "sensing.log"
boundary_path = session / "manifests" / "sensing_log_bytes_before_shutdown.txt"
sensing_bytes = sensing_path.read_bytes() if sensing_path.is_file() else b""
try:
    sensing_boundary = int(boundary_path.read_text(encoding="utf-8").strip())
    boundary_valid = 0 <= sensing_boundary <= len(sensing_bytes)
except (OSError, ValueError):
    sensing_boundary = len(sensing_bytes)
    boundary_valid = False
if not boundary_valid:
    sensing_boundary = len(sensing_bytes)
add_check(
    "sensing_log_shutdown_boundary",
    boundary_valid,
    {"boundary_bytes": sensing_boundary, "total_bytes": len(sensing_bytes)},
    "valid byte boundary captured after recorder stop and before sensing stop",
)
for key, content in (
    ("sensing_runtime", sensing_bytes[:sensing_boundary]),
    ("sensing_shutdown", sensing_bytes[sensing_boundary:]),
):
    for line in content.decode("utf-8", errors="replace").splitlines():
        if sensing_pattern.search(line):
            log_findings[key].append(line[:500])
            if len(log_findings[key]) >= 20:
                break
add_check(
    "sensing_runtime_log_clean",
    not log_findings["sensing_runtime"],
    log_findings["sensing_runtime"],
    [],
)

recorder_path = session / "logs" / "recorder.log"
if recorder_path.is_file():
    for line in recorder_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if recorder_pattern.search(line):
            log_findings["recorder"].append(line[:500])
            if len(log_findings["recorder"]) >= 20:
                break
add_check("recorder_log_clean", not log_findings["recorder"], log_findings["recorder"], [])

hash_records = []
total_raw_bytes = 0
if bag.is_dir():
    for path in sorted(item for item in bag.rglob("*") if item.is_file()):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(chunk)
        size = path.stat().st_size
        total_raw_bytes += size
        hash_records.append({"path": str(path.relative_to(session)), "size_bytes": size, "sha256": digest.hexdigest()})
add_check("raw_hashes", bool(hash_records) and total_raw_bytes > 0, {"file_count": len(hash_records), "total_raw_bytes": total_raw_bytes}, "at least one non-empty raw bag file")
hash_path = session / "manifests" / "raw_sha256.txt"
hash_path.write_text("".join(f"{item['sha256']}  {item['path']}\n" for item in hash_records), encoding="utf-8")

failures = [check["name"] for check in checks if not check["passed"]]
warnings = []
if log_findings["sensing_shutdown"]:
    warnings.append(
        {
            "name": "sensing_shutdown_log_findings",
            "details": log_findings["sensing_shutdown"],
            "impact": "occurred only after rosbag was sealed; retained for driver cleanup follow-up",
        }
    )
payload = {
    "schema_version": 1,
    "kind": "rc_mapping_recording_stop",
    "session_id": os.environ["SESSION_ID"],
    "completed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    "status": "PASS" if not failures else "FAIL",
    "asset_state": "SEALED_RAW" if recording_started and not failures else "PRESERVED_NOT_ACCEPTED",
    "failure_reason_from_start": os.environ.get("FAILURE_REASON", ""),
    "processes": {
        "recorder": {"stop_outcome": recorder_outcome, "exit_code": recorder_exit},
        "sensing": {"stop_outcome": sensing_outcome, "exit_code": sensing_exit},
        "process_remains": process_remains,
        "manual_stop_requested": manual_stop_requested,
        "disk_reserve_stop_requested": disk_reserve_stop_requested,
        "recorder_unexpected_exit": recorder_unexpected_exit,
    },
    "bag_info_exit_code": parse_code(os.environ["BAG_INFO_RESULT"]),
    "preflight": {
        "status": preflight_status,
        "degraded_lidar_override": degraded_lidar_override,
        "override": preflight_override or None,
    },
    "metadata": {
        "duration_ns": duration_ns(bag_info) if bag_info else 0,
        "message_count": int(bag_info.get("message_count", 0)) if bag_info else 0,
        "topics": topic_records,
    },
    "required_topics": required_topics,
    "optional_topics": optional_topics,
    "raw_files": hash_records,
    "total_raw_bytes": total_raw_bytes,
    "checks": checks,
    "failures": failures,
    "warnings": warnings,
    "log_findings": log_findings,
    "raw_data_deleted_or_reindexed": False,
}
target = session / "manifests" / "stop_manifest.json"
temporary = target.with_name(f".{target.name}.tmp")
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
temporary.replace(target)
print(payload["status"])
sys.exit(0 if payload["status"] == "PASS" else 4)
PY
FINALIZE_RESULT=$?
set -e

if [[ "${PROCESS_REMAINS}" == "false" ]]; then
  find "${ACTIVE_ROOT}" -mindepth 1 -maxdepth 1 -type f -delete
  rmdir "${ACTIVE_ROOT}" 2>/dev/null || true
else
  printf '警告：仍有进程或进程身份不匹配；为避免误杀，活动状态已保留在 %s。\n' "${ACTIVE_ROOT}" >&2
fi

printf '会话：%s\n' "${SESSION_ID}"
printf '目录：%s\n' "${SESSION_DIR}"
printf 'rosbag 停止结果：%s；sensing 停止结果：%s\n' "${RECORDER_OUTCOME}" "${SENSING_OUTCOME}"
if ((FINALIZE_RESULT == 0)); then
  printf '封存校验：PASS（原始 bag、topic 计数和 SHA-256 已写入 manifests）。\n'
  exit 0
fi
printf '封存校验：FAIL；原始数据已保留，禁止把本会话作为正式建图输入，详见 manifests/stop_manifest.json。\n' >&2
exit "${FINALIZE_RESULT}"
