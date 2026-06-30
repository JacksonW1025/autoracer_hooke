#!/usr/bin/env bash
set -euo pipefail

IFACE="${LIDAR_IFACE:-eth0}"
HOST_IP="${LIDAR_HOST_IP:-192.168.1.102}"
SENSOR_IP="${LIDAR_SENSOR_IP:-192.168.1.200}"
STALE_HOST_IPS="${LIDAR_STALE_HOST_IPS:-192.168.1.120}"

usage() {
  cat <<EOF
Usage:
  sudo -E $0 [--clear]

Environment:
  LIDAR_IFACE      network interface connected to the C32 LiDAR, default: eth0
  LIDAR_HOST_IP    host-side LiDAR address, default: 192.168.1.102
  LIDAR_SENSOR_IP  C32 LiDAR address, default: 192.168.1.200

This script uses a /32 host address plus a /32 route to the LiDAR. Do not use
192.168.1.102/24 on the Raspberry Pi when WiFi is also on 192.168.1.0/24; that
can steal SSH return traffic from wlan0.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "$EUID" -ne 0 ]]; then
  echo "ERROR: run with sudo so the script can configure ${IFACE}." >&2
  usage >&2
  exit 1
fi

if ! command -v ip >/dev/null 2>&1; then
  echo "ERROR: iproute2 'ip' command is required." >&2
  exit 1
fi

matching_addrs() {
  local target_ip="$1"
  ip -4 -o addr show dev "$IFACE" | awk -v ip="$target_ip" '
    $3 == "inet" {
      split($4, parts, "/")
      if (parts[1] == ip) {
        print $4
      }
    }
  '
}

clear_lidar_link() {
  ip route del "$SENSOR_IP/32" dev "$IFACE" 2>/dev/null || true
  while IFS= read -r addr; do
    [[ -n "$addr" ]] || continue
    ip addr del "$addr" dev "$IFACE" 2>/dev/null || true
  done < <(matching_addrs "$HOST_IP")
  for stale_ip in $STALE_HOST_IPS; do
    [[ "$stale_ip" != "$HOST_IP" ]] || continue
    while IFS= read -r addr; do
      [[ -n "$addr" ]] || continue
      ip addr del "$addr" dev "$IFACE" 2>/dev/null || true
    done < <(matching_addrs "$stale_ip")
  done
}

if [[ "${1:-}" == "--clear" ]]; then
  clear_lidar_link
  echo "Cleared ${IFACE} LiDAR address ${HOST_IP} and route ${SENSOR_IP}/32."
  exit 0
fi

if [[ $# -gt 0 ]]; then
  echo "ERROR: unknown argument: $1" >&2
  usage >&2
  exit 1
fi

ip link set "$IFACE" up
clear_lidar_link
ip addr add "$HOST_IP/32" dev "$IFACE"
ip route replace "$SENSOR_IP/32" dev "$IFACE" src "$HOST_IP" scope link

echo "Configured ${IFACE}:"
ip -4 -br addr show dev "$IFACE"
echo "Route to LiDAR:"
ip route get "$SENSOR_IP"
