#!/usr/bin/env bash
set -Eeuo pipefail

BITRATE="${1:-500000}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Please run with sudo:"
  echo "  sudo bash $0 ${BITRATE}"
  exit 1
fi

iface=""
for netdev in /sys/class/net/can*; do
  [ -e "$netdev" ] || continue
  target="$(readlink -f "$netdev")"
  if [[ "$target" == *"/virtual/net/"* ]]; then
    iface="$(basename "$netdev")"
    break
  fi
done

if [ -z "$iface" ]; then
  echo "No PEAK SocketCAN interface found. Is the pcan module loaded and PCAN-USB plugged in?"
  exit 1
fi

ip link set "$iface" down 2>/dev/null || true
ip link set "$iface" type can bitrate "$BITRATE"
ip link set "$iface" up

echo "$iface"
ip -details link show "$iface"
