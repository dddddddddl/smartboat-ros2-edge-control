#!/usr/bin/env bash
set -Eeuo pipefail

export DISPLAY="${DISPLAY:-:0}"

if ! command -v xrandr >/dev/null 2>&1; then
  exit 0
fi

if xrandr | grep -q '^DP-0 connected'; then
  xrandr --output DP-0 --mode 1920x1080 --rate 60 --primary || true
fi

if command -v xhost >/dev/null 2>&1; then
  xhost SI:localuser:nx SI:localuser:"${USER:-ubuntu2204}" >/dev/null 2>&1 || true
fi
