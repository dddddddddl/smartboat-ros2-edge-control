#!/usr/bin/env bash
set -Eeuo pipefail

DRIVER_URL="https://www.peak-system.com/quick/PCAN-Linux-Driver"
WORKDIR="${WORKDIR:-/tmp/peak-linux-driver-src}"
ARCHIVE="${ARCHIVE:-/tmp/peak-linux-driver.tar.gz}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Please run with sudo:"
  echo "  sudo bash $0"
  exit 1
fi

echo "==> Kernel: $(uname -r)"
if [ ! -e "/lib/modules/$(uname -r)/build" ]; then
  echo "Missing kernel headers: /lib/modules/$(uname -r)/build"
  exit 1
fi

echo "==> Downloading PEAK Linux driver"
python3 - "$DRIVER_URL" "$ARCHIVE" <<'PY'
import sys
import urllib.request

url, out = sys.argv[1], sys.argv[2]
urllib.request.urlretrieve(url, out)
print(out)
PY

echo "==> Extracting"
rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"
tar -xzf "$ARCHIVE" -C "$WORKDIR" --strip-components=1

echo "==> Building PEAK driver in SocketCAN/netdev mode"
make -C "$WORKDIR/driver" clean netdev

echo "==> Installing kernel module"
make -C "$WORKDIR/driver" install
depmod -a

echo "==> Loading pcan module"
modprobe pcan

echo "==> Loaded modules"
lsmod | grep -E '(^pcan|can_dev)' || true

echo "==> CAN interfaces"
find /sys/class/net -maxdepth 1 -type l -name 'can*' -print -exec readlink -f {} \;

echo
echo "Next, bring up the PEAK interface at 500K, for example:"
echo "  sudo ip link set canX type can bitrate 500000"
echo "  sudo ip link set canX up"
echo "  candump canX"
