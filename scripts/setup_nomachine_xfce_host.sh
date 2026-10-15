#!/usr/bin/env bash
set -Eeuo pipefail

NOMACHINE_VERSION="9.7.3_1"
NOMACHINE_SERIES="9.7"
NOMACHINE_DEB="nomachine_${NOMACHINE_VERSION}_arm64.deb"
NOMACHINE_URL="https://web9001.nomachine.com/download/${NOMACHINE_SERIES}/Arm/${NOMACHINE_DEB}"
NOMACHINE_MD5="6328c0c49b559ae3ae1b232d5b60cb77"
TARGET_USER="${SUDO_USER:-${USER}}"
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"

die() {
  printf '\n[ERROR] %s\n' "$*" >&2
  exit 1
}

info() {
  printf '\n==> %s\n' "$*"
}

require_root() {
  if [ "$(id -u)" -ne 0 ]; then
    die "请用 sudo 运行：sudo bash scripts/setup_nomachine_xfce_host.sh"
  fi
}

check_platform() {
  local arch
  arch="$(dpkg --print-architecture)"
  if [ "$arch" != "arm64" ]; then
    die "当前架构是 ${arch}，此脚本只安装 Jetson/aarch64 使用的 NoMachine arm64 包。"
  fi

  . /etc/os-release
  if [ "${VERSION_ID:-}" != "22.04" ]; then
    die "当前系统是 ${PRETTY_NAME:-unknown}，本项目宿主机目标是 Ubuntu 22.04。"
  fi
}

install_desktop() {
  info "安装 XFCE 和桌面基础组件"
  export DEBIAN_FRONTEND=noninteractive

  apt-get update
  apt-get install -y \
    ca-certificates \
    curl \
    dbus-x11 \
    xfce4 \
    xfce4-goodies \
    x11-xserver-utils

  if command -v gdm3 >/dev/null 2>&1; then
    printf '/usr/sbin/gdm3\n' > /etc/X11/default-display-manager
  fi
  systemctl set-default graphical.target
}

install_nomachine() {
  local deb_path="/tmp/${NOMACHINE_DEB}"

  if dpkg-query -W -f='${Status}' nomachine 2>/dev/null | grep -q 'install ok installed'; then
    info "NoMachine 已安装，跳过 DEB 安装"
    return
  fi

  info "下载 NoMachine ARM64 安装包"
  curl -fL --retry 3 --retry-delay 2 -o "$deb_path" "$NOMACHINE_URL"

  info "校验 NoMachine 安装包"
  echo "${NOMACHINE_MD5}  ${deb_path}" | md5sum -c -

  info "安装 NoMachine"
  apt-get install -y "$deb_path"
}

configure_xfce_session() {
  info "记录 NoMachine 虚拟桌面用户会话偏好为 XFCE"

  install -d -m 0755 -o "${TARGET_USER}" -g "${TARGET_USER}" "${TARGET_HOME}"
  cat > "${TARGET_HOME}/.dmrc" <<'EOF'
[Desktop]
Session=xfce
EOF
  chown "${TARGET_USER}:${TARGET_USER}" "${TARGET_HOME}/.dmrc"
  chmod 0644 "${TARGET_HOME}/.dmrc"
}

install_display_session_hook() {
  local hook="/etc/X11/Xsession.d/98smartboat-display-setup"

  info "安装物理显示初始化钩子"
  cat > "$hook" <<'EOF'
# Smartboat physical display setup for GDM/GNOME Xorg sessions.
if [ "${USER:-}" = "ubuntu2204" ] && [ -x /home/ubuntu2204/smartboat/scripts/smartboat_display_setup.sh ]; then
  ( sleep 5; /home/ubuntu2204/smartboat/scripts/smartboat_display_setup.sh ) &
fi
EOF
  chmod 0644 "$hook"
}

set_nomachine_key() {
  local file="$1"
  local key="$2"
  local value="$3"

  if grep -Eq "^[#[:space:]]*${key}[[:space:]]+" "$file"; then
    sed -i -E "s|^[#[:space:]]*${key}[[:space:]]+.*|${key} ${value}|" "$file"
  else
    printf '\n%s %s\n' "$key" "$value" >> "$file"
  fi
}

configure_nomachine() {
  local server_cfg="/usr/NX/etc/server.cfg"
  local node_cfg="/usr/NX/etc/node.cfg"

  [ -f "$server_cfg" ] || die "没有找到 ${server_cfg}，NoMachine 安装可能失败。"

  info "配置 NoMachine 无头虚拟桌面"
  cp -a "$server_cfg" "${server_cfg}.smartboat.bak"
  set_nomachine_key "$server_cfg" "CreateDisplay" "1"
  set_nomachine_key "$server_cfg" "DisplayOwner" "${TARGET_USER}"

  if [ -f "$node_cfg" ]; then
    cp -a "$node_cfg" "${node_cfg}.smartboat.bak"
    set_nomachine_key "$node_cfg" "DefaultDesktopCommand" '"/usr/bin/startxfce4"'
    set_nomachine_key "$node_cfg" "PhysicalDisplays" ":0"
  fi

  systemctl enable nxserver || true
  if command -v /usr/NX/bin/nxserver >/dev/null 2>&1; then
    /usr/NX/bin/nxserver --restart
  else
    systemctl restart nxserver
  fi
}

configure_nvidia_xorg_headless() {
  local xorg_dir="/etc/X11/xorg.conf.d"
  local old_headless_cfg="${xorg_dir}/20-smartboat-nvidia-headless.conf"

  info "确认 Xorg 使用 NVIDIA GPU，并禁用会影响物理 DP 的强制 headless 片段"
  install -d -m 0755 "$xorg_dir"

  if [ -f "$old_headless_cfg" ]; then
    mv "$old_headless_cfg" "${old_headless_cfg}.disabled"
  fi

  if [ -f /etc/X11/xorg.conf ] && ! grep -q 'Driver[[:space:]]\+"nvidia"' /etc/X11/xorg.conf; then
    cp -a /etc/X11/xorg.conf /etc/X11/xorg.conf.smartboat.bak
    nvidia-xconfig --allow-empty-initial-configuration || true
  fi
}

start_desktop_services() {
  info "启动并启用开机桌面服务"
  systemctl daemon-reload
  systemctl set-default graphical.target
  if command -v gdm3 >/dev/null 2>&1; then
    ln -sfn /lib/systemd/system/gdm.service /etc/systemd/system/display-manager.service
    printf '/usr/sbin/gdm3\n' > /etc/X11/default-display-manager
    systemctl daemon-reload
    systemctl restart display-manager.service
  fi
  systemctl restart nxserver
}

print_status() {
  info "检查服务状态"
  systemctl --no-pager --full status gdm3 nxserver || true

  info "NoMachine + XFCE 宿主机配置完成"
  printf '请在同一局域网客户端用 NoMachine 连接这台 Jetson 的 IP，用户名使用 %s。\n' "$TARGET_USER"
}

require_root
check_platform
install_desktop
install_nomachine
configure_xfce_session
install_display_session_hook
configure_nomachine
configure_nvidia_xorg_headless
start_desktop_services
print_status
