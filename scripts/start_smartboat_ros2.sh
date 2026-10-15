#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="/home/ubuntu2204/smartboat"
CONTAINER_NAME="smartboat_ros2_g39"
SERVICE_NAME="boat_ros2"
WORKDIR_IN_CONTAINER="/workspace/boat_ws"
MODE="launch"

case "${1:-}" in
  "")
    ;;
  --shell)
    MODE="shell"
    ;;
  -h|--help)
    printf '用法：%s [--shell]\n' "$0"
    printf '默认直接启动整船边缘节点；--shell 只进入 ROS 2 容器。\n'
    exit 0
    ;;
  *)
    printf '[ERROR] 未知参数：%s\n' "$1" >&2
    exit 2
    ;;
esac

die() {
  printf '\n[ERROR] %s\n' "$*" >&2
  printf '按回车退出...'
  read -r _ || true
  exit 1
}

info() {
  printf '\n==> %s\n' "$*"
}

remove_conflicting_containers() {
  local ids

  ids="$(docker ps -aq --filter "name=^/${CONTAINER_NAME}$" 2>/dev/null || true)"
  ids="${ids}"$'\n'"$(docker ps -aq --filter "name=_${CONTAINER_NAME}$" 2>/dev/null || true)"

  if [ -n "$(printf '%s\n' "$ids" | sed '/^$/d')" ]; then
    info "删除旧容器和 compose 残留容器，避免名称冲突：$CONTAINER_NAME"
    printf '%s\n' "$ids" | sed '/^$/d' | xargs -r docker rm -f >/dev/null
  fi
}

if ! command -v docker >/dev/null 2>&1; then
  die "没有找到 docker 命令。"
fi

if [ ! -d "$PROJECT_DIR" ]; then
  die "项目目录不存在：$PROJECT_DIR"
fi

cd "$PROJECT_DIR"

if docker compose version >/dev/null 2>&1; then
  COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
  COMPOSE=(docker-compose)
else
  die "没有找到 docker compose 或 docker-compose。"
fi

container_state="$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || true)"
needs_recreate="false"

if [ -n "$container_state" ]; then
  if ! docker inspect -f '{{range .Mounts}}{{println .Destination}}{{end}}' "$CONTAINER_NAME" 2>/dev/null | grep -qx '/host_boot'; then
    needs_recreate="true"
  fi
  if ! docker inspect -f '{{range .Mounts}}{{println .Destination}}{{end}}' "$CONTAINER_NAME" 2>/dev/null | grep -qx '/etc/mavlink-router'; then
    needs_recreate="true"
  fi
  if ! docker inspect -f '{{range .HostConfig.GroupAdd}}{{println .}}{{end}}' "$CONTAINER_NAME" 2>/dev/null | grep -qx 'dialout'; then
    needs_recreate="true"
  fi
  for required_group in 0 104 995; do
    if ! docker inspect -f '{{range .HostConfig.GroupAdd}}{{println .}}{{end}}' "$CONTAINER_NAME" 2>/dev/null | grep -qx "$required_group"; then
      needs_recreate="true"
    fi
  done
fi

case "$container_state" in
  running)
    if [ "$needs_recreate" = "true" ]; then
      info "容器缺少必要挂载或 Jetson GPU 设备组，正在重建：$SERVICE_NAME"
      remove_conflicting_containers
      "${COMPOSE[@]}" up -d --build --force-recreate "$SERVICE_NAME" || die "compose 重建失败。"
    else
      info "容器已经在运行：$CONTAINER_NAME"
    fi
    ;;
  "")
    info "容器还不存在，使用 compose 创建并启动：$SERVICE_NAME"
    "${COMPOSE[@]}" up -d "$SERVICE_NAME" || die "compose 启动失败。"
    ;;
  *)
    if [ "$needs_recreate" = "true" ]; then
      info "容器缺少必要挂载或 Jetson GPU 设备组，正在重建：$SERVICE_NAME"
      remove_conflicting_containers
      "${COMPOSE[@]}" up -d --build --force-recreate "$SERVICE_NAME" || die "compose 重建失败。"
    else
      info "容器当前状态是 $container_state，正在启动：$CONTAINER_NAME"
      docker start "$CONTAINER_NAME" >/dev/null || die "docker start 失败。"
    fi
    ;;
esac

if [ "$MODE" = "shell" ]; then
  info "进入 ROS2 容器。退出容器请执行 exit 或按 Ctrl-D。"
  docker exec -it \
    -w "$WORKDIR_IN_CONTAINER" \
    "$CONTAINER_NAME" \
    bash -lc 'source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; exec bash'
  exit $?
fi

if docker exec "$CONTAINER_NAME" bash -lc \
  "pgrep -af '[r]os2 launch smartboat_mission edge_nodes.launch.py|[m]avlink_autoconnect|[c]md_vel_to_mavlink_node|[a]voidance_executor_node'" \
  >/dev/null; then
  die "检测到已运行的整船/MAVLink/避障进程。请先关闭旧进程，避免重复控制源。"
fi

info "启动整船边缘节点和默认启用的生产避障链。按 Ctrl-C 安全退出。"
docker exec -it \
  -w "$WORKDIR_IN_CONTAINER" \
  "$CONTAINER_NAME" \
  bash -lc 'set -Eeo pipefail; source /opt/ros/humble/setup.bash; test -f install/setup.bash; source install/setup.bash; set -u; exec ros2 launch smartboat_mission edge_nodes.launch.py'
