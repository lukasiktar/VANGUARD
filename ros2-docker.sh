#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export HOST_UID="$(id -u)" HOST_GID="$(getent passwd "$(id -u)" | cut -d: -f4)"

PKG=vanguard
IMAGE=vanguard-ros2:humble-gpu
CONTAINER=vanguard_ros2
BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
DEVICES=()
USE_COMPOSE="${USE_COMPOSE:-0}"

usage() {
  cat <<EOF
Usage: ./ros2-docker.sh <command> [args]
  build            Build (or rebuild) the Docker image
  up               Start the container in the background
  down             Stop and remove the container
  shell            Open an interactive shell in the container
  colcon [args]    Run 'colcon build' in the container
  run <cmd...>     Run any command in the ROS 2 environment

Environment:
  USE_COMPOSE=1    Use docker compose (service '$PKG') instead of plain docker
EOF
}

d_build() {
  docker build -f docker/Dockerfile -t "$IMAGE" \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --build-arg USER_UID="$HOST_UID" --build-arg USER_GID="$HOST_GID" "$@" .
}

d_up() {
  if [ -n "$(docker ps -aq -f name="^${CONTAINER}$")" ]; then
    docker start "$CONTAINER" >/dev/null
    return
  fi
  local dev_args=() d
  for d in "${DEVICES[@]}"; do dev_args+=(--device "$d:$d"); done
  docker run -d -it --name "$CONTAINER" \
    --runtime nvidia \
    --network host --ipc host \
    -v /dev/input:/dev/input \
    -v /run/udev:/run/udev:ro \
    --device-cgroup-rule='c 13:* rmw' \
    --group-add "$(getent group input | cut -d: -f3)" \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e DISPLAY="${DISPLAY:-}" \
    -v "$PWD":/ros2_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --privileged -v /dev:/dev \
    "${dev_args[@]}" \
    "$IMAGE" sleep infinity >/dev/null
}

d_down() { docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; }

ensure_up() {
  if [ "$USE_COMPOSE" = 1 ]; then
    if [ -z "$(docker compose ps -q --status running "$PKG" 2>/dev/null)" ]; then
      docker compose up -d "$PKG"
    fi
  else
    [ -n "$(docker ps -q -f name="^${CONTAINER}$")" ] || d_up
  fi
}

exec_in() {
  local flags=()
  while [ "$1" != "--" ]; do flags+=("$1"); shift; done; shift
  if [ "$USE_COMPOSE" = 1 ]; then
    docker compose exec "${flags[@]}" "$PKG" "$@"
  else
    docker exec "${flags[@]}" -w /ros2_ws "$CONTAINER" "$@"
  fi
}

in_ros() {
  ensure_up
  local tty_flag=""; [ -t 0 ] && tty_flag="-it"
  exec_in $tty_flag -- bash -lc \
    'source /opt/ros/$ROS_DISTRO/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; '"$*"
}

cmd="${1:-}"; shift || true
case "$cmd" in
  build)  if [ "$USE_COMPOSE" = 1 ]; then docker compose build "$@"; else d_build "$@"; fi ;;
  up)     if [ "$USE_COMPOSE" = 1 ]; then docker compose up -d "$PKG"; else d_up; fi ;;
  down)   if [ "$USE_COMPOSE" = 1 ]; then docker compose down; else d_down; fi ;;
  shell)  ensure_up; exec_in -it -- bash ;;
  # --base-paths src: mapper/ (submodule) has its own workspace and container
  colcon) in_ros colcon build --symlink-install --parallel-workers 2 --base-paths src "$@" ;;
  run)    in_ros "$@" ;;
  *)      usage; exit 1 ;;
esac
