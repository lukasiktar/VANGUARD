---
description: Create a Docker image + dev setup for a ROS 2 Humble package on a Jetson Orin Nano (source stays local, ROS 2 runs in the container)
argument-hint: <package_name> [mode=cpu|gpu] [build_type=ament_cmake|ament_python]
allowed-tools: Bash(docker:*), Bash(docker compose:*), Bash(id:*), Bash(groups:*), Bash(uname:*), Bash(df:*), Bash(ls:*), Bash(mkdir:*), Bash(chmod:*), Bash(cat:*), Bash(find:*), Bash(test:*), Bash(./ros2-docker.sh:*), Read, Write, Edit, Glob, Grep
---

# Goal

Set up a ROS 2 development environment in the current project on an **NVIDIA Jetson Orin Nano (JetPack 6.x / L4T R36, Ubuntu 22.04, arm64)** where:

- **All source code, build output, install and logs live on the host** (the project directory).
- **ROS 2 only exists inside a Docker container.** The host does not need ROS 2 installed.
- The project directory is bind-mounted into the container at `/ros2_ws`, so edits on the host are instantly visible in the container and `colcon build` output lands back on the host, owned by the host user.

The ROS 2 distro is **Humble** — it is the distro that targets Ubuntu 22.04, which matches JetPack 6's userspace and CUDA libraries. Do not switch to Jazzy unless the user explicitly asks; if they do, warn that GPU mode won't work reliably because Jazzy needs Ubuntu 24.04.

Arguments given: `$ARGUMENTS`

Parse them as:
1. `PKG` – ROS 2 package name (required; snake_case). If missing, look for existing packages in `src/*/package.xml`; if there is exactly one, use it, otherwise ask the user.
2. `MODE` – `cpu` (default) or `gpu`.
   - `cpu`: base image `ros:humble-ros-base` (official, multi-arch arm64, ~1 GB). Good for normal ROS nodes.
   - `gpu`: base image `nvcr.io/nvidia/l4t-jetpack:<L4T_TAG>` (CUDA, cuDNN, TensorRT for Jetson) with ROS 2 Humble installed on top from the ROS apt repo. Needed for CUDA / TensorRT / GPU-accelerated inference. Much bigger (~8–10 GB).
3. `BUILD_TYPE` – `ament_cmake` (default) or `ament_python`.

Derive `IMAGE=<PKG>-ros2:humble-<MODE>` (lowercase, `_` replaced with `-` in the repo part).

# Steps

## 1. Preflight (Jetson specific)
- `uname -m` must be `aarch64`. If not, warn the user that this command targets the Jetson and stop unless they confirm.
- `cat /etc/nv_tegra_release` → read the L4T version, e.g. `# R36 (release), REVISION: 4.0` → `L4T_TAG=r36.4.0`. If it's R35 (JetPack 5, Ubuntu 20.04), tell the user and stop: Humble images still work in cpu mode, but the gpu base tag scheme differs and needs a manual decision.
- `docker --version` and `docker compose version`. If Docker is missing, tell the user to install it (e.g. `sudo apt install nvidia-container` for the NVIDIA container toolkit plus Docker from docs.docker.com) and stop. Don't run `sudo` commands yourself.
- `groups` must include `docker`. If not, tell the user to run `sudo usermod -aG docker $USER` and log out/in, then stop.
- `docker info` must list `nvidia` under Runtimes. If not, tell the user to run `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker` and stop. (Required for gpu mode; in cpu mode just warn and continue without the nvidia runtime line.)
- In gpu mode: verify the base tag exists with `docker manifest inspect nvcr.io/nvidia/l4t-jetpack:<L4T_TAG>`. If it doesn't, try the nearest lower R36 revision (e.g. r36.3.0) and tell the user which one you picked.
- `df -h /var/lib/docker` (or `/`): in gpu mode warn if less than ~15 GB is free — the Orin Nano often runs from an SD card or small NVMe.
- `id -u` and `id -g` → remember them.
- Inspect the current directory (`ls -la`, `find . -maxdepth 3 -name package.xml`). The current directory is the ROS 2 workspace root. Create `src/` if it doesn't exist.
- If any of the files below already exist, show a short diff summary and ask before overwriting. Never delete user source code.

## 2. Create the files

Create exactly these files, substituting `<PKG>`, `<IMAGE>`, `<BASE_IMAGE>` (`ros:humble-ros-base` for cpu, `nvcr.io/nvidia/l4t-jetpack:<L4T_TAG>` for gpu) and `<MODE>`.

### `docker/Dockerfile`
```dockerfile
# syntax=docker/dockerfile:1
ARG BASE_IMAGE=<BASE_IMAGE>

# --- Stage 1: collect only package.xml files so dependency layers are cached
#     and only rebuilt when dependencies change, not when source code changes.
FROM busybox AS deps
COPY src /src
RUN mkdir -p /deps && cd /src && \
    find . -name package.xml -exec sh -c 'mkdir -p "/deps/$(dirname "$1")" && cp "$1" "/deps/$1"' _ {} \;

# --- Stage 2: the ROS 2 dev image (no source code baked in)
FROM ${BASE_IMAGE}
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
SHELL ["/bin/bash", "-c"]

# Install ROS 2 Humble from apt if the base image doesn't have it
# (the official ros:humble image has it; the l4t-jetpack image does not).
RUN if [ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then \
      apt-get update && apt-get install -y --no-install-recommends curl ca-certificates software-properties-common \
      && add-apt-repository -y universe \
      && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}') \
      && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
      && dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb \
      && apt-get update && apt-get install -y --no-install-recommends ros-humble-ros-base \
      && rm -rf /var/lib/apt/lists/*; \
    fi

# Dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        build-essential \
        cmake \
        git \
        sudo \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching the host UID/GID so files created in the
# bind-mounted workspace are owned by the host user. Add it to the
# video group so it can use the GPU and cameras.
RUN if getent passwd ${USER_UID} >/dev/null; then userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)" || true; fi \
 && if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -G video -m -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# Install package dependencies declared in package.xml files
COPY --from=deps /deps /tmp/deps
RUN apt-get update \
 && (rosdep init 2>/dev/null || true) \
 && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install --from-paths /tmp/deps --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
 && rm -rf /tmp/deps /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
RUN rosdep update --rosdistro ${ROS_DISTRO} \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

### `docker/entrypoint.sh`
```bash
#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
exec "$@"
```

### `docker-compose.yml`
In cpu mode without an nvidia runtime available, omit the `runtime: nvidia` line and the two `NVIDIA_*` variables.
```yaml
services:
  ros2:
    build:
      context: .
      dockerfile: docker/Dockerfile
      args:
        BASE_IMAGE: <BASE_IMAGE>
        USER_UID: ${HOST_UID:-1000}
        USER_GID: ${HOST_GID:-1000}
    image: <IMAGE>
    container_name: <PKG>_ros2
    runtime: nvidia            # Jetson GPU access (NVIDIA container runtime)
    network_mode: host         # ROS 2 DDS discovery with other hosts/containers
    ipc: host                  # shared-memory transport
    stdin_open: true
    tty: true
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      - DISPLAY=${DISPLAY:-}
    volumes:
      - ./:/ros2_ws                       # the whole workspace stays on the host
      - /tmp/.X11-unix:/tmp/.X11-unix:rw  # optional: GUI tools (rviz2, rqt)
    # Hardware access — uncomment what your robot needs:
    # devices:
    #   - /dev/video0:/dev/video0         # USB / CSI camera
    #   - /dev/ttyUSB0:/dev/ttyUSB0       # serial (motor controller, lidar)
    #   - /dev/i2c-7:/dev/i2c-7           # I2C on the 40-pin header
    # privileged: true                    # broad access to GPIO etc. (use only if needed)
    command: sleep infinity
```

### `.dockerignore`
```
build/
install/
log/
.git/
**/__pycache__/
```

### `ros2-docker.sh` (workspace root, make it executable)
```bash
#!/usr/bin/env bash
# Helper for the Dockerized ROS 2 workspace on Jetson. Source stays on the host;
# ROS 2 runs only in the container.
set -euo pipefail
cd "$(dirname "$0")"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
SERVICE=ros2

usage() {
  cat <<EOF
Usage: ./ros2-docker.sh <command> [args]
  build            Build (or rebuild) the Docker image
  up               Start the container in the background
  down             Stop and remove the container
  shell            Open an interactive shell in the container
  colcon [args]    Run 'colcon build' in the container (e.g. --packages-select <PKG>)
  run <cmd...>     Run any command in the ROS 2 environment (e.g. run ros2 run <PKG> <node>)
EOF
}

ensure_up() {
  if [ -z "$(docker compose ps -q --status running $SERVICE 2>/dev/null)" ]; then
    docker compose up -d $SERVICE
  fi
}

in_ros() {  # run a command with ROS + workspace overlay sourced
  ensure_up
  local tty_flag=""; [ -t 0 ] && tty_flag="-it"
  docker compose exec $tty_flag $SERVICE bash -lc \
    'source /opt/ros/$ROS_DISTRO/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; '"$*"
}

cmd="${1:-}"; shift || true
case "$cmd" in
  build)  docker compose build "$@" ;;
  up)     docker compose up -d $SERVICE ;;
  down)   docker compose down ;;
  shell)  ensure_up; docker compose exec -it $SERVICE bash ;;
  colcon) in_ros colcon build --symlink-install --parallel-workers 2 "$@" ;;
  run)    in_ros "$@" ;;
  *)      usage; exit 1 ;;
esac
```
(`--parallel-workers 2` keeps the 8 GB Orin Nano from running out of RAM on bigger builds; the user can raise it.)

## 3. Build the image
- `chmod +x ros2-docker.sh docker/entrypoint.sh`
- `./ros2-docker.sh build`
- This can take a while on the Jetson (gpu mode pulls a large base image). If the build fails, read the error, fix the cause (bad rosdep key in a package.xml, wrong L4T tag, network issue) and retry. Don't silence errors.

## 4. Create the package (if it doesn't exist yet)
ROS 2 is not on the host, so create the package **inside the container**; the files appear in `./src` on the host via the bind mount:

- `./ros2-docker.sh run "cd src && ros2 pkg create <PKG> --build-type <BUILD_TYPE> --license Apache-2.0 --node-name <PKG>_node"`

If the package already exists in `src/`, skip this step.
If the package declares dependencies that were not in the image yet, run `./ros2-docker.sh build` again so rosdep installs them.

## 5. Build and verify
- `./ros2-docker.sh colcon --packages-select <PKG>`
- `./ros2-docker.sh run "ros2 pkg list | grep -x <PKG>"` — must print the package name.
- `./ros2-docker.sh run "uname -m"` — must print `aarch64`.
- gpu mode: `./ros2-docker.sh run "ls /usr/local/cuda/lib64/libcudart.so* && cat /etc/nv_tegra_release"` — CUDA must be present.
- Check on the host that `build/`, `install/` and `src/<PKG>` exist and are owned by the current user (`ls -la`).
- If there's a git repo, make sure `.gitignore` contains `build/`, `install/`, `log/` (create or extend it).

## 6. Report
Finish with a short summary:
- files created, mode, base image and image name `<IMAGE>`,
- the everyday workflow:
  - edit code on the host in `src/<PKG>`
  - `./ros2-docker.sh colcon` to build
  - `./ros2-docker.sh run ros2 run <PKG> <PKG>_node` to run
  - `./ros2-docker.sh shell` for an interactive ROS 2 shell
- hardware: uncomment the `devices:` entries in `docker-compose.yml` for cameras, serial or I2C, then `./ros2-docker.sh down && ./ros2-docker.sh up`,
- GUI tools: run `xhost +local:` on the host once, and add `ros-humble-rviz2` (or `ros-humble-desktop`) to the image.
