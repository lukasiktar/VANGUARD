# VANGUARD – ROS 2 Humble on Jetson Orin Nano (Docker)

| Item        | Value                                  |
|-------------|----------------------------------------|
| Hardware    | Jetson Orin Nano, JetPack 6 (L4T R36.5.2), Ubuntu 22.04, arm64 |
| ROS 2       | Humble (inside Docker only)            |
| Package     | `vanguard` (`ament_python`)            |
| Base image  | `nvcr.io/nvidia/l4t-jetpack:r36.4.0`   |
| Image       | `vanguard-ros2:humble-gpu`             |
| Container   | `vanguard_ros2`                        |
| Workspace   | host `~/VANGUARD` → container `/ros2_ws` |

## 1. Host prerequisites

[NVIDIA Docker setup for Jetson](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html)

```bash
sudo apt update
sudo apt install -y nvidia-container
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo usermod -aG docker $USER
```

Log out and back in, then verify:

```bash
groups | grep docker
docker info | grep -i runtimes
docker run --rm --runtime nvidia ubuntu:22.04 ls /dev | grep nvhost
```

Optional, CUDA toolkit on the host (not needed for the container):

```bash
sudo apt install -y nvidia-jetpack
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
nvcc --version
```

## 2. Project layout

```
VANGUARD/
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── docker-compose.yml
├── .dockerignore
├── ros2-docker.sh
├── SETUP.md
└── src/
    └── vanguard/
```

`build/`, `install/` and `log/` are created on the host by `colcon`.

## 3. Build the image

```bash
cd ~/VANGUARD
./ros2-docker.sh build
```

Rebuild after adding dependencies to any `package.xml`.

## 4. Create the package (already done)

```bash
./ros2-docker.sh run "cd src && ros2 pkg create vanguard --build-type ament_python --license Apache-2.0 --node-name vanguard_node"
```

## 5. Build and verify

```bash
./ros2-docker.sh colcon
./ros2-docker.sh run "ros2 pkg list | grep -x vanguard"
./ros2-docker.sh run ros2 run vanguard vanguard_node
./ros2-docker.sh run "nvcc --version"
./ros2-docker.sh run "python3 -c \"import ctypes; c=ctypes.CDLL('libcuda.so.1'); n=ctypes.c_int(); c.cuInit(0); c.cuDeviceGetCount(ctypes.byref(n)); print('CUDA devices:', n.value)\""
```

Expected: `vanguard`, `Hi from vanguard.`, `cuda_12.6`, `CUDA devices: 1`.

## 6. Commands

| Command                                  | Action                              |
|------------------------------------------|-------------------------------------|
| `./ros2-docker.sh build`                 | Build the image                     |
| `./ros2-docker.sh up`                    | Start the container                 |
| `./ros2-docker.sh down`                  | Stop and remove the container       |
| `./ros2-docker.sh shell`                 | Interactive ROS 2 shell             |
| `./ros2-docker.sh colcon [args]`         | `colcon build --symlink-install --parallel-workers 2` |
| `./ros2-docker.sh run <cmd...>`          | Run a command in the ROS 2 environment |

Edit code on the host in `src/vanguard`.

## 7. Plain Docker vs. Docker Compose

By default `ros2-docker.sh` uses plain `docker` (standalone container, no Compose needed).
Settings live at the top of `ros2-docker.sh` (`PKG`, `IMAGE`, `CONTAINER`, `BASE_IMAGE`, `DEVICES`).

To use Docker Compose instead (service `vanguard` in `docker-compose.yml`):

```bash
USE_COMPOSE=1 ./ros2-docker.sh build
USE_COMPOSE=1 ./ros2-docker.sh shell
```

Run `./ros2-docker.sh down` before switching modes, since both use the container name `vanguard_ros2`.

Plain Docker equivalent:

```bash
docker build -f docker/Dockerfile -t vanguard-ros2:humble-gpu \
  --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0 \
  --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .

docker run -d -it --name vanguard_ros2 --runtime nvidia --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v ~/VANGUARD:/ros2_ws -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  vanguard-ros2:humble-gpu sleep infinity

docker exec -it vanguard_ros2 bash
```

## 8. Adding more ROS 2 packages later

Each package gets its own service in `docker-compose.yml`, all on `network_mode: host` with the same `ROS_DOMAIN_ID` so they discover each other:

```yaml
services:
  vanguard:
    ...
  vanguard_perception:
    build:
      context: .
      dockerfile: docker/Dockerfile
      args:
        BASE_IMAGE: nvcr.io/nvidia/l4t-jetpack:r36.4.0
        USER_UID: ${HOST_UID:-1000}
        USER_GID: ${HOST_GID:-1000}
    image: vanguard-perception-ros2:humble-gpu
    container_name: vanguard_perception_ros2
    runtime: nvidia
    network_mode: host
    ipc: host
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    volumes:
      - ./:/ros2_ws
    command: bash -lc "colcon build --packages-select vanguard_perception && ros2 run vanguard_perception <node>"
```

Then `docker compose up -d` starts all of them.

## 9. Hardware and GUI

Devices (camera, serial, I2C):

- plain docker: `DEVICES=(/dev/video0 /dev/ttyUSB0 /dev/i2c-7)` in `ros2-docker.sh`
- compose: add under the service
  ```yaml
  devices:
    - /dev/video0:/dev/video0
  ```

Then `./ros2-docker.sh down && ./ros2-docker.sh up`.

GUI tools (rviz2, rqt): run `xhost +local:` on the host, add `ros-humble-rviz2` to the `apt-get install` list in `docker/Dockerfile`, then rebuild.
