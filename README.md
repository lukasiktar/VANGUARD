# VANGUARD

## ⚠️ Shut down the Jetson before unplugging it from the power supply
```bash
sudo shutdown now
```

### Connecting to the HEXSON EDU 450 over SSH

```bash
ssh drone_EDU
# password: vanguard1
```

Then start the ROS 2 Docker shell:

```bash
cd VANGUARD
bash ros2-docker.sh shell
```

### Repository layout

Two containers on the same host network and ROS domain, so all topics are visible in both:

| Path | Container | What |
|---|---|---|
| `src/` | `vanguard_ros2` (`./ros2-docker.sh`) | PX4 link (Micro XRCE Agent, `px4_msgs`), OptiTrack (`mocap4r2*`), `vanguard`, `px4_control` |
| `mapper/` | `isaac_ros_dev-aarch64-d455-container` (`mapper/scripts/dev.sh`) | D455 mapping, localization, cross detection, RViz over VNC. Own repo ([04_Vertical_mapper](https://github.com/lukasiktar/04_Vertical_mapper)), see [mapper/JETSON_QUICKSTART.md](mapper/JETSON_QUICKSTART.md) |

The mapper owns the D455; nothing in `vanguard_ros2` opens the camera.

Clone with the submodules (`mapper`, `px4_msgs`, `mocap4r2`, `mocap4r2_msgs`):

```bash
git clone --recurse-submodules <this repo> ~/VANGUARD
cd ~/VANGUARD/mapper && ./scripts/setup_workspace.sh    # fetches Isaac ROS sources, writes the container config
```

`src/mocap4ros2_optitrack` is vendored (not a submodule): upstream
[mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack) @ `7b92128` with
local changes (aarch64 `libNatNet.so` and headers, install of the NatNet library, params).

Maps (`mapper/maps/`) and bags are not in git. Back them up separately.

### Running Micro Agent Communication
```bash
MicroXRCEAgent udp4 -p 8888
```
Output should be something like:
```bash
...
[1791210413.717594] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x06E(2), participant_id: 0x001(1)
[1791210413.717719] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x00000001, publisher_id: 0x06E(3), participant_id: 0x001(1)
[1791210413.718087] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x00000001, datawriter_id: 0x06E(5), publisher_id: 0x06E(3)
[1791210413.720305] info     | ProxyClient.cpp    | create_replier           | replier created        | client_key: 0x00000001, requester_id: 0x800(7), participant_id: 0x001(1)
```

### Connecting Jetson Orin Nano to Optitrack.

Check IP adress of Motive PC and that NatNet is enabled.
Run this on Jetson Nano Orin inside Docker Container:
```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```

You should see something like this:
```bash
[mocap4r2_optitrack_driver_node]: Trying to connect to Optitrack NatNET SDK at 192.168.0.149 ...
[mocap4r2_optitrack_driver_node]: ... connected!
 [mocap4r2_optitrack_driver_node]: Application: Motive (ver. 3.0.1.1)
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_node]: NatNet Version: 4.0.0.0
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_node]: Client IP:192.168.0.10
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_node]: Server IP:192.168.0.149
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_node]: Server Name:
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_node]: Mocap Framerate : 120.00
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_node]: Configured!
```
If you don't then check if Motive PC is up and connected to Vanguard WiFi and also check for ip adress....it is configured as 192.168.0.149.

Activate your lifecycle node:
```bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```
Ant output is: 
```bash
[mocap4r2_optitrack_driver_node]: Activated!
```
Test with:
```bash
ros2 topic echo /rigid_bodies
```

Converting OptiTrack /rigid_bodies to PX4 visual_odometry_input
```bash
ros2 run vanguard mocap_to_px4_node
```


ROS 2 Humble workspace for Jetson Orin Nano, running in Docker. Full setup: [SETUP.md](SETUP.md).

## First time

```bash
cd ~/VANGUARD
./ros2-docker.sh build
./ros2-docker.sh up
./ros2-docker.sh colcon
```

## Run

```bash
./ros2-docker.sh run ros2 run vanguard vanguard_node
```

## Develop

```bash
./ros2-docker.sh colcon
./ros2-docker.sh colcon --packages-select vanguard
./ros2-docker.sh shell
./ros2-docker.sh run <cmd...>
```

## Container

```bash
./ros2-docker.sh up
./ros2-docker.sh down
./ros2-docker.sh build
```

## Check GPU

```bash
./ros2-docker.sh run "nvcc --version"
./ros2-docker.sh run "python3 -c \"import ctypes; c=ctypes.CDLL('libcuda.so.1'); n=ctypes.c_int(); c.cuInit(0); c.cuDeviceGetCount(ctypes.byref(n)); print('CUDA devices:', n.value)\""
```

## Plain Docker

Build image:

```bash
cd ~/VANGUARD
docker build -f docker/Dockerfile -t vanguard-ros2:humble-gpu \
  --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0 \
  --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .
```

Create and start container:

```bash
docker run -d -it --name vanguard_ros2 --runtime nvidia --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v ~/VANGUARD:/ros2_ws -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  vanguard-ros2:humble-gpu sleep infinity
```

Use container:

```bash
docker exec -it vanguard_ros2 bash
docker exec -it vanguard_ros2 bash -ic "colcon build --symlink-install --parallel-workers 2"
docker exec -it vanguard_ros2 bash -ic "ros2 run vanguard vanguard_node"
```

Manage container:

```bash
docker ps -a
docker stop vanguard_ros2
docker start vanguard_ros2
docker restart vanguard_ros2
docker logs vanguard_ros2
docker rm -f vanguard_ros2
```

One-shot container (removed on exit):

```bash
docker run --rm -it --runtime nvidia --network host --ipc host \
  -v ~/VANGUARD:/ros2_ws vanguard-ros2:humble-gpu
```

Images:

```bash
docker images
docker rmi vanguard-ros2:humble-gpu
docker system df
docker image prune
```

## Docker Compose

```bash
USE_COMPOSE=1 ./ros2-docker.sh build
USE_COMPOSE=1 ./ros2-docker.sh up
USE_COMPOSE=1 ./ros2-docker.sh shell
USE_COMPOSE=1 ./ros2-docker.sh down
```

## Joy node start
```bash
ros2 run joy joy_node
```

## Joystick control
```bash
ros2 run px4_control px4_control_node_manual_v2
```


## CMD_VEL node
```bash
ros2 run px4_control px4_control_node_cmd_vel
```

## Mapping, localization, crosses (D455)
See [mapper/JETSON_QUICKSTART.md](mapper/JETSON_QUICKSTART.md). In short, from `~/VANGUARD/mapper`:
```bash
./scripts/dev.sh --detach && ./scripts/usb_perms.sh
./scripts/map.sh <name> orin_nano with_cross_detector:=true
./scripts/localize.sh <name> orin_nano --prior-map with_cross_detector:=true
./scripts/rviz_vnc.sh restart light_remote
```

