# VANGUARD EDU 450 Drone setup + VSLAM setup

### Connecting to the HEXSON EDU 450 over SSH

```bash
ssh drone_EDU   
# password: vanguard1
# the alias can be different - depnds on settings on the host computer for SSH connection
```
# 1. Starting ROS2 drone Docker container
Then start the ROS 2 Docker shell:

```bash
cd ~/VANGUARD
bash ros2-docker.sh shell
```
### ⚠️ Shut down the Jetson before unplugging it from the power supply
```bash
sudo shutdown now
```


### Repository layout

Two containers on the same host network and ROS domain, so all topics are visible in both:

| Path | Container | What |
|---|---|---|
| `src/` | `vanguard_ros2` (`./ros2-docker.sh`) | PX4 link (Micro XRCE Agent, `px4_msgs`), OptiTrack (`mocap4r2*`), `vanguard`, `px4_control` |
| `mapper/` | `isaac_ros_dev-aarch64-d455-container` (`mapper/scripts/dev.sh`) | D455 mapping, localization, cross detection, RViz over VNC. |


Clone with the submodules (`mapper`, `px4_msgs`, `mocap4r2`, `mocap4r2_msgs`):

```bash
git clone --recurse-submodules <this repo> ~/VANGUARD
cd ~/VANGUARD/mapper && ./scripts/setup_workspace.sh    # fetches Isaac ROS sources, writes the container config
```

After pulling changes, build both workspaces once (each in its own container; not while flying):

```bash
cd ~/VANGUARD/mapper && ./scripts/build.sh                            # mapper (d455_mapping)
cd ~/VANGUARD && ./ros2-docker.sh colcon --packages-select vanguard     # vanguard nodes
```

### PX4 parameters: one set per odometry source

| Odometry to PX4 | Node (never both) | PX4 parameter file (QGC on the laptop: Parameters -> Tools -> Load from file) |
|---|---|---|
| OptiTrack | `mocap_to_px4_node` | `param_OPTITRACK_i_Optical_rade.params` (`EKF2_EV_DELAY = 30`; a copy is in `~/` on the Jetson) |
| cuVSLAM (D455) | `vslam_to_px4_node` (T4) | the same set with `EKF2_EV_DELAY = 90`; save it as its own file |


## T1: Running Micro Agent Communication inside Docker container

The first terminal starts the micro agent that translates the messages from ROS to PX4 and opposite.

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

### (Optional) Connecting Jetson Orin Nano to Optitrack.

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

If the Motive PC answers `ping 192.168.0.149` but you still get `... not connected :(`, check the Motive streaming settings (Data Streaming pane): **Local Interface** must be `192.168.0.149`, not `loopback` or the camera network's Ethernet adapter. The Transmission Type must be  `Unicast` .It often resets after a Motive PC reboot or WiFi reconnect. After fixing it, stop the launch (Ctrl+C) and start it again, because the driver only tries to connect once at startup.

Also enable VRPN and broadcast port set to `1510` .

Note: the node reaches the `inactive` state and prints `Configured!` even when the connection failed, so look for `... connected!` in the output.

Activate your lifecycle node:
```bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```
The output is: 
```bash
[mocap4r2_optitrack_driver_node]: Activated!
```
Test with:
```bash
ros2 topic echo /rigid_bodies
```

Converting OptiTrack /rigid_bodies to PX4 visual_odometry_input (OptiTrack parameter set; not
together with `vslam_to_px4_node`, both publish to the same PX4 topic):
```bash
ros2 run vanguard mocap_to_px4_node
```

# 2. Mapper and odometry container

Mapper and odometry container enables cuVSLAM odometry for drone and mapping and localization using cuVSLAM + RTAB-Map

## T2: Before you start (Jetson, every session)

Start the container and load camera access prelims.

```bash
ssh drone_EDU
cd ~/VANGUARD/mapper
./scripts/dev.sh --detach        # start the container (takes a while after a reboot / image change)
./scripts/usb_perms.sh           # camera access inside the container. Redo after a container
                                 # restart or after unplugging / replugging the camera
```

Optional check that the camera is seen (serial 260722303845, firmware 5.15.1.55):

```bash
./scripts/dev.sh -- 'rs-enumerate-devices -s'
```


## T2: Setup RViz VNC for streaming on host device

On the Jetson SSH:

#### 1. IF you are MAPPING:
```bash
cd ~/VANGUARD/mapper
./scripts/rviz_vnc.sh start mapping_remote        # RViz on a virtual desktop 
```

#### 2. IF you are LOCALIZING:
```bash
cd ~/VANGUARD/mapper
./scripts/rviz_vnc.sh start localization_remote        # RViz on a virtual desktop 
```

While flying use the lighter `light_remote` instead (RViz draws on the Jetson's CPU; watch that the
T4 delay stays around 90-110 ms).

On your HOST (laptop), not on the Jetson, two terminals (the first stays open):

T1:

```bash
ssh -L 5901:localhost:5901 drone_EDU
```

T2:

```bash
vncviewer localhost:5901
```

Blank screen / not connecting: `./scripts/rviz_vnc.sh status`, then `./scripts/rviz_vnc.sh restart <config>`
on the Jetson. If `/tmp/xvnc.log` in the container says `Address already in use`, the `ssh -L ...`
command was run on the Jetson itself: close it there.

## T3: Mapping / Localizing

#### 1. IF you are MAPPING:

```bash
cd ~/VANGUARD/mapper
./scripts/map.sh <map_name> orin_nano        # new name each time; add --force to overwrite

./scripts/map.sh <map_name> orin_nano with_cross_detector:=true px4_vio:=true  # with_cross_detector:=true and px4_vio:=true are optional
```

- `px4_vio:=true` on the drone: `base_link` = PX4 body and cuVSLAM is passed on for PX4 (T4).
- Keep the drone/camera still for about 30 s at the start (warm-up; with `px4_vio` T4 also levels
  cuVSLAM's frame then), move slowly, and finish where you started (same view) so a loop closes.
- Stop and save: **Ctrl-C once and wait until the prompt returns** (up to a minute).

Check progress from another terminal:

```bash
./scripts/dev.sh -- 'ros2 topic echo /rtabmap/info --field loop_closure_id'    # > 0 = loop closure
./scripts/dev.sh -- 'ros2 topic echo /visual_slam/status --once | grep vo_state'   # 1 = tracking OK
./scripts/map_info.sh <map_name> 
```

#### 2. IF you are LOCALIZING:

```bash
cd ~/VANGUARD/mapper
./scripts/localize.sh <map_name> orin_nano --prior-map with_cross_detector:=true px4_vio:=true   # --prior-map shows the mapped point cloud
```

- Use the same `px4_vio` setting the map was made with (`base_link` differs).
- Localized = `loop_closure_id` > 0. RViz keeps drawing the drone at its last known place even when
  RTAB-Map is lost; to be sure: `./scripts/dev.sh -- 'ros2 topic echo /rtabmap/localization_pose --field pose.covariance --once' | head -1`
  (9999.0 = not localized, a small number = localized). The cross detector only fuses while localized.

#### 3. Only odometry for PX4 (no map), INSTEAD of 1 or 2:

1, 2 and 3 each start the camera + cuVSLAM themselves (and with `px4_vio:=true` the feed for T4):
run only one of them at a time, never together (two cuVSLAMs give "cuVSLAM jump" in a loop; the
launch now refuses a second one).

```bash
cd ~/VANGUARD/mapper
./scripts/dev.sh -- 'ros2 launch d455_mapping vslam.launch.py platform:=orin_nano px4_vio:=true'
```

## T4: Start VSLAM to PX4 odometry node

Sends cuVSLAM to PX4's EKF2 as external vision, instead of OptiTrack; EKF2 fuses it with the H-Flow
(optical flow + rangefinder) and the IMU. While cuVSLAM is not tracking nothing is sent and PX4 flies
on the optical flow; when it comes back it continues where it left off, so a cuVSLAM jump never
reaches PX4. Needs: T3 running with `px4_vio:=true`, the cuVSLAM PX4 parameter set
(`EKF2_EV_DELAY = 90`), and NOT `mocap_to_px4_node`. Drone still on the ground when cuVSLAM starts.

```bash
cd ~/VANGUARD
./ros2-docker.sh run ros2 run vanguard vslam_to_px4_node
```

It logs `cuVSLAM odom frame levelled ...`, `cuVSLAM anchored ...`, then every 10 s
`15.0 Hz to PX4, camera -> PX4 delay ~90 ms` (~105 ms while mapping). It handles a restart of T3 by
itself (keep the drone still for a moment).

## Log
What EKF2 fuses, PX4's estimate next to what T4 sends, rangefinder and reset counters, one line per
second (listens only; a new file name per flight, Ctrl-C saves it). Before arming: `EV pos/yaw/hgt YYY`
and PX4 / "sent" within a few cm.
```bash
cd ~/VANGUARD
./ros2-docker.sh run python3 check_vio.py /ros2_ws/bags/<flight_name>.npz
```


## Extend a map (new flight in the old map's coordinates)
Copies `flight_map1` to `flight_map2` (the old map stays as it is) and maps into the copy. Start
where `flight_map1` was mapped: the new flight joins the old map at its first loop closure (the
cross detector logs `joined the old map`, before that it does not fuse).

```bash
cd ~/VANGUARD/mapper
./scripts/extend_map.sh flight_map1 flight_map2 orin_nano with_cross_detector:=true px4_vio:=true

./scripts/map_info.sh flight_map2      # after Ctrl-C: want "sessions joined"

./scripts/localize.sh flight_map2 orin_nano --prior-map with_cross_detector:=true px4_vio:=true
```

## Stop everything

```bash
# Ctrl-C once in the map.sh / localize.sh / extend_map.sh terminal and wait (saves the map), Ctrl-C in T4, then:
cd ~/VANGUARD/mapper
./scripts/rviz_vnc.sh stop
./scripts/stop.sh                  # stops all mapper launches; RTAB-Map saves its database first
```

Don't build or run other heavy jobs on the Jetson while flying: a starved cuVSLAM loses frames.

# DEPRICATED:

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
docker exec -it vanguard_ros2 bash -ic "colcon build --symlink-install --parallel-workers 2 --base-paths src"
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

