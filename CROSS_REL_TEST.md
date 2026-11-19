# Cross-to-drone vector test: vSLAM against OptiTrack

Measures the vector from the drone's base frame (`base_link`, the PX4 body origin) to the centre of a
cross on the static OptiTrack construction, in two independent ways, and compares them.

| Side | Source | Topic |
|---|---|---|
| vSLAM | RTAB-Map localization in a prior map + cross detector + pin placed in RViz, `map -> base_link` TF | `/cross_rel/vslam`, `/cross_rel/vslam_distance` |
| Ground truth | OptiTrack: drone rigid body (ID 28) and construction rigid body `VANGUARD_construction` (ID 6) | `/cross_rel/gt`, `/cross_rel/gt_distance` |

Both are `geometry_msgs/Vector3Stamped` in `base_link` axes (x forward, y left, z up).
The distance topics are `std_msgs/Float64`.

## What the nodes compute

- `cross_rel_base` (package `d455_mapping`): reads the pin position from `maps/<map>_pins.json`
  and the TF `map -> base_link`, and publishes `R^T (p_pin - t)`. It publishes only while
  `/rtabmap/localization_pose` variance is below `localization_max_variance` (1.0).
- `cross_rel_gt` (package `vanguard`): `R_drone^T (p_cross - p_drone) - base_in_pivot`, with
  `p_cross = p_pole + R_pole * cross_in_pole`. Both rigid bodies are assumed FLU, as in
  `mocap_to_px4_node`.
  - `base_in_pivot_xyz`: where the PX4 body origin is, in the drone rigid body's axes, from the Motive pivot.
  - `cross_in_pole_xyz`: where the cross is from the construction's pivot (0 if the pivot is at the cross).

## One-off preparation

1. Motive: the construction is a rigid body with Streaming ID 6 and a constant pose. Its pivot should
   sit at the centre of the cross (wand step). If not, put the offset in `cross_in_pole_xyz`.
2. Build once (the new entry points need a colcon build, nothing else needs a Docker image rebuild):

```bash
cd ~/VANGUARD/mapper && ./scripts/build.sh
cd ~/VANGUARD && ./ros2-docker.sh up && ./ros2-docker.sh colcon --packages-select vanguard
```

## Procedure

Replace `<map>` with the map name (here `lab_1_261117`).

### 1. OptiTrack

```bash
# T1
cd ~/VANGUARD && ./ros2-docker.sh run ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
# T2
cd ~/VANGUARD && ./ros2-docker.sh run ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

### 2. Camera, cuVSLAM and localization

```bash
# T3
cd ~/VANGUARD/mapper && ./scripts/dev.sh --detach && ./scripts/usb_perms.sh
./scripts/localize.sh <map> orin_nano --prior-map with_cross_detector:=true with_pins:=true px4_vio:=true
```

`px4_vio:=true` makes `base_link` the PX4 body, using the `px4_fusion.base_to_camera` extrinsic in
`mapper/src/d455_mapping/config/orin_nano.yaml`.

### 3. Pin the cross in RViz (over VNC)

```bash
# Jetson T4
cd ~/VANGUARD/mapper && ./scripts/rviz_vnc.sh start localization_remote
# laptop terminal 1 (keep open)
ssh -L 5901:localhost:5901 drone_EDU
# laptop terminal 2
vncviewer localhost:5901
```

In RViz wait for localization (`loop_closure_id` > 0 and orange crosses visible), choose
**Publish Point** (`P`) and click within 0.8 m of the pole's cross. Remove old pins first if needed:

```bash
cd ~/VANGUARD/mapper && ./scripts/dev.sh -- 'ros2 service call /pins/clear std_srvs/srv/Trigger'
```

### 4. Start the two vector nodes

```bash
# T5: vSLAM side (add -p pin_id:=<id> if the file holds several pins)
cd ~/VANGUARD/mapper && ./scripts/dev.sh -- 'ros2 run d455_mapping cross_rel_base --ros-args -p pins_file:=/maps/<map>_pins.json'

# T6: OptiTrack side
cd ~/VANGUARD && ./ros2-docker.sh run ros2 run vanguard cross_rel_gt --ros-args -p base_in_pivot_xyz:="[0.0,0.0,0.0]"
```

Check that the two distances roughly agree:

```bash
cd ~/VANGUARD && ./ros2-docker.sh run ros2 topic echo /cross_rel/vslam_distance
cd ~/VANGUARD && ./ros2-docker.sh run ros2 topic echo /cross_rel/gt_distance
```

### 5. Record

```bash
cd ~/VANGUARD && ./ros2-docker.sh shell
# in the shell:
cd /ros2_ws && ros2 bag record -s sqlite3 -o bags/cross_test1 /cross_rel/vslam /cross_rel/gt /rtabmap/localization_pose
```

Hold the drone still for about 10 s at several distances (1-5 m) and angles, carry it slowly between
them, and stop with Ctrl-C.

### 6. Extract and analyze

```bash
# in the container shell (copy the script in first if it is not visible:
#   docker cp ~/VANGUARD/compare_cross_rel.py vanguard_ros2:/ros2_ws/)
cd /ros2_ws && python3 compare_cross_rel.py extract bags/cross_test1 bags/cross_test1.npz

# on the host
cd ~/VANGUARD && python3 compare_cross_rel.py analyze bags/cross_test1.npz cross_test1.png [t0 t1]
```

`[t0 t1]` restricts the comparison to seconds since the first vSLAM sample. The analysis interpolates
the OptiTrack vector to the vSLAM stamps, scans a lag of +-0.3 s, and writes `cross_test1.png`
(per-axis traces and errors) and `cross_test1_distance.png` (error against distance).

### 7. Stop

```bash
cd ~/VANGUARD/mapper && ./scripts/rviz_vnc.sh stop
./scripts/stop.sh
```

## Result: bag `cross_test1`

2219 vSLAM samples compared, true distance to the cross 0.74-2.75 m. Error is vSLAM minus OptiTrack in
`base_link` axes, with `base_in_pivot_xyz` left at `[0, 0, 0]`.

| | mean | std | rms | max abs |
|---|---|---|---|---|
| x (forward) | +1.1 cm | 1.6 | 1.9 | 4.2 |
| y (left) | -2.9 cm | 1.7 | 3.4 | 5.9 |
| z (up) | +4.3 cm | 1.4 | 4.5 | 7.9 |
| 3D | +5.8 cm | 1.4 | 6.0 | 9.5 |
| distance | +1.3 cm | 1.1 | 1.7 | 4.3 |

Best lag is -20 ms (rms 6.0 cm there and at 0 ms), so the two stamp clocks agree and timing is not a factor.

| Distance bin | n | 3D rms | bias (x, y, z) |
|---|---|---|---|
| 0-1 m | 187 | 4.4 cm | 3.2, -0.1, 2.9 cm |
| 1-2 m | 1153 | 5.7 cm | 1.7, -2.3, 4.3 cm |
| 2-3 m | 879 | 6.6 cm | -0.1, -4.4, 4.6 cm |

### Reading it

- The error is mostly a constant offset, not noise: the 3D mean is 5.8 cm against a 6.0 cm rms and a
  1.4 cm std. The vSLAM vector is repeatable to about 1.5 cm per axis and sits about 6 cm from OptiTrack.
- The distance (norm) error is only 1.7 cm rms, so the offset is largely a direction offset rather than a range error.
- Candidate causes of the constant part (not yet separated, each is a few cm):
  1. The pin was clicked by hand in RViz (snapped to the detected cross within 0.8 m), so its map position carries the detector's error.
  2. The construction's Motive pivot is not exactly at the cross centre (`cross_in_pole_xyz` left at 0).
  3. `base_in_pivot_xyz` left at 0, while the PX4 body origin is a few mm to cm from the current Motive pivot.
  4. The camera extrinsic `base_to_camera` (xyz was derived with an assumption about the lever-arm bags).
- The y bias grows with distance (-0.1 cm at 0-1 m, -4.4 cm at 2-3 m), which points to a small angular
  error (about 1 degree of yaw between the map and the camera/base orientation) in addition to a fixed offset.

## Next steps

1. Put the construction's pivot exactly on the cross centre with the Motive wand, or measure `cross_in_pole_xyz`.
2. Determine `base_in_pivot_xyz` for the current drone pivot (lever-arm run) and pass it to `cross_rel_gt`.
3. Re-pin the cross (or place several pins and compare) to see how much of the offset is the pin.
4. Repeat the run at 3-5 m, since this bag only reaches 2.75 m, and at several angles to the cross.
