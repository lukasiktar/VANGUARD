# vg: one launcher for the whole stack

`./vg.py` (on the Jetson, in `~/VANGUARD`) starts and stops everything in the README / `mapper` /
`CROSS_REL_TEST.md` procedures. Each stage runs detached in its container with its own log in
`.vg/logs/<stage>.log`, so it survives an SSH drop. All settings (map name, body IDs, offsets,
launch flags, recorded topics) are in [vg.yaml](vg.yaml). The existing scripts are only called.

```bash
ssh drone_EDU
cd ~/VANGUARD
./vg.py modes                     # optitrack | mapping | localization | validation
./vg.py up validation             # everything for phase 3
./vg.py status
./vg.py check                     # topic rates, who publishes to PX4, vSLAM vs OptiTrack distance
./vg.py down                      # stop all (./vg.py down --containers also stops the mapper container)
```

| Mode | Stages | EKF2 source |
|---|---|---|
| `optitrack` | agent, optitrack (+ lifecycle activate), mocap_to_px4 | OptiTrack (`param_OPTITRACK_*`) |
| `mapping` | agent, mapper (dev.sh --detach + usb_perms), map, vslam_to_px4, rviz | cuVSLAM (`param_CUVSLAM_*`) |
| `localization` | agent, mapper, localize, vslam_to_px4, rviz | cuVSLAM |
| `validation` | agent, optitrack, mocap_gt, cross_rel_gt, mapper, localize (with pins), vslam_to_px4, cross_rel_base, rviz | cuVSLAM; OptiTrack is only ground truth |

`mocap_to_px4` and `vslam_to_px4` both publish `/fmu/in/vehicle_visual_odometry`; `vg` refuses
to run them together, and before starting either it checks that no other node publishes there.
In `validation`, OptiTrack goes through the new `mocap_gt_node` instead: it publishes
`/mocap/gt/vehicle_odometry` (same NED/FRD conversion as `mocap_to_px4_node`, comparable with
`/fmu/out/vehicle_odometry`) and `/mocap/gt/pose` (raw Motive), and never anything on `/fmu`.

## Changing things per run

```bash
./vg.py up mapping --map lab_2_261120                    # new map name (refuses an existing one)
./vg.py up localization --map lab_01_261105 --set mapper.prior_map=false
./vg.py up validation --set optitrack.base_in_pivot_xyz=[0.01,0,-0.03]
./vg.py config validation                                # show the resolved settings
./vg.py up validation -n                                 # dry run: print the commands only
./vg.py maps                                             # saved maps, with crosses / pins files
```

Single stages work too: `./vg.py up mapper` (re-runs `usb_perms.sh`, e.g. after replugging the
camera), `./vg.py up rviz`, `./vg.py stop localize`, `./vg.py logs localize -f`.
Stopping `map` waits up to 90 s, because RTAB-Map writes the database on SIGINT.

## Validation run (CROSS_REL_TEST.md with vg)

```bash
./vg.py up validation          # cross_rel_base is skipped if maps/<map>_pins.json is missing:
                            # pin the cross in RViz (VNC), then ./vg.py up cross_rel_base
./vg.py check                  # both distances should roughly agree
./vg.py record cross_test2     # topics: record.topics in vg.yaml
./vg.py stop record
./vg.py analyze cross_test2    # compare_cross_rel.py extract + analyze -> bags/cross_test2*.png
```

RViz is on the VNC desktop as before: `ssh -L 5901:localhost:5901 drone_EDU`, then
`vncviewer localhost:5901`.

Own stages go under `custom_stages` in `vg.yaml` (container `ros2` or `mapper`, `{map}` and
`{section.key}` are filled in) and can be added to any mode.

Build once after pulling (new `mocap_gt_node` entry point):
`./ros2-docker.sh colcon --packages-select vanguard`
