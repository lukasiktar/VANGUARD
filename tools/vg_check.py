#!/usr/bin/env python3
"""Snapshot for ./vg check (runs in vanguard_ros2): topic rates, who feeds PX4, cross_rel agreement.

    python3 tools/vg_check.py [-d seconds] topic...
Listens only. Topics whose message type is not built in this container are reported as such.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64

PX4_IN = '/fmu/in/vehicle_visual_odometry'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-d', '--duration', type=float, default=3.0)
    ap.add_argument('topics', nargs='+')
    a = ap.parse_args()

    rclpy.init()
    node = Node('vg_check')
    time.sleep(1.0)                                   # discovery
    types = dict(node.get_topic_names_and_types())
    counts, notes = {}, {}
    last = {}

    def counter(t):
        def cb(_msg):
            counts[t] += 1
        return cb

    for t in a.topics:
        counts[t] = 0
        if t not in types:
            notes[t] = 'no such topic'
            continue
        try:
            cls = get_message(types[t][0])
        except (AttributeError, ModuleNotFoundError, ValueError):
            notes[t] = f'{types[t][0]} not built here'
            continue
        node.create_subscription(cls, t, counter(t), qos_profile_sensor_data)

    for t in ('/cross_rel/vslam_distance', '/cross_rel/gt_distance'):
        if t in types:
            node.create_subscription(Float64, t, lambda m, t=t: last.__setitem__(t, m.data),
                                     qos_profile_sensor_data)

    t0 = time.monotonic()
    while time.monotonic() - t0 < a.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    dt = time.monotonic() - t0

    print(f'\n  topic rates over {dt:.1f} s')
    for t in a.topics:
        pubs = len(node.get_publishers_info_by_topic(t))
        if t in notes:
            print(f'    {t:<40} {"":>8}   {notes[t]}')
        else:
            hz = counts[t] / dt
            flag = '' if hz > 0 else '   <- silent'
            print(f'    {t:<40} {hz:7.1f} Hz  ({pubs} pub){flag}')

    pubs = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
            for i in node.get_publishers_info_by_topic(PX4_IN)]
    print(f'\n  {PX4_IN} publishers: {", ".join(pubs) or "none"}')
    if len(pubs) > 1:
        print('    !! more than one source feeds EKF2 external vision: stop all but one')

    if last:
        v = last.get('/cross_rel/vslam_distance')
        g = last.get('/cross_rel/gt_distance')
        if v is not None and g is not None:
            print(f'\n  cross distance: vSLAM {v:.3f} m, OptiTrack {g:.3f} m, '
                  f'diff {100 * (v - g):+.1f} cm')
        else:
            print(f'\n  cross distance: vSLAM {v}, OptiTrack {g} (one side silent)')
    print()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
