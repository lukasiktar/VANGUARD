#!/usr/bin/env python3
"""Watch cuVSLAM -> PX4 live: one line per second until Ctrl-C, optionally saved to .npz.

What EKF2 fuses (EV pos/yaw/hgt, flow, range, rejections), PX4's estimate next to what
vslam_to_px4_node sends, the rangefinder and the reset counters. Listens only.
    python3 check_vio.py [out.npz]
"""
import math
import sys
import time

import numpy as np
import rclpy
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_deg(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class Rec(Node):

    def __init__(self):
        super().__init__('check_vio')
        q = qos_profile_sensor_data
        self.flags = None
        self.lpos = None
        self.px4, self.sent, self.flag_log, self.lpos_log = [], [], [], []
        self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
                                 self.on_flags, q)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self.on_lpos, q)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_px4, q)
        self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
                                 self.on_sent, q)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_flags(self, m):
        self.flags = m
        self.flag_log.append([self.now(), m.cs_ev_pos, m.cs_ev_yaw, m.cs_ev_hgt, m.cs_opt_flow,
                              m.cs_rng_hgt, m.cs_baro_hgt, m.reject_hor_pos, m.reject_ver_pos,
                              m.reject_yaw])

    def on_lpos(self, m):
        self.lpos = m
        self.lpos_log.append([self.now(), m.dist_bottom, m.dist_bottom_valid,
                              m.xy_reset_counter, m.z_reset_counter, m.heading_reset_counter])

    def on_px4(self, m):
        self.px4.append([self.now(), *m.position, yaw_deg(m.q)])

    def on_sent(self, m):
        self.sent.append([self.now(), *m.position, yaw_deg(m.q), m.reset_counter])

    def status(self):
        now = self.now()
        f = self.flags

        def yn(b):
            return 'Y' if b else '-'
        fs = (f'EV pos/yaw/hgt {yn(f.cs_ev_pos)}{yn(f.cs_ev_yaw)}{yn(f.cs_ev_hgt)} '
              f'flow {yn(f.cs_opt_flow)} rng {yn(f.cs_rng_hgt)}'
              + (' REJECT' if (f.reject_hor_pos or f.reject_ver_pos or f.reject_yaw) else '')
              if f else 'no estimator flags')
        p = self.px4[-1] if self.px4 else None
        ps = f'PX4 x {p[1]:+.2f} y {p[2]:+.2f} z {p[3]:+.2f} yaw {p[4]:+6.1f}' if p else 'PX4: -'
        s = self.sent[-1] if self.sent else None
        ss = (f'sent x {s[1]:+.2f} y {s[2]:+.2f} z {s[3]:+.2f}' if s and now - s[0] < 0.5
              else 'sent: NOTHING (cuVSLAM not tracking?)')
        rs = (f'range {self.lpos.dist_bottom:.2f} m resets xy {self.lpos.xy_reset_counter} '
              f'z {self.lpos.z_reset_counter}' if self.lpos else '')
        return f'{fs} | {ps} | {ss} | {rs}'


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else None
    rclpy.init()
    node = Rec()
    print('Ctrl-C to stop.', flush=True)
    t0 = time.time()
    next_print = t0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.time() >= next_print:
                print(f'{time.time() - t0:6.0f} s | {node.status()}', flush=True)
                next_print += 1.0
    except KeyboardInterrupt:
        pass
    finally:
        if out:
            np.savez(out, px4=np.array(node.px4), sent=np.array(node.sent),
                     flags=np.array(node.flag_log, dtype=float),
                     lpos=np.array(node.lpos_log, dtype=float))
            print(f'\nSaved {out}: px4 {len(node.px4)}, sent {len(node.sent)}, '
                  f'flags {len(node.flag_log)}, lpos {len(node.lpos_log)}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
