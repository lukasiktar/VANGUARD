#!/usr/bin/env python3
"""Print a PX4 VehicleOdometry topic as x/y/z [m] and roll/pitch/yaw [deg].

What the bridge sends to PX4 (works without the flight controller):
    python3 check_odom.py /fmu/in/vehicle_visual_odometry
What PX4's EKF2 estimates after fusing it:
    python3 check_odom.py /fmu/out/vehicle_odometry
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleOdometry


def quat_to_euler_deg(w, x, y, z):
    """Hamilton quaternion (body FRD -> world) to roll, pitch, yaw in degrees."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class CheckOdom(Node):

    def __init__(self, topic):
        super().__init__('check_odom')
        self.msg = None
        self.count = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleOdometry, topic, self.callback, qos)
        self.create_timer(0.2, self.show)
        self.get_logger().info(f'Listening on {topic}')

    def callback(self, msg):
        self.msg = msg
        self.count += 1

    def show(self):
        rate = self.count / 0.2
        self.count = 0
        if self.msg is None:
            return
        x, y, z = self.msg.position
        roll, pitch, yaw = quat_to_euler_deg(*self.msg.q)
        print(f'x {x:+7.3f}  y {y:+7.3f}  z {z:+7.3f} m   |   '
              f'roll {roll:+7.1f}  pitch {pitch:+7.1f}  yaw {yaw:+7.1f} deg   |   '
              f'{rate:4.0f} Hz', flush=True)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/fmu/in/vehicle_visual_odometry'
    rclpy.init()
    node = CheckOdom(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()