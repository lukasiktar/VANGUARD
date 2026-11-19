"""OptiTrack ground truth: vector from the drone's base frame to the cross, in base axes.

Two Motive rigid bodies in /rigid_bodies: the drone (Streaming ID `drone_id`, 28) and the
construction (`pole_id`, 6) whose pivot is at the cross center. Publishes the same quantity as
d455_mapping cross_rel_base (the vSLAM side), so the two can be compared directly:

    v = R_drone^T (p_cross - p_pivot) - base_in_pivot          (x forward, y left, z up)

p_pivot, R_drone: the drone's rigid body pose. p_cross = p_pole + R_pole @ cross_in_pole (zero if
the construction's pivot is exactly at the cross center). base_in_pivot: the PX4 body origin (what
the mapper calls base_link) in the drone rigid body's axes, from the pivot [m]. Both rigid bodies
are assumed to have FLU axes (x forward, y left, z up), as mocap_to_px4_node assumes.

    ros2 run vanguard cross_rel_gt --ros-args -p base_in_pivot_xyz:="[-0.005, -0.009, 0.004]"

Publishes /cross_rel/gt (geometry_msgs/Vector3Stamped, stamp = the Motive frame's stamp) and
/cross_rel/gt_distance (std_msgs/Float64). Does not talk to PX4.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from mocap4r2_msgs.msg import RigidBodies
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


def quat_xyzw_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cross_in_base(p_drone, R_drone, p_pole, R_pole, base_in_pivot, cross_in_pole):
    """Vector from the base origin to the cross, in base axes (see module doc)."""
    p_cross = p_pole + R_pole @ cross_in_pole
    return R_drone.T @ (p_cross - p_drone) - base_in_pivot


def valid(p, q):
    n = math.sqrt(sum(v * v for v in q))
    return all(math.isfinite(v) for v in (*p, *q)) and abs(n - 1.0) <= 0.01


class CrossRelGt(Node):

    def __init__(self):
        super().__init__('cross_rel_gt')
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.drone_id = str(self.declare_parameter('drone_id', '28', dyn).value)
        self.pole_id = str(self.declare_parameter('pole_id', '6', dyn).value)
        self.base_in_pivot = np.array(
            self.declare_parameter('base_in_pivot_xyz', [0.0, 0.0, 0.0], dyn).value, float)
        self.cross_in_pole = np.array(
            self.declare_parameter('cross_in_pole_xyz', [0.0, 0.0, 0.0], dyn).value, float)

        self.pole = None            # last valid pose of the construction (it is static)
        self.last_drone = None      # to catch a frozen pose (tracking lost)
        self.count = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.vec_pub = self.create_publisher(Vector3Stamped, '/cross_rel/gt', 10)
        self.dist_pub = self.create_publisher(Float64, '/cross_rel/gt_distance', 10)
        self.create_subscription(RigidBodies, '/rigid_bodies', self.callback, qos)
        self.create_timer(2.0, self.watchdog)
        self.get_logger().info(
            f"drone '{self.drone_id}', construction '{self.pole_id}', base in pivot "
            f'{self.base_in_pivot.round(4).tolist()}, cross in construction '
            f'{self.cross_in_pole.round(4).tolist()}')

    def watchdog(self):
        if self.count == 0:
            self.get_logger().warn('nothing published in the last 2 s (both rigid bodies in '
                                   '/rigid_bodies?)', throttle_duration_sec=10.0)
        self.count = 0

    def callback(self, msg):
        bodies = {b.rigid_body_name: b for b in msg.rigidbodies}
        pole = bodies.get(self.pole_id)
        if pole is not None:
            p, o = pole.pose.position, pole.pose.orientation
            if valid((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)):
                if self.pole is None:
                    self.get_logger().info(f'construction found at Motive [{p.x:.3f} {p.y:.3f} '
                                           f'{p.z:.3f}]')
                self.pole = (np.array([p.x, p.y, p.z]),
                             quat_xyzw_to_matrix(o.x, o.y, o.z, o.w))
        drone = bodies.get(self.drone_id)
        if drone is None or self.pole is None:
            self.get_logger().warn(
                f"rigid bodies in the message: {sorted(bodies)}; need '{self.drone_id}' and "
                f"'{self.pole_id}'", throttle_duration_sec=5.0)
            return
        p, o = drone.pose.position, drone.pose.orientation
        sample = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        if not valid(sample[:3], sample[3:]):
            return
        if sample == self.last_drone:           # bit-identical: Motive repeats a lost pose
            return
        self.last_drone = sample

        v = cross_in_base(np.array(sample[:3]), quat_xyzw_to_matrix(*sample[3:]),
                          self.pole[0], self.pole[1], self.base_in_pivot, self.cross_in_pole)
        out = Vector3Stamped()
        out.header = msg.header
        out.header.frame_id = 'base_link'
        out.vector.x, out.vector.y, out.vector.z = map(float, v)
        self.vec_pub.publish(out)
        self.dist_pub.publish(Float64(data=float(np.linalg.norm(v))))
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CrossRelGt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
