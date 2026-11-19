"""PX4 /fmu/out/vehicle_odometry -> nav_msgs/Odometry /px4/odom (ROS frames) for the mapper's EKF.

The mapper container has no px4_msgs, so the conversion runs here and the mapper only sees a
standard message. PX4 NED / FRD -> ROS ENU / FLU; pose in `odom_frame`, twist in `base_frame`
(body), as robot_localization expects. No TF: the mapper's EKF publishes odom -> base_link.

Stamped with this node's clock on receipt: PX4's timestamp is PX4 boot time, a different clock,
while the camera is stamped with the same host clock as this node.

Resets: when EKF2 resets its estimate (reset_counter changes, e.g. mocap lost / regained) the pose
jumps. With smooth_resets the jump is folded into a fixed correction so /px4/odom stays continuous
(same as mapper/src/d455_mapping/d455_mapping/px4_bag_odom.py, which does this for bags).

    ros2 run vanguard px4_odom_bridge
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# NED -> ENU and FRD -> FLU basis changes (R_enu_flu = A @ R_ned_frd @ B)
A_ENU_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
B_FRD_FLU = np.diag([1., -1., -1.])

# Variance for a value PX4 marks invalid (NaN): large enough that the EKF ignores it
UNKNOWN_VAR = 1e6


def quat_wxyz_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat_xyzw(m):
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def finite(values):
    return all(math.isfinite(v) for v in values)


def variances(values, floor):
    """PX4 variances -> EKF variances: NaN = unknown, and never below `floor` (a zero would be
    taken as infinitely certain)."""
    return [max(float(v), floor) if math.isfinite(v) else UNKNOWN_VAR for v in values]


class Px4OdomBridge(Node):

    def __init__(self):
        super().__init__('px4_odom_bridge')
        topic = self.declare_parameter('topic', '/fmu/out/vehicle_odometry').value
        odom_topic = self.declare_parameter('odom_topic', '/px4/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.offset_ns = int(float(self.declare_parameter('time_offset_s', 0.0).value) * 1e9)
        self.smooth = bool(self.declare_parameter('smooth_resets', True).value)
        self.min_pos_var = float(self.declare_parameter('min_position_variance', 1e-4).value)
        self.min_ang_var = float(self.declare_parameter('min_orientation_variance', 1e-4).value)
        self.min_vel_var = float(self.declare_parameter('min_velocity_variance', 1e-3).value)
        self.min_rate_var = float(self.declare_parameter('min_angular_velocity_variance', 1e-3).value)

        # Reset correction: published pose = rc @ pose + tc
        self.rc, self.tc = np.eye(3), np.zeros(3)
        self.last = None            # last published (R, t)
        self.last_counter = None
        self.count = 0

        self.pub = self.create_publisher(Odometry, odom_topic, 20)
        # QoS must be best-effort or no samples arrive from PX4's uXRCE-DDS client.
        self.create_subscription(VehicleOdometry, topic, self.callback, qos_profile_sensor_data)
        self.create_timer(2.0, self.watchdog)
        self.get_logger().info(f'{topic} -> {odom_topic} ({self.odom_frame} -> {self.base_frame})')

    def watchdog(self):
        if self.count == 0:
            self.get_logger().warn('No usable vehicle_odometry received yet '
                                   '(is the Micro XRCE-DDS agent running?)',
                                   throttle_duration_sec=10.0)
        self.count = 0

    def callback(self, msg):
        # Without an orientation neither the pose nor the body-frame velocity can be given.
        if not finite(msg.q) or msg.pose_frame not in (VehicleOdometry.POSE_FRAME_NED,
                                                       VehicleOdometry.POSE_FRAME_FRD):
            self.get_logger().warn(f'vehicle_odometry without a valid attitude '
                                   f'(pose_frame={msg.pose_frame}), skipped',
                                   throttle_duration_sec=2.0)
            return
        self.count += 1

        r = A_ENU_NED @ quat_wxyz_to_matrix(msg.q) @ B_FRD_FLU
        pos_valid = finite(msg.position)
        t = A_ENU_NED @ np.asarray(msg.position, dtype=float) if pos_valid else np.zeros(3)

        # A reset is folded in at the first sample with a valid position after it.
        if pos_valid:
            if self.smooth and self.last is not None and msg.reset_counter != self.last_counter:
                jump_m = float(np.linalg.norm(self.last[1] - (self.rc @ t + self.tc)))
                self.rc = self.last[0] @ r.T
                self.tc = self.last[1] - self.rc @ t
                self.get_logger().info(f'PX4 estimator reset #{msg.reset_counter}: '
                                       f'jump of {jump_m:.2f} m removed')
            self.last_counter = msg.reset_counter
        r_pub, t_pub = self.rc @ r, self.rc @ t + self.tc
        if pos_valid:
            self.last = (r_pub, t_pub)

        # Body-frame velocity. The reset correction rotates world and attitude alike, so it
        # cancels out here and the uncorrected r is used.
        vel_valid = finite(msg.velocity)
        if not vel_valid:
            v = np.zeros(3)
        elif msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            v = B_FRD_FLU @ np.asarray(msg.velocity, dtype=float)
        elif msg.velocity_frame in (VehicleOdometry.VELOCITY_FRAME_NED,
                                    VehicleOdometry.VELOCITY_FRAME_FRD):
            v = r.T @ (A_ENU_NED @ np.asarray(msg.velocity, dtype=float))
        else:
            v, vel_valid = np.zeros(3), False
        rate_valid = finite(msg.angular_velocity)
        w = B_FRD_FLU @ np.asarray(msg.angular_velocity, dtype=float) if rate_valid else np.zeros(3)

        od = Odometry()
        stamp_ns = self.get_clock().now().nanoseconds + self.offset_ns
        od.header.stamp.sec, od.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        p = od.pose.pose
        p.position.x, p.position.y, p.position.z = map(float, t_pub)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(
            float, matrix_to_quat_xyzw(r_pub))
        tw = od.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = map(float, v)
        tw.angular.x, tw.angular.y, tw.angular.z = map(float, w)

        # Diagonal covariances. Position NED -> ENU swaps the horizontal axes. Orientation and
        # velocity variances are in frames that differ from ours, so the larger of the two
        # horizontal values is used for both (they are nearly equal in practice).
        pv = variances(msg.position_variance, self.min_pos_var) if pos_valid else [UNKNOWN_VAR] * 3
        pv = [pv[1], pv[0], pv[2]]
        ov = variances(msg.orientation_variance, self.min_ang_var)
        ov = [max(ov[0], ov[1])] * 2 + [ov[2]]
        vv = variances(msg.velocity_variance, self.min_vel_var) if vel_valid else [UNKNOWN_VAR] * 3
        vv = [max(vv[0], vv[1])] * 2 + [vv[2]]
        wv = [self.min_rate_var if rate_valid else UNKNOWN_VAR] * 3
        for k in range(3):
            od.pose.covariance[7 * k] = pv[k]
            od.pose.covariance[7 * (k + 3)] = ov[k]
            od.twist.covariance[7 * k] = vv[k]
            od.twist.covariance[7 * (k + 3)] = wv[k]
        self.pub.publish(od)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
