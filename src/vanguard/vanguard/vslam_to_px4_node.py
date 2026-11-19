"""cuVSLAM (/vslam/odom_px4, from the mapper) -> PX4 /fmu/in/vehicle_visual_odometry.

Used instead of mocap_to_px4_node when flying without OptiTrack. Run one of the two, never both:
they publish to the same PX4 topic. EKF2 fuses it with the optical flow, rangefinder and IMU, and
flies on those alone while cuVSLAM is not tracking (the mapper's vslam_px4 then sends nothing).

Mapper side: `ros2 launch d455_mapping vslam.launch.py platform:=orin_nano px4_vio:=true`, so the
pose is that of base_link = PX4 body (camera mounting px4_fusion.base_to_camera).

Levelling: cuVSLAM's odom frame is its starting pose, NOT level: it is tilted by however the drone
stood when cuVSLAM started. EKF2 takes the vision frame as level, so the tilt is measured from PX4's
roll/pitch (averaged over `level_samples` poses, PX4 taken at the image time) and removed.

Anchoring: cuVSLAM can come back from a tracking loss metres away from where it was (IMU-only
prediction while blind). Sent as is, EKF2 would reset its position to it, the local position would
jump, and a node holding an absolute setpoint (px4_control_node_manual_v3) would fly that jump.
So whenever the stream resumes after a gap of more than `gap_s`, it is re-anchored to continue
from the last pose sent, moved by what PX4 measured over the gap (optical flow + IMU meanwhile).
Only PX4's motion is used, not its absolute estimate: that can be off (a rangefinder blocked by a
hand pulled PX4's height 70 cm down on the bench) and would stay in the stream for good. The
first anchoring has nothing to continue and starts at PX4's pose. Without a PX4 estimate it falls
back to incrementing reset_counter (EKF2 then resets).
After a gap of more than `relevel_gap_s` the tilt is measured again (cuVSLAM may have restarted).

    ros2 run vanguard vslam_to_px4_node
"""
import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from vanguard.px4_odom_bridge import matrix_to_quat_xyzw, quat_wxyz_to_matrix

NAN3 = [float('nan')] * 3

# ROS FLU body in a z-up world -> PX4 FRD body in a z-down world: the same flip on both sides
# (as motive_to_px4 in mocap_to_px4_node: position [x, -y, -z], q [w, x, -y, -z])
FLIP = np.diag([1., -1., -1.])


def yaw_of(m):
    return math.atan2(m[1, 0], m[0, 0])


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def nearest_rotation(m):
    """The rotation matrix closest to m (to average rotations)."""
    u, _, vt = np.linalg.svd(m)
    return u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt


class VslamToPx4(Node):

    def __init__(self):
        super().__init__('vslam_to_px4')

        topic = self.declare_parameter('topic', '/vslam/odom_px4').value
        self.gap_ns = int(float(self.declare_parameter('gap_s', 0.2).value) * 1e9)
        self.relevel_gap_ns = int(float(self.declare_parameter('relevel_gap_s', 2.0).value) * 1e9)
        self.level_samples = int(self.declare_parameter('level_samples', 15).value)
        # cuVSLAM image stamps are this much later than the motion (hand test 2026-11-04: 19 ms)
        self.stamp_offset_ns = int(
            float(self.declare_parameter('stamp_offset_s', 0.019).value) * 1e9)
        self.min_pos_var = float(self.declare_parameter('min_position_variance', 1e-4).value)
        self.min_ang_var = float(self.declare_parameter('min_orientation_variance', 1e-4).value)

        self.px4 = deque(maxlen=200)   # (stamp [ns], R FRD body -> NED, position NED or None)
        self.level = None              # tilt correction, None until measured
        self.level_acc = []
        self.anchor_r = np.eye(3)      # sent pose = anchor_r @ levelled cuVSLAM + anchor_t
        self.anchor_t = np.zeros(3)
        self.anchor_pending = True     # anchor at the next pose sent
        self.last_sent = None          # (R, t) last sent + PX4's (R, p) at its image time
        self.last_rx_ns = None
        self.reset_counter = 0
        self.last_pub_ns = 0
        self.ages_ms = []
        self.sent = 0

        # PX4 /fmu/in subscribers are best-effort
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        # Only the newest pose matters, never queue old ones
        vslam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)
        self.create_subscription(Odometry, topic, self.vslam_callback, vslam_qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.px4_callback, qos_profile_sensor_data)

        self.create_timer(1.0, self.watchdog)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'{topic} -> /fmu/in/vehicle_visual_odometry')

    def warn(self, text):
        self.get_logger().warn(text, throttle_duration_sec=2.0)

    def watchdog(self):
        age = (self.get_clock().now().nanoseconds - self.last_pub_ns) * 1e-9
        if age > 0.5:
            if self.last_rx_ns is None:
                self.warn('No cuVSLAM pose received yet (mapper running with px4_vio:=true?)')
            else:
                self.warn('No cuVSLAM pose sent to PX4 in the last 0.5 s '
                          '(PX4 on optical flow only)')

    def report(self):
        """Rate and latency; the latency is what EKF2_EV_DELAY has to cover."""
        if self.ages_ms:
            delay = float(np.mean(self.ages_ms)) + self.stamp_offset_ns * 1e-6
            self.get_logger().info(
                f'{self.sent / 10.0:.1f} Hz to PX4, camera -> PX4 delay {delay:.0f} ms '
                f'(EKF2_EV_DELAY should be about this)')
        self.ages_ms = []
        self.sent = 0

    def px4_callback(self, msg):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or not all(map(math.isfinite, msg.q)):
            return
        now_ns = self.get_clock().now().nanoseconds
        # With UXRCE_DDS_SYNCT=1 PX4's timestamps arrive in ROS time
        stamp_ns = msg.timestamp * 1000
        if abs(now_ns - stamp_ns) > 1_000_000_000:
            self.get_logger().warn('PX4 timestamps are not in ROS time (UXRCE_DDS_SYNCT?), '
                                   'using the receive time', once=True)
            stamp_ns = now_ns
        p = np.array(msg.position, dtype=float) if all(map(math.isfinite, msg.position)) else None
        self.px4.append((stamp_ns, quat_wxyz_to_matrix(msg.q), p))

    def px4_at(self, t_ns):
        """PX4 estimate (R, p) closest to t_ns, None if there is none within 50 ms."""
        if not self.px4:
            return None
        stamp_ns, r, p = min(self.px4, key=lambda s: abs(s[0] - t_ns))
        return (r, p) if abs(stamp_ns - t_ns) < 50_000_000 else None

    def measure_level(self, r, t_ns):
        """Average the tilt of cuVSLAM's odom frame against PX4's level frame over a few poses."""
        px4 = self.px4_at(t_ns)
        if px4 is None:
            self.warn('No PX4 estimate for the cuVSLAM poses (is the Micro XRCE-DDS agent '
                      'running?): not sending to PX4 yet')
            return
        # odom (FRD) -> NED = Rz(yaw) @ tilt. Only the tilt is removed, the heading stays cuVSLAM's.
        m = px4[0] @ r.T
        self.level_acc.append(rot_z(yaw_of(m)).T @ m)
        if len(self.level_acc) < self.level_samples:
            return
        self.level = nearest_rotation(sum(self.level_acc))
        self.level_acc = []
        roll = math.degrees(math.atan2(self.level[2, 1], self.level[2, 2]))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -self.level[2, 0]))))
        self.get_logger().info(f'cuVSLAM odom frame levelled (tilt roll {roll:+.1f} deg, '
                               f'pitch {pitch:+.1f} deg)')

    def anchor(self, r, t, t_ns):
        """Make the levelled cuVSLAM pose (r, t) at t_ns continue the stream (see module doc)."""
        px4 = self.px4_at(t_ns)
        if px4 is None or px4[1] is None:
            self.reset_counter = (self.reset_counter + 1) % 256
            self.get_logger().warn(f'No PX4 position to anchor cuVSLAM to: PX4 reset '
                                   f'#{self.reset_counter} instead (its position may jump)')
            return
        if self.last_sent is None:
            yaw_new, p_new = yaw_of(px4[0]), px4[1]
        else:
            r_sent, t_sent, r_px4, p_px4 = self.last_sent
            # PX4's motion over the gap, turned from PX4's heading into the stream's
            to_stream = rot_z(yaw_of(r_sent) - yaw_of(r_px4))
            yaw_new = yaw_of(r_sent) + yaw_of(px4[0]) - yaw_of(r_px4)
            p_new = t_sent + to_stream @ (px4[1] - p_px4)
        old_r, old_t = self.anchor_r, self.anchor_t
        self.anchor_r = rot_z(yaw_new - yaw_of(r))
        self.anchor_t = p_new - self.anchor_r @ t
        # How far cuVSLAM moved against that over the gap (the jump kept away from PX4)
        jump = float(np.linalg.norm((old_r @ t + old_t) - p_new))
        turn = math.degrees(abs(math.remainder(yaw_of(old_r @ r) - yaw_new, math.tau)))
        self.get_logger().info(f'cuVSLAM anchored (it was {jump:.2f} m, {turn:.1f} deg off): '
                               f'sending to PX4')

    def vslam_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        motion_ns = stamp_ns - self.stamp_offset_ns

        # A gap means cuVSLAM lost tracking, jumped or restarted: anchor it again
        if self.last_rx_ns is not None:
            gap_ns = now_ns - self.last_rx_ns
            if gap_ns > self.gap_ns:
                self.anchor_pending = True
                self.get_logger().info(f'cuVSLAM back after {gap_ns * 1e-9:.1f} s')
                if gap_ns > self.relevel_gap_ns and self.level is not None:
                    self.level = None
                    self.get_logger().info('Long gap: measuring the tilt again')
        self.last_rx_ns = now_ns
        self.ages_ms.append((now_ns - stamp_ns) * 1e-6)

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        sample = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        q_norm = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
        if not all(math.isfinite(v) for v in sample) or abs(q_norm - 1.0) > 0.01:
            self.warn('Invalid pose from cuVSLAM, not sending to PX4')
            return

        q = [v / q_norm for v in (o.w, o.x, o.y, o.z)]
        r = FLIP @ quat_wxyz_to_matrix(q) @ FLIP
        t = FLIP @ np.array([p.x, p.y, p.z])

        if self.level is None:
            self.measure_level(r, motion_ns)
            return
        r = self.level @ r
        t = self.level @ t
        if self.anchor_pending:
            self.anchor(r, t, motion_ns)
            self.anchor_pending = False
        r = self.anchor_r @ r
        t = self.anchor_r @ t + self.anchor_t
        px4 = self.px4_at(motion_ns)
        if px4 is not None and px4[1] is not None:
            self.last_sent = (r, t, px4[0], px4[1])

        vehicle_odom = VehicleOdometry()

        # 0 -> PX4 stamps the message with its own time on arrival (as mocap_to_px4_node).
        # The whole camera -> PX4 latency then goes into EKF2_EV_DELAY (logged by report()).
        vehicle_odom.timestamp = 0
        vehicle_odom.timestamp_sample = 0

        # FRD world-fixed frame; after anchoring it coincides with PX4's local frame
        vehicle_odom.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        vehicle_odom.position = [float(v) for v in t]
        x, y, z, w = matrix_to_quat_xyzw(r)
        vehicle_odom.q = [float(w), float(x), float(y), float(z)]

        # Velocity comes from the optical flow (cuVSLAM's own twist is smoothed): NaN = not
        # available (0.0 would be read as a real measurement of zero velocity)
        vehicle_odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        vehicle_odom.velocity = NAN3
        vehicle_odom.angular_velocity = NAN3
        vehicle_odom.velocity_variance = NAN3

        # Fixed by the mapper (config px4_vio). EKF2 uses max(these, EKF2_EVP_NOISE^2 /
        # EKF2_EVA_NOISE^2)
        cov = msg.pose.covariance
        vehicle_odom.position_variance = [max(float(cov[7 * k]), self.min_pos_var)
                                          for k in range(3)]
        vehicle_odom.orientation_variance = [max(float(cov[7 * (k + 3)]), self.min_ang_var)
                                             for k in range(3)]

        vehicle_odom.reset_counter = self.reset_counter
        vehicle_odom.quality = 100

        self.odom_pub.publish(vehicle_odom)
        self.last_pub_ns = now_ns
        self.sent += 1


def main(args=None):
    rclpy.init(args=args)

    vslam2px4 = VslamToPx4()

    try:
        rclpy.spin(vslam2px4)
    except KeyboardInterrupt:
        pass

    vslam2px4.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
