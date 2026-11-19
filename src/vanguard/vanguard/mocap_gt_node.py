"""OptiTrack as ground truth only: /rigid_bodies -> /mocap/gt/*, never to PX4.

Use this instead of mocap_to_px4_node when cuVSLAM feeds EKF2 (vslam_to_px4_node) and OptiTrack
only validates. It publishes nothing on /fmu/in, so EKF2 keeps a single external-vision source.

Topics (prefix set by the `prefix` parameter, default /mocap/gt):
  <prefix>/vehicle_odometry  px4_msgs/VehicleOdometry, the same NED/FRD conversion that
                             mocap_to_px4_node sends to PX4, so it compares 1:1 with
                             /fmu/out/vehicle_odometry and /fmu/in/vehicle_visual_odometry.
                             timestamp = ROS time [us] at arrival (PX4 is in ROS time with
                             UXRCE_DDS_SYNCT=1).
  <prefix>/pose              geometry_msgs/PoseStamped, the raw Motive pose (frame 'optitrack').

Same filters as mocap_to_px4_node: NaN / non-unit quaternions and frozen (bit-identical) poses
are dropped, so gaps in the topic mean OptiTrack lost tracking. No rate limit.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies
from px4_msgs.msg import VehicleOdometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from vanguard.mocap_to_px4_node import motive_to_px4

NAN3 = [float('nan')] * 3


class MocapGt(Node):

    def __init__(self):
        super().__init__('mocap_gt')
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.body_id = str(self.declare_parameter('body_id', '28', dyn).value)
        prefix = str(self.declare_parameter('prefix', '/mocap/gt').value).rstrip('/')
        if prefix.startswith('/fmu') or prefix.startswith('fmu'):
            raise SystemExit(f"prefix '{prefix}' would publish to PX4; mocap_gt never does that")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.odom_pub = self.create_publisher(VehicleOdometry, f'{prefix}/vehicle_odometry', 20)
        self.pose_pub = self.create_publisher(PoseStamped, f'{prefix}/pose', 20)
        self.create_subscription(RigidBodies, '/rigid_bodies', self.callback, qos)

        self.last_sample = None
        self.count = 0
        self.create_timer(2.0, self.watchdog)
        self.get_logger().info(f"ground truth of body '{self.body_id}' -> {prefix}/* "
                               '(not sent to PX4)')

    def watchdog(self):
        if self.count == 0:
            self.get_logger().warn(f"no valid pose of body '{self.body_id}' in the last 2 s",
                                   throttle_duration_sec=10.0)
        self.count = 0

    def callback(self, msg):
        body = next((b for b in msg.rigidbodies if b.rigid_body_name == self.body_id), None)
        if body is None:
            names = [b.rigid_body_name for b in msg.rigidbodies]
            self.get_logger().warn(f"rigid body '{self.body_id}' not in message, got {names}",
                                   throttle_duration_sec=5.0)
            return
        p, o = body.pose.position, body.pose.orientation
        sample = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        n = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
        if not all(math.isfinite(v) for v in sample) or abs(n - 1.0) > 0.01:
            return
        if sample == self.last_sample:          # Motive repeats the last pose when tracking is lost
            return
        self.last_sample = sample

        now = self.get_clock().now()
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = now.to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'optitrack'
        pose.pose = body.pose
        self.pose_pub.publish(pose)

        odom = VehicleOdometry()
        odom.timestamp = now.nanoseconds // 1000
        odom.timestamp_sample = stamp.sec * 1_000_000 + stamp.nanosec // 1000
        odom.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        odom.position, odom.q = motive_to_px4(*sample)
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        odom.velocity = NAN3
        odom.angular_velocity = NAN3
        odom.position_variance = NAN3
        odom.orientation_variance = NAN3
        odom.velocity_variance = NAN3
        self.odom_pub.publish(odom)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MocapGt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
