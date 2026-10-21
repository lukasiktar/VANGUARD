import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)    
from mocap4r2_msgs.msg import RigidBodies
from px4_msgs.msg import VehicleOdometry

NAN3 = [float('nan')] * 3


def motive_to_px4(px, py, pz, ox, oy, oz, ow):
    #Thit is done based on rigid body orientation.
    n = math.sqrt(ox * ox + oy * oy + oz * oz + ow * ow)
    position = [px, -py, -pz]
    q = [ow / n, ox / n, -oy / n, -oz / n]  # [w, x, y, z]
    return position, q


class MocapToPx4(Node):

    def __init__(self):
        super().__init__('mocap_to_px4')

        # The driver publishes the Motive *Streaming ID* as a string in
        # rigid_body_name, so 28 and "28" in the YAML are both accepted here.
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('body_id', '28', dyn)
        self.declare_parameter('max_rate_hz', 50.0)  # PX4 wants 30-50 Hz

        self.body_id = str(self.get_parameter('body_id').value)
        max_rate = float(self.get_parameter('max_rate_hz').value)
        self.min_period_ns = int(1e9 / max_rate) if max_rate > 0.0 else 0

        self.get_logger().info(
            f"Tracking object for PX4 with body ID: '{self.body_id}'")

        self.last_sample = None      # last pose received (frozen detection)
        self.frozen_count = 0
        self.last_pub_ns = 0

        # PX4 /fmu/in subscribers are best-effort
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        # Only the newest mocap frame matters, never queue old ones
        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)
        self.mocap_sub = self.create_subscription(
            RigidBodies, '/rigid_bodies', self.mocap_callback, mocap_qos)

        self.create_timer(1.0, self.watchdog)

    def warn(self, text):
        self.get_logger().warn(text, throttle_duration_sec=2.0)

    def watchdog(self):
        age = (self.get_clock().now().nanoseconds - self.last_pub_ns) * 1e-9
        if age > 0.5:
            self.warn('No valid mocap pose sent to PX4 in the last 0.5 s')

    def mocap_callback(self, msg):
        # Find our body. Never publish anything if it is not in the frame.
        body = next((b for b in msg.rigidbodies
                     if b.rigid_body_name == self.body_id), None)
        if body is None:
            names = [b.rigid_body_name for b in msg.rigidbodies]
            self.warn(f"Rigid body '{self.body_id}' not in message, got {names}")
            return

        p = body.pose.position
        o = body.pose.orientation
        sample = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)

        # Reject NaN and the all-zero / non-unit quaternion of an untracked body
        q_norm = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
        if not all(math.isfinite(v) for v in sample) or abs(q_norm - 1.0) > 0.01:
            self.warn('Invalid pose from mocap, not sending to PX4')
            return

        # The driver does not forward Motive's "tracking valid" flag. A pose
        # that is bit-identical to the previous frame means tracking is lost
        # and Motive is repeating the last pose: do not feed that to the EKF.
        if sample == self.last_sample:
            self.frozen_count += 1
            if self.frozen_count >= 3:
                self.warn('Mocap pose is frozen (tracking lost?), not sending')
            return
        self.last_sample = sample
        self.frozen_count = 0

        # Rate limit
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_pub_ns < self.min_period_ns:
            return
        self.last_pub_ns = now_ns

        vehicle_odom = VehicleOdometry()

        # 0 -> PX4 stamps the message with its own time on arrival.
        # The whole mocap -> PX4 latency then goes into EKF2_EV_DELAY.
        vehicle_odom.timestamp = 0
        vehicle_odom.timestamp_sample = 0

        # FRD world-fixed frame, arbitrary heading reference
        vehicle_odom.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        vehicle_odom.position, vehicle_odom.q = motive_to_px4(*sample)

        # Mocap gives no velocity: NaN = "not available" (0.0 would be read
        # as a real measurement of zero velocity)
        vehicle_odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        vehicle_odom.velocity = NAN3
        vehicle_odom.angular_velocity = NAN3
        vehicle_odom.velocity_variance = NAN3

        # EKF2 uses max(these, EKF2_EVP_NOISE^2 / EKF2_EVA_NOISE^2)
        vehicle_odom.position_variance = [1e-4, 1e-4, 1e-4]       # (1 cm)^2
        vehicle_odom.orientation_variance = [1e-3, 1e-3, 1e-3]    # (~1.8 deg)^2

        self.odom_pub.publish(vehicle_odom)


def main(args=None):
    rclpy.init(args=args)

    mocap2px4 = MocapToPx4()

    try:
        rclpy.spin(mocap2px4)
    except KeyboardInterrupt:
        pass

    mocap2px4.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()