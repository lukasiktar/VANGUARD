#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

// OFFBOARD velocity control from a geometry_msgs/Twist, derived from
// px4_control_node_manual_v3 (same offboard handling, Twist instead of a gamepad).
//
// The Twist is in the vehicle's BODY frame (FRD), heading-only:
//   linear.x  -> velocity forward (m/s, along the nose)
//   linear.y  -> velocity right   (m/s)
//   linear.z  -> velocity down    (m/s, positive = descend)
//   angular.z -> yawspeed         (rad/s, positive = clockwise seen from above)
// angular.x/y are ignored. x/y are rotated into PX4's local frame by the current yaw
// from vehicle_odometry every cycle, so the motion turns with the nose while yawing.
// Roll/pitch are deliberately NOT applied: z stays vertical and tilting the vehicle
// to accelerate doesn't leak horizontal commands into climb/descent.
//
// Arming, takeoff/land and mode switching are all on the RC; this node never sends a
// vehicle command. While in OFFBOARD it flies the vehicle from the Twist; when the
// Twist is all-zero, or stops arriving, it holds position at the point of release.
//
// offboard_control_mode is streamed continuously even outside offboard, because PX4
// only accepts a switch INTO offboard when that stream is already live
// (offboardCheck.cpp: offboard_available = (any flag) && data_is_recent). The
// trajectory_setpoint stream, by contrast, is published ONLY while offboard is
// active: /fmu/in/trajectory_setpoint is bridged onto the same
// ORB_ID(trajectory_setpoint) that PX4's own FlightModeManager publishes to in
// RC-flown modes, and MulticopterPositionControl consumes it with no mode gating, so
// streaming it outside offboard fights the RC's own setpoints (reduced speed + jerky
// motion). offboardCheck never inspects trajectory_setpoint, so staying quiet is free.
class PX4OffboardCmdVel : public rclcpp::Node
{
public:
    PX4OffboardCmdVel() : Node("px4_cmd_vel_control")
    {
        rclcpp::QoS qos_pub(1);
        qos_pub.best_effort().durability_volatile();
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos_pub);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_pub);

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                latest_cmd_vel_ = *msg;
                last_cmd_vel_recv_time_ = this->get_clock()->now();
                has_cmd_vel_ = true;
            });

        // QoS must be best-effort or no samples arrive from PX4's uXRCE-DDS client.
        vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
            {
                vehicle_odometry_callback(msg);
            });

        // vehicle_status rather than vehicle_control_mode: see manual_v3 -- this
        // workspace's px4_msgs VehicleControlMode doesn't match the firmware's and
        // fails to deserialize. Rate-limited to 5Hz, so expect up to ~200ms after the
        // RC selects OFFBOARD before setpoints start flowing (PX4 holds xy meanwhile).
        vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
            {
                offboard_active_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
            });

        setpoint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]()
            {
                publish_offboard_setpoint();
            });

        RCLCPP_INFO(this->get_logger(), "PX4 Offboard cmd_vel node started!");
        RCLCPP_INFO(this->get_logger(),
            "/cmd_vel is body FRD: linear.x=forward, linear.y=right, linear.z=down (m/s), angular.z=yaw rate (rad/s, CW+). "
            "All-zero or stale Twist = hold position.");
        RCLCPP_INFO(this->get_logger(),
            "Setpoints are sent only while in OFFBOARD; arm/takeoff/land/mode switching are on the RC.");
    }

private:
    // Command limits; anything beyond is clamped per axis.
    static constexpr float MAX_XY_VELOCITY = 5.0f;  // m/s
    static constexpr float MAX_Z_VELOCITY = 2.0f;   // m/s
    static constexpr float MAX_YAW_RATE = 1.0f;     // rad/s
    // A Twist with every used component below this counts as "stop" (hold position).
    // Small because the input is software, not a stick that rests slightly off 0.
    static constexpr float CMD_DEADBAND = 1e-3f;
    // Vehicle odometry older than this is stale -- fall back to zero velocity rather
    // than hold at a position we can no longer trust.
    static constexpr float ODOM_TIMEOUT_SEC = 0.5f;
    // Twist older than this is treated as "stop", so a crashed or hung publisher
    // parks the vehicle instead of flying on at its last commanded velocity forever.
    // PX4's own offboard-loss failsafe would NOT catch that, since our setpoint
    // stream keeps flowing -- just with stale contents. The publisher must therefore
    // stream continuously (> 2Hz) while it wants the vehicle to keep moving.
    static constexpr float CMD_VEL_TIMEOUT_SEC = 0.5f;

    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Setpoints go out in the local NED frame, so a non-NED odometry frame would
        // mean the yaw and hold position are in a different frame than the velocities.
        if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED)
        {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "vehicle_odometry pose_frame is %u, not POSE_FRAME_NED", msg->pose_frame);
        }

        odom_position_ = msg->position;
        // q = [w, x, y, z], FRD body -> local NED. Yaw about local down (CW+ from
        // above). A NaN q gives a NaN yaw, which publish_offboard_setpoint rejects.
        const float qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];
        odom_yaw_ = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
        // Node clock on receipt, not msg->timestamp -- PX4 time and ROS time are
        // different clocks, so comparing a staleness window needs the former.
        last_odom_recv_time_ = this->get_clock()->now();
        has_odom_ = true;
    }

    void publish_offboard_setpoint()
    {
        const rclcpp::Time now = this->get_clock()->now();
        const uint64_t now_us = now.nanoseconds() / 1000;

        px4_msgs::msg::OffboardControlMode mode_msg{};
        mode_msg.timestamp = now_us;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.thrust_and_torque = false;
        mode_msg.direct_actuator = false;

        px4_msgs::msg::TrajectorySetpoint setpoint_msg{};
        setpoint_msg.timestamp = now_us;
        setpoint_msg.acceleration = {NAN, NAN, NAN};
        setpoint_msg.jerk = {NAN, NAN, NAN};
        // yaw is always NaN: PX4's PositionControl then substitutes the current yaw
        // and uses yawspeed as the rate, i.e. pure yaw-rate control.
        setpoint_msg.yaw = NAN;
        setpoint_msg.yawspeed = NAN;

        // Not our mode: heartbeat only, no setpoint (see file header). Hold state is
        // reset so the next offboard entry captures a fresh hold point.
        if (!offboard_active_)
        {
            mode_msg.position = false;
            mode_msg.velocity = true;

            hold_position_ = {NAN, NAN, NAN};
            was_cmd_idle_ = false;

            offboard_control_mode_publisher_->publish(mode_msg);
            return;
        }

        const bool odom_valid = has_odom_ && (now - last_odom_recv_time_).seconds() <= ODOM_TIMEOUT_SEC;

        if (!odom_valid)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "vehicle_odometry missing/stale -- commanding zero velocity, no position hold");

            mode_msg.position = false;
            mode_msg.velocity = true;
            setpoint_msg.position = {NAN, NAN, NAN};
            setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};

            // Force a fresh hold capture once odometry returns.
            was_cmd_idle_ = false;

            offboard_control_mode_publisher_->publish(mode_msg);
            trajectory_setpoint_publisher_->publish(setpoint_msg);
            return;
        }

        const bool cmd_fresh =
            has_cmd_vel_ && (now - last_cmd_vel_recv_time_).seconds() <= CMD_VEL_TIMEOUT_SEC;

        if (has_cmd_vel_ && !cmd_fresh)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "/cmd_vel missing/stale -- holding position");
        }

        // Body FRD (see file header).
        const float vx = cmd_fresh ? static_cast<float>(latest_cmd_vel_.linear.x) : 0.0f;
        const float vy = cmd_fresh ? static_cast<float>(latest_cmd_vel_.linear.y) : 0.0f;
        const float vz = cmd_fresh ? static_cast<float>(latest_cmd_vel_.linear.z) : 0.0f;
        const float yaw_rate = cmd_fresh ? static_cast<float>(latest_cmd_vel_.angular.z) : 0.0f;

        // NaN/inf in the Twist must never reach PX4 as a velocity: a NaN velocity
        // field would silently switch that axis's control type.
        const bool cmd_finite =
            std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz) && std::isfinite(yaw_rate);
        if (!cmd_finite)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "/cmd_vel contains NaN/inf -- holding position");
        }

        // Without a heading the body->local rotation is unknown, so hold instead.
        const bool yaw_valid = std::isfinite(odom_yaw_);
        if (!yaw_valid)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "vehicle_odometry attitude invalid (NaN yaw) -- holding position");
        }

        const bool cmd_idle =
            !cmd_finite || !yaw_valid
            || (std::fabs(vx) < CMD_DEADBAND
                && std::fabs(vy) < CMD_DEADBAND
                && std::fabs(vz) < CMD_DEADBAND
                && std::fabs(yaw_rate) < CMD_DEADBAND);

        if (cmd_idle)
        {
            if (!was_cmd_idle_)
            {
                // Moving -> idle: capture the hold point once, at the moment of release.
                hold_position_ = odom_position_;
            }

            mode_msg.position = true;
            mode_msg.velocity = false;
            setpoint_msg.position = hold_position_;
            setpoint_msg.velocity = {NAN, NAN, NAN};
        }
        else
        {
            if (was_cmd_idle_)
            {
                // Idle -> moving: drop the captured point so a stale one can't leak
                // back in (fails visibly as NaN rather than silently as an old value).
                hold_position_ = {NAN, NAN, NAN};
            }

            mode_msg.position = false;
            mode_msg.velocity = true;
            setpoint_msg.position = {NAN, NAN, NAN};
            // Clamp in the body frame, then rotate forward/right into North/East by
            // the current yaw. Down is the same axis in both frames.
            const float bx = std::clamp(vx, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
            const float by = std::clamp(vy, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
            const float cos_yaw = std::cos(odom_yaw_);
            const float sin_yaw = std::sin(odom_yaw_);
            setpoint_msg.velocity = {
                cos_yaw * bx - sin_yaw * by,
                sin_yaw * bx + cos_yaw * by,
                std::clamp(vz, -MAX_Z_VELOCITY, MAX_Z_VELOCITY)};
            setpoint_msg.yawspeed = std::clamp(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
        }

        was_cmd_idle_ = cmd_idle;

        offboard_control_mode_publisher_->publish(mode_msg);
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

    // Single-threaded executor (see main()): plain members are safe without a mutex
    // since the callbacks and the timer never run concurrently with each other.
    geometry_msgs::msg::Twist latest_cmd_vel_;
    bool has_cmd_vel_ = false;
    rclcpp::Time last_cmd_vel_recv_time_;

    // Defaults false so that until vehicle_status is ever received, the node only
    // emits the neutral heartbeat and never commands motion or position hold.
    bool offboard_active_ = false;

    bool has_odom_ = false;
    rclcpp::Time last_odom_recv_time_;
    std::array<float, 3> odom_position_{};
    float odom_yaw_ = NAN;

    std::array<float, 3> hold_position_{NAN, NAN, NAN};
    // Sentinel false (not "idle") so the first idle frame after entering offboard
    // still counts as a moving->idle transition and captures a hold point.
    bool was_cmd_idle_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4OffboardCmdVel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
