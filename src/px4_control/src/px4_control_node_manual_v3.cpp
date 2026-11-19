#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include <px4_msgs/srv/vehicle_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

// OFFBOARD-only joystick velocity control, derived from px4_control_node_manual_v2.
//
// Unlike v2 this node publishes NO manual_control_input: arming, takeoff/land and
// mode switching are done on the RC, which stays the sole manual-control source.
// This node's entire job is: while the vehicle is in OFFBOARD, fly it from the
// gamepad's left stick.
//
// The one exception is X (Cross), which requests OFFBOARD -- there is no RC in
// simulation, so without it there'd be no way to get into offboard on the sim bench.
// On real hardware you'd normally use the RC switch instead and can just not press it.
//
// Left stick is body-frame horizontal velocity (Y=forward, X=right) rotated into NED
// using live vehicle heading. Altitude is held at whatever it was on offboard entry;
// yaw is left alone. Releasing the stick holds position rather than drifting.
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
class PX4OffboardJoystick : public rclcpp::Node
{
public:
    PX4OffboardJoystick() : Node("px4_manual_control_v3")
    {
        vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

        rclcpp::QoS qos_pub(1);
        qos_pub.best_effort().durability_volatile();
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos_pub);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_pub);

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg)
            {
                joy_callback(msg);
            });

        // QoS must be best-effort or no samples arrive from PX4's uXRCE-DDS client.
        vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
            {
                vehicle_odometry_callback(msg);
            });

        // Whether the RC currently has us in OFFBOARD. vehicle_status (not the more
        // direct vehicle_control_mode.flag_control_offboard_enabled) because this
        // workspace's px4_msgs 1.17.0 VehicleControlMode is missing
        // flag_control_acceleration_enabled, flag_control_termination_enabled and
        // source_id vs the firmware's, so that topic fails to deserialize entirely.
        // VehicleStatus is byte-identical to the firmware's and actually works.
        //
        // It's rate-limited to 5Hz (dds_topics.yaml), so expect up to ~200ms after the
        // RC selects OFFBOARD before setpoints start flowing. PX4 fills that gap with
        // generateFailsafeSetpoint ("don't move along xy"), so it's a brief hold, not a
        // lurch. Raise that rate_limit in the firmware if it's ever too slow.
        vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
            {
                const bool active = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
                if (active != offboard_active_)
                {
                    RCLCPP_INFO(this->get_logger(), "%s OFFBOARD (nav_state=%u)",
                        active ? "Entered" : "Left", msg->nav_state);
                }
                offboard_active_ = active;
            });

        setpoint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]()
            {
                publish_offboard_setpoint();
            });

        RCLCPP_INFO(this->get_logger(), "PX4 Offboard Joystick node (v3) started!");
        RCLCPP_INFO(this->get_logger(),
            "Left stick = body-frame velocity (up/down=forward/back, left/right=left/right). "
            "Release to hold position. Altitude held at offboard entry, yaw untouched.");
        RCLCPP_INFO(this->get_logger(),
            "X (Cross) = request OFFBOARD mode (for the sim bench; use the RC switch on hardware). "
            "Arm/takeoff/land/other modes are all on the RC; this node sends no manual_control_input.");
    }

private:
    // Max horizontal speed at full stick deflection, m/s. The left stick is analog
    // (unlike v2's digital D-pad), so everything between 0 and this is reachable.
    static constexpr float MAX_XY_VELOCITY = 5.0f;
    // Stick magnitude below which input counts as released (hold position). Larger
    // than v2's D-pad value because an analog stick rarely rests at exactly 0 and
    // any residual offset would otherwise creep the vehicle. Tune to your stick.
    static constexpr float STICK_DEADBAND = 0.10f;
    // Vehicle odometry older than this is stale -- fall back to zero velocity rather
    // than rotate by, or hold at, a heading/position we can no longer trust.
    static constexpr float ODOM_TIMEOUT_SEC = 0.5f;
    // Joystick input older than this is treated as released, so a disconnected or
    // hung gamepad parks the vehicle instead of flying on at its last commanded
    // velocity forever. PX4's own offboard-loss failsafe would NOT catch that, since
    // our setpoint stream keeps flowing -- just with stale contents.
    static constexpr float JOY_TIMEOUT_SEC = 0.5f;

    // PS4 controller axis indices (Wireless Controller via joy_node). Standard ROS
    // sensor_msgs/Joy convention: left and up are positive.
    static constexpr size_t AXIS_LEFT_X = 0;  // Left=+1.0, Right=-1.0
    static constexpr size_t AXIS_LEFT_Y = 1;  // Up=+1.0, Down=-1.0
    static constexpr size_t MIN_AXES_REQUIRED = 2;

    static constexpr size_t BTN_OFFBOARD = 0;  // Cross / X

    // MAV_CMD_DO_SET_MODE: base_mode flag + PX4 custom main mode (px4_custom_mode.h)
    static constexpr uint16_t VEHICLE_CMD_DO_SET_MODE = 176;
    static constexpr float MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0f;
    static constexpr float PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0f;

    bool is_rising_edge(const sensor_msgs::msg::Joy::SharedPtr &msg, size_t idx) const
    {
        if (idx >= msg->buttons.size() || idx >= prev_buttons_.size())
        {
            return false;
        }
        return prev_buttons_[idx] == 0 && msg->buttons[idx] == 1;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() >= MIN_AXES_REQUIRED)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "First /joy message received (%zu axes, %zu buttons)",
                msg->axes.size(), msg->buttons.size());

            latest_axes_ = msg->axes;
            last_joy_recv_time_ = this->get_clock()->now();
            has_joy_data_ = true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "/joy message has only %zu axes (need >= %zu) -- ignored",
                msg->axes.size(), MIN_AXES_REQUIRED);
        }

        if (prev_buttons_.size() != msg->buttons.size())
        {
            // First message (or controller reconnect): establish baseline, don't fire.
            prev_buttons_ = msg->buttons;
            return;
        }

        if (is_rising_edge(msg, BTN_OFFBOARD))
        {
            switch_to_offboard_mode();
        }

        prev_buttons_ = msg->buttons;
    }

    void switch_to_offboard_mode()
    {
        RCLCPP_WARN(this->get_logger(), "[X] Requesting OFFBOARD mode (left stick now flies body-frame X/Y velocity)");
        send_vehicle_command(create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD));
    }

    void send_vehicle_command(const px4_msgs::msg::VehicleCommand &command)
    {
        if (!vehicle_command_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Vehicle command service is not available!");
            return;
        }

        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request = command;

        vehicle_command_client_->async_send_request(
            request,
            [this](rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
            {
                const auto &ack = future.get()->reply;

                if (ack.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
                {
                    RCLCPP_INFO(this->get_logger(), "PX4 ACK: command=%u ACCEPTED", ack.command);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "PX4 ACK: command=%u REJECTED (result=%u)",
                                 ack.command, ack.result);
                }
            });
    }

    px4_msgs::msg::VehicleCommand create_vehicle_command(
        uint16_t command, float param1 = NAN, float param2 = NAN)
    {
        px4_msgs::msg::VehicleCommand msg{};

        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = NAN;
        msg.param4 = NAN;
        msg.param5 = NAN;
        msg.param6 = NAN;
        msg.param7 = NAN;

        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;

        msg.confirmation = 0;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }

    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Setpoints are published in the same local frame regardless, so this still
        // works even if not NED -- just want to know if that assumption ever breaks.
        if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED)
        {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "vehicle_odometry pose_frame is %u, not POSE_FRAME_NED", msg->pose_frame);
        }

        // q is [w, x, y, z], Hamiltonian, body -> world. Yaw only (not full attitude),
        // so roll/pitch tilt never leaks into the horizontal velocity/position command.
        const float w = msg->q[0];
        const float x = msg->q[1];
        const float y = msg->q[2];
        const float z = msg->q[3];
        yaw_ = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

        odom_position_ = msg->position;
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
        setpoint_msg.yaw = NAN;       // yaw left to the RC/vehicle, never commanded here
        setpoint_msg.yawspeed = NAN;

        // Not our mode: heartbeat only, no setpoint (see file header for why the
        // setpoint must stay quiet here). Hold state is reset so the next offboard
        // entry captures a fresh hold point rather than one latched minutes ago in
        // an RC-flown mode.
        if (!offboard_active_)
        {
            mode_msg.position = false;
            mode_msg.velocity = true;

            have_hold_z_ = false;
            hold_position_x_ = NAN;
            hold_position_y_ = NAN;
            was_stick_idle_ = false;

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

            offboard_control_mode_publisher_->publish(mode_msg);
            trajectory_setpoint_publisher_->publish(setpoint_msg);
            return;
        }

        // Captured on the first valid-odometry frame after entering offboard (cleared
        // again on exit above), then held for the rest of this offboard session as the
        // altitude every setpoint below targets. Must NOT be latched once for the
        // node's lifetime: a boot-time capture would pin it to ground level and fly
        // the vehicle down the moment offboard engaged at altitude.
        if (!have_hold_z_)
        {
            hold_z_ = odom_position_[2];
            have_hold_z_ = true;
        }

        // A disconnected/hung gamepad reads as released rather than as its last
        // command, so the vehicle parks instead of flying on indefinitely.
        const bool joy_fresh =
            has_joy_data_ && (now - last_joy_recv_time_).seconds() <= JOY_TIMEOUT_SEC;

        if (!has_joy_data_)
        {
            // Silent before: nothing ever arrived, so there was no "stale" to warn about.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "no /joy message received yet -- holding position "
                "(is joy_node running, on the same ROS_DOMAIN_ID, publishing on /joy?)");
        }
        else if (!joy_fresh)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "/joy stale (last message %.2fs ago, timeout %.2fs) -- holding position "
                "(if this only happens with the stick held still, run joy_node with autorepeat_rate:=20.0)",
                (now - last_joy_recv_time_).seconds(), JOY_TIMEOUT_SEC);
        }

        const bool in_deadband =
            joy_fresh
            && std::fabs(latest_axes_[AXIS_LEFT_X]) < STICK_DEADBAND
            && std::fabs(latest_axes_[AXIS_LEFT_Y]) < STICK_DEADBAND;
        const bool stick_idle = !joy_fresh || in_deadband;

        // Logged on change only, so a released stick doesn't spam. Shows the raw axis
        // values the node actually reads, to catch a wrong AXIS_LEFT_X/Y mapping.
        if (in_deadband != was_in_deadband_)
        {
            RCLCPP_INFO(this->get_logger(),
                "stick %s: axes[%zu]=%.2f axes[%zu]=%.2f (deadband %.2f, %zu axes total)",
                in_deadband ? "in deadband -> holding position" : "out of deadband -> velocity control",
                AXIS_LEFT_X, latest_axes_[AXIS_LEFT_X], AXIS_LEFT_Y, latest_axes_[AXIS_LEFT_Y],
                STICK_DEADBAND, latest_axes_.size());
            was_in_deadband_ = in_deadband;
        }

        if (stick_idle)
        {
            if (!was_stick_idle_)
            {
                // Moving -> idle: capture the hold point once, at the moment of release.
                hold_position_x_ = odom_position_[0];
                hold_position_y_ = odom_position_[1];
            }

            mode_msg.position = true;
            mode_msg.velocity = false;
            setpoint_msg.position = {hold_position_x_, hold_position_y_, hold_z_};
            setpoint_msg.velocity = {NAN, NAN, NAN};
        }
        else
        {
            if (was_stick_idle_)
            {
                // Idle -> moving: drop the captured point so a stale one can't leak
                // back in (fails visibly as NaN rather than silently as an old value).
                hold_position_x_ = NAN;
                hold_position_y_ = NAN;
            }

            // Body frame (FRD): +X = forward, +Y = right. AXIS_LEFT_X is negated
            // because Joy reports left as positive while FRD +Y means right;
            // AXIS_LEFT_Y needs no flip since Joy's up-positive matches +X forward.
            const float vx_body = MAX_XY_VELOCITY * latest_axes_[AXIS_LEFT_Y];
            const float vy_body = -MAX_XY_VELOCITY * latest_axes_[AXIS_LEFT_X];

            float vn = std::cos(yaw_) * vx_body - std::sin(yaw_) * vy_body;
            float ve = std::sin(yaw_) * vx_body + std::cos(yaw_) * vy_body;

            vn = std::clamp(vn, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
            ve = std::clamp(ve, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);

            // xy on velocity, z on position (PX4 resolves control type per axis from
            // which field is finite -- PositionControl::_inputValid).
            mode_msg.position = true;
            mode_msg.velocity = true;
            setpoint_msg.position = {NAN, NAN, hold_z_};
            setpoint_msg.velocity = {vn, ve, NAN};
        }

        was_stick_idle_ = stick_idle;

        offboard_control_mode_publisher_->publish(mode_msg);
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    }

    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

    // Single-threaded executor (see main()): plain members are safe without a mutex
    // since the callbacks and the timer never run concurrently with each other.
    std::vector<float> latest_axes_;
    std::vector<int32_t> prev_buttons_;
    bool has_joy_data_ = false;
    rclcpp::Time last_joy_recv_time_;
    bool was_in_deadband_ = false;  // diagnostics only: logs deadband transitions

    // Defaults false so that until vehicle_status is ever received, the node only
    // emits the neutral heartbeat and never commands motion or position hold.
    bool offboard_active_ = false;

    bool has_odom_ = false;
    rclcpp::Time last_odom_recv_time_;
    float yaw_ = 0.0f;
    std::array<float, 3> odom_position_{};

    bool have_hold_z_ = false;
    float hold_z_ = 0.0f;
    float hold_position_x_ = NAN;
    float hold_position_y_ = NAN;
    // Sentinel false (not "idle") so the first idle frame after entering offboard
    // still counts as a moving->idle transition and captures a hold point.
    bool was_stick_idle_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4OffboardJoystick>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
