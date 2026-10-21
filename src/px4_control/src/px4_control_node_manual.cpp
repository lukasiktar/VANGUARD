// V5 peti node koji sam napravio


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/manual_control_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <px4_msgs/srv/vehicle_command.hpp>

#include <chrono>
#include <cmath>

class PX4ManualControl : public rclcpp::Node
{
public:
    PX4ManualControl() : Node("px4_manual_control")
    {
        vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

        rclcpp::QoS qos_pub(1);
        qos_pub.best_effort().durability_volatile();
        manual_control_publisher_ = this->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
            "/fmu/in/manual_control_input", qos_pub);
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

        // Publish continuously (not just on stick movement) so PX4 never sees
        // the manual control source go stale/timeout (COM_RC_LOSS_T) while sticks
        // are held steady. Also stream offboard_control_mode + trajectory_setpoint
        // unconditionally from startup: PX4 only accepts switching into OFFBOARD if
        // that stream is already recent (COM_OF_LOSS_T), so it must be running
        // before R2 is ever pressed, not started in response to it.
        manual_control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]()
            {
                publish_manual_control();
                publish_offboard_setpoint();
            });

        RCLCPP_INFO(this->get_logger(), "PX4 Manual Control node started!");
        RCLCPP_INFO(this->get_logger(), "Cross=ARM  Triangle=TAKEOFF  Square=LAND  Options=DISARM (kill)  R1=POSITION mode  Circle=LOITER mode  L1=MANUAL mode  L2=ALTITUDE mode  R2=OFFBOARD mode");
        RCLCPP_INFO(this->get_logger(), "Left stick=Throttle/Yaw  Right stick=Pitch/Roll (Mode 2)  D-pad=Offboard X/Y velocity");
        RCLCPP_INFO(this->get_logger(),
            "Sticks only move the vehicle once it's armed AND in a manual flight mode (R1). "
            "If it doesn't respond, check COM_RC_IN_MODE allows MAVLink/joystick input.");
    }

private:
    // Vehicle command IDs
    static constexpr uint16_t VEHICLE_CMD_ARM = 400;
    static constexpr uint16_t VEHICLE_CMD_TAKEOFF = 22;
    static constexpr uint16_t VEHICLE_CMD_LAND = 21;
    static constexpr uint16_t VEHICLE_CMD_DO_SET_MODE = 176;

    static constexpr float TARGET_ALTITUDE = 5.0f;

    // MAV_CMD_COMPONENT_ARM_DISARM force-disarm magic number: bypasses the
    // landed-state check so the kill button also works mid-flight.
    static constexpr float FORCE_DISARM_MAGIC = 21196.0f;

    // MAV_CMD_DO_SET_MODE params: base_mode flag + PX4 custom main/sub mode (px4_custom_mode.h)
    static constexpr float MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0f;
    static constexpr float PX4_CUSTOM_MAIN_MODE_MANUAL = 1.0f;
    static constexpr float PX4_CUSTOM_MAIN_MODE_ALTCTL = 2.0f;
    static constexpr float PX4_CUSTOM_MAIN_MODE_POSCTL = 3.0f;
    static constexpr float PX4_CUSTOM_MAIN_MODE_AUTO = 4.0f;
    static constexpr float PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3.0f;
    static constexpr float PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0f;

    // Max horizontal speed commanded by the D-pad in Offboard mode, m/s.
    static constexpr float MAX_XY_VELOCITY = 5.0f;

    // PS4 controller button indices (Wireless Controller via joy_node), confirmed live
    static constexpr size_t BTN_ARM = 0;      // Cross
    static constexpr size_t BTN_LOITER = 1;   // Circle
    static constexpr size_t BTN_TAKEOFF = 2;  // Triangle
    static constexpr size_t BTN_LAND = 3;     // Square
    static constexpr size_t BTN_MANUAL = 4;   // L1
    static constexpr size_t BTN_POSCTL = 5;   // R1
    static constexpr size_t BTN_DISARM = 9;   // Options (kill switch)

    // PS4 controller axis indices, confirmed live (see joystick probe)
    static constexpr size_t AXIS_LEFT_X = 0;
    static constexpr size_t AXIS_LEFT_Y = 1;
    static constexpr size_t AXIS_L2 = 2;      // analog trigger, rests at +1.0, pulled = negative
    static constexpr size_t AXIS_RIGHT_X = 3;
    static constexpr size_t AXIS_RIGHT_Y = 4;
    static constexpr size_t AXIS_R2 = 5;      // analog trigger, rests at +1.0, pulled = negative
    static constexpr size_t AXIS_DPAD_X = 6;  // D-pad, confirmed live: Left=-1.0, Right=+1.0
    static constexpr size_t AXIS_DPAD_Y = 7;  // D-pad, confirmed live: Up=+1.0, Down=-1.0
    static constexpr size_t MIN_AXES_REQUIRED = 8;

    // L2/R2 are analog triggers, not digital buttons, so treat "pulled past halfway"
    // as pressed and edge-detect that the same way as the digital buttons.
    static constexpr float TRIGGER_PRESSED_THRESHOLD = 0.5f;

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
            const bool l2_pressed_now = msg->axes[AXIS_L2] < TRIGGER_PRESSED_THRESHOLD;
            const bool r2_pressed_now = msg->axes[AXIS_R2] < TRIGGER_PRESSED_THRESHOLD;

            if (has_joy_data_ && l2_pressed_now && !l2_was_pressed_)
            {
                switch_to_altitude_mode();
            }

            if (has_joy_data_ && r2_pressed_now && !r2_was_pressed_)
            {
                switch_to_offboard_mode();
            }

            l2_was_pressed_ = l2_pressed_now;
            r2_was_pressed_ = r2_pressed_now;
            latest_axes_ = msg->axes;
            has_joy_data_ = true;
        }

        if (prev_buttons_.size() != msg->buttons.size())
        {
            // First message (or controller reconnect): establish baseline, don't fire.
            prev_buttons_ = msg->buttons;
            return;
        }

        if (is_rising_edge(msg, BTN_ARM))
        {
            arm();
        }
        else if (is_rising_edge(msg, BTN_TAKEOFF))
        {
            takeoff(TARGET_ALTITUDE);
        }
        else if (is_rising_edge(msg, BTN_LAND))
        {
            land();
        }
        else if (is_rising_edge(msg, BTN_DISARM))
        {
            disarm();
        }
        else if (is_rising_edge(msg, BTN_POSCTL))
        {
            switch_to_position_mode();
        }
        else if (is_rising_edge(msg, BTN_LOITER))
        {
            loiter();
        }
        else if (is_rising_edge(msg, BTN_MANUAL))
        {
            switch_to_manual_mode();
        }

        prev_buttons_ = msg->buttons;
    }

    // Builds and publishes manual_control_input from the latest stick state at a
    // fixed rate. Mode 2 RC layout: left stick = throttle/yaw, right stick = pitch/roll.
    // Axis signs below correct for this controller's raw convention (left/up positive)
    // to match PX4's roll+/pitch-/yaw+/throttle+ meanings.
    void publish_manual_control()
    {
        if (!has_joy_data_)
        {
            return;
        }

        px4_msgs::msg::ManualControlSetpoint msg{};

        const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000;
        msg.timestamp = now_us;
        msg.timestamp_sample = now_us;
        msg.valid = true;
        msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0;

        msg.throttle = latest_axes_[AXIS_LEFT_Y];
        msg.yaw = -latest_axes_[AXIS_LEFT_X];
        msg.pitch = -latest_axes_[AXIS_RIGHT_Y];
        msg.roll = -latest_axes_[AXIS_RIGHT_X];

        msg.flaps = NAN;
        msg.aux1 = NAN;
        msg.aux2 = NAN;
        msg.aux3 = NAN;
        msg.aux4 = NAN;
        msg.aux5 = NAN;
        msg.aux6 = NAN;

        msg.sticks_moving = false;
        msg.buttons = 0;

        manual_control_publisher_->publish(msg);
    }

    // Builds and publishes the offboard velocity setpoint from the D-pad state at a
    // fixed rate, unconditionally (whether or not Offboard mode is actually active
    // right now) -- PX4 requires this stream to already be recent before it will
    // accept switching into OFFBOARD, so it can't only start once R2 is pressed.
    // vz is explicitly commanded to 0 (not NaN) so altitude is actively held;
    // only X/Y are under D-pad control, in world/NED frame (not vehicle-relative).
    void publish_offboard_setpoint()
    {
        if (!has_joy_data_)
        {
            return;
        }

        const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000;

        px4_msgs::msg::OffboardControlMode mode_msg{};
        mode_msg.timestamp = now_us;
        mode_msg.position = false;
        mode_msg.velocity = true;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.thrust_and_torque = false;
        mode_msg.direct_actuator = false;
        offboard_control_mode_publisher_->publish(mode_msg);

        px4_msgs::msg::TrajectorySetpoint setpoint_msg{};
        setpoint_msg.timestamp = now_us;
        setpoint_msg.position = {NAN, NAN, NAN};
        setpoint_msg.velocity = {
            MAX_XY_VELOCITY * latest_axes_[AXIS_DPAD_Y],
            MAX_XY_VELOCITY * latest_axes_[AXIS_DPAD_X],
            0.0f};
        setpoint_msg.acceleration = {NAN, NAN, NAN};
        setpoint_msg.jerk = {NAN, NAN, NAN};
        setpoint_msg.yaw = NAN;
        setpoint_msg.yawspeed = NAN;
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    }

    void switch_to_offboard_mode()
    {
        // Offboard holds altitude (vz commanded to 0), but X/Y have no position
        // lock -- releasing the D-pad stops it (velocity back to 0) rather than
        // returning it to where it started, unlike Position mode's active hold.
        RCLCPP_WARN(this->get_logger(), "[R2] Switching to OFFBOARD mode (D-pad now flies X/Y velocity)");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD);
        send_vehicle_command(command);
    }

    void arm()
    {
        RCLCPP_INFO(this->get_logger(), "[Cross] ARM requested");
        // PX4 boots into Manual mode by default, which requires the throttle stick
        // pulled to the bottom (< -0.8) to arm -- a self-centering gamepad stick can't
        // do that. Position mode only requires throttle <= 0.2 (centered is fine), so
        // switch there first and only arm once that's confirmed.
        auto mode_command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_POSCTL);

        send_vehicle_command(mode_command, [this](bool accepted)
        {
            if (!accepted)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not switch to POSITION mode, aborting ARM");
                return;
            }

            arm_delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                {
                    arm_delay_timer_->cancel();
                    auto arm_command = create_vehicle_command(VEHICLE_CMD_ARM, 1.0f);
                    send_vehicle_command(arm_command);
                });
        });
    }

    void disarm()
    {
        RCLCPP_WARN(this->get_logger(), "[Options] DISARM (kill) requested");
        auto command = create_vehicle_command(VEHICLE_CMD_ARM, 0.0f, FORCE_DISARM_MAGIC);
        send_vehicle_command(command);
    }

    void takeoff(float altitude)
    {
        RCLCPP_INFO(this->get_logger(), "[Triangle] TAKEOFF requested, altitude=%.2f m", altitude);
        auto command = create_vehicle_command(VEHICLE_CMD_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN, altitude);
        send_vehicle_command(command);
    }

    void land()
    {
        RCLCPP_INFO(this->get_logger(), "[Square] LAND requested");
        auto command = create_vehicle_command(VEHICLE_CMD_LAND);
        send_vehicle_command(command);
    }

    void switch_to_position_mode()
    {
        RCLCPP_INFO(this->get_logger(), "[R1] Switching to POSITION mode (sticks now fly the vehicle)");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_POSCTL);
        send_vehicle_command(command);
    }

    void switch_to_altitude_mode()
    {
        // Altitude holds climb rate like Position mode (centered throttle = hover),
        // but roll/pitch are self-leveled only, same as Manual -- no GPS position
        // lock, so it'll still drift horizontally once you release the sticks.
        RCLCPP_WARN(this->get_logger(), "[L2] Switching to ALTITUDE mode (no position hold, altitude held)");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_ALTCTL);
        send_vehicle_command(command);
    }

    void switch_to_manual_mode()
    {
        // Manual has no position/altitude hold: throttle is direct thrust (centered
        // stick != hover) and roll/pitch is self-leveled attitude only, no velocity
        // or position lock. Expect it to climb/sink and drift once switched.
        RCLCPP_WARN(this->get_logger(), "[L1] Switching to MANUAL mode (no altitude/position hold!)");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_MANUAL);
        send_vehicle_command(command);
    }

    void loiter()
    {
        RCLCPP_INFO(this->get_logger(), "[Circle] Switching to LOITER mode (holds position, ignores sticks)");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
        send_vehicle_command(command);
    }

    void send_vehicle_command(
        const px4_msgs::msg::VehicleCommand &command,
        std::function<void(bool accepted)> on_result = nullptr)
    {
        if (!vehicle_command_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Vehicle command service is not available!");
            if (on_result)
            {
                on_result(false);
            }
            return;
        }

        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request = command;

        vehicle_command_client_->async_send_request(
            request,
            [this, on_result](rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
            {
                auto response = future.get();
                const auto &ack = response->reply;

                const bool accepted = (ack.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED);

                if (accepted)
                {
                    RCLCPP_INFO(this->get_logger(), "PX4 ACK: command=%u ACCEPTED", ack.command);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "PX4 ACK: command=%u REJECTED (result=%u)", ack.command, ack.result);
                }

                if (on_result)
                {
                    on_result(accepted);
                }
            });
    }

    px4_msgs::msg::VehicleCommand create_vehicle_command(
        uint16_t command,
        float param1 = NAN,
        float param2 = NAN,
        float param3 = NAN,
        float param4 = NAN,
        float param5 = NAN,
        float param6 = NAN,
        float param7 = NAN)
    {
        px4_msgs::msg::VehicleCommand msg{};

        msg.command = command;

        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;

        msg.target_system = 1;
        msg.target_component = 1;

        msg.source_system = 1;
        msg.source_component = 1;

        msg.confirmation = 0;
        msg.from_external = true;

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr manual_control_timer_;
    rclcpp::TimerBase::SharedPtr arm_delay_timer_;

    std::vector<int32_t> prev_buttons_;
    std::vector<float> latest_axes_;
    bool has_joy_data_ = false;
    bool l2_was_pressed_ = false;
    bool r2_was_pressed_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4ManualControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
