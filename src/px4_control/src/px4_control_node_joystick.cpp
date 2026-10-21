// V4 četvrti node koji sam napravio


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include <px4_msgs/srv/vehicle_command.hpp>

#include <cmath>

class PX4JoystickControl : public rclcpp::Node
{
public:
    PX4JoystickControl() : Node("px4_joystick_control")
    {
        vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg)
            {
                joy_callback(msg);
            });

        RCLCPP_INFO(this->get_logger(), "PX4 Joystick Control node started!");
        RCLCPP_INFO(this->get_logger(), "Cross=ARM  Triangle=TAKEOFF  Square=LAND  Options=DISARM (kill)");
    }

private:
    // Vehicle command IDs
    static constexpr uint16_t VEHICLE_CMD_ARM = 400;
    static constexpr uint16_t VEHICLE_CMD_TAKEOFF = 22;
    static constexpr uint16_t VEHICLE_CMD_LAND = 21;

    static constexpr float TARGET_ALTITUDE = 5.0f;

    // MAV_CMD_COMPONENT_ARM_DISARM force-disarm magic number: bypasses the
    // landed-state check so the kill button also works mid-flight.
    static constexpr float FORCE_DISARM_MAGIC = 21196.0f;

    // PS4 controller button indices (Wireless Controller via joy_node), confirmed live
    static constexpr size_t BTN_ARM = 0;      // Cross
    static constexpr size_t BTN_TAKEOFF = 2;  // Triangle
    static constexpr size_t BTN_LAND = 3;     // Square
    static constexpr size_t BTN_DISARM = 9;   // Options (kill switch)

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

        prev_buttons_ = msg->buttons;
    }

    void arm()
    {
        RCLCPP_INFO(this->get_logger(), "[Cross] ARM requested");
        auto command = create_vehicle_command(VEHICLE_CMD_ARM, 1.0f);
        send_vehicle_command(command);
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
                auto response = future.get();
                const auto &ack = response->reply;

                if (ack.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
                {
                    RCLCPP_INFO(this->get_logger(), "PX4 ACK: command=%u ACCEPTED", ack.command);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "PX4 ACK: command=%u REJECTED (result=%u)", ack.command, ack.result);
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

    std::vector<int32_t> prev_buttons_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4JoystickControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
