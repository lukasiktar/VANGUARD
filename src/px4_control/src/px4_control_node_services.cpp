// V2 drugi node koji sam napravio

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <px4_msgs/srv/vehicle_command.hpp>

#include <chrono>
#include <cmath>


class PX4Control : public rclcpp::Node
{
public:

    PX4Control() : Node("px4_control")
    {
    setup_subscribers();
    setup_timers();
    setup_clients();

    RCLCPP_INFO(this->get_logger(), "PX4 Control node started!");

    }

        // Function to send ARM command
    void arm()
    {
        RCLCPP_INFO(this->get_logger(),"ARM function called!");
        auto command = create_vehicle_command(VEHICLE_CMD_ARM,1.0);
        command_state_ = CommandState::WAITING_ARM_ACK;
        send_vehicle_command(command);
        RCLCPP_INFO(this->get_logger(),"ARM command sent: command=%u param1=%f",command.command,command.param1);
    }

        // Function to send TAKEOFF command with specified altitude
    void takeoff(float altitude)
    {
        RCLCPP_INFO(this->get_logger(),"TAKEOFF function called! Altitude: %.2f m",altitude);
        auto command = create_vehicle_command(VEHICLE_CMD_TAKEOFF,NAN,NAN,NAN,NAN,NAN,NAN,altitude);
        command_state_ = CommandState::WAITING_TAKEOFF_ACK;
        send_vehicle_command(command);
        RCLCPP_INFO(this->get_logger(),"TAKEOFF command sent: command=%u altitude=%.2f",command.command,command.param7);
    }

    void land()
    {
        RCLCPP_INFO(this->get_logger(), "LAND function called!");
        auto command = create_vehicle_command(VEHICLE_CMD_LAND);
        command_state_ = CommandState::WAITING_LAND_ACK;
        send_vehicle_command(command);
        RCLCPP_INFO(this->get_logger(), "LAND command sent: command=%u", command.command);
    }


private:
    // Constants
    static constexpr uint16_t VEHICLE_CMD_ARM = 400;
    static constexpr uint16_t VEHICLE_CMD_TAKEOFF = 22;
    static constexpr uint16_t VEHICLE_CMD_LAND = 21;
    static constexpr float TARGET_ALTITUDE = 5.0f;
    static constexpr float ALTITUDE_TOLERANCE = 0.35f;
    static constexpr size_t QOS_DEPTH = 1;

    // State
    enum class CommandState
    {
        IDLE,
        WAITING_ARM_ACK,
        WAITING_TAKEOFF_ACK,
        WAITING_LAND_ACK
    };

    CommandState command_state_ = CommandState::IDLE;

    bool altitude_reached_ = false;

    float current_altitude_ = 0.0f;
    //---------------------------------------------------------------
    //Setup functions

    void setup_subscribers()
    {
        rclcpp::QoS qos_sub(QOS_DEPTH);
        qos_sub.best_effort().durability_volatile();

        vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos_sub,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
            {
                vehicle_local_position_callback(msg);
            });
    }

    void setup_timers()
    {
        arm_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]()
            {
                this->arm();
                arm_timer_->cancel();
            });

    }

    void setup_clients()
    {
        vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
    }

    // --------------------Callbacks----------
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        if (!msg->z_valid)
            return;

        current_altitude_ = -msg->z;

        if (!altitude_reached_ &&
            command_state_ == CommandState::IDLE &&
            current_altitude_ >= TARGET_ALTITUDE - ALTITUDE_TOLERANCE)
        {
            altitude_reached_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Target altitude reached: %.2f m. Starting LAND...",
                current_altitude_);

            this->land();
        }
    }

    void send_vehicle_command(const px4_msgs::msg::VehicleCommand &command)
    {
        if (!this->vehicle_command_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(),"Vehicle command service is not available!");
            return;
        }
        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

        request->request = command;

        vehicle_command_client_->async_send_request(request,[this](rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
            {
                auto response = future.get();
                const auto &ack = response->reply;

                RCLCPP_INFO(this->get_logger(), "PX4 ACK received: command=%u result=%u", ack.command, ack.result);

                if (ack.result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
                {
                    RCLCPP_ERROR(this->get_logger(), "Command %u was not accepted! Result: %u", ack.command, ack.result);
                    this->command_state_ = CommandState::IDLE;
                    return;
                }

                if (ack.command == VEHICLE_CMD_ARM && this->command_state_ == CommandState::WAITING_ARM_ACK)
                {
                    RCLCPP_INFO(this->get_logger(), "ARM command ACCEPTED! Starting TAKEOFF...");
                    this->command_state_ = CommandState::WAITING_TAKEOFF_ACK;
                    takeoff(TARGET_ALTITUDE);
                    
                }
                else if (ack.command == VEHICLE_CMD_TAKEOFF && this->command_state_ == CommandState::WAITING_TAKEOFF_ACK)
                {
                    RCLCPP_INFO(this->get_logger(), "TAKEOFF command ACCEPTED!");
                    this->command_state_ = CommandState::IDLE;
                }
                else if (ack.command == VEHICLE_CMD_LAND && this->command_state_ == CommandState::WAITING_LAND_ACK)
                {
                    RCLCPP_INFO(this->get_logger(), "LAND command ACCEPTED!");
                    this->command_state_ = CommandState::IDLE;
                }
            });
    }

    // Command creation
    //privatna clanska funkcija za kreiranje VehicleCommand poruke
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

        msg.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }

    // ROS 2 interfaces
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr arm_timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}