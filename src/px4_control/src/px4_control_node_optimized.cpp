// V3 treci node koji sam napravio


#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

#include <chrono>
#include <cmath>


class PX4Control : public rclcpp::Node
{
public:

    PX4Control() : Node("px4_control")
    {
    setup_publishers();
    setup_subscribers();
    setup_timers();

    RCLCPP_INFO(this->get_logger(), "PX4 Control node started!");

    }

        // Function to send ARM command
    void arm()
    {
        RCLCPP_INFO(this->get_logger(),"ARM function called!");

        auto msg = create_vehicle_command(VEHICLE_CMD_ARM,1.0);
        vehicle_command_publisher_->publish(msg);
        command_state_ = CommandState::WAITING_ARM_ACK;

        RCLCPP_INFO(this->get_logger(),"ARM command published: command=%u param1=%f",msg.command,msg.param1);
    }

        // Function to send TAKEOFF command with specified altitude
    void takeoff(float altitude)
    {
        RCLCPP_INFO(this->get_logger(),"TAKEOFF function called! Altitude: %.2f m",altitude);

        auto msg = create_vehicle_command(VEHICLE_CMD_TAKEOFF,NAN,NAN,NAN,NAN,NAN,NAN,altitude);
        vehicle_command_publisher_->publish(msg);
        command_state_ = CommandState::WAITING_TAKEOFF_ACK;

        RCLCPP_INFO(this->get_logger(),"TAKEOFF command published: command=%u altitude=%.2f",msg.command,msg.param7);
    }

    void land()
    {
        RCLCPP_INFO(this->get_logger(), "LAND function called!");

        auto msg = create_vehicle_command(VEHICLE_CMD_LAND);
        vehicle_command_publisher_->publish(msg);
        command_state_ = CommandState::WAITING_LAND_ACK;

        RCLCPP_INFO(this->get_logger(), "LAND command published: command=%u", msg.command);
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

    //Setup functions
    void setup_publishers()
    {
        rclcpp::QoS qos_pub(QOS_DEPTH);
        qos_pub.best_effort().durability_volatile();

        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_pub);
    }

    void setup_subscribers()
    {
        rclcpp::QoS qos_sub(QOS_DEPTH);
        qos_sub.best_effort().durability_volatile();

        vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack_v1", qos_sub,
            [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
            {
                vehicle_command_ack_callback(msg);
            });

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

    // --------------------Callbacks----------
    void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PX4 ACK received: command=%u result=%u", msg->command, msg->result);

        if (msg->result != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Command rejected: command=%u result=%u", msg->command, msg->result);
            return;
        }

        switch (command_state_)
        {
            case CommandState::WAITING_ARM_ACK:
                if (msg->command == VEHICLE_CMD_ARM)
                {
                    RCLCPP_INFO(this->get_logger(), "ARM command ACCEPTED! Starting TAKEOFF...");
                    command_state_ = CommandState::IDLE;
                    this->takeoff(TARGET_ALTITUDE);
                }
                break;

            case CommandState::WAITING_TAKEOFF_ACK:
                if (msg->command == VEHICLE_CMD_TAKEOFF)
                {
                    RCLCPP_INFO(this->get_logger(), "TAKEOFF command ACCEPTED!");
                    command_state_ = CommandState::IDLE;
                }
                break;

            case CommandState::WAITING_LAND_ACK:
                if (msg->command == VEHICLE_CMD_LAND)
                {
                    RCLCPP_INFO(this->get_logger(), "LAND command ACCEPTED!");
                    command_state_ = CommandState::IDLE;
                }
                break;

            case CommandState::IDLE:
                break;
        }
    }

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
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;

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