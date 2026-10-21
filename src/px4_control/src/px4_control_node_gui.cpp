// V6 šesti node koji sam napravio


#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/manual_control_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <px4_msgs/srv/vehicle_command.hpp>

#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QGridLayout>
#include <QLabel>
#include <QTimer>

#include <chrono>
#include <cmath>
#include <functional>

class PX4GuiControlNode : public rclcpp::Node
{
public:
    PX4GuiControlNode() : Node("px4_gui_control")
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

        // The GUI has no sticks, so publish a fixed neutral manual_control_input
        // continuously. PX4's arming/mode-selection checks require this stream to
        // be present and recent for Manual/Altitude/Position -- without it, arming
        // and those mode switches are silently blocked regardless of COM_RC_IN_MODE.
        // Neutral values keep it hover-safe if a mode ends up actually reading sticks.
        // Same reasoning for the offboard stream re: entering OFFBOARD mode.
        setpoint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]()
            {
                publish_neutral_manual_control();
                publish_offboard_setpoint();
            });

        RCLCPP_INFO(this->get_logger(), "PX4 GUI Control node started!");
    }

    void set_status_callback(std::function<void(const std::string &)> callback)
    {
        status_callback_ = callback;
    }

    void arm()
    {
        RCLCPP_INFO(this->get_logger(), "[GUI] ARM requested");
        report_status("ARM requested...");

        // PX4 boots into Manual mode by default, which requires the throttle stick
        // pulled to the bottom (< -0.8) to arm -- switch to Position first (only
        // requires throttle <= 0.2, which our neutral 0.0 satisfies) and only arm
        // once that's confirmed.
        auto mode_command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_POSCTL);

        send_vehicle_command(mode_command, [this](bool accepted)
        {
            if (!accepted)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not switch to POSITION mode, aborting ARM");
                report_status("ARM failed: could not switch to POSITION mode");
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
        RCLCPP_WARN(this->get_logger(), "[GUI] DISARM (kill) requested");
        report_status("DISARM (kill) requested...");
        auto command = create_vehicle_command(VEHICLE_CMD_ARM, 0.0f, FORCE_DISARM_MAGIC);
        send_vehicle_command(command);
    }

    void takeoff()
    {
        RCLCPP_INFO(this->get_logger(), "[GUI] TAKEOFF requested, altitude=%.2f m", TARGET_ALTITUDE);
        report_status("TAKEOFF requested...");
        auto command = create_vehicle_command(VEHICLE_CMD_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN, TARGET_ALTITUDE);
        send_vehicle_command(command);
    }

    void land()
    {
        RCLCPP_INFO(this->get_logger(), "[GUI] LAND requested");
        report_status("LAND requested...");
        auto command = create_vehicle_command(VEHICLE_CMD_LAND);
        send_vehicle_command(command);
    }

    void switch_to_position_mode()
    {
        RCLCPP_INFO(this->get_logger(), "[GUI] Switching to POSITION mode");
        report_status("Switching to POSITION mode...");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_POSCTL);
        send_vehicle_command(command);
    }

    void loiter()
    {
        RCLCPP_INFO(this->get_logger(), "[GUI] Switching to LOITER mode");
        report_status("Switching to LOITER mode...");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
        send_vehicle_command(command);
    }

    void switch_to_manual_mode()
    {
        RCLCPP_WARN(this->get_logger(), "[GUI] Switching to MANUAL mode (no altitude/position hold!)");
        report_status("Switching to MANUAL mode...");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_MANUAL);
        send_vehicle_command(command);
    }

    void switch_to_altitude_mode()
    {
        RCLCPP_WARN(this->get_logger(), "[GUI] Switching to ALTITUDE mode (no position hold)");
        report_status("Switching to ALTITUDE mode...");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_ALTCTL);
        send_vehicle_command(command);
    }

    void switch_to_offboard_mode()
    {
        // GUI has no D-pad equivalent, so the offboard velocity setpoint we stream
        // is always zero -- this just puts it into a stationary hover in Offboard.
        RCLCPP_WARN(this->get_logger(), "[GUI] Switching to OFFBOARD mode (hovers in place, no velocity control from GUI)");
        report_status("Switching to OFFBOARD mode...");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD);
        send_vehicle_command(command);
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

    void report_status(const std::string &text)
    {
        if (status_callback_)
        {
            status_callback_(text);
        }
    }

    void publish_neutral_manual_control()
    {
        px4_msgs::msg::ManualControlSetpoint msg{};

        const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000;
        msg.timestamp = now_us;
        msg.timestamp_sample = now_us;
        msg.valid = true;
        msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0;

        msg.throttle = 0.0f;
        msg.yaw = 0.0f;
        msg.pitch = 0.0f;
        msg.roll = 0.0f;

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

    void publish_offboard_setpoint()
    {
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
        setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
        setpoint_msg.acceleration = {NAN, NAN, NAN};
        setpoint_msg.jerk = {NAN, NAN, NAN};
        setpoint_msg.yaw = NAN;
        setpoint_msg.yawspeed = NAN;
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    }

    void send_vehicle_command(
        const px4_msgs::msg::VehicleCommand &command,
        std::function<void(bool accepted)> on_result = nullptr)
    {
        if (!vehicle_command_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Vehicle command service is not available!");
            report_status("Vehicle command service is not available!");
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
                    report_status("Command " + std::to_string(ack.command) + " ACCEPTED");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "PX4 ACK: command=%u REJECTED (result=%u)", ack.command, ack.result);
                    report_status("Command " + std::to_string(ack.command) + " REJECTED (result=" + std::to_string(ack.result) + ")");
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

    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    rclcpp::TimerBase::SharedPtr arm_delay_timer_;

    std::function<void(const std::string &)> status_callback_;
};

class PX4GuiWindow : public QWidget
{
public:
    explicit PX4GuiWindow(std::shared_ptr<PX4GuiControlNode> node) : node_(node)
    {
        setWindowTitle("PX4 Drone Control");

        auto *layout = new QGridLayout(this);

        auto add_button = [&](const QString &text, int row, int col,
                               std::function<void()> on_click, const QString &style = QString())
        {
            auto *button = new QPushButton(text);
            button->setMinimumSize(160, 55);
            if (!style.isEmpty())
            {
                button->setStyleSheet(style);
            }
            QObject::connect(button, &QPushButton::clicked, this, [on_click]()
            {
                on_click();
            });
            layout->addWidget(button, row, col);
        };

        add_button("ARM", 0, 0, [this]() { node_->arm(); }, "background-color:#2ecc71;");
        add_button("DISARM (KILL)", 0, 1, [this]() { node_->disarm(); }, "background-color:#e74c3c; color:white;");
        add_button("TAKEOFF", 1, 0, [this]() { node_->takeoff(); });
        add_button("LAND", 1, 1, [this]() { node_->land(); });
        add_button("POSITION", 2, 0, [this]() { node_->switch_to_position_mode(); });
        add_button("LOITER", 2, 1, [this]() { node_->loiter(); });
        add_button("MANUAL", 3, 0, [this]() { node_->switch_to_manual_mode(); }, "background-color:#f39c12;");
        add_button("ALTITUDE", 3, 1, [this]() { node_->switch_to_altitude_mode(); }, "background-color:#f39c12;");
        add_button("OFFBOARD", 4, 0, [this]() { node_->switch_to_offboard_mode(); }, "background-color:#f39c12;");

        status_label_ = new QLabel("Ready");
        status_label_->setMinimumHeight(30);
        layout->addWidget(status_label_, 5, 0, 1, 2);

        node_->set_status_callback([this](const std::string &text)
        {
            status_label_->setText(QString::fromStdString(text));
        });
    }

private:
    std::shared_ptr<PX4GuiControlNode> node_;
    QLabel *status_label_;
};

int main(int argc, char **argv)
{
    // QApplication must be constructed before rclcpp::init(): DDS/rclcpp spins up
    // background threads that otherwise interfere with Qt's XCB thread setup and
    // leave the window created but never actually mapped/shown.
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PX4GuiControlNode>();
    PX4GuiWindow window(node);
    window.show();

    // Drive rclcpp callbacks (service responses, publish timers) from inside the
    // Qt event loop instead of a separate spin thread -- keeps everything on one
    // thread, so the status-label update from the ROS callback is safe as-is.
    QTimer ros_spin_timer;
    QObject::connect(&ros_spin_timer, &QTimer::timeout, [node]()
    {
        rclcpp::spin_some(node);
    });
    ros_spin_timer.start(10);

    const int result = app.exec();
    rclcpp::shutdown();
    return result;
}
