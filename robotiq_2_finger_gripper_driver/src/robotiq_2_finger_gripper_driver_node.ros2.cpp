#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include <robotiq_2_finger_gripper_driver/robotiq_2_finger_gripper_driver.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <robotiq_2_finger_gripper_driver/msg/robotiq2_finger_command.hpp>
#include <robotiq_2_finger_gripper_driver/msg/robotiq2_finger_state.hpp>

namespace robotiq_2_finger_gripper_driver
{
class Robotiq2FingerDriverNode : public rclcpp::Node
{
private:
  using Robotiq2FingerState =
    robotiq_2_finger_gripper_driver::msg::Robotiq2FingerState;
  std::shared_ptr<rclcpp::Publisher<Robotiq2FingerState>> status_pub_;
  using Robotiq2FingerCommand =
    robotiq_2_finger_gripper_driver::msg::Robotiq2FingerCommand;
  std::shared_ptr<rclcpp::Subscription<Robotiq2FingerCommand>> command_sub_;
  std::unique_ptr<Robotiq2FingerGripperModbusInterface> gripper_interface_ptr_;
  std::shared_ptr<rclcpp::TimerBase> poll_timer_;

public:
  explicit Robotiq2FingerDriverNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robotiq_2_finger_driver", options)
  {
    // Default ROS params
    const double DEFAULT_POLL_RATE = 10.0;

    const std::string DEFAULT_STATE_TOPIC("robotiq_2_finger_state");
    const std::string DEFAULT_COMMAND_TOPIC("robotiq_2_finger_command");
    const std::string DEFAULT_MODBUS_TCP_ADDRESS("");
    const int32_t DEFAULT_MODBUS_TCP_PORT = 502;
    const std::string DEFAULT_MODBUS_RTU_INTERFACE("");
    const int32_t DEFAULT_GRIPPER_BAUD_RATE = 115200;
    const int32_t DEFAULT_GRIPPER_SLAVE_ID = 0x09;

    // Get params
    const std::string modbus_tcp_address
        = this->declare_parameter(
            "modbus_tcp_address", DEFAULT_MODBUS_TCP_ADDRESS);
    const int32_t modbus_tcp_port
        = static_cast<int32_t>(this->declare_parameter(
            "modbus_tcp_port", DEFAULT_MODBUS_TCP_PORT));
    const std::string modbus_rtu_interface
        = this->declare_parameter(
            "modbus_rtu_interface", DEFAULT_MODBUS_RTU_INTERFACE);
    const int32_t modbus_rtu_baud_rate
        = static_cast<int32_t>(this->declare_parameter(
            "modbus_rtu_baud_rate", DEFAULT_GRIPPER_BAUD_RATE));
    const uint16_t gripper_slave_id
        = static_cast<uint16_t>(this->declare_parameter(
            "gripper_slave_id", DEFAULT_GRIPPER_SLAVE_ID));

    // Make ROS publisher + subscriber
    const std::string status_topic
        = this->declare_parameter("state_topic", DEFAULT_STATE_TOPIC);
    status_pub_ = this->create_publisher<Robotiq2FingerState>(status_topic, 1);

    using std::placeholders::_1;
    const std::string command_topic
        = this->declare_parameter("command_topic", DEFAULT_COMMAND_TOPIC);
    command_sub_ = this->create_subscription<Robotiq2FingerCommand>(
        command_topic, 1,
        std::bind(&Robotiq2FingerDriverNode::CommandCB, this, _1));

    // Make the logging function
    std::function<void(const std::string&)> logging_fn
        = [this] (const std::string& message)
    {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    };
    // Make interface to gripper
    if (!modbus_tcp_address.empty() && modbus_rtu_interface.empty())
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Connecting to Robotiq 2-Finger gripper with gripper slave ID "
          "%hx on Modbus TCP interface %s:%i...",
          gripper_slave_id, modbus_tcp_address.c_str(), modbus_tcp_port);
      gripper_interface_ptr_
          = std::make_unique<Robotiq2FingerGripperModbusTcpInterface>(
              logging_fn, modbus_tcp_address, modbus_tcp_port,
              gripper_slave_id);
    }
    else if (modbus_tcp_address.empty() && !modbus_rtu_interface.empty())
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Connecting to Robotiq 2-Finger gripper with gripper slave ID "
          "%hx on Modbus RTU interface %s at %d baud...",
          gripper_slave_id, modbus_rtu_interface.c_str(),
          modbus_rtu_baud_rate);
      gripper_interface_ptr_
          = std::make_unique<Robotiq2FingerGripperModbusRtuInterface>(
              logging_fn, modbus_rtu_interface, modbus_rtu_baud_rate,
              gripper_slave_id);
    }
    else if (!modbus_tcp_address.empty() && !modbus_rtu_interface.empty())
    {
      throw std::invalid_argument(
          "modbus_tcp_address and modbus_rtu_interface cannot both be used");
    }
    else
    {
      throw std::invalid_argument(
          "One of modbus_tcp_address or modbus_rtu_interface must be provided");
    }
    // Activate gripper
    const bool success = gripper_interface_ptr_->ActivateGripper();
    if (!success)
    {
      throw std::runtime_error("Unable to initialize gripper");
    }
    RCLCPP_INFO(this->get_logger(), "Gripper interface running");
    const double poll_rate
        = std::abs(this->declare_parameter("poll_rate", DEFAULT_POLL_RATE));
    poll_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(1. / poll_rate),
        std::bind(&Robotiq2FingerDriverNode::PublishGripperStatus, this));
  }

private:
  void CommandCB(const Robotiq2FingerCommand& command_msg)
  {
    try
    {
      const Robotiq2FingerGripperCommand
          gripper_command(command_msg.percent_closed,
                          command_msg.percent_speed,
                          command_msg.percent_effort);
      const bool sent
          = gripper_interface_ptr_->SendGripperCommand(gripper_command);
      if (!sent)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to send command to gripper");
      }
    }
    catch (const std::invalid_argument& ex)
    {
      RCLCPP_ERROR(
          this->get_logger(), "Failed to command - exception [%s]", ex.what());
    }
  }

  void PublishGripperStatus()
  {
    const Robotiq2FingerGripperStatus status
        = gripper_interface_ptr_->GetGripperStatus();
    Robotiq2FingerState state_msg;
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.actual_percent_closed = status.ActualPosition();
    state_msg.actual_percent_current = status.ActualCurrent();
    state_msg.target_percent_closed = status.TargetPosition();
    status_pub_->publish(state_msg);
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robotiq_2_finger_gripper_driver::Robotiq2FingerDriverNode)
