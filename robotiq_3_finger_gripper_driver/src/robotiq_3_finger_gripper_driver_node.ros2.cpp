#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include <robotiq_3_finger_gripper_driver/robotiq_3_finger_gripper_driver.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <robotiq_3_finger_gripper_driver/msg/robotiq3_finger_command.hpp>
#include <robotiq_3_finger_gripper_driver/msg/robotiq3_finger_state.hpp>

#include "robotiq_3_finger_gripper_driver_node.ros2.hpp"
#include "type_conversions.hpp"

namespace robotiq_3_finger_gripper_driver
{

Robotiq3FingerDriverNode::Robotiq3FingerDriverNode(
    const rclcpp::NodeOptions& options)
  : rclcpp::Node("robotiq_3_finger_driver", options)
{
  // Default ROS params
  const std::string DEFAULT_INTERFACE_TYPE("tcp");
  const double DEFAULT_POLL_RATE = 10.0;
  const std::string DEFAULT_STATE_TOPIC("robotiq_3_finger_state");
  const std::string DEFAULT_COMMAND_TOPIC("robotiq_3_finger_command");
  // Make the logging function
  std::function<void(const std::string&)> logging_fn
      = [this] (const std::string& message)
  {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  };
  const std::string interface_type
      = this->declare_parameter("interface_type", DEFAULT_INTERFACE_TYPE);
  // Make the interface
  using robotiq_3_finger_gripper_driver::Robotiq3FingerGripperModbusInterface;
  auto gripper_interface =
      std::make_shared<Robotiq3FingerGripperModbusInterface>(logging_fn);
  if (interface_type == "tcp")
  {
    const std::string DEFAULT_GRIPPER_IP_ADDRESS("192.168.1.11");
    const int32_t DEFAULT_GRIPPER_PORT = 502;
    const int32_t DEFAULT_GRIPPER_SLAVE_ID = 0x0002;
    const std::string gripper_ip_address
        = this->declare_parameter(
            "gripper_ip_address", DEFAULT_GRIPPER_IP_ADDRESS);
    const int32_t gripper_port
        = static_cast<int32_t>(this->declare_parameter(
            "gripper_port", DEFAULT_GRIPPER_PORT));
    const uint16_t gripper_slave_id
        = static_cast<uint16_t>(this->declare_parameter(
            "gripper_slave_id", DEFAULT_GRIPPER_SLAVE_ID));

    gripper_interface->ConnectModbusTcp(gripper_ip_address,
                                        gripper_port,
                                        gripper_slave_id);
  }
  else if (interface_type == "rtu")
  {
    const std::string DEFAULT_MODBUS_RTU_INTERFACE("/dev/ttyUSB0");
    const int32_t DEFAULT_GRIPPER_BAUD_RATE = 115200;
    const int32_t DEFAULT_GRIPPER_SLAVE_ID = 0x0009;
    using robotiq_3_finger_gripper_driver::
        Robotiq3FingerGripperModbusInterface;
    const std::string modbus_rtu_interface
        = this->declare_parameter(
            "modbus_rtu_interface", DEFAULT_MODBUS_RTU_INTERFACE);
    const int32_t gripper_baud_rate
        = static_cast<int32_t>(this->declare_parameter(
            "gripper_baud_rate", DEFAULT_GRIPPER_BAUD_RATE));
    const uint16_t gripper_slave_id
        = static_cast<uint16_t>(this->declare_parameter(
            "gripper_slave_id", DEFAULT_GRIPPER_SLAVE_ID));
    gripper_interface->ConnectModbusRtu(modbus_rtu_interface,
                                        gripper_baud_rate,
                                        gripper_slave_id);
  }
  else
  {
    RCLCPP_FATAL(
        this->get_logger(),
        "Invalid interface option [%s], valid options are [tcp] or [rtu]",
        interface_type.c_str());
  }
  gripper_interface_ptr_ = std::move(gripper_interface);

  const std::string status_topic
      = this->declare_parameter("state_topic", DEFAULT_STATE_TOPIC);
  const std::string command_topic
      = this->declare_parameter("command_topic", DEFAULT_COMMAND_TOPIC);

  using std::placeholders::_1;
  status_pub_
      = this->create_publisher<Robotiq3FingerState>(status_topic, 1);
  command_sub_
      = this->create_subscription<Robotiq3FingerCommand>(
          command_topic, 1,
          std::bind(&Robotiq3FingerDriverNode::CommandCB, this, _1));
  const bool success = gripper_interface_ptr_->ActivateGripper();
  if (!success)
  {
    throw std::runtime_error("Unable to initialize gripper");
  }
  gripper_interface_ptr_->Log("Gripper interface running");
  const double poll_rate
      = std::abs(this->declare_parameter("poll_rate", DEFAULT_POLL_RATE));
  poll_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      rclcpp::Duration::from_seconds(1. / poll_rate),
      std::bind(&Robotiq3FingerDriverNode::PublishGripperStatus, this));
}

void
Robotiq3FingerDriverNode::CommandCB(const Robotiq3FingerCommand& command_msg)
{
  try
  {
    const bool sent
        = gripper_interface_ptr_->SendGripperCommand(
            ConvertFingerCommand(command_msg));
    if (!sent)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to send command to gripper");
    }
  }
  catch (const std::invalid_argument& ex)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to command - exception [%s]", ex.what());
  }
}

void Robotiq3FingerDriverNode::PublishGripperStatus()
  {
    Robotiq3FingerState state_msg = ConvertFingerStatus(
        gripper_interface_ptr_->GetGripperStatus());
    state_msg.header.stamp = this->get_clock()->now();
    status_pub_->publish(state_msg);
  }
}  // namespace robotiq_3_finger_gripper_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robotiq_3_finger_gripper_driver::Robotiq3FingerDriverNode)
