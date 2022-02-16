#include <schunk_wsg_driver/schunk_wsg_driver_common.hpp>
#include <schunk_wsg_driver/schunk_wsg_driver_ethernet.hpp>
#include <schunk_wsg_driver/schunk_wsg_driver_can.hpp>
#include <schunk_wsg_driver/msg/wsg_command.hpp>
#include <schunk_wsg_driver/msg/wsg_state.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>

#include "schunk_wsg_driver_node.ros2.hpp"

namespace schunk_wsg_driver
{

SchunkWSGDriverNode::SchunkWSGDriverNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("schunk_wsg_driver", options)
{
  // Default ROS params
  const std::string DEFAULT_INTERFACE_TYPE("udp");
  const double DEFAULT_CONTROL_RATE = 10.0;
  const std::string DEFAULT_COMMAND_TOPIC("schunk_wsg_gripper_command");
  const std::string DEFAULT_STATE_TOPIC("schunk_wsg_gripper_state");
  const std::string DEFAULT_GRIPPER_IP_ADDRESS("172.31.1.121");
  const int32_t DEFAULT_GRIPPER_PORT = 1500;
  const int32_t DEFAULT_LOCAL_PORT = 1501;
  const std::string DEFAULT_SOCKETCAN_INTERFACE("can0");
  const int32_t DEFAULT_GRIPPER_BASE_CAN_ID = 0x000;
  // Make the logging function
  std::function<void(const std::string&)> logging_fn
      = [this] (const std::string& message)
  {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  };
  // Make the interface
  const std::string interface_type
      = this->declare_parameter("interface_type", DEFAULT_INTERFACE_TYPE);
  if (interface_type == "udp")
  {
    const std::string gripper_ip_address
        = this->declare_parameter("gripper_ip_address",
                                  DEFAULT_GRIPPER_IP_ADDRESS);
    const uint16_t gripper_port
        = static_cast<uint16_t>(
            this->declare_parameter("gripper_port", DEFAULT_GRIPPER_PORT));
    const uint16_t local_port
        = static_cast<uint16_t>(
            this->declare_parameter("local_port", DEFAULT_LOCAL_PORT));
    gripper_interface_
        = std::make_shared<schunk_wsg_driver::WSGUDPInterface>(
            logging_fn, gripper_ip_address, gripper_port, local_port);
  }
  else if (interface_type == "can")
  {
    const std::string can_interface
        = this->declare_parameter("socketcan_interface",
                                  DEFAULT_SOCKETCAN_INTERFACE);
    const uint32_t gripper_send_can_id
        = static_cast<uint32_t>(
            this->declare_parameter("gripper_base_can_id",
                                    DEFAULT_GRIPPER_BASE_CAN_ID));
    gripper_interface_
        = std::make_shared<schunk_wsg_driver::WSGCANInterface>(
            logging_fn, can_interface, gripper_send_can_id);
  }
  else
  {
    RCLCPP_FATAL(
        this->get_logger(),
        "Invalid interface option [%s], "
        "valid options are [udp] or [can]",
        interface_type.c_str());
  }

  using std::placeholders::_1;
  const std::string status_topic
      = this->declare_parameter("status_topic", DEFAULT_STATE_TOPIC);
  const std::string command_topic
      = this->declare_parameter("command_topic", DEFAULT_COMMAND_TOPIC);
  status_pub_ = this->create_publisher<WSGState>(status_topic, 1);
  command_sub_
      = this->create_subscription<WSGCommand>(
          command_topic, 1,
          std::bind(&SchunkWSGDriverNode::CommandCB, this, _1));

  const bool success = gripper_interface_->InitializeGripper();
  if (!success)
  {
    throw std::invalid_argument("Unable to initialize gripper");
  }
  gripper_interface_->Log("Gripper interface running");

  const double control_rate
      = std::abs(this->declare_parameter(
          "control_rate", DEFAULT_CONTROL_RATE));
  poll_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      rclcpp::Duration::from_seconds(1. / control_rate),
      std::bind(&SchunkWSGDriverNode::PublishGripperStatus, this));
}

SchunkWSGDriverNode::~SchunkWSGDriverNode()
{
   gripper_interface_->Log("Gripper interface shutting down");
}

void SchunkWSGDriverNode::CommandCB(
    const schunk_wsg_driver::msg::WSGCommand& command)
{
  const double target_position = std::abs(command.target_position);
  const double max_speed = std::abs(command.max_speed);
  const double max_effort = std::abs(command.max_effort);
  const bool sent
      = gripper_interface_->SetTargetPositionSpeedEffort(
          target_position, max_speed, max_effort);
  if (!sent)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to send command to gripper");
  }
}

void SchunkWSGDriverNode::PublishGripperStatus()
{
  gripper_interface_->RefreshGripperStatus();
  const GripperMotionStatus status
      = gripper_interface_->GetGripperStatus();
  schunk_wsg_driver::msg::WSGState state_msg;
  state_msg.actual_position = status.ActualPosition();
  state_msg.actual_velocity = status.ActualVelocity();
  state_msg.actual_effort = status.ActualEffort();
  state_msg.target_position = status.TargetPosition();
  state_msg.max_speed = status.MaxSpeed();
  state_msg.max_effort = status.MaxEffort();
  state_msg.header.stamp = this->get_clock()->now();
  status_pub_->publish(state_msg);
}

}  // namespace schunk_wsg_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(schunk_wsg_driver::SchunkWSGDriverNode)
