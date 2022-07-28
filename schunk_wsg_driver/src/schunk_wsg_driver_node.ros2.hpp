#pragma once

#include <memory>
#include <string>

#include <schunk_wsg_driver/msg/wsg_command.hpp>
#include <schunk_wsg_driver/msg/wsg_state.hpp>
#include <schunk_wsg_driver/schunk_wsg_driver_common.hpp>

#include <rclcpp/rclcpp.hpp>

namespace schunk_wsg_driver
{

class SchunkWSGDriverNode : public rclcpp::Node
{
public:
  explicit SchunkWSGDriverNode(const rclcpp::NodeOptions& options);

  ~SchunkWSGDriverNode();

private:
  using WSGState = schunk_wsg_driver::msg::WSGState;
  using WSGCommand = schunk_wsg_driver::msg::WSGCommand;

  std::shared_ptr<rclcpp::Publisher<WSGState>> status_pub_;
  std::shared_ptr<rclcpp::Subscription<WSGCommand>> command_sub_;
  std::shared_ptr<WSGInterface> gripper_interface_;
  std::shared_ptr<rclcpp::TimerBase> poll_timer_;

  void CommandCB(const WSGCommand& command);

  void PublishGripperStatus();
};

}  // namespace schunk_wsg_driver
