#pragma once

#include <dji_robomaster_ep_driver/joystick_controller_mappings.hpp>

#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace dji_robomaster_ep_driver
{
class ControllerTeleopNode : public rclcpp::Node
{
public:
  explicit ControllerTeleopNode(const rclcpp::NodeOptions& options);

  void PublishCommand();

private:
  void JoyCallback(const sensor_msgs::msg::Joy& joy_msg);

  std::unique_ptr<ControllerMapping> controller_mapping_;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  std::shared_ptr<rclcpp::Publisher<TwistStamped>> command_pub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joystick_sub_;
  std::shared_ptr<rclcpp::TimerBase> publish_timer_;
  geometry_msgs::msg::TwistStamped velocity_command_;
};
}  // namespace dji_robomaster_ep_driver
