#include <dji_robomaster_ep_driver/joystick_controller_mappings.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace dji_robomaster_ep_driver
{
class ControllerTeleopNode : public rclcpp::Node
{
public:
  explicit ControllerTeleopNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("joystick_teleop_node", options)
  {
    const double max_linear_velocity =
        this->declare_parameter("max_linear_velocity", 3.5);
    if (max_linear_velocity <= 0.0)
    {
      throw std::runtime_error("max_linear_velocity_ <= 0.0");
    }
    const double max_angular_velocity =
        this->declare_parameter("max_angular_velocity", 10.0);
    if (max_angular_velocity <= 0.0)
    {
      throw std::runtime_error("max_angular_velocity_ <= 0.0");
    }
    // Joystick type, options are "xbox_one"  or "3d_pro".
    const std::string joystick_type = this->declare_parameter(
        "joystick_type", std::string("xbox_one"));
    if (joystick_type == "xbox_one")
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Configured teleop controller for Xbox One controller");
      controller_mapping_ = std::make_unique<XboxOneControllerMapping>(
          max_linear_velocity, max_angular_velocity);
    }
    else if (joystick_type == "3d_pro")
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Configured teleop controller for Logitech 3D PRO controller");
      controller_mapping_ = std::make_unique<Logitech3DProControllerMapping>(
          max_linear_velocity, max_angular_velocity);
    }
    else
    {
      throw std::runtime_error(
          "Joystick type [" + joystick_type + "] not supported");
    }

    const std::string joystick_topic = this->declare_parameter(
        "joystick_topic", std::string("joy"));
    const std::string command_topic = this->declare_parameter(
        "command_topic", std::string("robomaster_ep/cmd_vel"));
    const std::string command_frame = this->declare_parameter(
        "command_frame", std::string("robomaster_body"));
    const double publish_hz = this->declare_parameter("publish_hz", 30.0);

    velocity_command_.header.frame_id = command_frame;
    command_pub_ = this->create_publisher<TwistStamped>(command_topic, 1);
    using std::placeholders::_1;
    auto joystick_callback =
        std::bind(&ControllerTeleopNode::JoyCallback, this, _1);
    joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joystick_topic, 1, joystick_callback);

    publish_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(1. / publish_hz),
        std::bind(&ControllerTeleopNode::PublishCommand, this));
  }

  void PublishCommand()
  {
    command_pub_->publish(velocity_command_);
  }

private:
  void JoyCallback(const sensor_msgs::msg::Joy& joy_msg)
  {
    velocity_command_.twist =
        controller_mapping_->ComputeVelocityCommand(joy_msg);
    velocity_command_.header.stamp = joy_msg.header.stamp;
  }

  std::unique_ptr<ControllerMapping> controller_mapping_;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  std::shared_ptr<rclcpp::Publisher<TwistStamped>> command_pub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joystick_sub_;
  std::shared_ptr<rclcpp::TimerBase> publish_timer_;
  geometry_msgs::msg::TwistStamped velocity_command_;
};
}  // namespace dji_robomaster_ep_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(dji_robomaster_ep_driver::ControllerTeleopNode)
