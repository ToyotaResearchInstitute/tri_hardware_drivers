#pragma once

#include <dji_robomaster_ep_driver/dji_robomaster_ep_driver.hpp>

#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace dji_robomaster_ep_driver
{
class DJIRobomasterEPDriverNode : public rclcpp::Node
{
public:
  explicit DJIRobomasterEPDriverNode(const rclcpp::NodeOptions& options);

  ~DJIRobomasterEPDriverNode();

  void Loop();

private:
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using TFMessage = tf2_msgs::msg::TFMessage;
  using Float64 = std_msgs::msg::Float64;

  void VelocityCommandCallback(const TwistStamped& msg);

  std::shared_ptr<rclcpp::Subscription<TwistStamped>> velocity_command_sub_;
  std::shared_ptr<rclcpp::Publisher<Odometry>> odometry_pub_;
  std::shared_ptr<rclcpp::Publisher<TFMessage>> tf_pub_;
  std::shared_ptr<rclcpp::Publisher<Float64>> battery_percent_pub_;

  std::shared_ptr<rclcpp::TimerBase> loop_timer_;

  std::unique_ptr<DJIRobomasterEPInterfaceTCP> robot_interface_;
  std::string odometry_frame_name_;
  std::string robot_frame_name_;
};
}  // namespace dji_robomaster_ep_driver
