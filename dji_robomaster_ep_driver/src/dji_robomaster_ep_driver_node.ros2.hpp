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
  void VelocityCommandCallback(const geometry_msgs::msg::TwistStamped& msg);

  using VelocityCommandSubscription =
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>;
  std::shared_ptr<VelocityCommandSubscription> velocity_command_sub_;

  using OdometryPublisher = rclcpp::Publisher<nav_msgs::msg::Odometry>;
  std::shared_ptr<OdometryPublisher> odometry_pub_;

  using TFMessagePublisher = rclcpp::Publisher<tf2_msgs::msg::TFMessage>;
  std::shared_ptr<TFMessagePublisher> tf_pub_;

  using Float64Publisher = rclcpp::Publisher<std_msgs::msg::Float64>;
  std::shared_ptr<Float64Publisher> battery_percent_pub_;

  std::shared_ptr<rclcpp::TimerBase> loop_timer_;

  std::unique_ptr<DJIRobomasterEPInterfaceTCP> robot_interface_;
  std::string odometry_frame_name_;
  std::string robot_frame_name_;
};
}  // namespace dji_robomaster_ep_driver
