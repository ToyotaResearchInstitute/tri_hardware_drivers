#pragma once

#include <memory>

#include <robotiq_ft_driver/robotiq_ft_driver.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace robotiq_ft_driver
{
class RobotiqFTDriverNode : public rclcpp::Node
{
private:
  std::string sensor_frame_;
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;
  std::shared_ptr<rclcpp::Publisher<WrenchStamped>> status_pub_;
  std::unique_ptr<RobotiqFTModbusRtuInterface> sensor_ptr_;
  std::shared_ptr<rclcpp::TimerBase> poll_timer_;

public:
  explicit RobotiqFTDriverNode(const rclcpp::NodeOptions& options);

private:
  void PublishWrench();
};
}  // namespace robotiq_ft_driver
