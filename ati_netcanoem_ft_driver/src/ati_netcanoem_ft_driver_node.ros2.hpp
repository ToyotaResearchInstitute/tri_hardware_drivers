#pragma once

#include <memory>
#include <string>
#include <ati_netcanoem_ft_driver/ati_netcanoem_ft_driver.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace ati_netcanoem_ft_driver
{
class AtiNetCanOemDriverNode : public rclcpp::Node
{
public:
  explicit AtiNetCanOemDriverNode(const rclcpp::NodeOptions& options);

private:
  void ResetOrSetBiasCallback(
      std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  void PublishWrench();

  std::string sensor_frame_;
  std::shared_ptr<
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> status_pub_;
  std::shared_ptr<
    rclcpp::Service<std_srvs::srv::SetBool>> reset_or_set_bias_service_;
  std::shared_ptr<rclcpp::TimerBase> poll_timer_;
  std::unique_ptr<AtiNetCanOemInterface> sensor_ptr_;
};
}
