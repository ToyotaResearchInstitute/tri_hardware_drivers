#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robotiq_ft_driver_node.ros2.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  using robotiq_ft_driver::RobotiqFTDriverNode;
  rclcpp::spin(std::make_shared<RobotiqFTDriverNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
