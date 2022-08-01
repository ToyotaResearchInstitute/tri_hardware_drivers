#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "schunk_wsg_driver_node.ros2.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  using schunk_wsg_driver::SchunkWSGDriverNode;
  rclcpp::spin(std::make_shared<SchunkWSGDriverNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
