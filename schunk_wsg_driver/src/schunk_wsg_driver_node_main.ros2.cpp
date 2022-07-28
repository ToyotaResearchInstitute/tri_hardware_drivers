#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "schunk_wsg_driver_node.ros2.hpp"

int main(int argc, char** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::spin(
      std::make_shared<schunk_wsg_driver::SchunkWSGDriverNode>(
          rclcpp::NodeOptions().arguments(args)));
  rclcpp::shutdown();
  return 0;
}
