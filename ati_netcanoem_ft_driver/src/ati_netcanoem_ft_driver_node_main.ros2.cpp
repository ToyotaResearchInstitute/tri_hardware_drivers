#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ati_netcanoem_ft_driver_node.ros2.hpp"

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  using ati_netcanoem_ft_driver::AtiNetCanOemDriverNode;
  rclcpp::spin(
      std::make_shared<AtiNetCanOemDriverNode>(
          rclcpp::NodeOptions().arguments(args)));
  rclcpp::shutdown();
  return 0;
}
