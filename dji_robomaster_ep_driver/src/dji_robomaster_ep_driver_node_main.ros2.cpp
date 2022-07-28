#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dji_robomaster_ep_driver_node.ros2.hpp"

int main(int argc, char** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  using dji_robomaster_ep_driver::DJIRobomasterEPDriverNode;
  rclcpp::spin(
      std::make_shared<DJIRobomasterEPDriverNode>(
          rclcpp::NodeOptions().arguments(args)));
  rclcpp::shutdown();
  return 0;
}
