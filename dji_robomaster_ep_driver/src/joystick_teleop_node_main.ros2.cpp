#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "joystick_teleop_node.ros2.hpp"

int main(int argc, char** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  using dji_robomaster_ep_driver::ControllerTeleopNode;
  rclcpp::spin(
      std::make_shared<ControllerTeleopNode>(
          rclcpp::NodeOptions().arguments(args)));
  rclcpp::shutdown();
  return 0;
}
