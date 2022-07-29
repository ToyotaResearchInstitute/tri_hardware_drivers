#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robotiq_3_finger_gripper_driver_node.ros2.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  using robotiq_3_finger_gripper_driver::Robotiq3FingerDriverNode;
  rclcpp::spin(
      std::make_shared<Robotiq3FingerDriverNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
