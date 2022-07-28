#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mocap_tf_publisher_node.ros2.hpp"

int main(int argc, char** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::spin(
      std::make_shared<tri_mocap_common::MocapTFPublisherNode>(
          rclcpp::NodeOptions().arguments(args)));
  rclcpp::shutdown();
  return 0;
}
