#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mocap_tf_publisher_node.ros2.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  using tri_mocap_common::MocapTFPublisherNode;
  rclcpp::spin(std::make_shared<MocapTFPublisherNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
