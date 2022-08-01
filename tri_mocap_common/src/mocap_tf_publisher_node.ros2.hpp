#pragma once

#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tri_mocap_common/msg/mocap_state.hpp>

namespace tri_mocap_common
{
class MocapTFPublisherNode : public rclcpp::Node
{
private:
  using TFMessage = tf2_msgs::msg::TFMessage;
  using MocapState = tri_mocap_common::msg::MocapState;

  std::shared_ptr<rclcpp::Publisher<TFMessage>> tf_pub_;
  std::shared_ptr<rclcpp::Subscription<MocapState>> mocap_sub_;
  bool override_timestamps_ = false;

public:
  explicit MocapTFPublisherNode(const rclcpp::NodeOptions& options);

private:
  void MocapStateCB(const MocapState& msg);
};
}  // namespace tri_mocap_common
