#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/TFMessage.hpp>
#include <tri_mocap_common/msg/MocapState.hpp>

#include "mocap_tf_publisher_node.ros2.hpp"

namespace tri_mocap_common
{
namespace
{
std::string MakeTfCompatibleName(const std::string& name)
{
  const std::pair<char, char> ascii_numbers(0x30, 0x39);
  const std::pair<char, char> ascii_upper_case(0x41, 0x5a);
  const std::pair<char, char> ascii_lower_case(0x61, 0x7a);

  std::ostringstream strm;
  for (size_t idx = 0; idx < name.size(); idx++)
  {
    const char character = name[idx];
    if (character >= ascii_numbers.first && character <= ascii_numbers.second)
    {
      strm << character;
    }
    else if (character >= ascii_upper_case.first
             && character <= ascii_upper_case.second)
    {
      strm << character;
    }
    else if (character >= ascii_lower_case.first
             && character <= ascii_lower_case.second)
    {
      strm << character;
    }
  }
  return strm.str();
}
}  // namespace

MocapTFPublisherNode::MocapTFPublisherNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("mocap_tf_publisher", options)
{
  // Default ROS params
  const std::string DEFAULT_TF_TOPIC("/tf");
  const std::string DEFAULT_TRACKER_TOPIC("mocap_state");
  const bool DEFAULT_OVERRIDE_TIMESTAMPS = false;

  // Get params
  const std::string tf_topic
      = this->declare_parameter("tf_topic", DEFAULT_TF_TOPIC);
  const std::string tracker_topic
      = this->declare_parameter("tracker_topic", DEFAULT_TRACKER_TOPIC);
  override_timestamps_ = this->declare_parameter(
      "override_timestamps", DEFAULT_OVERRIDE_TIMESTAMPS);

  // Make ROS publisher + subscriber
  tf_pub_ = this->create_publisher<TFMessage>(tf_topic, 1);

  using std::placeholders::_1;
  mocap_sub_ = this->create_subscription<MocapState>(
      tracker_topic, 1,
      std::bind(&MocapTFPublisherNode::MocapStateCB, this, _1));
}

void MocapTFPublisherNode::MocapStateCB(const MocapState& msg)
{
  TFMessage transforms_msg;

  const std::string& mocap_name = msg.tracker_name;
  const ros::Time transforms_time =
      (override_timestamps_) ? this->get_clock()->now() : msg.header.stamp;
  const std::string& mocap_frame = msg.header.frame_id;

  for (const auto& current_object : msg.tracked_objects)
  {
    for (const auto& current_segment : current_object.segments)
    {
      if (!current_segment.occluded)
      {
        /* Get the full segment name */
        const std::string full_segment_name
            = mocap_name + "_" + MakeTfCompatibleName(current_object.name)
              + "_" + MakeTfCompatibleName(current_segment.name);

        geometry_msgs::TransformStamped segment_transform;
        segment_transform.header.stamp = transforms_time;
        segment_transform.header.frame_id = mocap_frame;
        segment_transform.child_frame_id = full_segment_name;
        segment_transform.transform = current_segment.transform;

        transforms_msg.transforms.push_back(segment_transform);
      }
    }
  }

  if (transforms_msg.transforms.size() > 0)
  {
    tf_pub_->publish(transforms_msg);
  }
}
}  // namespace tri_mocap_common

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tri_mocap_common::MocapTFPublisherNode)
