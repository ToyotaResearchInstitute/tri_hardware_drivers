#include <dji_robomaster_ep_driver/dji_robomaster_ep_driver.hpp>

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/ros_conversions.hpp>
#include <common_robotics_utilities/ros_helpers.hpp>
#include <common_robotics_utilities/utility.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "dji_robomaster_ep_driver_node.ros2.hpp"

namespace dji_robomaster_ep_driver
{

DJIRobomasterEPDriverNode::DJIRobomasterEPDriverNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("dji_robomaster_ep_driver_node", options)
{
  const std::string robot_ip_address = this->declare_parameter(
      "robot_ip_address", std::string("192.168.42.2"));
  const int32_t robot_port =
      static_cast<int32_t>(this->declare_parameter("robot_port", 40923));
  const std::string robot_name = this->declare_parameter(
      "robot_name", std::string("robomaster_ep"));
  odometry_frame_name_ = this->declare_parameter(
      "odometry_frame_name", std::string("world"));
  robot_frame_name_ = this->declare_parameter(
      "robot_frame_name", std::string("robomaster_body"));
  const double loop_hz = this->declare_parameter("loop_hz", 60.0);

  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      robot_name + "/odometry", 1);
  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);
  battery_percent_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      robot_name + "/battery_percent", 1);
  using std::placeholders::_1;
  auto velocity_callback = std::bind(
      &DJIRobomasterEPDriverNode::VelocityCommandCallback, this, _1);
  velocity_command_sub_ =
      this->create_subscription<geometry_msgs::msg::TwistStamped>(
          robot_name + "/cmd_vel", 1, velocity_callback);

  RCLCPP_INFO(
      this->get_logger(), "Connecting to Robomaster EP at %s:%d...",
      robot_ip_address.c_str(), robot_port);

  // Commands time out after 100ms
  const int32_t safety_timeout_ms = 100;

  robot_interface_ = std::unique_ptr<DJIRobomasterEPInterfaceTCP>(
      new DJIRobomasterEPInterfaceTCP(
          robot_ip_address, robot_port, safety_timeout_ms));

  RCLCPP_INFO(this->get_logger(), "Starting command + status loop");
  robot_interface_->Start();

  loop_timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      rclcpp::Duration::from_seconds(1. / loop_hz),
      std::bind(&DJIRobomasterEPDriverNode::Loop, this));
}

DJIRobomasterEPDriverNode::~DJIRobomasterEPDriverNode()
{
  RCLCPP_INFO(this->get_logger(), "...stopping command + status loop");
  loop_timer_->cancel();
  robot_interface_->Stop();
}

void DJIRobomasterEPDriverNode::Loop()
{
  const auto latest_state = robot_interface_->LatestState();
  if (latest_state.IsValid())
  {
    const auto now_time = this->get_clock()->now();

    nav_msgs::msg::Odometry odometry_message;
    odometry_message.header.frame_id = odometry_frame_name_;
    odometry_message.header.stamp = now_time;
    odometry_message.child_frame_id = robot_frame_name_;
    odometry_message.pose.pose =
        common_robotics_utilities::ros_conversions
            ::EigenIsometry3dToGeometryPose(latest_state.Pose());
    odometry_message.pose.covariance = {{
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    odometry_message.twist.twist.linear.x = latest_state.Velocity()(0);
    odometry_message.twist.twist.linear.y = latest_state.Velocity()(1);
    odometry_message.twist.twist.linear.z = latest_state.Velocity()(2);
    odometry_message.twist.twist.angular.x = latest_state.Velocity()(3);
    odometry_message.twist.twist.angular.y = latest_state.Velocity()(4);
    odometry_message.twist.twist.angular.z = latest_state.Velocity()(5);
    odometry_message.twist.covariance = {{
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms.push_back(
        common_robotics_utilities::ros_conversions
            ::EigenIsometry3dToGeometryTransformStamped(
                latest_state.Pose(), odometry_frame_name_,
                robot_frame_name_));
    common_robotics_utilities::ros_helpers
        ::SetMessageTimestamps<geometry_msgs::msg::TransformStamped>(
            tf_message.transforms, now_time);

    std_msgs::msg::Float64 battery_percent_message;
    battery_percent_message.data = latest_state.BatteryPercent();

    odometry_pub_->publish(odometry_message);
    tf_pub_->publish(tf_message);
    battery_percent_pub_->publish(battery_percent_message);
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "Latest robot state is invalid");
  }
}

void DJIRobomasterEPDriverNode::VelocityCommandCallback(
  const geometry_msgs::msg::TwistStamped& msg)
{
  if (msg.header.frame_id != robot_frame_name_)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Velocity command with frame %s does not match robot frame %s",
        msg.header.frame_id.c_str(), robot_frame_name_.c_str());
    return;
  }

  Twist velocity_command = Twist::Zero();
  velocity_command(0) = msg.twist.linear.x;
  velocity_command(1) = msg.twist.linear.y;
  velocity_command(2) = msg.twist.linear.z;
  velocity_command(3) = msg.twist.angular.x;
  velocity_command(4) = msg.twist.angular.y;
  velocity_command(5) = msg.twist.angular.z;

  robot_interface_->CommandVelocity(velocity_command);
}

}  // namespace dji_robomaster_ep_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(dji_robomaster_ep_driver::DJIRobomasterEPDriverNode)
