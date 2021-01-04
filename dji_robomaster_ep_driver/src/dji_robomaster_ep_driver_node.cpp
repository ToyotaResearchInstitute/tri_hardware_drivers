#include <dji_robomaster_ep_driver/dji_robomaster_ep_driver.hpp>

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/ros_conversions.hpp>
#include <common_robotics_utilities/ros_helpers.hpp>
#include <common_robotics_utilities/utility.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>

namespace dji_robomaster_ep_driver
{
namespace
{
class DJIRobomasterEPDriver
{
public:
  DJIRobomasterEPDriver(
      const ros::NodeHandle& nh, const std::string& robot_ip_address,
      const int32_t robot_port, const std::string& robot_name,
      const std::string& odometry_frame_name,
      const std::string& robot_frame_name)
      : nh_(nh), odometry_frame_name_(odometry_frame_name),
        robot_frame_name_(robot_frame_name)
  {
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(
        robot_name + "/odometry", 1, false);
    tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 1, false);
    battery_percent_pub_ = nh_.advertise<std_msgs::Float64>(
        robot_name + "/battery_percent", 1, false);
    velocity_command_sub_ = nh_.subscribe(
        robot_name + "/cmd_vel", 1,
        &DJIRobomasterEPDriver::VelocityCommandCallback, this);

    ROS_INFO(
        "Connecting to Robomaster EP at %s:%d...",
        robot_ip_address.c_str(), robot_port);

    // Commands time out after 100ms
    const int32_t safety_timeout_ms = 100;

    robot_interface_ = std::unique_ptr<DJIRobomasterEPInterfaceTCP>(
        new DJIRobomasterEPInterfaceTCP(
            robot_ip_address, robot_port, safety_timeout_ms));
  }

  void Loop(const double loop_hz)
  {
    ROS_INFO("Starting command + status loop");
    robot_interface_->Start();
    ros::Rate loop_rate(loop_hz);
    while (ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();

      const auto latest_state = robot_interface_->LatestState();
      if (latest_state.IsValid())
      {
        const auto now_time = ros::Time::now();

        nav_msgs::Odometry odometry_message;
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

        tf2_msgs::TFMessage tf_message;
        tf_message.transforms.push_back(
            common_robotics_utilities::ros_conversions
                ::EigenIsometry3dToGeometryTransformStamped(
                    latest_state.Pose(), odometry_frame_name_,
                    robot_frame_name_));
        common_robotics_utilities::ros_helpers
            ::SetMessageTimestamps<geometry_msgs::TransformStamped>(
                tf_message.transforms, now_time);

        std_msgs::Float64 battery_percent_message;
        battery_percent_message.data = latest_state.BatteryPercent();

        odometry_pub_.publish(odometry_message);
        tf_pub_.publish(tf_message);
        battery_percent_pub_.publish(battery_percent_message);
      }
      else
      {
        ROS_DEBUG("Latest robot state is invalid");
      }
    }
    ROS_INFO("...stopping command + status loop");
    robot_interface_->Stop();
  }

private:
  void VelocityCommandCallback(const geometry_msgs::TwistStamped& msg)
  {
    if (msg.header.frame_id != robot_frame_name_)
    {
      ROS_WARN(
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

  ros::NodeHandle nh_;
  ros::Subscriber velocity_command_sub_;
  ros::Publisher odometry_pub_;
  ros::Publisher tf_pub_;
  ros::Publisher battery_percent_pub_;

  std::unique_ptr<DJIRobomasterEPInterfaceTCP> robot_interface_;
  std::string odometry_frame_name_;
  std::string robot_frame_name_;
};
}  // namespace
}  // namespace dji_robomaster_ep_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dji_robomaster_ep_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  const std::string robot_ip_address = nhp.param(
      std::string("robot_ip_address"), std::string("192.168.42.2"));
  const int32_t robot_port = nhp.param(std::string("robot_port"), 40923);
  const std::string robot_name = nhp.param(
      std::string("robot_name"), std::string("robomaster_ep"));
  const std::string odometry_frame_name = nhp.param(
      std::string("odometry_frame_name"), std::string("world"));
  const std::string robot_frame_name = nhp.param(
      std::string("robot_frame_name"), std::string("robomaster_body"));
  const double loop_hz = nhp.param(std::string("loop_hz"), 60.0);

  dji_robomaster_ep_driver::DJIRobomasterEPDriver driver(
      nh, robot_ip_address, robot_port, robot_name, odometry_frame_name,
      robot_frame_name);
  driver.Loop(loop_hz);
  return 0;
}

