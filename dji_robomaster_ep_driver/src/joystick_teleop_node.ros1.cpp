#include <dji_robomaster_ep_driver/joystick_controller_mappings.hpp>

#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace dji_robomaster_ep_driver
{
namespace
{
class ControllerTeleop
{
public:
  ControllerTeleop(
      const ros::NodeHandle& nh, const std::string& joy_type,
      const std::string& joy_topic, const std::string& command_topic,
      const std::string& command_frame, const double max_linear_velocity,
      const double max_angular_velocity)
      : nh_(nh)
  {
    if (max_linear_velocity <= 0.0)
    {
      throw std::invalid_argument("max_linear_velocity_ <= 0.0");
    }
    if (max_angular_velocity <= 0.0)
    {
      throw std::invalid_argument("max_angular_velocity_ <= 0.0");
    }

    if (joy_type == "xbox_one")
    {
      ROS_INFO("Configured teleop controller for Xbox One controller");
      controller_mapping_ = std::unique_ptr<ControllerMapping>(
          new XboxOneControllerMapping(max_linear_velocity,
                                       max_angular_velocity));
    }
    else if (joy_type == "3d_pro")
    {
      ROS_INFO("Configured teleop controller for Logitech 3D PRO controller");
      controller_mapping_ = std::unique_ptr<ControllerMapping>(
          new Logitech3DProControllerMapping(max_linear_velocity,
                                             max_angular_velocity));
    }
    else
    {
      throw std::invalid_argument(
          "Joystick type [" + joy_type + "] not supported");
    }

    velocity_command_.header.frame_id = command_frame;
    command_pub_ =
        nh_.advertise<geometry_msgs::TwistStamped>(command_topic, 1, false);
    joystick_sub_ = nh_.subscribe(
        joy_topic, 1, &ControllerTeleop::JoyCallback, this);
  }

  virtual ~ControllerTeleop() {}

  void Loop(const double loop_hz)
  {
    ros::Rate loop_rate(loop_hz);
    while (ros::ok())
    {
      ros::spinOnce();
      command_pub_.publish(velocity_command_);
      loop_rate.sleep();
    }
  }

private:
  void JoyCallback(const sensor_msgs::Joy& joy_msg)
  {
    velocity_command_.twist =
        controller_mapping_->ComputeVelocityCommand(joy_msg);
    velocity_command_.header.stamp = joy_msg.header.stamp;
  }

  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Subscriber joystick_sub_;
  geometry_msgs::TwistStamped velocity_command_;
  std::unique_ptr<ControllerMapping> controller_mapping_;
};
}  // namespace
}  // namespace dji_robomaster_ep_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_teleop_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Joystick type, options are "xbox_one"  or "3d_pro".
  const std::string joystick_type =
      nhp.param(std::string("joystick_type"), std::string("xbox_one"));
  const std::string joystick_topic =
      nhp.param(std::string("joystick_topic"), std::string("joy"));
  const std::string command_topic = nhp.param(
      std::string("command_topic"), std::string("robomaster_ep/cmd_vel"));
  const std::string command_frame =
      nhp.param(std::string("command_frame"), std::string("robomaster_body"));

  const double max_linear_velocity =
      nhp.param(std::string("max_linear_velocity"), 3.5);
  const double max_angular_velocity =
      nhp.param(std::string("max_angular_velocity"), 10.0);
  const double publish_hz = nhp.param(std::string("publish_hz"), 30.0);

  dji_robomaster_ep_driver::ControllerTeleop controller_teleop(
      nh, joystick_type, joystick_topic, command_topic,
      command_frame, max_linear_velocity, max_angular_velocity);
  controller_teleop.Loop(publish_hz);

  return 0;
}

