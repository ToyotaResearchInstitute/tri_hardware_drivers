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
      const ros::NodeHandle& nh, const std::string& joy_topic,
      const std::string& command_topic, const std::string& command_frame,
      const double max_linear_velocity, const double max_angular_velocity)
      : nh_(nh), max_linear_velocity_(max_linear_velocity),
        max_angular_velocity_(max_angular_velocity)
  {
    if (max_linear_velocity_ <= 0.0)
    {
      throw std::invalid_argument("max_linear_velocity_ <= 0.0");
    }
    if (max_angular_velocity_ <= 0.0)
    {
      throw std::invalid_argument("max_angular_velocity_ <= 0.0");
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

  double max_linear_velocity() const { return max_linear_velocity_; }

  double max_angular_velocity() const { return max_angular_velocity_; }

protected:
  virtual geometry_msgs::Twist ComputeVelocityCommand(
      const sensor_msgs::Joy& joy_msg) const = 0;

private:
  void JoyCallback(const sensor_msgs::Joy& joy_msg)
  {
    velocity_command_.twist = ComputeVelocityCommand(joy_msg);
    velocity_command_.header.stamp = joy_msg.header.stamp;
  }

  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Subscriber joystick_sub_;
  geometry_msgs::TwistStamped velocity_command_;
  const double max_linear_velocity_;
  const double max_angular_velocity_;
};

class XboxOneControllerTeleop : public ControllerTeleop
{
public:
  XboxOneControllerTeleop(
      const ros::NodeHandle& nh, const std::string& joy_topic,
      const std::string& command_topic, const std::string& command_frame,
      const double max_linear_velocity, const double max_angular_velocity)
      : ControllerTeleop(
          nh, joy_topic, command_topic, command_frame, max_linear_velocity,
          max_angular_velocity) {}

private:
  const size_t kLeftStickUpDown = 1;
  const size_t kLeftStickLeftRight = 0;
  const size_t kLeftTrigger = 2;
  const size_t kRightTrigger = 5;
  const size_t kAButton = 0;

  geometry_msgs::Twist ComputeVelocityCommand(
      const sensor_msgs::Joy& joy_msg) const override
  {
    const double left_stick_up_down = joy_msg.axes.at(kLeftStickUpDown);
    const double left_stick_left_right = joy_msg.axes.at(kLeftStickLeftRight);
    const double left_trigger = joy_msg.axes.at(kLeftTrigger);
    const double right_trigger = joy_msg.axes.at(kRightTrigger);
    const int32_t a_button = joy_msg.buttons.at(kAButton);
    const double merged_triggers = MergeTriggers(left_trigger, right_trigger);

    geometry_msgs::Twist velocity_command;
    if (a_button == 1)
    {
      velocity_command.linear.x = max_linear_velocity() * left_stick_up_down;
      velocity_command.linear.y = max_linear_velocity() * left_stick_left_right;
      velocity_command.linear.z = 0.0;
      velocity_command.angular.x = 0.0;
      velocity_command.angular.y = 0.0;
      velocity_command.angular.z = max_angular_velocity() * merged_triggers;
    }
    else
    {
      velocity_command.linear.x = 0.0;
      velocity_command.linear.y = 0.0;
      velocity_command.linear.z = 0.0;
      velocity_command.angular.x = 0.0;
      velocity_command.angular.y = 0.0;
      velocity_command.angular.z = 0.0;
    }

    return velocity_command;
  }

  static double MergeTriggers(
      const double left_trigger, const double right_trigger)
  {
    return ((-left_trigger - 1.0) * 0.5) + ((right_trigger + 1.0) * 0.5);
  }
};

class Logitech3DProControllerTeleop : public ControllerTeleop
{
public:
  Logitech3DProControllerTeleop(
      const ros::NodeHandle& nh, const std::string& joy_topic,
      const std::string& command_topic, const std::string& command_frame,
      const double max_linear_velocity, const double max_angular_velocity)
      : ControllerTeleop(
          nh, joy_topic, command_topic, command_frame, max_linear_velocity,
          max_angular_velocity) {}

private:
  const size_t kPitch = 1;
  const size_t kRoll = 0;
  const size_t kYaw = 2;
  const size_t kThumbButton = 1;

  geometry_msgs::Twist ComputeVelocityCommand(
      const sensor_msgs::Joy& joy_msg) const override
  {
    const double pitch_axis = joy_msg.axes.at(kPitch);
    const double roll_axis = joy_msg.axes.at(kRoll);
    const double yaw_axis = joy_msg.axes.at(kYaw);
    const int32_t thumb_button = joy_msg.buttons.at(kThumbButton);

    geometry_msgs::Twist velocity_command;
    if (thumb_button == 1)
    {
      velocity_command.linear.x = max_linear_velocity() * pitch_axis;
      velocity_command.linear.y = max_linear_velocity() * roll_axis;
      velocity_command.linear.z = 0.0;
      velocity_command.angular.x = 0.0;
      velocity_command.angular.y = 0.0;
      velocity_command.angular.z = max_angular_velocity() * yaw_axis;
    }
    else
    {
      velocity_command.linear.x = 0.0;
      velocity_command.linear.y = 0.0;
      velocity_command.linear.z = 0.0;
      velocity_command.angular.x = 0.0;
      velocity_command.angular.y = 0.0;
      velocity_command.angular.z = 0.0;
    }

    return velocity_command;
  }
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

  if (joystick_type == "xbox_one")
  {
    ROS_INFO("Creating teleop controller for Xbox One controller");
    dji_robomaster_ep_driver::XboxOneControllerTeleop controller_teleop(
        nh, joystick_topic, command_topic, command_frame,
        max_linear_velocity, max_angular_velocity);
    controller_teleop.Loop(publish_hz);
  }
  else if (joystick_type == "3d_pro")
  {
    ROS_INFO("Creating teleop controller for Logitech 3D PRO controller");
    dji_robomaster_ep_driver::Logitech3DProControllerTeleop controller_teleop(
        nh, joystick_topic, command_topic, command_frame,
        max_linear_velocity, max_angular_velocity);
    controller_teleop.Loop(publish_hz);
  }
  else
  {
    ROS_FATAL("Joystick type [%s] not supported", joystick_type.c_str());
  }

  return 0;
}

