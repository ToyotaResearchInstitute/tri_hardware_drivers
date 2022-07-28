#include <dji_robomaster_ep_driver/joystick_controller_mappings.hpp>

namespace dji_robomaster_ep_driver
{

#if DJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION == 2
  using Twist = geometry_msgs::msg::Twist;
#elif DJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION == 1
  using Twist = geometry_msgs::Twist;
#endif

Twist
XboxOneControllerMapping::ComputeVelocityCommand(const Joy& joy_msg) const
{
  const double left_stick_up_down = joy_msg.axes.at(kLeftStickUpDown);
  const double left_stick_left_right = joy_msg.axes.at(kLeftStickLeftRight);
  const double left_trigger = joy_msg.axes.at(kLeftTrigger);
  const double right_trigger = joy_msg.axes.at(kRightTrigger);
  const int32_t a_button = joy_msg.buttons.at(kAButton);
  const double merged_triggers = MergeTriggers(left_trigger, right_trigger);

  Twist velocity_command;
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

Twist
Logitech3DProControllerMapping::ComputeVelocityCommand(const Joy& joy_msg) const
{
  const double pitch_axis = joy_msg.axes.at(kPitch);
  const double roll_axis = joy_msg.axes.at(kRoll);
  const double yaw_axis = joy_msg.axes.at(kYaw);
  const int32_t thumb_button = joy_msg.buttons.at(kThumbButton);

  Twist velocity_command;
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

}
