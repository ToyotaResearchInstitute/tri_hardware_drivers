#pragma once

#if DJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION == 2
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#elif DJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION == 1
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#else
#error "Undefined or unknown DJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION"
#endif

namespace dji_robomaster_ep_driver
{
class ControllerMapping
{
public:
#if DJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION == 2
  using Twist = geometry_msgs::msg::Twist;
  using Joy = sensor_msgs::msg::Joy;
#elif DJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION == 1
  using Twist = geometry_msgs::Twist;
  using Joy = sensor_msgs::Joy;
#endif

  ControllerMapping(double max_linear_velocity, double max_angular_velocity)
    : max_linear_velocity_(max_linear_velocity),
      max_angular_velocity_(max_angular_velocity)
  {
  }

  virtual ~ControllerMapping() = default;

  virtual Twist ComputeVelocityCommand(const Joy& joy_msg) const = 0;

  double max_linear_velocity() const { return max_linear_velocity_; }

  double max_angular_velocity() const { return max_angular_velocity_; }

private:
  const double max_linear_velocity_;
  const double max_angular_velocity_;
};

class XboxOneControllerMapping : public ControllerMapping
{
public:
  using ControllerMapping::ControllerMapping;

  Twist ComputeVelocityCommand(const Joy& joy_msg) const override;

private:
  const size_t kLeftStickUpDown = 1;
  const size_t kLeftStickLeftRight = 0;
  const size_t kLeftTrigger = 2;
  const size_t kRightTrigger = 5;
  const size_t kAButton = 0;

  static double MergeTriggers(
      const double left_trigger, const double right_trigger)
  {
    return ((-left_trigger - 1.0) * 0.5) + ((right_trigger + 1.0) * 0.5);
  }
};

class Logitech3DProControllerMapping : public ControllerMapping
{
public:
  using ControllerMapping::ControllerMapping;

  Twist ComputeVelocityCommand(const Joy& joy_msg) const override;

private:
  const size_t kPitch = 1;
  const size_t kRoll = 0;
  const size_t kYaw = 2;
  const size_t kThumbButton = 1;
};
}  // namespace dji_robomaster_ep_driver
