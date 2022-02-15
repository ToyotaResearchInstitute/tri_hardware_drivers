#include <memory>

#include <robotiq_3_finger_gripper_driver/robotiq_3_finger_gripper_driver.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <robotiq_3_finger_gripper_driver/msg/robotiq3_finger_command.hpp>
#include <robotiq_3_finger_gripper_driver/msg/robotiq3_finger_state.hpp>

#include "type_conversions.hpp"

namespace robotiq_3_finger_gripper_driver
{
class Robotiq3FingerDriverNode : public rclcpp::Node
{
private:
  std::shared_ptr<rclcpp::Publisher<Robotiq3FingerState>> status_pub_;
  std::shared_ptr<rclcpp::Subscription<Robotiq3FingerCommand>> command_sub_;
  std::shared_ptr<Robotiq3FingerGripperInterface> gripper_interface_ptr_;
  std::shared_ptr<rclcpp::TimerBase> poll_timer_;

public:
  explicit Robotiq3FingerDriverNode(const rclcpp::NodeOptions& options);

private:
  void CommandCB(const Robotiq3FingerCommand& command_msg);

  void PublishGripperStatus();
};
}  // namespace robotiq_3_finger_gripper_driver
