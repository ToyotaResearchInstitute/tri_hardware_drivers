#include <memory>

#include <robotiq_2_finger_gripper_driver/robotiq_2_finger_gripper_driver.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <robotiq_2_finger_gripper_driver/msg/robotiq2_finger_command.hpp>
#include <robotiq_2_finger_gripper_driver/msg/robotiq2_finger_state.hpp>

namespace robotiq_2_finger_gripper_driver
{
class Robotiq2FingerDriverNode : public rclcpp::Node
{
private:
  using Robotiq2FingerState =
    robotiq_2_finger_gripper_driver::msg::Robotiq2FingerState;
  std::shared_ptr<rclcpp::Publisher<Robotiq2FingerState>> status_pub_;
  using Robotiq2FingerCommand =
    robotiq_2_finger_gripper_driver::msg::Robotiq2FingerCommand;
  std::shared_ptr<rclcpp::Subscription<Robotiq2FingerCommand>> command_sub_;
  std::unique_ptr<Robotiq2FingerGripperModbusInterface> gripper_interface_ptr_;
  std::shared_ptr<rclcpp::TimerBase> poll_timer_;

public:
  explicit Robotiq2FingerDriverNode(const rclcpp::NodeOptions& options);

private:
  void CommandCB(const Robotiq2FingerCommand& command_msg);

  void PublishGripperStatus();
};
}  // namespace robotiq_2_finger_gripper_driver
