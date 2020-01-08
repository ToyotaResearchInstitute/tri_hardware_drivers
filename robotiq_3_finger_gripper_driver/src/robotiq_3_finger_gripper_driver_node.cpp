#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <robotiq_3_finger_gripper_driver/robotiq_3_finger_gripper_driver.hpp>
// ROS
#include <ros/ros.h>
#include <robotiq_3_finger_gripper_driver/Robotiq3FingerCommand.h>
#include <robotiq_3_finger_gripper_driver/Robotiq3FingerState.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

namespace robotiq_3_finger_gripper_driver
{
class Robotiq3FingerDriver
{
private:

  ros::NodeHandle nh_;
  ros::Publisher status_pub_;
  ros::Subscriber command_sub_;
  std::shared_ptr<Robotiq3FingerGripperInterface>
    gripper_interface_ptr_;

public:

  Robotiq3FingerDriver(const ros::NodeHandle& nh,
                       const std::string& status_topic,
                       const std::string& command_topic,
                       const std::shared_ptr<Robotiq3FingerGripperInterface>&
                               gripper_interface_ptr)
    : nh_(nh), gripper_interface_ptr_(gripper_interface_ptr)
  {
    status_pub_
        = nh_.advertise<Robotiq3FingerState>(status_topic, 1, false);
    command_sub_
        = nh_.subscribe(command_topic, 1,
                        &Robotiq3FingerDriver::CommandCB, this);
    const bool success = gripper_interface_ptr_->ActivateGripper();
    if (!success)
    {
      throw std::runtime_error("Unable to initialize gripper");
    }
  }

  void Loop(const double control_rate)
  {
    gripper_interface_ptr_->Log("Gripper interface running");
    ros::Rate rate(control_rate);
    while (ros::ok())
    {
      PublishGripperStatus();
      ros::spinOnce();
      rate.sleep();
    }
    gripper_interface_ptr_->Log("Gripper interface shutting down");
  }

private:

  static Robotiq3FingerGripperActuatorCommand ConvertActuatorCommand(
      const Robotiq3FingerActuatorCommand& command)
  {
    return Robotiq3FingerGripperActuatorCommand(command.percent_closed,
                                                command.percent_speed,
                                                command.percent_effort);
  }

  static Robotiq3FingerActuatorState ConvertActuatorStatus(
      const Robotiq3FingerGripperActuatorStatus& status)
  {
    Robotiq3FingerActuatorState finger_state;
    finger_state.target_percent_closed = status.TargetPosition();
    finger_state.actual_percent_closed = status.ActualPosition();
    finger_state.actual_percent_current = status.ActualCurrent();
    finger_state.object_status = (uint8_t)status.ObjectStatus();
    return finger_state;
  }

  void CommandCB(Robotiq3FingerCommand command_msg)
  {
    try
    {
      const Robotiq3FingerGripperCommand
          gripper_command(ConvertActuatorCommand(command_msg.finger_a_command),
                          ConvertActuatorCommand(command_msg.finger_b_command),
                          ConvertActuatorCommand(command_msg.finger_c_command),
                          ConvertActuatorCommand(command_msg.scissor_command));
      const bool sent
          = gripper_interface_ptr_->SendGripperCommand(gripper_command);
      if (!sent)
      {
        ROS_ERROR_NAMED(ros::this_node::getName(),
                        "Failed to send command to gripper");
      }
    }
    catch (std::invalid_argument ex)
    {
      ROS_ERROR_NAMED(ros::this_node::getName(),
                      "Failed to command - exception [%s]",
                      ex.what());
    }
  }

  void PublishGripperStatus()
  {
    const Robotiq3FingerGripperStatus status
        = gripper_interface_ptr_->GetGripperStatus();
    Robotiq3FingerState state_msg;
    state_msg.finger_a_state = ConvertActuatorStatus(status.FingerAStatus());
    state_msg.finger_b_state = ConvertActuatorStatus(status.FingerBStatus());
    state_msg.finger_c_state = ConvertActuatorStatus(status.FingerCStatus());
    state_msg.scissor_state = ConvertActuatorStatus(status.ScissorStatus());
    state_msg.gripper_activation_state
        = (uint8_t)status.GripperActivationStatus();
    state_msg.gripper_mode_state = (uint8_t)status.GripperModeStatus();
    state_msg.gripper_action_state = (uint8_t)status.GripperActionStatus();
    state_msg.gripper_system_state = (uint8_t)status.GripperSystemStatus();
    state_msg.gripper_motion_state = (uint8_t)status.GripperMotionStatus();
    state_msg.gripper_fault_state = (uint8_t)status.GripperFaultStatus();
    state_msg.header.stamp = ros::Time::now();
    status_pub_.publish(state_msg);
  }
};
}

int main(int argc, char** argv)
{
  // Default ROS params
  const std::string DEFAULT_INTERFACE_TYPE("tcp");
  const double DEFAULT_POLL_RATE = 10.0;
  const std::string DEFAULT_STATE_TOPIC("robotiq_2_finger_state");
  const std::string DEFAULT_COMMAND_TOPIC("robotiq_2_finger_command");
  // Start ROS
  ros::init(argc, argv, "robotiq_3_finger_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Get params
  const std::string interface_type
      = nhp.param(std::string("interface_type"), DEFAULT_INTERFACE_TYPE);
  const double poll_rate
      = std::abs(nhp.param(std::string("poll_rate"), DEFAULT_POLL_RATE));
  const std::string status_topic
      = nhp.param(std::string("state_topic"), DEFAULT_STATE_TOPIC);
  const std::string command_topic
      = nhp.param(std::string("command_topic"), DEFAULT_COMMAND_TOPIC);

  // Make the logging function
  std::function<void(const std::string&)> logging_fn
      = [] (const std::string& message)
  {
    if (ros::ok())
    {
      ROS_INFO_NAMED(ros::this_node::getName(), "%s", message.c_str());
    }
    else
    {
      std::cout << "[Post-shutdown] " << message << std::endl;
    }
  };
  if (interface_type == "tcp")
  {
    const std::string DEFAULT_GRIPPER_IP_ADDRESS("192.168.1.11");
    const int32_t DEFAULT_GRIPPER_PORT = 502;
    const int32_t DEFAULT_GRIPPER_SLAVE_ID = 0x0002;
    using robotiq_3_finger_gripper_driver::Robotiq3FingerGripperModbusInterface;
    const std::string gripper_ip_address
        = nhp.param(std::string("gripper_ip_address"),
                    DEFAULT_GRIPPER_IP_ADDRESS);
    const int32_t gripper_port
        = nhp.param(std::string("gripper_port"),
                    DEFAULT_GRIPPER_PORT);
    const uint16_t gripper_slave_id
        = (uint16_t)nhp.param(std::string("gripper_slave_id"),
                              DEFAULT_GRIPPER_SLAVE_ID);
    // Make the interface
    std::shared_ptr<Robotiq3FingerGripperModbusInterface> gripper_interface_ptr(
          new Robotiq3FingerGripperModbusInterface(logging_fn));
    gripper_interface_ptr->ConnectModbusTcp(gripper_ip_address,
                                            gripper_port,
                                            gripper_slave_id);
    // Start the driver
    robotiq_3_finger_gripper_driver::Robotiq3FingerDriver
          gripper(nh, status_topic, command_topic, gripper_interface_ptr);
    gripper.Loop(poll_rate);
  }
  else if (interface_type == "rtu")
  {
    const std::string DEFAULT_MODBUS_RTU_INTERFACE("/dev/ttyUSB0");
    const int32_t DEFAULT_GRIPPER_BAUD_RATE = 115200;
    const int32_t DEFAULT_GRIPPER_SLAVE_ID = 0x0009;
    using robotiq_3_finger_gripper_driver::Robotiq3FingerGripperModbusInterface;
    const std::string modbus_rtu_interface
        = nhp.param(std::string("modbus_rtu_interface"),
                    DEFAULT_MODBUS_RTU_INTERFACE);
    const int32_t gripper_baud_rate
        = nhp.param(std::string("gripper_baud_rate"),
                    DEFAULT_GRIPPER_BAUD_RATE);
    const uint16_t gripper_slave_id
        = (uint16_t)nhp.param(std::string("gripper_slave_id"),
                              DEFAULT_GRIPPER_SLAVE_ID);
    // Make the interface
    std::shared_ptr<Robotiq3FingerGripperModbusInterface> gripper_interface_ptr(
          new Robotiq3FingerGripperModbusInterface(logging_fn));
    gripper_interface_ptr->ConnectModbusRtu(modbus_rtu_interface,
                                            gripper_baud_rate,
                                            gripper_slave_id);
    // Start the driver
    robotiq_3_finger_gripper_driver::Robotiq3FingerDriver
          gripper(nh, status_topic, command_topic, gripper_interface_ptr);
    gripper.Loop(poll_rate);
  }
  else
  {
    ROS_FATAL_NAMED(ros::this_node::getName(),
                    "Invalid interface option [%s], valid options are [tcp]"
                    " or [rtu]", interface_type.c_str());
  }
  return 0;
}
