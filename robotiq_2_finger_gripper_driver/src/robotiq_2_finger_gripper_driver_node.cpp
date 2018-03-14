#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <robotiq_2_finger_gripper_driver/robotiq_2_finger_gripper_driver.hpp>
// ROS
#include <ros/ros.h>
#include <robotiq_2_finger_gripper_driver/Robotiq2FingerCommand.h>
#include <robotiq_2_finger_gripper_driver/Robotiq2FingerState.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

namespace robotiq_2_finger_gripper_driver
{
class Robotiq2FingerDriver
{
private:

  ros::NodeHandle nh_;
  ros::Publisher status_pub_;
  ros::Subscriber command_sub_;
  std::unique_ptr<Robotiq2FingerGripperModbusRtuInterface>
    gripper_interface_ptr_;

public:

  Robotiq2FingerDriver(const ros::NodeHandle& nh,
                       const std::string& status_topic,
                       const std::string& command_topic,
                       const std::string& modbus_rtu_interface,
                       const int32_t gripper_baud_rate,
                       const uint16_t gripper_slave_id)
    : nh_(nh)
  {
    status_pub_
        = nh_.advertise<Robotiq2FingerState>(status_topic, 1, false);
    command_sub_
        = nh_.subscribe(command_topic, 1,
                        &Robotiq2FingerDriver::CommandCB, this);
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
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Connecting to Robotiq 2-Finger gripper with gripper slave"
                   " ID %hx on Modbus RTU interface %s at %d baud...",
                   gripper_slave_id,
                   modbus_rtu_interface.c_str(),
                   gripper_baud_rate);
    gripper_interface_ptr_
        = std::unique_ptr<Robotiq2FingerGripperModbusRtuInterface>(
            new Robotiq2FingerGripperModbusRtuInterface(logging_fn,
                                                        modbus_rtu_interface,
                                                        gripper_baud_rate,
                                                        gripper_slave_id));
    const bool success = gripper_interface_ptr_->ActivateGripper();
    if (!success)
    {
      throw std::runtime_error("Unable to initialize gripper");
    }
  }

  void Loop(const double control_rate)
  {
    ROS_INFO_NAMED(ros::this_node::getName(), "Gripper interface running");;
    ros::Rate rate(control_rate);
    while (ros::ok())
    {
      PublishGripperStatus();
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Gripper interface shutting down");
  }

private:

  void CommandCB(Robotiq2FingerCommand command_msg)
  {
    try
    {
      const Robotiq2FingerGripperCommand
          gripper_command(command_msg.percent_closed,
                          command_msg.percent_speed,
                          command_msg.percent_effort);
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
    const Robotiq2FingerGripperStatus status
        = gripper_interface_ptr_->GetGripperStatus();
    Robotiq2FingerState state_msg;
    state_msg.actual_percent_closed = status.ActualPosition();
    state_msg.actual_percent_current = status.ActualCurrent();
    state_msg.target_percent_closed = status.TargetPosition();
    state_msg.header.stamp = ros::Time::now();
    status_pub_.publish(state_msg);
  }
};
}

int main(int argc, char** argv)
{
  // Default ROS params
  const double DEFAULT_POLL_RATE = 10.0;
  const std::string DEFAULT_STATE_TOPIC("robotiq_2_finger_state");
  const std::string DEFAULT_COMMAND_TOPIC("robotiq_2_finger_command");
  const std::string DEFAULT_MODBUS_RTU_INTERFACE("/dev/ttyUSB0");
  const int32_t DEFAULT_GRIPPER_BAUD_RATE = 115200;
  const int32_t DEFAULT_GRIPPER_SLAVE_ID = 0x09;
  // Start ROS
  ros::init(argc, argv, "robotiq_2_finger_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Get params
  const std::string modbus_rtu_interface
      = nhp.param(std::string("modbus_rtu_interface"),
                  DEFAULT_MODBUS_RTU_INTERFACE);
  const int32_t gripper_baud_rate
      = nhp.param(std::string("gripper_baud_rate"),
                  DEFAULT_GRIPPER_BAUD_RATE);
  const uint16_t gripper_slave_id
      = (uint16_t)nhp.param(std::string("gripper_slave_id"),
                            DEFAULT_GRIPPER_SLAVE_ID);
  const double poll_rate
      = std::abs(nhp.param(std::string("poll_rate"), DEFAULT_POLL_RATE));
  const std::string status_topic
      = nhp.param(std::string("state_topic"), DEFAULT_STATE_TOPIC);
  const std::string command_topic
      = nhp.param(std::string("command_topic"), DEFAULT_COMMAND_TOPIC);
  // Start the driver
  robotiq_2_finger_gripper_driver::Robotiq2FingerDriver
        gripper(nh, status_topic, command_topic, modbus_rtu_interface,
                gripper_baud_rate, gripper_slave_id);
  gripper.Loop(poll_rate);
  return 0;
}
