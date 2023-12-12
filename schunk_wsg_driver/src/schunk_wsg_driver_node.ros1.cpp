#include <stdlib.h>
#include <stdio.h>
#include <schunk_wsg_driver/schunk_wsg_driver_common.hpp>
#include <schunk_wsg_driver/schunk_wsg_driver_ethernet.hpp>
#include <schunk_wsg_driver/schunk_wsg_driver_can.hpp>
#include <schunk_wsg_driver/WSGCommand.h>
#include <schunk_wsg_driver/WSGState.h>
// ROS
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

namespace schunk_wsg_driver
{
class SchunkWSGDriver
{
private:

  ros::NodeHandle nh_;
  ros::Subscriber command_sub_;
  ros::Publisher status_pub_;

  std::shared_ptr<WSGInterface> gripper_interface_ptr_;

public:

  SchunkWSGDriver(const ros::NodeHandle& nh,
                  const std::shared_ptr<WSGInterface>& gripper_interface,
                  const std::string& command_topic,
                  const std::string& status_topic,
                  const uint16_t update_period_ms)
    : nh_(nh), gripper_interface_ptr_(gripper_interface)
  {
    status_pub_ = nh_.advertise<WSGState>(status_topic, 1, false);
    command_sub_
        = nh_.subscribe(command_topic, 1, &SchunkWSGDriver::CommandCB, this);
    const bool success
        = gripper_interface_ptr_->InitializeGripper(update_period_ms);
    if (!success)
    {
      throw std::invalid_argument("Unable to initialize gripper");
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

  void CommandCB(WSGCommand command)
  {
    const double target_position = std::abs(command.target_position);
    const double max_speed = std::abs(command.max_speed);
    const double max_effort = std::abs(command.max_effort);
    const bool sent
        = gripper_interface_ptr_->SetTargetPositionSpeedEffort(target_position,
                                                               max_speed,
                                                               max_effort);
    if (!sent)
    {
      ROS_ERROR("Failed to send command to gripper");
    }
  }

  void PublishGripperStatus()
  {
    gripper_interface_ptr_->RefreshGripperStatus();
    const GripperMotionStatus status
        = gripper_interface_ptr_->GetGripperStatus();
    WSGState state_msg;
    state_msg.actual_position = status.ActualPosition();
    state_msg.actual_velocity = status.ActualVelocity();
    state_msg.actual_effort = status.ActualEffort();
    state_msg.target_position = status.TargetPosition();
    state_msg.max_speed = status.MaxSpeed();
    state_msg.max_effort = status.MaxEffort();
    state_msg.header.stamp = ros::Time::now();
    status_pub_.publish(state_msg);
  }
};
}

int main(int argc, char** argv)
{
  // Default ROS params
  const std::string DEFAULT_INTERFACE_TYPE("udp");
  const int32_t DEFAULT_UPDATE_PERIOD_MS = 20;
  const double DEFAULT_CONTROL_RATE = 10.0;
  const std::string DEFAULT_COMMAND_TOPIC("schunk_wsg_gripper_command");
  const std::string DEFAULT_STATE_TOPIC("schunk_wsg_gripper_state");
  const std::string DEFAULT_GRIPPER_IP_ADDRESS("172.31.1.121");
  const int32_t DEFAULT_GRIPPER_PORT = 1500;
  const int32_t DEFAULT_LOCAL_PORT = 1501;
  const std::string DEFAULT_SOCKETCAN_INTERFACE("can0");
  const int32_t DEFAULT_GRIPPER_BASE_CAN_ID = 0x000;
  // Start ROS
  ros::init(argc, argv, "schunk_wsg_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Get params
  const std::string interface_type
      = nhp.param(std::string("interface_type"), DEFAULT_INTERFACE_TYPE);
  const uint16_t update_period_ms = static_cast<uint16_t>(std::abs(
      nhp.param(std::string("update_period_ms"), DEFAULT_UPDATE_PERIOD_MS)));
  const double control_rate
      = std::abs(nhp.param(std::string("control_rate"), DEFAULT_CONTROL_RATE));
  const std::string command_topic
      = nhp.param(std::string("command_topic"), DEFAULT_COMMAND_TOPIC);
  const std::string status_topic
      = nhp.param(std::string("status_topic"), DEFAULT_STATE_TOPIC);
  // Make the logging function
  std::function<void(const std::string&)> logging_fn
      = [] (const std::string& message)
  {
    if (ros::ok())
    {
      ROS_INFO("%s", message.c_str());
    }
    else
    {
      std::cout << "[Post-shutdown] " << message << std::endl;
    }
  };
  if (interface_type == "tcp" || interface_type == "udp")
  {
    const std::string gripper_ip_address
        = nhp.param(std::string("gripper_ip_address"),
                    DEFAULT_GRIPPER_IP_ADDRESS);
    const uint16_t gripper_port
        = static_cast<uint16_t>(
            nhp.param(std::string("gripper_port"), DEFAULT_GRIPPER_PORT));
    std::shared_ptr<schunk_wsg_driver::WSGInterface> gripper_interface;
    if (interface_type == "tcp")
    {
      gripper_interface
          = std::make_shared<schunk_wsg_driver::WSGTCPInterface>(
              logging_fn, gripper_ip_address, gripper_port, local_port);
    }
    else if (interface_type == "udp")
    {
      const uint16_t local_port
          = static_cast<uint16_t>(
              nhp.param(std::string("local_port"), DEFAULT_LOCAL_PORT));
      gripper_interface
          = std::make_shared<schunk_wsg_driver::WSGUDPInterface>(
              logging_fn, gripper_ip_address, gripper_port, local_port);
    }
    schunk_wsg_driver::SchunkWSGDriver gripper(nh,
                                               gripper_interface,
                                               command_topic,
                                               status_topic,
                                               update_period_ms);
    gripper.Loop(control_rate);
  }
  else if (interface_type == "can")
  {
    const std::string can_interface
        = nhp.param(std::string("socketcan_interface"),
                    DEFAULT_SOCKETCAN_INTERFACE);
    const uint32_t gripper_send_can_id
        = static_cast<uint32_t>(
            nhp.param(std::string("gripper_base_can_id"),
                      DEFAULT_GRIPPER_BASE_CAN_ID));
    std::shared_ptr<schunk_wsg_driver::WSGCANInterface> gripper_interface(
          new schunk_wsg_driver::WSGCANInterface(logging_fn,
                                                 can_interface,
                                                 gripper_send_can_id));
    schunk_wsg_driver::SchunkWSGDriver gripper(nh,
                                               gripper_interface,
                                               command_topic,
                                               status_topic,
                                               update_period_ms);
    gripper.Loop(control_rate);
  }
  else
  {
    ROS_FATAL("Invalid interface option [%s], valid options are [udp] or [can]",
              interface_type.c_str());
  }
  return 0;
}
