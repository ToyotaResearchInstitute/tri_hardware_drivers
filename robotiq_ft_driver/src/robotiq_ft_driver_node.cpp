#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <robotiq_ft_driver/robotiq_ft_driver.hpp>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

namespace robotiq_ft_driver
{
class RobotiqFTDriver
{
private:

  ros::NodeHandle nh_;
  std::string sensor_frame_;
  ros::Publisher status_pub_;
  std::unique_ptr<RobotiqFTModbusRtuInterface> sensor_ptr_;

public:

  RobotiqFTDriver(const ros::NodeHandle& nh,
                  const std::string& status_topic,
                  const std::string& sensor_frame,
                  const std::string& modbus_rtu_interface,
                  const uint16_t sensor_slave_id)
    : nh_(nh), sensor_frame_(sensor_frame)
  {
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
                   "Connecting to Robotiq F/T sensor with sensor slave ID %hx"
                   " on Modbus RTU interface %s...",
                   sensor_slave_id,
                   modbus_rtu_interface.c_str());
    sensor_ptr_ = std::unique_ptr<RobotiqFTModbusRtuInterface>(
                    new RobotiqFTModbusRtuInterface(logging_fn,
                                                    modbus_rtu_interface,
                                                    sensor_slave_id));
    const std::string serial_num = sensor_ptr_->ReadSerialNumber();
    const std::string firmware_version = sensor_ptr_->ReadFirmwareVersion();
    const uint16_t year = sensor_ptr_->ReadYearOfManufacture();
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Connected to sensor with serial # %s firmware version %s"
                   " year of manufacture %hu",
                   serial_num.c_str(),
                   firmware_version.c_str(),
                   year);
    status_pub_
        = nh_.advertise<geometry_msgs::WrenchStamped>(status_topic, 1, false);
  }

  void Loop(const double poll_rate)
  {
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Starting to stream F/T measurements at %f Hz...",
                   poll_rate);
    ros::Rate rate(poll_rate);
    while (ros::ok())
    {
      ros::spinOnce();
      PublishWrench();
      rate.sleep();
    }
  }

private:

  void PublishWrench()
  {
    const Eigen::Matrix<double, 6, 1> wrench
        = sensor_ptr_->GetCurrentForceTorque();
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = sensor_frame_;
    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.x = wrench(0, 0);
    wrench_msg.wrench.force.y = wrench(1, 0);
    wrench_msg.wrench.force.z = wrench(2, 0);
    wrench_msg.wrench.torque.x = wrench(3, 0);
    wrench_msg.wrench.torque.y = wrench(4, 0);
    wrench_msg.wrench.torque.z = wrench(5, 0);
    status_pub_.publish(wrench_msg);
  }
};
}

int main(int argc, char** argv)
{
  // Default ROS params
  const double DEFAULT_POLL_RATE = 100.0;
  const std::string DEFAULT_STATUS_TOPIC("robotiq_ft_sensor");
  const std::string DEFAULT_SENSOR_FRAME("robotiq_ft_sensor");
  const std::string DEFAULT_MODBUS_RTU_INTERFACE("/dev/ttyUSB0");
  const int32_t DEFAULT_SENSOR_SLAVE_ID = 0x09;
  // Start ROS
  ros::init(argc, argv, "robotiq_ft_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Get params
  const std::string modbus_rtu_interface
      = nhp.param(std::string("modbus_rtu_interface"),
                  DEFAULT_MODBUS_RTU_INTERFACE);
  const uint16_t sensor_slave_id
      = (uint16_t)nhp.param(std::string("sensor_slave_id"),
                            DEFAULT_SENSOR_SLAVE_ID);
  const double poll_rate
      = std::abs(nhp.param(std::string("poll_rate"), DEFAULT_POLL_RATE));
  const std::string status_topic
      = nhp.param(std::string("status_topic"), DEFAULT_STATUS_TOPIC);
  const std::string sensor_frame
      = nhp.param(std::string("sensor_frame"), DEFAULT_SENSOR_FRAME);
  // Start the driver
  robotiq_ft_driver::RobotiqFTDriver sensor(nh,
                                            status_topic,
                                            sensor_frame,
                                            modbus_rtu_interface,
                                            sensor_slave_id);
  sensor.Loop(poll_rate);
  return 0;
}
