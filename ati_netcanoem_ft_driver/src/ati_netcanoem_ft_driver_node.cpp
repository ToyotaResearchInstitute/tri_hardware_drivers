#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <ati_netcanoem_ft_driver/ati_netcanoem_ft_driver.hpp>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/xmlrpc_manager.h>
#include <std_srvs/SetBool.h>
#include <signal.h>

namespace ati_netcanoem_ft_driver
{
class AtiNetCanOemDriver
{
private:

  ros::NodeHandle nh_;
  std::string sensor_frame_;
  ros::Publisher status_pub_;
  ros::ServiceServer reset_or_set_bias_service_;
  std::unique_ptr<AtiNetCanOemInterface> sensor_ptr_;

public:

  AtiNetCanOemDriver(const ros::NodeHandle& nh,
                     const std::string& status_topic,
                     const std::string& reset_or_set_bias_service,
                     const std::string& sensor_frame,
                     const std::string& can_interface,
                     const uint8_t sensor_base_can_id,
                     const uint8_t sensor_calibration_index)
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
                   "Connecting to ATI F/T sensor with CAN base ID %hhx"
                   " on socketcan interface %s...",
                   sensor_base_can_id,
                   can_interface.c_str());
    sensor_ptr_
        = std::unique_ptr<AtiNetCanOemInterface>(
            new AtiNetCanOemInterface(logging_fn,
                                      can_interface,
                                      sensor_base_can_id));
    const std::string serial_num = sensor_ptr_->ReadSerialNumber();
    const auto firmware_version = sensor_ptr_->ReadFirmwareVersion();
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Connected to sensor with serial # %s and firmware"
                   " version %hhu (major version) %hhu (minor version)"
                   " %hu (build)",
                   serial_num.c_str(),
                   firmware_version.MajorVersion(),
                   firmware_version.MinorVersion(),
                   firmware_version.BuildNumber());
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Attempting to load active calibration %hhu...",
                   sensor_calibration_index);
    const bool set_calibration
        = sensor_ptr_->LoadNewActiveCalibration(sensor_calibration_index);
    if (set_calibration)
    {
      ROS_INFO_NAMED(ros::this_node::getName(),
               "Loaded calibration %hhu",
               sensor_calibration_index);
      status_pub_
          = nh_.advertise<geometry_msgs::WrenchStamped>(status_topic, 1, false);
      reset_or_set_bias_service_
          = nh_.advertiseService(
              reset_or_set_bias_service,
              &AtiNetCanOemDriver::ResetOrSetBiasCallback, this);
    }
    else
    {
      ROS_FATAL_NAMED(ros::this_node::getName(),
              "Failed to load calibration %x",
              sensor_calibration_index);
    }
  }

  bool ResetOrSetBiasCallback(
      std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
  {
    if (req.data) {
      sensor_ptr_->ResetBias();
      res.message = "Reset bias";
    } else {
      sensor_ptr_->SetBias();
      res.message = "Set bias";
    }
    res.success = true;
    return true;
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
  const std::string DEFAULT_STATUS_TOPIC("ati_ft_sensor");
  const std::string DEFAULT_RESET_OR_SET_BIAS_SERVICE(
      "ati_ft_sensor_reset_or_set_bias");
  const std::string DEFAULT_SENSOR_FRAME("ati_ft_sensor");
  const std::string DEFAULT_SOCKETCAN_INTERFACE("can0");
  const int32_t DEFAULT_SENSOR_BASE_CAN_ID = 0x00;
  const int32_t DEFAULT_SENSOR_CALIBRATION = 0x00;
  // Start ROS
  ros::init(argc, argv, "ati_netcanoem_ft_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Get params
  const std::string can_interface
      = nhp.param(std::string("socketcan_interface"),
                  DEFAULT_SOCKETCAN_INTERFACE);
  const uint8_t sensor_base_can_id
      = static_cast<uint8_t>(nhp.param(std::string("sensor_base_can_id"),
                                       DEFAULT_SENSOR_BASE_CAN_ID));
  const double poll_rate
      = std::abs(nhp.param(std::string("poll_rate"), DEFAULT_POLL_RATE));
  const std::string status_topic
      = nhp.param(std::string("status_topic"), DEFAULT_STATUS_TOPIC);
  const std::string reset_or_set_bias_service
      = nhp.param(std::string("reset_or_set_bias_service"),
                  DEFAULT_RESET_OR_SET_BIAS_SERVICE);
  const std::string sensor_frame
      = nhp.param(std::string("sensor_frame"), DEFAULT_SENSOR_FRAME);
  const uint8_t sensor_calibration_index
      = static_cast<uint8_t>(nhp.param(std::string("sensor_calibration_index"),
                                       DEFAULT_SENSOR_CALIBRATION));
  // Start the driver
  ati_netcanoem_ft_driver::AtiNetCanOemDriver sensor(
      nh, status_topic, reset_or_set_bias_service, sensor_frame, can_interface,
      sensor_base_can_id, sensor_calibration_index);
  sensor.Loop(poll_rate);
  return 0;
}
