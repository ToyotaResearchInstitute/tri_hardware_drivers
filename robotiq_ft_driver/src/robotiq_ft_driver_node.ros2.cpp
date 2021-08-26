#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <robotiq_ft_driver/robotiq_ft_driver.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace robotiq_ft_driver
{
class RobotiqFTDriverNode : public rclcpp::Node
{
private:
  std::string sensor_frame_;
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;
  std::shared_ptr<rclcpp::Publisher<WrenchStamped>> status_pub_;
  std::unique_ptr<RobotiqFTModbusRtuInterface> sensor_ptr_;
  std::shared_ptr<rclcpp::TimerBase> poll_timer_;

public:
  explicit RobotiqFTDriverNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robotiq_ft_driver", options)
  {
    // Default ROS params
    const double DEFAULT_POLL_RATE = 100.0;
    const std::string DEFAULT_STATUS_TOPIC("robotiq_ft_sensor");
    const std::string DEFAULT_SENSOR_FRAME("robotiq_ft_sensor");
    const std::string DEFAULT_MODBUS_RTU_INTERFACE("/dev/ttyUSB0");
    const int32_t DEFAULT_SENSOR_SLAVE_ID = 0x09;
    sensor_frame_
        = this->declare_parameter("sensor_frame", DEFAULT_SENSOR_FRAME);
    // Make the logging function
    std::function<void(const std::string&)> logging_fn
        = [this] (const std::string& message)
    {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    };
    const std::string modbus_rtu_interface
        = this->declare_parameter("modbus_rtu_interface",
                                  DEFAULT_MODBUS_RTU_INTERFACE);
    const uint16_t sensor_slave_id
        = static_cast<uint16_t>(this->declare_parameter(
            "sensor_slave_id", DEFAULT_SENSOR_SLAVE_ID));
    RCLCPP_INFO(
        this->get_logger(),
        "Connecting to Robotiq F/T sensor with sensor slave ID %hx on "
        "Modbus RTU interface %s...",
        sensor_slave_id, modbus_rtu_interface.c_str());

    sensor_ptr_ = std::unique_ptr<RobotiqFTModbusRtuInterface>(
                    new RobotiqFTModbusRtuInterface(logging_fn,
                                                    modbus_rtu_interface,
                                                    sensor_slave_id));
    const std::string serial_num = sensor_ptr_->ReadSerialNumber();
    const std::string firmware_version = sensor_ptr_->ReadFirmwareVersion();
    const uint16_t year = sensor_ptr_->ReadYearOfManufacture();

    RCLCPP_INFO(
        this->get_logger(),
        "Connected to sensor with serial # %s firmware version %s year of "
        "manufacture %hu", serial_num.c_str(), firmware_version.c_str(), year);
    const std::string status_topic
        = this->declare_parameter("status_topic", DEFAULT_STATUS_TOPIC);
    status_pub_ = this->create_publisher<WrenchStamped>(status_topic, 1);

    const double poll_rate
        = std::abs(this->declare_parameter("poll_rate", DEFAULT_POLL_RATE));
    RCLCPP_INFO(
        this->get_logger(),
        "Starting to stream F/T measurements at %f Hz...", poll_rate);
    poll_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(1. / poll_rate),
        std::bind(&RobotiqFTDriverNode::PublishWrench, this));
  }

private:
  void PublishWrench()
  {
    const Eigen::Matrix<double, 6, 1> wrench
        = sensor_ptr_->GetCurrentForceTorque();
    WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = sensor_frame_;
    wrench_msg.header.stamp = this->get_clock()->now();
    wrench_msg.wrench.force.x = wrench(0, 0);
    wrench_msg.wrench.force.y = wrench(1, 0);
    wrench_msg.wrench.force.z = wrench(2, 0);
    wrench_msg.wrench.torque.x = wrench(3, 0);
    wrench_msg.wrench.torque.y = wrench(4, 0);
    wrench_msg.wrench.torque.z = wrench(5, 0);
    status_pub_->publish(wrench_msg);
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robotiq_ft_driver::RobotiqFTDriverNode)
