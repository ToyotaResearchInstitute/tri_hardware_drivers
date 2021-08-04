#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string>
#include <ati_netcanoem_ft_driver/ati_netcanoem_ft_driver.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace ati_netcanoem_ft_driver
{
class AtiNetCanOemDriver
{
private:

  std::shared_ptr<rclcpp::Node> node_;
  std::string sensor_frame_;
  std::shared_ptr<
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> status_pub_;
  std::shared_ptr<
    rclcpp::Service<std_srvs::srv::SetBool>> reset_or_set_bias_service_;
  std::unique_ptr<AtiNetCanOemInterface> sensor_ptr_;

public:

  AtiNetCanOemDriver(std::shared_ptr<rclcpp::Node> node,
                     const std::string& status_topic,
                     const std::string& reset_or_set_bias_service,
                     const std::string& sensor_frame,
                     const std::string& can_interface,
                     const uint8_t sensor_base_can_id,
                     const uint8_t sensor_calibration_index)
    : node_(std::move(node)), sensor_frame_(sensor_frame)
  {
    // Make the logging function
    std::function<void(const std::string&)> logging_fn
        = [this] (const std::string& message)
    {
      RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    };
    RCLCPP_INFO(
        node_->get_logger(),
        "Connecting to ATI F/T sensor with CAN base ID %hhx on socketcan "
        "interface %s...", sensor_base_can_id, can_interface.c_str());
    sensor_ptr_
        = std::unique_ptr<AtiNetCanOemInterface>(
            new AtiNetCanOemInterface(logging_fn,
                                      can_interface,
                                      sensor_base_can_id));
    const std::string serial_num = sensor_ptr_->ReadSerialNumber();
    const auto firmware_version = sensor_ptr_->ReadFirmwareVersion();
    RCLCPP_INFO(
        node_->get_logger(),
        "Connected to sensor with serial # %s and firmware version %hhu "
        "(major version) %hhu (minor version) %hu (build)",
        serial_num.c_str(), firmware_version.MajorVersion(),
        firmware_version.MinorVersion(), firmware_version.BuildNumber());
    RCLCPP_INFO(
        node_->get_logger(),
        "Attempting to load active calibration %hhu...",
        sensor_calibration_index);
    const bool set_calibration
        = sensor_ptr_->LoadNewActiveCalibration(sensor_calibration_index);
    if (set_calibration)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "Loaded calibration %hhu", sensor_calibration_index);
      status_pub_
          = node_->create_publisher<
              geometry_msgs::msg::WrenchStamped>(status_topic, 1);
      using std::placeholders::_1;
      using std::placeholders::_2;
      auto service_callback = std::bind(
          &AtiNetCanOemDriver::ResetOrSetBiasCallback, this, _1, _2);
      reset_or_set_bias_service_
          = node_->create_service<std_srvs::srv::SetBool>(
              reset_or_set_bias_service, service_callback);
    }
    else
    {
      RCLCPP_FATAL(
          node_->get_logger(),
          "Failed to load calibration %x", sensor_calibration_index);

    }
  }

  void ResetOrSetBiasCallback(
      std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data) {
      sensor_ptr_->ResetBias();
      res->message = "Reset bias";
    } else {
      sensor_ptr_->SetBias();
      res->message = "Set bias";
    }
    res->success = true;
  }

  void Loop(const double poll_rate)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "Starting to stream F/T measurements at %f Hz...", poll_rate);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    rclcpp::Rate rate(poll_rate);
    while (rclcpp::ok())
    {
      executor.spin_once(rate.period());
      PublishWrench();
      rate.sleep();
    }
  }

private:

  void PublishWrench()
  {
    const Eigen::Matrix<double, 6, 1> wrench
      = sensor_ptr_->GetCurrentForceTorque();
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = sensor_frame_;
    wrench_msg.header.stamp = node_->get_clock()->now();
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

int main(int argc, char** argv)
{
  // Start ROS
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<rclcpp::Node>("ati_netcanoem_ft_driver");

    // Default ROS params
    const double DEFAULT_POLL_RATE = 100.0;
    const std::string DEFAULT_STATUS_TOPIC("ati_ft_sensor");
    const std::string DEFAULT_RESET_OR_SET_BIAS_SERVICE(
        "ati_ft_sensor_reset_or_set_bias");
    const std::string DEFAULT_SENSOR_FRAME("ati_ft_sensor");
    const std::string DEFAULT_SOCKETCAN_INTERFACE("can0");
    const int32_t DEFAULT_SENSOR_BASE_CAN_ID = 0x00;
    const int32_t DEFAULT_SENSOR_CALIBRATION = 0x00;

    // Get params
    const std::string can_interface
        = node->declare_parameter("socketcan_interface",
                                  DEFAULT_SOCKETCAN_INTERFACE);

    const uint8_t sensor_base_can_id
        = static_cast<uint8_t>(node->declare_parameter(
            "sensor_base_can_id", DEFAULT_SENSOR_BASE_CAN_ID));

    const double poll_rate
        = node->declare_parameter("poll_rate", DEFAULT_POLL_RATE);

    const std::string status_topic
        = node->declare_parameter("status_topic", DEFAULT_STATUS_TOPIC);

    const std::string reset_or_set_bias_service
        = node->declare_parameter("reset_or_set_bias_service",
                                  DEFAULT_RESET_OR_SET_BIAS_SERVICE);

    const std::string sensor_frame
        = node->declare_parameter("sensor_frame", DEFAULT_SENSOR_FRAME);

    const uint8_t sensor_calibration_index
        = static_cast<uint8_t>(node->declare_parameter(
            "sensor_calibration_index", DEFAULT_SENSOR_CALIBRATION));

    // Start the driver
    ati_netcanoem_ft_driver::AtiNetCanOemDriver sensor(
        node, status_topic, reset_or_set_bias_service, sensor_frame,
        can_interface, sensor_base_can_id, sensor_calibration_index);
    sensor.Loop(poll_rate);
  }
  rclcpp::shutdown();
  return 0;
}
