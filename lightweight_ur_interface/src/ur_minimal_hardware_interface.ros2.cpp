#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <lightweight_ur_interface/ur_robot_config.hpp>
#include <lightweight_ur_interface/ur_minimal_realtime_driver.hpp>
#include <lightweight_ur_interface/msg/velocity_command.hpp>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/ros_conversions.hpp>

namespace lightweight_ur_interface
{
using common_robotics_utilities::ros_conversions
          ::EigenIsometry3dToGeometryPoseStamped;
using common_robotics_utilities::ros_conversions
          ::EigenVector3dToGeometryVector3;
using common_robotics_utilities::math::RotateVectorReverse;
using common_robotics_utilities::math::RotateVector;
using common_robotics_utilities::utility::ClampValueAndWarn;
using common_robotics_utilities::utility::CollectionsEqual;
using common_robotics_utilities::utility::GetKeysFromMapLike;

class URMinimalHardwareInterfaceNode : public rclcpp::Node
{
private:

  std::string base_frame_;
  std::string ee_frame_;
  std::vector<std::string> joint_names_;
  std::map<std::string, JointLimits> joint_limits_;
  double max_acceleration_limit_;

  std::shared_ptr<
    rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_pub_;
  std::shared_ptr<
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> ee_pose_pub_;
  std::shared_ptr<
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> ee_body_twist_pub_;
  std::shared_ptr<
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> ee_world_twist_pub_;
  std::shared_ptr<
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> ee_wrench_pub_;
  std::shared_ptr<
    rclcpp::Subscription<
      lightweight_ur_interface::msg::VelocityCommand>> velocity_command_sub_;
  std::shared_ptr<
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> twist_command_sub_;

  std::unique_ptr<URRealtimeInterface> robot_ptr_;
  std::mutex latest_state_mutex_;
  bool valid_latest_state_ = false;
  sensor_msgs::msg::JointState latest_joint_state_;
  Eigen::Isometry3d latest_tcp_pose_;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit URMinimalHardwareInterfaceNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("ur_minimal_hardware_interface", options)
  {
    RCLCPP_INFO(
        this->get_logger(), "Starting ur_minimal_hardware_interface...");
    const std::string DEFAULT_JOINT_STATE_TOPIC = "/ur10/joint_states";
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC
        = "/ur10/joint_command_velocity";
    const std::string DEFAULT_TWIST_COMMAND_TOPIC = "/ur10/ee_twist_command";
    const std::string DEFAULT_EE_POSE_TOPIC = "/ur10/ee_pose";
    const std::string DEFAULT_EE_WORLD_TWIST_TOPIC = "/ur10/ee_world_twist";
    const std::string DEFAULT_EE_BODY_TWIST_TOPIC = "/ur10/ee_body_twist";
    const std::string DEFAULT_EE_WRENCH_TOPIC = "/ur10/ee_wrench";
    const std::string DEFAULT_BASE_FRAME = "base";
    const std::string DEFAULT_EE_FRAME = "ur10_ee_frame";
    const std::string DEFAULT_ROBOT_HOSTNAME = "172.31.1.200";
    const double DEFAULT_VELOCITY_LIMIT_SCALING = 0.5;
    const double DEFAULT_ACCELERATION_LIMIT_SCALING = 0.5;

    base_frame_ = this->declare_parameter("base_frame", DEFAULT_BASE_FRAME);
    ee_frame_ = this->declare_parameter("ee_frame", DEFAULT_EE_FRAME);

    // Make sure our ordered joint names match our joint limits
    joint_names_ = lightweight_ur_interface::GetOrderedJointNames();
    if (joint_names_.size() != 6)
    {
      throw std::runtime_error("There must be exactly 6 joints");
    }

    const double velocity_limit_scaling = std::abs(this->declare_parameter(
        "velocity_limit_scaling", DEFAULT_VELOCITY_LIMIT_SCALING));
    const double acceleration_limit_scaling = std::abs(this->declare_parameter(
        "acceleration_limit_scaling", DEFAULT_ACCELERATION_LIMIT_SCALING));
    const double real_velocity_limit_scaling =
        common_robotics_utilities::utility::ClampValueAndWarn(
            velocity_limit_scaling, 0.0, 1.0);
    const double real_acceleration_limit_scaling =
        common_robotics_utilities::utility::ClampValueAndWarn(
            acceleration_limit_scaling, 0.0, 1.0);
    joint_limits_ = lightweight_ur_interface::GetLimits(
        real_velocity_limit_scaling, real_acceleration_limit_scaling);
    if (CollectionsEqual<std::string>(
            joint_names_,
            GetKeysFromMapLike<std::string, JointLimits>(joint_limits_))
        == false)
    {
      throw std::invalid_argument(
            "Ordered joint names do not match names provided with limits");
    }

    // Compute the maximum joint acceleration
    max_acceleration_limit_ = 0.0;
    for (auto itr = joint_limits_.begin(); itr != joint_limits_.end(); ++itr)
    {
      const JointLimits& joint_limit = itr->second;
      const double acceleration_limit = joint_limit.MaxAcceleration();
      max_acceleration_limit_
          = std::max(max_acceleration_limit_, acceleration_limit);
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Set max_acceleration_limit to %f", max_acceleration_limit_);

    const std::string joint_state_topic = this->declare_parameter(
        "joint_state_topic", DEFAULT_JOINT_STATE_TOPIC);
    joint_state_pub_ = this->create_publisher<
      sensor_msgs::msg::JointState>(joint_state_topic, 1);

    const std::string ee_pose_topic = this->declare_parameter(
        "ee_pose_topic", DEFAULT_EE_POSE_TOPIC);
    ee_pose_pub_ = this->create_publisher<
      geometry_msgs::msg::PoseStamped>(ee_pose_topic, 1);

    const std::string ee_world_twist_topic = this->declare_parameter(
        "ee_world_twist_topic", DEFAULT_EE_WORLD_TWIST_TOPIC);
    ee_world_twist_pub_ = this->create_publisher<
      geometry_msgs::msg::TwistStamped>(ee_world_twist_topic, 1);

    const std::string ee_body_twist_topic = this->declare_parameter(
        "ee_body_twist_topic", DEFAULT_EE_BODY_TWIST_TOPIC);
    ee_body_twist_pub_ = this->create_publisher<
      geometry_msgs::msg::TwistStamped>(ee_body_twist_topic, 1);

    const std::string ee_wrench_topic = this->declare_parameter(
        "ee_wrench_topic", DEFAULT_EE_WRENCH_TOPIC);
    ee_wrench_pub_ = this->create_publisher<
        geometry_msgs::msg::WrenchStamped>(ee_wrench_topic, 1);

    using std::placeholders::_1;

    const std::string velocity_command_topic = this->declare_parameter(
        "velocity_command_topic", DEFAULT_VELOCITY_COMMAND_TOPIC);
    auto velocity_command_callback = std::bind(
        &URMinimalHardwareInterfaceNode::VelocityCommandCallback, this, _1);
    velocity_command_sub_ = this->create_subscription<
      lightweight_ur_interface::msg::VelocityCommand>(
          velocity_command_topic, 1, velocity_command_callback);

    const std::string twist_command_topic = this->declare_parameter(
        "twist_command_topic", DEFAULT_TWIST_COMMAND_TOPIC);

    auto twist_command_callback = std::bind(
        &URMinimalHardwareInterfaceNode::TwistCommandCallback, this, _1);
    twist_command_sub_ =
        this->create_subscription<geometry_msgs::msg::TwistStamped>(
            twist_command_topic, 1, twist_command_callback);

    // Build robot interface
    const std::function<void(const URRealtimeState&)> callback_fn
        = [&] (const URRealtimeState& latest_state)
    {
      return PublishState(joint_names_, base_frame_, ee_frame_, latest_state);
    };
    std::function<void(const std::string&)> logging_fn
        = [this] (const std::string& message)
    {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    };
    const std::string robot_hostname
        = this->declare_parameter("robot_hostname", DEFAULT_ROBOT_HOSTNAME);
    robot_ptr_ = std::make_unique<URRealtimeInterface>(
        robot_hostname, callback_fn, logging_fn);
    // Start robot interface
    robot_ptr_->StartRecv();
    RCLCPP_INFO(this->get_logger(), "...startup complete");
  }

  ~URMinimalHardwareInterfaceNode()
  {
    robot_ptr_->StopRecv();
  }

  void PublishState(const std::vector<std::string>& joint_names,
            const std::string& base_frame_name,
            const std::string& ee_frame_name,
            const URRealtimeState& robot_state)
  {
    // Get the current time
    const rclcpp::Time state_time = this->get_clock()->now();
    // Joint State
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = state_time;
    joint_state_msg.name = joint_names;
    joint_state_msg.position = robot_state.ActualPosition();
    joint_state_msg.velocity = robot_state.ActualVelocity();
    joint_state_msg.effort = robot_state.TargetTorque();
    latest_state_mutex_.lock();
    latest_joint_state_ = joint_state_msg;
    latest_tcp_pose_ = robot_state.ActualTcpPose();
    valid_latest_state_ = true;
    latest_state_mutex_.unlock();
    // EE transform
    geometry_msgs::msg::PoseStamped ee_transform_msg
        = EigenIsometry3dToGeometryPoseStamped(robot_state.ActualTcpPose(),
                                               base_frame_);
    ee_transform_msg.header.stamp = state_time;
    // EE twist
    const Eigen::Matrix<double, 6, 1>& ee_world_twist
        = robot_state.ActualTcpTwist();
    geometry_msgs::msg::TwistStamped ee_world_twist_msg;
    ee_world_twist_msg.header.stamp = state_time;
    ee_world_twist_msg.header.frame_id = base_frame_name;
    ee_world_twist_msg.twist.linear
        = EigenVector3dToGeometryVector3(ee_world_twist.block<3, 1>(0, 0));
    ee_world_twist_msg.twist.angular
        = EigenVector3dToGeometryVector3(ee_world_twist.block<3, 1>(3, 0));
    const Eigen::Quaterniond ee_rotation(
          robot_state.ActualTcpPose().rotation());
    const Eigen::Vector3d body_frame_linear_velocity
        = RotateVectorReverse(ee_rotation, ee_world_twist.block<3, 1>(0, 0));
    const Eigen::Vector3d body_frame_angular_velocity
        = RotateVectorReverse(ee_rotation, ee_world_twist.block<3, 1>(3, 0));
    geometry_msgs::msg::TwistStamped ee_body_twist_msg;
    ee_body_twist_msg.header.stamp = state_time;
    ee_body_twist_msg.header.frame_id = ee_frame_name;
    ee_body_twist_msg.twist.linear
        = EigenVector3dToGeometryVector3(body_frame_linear_velocity);
    ee_body_twist_msg.twist.angular
        = EigenVector3dToGeometryVector3(body_frame_angular_velocity);
    // EE wrench
    const Eigen::Matrix<double, 6, 1>& ee_wrench
        = robot_state.ActualTcpWrench();
    geometry_msgs::msg::WrenchStamped ee_wrench_msg;
    ee_wrench_msg.header.stamp = state_time;
    ee_wrench_msg.header.frame_id = ee_frame_name;
    ee_wrench_msg.wrench.force
        = EigenVector3dToGeometryVector3(ee_wrench.block<3, 1>(0, 0));
    ee_wrench_msg.wrench.torque
        = EigenVector3dToGeometryVector3(ee_wrench.block<3, 1>(3, 0));
    // Publish
    joint_state_pub_->publish(joint_state_msg);
    ee_pose_pub_->publish(ee_transform_msg);
    ee_world_twist_pub_->publish(ee_world_twist_msg);
    ee_body_twist_pub_->publish(ee_body_twist_msg);
    ee_wrench_pub_->publish(ee_wrench_msg);
  }

  void SendVelocityCommand(const std::vector<double>& command)
  {
    const double max_command_acceleration = 3.2;
    // We need to determine the appropriate max acceleration here?
    const int max_str_len = 1024;
    char command_str_buffer[max_str_len];
    const int written = snprintf(command_str_buffer, max_str_len - 1,
          "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n",
          command[0], command[1], command[2], command[3], command[4],
          command[5], max_command_acceleration);
    if ((written <= 0) || (written >= max_str_len))
    {
      throw std::runtime_error("snprintf for command failed");
    }
    const std::string command_str(command_str_buffer);
    const bool success = robot_ptr_->SendURScriptCommand(command_str);
    if (!success)
    {
      throw std::runtime_error("Failed to send speedj(...) command");
    }
  }

  void VelocityCommandCallback(
      const lightweight_ur_interface::msg::VelocityCommand config_target)
  {
    if (config_target.name.size() == config_target.velocity.size())
    {
      // Push the command into a map
      std::map<std::string, double> command_map;
      for (size_t idx = 0; idx < config_target.name.size(); idx++)
      {
        const std::string& name = config_target.name[idx];
        const double command = config_target.velocity[idx];
        command_map[name] = command;
      }
      // Extract the joint commands in order
      std::vector<double> target_velocity(joint_names_.size(), 0.0);
      bool command_valid = true;
      for (size_t idx = 0; idx < joint_names_.size(); idx++)
      {
        // Get the name of the joint
        const std::string& joint_name = joint_names_[idx];
        // Get the commanded value
        const auto found_itr = command_map.find(joint_name);
        if (found_itr != command_map.end())
        {
          const double velocity = found_itr->second;
          // Get the limits for the joint
          const auto limits_found_itr = joint_limits_.find(joint_name);
          // If we have limits saved, limit the joint command
          if (limits_found_itr != joint_limits_.end())
          {
            const double velocity_limit
                = limits_found_itr->second.MaxVelocity();
            const double limited_velocity
                = common_robotics_utilities::utility::ClampValueAndWarn(
                    velocity, -velocity_limit, velocity_limit);
            target_velocity[idx] = limited_velocity;
          }
          // If we don't have limits saved, then we don't need to limit
          else
          {
            target_velocity[idx] = velocity;
          }
        }
        else
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Invalid VelocityCommand: joint %s missing",
              joint_name.c_str());
          command_valid = false;
        }
      }
      if (command_valid)
      {
        SendVelocityCommand(target_velocity);
      }
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid VelocityCommand: %zu names, %zu velocities",
          config_target.name.size(), config_target.velocity.size());
    }
  }

  void SendTwistCommand(const std::vector<double>& command)
  {
    const double max_command_acceleration = 3.2;
    // We need to determine the appropriate max acceleration here?
    const int max_str_len = 1024;
    char command_str_buffer[max_str_len];
    const int written = snprintf(command_str_buffer, max_str_len - 1,
          "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n",
          command[0], command[1], command[2], command[3], command[4],
          command[5], max_command_acceleration);
    if ((written <= 0) || (written >= max_str_len))
    {
      throw std::runtime_error("snprintf for command failed");
    }
    const std::string command_str(command_str_buffer);
    const bool success = robot_ptr_->SendURScriptCommand(command_str);
    if (!success)
    {
      throw std::runtime_error("Failed to send speedl(...) command");
    }
  }

  void TwistCommandCallback(
      const geometry_msgs::msg::TwistStamped& twist_command)
  {
    bool valid_twist = true;
    if (std::isinf(twist_command.twist.linear.x)
        || std::isnan(twist_command.twist.linear.x))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid Twist command, linear.x is NAN or INF");
      valid_twist = false;
    }
    if (std::isinf(twist_command.twist.linear.y)
        || std::isnan(twist_command.twist.linear.y))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid Twist command, linear.y is NAN or INF");
      valid_twist = false;
    }
    if (std::isinf(twist_command.twist.linear.z)
        || std::isnan(twist_command.twist.linear.z))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid Twist command, linear.z is NAN or INF");
      valid_twist = false;
    }
    if (std::isinf(twist_command.twist.angular.x)
        || std::isnan(twist_command.twist.angular.x))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid Twist command, angular.x is NAN or INF");
      valid_twist = false;
    }
    if (std::isinf(twist_command.twist.angular.y)
        || std::isnan(twist_command.twist.angular.y))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid Twist command, angular.y is NAN or INF");
      valid_twist = false;
    }
    if (std::isinf(twist_command.twist.angular.z)
        || std::isnan(twist_command.twist.angular.z))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid Twist command, angular.z is NAN or INF");
      valid_twist = false;
    }
    if (valid_twist)
    {
      const std::vector<double> raw_twist = {twist_command.twist.linear.x,
                                             twist_command.twist.linear.y,
                                             twist_command.twist.linear.z,
                                             twist_command.twist.angular.x,
                                             twist_command.twist.angular.y,
                                             twist_command.twist.angular.z};
      if (twist_command.header.frame_id == ee_frame_)
      {
        latest_state_mutex_.lock();
        const Eigen::Isometry3d latest_tcp_pose = latest_tcp_pose_;
        const bool valid_latest_state = valid_latest_state_;
        latest_state_mutex_.unlock();
        if (valid_latest_state)
        {
          const Eigen::Quaterniond latest_tcp_rotation(
                latest_tcp_pose.rotation());
          const Eigen::Vector3d ee_frame_linear_velocity(raw_twist[0],
                                                         raw_twist[1],
                                                         raw_twist[2]);
          const Eigen::Vector3d ee_frame_angular_velocity(raw_twist[3],
                                                          raw_twist[4],
                                                          raw_twist[5]);
          const Eigen::Vector3d base_frame_linear_velocity
              = RotateVector(latest_tcp_rotation, ee_frame_linear_velocity);
          const Eigen::Vector3d base_frame_angular_velocity
              = RotateVector(latest_tcp_rotation, ee_frame_angular_velocity);
          const std::vector<double> base_frame_twist
              = {base_frame_linear_velocity.x(),
                 base_frame_linear_velocity.y(),
                 base_frame_linear_velocity.z(),
                 base_frame_angular_velocity.x(),
                 base_frame_angular_velocity.y(),
                 base_frame_angular_velocity.z()};
          SendTwistCommand(base_frame_twist);
        }
        else
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Ignoring ee-frame Twist as latest state invalid");
        }
      }
      else if (twist_command.header.frame_id == base_frame_)
      {
        SendTwistCommand(raw_twist);
      }
      else
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid Twist frame: got [%s] needs [%s] or [%s]",
            twist_command.header.frame_id.c_str(), base_frame_.c_str(),
            ee_frame_.c_str());
      }
    }
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lightweight_ur_interface::URMinimalHardwareInterfaceNode)
