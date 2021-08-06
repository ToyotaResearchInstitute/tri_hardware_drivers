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
#include <lightweight_ur_interface/control_program.hpp>
#include <lightweight_ur_interface/ur_robot_config.hpp>
#include <lightweight_ur_interface/ur_minimal_realtime_driver.hpp>
#include <lightweight_ur_interface/ur_script_hardware_interface.hpp>
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

class URScriptHardwareInterfaceNode : public rclcpp::Node
{
private:

  static constexpr double FLOAT_CONVERSION_RATIO = 1000000.0;
  static constexpr double STOP_DECELERATION = 1.0;
  static constexpr double SPEED_ACCELERATION = 3.2;
  static constexpr double SPEED_COMMAND_WAIT = 0.008;

  std::string base_frame_;
  std::string ee_frame_;
  std::vector<std::string> joint_names_;
  std::map<std::string, JointLimits> joint_limits_;
  double max_acceleration_limit_;

  std::string our_ip_address_;
  int32_t control_port_;
  std::vector<ControlScriptCommand> control_script_command_queue_;

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
  std::shared_ptr<
    rclcpp::Service<std_srvs::srv::SetBool>> switch_teach_mode_server_;
  std::shared_ptr<rclcpp::TimerBase> control_timer_;

  std::atomic<bool> in_teach_mode_{false};
  std::unique_ptr<URRealtimeInterface> robot_ptr_;
  std::pair<int32_t, int32_t> initial_control_program_socket_fds_;
  std::mutex latest_state_mutex_;
  bool valid_latest_state_ = false;
  sensor_msgs::msg::JointState latest_joint_state_;
  Eigen::Isometry3d latest_tcp_pose_;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit URScriptHardwareInterfaceNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("ur_script_hardware_interface", options)
  {
    RCLCPP_INFO(
        this->get_logger(), "Starting ur_script_hardware_interface...");
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
    const std::string DEFAULT_TEACH_MODE_SERVICE = "/ur10/switch_teach_mode";
    const std::string DEFAULT_ROBOT_HOSTNAME = "172.31.1.200";
    const std::string DEFAULT_OUR_IP_ADDRESS = "172.31.1.100";
    constexpr int32_t DEFAULT_CONTROL_PORT = 50007;
    constexpr double DEFAULT_VELOCITY_LIMIT_SCALING = 0.5;
    constexpr double DEFAULT_ACCELERATION_LIMIT_SCALING = 0.5;

    base_frame_
        = this->declare_parameter("base_frame", DEFAULT_BASE_FRAME);
    ee_frame_
        = this->declare_parameter("ee_wrench_frame", DEFAULT_EE_FRAME);;

    our_ip_address_
        = this->declare_parameter("our_ip_address", DEFAULT_OUR_IP_ADDRESS);
    control_port_
        = static_cast<int32_t>(std::abs(
            this->declare_parameter("control_port", DEFAULT_CONTROL_PORT)));
    const double velocity_limit_scaling
        = std::abs(this->declare_parameter("velocity_limit_scaling",
                                           DEFAULT_VELOCITY_LIMIT_SCALING));
    const double acceleration_limit_scaling
        = std::abs(this->declare_parameter("acceleration_limit_scaling",
                                           DEFAULT_ACCELERATION_LIMIT_SCALING));
    const double real_velocity_limit_scaling
        = common_robotics_utilities::utility::ClampValueAndWarn(
            velocity_limit_scaling, 0.0, 1.0);
    const double real_acceleration_limit_scaling
        = common_robotics_utilities::utility::ClampValueAndWarn(
            acceleration_limit_scaling, 0.0, 1.0);

    // Joint names in true order
    joint_names_ = lightweight_ur_interface::GetOrderedJointNames();
    if (joint_names_.size() != 6)
    {
      throw std::runtime_error("There must be exactly 6 joints");
    }
    // Joint limits
    joint_limits_
        = lightweight_ur_interface::GetLimits(real_velocity_limit_scaling,
                                              real_acceleration_limit_scaling);
    // Make sure our ordered joint names match our joint limits
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

    const std::string ee_pose_topic
        = this->declare_parameter("ee_pose_topic", DEFAULT_EE_POSE_TOPIC);
    const std::string ee_world_twist_topic
        = this->declare_parameter("ee_world_twist_topic",
                                  DEFAULT_EE_WORLD_TWIST_TOPIC);
    const std::string ee_body_twist_topic
        = this->declare_parameter("ee_body_twist_topic",
                                  DEFAULT_EE_BODY_TWIST_TOPIC);
    const std::string ee_wrench_topic
        = this->declare_parameter("ee_wrench_topic", DEFAULT_EE_WRENCH_TOPIC);

    const std::string joint_state_topic
        = this->declare_parameter("joint_state_topic",
                                  DEFAULT_JOINT_STATE_TOPIC);
    joint_state_pub_ = this->create_publisher<
      sensor_msgs::msg::JointState>(joint_state_topic, 1);
    ee_pose_pub_ = this->create_publisher<
      geometry_msgs::msg::PoseStamped>(ee_pose_topic, 1);
    ee_world_twist_pub_ = this->create_publisher<
      geometry_msgs::msg::TwistStamped>(ee_world_twist_topic, 1);
    ee_body_twist_pub_ = this->create_publisher<
      geometry_msgs::msg::TwistStamped>(ee_body_twist_topic, 1);
    ee_wrench_pub_ = this->create_publisher<
      geometry_msgs::msg::WrenchStamped>(ee_wrench_topic, 1);

    using std::placeholders::_1;
    using std::placeholders::_2;

    const std::string velocity_command_topic
        = this->declare_parameter("velocity_command_topic",
                                  DEFAULT_VELOCITY_COMMAND_TOPIC);
    auto velocity_command_callback = std::bind(
        &URScriptHardwareInterfaceNode::VelocityCommandCallback, this, _1);
    velocity_command_sub_
        = this->create_subscription<
            lightweight_ur_interface::msg::VelocityCommand>(
              velocity_command_topic, 1, velocity_command_callback);

    const std::string twist_command_topic
        = this->declare_parameter("twist_command_topic",
                                  DEFAULT_TWIST_COMMAND_TOPIC);
    auto twist_command_callback = std::bind(
        &URScriptHardwareInterfaceNode::TwistCommandCallback, this, _1);
    twist_command_sub_
        = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            twist_command_topic, 1, twist_command_callback);

    const std::string teach_mode_service
        = this->declare_parameter("teach_mode_service",
                                  DEFAULT_TEACH_MODE_SERVICE);
    auto switch_teach_mode_callback = std::bind(
        &URScriptHardwareInterfaceNode::SwitchTeachModeCB, this, _1, _2);
    switch_teach_mode_server_
        = this->create_service<std_srvs::srv::SetBool>(
            teach_mode_service, switch_teach_mode_callback);

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
    // Start robot interface
    const std::string robot_hostname
        = this->declare_parameter("robot_hostname", DEFAULT_ROBOT_HOSTNAME);
    robot_ptr_ = std::make_unique<URRealtimeInterface>(
        robot_hostname, callback_fn, logging_fn);

    robot_ptr_->StartRecv();
    RCLCPP_INFO(this->get_logger(), "Started robot realtime interface");
    // Start control program interface
    initial_control_program_socket_fds_
        = StartControlProgram(our_ip_address_,
                              control_port_,
                              FLOAT_CONVERSION_RATIO,
                              STOP_DECELERATION,
                              SPEED_ACCELERATION,
                              SPEED_COMMAND_WAIT);
    const int32_t control_program_incoming_sock_fd
        = initial_control_program_socket_fds_.first;
    const int32_t control_program_sock_fd
        = initial_control_program_socket_fds_.second;
    RCLCPP_INFO(
        this->get_logger(),
        "Started robot control interface with fds %d, %d",
        control_program_incoming_sock_fd, control_program_sock_fd);

    constexpr double control_interval = 1. / 400.;
    control_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(control_interval),
        std::bind(&URScriptHardwareInterfaceNode::SpinOnce, this));
    RCLCPP_INFO(this->get_logger(), "...startup complete");
  }

  ~URScriptHardwareInterfaceNode()
  {
    const int32_t control_program_incoming_sock_fd
        = initial_control_program_socket_fds_.first;
    const int32_t control_program_sock_fd
        = initial_control_program_socket_fds_.second;
    const ControlScriptCommand exit_command
        = ControlScriptCommand::MakeExitProgramCommand();
    std::vector<uint8_t> buffer;
    exit_command.SerializeSelf(FLOAT_CONVERSION_RATIO, buffer);
    const ssize_t bytes_written = write(control_program_sock_fd,
                                        buffer.data(),
                                        buffer.size());
    if (bytes_written != static_cast<ssize_t>(buffer.size()))
    {
      perror(nullptr);
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to send command with %zu bytes, sent %zd instead",
          buffer.size(), bytes_written);
    }
    close(control_program_sock_fd);
    close(control_program_incoming_sock_fd);
    robot_ptr_->StopRecv();
  }

  void SpinOnce()
  {
    int32_t control_program_incoming_sock_fd
        = initial_control_program_socket_fds_.first;
    int32_t control_program_sock_fd
        = initial_control_program_socket_fds_.second;

    for (size_t idx = 0; idx < control_script_command_queue_.size(); idx++)
    {
      const ControlScriptCommand& current_command
          = control_script_command_queue_[idx];
      std::vector<uint8_t> buffer;
      current_command.SerializeSelf(FLOAT_CONVERSION_RATIO, buffer);
      const ssize_t bytes_written = write(control_program_sock_fd,
                                          buffer.data(),
                                          buffer.size());
      if (bytes_written != static_cast<ssize_t>(buffer.size()))
      {
        perror(nullptr);
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to send command with %zu bytes, sent %zd instead",
            buffer.size(), bytes_written);
        RCLCPP_INFO(
            this->get_logger(),
            "Trying to restart/reconnect to the robot control script...");
        close(control_program_sock_fd);
        close(control_program_incoming_sock_fd);
        exit(1);
        const std::pair<int32_t, int32_t> new_control_program_socket_fds
            = StartControlProgram(our_ip_address_,
                                  control_port_,
                                  FLOAT_CONVERSION_RATIO,
                                  STOP_DECELERATION,
                                  SPEED_ACCELERATION,
                                  SPEED_COMMAND_WAIT);
        control_program_incoming_sock_fd
            = new_control_program_socket_fds.first;
        control_program_sock_fd = new_control_program_socket_fds.second;
        RCLCPP_INFO(
            this->get_logger(),
            "Restarted/reconnected robot control interface with fds %d, %d",
             control_program_incoming_sock_fd, control_program_sock_fd);
      }
    }
    control_script_command_queue_.clear();
  }

  std::pair<int32_t, int32_t> StartControlProgram(
      const std::string& our_ip_address,
      const int32_t control_port,
      const double float_conversion_ratio,
      const double stop_deceleration,
      const double speed_acceleration,
      const double speed_command_wait)
  {
    const std::string control_script_string
        = lightweight_ur_driver::MakeControlProgram(our_ip_address,
                                                    control_port,
                                                    float_conversion_ratio,
                                                    stop_deceleration,
                                                    speed_acceleration,
                                                    speed_command_wait);
    const bool success = robot_ptr_->SendURScriptCommand(control_script_string);
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Uploaded control program");
    }
    else
    {
      throw std::runtime_error("Failed to upload control program");
    }
    // Setup communications with the control program
    int incoming_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (incoming_sock_fd < 0)
    {
      throw std::runtime_error(
            "ERROR opening socket for control program communication");
    }
    else
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Opened socket for control program communication");
    }
    struct sockaddr_in serv_addr;
    std::memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(static_cast<uint16_t>(control_port));
    int flag = 1;
    setsockopt(
        incoming_sock_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
    setsockopt(
        incoming_sock_fd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));
    if (bind(incoming_sock_fd, reinterpret_cast<struct sockaddr*>(&serv_addr),
             sizeof(serv_addr))
        < 0)
    {
      throw std::runtime_error(
            "ERROR on binding socket for control program communication");
    }
    else
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Bound socket for control program communication");
    }
    listen(incoming_sock_fd, 5);
    struct sockaddr_in cli_addr;
    socklen_t clilen;
    clilen = sizeof(cli_addr);
    int new_sock_fd = accept(
        incoming_sock_fd, reinterpret_cast<struct sockaddr*>(&cli_addr),
        &clilen);
    if (new_sock_fd < 0)
    {
      throw std::runtime_error(
            "ERROR on accepting control program communication");
    }
    else
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Accepted connection for control program communication");
    }
    return std::make_pair(incoming_sock_fd, new_sock_fd);
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

  void SwitchTeachModeCB(
      std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data)
    {
      if (in_teach_mode_.load() == false)
      {
        control_script_command_queue_.push_back(
              ControlScriptCommand::MakeTeachModeCommand());
        in_teach_mode_.store(true);
        res->success = true;
        res->message = "Entered teach mode";
      }
      else
      {
        res->success = true;
        res->message = "Already in teach mode, ignoring request to enter";
      }
    }
    else
    {
      if (in_teach_mode_.load() == true)
      {
        control_script_command_queue_.push_back(
              ControlScriptCommand::MakeExitTeachModeCommand());
        in_teach_mode_.store(false);
        res->success = true;
        res->message = "Exited teach mode";
      }
      else
      {
        res->success = true;
        res->message = "Not in teach mode, ignoring request to exit";
      }
    }
  }

  void VelocityCommandCallback(
      const lightweight_ur_interface::msg::VelocityCommand& config_target)
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
                = ClampValueAndWarn(velocity, -velocity_limit, velocity_limit);
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
        if (in_teach_mode_.load() == false)
        {
          control_script_command_queue_.push_back(
                ControlScriptCommand::MakeSpeedJCommand(target_velocity));
        }
        else
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Ignoring VelocityCommand since robot is in teach mode");
        }
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

  void TwistCommandCallback(const geometry_msgs::msg::TwistStamped& twist_command)
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
      if (in_teach_mode_.load() == false)
      {
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
            control_script_command_queue_.push_back(
                  ControlScriptCommand::MakeSpeedLCommand(base_frame_twist));
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
          control_script_command_queue_.push_back(
                ControlScriptCommand::MakeSpeedLCommand(raw_twist));
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
      else
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignoring Twist since robot is in teach mode");
      }
    }
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lightweight_ur_interface::URScriptHardwareInterfaceNode)
