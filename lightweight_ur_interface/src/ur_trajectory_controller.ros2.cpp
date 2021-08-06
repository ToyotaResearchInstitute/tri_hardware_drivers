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
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <lightweight_ur_interface/ur_robot_config.hpp>
#include <lightweight_ur_interface/msg/velocity_command.hpp>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/conversions.hpp>
#include <common_robotics_utilities/time_optimal_trajectory_parametrization.hpp>

namespace lightweight_ur_interface
{
using common_robotics_utilities::print::Print;
using common_robotics_utilities::utility::ClampValue;
using common_robotics_utilities::utility::ClampValueAndWarn;
using common_robotics_utilities::math::Add;
using common_robotics_utilities::math::Sub;
using common_robotics_utilities::math::Multiply;
using common_robotics_utilities::math::Divide;
using common_robotics_utilities::utility::IsSubset;
using common_robotics_utilities::utility::CollectionsEqual;
using common_robotics_utilities::utility::GetKeysFromMapLike;
using common_robotics_utilities::conversions::StdVectorDoubleToEigenVectorXd;
using common_robotics_utilities::conversions::EigenVectorXdToStdVectorDouble;
using common_robotics_utilities::time_optimal_trajectory_parametrization
          ::Trajectory;

class URTrajectoryControllerNode : public rclcpp::Node
{
private:

  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_position_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_acceleration_limits_;
  std::vector<PIDParams> joint_controller_parameters_;
  bool limit_acceleration_;

  bool current_config_valid_ = false;
  rclcpp::Time active_trajectory_start_time_;
  std::shared_ptr<Trajectory> active_trajectory_;
  std::vector<double> current_config_{};
  std::vector<double> current_velocities_{};

  std::vector<double> position_error_integrals_;
  std::vector<double> last_position_errors_;
  std::vector<double> previous_velocity_command_;

  double control_interval_;
  uint8_t iteration_count_ = 0x01;

  std::shared_ptr<
    rclcpp::Publisher<
      control_msgs::msg::JointTrajectoryControllerState>> status_pub_;
  std::shared_ptr<
    rclcpp::Publisher<
      lightweight_ur_interface::msg::VelocityCommand>> command_pub_;
  std::shared_ptr<
    rclcpp::Subscription<sensor_msgs::msg::JointState>> feedback_sub_;
  std::shared_ptr<
    rclcpp::Subscription<
      trajectory_msgs::msg::JointTrajectory>> trajectory_command_sub_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> abort_service_;
  std::shared_ptr<rclcpp::TimerBase> control_timer_;

public:

  explicit URTrajectoryControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ur_trajectory_controller", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting ur_trajectory_controller...");
    const std::string DEFAULT_TRAJECTORY_COMMAND_TOPIC
        = "/ur10/joint_command_trajectory";
    const std::string DEFAULT_STATUS_TOPIC = "/ur10_trajectory_controller/status";
    const std::string DEFAULT_STATE_FEEDBACK_TOPIC = "/ur10/joint_states";
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC
        = "/ur10/joint_command_velocity";
    const std::string DEFAULT_ABORT_SERVICE = "/ur10_trajectory_controller/abort";
    constexpr double DEFAULT_CONTROL_RATE = 200.0;
    constexpr double DEFAULT_VELOCITY_LIMIT_SCALING = 0.5;
    constexpr double DEFAULT_ACCELERATION_LIMIT_SCALING = 0.25;
    constexpr double DEFAULT_BASE_KP = 1.0;
    constexpr double DEFAULT_BASE_KD = 0.1;
    constexpr bool DEFAULT_LIMIT_ACCELERATION = false;

    // Joint limits
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

    const std::map<std::string, lightweight_ur_interface::JointLimits> limits
        = lightweight_ur_interface::GetLimits(real_velocity_limit_scaling,
                                              real_acceleration_limit_scaling);
    // Joint PID params
    const double base_kp
        = std::abs(this->declare_parameter("base_kp", DEFAULT_BASE_KP));
    const double base_kd
        = std::abs(this->declare_parameter("base_kd", DEFAULT_BASE_KD));

    const std::map<std::string, lightweight_ur_interface::PIDParams> params
        = lightweight_ur_interface::GetDefaultPositionControllerParams(
            base_kp, 0.0, base_kd, 0.0);

    limit_acceleration_
        = this->declare_parameter("limit_acceleration",
                                  DEFAULT_LIMIT_ACCELERATION);
    joint_names_ = GetKeysFromMapLike<std::string, JointLimits>(limits);
    position_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
    last_position_errors_ = std::vector<double>(joint_names_.size(), 0.0);
    previous_velocity_command_ = std::vector<double>(joint_names_.size(), 0.0);
    joint_position_limits_.resize(joint_names_.size());
    joint_velocity_limits_.resize(joint_names_.size());
    joint_acceleration_limits_.resize(joint_names_.size());
    joint_controller_parameters_.resize(joint_names_.size());
    for (size_t joint = 0; joint < joint_names_.size(); joint++)
    {
      const std::string& joint_name = joint_names_[joint];
      const JointLimits& joint_limits = limits.at(joint_name);
      joint_position_limits_[joint] = joint_limits.PositionLimits();
      joint_velocity_limits_[joint] = joint_limits.MaxVelocity();
      joint_acceleration_limits_[joint] = joint_limits.MaxAcceleration();
      joint_controller_parameters_[joint] = params.at(joint_name);
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Running with joint limits:\n%s\nand joint controller parameters:\n%s",
        Print(limits, false, "\n").c_str(), Print(params, false, "\n").c_str());

    const std::string status_topic
        = this->declare_parameter("status_topic", DEFAULT_STATUS_TOPIC);
    status_pub_ = this->create_publisher<
      control_msgs::msg::JointTrajectoryControllerState>(status_topic, 1);

    const std::string velocity_command_topic
        = this->declare_parameter("velocity_command_topic",
                                  DEFAULT_VELOCITY_COMMAND_TOPIC);
    command_pub_ = this->create_publisher<
      lightweight_ur_interface::msg::VelocityCommand>(
          velocity_command_topic, 1);

    using std::placeholders::_1;
    using std::placeholders::_2;

    const std::string state_feedback_topic
        = this->declare_parameter("state_feedback_topic",
                                  DEFAULT_STATE_FEEDBACK_TOPIC);
    auto feedback_callback = std::bind(
        &URTrajectoryControllerNode::StateFeedbackCallback, this, _1);
    feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        state_feedback_topic, 1, feedback_callback);

    const std::string trajectory_command_topic
        = this->declare_parameter("trajectory_command_topic",
                                  DEFAULT_TRAJECTORY_COMMAND_TOPIC);
    auto trajectory_command_callback = std::bind(
        &URTrajectoryControllerNode::TrajectoryCommandCallback, this, _1);
    trajectory_command_sub_
        = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            trajectory_command_topic, 1, trajectory_command_callback);

    const std::string abort_service
        = this->declare_parameter("abort_service", DEFAULT_ABORT_SERVICE);
    abort_service_ = this->create_service<std_srvs::srv::Empty>(
        abort_service,
        std::bind(&URTrajectoryControllerNode::AbortCB, this, _1, _2));

    const double control_rate
        = this->declare_parameter("control_rate", DEFAULT_CONTROL_RATE);

    control_interval_ = 1. / control_rate;

    control_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(control_interval_),
        std::bind(&URTrajectoryControllerNode::SpinOnce, this));
    RCLCPP_INFO(this->get_logger(), "...startup complete");
  }

  void SpinOnce()
  {
    constexpr uint8_t supersample_rate = 0x01;
    // Run controller
    if (iteration_count_ == supersample_rate)
    {
      if (active_trajectory_ && current_config_valid_)
      {
        const rclcpp::Time current_time = this->get_clock()->now();
        const double elapsed_time
            = (current_time - active_trajectory_start_time_).seconds();
        if (elapsed_time <= active_trajectory_->Duration())
        {
          const std::pair<Eigen::VectorXd, Eigen::VectorXd> current_pos_vel
              = active_trajectory_->GetPositionVelocity(elapsed_time);
          const std::vector<double> current_target_position
              = EigenVectorXdToStdVectorDouble(current_pos_vel.first);
          const std::vector<double> current_target_velocity
              = EigenVectorXdToStdVectorDouble(current_pos_vel.second);
          const std::vector<double> raw_command
              = ComputeRawCommandTrajectory(current_config_,
                                            current_velocities_,
                                            current_target_position,
                                            current_target_velocity,
                                            control_interval_);
          const std::vector<double> velocity_limited_command
              = LimitCommandVelocities(raw_command);
          if (limit_acceleration_)
          {
            const std::vector<double> acceleration_limited_command
                = LimitCommandAccelerations(
                    velocity_limited_command,
                    previous_velocity_command_,
                    control_interval_);
            CommandVelocities(acceleration_limited_command);
            previous_velocity_command_
                = acceleration_limited_command;
            PublishState(current_target_position,
                         acceleration_limited_command,
                         current_config_,
                         current_velocities_);
          }
          else
          {
            CommandVelocities(velocity_limited_command);
            previous_velocity_command_ = velocity_limited_command;
            PublishState(current_target_position,
                         velocity_limited_command,
                         current_config_,
                         current_velocities_);
          }
        }
        else
        {
          RCLCPP_INFO(
              this->get_logger(),
              "Reached the end of the current trajectory at %f seconds, "
              "switching to hold position",
              elapsed_time);
          const std::pair<Eigen::VectorXd, Eigen::VectorXd> end_pos_vel
              = active_trajectory_->GetPositionVelocity(
                  active_trajectory_->Duration());
          const std::vector<double> current_target_position
              = EigenVectorXdToStdVectorDouble(end_pos_vel.first);
          const std::vector<double> raw_command
              = ComputeRawCommandPosition(current_config_,
                                          current_target_position,
                                          control_interval_);
          const std::vector<double> velocity_limited_command
              = LimitCommandVelocities(raw_command);
          if (limit_acceleration_)
          {
            const std::vector<double> acceleration_limited_command
                = LimitCommandAccelerations(
                    velocity_limited_command,
                    previous_velocity_command_,
                    control_interval_);
            CommandVelocities(acceleration_limited_command);
            previous_velocity_command_
                = acceleration_limited_command;
            PublishState(current_target_position,
                         acceleration_limited_command,
                         current_config_,
                         current_velocities_);
          }
          else
          {
            CommandVelocities(velocity_limited_command);
            previous_velocity_command_ = velocity_limited_command;
            PublishState(current_target_position,
                         velocity_limited_command,
                         current_config_,
                         current_velocities_);
          }
        }
      }
      else if (current_config_valid_)
      {
        previous_velocity_command_.clear();
        previous_velocity_command_.resize(joint_names_.size(), 0.0);
        PublishState(current_config_,
                     std::vector<double>(joint_names_.size(), 0.0),
                     current_config_,
                     current_velocities_);
      }
      iteration_count_ = 0x01;
    }
    else
    {
      iteration_count_++;
    }
  }

  void PublishState(const std::vector<double>& target_config,
                    const std::vector<double>& target_velocities,
                    const std::vector<double>& current_config,
                    const std::vector<double>& current_velocities)
  {
    control_msgs::msg::JointTrajectoryControllerState state_msg;
    state_msg.joint_names = joint_names_;
    state_msg.desired.positions = target_config;
    state_msg.desired.velocities = target_velocities;
    state_msg.actual.positions = current_config;
    state_msg.actual.velocities = current_velocities;
    state_msg.error.positions = Sub(target_config, current_config);
    state_msg.error.velocities = Sub(target_velocities, current_velocities);
    status_pub_->publish(state_msg);
  }

  void AbortCB(std::shared_ptr<std_srvs::srv::Empty::Request> req,
               std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    UNUSED(req);
    UNUSED(res);
    RCLCPP_INFO(this->get_logger(), "Aborting trajectory execution");
    CommandVelocities(std::vector<double>(joint_names_.size(), 0.0));
    active_trajectory_.reset();
    position_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
    last_position_errors_ = std::vector<double>(joint_names_.size(), 0.0);
  }

  std::vector<double> ComputeNewConfigErrorIntegral(
      const std::vector<double>& raw_config_error_integral) const
  {
    std::vector<double> limited_config_error_integral(
          raw_config_error_integral.size(), 0.0);
    for (size_t idx = 0; idx < limited_config_error_integral.size(); idx++)
    {
      const double joint_config_error_integral = raw_config_error_integral[idx];
      const double i_clamp = joint_controller_parameters_[idx].Iclamp();
      const double clamped_joint_config_error_integral
          = ClampValue(joint_config_error_integral, -i_clamp, i_clamp);
      limited_config_error_integral[idx] = clamped_joint_config_error_integral;
    }
    return limited_config_error_integral;
  }

  std::vector<double> ComputeRawCommandTrajectory(
      const std::vector<double>& current_position,
      const std::vector<double>& current_velocity,
      const std::vector<double>& target_position,
      const std::vector<double>& target_velocity,
      const double time_interval)
  {
    // Compute position error
    const std::vector<double> position_errors
        = Sub(target_position, current_position);
    // Compute the integral of position error & update the stored value
    const std::vector<double> config_error_integrals_update
        = Multiply(Add(Multiply(last_position_errors_, 0.5),
                       Multiply(position_errors, 0.5)),
                   time_interval);
    position_error_integrals_
        = ComputeNewConfigErrorIntegral(Add(position_error_integrals_,
                                            config_error_integrals_update));
    // Compute the velocity error (the d term of the trajectory controller)
    const std::vector<double> velocity_errors
        = Sub(target_velocity, current_velocity);
    last_position_errors_ = position_errors;
    // Compute Feedback terms
    std::vector<double> velocity_commands(position_errors.size(), 0.0);
    for (size_t idx = 0; idx < velocity_commands.size(); idx++)
    {
      const double position_error = position_errors[idx];
      const double position_error_integral = position_error_integrals_[idx];
      const double velocity_error = velocity_errors[idx];
      const PIDParams& params = joint_controller_parameters_[idx];
      const double p_term = position_error * params.Kp();
      const double i_term = position_error_integral * params.Ki();
      const double d_term = velocity_error * params.Kd();
      const double feedback_term = p_term + i_term + d_term;
      velocity_commands[idx] = target_velocity[idx] + feedback_term;
    }
    return velocity_commands;
  }

  std::vector<double> ComputeRawCommandPosition(
      const std::vector<double>& current_config,
      const std::vector<double>& target_config,
      const double time_interval)
  {
    const std::vector<double> config_errors
        = Sub(target_config, current_config);
    const std::vector<double> config_error_integrals_update
        = Multiply(Add(Multiply(last_position_errors_, 0.5),
                       Multiply(config_errors, 0.5)),
                   time_interval);
    position_error_integrals_
        = ComputeNewConfigErrorIntegral(Add(position_error_integrals_,
                                            config_error_integrals_update));
    const std::vector<double> config_error_derivatives
        = Divide(Sub(config_errors, last_position_errors_), time_interval);
    last_position_errors_ = config_errors;
    // Compute Feedback terms
    std::vector<double> velocity_commands(config_errors.size(), 0.0);
    for (size_t idx = 0; idx < velocity_commands.size(); idx++)
    {
      const double joint_config_error = config_errors[idx];
      const double joint_config_error_integral = position_error_integrals_[idx];
      const double joint_config_error_derivative
          = config_error_derivatives[idx];
      const PIDParams& params = joint_controller_parameters_[idx];
      const double p_term = joint_config_error * params.Kp();
      const double i_term = joint_config_error_integral * params.Ki();
      const double d_term = joint_config_error_derivative * params.Kd();
      const double feedback_term = p_term + i_term + d_term;
      velocity_commands[idx] = feedback_term;
    }
    return velocity_commands;
  }

  std::vector<double> LimitCommandVelocities(
      const std::vector<double>& raw_velocities) const
  {
    std::vector<double> limited_velocities(raw_velocities.size(), 0.0);
    for (size_t idx = 0; idx < limited_velocities.size(); idx++)
    {
      const double raw_velocity = raw_velocities[idx];
      const double velocity_limit = joint_velocity_limits_[idx];
      const double limited_velocity
          = ClampValue(raw_velocity, -velocity_limit, velocity_limit);
      limited_velocities[idx] = limited_velocity;
    }
    return limited_velocities;
  }

  std::vector<double> LimitCommandAccelerations(
      const std::vector<double>& desired_velocities,
      const std::vector<double>& current_velocities,
      const double timestep) const
  {
    std::vector<double> limited_velocities(desired_velocities.size(), 0.0);
    for (size_t idx = 0; idx < limited_velocities.size(); idx++)
    {
      const double desired_velocity = desired_velocities[idx];
      const double current_velocity = current_velocities[idx];
      const double acceleration_limit = joint_acceleration_limits_[idx];
      const double desired_acceleration
          = (desired_velocity - current_velocity) / timestep;
      const double limited_acceleration
          = ClampValue(desired_acceleration,
                       -acceleration_limit,
                       acceleration_limit);
      const double limited_velocity
          = current_velocity + (limited_acceleration * timestep);
      const double fully_limited_velocity
          = ClampValue(limited_velocity,
                       -std::abs(desired_velocity),
                       std::abs(desired_velocity));
      limited_velocities[idx] = fully_limited_velocity;
    }
    return limited_velocities;
  }

  void CommandVelocities(const std::vector<double>& velocities)
  {
    if (velocities.size() == joint_names_.size())
    {
      lightweight_ur_interface::msg::VelocityCommand command_msg;
      command_msg.name = joint_names_;
      command_msg.velocity = velocities;
      command_pub_->publish(command_msg);
    }
  }

  void TrajectoryCommandCallback(
      const trajectory_msgs::msg::JointTrajectory& trajectory)
  {
    if (current_config_valid_ == false)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Ignoring trajectory command since current state is not valid");
      return;
    }
    if (trajectory.points.empty())
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Received empty trajectory, aborting current trajectory");
      active_trajectory_.reset();
    }
    else if (CollectionsEqual<std::string>(
                 trajectory.joint_names, joint_names_))
    {
      std::list<Eigen::VectorXd> ordered_waypoints;
      // Always add the current config to the beginning of the new trajectory
      ordered_waypoints.push_back(
            StdVectorDoubleToEigenVectorXd(current_config_));
      bool trajectory_valid = true;
      for (size_t tdx = 0; tdx < trajectory.points.size(); tdx++)
      {
        const trajectory_msgs::msg::JointTrajectoryPoint& current_point
            = trajectory.points[tdx];
        if (current_point.positions.size() != joint_names_.size())
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Invalid JointTrajectoryPoint: %zu positions, %zu joints",
              current_point.positions.size(), joint_names_.size());
          trajectory_valid = false;
          break;
        }
        // Push the command into a map
        std::map<std::string, double> command_map;
        for (size_t idx = 0; idx < trajectory.joint_names.size(); idx++)
        {
          const std::string& name = trajectory.joint_names[idx];
          const double position = current_point.positions[idx];
          command_map[name] = position;
        }
        // Extract the joint commands in order
        Eigen::VectorXd ordered_trajectory_point(joint_names_.size());
        for (size_t idx = 0; idx < joint_names_.size(); idx++)
        {
          // Get the name of the joint
          const std::string& joint_name = joint_names_[idx];
          // Get the position limits of the joint
          const std::pair<double, double>& joint_position_limit
              = joint_position_limits_[idx];
          // Get the commanded value
          const auto found_itr = command_map.find(joint_name);
          if (found_itr != command_map.end())
          {
            const double position = found_itr->second;
            const double limited_position
                = ClampValueAndWarn(position,
                                    joint_position_limit.first,
                                    joint_position_limit.second);
            ordered_trajectory_point(static_cast<ssize_t>(idx))
                = limited_position;
          }
          else
          {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid JointTrajectoryPoint: joint %s missing",
                joint_name.c_str());
            trajectory_valid = false;
            break;
          }
        }
        if (trajectory_valid)
        {
          ordered_waypoints.push_back(ordered_trajectory_point);
        }
        else
        {
          break;
        }
      }
      if (trajectory_valid)
      {
        // Make the trajectory
        const Eigen::VectorXd velocity_limits
            = StdVectorDoubleToEigenVectorXd(joint_velocity_limits_);
        const Eigen::VectorXd acceleration_limits
            = StdVectorDoubleToEigenVectorXd(joint_acceleration_limits_);
        const double max_path_deviation = 0.01;
        const double timestep = 0.001;
        try
        {
          std::shared_ptr<Trajectory> new_trajectory_ptr
              = std::make_shared<Trajectory>(ordered_waypoints,
                                             velocity_limits,
                                             acceleration_limits,
                                             max_path_deviation,
                                             timestep);
          RCLCPP_INFO(
              this->get_logger(), "Starting execution of new trajectory");
          active_trajectory_.reset();
          active_trajectory_ = new_trajectory_ptr;
          active_trajectory_start_time_ = this->get_clock()->now();
        }
        catch (const std::exception& ex)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "New trajectory could not be parametrized [%s]", ex.what());
          active_trajectory_.reset();
        }
      }
      else
      {
        active_trajectory_.reset();
      }
    }
    else
    {
      const std::string provided_joint_names = Print(trajectory.joint_names);
      const std::string our_joint_names = Print(joint_names_);
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid JointTrajectory, provided joint names [%s] do not match"
          " ours [%s]", provided_joint_names.c_str(), our_joint_names.c_str());
      active_trajectory_.reset();
    }
  }

  void StateFeedbackCallback(
      const sensor_msgs::msg::JointState& config_feedback)
  {
    if ((config_feedback.name.size() == config_feedback.position.size())
        && (config_feedback.name.size() == config_feedback.velocity.size())
        && IsSubset<std::string>(config_feedback.name, joint_names_))
    {
      // Push the joint state into a map
      std::map<std::string, std::pair<double, double>> joint_state_map;
      for (size_t idx = 0; idx < config_feedback.name.size(); idx++)
      {
        const std::string& name = config_feedback.name[idx];
        const double position = config_feedback.position[idx];
        const double velocity = config_feedback.velocity[idx];
        joint_state_map[name] = std::make_pair(position, velocity);
      }
      // Extract the joint state in order
      std::vector<double> current_config(joint_names_.size(), 0.0);
      std::vector<double> current_velocities(joint_names_.size(), 0.0);
      bool config_valid = true;
      for (size_t idx = 0; idx < joint_names_.size(); idx++)
      {
        const std::string& joint_name = joint_names_[idx];
        const auto found_itr = joint_state_map.find(joint_name);
        if (found_itr != joint_state_map.end())
        {
          const double position = found_itr->second.first;
          const double velocity = found_itr->second.second;
          current_config[idx] = position;
          current_velocities[idx] = velocity;
        }
        else
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Invalid JointState feedback: joint %s missing",
              joint_name.c_str());
          config_valid = false;
        }
      }
      if (config_valid == true)
      {
        current_config_ = current_config;
        current_velocities_ = current_velocities;
        current_config_valid_ = true;
      }
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Invalid JointState feedback: %zu names, %zu positions",
          config_feedback.name.size(), config_feedback.position.size());
      current_config_valid_ = false;
    }
  }
};
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lightweight_ur_interface::URTrajectoryControllerNode)
