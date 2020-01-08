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
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <lightweight_ur_interface/ur_robot_config.hpp>
#include <lightweight_ur_interface/PositionCommand.h>
#include <lightweight_ur_interface/VelocityCommand.h>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/conversions.hpp>

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
using common_robotics_utilities::utility::GetKeys;

class URPositionController
{
private:

  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_position_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_acceleration_limits_;
  std::vector<PIDParams> joint_controller_parameters_;
  bool limit_acceleration_;
  bool autoscale_velocities_;

  bool target_config_valid_;
  bool current_state_valid_;
  std::vector<double> target_config_;
  std::vector<double> current_config_;
  std::vector<double> current_velocities_;

  std::vector<double> config_error_integrals_;
  std::vector<double> last_config_errors_;

  ros::NodeHandle nh_;
  ros::Publisher status_pub_;
  ros::Publisher command_pub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber position_command_sub_;
  ros::ServiceServer abort_server_;

public:

  URPositionController(
      const ros::NodeHandle& nh,
      const std::string& position_command_topic,
      const std::string& state_feedback_topic,
      const std::string& velocity_command_topic,
      const std::string& status_topic,
      const std::string& abort_service,
      const std::map<std::string, JointLimits>& joint_limits,
      const std::map<std::string, PIDParams> joint_controller_params,
      const bool limit_acceleration,
      const bool autoscale_velocities)
    : nh_(nh)
  {
    limit_acceleration_ = limit_acceleration;
    autoscale_velocities_ = autoscale_velocities;
    target_config_valid_ = false;
    current_state_valid_ = false;
    target_config_ = std::vector<double>();
    current_config_ = std::vector<double>();
    current_velocities_ = std::vector<double>();
    joint_names_ = GetKeys(joint_limits);
    config_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
    last_config_errors_ = std::vector<double>(joint_names_.size(), 0.0);
    joint_position_limits_.resize(joint_names_.size());
    joint_velocity_limits_.resize(joint_names_.size());
    joint_acceleration_limits_.resize(joint_names_.size());
    joint_controller_parameters_.resize(joint_names_.size());
    for (size_t joint = 0; joint < joint_names_.size(); joint++)
    {
      const std::string& joint_name = joint_names_[joint];
      const JointLimits& limits = joint_limits.at(joint_name);
      joint_position_limits_[joint] = limits.PositionLimits();
      joint_velocity_limits_[joint] = limits.MaxVelocity();
      joint_acceleration_limits_[joint] = limits.MaxAcceleration();
      const PIDParams& params = joint_controller_params.at(joint_name);
      joint_controller_parameters_[joint] = params;
    }
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Running with joint limits:\n%s\nand"
                   " joint controller parameters:\n%s",
                   Print(joint_limits, false, "\n").c_str(),
                   Print(joint_controller_params, false, "\n").c_str());
    status_pub_
        = nh_.advertise<control_msgs::JointTrajectoryControllerState>(
            status_topic, 1, false);
    command_pub_
        = nh_.advertise<lightweight_ur_interface::VelocityCommand>(
            velocity_command_topic, 1, false);
    feedback_sub_
        = nh_.subscribe(state_feedback_topic,
                        1,
                        &URPositionController::StateFeedbackCallback,
                        this);
    position_command_sub_
        = nh_.subscribe(position_command_topic,
                        1,
                        &URPositionController::PositionCommandCallback,
                        this);
    abort_server_
        = nh_.advertiseService(abort_service,
                               &URPositionController::AbortCB,
                               this);
  }

  void Loop(const double control_rate)
  {
    const double control_interval = 1.0 / control_rate;
    ros::Rate spin_rate(control_rate);
    std::vector<double> previous_velocity_command(joint_names_.size(), 0.0);
    const uint8_t supersample_rate = 0x01;
    uint8_t iteration_count = 0x01;
    while (ros::ok())
    {
      // Process callbacks
      ros::spinOnce();
      // Run controller
      if (iteration_count == supersample_rate)
      {
        if (target_config_valid_ && current_state_valid_)
        {
          const std::vector<double> raw_config_correction
              = ComputeRawNextStep(current_config_,
                                   target_config_,
                                   control_interval);
          const std::vector<double> config_correction
              = (autoscale_velocities_)
                ? AutoscaleCorrection(current_config_,
                                      target_config_,
                                      raw_config_correction)
                : raw_config_correction;
          const std::vector<double> velocity_limited_config_correction
              = LimitCorrectionVelocities(config_correction);
          if (limit_acceleration_)
          {
            const std::vector<double> acceleration_limited_config_correction
                = LimitCorrectionAccelerations(
                    velocity_limited_config_correction,
                    previous_velocity_command,
                    control_interval);
            CommandVelocities(acceleration_limited_config_correction);
            previous_velocity_command = acceleration_limited_config_correction;
            PublishState(target_config_,
                         acceleration_limited_config_correction,
                         current_config_,
                         current_velocities_);
          }
          else
          {
            CommandVelocities(velocity_limited_config_correction);
            previous_velocity_command = velocity_limited_config_correction;
            PublishState(target_config_,
                         velocity_limited_config_correction,
                         current_config_,
                         current_velocities_);
          }
        }
        else if (current_state_valid_)
        {
          previous_velocity_command.clear();
          previous_velocity_command.resize(joint_names_.size(), 0.0);
          PublishState(current_config_,
                       std::vector<double>(joint_names_.size(), 0.0),
                       current_config_,
                       current_velocities_);
        }
        iteration_count = 0x01;
      }
      else
      {
        iteration_count++;
      }
      // Spin
      spin_rate.sleep();
    }
  }

  void PublishState(const std::vector<double>& target_config,
                    const std::vector<double>& target_velocities,
                    const std::vector<double>& current_config,
                    const std::vector<double>& current_velocities)
  {
    control_msgs::JointTrajectoryControllerState state_msg;
    state_msg.joint_names = joint_names_;
    state_msg.desired.positions = target_config;
    state_msg.desired.velocities = target_velocities;
    state_msg.actual.positions = current_config;
    state_msg.actual.velocities = current_velocities;
    state_msg.error.positions = Sub(target_config, current_config);
    state_msg.error.velocities = Sub(target_velocities, current_velocities);
    status_pub_.publish(state_msg);
  }

  bool AbortCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    UNUSED(req);
    UNUSED(res);
    ROS_INFO_NAMED(ros::this_node::getName(), "Aborting config target");
    CommandVelocities(std::vector<double>(joint_names_.size(), 0.0));
    target_config_valid_ = false;
    config_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
    last_config_errors_ = std::vector<double>(joint_names_.size(), 0.0);
    return true;
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

  std::vector<double> ComputeRawNextStep(
      const std::vector<double>& current_config,
      const std::vector<double>& target_config,
      const double time_interval)
  {
    const std::vector<double> config_errors
        = Sub(target_config, current_config);
    const std::vector<double> config_error_integrals_update
        = Multiply(Add(Multiply(last_config_errors_, 0.5),
                       Multiply(config_errors, 0.5)),
                   time_interval);
    config_error_integrals_
        = ComputeNewConfigErrorIntegral(Add(config_error_integrals_,
                                            config_error_integrals_update));
    const std::vector<double> config_error_derivatives
        = Divide(Sub(config_errors, last_config_errors_), time_interval);
    last_config_errors_ = config_errors;
    // Compute Feedback terms
    std::vector<double> feedback_terms(config_errors.size(), 0.0);
    for (size_t idx = 0; idx < feedback_terms.size(); idx++)
    {
      const double joint_config_error = config_errors[idx];
      const double joint_config_error_integral = config_error_integrals_[idx];
      const double joint_config_error_derivative
          = config_error_derivatives[idx];
      const PIDParams& params = joint_controller_parameters_[idx];
      const double p_term = joint_config_error * params.Kp();
      const double i_term = joint_config_error_integral * params.Ki();
      const double d_term = joint_config_error_derivative * params.Kd();
      const double feedback_term = p_term + i_term + d_term;
      feedback_terms[idx] = feedback_term;
    }
    return feedback_terms;
  }

  std::vector<double> AutoscaleCorrection(
      const std::vector<double>& current_position,
      const std::vector<double>& target_position,
      const std::vector<double>& raw_correction) const
  {
    const std::vector<double> position_error
        = Sub(target_position, current_position);
    const double max_position_error
        = *std::max_element(position_error.begin(), position_error.end());
    std::vector<double> autoscaled_correction(raw_correction.size(), 0.0);
    for (size_t idx = 0; idx < autoscaled_correction.size(); idx++)
    {
      const double raw_axis_correction = raw_correction[idx];
      const double axis_error = position_error[idx];
      const double axis_error_ratio
          = std::abs(axis_error / max_position_error);
      const double autoscaled_axis_correction
          = raw_axis_correction * axis_error_ratio;
      autoscaled_correction[idx] = autoscaled_axis_correction;
    }
    return autoscaled_correction;
  }

  std::vector<double> LimitCorrectionVelocities(
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

  std::vector<double> LimitCorrectionAccelerations(
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
      lightweight_ur_interface::VelocityCommand command_msg;
      command_msg.name = joint_names_;
      command_msg.velocity = velocities;
      command_pub_.publish(command_msg);
    }
  }

  void PositionCommandCallback(
      lightweight_ur_interface::PositionCommand config_target)
  {
    if (config_target.name.size() == config_target.position.size())
    {
      // Push the command into a map
      std::map<std::string, double> command_map;
      for (size_t idx = 0; idx < config_target.name.size(); idx++)
      {
        const std::string& name = config_target.name[idx];
        const double command = config_target.position[idx];
        command_map[name] = command;
      }
      // Extract the joint commands in order
      std::vector<double> target_config(joint_names_.size(), 0.0);
      bool command_valid = true;
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
          target_config[idx] = limited_position;
        }
        else
        {
          ROS_WARN_NAMED(ros::this_node::getName(),
                         "Invalid PositionCommand: joint %s missing",
                         joint_name.c_str());
          command_valid = false;
        }
      }
      if (command_valid == true)
      {
        ROS_INFO_NAMED(ros::this_node::getName(),
                       "Starting execution to a new target configuration");
        target_config_ = target_config;
        target_config_valid_ = true;
      }
      else
      {
        target_config_valid_ = false;
      }
    }
    else
    {
      ROS_WARN_NAMED(ros::this_node::getName(),
                     "Invalid PositionCommand: %zu names, %zu positions",
                     config_target.name.size(),
                     config_target.position.size());
      target_config_valid_ = false;
    }
  }

  void StateFeedbackCallback(sensor_msgs::JointState config_feedback)
  {
    if ((config_feedback.name.size() == config_feedback.position.size())
        && (config_feedback.name.size() == config_feedback.velocity.size())
        && IsSubset(config_feedback.name, joint_names_))
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
          ROS_WARN_NAMED(ros::this_node::getName(),
                         "Invalid JointState feedback: joint %s missing",
                         joint_name.c_str());
          config_valid = false;
        }
      }
      if (config_valid == true)
      {
        current_config_ = current_config;
        current_velocities_ = current_velocities;
        current_state_valid_ = true;
      }
    }
    else
    {
      ROS_WARN_NAMED(ros::this_node::getName(),
                     "Invalid JointState feedback: %zu names, %zu positions",
                     config_feedback.name.size(),
                     config_feedback.position.size());
      current_state_valid_ = false;
    }
  }
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_position_controller");
  ROS_INFO_NAMED(ros::this_node::getName(),
                 "Starting ur_position_controller...");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  const std::string DEFAULT_POSITION_COMMAND_TOPIC
      = "/ur10/joint_command_position";
  const std::string DEFAULT_STATUS_TOPIC
      = "/ur10_position_controller/status";
  const std::string DEFAULT_STATE_FEEDBACK_TOPIC
      = "/ur10/joint_states";
  const std::string DEFAULT_VELOCITY_COMMAND_TOPIC
      = "/ur10/joint_command_velocity";
  const std::string DEFAULT_ABORT_SERVICE
      = "/ur10_position_controller/abort";
  const double DEFAULT_CONTROL_RATE = 150.0;
  const double DEFAULT_VELOCITY_LIMIT_SCALING = 0.5;
  const double DEFAULT_ACCELERATION_LIMIT_SCALING = 0.5;
  const double DEFAULT_BASE_KP = 1.0;
  const double DEFAULT_BASE_KD = 0.1;
  const bool DEFAULT_LIMIT_ACCELERATION = false;
  const bool DEFAULT_ENABLE_VELOCITY_AUTOSCALING = true;
  const std::string position_command_topic
      = nhp.param(std::string("position_command_topic"),
                  DEFAULT_POSITION_COMMAND_TOPIC);
  const std::string state_feedback_topic
      = nhp.param(std::string("state_feedback_topic"),
                  DEFAULT_STATE_FEEDBACK_TOPIC);
  const std::string velocity_command_topic
      = nhp.param(std::string("velocity_command_topic"),
                  DEFAULT_VELOCITY_COMMAND_TOPIC);
  const std::string status_topic
      = nhp.param(std::string("status_topic"), DEFAULT_STATUS_TOPIC);
  const std::string abort_service
      = nhp.param(std::string("abort_service"), DEFAULT_ABORT_SERVICE);
  const double control_rate
      = nhp.param(std::string("control_rate"), DEFAULT_CONTROL_RATE);
  const double velocity_limit_scaling
      = std::abs(nhp.param(std::string("velocity_limit_scaling"),
                           DEFAULT_VELOCITY_LIMIT_SCALING));
  const double acceleration_limit_scaling
      = std::abs(nhp.param(std::string("acceleration_limit_scaling"),
                           DEFAULT_ACCELERATION_LIMIT_SCALING));
  const double real_velocity_limit_scaling
      = common_robotics_utilities::utility::ClampValueAndWarn(
          velocity_limit_scaling, 0.0, 1.0);
  const double real_acceleration_limit_scaling
      = common_robotics_utilities::utility::ClampValueAndWarn(
          acceleration_limit_scaling, 0.0, 1.0);
  const double base_kp
      = std::abs(nhp.param(std::string("base_kp"), DEFAULT_BASE_KP));
  const double base_kd
      = std::abs(nhp.param(std::string("base_kd"), DEFAULT_BASE_KD));
  const bool limit_acceleration
      = nhp.param(std::string("limit_acceleration"),
                  DEFAULT_LIMIT_ACCELERATION);
  const bool enable_velocity_autoscaling
      = nhp.param(std::string("enable_velocity_autoscaling"),
                  DEFAULT_ENABLE_VELOCITY_AUTOSCALING);
  // Joint limits
  const std::map<std::string, lightweight_ur_interface::JointLimits> limits
      = lightweight_ur_interface::GetLimits(real_velocity_limit_scaling,
                                            real_acceleration_limit_scaling);
  // Joint PID params
  const std::map<std::string, lightweight_ur_interface::PIDParams> params
      = lightweight_ur_interface::GetDefaultPositionControllerParams(base_kp,
                                                                     0.0,
                                                                     base_kd,
                                                                     0.0);
  lightweight_ur_interface::URPositionController
      controller(nh,
                 position_command_topic,
                 state_feedback_topic,
                 velocity_command_topic,
                 status_topic,
                 abort_service,
                 limits,
                 params,
                 limit_acceleration,
                 enable_velocity_autoscaling);
  ROS_INFO_NAMED(ros::this_node::getName(), "...startup complete");
  controller.Loop(control_rate);
  return 0;
}
