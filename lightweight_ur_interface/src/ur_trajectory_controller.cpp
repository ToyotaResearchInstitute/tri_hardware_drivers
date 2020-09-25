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
#include <trajectory_msgs/JointTrajectory.h>
#include <lightweight_ur_interface/ur_robot_config.hpp>
#include <lightweight_ur_interface/VelocityCommand.h>
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
using common_robotics_utilities::utility::SetsEqual;
using common_robotics_utilities::utility::GetKeysFromMapLike;
using common_robotics_utilities::conversions::StdVectorDoubleToEigenVectorXd;
using common_robotics_utilities::conversions::EigenVectorXdToStdVectorDouble;
using common_robotics_utilities::time_optimal_trajectory_parametrization
          ::Trajectory;

class URTrajectoryController
{
private:

  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_position_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_acceleration_limits_;
  std::vector<PIDParams> joint_controller_parameters_;
  bool limit_acceleration_;

  bool current_config_valid_;
  ros::Time active_trajectory_start_time_;
  std::shared_ptr<Trajectory> active_trajectory_;
  std::vector<double> current_config_;
  std::vector<double> current_velocities_;

  std::vector<double> position_error_integrals_;
  std::vector<double> last_position_errors_;

  ros::NodeHandle nh_;
  ros::Publisher status_pub_;
  ros::Publisher command_pub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber trajectory_command_sub_;
  ros::ServiceServer abort_server_;

public:

  URTrajectoryController(
      const ros::NodeHandle& nh,
      const std::string& trajectory_command_topic,
      const std::string& state_feedback_topic,
      const std::string& velocity_command_topic,
      const std::string& status_topic,
      const std::string& abort_service,
      const std::map<std::string, JointLimits>& joint_limits,
      const std::map<std::string, PIDParams> joint_controller_params,
      const bool limit_acceleration)
    : nh_(nh)
  {
    limit_acceleration_ = limit_acceleration;
    current_config_valid_ = false;
    current_config_ = std::vector<double>();
    current_velocities_ = std::vector<double>();
    joint_names_ = GetKeysFromMapLike<std::string, JointLimits>(joint_limits);
    position_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
    last_position_errors_ = std::vector<double>(joint_names_.size(), 0.0);
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
                        &URTrajectoryController::StateFeedbackCallback,
                        this);
    trajectory_command_sub_
        = nh_.subscribe(trajectory_command_topic,
                        1,
                        &URTrajectoryController::TrajectoryCommandCallback,
                        this);
    abort_server_
        = nh_.advertiseService(abort_service,
                               &URTrajectoryController::AbortCB,
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
        if (active_trajectory_ && current_config_valid_)
        {
          const double elapsed_time
              = (ros::Time::now() - active_trajectory_start_time_).toSec();
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
                                              control_interval);
            const std::vector<double> velocity_limited_command
                = LimitCommandVelocities(raw_command);
            if (limit_acceleration_)
            {
              const std::vector<double> acceleration_limited_command
                  = LimitCommandAccelerations(
                      velocity_limited_command,
                      previous_velocity_command,
                      control_interval);
              CommandVelocities(acceleration_limited_command);
              previous_velocity_command
                  = acceleration_limited_command;
              PublishState(current_target_position,
                           acceleration_limited_command,
                           current_config_,
                           current_velocities_);
            }
            else
            {
              CommandVelocities(velocity_limited_command);
              previous_velocity_command = velocity_limited_command;
              PublishState(current_target_position,
                           velocity_limited_command,
                           current_config_,
                           current_velocities_);
            }
          }
          else
          {
            ROS_INFO_ONCE_NAMED(ros::this_node::getName(),
                                "Reached the end of the current trajectory"
                                " @ %f seconds, switching to hold position",
                                elapsed_time);
            const std::pair<Eigen::VectorXd, Eigen::VectorXd> end_pos_vel
                = active_trajectory_->GetPositionVelocity(
                    active_trajectory_->Duration());
            const std::vector<double> current_target_position
                = EigenVectorXdToStdVectorDouble(end_pos_vel.first);
            const std::vector<double> raw_command
                = ComputeRawCommandPosition(current_config_,
                                            current_target_position,
                                            control_interval);
            const std::vector<double> velocity_limited_command
                = LimitCommandVelocities(raw_command);
            if (limit_acceleration_)
            {
              const std::vector<double> acceleration_limited_command
                  = LimitCommandAccelerations(
                      velocity_limited_command,
                      previous_velocity_command,
                      control_interval);
              CommandVelocities(acceleration_limited_command);
              previous_velocity_command
                  = acceleration_limited_command;
              PublishState(current_target_position,
                           acceleration_limited_command,
                           current_config_,
                           current_velocities_);
            }
            else
            {
              CommandVelocities(velocity_limited_command);
              previous_velocity_command = velocity_limited_command;
              PublishState(current_target_position,
                           velocity_limited_command,
                           current_config_,
                           current_velocities_);
            }
          }
        }
        else if (current_config_valid_)
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
    active_trajectory_.reset();
    position_error_integrals_ = std::vector<double>(joint_names_.size(), 0.0);
    last_position_errors_ = std::vector<double>(joint_names_.size(), 0.0);
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
      lightweight_ur_interface::VelocityCommand command_msg;
      command_msg.name = joint_names_;
      command_msg.velocity = velocities;
      command_pub_.publish(command_msg);
    }
  }

  void TrajectoryCommandCallback(trajectory_msgs::JointTrajectory trajectory)
  {
    if (current_config_valid_ == false)
    {
      ROS_WARN_NAMED(ros::this_node::getName(),
                     "Ignoring trajectory command since current state "
                     "is not valid");
      return;
    }
    if (trajectory.points.empty())
    {
      ROS_INFO_NAMED(ros::this_node::getName(),
                     "Received empty trajectory, aborting current trajectory");
      active_trajectory_.reset();
    }
    else if (SetsEqual(trajectory.joint_names, joint_names_))
    {
      std::list<Eigen::VectorXd> ordered_waypoints;
      // Always add the current config to the beginning of the new trajectory
      ordered_waypoints.push_back(
            StdVectorDoubleToEigenVectorXd(current_config_));
      bool trajectory_valid = true;
      for (size_t tdx = 0; tdx < trajectory.points.size(); tdx++)
      {
        const trajectory_msgs::JointTrajectoryPoint& current_point
            = trajectory.points[tdx];
        if (current_point.positions.size() != joint_names_.size())
        {
          ROS_WARN_NAMED(ros::this_node::getName(),
                         "Invalid JointTrajectoryPoint: %zu positions,"
                         " %zu joints",
                         current_point.positions.size(),
                         joint_names_.size());
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
            ROS_WARN_NAMED(ros::this_node::getName(),
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
          ROS_INFO_NAMED(ros::this_node::getName(),
                         "Starting execution of new trajectory");
          active_trajectory_.reset();
          active_trajectory_ = new_trajectory_ptr;
          active_trajectory_start_time_ = ros::Time::now();
        }
        catch (...)
        {
          ROS_WARN_NAMED(ros::this_node::getName(),
                         "New trajectory could not be parametrized");
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
      ROS_WARN_NAMED(ros::this_node::getName(),
                     "Invalid JointTrajectory, provided joint names [%s] "
                     "do not match ours [%s]",
                     provided_joint_names.c_str(),
                     our_joint_names.c_str());
      active_trajectory_.reset();
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
        current_config_valid_ = true;
      }
    }
    else
    {
      ROS_WARN_NAMED(ros::this_node::getName(),
                     "Invalid JointState feedback: %zu names, %zu positions",
                     config_feedback.name.size(),
                     config_feedback.position.size());
      current_config_valid_ = false;
    }
  }
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_trajectory_controller");
  ROS_INFO_NAMED(ros::this_node::getName(),
                 "Starting ur_trajectory_controller...");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  const std::string DEFAULT_TRAJECTORY_COMMAND_TOPIC
      = "/ur10/joint_command_trajectory";
  const std::string DEFAULT_STATUS_TOPIC = "/ur10_trajectory_controller/status";
  const std::string DEFAULT_STATE_FEEDBACK_TOPIC = "/ur10/joint_states";
  const std::string DEFAULT_VELOCITY_COMMAND_TOPIC
      = "/ur10/joint_command_velocity";
  const std::string DEFAULT_ABORT_SERVICE = "/ur10_trajectory_controller/abort";
  const double DEFAULT_CONTROL_RATE = 200.0;
  const double DEFAULT_VELOCITY_LIMIT_SCALING = 0.5;
  const double DEFAULT_ACCELERATION_LIMIT_SCALING = 0.25;
  const double DEFAULT_BASE_KP = 1.0;
  const double DEFAULT_BASE_KD = 0.1;
  const bool DEFAULT_LIMIT_ACCELERATION = false;
  const std::string trajectory_command_topic
      = nhp.param(std::string("trajectory_command_topic"),
                  DEFAULT_TRAJECTORY_COMMAND_TOPIC);
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
  lightweight_ur_interface::URTrajectoryController controller(
        nh,
        trajectory_command_topic,
        state_feedback_topic,
        velocity_command_topic,
        status_topic,
        abort_service,
        limits,
        params,
        limit_acceleration);
  ROS_INFO_NAMED(ros::this_node::getName(), "...startup complete");
  controller.Loop(control_rate);
  return 0;
}
