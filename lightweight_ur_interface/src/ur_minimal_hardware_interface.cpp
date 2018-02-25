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
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <lightweight_ur_interface/ur_robot_config.hpp>
#include <lightweight_ur_interface/ur_minimal_realtime_driver.hpp>
#include <lightweight_ur_interface/VelocityCommand.h>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>

namespace lightweight_ur_interface
{
    class URMinimalHardwareInterface
    {
    protected:

        std::string base_frame_;
        std::string ee_frame_;
        std::vector<std::string> joint_names_;
        std::map<std::string, JointLimits> joint_limits_;
        double max_acceleration_limit_;

        ros::NodeHandle nh_;
        ros::Publisher joint_state_pub_;
        ros::Publisher ee_pose_pub_;
        ros::Publisher ee_body_twist_pub_;
        ros::Publisher ee_world_twist_pub_;
        ros::Publisher ee_wrench_pub_;
        ros::Subscriber velocity_command_sub_;
        ros::Subscriber twist_command_sub_;

        std::unique_ptr<URRealtimeInterface> robot_ptr_;
        std::mutex latest_state_mutex_;
        bool valid_latest_state_;
        sensor_msgs::JointState latest_joint_state_;
        Eigen::Isometry3d latest_tcp_pose_;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        URMinimalHardwareInterface(ros::NodeHandle& nh,
                                   const std::string& velocity_command_topic,
                                   const std::string& twist_command_topic,
                                   const std::string& joint_state_topic,
                                   const std::string& ee_pose_topic,
                                   const std::string& ee_world_twist_topic,
                                   const std::string& ee_body_twist_topic,
                                   const std::string& ee_wrench_topic,
                                   const std::string& base_frame,
                                   const std::string& ee_frame,
                                   const std::vector<std::string>& ordered_joint_names,
                                   const std::map<std::string, JointLimits>& joint_limits,
                                   const std::string& robot_host) : nh_(nh)
        {
            valid_latest_state_ = false;
            // Make sure our ordered joint names match our joint limits
            joint_names_ = ordered_joint_names;
            if (joint_names_.size() != 6)
            {
                throw std::invalid_argument("There must be exactly 6 joints");
            }
            if (SetsEqual(joint_names_, arc_helpers::GetKeys(joint_limits)) == false)
            {
                throw std::invalid_argument("Ordered joint names do not match those provided with joint limits");
            }
            base_frame_ = base_frame;
            ee_frame_ = ee_frame;
            joint_limits_ = joint_limits;
            // Compute the maximum joint acceleration
            max_acceleration_limit_ = 0.0;
            for (auto itr = joint_limits_.begin(); itr != joint_limits_.end(); ++itr)
            {
                const JointLimits& joint_limit = itr->second;
                const double acceleration_limit = joint_limit.MaxAcceleration();
                max_acceleration_limit_ = std::max(max_acceleration_limit_, acceleration_limit);
            }
            ROS_INFO_NAMED(ros::this_node::getName(), "Set max_acceleration_limit to %f", max_acceleration_limit_);
            joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic, 1, false);
            ee_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(ee_pose_topic, 1, false);
            ee_world_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(ee_world_twist_topic, 1, false);
            ee_body_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(ee_body_twist_topic, 1, false);
            ee_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(ee_wrench_topic, 1, false);
            velocity_command_sub_ = nh_.subscribe(velocity_command_topic, 1, &URMinimalHardwareInterface::VelocityCommandCallback, this);
            twist_command_sub_ = nh_.subscribe(twist_command_topic, 1, &URMinimalHardwareInterface::TwistCommandCallback, this);
            // Build robot interface
            const std::function<void(const URRealtimeState&)> callback_fn = [&] (const URRealtimeState& latest_state) { return PublishState(joint_names_, base_frame_, ee_frame_, latest_state); };
            std::function<void(const std::string&)> logging_fn = [] (const std::string& message) { ROS_INFO_NAMED(ros::this_node::getName(), "%s", message.c_str()); };
            robot_ptr_ = std::unique_ptr<URRealtimeInterface>(new URRealtimeInterface(robot_host, callback_fn, logging_fn));
        }

        void Run(const double control_rate)
        {
            // Start robot interface
            robot_ptr_->StartRecv();
            // Start ROS spinloop
            ros::Rate looprate(control_rate);
            while (nh_.ok())
            {
                ros::spinOnce();
                looprate.sleep();
            }
            robot_ptr_->StopRecv();
        }

        void PublishState(const std::vector<std::string>& joint_names,
                          const std::string& base_frame_name,
                          const std::string& ee_frame_name,
                          const URRealtimeState& robot_state)
        {
            // Get the current time
            const ros::Time state_time = ros::Time::now();
            // Joint State
            sensor_msgs::JointState joint_state_msg;
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
            geometry_msgs::PoseStamped ee_transform_msg = EigenHelpersConversions::EigenIsometry3dToGeometryPoseStamped(robot_state.ActualTcpPose(), base_frame_);
            ee_transform_msg.header.stamp = state_time;
            // EE twist
            const Eigen::Matrix<double, 6, 1>& ee_world_twist = robot_state.ActualTcpTwist();
            geometry_msgs::TwistStamped ee_world_twist_msg;
            ee_world_twist_msg.header.stamp = state_time;
            ee_world_twist_msg.header.frame_id = base_frame_name;
            ee_world_twist_msg.twist.linear = EigenHelpersConversions::EigenVector3dToGeometryVector3(ee_world_twist.block<3, 1>(0, 0));
            ee_world_twist_msg.twist.angular = EigenHelpersConversions::EigenVector3dToGeometryVector3(ee_world_twist.block<3, 1>(3, 0));
            const Eigen::Quaterniond ee_rotation(robot_state.ActualTcpPose().rotation());
            const Eigen::Vector3d body_frame_linear_velocity = EigenHelpers::RotateVectorReverse(ee_rotation, ee_world_twist.block<3, 1>(0, 0));
            const Eigen::Vector3d body_frame_angular_velocity = EigenHelpers::RotateVectorReverse(ee_rotation, ee_world_twist.block<3, 1>(3, 0));
            geometry_msgs::TwistStamped ee_body_twist_msg;
            ee_body_twist_msg.header.stamp = state_time;
            ee_body_twist_msg.header.frame_id = ee_frame_name;
            ee_body_twist_msg.twist.linear = EigenHelpersConversions::EigenVector3dToGeometryVector3(body_frame_linear_velocity);
            ee_body_twist_msg.twist.angular = EigenHelpersConversions::EigenVector3dToGeometryVector3(body_frame_angular_velocity);
            // EE wrench
            const Eigen::Matrix<double, 6, 1>& ee_wrench = robot_state.ActualTcpWrench();
            geometry_msgs::WrenchStamped ee_wrench_msg;
            ee_wrench_msg.header.stamp = state_time;
            ee_wrench_msg.header.frame_id = ee_frame_name;
            ee_wrench_msg.wrench.force = EigenHelpersConversions::EigenVector3dToGeometryVector3(ee_wrench.block<3, 1>(0, 0));
            ee_wrench_msg.wrench.torque = EigenHelpersConversions::EigenVector3dToGeometryVector3(ee_wrench.block<3, 1>(3, 0));
            // Publish
            joint_state_pub_.publish(joint_state_msg);
            ee_pose_pub_.publish(ee_transform_msg);
            ee_world_twist_pub_.publish(ee_world_twist_msg);
            ee_body_twist_pub_.publish(ee_body_twist_msg);
            ee_wrench_pub_.publish(ee_wrench_msg);
        }

        void SendVelocityCommand(const std::vector<double>& command)
        {
            const double max_command_acceleration = 3.2;
            // We need to determine the appropriate max acceleration here?
            const int max_str_len = 1024;
            char command_str_buffer[max_str_len];
            const int written = snprintf(command_str_buffer, max_str_len - 1, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n", command[0], command[1], command[2], command[3], command[4], command[5], max_command_acceleration);
            assert(written > 0);
            assert(written < max_str_len);
            const std::string command_str(command_str_buffer);
            const bool success = robot_ptr_->SendURScriptCommand(command_str);
            if (!success)
            {
                throw std::runtime_error("Failed to send speedj(...) command");
            }
        }

        void VelocityCommandCallback(lightweight_ur_interface::VelocityCommand config_target)
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
                            const double velocity_limit = limits_found_itr->second.MaxVelocity();
                            const double limited_velocity = arc_helpers::ClampValueAndWarn(velocity, -velocity_limit, velocity_limit);
                            target_velocity[idx] = limited_velocity;
                        }
                        // If we don't have limits saved, then we don't need to limit the joint value
                        else
                        {
                            target_velocity[idx] = velocity;
                        }
                    }
                    else
                    {
                        ROS_WARN_NAMED(ros::this_node::getName(), "Invalid VelocityCommand: joint %s missing", joint_name.c_str());
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
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid VelocityCommand: %zu names, %zu velocities", config_target.name.size(), config_target.velocity.size());
            }
        }

        void SendTwistCommand(const std::vector<double>& command)
        {
            const double max_command_acceleration = 3.2;
            // We need to determine the appropriate max acceleration here?
            const int max_str_len = 1024;
            char command_str_buffer[max_str_len];
            const int written = snprintf(command_str_buffer, max_str_len - 1, "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n", command[0], command[1], command[2], command[3], command[4], command[5], max_command_acceleration);
            assert(written > 0);
            assert(written < max_str_len);
            const std::string command_str(command_str_buffer);
            const bool success = robot_ptr_->SendURScriptCommand(command_str);
            if (!success)
            {
                throw std::runtime_error("Failed to send speedl(...) command");
            }
        }

        void TwistCommandCallback(geometry_msgs::TwistStamped twist_command)
        {
            bool valid_twist = true;
            if (std::isinf(twist_command.twist.linear.x) || std::isnan(twist_command.twist.linear.x))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist command, linear.x is NAN or INF");
                valid_twist = false;
            }
            if (std::isinf(twist_command.twist.linear.y) || std::isnan(twist_command.twist.linear.y))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist command, linear.y is NAN or INF");
                valid_twist = false;
            }
            if (std::isinf(twist_command.twist.linear.z) || std::isnan(twist_command.twist.linear.z))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist command, linear.z is NAN or INF");
                valid_twist = false;
            }
            if (std::isinf(twist_command.twist.angular.x) || std::isnan(twist_command.twist.angular.x))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist command, angular.x is NAN or INF");
                valid_twist = false;
            }
            if (std::isinf(twist_command.twist.angular.y) || std::isnan(twist_command.twist.angular.y))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist command, angular.y is NAN or INF");
                valid_twist = false;
            }
            if (std::isinf(twist_command.twist.angular.z) || std::isnan(twist_command.twist.angular.z))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist command, angular.z is NAN or INF");
                valid_twist = false;
            }
            if (valid_twist)
            {
                const std::vector<double> raw_twist = {twist_command.twist.linear.x, twist_command.twist.linear.y, twist_command.twist.linear.z, twist_command.twist.angular.x, twist_command.twist.angular.y, twist_command.twist.angular.z};
                if (twist_command.header.frame_id == ee_frame_)
                {
                    latest_state_mutex_.lock();
                    const Eigen::Isometry3d latest_tcp_pose = latest_tcp_pose_;
                    const bool valid_latest_state = valid_latest_state_;
                    latest_state_mutex_.unlock();
                    if (valid_latest_state)
                    {
                        const Eigen::Quaterniond latest_tcp_rotation(latest_tcp_pose.rotation());
                        const Eigen::Vector3d ee_frame_linear_velocity(raw_twist[0], raw_twist[1], raw_twist[2]);
                        const Eigen::Vector3d ee_frame_angular_velocity(raw_twist[3], raw_twist[4], raw_twist[5]);
                        const Eigen::Vector3d base_frame_linear_velocity = EigenHelpers::RotateVector(latest_tcp_rotation, ee_frame_linear_velocity);
                        const Eigen::Vector3d base_frame_angular_velocity = EigenHelpers::RotateVector(latest_tcp_rotation, ee_frame_angular_velocity);
                        const std::vector<double> base_frame_twist = {base_frame_linear_velocity.x(),
                                                                      base_frame_linear_velocity.y(),
                                                                      base_frame_linear_velocity.z(),
                                                                      base_frame_angular_velocity.x(),
                                                                      base_frame_angular_velocity.y(),
                                                                      base_frame_angular_velocity.z()};
                        SendTwistCommand(base_frame_twist);
                    }
                    else
                    {
                        ROS_WARN_NAMED(ros::this_node::getName(), "Ignoring ee-frame Twist since latest state is not valid");
                    }
                }
                else if (twist_command.header.frame_id == base_frame_)
                {
                    SendTwistCommand(raw_twist);
                }
                else
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist frame: got [%s] needs [%s] or [%s]",
                                   twist_command.header.frame_id.c_str(), base_frame_.c_str(), ee_frame_.c_str());
                }
            }
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_minimal_hardware_interface");
    ROS_INFO_NAMED(ros::this_node::getName(), "Starting ur_minimal_hardware_interface...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_JOINT_STATE_TOPIC = "/ur10/joint_states";
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC = "/ur10/joint_command_velocity";
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
    const std::string joint_state_topic = nhp.param(std::string("joint_state_topic"), DEFAULT_JOINT_STATE_TOPIC);
    const std::string velocity_command_topic = nhp.param(std::string("velocity_command_topic"), DEFAULT_VELOCITY_COMMAND_TOPIC);
    const std::string twist_command_topic = nhp.param(std::string("twist_command_topic"), DEFAULT_TWIST_COMMAND_TOPIC);
    const std::string ee_pose_topic = nhp.param(std::string("ee_pose_topic"), DEFAULT_EE_POSE_TOPIC);
    const std::string ee_world_twist_topic = nhp.param(std::string("ee_world_twist_topic"), DEFAULT_EE_WORLD_TWIST_TOPIC);
    const std::string ee_body_twist_topic = nhp.param(std::string("ee_body_twist_topic"), DEFAULT_EE_BODY_TWIST_TOPIC);
    const std::string ee_wrench_topic = nhp.param(std::string("ee_wrench_topic"), DEFAULT_EE_WRENCH_TOPIC);
    const std::string base_frame = nhp.param(std::string("base_frame"), DEFAULT_BASE_FRAME);
    const std::string ee_frame = nhp.param(std::string("ee_frame"), DEFAULT_EE_FRAME);
    const std::string robot_hostname = nhp.param(std::string("robot_hostname"), DEFAULT_ROBOT_HOSTNAME);
    const double velocity_limit_scaling = std::abs(nhp.param(std::string("velocity_limit_scaling"), DEFAULT_VELOCITY_LIMIT_SCALING));
    const double acceleration_limit_scaling = std::abs(nhp.param(std::string("acceleration_limit_scaling"), DEFAULT_ACCELERATION_LIMIT_SCALING));
    const double real_velocity_limit_scaling = arc_helpers::ClampValueAndWarn(velocity_limit_scaling, 0.0, 1.0);
    const double real_acceleration_limit_scaling = arc_helpers::ClampValueAndWarn(acceleration_limit_scaling, 0.0, 1.0);
    // Joint names in true order
    const std::vector<std::string> ordered_joint_names = lightweight_ur_interface::GetOrderedJointNames();
    // Joint limits
    const std::map<std::string, lightweight_ur_interface::JointLimits> joint_limits = lightweight_ur_interface::GetLimits(real_velocity_limit_scaling, real_acceleration_limit_scaling);
    lightweight_ur_interface::URMinimalHardwareInterface interface(nh, velocity_command_topic, twist_command_topic, joint_state_topic, ee_pose_topic, ee_world_twist_topic, ee_body_twist_topic, ee_wrench_topic, base_frame, ee_frame, ordered_joint_names, joint_limits, robot_hostname);
    ROS_INFO_NAMED(ros::this_node::getName(), "...startup complete");
    interface.Run(400.0);
    return 0;
}
