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
        ros::Publisher ee_twist_pub_;
        ros::Publisher ee_wrench_pub_;
        ros::Subscriber velocity_command_sub_;
        ros::Subscriber twist_command_sub_;
        ros::Subscriber wrench_command_sub_;
        ros::ServiceServer switch_force_mode_server_;
        ros::ServiceServer switch_teach_mode_server_;

        std::atomic<bool> in_force_mode_;
        std::atomic<bool> in_teach_mode_;
        std::unique_ptr<URRealtimeInterface> robot_ptr_;
        std::mutex latest_state_mutex_;
        bool valid_latest_state_;
        sensor_msgs::JointState latest_joint_state_;
        std::vector<double> latest_raw_tcp_pose_;

    public:

        URMinimalHardwareInterface(ros::NodeHandle& nh,
                                   const std::string& velocity_command_topic,
                                   const std::string& twist_command_topic,
                                   const std::string& wrench_command_topic,
                                   const std::string& joint_state_topic,
                                   const std::string& ee_pose_topic,
                                   const std::string& ee_twist_topic,
                                   const std::string& ee_wrench_topic,
                                   const std::string& base_frame,
                                   const std::string& ee_frame,
                                   const std::string& force_mode_service,
                                   const std::string& teach_mode_service,
                                   const std::map<std::string, JointLimits>& joint_limits,
                                   const std::string& robot_host) : nh_(nh)
        {
            in_force_mode_.store(false);
            in_teach_mode_.store(false);
            valid_latest_state_ = false;
            joint_names_ = arc_helpers::GetKeys(joint_limits);
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
            ee_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(ee_twist_topic, 1, false);
            ee_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(ee_wrench_topic, 1, false);
            velocity_command_sub_ = nh_.subscribe(velocity_command_topic, 1, &URMinimalHardwareInterface::VelocityCommandCallback, this);
            twist_command_sub_ = nh_.subscribe(twist_command_topic, 1, &URMinimalHardwareInterface::TwistCommandCallback, this);
            wrench_command_sub_ = nh_.subscribe(wrench_command_topic, 1, &URMinimalHardwareInterface::WrenchCommandCallback, this);
            switch_force_mode_server_ = nh_.advertiseService(force_mode_service, &URMinimalHardwareInterface::SwitchForceModeCB, this);
            switch_teach_mode_server_ = nh_.advertiseService(teach_mode_service, &URMinimalHardwareInterface::SwitchTeachModeCB, this);
            // Build robot interface
            const std::function<void(const URRealtimeState&)> callback_fn = [&] (const URRealtimeState& latest_state) { return PublishState(joint_names_, ee_frame_, latest_state); };
            std::function<void(const std::string&)> logging_fn = [] (const std::string& message) { ROS_INFO_NAMED(ros::this_node::getName(), "%s", message.c_str()); };
            robot_ptr_ = std::unique_ptr<URRealtimeInterface>(new URRealtimeInterface(robot_host, callback_fn, logging_fn));
        }

        void Run()
        {
            // Start robot interface
            robot_ptr_->StartRecv();
            // Start ROS spinloop
            ros::Rate looprate(300.0);
            while (nh_.ok())
            {
                if (in_teach_mode_.load())
                {
                    const std::string cmd = "teach_mode()\n";
                    const bool success = robot_ptr_->SendURScriptCommand(cmd);
                    if (!success)
                    {
                        throw std::runtime_error("Failed to send tech mode command");
                    }
                }
                ros::spinOnce();
                looprate.sleep();
            }
            robot_ptr_->StopRecv();
        }

        void PublishState(const std::vector<std::string>& joint_names, const std::string& ee_frame_name, const URRealtimeState& robot_state)
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
            latest_raw_tcp_pose_ = robot_state.RawActualTcpPose();
            valid_latest_state_ = true;
            latest_state_mutex_.unlock();
            // EE transform
            geometry_msgs::PoseStamped ee_transform_msg = EigenHelpersConversions::EigenIsometry3dToGeometryPoseStamped(robot_state.ActualTcpPose(), base_frame_);
            ee_transform_msg.header.stamp = state_time;
            // EE twist
            const Eigen::Matrix<double, 6, 1>& ee_twist = robot_state.ActualTcpTwist();
            geometry_msgs::TwistStamped ee_twist_msg;
            ee_twist_msg.header.stamp = state_time;
            ee_twist_msg.header.frame_id = ee_frame_name;
            ee_twist_msg.twist.linear = EigenHelpersConversions::EigenVector3dToGeometryVector3(ee_twist.block<3, 1>(0, 0));
            ee_twist_msg.twist.angular = EigenHelpersConversions::EigenVector3dToGeometryVector3(ee_twist.block<3, 1>(3, 0));
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
            ee_twist_pub_.publish(ee_twist_msg);
            ee_wrench_pub_.publish(ee_wrench_msg);
        }

        bool SwitchForceModeCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
        {
            if (req.data)
            {
                if (in_force_mode_.load() == false)
                {
                    in_force_mode_.store(true);
                    res.success = true;
                    res.message = "Entered force mode";
                }
                else
                {
                    res.success = true;
                    res.message = "Already in force mode, ignoring request to enter force mode";
                }
            }
            else
            {
                if (in_force_mode_.load() == true)
                {
                    in_force_mode_.store(false);
                    res.success = true;
                    res.message = "Exited force mode";
                }
                else
                {
                    res.success = true;
                    res.message = "Not in force mode, ignoring request to exit force mode";
                }
            }
            return true;
        }

        bool SwitchTeachModeCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
        {
            if (req.data)
            {
                if (in_teach_mode_.load() == false)
                {
                    const std::string cmd = "teach_mode()\n";
                    const bool success = robot_ptr_->SendURScriptCommand(cmd);
                    if (success)
                    {
                        in_teach_mode_.store(true);
                        res.success = true;
                        res.message = "Entered teach mode";
                    }
                    else
                    {
                        res.success = false;
                        res.message = "Failed to enter teach mode";
                    }
                }
                else
                {
                    res.success = true;
                    res.message = "Already in teach mode, ignoring request to enter teach mode";
                }
            }
            else
            {
                if (in_teach_mode_.load() == true)
                {
                    const std::string cmd = "end_teach_mode()\n";
                    const bool success = robot_ptr_->SendURScriptCommand(cmd);
                    if (success)
                    {
                        in_teach_mode_.store(false);
                        res.success = true;
                        res.message = "Exited teach mode";
                    }
                    else
                    {
                        res.success = false;
                        res.message = "Failed to exit teach mode";
                    }
                }
                else
                {
                    res.success = true;
                    res.message = "Not in teach mode, ignoring request to exit teach mode";
                }
            }
            return true;
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
                    if (in_teach_mode_.load() == false)
                    {
                        SendVelocityCommand(target_velocity);
                    }
                    else
                    {
                        ROS_WARN_NAMED(ros::this_node::getName(), "Ignoring VelocityCommand since robot is in teach mode");
                    }
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
            int written = 0;
            if (in_force_mode_.load())
            {
                written = snprintf(command_str_buffer, max_str_len - 1, "force_mode(get_actual_tcp_pose(), [1, 1, 1, 1, 1, 1], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [10.0, 10.0, 10.0, 10.0, 10.0, 10.0])\nspeedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n", command[0], command[1], command[2], command[3], command[4], command[5], max_command_acceleration);
            }
            else
            {
                written = snprintf(command_str_buffer, max_str_len - 1, "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n", command[0], command[1], command[2], command[3], command[4], command[5], max_command_acceleration);
            }
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
            if (twist_command.header.frame_id == ee_frame_)
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
                    if (in_teach_mode_.load() == false)
                    {
                        SendTwistCommand(raw_twist);
                    }
                    else
                    {
                        ROS_WARN_NAMED(ros::this_node::getName(), "Ignoring Twist since robot is in teach mode");
                    }
                }
            }
            else
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Twist frame: got [%s] needs [%s]", twist_command.header.frame_id.c_str(), ee_frame_.c_str());
            }
        }

        void SendWrenchCommand(const std::vector<double>& wrench)
        {
            const int max_str_len = 1024;
            char command_str_buffer[max_str_len];
            const int written = snprintf(command_str_buffer, max_str_len - 1, "force_mode(get_actual_tcp_pose(), [1, 1, 1, 1, 1, 1], [%5.5f, %5.5f, %5.5f, %5.5f, %5.5f, %5.5f], 2, [10.0, 10.0, 10.0, 10.0, 10.0, 10.0])\n", wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);
            assert(written > 0);
            assert(written < max_str_len);
            const std::string command_str(command_str_buffer);
            const bool success = robot_ptr_->SendURScriptCommand(command_str);
            if (!success)
            {
                throw std::runtime_error("Failed to send force_mode(...) command");
            }
        }

        void WrenchCommandCallback(geometry_msgs::WrenchStamped wrench_command)
        {
            if (wrench_command.header.frame_id == ee_frame_)
            {
                bool valid_wrench = true;
                if (std::isinf(wrench_command.wrench.force.x) || std::isnan(wrench_command.wrench.force.x))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Wrench command, force.x is NAN or INF");
                    valid_wrench = false;
                }
                if (std::isinf(wrench_command.wrench.force.y) || std::isnan(wrench_command.wrench.force.y))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Wrench command, force.y is NAN or INF");
                    valid_wrench = false;
                }
                if (std::isinf(wrench_command.wrench.force.z) || std::isnan(wrench_command.wrench.force.z))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Wrench command, force.z is NAN or INF");
                    valid_wrench = false;
                }
                if (std::isinf(wrench_command.wrench.torque.x) || std::isnan(wrench_command.wrench.torque.x))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Wrench command, torque.x is NAN or INF");
                    valid_wrench = false;
                }
                if (std::isinf(wrench_command.wrench.torque.y) || std::isnan(wrench_command.wrench.torque.y))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Wrench command, torque.y is NAN or INF");
                    valid_wrench = false;
                }
                if (std::isinf(wrench_command.wrench.torque.z) || std::isnan(wrench_command.wrench.torque.z))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Wrench command, torque.z is NAN or INF");
                    valid_wrench = false;
                }
                if (valid_wrench)
                {
                    const std::vector<double> raw_wrench = {wrench_command.wrench.force.x, wrench_command.wrench.force.y, wrench_command.wrench.force.z, wrench_command.wrench.torque.x, wrench_command.wrench.torque.y, wrench_command.wrench.torque.z};
                    if (in_teach_mode_.load() == false)
                    {
                        SendWrenchCommand(raw_wrench);
                    }
                    else
                    {
                        ROS_WARN_NAMED(ros::this_node::getName(), "Ignoring Wrench since robot is in teach mode");
                    }
                }
            }
            else
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid Wrench frame: got [%s] needs [%s]", wrench_command.header.frame_id.c_str(), ee_frame_.c_str());
            }
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_velocity_interface");
    ROS_INFO_NAMED(ros::this_node::getName(), "Starting ur_velocity_interface...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_JOINT_STATE_TOPIC = "/ur10/joint_states";
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC = "/ur10/joint_command_velocity";
    const std::string DEFAULT_TWIST_COMMAND_TOPIC = "/ur10/ee_twist_command";
    const std::string DEFAULT_WRENCH_COMMAND_TOPIC = "/ur10/ee_wrench_command";
    const std::string DEFAULT_EE_POSE_TOPIC = "/ur10/ee_pose";
    const std::string DEFAULT_EE_TWIST_TOPIC = "/ur10/ee_twist";
    const std::string DEFAULT_EE_WRENCH_TOPIC = "/ur10/ee_wrench";
    const std::string DEFAULT_BASE_FRAME = "base";
    const std::string DEFAULT_EE_WRENCH_FRAME = "ur10_ee_ft_frame";
    const std::string DEFAULT_FORCE_MODE_SERVICE = "/ur10/switch_force_mode";
    const std::string DEFAULT_TEACH_MODE_SERVICE = "/ur10/switch_teach_mode";
    const std::string DEFAULT_ROBOT_HOSTNAME = "172.31.1.200";
    const double DEFAULT_VELOCITY_LIMIT_SCALING = 0.25;
    const double DEFAULT_ACCELERATION_LIMIT_SCALING = 0.25;
    const std::string joint_state_topic = nhp.param(std::string("joint_state_topic"), DEFAULT_JOINT_STATE_TOPIC);
    const std::string velocity_command_topic = nhp.param(std::string("velocity_command_topic"), DEFAULT_VELOCITY_COMMAND_TOPIC);
    const std::string twist_command_topic = nhp.param(std::string("twist_command_topic"), DEFAULT_TWIST_COMMAND_TOPIC);
    const std::string wrench_command_topic = nhp.param(std::string("wrench_command_topic"), DEFAULT_WRENCH_COMMAND_TOPIC);
    const std::string ee_pose_topic = nhp.param(std::string("ee_pose_topic"), DEFAULT_EE_POSE_TOPIC);
    const std::string ee_twist_topic = nhp.param(std::string("ee_twist_topic"), DEFAULT_EE_TWIST_TOPIC);
    const std::string ee_wrench_topic = nhp.param(std::string("ee_wrench_topic"), DEFAULT_EE_WRENCH_TOPIC);
    const std::string base_frame = nhp.param(std::string("base_frame"), DEFAULT_BASE_FRAME);
    const std::string ee_wrench_frame = nhp.param(std::string("ee_wrench_frame"), DEFAULT_EE_WRENCH_FRAME);
    const std::string force_mode_service = nhp.param(std::string("force_mode_service"), DEFAULT_FORCE_MODE_SERVICE);
    const std::string teach_mode_service = nhp.param(std::string("teach_mode_service"), DEFAULT_TEACH_MODE_SERVICE);
    const std::string robot_hostname = nhp.param(std::string("robot_hostname"), DEFAULT_ROBOT_HOSTNAME);
    const double velocity_limit_scaling = std::abs(nhp.param(std::string("velocity_limit_scaling"), DEFAULT_VELOCITY_LIMIT_SCALING));
    const double acceleration_limit_scaling = std::abs(nhp.param(std::string("acceleration_limit_scaling"), DEFAULT_ACCELERATION_LIMIT_SCALING));
    const double real_velocity_limit_scaling = arc_helpers::ClampValueAndWarn(velocity_limit_scaling, 0.0, 1.0);
    const double real_acceleration_limit_scaling = arc_helpers::ClampValueAndWarn(acceleration_limit_scaling, 0.0, 1.0);
    // Joint limits
    const std::map<std::string, lightweight_ur_interface::JointLimits> joint_limits = lightweight_ur_interface::GetLimits(real_velocity_limit_scaling, real_acceleration_limit_scaling);
    lightweight_ur_interface::URMinimalHardwareInterface interface(nh, velocity_command_topic, twist_command_topic, wrench_command_topic, joint_state_topic, ee_pose_topic, ee_twist_topic, ee_wrench_topic, base_frame, ee_wrench_frame, force_mode_service, teach_mode_service, joint_limits, robot_hostname);
    ROS_INFO_NAMED(ros::this_node::getName(), "...startup complete");
    interface.Run();
    return 0;
}
