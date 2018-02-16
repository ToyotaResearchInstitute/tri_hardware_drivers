#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <lightweight_ur_interface/ur_robot_config.hpp>
#include <lightweight_ur_interface/WrenchCommand.h>

namespace lightweight_ur_interface
{
    class URCartesianImpedanceController
    {
    protected:

        typedef Eigen::Matrix<double, 6, 1> Twist;

        std::vector<PIDParams> axis_velocity_controller_parameters_;
        std::vector<PIDParams> axis_force_controller_parameters_;
        std::vector<double> axis_velocity_limits_;
        std::vector<double> axis_effort_limits_;
        std::string pose_frame_;
        std::string wrench_frame_;

        bool target_pose_valid_;
        bool current_pose_valid_;
        Eigen::Isometry3d target_pose_;
        Eigen::Isometry3d current_pose_;

        Twist pose_error_integral_;
        Twist last_pose_error_;

        ros::NodeHandle nh_;
        ros::Publisher command_pub_;
        ros::Subscriber feedback_sub_;
        ros::Subscriber pose_target_sub_;
        ros::ServiceServer abort_server_;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        URCartesianImpedanceController(ros::NodeHandle& nh,
                                       const std::string& pose_frame,
                                       const std::string& wrench_frame,
                                       const std::string& target_pose_topic,
                                       const std::string& pose_feedback_topic,
                                       const std::string& wrench_command_topic,
                                       const std::string& abort_service,
                                       const std::vector<PIDParams>& axis_velocity_controller_parameters,
                                       const std::vector<PIDParams>& axis_force_controller_parameters,
                                       const std::vector<double>& axis_velocity_limits,
                                       const std::vector<double>& axis_effort_limits) : nh_(nh)
        {
            pose_error_integral_ = Twist::Zero();
            last_pose_error_ = Twist::Zero();
            target_pose_valid_ = false;
            current_pose_valid_ = false;
            target_pose_ = Eigen::Isometry3d::Identity();
            current_pose_ = Eigen::Isometry3d::Identity();
            if (axis_velocity_controller_parameters.size() != 6)
            {
                throw std::invalid_argument("Number of provided axis velocity controller parameters != 6");
            }
            axis_velocity_controller_parameters_ = axis_velocity_controller_parameters;
            if (axis_force_controller_parameters.size() != 6)
            {
                throw std::invalid_argument("Number of provided axis force controller parameters != 6");
            }
            axis_force_controller_parameters_ = axis_force_controller_parameters;
            if (axis_velocity_limits.size() != 6)
            {
                throw std::invalid_argument("Number of provided axis velocity limits != 6");
            }
            axis_velocity_limits_ = axis_velocity_limits;
            if (axis_effort_limits.size() != 6)
            {
                throw std::invalid_argument("Number of provided axis effort limits != 6");
            }
            axis_effort_limits_ = axis_effort_limits;
            pose_frame_ = pose_frame;
            wrench_frame_ = wrench_frame;
            command_pub_ = nh_.advertise<lightweight_ur_interface::WrenchCommand>(wrench_command_topic, 1, false);
            feedback_sub_ = nh_.subscribe(pose_feedback_topic, 1, &URCartesianImpedanceController::PoseFeedbackCallback, this);
            pose_target_sub_ = nh_.subscribe(target_pose_topic, 1, &URCartesianImpedanceController::PoseTargetCallback, this);
            abort_server_ = nh_.advertiseService(abort_service, &URCartesianImpedanceController::AbortCB, this);
        }

        void Loop(const double control_rate)
        {
            const double control_interval = 1.0 / control_rate;
            ros::Rate spin_rate(control_rate);
            const uint8_t supersample_rate = 0x01;
            uint8_t iteration_count = 0x01;
            while (ros::ok())
            {
                // Process callbacks
                ros::spinOnce();
                // Run controller
                if (iteration_count == supersample_rate)
                {
                    if (target_pose_valid_ && current_pose_valid_)
                    {
                        std::cout << "Current pose: " << PrettyPrint::PrettyPrint(current_pose_) << std::endl;
                        std::cout << "Target pose: " << PrettyPrint::PrettyPrint(target_pose_) << std::endl;
                        const Twist raw_pose_velocity_correction = ComputeRawNextStep(current_pose_, target_pose_, control_interval, axis_velocity_controller_parameters_);
                        const Twist raw_pose_wrench_correction = ComputeRawNextStep(current_pose_, target_pose_, control_interval, axis_force_controller_parameters_);
                        std::cout << "Raw pose velocity correction: " << PrettyPrint::PrettyPrint(raw_pose_velocity_correction) << std::endl;
                        std::cout << "Raw pose wrench correction: " << PrettyPrint::PrettyPrint(raw_pose_wrench_correction) << std::endl;
                        const Twist real_pose_velocity_correction = LimitCorrection(raw_pose_velocity_correction, axis_velocity_limits_);
                        const Twist real_pose_wrench_correction = LimitCorrection(raw_pose_wrench_correction, axis_effort_limits_);
                        std::cout << "Commanded ee_velocity correction: " << PrettyPrint::PrettyPrint(real_pose_velocity_correction) << std::endl;
                        std::cout << "Commanded applied_wrench correction: " << PrettyPrint::PrettyPrint(real_pose_wrench_correction) << std::endl;
                        CommandWrench(real_pose_velocity_correction, real_pose_wrench_correction, wrench_frame_);
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

        inline bool AbortCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
        {
            UNUSED(req);
            UNUSED(res);
            ROS_INFO_NAMED(ros::this_node::getName(), "Aborting pose target");
            target_pose_valid_ = false;
            pose_error_integral_ = Twist::Zero();
            last_pose_error_ = Twist::Zero();
            return true;
        }

        inline Twist ComputeNewPoseErrorIntegral(const Twist& raw_pose_error_integral, const std::vector<PIDParams>& axis_controller_parameters) const
        {
            Twist limited_pose_error_integral;
            limited_pose_error_integral(0) = arc_helpers::ClampValue(raw_pose_error_integral(0), -axis_controller_parameters[0].Iclamp(), axis_controller_parameters[0].Iclamp());
            limited_pose_error_integral(1) = arc_helpers::ClampValue(raw_pose_error_integral(1), -axis_controller_parameters[1].Iclamp(), axis_controller_parameters[1].Iclamp());
            limited_pose_error_integral(2) = arc_helpers::ClampValue(raw_pose_error_integral(2), -axis_controller_parameters[2].Iclamp(), axis_controller_parameters[2].Iclamp());
            limited_pose_error_integral(3) = arc_helpers::ClampValue(raw_pose_error_integral(3), -axis_controller_parameters[3].Iclamp(), axis_controller_parameters[3].Iclamp());
            limited_pose_error_integral(4) = arc_helpers::ClampValue(raw_pose_error_integral(4), -axis_controller_parameters[4].Iclamp(), axis_controller_parameters[4].Iclamp());
            limited_pose_error_integral(5) = arc_helpers::ClampValue(raw_pose_error_integral(5), -axis_controller_parameters[5].Iclamp(), axis_controller_parameters[5].Iclamp());
            return limited_pose_error_integral;
        }

        inline Twist ComputeRawNextStep(const Eigen::Isometry3d& current_pose, const Eigen::Isometry3d& target_pose, const double time_interval, const std::vector<PIDParams>& axis_controller_parameters)
        {
            // Compute the pose error in our 'world frame'
            const Twist pose_error = EigenHelpers::TwistBetweenTransforms(current_pose, target_pose);
            // Compute the integral of pose error & update the stored value
            pose_error_integral_ = ComputeNewPoseErrorIntegral(pose_error_integral_ + (pose_error * time_interval), axis_controller_parameters);
            // Compute the derivative of pose error
            const Twist pose_error_derivative = (pose_error - last_pose_error_) / time_interval;
            // Update the stored pose error
            last_pose_error_ = pose_error;
            // Convert pose errors into cartesian velocity
            Twist raw_pose_correction;
            raw_pose_correction(0) = (pose_error(0) * axis_controller_parameters[0].Kp())
                    + (pose_error_integral_(0) * axis_controller_parameters[0].Ki())
                    + (pose_error_derivative(0) * axis_controller_parameters[0].Kd());
            raw_pose_correction(1) = (pose_error(1) * axis_controller_parameters[1].Kp())
                    + (pose_error_integral_(1) * axis_controller_parameters[1].Ki())
                    + (pose_error_derivative(1) * axis_controller_parameters[1].Kd());
            raw_pose_correction(2) = (pose_error(2) * axis_controller_parameters[2].Kp())
                    + (pose_error_integral_(2) * axis_controller_parameters[2].Ki())
                    + (pose_error_derivative(2) * axis_controller_parameters[2].Kd());
            raw_pose_correction(3) = (pose_error(3) * axis_controller_parameters[3].Kp())
                    + (pose_error_integral_(3) * axis_controller_parameters[3].Ki())
                    + (pose_error_derivative(3) * axis_controller_parameters[3].Kd());
            raw_pose_correction(4) = (pose_error(4) * axis_controller_parameters[4].Kp())
                    + (pose_error_integral_(4) * axis_controller_parameters[4].Ki())
                    + (pose_error_derivative(4) * axis_controller_parameters[4].Kd());
            raw_pose_correction(5) = (pose_error(5) * axis_controller_parameters[5].Kp())
                    + (pose_error_integral_(5) * axis_controller_parameters[5].Ki())
                    + (pose_error_derivative(5) * axis_controller_parameters[5].Kd());
            return raw_pose_correction;
        }

        inline Twist LimitCorrection(const Twist& raw_correction, const std::vector<double>& axis_limits) const
        {
            Twist limited_correction;
            limited_correction(0) = arc_helpers::ClampValue(raw_correction(0), -axis_limits[0], axis_limits[0]);
            limited_correction(1) = arc_helpers::ClampValue(raw_correction(1), -axis_limits[1], axis_limits[1]);
            limited_correction(2) = arc_helpers::ClampValue(raw_correction(2), -axis_limits[2], axis_limits[2]);
            limited_correction(3) = arc_helpers::ClampValue(raw_correction(3), -axis_limits[3], axis_limits[3]);
            limited_correction(4) = arc_helpers::ClampValue(raw_correction(4), -axis_limits[4], axis_limits[4]);
            limited_correction(5) = arc_helpers::ClampValue(raw_correction(5), -axis_limits[5], axis_limits[5]);
            return limited_correction;
        }

        inline void CommandWrench(const Twist& ee_velocity, const Twist& applied_wrench, const std::string& frame)
        {
            lightweight_ur_interface::WrenchCommand command_msg;
            command_msg.header.frame_id = frame;
            command_msg.header.stamp = ros::Time::now();
            command_msg.ee_velocities.linear.x = ee_velocity(0, 0);
            command_msg.ee_velocities.linear.y = ee_velocity(1, 0);
            command_msg.ee_velocities.linear.z = ee_velocity(2, 0);
            command_msg.ee_velocities.angular.x = ee_velocity(3, 0);
            command_msg.ee_velocities.angular.y = ee_velocity(4, 0);
            command_msg.ee_velocities.angular.z = ee_velocity(5, 0);
            command_msg.applied_wrench.force.x = applied_wrench(0, 0);
            command_msg.applied_wrench.force.y = applied_wrench(1, 0);
            command_msg.applied_wrench.force.z = applied_wrench(2, 0);
            command_msg.applied_wrench.torque.x = applied_wrench(3, 0);
            command_msg.applied_wrench.torque.y = applied_wrench(4, 0);
            command_msg.applied_wrench.torque.z = applied_wrench(5, 0);
            command_pub_.publish(command_msg);
        }

        inline void PoseTargetCallback(geometry_msgs::PoseStamped target_pose)
        {
            if (target_pose.header.frame_id == pose_frame_)
            {
                ROS_INFO_NAMED(ros::this_node::getName(), "Starting execution to a new target pose");
                target_pose_ = EigenHelpersConversions::GeometryPoseToEigenIsometry3d(target_pose.pose);
                target_pose_valid_ = true;
            }
            else
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid target pose frame %s, should be %s", target_pose.header.frame_id.c_str(), pose_frame_.c_str());
                target_pose_valid_ = false;
            }
        }

        inline void PoseFeedbackCallback(geometry_msgs::PoseStamped pose_feedback)
        {
            if (pose_feedback.header.frame_id == pose_frame_)
            {
                current_pose_ = EigenHelpersConversions::GeometryPoseToEigenIsometry3d(pose_feedback.pose);
                current_pose_valid_ = true;
            }
            else
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid feedback pose frame %s, should be %s", pose_feedback.header.frame_id.c_str(), pose_frame_.c_str());
                current_pose_valid_ = false;
            }
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_cartesian_impedance_controller");
    ROS_INFO_NAMED(ros::this_node::getName(), "Starting ur_cartesian_impedance_controller...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_POSE_FRAME = "base";
    const std::string DEFAULT_WRENCH_FRAME = "ur10_ee_frame";
    const std::string DEFAULT_TARGET_POSE_TOPIC = "/ur10/target_cartesian_pose";
    const std::string DEFAULT_POSE_FEEDBACK_TOPIC = "/ur10/ee_pose";
    const std::string DEFAULT_WRENCH_COMMAND_TOPIC = "/ur10/ee_wrench_command";
    const std::string DEFAULT_ABORT_SERVICE = "/ur10_cartesian_impedance_controller/abort";
    const double DEFAULT_TRANSLATION_KP = 1.0;
    const double DEFAULT_ROTATION_KP = 1.0;
    const double DEFAULT_TRANSLATION_KD = 0.1;
    const double DEFAULT_ROTATION_KD = 0.1;
    const double DEFAULT_MAX_LINEAR_VELOCITY = 0.5;
    const double DEFAULT_MAX_ANGULAR_VELOCITY = 1.0;
    const double DEFAULT_TRANSLATION_STIFFNESS = 10.0;
    const double DEFAULT_ROTATION_STIFFNESS = 1.0;
    const double DEFAULT_TRANSLATION_DAMPING = 0.1;
    const double DEFAULT_ROTATION_DAMPING = 0.1;
    const double DEFAULT_CONTROL_RATE = 150.0;
    const double DEFAULT_MAX_FORCE = 50.0;
    const double DEFAULT_MAX_TORQUE = 5.0;
    const std::string pose_frame = nhp.param(std::string("pose_frame"), DEFAULT_POSE_FRAME);
    const std::string wrench_frame = nhp.param(std::string("wrench_frame"), DEFAULT_WRENCH_FRAME);
    const std::string target_pose_topic = nhp.param(std::string("target_pose_topic"), DEFAULT_TARGET_POSE_TOPIC);
    const std::string pose_feedback_topic = nhp.param(std::string("pose_feedback_topic"), DEFAULT_POSE_FEEDBACK_TOPIC);
    const std::string wrench_command_topic = nhp.param(std::string("wrench_command_topic"), DEFAULT_WRENCH_COMMAND_TOPIC);
    const std::string abort_service = nhp.param(std::string("abort_service"), DEFAULT_ABORT_SERVICE);
    const double translation_kp = std::abs(nhp.param(std::string("translation_kp"), DEFAULT_TRANSLATION_KP));
    const double rotation_kp = std::abs(nhp.param(std::string("rotation_kp"), DEFAULT_ROTATION_KP));
    const double translation_kd = std::abs(nhp.param(std::string("translation_kd"), DEFAULT_TRANSLATION_KD));
    const double rotation_kd = std::abs(nhp.param(std::string("rotation_kd"), DEFAULT_ROTATION_KD));
    const double max_linear_velocity = std::abs(nhp.param(std::string("max_linear_velocity"), DEFAULT_MAX_LINEAR_VELOCITY));
    const double max_angular_velocity = std::abs(nhp.param(std::string("max_angular_velocity"), DEFAULT_MAX_ANGULAR_VELOCITY));
    const double translation_stiffness = std::abs(nhp.param(std::string("translation_stiffness"), DEFAULT_TRANSLATION_STIFFNESS));
    const double rotation_stiffness = std::abs(nhp.param(std::string("rotation_stiffness"), DEFAULT_ROTATION_STIFFNESS));
    const double translation_damping = std::abs(nhp.param(std::string("translation_damping"), DEFAULT_TRANSLATION_DAMPING));
    const double rotation_damping = std::abs(nhp.param(std::string("rotation_damping"), DEFAULT_ROTATION_DAMPING));
    const double control_rate = std::abs(nhp.param(std::string("control_rate"), DEFAULT_CONTROL_RATE));
    const double max_force = std::abs(nhp.param(std::string("max_force"), DEFAULT_MAX_FORCE));
    const double max_torque = std::abs(nhp.param(std::string("max_torque"), DEFAULT_MAX_TORQUE));
    const std::vector<lightweight_ur_interface::PIDParams> axis_velocity_controller_params = lightweight_ur_interface::GetDefaultPoseControllerParams(translation_kp, translation_kd, rotation_kp, rotation_kd);
    const std::vector<double> axis_velocity_limits = {max_linear_velocity, max_linear_velocity, max_linear_velocity, max_angular_velocity, max_angular_velocity, max_angular_velocity};
    const std::vector<lightweight_ur_interface::PIDParams> axis_force_controller_params = lightweight_ur_interface::GetDefaultPoseControllerParams(translation_stiffness, translation_damping, rotation_stiffness, rotation_damping);
    const std::vector<double> axis_effort_limits = {max_force, max_force, max_force, max_torque, max_torque, max_torque};
    lightweight_ur_interface::URCartesianImpedanceController controller(nh,
                                                                        pose_frame,
                                                                        wrench_frame,
                                                                        target_pose_topic,
                                                                        pose_feedback_topic,
                                                                        wrench_command_topic,
                                                                        abort_service,
                                                                        axis_velocity_controller_params,
                                                                        axis_force_controller_params,
                                                                        axis_velocity_limits,
                                                                        axis_effort_limits);
    ROS_INFO_NAMED(ros::this_node::getName(), "...startup complete");
    controller.Loop(control_rate);
    return 0;
}
