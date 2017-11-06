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
#include <geometry_msgs/WrenchStamped.h>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <lightweight_ur_interface/ur_robot_config.hpp>

namespace lightweight_ur_interface
{
    class URCartesianImpedanceController
    {
    protected:

        typedef Eigen::Matrix<double, 6, 1> Twist;

        std::vector<PIDParams> axis_controller_parameters_;
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
                                       const std::vector<PIDParams>& axis_controller_parameters,
                                       const std::vector<double>& axis_effort_limits) : nh_(nh)
        {
            pose_error_integral_ = Twist::Zero();
            last_pose_error_ = Twist::Zero();
            target_pose_valid_ = false;
            current_pose_valid_ = false;
            target_pose_ = Eigen::Isometry3d::Identity();
            current_pose_ = Eigen::Isometry3d::Identity();
            if (axis_controller_parameters.size() != 6)
            {
                throw std::invalid_argument("Number of provided axis controller parameters != 6");
            }
            axis_controller_parameters_ = axis_controller_parameters;
            if (axis_effort_limits.size() != 6)
            {
                throw std::invalid_argument("Number of provided axis effort limits != 6");
            }
            axis_effort_limits_ = axis_effort_limits;
            pose_frame_ = pose_frame;
            wrench_frame_ = wrench_frame;
            command_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(wrench_command_topic, 1, false);
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
                        //std::cout << "Current pose: " << PrettyPrint::PrettyPrint(current_pose_) << std::endl;
                        //std::cout << "Target pose: " << PrettyPrint::PrettyPrint(target_pose_) << std::endl;
                        const Twist raw_pose_correction = ComputeRawNextStep(current_pose_, target_pose_, control_interval);
                        //std::cout << "Raw pose correction: " << PrettyPrint::PrettyPrint(raw_pose_correction) << std::endl;
                        const Twist real_pose_correction = LimitCorrectionWrench(raw_pose_correction);
                        //std::cout << "Commanded velocity: " << PrettyPrint::PrettyPrint(real_pose_correction) << std::endl;
                        CommandWrench(real_pose_correction, wrench_frame_);
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

        inline Twist ComputeNewPoseErrorIntegral(const Twist& raw_pose_error_integral) const
        {
            Twist limited_pose_error_integral;
            limited_pose_error_integral(0) = arc_helpers::ClampValue(raw_pose_error_integral(0), -axis_controller_parameters_[0].Iclamp(), axis_controller_parameters_[0].Iclamp());
            limited_pose_error_integral(1) = arc_helpers::ClampValue(raw_pose_error_integral(1), -axis_controller_parameters_[1].Iclamp(), axis_controller_parameters_[1].Iclamp());
            limited_pose_error_integral(2) = arc_helpers::ClampValue(raw_pose_error_integral(2), -axis_controller_parameters_[2].Iclamp(), axis_controller_parameters_[2].Iclamp());
            limited_pose_error_integral(3) = arc_helpers::ClampValue(raw_pose_error_integral(3), -axis_controller_parameters_[3].Iclamp(), axis_controller_parameters_[3].Iclamp());
            limited_pose_error_integral(4) = arc_helpers::ClampValue(raw_pose_error_integral(4), -axis_controller_parameters_[4].Iclamp(), axis_controller_parameters_[4].Iclamp());
            limited_pose_error_integral(5) = arc_helpers::ClampValue(raw_pose_error_integral(5), -axis_controller_parameters_[5].Iclamp(), axis_controller_parameters_[5].Iclamp());
            return limited_pose_error_integral;
        }

        inline Twist ComputeRawNextStep(const Eigen::Isometry3d& current_pose, const Eigen::Isometry3d& target_pose, const double time_interval)
        {
            // Compute the pose error in our 'world frame'
            const Twist pose_error = EigenHelpers::TwistBetweenTransforms(current_pose, target_pose);
            // Compute the integral of pose error & update the stored value
            pose_error_integral_ = ComputeNewPoseErrorIntegral(pose_error_integral_ + (pose_error * time_interval));
            // Compute the derivative of pose error
            const Twist pose_error_derivative = (pose_error - last_pose_error_) / time_interval;
            // Update the stored pose error
            last_pose_error_ = pose_error;
            // Convert pose errors into cartesian velocity
            Twist raw_pose_correction;
            raw_pose_correction(0) = (pose_error(0) * axis_controller_parameters_[0].Kp())
                    + (pose_error_integral_(0) * axis_controller_parameters_[0].Ki())
                    + (pose_error_derivative(0) * axis_controller_parameters_[0].Kd());
            raw_pose_correction(1) = (pose_error(1) * axis_controller_parameters_[1].Kp())
                    + (pose_error_integral_(1) * axis_controller_parameters_[1].Ki())
                    + (pose_error_derivative(1) * axis_controller_parameters_[1].Kd());
            raw_pose_correction(2) = (pose_error(2) * axis_controller_parameters_[2].Kp())
                    + (pose_error_integral_(2) * axis_controller_parameters_[2].Ki())
                    + (pose_error_derivative(2) * axis_controller_parameters_[2].Kd());
            raw_pose_correction(3) = (pose_error(3) * axis_controller_parameters_[3].Kp())
                    + (pose_error_integral_(3) * axis_controller_parameters_[3].Ki())
                    + (pose_error_derivative(3) * axis_controller_parameters_[3].Kd());
            raw_pose_correction(4) = (pose_error(4) * axis_controller_parameters_[4].Kp())
                    + (pose_error_integral_(4) * axis_controller_parameters_[4].Ki())
                    + (pose_error_derivative(4) * axis_controller_parameters_[4].Kd());
            raw_pose_correction(5) = (pose_error(5) * axis_controller_parameters_[5].Kp())
                    + (pose_error_integral_(5) * axis_controller_parameters_[5].Ki())
                    + (pose_error_derivative(5) * axis_controller_parameters_[5].Kd());
            return raw_pose_correction;
        }

        inline Twist LimitCorrectionWrench(const Twist& raw_wrench) const
        {
            Twist limited_wrench;
            limited_wrench(0) = arc_helpers::ClampValue(raw_wrench(0), -axis_effort_limits_[0], axis_effort_limits_[0]);
            limited_wrench(1) = arc_helpers::ClampValue(raw_wrench(1), -axis_effort_limits_[1], axis_effort_limits_[1]);
            limited_wrench(2) = arc_helpers::ClampValue(raw_wrench(2), -axis_effort_limits_[2], axis_effort_limits_[2]);
            limited_wrench(3) = arc_helpers::ClampValue(raw_wrench(3), -axis_effort_limits_[3], axis_effort_limits_[3]);
            limited_wrench(4) = arc_helpers::ClampValue(raw_wrench(4), -axis_effort_limits_[4], axis_effort_limits_[4]);
            limited_wrench(5) = arc_helpers::ClampValue(raw_wrench(5), -axis_effort_limits_[5], axis_effort_limits_[5]);
            return limited_wrench;
        }

        inline void CommandWrench(const Twist& command, const std::string& frame)
        {
            geometry_msgs::WrenchStamped command_msg;
            command_msg.header.frame_id = frame;
            command_msg.header.stamp = ros::Time::now();
            command_msg.wrench.force.x = command(0, 0);
            command_msg.wrench.force.y = command(1, 0);
            command_msg.wrench.force.z = command(2, 0);
            command_msg.wrench.torque.x = command(3, 0);
            command_msg.wrench.torque.y = command(4, 0);
            command_msg.wrench.torque.z = command(5, 0);
            command_pub_.publish(command_msg);
        }

        inline void PoseTargetCallback(geometry_msgs::PoseStamped target_pose)
        {
            if (target_pose.header.frame_id == pose_frame_)
            {
                target_pose_ = EigenHelpersConversions::GeometryPoseToEigenIsometry3d(target_pose.pose);
                target_pose_valid_ = true;
            }
            else
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "Invalid target pose frame %s, should be %s", target_pose.header.frame_id.c_str(), pose_frame_.c_str());
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
    const std::string DEFAULT_WRENCH_FRAME = "ur10_ee_ft_frame";
    const std::string DEFAULT_TARGET_POSE_TOPIC = "/ur10/target_cartesian_pose";
    const std::string DEFAULT_POSE_FEEDBACK_TOPIC = "/ur10/ee_pose";
    const std::string DEFAULT_WRENCH_COMMAND_TOPIC = "/ur10/ee_wrench_command";
    const std::string DEFAULT_ABORT_SERVICE = "/ur10_cartesian_impedance_controller/abort";
    const double DEFAULT_TRANSLATION_STIFFNESS = 10000.0;
    const double DEFAULT_ROTATION_STIFFNESS = 1000.0;
    const double DEFAULT_TRANSLATION_DAMPING = 0.1;
    const double DEFAULT_ROTATION_DAMPING = 0.1;
    const double DEFAULT_CONTROL_RATE = 150.0;
    const double DEFAULT_MAX_FORCE = 500.0;
    const double DEFAULT_MAX_TORQUE = 50.0;
    const std::string pose_frame = nhp.param(std::string("pose_frame"), DEFAULT_POSE_FRAME);
    const std::string wrench_frame = nhp.param(std::string("wrench_frame"), DEFAULT_WRENCH_FRAME);
    const std::string target_pose_topic = nhp.param(std::string("target_pose_topic"), DEFAULT_TARGET_POSE_TOPIC);
    const std::string pose_feedback_topic = nhp.param(std::string("pose_feedback_topic"), DEFAULT_POSE_FEEDBACK_TOPIC);
    const std::string wrench_command_topic = nhp.param(std::string("wrench_command_topic"), DEFAULT_WRENCH_COMMAND_TOPIC);
    const std::string abort_service = nhp.param(std::string("abort_service"), DEFAULT_ABORT_SERVICE);
    const double translation_stiffness = std::abs(nhp.param(std::string("translation_stiffness"), DEFAULT_TRANSLATION_STIFFNESS));
    const double rotation_stiffness = std::abs(nhp.param(std::string("rotation_stiffness"), DEFAULT_ROTATION_STIFFNESS));
    const double translation_damping = std::abs(nhp.param(std::string("translation_damping"), DEFAULT_TRANSLATION_DAMPING));
    const double rotation_damping = std::abs(nhp.param(std::string("rotation_damping"), DEFAULT_ROTATION_DAMPING));
    const double control_rate = std::abs(nhp.param(std::string("control_rate"), DEFAULT_CONTROL_RATE));
    const double max_force = std::abs(nhp.param(std::string("max_force"), DEFAULT_MAX_FORCE));
    const double max_torque = std::abs(nhp.param(std::string("max_torque"), DEFAULT_MAX_TORQUE));
    const std::vector<lightweight_ur_interface::PIDParams> axis_controller_params = lightweight_ur_interface::GetDefaultPoseControllerParams(translation_stiffness, translation_damping, rotation_stiffness, rotation_damping);
    const std::vector<double> axis_effort_limits = {max_force, max_force, max_force, max_torque, max_torque, max_torque};
    lightweight_ur_interface::URCartesianImpedanceController controller(nh, pose_frame, wrench_frame, target_pose_topic, pose_feedback_topic, wrench_command_topic, abort_service, axis_controller_params, axis_effort_limits);
    ROS_INFO_NAMED(ros::this_node::getName(), "...startup complete");
    controller.Loop(control_rate);
    return 0;
}
