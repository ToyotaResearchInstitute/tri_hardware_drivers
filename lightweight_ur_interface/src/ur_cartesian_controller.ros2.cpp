#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <functional>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/ros_conversions.hpp>
#include <lightweight_ur_interface/ur_robot_config.hpp>

namespace lightweight_ur_interface
{
using common_robotics_utilities::utility::ClampValue;
using common_robotics_utilities::ros_conversions::GeometryPoseToEigenIsometry3d;

class URCartesianControllerNode : public rclcpp::Node
{
private:

  typedef Eigen::Matrix<double, 6, 1> Twist;

  std::vector<PIDParams> axis_controller_parameters_;
  std::vector<double> axis_velocity_limits_;
  std::string pose_frame_;
  std::string twist_frame_;

  bool target_pose_valid_ = false;
  bool current_pose_valid_ = false;
  Eigen::Isometry3d target_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d current_pose_ = Eigen::Isometry3d::Identity();

  Twist pose_error_integral_ = Twist::Zero();
  Twist last_pose_error_ = Twist::Zero();

  double control_interval_;
  uint8_t iteration_count_ = 0x01;

  std::shared_ptr<
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> command_pub_;
  std::shared_ptr<
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> feedback_sub_;
  std::shared_ptr<
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> pose_target_sub_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> abort_service_;
  std::shared_ptr<rclcpp::TimerBase> control_timer_;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit URCartesianControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ur_cartesian_controller", options)
  {
    const std::string DEFAULT_POSE_FRAME = "base";
    const std::string DEFAULT_TWIST_FRAME = "ur10_ee_frame";
    const std::string DEFAULT_TARGET_POSE_TOPIC = "/ur10/target_cartesian_pose";
    const std::string DEFAULT_POSE_FEEDBACK_TOPIC = "/ur10/ee_pose";
    const std::string DEFAULT_TWIST_COMMAND_TOPIC = "/ur10/ee_twist_command";
    const std::string DEFAULT_ABORT_SERVICE = "/ur10_cartesian_controller/abort";
    constexpr double DEFAULT_TRANSLATION_KP = 1.0;
    constexpr double DEFAULT_ROTATION_KP = 1.0;
    constexpr double DEFAULT_TRANSLATION_KD = 0.1;
    constexpr double DEFAULT_ROTATION_KD = 0.1;
    constexpr double DEFAULT_CONTROL_RATE = 150.0;
    constexpr double DEFAULT_MAX_LINEAR_VELOCITY = 0.5;
    constexpr double DEFAULT_MAX_ANGULAR_VELOCITY = 1.0;

    RCLCPP_INFO(this->get_logger(), "Starting ur_cartesian_controller...");

    pose_frame_
        = this->declare_parameter("pose_frame", DEFAULT_POSE_FRAME);
    twist_frame_
        = this->declare_parameter("twist_frame", DEFAULT_TWIST_FRAME);

    const double translation_kp
        = std::abs(this->declare_parameter("translation_kp",
                                           DEFAULT_TRANSLATION_KP));
    const double rotation_kp
        = std::abs(this->declare_parameter("rotation_kp",
                                           DEFAULT_ROTATION_KP));
    const double translation_kd
        = std::abs(this->declare_parameter("translation_kd",
                                           DEFAULT_TRANSLATION_KD));
    const double rotation_kd
        = std::abs(this->declare_parameter("rotation_kd",
                                           DEFAULT_ROTATION_KD));
    axis_controller_parameters_ =
        lightweight_ur_interface::GetDefaultPoseControllerParams(
            translation_kp, translation_kd, rotation_kp, rotation_kd);

    const double max_linear_velocity
        = std::abs(this->declare_parameter("max_linear_velocity",
                                           DEFAULT_MAX_LINEAR_VELOCITY));
    const double max_angular_velocity
        = std::abs(this->declare_parameter("max_angular_velocity",
                                           DEFAULT_MAX_ANGULAR_VELOCITY));
    axis_velocity_limits_ = {
      max_linear_velocity, max_linear_velocity, max_linear_velocity,
      max_angular_velocity, max_angular_velocity, max_angular_velocity};

    const std::string twist_command_topic
        = this->declare_parameter("twist_command_topic",
                                  DEFAULT_TWIST_COMMAND_TOPIC);
    command_pub_
        = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            twist_command_topic, 1);

    using std::placeholders::_1;
    using std::placeholders::_2;
    const std::string pose_feedback_topic
        = this->declare_parameter("pose_feedback_topic",
                                  DEFAULT_POSE_FEEDBACK_TOPIC);
    feedback_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_feedback_topic, 1,
        std::bind(&URCartesianControllerNode::PoseFeedbackCallback, this, _1));

    const std::string target_pose_topic
        = this->declare_parameter("target_pose_topic",
                                  DEFAULT_TARGET_POSE_TOPIC);
    pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic, 1,
        std::bind(&URCartesianControllerNode::PoseTargetCallback, this, _1));

    const std::string abort_service
        = this->declare_parameter("abort_service",
                                  DEFAULT_ABORT_SERVICE);
    abort_service_ = this->create_service<std_srvs::srv::Empty>(
        abort_service, std::bind(
            &URCartesianControllerNode::AbortCB, this, _1, _2));

    const double control_rate
        = std::abs(this->declare_parameter("control_rate",
                                           DEFAULT_CONTROL_RATE));

    control_interval_ = 1. / control_rate;

    control_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(control_interval_),
        std::bind(&URCartesianControllerNode::SpinOnce, this));
    RCLCPP_INFO(this->get_logger(), "...startup complete");
  }

  void SpinOnce()
  {
    constexpr uint8_t supersample_rate = 0x01;
    // Run controller
    if (iteration_count_ == supersample_rate)
    {
      if (target_pose_valid_ && current_pose_valid_)
      {
        const Twist raw_pose_correction
            = ComputeRawNextStep(current_pose_,
                                 target_pose_,
                                 control_interval_);
        const Twist real_pose_correction
            = LimitCorrectionTwist(raw_pose_correction);
        CommandTwist(real_pose_correction, twist_frame_);
      }
      iteration_count_ = 0x01;
    }
    else
    {
      iteration_count_++;
    }
  }

private:

  void AbortCB(
      std::shared_ptr<std_srvs::srv::Empty::Request> req,
      std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    UNUSED(req);
    UNUSED(res);
    RCLCPP_INFO(this->get_logger(), "Aborting pose target");
    const Twist stop_twist = Twist::Zero();
    CommandTwist(stop_twist, twist_frame_);
    target_pose_valid_ = false;
    pose_error_integral_ = Twist::Zero();
    last_pose_error_ = Twist::Zero();
  }

  Twist ComputeNewPoseErrorIntegral(
      const Twist& raw_pose_error_integral) const
  {
    Twist limited_pose_error_integral;
    limited_pose_error_integral(0)
        = ClampValue(raw_pose_error_integral(0),
                     -axis_controller_parameters_[0].Iclamp(),
                      axis_controller_parameters_[0].Iclamp());
    limited_pose_error_integral(1)
        = ClampValue(raw_pose_error_integral(1),
                     -axis_controller_parameters_[1].Iclamp(),
                      axis_controller_parameters_[1].Iclamp());
    limited_pose_error_integral(2)
        = ClampValue(raw_pose_error_integral(2),
                     -axis_controller_parameters_[2].Iclamp(),
                      axis_controller_parameters_[2].Iclamp());
    limited_pose_error_integral(3)
        = ClampValue(raw_pose_error_integral(3),
                     -axis_controller_parameters_[3].Iclamp(),
                      axis_controller_parameters_[3].Iclamp());
    limited_pose_error_integral(4)
        = ClampValue(raw_pose_error_integral(4),
                     -axis_controller_parameters_[4].Iclamp(),
                      axis_controller_parameters_[4].Iclamp());
    limited_pose_error_integral(5)
        = ClampValue(raw_pose_error_integral(5),
                     -axis_controller_parameters_[5].Iclamp(),
                      axis_controller_parameters_[5].Iclamp());
    return limited_pose_error_integral;
  }

  Twist ComputePoseError(const Eigen::Isometry3d& current_pose,
                         const Eigen::Isometry3d& target_pose) const
  {
    return common_robotics_utilities::math::TwistBetweenTransforms(current_pose,
                                                                   target_pose);
  }

  Twist ComputeRawNextStep(const Eigen::Isometry3d& current_pose,
                           const Eigen::Isometry3d& target_pose,
                           const double time_interval)
  {
    // Compute the pose error in our 'world frame'
    const Twist pose_error = ComputePoseError(current_pose, target_pose);
    // Compute the integral of pose error & update the stored value
    pose_error_integral_
        = ComputeNewPoseErrorIntegral(pose_error_integral_
                                      + (pose_error * time_interval));
    // Compute the derivative of pose error
    const Twist pose_error_derivative
        = (pose_error - last_pose_error_) / time_interval;
    // Update the stored pose error
    last_pose_error_ = pose_error;
    // Convert pose errors into cartesian velocity
    Twist raw_pose_correction;
    raw_pose_correction(0)
        = (pose_error(0) * axis_controller_parameters_[0].Kp())
          + (pose_error_integral_(0) * axis_controller_parameters_[0].Ki())
          + (pose_error_derivative(0) * axis_controller_parameters_[0].Kd());
    raw_pose_correction(1)
        = (pose_error(1) * axis_controller_parameters_[1].Kp())
          + (pose_error_integral_(1) * axis_controller_parameters_[1].Ki())
          + (pose_error_derivative(1) * axis_controller_parameters_[1].Kd());
    raw_pose_correction(2)
        = (pose_error(2) * axis_controller_parameters_[2].Kp())
          + (pose_error_integral_(2) * axis_controller_parameters_[2].Ki())
          + (pose_error_derivative(2) * axis_controller_parameters_[2].Kd());
    raw_pose_correction(3)
        = (pose_error(3) * axis_controller_parameters_[3].Kp())
          + (pose_error_integral_(3) * axis_controller_parameters_[3].Ki())
          + (pose_error_derivative(3) * axis_controller_parameters_[3].Kd());
    raw_pose_correction(4)
        = (pose_error(4) * axis_controller_parameters_[4].Kp())
          + (pose_error_integral_(4) * axis_controller_parameters_[4].Ki())
          + (pose_error_derivative(4) * axis_controller_parameters_[4].Kd());
    raw_pose_correction(5)
        = (pose_error(5) * axis_controller_parameters_[5].Kp())
          + (pose_error_integral_(5) * axis_controller_parameters_[5].Ki())
          + (pose_error_derivative(5) * axis_controller_parameters_[5].Kd());
    return raw_pose_correction;
  }

  Twist LimitCorrectionTwist(const Twist& raw_twist) const
  {
    Twist limited_twist;
    limited_twist(0) = ClampValue(raw_twist(0),
                                  -axis_velocity_limits_[0],
                                  axis_velocity_limits_[0]);
    limited_twist(1) = ClampValue(raw_twist(1),
                                  -axis_velocity_limits_[1],
                                  axis_velocity_limits_[1]);
    limited_twist(2) = ClampValue(raw_twist(2),
                                  -axis_velocity_limits_[2],
                                  axis_velocity_limits_[2]);
    limited_twist(3) = ClampValue(raw_twist(3),
                                  -axis_velocity_limits_[3],
                                  axis_velocity_limits_[3]);
    limited_twist(4) = ClampValue(raw_twist(4),
                                  -axis_velocity_limits_[4],
                                  axis_velocity_limits_[4]);
    limited_twist(5) = ClampValue(raw_twist(5),
                                  -axis_velocity_limits_[5],
                                  axis_velocity_limits_[5]);
    return limited_twist;
  }

  void CommandTwist(const Twist& command, const std::string& frame)
  {
    geometry_msgs::msg::TwistStamped command_msg;
    command_msg.header.frame_id = frame;
    command_msg.header.stamp = this->get_clock()->now();
    command_msg.twist.linear.x = command(0, 0);
    command_msg.twist.linear.y = command(1, 0);
    command_msg.twist.linear.z = command(2, 0);
    command_msg.twist.angular.x = command(3, 0);
    command_msg.twist.angular.y = command(4, 0);
    command_msg.twist.angular.z = command(5, 0);
    command_pub_->publish(command_msg);
  }

  void PoseTargetCallback(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    if (target_pose.header.frame_id == pose_frame_)
    {
      RCLCPP_INFO(
          this->get_logger(), "Starting execution to a new target pose");
      target_pose_ = GeometryPoseToEigenIsometry3d(target_pose.pose);
      target_pose_valid_ = true;
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(), "Invalid target pose frame %s, should be %s",
          target_pose.header.frame_id.c_str(), pose_frame_.c_str());
      target_pose_valid_ = false;
    }
  }

  void
  PoseFeedbackCallback(const geometry_msgs::msg::PoseStamped& pose_feedback)
  {
    if (pose_feedback.header.frame_id == pose_frame_)
    {
      current_pose_ = GeometryPoseToEigenIsometry3d(pose_feedback.pose);
      current_pose_valid_ = true;
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(), "Invalid feedback pose frame %s, should be %s",
          pose_feedback.header.frame_id.c_str(), pose_frame_.c_str());
      current_pose_valid_ = false;
    }
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lightweight_ur_interface::URCartesianControllerNode)
