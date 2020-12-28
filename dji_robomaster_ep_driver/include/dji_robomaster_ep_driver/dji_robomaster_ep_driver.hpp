#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include <common_robotics_utilities/utility.hpp>

namespace dji_robomaster_ep_driver
{
using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock>;
using Twist = Eigen::Matrix<double, 6, 1>;

class DJIRobomasterEPState
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DJIRobomasterEPState(
      const Eigen::Vector3d& chassis_position,
      const Eigen::Vector3d& chassis_attitude,
      const Eigen::Vector3d& chassis_speed, const Eigen::Vector4d& wheel_speed,
      const double battery_percent);

  DJIRobomasterEPState() : is_valid_(false) {}

  bool IsValid() const { return is_valid_; }

  const Eigen::Isometry3d& Pose() const { return pose_; }

  const Twist& Velocity() const { return velocity_; }

  const Eigen::Vector4d& WheelSpeed() const { return wheel_speed_; }

  double BatteryPercent() const { return battery_percent_; }

  const SteadyTimePoint& Time() const { return time_; }

private:
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
  Twist velocity_ = Twist::Zero();
  Eigen::Vector4d wheel_speed_ = Eigen::Vector4d::Zero();
  SteadyTimePoint time_{};
  double battery_percent_ = 0.0;
  bool is_valid_ = false;
};

class DJIRobomasterEPInterfaceTCP
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DJIRobomasterEPInterfaceTCP(
      const std::string& robot_ip_address, const int32_t robot_port);

  ~DJIRobomasterEPInterfaceTCP() { Stop(); }

  void CommandVelocity(const Twist& velocity_command);

  DJIRobomasterEPState LatestState() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
  }

  void Start();

  void Stop();

private:
  void CommandLoop();

  DJIRobomasterEPState RunCommandCycle(const Eigen::Vector3d& velocity_command);

  std::string SendCommandAndAwaitResponse(const std::string& command_str);

  static inline double RadiansToDegrees(const double radians)
  {
    return radians * (180.0 / M_PI);
  }

  static inline double DegreesToRadians(const double degrees)
  {
    return degrees * (M_PI / 180.0);
  }

  static inline double RevolutionsPerMinuteToRadiansPerSecond(const double rpm)
  {
    return rpm * (M_PI * 2.0) / 60.0;
  }

  int socket_fd_ = -1;
  mutable std::mutex state_mutex_;
  std::atomic<bool> run_command_thread_{};
  std::thread command_thread_;

  SteadyTimePoint velocity_command_time_{};
  Eigen::Vector3d velocity_command_ = Eigen::Vector3d::Zero();
  DJIRobomasterEPState latest_state_;
};
}  // namespace dji_robomaster_ep_driver

