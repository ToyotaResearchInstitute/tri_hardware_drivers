#pragma once

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
#include <atomic>
#include <functional>
#include <Eigen/Geometry>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/serialization.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>

namespace lightweight_ur_interface
{
using Twist = Eigen::Matrix<double, 6, 1>;
using Wrench = Eigen::Matrix<double, 6, 1>;

template<typename T, typename Container=std::vector<T>>
inline common_robotics_utilities::serialization::Deserialized<Container>
DeserializeKnownSizeVector(
    const size_t size,
    const std::vector<uint8_t>& buffer,
    const uint64_t starting_offset,
    const common_robotics_utilities::serialization::Deserializer<T>&
        item_deserializer)
{
  uint64_t current_position = starting_offset;
  // Deserialize the items
  Container deserialized;
  deserialized.reserve(size);
  for (uint64_t idx = 0; idx < size; idx++)
  {
    const auto deserialized_item = item_deserializer(buffer, current_position);
    deserialized.push_back(deserialized_item.Value());
    current_position += deserialized_item.BytesRead();
  }
  deserialized.shrink_to_fit();
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - starting_offset;
  return common_robotics_utilities::serialization::MakeDeserialized(
      deserialized, bytes_read);
}

class URRealtimeState
{
private:
  template<typename T>
  using Deserialized
      = common_robotics_utilities::serialization::Deserialized<T>;

  std::vector<double> target_position_;
  std::vector<double> target_velocity_;
  std::vector<double> target_acceleration_;
  std::vector<double> target_current_;
  std::vector<double> target_torque_;
  std::vector<double> actual_position_;
  std::vector<double> actual_velocity_;
  std::vector<double> actual_acceleration_;
  std::vector<double> actual_current_;
  std::vector<double> actual_torque_;
  std::vector<double> control_current_;
  std::vector<double> motor_temperature_;
  std::vector<double> joint_voltage_;
  std::vector<double> joint_mode_;
  std::vector<double> actual_tcp_acceleration_;
  std::vector<double> raw_target_tcp_pose_;
  std::vector<double> raw_actual_tcp_pose_;
  Eigen::Isometry3d target_tcp_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d actual_tcp_pose_ = Eigen::Isometry3d::Identity();
  Twist target_tcp_twist_ = Twist::Zero();
  Wrench target_tcp_wrench_ = Wrench::Zero();
  Twist actual_tcp_twist_ = Twist::Zero();
  Wrench actual_tcp_wrench_ = Wrench::Zero();
  double protocol_version_ = 0.0;
  double controller_uptime_ = 0.0;
  double controller_rt_loop_time_ = 0.0;
  double robot_mode_ = 0.0;
  double safety_mode_ = 0.0;
  double trajectory_limiter_speed_scaling_ = 0.0;
  double linear_momentum_norm_ = 0.0;
  double mainboard_voltage_ = 0.0;
  double motorboard_voltage_ = 0.0;
  double mainboard_current_ = 0.0;
  bool initialized_ = false;

  static Deserialized<std::vector<double>>
  DeserializeKnownSizeDoubleVector(
      const size_t size, const std::vector<uint8_t>& buffer,
      const uint64_t starting_offset)
  {
    return DeserializeKnownSizeVector<double, std::vector<double>>(
        size, buffer, starting_offset,
        common_robotics_utilities::serialization
            ::DeserializeNetworkMemcpyable<double>);
  }

  static Eigen::Isometry3d TcpVectorToTransform(
      const std::vector<double>& tcp_vector)
  {
    const Eigen::Translation3d translation(
        tcp_vector.at(0), tcp_vector.at(1), tcp_vector.at(2));
    const double rx = tcp_vector.at(3);
    const double ry = tcp_vector.at(4);
    const double rz = tcp_vector.at(5);
    const double angle = std::sqrt((rx * rx) + (ry * ry) + (rz * rz));
    Eigen::Quaterniond rotation;
    if (angle < 1e-16)
    {
      rotation = Eigen::Quaterniond::Identity();
    }
    else
    {
      const double w = std::cos(angle * 0.5);
      const double x = (rx / angle) * std::sin(angle * 0.5);
      const double y = (ry / angle) * std::sin(angle * 0.5);
      const double z = (rz / angle) * std::sin(angle * 0.5);
      rotation = Eigen::Quaterniond(w, x, y, z);
    }
    const Eigen::Isometry3d transform = translation * rotation;
    return transform;
  }

  static Twist TcpVelocityToTwist(const std::vector<double>& tcp_velocity)
  {
    Twist twist = Twist::Zero();
    twist(0, 0) = tcp_velocity.at(0);
    twist(1, 0) = tcp_velocity.at(1);
    twist(2, 0) = tcp_velocity.at(2);
    twist(3, 0) = tcp_velocity.at(3);
    twist(4, 0) = tcp_velocity.at(4);
    twist(5, 0) = tcp_velocity.at(5);
    return twist;
  }

  static Wrench TcpForceToWrench(const std::vector<double>& tcp_force)
  {
    Wrench wrench = Wrench::Zero();
    wrench(0, 0) = tcp_force.at(0);
    wrench(1, 0) = tcp_force.at(1);
    wrench(2, 0) = tcp_force.at(2);
    wrench(3, 0) = tcp_force.at(3);
    wrench(4, 0) = tcp_force.at(4);
    wrench(5, 0) = tcp_force.at(5);
    return wrench;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  URRealtimeState() : initialized_(false) {}

  static Deserialized<URRealtimeState> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    URRealtimeState deserialized_state;
    const uint64_t bytes_read
        = deserialized_state.DeserializeSelf(buffer, starting_offset);
    return common_robotics_utilities::serialization::MakeDeserialized(
        deserialized_state, bytes_read);
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

  bool Initialized() { return initialized_; }

  const std::vector<double>& TargetPosition() const { return target_position_; }

  const std::vector<double>& TargetVelocity() const { return target_velocity_; }

  const std::vector<double>& TargetAcceleration() const
  { return target_acceleration_; }

  const std::vector<double>& TargetCurrent() const { return target_current_; }

  const std::vector<double>& TargetTorque() const { return target_torque_; }

  const std::vector<double>& ActualPosition() const { return actual_position_; }

  const std::vector<double>& ActualVelocity() const { return actual_velocity_; }

  const std::vector<double>& ActualAcceleration() const
  { return actual_acceleration_; }

  const std::vector<double>& ActualCurrent() const { return actual_current_; }

  const std::vector<double>& ActualTorque() const { return actual_torque_; }

  const std::vector<double>& ControlCurrent() const { return control_current_; }

  const std::vector<double>& MotorTemperature() const
  { return motor_temperature_; }

  const std::vector<double>& JointVoltage() const { return joint_voltage_; }

  const std::vector<double>& JointMode() const { return joint_mode_; }

  const std::vector<double>& ActualTcpAcceleration() const
  { return actual_tcp_acceleration_; }

  const std::vector<double>& RawTargetTcpPose() const
  { return raw_target_tcp_pose_; }

  const Eigen::Isometry3d& TargetTcpPose() const { return target_tcp_pose_; }

  const Twist& TargetTcpTwist() const { return target_tcp_twist_; }

  const Wrench& TargetTcpWrench() const { return target_tcp_wrench_; }

  const std::vector<double>& RawActualTcpPose() const
  { return raw_actual_tcp_pose_; }

  const Eigen::Isometry3d& ActualTcpPose() const { return actual_tcp_pose_; }

  const Twist& ActualTcpTwist() const { return actual_tcp_twist_; }

  const Wrench& ActualTcpWrench() const { return actual_tcp_wrench_; }

  double ProtocolVersion() const { return protocol_version_; }

  double ControllerUptime() const { return controller_uptime_; }

  double ControllerRtLoopTime() const { return controller_rt_loop_time_; }

  double RobotMode() const { return robot_mode_; }

  double SafetyMode() const { return safety_mode_; }

  double TrajectoryLimiterSpeedScaling() const
  { return trajectory_limiter_speed_scaling_; }

  double LinearMomentumNorm() const { return linear_momentum_norm_; }

  double MainboardVoltage() const { return mainboard_voltage_; }

  double MotorboardVoltage() const { return motorboard_voltage_; }

  double MainboardCurrent() const { return mainboard_current_; }
};

class URRealtimeInterface
{
private:

  int socket_fd_ = 0;
  struct sockaddr_in robot_addr_{};
  std::mutex socket_mutex_;
  std::atomic<bool> connected_;
  std::atomic<bool> running_;
  std::thread recv_thread_;
  std::function<void(const URRealtimeState&)> state_received_callback_fn_;
  std::function<void(const std::string&)> logging_fn_;

  void ConnectToRobot();

  void RecvLoop();

public:

  URRealtimeInterface(
      const std::string& robot_host,
      const std::function<void(const URRealtimeState&)>&
          state_received_callback_fn,
      const std::function<void(const std::string&)>& logging_fn);

  ~URRealtimeInterface();

  void Log(const std::string& message) { logging_fn_(message); }

  void StartRecv();

  void StopRecv();

  bool SendURScriptCommand(const std::string& command);
};
}  // namespace lightweight_ur_interface
