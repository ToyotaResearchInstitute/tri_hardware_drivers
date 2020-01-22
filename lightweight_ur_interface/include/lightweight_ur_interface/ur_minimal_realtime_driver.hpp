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
  using common_robotics_utilities::serialization::Deserialized;
  using common_robotics_utilities::serialization::MakeDeserialized;
  using common_robotics_utilities::serialization::DeserializeNetworkMemcpyable;

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
  Eigen::Isometry3d target_tcp_pose_;
  Eigen::Isometry3d actual_tcp_pose_;
  Eigen::Matrix<double, 6, 1> target_tcp_twist_;
  Eigen::Matrix<double, 6, 1> target_tcp_wrench_;
  Eigen::Matrix<double, 6, 1> actual_tcp_twist_;
  Eigen::Matrix<double, 6, 1> actual_tcp_wrench_;
  double protocol_version_;
  double controller_uptime_;
  double controller_rt_loop_time_;
  double robot_mode_;
  double safety_mode_;
  double trajectory_limiter_speed_scaling_;
  double linear_momentum_norm_;
  double mainboard_voltage_;
  double motorboard_voltage_;
  double mainboard_current_;
  bool initialized_;

  static inline Deserialized<std::vector<double>>
  DeserializeKnownSizeDoubleVector(
      const size_t size, const std::vector<uint8_t>& buffer,
      const uint64_t starting_offset)
  {
    return DeserializeKnownSizeVector<double, std::vector<double>>(
      size, buffer, starting_offset, DeserializeNetworkMemcpyable<double>);
  }

  static inline Eigen::Isometry3d TcpVectorToTransform(
      const std::vector<double>& tcp_vector)
  {
    const Eigen::Translation3d translation(tcp_vector[0],
                                           tcp_vector[1],
                                           tcp_vector[2]);
    const double rx = tcp_vector[3];
    const double ry = tcp_vector[4];
    const double rz = tcp_vector[5];
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

  static inline Eigen::Matrix<double, 6, 1> TcpVelocityToTwist(
      const std::vector<double>& tcp_velocity)
  {
    Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
    twist(0, 0) = tcp_velocity[0];
    twist(1, 0) = tcp_velocity[1];
    twist(2, 0) = tcp_velocity[2];
    twist(3, 0) = tcp_velocity[3];
    twist(4, 0) = tcp_velocity[4];
    twist(5, 0) = tcp_velocity[5];
    return twist;
  }

  static inline Eigen::Matrix<double, 6, 1> TcpForceToWrench(
      const std::vector<double>& tcp_velocity)
  {
    Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
    twist(0, 0) = tcp_velocity[0];
    twist(1, 0) = tcp_velocity[1];
    twist(2, 0) = tcp_velocity[2];
    twist(3, 0) = tcp_velocity[3];
    twist(4, 0) = tcp_velocity[4];
    twist(5, 0) = tcp_velocity[5];
    return twist;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  URRealtimeState() : initialized_(false) {}

  inline static Deserialized<URRealtimeState> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset)
  {
    URRealtimeState deserialized_state;
    const uint64_t bytes_read
        = deserialized_state.DeserializeSelf(buffer, starting_offset);
    return MakeDeserialized(deserialized_state, bytes_read);
  }

  uint64_t DeserializeSelf(
      const std::vector<uint8_t>& buffer, const uint64_t starting_offset);

  inline bool Initialized() { return initialized_; }

  inline const std::vector<double>& TargetPosition() const
  { return target_position_; }

  inline const std::vector<double>& TargetVelocity() const
  { return target_velocity_; }

  inline const std::vector<double>& TargetAcceleration() const
  { return target_acceleration_; }

  inline const std::vector<double>& TargetCurrent() const
  { return target_current_; }

  inline const std::vector<double>& TargetTorque() const
  { return target_torque_; }

  inline const std::vector<double>& ActualPosition() const
  { return actual_position_; }

  inline const std::vector<double>& ActualVelocity() const
  { return actual_velocity_; }

  inline const std::vector<double>& ActualAcceleration() const
  { return actual_acceleration_; }

  inline const std::vector<double>& ActualCurrent() const
  { return actual_current_; }

  inline const std::vector<double>& ActualTorque() const
  { return actual_torque_; }

  inline const std::vector<double>& ControlCurrent() const
  { return control_current_; }

  inline const std::vector<double>& MotorTemperature() const
  { return motor_temperature_; }

  inline const std::vector<double>& JointVoltage() const
  { return joint_voltage_; }

  inline const std::vector<double>& JointMode() const
  { return joint_mode_; }

  inline const std::vector<double>& ActualTcpAcceleration() const
  { return actual_tcp_acceleration_; }

  inline const std::vector<double>& RawTargetTcpPose() const
  { return raw_target_tcp_pose_; }

  inline const Eigen::Isometry3d& TargetTcpPose() const
  { return target_tcp_pose_; }

  inline const Eigen::Matrix<double, 6, 1>& TargetTcpTwist() const
  { return target_tcp_twist_; }

  inline const Eigen::Matrix<double, 6, 1>& TargetTcpWrench() const
  { return target_tcp_wrench_; }

  inline const std::vector<double>& RawActualTcpPose() const
  { return raw_actual_tcp_pose_; }

  inline const Eigen::Isometry3d& ActualTcpPose() const
  { return actual_tcp_pose_; }

  inline const Eigen::Matrix<double, 6, 1>& ActualTcpTwist() const
  { return actual_tcp_twist_; }

  inline const Eigen::Matrix<double, 6, 1>& ActualTcpWrench() const
  { return actual_tcp_wrench_; }

  inline double ProtocolVersion() const
  { return protocol_version_; }

  inline double ControllerUptime() const
  { return controller_uptime_; }

  inline double ControllerRtLoopTime() const
  { return controller_rt_loop_time_; }

  inline double RobotMode() const
  { return robot_mode_; }

  inline double SafetyMode() const
  { return safety_mode_; }

  inline double TrajectoryLimiterSpeedScaling() const
  { return trajectory_limiter_speed_scaling_; }

  inline double LinearMomentumNorm() const
  { return linear_momentum_norm_; }

  inline double MainboardVoltage() const
  { return mainboard_voltage_; }

  inline double MotorboardVoltage() const
  { return motorboard_voltage_; }

  inline double MainboardCurrent() const
  { return mainboard_current_; }
};

class URRealtimeInterface
{
private:

  int socket_fd_;
  struct sockaddr_in robot_addr_;
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

  inline void Log(const std::string& message) { logging_fn_(message); }

  void StartRecv();

  void StopRecv();

  bool SendURScriptCommand(const std::string& command);
};
}
