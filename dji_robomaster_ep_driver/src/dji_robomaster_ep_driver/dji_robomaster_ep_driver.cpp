#include <dji_robomaster_ep_driver/dji_robomaster_ep_driver.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <Eigen/Geometry>

#include <common_robotics_utilities/utility.hpp>

namespace dji_robomaster_ep_driver
{
DJIRobomasterEPState::DJIRobomasterEPState(
    const Eigen::Vector3d& chassis_position,
    const Eigen::Vector3d& chassis_attitude,
    const Eigen::Vector3d& chassis_speed, const Eigen::Vector4d& wheel_speed,
    const double battery_percent)
    : wheel_speed_(wheel_speed), battery_percent_(battery_percent),
      is_valid_(true)
{
  // Chassis pose
  // TODO(calderpg) Figure out how to handle the chassis attitude.
  CRU_UNUSED(chassis_attitude);

  const Eigen::Translation3d position(
      chassis_position.x(), chassis_position.y(), 0.0);

  const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(chassis_position.z(), Eigen::Vector3d::UnitZ()));

  const Eigen::Isometry3d pose = position * orientation;
  pose_ = pose;

  // Chassis velocity
  Twist velocity = Twist::Zero();
  velocity(0) = chassis_speed.x();
  velocity(1) = chassis_speed.y();
  velocity(2) = 0.0;
  velocity(3) = 0.0;
  velocity(4) = 0.0;
  velocity(5) = chassis_speed.z();
  velocity_ = velocity;

  // Set state time
  time_ = std::chrono::steady_clock::now();
}

DJIRobomasterEPInterfaceTCP::DJIRobomasterEPInterfaceTCP(
    const std::string& robot_ip_address, const int32_t robot_port,
    const int32_t safety_timeout_ms) : safety_timeout_ms_(safety_timeout_ms)
{
  if (safety_timeout_ms_ <= 0)
  {
    throw std::invalid_argument("safety_timeout_ms_ <= 0");
  }

  struct hostent* nameserver = gethostbyname(robot_ip_address.c_str());
  if (nameserver == nullptr)
  {
    perror(nullptr);
    throw std::runtime_error(
        "Failed to find robot at [" + robot_ip_address + "]");
  }
  struct sockaddr_in robot_addr;
  std::memset(&robot_addr, 0, sizeof(robot_addr));
  robot_addr.sin_family = AF_INET;
  std::memcpy(
      &robot_addr.sin_addr.s_addr,
      nameserver->h_addr,
      static_cast<size_t>(nameserver->h_length));
  robot_addr.sin_port = htons(static_cast<uint16_t>(robot_port));

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to create socket");
  }

  const int enable_flag = 1;

  const int setnodelay_res = setsockopt(
      socket_fd_, IPPROTO_TCP, TCP_NODELAY,
      reinterpret_cast<const void*>(&enable_flag), sizeof(int));
  if (setnodelay_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to enable TCP_NODELAY");
  }

  const int setquickack_res = setsockopt(
      socket_fd_, IPPROTO_TCP, TCP_QUICKACK,
      reinterpret_cast<const void*>(&enable_flag), sizeof(int));
  if (setquickack_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to enable TCP_QUICKACK");
  }

  const int setreuseaddr_res = setsockopt(
      socket_fd_, SOL_SOCKET, SO_REUSEADDR,
      reinterpret_cast<const void*>(&enable_flag), sizeof(int));
  if (setreuseaddr_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to enable SO_REUSEADDR");
  }

  struct timeval tv;
  tv.tv_sec  = 1;
  tv.tv_usec = 0;
  const int setrcvtimeo_res = setsockopt(
      socket_fd_, SOL_SOCKET, SO_RCVTIMEO,
      reinterpret_cast<const void*>(&tv), sizeof(tv));
  if (setrcvtimeo_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to enable SO_RCVTIMEO");
  }

  const int connect_res = connect(
      socket_fd_, reinterpret_cast<struct sockaddr*>(&robot_addr),
      sizeof(robot_addr));
  if (connect_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to connect");
  }
}

void DJIRobomasterEPInterfaceTCP::CommandVelocity(const Twist& velocity_command)
{
  const auto clamp_linear_velocity = [] (const double val)
  {
    const double linear_velocity_limit = 3.5;
    return common_robotics_utilities::utility::ClampValue(
        val, -linear_velocity_limit, linear_velocity_limit);
  };

  const auto clamp_angular_velocity = [] (const double val)
  {
    const double angular_velocity_limit = DegreesToRadians(600.0);
    return common_robotics_utilities::utility::ClampValue(
        val, -angular_velocity_limit, angular_velocity_limit);
  };

  // TODO(calderpg) Better handle vehicle frame rotation than this.
  const Eigen::Vector3d limited_velocity_command(
      clamp_linear_velocity(velocity_command(0)),
      -clamp_linear_velocity(velocity_command(1)),
      -clamp_angular_velocity(velocity_command(5)));

  std::lock_guard<std::mutex> lock(state_mutex_);
  velocity_command_ = limited_velocity_command;
  velocity_command_time_ = std::chrono::steady_clock::now();
}

void DJIRobomasterEPInterfaceTCP::Start()
{
  // Enter command mode
  SendCommandAndAwaitResponse("command;");
  // Switch to "free" command mode
  SendCommandAndAwaitResponse("robot mode free;");
  // Start command loop
  run_command_thread_.store(true);
  command_thread_ =
      std::thread(&DJIRobomasterEPInterfaceTCP::CommandLoop, this);
}

void DJIRobomasterEPInterfaceTCP::Stop()
{
  if (socket_fd_ >= 0)
  {
    // Stop command loop
    run_command_thread_.store(false);
    if (command_thread_.joinable())
    {
      command_thread_.join();
    }
    // Exit command mode
    SendCommandAndAwaitResponse("quit;");
    // Shutdown socket
    const int shutdown_res = shutdown(socket_fd_, SHUT_RDWR);
    if (shutdown_res != 0)
    {
      perror(nullptr);
      throw std::runtime_error("Failed to shutdown socket");
    }
    socket_fd_ = -1;
  }
}

void DJIRobomasterEPInterfaceTCP::CommandLoop()
{
  const std::chrono::milliseconds safety_timeout(safety_timeout_ms_);

  while (run_command_thread_.load())
  {
    Eigen::Vector3d velocity_command = Eigen::Vector3d::Zero();
    const SteadyTimePoint now_time = std::chrono::steady_clock::now();

    // Only command non-zero velocity if the command is new enough.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const std::chrono::milliseconds velocity_command_age =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              now_time - velocity_command_time_);
      if (velocity_command_age < safety_timeout)
      {
        velocity_command = velocity_command_;
      }
    }

    // Send command and receive the latest robot state.
    const DJIRobomasterEPState latest_state =
        RunCommandCycle(velocity_command);

    // Update the stored state
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = latest_state;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

DJIRobomasterEPState DJIRobomasterEPInterfaceTCP::RunCommandCycle(
    const Eigen::Vector3d& velocity_command)
{
  // Command robot velocity
  // When you command very low velocities, the robot tends to drift slowly.
  // To work around this, command zero wheel speed instead of chassis speed.
  if (velocity_command.norm() > 0.01)
  {
    const int buffer_len = 1024;
    char command_buffer[buffer_len];
    const int written = std::snprintf(
        command_buffer, buffer_len - 1,
        "chassis speed x %1.5f y %1.5f z %1.5f;",
        velocity_command.x(), velocity_command.y(),
        RadiansToDegrees(velocity_command.z()));
    if ((written <= 0) || (written >= buffer_len))
    {
      throw std::runtime_error("snprintf for velocity command failed");
    }
    const std::string velocity_command_str(command_buffer);
    SendCommandAndAwaitResponse(velocity_command_str);
  }
  else
  {
    SendCommandAndAwaitResponse("chassis wheel w1 0 w2 0 w3 0 w4 0;");
  }

  // Get robot state
  const std::string chassis_speed_str =
      SendCommandAndAwaitResponse("chassis speed ?;");
  const std::string chassis_position_str =
      SendCommandAndAwaitResponse("chassis position ?;");
  const std::string chassis_attitude_str =
      SendCommandAndAwaitResponse("chassis attitude ?;");
  const std::string robot_battery_str =
      SendCommandAndAwaitResponse("robot battery ?;");

  // Extract chassis and wheel speeds
  Eigen::Vector3d chassis_speed = Eigen::Vector3d::Zero();
  Eigen::Vector4d wheel_speed = Eigen::Vector4d::Zero();
  const int chassis_speed_read = std::sscanf(
      chassis_speed_str.c_str(), "%lf %lf %lf %lf %lf %lf %lf",
      &chassis_speed.x(), &chassis_speed.y(), &chassis_speed.z(),
      &wheel_speed(0), &wheel_speed(1), &wheel_speed(2), &wheel_speed(3));
  if (chassis_speed_read != 7)
  {
    throw std::runtime_error("Failed to extract chassis and wheel speed");
  }
  chassis_speed.y() = -chassis_speed.y();
  chassis_speed.z() = -DegreesToRadians(chassis_speed.z());
  wheel_speed(0) = RevolutionsPerMinuteToRadiansPerSecond(wheel_speed(0));
  wheel_speed(1) = RevolutionsPerMinuteToRadiansPerSecond(wheel_speed(1));
  wheel_speed(2) = RevolutionsPerMinuteToRadiansPerSecond(wheel_speed(2));
  wheel_speed(3) = RevolutionsPerMinuteToRadiansPerSecond(wheel_speed(3));

  // Extract chassis position
  Eigen::Vector3d chassis_position = Eigen::Vector3d::Zero();
  const int chassis_position_read = std::sscanf(
      chassis_position_str.c_str(), "%lf %lf %lf",
      &chassis_position.x(), &chassis_position.y(), &chassis_position.z());
  if (chassis_position_read != 3)
  {
    throw std::runtime_error("Failed to extract chassis position");
  }
  chassis_position.y() = -chassis_position.y();
  chassis_position.z() = -DegreesToRadians(chassis_position.z());

  // Extract chassis attitude
  Eigen::Vector3d chassis_attitude = Eigen::Vector3d::Zero();
  // Note that attitude is pitch, roll, yaw!
  const int chassis_attitude_read = std::sscanf(
      chassis_attitude_str.c_str(), "%lf %lf %lf",
      &chassis_attitude.y(), &chassis_attitude.x(), &chassis_attitude.z());
  if (chassis_attitude_read != 3)
  {
    throw std::runtime_error("Failed to extract chassis attitude");
  }
  chassis_attitude.x() = DegreesToRadians(chassis_attitude.x());
  chassis_attitude.y() = DegreesToRadians(chassis_attitude.y());
  chassis_attitude.z() = DegreesToRadians(chassis_attitude.z());

  // Extract battery percentage
  double battery_percent = 0.0;
  const int battery_percent_read = std::sscanf(
      robot_battery_str.c_str(), "%lf", &battery_percent);
  if (battery_percent_read != 1)
  {
    throw std::runtime_error("Failed to extract battery percentage");
  }

  return DJIRobomasterEPState(
      chassis_position, chassis_attitude, chassis_speed, wheel_speed,
      battery_percent);
}

std::string DJIRobomasterEPInterfaceTCP::SendCommandAndAwaitResponse(
    const std::string& command_str)
{
  const char command_end = ';';
  const char last_command_char = (command_str.size() > 0)
      ? static_cast<char>(command_str.back()) : '\0';
  const bool valid_command_format = (last_command_char == command_end);
  if (!valid_command_format)
  {
    throw std::runtime_error(
        "Command [" + command_str + "] is not correctly formatted");
  }

  const ssize_t bytes_written
      = write(socket_fd_, command_str.c_str(), command_str.size());
  if (bytes_written != static_cast<ssize_t>(command_str.size()))
  {
    perror(nullptr);
    throw std::runtime_error(
        "Failed to send command [" + command_str + "] with "
        + std::to_string(command_str.size()) + " bytes, sent "
        + std::to_string(bytes_written) + " instead");
  }

  std::vector<uint8_t> recv_buffer(1024, 0x00);
  const ssize_t bytes_read
      = read(socket_fd_, recv_buffer.data(), recv_buffer.size());
  if (bytes_read > 0)
  {
    const int enable_flag = 1;
    const int setquickack_res = setsockopt(
        socket_fd_, IPPROTO_TCP, TCP_QUICKACK,
        reinterpret_cast<const void*>(&enable_flag), sizeof(int));
    if (setquickack_res != 0)
    {
      perror(nullptr);
      throw std::runtime_error("Failed to enable TCP_QUICKACK");
    }

    const std::string response_str(
        reinterpret_cast<const char*>(recv_buffer.data()));

    return response_str;
  }
  else if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
  {
    return "";
  }
  else
  {
    perror(nullptr);
    throw std::runtime_error("Failed to receive response from robot");
  }
}
}  // namespace dji_robomaster_ep_driver

