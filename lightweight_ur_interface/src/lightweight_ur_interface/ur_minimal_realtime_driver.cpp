#include <lightweight_ur_interface/ur_minimal_realtime_driver.hpp>

namespace lightweight_ur_interface
{
using common_robotics_utilities::serialization::DeserializeNetworkMemcpyable;

uint64_t URRealtimeState::DeserializeSelf(const std::vector<uint8_t>& buffer,
                                          const uint64_t starting_offset)
{
  uint64_t current_offset = starting_offset;
  const auto deser_length
      = DeserializeNetworkMemcpyable<int32_t>(buffer, current_offset);
  const int32_t message_length = deser_length.Value();
  current_offset += deser_length.BytesRead();
  if ((buffer.size() - current_offset) < static_cast<uint64_t>(message_length))
  {
    throw std::runtime_error("Insufficient buffer to deserialize message:"
                             " buffer.size()=" + std::to_string(buffer.size())
                             + " current_offset="
                             + std::to_string(current_offset)
                             + " message_length="
                             + std::to_string(message_length));
  }
  const auto deser_uptime
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  controller_uptime_ = deser_uptime.Value();
  current_offset += deser_uptime.BytesRead();
  const auto deser_target_position
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  target_position_ = deser_target_position.Value();
  current_offset += deser_target_position.BytesRead();
  const auto deser_target_velocity
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  target_velocity_ = deser_target_velocity.Value();
  current_offset += deser_target_velocity.BytesRead();
  const auto deser_target_acceleration
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  target_acceleration_ = deser_target_acceleration.Value();
  current_offset += deser_target_acceleration.BytesRead();
  const auto deser_target_current
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  target_current_ = deser_target_current.Value();
  current_offset += deser_target_current.BytesRead();
  const auto deser_target_torque
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  target_torque_ = deser_target_torque.Value();
  current_offset += deser_target_torque.BytesRead();
  const auto deser_actual_position
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  actual_position_ = deser_actual_position.Value();
  current_offset += deser_actual_position.BytesRead();
  const auto deser_actual_velocity
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  actual_velocity_ = deser_actual_velocity.Value();
  current_offset += deser_actual_velocity.BytesRead();
  const auto deser_actual_current
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  actual_current_ = deser_actual_current.Value();
  current_offset += deser_actual_current.BytesRead();
  const auto deser_control_current
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  control_current_ = deser_control_current.Value();
  current_offset += deser_control_current.BytesRead();
  const auto deser_actual_tcp_vector
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  raw_actual_tcp_pose_ = deser_actual_tcp_vector.Value();
  actual_tcp_pose_ = TcpVectorToTransform(raw_actual_tcp_pose_);
  current_offset += deser_actual_tcp_vector.BytesRead();
  const auto deser_actual_tcp_twist
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  actual_tcp_twist_ = TcpVelocityToTwist(deser_actual_tcp_twist.Value());
  current_offset += deser_actual_tcp_twist.BytesRead();
  const auto deser_actual_tcp_wrench
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  actual_tcp_wrench_ = TcpForceToWrench(deser_actual_tcp_wrench.Value());
  current_offset += deser_actual_tcp_wrench.BytesRead();
  const auto deser_target_tcp_vector
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  raw_target_tcp_pose_ = deser_target_tcp_vector.Value();
  target_tcp_pose_ = TcpVectorToTransform(raw_target_tcp_pose_);
  current_offset += deser_target_tcp_vector.BytesRead();
  const auto deser_target_tcp_twist
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  target_tcp_twist_ = TcpVelocityToTwist(deser_target_tcp_twist.Value());
  current_offset += deser_target_tcp_twist.BytesRead();
  const auto deser_digital_io_bits
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_offset);
  current_offset += deser_digital_io_bits.BytesRead();
  const auto deser_motor_temperature
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  motor_temperature_ = deser_motor_temperature.Value();
  current_offset += deser_motor_temperature.BytesRead();
  const auto deser_controller_rt_loop_time
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  controller_rt_loop_time_ = deser_controller_rt_loop_time.Value();
  current_offset += deser_controller_rt_loop_time.BytesRead();
  const auto deser_test_val
      = DeserializeNetworkMemcpyable<uint64_t>(buffer, current_offset);
  current_offset += deser_test_val.BytesRead();
  const auto deser_robot_mode
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  robot_mode_ = deser_robot_mode.Value();
  current_offset += deser_robot_mode.BytesRead();
  const auto deser_joint_mode
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  joint_mode_ = deser_joint_mode.Value();
  current_offset += deser_joint_mode.BytesRead();
  const auto deser_safety_mode
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  safety_mode_ = deser_safety_mode.Value();
  current_offset += deser_safety_mode.BytesRead();
  const auto deser_actual_tcp_acceleration
      = DeserializeKnownSizeDoubleVector(3, buffer, current_offset);
  actual_tcp_acceleration_ = deser_actual_tcp_acceleration.Value();
  current_offset += deser_actual_tcp_acceleration.BytesRead();
  const auto deser_trajectory_limiter_speed_scaling
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  trajectory_limiter_speed_scaling_
      = deser_trajectory_limiter_speed_scaling.Value();
  current_offset += deser_trajectory_limiter_speed_scaling.BytesRead();
  const auto deser_linear_momentum_norm
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  linear_momentum_norm_ = deser_linear_momentum_norm.Value();
  current_offset += deser_linear_momentum_norm.BytesRead();
  const auto deser_mainboard_voltage
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  mainboard_voltage_ = deser_mainboard_voltage.Value();
  current_offset += deser_mainboard_voltage.BytesRead();
  const auto deser_motorboard_voltage
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  motorboard_voltage_ = deser_motorboard_voltage.Value();
  current_offset += deser_motorboard_voltage.BytesRead();
  const auto deser_mainboard_current
      = DeserializeNetworkMemcpyable<double>(buffer, current_offset);
  mainboard_current_ = deser_mainboard_current.Value();
  current_offset += deser_mainboard_current.BytesRead();
  const auto deser_joint_voltage
      = DeserializeKnownSizeDoubleVector(6, buffer, current_offset);
  joint_voltage_ = deser_joint_voltage.Value();
  current_offset += deser_joint_voltage.BytesRead();
  const uint64_t bytes_read = current_offset - starting_offset;
  return bytes_read;
}

URRealtimeInterface::URRealtimeInterface(
    const std::string& robot_host,
    const std::function<void(const URRealtimeState&)>&
      state_received_callback_fn,
    const std::function<void(const std::string&)>& logging_fn)
  : state_received_callback_fn_(state_received_callback_fn),
    logging_fn_(logging_fn)
{
  connected_.store(false);
  struct hostent* nameserver = gethostbyname(robot_host.c_str());
  if (nameserver == nullptr)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to find robot at " + robot_host);
  }
  Log("Connecting to robot at " + robot_host);
  bzero(reinterpret_cast<char*>(&robot_addr_), sizeof(robot_addr_));
  robot_addr_.sin_family = AF_INET;
  bcopy(reinterpret_cast<char*>(nameserver->h_addr),
        reinterpret_cast<char*>(&robot_addr_.sin_addr.s_addr),
        static_cast<size_t>(nameserver->h_length));
  robot_addr_.sin_port = htons(30003);
}

void URRealtimeInterface::ConnectToRobot()
{
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ <= 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to create socket");
  }
  const int enable_flag = 1;
  const int setnodelay_res
      = setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<const void*>(&enable_flag), sizeof(int));
  if (setnodelay_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to enable TCP_NODELAY");
  }
  const int setquickack_res
      = setsockopt(socket_fd_, IPPROTO_TCP, TCP_QUICKACK,
                   reinterpret_cast<const void*>(&enable_flag), sizeof(int));
  if (setquickack_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to enable TCP_QUICKACK");
  }
  const int setreuseaddr_res
      = setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<const void*>(&enable_flag), sizeof(int));
  if (setreuseaddr_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to enable SO_REUSEADDR");
  }
  connect(socket_fd_, reinterpret_cast<struct sockaddr*>(&robot_addr_),
          sizeof(robot_addr_));
  // Since the socket is nonblocking
  // we must select() on it to make sure it works
  fd_set writefds;
  FD_ZERO(&writefds);
  FD_SET(socket_fd_, &writefds);
  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;
  const int select_res
      = select(socket_fd_ + 1, nullptr, &writefds, nullptr, &timeout);
  if (select_res <= 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to select");
  }
  int so_error_flag = 0;
  unsigned int flag_len = 0;
  const int getsockopt_res
      = getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &so_error_flag, &flag_len);
  if (so_error_flag < 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to check socket error");
  }
  if (getsockopt_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to getsockopt SO_ERROR");
  }
  Log("Connected to robot");
  connected_.store(true);
}

URRealtimeInterface::~URRealtimeInterface()
{
  StopRecv();
  close(socket_fd_);
}

void URRealtimeInterface::StartRecv()
{
  Log("Connecting to robot...");
  ConnectToRobot();
  Log("Starting recv thread loop...");
  running_.store(true);
  recv_thread_ = std::thread(&URRealtimeInterface::RecvLoop, this);
}

void URRealtimeInterface::StopRecv()
{
  running_.store(false);
  recv_thread_.join();
}

void URRealtimeInterface::RecvLoop()
{
  std::vector<uint8_t> recv_buffer(2048, 0x00);
  struct timeval timeout;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(socket_fd_, &readfds);
  while (running_.load())
  {
    timeout.tv_sec = 0; // Do this each loop as select modifies timeout
    timeout.tv_usec = 500000; // Set timeout of 0.5 sec
    const int select_res
        = select(socket_fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (select_res <= 0)
    {
      perror(nullptr);
      throw std::runtime_error("Failed to select");
    }
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
        connected_.store(false);
        perror(nullptr);
        throw std::runtime_error("Failed to enable TCP_QUICKACK");
      }
      try
      {
        const URRealtimeState latest_state
            = URRealtimeState::Deserialize(recv_buffer, 0).Value();
        state_received_callback_fn_(latest_state);
      }
      catch (std::runtime_error ex)
      {
        Log("Message deserialization failed for " + std::to_string(bytes_read)
            + " bytes read with error " + ex.what());
      }
    }
    else
    {
      connected_.store(false);
      close(socket_fd_);
      Log("Connection to robot failed, retrying in 10 seconds");
      while (running_.load())
      {
        try
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(10.0));
          ConnectToRobot();
          break;
        }
        catch (...)
        {
          Log("Connection to robot failed, retrying in 10 seconds");
        }
      }
    }
  }
}

bool URRealtimeInterface::SendURScriptCommand(const std::string& command)
{
  const char command_end = '\n';
  const char last_command_char
      = (command.size() > 0) ? static_cast<char>(command.back()) : '\0';
  const bool valid_command_format = (last_command_char == command_end);
  const bool connected = connected_.load();
  if (valid_command_format && connected)
  {
    const ssize_t bytes_written
        = write(socket_fd_, command.c_str(), command.size());
    if (bytes_written == static_cast<ssize_t>(command.size()))
    {
      return true;
    }
    else
    {
      perror(nullptr);
      Log("Failed to send command [" + command + "] with "
          + std::to_string(command.size()) + " bytes, sent "
          + std::to_string(bytes_written) + " instead");
      return false;
    }
  }
  else if (connected)
  {
    Log("Could not send command [" + command + "] - not correctly formatted");
    return false;
  }
  else
  {
    Log("Could not send command [" + command + "] - robot is not connected");
    return false;
  }
}
}
