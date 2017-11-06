#include <lightweight_ur_interface/ur_minimal_realtime_driver.hpp>

namespace lightweight_ur_interface
{
    uint64_t URRealtimeState::DeserializeSelf(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        uint64_t current_offset = current;
        const std::pair<int32_t, uint64_t> deserialized_length = arc_helpers::DeserializeNetworkFixedSizePOD<int32_t>(buffer, current_offset);
        const int32_t message_length = deserialized_length.first;
        current_offset += deserialized_length.second;
        if ((buffer.size() - current_offset) < (uint64_t)message_length)
        {
            const std::string msg = "Insufficient buffer to deserialize message: buffer.size()=" + std::to_string(buffer.size()) + " current_offset=" + std::to_string(current_offset) + " message_length=" + std::to_string(message_length);
            throw std::runtime_error(msg);
        }
        const std::pair<double, uint64_t> deserialized_uptime = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        controller_uptime_ = deserialized_uptime.first;
        current_offset += deserialized_uptime.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_target_position = DeserializeDoubleVector(6, buffer, current_offset);
        target_position_ = deserialized_target_position.first;
        current_offset += deserialized_target_position.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_target_velocity = DeserializeDoubleVector(6, buffer, current_offset);
        target_velocity_ = deserialized_target_velocity.first;
        current_offset += deserialized_target_velocity.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_target_acceleration = DeserializeDoubleVector(6, buffer, current_offset);
        target_acceleration_ = deserialized_target_acceleration.first;
        current_offset += deserialized_target_acceleration.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_target_current = DeserializeDoubleVector(6, buffer, current_offset);
        target_current_ = deserialized_target_current.first;
        current_offset += deserialized_target_current.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_target_torque = DeserializeDoubleVector(6, buffer, current_offset);
        target_torque_ = deserialized_target_torque.first;
        current_offset += deserialized_target_torque.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_actual_position = DeserializeDoubleVector(6, buffer, current_offset);
        actual_position_ = deserialized_actual_position.first;
        current_offset += deserialized_actual_position.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_actual_velocity = DeserializeDoubleVector(6, buffer, current_offset);
        actual_velocity_ = deserialized_actual_velocity.first;
        current_offset += deserialized_actual_velocity.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_actual_current = DeserializeDoubleVector(6, buffer, current_offset);
        actual_current_ = deserialized_actual_current.first;
        current_offset += deserialized_actual_current.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_control_current = DeserializeDoubleVector(6, buffer, current_offset);
        control_current_ = deserialized_control_current.first;
        current_offset += deserialized_control_current.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_actual_tcp_vector = DeserializeDoubleVector(6, buffer, current_offset);
        raw_actual_tcp_pose_ = deserialized_actual_tcp_vector.first;
        actual_tcp_pose_ = TcpVectorToTransform(raw_actual_tcp_pose_);
        current_offset += deserialized_actual_tcp_vector.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_actual_tcp_twist = DeserializeDoubleVector(6, buffer, current_offset);
        actual_tcp_twist_ = TcpVelocityToTwist(deserialized_actual_tcp_twist.first);
        current_offset += deserialized_actual_tcp_twist.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_actual_tcp_wrench = DeserializeDoubleVector(6, buffer, current_offset);
        actual_tcp_wrench_ = TcpForceToWrench(deserialized_actual_tcp_wrench.first);
        current_offset += deserialized_actual_tcp_wrench.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_target_tcp_vector = DeserializeDoubleVector(6, buffer, current_offset);
        raw_target_tcp_pose_ = deserialized_target_tcp_vector.first;
        target_tcp_pose_ = TcpVectorToTransform(raw_target_tcp_pose_);
        current_offset += deserialized_target_tcp_vector.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_target_tcp_twist = DeserializeDoubleVector(6, buffer, current_offset);
        target_tcp_twist_ = TcpVelocityToTwist(deserialized_target_tcp_twist.first);
        current_offset += deserialized_target_tcp_twist.second;
        const std::pair<double, uint64_t> deserialized_digital_io_bits = arc_helpers::DeserializeNetworkFixedSizePOD<uint64_t>(buffer, current_offset);
        current_offset += deserialized_digital_io_bits.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_motor_temperature = DeserializeDoubleVector(6, buffer, current_offset);
        motor_temperature_ = deserialized_motor_temperature.first;
        current_offset += deserialized_motor_temperature.second;
        const std::pair<double, uint64_t> deserialized_controller_rt_loop_time = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        controller_rt_loop_time_ = deserialized_controller_rt_loop_time.first;
        current_offset += deserialized_controller_rt_loop_time.second;
        const std::pair<double, uint64_t> deserialized_test_val = arc_helpers::DeserializeNetworkFixedSizePOD<uint64_t>(buffer, current_offset);
        current_offset += deserialized_test_val.second;
        const std::pair<double, uint64_t> deserialized_robot_mode = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        robot_mode_ = deserialized_robot_mode.first;
        current_offset += deserialized_robot_mode.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_joint_mode = DeserializeDoubleVector(6, buffer, current_offset);
        joint_mode_ = deserialized_joint_mode.first;
        current_offset += deserialized_joint_mode.second;
        const std::pair<double, uint64_t> deserialized_safety_mode = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        safety_mode_ = deserialized_safety_mode.first;
        current_offset += deserialized_safety_mode.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_actual_tcp_acceleration = DeserializeDoubleVector(3, buffer, current_offset);
        actual_tcp_acceleration_ = deserialized_actual_tcp_acceleration.first;
        current_offset += deserialized_actual_tcp_acceleration.second;
        const std::pair<double, uint64_t> deserialized_trajectory_limiter_speed_scaling = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        trajectory_limiter_speed_scaling_ = deserialized_trajectory_limiter_speed_scaling.first;
        current_offset += deserialized_trajectory_limiter_speed_scaling.second;
        const std::pair<double, uint64_t> deserialized_linear_momentum_norm = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        linear_momentum_norm_ = deserialized_linear_momentum_norm.first;
        current_offset += deserialized_linear_momentum_norm.second;
        const std::pair<double, uint64_t> deserialized_mainboard_voltage = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        mainboard_voltage_ = deserialized_mainboard_voltage.first;
        current_offset += deserialized_mainboard_voltage.second;
        const std::pair<double, uint64_t> deserialized_motorboard_voltage = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        motorboard_voltage_ = deserialized_motorboard_voltage.first;
        current_offset += deserialized_motorboard_voltage.second;
        const std::pair<double, uint64_t> deserialized_mainboard_current = arc_helpers::DeserializeNetworkFixedSizePOD<double>(buffer, current_offset);
        mainboard_current_ = deserialized_mainboard_current.first;
        current_offset += deserialized_mainboard_current.second;
        const std::pair<std::vector<double>, uint64_t> deserialized_joint_voltage = DeserializeDoubleVector(6, buffer, current_offset);
        joint_voltage_ = deserialized_joint_voltage.first;
        current_offset += deserialized_joint_voltage.second;
        const uint64_t bytes_read = current_offset - current;
        return bytes_read;
    }

    URRealtimeInterface::URRealtimeInterface(const std::string& robot_host, const std::function<void(const URRealtimeState&)>& state_received_callback_fn, const std::function<void(const std::string&)>& logging_fn) : state_received_callback_fn_(state_received_callback_fn), logging_fn_(logging_fn)
    {
        connected_.store(false);
        struct hostent* nameserver = gethostbyname(robot_host.c_str());
        if (nameserver == NULL)
        {
            perror(NULL);
            throw std::runtime_error("Failed to find robot at " + robot_host);
        }
        bzero((char*)&robot_addr_, sizeof(robot_addr_));
        robot_addr_.sin_family = AF_INET;
        bcopy((char *)nameserver->h_addr, (char *)&robot_addr_.sin_addr.s_addr, nameserver->h_length);
        robot_addr_.sin_port = htons(30003);
    }

    void URRealtimeInterface::ConnectToRobot()
    {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ <= 0)
        {
            perror(NULL);
            throw std::runtime_error("Failed to create socket");
        }
        const int enable_flag = 1;
        const int setnodelay_res = setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, (char *)&enable_flag, sizeof(int));
        if (setnodelay_res != 0)
        {
            perror(NULL);
            throw std::runtime_error("Failed to enable TCP_NODELAY");
        }
        const int setquickack_res = setsockopt(socket_fd_, IPPROTO_TCP, TCP_QUICKACK, (char *)&enable_flag, sizeof(int));
        if (setquickack_res != 0)
        {
            perror(NULL);
            throw std::runtime_error("Failed to enable TCP_QUICKACK");
        }
        const int setreuseaddr_res = setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, (char *)&enable_flag, sizeof(int));
        if (setreuseaddr_res != 0)
        {
            perror(NULL);
            throw std::runtime_error("Failed to enable SO_REUSEADDR");
        }
//        const int setnonblock_res = fcntl(socket_fd_, F_SETFL, O_NONBLOCK);
//        if (setnonblock_res != 0)
//        {
//            perror(NULL);
//            throw std::runtime_error("Failed to enable O_NONBLOCK");
//        }
        connect(socket_fd_, (struct sockaddr *)&robot_addr_, sizeof(robot_addr_));
        // Since the socket is nonblocking, we must select() on it to make sure it works
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(socket_fd_, &writefds);
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        const int select_res = select(socket_fd_ + 1, NULL, &writefds, NULL, &timeout);
        if (select_res <= 0)
        {
            perror(NULL);
            throw std::runtime_error("Failed to select");
        }
        int so_error_flag = 0;
        unsigned int flag_len = 0;
        const int getsockopt_res = getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &so_error_flag, &flag_len);
        if (so_error_flag < 0)
        {
            perror(NULL);
            throw std::runtime_error("Failed to check socket error");
        }
        if (getsockopt_res != 0)
        {
            perror(NULL);
            throw std::runtime_error("Failed to getsockopt SO_ERROR");
        }
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
            const int select_res = select(socket_fd_ + 1, &readfds, NULL, NULL, &timeout);
            if (select_res <= 0)
            {
                perror(NULL);
                throw std::runtime_error("Failed to select");
            }
            const ssize_t bytes_read = read(socket_fd_, recv_buffer.data(), recv_buffer.size());
            if (bytes_read > 0)
            {
                const int enable_flag = 1;
                const int setquickack_res = setsockopt(socket_fd_, IPPROTO_TCP, TCP_QUICKACK, (char *)&enable_flag, sizeof(int));
                if (setquickack_res != 0)
                {
                    connected_.store(false);
                    perror(NULL);
                    throw std::runtime_error("Failed to enable TCP_QUICKACK");
                }
                try
                {
                    const URRealtimeState latest_state = URRealtimeState::Deserialize(recv_buffer, 0);
                    state_received_callback_fn_(latest_state);
                }
                catch (std::runtime_error ex)
                {
                    Log("Message deserialization failed for " + std::to_string(bytes_read) + " bytes read with error " + ex.what());
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
        const char last_command_char = (command.size() > 0) ? (char)(command.back()) : '\0';
        const bool valid_command_format = (last_command_char == command_end);
        const bool connected = connected_.load();
        if (valid_command_format && connected)
        {
            const ssize_t bytes_written = write(socket_fd_, command.c_str(), command.size());
            if (bytes_written == (ssize_t)command.size())
            {
                return true;
            }
            else
            {
                perror(NULL);
                Log("Failed to send command [" + command + "] with " + std::to_string(command.size()) + " bytes, sent " + std::to_string(bytes_written) + " instead");
                return false;
            }
        }
        else if (connected)
        {
            Log("Could not send command [" + command + "] as it is not correctly formatted");
            return false;
        }
        else
        {
            Log("Could not send command [" + command + "] as robot is not connected");
            return false;
        }
    }
}
