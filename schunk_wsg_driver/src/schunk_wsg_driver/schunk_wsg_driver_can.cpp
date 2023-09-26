#include <schunk_wsg_driver/schunk_wsg_driver_can.hpp>
#include <unistd.h>

namespace schunk_wsg_driver
{
WSGCANInterface::WSGCANInterface(
    const std::function<void(const std::string&)>& logging_fn,
    const std::string& socketcan_interface,
    const uint32_t gripper_send_can_id)
  : WSGInterface(logging_fn),
    gripper_send_can_id_(gripper_send_can_id),
    gripper_recv_can_id_(gripper_send_can_id + 1u)
{
  Log("Attempting to create WSG gripper CAN interface with socketcan interface "
      + socketcan_interface + " gripper base_can_id "
      + std::to_string(gripper_send_can_id));
  // Make the socketcan socket
  can_socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_fd_ <= 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to create socketcan socket");
  }
  // Locate the desired socketcan interface
  struct ifreq interface;
  // Figure out how much we can write
  const size_t max_ifr_name_length = IFNAMSIZ - 1;
  strncpy(interface.ifr_name, socketcan_interface.c_str(), max_ifr_name_length);
  interface.ifr_name[IFNAMSIZ - 1] = 0x00; // Make sure it is null-terminated
  // Invoke ioctl to find the index of the interface
  ioctl(can_socket_fd_, SIOCGIFINDEX, &interface);
  // Set interface options
  struct can_filter filter[1];
  filter[0].can_id   = gripper_recv_can_id_;
  filter[0].can_mask = CAN_SFF_MASK;
  const int setsockopt_can_result
      = setsockopt(can_socket_fd_,
                   SOL_CAN_RAW,
                   CAN_RAW_FILTER,
                   &filter,
                   sizeof(filter));
  if (setsockopt_can_result != 0)
  {
    perror(nullptr);
    throw std::runtime_error("setsockopt CAN configuration failed");
  }
  struct timeval read_timeout;
  read_timeout.tv_sec = 1;
  read_timeout.tv_usec = 0;
  const int setsockopt_timeout_result
      = setsockopt(can_socket_fd_,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   &read_timeout,
                   sizeof(read_timeout));
  if (setsockopt_timeout_result != 0)
  {
    perror(nullptr);
    throw std::runtime_error("setsockopt timeout configuration failed");
  }
  // Bind the socket to the interface
  struct sockaddr_can can_interface;
  can_interface.can_family = AF_CAN;
  can_interface.can_ifindex = interface.ifr_ifindex;
  const int bind_result = bind(
      can_socket_fd_, reinterpret_cast<struct sockaddr*>(&can_interface),
      sizeof(can_interface));
  if (bind_result != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to bind socketcan socket");
  }
  // Start receive thread
  active_.store(true);
  recv_thread_
      = std::thread(std::bind(&WSGCANInterface::RecvFromGripper, this));
}

WSGCANInterface::~WSGCANInterface()
{
  if (active_.load())
  {
    Shutdown();
  }
}

void WSGCANInterface::ShutdownConnection()
{
  Log("Starting shutdown...");
  // Stop recv thread
  active_.store(false);
  Log("Waiting for recv thread to terminate...");
  recv_thread_.join();
  // Clean up socket
  close(can_socket_fd_);
  Log("...finished cleanup");
}

bool WSGCANInterface::CommandGripper(const WSGRawCommandMessage& command)
{
  std::vector<uint8_t> serialized_command_buffer;
  WSGRawCommandMessage::Serialize(command, serialized_command_buffer);
  // Diagnostic info
  const size_t serialized_command_size = serialized_command_buffer.size();
  const size_t num_frames
      = static_cast<size_t>(
          ceil(static_cast<double>(serialized_command_size)
                  / static_cast<double>(CAN_MAX_DLEN)));
  struct can_frame frame;
  frame.can_id = gripper_send_can_id_;
  for (size_t frame_num = 0; frame_num < num_frames; frame_num++)
  {
    // Zero the frame data
    memset(frame.data, 0, static_cast<size_t>(CAN_MAX_DLEN));
    const size_t starting_offset
        = frame_num * static_cast<size_t>(CAN_MAX_DLEN);
    const size_t bytes_to_write
        = std::min(static_cast<size_t>(CAN_MAX_DLEN),
                   serialized_command_size - starting_offset);
    frame.can_dlc = static_cast<uint8_t>(bytes_to_write);
    memcpy(frame.data,
           serialized_command_buffer.data() + starting_offset,
           bytes_to_write);
    const ssize_t bytes_sent = write(can_socket_fd_, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame))
    {
      return false;
    }
  }
  return true;
}

void WSGCANInterface::RecvFromGripper()
{
  std::vector<uint8_t> recv_buffer;
  while (active_.load())
  {
    struct can_frame frame;
    const ssize_t read_size = read(can_socket_fd_, &frame, CAN_MTU);
    if (read_size == CAN_MTU)
    {
      if (frame.can_dlc == CAN_MAX_DLEN)
      {
        recv_buffer.insert(recv_buffer.end(),
                           frame.data,
                           frame.data + frame.can_dlc);
      }
      else if (frame.can_dlc < CAN_MAX_DLEN)
      {
        recv_buffer.insert(recv_buffer.end(),
                           frame.data,
                           frame.data + frame.can_dlc);
        const auto deserialized_status_msg
            = WSGRawStatusMessage::Deserialize(recv_buffer, 0);
        if (deserialized_status_msg.BytesRead() < recv_buffer.size())
        {
          recv_buffer.erase(
              recv_buffer.begin(),
              recv_buffer.begin()
                  + static_cast<ssize_t>(deserialized_status_msg.BytesRead()));
        }
        else
        {
          recv_buffer.clear();
        }
        AppendToStatusQueue(deserialized_status_msg.Value());
      }
      else
      {
        throw std::runtime_error("Invalid frame.can_dlc size");
      }
    }
    else if (read_size < 0)
    {
      if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
      else
      {
        throw std::runtime_error("Error in recv");
      }
    }
    else
    {
      throw std::runtime_error("Read size != CAN_MTU");
    }
  }
}
}
