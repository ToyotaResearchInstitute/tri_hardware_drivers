#include <schunk_wsg_driver/schunk_wsg_driver_ethernet.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace schunk_wsg_driver
{
WSGUDPInterface::WSGUDPInterface(
    const std::function<void(const std::string&)>& logging_fn,
    const std::string& gripper_ip_address,
    const uint16_t gripper_port,
    const uint16_t local_port)
  : WSGInterface(logging_fn)
{
  Log("Attempting to create WSG gripper UDP interface with gripper IP "
      + gripper_ip_address + " gripper port " + std::to_string(gripper_port)
      + " local port " + std::to_string(local_port));
  // Make the sockets
  send_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_socket_fd_ <= 0)
  {
    perror(NULL);
    throw std::runtime_error("Failed to create send socket");
  }
  recv_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (recv_socket_fd_ <= 0)
  {
    perror(NULL);
    throw std::runtime_error("Failed to create recv socket");
  }
  // Make the local sockaddr
  local_sockaddr_.sin_addr.s_addr = INADDR_ANY;
  local_sockaddr_.sin_family = AF_INET;
  local_sockaddr_.sin_port = htons(local_port);
  // Make the gripper sockaddr
  gripper_sockaddr_.sin_addr.s_addr = inet_addr(gripper_ip_address.c_str());
  gripper_sockaddr_.sin_family = AF_INET;
  gripper_sockaddr_.sin_port = htons(gripper_port);
  // Bind the local socket
  const int bind_result = bind(recv_socket_fd_,
                               (struct sockaddr *)&local_sockaddr_,
                               sizeof(local_sockaddr_));
  if (bind_result != 0)
  {
    perror(NULL);
    throw std::runtime_error("Failed to bind recv socket");
  }
  // Start receive thread
  active_.store(true);
  recv_thread_
      = std::thread(std::bind(&WSGUDPInterface::RecvFromGripper, this));
}

WSGUDPInterface::~WSGUDPInterface()
{
  if (active_.load())
  {
    Shutdown();
  }
}

void WSGUDPInterface::ShutdownConnection()
{
  Log("Starting shutdown...");
  // Stop recv thread
  active_.store(false);
  Log("Waiting for recv thread to terminate...");
  recv_thread_.join();
  // Clean up sockets
  close(send_socket_fd_);
  close(recv_socket_fd_);
  Log("...finished cleanup");
}

bool WSGUDPInterface::CommandGripper(const WSGRawCommandMessage& command)
{
  std::vector<uint8_t> serialized_command_buffer;
  WSGRawCommandMessage::Serialize(command, serialized_command_buffer);
  const ssize_t send_result = sendto(send_socket_fd_,
                                     serialized_command_buffer.data(),
                                     serialized_command_buffer.size(),
                                     0,
                                     (struct sockaddr*)&gripper_sockaddr_,
                                     sizeof(gripper_sockaddr_));
  if (send_result == (ssize_t)serialized_command_buffer.size())
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<WSGRawStatusMessage> WSGUDPInterface::GetStatusQueue()
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  const std::vector<WSGRawStatusMessage> status_queue = status_queue_;
  status_queue_.clear();
  return status_queue;
}

void WSGUDPInterface::RecvFromGripper()
{
  std::vector<uint8_t> recv_buffer(1024, 0x00);
  struct sockaddr_storage source_sockaddr;
  socklen_t source_sockaddr_len = sizeof(source_sockaddr);
  while (active_.load())
  {
    const ssize_t read_size = recvfrom(recv_socket_fd_,
                                       recv_buffer.data(),
                                       recv_buffer.size(),
                                       MSG_DONTWAIT,
                                       (struct sockaddr*)&source_sockaddr,
                                       &source_sockaddr_len);
    if (read_size < 0)
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
    else if (read_size == (ssize_t)recv_buffer.size())
    {
      throw std::runtime_error("Received message larger than recv buffer");
    }
    else
    {
      std::vector<uint8_t> serialized_message;
      serialized_message.insert(serialized_message.end(),
                                recv_buffer.begin(),
                                recv_buffer.begin() + read_size);
      const WSGRawStatusMessage status_msg
          = WSGRawStatusMessage::Deserialize(serialized_message, 0).first;
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_queue_.push_back(status_msg);
    }
  }
}
}
