#include <schunk_wsg_driver/schunk_wsg_driver_ethernet.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
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
    perror(nullptr);
    throw std::runtime_error("Failed to create send socket");
  }
  recv_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (recv_socket_fd_ <= 0)
  {
    perror(nullptr);
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
  const int bind_result = bind(
      recv_socket_fd_, reinterpret_cast<struct sockaddr*>(&local_sockaddr_),
      sizeof(local_sockaddr_));
  if (bind_result != 0)
  {
    perror(nullptr);
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
  const ssize_t send_result = sendto(
      send_socket_fd_, serialized_command_buffer.data(),
      serialized_command_buffer.size(), 0,
      reinterpret_cast<struct sockaddr*>(&gripper_sockaddr_),
      sizeof(gripper_sockaddr_));
  if (send_result == static_cast<ssize_t>(serialized_command_buffer.size()))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void WSGUDPInterface::RecvFromGripper()
{
  std::vector<uint8_t> recv_buffer(1024, 0x00);
  struct sockaddr_storage source_sockaddr;
  socklen_t source_sockaddr_len = sizeof(source_sockaddr);
  while (active_.load())
  {
    const ssize_t read_size = recvfrom(
        recv_socket_fd_, recv_buffer.data(), recv_buffer.size(), MSG_DONTWAIT,
        reinterpret_cast<struct sockaddr*>(&source_sockaddr),
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
    else if (read_size == static_cast<ssize_t>(recv_buffer.size()))
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
          = WSGRawStatusMessage::Deserialize(serialized_message, 0).Value();
      AppendToStatusQueue(status_msg);
    }
  }
}

WSGTCPInterface::WSGTCPInterface(
    const std::function<void(const std::string&)>& logging_fn,
    const std::string& gripper_ip_address,
    const uint16_t gripper_port)
  : WSGInterface(logging_fn)
{
  Log("Attempting to create WSG gripper TCP interface with gripper IP "
      + gripper_ip_address + " gripper port " + std::to_string(gripper_port));

  struct hostent* nameserver = gethostbyname(gripper_ip_address.c_str());
  if (nameserver == nullptr)
  {
    perror(nullptr);
    throw std::runtime_error(
        "Failed to find gripper at [" + gripper_ip_address + "]");
  }

  struct sockaddr_in gripper_addr;
  std::memset(&gripper_addr, 0, sizeof(gripper_addr));
  gripper_addr.sin_family = AF_INET;
  std::memcpy(
      &gripper_addr.sin_addr.s_addr,
      nameserver->h_addr,
      static_cast<size_t>(nameserver->h_length));
  gripper_addr.sin_port = htons(static_cast<uint16_t>(gripper_port));

  // Make the sockets
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ <= 0)
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
      socket_fd_, reinterpret_cast<struct sockaddr*>(&gripper_addr),
      sizeof(gripper_addr));
  if (connect_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to connect");
  }

  // Start receive thread
  active_.store(true);
  recv_thread_
      = std::thread(std::bind(&WSGTCPInterface::RecvFromGripper, this));
}

WSGTCPInterface::~WSGTCPInterface()
{
  if (active_.load())
  {
    Shutdown();
  }
}

void WSGTCPInterface::ShutdownConnection()
{
  Log("Starting shutdown...");
  // Stop recv thread
  active_.store(false);
  Log("Waiting for recv thread to terminate...");
  recv_thread_.join();
  Log("Close socket...");
  // TODO(eric.cousineau): Calling shutdown(socket_fd_, SHUT_RDWR) here causes
  // the gripper to stop. This would not be a problem, but I (Eric) cannot seem
  // to get the gripper to restart. Instead, we close the socket.
  const int close_res = close(socket_fd_);
  if (close_res != 0)
  {
    perror(nullptr);
    throw std::runtime_error("Failed to close socket");
  }
  socket_fd_ = -1;
  Log("...finished cleanup");
}

bool WSGTCPInterface::CommandGripper(const WSGRawCommandMessage& command)
{
  std::vector<uint8_t> serialized_command_buffer;
  WSGRawCommandMessage::Serialize(command, serialized_command_buffer);
  const ssize_t send_result = write(
      socket_fd_, serialized_command_buffer.data(),
      serialized_command_buffer.size());
  if (send_result == static_cast<ssize_t>(serialized_command_buffer.size()))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void WSGTCPInterface::RecvFromGripper()
{
  std::vector<uint8_t> recv_buffer(1024, 0x00);
  while (active_.load())
  {
    const ssize_t read_size = read(
        socket_fd_, recv_buffer.data(), recv_buffer.size());
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
    else if (read_size == static_cast<ssize_t>(recv_buffer.size()))
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
          = WSGRawStatusMessage::Deserialize(serialized_message, 0).Value();
      AppendToStatusQueue(status_msg);
    }
  }
}

}
