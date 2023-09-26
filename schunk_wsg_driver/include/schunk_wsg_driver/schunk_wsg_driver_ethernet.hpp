#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <array>
#include <map>
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <netinet/in.h>
#include <schunk_wsg_driver/schunk_wsg_driver_common.hpp>

namespace schunk_wsg_driver
{
class WSGUDPInterface : public WSGInterface
{
private:

  int send_socket_fd_;
  int recv_socket_fd_;
  struct sockaddr_in local_sockaddr_;
  struct sockaddr_in gripper_sockaddr_;
  std::thread recv_thread_;
  std::atomic<bool> active_;

public:

  WSGUDPInterface(const std::function<void(const std::string&)>& logging_fn,
                  const std::string& gripper_ip_address,
                  const uint16_t gripper_port,
                  const uint16_t local_port);

  ~WSGUDPInterface();

protected:

  void RecvFromGripper();

  virtual bool CommandGripper(const WSGRawCommandMessage& command);

  virtual void ShutdownConnection();
};
}
