#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <array>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <schunk_wsg_driver/schunk_wsg_driver_common.hpp>

namespace schunk_wsg_driver
{
class WSGCANInterface : public WSGInterface
{
private:

  int can_socket_fd_;
  uint32_t gripper_send_can_id_;
  uint32_t gripper_recv_can_id_;
  std::thread recv_thread_;
  std::atomic<bool> active_;
  std::mutex status_mutex_;
  std::vector<WSGRawStatusMessage> status_queue_;

public:

  WSGCANInterface(const std::function<void(const std::string&)>& logging_fn,
                  const std::string& socketcan_interface,
                  const uint32_t gripper_send_can_id);

  ~WSGCANInterface();

protected:

  void RecvFromGripper();

  virtual bool CommandGripper(const WSGRawCommandMessage& command);

  virtual std::vector<WSGRawStatusMessage> GetStatusQueue();

  virtual void ShutdownConnection();
};
}
