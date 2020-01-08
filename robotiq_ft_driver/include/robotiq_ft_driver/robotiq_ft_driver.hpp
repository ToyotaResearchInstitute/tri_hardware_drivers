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
#include <functional>
#include <atomic>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <Eigen/Geometry>
#include <modbus/modbus.h>

namespace robotiq_ft_driver
{
class RobotiqFTModbusRtuInterface
{
private:

  modbus_t* modbus_interface_ptr_;
  std::function<void(const std::string&)> logging_fn_;

  enum REGISTERS : uint16_t { FX_REGISTER=180,
                              FY_REGISTER=181,
                              FZ_REGISTER=182,
                              TX_REGISTER=183,
                              TY_REGISTER=184,
                              TZ_REGISTER=185,
                              AX_REGISTER=190,
                              AY_REGISTER=191,
                              AZ_REGISTER=192,
                              FW1_REGISTER=500,
                              FW2_REGISTER=501,
                              FW3_REGISTER=502,
                              SN1_REGISTER=510,
                              SN2_REGISTER=511,
                              SN3_REGISTER=512,
                              SN4_REGISTER=513,
                              YOM_REGISTER=514 };

public:

  RobotiqFTModbusRtuInterface(
      const std::function<void(const std::string&)>& logging_fn,
      const std::string& modbus_rtu_interface,
      const uint16_t sensor_slave_id);

  ~RobotiqFTModbusRtuInterface();

  inline void Log(const std::string& message) { logging_fn_(message); }

  Eigen::Matrix<double, 6, 1> GetCurrentForceTorque();

  Eigen::Vector3d GetCurrentAcceleration();

  std::string ReadSerialNumber();

  std::string ReadFirmwareVersion();

  uint16_t ReadYearOfManufacture();

private:

  void ShutdownConnection();
};
}
