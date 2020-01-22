#include <robotiq_ft_driver/robotiq_ft_driver.hpp>
#include <stdint.h>

namespace robotiq_ft_driver
{
RobotiqFTModbusRtuInterface::RobotiqFTModbusRtuInterface(
    const std::function<void(const std::string&)>& logging_fn,
    const std::string& modbus_rtu_interface,
    const uint16_t sensor_slave_id)
  : logging_fn_(logging_fn)
{
  const int baud_rate = 19200;
  const int data_bits = 8;
  const int stop_bits = 1;
  const char parity = 'N';
  modbus_interface_ptr_ = nullptr;
  modbus_interface_ptr_ = modbus_new_rtu(modbus_rtu_interface.c_str(),
                                         baud_rate,
                                         parity,
                                         data_bits,
                                         stop_bits);
  if (modbus_interface_ptr_ == nullptr)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to create Modbus RTU interface to "
                             + modbus_rtu_interface + " with error: "
                             + error_msg);
  }
  const int mss_ret = modbus_set_slave(modbus_interface_ptr_,
                                       (int)sensor_slave_id);
  if (mss_ret != 0)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Invalid Modbus slave address "
                             + std::to_string(sensor_slave_id)
                             + " with error: " + error_msg);
  }
  const int mc_ret = modbus_connect(modbus_interface_ptr_);
  if (mc_ret != 0)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to connect with error: " + error_msg);
  }
}

RobotiqFTModbusRtuInterface::~RobotiqFTModbusRtuInterface()
{
  ShutdownConnection();
}

Eigen::Matrix<double, 6, 1> RobotiqFTModbusRtuInterface::GetCurrentForceTorque()
{
  std::vector<uint16_t> raw_ft_buffer(6, 0x00);
  const int ret = modbus_read_registers(modbus_interface_ptr_,
                                        FX_REGISTER,
                                        6,
                                        raw_ft_buffer.data());
  if (ret != 6)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to read F/T registers with error: "
                             + error_msg);
  }
  int16_t raw_fx = 0;
  int16_t raw_fy = 0;
  int16_t raw_fz = 0;
  int16_t raw_tx = 0;
  int16_t raw_ty = 0;
  int16_t raw_tz = 0;
  memcpy(&raw_fx, &raw_ft_buffer[0], sizeof(int16_t));
  memcpy(&raw_fy, &raw_ft_buffer[1], sizeof(int16_t));
  memcpy(&raw_fz, &raw_ft_buffer[2], sizeof(int16_t));
  memcpy(&raw_tx, &raw_ft_buffer[3], sizeof(int16_t));
  memcpy(&raw_ty, &raw_ft_buffer[4], sizeof(int16_t));
  memcpy(&raw_tz, &raw_ft_buffer[5], sizeof(int16_t));
  Eigen::Matrix<double, 6, 1> ft;
  ft(0, 0) = (double)raw_fx / 100.0;
  ft(1, 0) = (double)raw_fy / 100.0;
  ft(2, 0) = (double)raw_fz / 100.0;
  ft(3, 0) = (double)raw_tx / 1000.0;
  ft(4, 0) = (double)raw_ty / 1000.0;
  ft(5, 0) = (double)raw_tz / 1000.0;
  return ft;
}

Eigen::Vector3d RobotiqFTModbusRtuInterface::GetCurrentAcceleration()
{
  std::vector<uint16_t> raw_acc_buffer(3, 0x00);
  const int ret = modbus_read_registers(modbus_interface_ptr_,
                                        AX_REGISTER,
                                        3,
                                        raw_acc_buffer.data());
  if (ret != 3)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error(
          "Failed to read Accelerometer registers with error: " + error_msg);
  }
  int16_t raw_ax = 0;
  int16_t raw_ay = 0;
  int16_t raw_az = 0;
  memcpy(&raw_ax, &raw_acc_buffer[0], sizeof(int16_t));
  memcpy(&raw_ay, &raw_acc_buffer[1], sizeof(int16_t));
  memcpy(&raw_az, &raw_acc_buffer[2], sizeof(int16_t));
  Eigen::Vector3d acc;
  acc.x() = (double)raw_ax / 1000.0;
  acc.y() = (double)raw_ay / 1000.0;
  acc.z() = (double)raw_az / 1000.0;
  acc = acc * 9.806;
  return acc;
}

std::string RobotiqFTModbusRtuInterface::ReadSerialNumber()
{
  std::vector<uint16_t> raw_sn_buffer(4, 0x00);
  const int ret = modbus_read_registers(modbus_interface_ptr_,
                                        SN1_REGISTER,
                                        4,
                                        raw_sn_buffer.data());
  if (ret != 4)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error(
          "Failed to read Serial Number registers with error: " + error_msg);
  }
  const uint64_t raw_sn 
      = (uint64_t)(raw_sn_buffer[3] << 16) + (uint64_t)raw_sn_buffer[2];
  if (raw_sn > 0)
  {
    const int max_str_len = 128;
    char sn_str_buffer[max_str_len];
    const int written = snprintf(sn_str_buffer,
                                 max_str_len - 1,
                                 "%c%c%c-%.4lu",
                                 (char)(raw_sn_buffer[0] >> 8),
                                 (char)(raw_sn_buffer[0] & 0xff),
                                 (char)(raw_sn_buffer[1] >> 8), raw_sn);
    if (written <= 0)
    {
      throw std::runtime_error("written <= 0");
    }
    else if (written >= max_str_len)
    {
      throw std::runtime_error("written >= max_str_len");
    }
    return std::string(sn_str_buffer);
  }
  else
  {
    return std::string("UNKNOWN");
  }
}

std::string RobotiqFTModbusRtuInterface::ReadFirmwareVersion()
{
  std::vector<uint16_t> raw_fw_buffer(3, 0x00);
  const int ret = modbus_read_registers(modbus_interface_ptr_,
                                        FW1_REGISTER,
                                        3,
                                        raw_fw_buffer.data());
  if (ret != 3)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to read Firmware registers with error: "
                             + error_msg);
  }
  const int max_str_len = 128;
  char fw_str_buffer[max_str_len];
  const int written
      = snprintf(fw_str_buffer, max_str_len - 1,
                 "%c%c%c-%hhu.%hhu.%hhu",
                 (char)(raw_fw_buffer[0] >> 8),
                 (char)(raw_fw_buffer[0] & 0xff),
                 (char)(raw_fw_buffer[1] >> 8),
                 (char)(raw_fw_buffer[1] & 0xff),
                 (char)(raw_fw_buffer[2] >> 8),
                 (char)(raw_fw_buffer[2] & 0xff));
  if (written <= 0)
  {
    throw std::runtime_error("written <= 0");
  }
  else if (written >= max_str_len)
  {
    throw std::runtime_error("written >= max_str_len");
  }
  return std::string(fw_str_buffer);
}

uint16_t RobotiqFTModbusRtuInterface::ReadYearOfManufacture()
{
  uint16_t yom;
  const int ret
      = modbus_read_registers(modbus_interface_ptr_, YOM_REGISTER, 1, &yom);
  if (ret != 1)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to read YOM register with error: "
                             + error_msg);
  }
  return yom;
}

void RobotiqFTModbusRtuInterface::ShutdownConnection()
{
  Log("Closing modbus connection...");
  modbus_close(modbus_interface_ptr_);
  modbus_free(modbus_interface_ptr_);
  Log("...finished cleanup");
}
}
