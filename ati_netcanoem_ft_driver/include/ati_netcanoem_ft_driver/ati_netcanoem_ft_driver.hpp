#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
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

namespace ati_netcanoem_ft_driver
{
class AtiNetCanOemInterface
{
private:

  int can_socket_fd_;
  uint8_t sensor_base_can_id_;
  std::function<void(const std::string&)> logging_fn_;
  bool has_active_calibration_;
  Eigen::Matrix<double, 6, 6> active_calibration_matrix_;
  Eigen::Matrix<double, 6, 1> active_force_torque_counts_vector_;

  enum OPCODE : uint8_t { READ_SG_A=0x0,
                          READ_SG_B=0x1,
                          READ_MATRIX_ROW_A=0x2,
                          READ_MATRIX_ROW_B=0x3,
                          READ_MATRIX_ROW_C=0x4,
                          READ_SERIAL_NUMBER=0x5,
                          SET_ACTIVE_CALIBRATION=0x6,
                          READ_COUNTS_PER_UNIT=0x7,
                          READ_UNIT_CODES=0x8,
                          READ_ADC_VOLTAGES=0x9,
                          RESET=0xc,
                          SET_BASE_ID_BITS=0xd,
                          SET_BAUD_RATE=0xe,
                          READ_FIRMWARE_VERSION=0xf };

  enum STATUS_CODE_BITMASK : uint16_t { WATCHDOG_RESET=(1 << 0),
                                        DAC_ADC_CHECK_TOO_HIGH=(1 << 1),
                                        DAC_ADC_CHECK_TOO_LOW=(1 << 2),
                                        FAKE_GND_OUT_OF_RANGE=(1 << 3),
                                        SUPPLY_VOLTAGE_TOO_HIGH=(1 << 4),
                                        SUPPLY_VOLTAGE_TOO_LOW=(1 << 5),
                                        BAD_ACTIVE_CALIBRATION=(1 << 6),
                                        EEPROM_FAILURE=(1 << 7),
                                        INVALID_CONFIGURATION=(1 << 8),
                                        TEMP_TOO_HIGH=(1 << 11),
                                        TEMP_TOO_LOW=(1 << 12),
                                        CAN_BUS_ERROR=(1 << 14),
                                        ANY_ERROR=(1 << 15) };

  class DataElement
  {
  private:

    uint8_t opcode_;
    std::vector<uint8_t> payload_;

  public:

    DataElement(const uint8_t opcode, const std::vector<uint8_t>& payload)
    {
      if (opcode > 0x0F)
      {
        throw std::invalid_argument("Invalid opcode > 0x0F");
      }
      opcode_ = opcode;
      if (payload.size() > 8)
      {
        throw std::invalid_argument("Payload > 8 bytes is too long");
      }
      payload_ = payload;
    }

    DataElement(const uint8_t opcode)
    {
      if (opcode > 0x0F)
      {
        throw std::invalid_argument("Invalid opcode > 0x0F");
      }
      opcode_ = opcode;
      payload_.clear();
    }

    inline void AppendToPayload(const uint8_t value)
    {
      if (payload_.size() < 8)
      {
        payload_.push_back(value);
      }
      else
      {
        throw std::invalid_argument("Attempted to grow payload > 8 bytes");
      }
    }

    inline uint8_t Opcode() const { return opcode_; }

    inline const std::vector<uint8_t>& Payload() const { return payload_; }
  };

public:

  AtiNetCanOemInterface(
      const std::function<void(const std::string&)>& logging_fn,
      const std::string& socketcan_interface,
      const uint8_t sensor_base_can_id);

  ~AtiNetCanOemInterface();

  inline void Log(const std::string& message) { logging_fn_(message); }

  Eigen::Matrix<double, 6, 1> GetCurrentForceTorque();

  std::pair<uint16_t, Eigen::Matrix<double, 6, 1>> ReadRawStrainGaugeData();

  bool LoadNewActiveCalibration(const uint8_t calibration);

  Eigen::Matrix<double, 6, 6> ReadActiveCalibrationMatrix();

  Eigen::Matrix<double, 1, 6> ReadActiveCalibrationMatrixRow(const uint8_t row);

  std::string ReadSerialNumber();

  bool SetActiveCalibration(const uint8_t calibration);

  std::pair<uint32_t, uint32_t> ReadCountsPerUnit();

  std::pair<uint8_t, uint8_t> ReadUnitCodes();

  std::vector<uint16_t> ReadDiagnosticADCVoltages();

  void ResetSensor();

  bool SetSensorBaseCanID(const uint8_t new_base_can_id);

  bool SetSensorCanRate(const uint8_t rate_divisor);

  std::pair<std::pair<uint8_t, uint8_t>, uint16_t> ReadFirmwareVersion();

private:

  std::vector<DataElement> SendFrameAndAwaitResponse(
      const DataElement& command,
      const uint8_t num_response_frames,
      const double timeout);

  void ShutdownConnection();

  void ParseStatusCode(const uint16_t status_code);
};
}
