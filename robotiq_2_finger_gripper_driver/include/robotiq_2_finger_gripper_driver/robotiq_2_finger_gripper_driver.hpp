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
#include <modbus/modbus.h>
#include <sstream>

namespace robotiq_2_finger_gripper_driver
{
class Robotiq2FingerGripperStatus
{
public:

  enum FAULT_STATUS : uint8_t { NO_FAULT=0x00,
                                PRIORITY_REACTIVATION_REQUIRED=0x05,
                                PRIORITY_ACTIVATION_BIT_NOT_SET=0x07,
                                MINOR_OVER_TEMP=0x08,
                                MAJOR_UNDERVOLTAGE_LOCKOUT=0x0a,
                                MAJOR_AUTORELEASE_IN_PROGRESS=0x0b,
                                MAJOR_PROCESSOR_FAULT=0x0c,
                                MAJOR_ACTIVATION_FAILED=0x0d,
                                MAJOR_OVERCURRENT_LOCKOUT=0x0e,
                                MAJOR_AUTORELEASE_COMPLETE=0x0f };

  enum STATUS_BITMASKS : uint8_t { GACT_MASK=0b00000001,
                                   GGTO_MASK=0b00001000,
                                   GSTA_MASK=0b00110000,
                                   GOBJ_MASK=0b11000000,
                                   GFLT_MASK=0b00001111,
                                   KFLT_MASK=0b11110000 };

  enum REGISTER_MASKS : uint16_t { GRIPPER_STATUS_MASK=0x00ff,
                                   FAULT_STATUS_MASK=0x00ff,
                                   POSITION_REQUEST_MASK=0xff00,
                                   POSITION_MASK=0x00ff,
                                   CURRENT_MASK=0xff00 };

  enum GRIPPER_ACTIVATION : uint8_t { GRIPPER_RESET=0x00,
                                      GRIPPER_ACTIVATED=0x01 };

  enum ACTION_STATUS : uint8_t { GRIPPER_STOPPED=0x00,
                                 GRIPPER_GOTO=0x01 };

  enum GRIPPER_STATUS : uint8_t { GRIPPER_IN_RELEASE=0x00,
                                  ACTIVATION_IN_PROGRESS=0x01,
                                  ACTIVATION_COMPLETED=0x03 };

  enum OBJECT_STATUS : uint8_t { FINGERS_IN_MOTION=0x00,
                                 FINGERS_STOPPED_CONTACT_OPENING=0x01,
                                 FINGERS_STOPPED_CONTACT_CLOSING=0x02,
                                 FINGERS_AT_REQUESTED=0x03,
                                 FINGERS_STOPPED=0x04 };

private:

  double target_position_;
  double actual_position_;
  double actual_current_;
  GRIPPER_ACTIVATION gripper_activation_;
  ACTION_STATUS action_status_;
  GRIPPER_STATUS gripper_status_;
  OBJECT_STATUS object_status_;
  FAULT_STATUS fault_status_;

public:

  Robotiq2FingerGripperStatus(const std::vector<uint8_t>& received_bytes)
  {
    if (received_bytes.size() != 6)
    {
      throw std::invalid_argument("received_bytes.size() != 6");
    }
    const uint8_t raw_gripper_status_byte = received_bytes[0];
    gripper_activation_
        = (GRIPPER_ACTIVATION)(raw_gripper_status_byte & GACT_MASK);
    action_status_
        = (ACTION_STATUS)((raw_gripper_status_byte & GGTO_MASK) >> 3);
    gripper_status_
        = (GRIPPER_STATUS)((raw_gripper_status_byte & GSTA_MASK) >> 4);
    object_status_
        = (OBJECT_STATUS)((raw_gripper_status_byte & GOBJ_MASK) >> 6);
    if ((object_status_ == FINGERS_IN_MOTION)
        && (action_status_ == GRIPPER_STOPPED))
    {
      object_status_ = FINGERS_STOPPED;
    }
    const uint8_t raw_fault_status_byte = received_bytes[2];
    fault_status_ = (FAULT_STATUS)(raw_fault_status_byte & GFLT_MASK);
    const uint8_t raw_target_position = received_bytes[3];
    const uint8_t raw_actual_position = received_bytes[4];
    const uint8_t raw_actual_current = received_bytes[5];
    target_position_ = (double)raw_target_position / 255.0;
    actual_position_ = (double)raw_actual_position / 255.0;
    actual_current_ = (double)raw_actual_current / 255.0;
  }

  inline GRIPPER_ACTIVATION GripperActivation() const
  { return gripper_activation_; }

  inline ACTION_STATUS ActionStatus() const
  { return action_status_; }

  inline GRIPPER_STATUS GripperStatus() const
  { return gripper_status_; }

  inline OBJECT_STATUS ObjectStatus() const
  { return object_status_; }

  inline FAULT_STATUS FaultStatus() const
  { return fault_status_; }

  inline double TargetPosition() const
  { return target_position_; }

  inline double ActualPosition() const
  { return actual_position_; }

  inline double ActualCurrent() const
  { return actual_current_; }

  inline bool IsStopped() const
  {
    if (object_status_ == FINGERS_IN_MOTION)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  inline bool IsActivated() const
  {
    if ((gripper_activation_ == GRIPPER_ACTIVATED)
        && (gripper_status_ == ACTIVATION_COMPLETED))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  inline bool HasFault() const
  {
    if (fault_status_ == NO_FAULT)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  inline std::string Print() const
  {
    std::ostringstream strm;
    strm << "GRIPPER_ACTIVATION: " << gripper_activation_
         << " ACTION_STATUS: " << action_status_
         << " GRIPPER_STATUS: " << gripper_status_
         << " OBJECT_STATUS: " << object_status_
         << " FAULT_STATUS: " << fault_status_
         << " Target position: " << target_position_
         << " Actual position: " << actual_position_
         << " Actual current: " << actual_current_;
    return strm.str();
  }
};

class Robotiq2FingerGripperCommand
{
protected:

  double target_position_;
  double target_speed_;
  double target_force_;

public:

  Robotiq2FingerGripperCommand(const double target_position,
                               const double target_speed,
                               const double target_force)
  {
    if (std::isinf(target_position) || std::isnan(target_position))
    {
      throw std::invalid_argument("target_position is NAN or INF");
    }
    else if ((target_position < 0.0) || (target_position > 1.0))
    {
      throw std::invalid_argument("target_position > 1.0 or < 0.0");
    }
    if (std::isinf(target_speed) || std::isnan(target_speed))
    {
      throw std::invalid_argument("target_speed is NAN or INF");
    }
    else if ((target_speed < 0.0) || (target_speed > 1.0))
    {
      throw std::invalid_argument("target_speed > 1.0 or < 0.0");
    }
    if (std::isinf(target_force) || std::isnan(target_force))
    {
      throw std::invalid_argument("target_force is NAN or INF");
    }
    else if ((target_force < 0.0) || (target_force > 1.0))
    {
      throw std::invalid_argument("target_force > 1.0 or < 0.0");
    }
    target_position_ = target_position;
    target_speed_ = target_speed;
    target_force_ = target_force;
  }

  inline double TargetPosition() const { return target_position_; }

  inline double TargetSpeed() const { return target_speed_; }

  inline double TargetForce() const { return target_force_; }

  inline uint8_t PositionCommand() const
  {
    return (uint8_t)(255.0 * target_position_);
  }

  inline uint8_t SpeedCommand() const
  {
    return (uint8_t)(255.0 * target_speed_);
  }

  inline uint8_t ForceCommand() const
  {
    return (uint8_t)(255.0 * target_force_);
  }
};

class Robotiq2FingerGripperModbusInterface
{
protected:

  modbus_t* modbus_interface_ptr_;
  std::function<void(const std::string&)> logging_fn_;

  const uint16_t ROGI_FIRST_REGISTER = 0x03e8;
  const uint16_t RIGO_FIRST_REGISTER = 0x07d0;
  const uint16_t NUM_REGISTERS = 3;

public:

  Robotiq2FingerGripperModbusInterface(
      const std::function<void(const std::string&)>& logging_fn);

  virtual ~Robotiq2FingerGripperModbusInterface();

  inline void Log(const std::string& message) { logging_fn_(message); }

  Robotiq2FingerGripperStatus GetGripperStatus();

  bool SendGripperCommand(const Robotiq2FingerGripperCommand& command);

  bool CommandGripperBlocking(const Robotiq2FingerGripperCommand& command);

  bool ReactivateGripper();

  bool ActivateGripper();

protected:

  void ConfigureModbusConnection(const uint16_t gripper_slave_id);

  bool WriteMultipleRegisters(const uint16_t start_register,
                              const std::vector<uint16_t>& register_values);

  void ShutdownConnection();
};

class Robotiq2FingerGripperModbusRtuInterface
    : public Robotiq2FingerGripperModbusInterface
{
public:

  Robotiq2FingerGripperModbusRtuInterface(
      const std::function<void(const std::string&)>& logging_fn,
      const std::string& modbus_rtu_interface,
      const int32_t modbus_rtu_baud_rate,
      const uint16_t gripper_slave_id);
};

class Robotiq2FingerGripperModbusTcpInterface
    : public Robotiq2FingerGripperModbusInterface
{
public:

  Robotiq2FingerGripperModbusTcpInterface(
      const std::function<void(const std::string&)>& logging_fn,
      const std::string& modbus_tcp_address,
      const int32_t modbus_tcp_port,
      const uint16_t gripper_slave_id);
};
}
