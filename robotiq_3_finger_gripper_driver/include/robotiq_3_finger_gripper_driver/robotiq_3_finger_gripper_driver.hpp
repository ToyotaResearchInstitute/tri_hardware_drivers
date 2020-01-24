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
#include <modbus/modbus.h>
#include <sstream>

namespace robotiq_3_finger_gripper_driver
{
class Robotiq3FingerGripperActuatorStatus
{
public:

  enum OBJECT_STATUS : uint8_t
  { FINGER_IN_MOTION=0x00,
    FINGER_STOPPED_CONTACT_OPENING=0x01,
    FINGER_STOPPED_CONTACT_CLOSING=0x02,
    FINGER_AT_REQUESTED=0x03,
    FINGER_STOPPED=0x04 };

private:

  double target_position_;
  double actual_position_;
  double actual_current_;
  OBJECT_STATUS object_status_;

public:

  Robotiq3FingerGripperActuatorStatus()
    : target_position_(0.0),
      actual_position_(0.0),
      actual_current_(0.0),
      object_status_(FINGER_STOPPED) {}

  Robotiq3FingerGripperActuatorStatus(const double target_position,
                                      const double actual_position,
                                      const double actual_current,
                                      const OBJECT_STATUS object_status)
    : target_position_(target_position),
      actual_position_(actual_position),
      actual_current_(actual_current),
      object_status_(object_status) {}

  Robotiq3FingerGripperActuatorStatus(const uint8_t raw_target_position,
                                      const uint8_t raw_actual_position,
                                      const uint8_t raw_actual_current,
                                      const uint8_t raw_object_status,
                                      const bool gto)
    : target_position_(static_cast<double>(raw_target_position) / 255.0),
      actual_position_(static_cast<double>(raw_actual_position) / 255.0),
      actual_current_(static_cast<double>(raw_actual_current) / 255.0)
  {
    if (gto)
    {
      object_status_ = static_cast<OBJECT_STATUS>(raw_object_status);
    }
    else
    {
      object_status_ = FINGER_STOPPED;
    }
  }

  inline double TargetPosition() const
  { return target_position_; }

  inline double ActualPosition() const
  { return actual_position_; }

  inline double ActualCurrent() const
  { return actual_current_; }

  inline OBJECT_STATUS ObjectStatus() const
  { return object_status_; }

  inline std::string Print() const
  {
    std::ostringstream strm;
    strm << "OBJECT_STATUS: " << static_cast<int32_t>(object_status_)
         << " Target position: " << target_position_
         << " Actual position: " << actual_position_
         << " Actual current: " << actual_current_;
    return strm.str();
  }
};

class Robotiq3FingerGripperStatus
{
public:

  enum GRIPPER_ACTIVATION_STATUS : uint8_t
  { GRIPPER_RESET=0x00,
    GRIPPER_ACTIVATION=0x01 };

  enum GRIPPER_MODE_STATUS : uint8_t
  { GRIPPER_MODE_BASIC=0x00,
    GRIPPER_MODE_PINCH=0x01,
    GRIPPER_MODE_WIDE=0x02,
    GRIPPER_SCISSOR_MODE=0x03 };

  enum GRIPPER_ACTION_STATUS : uint8_t
  { GRIPPER_STOPPED_OR_BUSY=0x00,
    GRIPPER_GOTO=0x01 };

  enum GRIPPER_SYSTEM_STATUS : uint8_t
  { GRIPPER_RESET_OR_AUTO_RELEASE=0x00,
    GRIPPER_ACTIVATION_IN_PROGRESS=0x01,
    GRIPPER_MODE_CHANGE_IN_PROGRESS=0x02,
    GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE=0x03 };

  enum GRIPPER_MOTION_STATUS : uint8_t
  { GRIPPER_IN_MOTION=0x00,
    GRIPPER_ONE_OR_TWO_STOPPED_EARLY=0x01,
    GRIPPER_ALL_STOPPED_EARLY=0x02,
    GRIPPER_ALL_AT_REQUESTED=0x03,
    GRIPPER_STOPPED_UNKNOWN=0x04 };

  enum GRIPPER_FAULT_STATUS : uint8_t
  { NO_FAULTS=0x00,
    PRIORITY_ACTIVATION_MUST_BE_SET=0x05,
    PRIORITY_MODE_CHANGE_NEEDED=0x06,
    PRIORITY_NEEDS_ACTIVATION=0x07,
    MINOR_COMM_CHIP_NOT_READY=0x09,
    MINOR_CHANGING_MODE_FAULT=0x0a,
    MINOR_AUTO_RELEASE_IN_PROGRESS=0x0b,
    MAJOR_ACTIVATION_FAULT=0x0d,
    MAJOR_CHANGING_MODE_FAULT=0x0e,
    MAJOR_AUTO_RELEASE_COMPLETE=0x0f };

  enum STATUS_BITMASKS : uint8_t
  { GACT_MASK=0b00000001,
    GMOD_MASK=0b00000110,
    GGTO_MASK=0b00001000,
    GIMC_MASK=0b00110000,
    GSTA_MASK=0b11000000,
    GDTA_MASK=0b00000011,
    GDTB_MASK=0b00001100,
    GDTC_MASK=0b00110000,
    GDTS_MASK=0b11000000,
    GFLT_MASK=0b00001111 };

  enum STATUS_OFFSETS : uint8_t
  { GACT_OFFSET=0x00,
    GMOD_OFFSET=0x01,
    GGTO_OFFSET=0x03,
    GIMC_OFFSET=0x04,
    GSTA_OFFSET=0x06,
    GDTA_OFFSET=0x00,
    GDTB_OFFSET=0x02,
    GDTC_OFFSET=0x04,
    GDTS_OFFSET=0x06,
    GFLT_OFFSET=0x00 };

  enum REGISTER_MASKS : uint16_t
  { GRIPPER_STATUS_MASK=0x00ff,
    OBJECT_STATUS_MASK=0xff00,
    FAULT_STATUS_MASK=0x00ff,
    FINGER_A_POSITION_REQUEST_MASK=0xff00,
    FINGER_A_POSITION_MASK=0x00ff,
    FINGER_A_CURRENT_MASK=0xff00,
    FINGER_B_POSITION_REQUEST_MASK=0x00ff,
    FINGER_B_POSITION_MASK=0xff00,
    FINGER_B_CURRENT_MASK=0x00ff,
    FINGER_C_POSITION_REQUEST_MASK=0xff00,
    FINGER_C_POSITION_MASK=0x00ff,
    FINGER_C_CURRENT_MASK=0xff00,
    SCISSOR_POSITION_REQUEST_MASK=0x00ff,
    SCISSOR_POSITION_MASK=0xff00,
    SCISSOR_CURRENT_MASK=0x00ff};

private:

  Robotiq3FingerGripperActuatorStatus finger_a_status_;
  Robotiq3FingerGripperActuatorStatus finger_b_status_;
  Robotiq3FingerGripperActuatorStatus finger_c_status_;
  Robotiq3FingerGripperActuatorStatus scissor_status_;
  GRIPPER_ACTIVATION_STATUS gripper_activation_status_;
  GRIPPER_MODE_STATUS gripper_mode_status_;
  GRIPPER_ACTION_STATUS gripper_action_status_;
  GRIPPER_SYSTEM_STATUS gripper_system_status_;
  GRIPPER_MOTION_STATUS gripper_motion_status_;
  GRIPPER_FAULT_STATUS gripper_fault_status_;

public:

  Robotiq3FingerGripperStatus(const std::vector<uint8_t>& received_bytes)
  {
    if (received_bytes.size() < 15)
    {
      throw std::invalid_argument("received_bytes.size() < 15");
    }
    const uint8_t gripper_status_byte = received_bytes[0];
    gripper_activation_status_
        = static_cast<GRIPPER_ACTIVATION_STATUS>(
            (gripper_status_byte & GACT_MASK) >> GACT_OFFSET);
    gripper_mode_status_
        = static_cast<GRIPPER_MODE_STATUS>(
            (gripper_status_byte & GMOD_MASK) >> GMOD_OFFSET);
    gripper_action_status_
        = static_cast<GRIPPER_ACTION_STATUS>(
            (gripper_status_byte & GGTO_MASK) >> GGTO_OFFSET);
    gripper_system_status_
        = static_cast<GRIPPER_SYSTEM_STATUS>(
            (gripper_status_byte & GIMC_MASK) >> GIMC_OFFSET);
    gripper_motion_status_
        = static_cast<GRIPPER_MOTION_STATUS>(
            (gripper_status_byte & GSTA_MASK) >> GSTA_OFFSET);
    const bool gto = (gripper_action_status_ == GRIPPER_GOTO);
    // Gripper fault status
    const uint8_t raw_fault_status_byte = received_bytes[2];
    gripper_fault_status_
        = static_cast<GRIPPER_FAULT_STATUS>(
            (raw_fault_status_byte & GFLT_MASK) >> GFLT_OFFSET);
    // Per-finger object status
    const uint8_t object_status_byte = received_bytes[1];
    const uint8_t gdta
        = static_cast<uint8_t>((object_status_byte & GDTA_MASK) >> GDTA_OFFSET);
    const uint8_t gdtb
        = static_cast<uint8_t>((object_status_byte & GDTB_MASK) >> GDTB_OFFSET);
    const uint8_t gdtc
        = static_cast<uint8_t>((object_status_byte & GDTC_MASK) >> GDTC_OFFSET);
    const uint8_t gdts
        = static_cast<uint8_t>((object_status_byte & GDTS_MASK) >> GDTS_OFFSET);
    // Per-finger status
    finger_a_status_ = Robotiq3FingerGripperActuatorStatus(received_bytes[3],
                                                           received_bytes[4],
                                                           received_bytes[5],
                                                           gdta, gto);
    finger_b_status_ = Robotiq3FingerGripperActuatorStatus(received_bytes[6],
                                                           received_bytes[7],
                                                           received_bytes[8],
                                                           gdtb, gto);
    finger_c_status_ = Robotiq3FingerGripperActuatorStatus(received_bytes[9],
                                                           received_bytes[10],
                                                           received_bytes[11],
                                                           gdtc, gto);
    scissor_status_ = Robotiq3FingerGripperActuatorStatus(received_bytes[12],
                                                          received_bytes[13],
                                                          received_bytes[14],
                                                          gdts, gto);
  }

  inline GRIPPER_ACTIVATION_STATUS GripperActivationStatus() const
  { return gripper_activation_status_; }

  inline GRIPPER_MODE_STATUS GripperModeStatus() const
  { return gripper_mode_status_; }

  inline GRIPPER_ACTION_STATUS GripperActionStatus() const
  { return gripper_action_status_; }

  inline GRIPPER_SYSTEM_STATUS GripperSystemStatus() const
  { return gripper_system_status_; }

  inline GRIPPER_MOTION_STATUS GripperMotionStatus() const
  { return gripper_motion_status_; }

  inline GRIPPER_FAULT_STATUS GripperFaultStatus() const
  { return gripper_fault_status_; }

  inline const Robotiq3FingerGripperActuatorStatus& FingerAStatus() const
  { return finger_a_status_; }

  inline const Robotiq3FingerGripperActuatorStatus& FingerBStatus() const
  { return finger_b_status_; }

  inline const Robotiq3FingerGripperActuatorStatus& FingerCStatus() const
  { return finger_b_status_; }

  inline const Robotiq3FingerGripperActuatorStatus& ScissorStatus() const
  { return scissor_status_; }

  inline bool IsStopped() const
  {
    if (gripper_motion_status_ == GRIPPER_IN_MOTION)
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
    if ((gripper_activation_status_ == GRIPPER_ACTIVATION)
        && (gripper_system_status_ == GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE))
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
    if (gripper_fault_status_ == NO_FAULTS)
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
    strm << "GRIPPER_ACTIVATION_STATUS: "
         << static_cast<int32_t>(gripper_activation_status_)
         << " GRIPPER_MODE_STATUS: "
         << static_cast<int32_t>(gripper_mode_status_)
         << " GRIPPER_ACTION_STATUS: "
         << static_cast<int32_t>(gripper_action_status_)
         << " GRIPPER_SYSTEM_STATUS: "
         << static_cast<int32_t>(gripper_system_status_)
         << " GRIPPER_MOTION_STATUS: "
         << static_cast<int32_t>(gripper_motion_status_)
         << " GRIPPER_FAULT_STATUS: "
         << static_cast<int32_t>(gripper_fault_status_)
         << " Finger A: " << finger_a_status_.Print()
         << " Finger B: " << finger_b_status_.Print()
         << " Finger C: " << finger_c_status_.Print()
         << " Scissor: " << scissor_status_.Print();
    return strm.str();
  }
};

class Robotiq3FingerGripperActuatorCommand
{
protected:

  double target_position_;
  double target_speed_;
  double target_force_;

public:

  Robotiq3FingerGripperActuatorCommand(const double target_position,
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

  inline double TargetPosition() const
  { return target_position_; }

  inline double TargetSpeed() const
  { return target_speed_; }

  inline double TargetForce() const
  { return target_force_; }

  inline uint8_t PositionCommand() const
  {
    return static_cast<uint8_t>(255.0 * target_position_);
  }

  inline uint8_t SpeedCommand() const
  {
    return static_cast<uint8_t>(255.0 * target_speed_);
  }

  inline uint8_t ForceCommand() const
  {
    return static_cast<uint8_t>(255.0 * target_force_);
  }
};

class Robotiq3FingerGripperCommand
{
private:

  Robotiq3FingerGripperActuatorCommand finger_a_command_;
  Robotiq3FingerGripperActuatorCommand finger_b_command_;
  Robotiq3FingerGripperActuatorCommand finger_c_command_;
  Robotiq3FingerGripperActuatorCommand scissor_command_;

public:

  Robotiq3FingerGripperCommand(
      const Robotiq3FingerGripperActuatorCommand& finger_a_command,
      const Robotiq3FingerGripperActuatorCommand& finger_b_command,
      const Robotiq3FingerGripperActuatorCommand& finger_c_command,
      const Robotiq3FingerGripperActuatorCommand& scissor_command)
    : finger_a_command_(finger_a_command),
      finger_b_command_(finger_b_command),
      finger_c_command_(finger_c_command),
      scissor_command_(scissor_command) {}

  inline const Robotiq3FingerGripperActuatorCommand& FingerACommand() const
  { return finger_a_command_; }

  inline const Robotiq3FingerGripperActuatorCommand& FingerBCommand() const
  { return finger_b_command_; }

  inline const Robotiq3FingerGripperActuatorCommand& FingerCCommand() const
  { return finger_c_command_; }

  inline const Robotiq3FingerGripperActuatorCommand& ScissorCommand() const
  { return scissor_command_; }
};

class Robotiq3FingerGripperInterface
{
private:

  std::function<void(const std::string&)> logging_fn_;

public:

  const size_t NUM_ROBOTIQ_REGISTERS = 15;

  explicit Robotiq3FingerGripperInterface(
      const std::function<void(const std::string&)>& logging_fn);

  virtual ~Robotiq3FingerGripperInterface() {}

  inline void Log(const std::string& message) { logging_fn_(message); }

  Robotiq3FingerGripperStatus GetGripperStatus();

  bool SendGripperCommand(const Robotiq3FingerGripperCommand& command);

  bool CommandGripperBlocking(const Robotiq3FingerGripperCommand& command);

  bool ReactivateGripper();

  bool ActivateGripper();

protected:

  virtual bool WriteROGIRegisters(
      const std::vector<uint8_t>& register_values) = 0;

  virtual std::vector<uint8_t> ReadRIGORegisters() = 0;

  virtual void ShutdownConnection() = 0;
};

class Robotiq3FingerGripperModbusInterface
    : public Robotiq3FingerGripperInterface
{
private:

  enum INTERFACE_TYPE { NONE, TCP, RTU };
  INTERFACE_TYPE interface_type_;
  modbus_t* modbus_interface_ptr_;
  const uint16_t NUM_MODBUS_REGISTERS = 8;
  uint16_t rogi_first_register_;
  uint16_t rigo_first_register_;

public:

  explicit Robotiq3FingerGripperModbusInterface(
      const std::function<void(const std::string&)>& logging_fn);

  ~Robotiq3FingerGripperModbusInterface();

  void ConnectModbusTcp(const std::string& gripper_ip_address,
                        const int32_t gripper_port,
                        const uint16_t gripper_slave_id);

  void ConnectModbusRtu(const std::string& modbus_rtu_interface,
                        const int32_t gripper_baud_rate,
                        const uint16_t gripper_slave_id);

private:

  virtual bool WriteROGIRegisters(
      const std::vector<uint8_t>& register_values);

  virtual std::vector<uint8_t> ReadRIGORegisters();

  virtual void ShutdownConnection();
};
}
