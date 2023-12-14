#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <functional>
#include <common_robotics_utilities/maybe.hpp>
#include <common_robotics_utilities/serialization.hpp>

namespace schunk_wsg_driver
{
// This code for calculating the checksum was derived from the WSG
// Command Set Reference Manual.
const std::array<uint16_t, 256> CRC_TABLE =
{{
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
}};

inline uint16_t ComputeCRC(const std::vector<uint8_t>& buffer,
                           const size_t start_idx,
                           const size_t end_idx)
{
  if ((start_idx >= buffer.size()) || (end_idx > buffer.size()))
  {
    throw std::runtime_error(
          "Start or end index for CRC computation is outside buffer bounds");
  }
  uint16_t crc = 0xFFFF;
  /* Process each byte prior to checksum field */
  for (size_t idx = start_idx; idx < end_idx; idx++)
  {
    const uint16_t index =  (crc ^ buffer[idx]) & 0x00FF;
    crc = static_cast<uint16_t>(CRC_TABLE[index] ^ (crc >> 8));
  }
  return crc;
}

enum GripperCommand : uint8_t
{
  kLoop = 0x06,
  kDisconnectAnnounce = 0x07,
  kHome = 0x20,
  kPrePosition = 0x21,
  kStop = 0x22,
  kFastStop = 0x23,
  kAcknowledgeStopOrFault = 0x24,
  kGrasp = 0x25,
  kRelease = 0x26,
  kSetAccel = 0x30,
  kGetAccel = 0x31,
  kSetForceLimit = 0x32,
  kGetForceLimit = 0x33,
  kSetSoftLimits = 0x34,
  kGetSoftLimits = 0x35,
  kClearSoftLimits = 0x36,
  kTareForceSensor = 0x38,
  kGetSystemState = 0x40,
  kGetGraspState = 0x41,
  kGetGraspStats = 0x42,
  kGetOpeningWidth = 0x43,
  kGetSpeed = 0x44,
  kGetForce = 0x45,
  kGetTemperature = 0x46,
  kGetSystemInfo = 0x50,
  kSetDeviceTag = 0x51,
  kGetDeviceTag = 0x52,
  kGetSystemLimits = 0x53,
  kGetFingerInfo = 0x60,
  kGetFingerFlags = 0x61,
  kFingerPowerControl = 0x62,
  kGetFingerData = 0x63,
};

// Status codes
enum StatusCode : uint16_t
{
  E_SUCCESS = 0, //!< No error
  E_NOT_AVAILABLE, //!< Device, service or data is not available
  E_NO_SENSOR, //!< No sensor connected
  E_NOT_INITIALIZED, //!< The device is not initialized
  E_ALREADY_RUNNING, //!< Service is already running
  E_FEATURE_NOT_SUPPORTED, //!< The asked feature is not supported
  E_INCONSISTENT_DATA, //!< One or more dependent parameters mismatch
  E_TIMEOUT, //!< Timeout error
  E_READ_ERROR, //!< Error while reading from a device
  E_WRITE_ERROR, //!< Error while writing to a device
  E_INSUFFICIENT_RESOURCES, //!< No memory available
  E_CHECKSUM_ERROR, //!< Checksum error
  E_NO_PARAM_EXPECTED, //!< No parameters expected
  E_NOT_ENOUGH_PARAMS, //!< Not enough parameters
  E_CMD_UNKNOWN, //!< Unknown command
  E_CMD_FORMAT_ERROR, //!< Command format error
  E_ACCESS_DENIED, //!< Access denied
  E_ALREADY_OPEN, //!< The interface is already open
  E_CMD_FAILED, //!< Command failed
  E_CMD_ABORTED, //!< Command aborted
  E_INVALID_HANDLE, //!< invalid handle
  E_NOT_FOUND, //!< device not found
  E_NOT_OPEN, //!< device not open
  E_IO_ERROR, //!< I/O error
  E_INVALID_PARAMETER, //!< invalid parameter
  E_INDEX_OUT_OF_BOUNDS, //!< index out of bounds
  E_CMD_PENDING, //!< Command execution needs more time
  E_OVERRUN, //!< Data overrun
  E_RANGE_ERROR, //!< Range error
  E_AXIS_BLOCKED, //!< Axis is blocked
  E_FILE_EXISTS //!< File already exists
};

// State flag bits; for definitions see the command set reference.
enum StateFlagBits : uint32_t
{
  // 31-21 reserved.
  SF_SCRIPT_FAILURE   = 1 << 20,
  SF_SCRIPT_RUNNING   = 1 << 19,
  SF_CMD_FAILURE    = 1 << 18,
  SF_FINGER_FAULT     = 1 << 17,
  SF_CURR_FAULT     = 1 << 16,
  SF_POWER_FAULT    = 1 << 15,
  SF_TEMP_FAULT     = 1 << 14,
  SF_TEMP_WARNING     = 1 << 13,
  SF_FAST_STOP      = 1 << 12,
  // 11 reserved.
  // 10 reserved.
  SF_FORCECNTL_MODE   = 1 << 9,
  // 8 reserved.
  SF_TARGET_POS_REACHED = 1 << 7,
  SF_AXIS_STOPPED     = 1 << 6,
  SF_SOFT_LIMIT_PLUS  = 1 << 5,
  SF_SOFT_LIMIT_MINUS   = 1 << 4,
  SF_BLOCKED_PLUS     = 1 << 3,
  SF_BLOCKED_MINUS    = 1 << 2,
  SF_MOVING       = 1 << 1,
  SF_REFERENCED     = 1 << 0,
};

// Grasping states; for definitions see the command set reference.
enum GraspingState : uint8_t
{
  kIdle = 0,
  kGrasping = 1,
  kNoPartFound = 2,
  kPartLost = 3,
  kHolding = 4,
  kReleasing = 5,
  kPositioning = 6,
  kError = 7
};

class WSGRawStatusMessage
{
private:
  template<typename T>
  using Deserialized
      = common_robotics_utilities::serialization::Deserialized<T>;

  uint8_t command_;
  uint16_t status_;
  std::vector<uint8_t> param_buffer_;

public:

  uint8_t Command() const { return command_; }

  uint16_t Status() const { return status_; }

  const std::vector<uint8_t>& ParamBuffer() const { return param_buffer_; }

  static Deserialized<WSGRawStatusMessage> Deserialize(
      const std::vector<uint8_t>& buffer, const uint64_t current);

  uint64_t DeserializeSelf(const std::vector<uint8_t>& buffer,
                           const uint64_t current);
};

class WSGRawCommandMessage
{
private:

  uint8_t command_;
  std::vector<uint8_t> param_buffer_;

public:

  WSGRawCommandMessage(const uint8_t command,
                       const std::vector<uint8_t>& param_buffer)
    : command_(command), param_buffer_(param_buffer) {}

  WSGRawCommandMessage(const uint8_t command)
    : command_(command), param_buffer_({}) {}

  uint8_t Command() const { return command_; }

  const std::vector<uint8_t>& ParamBuffer() const { return param_buffer_; }

  template<typename T>
  void AppendParameterToBuffer(const T& parameter)
  {
    std::vector<uint8_t> temp_buffer;
    common_robotics_utilities::serialization
        ::SerializeMemcpyable(parameter, temp_buffer);
    param_buffer_.insert(param_buffer_.end(),
                         temp_buffer.begin(),
                         temp_buffer.end());
  }

  static uint64_t Serialize(const WSGRawCommandMessage& message,
                            std::vector<uint8_t>& buffer);

  uint64_t SerializeSelf(std::vector<uint8_t>& buffer) const;
};

class GripperMotionStatus
{
private:

  double actual_position_;
  double actual_velocity_;
  double actual_effort_;
  double target_position_;
  double max_speed_;
  double max_effort_;

public:

  GripperMotionStatus(const double actual_position,
                      const double actual_velocity,
                      const double actual_effort,
                      const double target_position,
                      const double max_speed,
                      const double max_effort)
    : actual_position_(actual_position),
      actual_velocity_(actual_velocity),
      actual_effort_(actual_effort),
      target_position_(target_position),
      max_speed_(max_speed),
      max_effort_(max_effort) {}

  GripperMotionStatus()
    : actual_position_(-1.0),
      actual_velocity_(0.0),
      actual_effort_(0.0),
      target_position_(-1.0),
      max_speed_(0.0),
      max_effort_(0.0) {}

  void UpdateActualPosition(const double actual_position)
  {
    actual_position_ = actual_position;
  }

  void UpdateActualVelocity(const double actual_velocity)
  {
    actual_velocity_ = actual_velocity;
  }

  void UpdateActualEffort(const double actual_effort)
  {
    actual_effort_ = actual_effort;
  }

  void UpdateTargetPositionSpeedEffort(const double target_position,
                                       const double max_speed,
                                       const double max_effort)
  {
    target_position_ = target_position;
    max_speed_ = max_speed;
    max_effort_ = max_effort;
  }

  double ActualPosition() const { return actual_position_; }

  double ActualVelocity() const { return actual_velocity_; }

  double ActualEffort() const { return actual_effort_; }

  double TargetPosition() const { return target_position_; }

  double MaxSpeed() const { return max_speed_; }

  double MaxEffort() const { return max_effort_; }
};

class PhysicalLimits
{
private:

  double stroke_mm_;
  double min_speed_mm_per_s_;
  double max_speed_mm_per_s_;
  double min_acc_mm_per_ss_;
  double max_acc_mm_per_ss_;
  double min_force_;
  double nominal_force_;
  double overdrive_force_;

public:

  PhysicalLimits(const double stroke_mm,
                 const double min_speed_mm_per_s,
                 const double max_speed_mm_per_s,
                 const double min_acc_mm_per_ss,
                 const double max_acc_mm_per_ss,
                 const double min_force,
                 const double nominal_force,
                 const double overdrive_force)
    : stroke_mm_(stroke_mm),
      min_speed_mm_per_s_(min_speed_mm_per_s),
      max_speed_mm_per_s_(max_speed_mm_per_s),
      min_acc_mm_per_ss_(min_acc_mm_per_ss),
      max_acc_mm_per_ss_(max_acc_mm_per_ss),
      min_force_(min_force),
      nominal_force_(nominal_force),
      overdrive_force_(overdrive_force) {}

  PhysicalLimits()
    : stroke_mm_(0.0),
      min_speed_mm_per_s_(0.0),
      max_speed_mm_per_s_(0.0),
      min_acc_mm_per_ss_(0.0),
      max_acc_mm_per_ss_(0.0),
      min_force_(0.0),
      nominal_force_(0.0),
      overdrive_force_(0.0) {}

  double StrokeMM() const { return stroke_mm_; }

  std::pair<double, double> MinMaxSpeedMMPerS() const
  {
    return std::make_pair(min_speed_mm_per_s_, max_speed_mm_per_s_);
  }

  std::pair<double, double> MinMaxAccelMMPerSS() const
  {
    return std::make_pair(min_acc_mm_per_ss_, max_acc_mm_per_ss_);
  }

  double MinForce() const { return min_force_; }

  double NominalForce() const { return nominal_force_; }

  double OverdriveForce() const { return overdrive_force_; }

  std::string Print() const;
};

class WSGInterface
{
private:
  template<typename T>
  using OwningMaybe = common_robotics_utilities::OwningMaybe<T>;

  mutable std::mutex status_mutex_;
  OwningMaybe<PhysicalLimits> maybe_physical_limits_;
  GripperMotionStatus motion_status_;

  std::mutex status_queue_mutex_;
  std::vector<WSGRawStatusMessage> status_queue_;

  std::function<void(const std::string&)> logging_fn_;
  bool use_scripting_interface_ = false;

public:

  WSGInterface(const std::function<void(const std::string&)>& logging_fn)
    : logging_fn_(logging_fn) {}

  virtual ~WSGInterface() {}

  void Log(const std::string& message) { logging_fn_(message); }

  // Initialize gripper.
  // @param update_period_ms Update period, namely used for recurring status.
  // @param use_scripting_interface Set to true if using scripting to both send
  //   commands and receive status. Note that this mode will not use Schunk's
  //   recurring status feature, and instead rely on the script to send status
  //   data back.
  bool InitializeGripper(
      const uint16_t update_period_ms,
      bool use_scripting_interface = false);

  bool SetTargetPositionSpeedEffort(const double target_position,
                                    const double max_speed,
                                    const double max_effort);

  OwningMaybe<PhysicalLimits> GetGripperPhysicalLimits() const;

  GripperMotionStatus GetGripperStatus() const;

  void RefreshGripperStatus();

  void Shutdown();

  OwningMaybe<WSGRawStatusMessage> SendCommandAndAwaitStatus(
      const WSGRawCommandMessage& command, const double timeout);

protected:

  double GetCommandPositionMM(const double target_position,
                              const PhysicalLimits& limits);

  double GetCommandSpeedMMpS(const double target_speed,
                             const PhysicalLimits& limits);

  double GetCommandEffortN(const double target_effort,
                           const PhysicalLimits& limits);

  bool StartGripper();

  bool StopGripper();

  enum HomeDirection : uint8_t {kDefault=0,
                                kPositive=1,
                                kNegative=2};

  bool Home(const HomeDirection direction);

  bool Tare();

  bool Grasp(const double width_mm, const double speed_mm_per_s);

  bool SetForceLimit(const double force);

  bool SetForceLimitNonBlocking(const double force);

  bool SetAcceleration(const double acceleration_mm_per_s);

  bool ClearSoftLimits();

  bool EnableRecurringStatus(const GripperCommand command,
                             const uint16_t update_period_ms,
                             const double timeout);

  bool DisableRecurringStatus(const GripperCommand command,
                              const double timeout);

  enum PrePositionStopMode : uint8_t {kPrePositionClampOnBlock=0,
                                      kPrePositionStopOnBlock=1};

  enum PrePositionMoveMode : uint8_t {kPrePositionAbsolute=0,
                                      kPrePositionRelative=2};

  WSGRawCommandMessage MakePrePositionCommand(
      const PrePositionStopMode stop_mode,
      const PrePositionMoveMode move_mode,
      double width_mm,
      double speed_mm_per_s) const;

  bool PrePosition(const PrePositionStopMode stop_mode,
                   const PrePositionMoveMode move_mode,
                   double width_mm,
                   double speed_mm_per_s);

  bool PrePositionNonBlocking(const PrePositionStopMode stop_mode,
                              const PrePositionMoveMode move_mode,
                              double width_mm,
                              double speed_mm_per_s);

  PhysicalLimits RetrieveGripperPhysicalLimits();

  std::vector<WSGRawStatusMessage> GetStatusQueue();

  void AppendToStatusQueue(const WSGRawStatusMessage& status_msg);

  virtual bool CommandGripper(const WSGRawCommandMessage& command) = 0;

  virtual void ShutdownConnection() = 0;
};
}
