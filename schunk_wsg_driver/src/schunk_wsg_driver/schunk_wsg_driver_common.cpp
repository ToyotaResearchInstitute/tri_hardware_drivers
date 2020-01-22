#include <schunk_wsg_driver/schunk_wsg_driver_common.hpp>

namespace schunk_wsg_driver
{
using common_robotics_utilities::serialization::DeserializeMemcpyable;

uint64_t WSGRawCommandMessage::Serialize(const WSGRawCommandMessage& message,
                                         std::vector<uint8_t>& buffer)
{
  return message.SerializeSelf(buffer);
}

uint64_t WSGRawCommandMessage::SerializeSelf(std::vector<uint8_t>& buffer) const
{
  const uint64_t start_buffer_size = buffer.size();
  // Write the header
  buffer.push_back(0xaa);
  buffer.push_back(0xaa);
  buffer.push_back(0xaa);
  buffer.push_back(command_);
  const uint16_t payload_size = (uint16_t)param_buffer_.size();
  buffer.push_back((uint8_t)(payload_size & 0xff));
  buffer.push_back((uint8_t)((payload_size >> 8) & 0xff));
  // Copy the payload buffer
  buffer.insert(buffer.end(), param_buffer_.begin(), param_buffer_.end());
  // Compute the checksum
  const uint16_t checksum
      = ComputeCRC(buffer, start_buffer_size, buffer.size());
  // Add the checksum
  buffer.push_back((uint8_t)(checksum & 0xff));
  buffer.push_back((uint8_t)((checksum >> 8) & 0xff));
  // Figure out how many bytes were written
  const uint64_t end_buffer_size = buffer.size();
  const uint64_t bytes_written = end_buffer_size - start_buffer_size;
  return bytes_written;
}

Deserialized<WSGRawStatusMessage> WSGRawStatusMessage::Deserialize(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  WSGRawStatusMessage status_message;
  const uint64_t bytes_read = status_message.DeserializeSelf(buffer, current);
  return Deserialized<WSGRawStatusMessage>(status_message, bytes_read);
}

uint64_t WSGRawStatusMessage::DeserializeSelf(
    const std::vector<uint8_t>& buffer, const uint64_t current)
{
  uint64_t current_position = current;
  // Make sure the buffer is big enough
  if ((buffer.size() - current) < 8)
  {
    throw std::runtime_error("Buffer is not large enough to load header");
  }
  // Read the fixed-size elements of the status message
  // Check the header
  const uint8_t h1 = buffer[current_position + 0];
  const uint8_t h2 = buffer[current_position + 1];
  const uint8_t h3 = buffer[current_position + 2];
  if ((h1 != 0xaa) || (h2 != 0xaa) || (h3 != 0xaa))
  {
    throw std::runtime_error("Message header invalid - h1 != h2 != h3 != 0xaa");
  }
  // Load the command
  command_ = buffer[current_position + 3];
  // Load the payload size
  size_t payload_size = (size_t)buffer[current_position + 4]
                        + ((size_t)buffer[current_position + 5] << 8);
  if (payload_size < 2)
  {
    throw std::runtime_error("Invalid message payload size < 2");
  }
  // Load the status code
  status_ = (uint16_t)(buffer[current_position + 6]
            + (buffer[current_position + 7] << 8));
  // Update current position
  current_position += 8;
  // Check to make sure the buffer is large enough, now that we know the size
  if ((buffer.size() - 8) < payload_size)
  {
    throw std::runtime_error("Buffer is too small to load message payload");
  }
  // Copy over the payload buffer
  const size_t checksum_size = 2;
  param_buffer_.clear();
  param_buffer_.insert(param_buffer_.end(),
                       buffer.begin() + (ssize_t)current_position,
                       buffer.begin() + (ssize_t)current_position
                       + (ssize_t)payload_size - (ssize_t)checksum_size);
  current_position += (payload_size - checksum_size);
  // Read the checksum
  const uint16_t read_checksum
      = (uint16_t)(buffer[current_position]
                   + (buffer[current_position + 1] << 8));
  // Validate the checksum
  const uint16_t computed_checksum
      = ComputeCRC(buffer, current, current_position);
  if (read_checksum != computed_checksum)
  {
    throw std::runtime_error(
          "Received message checksum does not match computed checksum");
  }
  current_position += 2;
  // Figure out how many bytes were read
  const uint64_t bytes_read = current_position - current;
  return bytes_read;
}

std::string PhysicalLimits::Print() const
{
  std::ostringstream strm;
  strm << "Physical limits:";
  strm << "\nStroke (mm) " << StrokeMM();
  strm << "\nMin speed (mm/s) " << MinMaxSpeedMMPerS().first;
  strm << "\nMax speed (mm/s) " << MinMaxSpeedMMPerS().second;
  strm << "\nMin accel (mm/ss) " << MinMaxAccelMMPerSS().first;
  strm << "\nMax accel (mm/ss) " << MinMaxAccelMMPerSS().second;
  strm << "\nMin force (n) " << MinForce();
  strm << "\nNominal force (n) " << NominalForce();
  strm << "\nOverdrive force (n) " << OverdriveForce();
  return strm.str();
}

// Internal implementation

OwningMaybe<WSGRawStatusMessage>
WSGInterface::SendCommandAndAwaitStatus(const WSGRawCommandMessage& command,
                                        const double timeout)
{
  const std::chrono::duration<double> timeout_duration(timeout);
  const auto start_time = std::chrono::steady_clock::now();
  const bool result = CommandGripper(command);
  if (result == false)
  {
    throw std::runtime_error("Failed to send command");
  }
  OwningMaybe<WSGRawStatusMessage> response;
  while (!response.HasValue())
  {
    std::vector<WSGRawStatusMessage> status_queue = GetStatusQueue();
    for (size_t idx = 0; idx < status_queue.size(); idx++)
    {
      const WSGRawStatusMessage& candidate_response = status_queue[idx];
      if (candidate_response.Command() != command.Command())
      {
        continue;
      }
      else if (candidate_response.Status() == E_CMD_PENDING)
      {
        const std::string msg = "Command pending";
        Log(msg);
      }
      else
      {
        response = OwningMaybe<WSGRawStatusMessage>(candidate_response);
        if (candidate_response.Status() != E_SUCCESS)
        {
          const std::string msg = "Non-success response: "
                                  + std::to_string(candidate_response.Status());
          Log(msg);
        }
      }
    }
    const auto current_time = std::chrono::steady_clock::now();
    const auto elapsed_time = current_time - start_time;
    if (elapsed_time > timeout_duration)
    {
      break;
    }
  }
  if (!response.HasValue())
  {
    Log("Failed to receive response in timeout period");
  }
  return response;
}

bool WSGInterface::StopGripper()
{
  const WSGRawCommandMessage stop_command(kStop);
  const bool result = CommandGripper(stop_command);
  return result;
}

bool WSGInterface::Home(const HomeDirection direction)
{
  const WSGRawCommandMessage home_command(kHome, {direction});
  const auto maybe_response = SendCommandAndAwaitStatus(home_command, 4.0);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Homed successfully");
      return true;
    }
    else
    {
      Log("Failed to home");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::Tare()
{
  const WSGRawCommandMessage tare_command(kTareForceSensor);
  const auto maybe_response = SendCommandAndAwaitStatus(tare_command, 4.0);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Homed successfully");
      return true;
    }
    else if (maybe_response.Value().Status() == E_NOT_AVAILABLE)
    {
      Log("Tare not available, ignoring");
      return true;
    }
    else
    {
      Log("Failed to home");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::Grasp(const double width_mm, const double speed_mm_per_s)
{
  WSGRawCommandMessage grasp_command(kGrasp);
  grasp_command.AppendParameterToBuffer((float)width_mm);
  grasp_command.AppendParameterToBuffer((float)speed_mm_per_s);
  const auto maybe_response = SendCommandAndAwaitStatus(grasp_command, 6.0);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Grasp successful");
      return true;
    }
    else
    {
      Log("Failed to grasp");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::SetForceLimit(const double force)
{
  WSGRawCommandMessage force_limit_command(kSetForceLimit);
  force_limit_command.AppendParameterToBuffer((float)force);
  const auto maybe_response
      = SendCommandAndAwaitStatus(force_limit_command, 0.1);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Set force limit (blocking) successfully");
      return true;
    }
    else
    {
      Log("Failed to set force limit (blocking)");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::SetForceLimitNonBlocking(const double force)
{
  WSGRawCommandMessage force_limit_command(kSetForceLimit);
  force_limit_command.AppendParameterToBuffer((float)force);
  const bool result = CommandGripper(force_limit_command);
  return result;
}

bool WSGInterface::SetAcceleration(const double acceleration_mm_per_s)
{
  WSGRawCommandMessage acceleration_command(kSetAccel);
  acceleration_command.AppendParameterToBuffer((float)acceleration_mm_per_s);
  const auto maybe_response
      = SendCommandAndAwaitStatus(acceleration_command, 0.1);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Set acceleration successfully");
      return true;
    }
    else
    {
      Log("Failed to set acceleration");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::ClearSoftLimits()
{
  const WSGRawCommandMessage clear_limits_command(kClearSoftLimits);
  const auto maybe_response
      = SendCommandAndAwaitStatus(clear_limits_command, 4.0);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Cleared soft limits successfully");
      return true;
    }
    else
    {
      Log("Failed to clear soft limits");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::EnableRecurringStatus(const GripperCommand command,
                                         const uint16_t update_period_ms,
                                         const double timeout)
{
  WSGRawCommandMessage recurring_status_command(command);
  recurring_status_command.AppendParameterToBuffer((uint8_t)0x01);
  recurring_status_command.AppendParameterToBuffer(update_period_ms);
  const auto maybe_response
      = SendCommandAndAwaitStatus(recurring_status_command, timeout);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Enabled recurring status successfully");
      return true;
    }
    else
    {
      Log("Failed to enable recurring status");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::DisableRecurringStatus(const GripperCommand command,
                                          const double timeout)
{
  WSGRawCommandMessage recurring_status_command(command);
  recurring_status_command.AppendParameterToBuffer((uint8_t)0x00);
  recurring_status_command.AppendParameterToBuffer((uint16_t)0x00);
  const auto maybe_response
      = SendCommandAndAwaitStatus(recurring_status_command, timeout);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("Disabled recurring status successfully");
      return true;
    }
    else
    {
      Log("Failed to disable recurring status");
      return false;
    }
  }
  else
  {
    return false;
  }
}

WSGRawCommandMessage WSGInterface::MakePrePositionCommand(
    const PrePositionStopMode stop_mode,
    const PrePositionMoveMode move_mode,
    double width_mm,
    double speed_mm_per_s) const
{
  WSGRawCommandMessage preposition_command(kPrePosition);
  const uint8_t flags = stop_mode | move_mode;
  preposition_command.AppendParameterToBuffer(flags);
  preposition_command.AppendParameterToBuffer((float)width_mm);
  preposition_command.AppendParameterToBuffer((float)speed_mm_per_s);
  return preposition_command;
}

bool WSGInterface::PrePosition(const PrePositionStopMode stop_mode,
                               const PrePositionMoveMode move_mode,
                               double width_mm,
                               double speed_mm_per_s)
{
  const WSGRawCommandMessage preposition_command
      = MakePrePositionCommand(stop_mode, move_mode, width_mm, speed_mm_per_s);
  const auto maybe_response
      = SendCommandAndAwaitStatus(preposition_command, 6.0);
  if (maybe_response)
  {
    if (maybe_response.Value().Status() == E_SUCCESS)
    {
      Log("PrePositioned (blocking) successfully");
      return true;
    }
    else
    {
      Log("Failed to preposition (blocking)");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool WSGInterface::PrePositionNonBlocking(const PrePositionStopMode stop_mode,
                                          const PrePositionMoveMode move_mode,
                                          double width_mm,
                                          double speed_mm_per_s)
{
  const WSGRawCommandMessage preposition_command
      = MakePrePositionCommand(stop_mode, move_mode, width_mm, speed_mm_per_s);
  const bool result = CommandGripper(preposition_command);
  return result;
}

PhysicalLimits WSGInterface::GetGripperPhysicalLimits()
{
  Log("Getting gripper physical limits...");
  const WSGRawCommandMessage physical_limits_command(kGetSystemLimits);
  const auto maybe_status
      = SendCommandAndAwaitStatus(physical_limits_command, 0.1);
  if (!maybe_status)
  {
    throw std::runtime_error("Did not receive status in timeout limit");
  }
  const std::vector<uint8_t>& param_buffer = maybe_status.Value().ParamBuffer();
  const double stroke_mm
      = DeserializeMemcpyable<float>(param_buffer, 0).Value();
  const double min_speed_mm_per_s
      = DeserializeMemcpyable<float>(param_buffer, 4).Value();
  const double max_speed_mm_per_s
      = DeserializeMemcpyable<float>(param_buffer, 8).Value();
  const double min_acc_mm_per_ss
      = DeserializeMemcpyable<float>(param_buffer, 12).Value();
  const double max_acc_mm_per_ss
      = DeserializeMemcpyable<float>(param_buffer, 16).Value();
  const double min_force
      = DeserializeMemcpyable<float>(param_buffer, 20).Value();
  const double nominal_force
      = DeserializeMemcpyable<float>(param_buffer, 24).Value();
  const double overdrive_force
      = DeserializeMemcpyable<float>(param_buffer, 28).Value();
  Log("...loaded physical limits from gripper");
  return PhysicalLimits(stroke_mm,
                        min_speed_mm_per_s,
                        max_speed_mm_per_s,
                        min_acc_mm_per_ss,
                        max_acc_mm_per_ss,
                        min_force,
                        nominal_force,
                        overdrive_force);
}

// Public interface

bool WSGInterface::InitializeGripper()
{
  Log("Initializing gripper...");
  // Enable periodic updates
  const uint16_t update_period_ms = 20;
  const double update_adjust_timeout = 0.25;
  bool success = true;
  Log("Enabling recurring status...");
  success &= EnableRecurringStatus(kGetSystemState,
                                   update_period_ms,
                                   update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to enable recurring kGetSystemState");
  }
  success &= EnableRecurringStatus(kGetGraspState,
                                   update_period_ms,
                                   update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to enable recurring kGetGraspState");
  }
  success &= EnableRecurringStatus(kGetOpeningWidth,
                                   update_period_ms,
                                   update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to enable recurring kGetOpeningWidth");
  }
  success &= EnableRecurringStatus(kGetSpeed,
                                   update_period_ms,
                                   update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to enable recurring kGetSpeed");
  }
  success &= EnableRecurringStatus(kGetForce,
                                   update_period_ms,
                                   update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to enable recurring kGetForce");
  }
  // Home the gripper
  Log("Homing the gripper...");
  success &= Home(kNegative);
  if (!success)
  {
    throw std::runtime_error("Failed to home kNegative");
  }
  success &= Home(kPositive);
  if (!success)
  {
    throw std::runtime_error("Failed to home kPositive");
  }
  success &= Tare();
  if (!success)
  {
    throw std::runtime_error("Failed to Tare");
  }
  // Get the physical limits
  const PhysicalLimits limits = GetGripperPhysicalLimits();
  Log(limits.Print());
  // Set all limits to max
  Log("Reseting gripper soft limits...");
  success &= ClearSoftLimits();
  if (!success)
  {
    throw std::runtime_error("Failed to ClearSoftLimits");
  }
  success &= SetAcceleration(limits.MinMaxAccelMMPerSS().second);
  if (!success)
  {
    throw std::runtime_error("Failed to SetAcceleration");
  }
  std::lock_guard<std::mutex> status_lock(status_mutex_);
  maybe_physical_limits_ = OwningMaybe<PhysicalLimits>(limits);
  return success;
}

void WSGInterface::Shutdown()
{
  Log("Shutting down gripper...");
  bool success = true;
  const double update_adjust_timeout = 0.25;
  success &= StopGripper();
  if (!success)
  {
    throw std::runtime_error("Failed to stop gripper");
  }
  success &= DisableRecurringStatus(kGetSystemState, update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to disable recurring kGetSystemState");
  }
  success &= DisableRecurringStatus(kGetGraspState, update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to disable recurring kGetGraspState");
  }
  success &= DisableRecurringStatus(kGetOpeningWidth, update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to disable recurring kGetOpeningWidth");
  }
  success &= DisableRecurringStatus(kGetSpeed, update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to disable recurring kGetSpeed");
  }
  success &= DisableRecurringStatus(kGetForce, update_adjust_timeout);
  if (!success)
  {
    throw std::runtime_error("Failed to disable recurring kGetForce");
  }
  ShutdownConnection();
}

double WSGInterface::GetCommandPositionMM(const double target_position,
                                          const PhysicalLimits& limits)
{
  const double target_position_mm = target_position * 1000.0;
  const double min_position_mm = 0.0;
  const double max_position_mm = limits.StrokeMM();
  if (target_position > max_position_mm)
  {
    Log("Target position > maximum of "
        + std::to_string(max_position_mm) + " mm");
    return max_position_mm;
  }
  else if (target_position < min_position_mm)
  {
    Log("Target position < minimum of "
        + std::to_string(min_position_mm) + " mm");
    return min_position_mm;
  }
  else
  {
    return target_position_mm;
  }
}

double WSGInterface::GetCommandSpeedMMpS(const double target_speed,
                                         const PhysicalLimits& limits)
{
  const double min_mmps = limits.MinMaxSpeedMMPerS().first;
  const double max_mmps = limits.MinMaxSpeedMMPerS().second;
  if (target_speed > 0.0)
  {
    const double target_speed_mmps = target_speed * 1000.0;
    if (target_speed_mmps > max_mmps)
    {
      Log("Target speed greater than using physical limits max speed of "
          + std::to_string(max_mmps) + " mm/s");
      return max_mmps;
    }
    else if (target_speed < min_mmps)
    {
      Log("Target speed lower than using physical limits min speed of "
          + std::to_string(min_mmps) + " mm/s");
      return min_mmps;
    }
    else
    {
      return target_speed_mmps;
    }
  }
  else
  {
    Log("Target speed <= 0.0, using physical limits max speed of "
        + std::to_string(max_mmps) + " mm/s");
    return max_mmps;
  }
}

double WSGInterface::GetCommandEffortN(const double target_effort,
                                       const PhysicalLimits& limits)
{
  const double min_effort = limits.MinForce();
  const double max_effort = limits.NominalForce();
  if (target_effort > 0.0)
  {
    if (target_effort > max_effort)
    {
      Log("Target effort greater than physical limits max effort of "
          + std::to_string(max_effort) + " N");
      return max_effort;
    }
    else if (target_effort < min_effort)
    {
      Log("Target effort lower than physical limits min effort of "
          + std::to_string(min_effort) + " N");
      return min_effort;
    }
    else
    {
      return target_effort;
    }
  }
  else
  {
    Log("Target effort <= 0.0, using physical limits max effort of "
        + std::to_string(max_effort) + " N");
    return max_effort;
  }
}

bool WSGInterface::SetTargetPositionSpeedEffort(const double target_position,
                                                const double max_speed,
                                                const double max_effort)
{
  RefreshGripperStatus();
  const double force_deadband = 5.0;
  const GripperMotionStatus current_status = GetGripperStatus();
  status_mutex_.lock();
  if (!maybe_physical_limits_)
  {
    throw std::runtime_error("Physical limits are not available");
  }
  const PhysicalLimits physical_limits = maybe_physical_limits_.Value();
  status_mutex_.unlock();
  bool is_new_command = false;
  if (std::abs(max_effort - current_status.ActualEffort()) > force_deadband)
  {
    is_new_command = true;
  }
  if (target_position > current_status.ActualPosition()
      && current_status.ActualPosition() > current_status.TargetPosition())
  {
    is_new_command = true;
  }
  else if (target_position < current_status.ActualPosition()
           && current_status.ActualPosition() < current_status.TargetPosition())
  {
    is_new_command = true;
  }
  if (is_new_command)
  {
    const double target_position_mm
        = GetCommandPositionMM(target_position, physical_limits);
    const double max_speed_mmps
        = GetCommandSpeedMMpS(max_speed, physical_limits);
    const double max_effort_n
        = GetCommandEffortN(max_effort, physical_limits);
    bool success = true;
    success &= SetForceLimitNonBlocking(max_effort_n);
    success &= PrePositionNonBlocking(kPrePositionClampOnBlock,
                                      kPrePositionAbsolute,
                                      target_position_mm,
                                      max_speed_mmps);
    // Update motion status
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    motion_status_.UpdateTargetPositionSpeedEffort(target_position,
                                                   max_speed,
                                                   max_effort);
    return success;
  }
  else
  {
    return true;
  }
}

GripperMotionStatus WSGInterface::GetGripperStatus()
{
  std::lock_guard<std::mutex> status_lock(status_mutex_);
  // The Schunk gripper only returns a positive value for force,
  // so we invert when the direction of motion is reversed
  if (motion_status_.ActualVelocity() <= 0.0)
  {
    motion_status_.UpdateActualEffort(-motion_status_.ActualEffort());
  }
  return motion_status_;
}

void WSGInterface::RefreshGripperStatus()
{
  // Get the queued status messages and process them
  const std::vector<WSGRawStatusMessage> status_queue = GetStatusQueue();
  std::lock_guard<std::mutex> status_lock(status_mutex_);
  for (size_t idx = 0; idx < status_queue.size(); idx++)
  {
    const WSGRawStatusMessage& message = status_queue[idx];
    if (message.Status() != E_SUCCESS)
    {
      continue;
    }
    const std::vector<uint8_t>& param_buffer = message.ParamBuffer();
    if (message.Command() == kGetOpeningWidth)
    {
      const double opening_width_mm
          = DeserializeMemcpyable<float>(param_buffer, 0).Value();
      const double opening_width = opening_width_mm * 0.001;
      motion_status_.UpdateActualPosition(opening_width);
    }
    else if (message.Command() == kGetForce)
    {
      const double force
          = DeserializeMemcpyable<float>(param_buffer, 0).Value();
      motion_status_.UpdateActualEffort(force);
    }
    else if (message.Command() == kGetSpeed)
    {
      const double speed_mm_s
          = DeserializeMemcpyable<float>(param_buffer, 0).Value();
      const double speed = speed_mm_s * 0.001;
      motion_status_.UpdateActualVelocity(speed);
    }
  }
}
}
