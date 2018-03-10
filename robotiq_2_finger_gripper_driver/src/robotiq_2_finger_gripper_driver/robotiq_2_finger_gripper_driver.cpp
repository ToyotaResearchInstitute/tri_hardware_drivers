#include <robotiq_2_finger_gripper_driver/robotiq_2_finger_gripper_driver.hpp>
#include <stdint.h>

namespace robotiq_2_finger_gripper_driver
{
Robotiq2FingerGripperModbusRtuInterface
::Robotiq2FingerGripperModbusRtuInterface(
    const std::function<void(const std::string&)>& logging_fn,
    const std::string& modbus_rtu_interface,
    const int32_t gripper_baud_rate,
    const uint16_t gripper_slave_id)
  : logging_fn_(logging_fn)
{
  const int data_bits = 8;
  const int stop_bits = 1;
  const char parity = 'N';
  modbus_interface_ptr_ = nullptr;
  modbus_interface_ptr_ = modbus_new_rtu(modbus_rtu_interface.c_str(),
                                         gripper_baud_rate,
                                         parity,
                                         data_bits,
                                         stop_bits);
  if (modbus_interface_ptr_ == nullptr)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to create Modbus RTU interface to "
                             + modbus_rtu_interface
                             + " with error: " + error_msg);
  }
  const int mss_ret = modbus_set_slave(modbus_interface_ptr_,
                                       (int)gripper_slave_id);
  if (mss_ret != 0)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Invalid Modbus slave address "
                             + std::to_string(gripper_slave_id)
                             + " with error: " + error_msg);
  }
  const int mc_ret = modbus_connect(modbus_interface_ptr_);
  if (mc_ret != 0)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to connect with error: " + error_msg);
  }
}

Robotiq2FingerGripperModbusRtuInterface
::~Robotiq2FingerGripperModbusRtuInterface()
{
  ShutdownConnection();
}

Robotiq2FingerGripperStatus
Robotiq2FingerGripperModbusRtuInterface::GetGripperStatus()
{
  std::vector<uint16_t> raw_status_buffer(3, 0x0000);
  const int ret = modbus_read_registers(modbus_interface_ptr_,
                                        RIGO_FIRST_REGISTER,
                                        3,
                                        raw_status_buffer.data());
  if (ret != 3)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to read status registers with error: "
                             + error_msg);
  }
  const std::vector<uint8_t> received_bytes
      = {(uint8_t)((raw_status_buffer[0] & 0xff00) >> 8),
         (uint8_t)(raw_status_buffer[0] & 0x00ff),
         (uint8_t)((raw_status_buffer[1] & 0xff00) >> 8),
         (uint8_t)(raw_status_buffer[1] & 0x00ff),
         (uint8_t)((raw_status_buffer[2] & 0xff00) >> 8),
         (uint8_t)(raw_status_buffer[2] & 0x00ff)};
  const Robotiq2FingerGripperStatus status(received_bytes);
  return status;
}

bool Robotiq2FingerGripperModbusRtuInterface::SendGripperCommand(
    const Robotiq2FingerGripperCommand& command)
{
  const Robotiq2FingerGripperStatus gripper_status = GetGripperStatus();
  if (gripper_status.IsActivated())
  {
    // First, stop the gripper
    const std::vector<uint16_t> stop_gripper_command = {0x0100, 0x0000, 0x0000};
    const bool stop_sent = WriteMultipleRegisters(ROGI_FIRST_REGISTER,
                                                  stop_gripper_command);
    if (stop_sent == false)
    {
      return false;
    }
    // Second, set the target position, speed, and force
    // One reserved byte
    const uint16_t byte_0 = 0b00000000;
    // Position request
    const uint16_t byte_1 = command.PositionCommand();
    // Speed request
    const uint16_t byte_2 = command.SpeedCommand();
    // Force request
    const uint16_t byte_3 = command.ForceCommand();
    // Assemble
    const uint16_t command_register_1 = byte_1 | (uint16_t)(byte_0 << 8);
    const uint16_t command_register_2 = byte_3 | (uint16_t)(byte_2 << 8);
    // Send
    const std::vector<uint16_t> set_command = {0x0100,
                                               command_register_1,
                                               command_register_2};
    const bool set_sent = WriteMultipleRegisters(ROGI_FIRST_REGISTER,
                                                 set_command);
    if (set_sent == false)
    {
      return false;
    }
    // Third, restart the gripper
    const std::vector<uint16_t> restart_command = {0x0900,
                                                   command_register_1,
                                                   command_register_2};
    const bool restart_sent = WriteMultipleRegisters(ROGI_FIRST_REGISTER,
                                                     restart_command);
    if (restart_sent == false)
    {
      return false;
    }
    return true;
  }
  else
  {
    Log("Gripper is not activated, cannot command motion");
    return false;
  }
}

bool Robotiq2FingerGripperModbusRtuInterface::CommandGripperBlocking(
    const Robotiq2FingerGripperCommand& command)
{
  if (SendGripperCommand(command))
  {
    // Wait for the motion to finish
    do
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
    }
    while (GetGripperStatus().IsStopped() != true);
    // Make sure we finished with no faults
    const Robotiq2FingerGripperStatus gripper_status = GetGripperStatus();
    if (gripper_status.IsActivated() && (gripper_status.HasFault() == false))
    {
      return true;
    }
    else
    {
      Log("Motion command failed or faulted");
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool Robotiq2FingerGripperModbusRtuInterface::ReactivateGripper()
{
  Log("Reinitializing/activating the gripper...");
  // First, reset the gripper
  const std::vector<uint16_t> reset_command = {0x0000, 0x0000, 0x0000};
  const bool reset_sent = WriteMultipleRegisters(ROGI_FIRST_REGISTER,
                                                 reset_command);
  if (reset_sent == false)
  {
    return false;
  }
  // Wait a little
  std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  // Second, activate the gripper
  const std::vector<uint16_t> activate_command = {0x0100, 0x0000, 0x0000};
  const bool activate_sent = WriteMultipleRegisters(ROGI_FIRST_REGISTER,
                                                    activate_command);
  if (activate_sent == false)
  {
    return false;
  }
  // Wait for the gripper to finish activating
  Log("Waiting for activation to complete...");
  do
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  }
  while (GetGripperStatus().GripperStatus()
         != Robotiq2FingerGripperStatus::ACTIVATION_COMPLETED);
  // Make sure activation completed successfully
  const Robotiq2FingerGripperStatus gripper_status = GetGripperStatus();
  if (gripper_status.IsActivated() && (gripper_status.HasFault() == false))
  {
    Log("...gripper activation finished successfully");
    return true;
  }
  else
  {
    Log("...gripper activation failed or finished with a fault");
    return false;
  }
}

bool Robotiq2FingerGripperModbusRtuInterface::ActivateGripper()
{
  const Robotiq2FingerGripperStatus gripper_status = GetGripperStatus();
  if (gripper_status.IsActivated())
  {
    Log("Gripper is already activated, no need to reactivate");
    return true;
  }
  else
  {
    return ReactivateGripper();
  }
}

bool Robotiq2FingerGripperModbusRtuInterface::WriteMultipleRegisters(
    const uint16_t start_register,
    const std::vector<uint16_t>& register_values)
{
  const int num_registers = (int)register_values.size();
  const int registers_written = modbus_write_registers(modbus_interface_ptr_,
                                                       start_register,
                                                       num_registers,
                                                       register_values.data());
  if (registers_written == -1)
  {
    const std::string error_msg(modbus_strerror(errno));
    Log("modbus_write_registers error: " + error_msg);
    return false;
  }
  else if (registers_written != num_registers)
  {
    Log("Modbus wrote " + std::to_string(registers_written)
        + " registers, expected to write "
        + std::to_string(num_registers) + " registers");
    return false;
  }
  else
  {
    return true;
  }
}

void Robotiq2FingerGripperModbusRtuInterface::ShutdownConnection()
{
  Log("Closing modbus connection...");
  modbus_close(modbus_interface_ptr_);
  modbus_free(modbus_interface_ptr_);
  Log("...finished cleanup");
}
}
