#include <robotiq_3_finger_gripper_driver/robotiq_3_finger_gripper_driver.hpp>
#include <stdint.h>
#include <tri_driver_common/print.hpp>

namespace robotiq_3_finger_gripper_driver
{
Robotiq3FingerGripperInterface
::Robotiq3FingerGripperInterface(
    const std::function<void(const std::string&)>& logging_fn)
  : logging_fn_(logging_fn) {}

Robotiq3FingerGripperStatus
Robotiq3FingerGripperInterface::GetGripperStatus()
{
  const std::vector<uint8_t> received_registers = ReadRIGORegisters();
  const Robotiq3FingerGripperStatus status(received_registers);
  return status;
}

bool Robotiq3FingerGripperInterface::SendGripperCommand(
    const Robotiq3FingerGripperCommand& command)
{
  const Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
  if (gripper_status.IsActivated())
  {
    // First, stop the gripper
    std::vector<uint8_t> stop_gripper_command(NUM_ROBOTIQ_REGISTERS, 0x00);
    stop_gripper_command[0] = 0x01;
    const bool stop_sent = WriteROGIRegisters(stop_gripper_command);
    if (stop_sent == false)
    {
      return false;
    }
    // Second, set the target position, speed, and force
    std::vector<uint8_t> set_command(NUM_ROBOTIQ_REGISTERS, 0x00);
    // Set the action request (activated, GTO=false)
    set_command[0] = 0b00000001;
    // Set the gripper option (ICS=true, ICF=true)
    set_command[1] = 0b00001100;
    set_command[2] = 0x00;
    // Set finger A command
    set_command[3] = command.FingerACommand().PositionCommand();
    set_command[4] = command.FingerACommand().SpeedCommand();
    set_command[5] = command.FingerACommand().ForceCommand();
    // Set finger b command
    set_command[6] = command.FingerBCommand().PositionCommand();
    set_command[7] = command.FingerBCommand().SpeedCommand();
    set_command[8] = command.FingerBCommand().ForceCommand();
    // Set finger C command
    set_command[9] = command.FingerCCommand().PositionCommand();
    set_command[10] = command.FingerCCommand().SpeedCommand();
    set_command[11] = command.FingerCCommand().ForceCommand();
    // Set scissor command
    set_command[12] = command.ScissorCommand().PositionCommand();
    set_command[13] = command.ScissorCommand().SpeedCommand();
    set_command[14] = command.ScissorCommand().ForceCommand();
    // Send set command
    const bool set_sent = WriteROGIRegisters(set_command);
    if (set_sent == false)
    {
      return false;
    }
    // Third, restart the gripper
    std::vector<uint8_t> restart_command = set_command;
    // Set the action request (activated, GTO=true)
    restart_command[0] = 0b00001001;
    const bool restart_sent = WriteROGIRegisters(restart_command);
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

bool Robotiq3FingerGripperInterface::CommandGripperBlocking(
    const Robotiq3FingerGripperCommand& command)
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
    const Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
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

bool Robotiq3FingerGripperInterface::ReactivateGripper()
{
  Log("Reinitializing/activating the gripper...");
  // First, reset the gripper
  const std::vector<uint8_t> reset_command(NUM_ROBOTIQ_REGISTERS, 0x00);
  const bool reset_sent = WriteROGIRegisters(reset_command);
  if (reset_sent == false)
  {
    return false;
  }
  // Wait a little
  std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  // Second, activate the gripper
  std::vector<uint8_t> activate_command(NUM_ROBOTIQ_REGISTERS, 0x00);
  activate_command[0] = 0x01;
  const bool activate_sent = WriteROGIRegisters(activate_command);
  if (activate_sent == false)
  {
    return false;
  }
  // Wait for the gripper to finish activating
  Log("Waiting for activation to complete...");
  do
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    Log(GetGripperStatus().Print());
  }
  while (!GetGripperStatus().IsActivated());
  // Make sure activation completed successfully
  const Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
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

bool Robotiq3FingerGripperInterface::ActivateGripper()
{
  const Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
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

Robotiq3FingerGripperModbusInterface::Robotiq3FingerGripperModbusInterface(
    const std::function<void (const std::string &)>& logging_fn)
  : Robotiq3FingerGripperInterface(logging_fn)
{
  modbus_interface_ptr_ = nullptr;
  rogi_first_register_ = 0x0000;
  rigo_first_register_ = 0x0000;
  interface_type_ = NONE;
}

Robotiq3FingerGripperModbusInterface::~Robotiq3FingerGripperModbusInterface()
{
  ShutdownConnection();
}

void Robotiq3FingerGripperModbusInterface
::ConnectModbusTcp(const std::string& gripper_ip_address,
                   const int32_t gripper_port,
                   const uint16_t gripper_slave_id)
{
  rogi_first_register_ = 0x0000;
  rigo_first_register_ = 0x0000;
  if (modbus_interface_ptr_ != nullptr)
  {
    ShutdownConnection();
  }
  modbus_interface_ptr_
      = modbus_new_tcp(gripper_ip_address.c_str(), gripper_port);
  if (modbus_interface_ptr_ == nullptr)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to create Modbus TCP interface to "
                             + gripper_ip_address + " at port "
                             + std::to_string(gripper_port)
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
  interface_type_ = TCP;
}

void Robotiq3FingerGripperModbusInterface
::ConnectModbusRtu(const std::string& modbus_rtu_interface,
                   const int32_t gripper_baud_rate,
                   const uint16_t gripper_slave_id)
{
  rogi_first_register_ = 0x03e8;
  rigo_first_register_ = 0x07d0;
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
  interface_type_ = RTU;
}

uint16_t MakeRegisterValue(const uint8_t low_byte, const uint8_t high_byte)
{
  return (uint16_t)((uint16_t)low_byte | (uint16_t)((uint16_t)high_byte << 8));
}

bool Robotiq3FingerGripperModbusInterface::WriteROGIRegisters(
    const std::vector<uint8_t>& register_values)
{
  if (register_values.size() != NUM_ROBOTIQ_REGISTERS)
  {
    Log("Command with register_values.size() != NUM_REGISTERS");
    return false;
  }
  // Assemble registers
  std::vector<uint16_t> modbus_register_values(NUM_MODBUS_REGISTERS, 0x0000);
  modbus_register_values[0]
      = MakeRegisterValue(register_values[0], register_values[1]);
  modbus_register_values[1]
      = MakeRegisterValue(register_values[2], register_values[3]);
  modbus_register_values[2]
      = MakeRegisterValue(register_values[4], register_values[5]);
  modbus_register_values[3]
      = MakeRegisterValue(register_values[6], register_values[7]);
  modbus_register_values[4]
      = MakeRegisterValue(register_values[8], register_values[9]);
  modbus_register_values[5]
      = MakeRegisterValue(register_values[10], register_values[11]);
  modbus_register_values[6]
      = MakeRegisterValue(register_values[12], register_values[13]);
  modbus_register_values[7]
      = MakeRegisterValue(register_values[14], 0x00);
  // Send
  const int num_registers = (int)modbus_register_values.size();
  const int registers_written
      = modbus_write_registers(modbus_interface_ptr_,
                               rogi_first_register_,
                               num_registers,
                               modbus_register_values.data());
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

std::vector<uint8_t>
Robotiq3FingerGripperModbusInterface::ReadRIGORegisters()
{
  std::vector<uint16_t> raw_status_buffer(NUM_MODBUS_REGISTERS, 0x0000);
  int ret = 0;
  if (interface_type_ == TCP)
  {
    ret = modbus_read_input_registers(modbus_interface_ptr_,
                                      rigo_first_register_,
                                      NUM_MODBUS_REGISTERS,
                                      raw_status_buffer.data());
  }
  else if (interface_type_ == RTU)
  {
    ret = modbus_read_registers(modbus_interface_ptr_,
                                rigo_first_register_,
                                NUM_MODBUS_REGISTERS,
                                raw_status_buffer.data());
  }
  if (ret != NUM_MODBUS_REGISTERS)
  {
    const std::string error_msg(modbus_strerror(errno));
    throw std::runtime_error("Failed to read status registers with error: "
                             + error_msg);
  }
  Log(tri_driver_common::print::Print(raw_status_buffer));
  std::vector<uint8_t> received_bytes(NUM_MODBUS_REGISTERS * 2, 0x00);
  for (size_t rdx = 0, bdx = 0; rdx < raw_status_buffer.size(); rdx++, bdx += 2)
  {
    const uint16_t raw_register = raw_status_buffer[rdx];
    received_bytes[bdx + 1] = (uint8_t)((raw_register & 0xff00) >> 8);
    received_bytes[bdx + 0] = (uint8_t)(raw_register & 0x00ff);
  }
  Log(tri_driver_common::print::Print(received_bytes));
  return received_bytes;
}

void Robotiq3FingerGripperModbusInterface::ShutdownConnection()
{
  Log("Closing modbus connection...");
  modbus_close(modbus_interface_ptr_);
  modbus_free(modbus_interface_ptr_);
  modbus_interface_ptr_ = nullptr;
  Log("...finished cleanup");
  interface_type_ = NONE;
}
}
