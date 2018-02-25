#include <robotiq_2_finger_gripper_driver/robotiq_2_finger_gripper_driver.hpp>
#include <stdint.h>

namespace robotiq_2_finger_gripper_driver
{
    Robotiq2FingerGripperModbusRtuInterface::Robotiq2FingerGripperModbusRtuInterface(const std::function<void(const std::string&)>& logging_fn, const std::string& modbus_rtu_interface, const uint16_t gripper_slave_id) : logging_fn_(logging_fn)
    {
        const int baud_rate = 115200;
        const int data_bits = 8;
        const int stop_bits = 1;
        const char parity = 'N';
        modbus_interface_ptr_ = nullptr;
        modbus_interface_ptr_ = modbus_new_rtu(modbus_rtu_interface.c_str(), baud_rate, parity, data_bits, stop_bits);
        if (modbus_interface_ptr_ == nullptr)
        {
            const std::string error_msg(modbus_strerror(errno));
            throw std::runtime_error("Failed to create Modbus RTU interface to " + modbus_rtu_interface + " with error: " + error_msg);
        }
        const int mss_ret = modbus_set_slave(modbus_interface_ptr_, (int)gripper_slave_id);
        if (mss_ret != 0)
        {
            const std::string error_msg(modbus_strerror(errno));
            throw std::runtime_error("Invalid Modbus slave address " + std::to_string(gripper_slave_id) + " with error: " + error_msg);
        }
        const int mc_ret = modbus_connect(modbus_interface_ptr_);
        if (mc_ret != 0)
        {
            const std::string error_msg(modbus_strerror(errno));
            throw std::runtime_error("Failed to connect with error: " + error_msg);
        }
        struct timeval response_timeout;
        struct timeval byte_timeout;
        modbus_get_response_timeout(modbus_interface_ptr_, &response_timeout);
        modbus_get_byte_timeout(modbus_interface_ptr_, &byte_timeout);
        Log("Modbus response timeout " + std::to_string(response_timeout.tv_sec) + " (s) " + std::to_string(response_timeout.tv_usec) + " (us)");
        Log("Modbus byte timeout " + std::to_string(byte_timeout.tv_sec) + " (s) " + std::to_string(byte_timeout.tv_usec) + " (us)");
        struct timeval new_response_timeout;
        new_response_timeout.tv_sec = 10;
        new_response_timeout.tv_usec = 0;
        struct timeval new_byte_timeout;
        new_byte_timeout.tv_sec = 10;
        new_byte_timeout.tv_usec = 0;
        modbus_set_response_timeout(modbus_interface_ptr_, &new_response_timeout);
        modbus_set_byte_timeout(modbus_interface_ptr_, &new_byte_timeout);
    }

    Robotiq2FingerGripperModbusRtuInterface::~Robotiq2FingerGripperModbusRtuInterface()
    {
        ShutdownConnection();
    }

    Robotiq2FingerGripperStatus Robotiq2FingerGripperModbusRtuInterface::GetGripperStatus()
    {
        std::vector<uint16_t> raw_status_buffer(3, 0x0000);
        const int ret = modbus_read_registers(modbus_interface_ptr_, RIGO_FIRST_REGISTER, 3, raw_status_buffer.data());
        if (ret != 3)
        {
            const std::string error_msg(modbus_strerror(errno));
            throw std::runtime_error("Failed to read status registers with error: " + error_msg);
        }
        const std::vector<uint8_t> received_bytes = {(uint8_t)((raw_status_buffer[0] & 0xff00) >> 8),
                                               (uint8_t)(raw_status_buffer[0] & 0x00ff),
                                               (uint8_t)((raw_status_buffer[1] & 0xff00) >> 8),
                                               (uint8_t)(raw_status_buffer[1] & 0x00ff),
                                               (uint8_t)((raw_status_buffer[2] & 0xff00) >> 8),
                                               (uint8_t)(raw_status_buffer[2] & 0x00ff)};
        //Log("Raw status bytes: " + std::to_string(received_bytes[0]) + "," + std::to_string(received_bytes[1])
        //                   + "," + std::to_string(received_bytes[2]) + "," + std::to_string(received_bytes[3])
        //                   + "," + std::to_string(received_bytes[4]) + "," + std::to_string(received_bytes[5]));
        const Robotiq2FingerGripperStatus status(received_bytes);
        //Log(status.Print());
        return status;
    }

    bool Robotiq2FingerGripperModbusRtuInterface::SendGripperCommand(const Robotiq2FingerGripperCommand& command)
    {
        const Robotiq2FingerGripperStatus gripper_status = GetGripperStatus();
        if (gripper_status.IsActivated())
        {
            // First, stop the gripper
            const std::vector<uint16_t> stop_gripper_command = {0x0100, 0x0000, 0x0000};
            const bool stop_sent = RobotiqCompatibleWriteMultipleRegisters(ROGI_FIRST_REGISTER, stop_gripper_command);
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
            const std::vector<uint16_t> set_command = {0x0100, command_register_1, command_register_2};
            const bool set_sent = RobotiqCompatibleWriteMultipleRegisters(ROGI_FIRST_REGISTER, set_command);
            if (set_sent == false)
            {
                return false;
            }
            // Third, restart the gripper
            const std::vector<uint16_t> restart_command = {0x0900, command_register_1, command_register_2};
            const bool restart_sent = RobotiqCompatibleWriteMultipleRegisters(ROGI_FIRST_REGISTER, restart_command);
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

    bool Robotiq2FingerGripperModbusRtuInterface::CommandGripperBlocking(const Robotiq2FingerGripperCommand& command)
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
        const bool reset_sent = RobotiqCompatibleWriteMultipleRegisters(ROGI_FIRST_REGISTER, reset_command);
        if (reset_sent == false)
        {
            return false;
        }
        // Wait a little
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
        // Second, activate the gripper
        const std::vector<uint16_t> activate_command = {0x0100, 0x0000, 0x0000};
        const bool activate_sent = RobotiqCompatibleWriteMultipleRegisters(ROGI_FIRST_REGISTER, activate_command);
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
        while (GetGripperStatus().GripperStatus() != Robotiq2FingerGripperStatus::ACTIVATION_COMPLETED);
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

    bool Robotiq2FingerGripperModbusRtuInterface::RobotiqCompatibleWriteMultipleRegisters(const uint16_t start_register, const std::vector<uint16_t>& register_values)
    {
        std::vector<uint8_t> raw_request;
        raw_request.push_back(0x09); // slave address
        raw_request.push_back(0x10); // Function code 16 "preset multiple registers"
        raw_request.push_back((uint8_t)((start_register & 0xff00) >> 8)); // Top byte of register #
        raw_request.push_back((uint8_t)(start_register & 0x00ff)); // Bottom byte of register #
        const uint16_t num_registers = (uint16_t)register_values.size();
        raw_request.push_back((uint8_t)((num_registers & 0xff00) >> 8)); // Top byte of # registers
        raw_request.push_back((uint8_t)(num_registers & 0x00ff)); // Bottom byte of # registers
        for (size_t idx = 0; idx < register_values.size(); idx++)
        {
            const uint16_t register_value = register_values[idx];
            raw_request.push_back((uint8_t)((register_value & 0xff00) >> 8)); // Top byte of # register value
            raw_request.push_back((uint8_t)(register_value & 0x00ff)); // Bottom byte of # register value
        }
        const int request_length = modbus_send_raw_request(modbus_interface_ptr_, raw_request.data(), (int)raw_request.size());
        if (request_length == -1)
        {
            const std::string error_msg(modbus_strerror(errno));
            Log("Modbus send_raw_request error: " + error_msg);
            //return false;
        }
        std::vector<uint8_t> response(MODBUS_RTU_MAX_ADU_LENGTH, 0x00);
        const int response_len = modbus_receive_confirmation(modbus_interface_ptr_, response.data());
        if (response_len == -1)
        {
            const std::string error_msg(modbus_strerror(errno));
            Log("Modbus receive_confirmation error: " + error_msg);
            //return false;
        }
        return true;
    }

    void Robotiq2FingerGripperModbusRtuInterface::ShutdownConnection()
    {
        Log("Closing modbus connection...");
        modbus_close(modbus_interface_ptr_);
        modbus_free(modbus_interface_ptr_);
        Log("...finished cleanup");
    }
}
