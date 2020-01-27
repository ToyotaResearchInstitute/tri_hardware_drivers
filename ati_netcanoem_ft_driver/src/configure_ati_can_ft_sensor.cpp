#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <ati_netcanoem_ft_driver/ati_netcanoem_ft_driver.hpp>

int main(int argc, char** argv)
{
  if (argc >= 5)
  {
    const std::string can_interface(argv[1]);
    const uint8_t sensor_base_can_id
      = static_cast<uint8_t>(std::atoi(argv[2]));
    const uint8_t new_sensor_base_can_id
      = static_cast<uint8_t>(std::atoi(argv[3]));
    const uint8_t new_sensor_can_baud_rate_divisor
      = static_cast<uint8_t>(std::atoi(argv[4]));
    std::cout << "Connecting to ATI F/T sensor with CAN base ID "
              << sensor_base_can_id << " on socketcan interface "
              << can_interface << std::endl;
    std::function<void(const std::string&)> logging_fn =
      [] (const std::string& message)
    { std::cout << message << std::endl; };
    ati_netcanoem_ft_driver::AtiNetCanOemInterface sensor(logging_fn,
                                can_interface,
                                sensor_base_can_id);
    const std::string serial_num = sensor.ReadSerialNumber();
    const auto firmware_version = sensor.ReadFirmwareVersion();
    std::cout << "Connected to sensor with serial # " << serial_num
              << " and firmware version "  << firmware_version.MajorVersion()
              << " (major version) " << firmware_version.MinorVersion()
              << " (minor version) " << firmware_version.BuildNumber()
              << " (build)" << std::endl;
    std::cout << "Setting sensor CAN base ID to "
              << new_sensor_base_can_id << std::endl;
    const bool set_base_can_id
        = sensor.SetSensorBaseCanID(new_sensor_base_can_id);
    if (set_base_can_id == false)
    {
      std::cerr << "Failed to set new CAN base ID" << std::endl;
      return 1;
    }
    std::cout << "Setting sensor CAN baud rate divisor to "
              << new_sensor_can_baud_rate_divisor << std::endl;
    const bool set_sensor_can_baud_rate_divisor
        = sensor.SetSensorCanRate(new_sensor_can_baud_rate_divisor);
    if (set_sensor_can_baud_rate_divisor == false)
    {
      std::cerr << "Failed to set new CAN baud rate divisor" << std::endl;
      return 1;
    }
    std::cout << "New settings applied, power cycle sensor" << std::endl;
    return 0;
  }
  else
  {
    std::cerr << "You must provide 4 arguments: <socketcan interface name>"
                 " <current sensor base can id> <new sensor base can id>"
                 " <new sensor can baud rate divisor>" << std::endl;
    return -1;
  }
}
