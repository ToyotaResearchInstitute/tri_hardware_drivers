#include <ati_netcanoem_ft_driver/ati_netcanoem_ft_driver.hpp>
#include <common_robotics_utilities/serialization.hpp>

namespace ati_netcanoem_ft_driver
{
const int32_t OPCODE_BITS = 4;

AtiNetCanOemInterface::AtiNetCanOemInterface(
    const std::function<void(const std::string&)>& logging_fn,
    const std::string& socketcan_interface,
    const uint8_t sensor_base_can_id)
  : logging_fn_(logging_fn), has_active_calibration_(false)
{
  if (sensor_base_can_id > 0x7f)
  {
    throw std::invalid_argument("Base CAN ID is greater than 7 bits");
  }
  sensor_base_can_id_ = sensor_base_can_id;
  // Make the socketcan socket
  Log("Opening CAN socket...");
  can_socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_fd_ <= 0)
  {
    perror(NULL);
    throw std::runtime_error("Failed to create socketcan socket");
  }
  Log("...CAN socket opened");
  // Locate the desired socketcan interface
  struct ifreq interface;
  // Figure out how much we can write
  const size_t max_ifr_name_length = IFNAMSIZ - 1;
  strncpy(interface.ifr_name, socketcan_interface.c_str(), max_ifr_name_length);
  interface.ifr_name[IFNAMSIZ - 1] = 0x00; // Make sure it is null-terminated
  // Invoke ioctl to find the index of the interface
  ioctl(can_socket_fd_, SIOCGIFINDEX, &interface);
  // Set interface options - we filter only the CAN IDs we care about
  struct can_filter filter[14];
  filter[0].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x0;
  filter[0].can_mask = CAN_SFF_MASK;
  filter[1].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x1;
  filter[1].can_mask = CAN_SFF_MASK;
  filter[2].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x2;
  filter[2].can_mask = CAN_SFF_MASK;
  filter[3].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x3;
  filter[3].can_mask = CAN_SFF_MASK;
  filter[4].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x4;
  filter[4].can_mask = CAN_SFF_MASK;
  filter[5].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x5;
  filter[5].can_mask = CAN_SFF_MASK;
  filter[6].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x6;
  filter[6].can_mask = CAN_SFF_MASK;
  filter[7].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x7;
  filter[7].can_mask = CAN_SFF_MASK;
  filter[8].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x8;
  filter[8].can_mask = CAN_SFF_MASK;
  filter[9].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0x9;
  filter[9].can_mask = CAN_SFF_MASK;
  filter[10].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0xc;
  filter[10].can_mask = CAN_SFF_MASK;
  filter[11].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0xd;
  filter[11].can_mask = CAN_SFF_MASK;
  filter[12].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0xe;
  filter[12].can_mask = CAN_SFF_MASK;
  filter[13].can_id   = (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | 0xf;
  filter[13].can_mask = CAN_SFF_MASK;
  // Apply the filters
  Log("Setting CAN ID filters...");
  const int setsockopt_result = setsockopt(can_socket_fd_,
                                           SOL_CAN_RAW,
                                           CAN_RAW_FILTER,
                                           &filter,
                                           sizeof(filter));
  if (setsockopt_result != 0)
  {
    perror(NULL);
    throw std::runtime_error("setsockopt failed");
  }
  Log("...set CAN ID filters");
  Log("Setting socket timeout...");
  struct timeval read_timeout;
  read_timeout.tv_sec = 1;
  read_timeout.tv_usec = 0;
  const int setsockopt_timeout_result
      = setsockopt(can_socket_fd_,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   &read_timeout,
                   sizeof(read_timeout));
  if (setsockopt_timeout_result != 0)
  {
    perror(NULL);
    throw std::runtime_error("setsockopt timeout configuration failed");
  }
  Log("...socket timeout set");
  // Bind the socket to the interface
  struct sockaddr_can can_interface;
  can_interface.can_family = AF_CAN;
  can_interface.can_ifindex = interface.ifr_ifindex;
  Log("Binding to CAN interface...");
  const int bind_result = bind(can_socket_fd_,
                               (struct sockaddr *)&can_interface,
                               sizeof(can_interface));
  if (bind_result != 0)
  {
    throw std::runtime_error("Failed to bind socketcan socket");
  }
  Log("...bound to CAN interface");
  ResetBias();
}

AtiNetCanOemInterface::~AtiNetCanOemInterface()
{
  ShutdownConnection();
}

Eigen::Matrix<double, 6, 1> AtiNetCanOemInterface::GetCurrentForceTorque()
{
  if (has_active_calibration_)
  {
    const std::pair<uint16_t, Eigen::Matrix<double, 6, 1>> strain_gauge_data
        = ReadRawStrainGaugeData();
    const uint16_t status_code = strain_gauge_data.first;
    ParseStatusCode(status_code);
    const Eigen::Matrix<double, 6, 1>& strain_gauge_values
        = strain_gauge_data.second;
    const Eigen::Matrix<double, 6, 1> wrench_counts
        = active_calibration_matrix_ * (strain_gauge_values - active_bias_);
    const Eigen::Matrix<double, 6, 1> wrench
        = wrench_counts.cwiseProduct(active_force_torque_inv_counts_vector_);
    return wrench;
  }
  else
  {
    throw std::runtime_error("No active calibration to use");
  }
}

void AtiNetCanOemInterface::SetBias()
{
  const std::pair<uint16_t, Eigen::Matrix<double, 6, 1>> strain_gauge_data
      = ReadRawStrainGaugeData();
  const uint16_t status_code = strain_gauge_data.first;
  ParseStatusCode(status_code);
  const Eigen::Matrix<double, 6, 1>& strain_gauge_values
      = strain_gauge_data.second;
  active_bias_ = strain_gauge_values;
}

std::pair<uint16_t, Eigen::Matrix<double, 6, 1>>
AtiNetCanOemInterface::ReadRawStrainGaugeData()
{
  const DataElement read_strain_gauges(READ_SG_A);
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(read_strain_gauges, 0x02, 0.1);
  Eigen::Matrix<double, 6, 1> raw_values = Eigen::Matrix<double, 6, 1>::Zero();
  if (response.size() != 2)
  {
    throw std::runtime_error("Failed to read strain gauges in timeout");
  }
  const DataElement& response_msg_1 = response[0];
  if (response_msg_1.Opcode() != READ_SG_A)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  if (response_msg_1.Payload().size() != 8)
  {
    throw std::runtime_error("Invalid payload data for read strain gauges");
  }
  const uint16_t status_code
      = common_robotics_utilities::serialization
        ::DeserializeNetworkMemcpyable<uint16_t>(
            response_msg_1.Payload(), 0).first;
  raw_values(0, 0)
      = (double)common_robotics_utilities::serialization
        ::DeserializeNetworkMemcpyable<int16_t>(
            response_msg_1.Payload(), 2).first;
  raw_values(2, 0)
      = (double)common_robotics_utilities::serialization
        ::DeserializeNetworkMemcpyable<int16_t>(
            response_msg_1.Payload(), 4).first;
  raw_values(4, 0)
      = (double)common_robotics_utilities::serialization
        ::DeserializeNetworkMemcpyable<int16_t>(
            response_msg_1.Payload(), 6).first;
  const DataElement& response_msg_2 = response[1];
  if (response_msg_2.Opcode() != READ_SG_B)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  if (response_msg_2.Payload().size() != 6)
  {
    throw std::runtime_error("Invalid payload data for read strain gauges");
  }
  raw_values(1, 0)
      = (double)common_robotics_utilities::serialization
        ::DeserializeNetworkMemcpyable<int16_t>(
            response_msg_2.Payload(), 0).first;
  raw_values(3, 0)
      = (double)common_robotics_utilities::serialization
        ::DeserializeNetworkMemcpyable<int16_t>(
            response_msg_2.Payload(), 2).first;
  raw_values(5, 0)
      = (double)common_robotics_utilities::serialization
        ::DeserializeNetworkMemcpyable<int16_t>(
            response_msg_2.Payload(), 4).first;
  return std::make_pair(status_code, raw_values);
}

bool AtiNetCanOemInterface::LoadNewActiveCalibration(const uint8_t calibration)
{
  const bool can_set_calibration = SetActiveCalibration(calibration);
  if (can_set_calibration)
  {
    const Eigen::Matrix<double, 6, 6> raw_calibration_matrix
        = ReadActiveCalibrationMatrix();
    const std::pair<uint32_t, uint32_t> counts = ReadCountsPerUnit();
    const double counts_per_force = (double)counts.first;
    const double counts_per_torque = (double)counts.second;
    active_force_torque_inv_counts_vector_(0, 0) = 1.0 / counts_per_force;
    active_force_torque_inv_counts_vector_(1, 0) = 1.0 / counts_per_force;
    active_force_torque_inv_counts_vector_(2, 0) = 1.0 / counts_per_force;
    active_force_torque_inv_counts_vector_(3, 0) = 1.0 / counts_per_torque;
    active_force_torque_inv_counts_vector_(4, 0) = 1.0 / counts_per_torque;
    active_force_torque_inv_counts_vector_(5, 0) = 1.0 / counts_per_torque;
    active_calibration_matrix_ = raw_calibration_matrix;
    has_active_calibration_ = true;
    return true;
  }
  else
  {
    return false;
  }
}

Eigen::Matrix<double, 6, 6> AtiNetCanOemInterface::ReadActiveCalibrationMatrix()
{
  Eigen::Matrix<double, 6, 6> calibration_matrix
      = Eigen::Matrix<double, 6, 6>::Zero();
  calibration_matrix.block<1, 6>(0, 0) = ReadActiveCalibrationMatrixRow(0x00);
  calibration_matrix.block<1, 6>(1, 0) = ReadActiveCalibrationMatrixRow(0x01);
  calibration_matrix.block<1, 6>(2, 0) = ReadActiveCalibrationMatrixRow(0x02);
  calibration_matrix.block<1, 6>(3, 0) = ReadActiveCalibrationMatrixRow(0x03);
  calibration_matrix.block<1, 6>(4, 0) = ReadActiveCalibrationMatrixRow(0x04);
  calibration_matrix.block<1, 6>(5, 0) = ReadActiveCalibrationMatrixRow(0x05);
  return calibration_matrix;
}

Eigen::Matrix<double, 1, 6>
AtiNetCanOemInterface::ReadActiveCalibrationMatrixRow(const uint8_t row)
{
  Log("Reading active calibration matrix row...");
  if (row > 5)
  {
    throw std::invalid_argument("Row index must be <= 5");
  }
  const DataElement read_calibration_row(READ_MATRIX_ROW_A,
                                         std::vector<uint8_t>{row});
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(read_calibration_row, 0x03, 0.1);
  Eigen::Matrix<double, 1, 6> matrix_row = Eigen::Matrix<double, 1, 6>::Zero();
  if (response.size() != 3)
  {
    throw std::runtime_error("Failed to read calibration matrix row");
  }
  const DataElement& response_msg_1 = response.at(0);
  if (response_msg_1.Opcode() != READ_MATRIX_ROW_A)
  {
    throw std::runtime_error("Invalid response opcode (!= ROW_A)");
  }
  if (response_msg_1.Payload().size() != 8)
  {
    throw std::runtime_error("Invalid data for read calibration matrix row");
  }
  matrix_row(0, 0)
      = common_robotics_utilities::serialization::DeserializeNetworkMemcpyable<float>(
          response_msg_1.Payload(), 0).first;
  matrix_row(0, 1)
      = common_robotics_utilities::serialization::DeserializeNetworkMemcpyable<float>(
          response_msg_1.Payload(), 4).first;
  const DataElement& response_msg_2 = response.at(1);
  if (response_msg_2.Opcode() != READ_MATRIX_ROW_B)
  {
    throw std::runtime_error("Invalid response opcode (!= ROW_B)");
  }
  if (response_msg_2.Payload().size() != 8)
  {
    throw std::runtime_error("Invalid data for read calibration matrix row");
  }
  matrix_row(0, 2)
      = common_robotics_utilities::serialization::DeserializeNetworkMemcpyable<float>(
          response_msg_2.Payload(), 0).first;
  matrix_row(0, 3)
      = common_robotics_utilities::serialization::DeserializeNetworkMemcpyable<float>(
          response_msg_2.Payload(), 4).first;
  const DataElement& response_msg_3 = response.at(2);
  if (response_msg_3.Opcode() != READ_MATRIX_ROW_C)
  {
    throw std::runtime_error("Invalid response opcode (!= ROW_C)");
  }
  if (response_msg_3.Payload().size() != 8)
  {
    throw std::runtime_error("Invalid data for read calibration matrix row");
  }
  matrix_row(0, 4)
      = common_robotics_utilities::serialization::DeserializeNetworkMemcpyable<float>(
          response_msg_3.Payload(), 0).first;
  matrix_row(0, 5)
      = common_robotics_utilities::serialization::DeserializeNetworkMemcpyable<float>(
          response_msg_3.Payload(), 4).first;
  return matrix_row;
}

std::string AtiNetCanOemInterface::ReadSerialNumber()
{
  Log("Trying to read serial number...");
  const DataElement read_serial_number(READ_SERIAL_NUMBER);
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(read_serial_number, 0x01, 0.1);
  if (response.size() != 1)
  {
    throw std::runtime_error("Failed to read serial number in timeout");
  }
  Log("...got serial number response");
  const DataElement& response_msg = response[0];
  if (response_msg.Opcode() != READ_SERIAL_NUMBER)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  if (response_msg.Payload().size() != 8)
  {
    throw std::runtime_error("Invalid payload data for read serial number");
  }
  const std::string serial_number(reinterpret_cast<const char*>(
                                    response_msg.Payload().data()), 8);
  return serial_number;
}

bool
AtiNetCanOemInterface::SetActiveCalibration(const uint8_t calibration)
{
  Log("Setting active calibration...");
  if (calibration > 15)
  {
    throw std::invalid_argument("Desired calibration index must be <= 15");
  }
  const DataElement set_active_calibration(SET_ACTIVE_CALIBRATION,
                                           std::vector<uint8_t>{calibration});
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(set_active_calibration, 0x01, 0.1);
  if (response.size() != 1)
  {
    throw std::runtime_error("Failed to set active calibration in timeout");
  }
  const DataElement& response_msg = response[0];
  if (response_msg.Opcode() != SET_ACTIVE_CALIBRATION)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  if (response_msg.Payload().size() != 1)
  {
    throw std::runtime_error("Invalid payload data for set active calibration");
  }
  const uint8_t active_calibration = response_msg.Payload()[0];
  if (active_calibration == calibration)
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::pair<uint32_t, uint32_t> AtiNetCanOemInterface::ReadCountsPerUnit()
{
  Log("Reading counts per unit...");
  const DataElement read_counts_per_unit(READ_COUNTS_PER_UNIT);
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(read_counts_per_unit, 0x01, 0.1);
  if (response.size() != 1)
  {
    throw std::runtime_error("Failed to read counts per unit in timeout");
  }
  const DataElement& response_msg = response[0];
  if (response_msg.Opcode() != READ_COUNTS_PER_UNIT)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  if (response_msg.Payload().size() != 8)
  {
    throw std::runtime_error("Invalid payload data for read counts per unit");
  }
  const uint32_t force_counts
      = common_robotics_utilities::serialization
          ::DeserializeNetworkMemcpyable<uint32_t>(
              response_msg.Payload(), 0).first;
  const uint32_t torque_counts
      = common_robotics_utilities::serialization
          ::DeserializeNetworkMemcpyable<uint32_t>(
              response_msg.Payload(), 4).first;
  return std::make_pair(force_counts, torque_counts);
}

std::pair<uint8_t, uint8_t> AtiNetCanOemInterface::ReadUnitCodes()
{
  Log("Reading unit codes...");
  const DataElement read_unit_codes(READ_UNIT_CODES);
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(read_unit_codes, 0x01, 0.1);
  if (response.size() != 1)
  {
    throw std::runtime_error("Failed to read unit codes in timeout");
  }
  const DataElement& response_msg = response[0];
  if (response_msg.Opcode() != READ_UNIT_CODES)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  if (response_msg.Payload().size() != 2)
  {
    throw std::runtime_error("Invalid payload data for read unit codes");
  }
  const uint8_t force_unit = response_msg.Payload()[0];
  const uint8_t torque_unit = response_msg.Payload()[1];
  return std::make_pair(force_unit, torque_unit);
}

std::vector<uint16_t> AtiNetCanOemInterface::ReadDiagnosticADCVoltages()
{
  Log("Reading ADC voltages...");
  const std::vector<uint8_t> adc_indices = {0x00, 0x02, 0x03, 0x04, 0x05};
  std::vector<uint16_t> adc_voltages;
  for (const auto adc_index : adc_indices)
  {
    const DataElement read_adc(READ_ADC_VOLTAGES,
                               std::vector<uint8_t>{adc_index});
    const std::vector<DataElement> response
        = SendFrameAndAwaitResponse(read_adc, 0x01, 0.1);
    if (response.size() != 1)
    {
      throw std::runtime_error("Failed to read ADC voltages in timeout");
    }
    const DataElement& response_msg = response[0];
    if (response_msg.Opcode() != READ_ADC_VOLTAGES)
    {
      throw std::runtime_error("Invalid response opcode");
    }
    if (response_msg.Payload().size() != 2)
    {
      throw std::runtime_error("Invalid payload data for read ADC voltages");
    }
    const uint16_t adc_voltage
        = common_robotics_utilities::serialization
            ::DeserializeNetworkMemcpyable<uint16_t>(
                response_msg.Payload(), 0).first;
    adc_voltages.push_back(adc_voltage);
  }
  return adc_voltages;
}

void AtiNetCanOemInterface::ResetSensor()
{
  const DataElement reset(RESET);
  SendFrameAndAwaitResponse(reset, 0x0, 0.1);
}

bool AtiNetCanOemInterface::SetSensorBaseCanID(const uint8_t new_base_can_id)
{
  Log("Setting new base CAN ID...");
  if (new_base_can_id > 0x7f)
  {
    throw std::invalid_argument("New base CAN ID is > 0x7f");
  }
  const DataElement set_base_can_id(SET_BASE_ID_BITS,
                                    std::vector<uint8_t>{new_base_can_id});
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(set_base_can_id, 0x01, 0.1);
  if (response.size() != 1)
  {
    throw std::runtime_error("Failed to set base CAN ID in timeout");
  }
  const DataElement& response_msg = response[0];
  if (response_msg.Opcode() != SET_BASE_ID_BITS)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  return true;
}

bool AtiNetCanOemInterface::SetSensorCanRate(const uint8_t rate_divisor)
{
  Log("Setting new CAN rate...");
  const DataElement set_baud_rate(SET_BAUD_RATE,
                                  std::vector<uint8_t>{rate_divisor});
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(set_baud_rate, 0x01, 0.1);
  if (response.size() != 1)
  {
    throw std::runtime_error("Failed to set CAN baud rate in timeout");
  }
  const DataElement& response_msg = response[0];
  if (response_msg.Opcode() != SET_BAUD_RATE)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  return true;
}

std::pair<std::pair<uint8_t, uint8_t>, uint16_t>
AtiNetCanOemInterface::ReadFirmwareVersion()
{
  Log("Trying to read firmware version...");
  const DataElement read_firmware(READ_FIRMWARE_VERSION);
  const std::vector<DataElement> response
      = SendFrameAndAwaitResponse(read_firmware, 0x01, 0.1);
  if (response.size() != 1)
  {
    throw std::runtime_error("Failed to read firmware version in timeout");
  }
  Log("...got firmware version response");
  const DataElement& response_msg = response[0];
  if (response_msg.Opcode() != READ_FIRMWARE_VERSION)
  {
    throw std::runtime_error("Invalid response opcode");
  }
  if (response_msg.Payload().size() != 4)
  {
    throw std::runtime_error("Invalid payload data for read firmware version");
  }
  const uint8_t major_version = response_msg.Payload()[0];
  const uint8_t minor_version = response_msg.Payload()[1];
  const uint16_t build_number =
      common_robotics_utilities::serialization::DeserializeNetworkMemcpyable<uint16_t>(
        response_msg.Payload(), 2).first;
  return std::make_pair(std::make_pair(major_version, minor_version),
                        build_number);
}

std::vector<AtiNetCanOemInterface::DataElement>
AtiNetCanOemInterface::SendFrameAndAwaitResponse(
    const DataElement& command,
    const uint8_t num_response_frames,
    const double timeout)
{
  const std::chrono::duration<double> timeout_duration(timeout);
  // Assemble into CAN frame
  struct can_frame frame;
  frame.can_id =
      (uint32_t)(sensor_base_can_id_ << OPCODE_BITS) | command.Opcode();
  const std::vector<uint8_t>& payload = command.Payload();
  frame.can_dlc = (uint8_t)payload.size();
  memcpy(frame.data, payload.data(), payload.size());
  // Send the frame
  const ssize_t bytes_sent = write(can_socket_fd_, &frame, sizeof(frame));
  // Check that the frame was sent
  if (bytes_sent != sizeof(frame))
  {
    throw std::runtime_error("Failure to send CAN frame");
  }
  // Wait for the response
  if (num_response_frames > 0)
  {
    const auto start_time = std::chrono::steady_clock::now();
    std::vector<DataElement> response_frames;
    while (response_frames.size() < num_response_frames)
    {
      struct can_frame frame;
      const ssize_t read_size = read(can_socket_fd_, &frame, CAN_MTU);
      if (read_size == CAN_MTU)
      {
        if (frame.can_dlc <= CAN_MAX_DLEN)
        {
          const uint8_t opcode = (uint8_t)frame.can_id & 0xF;
          std::vector<uint8_t> payload;
          if (frame.can_dlc > 0)
          {
            payload.insert(payload.end(),
                           frame.data,
                           frame.data + frame.can_dlc);
          }
          response_frames.push_back(DataElement(opcode, payload));
        }
        else
        {
          throw std::runtime_error("Invalid frame.can_dlc size");
        }
      }
      else if (read_size < 0)
      {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        else
        {
          throw std::runtime_error("Error in recv");
        }
      }
      else
      {
        throw std::runtime_error("Read size != CAN_MTU");
      }
      const auto current_time = std::chrono::steady_clock::now();
      const auto elapsed_time = current_time - start_time;
      if (elapsed_time > timeout_duration)
      {
        break;
      }
    }
    return response_frames;
  }
  else
  {
    return std::vector<DataElement>();
  }
}

void AtiNetCanOemInterface::ShutdownConnection()
{
  Log("Closing socket...");
  close(can_socket_fd_);
  Log("...finished cleanup");
}

void AtiNetCanOemInterface::ParseStatusCode(const uint16_t status_code)
{
  if (status_code == 0)
  {
    return;
  }
  std::string error_message = "STOP OPERATION! Errors:";
  if ((status_code & WATCHDOG_RESET) > 0)
  {
    error_message += " +WATCHDOG_RESET";
  }
  if ((status_code & DAC_ADC_CHECK_TOO_HIGH) > 0)
  {
    error_message += " +DAC_ADC_CHECK_TOO_HIGH";
  }
  if ((status_code & DAC_ADC_CHECK_TOO_LOW) > 0)
  {
    error_message += " +DAC_ADC_CHECK_TOO_LOW";
  }
  if ((status_code & FAKE_GND_OUT_OF_RANGE) > 0)
  {
    error_message += " +FAKE_GND_OUT_OF_RANGE";
  }
  if ((status_code & SUPPLY_VOLTAGE_TOO_HIGH) > 0)
  {
    error_message += " +SUPPLY_VOLTAGE_TOO_HIGH";
  }
  if ((status_code & SUPPLY_VOLTAGE_TOO_LOW) > 0)
  {
    error_message += " +SUPPLY_VOLTAGE_TOO_LOW";
  }
  if ((status_code & BAD_ACTIVE_CALIBRATION) > 0)
  {
    error_message += " +BAD_ACTIVE_CALIBRATION";
  }
  if ((status_code & EEPROM_FAILURE) > 0)
  {
    error_message += " +EEPROM_FAILURE";
  }
  if ((status_code & INVALID_CONFIGURATION) > 0)
  {
    error_message += " +INVALID_CONFIGURATION";
  }
  if ((status_code & TEMP_TOO_HIGH) > 0)
  {
    error_message += " +TEMP_TOO_HIGH";
  }
  if ((status_code & TEMP_TOO_LOW) > 0)
  {
    error_message += " +TEMP_TOO_LOW";
  }
  if ((status_code & CAN_BUS_ERROR) > 0)
  {
    error_message += " +CAN_BUS_ERROR";
  }
  if ((status_code & ANY_ERROR) > 0)
  {
    error_message += " +ANY_ERROR";
  }
  throw std::runtime_error(error_message);
}
}
