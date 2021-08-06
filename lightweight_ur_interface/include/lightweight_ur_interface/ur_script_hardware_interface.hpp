#include <stdlib.h>
#include <vector>
#include <functional>
#include <common_robotics_utilities/serialization.hpp>

namespace lightweight_ur_interface
{
class ControlScriptCommand
{
public:

  enum CONTROL_MODE : int32_t { MODE_IDLE=0,
                                MODE_TEACH=1,
                                MODE_SPEEDJ=2,
                                MODE_SPEEDL=3,
                                MODE_WRENCH=4 };
  enum FORCE_MODE : int32_t { MODE_RIGID=0,
                              MODE_FORCE=1,
                              MODE_DONT_CHANGE=2,
                              MODE_KEEP_LIMITS=3 };

private:

  std::vector<double> speed_;
  std::vector<double> wrench_;
  std::vector<double> force_mode_limits_;
  std::vector<int32_t> force_mode_selection_vector_;
  CONTROL_MODE control_mode_;
  FORCE_MODE force_mode_;
  bool running_;

  template<typename T, typename Allocator>
  static uint64_t SerializeKnownSizeVector(
      const std::vector<T, Allocator>& vec_to_serialize,
      std::vector<uint8_t>& buffer,
      const std::function<uint64_t
      (const T&, std::vector<uint8_t>&)>& item_serializer)
  {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the contained items
    for (size_t idx = 0; idx < vec_to_serialize.size(); idx++)
    {
      const T& current = vec_to_serialize[idx];
      item_serializer(current, buffer);
    }
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static uint64_t SerializeKnownSizeInt32Vector(
      const std::vector<int32_t>& vec_to_serialize,
      std::vector<uint8_t>& buffer)
  {
    using common_robotics_utilities::serialization::SerializeNetworkMemcpyable;
    return SerializeKnownSizeVector<int32_t>(
          vec_to_serialize, buffer, SerializeNetworkMemcpyable<int32_t>);
  }

public:

  ControlScriptCommand()
  {
    speed_.resize(6, 0.0);
    wrench_.resize(6, 0.0);
    force_mode_limits_.resize(6, 0.0);
    force_mode_selection_vector_.resize(6, 0);
    control_mode_ = MODE_IDLE;
    force_mode_ = MODE_RIGID;
    running_ = true;
  }

  static uint64_t Serialize(const ControlScriptCommand& command,
                            const double float_conversion_ratio,
                            std::vector<uint8_t>& buffer)
  {
    return command.SerializeSelf(float_conversion_ratio, buffer);
  }

  static std::vector<int32_t> ConvertToInt32Vector(
      const std::vector<double>& vector, const double float_conversion_ratio)
  {
    std::vector<int32_t> int32_vector(vector.size());
    for (size_t idx = 0; idx < vector.size(); idx++)
    {
      int32_vector[idx]
          = static_cast<int32_t>(vector[idx] * float_conversion_ratio);
    }
    return int32_vector;
  }

  uint64_t SerializeSelf(const double float_conversion_ratio,
                         std::vector<uint8_t>& buffer) const
  {
    using common_robotics_utilities::serialization::SerializeNetworkMemcpyable;
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the contained items
    SerializeKnownSizeInt32Vector(
          ConvertToInt32Vector(speed_, float_conversion_ratio), buffer);
    SerializeKnownSizeInt32Vector(
          ConvertToInt32Vector(wrench_, float_conversion_ratio), buffer);
    SerializeKnownSizeInt32Vector(
          ConvertToInt32Vector(force_mode_limits_,
                               float_conversion_ratio), buffer);
    SerializeKnownSizeInt32Vector(force_mode_selection_vector_, buffer);
    SerializeNetworkMemcpyable<int32_t>(
        static_cast<int32_t>(control_mode_), buffer);
    SerializeNetworkMemcpyable<int32_t>(
        static_cast<int32_t>(force_mode_), buffer);
    if (running_)
    {
      SerializeNetworkMemcpyable<int32_t>(1, buffer);
    }
    else
    {
      SerializeNetworkMemcpyable<int32_t>(0, buffer);
    }
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static ControlScriptCommand MakeSpeedJCommand(
      const std::vector<double>& joint_velocities)
  {
    ControlScriptCommand command;
    if (joint_velocities.size() != 6)
    {
      throw std::runtime_error("joint_velocities.size() != 6");
    }
    command.speed_ = joint_velocities;
    command.control_mode_ = MODE_SPEEDJ;
    command.force_mode_ = MODE_DONT_CHANGE;
    return command;
  }

  static ControlScriptCommand MakeSpeedLCommand(
      const std::vector<double>& ee_velocities)
  {
    ControlScriptCommand command;
    if (ee_velocities.size() != 6)
    {
      throw std::runtime_error("ee_velocities.size() != 6");
    }
    command.speed_ = ee_velocities;
    command.control_mode_ = MODE_SPEEDL;
    command.force_mode_ = MODE_DONT_CHANGE;
    return command;
  }

  static ControlScriptCommand MakeTeachModeCommand()
  {
    ControlScriptCommand command;
    command.control_mode_ = MODE_TEACH;
    command.force_mode_ = MODE_RIGID;
    return command;
  }

  static ControlScriptCommand MakeExitTeachModeCommand()
  {
    ControlScriptCommand command;
    command.control_mode_ = MODE_IDLE;
    command.force_mode_ = MODE_RIGID;
    return command;
  }

  static ControlScriptCommand MakeForceModeCommand(
      const std::vector<double>& wrench,
      const std::vector<double>& force_mode_limits,
      const std::vector<int32_t>& force_mode_selection_vector)
  {
    ControlScriptCommand command;
    if (wrench.size() != 6)
    {
      throw std::runtime_error("wrench.size() != 6");
    }
    command.wrench_ = wrench;
    if (force_mode_limits.size() != 6)
    {
      throw std::runtime_error("force_mode_limits.size() != 6");
    }
    command.force_mode_limits_ = force_mode_limits;
    if (force_mode_selection_vector.size() != 6)
    {
      throw std::runtime_error("force_mode_selection_vector.size() != 6");
    }
    command.force_mode_selection_vector_ = force_mode_selection_vector;
    command.control_mode_ = MODE_WRENCH;
    command.force_mode_ = MODE_FORCE;
    return command;
  }

  static ControlScriptCommand MakeWrenchModeCommand(
      const std::vector<double>& ee_velocities,
      const std::vector<double>& wrench)
  {
    ControlScriptCommand command;
    if (ee_velocities.size() != 6)
    {
      throw std::runtime_error("ee_velocities.size() != 6");
    }
    if (wrench.size() != 6)
    {
      throw std::runtime_error("wrench.size() != 6");
    }
    command.speed_ = ee_velocities;
    command.wrench_ = wrench;
    command.control_mode_ = MODE_WRENCH;
    command.force_mode_ = MODE_KEEP_LIMITS;
    return command;
  }

  static ControlScriptCommand MakeIdleModeCommand()
  {
    ControlScriptCommand command;
    command.control_mode_ = MODE_IDLE;
    command.force_mode_ = MODE_RIGID;
    return command;
  }

  static ControlScriptCommand MakeExitForceModeCommand()
  {
    ControlScriptCommand command;
    command.control_mode_ = MODE_IDLE;
    command.force_mode_ = MODE_RIGID;
    return command;
  }

  static ControlScriptCommand MakeExitProgramCommand()
  {
    ControlScriptCommand command;
    command.running_ = false;
    return command;
  }
};
}
