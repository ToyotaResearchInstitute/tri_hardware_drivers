#pragma once

#include <robotiq_3_finger_gripper_driver/robotiq_3_finger_gripper_driver.hpp>

#if ROBOTIQ_3_FINGER_GRIPPER_DRIVER__SUPPORTED_ROS_VERSION == 2
#include <robotiq_3_finger_gripper_driver/msg/robotiq3_finger_command.hpp>
#include <robotiq_3_finger_gripper_driver/msg/robotiq3_finger_state.hpp>
#elif ROBOTIQ_3_FINGER_GRIPPER_DRIVER__SUPPORTED_ROS_VERSION == 1
#include <robotiq_3_finger_gripper_driver/Robotiq3FingerCommand.h>
#include <robotiq_3_finger_gripper_driver/Robotiq3FingerState.h>
#else
#error "Undefined or unknown ROBOTIQ_3_FINGER_GRIPPER_DRIVER__SUPPORTED_ROS_VERSION"
#endif

namespace robotiq_3_finger_gripper_driver
{
#if ROBOTIQ_3_FINGER_GRIPPER_DRIVER__SUPPORTED_ROS_VERSION == 2
using Robotiq3FingerCommand = msg::Robotiq3FingerCommand;
using Robotiq3FingerActuatorCommand = msg::Robotiq3FingerActuatorCommand;
using Robotiq3FingerState = msg::Robotiq3FingerState;
using Robotiq3FingerActuatorState = msg::Robotiq3FingerActuatorState;
#endif

inline Robotiq3FingerGripperActuatorCommand ConvertActuatorCommand(
    const Robotiq3FingerActuatorCommand& command)
{
  return Robotiq3FingerGripperActuatorCommand(command.percent_closed,
                                              command.percent_speed,
                                              command.percent_effort);
}

inline Robotiq3FingerGripperCommand ConvertFingerCommand(
    const Robotiq3FingerCommand& command)
{
  return Robotiq3FingerGripperCommand(
      ConvertActuatorCommand(command.finger_a_command),
      ConvertActuatorCommand(command.finger_b_command),
      ConvertActuatorCommand(command.finger_c_command),
      ConvertActuatorCommand(command.scissor_command));
}

inline Robotiq3FingerActuatorState ConvertActuatorStatus(
    const Robotiq3FingerGripperActuatorStatus& status)
{
  Robotiq3FingerActuatorState finger_state;
  finger_state.target_percent_closed = status.TargetPosition();
  finger_state.actual_percent_closed = status.ActualPosition();
  finger_state.actual_percent_current = status.ActualCurrent();
  finger_state.object_status = static_cast<uint8_t>(status.ObjectStatus());
  return finger_state;
}

inline Robotiq3FingerState ConvertFingerStatus(
    const Robotiq3FingerGripperStatus& status)
{
  Robotiq3FingerState state;
  state.finger_a_state = ConvertActuatorStatus(status.FingerAStatus());
  state.finger_b_state = ConvertActuatorStatus(status.FingerBStatus());
  state.finger_c_state = ConvertActuatorStatus(status.FingerCStatus());
  state.scissor_state = ConvertActuatorStatus(status.ScissorStatus());
  state.gripper_activation_state
      = static_cast<uint8_t>(status.GripperActivationStatus());
  state.gripper_mode_state
      = static_cast<uint8_t>(status.GripperModeStatus());
  state.gripper_action_state
      = static_cast<uint8_t>(status.GripperActionStatus());
  state.gripper_system_state
      = static_cast<uint8_t>(status.GripperSystemStatus());
  state.gripper_motion_state
      = static_cast<uint8_t>(status.GripperMotionStatus());
  state.gripper_fault_state
      = static_cast<uint8_t>(status.GripperFaultStatus());
  return state;
}
}
