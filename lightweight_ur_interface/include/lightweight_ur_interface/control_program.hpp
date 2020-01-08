#pragma once

#include <string>
#include <sstream>

namespace lightweight_ur_driver
{
inline std::string MakeControlProgram(
    const std::string& our_ip_address, const int32_t control_port,
    const double float_conversion_ratio, const double stop_deceleration,
    const double speed_acceleration, const double speed_command_wait)
{
  // URScript is whitespace- and indentation-sensitive
  // Do not modify to match C++ style!
  std::ostringstream control_script_strm;
  control_script_strm << "def ur_driver_program():\n";
  control_script_strm << "    PC_IP_ADDRESS = \"" + our_ip_address + "\"\n";
  control_script_strm << "    PC_CONTROL_PORT = " + std::to_string(control_port) + "\n";
  control_script_strm << "    FLOAT_CONVERSION = " + std::to_string(float_conversion_ratio) + "\n";
  control_script_strm << "    STOP_DECELERATION = " + std::to_string(stop_deceleration) + "\n";
  control_script_strm << "    SPEED_ACCELERATION = " + std::to_string(speed_acceleration) + "\n";
  control_script_strm << "    SPEED_COMMAND_WAIT = " + std::to_string(speed_command_wait) + "\n";
  control_script_strm << "    MODE_IDLE = 0\n";
  control_script_strm << "    MODE_TEACH = 1\n";
  control_script_strm << "    MODE_SPEEDJ = 2\n";
  control_script_strm << "    MODE_SPEEDL = 3\n";
  control_script_strm << "    MODE_WRENCH = 4\n";
  control_script_strm << "    MODE_RIGID = 0\n";
  control_script_strm << "    MODE_FORCE = 1\n";
  control_script_strm << "    MODE_DONT_CHANGE = 2\n";
  control_script_strm << "    MODE_KEEP_LIMITS = 3\n";
  control_script_strm << "    command_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
  control_script_strm << "    command_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
  control_script_strm << "    command_force_mode_limits = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
  control_script_strm << "    command_force_mode_selection_vector = [0, 0, 0, 0, 0, 0]\n";
  control_script_strm << "    command_control_mode = MODE_IDLE\n";
  control_script_strm << "    command_force_mode = MODE_RIGID\n";
  control_script_strm << "    def update_command(new_speed, new_wrench, new_fml, new_fmsv, new_control_mode, new_force_mode):\n";
  control_script_strm << "        enter_critical\n";
  control_script_strm << "        command_speed = new_speed\n";
  control_script_strm << "        command_wrench = new_wrench\n";
  control_script_strm << "        command_force_mode_limits = new_fml\n";
  control_script_strm << "        command_force_mode_selection_vector = new_fmsv\n";
  control_script_strm << "        command_control_mode = new_control_mode\n";
  control_script_strm << "        command_force_mode = new_force_mode\n";
  control_script_strm << "        exit_critical\n";
  control_script_strm << "    end\n";
  control_script_strm << "    thread command_thread():\n";
  control_script_strm << "        active_force_mode_limits = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
  control_script_strm << "        active_force_mode_selection_vector = [0, 0, 0, 0, 0, 0]\n";
  control_script_strm << "        active_control_mode = MODE_IDLE\n";
  control_script_strm << "        active_force_mode = MODE_RIGID\n";
  control_script_strm << "        while True:\n";
  control_script_strm << "            enter_critical\n";
  control_script_strm << "            cmd_speed = command_speed\n";
  control_script_strm << "            cmd_wrench = command_wrench\n";
  control_script_strm << "            cmd_force_mode_limits = command_force_mode_limits\n";
  control_script_strm << "            cmd_force_mode_selection_vector = command_force_mode_selection_vector\n";
  control_script_strm << "            cmd_control_mode = command_control_mode\n";
  control_script_strm << "            cmd_force_mode = command_force_mode\n";
  control_script_strm << "            exit_critical\n";
  control_script_strm << "            if (active_control_mode == MODE_TEACH) and (cmd_control_mode != MODE_TEACH):\n";
  control_script_strm << "                active_control_mode = MODE_IDLE\n";
  control_script_strm << "                end_teach_mode()\n";
  control_script_strm << "            elif (active_control_mode != MODE_TEACH) and (cmd_control_mode == MODE_TEACH):\n";
  control_script_strm << "                active_control_mode = MODE_TEACH\n";
  control_script_strm << "                stopj(STOP_DECELERATION)\n";
  control_script_strm << "                teach_mode()\n";
  control_script_strm << "            elif (active_control_mode != MODE_IDLE) and (cmd_control_mode == MODE_IDLE):\n";
  control_script_strm << "                active_control_mode = MODE_IDLE\n";
  control_script_strm << "                stopj(STOP_DECELERATION)\n";
  control_script_strm << "            end\n";
  control_script_strm << "            if (active_force_mode != MODE_FORCE) and (cmd_force_mode == MODE_FORCE):\n";
  control_script_strm << "                active_force_mode = MODE_FORCE\n";
  control_script_strm << "                active_force_mode_limits = cmd_force_mode_limits\n";
  control_script_strm << "                active_force_mode_selection_vector = cmd_force_mode_selection_vector\n";
  control_script_strm << "                textmsg(\"MF:\", cmd_force_mode_selection_vector)\n";
  control_script_strm << "                force_mode(get_actual_tcp_pose(), cmd_force_mode_selection_vector, cmd_wrench, 2, cmd_force_mode_limits)\n";
  control_script_strm << "            elif (active_force_mode == MODE_FORCE) and (cmd_force_mode == MODE_KEEP_LIMITS):\n";
//  control_script_strm << "                end_force_mode()\n";
  control_script_strm << "                textmsg(\"MKL:\", active_force_mode_selection_vector)\n";
  control_script_strm << "                force_mode(get_actual_tcp_pose(), active_force_mode_selection_vector, cmd_wrench, 2, active_force_mode_limits)\n";
  control_script_strm << "            elif (active_force_mode != MODE_RIGID) and (cmd_force_mode == MODE_RIGID):\n";
  control_script_strm << "                active_force_mode = MODE_RIGID\n";
  control_script_strm << "                end_force_mode()\n";
  control_script_strm << "            end\n";
  control_script_strm << "            if cmd_control_mode == MODE_SPEEDJ:\n";
  control_script_strm << "                active_control_mode = MODE_SPEEDJ\n";
  control_script_strm << "                speedj(cmd_speed, SPEED_ACCELERATION, SPEED_COMMAND_WAIT)\n";
  control_script_strm << "            elif cmd_control_mode == MODE_SPEEDL:\n";
  control_script_strm << "                active_control_mode = MODE_SPEEDL\n";
  control_script_strm << "                speedl(cmd_speed, SPEED_ACCELERATION, SPEED_COMMAND_WAIT)\n";
//  control_script_strm << "            elif cmd_control_mode == MODE_WRENCH:\n";
//  control_script_strm << "                active_control_mode = MODE_WRENCH\n";
//  control_script_strm << "                speedl(cmd_speed, SPEED_ACCELERATION, SPEED_COMMAND_WAIT)\n";
  control_script_strm << "            else:\n";
  control_script_strm << "                sync()\n";
  control_script_strm << "            end\n";
  control_script_strm << "        end\n";
  control_script_strm << "    end\n";
  control_script_strm << "    socket_open(PC_IP_ADDRESS, PC_CONTROL_PORT)\n";
  control_script_strm << "    thread_command = run command_thread()\n";
  control_script_strm << "    recv_running = 1\n";
  control_script_strm << "    while recv_running == 1:\n";
  control_script_strm << "        params = socket_read_binary_integer(6 + 6 + 6 + 6 + 3)\n";
  control_script_strm << "        valid_data = params[0]\n";
  control_script_strm << "        if valid_data > 0:\n";
  control_script_strm << "            recv_speed = [params[1] / FLOAT_CONVERSION, params[2] / FLOAT_CONVERSION, params[3] / FLOAT_CONVERSION, params[4] / FLOAT_CONVERSION, params[5] / FLOAT_CONVERSION, params[6] / FLOAT_CONVERSION]\n";
  control_script_strm << "            recv_wrench = [params[7] / FLOAT_CONVERSION, params[8] / FLOAT_CONVERSION, params[9] / FLOAT_CONVERSION, params[10] / FLOAT_CONVERSION, params[11] / FLOAT_CONVERSION, params[12] / FLOAT_CONVERSION]\n";
  control_script_strm << "            recv_force_mode_limits = [params[13] / FLOAT_CONVERSION, params[14] / FLOAT_CONVERSION, params[15] / FLOAT_CONVERSION, params[16] / FLOAT_CONVERSION, params[17] / FLOAT_CONVERSION, params[18] / FLOAT_CONVERSION]\n";
  control_script_strm << "            recv_force_mode_selection_vector = [params[19], params[20], params[21], params[22], params[23], params[24]]\n";
  control_script_strm << "            recv_control_mode = params[25]\n";
  control_script_strm << "            recv_force_mode = params[26]\n";
  control_script_strm << "            recv_running = params[27]\n";
  control_script_strm << "            if recv_running == 1:\n";
  control_script_strm << "                update_command(recv_speed, recv_wrench, recv_force_mode_limits, recv_force_mode_selection_vector, recv_control_mode, recv_force_mode)\n";
  control_script_strm << "            else:\n";
  control_script_strm << "                update_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0, 0, 0, 0, 0, 0], MODE_IDLE, MODE_RIGID)\n";
  control_script_strm << "            end\n";
  control_script_strm << "        else:\n";
  control_script_strm << "            update_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0, 0, 0, 0, 0, 0], MODE_IDLE, MODE_DONT_CHANGE)\n";
  control_script_strm << "        end\n";
  control_script_strm << "    end\n";
  control_script_strm << "    sleep(0.1)\n";
  control_script_strm << "    socket_close()\n";
  control_script_strm << "    kill thread_command\n";
  control_script_strm << "end\n";
  return control_script_strm.str();
}
}
