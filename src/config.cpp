#include "diffdrive_arduino/config.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>

namespace
{
int integer(const hardware_interface::HardwareInfo & info, const std::string & key, int fallback)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) { return fallback; }
  int value = 0;
  const auto & text = it->second;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc() || result.ptr != text.data() + text.size()) {
    throw std::invalid_argument("Invalid integer parameter: " + key);
  }
  return value;
}

std::string required(const hardware_interface::HardwareInfo & info, const std::string & key)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end() || it->second.empty()) {
    throw std::invalid_argument("Missing parameter: " + key);
  }
  return it->second;
}

void check_joint(const hardware_interface::HardwareInfo & info, const std::string & name)
{
  const auto joint = std::find_if(info.joints.begin(), info.joints.end(),
    [&name](const auto & value) { return value.name == name; });
  if (joint == info.joints.end() || joint->command_interfaces.size() != 1 ||
    joint->command_interfaces[0].name != "velocity" ||
    joint->command_interfaces[0].data_type != "double" || joint->state_interfaces.size() != 2)
  {
    throw std::invalid_argument("Joint must have velocity command and position/velocity states: " + name);
  }
  bool position = false;
  bool velocity = false;
  for (const auto & interface : joint->state_interfaces) {
    if (interface.data_type != "double") {
      throw std::invalid_argument("Wheel interfaces must use double values: " + name);
    }
    position |= interface.name == "position";
    velocity |= interface.name == "velocity";
  }
  if (!position || !velocity) {
    throw std::invalid_argument("Missing position/velocity state interface: " + name);
  }
}
}  // namespace

Config read_config(const hardware_interface::HardwareInfo & info, bool real_robot)
{
  Config cfg;
  cfg.left_wheel_name = required(info, "left_wheel_name");
  cfg.right_wheel_name = required(info, "right_wheel_name");
  if (cfg.left_wheel_name == cfg.right_wheel_name || info.joints.size() != 2) {
    throw std::invalid_argument("Exactly two distinct wheel joints are required");
  }
  check_joint(info, cfg.left_wheel_name);
  check_joint(info, cfg.right_wheel_name);
  cfg.enc_counts_per_rev = integer(info, "enc_counts_per_rev", cfg.enc_counts_per_rev);
  if (cfg.enc_counts_per_rev <= 0) {
    throw std::invalid_argument("enc_counts_per_rev must be positive");
  }
  if (!real_robot) { return cfg; }

  const auto rate = info.hardware_parameters.find("loop_rate");
  if (rate != info.hardware_parameters.end()) {
    size_t consumed = 0;
    cfg.loop_rate = std::stod(rate->second, &consumed);
    if (consumed != rate->second.size()) { throw std::invalid_argument("Invalid loop_rate"); }
  }
  const auto device = info.hardware_parameters.find("device");
  if (device != info.hardware_parameters.end()) { cfg.device = device->second; }
  cfg.baud_rate = integer(info, "baud_rate", cfg.baud_rate);
  cfg.timeout = integer(info, "timeout", cfg.timeout);
  cfg.startup_delay_ms = integer(info, "startup_delay_ms", cfg.startup_delay_ms);
  cfg.max_encoder_delta = integer(info, "max_encoder_delta", cfg.enc_counts_per_rev);
  cfg.max_motor_command = integer(info, "max_motor_command", cfg.max_motor_command);
  cfg.pid_p = integer(info, "pid_p", cfg.pid_p);
  cfg.pid_d = integer(info, "pid_d", cfg.pid_d);
  cfg.pid_i = integer(info, "pid_i", cfg.pid_i);
  cfg.pid_o = integer(info, "pid_o", cfg.pid_o);
  if (!std::isfinite(cfg.loop_rate) || cfg.loop_rate <= 0 || cfg.device.empty() ||
    cfg.baud_rate <= 0 || cfg.timeout <= 0 || cfg.startup_delay_ms < 0 ||
    cfg.max_encoder_delta <= 0 || cfg.max_motor_command <= 0 || cfg.max_motor_command > 32767 ||
    cfg.pid_p < 0 || cfg.pid_d < 0 || cfg.pid_i < 0 || cfg.pid_o <= 0 ||
    cfg.pid_p > 32767 || cfg.pid_d > 32767 || cfg.pid_i > 32767 || cfg.pid_o > 32767)
  {
    throw std::invalid_argument("Invalid serial, timing, encoder, motor limit or PID parameter");
  }
  const auto pid = std::to_string(cfg.pid_p) + ":" + std::to_string(cfg.pid_d) + ":" +
    std::to_string(cfg.pid_i) + ":" + std::to_string(cfg.pid_o);
  if (pid.size() > 15) {
    throw std::invalid_argument("PID values exceed the firmware's 15-character argument buffer");
  }
  return cfg;
}
