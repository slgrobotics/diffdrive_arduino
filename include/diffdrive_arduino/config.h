#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>
#include "hardware_interface/hardware_info.hpp"

struct Config
{
  std::string left_wheel_name;
  std::string right_wheel_name;
  double loop_rate = 30.0;  // Arduino PID rate, not the controller_manager update rate.
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 10;  // Milliseconds per serial transaction.
  int startup_delay_ms = 2000;
  int enc_counts_per_rev = 1920;
  int max_encoder_delta = 1920;  // Maximum ticks per read, after rollover correction.
  int max_motor_command = 32767;  // AVR atoi() has a signed 16-bit range.
  int pid_p = 30;
  int pid_d = 20;
  int pid_i = 0;
  int pid_o = 100;
};

// Throws std::invalid_argument for invalid parameters or joint interfaces.
Config read_config(const hardware_interface::HardwareInfo & info, bool real_robot);

#endif
