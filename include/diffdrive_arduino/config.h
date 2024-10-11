#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30; // Hz
  std::string device = "/dev/ttyACM0";  // Arduino Mega 2560
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 1920;
  float wheels_pwm_factor = 1.0;
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H