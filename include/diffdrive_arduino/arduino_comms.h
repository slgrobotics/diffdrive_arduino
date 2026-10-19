#pragma once

#include "diffdrive_arduino/posix_serial.h"

class ArduinoComms
{

/*
Matching Arduino code can be found in the following sketches:
    https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/WheelsROS_Seggy/Comm.ino
    https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/DraggerROS/DraggerComm.ino
*/

public:

  ArduinoComms()
  {  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  { setup(serial_device, baud_rate, timeout_ms); }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  void sendEmptyMsg();
  
  bool readEncoderValues(int &val_1, int &val_2);
  bool readHealthValues(int &voltage_mv, int &current_ma, int &free_mem_bytes);
  bool readPingValues(int &front_right, int &front_left, int &back_right, int &back_left);

  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  PosixSerial serial_conn_;  ///< Underlying serial connection
};
